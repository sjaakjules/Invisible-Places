#include "serialization/RenderSetupDocument.hpp"

#include "serialization/ProjectDocumentJson.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <system_error>
#include <utility>

#include <nlohmann/json.hpp>

namespace invisible_places::serialization {

namespace {

using nlohmann::json;
using invisible_places::renderer::pointcloud::PointCloudRendererMode;

constexpr std::string_view kRenderSetupSuffix = ".iprender.json";
constexpr std::uint32_t kSmoothVelocityRenderSetupSchemaVersion = 4U;
constexpr std::uint32_t kRelativePalettePhaseRenderSetupSchemaVersion = 5U;
static_assert(
    kRenderSetupDocumentSchemaVersion >=
    kSmoothVelocityRenderSetupSchemaVersion);
static_assert(
    kRenderSetupDocumentSchemaVersion >=
    kRelativePalettePhaseRenderSetupSchemaVersion);

void MigrateLegacySmoothPalettePhaseKeys(
    invisible_places::timing::TimingTakeSceneState* state) {
    if (state == nullptr) {
        return;
    }
    using invisible_places::timing::TimingColouriseEffectParameter;
    using invisible_places::water::WaterScenarioInterpolation;
    for (auto& effect : state->colouriseEffects) {
        for (auto& key : effect.effectParameterKeys) {
            if (key.parameter ==
                    TimingColouriseEffectParameter::PalettePhase &&
                key.interpolation ==
                    WaterScenarioInterpolation::Smooth) {
                key.interpolation =
                    WaterScenarioInterpolation::SmoothVelocity;
            }
        }
    }
}

void SetError(std::string* errorMessage, std::string message) {
    if (errorMessage != nullptr) {
        *errorMessage = std::move(message);
    }
}

bool EndsWith(std::string_view text, std::string_view suffix) {
    return text.size() >= suffix.size() &&
           text.substr(text.size() - suffix.size()) == suffix;
}

std::string RendererModeName(PointCloudRendererMode mode) {
    switch (mode) {
        case PointCloudRendererMode::Beauty:
            return "beauty";
        case PointCloudRendererMode::FastBasic:
            return "fast_basic";
    }
    return "beauty";
}

PointCloudRendererMode ParseRendererMode(const json& value) {
    const auto name = value.is_string()
                          ? value.get<std::string>()
                          : std::string{"beauty"};
    if (name == "fast_basic" || name == "fast") {
        return PointCloudRendererMode::FastBasic;
    }
    return PointCloudRendererMode::Beauty;
}

json RendererStateToJson(const RenderSetupRendererState& state) {
    return {
        {"point_cloud_renderer_mode",
         RendererModeName(state.pointCloudRendererMode)},
        {"background_color", state.backgroundColor},
        {"eye_dome_lighting_enabled", state.eyeDomeLightingEnabled},
        {"eye_dome_lighting_thickness", state.eyeDomeLightingThickness},
        {"pro_res_alpha_preview_enabled", state.proResAlphaPreviewEnabled},
        {"gaussian_splat_footprint_boost",
         state.gaussianSplatFootprintBoost},
        {"density_policy", state.densityPolicy},
    };
}

RenderSetupRendererState ParseRendererState(const json& value) {
    RenderSetupRendererState state;
    if (value.contains("point_cloud_renderer_mode")) {
        state.pointCloudRendererMode =
            ParseRendererMode(value.at("point_cloud_renderer_mode"));
    }
    if (value.contains("background_color")) {
        state.backgroundColor =
            value.at("background_color").get<std::array<float, 4>>();
    }
    state.eyeDomeLightingEnabled = value.value(
        "eye_dome_lighting_enabled",
        state.eyeDomeLightingEnabled);
    state.eyeDomeLightingThickness = value.value(
        "eye_dome_lighting_thickness",
        state.eyeDomeLightingThickness);
    state.proResAlphaPreviewEnabled = value.value(
        "pro_res_alpha_preview_enabled",
        state.proResAlphaPreviewEnabled);
    state.gaussianSplatFootprintBoost = value.value(
        "gaussian_splat_footprint_boost",
        state.gaussianSplatFootprintBoost);
    state.densityPolicy = value.value(
        "density_policy",
        state.densityPolicy);
    return state;
}

json SummaryToJson(const RenderSetupSummary& summary) {
    return {
        {"water_run_count", summary.waterRunCount},
        {"active_water_track_count", summary.activeWaterTrackCount},
        {"water_key_count", summary.waterKeyCount},
        {"enabled_colourise_effect_count",
         summary.enabledColouriseEffectCount},
        {"colourise_key_count", summary.colouriseKeyCount},
    };
}

RenderSetupSummary ParseSummary(const json& value) {
    RenderSetupSummary summary;
    summary.waterRunCount = value.value(
        "water_run_count",
        summary.waterRunCount);
    summary.activeWaterTrackCount = value.value(
        "active_water_track_count",
        summary.activeWaterTrackCount);
    summary.waterKeyCount = value.value(
        "water_key_count",
        summary.waterKeyCount);
    summary.enabledColouriseEffectCount = value.value(
        "enabled_colourise_effect_count",
        summary.enabledColouriseEffectCount);
    summary.colouriseKeyCount = value.value(
        "colourise_key_count",
        summary.colouriseKeyCount);
    return summary;
}

json FingerprintToJson(const RenderSetupSourceFingerprint& fingerprint) {
    return {
        {"scene_role", fingerprint.sceneRole},
        {"source_path", fingerprint.sourcePath.generic_string()},
        {"file_size", fingerprint.fileSize},
        {"modification_time_ticks", fingerprint.modificationTimeTicks},
    };
}

RenderSetupSourceFingerprint ParseFingerprint(const json& value) {
    RenderSetupSourceFingerprint fingerprint;
    fingerprint.sceneRole = value.value("scene_role", std::string{});
    fingerprint.sourcePath = value.value("source_path", std::string{});
    fingerprint.fileSize = value.value("file_size", fingerprint.fileSize);
    fingerprint.modificationTimeTicks = value.value(
        "modification_time_ticks",
        fingerprint.modificationTimeTicks);
    return fingerprint;
}

json HistoryEntryToJson(const RenderSetupHistoryEntry& entry) {
    return {
        {"setup_path", entry.setupPath.generic_string()},
        {"output_path", entry.outputPath.generic_string()},
        {"status", RenderSetupStatusName(entry.status)},
        {"created_utc", entry.createdUtc},
        {"completed_utc", entry.completedUtc},
        {"animation_name", entry.animationName},
        {"scene_group", entry.sceneGroupName},
        {"timing_take_name", entry.timingTakeName},
        {"edited_setting_count", entry.editedSettingCount},
    };
}

std::optional<RenderSetupHistoryEntry> ParseHistoryEntry(
    const json& value,
    std::string* errorMessage) {
    if (!value.is_object()) {
        SetError(errorMessage, "Render setup history entry is not an object.");
        return std::nullopt;
    }
    const auto status = ParseRenderSetupStatus(
        value.value("status", std::string{"rendering"}));
    if (!status.has_value()) {
        SetError(errorMessage, "Render setup history entry has an invalid status.");
        return std::nullopt;
    }
    RenderSetupHistoryEntry entry;
    entry.setupPath = value.value("setup_path", std::string{});
    entry.outputPath = value.value("output_path", std::string{});
    entry.status = status.value();
    entry.createdUtc = value.value("created_utc", std::string{});
    entry.completedUtc = value.value("completed_utc", std::string{});
    entry.animationName = value.value("animation_name", std::string{});
    entry.sceneGroupName = value.value("scene_group", std::string{});
    entry.timingTakeName = value.value("timing_take_name", std::string{});
    entry.editedSettingCount = value.value(
        "edited_setting_count",
        entry.editedSettingCount);
    return entry;
}

bool HistoryEntryNewer(
    const RenderSetupHistoryEntry& left,
    const RenderSetupHistoryEntry& right) {
    if (left.createdUtc != right.createdUtc) {
        return left.createdUtc > right.createdUtc;
    }
    return left.setupPath.generic_string() > right.setupPath.generic_string();
}

bool WriteJsonAtomically(
    const json& value,
    const std::filesystem::path& outputPath,
    std::string* errorMessage) {
    if (outputPath.empty()) {
        SetError(errorMessage, "Render setup output path is empty.");
        return false;
    }
    if (const auto parent = outputPath.parent_path(); !parent.empty()) {
        std::error_code createError;
        std::filesystem::create_directories(parent, createError);
        if (createError) {
            SetError(
                errorMessage,
                "Failed to create render setup directory: " +
                    createError.message());
            return false;
        }
    }

    auto temporaryPath = outputPath;
    temporaryPath += ".tmp";
    std::ofstream output{temporaryPath, std::ios::binary | std::ios::trunc};
    if (!output.is_open()) {
        SetError(errorMessage, "Failed to open render setup temporary file.");
        return false;
    }
    output << value.dump(2);
    output.flush();
    bool wrote = output.good();
    output.close();
    wrote = wrote && !output.fail();
    if (!wrote) {
        std::error_code ignored;
        std::filesystem::remove(temporaryPath, ignored);
        SetError(errorMessage, "Failed to write complete render setup file.");
        return false;
    }

    std::error_code renameError;
    std::filesystem::rename(temporaryPath, outputPath, renameError);
    if (renameError) {
        std::error_code ignored;
        std::filesystem::remove(temporaryPath, ignored);
        SetError(
            errorMessage,
            "Failed to atomically replace render setup file: " +
                renameError.message());
        return false;
    }
    return true;
}

std::optional<json> ReadJson(
    const std::filesystem::path& inputPath,
    std::string* errorMessage) {
    std::ifstream input{inputPath, std::ios::binary};
    if (!input.is_open()) {
        SetError(errorMessage, "Failed to open render setup file.");
        return std::nullopt;
    }
    try {
        return json::parse(input);
    } catch (const std::exception& error) {
        SetError(
            errorMessage,
            "Failed to parse render setup JSON: " + std::string{error.what()});
        return std::nullopt;
    }
}

json RenderSetupToJson(const RenderSetupDocument& document) {
    const auto createdUtc = document.createdUtc.empty()
                                ? CurrentUtcTimestamp()
                                : document.createdUtc;
    json authoredWaterJson;
    if (!document.authoredWater.pathCache.has_value() &&
        document.authoredWater.rippleRuntimeCaches.empty()) {
        authoredWaterJson = WaterSourcesDocumentToJson(document.authoredWater);
    } else {
        auto cacheFreeWater = document.authoredWater;
        cacheFreeWater.pathCache.reset();
        cacheFreeWater.rippleRuntimeCaches.clear();
        authoredWaterJson = WaterSourcesDocumentToJson(cacheFreeWater);
    }

    json fingerprints = json::array();
    for (const auto& fingerprint : document.sourceFingerprints) {
        fingerprints.push_back(FingerprintToJson(fingerprint));
    }
    json animationTrailProfiles = json::array();
    for (const auto& profile : document.waterAnimationTrailProfiles) {
        animationTrailProfiles.push_back(
            WaterAnimationTrailProfileToJson(profile));
    }
    json waterPointVisuals = json::array();
    for (const auto& visual : document.waterPointVisuals) {
        waterPointVisuals.push_back(PointCloudVisualToJson(visual));
    }
    json snapshot{
        {"animation", AnimationPathToJson(document.animation)},
        {"export_preset", ExportPresetToJson(document.exportPreset)},
        {"live_point_visual",
         PointCloudStyleToJson(document.livePointVisual)},
        {"timing_take_scene_state",
         TimingTakeSceneStateToJson(document.timingState)},
        {"authored_water", std::move(authoredWaterJson)},
        {"water_animation_trail_settings",
         WaterAnimationTrailSettingsToJson(
             document.waterAnimationTrailSettings)},
        {"water_animation_trail_profiles",
         std::move(animationTrailProfiles)},
        {"selected_water_animation_trail_profile",
         document.selectedWaterAnimationTrailProfileName},
        {"water_point_visuals", std::move(waterPointVisuals)},
        {"selected_water_point_visual",
         document.selectedWaterPointVisualName},
        {"water_point_visual_style",
         PointCloudStyleToJson(document.waterPointVisualStyle)},
        {"renderer", RendererStateToJson(document.renderer)},
    };
    if (document.tempWaterAnimationTrailSettings.has_value()) {
        snapshot["temp_water_animation_trail_settings"] =
            WaterAnimationTrailSettingsToJson(
                document.tempWaterAnimationTrailSettings.value());
    }
    if (document.tempWaterPointVisualStyle.has_value()) {
        snapshot["temp_water_point_visual_style"] =
            PointCloudStyleToJson(
                document.tempWaterPointVisualStyle.value());
    }
    return {
        {"schema_version", kRenderSetupDocumentSchemaVersion},
        {"status", RenderSetupStatusName(document.status)},
        {"created_utc", createdUtc},
        {"completed_utc", document.completedUtc},
        {"failure_message", document.failureMessage},
        {"metadata",
         {
             {"output_path", document.outputPath.generic_string()},
             {"log_path", document.logPath.generic_string()},
             {"source_project_path",
              document.sourceProjectPath.generic_string()},
             {"source_project_identity", document.sourceProjectIdentity},
             {"original_animation_path",
              document.originalAnimationPath.generic_string()},
             {"scene_group", document.sceneGroupName},
             {"timing_take_id",
              invisible_places::timing::NormalizeTimingTakeId(
                  document.timingTakeId)},
             {"timing_take_name", document.timingTakeName},
             {"visual_name", document.visualName},
             {"animation_modified", document.animationModified},
             {"summary", SummaryToJson(document.summary)},
             {"edited_settings", document.editedSettingLabels},
             {"source_fingerprints", std::move(fingerprints)},
         }},
        {"snapshot", std::move(snapshot)},
    };
}

std::optional<RenderSetupHistoryEntry> HistoryEntryFromRoot(
    const json& root,
    const std::filesystem::path& setupPath,
    std::string* errorMessage) {
    if (!root.is_object()) {
        SetError(errorMessage, "Render setup document root is not an object.");
        return std::nullopt;
    }
    const auto schema = root.value("schema_version", 0U);
    if (schema < kMinimumRenderSetupDocumentSchemaVersion ||
        schema > kRenderSetupDocumentSchemaVersion) {
        SetError(
            errorMessage,
            schema > kRenderSetupDocumentSchemaVersion
                ? "Render setup uses a newer unsupported schema version."
                : "Render setup schema version is missing or unsupported.");
        return std::nullopt;
    }
    if (!root.contains("status") || !root.at("status").is_string()) {
        SetError(errorMessage, "Render setup status is missing or invalid.");
        return std::nullopt;
    }
    const auto status = ParseRenderSetupStatus(
        root.at("status").get<std::string>());
    if (!status.has_value()) {
        SetError(errorMessage, "Render setup has an invalid status.");
        return std::nullopt;
    }
    const auto metadata = root.value("metadata", json::object());
    RenderSetupHistoryEntry entry;
    entry.setupPath = setupPath;
    entry.outputPath = metadata.value("output_path", std::string{});
    entry.status = status.value();
    entry.createdUtc = root.value("created_utc", std::string{});
    entry.completedUtc = root.value("completed_utc", std::string{});
    const auto snapshot = root.value("snapshot", json::object());
    const auto animation = snapshot.value("animation", json::object());
    entry.animationName = animation.value("name", std::string{});
    entry.sceneGroupName = metadata.value("scene_group", std::string{});
    entry.timingTakeName = metadata.value(
        "timing_take_name",
        std::string{});
    const auto edited = metadata.value("edited_settings", json::array());
    entry.editedSettingCount = edited.is_array() ? edited.size() : 0U;
    return entry;
}

}  // namespace

std::string_view RenderSetupStatusName(RenderSetupStatus status) {
    switch (status) {
        case RenderSetupStatus::Rendering:
            return "rendering";
        case RenderSetupStatus::Completed:
            return "completed";
        case RenderSetupStatus::Failed:
            return "failed";
        case RenderSetupStatus::Cancelled:
            return "cancelled";
    }
    return "rendering";
}

std::optional<RenderSetupStatus> ParseRenderSetupStatus(
    std::string_view name) {
    if (name == "rendering") {
        return RenderSetupStatus::Rendering;
    }
    if (name == "completed") {
        return RenderSetupStatus::Completed;
    }
    if (name == "failed") {
        return RenderSetupStatus::Failed;
    }
    if (name == "cancelled" || name == "canceled") {
        return RenderSetupStatus::Cancelled;
    }
    return std::nullopt;
}

std::string CurrentUtcTimestamp() {
    const auto now = std::chrono::system_clock::now();
    const auto time = std::chrono::system_clock::to_time_t(now);
    std::tm utc{};
#if defined(_WIN32)
    gmtime_s(&utc, &time);
#else
    gmtime_r(&time, &utc);
#endif
    std::ostringstream stream;
    stream << std::put_time(&utc, "%Y-%m-%dT%H:%M:%SZ");
    return stream.str();
}

RenderSetupSummary SummarizeRenderSetupTiming(
    const invisible_places::timing::TimingTakeSceneState& state) {
    RenderSetupSummary summary;
    summary.waterRunCount = state.waterFeatureTimingRuns.size();
    for (const auto& run : state.waterFeatureTimingRuns) {
        for (const auto& feature : run.features) {
            for (const auto& track : feature.settings) {
                if (track.active) {
                    ++summary.activeWaterTrackCount;
                    summary.waterKeyCount += track.keys.size();
                }
            }
        }
    }
    for (const auto& effect : state.colouriseEffects) {
        if (!effect.enabled) {
            continue;
        }
        ++summary.enabledColouriseEffectCount;
        if (effect.colouriseEnabled) {
            summary.colouriseKeyCount += effect.paletteKeys.size();
            summary.colouriseKeyCount +=
                effect.paletteStopParameterKeys.size();
        }
        summary.colouriseKeyCount += static_cast<std::size_t>(
            std::count_if(
                effect.effectParameterKeys.begin(),
                effect.effectParameterKeys.end(),
                [&](const invisible_places::timing::
                        TimingColouriseEffectParameterKey& key) {
                    return invisible_places::timing::
                        TimingEffectParameterIsSupported(
                            effect,
                            key.parameter);
                }));
        summary.colouriseKeyCount += effect.boundsParameterKeys.size();
        summary.colouriseKeyCount += effect.boundsKeys.size();
    }
    return summary;
}

std::filesystem::path AllocateRenderSetupSidecarPath(
    const std::filesystem::path& outputPath,
    std::string_view createdUtc) {
    auto timestamp = std::string{
        createdUtc.empty() ? CurrentUtcTimestamp() : createdUtc};
    timestamp.erase(
        std::remove_if(
            timestamp.begin(),
            timestamp.end(),
            [](char character) {
                return std::isalnum(
                           static_cast<unsigned char>(character)) == 0;
            }),
        timestamp.end());
    if (timestamp.empty()) {
        timestamp = "render";
    }

    const auto parent = outputPath.parent_path();
    const auto stem = outputPath.stem().string().empty()
                          ? std::string{"render"}
                          : outputPath.stem().string();
    auto candidate = parent /
                     (stem + "_" + timestamp +
                      std::string{kRenderSetupSuffix});
    for (std::size_t suffix = 2U; std::filesystem::exists(candidate); ++suffix) {
        std::ostringstream name;
        name << stem << '_' << timestamp << '_' << std::setw(2)
             << std::setfill('0') << suffix << kRenderSetupSuffix;
        candidate = parent / name.str();
    }
    return candidate;
}

bool SaveRenderSetupDocument(
    const RenderSetupDocument& document,
    const std::filesystem::path& outputPath,
    std::string* errorMessage) {
    try {
        return WriteJsonAtomically(
            RenderSetupToJson(document),
            outputPath,
            errorMessage);
    } catch (const std::exception& error) {
        SetError(
            errorMessage,
            "Failed to serialize render setup: " + std::string{error.what()});
        return false;
    }
}

std::optional<RenderSetupDocument> LoadRenderSetupDocument(
    const std::filesystem::path& inputPath,
    std::string* errorMessage) {
    const auto root = ReadJson(inputPath, errorMessage);
    if (!root.has_value()) {
        return std::nullopt;
    }
    if (!root->is_object()) {
        SetError(errorMessage, "Render setup document root is not an object.");
        return std::nullopt;
    }
    const auto schema = root->value("schema_version", 0U);
    if (schema < kMinimumRenderSetupDocumentSchemaVersion ||
        schema > kRenderSetupDocumentSchemaVersion) {
        SetError(
            errorMessage,
            schema > kRenderSetupDocumentSchemaVersion
                ? "Render setup uses a newer unsupported schema version."
                : "Render setup schema version is missing or unsupported.");
        return std::nullopt;
    }
    if (!root->contains("status") || !root->at("status").is_string()) {
        SetError(errorMessage, "Render setup status is missing or invalid.");
        return std::nullopt;
    }
    const auto status = ParseRenderSetupStatus(
        root->at("status").get<std::string>());
    if (!status.has_value()) {
        SetError(errorMessage, "Render setup has an invalid status.");
        return std::nullopt;
    }
    if (!root->contains("metadata") ||
        !root->at("metadata").is_object() ||
        !root->contains("snapshot") ||
        !root->at("snapshot").is_object()) {
        SetError(errorMessage, "Render setup metadata or snapshot is missing.");
        return std::nullopt;
    }

    try {
        const auto& metadata = root->at("metadata");
        const auto& snapshot = root->at("snapshot");
        std::string nestedError;
        if (!snapshot.contains("animation") ||
            !snapshot.contains("export_preset") ||
            !snapshot.contains("live_point_visual") ||
            !snapshot.contains("timing_take_scene_state") ||
            !snapshot.contains("authored_water") ||
            !snapshot.contains("water_animation_trail_settings") ||
            !snapshot.contains("water_animation_trail_profiles") ||
            !snapshot.contains("water_point_visuals") ||
            !snapshot.contains("water_point_visual_style") ||
            !snapshot.contains("renderer")) {
            SetError(errorMessage, "Render setup snapshot is incomplete.");
            return std::nullopt;
        }
        auto animation = AnimationPathFromJson(
            snapshot.at("animation"),
            &nestedError);
        if (!animation.has_value()) {
            SetError(errorMessage, nestedError);
            return std::nullopt;
        }
        auto preset = ExportPresetFromJson(
            snapshot.at("export_preset"),
            &nestedError);
        if (!preset.has_value()) {
            SetError(errorMessage, nestedError);
            return std::nullopt;
        }
        auto visual = PointCloudStyleFromJson(
            snapshot.at("live_point_visual"),
            &nestedError);
        if (!visual.has_value()) {
            SetError(errorMessage, nestedError);
            return std::nullopt;
        }
        auto timingStateJson =
            snapshot.at("timing_take_scene_state");
        if (schema < kRelativePalettePhaseRenderSetupSchemaVersion) {
            MigrateAbsoluteTimingColourisePalettePhaseKeys(
                &timingStateJson);
        }
        auto timingState = TimingTakeSceneStateFromJson(
            timingStateJson,
            &nestedError);
        if (!timingState.has_value()) {
            SetError(errorMessage, nestedError);
            return std::nullopt;
        }
        auto authoredWater = WaterSourcesDocumentFromJson(
            snapshot.at("authored_water"),
            &nestedError);
        if (!authoredWater.has_value()) {
            SetError(errorMessage, nestedError);
            return std::nullopt;
        }
        authoredWater->pathCache.reset();
        authoredWater->rippleRuntimeCaches.clear();
        auto animationTrailSettings = WaterAnimationTrailSettingsFromJson(
            snapshot.at("water_animation_trail_settings"),
            &nestedError);
        if (!animationTrailSettings.has_value()) {
            SetError(errorMessage, nestedError);
            return std::nullopt;
        }
        auto waterPointVisualStyle = PointCloudStyleFromJson(
            snapshot.at("water_point_visual_style"),
            &nestedError);
        if (!waterPointVisualStyle.has_value()) {
            SetError(errorMessage, nestedError);
            return std::nullopt;
        }

        RenderSetupDocument document;
        document.schemaVersion = schema;
        document.status = status.value();
        document.createdUtc = root->value("created_utc", std::string{});
        document.completedUtc = root->value("completed_utc", std::string{});
        document.failureMessage = root->value(
            "failure_message",
            std::string{});
        document.outputPath = metadata.value("output_path", std::string{});
        document.logPath = metadata.value("log_path", std::string{});
        document.sourceProjectPath = metadata.value(
            "source_project_path",
            std::string{});
        document.sourceProjectIdentity = metadata.value(
            "source_project_identity",
            std::string{});
        document.originalAnimationPath = metadata.value(
            "original_animation_path",
            std::string{});
        document.sceneGroupName = metadata.value(
            "scene_group",
            std::string{});
        document.timingTakeId = invisible_places::timing::NormalizeTimingTakeId(
            metadata.value("timing_take_id", timingState->takeId));
        document.timingTakeName = metadata.value(
            "timing_take_name",
            std::string{invisible_places::timing::kAuthoredTimingTakeName});
        document.visualName = metadata.value(
            "visual_name",
            std::string{"Unnamed"});
        document.animationModified = metadata.value(
            "animation_modified",
            false);
        if (metadata.contains("summary") &&
            metadata.at("summary").is_object()) {
            document.summary = ParseSummary(metadata.at("summary"));
        }
        if (metadata.contains("edited_settings") &&
            metadata.at("edited_settings").is_array()) {
            document.editedSettingLabels =
                metadata.at("edited_settings").get<std::vector<std::string>>();
        }
        if (metadata.contains("source_fingerprints") &&
            metadata.at("source_fingerprints").is_array()) {
            for (const auto& value : metadata.at("source_fingerprints")) {
                document.sourceFingerprints.push_back(ParseFingerprint(value));
            }
        }
        document.animation = std::move(animation.value());
        document.exportPreset = std::move(preset.value());
        document.livePointVisual = std::move(visual.value());
        document.timingState = std::move(timingState.value());
        if (schema < kSmoothVelocityRenderSetupSchemaVersion) {
            MigrateLegacySmoothPalettePhaseKeys(
                &document.timingState);
        }
        document.authoredWater = std::move(authoredWater.value());
        document.waterAnimationTrailSettings =
            std::move(animationTrailSettings.value());
        document.selectedWaterAnimationTrailProfileName = snapshot.value(
            "selected_water_animation_trail_profile",
            document.selectedWaterAnimationTrailProfileName);
        if (!snapshot.at("water_animation_trail_profiles").is_array() ||
            !snapshot.at("water_point_visuals").is_array()) {
            SetError(
                errorMessage,
                "Render setup water profile or visual list is invalid.");
            return std::nullopt;
        }
        for (const auto& value :
             snapshot.at("water_animation_trail_profiles")) {
            auto profile = WaterAnimationTrailProfileFromJson(
                value,
                &nestedError);
            if (!profile.has_value()) {
                SetError(errorMessage, nestedError);
                return std::nullopt;
            }
            document.waterAnimationTrailProfiles.push_back(
                std::move(profile.value()));
        }
        for (const auto& value : snapshot.at("water_point_visuals")) {
            auto visual = PointCloudVisualFromJson(value, &nestedError);
            if (!visual.has_value()) {
                SetError(errorMessage, nestedError);
                return std::nullopt;
            }
            document.waterPointVisuals.push_back(
                std::move(visual.value()));
        }
        document.selectedWaterPointVisualName = snapshot.value(
            "selected_water_point_visual",
            document.selectedWaterPointVisualName);
        document.waterPointVisualStyle =
            std::move(waterPointVisualStyle.value());
        if (snapshot.contains("temp_water_animation_trail_settings")) {
            auto settings = WaterAnimationTrailSettingsFromJson(
                snapshot.at("temp_water_animation_trail_settings"),
                &nestedError);
            if (!settings.has_value()) {
                SetError(errorMessage, nestedError);
                return std::nullopt;
            }
            document.tempWaterAnimationTrailSettings =
                std::move(settings.value());
        }
        if (snapshot.contains("temp_water_point_visual_style")) {
            auto style = PointCloudStyleFromJson(
                snapshot.at("temp_water_point_visual_style"),
                &nestedError);
            if (!style.has_value()) {
                SetError(errorMessage, nestedError);
                return std::nullopt;
            }
            document.tempWaterPointVisualStyle =
                std::move(style.value());
        }
        document.renderer = ParseRendererState(snapshot.at("renderer"));
        return document;
    } catch (const std::exception& error) {
        SetError(
            errorMessage,
            "Failed to parse render setup: " + std::string{error.what()});
        return std::nullopt;
    }
}

bool UpdateRenderSetupDocumentStatus(
    const std::filesystem::path& inputPath,
    RenderSetupStatus status,
    std::string_view completedUtc,
    std::string_view failureMessage,
    std::string* errorMessage) {
    auto document = LoadRenderSetupDocument(inputPath, errorMessage);
    if (!document.has_value()) {
        return false;
    }
    document->status = status;
    document->failureMessage = std::string{failureMessage};
    if (status == RenderSetupStatus::Rendering) {
        document->completedUtc.clear();
    } else {
        document->completedUtc = completedUtc.empty()
                                     ? CurrentUtcTimestamp()
                                     : std::string{completedUtc};
    }
    return SaveRenderSetupDocument(document.value(), inputPath, errorMessage);
}

std::optional<RenderSetupHistoryEntry> ReadRenderSetupHistoryEntry(
    const std::filesystem::path& setupPath,
    std::string* errorMessage) {
    const auto root = ReadJson(setupPath, errorMessage);
    if (!root.has_value()) {
        return std::nullopt;
    }
    try {
        return HistoryEntryFromRoot(root.value(), setupPath, errorMessage);
    } catch (const std::exception& error) {
        SetError(
            errorMessage,
            "Failed to read render setup history metadata: " +
                std::string{error.what()});
        return std::nullopt;
    }
}

std::vector<RenderSetupHistoryEntry> DiscoverRenderSetupHistory(
    const std::filesystem::path& directory,
    std::size_t maximumEntries,
    std::string* errorMessage) {
    std::vector<RenderSetupHistoryEntry> entries;
    std::error_code iteratorError;
    std::filesystem::directory_iterator iterator{directory, iteratorError};
    if (iteratorError) {
        SetError(
            errorMessage,
            "Failed to inspect render setup directory: " +
                iteratorError.message());
        return entries;
    }
    for (const auto& item : iterator) {
        std::error_code typeError;
        if (!item.is_regular_file(typeError) || typeError ||
            !EndsWith(item.path().filename().string(), kRenderSetupSuffix)) {
            continue;
        }
        std::string ignoredError;
        auto entry = ReadRenderSetupHistoryEntry(item.path(), &ignoredError);
        if (entry.has_value()) {
            entries.push_back(std::move(entry.value()));
        }
    }
    std::sort(entries.begin(), entries.end(), HistoryEntryNewer);
    if (entries.size() > maximumEntries) {
        entries.resize(maximumEntries);
    }
    return entries;
}

bool SaveRenderSetupHistoryIndex(
    const RenderSetupHistoryIndex& index,
    const std::filesystem::path& outputPath,
    std::string* errorMessage) {
    auto entries = index.entries;
    std::sort(entries.begin(), entries.end(), HistoryEntryNewer);
    if (entries.size() > kMaximumRenderSetupHistoryEntries) {
        entries.resize(kMaximumRenderSetupHistoryEntries);
    }
    json entriesJson = json::array();
    for (const auto& entry : entries) {
        entriesJson.push_back(HistoryEntryToJson(entry));
    }
    return WriteJsonAtomically(
        {
            {"schema_version", kRenderSetupHistorySchemaVersion},
            {"entries", std::move(entriesJson)},
        },
        outputPath,
        errorMessage);
}

std::optional<RenderSetupHistoryIndex> LoadRenderSetupHistoryIndex(
    const std::filesystem::path& inputPath,
    std::string* errorMessage) {
    const auto root = ReadJson(inputPath, errorMessage);
    if (!root.has_value()) {
        return std::nullopt;
    }
    try {
        if (!root->is_object() ||
            root->value("schema_version", 0U) !=
                kRenderSetupHistorySchemaVersion ||
            !root->contains("entries") || !root->at("entries").is_array()) {
            SetError(
                errorMessage,
                "Render setup history index is invalid or unsupported.");
            return std::nullopt;
        }
        RenderSetupHistoryIndex index;
        for (const auto& value : root->at("entries")) {
            auto entry = ParseHistoryEntry(value, errorMessage);
            if (!entry.has_value()) {
                return std::nullopt;
            }
            index.entries.push_back(std::move(entry.value()));
        }
        std::sort(index.entries.begin(), index.entries.end(), HistoryEntryNewer);
        if (index.entries.size() > kMaximumRenderSetupHistoryEntries) {
            index.entries.resize(kMaximumRenderSetupHistoryEntries);
        }
        return index;
    } catch (const std::exception& error) {
        SetError(
            errorMessage,
            "Failed to parse render setup history index: " +
                std::string{error.what()});
        return std::nullopt;
    }
}

bool UpsertRenderSetupHistoryEntry(
    const std::filesystem::path& indexPath,
    const RenderSetupHistoryEntry& entry,
    std::size_t maximumEntries,
    std::string* errorMessage) {
    RenderSetupHistoryIndex index;
    if (std::filesystem::exists(indexPath)) {
        auto loaded = LoadRenderSetupHistoryIndex(indexPath, errorMessage);
        if (!loaded.has_value()) {
            return false;
        }
        index = std::move(loaded.value());
    }
    const auto normalizedPath = entry.setupPath.lexically_normal();
    index.entries.erase(
        std::remove_if(
            index.entries.begin(),
            index.entries.end(),
            [&normalizedPath](const RenderSetupHistoryEntry& existing) {
                return existing.setupPath.lexically_normal() == normalizedPath;
            }),
        index.entries.end());
    index.entries.push_back(entry);
    std::sort(index.entries.begin(), index.entries.end(), HistoryEntryNewer);
    maximumEntries = std::min(
        maximumEntries,
        kMaximumRenderSetupHistoryEntries);
    if (index.entries.size() > maximumEntries) {
        index.entries.resize(maximumEntries);
    }
    return SaveRenderSetupHistoryIndex(index, indexPath, errorMessage);
}

}  // namespace invisible_places::serialization
