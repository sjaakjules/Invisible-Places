#pragma once

#include "serialization/ProjectDocument.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace invisible_places::serialization {

inline constexpr std::uint32_t kRenderSetupDocumentSchemaVersion = 4U;
inline constexpr std::uint32_t kMinimumRenderSetupDocumentSchemaVersion = 1U;
inline constexpr std::uint32_t kRenderSetupHistorySchemaVersion = 1U;
inline constexpr std::size_t kMaximumRenderSetupHistoryEntries = 100U;

enum class RenderSetupStatus : std::uint8_t {
    Rendering = 0,
    Completed,
    Failed,
    Cancelled,
};

struct RenderSetupRendererState {
    invisible_places::renderer::pointcloud::PointCloudRendererMode
        pointCloudRendererMode =
            invisible_places::renderer::pointcloud::PointCloudRendererMode::Beauty;
    std::array<float, 4> backgroundColor{0.0F, 0.0F, 0.0F, 1.0F};
    bool eyeDomeLightingEnabled = false;
    float eyeDomeLightingThickness = 1.0F;
    bool proResAlphaPreviewEnabled = false;
    float gaussianSplatFootprintBoost = 1.5F;
    // Stable policy identifier rather than a transient loaded source path.
    std::string densityPolicy = "finest_available";
};

struct RenderSetupSourceFingerprint {
    std::string sceneRole;
    std::filesystem::path sourcePath;
    std::uint64_t fileSize = 0U;
    std::int64_t modificationTimeTicks = 0;
};

struct RenderSetupSummary {
    std::size_t waterRunCount = 0U;
    std::size_t activeWaterTrackCount = 0U;
    std::size_t waterKeyCount = 0U;
    std::size_t enabledColouriseEffectCount = 0U;
    std::size_t colouriseKeyCount = 0U;
};

struct RenderSetupDocument {
    std::uint32_t schemaVersion = kRenderSetupDocumentSchemaVersion;
    RenderSetupStatus status = RenderSetupStatus::Rendering;
    std::string createdUtc;
    std::string completedUtc;
    std::string failureMessage;

    std::filesystem::path outputPath;
    std::filesystem::path logPath;
    std::filesystem::path sourceProjectPath;
    std::string sourceProjectIdentity;
    std::filesystem::path originalAnimationPath;
    std::string sceneGroupName;
    std::string timingTakeId =
        std::string{invisible_places::timing::kAuthoredTimingTakeId};
    std::string timingTakeName =
        std::string{invisible_places::timing::kAuthoredTimingTakeName};
    std::string visualName = "Unnamed";
    bool animationModified = false;

    invisible_places::camera::AnimationPath animation{};
    invisible_places::output::ExportPreset exportPreset =
        invisible_places::output::MakeMp4ExportPreset();
    invisible_places::renderer::pointcloud::PointCloudStyleState livePointVisual{};
    invisible_places::timing::TimingTakeSceneState timingState{};
    // SaveRenderSetupDocument always removes runtime/path caches from this
    // snapshot. Authored sources, profiles, and temporary edited values remain.
    WaterSourcesDocument authoredWater{};
    invisible_places::water::WaterAnimationTrailSettings
        waterAnimationTrailSettings{};
    std::optional<invisible_places::water::WaterAnimationTrailSettings>
        tempWaterAnimationTrailSettings;
    std::vector<WaterAnimationTrailProfileDocument>
        waterAnimationTrailProfiles;
    std::string selectedWaterAnimationTrailProfileName = "Default";
    std::vector<ProjectLayerDocument::PointVisual> waterPointVisuals;
    std::string selectedWaterPointVisualName = "Water Flow_preset";
    invisible_places::renderer::pointcloud::PointCloudStyleState
        waterPointVisualStyle{};
    std::optional<
        invisible_places::renderer::pointcloud::PointCloudStyleState>
        tempWaterPointVisualStyle;
    RenderSetupRendererState renderer{};
    RenderSetupSummary summary{};
    std::vector<std::string> editedSettingLabels;
    std::vector<RenderSetupSourceFingerprint> sourceFingerprints;
};

struct RenderSetupHistoryEntry {
    std::filesystem::path setupPath;
    std::filesystem::path outputPath;
    RenderSetupStatus status = RenderSetupStatus::Rendering;
    std::string createdUtc;
    std::string completedUtc;
    std::string animationName;
    std::string sceneGroupName;
    std::string timingTakeName;
    std::size_t editedSettingCount = 0U;
};

struct RenderSetupHistoryIndex {
    std::uint32_t schemaVersion = kRenderSetupHistorySchemaVersion;
    std::vector<RenderSetupHistoryEntry> entries;
};

[[nodiscard]] std::string_view RenderSetupStatusName(RenderSetupStatus status);
[[nodiscard]] std::optional<RenderSetupStatus> ParseRenderSetupStatus(
    std::string_view name);
[[nodiscard]] std::string CurrentUtcTimestamp();
[[nodiscard]] RenderSetupSummary SummarizeRenderSetupTiming(
    const invisible_places::timing::TimingTakeSceneState& state);

// Returns a dated sidecar beside outputPath. Existing names are preserved and
// receive a numeric suffix, so separate renders never overwrite provenance.
[[nodiscard]] std::filesystem::path AllocateRenderSetupSidecarPath(
    const std::filesystem::path& outputPath,
    std::string_view createdUtc = {});

bool SaveRenderSetupDocument(
    const RenderSetupDocument& document,
    const std::filesystem::path& outputPath,
    std::string* errorMessage = nullptr);
[[nodiscard]] std::optional<RenderSetupDocument> LoadRenderSetupDocument(
    const std::filesystem::path& inputPath,
    std::string* errorMessage = nullptr);
bool UpdateRenderSetupDocumentStatus(
    const std::filesystem::path& inputPath,
    RenderSetupStatus status,
    std::string_view completedUtc = {},
    std::string_view failureMessage = {},
    std::string* errorMessage = nullptr);

[[nodiscard]] std::optional<RenderSetupHistoryEntry>
ReadRenderSetupHistoryEntry(
    const std::filesystem::path& setupPath,
    std::string* errorMessage = nullptr);
[[nodiscard]] std::vector<RenderSetupHistoryEntry> DiscoverRenderSetupHistory(
    const std::filesystem::path& directory,
    std::size_t maximumEntries = kMaximumRenderSetupHistoryEntries,
    std::string* errorMessage = nullptr);

bool SaveRenderSetupHistoryIndex(
    const RenderSetupHistoryIndex& index,
    const std::filesystem::path& outputPath,
    std::string* errorMessage = nullptr);
[[nodiscard]] std::optional<RenderSetupHistoryIndex>
LoadRenderSetupHistoryIndex(
    const std::filesystem::path& inputPath,
    std::string* errorMessage = nullptr);
bool UpsertRenderSetupHistoryEntry(
    const std::filesystem::path& indexPath,
    const RenderSetupHistoryEntry& entry,
    std::size_t maximumEntries = kMaximumRenderSetupHistoryEntries,
    std::string* errorMessage = nullptr);

}  // namespace invisible_places::serialization
