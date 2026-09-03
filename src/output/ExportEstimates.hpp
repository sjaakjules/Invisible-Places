#pragma once

#include "output/RenderPreset.hpp"

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace invisible_places::output {

// One completed export, persisted locally so the export panel can quote a
// real time and file-size figure the next time a comparable selection is
// made. Records live outside the project document: estimates are machine
// facts, not authored content, and must never dirty an artist save.
struct ExportHistoryRecord {
    std::string animationName;
    std::string visualName;
    AnimationExportMode mode = AnimationExportMode::FastPreviewMp4;
    AnimationExportQuality quality = AnimationExportQuality::Normal;
    bool useVideoToolbox = false;
    bool externalAlphaMatte = false;
    std::uint32_t supersampleScale = 1U;
    std::uint32_t outputWidth = 0U;
    std::uint32_t outputHeight = 0U;
    std::uint32_t frameCount = 0U;
    double wallSeconds = 0.0;
    // Median warm GPU capture cost. Wall seconds also include one-time
    // setup, so scaling to a different frame count uses this rate plus the
    // recorded fixed overhead rather than a plain wall/frames division.
    double captureSecondsPerFrame = 0.0;
    std::uint64_t outputBytes = 0U;
    std::string completedAtIso;
};

// Missing or unreadable history parses to an empty list; the file is
// recreated by the next append.
[[nodiscard]] std::vector<ExportHistoryRecord> LoadExportHistory(
    const std::filesystem::path& historyPath);

// Appends via a temp-file rename, keeping the newest maxRecords entries.
// Returns an empty string on success, otherwise a description of the error.
std::string AppendExportHistoryRecord(
    const std::filesystem::path& historyPath,
    const ExportHistoryRecord& record,
    std::size_t maxRecords = 200U);

struct ExportEstimateSelection {
    AnimationExportMode mode = AnimationExportMode::FastPreviewMp4;
    AnimationExportQuality quality = AnimationExportQuality::Normal;
    bool useVideoToolbox = false;
    bool externalAlphaMatte = false;
    std::uint32_t supersampleScale = 1U;
    std::uint32_t outputWidth = 0U;
    std::uint32_t outputHeight = 0U;
    std::string animationName;
    std::string visualName;
};

// The encoder settings must match exactly - a VideoToolbox HQ figure says
// nothing about an x265 render. Among matches, content similarity dominates
// (same visual, then same animation), then same supersample and resolution;
// newer records win ties. Returns nullptr when nothing comparable exists.
[[nodiscard]] const ExportHistoryRecord* FindBestExportHistoryMatch(
    const std::vector<ExportHistoryRecord>& records,
    const ExportEstimateSelection& selection);

struct ExportTimeSizeEstimate {
    double secondsPerFrame = 0.0;
    double totalSeconds = 0.0;
    std::uint64_t totalBytes = 0U;
    // The matched render used a different pixel count and the per-frame rate
    // was scaled linearly (measured: supersample 2 vs 4 tracks pixel count).
    bool pixelScaled = false;
};

[[nodiscard]] ExportTimeSizeEstimate EstimateExportTimeAndSize(
    const ExportHistoryRecord& record,
    const ExportEstimateSelection& selection,
    std::size_t selectionFrameCount);

// Measured SSIM against the ProRes 4444 / lossless PNG gold standard, as
// percentages, at the two content anchors that were scored on Surface_05:
// "smooth" (the dense Base pass) and "fine" (the near-pixel-speckle Thin
// pass, the hardest content for any encoder). Anchors that were never
// measured stay empty rather than being guessed.
struct ExportQualityEstimate {
    std::optional<float> colourSmoothPercent;
    std::optional<float> colourFinePercent;
    std::optional<float> alphaSmoothPercent;
    std::optional<float> alphaFinePercent;
    bool lossless = false;
};

[[nodiscard]] std::optional<ExportQualityEstimate> LookupExportQualityEstimate(
    AnimationExportMode mode,
    AnimationExportQuality quality,
    bool useVideoToolbox);

}  // namespace invisible_places::output
