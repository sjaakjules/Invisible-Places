#include "output/ExportEstimates.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cctype>
#include <fstream>
#include <string_view>
#include <system_error>

namespace invisible_places::output {
namespace {

// Stable string keys so history survives enum reordering across versions.
constexpr std::pair<AnimationExportMode, std::string_view> kModeNames[] = {
    {AnimationExportMode::FastPreviewMp4, "mp4"},
    {AnimationExportMode::TestMp4, "test_mp4"},
    {AnimationExportMode::HevcAlphaMp4, "hevc_alpha_mp4"},
    {AnimationExportMode::PngStack, "png_stack"},
    {AnimationExportMode::FastPngStack, "fast_png_stack"},
    {AnimationExportMode::HqPreviewDensityExr, "hq_preview_density_exr"},
    {AnimationExportMode::ProRes422Mov, "prores422"},
    {AnimationExportMode::ProRes422HqMov, "prores422_hq"},
    {AnimationExportMode::ProRes422AlphaMatteMov, "prores422_alpha"},
    {AnimationExportMode::ProRes422HqAlphaMatteMov, "prores422_hq_alpha"},
    {AnimationExportMode::ProRes422VideoToolboxMov, "prores422_vt"},
    {AnimationExportMode::ProRes422HqVideoToolboxMov, "prores422_hq_vt"},
    {AnimationExportMode::ProRes4444Mov, "prores4444"},
    {AnimationExportMode::ProRes4444XqMov, "prores4444_xq"},
    {AnimationExportMode::ProRes4444VideoToolboxMov, "prores4444_vt"},
    {AnimationExportMode::ProRes4444XqVideoToolboxMov, "prores4444_xq_vt"},
};

constexpr std::pair<AnimationExportQuality, std::string_view> kQualityNames[] = {
    {AnimationExportQuality::Normal, "normal"},
    {AnimationExportQuality::Hq, "hq"},
    {AnimationExportQuality::Xq, "xq"},
};

[[nodiscard]] std::string_view ModeName(AnimationExportMode mode) {
    for (const auto& [value, name] : kModeNames) {
        if (value == mode) {
            return name;
        }
    }
    return "mp4";
}

[[nodiscard]] std::optional<AnimationExportMode> ModeFromName(
    std::string_view name) {
    for (const auto& [value, modeName] : kModeNames) {
        if (modeName == name) {
            return value;
        }
    }
    return std::nullopt;
}

[[nodiscard]] std::string_view QualityName(AnimationExportQuality quality) {
    for (const auto& [value, name] : kQualityNames) {
        if (value == quality) {
            return name;
        }
    }
    return "normal";
}

[[nodiscard]] std::optional<AnimationExportQuality> QualityFromName(
    std::string_view name) {
    for (const auto& [value, qualityName] : kQualityNames) {
        if (qualityName == name) {
            return value;
        }
    }
    return std::nullopt;
}

[[nodiscard]] bool EqualsIgnoreCase(std::string_view a, std::string_view b) {
    return a.size() == b.size() &&
           std::equal(a.begin(), a.end(), b.begin(), [](char x, char y) {
               return std::tolower(static_cast<unsigned char>(x)) ==
                      std::tolower(static_cast<unsigned char>(y));
           });
}

[[nodiscard]] std::uint64_t SelectionPixels(
    std::uint32_t width,
    std::uint32_t height,
    std::uint32_t supersampleScale) {
    const auto scale = std::max<std::uint64_t>(1U, supersampleScale);
    return static_cast<std::uint64_t>(width) * height * scale * scale;
}

[[nodiscard]] nlohmann::json RecordToJson(const ExportHistoryRecord& record) {
    return nlohmann::json{
        {"animation", record.animationName},
        {"visual", record.visualName},
        {"mode", ModeName(record.mode)},
        {"quality", QualityName(record.quality)},
        {"video_toolbox", record.useVideoToolbox},
        {"external_alpha_matte", record.externalAlphaMatte},
        {"supersample", record.supersampleScale},
        {"width", record.outputWidth},
        {"height", record.outputHeight},
        {"frames", record.frameCount},
        {"wall_seconds", record.wallSeconds},
        {"capture_seconds_per_frame", record.captureSecondsPerFrame},
        {"output_bytes", record.outputBytes},
        {"completed_at", record.completedAtIso},
    };
}

[[nodiscard]] std::optional<ExportHistoryRecord> RecordFromJson(
    const nlohmann::json& value) {
    if (!value.is_object()) {
        return std::nullopt;
    }
    const auto mode = ModeFromName(value.value("mode", std::string{}));
    const auto quality = QualityFromName(value.value("quality", std::string{}));
    if (!mode.has_value() || !quality.has_value()) {
        return std::nullopt;
    }
    ExportHistoryRecord record;
    record.animationName = value.value("animation", std::string{});
    record.visualName = value.value("visual", std::string{});
    record.mode = mode.value();
    record.quality = quality.value();
    record.useVideoToolbox = value.value("video_toolbox", false);
    record.externalAlphaMatte = value.value("external_alpha_matte", false);
    record.supersampleScale = value.value("supersample", 1U);
    record.outputWidth = value.value("width", 0U);
    record.outputHeight = value.value("height", 0U);
    record.frameCount = value.value("frames", 0U);
    record.wallSeconds = value.value("wall_seconds", 0.0);
    record.captureSecondsPerFrame =
        value.value("capture_seconds_per_frame", 0.0);
    record.outputBytes = value.value("output_bytes", std::uint64_t{0U});
    record.completedAtIso = value.value("completed_at", std::string{});
    if (record.frameCount == 0U || record.wallSeconds <= 0.0) {
        return std::nullopt;
    }
    return record;
}

}  // namespace

std::vector<ExportHistoryRecord> LoadExportHistory(
    const std::filesystem::path& historyPath) {
    std::ifstream stream{historyPath};
    if (!stream.is_open()) {
        return {};
    }
    const auto document =
        nlohmann::json::parse(stream, nullptr, /*allow_exceptions=*/false);
    if (!document.is_object()) {
        return {};
    }
    const auto records = document.find("renders");
    if (records == document.end() || !records->is_array()) {
        return {};
    }
    std::vector<ExportHistoryRecord> history;
    history.reserve(records->size());
    for (const auto& entry : *records) {
        if (auto record = RecordFromJson(entry); record.has_value()) {
            history.push_back(std::move(record.value()));
        }
    }
    return history;
}

std::string AppendExportHistoryRecord(
    const std::filesystem::path& historyPath,
    const ExportHistoryRecord& record,
    std::size_t maxRecords) {
    auto history = LoadExportHistory(historyPath);
    history.push_back(record);
    if (maxRecords > 0U && history.size() > maxRecords) {
        history.erase(
            history.begin(),
            history.begin() +
                static_cast<std::ptrdiff_t>(history.size() - maxRecords));
    }

    nlohmann::json renders = nlohmann::json::array();
    for (const auto& entry : history) {
        renders.push_back(RecordToJson(entry));
    }
    const nlohmann::json document{{"version", 1}, {"renders", renders}};

    std::error_code directoryError;
    std::filesystem::create_directories(
        historyPath.parent_path(),
        directoryError);
    if (directoryError) {
        return "Unable to create " + historyPath.parent_path().string() +
               ": " + directoryError.message();
    }
    auto temporaryPath = historyPath;
    temporaryPath += ".tmp";
    {
        std::ofstream stream{temporaryPath, std::ios::trunc};
        if (!stream.is_open()) {
            return "Unable to write " + temporaryPath.string() + ".";
        }
        stream << document.dump(1, '\t') << '\n';
        if (!stream.good()) {
            return "Unable to write " + temporaryPath.string() + ".";
        }
    }
    std::error_code renameError;
    std::filesystem::rename(temporaryPath, historyPath, renameError);
    if (renameError) {
        std::filesystem::remove(temporaryPath, renameError);
        return "Unable to replace " + historyPath.string() + ".";
    }
    return {};
}

const ExportHistoryRecord* FindBestExportHistoryMatch(
    const std::vector<ExportHistoryRecord>& records,
    const ExportEstimateSelection& selection) {
    const ExportHistoryRecord* best = nullptr;
    int bestScore = -1;
    for (const auto& record : records) {
        if (record.mode != selection.mode ||
            record.quality != selection.quality ||
            record.useVideoToolbox != selection.useVideoToolbox ||
            record.externalAlphaMatte != selection.externalAlphaMatte) {
            continue;
        }
        int score = 0;
        if (!selection.visualName.empty() &&
            EqualsIgnoreCase(record.visualName, selection.visualName)) {
            score += 8;
        }
        if (!selection.animationName.empty() &&
            EqualsIgnoreCase(record.animationName, selection.animationName)) {
            score += 4;
        }
        if (record.supersampleScale == selection.supersampleScale) {
            score += 2;
        }
        if (record.outputWidth == selection.outputWidth &&
            record.outputHeight == selection.outputHeight) {
            score += 1;
        }
        // >= keeps the newest of equally similar renders: history is stored
        // oldest first.
        if (score >= bestScore) {
            bestScore = score;
            best = &record;
        }
    }
    return best;
}

ExportTimeSizeEstimate EstimateExportTimeAndSize(
    const ExportHistoryRecord& record,
    const ExportEstimateSelection& selection,
    std::size_t selectionFrameCount) {
    ExportTimeSizeEstimate estimate;
    const auto frameCount = std::max<std::size_t>(1U, selectionFrameCount);
    const auto recordFrames = std::max<std::uint32_t>(1U, record.frameCount);

    auto secondsPerFrame = record.captureSecondsPerFrame > 0.0
                               ? record.captureSecondsPerFrame
                               : record.wallSeconds / recordFrames;
    const auto recordPixels = SelectionPixels(
        record.outputWidth,
        record.outputHeight,
        record.supersampleScale);
    const auto selectionPixels = SelectionPixels(
        selection.outputWidth,
        selection.outputHeight,
        selection.supersampleScale);
    if (recordPixels > 0U && selectionPixels > 0U &&
        recordPixels != selectionPixels) {
        secondsPerFrame *= static_cast<double>(selectionPixels) /
                           static_cast<double>(recordPixels);
        estimate.pixelScaled = true;
    }
    // Setup (sidecar parse, frustum masks, residency) is a fixed cost on top
    // of the per-frame rate; the record keeps it as wall minus capture.
    const auto recordCaptureSeconds =
        record.captureSecondsPerFrame > 0.0
            ? record.captureSecondsPerFrame * recordFrames
            : record.wallSeconds;
    const auto fixedOverheadSeconds =
        std::max(0.0, record.wallSeconds - recordCaptureSeconds);

    estimate.secondsPerFrame = secondsPerFrame;
    estimate.totalSeconds =
        secondsPerFrame * static_cast<double>(frameCount) +
        fixedOverheadSeconds;
    estimate.totalBytes = static_cast<std::uint64_t>(
        static_cast<double>(record.outputBytes) /
        static_cast<double>(recordFrames) * static_cast<double>(frameCount));
    return estimate;
}

std::optional<ExportQualityEstimate> LookupExportQualityEstimate(
    AnimationExportMode mode,
    AnimationExportQuality quality,
    bool useVideoToolbox) {
    // Sources: docs/thin_export_encoding.md (2026-09-02..04 measurements on
    // Surface_05). "Fine" is the Thin_v2 speckle pass scored against
    // lossless PNG / ProRes 4444; "smooth" is the Base_v2 pass.
    ExportQualityEstimate estimate;
    switch (mode) {
        case AnimationExportMode::PngStack:
        case AnimationExportMode::FastPngStack:
        case AnimationExportMode::HqPreviewDensityExr:
            estimate.lossless = true;
            return estimate;
        case AnimationExportMode::TestMp4:
            // Motion-interpolated preview; scoring it against the gold
            // standard would be meaningless.
            return std::nullopt;
        case AnimationExportMode::FastPreviewMp4:
        case AnimationExportMode::HevcAlphaMp4:
            switch (quality) {
                case AnimationExportQuality::Normal:
                    return std::nullopt;
                case AnimationExportQuality::Hq:
                    if (useVideoToolbox) {
                        estimate.colourSmoothPercent = 98.8F;
                        estimate.colourFinePercent = 92.3F;
                        estimate.alphaSmoothPercent = 99.6F;
                        estimate.alphaFinePercent = 90.4F;
                    } else {
                        estimate.colourFinePercent = 97.2F;
                        estimate.alphaFinePercent = 99.7F;
                    }
                    return estimate;
                case AnimationExportQuality::Xq:
                    // The x265 Max tier uses lower CRFs than HQ; the HQ CPU
                    // scores are quoted as its floor rather than guessing.
                    estimate.colourFinePercent = 97.2F;
                    estimate.alphaFinePercent = useVideoToolbox ? 99.8F : 99.7F;
                    return estimate;
            }
            return std::nullopt;
        case AnimationExportMode::ProRes422Mov:
        case AnimationExportMode::ProRes422AlphaMatteMov:
        case AnimationExportMode::ProRes422VideoToolboxMov:
            if (quality != AnimationExportQuality::Hq) {
                return std::nullopt;
            }
            [[fallthrough]];
        case AnimationExportMode::ProRes422HqMov:
        case AnimationExportMode::ProRes422HqAlphaMatteMov:
        case AnimationExportMode::ProRes422HqVideoToolboxMov:
            estimate.colourSmoothPercent = 98.4F;
            estimate.colourFinePercent = 95.2F;
            estimate.alphaSmoothPercent = 99.8F;
            estimate.alphaFinePercent = 98.5F;
            return estimate;
        case AnimationExportMode::ProRes4444Mov:
        case AnimationExportMode::ProRes4444XqMov:
        case AnimationExportMode::ProRes4444VideoToolboxMov:
        case AnimationExportMode::ProRes4444XqVideoToolboxMov:
            // The gold standard itself; scored 99.96/99.98 on the speckle
            // pass against lossless PNG.
            estimate.colourFinePercent = 99.96F;
            estimate.alphaFinePercent = 99.98F;
            return estimate;
    }
    return std::nullopt;
}

}  // namespace invisible_places::output
