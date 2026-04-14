#pragma once

#include "renderer/pointcloud/PointCloudPreviewState.hpp"

#include <array>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace invisible_places::serialization {

enum class SerializedLayerKind {
    PointCloud,
    GaussianSplat
};

struct ProjectLayerDocument {
    SerializedLayerKind kind = SerializedLayerKind::PointCloud;
    std::filesystem::path sourcePath;
    bool loaded = false;
    bool visible = false;
    std::uint64_t pointBudgetActivePoints = 0;
    std::optional<invisible_places::renderer::pointcloud::PointCloudStyleState> pointStyle;
};

struct ProjectDocument {
    std::uint32_t schemaVersion = 2;
    std::string projectName;
    std::vector<ProjectLayerDocument> layers;
    std::filesystem::path selectedLayerPath;
    std::array<float, 4> backgroundColor{0.0F, 0.0F, 0.0F, 1.0F};
    bool sidePanelPinned = false;
    bool autoLowerGsplatQualityWhileNavigating = true;
};

struct PointCloudStylePresetDocument {
    std::uint32_t schemaVersion = 1;
    std::string presetName;
    invisible_places::renderer::pointcloud::PointCloudStyleState style{};
};

bool SaveProjectDocument(
    const ProjectDocument& document,
    const std::filesystem::path& outputPath,
    std::string* errorMessage);
std::optional<ProjectDocument> LoadProjectDocument(
    const std::filesystem::path& inputPath,
    std::string* errorMessage);
bool SavePointCloudStylePreset(
    const PointCloudStylePresetDocument& document,
    const std::filesystem::path& outputPath,
    std::string* errorMessage);
std::optional<PointCloudStylePresetDocument> LoadPointCloudStylePreset(
    const std::filesystem::path& inputPath,
    std::string* errorMessage);

}  // namespace invisible_places::serialization
