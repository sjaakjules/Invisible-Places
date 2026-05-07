#pragma once

#include "camera/AnimationPath.hpp"
#include "camera/CameraShot.hpp"
#include "output/RenderPreset.hpp"
#include "renderer/pointcloud/PointCloudPreviewState.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
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
    struct PointVisual {
        std::string name = "Unnamed";
        invisible_places::renderer::pointcloud::PointCloudStyleState style{};
    };
    std::vector<PointVisual> pointVisuals;
    std::string selectedPointVisualName = "Unnamed";
};

struct ProjectDocument {
    std::uint32_t schemaVersion = 11;
    std::string projectName;
    std::vector<ProjectLayerDocument> layers;
    std::optional<invisible_places::camera::CameraState> cameraState;
    std::vector<invisible_places::camera::CameraShot> cameraShots;
    std::vector<std::size_t> cameraPathShotIndices;
    std::uint32_t cameraPathDurationFrames = 180;
    std::filesystem::path selectedLayerPath;
    std::filesystem::path lastAnimationPath;
    std::array<float, 4> backgroundColor{0.0F, 0.0F, 0.0F, 1.0F};
    bool sidePanelPinned = false;
    bool autoLowerGsplatQualityWhileNavigating = true;
    invisible_places::renderer::pointcloud::PointCloudPreviewLodMode pointCloudPreviewLodMode =
        invisible_places::renderer::pointcloud::PointCloudPreviewLodMode::AutoCameraLod;
    std::uint64_t interactivePointCap = 10'000'000ULL;
    invisible_places::output::RenderJobSettings renderJobSettings{};
};

struct PointCloudStylePresetDocument {
    std::uint32_t schemaVersion = 2;
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
bool SaveAnimationPath(
    const invisible_places::camera::AnimationPath& path,
    const std::filesystem::path& outputPath,
    std::string* errorMessage);
std::optional<invisible_places::camera::AnimationPath> LoadAnimationPath(
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
