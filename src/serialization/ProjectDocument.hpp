#pragma once

#include "camera/AnimationPath.hpp"
#include "camera/CameraShot.hpp"
#include "output/RenderPreset.hpp"
#include "renderer/pointcloud/PointCloudPreviewState.hpp"
#include "water/WaterFlow.hpp"

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
    struct SavedAnimation {
        std::filesystem::path filePath;
        std::vector<std::filesystem::path> associatedLayerPaths;
    };

    std::uint32_t schemaVersion = 18;
    std::string projectName;
    std::vector<ProjectLayerDocument> layers;
    std::optional<invisible_places::camera::CameraState> cameraState;
    std::vector<invisible_places::camera::CameraShot> cameraShots;
    std::vector<std::size_t> cameraPathShotIndices;
    std::uint32_t cameraPathDurationFrames = 180;
    std::vector<SavedAnimation> savedAnimations;
    bool hasSavedAnimationRegistry = false;
    std::filesystem::path selectedLayerPath;
    std::filesystem::path lastAnimationPath;
    std::array<float, 4> backgroundColor{0.0F, 0.0F, 0.0F, 1.0F};
    bool eyeDomeLightingEnabled = false;
    bool constantUpdateView = false;
    bool liveVisualEffects = false;
    bool sidePanelPinned = false;
    bool autoLowerGsplatQualityWhileNavigating = true;
    float eyeDomeLightingThickness = 1.0F;
    invisible_places::renderer::pointcloud::PointCloudPreviewLodMode pointCloudPreviewLodMode =
        invisible_places::renderer::pointcloud::PointCloudPreviewLodMode::AutoCameraLod;
    std::uint64_t interactivePointCap = 10'000'000ULL;
    invisible_places::renderer::pointcloud::PointCloudRendererMode pointCloudRendererMode =
        invisible_places::renderer::pointcloud::PointCloudRendererMode::Beauty;
    invisible_places::output::RenderJobSettings renderJobSettings{};
    std::vector<invisible_places::water::WaterEmitter> waterEmitters;
    invisible_places::water::WaterSourceSettings waterSourceSettings{};
    std::optional<invisible_places::water::WaterSourceSettings> tempWaterSourceSettings;
    invisible_places::water::WaterAnimationTrailSettings waterAnimationTrailSettings{};
    std::optional<invisible_places::water::WaterAnimationTrailSettings> tempWaterAnimationTrailSettings;
    invisible_places::renderer::pointcloud::PointCloudStyleState waterPointVisualStyle{};
    std::optional<invisible_places::renderer::pointcloud::PointCloudStyleState> tempWaterPointVisualStyle;
    invisible_places::water::WaterVisualSettings waterVisualSettings{};
    std::optional<invisible_places::water::WaterVisualSettings> tempWaterVisualSettings;
    invisible_places::water::WaterSettingsBundle waterSettings{};
    std::optional<invisible_places::water::WaterSettingsBundle> tempWaterSettings;
    invisible_places::water::WaterBakeSettings waterBakeSettings{};
    invisible_places::water::WaterRenderSettings waterRenderSettings{};
};

struct PointCloudStylePresetDocument {
    std::uint32_t schemaVersion = 2;
    std::string presetName;
    invisible_places::renderer::pointcloud::PointCloudStyleState style{};
};

struct WaterSourcesDocument {
    std::uint32_t schemaVersion = 2;
    std::vector<invisible_places::water::WaterEmitter> emitters;
    invisible_places::water::WaterSourceSettings sourceSettings{};
    std::optional<invisible_places::water::WaterSourceSettings> tempSourceSettings;
    invisible_places::water::WaterSettingsBundle settings{};
    std::optional<invisible_places::water::WaterSettingsBundle> tempSettings;
    invisible_places::water::WaterBakeSettings bakeSettings{};
    invisible_places::water::WaterRenderSettings renderSettings{};
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
bool SaveWaterSourcesDocument(
    const WaterSourcesDocument& document,
    const std::filesystem::path& outputPath,
    std::string* errorMessage);
std::optional<WaterSourcesDocument> LoadWaterSourcesDocument(
    const std::filesystem::path& inputPath,
    std::string* errorMessage);

}  // namespace invisible_places::serialization
