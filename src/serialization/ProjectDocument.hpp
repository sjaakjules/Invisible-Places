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

inline constexpr std::size_t kMaxSerializedWaterRippleRuntimeCacheMemberships = 250'000U;

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

struct WaterAnimationTrailProfileDocument {
    std::string name = "Unnamed";
    invisible_places::water::WaterAnimationTrailSettings settings{};
};

struct WaterPathProfileDocument {
    std::string name = "Default";
    invisible_places::water::WaterPathGenerationSettings settings{};
};

struct WaterLaneProfileDocument {
    std::string name = "Default";
    invisible_places::water::WaterFlowStreamSettings settings{};
};

struct WaterTrailProfileDocument {
    std::string name = "Default";
    invisible_places::water::WaterTrailGeometrySettings geometry{};
    invisible_places::renderer::pointcloud::PointCloudStyleState style{};
};

struct WaterRippleRuntimeCacheDocument {
    std::uint32_t schemaVersion = 1;
    std::filesystem::path supportLayerPath;
    std::string supportSignature;
    std::string regionFingerprint;
    std::vector<invisible_places::water::WaterRippleRuntimeMembership> memberships;
    std::vector<invisible_places::water::WaterRippleRuntimeParams> params;
    bool stale = false;
};

struct ProjectDocument {
    struct SavedAnimation {
        std::filesystem::path filePath;
        std::vector<std::filesystem::path> associatedLayerPaths;
    };

    std::uint32_t schemaVersion = 25;
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
    std::vector<invisible_places::water::WaterEffectLayer> waterRippleLayers;
    invisible_places::water::WaterSourceSettings waterSourceSettings{};
    std::optional<invisible_places::water::WaterSourceSettings> tempWaterSourceSettings;
    invisible_places::water::WaterAnimationTrailSettings waterAnimationTrailSettings{};
    std::optional<invisible_places::water::WaterAnimationTrailSettings> tempWaterAnimationTrailSettings;
    std::vector<WaterAnimationTrailProfileDocument> waterAnimationTrailProfiles;
    invisible_places::water::WaterTrailGeometrySettings waterTrailGeometry{};
    std::vector<WaterPathProfileDocument> waterPathProfiles;
    std::vector<WaterLaneProfileDocument> waterLaneProfiles;
    std::vector<WaterTrailProfileDocument> waterTrailProfiles;
    std::string selectedWaterPathProfileName = "Default";
    std::string selectedWaterLaneProfileName = "Default";
    std::string selectedWaterTrailProfileName = "Default";
    std::optional<invisible_places::water::WaterPathGenerationSettings> tempWaterPathProfileSettings;
    std::optional<invisible_places::water::WaterFlowStreamSettings> tempWaterLaneProfileSettings;
    std::optional<WaterTrailProfileDocument> tempWaterTrailProfile;
    invisible_places::water::WaterCausticLookSettings waterCausticLookSettings{};
    std::optional<invisible_places::water::WaterCausticLookSettings> tempWaterCausticLookSettings;
    std::vector<ProjectLayerDocument::PointVisual> waterPointVisuals;
    std::string selectedWaterPointVisualName = "Water Flow_preset";
    invisible_places::renderer::pointcloud::PointCloudStyleState waterPointVisualStyle{};
    std::optional<invisible_places::renderer::pointcloud::PointCloudStyleState> tempWaterPointVisualStyle;
    invisible_places::water::WaterVisualSettings waterVisualSettings{};
    std::optional<invisible_places::water::WaterVisualSettings> tempWaterVisualSettings;
    invisible_places::water::WaterSettingsBundle waterSettings{};
    std::optional<invisible_places::water::WaterSettingsBundle> tempWaterSettings;
    invisible_places::water::WaterBakeSettings waterBakeSettings{};
    invisible_places::water::WaterRenderSettings waterRenderSettings{};
    invisible_places::water::WaterFlowStreamSettings waterFlowStreamSettings{};
    invisible_places::water::WaterFieldSettings waterFieldSettings{};
    invisible_places::water::WaterFieldStreamSettings waterFieldStreamSettings{};
    std::vector<invisible_places::water::WaterEffectLayer> waterFieldLayers;
    std::optional<invisible_places::water::WaterPathCache> waterPathCache;
    std::vector<WaterRippleRuntimeCacheDocument> waterRippleRuntimeCaches;
};

struct PointCloudStylePresetDocument {
    std::uint32_t schemaVersion = 2;
    std::string presetName;
    invisible_places::renderer::pointcloud::PointCloudStyleState style{};
};

struct WaterSourcesDocument {
    std::uint32_t schemaVersion = 7;
    std::vector<invisible_places::water::WaterEmitter> emitters;
    std::vector<invisible_places::water::WaterEffectLayer> rippleLayers;
    std::vector<invisible_places::water::WaterEffectLayer> fieldLayers;
    invisible_places::water::WaterSourceSettings sourceSettings{};
    std::optional<invisible_places::water::WaterSourceSettings> tempSourceSettings;
    invisible_places::water::WaterCausticLookSettings causticLookSettings{};
    std::optional<invisible_places::water::WaterCausticLookSettings> tempCausticLookSettings;
    invisible_places::water::WaterSettingsBundle settings{};
    std::optional<invisible_places::water::WaterSettingsBundle> tempSettings;
    std::vector<WaterPathProfileDocument> pathProfiles;
    std::vector<WaterLaneProfileDocument> laneProfiles;
    std::vector<WaterTrailProfileDocument> trailProfiles;
    std::string selectedPathProfileName = "Default";
    std::string selectedLaneProfileName = "Default";
    std::string selectedTrailProfileName = "Default";
    std::optional<invisible_places::water::WaterPathGenerationSettings> tempPathProfileSettings;
    std::optional<invisible_places::water::WaterFlowStreamSettings> tempLaneProfileSettings;
    std::optional<WaterTrailProfileDocument> tempTrailProfile;
    invisible_places::water::WaterBakeSettings bakeSettings{};
    invisible_places::water::WaterRenderSettings renderSettings{};
    invisible_places::water::WaterFlowStreamSettings flowStreamSettings{};
    invisible_places::water::WaterTrailGeometrySettings trailGeometry{};
    invisible_places::water::WaterFieldSettings fieldSettings{};
    invisible_places::water::WaterFieldStreamSettings fieldStreamSettings{};
    std::optional<invisible_places::water::WaterPathCache> pathCache;
    std::vector<WaterRippleRuntimeCacheDocument> rippleRuntimeCaches;
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
bool SaveWaterPathCacheDocument(
    const invisible_places::water::WaterPathCache& document,
    const std::filesystem::path& outputPath,
    std::string* errorMessage);
std::optional<invisible_places::water::WaterPathCache> LoadWaterPathCacheDocument(
    const std::filesystem::path& inputPath,
    std::string* errorMessage);

}  // namespace invisible_places::serialization
