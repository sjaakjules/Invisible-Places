#pragma once

#include "camera/AnimationPath.hpp"
#include "camera/CameraShot.hpp"
#include "output/RenderPreset.hpp"
#include "renderer/gsplat/GsplatLayer.hpp"
#include "renderer/pointcloud/PointCloudPreviewState.hpp"
#include "water/RainSimulation.hpp"
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
inline constexpr std::uint32_t kProjectDocumentSchemaVersion = 42U;
inline constexpr std::uint32_t kWaterSourcesDocumentSchemaVersion = 17U;
inline constexpr std::uint32_t kWaterPathCacheSidecarSchemaVersion = 1U;
inline constexpr std::uint64_t kMaximumPersistedWaterCacheBytes = 5ULL * 1024ULL * 1024ULL * 1024ULL;

enum class SerializedLayerKind {
    PointCloud,
    GaussianSplat
};

struct ProjectLayerDocument {
    SerializedLayerKind kind = SerializedLayerKind::PointCloud;
    std::filesystem::path sourcePath;
    std::string sceneGroupName;
    std::string sceneRole;
    float inferredPointSpacingMeters = 0.0F;
    float pointSpacingMeters = 0.0F;
    bool pointSpacingManualOverride = false;
    bool scenePrimaryRole = false;
    std::filesystem::path selectedSceneVariantPath;
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
    invisible_places::water::WaterFlowTrailSettings settings{};
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

struct WaterPathCacheManifestDocument {
    std::filesystem::path relativePath;
    std::uint32_t cacheSchema = kWaterPathCacheSidecarSchemaVersion;
    std::string supportSignature;
    std::string emitterSettingsFingerprint;
    std::uint64_t payloadBytes = 0U;
    std::array<std::uint64_t, 4> checksum{};
};

struct WaterSurfaceCacheManifestDocument {
    std::filesystem::path relativePath;
    std::uint32_t cacheSchema = invisible_places::water::kWaterSurfaceCacheSchemaVersion;
    std::string algorithmId = "water-surface-v3";
    std::string sourceFingerprint;
    std::uint64_t payloadBytes = 0U;
    std::array<std::uint64_t, 4> checksum{};
    std::uint64_t requestedRebuildGeneration = 0U;
    std::uint64_t builtRebuildGeneration = 0U;
};

struct WaterSceneStateDocument {
    std::string sceneGroupName = "Default";
    std::vector<invisible_places::water::WaterEmitter> emitters;
    std::vector<invisible_places::water::WaterManualFlowPathSource> manualFlowPaths;
    std::vector<invisible_places::water::WaterSeepageNode> seepageNodes;
    std::vector<invisible_places::water::WaterEffectLayer> rippleLayers;
    std::vector<invisible_places::water::WaterEffectLayer> fieldLayers;
    std::optional<invisible_places::water::WaterPathCache> pathCache;
    std::optional<WaterPathCacheManifestDocument> pathCacheManifest;
    std::vector<WaterRippleRuntimeCacheDocument> rippleRuntimeCaches;
    std::filesystem::path dynamicMeshPath;
    std::vector<invisible_places::water::WaterDynamicMeshAttractor> dynamicMeshAttractors;
    std::vector<invisible_places::water::WaterDynamicMeshEmitterMotion> dynamicMeshEmitterMotions;
};

struct ScenePointVisualStateDocument {
    std::string sceneGroupName;
    ProjectLayerDocument::PointVisual visual;
};

struct ScenePointCloudRoleSourceDocument {
    std::string sceneRole;
    std::filesystem::path analysisSourcePath;
    std::filesystem::path displaySourcePath;
};

struct ScenePointCloudGroupDocument {
    std::string sceneGroupName;
    float displaySpacingMeters = 0.0F;
    bool displayLoaded = false;
    bool displayVisible = false;
    std::vector<ScenePointCloudRoleSourceDocument> roleSources;
    std::optional<WaterSurfaceCacheManifestDocument> waterSurfaceCache;
};

struct ProjectDocument {
    struct SavedAnimation {
        std::filesystem::path filePath;
        std::vector<std::filesystem::path> associatedLayerPaths;
    };

    std::uint32_t schemaVersion = kProjectDocumentSchemaVersion;
    std::string projectName;
    std::vector<ProjectLayerDocument::PointVisual> pointVisuals;
    std::string selectedPointVisualName = "Unnamed";
    std::vector<ScenePointVisualStateDocument> sceneVisualStates;
    std::vector<ScenePointCloudGroupDocument> scenePointCloudGroups;
    invisible_places::renderer::gsplat::GaussianSplatStyleState gsplatVisualStyle{};
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
    bool proResAlphaPreviewEnabled = false;
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
    std::vector<invisible_places::output::ExportPreset> exportPresets;
    std::string selectedExportPresetName = std::string{invisible_places::output::kMp4PresetName};
    std::optional<invisible_places::output::ExportPreset> tempExportPreset;
    std::vector<invisible_places::water::WaterEmitter> waterEmitters;
    std::vector<invisible_places::water::WaterManualFlowPathSource> waterManualFlowPaths;
    std::vector<invisible_places::water::WaterSeepageNode> waterSeepageNodes;
    invisible_places::water::WaterSeepageLookSettings waterSeepageDefaultLook{};
    std::vector<invisible_places::water::WaterSeepageLookProfile> waterSeepageLookProfiles;
    std::vector<invisible_places::water::WaterScenarioDefinition> waterScenarios;
    std::string selectedWaterScenarioId;
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
    std::optional<invisible_places::water::WaterFlowTrailSettings> tempWaterLaneProfileSettings;
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
    invisible_places::water::WaterFlowTrailSettings waterFlowTrailSettings{};
    invisible_places::water::WaterFieldSettings waterFieldSettings{};
    invisible_places::water::WaterFieldTrailSettings waterFieldTrailSettings{};
    invisible_places::water::WaterDynamicMeshFlowSettings waterDynamicMeshFlowSettings{};
    invisible_places::water::RainRuntimeSettings waterRainSettings =
        invisible_places::water::DefaultRainRuntimeSettings();
    invisible_places::water::WaterRainVisualSettings waterRainVisualSettings =
        invisible_places::water::RainVisualPreset("Rain Fine Lines");
    std::vector<WaterSceneStateDocument> waterSceneStates;
    std::vector<invisible_places::water::WaterEffectLayer> waterFieldLayers;
    std::optional<invisible_places::water::WaterPathCache> waterPathCache;
    std::optional<WaterPathCacheManifestDocument> waterPathCacheManifest;
    std::vector<WaterRippleRuntimeCacheDocument> waterRippleRuntimeCaches;
};

struct PointCloudStylePresetDocument {
    std::uint32_t schemaVersion = 3;
    std::string presetName;
    invisible_places::renderer::pointcloud::PointCloudStyleState style{};
};

struct WaterSourcesDocument {
    std::uint32_t schemaVersion = kWaterSourcesDocumentSchemaVersion;
    std::vector<invisible_places::water::WaterEmitter> emitters;
    std::vector<invisible_places::water::WaterManualFlowPathSource> manualFlowPaths;
    std::vector<invisible_places::water::WaterSeepageNode> seepageNodes;
    invisible_places::water::WaterSeepageLookSettings seepageDefaultLook{};
    std::vector<invisible_places::water::WaterSeepageLookProfile> seepageLookProfiles;
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
    std::optional<invisible_places::water::WaterFlowTrailSettings> tempLaneProfileSettings;
    std::optional<WaterTrailProfileDocument> tempTrailProfile;
    invisible_places::water::WaterBakeSettings bakeSettings{};
    invisible_places::water::WaterRenderSettings renderSettings{};
    invisible_places::water::WaterFlowTrailSettings flowTrailSettings{};
    invisible_places::water::WaterTrailGeometrySettings trailGeometry{};
    invisible_places::water::WaterFieldSettings fieldSettings{};
    invisible_places::water::WaterFieldTrailSettings fieldTrailSettings{};
    invisible_places::water::WaterDynamicMeshFlowSettings dynamicMeshFlowSettings{};
    invisible_places::water::RainRuntimeSettings rainSettings =
        invisible_places::water::DefaultRainRuntimeSettings();
    invisible_places::water::WaterRainVisualSettings rainVisualSettings =
        invisible_places::water::RainVisualPreset("Rain Fine Lines");
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
    std::string* errorMessage,
    WaterPathCacheManifestDocument* manifest = nullptr);
bool SaveContentAddressedWaterPathCacheDocument(
    const invisible_places::water::WaterPathCache& document,
    const std::filesystem::path& outputDirectory,
    std::string* errorMessage,
    WaterPathCacheManifestDocument* manifest = nullptr,
    std::filesystem::path* outputPath = nullptr);
std::optional<invisible_places::water::WaterPathCache> LoadWaterPathCacheDocument(
    const std::filesystem::path& inputPath,
    std::string* errorMessage,
    WaterPathCacheManifestDocument* manifest = nullptr);

}  // namespace invisible_places::serialization
