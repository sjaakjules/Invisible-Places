#include "serialization/ProjectDocument.hpp"

#include "style/RenderParameterBinding.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <limits>
#include <optional>
#include <sstream>
#include <string_view>
#include <unordered_set>

#include <nlohmann/json.hpp>

namespace invisible_places::serialization {

namespace {

using nlohmann::json;
using invisible_places::camera::AnimationExportSettings;
using invisible_places::camera::AnimationPath;
using invisible_places::camera::AnimationPathKey;
using invisible_places::camera::CameraShot;
using invisible_places::camera::CameraState;
using invisible_places::output::AnimationExportMode;
using invisible_places::output::ExportPreset;
using invisible_places::output::RenderJobSettings;
using invisible_places::renderer::gsplat::GaussianSplatColorMode;
using invisible_places::renderer::gsplat::GaussianSplatDebugMode;
using invisible_places::renderer::gsplat::GaussianSplatQualityMode;
using invisible_places::renderer::gsplat::GaussianSplatStyleState;
using invisible_places::renderer::pointcloud::PointCloudColorMode;
using invisible_places::renderer::pointcloud::PointCloudColormapId;
using invisible_places::renderer::pointcloud::PointCloudFalloffProfile;
using invisible_places::renderer::pointcloud::PointCloudGeometryMode;
using invisible_places::renderer::pointcloud::PointCloudHeightFoamShorelineSettings;
using invisible_places::renderer::pointcloud::PointCloudNprPreset;
using invisible_places::renderer::pointcloud::PointCloudPreviewLodMode;
using invisible_places::renderer::pointcloud::PointCloudRendererMode;
using invisible_places::renderer::pointcloud::PointCloudScreenSpriteSizeMode;
using invisible_places::renderer::pointcloud::PointCloudStyleState;
using invisible_places::renderer::pointcloud::PointCloudStylisationMode;
using invisible_places::renderer::pointcloud::PointCloudShorelineWaveAlgorithm;
using invisible_places::style::FieldMapConfig;
using invisible_places::style::ParameterSourceMode;
using invisible_places::style::RenderParameterBinding;
using invisible_places::water::WaterBakeSettings;
using invisible_places::water::WaterCausticLookSettings;
using invisible_places::water::WaterEffectBlendMode;
using invisible_places::water::WaterEffectFeatureType;
using invisible_places::water::WaterEffectLayer;
using invisible_places::water::WaterEffectResponseSettings;
using invisible_places::water::WaterEmitter;
using invisible_places::water::WaterEmitterOrigin;
using invisible_places::water::WaterEmitterStatus;
using invisible_places::water::WaterManualFlowPathSource;
using invisible_places::water::WaterAnimationTrailSettings;
using invisible_places::water::WaterFieldOutputMode;
using invisible_places::water::WaterFieldSettings;
using invisible_places::water::WaterFieldTrailSettings;
using invisible_places::water::WaterFlowTrailSettings;
using invisible_places::water::WaterDynamicMeshAttractor;
using invisible_places::water::WaterDynamicMeshFlowSettings;
using invisible_places::water::WaterParticleTrailSettings;
using invisible_places::water::WaterParticleTrailShapeSettings;
using invisible_places::water::WaterParticleVisualSettings;
using invisible_places::water::WaterPathAutoTuneDiagnostics;
using invisible_places::water::WaterPathBranch;
using invisible_places::water::WaterPathAnalysisCache;
using invisible_places::water::WaterPathAnalysisSample;
using invisible_places::water::WaterPathBranchAnalysis;
using invisible_places::water::WaterPathBranchRole;
using invisible_places::water::WaterPathCache;
using invisible_places::water::WaterPathGenerationSettings;
using invisible_places::water::WaterPathTerminationReason;
using invisible_places::water::WaterRenderSettings;
using invisible_places::water::RainRuntimeSettings;
using invisible_places::water::WaterRainVisualSettings;
using invisible_places::water::WaterRippleOverlayType;
using invisible_places::water::WaterRipplePatternSettings;
using invisible_places::water::WaterRippleRuntimeMembership;
using invisible_places::water::WaterRippleRuntimeParams;
using invisible_places::water::WaterScaleMode;
using invisible_places::water::WaterSeepageLookProfile;
using invisible_places::water::WaterSeepageLookSettings;
using invisible_places::water::WaterSeepageNode;
using invisible_places::water::WaterSeepageNodeAnimationState;
using invisible_places::water::WaterSeepageNodeKey;
using invisible_places::water::WaterSeepageNodeTrack;
using invisible_places::water::WaterSeepagePattern;
using invisible_places::water::WaterSeepageQuality;
using invisible_places::water::WaterScenarioDefinition;
using invisible_places::water::WaterScenarioInterpolation;
using invisible_places::water::WaterScenarioKey;
using invisible_places::water::WaterScenarioState;
using invisible_places::water::WaterScenarioTrack;
using invisible_places::water::WaterSettingsBundle;
using invisible_places::water::WaterSourceSettingsAssignment;
using invisible_places::water::WaterSourceSettings;
using invisible_places::water::WaterTrailGeometrySettings;
using invisible_places::water::WaterVisualSettings;

constexpr std::uintmax_t kLargeJsonRippleCacheStripBytes = 32ULL * 1024ULL * 1024ULL;
constexpr std::array<char, 8> kWaterPathCacheSidecarMagic{'I', 'P', 'F', 'L', 'O', 'W', 'C', '1'};
constexpr std::uint32_t kManualFlowSurfaceGuideProjectSchemaVersion = 40U;
constexpr std::uint32_t kManualFlowSurfaceGuideSourcesSchemaVersion = 16U;

constexpr std::string_view kProjectVisualEditedSuffix = "_edited";
constexpr std::string_view kProjectVisualLegacyEditedSuffix = "_Edited";

json SerializeWaterSettingsBundle(const WaterSettingsBundle& settings);
WaterSettingsBundle ParseWaterSettingsBundle(const json& settingsJson);
json SerializeWaterSourceSettings(const WaterSourceSettings& settings);
WaterSourceSettings ParseWaterSourceSettings(const json& settingsJson);
json SerializeWaterAnimationTrailSettings(const WaterAnimationTrailSettings& settings);
WaterAnimationTrailSettings ParseWaterAnimationTrailSettings(const json& settingsJson);
json SerializeWaterTrailGeometrySettings(const WaterTrailGeometrySettings& settings);
WaterTrailGeometrySettings ParseWaterTrailGeometrySettings(const json& settingsJson);
json SerializeWaterCausticLookSettings(const WaterCausticLookSettings& settings);
WaterCausticLookSettings ParseWaterCausticLookSettings(const json& settingsJson);
json SerializeWaterEffectLayer(const WaterEffectLayer& layer);
WaterEffectLayer ParseWaterEffectLayer(const json& layerJson);
json SerializeWaterRippleRuntimeCache(const WaterRippleRuntimeCacheDocument& cache);
WaterRippleRuntimeCacheDocument ParseWaterRippleRuntimeCache(const json& cacheJson);
json SerializeWaterFlowTrailSettings(const WaterFlowTrailSettings& settings);
WaterFlowTrailSettings ParseWaterFlowTrailSettings(const json& settingsJson);
json SerializeWaterFieldSettings(const WaterFieldSettings& settings);
WaterFieldSettings ParseWaterFieldSettings(const json& settingsJson);
json SerializeWaterFieldTrailSettings(const WaterFieldTrailSettings& settings);
WaterFieldTrailSettings ParseWaterFieldTrailSettings(const json& settingsJson);
json SerializeWaterDynamicMeshFlowSettings(WaterDynamicMeshFlowSettings settings);
WaterDynamicMeshFlowSettings ParseWaterDynamicMeshFlowSettings(const json& settingsJson);
json SerializeWaterRainSettings(
    const RainRuntimeSettings& settings,
    const WaterRainVisualSettings& visual);
RainRuntimeSettings ParseWaterRainSettings(const json& settingsJson);
WaterRainVisualSettings ParseWaterRainVisualSettings(const json& settingsJson);
json SerializeWaterPathCache(const WaterPathCache& cache);
WaterPathCache ParseWaterPathCache(const json& cacheJson);
json SerializeWaterVisualSettings(const WaterVisualSettings& settings);
WaterVisualSettings ParseWaterVisualSettings(const json& settingsJson);
PointCloudStyleState MakeLegacyWaterPointVisualStyle(const WaterVisualSettings& visualSettings);

json SerializeWaterPathCacheManifest(const WaterPathCacheManifestDocument& manifest) {
    return json{
        {"relative_path", manifest.relativePath.generic_string()},
        {"cache_schema", manifest.cacheSchema},
        {"support_signature", manifest.supportSignature},
        {"emitter_settings_fingerprint", manifest.emitterSettingsFingerprint},
        {"payload_bytes", manifest.payloadBytes},
        {"checksum", manifest.checksum},
    };
}

WaterPathCacheManifestDocument ParseWaterPathCacheManifest(const json& manifestJson) {
    WaterPathCacheManifestDocument manifest;
    manifest.relativePath = manifestJson.value("relative_path", std::string{});
    manifest.cacheSchema = manifestJson.value("cache_schema", manifest.cacheSchema);
    manifest.supportSignature = manifestJson.value("support_signature", std::string{});
    manifest.emitterSettingsFingerprint =
        manifestJson.value("emitter_settings_fingerprint", std::string{});
    manifest.payloadBytes = manifestJson.value("payload_bytes", 0ULL);
    if (manifestJson.contains("checksum") && manifestJson.at("checksum").is_array() &&
        manifestJson.at("checksum").size() == manifest.checksum.size()) {
        try {
            manifest.checksum =
                manifestJson.at("checksum").get<std::array<std::uint64_t, 4>>();
        } catch (const json::exception&) {
            // A malformed derived-cache checksum must not make the authored
            // project unreadable. The zero checksum below will fail normal
            // sidecar validation and the manifest will be retired on save.
            manifest.checksum = {};
        }
    }
    return manifest;
}

json SerializeWaterSurfaceCacheManifest(const WaterSurfaceCacheManifestDocument& manifest) {
    return json{
        {"relative_path", manifest.relativePath.generic_string()},
        {"cache_schema", manifest.cacheSchema},
        {"algorithm_id", manifest.algorithmId},
        {"source_fingerprint", manifest.sourceFingerprint},
        {"payload_bytes", manifest.payloadBytes},
        {"checksum", manifest.checksum},
        {"requested_rebuild_generation", manifest.requestedRebuildGeneration},
        {"built_rebuild_generation", manifest.builtRebuildGeneration},
    };
}

WaterSurfaceCacheManifestDocument ParseWaterSurfaceCacheManifest(const json& manifestJson) {
    WaterSurfaceCacheManifestDocument manifest;
    manifest.relativePath = manifestJson.value("relative_path", std::string{});
    manifest.cacheSchema = manifestJson.value("cache_schema", manifest.cacheSchema);
    manifest.algorithmId = manifestJson.value("algorithm_id", manifest.algorithmId);
    manifest.sourceFingerprint = manifestJson.value("source_fingerprint", std::string{});
    manifest.payloadBytes = manifestJson.value("payload_bytes", 0ULL);
    if (manifestJson.contains("checksum") && manifestJson.at("checksum").is_array() &&
        manifestJson.at("checksum").size() == manifest.checksum.size()) {
        try {
            manifest.checksum =
                manifestJson.at("checksum").get<std::array<std::uint64_t, 4>>();
        } catch (const json::exception&) {
            manifest.checksum = {};
        }
    }
    manifest.requestedRebuildGeneration =
        manifestJson.value("requested_rebuild_generation", 0ULL);
    manifest.builtRebuildGeneration =
        manifestJson.value("built_rebuild_generation", 0ULL);
    return manifest;
}

enum class LegacyPointCloudRenderMode {
    Solid,
    EmissiveHard,
    EmissiveFeathered,
    GaussianPointSprite
};

const char* ParameterSourceModeName(ParameterSourceMode mode) {
    switch (mode) {
        case ParameterSourceMode::Constant:
            return "constant";
        case ParameterSourceMode::FieldMapped:
            return "field_mapped";
    }

    return "constant";
}

bool IsJsonWhitespace(char character) {
    return std::isspace(static_cast<unsigned char>(character)) != 0;
}

std::size_t SkipJsonWhitespace(const std::string& text, std::size_t offset) {
    while (offset < text.size() && IsJsonWhitespace(text[offset])) {
        ++offset;
    }
    return offset;
}

std::optional<std::size_t> FindJsonArrayEnd(const std::string& text, std::size_t arrayStart) {
    if (arrayStart >= text.size() || text[arrayStart] != '[') {
        return std::nullopt;
    }

    std::size_t depth = 0U;
    bool inString = false;
    bool escaped = false;
    for (std::size_t offset = arrayStart; offset < text.size(); ++offset) {
        const char character = text[offset];
        if (inString) {
            if (escaped) {
                escaped = false;
            } else if (character == '\\') {
                escaped = true;
            } else if (character == '"') {
                inString = false;
            }
            continue;
        }

        if (character == '"') {
            inString = true;
        } else if (character == '[') {
            ++depth;
        } else if (character == ']') {
            if (depth == 0U) {
                return std::nullopt;
            }
            --depth;
            if (depth == 0U) {
                return offset;
            }
        }
    }
    return std::nullopt;
}

bool StripJsonArrayPropertyValue(std::string* text, std::string_view propertyName) {
    if (text == nullptr || text->empty()) {
        return false;
    }

    const std::string keyToken = "\"" + std::string{propertyName} + "\"";
    bool stripped = false;
    std::size_t searchOffset = 0U;
    while (searchOffset < text->size()) {
        const auto keyOffset = text->find(keyToken, searchOffset);
        if (keyOffset == std::string::npos) {
            break;
        }

        const auto colonOffset = SkipJsonWhitespace(*text, keyOffset + keyToken.size());
        if (colonOffset >= text->size() || (*text)[colonOffset] != ':') {
            searchOffset = keyOffset + keyToken.size();
            continue;
        }
        const auto arrayStart = SkipJsonWhitespace(*text, colonOffset + 1U);
        if (arrayStart >= text->size() || (*text)[arrayStart] != '[') {
            searchOffset = keyOffset + keyToken.size();
            continue;
        }
        const auto arrayEnd = FindJsonArrayEnd(*text, arrayStart);
        if (!arrayEnd.has_value()) {
            searchOffset = keyOffset + keyToken.size();
            continue;
        }

        text->replace(arrayStart, arrayEnd.value() - arrayStart + 1U, "[]");
        searchOffset = arrayStart + 2U;
        stripped = true;
    }
    return stripped;
}

bool ShouldSerializeWaterRippleRuntimeCache(const WaterRippleRuntimeCacheDocument& cache) {
    return !cache.stale &&
           !cache.memberships.empty() &&
           !cache.params.empty() &&
           cache.memberships.size() <= kMaxSerializedWaterRippleRuntimeCacheMemberships;
}

ParameterSourceMode ParseParameterSourceMode(const json& value) {
    const auto modeName = value.get<std::string>();
    return modeName == "field_mapped" ? ParameterSourceMode::FieldMapped : ParameterSourceMode::Constant;
}

const char* PointCloudColorModeName(PointCloudColorMode mode) {
    switch (mode) {
        case PointCloudColorMode::SourceRgb:
            return "source_rgb";
        case PointCloudColorMode::SolidColor:
            return "solid_color";
        case PointCloudColorMode::ScalarColormap:
            return "scalar_colormap";
    }

    return "source_rgb";
}

PointCloudColorMode ParsePointCloudColorMode(const json& value) {
    const auto modeName = value.get<std::string>();
    if (modeName == "solid_color") {
        return PointCloudColorMode::SolidColor;
    }
    if (modeName == "scalar_colormap") {
        return PointCloudColorMode::ScalarColormap;
    }
    return PointCloudColorMode::SourceRgb;
}

const char* PointCloudColormapName(PointCloudColormapId colormap) {
    switch (colormap) {
        case PointCloudColormapId::Viridis:
            return "viridis";
        case PointCloudColormapId::Plasma:
            return "plasma";
        case PointCloudColormapId::Inferno:
            return "inferno";
        case PointCloudColormapId::Magma:
            return "magma";
        case PointCloudColormapId::Cividis:
            return "cividis";
        case PointCloudColormapId::Turbo:
            return "turbo";
        case PointCloudColormapId::Topographic:
            return "topographic";
        case PointCloudColormapId::LandSurface:
            return "land_surface";
        case PointCloudColormapId::ExponentialFire:
            return "exponential_fire";
        case PointCloudColormapId::ExponentialIce:
            return "exponential_ice";
        case PointCloudColormapId::HighContrast:
            return "high_contrast";
        case PointCloudColormapId::CustomGradient:
            return "custom_gradient";
    }

    return "viridis";
}

PointCloudColormapId ParsePointCloudColormap(const json& value) {
    const auto colormapName = value.get<std::string>();
    if (colormapName == "plasma") {
        return PointCloudColormapId::Plasma;
    }
    if (colormapName == "inferno") {
        return PointCloudColormapId::Inferno;
    }
    if (colormapName == "magma") {
        return PointCloudColormapId::Magma;
    }
    if (colormapName == "cividis") {
        return PointCloudColormapId::Cividis;
    }
    if (colormapName == "turbo") {
        return PointCloudColormapId::Turbo;
    }
    if (colormapName == "topographic" || colormapName == "topo") {
        return PointCloudColormapId::Topographic;
    }
    if (colormapName == "land_surface" || colormapName == "landsurf") {
        return PointCloudColormapId::LandSurface;
    }
    if (colormapName == "exponential_fire" || colormapName == "exp_fire") {
        return PointCloudColormapId::ExponentialFire;
    }
    if (colormapName == "exponential_ice" || colormapName == "exp_ice") {
        return PointCloudColormapId::ExponentialIce;
    }
    if (colormapName == "high_contrast") {
        return PointCloudColormapId::HighContrast;
    }
    if (colormapName == "custom_gradient" || colormapName == "custom") {
        return PointCloudColormapId::CustomGradient;
    }
    return PointCloudColormapId::Viridis;
}

const char* PointCloudPreviewLodModeName(PointCloudPreviewLodMode mode) {
    switch (mode) {
        case PointCloudPreviewLodMode::FullResolution:
            return "full_resolution";
        case PointCloudPreviewLodMode::AutoCameraLod:
            return "auto_camera_lod";
        case PointCloudPreviewLodMode::ForceLod:
            return "force_lod";
    }

    return "auto_camera_lod";
}

PointCloudPreviewLodMode ParsePointCloudPreviewLodMode(const json& value) {
    const auto modeName = value.get<std::string>();
    if (modeName == "full_resolution") {
        return PointCloudPreviewLodMode::FullResolution;
    }
    if (modeName == "force_lod") {
        return PointCloudPreviewLodMode::ForceLod;
    }
    return PointCloudPreviewLodMode::AutoCameraLod;
}

const char* PointCloudRendererModeName(PointCloudRendererMode mode) {
    switch (mode) {
        case PointCloudRendererMode::Beauty:
            return "beauty";
        case PointCloudRendererMode::FastBasic:
            return "fast_basic";
    }

    return "beauty";
}

PointCloudRendererMode ParsePointCloudRendererMode(const json& value) {
    const auto modeName = value.get<std::string>();
    if (modeName == "fast_basic") {
        return PointCloudRendererMode::FastBasic;
    }
    return PointCloudRendererMode::Beauty;
}

const char* PointCloudGeometryModeName(PointCloudGeometryMode mode) {
    switch (mode) {
        case PointCloudGeometryMode::ScreenSprites:
            return "screen_sprites";
        case PointCloudGeometryMode::WorldSurfels:
            return "world_surfels";
        case PointCloudGeometryMode::CameraFacingWorldSprites:
            return "camera_facing_world_sprites";
    }

    return "screen_sprites";
}

PointCloudGeometryMode ParsePointCloudGeometryMode(const json& value) {
    const auto modeName = value.get<std::string>();
    if (modeName == "world_surfels") {
        return PointCloudGeometryMode::WorldSurfels;
    }
    if (modeName == "camera_facing_world_sprites" ||
        modeName == "camera_facing_world_surfels" ||
        modeName == "world_sprites") {
        return PointCloudGeometryMode::CameraFacingWorldSprites;
    }
    return PointCloudGeometryMode::ScreenSprites;
}

const char* PointCloudScreenSpriteSizeModeName(PointCloudScreenSpriteSizeMode mode) {
    switch (mode) {
        case PointCloudScreenSpriteSizeMode::Pixels:
            return "pixels";
        case PointCloudScreenSpriteSizeMode::WorldMillimeters:
            return "world_millimeters";
    }

    return "pixels";
}

PointCloudScreenSpriteSizeMode ParsePointCloudScreenSpriteSizeMode(const json& value) {
    const auto modeName = value.get<std::string>();
    if (modeName == "world_millimeters" || modeName == "world_mm" || modeName == "millimeters" ||
        modeName == "mm") {
        return PointCloudScreenSpriteSizeMode::WorldMillimeters;
    }
    return PointCloudScreenSpriteSizeMode::Pixels;
}

LegacyPointCloudRenderMode ParseLegacyPointCloudRenderMode(const json& value) {
    const auto modeName = value.get<std::string>();
    if (modeName == "emissive_hard") {
        return LegacyPointCloudRenderMode::EmissiveHard;
    }
    if (modeName == "emissive_feathered") {
        return LegacyPointCloudRenderMode::EmissiveFeathered;
    }
    if (modeName == "gaussian_point_sprite") {
        return LegacyPointCloudRenderMode::GaussianPointSprite;
    }
    return LegacyPointCloudRenderMode::Solid;
}

const char* PointCloudFalloffProfileName(PointCloudFalloffProfile profile) {
    switch (profile) {
        case PointCloudFalloffProfile::HardDisc:
            return "hard_disc";
        case PointCloudFalloffProfile::SoftDisc:
            return "soft_disc";
        case PointCloudFalloffProfile::Gaussian:
            return "gaussian";
        case PointCloudFalloffProfile::Rim:
            return "rim";
    }

    return "soft_disc";
}

PointCloudFalloffProfile ParsePointCloudFalloffProfile(const json& value) {
    const auto profileName = value.get<std::string>();
    if (profileName == "hard_disc") {
        return PointCloudFalloffProfile::HardDisc;
    }
    if (profileName == "gaussian") {
        return PointCloudFalloffProfile::Gaussian;
    }
    if (profileName == "rim") {
        return PointCloudFalloffProfile::Rim;
    }
    return PointCloudFalloffProfile::SoftDisc;
}

const char* PointCloudStylisationModeName(PointCloudStylisationMode mode) {
    switch (mode) {
        case PointCloudStylisationMode::Off:
            return "off";
        case PointCloudStylisationMode::NprStylisation:
            return "npr_stylisation";
        case PointCloudStylisationMode::BrushParticles:
            return "brush_particles";
    }

    return "off";
}

PointCloudStylisationMode ParsePointCloudStylisationMode(const json& value) {
    const auto modeName = value.get<std::string>();
    if (modeName == "npr_stylisation" || modeName == "npr_stylization") {
        return PointCloudStylisationMode::NprStylisation;
    }
    if (modeName == "brush_particles") {
        return PointCloudStylisationMode::BrushParticles;
    }
    return PointCloudStylisationMode::Off;
}

const char* PointCloudNprPresetName(PointCloudNprPreset preset) {
    switch (preset) {
        case PointCloudNprPreset::Watercolor:
            return "watercolor";
        case PointCloudNprPreset::Cartoon:
            return "cartoon";
    }

    return "watercolor";
}

PointCloudNprPreset ParsePointCloudNprPreset(const json& value) {
    const auto presetName = value.get<std::string>();
    if (presetName == "cartoon") {
        return PointCloudNprPreset::Cartoon;
    }
    return PointCloudNprPreset::Watercolor;
}

const char* PointCloudShorelineWaveAlgorithmName(PointCloudShorelineWaveAlgorithm algorithm) {
    switch (algorithm) {
        case PointCloudShorelineWaveAlgorithm::FoamFronts:
            return "foam_fronts";
        case PointCloudShorelineWaveAlgorithm::HeightFoam:
            return "height_foam";
    }
    return "foam_fronts";
}

PointCloudShorelineWaveAlgorithm ParsePointCloudShorelineWaveAlgorithm(const json& value) {
    if (value.is_string() && value.get<std::string>() == "height_foam") {
        return PointCloudShorelineWaveAlgorithm::HeightFoam;
    }
    return PointCloudShorelineWaveAlgorithm::FoamFronts;
}

json SerializeHeightFoamShorelineSettings(const PointCloudHeightFoamShorelineSettings& settings) {
    return json{
        {"runup_z", settings.runupZ},
        {"break_z", settings.breakZ},
        {"offshore_reach_meters", settings.offshoreReachMeters},
        {"edge_fade_meters", settings.edgeFadeMeters},
        {"direction", std::array<float, 2>{settings.directionX, settings.directionY}},
        {"pattern_scale", settings.patternScale},
        {"wavelength_meters", settings.wavelengthMeters},
        {"speed", settings.speed},
        {"warp", settings.warp},
        {"turbulence", settings.turbulence},
        {"density", settings.density},
        {"phase", settings.phase},
        {"intensity", settings.intensity},
        {"offshore_foam_strength", settings.offshoreFoamStrength},
        {"incoming_strength", settings.incomingStrength},
        {"return_strength", settings.returnStrength},
        {"emission_add", settings.emissionAdd},
        {"opacity_add", settings.opacityAdd},
        {"opacity_multiply", settings.opacityMultiply},
        {"point_size_add", settings.pointSizeAdd},
        {"point_size_multiply", settings.pointSizeMultiply},
        {"colour_mix", settings.colourMix},
        {"colour", settings.colour},
        {"seed", settings.seed},
    };
}

PointCloudHeightFoamShorelineSettings ParseHeightFoamShorelineSettings(const json& settingsJson) {
    PointCloudHeightFoamShorelineSettings settings;
    settings.runupZ = std::clamp(settingsJson.value("runup_z", settings.runupZ), -1000.0F, 1000.0F);
    settings.breakZ = settingsJson.value("break_z", settings.breakZ);
    settings.offshoreReachMeters = std::clamp(
        settingsJson.value("offshore_reach_meters", settings.offshoreReachMeters),
        0.001F,
        50.0F);
    settings.edgeFadeMeters = std::clamp(
        settingsJson.value("edge_fade_meters", settings.edgeFadeMeters),
        0.0F,
        10.0F);
    if (settingsJson.contains("direction")) {
        const auto direction = settingsJson.at("direction").get<std::array<float, 2>>();
        settings.directionX = direction[0];
        settings.directionY = direction[1];
    }
    settings.patternScale = std::clamp(
        settingsJson.value("pattern_scale", settings.patternScale),
        0.01F,
        50.0F);
    settings.wavelengthMeters = std::clamp(
        settingsJson.value("wavelength_meters", settings.wavelengthMeters),
        0.002F,
        10.0F);
    settings.speed = std::clamp(settingsJson.value("speed", settings.speed), 0.0F, 10.0F);
    settings.warp = std::clamp(settingsJson.value("warp", settings.warp), 0.0F, 3.0F);
    settings.turbulence = std::clamp(
        settingsJson.value("turbulence", settings.turbulence),
        0.0F,
        1.0F);
    settings.density = std::clamp(settingsJson.value("density", settings.density), 0.0F, 1.0F);
    settings.phase = settingsJson.value("phase", settings.phase);
    settings.intensity = std::clamp(settingsJson.value("intensity", settings.intensity), 0.0F, 5.0F);
    settings.offshoreFoamStrength = std::clamp(
        settingsJson.value("offshore_foam_strength", settings.offshoreFoamStrength),
        0.0F,
        3.0F);
    settings.incomingStrength = std::clamp(
        settingsJson.value("incoming_strength", settings.incomingStrength),
        0.0F,
        5.0F);
    settings.returnStrength = std::clamp(
        settingsJson.value("return_strength", settings.returnStrength),
        0.0F,
        1.0F);
    settings.emissionAdd = std::clamp(
        settingsJson.value("emission_add", settings.emissionAdd),
        0.0F,
        8.0F);
    settings.opacityAdd = std::clamp(
        settingsJson.value("opacity_add", settings.opacityAdd),
        -1.0F,
        2.0F);
    settings.opacityMultiply = std::clamp(
        settingsJson.value("opacity_multiply", settings.opacityMultiply),
        0.0F,
        8.0F);
    settings.pointSizeAdd = std::clamp(
        settingsJson.value("point_size_add", settings.pointSizeAdd),
        -256.0F,
        512.0F);
    settings.pointSizeMultiply = std::clamp(
        settingsJson.value("point_size_multiply", settings.pointSizeMultiply),
        0.0F,
        8.0F);
    settings.colourMix = std::clamp(settingsJson.value("colour_mix", settings.colourMix), 0.0F, 1.0F);
    if (settingsJson.contains("colour")) {
        settings.colour = settingsJson.at("colour").get<std::array<float, 3>>();
    }
    settings.seed = settingsJson.value("seed", settings.seed);
    settings.breakZ = invisible_places::renderer::pointcloud::NormalizeHeightFoamBreakZ(
        settings.runupZ,
        settings.offshoreReachMeters,
        settings.edgeFadeMeters,
        settings.breakZ);
    return settings;
}

const char* SerializedLayerKindName(SerializedLayerKind kind) {
    return kind == SerializedLayerKind::GaussianSplat ? "gsplat" : "point_cloud";
}

SerializedLayerKind ParseSerializedLayerKind(const json& value) {
    return value.get<std::string>() == "gsplat" ? SerializedLayerKind::GaussianSplat
                                                 : SerializedLayerKind::PointCloud;
}

const char* GaussianSplatColorModeName(GaussianSplatColorMode mode) {
    switch (mode) {
        case GaussianSplatColorMode::FullSh:
            return "full_sh";
        case GaussianSplatColorMode::DcOnly:
            return "dc_only";
    }

    return "full_sh";
}

GaussianSplatColorMode ParseGaussianSplatColorMode(const json& value) {
    const auto modeName = value.get<std::string>();
    if (modeName == "dc_only" || modeName == "dc") {
        return GaussianSplatColorMode::DcOnly;
    }
    return GaussianSplatColorMode::FullSh;
}

const char* GaussianSplatDebugModeName(GaussianSplatDebugMode mode) {
    switch (mode) {
        case GaussianSplatDebugMode::Final:
            return "final";
        case GaussianSplatDebugMode::Opacity:
            return "opacity";
        case GaussianSplatDebugMode::Scale:
            return "scale";
        case GaussianSplatDebugMode::Depth:
            return "depth";
        case GaussianSplatDebugMode::LayerTint:
            return "layer_tint";
    }

    return "final";
}

GaussianSplatDebugMode ParseGaussianSplatDebugMode(const json& value) {
    const auto modeName = value.get<std::string>();
    if (modeName == "opacity") {
        return GaussianSplatDebugMode::Opacity;
    }
    if (modeName == "scale") {
        return GaussianSplatDebugMode::Scale;
    }
    if (modeName == "depth") {
        return GaussianSplatDebugMode::Depth;
    }
    if (modeName == "layer_tint") {
        return GaussianSplatDebugMode::LayerTint;
    }
    return GaussianSplatDebugMode::Final;
}

const char* GaussianSplatQualityModeName(GaussianSplatQualityMode mode) {
    switch (mode) {
        case GaussianSplatQualityMode::Fast:
            return "fast";
        case GaussianSplatQualityMode::Medium:
            return "medium";
        case GaussianSplatQualityMode::SurfaceGuided:
            return "surface_guided";
        case GaussianSplatQualityMode::High:
            return "high";
    }

    return "fast";
}

GaussianSplatQualityMode ParseGaussianSplatQualityMode(const json& value) {
    const auto modeName = value.get<std::string>();
    if (modeName == "medium") {
        return GaussianSplatQualityMode::Medium;
    }
    if (modeName == "surface_guided" || modeName == "surface") {
        return GaussianSplatQualityMode::SurfaceGuided;
    }
    if (modeName == "high") {
        return GaussianSplatQualityMode::High;
    }
    return GaussianSplatQualityMode::Fast;
}

json SerializeBinding(const RenderParameterBinding& binding) {
    return json{
        {"active", binding.active},
        {"mode", ParameterSourceModeName(binding.mode)},
        {"constant_value", binding.constantValue},
        {"field_map",
         {
             {"field_slot", binding.fieldMap.fieldSlot},
             {"field_name", binding.fieldMap.fieldName},
             {"input_min", binding.fieldMap.inputMin},
             {"input_max", binding.fieldMap.inputMax},
             {"output_min", binding.fieldMap.outputMin},
             {"output_max", binding.fieldMap.outputMax},
             {"gamma", binding.fieldMap.gamma},
             {"flags", binding.fieldMap.flags},
         }},
    };
}

RenderParameterBinding ParseBinding(const json& bindingJson) {
    RenderParameterBinding binding;

    binding.active = bindingJson.value("active", true);
    if (bindingJson.contains("mode")) {
        binding.mode = ParseParameterSourceMode(bindingJson.at("mode"));
    }
    if (bindingJson.contains("constant_value")) {
        binding.constantValue = bindingJson.at("constant_value").get<std::array<float, 4>>();
    }

    if (bindingJson.contains("field_map")) {
        const auto& fieldMapJson = bindingJson.at("field_map");
        binding.fieldMap.fieldSlot = fieldMapJson.value("field_slot", -1);
        binding.fieldMap.fieldName = fieldMapJson.value("field_name", std::string{});
        binding.fieldMap.inputMin = fieldMapJson.value("input_min", 0.0F);
        binding.fieldMap.inputMax = fieldMapJson.value("input_max", 1.0F);
        binding.fieldMap.outputMin = fieldMapJson.value("output_min", 0.0F);
        binding.fieldMap.outputMax = fieldMapJson.value("output_max", 1.0F);
        binding.fieldMap.gamma = fieldMapJson.value("gamma", 1.0F);
        binding.fieldMap.flags = fieldMapJson.value(
            "flags",
            static_cast<std::uint32_t>(
                invisible_places::style::FieldMapFlagClamp |
                invisible_places::style::FieldMapFlagUseLayerStats));
    }

    return binding;
}

void MigrateLegacyPointCloudRenderMode(
    LegacyPointCloudRenderMode mode,
    bool hadEmissiveStrength,
    PointCloudStyleState* style) {
    if (style == nullptr) {
        return;
    }

    switch (mode) {
        case LegacyPointCloudRenderMode::Solid:
            break;
        case LegacyPointCloudRenderMode::EmissiveHard:
            style->falloffProfile = PointCloudFalloffProfile::HardDisc;
            if (!hadEmissiveStrength && invisible_places::style::ScalarConstant(style->emissiveStrength) <= 0.0F) {
                invisible_places::style::SetScalarConstant(&style->emissiveStrength, 1.0F);
            }
            break;
        case LegacyPointCloudRenderMode::EmissiveFeathered:
            style->falloffProfile = PointCloudFalloffProfile::Gaussian;
            if (!hadEmissiveStrength && invisible_places::style::ScalarConstant(style->emissiveStrength) <= 0.0F) {
                invisible_places::style::SetScalarConstant(&style->emissiveStrength, 1.0F);
            }
            break;
        case LegacyPointCloudRenderMode::GaussianPointSprite:
            style->falloffProfile = PointCloudFalloffProfile::Gaussian;
            break;
    }
}

PointCloudStyleState ParsePointCloudStyle(const json& styleJson);

json SerializePointCloudStyle(const PointCloudStyleState& style) {
    return json{
        {"geometry_mode", PointCloudGeometryModeName(style.geometryMode)},
        {"screen_sprite_size_mode", PointCloudScreenSpriteSizeModeName(style.screenSpriteSizeMode)},
        {"falloff_profile", PointCloudFalloffProfileName(style.falloffProfile)},
        {"stylisation_mode", PointCloudStylisationModeName(style.stylisationMode)},
        {"npr_preset", PointCloudNprPresetName(style.nprPreset)},
        {"color_mode", PointCloudColorModeName(style.colorMode)},
        {"colormap", PointCloudColormapName(style.colormap)},
        {"solid_color", style.solidColor},
        {"gradient_start_color", style.gradientStartColor},
        {"gradient_end_color", style.gradientEndColor},
        {"colorize_color", style.colorizeColor},
        {"colorize_amount", style.colorizeAmount},
        {"stylisation_strength", style.stylisationStrength},
        {"stylisation_color_levels", style.stylisationColorLevels},
        {"stylisation_ink_strength", style.stylisationInkStrength},
        {"stylisation_paper_grain", style.stylisationPaperGrain},
        {"stylisation_pigment_bleed", style.stylisationPigmentBleed},
        {"brush_aspect", style.brushAspect},
        {"stroke_jitter", style.strokeJitter},
        {"hatch_strength", style.hatchStrength},
        {"stroke_opacity_variance", style.strokeOpacityVariance},
        {"pigment_variation", style.pigmentVariation},
        {"pigment_animation_speed", style.pigmentAnimationSpeed},
        {"granulation_angle_strength", style.granulationAngleStrength},
        {"roughness_motion_strength", style.roughnessMotionStrength},
        {"roughness_motion_scale", style.roughnessMotionScale},
        {"roughness_motion_speed", style.roughnessMotionSpeed},
        {"roughness_motion_threshold", style.roughnessMotionThreshold},
        {"roughness_motion_ground_id", style.roughnessMotionGroundId},
        {"caustic_animation", style.causticAnimation},
        {"caustic_intensity", style.causticIntensity},
        {"caustic_scale", style.causticScale},
        {"caustic_speed", style.causticSpeed},
        {"caustic_line_sharpness", style.causticLineSharpness},
        {"caustic_warp", style.causticWarp},
        {"caustic_cell_size_meters", style.causticCellSizeMeters},
        {"caustic_line_width_meters", style.causticLineWidthMeters},
        {"caustic_feather_meters", style.causticFeatherMeters},
        {"caustic_surface_point_spacing_meters", style.causticSurfacePointSpacingMeters},
        {"caustic_warp_amplitude_meters", style.causticWarpAmplitudeMeters},
        {"caustic_tint", style.causticTint},
        {"caustic_emission_boost", style.causticEmissionBoost},
        {"caustic_opacity_boost", style.causticOpacityBoost},
        {"caustic_point_size_boost", style.causticPointSizeBoost},
        {"caustic_mask_field_slot", style.causticMaskFieldSlot},
        {"caustic_edge_field_slot", style.causticEdgeFieldSlot},
        {"caustic_seed_field_slot", style.causticSeedFieldSlot},
        {"shoreline_wave_enabled", style.shorelineWaveEnabled},
        {"shoreline_wave_algorithm", PointCloudShorelineWaveAlgorithmName(style.shorelineWaveAlgorithm)},
        {"shoreline_height_foam", SerializeHeightFoamShorelineSettings(style.shorelineHeightFoam)},
        {"shoreline_boundary_z", style.shorelineBoundaryZ},
        {"shoreline_height_reach_meters", style.shorelineHeightReachMeters},
        {"shoreline_edge_fade_meters", style.shorelineEdgeFadeMeters},
        {"shoreline_direction", std::array<float, 2>{style.shorelineDirectionX, style.shorelineDirectionY}},
        {"shoreline_pattern_scale", style.shorelinePatternScale},
        {"shoreline_wavelength_meters", style.shorelineWavelengthMeters},
        {"shoreline_speed", style.shorelineSpeed},
        {"shoreline_warp", style.shorelineWarp},
        {"shoreline_turbulence", style.shorelineTurbulence},
        {"shoreline_density", style.shorelineDensity},
        {"shoreline_phase", style.shorelinePhase},
        {"shoreline_intensity", style.shorelineIntensity},
        {"shoreline_emission_add", style.shorelineEmissionAdd},
        {"shoreline_opacity_add", style.shorelineOpacityAdd},
        {"shoreline_opacity_multiply", style.shorelineOpacityMultiply},
        {"shoreline_point_size_add", style.shorelinePointSizeAdd},
        {"shoreline_point_size_multiply", style.shorelinePointSizeMultiply},
        {"shoreline_colour_mix", style.shorelineColourMix},
        {"shoreline_colour", style.shorelineColour},
        {"shoreline_seed", style.shorelineSeed},
        {"exposure", style.exposure},
        {"inner_radius", style.innerRadius},
        {"gaussian_sharpness", style.gaussianSharpness},
        {"feather_power", style.featherPower},
        {"water_streak_aspect", style.waterStreakAspect},
        {"solid_centers", style.solidCenters},
        {"flow_animation", style.flowAnimation},
        {"water_path_view", style.waterPathView},
        {"water_trail_overlay", style.waterTrailOverlay},
        {"point_size", SerializeBinding(style.pointSize)},
        {"surfel_diameter", SerializeBinding(style.surfelDiameter)},
        {"opacity", SerializeBinding(style.opacity)},
        {"emissive_strength", SerializeBinding(style.emissiveStrength)},
        {"depth_fade", SerializeBinding(style.depthFade)},
        {"colormap_position", SerializeBinding(style.colormapPosition)},
    };
}

json SerializePointCloudVisual(const ProjectLayerDocument::PointVisual& visual) {
    return json{
        {"name", visual.name.empty() ? std::string{"Unnamed"} : visual.name},
        {"point_style", SerializePointCloudStyle(visual.style)},
    };
}

ProjectLayerDocument::PointVisual ParsePointCloudVisual(const json& visualJson) {
    ProjectLayerDocument::PointVisual visual;
    visual.name = visualJson.value("name", std::string{"Unnamed"});
    if (visual.name.empty()) {
        visual.name = "Unnamed";
    }
    if (visualJson.contains("point_style")) {
        visual.style = ParsePointCloudStyle(visualJson.at("point_style"));
    }
    return visual;
}

std::string TrimAsciiWhitespace(std::string value) {
    const auto first = std::find_if_not(value.begin(), value.end(), IsJsonWhitespace);
    const auto last = std::find_if_not(value.rbegin(), value.rend(), IsJsonWhitespace).base();
    if (first >= last) {
        return {};
    }
    return std::string{first, last};
}

std::string NormalizeProjectPointVisualName(std::string_view name) {
    auto normalized = TrimAsciiWhitespace(std::string{name});
    if (normalized.empty()) {
        normalized = "Unnamed";
    }
    if (normalized.size() > kProjectVisualLegacyEditedSuffix.size() &&
        normalized.ends_with(kProjectVisualLegacyEditedSuffix)) {
        normalized.replace(
            normalized.size() - kProjectVisualLegacyEditedSuffix.size(),
            kProjectVisualLegacyEditedSuffix.size(),
            kProjectVisualEditedSuffix);
    }
    return normalized;
}

bool IsProjectEditedPointVisualName(std::string_view name) {
    const auto normalized = NormalizeProjectPointVisualName(name);
    return normalized.size() > kProjectVisualEditedSuffix.size() &&
           normalized.ends_with(kProjectVisualEditedSuffix);
}

std::string ProjectPointVisualBaseName(std::string_view name) {
    auto normalized = NormalizeProjectPointVisualName(name);
    if (IsProjectEditedPointVisualName(normalized)) {
        normalized.resize(normalized.size() - kProjectVisualEditedSuffix.size());
    }
    return normalized.empty() ? std::string{"Unnamed"} : normalized;
}

std::string ProjectSceneVisualName(std::string_view baseName, std::string_view sceneGroupName) {
    auto base = ProjectPointVisualBaseName(baseName);
    auto scene = TrimAsciiWhitespace(std::string{sceneGroupName});
    if (scene.empty()) {
        return base;
    }
    return base + "_" + scene;
}

std::string LayerSceneVisualKey(const ProjectLayerDocument& layer) {
    if (!layer.sceneGroupName.empty()) {
        return layer.sceneGroupName;
    }
    if (!layer.sourcePath.empty()) {
        return layer.sourcePath.stem().string();
    }
    return {};
}

std::optional<std::size_t> FindPointVisualDocumentIndex(
    const std::vector<ProjectLayerDocument::PointVisual>& visuals,
    std::string_view name) {
    const auto normalized = NormalizeProjectPointVisualName(name);
    for (std::size_t index = 0; index < visuals.size(); ++index) {
        if (NormalizeProjectPointVisualName(visuals[index].name) == normalized) {
            return index;
        }
    }
    return std::nullopt;
}

void UpsertPointVisualDocument(
    std::vector<ProjectLayerDocument::PointVisual>* visuals,
    std::string_view name,
    const PointCloudStyleState& style,
    bool replaceExisting) {
    if (visuals == nullptr) {
        return;
    }

    const auto normalized = NormalizeProjectPointVisualName(name);
    if (const auto existing = FindPointVisualDocumentIndex(*visuals, normalized); existing.has_value()) {
        if (replaceExisting) {
            (*visuals)[existing.value()].name = normalized;
            (*visuals)[existing.value()].style = style;
        }
        return;
    }

    visuals->push_back({.name = normalized, .style = style});
}

std::string NormalizeLegacySceneRole(std::string_view role) {
    std::string normalized;
    normalized.reserve(role.size());
    for (const char character : role) {
        const auto byte = static_cast<unsigned char>(character);
        if (std::isalnum(byte) == 0) {
            continue;
        }
        normalized.push_back(static_cast<char>(std::toupper(byte)));
    }
    return normalized;
}

void CopyLegacyShorelineSettings(PointCloudStyleState* target, const PointCloudStyleState& source) {
    if (target == nullptr) {
        return;
    }
    target->shorelineWaveEnabled = source.shorelineWaveEnabled;
    target->shorelineWaveAlgorithm = source.shorelineWaveAlgorithm;
    target->shorelineHeightFoam = source.shorelineHeightFoam;
    target->shorelineBoundaryZ = source.shorelineBoundaryZ;
    target->shorelineHeightReachMeters = source.shorelineHeightReachMeters;
    target->shorelineEdgeFadeMeters = source.shorelineEdgeFadeMeters;
    target->shorelineDirectionX = source.shorelineDirectionX;
    target->shorelineDirectionY = source.shorelineDirectionY;
    target->shorelinePatternScale = source.shorelinePatternScale;
    target->shorelineWavelengthMeters = source.shorelineWavelengthMeters;
    target->shorelineSpeed = source.shorelineSpeed;
    target->shorelineWarp = source.shorelineWarp;
    target->shorelineTurbulence = source.shorelineTurbulence;
    target->shorelineDensity = source.shorelineDensity;
    target->shorelinePhase = source.shorelinePhase;
    target->shorelineIntensity = source.shorelineIntensity;
    target->shorelineEmissionAdd = source.shorelineEmissionAdd;
    target->shorelineOpacityAdd = source.shorelineOpacityAdd;
    target->shorelineOpacityMultiply = source.shorelineOpacityMultiply;
    target->shorelinePointSizeAdd = source.shorelinePointSizeAdd;
    target->shorelinePointSizeMultiply = source.shorelinePointSizeMultiply;
    target->shorelineColourMix = source.shorelineColourMix;
    target->shorelineColour = source.shorelineColour;
    target->shorelineSeed = source.shorelineSeed;
}

void CopyLegacyRoughnessMotionSettings(PointCloudStyleState* target, const PointCloudStyleState& source) {
    if (target == nullptr) {
        return;
    }
    target->roughnessMotionStrength = source.roughnessMotionStrength;
    target->roughnessMotionScale = source.roughnessMotionScale;
    target->roughnessMotionSpeed = source.roughnessMotionSpeed;
    target->roughnessMotionThreshold = source.roughnessMotionThreshold;
    target->roughnessMotionGroundId = source.roughnessMotionGroundId;
    target->roughnessMotionFullLayer = source.roughnessMotionFullLayer;
}

void MergeLegacySceneRoleSpecificVisualSettings(
    PointCloudStyleState* target,
    const PointCloudStyleState& source,
    std::string_view role) {
    if (target == nullptr) {
        return;
    }
    const auto normalizedRole = NormalizeLegacySceneRole(role);
    if (normalizedRole == "SAND" && source.shorelineWaveEnabled) {
        CopyLegacyShorelineSettings(target, source);
    }
    if (normalizedRole == "VEG" &&
        invisible_places::renderer::pointcloud::PointCloudStyleHasActiveRoughnessMotion(source)) {
        CopyLegacyRoughnessMotionSettings(target, source);
    }
}

std::optional<std::size_t> FindSceneVisualStateIndex(
    const std::vector<ScenePointVisualStateDocument>& states,
    std::string_view sceneGroupName,
    std::string_view visualName) {
    const auto scene = TrimAsciiWhitespace(std::string{sceneGroupName});
    const auto visual = NormalizeProjectPointVisualName(visualName);
    for (std::size_t index = 0; index < states.size(); ++index) {
        if (states[index].sceneGroupName == scene &&
            NormalizeProjectPointVisualName(states[index].visual.name) == visual) {
            return index;
        }
    }
    return std::nullopt;
}

void UpsertSceneVisualStateDocument(
    std::vector<ScenePointVisualStateDocument>* states,
    std::string_view sceneGroupName,
    std::string_view visualName,
    const PointCloudStyleState& style) {
    if (states == nullptr) {
        return;
    }

    const auto scene = TrimAsciiWhitespace(std::string{sceneGroupName});
    if (scene.empty()) {
        return;
    }

    ScenePointVisualStateDocument state;
    state.sceneGroupName = scene;
    state.visual.name = NormalizeProjectPointVisualName(visualName);
    state.visual.style = style;
    if (const auto existing = FindSceneVisualStateIndex(*states, scene, state.visual.name);
        existing.has_value()) {
        (*states)[existing.value()] = std::move(state);
    } else {
        states->push_back(std::move(state));
    }
}

void PruneSceneVisualStatesToKnownSceneGroups(ProjectDocument* document) {
    if (document == nullptr || document->sceneVisualStates.empty()) {
        return;
    }

    std::unordered_set<std::string> knownSceneGroups;
    for (const auto& layer : document->layers) {
        auto sceneGroup = TrimAsciiWhitespace(layer.sceneGroupName);
        if (!sceneGroup.empty()) {
            knownSceneGroups.insert(std::move(sceneGroup));
        }
    }
    if (knownSceneGroups.empty()) {
        return;
    }

    document->sceneVisualStates.erase(
        std::remove_if(
            document->sceneVisualStates.begin(),
            document->sceneVisualStates.end(),
            [&knownSceneGroups](const ScenePointVisualStateDocument& state) {
                return knownSceneGroups.find(TrimAsciiWhitespace(state.sceneGroupName)) ==
                       knownSceneGroups.end();
            }),
        document->sceneVisualStates.end());
}

bool SceneGroupHasName(std::string_view sceneGroupName, std::string_view expected) {
    return TrimAsciiWhitespace(std::string{sceneGroupName}) == expected;
}

void MigrateLegacyLayerPointVisuals(ProjectDocument* document) {
    if (document == nullptr) {
        return;
    }

    const bool alreadyHasProjectVisuals = !document->pointVisuals.empty();
    bool selectedFromExhibitionScene = false;
    std::optional<std::string> firstLegacySelectedBase;

    for (const auto& layer : document->layers) {
        if (layer.kind != SerializedLayerKind::PointCloud) {
            continue;
        }

        const bool exhibitionSceneLayer = SceneGroupHasName(layer.sceneGroupName, "ExhibitionScene");
        const auto sceneKey = LayerSceneVisualKey(layer);
        const auto selectedName = NormalizeProjectPointVisualName(layer.selectedPointVisualName);
        if (!selectedName.empty() && !firstLegacySelectedBase.has_value()) {
            firstLegacySelectedBase = ProjectPointVisualBaseName(selectedName);
        }

        if (layer.pointStyle.has_value() && layer.pointVisuals.empty() && !alreadyHasProjectVisuals) {
            const auto fallbackName = ProjectPointVisualBaseName(selectedName);
            UpsertPointVisualDocument(
                &document->pointVisuals,
                fallbackName,
                layer.pointStyle.value(),
                exhibitionSceneLayer);
        }

        for (const auto& legacyVisual : layer.pointVisuals) {
            const auto visualName = NormalizeProjectPointVisualName(legacyVisual.name);
            const auto baseName = ProjectPointVisualBaseName(visualName);
            const bool selectedLegacyVisual = visualName == selectedName;
            const bool editedLegacyVisual = IsProjectEditedPointVisualName(visualName);

            if (exhibitionSceneLayer) {
                if (editedLegacyVisual && selectedLegacyVisual) {
                    if (const auto existing = FindPointVisualDocumentIndex(document->pointVisuals, baseName);
                        existing.has_value()) {
                        if (!selectedFromExhibitionScene) {
                            document->pointVisuals[existing.value()].style = legacyVisual.style;
                        }
                        MergeLegacySceneRoleSpecificVisualSettings(
                            &document->pointVisuals[existing.value()].style,
                            legacyVisual.style,
                            layer.sceneRole);
                    } else {
                        UpsertPointVisualDocument(
                            &document->pointVisuals,
                            baseName,
                            legacyVisual.style,
                            true);
                    }
                    document->selectedPointVisualName = baseName;
                    selectedFromExhibitionScene = true;
                } else if (!editedLegacyVisual) {
                    if (const auto existing = FindPointVisualDocumentIndex(document->pointVisuals, visualName);
                        existing.has_value()) {
                        MergeLegacySceneRoleSpecificVisualSettings(
                            &document->pointVisuals[existing.value()].style,
                            legacyVisual.style,
                            layer.sceneRole);
                    } else {
                        UpsertPointVisualDocument(
                            &document->pointVisuals,
                            visualName,
                            legacyVisual.style,
                            !alreadyHasProjectVisuals);
                    }
                    if (selectedLegacyVisual && !selectedFromExhibitionScene) {
                        document->selectedPointVisualName = visualName;
                        selectedFromExhibitionScene = true;
                    }
                }
                continue;
            }

            if (editedLegacyVisual) {
                if (!sceneKey.empty()) {
                    UpsertSceneVisualStateDocument(
                        &document->sceneVisualStates,
                        sceneKey,
                        ProjectSceneVisualName(baseName, sceneKey),
                        legacyVisual.style);
                } else if (!alreadyHasProjectVisuals) {
                    UpsertPointVisualDocument(&document->pointVisuals, baseName, legacyVisual.style, false);
                }
                continue;
            }

            UpsertPointVisualDocument(
                &document->pointVisuals,
                visualName,
                legacyVisual.style,
                false);
        }
    }

    if (document->pointVisuals.empty()) {
        document->pointVisuals.push_back(
            {.name = std::string{"Unnamed"}, .style = PointCloudStyleState{}});
    }

    document->selectedPointVisualName = ProjectPointVisualBaseName(
        selectedFromExhibitionScene
            ? document->selectedPointVisualName
            : firstLegacySelectedBase.value_or(document->selectedPointVisualName));

    if (!FindPointVisualDocumentIndex(document->pointVisuals, document->selectedPointVisualName).has_value()) {
        document->selectedPointVisualName = NormalizeProjectPointVisualName(document->pointVisuals.front().name);
    }
}

json SerializeScenePointVisualState(const ScenePointVisualStateDocument& state) {
    return json{
        {"scene_group", state.sceneGroupName},
        {"visual", SerializePointCloudVisual(state.visual)},
    };
}

ScenePointVisualStateDocument ParseScenePointVisualState(const json& stateJson) {
    ScenePointVisualStateDocument state;
    state.sceneGroupName = stateJson.value("scene_group", std::string{});
    if (stateJson.contains("visual")) {
        state.visual = ParsePointCloudVisual(stateJson.at("visual"));
    }
    if (state.visual.name.empty()) {
        state.visual.name = ProjectSceneVisualName("Unnamed", state.sceneGroupName);
    }
    return state;
}

json SerializeScenePointCloudRoleSource(
    const ScenePointCloudRoleSourceDocument &source) {
  return json{
      {"scene_role", source.sceneRole},
      {"analysis_source_path", source.analysisSourcePath.generic_string()},
      {"display_source_path", source.displaySourcePath.generic_string()},
  };
}

ScenePointCloudRoleSourceDocument
ParseScenePointCloudRoleSource(const json &sourceJson) {
  ScenePointCloudRoleSourceDocument source;
  source.sceneRole = sourceJson.value("scene_role", std::string{});
  source.analysisSourcePath =
      sourceJson.value("analysis_source_path", std::string{});
  source.displaySourcePath =
      sourceJson.value("display_source_path", std::string{});
  return source;
}

json SerializeScenePointCloudGroup(const ScenePointCloudGroupDocument &group) {
  json groupJson{
      {"scene_group", group.sceneGroupName},
      {"display_spacing_meters", group.displaySpacingMeters},
      {"display_loaded", group.displayLoaded},
      {"display_visible", group.displayVisible},
      {"roles", json::array()},
  };
  for (const auto &source : group.roleSources) {
    groupJson["roles"].push_back(SerializeScenePointCloudRoleSource(source));
  }
  if (group.waterSurfaceCache.has_value()) {
    groupJson["water_surface_cache"] =
        SerializeWaterSurfaceCacheManifest(group.waterSurfaceCache.value());
  }
  return groupJson;
}

ScenePointCloudGroupDocument ParseScenePointCloudGroup(const json &groupJson) {
  ScenePointCloudGroupDocument group;
  group.sceneGroupName = groupJson.value("scene_group", std::string{});
  group.displaySpacingMeters = groupJson.value("display_spacing_meters", 0.0F);
  group.displayLoaded = groupJson.value("display_loaded", false);
  group.displayVisible = groupJson.value("display_visible", false);
  if (groupJson.contains("roles") && groupJson.at("roles").is_array()) {
    for (const auto &sourceJson : groupJson.at("roles")) {
      group.roleSources.push_back(ParseScenePointCloudRoleSource(sourceJson));
    }
  }
  if (groupJson.contains("water_surface_cache") &&
      groupJson.at("water_surface_cache").is_object()) {
    group.waterSurfaceCache =
        ParseWaterSurfaceCacheManifest(groupJson.at("water_surface_cache"));
  }
  return group;
}

json SerializeGaussianSplatStyle(const GaussianSplatStyleState& style) {
    return json{
        {"color_mode", GaussianSplatColorModeName(style.colorMode)},
        {"debug_mode", GaussianSplatDebugModeName(style.debugMode)},
        {"quality_mode", GaussianSplatQualityModeName(style.qualityMode)},
        {"opacity_multiplier", style.opacityMultiplier},
        {"scale_multiplier", style.scaleMultiplier},
        {"exposure", style.exposure},
        {"saturation", style.saturation},
        {"layer_tint", style.layerTint},
    };
}

GaussianSplatStyleState ParseGaussianSplatStyle(const json& styleJson) {
    GaussianSplatStyleState style;
    if (styleJson.contains("color_mode")) {
        style.colorMode = ParseGaussianSplatColorMode(styleJson.at("color_mode"));
    }
    if (styleJson.contains("debug_mode")) {
        style.debugMode = ParseGaussianSplatDebugMode(styleJson.at("debug_mode"));
    }
    if (styleJson.contains("quality_mode")) {
        style.qualityMode = ParseGaussianSplatQualityMode(styleJson.at("quality_mode"));
    }
    style.opacityMultiplier =
        std::clamp(styleJson.value("opacity_multiplier", style.opacityMultiplier), 0.0F, 64.0F);
    style.scaleMultiplier =
        std::clamp(styleJson.value("scale_multiplier", style.scaleMultiplier), 0.001F, 64.0F);
    style.exposure = std::clamp(styleJson.value("exposure", style.exposure), 0.0F, 64.0F);
    style.saturation = std::clamp(styleJson.value("saturation", style.saturation), 0.0F, 64.0F);
    if (styleJson.contains("layer_tint")) {
        style.layerTint = styleJson.at("layer_tint").get<std::array<float, 4>>();
    }
    return style;
}

PointCloudStyleState ParsePointCloudStyle(const json& styleJson) {
    PointCloudStyleState style;
    std::optional<LegacyPointCloudRenderMode> legacyRenderMode;
    if (styleJson.contains("geometry_mode")) {
        style.geometryMode = ParsePointCloudGeometryMode(styleJson.at("geometry_mode"));
    }
    if (styleJson.contains("screen_sprite_size_mode")) {
        style.screenSpriteSizeMode = ParsePointCloudScreenSpriteSizeMode(styleJson.at("screen_sprite_size_mode"));
    }
    if (styleJson.contains("render_mode")) {
        legacyRenderMode = ParseLegacyPointCloudRenderMode(styleJson.at("render_mode"));
    }
    if (styleJson.contains("falloff_profile")) {
        style.falloffProfile = ParsePointCloudFalloffProfile(styleJson.at("falloff_profile"));
    }
    if (styleJson.contains("stylisation_mode")) {
        style.stylisationMode = ParsePointCloudStylisationMode(styleJson.at("stylisation_mode"));
    } else if (styleJson.contains("stylization_mode")) {
        style.stylisationMode = ParsePointCloudStylisationMode(styleJson.at("stylization_mode"));
    }
    if (styleJson.contains("npr_preset")) {
        style.nprPreset = ParsePointCloudNprPreset(styleJson.at("npr_preset"));
    }
    if (styleJson.contains("color_mode")) {
        style.colorMode = ParsePointCloudColorMode(styleJson.at("color_mode"));
    }
    if (styleJson.contains("colormap")) {
        style.colormap = ParsePointCloudColormap(styleJson.at("colormap"));
    }
    if (styleJson.contains("solid_color")) {
        style.solidColor = styleJson.at("solid_color").get<std::array<float, 4>>();
    }
    if (styleJson.contains("gradient_start_color")) {
        style.gradientStartColor = styleJson.at("gradient_start_color").get<std::array<float, 3>>();
    }
    if (styleJson.contains("gradient_end_color")) {
        style.gradientEndColor = styleJson.at("gradient_end_color").get<std::array<float, 3>>();
    }
    if (styleJson.contains("colorize_color")) {
        style.colorizeColor = styleJson.at("colorize_color").get<std::array<float, 3>>();
    } else if (styleJson.contains("colourise_color")) {
        style.colorizeColor = styleJson.at("colourise_color").get<std::array<float, 3>>();
    }
    style.colorizeAmount = styleJson.value("colorize_amount", styleJson.value("colourise_amount", style.colorizeAmount));
    style.stylisationStrength = styleJson.value(
        "stylisation_strength",
        styleJson.value("stylization_strength", style.stylisationStrength));
    style.stylisationColorLevels = styleJson.value(
        "stylisation_color_levels",
        styleJson.value("stylization_color_levels", style.stylisationColorLevels));
    style.stylisationInkStrength = styleJson.value(
        "stylisation_ink_strength",
        styleJson.value("stylization_ink_strength", style.stylisationInkStrength));
    style.stylisationPaperGrain = styleJson.value(
        "stylisation_paper_grain",
        styleJson.value("stylization_paper_grain", style.stylisationPaperGrain));
    style.stylisationPigmentBleed = styleJson.value(
        "stylisation_pigment_bleed",
        styleJson.value("stylization_pigment_bleed", style.stylisationPigmentBleed));
    style.brushAspect = styleJson.value("brush_aspect", style.brushAspect);
    style.strokeJitter = styleJson.value("stroke_jitter", style.strokeJitter);
    style.hatchStrength = styleJson.value("hatch_strength", style.hatchStrength);
    style.strokeOpacityVariance = styleJson.value(
        "stroke_opacity_variance",
        style.strokeOpacityVariance);
    style.pigmentVariation = styleJson.value(
        "pigment_variation",
        styleJson.value("stylisation_pigment_variation", styleJson.value("stylization_pigment_variation", style.pigmentVariation)));
    style.pigmentAnimationSpeed = styleJson.value(
        "pigment_animation_speed",
        styleJson.value(
            "stylisation_pigment_animation_speed",
            styleJson.value("stylization_pigment_animation_speed", style.pigmentAnimationSpeed)));
    style.granulationAngleStrength = styleJson.value(
        "granulation_angle_strength",
        styleJson.value(
            "stylisation_granulation_angle_strength",
            styleJson.value("stylization_granulation_angle_strength", style.granulationAngleStrength)));
    style.roughnessMotionStrength =
        std::clamp(styleJson.value("roughness_motion_strength", style.roughnessMotionStrength), 0.0F, 1.0F);
    style.roughnessMotionScale =
        std::clamp(styleJson.value("roughness_motion_scale", style.roughnessMotionScale), 0.01F, 50.0F);
    style.roughnessMotionSpeed =
        std::clamp(styleJson.value("roughness_motion_speed", style.roughnessMotionSpeed), 0.0F, 8.0F);
    style.roughnessMotionThreshold =
        std::clamp(styleJson.value("roughness_motion_threshold", style.roughnessMotionThreshold), 0.0F, 1.0F);
    style.roughnessMotionGroundId =
        std::clamp(styleJson.value("roughness_motion_ground_id", style.roughnessMotionGroundId), 0.0F, 1.0F);
    style.causticAnimation = styleJson.value("caustic_animation", style.causticAnimation);
    style.causticIntensity = std::clamp(styleJson.value("caustic_intensity", style.causticIntensity), 0.0F, 5.0F);
    style.causticScale = std::clamp(styleJson.value("caustic_scale", style.causticScale), 0.01F, 80.0F);
    style.causticSpeed = std::clamp(styleJson.value("caustic_speed", style.causticSpeed), 0.0F, 10.0F);
    style.causticLineSharpness =
        std::clamp(styleJson.value("caustic_line_sharpness", style.causticLineSharpness), 0.0F, 1.0F);
    style.causticWarp = std::clamp(styleJson.value("caustic_warp", style.causticWarp), 0.0F, 3.0F);
    const bool hasLegacyCausticPattern =
        styleJson.contains("caustic_scale") ||
        styleJson.contains("caustic_line_sharpness") ||
        styleJson.contains("caustic_warp");
    const float legacyCausticCellSize = 1.0F / std::max(0.01F, style.causticScale);
    const float legacyCausticLineWidth =
        legacyCausticCellSize *
        (0.16F + (0.025F - 0.16F) * std::clamp(style.causticLineSharpness, 0.0F, 1.0F));
    style.causticCellSizeMeters = std::clamp(
        styleJson.value(
            "caustic_cell_size_meters",
            hasLegacyCausticPattern ? legacyCausticCellSize : style.causticCellSizeMeters),
        0.005F,
        5.0F);
    style.causticLineWidthMeters = std::clamp(
        styleJson.value(
            "caustic_line_width_meters",
            hasLegacyCausticPattern ? legacyCausticLineWidth : style.causticLineWidthMeters),
        0.0005F,
        0.50F);
    style.causticFeatherMeters = std::clamp(
        styleJson.value(
            "caustic_feather_meters",
            hasLegacyCausticPattern ? style.causticLineWidthMeters * 0.4F : style.causticFeatherMeters),
        0.0005F,
        0.50F);
    style.causticSurfacePointSpacingMeters = std::clamp(
        styleJson.value("caustic_surface_point_spacing_meters", style.causticSurfacePointSpacingMeters),
        0.0005F,
        0.10F);
    style.causticWarpAmplitudeMeters = std::clamp(
        styleJson.value(
            "caustic_warp_amplitude_meters",
            hasLegacyCausticPattern ? legacyCausticCellSize * style.causticWarp * 0.5F
                                    : style.causticWarpAmplitudeMeters),
        0.0F,
        2.0F);
    if (styleJson.contains("caustic_tint")) {
        style.causticTint = styleJson.at("caustic_tint").get<std::array<float, 3>>();
    }
    style.causticEmissionBoost =
        std::clamp(styleJson.value("caustic_emission_boost", style.causticEmissionBoost), 0.0F, 8.0F);
    style.causticOpacityBoost =
        std::clamp(styleJson.value("caustic_opacity_boost", style.causticOpacityBoost), 0.0F, 2.0F);
    style.causticPointSizeBoost =
        std::clamp(styleJson.value("caustic_point_size_boost", style.causticPointSizeBoost), 0.0F, 4.0F);
    style.causticMaskFieldSlot = styleJson.value("caustic_mask_field_slot", style.causticMaskFieldSlot);
    style.causticEdgeFieldSlot = styleJson.value("caustic_edge_field_slot", style.causticEdgeFieldSlot);
    style.causticSeedFieldSlot = styleJson.value("caustic_seed_field_slot", style.causticSeedFieldSlot);
    style.shorelineWaveEnabled = styleJson.value("shoreline_wave_enabled", style.shorelineWaveEnabled);
    if (styleJson.contains("shoreline_wave_algorithm")) {
        style.shorelineWaveAlgorithm =
            ParsePointCloudShorelineWaveAlgorithm(styleJson.at("shoreline_wave_algorithm"));
    }
    if (styleJson.contains("shoreline_height_foam")) {
        style.shorelineHeightFoam =
            ParseHeightFoamShorelineSettings(styleJson.at("shoreline_height_foam"));
    }
    style.shorelineBoundaryZ =
        std::clamp(styleJson.value("shoreline_boundary_z", style.shorelineBoundaryZ), -1000.0F, 1000.0F);
    style.shorelineHeightReachMeters = std::clamp(
        styleJson.value("shoreline_height_reach_meters", style.shorelineHeightReachMeters),
        0.001F,
        50.0F);
    style.shorelineEdgeFadeMeters = std::clamp(
        styleJson.value("shoreline_edge_fade_meters", style.shorelineEdgeFadeMeters),
        0.0F,
        10.0F);
    if (styleJson.contains("shoreline_direction")) {
        const auto direction = styleJson.at("shoreline_direction").get<std::array<float, 2>>();
        style.shorelineDirectionX = direction[0];
        style.shorelineDirectionY = direction[1];
    }
    style.shorelinePatternScale =
        std::clamp(styleJson.value("shoreline_pattern_scale", style.shorelinePatternScale), 0.01F, 50.0F);
    style.shorelineWavelengthMeters =
        std::clamp(styleJson.value("shoreline_wavelength_meters", style.shorelineWavelengthMeters), 0.002F, 10.0F);
    style.shorelineSpeed = std::clamp(styleJson.value("shoreline_speed", style.shorelineSpeed), 0.0F, 10.0F);
    style.shorelineWarp = std::clamp(styleJson.value("shoreline_warp", style.shorelineWarp), 0.0F, 3.0F);
    style.shorelineTurbulence =
        std::clamp(styleJson.value("shoreline_turbulence", style.shorelineTurbulence), 0.0F, 1.0F);
    style.shorelineDensity = std::clamp(styleJson.value("shoreline_density", style.shorelineDensity), 0.0F, 1.0F);
    style.shorelinePhase = styleJson.value("shoreline_phase", style.shorelinePhase);
    style.shorelineIntensity =
        std::clamp(styleJson.value("shoreline_intensity", style.shorelineIntensity), 0.0F, 5.0F);
    style.shorelineEmissionAdd =
        std::clamp(styleJson.value("shoreline_emission_add", style.shorelineEmissionAdd), 0.0F, 8.0F);
    style.shorelineOpacityAdd =
        std::clamp(styleJson.value("shoreline_opacity_add", style.shorelineOpacityAdd), -1.0F, 2.0F);
    style.shorelineOpacityMultiply =
        std::clamp(styleJson.value("shoreline_opacity_multiply", style.shorelineOpacityMultiply), 0.0F, 8.0F);
    style.shorelinePointSizeAdd =
        std::clamp(styleJson.value("shoreline_point_size_add", style.shorelinePointSizeAdd), -256.0F, 512.0F);
    style.shorelinePointSizeMultiply = std::clamp(
        styleJson.value("shoreline_point_size_multiply", style.shorelinePointSizeMultiply),
        0.0F,
        8.0F);
    style.shorelineColourMix =
        std::clamp(styleJson.value("shoreline_colour_mix", style.shorelineColourMix), 0.0F, 1.0F);
    if (styleJson.contains("shoreline_colour")) {
        style.shorelineColour = styleJson.at("shoreline_colour").get<std::array<float, 3>>();
    }
    style.shorelineSeed = styleJson.value("shoreline_seed", style.shorelineSeed);
    style.exposure = styleJson.value("exposure", style.exposure);
    style.innerRadius = styleJson.value("inner_radius", style.innerRadius);
    style.gaussianSharpness = styleJson.value("gaussian_sharpness", style.gaussianSharpness);
    style.featherPower = styleJson.value("feather_power", style.featherPower);
    style.waterStreakAspect = std::clamp(styleJson.value("water_streak_aspect", style.waterStreakAspect), 1.0F, 32.0F);
    style.solidCenters = styleJson.value("solid_centers", style.solidCenters);
    style.flowAnimation = styleJson.value("flow_animation", style.flowAnimation);
    style.waterPathView = styleJson.value("water_path_view", style.waterPathView);
    style.waterTrailOverlay = styleJson.value(
        "water_trail_overlay",
        styleJson.value(
            "water_stream_overlay",
            styleJson.value("water_overlay_render_mode", std::string{}) == "stream"));
    if (styleJson.contains("point_size")) {
        style.pointSize = ParseBinding(styleJson.at("point_size"));
    }
    if (styleJson.contains("surfel_diameter")) {
        style.surfelDiameter = ParseBinding(styleJson.at("surfel_diameter"));
    }
    if (styleJson.contains("opacity")) {
        style.opacity = ParseBinding(styleJson.at("opacity"));
    }
    if (styleJson.contains("emissive_strength")) {
        style.emissiveStrength = ParseBinding(styleJson.at("emissive_strength"));
    }
    if (styleJson.contains("depth_fade")) {
        style.depthFade = ParseBinding(styleJson.at("depth_fade"));
    }
    if (styleJson.contains("colormap_position")) {
        style.colormapPosition = ParseBinding(styleJson.at("colormap_position"));
    }
    if (legacyRenderMode.has_value()) {
        MigrateLegacyPointCloudRenderMode(
            legacyRenderMode.value(),
            styleJson.contains("emissive_strength"),
            &style);
    }
    return style;
}

json SerializeProjectLayer(const ProjectLayerDocument& layer) {
    json layerJson{
        {"kind", SerializedLayerKindName(layer.kind)},
        {"source_path", layer.sourcePath.generic_string()},
        {"loaded", layer.loaded},
        {"visible", layer.visible},
        {"point_budget_active_points", layer.pointBudgetActivePoints},
    };
    if (!layer.sceneGroupName.empty()) {
        layerJson["scene_group"] = layer.sceneGroupName;
    }
    if (!layer.sceneRole.empty()) {
        layerJson["scene_role"] = layer.sceneRole;
    }
    if (layer.inferredPointSpacingMeters > 0.0F) {
        layerJson["inferred_point_spacing_meters"] = layer.inferredPointSpacingMeters;
    }
    if (layer.pointSpacingMeters > 0.0F) {
        layerJson["point_spacing_meters"] = layer.pointSpacingMeters;
    }
    if (layer.pointSpacingManualOverride) {
        layerJson["point_spacing_manual_override"] = layer.pointSpacingManualOverride;
    }
    if (layer.scenePrimaryRole) {
        layerJson["scene_primary_role"] = layer.scenePrimaryRole;
    }
    if (!layer.selectedSceneVariantPath.empty()) {
        layerJson["selected_scene_variant_path"] = layer.selectedSceneVariantPath.generic_string();
    }
    return layerJson;
}

ProjectLayerDocument ParseProjectLayer(const json& layerJson) {
    ProjectLayerDocument layer;
    layer.kind = ParseSerializedLayerKind(layerJson.at("kind"));
    layer.sourcePath = layerJson.value("source_path", std::string{});
    layer.sceneGroupName = layerJson.value("scene_group", std::string{});
    layer.sceneRole = layerJson.value("scene_role", std::string{});
    layer.inferredPointSpacingMeters = layerJson.value("inferred_point_spacing_meters", 0.0F);
    layer.pointSpacingMeters = layerJson.value("point_spacing_meters", 0.0F);
    layer.pointSpacingManualOverride = layerJson.value("point_spacing_manual_override", false);
    layer.scenePrimaryRole = layerJson.value("scene_primary_role", false);
    layer.selectedSceneVariantPath = layerJson.value("selected_scene_variant_path", std::string{});
    layer.loaded = layerJson.value("loaded", false);
    layer.visible = layerJson.value("visible", false);
    layer.pointBudgetActivePoints = layerJson.value("point_budget_active_points", 0ULL);
    if (layerJson.contains("point_style")) {
        layer.pointStyle = ParsePointCloudStyle(layerJson.at("point_style"));
    }
    if (layerJson.contains("point_visuals") && layerJson.at("point_visuals").is_array()) {
        for (const auto& visualJson : layerJson.at("point_visuals")) {
            layer.pointVisuals.push_back(ParsePointCloudVisual(visualJson));
        }
        layer.selectedPointVisualName =
            layerJson.value("selected_point_visual", layer.selectedPointVisualName);
        if (layer.selectedPointVisualName.empty()) {
            layer.selectedPointVisualName = "Unnamed";
        }
    }
    return layer;
}

bool SerializedPathsMatch(const std::filesystem::path &left,
                          const std::filesystem::path &right) {
  if (left.empty() || right.empty()) {
    return left.empty() && right.empty();
  }
  return left.lexically_normal().generic_string() ==
         right.lexically_normal().generic_string();
}

float LegacySceneLayerSpacingMeters(const ProjectLayerDocument &layer) {
  if (std::isfinite(layer.inferredPointSpacingMeters) &&
      layer.inferredPointSpacingMeters > 0.0F) {
    return layer.inferredPointSpacingMeters;
  }
  if (std::isfinite(layer.pointSpacingMeters) &&
      layer.pointSpacingMeters > 0.0F) {
    return layer.pointSpacingMeters;
  }
  return 0.0F;
}

int LegacySceneSpacingPreference(const ProjectLayerDocument &layer) {
  if (layer.scenePrimaryRole) {
    return 3;
  }
  if (TrimAsciiWhitespace(layer.sceneRole) == "ROCK") {
    return 2;
  }
  return 1;
}

bool IsLegacyScenePointCloudLayer(const ProjectLayerDocument &layer) {
  return layer.kind == SerializedLayerKind::PointCloud &&
         !TrimAsciiWhitespace(layer.sceneGroupName).empty() &&
         !TrimAsciiWhitespace(layer.sceneRole).empty();
}

void MigrateLegacyScenePointCloudGroups(ProjectDocument *document) {
  if (document == nullptr) {
    return;
  }

  document->scenePointCloudGroups.clear();
  for (const auto &layer : document->layers) {
    if (!IsLegacyScenePointCloudLayer(layer)) {
      continue;
    }
    const auto sceneGroupName = TrimAsciiWhitespace(layer.sceneGroupName);
    const auto sceneRole = TrimAsciiWhitespace(layer.sceneRole);
    auto groupIt = std::find_if(
        document->scenePointCloudGroups.begin(),
        document->scenePointCloudGroups.end(),
        [&sceneGroupName](const ScenePointCloudGroupDocument &group) {
          return group.sceneGroupName == sceneGroupName;
        });
    if (groupIt == document->scenePointCloudGroups.end()) {
      document->scenePointCloudGroups.push_back(
          {.sceneGroupName = sceneGroupName});
      groupIt = std::prev(document->scenePointCloudGroups.end());
    }
    if (std::none_of(
            groupIt->roleSources.begin(), groupIt->roleSources.end(),
            [&sceneRole](const ScenePointCloudRoleSourceDocument &source) {
              return source.sceneRole == sceneRole;
            })) {
      groupIt->roleSources.push_back({.sceneRole = sceneRole});
    }
  }

  for (auto &group : document->scenePointCloudGroups) {
    for (auto &source : group.roleSources) {
      std::filesystem::path selectedCandidate;
      std::filesystem::path fallbackCandidate;
      for (const auto &layer : document->layers) {
        if (!IsLegacyScenePointCloudLayer(layer) ||
            TrimAsciiWhitespace(layer.sceneGroupName) != group.sceneGroupName ||
            TrimAsciiWhitespace(layer.sceneRole) != source.sceneRole) {
          continue;
        }
        if (fallbackCandidate.empty() && !layer.sourcePath.empty()) {
          fallbackCandidate = layer.sourcePath;
        }
        if (layer.selectedSceneVariantPath.empty()) {
          continue;
        }
        if (selectedCandidate.empty()) {
          selectedCandidate = layer.selectedSceneVariantPath;
        }
        if (SerializedPathsMatch(layer.sourcePath,
                                 layer.selectedSceneVariantPath)) {
          selectedCandidate = layer.selectedSceneVariantPath;
          break;
        }
      }
      const auto &candidate =
          selectedCandidate.empty() ? fallbackCandidate : selectedCandidate;
      source.analysisSourcePath = candidate;
      source.displaySourcePath = candidate;
    }

    const ProjectLayerDocument *bestSpacingLayer = nullptr;
    int bestSpacingPreference = 0;
    bool matchedSelectedLayer = false;
    for (const auto &source : group.roleSources) {
      for (const auto &layer : document->layers) {
        if (!IsLegacyScenePointCloudLayer(layer) ||
            TrimAsciiWhitespace(layer.sceneGroupName) != group.sceneGroupName ||
            TrimAsciiWhitespace(layer.sceneRole) != source.sceneRole ||
            !SerializedPathsMatch(layer.sourcePath, source.displaySourcePath)) {
          continue;
        }
        matchedSelectedLayer = true;
        group.displayLoaded |= layer.loaded;
        group.displayVisible |= layer.visible;
        const int spacingPreference = LegacySceneSpacingPreference(layer);
        if (LegacySceneLayerSpacingMeters(layer) > 0.0F &&
            (bestSpacingLayer == nullptr ||
             spacingPreference > bestSpacingPreference)) {
          bestSpacingLayer = &layer;
          bestSpacingPreference = spacingPreference;
        }
      }
    }

    if (!matchedSelectedLayer) {
      for (const auto &layer : document->layers) {
        if (!IsLegacyScenePointCloudLayer(layer) ||
            TrimAsciiWhitespace(layer.sceneGroupName) != group.sceneGroupName) {
          continue;
        }
        group.displayLoaded |= layer.loaded;
        group.displayVisible |= layer.visible;
        const int spacingPreference = LegacySceneSpacingPreference(layer);
        if (LegacySceneLayerSpacingMeters(layer) > 0.0F &&
            (bestSpacingLayer == nullptr ||
             spacingPreference > bestSpacingPreference)) {
          bestSpacingLayer = &layer;
          bestSpacingPreference = spacingPreference;
        }
      }
    }
    if (bestSpacingLayer != nullptr) {
      group.displaySpacingMeters =
          LegacySceneLayerSpacingMeters(*bestSpacingLayer);
    }
  }
}

json SerializeCameraState(const CameraState& state) {
    json stateJson{
        {"position", state.position},
        {"orientation", state.orientation},
        {"target", state.target},
        {"fov_degrees", state.fovDegrees},
        {"near_plane", state.nearPlane},
        {"far_plane", state.farPlane},
        {"has_depth_of_field", state.hasDepthOfField},
        {"focus_distance", state.focusDistance},
        {"aperture_f_stops", state.apertureFStops},
        {"depth_of_field_max_blur_px", state.depthOfFieldMaxBlurPixels},
    };
    if (state.hasOrbitCenter) {
        stateJson["orbit_center"] = state.orbitCenter;
    }
    return stateJson;
}

CameraState ParseCameraState(const json& stateJson) {
    CameraState state;
    if (stateJson.contains("position")) {
        state.position = stateJson.at("position").get<std::array<float, 3>>();
    }
    if (stateJson.contains("orientation")) {
        state.orientation = stateJson.at("orientation").get<std::array<float, 4>>();
    }
    if (stateJson.contains("target")) {
        state.target = stateJson.at("target").get<std::array<float, 3>>();
    }
    if (stateJson.contains("orbit_center")) {
        state.orbitCenter = stateJson.at("orbit_center").get<std::array<float, 3>>();
        state.hasOrbitCenter = true;
    }
    state.fovDegrees = stateJson.value("fov_degrees", 60.0F);
    state.nearPlane = stateJson.value("near_plane", 0.01F);
    state.farPlane = stateJson.value("far_plane", 1000.0F);
    state.hasDepthOfField = stateJson.value("has_depth_of_field", false);
    state.focusDistance = stateJson.value("focus_distance", 1.0F);
    state.apertureFStops = stateJson.value("aperture_f_stops", 8.0F);
    state.depthOfFieldMaxBlurPixels = stateJson.value("depth_of_field_max_blur_px", 24.0F);
    return state;
}

json SerializePathArray(const std::vector<std::filesystem::path>& paths) {
    json pathsJson = json::array();
    for (const auto& path : paths) {
        pathsJson.push_back(path.generic_string());
    }
    return pathsJson;
}

std::vector<std::filesystem::path> ParsePathArray(const json& pathsJson) {
    std::vector<std::filesystem::path> paths;
    if (!pathsJson.is_array()) {
        return paths;
    }

    for (const auto& pathJson : pathsJson) {
        if (pathJson.is_string()) {
            paths.emplace_back(pathJson.get<std::string>());
        }
    }
    return paths;
}

json SerializeCameraShot(const CameraShot& shot) {
    auto shotJson = json{
        {"id", shot.id},
        {"name", shot.name},
        {"camera", SerializeCameraState(shot.state)},
        {"associated_layer_paths", SerializePathArray(shot.associatedLayerPaths)},
    };
    return shotJson;
}

CameraShot ParseCameraShot(const json& shotJson) {
    CameraShot shot;
    shot.id = shotJson.value("id", std::string{});
    shot.name = shotJson.value("name", std::string{"Camera Shot"});
    shot.durationFrames = shotJson.value("duration_frames", 90U);
    if (shotJson.contains("camera")) {
        shot.state = ParseCameraState(shotJson.at("camera"));
    }
    if (shotJson.contains("associated_layer_paths")) {
        shot.associatedLayerPaths = ParsePathArray(shotJson.at("associated_layer_paths"));
    }
    return shot;
}

std::string UniqueIndexedId(
    const char* prefix,
    std::size_t index,
    std::unordered_set<std::string>* usedIds) {
    if (usedIds == nullptr) {
        return std::string{prefix} + std::to_string(index + 1U);
    }

    auto candidate = std::string{prefix} + std::to_string(index + 1U);
    std::size_t suffix = index + 1U;
    while (usedIds->contains(candidate)) {
        candidate = std::string{prefix} + std::to_string(++suffix);
    }
    usedIds->insert(candidate);
    return candidate;
}

void EnsureCameraShotIds(std::vector<CameraShot>* shots) {
    if (shots == nullptr) {
        return;
    }

    std::unordered_set<std::string> usedIds;
    for (const auto& shot : *shots) {
        if (!shot.id.empty()) {
            usedIds.insert(shot.id);
        }
    }

    for (std::size_t index = 0; index < shots->size(); ++index) {
        if ((*shots)[index].id.empty()) {
            (*shots)[index].id = UniqueIndexedId("camera_", index, &usedIds);
        }
    }
}

void EnsureAnimationPathKeyIds(AnimationPath* path) {
    if (path == nullptr) {
        return;
    }

    std::unordered_set<std::string> usedIds;
    for (const auto& key : path->keys) {
        if (!key.id.empty()) {
            usedIds.insert(key.id);
        }
    }

    for (std::size_t index = 0; index < path->keys.size(); ++index) {
        if (path->keys[index].id.empty()) {
            path->keys[index].id = UniqueIndexedId("key_", index, &usedIds);
        }
    }

    for (std::size_t trackIndex = 0; trackIndex < path->waterScenarioTracks.size(); ++trackIndex) {
        auto& track = path->waterScenarioTracks[trackIndex];
        std::unordered_set<std::string> usedWaterKeyIds;
        for (const auto& key : track.keys) {
            if (!key.id.empty()) {
                usedWaterKeyIds.insert(key.id);
            }
        }
        const auto prefix = std::string{"water_key_"} + std::to_string(trackIndex + 1U) + "_";
        for (std::size_t keyIndex = 0; keyIndex < track.keys.size(); ++keyIndex) {
            if (track.keys[keyIndex].id.empty()) {
                track.keys[keyIndex].id =
                    UniqueIndexedId(prefix.c_str(), keyIndex, &usedWaterKeyIds);
            }
        }
    }
}

json SerializeAnimationPathKey(const AnimationPathKey& key) {
    return json{
        {"id", key.id},
        {"camera_position", key.cameraPosition},
        {"focus_point", key.focusPoint},
        {"has_orientation", key.hasOrientation},
        {"orientation", key.orientation},
        {"has_focus_distance", key.hasFocusDistance},
        {"focus_distance", key.focusDistance},
        {"has_aperture_f_stops", key.hasApertureFStops},
        {"aperture_f_stops", key.apertureFStops},
        {"fov_degrees", key.fovDegrees},
        {"near_plane", key.nearPlane},
        {"far_plane", key.farPlane},
        {"duration_frames", key.durationFrames},
        {"source_shot_name", key.sourceShotName},
        {"linked_camera_id", key.linkedCameraId},
        {"linked_camera_name", key.linkedCameraName},
    };
}

AnimationPathKey ParseAnimationPathKey(const json& keyJson) {
    AnimationPathKey key;
    key.id = keyJson.value("id", std::string{});
    if (keyJson.contains("camera_position")) {
        key.cameraPosition = keyJson.at("camera_position").get<std::array<float, 3>>();
    }
    if (keyJson.contains("focus_point")) {
        key.focusPoint = keyJson.at("focus_point").get<std::array<float, 3>>();
    }
    key.hasOrientation = keyJson.value("has_orientation", key.hasOrientation);
    if (keyJson.contains("orientation")) {
        key.orientation = keyJson.at("orientation").get<std::array<float, 4>>();
        key.hasOrientation = keyJson.value("has_orientation", true);
    }
    key.hasFocusDistance = keyJson.value("has_focus_distance", key.hasFocusDistance);
    key.focusDistance = keyJson.value("focus_distance", key.focusDistance);
    key.hasApertureFStops = keyJson.value("has_aperture_f_stops", key.hasApertureFStops);
    key.apertureFStops = keyJson.value("aperture_f_stops", key.apertureFStops);
    key.fovDegrees = keyJson.value("fov_degrees", key.fovDegrees);
    key.nearPlane = keyJson.value("near_plane", key.nearPlane);
    key.farPlane = keyJson.value("far_plane", key.farPlane);
    key.durationFrames = keyJson.value("duration_frames", key.durationFrames);
    key.sourceShotName = keyJson.value("source_shot_name", std::string{});
    key.linkedCameraId = keyJson.value("linked_camera_id", std::string{});
    key.linkedCameraName = keyJson.value("linked_camera_name", std::string{});
    return key;
}

json SerializeAnimationExportSettings(const AnimationExportSettings& settings) {
    return json{
        {"output_directory", settings.outputDirectory},
        {"width", settings.width},
        {"height", settings.height},
        {"fps", settings.framesPerSecond},
        {"still_camera_duration_seconds", settings.stillCameraDurationSeconds},
        {"start_frame", settings.startFrame},
        {"end_frame", settings.endFrame},
    };
}

AnimationExportSettings ParseAnimationExportSettings(const json& settingsJson) {
    AnimationExportSettings settings;
    settings.outputDirectory = settingsJson.value("output_directory", std::string{});
    settings.width = settingsJson.value("width", settings.width);
    settings.height = settingsJson.value("height", settings.height);
    settings.framesPerSecond = settingsJson.value("fps", settings.framesPerSecond);
    settings.stillCameraDurationSeconds =
        std::clamp(settingsJson.value("still_camera_duration_seconds", settings.stillCameraDurationSeconds), 0.001F, 3600.0F);
    settings.startFrame = settingsJson.value("start_frame", settings.startFrame);
    settings.endFrame = settingsJson.value("end_frame", settings.endFrame);
    return settings;
}

json SerializeWaterScenarioTrack(const WaterScenarioTrack& track);
WaterScenarioTrack ParseWaterScenarioTrack(const json& trackJson);

json SerializeAnimationPath(const AnimationPath& path) {
    json pathJson{
        {"schema_version", kAnimationDocumentSchemaVersion},
        {"name", path.name},
        {"duration_frames", path.durationFrames},
        {"associated_layer_paths", SerializePathArray(path.associatedLayerPaths)},
        {"depth_of_field_enabled", path.depthOfFieldEnabled},
        {"aperture_f_stops", path.apertureFStops},
        {"depth_of_field_max_blur_px", path.depthOfFieldMaxBlurPixels},
        {"export_settings", SerializeAnimationExportSettings(path.exportSettings)},
        {"selected_water_scenario_id", path.selectedWaterScenarioId},
        {"water_scenario_tracks", json::array()},
        {"keys", json::array()},
    };
    if (path.waterAnimationTrailSettings.has_value()) {
        pathJson["water_animation_trail_settings"] =
            SerializeWaterAnimationTrailSettings(path.waterAnimationTrailSettings.value());
    }
    if (path.tempWaterAnimationTrailSettings.has_value()) {
        pathJson["temp_water_animation_trail_settings"] =
            SerializeWaterAnimationTrailSettings(path.tempWaterAnimationTrailSettings.value());
    }
    if (path.waterCausticLookSettings.has_value()) {
        pathJson["water_caustic_look_settings"] =
            SerializeWaterCausticLookSettings(path.waterCausticLookSettings.value());
    }
    if (path.tempWaterCausticLookSettings.has_value()) {
        pathJson["temp_water_caustic_look_settings"] =
            SerializeWaterCausticLookSettings(path.tempWaterCausticLookSettings.value());
    }
    for (const auto& key : path.keys) {
        pathJson["keys"].push_back(SerializeAnimationPathKey(key));
    }
    for (const auto& track : path.waterScenarioTracks) {
        pathJson["water_scenario_tracks"].push_back(SerializeWaterScenarioTrack(track));
    }
    return pathJson;
}

AnimationPath ParseAnimationPath(const json& pathJson) {
    AnimationPath path;
    path.name = pathJson.value("name", path.name);
    path.durationFrames = pathJson.value("duration_frames", path.durationFrames);
    if (pathJson.contains("associated_layer_paths")) {
        path.associatedLayerPaths = ParsePathArray(pathJson.at("associated_layer_paths"));
    }
    path.depthOfFieldEnabled = pathJson.value("depth_of_field_enabled", path.depthOfFieldEnabled);
    path.apertureFStops = pathJson.value("aperture_f_stops", path.apertureFStops);
    path.depthOfFieldMaxBlurPixels =
        pathJson.value("depth_of_field_max_blur_px", path.depthOfFieldMaxBlurPixels);
    path.selectedWaterScenarioId =
        pathJson.value("selected_water_scenario_id", std::string{});
    if (pathJson.contains("export_settings")) {
        path.exportSettings = ParseAnimationExportSettings(pathJson.at("export_settings"));
    }
    if (pathJson.contains("water_animation_trail_settings")) {
        path.waterAnimationTrailSettings =
            ParseWaterAnimationTrailSettings(pathJson.at("water_animation_trail_settings"));
    }
    if (pathJson.contains("temp_water_animation_trail_settings")) {
        path.tempWaterAnimationTrailSettings =
            ParseWaterAnimationTrailSettings(pathJson.at("temp_water_animation_trail_settings"));
    }
    if (pathJson.contains("water_point_visual_style")) {
        path.waterPointVisualStyle = ParsePointCloudStyle(pathJson.at("water_point_visual_style"));
    }
    if (pathJson.contains("temp_water_point_visual_style")) {
        path.tempWaterPointVisualStyle = ParsePointCloudStyle(pathJson.at("temp_water_point_visual_style"));
    }
    if (pathJson.contains("water_caustic_look_settings")) {
        path.waterCausticLookSettings =
            ParseWaterCausticLookSettings(pathJson.at("water_caustic_look_settings"));
    }
    if (pathJson.contains("temp_water_caustic_look_settings")) {
        path.tempWaterCausticLookSettings =
            ParseWaterCausticLookSettings(pathJson.at("temp_water_caustic_look_settings"));
    }
    if (pathJson.contains("water_visual_settings")) {
        path.waterVisualSettings = ParseWaterVisualSettings(pathJson.at("water_visual_settings"));
        if (!path.waterAnimationTrailSettings.has_value()) {
            WaterAnimationTrailSettings trailSettings;
            trailSettings.colorVariation = path.waterVisualSettings->colorVariation;
            path.waterAnimationTrailSettings = trailSettings;
        }
        if (!path.waterPointVisualStyle.has_value()) {
            path.waterPointVisualStyle = MakeLegacyWaterPointVisualStyle(path.waterVisualSettings.value());
        }
    } else if (pathJson.contains("water_settings")) {
        const auto legacySettings = ParseWaterSettingsBundle(pathJson.at("water_settings"));
        path.waterVisualSettings = legacySettings.visual;
        path.waterSettings = legacySettings;
        if (!path.waterAnimationTrailSettings.has_value()) {
            WaterAnimationTrailSettings trailSettings;
            trailSettings.particleDensity = legacySettings.trail.particleDensity;
            trailSettings.particleSpeed = legacySettings.trail.particleSpeed;
            trailSettings.colorVariation = legacySettings.visual.colorVariation;
            path.waterAnimationTrailSettings = trailSettings;
        }
        if (!path.waterPointVisualStyle.has_value()) {
            path.waterPointVisualStyle = MakeLegacyWaterPointVisualStyle(legacySettings.visual);
        }
    }
    if (pathJson.contains("temp_water_visual_settings")) {
        path.tempWaterVisualSettings = ParseWaterVisualSettings(pathJson.at("temp_water_visual_settings"));
        if (!path.tempWaterAnimationTrailSettings.has_value()) {
            auto trailSettings = path.waterAnimationTrailSettings.value_or(WaterAnimationTrailSettings{});
            trailSettings.colorVariation = path.tempWaterVisualSettings->colorVariation;
            path.tempWaterAnimationTrailSettings = trailSettings;
        }
        if (!path.tempWaterPointVisualStyle.has_value()) {
            path.tempWaterPointVisualStyle =
                MakeLegacyWaterPointVisualStyle(path.tempWaterVisualSettings.value());
        }
    } else if (pathJson.contains("temp_water_settings")) {
        const auto legacySettings = ParseWaterSettingsBundle(pathJson.at("temp_water_settings"));
        path.tempWaterVisualSettings = legacySettings.visual;
        path.tempWaterSettings = legacySettings;
        if (!path.tempWaterAnimationTrailSettings.has_value()) {
            WaterAnimationTrailSettings trailSettings;
            trailSettings.particleDensity = legacySettings.trail.particleDensity;
            trailSettings.particleSpeed = legacySettings.trail.particleSpeed;
            trailSettings.colorVariation = legacySettings.visual.colorVariation;
            path.tempWaterAnimationTrailSettings = trailSettings;
        }
        if (!path.tempWaterPointVisualStyle.has_value()) {
            path.tempWaterPointVisualStyle = MakeLegacyWaterPointVisualStyle(legacySettings.visual);
        }
    }
    if (pathJson.contains("keys")) {
        for (const auto& keyJson : pathJson.at("keys")) {
            path.keys.push_back(ParseAnimationPathKey(keyJson));
        }
    }
    if (pathJson.contains("water_scenario_tracks") &&
        pathJson.at("water_scenario_tracks").is_array()) {
        for (const auto& trackJson : pathJson.at("water_scenario_tracks")) {
            path.waterScenarioTracks.push_back(ParseWaterScenarioTrack(trackJson));
        }
    }
    EnsureAnimationPathKeyIds(&path);
    return path;
}

json SerializeRenderJobSettings(const RenderJobSettings& settings) {
    return json{
        {"output_directory", settings.outputDirectory},
        {"width", settings.width},
        {"height", settings.height},
        {"fps", settings.framesPerSecond},
        {"still_camera_duration_seconds", settings.stillCameraDurationSeconds},
        {"tile_size", settings.tileSize},
        {"start_frame", settings.startFrame},
        {"end_frame", settings.endFrame},
        {"from_shot_index", settings.fromShotIndex},
        {"to_shot_index", settings.toShotIndex},
        {"supersample_scale", settings.supersampleScale},
        {"spatial_antialiasing", settings.spatialAntialiasing},
        {"temporal_supersampling", settings.temporalSupersampling},
        {"temporal_sample_count", settings.temporalSampleCount},
        {"motion_blur", settings.motionBlur},
        {"motion_blur_sample_count", settings.motionBlurSampleCount},
        {"motion_blur_shutter_angle_degrees", settings.motionBlurShutterAngleDegrees},
    };
}

RenderJobSettings ParseRenderJobSettings(const json& settingsJson) {
    RenderJobSettings settings;
    settings.outputDirectory = settingsJson.value("output_directory", std::string{});
    settings.width = settingsJson.value("width", 1920U);
    settings.height = settingsJson.value("height", 1080U);
    settings.framesPerSecond = settingsJson.value("fps", 30U);
    settings.stillCameraDurationSeconds =
        std::clamp(settingsJson.value("still_camera_duration_seconds", settings.stillCameraDurationSeconds), 0.001F, 3600.0F);
    settings.tileSize = settingsJson.value("tile_size", 512U);
    settings.startFrame = settingsJson.value("start_frame", 0U);
    settings.endFrame = settingsJson.value("end_frame", 0U);
    settings.fromShotIndex = settingsJson.value("from_shot_index", static_cast<std::size_t>(0U));
    settings.toShotIndex = settingsJson.value("to_shot_index", static_cast<std::size_t>(1U));
    settings.supersampleScale =
        std::clamp(settingsJson.value("supersample_scale", settings.supersampleScale), 1U, 8U);
    settings.spatialAntialiasing =
        settingsJson.value("spatial_antialiasing", settings.spatialAntialiasing);
    settings.temporalSupersampling =
        settingsJson.value("temporal_supersampling", settings.temporalSupersampling);
    settings.temporalSampleCount =
        std::clamp(settingsJson.value("temporal_sample_count", settings.temporalSampleCount), 1U, 64U);
    settings.motionBlur = settingsJson.value("motion_blur", settings.motionBlur);
    settings.motionBlurSampleCount =
        std::clamp(settingsJson.value("motion_blur_sample_count", settings.motionBlurSampleCount), 1U, 64U);
    settings.motionBlurShutterAngleDegrees =
        std::clamp(
            settingsJson.value(
                "motion_blur_shutter_angle_degrees",
                settings.motionBlurShutterAngleDegrees),
            0.0F,
            360.0F);
    return settings;
}

const char* AnimationExportModeName(AnimationExportMode mode) {
    switch (mode) {
        case AnimationExportMode::FastPreviewMp4:
            return "fast_preview_mp4";
        case AnimationExportMode::HevcAlphaMp4:
            return "hevc_alpha_mp4";
        case AnimationExportMode::PngStack:
            return "png_stack";
        case AnimationExportMode::FastPngStack:
            return "fast_png_stack";
        case AnimationExportMode::HqPreviewDensityExr:
            return "hq_preview_density_exr";
        case AnimationExportMode::ProRes422Mov:
            return "prores_422_mov";
        case AnimationExportMode::ProRes422HqMov:
            return "prores_422_hq_mov";
        case AnimationExportMode::ProRes422AlphaMatteMov:
            return "prores_422_alpha_matte_mov";
        case AnimationExportMode::ProRes422HqAlphaMatteMov:
            return "prores_422_hq_alpha_matte_mov";
        case AnimationExportMode::ProRes422VideoToolboxMov:
            return "prores_422_videotoolbox_mov";
        case AnimationExportMode::ProRes422HqVideoToolboxMov:
            return "prores_422_hq_videotoolbox_mov";
        case AnimationExportMode::ProRes4444Mov:
            return "prores_4444_mov";
        case AnimationExportMode::ProRes4444XqMov:
            return "prores_4444_xq_mov";
        case AnimationExportMode::ProRes4444VideoToolboxMov:
            return "prores_4444_videotoolbox_mov";
        case AnimationExportMode::ProRes4444XqVideoToolboxMov:
            return "prores_4444_xq_videotoolbox_mov";
    }

    return "fast_preview_mp4";
}

const char* AnimationExportQualityName(invisible_places::output::AnimationExportQuality quality) {
    switch (quality) {
        case invisible_places::output::AnimationExportQuality::Normal:
            return "normal";
        case invisible_places::output::AnimationExportQuality::Hq:
            return "hq";
        case invisible_places::output::AnimationExportQuality::Xq:
            return "xq";
    }
    return "normal";
}

invisible_places::output::AnimationExportQuality ParseAnimationExportQuality(const json& qualityJson) {
    const auto quality = qualityJson.is_string() ? qualityJson.get<std::string>() : std::string{"normal"};
    if (quality == "hq" || quality == "high" || quality == "high_quality") {
        return invisible_places::output::AnimationExportQuality::Hq;
    }
    if (quality == "xq" || quality == "extra_quality" || quality == "maximum") {
        return invisible_places::output::AnimationExportQuality::Xq;
    }
    return invisible_places::output::AnimationExportQuality::Normal;
}

AnimationExportMode ParseAnimationExportMode(const json& modeJson) {
    const auto mode = modeJson.is_string()
                          ? modeJson.get<std::string>()
                          : std::string{AnimationExportModeName(AnimationExportMode::FastPreviewMp4)};
    if (mode == "hq_preview_density_exr" || mode == "hq_exr") {
        return AnimationExportMode::HqPreviewDensityExr;
    }
    if (mode == "hevc_alpha_mp4" || mode == "h265_alpha_mp4" || mode == "h_265_alpha_mp4") {
        return AnimationExportMode::HevcAlphaMp4;
    }
    if (mode == "png_stack" || mode == "png_sequence") {
        return AnimationExportMode::PngStack;
    }
    if (mode == "fast_png_stack" || mode == "fast_png_sequence") {
        return AnimationExportMode::FastPngStack;
    }
    if (mode == "prores_422_mov" || mode == "prores_422") {
        return AnimationExportMode::ProRes422Mov;
    }
    if (mode == "prores_422_hq_mov" || mode == "prores_422_hq") {
        return AnimationExportMode::ProRes422HqMov;
    }
    if (mode == "prores_422_alpha_matte_mov" ||
        mode == "prores_422_alpha_matte" ||
        mode == "prores_422_alpha_mov" ||
        mode == "prores_422_alpha") {
        return AnimationExportMode::ProRes422AlphaMatteMov;
    }
    if (mode == "prores_422_hq_alpha_matte_mov" ||
        mode == "prores_422_hq_alpha_matte" ||
        mode == "prores_422_hq_alpha_mov" ||
        mode == "prores_422_hq_alpha") {
        return AnimationExportMode::ProRes422HqAlphaMatteMov;
    }
    if (mode == "prores_422_videotoolbox_mov" || mode == "prores_422_videotoolbox") {
        return AnimationExportMode::ProRes422VideoToolboxMov;
    }
    if (mode == "prores_422_hq_videotoolbox_mov" || mode == "prores_422_hq_videotoolbox") {
        return AnimationExportMode::ProRes422HqVideoToolboxMov;
    }
    if (mode == "prores_4444_mov" || mode == "prores_4444") {
        return AnimationExportMode::ProRes4444Mov;
    }
    if (mode == "prores_4444_xq_mov" || mode == "prores_4444_xq") {
        return AnimationExportMode::ProRes4444XqMov;
    }
    if (mode == "prores_4444_videotoolbox_mov" || mode == "prores_4444_videotoolbox") {
        return AnimationExportMode::ProRes4444VideoToolboxMov;
    }
    if (mode == "prores_4444_xq_videotoolbox_mov" || mode == "prores_4444_xq_videotoolbox") {
        return AnimationExportMode::ProRes4444XqVideoToolboxMov;
    }
    return AnimationExportMode::FastPreviewMp4;
}

json SerializeExportPreset(const ExportPreset& preset) {
    return json{
        {"name", preset.name},
        {"mode", AnimationExportModeName(preset.mode)},
        {"quality", AnimationExportQualityName(preset.quality)},
        {"use_video_toolbox", preset.useVideoToolbox},
        {"external_alpha_matte", preset.externalAlphaMatte},
        {"settings", SerializeRenderJobSettings(preset.settings)},
    };
}

ExportPreset ParseExportPreset(const json& presetJson) {
    ExportPreset preset = invisible_places::output::MakeFastPreviewMp4ExportPreset();
    preset.name = presetJson.value("name", preset.name);
    if (presetJson.contains("mode")) {
        preset.mode = ParseAnimationExportMode(presetJson.at("mode"));
    }
    if (presetJson.contains("quality")) {
        preset.quality = ParseAnimationExportQuality(presetJson.at("quality"));
    }
    preset.useVideoToolbox = presetJson.value("use_video_toolbox", preset.useVideoToolbox);
    preset.externalAlphaMatte = presetJson.value("external_alpha_matte", preset.externalAlphaMatte);
    if (presetJson.contains("settings")) {
        preset.settings = ParseRenderJobSettings(presetJson.at("settings"));
    }
    return invisible_places::output::NormalizeExportPresetForCurrentSchema(std::move(preset));
}

json SerializeSavedAnimation(const ProjectDocument::SavedAnimation& animation) {
    return json{
        {"file_path", animation.filePath.generic_string()},
        {"associated_layer_paths", SerializePathArray(animation.associatedLayerPaths)},
    };
}

ProjectDocument::SavedAnimation ParseSavedAnimation(const json& animationJson) {
    ProjectDocument::SavedAnimation animation;
    animation.filePath = animationJson.value("file_path", std::string{});
    if (animationJson.contains("associated_layer_paths")) {
        animation.associatedLayerPaths = ParsePathArray(animationJson.at("associated_layer_paths"));
    }
    return animation;
}

std::string SerializedWaterScaleModeName(WaterScaleMode mode) {
    return invisible_places::water::WaterScaleModeName(mode);
}

WaterScaleMode ParseWaterScaleMode(const json& modeJson) {
    const auto modeName = modeJson.get<std::string>();
    if (modeName == "aerial") {
        return WaterScaleMode::Aerial;
    }
    if (modeName == "detail") {
        return WaterScaleMode::Detail;
    }
    return WaterScaleMode::Mid;
}

std::string SerializedWaterEmitterOriginName(WaterEmitterOrigin origin) {
    return invisible_places::water::WaterEmitterOriginName(origin);
}

WaterEmitterOrigin ParseWaterEmitterOrigin(const json& originJson) {
    const auto originName = originJson.get<std::string>();
    if (originName == "auto") {
        return WaterEmitterOrigin::AutoSuggested;
    }
    if (originName == "propagated") {
        return WaterEmitterOrigin::Propagated;
    }
    return WaterEmitterOrigin::Manual;
}

std::string SerializedWaterEmitterStatusName(WaterEmitterStatus status) {
    return invisible_places::water::WaterEmitterStatusName(status);
}

WaterEmitterStatus ParseWaterEmitterStatus(const json& statusJson) {
    const auto statusName = statusJson.get<std::string>();
    if (statusName == "candidate") {
        return WaterEmitterStatus::Candidate;
    }
    if (statusName == "disabled") {
        return WaterEmitterStatus::Disabled;
    }
    return WaterEmitterStatus::Accepted;
}

std::string SerializedWaterSourceSettingsAssignmentName(WaterSourceSettingsAssignment assignment) {
    switch (assignment) {
        case WaterSourceSettingsAssignment::Default:
            return "default";
        case WaterSourceSettingsAssignment::Custom:
            return "custom";
        case WaterSourceSettingsAssignment::LinkedEmitter:
            return "linked_emitter";
    }
    return "default";
}

WaterSourceSettingsAssignment ParseWaterSourceSettingsAssignment(const json& assignmentJson) {
    const auto assignmentName = assignmentJson.get<std::string>();
    if (assignmentName == "custom") {
        return WaterSourceSettingsAssignment::Custom;
    }
    if (assignmentName == "linked_emitter") {
        return WaterSourceSettingsAssignment::LinkedEmitter;
    }
    return WaterSourceSettingsAssignment::Default;
}

json SerializeWaterEmitter(const WaterEmitter& emitter) {
    json emitterJson{
        {"id", emitter.id},
        {"name", emitter.name},
        {"position", std::array<float, 3>{emitter.position.x, emitter.position.y, emitter.position.z}},
        {"radius", emitter.radius},
        {"strength", emitter.strength},
        {"speed", emitter.speed},
        {"origin", SerializedWaterEmitterOriginName(emitter.origin)},
        {"status", SerializedWaterEmitterStatusName(emitter.status)},
        {"confidence", emitter.confidence},
        {"settings_assignment", SerializedWaterSourceSettingsAssignmentName(emitter.sourceSettingsAssignment)},
        {"path_profile", emitter.pathProfileName},
        {"lane_profile", emitter.laneProfileName},
        {"trail_profile", emitter.trailProfileName},
        {"path_profile_locked", emitter.pathProfileLocked},
        {"lane_profile_locked", emitter.laneProfileLocked},
        {"trail_profile_locked", emitter.trailProfileLocked},
        {"maximum_flow_strength", emitter.maximumFlowStrength},
        {"rain_response", emitter.rainResponse},
        {"show_trail", emitter.showTrail},
    };
    if (emitter.parentId.has_value()) {
        emitterJson["parent_id"] = emitter.parentId.value();
    }
    if (emitter.sourceSettingsAssignment == WaterSourceSettingsAssignment::LinkedEmitter &&
        emitter.linkedSourceSettingsEmitterId.has_value()) {
        emitterJson["linked_settings_emitter_id"] = emitter.linkedSourceSettingsEmitterId.value();
    }
    if (emitter.sourceSettings.has_value()) {
        emitterJson["source_settings"] = SerializeWaterSourceSettings(emitter.sourceSettings.value());
    }
    if (emitter.tempSourceSettings.has_value()) {
        emitterJson["temp_source_settings"] = SerializeWaterSourceSettings(emitter.tempSourceSettings.value());
    }
    return emitterJson;
}

WaterEmitter ParseWaterEmitter(const json& emitterJson) {
    WaterEmitter emitter;
    emitter.id = emitterJson.value("id", 0U);
    emitter.name = emitterJson.value("name", std::string{"Water Source"});
    if (emitterJson.contains("position")) {
        const auto position = emitterJson.at("position").get<std::array<float, 3>>();
        emitter.position = {position[0], position[1], position[2]};
    }
    emitter.radius = emitterJson.value("radius", emitter.radius);
    emitter.strength = emitterJson.value("strength", emitter.strength);
    emitter.speed = emitterJson.value("speed", emitter.speed);
    if (emitterJson.contains("scope")) {
        emitter.scope = ParseWaterScaleMode(emitterJson.at("scope"));
    }
    if (emitterJson.contains("origin")) {
        emitter.origin = ParseWaterEmitterOrigin(emitterJson.at("origin"));
    }
    if (emitterJson.contains("status")) {
        emitter.status = ParseWaterEmitterStatus(emitterJson.at("status"));
    }
    emitter.confidence = emitterJson.value("confidence", emitter.confidence);
    emitter.pathProfileName = emitterJson.value("path_profile", emitter.pathProfileName);
    emitter.laneProfileName = emitterJson.value("lane_profile", emitter.laneProfileName);
    emitter.trailProfileName = emitterJson.value("trail_profile", emitter.trailProfileName);
    emitter.pathProfileLocked = emitterJson.value("path_profile_locked", emitter.pathProfileLocked);
    emitter.laneProfileLocked = emitterJson.value("lane_profile_locked", emitter.laneProfileLocked);
    emitter.trailProfileLocked = emitterJson.value("trail_profile_locked", emitter.trailProfileLocked);
    emitter.maximumFlowStrength = std::clamp(
        emitterJson.value("maximum_flow_strength", emitter.maximumFlowStrength),
        0.0F,
        1.0F);
    emitter.rainResponse = std::clamp(
        emitterJson.value("rain_response", emitter.rainResponse),
        0.0F,
        1.0F);
    emitter.showTrail = emitterJson.value("show_trail", emitter.showTrail);
    if (emitterJson.contains("parent_id")) {
        emitter.parentId = emitterJson.at("parent_id").get<std::uint32_t>();
    }
    const bool hasExplicitSettingsAssignment = emitterJson.contains("settings_assignment");
    if (hasExplicitSettingsAssignment) {
        emitter.sourceSettingsAssignment =
            ParseWaterSourceSettingsAssignment(emitterJson.at("settings_assignment"));
    }
    if (emitterJson.contains("linked_settings_emitter_id")) {
        emitter.linkedSourceSettingsEmitterId =
            emitterJson.at("linked_settings_emitter_id").get<std::uint32_t>();
    }
    if (emitterJson.contains("source_settings")) {
        emitter.sourceSettings = ParseWaterSourceSettings(emitterJson.at("source_settings"));
    }
    if (emitterJson.contains("temp_source_settings")) {
        emitter.tempSourceSettings = ParseWaterSourceSettings(emitterJson.at("temp_source_settings"));
    }
    if (!hasExplicitSettingsAssignment &&
        (emitter.sourceSettings.has_value() || emitter.tempSourceSettings.has_value())) {
        emitter.sourceSettingsAssignment = WaterSourceSettingsAssignment::Custom;
    }
    if (emitter.sourceSettingsAssignment != WaterSourceSettingsAssignment::LinkedEmitter) {
        emitter.linkedSourceSettingsEmitterId.reset();
    }
    return emitter;
}

json SerializeWaterManualFlowPath(const WaterManualFlowPathSource& source) {
    json sourceJson{
        {"id", source.id},
        {"name", source.name},
        {"lane_profile", source.laneProfileName},
        {"trail_profile", source.trailProfileName},
        {"lane_profile_locked", source.laneProfileLocked},
        {"trail_profile_locked", source.trailProfileLocked},
        {"use_surface_guide", source.useSurfaceGuide},
        {"maximum_flow_strength", source.maximumFlowStrength},
        {"rain_response", source.rainResponse},
        {"show_trail", source.showTrail},
        {"control_points", json::array()},
    };
    for (const auto& point : source.controlPoints) {
        sourceJson["control_points"].push_back(std::array<float, 3>{point.x, point.y, point.z});
    }
    return sourceJson;
}

WaterManualFlowPathSource ParseWaterManualFlowPath(
    const json& sourceJson,
    bool defaultUseSurfaceGuide) {
    WaterManualFlowPathSource source;
    source.id = sourceJson.value("id", source.id);
    source.name = sourceJson.value("name", source.name);
    source.laneProfileName = sourceJson.value("lane_profile", source.laneProfileName);
    source.trailProfileName = sourceJson.value("trail_profile", source.trailProfileName);
    source.laneProfileLocked = sourceJson.value("lane_profile_locked", source.laneProfileLocked);
    source.trailProfileLocked = sourceJson.value("trail_profile_locked", source.trailProfileLocked);
    source.useSurfaceGuide = sourceJson.value("use_surface_guide", defaultUseSurfaceGuide);
    source.maximumFlowStrength = std::clamp(
        sourceJson.value("maximum_flow_strength", source.maximumFlowStrength),
        0.0F,
        1.0F);
    source.rainResponse = std::clamp(
        sourceJson.value("rain_response", source.rainResponse),
        0.0F,
        1.0F);
    source.showTrail = sourceJson.value("show_trail", source.showTrail);
    if (sourceJson.contains("control_points") && sourceJson.at("control_points").is_array()) {
        for (const auto& pointJson : sourceJson.at("control_points")) {
            if (!pointJson.is_array() || pointJson.size() != 3U) {
                continue;
            }
            const auto point = pointJson.get<std::array<float, 3>>();
            source.controlPoints.push_back({point[0], point[1], point[2]});
        }
    }
    return source;
}

json SerializeWaterRegionVertices(const std::vector<invisible_places::io::Float3>& vertices) {
    json verticesJson = json::array();
    for (const auto& vertex : vertices) {
        verticesJson.push_back(std::array<float, 3>{vertex.x, vertex.y, vertex.z});
    }
    return verticesJson;
}

std::vector<invisible_places::io::Float3> ParseWaterRegionVertices(const json& verticesJson) {
    std::vector<invisible_places::io::Float3> vertices;
    if (!verticesJson.is_array()) {
        return vertices;
    }
    vertices.reserve(verticesJson.size());
    for (const auto& vertexJson : verticesJson) {
        const auto vertex = vertexJson.get<std::array<float, 3>>();
        vertices.push_back({vertex[0], vertex[1], vertex[2]});
    }
    return vertices;
}

struct LegacyCausticRegion {
    std::uint32_t id = 0;
    std::string name = "Caustics";
    std::filesystem::path targetLayerSourcePath;
    std::vector<invisible_places::io::Float3> vertices;
    std::vector<invisible_places::io::Float3> hull;
    float edgeBlendWidth = 0.60F;
    bool enabled = true;
};

LegacyCausticRegion ParseLegacyCausticRegion(const json& regionJson) {
    LegacyCausticRegion region;
    region.id = regionJson.value("id", 0U);
    region.name = regionJson.value("name", region.name);
    region.targetLayerSourcePath = regionJson.value("target_layer_source_path", std::string{});
    if (regionJson.contains("vertices")) {
        region.vertices = ParseWaterRegionVertices(regionJson.at("vertices"));
    }
    if (regionJson.contains("hull")) {
        region.hull = ParseWaterRegionVertices(regionJson.at("hull"));
    }
    region.edgeBlendWidth =
        std::clamp(regionJson.value("edge_blend_width", region.edgeBlendWidth), 0.01F, 50.0F);
    region.enabled = regionJson.value("enabled", region.enabled);
    if (region.hull.empty()) {
        region.hull = invisible_places::water::BuildWaterRegionHull(region.vertices);
    }
    return region;
}

const char* WaterEffectFeatureTypeName(WaterEffectFeatureType type) {
    switch (type) {
        case WaterEffectFeatureType::FieldBridgeAllowedRegion:
            return "field_bridge_allowed_region";
        case WaterEffectFeatureType::FieldBridgeBlockedRegion:
            return "field_bridge_blocked_region";
        case WaterEffectFeatureType::FieldNoFlowRegion:
            return "field_no_flow_region";
        case WaterEffectFeatureType::FieldSurfaceMotion:
            return "field_surface_motion";
        case WaterEffectFeatureType::Ripple:
            return "ripple";
    }
    return "ripple";
}

WaterEffectFeatureType ParseWaterEffectFeatureType(const json& typeJson) {
    const auto name = typeJson.get<std::string>();
    if (name == "field_bridge_allowed_region") {
        return WaterEffectFeatureType::FieldBridgeAllowedRegion;
    }
    if (name == "field_bridge_blocked_region") {
        return WaterEffectFeatureType::FieldBridgeBlockedRegion;
    }
    if (name == "field_no_flow_region") {
        return WaterEffectFeatureType::FieldNoFlowRegion;
    }
    if (name == "field_surface_motion") {
        return WaterEffectFeatureType::FieldSurfaceMotion;
    }
    return WaterEffectFeatureType::Ripple;
}

const char* WaterRippleOverlayTypeName(WaterRippleOverlayType type) {
    return invisible_places::water::WaterRippleOverlayTypeNameForStorage(type).data();
}

WaterRippleOverlayType ParseWaterRippleOverlayType(const json& typeJson) {
    const auto name = typeJson.get<std::string>();
    return invisible_places::water::ParseWaterRippleOverlayTypeName(name)
        .value_or(WaterRippleOverlayType::CausticLace);
}

const char* WaterEffectBlendModeName(WaterEffectBlendMode mode) {
    switch (mode) {
        case WaterEffectBlendMode::Max:
            return "max";
        case WaterEffectBlendMode::Multiply:
            return "multiply";
        case WaterEffectBlendMode::Screen:
            return "screen";
        case WaterEffectBlendMode::Override:
            return "override";
        case WaterEffectBlendMode::Add:
            return "add";
    }
    return "add";
}

WaterEffectBlendMode ParseWaterEffectBlendMode(const json& modeJson) {
    const auto name = modeJson.get<std::string>();
    if (name == "max") {
        return WaterEffectBlendMode::Max;
    }
    if (name == "multiply") {
        return WaterEffectBlendMode::Multiply;
    }
    if (name == "screen") {
        return WaterEffectBlendMode::Screen;
    }
    if (name == "override") {
        return WaterEffectBlendMode::Override;
    }
    return WaterEffectBlendMode::Add;
}

const char* WaterFieldOutputModeName(WaterFieldOutputMode mode) {
    switch (mode) {
        case WaterFieldOutputMode::Trails:
            return "trails";
        case WaterFieldOutputMode::SurfaceMotion:
            return "surface_motion";
        case WaterFieldOutputMode::Both:
            return "both";
    }
    return "both";
}

WaterFieldOutputMode ParseWaterFieldOutputMode(const json& modeJson) {
    const auto name = modeJson.get<std::string>();
    if (name == "trails" || name == "streamlines") {
        return WaterFieldOutputMode::Trails;
    }
    if (name == "surface_motion") {
        return WaterFieldOutputMode::SurfaceMotion;
    }
    return WaterFieldOutputMode::Both;
}

json SerializeWaterEffectResponseSettings(const WaterEffectResponseSettings& settings) {
    return json{
        {"intensity", settings.intensity},
        {"emission_add", settings.emissionAdd},
        {"opacity_add", settings.opacityAdd},
        {"opacity_multiply", settings.opacityMultiply},
        {"point_size_add", settings.pointSizeAdd},
        {"point_size_multiply", settings.pointSizeMultiply},
        {"hue_shift", settings.hueShift},
        {"colourise", {settings.colouriseRed, settings.colouriseGreen, settings.colouriseBlue}},
        {"colourise_amount", settings.colouriseAmount},
        {"gaussian_sharpness_bias", settings.gaussianSharpnessBias},
    };
}

WaterEffectResponseSettings ParseWaterEffectResponseSettings(const json& settingsJson) {
    WaterEffectResponseSettings settings;
    settings.intensity = settingsJson.value("intensity", settings.intensity);
    settings.emissionAdd = settingsJson.value("emission_add", settings.emissionAdd);
    settings.opacityAdd = settingsJson.value("opacity_add", settings.opacityAdd);
    settings.opacityMultiply = settingsJson.value("opacity_multiply", settings.opacityMultiply);
    settings.pointSizeAdd = settingsJson.value("point_size_add", settings.pointSizeAdd);
    settings.pointSizeMultiply = settingsJson.value("point_size_multiply", settings.pointSizeMultiply);
    settings.hueShift = settingsJson.value("hue_shift", settings.hueShift);
    if (settingsJson.contains("colourise")) {
        const auto colour = settingsJson.at("colourise").get<std::array<float, 3>>();
        settings.colouriseRed = colour[0];
        settings.colouriseGreen = colour[1];
        settings.colouriseBlue = colour[2];
    }
    settings.colouriseAmount = settingsJson.value("colourise_amount", settings.colouriseAmount);
    settings.gaussianSharpnessBias = settingsJson.value(
        "gaussian_sharpness_bias",
        settings.gaussianSharpnessBias);
    return settings;
}

const char* WaterSeepageQualityName(WaterSeepageQuality quality) {
    switch (quality) {
        case WaterSeepageQuality::Low:
            return "low";
        case WaterSeepageQuality::Balanced:
            return "balanced";
        case WaterSeepageQuality::High:
            return "high";
        case WaterSeepageQuality::Auto:
            return "auto";
    }
    return "auto";
}

WaterSeepageQuality ParseWaterSeepageQuality(const json& qualityJson) {
    const auto name = qualityJson.get<std::string>();
    if (name == "low") {
        return WaterSeepageQuality::Low;
    }
    if (name == "balanced") {
        return WaterSeepageQuality::Balanced;
    }
    if (name == "high") {
        return WaterSeepageQuality::High;
    }
    return WaterSeepageQuality::Auto;
}

const char* WaterSeepagePatternName(WaterSeepagePattern pattern) {
    switch (pattern) {
        case WaterSeepagePattern::LegacyRipples:
            return "legacy_ripples";
        case WaterSeepagePattern::WetRockSheen:
            return "wet_rock_sheen";
        case WaterSeepagePattern::ChaoticBloom:
            return "chaotic_bloom";
        case WaterSeepagePattern::WettingTrickle:
            return "wetting_trickle";
    }
    return "legacy_ripples";
}

WaterSeepagePattern ParseWaterSeepagePattern(const json& patternJson) {
    if (!patternJson.is_string()) {
        return WaterSeepagePattern::LegacyRipples;
    }
    const auto name = patternJson.get<std::string>();
    if (name == "wet_rock_sheen") {
        return WaterSeepagePattern::WetRockSheen;
    }
    if (name == "chaotic_bloom") {
        return WaterSeepagePattern::ChaoticBloom;
    }
    if (name == "wetting_trickle") {
        return WaterSeepagePattern::WettingTrickle;
    }
    return WaterSeepagePattern::LegacyRipples;
}

json SerializeWaterSeepageLookSettings(const WaterSeepageLookSettings& settings) {
    return json{
        {"quality", WaterSeepageQualityName(settings.quality)},
        {"pattern", WaterSeepagePatternName(settings.pattern)},
        {"base_wetness", settings.baseWetness},
        {"density", settings.density},
        {"glisten", settings.glisten},
        {"pattern_scale", settings.patternScale},
        {"wavelength_meters", settings.wavelengthMeters},
        {"speed", settings.speed},
        {"warp", settings.warp},
        {"turbulence", settings.turbulence},
        {"phase", settings.phase},
        {"rain_response", settings.rainResponse},
        {"feature_size_meters", settings.featureSizeMeters},
        {"contrast", settings.contrast},
        {"evolution", settings.evolution},
        {"roughness", settings.roughness},
        {"angle_response", settings.angleResponse},
        {"micro_normal_strength", settings.microNormalStrength},
        {"glint_density", settings.glintDensity},
        {"environment_azimuth_degrees", settings.environmentAzimuthDegrees},
        {"environment_elevation_degrees", settings.environmentElevationDegrees},
        {"curl", settings.curl},
        {"breakup", settings.breakup},
        {"downhill_drift_meters_per_second", settings.downhillDriftMetersPerSecond},
        {"trickle_patch_size_meters", settings.tricklePatchSizeMeters},
        {"trickle_length_meters", settings.trickleLengthMeters},
        {"trickle_width_meters", settings.trickleWidthMeters},
        {"trickle_front_softness", settings.trickleFrontSoftness},
        {"blend_mode", WaterEffectBlendModeName(settings.blendMode)},
        {"response", SerializeWaterEffectResponseSettings(settings.response)},
    };
}

WaterSeepageLookSettings ParseWaterSeepageLookSettings(const json& settingsJson) {
    WaterSeepageLookSettings settings;
    // Documents written before pattern selection existed used the legacy
    // projected ripple evaluator. Preserve that appearance exactly.
    settings.pattern = WaterSeepagePattern::LegacyRipples;
    if (settingsJson.contains("quality")) {
        settings.quality = ParseWaterSeepageQuality(settingsJson.at("quality"));
    }
    if (settingsJson.contains("pattern")) {
        settings.pattern = ParseWaterSeepagePattern(settingsJson.at("pattern"));
    }
    settings.baseWetness = settingsJson.value("base_wetness", settings.baseWetness);
    settings.density = settingsJson.value("density", settings.density);
    settings.glisten = settingsJson.value("glisten", settings.glisten);
    settings.patternScale = settingsJson.value("pattern_scale", settings.patternScale);
    settings.wavelengthMeters = settingsJson.value("wavelength_meters", settings.wavelengthMeters);
    settings.speed = settingsJson.value("speed", settings.speed);
    settings.warp = settingsJson.value("warp", settings.warp);
    settings.turbulence = settingsJson.value("turbulence", settings.turbulence);
    settings.phase = settingsJson.value("phase", settings.phase);
    settings.rainResponse = settingsJson.value("rain_response", settings.rainResponse);
    settings.featureSizeMeters = settingsJson.value("feature_size_meters", settings.featureSizeMeters);
    settings.contrast = settingsJson.value("contrast", settings.contrast);
    settings.evolution = settingsJson.value("evolution", settings.evolution);
    settings.roughness = settingsJson.value("roughness", settings.roughness);
    settings.angleResponse = settingsJson.value("angle_response", settings.angleResponse);
    settings.microNormalStrength = settingsJson.value(
        "micro_normal_strength",
        settings.microNormalStrength);
    settings.glintDensity = settingsJson.value("glint_density", settings.glintDensity);
    settings.environmentAzimuthDegrees = settingsJson.value(
        "environment_azimuth_degrees",
        settings.environmentAzimuthDegrees);
    settings.environmentElevationDegrees = settingsJson.value(
        "environment_elevation_degrees",
        settings.environmentElevationDegrees);
    settings.curl = settingsJson.value("curl", settings.curl);
    settings.breakup = settingsJson.value("breakup", settings.breakup);
    settings.downhillDriftMetersPerSecond = settingsJson.value(
        "downhill_drift_meters_per_second",
        settings.downhillDriftMetersPerSecond);
    settings.tricklePatchSizeMeters = settingsJson.value(
        "trickle_patch_size_meters",
        settings.tricklePatchSizeMeters);
    settings.trickleLengthMeters = settingsJson.value(
        "trickle_length_meters",
        settings.trickleLengthMeters);
    settings.trickleWidthMeters = settingsJson.value(
        "trickle_width_meters",
        settings.trickleWidthMeters);
    settings.trickleFrontSoftness = settingsJson.value(
        "trickle_front_softness",
        settings.trickleFrontSoftness);
    if (settingsJson.contains("blend_mode")) {
        settings.blendMode = ParseWaterEffectBlendMode(settingsJson.at("blend_mode"));
    }
    if (settingsJson.contains("response")) {
        settings.response = ParseWaterEffectResponseSettings(settingsJson.at("response"));
    }
    return settings;
}

json SerializeWaterSeepageLookProfile(const WaterSeepageLookProfile& profile) {
    return json{
        {"name", profile.name},
        {"settings", SerializeWaterSeepageLookSettings(profile.settings)},
    };
}

WaterSeepageLookProfile ParseWaterSeepageLookProfile(const json& profileJson) {
    WaterSeepageLookProfile profile;
    profile.name = profileJson.value("name", profile.name);
    if (profileJson.contains("settings")) {
        profile.settings = ParseWaterSeepageLookSettings(profileJson.at("settings"));
    }
    return profile;
}

const char* WaterScenarioInterpolationName(WaterScenarioInterpolation interpolation) {
    switch (interpolation) {
        case WaterScenarioInterpolation::Smooth:
            return "smooth";
        case WaterScenarioInterpolation::Linear:
            return "linear";
        case WaterScenarioInterpolation::Hold:
            return "hold";
    }
    return "smooth";
}

WaterScenarioInterpolation ParseWaterScenarioInterpolation(const json& interpolationJson) {
    if (!interpolationJson.is_string()) {
        return WaterScenarioInterpolation::Smooth;
    }
    const auto name = interpolationJson.get<std::string>();
    if (name == "linear") {
        return WaterScenarioInterpolation::Linear;
    }
    if (name == "hold") {
        return WaterScenarioInterpolation::Hold;
    }
    return WaterScenarioInterpolation::Smooth;
}

json SerializeWaterScenarioState(const WaterScenarioState& state) {
    return json{
        {"seepage_look", SerializeWaterSeepageLookSettings(state.seepageLook)},
        {"seepage_level", state.seepageLevel},
        {"seepage_spread", state.seepageSpread},
        {"rain_level", state.rainLevel},
        {"flow_level", state.flowLevel},
        {"shoreline_level", state.shorelineLevel},
        {"mesh_flow_level", state.meshFlowLevel},
        {"mesh_flow_rain_gain", state.meshFlowRainGain},
        {"mesh_flow_persistence_scale", state.meshFlowPersistenceScale},
        {"mesh_flow_rain_rise_seconds", state.meshFlowRainRiseSeconds},
        {"mesh_flow_rain_recession_seconds", state.meshFlowRainRecessionSeconds},
        {"seepage_rain_delay_seconds", state.seepageRainDelaySeconds},
        {"seepage_rain_rise_seconds", state.seepageRainRiseSeconds},
        {"seepage_rain_recession_seconds", state.seepageRainRecessionSeconds},
    };
}

WaterScenarioState ParseWaterScenarioState(const json& stateJson) {
    WaterScenarioState state;
    if (stateJson.contains("seepage_look")) {
        state.seepageLook = ParseWaterSeepageLookSettings(stateJson.at("seepage_look"));
    }
    state.seepageLevel = stateJson.value("seepage_level", state.seepageLevel);
    state.seepageSpread = stateJson.value("seepage_spread", state.seepageSpread);
    state.rainLevel = stateJson.value("rain_level", state.rainLevel);
    state.flowLevel = stateJson.value("flow_level", state.flowLevel);
    state.shorelineLevel = stateJson.value("shoreline_level", state.shorelineLevel);
    state.meshFlowLevel = stateJson.value(
        "mesh_flow_level",
        state.meshFlowLevel);
    state.meshFlowRainGain = stateJson.value(
        "mesh_flow_rain_gain",
        state.meshFlowRainGain);
    state.meshFlowPersistenceScale = stateJson.value(
        "mesh_flow_persistence_scale",
        state.meshFlowPersistenceScale);
    state.meshFlowRainRiseSeconds = stateJson.value(
        "mesh_flow_rain_rise_seconds",
        state.meshFlowRainRiseSeconds);
    state.meshFlowRainRecessionSeconds = stateJson.value(
        "mesh_flow_rain_recession_seconds",
        state.meshFlowRainRecessionSeconds);
    state.seepageRainDelaySeconds = stateJson.value(
        "seepage_rain_delay_seconds",
        state.seepageRainDelaySeconds);
    state.seepageRainRiseSeconds = stateJson.value(
        "seepage_rain_rise_seconds",
        state.seepageRainRiseSeconds);
    state.seepageRainRecessionSeconds = stateJson.value(
        "seepage_rain_recession_seconds",
        state.seepageRainRecessionSeconds);
    return invisible_places::water::SanitizeWaterScenarioState(std::move(state));
}

// The seeded scenarios were renamed for the exhibition deliverables
// (Past/Future was Pre-Colonisation Wet, Current was Contemporary Managed).
// Ids are stable, so files carrying the old default display names pick up
// the new ones on load; authored custom names pass through untouched.
std::string MigrateWaterScenarioDisplayName(const std::string& scenarioId, std::string name) {
    if (scenarioId == "pre-colonisation-wet" && name == "Pre-Colonisation Wet") {
        return "Past/Future";
    }
    if (scenarioId == "contemporary-managed" && name == "Contemporary Managed") {
        return "Current";
    }
    return name;
}

json SerializeWaterScenarioDefinition(const WaterScenarioDefinition& definition) {
    return json{
        {"id", definition.id},
        {"name", definition.name},
        {"state", SerializeWaterScenarioState(definition.state)},
    };
}

WaterScenarioDefinition ParseWaterScenarioDefinition(const json& definitionJson) {
    WaterScenarioDefinition definition;
    definition.id = definitionJson.value("id", definition.id);
    definition.name = MigrateWaterScenarioDisplayName(
        definition.id,
        definitionJson.value("name", definition.name));
    if (definitionJson.contains("state")) {
        definition.state = ParseWaterScenarioState(definitionJson.at("state"));
    }
    return definition;
}

json SerializeWaterScenarioKey(const WaterScenarioKey& key) {
    return json{
        {"id", key.id},
        {"position", std::clamp(key.position, 0.0F, 1.0F)},
        {"state", SerializeWaterScenarioState(key.state)},
        {"interpolation", WaterScenarioInterpolationName(key.interpolation)},
    };
}

WaterScenarioKey ParseWaterScenarioKey(const json& keyJson) {
    WaterScenarioKey key;
    key.id = keyJson.value("id", key.id);
    key.position = std::clamp(keyJson.value("position", key.position), 0.0F, 1.0F);
    if (keyJson.contains("state")) {
        key.state = ParseWaterScenarioState(keyJson.at("state"));
    }
    if (keyJson.contains("interpolation")) {
        key.interpolation = ParseWaterScenarioInterpolation(keyJson.at("interpolation"));
    }
    return key;
}

json SerializeWaterSeepageNodeAnimationState(
    const WaterSeepageNodeAnimationState& state) {
    return json{
        {"activity", std::clamp(state.activity, 0.0F, 1.0F)},
        {"local_spread", std::clamp(state.localSpread, 0.0F, 1.0F)},
        {"wetting_progress", std::clamp(state.wettingProgress, 0.0F, 1.0F)},
        {"reach_scale", std::max(0.0F, state.reachScale)},
        {"width_scale", std::max(0.0F, state.widthScale)},
        {"prominence", std::max(0.0F, state.prominence)},
    };
}

WaterSeepageNodeAnimationState ParseWaterSeepageNodeAnimationState(
    const json& stateJson) {
    WaterSeepageNodeAnimationState state;
    state.activity = std::clamp(
        stateJson.value("activity", state.activity),
        0.0F,
        1.0F);
    state.localSpread = std::clamp(
        stateJson.value("local_spread", state.localSpread),
        0.0F,
        1.0F);
    state.wettingProgress = std::clamp(
        stateJson.value("wetting_progress", state.wettingProgress),
        0.0F,
        1.0F);
    state.reachScale = std::max(
        0.0F,
        stateJson.value("reach_scale", state.reachScale));
    state.widthScale = std::max(
        0.0F,
        stateJson.value("width_scale", state.widthScale));
    state.prominence = std::max(
        0.0F,
        stateJson.value("prominence", state.prominence));
    return state;
}

json SerializeWaterSeepageNodeKey(const WaterSeepageNodeKey& key) {
    return json{
        {"id", key.id},
        {"position", std::clamp(key.position, 0.0F, 1.0F)},
        {"state", SerializeWaterSeepageNodeAnimationState(key.state)},
        {"interpolation", WaterScenarioInterpolationName(key.interpolation)},
    };
}

WaterSeepageNodeKey ParseWaterSeepageNodeKey(const json& keyJson) {
    WaterSeepageNodeKey key;
    key.id = keyJson.value("id", key.id);
    key.position = std::clamp(keyJson.value("position", key.position), 0.0F, 1.0F);
    if (keyJson.contains("state") && keyJson.at("state").is_object()) {
        key.state = ParseWaterSeepageNodeAnimationState(keyJson.at("state"));
    }
    if (keyJson.contains("interpolation")) {
        key.interpolation = ParseWaterScenarioInterpolation(keyJson.at("interpolation"));
    }
    return key;
}

json SerializeWaterSeepageNodeTrack(const WaterSeepageNodeTrack& track) {
    json trackJson{
        {"node_id", track.nodeId},
        {"keys", json::array()},
    };
    for (const auto& key : track.keys) {
        trackJson["keys"].push_back(SerializeWaterSeepageNodeKey(key));
    }
    return trackJson;
}

WaterSeepageNodeTrack ParseWaterSeepageNodeTrack(const json& trackJson) {
    WaterSeepageNodeTrack track;
    track.nodeId = trackJson.value("node_id", track.nodeId);
    if (trackJson.contains("keys") && trackJson.at("keys").is_array()) {
        for (const auto& keyJson : trackJson.at("keys")) {
            track.keys.push_back(ParseWaterSeepageNodeKey(keyJson));
        }
        std::stable_sort(
            track.keys.begin(),
            track.keys.end(),
            [](const WaterSeepageNodeKey& left, const WaterSeepageNodeKey& right) {
                return left.position < right.position;
            });
    }
    return track;
}

const char* WaterTimingFeatureName(invisible_places::water::WaterTimingFeature feature) {
    switch (feature) {
        case invisible_places::water::WaterTimingFeature::Shoreline:
            return "shoreline";
        case invisible_places::water::WaterTimingFeature::Seepage:
            return "seepage";
        case invisible_places::water::WaterTimingFeature::Rain:
            return "rain";
        case invisible_places::water::WaterTimingFeature::Flow:
            return "flow";
        case invisible_places::water::WaterTimingFeature::MeshFlow:
            return "mesh_flow";
    }
    return "rain";
}

invisible_places::water::WaterTimingFeature ParseWaterTimingFeature(
    const json& featureJson) {
    if (!featureJson.is_string()) {
        return invisible_places::water::WaterTimingFeature::Rain;
    }
    const auto name = featureJson.get<std::string>();
    if (name == "shoreline") {
        return invisible_places::water::WaterTimingFeature::Shoreline;
    }
    if (name == "seepage") {
        return invisible_places::water::WaterTimingFeature::Seepage;
    }
    if (name == "flow") {
        return invisible_places::water::WaterTimingFeature::Flow;
    }
    if (name == "mesh_flow") {
        return invisible_places::water::WaterTimingFeature::MeshFlow;
    }
    return invisible_places::water::WaterTimingFeature::Rain;
}

json SerializeWaterTimingKey(const invisible_places::water::WaterTimingKey& key) {
    return json{
        {"id", key.id},
        {"position", std::clamp(key.position, 0.0F, 1.0F)},
        {"level", std::clamp(key.level, 0.0F, 1.0F)},
        {"interpolation", WaterScenarioInterpolationName(key.interpolation)},
    };
}

invisible_places::water::WaterTimingKey ParseWaterTimingKey(const json& keyJson) {
    invisible_places::water::WaterTimingKey key;
    key.id = keyJson.value("id", key.id);
    key.position = std::clamp(keyJson.value("position", key.position), 0.0F, 1.0F);
    key.level = std::clamp(keyJson.value("level", key.level), 0.0F, 1.0F);
    if (keyJson.contains("interpolation")) {
        key.interpolation = ParseWaterScenarioInterpolation(keyJson.at("interpolation"));
    }
    return key;
}

json SerializeWaterTimingRun(const invisible_places::water::WaterTimingRun& run) {
    json runJson{
        {"id", run.id},
        {"name", run.name},
        {"feature", WaterTimingFeatureName(run.feature)},
        {"keys", json::array()},
    };
    for (const auto& key : run.keys) {
        runJson["keys"].push_back(SerializeWaterTimingKey(key));
    }
    return runJson;
}

invisible_places::water::WaterTimingRun ParseWaterTimingRun(const json& runJson) {
    invisible_places::water::WaterTimingRun run;
    run.id = runJson.value("id", run.id);
    run.name = runJson.value("name", run.name);
    if (runJson.contains("feature")) {
        run.feature = ParseWaterTimingFeature(runJson.at("feature"));
    }
    if (runJson.contains("keys") && runJson.at("keys").is_array()) {
        for (const auto& keyJson : runJson.at("keys")) {
            run.keys.push_back(ParseWaterTimingKey(keyJson));
        }
    }
    return invisible_places::water::SanitizeWaterTimingRun(std::move(run));
}

json SerializeWaterTimingRunAssignment(
    const invisible_places::water::WaterTimingRunAssignment& assignment) {
    return json{
        {"feature", WaterTimingFeatureName(assignment.feature)},
        {"run_id", assignment.runId},
        {"run_name", assignment.runName},
        {"fallback_run", SerializeWaterTimingRun(assignment.fallbackRun)},
    };
}

invisible_places::water::WaterTimingRunAssignment ParseWaterTimingRunAssignment(
    const json& assignmentJson) {
    invisible_places::water::WaterTimingRunAssignment assignment;
    if (assignmentJson.contains("feature")) {
        assignment.feature = ParseWaterTimingFeature(assignmentJson.at("feature"));
    }
    assignment.runId = assignmentJson.value("run_id", assignment.runId);
    assignment.runName = assignmentJson.value("run_name", assignment.runName);
    if (assignmentJson.contains("fallback_run")) {
        assignment.fallbackRun = ParseWaterTimingRun(assignmentJson.at("fallback_run"));
    }
    return assignment;
}

json SerializeWaterScenarioTrack(const WaterScenarioTrack& track) {
    json trackJson{
        {"scenario_id", track.scenarioId},
        {"scenario_name", track.scenarioName},
        {"fallback_scenario", SerializeWaterScenarioDefinition(track.fallbackScenario)},
        {"keys", json::array()},
        {"seepage_node_tracks", json::array()},
        {"timing_assignments", json::array()},
    };
    for (const auto& key : track.keys) {
        trackJson["keys"].push_back(SerializeWaterScenarioKey(key));
    }
    for (const auto& nodeTrack : track.seepageNodeTracks) {
        trackJson["seepage_node_tracks"].push_back(
            SerializeWaterSeepageNodeTrack(nodeTrack));
    }
    for (const auto& assignment : track.timingAssignments) {
        trackJson["timing_assignments"].push_back(
            SerializeWaterTimingRunAssignment(assignment));
    }
    return trackJson;
}

WaterScenarioTrack ParseWaterScenarioTrack(const json& trackJson) {
    WaterScenarioTrack track;
    track.scenarioId = trackJson.value("scenario_id", track.scenarioId);
    track.scenarioName = MigrateWaterScenarioDisplayName(
        track.scenarioId,
        trackJson.value("scenario_name", track.scenarioName));
    if (trackJson.contains("fallback_scenario")) {
        track.fallbackScenario = ParseWaterScenarioDefinition(trackJson.at("fallback_scenario"));
    }
    if (trackJson.contains("keys") && trackJson.at("keys").is_array()) {
        for (const auto& keyJson : trackJson.at("keys")) {
            track.keys.push_back(ParseWaterScenarioKey(keyJson));
        }
        std::stable_sort(
            track.keys.begin(),
            track.keys.end(),
            [](const WaterScenarioKey& left, const WaterScenarioKey& right) {
                return left.position < right.position;
            });
    }
    if (trackJson.contains("seepage_node_tracks") &&
        trackJson.at("seepage_node_tracks").is_array()) {
        for (const auto& nodeTrackJson : trackJson.at("seepage_node_tracks")) {
            track.seepageNodeTracks.push_back(
                ParseWaterSeepageNodeTrack(nodeTrackJson));
        }
    }
    if (trackJson.contains("timing_assignments") &&
        trackJson.at("timing_assignments").is_array()) {
        for (const auto& assignmentJson : trackJson.at("timing_assignments")) {
            track.timingAssignments.push_back(
                ParseWaterTimingRunAssignment(assignmentJson));
        }
    }
    return track;
}

json SerializeWaterSeepageNode(const WaterSeepageNode& node) {
    json nodeJson{
        {"id", node.id},
        {"name", node.name},
        {"position", {node.position.x, node.position.y, node.position.z}},
        {"surface_normal", {node.surfaceNormal.x, node.surfaceNormal.y, node.surfaceNormal.z}},
        {"down_axis", {node.downAxis.x, node.downAxis.y, node.downAxis.z}},
        {"reach_meters", node.reachMeters},
        {"width_meters", node.widthMeters},
        {"prominence", node.prominence},
        {"selection_reach_limit_meters", node.selectionReachLimitMeters},
        {"selection_width_limit_meters", node.selectionWidthLimitMeters},
        {"edge_feather_meters", node.edgeFeatherMeters},
        {"depth_tolerance_meters", node.depthToleranceMeters},
        {"normal_alignment", node.normalAlignment},
        {"strength", node.strength},
        {"seed", node.seed},
        {"enabled_in_viewport", node.enabledInViewport},
        {"enabled_in_export", node.enabledInExport},
        {"target_scene_roles", node.targetSceneRoles},
        {"look_profile_name", node.lookProfileName},
    };
    if (node.lookOverride.has_value()) {
        nodeJson["look_override"] = SerializeWaterSeepageLookSettings(node.lookOverride.value());
    }
    if (node.tempLookOverride.has_value()) {
        nodeJson["temp_look_override"] =
            SerializeWaterSeepageLookSettings(node.tempLookOverride.value());
    }
    return nodeJson;
}

WaterSeepageNode ParseWaterSeepageNode(const json& nodeJson) {
    WaterSeepageNode node;
    node.id = nodeJson.value("id", node.id);
    node.name = nodeJson.value("name", node.name);
    if (nodeJson.contains("position")) {
        const auto position = nodeJson.at("position").get<std::array<float, 3>>();
        node.position = {position[0], position[1], position[2]};
    }
    if (nodeJson.contains("surface_normal")) {
        const auto normal = nodeJson.at("surface_normal").get<std::array<float, 3>>();
        node.surfaceNormal = {normal[0], normal[1], normal[2]};
    }
    if (nodeJson.contains("down_axis")) {
        const auto downAxis = nodeJson.at("down_axis").get<std::array<float, 3>>();
        node.downAxis = {downAxis[0], downAxis[1], downAxis[2]};
    }
    node.reachMeters = nodeJson.value("reach_meters", node.reachMeters);
    const bool hasLiveWidth = nodeJson.contains("width_meters");
    if (hasLiveWidth) {
        node.widthMeters = nodeJson.value("width_meters", node.widthMeters);
    } else {
        const float legacyStartWidth =
            nodeJson.value("start_width_meters", node.startWidthMeters);
        const float legacyEndWidth =
            nodeJson.value("end_width_meters", node.endWidthMeters);
        node.widthMeters = std::max(legacyStartWidth, legacyEndWidth);
    }
    node.widthMeters = std::max(0.0F, node.widthMeters);
    // These aliases keep old code paths readable during migration; schema-19
    // writes only the live width and explicit topology limits above.
    node.startWidthMeters = node.widthMeters;
    node.endWidthMeters = node.widthMeters;
    node.prominence = std::max(
        0.0F,
        nodeJson.value("prominence", node.prominence));
    node.selectionReachLimitMeters = std::max(
        node.reachMeters,
        nodeJson.value(
            "selection_reach_limit_meters",
            node.reachMeters * 1.875F));
    node.selectionWidthLimitMeters = std::max(
        node.widthMeters,
        nodeJson.value(
            "selection_width_limit_meters",
            node.widthMeters * 1.62F));
    node.edgeFeatherMeters = nodeJson.value("edge_feather_meters", node.edgeFeatherMeters);
    node.depthToleranceMeters =
        nodeJson.value("depth_tolerance_meters", node.depthToleranceMeters);
    node.normalAlignment = nodeJson.value("normal_alignment", node.normalAlignment);
    node.strength = nodeJson.value("strength", node.strength);
    node.seed = nodeJson.value("seed", node.seed);
    node.enabledInViewport = nodeJson.value("enabled_in_viewport", node.enabledInViewport);
    node.enabledInExport = nodeJson.value("enabled_in_export", node.enabledInExport);
    if (nodeJson.contains("target_scene_roles") && nodeJson.at("target_scene_roles").is_array()) {
        node.targetSceneRoles = nodeJson.at("target_scene_roles").get<std::vector<std::string>>();
    }
    node.lookProfileName = nodeJson.value("look_profile_name", node.lookProfileName);
    if (nodeJson.contains("look_override")) {
        node.lookOverride = ParseWaterSeepageLookSettings(nodeJson.at("look_override"));
    }
    if (nodeJson.contains("temp_look_override")) {
        node.tempLookOverride = ParseWaterSeepageLookSettings(nodeJson.at("temp_look_override"));
    }
    return node;
}

json SerializeWaterRipplePatternSettings(const WaterRipplePatternSettings& settings) {
    return json{
        {"pattern_scale", settings.patternScale},
        {"wavelength_meters", settings.wavelengthMeters},
        {"speed", settings.speed},
        {"warp", settings.warp},
        {"turbulence", settings.turbulence},
        {"density", settings.density},
        {"phase", settings.phase},
        {"direction", {settings.directionX, settings.directionY, settings.directionZ}},
    };
}

WaterRipplePatternSettings ParseWaterRipplePatternSettings(
    const json& settingsJson,
    WaterRippleOverlayType type) {
    auto settings = invisible_places::water::DefaultWaterRipplePatternSettings(type);
    settings.patternScale = std::clamp(settingsJson.value("pattern_scale", settings.patternScale), 0.001F, 100.0F);
    settings.wavelengthMeters = std::clamp(
        settingsJson.value("wavelength_meters", settings.wavelengthMeters),
        0.001F,
        50.0F);
    settings.speed = std::clamp(settingsJson.value("speed", settings.speed), 0.0F, 20.0F);
    settings.warp = std::clamp(settingsJson.value("warp", settings.warp), 0.0F, 20.0F);
    settings.turbulence = std::clamp(settingsJson.value("turbulence", settings.turbulence), 0.0F, 20.0F);
    settings.density = std::clamp(settingsJson.value("density", settings.density), 0.0F, 1.0F);
    settings.phase = settingsJson.value("phase", settings.phase);
    if (settingsJson.contains("direction")) {
        const auto direction = settingsJson.at("direction").get<std::array<float, 3>>();
        settings.directionX = direction[0];
        settings.directionY = direction[1];
        settings.directionZ = direction[2];
    }
    return settings;
}

json SerializeWaterRipplePatternSettingsByOverlay(const WaterEffectLayer& layer) {
    json settingsJson = json::object();
    auto settings = layer.overlayPatternSettings;
    for (const auto type : invisible_places::water::AllWaterRippleOverlayTypes()) {
        const auto index = invisible_places::water::WaterRippleOverlayTypeIndex(type);
        if (settings[index].patternScale <= 0.0F || settings[index].wavelengthMeters <= 0.0F) {
            settings[index] = invisible_places::water::DefaultWaterRipplePatternSettings(type);
        }
    }
    settings[invisible_places::water::WaterRippleOverlayTypeIndex(layer.rippleOverlayType)] =
        invisible_places::water::ActiveWaterRipplePatternSettings(layer);
    for (const auto type : invisible_places::water::AllWaterRippleOverlayTypes()) {
        settingsJson[std::string{invisible_places::water::WaterRippleOverlayTypeNameForStorage(type)}] =
            SerializeWaterRipplePatternSettings(
                settings[invisible_places::water::WaterRippleOverlayTypeIndex(type)]);
    }
    return settingsJson;
}

json SerializeWaterEffectLayer(const WaterEffectLayer& layer) {
    return json{
        {"id", layer.id},
        {"name", layer.name},
        {"feature_type", WaterEffectFeatureTypeName(layer.featureType)},
        {"overlay_type", WaterRippleOverlayTypeName(layer.rippleOverlayType)},
        {"blend_mode", WaterEffectBlendModeName(layer.blendMode)},
        {"target_layer_source_path", layer.targetLayerSourcePath.generic_string()},
        {"target_scene_roles", layer.targetSceneRoles},
        {"vertices", SerializeWaterRegionVertices(layer.vertices)},
        {"hull", SerializeWaterRegionVertices(layer.hull)},
        {"enabled_in_viewport", layer.enabledInViewport},
        {"enabled_in_export", layer.enabledInExport},
        {"blend_priority", layer.blendPriority},
        {"edge_blend_width", layer.edgeBlendWidth},
        {"region_strength", layer.regionStrength},
        {"pattern_scale", layer.patternScale},
        {"speed", layer.speed},
        {"wavelength_meters", layer.wavelengthMeters},
        {"warp", layer.warp},
        {"turbulence", layer.turbulence},
        {"density", layer.density},
        {"phase", layer.phase},
        {"direction", {layer.directionX, layer.directionY, layer.directionZ}},
        {"overlay_pattern_settings", SerializeWaterRipplePatternSettingsByOverlay(layer)},
        {"seed", layer.seed},
        {"max_affected_points", layer.maxAffectedPoints},
        {"response", SerializeWaterEffectResponseSettings(layer.response)},
    };
}

WaterEffectLayer ParseWaterEffectLayer(const json& layerJson) {
    WaterEffectLayer layer;
    layer.id = layerJson.value("id", 0U);
    layer.name = layerJson.value("name", layer.name);
    if (layerJson.contains("feature_type")) {
        layer.featureType = ParseWaterEffectFeatureType(layerJson.at("feature_type"));
    }
    if (layerJson.contains("overlay_type")) {
        layer.rippleOverlayType = ParseWaterRippleOverlayType(layerJson.at("overlay_type"));
    }
    if (layerJson.contains("blend_mode")) {
        layer.blendMode = ParseWaterEffectBlendMode(layerJson.at("blend_mode"));
    }
    layer.targetLayerSourcePath = layerJson.value("target_layer_source_path", std::string{});
    if (layerJson.contains("target_scene_roles") && layerJson.at("target_scene_roles").is_array()) {
        layer.targetSceneRoles = layerJson.at("target_scene_roles").get<std::vector<std::string>>();
    }
    if (layerJson.contains("vertices")) {
        layer.vertices = ParseWaterRegionVertices(layerJson.at("vertices"));
    }
    if (layerJson.contains("hull")) {
        layer.hull = ParseWaterRegionVertices(layerJson.at("hull"));
    }
    layer.enabledInViewport = layerJson.value("enabled_in_viewport", layer.enabledInViewport);
    layer.enabledInExport = layerJson.value("enabled_in_export", layer.enabledInExport);
    layer.blendPriority = layerJson.value("blend_priority", layer.blendPriority);
    layer.edgeBlendWidth = std::clamp(layerJson.value("edge_blend_width", layer.edgeBlendWidth), 0.001F, 50.0F);
    layer.regionStrength = std::clamp(layerJson.value("region_strength", layer.regionStrength), 0.0F, 8.0F);
    layer.patternScale = std::clamp(layerJson.value("pattern_scale", layer.patternScale), 0.001F, 100.0F);
    layer.speed = std::clamp(layerJson.value("speed", layer.speed), 0.0F, 20.0F);
    layer.wavelengthMeters = std::clamp(layerJson.value("wavelength_meters", layer.wavelengthMeters), 0.001F, 50.0F);
    layer.warp = std::clamp(layerJson.value("warp", layer.warp), 0.0F, 20.0F);
    layer.turbulence = std::clamp(layerJson.value("turbulence", layer.turbulence), 0.0F, 20.0F);
    layer.density = std::clamp(layerJson.value("density", layer.density), 0.0F, 1.0F);
    layer.phase = layerJson.value("phase", layer.phase);
    if (layerJson.contains("direction")) {
        const auto direction = layerJson.at("direction").get<std::array<float, 3>>();
        layer.directionX = direction[0];
        layer.directionY = direction[1];
        layer.directionZ = direction[2];
    }
    const auto legacyActiveSettings = invisible_places::water::ActiveWaterRipplePatternSettings(layer);
    for (const auto type : invisible_places::water::AllWaterRippleOverlayTypes()) {
        layer.overlayPatternSettings[invisible_places::water::WaterRippleOverlayTypeIndex(type)] =
            invisible_places::water::DefaultWaterRipplePatternSettings(type);
    }
    layer.overlayPatternSettings[invisible_places::water::WaterRippleOverlayTypeIndex(layer.rippleOverlayType)] =
        legacyActiveSettings;
    if (layerJson.contains("overlay_pattern_settings") &&
        layerJson.at("overlay_pattern_settings").is_object()) {
        const auto& byOverlay = layerJson.at("overlay_pattern_settings");
        for (const auto type : invisible_places::water::AllWaterRippleOverlayTypes()) {
            const auto key = std::string{invisible_places::water::WaterRippleOverlayTypeNameForStorage(type)};
            if (byOverlay.contains(key)) {
                layer.overlayPatternSettings[invisible_places::water::WaterRippleOverlayTypeIndex(type)] =
                    ParseWaterRipplePatternSettings(byOverlay.at(key), type);
            }
        }
    }
    invisible_places::water::ApplyActiveWaterRipplePatternSettings(&layer);
    layer.seed = layerJson.value("seed", layer.seed);
    layer.maxAffectedPoints = layerJson.value("max_affected_points", layer.maxAffectedPoints);
    if (layerJson.contains("response")) {
        layer.response = ParseWaterEffectResponseSettings(layerJson.at("response"));
    }
    if (layer.hull.empty()) {
        layer.hull = invisible_places::water::BuildWaterRegionHull(layer.vertices);
    }
    return layer;
}

json SerializeVec3(const glm::vec3& value) {
    return json::array({value.x, value.y, value.z});
}

float ParseFiniteFloat(const json& valueJson, float fallback) {
    if (!valueJson.is_number()) {
        return fallback;
    }
    const float value = valueJson.get<float>();
    return std::isfinite(value) ? value : fallback;
}

glm::vec3 ParseVec3(const json& valueJson, const glm::vec3& fallback) {
    if (!valueJson.is_array() || valueJson.size() < 3U) {
        return fallback;
    }
    return glm::vec3{
        ParseFiniteFloat(valueJson.at(0), fallback.x),
        ParseFiniteFloat(valueJson.at(1), fallback.y),
        ParseFiniteFloat(valueJson.at(2), fallback.z),
    };
}

json SerializeWaterRippleRuntimeMembership(const WaterRippleRuntimeMembership& membership) {
    return json{
        {"point_index", membership.pointIndex},
        {"param_index", membership.paramIndex},
        {"edge_distance", membership.edgeDistance},
        {"seed", membership.seed},
        {"shore_distance", membership.shoreDistance},
    };
}

WaterRippleRuntimeMembership ParseWaterRippleRuntimeMembership(const json& membershipJson) {
    WaterRippleRuntimeMembership membership;
    membership.pointIndex =
        membershipJson.value("point_index", std::numeric_limits<std::uint32_t>::max());
    membership.paramIndex = membershipJson.value("param_index", membership.paramIndex);
    membership.edgeDistance = std::max(0.0F, membershipJson.value("edge_distance", membership.edgeDistance));
    membership.seed = membershipJson.value("seed", membership.seed);
    membership.shoreDistance = std::max(0.0F, membershipJson.value("shore_distance", membership.shoreDistance));
    return membership;
}

json SerializeWaterRippleRuntimeParams(const WaterRippleRuntimeParams& params) {
    return json{
        {"overlay_type", WaterRippleOverlayTypeName(params.overlayType)},
        {"blend_mode", WaterEffectBlendModeName(params.blendMode)},
        {"layer_id", params.layerId},
        {"seed", params.seed},
        {"region_center", SerializeVec3(params.regionCenter)},
        {"direction", SerializeVec3(params.direction)},
        {"region_strength", params.regionStrength},
        {"edge_blend_width", params.edgeBlendWidth},
        {"pattern_scale", params.patternScale},
        {"wavelength_meters", params.wavelengthMeters},
        {"speed", params.speed},
        {"warp", params.warp},
        {"turbulence", params.turbulence},
        {"density", params.density},
        {"phase", params.phase},
        {"response", SerializeWaterEffectResponseSettings(params.response)},
    };
}

WaterRippleRuntimeParams ParseWaterRippleRuntimeParams(const json& paramsJson) {
    WaterRippleRuntimeParams params;
    if (paramsJson.contains("overlay_type")) {
        params.overlayType = ParseWaterRippleOverlayType(paramsJson.at("overlay_type"));
    }
    if (paramsJson.contains("blend_mode")) {
        params.blendMode = ParseWaterEffectBlendMode(paramsJson.at("blend_mode"));
    }
    params.layerId = paramsJson.value("layer_id", params.layerId);
    params.seed = paramsJson.value("seed", params.seed);
    if (paramsJson.contains("region_center")) {
        params.regionCenter = ParseVec3(paramsJson.at("region_center"), params.regionCenter);
    }
    if (paramsJson.contains("direction")) {
        params.direction = ParseVec3(paramsJson.at("direction"), params.direction);
    }
    params.regionStrength = std::clamp(paramsJson.value("region_strength", params.regionStrength), 0.0F, 8.0F);
    params.edgeBlendWidth = std::clamp(paramsJson.value("edge_blend_width", params.edgeBlendWidth), 0.001F, 50.0F);
    params.patternScale = std::clamp(paramsJson.value("pattern_scale", params.patternScale), 0.001F, 100.0F);
    params.wavelengthMeters =
        std::clamp(paramsJson.value("wavelength_meters", params.wavelengthMeters), 0.001F, 50.0F);
    params.speed = std::clamp(paramsJson.value("speed", params.speed), 0.0F, 20.0F);
    params.warp = std::clamp(paramsJson.value("warp", params.warp), 0.0F, 20.0F);
    params.turbulence = std::clamp(paramsJson.value("turbulence", params.turbulence), 0.0F, 20.0F);
    params.density = std::clamp(paramsJson.value("density", params.density), 0.0F, 1.0F);
    params.phase = paramsJson.value("phase", params.phase);
    if (paramsJson.contains("response")) {
        params.response = ParseWaterEffectResponseSettings(paramsJson.at("response"));
    }
    return params;
}

json SerializeWaterRippleRuntimeCache(const WaterRippleRuntimeCacheDocument& cache) {
    json cacheJson{
        {"schema_version", cache.schemaVersion},
        {"support_layer_path", cache.supportLayerPath.generic_string()},
        {"support_signature", cache.supportSignature},
        {"region_fingerprint", cache.regionFingerprint},
        {"stale", cache.stale},
        {"memberships", json::array()},
        {"params", json::array()},
    };
    for (const auto& membership : cache.memberships) {
        cacheJson["memberships"].push_back(SerializeWaterRippleRuntimeMembership(membership));
    }
    for (const auto& params : cache.params) {
        cacheJson["params"].push_back(SerializeWaterRippleRuntimeParams(params));
    }
    return cacheJson;
}

WaterRippleRuntimeCacheDocument ParseWaterRippleRuntimeCache(const json& cacheJson) {
    WaterRippleRuntimeCacheDocument cache;
    cache.schemaVersion = cacheJson.value("schema_version", cache.schemaVersion);
    cache.supportLayerPath = cacheJson.value("support_layer_path", std::string{});
    cache.supportSignature = cacheJson.value("support_signature", cache.supportSignature);
    cache.regionFingerprint = cacheJson.value("region_fingerprint", cache.regionFingerprint);
    cache.stale = cacheJson.value("stale", cache.stale);
    if (cacheJson.contains("memberships") && cacheJson.at("memberships").is_array()) {
        const auto& membershipsJson = cacheJson.at("memberships");
        if (membershipsJson.size() <= kMaxSerializedWaterRippleRuntimeCacheMemberships) {
            for (const auto& membershipJson : membershipsJson) {
                auto membership = ParseWaterRippleRuntimeMembership(membershipJson);
                if (membership.pointIndex != std::numeric_limits<std::uint32_t>::max()) {
                    cache.memberships.push_back(membership);
                }
            }
        } else {
            cache.stale = true;
        }
    }
    if (cacheJson.contains("params") && cacheJson.at("params").is_array()) {
        for (const auto& paramsJson : cacheJson.at("params")) {
            cache.params.push_back(ParseWaterRippleRuntimeParams(paramsJson));
        }
    }
    return cache;
}

json SerializeWaterFlowTrailSettings(const WaterFlowTrailSettings& settings) {
    return json{
        {"enabled", settings.enabled},
        {"trail_count_total", settings.trailCountTotal},
        {"lane_count", settings.laneCount},
        {"trail_length_meters", settings.trailLengthMeters},
        {"trail_point_spacing_meters", settings.trailPointSpacingMeters},
        {"trail_width_meters", settings.trailWidthMeters},
        {"trail_streak_length_meters", settings.trailStreakLengthMeters},
        {"surface_offset_meters", settings.surfaceOffsetMeters},
        {"path_attraction", settings.pathAttraction},
        {"lane_spread_meters", settings.laneSpreadMeters},
        {"lane_crossing", settings.laneCrossing},
        {"trail_smoothness", settings.trailSmoothness},
        {"trail_looseness", settings.trailLooseness},
        {"turbulence", settings.turbulence},
        {"surface_follow", settings.surfaceFollow},
        {"downhill_pull", settings.downhillPull},
        {"terrain_width_response", settings.terrainWidthResponse},
        {"turbulence_scale_meters", settings.turbulenceScaleMeters},
        {"speed_meters_per_second", settings.speedMetersPerSecond},
        {"seed", settings.seed},
    };
}

WaterFlowTrailSettings ParseWaterFlowTrailSettings(const json& settingsJson) {
    WaterFlowTrailSettings settings;
    settings.enabled = settingsJson.value("enabled", settings.enabled);
    settings.trailCountTotal = settingsJson.value(
        "trail_count_total",
        settingsJson.value("stream_count_total", settings.trailCountTotal));
    settings.laneCount = settingsJson.value("lane_count", settings.laneCount);
    settings.trailLengthMeters = settingsJson.value(
        "trail_length_meters",
        settingsJson.value("stream_length_meters", settings.trailLengthMeters));
    settings.trailPointSpacingMeters = settingsJson.value(
        "trail_point_spacing_meters",
        settingsJson.value("stream_point_spacing_meters", settings.trailPointSpacingMeters));
    settings.trailWidthMeters = settingsJson.value(
        "trail_width_meters",
        settingsJson.value("stream_width_meters", settings.trailWidthMeters));
    settings.trailStreakLengthMeters = settingsJson.value(
        "trail_streak_length_meters",
        settingsJson.value("stream_world_length_meters", settings.trailStreakLengthMeters));
    settings.surfaceOffsetMeters = settingsJson.value("surface_offset_meters", settings.surfaceOffsetMeters);
    settings.pathAttraction = settingsJson.value("path_attraction", settings.pathAttraction);
    settings.laneSpreadMeters = settingsJson.value("lane_spread_meters", settings.laneSpreadMeters);
    settings.laneCrossing = settingsJson.value("lane_crossing", settings.laneCrossing);
    settings.trailSmoothness = settingsJson.value(
        "trail_smoothness",
        settingsJson.value("stream_smoothness", settings.trailSmoothness));
    settings.trailLooseness = settingsJson.value(
        "trail_looseness",
        settingsJson.value("stream_looseness", settings.trailLooseness));
    settings.turbulence = settingsJson.value("turbulence", settings.turbulence);
    if (settingsJson.contains("surface_follow")) {
        settings.surfaceFollow = std::clamp(
            ParseFiniteFloat(settingsJson.at("surface_follow"), settings.surfaceFollow),
            0.0F,
            1.0F);
    }
    if (settingsJson.contains("downhill_pull")) {
        settings.downhillPull = std::clamp(
            ParseFiniteFloat(settingsJson.at("downhill_pull"), settings.downhillPull),
            0.0F,
            1.0F);
    }
    if (settingsJson.contains("terrain_width_response")) {
        settings.terrainWidthResponse = std::clamp(
            ParseFiniteFloat(
                settingsJson.at("terrain_width_response"),
                settings.terrainWidthResponse),
            0.0F,
            1.0F);
    }
    if (settingsJson.contains("turbulence_scale_meters")) {
        settings.turbulenceScaleMeters = std::clamp(
            ParseFiniteFloat(
                settingsJson.at("turbulence_scale_meters"),
                settings.turbulenceScaleMeters),
            0.005F,
            100.0F);
    }
    settings.speedMetersPerSecond = settingsJson.value("speed_meters_per_second", settings.speedMetersPerSecond);
    settings.seed = settingsJson.value("seed", settings.seed);
    return settings;
}

json SerializeWaterFieldSettings(const WaterFieldSettings& settings) {
    return json{
        {"enabled", settings.enabled},
        {"output_mode", WaterFieldOutputModeName(settings.outputMode)},
        {"corridor_radius_meters", settings.corridorRadiusMeters},
        {"field_resolution_meters", settings.fieldResolutionMeters},
        {"projection_resolution_meters", settings.projectionResolutionMeters},
        {"guide_weight", settings.guideWeight},
        {"downhill_weight", settings.downhillWeight},
        {"graph_weight", settings.graphWeight},
        {"lateral_weight", settings.lateralWeight},
        {"field_smoothing", settings.fieldSmoothing},
        {"wetness_spread", settings.wetnessSpread},
        {"surface_offset_meters", settings.surfaceOffsetMeters},
        {"surface_confidence_threshold", settings.surfaceConfidenceThreshold},
        {"max_bridge_distance_meters", settings.maxBridgeDistanceMeters},
        {"bridge_aggression", settings.bridgeAggression},
        {"turbulence", settings.turbulence},
        {"seed", settings.seed},
    };
}

WaterFieldSettings ParseWaterFieldSettings(const json& settingsJson) {
    WaterFieldSettings settings;
    settings.enabled = settingsJson.value("enabled", settings.enabled);
    if (settingsJson.contains("output_mode")) {
        settings.outputMode = ParseWaterFieldOutputMode(settingsJson.at("output_mode"));
    }
    settings.corridorRadiusMeters = settingsJson.value("corridor_radius_meters", settings.corridorRadiusMeters);
    settings.fieldResolutionMeters = settingsJson.value("field_resolution_meters", settings.fieldResolutionMeters);
    settings.projectionResolutionMeters = settingsJson.value("projection_resolution_meters", settings.projectionResolutionMeters);
    settings.guideWeight = settingsJson.value("guide_weight", settings.guideWeight);
    settings.downhillWeight = settingsJson.value("downhill_weight", settings.downhillWeight);
    settings.graphWeight = settingsJson.value("graph_weight", settings.graphWeight);
    settings.lateralWeight = settingsJson.value("lateral_weight", settings.lateralWeight);
    settings.fieldSmoothing = settingsJson.value("field_smoothing", settings.fieldSmoothing);
    settings.wetnessSpread = settingsJson.value("wetness_spread", settings.wetnessSpread);
    settings.surfaceOffsetMeters = settingsJson.value("surface_offset_meters", settings.surfaceOffsetMeters);
    settings.surfaceConfidenceThreshold = settingsJson.value("surface_confidence_threshold", settings.surfaceConfidenceThreshold);
    settings.maxBridgeDistanceMeters = settingsJson.value("max_bridge_distance_meters", settings.maxBridgeDistanceMeters);
    settings.bridgeAggression = settingsJson.value("bridge_aggression", settings.bridgeAggression);
    settings.turbulence = settingsJson.value("turbulence", settings.turbulence);
    settings.seed = settingsJson.value("seed", settings.seed);
    return settings;
}

json SerializeWaterFieldTrailSettings(const WaterFieldTrailSettings& settings) {
    return json{
        {"enabled", settings.enabled},
        {"trail_count", settings.trailCount},
        {"seed_spacing_meters", settings.seedSpacingMeters},
        {"trail_length_meters", settings.trailLengthMeters},
        {"trail_point_spacing_meters", settings.trailPointSpacingMeters},
        {"trail_width_meters", settings.trailWidthMeters},
        {"trail_streak_length_meters", settings.trailStreakLengthMeters},
        {"momentum", settings.momentum},
        {"max_turn_angle_degrees", settings.maxTurnAngleDegrees},
        {"speed_meters_per_second", settings.speedMetersPerSecond},
        {"fade_on_low_confidence", settings.fadeOnLowConfidence},
    };
}

WaterFieldTrailSettings ParseWaterFieldTrailSettings(const json& settingsJson) {
    WaterFieldTrailSettings settings;
    settings.enabled = settingsJson.value("enabled", settings.enabled);
    settings.trailCount = settingsJson.value(
        "trail_count",
        settingsJson.value("streamline_count", settings.trailCount));
    settings.seedSpacingMeters = settingsJson.value("seed_spacing_meters", settings.seedSpacingMeters);
    settings.trailLengthMeters = settingsJson.value(
        "trail_length_meters",
        settingsJson.value("streamline_length_meters", settings.trailLengthMeters));
    settings.trailPointSpacingMeters = settingsJson.value(
        "trail_point_spacing_meters",
        settingsJson.value("step_length_meters", settings.trailPointSpacingMeters));
    settings.trailWidthMeters = settingsJson.value(
        "trail_width_meters",
        settingsJson.value("streamline_width_meters", settings.trailWidthMeters));
    settings.trailStreakLengthMeters = settingsJson.value(
        "trail_streak_length_meters",
        settingsJson.value("stream_world_length_meters", settings.trailStreakLengthMeters));
    settings.momentum = settingsJson.value("momentum", settings.momentum);
    settings.maxTurnAngleDegrees = settingsJson.value("max_turn_angle_degrees", settings.maxTurnAngleDegrees);
    settings.speedMetersPerSecond = settingsJson.value("speed_meters_per_second", settings.speedMetersPerSecond);
    settings.fadeOnLowConfidence = settingsJson.value("fade_on_low_confidence", settings.fadeOnLowConfidence);
    return settings;
}

json SerializeWaterDynamicMeshMotionKeyframe(const invisible_places::water::WaterDynamicMeshMotionKeyframe& keyframe) {
    return json{
        {"time_seconds", keyframe.timeSeconds},
        {"position", {keyframe.position.x, keyframe.position.y, keyframe.position.z}},
    };
}

invisible_places::water::WaterDynamicMeshMotionKeyframe ParseWaterDynamicMeshMotionKeyframe(
    const json& keyframeJson) {
    invisible_places::water::WaterDynamicMeshMotionKeyframe keyframe;
    keyframe.timeSeconds = std::clamp(keyframeJson.value("time_seconds", keyframe.timeSeconds), 0.0F, 86400.0F);
    if (keyframeJson.contains("position")) {
        const auto position = keyframeJson.at("position").get<std::array<float, 3>>();
        keyframe.position = {position[0], position[1], position[2]};
    }
    return keyframe;
}

json SerializeWaterDynamicMeshAttractor(const WaterDynamicMeshAttractor& attractor) {
    json attractorJson{
        {"id", attractor.id},
        {"name", attractor.name},
        {"position", {attractor.position.x, attractor.position.y, attractor.position.z}},
        {"radius_meters", attractor.radiusMeters},
        {"strength", attractor.strength},
        {"enabled", attractor.enabled},
    };
    if (!attractor.keyframes.empty()) {
        attractorJson["keyframes"] = json::array();
        for (const auto& keyframe : attractor.keyframes) {
            attractorJson["keyframes"].push_back(SerializeWaterDynamicMeshMotionKeyframe(keyframe));
        }
    }
    return attractorJson;
}

WaterDynamicMeshAttractor ParseWaterDynamicMeshAttractor(const json& attractorJson) {
    WaterDynamicMeshAttractor attractor;
    attractor.id = attractorJson.value("id", attractor.id);
    attractor.name = attractorJson.value("name", attractor.name);
    if (attractorJson.contains("position")) {
        const auto position = attractorJson.at("position").get<std::array<float, 3>>();
        attractor.position = {position[0], position[1], position[2]};
    }
    attractor.radiusMeters = std::clamp(
        attractorJson.value("radius_meters", attractor.radiusMeters),
        0.001F,
        100.0F);
    attractor.strength = std::clamp(attractorJson.value("strength", attractor.strength), 0.0F, 10.0F);
    attractor.enabled = attractorJson.value("enabled", attractor.enabled);
    if (attractorJson.contains("keyframes") && attractorJson.at("keyframes").is_array()) {
        attractor.keyframes.clear();
        for (const auto& keyframeJson : attractorJson.at("keyframes")) {
            attractor.keyframes.push_back(ParseWaterDynamicMeshMotionKeyframe(keyframeJson));
        }
    }
    return attractor;
}

json SerializeWaterDynamicMeshEmitterMotion(
    const invisible_places::water::WaterDynamicMeshEmitterMotion& motion) {
    json motionJson{
        {"emitter_id", motion.emitterId},
        {"name", motion.name},
        {"enabled", motion.enabled},
        {"keyframes", json::array()},
    };
    for (const auto& keyframe : motion.keyframes) {
        motionJson["keyframes"].push_back(SerializeWaterDynamicMeshMotionKeyframe(keyframe));
    }
    return motionJson;
}

invisible_places::water::WaterDynamicMeshEmitterMotion ParseWaterDynamicMeshEmitterMotion(
    const json& motionJson) {
    invisible_places::water::WaterDynamicMeshEmitterMotion motion;
    motion.emitterId = motionJson.value("emitter_id", motion.emitterId);
    motion.name = motionJson.value("name", motion.name);
    motion.enabled = motionJson.value("enabled", motion.enabled);
    if (motionJson.contains("keyframes") && motionJson.at("keyframes").is_array()) {
        motion.keyframes.clear();
        for (const auto& keyframeJson : motionJson.at("keyframes")) {
            motion.keyframes.push_back(ParseWaterDynamicMeshMotionKeyframe(keyframeJson));
        }
    }
    return motion;
}

json SerializeWaterDynamicMeshFlowSettings(WaterDynamicMeshFlowSettings settings) {
    settings = invisible_places::water::SanitizeWaterDynamicMeshFlowSettings(
        std::move(settings));
    json settingsJson{
        {"enabled", settings.enabled},
        {"gpu_preview_enabled", settings.gpuPreviewEnabled},
        {"show_trails", settings.showTrails},
        {"cache_cell_size_meters", settings.cacheCellSizeMeters},
        {"projection_search_radius_meters", settings.projectionSearchRadiusMeters},
        {"ambiguity_height_meters", settings.ambiguityHeightMeters},
        {"particle_capacity", settings.particleCapacity},
        {"history_length", settings.historyLength},
        {"dry_concavity_focus", settings.dryConcavityFocus},
        {"rain_spawn_spread", settings.rainSpawnSpread},
        {"rain_distributed_source_fraction", settings.rainDistributedSourceFraction},
        {"preview_particle_limit", settings.previewParticleLimit},
        {"final_particle_limit", settings.finalParticleLimit},
        {"trail_length_meters", settings.trailLengthMeters},
        {"step_meters", settings.stepMeters},
        {"trail_width_meters", settings.trailWidthMeters},
        {"trail_streak_length_meters", settings.trailStreakLengthMeters},
        {"trail_opacity_dry", settings.trailOpacityDry},
        {"trail_opacity_wet", settings.trailOpacityWet},
        {"trail_emission_dry", settings.trailEmissionDry},
        {"trail_emission_wet", settings.trailEmissionWet},
        {"trail_exposure", settings.trailExposure},
        {"surface_offset_meters", settings.surfaceOffsetMeters},
        {"speed_meters_per_second", settings.speedMetersPerSecond},
        {"downhill_weight", settings.downhillWeight},
        {"attractor_weight", settings.attractorWeight},
        {"source_velocity_weight", settings.sourceVelocityWeight},
        {"curl_strength", settings.curlStrength},
        {"branching_strength", settings.branchingStrength},
        {"eddy_strength", settings.eddyStrength},
        {"topology_response", settings.topologyResponse},
        {"inertia", settings.inertia},
        {"particle_noise_strength", settings.particleNoiseStrength},
        {"particle_noise_scale_meters", settings.particleNoiseScaleMeters},
        {"particle_noise_speed", settings.particleNoiseSpeed},
        {"shared_wind_strength", settings.sharedWindStrength},
        {"shared_wind_scale_meters", settings.sharedWindScaleMeters},
        {"shared_wind_speed", settings.sharedWindSpeed},
        {"contact_fade_seconds", settings.contactFadeSeconds},
        {"rock_response",
         {
             {"radius_meters", settings.rockResponse.radiusMeters},
             {"opacity_add", settings.rockResponse.opacityAdd},
             {"emission_add", settings.rockResponse.emissionAdd},
             {"colourise",
              {
                  settings.rockResponse.colourise.x,
                  settings.rockResponse.colourise.y,
                  settings.rockResponse.colourise.z,
              }},
             {"colourise_amount", settings.rockResponse.colouriseAmount},
             {"persistence_seconds", settings.rockResponse.persistenceSeconds},
         }},
        {"vegetation_response",
         {
             {"radius_meters", settings.vegetationResponse.radiusMeters},
             {"opacity_add", settings.vegetationResponse.opacityAdd},
             {"emission_add", settings.vegetationResponse.emissionAdd},
             {"colourise",
              {
                  settings.vegetationResponse.colourise.x,
                  settings.vegetationResponse.colourise.y,
                  settings.vegetationResponse.colourise.z,
              }},
             {"colourise_amount", settings.vegetationResponse.colouriseAmount},
             {"persistence_seconds", settings.vegetationResponse.persistenceSeconds},
             {"twinkle", settings.vegetationResponse.twinkle},
             {"stream_depth_meters", settings.vegetationResponse.streamDepthMeters},
         }},
        {"animation_duration_seconds", settings.animationDurationSeconds},
        {"seed", settings.seed},
        {"particle_preset_name", settings.particlePresetName},
        {"trail_profile_name", settings.trailProfileName},
    };
    return settingsJson;
}

WaterDynamicMeshFlowSettings ParseWaterDynamicMeshFlowSettings(const json& settingsJson) {
    auto settings = invisible_places::water::DefaultWaterDynamicMeshFlowSettings();
    settings.enabled = settingsJson.value("enabled", settings.enabled);
    settings.gpuPreviewEnabled = settingsJson.value("gpu_preview_enabled", settings.gpuPreviewEnabled);
    settings.showTrails = settingsJson.value("show_trails", settings.showTrails);
    settings.automaticSources = settingsJson.value(
        "automatic_sources",
        settings.automaticSources);
    settings.meshPath = settingsJson.value("mesh_path", settings.meshPath.generic_string());
    settings.cacheCellSizeMeters = std::clamp(
        settingsJson.value("cache_cell_size_meters", settings.cacheCellSizeMeters),
        0.005F,
        5.0F);
    settings.projectionSearchRadiusMeters = std::clamp(
        settingsJson.value("projection_search_radius_meters", settings.projectionSearchRadiusMeters),
        0.005F,
        25.0F);
    settings.ambiguityHeightMeters = std::clamp(
        settingsJson.value("ambiguity_height_meters", settings.ambiguityHeightMeters),
        0.0F,
        25.0F);
    settings.particleCapacity = settingsJson.value(
        "particle_capacity",
        settings.particleCapacity);
    settings.historyLength = settingsJson.value(
        "history_length",
        settings.historyLength);
    settings.sourceBandWidthMeters = settingsJson.value(
        "source_band_width_meters",
        settings.sourceBandWidthMeters);
    settings.sourceBandFraction = settingsJson.value(
        "source_band_fraction",
        settings.sourceBandFraction);
    settings.dryConcavityFocus = settingsJson.value(
        "dry_concavity_focus",
        settings.dryConcavityFocus);
    settings.rainSpawnSpread = settingsJson.value(
        "rain_spawn_spread",
        settings.rainSpawnSpread);
    settings.rainDistributedSourceFraction = settingsJson.value(
        "rain_distributed_source_fraction",
        settings.rainDistributedSourceFraction);
    settings.previewParticleLimit = std::clamp<std::uint32_t>(
        settingsJson.value("preview_particle_limit", settings.previewParticleLimit),
        1U,
        100000U);
    settings.finalParticleLimit = std::clamp<std::uint32_t>(
        settingsJson.value("final_particle_limit", settings.finalParticleLimit),
        1U,
        250000U);
    settings.trailLengthMeters = std::clamp(
        settingsJson.value("trail_length_meters", settings.trailLengthMeters),
        0.02F,
        100.0F);
    settings.stepMeters = std::clamp(settingsJson.value("step_meters", settings.stepMeters), 0.002F, 5.0F);
    settings.trailWidthMeters = std::clamp(
        settingsJson.value("trail_width_meters", settings.trailWidthMeters),
        0.0005F,
        1.0F);
    settings.trailStreakLengthMeters = std::clamp(
        settingsJson.value("trail_streak_length_meters", settings.trailStreakLengthMeters),
        0.001F,
        5.0F);
    settings.trailOpacityDry = settingsJson.value(
        "trail_opacity_dry",
        settings.trailOpacityDry);
    settings.trailOpacityWet = settingsJson.value(
        "trail_opacity_wet",
        settings.trailOpacityWet);
    settings.trailEmissionDry = settingsJson.value(
        "trail_emission_dry",
        settings.trailEmissionDry);
    settings.trailEmissionWet = settingsJson.value(
        "trail_emission_wet",
        settings.trailEmissionWet);
    settings.trailExposure = settingsJson.value(
        "trail_exposure",
        settings.trailExposure);
    settings.surfaceOffsetMeters = std::clamp(
        settingsJson.value("surface_offset_meters", settings.surfaceOffsetMeters),
        -1.0F,
        1.0F);
    settings.speedMetersPerSecond = std::clamp(
        settingsJson.value("speed_meters_per_second", settings.speedMetersPerSecond),
        0.001F,
        100.0F);
    settings.downhillWeight = std::clamp(settingsJson.value("downhill_weight", settings.downhillWeight), 0.0F, 10.0F);
    settings.attractorWeight = std::clamp(
        settingsJson.value("attractor_weight", settings.attractorWeight),
        0.0F,
        10.0F);
    settings.sourceVelocityWeight = std::clamp(
        settingsJson.value("source_velocity_weight", settings.sourceVelocityWeight),
        0.0F,
        10.0F);
    settings.curlStrength = std::clamp(settingsJson.value("curl_strength", settings.curlStrength), 0.0F, 10.0F);
    settings.branchingStrength = std::clamp(
        settingsJson.value("branching_strength", settings.branchingStrength),
        0.0F,
        10.0F);
    settings.eddyStrength = std::clamp(settingsJson.value("eddy_strength", settings.eddyStrength), 0.0F, 10.0F);
    settings.topologyResponse = std::clamp(
        settingsJson.value("topology_response", settings.topologyResponse),
        0.0F,
        10.0F);
    settings.inertia = std::clamp(settingsJson.value("inertia", settings.inertia), 0.0F, 0.98F);
    settings.particleNoiseStrength = settingsJson.value(
        "particle_noise_strength",
        settings.particleNoiseStrength);
    settings.particleNoiseScaleMeters = settingsJson.value(
        "particle_noise_scale_meters",
        settings.particleNoiseScaleMeters);
    settings.particleNoiseSpeed = settingsJson.value(
        "particle_noise_speed",
        settings.particleNoiseSpeed);
    settings.sharedWindStrength = settingsJson.value(
        "shared_wind_strength",
        settings.sharedWindStrength);
    settings.sharedWindScaleMeters = settingsJson.value(
        "shared_wind_scale_meters",
        settings.sharedWindScaleMeters);
    settings.sharedWindSpeed = settingsJson.value(
        "shared_wind_speed",
        settings.sharedWindSpeed);
    settings.contactFadeSeconds = settingsJson.value(
        "contact_fade_seconds",
        settings.contactFadeSeconds);
    const auto parseColour = [](const json& value,
                                invisible_places::io::Float3 fallback) {
        if (!value.is_array() || value.size() != 3U) {
            return fallback;
        }
        try {
            return invisible_places::io::Float3{
                value.at(0).get<float>(),
                value.at(1).get<float>(),
                value.at(2).get<float>(),
            };
        } catch (const json::exception&) {
            return fallback;
        }
    };
    if (settingsJson.contains("rock_response") &&
        settingsJson.at("rock_response").is_object()) {
        const auto& response = settingsJson.at("rock_response");
        settings.rockResponse.radiusMeters = response.value(
            "radius_meters",
            settings.rockResponse.radiusMeters);
        settings.rockResponse.opacityAdd = response.value(
            "opacity_add",
            settings.rockResponse.opacityAdd);
        settings.rockResponse.emissionAdd = response.value(
            "emission_add",
            settings.rockResponse.emissionAdd);
        if (response.contains("colourise")) {
            settings.rockResponse.colourise = parseColour(
                response.at("colourise"),
                settings.rockResponse.colourise);
        }
        settings.rockResponse.colouriseAmount = response.value(
            "colourise_amount",
            settings.rockResponse.colouriseAmount);
        settings.rockResponse.persistenceSeconds = response.value(
            "persistence_seconds",
            settings.rockResponse.persistenceSeconds);
    }
    if (settingsJson.contains("vegetation_response") &&
        settingsJson.at("vegetation_response").is_object()) {
        const auto& response = settingsJson.at("vegetation_response");
        settings.vegetationResponse.radiusMeters = response.value(
            "radius_meters",
            settings.vegetationResponse.radiusMeters);
        settings.vegetationResponse.opacityAdd = response.value(
            "opacity_add",
            settings.vegetationResponse.opacityAdd);
        settings.vegetationResponse.emissionAdd = response.value(
            "emission_add",
            settings.vegetationResponse.emissionAdd);
        if (response.contains("colourise")) {
            settings.vegetationResponse.colourise = parseColour(
                response.at("colourise"),
                settings.vegetationResponse.colourise);
        }
        settings.vegetationResponse.colouriseAmount = response.value(
            "colourise_amount",
            settings.vegetationResponse.colouriseAmount);
        settings.vegetationResponse.persistenceSeconds = response.value(
            "persistence_seconds",
            settings.vegetationResponse.persistenceSeconds);
        settings.vegetationResponse.twinkle = response.value(
            "twinkle",
            settings.vegetationResponse.twinkle);
        settings.vegetationResponse.streamDepthMeters = response.value(
            "stream_depth_meters",
            settings.vegetationResponse.streamDepthMeters);
    }
    settings.animationDurationSeconds = std::clamp(
        settingsJson.value("animation_duration_seconds", settings.animationDurationSeconds),
        0.0F,
        86400.0F);
    settings.seed = settingsJson.value("seed", settings.seed);
    settings.particlePresetName = std::string{
        invisible_places::water::NormalizeWaterDynamicMeshParticlePresetName(
            settingsJson.value("particle_preset_name", settings.particlePresetName))};
    settings.trailProfileName = settingsJson.value("trail_profile_name", settings.trailProfileName);
    if (settingsJson.contains("attractors") && settingsJson.at("attractors").is_array()) {
        settings.attractors.clear();
        for (const auto& attractorJson : settingsJson.at("attractors")) {
            settings.attractors.push_back(ParseWaterDynamicMeshAttractor(attractorJson));
        }
    }
    if (settingsJson.contains("emitter_motions") && settingsJson.at("emitter_motions").is_array()) {
        settings.emitterMotions.clear();
        for (const auto& motionJson : settingsJson.at("emitter_motions")) {
            settings.emitterMotions.push_back(ParseWaterDynamicMeshEmitterMotion(motionJson));
        }
    }
    return invisible_places::water::SanitizeWaterDynamicMeshFlowSettings(
        std::move(settings));
}

WaterDynamicMeshFlowSettings MigrateLegacyAutomaticMeshFlowDefaults(
    WaterDynamicMeshFlowSettings settings) {
    // Pre-schema-45 Mesh Flow allowed ordinary Flow emitters and inherited
    // aggressive path/profile values. Once sources become scene-wide and
    // automatic those values create the bright, SampleScene-sized burst that
    // this migration is intended to retire. Preserve authored enablement,
    // capacity, contact response, and scenario state while adopting the new
    // subtle fixed-capacity routing/presentation defaults.
    settings.automaticSources = true;
    settings.attractors.clear();
    settings.emitterMotions.clear();
    settings.sourceBandWidthMeters = 0.75F;
    settings.sourceBandFraction = 0.04F;
    settings.dryConcavityFocus = 0.90F;
    settings.rainSpawnSpread = 0.75F;
    settings.rainDistributedSourceFraction = 0.55F;
    settings.trailWidthMeters = 0.0025F;
    settings.trailStreakLengthMeters = 0.030F;
    settings.surfaceOffsetMeters = 0.003F;
    settings.trailOpacityDry = 0.025F;
    settings.trailOpacityWet = 0.14F;
    settings.trailEmissionDry = 0.04F;
    settings.trailEmissionWet = 0.45F;
    settings.trailExposure = 1.25F;
    settings.speedMetersPerSecond = 0.26F;
    settings.downhillWeight = 1.75F;
    settings.inertia = 0.88F;
    settings.particleNoiseStrength = 0.10F;
    settings.particleNoiseScaleMeters = 0.45F;
    settings.particleNoiseSpeed = 0.18F;
    settings.sharedWindStrength = 0.035F;
    settings.sharedWindScaleMeters = 3.0F;
    settings.sharedWindSpeed = 0.025F;
    return invisible_places::water::SanitizeWaterDynamicMeshFlowSettings(
        std::move(settings));
}

WaterDynamicMeshFlowSettings ProjectLevelWaterDynamicMeshFlowSettings(WaterDynamicMeshFlowSettings settings) {
    settings.meshPath.clear();
    settings.attractors.clear();
    settings.emitterMotions.clear();
    return settings;
}

json SerializeWaterSceneState(const WaterSceneStateDocument& state) {
    json stateJson{
        {"scene_group", state.sceneGroupName.empty() ? std::string{"Default"} : state.sceneGroupName},
        {"water_emitters", json::array()},
        {"water_manual_flow_paths", json::array()},
        {"water_seepage_nodes", json::array()},
        {"water_ripple_layers", json::array()},
        {"water_field_layers", json::array()},
        {"water_ripple_runtime_caches", json::array()},
        {"dynamic_mesh_path", state.dynamicMeshPath.generic_string()},
    };
    for (const auto& emitter : state.emitters) {
        stateJson["water_emitters"].push_back(SerializeWaterEmitter(emitter));
    }
    for (const auto& source : state.manualFlowPaths) {
        stateJson["water_manual_flow_paths"].push_back(SerializeWaterManualFlowPath(source));
    }
    for (const auto& node : state.seepageNodes) {
        stateJson["water_seepage_nodes"].push_back(SerializeWaterSeepageNode(node));
    }
    for (const auto& layer : state.rippleLayers) {
        stateJson["water_ripple_layers"].push_back(SerializeWaterEffectLayer(layer));
    }
    for (const auto& layer : state.fieldLayers) {
        stateJson["water_field_layers"].push_back(SerializeWaterEffectLayer(layer));
    }
    if (state.pathCacheManifest.has_value()) {
        stateJson["water_path_cache_manifest"] =
            SerializeWaterPathCacheManifest(state.pathCacheManifest.value());
    } else if (state.pathCache.has_value() && !state.pathCache->branches.empty() &&
               !state.pathCache->stale) {
        // Kept only as a compatibility fallback. Schema-42 project saves prepare
        // a binary sidecar and populate the manifest before reaching this point.
        stateJson["water_path_cache"] = SerializeWaterPathCache(state.pathCache.value());
    }
    for (const auto& cache : state.rippleRuntimeCaches) {
        if (ShouldSerializeWaterRippleRuntimeCache(cache)) {
            stateJson["water_ripple_runtime_caches"].push_back(SerializeWaterRippleRuntimeCache(cache));
        }
    }
    return stateJson;
}

WaterSceneStateDocument ParseWaterSceneState(
    const json& stateJson,
    bool defaultUseSurfaceGuide) {
    WaterSceneStateDocument state;
    state.sceneGroupName = stateJson.value("scene_group", state.sceneGroupName);
    if (state.sceneGroupName.empty()) {
        state.sceneGroupName = "Default";
    }
    state.dynamicMeshPath = stateJson.value("dynamic_mesh_path", std::string{});
    if (stateJson.contains("water_emitters") && stateJson.at("water_emitters").is_array()) {
        for (const auto& emitterJson : stateJson.at("water_emitters")) {
            state.emitters.push_back(ParseWaterEmitter(emitterJson));
        }
    }
    if (stateJson.contains("water_manual_flow_paths") &&
        stateJson.at("water_manual_flow_paths").is_array()) {
        for (const auto& sourceJson : stateJson.at("water_manual_flow_paths")) {
            state.manualFlowPaths.push_back(
                ParseWaterManualFlowPath(sourceJson, defaultUseSurfaceGuide));
        }
    }
    if (stateJson.contains("water_seepage_nodes") &&
        stateJson.at("water_seepage_nodes").is_array()) {
        for (const auto& nodeJson : stateJson.at("water_seepage_nodes")) {
            state.seepageNodes.push_back(ParseWaterSeepageNode(nodeJson));
        }
    }
    if (stateJson.contains("water_ripple_layers") && stateJson.at("water_ripple_layers").is_array()) {
        for (const auto& layerJson : stateJson.at("water_ripple_layers")) {
            state.rippleLayers.push_back(ParseWaterEffectLayer(layerJson));
        }
    }
    if (stateJson.contains("water_field_layers") && stateJson.at("water_field_layers").is_array()) {
        for (const auto& layerJson : stateJson.at("water_field_layers")) {
            auto layer = ParseWaterEffectLayer(layerJson);
            if (!layerJson.contains("feature_type")) {
                layer.featureType = WaterEffectFeatureType::FieldSurfaceMotion;
            }
            state.fieldLayers.push_back(std::move(layer));
        }
    }
    if (stateJson.contains("water_path_cache")) {
        state.pathCache = ParseWaterPathCache(stateJson.at("water_path_cache"));
    }
    if (stateJson.contains("water_path_cache_manifest") &&
        stateJson.at("water_path_cache_manifest").is_object()) {
        state.pathCacheManifest =
            ParseWaterPathCacheManifest(stateJson.at("water_path_cache_manifest"));
    }
    if (stateJson.contains("water_ripple_runtime_caches") &&
        stateJson.at("water_ripple_runtime_caches").is_array()) {
        for (const auto& cacheJson : stateJson.at("water_ripple_runtime_caches")) {
            auto cache = ParseWaterRippleRuntimeCache(cacheJson);
            if (!cache.memberships.empty() && !cache.params.empty()) {
                state.rippleRuntimeCaches.push_back(std::move(cache));
            }
        }
    }
    if (stateJson.contains("dynamic_mesh_attractors") &&
        stateJson.at("dynamic_mesh_attractors").is_array()) {
        for (const auto& attractorJson : stateJson.at("dynamic_mesh_attractors")) {
            state.dynamicMeshAttractors.push_back(ParseWaterDynamicMeshAttractor(attractorJson));
        }
    }
    if (stateJson.contains("dynamic_mesh_emitter_motions") &&
        stateJson.at("dynamic_mesh_emitter_motions").is_array()) {
        for (const auto& motionJson : stateJson.at("dynamic_mesh_emitter_motions")) {
            state.dynamicMeshEmitterMotions.push_back(ParseWaterDynamicMeshEmitterMotion(motionJson));
        }
    }
    return state;
}

bool WaterSceneStateHasPayload(const WaterSceneStateDocument& state) {
    return !state.emitters.empty() ||
           !state.manualFlowPaths.empty() ||
           !state.seepageNodes.empty() ||
           !state.rippleLayers.empty() ||
           !state.fieldLayers.empty() ||
           (state.pathCache.has_value() && !state.pathCache->branches.empty()) ||
           state.pathCacheManifest.has_value() ||
           !state.rippleRuntimeCaches.empty() ||
           !state.dynamicMeshPath.empty();
}

WaterSceneStateDocument MakeDefaultWaterSceneStateFromProject(const ProjectDocument& document) {
    WaterSceneStateDocument state;
    state.sceneGroupName = "Default";
    state.emitters = document.waterEmitters;
    state.manualFlowPaths = document.waterManualFlowPaths;
    state.seepageNodes = document.waterSeepageNodes;
    state.rippleLayers = document.waterRippleLayers;
    state.fieldLayers = document.waterFieldLayers;
    state.pathCache = document.waterPathCache;
    state.pathCacheManifest = document.waterPathCacheManifest;
    state.rippleRuntimeCaches = document.waterRippleRuntimeCaches;
    state.dynamicMeshPath = document.waterDynamicMeshFlowSettings.meshPath;
    return state;
}

json SerializeWaterRainSettings(
    const RainRuntimeSettings& settings,
    const WaterRainVisualSettings& visual) {
    return json{
        {"version", 3},
        {"enabled", settings.enabled},
        {"impact_effects_enabled", settings.impactEffectsEnabled},
        {"sand_effects_enabled", settings.sandEffectsEnabled},
        {"rock_effects_enabled", settings.rockEffectsEnabled},
        {"vegetation_effects_enabled", settings.vegetationEffectsEnabled},
        {"intensity_preset", invisible_places::water::WaterRainIntensityPresetNameForStorage(
                                 static_cast<invisible_places::water::WaterRainIntensityPreset>(
                                     settings.intensityPreset))},
        {"visual_profile_name", settings.visualProfileName},
        {"active_particle_count", settings.activeParticleCount},
        {"seed", settings.seed},
        {"rain_level", settings.rainLevel},
        {"density", settings.density},
        {"fall_speed_meters_per_second", settings.fallSpeedMetersPerSecond},
        {"droplet_size_scale", settings.dropletSizeScale},
        {"opacity_scale", settings.opacityScale},
        {"emission_scale", settings.emissionScale},
        {"spawn_height_meters", settings.spawnHeightMeters},
        {"spawn_radius_meters", settings.spawnRadiusMeters},
        {"camera_death_distance_meters", settings.cameraDeathDistanceMeters},
        {"wind_direction_x", settings.windDirectionX},
        {"wind_direction_y", settings.windDirectionY},
        {"wind_speed_meters_per_second", settings.windSpeedMetersPerSecond},
        {"turbulence", settings.turbulence},
        {"gust_strength", settings.gustStrength},
        {"gust_scale_meters", settings.gustScaleMeters},
        {"gust_speed_meters_per_second", settings.gustSpeedMetersPerSecond},
        {"weather_front_strength", settings.weatherFrontStrength},
        {"weather_front_scale_meters", settings.weatherFrontScaleMeters},
        {"weather_front_speed_meters_per_second", settings.weatherFrontSpeedMetersPerSecond},
        {"sand_effect_scale", settings.sandEffectScale},
        {"rock_effect_scale", settings.rockEffectScale},
        {"vegetation_effect_scale", settings.vegetationEffectScale},
        {"near_surface",
         {{"approach_distance_meters", settings.nearSurface.approachDistanceMeters},
          {"minimum_speed_factor", settings.nearSurface.minimumSpeedFactor},
          {"squish", settings.nearSurface.squish},
          {"normal_alignment", settings.nearSurface.normalAlignment}}},
        {"rock_impact",
         {{"edge_breakup", settings.rockImpact.edgeBreakup},
          {"spread_speed", settings.rockImpact.spreadSpeed},
          {"centre_falloff", settings.rockImpact.centreFalloff},
          {"height_bias", settings.rockImpact.heightBias},
          {"persistence", settings.rockImpact.persistence}}},
        {"vegetation_impact",
         {{"twinkle", settings.vegetationImpact.twinkle},
          {"propagation_meters_per_second", settings.vegetationImpact.propagationMetersPerSecond},
          {"hop_spacing_meters", settings.vegetationImpact.hopSpacingMeters},
          {"stream_width_meters", settings.vegetationImpact.streamWidthMeters},
          {"stream_spread", settings.vegetationImpact.streamSpread}}},
        {"visual_profile",
         {{"colour", visual.colour},
          {"width_meters", visual.widthMeters},
          {"streak_length_meters", visual.streakLengthMeters},
          {"softness", visual.softness},
          {"opacity", visual.opacity},
          {"emission", visual.emission},
          {"minimum_screen_pixels", visual.minimumScreenPixels},
          {"maximum_screen_pixels", visual.maximumScreenPixels}}},
    };
}

RainRuntimeSettings ParseWaterRainSettings(const json& settingsJson) {
    auto settings = invisible_places::water::DefaultRainRuntimeSettings();
    settings.enabled = false;
    const int version = settingsJson.is_object() ? settingsJson.value("version", 0) : 0;
    if (version < 2 || version > 3) {
        return settings;
    }
    settings.enabled = settingsJson.value("enabled", settings.enabled);
    settings.impactEffectsEnabled =
        settingsJson.value("impact_effects_enabled", settings.impactEffectsEnabled);
    settings.sandEffectsEnabled = settingsJson.value("sand_effects_enabled", settings.sandEffectsEnabled);
    settings.rockEffectsEnabled = settingsJson.value("rock_effects_enabled", settings.rockEffectsEnabled);
    settings.vegetationEffectsEnabled =
        settingsJson.value("vegetation_effects_enabled", settings.vegetationEffectsEnabled);
    if (settingsJson.contains("intensity_preset")) {
        const auto preset = invisible_places::water::ParseWaterRainIntensityPresetName(
            settingsJson.at("intensity_preset").get<std::string>());
        if (preset.has_value()) {
            settings.intensityPreset = static_cast<invisible_places::water::RainIntensityPreset>(preset.value());
        }
    }
    settings.visualProfileName = settingsJson.value("visual_profile_name", settings.visualProfileName);
    settings.activeParticleCount = settingsJson.value("active_particle_count", settings.activeParticleCount);
    settings.seed = settingsJson.value("seed", settings.seed);
    settings.rainLevel = settingsJson.value("rain_level", settings.rainLevel);
    settings.density = settingsJson.value("density", settings.density);
    settings.fallSpeedMetersPerSecond =
        settingsJson.value("fall_speed_meters_per_second", settings.fallSpeedMetersPerSecond);
    settings.dropletSizeScale = settingsJson.value("droplet_size_scale", settings.dropletSizeScale);
    settings.opacityScale = settingsJson.value("opacity_scale", settings.opacityScale);
    settings.emissionScale = settingsJson.value("emission_scale", settings.emissionScale);
    settings.spawnHeightMeters = settingsJson.value("spawn_height_meters", settings.spawnHeightMeters);
    settings.spawnRadiusMeters = settingsJson.value("spawn_radius_meters", settings.spawnRadiusMeters);
    settings.cameraDeathDistanceMeters =
        settingsJson.value("camera_death_distance_meters", settings.cameraDeathDistanceMeters);
    settings.windDirectionX = settingsJson.value("wind_direction_x", settings.windDirectionX);
    settings.windDirectionY = settingsJson.value("wind_direction_y", settings.windDirectionY);
    settings.windSpeedMetersPerSecond =
        settingsJson.value("wind_speed_meters_per_second", settings.windSpeedMetersPerSecond);
    settings.turbulence = settingsJson.value("turbulence", settings.turbulence);
    settings.gustStrength = settingsJson.value("gust_strength", settings.gustStrength);
    settings.gustScaleMeters = settingsJson.value("gust_scale_meters", settings.gustScaleMeters);
    settings.gustSpeedMetersPerSecond =
        settingsJson.value("gust_speed_meters_per_second", settings.gustSpeedMetersPerSecond);
    settings.weatherFrontStrength =
        settingsJson.value("weather_front_strength", settings.weatherFrontStrength);
    settings.weatherFrontScaleMeters =
        settingsJson.value("weather_front_scale_meters", settings.weatherFrontScaleMeters);
    settings.weatherFrontSpeedMetersPerSecond =
        settingsJson.value("weather_front_speed_meters_per_second", settings.weatherFrontSpeedMetersPerSecond);
    settings.sandEffectScale = settingsJson.value("sand_effect_scale", settings.sandEffectScale);
    settings.rockEffectScale = settingsJson.value("rock_effect_scale", settings.rockEffectScale);
    settings.vegetationEffectScale =
        settingsJson.value("vegetation_effect_scale", settings.vegetationEffectScale);
    if (version >= 3 && settingsJson.contains("near_surface") &&
        settingsJson.at("near_surface").is_object()) {
        const auto& tuning = settingsJson.at("near_surface");
        settings.nearSurface.approachDistanceMeters = std::max(
            0.0F,
            tuning.value("approach_distance_meters", settings.nearSurface.approachDistanceMeters));
        settings.nearSurface.minimumSpeedFactor = std::clamp(
            tuning.value("minimum_speed_factor", settings.nearSurface.minimumSpeedFactor),
            0.0F,
            1.0F);
        settings.nearSurface.squish = std::clamp(
            tuning.value("squish", settings.nearSurface.squish),
            0.0F,
            1.0F);
        settings.nearSurface.normalAlignment = std::clamp(
            tuning.value("normal_alignment", settings.nearSurface.normalAlignment),
            0.0F,
            1.0F);
    }
    if (version >= 3 && settingsJson.contains("rock_impact") &&
        settingsJson.at("rock_impact").is_object()) {
        const auto& tuning = settingsJson.at("rock_impact");
        settings.rockImpact.edgeBreakup = std::clamp(
            tuning.value("edge_breakup", settings.rockImpact.edgeBreakup), 0.0F, 1.0F);
        settings.rockImpact.spreadSpeed = std::max(
            0.0F, tuning.value("spread_speed", settings.rockImpact.spreadSpeed));
        settings.rockImpact.centreFalloff = std::clamp(
            tuning.value("centre_falloff", settings.rockImpact.centreFalloff), 0.0F, 1.0F);
        settings.rockImpact.heightBias = std::clamp(
            tuning.value("height_bias", settings.rockImpact.heightBias), 0.0F, 2.0F);
        settings.rockImpact.persistence = std::max(
            0.05F, tuning.value("persistence", settings.rockImpact.persistence));
    }
    if (version >= 3 && settingsJson.contains("vegetation_impact") &&
        settingsJson.at("vegetation_impact").is_object()) {
        const auto& tuning = settingsJson.at("vegetation_impact");
        settings.vegetationImpact.twinkle = std::max(
            0.0F, tuning.value("twinkle", settings.vegetationImpact.twinkle));
        settings.vegetationImpact.propagationMetersPerSecond = std::max(
            0.0F,
            tuning.value(
                "propagation_meters_per_second",
                settings.vegetationImpact.propagationMetersPerSecond));
        settings.vegetationImpact.hopSpacingMeters = std::max(
            0.001F,
            tuning.value("hop_spacing_meters", settings.vegetationImpact.hopSpacingMeters));
        settings.vegetationImpact.streamWidthMeters = std::max(
            0.0001F,
            tuning.value("stream_width_meters", settings.vegetationImpact.streamWidthMeters));
        settings.vegetationImpact.streamSpread = std::clamp(
            tuning.value("stream_spread", settings.vegetationImpact.streamSpread), 0.0F, 2.0F);
    }
    return settings;
}

WaterRainVisualSettings ParseWaterRainVisualSettings(const json& settingsJson) {
    std::string profileName = "Rain Fine Lines";
    if (settingsJson.is_object() && settingsJson.value("version", 0) >= 2 &&
        settingsJson.value("version", 0) <= 3) {
        profileName = settingsJson.value("visual_profile_name", profileName);
    }
    auto visual = invisible_places::water::RainVisualPreset(profileName);
    if (!settingsJson.is_object() || settingsJson.value("version", 0) < 2 ||
        settingsJson.value("version", 0) > 3 ||
        !settingsJson.contains("visual_profile") || !settingsJson.at("visual_profile").is_object()) {
        return visual;
    }
    const auto& visualJson = settingsJson.at("visual_profile");
    if (visualJson.contains("colour")) {
        visual.colour = visualJson.at("colour").get<std::array<float, 3>>();
    }
    visual.widthMeters = visualJson.value("width_meters", visual.widthMeters);
    visual.streakLengthMeters = visualJson.value("streak_length_meters", visual.streakLengthMeters);
    visual.softness = visualJson.value("softness", visual.softness);
    visual.opacity = visualJson.value("opacity", visual.opacity);
    visual.emission = visualJson.value("emission", visual.emission);
    visual.minimumScreenPixels =
        visualJson.value("minimum_screen_pixels", visual.minimumScreenPixels);
    visual.maximumScreenPixels =
        visualJson.value("maximum_screen_pixels", visual.maximumScreenPixels);
    return visual;
}

json SerializeWaterPathGenerationSettings(const WaterPathGenerationSettings& settings) {
    return json{
        {"auto_tune", settings.autoTune},
        {"support_voxel_size", settings.supportVoxelSize},
        {"max_bridge_distance", settings.maxBridgeDistance},
        {"smoothing", settings.smoothing},
        {"path_length", settings.pathLength},
        {"path_sample_spacing", settings.pathSampleSpacing},
        {"branching", settings.branching},
        {"coverage", settings.coverage},
        {"gap_tolerance", settings.gapTolerance},
        {"attractor_enabled", settings.attractorEnabled},
        {"attractor_position", {settings.attractorPosition.x, settings.attractorPosition.y, settings.attractorPosition.z}},
        {"attractor_strength", settings.attractorStrength},
        {"max_steps", settings.maxSteps},
        {"support_sample_limit", settings.supportSampleLimit},
    };
}

WaterPathGenerationSettings ParseWaterPathGenerationSettings(const json& settingsJson) {
    WaterPathGenerationSettings settings;
    if (settingsJson.contains("scale_mode")) {
        settings.legacyScaleMode = ParseWaterScaleMode(settingsJson.at("scale_mode"));
        settings = invisible_places::water::DefaultWaterPathGenerationSettings(settings.legacyScaleMode);
    }
    settings.autoTune = settingsJson.value("auto_tune", settings.autoTune);
    settings.supportVoxelSize = settingsJson.value("support_voxel_size", settings.supportVoxelSize);
    settings.maxBridgeDistance = settingsJson.value("max_bridge_distance", settings.maxBridgeDistance);
    settings.smoothing = settingsJson.value("smoothing", settings.smoothing);
    settings.pathLength = settingsJson.value("path_length", settings.pathLength);
    settings.pathSampleSpacing = settingsJson.value(
        "path_sample_spacing",
        settingsJson.value("path_density", settings.pathSampleSpacing));
    settings.branching = std::clamp(settingsJson.value("branching", settings.branching), 0.0F, 1.0F);
    settings.coverage = std::clamp(settingsJson.value("coverage", settings.coverage), 0.0F, 1.0F);
    settings.gapTolerance = std::clamp(settingsJson.value("gap_tolerance", settings.gapTolerance), 0.0F, 1.0F);
    settings.attractorEnabled = settingsJson.value("attractor_enabled", settings.attractorEnabled);
    if (settingsJson.contains("attractor_position")) {
        const auto position = settingsJson.at("attractor_position").get<std::array<float, 3>>();
        settings.attractorPosition = {position[0], position[1], position[2]};
    }
    settings.attractorStrength = std::clamp(
        settingsJson.value("attractor_strength", settings.attractorStrength),
        0.0F,
        1.0F);
    settings.maxSteps = settingsJson.value("max_steps", settings.maxSteps);
    settings.supportSampleLimit = settingsJson.value("support_sample_limit", settings.supportSampleLimit);
    return settings;
}

json SerializeWaterParticleTrailSettings(const WaterParticleTrailSettings& settings) {
    return json{
        {"particle_density", settings.particleDensity},
        {"particle_jitter", settings.particleJitter},
        {"particle_speed", settings.particleSpeed},
        {"spline_anchor_spacing", settings.splineAnchorSpacing},
    };
}

WaterParticleTrailSettings ParseWaterParticleTrailSettings(const json& settingsJson) {
    WaterParticleTrailSettings settings;
    settings.particleDensity =
        std::clamp(settingsJson.value("particle_density", settings.particleDensity), 0.05F, 10.0F);
    settings.particleJitter =
        std::clamp(settingsJson.value("particle_jitter", settings.particleJitter), 0.0F, 3.0F);
    settings.particleSpeed =
        std::clamp(settingsJson.value("particle_speed", settings.particleSpeed), 0.05F, 8.0F);
    settings.splineAnchorSpacing =
        std::clamp(settingsJson.value("spline_anchor_spacing", settings.splineAnchorSpacing), 0.01F, 25.0F);
    return settings;
}

json SerializeWaterParticleTrailShapeSettings(const WaterParticleTrailShapeSettings& settings) {
    return json{
        {"particle_jitter", settings.particleJitter},
        {"spline_anchor_spacing", settings.splineAnchorSpacing},
        {"trail_lane_count", settings.trailLaneCount},
        {"trail_looseness", settings.trailLooseness},
        {"trail_smoothness", settings.trailSmoothness},
        {"trail_turbulence", settings.trailTurbulence},
        {"trail_momentum", settings.trailMomentum},
        {"normal_turbulence_response", settings.normalTurbulenceResponse},
    };
}

WaterParticleTrailShapeSettings ParseWaterParticleTrailShapeSettings(const json& settingsJson) {
    WaterParticleTrailShapeSettings settings;
    settings.particleJitter =
        std::clamp(settingsJson.value("particle_jitter", settings.particleJitter), 0.0F, 3.0F);
    settings.splineAnchorSpacing =
        std::clamp(settingsJson.value("spline_anchor_spacing", settings.splineAnchorSpacing), 0.01F, 25.0F);
    settings.trailLaneCount =
        std::clamp<std::uint32_t>(settingsJson.value("trail_lane_count", settings.trailLaneCount), 0U, 32U);
    settings.trailTurbulence =
        std::clamp(settingsJson.value("trail_turbulence", settings.trailTurbulence), 0.0F, 3.0F);
    settings.trailMomentum =
        std::clamp(settingsJson.value("trail_momentum", settings.trailMomentum), 0.0F, 0.98F);
    settings.normalTurbulenceResponse =
        std::clamp(settingsJson.value("normal_turbulence_response", settings.normalTurbulenceResponse), 0.0F, 3.0F);
    if (settingsJson.contains("trail_looseness")) {
        settings.trailLooseness =
            std::clamp(settingsJson.value("trail_looseness", settings.trailLooseness), 0.0F, 1.0F);
    } else if (
        settingsJson.contains("trail_turbulence") ||
        settingsJson.contains("trail_momentum") ||
        settingsJson.contains("normal_turbulence_response")) {
        settings.trailLooseness = std::clamp(
            (std::clamp(settings.trailTurbulence / 1.5F, 0.0F, 1.0F) * 0.45F) +
                (settings.trailMomentum * 0.35F) +
                (std::clamp(settings.normalTurbulenceResponse / 1.5F, 0.0F, 1.0F) * 0.20F),
            0.0F,
            1.0F);
    }
    settings.trailSmoothness =
        std::clamp(settingsJson.value("trail_smoothness", settings.trailSmoothness), 0.0F, 1.0F);
    return settings;
}

json SerializeWaterAnimationTrailSettings(const WaterAnimationTrailSettings& settings) {
    return json{
        {"particle_density", settings.particleDensity},
        {"particle_speed", settings.particleSpeed},
        {"color_variation", settings.colorVariation},
        {"trail_length_meters", settings.trailLengthMeters},
        {"trail_sample_spacing_meters", settings.trailSampleSpacingMeters},
    };
}

WaterAnimationTrailSettings ParseWaterAnimationTrailSettings(const json& settingsJson) {
    WaterAnimationTrailSettings settings;
    settings.particleDensity =
        std::clamp(settingsJson.value("particle_density", settings.particleDensity), 0.05F, 10.0F);
    settings.particleSpeed =
        std::clamp(settingsJson.value("particle_speed", settings.particleSpeed), 0.05F, 8.0F);
    settings.colorVariation =
        std::clamp(settingsJson.value("color_variation", settings.colorVariation), 0.0F, 1.0F);
    settings.trailLengthMeters =
        std::clamp(settingsJson.value("trail_length_meters", settings.trailLengthMeters), 0.0F, 25.0F);
    settings.trailSampleSpacingMeters =
        std::clamp(settingsJson.value("trail_sample_spacing_meters", settings.trailSampleSpacingMeters), 0.0F, 25.0F);
    return settings;
}

json SerializeWaterTrailGeometrySettings(const WaterTrailGeometrySettings& settings) {
    return json{
        {"trail_length_meters", settings.trailLengthMeters},
        {"point_spacing_meters", settings.pointSpacingMeters},
        {"width_meters", settings.widthMeters},
        {"streak_length_meters", settings.streakLengthMeters},
    };
}

WaterTrailGeometrySettings ParseWaterTrailGeometrySettings(const json& settingsJson) {
    WaterTrailGeometrySettings settings;
    settings.trailLengthMeters =
        std::clamp(settingsJson.value("trail_length_meters", settings.trailLengthMeters), 0.001F, 50.0F);
    settings.pointSpacingMeters =
        std::clamp(settingsJson.value("point_spacing_meters", settings.pointSpacingMeters), 0.001F, 10.0F);
    settings.widthMeters =
        std::clamp(settingsJson.value("width_meters", settings.widthMeters), 0.0005F, 1.0F);
    settings.streakLengthMeters = std::clamp(
        settingsJson.value(
            "streak_length_meters",
            settingsJson.value("world_length_meters", settings.streakLengthMeters)),
        0.001F,
        5.0F);
    return settings;
}

json SerializeWaterPathProfile(const WaterPathProfileDocument& profile) {
    return json{
        {"name", profile.name},
        {"settings", SerializeWaterPathGenerationSettings(profile.settings)},
    };
}

WaterPathProfileDocument ParseWaterPathProfile(const json& profileJson) {
    WaterPathProfileDocument profile;
    profile.name = profileJson.value("name", profile.name);
    if (profileJson.contains("settings")) {
        profile.settings = ParseWaterPathGenerationSettings(profileJson.at("settings"));
    }
    return profile;
}

json SerializeWaterLaneProfile(const WaterLaneProfileDocument& profile) {
    return json{
        {"name", profile.name},
        {"settings", SerializeWaterFlowTrailSettings(profile.settings)},
    };
}

WaterLaneProfileDocument ParseWaterLaneProfile(const json& profileJson) {
    WaterLaneProfileDocument profile;
    profile.name = profileJson.value("name", profile.name);
    if (profileJson.contains("settings")) {
        profile.settings = ParseWaterFlowTrailSettings(profileJson.at("settings"));
    }
    return profile;
}

json SerializeWaterTrailProfile(const WaterTrailProfileDocument& profile) {
    return json{
        {"name", profile.name},
        {"geometry", SerializeWaterTrailGeometrySettings(profile.geometry)},
        {"style", SerializePointCloudStyle(profile.style)},
    };
}

WaterTrailProfileDocument ParseWaterTrailProfile(const json& profileJson) {
    WaterTrailProfileDocument profile;
    profile.name = profileJson.value("name", profile.name);
    if (profileJson.contains("geometry")) {
        profile.geometry = ParseWaterTrailGeometrySettings(profileJson.at("geometry"));
    }
    if (profileJson.contains("style")) {
        profile.style = ParsePointCloudStyle(profileJson.at("style"));
    }
    return profile;
}

json SerializeWaterAnimationTrailProfile(const WaterAnimationTrailProfileDocument& profile) {
    return json{
        {"name", profile.name},
        {"settings", SerializeWaterAnimationTrailSettings(profile.settings)},
    };
}

WaterAnimationTrailProfileDocument ParseWaterAnimationTrailProfile(const json& profileJson) {
    WaterAnimationTrailProfileDocument profile;
    profile.name = profileJson.value("name", profile.name);
    if (profileJson.contains("settings")) {
        profile.settings = ParseWaterAnimationTrailSettings(profileJson.at("settings"));
    }
    return profile;
}

json SerializeWaterCausticLookSettings(const WaterCausticLookSettings& settings) {
    return json{
        {"enabled", settings.enabled},
        {"intensity", settings.intensity},
        {"scale", settings.scale},
        {"speed", settings.speed},
        {"line_sharpness", settings.lineSharpness},
        {"warp", settings.warp},
        {"cell_size_meters", settings.cellSizeMeters},
        {"line_width_meters", settings.lineWidthMeters},
        {"feather_meters", settings.featherMeters},
        {"surface_point_spacing_meters", settings.surfacePointSpacingMeters},
        {"warp_amplitude_meters", settings.warpAmplitudeMeters},
        {"tint", std::array<float, 3>{settings.tintRed, settings.tintGreen, settings.tintBlue}},
        {"emission_boost", settings.emissionBoost},
        {"opacity_boost", settings.opacityBoost},
        {"point_size_boost", settings.pointSizeBoost},
    };
}

WaterCausticLookSettings ParseWaterCausticLookSettings(const json& settingsJson) {
    WaterCausticLookSettings settings;
    settings.enabled = settingsJson.value("enabled", settings.enabled);
    settings.intensity = std::clamp(settingsJson.value("intensity", settings.intensity), 0.0F, 5.0F);
    settings.scale = std::clamp(settingsJson.value("scale", settings.scale), 0.01F, 80.0F);
    settings.speed = std::clamp(settingsJson.value("speed", settings.speed), 0.0F, 10.0F);
    settings.lineSharpness = std::clamp(settingsJson.value("line_sharpness", settings.lineSharpness), 0.0F, 1.0F);
    settings.warp = std::clamp(settingsJson.value("warp", settings.warp), 0.0F, 3.0F);
    const bool hasLegacyPattern =
        settingsJson.contains("scale") ||
        settingsJson.contains("line_sharpness") ||
        settingsJson.contains("warp");
    const bool hasCellSize = settingsJson.contains("cell_size_meters");
    const bool hasLineWidth = settingsJson.contains("line_width_meters");
    const bool hasFeather = settingsJson.contains("feather_meters");
    const bool hasPointSpacing = settingsJson.contains("surface_point_spacing_meters");
    const bool hasWarpAmplitude = settingsJson.contains("warp_amplitude_meters");
    const float legacyCellSize = 1.0F / std::max(0.01F, settings.scale);
    const float legacyLineWidth =
        legacyCellSize * (0.16F + (0.025F - 0.16F) * std::clamp(settings.lineSharpness, 0.0F, 1.0F));
    settings.cellSizeMeters = std::clamp(
        settingsJson.value(
            "cell_size_meters",
            !hasCellSize && hasLegacyPattern ? legacyCellSize : settings.cellSizeMeters),
        0.005F,
        5.0F);
    settings.lineWidthMeters = std::clamp(
        settingsJson.value(
            "line_width_meters",
            !hasLineWidth && hasLegacyPattern ? legacyLineWidth : settings.lineWidthMeters),
        0.0005F,
        0.50F);
    settings.featherMeters = std::clamp(
        settingsJson.value(
            "feather_meters",
            !hasFeather && hasLegacyPattern ? settings.lineWidthMeters * 0.4F : settings.featherMeters),
        0.0005F,
        0.50F);
    settings.surfacePointSpacingMeters = std::clamp(
        settingsJson.value("surface_point_spacing_meters", settings.surfacePointSpacingMeters),
        0.0005F,
        0.10F);
    if (!hasPointSpacing) {
        settings.surfacePointSpacingMeters = std::clamp(settings.surfacePointSpacingMeters, 0.001F, 0.020F);
    }
    settings.warpAmplitudeMeters = std::clamp(
        settingsJson.value(
            "warp_amplitude_meters",
            !hasWarpAmplitude && hasLegacyPattern ? legacyCellSize * settings.warp * 0.5F
                                                  : settings.warpAmplitudeMeters),
        0.0F,
        2.0F);
    if (settingsJson.contains("tint")) {
        const auto tint = settingsJson.at("tint").get<std::array<float, 3>>();
        settings.tintRed = std::clamp(tint[0], 0.0F, 4.0F);
        settings.tintGreen = std::clamp(tint[1], 0.0F, 4.0F);
        settings.tintBlue = std::clamp(tint[2], 0.0F, 4.0F);
    }
    settings.emissionBoost = std::clamp(settingsJson.value("emission_boost", settings.emissionBoost), 0.0F, 8.0F);
    settings.opacityBoost = std::clamp(settingsJson.value("opacity_boost", settings.opacityBoost), 0.0F, 2.0F);
    settings.pointSizeBoost =
        std::clamp(settingsJson.value("point_size_boost", settings.pointSizeBoost), 0.0F, 4.0F);
    return settings;
}

json SerializeWaterParticleVisualSettings(const WaterParticleVisualSettings& settings) {
    return json{
        {"particle_size_pixels", settings.particleSizePixels},
        {"particle_opacity", settings.particleOpacity},
        {"color_variation", settings.colorVariation},
        {"glow", settings.glow},
    };
}

WaterParticleVisualSettings ParseWaterParticleVisualSettings(const json& settingsJson) {
    WaterParticleVisualSettings settings;
    settings.particleSizePixels =
        std::clamp(settingsJson.value("particle_size_pixels", settings.particleSizePixels), 1.0F, 96.0F);
    settings.particleOpacity =
        std::clamp(settingsJson.value("particle_opacity", settings.particleOpacity), 0.0F, 1.0F);
    settings.colorVariation =
        std::clamp(settingsJson.value("color_variation", settings.colorVariation), 0.0F, 1.0F);
    settings.glow = std::clamp(settingsJson.value("glow", settings.glow), 0.0F, 4.0F);
    return settings;
}

json SerializeWaterSourceSettings(const WaterSourceSettings& settings) {
    return json{
        {"path_generation", SerializeWaterPathGenerationSettings(settings.path)},
        {"trail_shape", SerializeWaterParticleTrailShapeSettings(settings.trailShape)},
    };
}

WaterSourceSettings ParseWaterSourceSettings(const json& settingsJson) {
    WaterSourceSettings settings;
    if (settingsJson.contains("path_generation")) {
        settings.path = ParseWaterPathGenerationSettings(settingsJson.at("path_generation"));
    } else if (settingsJson.contains("support_voxel_size") || settingsJson.contains("scale_mode")) {
        settings.path = ParseWaterPathGenerationSettings(settingsJson);
    }
    if (settingsJson.contains("trail_shape")) {
        settings.trailShape = ParseWaterParticleTrailShapeSettings(settingsJson.at("trail_shape"));
    } else if (settingsJson.contains("point_trail")) {
        const auto legacyTrail = ParseWaterParticleTrailSettings(settingsJson.at("point_trail"));
        settings.trailShape.particleJitter = legacyTrail.particleJitter;
        settings.trailShape.splineAnchorSpacing = legacyTrail.splineAnchorSpacing;
    } else if (settingsJson.contains("particle_jitter") || settingsJson.contains("spline_anchor_spacing")) {
        settings.trailShape = ParseWaterParticleTrailShapeSettings(settingsJson);
    }
    return settings;
}

json SerializeWaterVisualSettings(const WaterVisualSettings& settings) {
    return SerializeWaterParticleVisualSettings(settings);
}

WaterVisualSettings ParseWaterVisualSettings(const json& settingsJson) {
    return ParseWaterParticleVisualSettings(settingsJson);
}

void ConfigureLegacyWaterFieldBinding(
    RenderParameterBinding* binding,
    std::int32_t fieldSlot,
    const std::string& fieldName,
    float outputMin,
    float outputMax) {
    if (binding == nullptr) {
        return;
    }
    binding->active = true;
    binding->mode = ParameterSourceMode::FieldMapped;
    binding->fieldMap.fieldSlot = fieldSlot;
    binding->fieldMap.fieldName = fieldName;
    binding->fieldMap.inputMin = 0.0F;
    binding->fieldMap.inputMax = 1.0F;
    binding->fieldMap.outputMin = outputMin;
    binding->fieldMap.outputMax = outputMax;
    binding->fieldMap.gamma = 1.0F;
    invisible_places::style::SetFieldMapFlag(
        &binding->fieldMap,
        invisible_places::style::FieldMapFlagUseLayerStats,
        false);
}

PointCloudStyleState MakeLegacyWaterPointVisualStyle(const WaterVisualSettings& visualSettings) {
    PointCloudStyleState style;
    style.geometryMode = PointCloudGeometryMode::ScreenSprites;
    style.falloffProfile = PointCloudFalloffProfile::Gaussian;
    style.colorMode = PointCloudColorMode::SourceRgb;
    style.solidColor = {0.04F, 0.74F, 1.0F, 1.0F};
    style.colorizeColor = {0.05F, 0.82F, 1.0F};
    style.colorizeAmount = 0.0F;
    style.exposure = 1.8F;
    style.gaussianSharpness = 1.65F;
    style.solidCenters = true;
    style.flowAnimation = true;
    style.waterPathView = false;
    invisible_places::style::SetScalarConstant(
        &style.pointSize,
        std::clamp(visualSettings.particleSizePixels, 1.0F, 96.0F));
    ConfigureLegacyWaterFieldBinding(
        &style.opacity,
        6,
        "confidence",
        0.0F,
        std::clamp(visualSettings.particleOpacity, 0.0F, 1.0F));
    ConfigureLegacyWaterFieldBinding(
        &style.emissiveStrength,
        7,
        "accumulation",
        0.0F,
        std::clamp(visualSettings.glow, 0.0F, 4.0F));
    invisible_places::style::SetScalarConstant(&style.depthFade, 0.0F);
    invisible_places::style::SetScalarConstant(&style.colormapPosition, 0.5F);
    invisible_places::style::SetScalarConstant(&style.surfelDiameter, 0.012F);
    return style;
}

json SerializeWaterSettingsBundle(const WaterSettingsBundle& settings) {
    return json{
        {"path_generation", SerializeWaterPathGenerationSettings(settings.path)},
        {"point_trail", SerializeWaterParticleTrailSettings(settings.trail)},
        {"visuals", SerializeWaterParticleVisualSettings(settings.visual)},
    };
}

WaterSettingsBundle ParseWaterSettingsBundle(const json& settingsJson) {
    WaterSettingsBundle settings;
    if (settingsJson.contains("path_generation")) {
        settings.path = ParseWaterPathGenerationSettings(settingsJson.at("path_generation"));
    } else if (settingsJson.contains("support_voxel_size") || settingsJson.contains("scale_mode")) {
        settings.path = ParseWaterPathGenerationSettings(settingsJson);
    }
    if (settingsJson.contains("point_trail")) {
        settings.trail = ParseWaterParticleTrailSettings(settingsJson.at("point_trail"));
    } else if (settingsJson.contains("particle_density") || settingsJson.contains("spline_anchor_spacing")) {
        settings.trail = ParseWaterParticleTrailSettings(settingsJson);
    }
    if (settingsJson.contains("visuals")) {
        settings.visual = ParseWaterParticleVisualSettings(settingsJson.at("visuals"));
    } else if (settingsJson.contains("particle_size_pixels") || settingsJson.contains("glow")) {
        settings.visual = ParseWaterParticleVisualSettings(settingsJson);
    }
    return settings;
}

json SerializeWaterBakeSettings(const WaterBakeSettings& settings) {
    return SerializeWaterPathGenerationSettings(settings);
}

WaterBakeSettings ParseWaterBakeSettings(const json& settingsJson) {
    return ParseWaterPathGenerationSettings(settingsJson);
}

json SerializeWaterRenderSettings(const WaterRenderSettings& settings) {
    return SerializeWaterSettingsBundle(settings);
}

WaterRenderSettings ParseWaterRenderSettings(const json& settingsJson) {
    return ParseWaterSettingsBundle(settingsJson);
}

std::string WaterPathBranchRoleName(WaterPathBranchRole role) {
    switch (role) {
        case WaterPathBranchRole::Main:
            return "main";
        case WaterPathBranchRole::Secondary:
            return "secondary";
        case WaterPathBranchRole::Spread:
            return "spread";
    }
    return "main";
}

WaterPathBranchRole ParseWaterPathBranchRole(const json& roleJson) {
    const auto roleName = roleJson.get<std::string>();
    if (roleName == "secondary") {
        return WaterPathBranchRole::Secondary;
    }
    if (roleName == "spread") {
        return WaterPathBranchRole::Spread;
    }
    return WaterPathBranchRole::Main;
}

std::string WaterPathTerminationReasonName(WaterPathTerminationReason reason) {
    switch (reason) {
        case WaterPathTerminationReason::ReachedLength:
            return "reached_length";
        case WaterPathTerminationReason::NoSupport:
            return "no_support";
        case WaterPathTerminationReason::MaxSteps:
            return "max_steps";
        case WaterPathTerminationReason::Loop:
            return "loop";
        case WaterPathTerminationReason::Duplicate:
            return "duplicate";
        case WaterPathTerminationReason::Empty:
            return "empty";
    }
    return "empty";
}

WaterPathTerminationReason ParseWaterPathTerminationReason(const json& reasonJson) {
    const auto reasonName = reasonJson.get<std::string>();
    if (reasonName == "reached_length") {
        return WaterPathTerminationReason::ReachedLength;
    }
    if (reasonName == "no_support") {
        return WaterPathTerminationReason::NoSupport;
    }
    if (reasonName == "max_steps") {
        return WaterPathTerminationReason::MaxSteps;
    }
    if (reasonName == "loop") {
        return WaterPathTerminationReason::Loop;
    }
    if (reasonName == "duplicate") {
        return WaterPathTerminationReason::Duplicate;
    }
    return WaterPathTerminationReason::Empty;
}

json SerializeWaterOverlayPoint(const invisible_places::water::WaterOverlayPoint& point) {
    return json{
        {"position", std::array<float, 3>{point.position.x, point.position.y, point.position.z}},
        {"rgb", std::array<std::uint32_t, 3>{point.red, point.green, point.blue}},
        {"flow_id", point.flowId},
        {"emitter_id", point.emitterId},
        {"path_distance", point.pathDistance},
        {"phase", point.phase},
        {"speed", point.speed},
        {"width", point.width},
        {"confidence", point.confidence},
        {"accumulation", point.accumulation},
        {"pooling", point.pooling},
        {"particle_role", point.particleRole},
        {"path_start_index", point.pathStartIndex},
        {"path_point_count", point.pathPointCount},
        {"jitter_seed", point.jitterSeed},
        {"trail_age", point.trailAge},
        {"trail_length", point.trailLength},
        {"feature_type", point.featureType},
        {"region_id", point.regionId},
        {"surface_steepness", point.surfaceSteepness},
        {"trail_lane_id", point.trailLaneId},
        {"trail_lateral_offset", point.trailLateralOffset},
    };
}

invisible_places::water::WaterOverlayPoint ParseWaterOverlayPoint(const json& pointJson) {
    invisible_places::water::WaterOverlayPoint point;
    if (pointJson.contains("position")) {
        const auto position = pointJson.at("position").get<std::array<float, 3>>();
        point.position = {position[0], position[1], position[2]};
    }
    if (pointJson.contains("rgb")) {
        const auto rgb = pointJson.at("rgb").get<std::array<std::uint32_t, 3>>();
        point.red = static_cast<std::uint8_t>(std::min<std::uint32_t>(255U, rgb[0]));
        point.green = static_cast<std::uint8_t>(std::min<std::uint32_t>(255U, rgb[1]));
        point.blue = static_cast<std::uint8_t>(std::min<std::uint32_t>(255U, rgb[2]));
    }
    point.flowId = pointJson.value("flow_id", point.flowId);
    point.emitterId = pointJson.value("emitter_id", point.emitterId);
    point.pathDistance = pointJson.value("path_distance", point.pathDistance);
    point.phase = pointJson.value("phase", point.phase);
    point.speed = pointJson.value("speed", point.speed);
    point.width = pointJson.value("width", point.width);
    point.confidence = pointJson.value("confidence", point.confidence);
    point.accumulation = pointJson.value("accumulation", point.accumulation);
    point.pooling = pointJson.value("pooling", point.pooling);
    point.particleRole = pointJson.value("particle_role", point.particleRole);
    point.pathStartIndex = pointJson.value("path_start_index", point.pathStartIndex);
    point.pathPointCount = pointJson.value("path_point_count", point.pathPointCount);
    point.jitterSeed = pointJson.value("jitter_seed", point.jitterSeed);
    point.trailAge = pointJson.value("trail_age", point.trailAge);
    point.trailLength = pointJson.value("trail_length", point.trailLength);
    point.featureType = pointJson.value("feature_type", point.featureType);
    point.regionId = pointJson.value("region_id", point.regionId);
    point.surfaceSteepness = pointJson.value("surface_steepness", point.surfaceSteepness);
    point.trailLaneId = pointJson.value("trail_lane_id", point.trailLaneId);
    point.trailLateralOffset = pointJson.value("trail_lateral_offset", point.trailLateralOffset);
    return point;
}

json SerializeWaterPathDiagnostics(const WaterPathAutoTuneDiagnostics& diagnostics) {
    return json{
        {"estimated_point_spacing", diagnostics.estimatedPointSpacing},
        {"support_voxel_size", diagnostics.supportVoxelSize},
        {"max_bridge_distance", diagnostics.maxBridgeDistance},
        {"path_sample_spacing", diagnostics.pathSampleSpacing},
        {"branch_search_radius", diagnostics.branchSearchRadius},
        {"average_confidence", diagnostics.averageConfidence},
        {"iteration_count", diagnostics.iterationCount},
        {"pilot_trace_count", diagnostics.pilotTraceCount},
        {"branch_count", diagnostics.branchCount},
        {"low_confidence_branch_count", diagnostics.lowConfidenceBranchCount},
        {"summary", diagnostics.summary},
    };
}

WaterPathAutoTuneDiagnostics ParseWaterPathDiagnostics(const json& diagnosticsJson) {
    WaterPathAutoTuneDiagnostics diagnostics;
    diagnostics.estimatedPointSpacing =
        diagnosticsJson.value("estimated_point_spacing", diagnostics.estimatedPointSpacing);
    diagnostics.supportVoxelSize = diagnosticsJson.value("support_voxel_size", diagnostics.supportVoxelSize);
    diagnostics.maxBridgeDistance = diagnosticsJson.value("max_bridge_distance", diagnostics.maxBridgeDistance);
    diagnostics.pathSampleSpacing = diagnosticsJson.value("path_sample_spacing", diagnostics.pathSampleSpacing);
    diagnostics.branchSearchRadius = diagnosticsJson.value("branch_search_radius", diagnostics.branchSearchRadius);
    diagnostics.averageConfidence = diagnosticsJson.value("average_confidence", diagnostics.averageConfidence);
    diagnostics.iterationCount = diagnosticsJson.value("iteration_count", diagnostics.iterationCount);
    diagnostics.pilotTraceCount = diagnosticsJson.value("pilot_trace_count", diagnostics.pilotTraceCount);
    diagnostics.branchCount = diagnosticsJson.value("branch_count", diagnostics.branchCount);
    diagnostics.lowConfidenceBranchCount =
        diagnosticsJson.value("low_confidence_branch_count", diagnostics.lowConfidenceBranchCount);
    diagnostics.summary = diagnosticsJson.value("summary", diagnostics.summary);
    return diagnostics;
}

json SerializeWaterPathBranch(const WaterPathBranch& branch) {
    json branchJson{
        {"id", branch.id},
        {"emitter_id", branch.emitterId},
        {"role", WaterPathBranchRoleName(branch.role)},
        {"termination_reason", WaterPathTerminationReasonName(branch.terminationReason)},
        {"confidence", branch.confidence},
        {"length", branch.length},
        {"flatness", branch.flatness},
        {"gap_count", branch.gapCount},
        {"bake_fingerprint", branch.bakeFingerprint},
        {"raw_anchors", json::array()},
    };
    if (branch.parentId.has_value()) {
        branchJson["parent_id"] = branch.parentId.value();
    }
    for (const auto& point : branch.rawAnchors) {
        branchJson["raw_anchors"].push_back(SerializeWaterOverlayPoint(point));
    }
    return branchJson;
}

WaterPathBranch ParseWaterPathBranch(const json& branchJson) {
    WaterPathBranch branch;
    branch.id = branchJson.value("id", 0U);
    if (branchJson.contains("parent_id")) {
        branch.parentId = branchJson.at("parent_id").get<std::uint32_t>();
    }
    branch.emitterId = branchJson.value("emitter_id", 0U);
    if (branchJson.contains("role")) {
        branch.role = ParseWaterPathBranchRole(branchJson.at("role"));
    }
    if (branchJson.contains("termination_reason")) {
        branch.terminationReason = ParseWaterPathTerminationReason(branchJson.at("termination_reason"));
    }
    branch.confidence = branchJson.value("confidence", branch.confidence);
    branch.length = branchJson.value("length", branch.length);
    branch.flatness = branchJson.value("flatness", branch.flatness);
    branch.gapCount = branchJson.value("gap_count", branch.gapCount);
    branch.bakeFingerprint = branchJson.value("bake_fingerprint", branch.bakeFingerprint);
    if (branchJson.contains("raw_anchors") && branchJson.at("raw_anchors").is_array()) {
        for (const auto& pointJson : branchJson.at("raw_anchors")) {
            branch.rawAnchors.push_back(ParseWaterOverlayPoint(pointJson));
        }
    }
    return branch;
}

json SerializeWaterPathAnalysisSample(const WaterPathAnalysisSample& sample) {
    return json{
        {"branch_id", sample.branchId},
        {"sample_index", sample.sampleIndex},
        {"path_distance", sample.pathDistance},
        {"slope", sample.slope},
        {"flatness", sample.flatness},
        {"curvature", sample.curvature},
        {"neighbor_density", sample.neighborDensity},
        {"nearest_path_distance", sample.nearestPathDistance},
        {"confluence", sample.confluence},
        {"channel_width", sample.channelWidth},
        {"speed", sample.speed},
        {"turbulence", sample.turbulence},
        {"eddy_potential", sample.eddyPotential},
        {"ripple_potential", sample.ripplePotential},
    };
}

WaterPathAnalysisSample ParseWaterPathAnalysisSample(const json& sampleJson) {
    WaterPathAnalysisSample sample;
    sample.branchId = sampleJson.value("branch_id", sample.branchId);
    sample.sampleIndex = sampleJson.value("sample_index", sample.sampleIndex);
    sample.pathDistance = sampleJson.value("path_distance", sample.pathDistance);
    sample.slope = std::clamp(sampleJson.value("slope", sample.slope), 0.0F, 1.0F);
    sample.flatness = std::clamp(sampleJson.value("flatness", sample.flatness), 0.0F, 1.0F);
    sample.curvature = std::clamp(sampleJson.value("curvature", sample.curvature), 0.0F, 1.0F);
    sample.neighborDensity =
        std::clamp(sampleJson.value("neighbor_density", sample.neighborDensity), 0.0F, 1.0F);
    sample.nearestPathDistance =
        std::max(0.0F, sampleJson.value("nearest_path_distance", sample.nearestPathDistance));
    sample.confluence = std::clamp(sampleJson.value("confluence", sample.confluence), 0.0F, 1.0F);
    sample.channelWidth = std::max(0.0F, sampleJson.value("channel_width", sample.channelWidth));
    sample.speed = std::max(0.0F, sampleJson.value("speed", sample.speed));
    sample.turbulence = std::clamp(sampleJson.value("turbulence", sample.turbulence), 0.0F, 1.0F);
    sample.eddyPotential = std::clamp(sampleJson.value("eddy_potential", sample.eddyPotential), 0.0F, 1.0F);
    sample.ripplePotential =
        std::clamp(sampleJson.value("ripple_potential", sample.ripplePotential), 0.0F, 1.0F);
    return sample;
}

json SerializeWaterPathBranchAnalysis(const WaterPathBranchAnalysis& branchAnalysis) {
    json branchJson{
        {"branch_id", branchAnalysis.branchId},
        {"samples", json::array()},
    };
    for (const auto& sample : branchAnalysis.samples) {
        branchJson["samples"].push_back(SerializeWaterPathAnalysisSample(sample));
    }
    return branchJson;
}

WaterPathBranchAnalysis ParseWaterPathBranchAnalysis(const json& branchJson) {
    WaterPathBranchAnalysis branchAnalysis;
    branchAnalysis.branchId = branchJson.value("branch_id", branchAnalysis.branchId);
    if (branchJson.contains("samples") && branchJson.at("samples").is_array()) {
        for (const auto& sampleJson : branchJson.at("samples")) {
            branchAnalysis.samples.push_back(ParseWaterPathAnalysisSample(sampleJson));
        }
    }
    return branchAnalysis;
}

json SerializeWaterPathAnalysisCache(const WaterPathAnalysisCache& analysis) {
    json analysisJson{
        {"schema_version", analysis.schemaVersion},
        {"analysis_radius_meters", analysis.analysisRadiusMeters},
        {"branches", json::array()},
    };
    for (const auto& branchAnalysis : analysis.branches) {
        analysisJson["branches"].push_back(SerializeWaterPathBranchAnalysis(branchAnalysis));
    }
    return analysisJson;
}

WaterPathAnalysisCache ParseWaterPathAnalysisCache(const json& analysisJson) {
    WaterPathAnalysisCache analysis;
    analysis.schemaVersion = analysisJson.value("schema_version", analysis.schemaVersion);
    analysis.analysisRadiusMeters =
        std::max(0.0F, analysisJson.value("analysis_radius_meters", analysis.analysisRadiusMeters));
    if (analysisJson.contains("branches") && analysisJson.at("branches").is_array()) {
        for (const auto& branchJson : analysisJson.at("branches")) {
            analysis.branches.push_back(ParseWaterPathBranchAnalysis(branchJson));
        }
    }
    return analysis;
}

json SerializeWaterPathCache(const WaterPathCache& cache) {
    json cacheJson{
        {"schema_version", cache.schemaVersion},
        {"support_layer_path", cache.supportLayerPath.generic_string()},
        {"support_signature", cache.supportSignature},
        {"emitter_settings_fingerprint", cache.emitterSettingsFingerprint},
        {"requested_settings", SerializeWaterPathGenerationSettings(cache.requestedSettings)},
        {"tuned_settings", SerializeWaterPathGenerationSettings(cache.tunedSettings)},
        {"diagnostics", SerializeWaterPathDiagnostics(cache.diagnostics)},
        {"hidden_branch_ids", cache.hiddenBranchIds},
        {"stale", cache.stale},
        {"branches", json::array()},
    };
    for (const auto& branch : cache.branches) {
        cacheJson["branches"].push_back(SerializeWaterPathBranch(branch));
    }
    if (cache.analysis.has_value()) {
        cacheJson["analysis"] = SerializeWaterPathAnalysisCache(cache.analysis.value());
    }
    return cacheJson;
}

WaterPathCache ParseWaterPathCache(const json& cacheJson) {
    WaterPathCache cache;
    cache.schemaVersion = cacheJson.value("schema_version", 1U);
    cache.supportLayerPath = cacheJson.value("support_layer_path", std::string{});
    cache.supportSignature = cacheJson.value("support_signature", cache.supportSignature);
    cache.emitterSettingsFingerprint =
        cacheJson.value("emitter_settings_fingerprint", cache.emitterSettingsFingerprint);
    if (cacheJson.contains("requested_settings")) {
        cache.requestedSettings = ParseWaterPathGenerationSettings(cacheJson.at("requested_settings"));
    }
    if (cacheJson.contains("tuned_settings")) {
        cache.tunedSettings = ParseWaterPathGenerationSettings(cacheJson.at("tuned_settings"));
    }
    if (cacheJson.contains("diagnostics")) {
        cache.diagnostics = ParseWaterPathDiagnostics(cacheJson.at("diagnostics"));
    }
    if (cacheJson.contains("hidden_branch_ids") && cacheJson.at("hidden_branch_ids").is_array()) {
        cache.hiddenBranchIds = cacheJson.at("hidden_branch_ids").get<std::vector<std::uint32_t>>();
    }
    cache.stale = cacheJson.value("stale", cache.stale);
    if (cacheJson.contains("branches") && cacheJson.at("branches").is_array()) {
        for (const auto& branchJson : cacheJson.at("branches")) {
            cache.branches.push_back(ParseWaterPathBranch(branchJson));
        }
    }
    if (cacheJson.contains("analysis")) {
        cache.analysis = ParseWaterPathAnalysisCache(cacheJson.at("analysis"));
    }
    return cache;
}

std::array<std::uint64_t, 4> MakeWaterPathCachePayloadDigest() {
    return {
        1469598103934665603ULL,
        1099511628211ULL,
        0x9E3779B97F4A7C15ULL,
        0xD6E8FEB86659FD93ULL,
    };
}

void UpdateWaterPathCachePayloadDigest(
    std::array<std::uint64_t, 4>* digest,
    const std::uint8_t* bytes,
    std::size_t byteCount) {
    if (digest == nullptr || bytes == nullptr) {
        return;
    }
    constexpr std::array<std::uint64_t, 4> primes{
        1099511628211ULL,
        0x100000001B3ULL,
        0x9E3779B185EBCA87ULL,
        0xC2B2AE3D27D4EB4FULL,
    };
    for (std::size_t index = 0U; index < byteCount; ++index) {
        for (std::size_t lane = 0U; lane < digest->size(); ++lane) {
            (*digest)[lane] ^= static_cast<std::uint64_t>(bytes[index]) + lane * 0x9DU;
            (*digest)[lane] *= primes[lane];
            (*digest)[lane] ^= (*digest)[lane] >> 31U;
        }
    }
}

std::array<std::uint64_t, 4> DigestWaterPathCachePayload(
    const std::vector<std::uint8_t>& payload) {
    auto digest = MakeWaterPathCachePayloadDigest();
    UpdateWaterPathCachePayloadDigest(&digest, payload.data(), payload.size());
    return digest;
}

bool WaterPathCacheMayFitPersistenceCeiling(const WaterPathCache& cache) {
    // CBOR retains the descriptive object keys for every point. This deliberately
    // conservative bound rejects pathological caches before constructing either
    // the full JSON DOM or the encoded payload.
    std::uint64_t estimate = 64ULL * 1024ULL;
    auto add = [&](std::uint64_t count, std::uint64_t bytesPerItem) {
        if (count > (kMaximumPersistedWaterCacheBytes -
                     std::min(estimate, kMaximumPersistedWaterCacheBytes)) /
                        bytesPerItem) {
            estimate = kMaximumPersistedWaterCacheBytes + 1ULL;
            return;
        }
        estimate += count * bytesPerItem;
    };
    add(cache.supportLayerPath.generic_string().size(), 4ULL);
    add(cache.supportSignature.size(), 4ULL);
    add(cache.emitterSettingsFingerprint.size(), 4ULL);
    add(cache.diagnostics.summary.size(), 4ULL);
    add(cache.hiddenBranchIds.size(), 16ULL);
    for (const auto& branch : cache.branches) {
        add(1ULL, 4096ULL);
        add(branch.bakeFingerprint.size(), 4ULL);
        add(branch.rawAnchors.size(), 1024ULL);
    }
    if (cache.analysis.has_value()) {
        for (const auto& branch : cache.analysis->branches) {
            add(1ULL, 512ULL);
            add(branch.samples.size(), 512ULL);
        }
    }
    return estimate <= kMaximumPersistedWaterCacheBytes;
}

struct EncodedWaterPathCache {
    std::vector<std::uint8_t> payload;
    std::array<std::uint64_t, 4> checksum{};
};

std::optional<EncodedWaterPathCache> EncodeWaterPathCache(
    const WaterPathCache& document,
    std::string* errorMessage) {
    if (!WaterPathCacheMayFitPersistenceCeiling(document)) {
        if (errorMessage != nullptr) {
            *errorMessage =
                "Water path cache estimate exceeds the 5 GiB persistence ceiling.";
        }
        return std::nullopt;
    }
    try {
        EncodedWaterPathCache encoded;
        encoded.payload = json::to_cbor(SerializeWaterPathCache(document));
        if (encoded.payload.size() > kMaximumPersistedWaterCacheBytes) {
            if (errorMessage != nullptr) {
                *errorMessage = "Water path cache exceeds the 5 GiB persistence ceiling.";
            }
            return std::nullopt;
        }
        encoded.checksum = DigestWaterPathCachePayload(encoded.payload);
        return encoded;
    } catch (const std::exception& error) {
        if (errorMessage != nullptr) {
            *errorMessage = "Failed to encode water path cache sidecar: " +
                            std::string{error.what()};
        }
        return std::nullopt;
    }
}

std::string WaterPathCacheChecksumToken(
    const std::array<std::uint64_t, 4>& checksum) {
    std::ostringstream token;
    token << std::hex << std::setfill('0');
    for (const auto lane : checksum) {
        token << std::setw(16) << lane;
    }
    return token.str();
}

bool ExistingWaterPathCacheMatches(
    const std::filesystem::path& outputPath,
    const EncodedWaterPathCache& encoded) {
    std::error_code sizeError;
    const auto fileBytes = std::filesystem::file_size(outputPath, sizeError);
    const std::uint64_t headerBytes =
        kWaterPathCacheSidecarMagic.size() + sizeof(std::uint32_t) +
        sizeof(std::uint64_t) + encoded.checksum.size() * sizeof(encoded.checksum.front());
    if (sizeError || fileBytes != headerBytes + encoded.payload.size()) {
        return false;
    }
    std::ifstream input{outputPath, std::ios::binary};
    std::array<char, kWaterPathCacheSidecarMagic.size()> magic{};
    std::uint32_t schema = 0U;
    std::uint64_t payloadBytes = 0U;
    std::array<std::uint64_t, 4> checksum{};
    input.read(magic.data(), static_cast<std::streamsize>(magic.size()));
    input.read(reinterpret_cast<char*>(&schema), sizeof(schema));
    input.read(reinterpret_cast<char*>(&payloadBytes), sizeof(payloadBytes));
    input.read(
        reinterpret_cast<char*>(checksum.data()),
        static_cast<std::streamsize>(checksum.size() * sizeof(checksum.front())));
    if (!input.good() || magic != kWaterPathCacheSidecarMagic ||
        schema != kWaterPathCacheSidecarSchemaVersion ||
        payloadBytes != encoded.payload.size() || checksum != encoded.checksum) {
        return false;
    }
    auto streamedChecksum = MakeWaterPathCachePayloadDigest();
    std::array<std::uint8_t, 64U * 1024U> buffer{};
    std::uint64_t remaining = payloadBytes;
    while (remaining > 0U) {
        const auto chunk = static_cast<std::size_t>(
            std::min<std::uint64_t>(remaining, buffer.size()));
        input.read(
            reinterpret_cast<char*>(buffer.data()),
            static_cast<std::streamsize>(chunk));
        if (input.gcount() != static_cast<std::streamsize>(chunk)) {
            return false;
        }
        UpdateWaterPathCachePayloadDigest(&streamedChecksum, buffer.data(), chunk);
        remaining -= chunk;
    }
    return streamedChecksum == encoded.checksum &&
           input.peek() == std::char_traits<char>::eof();
}

bool WriteEncodedWaterPathCache(
    const WaterPathCache& document,
    const EncodedWaterPathCache& encoded,
    const std::filesystem::path& outputPath,
    std::string* errorMessage,
    WaterPathCacheManifestDocument* manifest) {
    if (const auto parent = outputPath.parent_path(); !parent.empty()) {
        std::error_code createError;
        std::filesystem::create_directories(parent, createError);
        if (createError) {
            if (errorMessage != nullptr) {
                *errorMessage = "Failed to create water path cache directory: " +
                                createError.message();
            }
            return false;
        }
    }
    if (!ExistingWaterPathCacheMatches(outputPath, encoded)) {
        const auto temporaryPath = outputPath.string() + ".tmp";
        std::ofstream output{temporaryPath, std::ios::binary | std::ios::trunc};
        if (!output.is_open()) {
            if (errorMessage != nullptr) {
                *errorMessage = "Failed to open water path cache sidecar for writing.";
            }
            return false;
        }
        const std::uint32_t schema = kWaterPathCacheSidecarSchemaVersion;
        const std::uint64_t payloadBytes = encoded.payload.size();
        output.write(kWaterPathCacheSidecarMagic.data(), kWaterPathCacheSidecarMagic.size());
        output.write(reinterpret_cast<const char*>(&schema), sizeof(schema));
        output.write(reinterpret_cast<const char*>(&payloadBytes), sizeof(payloadBytes));
        output.write(
            reinterpret_cast<const char*>(encoded.checksum.data()),
            static_cast<std::streamsize>(
                encoded.checksum.size() * sizeof(encoded.checksum.front())));
        output.write(
            reinterpret_cast<const char*>(encoded.payload.data()),
            static_cast<std::streamsize>(encoded.payload.size()));
        output.flush();
        bool writeSucceeded = output.good();
        output.close();
        writeSucceeded = writeSucceeded && !output.fail();
        if (!writeSucceeded) {
            std::error_code cleanupError;
            std::filesystem::remove(temporaryPath, cleanupError);
            if (errorMessage != nullptr) {
                *errorMessage = "Failed to write complete water path cache sidecar.";
            }
            return false;
        }
        std::error_code renameError;
        std::filesystem::rename(temporaryPath, outputPath, renameError);
        if (renameError) {
            std::error_code cleanupError;
            std::filesystem::remove(temporaryPath, cleanupError);
            if (errorMessage != nullptr) {
                *errorMessage = "Failed to atomically replace water path cache sidecar: " +
                                renameError.message();
            }
            return false;
        }
    }
    if (manifest != nullptr) {
        manifest->relativePath = outputPath;
        manifest->cacheSchema = kWaterPathCacheSidecarSchemaVersion;
        manifest->supportSignature = document.supportSignature;
        manifest->emitterSettingsFingerprint = document.emitterSettingsFingerprint;
        manifest->payloadBytes = encoded.payload.size();
        manifest->checksum = encoded.checksum;
    }
    return true;
}

template <typename TDocument>
bool WriteJsonDocument(
    const TDocument& document,
    const json& documentJson,
    const std::filesystem::path& outputPath,
    std::string* errorMessage) {
    (void)document;
    if (const auto parentPath = outputPath.parent_path(); !parentPath.empty()) {
        std::error_code createError;
        std::filesystem::create_directories(parentPath, createError);
        if (createError) {
            if (errorMessage != nullptr) {
                *errorMessage = "Failed to create output directory: " + createError.message();
            }
            return false;
        }
    }

    const auto temporaryPath = outputPath.string() + ".tmp";
    std::ofstream output{temporaryPath, std::ios::trunc};
    if (!output.is_open()) {
        if (errorMessage != nullptr) {
            *errorMessage = "Failed to open output file for writing.";
        }
        return false;
    }

    output << documentJson.dump(2);
    output.flush();
    bool writeSucceeded = output.good();
    output.close();
    writeSucceeded = writeSucceeded && !output.fail();
    if (!writeSucceeded) {
        std::error_code cleanupError;
        std::filesystem::remove(temporaryPath, cleanupError);
        if (errorMessage != nullptr) {
            *errorMessage = "Failed to write complete output file.";
        }
        return false;
    }
    std::error_code renameError;
    std::filesystem::rename(temporaryPath, outputPath, renameError);
    if (renameError) {
        std::error_code cleanupError;
        std::filesystem::remove(temporaryPath, cleanupError);
        if (errorMessage != nullptr) {
            *errorMessage = "Failed to atomically replace output file: " + renameError.message();
        }
        return false;
    }
    return true;
}

std::optional<json> ReadJsonDocument(const std::filesystem::path& inputPath, std::string* errorMessage) {
    std::ifstream input{inputPath, std::ios::binary};
    if (!input.is_open()) {
        if (errorMessage != nullptr) {
            *errorMessage = "Failed to open input file.";
        }
        return std::nullopt;
    }

    try {
        std::error_code sizeError;
        const auto fileSize = std::filesystem::file_size(inputPath, sizeError);
        if (!sizeError && fileSize >= kLargeJsonRippleCacheStripBytes) {
            std::ostringstream buffer;
            buffer << input.rdbuf();
            auto text = buffer.str();
            StripJsonArrayPropertyValue(&text, "water_ripple_runtime_caches");
            return json::parse(text);
        }
        return json::parse(input);
    } catch (const std::exception& error) {
        if (errorMessage != nullptr) {
            *errorMessage = "Failed to parse JSON: " + std::string{error.what()};
        }
        return std::nullopt;
    }
}

std::optional<WaterPathCache> PruneSettledWaterPathCache(
    WaterPathCache cache,
    const std::vector<WaterEmitter>& emitters) {
    if (cache.stale || cache.branches.empty()) {
        return std::nullopt;
    }
    std::unordered_set<std::uint32_t> emitterIds;
    emitterIds.reserve(emitters.size());
    for (const auto& emitter : emitters) {
        emitterIds.insert(emitter.id);
    }
    std::erase_if(cache.branches, [&](const WaterPathBranch& branch) {
        return branch.emitterId != 0U && !emitterIds.contains(branch.emitterId);
    });
    if (cache.branches.empty()) {
        return std::nullopt;
    }
    std::unordered_set<std::uint32_t> branchIds;
    branchIds.reserve(cache.branches.size());
    for (const auto& branch : cache.branches) {
        branchIds.insert(branch.id);
    }
    for (auto& branch : cache.branches) {
        if (branch.parentId.has_value() && !branchIds.contains(branch.parentId.value())) {
            branch.parentId.reset();
        }
    }
    std::erase_if(cache.hiddenBranchIds, [&](std::uint32_t branchId) {
        return !branchIds.contains(branchId);
    });
    if (cache.analysis.has_value()) {
        std::erase_if(cache.analysis->branches, [&](const WaterPathBranchAnalysis& branch) {
            return !branchIds.contains(branch.branchId);
        });
    }
    cache.diagnostics.branchCount = static_cast<std::uint32_t>(cache.branches.size());
    cache.stale = false;
    return cache;
}

const ScenePointCloudGroupDocument* FindSceneGroupDocument(
    const ProjectDocument& document,
    std::string_view sceneGroupName) {
    const auto it = std::find_if(
        document.scenePointCloudGroups.begin(),
        document.scenePointCloudGroups.end(),
        [&](const ScenePointCloudGroupDocument& group) {
            return group.sceneGroupName == sceneGroupName;
        });
    return it == document.scenePointCloudGroups.end() ? nullptr : &*it;
}

std::string SelectedLayerSceneGroupName(const ProjectDocument& document) {
    if (document.selectedLayerPath.empty()) {
        return {};
    }
    const auto layerIt = std::find_if(
        document.layers.begin(),
        document.layers.end(),
        [&](const ProjectLayerDocument& layer) {
            return SerializedPathsMatch(layer.sourcePath, document.selectedLayerPath) ||
                   SerializedPathsMatch(
                       layer.selectedSceneVariantPath,
                       document.selectedLayerPath);
        });
    return layerIt == document.layers.end()
               ? std::string{}
               : TrimAsciiWhitespace(layerIt->sceneGroupName);
}

std::string UniqueVisibleSceneGroupName(const ProjectDocument& document) {
    const ScenePointCloudGroupDocument* visibleGroup = nullptr;
    for (const auto& group : document.scenePointCloudGroups) {
        if (!group.displayVisible || TrimAsciiWhitespace(group.sceneGroupName).empty()) {
            continue;
        }
        if (visibleGroup != nullptr) {
            return {};
        }
        visibleGroup = &group;
    }
    return visibleGroup == nullptr
               ? std::string{}
               : TrimAsciiWhitespace(visibleGroup->sceneGroupName);
}

std::string ResolveActiveWaterSceneGroupName(const ProjectDocument& document) {
    if (auto explicitScene =
            TrimAsciiWhitespace(document.activeWaterSceneGroupName);
        !explicitScene.empty()) {
        return explicitScene;
    }
    if (auto selectedScene = SelectedLayerSceneGroupName(document);
        !selectedScene.empty()) {
        return selectedScene;
    }
    if (auto visibleScene = UniqueVisibleSceneGroupName(document);
        !visibleScene.empty()) {
        return visibleScene;
    }
    const auto defaultIt = std::find_if(
        document.waterSceneStates.begin(),
        document.waterSceneStates.end(),
        [](const WaterSceneStateDocument& state) {
            return TrimAsciiWhitespace(state.sceneGroupName) == "Default";
        });
    if (defaultIt != document.waterSceneStates.end()) {
        return "Default";
    }
    if (!document.waterSceneStates.empty()) {
        return TrimAsciiWhitespace(document.waterSceneStates.front().sceneGroupName);
    }
    if (document.scenePointCloudGroups.size() == 1U) {
        return TrimAsciiWhitespace(
            document.scenePointCloudGroups.front().sceneGroupName);
    }
    return "Default";
}

const WaterSceneStateDocument* FindActiveWaterSceneState(
    const ProjectDocument& document) {
    const auto sceneGroupName = ResolveActiveWaterSceneGroupName(document);
    const auto it = std::find_if(
        document.waterSceneStates.begin(),
        document.waterSceneStates.end(),
        [&](const WaterSceneStateDocument& state) {
            return TrimAsciiWhitespace(state.sceneGroupName) == sceneGroupName;
        });
    return it == document.waterSceneStates.end() ? nullptr : &*it;
}

void ApplyActiveWaterSceneState(ProjectDocument* document) {
    if (document == nullptr) {
        return;
    }
    document->activeWaterSceneGroupName =
        ResolveActiveWaterSceneGroupName(*document);
    const auto* activeSceneState = FindActiveWaterSceneState(*document);
    if (activeSceneState == nullptr) {
        return;
    }
    document->waterEmitters = activeSceneState->emitters;
    document->waterManualFlowPaths = activeSceneState->manualFlowPaths;
    document->waterSeepageNodes = activeSceneState->seepageNodes;
    document->waterRippleLayers = activeSceneState->rippleLayers;
    document->waterFieldLayers = activeSceneState->fieldLayers;
    document->waterPathCache = activeSceneState->pathCache;
    document->waterPathCacheManifest = activeSceneState->pathCacheManifest;
    document->waterRippleRuntimeCaches = activeSceneState->rippleRuntimeCaches;
    document->waterDynamicMeshFlowSettings.meshPath =
        activeSceneState->dynamicMeshPath;
    // Authored Mesh Flow attractors and ordinary Flow-emitter motions are
    // migration-only input. Automatic Ground emergence must never acquire
    // those legacy source overrides when a scene state becomes active.
    document->waterDynamicMeshFlowSettings.attractors.clear();
    document->waterDynamicMeshFlowSettings.emitterMotions.clear();
}

std::filesystem::path SceneCacheRootForWaterState(
    const ProjectDocument& document,
    const WaterSceneStateDocument& state,
    const std::filesystem::path& projectPath) {
    if (const auto* group = FindSceneGroupDocument(document, state.sceneGroupName); group != nullptr) {
        for (const auto& role : group->roleSources) {
            const auto& sourcePath = !role.analysisSourcePath.empty()
                                         ? role.analysisSourcePath
                                         : role.displaySourcePath;
            if (!sourcePath.empty() && !sourcePath.parent_path().empty()) {
                auto resolvedSourcePath = sourcePath.lexically_normal();
                if (!sourcePath.is_absolute()) {
                    const auto projectRelativePath =
                        (projectPath.parent_path() / sourcePath).lexically_normal();
                    std::error_code projectRelativeError;
                    if (std::filesystem::is_regular_file(
                            projectRelativePath,
                            projectRelativeError)) {
                        resolvedSourcePath = projectRelativePath;
                    } else {
                        std::error_code currentWorkingDirectoryError;
                        const auto currentWorkingDirectoryPath =
                            std::filesystem::absolute(
                                sourcePath,
                                currentWorkingDirectoryError)
                                .lexically_normal();
                        std::error_code currentWorkingDirectoryFileError;
                        if (!currentWorkingDirectoryError &&
                            std::filesystem::is_regular_file(
                                currentWorkingDirectoryPath,
                                currentWorkingDirectoryFileError)) {
                            resolvedSourcePath = currentWorkingDirectoryPath;
                        } else {
                            resolvedSourcePath = projectRelativePath;
                        }
                    }
                }
                return resolvedSourcePath.parent_path() /
                       ".invisible_places" / "cache" / "flow";
            }
        }
    }
    return projectPath.parent_path() / ".invisible_places" / "cache" / "flow";
}

std::filesystem::path ManifestPathRelativeToProject(
    const std::filesystem::path& sidecarPath,
    const std::filesystem::path& projectPath) {
    std::error_code relativeError;
    auto relative = std::filesystem::relative(sidecarPath, projectPath.parent_path(), relativeError);
    return relativeError || relative.empty() ? sidecarPath : relative;
}

WaterSceneStateDocument PrepareWaterSceneStateForSave(
    const ProjectDocument& document,
    const WaterSceneStateDocument& sourceState,
    const std::filesystem::path& projectPath) {
    // Copy authored state first, but deliberately leave the potentially very
    // large derived path cache behind until it has passed the persistence
    // preflight. This keeps a pathological cache from being duplicated in RAM
    // merely because the user saves the project.
    WaterSceneStateDocument state;
    state.sceneGroupName = sourceState.sceneGroupName;
    state.emitters = sourceState.emitters;
    state.manualFlowPaths = sourceState.manualFlowPaths;
    state.seepageNodes = sourceState.seepageNodes;
    state.rippleLayers = sourceState.rippleLayers;
    state.fieldLayers = sourceState.fieldLayers;
    state.pathCacheManifest = sourceState.pathCacheManifest;
    state.rippleRuntimeCaches = sourceState.rippleRuntimeCaches;
    state.dynamicMeshPath = sourceState.dynamicMeshPath;

    if (!sourceState.pathCache.has_value()) {
        // A manifest without its validated payload is only a reference to
        // missing or corrupt derived data. Do not perpetuate it on the next
        // settled save.
        state.pathCacheManifest.reset();
        return state;
    }
    if (!WaterPathCacheMayFitPersistenceCeiling(sourceState.pathCache.value())) {
        // Authored Flow sources remain saveable even when a corrupt or
        // pathological derived cache is too large to persist safely.
        state.pathCacheManifest.reset();
        return state;
    }
    state.pathCache = sourceState.pathCache;
    const auto pruned = PruneSettledWaterPathCache(
        std::move(state.pathCache.value()),
        state.emitters);
    if (!pruned.has_value()) {
        state.pathCache.reset();
        state.pathCacheManifest.reset();
        return state;
    }

    auto sidecarDirectory = SceneCacheRootForWaterState(document, state, projectPath);
    std::filesystem::path sidecarPath;
    WaterPathCacheManifestDocument manifest;
    std::string sidecarError;
    if (!SaveContentAddressedWaterPathCacheDocument(
            pruned.value(),
            sidecarDirectory,
            &sidecarError,
            &manifest,
            &sidecarPath)) {
        sidecarDirectory =
            projectPath.parent_path() / ".invisible_places" / "cache" / "flow";
        sidecarError.clear();
        if (!SaveContentAddressedWaterPathCacheDocument(
                pruned.value(),
                sidecarDirectory,
                &sidecarError,
                &manifest,
                &sidecarPath)) {
            // Authored source data remains in the project; omit only the derived cache
            // when neither the scene nor project cache location is writable.
            state.pathCache.reset();
            state.pathCacheManifest.reset();
            return state;
        }
    }
    manifest.relativePath = ManifestPathRelativeToProject(sidecarPath, projectPath);
    manifest.supportSignature = pruned->supportSignature;
    manifest.emitterSettingsFingerprint = pruned->emitterSettingsFingerprint;
    state.pathCacheManifest = std::move(manifest);
    state.pathCache.reset();
    return state;
}

std::filesystem::path ResolveManifestPath(
    const std::filesystem::path& projectPath,
    const std::filesystem::path& manifestPath) {
    return manifestPath.is_absolute()
               ? manifestPath
               : (projectPath.parent_path() / manifestPath).lexically_normal();
}

void LoadWaterPathSidecars(
    const std::filesystem::path& projectPath,
    ProjectDocument* document) {
    if (document == nullptr) {
        return;
    }
    for (auto& state : document->waterSceneStates) {
        if (!state.pathCacheManifest.has_value()) {
            if (state.pathCache.has_value()) {
                state.pathCache = PruneSettledWaterPathCache(
                    std::move(state.pathCache.value()),
                    state.emitters);
            }
            continue;
        }

        // A schema-43 manifest is authoritative. Never fall back to an
        // embedded cache if its referenced sidecar is absent or invalid.
        state.pathCache.reset();
        if (state.pathCacheManifest->relativePath.empty()) {
            continue;
        }
        WaterPathCacheManifestDocument loadedManifest;
        std::string loadError;
        auto loaded = LoadWaterPathCacheDocument(
            ResolveManifestPath(projectPath, state.pathCacheManifest->relativePath),
            &loadError,
            &loadedManifest);
        if (!loaded.has_value() || loaded->stale ||
            loadedManifest.cacheSchema != state.pathCacheManifest->cacheSchema ||
            loadedManifest.payloadBytes != state.pathCacheManifest->payloadBytes ||
            loadedManifest.checksum != state.pathCacheManifest->checksum ||
            loaded->supportSignature != state.pathCacheManifest->supportSignature ||
            loaded->emitterSettingsFingerprint !=
                state.pathCacheManifest->emitterSettingsFingerprint) {
            continue;
        }
        state.pathCache = PruneSettledWaterPathCache(
            std::move(loaded.value()),
            state.emitters);
    }
}

}  // namespace

bool SaveProjectDocument(
    const ProjectDocument& document,
    const std::filesystem::path& outputPath,
    std::string* errorMessage) {
    const auto activeWaterSceneGroupName =
        ResolveActiveWaterSceneGroupName(document);
    json projectJson{
        {"schema_version", kProjectDocumentSchemaVersion},
        {"project_name", document.projectName},
        {"active_water_scene_group", activeWaterSceneGroupName},
        {"selected_layer_path", document.selectedLayerPath.generic_string()},
        {"last_animation_path", document.lastAnimationPath.generic_string()},
        {"background_color", document.backgroundColor},
        {"eye_dome_lighting_enabled", document.eyeDomeLightingEnabled},
        {"pro_res_alpha_preview_enabled", document.proResAlphaPreviewEnabled},
        {"eye_dome_lighting_thickness", document.eyeDomeLightingThickness},
        {"constant_update_view", document.constantUpdateView},
        {"live_visual_effects", document.liveVisualEffects},
        {"side_panel_pinned", document.sidePanelPinned},
        {"show_lidar_tab", document.showLidarTab},
        {"show_gsplat_tab", document.showGsplatTab},
        {"auto_lower_gsplat_quality_while_navigating", document.autoLowerGsplatQualityWhileNavigating},
        {"point_visuals", json::array()},
        {"selected_point_visual", document.selectedPointVisualName.empty() ? std::string{"Unnamed"}
                                                                           : document.selectedPointVisualName},
        {"scene_visual_states", json::array()},
        {"scene_point_cloud_groups", json::array()},
        {"gsplat_visual_style", SerializeGaussianSplatStyle(document.gsplatVisualStyle)},
        {"point_cloud_preview_lod_mode", PointCloudPreviewLodModeName(document.pointCloudPreviewLodMode)},
        {"interactive_point_cap", document.interactivePointCap},
        {"point_cloud_renderer_mode", PointCloudRendererModeName(document.pointCloudRendererMode)},
        {"render_job", SerializeRenderJobSettings(document.renderJobSettings)},
        {"export_presets", json::array()},
        {"selected_export_preset", document.selectedExportPresetName},
        {"water_source_settings", SerializeWaterSourceSettings(document.waterSourceSettings)},
        {"water_animation_trail_settings", SerializeWaterAnimationTrailSettings(document.waterAnimationTrailSettings)},
        {"water_animation_trail_profiles", json::array()},
        {"water_trail_geometry", SerializeWaterTrailGeometrySettings(document.waterTrailGeometry)},
        {"water_path_profiles", json::array()},
        {"water_lane_profiles", json::array()},
        {"water_trail_profiles", json::array()},
        {"selected_water_path_profile", document.selectedWaterPathProfileName},
        {"selected_water_lane_profile", document.selectedWaterLaneProfileName},
        {"selected_water_trail_profile", document.selectedWaterTrailProfileName},
        {"water_caustic_look_settings", SerializeWaterCausticLookSettings(document.waterCausticLookSettings)},
        {"water_seepage_nodes", json::array()},
        {"water_seepage_default_look", SerializeWaterSeepageLookSettings(document.waterSeepageDefaultLook)},
        {"water_seepage_look_profiles", json::array()},
        {"water_scenarios", json::array()},
        {"selected_water_scenario", document.selectedWaterScenarioId},
        {"water_timing_runs", json::array()},
        {"water_timing_run_sequence", document.waterTimingRunSequence},
        {"water_flow_trail_settings", SerializeWaterFlowTrailSettings(document.waterFlowTrailSettings)},
        {"water_show_flow_trails", document.waterShowFlowTrails},
        {"water_field_settings", SerializeWaterFieldSettings(document.waterFieldSettings)},
        {"water_field_trail_settings", SerializeWaterFieldTrailSettings(document.waterFieldTrailSettings)},
        {"water_dynamic_mesh_flow_settings",
         SerializeWaterDynamicMeshFlowSettings(
             ProjectLevelWaterDynamicMeshFlowSettings(document.waterDynamicMeshFlowSettings))},
        {"water_rain_settings",
         SerializeWaterRainSettings(document.waterRainSettings, document.waterRainVisualSettings)},
        {"water_point_visuals", json::array()},
        {"selected_water_point_visual", document.selectedWaterPointVisualName},
        {"water_scene_states", json::array()},
    };
    for (const auto& visual : document.pointVisuals) {
        projectJson["point_visuals"].push_back(SerializePointCloudVisual(visual));
    }
    for (const auto& state : document.sceneVisualStates) {
        projectJson["scene_visual_states"].push_back(SerializeScenePointVisualState(state));
    }
    for (const auto& group : document.scenePointCloudGroups) {
        projectJson["scene_point_cloud_groups"].push_back(SerializeScenePointCloudGroup(group));
    }
    for (const auto& preset : document.exportPresets) {
        projectJson["export_presets"].push_back(SerializeExportPreset(preset));
    }
    if (document.tempExportPreset.has_value()) {
        projectJson["temp_export_preset"] = SerializeExportPreset(document.tempExportPreset.value());
    }
    if (!document.waterSceneStates.empty()) {
        for (const auto& state : document.waterSceneStates) {
            auto preparedState = PrepareWaterSceneStateForSave(document, state, outputPath);
            if (WaterSceneStateHasPayload(preparedState)) {
                projectJson["water_scene_states"].push_back(
                    SerializeWaterSceneState(preparedState));
            }
        }
    } else {
        auto fallbackSceneState = MakeDefaultWaterSceneStateFromProject(document);
        fallbackSceneState.sceneGroupName = activeWaterSceneGroupName;
        fallbackSceneState = PrepareWaterSceneStateForSave(document, fallbackSceneState, outputPath);
        if (WaterSceneStateHasPayload(fallbackSceneState)) {
            projectJson["water_scene_states"].push_back(
                SerializeWaterSceneState(fallbackSceneState));
        }
    }
    for (const auto& profile : document.waterAnimationTrailProfiles) {
        projectJson["water_animation_trail_profiles"].push_back(SerializeWaterAnimationTrailProfile(profile));
    }
    for (const auto& profile : document.waterPathProfiles) {
        projectJson["water_path_profiles"].push_back(SerializeWaterPathProfile(profile));
    }
    for (const auto& profile : document.waterLaneProfiles) {
        projectJson["water_lane_profiles"].push_back(SerializeWaterLaneProfile(profile));
    }
    for (const auto& profile : document.waterTrailProfiles) {
        projectJson["water_trail_profiles"].push_back(SerializeWaterTrailProfile(profile));
    }
    for (const auto& node : document.waterSeepageNodes) {
        projectJson["water_seepage_nodes"].push_back(SerializeWaterSeepageNode(node));
    }
    for (const auto& profile : document.waterSeepageLookProfiles) {
        projectJson["water_seepage_look_profiles"].push_back(
            SerializeWaterSeepageLookProfile(profile));
    }
    for (const auto& scenario : document.waterScenarios) {
        projectJson["water_scenarios"].push_back(
            SerializeWaterScenarioDefinition(scenario));
    }
    for (const auto& run : document.waterTimingRuns) {
        projectJson["water_timing_runs"].push_back(SerializeWaterTimingRun(run));
    }
    if (document.tempWaterScenario.has_value()) {
        projectJson["temp_water_scenario"] =
            SerializeWaterScenarioDefinition(document.tempWaterScenario.value());
    }
    for (const auto& visual : document.waterPointVisuals) {
        projectJson["water_point_visuals"].push_back(SerializePointCloudVisual(visual));
    }
    if (document.tempWaterSourceSettings.has_value()) {
        projectJson["temp_water_source_settings"] =
            SerializeWaterSourceSettings(document.tempWaterSourceSettings.value());
    }
    if (document.tempWaterAnimationTrailSettings.has_value()) {
        projectJson["temp_water_animation_trail_settings"] =
            SerializeWaterAnimationTrailSettings(document.tempWaterAnimationTrailSettings.value());
    }
    if (document.tempWaterPathProfileSettings.has_value()) {
        projectJson["temp_water_path_profile_settings"] =
            SerializeWaterPathGenerationSettings(document.tempWaterPathProfileSettings.value());
    }
    if (document.tempWaterLaneProfileSettings.has_value()) {
        projectJson["temp_water_lane_profile_settings"] =
            SerializeWaterFlowTrailSettings(document.tempWaterLaneProfileSettings.value());
    }
    if (document.tempWaterTrailProfile.has_value()) {
        projectJson["temp_water_trail_profile"] =
            SerializeWaterTrailProfile(document.tempWaterTrailProfile.value());
    }
    if (document.tempWaterCausticLookSettings.has_value()) {
        projectJson["temp_water_caustic_look_settings"] =
            SerializeWaterCausticLookSettings(document.tempWaterCausticLookSettings.value());
    }
    if (document.cameraState.has_value()) {
        projectJson["camera"] = SerializeCameraState(document.cameraState.value());
    }

    projectJson["layers"] = json::array();
    for (const auto& layer : document.layers) {
        projectJson["layers"].push_back(SerializeProjectLayer(layer));
    }

    projectJson["camera_shots"] = json::array();
    for (const auto& shot : document.cameraShots) {
        projectJson["camera_shots"].push_back(SerializeCameraShot(shot));
    }
    projectJson["camera_path"] = json{
        {"shot_indices", document.cameraPathShotIndices},
        {"duration_frames", document.cameraPathDurationFrames},
    };
    projectJson["saved_animations"] = json::array();
    for (const auto& animation : document.savedAnimations) {
        projectJson["saved_animations"].push_back(SerializeSavedAnimation(animation));
    }
    return WriteJsonDocument(document, projectJson, outputPath, errorMessage);
}

std::optional<ProjectDocument> LoadProjectDocument(
    const std::filesystem::path& inputPath,
    std::string* errorMessage) {
    const auto projectJson = ReadJsonDocument(inputPath, errorMessage);
    if (!projectJson.has_value()) {
        return std::nullopt;
    }

    ProjectDocument document;
    document.schemaVersion = projectJson->value("schema_version", 1U);
    const bool defaultManualSurfaceGuide =
        document.schemaVersion >= kManualFlowSurfaceGuideProjectSchemaVersion;
    document.projectName = projectJson->value("project_name", std::string{"Invisible Places"});
    document.activeWaterSceneGroupName = TrimAsciiWhitespace(
        projectJson->value("active_water_scene_group", std::string{}));
    document.selectedLayerPath = projectJson->value("selected_layer_path", std::string{});
    document.lastAnimationPath = projectJson->value("last_animation_path", std::string{});
    document.backgroundColor =
        projectJson->value("background_color", std::array<float, 4>{0.0F, 0.0F, 0.0F, 1.0F});
    document.eyeDomeLightingEnabled = projectJson->value("eye_dome_lighting_enabled", false);
    document.proResAlphaPreviewEnabled = projectJson->value("pro_res_alpha_preview_enabled", false);
    document.eyeDomeLightingThickness = projectJson->value("eye_dome_lighting_thickness", 1.0F);
    document.constantUpdateView = projectJson->value("constant_update_view", false);
    document.liveVisualEffects = projectJson->value("live_visual_effects", false);
    document.sidePanelPinned = projectJson->value("side_panel_pinned", false);
    document.showLidarTab = projectJson->value("show_lidar_tab", false);
    document.showGsplatTab = projectJson->value("show_gsplat_tab", false);
    document.autoLowerGsplatQualityWhileNavigating =
        projectJson->value("auto_lower_gsplat_quality_while_navigating", true);
    if (projectJson->contains("point_visuals") && projectJson->at("point_visuals").is_array()) {
        for (const auto& visualJson : projectJson->at("point_visuals")) {
            auto visual = ParsePointCloudVisual(visualJson);
            visual.name = NormalizeProjectPointVisualName(visual.name);
            UpsertPointVisualDocument(&document.pointVisuals, visual.name, visual.style, true);
        }
    }
    document.selectedPointVisualName =
        NormalizeProjectPointVisualName(projectJson->value("selected_point_visual", document.selectedPointVisualName));
    if (projectJson->contains("scene_visual_states") && projectJson->at("scene_visual_states").is_array()) {
        for (const auto& stateJson : projectJson->at("scene_visual_states")) {
            auto state = ParseScenePointVisualState(stateJson);
            state.sceneGroupName = TrimAsciiWhitespace(state.sceneGroupName);
            state.visual.name = NormalizeProjectPointVisualName(state.visual.name);
            if (!state.sceneGroupName.empty()) {
                UpsertSceneVisualStateDocument(
                    &document.sceneVisualStates,
                    state.sceneGroupName,
                    state.visual.name,
                    state.visual.style);
            }
        }
    }
    if (document.schemaVersion >= 33U &&
        projectJson->contains("scene_point_cloud_groups") &&
        projectJson->at("scene_point_cloud_groups").is_array()) {
        for (const auto& groupJson : projectJson->at("scene_point_cloud_groups")) {
            document.scenePointCloudGroups.push_back(ParseScenePointCloudGroup(groupJson));
        }
    }
    if (projectJson->contains("gsplat_visual_style")) {
        document.gsplatVisualStyle = ParseGaussianSplatStyle(projectJson->at("gsplat_visual_style"));
    }
    if (projectJson->contains("point_cloud_preview_lod_mode")) {
        document.pointCloudPreviewLodMode =
            ParsePointCloudPreviewLodMode(projectJson->at("point_cloud_preview_lod_mode"));
    }
    document.interactivePointCap = projectJson->value("interactive_point_cap", 10'000'000ULL);
    if (projectJson->contains("point_cloud_renderer_mode")) {
        document.pointCloudRendererMode =
            ParsePointCloudRendererMode(projectJson->at("point_cloud_renderer_mode"));
    }
    if (projectJson->contains("render_job")) {
        document.renderJobSettings = ParseRenderJobSettings(projectJson->at("render_job"));
    }
    if (projectJson->contains("export_presets") && projectJson->at("export_presets").is_array()) {
        for (const auto& presetJson : projectJson->at("export_presets")) {
            document.exportPresets.push_back(ParseExportPreset(presetJson));
        }
    }
    document.selectedExportPresetName =
        projectJson->value("selected_export_preset", document.selectedExportPresetName);
    if (projectJson->contains("temp_export_preset")) {
        document.tempExportPreset = ParseExportPreset(projectJson->at("temp_export_preset"));
    }
    if (projectJson->contains("water_source_settings")) {
        document.waterSourceSettings = ParseWaterSourceSettings(projectJson->at("water_source_settings"));
    }
    if (projectJson->contains("temp_water_source_settings")) {
        document.tempWaterSourceSettings = ParseWaterSourceSettings(projectJson->at("temp_water_source_settings"));
    }
    if (projectJson->contains("water_animation_trail_settings")) {
        document.waterAnimationTrailSettings =
            ParseWaterAnimationTrailSettings(projectJson->at("water_animation_trail_settings"));
    }
    const bool hasWaterTrailGeometry = projectJson->contains("water_trail_geometry");
    if (hasWaterTrailGeometry) {
        document.waterTrailGeometry =
            ParseWaterTrailGeometrySettings(projectJson->at("water_trail_geometry"));
    }
    if (projectJson->contains("water_caustic_look_settings")) {
        document.waterCausticLookSettings =
            ParseWaterCausticLookSettings(projectJson->at("water_caustic_look_settings"));
    }
    if (projectJson->contains("water_seepage_default_look")) {
        document.waterSeepageDefaultLook =
            ParseWaterSeepageLookSettings(projectJson->at("water_seepage_default_look"));
    }
    if (projectJson->contains("water_seepage_look_profiles") &&
        projectJson->at("water_seepage_look_profiles").is_array()) {
        for (const auto& profileJson : projectJson->at("water_seepage_look_profiles")) {
            document.waterSeepageLookProfiles.push_back(
                ParseWaterSeepageLookProfile(profileJson));
        }
    }
    if (projectJson->contains("water_seepage_nodes") &&
        projectJson->at("water_seepage_nodes").is_array()) {
        for (const auto& nodeJson : projectJson->at("water_seepage_nodes")) {
            document.waterSeepageNodes.push_back(ParseWaterSeepageNode(nodeJson));
        }
    }
    if (projectJson->contains("water_scenarios") &&
        projectJson->at("water_scenarios").is_array()) {
        for (const auto& scenarioJson : projectJson->at("water_scenarios")) {
            document.waterScenarios.push_back(ParseWaterScenarioDefinition(scenarioJson));
        }
    } else {
        document.waterScenarios = invisible_places::water::DefaultWaterScenarioDefinitions();
    }
    document.selectedWaterScenarioId =
        projectJson->value("selected_water_scenario", std::string{});
    if (projectJson->contains("water_timing_runs") &&
        projectJson->at("water_timing_runs").is_array()) {
        for (const auto& runJson : projectJson->at("water_timing_runs")) {
            document.waterTimingRuns.push_back(ParseWaterTimingRun(runJson));
        }
    }
    document.waterTimingRunSequence = projectJson->value(
        "water_timing_run_sequence",
        document.waterTimingRunSequence);
    if (projectJson->contains("temp_water_scenario")) {
        document.tempWaterScenario =
            ParseWaterScenarioDefinition(projectJson->at("temp_water_scenario"));
    }
    document.waterShowFlowTrails = projectJson->value(
        "water_show_flow_trails",
        document.waterShowFlowTrails);
    if (projectJson->contains("water_flow_trail_settings")) {
        document.waterFlowTrailSettings =
            ParseWaterFlowTrailSettings(projectJson->at("water_flow_trail_settings"));
    } else if (projectJson->contains("water_flow_stream_settings")) {
        document.waterFlowTrailSettings =
            ParseWaterFlowTrailSettings(projectJson->at("water_flow_stream_settings"));
    }
    if (!hasWaterTrailGeometry) {
        document.waterTrailGeometry =
            invisible_places::water::WaterTrailGeometryFromFlowTrailSettings(document.waterFlowTrailSettings);
    }
    if (projectJson->contains("water_field_settings")) {
        document.waterFieldSettings = ParseWaterFieldSettings(projectJson->at("water_field_settings"));
    }
    if (projectJson->contains("water_field_trail_settings")) {
        document.waterFieldTrailSettings =
            ParseWaterFieldTrailSettings(projectJson->at("water_field_trail_settings"));
    } else if (projectJson->contains("water_field_stream_settings")) {
        document.waterFieldTrailSettings =
            ParseWaterFieldTrailSettings(projectJson->at("water_field_stream_settings"));
    }
    if (projectJson->contains("water_dynamic_mesh_flow_settings")) {
        document.waterDynamicMeshFlowSettings =
            ParseWaterDynamicMeshFlowSettings(projectJson->at("water_dynamic_mesh_flow_settings"));
        if (document.schemaVersion < 45U) {
            document.waterDynamicMeshFlowSettings =
                MigrateLegacyAutomaticMeshFlowDefaults(
                    std::move(document.waterDynamicMeshFlowSettings));
        }
    }
    if (projectJson->contains("water_rain_settings")) {
        document.waterRainSettings = ParseWaterRainSettings(projectJson->at("water_rain_settings"));
        document.waterRainVisualSettings =
            ParseWaterRainVisualSettings(projectJson->at("water_rain_settings"));
    }
    if (projectJson->contains("temp_water_animation_trail_settings")) {
        document.tempWaterAnimationTrailSettings =
            ParseWaterAnimationTrailSettings(projectJson->at("temp_water_animation_trail_settings"));
    }
    if (projectJson->contains("water_animation_trail_profiles") &&
        projectJson->at("water_animation_trail_profiles").is_array()) {
        for (const auto& profileJson : projectJson->at("water_animation_trail_profiles")) {
            document.waterAnimationTrailProfiles.push_back(ParseWaterAnimationTrailProfile(profileJson));
        }
    }
    if (projectJson->contains("water_path_profiles") &&
        projectJson->at("water_path_profiles").is_array()) {
        for (const auto& profileJson : projectJson->at("water_path_profiles")) {
            document.waterPathProfiles.push_back(ParseWaterPathProfile(profileJson));
        }
    }
    if (projectJson->contains("water_lane_profiles") &&
        projectJson->at("water_lane_profiles").is_array()) {
        for (const auto& profileJson : projectJson->at("water_lane_profiles")) {
            document.waterLaneProfiles.push_back(ParseWaterLaneProfile(profileJson));
        }
    }
    if (projectJson->contains("water_trail_profiles") &&
        projectJson->at("water_trail_profiles").is_array()) {
        for (const auto& profileJson : projectJson->at("water_trail_profiles")) {
            document.waterTrailProfiles.push_back(ParseWaterTrailProfile(profileJson));
        }
    }
    document.selectedWaterPathProfileName =
        projectJson->value("selected_water_path_profile", document.selectedWaterPathProfileName);
    document.selectedWaterLaneProfileName =
        projectJson->value("selected_water_lane_profile", document.selectedWaterLaneProfileName);
    document.selectedWaterTrailProfileName =
        projectJson->value("selected_water_trail_profile", document.selectedWaterTrailProfileName);
    if (projectJson->contains("temp_water_path_profile_settings")) {
        document.tempWaterPathProfileSettings =
            ParseWaterPathGenerationSettings(projectJson->at("temp_water_path_profile_settings"));
    }
    if (projectJson->contains("temp_water_lane_profile_settings")) {
        document.tempWaterLaneProfileSettings =
            ParseWaterFlowTrailSettings(projectJson->at("temp_water_lane_profile_settings"));
    }
    if (projectJson->contains("temp_water_trail_profile")) {
        document.tempWaterTrailProfile = ParseWaterTrailProfile(projectJson->at("temp_water_trail_profile"));
    }
    if (projectJson->contains("temp_water_caustic_look_settings")) {
        document.tempWaterCausticLookSettings =
            ParseWaterCausticLookSettings(projectJson->at("temp_water_caustic_look_settings"));
    }
    if (projectJson->contains("water_point_visuals") && projectJson->at("water_point_visuals").is_array()) {
        for (const auto& visualJson : projectJson->at("water_point_visuals")) {
            document.waterPointVisuals.push_back(ParsePointCloudVisual(visualJson));
        }
    }
    document.selectedWaterPointVisualName =
        projectJson->value("selected_water_point_visual", document.selectedWaterPointVisualName);
    if (document.selectedWaterPointVisualName.empty()) {
        document.selectedWaterPointVisualName = "Water Flow_preset";
    }
    if (projectJson->contains("water_point_visual_style")) {
        document.waterPointVisualStyle = ParsePointCloudStyle(projectJson->at("water_point_visual_style"));
    }
    if (projectJson->contains("temp_water_point_visual_style")) {
        document.tempWaterPointVisualStyle = ParsePointCloudStyle(projectJson->at("temp_water_point_visual_style"));
    }
    const bool hasNativeRippleLayers = projectJson->contains("water_ripple_layers") &&
                                      projectJson->at("water_ripple_layers").is_array();
    if (hasNativeRippleLayers &&
        projectJson->at("water_ripple_layers").is_array()) {
        for (const auto& layerJson : projectJson->at("water_ripple_layers")) {
            document.waterRippleLayers.push_back(ParseWaterEffectLayer(layerJson));
        }
    }
    if (projectJson->contains("water_field_layers") &&
        projectJson->at("water_field_layers").is_array()) {
        for (const auto& layerJson : projectJson->at("water_field_layers")) {
            auto layer = ParseWaterEffectLayer(layerJson);
            if (!layerJson.contains("feature_type")) {
                layer.featureType = WaterEffectFeatureType::FieldSurfaceMotion;
            }
            document.waterFieldLayers.push_back(std::move(layer));
        }
    }
    if (projectJson->contains("water_caustic_regions") &&
        projectJson->at("water_caustic_regions").is_array()) {
        for (const auto& regionJson : projectJson->at("water_caustic_regions")) {
            auto region = ParseLegacyCausticRegion(regionJson);
            if (!hasNativeRippleLayers) {
                WaterEffectLayer layer;
                layer.id = region.id;
                layer.name = region.name.empty() ? "Caustic Lace" : region.name;
                layer.featureType = WaterEffectFeatureType::Ripple;
                layer.rippleOverlayType = WaterRippleOverlayType::CausticLace;
                layer.targetLayerSourcePath = region.targetLayerSourcePath;
                layer.vertices = region.vertices;
                layer.hull = region.hull;
                layer.enabledInViewport = region.enabled;
                layer.enabledInExport = region.enabled;
                layer.edgeBlendWidth = region.edgeBlendWidth;
                layer.response.intensity = document.waterCausticLookSettings.intensity;
                layer.response.emissionAdd = document.waterCausticLookSettings.emissionBoost;
                layer.response.opacityAdd = document.waterCausticLookSettings.opacityBoost;
                layer.response.pointSizeAdd = document.waterCausticLookSettings.pointSizeBoost;
                layer.response.colouriseRed = document.waterCausticLookSettings.tintRed;
                layer.response.colouriseGreen = document.waterCausticLookSettings.tintGreen;
                layer.response.colouriseBlue = document.waterCausticLookSettings.tintBlue;
                layer.speed = document.waterCausticLookSettings.speed;
                layer.wavelengthMeters = document.waterCausticLookSettings.cellSizeMeters;
                layer.warp = document.waterCausticLookSettings.warp;
                for (const auto type : invisible_places::water::AllWaterRippleOverlayTypes()) {
                    layer.overlayPatternSettings[invisible_places::water::WaterRippleOverlayTypeIndex(type)] =
                        invisible_places::water::DefaultWaterRipplePatternSettings(type);
                }
                invisible_places::water::StoreActiveWaterRipplePatternSettings(&layer);
                document.waterRippleLayers.push_back(std::move(layer));
            }
        }
    }
    if (projectJson->contains("water_path_cache")) {
        document.waterPathCache = ParseWaterPathCache(projectJson->at("water_path_cache"));
    }
    if (projectJson->contains("water_ripple_runtime_caches") &&
        projectJson->at("water_ripple_runtime_caches").is_array()) {
        for (const auto& cacheJson : projectJson->at("water_ripple_runtime_caches")) {
            auto cache = ParseWaterRippleRuntimeCache(cacheJson);
            if (!cache.memberships.empty() && !cache.params.empty()) {
                document.waterRippleRuntimeCaches.push_back(std::move(cache));
            }
        }
    }
    if (projectJson->contains("water_scene_states") && projectJson->at("water_scene_states").is_array()) {
        for (const auto& stateJson : projectJson->at("water_scene_states")) {
            auto state = ParseWaterSceneState(stateJson, defaultManualSurfaceGuide);
            if (WaterSceneStateHasPayload(state)) {
                document.waterSceneStates.push_back(std::move(state));
            }
        }
    }
    LoadWaterPathSidecars(inputPath, &document);
    if (projectJson->contains("water_visual_settings")) {
        document.waterVisualSettings = ParseWaterVisualSettings(projectJson->at("water_visual_settings"));
        if (!projectJson->contains("water_animation_trail_settings")) {
            document.waterAnimationTrailSettings.colorVariation = document.waterVisualSettings.colorVariation;
        }
        if (!projectJson->contains("water_point_visual_style")) {
            document.waterPointVisualStyle = MakeLegacyWaterPointVisualStyle(document.waterVisualSettings);
        }
    }
    if (projectJson->contains("temp_water_visual_settings")) {
        document.tempWaterVisualSettings = ParseWaterVisualSettings(projectJson->at("temp_water_visual_settings"));
        if (!document.tempWaterAnimationTrailSettings.has_value()) {
            auto trailSettings = document.waterAnimationTrailSettings;
            trailSettings.colorVariation = document.tempWaterVisualSettings->colorVariation;
            document.tempWaterAnimationTrailSettings = trailSettings;
        }
        if (!document.tempWaterPointVisualStyle.has_value()) {
            document.tempWaterPointVisualStyle =
                MakeLegacyWaterPointVisualStyle(document.tempWaterVisualSettings.value());
        }
    }
    if (projectJson->contains("water_settings")) {
        document.waterSettings = ParseWaterSettingsBundle(projectJson->at("water_settings"));
        if (!projectJson->contains("water_source_settings")) {
            document.waterSourceSettings.path = document.waterSettings.path;
            document.waterSourceSettings.trailShape.particleJitter = document.waterSettings.trail.particleJitter;
            document.waterSourceSettings.trailShape.splineAnchorSpacing =
                document.waterSettings.trail.splineAnchorSpacing;
        }
        if (!projectJson->contains("water_visual_settings")) {
            document.waterVisualSettings = document.waterSettings.visual;
        }
        if (!projectJson->contains("water_animation_trail_settings")) {
            document.waterAnimationTrailSettings.particleDensity = document.waterSettings.trail.particleDensity;
            document.waterAnimationTrailSettings.particleSpeed = document.waterSettings.trail.particleSpeed;
            document.waterAnimationTrailSettings.colorVariation = document.waterSettings.visual.colorVariation;
        }
        if (!projectJson->contains("water_point_visual_style")) {
            document.waterPointVisualStyle = MakeLegacyWaterPointVisualStyle(document.waterSettings.visual);
        }
    } else {
        if (projectJson->contains("water_bake_settings")) {
            document.waterSettings.path = ParseWaterBakeSettings(projectJson->at("water_bake_settings"));
            if (!projectJson->contains("water_source_settings")) {
                document.waterSourceSettings.path = document.waterSettings.path;
            }
        }
        if (projectJson->contains("water_render_settings")) {
            const auto legacyRender = ParseWaterRenderSettings(projectJson->at("water_render_settings"));
            document.waterSettings.trail = legacyRender.trail;
            document.waterSettings.visual = legacyRender.visual;
            if (!projectJson->contains("water_source_settings")) {
                document.waterSourceSettings.trailShape.particleJitter = legacyRender.trail.particleJitter;
                document.waterSourceSettings.trailShape.splineAnchorSpacing =
                    legacyRender.trail.splineAnchorSpacing;
            }
            if (!projectJson->contains("water_visual_settings")) {
                document.waterVisualSettings = legacyRender.visual;
            }
            if (!projectJson->contains("water_animation_trail_settings")) {
                document.waterAnimationTrailSettings.particleDensity = legacyRender.trail.particleDensity;
                document.waterAnimationTrailSettings.particleSpeed = legacyRender.trail.particleSpeed;
                document.waterAnimationTrailSettings.colorVariation = legacyRender.visual.colorVariation;
            }
            if (!projectJson->contains("water_point_visual_style")) {
                document.waterPointVisualStyle = MakeLegacyWaterPointVisualStyle(legacyRender.visual);
            }
        }
    }
    if (projectJson->contains("temp_water_settings")) {
        document.tempWaterSettings = ParseWaterSettingsBundle(projectJson->at("temp_water_settings"));
        if (!document.tempWaterSourceSettings.has_value()) {
            WaterSourceSettings tempSource;
            tempSource.path = document.tempWaterSettings->path;
            tempSource.trailShape.particleJitter = document.tempWaterSettings->trail.particleJitter;
            tempSource.trailShape.splineAnchorSpacing = document.tempWaterSettings->trail.splineAnchorSpacing;
            document.tempWaterSourceSettings = tempSource;
        }
        if (!document.tempWaterVisualSettings.has_value()) {
            document.tempWaterVisualSettings = document.tempWaterSettings->visual;
        }
        if (!document.tempWaterAnimationTrailSettings.has_value()) {
            WaterAnimationTrailSettings tempTrail;
            tempTrail.particleDensity = document.tempWaterSettings->trail.particleDensity;
            tempTrail.particleSpeed = document.tempWaterSettings->trail.particleSpeed;
            tempTrail.colorVariation = document.tempWaterSettings->visual.colorVariation;
            document.tempWaterAnimationTrailSettings = tempTrail;
        }
        if (!document.tempWaterPointVisualStyle.has_value()) {
            document.tempWaterPointVisualStyle =
                MakeLegacyWaterPointVisualStyle(document.tempWaterSettings->visual);
        }
    }
    if (!projectJson->contains("water_point_visual_style") &&
        !projectJson->contains("water_visual_settings") &&
        !projectJson->contains("water_settings") &&
        !projectJson->contains("water_render_settings")) {
        document.waterPointVisualStyle = MakeLegacyWaterPointVisualStyle(document.waterVisualSettings);
    }
    document.waterSettings.path = document.waterSourceSettings.path;
    document.waterSettings.trail.particleJitter = document.waterSourceSettings.trailShape.particleJitter;
    document.waterSettings.trail.splineAnchorSpacing = document.waterSourceSettings.trailShape.splineAnchorSpacing;
    document.waterSettings.trail.particleDensity = document.waterAnimationTrailSettings.particleDensity;
    document.waterSettings.trail.particleSpeed = document.waterAnimationTrailSettings.particleSpeed;
    document.waterSettings.visual = document.waterVisualSettings;
    document.waterSettings.visual.colorVariation = document.waterAnimationTrailSettings.colorVariation;
    document.waterBakeSettings = document.waterSourceSettings.path;
    document.waterRenderSettings = document.waterSettings;
    if (projectJson->contains("camera")) {
        document.cameraState = ParseCameraState(projectJson->at("camera"));
    }

    if (projectJson->contains("layers")) {
        for (const auto& layerJson : projectJson->at("layers")) {
            document.layers.push_back(ParseProjectLayer(layerJson));
        }
    }
    if (document.schemaVersion < 33U) {
        MigrateLegacyScenePointCloudGroups(&document);
    }
    if (document.schemaVersion < 30U || document.pointVisuals.empty()) {
        MigrateLegacyLayerPointVisuals(&document);
    } else if (
        !document.selectedPointVisualName.empty() &&
        !FindPointVisualDocumentIndex(document.pointVisuals, document.selectedPointVisualName).has_value()) {
        document.selectedPointVisualName = NormalizeProjectPointVisualName(document.pointVisuals.front().name);
    }
    PruneSceneVisualStatesToKnownSceneGroups(&document);

    if (projectJson->contains("camera_shots")) {
        for (const auto& shotJson : projectJson->at("camera_shots")) {
            document.cameraShots.push_back(ParseCameraShot(shotJson));
        }
    }
    EnsureCameraShotIds(&document.cameraShots);
    if (projectJson->contains("camera_path")) {
        const auto& cameraPathJson = projectJson->at("camera_path");
        document.cameraPathDurationFrames =
            cameraPathJson.value("duration_frames", document.cameraPathDurationFrames);
        if (cameraPathJson.contains("shot_indices")) {
            document.cameraPathShotIndices =
                cameraPathJson.at("shot_indices").get<std::vector<std::size_t>>();
        }
    }
    document.hasSavedAnimationRegistry = projectJson->contains("saved_animations");
    if (document.hasSavedAnimationRegistry && projectJson->at("saved_animations").is_array()) {
        for (const auto& animationJson : projectJson->at("saved_animations")) {
            document.savedAnimations.push_back(ParseSavedAnimation(animationJson));
        }
    }
    if (projectJson->contains("water_emitters") && projectJson->at("water_emitters").is_array()) {
        for (const auto& emitterJson : projectJson->at("water_emitters")) {
            document.waterEmitters.push_back(ParseWaterEmitter(emitterJson));
        }
    }
    if (document.waterSceneStates.empty() &&
        projectJson->contains("water_manual_flow_paths") &&
        projectJson->at("water_manual_flow_paths").is_array()) {
        for (const auto& sourceJson : projectJson->at("water_manual_flow_paths")) {
            document.waterManualFlowPaths.push_back(
                ParseWaterManualFlowPath(sourceJson, defaultManualSurfaceGuide));
        }
    }
    if (document.waterSceneStates.empty() && document.waterPathCache.has_value()) {
        document.waterPathCache = PruneSettledWaterPathCache(
            document.waterPathCache.value(),
            document.waterEmitters);
    }
    if (!document.waterSceneStates.empty()) {
        ApplyActiveWaterSceneState(&document);
    }
    if (document.schemaVersion < kProjectDocumentSchemaVersion) {
        document.schemaVersion = kProjectDocumentSchemaVersion;
    }

    return document;
}

bool SaveWaterSourcesDocument(
    const WaterSourcesDocument& document,
    const std::filesystem::path& outputPath,
    std::string* errorMessage) {
    json sourcesJson{
        {"schema_version", kWaterSourcesDocumentSchemaVersion},
        {"water_source_settings", SerializeWaterSourceSettings(document.sourceSettings)},
        {"water_caustic_look_settings", SerializeWaterCausticLookSettings(document.causticLookSettings)},
        {"water_flow_trail_settings", SerializeWaterFlowTrailSettings(document.flowTrailSettings)},
        {"show_flow_trails", document.showFlowTrails},
        {"water_trail_geometry", SerializeWaterTrailGeometrySettings(document.trailGeometry)},
        {"water_path_profiles", json::array()},
        {"water_lane_profiles", json::array()},
        {"water_trail_profiles", json::array()},
        {"selected_water_path_profile", document.selectedPathProfileName},
        {"selected_water_lane_profile", document.selectedLaneProfileName},
        {"selected_water_trail_profile", document.selectedTrailProfileName},
        {"water_field_settings", SerializeWaterFieldSettings(document.fieldSettings)},
        {"water_field_trail_settings", SerializeWaterFieldTrailSettings(document.fieldTrailSettings)},
        {"water_dynamic_mesh_flow_settings", SerializeWaterDynamicMeshFlowSettings(document.dynamicMeshFlowSettings)},
        {"water_rain_settings", SerializeWaterRainSettings(document.rainSettings, document.rainVisualSettings)},
        {"water_emitters", json::array()},
        {"water_manual_flow_paths", json::array()},
        {"water_seepage_nodes", json::array()},
        {"water_seepage_default_look", SerializeWaterSeepageLookSettings(document.seepageDefaultLook)},
        {"water_seepage_look_profiles", json::array()},
        {"water_ripple_layers", json::array()},
        {"water_field_layers", json::array()},
        {"water_ripple_runtime_caches", json::array()},
    };
    if (document.tempSourceSettings.has_value()) {
        sourcesJson["temp_water_source_settings"] =
            SerializeWaterSourceSettings(document.tempSourceSettings.value());
    }
    if (document.tempCausticLookSettings.has_value()) {
        sourcesJson["temp_water_caustic_look_settings"] =
            SerializeWaterCausticLookSettings(document.tempCausticLookSettings.value());
    }
    if (document.tempPathProfileSettings.has_value()) {
        sourcesJson["temp_water_path_profile_settings"] =
            SerializeWaterPathGenerationSettings(document.tempPathProfileSettings.value());
    }
    if (document.tempLaneProfileSettings.has_value()) {
        sourcesJson["temp_water_lane_profile_settings"] =
            SerializeWaterFlowTrailSettings(document.tempLaneProfileSettings.value());
    }
    if (document.tempTrailProfile.has_value()) {
        sourcesJson["temp_water_trail_profile"] =
            SerializeWaterTrailProfile(document.tempTrailProfile.value());
    }
    for (const auto& profile : document.pathProfiles) {
        sourcesJson["water_path_profiles"].push_back(SerializeWaterPathProfile(profile));
    }
    for (const auto& profile : document.laneProfiles) {
        sourcesJson["water_lane_profiles"].push_back(SerializeWaterLaneProfile(profile));
    }
    for (const auto& profile : document.trailProfiles) {
        sourcesJson["water_trail_profiles"].push_back(SerializeWaterTrailProfile(profile));
    }
    for (const auto& emitter : document.emitters) {
        sourcesJson["water_emitters"].push_back(SerializeWaterEmitter(emitter));
    }
    for (const auto& source : document.manualFlowPaths) {
        sourcesJson["water_manual_flow_paths"].push_back(SerializeWaterManualFlowPath(source));
    }
    for (const auto& node : document.seepageNodes) {
        sourcesJson["water_seepage_nodes"].push_back(SerializeWaterSeepageNode(node));
    }
    for (const auto& profile : document.seepageLookProfiles) {
        sourcesJson["water_seepage_look_profiles"].push_back(
            SerializeWaterSeepageLookProfile(profile));
    }
    for (const auto& layer : document.rippleLayers) {
        sourcesJson["water_ripple_layers"].push_back(SerializeWaterEffectLayer(layer));
    }
    for (const auto& layer : document.fieldLayers) {
        sourcesJson["water_field_layers"].push_back(SerializeWaterEffectLayer(layer));
    }
    for (const auto& cache : document.rippleRuntimeCaches) {
        if (ShouldSerializeWaterRippleRuntimeCache(cache)) {
            sourcesJson["water_ripple_runtime_caches"].push_back(SerializeWaterRippleRuntimeCache(cache));
        }
    }
    if (document.pathCache.has_value()) {
        const auto pruned = PruneSettledWaterPathCache(
            document.pathCache.value(),
            document.emitters);
        if (pruned.has_value()) {
            sourcesJson["water_path_cache"] = SerializeWaterPathCache(pruned.value());
        }
    }
    return WriteJsonDocument(document, sourcesJson, outputPath, errorMessage);
}

std::optional<WaterSourcesDocument> LoadWaterSourcesDocument(
    const std::filesystem::path& inputPath,
    std::string* errorMessage) {
    const auto sourcesJson = ReadJsonDocument(inputPath, errorMessage);
    if (!sourcesJson.has_value()) {
        return std::nullopt;
    }

    WaterSourcesDocument document;
    document.schemaVersion = sourcesJson->value("schema_version", 1U);
    const bool defaultManualSurfaceGuide =
        document.schemaVersion >= kManualFlowSurfaceGuideSourcesSchemaVersion;
    if (sourcesJson->contains("water_source_settings")) {
        document.sourceSettings = ParseWaterSourceSettings(sourcesJson->at("water_source_settings"));
    }
    if (sourcesJson->contains("temp_water_source_settings")) {
        document.tempSourceSettings = ParseWaterSourceSettings(sourcesJson->at("temp_water_source_settings"));
    }
    if (sourcesJson->contains("water_caustic_look_settings")) {
        document.causticLookSettings =
            ParseWaterCausticLookSettings(sourcesJson->at("water_caustic_look_settings"));
    }
    if (sourcesJson->contains("water_seepage_default_look")) {
        document.seepageDefaultLook =
            ParseWaterSeepageLookSettings(sourcesJson->at("water_seepage_default_look"));
    }
    if (sourcesJson->contains("water_seepage_look_profiles") &&
        sourcesJson->at("water_seepage_look_profiles").is_array()) {
        for (const auto& profileJson : sourcesJson->at("water_seepage_look_profiles")) {
            document.seepageLookProfiles.push_back(ParseWaterSeepageLookProfile(profileJson));
        }
    }
    if (sourcesJson->contains("water_flow_trail_settings")) {
        document.flowTrailSettings =
            ParseWaterFlowTrailSettings(sourcesJson->at("water_flow_trail_settings"));
    } else if (sourcesJson->contains("water_flow_stream_settings")) {
        document.flowTrailSettings =
            ParseWaterFlowTrailSettings(sourcesJson->at("water_flow_stream_settings"));
    }
    document.showFlowTrails = sourcesJson->value(
        "show_flow_trails",
        document.showFlowTrails);
    if (sourcesJson->contains("water_trail_geometry")) {
        document.trailGeometry =
            ParseWaterTrailGeometrySettings(sourcesJson->at("water_trail_geometry"));
    } else {
        document.trailGeometry =
            invisible_places::water::WaterTrailGeometryFromFlowTrailSettings(document.flowTrailSettings);
    }
    if (sourcesJson->contains("water_path_profiles") &&
        sourcesJson->at("water_path_profiles").is_array()) {
        for (const auto& profileJson : sourcesJson->at("water_path_profiles")) {
            document.pathProfiles.push_back(ParseWaterPathProfile(profileJson));
        }
    }
    if (sourcesJson->contains("water_lane_profiles") &&
        sourcesJson->at("water_lane_profiles").is_array()) {
        for (const auto& profileJson : sourcesJson->at("water_lane_profiles")) {
            document.laneProfiles.push_back(ParseWaterLaneProfile(profileJson));
        }
    }
    if (sourcesJson->contains("water_trail_profiles") &&
        sourcesJson->at("water_trail_profiles").is_array()) {
        for (const auto& profileJson : sourcesJson->at("water_trail_profiles")) {
            document.trailProfiles.push_back(ParseWaterTrailProfile(profileJson));
        }
    }
    document.selectedPathProfileName =
        sourcesJson->value("selected_water_path_profile", document.selectedPathProfileName);
    document.selectedLaneProfileName =
        sourcesJson->value("selected_water_lane_profile", document.selectedLaneProfileName);
    document.selectedTrailProfileName =
        sourcesJson->value("selected_water_trail_profile", document.selectedTrailProfileName);
    if (sourcesJson->contains("temp_water_path_profile_settings")) {
        document.tempPathProfileSettings =
            ParseWaterPathGenerationSettings(sourcesJson->at("temp_water_path_profile_settings"));
    }
    if (sourcesJson->contains("temp_water_lane_profile_settings")) {
        document.tempLaneProfileSettings =
            ParseWaterFlowTrailSettings(sourcesJson->at("temp_water_lane_profile_settings"));
    }
    if (sourcesJson->contains("temp_water_trail_profile")) {
        document.tempTrailProfile = ParseWaterTrailProfile(sourcesJson->at("temp_water_trail_profile"));
    }
    if (sourcesJson->contains("water_field_settings")) {
        document.fieldSettings = ParseWaterFieldSettings(sourcesJson->at("water_field_settings"));
    }
    if (sourcesJson->contains("water_field_trail_settings")) {
        document.fieldTrailSettings =
            ParseWaterFieldTrailSettings(sourcesJson->at("water_field_trail_settings"));
    } else if (sourcesJson->contains("water_field_stream_settings")) {
        document.fieldTrailSettings =
            ParseWaterFieldTrailSettings(sourcesJson->at("water_field_stream_settings"));
    }
    if (sourcesJson->contains("water_rain_settings")) {
        document.rainSettings = ParseWaterRainSettings(sourcesJson->at("water_rain_settings"));
        document.rainVisualSettings = ParseWaterRainVisualSettings(sourcesJson->at("water_rain_settings"));
    }
    if (sourcesJson->contains("water_dynamic_mesh_flow_settings")) {
        document.dynamicMeshFlowSettings =
            ParseWaterDynamicMeshFlowSettings(sourcesJson->at("water_dynamic_mesh_flow_settings"));
        if (document.schemaVersion < 19U) {
            document.dynamicMeshFlowSettings =
                MigrateLegacyAutomaticMeshFlowDefaults(
                    std::move(document.dynamicMeshFlowSettings));
        }
    }
    if (sourcesJson->contains("temp_water_caustic_look_settings")) {
        document.tempCausticLookSettings =
            ParseWaterCausticLookSettings(sourcesJson->at("temp_water_caustic_look_settings"));
    }
    if (sourcesJson->contains("water_settings")) {
        document.settings = ParseWaterSettingsBundle(sourcesJson->at("water_settings"));
        if (!sourcesJson->contains("water_source_settings")) {
            document.sourceSettings.path = document.settings.path;
            document.sourceSettings.trailShape.particleJitter = document.settings.trail.particleJitter;
            document.sourceSettings.trailShape.splineAnchorSpacing = document.settings.trail.splineAnchorSpacing;
        }
    } else {
        if (sourcesJson->contains("water_bake_settings")) {
            document.settings.path = ParseWaterBakeSettings(sourcesJson->at("water_bake_settings"));
            if (!sourcesJson->contains("water_source_settings")) {
                document.sourceSettings.path = document.settings.path;
            }
        }
        if (sourcesJson->contains("water_render_settings")) {
            const auto legacyRender = ParseWaterRenderSettings(sourcesJson->at("water_render_settings"));
            document.settings.trail = legacyRender.trail;
            document.settings.visual = legacyRender.visual;
            if (!sourcesJson->contains("water_source_settings")) {
                document.sourceSettings.trailShape.particleJitter = legacyRender.trail.particleJitter;
                document.sourceSettings.trailShape.splineAnchorSpacing =
                    legacyRender.trail.splineAnchorSpacing;
            }
        }
    }
    if (sourcesJson->contains("temp_water_settings")) {
        document.tempSettings = ParseWaterSettingsBundle(sourcesJson->at("temp_water_settings"));
        if (!document.tempSourceSettings.has_value()) {
            WaterSourceSettings tempSource;
            tempSource.path = document.tempSettings->path;
            tempSource.trailShape.particleJitter = document.tempSettings->trail.particleJitter;
            tempSource.trailShape.splineAnchorSpacing = document.tempSettings->trail.splineAnchorSpacing;
            document.tempSourceSettings = tempSource;
        }
    }
    document.settings.path = document.sourceSettings.path;
    document.settings.trail.particleJitter = document.sourceSettings.trailShape.particleJitter;
    document.settings.trail.splineAnchorSpacing = document.sourceSettings.trailShape.splineAnchorSpacing;
    document.bakeSettings = document.sourceSettings.path;
    document.renderSettings = document.settings;
    if (sourcesJson->contains("water_emitters") && sourcesJson->at("water_emitters").is_array()) {
        for (const auto& emitterJson : sourcesJson->at("water_emitters")) {
            document.emitters.push_back(ParseWaterEmitter(emitterJson));
        }
    }
    if (sourcesJson->contains("water_manual_flow_paths") &&
        sourcesJson->at("water_manual_flow_paths").is_array()) {
        for (const auto& sourceJson : sourcesJson->at("water_manual_flow_paths")) {
            document.manualFlowPaths.push_back(
                ParseWaterManualFlowPath(sourceJson, defaultManualSurfaceGuide));
        }
    }
    if (sourcesJson->contains("water_seepage_nodes") &&
        sourcesJson->at("water_seepage_nodes").is_array()) {
        for (const auto& nodeJson : sourcesJson->at("water_seepage_nodes")) {
            document.seepageNodes.push_back(ParseWaterSeepageNode(nodeJson));
        }
    }
    const bool hasNativeRippleLayers = sourcesJson->contains("water_ripple_layers") &&
                                      sourcesJson->at("water_ripple_layers").is_array();
    if (hasNativeRippleLayers) {
        for (const auto& layerJson : sourcesJson->at("water_ripple_layers")) {
            document.rippleLayers.push_back(ParseWaterEffectLayer(layerJson));
        }
    }
    if (sourcesJson->contains("water_field_layers") &&
        sourcesJson->at("water_field_layers").is_array()) {
        for (const auto& layerJson : sourcesJson->at("water_field_layers")) {
            auto layer = ParseWaterEffectLayer(layerJson);
            if (!layerJson.contains("feature_type")) {
                layer.featureType = WaterEffectFeatureType::FieldSurfaceMotion;
            }
            document.fieldLayers.push_back(std::move(layer));
        }
    }
    if (sourcesJson->contains("water_caustic_regions") &&
        sourcesJson->at("water_caustic_regions").is_array()) {
        for (const auto& regionJson : sourcesJson->at("water_caustic_regions")) {
            auto region = ParseLegacyCausticRegion(regionJson);
            if (!hasNativeRippleLayers) {
                WaterEffectLayer layer;
                layer.id = region.id;
                layer.name = region.name.empty() ? "Caustic Lace" : region.name;
                layer.featureType = WaterEffectFeatureType::Ripple;
                layer.rippleOverlayType = WaterRippleOverlayType::CausticLace;
                layer.targetLayerSourcePath = region.targetLayerSourcePath;
                layer.vertices = region.vertices;
                layer.hull = region.hull;
                layer.enabledInViewport = region.enabled;
                layer.enabledInExport = region.enabled;
                layer.edgeBlendWidth = region.edgeBlendWidth;
                layer.response.intensity = document.causticLookSettings.intensity;
                layer.response.emissionAdd = document.causticLookSettings.emissionBoost;
                layer.response.opacityAdd = document.causticLookSettings.opacityBoost;
                layer.response.pointSizeAdd = document.causticLookSettings.pointSizeBoost;
                layer.response.colouriseRed = document.causticLookSettings.tintRed;
                layer.response.colouriseGreen = document.causticLookSettings.tintGreen;
                layer.response.colouriseBlue = document.causticLookSettings.tintBlue;
                layer.speed = document.causticLookSettings.speed;
                layer.wavelengthMeters = document.causticLookSettings.cellSizeMeters;
                layer.warp = document.causticLookSettings.warp;
                for (const auto type : invisible_places::water::AllWaterRippleOverlayTypes()) {
                    layer.overlayPatternSettings[invisible_places::water::WaterRippleOverlayTypeIndex(type)] =
                        invisible_places::water::DefaultWaterRipplePatternSettings(type);
                }
                invisible_places::water::StoreActiveWaterRipplePatternSettings(&layer);
                document.rippleLayers.push_back(std::move(layer));
            }
        }
    }
    if (sourcesJson->contains("water_path_cache")) {
        document.pathCache = ParseWaterPathCache(sourcesJson->at("water_path_cache"));
    }
    if (sourcesJson->contains("water_ripple_runtime_caches") &&
        sourcesJson->at("water_ripple_runtime_caches").is_array()) {
        for (const auto& cacheJson : sourcesJson->at("water_ripple_runtime_caches")) {
            auto cache = ParseWaterRippleRuntimeCache(cacheJson);
            if (!cache.memberships.empty() && !cache.params.empty()) {
                document.rippleRuntimeCaches.push_back(std::move(cache));
            }
        }
    }
    return document;
}

bool SaveAnimationPath(
    const invisible_places::camera::AnimationPath& path,
    const std::filesystem::path& outputPath,
    std::string* errorMessage) {
    return WriteJsonDocument(path, SerializeAnimationPath(path), outputPath, errorMessage);
}

std::optional<invisible_places::camera::AnimationPath> LoadAnimationPath(
    const std::filesystem::path& inputPath,
    std::string* errorMessage) {
    const auto pathJson = ReadJsonDocument(inputPath, errorMessage);
    if (!pathJson.has_value()) {
        return std::nullopt;
    }

    try {
        return ParseAnimationPath(pathJson.value());
    } catch (const std::exception& error) {
        if (errorMessage != nullptr) {
            *errorMessage = "Failed to parse animation path: " + std::string{error.what()};
        }
        return std::nullopt;
    }
}

bool SavePointCloudStylePreset(
    const PointCloudStylePresetDocument& document,
    const std::filesystem::path& outputPath,
    std::string* errorMessage) {
    const json presetJson{
        {"schema_version", document.schemaVersion},
        {"preset_name", document.presetName},
        {"point_style", SerializePointCloudStyle(document.style)},
    };
    return WriteJsonDocument(document, presetJson, outputPath, errorMessage);
}

std::optional<PointCloudStylePresetDocument> LoadPointCloudStylePreset(
    const std::filesystem::path& inputPath,
    std::string* errorMessage) {
    const auto presetJson = ReadJsonDocument(inputPath, errorMessage);
    if (!presetJson.has_value()) {
        return std::nullopt;
    }

    PointCloudStylePresetDocument document;
    document.schemaVersion = presetJson->value("schema_version", 1U);
    document.presetName = presetJson->value("preset_name", std::string{"Point Cloud Style"});
    if (presetJson->contains("point_style")) {
        document.style = ParsePointCloudStyle(presetJson->at("point_style"));
    }
    return document;
}

bool SaveWaterPathCacheDocument(
    const invisible_places::water::WaterPathCache& document,
    const std::filesystem::path& outputPath,
    std::string* errorMessage,
    WaterPathCacheManifestDocument* manifest) {
    if (outputPath.extension() != ".flowpathcache") {
        return WriteJsonDocument(document, SerializeWaterPathCache(document), outputPath, errorMessage);
    }
    const auto encoded = EncodeWaterPathCache(document, errorMessage);
    return encoded.has_value() &&
           WriteEncodedWaterPathCache(
               document,
               encoded.value(),
               outputPath,
               errorMessage,
               manifest);
}

bool SaveContentAddressedWaterPathCacheDocument(
    const invisible_places::water::WaterPathCache& document,
    const std::filesystem::path& outputDirectory,
    std::string* errorMessage,
    WaterPathCacheManifestDocument* manifest,
    std::filesystem::path* outputPath) {
    const auto encoded = EncodeWaterPathCache(document, errorMessage);
    if (!encoded.has_value()) {
        return false;
    }
    const auto resolvedPath = outputDirectory /
                              (WaterPathCacheChecksumToken(encoded->checksum) +
                               ".flowpathcache");
    if (!WriteEncodedWaterPathCache(
            document,
            encoded.value(),
            resolvedPath,
            errorMessage,
            manifest)) {
        return false;
    }
    if (outputPath != nullptr) {
        *outputPath = resolvedPath;
    }
    return true;
}

std::optional<invisible_places::water::WaterPathCache> LoadWaterPathCacheDocument(
    const std::filesystem::path& inputPath,
    std::string* errorMessage,
    WaterPathCacheManifestDocument* manifest) {
    std::error_code inputSizeError;
    const auto inputBytes = std::filesystem::file_size(inputPath, inputSizeError);
    constexpr std::uint64_t maximumSidecarBytes =
        kMaximumPersistedWaterCacheBytes + kWaterPathCacheSidecarMagic.size() +
        sizeof(std::uint32_t) + sizeof(std::uint64_t) +
        4U * sizeof(std::uint64_t);
    if (!inputSizeError && inputBytes > maximumSidecarBytes) {
        if (errorMessage != nullptr) {
            *errorMessage = "Water path cache file exceeds the 5 GiB persistence ceiling.";
        }
        return std::nullopt;
    }
    std::ifstream binaryInput{inputPath, std::ios::binary};
    if (binaryInput.is_open()) {
        std::array<char, kWaterPathCacheSidecarMagic.size()> magic{};
        binaryInput.read(magic.data(), static_cast<std::streamsize>(magic.size()));
        if (binaryInput.gcount() == static_cast<std::streamsize>(magic.size()) &&
            magic == kWaterPathCacheSidecarMagic) {
            std::uint32_t schema = 0U;
            std::uint64_t payloadBytes = 0U;
            std::array<std::uint64_t, 4> expectedChecksum{};
            binaryInput.read(reinterpret_cast<char*>(&schema), sizeof(schema));
            binaryInput.read(reinterpret_cast<char*>(&payloadBytes), sizeof(payloadBytes));
            binaryInput.read(
                reinterpret_cast<char*>(expectedChecksum.data()),
                static_cast<std::streamsize>(expectedChecksum.size() * sizeof(expectedChecksum.front())));
            if (!binaryInput.good() || schema != kWaterPathCacheSidecarSchemaVersion ||
                payloadBytes > kMaximumPersistedWaterCacheBytes ||
                payloadBytes > static_cast<std::uint64_t>(std::numeric_limits<std::size_t>::max())) {
                if (errorMessage != nullptr) {
                    *errorMessage = "Water path cache sidecar header is invalid.";
                }
                return std::nullopt;
            }
            std::error_code sizeError;
            const auto fileBytes = std::filesystem::file_size(inputPath, sizeError);
            const std::uint64_t headerBytes =
                static_cast<std::uint64_t>(magic.size()) + sizeof(schema) +
                sizeof(payloadBytes) +
                expectedChecksum.size() * sizeof(expectedChecksum.front());
            if (sizeError || fileBytes != headerBytes + payloadBytes) {
                if (errorMessage != nullptr) {
                    *errorMessage =
                        "Water path cache sidecar payload length does not match the file.";
                }
                return std::nullopt;
            }
            try {
                std::vector<std::uint8_t> payload(
                    static_cast<std::size_t>(payloadBytes));
                binaryInput.read(
                    reinterpret_cast<char*>(payload.data()),
                    static_cast<std::streamsize>(payload.size()));
                if (!binaryInput.good() ||
                    binaryInput.peek() != std::char_traits<char>::eof()) {
                    if (errorMessage != nullptr) {
                        *errorMessage =
                            "Water path cache sidecar payload is truncated or has trailing data.";
                    }
                    return std::nullopt;
                }
                const auto checksum = DigestWaterPathCachePayload(payload);
                if (checksum != expectedChecksum) {
                    if (errorMessage != nullptr) {
                        *errorMessage = "Water path cache sidecar checksum is invalid.";
                    }
                    return std::nullopt;
                }
                auto cache = ParseWaterPathCache(json::from_cbor(payload));
                if (manifest != nullptr) {
                    manifest->relativePath = inputPath;
                    manifest->cacheSchema = schema;
                    manifest->supportSignature = cache.supportSignature;
                    manifest->emitterSettingsFingerprint = cache.emitterSettingsFingerprint;
                    manifest->payloadBytes = payloadBytes;
                    manifest->checksum = checksum;
                }
                return cache;
            } catch (const std::exception& error) {
                if (errorMessage != nullptr) {
                    *errorMessage = "Failed to allocate or decode water path cache sidecar: " +
                                    std::string{error.what()};
                }
                return std::nullopt;
            }
        }
    }

    const auto cacheJson = ReadJsonDocument(inputPath, errorMessage);
    if (!cacheJson.has_value()) {
        return std::nullopt;
    }

    try {
        return ParseWaterPathCache(cacheJson.value());
    } catch (const std::exception& error) {
        if (errorMessage != nullptr) {
            *errorMessage = "Failed to parse water path cache: " + std::string{error.what()};
        }
        return std::nullopt;
    }
}

}  // namespace invisible_places::serialization
