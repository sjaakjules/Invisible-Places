#include "serialization/ProjectDocument.hpp"

#include "style/RenderParameterBinding.hpp"

#include <algorithm>
#include <fstream>
#include <optional>
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
using invisible_places::output::RenderJobSettings;
using invisible_places::renderer::pointcloud::PointCloudColorMode;
using invisible_places::renderer::pointcloud::PointCloudColormapId;
using invisible_places::renderer::pointcloud::PointCloudDepthContribution;
using invisible_places::renderer::pointcloud::PointCloudFalloffProfile;
using invisible_places::renderer::pointcloud::PointCloudGeometryMode;
using invisible_places::renderer::pointcloud::PointCloudNprPreset;
using invisible_places::renderer::pointcloud::PointCloudPreviewLodMode;
using invisible_places::renderer::pointcloud::PointCloudRendererMode;
using invisible_places::renderer::pointcloud::PointCloudScreenSpriteSizeMode;
using invisible_places::renderer::pointcloud::PointCloudStyleState;
using invisible_places::renderer::pointcloud::PointCloudStylisationMode;
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
using invisible_places::water::WaterAnimationTrailSettings;
using invisible_places::water::WaterFieldOutputMode;
using invisible_places::water::WaterFieldSettings;
using invisible_places::water::WaterFieldStreamSettings;
using invisible_places::water::WaterFlowStreamSettings;
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
using invisible_places::water::WaterRippleOverlayType;
using invisible_places::water::WaterRipplePatternSettings;
using invisible_places::water::WaterScaleMode;
using invisible_places::water::WaterSettingsBundle;
using invisible_places::water::WaterSourceSettingsAssignment;
using invisible_places::water::WaterSourceSettings;
using invisible_places::water::WaterTrailGeometrySettings;
using invisible_places::water::WaterVisualSettings;

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
json SerializeWaterFlowStreamSettings(const WaterFlowStreamSettings& settings);
WaterFlowStreamSettings ParseWaterFlowStreamSettings(const json& settingsJson);
json SerializeWaterFieldSettings(const WaterFieldSettings& settings);
WaterFieldSettings ParseWaterFieldSettings(const json& settingsJson);
json SerializeWaterFieldStreamSettings(const WaterFieldStreamSettings& settings);
WaterFieldStreamSettings ParseWaterFieldStreamSettings(const json& settingsJson);
json SerializeWaterVisualSettings(const WaterVisualSettings& settings);
WaterVisualSettings ParseWaterVisualSettings(const json& settingsJson);
PointCloudStyleState MakeLegacyWaterPointVisualStyle(const WaterVisualSettings& visualSettings);

enum class LegacyPointCloudRenderMode {
    Solid,
    EmissiveHard,
    EmissiveFeathered,
    DepthXray,
    WeightedTransparent,
    ComputeDensity,
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
    if (modeName == "depth_xray") {
        return LegacyPointCloudRenderMode::DepthXray;
    }
    if (modeName == "weighted_transparent") {
        return LegacyPointCloudRenderMode::WeightedTransparent;
    }
    if (modeName == "compute_density") {
        return LegacyPointCloudRenderMode::ComputeDensity;
    }
    if (modeName == "gaussian_point_sprite") {
        return LegacyPointCloudRenderMode::GaussianPointSprite;
    }
    return LegacyPointCloudRenderMode::Solid;
}

const char* PointCloudDepthContributionName(PointCloudDepthContribution contribution) {
    switch (contribution) {
        case PointCloudDepthContribution::None:
            return "none";
        case PointCloudDepthContribution::AlphaThreshold:
            return "alpha_threshold";
        case PointCloudDepthContribution::Always:
            return "always";
    }

    return "alpha_threshold";
}

PointCloudDepthContribution ParsePointCloudDepthContribution(const json& value) {
    const auto contributionName = value.get<std::string>();
    if (contributionName == "none") {
        return PointCloudDepthContribution::None;
    }
    if (contributionName == "always") {
        return PointCloudDepthContribution::Always;
    }
    return PointCloudDepthContribution::AlphaThreshold;
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

const char* SerializedLayerKindName(SerializedLayerKind kind) {
    return kind == SerializedLayerKind::GaussianSplat ? "gsplat" : "point_cloud";
}

SerializedLayerKind ParseSerializedLayerKind(const json& value) {
    return value.get<std::string>() == "gsplat" ? SerializedLayerKind::GaussianSplat
                                                 : SerializedLayerKind::PointCloud;
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
            style->depthContribution = PointCloudDepthContribution::AlphaThreshold;
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
        case LegacyPointCloudRenderMode::DepthXray:
            style->depthContribution = PointCloudDepthContribution::Always;
            style->xrayStrength.active = true;
            if (invisible_places::style::ScalarConstant(style->xrayStrength) <= 0.0F) {
                invisible_places::style::SetScalarConstant(&style->xrayStrength, 1.0F);
            }
            break;
        case LegacyPointCloudRenderMode::WeightedTransparent:
        case LegacyPointCloudRenderMode::ComputeDensity:
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
        {"depth_contribution", PointCloudDepthContributionName(style.depthContribution)},
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
        {"exposure", style.exposure},
        {"inner_radius", style.innerRadius},
        {"gaussian_sharpness", style.gaussianSharpness},
        {"feather_power", style.featherPower},
        {"depth_falloff", style.depthFalloff},
        {"depth_bias", style.depthBias},
        {"front_alpha", style.frontAlpha},
        {"hidden_alpha", style.hiddenAlpha},
        {"density_scale", style.densityScale},
        {"density_clamp", style.densityClamp},
        {"water_streak_aspect", style.waterStreakAspect},
        {"depth_alpha_threshold", style.depthAlphaThreshold},
        {"solid_centers", style.solidCenters},
        {"flow_animation", style.flowAnimation},
        {"water_path_view", style.waterPathView},
        {"water_stream_overlay", style.waterStreamOverlay},
        {"point_size", SerializeBinding(style.pointSize)},
        {"surfel_diameter", SerializeBinding(style.surfelDiameter)},
        {"opacity", SerializeBinding(style.opacity)},
        {"emissive_strength", SerializeBinding(style.emissiveStrength)},
        {"xray_strength", SerializeBinding(style.xrayStrength)},
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
    if (styleJson.contains("depth_contribution")) {
        style.depthContribution = ParsePointCloudDepthContribution(styleJson.at("depth_contribution"));
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
    style.exposure = styleJson.value("exposure", style.exposure);
    style.innerRadius = styleJson.value("inner_radius", style.innerRadius);
    style.gaussianSharpness = styleJson.value("gaussian_sharpness", style.gaussianSharpness);
    style.featherPower = styleJson.value("feather_power", style.featherPower);
    style.depthFalloff = styleJson.value("depth_falloff", style.depthFalloff);
    style.depthBias = styleJson.value("depth_bias", style.depthBias);
    style.frontAlpha = styleJson.value("front_alpha", style.frontAlpha);
    style.hiddenAlpha = styleJson.value("hidden_alpha", style.hiddenAlpha);
    style.densityScale = styleJson.value("density_scale", style.densityScale);
    style.densityClamp = styleJson.value("density_clamp", style.densityClamp);
    style.waterStreakAspect = std::clamp(styleJson.value("water_streak_aspect", style.waterStreakAspect), 1.0F, 32.0F);
    style.depthAlphaThreshold = styleJson.value("depth_alpha_threshold", style.depthAlphaThreshold);
    style.solidCenters = styleJson.value("solid_centers", style.solidCenters);
    style.flowAnimation = styleJson.value("flow_animation", style.flowAnimation);
    style.waterPathView = styleJson.value("water_path_view", style.waterPathView);
    style.waterStreamOverlay = styleJson.value(
        "water_stream_overlay",
        styleJson.value("water_overlay_render_mode", std::string{}) == "stream");
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
    if (styleJson.contains("xray_strength")) {
        style.xrayStrength = ParseBinding(styleJson.at("xray_strength"));
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
    if (layer.pointStyle.has_value()) {
        layerJson["point_style"] = SerializePointCloudStyle(layer.pointStyle.value());
    }
    if (!layer.pointVisuals.empty()) {
        json visualsJson = json::array();
        for (const auto& visual : layer.pointVisuals) {
            visualsJson.push_back(SerializePointCloudVisual(visual));
        }
        layerJson["point_visuals"] = std::move(visualsJson);
        layerJson["selected_point_visual"] =
            layer.selectedPointVisualName.empty() ? std::string{"Unnamed"} : layer.selectedPointVisualName;
    }
    return layerJson;
}

ProjectLayerDocument ParseProjectLayer(const json& layerJson) {
    ProjectLayerDocument layer;
    layer.kind = ParseSerializedLayerKind(layerJson.at("kind"));
    layer.sourcePath = layerJson.value("source_path", std::string{});
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

json SerializeAnimationPath(const AnimationPath& path) {
    json pathJson{
        {"schema_version", 7U},
        {"name", path.name},
        {"duration_frames", path.durationFrames},
        {"associated_layer_paths", SerializePathArray(path.associatedLayerPaths)},
        {"depth_of_field_enabled", path.depthOfFieldEnabled},
        {"aperture_f_stops", path.apertureFStops},
        {"depth_of_field_max_blur_px", path.depthOfFieldMaxBlurPixels},
        {"export_settings", SerializeAnimationExportSettings(path.exportSettings)},
        {"export_visuals", path.exportVisualNames},
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
    if (pathJson.contains("export_settings")) {
        path.exportSettings = ParseAnimationExportSettings(pathJson.at("export_settings"));
    }
    if (pathJson.contains("export_visuals") && pathJson.at("export_visuals").is_array()) {
        path.exportVisualNames = pathJson.at("export_visuals").get<std::vector<std::string>>();
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
    return settings;
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
        case WaterFieldOutputMode::Streamlines:
            return "streamlines";
        case WaterFieldOutputMode::SurfaceMotion:
            return "surface_motion";
        case WaterFieldOutputMode::Both:
            return "both";
    }
    return "both";
}

WaterFieldOutputMode ParseWaterFieldOutputMode(const json& modeJson) {
    const auto name = modeJson.get<std::string>();
    if (name == "streamlines") {
        return WaterFieldOutputMode::Streamlines;
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

json SerializeWaterFlowStreamSettings(const WaterFlowStreamSettings& settings) {
    return json{
        {"enabled", settings.enabled},
        {"stream_count_total", settings.streamCountTotal},
        {"lane_count", settings.laneCount},
        {"stream_length_meters", settings.streamLengthMeters},
        {"stream_point_spacing_meters", settings.streamPointSpacingMeters},
        {"stream_width_meters", settings.streamWidthMeters},
        {"stream_world_length_meters", settings.streamWorldLengthMeters},
        {"surface_offset_meters", settings.surfaceOffsetMeters},
        {"path_attraction", settings.pathAttraction},
        {"lane_spread_meters", settings.laneSpreadMeters},
        {"lane_crossing", settings.laneCrossing},
        {"stream_smoothness", settings.streamSmoothness},
        {"stream_looseness", settings.streamLooseness},
        {"turbulence", settings.turbulence},
        {"speed_meters_per_second", settings.speedMetersPerSecond},
        {"seed", settings.seed},
    };
}

WaterFlowStreamSettings ParseWaterFlowStreamSettings(const json& settingsJson) {
    WaterFlowStreamSettings settings;
    settings.enabled = settingsJson.value("enabled", settings.enabled);
    settings.streamCountTotal = settingsJson.value("stream_count_total", settings.streamCountTotal);
    settings.laneCount = settingsJson.value("lane_count", settings.laneCount);
    settings.streamLengthMeters = settingsJson.value("stream_length_meters", settings.streamLengthMeters);
    settings.streamPointSpacingMeters = settingsJson.value("stream_point_spacing_meters", settings.streamPointSpacingMeters);
    settings.streamWidthMeters = settingsJson.value("stream_width_meters", settings.streamWidthMeters);
    settings.streamWorldLengthMeters = settingsJson.value("stream_world_length_meters", settings.streamWorldLengthMeters);
    settings.surfaceOffsetMeters = settingsJson.value("surface_offset_meters", settings.surfaceOffsetMeters);
    settings.pathAttraction = settingsJson.value("path_attraction", settings.pathAttraction);
    settings.laneSpreadMeters = settingsJson.value("lane_spread_meters", settings.laneSpreadMeters);
    settings.laneCrossing = settingsJson.value("lane_crossing", settings.laneCrossing);
    settings.streamSmoothness = settingsJson.value("stream_smoothness", settings.streamSmoothness);
    settings.streamLooseness = settingsJson.value("stream_looseness", settings.streamLooseness);
    settings.turbulence = settingsJson.value("turbulence", settings.turbulence);
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

json SerializeWaterFieldStreamSettings(const WaterFieldStreamSettings& settings) {
    return json{
        {"enabled", settings.enabled},
        {"streamline_count", settings.streamlineCount},
        {"seed_spacing_meters", settings.seedSpacingMeters},
        {"streamline_length_meters", settings.streamlineLengthMeters},
        {"step_length_meters", settings.stepLengthMeters},
        {"streamline_width_meters", settings.streamlineWidthMeters},
        {"stream_world_length_meters", settings.streamWorldLengthMeters},
        {"momentum", settings.momentum},
        {"max_turn_angle_degrees", settings.maxTurnAngleDegrees},
        {"speed_meters_per_second", settings.speedMetersPerSecond},
        {"fade_on_low_confidence", settings.fadeOnLowConfidence},
    };
}

WaterFieldStreamSettings ParseWaterFieldStreamSettings(const json& settingsJson) {
    WaterFieldStreamSettings settings;
    settings.enabled = settingsJson.value("enabled", settings.enabled);
    settings.streamlineCount = settingsJson.value("streamline_count", settings.streamlineCount);
    settings.seedSpacingMeters = settingsJson.value("seed_spacing_meters", settings.seedSpacingMeters);
    settings.streamlineLengthMeters = settingsJson.value("streamline_length_meters", settings.streamlineLengthMeters);
    settings.stepLengthMeters = settingsJson.value("step_length_meters", settings.stepLengthMeters);
    settings.streamlineWidthMeters = settingsJson.value("streamline_width_meters", settings.streamlineWidthMeters);
    settings.streamWorldLengthMeters = settingsJson.value("stream_world_length_meters", settings.streamWorldLengthMeters);
    settings.momentum = settingsJson.value("momentum", settings.momentum);
    settings.maxTurnAngleDegrees = settingsJson.value("max_turn_angle_degrees", settings.maxTurnAngleDegrees);
    settings.speedMetersPerSecond = settingsJson.value("speed_meters_per_second", settings.speedMetersPerSecond);
    settings.fadeOnLowConfidence = settingsJson.value("fade_on_low_confidence", settings.fadeOnLowConfidence);
    return settings;
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
        {"world_length_meters", settings.worldLengthMeters},
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
    settings.worldLengthMeters =
        std::clamp(settingsJson.value("world_length_meters", settings.worldLengthMeters), 0.001F, 5.0F);
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
        {"settings", SerializeWaterFlowStreamSettings(profile.settings)},
    };
}

WaterLaneProfileDocument ParseWaterLaneProfile(const json& profileJson) {
    WaterLaneProfileDocument profile;
    profile.name = profileJson.value("name", profile.name);
    if (profileJson.contains("settings")) {
        profile.settings = ParseWaterFlowStreamSettings(profileJson.at("settings"));
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
    style.depthContribution = PointCloudDepthContribution::None;
    style.falloffProfile = PointCloudFalloffProfile::Gaussian;
    style.colorMode = PointCloudColorMode::SourceRgb;
    style.solidColor = {0.04F, 0.74F, 1.0F, 1.0F};
    style.colorizeColor = {0.05F, 0.82F, 1.0F};
    style.colorizeAmount = 0.0F;
    style.exposure = 1.8F;
    style.gaussianSharpness = 1.65F;
    style.densityScale = 1.0F;
    style.densityClamp = 8.0F;
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
    invisible_places::style::SetScalarConstant(&style.xrayStrength, 0.0F);
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

    std::ofstream output{outputPath, std::ios::trunc};
    if (!output.is_open()) {
        if (errorMessage != nullptr) {
            *errorMessage = "Failed to open output file for writing.";
        }
        return false;
    }

    output << documentJson.dump(2);
    return true;
}

std::optional<json> ReadJsonDocument(const std::filesystem::path& inputPath, std::string* errorMessage) {
    std::ifstream input{inputPath};
    if (!input.is_open()) {
        if (errorMessage != nullptr) {
            *errorMessage = "Failed to open input file.";
        }
        return std::nullopt;
    }

    try {
        return json::parse(input);
    } catch (const std::exception& error) {
        if (errorMessage != nullptr) {
            *errorMessage = "Failed to parse JSON: " + std::string{error.what()};
        }
        return std::nullopt;
    }
}

}  // namespace

bool SaveProjectDocument(
    const ProjectDocument& document,
    const std::filesystem::path& outputPath,
    std::string* errorMessage) {
    json projectJson{
        {"schema_version", document.schemaVersion},
        {"project_name", document.projectName},
        {"selected_layer_path", document.selectedLayerPath.generic_string()},
        {"last_animation_path", document.lastAnimationPath.generic_string()},
        {"background_color", document.backgroundColor},
        {"eye_dome_lighting_enabled", document.eyeDomeLightingEnabled},
        {"eye_dome_lighting_thickness", document.eyeDomeLightingThickness},
        {"constant_update_view", document.constantUpdateView},
        {"live_visual_effects", document.liveVisualEffects},
        {"side_panel_pinned", document.sidePanelPinned},
        {"auto_lower_gsplat_quality_while_navigating", document.autoLowerGsplatQualityWhileNavigating},
        {"point_cloud_preview_lod_mode", PointCloudPreviewLodModeName(document.pointCloudPreviewLodMode)},
        {"interactive_point_cap", document.interactivePointCap},
        {"point_cloud_renderer_mode", PointCloudRendererModeName(document.pointCloudRendererMode)},
        {"render_job", SerializeRenderJobSettings(document.renderJobSettings)},
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
        {"water_flow_stream_settings", SerializeWaterFlowStreamSettings(document.waterFlowStreamSettings)},
        {"water_field_settings", SerializeWaterFieldSettings(document.waterFieldSettings)},
        {"water_field_stream_settings", SerializeWaterFieldStreamSettings(document.waterFieldStreamSettings)},
        {"water_point_visuals", json::array()},
        {"selected_water_point_visual", document.selectedWaterPointVisualName},
        {"water_ripple_layers", json::array()},
        {"water_field_layers", json::array()},
    };
    for (const auto& layer : document.waterRippleLayers) {
        projectJson["water_ripple_layers"].push_back(SerializeWaterEffectLayer(layer));
    }
    for (const auto& layer : document.waterFieldLayers) {
        projectJson["water_field_layers"].push_back(SerializeWaterEffectLayer(layer));
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
            SerializeWaterFlowStreamSettings(document.tempWaterLaneProfileSettings.value());
    }
    if (document.tempWaterTrailProfile.has_value()) {
        projectJson["temp_water_trail_profile"] =
            SerializeWaterTrailProfile(document.tempWaterTrailProfile.value());
    }
    if (document.tempWaterCausticLookSettings.has_value()) {
        projectJson["temp_water_caustic_look_settings"] =
            SerializeWaterCausticLookSettings(document.tempWaterCausticLookSettings.value());
    }
    if (document.waterPathCache.has_value() && !document.waterPathCache->branches.empty()) {
        projectJson["water_path_cache"] = SerializeWaterPathCache(document.waterPathCache.value());
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
    projectJson["water_emitters"] = json::array();
    for (const auto& emitter : document.waterEmitters) {
        projectJson["water_emitters"].push_back(SerializeWaterEmitter(emitter));
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
    document.projectName = projectJson->value("project_name", std::string{"Invisible Places"});
    document.selectedLayerPath = projectJson->value("selected_layer_path", std::string{});
    document.lastAnimationPath = projectJson->value("last_animation_path", std::string{});
    document.backgroundColor =
        projectJson->value("background_color", std::array<float, 4>{0.0F, 0.0F, 0.0F, 1.0F});
    document.eyeDomeLightingEnabled = projectJson->value("eye_dome_lighting_enabled", false);
    document.eyeDomeLightingThickness = projectJson->value("eye_dome_lighting_thickness", 1.0F);
    document.constantUpdateView = projectJson->value("constant_update_view", false);
    document.liveVisualEffects = projectJson->value("live_visual_effects", false);
    document.sidePanelPinned = projectJson->value("side_panel_pinned", false);
    document.autoLowerGsplatQualityWhileNavigating =
        projectJson->value("auto_lower_gsplat_quality_while_navigating", true);
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
    if (projectJson->contains("water_flow_stream_settings")) {
        document.waterFlowStreamSettings =
            ParseWaterFlowStreamSettings(projectJson->at("water_flow_stream_settings"));
    }
    if (!hasWaterTrailGeometry) {
        document.waterTrailGeometry =
            invisible_places::water::WaterTrailGeometryFromFlowStreamSettings(document.waterFlowStreamSettings);
    }
    if (projectJson->contains("water_field_settings")) {
        document.waterFieldSettings = ParseWaterFieldSettings(projectJson->at("water_field_settings"));
    }
    if (projectJson->contains("water_field_stream_settings")) {
        document.waterFieldStreamSettings =
            ParseWaterFieldStreamSettings(projectJson->at("water_field_stream_settings"));
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
            ParseWaterFlowStreamSettings(projectJson->at("temp_water_lane_profile_settings"));
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

    return document;
}

bool SaveWaterSourcesDocument(
    const WaterSourcesDocument& document,
    const std::filesystem::path& outputPath,
    std::string* errorMessage) {
    json sourcesJson{
        {"schema_version", document.schemaVersion},
        {"water_source_settings", SerializeWaterSourceSettings(document.sourceSettings)},
        {"water_caustic_look_settings", SerializeWaterCausticLookSettings(document.causticLookSettings)},
        {"water_flow_stream_settings", SerializeWaterFlowStreamSettings(document.flowStreamSettings)},
        {"water_trail_geometry", SerializeWaterTrailGeometrySettings(document.trailGeometry)},
        {"water_path_profiles", json::array()},
        {"water_lane_profiles", json::array()},
        {"water_trail_profiles", json::array()},
        {"selected_water_path_profile", document.selectedPathProfileName},
        {"selected_water_lane_profile", document.selectedLaneProfileName},
        {"selected_water_trail_profile", document.selectedTrailProfileName},
        {"water_field_settings", SerializeWaterFieldSettings(document.fieldSettings)},
        {"water_field_stream_settings", SerializeWaterFieldStreamSettings(document.fieldStreamSettings)},
        {"water_emitters", json::array()},
        {"water_ripple_layers", json::array()},
        {"water_field_layers", json::array()},
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
            SerializeWaterFlowStreamSettings(document.tempLaneProfileSettings.value());
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
    for (const auto& layer : document.rippleLayers) {
        sourcesJson["water_ripple_layers"].push_back(SerializeWaterEffectLayer(layer));
    }
    for (const auto& layer : document.fieldLayers) {
        sourcesJson["water_field_layers"].push_back(SerializeWaterEffectLayer(layer));
    }
    if (document.pathCache.has_value() && !document.pathCache->branches.empty()) {
        sourcesJson["water_path_cache"] = SerializeWaterPathCache(document.pathCache.value());
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
    if (sourcesJson->contains("water_flow_stream_settings")) {
        document.flowStreamSettings =
            ParseWaterFlowStreamSettings(sourcesJson->at("water_flow_stream_settings"));
    }
    if (sourcesJson->contains("water_trail_geometry")) {
        document.trailGeometry =
            ParseWaterTrailGeometrySettings(sourcesJson->at("water_trail_geometry"));
    } else {
        document.trailGeometry =
            invisible_places::water::WaterTrailGeometryFromFlowStreamSettings(document.flowStreamSettings);
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
            ParseWaterFlowStreamSettings(sourcesJson->at("temp_water_lane_profile_settings"));
    }
    if (sourcesJson->contains("temp_water_trail_profile")) {
        document.tempTrailProfile = ParseWaterTrailProfile(sourcesJson->at("temp_water_trail_profile"));
    }
    if (sourcesJson->contains("water_field_settings")) {
        document.fieldSettings = ParseWaterFieldSettings(sourcesJson->at("water_field_settings"));
    }
    if (sourcesJson->contains("water_field_stream_settings")) {
        document.fieldStreamSettings =
            ParseWaterFieldStreamSettings(sourcesJson->at("water_field_stream_settings"));
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
    std::string* errorMessage) {
    return WriteJsonDocument(document, SerializeWaterPathCache(document), outputPath, errorMessage);
}

std::optional<invisible_places::water::WaterPathCache> LoadWaterPathCacheDocument(
    const std::filesystem::path& inputPath,
    std::string* errorMessage) {
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
