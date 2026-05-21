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
using invisible_places::renderer::pointcloud::PointCloudRaycastPrimitiveMode;
using invisible_places::renderer::pointcloud::PointCloudRendererMode;
using invisible_places::renderer::pointcloud::PointCloudStyleState;
using invisible_places::renderer::pointcloud::PointCloudStylisationMode;
using invisible_places::style::FieldMapConfig;
using invisible_places::style::ParameterSourceMode;
using invisible_places::style::RenderParameterBinding;
using invisible_places::water::WaterBakeSettings;
using invisible_places::water::WaterEmitter;
using invisible_places::water::WaterEmitterOrigin;
using invisible_places::water::WaterEmitterStatus;
using invisible_places::water::WaterAnimationTrailSettings;
using invisible_places::water::WaterParticleTrailSettings;
using invisible_places::water::WaterParticleTrailShapeSettings;
using invisible_places::water::WaterParticleVisualSettings;
using invisible_places::water::WaterPathGenerationSettings;
using invisible_places::water::WaterRenderSettings;
using invisible_places::water::WaterScaleMode;
using invisible_places::water::WaterSettingsBundle;
using invisible_places::water::WaterSourceSettingsAssignment;
using invisible_places::water::WaterSourceSettings;
using invisible_places::water::WaterVisualSettings;

json SerializeWaterSettingsBundle(const WaterSettingsBundle& settings);
WaterSettingsBundle ParseWaterSettingsBundle(const json& settingsJson);
json SerializeWaterSourceSettings(const WaterSourceSettings& settings);
WaterSourceSettings ParseWaterSourceSettings(const json& settingsJson);
json SerializeWaterAnimationTrailSettings(const WaterAnimationTrailSettings& settings);
WaterAnimationTrailSettings ParseWaterAnimationTrailSettings(const json& settingsJson);
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
        case PointCloudRendererMode::Raytraced:
            return "raytraced";
    }

    return "beauty";
}

PointCloudRendererMode ParsePointCloudRendererMode(const json& value) {
    const auto modeName = value.get<std::string>();
    if (modeName == "fast_basic") {
        return PointCloudRendererMode::FastBasic;
    }
    if (modeName == "raytraced") {
        return PointCloudRendererMode::Raytraced;
    }
    return PointCloudRendererMode::Beauty;
}

const char* PointCloudRaycastPrimitiveModeName(PointCloudRaycastPrimitiveMode mode) {
    switch (mode) {
        case PointCloudRaycastPrimitiveMode::StyleSurfels:
            return "style_surfels";
        case PointCloudRaycastPrimitiveMode::SoftDensitySpheres:
            return "soft_density_spheres";
    }

    return "style_surfels";
}

PointCloudRaycastPrimitiveMode ParsePointCloudRaycastPrimitiveMode(const json& value) {
    const auto modeName = value.get<std::string>();
    if (modeName == "soft_density_spheres" || modeName == "spheres") {
        return PointCloudRaycastPrimitiveMode::SoftDensitySpheres;
    }
    return PointCloudRaycastPrimitiveMode::StyleSurfels;
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
        {"depth_contribution", PointCloudDepthContributionName(style.depthContribution)},
        {"falloff_profile", PointCloudFalloffProfileName(style.falloffProfile)},
        {"stylisation_mode", PointCloudStylisationModeName(style.stylisationMode)},
        {"npr_preset", PointCloudNprPresetName(style.nprPreset)},
        {"color_mode", PointCloudColorModeName(style.colorMode)},
        {"colormap", PointCloudColormapName(style.colormap)},
        {"solid_color", style.solidColor},
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
        {"depth_alpha_threshold", style.depthAlphaThreshold},
        {"solid_centers", style.solidCenters},
        {"flow_animation", style.flowAnimation},
        {"water_path_view", style.waterPathView},
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
    style.depthAlphaThreshold = styleJson.value("depth_alpha_threshold", style.depthAlphaThreshold);
    style.solidCenters = styleJson.value("solid_centers", style.solidCenters);
    style.flowAnimation = styleJson.value("flow_animation", style.flowAnimation);
    style.waterPathView = styleJson.value("water_path_view", style.waterPathView);
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
        {"schema_version", 6U},
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
    if (path.waterPointVisualStyle.has_value()) {
        pathJson["water_point_visual_style"] = SerializePointCloudStyle(path.waterPointVisualStyle.value());
    }
    if (path.tempWaterPointVisualStyle.has_value()) {
        pathJson["temp_water_point_visual_style"] = SerializePointCloudStyle(path.tempWaterPointVisualStyle.value());
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
        {"raycast_primitive_mode", PointCloudRaycastPrimitiveModeName(settings.raycastPrimitiveMode)},
        {"raycast_samples_per_pixel", settings.raycastSamplesPerPixel},
        {"raycast_max_depth", settings.raycastMaxDepth},
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
    if (settingsJson.contains("raycast_primitive_mode")) {
        settings.raycastPrimitiveMode =
            ParsePointCloudRaycastPrimitiveMode(settingsJson.at("raycast_primitive_mode"));
    }
    settings.raycastSamplesPerPixel = settingsJson.value("raycast_samples_per_pixel", 4U);
    settings.raycastMaxDepth = settingsJson.value("raycast_max_depth", 0.0F);
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

json SerializeWaterPathGenerationSettings(const WaterPathGenerationSettings& settings) {
    return json{
        {"support_voxel_size", settings.supportVoxelSize},
        {"max_bridge_distance", settings.maxBridgeDistance},
        {"smoothing", settings.smoothing},
        {"path_length", settings.pathLength},
        {"path_sample_spacing", settings.pathSampleSpacing},
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
    settings.supportVoxelSize = settingsJson.value("support_voxel_size", settings.supportVoxelSize);
    settings.maxBridgeDistance = settingsJson.value("max_bridge_distance", settings.maxBridgeDistance);
    settings.smoothing = settingsJson.value("smoothing", settings.smoothing);
    settings.pathLength = settingsJson.value("path_length", settings.pathLength);
    settings.pathSampleSpacing = settingsJson.value(
        "path_sample_spacing",
        settingsJson.value("path_density", settings.pathSampleSpacing));
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
    };
}

WaterParticleTrailShapeSettings ParseWaterParticleTrailShapeSettings(const json& settingsJson) {
    WaterParticleTrailShapeSettings settings;
    settings.particleJitter =
        std::clamp(settingsJson.value("particle_jitter", settings.particleJitter), 0.0F, 3.0F);
    settings.splineAnchorSpacing =
        std::clamp(settingsJson.value("spline_anchor_spacing", settings.splineAnchorSpacing), 0.01F, 25.0F);
    return settings;
}

json SerializeWaterAnimationTrailSettings(const WaterAnimationTrailSettings& settings) {
    return json{
        {"particle_density", settings.particleDensity},
        {"particle_speed", settings.particleSpeed},
        {"color_variation", settings.colorVariation},
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
        {"water_point_visual_style", SerializePointCloudStyle(document.waterPointVisualStyle)},
    };
    if (document.tempWaterSourceSettings.has_value()) {
        projectJson["temp_water_source_settings"] =
            SerializeWaterSourceSettings(document.tempWaterSourceSettings.value());
    }
    if (document.tempWaterAnimationTrailSettings.has_value()) {
        projectJson["temp_water_animation_trail_settings"] =
            SerializeWaterAnimationTrailSettings(document.tempWaterAnimationTrailSettings.value());
    }
    if (document.tempWaterPointVisualStyle.has_value()) {
        projectJson["temp_water_point_visual_style"] =
            SerializePointCloudStyle(document.tempWaterPointVisualStyle.value());
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
    if (projectJson->contains("temp_water_animation_trail_settings")) {
        document.tempWaterAnimationTrailSettings =
            ParseWaterAnimationTrailSettings(projectJson->at("temp_water_animation_trail_settings"));
    }
    if (projectJson->contains("water_point_visual_style")) {
        document.waterPointVisualStyle = ParsePointCloudStyle(projectJson->at("water_point_visual_style"));
    }
    if (projectJson->contains("temp_water_point_visual_style")) {
        document.tempWaterPointVisualStyle = ParsePointCloudStyle(projectJson->at("temp_water_point_visual_style"));
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
        {"water_emitters", json::array()},
    };
    if (document.tempSourceSettings.has_value()) {
        sourcesJson["temp_water_source_settings"] =
            SerializeWaterSourceSettings(document.tempSourceSettings.value());
    }
    for (const auto& emitter : document.emitters) {
        sourcesJson["water_emitters"].push_back(SerializeWaterEmitter(emitter));
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

}  // namespace invisible_places::serialization
