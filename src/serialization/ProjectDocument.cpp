#include "serialization/ProjectDocument.hpp"

#include "style/RenderParameterBinding.hpp"

#include <fstream>

#include <nlohmann/json.hpp>

namespace invisible_places::serialization {

namespace {

using nlohmann::json;
using invisible_places::renderer::pointcloud::PointCloudColorMode;
using invisible_places::renderer::pointcloud::PointCloudColormapId;
using invisible_places::renderer::pointcloud::PointCloudStyleState;
using invisible_places::style::FieldMapConfig;
using invisible_places::style::ParameterSourceMode;
using invisible_places::style::RenderParameterBinding;

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
    return PointCloudColormapId::Viridis;
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

json SerializePointCloudStyle(const PointCloudStyleState& style) {
    return json{
        {"color_mode", PointCloudColorModeName(style.colorMode)},
        {"colormap", PointCloudColormapName(style.colormap)},
        {"solid_color", style.solidColor},
        {"point_size", SerializeBinding(style.pointSize)},
        {"opacity", SerializeBinding(style.opacity)},
        {"emissive_strength", SerializeBinding(style.emissiveStrength)},
        {"xray_strength", SerializeBinding(style.xrayStrength)},
        {"depth_fade", SerializeBinding(style.depthFade)},
        {"colormap_position", SerializeBinding(style.colormapPosition)},
    };
}

PointCloudStyleState ParsePointCloudStyle(const json& styleJson) {
    PointCloudStyleState style;
    if (styleJson.contains("color_mode")) {
        style.colorMode = ParsePointCloudColorMode(styleJson.at("color_mode"));
    }
    if (styleJson.contains("colormap")) {
        style.colormap = ParsePointCloudColormap(styleJson.at("colormap"));
    }
    if (styleJson.contains("solid_color")) {
        style.solidColor = styleJson.at("solid_color").get<std::array<float, 4>>();
    }
    if (styleJson.contains("point_size")) {
        style.pointSize = ParseBinding(styleJson.at("point_size"));
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
    return layer;
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
        {"background_color", document.backgroundColor},
        {"side_panel_pinned", document.sidePanelPinned},
        {"auto_lower_gsplat_quality_while_navigating", document.autoLowerGsplatQualityWhileNavigating},
    };

    projectJson["layers"] = json::array();
    for (const auto& layer : document.layers) {
        projectJson["layers"].push_back(SerializeProjectLayer(layer));
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
    document.backgroundColor =
        projectJson->value("background_color", std::array<float, 4>{0.0F, 0.0F, 0.0F, 1.0F});
    document.sidePanelPinned = projectJson->value("side_panel_pinned", false);
    document.autoLowerGsplatQualityWhileNavigating =
        projectJson->value("auto_lower_gsplat_quality_while_navigating", true);

    if (projectJson->contains("layers")) {
        for (const auto& layerJson : projectJson->at("layers")) {
            document.layers.push_back(ParseProjectLayer(layerJson));
        }
    }

    return document;
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
