#include "style/RenderParameterBinding.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <optional>

namespace invisible_places::style {

namespace {

float SafeGamma(float value) {
    return std::max(0.0001F, value);
}

std::string NormalizedFieldName(std::string_view value) {
    std::string normalized;
    normalized.reserve(value.size());
    for (const char character : value) {
        const auto byte = static_cast<unsigned char>(character);
        normalized.push_back(static_cast<char>(std::tolower(byte)));
    }
    return normalized;
}

}  // namespace

bool HasFieldMapFlag(const FieldMapConfig& config, FieldMapFlags flag) {
    return (config.flags & static_cast<std::uint32_t>(flag)) != 0U;
}

void SetFieldMapFlag(FieldMapConfig* config, FieldMapFlags flag, bool enabled) {
    if (config == nullptr) {
        return;
    }

    if (enabled) {
        config->flags |= static_cast<std::uint32_t>(flag);
    } else {
        config->flags &= ~static_cast<std::uint32_t>(flag);
    }
}

void SetScalarConstant(RenderParameterBinding* binding, float value) {
    if (binding == nullptr) {
        return;
    }

    binding->constantValue[0] = value;
}

float ScalarConstant(const RenderParameterBinding& binding) {
    return binding.constantValue[0];
}

void ConfigureFieldMapFromStats(
    RenderParameterBinding* binding,
    std::int32_t fieldSlot,
    const std::string& fieldName,
    float outputMin,
    float outputMax,
    const invisible_places::io::ScalarFieldStats* fieldStats) {
    if (binding == nullptr) {
        return;
    }

    binding->mode = ParameterSourceMode::FieldMapped;
    binding->fieldMap.fieldSlot = fieldSlot;
    binding->fieldMap.fieldName = fieldName;
    binding->fieldMap.outputMin = outputMin;
    binding->fieldMap.outputMax = outputMax;
    binding->fieldMap.gamma = 1.0F;
    binding->fieldMap.flags = FieldMapFlagClamp | FieldMapFlagUseLayerStats;

    if (fieldStats != nullptr && fieldStats->valid) {
        binding->fieldMap.inputMin = fieldStats->minimum;
        binding->fieldMap.inputMax = fieldStats->maximum;
    } else {
        binding->fieldMap.inputMin = 0.0F;
        binding->fieldMap.inputMax = 1.0F;
    }
}

void SyncBindingFieldReference(
    RenderParameterBinding* binding,
    const std::vector<invisible_places::io::ScalarFieldStats>& scalarFields) {
    if (binding == nullptr || binding->mode != ParameterSourceMode::FieldMapped) {
        return;
    }

    // Field names are the stable contract between density variants. CloudCompare
    // exports do not guarantee that otherwise-identical PLYs keep scalar fields
    // in the same property order, so a valid old slot must never override a
    // saved name.
    if (!binding->fieldMap.fieldName.empty()) {
        const auto exactIt = std::find_if(
            scalarFields.begin(),
            scalarFields.end(),
            [&binding](const invisible_places::io::ScalarFieldStats& field) {
                return field.name == binding->fieldMap.fieldName;
            });
        if (exactIt != scalarFields.end()) {
            binding->fieldMap.fieldSlot = static_cast<std::int32_t>(
                std::distance(scalarFields.begin(), exactIt));
            binding->fieldMap.fieldName = exactIt->name;
            return;
        }

        const auto normalizedTarget = NormalizedFieldName(binding->fieldMap.fieldName);
        std::optional<std::size_t> normalizedMatch;
        for (std::size_t index = 0; index < scalarFields.size(); ++index) {
            if (NormalizedFieldName(scalarFields[index].name) != normalizedTarget) {
                continue;
            }
            if (normalizedMatch.has_value()) {
                normalizedMatch.reset();
                break;
            }
            normalizedMatch = index;
        }
        if (normalizedMatch.has_value()) {
            binding->fieldMap.fieldSlot = static_cast<std::int32_t>(normalizedMatch.value());
            binding->fieldMap.fieldName = scalarFields[normalizedMatch.value()].name;
            return;
        }

        // Keep the authored name and mapping settings so the binding can recover
        // when the user switches back to a variant that provides the field.
        binding->fieldMap.fieldSlot = -1;
        return;
    }

    // Slot-only bindings are legacy data. Populate their durable name once.
    if (binding->fieldMap.fieldSlot >= 0 &&
        static_cast<std::size_t>(binding->fieldMap.fieldSlot) < scalarFields.size()) {
        binding->fieldMap.fieldName = scalarFields[static_cast<std::size_t>(binding->fieldMap.fieldSlot)].name;
    } else {
        binding->fieldMap.fieldSlot = -1;
    }
}

float ResolveBindingInputMinimum(
    const RenderParameterBinding& binding,
    const invisible_places::io::ScalarFieldStats* fieldStats) {
    if (fieldStats != nullptr && fieldStats->valid &&
        HasFieldMapFlag(binding.fieldMap, FieldMapFlagUseLayerStats)) {
        return fieldStats->minimum;
    }

    return binding.fieldMap.inputMin;
}

float ResolveBindingInputMaximum(
    const RenderParameterBinding& binding,
    const invisible_places::io::ScalarFieldStats* fieldStats) {
    if (fieldStats != nullptr && fieldStats->valid &&
        HasFieldMapFlag(binding.fieldMap, FieldMapFlagUseLayerStats)) {
        return fieldStats->maximum;
    }

    return binding.fieldMap.inputMax;
}

float EvaluateScalarBinding(
    const RenderParameterBinding& binding,
    float fieldValue,
    const invisible_places::io::ScalarFieldStats* fieldStats) {
    if (binding.mode == ParameterSourceMode::Constant) {
        return ScalarConstant(binding);
    }

    const float inputMin = ResolveBindingInputMinimum(binding, fieldStats);
    const float inputMax = ResolveBindingInputMaximum(binding, fieldStats);
    const float inputWidth = std::max(1.0e-5F, inputMax - inputMin);

    float normalized = (fieldValue - inputMin) / inputWidth;
    if (HasFieldMapFlag(binding.fieldMap, FieldMapFlagInvert)) {
        normalized = 1.0F - normalized;
    }

    if (HasFieldMapFlag(binding.fieldMap, FieldMapFlagClamp)) {
        normalized = std::clamp(normalized, 0.0F, 1.0F);
        normalized = std::pow(normalized, SafeGamma(binding.fieldMap.gamma));
    } else {
        const float sign = normalized < 0.0F ? -1.0F : 1.0F;
        normalized = sign * std::pow(std::abs(normalized), SafeGamma(binding.fieldMap.gamma));
    }

    return binding.fieldMap.outputMin +
           ((binding.fieldMap.outputMax - binding.fieldMap.outputMin) * normalized);
}

}  // namespace invisible_places::style
