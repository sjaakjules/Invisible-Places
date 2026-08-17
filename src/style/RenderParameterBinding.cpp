#include "style/RenderParameterBinding.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <iomanip>
#include <optional>
#include <sstream>

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

bool AuthoringFloatEqual(float left, float right, float epsilon) {
    if (left == right) {
        return true;
    }
    if (std::isnan(left) && std::isnan(right)) {
        return true;
    }
    return std::isfinite(left) && std::isfinite(right) &&
           std::abs(left - right) <= std::max(0.0F, epsilon);
}

std::vector<FieldMapBoundsMemoryEntry> CanonicalBoundsMemory(
    const FieldMapConfig& source) {
    auto sanitized = source;
    SanitizeFieldMapBoundsMemory(&sanitized);
    std::sort(
        sanitized.boundsMemory.begin(),
        sanitized.boundsMemory.end(),
        [](const FieldMapBoundsMemoryEntry& left,
           const FieldMapBoundsMemoryEntry& right) {
            return NormalizedFieldName(left.fieldName) <
                   NormalizedFieldName(right.fieldName);
        });
    return sanitized.boundsMemory;
}

std::string FormatAuthoringFloat(float value) {
    std::ostringstream output;
    output << std::setprecision(6) << std::defaultfloat << value;
    return output.str();
}

void AppendFieldIdentity(std::ostringstream* output, const FieldMapConfig& map) {
    if (!map.fieldName.empty()) {
        *output << "field " << std::quoted(map.fieldName);
    } else if (map.fieldSlot >= 0) {
        *output << "field slot " << map.fieldSlot;
    } else {
        *output << "no field";
    }
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

namespace {

std::vector<FieldMapBoundsMemoryEntry>::iterator FindBoundsMemoryEntry(
    std::vector<FieldMapBoundsMemoryEntry>& memory,
    std::string_view fieldName) {
    const auto normalized = NormalizedFieldName(fieldName);
    return std::find_if(
        memory.begin(),
        memory.end(),
        [&](const FieldMapBoundsMemoryEntry& entry) {
            return NormalizedFieldName(entry.fieldName) == normalized;
        });
}

}  // namespace

void RememberFieldMapBounds(FieldMapConfig* config) {
    if (config == nullptr || config->fieldName.empty()) {
        return;
    }

    const auto existing =
        FindBoundsMemoryEntry(config->boundsMemory, config->fieldName);
    if (HasFieldMapFlag(*config, FieldMapFlagUseLayerStats)) {
        // Layer-stats bounds always re-derive from the field itself, so a
        // remembered manual pair would wrongly resurrect on return.
        if (existing != config->boundsMemory.end()) {
            config->boundsMemory.erase(existing);
        }
        return;
    }
    if (!std::isfinite(config->inputMin) ||
        !std::isfinite(config->inputMax)) {
        return;
    }

    if (existing != config->boundsMemory.end()) {
        existing->fieldName = config->fieldName;
        existing->inputMin = config->inputMin;
        existing->inputMax = config->inputMax;
    } else {
        config->boundsMemory.push_back(
            {.fieldName = config->fieldName,
             .inputMin = config->inputMin,
             .inputMax = config->inputMax});
    }
}

bool RestoreFieldMapBoundsMemory(FieldMapConfig* config) {
    if (config == nullptr || config->fieldName.empty()) {
        return false;
    }

    const auto entry =
        FindBoundsMemoryEntry(config->boundsMemory, config->fieldName);
    if (entry == config->boundsMemory.end()) {
        return false;
    }

    config->inputMin = entry->inputMin;
    config->inputMax = entry->inputMax;
    SetFieldMapFlag(config, FieldMapFlagUseLayerStats, false);
    return true;
}

void SanitizeFieldMapBoundsMemory(FieldMapConfig* config) {
    if (config == nullptr) {
        return;
    }

    constexpr std::size_t kMaximumEntries = 64U;
    const auto currentName = NormalizedFieldName(config->fieldName);
    std::vector<FieldMapBoundsMemoryEntry> sanitized;
    sanitized.reserve(std::min(config->boundsMemory.size(), kMaximumEntries));
    // Walk newest-first so duplicate names keep the most recent entry, then
    // restore the original ordering for stable serialization.
    for (auto it = config->boundsMemory.rbegin();
         it != config->boundsMemory.rend();
         ++it) {
        if (it->fieldName.empty() || !std::isfinite(it->inputMin) ||
            !std::isfinite(it->inputMax)) {
            continue;
        }
        const auto normalized = NormalizedFieldName(it->fieldName);
        if (!currentName.empty() && normalized == currentName) {
            continue;
        }
        const bool duplicate = std::any_of(
            sanitized.begin(),
            sanitized.end(),
            [&](const FieldMapBoundsMemoryEntry& kept) {
                return NormalizedFieldName(kept.fieldName) == normalized;
            });
        if (duplicate || sanitized.size() >= kMaximumEntries) {
            continue;
        }
        auto entry = *it;
        if (entry.inputMin > entry.inputMax) {
            std::swap(entry.inputMin, entry.inputMax);
        }
        sanitized.push_back(std::move(entry));
    }
    std::reverse(sanitized.begin(), sanitized.end());
    config->boundsMemory = std::move(sanitized);
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

bool ScalarRenderParameterBindingsAuthoringEqual(
    const RenderParameterBinding& left,
    const RenderParameterBinding& right,
    float epsilon) {
    if (left.active != right.active || left.mode != right.mode ||
        !AuthoringFloatEqual(
            ScalarConstant(left), ScalarConstant(right), epsilon) ||
        left.fieldMap.flags != right.fieldMap.flags) {
        return false;
    }

    const auto leftField = NormalizedFieldName(left.fieldMap.fieldName);
    const auto rightField = NormalizedFieldName(right.fieldMap.fieldName);
    if (leftField != rightField ||
        (leftField.empty() &&
         left.fieldMap.fieldSlot != right.fieldMap.fieldSlot)) {
        return false;
    }

    const bool useLayerStats =
        HasFieldMapFlag(left.fieldMap, FieldMapFlagUseLayerStats);
    if ((!useLayerStats &&
         (!AuthoringFloatEqual(
              left.fieldMap.inputMin,
              right.fieldMap.inputMin,
              epsilon) ||
          !AuthoringFloatEqual(
              left.fieldMap.inputMax,
              right.fieldMap.inputMax,
              epsilon))) ||
        !AuthoringFloatEqual(
            left.fieldMap.outputMin,
            right.fieldMap.outputMin,
            epsilon) ||
        !AuthoringFloatEqual(
            left.fieldMap.outputMax,
            right.fieldMap.outputMax,
            epsilon) ||
        !AuthoringFloatEqual(
            left.fieldMap.gamma,
            right.fieldMap.gamma,
            epsilon)) {
        return false;
    }

    const auto leftMemory = CanonicalBoundsMemory(left.fieldMap);
    const auto rightMemory = CanonicalBoundsMemory(right.fieldMap);
    if (leftMemory.size() != rightMemory.size()) {
        return false;
    }
    for (std::size_t index = 0U; index < leftMemory.size(); ++index) {
        if (NormalizedFieldName(leftMemory[index].fieldName) !=
                NormalizedFieldName(rightMemory[index].fieldName) ||
            !AuthoringFloatEqual(
                leftMemory[index].inputMin,
                rightMemory[index].inputMin,
                epsilon) ||
            !AuthoringFloatEqual(
                leftMemory[index].inputMax,
                rightMemory[index].inputMax,
                epsilon)) {
            return false;
        }
    }
    return true;
}

std::string DescribeScalarRenderParameterBindingAuthoringState(
    const RenderParameterBinding& binding) {
    std::ostringstream output;
    output << (binding.active ? "active" : "inactive") << "; ";
    if (binding.mode == ParameterSourceMode::Constant) {
        output << "Constant "
               << FormatAuthoringFloat(ScalarConstant(binding))
               << "; retained ";
        AppendFieldIdentity(&output, binding.fieldMap);
    } else {
        output << "Field-Mapped ";
        AppendFieldIdentity(&output, binding.fieldMap);
    }

    if (HasFieldMapFlag(binding.fieldMap, FieldMapFlagUseLayerStats)) {
        output << "; input layer stats";
    } else {
        output << "; input "
               << FormatAuthoringFloat(binding.fieldMap.inputMin)
               << ".."
               << FormatAuthoringFloat(binding.fieldMap.inputMax);
    }
    output << "; output "
           << FormatAuthoringFloat(binding.fieldMap.outputMin)
           << ".."
           << FormatAuthoringFloat(binding.fieldMap.outputMax)
           << "; gamma "
           << FormatAuthoringFloat(binding.fieldMap.gamma)
           << "; clamp "
           << (HasFieldMapFlag(binding.fieldMap, FieldMapFlagClamp)
                   ? "on"
                   : "off")
           << "; invert "
           << (HasFieldMapFlag(binding.fieldMap, FieldMapFlagInvert)
                   ? "on"
                   : "off");
    if (binding.mode == ParameterSourceMode::FieldMapped) {
        output << "; retained constant "
               << FormatAuthoringFloat(ScalarConstant(binding));
    }

    constexpr std::uint32_t kKnownFlags =
        FieldMapFlagClamp | FieldMapFlagInvert |
        FieldMapFlagUseLayerStats;
    const std::uint32_t unknownFlags =
        binding.fieldMap.flags & ~kKnownFlags;
    if (unknownFlags != 0U) {
        output << "; unknown flags 0x" << std::hex << unknownFlags
               << std::dec;
    }

    const auto memory = CanonicalBoundsMemory(binding.fieldMap);
    output << "; remembered bounds ";
    if (memory.empty()) {
        output << "none";
    } else {
        for (std::size_t index = 0U; index < memory.size(); ++index) {
            if (index > 0U) {
                output << ", ";
            }
            output << std::quoted(memory[index].fieldName) << " "
                   << FormatAuthoringFloat(memory[index].inputMin)
                   << ".."
                   << FormatAuthoringFloat(memory[index].inputMax);
        }
    }
    return output.str();
}

}  // namespace invisible_places::style
