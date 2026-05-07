#pragma once

#include "io/PointCloudData.hpp"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace invisible_places::style {

enum class ParameterSourceMode {
    Constant,
    FieldMapped
};

enum FieldMapFlags : std::uint32_t {
    FieldMapFlagNone = 0U,
    FieldMapFlagClamp = 1U << 0U,
    FieldMapFlagInvert = 1U << 1U,
    FieldMapFlagUseLayerStats = 1U << 2U,
};

struct FieldMapConfig {
    std::int32_t fieldSlot = -1;
    std::string fieldName;
    float inputMin = 0.0F;
    float inputMax = 1.0F;
    float outputMin = 0.0F;
    float outputMax = 1.0F;
    float gamma = 1.0F;
    std::uint32_t flags = FieldMapFlagClamp | FieldMapFlagUseLayerStats;
};

struct RenderParameterBinding {
    bool active = true;
    ParameterSourceMode mode = ParameterSourceMode::Constant;
    std::array<float, 4> constantValue{0.0F, 0.0F, 0.0F, 0.0F};
    FieldMapConfig fieldMap{};
};

[[nodiscard]] bool HasFieldMapFlag(const FieldMapConfig& config, FieldMapFlags flag);
void SetFieldMapFlag(FieldMapConfig* config, FieldMapFlags flag, bool enabled);
void SetScalarConstant(RenderParameterBinding* binding, float value);
[[nodiscard]] float ScalarConstant(const RenderParameterBinding& binding);
void ConfigureFieldMapFromStats(
    RenderParameterBinding* binding,
    std::int32_t fieldSlot,
    const std::string& fieldName,
    float outputMin,
    float outputMax,
    const invisible_places::io::ScalarFieldStats* fieldStats);
void SyncBindingFieldReference(
    RenderParameterBinding* binding,
    const std::vector<invisible_places::io::ScalarFieldStats>& scalarFields);
[[nodiscard]] float ResolveBindingInputMinimum(
    const RenderParameterBinding& binding,
    const invisible_places::io::ScalarFieldStats* fieldStats);
[[nodiscard]] float ResolveBindingInputMaximum(
    const RenderParameterBinding& binding,
    const invisible_places::io::ScalarFieldStats* fieldStats);
[[nodiscard]] float EvaluateScalarBinding(
    const RenderParameterBinding& binding,
    float fieldValue,
    const invisible_places::io::ScalarFieldStats* fieldStats);

}  // namespace invisible_places::style
