#pragma once

#include <array>
#include <cstdint>
#include <string>

namespace invisible_places::style {

enum class ParameterSourceMode {
    Constant,
    FieldMapped
};

struct FieldMapConfig {
    std::string fieldName;
    float inputMin = 0.0F;
    float inputMax = 1.0F;
    float outputMin = 0.0F;
    float outputMax = 1.0F;
    float gamma = 1.0F;
    std::uint32_t flags = 0;
};

struct RenderParameterBinding {
    ParameterSourceMode mode = ParameterSourceMode::Constant;
    std::array<float, 4> constantValue{0.0F, 0.0F, 0.0F, 0.0F};
    FieldMapConfig fieldMap{};
};

}  // namespace invisible_places::style

