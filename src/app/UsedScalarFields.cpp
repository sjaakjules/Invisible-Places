#include "app/UsedScalarFields.hpp"

#include <cctype>

namespace invisible_places::app {

namespace {

std::string NormalizedUsedFieldName(std::string_view name) {
    std::string normalized;
    normalized.reserve(name.size());
    for (const char character : name) {
        const auto byte = static_cast<unsigned char>(character);
        if (std::isalnum(byte) == 0) {
            continue;
        }
        normalized.push_back(static_cast<char>(std::tolower(byte)));
    }
    return normalized;
}

}  // namespace

void UsedScalarFieldSet::AddFieldName(std::string_view name) {
    if (name.empty()) {
        return;
    }
    auto normalized = NormalizedUsedFieldName(name);
    if (normalized.empty()) {
        return;
    }
    if (!normalizedNames_.insert(std::move(normalized)).second) {
        return;
    }
    orderedNames_.emplace_back(name);
}

void UsedScalarFieldSet::AddBinding(
    const style::RenderParameterBinding& binding) {
    AddFieldName(binding.fieldMap.fieldName);
}

void UsedScalarFieldSet::AddColouriseEffect(
    const timing::TimingColouriseEffect& effect) {
    if (effect.field.source != timing::TimingColouriseFieldSource::Scalar) {
        return;
    }
    AddFieldName(effect.field.scalarFieldName);
}

bool UsedScalarFieldSet::Contains(std::string_view name) const {
    return normalizedNames_.contains(NormalizedUsedFieldName(name));
}

const std::vector<std::string>& AlwaysResidentScalarFieldPatterns() {
    static const std::vector<std::string> patterns{
        "roughness",
        "groundid",
    };
    return patterns;
}

}  // namespace invisible_places::app
