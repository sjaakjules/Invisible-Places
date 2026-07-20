#include "style/RenderParameterBinding.hpp"

#include <cstdint>
#include <string>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

namespace {

invisible_places::io::ScalarFieldStats Field(std::string name) {
    invisible_places::io::ScalarFieldStats field;
    field.name = std::move(name);
    field.minimum = 0.0F;
    field.maximum = 1.0F;
    field.count = 1U;
    field.valid = true;
    return field;
}

invisible_places::style::RenderParameterBinding MappedBinding(
    std::int32_t slot,
    std::string name) {
    invisible_places::style::RenderParameterBinding binding;
    invisible_places::style::SetScalarConstant(&binding, 0.42F);
    invisible_places::style::ConfigureFieldMapFromStats(
        &binding,
        slot,
        name,
        1.5F,
        2.2F,
        nullptr);
    binding.fieldMap.inputMin = -3.0F;
    binding.fieldMap.inputMax = 9.0F;
    binding.fieldMap.gamma = 1.7F;
    binding.fieldMap.flags = invisible_places::style::FieldMapFlagInvert;
    return binding;
}

}  // namespace

TEST_CASE("Scalar binding names override stale numeric slots", "[style][field-binding]") {
    auto binding = MappedBinding(0, "Interest");
    const std::vector fields{Field("Roughness"), Field("Interest")};

    invisible_places::style::SyncBindingFieldReference(&binding, fields);

    CHECK(binding.mode == invisible_places::style::ParameterSourceMode::FieldMapped);
    CHECK(binding.fieldMap.fieldSlot == 1);
    CHECK(binding.fieldMap.fieldName == "Interest");
}

TEST_CASE("Scalar binding names accept one unique case-insensitive match", "[style][field-binding]") {
    auto binding = MappedBinding(7, "interest");
    const std::vector fields{Field("Roughness"), Field("Interest")};

    invisible_places::style::SyncBindingFieldReference(&binding, fields);

    CHECK(binding.fieldMap.fieldSlot == 1);
    CHECK(binding.fieldMap.fieldName == "Interest");
}

TEST_CASE("Ambiguous case-insensitive scalar names remain unresolved", "[style][field-binding]") {
    auto binding = MappedBinding(0, "interest");
    const std::vector fields{Field("Interest"), Field("INTEREST")};

    invisible_places::style::SyncBindingFieldReference(&binding, fields);

    CHECK(binding.mode == invisible_places::style::ParameterSourceMode::FieldMapped);
    CHECK(binding.fieldMap.fieldSlot == -1);
    CHECK(binding.fieldMap.fieldName == "interest");
}

TEST_CASE("Missing scalar names preserve authored mappings for later recovery", "[style][field-binding]") {
    auto binding = MappedBinding(1, "Interest");
    const auto originalConstant = binding.constantValue;
    const auto originalMap = binding.fieldMap;
    const std::vector fields{Field("Roughness"), Field("Classification")};

    invisible_places::style::SyncBindingFieldReference(&binding, fields);

    CHECK(binding.mode == invisible_places::style::ParameterSourceMode::FieldMapped);
    CHECK(binding.constantValue == originalConstant);
    CHECK(binding.fieldMap.fieldSlot == -1);
    CHECK(binding.fieldMap.fieldName == originalMap.fieldName);
    CHECK(binding.fieldMap.inputMin == Catch::Approx(originalMap.inputMin));
    CHECK(binding.fieldMap.inputMax == Catch::Approx(originalMap.inputMax));
    CHECK(binding.fieldMap.outputMin == Catch::Approx(originalMap.outputMin));
    CHECK(binding.fieldMap.outputMax == Catch::Approx(originalMap.outputMax));
    CHECK(binding.fieldMap.gamma == Catch::Approx(originalMap.gamma));
    CHECK(binding.fieldMap.flags == originalMap.flags);
}

TEST_CASE("Legacy slot-only scalar bindings acquire a durable name once", "[style][field-binding]") {
    auto binding = MappedBinding(1, "");
    const std::vector originalOrder{Field("Interest"), Field("Roughness")};

    invisible_places::style::SyncBindingFieldReference(&binding, originalOrder);

    CHECK(binding.fieldMap.fieldSlot == 1);
    CHECK(binding.fieldMap.fieldName == "Roughness");

    const std::vector reordered{Field("Roughness"), Field("Interest")};
    invisible_places::style::SyncBindingFieldReference(&binding, reordered);

    CHECK(binding.fieldMap.fieldSlot == 0);
    CHECK(binding.fieldMap.fieldName == "Roughness");
}
