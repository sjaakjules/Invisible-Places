#include "style/RenderParameterBinding.hpp"

#include <cstdint>
#include <limits>
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

TEST_CASE("Manual input bounds survive switching fields and back", "[style][field-binding]") {
    auto stats = Field("Interest");
    stats.minimum = -2.0F;
    stats.maximum = 6.0F;
    auto roughness = Field("Roughness");

    invisible_places::style::RenderParameterBinding binding;
    invisible_places::style::ConfigureFieldMapFromStats(
        &binding, 0, "Interest", 0.0F, 1.0F, &stats);
    REQUIRE(invisible_places::style::HasFieldMapFlag(
        binding.fieldMap,
        invisible_places::style::FieldMapFlagUseLayerStats));

    // Manual edit narrows the mapped range.
    invisible_places::style::SetFieldMapFlag(
        &binding.fieldMap,
        invisible_places::style::FieldMapFlagUseLayerStats,
        false);
    binding.fieldMap.inputMin = 0.5F;
    binding.fieldMap.inputMax = 3.5F;

    // Switch to Roughness: stash, defaults, no memory for the new field.
    invisible_places::style::RememberFieldMapBounds(&binding.fieldMap);
    invisible_places::style::ConfigureFieldMapFromStats(
        &binding, 1, "Roughness", 0.0F, 1.0F, &roughness);
    CHECK_FALSE(
        invisible_places::style::RestoreFieldMapBoundsMemory(&binding.fieldMap));
    CHECK(invisible_places::style::HasFieldMapFlag(
        binding.fieldMap,
        invisible_places::style::FieldMapFlagUseLayerStats));

    // Return to Interest: the manual bounds come back and stats mode stays off.
    invisible_places::style::RememberFieldMapBounds(&binding.fieldMap);
    invisible_places::style::ConfigureFieldMapFromStats(
        &binding, 0, "Interest", 0.0F, 1.0F, &stats);
    CHECK(invisible_places::style::RestoreFieldMapBoundsMemory(&binding.fieldMap));
    CHECK(binding.fieldMap.inputMin == Catch::Approx(0.5F));
    CHECK(binding.fieldMap.inputMax == Catch::Approx(3.5F));
    CHECK_FALSE(invisible_places::style::HasFieldMapFlag(
        binding.fieldMap,
        invisible_places::style::FieldMapFlagUseLayerStats));
}

TEST_CASE("Reverting a field to layer stats forgets its remembered bounds", "[style][field-binding]") {
    auto stats = Field("Interest");
    auto roughness = Field("Roughness");

    invisible_places::style::RenderParameterBinding binding;
    invisible_places::style::ConfigureFieldMapFromStats(
        &binding, 0, "Interest", 0.0F, 1.0F, &stats);
    invisible_places::style::SetFieldMapFlag(
        &binding.fieldMap,
        invisible_places::style::FieldMapFlagUseLayerStats,
        false);
    binding.fieldMap.inputMin = 0.2F;
    binding.fieldMap.inputMax = 0.8F;
    invisible_places::style::RememberFieldMapBounds(&binding.fieldMap);
    REQUIRE(binding.fieldMap.boundsMemory.size() == 1U);

    // Back on layer stats: leaving the field must erase the stale entry so a
    // later return gives the defaults again.
    invisible_places::style::SetFieldMapFlag(
        &binding.fieldMap,
        invisible_places::style::FieldMapFlagUseLayerStats,
        true);
    invisible_places::style::RememberFieldMapBounds(&binding.fieldMap);
    CHECK(binding.fieldMap.boundsMemory.empty());

    invisible_places::style::ConfigureFieldMapFromStats(
        &binding, 1, "Roughness", 0.0F, 1.0F, &roughness);
    CHECK_FALSE(
        invisible_places::style::RestoreFieldMapBoundsMemory(&binding.fieldMap));
}

TEST_CASE("Bounds memory sanitize drops unusable and duplicate entries", "[style][field-binding]") {
    invisible_places::style::FieldMapConfig config;
    config.fieldName = "Current";
    config.boundsMemory = {
        {.fieldName = "", .inputMin = 0.0F, .inputMax = 1.0F},
        {.fieldName = "Stale", .inputMin = 5.0F, .inputMax = 2.0F},
        {.fieldName = "stale", .inputMin = 4.0F, .inputMax = -1.0F},
        {.fieldName = "Current", .inputMin = 0.1F, .inputMax = 0.9F},
        {.fieldName = "Kept",
         .inputMin = std::numeric_limits<float>::quiet_NaN(),
         .inputMax = 1.0F},
        {.fieldName = "Kept", .inputMin = 0.25F, .inputMax = 0.75F},
    };

    invisible_places::style::SanitizeFieldMapBoundsMemory(&config);

    REQUIRE(config.boundsMemory.size() == 2U);
    // The later duplicate wins and inverted bounds are reordered.
    CHECK(config.boundsMemory[0].fieldName == "stale");
    CHECK(config.boundsMemory[0].inputMin == Catch::Approx(-1.0F));
    CHECK(config.boundsMemory[0].inputMax == Catch::Approx(4.0F));
    CHECK(config.boundsMemory[1].fieldName == "Kept");
    CHECK(config.boundsMemory[1].inputMin == Catch::Approx(0.25F));
    CHECK(config.boundsMemory[1].inputMax == Catch::Approx(0.75F));
}
