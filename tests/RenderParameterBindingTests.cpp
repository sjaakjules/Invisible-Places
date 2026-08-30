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

TEST_CASE(
    "Binding defaults never rebind an authored name that is not resident",
    "[style][field-binding]") {
    // Residency streams columns on demand, so `scalarFields` is routinely a
    // partial subset that does not yet contain the authored field. The
    // authored mapping must survive untouched: the renderer shows the
    // constant fallback and the field loader keys on the retained name.
    auto binding = MappedBinding(7, "A_R_MeanCurvature_Combined");
    const auto originalMap = binding.fieldMap;
    const std::vector residentSubset{Field("Roughness"), Field("GroundID")};

    invisible_places::style::EnsureFieldMappedBindingDefaults(
        &binding,
        residentSubset,
        0.0F,
        1.0F);

    CHECK(binding.mode == invisible_places::style::ParameterSourceMode::FieldMapped);
    CHECK(binding.fieldMap.fieldSlot == -1);
    CHECK(binding.fieldMap.fieldName == "A_R_MeanCurvature_Combined");
    CHECK(binding.fieldMap.inputMin == Catch::Approx(originalMap.inputMin));
    CHECK(binding.fieldMap.inputMax == Catch::Approx(originalMap.inputMax));
    CHECK(binding.fieldMap.outputMin == Catch::Approx(originalMap.outputMin));
    CHECK(binding.fieldMap.outputMax == Catch::Approx(originalMap.outputMax));
    CHECK(binding.fieldMap.gamma == Catch::Approx(originalMap.gamma));
    CHECK(binding.fieldMap.flags == originalMap.flags);

    // Once the authored column streams in, the same entry point resolves the
    // durable name to its resident slot without touching the mapping.
    const std::vector withAuthored{
        Field("Roughness"),
        Field("GroundID"),
        Field("A_R_MeanCurvature_Combined"),
    };
    invisible_places::style::EnsureFieldMappedBindingDefaults(
        &binding,
        withAuthored,
        0.0F,
        1.0F);
    CHECK(binding.fieldMap.fieldSlot == 2);
    CHECK(binding.fieldMap.fieldName == "A_R_MeanCurvature_Combined");
    CHECK(binding.fieldMap.outputMin == Catch::Approx(originalMap.outputMin));
    CHECK(binding.fieldMap.outputMax == Catch::Approx(originalMap.outputMax));
}

TEST_CASE(
    "Binding defaults seed only a binding with no authored field",
    "[style][field-binding]") {
    auto binding = MappedBinding(-1, "");
    const std::vector fields{Field("Roughness"), Field("Interest")};

    invisible_places::style::EnsureFieldMappedBindingDefaults(
        &binding,
        fields,
        0.25F,
        0.75F);

    CHECK(binding.fieldMap.fieldSlot == 0);
    CHECK(binding.fieldMap.fieldName == "Roughness");
    CHECK(binding.fieldMap.outputMin == Catch::Approx(0.25F));
    CHECK(binding.fieldMap.outputMax == Catch::Approx(0.75F));

    // An unresolvable legacy slot with no name also seeds the default rather
    // than pointing at a column that does not exist.
    auto legacy = MappedBinding(9, "");
    invisible_places::style::EnsureFieldMappedBindingDefaults(
        &legacy,
        fields,
        0.0F,
        1.0F);
    CHECK(legacy.fieldMap.fieldSlot == 0);
    CHECK(legacy.fieldMap.fieldName == "Roughness");

    // No resident fields at all leaves the binding untouched.
    auto untouched = MappedBinding(3, "Interest");
    invisible_places::style::EnsureFieldMappedBindingDefaults(
        &untouched,
        {},
        0.0F,
        1.0F);
    CHECK(untouched.fieldMap.fieldSlot == 3);
    CHECK(untouched.fieldMap.fieldName == "Interest");
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

TEST_CASE("Scalar binding authoring comparison ignores runtime-only field resolution",
          "[style][field-binding][baseline]") {
    using invisible_places::style::FieldMapFlagClamp;
    using invisible_places::style::FieldMapFlagUseLayerStats;
    using invisible_places::style::ScalarRenderParameterBindingsAuthoringEqual;

    auto saved = MappedBinding(3, "Wetness");
    saved.fieldMap.flags = FieldMapFlagClamp | FieldMapFlagUseLayerStats;
    saved.fieldMap.boundsMemory = {
        {.fieldName = "Heat", .inputMin = -2.0F, .inputMax = 6.0F},
        {.fieldName = "Roughness", .inputMin = 0.25F, .inputMax = 0.75F},
    };
    auto live = saved;
    live.fieldMap.fieldName = "wetness";
    live.fieldMap.fieldSlot = 17;
    live.fieldMap.inputMin = -500.0F;
    live.fieldMap.inputMax = 900.0F;
    live.fieldMap.boundsMemory = {
        {.fieldName = "roughness", .inputMin = 0.25F, .inputMax = 0.75F},
        {.fieldName = "HEAT", .inputMin = -2.0F, .inputMax = 6.0F},
    };
    live.constantValue[1] = 99.0F;

    CHECK(ScalarRenderParameterBindingsAuthoringEqual(live, saved));
}

TEST_CASE("Scalar binding authoring comparison covers every retained editor setting",
          "[style][field-binding][baseline]") {
    using invisible_places::style::FieldMapFlagClamp;
    using invisible_places::style::FieldMapFlagInvert;
    using invisible_places::style::ParameterSourceMode;
    using invisible_places::style::ScalarRenderParameterBindingsAuthoringEqual;

    auto saved = MappedBinding(3, "Wetness");
    saved.fieldMap.flags = FieldMapFlagClamp;
    saved.fieldMap.boundsMemory = {
        {.fieldName = "Heat", .inputMin = -2.0F, .inputMax = 6.0F},
    };

    SECTION("active") {
        auto live = saved;
        live.active = !saved.active;
        CHECK_FALSE(ScalarRenderParameterBindingsAuthoringEqual(live, saved));
    }
    SECTION("mode") {
        auto live = saved;
        live.mode = ParameterSourceMode::Constant;
        CHECK_FALSE(ScalarRenderParameterBindingsAuthoringEqual(live, saved));
    }
    SECTION("constant fallback") {
        auto live = saved;
        invisible_places::style::SetScalarConstant(&live, 0.73F);
        CHECK_FALSE(ScalarRenderParameterBindingsAuthoringEqual(live, saved));
    }
    SECTION("durable field") {
        auto live = saved;
        live.fieldMap.fieldName = "Heat";
        CHECK_FALSE(ScalarRenderParameterBindingsAuthoringEqual(live, saved));
    }
    SECTION("manual input range") {
        auto live = saved;
        live.fieldMap.inputMin = -2.0F;
        CHECK_FALSE(ScalarRenderParameterBindingsAuthoringEqual(live, saved));
    }
    SECTION("output range") {
        auto live = saved;
        live.fieldMap.outputMax = 3.0F;
        CHECK_FALSE(ScalarRenderParameterBindingsAuthoringEqual(live, saved));
    }
    SECTION("gamma") {
        auto live = saved;
        live.fieldMap.gamma = 2.1F;
        CHECK_FALSE(ScalarRenderParameterBindingsAuthoringEqual(live, saved));
    }
    SECTION("clamp and invert flags") {
        auto live = saved;
        live.fieldMap.flags = FieldMapFlagClamp | FieldMapFlagInvert;
        CHECK_FALSE(ScalarRenderParameterBindingsAuthoringEqual(live, saved));
    }
    SECTION("layer stats mode") {
        auto live = saved;
        live.fieldMap.flags |=
            invisible_places::style::FieldMapFlagUseLayerStats;
        CHECK_FALSE(ScalarRenderParameterBindingsAuthoringEqual(live, saved));
    }
    SECTION("remembered field ranges") {
        auto live = saved;
        live.fieldMap.boundsMemory.front().inputMax = 7.0F;
        CHECK_FALSE(ScalarRenderParameterBindingsAuthoringEqual(live, saved));
    }
    SECTION("remembered field membership") {
        auto live = saved;
        live.fieldMap.boundsMemory.push_back(
            {.fieldName = "Roughness",
             .inputMin = 0.25F,
             .inputMax = 0.75F});
        CHECK_FALSE(ScalarRenderParameterBindingsAuthoringEqual(live, saved));
    }
    SECTION("remembered field identity") {
        auto live = saved;
        live.fieldMap.boundsMemory.front().fieldName = "Roughness";
        CHECK_FALSE(ScalarRenderParameterBindingsAuthoringEqual(live, saved));
    }
    SECTION("legacy slot identity") {
        auto left = saved;
        auto right = saved;
        left.fieldMap.fieldName.clear();
        right.fieldMap.fieldName.clear();
        left.fieldMap.fieldSlot = 1;
        right.fieldMap.fieldSlot = 2;
        CHECK_FALSE(ScalarRenderParameterBindingsAuthoringEqual(left, right));
    }
}

TEST_CASE("Scalar binding authoring comparison uses a display-scale epsilon",
          "[style][field-binding][baseline]") {
    using invisible_places::style::ScalarRenderParameterBindingsAuthoringEqual;

    const auto saved = MappedBinding(3, "Wetness");
    auto live = saved;
    live.fieldMap.outputMax += 5.0e-5F;
    CHECK(ScalarRenderParameterBindingsAuthoringEqual(live, saved));
    live.fieldMap.outputMax += 2.0e-4F;
    CHECK_FALSE(ScalarRenderParameterBindingsAuthoringEqual(live, saved));
}

TEST_CASE("Scalar binding saved-state text is complete and deterministic",
          "[style][field-binding][baseline]") {
    using invisible_places::style::DescribeScalarRenderParameterBindingAuthoringState;
    using invisible_places::style::FieldMapFlagInvert;

    auto mapped = MappedBinding(3, "Wetness");
    mapped.active = false;
    mapped.fieldMap.flags = FieldMapFlagInvert | (1U << 3U);
    mapped.fieldMap.boundsMemory = {
        {.fieldName = "Roughness", .inputMin = 0.25F, .inputMax = 0.75F},
        {.fieldName = "Heat", .inputMin = -2.0F, .inputMax = 6.0F},
    };
    CHECK(
        DescribeScalarRenderParameterBindingAuthoringState(mapped) ==
        "inactive; Field-Mapped field \"Wetness\"; input -3..9; output "
        "1.5..2.2; gamma 1.7; clamp off; invert on; retained constant 0.42; "
        "unknown flags 0x8; remembered bounds \"Heat\" -2..6, "
        "\"Roughness\" 0.25..0.75");

    invisible_places::style::RenderParameterBinding constant;
    invisible_places::style::SetScalarConstant(&constant, 0.25F);
    CHECK(
        DescribeScalarRenderParameterBindingAuthoringState(constant) ==
        "active; Constant 0.25; retained no field; input layer stats; "
        "output 0..1; gamma 1; clamp on; invert off; remembered bounds none");

    auto mappedStats = MappedBinding(4, "");
    mappedStats.fieldMap.flags =
        invisible_places::style::FieldMapFlagClamp |
        invisible_places::style::FieldMapFlagUseLayerStats;
    CHECK(
        DescribeScalarRenderParameterBindingAuthoringState(mappedStats) ==
        "active; Field-Mapped field slot 4; input layer stats; output "
        "1.5..2.2; gamma 1.7; clamp on; invert off; retained constant "
        "0.42; remembered bounds none");
}
