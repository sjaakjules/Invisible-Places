#include "ui/CompositionGuide.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

TEST_CASE("Composition guide combines thirds with an emphasized halfway line", "[composition-guide]") {
    const auto lines = invisible_places::ui::BuildCompositionGuideLines(
        true,
        true);

    REQUIRE(lines.count == 3U);
    CHECK(lines.values[0].normalizedPosition == Catch::Approx(1.0F / 3.0F));
    CHECK_FALSE(lines.values[0].halfway);
    CHECK(lines.values[1].normalizedPosition == Catch::Approx(0.5F));
    CHECK(lines.values[1].halfway);
    CHECK(lines.values[2].normalizedPosition == Catch::Approx(2.0F / 3.0F));
    CHECK_FALSE(lines.values[2].halfway);
}

TEST_CASE("Composition guide line groups can be shown independently", "[composition-guide]") {
    SECTION("halfway only") {
        const auto lines = invisible_places::ui::BuildCompositionGuideLines(
            true,
            false);

        REQUIRE(lines.count == 1U);
        CHECK(lines.values[0].normalizedPosition == Catch::Approx(0.5F));
        CHECK(lines.values[0].halfway);
    }

    SECTION("thirds only") {
        const auto lines = invisible_places::ui::BuildCompositionGuideLines(
            false,
            true);

        REQUIRE(lines.count == 2U);
        CHECK(lines.values[0].normalizedPosition == Catch::Approx(1.0F / 3.0F));
        CHECK(lines.values[1].normalizedPosition == Catch::Approx(2.0F / 3.0F));
        CHECK_FALSE(lines.values[0].halfway);
        CHECK_FALSE(lines.values[1].halfway);
    }

    SECTION("neither group") {
        const auto lines = invisible_places::ui::BuildCompositionGuideLines(
            false,
            false);

        CHECK(lines.count == 0U);
    }
}
