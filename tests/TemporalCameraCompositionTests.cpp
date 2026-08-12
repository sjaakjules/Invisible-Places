#include "renderer/core/TemporalCameraComposition.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

TEST_CASE("Temporal camera overlap preserves straight alpha", "[camera][seam][overlap]") {
    const std::array<float, 4> redTop{1.0F, 0.0F, 0.0F, 0.5F};
    const std::array<float, 4> blueBottom{0.0F, 0.0F, 1.0F, 0.5F};

    const auto result = invisible_places::renderer::core::
        CompositeStraightAlphaOver(redTop, blueBottom);

    CHECK(result[0] == Catch::Approx(2.0F / 3.0F));
    CHECK(result[1] == Catch::Approx(0.0F));
    CHECK(result[2] == Catch::Approx(1.0F / 3.0F));
    CHECK(result[3] == Catch::Approx(0.75F));
}

TEST_CASE("Temporal camera overlap applies opacity only to its chosen top layer", "[camera][seam][overlap]") {
    const std::array<float, 4> opaqueTop{1.0F, 0.0F, 0.0F, 1.0F};
    const std::array<float, 4> opaqueBottom{0.0F, 0.0F, 1.0F, 1.0F};

    const auto halfTop = invisible_places::renderer::core::
        CompositeStraightAlphaOver(
            opaqueTop,
            opaqueBottom,
            0.5F);
    CHECK(halfTop[0] == Catch::Approx(0.5F));
    CHECK(halfTop[1] == Catch::Approx(0.0F));
    CHECK(halfTop[2] == Catch::Approx(0.5F));
    CHECK(halfTop[3] == Catch::Approx(1.0F));

    const auto transparentTop = invisible_places::renderer::core::
        CompositeStraightAlphaOver(
            opaqueTop,
            opaqueBottom,
            0.0F);
    CHECK(transparentTop == opaqueBottom);
}

TEST_CASE("Temporal camera overlap keeps filename layer order when A and B switch", "[camera][seam][overlap]") {
    using invisible_places::renderer::core::CurrentSourceIsSelectedTop;

    CHECK(CurrentSourceIsSelectedTop(true, true));
    CHECK_FALSE(CurrentSourceIsSelectedTop(false, true));
    CHECK_FALSE(CurrentSourceIsSelectedTop(true, false));
    CHECK(CurrentSourceIsSelectedTop(false, false));
}
