#include "app/AnimationRegistryOrder.hpp"

#include <catch2/catch_test_macros.hpp>

#include <string>
#include <vector>

TEST_CASE(
    "Animation registry rows move without separating parallel state",
    "[app][animation][registry][order]") {
    using invisible_places::app::animation_registry::MoveRow;
    using invisible_places::app::animation_registry::RemapIndexAfterMove;

    SECTION("Moving a used animation to the top shifts intervening rows") {
        std::vector<std::string> files{"A", "B", "C", "D"};
        std::vector<bool> dirty{false, true, false, true};

        REQUIRE(MoveRow(&files, 3U, 0U));
        REQUIRE(MoveRow(&dirty, 3U, 0U));
        CHECK(files == std::vector<std::string>{"D", "A", "B", "C"});
        CHECK(dirty == std::vector<bool>{true, false, true, false});
        CHECK(RemapIndexAfterMove(3U, 3U, 0U) == 0U);
        CHECK(RemapIndexAfterMove(0U, 3U, 0U) == 1U);
        CHECK(RemapIndexAfterMove(2U, 3U, 0U) == 3U);
    }

    SECTION("Moving down places the source on the target row") {
        std::vector<std::string> files{"A", "B", "C", "D"};

        REQUIRE(MoveRow(&files, 0U, 2U));
        CHECK(files == std::vector<std::string>{"B", "C", "A", "D"});
        CHECK(RemapIndexAfterMove(0U, 0U, 2U) == 2U);
        CHECK(RemapIndexAfterMove(1U, 0U, 2U) == 0U);
        CHECK(RemapIndexAfterMove(2U, 0U, 2U) == 1U);
        CHECK(RemapIndexAfterMove(3U, 0U, 2U) == 3U);
    }

    SECTION("Invalid and no-op moves leave rows untouched") {
        std::vector<std::string> files{"A", "B"};

        CHECK_FALSE(MoveRow(&files, 0U, 0U));
        CHECK_FALSE(MoveRow(&files, 2U, 0U));
        CHECK(files == std::vector<std::string>{"A", "B"});
    }
}
