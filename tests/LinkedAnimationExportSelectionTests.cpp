#include "app/LinkedAnimationExportSelection.hpp"

#include <catch2/catch_test_macros.hpp>

TEST_CASE(
    "Linked animation export selection uses stable canonical member order",
    "[app][animation][linked][export]") {
    using invisible_places::app::LinkedAnimationExportSelection;
    using invisible_places::app::ResolveLinkedAnimationExportMemberPlan;

    const auto a = ResolveLinkedAnimationExportMemberPlan(
        LinkedAnimationExportSelection::A);
    REQUIRE(a.count == 1U);
    CHECK(a.memberIndices[0] == 0U);

    const auto b = ResolveLinkedAnimationExportMemberPlan(
        LinkedAnimationExportSelection::B);
    REQUIRE(b.count == 1U);
    CHECK(b.memberIndices[0] == 1U);

    const auto both = ResolveLinkedAnimationExportMemberPlan(
        LinkedAnimationExportSelection::Both);
    REQUIRE(both.count == 2U);
    CHECK(both.memberIndices[0] == 0U);
    CHECK(both.memberIndices[1] == 1U);
}
