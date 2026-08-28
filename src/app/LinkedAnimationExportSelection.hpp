#pragma once

#include <array>
#include <cstddef>

namespace invisible_places::app {

// Linked members are always addressed in the reciprocal transport's stable,
// path-sorted order. The labels A and B therefore do not change when the
// other member happens to be the animation loaded in the editor.
enum class LinkedAnimationExportSelection {
    A,
    B,
    Both,
};

struct LinkedAnimationExportMemberPlan {
    std::array<std::size_t, 2U> memberIndices{};
    std::size_t count = 0U;
};

[[nodiscard]] constexpr LinkedAnimationExportMemberPlan
ResolveLinkedAnimationExportMemberPlan(
    LinkedAnimationExportSelection selection) {
    switch (selection) {
        case LinkedAnimationExportSelection::A:
            return {.memberIndices = {0U, 0U}, .count = 1U};
        case LinkedAnimationExportSelection::B:
            return {.memberIndices = {1U, 0U}, .count = 1U};
        case LinkedAnimationExportSelection::Both:
            return {.memberIndices = {0U, 1U}, .count = 2U};
    }
    return {};
}

}  // namespace invisible_places::app
