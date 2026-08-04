#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <numeric>
#include <span>
#include <string>
#include <vector>

namespace invisible_places::app {

// One resident scalar field considered for eviction. lastRequiredTick is
// the residency sweep's monotonic clock value from the last sweep whose
// required-field set contained this field; fields the authored state still
// references keep a fresh tick and are additionally marked required.
// Disk-backed fields (a non-negative sourceIndex) are evictable because the
// on-demand loader can stream them back; runtime-generated fields
// (water_effect_*, ripple_*) are not.
struct ScalarFieldResidencyCandidate {
    std::size_t sessionIndex = 0;
    std::string fieldName;
    std::uint64_t bytes = 0;
    std::uint64_t lastRequiredTick = 0;
    bool required = false;
    bool evictable = false;
};

// Pure eviction policy: given every resident field and the current/budget
// byte totals, returns indices into `candidates` to evict —
// least-recently-required first — until the projected total drops to the
// budget or no legal candidate remains. A zero budget disables eviction.
// Required and non-evictable fields are never selected.
inline std::vector<std::size_t> SelectScalarFieldEvictions(
    std::span<const ScalarFieldResidencyCandidate> candidates,
    std::uint64_t currentBytes,
    std::uint64_t budgetBytes) {
    if (budgetBytes == 0U || currentBytes <= budgetBytes) {
        return {};
    }
    std::vector<std::size_t> order;
    order.reserve(candidates.size());
    for (std::size_t index = 0; index < candidates.size(); ++index) {
        const auto& candidate = candidates[index];
        if (candidate.evictable && !candidate.required) {
            order.push_back(index);
        }
    }
    std::stable_sort(
        order.begin(),
        order.end(),
        [&candidates](std::size_t left, std::size_t right) {
            return candidates[left].lastRequiredTick <
                   candidates[right].lastRequiredTick;
        });
    std::vector<std::size_t> selected;
    std::uint64_t projected = currentBytes;
    for (const auto index : order) {
        if (projected <= budgetBytes) {
            break;
        }
        selected.push_back(index);
        const auto bytes = candidates[index].bytes;
        projected = bytes > projected ? 0U : projected - bytes;
    }
    return selected;
}

}  // namespace invisible_places::app
