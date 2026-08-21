#pragma once

#include <cstddef>
#include <utility>
#include <vector>

namespace invisible_places::app::animation_registry {

// Moves one row to the destination row, preserving the relative order of
// every row it crosses. This is intentionally generic because the animation
// registry is stored as several parallel vectors that must move together.
template <typename T>
bool MoveRow(
    std::vector<T>* rows,
    std::size_t sourceIndex,
    std::size_t destinationIndex) {
    if (rows == nullptr || sourceIndex >= rows->size() ||
        destinationIndex >= rows->size() ||
        sourceIndex == destinationIndex) {
        return false;
    }
    typename std::vector<T>::value_type moved =
        std::move((*rows)[sourceIndex]);
    rows->erase(
        rows->begin() + static_cast<std::ptrdiff_t>(sourceIndex));
    rows->insert(
        rows->begin() + static_cast<std::ptrdiff_t>(destinationIndex),
        std::move(moved));
    return true;
}

// Keeps index-based UI state attached to the same animation after MoveRow.
[[nodiscard]] constexpr std::size_t RemapIndexAfterMove(
    std::size_t index,
    std::size_t sourceIndex,
    std::size_t destinationIndex) {
    if (index == sourceIndex) {
        return destinationIndex;
    }
    if (sourceIndex < destinationIndex && index > sourceIndex &&
        index <= destinationIndex) {
        return index - 1U;
    }
    if (destinationIndex < sourceIndex && index >= destinationIndex &&
        index < sourceIndex) {
        return index + 1U;
    }
    return index;
}

}  // namespace invisible_places::app::animation_registry
