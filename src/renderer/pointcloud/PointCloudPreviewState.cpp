#include "renderer/pointcloud/PointCloudPreviewState.hpp"

#include <algorithm>
#include <numeric>

namespace invisible_places::renderer::pointcloud {

namespace {

std::uint64_t GreatestCommonDivisor(std::uint64_t left, std::uint64_t right) {
    while (right != 0) {
        const auto remainder = left % right;
        left = right;
        right = remainder;
    }

    return left;
}

std::uint64_t MakeRelativelyPrimeStep(std::uint64_t totalPoints) {
    if (totalPoints <= 1) {
        return 1;
    }

    std::uint64_t candidate = (totalPoints / 2U) | 1U;
    while (GreatestCommonDivisor(candidate, totalPoints) != 1U) {
        candidate += 2U;
    }

    return candidate;
}

}  // namespace

std::uint64_t ClampPointBudget(std::uint64_t totalPoints, std::uint64_t requestedPoints) {
    if (totalPoints == 0) {
        return 0;
    }

    if (requestedPoints == 0) {
        return 1;
    }

    return std::min(totalPoints, requestedPoints);
}

std::vector<std::uint32_t> GenerateDeterministicSampleIndices(
    std::uint64_t totalPoints,
    std::uint64_t requestedPoints) {
    const auto clampedRequested = ClampPointBudget(totalPoints, requestedPoints);
    if (clampedRequested == 0 || clampedRequested >= totalPoints) {
        return {};
    }

    std::vector<std::uint32_t> indices;
    indices.reserve(static_cast<std::size_t>(clampedRequested));

    const auto step = MakeRelativelyPrimeStep(totalPoints);
    const std::uint64_t seed = 0x9E3779B97F4A7C15ULL % totalPoints;

    for (std::uint64_t sampleIndex = 0; sampleIndex < clampedRequested; ++sampleIndex) {
        const auto pointIndex = (seed + (sampleIndex * step)) % totalPoints;
        indices.push_back(static_cast<std::uint32_t>(pointIndex));
    }

    return indices;
}

PointBudgetState MakePointBudgetState(std::uint64_t totalPoints, std::uint64_t requestedPoints) {
    PointBudgetState state;
    state.totalPoints = totalPoints;
    state.activePoints = ClampPointBudget(totalPoints, requestedPoints);

    if (state.totalPoints > 0) {
        state.activeFraction =
            static_cast<float>(state.activePoints) / static_cast<float>(state.totalPoints);
    }

    state.sampledIndices = GenerateDeterministicSampleIndices(totalPoints, state.activePoints);
    return state;
}

}  // namespace invisible_places::renderer::pointcloud
