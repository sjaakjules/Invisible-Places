#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>
#include <vector>

namespace invisible_places::renderer::core {

// A swapchain image is owned by one specific queue submission, not by the
// reusable fence handle associated with its frame slot.
struct SwapchainImageOwner {
    std::uint32_t frameSlot = std::numeric_limits<std::uint32_t>::max();
    std::uint64_t submissionSerial = 0U;

    [[nodiscard]] constexpr bool Valid() const {
        return submissionSerial != 0U &&
               frameSlot != std::numeric_limits<std::uint32_t>::max();
    }

    friend constexpr bool operator==(
        const SwapchainImageOwner&,
        const SwapchainImageOwner&) = default;
};

enum class SwapchainImageOwnerState {
    Unowned,
    Active,
    Completed,
    Stale,
};

[[nodiscard]] constexpr SwapchainImageOwnerState ClassifySwapchainImageOwner(
    const SwapchainImageOwner& owner,
    std::uint64_t activeSubmissionSerial,
    std::uint64_t completedSubmissionSerial) {
    if (!owner.Valid()) {
        return SwapchainImageOwnerState::Unowned;
    }
    if (owner.submissionSerial <= completedSubmissionSerial) {
        return SwapchainImageOwnerState::Completed;
    }
    if (owner.submissionSerial == activeSubmissionSerial) {
        return SwapchainImageOwnerState::Active;
    }
    return SwapchainImageOwnerState::Stale;
}

inline std::uint32_t ClearCompletedImageOwners(
    std::vector<SwapchainImageOwner>* owners,
    std::uint32_t completedFrameSlot,
    std::uint64_t completedSubmissionSerial) {
    if (owners == nullptr || completedSubmissionSerial == 0U) {
        return 0U;
    }
    std::uint32_t cleared = 0U;
    for (auto& owner : *owners) {
        if (owner.frameSlot == completedFrameSlot &&
            owner.submissionSerial == completedSubmissionSerial) {
            owner = {};
            ++cleared;
        }
    }
    return cleared;
}

[[nodiscard]] constexpr std::uint64_t TimestampMask(
    std::uint32_t timestampValidBits) {
    if (timestampValidBits == 0U) {
        return 0U;
    }
    if (timestampValidBits >= 64U) {
        return std::numeric_limits<std::uint64_t>::max();
    }
    return (std::uint64_t{1U} << timestampValidBits) - 1U;
}

[[nodiscard]] constexpr std::uint64_t TimestampTickDelta(
    std::uint64_t start,
    std::uint64_t end,
    std::uint32_t timestampValidBits) {
    const std::uint64_t mask = TimestampMask(timestampValidBits);
    if (mask == 0U) {
        return 0U;
    }
    start &= mask;
    end &= mask;
    return (end - start) & mask;
}

[[nodiscard]] constexpr double TimestampDeltaMilliseconds(
    std::uint64_t start,
    std::uint64_t end,
    std::uint32_t timestampValidBits,
    double timestampPeriodNanoseconds) {
    if (!(timestampPeriodNanoseconds > 0.0)) {
        return 0.0;
    }
    return static_cast<double>(
               TimestampTickDelta(start, end, timestampValidBits)) *
           timestampPeriodNanoseconds / 1'000'000.0;
}

struct TimestampQueryResult {
    std::uint64_t value = 0U;
    std::uint64_t available = 0U;
};

[[nodiscard]] inline bool TimestampResultsAvailable(
    std::span<const TimestampQueryResult> results) {
    return !results.empty() &&
           std::all_of(
               results.begin(),
               results.end(),
               [](const TimestampQueryResult& result) {
                   return result.available != 0U;
               });
}

template <std::size_t PhaseCount>
class RollingPhaseAverages {
  public:
    void AddFrame(
        const std::array<double, PhaseCount>& milliseconds,
        const std::array<bool, PhaseCount>& active,
        double elapsedMilliseconds) {
        for (std::size_t index = 0U; index < PhaseCount; ++index) {
            if (!active[index]) {
                continue;
            }
            sums_[index] += milliseconds[index];
            ++sampleCounts_[index];
        }
        elapsedMilliseconds_ += std::max(0.0, elapsedMilliseconds);
        ++frameCount_;
    }

    [[nodiscard]] bool PublishIfReady(
        double windowMilliseconds = 500.0) {
        if (frameCount_ == 0U ||
            elapsedMilliseconds_ <
                std::max(0.0, windowMilliseconds)) {
            return false;
        }
        for (std::size_t index = 0U; index < PhaseCount; ++index) {
            if (sampleCounts_[index] == 0U) {
                publishedActive_[index] = false;
                continue;
            }
            publishedAverages_[index] =
                sums_[index] /
                static_cast<double>(sampleCounts_[index]);
            publishedActive_[index] = true;
        }
        sums_.fill(0.0);
        sampleCounts_.fill(0U);
        elapsedMilliseconds_ = 0.0;
        frameCount_ = 0U;
        return true;
    }

    void Reset() {
        sums_.fill(0.0);
        sampleCounts_.fill(0U);
        publishedAverages_.fill(0.0);
        publishedActive_.fill(false);
        elapsedMilliseconds_ = 0.0;
        frameCount_ = 0U;
    }

    [[nodiscard]] double PublishedAverage(
        std::size_t phaseIndex) const {
        return phaseIndex < PhaseCount
                   ? publishedAverages_[phaseIndex]
                   : 0.0;
    }

    [[nodiscard]] bool PublishedActive(
        std::size_t phaseIndex) const {
        return phaseIndex < PhaseCount &&
               publishedActive_[phaseIndex];
    }

    [[nodiscard]] std::uint32_t PendingSampleCount(
        std::size_t phaseIndex) const {
        return phaseIndex < PhaseCount
                   ? sampleCounts_[phaseIndex]
                   : 0U;
    }

  private:
    std::array<double, PhaseCount> sums_{};
    std::array<std::uint32_t, PhaseCount> sampleCounts_{};
    std::array<double, PhaseCount> publishedAverages_{};
    std::array<bool, PhaseCount> publishedActive_{};
    double elapsedMilliseconds_ = 0.0;
    std::uint32_t frameCount_ = 0U;
};

}  // namespace invisible_places::renderer::core
