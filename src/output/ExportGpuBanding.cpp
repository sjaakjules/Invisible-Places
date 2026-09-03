#include "output/ExportGpuBanding.hpp"

#include <algorithm>
#include <cmath>

namespace invisible_places::output {

namespace {

constexpr std::uint32_t kMaximumAutomaticBandCount = 3U;
constexpr std::uint32_t kSlowSamplesBeforeSplit = 2U;
constexpr std::uint32_t kFastSamplesBeforeMerge = 4U;
constexpr double kMaximumTargetSubmissionSeconds = 1.5;
constexpr double kFastBandedFrameSeconds = 1.2;

}  // namespace

void PrimeExportGpuBanding(
    ExportGpuBandingState* state,
    bool containsWorldSurfels) {
    if (state == nullptr || state->primed) {
        return;
    }
    state->primed = true;
    state->bandCount = containsWorldSurfels ? 2U : 1U;
}

void ObserveExportGpuSample(
    ExportGpuBandingState* state,
    double gpuSeconds) {
    if (state == nullptr || !std::isfinite(gpuSeconds) || gpuSeconds < 0.0) {
        return;
    }

    ++state->completedSamples;
    if (state->completedSamples == 1U) {
        // First-frame sorting, allocation, and pipeline warm-up can be tens of
        // seconds even when every following frame is comfortably sub-second.
        state->consecutiveSlowSamples = 0U;
        state->consecutiveFastSamples = 0U;
        return;
    }

    const auto bands = std::max(1U, state->bandCount);
    const double approximateSubmissionSeconds =
        gpuSeconds / static_cast<double>(bands);
    if (approximateSubmissionSeconds > kMaximumTargetSubmissionSeconds &&
        state->bandCount < kMaximumAutomaticBandCount) {
        ++state->consecutiveSlowSamples;
        state->consecutiveFastSamples = 0U;
        if (state->consecutiveSlowSamples >= kSlowSamplesBeforeSplit) {
            ++state->bandCount;
            state->consecutiveSlowSamples = 0U;
        }
        return;
    }
    state->consecutiveSlowSamples = 0U;

    // A banded frame cannot be faster than its equivalent unbanded frame: it
    // repeats vertex work and command recording. Therefore a fast *total*
    // banded time is a safe signal to remove a split without scheduling a
    // deliberately slow probe frame.
    if (state->bandCount > 1U && gpuSeconds < kFastBandedFrameSeconds) {
        ++state->consecutiveFastSamples;
        if (state->consecutiveFastSamples >= kFastSamplesBeforeMerge) {
            --state->bandCount;
            state->consecutiveFastSamples = 0U;
        }
    } else {
        state->consecutiveFastSamples = 0U;
    }
}

std::uint32_t ExportGpuBandRows(
    const ExportGpuBandingState& state,
    std::uint32_t frameHeight) {
    const auto bands = std::max(1U, state.bandCount);
    if (bands <= 1U || frameHeight <= 1U) {
        return 0U;
    }
    return (frameHeight + bands - 1U) / bands;
}

}  // namespace invisible_places::output
