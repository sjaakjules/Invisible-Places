#pragma once

#include <cstdint>

namespace invisible_places::output {

// Export rendering shares the graphics queue with the live viewport. Splitting
// a frame into row bands gives the viewport opportunities to submit work, but
// every band also replays the complete point-cloud vertex stage. Keep the
// controller deliberately conservative so responsive rendering does not turn
// into an N-times vertex-work regression.
struct ExportGpuBandingState {
    std::uint32_t bandCount = 1U;
    std::uint32_t completedSamples = 0U;
    std::uint32_t consecutiveSlowSamples = 0U;
    std::uint32_t consecutiveFastSamples = 0U;
    bool primed = false;
};

// World surfels expand every point to a four-vertex strip and can create
// long first
// submissions. They start with one protective split; screen sprites start on
// the fastest single-submit path.
void PrimeExportGpuBanding(
    ExportGpuBandingState* state,
    bool containsWorldSurfels);

// Learns from warm GPU samples. The cold first sample is ignored because sort
// allocation, pipeline warm-up, and residency work are not representative of
// steady animation frames.
void ObserveExportGpuSample(
    ExportGpuBandingState* state,
    double gpuSeconds);

// Zero preserves the historical single full-frame submission.
[[nodiscard]] std::uint32_t ExportGpuBandRows(
    const ExportGpuBandingState& state,
    std::uint32_t frameHeight);

}  // namespace invisible_places::output
