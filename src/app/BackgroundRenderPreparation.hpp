#pragma once

#include <cstddef>

namespace invisible_places::app {

// Detached workers reconstruct derived Water resources from the immutable
// authored package before taking the export-layer snapshot. Keep this policy
// independent from PreviewRuntimeState so a worker cannot accidentally regress
// to waiting only for ordinary point-cloud loads.
struct BackgroundRenderPreparationObservation {
    bool layerLoadActive = false;
    bool scalarFieldLoadActive = false;
    std::size_t queuedLayerLoadCount = 0U;
    bool waterSurfaceWarmupActive = false;
    bool waterSurfacePreprocessPending = false;
    bool initialFlowRefreshRequested = false;
    bool flowJobsSettled = false;
    bool seepageSupportActive = false;
    std::size_t requiredSeepageTopologyCount = 0U;
    std::size_t readySeepageTopologyCount = 0U;
};

[[nodiscard]] bool BackgroundRenderPreparationReady(
    const BackgroundRenderPreparationObservation& observation);

}  // namespace invisible_places::app
