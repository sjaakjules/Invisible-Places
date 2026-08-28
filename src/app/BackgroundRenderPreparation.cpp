#include "app/BackgroundRenderPreparation.hpp"

namespace invisible_places::app {

bool BackgroundRenderPreparationReady(
    const BackgroundRenderPreparationObservation& observation) {
    return !observation.layerLoadActive &&
           !observation.scalarFieldLoadActive &&
           observation.queuedLayerLoadCount == 0U &&
           !observation.waterSurfaceWarmupActive &&
           !observation.waterSurfacePreprocessPending &&
           observation.initialFlowRefreshRequested &&
           observation.flowJobsSettled &&
           !observation.seepageSupportActive &&
           observation.readySeepageTopologyCount >=
               observation.requiredSeepageTopologyCount;
}

}  // namespace invisible_places::app
