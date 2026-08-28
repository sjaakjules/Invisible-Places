#include "app/BackgroundRenderQueue.hpp"

#include <algorithm>

namespace invisible_places::app {

bool BackgroundRenderDependencyBlocks(
    const BackgroundRenderDependencyObservation& observation) {
    if (observation.statusLoaded && observation.terminal) {
        return false;
    }
    if (observation.statusLoaded && observation.processId > 0 &&
        !observation.processAlive && !observation.statusFresh) {
        return false;
    }
    return true;
}

std::size_t CountBlockingBackgroundRenderDependencies(
    std::span<const BackgroundRenderDependencyObservation> observations) {
    return static_cast<std::size_t>(std::count_if(
        observations.begin(),
        observations.end(),
        BackgroundRenderDependencyBlocks));
}

}  // namespace invisible_places::app
