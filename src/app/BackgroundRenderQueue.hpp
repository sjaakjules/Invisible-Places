#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

namespace invisible_places::app {

// Filesystem/process observations are separated from queue policy so durable
// predecessor handling can be tested without launching workers. A missing or
// newly-starting status remains blocking. A dead worker is released only once
// its heartbeat is stale, preventing both premature overlap and permanent
// queue starvation.
struct BackgroundRenderDependencyObservation {
    bool statusLoaded = false;
    bool terminal = false;
    std::int64_t processId = 0;
    bool processAlive = false;
    bool statusFresh = false;
};

[[nodiscard]] bool BackgroundRenderDependencyBlocks(
    const BackgroundRenderDependencyObservation& observation);

[[nodiscard]] std::size_t CountBlockingBackgroundRenderDependencies(
    std::span<const BackgroundRenderDependencyObservation> observations);

}  // namespace invisible_places::app
