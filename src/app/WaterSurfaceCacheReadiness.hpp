#pragma once

#include "renderer/core/VulkanViewportShell.hpp"
#include "water/WaterSurfaceCache.hpp"

namespace invisible_places::app {

[[nodiscard]] inline bool WaterSurfaceResidentTablesReady(
    const invisible_places::water::WaterSurfaceCache* installedCache,
    const invisible_places::renderer::core::WaterSurfaceFlowGpuView& flowView,
    const invisible_places::renderer::core::WaterGroundFlowGpuView& groundView,
    bool uploadPending) {
    if (installedCache == nullptr || uploadPending) {
        return false;
    }

    const bool flowReady =
        installedCache->flowSurfaceSurfels.empty() ||
        (flowView.valid && flowView.preprocessingComplete &&
         flowView.cacheIdentity != nullptr &&
         *flowView.cacheIdentity == installedCache->cacheIdentity);
    const bool groundReady =
        installedCache->groundCells.empty() ||
        (groundView.valid && groundView.cacheIdentity != nullptr &&
         *groundView.cacheIdentity == installedCache->cacheIdentity);
    return flowReady && groundReady;
}

}  // namespace invisible_places::app
