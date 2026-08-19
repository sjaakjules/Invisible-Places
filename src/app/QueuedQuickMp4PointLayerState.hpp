#pragma once

#include "renderer/core/VulkanViewportShell.hpp"

#include <cstddef>
#include <span>
#include <vector>

namespace invisible_places::app {

using QuickMp4PointCloudLayerState =
    invisible_places::renderer::core::SceneRenderState::PointCloudLayerState;

// Restores the queue-time layer inventory and immutable source metadata while
// retaining state that was deliberately rebuilt for one animation job. The
// selected Visual's effective styles are job-specific across every targeted
// scene sibling; every layer's draw count is animation-frustum-specific.
[[nodiscard]] bool RestoreQueuedQuickMp4PointLayerState(
    std::vector<QuickMp4PointCloudLayerState>* layers,
    std::span<const QuickMp4PointCloudLayerState> frozenBaseLayers,
    std::span<const std::size_t> requestVisualLayerIds);

}  // namespace invisible_places::app
