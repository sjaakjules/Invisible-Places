#include "app/QueuedQuickMp4PointLayerState.hpp"

#include <algorithm>
#include <optional>
#include <utility>

namespace invisible_places::app {

bool RestoreQueuedQuickMp4PointLayerState(
    std::vector<QuickMp4PointCloudLayerState>* layers,
    std::span<const QuickMp4PointCloudLayerState> frozenBaseLayers,
    std::span<const std::size_t> requestVisualLayerIds) {
    if (layers == nullptr) {
        return false;
    }

    layers->erase(
        std::remove_if(
            layers->begin(),
            layers->end(),
            [&](const auto& layer) {
                return std::none_of(
                    frozenBaseLayers.begin(),
                    frozenBaseLayers.end(),
                    [&](const auto& frozenLayer) {
                        return frozenLayer.layerId == layer.layerId;
                    });
            }),
        layers->end());

    // A queue-time layer that is no longer resident cannot be reconstructed
    // safely from a different live session. Reject the job before mutating
    // styles so its render and sidecar cannot describe different inventories.
    if (layers->size() != frozenBaseLayers.size() ||
        std::any_of(
            frozenBaseLayers.begin(),
            frozenBaseLayers.end(),
            [&](const auto& frozenLayer) {
                return std::count_if(
                           layers->begin(),
                           layers->end(),
                           [&](const auto& layer) {
                               return layer.layerId == frozenLayer.layerId;
                           }) != 1;
            })) {
        return false;
    }

    for (auto& layer : *layers) {
        const auto frozen = std::find_if(
            frozenBaseLayers.begin(),
            frozenBaseLayers.end(),
            [&](const auto& candidate) {
                return candidate.layerId == layer.layerId;
            });
        if (frozen == frozenBaseLayers.end()) {
            return false;
        }

        const auto drawPointCount = layer.drawPointCount;
        std::optional<invisible_places::renderer::pointcloud::
                          PointCloudStyleState>
            requestVisualStyle;
        if (std::find(
                requestVisualLayerIds.begin(),
                requestVisualLayerIds.end(),
                layer.layerId) != requestVisualLayerIds.end()) {
            requestVisualStyle = std::move(layer.style);
        }

        layer = *frozen;

        // Frustum selection is specific to each animation, so retain the draw
        // count derived when this job was started. Every target scene layer's
        // complete effective style includes all named-Visual authoring plus
        // its role, renderer-mode, and Water-activity derivations.
        layer.drawPointCount = drawPointCount;
        if (requestVisualStyle.has_value()) {
            layer.style = std::move(requestVisualStyle.value());
        }
    }
    return true;
}

}  // namespace invisible_places::app
