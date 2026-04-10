#pragma once

namespace invisible_places::renderer::core {

struct RendererBootstrapConfig {
    bool enableValidationLayers = true;
    bool preferAdaptivePreview = true;
    bool allowOfflineTiling = true;
};

}  // namespace invisible_places::renderer::core

