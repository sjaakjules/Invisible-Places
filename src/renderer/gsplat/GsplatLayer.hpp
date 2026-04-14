#pragma once

#include "io/TransformMatrix.hpp"

#include <array>
#include <filesystem>
#include <string>

namespace invisible_places::renderer::gsplat {

enum class GaussianSplatColorMode {
    FullSh,
    DcOnly
};

enum class GaussianSplatDebugMode {
    Final,
    Opacity,
    Scale,
    Depth,
    LayerTint
};

enum class GaussianSplatQualityMode {
    Fast,
    Medium,
    SurfaceGuided,
    High
};

inline GaussianSplatQualityMode ResolveEffectiveGaussianSplatQualityMode(
    GaussianSplatQualityMode requestedMode,
    bool autoLowerDuringNavigation,
    bool navigationActive) {
    if (!autoLowerDuringNavigation || !navigationActive) {
        return requestedMode;
    }

    switch (requestedMode) {
        case GaussianSplatQualityMode::Fast:
            return GaussianSplatQualityMode::Fast;
        case GaussianSplatQualityMode::Medium:
            return GaussianSplatQualityMode::Fast;
        case GaussianSplatQualityMode::SurfaceGuided:
            return GaussianSplatQualityMode::Medium;
        case GaussianSplatQualityMode::High:
            return GaussianSplatQualityMode::SurfaceGuided;
    }

    return requestedMode;
}

struct GaussianSplatStyleState {
    bool transformEnabled = true;
    GaussianSplatColorMode colorMode = GaussianSplatColorMode::FullSh;
    GaussianSplatDebugMode debugMode = GaussianSplatDebugMode::Final;
    GaussianSplatQualityMode qualityMode = GaussianSplatQualityMode::Fast;
    float opacityMultiplier = 1.0F;
    float scaleMultiplier = 1.0F;
    float exposure = 1.0F;
    float saturation = 1.0F;
    std::array<float, 4> layerTint{0.94F, 0.82F, 0.60F, 1.0F};
};

struct GsplatLayer {
    std::string layerName;
    std::filesystem::path assetPath;
    invisible_places::io::Matrix4d localToWorld;
    bool visible = true;
    GaussianSplatStyleState style{};
};

}  // namespace invisible_places::renderer::gsplat
