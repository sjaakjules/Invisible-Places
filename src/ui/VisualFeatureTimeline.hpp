#pragma once

#include "timing/TimingColourise.hpp"

#include <cstdint>
#include <optional>

namespace invisible_places::ui {

// The Visual Feature editor presents several key families on one graph. Keys
// may snap to another key in their own family, but must never be pulled onto a
// different family's key merely because all visible curves share the graph.
enum class VisualFeatureTimelineSnapDomain : std::uint8_t {
    General = 0,
    Position,
    Fade,
    Skew,
    Palette,
    Intensity,
};

[[nodiscard]] constexpr VisualFeatureTimelineSnapDomain
VisualFeatureTimelineSnapDomainFor(
    std::optional<timing::TimingColouriseBoundsParameter> boundsParameter,
    std::optional<timing::TimingColouriseEffectParameter> effectParameter) {
    if (boundsParameter.has_value()) {
        switch (boundsParameter.value()) {
            case timing::TimingColouriseBoundsParameter::Lower:
            case timing::TimingColouriseBoundsParameter::Upper:
            case timing::TimingColouriseBoundsParameter::Centre:
            case timing::TimingColouriseBoundsParameter::Spread:
                return VisualFeatureTimelineSnapDomain::Position;
            case timing::TimingColouriseBoundsParameter::EdgeFade:
            case timing::TimingColouriseBoundsParameter::EdgeFadeLower:
            case timing::TimingColouriseBoundsParameter::EdgeFadeUpper:
                return VisualFeatureTimelineSnapDomain::Fade;
        }
    }
    if (effectParameter.has_value()) {
        switch (effectParameter.value()) {
            case timing::TimingColouriseEffectParameter::PaletteSkewCentre:
            case timing::TimingColouriseEffectParameter::PaletteSkewLower:
            case timing::TimingColouriseEffectParameter::PaletteSkewUpper:
            case timing::TimingColouriseEffectParameter::PaletteSkewSpread:
            case timing::TimingColouriseEffectParameter::EmissiveSkewCentre:
            case timing::TimingColouriseEffectParameter::EmissiveSkewSpread:
                return VisualFeatureTimelineSnapDomain::Skew;
            case timing::TimingColouriseEffectParameter::PalettePhase:
            case timing::TimingColouriseEffectParameter::AmountOverride:
            case timing::TimingColouriseEffectParameter::BlendMix:
                return VisualFeatureTimelineSnapDomain::Palette;
            case timing::TimingColouriseEffectParameter::EmissiveLevel:
                return VisualFeatureTimelineSnapDomain::Intensity;
        }
    }
    return VisualFeatureTimelineSnapDomain::General;
}

[[nodiscard]] constexpr bool VisualFeatureTimelineSnapDomainsCanSnap(
    VisualFeatureTimelineSnapDomain dragged,
    VisualFeatureTimelineSnapDomain target) {
    return dragged == target;
}

[[nodiscard]] constexpr bool VisualFeatureTimelineKeysCanSnap(
    std::optional<timing::TimingColouriseBoundsParameter> draggedBounds,
    std::optional<timing::TimingColouriseEffectParameter> draggedEffect,
    std::optional<timing::TimingColouriseBoundsParameter> targetBounds,
    std::optional<timing::TimingColouriseEffectParameter> targetEffect) {
    return VisualFeatureTimelineSnapDomainsCanSnap(
        VisualFeatureTimelineSnapDomainFor(
            draggedBounds,
            draggedEffect),
        VisualFeatureTimelineSnapDomainFor(
            targetBounds,
            targetEffect));
}

}  // namespace invisible_places::ui
