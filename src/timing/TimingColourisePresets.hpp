#pragma once

#include "timing/TimingColourise.hpp"

#include <optional>
#include <span>
#include <string_view>

namespace invisible_places::timing {

inline constexpr std::size_t kBuiltInTimingColourisePalettePresetCount = 25U;

// Built-in palettes are deliberately compact approximations of familiar
// scientific colour maps. The returned catalog has static lifetime and is
// read-only; copy a palette before exposing it to editing controls.
[[nodiscard]] std::span<const TimingColourisePaletteDefinition>
BuiltInTimingColourisePalettePresets();

[[nodiscard]] const TimingColourisePaletteDefinition*
FindBuiltInTimingColourisePalettePreset(std::string_view presetId);

// Returns an independent snapshot. Mutating the result never changes the
// built-in preset or a palette copied for another Colourise effect.
[[nodiscard]] std::optional<TimingColourisePalette>
CopyBuiltInTimingColourisePalettePreset(std::string_view presetId);

}  // namespace invisible_places::timing
