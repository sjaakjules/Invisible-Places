#pragma once

namespace invisible_places::ui {

enum class SidePanelMode {
    Collapsed,
    RevealOnHover,
    Expanded,
    Pinned
};

struct SidePanelState {
    bool pinned = false;
    float panelWidth = 360.0F;
    float handleWidth = 18.0F;
    float revealAmount = 0.0F;
    bool hovered = false;
    SidePanelMode mode = SidePanelMode::Collapsed;
};

}  // namespace invisible_places::ui
