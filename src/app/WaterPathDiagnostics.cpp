#include "app/WaterPathDiagnostics.hpp"

namespace invisible_places::app {

namespace {

constexpr std::array kWaterPathDiagnosticColorModes{
    WaterPathDiagnosticColorMode::Branch,
    WaterPathDiagnosticColorMode::Slope,
    WaterPathDiagnosticColorMode::Flatness,
    WaterPathDiagnosticColorMode::Curvature,
    WaterPathDiagnosticColorMode::NeighborDensity,
    WaterPathDiagnosticColorMode::Confluence,
    WaterPathDiagnosticColorMode::ChannelWidth,
    WaterPathDiagnosticColorMode::Speed,
    WaterPathDiagnosticColorMode::Turbulence,
    WaterPathDiagnosticColorMode::Eddies,
    WaterPathDiagnosticColorMode::Ripples,
};

}  // namespace

std::span<const WaterPathDiagnosticColorMode> AllWaterPathDiagnosticColorModes() {
    return kWaterPathDiagnosticColorModes;
}

const char* WaterPathDiagnosticColorModeLabel(WaterPathDiagnosticColorMode mode) {
    switch (mode) {
        case WaterPathDiagnosticColorMode::Branch:
            return "Branch";
        case WaterPathDiagnosticColorMode::Slope:
            return "Slope";
        case WaterPathDiagnosticColorMode::Flatness:
            return "Flatness";
        case WaterPathDiagnosticColorMode::Curvature:
            return "Curvature";
        case WaterPathDiagnosticColorMode::NeighborDensity:
            return "Neighbor Density";
        case WaterPathDiagnosticColorMode::Confluence:
            return "Confluence";
        case WaterPathDiagnosticColorMode::ChannelWidth:
            return "Channel Width";
        case WaterPathDiagnosticColorMode::Speed:
            return "Speed";
        case WaterPathDiagnosticColorMode::Turbulence:
            return "Turbulence";
        case WaterPathDiagnosticColorMode::Eddies:
            return "Eddies";
        case WaterPathDiagnosticColorMode::Ripples:
            return "Ripples";
    }
    return "Branch";
}

bool WaterPathDiagnosticRebuildCountersEqual(
    const WaterPathDiagnosticRebuildCounters& left,
    const WaterPathDiagnosticRebuildCounters& right) {
    return left.pathBakes == right.pathBakes &&
           left.laneBuilds == right.laneBuilds &&
           left.trailBuilds == right.trailBuilds;
}

WaterPathDiagnosticModeChangeStats RecordWaterPathDiagnosticModeChange(
    WaterPathDiagnosticModeChangeStats previous,
    const WaterPathDiagnosticRebuildCounters& before,
    const WaterPathDiagnosticRebuildCounters& after,
    std::chrono::steady_clock::duration latency) {
    previous.changeCount += 1U;
    previous.lastLatencyMs = std::chrono::duration<double, std::milli>(latency).count();
    previous.lastChangeTouchedRebuildCounters =
        !WaterPathDiagnosticRebuildCountersEqual(before, after);
    previous.before = before;
    previous.after = after;
    return previous;
}

}  // namespace invisible_places::app
