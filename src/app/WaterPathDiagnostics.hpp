#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <span>

namespace invisible_places::app {

enum class WaterPathDiagnosticColorMode {
    Branch,
    Slope,
    Flatness,
    Curvature,
    NeighborDensity,
    Confluence,
    ChannelWidth,
    Speed,
    Turbulence,
    Eddies,
    Ripples
};

struct WaterPathDiagnosticRebuildCounters {
    std::uint64_t pathBakes = 0;
    std::uint64_t laneBuilds = 0;
    std::uint64_t trailBuilds = 0;
};

struct WaterPathDiagnosticModeChangeStats {
    std::uint64_t changeCount = 0;
    double lastLatencyMs = 0.0;
    bool lastChangeTouchedRebuildCounters = false;
    WaterPathDiagnosticRebuildCounters before{};
    WaterPathDiagnosticRebuildCounters after{};
};

[[nodiscard]] std::span<const WaterPathDiagnosticColorMode> AllWaterPathDiagnosticColorModes();
[[nodiscard]] const char* WaterPathDiagnosticColorModeLabel(WaterPathDiagnosticColorMode mode);
[[nodiscard]] bool WaterPathDiagnosticRebuildCountersEqual(
    const WaterPathDiagnosticRebuildCounters& left,
    const WaterPathDiagnosticRebuildCounters& right);
[[nodiscard]] WaterPathDiagnosticModeChangeStats RecordWaterPathDiagnosticModeChange(
    WaterPathDiagnosticModeChangeStats previous,
    const WaterPathDiagnosticRebuildCounters& before,
    const WaterPathDiagnosticRebuildCounters& after,
    std::chrono::steady_clock::duration latency);

}  // namespace invisible_places::app
