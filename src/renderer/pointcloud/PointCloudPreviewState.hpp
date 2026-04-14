#pragma once

#include "io/PointCloudData.hpp"
#include "style/RenderParameterBinding.hpp"

#include <array>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace invisible_places::renderer::pointcloud {

enum class PointCloudColorMode {
    SourceRgb,
    SolidColor,
    ScalarColormap
};

enum class PointCloudColormapId {
    Viridis,
    Plasma,
    Inferno,
    Magma,
    Cividis,
    Turbo
};

struct PointCloudStyleState {
    PointCloudStyleState();

    PointCloudColorMode colorMode = PointCloudColorMode::SourceRgb;
    PointCloudColormapId colormap = PointCloudColormapId::Viridis;
    std::array<float, 4> solidColor{0.93F, 0.88F, 0.72F, 1.0F};
    invisible_places::style::RenderParameterBinding pointSize;
    invisible_places::style::RenderParameterBinding opacity;
    invisible_places::style::RenderParameterBinding emissiveStrength;
    invisible_places::style::RenderParameterBinding xrayStrength;
    invisible_places::style::RenderParameterBinding depthFade;
    invisible_places::style::RenderParameterBinding colormapPosition;
};

struct PointBudgetState {
    std::uint64_t totalPoints = 0;
    std::uint64_t activePoints = 0;
    float activeFraction = 1.0F;
    std::vector<std::uint32_t> sampledIndices;

    [[nodiscard]] bool UsesSampledIndices() const { return !sampledIndices.empty(); }
};

struct PointCloudSessionState {
    std::filesystem::path sourcePath;
    std::string displayName;
    bool loaded = false;
    bool active = false;
    bool hasSourceRgb = false;
    bool hasFocusPoint = false;
    std::uint64_t totalPoints = 0;
    invisible_places::io::Bounds3f bounds{};
    invisible_places::io::Float3 focusPoint{};
    std::vector<invisible_places::io::ScalarFieldStats> scalarFields;
    PointBudgetState budget{};
    PointCloudStyleState style{};
};

std::uint64_t ClampPointBudget(std::uint64_t totalPoints, std::uint64_t requestedPoints);
PointBudgetState MakePointBudgetState(std::uint64_t totalPoints, std::uint64_t requestedPoints);
std::vector<std::uint32_t> GenerateDeterministicSampleIndices(
    std::uint64_t totalPoints,
    std::uint64_t requestedPoints);

}  // namespace invisible_places::renderer::pointcloud
