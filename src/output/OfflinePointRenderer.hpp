#pragma once

#include "camera/CameraState.hpp"
#include "io/PointCloudData.hpp"
#include "output/ExrWriter.hpp"
#include "renderer/pointcloud/PointCloudPreviewState.hpp"
#include "water/RainSimulation.hpp"
#include "water/WaterFlow.hpp"

#include <cstdint>
#include <limits>
#include <span>
#include <vector>

#include <glm/mat4x4.hpp>
#include <glm/vec2.hpp>

namespace invisible_places::output {

struct OfflinePointLayer {
    const invisible_places::io::LoadedPointCloud* cloud = nullptr;
    invisible_places::renderer::pointcloud::PointCloudStyleState style{};
    invisible_places::renderer::pointcloud::ResolvedTimingColouriseStack timingColourise{};
    // Fixed water scalar slots are valid only for app-generated Water
    // overlays, never for an authored/source point cloud with a coincidental
    // field count or flow-style flag.
    bool generatedWaterOverlay = false;
    bool hasSourceRgb = false;
    bool fastBasic = false;
    std::uint64_t drawPointCount = 0;
    glm::mat4 localToWorld{1.0F};
    invisible_places::renderer::pointcloud::PointCloudDensityCompensation densityCompensation{};
    bool roughnessMotionFullLayer = false;
    std::size_t roughnessMotionFieldSlot = std::numeric_limits<std::size_t>::max();
    std::size_t groundIdMotionFieldSlot = std::numeric_limits<std::size_t>::max();
    // Resolved once per layer; the surface-stability weight is evaluated per
    // point, so it must not re-scan field names inside the draw loops.
    std::size_t surfaceStabilityFieldSlot = std::numeric_limits<std::size_t>::max();
    invisible_places::water::WaterSeepageSpatialGrid seepageGrid;
    invisible_places::water::WaterSurfaceRole rainCollisionRole =
        invisible_places::water::WaterSurfaceRole::None;
    const invisible_places::water::RainImpactGrid* rainImpactGrid = nullptr;
    // Which impact effects shade this layer's points and inside which
    // world-Z bands. Mirrors the viewport's styleData.rainImpactControl.x
    // plus band uniforms and is deliberately independent of
    // rainCollisionRole; thread RainImpactEffectMask(settings) and the
    // settings' sand/rock/vegetation bands when a grid is attached. The
    // defaults (all effects, unbounded bands) shade every point in range.
    std::uint32_t rainEffectMask =
        invisible_places::water::kRainImpactEffectAllBits;
    invisible_places::water::RainImpactHeightBand rainRingsBand{};
    invisible_places::water::RainImpactHeightBand rainWetnessBand{};
    invisible_places::water::RainImpactHeightBand rainDropletsBand{};
    float roughnessMotionMinimum = 0.0F;
    float roughnessMotionInvRange = 1.0F;
};

struct OfflineRainFrame {
    invisible_places::water::RainRuntimeSettings settings{};
    invisible_places::water::WaterRainVisualSettings visual{};
    std::span<const invisible_places::water::RainParticle> particles;
    float timeSeconds = 0.0F;
};

struct OfflineRainSimulationState {
    explicit OfflineRainSimulationState(
        std::uint32_t particleCapacity = invisible_places::water::kRainParticleCapacity)
        : simulator(particleCapacity) {}

    invisible_places::water::RainSimulator simulator;
    invisible_places::water::RainImpactGrid impactGrid;
    invisible_places::water::RainSimulationDiagnostics diagnostics{};
    OfflineRainFrame frame{};
};

struct OfflineRenderTile {
    std::uint32_t x0 = 0;
    std::uint32_t y0 = 0;
    std::uint32_t x1 = 0;
    std::uint32_t y1 = 0;
};

struct OfflinePointRenderDiagnostics {
    std::uint64_t depthVisitedPoints = 0;
    std::uint64_t accumulationVisitedPoints = 0;
    std::uint64_t depthCoveredPixels = 0;
    std::uint64_t accumulationCoveredPixels = 0;
    std::uint32_t depthPassLayers = 0;
    std::uint32_t accumulationPassLayers = 0;
    std::uint32_t skippedInactiveBindings = 0;
    double depthPassMs = 0.0;
    double accumulationPassMs = 0.0;
    double compositePassMs = 0.0;
};

struct OfflinePointRenderScratch {
    std::vector<float> accumR;
    std::vector<float> accumG;
    std::vector<float> accumB;
    std::vector<float> accumA;
    std::vector<float> revealage;
    std::vector<float> emissionR;
    std::vector<float> emissionG;
    std::vector<float> emissionB;
    std::vector<float> emissionA;
    std::vector<float> prepassDepth;
};

void InitializeExrImage(ExrImage* image, std::uint32_t width, std::uint32_t height);
void AdvanceOfflineRainFrame(
    OfflineRainSimulationState* state,
    const invisible_places::water::WaterSurfaceCache& surfaceCache,
    const invisible_places::water::RainRuntimeSettings& settings,
    const invisible_places::water::WaterRainVisualSettings& visual,
    const invisible_places::camera::CameraState& cameraState,
    float timeSeconds,
    float deltaSeconds);
std::vector<OfflineRenderTile> BuildOfflineRenderTiles(
    std::uint32_t width,
    std::uint32_t height,
    std::uint32_t tileSize);
void RenderPointCloudTile(
    const std::vector<OfflinePointLayer>& layers,
    const invisible_places::camera::CameraState& cameraState,
    const OfflineRenderTile& tile,
    ExrImage* image,
    OfflinePointRenderDiagnostics* diagnostics = nullptr,
    OfflinePointRenderScratch* scratch = nullptr,
    float stylisationTimeSeconds = 0.0F,
    const OfflineRainFrame* rainFrame = nullptr);

}  // namespace invisible_places::output
