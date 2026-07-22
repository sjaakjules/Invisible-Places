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
    bool hasSourceRgb = false;
    bool fastBasic = false;
    std::uint64_t drawPointCount = 0;
    glm::mat4 localToWorld{1.0F};
    invisible_places::renderer::pointcloud::PointCloudDensityCompensation densityCompensation{};
    bool roughnessMotionFullLayer = false;
    std::size_t roughnessMotionFieldSlot = std::numeric_limits<std::size_t>::max();
    std::size_t groundIdMotionFieldSlot = std::numeric_limits<std::size_t>::max();
    std::size_t waterEffectEmissionAddFieldSlot = std::numeric_limits<std::size_t>::max();
    std::size_t waterEffectOpacityAddFieldSlot = std::numeric_limits<std::size_t>::max();
    std::size_t waterEffectOpacityMultiplyFieldSlot = std::numeric_limits<std::size_t>::max();
    std::size_t waterEffectPointSizeAddFieldSlot = std::numeric_limits<std::size_t>::max();
    std::size_t waterEffectPointSizeMultiplyFieldSlot = std::numeric_limits<std::size_t>::max();
    std::size_t waterEffectColourRedFieldSlot = std::numeric_limits<std::size_t>::max();
    std::size_t waterEffectColourGreenFieldSlot = std::numeric_limits<std::size_t>::max();
    std::size_t waterEffectColourBlueFieldSlot = std::numeric_limits<std::size_t>::max();
    std::size_t waterEffectColourMixFieldSlot = std::numeric_limits<std::size_t>::max();
    std::vector<invisible_places::water::WaterRippleRuntimeMembership> rippleMemberships;
    std::vector<invisible_places::water::WaterRippleRuntimeParams> rippleParams;
    std::vector<glm::uvec2> rippleMembershipRanges;
    invisible_places::water::WaterSeepageSpatialGrid seepageGrid;
    invisible_places::water::WaterSurfaceRole rainCollisionRole =
        invisible_places::water::WaterSurfaceRole::None;
    const invisible_places::water::RainImpactGrid* rainImpactGrid = nullptr;
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
