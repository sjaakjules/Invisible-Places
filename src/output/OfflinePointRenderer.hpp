#pragma once

#include "camera/CameraState.hpp"
#include "io/PointCloudData.hpp"
#include "output/ExrWriter.hpp"
#include "renderer/pointcloud/PointCloudPreviewState.hpp"

#include <cstdint>
#include <vector>

#include <glm/mat4x4.hpp>

namespace invisible_places::output {

struct OfflinePointLayer {
    const invisible_places::io::LoadedPointCloud* cloud = nullptr;
    invisible_places::renderer::pointcloud::PointCloudStyleState style{};
    bool hasSourceRgb = false;
    glm::mat4 localToWorld{1.0F};
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
    OfflinePointRenderScratch* scratch = nullptr);

}  // namespace invisible_places::output
