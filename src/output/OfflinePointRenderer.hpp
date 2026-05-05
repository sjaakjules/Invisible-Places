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

void InitializeExrImage(ExrImage* image, std::uint32_t width, std::uint32_t height);
std::vector<OfflineRenderTile> BuildOfflineRenderTiles(
    std::uint32_t width,
    std::uint32_t height,
    std::uint32_t tileSize);
void RenderPointCloudTile(
    const std::vector<OfflinePointLayer>& layers,
    const invisible_places::camera::CameraState& cameraState,
    const OfflineRenderTile& tile,
    ExrImage* image);

}  // namespace invisible_places::output
