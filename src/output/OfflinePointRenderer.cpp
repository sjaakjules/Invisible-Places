#include "output/OfflinePointRenderer.hpp"

#include "renderer/pointcloud/Colormap.hpp"

#include "camera/OrbitCamera.hpp"
#include "style/RenderParameterBinding.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

#include <glm/geometric.hpp>
#include <glm/matrix.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

namespace invisible_places::output {

namespace {

constexpr std::size_t kOfflinePointChunkSize = 1'000'000U;

float Clamp01(float value) {
    return std::clamp(value, 0.0F, 1.0F);
}

float ScalarFieldValue(
    const invisible_places::io::LoadedPointCloud& cloud,
    const invisible_places::style::RenderParameterBinding& binding,
    std::size_t pointIndex) {
    if (binding.fieldMap.fieldSlot < 0 ||
        static_cast<std::size_t>(binding.fieldMap.fieldSlot) >= cloud.scalarFields.size()) {
        return 0.0F;
    }

    const auto fieldIndex = static_cast<std::size_t>(binding.fieldMap.fieldSlot);
    const auto valueIndex = cloud.ScalarFieldValueIndex(fieldIndex, pointIndex);
    if (valueIndex >= cloud.scalarFieldValues.size()) {
        return 0.0F;
    }

    return cloud.scalarFieldValues[valueIndex];
}

float EvaluateBinding(
    const invisible_places::io::LoadedPointCloud& cloud,
    const invisible_places::style::RenderParameterBinding& binding,
    std::size_t pointIndex) {
    const invisible_places::io::ScalarFieldStats* fieldStats = nullptr;
    if (binding.fieldMap.fieldSlot >= 0 &&
        static_cast<std::size_t>(binding.fieldMap.fieldSlot) < cloud.scalarFields.size()) {
        fieldStats = &cloud.scalarFields[static_cast<std::size_t>(binding.fieldMap.fieldSlot)];
    }

    return invisible_places::style::EvaluateScalarBinding(
        binding,
        ScalarFieldValue(cloud, binding, pointIndex),
        fieldStats);
}

glm::vec3 SourceRgb(std::uint32_t packedColor) {
    return {
        static_cast<float>(packedColor & 0xFFU) / 255.0F,
        static_cast<float>((packedColor >> 8U) & 0xFFU) / 255.0F,
        static_cast<float>((packedColor >> 16U) & 0xFFU) / 255.0F,
    };
}

glm::vec3 ResolvePointColor(
    const OfflinePointLayer& layer,
    std::size_t pointIndex) {
    if (layer.cloud == nullptr) {
        return {1.0F, 1.0F, 1.0F};
    }

    const auto& cloud = *layer.cloud;
    if (layer.style.colorMode == invisible_places::renderer::pointcloud::PointCloudColorMode::SourceRgb &&
        layer.hasSourceRgb &&
        pointIndex < cloud.packedColors.size()) {
        return SourceRgb(cloud.packedColors[pointIndex]);
    }

    if (layer.style.colorMode == invisible_places::renderer::pointcloud::PointCloudColorMode::ScalarColormap &&
        !cloud.scalarFields.empty()) {
        const auto color = invisible_places::renderer::pointcloud::SampleColormap(
            layer.style.colormap,
            EvaluateBinding(cloud, layer.style.colormapPosition, pointIndex));
        return {color[0], color[1], color[2]};
    }

    return {
        layer.style.solidColor[0],
        layer.style.solidColor[1],
        layer.style.solidColor[2],
    };
}

struct OfflinePointSample {
    glm::vec3 worldCenter{0.0F, 0.0F, 0.0F};
    float pixelCenterX = 0.0F;
    float pixelCenterY = 0.0F;
    float viewDepth = 0.0F;
    float pointSize = 1.0F;
    float surfelDiameter = 0.005F;
    float opacity = 1.0F;
    float emissive = 0.0F;
    float xray = 0.0F;
    float depthFade = 0.0F;
    glm::vec3 color{1.0F, 1.0F, 1.0F};
    bool worldSurfels = false;
    bool hasNormal = false;
    glm::vec3 normal{0.0F, 0.0F, 1.0F};
    glm::vec3 tangent{1.0F, 0.0F, 0.0F};
    glm::vec3 bitangent{0.0F, 1.0F, 0.0F};
};

float SmoothStep(float edge0, float edge1, float value) {
    const float width = edge1 - edge0;
    if (std::abs(width) <= 1.0e-6F) {
        return value < edge0 ? 0.0F : 1.0F;
    }

    const float t = std::clamp((value - edge0) / width, 0.0F, 1.0F);
    return t * t * (3.0F - (2.0F * t));
}

float PointFalloff(
    const invisible_places::renderer::pointcloud::PointCloudStyleState& style,
    float normalizedRadius,
    float normalizedRadiusSquared) {
    using invisible_places::renderer::pointcloud::PointCloudFalloffProfile;

    if (normalizedRadiusSquared > 1.0F) {
        return 0.0F;
    }

    auto profile = style.falloffProfile;

    switch (profile) {
        case PointCloudFalloffProfile::HardDisc:
            return 1.0F;
        case PointCloudFalloffProfile::Gaussian:
            return std::exp(-normalizedRadiusSquared * std::max(0.001F, style.gaussianSharpness));
        case PointCloudFalloffProfile::Rim:
            return std::pow(
                std::max(0.0F, 1.0F - normalizedRadius),
                std::max(0.001F, style.featherPower));
        case PointCloudFalloffProfile::SoftDisc:
            return SmoothStep(1.0F, std::clamp(style.innerRadius, 0.0F, 0.99F), normalizedRadius);
    }

    return 1.0F;
}

float ResolveDepthFadeAlpha(
    const OfflinePointSample& sample,
    const invisible_places::camera::CameraState& cameraState,
    float viewDepth) {
    const float depthNorm = std::clamp(
        (viewDepth - cameraState.nearPlane) /
            std::max(1.0e-5F, cameraState.farPlane - cameraState.nearPlane),
        0.0F,
        1.0F);
    return std::lerp(
        1.0F,
        1.0F - depthNorm,
        std::clamp(sample.depthFade, 0.0F, 1.0F));
}

float WeightedAlphaWeight(
    float alpha,
    float viewDepth,
    const invisible_places::camera::CameraState& cameraState) {
    const float depthNorm = std::clamp(
        (viewDepth - cameraState.nearPlane) /
            std::max(1.0e-5F, cameraState.farPlane - cameraState.nearPlane),
        0.0F,
        1.0F);
    const float opacityWeight = std::pow(std::min(1.0F, alpha * 8.0F) + 0.01F, 3.0F);
    const float frontWeight = std::pow(1.0F - depthNorm, 4.0F);
    return std::clamp((opacityWeight * 0.5F) + (opacityWeight * frontWeight * 128.0F), 1.0e-3F, 256.0F);
}

glm::vec3 CameraRight(const invisible_places::camera::OrbitCameraMatrices& matrices) {
    return glm::normalize(glm::vec3{matrices.view[0][0], matrices.view[1][0], matrices.view[2][0]});
}

glm::vec3 CameraUp(const invisible_places::camera::OrbitCameraMatrices& matrices) {
    return glm::normalize(glm::vec3{matrices.view[0][1], matrices.view[1][1], matrices.view[2][1]});
}

glm::vec3 ToGlm(const invisible_places::io::Float3& value) {
    return {value.x, value.y, value.z};
}

bool IsFinite(glm::vec3 value) {
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

bool ProjectWorldToPixel(
    const glm::vec3& worldPosition,
    const invisible_places::camera::OrbitCameraMatrices& matrices,
    const ExrImage& image,
    float* pixelX,
    float* pixelY) {
    const glm::vec4 clip = matrices.viewProjection * glm::vec4{worldPosition, 1.0F};
    if (clip.w <= 1.0e-6F) {
        return false;
    }

    const glm::vec3 ndc = glm::vec3{clip} / clip.w;
    if (!IsFinite(ndc)) {
        return false;
    }

    if (pixelX != nullptr) {
        *pixelX = (ndc.x * 0.5F + 0.5F) * static_cast<float>(image.width);
    }
    if (pixelY != nullptr) {
        *pixelY = (ndc.y * 0.5F + 0.5F) * static_cast<float>(image.height);
    }
    return true;
}

glm::vec3 PixelRayDirection(
    std::uint32_t x,
    std::uint32_t y,
    const invisible_places::camera::OrbitCameraMatrices& matrices,
    const ExrImage& image,
    const glm::mat4& inverseViewProjection) {
    const float ndcX =
        ((static_cast<float>(x) + 0.5F) / static_cast<float>(std::max<std::uint32_t>(1U, image.width))) * 2.0F - 1.0F;
    const float ndcY =
        ((static_cast<float>(y) + 0.5F) / static_cast<float>(std::max<std::uint32_t>(1U, image.height))) * 2.0F - 1.0F;
    glm::vec4 farWorld = inverseViewProjection * glm::vec4{ndcX, ndcY, 1.0F, 1.0F};
    if (std::abs(farWorld.w) > 1.0e-6F) {
        farWorld /= farWorld.w;
    }
    return glm::normalize(glm::vec3{farWorld} - matrices.position);
}

void ResolveSurfelBasis(
    OfflinePointSample* sample,
    const invisible_places::camera::OrbitCameraMatrices& matrices) {
    if (sample == nullptr) {
        return;
    }

    const glm::vec3 cameraRight = CameraRight(matrices);
    const glm::vec3 cameraUp = CameraUp(matrices);
    if (!sample->hasNormal || glm::dot(sample->normal, sample->normal) <= 1.0e-8F) {
        sample->normal = matrices.position - sample->worldCenter;
        if (glm::dot(sample->normal, sample->normal) <= 1.0e-8F) {
            sample->normal = glm::normalize(glm::cross(cameraRight, cameraUp));
        } else {
            sample->normal = glm::normalize(sample->normal);
        }
        sample->tangent = cameraRight;
        sample->bitangent = cameraUp;
        return;
    }

    sample->normal = glm::normalize(sample->normal);
    sample->tangent = cameraRight - (sample->normal * glm::dot(cameraRight, sample->normal));
    if (glm::dot(sample->tangent, sample->tangent) <= 1.0e-8F) {
        sample->tangent = cameraUp - (sample->normal * glm::dot(cameraUp, sample->normal));
    }
    if (glm::dot(sample->tangent, sample->tangent) <= 1.0e-8F) {
        sample->tangent = std::abs(sample->normal.z) < 0.999F
                               ? glm::cross(glm::vec3{0.0F, 0.0F, 1.0F}, sample->normal)
                               : glm::cross(glm::vec3{0.0F, 1.0F, 0.0F}, sample->normal);
    }
    sample->tangent = glm::normalize(sample->tangent);
    sample->bitangent = glm::normalize(glm::cross(sample->normal, sample->tangent));
}

bool BuildOfflinePointSample(
    const OfflinePointLayer& layer,
    const invisible_places::camera::OrbitCameraMatrices& matrices,
    const invisible_places::camera::CameraState& cameraState,
    std::size_t pointIndex,
    const ExrImage& image,
    OfflinePointSample* sample) {
    if (layer.cloud == nullptr || sample == nullptr) {
        return false;
    }

    const auto& cloud = *layer.cloud;
    const auto& point = cloud.positions[pointIndex];
    const glm::vec4 worldPosition =
        layer.localToWorld * glm::vec4{point.x, point.y, point.z, 1.0F};
    if (std::abs(worldPosition.w) <= 1.0e-6F) {
        return false;
    }

    const glm::vec4 normalizedWorld = worldPosition / worldPosition.w;
    const glm::vec4 viewPosition = matrices.view * normalizedWorld;
    const float viewDepth = -viewPosition.z;
    if (viewDepth <= cameraState.nearPlane || viewDepth >= cameraState.farPlane) {
        return false;
    }

    const glm::vec4 clip = matrices.viewProjection * normalizedWorld;
    if (clip.w <= 1.0e-6F) {
        return false;
    }

    const glm::vec3 ndc = glm::vec3{clip} / clip.w;
    const bool worldSurfels =
        layer.style.geometryMode ==
        invisible_places::renderer::pointcloud::PointCloudGeometryMode::WorldSurfels;
    if (!worldSurfels &&
        (ndc.x < -1.0F || ndc.x > 1.0F || ndc.y < -1.0F || ndc.y > 1.0F ||
         ndc.z < -1.0F || ndc.z > 1.0F)) {
        return false;
    }

    sample->pixelCenterX = (ndc.x * 0.5F + 0.5F) * static_cast<float>(image.width);
    sample->pixelCenterY = (ndc.y * 0.5F + 0.5F) * static_cast<float>(image.height);
    sample->worldCenter = glm::vec3{normalizedWorld};
    sample->viewDepth = viewDepth;
    sample->pointSize = std::clamp(EvaluateBinding(cloud, layer.style.pointSize, pointIndex), 1.0F, 64.0F);
    sample->worldSurfels = worldSurfels;
    sample->surfelDiameter = std::max(0.0F, EvaluateBinding(cloud, layer.style.surfelDiameter, pointIndex));
    sample->opacity = Clamp01(EvaluateBinding(cloud, layer.style.opacity, pointIndex));
    sample->emissive = std::max(0.0F, EvaluateBinding(cloud, layer.style.emissiveStrength, pointIndex));
    sample->xray = Clamp01(EvaluateBinding(cloud, layer.style.xrayStrength, pointIndex));
    sample->depthFade = Clamp01(EvaluateBinding(cloud, layer.style.depthFade, pointIndex));
    sample->color = ResolvePointColor(layer, pointIndex);
    if (sample->worldSurfels) {
        if (sample->surfelDiameter <= 1.0e-6F) {
            return false;
        }

        sample->hasNormal = cloud.hasNormals && pointIndex < cloud.normals.size();
        if (sample->hasNormal) {
            const glm::vec3 localNormal = ToGlm(cloud.normals[pointIndex]);
            sample->hasNormal = glm::dot(localNormal, localNormal) > 1.0e-8F;
            if (sample->hasNormal) {
                sample->normal = glm::normalize(glm::transpose(glm::inverse(glm::mat3{layer.localToWorld})) * localNormal);
                sample->hasNormal = IsFinite(sample->normal) && glm::dot(sample->normal, sample->normal) > 1.0e-8F;
            }
        }
        ResolveSurfelBasis(sample, matrices);
    }
    return sample->opacity > 0.0F;
}

template <typename PixelCallback>
void VisitCoveredPixels(
    const OfflinePointSample& sample,
    const invisible_places::renderer::pointcloud::PointCloudStyleState& style,
    const invisible_places::camera::OrbitCameraMatrices& matrices,
    const ExrImage& image,
    const OfflineRenderTile& tile,
    std::uint32_t tileWidth,
    PixelCallback callback) {
    if (sample.worldSurfels) {
        const float radiusWorld = sample.surfelDiameter * 0.5F;
        const std::array<glm::vec3, 4> quadCorners = {
            sample.worldCenter + ((-sample.tangent - sample.bitangent) * radiusWorld),
            sample.worldCenter + ((sample.tangent - sample.bitangent) * radiusWorld),
            sample.worldCenter + ((sample.tangent + sample.bitangent) * radiusWorld),
            sample.worldCenter + ((-sample.tangent + sample.bitangent) * radiusWorld),
        };

        float minPixelX = std::numeric_limits<float>::max();
        float minPixelY = std::numeric_limits<float>::max();
        float maxPixelX = std::numeric_limits<float>::lowest();
        float maxPixelY = std::numeric_limits<float>::lowest();
        bool projectedAnyCorner = false;
        for (const auto& corner : quadCorners) {
            float pixelX = 0.0F;
            float pixelY = 0.0F;
            if (!ProjectWorldToPixel(corner, matrices, image, &pixelX, &pixelY)) {
                continue;
            }
            projectedAnyCorner = true;
            minPixelX = std::min(minPixelX, pixelX);
            minPixelY = std::min(minPixelY, pixelY);
            maxPixelX = std::max(maxPixelX, pixelX);
            maxPixelY = std::max(maxPixelY, pixelY);
        }
        if (!projectedAnyCorner) {
            return;
        }

        const int minX = std::max<int>(static_cast<int>(tile.x0), static_cast<int>(std::floor(minPixelX)) - 1);
        const int maxX = std::min<int>(static_cast<int>(tile.x1) - 1, static_cast<int>(std::ceil(maxPixelX)) + 1);
        const int minY = std::max<int>(static_cast<int>(tile.y0), static_cast<int>(std::floor(minPixelY)) - 1);
        const int maxY = std::min<int>(static_cast<int>(tile.y1) - 1, static_cast<int>(std::ceil(maxPixelY)) + 1);
        if (minX > maxX || minY > maxY) {
            return;
        }

        const glm::mat4 inverseViewProjection = glm::inverse(matrices.viewProjection);
        const float radiusSquaredWorld = radiusWorld * radiusWorld;
        for (int y = minY; y <= maxY; ++y) {
            for (int x = minX; x <= maxX; ++x) {
                const glm::vec3 rayDirection = PixelRayDirection(
                    static_cast<std::uint32_t>(x),
                    static_cast<std::uint32_t>(y),
                    matrices,
                    image,
                    inverseViewProjection);
                const float denominator = glm::dot(sample.normal, rayDirection);
                if (std::abs(denominator) <= 1.0e-6F) {
                    continue;
                }

                const float distanceAlongRay =
                    glm::dot(sample.worldCenter - matrices.position, sample.normal) / denominator;
                if (distanceAlongRay <= 0.0F) {
                    continue;
                }

                const glm::vec3 hitPoint = matrices.position + (rayDirection * distanceAlongRay);
                const glm::vec3 localOffset = hitPoint - sample.worldCenter;
                const float u = glm::dot(localOffset, sample.tangent);
                const float v = glm::dot(localOffset, sample.bitangent);
                const float normalizedRadiusSquared = ((u * u) + (v * v)) / radiusSquaredWorld;
                if (normalizedRadiusSquared > 1.0F) {
                    continue;
                }

                const float normalizedRadius = std::sqrt(normalizedRadiusSquared);
                const float falloff = PointFalloff(style, normalizedRadius, normalizedRadiusSquared);
                if (falloff <= 1.0e-5F) {
                    continue;
                }

                const glm::vec4 viewPosition = matrices.view * glm::vec4{hitPoint, 1.0F};
                const float coveredViewDepth = -viewPosition.z;
                if (coveredViewDepth <= 0.0F) {
                    continue;
                }

                const auto localIndex =
                    static_cast<std::size_t>(y - static_cast<int>(tile.y0)) * static_cast<std::size_t>(tileWidth) +
                    static_cast<std::size_t>(x - static_cast<int>(tile.x0));
                callback(
                    static_cast<std::uint32_t>(x),
                    static_cast<std::uint32_t>(y),
                    localIndex,
                    falloff,
                    coveredViewDepth);
            }
        }
        return;
    }

    const auto radiusPixels = static_cast<int>(std::ceil(sample.pointSize * 0.5F));
    const int centerX = static_cast<int>(std::floor(sample.pixelCenterX));
    const int centerY = static_cast<int>(std::floor(sample.pixelCenterY));
    const int minX = std::max<int>(static_cast<int>(tile.x0), centerX - radiusPixels);
    const int maxX = std::min<int>(static_cast<int>(tile.x1) - 1, centerX + radiusPixels);
    const int minY = std::max<int>(static_cast<int>(tile.y0), centerY - radiusPixels);
    const int maxY = std::min<int>(static_cast<int>(tile.y1) - 1, centerY + radiusPixels);
    if (minX > maxX || minY > maxY) {
        return;
    }

    const float safeRadius = std::max(0.5F, sample.pointSize * 0.5F);
    for (int y = minY; y <= maxY; ++y) {
        for (int x = minX; x <= maxX; ++x) {
            const float dx = (static_cast<float>(x) + 0.5F) - sample.pixelCenterX;
            const float dy = (static_cast<float>(y) + 0.5F) - sample.pixelCenterY;
            const float normalizedRadiusSquared = ((dx * dx) + (dy * dy)) / (safeRadius * safeRadius);
            if (normalizedRadiusSquared > 1.0F) {
                continue;
            }

            const float normalizedRadius = std::sqrt(normalizedRadiusSquared);
            const float falloff = PointFalloff(style, normalizedRadius, normalizedRadiusSquared);
            if (falloff <= 1.0e-5F) {
                continue;
            }

            const auto localIndex =
                static_cast<std::size_t>(y - static_cast<int>(tile.y0)) * static_cast<std::size_t>(tileWidth) +
                static_cast<std::size_t>(x - static_cast<int>(tile.x0));
            callback(
                static_cast<std::uint32_t>(x),
                static_cast<std::uint32_t>(y),
                localIndex,
                falloff,
                sample.viewDepth);
        }
    }
}

}  // namespace

void InitializeExrImage(ExrImage* image, std::uint32_t width, std::uint32_t height) {
    if (image == nullptr) {
        return;
    }

    image->width = width;
    image->height = height;
    const auto pixelCount = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
    image->beautyR.assign(pixelCount, 0.0F);
    image->beautyG.assign(pixelCount, 0.0F);
    image->beautyB.assign(pixelCount, 0.0F);
    image->alpha.assign(pixelCount, 0.0F);
    image->depth.assign(pixelCount, std::numeric_limits<float>::infinity());
}

std::vector<OfflineRenderTile> BuildOfflineRenderTiles(
    std::uint32_t width,
    std::uint32_t height,
    std::uint32_t tileSize) {
    const auto safeTileSize = std::max<std::uint32_t>(1U, tileSize);
    std::vector<OfflineRenderTile> tiles;
    for (std::uint32_t y = 0; y < height; y += safeTileSize) {
        for (std::uint32_t x = 0; x < width; x += safeTileSize) {
            tiles.push_back(
                {.x0 = x,
                 .y0 = y,
                 .x1 = std::min(width, x + safeTileSize),
                 .y1 = std::min(height, y + safeTileSize)});
        }
    }
    return tiles;
}

void RenderPointCloudTile(
    const std::vector<OfflinePointLayer>& layers,
    const invisible_places::camera::CameraState& cameraState,
    const OfflineRenderTile& tile,
    ExrImage* image) {
    if (image == nullptr || image->width == 0 || image->height == 0 || tile.x0 >= tile.x1 || tile.y0 >= tile.y1) {
        return;
    }

    invisible_places::camera::OrbitCamera camera;
    camera.ApplyState(cameraState);
    const float aspectRatio = static_cast<float>(image->width) / static_cast<float>(image->height);
    const auto matrices = camera.Matrices(aspectRatio);
    const std::uint32_t tileWidth = tile.x1 - tile.x0;
    const std::uint32_t tileHeight = tile.y1 - tile.y0;
    const auto tilePixelCount = static_cast<std::size_t>(tileWidth) * static_cast<std::size_t>(tileHeight);

    std::vector<float> accumR(tilePixelCount, 0.0F);
    std::vector<float> accumG(tilePixelCount, 0.0F);
    std::vector<float> accumB(tilePixelCount, 0.0F);
    std::vector<float> accumA(tilePixelCount, 0.0F);
    std::vector<float> revealage(tilePixelCount, 1.0F);
    std::vector<float> emissionR(tilePixelCount, 0.0F);
    std::vector<float> emissionG(tilePixelCount, 0.0F);
    std::vector<float> emissionB(tilePixelCount, 0.0F);
    std::vector<float> emissionA(tilePixelCount, 0.0F);

    for (const auto& layer : layers) {
        if (layer.cloud == nullptr || layer.cloud->positions.empty()) {
            continue;
        }

        const auto& cloud = *layer.cloud;
        for (std::size_t chunkStart = 0; chunkStart < cloud.positions.size(); chunkStart += kOfflinePointChunkSize) {
            const auto chunkEnd = std::min(cloud.positions.size(), chunkStart + kOfflinePointChunkSize);
            for (std::size_t pointIndex = chunkStart; pointIndex < chunkEnd; ++pointIndex) {
                OfflinePointSample sample;
                if (!BuildOfflinePointSample(layer, matrices, cameraState, pointIndex, *image, &sample)) {
                    continue;
                }

                VisitCoveredPixels(
                    sample,
                    layer.style,
                    matrices,
                    *image,
                    tile,
                    tileWidth,
                    [&](std::uint32_t x, std::uint32_t y, std::size_t, float falloff, float coveredViewDepth) {
                        const auto pixelIndex =
                            static_cast<std::size_t>(y) * static_cast<std::size_t>(image->width) +
                            static_cast<std::size_t>(x);
                        const float alpha =
                            std::clamp(
                                sample.opacity * falloff * ResolveDepthFadeAlpha(sample, cameraState, coveredViewDepth),
                                0.0F,
                                0.995F);
                        if (pixelIndex < image->depth.size() &&
                            coveredViewDepth < image->depth[pixelIndex] &&
                            invisible_places::renderer::pointcloud::PointCloudAlphaContributesDepth(
                                layer.style,
                                alpha)) {
                            image->depth[pixelIndex] = coveredViewDepth;
                        }
                    });
            }
        }
    }

    for (const auto& layer : layers) {
        if (layer.cloud == nullptr || layer.cloud->positions.empty()) {
            continue;
        }

        const auto& cloud = *layer.cloud;
        for (std::size_t chunkStart = 0; chunkStart < cloud.positions.size(); chunkStart += kOfflinePointChunkSize) {
            const auto chunkEnd = std::min(cloud.positions.size(), chunkStart + kOfflinePointChunkSize);
            for (std::size_t pointIndex = chunkStart; pointIndex < chunkEnd; ++pointIndex) {
                OfflinePointSample sample;
                if (!BuildOfflinePointSample(layer, matrices, cameraState, pointIndex, *image, &sample)) {
                    continue;
                }

                VisitCoveredPixels(
                    sample,
                    layer.style,
                    matrices,
                    *image,
                    tile,
                    tileWidth,
                    [&](std::uint32_t x,
                        std::uint32_t y,
                        std::size_t localIndex,
                        float falloff,
                        float coveredViewDepth) {
                        const auto pixelIndex =
                            static_cast<std::size_t>(y) * static_cast<std::size_t>(image->width) +
                            static_cast<std::size_t>(x);
                        if (pixelIndex >= image->depth.size()) {
                            return;
                        }

                        const float alpha =
                            std::clamp(
                                sample.opacity * falloff * ResolveDepthFadeAlpha(sample, cameraState, coveredViewDepth),
                                0.0F,
                                0.995F);
                        if (alpha <= 1.0e-5F) {
                            return;
                        }

                        const float densityScale = std::max(1.0F, layer.style.densityScale);
                        const float densityClamp = std::max(0.0F, layer.style.densityClamp);
                        const float weightedAlpha = std::clamp(
                            densityClamp > 0.0F ? std::min(alpha * densityScale, densityClamp) : alpha,
                            0.0F,
                            0.995F);
                        const float weight = WeightedAlphaWeight(weightedAlpha, coveredViewDepth, cameraState);
                        accumR[localIndex] += sample.color.r * weightedAlpha * weight;
                        accumG[localIndex] += sample.color.g * weightedAlpha * weight;
                        accumB[localIndex] += sample.color.b * weightedAlpha * weight;
                        accumA[localIndex] += weightedAlpha * weight;
                        revealage[localIndex] *= (1.0F - weightedAlpha);

                        const float emissionGain = sample.emissive * std::max(0.0F, layer.style.exposure);
                        if (emissionGain > 1.0e-5F) {
                            emissionR[localIndex] += sample.color.r * alpha * emissionGain;
                            emissionG[localIndex] += sample.color.g * alpha * emissionGain;
                            emissionB[localIndex] += sample.color.b * alpha * emissionGain;
                            emissionA[localIndex] += alpha * emissionGain;
                        }

                        if (sample.xray > 1.0e-5F) {
                            const float sceneDepth = image->depth[pixelIndex];
                            if (!std::isfinite(sceneDepth) || sample.xray <= 1.0e-5F) {
                                return;
                            }

                            const float behind = std::max(
                                coveredViewDepth - sceneDepth - std::max(0.0F, layer.style.depthBias),
                                0.0F);
                            const float hiddenFade =
                                std::exp(-behind * std::max(0.0F, layer.style.depthFalloff));
                            const float frontMask =
                                coveredViewDepth <= sceneDepth + std::max(0.0F, layer.style.depthBias) ? 1.0F : 0.0F;
                            const float xrayAlpha =
                                alpha * sample.xray *
                                std::lerp(
                                    layer.style.hiddenAlpha * hiddenFade,
                                    layer.style.frontAlpha,
                                    frontMask);
                            if (xrayAlpha <= 1.0e-5F) {
                                return;
                            }

                            const float gain = std::max(0.0F, layer.style.exposure);
                            emissionR[localIndex] += sample.color.r * xrayAlpha * gain;
                            emissionG[localIndex] += sample.color.g * xrayAlpha * gain;
                            emissionB[localIndex] += sample.color.b * xrayAlpha * gain;
                            emissionA[localIndex] += xrayAlpha * gain;
                        }
                    });
            }
        }
    }

    for (std::uint32_t y = tile.y0; y < tile.y1; ++y) {
        for (std::uint32_t x = tile.x0; x < tile.x1; ++x) {
            const auto localIndex =
                static_cast<std::size_t>(y - tile.y0) * static_cast<std::size_t>(tileWidth) +
                static_cast<std::size_t>(x - tile.x0);
            const auto pixelIndex =
                static_cast<std::size_t>(y) * static_cast<std::size_t>(image->width) + static_cast<std::size_t>(x);
            if (pixelIndex >= image->alpha.size()) {
                continue;
            }

            const float transparentAlpha = std::clamp(1.0F - revealage[localIndex], 0.0F, 1.0F);
            const glm::vec3 transparentColor =
                accumA[localIndex] > 1.0e-5F
                    ? glm::vec3{
                          accumR[localIndex] / accumA[localIndex],
                          accumG[localIndex] / accumA[localIndex],
                          accumB[localIndex] / accumA[localIndex]}
                    : glm::vec3{0.0F, 0.0F, 0.0F};
            const glm::vec3 emission{
                1.0F - std::exp(-std::max(0.0F, emissionR[localIndex])),
                1.0F - std::exp(-std::max(0.0F, emissionG[localIndex])),
                1.0F - std::exp(-std::max(0.0F, emissionB[localIndex])),
            };
            const float emissionAlpha =
                std::clamp(1.0F - std::exp(-std::max(0.0F, emissionA[localIndex])), 0.0F, 1.0F);
            const float sourceAlpha = std::max(transparentAlpha, emissionAlpha);
            if (sourceAlpha <= 1.0e-5F) {
                continue;
            }

            const glm::vec3 desiredContribution = (transparentColor * transparentAlpha) + emission;
            const glm::vec3 sourceColor = desiredContribution / std::max(sourceAlpha, 1.0e-5F);
            const float destinationAlpha = image->alpha[pixelIndex];
            const glm::vec3 destinationColor{
                image->beautyR[pixelIndex],
                image->beautyG[pixelIndex],
                image->beautyB[pixelIndex],
            };
            const float outputAlpha = sourceAlpha + (destinationAlpha * (1.0F - sourceAlpha));
            const glm::vec3 outputColor =
                (sourceColor * sourceAlpha) + (destinationColor * (1.0F - sourceAlpha));

            image->beautyR[pixelIndex] = outputColor.r;
            image->beautyG[pixelIndex] = outputColor.g;
            image->beautyB[pixelIndex] = outputColor.b;
            image->alpha[pixelIndex] = outputAlpha;
        }
    }
}

}  // namespace invisible_places::output
