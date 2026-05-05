#include "output/OfflinePointRenderer.hpp"

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

glm::vec3 ClampColor(glm::vec3 color) {
    return {
        Clamp01(color.r),
        Clamp01(color.g),
        Clamp01(color.b),
    };
}

glm::vec3 Viridis(float t) {
    return ClampColor({
        0.277727F + 0.520420F * t + 0.117231F * t * t - 0.219384F * t * t * t,
        0.005407F + 1.404613F * t - 1.653928F * t * t + 0.743293F * t * t * t,
        0.334099F + 1.384590F * t - 1.584386F * t * t + 0.630205F * t * t * t,
    });
}

glm::vec3 Plasma(float t) {
    return ClampColor({
        0.058732F + 2.176514F * t - 2.689460F * t * t + 1.466006F * t * t * t,
        0.023336F + 0.238383F * t + 1.118022F * t * t - 0.905937F * t * t * t,
        0.543817F + 0.753960F * t - 1.308172F * t * t + 0.806940F * t * t * t,
    });
}

glm::vec3 Inferno(float t) {
    return ClampColor({
        0.000218F + 1.538871F * t - 1.908164F * t * t + 0.873490F * t * t * t,
        0.001651F - 0.117089F * t + 2.064416F * t * t - 1.277355F * t * t * t,
        0.013866F + 0.635041F * t + 0.618582F * t * t - 0.700491F * t * t * t,
    });
}

glm::vec3 Magma(float t) {
    return ClampColor({
        0.001462F + 1.384825F * t - 1.875795F * t * t + 0.850876F * t * t * t,
        0.000466F - 0.251373F * t + 1.927205F * t * t - 1.035104F * t * t * t,
        0.013866F + 0.873807F * t + 0.143707F * t * t - 0.282206F * t * t * t,
    });
}

glm::vec3 Cividis(float t) {
    return ClampColor({
        0.000000F + 0.975500F * t - 0.676400F * t * t + 0.187000F * t * t * t,
        0.126200F + 0.662500F * t - 0.360900F * t * t + 0.079200F * t * t * t,
        0.301500F + 0.539600F * t - 0.540700F * t * t + 0.219900F * t * t * t,
    });
}

glm::vec3 Turbo(float t) {
    return ClampColor({
        0.13572138F + 4.61539260F * t - 42.66032258F * t * t + 132.13108234F * t * t * t -
            152.94239396F * t * t * t * t + 59.28637943F * t * t * t * t * t,
        0.09140261F + 2.19418839F * t + 4.84296658F * t * t - 14.18503333F * t * t * t +
            4.27729857F * t * t * t * t + 2.82956604F * t * t * t * t * t,
        0.10667330F + 12.64194608F * t - 60.58204836F * t * t + 110.36276771F * t * t * t -
            89.90310912F * t * t * t * t + 27.34824973F * t * t * t * t * t,
    });
}

glm::vec3 ApplyColormap(
    invisible_places::renderer::pointcloud::PointCloudColormapId colormap,
    float value) {
    const float t = Clamp01(value);
    switch (colormap) {
        case invisible_places::renderer::pointcloud::PointCloudColormapId::Plasma:
            return Plasma(t);
        case invisible_places::renderer::pointcloud::PointCloudColormapId::Inferno:
            return Inferno(t);
        case invisible_places::renderer::pointcloud::PointCloudColormapId::Magma:
            return Magma(t);
        case invisible_places::renderer::pointcloud::PointCloudColormapId::Cividis:
            return Cividis(t);
        case invisible_places::renderer::pointcloud::PointCloudColormapId::Turbo:
            return Turbo(t);
        case invisible_places::renderer::pointcloud::PointCloudColormapId::Viridis:
            return Viridis(t);
    }

    return Viridis(t);
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
        return ApplyColormap(
            layer.style.colormap,
            EvaluateBinding(cloud, layer.style.colormapPosition, pointIndex));
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

invisible_places::renderer::pointcloud::PointCloudRenderMode EffectiveOfflineRenderMode(
    invisible_places::renderer::pointcloud::PointCloudRenderMode mode) {
    if (mode == invisible_places::renderer::pointcloud::PointCloudRenderMode::ComputeDensity ||
        mode == invisible_places::renderer::pointcloud::PointCloudRenderMode::GaussianPointSprite) {
        return invisible_places::renderer::pointcloud::PointCloudRenderMode::WeightedTransparent;
    }

    return mode;
}

float PointFalloff(
    const invisible_places::renderer::pointcloud::PointCloudStyleState& style,
    invisible_places::renderer::pointcloud::PointCloudRenderMode mode,
    float normalizedRadius,
    float normalizedRadiusSquared) {
    using invisible_places::renderer::pointcloud::PointCloudFalloffProfile;
    using invisible_places::renderer::pointcloud::PointCloudRenderMode;

    if (normalizedRadiusSquared > 1.0F) {
        return 0.0F;
    }

    auto profile = style.falloffProfile;
    if (mode == PointCloudRenderMode::EmissiveHard) {
        profile = PointCloudFalloffProfile::HardDisc;
    } else if (style.renderMode == PointCloudRenderMode::ComputeDensity ||
               style.renderMode == PointCloudRenderMode::GaussianPointSprite) {
        profile = PointCloudFalloffProfile::Gaussian;
    }

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

glm::vec3 ResolveSolidShadedColor(
    const OfflinePointSample& sample,
    const invisible_places::camera::CameraState& cameraState,
    float viewDepth) {
    const float depthNorm = std::clamp(
        (viewDepth - cameraState.nearPlane) /
            std::max(1.0e-5F, cameraState.farPlane - cameraState.nearPlane),
        0.0F,
        1.0F);
    const float fade = std::lerp(
        1.0F,
        1.0F - (depthNorm * 0.65F),
        std::clamp(sample.depthFade, 0.0F, 1.0F));

    glm::vec3 shadedColor = sample.color * fade;
    shadedColor = glm::mix(
        shadedColor,
        glm::vec3{1.0F, 1.0F, 1.0F},
        std::clamp(sample.xray, 0.0F, 1.0F) * 0.45F);
    shadedColor += std::max(0.0F, sample.emissive) * 0.35F * sample.color;
    return ClampColor(shadedColor);
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
    invisible_places::renderer::pointcloud::PointCloudRenderMode mode,
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
                const float falloff = PointFalloff(style, mode, normalizedRadius, normalizedRadiusSquared);
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
            const float falloff = PointFalloff(style, mode, normalizedRadius, normalizedRadiusSquared);
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

void WritePixelIfCloser(
    const glm::vec3& color,
    float alpha,
    float viewDepth,
    std::uint32_t x,
    std::uint32_t y,
    ExrImage* image) {
    const auto pixelIndex =
        static_cast<std::size_t>(y) * static_cast<std::size_t>(image->width) + static_cast<std::size_t>(x);
    if (pixelIndex >= image->depth.size() || viewDepth >= image->depth[pixelIndex]) {
        return;
    }

    image->depth[pixelIndex] = viewDepth;
    image->beautyR[pixelIndex] = color.r;
    image->beautyG[pixelIndex] = color.g;
    image->beautyB[pixelIndex] = color.b;
    image->alpha[pixelIndex] = alpha;
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

    using invisible_places::renderer::pointcloud::PointCloudRenderMode;

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
        const auto mode = EffectiveOfflineRenderMode(layer.style.renderMode);
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
                    mode,
                    matrices,
                    *image,
                    tile,
                    tileWidth,
                    [&](std::uint32_t x, std::uint32_t y, std::size_t, float, float coveredViewDepth) {
                        const auto pixelIndex =
                            static_cast<std::size_t>(y) * static_cast<std::size_t>(image->width) +
                            static_cast<std::size_t>(x);
                        if (pixelIndex < image->depth.size() && coveredViewDepth < image->depth[pixelIndex]) {
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
        const auto mode = EffectiveOfflineRenderMode(layer.style.renderMode);
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
                    mode,
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

                        const float alpha = std::clamp(sample.opacity * falloff, 0.0F, 0.995F);
                        if (alpha <= 1.0e-5F) {
                            return;
                        }

                        if (mode == PointCloudRenderMode::Solid) {
                            if (coveredViewDepth > image->depth[pixelIndex] + 1.0e-4F) {
                                return;
                            }

                            const glm::vec3 color = ResolveSolidShadedColor(sample, cameraState, coveredViewDepth);
                            image->beautyR[pixelIndex] = color.r;
                            image->beautyG[pixelIndex] = color.g;
                            image->beautyB[pixelIndex] = color.b;
                            image->alpha[pixelIndex] = alpha;
                            return;
                        }

                        if (mode == PointCloudRenderMode::DepthXray) {
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

                            const float gain = std::max(1.0F, sample.emissive) * std::max(0.0F, layer.style.exposure);
                            emissionR[localIndex] += sample.color.r * xrayAlpha * gain;
                            emissionG[localIndex] += sample.color.g * xrayAlpha * gain;
                            emissionB[localIndex] += sample.color.b * xrayAlpha * gain;
                            emissionA[localIndex] += xrayAlpha * gain;
                            return;
                        }

                        if (mode == PointCloudRenderMode::EmissiveHard ||
                            mode == PointCloudRenderMode::EmissiveFeathered) {
                            const float gain = sample.emissive * std::max(0.0F, layer.style.exposure);
                            if (gain <= 1.0e-5F) {
                                return;
                            }

                            emissionR[localIndex] += sample.color.r * alpha * gain;
                            emissionG[localIndex] += sample.color.g * alpha * gain;
                            emissionB[localIndex] += sample.color.b * alpha * gain;
                            emissionA[localIndex] += alpha * gain;
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
