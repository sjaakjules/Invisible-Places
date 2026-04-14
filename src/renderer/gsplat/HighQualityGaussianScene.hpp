#pragma once

#include "io/PointCloudData.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

#include <glm/mat4x4.hpp>

namespace invisible_places::renderer::gsplat {

struct HighQualityGaussianLayerInput {
    std::size_t layerId = 0;
    std::uint64_t revision = 0;
    std::uint32_t splatCount = 0;
    glm::mat4 localToWorld{1.0F};
    bool transformEnabled = true;
};

struct HighQualityGaussianLayerSignature {
    std::size_t layerId = 0;
    std::uint64_t revision = 0;
};

struct HighQualityGaussianLayerRange {
    std::size_t layerId = 0;
    std::uint32_t styleIndex = 0;
    std::uint32_t mergedStart = 0;
    std::uint32_t splatCount = 0;
    std::uint64_t revision = 0;
};

std::vector<HighQualityGaussianLayerSignature> BuildHighQualityGaussianLayerSignatures(
    const std::vector<HighQualityGaussianLayerInput>& layers);

bool HighQualityGaussianLayerSignaturesMatch(
    const std::vector<HighQualityGaussianLayerSignature>& left,
    const std::vector<HighQualityGaussianLayerSignature>& right);

std::vector<HighQualityGaussianLayerRange> BuildHighQualityGaussianLayerRanges(
    const std::vector<HighQualityGaussianLayerInput>& layers);

std::vector<std::uint32_t> SortHighQualityGaussianIndices(
    const std::vector<io::Float3>& mergedLocalCenters,
    const std::vector<HighQualityGaussianLayerInput>& layers,
    const std::vector<HighQualityGaussianLayerRange>& ranges,
    const glm::mat4& view);

}  // namespace invisible_places::renderer::gsplat
