#include "renderer/gsplat/HighQualityGaussianScene.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

#include <glm/vec4.hpp>

namespace invisible_places::renderer::gsplat {

std::vector<HighQualityGaussianLayerSignature> BuildHighQualityGaussianLayerSignatures(
    const std::vector<HighQualityGaussianLayerInput>& layers) {
    std::vector<HighQualityGaussianLayerSignature> signatures;
    signatures.reserve(layers.size());

    for (const auto& layer : layers) {
        HighQualityGaussianLayerSignature signature;
        signature.layerId = layer.layerId;
        signature.revision = layer.revision;
        signatures.push_back(signature);
    }

    return signatures;
}

bool HighQualityGaussianLayerSignaturesMatch(
    const std::vector<HighQualityGaussianLayerSignature>& left,
    const std::vector<HighQualityGaussianLayerSignature>& right) {
    if (left.size() != right.size()) {
        return false;
    }

    for (std::size_t index = 0; index < left.size(); ++index) {
        if (left[index].layerId != right[index].layerId ||
            left[index].revision != right[index].revision) {
            return false;
        }
    }

    return true;
}

std::vector<HighQualityGaussianLayerRange> BuildHighQualityGaussianLayerRanges(
    const std::vector<HighQualityGaussianLayerInput>& layers) {
    std::vector<HighQualityGaussianLayerRange> ranges;
    ranges.reserve(layers.size());

    std::uint32_t mergedStart = 0;
    for (std::uint32_t styleIndex = 0; styleIndex < layers.size(); ++styleIndex) {
        const auto& layer = layers[styleIndex];
        HighQualityGaussianLayerRange range;
        range.layerId = layer.layerId;
        range.styleIndex = styleIndex;
        range.mergedStart = mergedStart;
        range.splatCount = layer.splatCount;
        range.revision = layer.revision;
        ranges.push_back(range);
        mergedStart += layer.splatCount;
    }

    return ranges;
}

std::vector<std::uint32_t> SortHighQualityGaussianIndices(
    const std::vector<io::Float3>& mergedLocalCenters,
    const std::vector<HighQualityGaussianLayerInput>& layers,
    const std::vector<HighQualityGaussianLayerRange>& ranges,
    const glm::mat4& view) {
    std::vector<std::uint32_t> sortedIndices(mergedLocalCenters.size());
    std::iota(sortedIndices.begin(), sortedIndices.end(), 0U);

    if (mergedLocalCenters.empty()) {
        return sortedIndices;
    }

    std::vector<float> depthKeys(mergedLocalCenters.size(), 0.0F);
    float minimumDepth = std::numeric_limits<float>::max();
    float maximumDepth = std::numeric_limits<float>::lowest();

    for (const auto& range : ranges) {
        if (range.styleIndex >= layers.size()) {
            continue;
        }

        const auto& layer = layers[range.styleIndex];
        const glm::mat4 localToWorld = layer.transformEnabled ? layer.localToWorld : glm::mat4{1.0F};
        for (std::uint32_t localIndex = 0; localIndex < range.splatCount; ++localIndex) {
            const auto mergedIndex = range.mergedStart + localIndex;
            const auto& localCenter = mergedLocalCenters[mergedIndex];
            const glm::vec4 worldCenter4 =
                localToWorld * glm::vec4{localCenter.x, localCenter.y, localCenter.z, 1.0F};
            const float inverseW = std::abs(worldCenter4.w) > 1.0e-6F ? (1.0F / worldCenter4.w) : 1.0F;
            const glm::vec4 worldCenter = glm::vec4{
                worldCenter4.x * inverseW,
                worldCenter4.y * inverseW,
                worldCenter4.z * inverseW,
                1.0F};
            const glm::vec4 viewCenter = view * worldCenter;
            depthKeys[mergedIndex] = -viewCenter.z;
            minimumDepth = std::min(minimumDepth, depthKeys[mergedIndex]);
            maximumDepth = std::max(maximumDepth, depthKeys[mergedIndex]);
        }
    }

    const float depthRange = maximumDepth - minimumDepth;
    if (depthRange <= 1.0e-6F) {
        return sortedIndices;
    }

    constexpr std::uint32_t kBucketCount = 2048U;
    std::vector<std::uint32_t> bucketCounts(kBucketCount, 0U);
    std::vector<std::uint32_t> bucketOffsets(kBucketCount, 0U);
    std::vector<std::uint32_t> bucketCursor(kBucketCount, 0U);
    std::vector<std::uint32_t> bucketIndices(mergedLocalCenters.size(), 0U);

    for (std::uint32_t mergedIndex = 0; mergedIndex < mergedLocalCenters.size(); ++mergedIndex) {
        const float normalizedDepth =
            std::clamp((depthKeys[mergedIndex] - minimumDepth) / depthRange, 0.0F, 1.0F);
        const auto bucketIndex = static_cast<std::uint32_t>(
            normalizedDepth * static_cast<float>(kBucketCount - 1U));
        bucketIndices[mergedIndex] = bucketIndex;
        bucketCounts[bucketIndex] += 1U;
    }

    std::uint32_t nextOffset = 0U;
    for (std::uint32_t bucketIndex = kBucketCount; bucketIndex-- > 0U;) {
        bucketOffsets[bucketIndex] = nextOffset;
        bucketCursor[bucketIndex] = nextOffset;
        nextOffset += bucketCounts[bucketIndex];
    }

    for (std::uint32_t mergedIndex = 0; mergedIndex < mergedLocalCenters.size(); ++mergedIndex) {
        const auto bucketIndex = bucketIndices[mergedIndex];
        sortedIndices[bucketCursor[bucketIndex]++] = mergedIndex;
    }

    return sortedIndices;
}

}  // namespace invisible_places::renderer::gsplat
