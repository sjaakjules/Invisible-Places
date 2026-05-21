#include "renderer/pointcloud/RaycastBvh.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace invisible_places::renderer::pointcloud {

namespace {

struct BuildEntry {
    invisible_places::io::Float3 center{};
    float radius = 0.0F;
    std::uint32_t pointIndex = 0;
};

struct BuildBounds {
    invisible_places::io::Float3 minimum{
        std::numeric_limits<float>::max(),
        std::numeric_limits<float>::max(),
        std::numeric_limits<float>::max()};
    invisible_places::io::Float3 maximum{
        std::numeric_limits<float>::lowest(),
        std::numeric_limits<float>::lowest(),
        std::numeric_limits<float>::lowest()};
    bool valid = false;
};

float EntryCenterAxis(const BuildEntry& entry, int axis) {
    if (axis == 0) {
        return entry.center.x;
    }
    if (axis == 1) {
        return entry.center.y;
    }
    return entry.center.z;
}

void ExpandBounds(BuildBounds* bounds, const BuildEntry& entry) {
    if (bounds == nullptr) {
        return;
    }

    const float radius = std::max(0.0F, entry.radius);
    const invisible_places::io::Float3 minimum{
        entry.center.x - radius,
        entry.center.y - radius,
        entry.center.z - radius};
    const invisible_places::io::Float3 maximum{
        entry.center.x + radius,
        entry.center.y + radius,
        entry.center.z + radius};

    if (!bounds->valid) {
        bounds->minimum = minimum;
        bounds->maximum = maximum;
        bounds->valid = true;
        return;
    }

    bounds->minimum.x = std::min(bounds->minimum.x, minimum.x);
    bounds->minimum.y = std::min(bounds->minimum.y, minimum.y);
    bounds->minimum.z = std::min(bounds->minimum.z, minimum.z);
    bounds->maximum.x = std::max(bounds->maximum.x, maximum.x);
    bounds->maximum.y = std::max(bounds->maximum.y, maximum.y);
    bounds->maximum.z = std::max(bounds->maximum.z, maximum.z);
}

BuildBounds ComputeBounds(
    const std::vector<BuildEntry>& entries,
    std::size_t first,
    std::size_t last) {
    BuildBounds bounds;
    for (std::size_t index = first; index < last; ++index) {
        ExpandBounds(&bounds, entries[index]);
    }
    return bounds;
}

int LongestAxis(const BuildBounds& bounds) {
    const float xExtent = bounds.maximum.x - bounds.minimum.x;
    const float yExtent = bounds.maximum.y - bounds.minimum.y;
    const float zExtent = bounds.maximum.z - bounds.minimum.z;
    if (xExtent >= yExtent && xExtent >= zExtent) {
        return 0;
    }
    if (yExtent >= zExtent) {
        return 1;
    }
    return 2;
}

RaycastBvhNodeGpu MakeNode(const BuildBounds& bounds) {
    RaycastBvhNodeGpu node;
    node.boundsMin = {bounds.minimum.x, bounds.minimum.y, bounds.minimum.z, 0.0F};
    node.boundsMax = {bounds.maximum.x, bounds.maximum.y, bounds.maximum.z, 0.0F};
    return node;
}

std::uint32_t BuildNode(
    std::vector<BuildEntry>* entries,
    std::size_t first,
    std::size_t last,
    std::uint32_t maxLeafSize,
    LinearRaycastBvh* bvh) {
    const auto nodeIndex = static_cast<std::uint32_t>(bvh->nodes.size());
    const auto bounds = ComputeBounds(*entries, first, last);
    bvh->nodes.push_back(MakeNode(bounds));

    const std::size_t count = last - first;
    if (count <= std::max<std::uint32_t>(1U, maxLeafSize)) {
        const auto firstPoint = static_cast<std::uint32_t>(bvh->pointIndices.size());
        for (std::size_t index = first; index < last; ++index) {
            bvh->pointIndices.push_back((*entries)[index].pointIndex);
        }
        bvh->nodes[nodeIndex].meta = {
            0U,
            0U,
            firstPoint,
            static_cast<std::uint32_t>(count)};
        return nodeIndex;
    }

    const int axis = LongestAxis(bounds);
    const std::size_t middle = first + (count / 2U);
    std::nth_element(
        entries->begin() + static_cast<std::ptrdiff_t>(first),
        entries->begin() + static_cast<std::ptrdiff_t>(middle),
        entries->begin() + static_cast<std::ptrdiff_t>(last),
        [axis](const BuildEntry& left, const BuildEntry& right) {
            const float leftCenter = EntryCenterAxis(left, axis);
            const float rightCenter = EntryCenterAxis(right, axis);
            if (leftCenter == rightCenter) {
                return left.pointIndex < right.pointIndex;
            }
            return leftCenter < rightCenter;
        });

    std::sort(
        entries->begin() + static_cast<std::ptrdiff_t>(first),
        entries->begin() + static_cast<std::ptrdiff_t>(middle),
        [axis](const BuildEntry& left, const BuildEntry& right) {
            const float leftCenter = EntryCenterAxis(left, axis);
            const float rightCenter = EntryCenterAxis(right, axis);
            if (leftCenter == rightCenter) {
                return left.pointIndex < right.pointIndex;
            }
            return leftCenter < rightCenter;
        });
    std::sort(
        entries->begin() + static_cast<std::ptrdiff_t>(middle),
        entries->begin() + static_cast<std::ptrdiff_t>(last),
        [axis](const BuildEntry& left, const BuildEntry& right) {
            const float leftCenter = EntryCenterAxis(left, axis);
            const float rightCenter = EntryCenterAxis(right, axis);
            if (leftCenter == rightCenter) {
                return left.pointIndex < right.pointIndex;
            }
            return leftCenter < rightCenter;
        });

    const auto leftChild = BuildNode(entries, first, middle, maxLeafSize, bvh);
    const auto rightChild = BuildNode(entries, middle, last, maxLeafSize, bvh);
    bvh->nodes[nodeIndex].meta = {leftChild, rightChild, 0U, 0U};
    return nodeIndex;
}

}  // namespace

LinearRaycastBvh BuildLinearRaycastBvh(
    const std::vector<RaycastBvhBuildPoint>& points,
    std::uint32_t maxLeafSize) {
    LinearRaycastBvh bvh;
    if (points.empty()) {
        return bvh;
    }

    std::vector<BuildEntry> entries;
    entries.reserve(points.size());
    for (const auto& point : points) {
        if (!std::isfinite(point.center.x) ||
            !std::isfinite(point.center.y) ||
            !std::isfinite(point.center.z)) {
            continue;
        }
        entries.push_back(
            {.center = point.center,
             .radius = std::isfinite(point.radius) ? std::max(0.0F, point.radius) : 0.0F,
             .pointIndex = point.pointIndex});
    }
    if (entries.empty()) {
        return bvh;
    }

    std::sort(entries.begin(), entries.end(), [](const BuildEntry& left, const BuildEntry& right) {
        return left.pointIndex < right.pointIndex;
    });
    bvh.nodes.reserve(entries.size() * 2U);
    bvh.pointIndices.reserve(entries.size());
    static_cast<void>(BuildNode(&entries, 0U, entries.size(), maxLeafSize, &bvh));
    return bvh;
}

}  // namespace invisible_places::renderer::pointcloud
