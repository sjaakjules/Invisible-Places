#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <string>
#include <vector>

namespace invisible_places::io {

struct Float3 {
    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;
};

struct Bounds3f {
    Float3 minimum{};
    Float3 maximum{};
    bool valid = false;

    void Expand(const Float3& point);
};

struct ScalarFieldStats {
    std::string name;
    float minimum = 0.0F;
    float maximum = 0.0F;
    std::uint64_t count = 0;
    bool valid = false;

    void Include(float value);
};

struct LoadedPointCloud {
    std::filesystem::path sourcePath;
    std::string layerName;
    std::vector<Float3> positions;
    std::vector<Float3> normals;
    std::vector<std::uint32_t> packedColors;
    std::vector<float> scalarFieldValues;
    std::vector<ScalarFieldStats> scalarFields;
    Bounds3f bounds;
    Float3 focusPoint{};
    bool hasSourceRgb = false;
    bool hasNormals = false;
    bool hasFocusPoint = false;

    [[nodiscard]] std::size_t PointCount() const { return positions.size(); }
    [[nodiscard]] std::size_t ScalarFieldCount() const { return scalarFields.size(); }
    [[nodiscard]] std::size_t ScalarFieldValueIndex(
        std::size_t fieldIndex,
        std::size_t pointIndex) const;
};

struct PointCloudLoadResult {
    LoadedPointCloud cloud;
    std::string errorMessage;
    bool success = false;
};

struct PointCloudLoadProgress {
    std::uint64_t pointsRead = 0;
    std::uint64_t totalPoints = 0;
    std::uint64_t bytesRead = 0;
    std::uint64_t totalBytes = 0;
};

using PointCloudLoadProgressCallback = std::function<void(const PointCloudLoadProgress&)>;

PointCloudLoadResult LoadPointCloud(
    const std::filesystem::path& filePath,
    const PointCloudLoadProgressCallback& progressCallback = {});

}  // namespace invisible_places::io
