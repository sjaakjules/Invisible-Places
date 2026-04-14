#pragma once

#include "io/PointCloudData.hpp"
#include "io/TransformMatrix.hpp"

#include <array>
#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

namespace invisible_places::io {

struct LoadedGaussianSplat {
    static constexpr std::size_t kShCoefficientsPerSplat = 48;

    std::filesystem::path sourcePath;
    std::filesystem::path transformPath;
    std::string layerName;
    std::vector<Float3> centers;
    std::vector<std::array<float, 3>> scales;
    std::vector<std::array<float, 4>> rotations;
    std::vector<float> opacities;
    std::vector<float> shCoefficients;
    Matrix4d localToWorld{};
    Bounds3f localBounds;
    Bounds3f bounds;
    Float3 localFocusPoint{};
    Float3 focusPoint{};
    bool hasLocalFocusPoint = false;
    bool hasFocusPoint = false;

    [[nodiscard]] std::size_t SplatCount() const { return centers.size(); }
    [[nodiscard]] std::size_t ShCoefficientOffset(std::size_t splatIndex) const {
        return splatIndex * kShCoefficientsPerSplat;
    }
};

struct GaussianSplatLoadResult {
    LoadedGaussianSplat splats;
    std::string errorMessage;
    bool success = false;
};

GaussianSplatLoadResult LoadGaussianSplat(
    const std::filesystem::path& filePath,
    const std::filesystem::path& transformPath);

}  // namespace invisible_places::io
