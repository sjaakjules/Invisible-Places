#include "io/GaussianSplatData.hpp"

#include "io/PlyHeader.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string_view>
#include <vector>

namespace invisible_places::io {

namespace {

enum class ScalarType {
    Int8,
    UInt8,
    Int16,
    UInt16,
    Int32,
    UInt32,
    Float32,
    Float64
};

enum class PropertySemantic {
    Skip,
    PositionX,
    PositionY,
    PositionZ,
    FeatureDc,
    FeatureRest,
    Opacity,
    Scale,
    Rotation
};

struct PropertyLayout {
    PropertySemantic semantic = PropertySemantic::Skip;
    ScalarType type = ScalarType::Float32;
    std::uint32_t offset = 0;
    std::uint32_t size = 0;
    std::uint32_t componentIndex = 0;
};

struct GaussianSplatLayout {
    std::vector<PropertyLayout> properties;
    std::uint32_t recordSize = 0;
    bool hasPositionX = false;
    bool hasPositionY = false;
    bool hasPositionZ = false;
    std::uint32_t dcCount = 0;
    std::uint32_t restCount = 0;
    std::uint32_t scaleCount = 0;
    std::uint32_t rotationCount = 0;
    bool hasOpacity = false;
};

constexpr std::size_t kMaxFocusSamples = 16384;

template <typename T>
T ReadScalar(const std::byte* bytes) {
    T value{};
    std::memcpy(&value, bytes, sizeof(T));
    return value;
}

std::optional<ScalarType> ParseScalarType(std::string_view typeName) {
    if (typeName == "char" || typeName == "int8") {
        return ScalarType::Int8;
    }
    if (typeName == "uchar" || typeName == "uint8") {
        return ScalarType::UInt8;
    }
    if (typeName == "short" || typeName == "int16") {
        return ScalarType::Int16;
    }
    if (typeName == "ushort" || typeName == "uint16") {
        return ScalarType::UInt16;
    }
    if (typeName == "int" || typeName == "int32") {
        return ScalarType::Int32;
    }
    if (typeName == "uint" || typeName == "uint32") {
        return ScalarType::UInt32;
    }
    if (typeName == "float" || typeName == "float32") {
        return ScalarType::Float32;
    }
    if (typeName == "double" || typeName == "float64") {
        return ScalarType::Float64;
    }

    return std::nullopt;
}

std::uint32_t ScalarTypeSize(ScalarType type) {
    switch (type) {
        case ScalarType::Int8:
        case ScalarType::UInt8:
            return 1;
        case ScalarType::Int16:
        case ScalarType::UInt16:
            return 2;
        case ScalarType::Int32:
        case ScalarType::UInt32:
        case ScalarType::Float32:
            return 4;
        case ScalarType::Float64:
            return 8;
    }

    return 0;
}

double ReadScalarAsDouble(const std::byte* bytes, ScalarType type) {
    switch (type) {
        case ScalarType::Int8:
            return static_cast<double>(ReadScalar<std::int8_t>(bytes));
        case ScalarType::UInt8:
            return static_cast<double>(ReadScalar<std::uint8_t>(bytes));
        case ScalarType::Int16:
            return static_cast<double>(ReadScalar<std::int16_t>(bytes));
        case ScalarType::UInt16:
            return static_cast<double>(ReadScalar<std::uint16_t>(bytes));
        case ScalarType::Int32:
            return static_cast<double>(ReadScalar<std::int32_t>(bytes));
        case ScalarType::UInt32:
            return static_cast<double>(ReadScalar<std::uint32_t>(bytes));
        case ScalarType::Float32:
            return static_cast<double>(ReadScalar<float>(bytes));
        case ScalarType::Float64:
            return ReadScalar<double>(bytes);
    }

    return 0.0;
}

bool StartsWith(std::string_view value, std::string_view prefix) {
    return value.substr(0, prefix.size()) == prefix;
}

std::optional<std::uint32_t> ParseIndexedSuffix(std::string_view name, std::string_view prefix) {
    if (!StartsWith(name, prefix)) {
        return std::nullopt;
    }

    const auto suffix = name.substr(prefix.size());
    if (suffix.empty()) {
        return std::nullopt;
    }

    std::uint32_t value = 0;
    for (const char character : suffix) {
        if (character < '0' || character > '9') {
            return std::nullopt;
        }
        value = (value * 10U) + static_cast<std::uint32_t>(character - '0');
    }
    return value;
}

std::optional<GaussianSplatLayout> BuildGaussianSplatLayout(
    const PlyHeader& header,
    std::string* errorMessage) {
    GaussianSplatLayout layout;
    layout.properties.reserve(header.properties.size());

    for (const auto& property : header.properties) {
        const auto scalarType = ParseScalarType(property.type);
        if (!scalarType.has_value()) {
            if (errorMessage != nullptr) {
                *errorMessage = "Unsupported PLY property type: " + property.type;
            }
            return std::nullopt;
        }

        PropertyLayout propertyLayout;
        propertyLayout.type = scalarType.value();
        propertyLayout.offset = layout.recordSize;
        propertyLayout.size = ScalarTypeSize(propertyLayout.type);

        if (property.name == "x") {
            propertyLayout.semantic = PropertySemantic::PositionX;
            layout.hasPositionX = true;
        } else if (property.name == "y") {
            propertyLayout.semantic = PropertySemantic::PositionY;
            layout.hasPositionY = true;
        } else if (property.name == "z") {
            propertyLayout.semantic = PropertySemantic::PositionZ;
            layout.hasPositionZ = true;
        } else if (const auto index = ParseIndexedSuffix(property.name, "f_dc_"); index.has_value()) {
            propertyLayout.semantic = PropertySemantic::FeatureDc;
            propertyLayout.componentIndex = index.value();
            layout.dcCount = std::max(layout.dcCount, index.value() + 1U);
        } else if (const auto index = ParseIndexedSuffix(property.name, "f_rest_"); index.has_value()) {
            propertyLayout.semantic = PropertySemantic::FeatureRest;
            propertyLayout.componentIndex = index.value();
            layout.restCount = std::max(layout.restCount, index.value() + 1U);
        } else if (property.name == "opacity") {
            propertyLayout.semantic = PropertySemantic::Opacity;
            layout.hasOpacity = true;
        } else if (const auto index = ParseIndexedSuffix(property.name, "scale_"); index.has_value()) {
            propertyLayout.semantic = PropertySemantic::Scale;
            propertyLayout.componentIndex = index.value();
            layout.scaleCount = std::max(layout.scaleCount, index.value() + 1U);
        } else if (const auto index = ParseIndexedSuffix(property.name, "rot_"); index.has_value()) {
            propertyLayout.semantic = PropertySemantic::Rotation;
            propertyLayout.componentIndex = index.value();
            layout.rotationCount = std::max(layout.rotationCount, index.value() + 1U);
        }

        layout.recordSize += propertyLayout.size;
        layout.properties.push_back(propertyLayout);
    }

    if (!(layout.hasPositionX && layout.hasPositionY && layout.hasPositionZ)) {
        if (errorMessage != nullptr) {
            *errorMessage = "Gaussian splat PLY is missing x/y/z properties.";
        }
        return std::nullopt;
    }

    if (layout.dcCount < 3U || layout.restCount < 45U || !layout.hasOpacity ||
        layout.scaleCount < 3U || layout.rotationCount < 4U) {
        if (errorMessage != nullptr) {
            *errorMessage =
                "Gaussian splat PLY does not expose the expected Polycam-style SH/opacity/scale/rotation fields.";
        }
        return std::nullopt;
    }

    return layout;
}

float DecodeOpacity(double rawOpacity) {
    const auto clamped = std::clamp(rawOpacity, -32.0, 32.0);
    return static_cast<float>(1.0 / (1.0 + std::exp(-clamped)));
}

float DecodeScale(double rawScale) {
    return static_cast<float>(std::exp(std::clamp(rawScale, -16.0, 16.0)));
}

std::array<float, 4> NormalizeQuaternion(std::array<float, 4> quaternion) {
    const auto lengthSquared =
        (quaternion[0] * quaternion[0]) +
        (quaternion[1] * quaternion[1]) +
        (quaternion[2] * quaternion[2]) +
        (quaternion[3] * quaternion[3]);
    if (lengthSquared <= std::numeric_limits<float>::epsilon()) {
        return {1.0F, 0.0F, 0.0F, 0.0F};
    }

    const auto inverseLength = 1.0F / std::sqrt(lengthSquared);
    for (auto& value : quaternion) {
        value *= inverseLength;
    }
    return quaternion;
}

Float3 TransformPoint(const Matrix4d& matrix, const Float3& point) {
    const double x = point.x;
    const double y = point.y;
    const double z = point.z;

    const double tx = (matrix.At(0, 0) * x) + (matrix.At(0, 1) * y) + (matrix.At(0, 2) * z) + matrix.At(0, 3);
    const double ty = (matrix.At(1, 0) * x) + (matrix.At(1, 1) * y) + (matrix.At(1, 2) * z) + matrix.At(1, 3);
    const double tz = (matrix.At(2, 0) * x) + (matrix.At(2, 1) * y) + (matrix.At(2, 2) * z) + matrix.At(2, 3);

    return {static_cast<float>(tx), static_cast<float>(ty), static_cast<float>(tz)};
}

Float3 BoundsCenter(const Bounds3f& bounds) {
    return {
        0.5F * (bounds.minimum.x + bounds.maximum.x),
        0.5F * (bounds.minimum.y + bounds.maximum.y),
        0.5F * (bounds.minimum.z + bounds.maximum.z),
    };
}

float MedianComponent(std::vector<float>* values) {
    if (values == nullptr || values->empty()) {
        return 0.0F;
    }

    const auto middle = values->begin() + static_cast<std::ptrdiff_t>(values->size() / 2U);
    std::nth_element(values->begin(), middle, values->end());
    return *middle;
}

Float3 ComputeRepresentativeFocusPoint(const Bounds3f& bounds, const std::vector<Float3>& focusSamples) {
    if (focusSamples.empty()) {
        return bounds.valid ? BoundsCenter(bounds) : Float3{};
    }

    std::vector<float> xValues;
    std::vector<float> yValues;
    std::vector<float> zValues;
    xValues.reserve(focusSamples.size());
    yValues.reserve(focusSamples.size());
    zValues.reserve(focusSamples.size());

    for (const auto& sample : focusSamples) {
        xValues.push_back(sample.x);
        yValues.push_back(sample.y);
        zValues.push_back(sample.z);
    }

    return {
        MedianComponent(&xValues),
        MedianComponent(&yValues),
        MedianComponent(&zValues),
    };
}

std::size_t RecommendedSplatsPerChunk(std::uint32_t recordSize) {
    constexpr std::size_t targetChunkBytes = 8U * 1024U * 1024U;
    return std::max<std::size_t>(1U, targetChunkBytes / std::max<std::uint32_t>(1U, recordSize));
}

}  // namespace

GaussianSplatLoadResult LoadGaussianSplat(
    const std::filesystem::path& filePath,
    const std::filesystem::path& transformPath) {
    const auto headerResult = ParsePlyHeader(filePath);
    if (!headerResult.success) {
        return {.errorMessage = headerResult.errorMessage, .success = false};
    }

    if (headerResult.header.format != "binary_little_endian") {
        return {.errorMessage = "Only binary little-endian Gaussian splat PLY files are supported.", .success = false};
    }

    const auto matrixResult = ParseMatrix4x4(transformPath);
    if (!matrixResult.success) {
        return {.errorMessage = matrixResult.errorMessage, .success = false};
    }

    std::string layoutError;
    const auto layout = BuildGaussianSplatLayout(headerResult.header, &layoutError);
    if (!layout.has_value()) {
        return {.errorMessage = layoutError, .success = false};
    }

    std::ifstream input{filePath, std::ios::binary};
    if (!input.is_open()) {
        return {.errorMessage = "Unable to open Gaussian splat PLY data.", .success = false};
    }
    input.seekg(static_cast<std::streamoff>(headerResult.header.dataOffsetBytes), std::ios::beg);

    LoadedGaussianSplat loadedSplats;
    loadedSplats.sourcePath = filePath;
    loadedSplats.transformPath = transformPath;
    loadedSplats.layerName = filePath.stem().string();
    loadedSplats.localToWorld = matrixResult.matrix;
    loadedSplats.centers.resize(static_cast<std::size_t>(headerResult.header.vertexCount));
    loadedSplats.scales.resize(static_cast<std::size_t>(headerResult.header.vertexCount));
    loadedSplats.rotations.resize(static_cast<std::size_t>(headerResult.header.vertexCount));
    loadedSplats.opacities.resize(static_cast<std::size_t>(headerResult.header.vertexCount));
    loadedSplats.shCoefficients.resize(
        static_cast<std::size_t>(headerResult.header.vertexCount) * LoadedGaussianSplat::kShCoefficientsPerSplat,
        0.0F);

    std::vector<Float3> focusSamples;
    std::vector<Float3> localFocusSamples;
    focusSamples.reserve(std::min<std::uint64_t>(
        headerResult.header.vertexCount,
        static_cast<std::uint64_t>(kMaxFocusSamples)));
    localFocusSamples.reserve(std::min<std::uint64_t>(
        headerResult.header.vertexCount,
        static_cast<std::uint64_t>(kMaxFocusSamples)));

    const auto splatsPerChunk = RecommendedSplatsPerChunk(layout->recordSize);
    std::vector<std::byte> chunkBytes(splatsPerChunk * layout->recordSize);

    std::size_t splatIndex = 0;
    while (splatIndex < loadedSplats.SplatCount()) {
        const auto remaining = loadedSplats.SplatCount() - splatIndex;
        const auto splatsThisChunk = std::min<std::size_t>(remaining, splatsPerChunk);
        const auto bytesThisChunk = splatsThisChunk * layout->recordSize;

        input.read(reinterpret_cast<char*>(chunkBytes.data()), static_cast<std::streamsize>(bytesThisChunk));
        if (input.gcount() != static_cast<std::streamsize>(bytesThisChunk)) {
            return {.errorMessage = "Unexpected EOF while reading Gaussian splat payload.", .success = false};
        }

        for (std::size_t chunkIndex = 0; chunkIndex < splatsThisChunk; ++chunkIndex, ++splatIndex) {
            const auto* record = chunkBytes.data() + (chunkIndex * layout->recordSize);
            Float3 center{};
            std::array<float, 3> decodedScale{1.0F, 1.0F, 1.0F};
            std::array<float, 4> decodedRotation{1.0F, 0.0F, 0.0F, 0.0F};
            float decodedOpacity = 1.0F;

            for (const auto& property : layout->properties) {
                const auto* propertyBytes = record + property.offset;
                const auto value = ReadScalarAsDouble(propertyBytes, property.type);

                switch (property.semantic) {
                    case PropertySemantic::PositionX:
                        center.x = static_cast<float>(value);
                        break;
                    case PropertySemantic::PositionY:
                        center.y = static_cast<float>(value);
                        break;
                    case PropertySemantic::PositionZ:
                        center.z = static_cast<float>(value);
                        break;
                    case PropertySemantic::FeatureDc:
                        if (property.componentIndex < 3U) {
                            loadedSplats.shCoefficients[
                                loadedSplats.ShCoefficientOffset(splatIndex) + property.componentIndex] =
                                static_cast<float>(value);
                        }
                        break;
                    case PropertySemantic::FeatureRest:
                        if (property.componentIndex < 45U) {
                            loadedSplats.shCoefficients[
                                loadedSplats.ShCoefficientOffset(splatIndex) + 3U + property.componentIndex] =
                                static_cast<float>(value);
                        }
                        break;
                    case PropertySemantic::Opacity:
                        decodedOpacity = DecodeOpacity(value);
                        break;
                    case PropertySemantic::Scale:
                        if (property.componentIndex < decodedScale.size()) {
                            decodedScale[property.componentIndex] = DecodeScale(value);
                        }
                        break;
                    case PropertySemantic::Rotation:
                        if (property.componentIndex < decodedRotation.size()) {
                            decodedRotation[property.componentIndex] = static_cast<float>(value);
                        }
                        break;
                    case PropertySemantic::Skip:
                        break;
                }
            }

            loadedSplats.centers[splatIndex] = center;
            loadedSplats.scales[splatIndex] = decodedScale;
            loadedSplats.rotations[splatIndex] = NormalizeQuaternion(decodedRotation);
            loadedSplats.opacities[splatIndex] = decodedOpacity;

            loadedSplats.localBounds.Expand(center);
            if (localFocusSamples.size() < kMaxFocusSamples) {
                localFocusSamples.push_back(center);
            }

            const auto transformedCenter = TransformPoint(loadedSplats.localToWorld, center);
            loadedSplats.bounds.Expand(transformedCenter);
            if (focusSamples.size() < kMaxFocusSamples) {
                focusSamples.push_back(transformedCenter);
            }
        }
    }

    loadedSplats.localFocusPoint = ComputeRepresentativeFocusPoint(loadedSplats.localBounds, localFocusSamples);
    loadedSplats.hasLocalFocusPoint = loadedSplats.localBounds.valid;
    loadedSplats.focusPoint = ComputeRepresentativeFocusPoint(loadedSplats.bounds, focusSamples);
    loadedSplats.hasFocusPoint = loadedSplats.bounds.valid;

    return {.splats = std::move(loadedSplats), .success = true};
}

}  // namespace invisible_places::io
