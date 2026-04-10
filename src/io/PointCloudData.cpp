#include "io/PointCloudData.hpp"

#include "io/PlyHeader.hpp"

#include <algorithm>
#include <array>
#include <cstring>
#include <fstream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string_view>

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
    ColorR,
    ColorG,
    ColorB,
    ScalarField
};

struct PropertyLayout {
    PropertySemantic semantic = PropertySemantic::Skip;
    ScalarType type = ScalarType::Float32;
    std::uint32_t offset = 0;
    std::uint32_t size = 0;
    std::uint32_t scalarFieldIndex = 0;
};

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

template <typename T>
T ReadScalar(const std::byte* bytes) {
    T value{};
    std::memcpy(&value, bytes, sizeof(T));
    return value;
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

std::uint8_t ReadScalarAsByte(const std::byte* bytes, ScalarType type) {
    const auto value = std::clamp(ReadScalarAsDouble(bytes, type), 0.0, 255.0);
    return static_cast<std::uint8_t>(value);
}

std::uint32_t PackRgba8(std::uint8_t red, std::uint8_t green, std::uint8_t blue) {
    return static_cast<std::uint32_t>(red) |
           (static_cast<std::uint32_t>(green) << 8U) |
           (static_cast<std::uint32_t>(blue) << 16U) |
           (0xFFU << 24U);
}

bool StartsWith(std::string_view value, std::string_view prefix) {
    return value.substr(0, prefix.size()) == prefix;
}

std::string ScalarFieldDisplayName(std::string_view propertyName) {
    constexpr std::string_view prefix = "scalar_";
    if (!StartsWith(propertyName, prefix)) {
        return std::string{propertyName};
    }

    return std::string{propertyName.substr(prefix.size())};
}

struct PointCloudLayout {
    std::vector<PropertyLayout> properties;
    std::uint32_t recordSize = 0;
    std::uint32_t scalarFieldCount = 0;
};

constexpr std::size_t kMaxFocusSamples = 16384;

std::optional<PointCloudLayout> BuildPointCloudLayout(const PlyHeader& header, std::string* errorMessage) {
    PointCloudLayout layout;
    layout.properties.reserve(header.properties.size());

    std::uint32_t scalarFieldIndex = 0;
    bool sawX = false;
    bool sawY = false;
    bool sawZ = false;

    for (const auto& property : header.properties) {
        const auto scalarType = ParseScalarType(property.type);
        if (!scalarType.has_value()) {
            if (errorMessage != nullptr) {
                *errorMessage = "Unsupported PLY property type: " + property.type;
            }
            return std::nullopt;
        }

        PropertyLayout layoutEntry;
        layoutEntry.type = scalarType.value();
        layoutEntry.offset = layout.recordSize;
        layoutEntry.size = ScalarTypeSize(layoutEntry.type);

        if (property.name == "x") {
            layoutEntry.semantic = PropertySemantic::PositionX;
            sawX = true;
        } else if (property.name == "y") {
            layoutEntry.semantic = PropertySemantic::PositionY;
            sawY = true;
        } else if (property.name == "z") {
            layoutEntry.semantic = PropertySemantic::PositionZ;
            sawZ = true;
        } else if (property.name == "red") {
            layoutEntry.semantic = PropertySemantic::ColorR;
        } else if (property.name == "green") {
            layoutEntry.semantic = PropertySemantic::ColorG;
        } else if (property.name == "blue") {
            layoutEntry.semantic = PropertySemantic::ColorB;
        } else if (StartsWith(property.name, "scalar_")) {
            layoutEntry.semantic = PropertySemantic::ScalarField;
            layoutEntry.scalarFieldIndex = scalarFieldIndex++;
        }

        layout.recordSize += layoutEntry.size;
        layout.properties.push_back(layoutEntry);
    }

    if (!(sawX && sawY && sawZ)) {
        if (errorMessage != nullptr) {
            *errorMessage = "PLY point cloud is missing x/y/z properties.";
        }
        return std::nullopt;
    }

    layout.scalarFieldCount = scalarFieldIndex;
    return layout;
}

std::size_t RecommendedPointsPerChunk(std::uint32_t recordSize) {
    constexpr std::size_t targetChunkBytes = 8U * 1024U * 1024U;
    return std::max<std::size_t>(1, targetChunkBytes / std::max<std::uint32_t>(1, recordSize));
}

float MedianComponent(std::vector<float>* values) {
    if (values == nullptr || values->empty()) {
        return 0.0F;
    }

    const auto middle = values->begin() + static_cast<std::ptrdiff_t>(values->size() / 2U);
    std::nth_element(values->begin(), middle, values->end());
    return *middle;
}

Float3 BoundsCenter(const Bounds3f& bounds) {
    return {
        0.5F * (bounds.minimum.x + bounds.maximum.x),
        0.5F * (bounds.minimum.y + bounds.maximum.y),
        0.5F * (bounds.minimum.z + bounds.maximum.z),
    };
}

Float3 ComputeRepresentativeFocusPoint(
    const Bounds3f& bounds,
    const std::vector<Float3>& focusSamples) {
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

    const Float3 targetMedian{
        MedianComponent(&xValues),
        MedianComponent(&yValues),
        MedianComponent(&zValues),
    };

    const auto nearestSample = std::min_element(
        focusSamples.begin(),
        focusSamples.end(),
        [&targetMedian](const Float3& left, const Float3& right) {
            const auto leftDistance =
                ((left.x - targetMedian.x) * (left.x - targetMedian.x)) +
                ((left.y - targetMedian.y) * (left.y - targetMedian.y)) +
                ((left.z - targetMedian.z) * (left.z - targetMedian.z));
            const auto rightDistance =
                ((right.x - targetMedian.x) * (right.x - targetMedian.x)) +
                ((right.y - targetMedian.y) * (right.y - targetMedian.y)) +
                ((right.z - targetMedian.z) * (right.z - targetMedian.z));
            return leftDistance < rightDistance;
        });

    return nearestSample != focusSamples.end() ? *nearestSample : targetMedian;
}

}  // namespace

void Bounds3f::Expand(const Float3& point) {
    if (!valid) {
        minimum = point;
        maximum = point;
        valid = true;
        return;
    }

    minimum.x = std::min(minimum.x, point.x);
    minimum.y = std::min(minimum.y, point.y);
    minimum.z = std::min(minimum.z, point.z);

    maximum.x = std::max(maximum.x, point.x);
    maximum.y = std::max(maximum.y, point.y);
    maximum.z = std::max(maximum.z, point.z);
}

void ScalarFieldStats::Include(float value) {
    if (!valid) {
        minimum = value;
        maximum = value;
        count = 1;
        valid = true;
        return;
    }

    minimum = std::min(minimum, value);
    maximum = std::max(maximum, value);
    ++count;
}

std::size_t LoadedPointCloud::ScalarFieldValueIndex(std::size_t fieldIndex, std::size_t pointIndex) const {
    return (fieldIndex * PointCount()) + pointIndex;
}

PointCloudLoadResult LoadPointCloud(const std::filesystem::path& filePath) {
    const auto headerResult = ParsePlyHeader(filePath);
    if (!headerResult.success) {
        return {.errorMessage = headerResult.errorMessage, .success = false};
    }

    const auto& header = headerResult.header;
    if (header.format != "binary_little_endian") {
        return {.errorMessage = "Only binary_little_endian PLY point clouds are supported.", .success = false};
    }

    std::string layoutError;
    const auto layout = BuildPointCloudLayout(header, &layoutError);
    if (!layout.has_value()) {
        return {.errorMessage = layoutError, .success = false};
    }

    std::ifstream input{filePath, std::ios::binary};
    if (!input.is_open()) {
        return {.errorMessage = "Unable to open point cloud file.", .success = false};
    }

    input.seekg(static_cast<std::streamoff>(header.dataOffsetBytes), std::ios::beg);
    if (!input.good()) {
        return {.errorMessage = "Failed to seek to PLY payload.", .success = false};
    }

    LoadedPointCloud cloud;
    cloud.sourcePath = filePath;
    cloud.layerName = filePath.stem().string();
    cloud.hasSourceRgb = header.HasColorRgb();

    try {
        cloud.positions.resize(static_cast<std::size_t>(header.vertexCount));
        cloud.packedColors.resize(static_cast<std::size_t>(header.vertexCount), PackRgba8(255, 255, 255));
        cloud.scalarFields.reserve(layout->scalarFieldCount);
        for (const auto& property : header.properties) {
            if (!StartsWith(property.name, "scalar_")) {
                continue;
            }

            cloud.scalarFields.push_back({.name = ScalarFieldDisplayName(property.name)});
        }

        cloud.scalarFieldValues.resize(
            static_cast<std::size_t>(header.vertexCount) * cloud.scalarFields.size());
    } catch (const std::exception& error) {
        return {.errorMessage = std::string{"Point cloud allocation failed: "} + error.what(), .success = false};
    }

    const auto pointsPerChunk = RecommendedPointsPerChunk(layout->recordSize);
    const auto focusSampleStride =
        std::max<std::uint64_t>(1ULL, header.vertexCount / static_cast<std::uint64_t>(kMaxFocusSamples));
    std::vector<std::byte> chunkBuffer(pointsPerChunk * layout->recordSize);
    std::vector<Float3> focusSamples;
    focusSamples.reserve(std::min<std::uint64_t>(header.vertexCount, kMaxFocusSamples));

    for (std::uint64_t pointStart = 0; pointStart < header.vertexCount; pointStart += pointsPerChunk) {
        const auto remaining = header.vertexCount - pointStart;
        const auto pointsThisChunk = static_cast<std::size_t>(std::min<std::uint64_t>(remaining, pointsPerChunk));
        const auto bytesToRead = pointsThisChunk * layout->recordSize;

        input.read(reinterpret_cast<char*>(chunkBuffer.data()), static_cast<std::streamsize>(bytesToRead));
        if (input.gcount() != static_cast<std::streamsize>(bytesToRead)) {
            return {.errorMessage = "Unexpected EOF while reading point cloud payload.", .success = false};
        }

        for (std::size_t localIndex = 0; localIndex < pointsThisChunk; ++localIndex) {
            const auto globalIndex = static_cast<std::size_t>(pointStart) + localIndex;
            const auto* recordBytes = chunkBuffer.data() + (localIndex * layout->recordSize);

            Float3 position{};
            std::uint8_t red = 255;
            std::uint8_t green = 255;
            std::uint8_t blue = 255;

            for (const auto& property : layout->properties) {
                const auto* propertyBytes = recordBytes + property.offset;

                switch (property.semantic) {
                    case PropertySemantic::PositionX:
                        position.x = static_cast<float>(ReadScalarAsDouble(propertyBytes, property.type));
                        break;
                    case PropertySemantic::PositionY:
                        position.y = static_cast<float>(ReadScalarAsDouble(propertyBytes, property.type));
                        break;
                    case PropertySemantic::PositionZ:
                        position.z = static_cast<float>(ReadScalarAsDouble(propertyBytes, property.type));
                        break;
                    case PropertySemantic::ColorR:
                        red = ReadScalarAsByte(propertyBytes, property.type);
                        break;
                    case PropertySemantic::ColorG:
                        green = ReadScalarAsByte(propertyBytes, property.type);
                        break;
                    case PropertySemantic::ColorB:
                        blue = ReadScalarAsByte(propertyBytes, property.type);
                        break;
                    case PropertySemantic::ScalarField: {
                        const auto scalarValue =
                            static_cast<float>(ReadScalarAsDouble(propertyBytes, property.type));
                        cloud.scalarFieldValues[cloud.ScalarFieldValueIndex(property.scalarFieldIndex, globalIndex)] =
                            scalarValue;
                        cloud.scalarFields[property.scalarFieldIndex].Include(scalarValue);
                        break;
                    }
                    case PropertySemantic::Skip:
                        break;
                }
            }

            cloud.positions[globalIndex] = position;
            cloud.packedColors[globalIndex] = PackRgba8(red, green, blue);
            cloud.bounds.Expand(position);
            if ((globalIndex % focusSampleStride) == 0 && focusSamples.size() < kMaxFocusSamples) {
                focusSamples.push_back(position);
            }
        }
    }

    cloud.focusPoint = ComputeRepresentativeFocusPoint(cloud.bounds, focusSamples);
    cloud.hasFocusPoint = cloud.bounds.valid;

    return {.cloud = std::move(cloud), .success = true};
}

}  // namespace invisible_places::io
