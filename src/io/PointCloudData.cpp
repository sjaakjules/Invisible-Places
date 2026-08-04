#include "io/PointCloudData.hpp"

#include "io/PlyHeader.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstring>
#include <fstream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string_view>
#include <thread>

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
    NormalX,
    NormalY,
    NormalZ,
    ScalarField
};

struct PropertyLayout {
    PropertySemantic semantic = PropertySemantic::Skip;
    ScalarType type = ScalarType::Float32;
    std::uint32_t offset = 0;
    std::uint32_t size = 0;
    std::uint32_t scalarFieldIndex = 0;
    bool isRoughness = false;
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

std::string LowerAscii(std::string_view value) {
    std::string lowered;
    lowered.reserve(value.size());
    for (const char character : value) {
        lowered.push_back(static_cast<char>(
            std::tolower(static_cast<unsigned char>(character))));
    }
    return lowered;
}

// Mirrors the renderer's field-name normalization so containsPatterns match
// the same fields its by-name heuristics (roughness, ground id) resolve.
std::string NormalizedAlnumFieldName(std::string_view value) {
    std::string normalized;
    normalized.reserve(value.size());
    for (const char character : value) {
        const auto byte = static_cast<unsigned char>(character);
        if (std::isalnum(byte) == 0) {
            continue;
        }
        normalized.push_back(static_cast<char>(std::tolower(byte)));
    }
    return normalized;
}

}  // namespace

bool PointCloudScalarFieldFilterSelects(
    const PointCloudScalarFieldFilter& filter,
    std::string_view displayName,
    std::uint32_t sourceIndex) {
    if (filter.LoadsAll()) {
        return true;
    }
    if (std::find(
            filter.sourceIndices.begin(),
            filter.sourceIndices.end(),
            sourceIndex) != filter.sourceIndices.end()) {
        return true;
    }
    const auto loweredName = LowerAscii(displayName);
    if (std::any_of(
            filter.names.begin(),
            filter.names.end(),
            [&loweredName](const std::string& candidate) {
                return LowerAscii(candidate) == loweredName;
            })) {
        return true;
    }
    if (filter.containsPatterns.empty()) {
        return false;
    }
    const auto normalizedName = NormalizedAlnumFieldName(displayName);
    return std::any_of(
        filter.containsPatterns.begin(),
        filter.containsPatterns.end(),
        [&normalizedName](const std::string& pattern) {
            return !pattern.empty() &&
                   normalizedName.find(NormalizedAlnumFieldName(pattern)) !=
                       std::string::npos;
        });
}

namespace {

struct PointCloudLayout {
    std::vector<PropertyLayout> properties;
    std::uint32_t recordSize = 0;
    std::uint32_t scalarFieldCount = 0;
    bool hasNormals = false;
    bool hasRoughness = false;
};

constexpr std::size_t kMaxFocusSamples = 16384;

std::optional<PointCloudLayout> BuildPointCloudLayout(const PlyHeader& header, std::string* errorMessage) {
    PointCloudLayout layout;
    layout.properties.reserve(header.properties.size());

    std::uint32_t scalarFieldIndex = 0;
    bool sawX = false;
    bool sawY = false;
    bool sawZ = false;
    const bool hasLongNormalTriplet =
        header.HasProperty("normal_x") && header.HasProperty("normal_y") && header.HasProperty("normal_z");
    const bool hasShortNormalTriplet =
        !hasLongNormalTriplet && header.HasProperty("nx") && header.HasProperty("ny") && header.HasProperty("nz");
    layout.hasNormals = hasLongNormalTriplet || hasShortNormalTriplet;

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
        } else if (hasLongNormalTriplet && property.name == "normal_x") {
            layoutEntry.semantic = PropertySemantic::NormalX;
        } else if (hasLongNormalTriplet && property.name == "normal_y") {
            layoutEntry.semantic = PropertySemantic::NormalY;
        } else if (hasLongNormalTriplet && property.name == "normal_z") {
            layoutEntry.semantic = PropertySemantic::NormalZ;
        } else if (hasShortNormalTriplet && property.name == "nx") {
            layoutEntry.semantic = PropertySemantic::NormalX;
        } else if (hasShortNormalTriplet && property.name == "ny") {
            layoutEntry.semantic = PropertySemantic::NormalY;
        } else if (hasShortNormalTriplet && property.name == "nz") {
            layoutEntry.semantic = PropertySemantic::NormalZ;
        } else if (StartsWith(property.name, "scalar_")) {
            layoutEntry.semantic = PropertySemantic::ScalarField;
            layoutEntry.scalarFieldIndex = scalarFieldIndex++;
            layoutEntry.isRoughness =
                property.name == "scalar_Roughness" || property.name == "scalar_roughness";
            layout.hasRoughness = layout.hasRoughness || layoutEntry.isRoughness;
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

Float3 NormalizeNormal(Float3 normal) {
    if (!std::isfinite(normal.x) || !std::isfinite(normal.y) || !std::isfinite(normal.z)) {
        return {};
    }

    const float lengthSquared = (normal.x * normal.x) + (normal.y * normal.y) + (normal.z * normal.z);
    if (lengthSquared <= 1.0e-12F) {
        return {};
    }

    const float inverseLength = 1.0F / std::sqrt(lengthSquared);
    return {
        normal.x * inverseLength,
        normal.y * inverseLength,
        normal.z * inverseLength,
    };
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
    // Non-finite scalar samples remain in the field-major point data so
    // renderers can treat them as missing on a per-point basis. They must not
    // poison the finite metadata range used to initialise UI bounds.
    if (!std::isfinite(value)) {
        return;
    }
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

std::optional<std::size_t> LoadedPointCloud::ResidentSlotForSourceIndex(
    std::int32_t sourceIndex) const {
    if (sourceIndex < 0) {
        return std::nullopt;
    }
    for (std::size_t slot = 0; slot < scalarFields.size(); ++slot) {
        if (scalarFields[slot].sourceIndex == sourceIndex) {
            return slot;
        }
    }
    return std::nullopt;
}

PointCloudLoadResult LoadPointCloud(
    const std::filesystem::path& filePath,
    const PointCloudScalarFieldFilter& fieldFilter,
    unsigned threadCount) {
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
    cloud.hasNormals = layout->hasNormals;

    // File-order scalar slot -> resident matrix row, or -1 when the filter
    // leaves the field on disk. Selected fields keep their relative file
    // order so repeat loads with the same filter produce identical slots.
    std::vector<std::int32_t> residentSlotBySourceIndex(layout->scalarFieldCount, -1);
    try {
        cloud.positions.resize(static_cast<std::size_t>(header.vertexCount));
        if (cloud.hasNormals) {
            cloud.normals.resize(static_cast<std::size_t>(header.vertexCount));
        }
        cloud.packedColors.resize(static_cast<std::size_t>(header.vertexCount), PackRgba8(255, 255, 255));
        cloud.availableScalarFields.reserve(layout->scalarFieldCount);
        cloud.scalarFields.reserve(layout->scalarFieldCount);
        std::uint32_t sourceIndex = 0;
        for (const auto& property : header.properties) {
            if (!StartsWith(property.name, "scalar_")) {
                continue;
            }

            auto displayName = ScalarFieldDisplayName(property.name);
            cloud.availableScalarFields.push_back({
                .name = displayName,
                .sourceIndex = sourceIndex,
            });
            if (PointCloudScalarFieldFilterSelects(fieldFilter, displayName, sourceIndex)) {
                residentSlotBySourceIndex[sourceIndex] =
                    static_cast<std::int32_t>(cloud.scalarFields.size());
                ScalarFieldStats stats;
                stats.name = std::move(displayName);
                stats.sourceIndex = static_cast<std::int32_t>(sourceIndex);
                cloud.scalarFields.push_back(std::move(stats));
            }
            ++sourceIndex;
        }

        cloud.scalarFieldValues.resize(
            static_cast<std::size_t>(header.vertexCount) * cloud.scalarFields.size());
    } catch (const std::exception& error) {
        return {.errorMessage = std::string{"Point cloud allocation failed: "} + error.what(), .success = false};
    }

    input.close();

    // The record stride is fixed, so the payload splits into disjoint,
    // independently parseable vertex ranges: each worker opens its own
    // stream, seeks straight to its byte range, and writes its points into
    // the shared destination arrays at disjoint indices. Only the property
    // subset a record actually contributes survives into the flattened
    // program below — with a field filter active this skips most of the
    // record's ~40 properties per point.
    struct ActiveProperty {
        PropertySemantic semantic = PropertySemantic::Skip;
        ScalarType type = ScalarType::Float32;
        std::uint32_t offset = 0;
        std::int32_t residentSlot = -1;
    };
    std::vector<ActiveProperty> activeProperties;
    activeProperties.reserve(layout->properties.size());
    for (const auto& property : layout->properties) {
        if (property.semantic == PropertySemantic::Skip) {
            continue;
        }
        std::int32_t residentSlot = -1;
        if (property.semantic == PropertySemantic::ScalarField) {
            residentSlot = residentSlotBySourceIndex[property.scalarFieldIndex];
            if (residentSlot < 0) {
                continue;
            }
        }
        activeProperties.push_back({
            .semantic = property.semantic,
            .type = property.type,
            .offset = property.offset,
            .residentSlot = residentSlot,
        });
    }

    const auto focusSampleStride =
        std::max<std::uint64_t>(1ULL, header.vertexCount / static_cast<std::uint64_t>(kMaxFocusSamples));
    struct RangeParseOutput {
        Bounds3f bounds;
        std::vector<ScalarFieldStats> fieldStats;
        std::vector<Float3> focusSamples;
        std::string errorMessage;
    };
    const auto readFloat = [](const std::byte* bytes, ScalarType type) {
        // Float32 dominates real exports; reading it directly skips the
        // double round-trip (which is value-identical for floats anyway).
        return type == ScalarType::Float32
                   ? ReadScalar<float>(bytes)
                   : static_cast<float>(ReadScalarAsDouble(bytes, type));
    };
    const auto parseRange = [&](std::uint64_t rangeBegin,
                                std::uint64_t rangeEnd,
                                RangeParseOutput* out) {
        out->fieldStats.resize(cloud.scalarFields.size());
        std::ifstream rangeInput{filePath, std::ios::binary};
        if (!rangeInput.is_open()) {
            out->errorMessage = "Unable to open point cloud file.";
            return;
        }
        rangeInput.seekg(
            static_cast<std::streamoff>(
                header.dataOffsetBytes + rangeBegin * layout->recordSize),
            std::ios::beg);
        if (!rangeInput.good()) {
            out->errorMessage = "Failed to seek to PLY payload.";
            return;
        }
        const auto pointsPerChunk = RecommendedPointsPerChunk(layout->recordSize);
        std::vector<std::byte> chunkBuffer(pointsPerChunk * layout->recordSize);
        for (std::uint64_t pointStart = rangeBegin; pointStart < rangeEnd; pointStart += pointsPerChunk) {
            const auto remaining = rangeEnd - pointStart;
            const auto pointsThisChunk = static_cast<std::size_t>(std::min<std::uint64_t>(remaining, pointsPerChunk));
            const auto bytesToRead = pointsThisChunk * layout->recordSize;

            rangeInput.read(reinterpret_cast<char*>(chunkBuffer.data()), static_cast<std::streamsize>(bytesToRead));
            if (rangeInput.gcount() != static_cast<std::streamsize>(bytesToRead)) {
                out->errorMessage = "Unexpected EOF while reading point cloud payload.";
                return;
            }

            for (std::size_t localIndex = 0; localIndex < pointsThisChunk; ++localIndex) {
                const auto globalIndex = static_cast<std::size_t>(pointStart) + localIndex;
                const auto* recordBytes = chunkBuffer.data() + (localIndex * layout->recordSize);

                Float3 position{};
                Float3 normal{};
                std::uint8_t red = 255;
                std::uint8_t green = 255;
                std::uint8_t blue = 255;

                for (const auto& property : activeProperties) {
                    const auto* propertyBytes = recordBytes + property.offset;

                    switch (property.semantic) {
                        case PropertySemantic::PositionX:
                            position.x = readFloat(propertyBytes, property.type);
                            break;
                        case PropertySemantic::PositionY:
                            position.y = readFloat(propertyBytes, property.type);
                            break;
                        case PropertySemantic::PositionZ:
                            position.z = readFloat(propertyBytes, property.type);
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
                        case PropertySemantic::NormalX:
                            normal.x = readFloat(propertyBytes, property.type);
                            break;
                        case PropertySemantic::NormalY:
                            normal.y = readFloat(propertyBytes, property.type);
                            break;
                        case PropertySemantic::NormalZ:
                            normal.z = readFloat(propertyBytes, property.type);
                            break;
                        case PropertySemantic::ScalarField: {
                            const auto scalarValue = readFloat(propertyBytes, property.type);
                            cloud.scalarFieldValues[cloud.ScalarFieldValueIndex(
                                static_cast<std::size_t>(property.residentSlot), globalIndex)] = scalarValue;
                            out->fieldStats[static_cast<std::size_t>(property.residentSlot)]
                                .Include(scalarValue);
                            break;
                        }
                        case PropertySemantic::Skip:
                            break;
                    }
                }

                cloud.positions[globalIndex] = position;
                if (cloud.hasNormals) {
                    cloud.normals[globalIndex] = NormalizeNormal(normal);
                }
                cloud.packedColors[globalIndex] = PackRgba8(red, green, blue);
                out->bounds.Expand(position);
                if ((globalIndex % focusSampleStride) == 0) {
                    out->focusSamples.push_back(position);
                }
            }
        }
    };

    // One range per worker, sized so small clouds stay single-threaded and
    // the calling thread always parses the first range itself.
    constexpr std::uint64_t kMinimumPointsPerParseThread = 1'000'000ULL;
    const auto hardwareThreads = std::max(1U, std::thread::hardware_concurrency());
    auto effectiveThreads = threadCount != 0U
                                ? threadCount
                                : std::min(hardwareThreads, 8U);
    if (threadCount == 0U) {
        const auto sizedThreads = static_cast<unsigned>(std::min<std::uint64_t>(
            effectiveThreads,
            std::max<std::uint64_t>(1ULL, header.vertexCount / kMinimumPointsPerParseThread)));
        effectiveThreads = std::max(1U, sizedThreads);
    }
    effectiveThreads = static_cast<unsigned>(std::min<std::uint64_t>(
        effectiveThreads,
        std::max<std::uint64_t>(1ULL, header.vertexCount)));

    std::vector<RangeParseOutput> outputs(effectiveThreads);
    {
        std::vector<std::jthread> workers;
        workers.reserve(effectiveThreads > 0U ? effectiveThreads - 1U : 0U);
        const auto pointsPerRange =
            (header.vertexCount + effectiveThreads - 1U) / effectiveThreads;
        for (unsigned workerIndex = 1U; workerIndex < effectiveThreads; ++workerIndex) {
            const auto rangeBegin = std::min<std::uint64_t>(
                header.vertexCount,
                static_cast<std::uint64_t>(workerIndex) * pointsPerRange);
            const auto rangeEnd = std::min<std::uint64_t>(
                header.vertexCount,
                rangeBegin + pointsPerRange);
            workers.emplace_back([&parseRange, rangeBegin, rangeEnd, &outputs, workerIndex]() {
                parseRange(rangeBegin, rangeEnd, &outputs[workerIndex]);
            });
        }
        parseRange(
            0U,
            std::min<std::uint64_t>(header.vertexCount, pointsPerRange),
            &outputs[0]);
    }

    // Deterministic merge: min/max/count folds are order-independent, and
    // ranges are visited in index order so the concatenated focus samples
    // (truncated to the sequential cap) match a single-threaded parse
    // exactly.
    std::vector<Float3> focusSamples;
    focusSamples.reserve(std::min<std::uint64_t>(header.vertexCount, kMaxFocusSamples));
    for (const auto& output : outputs) {
        if (!output.errorMessage.empty()) {
            return {.errorMessage = output.errorMessage, .success = false};
        }
        if (output.bounds.valid) {
            cloud.bounds.Expand(output.bounds.minimum);
            cloud.bounds.Expand(output.bounds.maximum);
        }
        for (std::size_t slot = 0; slot < output.fieldStats.size(); ++slot) {
            const auto& local = output.fieldStats[slot];
            if (!local.valid) {
                continue;
            }
            auto& merged = cloud.scalarFields[slot];
            if (!merged.valid) {
                merged.minimum = local.minimum;
                merged.maximum = local.maximum;
                merged.count = local.count;
                merged.valid = true;
            } else {
                merged.minimum = std::min(merged.minimum, local.minimum);
                merged.maximum = std::max(merged.maximum, local.maximum);
                merged.count += local.count;
            }
        }
        for (const auto& sample : output.focusSamples) {
            if (focusSamples.size() >= kMaxFocusSamples) {
                break;
            }
            focusSamples.push_back(sample);
        }
    }

    cloud.focusPoint = ComputeRepresentativeFocusPoint(cloud.bounds, focusSamples);
    cloud.hasFocusPoint = cloud.bounds.valid;

    return {.cloud = std::move(cloud), .success = true};
}

PointCloudStreamResult StreamPointCloudPositionsNormals(
    const std::filesystem::path& filePath,
    const PointCloudPositionNormalVisitor& visitor) {
    const auto headerResult = ParsePlyHeader(filePath);
    if (!headerResult.success) {
        return {.errorMessage = headerResult.errorMessage};
    }

    const auto& header = headerResult.header;
    if (header.format != "binary_little_endian") {
        return {.errorMessage = "Only binary_little_endian PLY point clouds are supported."};
    }

    std::string layoutError;
    const auto layout = BuildPointCloudLayout(header, &layoutError);
    if (!layout.has_value()) {
        return {.errorMessage = layoutError};
    }

    std::ifstream input{filePath, std::ios::binary};
    if (!input.is_open()) {
        return {.errorMessage = "Unable to open point cloud file."};
    }

    input.seekg(static_cast<std::streamoff>(header.dataOffsetBytes), std::ios::beg);
    if (!input.good()) {
        return {.errorMessage = "Failed to seek to PLY payload."};
    }

    PointCloudStreamResult result;
    result.hasNormals = layout->hasNormals;
    result.hasRoughness = layout->hasRoughness;
    const auto pointsPerChunk = RecommendedPointsPerChunk(layout->recordSize);
    std::vector<std::byte> chunkBuffer(pointsPerChunk * layout->recordSize);

    for (std::uint64_t pointStart = 0; pointStart < header.vertexCount; pointStart += pointsPerChunk) {
        const auto remaining = header.vertexCount - pointStart;
        const auto pointsThisChunk = static_cast<std::size_t>(std::min<std::uint64_t>(remaining, pointsPerChunk));
        const auto bytesToRead = pointsThisChunk * layout->recordSize;

        input.read(reinterpret_cast<char*>(chunkBuffer.data()), static_cast<std::streamsize>(bytesToRead));
        if (input.gcount() != static_cast<std::streamsize>(bytesToRead)) {
            result.errorMessage = "Unexpected EOF while reading point cloud payload.";
            return result;
        }

        for (std::size_t localIndex = 0; localIndex < pointsThisChunk; ++localIndex) {
            const auto globalIndex = pointStart + static_cast<std::uint64_t>(localIndex);
            const auto* recordBytes = chunkBuffer.data() + (localIndex * layout->recordSize);
            PointCloudPositionNormalSample sample;
            sample.hasNormal = layout->hasNormals;

            for (const auto& property : layout->properties) {
                const auto* propertyBytes = recordBytes + property.offset;
                switch (property.semantic) {
                    case PropertySemantic::PositionX:
                        sample.position.x = static_cast<float>(ReadScalarAsDouble(propertyBytes, property.type));
                        break;
                    case PropertySemantic::PositionY:
                        sample.position.y = static_cast<float>(ReadScalarAsDouble(propertyBytes, property.type));
                        break;
                    case PropertySemantic::PositionZ:
                        sample.position.z = static_cast<float>(ReadScalarAsDouble(propertyBytes, property.type));
                        break;
                    case PropertySemantic::NormalX:
                        sample.normal.x = static_cast<float>(ReadScalarAsDouble(propertyBytes, property.type));
                        break;
                    case PropertySemantic::NormalY:
                        sample.normal.y = static_cast<float>(ReadScalarAsDouble(propertyBytes, property.type));
                        break;
                    case PropertySemantic::NormalZ:
                        sample.normal.z = static_cast<float>(ReadScalarAsDouble(propertyBytes, property.type));
                        break;
                    case PropertySemantic::ScalarField:
                        if (property.isRoughness) {
                            sample.roughness = static_cast<float>(
                                ReadScalarAsDouble(propertyBytes, property.type));
                            sample.hasRoughness = std::isfinite(sample.roughness);
                        }
                        break;
                    default:
                        break;
                }
            }

            if (sample.hasNormal) {
                sample.normal = NormalizeNormal(sample.normal);
            }
            result.bounds.Expand(sample.position);
            ++result.pointCount;
            if (visitor && !visitor(sample, globalIndex)) {
                result.cancelled = true;
                return result;
            }
        }
    }

    result.success = true;
    return result;
}

PointCloudSelectedValueStreamResult StreamPointCloudSelectedValues(
    const std::filesystem::path& filePath,
    const PointCloudSelectedValueSelector& selector,
    const PointCloudSelectedValueVisitor& visitor) {
    if (selector.source == PointCloudSelectedValueSource::ScalarField &&
        selector.scalarFieldName.empty()) {
        return {
            .errorMessage =
                "A scalar field name is required when streaming a selected "
                "point-cloud value.",
        };
    }

    const auto headerResult = ParsePlyHeader(filePath);
    if (!headerResult.success) {
        return {.errorMessage = headerResult.errorMessage};
    }

    const auto& header = headerResult.header;
    if (header.format != "binary_little_endian") {
        return {
            .errorMessage =
                "Only binary_little_endian PLY point clouds are supported.",
        };
    }

    std::string layoutError;
    const auto layout = BuildPointCloudLayout(header, &layoutError);
    if (!layout.has_value()) {
        return {.errorMessage = layoutError};
    }

    const PropertyLayout* selectedScalar = nullptr;
    std::array<const PropertyLayout*, 3> normalProperties{};
    for (std::size_t index = 0U;
         index < layout->properties.size() &&
         index < header.properties.size();
         ++index) {
        const auto& property = layout->properties[index];
        switch (property.semantic) {
            case PropertySemantic::NormalX:
                normalProperties[0U] = &property;
                break;
            case PropertySemantic::NormalY:
                normalProperties[1U] = &property;
                break;
            case PropertySemantic::NormalZ:
                normalProperties[2U] = &property;
                break;
            case PropertySemantic::ScalarField:
                if (selector.source ==
                        PointCloudSelectedValueSource::ScalarField &&
                    ScalarFieldDisplayName(
                        header.properties[index].name) ==
                        selector.scalarFieldName) {
                    selectedScalar = &property;
                }
                break;
            default:
                break;
        }
    }

    if (selector.source == PointCloudSelectedValueSource::ScalarField &&
        selectedScalar == nullptr) {
        return {
            .errorMessage =
                "Point cloud does not contain scalar field '" +
                selector.scalarFieldName + "'.",
        };
    }
    if (selector.source != PointCloudSelectedValueSource::ScalarField &&
        (!layout->hasNormals ||
         std::any_of(
             normalProperties.begin(),
             normalProperties.end(),
             [](const PropertyLayout* property) {
                 return property == nullptr;
             }))) {
        return {
            .errorMessage =
                "Point cloud does not contain a complete normal triplet.",
        };
    }

    std::ifstream input{filePath, std::ios::binary};
    if (!input.is_open()) {
        return {.errorMessage = "Unable to open point cloud file."};
    }
    input.seekg(
        static_cast<std::streamoff>(header.dataOffsetBytes),
        std::ios::beg);
    if (!input.good()) {
        return {.errorMessage = "Failed to seek to PLY payload."};
    }

    PointCloudSelectedValueStreamResult result;
    const auto pointsPerChunk =
        RecommendedPointsPerChunk(layout->recordSize);
    std::vector<std::byte> chunkBuffer(
        pointsPerChunk * layout->recordSize);
    for (std::uint64_t pointStart = 0U;
         pointStart < header.vertexCount;
         pointStart += pointsPerChunk) {
        const auto remaining = header.vertexCount - pointStart;
        const auto pointsThisChunk = static_cast<std::size_t>(
            std::min<std::uint64_t>(remaining, pointsPerChunk));
        const auto bytesToRead =
            pointsThisChunk * layout->recordSize;
        input.read(
            reinterpret_cast<char*>(chunkBuffer.data()),
            static_cast<std::streamsize>(bytesToRead));
        if (input.gcount() !=
            static_cast<std::streamsize>(bytesToRead)) {
            result.errorMessage =
                "Unexpected EOF while reading point cloud payload.";
            return result;
        }

        for (std::size_t localIndex = 0U;
             localIndex < pointsThisChunk;
             ++localIndex) {
            const auto globalIndex =
                pointStart +
                static_cast<std::uint64_t>(localIndex);
            const auto* recordBytes =
                chunkBuffer.data() +
                (localIndex * layout->recordSize);
            float value = 0.0F;
            if (selectedScalar != nullptr) {
                value = static_cast<float>(ReadScalarAsDouble(
                    recordBytes + selectedScalar->offset,
                    selectedScalar->type));
            } else {
                Float3 normal{
                    .x = static_cast<float>(ReadScalarAsDouble(
                        recordBytes + normalProperties[0U]->offset,
                        normalProperties[0U]->type)),
                    .y = static_cast<float>(ReadScalarAsDouble(
                        recordBytes + normalProperties[1U]->offset,
                        normalProperties[1U]->type)),
                    .z = static_cast<float>(ReadScalarAsDouble(
                        recordBytes + normalProperties[2U]->offset,
                        normalProperties[2U]->type)),
                };
                normal = NormalizeNormal(normal);
                switch (selector.source) {
                    case PointCloudSelectedValueSource::NormalX:
                        value = normal.x;
                        break;
                    case PointCloudSelectedValueSource::NormalY:
                        value = normal.y;
                        break;
                    case PointCloudSelectedValueSource::NormalZ:
                        value = normal.z;
                        break;
                    case PointCloudSelectedValueSource::ScalarField:
                        break;
                }
            }

            ++result.pointCount;
            if (visitor && !visitor(value, globalIndex)) {
                result.cancelled = true;
                return result;
            }
        }
    }

    result.success = true;
    return result;
}

}  // namespace invisible_places::io
