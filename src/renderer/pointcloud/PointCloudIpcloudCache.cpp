#include "renderer/pointcloud/PointCloudIpcloudCache.hpp"

#include "io/PlyHeader.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <limits>
#include <nlohmann/json.hpp>
#include <optional>
#include <queue>
#include <sstream>
#include <system_error>
#include <unordered_set>

namespace invisible_places::renderer::pointcloud {

namespace {

using json = nlohmann::json;

constexpr std::uint64_t kFnvOffset = 1469598103934665603ULL;
constexpr std::uint64_t kFnvPrime = 1099511628211ULL;
constexpr std::uint64_t kRepresentativeMagic = 0x3156455244435053ULL;  // SPCDREV1
constexpr std::uint64_t kHierarchyMagic = 0x3148435049435049ULL;       // IPCIPCH1
constexpr std::uint64_t kAttributeSchemaMagic = 0x3148504349504349ULL; // ICPICPH1
constexpr std::uint64_t kRawChunkMagic = 0x314b4e5548435049ULL;        // IPCHUNK1
constexpr std::uint64_t kScalarStatsMagic = 0x3153544154435049ULL;     // IPCATS1
constexpr std::uint64_t kNodePagesMagic = 0x3153475044435049ULL;       // IPCDPGS1

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
};

struct PointCloudRecordLayout {
    std::vector<PropertyLayout> properties;
    std::vector<std::string> scalarFieldNames;
    std::uint32_t recordSize = 0;
    bool hasRgb = false;
    bool hasNormals = false;
};

struct RepresentativeFileHeader {
    std::uint64_t magic = kRepresentativeMagic;
    std::uint32_t version = 1U;
    std::uint32_t headerSize = sizeof(RepresentativeFileHeader);
    std::uint64_t pointCount = 0;
    std::uint64_t representedSourceCount = 0;
    std::uint32_t scalarFieldCount = 0;
    std::uint32_t hasRgb = 0;
    std::uint32_t hasNormals = 0;
    std::uint32_t reserved0 = 0;
};

struct HierarchyFileHeader {
    std::uint64_t magic = kHierarchyMagic;
    std::uint32_t version = 1U;
    std::uint32_t headerSize = sizeof(HierarchyFileHeader);
    std::uint64_t nodeCount = 0;
    std::uint64_t representativeCount = 0;
    std::uint64_t scalarStatsCount = 0;
    std::uint64_t nodeScalarStatsCount = 0;
    std::uint32_t sourcePointCount = 0;
    std::uint32_t scalarFieldCount = 0;
};

struct RawChunkRangeHeader {
    std::uint64_t magic = kRawChunkMagic;
    std::uint32_t version = 1U;
    std::uint32_t headerSize = sizeof(RawChunkRangeHeader);
    std::uint64_t firstPoint = 0;
    std::uint64_t pointCount = 0;
    std::uint64_t sourceByteOffset = 0;
    std::uint64_t sourceByteCount = 0;
};

struct ScalarStatsFileHeader {
    std::uint64_t magic = kScalarStatsMagic;
    std::uint32_t version = 1U;
    std::uint32_t headerSize = sizeof(ScalarStatsFileHeader);
    std::uint64_t scalarFieldCount = 0;
};

struct NodePagesFileHeader {
    std::uint64_t magic = kNodePagesMagic;
    std::uint32_t version = 1U;
    std::uint32_t headerSize = sizeof(NodePagesFileHeader);
    std::uint64_t pageCount = 0;
};

static_assert(std::is_trivially_copyable_v<RepresentativeFileHeader>);
static_assert(std::is_trivially_copyable_v<HierarchyFileHeader>);
static_assert(std::is_trivially_copyable_v<RawChunkRangeHeader>);
static_assert(std::is_trivially_copyable_v<ScalarStatsFileHeader>);
static_assert(std::is_trivially_copyable_v<NodePagesFileHeader>);

std::uint64_t HashBytes(std::uint64_t hash, const void* data, std::size_t size) {
    const auto* bytes = static_cast<const unsigned char*>(data);
    for (std::size_t index = 0; index < size; ++index) {
        hash ^= static_cast<std::uint64_t>(bytes[index]);
        hash *= kFnvPrime;
    }
    return hash;
}

std::uint64_t HashString(std::uint64_t hash, const std::string& text) {
    return HashBytes(hash, text.data(), text.size());
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
    return static_cast<std::uint8_t>(std::clamp(ReadScalarAsDouble(bytes, type), 0.0, 255.0));
}

std::uint32_t PackRgba8(std::uint8_t red, std::uint8_t green, std::uint8_t blue) {
    return static_cast<std::uint32_t>(red) |
           (static_cast<std::uint32_t>(green) << 8U) |
           (static_cast<std::uint32_t>(blue) << 16U) |
           (0xFFU << 24U);
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
            return 1U;
        case ScalarType::Int16:
        case ScalarType::UInt16:
            return 2U;
        case ScalarType::Int32:
        case ScalarType::UInt32:
        case ScalarType::Float32:
            return 4U;
        case ScalarType::Float64:
            return 8U;
    }
    return 0U;
}

bool StartsWith(std::string_view value, std::string_view prefix) {
    return value.substr(0, prefix.size()) == prefix;
}

std::string ScalarFieldDisplayName(std::string_view propertyName) {
    constexpr std::string_view prefix = "scalar_";
    return StartsWith(propertyName, prefix) ? std::string{propertyName.substr(prefix.size())}
                                           : std::string{propertyName};
}

invisible_places::io::Float3 NormalizeNormal(invisible_places::io::Float3 normal) {
    if (!std::isfinite(normal.x) || !std::isfinite(normal.y) || !std::isfinite(normal.z)) {
        return {};
    }
    const float lengthSquared = (normal.x * normal.x) + (normal.y * normal.y) + (normal.z * normal.z);
    if (lengthSquared <= 1.0e-12F) {
        return {};
    }
    const float inverseLength = 1.0F / std::sqrt(lengthSquared);
    return {normal.x * inverseLength, normal.y * inverseLength, normal.z * inverseLength};
}

std::optional<PointCloudRecordLayout> BuildRecordLayout(
    const invisible_places::io::PlyHeader& header,
    std::string* errorMessage) {
    PointCloudRecordLayout layout;
    layout.properties.reserve(header.properties.size());
    layout.hasRgb = header.HasColorRgb();

    bool sawX = false;
    bool sawY = false;
    bool sawZ = false;
    const bool hasLongNormalTriplet =
        header.HasProperty("normal_x") && header.HasProperty("normal_y") && header.HasProperty("normal_z");
    const bool hasShortNormalTriplet =
        !hasLongNormalTriplet && header.HasProperty("nx") && header.HasProperty("ny") && header.HasProperty("nz");
    layout.hasNormals = hasLongNormalTriplet || hasShortNormalTriplet;

    std::uint32_t scalarFieldIndex = 0;
    for (const auto& property : header.properties) {
        const auto scalarType = ParseScalarType(property.type);
        if (!scalarType.has_value()) {
            if (errorMessage != nullptr) {
                *errorMessage = "Unsupported PLY property type: " + property.type;
            }
            return std::nullopt;
        }

        PropertyLayout entry;
        entry.type = scalarType.value();
        entry.offset = layout.recordSize;
        entry.size = ScalarTypeSize(entry.type);
        if (property.name == "x") {
            entry.semantic = PropertySemantic::PositionX;
            sawX = true;
        } else if (property.name == "y") {
            entry.semantic = PropertySemantic::PositionY;
            sawY = true;
        } else if (property.name == "z") {
            entry.semantic = PropertySemantic::PositionZ;
            sawZ = true;
        } else if (property.name == "red") {
            entry.semantic = PropertySemantic::ColorR;
        } else if (property.name == "green") {
            entry.semantic = PropertySemantic::ColorG;
        } else if (property.name == "blue") {
            entry.semantic = PropertySemantic::ColorB;
        } else if (hasLongNormalTriplet && property.name == "normal_x") {
            entry.semantic = PropertySemantic::NormalX;
        } else if (hasLongNormalTriplet && property.name == "normal_y") {
            entry.semantic = PropertySemantic::NormalY;
        } else if (hasLongNormalTriplet && property.name == "normal_z") {
            entry.semantic = PropertySemantic::NormalZ;
        } else if (hasShortNormalTriplet && property.name == "nx") {
            entry.semantic = PropertySemantic::NormalX;
        } else if (hasShortNormalTriplet && property.name == "ny") {
            entry.semantic = PropertySemantic::NormalY;
        } else if (hasShortNormalTriplet && property.name == "nz") {
            entry.semantic = PropertySemantic::NormalZ;
        } else if (StartsWith(property.name, "scalar_")) {
            entry.semantic = PropertySemantic::ScalarField;
            entry.scalarFieldIndex = scalarFieldIndex++;
            layout.scalarFieldNames.push_back(ScalarFieldDisplayName(property.name));
        }
        layout.recordSize += entry.size;
        layout.properties.push_back(entry);
    }

    if (!(sawX && sawY && sawZ)) {
        if (errorMessage != nullptr) {
            *errorMessage = "PLY point cloud is missing x/y/z properties.";
        }
        return std::nullopt;
    }
    return layout;
}

std::uint64_t HeaderHash(const invisible_places::io::PlyHeader& header) {
    std::uint64_t hash = kFnvOffset;
    hash = HashString(hash, header.format);
    hash = HashBytes(hash, &header.vertexCount, sizeof(header.vertexCount));
    for (const auto& comment : header.comments) {
        hash = HashString(hash, comment);
    }
    for (const auto& property : header.properties) {
        hash = HashString(hash, property.type);
        hash = HashString(hash, property.name);
    }
    return hash;
}

std::uint64_t SampledContentHash(
    const std::filesystem::path& sourcePath,
    std::uint64_t dataOffsetBytes,
    std::uint64_t fileSizeBytes) {
    std::ifstream input{sourcePath, std::ios::binary};
    if (!input || fileSizeBytes <= dataOffsetBytes) {
        return 0ULL;
    }

    constexpr std::uint64_t kSampleBytes = 64ULL * 1024ULL;
    const std::uint64_t payloadBytes = fileSizeBytes - dataOffsetBytes;
    std::array<std::uint64_t, 3> offsets = {
        dataOffsetBytes,
        dataOffsetBytes + (payloadBytes / 2ULL),
        fileSizeBytes > kSampleBytes ? fileSizeBytes - std::min(kSampleBytes, payloadBytes) : dataOffsetBytes,
    };

    std::uint64_t hash = kFnvOffset;
    std::vector<char> buffer(static_cast<std::size_t>(kSampleBytes));
    for (const auto offset : offsets) {
        input.clear();
        input.seekg(static_cast<std::streamoff>(offset), std::ios::beg);
        if (!input) {
            continue;
        }
        const auto remaining = fileSizeBytes > offset ? fileSizeBytes - offset : 0ULL;
        const auto bytesToRead = static_cast<std::size_t>(std::min<std::uint64_t>(kSampleBytes, remaining));
        input.read(buffer.data(), static_cast<std::streamsize>(bytesToRead));
        const auto bytesRead = static_cast<std::size_t>(std::max<std::streamsize>(0, input.gcount()));
        hash = HashBytes(hash, &offset, sizeof(offset));
        hash = HashBytes(hash, buffer.data(), bytesRead);
    }
    return hash;
}

std::string Hex64(std::uint64_t value) {
    std::ostringstream output;
    output << std::hex << std::setfill('0') << std::setw(16) << value;
    return output.str();
}

std::uint64_t FingerprintHash(const PointCloudIpcloudSourceInfo& sourceInfo) {
    std::uint64_t hash = kFnvOffset;
    hash = HashBytes(hash, &sourceInfo.normalizedPathHash, sizeof(sourceInfo.normalizedPathHash));
    hash = HashBytes(hash, &sourceInfo.sourceSizeBytes, sizeof(sourceInfo.sourceSizeBytes));
    hash = HashBytes(hash, &sourceInfo.sourceWriteTimeNs, sizeof(sourceInfo.sourceWriteTimeNs));
    hash = HashBytes(hash, &sourceInfo.headerHash, sizeof(sourceInfo.headerHash));
    hash = HashBytes(hash, &sourceInfo.sampledContentHash, sizeof(sourceInfo.sampledContentHash));
    hash = HashBytes(hash, &sourceInfo.pointCount, sizeof(sourceInfo.pointCount));
    hash = HashBytes(hash, &sourceInfo.scalarFieldCount, sizeof(sourceInfo.scalarFieldCount));
    return hash;
}

std::filesystem::path TemporaryBundlePath(const std::filesystem::path& bundlePath) {
    auto temporaryPath = bundlePath;
    temporaryPath += ".tmp";
    return temporaryPath;
}

std::filesystem::path ManifestPath(const std::filesystem::path& bundlePath) {
    return bundlePath / "manifest.json";
}

std::filesystem::path BuildStatusPath(const std::filesystem::path& bundlePath) {
    return bundlePath / "build_status.json";
}

std::optional<json> ReadJsonFile(const std::filesystem::path& path, std::string* errorMessage = nullptr) {
    std::ifstream input{path};
    if (!input) {
        if (errorMessage != nullptr) {
            *errorMessage = "missing " + path.filename().string();
        }
        return std::nullopt;
    }
    try {
        json value;
        input >> value;
        return value;
    } catch (const std::exception& error) {
        if (errorMessage != nullptr) {
            *errorMessage = error.what();
        }
    }
    return std::nullopt;
}

bool WriteJsonFile(const std::filesystem::path& path, const json& value, std::string* errorMessage = nullptr) {
    std::ofstream output{path, std::ios::trunc};
    if (!output) {
        if (errorMessage != nullptr) {
            *errorMessage = "could not write " + path.filename().string();
        }
        return false;
    }
    output << std::setw(2) << value << "\n";
    return static_cast<bool>(output);
}

PointCloudIpcloudBuildStatus ParseBuildStatus(const json& value) {
    PointCloudIpcloudBuildStatus status;
    status.parseHeaderComplete = value.value("parse_header_complete", false);
    status.upperHierarchyComplete = value.value("upper_hierarchy_complete", false);
    status.nodePagesComplete = value.value("node_pages_complete", false);
    status.representativeRangesComplete = value.value("representative_ranges_complete", false);
    status.rawChunkRangesComplete = value.value("raw_chunk_ranges_complete", false);
    status.scalarStatsComplete = value.value("scalar_stats_complete", false);
    status.complete = value.value("complete", false);
    status.failed = value.value("failed", false);
    status.interrupted = value.value("interrupted", false);
    status.rawChunksCompleted = value.value("raw_chunks_completed", 0ULL);
    status.rawChunkCount = value.value("raw_chunk_count", 0ULL);
    status.phase = value.value("phase", std::string{});
    status.message = value.value("message", std::string{});
    return status;
}

json BuildStatusJson(const PointCloudIpcloudBuildStatus& status) {
    return {
        {"parse_header_complete", status.parseHeaderComplete},
        {"upper_hierarchy_complete", status.upperHierarchyComplete},
        {"node_pages_complete", status.nodePagesComplete},
        {"representative_ranges_complete", status.representativeRangesComplete},
        {"raw_chunk_ranges_complete", status.rawChunkRangesComplete},
        {"scalar_stats_complete", status.scalarStatsComplete},
        {"complete", status.complete},
        {"failed", status.failed},
        {"interrupted", status.interrupted},
        {"raw_chunks_completed", status.rawChunksCompleted},
        {"raw_chunk_count", status.rawChunkCount},
        {"phase", status.phase},
        {"message", status.message},
    };
}

bool RequiredBundleFilesExist(const std::filesystem::path& bundlePath, std::string* missingFile) {
    constexpr std::array<std::string_view, 8> files = {
        "manifest.json",
        "attribute_schema.bin",
        "hierarchy.bin",
        "node_pages.bin",
        "node_stats.bin",
        "lod_representatives.bin",
        "scalar_stats.bin",
        "build_status.json",
    };
    for (const auto file : files) {
        const auto path = bundlePath / std::string{file};
        std::error_code error;
        if (!std::filesystem::is_regular_file(path, error)) {
            if (missingFile != nullptr) {
                *missingFile = std::string{file};
            }
            return false;
        }
    }
    std::error_code error;
    if (!std::filesystem::is_directory(bundlePath / "raw_chunks", error)) {
        if (missingFile != nullptr) {
            *missingFile = "raw_chunks";
        }
        return false;
    }
    return true;
}

bool ManifestMatchesSourceAndBuild(
    const json& manifest,
    const PointCloudIpcloudSourceInfo& sourceInfo,
    const PointCloudLodBuildConfig& buildConfig,
    std::string* reason) {
    auto reject = [reason](std::string text) {
        if (reason != nullptr) {
            *reason = std::move(text);
        }
        return false;
    };
    if (manifest.value("cache_format", std::string{}) != "InvisiblePlacesPointCloudIpcloud" ||
        manifest.value("cache_format_version", 0U) != kPointCloudIpcloudCacheFormatVersion) {
        return reject("format version mismatch");
    }
    const auto source = manifest.value("source", json::object());
    if (source.value("path_hash", 0ULL) != sourceInfo.normalizedPathHash ||
        source.value("size_bytes", 0ULL) != sourceInfo.sourceSizeBytes ||
        source.value("modified_time_ns", std::int64_t{0}) != sourceInfo.sourceWriteTimeNs ||
        source.value("header_hash", 0ULL) != sourceInfo.headerHash ||
        source.value("sampled_content_hash", 0ULL) != sourceInfo.sampledContentHash ||
        source.value("point_count", 0ULL) != sourceInfo.pointCount ||
        source.value("scalar_field_count", 0U) != sourceInfo.scalarFieldCount ||
        source.value("has_rgb", false) != sourceInfo.hasRgb ||
        source.value("has_normals", false) != sourceInfo.hasNormals) {
        return reject("source fingerprint mismatch");
    }
    const auto build = manifest.value("build", json::object());
    if (build.value("attribute_schema_version", 0U) != kPointCloudIpcloudAttributeSchemaVersion ||
        build.value("build_settings_version", 0U) != kPointCloudIpcloudBuildSettingsVersion ||
        build.value("max_leaf_source_points", 0U) != buildConfig.maxLeafSourcePoints ||
        build.value("max_depth", 0U) != buildConfig.maxDepth ||
        build.value("max_internal_representatives", 0U) != buildConfig.maxInternalRepresentatives) {
        return reject("build settings mismatch");
    }
    return true;
}

template <typename T>
bool WriteVector(std::ofstream& output, const std::vector<T>& values) {
    static_assert(std::is_trivially_copyable_v<T>);
    if (values.empty()) {
        return true;
    }
    output.write(
        reinterpret_cast<const char*>(values.data()),
        static_cast<std::streamsize>(values.size() * sizeof(T)));
    return static_cast<bool>(output);
}

template <typename T>
bool ReadVector(std::ifstream& input, std::vector<T>* values, std::uint64_t count) {
    static_assert(std::is_trivially_copyable_v<T>);
    if (values == nullptr ||
        count > static_cast<std::uint64_t>(std::numeric_limits<std::size_t>::max())) {
        return false;
    }
    values->resize(static_cast<std::size_t>(count));
    if (values->empty()) {
        return true;
    }
    input.read(
        reinterpret_cast<char*>(values->data()),
        static_cast<std::streamsize>(values->size() * sizeof(T)));
    return static_cast<bool>(input);
}

std::vector<std::uint32_t> SelectPreviewSourceIndices(
    const invisible_places::io::LoadedPointCloud& cloud,
    const PointCloudLodHierarchy& hierarchy,
    std::uint32_t targetCount) {
    std::vector<std::uint32_t> indices;
    indices.reserve(std::min<std::size_t>(targetCount, cloud.positions.size()));
    std::unordered_set<std::uint32_t> seen;
    seen.reserve(indices.capacity());

    if (!hierarchy.Empty()) {
        std::queue<std::uint32_t> queue;
        queue.push(0U);
        while (!queue.empty() && indices.size() < targetCount) {
            const auto nodeIndex = queue.front();
            queue.pop();
            if (nodeIndex >= hierarchy.nodes.size()) {
                continue;
            }
            const auto& node = hierarchy.nodes[nodeIndex];
            for (std::uint32_t offset = 0; offset < node.representativeCount && indices.size() < targetCount; ++offset) {
                const auto representativeIndex = node.firstRepresentative + offset;
                if (representativeIndex >= hierarchy.representatives.size()) {
                    continue;
                }
                const auto sourceIndex = hierarchy.representatives[representativeIndex].sourcePointIndex;
                if (sourceIndex < cloud.positions.size() && seen.insert(sourceIndex).second) {
                    indices.push_back(sourceIndex);
                }
            }
            for (std::uint32_t childOffset = 0; childOffset < node.childCount; ++childOffset) {
                queue.push(node.childIndices[childOffset]);
            }
        }
    }

    if (indices.size() < targetCount && !cloud.positions.empty()) {
        const auto stride = std::max<std::uint64_t>(
            1ULL,
            static_cast<std::uint64_t>(cloud.positions.size()) / std::max<std::uint32_t>(1U, targetCount));
        for (std::uint64_t pointIndex = 0; pointIndex < cloud.positions.size() && indices.size() < targetCount;
             pointIndex += stride) {
            const auto sourceIndex = static_cast<std::uint32_t>(std::min<std::uint64_t>(
                pointIndex,
                static_cast<std::uint64_t>(std::numeric_limits<std::uint32_t>::max())));
            if (seen.insert(sourceIndex).second) {
                indices.push_back(sourceIndex);
            }
        }
    }
    return indices;
}

invisible_places::io::LoadedPointCloud MakePreviewCloudFromSourceIndices(
    const invisible_places::io::LoadedPointCloud& source,
    const std::vector<std::uint32_t>& sourceIndices) {
    invisible_places::io::LoadedPointCloud preview;
    preview.sourcePath = source.sourcePath;
    preview.layerName = source.layerName;
    preview.hasSourceRgb = source.hasSourceRgb;
    preview.hasNormals = source.hasNormals;
    preview.hasFocusPoint = source.hasFocusPoint;
    preview.focusPoint = source.focusPoint;
    preview.scalarFields = source.scalarFields;
    preview.positions.reserve(sourceIndices.size());
    if (preview.hasNormals) {
        preview.normals.reserve(sourceIndices.size());
    }
    preview.packedColors.reserve(sourceIndices.size());
    preview.scalarFieldValues.resize(source.scalarFields.size() * sourceIndices.size());

    for (std::size_t previewIndex = 0; previewIndex < sourceIndices.size(); ++previewIndex) {
        const auto sourceIndex = sourceIndices[previewIndex];
        if (sourceIndex >= source.positions.size()) {
            continue;
        }
        preview.positions.push_back(source.positions[sourceIndex]);
        preview.bounds.Expand(source.positions[sourceIndex]);
        if (preview.hasNormals && sourceIndex < source.normals.size()) {
            preview.normals.push_back(source.normals[sourceIndex]);
        }
        preview.packedColors.push_back(
            sourceIndex < source.packedColors.size() ? source.packedColors[sourceIndex] : PackRgba8(255, 255, 255));
        for (std::size_t fieldIndex = 0; fieldIndex < source.scalarFields.size(); ++fieldIndex) {
            const auto sourceValueIndex = source.ScalarFieldValueIndex(fieldIndex, sourceIndex);
            const auto previewValueIndex = preview.ScalarFieldValueIndex(fieldIndex, previewIndex);
            if (sourceValueIndex < source.scalarFieldValues.size() &&
                previewValueIndex < preview.scalarFieldValues.size()) {
                const float value = source.scalarFieldValues[sourceValueIndex];
                preview.scalarFieldValues[previewValueIndex] = value;
            }
        }
    }
    if (!preview.bounds.valid) {
        preview.bounds = source.bounds;
    }
    if (!preview.hasFocusPoint && preview.bounds.valid) {
        preview.focusPoint = {
            0.5F * (preview.bounds.minimum.x + preview.bounds.maximum.x),
            0.5F * (preview.bounds.minimum.y + preview.bounds.maximum.y),
            0.5F * (preview.bounds.minimum.z + preview.bounds.maximum.z),
        };
        preview.hasFocusPoint = true;
    }
    return preview;
}

bool WriteString(std::ofstream& output, const std::string& text) {
    const auto size = static_cast<std::uint32_t>(
        std::min<std::size_t>(text.size(), std::numeric_limits<std::uint32_t>::max()));
    output.write(reinterpret_cast<const char*>(&size), sizeof(size));
    if (size > 0U) {
        output.write(text.data(), static_cast<std::streamsize>(size));
    }
    return static_cast<bool>(output);
}

std::optional<std::string> ReadString(std::ifstream& input) {
    std::uint32_t size = 0;
    input.read(reinterpret_cast<char*>(&size), sizeof(size));
    if (!input) {
        return std::nullopt;
    }
    std::string text(size, '\0');
    if (size > 0U) {
        input.read(text.data(), static_cast<std::streamsize>(size));
        if (!input) {
            return std::nullopt;
        }
    }
    return text;
}

bool WritePreviewRepresentatives(
    const std::filesystem::path& path,
    const invisible_places::io::LoadedPointCloud& preview,
    const std::vector<std::uint32_t>& sourceIndices,
    std::uint64_t representedSourceCount,
    std::string* errorMessage) {
    std::ofstream output{path, std::ios::binary | std::ios::trunc};
    if (!output) {
        if (errorMessage != nullptr) {
            *errorMessage = "could not write lod_representatives.bin";
        }
        return false;
    }
    RepresentativeFileHeader header;
    header.pointCount = preview.PointCount();
    header.representedSourceCount = representedSourceCount;
    header.scalarFieldCount = static_cast<std::uint32_t>(
        std::min<std::size_t>(preview.scalarFields.size(), std::numeric_limits<std::uint32_t>::max()));
    header.hasRgb = preview.hasSourceRgb ? 1U : 0U;
    header.hasNormals = preview.hasNormals ? 1U : 0U;
    output.write(reinterpret_cast<const char*>(&header), sizeof(header));
    for (const auto& field : preview.scalarFields) {
        if (!WriteString(output, field.name)) {
            return false;
        }
        output.write(reinterpret_cast<const char*>(&field.minimum), sizeof(field.minimum));
        output.write(reinterpret_cast<const char*>(&field.maximum), sizeof(field.maximum));
        output.write(reinterpret_cast<const char*>(&field.count), sizeof(field.count));
        const std::uint32_t valid = field.valid ? 1U : 0U;
        output.write(reinterpret_cast<const char*>(&valid), sizeof(valid));
    }
    for (std::size_t index = 0; index < preview.positions.size(); ++index) {
        const std::uint32_t sourcePointIndex =
            index < sourceIndices.size() ? sourceIndices[index] : static_cast<std::uint32_t>(index);
        const auto color = index < preview.packedColors.size() ? preview.packedColors[index] : PackRgba8(255, 255, 255);
        const auto normal = (preview.hasNormals && index < preview.normals.size()) ? preview.normals[index]
                                                                                   : invisible_places::io::Float3{};
        output.write(reinterpret_cast<const char*>(&sourcePointIndex), sizeof(sourcePointIndex));
        output.write(reinterpret_cast<const char*>(&preview.positions[index]), sizeof(preview.positions[index]));
        output.write(reinterpret_cast<const char*>(&color), sizeof(color));
        if (preview.hasNormals) {
            output.write(reinterpret_cast<const char*>(&normal), sizeof(normal));
        }
        for (std::size_t fieldIndex = 0; fieldIndex < preview.scalarFields.size(); ++fieldIndex) {
            float value = 0.0F;
            const auto valueIndex = preview.ScalarFieldValueIndex(fieldIndex, index);
            if (valueIndex < preview.scalarFieldValues.size()) {
                value = preview.scalarFieldValues[valueIndex];
            }
            output.write(reinterpret_cast<const char*>(&value), sizeof(value));
        }
    }
    if (!output && errorMessage != nullptr) {
        *errorMessage = "could not write representative payload";
    }
    return static_cast<bool>(output);
}

PointCloudIpcloudPreview ReadPreviewRepresentatives(const std::filesystem::path& path) {
    PointCloudIpcloudPreview result;
    std::ifstream input{path, std::ios::binary};
    if (!input) {
        result.reason = "missing lod_representatives.bin";
        return result;
    }

    RepresentativeFileHeader header;
    input.read(reinterpret_cast<char*>(&header), sizeof(header));
    if (!input || header.magic != kRepresentativeMagic || header.version != 1U ||
        header.headerSize != sizeof(RepresentativeFileHeader)) {
        result.reason = "invalid representative payload header";
        return result;
    }
    if (header.pointCount > static_cast<std::uint64_t>(std::numeric_limits<std::size_t>::max()) ||
        header.scalarFieldCount > static_cast<std::uint32_t>(std::numeric_limits<std::size_t>::max())) {
        result.reason = "representative payload too large";
        return result;
    }

    auto& cloud = result.cloud;
    cloud.hasSourceRgb = header.hasRgb != 0U;
    cloud.hasNormals = header.hasNormals != 0U;
    cloud.scalarFields.reserve(header.scalarFieldCount);
    for (std::uint32_t fieldIndex = 0; fieldIndex < header.scalarFieldCount; ++fieldIndex) {
        auto name = ReadString(input);
        if (!name.has_value()) {
            result.reason = "representative scalar schema truncated";
            return result;
        }
        invisible_places::io::ScalarFieldStats stats;
        stats.name = std::move(name.value());
        std::uint32_t valid = 0;
        input.read(reinterpret_cast<char*>(&stats.minimum), sizeof(stats.minimum));
        input.read(reinterpret_cast<char*>(&stats.maximum), sizeof(stats.maximum));
        input.read(reinterpret_cast<char*>(&stats.count), sizeof(stats.count));
        input.read(reinterpret_cast<char*>(&valid), sizeof(valid));
        if (!input) {
            result.reason = "representative scalar stats truncated";
            return result;
        }
        stats.valid = valid != 0U;
        cloud.scalarFields.push_back(std::move(stats));
    }

    const auto pointCount = static_cast<std::size_t>(header.pointCount);
    cloud.positions.reserve(pointCount);
    if (cloud.hasNormals) {
        cloud.normals.reserve(pointCount);
    }
    cloud.packedColors.reserve(pointCount);
    cloud.scalarFieldValues.resize(pointCount * cloud.scalarFields.size());
    result.sourcePointIndices.reserve(pointCount);
    for (std::size_t index = 0; index < pointCount; ++index) {
        std::uint32_t sourcePointIndex = 0;
        invisible_places::io::Float3 position{};
        std::uint32_t color = PackRgba8(255, 255, 255);
        invisible_places::io::Float3 normal{};
        input.read(reinterpret_cast<char*>(&sourcePointIndex), sizeof(sourcePointIndex));
        input.read(reinterpret_cast<char*>(&position), sizeof(position));
        input.read(reinterpret_cast<char*>(&color), sizeof(color));
        if (cloud.hasNormals) {
            input.read(reinterpret_cast<char*>(&normal), sizeof(normal));
        }
        if (!input) {
            result.reason = "representative point payload truncated";
            return result;
        }
        result.sourcePointIndices.push_back(sourcePointIndex);
        cloud.positions.push_back(position);
        cloud.bounds.Expand(position);
        cloud.packedColors.push_back(color);
        if (cloud.hasNormals) {
            cloud.normals.push_back(normal);
        }
        for (std::size_t fieldIndex = 0; fieldIndex < cloud.scalarFields.size(); ++fieldIndex) {
            float value = 0.0F;
            input.read(reinterpret_cast<char*>(&value), sizeof(value));
            if (!input) {
                result.reason = "representative scalar values truncated";
                return result;
            }
            cloud.scalarFieldValues[cloud.ScalarFieldValueIndex(fieldIndex, index)] = value;
        }
    }

    if (cloud.bounds.valid) {
        cloud.focusPoint = {
            0.5F * (cloud.bounds.minimum.x + cloud.bounds.maximum.x),
            0.5F * (cloud.bounds.minimum.y + cloud.bounds.maximum.y),
            0.5F * (cloud.bounds.minimum.z + cloud.bounds.maximum.z),
        };
        cloud.hasFocusPoint = true;
    }
    result.representedSourceCount = header.representedSourceCount;
    result.loaded = !cloud.positions.empty();
    result.status = result.loaded ? "Loaded .ipcloud representative preview" : "Representative preview empty";
    return result;
}

bool WriteHierarchyFile(const std::filesystem::path& path, const PointCloudLodHierarchy& hierarchy) {
    std::ofstream output{path, std::ios::binary | std::ios::trunc};
    if (!output) {
        return false;
    }
    HierarchyFileHeader header;
    header.nodeCount = hierarchy.nodes.size();
    header.representativeCount = hierarchy.representatives.size();
    header.scalarStatsCount = hierarchy.scalarFieldStats.size();
    header.nodeScalarStatsCount = hierarchy.nodeScalarStats.size();
    header.sourcePointCount = hierarchy.sourcePointCount;
    header.scalarFieldCount = hierarchy.scalarFieldCount;
    output.write(reinterpret_cast<const char*>(&header), sizeof(header));
    return WriteVector(output, hierarchy.nodes) &&
           WriteVector(output, hierarchy.representatives) &&
           WriteVector(output, hierarchy.scalarFieldStats) &&
           WriteVector(output, hierarchy.nodeScalarStats);
}

bool WriteNodeStatsFile(const std::filesystem::path& path, const PointCloudLodHierarchy& hierarchy) {
    std::ofstream output{path, std::ios::binary | std::ios::trunc};
    if (!output) {
        return false;
    }
    const std::uint64_t nodeCount = hierarchy.nodes.size();
    output.write(reinterpret_cast<const char*>(&nodeCount), sizeof(nodeCount));
    return WriteVector(output, hierarchy.nodes);
}

bool WriteScalarStatsFile(const std::filesystem::path& path, const invisible_places::io::LoadedPointCloud& cloud) {
    std::ofstream output{path, std::ios::binary | std::ios::trunc};
    if (!output) {
        return false;
    }
    ScalarStatsFileHeader header;
    header.scalarFieldCount = cloud.scalarFields.size();
    output.write(reinterpret_cast<const char*>(&header), sizeof(header));
    for (const auto& field : cloud.scalarFields) {
        if (!WriteString(output, field.name)) {
            return false;
        }
        output.write(reinterpret_cast<const char*>(&field.minimum), sizeof(field.minimum));
        output.write(reinterpret_cast<const char*>(&field.maximum), sizeof(field.maximum));
        output.write(reinterpret_cast<const char*>(&field.count), sizeof(field.count));
        const std::uint32_t valid = field.valid ? 1U : 0U;
        output.write(reinterpret_cast<const char*>(&valid), sizeof(valid));
    }
    return static_cast<bool>(output);
}

bool WriteAttributeSchemaFile(
    const std::filesystem::path& path,
    const PointCloudIpcloudSourceInfo& sourceInfo,
    const invisible_places::io::LoadedPointCloud& cloud) {
    std::ofstream output{path, std::ios::binary | std::ios::trunc};
    if (!output) {
        return false;
    }
    const std::uint64_t magic = kAttributeSchemaMagic;
    output.write(reinterpret_cast<const char*>(&magic), sizeof(magic));
    output.write(reinterpret_cast<const char*>(&sourceInfo.recordSizeBytes), sizeof(sourceInfo.recordSizeBytes));
    const std::uint32_t scalarFieldCount = static_cast<std::uint32_t>(
        std::min<std::size_t>(cloud.scalarFields.size(), std::numeric_limits<std::uint32_t>::max()));
    output.write(reinterpret_cast<const char*>(&scalarFieldCount), sizeof(scalarFieldCount));
    const std::uint32_t hasRgb = cloud.hasSourceRgb ? 1U : 0U;
    const std::uint32_t hasNormals = cloud.hasNormals ? 1U : 0U;
    output.write(reinterpret_cast<const char*>(&hasRgb), sizeof(hasRgb));
    output.write(reinterpret_cast<const char*>(&hasNormals), sizeof(hasNormals));
    for (const auto& field : cloud.scalarFields) {
        if (!WriteString(output, field.name)) {
            return false;
        }
    }
    return static_cast<bool>(output);
}

bool WriteNodePagesFile(const std::filesystem::path& path, const PointCloudLodHierarchy& hierarchy) {
    std::ofstream output{path, std::ios::binary | std::ios::trunc};
    if (!output) {
        return false;
    }
    std::uint32_t maxDepth = 0;
    for (const auto& node : hierarchy.nodes) {
        maxDepth = std::max(maxDepth, node.depth);
    }
    NodePagesFileHeader header;
    header.pageCount = static_cast<std::uint64_t>(maxDepth) + 1ULL;
    output.write(reinterpret_cast<const char*>(&header), sizeof(header));
    for (std::uint32_t depth = 0; depth <= maxDepth; ++depth) {
        std::uint64_t firstNode = std::numeric_limits<std::uint64_t>::max();
        std::uint64_t count = 0;
        for (std::size_t nodeIndex = 0; nodeIndex < hierarchy.nodes.size(); ++nodeIndex) {
            if (hierarchy.nodes[nodeIndex].depth == depth) {
                firstNode = std::min<std::uint64_t>(firstNode, nodeIndex);
                ++count;
            }
        }
        if (firstNode == std::numeric_limits<std::uint64_t>::max()) {
            firstNode = 0;
        }
        output.write(reinterpret_cast<const char*>(&depth), sizeof(depth));
        output.write(reinterpret_cast<const char*>(&firstNode), sizeof(firstNode));
        output.write(reinterpret_cast<const char*>(&count), sizeof(count));
    }
    return static_cast<bool>(output);
}

bool WriteRawChunkRanges(
    const std::filesystem::path& directory,
    const PointCloudIpcloudSourceInfo& sourceInfo,
    std::uint64_t* rawChunkCount) {
    std::error_code error;
    std::filesystem::create_directories(directory, error);
    if (error) {
        return false;
    }
    constexpr std::uint64_t kPointsPerChunk = 1'000'000ULL;
    const auto chunkCount = sourceInfo.pointCount == 0ULL
                                ? 0ULL
                                : ((sourceInfo.pointCount + kPointsPerChunk - 1ULL) / kPointsPerChunk);
    if (rawChunkCount != nullptr) {
        *rawChunkCount = chunkCount;
    }
    for (std::uint64_t chunkIndex = 0; chunkIndex < chunkCount; ++chunkIndex) {
        std::ostringstream name;
        name << "chunk_" << std::setfill('0') << std::setw(6) << chunkIndex << ".bin";
        std::ofstream output{directory / name.str(), std::ios::binary | std::ios::trunc};
        if (!output) {
            return false;
        }
        RawChunkRangeHeader header;
        header.firstPoint = chunkIndex * kPointsPerChunk;
        header.pointCount = std::min(kPointsPerChunk, sourceInfo.pointCount - header.firstPoint);
        header.sourceByteOffset =
            sourceInfo.dataOffsetBytes + (header.firstPoint * static_cast<std::uint64_t>(sourceInfo.recordSizeBytes));
        header.sourceByteCount = header.pointCount * static_cast<std::uint64_t>(sourceInfo.recordSizeBytes);
        output.write(reinterpret_cast<const char*>(&header), sizeof(header));
        if (!output) {
            return false;
        }
    }
    return true;
}

std::uint32_t MaxHierarchyDepth(const PointCloudLodHierarchy& hierarchy) {
    std::uint32_t maxDepth = 0;
    for (const auto& node : hierarchy.nodes) {
        maxDepth = std::max(maxDepth, node.depth);
    }
    return maxDepth;
}

std::uint64_t LeafNodeCount(const PointCloudLodHierarchy& hierarchy) {
    return static_cast<std::uint64_t>(std::count_if(
        hierarchy.nodes.begin(),
        hierarchy.nodes.end(),
        [](const PointCloudLodNode& node) { return node.IsLeaf(); }));
}

json BoundsJson(const invisible_places::io::Bounds3f& bounds) {
    if (!bounds.valid) {
        return json{{"valid", false}};
    }
    return json{
        {"valid", true},
        {"minimum", {bounds.minimum.x, bounds.minimum.y, bounds.minimum.z}},
        {"maximum", {bounds.maximum.x, bounds.maximum.y, bounds.maximum.z}},
    };
}

json MakeManifest(
    const PointCloudIpcloudSourceInfo& sourceInfo,
    const PointCloudLodBuildConfig& buildConfig,
    const invisible_places::io::LoadedPointCloud& cloud,
    const PointCloudLodHierarchy& hierarchy,
    std::uint64_t previewPointCount,
    std::uint64_t rawChunkCount) {
    const float estimatedSpacing =
        !hierarchy.nodes.empty() ? hierarchy.nodes.front().spacingMeters : 0.0F;
    return {
        {"cache_format", "InvisiblePlacesPointCloudIpcloud"},
        {"cache_format_version", kPointCloudIpcloudCacheFormatVersion},
        {"source",
         {
             {"path", sourceInfo.sourcePath.lexically_normal().generic_string()},
             {"path_hash", sourceInfo.normalizedPathHash},
             {"size_bytes", sourceInfo.sourceSizeBytes},
             {"modified_time_ns", sourceInfo.sourceWriteTimeNs},
             {"header_hash", sourceInfo.headerHash},
             {"sampled_content_hash", sourceInfo.sampledContentHash},
             {"point_count", sourceInfo.pointCount},
             {"scalar_field_count", sourceInfo.scalarFieldCount},
             {"has_rgb", sourceInfo.hasRgb},
             {"has_normals", sourceInfo.hasNormals},
         }},
        {"build",
         {
             {"attribute_schema_version", kPointCloudIpcloudAttributeSchemaVersion},
             {"build_settings_version", kPointCloudIpcloudBuildSettingsVersion},
             {"max_leaf_source_points", buildConfig.maxLeafSourcePoints},
             {"max_depth", buildConfig.maxDepth},
             {"max_internal_representatives", buildConfig.maxInternalRepresentatives},
         }},
        {"cloud",
         {
             {"bounds", BoundsJson(cloud.bounds)},
             {"estimated_raw_spacing_meters", estimatedSpacing},
             {"hierarchy_node_count", hierarchy.nodes.size()},
             {"leaf_chunk_count", LeafNodeCount(hierarchy)},
             {"max_tree_depth", MaxHierarchyDepth(hierarchy)},
         }},
        {"preview",
         {
             {"point_count", previewPointCount},
             {"represented_source_count", sourceInfo.pointCount},
         }},
        {"files",
         {
             {"attribute_schema", "attribute_schema.bin"},
             {"hierarchy", "hierarchy.bin"},
             {"node_pages", "node_pages.bin"},
             {"node_stats", "node_stats.bin"},
             {"lod_representatives", "lod_representatives.bin"},
             {"scalar_stats", "scalar_stats.bin"},
             {"raw_chunks", "raw_chunks/"},
         }},
        {"raw_chunk_count", rawChunkCount},
    };
}

std::vector<std::uint32_t> BuildStridedPreviewIndices(std::uint64_t pointCount, std::uint32_t targetCount) {
    std::vector<std::uint32_t> indices;
    if (pointCount == 0ULL || targetCount == 0U) {
        return indices;
    }
    const auto clampedTarget = static_cast<std::uint64_t>(std::min<std::uint64_t>(targetCount, pointCount));
    indices.reserve(static_cast<std::size_t>(clampedTarget));
    for (std::uint64_t outputIndex = 0; outputIndex < clampedTarget; ++outputIndex) {
        const auto sourceIndex = (outputIndex * pointCount) / clampedTarget;
        indices.push_back(static_cast<std::uint32_t>(std::min<std::uint64_t>(
            sourceIndex,
            static_cast<std::uint64_t>(std::numeric_limits<std::uint32_t>::max()))));
    }
    return indices;
}

bool ReadRecordAt(
    std::ifstream& input,
    const PointCloudIpcloudSourceInfo& sourceInfo,
    const PointCloudRecordLayout& layout,
    std::uint64_t pointIndex,
    std::vector<std::byte>* recordBuffer) {
    if (recordBuffer == nullptr || layout.recordSize == 0U) {
        return false;
    }
    recordBuffer->resize(layout.recordSize);
    const auto offset =
        sourceInfo.dataOffsetBytes + (pointIndex * static_cast<std::uint64_t>(layout.recordSize));
    input.clear();
    input.seekg(static_cast<std::streamoff>(offset), std::ios::beg);
    if (!input) {
        return false;
    }
    input.read(
        reinterpret_cast<char*>(recordBuffer->data()),
        static_cast<std::streamsize>(recordBuffer->size()));
    return input.gcount() == static_cast<std::streamsize>(recordBuffer->size());
}

}  // namespace

const char* PointCloudIpcloudCacheStateName(PointCloudIpcloudCacheState state) {
    switch (state) {
        case PointCloudIpcloudCacheState::Missing:
            return "missing";
        case PointCloudIpcloudCacheState::Hit:
            return "hit";
        case PointCloudIpcloudCacheState::Stale:
            return "stale";
        case PointCloudIpcloudCacheState::Partial:
            return "partial";
        case PointCloudIpcloudCacheState::Corrupt:
            return "corrupt";
    }
    return "unknown";
}

std::optional<PointCloudIpcloudSourceInfo> MakePointCloudIpcloudSourceInfo(
    const std::filesystem::path& sourcePath,
    std::string* errorMessage) {
    const auto headerResult = invisible_places::io::ParsePlyHeader(sourcePath);
    if (!headerResult.success) {
        if (errorMessage != nullptr) {
            *errorMessage = headerResult.errorMessage;
        }
        return std::nullopt;
    }
    if (headerResult.header.format != "binary_little_endian") {
        if (errorMessage != nullptr) {
            *errorMessage = "Only binary_little_endian PLY point clouds are supported.";
        }
        return std::nullopt;
    }
    std::string layoutError;
    const auto layout = BuildRecordLayout(headerResult.header, &layoutError);
    if (!layout.has_value()) {
        if (errorMessage != nullptr) {
            *errorMessage = layoutError;
        }
        return std::nullopt;
    }

    PointCloudIpcloudSourceInfo sourceInfo;
    sourceInfo.sourcePath = sourcePath;
    sourceInfo.normalizedPathHash = HashPointCloudLodCachePath(sourcePath);
    sourceInfo.headerHash = HeaderHash(headerResult.header);
    sourceInfo.pointCount = headerResult.header.vertexCount;
    sourceInfo.scalarFieldCount = static_cast<std::uint32_t>(
        std::min<std::size_t>(layout->scalarFieldNames.size(), std::numeric_limits<std::uint32_t>::max()));
    sourceInfo.hasRgb = layout->hasRgb;
    sourceInfo.hasNormals = layout->hasNormals;
    sourceInfo.dataOffsetBytes = headerResult.header.dataOffsetBytes;
    sourceInfo.recordSizeBytes = layout->recordSize;

    std::error_code error;
    if (std::filesystem::exists(sourcePath, error) && std::filesystem::is_regular_file(sourcePath, error)) {
        sourceInfo.sourceSizeBytes = std::filesystem::file_size(sourcePath, error);
        const auto writeTime = std::filesystem::last_write_time(sourcePath, error);
        if (!error) {
            sourceInfo.sourceWriteTimeNs =
                std::chrono::duration_cast<std::chrono::nanoseconds>(writeTime.time_since_epoch()).count();
        }
    }
    sourceInfo.sampledContentHash =
        SampledContentHash(sourcePath, sourceInfo.dataOffsetBytes, sourceInfo.sourceSizeBytes);
    return sourceInfo;
}

std::filesystem::path BuildPointCloudIpcloudBundlePath(
    const std::filesystem::path& cacheDirectory,
    const PointCloudIpcloudSourceInfo& sourceInfo) {
    const auto stem = sourceInfo.sourcePath.stem().empty() ? std::string{"PointCloud"}
                                                           : sourceInfo.sourcePath.stem().string();
    return cacheDirectory / (stem + "." + Hex64(FingerprintHash(sourceInfo)) + ".ipcloud");
}

std::filesystem::path BuildPointCloudIpcloudBundlePath(
    const std::filesystem::path& cacheDirectory,
    const std::filesystem::path& sourcePath) {
    std::string errorMessage;
    const auto sourceInfo = MakePointCloudIpcloudSourceInfo(sourcePath, &errorMessage);
    if (!sourceInfo.has_value()) {
        return {};
    }
    return BuildPointCloudIpcloudBundlePath(cacheDirectory, sourceInfo.value());
}

PointCloudIpcloudInspection InspectPointCloudIpcloudBundle(
    const std::filesystem::path& bundlePath,
    const PointCloudIpcloudSourceInfo& sourceInfo,
    const PointCloudLodBuildConfig& buildConfig) {
    PointCloudIpcloudInspection inspection;
    inspection.bundlePath = bundlePath;
    inspection.temporaryBundlePath = TemporaryBundlePath(bundlePath);

    std::error_code error;
    const bool hasCompleteBundle = std::filesystem::is_directory(bundlePath, error);
    const bool hasTemporaryBundle = std::filesystem::is_directory(inspection.temporaryBundlePath, error);
    if (!hasCompleteBundle) {
        if (hasTemporaryBundle) {
            inspection.state = PointCloudIpcloudCacheState::Partial;
            inspection.status = ".ipcloud partial";
            inspection.reason = "temporary bundle exists without complete publish";
            if (auto statusJson = ReadJsonFile(BuildStatusPath(inspection.temporaryBundlePath)); statusJson.has_value()) {
                inspection.buildStatus = ParseBuildStatus(statusJson.value());
            }
        } else {
            inspection.state = PointCloudIpcloudCacheState::Missing;
            inspection.status = ".ipcloud cache miss";
            inspection.reason = "bundle missing";
        }
        return inspection;
    }

    std::string missingFile;
    if (!RequiredBundleFilesExist(bundlePath, &missingFile)) {
        inspection.state = PointCloudIpcloudCacheState::Corrupt;
        inspection.status = ".ipcloud corrupt";
        inspection.reason = "missing " + missingFile;
        return inspection;
    }

    std::string jsonError;
    auto manifest = ReadJsonFile(ManifestPath(bundlePath), &jsonError);
    if (!manifest.has_value()) {
        inspection.state = PointCloudIpcloudCacheState::Corrupt;
        inspection.status = ".ipcloud corrupt";
        inspection.reason = "manifest invalid: " + jsonError;
        return inspection;
    }

    std::string validationReason;
    if (!ManifestMatchesSourceAndBuild(manifest.value(), sourceInfo, buildConfig, &validationReason)) {
        inspection.state = PointCloudIpcloudCacheState::Stale;
        inspection.status = ".ipcloud stale";
        inspection.reason = validationReason;
        return inspection;
    }

    if (auto statusJson = ReadJsonFile(BuildStatusPath(bundlePath)); statusJson.has_value()) {
        inspection.buildStatus = ParseBuildStatus(statusJson.value());
    }
    if (!inspection.buildStatus.complete || inspection.buildStatus.failed || inspection.buildStatus.interrupted) {
        inspection.state = PointCloudIpcloudCacheState::Partial;
        inspection.status = ".ipcloud partial";
        inspection.reason = inspection.buildStatus.message.empty() ? "bundle is not complete"
                                                                   : inspection.buildStatus.message;
        return inspection;
    }

    const auto preview = manifest->value("preview", json::object());
    inspection.previewPointCount = preview.value("point_count", 0ULL);
    inspection.representedSourceCount = preview.value("represented_source_count", sourceInfo.pointCount);
    inspection.state = PointCloudIpcloudCacheState::Hit;
    inspection.status = ".ipcloud cache hit";
    inspection.reason = "valid bundle";
    return inspection;
}

PointCloudIpcloudPreview LoadPointCloudIpcloudPreview(
    const std::filesystem::path& bundlePath,
    const PointCloudIpcloudSourceInfo& sourceInfo,
    const PointCloudLodBuildConfig& buildConfig) {
    const auto start = std::chrono::steady_clock::now();
    auto inspection = InspectPointCloudIpcloudBundle(bundlePath, sourceInfo, buildConfig);
    if (inspection.state != PointCloudIpcloudCacheState::Hit) {
        PointCloudIpcloudPreview result;
        result.status = inspection.status;
        result.reason = inspection.reason;
        return result;
    }
    auto result = ReadPreviewRepresentatives(bundlePath / "lod_representatives.bin");
    result.fromPersistentBundle = result.loaded;
    result.representedSourceCount = inspection.representedSourceCount;
    if (result.loaded) {
        result.cloud.sourcePath = sourceInfo.sourcePath;
        result.cloud.layerName = sourceInfo.sourcePath.stem().string();
        result.hierarchy = BuildPointCloudLodHierarchy(
            result.cloud,
            {.maxLeafSourcePoints = 256U, .maxDepth = 8U, .maxInternalRepresentatives = 16U});
        result.status = "Rendering coarse preview from .ipcloud cache";
    }
    const auto end = std::chrono::steady_clock::now();
    result.loadMilliseconds = std::chrono::duration<double, std::milli>(end - start).count();
    return result;
}

PointCloudIpcloudPreview LoadPointCloudIpcloudSourcePreview(
    const std::filesystem::path& sourcePath,
    std::uint32_t targetRepresentativeCount) {
    const auto start = std::chrono::steady_clock::now();
    PointCloudIpcloudPreview result;
    std::string sourceError;
    const auto sourceInfo = MakePointCloudIpcloudSourceInfo(sourcePath, &sourceError);
    if (!sourceInfo.has_value()) {
        result.status = "Could not inspect source preview";
        result.reason = sourceError;
        return result;
    }
    const auto headerResult = invisible_places::io::ParsePlyHeader(sourcePath);
    if (!headerResult.success) {
        result.status = "Could not parse source preview header";
        result.reason = headerResult.errorMessage;
        return result;
    }
    std::string layoutError;
    const auto layout = BuildRecordLayout(headerResult.header, &layoutError);
    if (!layout.has_value()) {
        result.status = "Could not build source preview schema";
        result.reason = layoutError;
        return result;
    }

    std::ifstream input{sourcePath, std::ios::binary};
    if (!input) {
        result.status = "Could not open source preview";
        result.reason = "unable to open source file";
        return result;
    }

    auto& cloud = result.cloud;
    cloud.sourcePath = sourcePath;
    cloud.layerName = sourcePath.stem().string();
    cloud.hasSourceRgb = layout->hasRgb;
    cloud.hasNormals = layout->hasNormals;
    cloud.scalarFields.reserve(layout->scalarFieldNames.size());
    for (const auto& name : layout->scalarFieldNames) {
        cloud.scalarFields.push_back({.name = name});
    }

    result.sourcePointIndices = BuildStridedPreviewIndices(sourceInfo->pointCount, targetRepresentativeCount);
    cloud.positions.reserve(result.sourcePointIndices.size());
    if (cloud.hasNormals) {
        cloud.normals.reserve(result.sourcePointIndices.size());
    }
    cloud.packedColors.reserve(result.sourcePointIndices.size());
    cloud.scalarFieldValues.resize(result.sourcePointIndices.size() * cloud.scalarFields.size());

    std::vector<std::byte> recordBuffer;
    for (std::size_t previewIndex = 0; previewIndex < result.sourcePointIndices.size(); ++previewIndex) {
        const auto sourcePointIndex = result.sourcePointIndices[previewIndex];
        if (!ReadRecordAt(input, sourceInfo.value(), layout.value(), sourcePointIndex, &recordBuffer)) {
            result.loaded = false;
            result.status = "Source preview truncated";
            result.reason = "could not read representative record";
            return result;
        }

        invisible_places::io::Float3 position{};
        invisible_places::io::Float3 normal{};
        std::uint8_t red = 255;
        std::uint8_t green = 255;
        std::uint8_t blue = 255;
        for (const auto& property : layout->properties) {
            const auto* bytes = recordBuffer.data() + property.offset;
            switch (property.semantic) {
                case PropertySemantic::PositionX:
                    position.x = static_cast<float>(ReadScalarAsDouble(bytes, property.type));
                    break;
                case PropertySemantic::PositionY:
                    position.y = static_cast<float>(ReadScalarAsDouble(bytes, property.type));
                    break;
                case PropertySemantic::PositionZ:
                    position.z = static_cast<float>(ReadScalarAsDouble(bytes, property.type));
                    break;
                case PropertySemantic::ColorR:
                    red = ReadScalarAsByte(bytes, property.type);
                    break;
                case PropertySemantic::ColorG:
                    green = ReadScalarAsByte(bytes, property.type);
                    break;
                case PropertySemantic::ColorB:
                    blue = ReadScalarAsByte(bytes, property.type);
                    break;
                case PropertySemantic::NormalX:
                    normal.x = static_cast<float>(ReadScalarAsDouble(bytes, property.type));
                    break;
                case PropertySemantic::NormalY:
                    normal.y = static_cast<float>(ReadScalarAsDouble(bytes, property.type));
                    break;
                case PropertySemantic::NormalZ:
                    normal.z = static_cast<float>(ReadScalarAsDouble(bytes, property.type));
                    break;
                case PropertySemantic::ScalarField: {
                    const float value = static_cast<float>(ReadScalarAsDouble(bytes, property.type));
                    const auto valueIndex = cloud.ScalarFieldValueIndex(property.scalarFieldIndex, previewIndex);
                    if (valueIndex < cloud.scalarFieldValues.size()) {
                        cloud.scalarFieldValues[valueIndex] = value;
                    }
                    if (property.scalarFieldIndex < cloud.scalarFields.size() && std::isfinite(value)) {
                        cloud.scalarFields[property.scalarFieldIndex].Include(value);
                    }
                    break;
                }
                case PropertySemantic::Skip:
                    break;
            }
        }
        cloud.positions.push_back(position);
        cloud.bounds.Expand(position);
        if (cloud.hasNormals) {
            cloud.normals.push_back(NormalizeNormal(normal));
        }
        cloud.packedColors.push_back(PackRgba8(red, green, blue));
    }
    if (cloud.bounds.valid) {
        cloud.focusPoint = {
            0.5F * (cloud.bounds.minimum.x + cloud.bounds.maximum.x),
            0.5F * (cloud.bounds.minimum.y + cloud.bounds.maximum.y),
            0.5F * (cloud.bounds.minimum.z + cloud.bounds.maximum.z),
        };
        cloud.hasFocusPoint = true;
    }
    result.representedSourceCount = sourceInfo->pointCount;
    result.hierarchy = BuildPointCloudLodHierarchy(
        cloud,
        {.maxLeafSourcePoints = 256U, .maxDepth = 8U, .maxInternalRepresentatives = 16U});
    result.loaded = !cloud.positions.empty() && !result.hierarchy.Empty();
    result.status = result.loaded ? "Rendering coarse preview from source sample" : "Source preview empty";
    const auto end = std::chrono::steady_clock::now();
    result.loadMilliseconds = std::chrono::duration<double, std::milli>(end - start).count();
    return result;
}

PointCloudIpcloudSaveResult SavePointCloudIpcloudBundle(
    const std::filesystem::path& bundlePath,
    const PointCloudIpcloudSourceInfo& sourceInfo,
    const PointCloudLodBuildConfig& buildConfig,
    const invisible_places::io::LoadedPointCloud& cloud,
    const PointCloudLodHierarchy& hierarchy) {
    PointCloudIpcloudSaveResult result;
    result.bundlePath = bundlePath;
    if (bundlePath.empty() || cloud.positions.empty() || hierarchy.Empty()) {
        result.errorMessage = "empty cloud or hierarchy";
        return result;
    }

    const auto temporaryPath = TemporaryBundlePath(bundlePath);
    std::error_code error;
    std::filesystem::remove_all(temporaryPath, error);
    error.clear();
    std::filesystem::create_directories(temporaryPath / "raw_chunks", error);
    if (error) {
        result.errorMessage = "could not create temporary .ipcloud bundle: " + error.message();
        return result;
    }

    auto writeStatus = [&](PointCloudIpcloudBuildStatus status) {
        status.interrupted = !status.complete && !status.failed;
        static_cast<void>(WriteJsonFile(BuildStatusPath(temporaryPath), BuildStatusJson(status)));
    };
    writeStatus({
        .parseHeaderComplete = true,
        .phase = "Writing .ipcloud bundle",
        .message = "temporary bundle in progress",
    });

    std::string writeError;
    const auto previewSourceIndices = SelectPreviewSourceIndices(
        cloud,
        hierarchy,
        kPointCloudIpcloudDefaultPreviewPointCount);
    const auto previewCloud = MakePreviewCloudFromSourceIndices(cloud, previewSourceIndices);
    if (!WritePreviewRepresentatives(
            temporaryPath / "lod_representatives.bin",
            previewCloud,
            previewSourceIndices,
            sourceInfo.pointCount,
            &writeError)) {
        result.errorMessage = writeError;
        return result;
    }
    writeStatus({
        .parseHeaderComplete = true,
        .upperHierarchyComplete = true,
        .representativeRangesComplete = true,
        .phase = "Representatives written",
        .message = "representative preview complete",
    });

    if (!WriteAttributeSchemaFile(temporaryPath / "attribute_schema.bin", sourceInfo, cloud) ||
        !WriteHierarchyFile(temporaryPath / "hierarchy.bin", hierarchy) ||
        !WriteNodePagesFile(temporaryPath / "node_pages.bin", hierarchy) ||
        !WriteNodeStatsFile(temporaryPath / "node_stats.bin", hierarchy) ||
        !WriteScalarStatsFile(temporaryPath / "scalar_stats.bin", cloud)) {
        result.errorMessage = "could not write .ipcloud hierarchy/schema files";
        return result;
    }
    writeStatus({
        .parseHeaderComplete = true,
        .upperHierarchyComplete = true,
        .nodePagesComplete = true,
        .representativeRangesComplete = true,
        .scalarStatsComplete = true,
        .phase = "Hierarchy written",
        .message = "hierarchy and scalar stats complete",
    });

    std::uint64_t rawChunkCount = 0;
    if (!WriteRawChunkRanges(temporaryPath / "raw_chunks", sourceInfo, &rawChunkCount)) {
        result.errorMessage = "could not write raw chunk range metadata";
        return result;
    }

    const PointCloudIpcloudBuildStatus completeStatus{
        .parseHeaderComplete = true,
        .upperHierarchyComplete = true,
        .nodePagesComplete = true,
        .representativeRangesComplete = true,
        .rawChunkRangesComplete = true,
        .scalarStatsComplete = true,
        .complete = true,
        .failed = false,
        .interrupted = false,
        .rawChunksCompleted = rawChunkCount,
        .rawChunkCount = rawChunkCount,
        .phase = "Complete",
        .message = "published .ipcloud bundle",
    };
    if (!WriteJsonFile(
            temporaryPath / "manifest.json",
            MakeManifest(
                sourceInfo,
                buildConfig,
                cloud,
                hierarchy,
                previewCloud.PointCount(),
                rawChunkCount),
            &writeError) ||
        !WriteJsonFile(BuildStatusPath(temporaryPath), BuildStatusJson(completeStatus), &writeError)) {
        result.errorMessage = writeError.empty() ? "could not write .ipcloud manifest/status" : writeError;
        return result;
    }
    {
        std::ofstream log{temporaryPath / "build_log.txt", std::ios::trunc};
        log << "Built .ipcloud v" << kPointCloudIpcloudCacheFormatVersion << "\n"
            << "source=" << sourceInfo.sourcePath.lexically_normal().generic_string() << "\n"
            << "points=" << sourceInfo.pointCount << "\n"
            << "preview_points=" << previewCloud.PointCount() << "\n"
            << "raw_chunk_ranges=" << rawChunkCount << "\n";
    }

    auto previousBundlePath = bundlePath;
    previousBundlePath += ".old";
    std::filesystem::remove_all(previousBundlePath, error);
    error.clear();
    const bool hadPreviousBundle = std::filesystem::exists(bundlePath, error);
    if (error) {
        std::filesystem::remove_all(temporaryPath, error);
        result.errorMessage = "could not inspect existing .ipcloud bundle: " + error.message();
        return result;
    }
    if (hadPreviousBundle) {
        std::filesystem::rename(bundlePath, previousBundlePath, error);
        if (error) {
            std::filesystem::remove_all(temporaryPath, error);
            result.errorMessage = "could not preserve previous .ipcloud bundle: " + error.message();
            return result;
        }
    }
    error.clear();
    std::filesystem::rename(temporaryPath, bundlePath, error);
    if (error) {
        const auto publishError = error.message();
        if (hadPreviousBundle) {
            std::error_code restoreError;
            std::filesystem::rename(previousBundlePath, bundlePath, restoreError);
        }
        std::filesystem::remove_all(temporaryPath, error);
        result.errorMessage = "could not publish .ipcloud bundle: " + publishError;
        return result;
    }
    if (hadPreviousBundle) {
        std::filesystem::remove_all(previousBundlePath, error);
    }
    result.saved = true;
    result.status = ".ipcloud bundle ready";
    return result;
}

PointCloudIpcloudResumeResult ResumePointCloudIpcloudBuild(
    const std::filesystem::path& bundlePath,
    const PointCloudIpcloudSourceInfo& sourceInfo,
    const PointCloudLodBuildConfig& buildConfig) {
    PointCloudIpcloudResumeResult result;
    const auto inspection = InspectPointCloudIpcloudBundle(bundlePath, sourceInfo, buildConfig);
    result.buildStatus = inspection.buildStatus;
    if (inspection.state == PointCloudIpcloudCacheState::Hit) {
        result.resumable = false;
        result.restartRequired = false;
        result.status = ".ipcloud already complete";
        return result;
    }
    if (inspection.state == PointCloudIpcloudCacheState::Partial &&
        (inspection.buildStatus.representativeRangesComplete ||
         inspection.buildStatus.upperHierarchyComplete ||
         inspection.buildStatus.rawChunksCompleted > 0ULL) &&
        !inspection.buildStatus.failed) {
        result.resumable = true;
        result.restartRequired = false;
        result.status = ".ipcloud partial build resumable";
        result.reason = inspection.reason;
        return result;
    }
    result.resumable = false;
    result.restartRequired = true;
    result.status = ".ipcloud rebuild required";
    result.reason = inspection.reason.empty() ? inspection.status : inspection.reason;
    return result;
}

}  // namespace invisible_places::renderer::pointcloud
