#include "io/AdaptiveHqCache.hpp"

#include "io/PlyHeader.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <bit>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstring>
#include <exception>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <numeric>
#include <queue>
#include <sstream>
#include <system_error>
#include <thread>
#include <utility>

namespace invisible_places::io {

namespace {

constexpr std::uint64_t kFingerprintOffset = 14695981039346656037ULL;
constexpr std::uint64_t kFingerprintPrime = 1099511628211ULL;
constexpr std::size_t kFingerprintWindowBytes = 16U * 1024U;
constexpr std::size_t kSortRunTargetBytes = 128U * 1024U * 1024U;
constexpr std::size_t kMergeReadBufferBytes = 512U * 1024U;

enum class CacheScalarType : std::uint8_t {
    Int8,
    UInt8,
    Int16,
    UInt16,
    Int32,
    UInt32,
    Float32,
    Float64,
};

struct SourceLayout {
    struct TypedOffset {
        std::uint32_t offset = 0U;
        CacheScalarType type = CacheScalarType::Float32;
        bool present = false;
    };

    struct ScalarField {
        std::string name;
        std::uint32_t sourceIndex = 0U;
        std::uint32_t offset = 0U;
        CacheScalarType type = CacheScalarType::Float32;
    };

    PlyHeader header;
    std::uint32_t recordSize = 0U;
    TypedOffset x;
    TypedOffset y;
    TypedOffset z;
    TypedOffset red;
    TypedOffset green;
    TypedOffset blue;
    TypedOffset normalX;
    TypedOffset normalY;
    TypedOffset normalZ;
    bool hasSourceRgb = false;
    bool hasNormals = false;
    std::vector<ScalarField> scalarFields;
};

struct SourceLayoutResult {
    SourceLayout layout;
    std::string errorMessage;
    bool success = false;
};

struct SortedRun {
    std::filesystem::path path;
    std::uint64_t entryCount = 0U;
};

void FingerprintBytes(
    std::uint64_t* hash,
    const void* bytes,
    std::size_t byteCount) {
    const auto* data = static_cast<const std::uint8_t*>(bytes);
    for (std::size_t index = 0U; index < byteCount; ++index) {
        *hash ^= data[index];
        *hash *= kFingerprintPrime;
    }
}

void FingerprintString(std::uint64_t* hash, std::string_view value) {
    FingerprintBytes(hash, value.data(), value.size());
    constexpr std::uint8_t separator = 0xffU;
    FingerprintBytes(hash, &separator, sizeof(separator));
}

template <typename Value>
void FingerprintValue(std::uint64_t* hash, const Value& value) {
    FingerprintBytes(hash, &value, sizeof(value));
}

std::string FingerprintHex(std::uint64_t hash) {
    std::ostringstream output;
    output << std::hex << std::setfill('0') << std::setw(16) << hash;
    return output.str();
}

std::filesystem::path NormalizedAbsolutePath(
    const std::filesystem::path& path) {
    std::error_code error;
    auto normalized = std::filesystem::weakly_canonical(path, error);
    if (!error) {
        return normalized;
    }
    error.clear();
    normalized = std::filesystem::absolute(path, error);
    return error ? path.lexically_normal() : normalized.lexically_normal();
}

bool LooksLikeCloudSyncedPath(const std::filesystem::path& path) {
    auto text = path.generic_string();
    std::transform(
        text.begin(),
        text.end(),
        text.begin(),
        [](unsigned char value) {
            return static_cast<char>(std::tolower(value));
        });
    return text.find("/library/cloudstorage/") != std::string::npos ||
           text.find("onedrive") != std::string::npos;
}

std::optional<CacheScalarType> ParseScalarType(std::string_view value) {
    if (value == "char" || value == "int8") {
        return CacheScalarType::Int8;
    }
    if (value == "uchar" || value == "uint8") {
        return CacheScalarType::UInt8;
    }
    if (value == "short" || value == "int16") {
        return CacheScalarType::Int16;
    }
    if (value == "ushort" || value == "uint16") {
        return CacheScalarType::UInt16;
    }
    if (value == "int" || value == "int32") {
        return CacheScalarType::Int32;
    }
    if (value == "uint" || value == "uint32") {
        return CacheScalarType::UInt32;
    }
    if (value == "float" || value == "float32") {
        return CacheScalarType::Float32;
    }
    if (value == "double" || value == "float64") {
        return CacheScalarType::Float64;
    }
    return std::nullopt;
}

std::uint32_t ScalarTypeSize(CacheScalarType type) {
    switch (type) {
        case CacheScalarType::Int8:
        case CacheScalarType::UInt8:
            return 1U;
        case CacheScalarType::Int16:
        case CacheScalarType::UInt16:
            return 2U;
        case CacheScalarType::Int32:
        case CacheScalarType::UInt32:
        case CacheScalarType::Float32:
            return 4U;
        case CacheScalarType::Float64:
            return 8U;
    }
    return 0U;
}

template <typename Value>
Value ReadLittleEndian(const std::byte* bytes) {
    Value value{};
    std::memcpy(&value, bytes, sizeof(Value));
    if constexpr (std::endian::native == std::endian::big) {
        auto* valueBytes = reinterpret_cast<std::byte*>(&value);
        std::reverse(valueBytes, valueBytes + sizeof(Value));
    }
    return value;
}

double ReadScalar(const std::byte* bytes, CacheScalarType type) {
    switch (type) {
        case CacheScalarType::Int8:
            return ReadLittleEndian<std::int8_t>(bytes);
        case CacheScalarType::UInt8:
            return ReadLittleEndian<std::uint8_t>(bytes);
        case CacheScalarType::Int16:
            return ReadLittleEndian<std::int16_t>(bytes);
        case CacheScalarType::UInt16:
            return ReadLittleEndian<std::uint16_t>(bytes);
        case CacheScalarType::Int32:
            return ReadLittleEndian<std::int32_t>(bytes);
        case CacheScalarType::UInt32:
            return ReadLittleEndian<std::uint32_t>(bytes);
        case CacheScalarType::Float32:
            return ReadLittleEndian<float>(bytes);
        case CacheScalarType::Float64:
            return ReadLittleEndian<double>(bytes);
    }
    return 0.0;
}

SourceLayoutResult BuildSourceLayout(
    const std::filesystem::path& sourcePath) {
    const auto parsed = ParsePlyHeader(sourcePath);
    if (!parsed.success) {
        return {.errorMessage = parsed.errorMessage};
    }
    if (parsed.header.format != "binary_little_endian") {
        return {
            .errorMessage =
                "Adaptive HQ caching requires a binary little-endian PLY.",
        };
    }
    if (!parsed.header.LooksLikePointCloud()) {
        return {.errorMessage = "The PLY is not a point-cloud source."};
    }
    if (parsed.header.vertexCount >
        static_cast<std::uint64_t>(
            std::numeric_limits<std::uint32_t>::max())) {
        return {
            .errorMessage =
                "Adaptive HQ currently supports at most 4,294,967,295 points per source.",
        };
    }

    SourceLayout layout;
    layout.header = parsed.header;
    const bool hasLongNormalTriplet =
        parsed.header.HasProperty("normal_x") &&
        parsed.header.HasProperty("normal_y") &&
        parsed.header.HasProperty("normal_z");
    const bool hasShortNormalTriplet =
        !hasLongNormalTriplet && parsed.header.HasProperty("nx") &&
        parsed.header.HasProperty("ny") && parsed.header.HasProperty("nz");
    std::uint64_t recordSize = 0U;
    for (const auto& property : parsed.header.properties) {
        if (property.isList) {
            return {
                .errorMessage =
                    "Adaptive HQ cannot cache list-valued vertex properties.",
            };
        }
        if (property.name == kPointCloudSourceIndexPropertyName) {
            return {
                .errorMessage =
                    "The source already contains the reserved adaptive-HQ index property.",
            };
        }
        const auto type = ParseScalarType(property.type);
        if (!type.has_value()) {
            return {
                .errorMessage =
                    "Adaptive HQ encountered an unsupported PLY scalar type: " +
                    property.type,
            };
        }
        if (recordSize >
            std::numeric_limits<std::uint32_t>::max() -
                ScalarTypeSize(*type)) {
            return {.errorMessage = "The PLY vertex record is too large."};
        }
        const auto offset = static_cast<std::uint32_t>(recordSize);
        const auto assign = [offset, type](SourceLayout::TypedOffset* target) {
            target->offset = offset;
            target->type = *type;
            target->present = true;
        };
        if (property.name == "x") {
            assign(&layout.x);
        } else if (property.name == "y") {
            assign(&layout.y);
        } else if (property.name == "z") {
            assign(&layout.z);
        } else if (property.name == "red") {
            assign(&layout.red);
        } else if (property.name == "green") {
            assign(&layout.green);
        } else if (property.name == "blue") {
            assign(&layout.blue);
        } else if ((hasLongNormalTriplet && property.name == "normal_x") ||
                   (hasShortNormalTriplet && property.name == "nx")) {
            assign(&layout.normalX);
        } else if ((hasLongNormalTriplet && property.name == "normal_y") ||
                   (hasShortNormalTriplet && property.name == "ny")) {
            assign(&layout.normalY);
        } else if ((hasLongNormalTriplet && property.name == "normal_z") ||
                   (hasShortNormalTriplet && property.name == "nz")) {
            assign(&layout.normalZ);
        } else if (property.name.rfind("scalar_", 0U) == 0U) {
            layout.scalarFields.push_back({
                .name = property.name.substr(7U),
                .sourceIndex = static_cast<std::uint32_t>(
                    layout.scalarFields.size()),
                .offset = offset,
                .type = *type,
            });
        }
        recordSize += ScalarTypeSize(*type);
    }
    if (!layout.x.present || !layout.y.present || !layout.z.present ||
        recordSize == 0U) {
        return {.errorMessage = "The PLY has no complete XYZ vertex layout."};
    }
    layout.hasSourceRgb =
        layout.red.present && layout.green.present && layout.blue.present;
    layout.hasNormals = layout.normalX.present && layout.normalY.present &&
                        layout.normalZ.present;
    layout.recordSize = static_cast<std::uint32_t>(recordSize);
    return {.layout = std::move(layout), .success = true};
}

Float3 ReadPosition(const std::byte* record, const SourceLayout& layout) {
    return {
        .x = static_cast<float>(
            ReadScalar(record + layout.x.offset, layout.x.type)),
        .y = static_cast<float>(
            ReadScalar(record + layout.y.offset, layout.y.type)),
        .z = static_cast<float>(
            ReadScalar(record + layout.z.offset, layout.z.type)),
    };
}

Float3 ReadNormal(const std::byte* record, const SourceLayout& layout) {
    if (!layout.hasNormals) {
        return {};
    }
    Float3 normal{
        .x = static_cast<float>(ReadScalar(
            record + layout.normalX.offset,
            layout.normalX.type)),
        .y = static_cast<float>(ReadScalar(
            record + layout.normalY.offset,
            layout.normalY.type)),
        .z = static_cast<float>(ReadScalar(
            record + layout.normalZ.offset,
            layout.normalZ.type)),
    };
    if (!std::isfinite(normal.x) || !std::isfinite(normal.y) ||
        !std::isfinite(normal.z)) {
        return {};
    }
    const auto lengthSquared = normal.x * normal.x + normal.y * normal.y +
                               normal.z * normal.z;
    if (lengthSquared <= 1.0e-12F) {
        return {};
    }
    const auto inverseLength = 1.0F / std::sqrt(lengthSquared);
    normal.x *= inverseLength;
    normal.y *= inverseLength;
    normal.z *= inverseLength;
    return normal;
}

std::uint8_t ReadColorChannel(
    const std::byte* record,
    const SourceLayout::TypedOffset& property) {
    if (!property.present) {
        return 255U;
    }
    return static_cast<std::uint8_t>(std::clamp(
        ReadScalar(record + property.offset, property.type),
        0.0,
        255.0));
}

std::uint32_t ReadPackedColor(
    const std::byte* record,
    const SourceLayout& layout) {
    const auto red = ReadColorChannel(record, layout.red);
    const auto green = ReadColorChannel(record, layout.green);
    const auto blue = ReadColorChannel(record, layout.blue);
    return static_cast<std::uint32_t>(red) |
           (static_cast<std::uint32_t>(green) << 8U) |
           (static_cast<std::uint32_t>(blue) << 16U) | (0xffU << 24U);
}

bool IsFinite(const Float3& point) {
    return std::isfinite(point.x) && std::isfinite(point.y) &&
           std::isfinite(point.z);
}

std::string SchemaFingerprint(const SourceLayout& layout) {
    auto hash = kFingerprintOffset;
    FingerprintString(&hash, layout.header.format);
    FingerprintValue(&hash, layout.header.vertexCount);
    for (const auto& property : layout.header.properties) {
        FingerprintString(&hash, property.type);
        FingerprintString(&hash, property.name);
        FingerprintValue(&hash, property.isList);
        FingerprintString(&hash, property.listCountType);
        FingerprintString(&hash, property.listValueType);
    }
    return FingerprintHex(hash);
}

std::optional<std::string> SampledContentFingerprint(
    const std::filesystem::path& sourcePath,
    std::uintmax_t byteSize,
    std::string* errorMessage) {
    std::ifstream input{sourcePath, std::ios::binary};
    if (!input.is_open()) {
        *errorMessage = "Unable to open the source for fingerprinting.";
        return std::nullopt;
    }
    auto hash = kFingerprintOffset;
    FingerprintValue(&hash, byteSize);
    if (byteSize == 0U) {
        return FingerprintHex(hash);
    }
    const auto window = std::min<std::uintmax_t>(
        byteSize,
        kFingerprintWindowBytes);
    std::array<std::uintmax_t, 5U> positions{
        0U,
        byteSize / 4U,
        byteSize / 2U,
        (byteSize * 3U) / 4U,
        byteSize > window ? byteSize - window : 0U,
    };
    std::vector<std::byte> bytes(static_cast<std::size_t>(window));
    for (auto position : positions) {
        position = std::min(position, byteSize - window);
        input.clear();
        input.seekg(static_cast<std::streamoff>(position), std::ios::beg);
        input.read(
            reinterpret_cast<char*>(bytes.data()),
            static_cast<std::streamsize>(bytes.size()));
        if (input.gcount() != static_cast<std::streamsize>(bytes.size())) {
            *errorMessage = "Unable to sample the source fingerprint.";
            return std::nullopt;
        }
        FingerprintValue(&hash, position);
        FingerprintBytes(&hash, bytes.data(), bytes.size());
    }
    return FingerprintHex(hash);
}

std::string IdentityFingerprint(const AdaptiveHqSourceIdentity& identity) {
    auto hash = kFingerprintOffset;
    FingerprintString(&hash, kAdaptiveHqCacheAlgorithmId);
    FingerprintValue(&hash, kAdaptiveHqCacheSchemaVersion);
    FingerprintString(&hash, identity.path.generic_string());
    FingerprintValue(&hash, identity.byteSize);
    FingerprintValue(&hash, identity.writeTimeTicks);
    FingerprintString(&hash, identity.schemaFingerprint);
    FingerprintString(&hash, identity.contentFingerprint);
    FingerprintValue(&hash, identity.pointCount);
    FingerprintValue(&hash, identity.recordSize);
    return FingerprintHex(hash);
}

std::string SourcePathKey(const std::filesystem::path& sourcePath) {
    auto hash = kFingerprintOffset;
    FingerprintString(&hash, sourcePath.generic_string());
    return FingerprintHex(hash);
}

std::int64_t FileWriteTimeTicks(
    const std::filesystem::path& path,
    std::error_code* error) {
    const auto writeTime = std::filesystem::last_write_time(path, *error);
    if (*error) {
        return 0;
    }
    return static_cast<std::int64_t>(writeTime.time_since_epoch().count());
}

struct SortedRunsResult {
    std::vector<SortedRun> runs;
    std::string errorMessage;
    bool success = false;
    bool cancelled = false;
};

struct MergeRunsResult {
    std::uint64_t dataOffsetBytes = 0U;
    std::uint64_t dataByteSize = 0U;
    std::uint64_t scalarDataByteSize = 0U;
    std::uint32_t geometryBytesPerPoint = 0U;
    bool hasSourceRgb = false;
    bool hasNormals = false;
    std::vector<AdaptiveHqCacheScalarField> scalarFields;
    std::vector<AdaptiveHqCacheBlock> blocks;
    std::string errorMessage;
    bool success = false;
    bool cancelled = false;
};

class TemporaryDirectoryGuard {
  public:
    explicit TemporaryDirectoryGuard(std::filesystem::path path)
        : path_(std::move(path)) {}
    ~TemporaryDirectoryGuard() {
        std::error_code ignored;
        std::filesystem::remove_all(path_, ignored);
    }

  private:
    std::filesystem::path path_;
};

std::optional<AdaptiveHqCacheIndex> LoadValidatedCacheIndex(
    const std::filesystem::path& cacheRoot,
    const std::filesystem::path& indexPath,
    const AdaptiveHqSourceIdentity& expectedSource);
std::optional<Bounds3f> ScanSourceBounds(
    const AdaptiveHqSourceIdentity& source,
    const SourceLayout& layout,
    std::stop_token stopToken,
    const AdaptiveHqCacheProgress& progress,
    std::string* errorMessage);
SortedRunsResult CreateSortedRuns(
    const AdaptiveHqSourceIdentity& source,
    const SourceLayout& layout,
    const Bounds3f& sourceBounds,
    const std::filesystem::path& temporaryDirectory,
    std::stop_token stopToken,
    const AdaptiveHqCacheProgress& progress);
MergeRunsResult MergeSortedRuns(
    const AdaptiveHqSourceIdentity& source,
    const SourceLayout& layout,
    std::span<const SortedRun> runs,
    const std::filesystem::path& geometryOutputPath,
    const std::filesystem::path& scalarOutputPath,
    std::stop_token stopToken,
    const AdaptiveHqCacheProgress& progress);
nlohmann::json CacheIndexToJson(const AdaptiveHqCacheIndex& index);

}  // namespace

AdaptiveHqCacheOpenResult OpenOrBuildAdaptiveHqCache(
    const std::filesystem::path& cacheRoot,
    const AdaptiveHqSourceIdentity& expectedSource,
    std::stop_token stopToken,
    const AdaptiveHqCacheProgress& progress) {
    if (stopToken.stop_requested()) {
        return {.cancelled = true};
    }
    const auto inspected = InspectAdaptiveHqSource(expectedSource.path);
    if (!inspected.success) {
        return {.errorMessage = inspected.errorMessage};
    }
    if (inspected.identity != expectedSource) {
        return {
            .errorMessage =
                "The adaptive-HQ source changed before its cache request began; retrying with its new identity is required.",
        };
    }

    const auto normalizedRoot = NormalizedAbsolutePath(cacheRoot);
    const auto sourceDirectory = expectedSource.path.parent_path();
    if (normalizedRoot.empty() || normalizedRoot == sourceDirectory ||
        LooksLikeCloudSyncedPath(normalizedRoot)) {
        return {
            .errorMessage =
                "The adaptive-HQ cache root must be the machine-local Saved cache, never the source directory or OneDrive.",
        };
    }
    const auto entryRoot = normalizedRoot /
        SourcePathKey(expectedSource.path);
    const auto indexPath = entryRoot / "index.json";
    std::error_code fileError;
    std::filesystem::create_directories(entryRoot, fileError);
    if (fileError) {
        return {
            .errorMessage =
                "Unable to create Saved/.invisible_places/cache/adaptive_hq.",
        };
    }
    if (const auto cached = LoadValidatedCacheIndex(
            entryRoot,
            indexPath,
            expectedSource);
        cached.has_value()) {
        if (progress) {
            progress(1.0F);
        }
        return {
            .index = *cached,
            .success = true,
            .built = false,
        };
    }

    const auto layout = BuildSourceLayout(expectedSource.path);
    if (!layout.success) {
        return {.errorMessage = layout.errorMessage};
    }
    const auto payloadEnd = layout.layout.header.dataOffsetBytes +
        expectedSource.pointCount * expectedSource.recordSize;
    if (payloadEnd > expectedSource.byteSize) {
        return {
            .errorMessage =
                "The source PLY payload is shorter than its declared vertex data.",
        };
    }

    const auto uniqueTicks = std::chrono::steady_clock::now()
                                 .time_since_epoch()
                                 .count();
    std::ostringstream uniqueTokenStream;
    uniqueTokenStream << std::hex << uniqueTicks;
    const auto uniqueToken = uniqueTokenStream.str();
    const auto temporaryDirectory =
        entryRoot / (".building-" + uniqueToken);
    std::filesystem::create_directories(temporaryDirectory, fileError);
    if (fileError) {
        return {
            .errorMessage =
                "Unable to create the adaptive-HQ temporary build directory.",
        };
    }
    TemporaryDirectoryGuard temporaryGuard{temporaryDirectory};

    std::string boundsError;
    const auto sourceBounds = ScanSourceBounds(
        expectedSource,
        layout.layout,
        stopToken,
        progress,
        &boundsError);
    if (!sourceBounds.has_value()) {
        return {
            .errorMessage = boundsError,
            .cancelled = stopToken.stop_requested(),
        };
    }
    const auto runs = CreateSortedRuns(
        expectedSource,
        layout.layout,
        *sourceBounds,
        temporaryDirectory,
        stopToken,
        progress);
    if (!runs.success) {
        return {
            .errorMessage = runs.errorMessage,
            .cancelled = runs.cancelled,
        };
    }

    const auto temporaryDataPath = temporaryDirectory / "geometry.bin";
    const auto temporaryScalarPath = temporaryDirectory / "scalars.bin";
    auto merged = MergeSortedRuns(
        expectedSource,
        layout.layout,
        runs.runs,
        temporaryDataPath,
        temporaryScalarPath,
        stopToken,
        progress);
    if (!merged.success) {
        return {
            .errorMessage = merged.errorMessage,
            .cancelled = merged.cancelled,
        };
    }
    const auto finalInspection = InspectAdaptiveHqSource(expectedSource.path);
    if (!finalInspection.success ||
        finalInspection.identity != expectedSource) {
        return {
            .errorMessage =
                "The source PLY changed while its adaptive-HQ cache was building; the incomplete cache was discarded.",
        };
    }

    const auto cacheFingerprint = IdentityFingerprint(expectedSource);
    const auto dataFileName =
        "geometry-" + cacheFingerprint + "-" + uniqueToken + ".bin";
    const auto scalarFileName =
        "scalars-" + cacheFingerprint + "-" + uniqueToken + ".bin";
    const auto finalDataPath = entryRoot / dataFileName;
    const auto finalScalarPath = entryRoot / scalarFileName;
    std::filesystem::rename(
        temporaryDataPath,
        finalDataPath,
        fileError);
    if (fileError) {
        return {
            .errorMessage =
                "Unable to publish the adaptive-HQ geometry cache locally.",
        };
    }
    fileError.clear();
    std::filesystem::rename(
        temporaryScalarPath,
        finalScalarPath,
        fileError);
    if (fileError) {
        std::error_code ignored;
        std::filesystem::remove(finalDataPath, ignored);
        return {
            .errorMessage =
                "Unable to publish the adaptive-HQ scalar cache locally.",
        };
    }

    AdaptiveHqCacheIndex builtIndex{
        .cacheRoot = entryRoot,
        .indexPath = indexPath,
        .dataPath = finalDataPath,
        .scalarDataPath = finalScalarPath,
        .cacheFingerprint = cacheFingerprint,
        .source = expectedSource,
        .dataOffsetBytes = merged.dataOffsetBytes,
        .dataByteSize = merged.dataByteSize,
        .scalarDataByteSize = merged.scalarDataByteSize,
        .cachedRecordSize = merged.geometryBytesPerPoint,
        .hasSourceRgb = merged.hasSourceRgb,
        .hasNormals = merged.hasNormals,
        .scalarFields = std::move(merged.scalarFields),
        .blocks = std::move(merged.blocks),
    };
    const auto temporaryIndexPath =
        temporaryDirectory / "index.json";
    {
        std::ofstream indexOutput{
            temporaryIndexPath,
            std::ios::binary | std::ios::trunc};
        if (!indexOutput.is_open()) {
            std::filesystem::remove(finalDataPath, fileError);
            std::filesystem::remove(finalScalarPath, fileError);
            return {
                .errorMessage =
                    "Unable to create the adaptive-HQ sidecar index.",
            };
        }
        indexOutput << CacheIndexToJson(builtIndex).dump(2) << '\n';
        indexOutput.flush();
        if (!indexOutput.good()) {
            std::filesystem::remove(finalDataPath, fileError);
            std::filesystem::remove(finalScalarPath, fileError);
            return {
                .errorMessage =
                    "Unable to write the adaptive-HQ sidecar index.",
            };
        }
    }

    // Publish the sidecar last. Until this rename succeeds, readers continue
    // to see the prior complete cache generation (if one exists).
    fileError.clear();
    std::filesystem::rename(temporaryIndexPath, indexPath, fileError);
    if (fileError) {
        // Windows does not replace an existing destination with rename. The
        // macOS production path uses the atomic branch above; this fallback
        // is retained for cross-platform development builds.
        fileError.clear();
        std::filesystem::remove(indexPath, fileError);
        fileError.clear();
        std::filesystem::rename(temporaryIndexPath, indexPath, fileError);
    }
    if (fileError) {
        std::filesystem::remove(finalDataPath, fileError);
        std::filesystem::remove(finalScalarPath, fileError);
        return {
            .errorMessage =
                "Unable to publish the adaptive-HQ sidecar index.",
        };
    }

    // Old generations are local derived data. Remove them only after the new
    // sidecar is visible, and never consider files outside this cache entry.
    for (std::filesystem::directory_iterator iterator{
             entryRoot,
             fileError};
         !fileError && iterator != std::filesystem::directory_iterator{};
         iterator.increment(fileError)) {
        const auto& path = iterator->path();
        const auto filename = path.filename().string();
        const bool generatedPayload =
            filename.rfind("points-", 0U) == 0U ||
            filename.rfind("geometry-", 0U) == 0U ||
            filename.rfind("scalars-", 0U) == 0U;
        if (path == finalDataPath || path == finalScalarPath ||
            !iterator->is_regular_file(fileError) ||
            !generatedPayload) {
            continue;
        }
        std::error_code ignored;
        std::filesystem::remove(path, ignored);
    }

    const auto validated = LoadValidatedCacheIndex(
        entryRoot,
        indexPath,
        expectedSource);
    if (!validated.has_value()) {
        return {
            .errorMessage =
                "The newly written adaptive-HQ cache did not pass validation.",
        };
    }
    if (progress) {
        progress(1.0F);
    }
    return {
        .index = *validated,
        .success = true,
        .built = true,
    };
}

std::vector<std::uint32_t> SelectAdaptiveHqCacheBlocks(
    const AdaptiveHqCacheIndex& index,
    const AdaptiveHqBoundsPredicate& includeBounds) {
    std::vector<std::uint32_t> selected;
    selected.reserve(index.blocks.size());
    for (const auto& block : index.blocks) {
        if (!includeBounds || includeBounds(block.bounds)) {
            selected.push_back(block.blockIndex);
        }
    }
    return selected;
}

std::vector<PointCloudSourceRange> AdaptiveHqCacheBlockRanges(
    const AdaptiveHqCacheIndex& index,
    std::span<const std::uint32_t> blockIndices) {
    std::vector<std::uint32_t> sorted(
        blockIndices.begin(),
        blockIndices.end());
    std::sort(sorted.begin(), sorted.end());
    sorted.erase(std::unique(sorted.begin(), sorted.end()), sorted.end());
    std::vector<PointCloudSourceRange> ranges;
    ranges.reserve(sorted.size());
    for (const auto blockIndex : sorted) {
        if (blockIndex >= index.blocks.size()) {
            continue;
        }
        const auto& block = index.blocks[blockIndex];
        if (!ranges.empty()) {
            auto& tail = ranges.back();
            if (tail.firstPoint + tail.pointCount == block.firstPoint) {
                tail.pointCount += block.pointCount;
                continue;
            }
        }
        ranges.push_back({
            .firstPoint = block.firstPoint,
            .pointCount = block.pointCount,
        });
    }
    return ranges;
}

AdaptiveHqSourceIdentityResult InspectAdaptiveHqSource(
    const std::filesystem::path& sourcePath) {
    const auto normalizedPath = NormalizedAbsolutePath(sourcePath);
    std::error_code error;
    if (!std::filesystem::is_regular_file(normalizedPath, error) || error) {
        return {.errorMessage = "The adaptive-HQ source file is unavailable."};
    }
    const auto byteSize = std::filesystem::file_size(normalizedPath, error);
    if (error) {
        return {.errorMessage = "Unable to read the adaptive-HQ source size."};
    }
    const auto writeTimeTicks = FileWriteTimeTicks(normalizedPath, &error);
    if (error) {
        return {
            .errorMessage =
                "Unable to read the adaptive-HQ source modification time.",
        };
    }
    const auto layout = BuildSourceLayout(normalizedPath);
    if (!layout.success) {
        return {.errorMessage = layout.errorMessage};
    }
    std::string fingerprintError;
    const auto contentFingerprint = SampledContentFingerprint(
        normalizedPath,
        byteSize,
        &fingerprintError);
    if (!contentFingerprint.has_value()) {
        return {.errorMessage = fingerprintError};
    }
    return {
        .identity = {
            .path = normalizedPath,
            .byteSize = byteSize,
            .writeTimeTicks = writeTimeTicks,
            .schemaFingerprint = SchemaFingerprint(layout.layout),
            .contentFingerprint = *contentFingerprint,
            .pointCount = layout.layout.header.vertexCount,
            .recordSize = layout.layout.recordSize,
        },
        .success = true,
    };
}

namespace {

nlohmann::json SourceIdentityToJson(
    const AdaptiveHqSourceIdentity& source) {
    return {
        {"path", source.path.generic_string()},
        {"byte_size", source.byteSize},
        {"write_time_ticks", source.writeTimeTicks},
        {"schema_fingerprint", source.schemaFingerprint},
        {"content_fingerprint", source.contentFingerprint},
        {"point_count", source.pointCount},
        {"record_size", source.recordSize},
    };
}

std::optional<AdaptiveHqSourceIdentity> SourceIdentityFromJson(
    const nlohmann::json& value) {
    try {
        AdaptiveHqSourceIdentity source;
        source.path = value.at("path").get<std::string>();
        source.path = NormalizedAbsolutePath(source.path);
        source.byteSize = value.at("byte_size").get<std::uintmax_t>();
        source.writeTimeTicks =
            value.at("write_time_ticks").get<std::int64_t>();
        source.schemaFingerprint =
            value.at("schema_fingerprint").get<std::string>();
        source.contentFingerprint =
            value.at("content_fingerprint").get<std::string>();
        source.pointCount = value.at("point_count").get<std::uint64_t>();
        source.recordSize = value.at("record_size").get<std::uint32_t>();
        return source;
    } catch (const std::exception&) {
        return std::nullopt;
    }
}

nlohmann::json BoundsToJson(const Bounds3f& bounds) {
    return {
        {"minimum", {bounds.minimum.x, bounds.minimum.y, bounds.minimum.z}},
        {"maximum", {bounds.maximum.x, bounds.maximum.y, bounds.maximum.z}},
    };
}

std::optional<Bounds3f> BoundsFromJson(const nlohmann::json& value) {
    try {
        const auto& minimum = value.at("minimum");
        const auto& maximum = value.at("maximum");
        if (!minimum.is_array() || minimum.size() != 3U ||
            !maximum.is_array() || maximum.size() != 3U) {
            return std::nullopt;
        }
        Bounds3f bounds{
            .minimum = {
                minimum.at(0U).get<float>(),
                minimum.at(1U).get<float>(),
                minimum.at(2U).get<float>(),
            },
            .maximum = {
                maximum.at(0U).get<float>(),
                maximum.at(1U).get<float>(),
                maximum.at(2U).get<float>(),
            },
            .valid = true,
        };
        if (!IsFinite(bounds.minimum) || !IsFinite(bounds.maximum) ||
            bounds.minimum.x > bounds.maximum.x ||
            bounds.minimum.y > bounds.maximum.y ||
            bounds.minimum.z > bounds.maximum.z) {
            return std::nullopt;
        }
        return bounds;
    } catch (const std::exception&) {
        return std::nullopt;
    }
}

std::optional<AdaptiveHqCacheIndex> LoadValidatedCacheIndex(
    const std::filesystem::path& cacheRoot,
    const std::filesystem::path& indexPath,
    const AdaptiveHqSourceIdentity& expectedSource) {
    std::ifstream input{indexPath, std::ios::binary};
    if (!input.is_open()) {
        return std::nullopt;
    }
    nlohmann::json document;
    try {
        input >> document;
        if (document.at("schema_version").get<std::uint32_t>() !=
                kAdaptiveHqCacheSchemaVersion ||
            document.at("algorithm").get<std::string>() !=
                kAdaptiveHqCacheAlgorithmId) {
            return std::nullopt;
        }
    } catch (const std::exception&) {
        return std::nullopt;
    }

    const auto source = SourceIdentityFromJson(document.value(
        "source",
        nlohmann::json::object()));
    if (!source.has_value() || *source != expectedSource) {
        return std::nullopt;
    }

    AdaptiveHqCacheIndex index;
    index.cacheRoot = cacheRoot;
    index.indexPath = indexPath;
    index.source = *source;
    try {
        index.cacheFingerprint =
            document.at("cache_fingerprint").get<std::string>();
        const auto dataFile =
            document.at("data_file").get<std::string>();
        const auto scalarDataFile =
            document.at("scalar_data_file").get<std::string>();
        const std::filesystem::path dataFilePath{dataFile};
        const std::filesystem::path scalarDataFilePath{scalarDataFile};
        if (dataFilePath.empty() || dataFilePath.has_parent_path() ||
            dataFilePath.filename() != dataFilePath ||
            scalarDataFilePath.empty() ||
            scalarDataFilePath.has_parent_path() ||
            scalarDataFilePath.filename() != scalarDataFilePath) {
            return std::nullopt;
        }
        index.dataPath = cacheRoot / dataFilePath;
        index.scalarDataPath = cacheRoot / scalarDataFilePath;
        index.dataOffsetBytes =
            document.at("data_offset_bytes").get<std::uint64_t>();
        index.dataByteSize =
            document.at("data_byte_size").get<std::uint64_t>();
        index.scalarDataByteSize =
            document.at("scalar_data_byte_size").get<std::uint64_t>();
        index.cachedRecordSize =
            document.at("cached_record_size").get<std::uint32_t>();
        index.hasSourceRgb = document.at("has_source_rgb").get<bool>();
        index.hasNormals = document.at("has_normals").get<bool>();
        const auto& scalarFields = document.at("scalar_fields");
        if (!scalarFields.is_array()) {
            return std::nullopt;
        }
        index.scalarFields.reserve(scalarFields.size());
        for (const auto& value : scalarFields) {
            AdaptiveHqCacheScalarField field;
            field.name = value.at("name").get<std::string>();
            field.sourceIndex =
                value.at("source_index").get<std::uint32_t>();
            field.stats.name = field.name;
            field.stats.sourceIndex =
                static_cast<std::int32_t>(field.sourceIndex);
            field.stats.minimum = value.at("minimum").get<float>();
            field.stats.maximum = value.at("maximum").get<float>();
            field.stats.count = value.at("count").get<std::uint64_t>();
            field.stats.valid = value.at("valid").get<bool>();
            index.scalarFields.push_back(std::move(field));
        }
        const auto& blocks = document.at("blocks");
        if (!blocks.is_array()) {
            return std::nullopt;
        }
        index.blocks.reserve(blocks.size());
        for (const auto& value : blocks) {
            const auto bounds = BoundsFromJson(value.at("bounds"));
            if (!bounds.has_value()) {
                return std::nullopt;
            }
            index.blocks.push_back({
                .blockIndex =
                    value.at("block_index").get<std::uint32_t>(),
                .firstPoint =
                    value.at("first_point").get<std::uint64_t>(),
                .pointCount =
                    value.at("point_count").get<std::uint64_t>(),
                .fileOffsetBytes =
                    value.at("file_offset_bytes").get<std::uint64_t>(),
                .byteSize = value.at("byte_size").get<std::uint64_t>(),
                .scalarFileOffsetBytes = value.at("scalar_file_offset_bytes")
                                             .get<std::uint64_t>(),
                .scalarByteSize =
                    value.at("scalar_byte_size").get<std::uint64_t>(),
                .bounds = *bounds,
            });
        }
    } catch (const std::exception&) {
        return std::nullopt;
    }

    const auto expectedGeometryBytesPerPoint =
        static_cast<std::uint32_t>(sizeof(Float3) + sizeof(std::uint32_t) +
                                   sizeof(std::uint32_t) +
                                   (index.hasNormals ? sizeof(Float3) : 0U));
    if (index.cacheFingerprint != IdentityFingerprint(expectedSource) ||
        index.dataOffsetBytes != 0U ||
        index.cachedRecordSize != expectedGeometryBytesPerPoint ||
        index.dataByteSize !=
            expectedSource.pointCount * index.cachedRecordSize ||
        index.scalarDataByteSize !=
            expectedSource.pointCount * index.scalarFields.size() *
                sizeof(float)) {
        return std::nullopt;
    }
    for (std::size_t field = 0U; field < index.scalarFields.size(); ++field) {
        const auto& scalar = index.scalarFields[field];
        if (scalar.name.empty() || scalar.sourceIndex != field ||
            scalar.stats.sourceIndex != static_cast<std::int32_t>(field) ||
            (scalar.stats.valid &&
             (!std::isfinite(scalar.stats.minimum) ||
              !std::isfinite(scalar.stats.maximum) ||
              scalar.stats.minimum > scalar.stats.maximum))) {
            return std::nullopt;
        }
    }
    std::uint64_t expectedFirstPoint = 0U;
    std::uint64_t expectedOffset = index.dataOffsetBytes;
    std::uint64_t expectedScalarOffset = 0U;
    for (std::size_t blockIndex = 0U;
         blockIndex < index.blocks.size();
         ++blockIndex) {
        const auto& block = index.blocks[blockIndex];
        if (block.blockIndex != blockIndex || block.pointCount == 0U ||
            block.firstPoint != expectedFirstPoint ||
            block.fileOffsetBytes != expectedOffset ||
            block.byteSize != block.pointCount * index.cachedRecordSize ||
            block.scalarFileOffsetBytes != expectedScalarOffset ||
            block.scalarByteSize !=
                block.pointCount * index.scalarFields.size() *
                    sizeof(float) ||
            block.byteSize > kAdaptiveHqCacheMaximumBlockBytes +
                                 index.cachedRecordSize) {
            return std::nullopt;
        }
        expectedFirstPoint += block.pointCount;
        expectedOffset += block.byteSize;
        expectedScalarOffset += block.scalarByteSize;
    }
    if (expectedFirstPoint != expectedSource.pointCount ||
        expectedOffset != index.dataOffsetBytes + index.dataByteSize ||
        expectedScalarOffset != index.scalarDataByteSize) {
        return std::nullopt;
    }
    std::error_code fileError;
    const auto dataSize = std::filesystem::file_size(index.dataPath, fileError);
    if (fileError || dataSize != expectedOffset) {
        return std::nullopt;
    }
    fileError.clear();
    const auto scalarDataSize =
        std::filesystem::file_size(index.scalarDataPath, fileError);
    if (fileError || scalarDataSize != expectedScalarOffset) {
        return std::nullopt;
    }
    return index;
}

std::optional<Bounds3f> ScanSourceBounds(
    const AdaptiveHqSourceIdentity& source,
    const SourceLayout& layout,
    std::stop_token stopToken,
    const AdaptiveHqCacheProgress& progress,
    std::string* errorMessage) {
    std::ifstream input{source.path, std::ios::binary};
    if (!input.is_open()) {
        *errorMessage = "Unable to open the source while building its cache.";
        return std::nullopt;
    }
    input.seekg(
        static_cast<std::streamoff>(layout.header.dataOffsetBytes),
        std::ios::beg);
    constexpr std::size_t targetBytes = 8U * 1024U * 1024U;
    const auto pointsPerChunk = std::max<std::size_t>(
        1U,
        targetBytes / layout.recordSize);
    std::vector<std::byte> bytes(pointsPerChunk * layout.recordSize);
    Bounds3f bounds;
    std::uint64_t scanned = 0U;
    while (scanned < source.pointCount) {
        if (stopToken.stop_requested()) {
            return std::nullopt;
        }
        const auto pointCount = static_cast<std::size_t>(
            std::min<std::uint64_t>(
                pointsPerChunk,
                source.pointCount - scanned));
        const auto byteCount = pointCount * layout.recordSize;
        input.read(
            reinterpret_cast<char*>(bytes.data()),
            static_cast<std::streamsize>(byteCount));
        if (input.gcount() != static_cast<std::streamsize>(byteCount)) {
            *errorMessage =
                "Unexpected EOF while scanning adaptive-HQ source bounds.";
            return std::nullopt;
        }
        for (std::size_t pointIndex = 0U;
             pointIndex < pointCount;
             ++pointIndex) {
            const auto point = ReadPosition(
                bytes.data() + pointIndex * layout.recordSize,
                layout);
            if (IsFinite(point)) {
                bounds.Expand(point);
            }
        }
        scanned += pointCount;
        if (progress) {
            progress(
                source.pointCount == 0U
                    ? 0.2F
                    : 0.2F * static_cast<float>(scanned) /
                          static_cast<float>(source.pointCount));
        }
    }
    if (!bounds.valid) {
        *errorMessage = "The adaptive-HQ source contains no finite points.";
        return std::nullopt;
    }
    return bounds;
}

std::uint64_t ExpandMortonBits(std::uint32_t value) {
    std::uint64_t expanded = value & 0x1fffffU;
    expanded = (expanded | (expanded << 32U)) & 0x1f00000000ffffULL;
    expanded = (expanded | (expanded << 16U)) & 0x1f0000ff0000ffULL;
    expanded = (expanded | (expanded << 8U)) & 0x100f00f00f00f00fULL;
    expanded = (expanded | (expanded << 4U)) & 0x10c30c30c30c30c3ULL;
    expanded = (expanded | (expanded << 2U)) & 0x1249249249249249ULL;
    return expanded;
}

std::uint32_t QuantizeMortonAxis(float value, float minimum, float maximum) {
    constexpr std::uint32_t maximumQuantized = (1U << 21U) - 1U;
    const auto extent = static_cast<double>(maximum) - minimum;
    if (!(extent > 0.0) || !std::isfinite(value)) {
        return 0U;
    }
    const auto normalized = std::clamp(
        (static_cast<double>(value) - minimum) / extent,
        0.0,
        1.0);
    return static_cast<std::uint32_t>(
        std::llround(normalized * maximumQuantized));
}

std::uint64_t MortonKey(const Float3& point, const Bounds3f& bounds) {
    const auto x = QuantizeMortonAxis(
        point.x,
        bounds.minimum.x,
        bounds.maximum.x);
    const auto y = QuantizeMortonAxis(
        point.y,
        bounds.minimum.y,
        bounds.maximum.y);
    const auto z = QuantizeMortonAxis(
        point.z,
        bounds.minimum.z,
        bounds.maximum.z);
    return ExpandMortonBits(x) | (ExpandMortonBits(y) << 1U) |
           (ExpandMortonBits(z) << 2U);
}

void StoreLittleEndianUint64(std::byte* output, std::uint64_t value) {
    if constexpr (std::endian::native == std::endian::big) {
        auto* bytes = reinterpret_cast<std::byte*>(&value);
        std::reverse(bytes, bytes + sizeof(value));
    }
    std::memcpy(output, &value, sizeof(value));
}

void StoreLittleEndianUint32(std::byte* output, std::uint32_t value) {
    if constexpr (std::endian::native == std::endian::big) {
        value = ((value & 0x000000ffU) << 24U) |
                ((value & 0x0000ff00U) << 8U) |
                ((value & 0x00ff0000U) >> 8U) |
                ((value & 0xff000000U) >> 24U);
    }
    std::memcpy(output, &value, sizeof(value));
}

std::uint64_t EntryMortonKey(const std::byte* entry) {
    return ReadLittleEndian<std::uint64_t>(entry);
}

std::uint32_t EntrySourceIndex(const std::byte* entry) {
    return ReadLittleEndian<std::uint32_t>(entry + sizeof(std::uint64_t));
}

SortedRunsResult CreateSortedRuns(
    const AdaptiveHqSourceIdentity& source,
    const SourceLayout& layout,
    const Bounds3f& sourceBounds,
    const std::filesystem::path& temporaryDirectory,
    std::stop_token stopToken,
    const AdaptiveHqCacheProgress& progress) {
    std::ifstream input{source.path, std::ios::binary};
    if (!input.is_open()) {
        return {.errorMessage = "Unable to open the source for Morton sorting."};
    }
    input.seekg(
        static_cast<std::streamoff>(layout.header.dataOffsetBytes),
        std::ios::beg);
    if (!input.good()) {
        return {.errorMessage = "Unable to seek to the source PLY payload."};
    }

    constexpr std::size_t prefixBytes =
        sizeof(std::uint64_t) + sizeof(std::uint32_t);
    const auto entrySize =
        prefixBytes + static_cast<std::size_t>(layout.recordSize);
    const auto pointsPerRun = std::max<std::size_t>(
        1U,
        kSortRunTargetBytes / entrySize);
    constexpr std::size_t sourceChunkTargetBytes = 8U * 1024U * 1024U;
    const auto sourceChunkPoints = std::max<std::size_t>(
        1U,
        sourceChunkTargetBytes / layout.recordSize);
    std::vector<std::byte> sourceBytes(
        sourceChunkPoints * layout.recordSize);

    SortedRunsResult result;
    std::uint64_t nextSourcePoint = 0U;
    while (nextSourcePoint < source.pointCount) {
        if (stopToken.stop_requested()) {
            result.cancelled = true;
            return result;
        }
        const auto entriesThisRun = static_cast<std::size_t>(
            std::min<std::uint64_t>(
                pointsPerRun,
                source.pointCount - nextSourcePoint));
        std::vector<std::byte> entries(entriesThisRun * entrySize);
        std::vector<std::uint32_t> order(entriesThisRun);
        std::size_t runPointOffset = 0U;
        while (runPointOffset < entriesThisRun) {
            if (stopToken.stop_requested()) {
                result.cancelled = true;
                return result;
            }
            const auto chunkPointCount = std::min(
                sourceChunkPoints,
                entriesThisRun - runPointOffset);
            const auto chunkByteCount =
                chunkPointCount * layout.recordSize;
            input.read(
                reinterpret_cast<char*>(sourceBytes.data()),
                static_cast<std::streamsize>(chunkByteCount));
            if (input.gcount() !=
                static_cast<std::streamsize>(chunkByteCount)) {
                result.errorMessage =
                    "Unexpected EOF while creating adaptive-HQ sort runs.";
                return result;
            }
            for (std::size_t localIndex = 0U;
                 localIndex < chunkPointCount;
                 ++localIndex) {
                const auto runIndex = runPointOffset + localIndex;
                const auto sourceIndex = static_cast<std::uint32_t>(
                    nextSourcePoint + runIndex);
                auto* entry = entries.data() + runIndex * entrySize;
                const auto* sourceRecord =
                    sourceBytes.data() + localIndex * layout.recordSize;
                StoreLittleEndianUint64(
                    entry,
                    MortonKey(ReadPosition(sourceRecord, layout), sourceBounds));
                StoreLittleEndianUint32(
                    entry + sizeof(std::uint64_t),
                    sourceIndex);
                std::memcpy(
                    entry + prefixBytes,
                    sourceRecord,
                    layout.recordSize);
                order[runIndex] = static_cast<std::uint32_t>(runIndex);
            }
            runPointOffset += chunkPointCount;
        }
        std::sort(
            order.begin(),
            order.end(),
            [&](std::uint32_t leftIndex, std::uint32_t rightIndex) {
                const auto* left = entries.data() + leftIndex * entrySize;
                const auto* right = entries.data() + rightIndex * entrySize;
                const auto leftKey = EntryMortonKey(left);
                const auto rightKey = EntryMortonKey(right);
                if (leftKey != rightKey) {
                    return leftKey < rightKey;
                }
                return EntrySourceIndex(left) < EntrySourceIndex(right);
            });

        const auto runPath = temporaryDirectory /
            ("run-" + std::to_string(result.runs.size()) + ".bin");
        std::ofstream runOutput{runPath, std::ios::binary | std::ios::trunc};
        if (!runOutput.is_open()) {
            result.errorMessage =
                "Unable to create an adaptive-HQ temporary sort run.";
            return result;
        }
        for (const auto entryIndex : order) {
            runOutput.write(
                reinterpret_cast<const char*>(
                    entries.data() + entryIndex * entrySize),
                static_cast<std::streamsize>(entrySize));
        }
        runOutput.flush();
        if (!runOutput.good()) {
            result.errorMessage =
                "Unable to write an adaptive-HQ temporary sort run.";
            return result;
        }
        result.runs.push_back({
            .path = runPath,
            .entryCount = entriesThisRun,
        });
        nextSourcePoint += entriesThisRun;
        if (progress) {
            progress(
                0.2F + 0.45F *
                    static_cast<float>(nextSourcePoint) /
                    static_cast<float>(source.pointCount));
        }
    }
    result.success = true;
    return result;
}

class SortedRunReader {
  public:
    SortedRunReader(const SortedRun& run, std::size_t entrySize)
        : input_(run.path, std::ios::binary),
          entrySize_(entrySize),
          remainingEntries_(run.entryCount) {
        const auto entriesPerBuffer = std::max<std::size_t>(
            1U,
            kMergeReadBufferBytes / entrySize_);
        buffer_.resize(entriesPerBuffer * entrySize_);
    }

    [[nodiscard]] bool IsOpen() const { return input_.is_open(); }
    [[nodiscard]] const std::byte* Current() const { return current_; }

    bool Advance() {
        if (nextBufferedEntry_ >= bufferedEntryCount_) {
            if (remainingEntries_ == 0U) {
                current_ = nullptr;
                return false;
            }
            const auto capacity = buffer_.size() / entrySize_;
            bufferedEntryCount_ = static_cast<std::size_t>(
                std::min<std::uint64_t>(capacity, remainingEntries_));
            const auto byteCount = bufferedEntryCount_ * entrySize_;
            input_.read(
                reinterpret_cast<char*>(buffer_.data()),
                static_cast<std::streamsize>(byteCount));
            if (input_.gcount() != static_cast<std::streamsize>(byteCount)) {
                failed_ = true;
                current_ = nullptr;
                return false;
            }
            remainingEntries_ -= bufferedEntryCount_;
            nextBufferedEntry_ = 0U;
        }
        current_ =
            buffer_.data() + nextBufferedEntry_ * entrySize_;
        ++nextBufferedEntry_;
        return true;
    }

    [[nodiscard]] bool Failed() const { return failed_; }

  private:
    std::ifstream input_;
    std::size_t entrySize_ = 0U;
    std::uint64_t remainingEntries_ = 0U;
    std::vector<std::byte> buffer_;
    std::size_t bufferedEntryCount_ = 0U;
    std::size_t nextBufferedEntry_ = 0U;
    const std::byte* current_ = nullptr;
    bool failed_ = false;
};

struct MergeNode {
    std::uint64_t mortonKey = 0U;
    std::uint32_t sourceIndex = 0U;
    std::size_t runIndex = 0U;
};

struct MergeNodeGreater {
    bool operator()(const MergeNode& left, const MergeNode& right) const {
        if (left.mortonKey != right.mortonKey) {
            return left.mortonKey > right.mortonKey;
        }
        return left.sourceIndex > right.sourceIndex;
    }
};

MergeRunsResult MergeSortedRuns(
    const AdaptiveHqSourceIdentity& source,
    const SourceLayout& layout,
    std::span<const SortedRun> runs,
    const std::filesystem::path& geometryOutputPath,
    const std::filesystem::path& scalarOutputPath,
    std::stop_token stopToken,
    const AdaptiveHqCacheProgress& progress) {
    constexpr std::size_t prefixBytes =
        sizeof(std::uint64_t) + sizeof(std::uint32_t);
    const auto entrySize =
        prefixBytes + static_cast<std::size_t>(layout.recordSize);
    static_assert(sizeof(Float3) == sizeof(float) * 3U);
    const auto geometryBytesPerPoint = static_cast<std::uint32_t>(
        sizeof(Float3) + sizeof(std::uint32_t) + sizeof(std::uint32_t) +
        (layout.hasNormals ? sizeof(Float3) : 0U));
    const auto totalPayloadBytes =
        source.pointCount * geometryBytesPerPoint;
    const auto blockCount = std::max<std::uint64_t>(
        1U,
        (totalPayloadBytes + kAdaptiveHqCacheTargetBlockBytes - 1U) /
            kAdaptiveHqCacheTargetBlockBytes);
    // Balance the final generation so the tail is not a tiny block. Except
    // for a source whose whole payload is under 1 MiB, blocks remain close
    // to 4 MiB and always below the 8 MiB hard limit.
    const auto pointsPerBlock = std::max<std::uint64_t>(
        1U,
        (source.pointCount + blockCount - 1U) / blockCount);

    std::ofstream geometryOutput{
        geometryOutputPath,
        std::ios::binary | std::ios::trunc};
    std::ofstream scalarOutput{
        scalarOutputPath,
        std::ios::binary | std::ios::trunc};
    if (!geometryOutput.is_open() || !scalarOutput.is_open()) {
        return {
            .errorMessage =
                "Unable to create the adaptive-HQ column cache payloads.",
        };
    }

    MergeRunsResult result;
    result.dataOffsetBytes = 0U;
    result.geometryBytesPerPoint = geometryBytesPerPoint;
    result.hasSourceRgb = layout.hasSourceRgb;
    result.hasNormals = layout.hasNormals;
    result.scalarFields.reserve(layout.scalarFields.size());
    for (const auto& sourceField : layout.scalarFields) {
        AdaptiveHqCacheScalarField field;
        field.name = sourceField.name;
        field.sourceIndex = sourceField.sourceIndex;
        field.stats.name = sourceField.name;
        field.stats.sourceIndex =
            static_cast<std::int32_t>(sourceField.sourceIndex);
        result.scalarFields.push_back(std::move(field));
    }
    std::vector<std::unique_ptr<SortedRunReader>> readers;
    readers.reserve(runs.size());
    std::priority_queue<
        MergeNode,
        std::vector<MergeNode>,
        MergeNodeGreater>
        pending;
    for (std::size_t runIndex = 0U; runIndex < runs.size(); ++runIndex) {
        auto reader = std::make_unique<SortedRunReader>(runs[runIndex], entrySize);
        if (!reader->IsOpen() || !reader->Advance()) {
            return {
                .errorMessage =
                    "Unable to open an adaptive-HQ temporary sort run.",
            };
        }
        const auto* entry = reader->Current();
        pending.push({
            .mortonKey = EntryMortonKey(entry),
            .sourceIndex = EntrySourceIndex(entry),
            .runIndex = runIndex,
        });
        readers.push_back(std::move(reader));
    }

    std::vector<Float3> positions;
    std::vector<std::uint32_t> packedColors;
    std::vector<Float3> normals;
    std::vector<std::uint32_t> sourceIndices;
    std::vector<std::vector<float>> scalarColumns(
        layout.scalarFields.size());
    positions.reserve(static_cast<std::size_t>(pointsPerBlock));
    packedColors.reserve(static_cast<std::size_t>(pointsPerBlock));
    sourceIndices.reserve(static_cast<std::size_t>(pointsPerBlock));
    if (layout.hasNormals) {
        normals.reserve(static_cast<std::size_t>(pointsPerBlock));
    }
    for (auto& column : scalarColumns) {
        column.reserve(static_cast<std::size_t>(pointsPerBlock));
    }

    AdaptiveHqCacheBlock currentBlock{};
    std::uint64_t writtenPoints = 0U;
    const auto startBlock = [&]() {
        currentBlock = {
            .blockIndex = static_cast<std::uint32_t>(result.blocks.size()),
            .firstPoint = writtenPoints,
            .pointCount = 0U,
            .fileOffsetBytes = result.dataByteSize,
            .byteSize = 0U,
            .scalarFileOffsetBytes = result.scalarDataByteSize,
            .scalarByteSize = 0U,
            .bounds = {},
        };
    };
    const auto writeVector = [](std::ostream& output, const auto& values) {
        if (values.empty()) {
            return true;
        }
        output.write(
            reinterpret_cast<const char*>(values.data()),
            static_cast<std::streamsize>(
                values.size() * sizeof(values.front())));
        return output.good();
    };
    const auto finishBlock = [&]() {
        if (currentBlock.pointCount == 0U) {
            return true;
        }
        if (!currentBlock.bounds.valid) {
            result.errorMessage =
                "An adaptive-HQ cache block contains no finite points.";
            return false;
        }
        currentBlock.byteSize =
            currentBlock.pointCount * geometryBytesPerPoint;
        currentBlock.scalarByteSize =
            currentBlock.pointCount * layout.scalarFields.size() *
            sizeof(float);
        if (!writeVector(geometryOutput, positions) ||
            !writeVector(geometryOutput, packedColors) ||
            (layout.hasNormals && !writeVector(geometryOutput, normals)) ||
            !writeVector(geometryOutput, sourceIndices)) {
            result.errorMessage =
                "Unable to write the adaptive-HQ geometry columns.";
            return false;
        }
        for (const auto& column : scalarColumns) {
            if (!writeVector(scalarOutput, column)) {
                result.errorMessage =
                    "Unable to write the adaptive-HQ scalar columns.";
                return false;
            }
        }
        result.dataByteSize += currentBlock.byteSize;
        result.scalarDataByteSize += currentBlock.scalarByteSize;
        result.blocks.push_back(currentBlock);
        positions.clear();
        packedColors.clear();
        normals.clear();
        sourceIndices.clear();
        for (auto& column : scalarColumns) {
            column.clear();
        }
        return true;
    };
    if (source.pointCount > 0U) {
        startBlock();
    }
    while (!pending.empty()) {
        if (stopToken.stop_requested()) {
            result.cancelled = true;
            return result;
        }
        const auto node = pending.top();
        pending.pop();
        auto& reader = *readers[node.runIndex];
        const auto* entry = reader.Current();
        const auto* sourceRecord = entry + prefixBytes;
        const auto point = ReadPosition(sourceRecord, layout);
        positions.push_back(point);
        packedColors.push_back(ReadPackedColor(sourceRecord, layout));
        if (layout.hasNormals) {
            normals.push_back(ReadNormal(sourceRecord, layout));
        }
        sourceIndices.push_back(node.sourceIndex);
        for (std::size_t field = 0U;
             field < layout.scalarFields.size();
             ++field) {
            const auto& sourceField = layout.scalarFields[field];
            const auto value = static_cast<float>(ReadScalar(
                sourceRecord + sourceField.offset,
                sourceField.type));
            scalarColumns[field].push_back(value);
            result.scalarFields[field].stats.Include(value);
        }
        if (IsFinite(point)) {
            currentBlock.bounds.Expand(point);
        }
        ++currentBlock.pointCount;
        ++writtenPoints;
        if (currentBlock.pointCount == pointsPerBlock) {
            if (!finishBlock()) {
                return result;
            }
            if (writtenPoints < source.pointCount) {
                startBlock();
            }
        }

        if (reader.Advance()) {
            const auto* nextEntry = reader.Current();
            pending.push({
                .mortonKey = EntryMortonKey(nextEntry),
                .sourceIndex = EntrySourceIndex(nextEntry),
                .runIndex = node.runIndex,
            });
        } else if (reader.Failed()) {
            result.errorMessage =
                "Unable to read an adaptive-HQ temporary sort run.";
            return result;
        }
        if (progress &&
            (writtenPoints == source.pointCount ||
             (writtenPoints % 262144U) == 0U)) {
            progress(
                0.65F + 0.33F *
                    static_cast<float>(writtenPoints) /
                    static_cast<float>(source.pointCount));
        }
    }
    if (currentBlock.pointCount > 0U &&
        (result.blocks.empty() ||
         result.blocks.back().firstPoint != currentBlock.firstPoint)) {
        if (!finishBlock()) {
            return result;
        }
    }
    geometryOutput.flush();
    scalarOutput.flush();
    if (!geometryOutput.good() || !scalarOutput.good() ||
        writtenPoints != source.pointCount ||
        result.dataByteSize != source.pointCount * geometryBytesPerPoint ||
        result.scalarDataByteSize !=
            source.pointCount * layout.scalarFields.size() * sizeof(float)) {
        result.errorMessage =
            "The adaptive-HQ cache payload did not finish writing.";
        return result;
    }
    result.success = true;
    return result;
}

nlohmann::json CacheIndexToJson(const AdaptiveHqCacheIndex& index) {
    nlohmann::json blocks = nlohmann::json::array();
    for (const auto& block : index.blocks) {
        blocks.push_back({
            {"block_index", block.blockIndex},
            {"first_point", block.firstPoint},
            {"point_count", block.pointCount},
            {"file_offset_bytes", block.fileOffsetBytes},
            {"byte_size", block.byteSize},
            {"scalar_file_offset_bytes", block.scalarFileOffsetBytes},
            {"scalar_byte_size", block.scalarByteSize},
            {"bounds", BoundsToJson(block.bounds)},
        });
    }
    nlohmann::json scalarFields = nlohmann::json::array();
    for (const auto& field : index.scalarFields) {
        scalarFields.push_back({
            {"name", field.name},
            {"source_index", field.sourceIndex},
            {"minimum", field.stats.minimum},
            {"maximum", field.stats.maximum},
            {"count", field.stats.count},
            {"valid", field.stats.valid},
        });
    }
    return {
        {"schema_version", kAdaptiveHqCacheSchemaVersion},
        {"algorithm", kAdaptiveHqCacheAlgorithmId},
        {"cache_fingerprint", index.cacheFingerprint},
        {"source", SourceIdentityToJson(index.source)},
        {"data_file", index.dataPath.filename().generic_string()},
        {"scalar_data_file",
         index.scalarDataPath.filename().generic_string()},
        {"data_offset_bytes", index.dataOffsetBytes},
        {"data_byte_size", index.dataByteSize},
        {"scalar_data_byte_size", index.scalarDataByteSize},
        {"cached_record_size", index.cachedRecordSize},
        {"has_source_rgb", index.hasSourceRgb},
        {"has_normals", index.hasNormals},
        {"scalar_fields", std::move(scalarFields)},
        {"target_block_bytes", kAdaptiveHqCacheTargetBlockBytes},
        {"blocks", std::move(blocks)},
    };
}

}  // namespace

std::uint64_t AdaptiveHqFieldFilterFingerprint(
    const PointCloudScalarFieldFilter& filter) {
    auto hash = kFingerprintOffset;
    FingerprintValue(&hash, filter.mode);
    for (const auto& name : filter.names) {
        FingerprintString(&hash, name);
    }
    constexpr std::uint32_t section = 0xffffffffU;
    FingerprintValue(&hash, section);
    for (const auto sourceIndex : filter.sourceIndices) {
        FingerprintValue(&hash, sourceIndex);
    }
    FingerprintValue(&hash, section);
    for (const auto& pattern : filter.containsPatterns) {
        FingerprintString(&hash, pattern);
    }
    return hash;
}

PointCloudSubsetLoadResult LoadAdaptiveHqCacheBlock(
    const AdaptiveHqCacheIndex& index,
    std::uint32_t blockIndex,
    const PointCloudScalarFieldFilter& fieldFilter,
    std::stop_token stopToken,
    const PointCloudLoadProgress& progress) {
    if (blockIndex >= index.blocks.size()) {
        return {.errorMessage = "Adaptive-HQ block index is out of range."};
    }
    const auto& block = index.blocks[blockIndex];
    PointCloudSubsetLoadResult loaded;
    loaded.sourcePointCount = index.source.pointCount;
    if (stopToken.stop_requested()) {
        loaded.cancelled = true;
        return loaded;
    }
    if (progress) {
        progress(0U, block.pointCount);
    }
    const auto pointCount = static_cast<std::size_t>(block.pointCount);
    auto& cloud = loaded.cloud;
    cloud.sourcePath = index.source.path;
    cloud.layerName = index.source.path.stem().string();
    cloud.hasSourceRgb = index.hasSourceRgb;
    cloud.hasNormals = index.hasNormals;
    cloud.availableScalarFields.reserve(index.scalarFields.size());
    std::vector<std::size_t> selectedFields;
    selectedFields.reserve(index.scalarFields.size());
    for (std::size_t field = 0U; field < index.scalarFields.size(); ++field) {
        const auto& cached = index.scalarFields[field];
        cloud.availableScalarFields.push_back({
            .name = cached.name,
            .sourceIndex = cached.sourceIndex,
        });
        if (PointCloudScalarFieldFilterSelects(
                fieldFilter,
                cached.name,
                cached.sourceIndex)) {
            selectedFields.push_back(field);
            auto stats = cached.stats;
            stats.name = cached.name;
            stats.sourceIndex =
                static_cast<std::int32_t>(cached.sourceIndex);
            cloud.scalarFields.push_back(std::move(stats));
        }
    }
    try {
        cloud.positions.resize(pointCount);
        cloud.packedColors.resize(pointCount);
        loaded.sourcePointIndices.resize(pointCount);
        if (cloud.hasNormals) {
            cloud.normals.resize(pointCount);
        }
        cloud.scalarFieldValues.resize(
            pointCount * selectedFields.size());
    } catch (const std::exception& error) {
        loaded.errorMessage =
            std::string{"Adaptive-HQ block allocation failed: "} +
            error.what();
        return loaded;
    }

    const auto readExact = [](std::istream& input,
                              void* destination,
                              std::size_t byteCount) {
        if (byteCount == 0U) {
            return true;
        }
        input.read(
            static_cast<char*>(destination),
            static_cast<std::streamsize>(byteCount));
        return input.gcount() == static_cast<std::streamsize>(byteCount);
    };
    std::ifstream geometry{index.dataPath, std::ios::binary};
    if (!geometry.is_open()) {
        loaded.errorMessage =
            "Unable to open the adaptive-HQ geometry cache.";
        return loaded;
    }
    geometry.seekg(
        static_cast<std::streamoff>(block.fileOffsetBytes),
        std::ios::beg);
    if (!geometry.good() ||
        !readExact(
            geometry,
            cloud.positions.data(),
            pointCount * sizeof(Float3)) ||
        !readExact(
            geometry,
            cloud.packedColors.data(),
            pointCount * sizeof(std::uint32_t)) ||
        (cloud.hasNormals &&
         !readExact(
             geometry,
             cloud.normals.data(),
             pointCount * sizeof(Float3))) ||
        !readExact(
            geometry,
            loaded.sourcePointIndices.data(),
            pointCount * sizeof(std::uint32_t))) {
        loaded.errorMessage =
            "Adaptive-HQ geometry block read was incomplete.";
        return loaded;
    }

    if (!selectedFields.empty()) {
        std::ifstream scalars{index.scalarDataPath, std::ios::binary};
        if (!scalars.is_open()) {
            loaded.errorMessage =
                "Unable to open the adaptive-HQ scalar cache.";
            return loaded;
        }
        const auto fieldByteSize = pointCount * sizeof(float);
        for (std::size_t resident = 0U;
             resident < selectedFields.size();
             ++resident) {
            if (stopToken.stop_requested()) {
                loaded.cancelled = true;
                return loaded;
            }
            const auto sourceField = selectedFields[resident];
            const auto fieldOffset = block.scalarFileOffsetBytes +
                sourceField * fieldByteSize;
            scalars.clear();
            scalars.seekg(
                static_cast<std::streamoff>(fieldOffset),
                std::ios::beg);
            auto* destination = cloud.scalarFieldValues.data() +
                resident * pointCount;
            if (!scalars.good() ||
                !readExact(
                    scalars,
                    destination,
                    fieldByteSize)) {
                loaded.errorMessage =
                    "Adaptive-HQ scalar column read was incomplete.";
                return loaded;
            }
        }
    }
    cloud.bounds = block.bounds;
    if (cloud.bounds.valid) {
        cloud.focusPoint = {
            (cloud.bounds.minimum.x + cloud.bounds.maximum.x) * 0.5F,
            (cloud.bounds.minimum.y + cloud.bounds.maximum.y) * 0.5F,
            (cloud.bounds.minimum.z + cloud.bounds.maximum.z) * 0.5F,
        };
        cloud.hasFocusPoint = true;
    }
    loaded.includedPointCount = block.pointCount;
    loaded.success = true;
    if (progress) {
        progress(block.pointCount, block.pointCount);
    }
    return loaded;
}

std::vector<AdaptiveHqResidentBlock::MicroBlock>
BuildAdaptiveHqMicroBlocks(
    std::span<const Float3> positions,
    std::uint32_t targetPointCount) {
    std::vector<AdaptiveHqResidentBlock::MicroBlock> blocks;
    if (positions.empty()) {
        return blocks;
    }
    const auto pointsPerBlock = std::max(1U, targetPointCount);
    blocks.reserve(
        (positions.size() + pointsPerBlock - 1U) / pointsPerBlock);
    for (std::size_t first = 0U; first < positions.size();) {
        const auto count = std::min<std::size_t>(
            pointsPerBlock,
            positions.size() - first);
        AdaptiveHqResidentBlock::MicroBlock block;
        block.firstPoint = static_cast<std::uint32_t>(first);
        block.pointCount = static_cast<std::uint32_t>(count);
        for (std::size_t local = 0U; local < count; ++local) {
            block.bounds.Expand(positions[first + local]);
        }
        blocks.push_back(block);
        first += count;
    }
    return blocks;
}

namespace {

template <typename Value>
void ReorderBySourceIndex(
    std::vector<Value>* values,
    std::span<const std::uint32_t> order) {
    if (values == nullptr || values->empty()) {
        return;
    }
    std::vector<Value> reordered;
    reordered.reserve(order.size());
    for (const auto oldIndex : order) {
        reordered.push_back(std::move((*values)[oldIndex]));
    }
    *values = std::move(reordered);
}

bool ScalarLayoutsMatch(
    const LoadedPointCloud& left,
    const LoadedPointCloud& right) {
    if (left.scalarFields.size() != right.scalarFields.size() ||
        left.availableScalarFields.size() !=
            right.availableScalarFields.size()) {
        return false;
    }
    for (std::size_t field = 0U;
         field < left.scalarFields.size();
         ++field) {
        if (left.scalarFields[field].name != right.scalarFields[field].name ||
            left.scalarFields[field].sourceIndex !=
                right.scalarFields[field].sourceIndex) {
            return false;
        }
    }
    for (std::size_t field = 0U;
         field < left.availableScalarFields.size();
         ++field) {
        if (left.availableScalarFields[field].name !=
                right.availableScalarFields[field].name ||
            left.availableScalarFields[field].sourceIndex !=
                right.availableScalarFields[field].sourceIndex) {
            return false;
        }
    }
    return true;
}

}  // namespace

PointCloudSubsetLoadResult AssembleAdaptiveHqCacheSubset(
    const AdaptiveHqCacheIndex& index,
    std::span<const std::uint32_t> activeBlockIndices,
    std::span<const AdaptiveHqResidentBlock> residentBlocks,
    const PointCloudSubsetPredicate& includePoint,
    const PointCloudGridDecimation& gridDecimation,
    std::stop_token stopToken,
    bool restoreSourceOrder,
    const AdaptiveHqBoundsPredicate& includeMicroBlockBounds) {
    PointCloudSubsetLoadResult result;
    result.sourcePointCount = index.source.pointCount;
    result.cloud.sourcePath = index.source.path;
    if (activeBlockIndices.empty()) {
        result.success = true;
        return result;
    }
    std::vector<const AdaptiveHqResidentBlock*> residentByIndex(
        index.blocks.size(),
        nullptr);
    for (const auto& resident : residentBlocks) {
        if (resident.blockIndex < residentByIndex.size() &&
            resident.points != nullptr && resident.points->success) {
            residentByIndex[resident.blockIndex] = &resident;
        }
    }
    const AdaptiveHqResidentBlock* firstResident = nullptr;
    std::uint64_t maximumPoints = 0U;
    for (const auto blockIndex : activeBlockIndices) {
        if (blockIndex >= residentByIndex.size() ||
            residentByIndex[blockIndex] == nullptr) {
            result.errorMessage =
                "An active adaptive-HQ block is not resident.";
            return result;
        }
        if (firstResident == nullptr) {
            firstResident = residentByIndex[blockIndex];
        }
        maximumPoints += index.blocks[blockIndex].pointCount;
    }
    const auto& reference = firstResident->points->cloud;
    result.cloud.layerName = reference.layerName;
    result.cloud.availableScalarFields = reference.availableScalarFields;
    result.cloud.scalarFields = reference.scalarFields;
    result.cloud.hasSourceRgb = reference.hasSourceRgb;
    result.cloud.hasNormals = reference.hasNormals;
    for (const auto blockIndex : activeBlockIndices) {
        const auto& blockResult = *residentByIndex[blockIndex]->points;
        const auto& blockCloud = blockResult.cloud;
        if (!ScalarLayoutsMatch(reference, blockCloud) ||
            blockCloud.hasNormals != reference.hasNormals ||
            blockCloud.PointCount() !=
                blockResult.sourcePointIndices.size()) {
            result.errorMessage =
                "Adaptive-HQ resident blocks have incompatible layouts.";
            return result;
        }
    }

    // The on-disk blocks are intentionally large enough for efficient seeks.
    // Their retained in-memory micro-block bounds let live aHQ accept small,
    // contiguous Morton ranges conservatively instead of evaluating the
    // frustum predicate for millions of individual points. Exact per-point
    // classification remains the compatibility fallback for callers without
    // micro-blocks (and for fixed-HQ tests).
    struct IncludedRange {
        std::uint32_t firstPoint = 0U;
        std::uint32_t pointCount = 0U;
    };
    const auto hardwareThreads = std::max(1U, std::thread::hardware_concurrency());
    const auto workerCount = static_cast<std::size_t>(std::clamp<std::uint64_t>(
        maximumPoints / 250'000U,
        1U,
        std::min<std::uint64_t>(
            8U,
            std::min<std::uint64_t>(
                hardwareThreads,
                activeBlockIndices.size()))));
    std::vector<std::vector<IncludedRange>> includedByBlock(
        activeBlockIndices.size());
    std::atomic<std::size_t> nextBlock{0U};
    std::atomic_bool cancelled{false};
    std::atomic_bool failed{false};
    std::exception_ptr workerError;
    std::mutex workerErrorMutex;
    const auto classifyBlocks = [&]() {
        try {
            while (!failed.load(std::memory_order_acquire) &&
                   !cancelled.load(std::memory_order_acquire)) {
                const auto slot = nextBlock.fetch_add(
                    1U,
                    std::memory_order_relaxed);
                if (slot >= activeBlockIndices.size()) {
                    return;
                }
                const auto blockIndex = activeBlockIndices[slot];
                const auto& blockCloud =
                    residentByIndex[blockIndex]->points->cloud;
                auto& included = includedByBlock[slot];
                const auto appendRange = [&](std::uint32_t first,
                                             std::uint32_t count) {
                    if (count == 0U) {
                        return;
                    }
                    if (!included.empty() &&
                        included.back().firstPoint +
                                included.back().pointCount == first) {
                        included.back().pointCount += count;
                    } else {
                        included.push_back({first, count});
                    }
                };
                const auto& microBlocks =
                    residentByIndex[blockIndex]->microBlocks;
                bool validMicroBlocks = includeMicroBlockBounds &&
                    microBlocks != nullptr && !microBlocks->empty();
                std::uint32_t expectedFirst = 0U;
                if (validMicroBlocks) {
                    for (const auto& micro : *microBlocks) {
                        if (micro.pointCount == 0U ||
                            micro.firstPoint != expectedFirst ||
                            static_cast<std::uint64_t>(micro.firstPoint) +
                                    micro.pointCount >
                                blockCloud.PointCount() ||
                            !micro.bounds.valid) {
                            validMicroBlocks = false;
                            break;
                        }
                        expectedFirst += micro.pointCount;
                    }
                    validMicroBlocks = validMicroBlocks &&
                        expectedFirst == blockCloud.PointCount();
                }
                if (validMicroBlocks) {
                    included.reserve(microBlocks->size());
                    for (std::size_t microIndex = 0U;
                         microIndex < microBlocks->size();
                         ++microIndex) {
                        if ((microIndex & 63U) == 0U &&
                            stopToken.stop_requested()) {
                            cancelled.store(true, std::memory_order_release);
                            return;
                        }
                        const auto& micro = (*microBlocks)[microIndex];
                        if (includeMicroBlockBounds(micro.bounds)) {
                            appendRange(micro.firstPoint, micro.pointCount);
                        }
                    }
                    continue;
                }
                included.reserve(blockCloud.PointCount() / 32U + 1U);
                for (std::uint32_t pointIndex = 0U;
                     pointIndex < blockCloud.PointCount();
                     ++pointIndex) {
                    if ((pointIndex & 4095U) == 0U &&
                        stopToken.stop_requested()) {
                        cancelled.store(true, std::memory_order_release);
                        return;
                    }
                    if (!includePoint ||
                        includePoint(blockCloud.positions[pointIndex])) {
                        appendRange(pointIndex, 1U);
                    }
                }
            }
        } catch (...) {
            {
                std::scoped_lock lock(workerErrorMutex);
                if (workerError == nullptr) {
                    workerError = std::current_exception();
                }
            }
            failed.store(true, std::memory_order_release);
        }
    };
    {
        std::vector<std::jthread> workers;
        workers.reserve(workerCount - 1U);
        for (std::size_t worker = 1U; worker < workerCount; ++worker) {
            workers.emplace_back(classifyBlocks);
        }
        classifyBlocks();
    }
    if (cancelled.load(std::memory_order_acquire) ||
        stopToken.stop_requested()) {
        result.cancelled = true;
        return result;
    }
    if (failed.load(std::memory_order_acquire)) {
        try {
            if (workerError != nullptr) {
                std::rethrow_exception(workerError);
            }
        } catch (const std::exception& error) {
            result.errorMessage =
                std::string{"Adaptive-HQ patch classification failed: "} +
                error.what();
        } catch (...) {
            result.errorMessage =
                "Adaptive-HQ patch classification failed unexpectedly.";
        }
        return result;
    }

    std::vector<std::size_t> blockOffsets(activeBlockIndices.size() + 1U, 0U);
    for (std::size_t slot = 0U; slot < includedByBlock.size(); ++slot) {
        std::size_t blockPointCount = 0U;
        for (const auto& range : includedByBlock[slot]) {
            blockPointCount += range.pointCount;
        }
        blockOffsets[slot + 1U] = blockOffsets[slot] + blockPointCount;
    }
    const auto includedCount = blockOffsets.back();
    try {
        result.cloud.positions.resize(includedCount);
        result.cloud.packedColors.resize(includedCount);
        result.sourcePointIndices.resize(includedCount);
        if (result.cloud.hasNormals) {
            result.cloud.normals.resize(includedCount);
        }
        result.cloud.scalarFieldValues.resize(
            includedCount * reference.scalarFields.size());
    } catch (const std::exception& error) {
        result.errorMessage =
            std::string{"Adaptive-HQ patch allocation failed: "} +
            error.what();
        return result;
    }

    nextBlock.store(0U, std::memory_order_release);
    const auto copyBlocks = [&]() {
        while (!cancelled.load(std::memory_order_acquire)) {
            const auto slot = nextBlock.fetch_add(
                1U,
                std::memory_order_relaxed);
            if (slot >= activeBlockIndices.size()) {
                return;
            }
            const auto blockIndex = activeBlockIndices[slot];
            const auto& blockResult = *residentByIndex[blockIndex]->points;
            const auto& blockCloud = blockResult.cloud;
            const auto& included = includedByBlock[slot];
            std::size_t destination = blockOffsets[slot];
            for (const auto& range : included) {
                if (stopToken.stop_requested()) {
                    cancelled.store(true, std::memory_order_release);
                    return;
                }
                const auto source = static_cast<std::size_t>(
                    range.firstPoint);
                const auto count = static_cast<std::size_t>(
                    range.pointCount);
                std::copy_n(
                    blockCloud.positions.begin() + source,
                    count,
                    result.cloud.positions.begin() + destination);
                std::copy_n(
                    blockCloud.packedColors.begin() + source,
                    count,
                    result.cloud.packedColors.begin() + destination);
                std::copy_n(
                    blockResult.sourcePointIndices.begin() + source,
                    count,
                    result.sourcePointIndices.begin() + destination);
                if (result.cloud.hasNormals) {
                    std::copy_n(
                        blockCloud.normals.begin() + source,
                        count,
                        result.cloud.normals.begin() + destination);
                }
                for (std::size_t field = 0U;
                     field < reference.scalarFields.size();
                     ++field) {
                    std::copy_n(
                        blockCloud.scalarFieldValues.begin() +
                            static_cast<std::ptrdiff_t>(
                                blockCloud.ScalarFieldValueIndex(
                                    field,
                                    source)),
                        count,
                        result.cloud.scalarFieldValues.begin() +
                            static_cast<std::ptrdiff_t>(
                                field * includedCount + destination));
                }
                destination += count;
            }
        }
    };
    {
        std::vector<std::jthread> workers;
        workers.reserve(workerCount - 1U);
        for (std::size_t worker = 1U; worker < workerCount; ++worker) {
            workers.emplace_back(copyBlocks);
        }
        copyBlocks();
    }
    if (cancelled.load(std::memory_order_acquire) ||
        stopToken.stop_requested()) {
        result.cancelled = true;
        return result;
    }
    result.includedPointCount = result.cloud.PointCount();

    if (restoreSourceOrder) {
        std::vector<std::uint32_t> order(result.cloud.PointCount());
        std::iota(order.begin(), order.end(), 0U);
        std::sort(
            order.begin(),
            order.end(),
            [&](std::uint32_t left, std::uint32_t right) {
                return result.sourcePointIndices[left] <
                    result.sourcePointIndices[right];
            });
        ReorderBySourceIndex(&result.cloud.positions, order);
        ReorderBySourceIndex(&result.cloud.packedColors, order);
        if (result.cloud.hasNormals) {
            ReorderBySourceIndex(&result.cloud.normals, order);
        }
        ReorderBySourceIndex(&result.sourcePointIndices, order);
        const auto fieldCount = result.cloud.scalarFields.size();
        if (fieldCount > 0U) {
            std::vector<float> reorderedScalarValues;
            reorderedScalarValues.resize(
                result.cloud.PointCount() * fieldCount);
            for (std::size_t field = 0U; field < fieldCount; ++field) {
                for (std::size_t destination = 0U;
                     destination < order.size();
                     ++destination) {
                    reorderedScalarValues[
                        field * order.size() + destination] =
                        result.cloud.scalarFieldValues[
                            field * order.size() + order[destination]];
                }
            }
            result.cloud.scalarFieldValues =
                std::move(reorderedScalarValues);
        }
    }

    result.cloud.bounds = {};
    for (const auto& point : result.cloud.positions) {
        result.cloud.bounds.Expand(point);
    }
    if (result.cloud.bounds.valid) {
        result.cloud.focusPoint = {
            (result.cloud.bounds.minimum.x + result.cloud.bounds.maximum.x) *
                0.5F,
            (result.cloud.bounds.minimum.y + result.cloud.bounds.maximum.y) *
                0.5F,
            (result.cloud.bounds.minimum.z + result.cloud.bounds.maximum.z) *
                0.5F,
        };
        result.cloud.hasFocusPoint = true;
    }
    for (std::size_t field = 0U;
         field < result.cloud.scalarFields.size();
         ++field) {
        auto& stats = result.cloud.scalarFields[field];
        stats.minimum = 0.0F;
        stats.maximum = 0.0F;
        stats.count = 0U;
        stats.valid = false;
        const auto valuesBegin =
            result.cloud.scalarFieldValues.begin() +
            static_cast<std::ptrdiff_t>(
                field * result.cloud.PointCount());
        const auto valuesEnd = valuesBegin +
            static_cast<std::ptrdiff_t>(result.cloud.PointCount());
        for (auto value = valuesBegin; value != valuesEnd; ++value) {
            stats.Include(*value);
        }
    }
    if (!DecimatePointCloudSubsetByGrid(
            &result,
            gridDecimation,
            stopToken)) {
        result.cancelled = true;
        return result;
    }
    result.success = true;
    return result;
}

}  // namespace invisible_places::io
