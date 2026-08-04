#include "io/PointCloudFieldCache.hpp"

#include <nlohmann/json.hpp>

#include <cctype>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <system_error>

namespace invisible_places::io {

namespace {

using nlohmann::json;

constexpr std::string_view kManifestFileName = "manifest.json";
constexpr std::string_view kGeometryFileName = "geometry.bin";

std::uint64_t Fnv1aHash(std::string_view text) {
    std::uint64_t hash = 1469598103934665603ULL;
    for (const char character : text) {
        hash ^= static_cast<std::uint64_t>(
            static_cast<unsigned char>(character));
        hash *= 1099511628211ULL;
    }
    return hash;
}

std::string SanitizedFieldFileToken(std::string_view name) {
    std::string token;
    token.reserve(name.size());
    for (const char character : name) {
        const auto byte = static_cast<unsigned char>(character);
        token.push_back(
            std::isalnum(byte) != 0 ? static_cast<char>(std::tolower(byte))
                                    : '-');
    }
    return token;
}

std::filesystem::path FieldFilePath(
    const std::filesystem::path& cacheDirectory,
    std::uint32_t sourceIndex,
    std::string_view fieldName) {
    return cacheDirectory /
           ("field_" + std::to_string(sourceIndex) + "_" +
            SanitizedFieldFileToken(fieldName) + ".bin");
}

struct SourceIdentity {
    std::uint64_t sizeBytes = 0;
    std::int64_t mtimeNanoseconds = 0;
};

std::optional<SourceIdentity> StatSourceIdentity(
    const std::filesystem::path& sourcePath) {
    std::error_code error;
    const auto size = std::filesystem::file_size(sourcePath, error);
    if (error) {
        return std::nullopt;
    }
    const auto mtime = std::filesystem::last_write_time(sourcePath, error);
    if (error) {
        return std::nullopt;
    }
    return SourceIdentity{
        .sizeBytes = static_cast<std::uint64_t>(size),
        .mtimeNanoseconds =
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                mtime.time_since_epoch())
                .count(),
    };
}

// All cache artefacts are written to a sibling temp path and renamed into
// place, so readers only ever observe complete files.
bool WriteFileAtomically(
    const std::filesystem::path& path,
    const void* data,
    std::size_t sizeBytes) {
    const auto tempPath =
        path.parent_path() / (path.filename().string() + ".tmp");
    {
        std::ofstream output{tempPath, std::ios::binary | std::ios::trunc};
        if (!output.is_open()) {
            return false;
        }
        output.write(
            static_cast<const char*>(data),
            static_cast<std::streamsize>(sizeBytes));
        if (!output.good()) {
            return false;
        }
    }
    std::error_code error;
    std::filesystem::rename(tempPath, path, error);
    if (error) {
        std::filesystem::remove(tempPath, error);
        return false;
    }
    return true;
}

bool ReadExactFile(
    const std::filesystem::path& path,
    void* data,
    std::size_t sizeBytes) {
    std::error_code error;
    const auto onDisk = std::filesystem::file_size(path, error);
    if (error || onDisk != sizeBytes) {
        return false;
    }
    std::ifstream input{path, std::ios::binary};
    if (!input.is_open()) {
        return false;
    }
    input.read(
        static_cast<char*>(data),
        static_cast<std::streamsize>(sizeBytes));
    return input.gcount() == static_cast<std::streamsize>(sizeBytes);
}

json StatsToJson(const ScalarFieldStats& stats) {
    return json{
        {"minimum", stats.minimum},
        {"maximum", stats.maximum},
        {"count", stats.count},
        {"valid", stats.valid},
    };
}

void StatsFromJson(const json& statsJson, ScalarFieldStats* stats) {
    stats->minimum = statsJson.value("minimum", 0.0F);
    stats->maximum = statsJson.value("maximum", 0.0F);
    stats->count = statsJson.value("count", std::uint64_t{0});
    stats->valid = statsJson.value("valid", false);
}

bool WriteManifest(
    const std::filesystem::path& cacheDirectory,
    const PointCloudFieldCacheManifest& manifest) {
    json fieldsJson = json::array();
    for (const auto& field : manifest.fields) {
        json fieldJson{
            {"name", field.name},
            {"source_index", field.sourceIndex},
        };
        fieldJson["stats"] = StatsToJson(field.stats);
        fieldsJson.push_back(std::move(fieldJson));
    }
    const json manifestJson{
        {"schema", manifest.schemaVersion},
        {"source_size_bytes", manifest.sourceSizeBytes},
        {"source_mtime_ns", manifest.sourceMtimeNanoseconds},
        {"point_count", manifest.pointCount},
        {"has_source_rgb", manifest.hasSourceRgb},
        {"has_normals", manifest.hasNormals},
        {"has_focus_point", manifest.hasFocusPoint},
        {"focus_point",
         json::array({manifest.focusPoint.x, manifest.focusPoint.y, manifest.focusPoint.z})},
        {"bounds_valid", manifest.bounds.valid},
        {"bounds_minimum",
         json::array({manifest.bounds.minimum.x, manifest.bounds.minimum.y, manifest.bounds.minimum.z})},
        {"bounds_maximum",
         json::array({manifest.bounds.maximum.x, manifest.bounds.maximum.y, manifest.bounds.maximum.z})},
        {"fields", std::move(fieldsJson)},
    };
    const auto serialized = manifestJson.dump(2);
    return WriteFileAtomically(
        cacheDirectory / kManifestFileName,
        serialized.data(),
        serialized.size());
}

std::optional<PointCloudFieldCacheManifest> ReadManifest(
    const std::filesystem::path& cacheDirectory) {
    std::ifstream input{cacheDirectory / kManifestFileName};
    if (!input.is_open()) {
        return std::nullopt;
    }
    json manifestJson;
    try {
        input >> manifestJson;
    } catch (const json::exception&) {
        return std::nullopt;
    }
    if (!manifestJson.is_object() ||
        manifestJson.value("schema", 0U) !=
            kPointCloudFieldCacheSchemaVersion) {
        return std::nullopt;
    }
    PointCloudFieldCacheManifest manifest;
    try {
        manifest.sourceSizeBytes =
            manifestJson.value("source_size_bytes", std::uint64_t{0});
        manifest.sourceMtimeNanoseconds =
            manifestJson.value("source_mtime_ns", std::int64_t{0});
        manifest.pointCount = manifestJson.value("point_count", std::uint64_t{0});
        manifest.hasSourceRgb = manifestJson.value("has_source_rgb", false);
        manifest.hasNormals = manifestJson.value("has_normals", false);
        manifest.hasFocusPoint = manifestJson.value("has_focus_point", false);
        if (const auto focusIt = manifestJson.find("focus_point");
            focusIt != manifestJson.end() && focusIt->is_array() &&
            focusIt->size() == 3U) {
            manifest.focusPoint = {
                focusIt->at(0).get<float>(),
                focusIt->at(1).get<float>(),
                focusIt->at(2).get<float>(),
            };
        }
        manifest.bounds.valid = manifestJson.value("bounds_valid", false);
        const auto readVec = [&](const char* key, Float3* out) {
            if (const auto it = manifestJson.find(key);
                it != manifestJson.end() && it->is_array() && it->size() == 3U) {
                *out = {
                    it->at(0).get<float>(),
                    it->at(1).get<float>(),
                    it->at(2).get<float>(),
                };
            }
        };
        readVec("bounds_minimum", &manifest.bounds.minimum);
        readVec("bounds_maximum", &manifest.bounds.maximum);
        if (const auto fieldsIt = manifestJson.find("fields");
            fieldsIt != manifestJson.end() && fieldsIt->is_array()) {
            manifest.fields.reserve(fieldsIt->size());
            for (const auto& fieldJson : *fieldsIt) {
                PointCloudFieldCacheFieldEntry entry;
                entry.name = fieldJson.value("name", std::string{});
                entry.sourceIndex = fieldJson.value("source_index", 0U);
                if (const auto statsIt = fieldJson.find("stats");
                    statsIt != fieldJson.end() && statsIt->is_object()) {
                    StatsFromJson(*statsIt, &entry.stats);
                }
                entry.stats.name = entry.name;
                entry.stats.sourceIndex =
                    static_cast<std::int32_t>(entry.sourceIndex);
                if (!entry.name.empty()) {
                    manifest.fields.push_back(std::move(entry));
                }
            }
        }
    } catch (const json::exception&) {
        return std::nullopt;
    }
    return manifest;
}

std::size_t GeometryFileSizeBytes(const PointCloudFieldCacheManifest& manifest) {
    const auto pointCount = static_cast<std::size_t>(manifest.pointCount);
    return pointCount * sizeof(Float3) + pointCount * sizeof(std::uint32_t) +
           (manifest.hasNormals ? pointCount * sizeof(Float3) : 0U);
}

bool WriteGeometryFile(
    const std::filesystem::path& cacheDirectory,
    const LoadedPointCloud& cloud) {
    std::vector<std::byte> blob;
    const auto pointCount = cloud.PointCount();
    blob.resize(
        pointCount * sizeof(Float3) + pointCount * sizeof(std::uint32_t) +
        (cloud.hasNormals ? pointCount * sizeof(Float3) : 0U));
    auto* cursor = blob.data();
    std::memcpy(cursor, cloud.positions.data(), pointCount * sizeof(Float3));
    cursor += pointCount * sizeof(Float3);
    std::memcpy(
        cursor,
        cloud.packedColors.data(),
        pointCount * sizeof(std::uint32_t));
    cursor += pointCount * sizeof(std::uint32_t);
    if (cloud.hasNormals) {
        std::memcpy(cursor, cloud.normals.data(), pointCount * sizeof(Float3));
    }
    return WriteFileAtomically(
        cacheDirectory / kGeometryFileName,
        blob.data(),
        blob.size());
}

bool ReadGeometryFile(
    const std::filesystem::path& cacheDirectory,
    const PointCloudFieldCacheManifest& manifest,
    LoadedPointCloud* cloud) {
    const auto pointCount = static_cast<std::size_t>(manifest.pointCount);
    std::vector<std::byte> blob(GeometryFileSizeBytes(manifest));
    if (!ReadExactFile(
            cacheDirectory / kGeometryFileName,
            blob.data(),
            blob.size())) {
        return false;
    }
    try {
        cloud->positions.resize(pointCount);
        cloud->packedColors.resize(pointCount);
        if (manifest.hasNormals) {
            cloud->normals.resize(pointCount);
        }
    } catch (const std::exception&) {
        return false;
    }
    const auto* cursor = blob.data();
    std::memcpy(cloud->positions.data(), cursor, pointCount * sizeof(Float3));
    cursor += pointCount * sizeof(Float3);
    std::memcpy(
        cloud->packedColors.data(),
        cursor,
        pointCount * sizeof(std::uint32_t));
    cursor += pointCount * sizeof(std::uint32_t);
    if (manifest.hasNormals) {
        std::memcpy(cloud->normals.data(), cursor, pointCount * sizeof(Float3));
    }
    return true;
}

// Removes every cache artefact so a stale cache can never mix old field
// values with a rewritten source.
void ClearCacheDirectory(const std::filesystem::path& cacheDirectory) {
    std::error_code error;
    if (!std::filesystem::exists(cacheDirectory, error)) {
        return;
    }
    for (const auto& entry :
         std::filesystem::directory_iterator{cacheDirectory, error}) {
        std::error_code removeError;
        std::filesystem::remove(entry.path(), removeError);
    }
}

// Best-effort write of every cache artefact for a freshly PLY-loaded cloud.
void WriteCacheFromLoadedCloud(
    const std::filesystem::path& sourcePath,
    const LoadedPointCloud& cloud) {
    const auto identity = StatSourceIdentity(sourcePath);
    if (!identity.has_value()) {
        return;
    }
    const auto cacheDirectory = PointCloudFieldCacheDirectory(sourcePath);
    std::error_code error;
    std::filesystem::create_directories(cacheDirectory, error);
    if (error) {
        return;
    }

    PointCloudFieldCacheManifest manifest;
    const auto existing = ReadManifest(cacheDirectory);
    const bool existingMatches =
        existing.has_value() &&
        existing->sourceSizeBytes == identity->sizeBytes &&
        existing->sourceMtimeNanoseconds == identity->mtimeNanoseconds &&
        existing->pointCount == cloud.PointCount();
    if (!existingMatches) {
        ClearCacheDirectory(cacheDirectory);
    } else {
        manifest = existing.value();
    }
    manifest.schemaVersion = kPointCloudFieldCacheSchemaVersion;
    manifest.sourceSizeBytes = identity->sizeBytes;
    manifest.sourceMtimeNanoseconds = identity->mtimeNanoseconds;
    manifest.pointCount = cloud.PointCount();
    manifest.hasSourceRgb = cloud.hasSourceRgb;
    manifest.hasNormals = cloud.hasNormals;
    manifest.hasFocusPoint = cloud.hasFocusPoint;
    manifest.focusPoint = cloud.focusPoint;
    manifest.bounds = cloud.bounds;

    // The manifest lists every on-disk field; stats become valid once the
    // field is materialised.
    std::vector<PointCloudFieldCacheFieldEntry> fields;
    fields.reserve(cloud.availableScalarFields.size());
    for (const auto& available : cloud.availableScalarFields) {
        PointCloudFieldCacheFieldEntry entry;
        entry.name = available.name;
        entry.sourceIndex = available.sourceIndex;
        entry.stats.name = available.name;
        entry.stats.sourceIndex =
            static_cast<std::int32_t>(available.sourceIndex);
        for (const auto& existingEntry : manifest.fields) {
            if (existingEntry.sourceIndex == available.sourceIndex &&
                existingEntry.name == available.name) {
                entry.stats = existingEntry.stats;
                entry.stats.name = available.name;
                entry.stats.sourceIndex =
                    static_cast<std::int32_t>(available.sourceIndex);
                break;
            }
        }
        fields.push_back(std::move(entry));
    }
    manifest.fields = std::move(fields);

    std::error_code existsError;
    if (!std::filesystem::exists(
            cacheDirectory / kGeometryFileName,
            existsError) &&
        !WriteGeometryFile(cacheDirectory, cloud)) {
        std::cerr << "Point-cloud field cache: geometry write failed for "
                  << sourcePath.filename().string() << std::endl;
        return;
    }

    const auto pointCount = cloud.PointCount();
    for (std::size_t slot = 0; slot < cloud.scalarFields.size(); ++slot) {
        const auto& stats = cloud.scalarFields[slot];
        if (stats.sourceIndex < 0) {
            continue;  // generated fields never cache
        }
        for (auto& entry : manifest.fields) {
            if (entry.sourceIndex ==
                    static_cast<std::uint32_t>(stats.sourceIndex) &&
                entry.name == stats.name) {
                entry.stats = stats;
                break;
            }
        }
        const auto fieldPath = FieldFilePath(
            cacheDirectory,
            static_cast<std::uint32_t>(stats.sourceIndex),
            stats.name);
        std::error_code fieldExistsError;
        if (std::filesystem::exists(fieldPath, fieldExistsError)) {
            continue;
        }
        if (!WriteFileAtomically(
                fieldPath,
                cloud.scalarFieldValues.data() +
                    cloud.ScalarFieldValueIndex(slot, 0U),
                pointCount * sizeof(float))) {
            std::cerr << "Point-cloud field cache: field write failed for '"
                      << stats.name << "'." << std::endl;
        }
    }

    if (!WriteManifest(cacheDirectory, manifest)) {
        std::cerr << "Point-cloud field cache: manifest write failed for "
                  << sourcePath.filename().string() << std::endl;
    }
}

}  // namespace

std::filesystem::path PointCloudFieldCacheDirectory(
    const std::filesystem::path& sourcePath) {
    const auto normalized = sourcePath.lexically_normal();
    const auto hash = Fnv1aHash(normalized.generic_string());
    char hashText[17];
    std::snprintf(hashText, sizeof(hashText), "%016llx",
                  static_cast<unsigned long long>(hash));
    return normalized.parent_path() / ".invisible_places" / "cache" /
           "fields" / (normalized.stem().string() + "-" + hashText);
}

std::optional<PointCloudFieldCacheManifest>
LoadValidPointCloudFieldCacheManifest(
    const std::filesystem::path& sourcePath) {
    const auto identity = StatSourceIdentity(sourcePath);
    if (!identity.has_value()) {
        return std::nullopt;
    }
    auto manifest = ReadManifest(PointCloudFieldCacheDirectory(sourcePath));
    if (!manifest.has_value() ||
        manifest->sourceSizeBytes != identity->sizeBytes ||
        manifest->sourceMtimeNanoseconds != identity->mtimeNanoseconds) {
        return std::nullopt;
    }
    return manifest;
}

PointCloudLoadResult LoadPointCloudWithFieldCache(
    const std::filesystem::path& sourcePath,
    const PointCloudScalarFieldFilter& fieldFilter) {
    const auto manifest = LoadValidPointCloudFieldCacheManifest(sourcePath);
    if (!manifest.has_value()) {
        auto result = LoadPointCloud(sourcePath, fieldFilter);
        if (result.success) {
            WriteCacheFromLoadedCloud(sourcePath, result.cloud);
        }
        return result;
    }

    const auto cacheDirectory = PointCloudFieldCacheDirectory(sourcePath);
    LoadedPointCloud cloud;
    cloud.sourcePath = sourcePath;
    cloud.layerName = sourcePath.stem().string();
    cloud.hasSourceRgb = manifest->hasSourceRgb;
    cloud.hasNormals = manifest->hasNormals;
    cloud.hasFocusPoint = manifest->hasFocusPoint;
    cloud.focusPoint = manifest->focusPoint;
    cloud.bounds = manifest->bounds;
    if (!ReadGeometryFile(cacheDirectory, manifest.value(), &cloud)) {
        // Geometry never cached (e.g. the cache was seeded by a lone
        // on-demand field write) or unreadable: take the PLY path, which
        // also repairs the cache.
        auto result = LoadPointCloud(sourcePath, fieldFilter);
        if (result.success) {
            WriteCacheFromLoadedCloud(sourcePath, result.cloud);
        }
        return result;
    }

    cloud.availableScalarFields.reserve(manifest->fields.size());
    for (const auto& entry : manifest->fields) {
        cloud.availableScalarFields.push_back({
            .name = entry.name,
            .sourceIndex = entry.sourceIndex,
        });
    }

    const auto pointCount = cloud.PointCount();
    bool manifestNeedsUpdate = false;
    auto updatedManifest = manifest.value();
    for (const auto& entry : manifest->fields) {
        if (!PointCloudScalarFieldFilterSelects(
                fieldFilter,
                entry.name,
                entry.sourceIndex)) {
            continue;
        }
        const auto slot = cloud.scalarFields.size();
        try {
            cloud.scalarFields.push_back(entry.stats);
            cloud.scalarFields.back().name = entry.name;
            cloud.scalarFields.back().sourceIndex =
                static_cast<std::int32_t>(entry.sourceIndex);
            cloud.scalarFieldValues.resize(
                (slot + 1U) * pointCount,
                0.0F);
        } catch (const std::exception& error) {
            return {
                .errorMessage =
                    std::string{"Point cloud allocation failed: "} +
                    error.what(),
                .success = false,
            };
        }
        auto* destination =
            cloud.scalarFieldValues.data() +
            cloud.ScalarFieldValueIndex(slot, 0U);
        const auto fieldPath = FieldFilePath(
            cacheDirectory,
            entry.sourceIndex,
            entry.name);
        if (ReadExactFile(
                fieldPath,
                destination,
                pointCount * sizeof(float))) {
            continue;
        }
        // Selected but not cached yet: stream the one field from the PLY
        // and write it through for next time.
        ScalarFieldStats streamedStats;
        streamedStats.name = entry.name;
        streamedStats.sourceIndex =
            static_cast<std::int32_t>(entry.sourceIndex);
        const auto streamResult = StreamPointCloudSelectedValues(
            sourcePath,
            {.source = PointCloudSelectedValueSource::ScalarField,
             .scalarFieldName = entry.name},
            [&](float value, std::uint64_t pointIndex) {
                if (pointIndex < pointCount) {
                    destination[pointIndex] = value;
                    streamedStats.Include(value);
                }
                return true;
            });
        if (!streamResult.success ||
            streamResult.pointCount != pointCount) {
            return {
                .errorMessage =
                    "Field cache stream for '" + entry.name + "' failed: " +
                    (streamResult.errorMessage.empty()
                         ? "unexpected point count"
                         : streamResult.errorMessage),
                .success = false,
            };
        }
        cloud.scalarFields[slot] = streamedStats;
        if (!WriteFileAtomically(
                fieldPath,
                destination,
                pointCount * sizeof(float))) {
            std::cerr << "Point-cloud field cache: field write failed for '"
                      << entry.name << "'." << std::endl;
        }
        for (auto& manifestEntry : updatedManifest.fields) {
            if (manifestEntry.sourceIndex == entry.sourceIndex &&
                manifestEntry.name == entry.name) {
                manifestEntry.stats = streamedStats;
                manifestNeedsUpdate = true;
                break;
            }
        }
    }
    if (manifestNeedsUpdate &&
        !WriteManifest(cacheDirectory, updatedManifest)) {
        std::cerr << "Point-cloud field cache: manifest update failed for "
                  << sourcePath.filename().string() << std::endl;
    }

    return {.cloud = std::move(cloud), .success = true};
}

bool ReadPointCloudCachedField(
    const std::filesystem::path& sourcePath,
    std::string_view fieldName,
    std::span<float> values,
    ScalarFieldStats* stats) {
    const auto manifest = LoadValidPointCloudFieldCacheManifest(sourcePath);
    if (!manifest.has_value() ||
        manifest->pointCount != values.size()) {
        return false;
    }
    for (const auto& entry : manifest->fields) {
        if (entry.name != fieldName) {
            continue;
        }
        if (!ReadExactFile(
                FieldFilePath(
                    PointCloudFieldCacheDirectory(sourcePath),
                    entry.sourceIndex,
                    entry.name),
                values.data(),
                values.size() * sizeof(float))) {
            return false;
        }
        if (stats != nullptr) {
            *stats = entry.stats;
            stats->name = entry.name;
            stats->sourceIndex =
                static_cast<std::int32_t>(entry.sourceIndex);
        }
        return true;
    }
    return false;
}

bool WritePointCloudCachedField(
    const std::filesystem::path& sourcePath,
    const ScalarFieldStats& stats,
    std::span<const float> values) {
    if (stats.sourceIndex < 0 || stats.name.empty() || values.empty()) {
        return false;
    }
    const auto identity = StatSourceIdentity(sourcePath);
    if (!identity.has_value()) {
        return false;
    }
    const auto cacheDirectory = PointCloudFieldCacheDirectory(sourcePath);
    std::error_code error;
    std::filesystem::create_directories(cacheDirectory, error);
    if (error) {
        return false;
    }
    auto manifest = ReadManifest(cacheDirectory);
    const bool manifestMatches =
        manifest.has_value() &&
        manifest->sourceSizeBytes == identity->sizeBytes &&
        manifest->sourceMtimeNanoseconds == identity->mtimeNanoseconds &&
        manifest->pointCount == values.size();
    if (!manifestMatches) {
        ClearCacheDirectory(cacheDirectory);
        manifest = PointCloudFieldCacheManifest{};
        manifest->sourceSizeBytes = identity->sizeBytes;
        manifest->sourceMtimeNanoseconds = identity->mtimeNanoseconds;
        manifest->pointCount = values.size();
    }
    if (!WriteFileAtomically(
            FieldFilePath(
                cacheDirectory,
                static_cast<std::uint32_t>(stats.sourceIndex),
                stats.name),
            values.data(),
            values.size() * sizeof(float))) {
        return false;
    }
    bool found = false;
    for (auto& entry : manifest->fields) {
        if (entry.sourceIndex ==
                static_cast<std::uint32_t>(stats.sourceIndex) &&
            entry.name == stats.name) {
            entry.stats = stats;
            found = true;
            break;
        }
    }
    if (!found) {
        manifest->fields.push_back({
            .name = stats.name,
            .sourceIndex = static_cast<std::uint32_t>(stats.sourceIndex),
            .stats = stats,
        });
    }
    return WriteManifest(cacheDirectory, manifest.value());
}

}  // namespace invisible_places::io
