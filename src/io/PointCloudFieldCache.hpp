#pragma once

#include "io/PointCloudData.hpp"

#include <cstdint>
#include <filesystem>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace invisible_places::io {

inline constexpr std::uint32_t kPointCloudFieldCacheSchemaVersion = 1U;

// A field-major on-disk mirror of one source point cloud, so restarts and
// on-demand field loads read compact contiguous arrays instead of
// re-scanning the interleaved PLY record stream. Layout, beside the source
// file under <dir>/.invisible_places/cache/fields/<stem>-<hash>/:
//
//   manifest.json      source identity (size + mtime), point count, flags,
//                      bounds/focus, and per-field stats for every field
//                      that has ever been materialised
//   geometry.bin       positions, packed colours, then normals (if any)
//   field_<idx>_<name>.bin   one float per point, file-order index idx
//
// Everything is best-effort: a missing, stale, or partial cache falls back
// to the PLY loader, and all writes go through a temp file + rename so a
// crash never leaves a truncated artefact behind. The cache is only
// trusted while the manifest's recorded source size and mtime match the
// file on disk.
struct PointCloudFieldCacheFieldEntry {
    std::string name;
    std::uint32_t sourceIndex = 0;
    ScalarFieldStats stats;  // valid only once materialised at least once
};

struct PointCloudFieldCacheManifest {
    std::uint32_t schemaVersion = kPointCloudFieldCacheSchemaVersion;
    std::uint64_t sourceSizeBytes = 0;
    std::int64_t sourceMtimeNanoseconds = 0;
    std::uint64_t pointCount = 0;
    bool hasSourceRgb = false;
    bool hasNormals = false;
    bool hasFocusPoint = false;
    Float3 focusPoint{};
    Bounds3f bounds{};
    std::vector<PointCloudFieldCacheFieldEntry> fields;
};

[[nodiscard]] std::filesystem::path PointCloudFieldCacheDirectory(
    const std::filesystem::path& sourcePath);

// The desktop app points this at its machine-local Saved tree when source
// PLY files live in a synchronized folder. Empty restores the historical
// beside-source location used by standalone tools and tests.
void SetPointCloudFieldCacheRoot(const std::filesystem::path& cacheRoot);

// Manifest loaded from cacheDirectory when present, parseable, schema-
// compatible, and matching the current source file identity.
[[nodiscard]] std::optional<PointCloudFieldCacheManifest>
LoadValidPointCloudFieldCacheManifest(const std::filesystem::path& sourcePath);

// Loads through the cache: assembles geometry and every filter-selected,
// cached field from the cache directory, streams selected-but-uncached
// fields from the PLY (writing them through), or falls back to a full
// filtered PLY load on any miss — after which geometry, resident fields,
// and the manifest are written for next time. Behaviour (returned cloud,
// stats, availableScalarFields) is identical to LoadPointCloud with the
// same filter.
[[nodiscard]] PointCloudLoadResult LoadPointCloudWithFieldCache(
    const std::filesystem::path& sourcePath,
    const PointCloudScalarFieldFilter& fieldFilter = {});

// Reads one cached field's values (values.size() must equal the manifest
// point count). Returns false when the cache or that field file is absent,
// stale, or the wrong size. stats, when non-null, receives the manifest
// stats for the field.
[[nodiscard]] bool ReadPointCloudCachedField(
    const std::filesystem::path& sourcePath,
    std::string_view fieldName,
    std::span<float> values,
    ScalarFieldStats* stats);

// Writes one field's values and upserts its manifest stats, creating the
// cache directory and manifest if needed (geometry may be cached later by
// the next full load). Returns false when the cache identity cannot be
// established or any write fails; failures leave prior contents intact.
[[nodiscard]] bool WritePointCloudCachedField(
    const std::filesystem::path& sourcePath,
    const ScalarFieldStats& stats,
    std::span<const float> values);

}  // namespace invisible_places::io
