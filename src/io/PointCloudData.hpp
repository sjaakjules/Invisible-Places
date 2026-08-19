#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <optional>
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
    // File-order position among the source's scalar_* properties, or -1 for
    // runtime-generated fields. This is the slot the field occupied when
    // every on-disk field loaded, which legacy documents may still reference
    // by raw index.
    std::int32_t sourceIndex = -1;

    void Include(float value);
};

// Every scalar_* property present in a source file, in file order, whether
// or not its values are resident in memory.
struct AvailableScalarField {
    std::string name;  // display name (scalar_ prefix stripped)
    std::uint32_t sourceIndex = 0;
};

// Chooses which on-disk scalar fields LoadPointCloud materialises. All is
// the historical behaviour. Selected loads only fields that match at least
// one criterion: display name in `names` (ASCII case-insensitive; unknown
// names are ignored, so runtime-generated field names may be passed
// freely), file-order index in `sourceIndices`, or normalized name
// (lowercase, alphanumerics only — matching the renderer's field
// heuristics) containing an entry of `containsPatterns`. Geometry, colour,
// and normals always load, and availableScalarFields always records the
// full on-disk field list.
struct PointCloudScalarFieldFilter {
    enum class Mode : std::uint8_t {
        All = 0,
        Selected,
    };
    Mode mode = Mode::All;
    std::vector<std::string> names;
    std::vector<std::uint32_t> sourceIndices;
    std::vector<std::string> containsPatterns;

    [[nodiscard]] bool LoadsAll() const { return mode == Mode::All; }
};

// The selection predicate LoadPointCloud applies, shared with the field
// cache so cache assembly and PLY loads pick identical field sets.
[[nodiscard]] bool PointCloudScalarFieldFilterSelects(
    const PointCloudScalarFieldFilter& filter,
    std::string_view displayName,
    std::uint32_t sourceIndex);

struct LoadedPointCloud {
    std::filesystem::path sourcePath;
    std::string layerName;
    std::vector<Float3> positions;
    std::vector<Float3> normals;
    std::vector<std::uint32_t> packedColors;
    std::vector<float> scalarFieldValues;
    std::vector<ScalarFieldStats> scalarFields;
    // The complete on-disk field list in file order, independent of which
    // fields are resident in scalarFields. Empty for clouds built entirely
    // at runtime (generated overlays).
    std::vector<AvailableScalarField> availableScalarFields;
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
    // Resident slot of the field whose file-order position is sourceIndex,
    // or nullopt when that field is not resident. Legacy documents may
    // address fields by file-order slot; this is the runtime translation.
    [[nodiscard]] std::optional<std::size_t> ResidentSlotForSourceIndex(
        std::int32_t sourceIndex) const;
};

struct PointCloudLoadResult {
    LoadedPointCloud cloud;
    std::string errorMessage;
    bool success = false;
};

struct PointCloudPositionNormalSample {
    Float3 position{};
    Float3 normal{};
    bool hasNormal = false;
    float roughness = 0.0F;
    bool hasRoughness = false;
};

struct PointCloudStreamResult {
    Bounds3f bounds;
    std::uint64_t pointCount = 0;
    std::string errorMessage;
    bool hasNormals = false;
    bool hasRoughness = false;
    bool success = false;
    bool cancelled = false;
};

using PointCloudPositionNormalVisitor =
    std::function<bool(const PointCloudPositionNormalSample&, std::uint64_t)>;

enum class PointCloudSelectedValueSource : std::uint8_t {
    ScalarField = 0,
    NormalX,
    NormalY,
    NormalZ,
};

struct PointCloudSelectedValueSelector {
    PointCloudSelectedValueSource source =
        PointCloudSelectedValueSource::ScalarField;
    // Uses the same display name as LoadedPointCloud::scalarFields; the
    // on-disk "scalar_" prefix is intentionally hidden from callers.
    std::string scalarFieldName;
};

struct PointCloudSelectedValueStreamResult {
    std::uint64_t pointCount = 0;
    std::string errorMessage;
    bool success = false;
    bool cancelled = false;
};

using PointCloudSelectedValueVisitor =
    std::function<bool(float, std::uint64_t)>;

// The fixed record stride lets the payload parse in parallel: disjoint
// vertex ranges are read and decoded by independent workers directly into
// the destination arrays, and stats/bounds/focus samples merge
// deterministically, so the result is bit-identical to a single-threaded
// parse. threadCount 0 sizes the pool automatically (bounded by hardware
// concurrency and the cloud's point count; small clouds stay
// single-threaded); an explicit count forces that many ranges.
PointCloudLoadResult LoadPointCloud(
    const std::filesystem::path& filePath,
    const PointCloudScalarFieldFilter& fieldFilter = {},
    unsigned threadCount = 0U);
PointCloudStreamResult StreamPointCloudPositionsNormals(
    const std::filesystem::path& filePath,
    const PointCloudPositionNormalVisitor& visitor);
PointCloudSelectedValueStreamResult StreamPointCloudSelectedValues(
    const std::filesystem::path& filePath,
    const PointCloudSelectedValueSelector& selector,
    const PointCloudSelectedValueVisitor& visitor);

}  // namespace invisible_places::io
