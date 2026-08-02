#pragma once

#include "io/PointCloudData.hpp"
#include "timing/TimingColourise.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <span>
#include <stop_token>
#include <string>
#include <string_view>
#include <vector>

namespace invisible_places::timing {

// The full raw range remains exact, but this density leaves enough source
// buckets for Distribution Spread to reveal narrow concentrations even when
// a handful of outliers make the raw range much wider than the useful data.
inline constexpr std::size_t kTimingColouriseHistogramBinCount = 16'384U;
inline constexpr std::size_t
    kTimingColouriseHistogramReferenceDisplayBinCount = 256U;
inline constexpr std::uint32_t
    kTimingColouriseHistogramCacheSchemaVersion = 2U;
inline constexpr std::size_t kTimingColouriseAuthoredLayerCount = 3U;

struct TimingColouriseLayerFieldSet {
    std::vector<std::string> scalarFieldNames;
    bool hasNormals = false;
};

struct TimingColouriseFieldVariant {
    std::string id;
    std::string name;
    TimingColouriseFieldSelector selector{};
};

struct TimingColouriseFieldFamily {
    std::string id;
    std::string name;
    std::vector<TimingColouriseFieldVariant> variants;
};

// The input order is SAND, ROCK, VEG. Only scalar fields present on all three
// authored layers are returned. Synthetic Normal X/Y/Z variants are present
// only when all three layers expose resident normals.
[[nodiscard]] std::vector<TimingColouriseFieldFamily>
BuildTimingColouriseFieldCatalog(
    const std::array<
        TimingColouriseLayerFieldSet,
        kTimingColouriseAuthoredLayerCount>& layers);

[[nodiscard]] TimingColouriseLayerFieldSet
TimingColouriseLayerFieldSetFromCloud(
    const invisible_places::io::LoadedPointCloud& cloud);

[[nodiscard]] bool IsGeneratedTimingColouriseScalarField(
    std::string_view fieldName);

struct TimingColouriseHistogram {
    float minimum = 0.0F;
    float maximum = 0.0F;
    std::uint64_t finiteValueCount = 0U;
    // High-resolution storage must remain heap-backed: macOS worker threads
    // have a much smaller default stack than the main UI thread, and a few
    // fixed-array result copies can otherwise exhaust it before work begins.
    std::vector<std::uint64_t> bins =
        std::vector<std::uint64_t>(
            kTimingColouriseHistogramBinCount,
            0U);

    [[nodiscard]] bool Valid() const {
        return bins.size() == kTimingColouriseHistogramBinCount &&
               finiteValueCount > 0U && maximum >= minimum;
    }
};

// Integrates the high-resolution counts over a sliding raw-value window the
// width of one former 256-bin bucket. This preserves broad distribution shape
// and prevents isolated endpoint spikes from flattening the graph, while the
// original 16,384 bins remain available to the Distribution Spread axis.
[[nodiscard]] std::vector<std::uint64_t>
BuildTimingColouriseHistogramDisplayBins(
    const TimingColouriseHistogram& histogram);

// Maps a bin count linearly onto the histogram's occupied display range. The
// least-populated bucket is pinned to zero and the most-populated bucket to
// one, so dense fields do not produce a misleading raised baseline when
// every bucket contains values. Empty buckets participate in the minimum. An
// entirely uniform non-empty histogram remains visible at full height because
// it has no relative range to stretch.
[[nodiscard]] float TimingColouriseHistogramDisplayHeight(
    std::uint64_t binCount,
    std::uint64_t minimumBinCount,
    std::uint64_t maximumBinCount);

enum class TimingColouriseHistogramAxisMode : std::uint8_t {
    Raw = 0,
    DistributionSpread,
};

enum class TimingColouriseHistogramAxisShape : std::uint8_t {
    Raw = 0,
    Centred,
    PositiveOneSided,
    NegativeOneSided,
};

// A display-only editing axis derived from the cached histogram. Raw bounds
// and keyed values remain unchanged: the UI maps them through RawToUnit for
// drawing and maps pointer positions back through UnitToRaw before editing.
//
// Distribution Spread uses a piecewise-linear CDF warp with a small raw-linear
// contribution. The raw contribution keeps empty histogram runs invertible,
// while the CDF contribution gives dense regions more mouse resolution.
// Centred fields split the CDF at raw zero and anchor it exactly at the middle
// of a -1..1 presentation. Predominantly one-sided fields use the full CDF as
// a 0..1 presentation. One additional knot accommodates raw zero between the
// histogram's high-resolution linear bins.
struct TimingColouriseHistogramAxis {
    static constexpr std::size_t kMaximumKnotCount =
        kTimingColouriseHistogramBinCount + 2U;

    TimingColouriseHistogramAxisMode mode =
        TimingColouriseHistogramAxisMode::Raw;
    TimingColouriseHistogramAxisShape shape =
        TimingColouriseHistogramAxisShape::Raw;
    float rawMinimum = 0.0F;
    float rawMaximum = 1.0F;
    float zeroUnit = 0.0F;
    bool validRange = false;
    std::size_t knotCount = 0U;
    std::array<float, kMaximumKnotCount> rawKnots{};
    std::array<float, kMaximumKnotCount> unitKnots{};

    [[nodiscard]] bool UsesDistributionSpread() const {
        return mode ==
               TimingColouriseHistogramAxisMode::DistributionSpread;
    }
    [[nodiscard]] float RawToUnit(float rawValue) const;
    [[nodiscard]] float UnitToRaw(float unitValue) const;
};

// Building this axis never revisits point data. It derives the warp from the
// histogram bins already held in memory or loaded from the persistent cache.
// Invalid and constant histograms fall back to a non-editable Raw axis.
[[nodiscard]] TimingColouriseHistogramAxis
BuildTimingColouriseHistogramAxis(
    const TimingColouriseHistogram& histogram,
    TimingColouriseHistogramAxisMode requestedMode);

struct TimingColouriseHistogramResult {
    TimingColouriseHistogram histogram{};
    std::string errorMessage;
    bool success = false;
    bool cancelled = false;
};

using TimingColouriseResidentCloudBundle = std::array<
    const invisible_places::io::LoadedPointCloud*,
    kTimingColouriseAuthoredLayerCount>;

struct TimingColouriseHistogramLayerSource {
    // Used without file IO when the selected field/component arrays are
    // complete. A released or incomplete resident cloud falls back to
    // sourcePath.
    const invisible_places::io::LoadedPointCloud* residentCloud = nullptr;
    std::filesystem::path sourcePath;
};

using TimingColouriseHistogramSourceBundle = std::array<
    TimingColouriseHistogramLayerSource,
    kTimingColouriseAuthoredLayerCount>;

// Computes the exact range and high-resolution linear bins in two passes over
// resident field-major arrays. Non-finite values are ignored. This function
// does no file IO and is suitable for a cancellable background worker.
[[nodiscard]] TimingColouriseHistogramResult
ComputeTimingColouriseHistogram(
    const TimingColouriseResidentCloudBundle& clouds,
    const TimingColouriseFieldSelector& selector,
    std::stop_token stopToken = {});

// Prefers complete resident field-major arrays and streams only layers whose
// selected values have been released. Streaming makes two bounded-memory
// passes over each required PLY so the full-scene range and bins remain exact.
[[nodiscard]] TimingColouriseHistogramResult
ComputeTimingColouriseHistogramFromSources(
    const TimingColouriseHistogramSourceBundle& sources,
    const TimingColouriseFieldSelector& selector,
    std::stop_token stopToken = {});

struct TimingColouriseHistogramSourceIdentity {
    std::filesystem::path sourcePath;
    std::uint64_t fileSize = 0U;
    std::int64_t modificationTimeNanoseconds = 0;
};

struct TimingColouriseHistogramFingerprintInput {
    std::uint32_t schemaVersion =
        kTimingColouriseHistogramCacheSchemaVersion;
    std::string sceneGroupName;
    TimingColouriseFieldSelector selector{};
    // SAND, ROCK, VEG, in that order.
    std::array<
        TimingColouriseHistogramSourceIdentity,
        kTimingColouriseAuthoredLayerCount>
        sources{};
    std::uint32_t displaySpacingMicrometres = 0U;
};

[[nodiscard]] TimingColouriseHistogramSourceIdentity
InspectTimingColouriseHistogramSource(
    const std::filesystem::path& sourcePath);

// Returns a deterministic lowercase hexadecimal FNV-1a fingerprint. Every
// member of TimingColouriseHistogramFingerprintInput participates.
[[nodiscard]] std::string BuildTimingColouriseHistogramFingerprint(
    const TimingColouriseHistogramFingerprintInput& input);

[[nodiscard]] std::filesystem::path
TimingColouriseHistogramCacheDirectory(
    const std::filesystem::path& sceneRoot);
[[nodiscard]] std::filesystem::path TimingColouriseHistogramCachePath(
    const std::filesystem::path& sceneRoot,
    std::string_view fingerprint);

// Binary cache helpers include and validate the supplied fingerprint, schema,
// histogram range, count, and bin checksum. Corrupt or stale data is rejected.
[[nodiscard]] bool SaveTimingColouriseHistogramCache(
    const std::filesystem::path& cachePath,
    std::string_view fingerprint,
    const TimingColouriseHistogram& histogram,
    std::string* errorMessage = nullptr);
[[nodiscard]] std::optional<TimingColouriseHistogram>
LoadTimingColouriseHistogramCache(
    const std::filesystem::path& cachePath,
    std::string_view expectedFingerprint,
    std::string* errorMessage = nullptr);

}  // namespace invisible_places::timing
