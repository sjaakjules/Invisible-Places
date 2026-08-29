#pragma once

#include "io/AdaptiveHqCache.hpp"
#include "io/PointCloudData.hpp"

#include <glm/mat4x4.hpp>

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <span>
#include <stop_token>
#include <string>
#include <string_view>
#include <vector>

namespace invisible_places::app {

constexpr float kLinkedHqViewportBorderFraction = 0.05F;
// Full-data Surface_05 profiling found that a 20% guard cuts the initial
// cached payload by roughly 43% versus the old whole-path union while still
// leaving several seconds of ordinary animation travel between refreshes.
constexpr float kAdaptiveHqViewportGuardFraction = 0.20F;
// Start preparing the next guard while the old patch still covers an extra
// 10% viewport border. Rendering itself uses a zero-border coverage check,
// so the complete 5 mm fallback is only exposed if I/O loses that headroom.
constexpr float kAdaptiveHqRefreshBorderFraction = 0.10F;
// Per role, excluding the blocks in the current guard (which are never
// evicted). The original 32-block fringe was only about 128 MiB and caused
// recently visited camera regions to be decoded repeatedly. Site3's largest
// complete 1 mm geometry role plus a few live scalar columns fits within this
// 6 GiB working set, while the limit still prevents an unbounded session.
constexpr std::uint64_t kAdaptiveHqRetainedFringeByteBudgetPerRole =
    6ULL * 1024ULL * 1024ULL * 1024ULL;
// A retained coarse guard consumes one uint32 point index plus six uint32
// surfel indices on the GPU. These limits keep timeline reuse broad on the
// user's unified-memory Mac while bounding the fallback layer independently
// of the decoded 1 mm block cache.
constexpr std::uint64_t kAdaptiveHqFiveMillimeterGuardGpuBytesPerPoint =
    7ULL * sizeof(std::uint32_t);
constexpr std::uint64_t kAdaptiveHqFiveMillimeterGuardByteBudgetPerRole =
    768ULL * 1024ULL * 1024ULL;
// Camera navigation changes direction unpredictably, so retain only the
// latest neighbouring guard and cap it at 30% of the complete coarse role.
constexpr double kAdaptiveHqNavigationRetainedGuardFraction = 0.30;
// Timeline guards overlap strongly between adjacent frames. Beyond 20% of
// the complete coarse cloud, drawing stale coarse history costs more than
// regenerating it helps; the current mask remains complete at every cap.
constexpr double kAdaptiveHqTimelineRetainedGuardFraction = 0.20;
// Cache blocks parse independently. Eight readers match the M1 Max worker
// cap used by patch assembly while retaining deterministic commit order.
constexpr std::size_t kAdaptiveHqBlockReadWorkerCount = 8U;
// Conservative coarse-layer visibility mask. Cells, rather than individual
// frustum planes at draw time, keep the CPU scan fast while the guard border
// prevents ordinary camera motion from exposing an edge.
constexpr std::uint32_t kAdaptiveHqFiveMillimeterGuardGridDimension = 64U;
constexpr double kAdaptiveHqFiveMillimeterGuardUsefulFraction = 0.90;

// Fine-patch visibility during an aHQ guard refresh is deliberately separate
// from coarse-layer masking. Once the camera leaves the published guard, the
// complete 5 mm cloud must return so newly exposed pixels cannot become holes,
// but the last fine patch can remain useful wherever it still intersects the
// new view. Keeping its depth transition active avoids drawing distant stale
// fine points at full density while the replacement is assembled.
struct LinkedHqPatchDrawPolicy {
    bool renderFinePatches = false;
    bool applyFineAdaptiveDensity = false;
    bool applyCoarseAdaptiveDensity = false;
    bool retainingFineDuringRefresh = false;
};

struct AdaptiveHqResidentRetention {
    std::vector<invisible_places::io::AdaptiveHqResidentBlock> blocks;
    std::uint64_t activePayloadBytes = 0U;
    std::uint64_t inactivePayloadBytes = 0U;
    std::size_t inactiveBlockCount = 0U;
};

enum class AdaptiveHqInteractionProfile : std::uint8_t {
    Navigation = 0,
    Timeline,
};

// Superseded CPU patches are retired into a bounded warm pool rather than
// freed on a fixed grace. Reclaiming an exact-selection return from that pool
// skips the multi-hundred-millisecond reassembly entirely; the pool budget
// plus farthest-first release under pressure bounds the retained bytes.
constexpr std::uint64_t kAdaptiveHqRetiredPatchPoolByteBudget =
    4ULL * 1024ULL * 1024ULL * 1024ULL;

// The per-role decoded-block fringe and the retired-patch pool are sized for
// a 64 GiB unified-memory Mac. Smaller machines scale both down so the aHQ
// working set stays near a quarter of physical memory, leaving the remainder
// for the resident 5 mm baseline, scalar fields, and the GPU share of
// unified memory.
struct AdaptiveHqMemoryBudget {
    std::uint64_t retainedFringeBytesPerRole =
        kAdaptiveHqRetainedFringeByteBudgetPerRole;
    std::uint64_t retiredPatchPoolBytes =
        kAdaptiveHqRetiredPatchPoolByteBudget;
};

// Physical RAM in bytes, or 0 when it cannot be determined.
[[nodiscard]] std::uint64_t DetectPhysicalMemoryBytes();

[[nodiscard]] AdaptiveHqMemoryBudget ResolveAdaptiveHqMemoryBudget(
    std::uint64_t physicalMemoryBytes);

// Warm-pool retention tiers by guard displacement. Subtle movement within an
// area keeps superseded patches reusable for minutes; a section move keeps
// them briefly for an immediate return; a teleport far outside the prepared
// depth demotes them soonest. Memory pressure always overrides the holds by
// releasing the farthest retired patch first.
struct AdaptiveHqRetiredPatchHold {
    std::chrono::milliseconds minimumHold{250};
    std::chrono::milliseconds maximumHold{std::chrono::minutes{5}};
};

[[nodiscard]] AdaptiveHqRetiredPatchHold
ResolveAdaptiveHqRetiredPatchHold(
    AdaptiveHqInteractionProfile interactionProfile,
    float guardDisplacementMeters,
    float preparedGuardDepthMeters);

struct AdaptiveHqFiveMillimeterGuardRetention {
    std::vector<std::uint32_t> indices;
    std::size_t retainedHistoryPointCount = 0U;
    bool historyLimited = false;
};

// Retains the newly requested 5 mm mask plus a bounded spatial-history
// fringe. Timeline work first reuses the complete retained working set;
// navigation keeps only the immediately prior guard. If either exceeds its
// point/GPU-byte cap, the latest guard is tried before falling back to the
// current request. Inputs and output are sorted unique point indices.
[[nodiscard]] AdaptiveHqFiveMillimeterGuardRetention
RetainAdaptiveHqFiveMillimeterGuard(
    std::span<const std::uint32_t> retainedGuard,
    std::span<const std::uint32_t> latestGuard,
    std::span<const std::uint32_t> currentGuard,
    std::uint64_t fullPointCount,
    AdaptiveHqInteractionProfile interactionProfile);

[[nodiscard]] float AdaptiveHqBoundsDistanceSquared(
    const invisible_places::io::Bounds3f& bounds,
    const invisible_places::io::Float3& point);

// Counts the decoded vector payload retained by a block. This is intentionally
// based on resident geometry, source IDs, and selected scalar columns rather
// than compressed/on-disk bytes, so the LRU policy reflects application RAM.
[[nodiscard]] std::uint64_t AdaptiveHqResidentBlockPayloadBytes(
    const invisible_places::io::AdaptiveHqResidentBlock& block);

// Active blocks always survive in the selector's deterministic order. The
// most recently used inactive blocks then fill a bounded per-role RAM working
// set, allowing orbiting and back-and-forth animation work to reuse decoded
// blocks without re-reading or re-parsing the local cache.
[[nodiscard]] AdaptiveHqResidentRetention RetainAdaptiveHqResidentBlocks(
    std::span<const invisible_places::io::AdaptiveHqResidentBlock> candidates,
    std::span<const std::uint32_t> activeBlockIndices,
    std::uint64_t inactiveByteBudget,
    AdaptiveHqInteractionProfile interactionProfile =
        AdaptiveHqInteractionProfile::Timeline);

[[nodiscard]] LinkedHqPatchDrawPolicy ResolveLinkedHqPatchDrawPolicy(
    bool linkedHqEnabled,
    bool adaptiveHqEnabled,
    bool adaptiveGuardCoversView,
    bool adaptiveTransitionValid);

// An unlinked animation covers its complete camera path rather than borrowing
// the linked pair's midpoint-only policy. Uniform samples keep curved motion
// represented between authored keys, while every authored key time is added
// separately by BuildUnlinkedAnimationHqSampleTimes.
constexpr std::size_t kUnlinkedAnimationHqUniformViewCount = 9U;

// Patch densities the HQ control offers. 1 mm keeps every source point
// inside the selected view union; coarser choices thin the 1 mm scan with the
// density-preserving cell stratification of the display-density cache
// (cells of the chosen spacing, keeping (1 mm / spacing)^2 of the parents)
// and declare that nominal spacing so density compensation restores the
// 1 mm coverage.
constexpr std::array<std::uint32_t, 3U> kLinkedHqPatchSpacingChoicesMicrometres{
    1'000U,
    2'000U,
    3'000U,
};

[[nodiscard]] std::uint32_t SanitizeLinkedHqPatchSpacing(
    std::uint32_t spacingMicrometres);

// Expected kept fraction of the 1 mm scan at the given patch spacing
// ((1 mm / spacing)^2; exactly 1 at 1 mm).
[[nodiscard]] float LinkedHqPatchKeepFraction(
    std::uint32_t spacingMicrometres);

// The supplied cameras define a union. Linked animations supply their two
// authored midpoint cameras; an unlinked animation supplies views spanning
// its complete path. BorderFraction is a fraction of the complete viewport
// dimension on every side: 0.05 extends the normalized X/Y clip limits from
// +/-1.0 to +/-1.1. Near/far clipping is intentionally never padded.
struct LinkedHqFrustumUnion {
    std::vector<glm::mat4> viewProjections;
    float borderFraction = kLinkedHqViewportBorderFraction;
    // Positive finite values limit accepted perspective view depth (clip.w).
    // Fixed HQ leaves this unlimited; aHQ only prepares fine data a little
    // beyond its GPU transition instead of scanning the complete far plane.
    float maximumViewDepthMeters = 0.0F;

    [[nodiscard]] bool Contains(
        const invisible_places::io::Float3& point) const;

    // Conservative block test used by the aHQ sidecar index. False means
    // every corner is outside at least one plane for every guarded view;
    // true may include a boundary block, which the exact point test trims.
    [[nodiscard]] bool IntersectsBounds(
        const invisible_places::io::Bounds3f& bounds) const;
};

// Builds the view union used by the compact patch pipeline. Exact duplicate
// matrices are discarded, but every distinct supplied view is retained so an
// unlinked animation can cover its complete path. Empty input produces an
// empty union that contains no points.
[[nodiscard]] LinkedHqFrustumUnion BuildAnimationHqFrustumUnion(
    std::span<const glm::mat4> viewProjections,
    float borderFraction = kLinkedHqViewportBorderFraction);

// Tests whether a complete current view, through requiredDepthMeters, remains
// inside an already prepared guarded union. aHQ uses this to retain its patch
// while navigating and to request a background replacement before exposing a
// patch edge. Fixed HQ does not use this policy.
[[nodiscard]] bool LinkedHqFrustumUnionCoversView(
    const LinkedHqFrustumUnion& frustumUnion,
    const glm::mat4& view,
    const glm::mat4& projection,
    float requiredDepthMeters,
    float requiredBorderFraction = 0.0F);

// Returns complete-path sample times in seconds. The schedule always includes
// the start, midpoint, end, and every finite authored key time, with at least
// kUnlinkedAnimationHqUniformViewCount evenly spaced samples for a non-zero
// duration. Results are sorted and near-duplicates are removed.
[[nodiscard]] std::vector<float> BuildUnlinkedAnimationHqSampleTimes(
    std::span<const float> authoredKeyTimesSeconds,
    float durationSeconds);

// Full-density live culling uses the focused Feature Run View as one stable
// camera section. Sampling every 30 fps frame (plus a small boundary pad)
// keeps the mask unchanged throughout that section, avoiding density or
// visibility pops while playback is running.
[[nodiscard]] std::vector<float> BuildAnimationSectionFrustumSampleTimes(
    float durationSeconds,
    float normalizedStart,
    float normalizedEnd,
    std::uint32_t framesPerSecond = 30U,
    std::uint32_t paddingFrames = 2U);

struct LinkedHqIndexPartition {
    std::vector<std::uint32_t> inside;
    std::vector<std::uint32_t> outside;
    bool cancelled = false;
};

// Evaluates every point once and places it in exactly one ordered partition.
// The inside predicate is shared with filtered 1 mm loading, making the 1 mm
// patch and complementary 5 mm mask agree at frustum boundaries.
[[nodiscard]] LinkedHqIndexPartition PartitionLinkedHqIndices(
    std::span<const invisible_places::io::Float3> positions,
    const LinkedHqFrustumUnion& frustumUnion,
    std::stop_token stopToken = {},
    const invisible_places::io::PointCloudLoadProgress& progress = {});

struct LinkedHqSourceFingerprint {
    std::filesystem::path path;
    std::uintmax_t byteSize = 0U;
    std::int64_t writeTimeTicks = 0;
    std::string schemaFingerprint;
    std::string contentFingerprint;
    std::uint64_t pointCount = 0U;
    std::uint32_t recordSize = 0U;

    [[nodiscard]] bool operator==(
        const LinkedHqSourceFingerprint&) const = default;
};

[[nodiscard]] bool TryFingerprintLinkedHqSource(
    const std::filesystem::path& path,
    LinkedHqSourceFingerprint* fingerprint,
    std::string* errorMessage = nullptr);

// Stable within an application build and process architecture; intended for
// in-memory invalidation only, never persisted into a project document.
[[nodiscard]] std::uint64_t BuildLinkedHqSelectionFingerprint(
    std::string_view pairKey,
    const LinkedHqFrustumUnion& frustumUnion,
    std::span<const LinkedHqSourceFingerprint> sources);

enum class LinkedHqPreparationStage : std::uint8_t {
    Waiting = 0,
    Scanning,
    Organising,
    Uploading,
    Ready,
    Failed,
};

[[nodiscard]] const char* LinkedHqPreparationStageName(
    LinkedHqPreparationStage stage);

// Maps stages onto a single progress interval used by the HQ control. The
// caller retains the maximum it has published, so cross-thread estimates can
// only move the button forwards.
[[nodiscard]] float LinkedHqOverallProgress(
    LinkedHqPreparationStage stage,
    float stageProgress);

}  // namespace invisible_places::app
