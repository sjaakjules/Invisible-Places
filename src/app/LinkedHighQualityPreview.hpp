#pragma once

#include "io/PointCloudData.hpp"

#include <glm/mat4x4.hpp>

#include <array>
#include <cstdint>
#include <filesystem>
#include <span>
#include <stop_token>
#include <string>
#include <string_view>
#include <vector>

namespace invisible_places::app {

constexpr float kLinkedHqViewportBorderFraction = 0.05F;

// Patch densities the HQ control offers. 1 mm keeps every source point
// inside the midpoint union; coarser choices keep a hash-selected fraction
// of the 1 mm scan (one in (spacing / 1 mm)^2 points) and declare that
// nominal spacing so density compensation restores the 1 mm coverage.
constexpr std::array<std::uint32_t, 3U> kLinkedHqPatchSpacingChoicesMicrometres{
    1'000U,
    2'000U,
    3'000U,
};

[[nodiscard]] std::uint32_t SanitizeLinkedHqPatchSpacing(
    std::uint32_t spacingMicrometres);

// One-in-N keep modulus for the 1 mm scan at the given patch spacing.
[[nodiscard]] std::uint32_t LinkedHqDecimationKeepOneIn(
    std::uint32_t spacingMicrometres);

// The two authored midpoint cameras define a union. BorderFraction is a
// fraction of the complete viewport dimension on every side: 0.05 extends
// the normalized X/Y clip limits from +/-1.0 to +/-1.1. Near/far clipping is
// intentionally never padded.
struct LinkedHqFrustumUnion {
    std::array<glm::mat4, 2U> midpointViewProjections{
        glm::mat4{1.0F},
        glm::mat4{1.0F},
    };
    float borderFraction = kLinkedHqViewportBorderFraction;

    [[nodiscard]] bool Contains(
        const invisible_places::io::Float3& point) const;
};

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
