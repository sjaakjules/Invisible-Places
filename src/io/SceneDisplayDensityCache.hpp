#pragma once

#include "io/AssetDiscovery.hpp"

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace invisible_places::io {

inline constexpr std::uint32_t kSceneDisplayDensityCacheSchemaVersion = 1U;
inline constexpr std::string_view kSceneDisplayDensityCacheAlgorithmId =
    "scene-display-density-stratified-prefilter-v1";
inline constexpr std::uint64_t
    kSceneDisplayDensityCacheLegacyAlgorithmVersion = 1U;
inline constexpr std::string_view kSceneDisplayDensityCacheLegacyPositionPolicy =
    "real-parent-stable-hash";
inline constexpr std::uint64_t
    kSceneDisplayDensityCacheQ1CentroidMedoidAlgorithmVersion = 2U;
inline constexpr std::uint64_t
    kSceneDisplayDensityCacheCircularFieldAlgorithmVersion = 3U;
inline constexpr std::string_view
    kSceneDisplayDensityCacheQ1CentroidMedoidPositionPolicy =
        "real-parent-q1-centroid-medoid-qN-stable-hash";

enum class SceneDisplayDensityCacheState : std::uint8_t {
    Unavailable = 0,
    Activated,
    Rejected,
};

struct SceneDisplayDensityCacheRoleProvenance {
    std::string role;
    std::filesystem::path canonicalSourcePath;
    std::filesystem::path logicalDisplayPath;
    std::filesystem::path cachedDisplayPath;
    std::string sourceSha256;
    std::string outputSha256;
    std::uint64_t outputPointCount = 0U;
};

struct SceneDisplayDensityCacheActivation {
    SceneDisplayDensityCacheState state =
        SceneDisplayDensityCacheState::Unavailable;
    std::filesystem::path cacheRoot;
    std::filesystem::path manifestPath;
    std::string bundleFingerprint;
    std::string algorithmId;
    std::string message;
    std::vector<SceneDisplayDensityCacheRoleProvenance> roles;

    [[nodiscard]] bool Activated() const {
        return state == SceneDisplayDensityCacheState::Activated;
    }
};

// Validates the scene-specific active bundle before changing any asset. A
// successful activation preserves every logical/shared source path and only
// redirects the three Scene3 5 mm payload reads to the local cache. This keeps
// scene grouping, project paths, and canonical 1 mm export selection stable.
// Missing or invalid cache state removes prior redirects and leaves the catalog
// unchanged.
[[nodiscard]] SceneDisplayDensityCacheActivation
ActivateScene3DisplayDensityCache(
    const std::filesystem::path& cacheRoot,
    AssetCatalog* catalog);

// Point-cloud readers call this at their I/O boundary. Paths without a
// validated active override are returned unchanged.
[[nodiscard]] std::filesystem::path ResolveSceneDisplayDensityPayloadPath(
    const std::filesystem::path& logicalPath);
[[nodiscard]] std::string ActiveSceneDisplayDensityBundleFingerprint();
void ClearSceneDisplayDensityCacheActivation();

// Shared with explicit cache build/promotion verification and small synthetic
// tests so every stage uses the same SHA-256 implementation.
[[nodiscard]] std::string ComputeSceneDisplayDensitySha256(
    std::string_view value);
[[nodiscard]] std::optional<std::string>
ComputeSceneDisplayDensityFileSha256(
    const std::filesystem::path& path,
    std::string* errorMessage = nullptr);

}  // namespace invisible_places::io
