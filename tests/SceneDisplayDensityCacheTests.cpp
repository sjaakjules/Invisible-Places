#include "io/AssetDiscovery.hpp"
#include "io/PointCloudData.hpp"
#include "io/SceneDisplayDensityCache.hpp"
#include "scene/PointCloudVariants.hpp"

#include <catch2/catch_test_macros.hpp>

#include <nlohmann/json.hpp>

#include <array>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>

#if defined(__APPLE__) || defined(__linux__)
#include <sys/stat.h>
#endif

namespace {

constexpr std::uint64_t kGeneratorQ1CentroidAlgorithmVersion = 2U;
constexpr std::string_view kGeneratorQ1CentroidPositionPolicy =
    "real-parent-q1-centroid-medoid-qN-stable-hash";

class TemporaryDirectory {
  public:
    TemporaryDirectory() {
        static std::atomic_uint64_t nextId{0U};
        path = std::filesystem::temp_directory_path() /
               ("invisible-places-density-activation-" +
                std::to_string(
                    std::chrono::steady_clock::now()
                        .time_since_epoch()
                        .count()) +
                "-" + std::to_string(nextId.fetch_add(1U)));
        std::filesystem::create_directories(path);
    }

    ~TemporaryDirectory() {
        std::error_code error;
        std::filesystem::remove_all(path, error);
    }

    TemporaryDirectory(const TemporaryDirectory&) = delete;
    TemporaryDirectory& operator=(const TemporaryDirectory&) = delete;

    std::filesystem::path path;
};

template <typename Value>
void WriteBinary(std::ostream* output, const Value& value) {
    output->write(
        reinterpret_cast<const char*>(&value),
        static_cast<std::streamsize>(sizeof(Value)));
}

void WritePointCloud(
    const std::filesystem::path& path,
    std::uint64_t pointCount,
    float xOffset) {
    std::filesystem::create_directories(path.parent_path());
    std::ofstream output{path, std::ios::binary | std::ios::trunc};
    output << "ply\n";
    output << "format binary_little_endian 1.0\n";
    output << "element vertex " << pointCount << "\n";
    output << "property float x\n";
    output << "property float y\n";
    output << "property float z\n";
    output << "property uchar red\n";
    output << "property uchar green\n";
    output << "property uchar blue\n";
    output << "property float scalar_ScanID\n";
    output << "end_header\n";
    for (std::uint64_t index = 0U; index < pointCount; ++index) {
        const auto x = xOffset + static_cast<float>(index);
        const auto y = static_cast<float>(index) * 0.5F;
        const auto z = 0.25F;
        const std::uint8_t red = 10U;
        const std::uint8_t green = 20U;
        const std::uint8_t blue = 30U;
        const float scanId = 13.0F;
        WriteBinary(&output, x);
        WriteBinary(&output, y);
        WriteBinary(&output, z);
        WriteBinary(&output, red);
        WriteBinary(&output, green);
        WriteBinary(&output, blue);
        WriteBinary(&output, scanId);
    }
}

std::int64_t MtimeNanoseconds(const std::filesystem::path& path) {
#if defined(__APPLE__) || defined(__linux__)
    struct stat metadata {};
    REQUIRE(::stat(path.c_str(), &metadata) == 0);
#if defined(__APPLE__)
    return static_cast<std::int64_t>(metadata.st_mtimespec.tv_sec) *
               1'000'000'000LL +
           static_cast<std::int64_t>(metadata.st_mtimespec.tv_nsec);
#else
    return static_cast<std::int64_t>(metadata.st_mtim.tv_sec) *
               1'000'000'000LL +
           static_cast<std::int64_t>(metadata.st_mtim.tv_nsec);
#endif
#else
    const auto modified = std::filesystem::last_write_time(path);
    const auto systemModified =
        std::filesystem::file_time_type::clock::to_sys(modified);
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               systemModified.time_since_epoch())
        .count();
#endif
}

struct SyntheticBundle {
    std::filesystem::path dataRoot;
    std::filesystem::path cacheRoot;
    std::filesystem::path manifestPath;
    std::string fingerprint;
    std::array<std::filesystem::path, 3U> sourcePaths;
    std::array<std::filesystem::path, 3U> logicalDisplayPaths;
    std::array<std::filesystem::path, 3U> cachedDisplayPaths;
};

SyntheticBundle WriteSyntheticBundle(
    const std::filesystem::path& root,
    std::string_view rgbFilter = "renderer-byte-mean",
    std::uint64_t algorithmVersion =
        invisible_places::io::
            kSceneDisplayDensityCacheLegacyAlgorithmVersion,
    std::string_view positionPolicy =
        invisible_places::io::
            kSceneDisplayDensityCacheLegacyPositionPolicy,
    double voxelSizeMeters = 0.005) {
    constexpr std::array<std::string_view, 3U> roles{
        "ROCK",
        "SAND",
        "VEG",
    };
    SyntheticBundle bundle;
    bundle.dataRoot = root / "Data";
    bundle.cacheRoot = root / "Saved" / ".invisible_places" / "cache" /
                       "display_density" / "Scene3";
    const auto stagingRoot = bundle.cacheRoot / "staging";

    const nlohmann::json algorithm{
        {"id",
         invisible_places::io::kSceneDisplayDensityCacheAlgorithmId},
        {"version", algorithmVersion},
        {"seed_hex", "0000000000001234"},
        {"voxel_size_m", voxelSizeMeters},
        {"rgb_filter", rgbFilter},
        {"apportionment", "seeded-systematic-parent-population"},
        {"position_policy", positionPolicy},
        {"cell_grid_offset", "half-voxel-xyz"},
        {"normal_filter",
         "hemisphere-aligned-normalized-mean-cosine-gate-0.5"},
        {"categorical_filter", "scanid-mode-low-value-tie"},
        {"continuous_filter", "finite-arithmetic-mean"},
    };
    const auto schemaDigest =
        invisible_places::io::ComputeSceneDisplayDensitySha256(
            nlohmann::json{
                {"format", "binary_little_endian"},
                {"vertex_properties",
                 nlohmann::json::array({
                     {{"type", "float"}, {"name", "x"}},
                     {{"type", "float"}, {"name", "y"}},
                     {{"type", "float"}, {"name", "z"}},
                     {{"type", "uchar"}, {"name", "red"}},
                     {{"type", "uchar"}, {"name", "green"}},
                     {{"type", "uchar"}, {"name", "blue"}},
                     {{"type", "float"}, {"name", "scalar_ScanID"}},
                 })},
            }
                .dump());

    nlohmann::json roleEntries = nlohmann::json::array();
    nlohmann::json fingerprintRoles = nlohmann::json::array();
    for (std::size_t index = 0U; index < roles.size(); ++index) {
        const auto role = std::string{roles[index]};
        bundle.sourcePaths[index] =
            bundle.dataRoot / "Scene3" /
            ("Site3-" + role + "-1mm.ply");
        bundle.logicalDisplayPaths[index] =
            bundle.dataRoot / "Scene3" /
            ("Site3-" + role + "-5mm.ply");
        bundle.cachedDisplayPaths[index] =
            stagingRoot / "Scene3" /
            ("Site3-" + role + "-5mm.ply");
        WritePointCloud(bundle.sourcePaths[index], 5U + index, 1.0F);
        WritePointCloud(bundle.logicalDisplayPaths[index], 2U, 10.0F);
        WritePointCloud(bundle.cachedDisplayPaths[index], 3U + index, 100.0F);

        const auto sourceSha256 =
            invisible_places::io::ComputeSceneDisplayDensityFileSha256(
                bundle.sourcePaths[index]);
        const auto outputSha256 =
            invisible_places::io::ComputeSceneDisplayDensityFileSha256(
                bundle.cachedDisplayPaths[index]);
        REQUIRE(sourceSha256.has_value());
        REQUIRE(outputSha256.has_value());
        roleEntries.push_back({
            {"role", role},
            {"source",
             {{"path", std::filesystem::absolute(bundle.sourcePaths[index]).string()},
              {"size_bytes", std::filesystem::file_size(bundle.sourcePaths[index])},
              {"mtime_ns", MtimeNanoseconds(bundle.sourcePaths[index])},
              {"sha256", sourceSha256.value()},
              {"vertex_count", 5U + index},
              {"schema_sha256", schemaDigest}}},
            {"requested_point_count", 3U + index},
            {"output",
             {{"file",
               std::filesystem::relative(
                   bundle.cachedDisplayPaths[index],
                   stagingRoot)
                   .generic_string()},
              {"sha256", outputSha256.value()},
              {"size_bytes", std::filesystem::file_size(bundle.cachedDisplayPaths[index])},
              {"mtime_ns", MtimeNanoseconds(bundle.cachedDisplayPaths[index])},
              {"vertex_count", 3U + index},
              {"schema_sha256", schemaDigest}}},
        });
        fingerprintRoles.push_back({
            {"role", role},
            {"requested_point_count", 3U + index},
            {"source_sha256", sourceSha256.value()},
            {"source_schema_sha256", schemaDigest},
            {"output_sha256", outputSha256.value()},
            {"output_schema_sha256", schemaDigest},
        });
    }

    bundle.fingerprint =
        invisible_places::io::ComputeSceneDisplayDensitySha256(
            nlohmann::json{
                {"schema_version", 1U},
                {"algorithm", algorithm},
                {"scene", "Scene3"},
                {"roles", fingerprintRoles},
            }
                .dump());
    const auto bundleRoot = bundle.cacheRoot / bundle.fingerprint;
    std::filesystem::rename(stagingRoot, bundleRoot);
    for (std::size_t index = 0U; index < bundle.cachedDisplayPaths.size(); ++index) {
        bundle.cachedDisplayPaths[index] =
            bundleRoot / "Scene3" /
            bundle.cachedDisplayPaths[index].filename();
    }
    bundle.manifestPath = bundleRoot / "display-density-manifest.json";
    {
        std::ofstream manifest{bundle.manifestPath};
        manifest << nlohmann::json{
                        {"schema_version", 1U},
                        {"algorithm", algorithm},
                        {"scene", "Scene3"},
                        {"complete", true},
                        {"bundle_fingerprint", bundle.fingerprint},
                        {"roles", std::move(roleEntries)},
                    }
                        .dump(2);
    }
    const auto manifestSha256 =
        invisible_places::io::ComputeSceneDisplayDensityFileSha256(
            bundle.manifestPath);
    REQUIRE(manifestSha256.has_value());
    std::filesystem::create_directories(bundle.cacheRoot);
    {
        std::ofstream pointer{bundle.cacheRoot / "active-bundle.json"};
        pointer << nlohmann::json{
                       {"schema_version", 1U},
                       {"bundle_fingerprint", bundle.fingerprint},
                       {"manifest_sha256", manifestSha256.value()},
                   }
                       .dump(2);
    }
    return bundle;
}

const invisible_places::io::PointCloudAsset* FindAsset(
    const invisible_places::io::AssetCatalog& catalog,
    std::string_view role,
    float spacingMeters) {
    const auto found = std::find_if(
        catalog.pointClouds.begin(),
        catalog.pointClouds.end(),
        [&](const auto& asset) {
            return asset.sceneGroupName == "Scene3" &&
                   asset.sceneRole == role &&
                   std::abs(
                       asset.inferredPointSpacingMeters - spacingMeters) <
                       1.0e-7F;
        });
    return found == catalog.pointClouds.end() ? nullptr : &*found;
}

}  // namespace

TEST_CASE(
    "Scene3 display-density cache activates as one transparent transaction",
    "[pointcloud][density][cache]") {
    TemporaryDirectory temporary;
    const auto bundle = WriteSyntheticBundle(temporary.path);
    auto catalog = invisible_places::io::DiscoverAssets(bundle.dataRoot);

    const auto beforeGroups =
        invisible_places::scene::BuildScenePointCloudGroups(catalog);
    REQUIRE(beforeGroups.size() == 1U);
    REQUIRE(beforeGroups.front().completeDisplayBundles.size() == 2U);

    const auto activation =
        invisible_places::io::ActivateScene3DisplayDensityCache(
            bundle.cacheRoot,
            &catalog);
    REQUIRE(activation.Activated());
    REQUIRE(activation.roles.size() == 3U);
    CHECK(activation.bundleFingerprint == bundle.fingerprint);

    for (std::size_t index = 0U; index < bundle.sourcePaths.size(); ++index) {
        const auto role = std::array<std::string_view, 3U>{
            "ROCK",
            "SAND",
            "VEG",
        }[index];
        const auto* source = FindAsset(catalog, role, 0.001F);
        const auto* display = FindAsset(catalog, role, 0.005F);
        REQUIRE(source != nullptr);
        REQUIRE(display != nullptr);
        CHECK(source->filePath == bundle.sourcePaths[index]);
        CHECK(display->filePath == bundle.logicalDisplayPaths[index]);
        CHECK(source->header.vertexCount == 5U + index);
        CHECK(display->header.vertexCount == 3U + index);
        CHECK(
            invisible_places::io::ResolveSceneDisplayDensityPayloadPath(
                source->filePath) == source->filePath);
        CHECK(
            invisible_places::io::ResolveSceneDisplayDensityPayloadPath(
                display->filePath) ==
            std::filesystem::canonical(bundle.cachedDisplayPaths[index]));
    }

    const auto afterGroups =
        invisible_places::scene::BuildScenePointCloudGroups(catalog);
    REQUIRE(afterGroups.size() == 1U);
    CHECK(afterGroups.front().sourceFolder == bundle.dataRoot / "Scene3");
    REQUIRE(afterGroups.front().completeDisplayBundles.size() == 2U);
    CHECK(
        afterGroups.front().FindCompleteDisplayBundle(1'000U) != nullptr);
    const auto* displayBundle =
        afterGroups.front().FindCompleteDisplayBundle(5'000U);
    REQUIRE(displayBundle != nullptr);
    CHECK(
        displayBundle->Find(
            invisible_places::scene::ScenePointCloudRole::Rock)
            .sourcePath == bundle.logicalDisplayPaths[0U]);

    const auto loaded =
        invisible_places::io::LoadPointCloud(bundle.logicalDisplayPaths[0U]);
    REQUIRE(loaded.success);
    REQUIRE(loaded.cloud.PointCount() == 3U);
    CHECK(loaded.cloud.sourcePath == bundle.logicalDisplayPaths[0U]);
    CHECK(loaded.cloud.positions.front().x == 100.0F);

    invisible_places::io::ClearSceneDisplayDensityCacheActivation();
    CHECK(
        invisible_places::io::ActiveSceneDisplayDensityBundleFingerprint()
            .empty());
    CHECK(
        invisible_places::io::ResolveSceneDisplayDensityPayloadPath(
            bundle.logicalDisplayPaths[0U]) ==
        bundle.logicalDisplayPaths[0U]);
}

TEST_CASE(
    "Scene3 display-density cache accepts the exact q1 centroid-medoid policy",
    "[pointcloud][density][cache]") {
    TemporaryDirectory temporary;
    const auto bundle = WriteSyntheticBundle(
        temporary.path,
        "renderer-byte-mean",
        kGeneratorQ1CentroidAlgorithmVersion,
        kGeneratorQ1CentroidPositionPolicy);
    auto catalog = invisible_places::io::DiscoverAssets(bundle.dataRoot);

    const auto activation =
        invisible_places::io::ActivateScene3DisplayDensityCache(
            bundle.cacheRoot,
            &catalog);
    INFO(activation.message);
    REQUIRE(activation.Activated());
    CHECK(activation.bundleFingerprint == bundle.fingerprint);
    CHECK(
        invisible_places::io::ActiveSceneDisplayDensityBundleFingerprint() ==
        bundle.fingerprint);
}

TEST_CASE(
    "Scene3 display-density cache rejects crossed or unknown position policies",
    "[pointcloud][density][cache]") {
    struct UnsupportedPolicy {
        std::uint64_t version;
        std::string_view positionPolicy;
    };
    constexpr std::array<UnsupportedPolicy, 4U> unsupportedPolicies{
        UnsupportedPolicy{
            invisible_places::io::
                kSceneDisplayDensityCacheLegacyAlgorithmVersion,
            kGeneratorQ1CentroidPositionPolicy},
        UnsupportedPolicy{
            kGeneratorQ1CentroidAlgorithmVersion,
            invisible_places::io::
                kSceneDisplayDensityCacheLegacyPositionPolicy},
        UnsupportedPolicy{
            kGeneratorQ1CentroidAlgorithmVersion,
            "real-parent-q1-centroid-medoid-qN-stable-hash-unknown"},
        UnsupportedPolicy{
            3U,
            kGeneratorQ1CentroidPositionPolicy},
    };

    TemporaryDirectory temporary;
    for (std::size_t index = 0U; index < unsupportedPolicies.size(); ++index) {
        const auto& policy = unsupportedPolicies[index];
        const auto bundle = WriteSyntheticBundle(
            temporary.path / std::to_string(index),
            "renderer-byte-mean",
            policy.version,
            policy.positionPolicy);
        auto catalog = invisible_places::io::DiscoverAssets(bundle.dataRoot);

        const auto activation =
            invisible_places::io::ActivateScene3DisplayDensityCache(
                bundle.cacheRoot,
                &catalog);
        INFO("version " << policy.version << ", policy "
                        << policy.positionPolicy);
        CHECK_FALSE(activation.Activated());
        CHECK(
            activation.state ==
            invisible_places::io::SceneDisplayDensityCacheState::Rejected);
        CHECK(
            activation.message.find("prefilter policy") !=
            std::string::npos);
    }
}

TEST_CASE(
    "Scene3 display-density cache rejects one stale role without partial overlay",
    "[pointcloud][density][cache]") {
    TemporaryDirectory temporary;
    const auto bundle = WriteSyntheticBundle(temporary.path);
    const auto originalMtime =
        std::filesystem::last_write_time(bundle.cachedDisplayPaths[1U]);
    {
        std::fstream output{
            bundle.cachedDisplayPaths[1U],
            std::ios::binary | std::ios::in | std::ios::out};
        REQUIRE(output.is_open());
        output.seekp(-static_cast<std::streamoff>(sizeof(float)), std::ios::end);
        const float changedScanId = 99.0F;
        WriteBinary(&output, changedScanId);
    }
    std::filesystem::last_write_time(
        bundle.cachedDisplayPaths[1U],
        originalMtime);
    auto catalog = invisible_places::io::DiscoverAssets(bundle.dataRoot);

    const auto activation =
        invisible_places::io::ActivateScene3DisplayDensityCache(
            bundle.cacheRoot,
            &catalog);
    CHECK_FALSE(activation.Activated());
    CHECK(
        activation.state ==
        invisible_places::io::SceneDisplayDensityCacheState::Rejected);
    CHECK(
        activation.message.find("SHA-256") != std::string::npos);

    for (std::size_t index = 0U; index < bundle.logicalDisplayPaths.size(); ++index) {
        const auto role = std::array<std::string_view, 3U>{
            "ROCK",
            "SAND",
            "VEG",
        }[index];
        const auto* display = FindAsset(catalog, role, 0.005F);
        REQUIRE(display != nullptr);
        CHECK(display->header.vertexCount == 2U);
        CHECK(
            invisible_places::io::ResolveSceneDisplayDensityPayloadPath(
                display->filePath) == display->filePath);
    }
}

TEST_CASE(
    "Scene3 display-density cache rejects malformed untrusted metadata",
    "[pointcloud][density][cache]") {
    TemporaryDirectory temporary;
    const auto bundle = WriteSyntheticBundle(temporary.path);
    auto catalog = invisible_places::io::DiscoverAssets(bundle.dataRoot);

    {
        std::ofstream pointer{
            bundle.cacheRoot / "active-bundle.json",
            std::ios::trunc};
        pointer << nlohmann::json{
                       {"schema_version", "one"},
                       {"bundle_fingerprint", bundle.fingerprint},
                       {"manifest_sha256", std::string(64U, '0')},
                   }
                       .dump(2);
    }
    const auto wrongType =
        invisible_places::io::ActivateScene3DisplayDensityCache(
            bundle.cacheRoot,
            &catalog);
    CHECK_FALSE(wrongType.Activated());
    CHECK(
        wrongType.state ==
        invisible_places::io::SceneDisplayDensityCacheState::Rejected);

    nlohmann::json manifest;
    {
        std::ifstream input{bundle.manifestPath};
        input >> manifest;
    }
    manifest["roles"][0U]["source"]["mtime_ns"] =
        std::numeric_limits<std::uint64_t>::max();
    {
        std::ofstream output{bundle.manifestPath, std::ios::trunc};
        output << manifest.dump(2);
    }
    const auto manifestSha256 =
        invisible_places::io::ComputeSceneDisplayDensityFileSha256(
            bundle.manifestPath);
    REQUIRE(manifestSha256.has_value());
    {
        std::ofstream pointer{
            bundle.cacheRoot / "active-bundle.json",
            std::ios::trunc};
        pointer << nlohmann::json{
                       {"schema_version", 1U},
                       {"bundle_fingerprint", bundle.fingerprint},
                       {"manifest_sha256", manifestSha256.value()},
                   }
                       .dump(2);
    }
    const auto oversizedInteger =
        invisible_places::io::ActivateScene3DisplayDensityCache(
            bundle.cacheRoot,
            &catalog);
    CHECK_FALSE(oversizedInteger.Activated());
    CHECK(
        oversizedInteger.state ==
        invisible_places::io::SceneDisplayDensityCacheState::Rejected);
}

TEST_CASE(
    "Scene display-density SHA-256 matches the standard test vector",
    "[pointcloud][density][cache]") {
    CHECK(
        invisible_places::io::ComputeSceneDisplayDensitySha256("") ==
        "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
    CHECK(
        invisible_places::io::ComputeSceneDisplayDensitySha256("abc") ==
        "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
}

TEST_CASE(
    "Scene3 display-density pointer binds the exact manifest bytes",
    "[pointcloud][density][cache]") {
    TemporaryDirectory temporary;
    const auto bundle = WriteSyntheticBundle(temporary.path);
    {
        std::ofstream manifest{bundle.manifestPath, std::ios::app};
        manifest << '\n';
    }
    auto catalog = invisible_places::io::DiscoverAssets(bundle.dataRoot);

    const auto activation =
        invisible_places::io::ActivateScene3DisplayDensityCache(
            bundle.cacheRoot,
            &catalog);
    CHECK_FALSE(activation.Activated());
    CHECK(
        activation.state ==
        invisible_places::io::SceneDisplayDensityCacheState::Rejected);
    CHECK(
        activation.message.find("exact bundle manifest bytes") !=
        std::string::npos);
}

TEST_CASE(
    "Scene3 live cache rejects the experimental linear-light child",
    "[pointcloud][density][cache]") {
    TemporaryDirectory temporary;
    const auto bundle = WriteSyntheticBundle(
        temporary.path,
        "srgb-linear-light",
        kGeneratorQ1CentroidAlgorithmVersion,
        kGeneratorQ1CentroidPositionPolicy);
    auto catalog = invisible_places::io::DiscoverAssets(bundle.dataRoot);

    const auto activation =
        invisible_places::io::ActivateScene3DisplayDensityCache(
            bundle.cacheRoot,
            &catalog);
    CHECK_FALSE(activation.Activated());
    CHECK(
        activation.state ==
        invisible_places::io::SceneDisplayDensityCacheState::Rejected);
    CHECK(
        activation.message.find("prefilter policy") != std::string::npos);
}

TEST_CASE(
    "Scene3 live cache rejects a refined child built on a non-5mm grid",
    "[pointcloud][density][cache]") {
    TemporaryDirectory temporary;
    const auto bundle = WriteSyntheticBundle(
        temporary.path,
        "renderer-byte-mean",
        kGeneratorQ1CentroidAlgorithmVersion,
        kGeneratorQ1CentroidPositionPolicy,
        0.004);
    auto catalog = invisible_places::io::DiscoverAssets(bundle.dataRoot);

    const auto activation =
        invisible_places::io::ActivateScene3DisplayDensityCache(
            bundle.cacheRoot,
            &catalog);
    CHECK_FALSE(activation.Activated());
    CHECK(
        activation.state ==
        invisible_places::io::SceneDisplayDensityCacheState::Rejected);
    CHECK(
        activation.message.find("prefilter policy") !=
        std::string::npos);
}

TEST_CASE(
    "Scene3 display-density cache is optional and leaves canonical exports alone",
    "[pointcloud][density][cache]") {
    TemporaryDirectory temporary;
    const auto bundle = WriteSyntheticBundle(temporary.path);
    auto catalog = invisible_places::io::DiscoverAssets(bundle.dataRoot);

    const auto unavailable =
        invisible_places::io::ActivateScene3DisplayDensityCache(
            temporary.path / "missing-cache",
            &catalog);
    CHECK_FALSE(unavailable.Activated());
    CHECK(
        unavailable.state ==
        invisible_places::io::SceneDisplayDensityCacheState::Unavailable);
    for (const auto& sourcePath : bundle.sourcePaths) {
        CHECK(
            invisible_places::io::ResolveSceneDisplayDensityPayloadPath(
                sourcePath) == sourcePath);
    }
}

TEST_CASE(
    "Python builder bundle activates through the C++ cache contract",
    "[pointcloud][density][cache][integration]") {
    const char* fixtureRoot =
        std::getenv("INVISIBLE_PLACES_DISPLAY_DENSITY_FIXTURE_ROOT");
    if (fixtureRoot == nullptr || std::string_view{fixtureRoot}.empty()) {
        SKIP(
            "Set INVISIBLE_PLACES_DISPLAY_DENSITY_FIXTURE_ROOT to a tiny "
            "bundle generated by build_scene3_display_density_cache.py.");
    }
    const std::filesystem::path root{fixtureRoot};
    auto catalog = invisible_places::io::DiscoverAssets(root / "Data");
    const auto activation =
        invisible_places::io::ActivateScene3DisplayDensityCache(
            root / "cache",
            &catalog);
    INFO(activation.message);
    REQUIRE(activation.Activated());
    REQUIRE(activation.roles.size() == 3U);
    CHECK_FALSE(
        invisible_places::io::ActiveSceneDisplayDensityBundleFingerprint()
            .empty());
    for (const auto& role : activation.roles) {
        CHECK(
            invisible_places::io::ResolveSceneDisplayDensityPayloadPath(
                role.logicalDisplayPath) == role.cachedDisplayPath);
    }
    const auto groups =
        invisible_places::scene::BuildScenePointCloudGroups(catalog);
    REQUIRE(groups.size() == 1U);
    REQUIRE(groups.front().FindCompleteDisplayBundle(1'000U) != nullptr);
    REQUIRE(groups.front().FindCompleteDisplayBundle(5'000U) != nullptr);
}
