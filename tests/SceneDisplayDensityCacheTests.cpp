#include "io/AssetDiscovery.hpp"
#include "io/PointCloudData.hpp"
#include "io/SceneDisplayDensityCache.hpp"
#include "scene/PointCloudVariants.hpp"

#include <catch2/catch_test_macros.hpp>

#include <nlohmann/json.hpp>

#include <array>
#include <algorithm>
#include <atomic>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

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
    std::array<std::filesystem::path, 3U> fineToCoarsePaths;
    std::array<std::filesystem::path, 3U> fineWeightPaths;
    std::array<std::filesystem::path, 3U> coarseWeightPaths;
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
        nlohmann::json roleEntry{
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
        };
        nlohmann::json fingerprintRole{
            {"role", role},
            {"requested_point_count", 3U + index},
            {"source_sha256", sourceSha256.value()},
            {"source_schema_sha256", schemaDigest},
            {"output_sha256", outputSha256.value()},
            {"output_schema_sha256", schemaDigest},
        };
        if (algorithmVersion == invisible_places::io::
                                    kSceneDisplayDensityCacheSurfaceAnalysisAlgorithmVersion) {
            const auto analysisRoot = stagingRoot / "Analysis";
            std::filesystem::create_directories(analysisRoot);
            bundle.fineToCoarsePaths[index] =
                analysisRoot / ("Site3-" + role + "-1mm-to-5mm.u32");
            const auto parentCountPath =
                analysisRoot / ("Site3-" + role + "-5mm-parent-count.u32");
            bundle.fineWeightPaths[index] =
                analysisRoot / ("Site3-" + role + "-1mm-stability.u32");
            bundle.coarseWeightPaths[index] =
                analysisRoot / ("Site3-" + role + "-5mm-stability.u32");
            const auto writeU32 = [](const std::filesystem::path& path,
                                     const std::vector<std::uint32_t>& values) {
                std::ofstream output{
                    path,
                    std::ios::binary | std::ios::trunc};
                REQUIRE(output.is_open());
                output.write(
                    reinterpret_cast<const char*>(values.data()),
                    static_cast<std::streamsize>(
                        values.size() * sizeof(std::uint32_t)));
            };
            const std::size_t fineCount = 5U + index;
            const std::size_t coarseCount = 3U + index;
            std::vector<std::uint32_t> links(fineCount);
            std::vector<std::uint32_t> fineWeights(fineCount);
            std::vector<std::uint32_t> coarseWeights(coarseCount);
            std::vector<std::uint32_t> parentCounts(coarseCount, 0U);
            for (std::size_t point = 0U; point < fineCount; ++point) {
                links[point] = static_cast<std::uint32_t>(
                    point % coarseCount);
                fineWeights[point] =
                    0x10203040U + static_cast<std::uint32_t>(point);
                ++parentCounts[links[point]];
            }
            for (std::size_t point = 0U; point < coarseCount; ++point) {
                coarseWeights[point] =
                    0x50607080U + static_cast<std::uint32_t>(point);
            }
            writeU32(bundle.fineToCoarsePaths[index], links);
            writeU32(parentCountPath, parentCounts);
            writeU32(bundle.fineWeightPaths[index], fineWeights);
            writeU32(bundle.coarseWeightPaths[index], coarseWeights);
            const auto proof = [&](const std::filesystem::path& path,
                                   std::size_t count) {
                const auto sha = invisible_places::io::
                    ComputeSceneDisplayDensityFileSha256(path);
                REQUIRE(sha.has_value());
                return nlohmann::json{
                    {"file",
                     std::filesystem::relative(path, stagingRoot)
                         .generic_string()},
                    {"size_bytes", std::filesystem::file_size(path)},
                    {"mtime_ns", MtimeNanoseconds(path)},
                    {"sha256", sha.value()},
                    {"count", count},
                    {"encoding", "little-endian-uint32"},
                };
            };
            const auto fineToCoarseProof =
                proof(bundle.fineToCoarsePaths[index], fineCount);
            const auto parentCountProof =
                proof(parentCountPath, coarseCount);
            const auto fineWeightProof =
                proof(bundle.fineWeightPaths[index], fineCount);
            const auto coarseWeightProof =
                proof(bundle.coarseWeightPaths[index], coarseCount);
            roleEntry["analysis"] = {
                {"schema_version", 1U},
                {"fine_to_coarse", fineToCoarseProof},
                {"coarse_parent_count", parentCountProof},
                {"fine_stability_weights", fineWeightProof},
                {"coarse_stability_weights", coarseWeightProof},
            };
            fingerprintRole["analysis_schema_version"] = 1U;
            fingerprintRole["fine_to_coarse_sha256"] =
                fineToCoarseProof.at("sha256");
            fingerprintRole["coarse_parent_count_sha256"] =
                parentCountProof.at("sha256");
            fingerprintRole["fine_stability_weights_sha256"] =
                fineWeightProof.at("sha256");
            fingerprintRole["coarse_stability_weights_sha256"] =
                coarseWeightProof.at("sha256");
        }
        roleEntries.push_back(std::move(roleEntry));
        fingerprintRoles.push_back(std::move(fingerprintRole));
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
        if (!bundle.fineToCoarsePaths[index].empty()) {
            bundle.fineToCoarsePaths[index] =
                bundleRoot / "Analysis" /
                bundle.fineToCoarsePaths[index].filename();
            bundle.fineWeightPaths[index] =
                bundleRoot / "Analysis" /
                bundle.fineWeightPaths[index].filename();
            bundle.coarseWeightPaths[index] =
                bundleRoot / "Analysis" /
                bundle.coarseWeightPaths[index].filename();
        }
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
    "Scene3 display-density cache accepts circular-field algorithm v3",
    "[pointcloud][density][cache]") {
    TemporaryDirectory temporary;
    const auto bundle = WriteSyntheticBundle(
        temporary.path,
        "renderer-byte-mean",
        invisible_places::io::
            kSceneDisplayDensityCacheCircularFieldAlgorithmVersion,
        kGeneratorQ1CentroidPositionPolicy);
    auto catalog = invisible_places::io::DiscoverAssets(bundle.dataRoot);

    const auto activation =
        invisible_places::io::ActivateScene3DisplayDensityCache(
            bundle.cacheRoot,
            &catalog);
    INFO(activation.message);
    REQUIRE(activation.Activated());
    CHECK(activation.bundleFingerprint == bundle.fingerprint);
}

TEST_CASE(
    "Scene3 surface-analysis bundle supplies exact fine and coarse sidecars",
    "[pointcloud][density][cache][surface-stability]") {
    TemporaryDirectory temporary;
    const auto bundle = WriteSyntheticBundle(
        temporary.path,
        "renderer-byte-mean",
        invisible_places::io::
            kSceneDisplayDensityCacheSurfaceAnalysisAlgorithmVersion,
        kGeneratorQ1CentroidPositionPolicy);
    auto catalog = invisible_places::io::DiscoverAssets(bundle.dataRoot);

    const auto activation =
        invisible_places::io::ActivateScene3DisplayDensityCache(
            bundle.cacheRoot,
            &catalog);
    INFO(activation.message);
    REQUIRE(activation.Activated());
    REQUIRE(activation.roles.size() == 3U);
    CHECK_FALSE(activation.roles[0U].fineToCoarseLinkPath.empty());
    CHECK_FALSE(activation.roles[0U].fineStabilityWeightsPath.empty());
    CHECK_FALSE(activation.roles[0U].coarseStabilityWeightsPath.empty());

    const auto coarse = invisible_places::io::LoadPointCloud(
        bundle.logicalDisplayPaths[0U]);
    REQUIRE(coarse.success);
    REQUIRE(coarse.cloud.PointCount() == 3U);
    const auto coarseField = std::find_if(
        coarse.cloud.scalarFields.begin(),
        coarse.cloud.scalarFields.end(),
        [](const auto& field) {
            return field.name == invisible_places::io::
                                     kPointCloudSurfaceStabilityPackedFieldName;
        });
    REQUIRE(coarseField != coarse.cloud.scalarFields.end());
    const auto coarseSlot = static_cast<std::size_t>(
        std::distance(coarse.cloud.scalarFields.begin(), coarseField));
    CHECK(
        std::bit_cast<std::uint32_t>(
            coarse.cloud.scalarFieldValues[
                coarse.cloud.ScalarFieldValueIndex(coarseSlot, 0U)]) ==
        0x50607080U);

    const auto fine = invisible_places::io::LoadPointCloud(
        bundle.sourcePaths[0U]);
    REQUIRE(fine.success);
    REQUIRE(fine.cloud.PointCount() == 5U);
    const auto fineField = std::find_if(
        fine.cloud.scalarFields.begin(),
        fine.cloud.scalarFields.end(),
        [](const auto& field) {
            return field.name == invisible_places::io::
                                     kPointCloudSurfaceStabilityPackedFieldName;
        });
    REQUIRE(fineField != fine.cloud.scalarFields.end());
    const auto fineSlot = static_cast<std::size_t>(
        std::distance(fine.cloud.scalarFields.begin(), fineField));
    CHECK(
        std::bit_cast<std::uint32_t>(
            fine.cloud.scalarFieldValues[
                fine.cloud.ScalarFieldValueIndex(fineSlot, 4U)]) ==
        0x10203044U);

    std::vector<std::uint32_t> links;
    REQUIRE(
        invisible_places::io::ReadSceneDisplayDensityFineToCoarseLinks(
            bundle.sourcePaths[0U],
            {},
            &links));
    CHECK(links == std::vector<std::uint32_t>{0U, 1U, 2U, 0U, 1U});

    // The subset/aHQ path passes scattered, unordered source indices (aHQ
    // blocks are Morton-shuffled) and gathers through the resident weights
    // copy. Values, the hidden authoring flag, and the availability contract
    // must match the complete-load column exactly.
    const auto packedAt = [&](std::size_t point) {
        return std::bit_cast<std::uint32_t>(
            fine.cloud.scalarFieldValues[
                fine.cloud.ScalarFieldValueIndex(fineSlot, point)]);
    };
    invisible_places::io::LoadedPointCloud subsetCloud;
    subsetCloud.positions.resize(3U);
    const std::vector<std::uint32_t> shuffledIndices{4U, 0U, 3U};
    REQUIRE(
        invisible_places::io::AppendSceneDisplayDensitySurfaceWeights(
            bundle.sourcePaths[0U],
            shuffledIndices,
            &subsetCloud));
    REQUIRE(subsetCloud.scalarFields.size() == 1U);
    const auto& appendedField = subsetCloud.scalarFields.front();
    CHECK(
        appendedField.name ==
        invisible_places::io::kPointCloudSurfaceStabilityPackedFieldName);
    CHECK_FALSE(appendedField.authoringVisible);
    CHECK(appendedField.sourceIndex == -1);
    CHECK(subsetCloud.availableScalarFields.empty());
    REQUIRE(subsetCloud.scalarFieldValues.size() == shuffledIndices.size());
    for (std::size_t point = 0U; point < shuffledIndices.size(); ++point) {
        CHECK(
            std::bit_cast<std::uint32_t>(
                subsetCloud.scalarFieldValues[
                    subsetCloud.ScalarFieldValueIndex(0U, point)]) ==
            packedAt(shuffledIndices[point]));
    }

    // The links read accepts the same subset spans.
    std::vector<std::uint32_t> subsetLinks;
    REQUIRE(
        invisible_places::io::ReadSceneDisplayDensityFineToCoarseLinks(
            bundle.sourcePaths[0U],
            shuffledIndices,
            &subsetLinks));
    CHECK(subsetLinks == std::vector<std::uint32_t>{1U, 0U, 0U});

    // An out-of-range source index refuses the append outright and leaves
    // the cloud untouched rather than mis-indexing weights.
    invisible_places::io::LoadedPointCloud rejectedCloud;
    rejectedCloud.positions.resize(1U);
    const std::vector<std::uint32_t> outOfRange{5U};
    CHECK_FALSE(
        invisible_places::io::AppendSceneDisplayDensitySurfaceWeights(
            bundle.sourcePaths[0U],
            outOfRange,
            &rejectedCloud));
    CHECK(rejectedCloud.scalarFields.empty());
    CHECK(rejectedCloud.scalarFieldValues.empty());

    // A source with no registered sidecar is a quiet no-op: no column, no
    // failure side effects.
    invisible_places::io::LoadedPointCloud unrelatedCloud;
    unrelatedCloud.positions.resize(1U);
    CHECK_FALSE(
        invisible_places::io::AppendSceneDisplayDensitySurfaceWeights(
            temporary.path / "unregistered.ply",
            {},
            &unrelatedCloud));
    CHECK(unrelatedCloud.scalarFields.empty());
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
            5U,
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
