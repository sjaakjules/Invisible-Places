#include "io/SceneDisplayDensityCache.hpp"

#include "io/PlyHeader.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <mutex>
#include <optional>
#include <set>
#include <sstream>
#include <string_view>
#include <system_error>
#include <vector>

#if defined(__APPLE__) || defined(__linux__)
#include <sys/stat.h>
#endif
#if defined(__APPLE__)
// Hardware-accelerated SHA-256 (libSystem; no extra framework). The portable
// software implementation below stays the reference for other platforms and
// is far too slow for the ~2 GiB verified at every launch in a Debug build.
#include <CommonCrypto/CommonDigest.h>
#endif

namespace invisible_places::io {

namespace {

using nlohmann::json;

constexpr std::string_view kSceneName = "Scene3";
constexpr std::string_view kActiveBundleFileName = "active-bundle.json";
constexpr std::string_view kManifestFileName = "display-density-manifest.json";
constexpr std::uint64_t kMaximumJsonBytes = 1024U * 1024U;
constexpr float kCanonicalSpacingMeters = 0.001F;
constexpr float kDisplaySpacingMeters = 0.005F;
constexpr float kSpacingToleranceMeters = 1.0e-7F;
constexpr double kDisplayVoxelSizeMeters = 0.005;
constexpr std::array<std::string_view, 3U> kRequiredRoles{
    "ROCK",
    "SAND",
    "VEG",
};

[[nodiscard]] bool IsSupportedLivePositionPolicy(
    std::string_view algorithmId,
    std::uint64_t algorithmVersion,
    std::string_view positionPolicy) {
    if (algorithmId != kSceneDisplayDensityCacheAlgorithmId) {
        return false;
    }
    const bool legacyStableHash =
        algorithmVersion == kSceneDisplayDensityCacheLegacyAlgorithmVersion &&
        positionPolicy == kSceneDisplayDensityCacheLegacyPositionPolicy;
    const bool q1CentroidMedoid =
        algorithmVersion ==
            kSceneDisplayDensityCacheQ1CentroidMedoidAlgorithmVersion &&
        positionPolicy ==
            kSceneDisplayDensityCacheQ1CentroidMedoidPositionPolicy;
    return legacyStableHash || q1CentroidMedoid;
}

std::mutex gPayloadOverridesMutex;
std::map<std::string, std::filesystem::path> gPayloadOverrides;
std::string gActiveBundleFingerprint;

class SoftwareSha256 {
  public:
    void Update(const void* data, std::size_t size) {
        const auto* bytes = static_cast<const std::uint8_t*>(data);
        totalBytes_ += size;
        while (size > 0U) {
            const auto copied = std::min(size, block_.size() - blockSize_);
            std::memcpy(block_.data() + blockSize_, bytes, copied);
            blockSize_ += copied;
            bytes += copied;
            size -= copied;
            if (blockSize_ == block_.size()) {
                Transform(block_.data());
                blockSize_ = 0U;
            }
        }
    }

    [[nodiscard]] std::string Finish() {
        const auto bitCount = static_cast<std::uint64_t>(totalBytes_) * 8ULL;
        const std::uint8_t marker = 0x80U;
        Update(&marker, 1U);
        const std::uint8_t zero = 0U;
        while (blockSize_ != 56U) {
            Update(&zero, 1U);
        }
        std::array<std::uint8_t, 8U> encodedLength{};
        for (std::size_t index = 0U; index < encodedLength.size(); ++index) {
            encodedLength[encodedLength.size() - 1U - index] =
                static_cast<std::uint8_t>(bitCount >> (index * 8U));
        }
        Update(encodedLength.data(), encodedLength.size());

        std::ostringstream digest;
        digest << std::hex << std::setfill('0');
        for (const auto word : state_) {
            digest << std::setw(8) << word;
        }
        return digest.str();
    }

  private:
    static constexpr std::array<std::uint32_t, 64U> kRoundConstants{
        0x428a2f98U, 0x71374491U, 0xb5c0fbcfU, 0xe9b5dba5U,
        0x3956c25bU, 0x59f111f1U, 0x923f82a4U, 0xab1c5ed5U,
        0xd807aa98U, 0x12835b01U, 0x243185beU, 0x550c7dc3U,
        0x72be5d74U, 0x80deb1feU, 0x9bdc06a7U, 0xc19bf174U,
        0xe49b69c1U, 0xefbe4786U, 0x0fc19dc6U, 0x240ca1ccU,
        0x2de92c6fU, 0x4a7484aaU, 0x5cb0a9dcU, 0x76f988daU,
        0x983e5152U, 0xa831c66dU, 0xb00327c8U, 0xbf597fc7U,
        0xc6e00bf3U, 0xd5a79147U, 0x06ca6351U, 0x14292967U,
        0x27b70a85U, 0x2e1b2138U, 0x4d2c6dfcU, 0x53380d13U,
        0x650a7354U, 0x766a0abbU, 0x81c2c92eU, 0x92722c85U,
        0xa2bfe8a1U, 0xa81a664bU, 0xc24b8b70U, 0xc76c51a3U,
        0xd192e819U, 0xd6990624U, 0xf40e3585U, 0x106aa070U,
        0x19a4c116U, 0x1e376c08U, 0x2748774cU, 0x34b0bcb5U,
        0x391c0cb3U, 0x4ed8aa4aU, 0x5b9cca4fU, 0x682e6ff3U,
        0x748f82eeU, 0x78a5636fU, 0x84c87814U, 0x8cc70208U,
        0x90befffaU, 0xa4506cebU, 0xbef9a3f7U, 0xc67178f2U,
    };

    static std::uint32_t RotateRight(std::uint32_t value, unsigned amount) {
        return (value >> amount) | (value << (32U - amount));
    }

    void Transform(const std::uint8_t* block) {
        std::array<std::uint32_t, 64U> words{};
        for (std::size_t index = 0U; index < 16U; ++index) {
            const auto offset = index * 4U;
            words[index] =
                (static_cast<std::uint32_t>(block[offset]) << 24U) |
                (static_cast<std::uint32_t>(block[offset + 1U]) << 16U) |
                (static_cast<std::uint32_t>(block[offset + 2U]) << 8U) |
                static_cast<std::uint32_t>(block[offset + 3U]);
        }
        for (std::size_t index = 16U; index < words.size(); ++index) {
            const auto s0 = RotateRight(words[index - 15U], 7U) ^
                            RotateRight(words[index - 15U], 18U) ^
                            (words[index - 15U] >> 3U);
            const auto s1 = RotateRight(words[index - 2U], 17U) ^
                            RotateRight(words[index - 2U], 19U) ^
                            (words[index - 2U] >> 10U);
            words[index] = words[index - 16U] + s0 +
                           words[index - 7U] + s1;
        }

        auto a = state_[0U];
        auto b = state_[1U];
        auto c = state_[2U];
        auto d = state_[3U];
        auto e = state_[4U];
        auto f = state_[5U];
        auto g = state_[6U];
        auto h = state_[7U];
        for (std::size_t index = 0U; index < words.size(); ++index) {
            const auto sum1 = RotateRight(e, 6U) ^ RotateRight(e, 11U) ^
                              RotateRight(e, 25U);
            const auto choose = (e & f) ^ ((~e) & g);
            const auto temporary1 = h + sum1 + choose +
                                    kRoundConstants[index] + words[index];
            const auto sum0 = RotateRight(a, 2U) ^ RotateRight(a, 13U) ^
                              RotateRight(a, 22U);
            const auto majority = (a & b) ^ (a & c) ^ (b & c);
            const auto temporary2 = sum0 + majority;
            h = g;
            g = f;
            f = e;
            e = d + temporary1;
            d = c;
            c = b;
            b = a;
            a = temporary1 + temporary2;
        }
        state_[0U] += a;
        state_[1U] += b;
        state_[2U] += c;
        state_[3U] += d;
        state_[4U] += e;
        state_[5U] += f;
        state_[6U] += g;
        state_[7U] += h;
    }

    std::array<std::uint32_t, 8U> state_{
        0x6a09e667U,
        0xbb67ae85U,
        0x3c6ef372U,
        0xa54ff53aU,
        0x510e527fU,
        0x9b05688cU,
        0x1f83d9abU,
        0x5be0cd19U,
    };
    std::array<std::uint8_t, 64U> block_{};
    std::size_t blockSize_ = 0U;
    std::size_t totalBytes_ = 0U;
};

#if defined(__APPLE__)
class Sha256 {
  public:
    Sha256() { CC_SHA256_Init(&context_); }

    void Update(const void* data, std::size_t size) {
        const auto* bytes = static_cast<const std::uint8_t*>(data);
        while (size > 0U) {
            const auto chunk = static_cast<CC_LONG>(std::min<std::size_t>(
                size,
                static_cast<std::size_t>(std::numeric_limits<CC_LONG>::max())));
            CC_SHA256_Update(&context_, bytes, chunk);
            bytes += chunk;
            size -= chunk;
        }
    }

    [[nodiscard]] std::string Finish() {
        std::array<unsigned char, CC_SHA256_DIGEST_LENGTH> raw{};
        CC_SHA256_Final(raw.data(), &context_);
        std::ostringstream digest;
        digest << std::hex << std::setfill('0');
        for (const auto byte : raw) {
            digest << std::setw(2) << static_cast<unsigned int>(byte);
        }
        return digest.str();
    }

  private:
    CC_SHA256_CTX context_{};
};
#else
using Sha256 = SoftwareSha256;
#endif

std::optional<std::string> Sha256File(
    const std::filesystem::path& path,
    std::string* errorMessage) {
    std::ifstream input{path, std::ios::binary};
    if (!input.is_open()) {
        *errorMessage = "Could not open " + path.generic_string() +
                        " for SHA-256 verification.";
        return std::nullopt;
    }
    Sha256 digest;
    std::vector<char> buffer(8U * 1024U * 1024U);
    while (input.good()) {
        input.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
        const auto count = input.gcount();
        if (count > 0) {
            digest.Update(buffer.data(), static_cast<std::size_t>(count));
        }
    }
    if (!input.eof()) {
        *errorMessage = "Could not finish SHA-256 verification for " +
                        path.generic_string() + ".";
        return std::nullopt;
    }
    return digest.Finish();
}

std::string Sha256Text(std::string_view value) {
    Sha256 digest;
    digest.Update(value.data(), value.size());
    return digest.Finish();
}

std::string NormalizedAbsolutePathKey(const std::filesystem::path& path) {
    std::error_code error;
    auto normalized = std::filesystem::weakly_canonical(path, error);
    if (!error) {
        return normalized.generic_string();
    }
    error.clear();
    normalized = std::filesystem::absolute(path, error);
    return (error ? path : normalized).lexically_normal().generic_string();
}

void InstallPayloadOverrides(
    std::map<std::string, std::filesystem::path> overrides,
    std::string bundleFingerprint = {}) {
    std::scoped_lock lock(gPayloadOverridesMutex);
    gPayloadOverrides = std::move(overrides);
    gActiveBundleFingerprint = std::move(bundleFingerprint);
}

bool IsHexString(std::string_view value, std::size_t expectedSize) {
    return value.size() == expectedSize &&
           std::all_of(value.begin(), value.end(), [](const char character) {
               return std::isxdigit(
                          static_cast<unsigned char>(character)) != 0;
           });
}

bool IsHexDigest(std::string_view value) {
    return IsHexString(value, 64U);
}

std::optional<json> ReadBoundedJson(
    const std::filesystem::path& path,
    std::string* errorMessage) {
    std::error_code metadataError;
    const auto size = std::filesystem::file_size(path, metadataError);
    if (metadataError) {
        *errorMessage = "Could not stat " + path.generic_string() + ".";
        return std::nullopt;
    }
    if (size == 0U || size > kMaximumJsonBytes) {
        *errorMessage =
            path.generic_string() + " is empty or exceeds the 1 MiB limit.";
        return std::nullopt;
    }
    std::ifstream input{path};
    if (!input.is_open()) {
        *errorMessage = "Could not open " + path.generic_string() + ".";
        return std::nullopt;
    }
    try {
        json document;
        input >> document;
        if (!document.is_object()) {
            *errorMessage = path.generic_string() + " is not a JSON object.";
            return std::nullopt;
        }
        return document;
    } catch (const json::exception& error) {
        *errorMessage =
            "Could not parse " + path.generic_string() + ": " +
            error.what();
        return std::nullopt;
    }
}

std::optional<std::int64_t> FileMtimeNanoseconds(
    const std::filesystem::path& path) {
#if defined(__APPLE__) || defined(__linux__)
    struct stat metadata {};
    if (::stat(path.c_str(), &metadata) != 0) {
        return std::nullopt;
    }
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
    std::error_code error;
    const auto modified = std::filesystem::last_write_time(path, error);
    if (error) {
        return std::nullopt;
    }
    const auto systemModified =
        std::filesystem::file_time_type::clock::to_sys(modified);
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               systemModified.time_since_epoch())
        .count();
#endif
}

bool PathsReferToSameFile(
    const std::filesystem::path& left,
    const std::filesystem::path& right) {
    std::error_code equivalentError;
    if (std::filesystem::equivalent(left, right, equivalentError) &&
        !equivalentError) {
        return true;
    }
    return NormalizedAbsolutePathKey(left) ==
           NormalizedAbsolutePathKey(right);
}

bool PathIsInside(
    const std::filesystem::path& child,
    const std::filesystem::path& parent) {
    const auto relative = child.lexically_relative(parent);
    if (relative.empty() || relative.is_absolute()) {
        return false;
    }
    return std::none_of(
        relative.begin(),
        relative.end(),
        [](const auto& component) { return component == ".."; });
}

bool SameProperty(const PlyProperty& left, const PlyProperty& right) {
    return left.type == right.type &&
           left.name == right.name &&
           left.isList == right.isList &&
           left.listCountType == right.listCountType &&
           left.listValueType == right.listValueType;
}

bool SameVertexSchema(const PlyHeader& left, const PlyHeader& right) {
    return left.format == right.format &&
           left.properties.size() == right.properties.size() &&
           std::equal(
               left.properties.begin(),
               left.properties.end(),
               right.properties.begin(),
               SameProperty);
}

bool AssetMatches(
    const PointCloudAsset& asset,
    std::string_view role,
    float spacingMeters) {
    return asset.sceneGroupName == kSceneName &&
           asset.sceneRole == role &&
           std::abs(asset.inferredPointSpacingMeters - spacingMeters) <=
               kSpacingToleranceMeters;
}

PointCloudAsset* FindUniqueAsset(
    AssetCatalog* catalog,
    std::string_view role,
    float spacingMeters,
    std::string* errorMessage) {
    PointCloudAsset* match = nullptr;
    for (auto& asset : catalog->pointClouds) {
        if (!AssetMatches(asset, role, spacingMeters)) {
            continue;
        }
        if (match != nullptr) {
            std::ostringstream message;
            message << "Scene3 has multiple " << role << " "
                    << (spacingMeters * 1'000.0F)
                    << " mm assets; the cache cannot select one safely.";
            *errorMessage = message.str();
            return nullptr;
        }
        match = &asset;
    }
    if (match == nullptr) {
        std::ostringstream message;
        message << "Scene3 is missing its unique " << role << " "
                << (spacingMeters * 1'000.0F) << " mm asset.";
        *errorMessage = message.str();
    }
    return match;
}

bool ReadUnsigned(
    const json& object,
    std::string_view key,
    std::uint64_t* value) {
    const auto it = object.find(key);
    if (it == object.end() || !it->is_number_integer()) {
        return false;
    }
    try {
        if (it->is_number_unsigned()) {
            *value = it->get<std::uint64_t>();
            return true;
        }
        const auto signedValue = it->get<std::int64_t>();
        if (signedValue < 0) {
            return false;
        }
        *value = static_cast<std::uint64_t>(signedValue);
        return true;
    } catch (const json::exception&) {
        return false;
    }
}

bool ReadSigned(
    const json& object,
    std::string_view key,
    std::int64_t* value) {
    const auto it = object.find(key);
    if (it == object.end() || !it->is_number_integer()) {
        return false;
    }
    try {
        if (it->is_number_unsigned()) {
            const auto unsignedValue = it->get<std::uint64_t>();
            if (unsignedValue >
                static_cast<std::uint64_t>(
                    std::numeric_limits<std::int64_t>::max())) {
                return false;
            }
            *value = static_cast<std::int64_t>(unsignedValue);
            return true;
        }
        *value = it->get<std::int64_t>();
        return true;
    } catch (const json::exception&) {
        return false;
    }
}

bool ReadString(
    const json& object,
    std::string_view key,
    std::string* value) {
    const auto it = object.find(key);
    if (it == object.end() || !it->is_string()) {
        return false;
    }
    try {
        *value = it->get<std::string>();
        return true;
    } catch (const json::exception&) {
        return false;
    }
}

bool ReadBool(
    const json& object,
    std::string_view key,
    bool* value) {
    const auto it = object.find(key);
    if (it == object.end() || !it->is_boolean()) {
        return false;
    }
    *value = it->get<bool>();
    return true;
}

bool ReadDouble(
    const json& object,
    std::string_view key,
    double* value) {
    const auto it = object.find(key);
    if (it == object.end() || !it->is_number()) {
        return false;
    }
    try {
        *value = it->get<double>();
        return std::isfinite(*value);
    } catch (const json::exception&) {
        return false;
    }
}

std::string VertexSchemaSha256(const PlyHeader& header) {
    json properties = json::array();
    for (const auto& property : header.properties) {
        properties.push_back({
            {"type", property.type},
            {"name", property.name},
        });
    }
    return Sha256Text(
        json{
            {"format", header.format},
            {"vertex_properties", std::move(properties)},
        }
            .dump());
}

SceneDisplayDensityCacheActivation Reject(
    SceneDisplayDensityCacheActivation result,
    std::string message) {
    result.state = SceneDisplayDensityCacheState::Rejected;
    result.message = std::move(message);
    result.roles.clear();
    return result;
}

}  // namespace

std::filesystem::path ResolveSceneDisplayDensityPayloadPath(
    const std::filesystem::path& logicalPath) {
    std::scoped_lock lock(gPayloadOverridesMutex);
    const auto override =
        gPayloadOverrides.find(NormalizedAbsolutePathKey(logicalPath));
    return override == gPayloadOverrides.end()
               ? logicalPath
               : override->second;
}

std::string ActiveSceneDisplayDensityBundleFingerprint() {
    std::scoped_lock lock(gPayloadOverridesMutex);
    return gActiveBundleFingerprint;
}

void ClearSceneDisplayDensityCacheActivation() {
    InstallPayloadOverrides({});
}

SceneDisplayDensityCacheActivation ActivateScene3DisplayDensityCacheImpl(
    const std::filesystem::path& cacheRoot,
    AssetCatalog* catalog) {
    // A rejected revalidation must never leave an older process-local mapping
    // active. New redirects are published together only after every role has
    // passed validation.
    InstallPayloadOverrides({});

    SceneDisplayDensityCacheActivation result;
    result.cacheRoot = cacheRoot;
    if (catalog == nullptr) {
        return Reject(std::move(result), "Asset catalog is unavailable.");
    }

    const auto pointerPath = cacheRoot / kActiveBundleFileName;
    std::error_code pointerError;
    const auto pointerStatus =
        std::filesystem::symlink_status(pointerPath, pointerError);
    if (pointerStatus.type() == std::filesystem::file_type::not_found ||
        pointerError) {
        result.state = SceneDisplayDensityCacheState::Unavailable;
        result.message = "No active local Scene3 display-density cache.";
        return result;
    }
    if (pointerStatus.type() != std::filesystem::file_type::regular) {
        return Reject(
            std::move(result),
            "Active bundle pointer is not a regular local file.");
    }

    std::string readError;
    const auto pointer = ReadBoundedJson(pointerPath, &readError);
    if (!pointer.has_value()) {
        return Reject(std::move(result), std::move(readError));
    }
    std::uint64_t pointerSchema = 0U;
    if (!ReadUnsigned(pointer.value(), "schema_version", &pointerSchema) ||
        pointerSchema != kSceneDisplayDensityCacheSchemaVersion) {
        return Reject(
            std::move(result),
            "Active bundle pointer has an unsupported schema version.");
    }
    if (!ReadString(
            pointer.value(),
            "bundle_fingerprint",
            &result.bundleFingerprint) ||
        !IsHexDigest(result.bundleFingerprint)) {
        return Reject(
            std::move(result),
            "Active bundle pointer has no valid SHA-256 bundle fingerprint.");
    }
    std::string recordedManifestSha256;
    if (!ReadString(
            pointer.value(),
            "manifest_sha256",
            &recordedManifestSha256) ||
        !IsHexDigest(recordedManifestSha256)) {
        return Reject(
            std::move(result),
            "Active bundle pointer has no valid manifest SHA-256 binding.");
    }

    const auto bundlePath = cacheRoot / result.bundleFingerprint;
    const auto bundleStatus = std::filesystem::symlink_status(bundlePath);
    if (bundleStatus.type() != std::filesystem::file_type::directory) {
        return Reject(
            std::move(result),
            "Active bundle directory is missing or is a symbolic link.");
    }
    result.manifestPath = bundlePath / kManifestFileName;
    std::error_code manifestStatusError;
    const auto manifestStatus =
        std::filesystem::symlink_status(
            result.manifestPath,
            manifestStatusError);
    if (manifestStatusError ||
        manifestStatus.type() != std::filesystem::file_type::regular) {
        return Reject(
            std::move(result),
            "Bundle manifest is missing or is not a regular local file.");
    }
    const auto manifest = ReadBoundedJson(result.manifestPath, &readError);
    if (!manifest.has_value()) {
        return Reject(std::move(result), std::move(readError));
    }
    const auto actualManifestSha256 =
        Sha256File(result.manifestPath, &readError);
    if (!actualManifestSha256.has_value()) {
        return Reject(std::move(result), std::move(readError));
    }
    if (actualManifestSha256.value() != recordedManifestSha256) {
        return Reject(
            std::move(result),
            "Active pointer does not match the exact bundle manifest bytes.");
    }
    std::uint64_t manifestSchema = 0U;
    std::string manifestScene;
    std::string manifestBundleFingerprint;
    bool manifestComplete = false;
    if (!ReadUnsigned(*manifest, "schema_version", &manifestSchema) ||
        manifestSchema != kSceneDisplayDensityCacheSchemaVersion ||
        !ReadString(*manifest, "scene", &manifestScene) ||
        manifestScene != kSceneName ||
        !ReadBool(*manifest, "complete", &manifestComplete) ||
        !manifestComplete ||
        !ReadString(
            *manifest,
            "bundle_fingerprint",
            &manifestBundleFingerprint) ||
        manifestBundleFingerprint != result.bundleFingerprint) {
        return Reject(
            std::move(result),
            "Bundle manifest identity, scene, completeness, or schema is invalid.");
    }

    const auto algorithmIt = manifest->find("algorithm");
    std::uint64_t algorithmVersion = 0U;
    double voxelSizeMeters = 0.0;
    std::string seedHex;
    std::string rgbFilter;
    std::string apportionment;
    std::string positionPolicy;
    std::string cellGridOffset;
    std::string normalFilter;
    std::string categoricalFilter;
    std::string continuousFilter;
    if (algorithmIt == manifest->end() || !algorithmIt->is_object() ||
        !ReadString(*algorithmIt, "id", &result.algorithmId) ||
        !ReadUnsigned(*algorithmIt, "version", &algorithmVersion) ||
        !ReadString(*algorithmIt, "seed_hex", &seedHex) ||
        !IsHexString(seedHex, 16U) ||
        !ReadDouble(*algorithmIt, "voxel_size_m", &voxelSizeMeters) ||
        std::abs(voxelSizeMeters - kDisplayVoxelSizeMeters) > 1.0e-12 ||
        !ReadString(*algorithmIt, "rgb_filter", &rgbFilter) ||
        rgbFilter != "renderer-byte-mean" ||
        !ReadString(*algorithmIt, "apportionment", &apportionment) ||
        apportionment != "seeded-systematic-parent-population" ||
        !ReadString(*algorithmIt, "position_policy", &positionPolicy) ||
        !ReadString(*algorithmIt, "cell_grid_offset", &cellGridOffset) ||
        cellGridOffset != "half-voxel-xyz" ||
        !ReadString(*algorithmIt, "normal_filter", &normalFilter) ||
        normalFilter !=
            "hemisphere-aligned-normalized-mean-cosine-gate-0.5" ||
        !ReadString(
            *algorithmIt,
            "categorical_filter",
            &categoricalFilter) ||
        categoricalFilter != "scanid-mode-low-value-tie" ||
        !ReadString(*algorithmIt, "continuous_filter", &continuousFilter) ||
        continuousFilter != "finite-arithmetic-mean" ||
        !IsSupportedLivePositionPolicy(
            result.algorithmId,
            algorithmVersion,
            positionPolicy)) {
        return Reject(
            std::move(result),
            "Bundle algorithm or prefilter policy is unsupported for live 1 mm render parity.");
    }

    const auto rolesIt = manifest->find("roles");
    if (rolesIt == manifest->end() || !rolesIt->is_array() ||
        rolesIt->size() != kRequiredRoles.size()) {
        return Reject(
            std::move(result),
            "Bundle must contain exactly one ROCK, SAND, and VEG role.");
    }

    std::error_code canonicalError;
    const auto canonicalBundle =
        std::filesystem::canonical(bundlePath, canonicalError);
    if (canonicalError) {
        return Reject(
            std::move(result),
            "Could not resolve the active bundle directory.");
    }

    struct PendingOverlay {
        PointCloudAsset* displayAsset = nullptr;
        PlyHeader cachedHeader;
        std::filesystem::path cachedPath;
        SceneDisplayDensityCacheRoleProvenance provenance;
        std::uint64_t expectedSourceSize = 0U;
        std::int64_t expectedSourceMtime = 0;
        std::uint64_t expectedOutputSize = 0U;
        std::int64_t expectedOutputMtime = 0;
    };
    std::array<PendingOverlay, kRequiredRoles.size()> pending;
    std::array<json, kRequiredRoles.size()> fingerprintRoles;
    std::set<std::string> seenRoles;

    for (const auto& roleJson : *rolesIt) {
        if (!roleJson.is_object()) {
            return Reject(std::move(result), "Bundle role entry is not an object.");
        }
        std::string role;
        if (!ReadString(roleJson, "role", &role) ||
            std::find(kRequiredRoles.begin(), kRequiredRoles.end(), role) ==
                kRequiredRoles.end() ||
            !seenRoles.insert(role).second) {
            return Reject(
                std::move(result),
                "Bundle role names are missing, duplicated, or unsupported.");
        }
        const auto roleIndex = static_cast<std::size_t>(
            std::distance(
                kRequiredRoles.begin(),
                std::find(kRequiredRoles.begin(), kRequiredRoles.end(), role)));
        auto& overlay = pending[roleIndex];

        std::string assetError;
        auto* canonicalAsset = FindUniqueAsset(
            catalog,
            role,
            kCanonicalSpacingMeters,
            &assetError);
        if (canonicalAsset == nullptr) {
            return Reject(std::move(result), std::move(assetError));
        }
        overlay.displayAsset = FindUniqueAsset(
            catalog,
            role,
            kDisplaySpacingMeters,
            &assetError);
        if (overlay.displayAsset == nullptr) {
            return Reject(std::move(result), std::move(assetError));
        }

        const auto sourceIt = roleJson.find("source");
        const auto outputIt = roleJson.find("output");
        if (sourceIt == roleJson.end() || !sourceIt->is_object() ||
            outputIt == roleJson.end() || !outputIt->is_object()) {
            return Reject(
                std::move(result),
                "Bundle role is missing its source or output identity.");
        }

        std::string sourcePathText;
        std::string sourceSha256;
        std::string sourceSchemaSha256;
        std::uint64_t sourceSize = 0U;
        std::uint64_t sourceVertexCount = 0U;
        std::int64_t sourceMtime = 0;
        if (!ReadString(*sourceIt, "path", &sourcePathText) ||
            !ReadString(*sourceIt, "sha256", &sourceSha256) ||
            !ReadString(*sourceIt, "schema_sha256", &sourceSchemaSha256) ||
            !ReadUnsigned(*sourceIt, "size_bytes", &sourceSize) ||
            !ReadUnsigned(*sourceIt, "vertex_count", &sourceVertexCount) ||
            !ReadSigned(*sourceIt, "mtime_ns", &sourceMtime) ||
            !IsHexDigest(sourceSha256) ||
            !IsHexDigest(sourceSchemaSha256)) {
            return Reject(
                std::move(result),
                "Bundle source identity is incomplete or malformed.");
        }
        const std::filesystem::path recordedSource{sourcePathText};
        if (!recordedSource.is_absolute() ||
            !PathsReferToSameFile(recordedSource, canonicalAsset->filePath)) {
            return Reject(
                std::move(result),
                "Bundle source path does not match the discovered canonical " +
                    role + " 1 mm asset.");
        }
        std::error_code sourceMetadataError;
        const auto currentSourceSize =
            std::filesystem::file_size(canonicalAsset->filePath, sourceMetadataError);
        const auto currentSourceMtime =
            FileMtimeNanoseconds(canonicalAsset->filePath);
        if (sourceMetadataError || !currentSourceMtime.has_value() ||
            currentSourceSize != sourceSize ||
            currentSourceMtime.value() != sourceMtime ||
            canonicalAsset->header.vertexCount != sourceVertexCount) {
            return Reject(
                std::move(result),
                "Canonical " + role +
                    " 1 mm source identity changed after the cache was built.");
        }

        std::string outputFileText;
        std::string outputSha256;
        std::string outputSchemaSha256;
        std::uint64_t outputSize = 0U;
        std::uint64_t outputVertexCount = 0U;
        std::uint64_t requestedPointCount = 0U;
        std::int64_t outputMtime = 0;
        if (!ReadString(*outputIt, "file", &outputFileText) ||
            !ReadString(*outputIt, "sha256", &outputSha256) ||
            !ReadString(*outputIt, "schema_sha256", &outputSchemaSha256) ||
            !ReadUnsigned(*outputIt, "size_bytes", &outputSize) ||
            !ReadUnsigned(*outputIt, "vertex_count", &outputVertexCount) ||
            !ReadSigned(*outputIt, "mtime_ns", &outputMtime) ||
            !ReadUnsigned(
                roleJson,
                "requested_point_count",
                &requestedPointCount) ||
            !IsHexDigest(outputSha256) ||
            !IsHexDigest(outputSchemaSha256) ||
            sourceSchemaSha256 != outputSchemaSha256 ||
            outputVertexCount == 0U ||
            outputVertexCount != requestedPointCount) {
            return Reject(
                std::move(result),
                "Bundle output identity is incomplete, malformed, or schema-incompatible.");
        }

        const std::filesystem::path relativeOutput{outputFileText};
        if (relativeOutput.empty() || relativeOutput.is_absolute() ||
            relativeOutput.extension() != ".ply") {
            return Reject(
                std::move(result),
                "Bundle output path must be a relative PLY path.");
        }
        const auto outputStatus =
            std::filesystem::symlink_status(bundlePath / relativeOutput);
        if (outputStatus.type() != std::filesystem::file_type::regular) {
            return Reject(
                std::move(result),
                "Bundle output is missing or is a symbolic link.");
        }
        canonicalError.clear();
        overlay.cachedPath =
            std::filesystem::canonical(bundlePath / relativeOutput, canonicalError);
        if (canonicalError ||
            !PathIsInside(overlay.cachedPath, canonicalBundle) ||
            PathsReferToSameFile(
                overlay.cachedPath,
                overlay.displayAsset->filePath) ||
            PathsReferToSameFile(
                overlay.cachedPath,
                canonicalAsset->filePath)) {
            return Reject(
                std::move(result),
                "Bundle output resolves outside its immutable bundle directory or aliases a shared source.");
        }
        if (InferPointCloudSceneRoleFromName(
                overlay.cachedPath.stem().string()) != role ||
            std::abs(
                InferPointSpacingMetersFromName(
                    overlay.cachedPath.stem().string()) -
                kDisplaySpacingMeters) > kSpacingToleranceMeters) {
            return Reject(
                std::move(result),
                "Bundle output filename does not identify its role and 5 mm display density.");
        }

        std::error_code outputMetadataError;
        const auto currentOutputSize =
            std::filesystem::file_size(overlay.cachedPath, outputMetadataError);
        const auto currentOutputMtime =
            FileMtimeNanoseconds(overlay.cachedPath);
        const auto cachedHeader = ParsePlyHeader(overlay.cachedPath);
        if (outputMetadataError || !currentOutputMtime.has_value() ||
            currentOutputSize != outputSize ||
            currentOutputMtime.value() != outputMtime ||
            !cachedHeader.success ||
            cachedHeader.header.vertexCount != outputVertexCount ||
            !cachedHeader.header.LooksLikePointCloud() ||
            !SameVertexSchema(canonicalAsset->header, cachedHeader.header) ||
            VertexSchemaSha256(canonicalAsset->header) !=
                sourceSchemaSha256 ||
            VertexSchemaSha256(cachedHeader.header) !=
                outputSchemaSha256) {
            return Reject(
                std::move(result),
                "Cached " + role +
                    " output changed or no longer matches its canonical PLY schema.");
        }
        const auto actualOutputSha256 =
            Sha256File(overlay.cachedPath, &readError);
        if (!actualOutputSha256.has_value()) {
            return Reject(std::move(result), std::move(readError));
        }
        const auto outputSizeAfterHash =
            std::filesystem::file_size(
                overlay.cachedPath,
                outputMetadataError);
        const auto outputMtimeAfterHash =
            FileMtimeNanoseconds(overlay.cachedPath);
        if (actualOutputSha256.value() != outputSha256 ||
            outputMetadataError || !outputMtimeAfterHash.has_value() ||
            outputSizeAfterHash != outputSize ||
            outputMtimeAfterHash.value() != outputMtime) {
            return Reject(
                std::move(result),
                "Cached " + role +
                    " output failed full SHA-256 verification or changed while being verified.");
        }
        overlay.cachedHeader = cachedHeader.header;
        overlay.expectedSourceSize = sourceSize;
        overlay.expectedSourceMtime = sourceMtime;
        overlay.expectedOutputSize = outputSize;
        overlay.expectedOutputMtime = outputMtime;
        overlay.provenance = {
            .role = role,
            .canonicalSourcePath = canonicalAsset->filePath,
            .logicalDisplayPath = overlay.displayAsset->filePath,
            .cachedDisplayPath = overlay.cachedPath,
            .sourceSha256 = sourceSha256,
            .outputSha256 = outputSha256,
            .outputPointCount = outputVertexCount,
        };
        fingerprintRoles[roleIndex] = {
            {"role", role},
            {"requested_point_count", requestedPointCount},
            {"source_sha256", sourceSha256},
            {"source_schema_sha256", sourceSchemaSha256},
            {"output_sha256", outputSha256},
            {"output_schema_sha256", outputSchemaSha256},
        };
    }

    if (seenRoles.size() != kRequiredRoles.size()) {
        return Reject(
            std::move(result),
            "Bundle is not a complete ROCK/SAND/VEG transaction.");
    }

    json fingerprintRoleArray = json::array();
    for (auto& role : fingerprintRoles) {
        fingerprintRoleArray.push_back(std::move(role));
    }
    const auto computedBundleFingerprint = Sha256Text(
        json{
            {"schema_version", kSceneDisplayDensityCacheSchemaVersion},
            {"algorithm", *algorithmIt},
            {"scene", kSceneName},
            {"roles", std::move(fingerprintRoleArray)},
        }
            .dump());
    if (computedBundleFingerprint != result.bundleFingerprint) {
        return Reject(
            std::move(result),
            "Bundle fingerprint does not match its canonical manifest identity.");
    }

    // Full output hashing can take several seconds. Recheck all six file
    // identities together afterwards so an external sync or partial rebuild
    // during validation cannot publish a mixed-time transaction.
    for (const auto& overlay : pending) {
        std::error_code metadataError;
        const auto sourceSize = std::filesystem::file_size(
            overlay.provenance.canonicalSourcePath,
            metadataError);
        const auto sourceMtime = FileMtimeNanoseconds(
            overlay.provenance.canonicalSourcePath);
        if (metadataError || !sourceMtime.has_value() ||
            sourceSize != overlay.expectedSourceSize ||
            sourceMtime.value() != overlay.expectedSourceMtime) {
            return Reject(
                std::move(result),
                "A canonical 1 mm source changed while the local bundle was being verified.");
        }
        metadataError.clear();
        const auto outputSize = std::filesystem::file_size(
            overlay.cachedPath,
            metadataError);
        const auto outputMtime = FileMtimeNanoseconds(overlay.cachedPath);
        if (metadataError || !outputMtime.has_value() ||
            outputSize != overlay.expectedOutputSize ||
            outputMtime.value() != overlay.expectedOutputMtime) {
            return Reject(
                std::move(result),
                "A cached 5 mm output changed while the complete bundle was being verified.");
        }
    }

    std::map<std::string, std::filesystem::path> overrides;
    result.roles.reserve(pending.size());
    for (auto& overlay : pending) {
        overrides.emplace(
            NormalizedAbsolutePathKey(overlay.displayAsset->filePath),
            overlay.cachedPath);
        overlay.displayAsset->header = std::move(overlay.cachedHeader);
        result.roles.push_back(std::move(overlay.provenance));
    }
    InstallPayloadOverrides(
        std::move(overrides),
        result.bundleFingerprint);

    result.state = SceneDisplayDensityCacheState::Activated;
    result.message =
        "Activated deterministic local Scene3 ROCK/SAND/VEG 5 mm display bundle " +
        result.bundleFingerprint.substr(0U, 12U) +
        " after full local output SHA-256 verification; canonical 1 mm/export sources remain unchanged.";
    return result;
}

SceneDisplayDensityCacheActivation ActivateScene3DisplayDensityCache(
    const std::filesystem::path& cacheRoot,
    AssetCatalog* catalog) {
    try {
        return ActivateScene3DisplayDensityCacheImpl(cacheRoot, catalog);
    } catch (const std::exception& error) {
        // Cache metadata is local but still untrusted input. A malformed path,
        // invalid UTF-8 string, oversized number, or filesystem race must only
        // disable the optional cache; it must never prevent the app from
        // falling back to its shared display files.
        InstallPayloadOverrides({});
        SceneDisplayDensityCacheActivation result;
        result.state = SceneDisplayDensityCacheState::Rejected;
        result.cacheRoot = cacheRoot;
        result.message =
            "Display-density cache validation failed safely: " +
            std::string{error.what()};
        return result;
    } catch (...) {
        InstallPayloadOverrides({});
        SceneDisplayDensityCacheActivation result;
        result.state = SceneDisplayDensityCacheState::Rejected;
        result.cacheRoot = cacheRoot;
        result.message =
            "Display-density cache validation failed safely with an unknown error.";
        return result;
    }
}

std::string ComputeSceneDisplayDensitySha256(std::string_view value) {
    return Sha256Text(value);
}

std::optional<std::string> ComputeSceneDisplayDensityFileSha256(
    const std::filesystem::path& path,
    std::string* errorMessage) {
    std::string localError;
    auto result = Sha256File(path, &localError);
    if (errorMessage != nullptr) {
        *errorMessage = std::move(localError);
    }
    return result;
}

}  // namespace invisible_places::io
