#include "app/LinkedHighQualityPreview.hpp"

#include "io/SceneDisplayDensityCache.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <limits>
#include <system_error>

#include <glm/vec4.hpp>

namespace invisible_places::app {

namespace {

void HashBytes(
    std::uint64_t* hash,
    const void* bytes,
    std::size_t byteCount) {
    constexpr std::uint64_t kFnvPrime = 1099511628211ULL;
    const auto* data = static_cast<const unsigned char*>(bytes);
    for (std::size_t index = 0U; index < byteCount; ++index) {
        *hash ^= static_cast<std::uint64_t>(data[index]);
        *hash *= kFnvPrime;
    }
}

template <typename T>
void HashValue(std::uint64_t* hash, const T& value) {
    HashBytes(hash, &value, sizeof(value));
}

}  // namespace

std::uint32_t SanitizeLinkedHqPatchSpacing(std::uint32_t spacingMicrometres) {
    std::uint32_t best = kLinkedHqPatchSpacingChoicesMicrometres.front();
    for (const auto choice : kLinkedHqPatchSpacingChoicesMicrometres) {
        if (choice <= spacingMicrometres) {
            best = choice;
        }
    }
    return best;
}

float LinkedHqPatchKeepFraction(std::uint32_t spacingMicrometres) {
    const auto spacing = SanitizeLinkedHqPatchSpacing(spacingMicrometres);
    const float ratio = static_cast<float>(spacing) /
        static_cast<float>(kLinkedHqPatchSpacingChoicesMicrometres.front());
    return 1.0F / std::max(1.0F, ratio * ratio);
}

bool LinkedHqFrustumUnion::Contains(
    const invisible_places::io::Float3& point) const {
    const float safeBorder = std::max(0.0F, borderFraction);
    const float lateralLimit = 1.0F + (2.0F * safeBorder);
    for (const auto& viewProjection : midpointViewProjections) {
        const glm::vec4 clip = viewProjection * glm::vec4{
            point.x,
            point.y,
            point.z,
            1.0F,
        };
        if (!std::isfinite(clip.x) || !std::isfinite(clip.y) ||
            !std::isfinite(clip.z) || !std::isfinite(clip.w) ||
            clip.w <= 1.0e-7F) {
            continue;
        }
        const float paddedW = lateralLimit * clip.w;
        if (clip.x >= -paddedW && clip.x <= paddedW &&
            clip.y >= -paddedW && clip.y <= paddedW &&
            clip.z >= -clip.w && clip.z <= clip.w) {
            return true;
        }
    }
    return false;
}

LinkedHqFrustumUnion BuildAnimationHqFrustumUnion(
    std::span<const glm::mat4> midpointViewProjections,
    float borderFraction) {
    LinkedHqFrustumUnion result;
    result.borderFraction = std::max(0.0F, borderFraction);
    if (midpointViewProjections.empty()) {
        return result;
    }
    result.midpointViewProjections[0U] = midpointViewProjections[0U];
    result.midpointViewProjections[1U] =
        midpointViewProjections.size() > 1U
            ? midpointViewProjections[1U]
            : midpointViewProjections[0U];
    return result;
}

LinkedHqIndexPartition PartitionLinkedHqIndices(
    std::span<const invisible_places::io::Float3> positions,
    const LinkedHqFrustumUnion& frustumUnion,
    std::stop_token stopToken,
    const invisible_places::io::PointCloudLoadProgress& progress) {
    LinkedHqIndexPartition partition;
    if (positions.size() >
        static_cast<std::size_t>(
            std::numeric_limits<std::uint32_t>::max())) {
        return partition;
    }
    partition.inside.reserve(positions.size() / 8U);
    partition.outside.reserve(positions.size());
    if (progress) {
        progress(0U, positions.size());
    }
    for (std::size_t index = 0U; index < positions.size(); ++index) {
        if ((index & 4095U) == 0U && stopToken.stop_requested()) {
            partition.cancelled = true;
            if (progress) {
                progress(index, positions.size());
            }
            return partition;
        }
        auto& destination = frustumUnion.Contains(positions[index])
                                ? partition.inside
                                : partition.outside;
        destination.push_back(static_cast<std::uint32_t>(index));
        if (progress && (index & 65535U) == 65535U) {
            progress(index + 1U, positions.size());
        }
    }
    if (progress) {
        progress(positions.size(), positions.size());
    }
    partition.outside.shrink_to_fit();
    return partition;
}

bool TryFingerprintLinkedHqSource(
    const std::filesystem::path& path,
    LinkedHqSourceFingerprint* fingerprint,
    std::string* errorMessage) {
    if (fingerprint == nullptr) {
        if (errorMessage != nullptr) {
            *errorMessage = "HQ source fingerprint output is null.";
        }
        return false;
    }
    const auto payloadPath =
        invisible_places::io::ResolveSceneDisplayDensityPayloadPath(path);
    std::error_code error;
    const auto byteSize = std::filesystem::file_size(payloadPath, error);
    if (error) {
        if (errorMessage != nullptr) {
            *errorMessage =
                "Unable to read HQ source size: " + error.message();
        }
        return false;
    }
    const auto writeTime =
        std::filesystem::last_write_time(payloadPath, error);
    if (error) {
        if (errorMessage != nullptr) {
            *errorMessage =
                "Unable to read HQ source timestamp: " + error.message();
        }
        return false;
    }
    *fingerprint = {
        .path = std::filesystem::weakly_canonical(payloadPath, error),
        .byteSize = byteSize,
        .writeTimeTicks = static_cast<std::int64_t>(
            writeTime.time_since_epoch().count()),
    };
    if (error) {
        fingerprint->path = payloadPath.lexically_normal();
    }
    return true;
}

std::uint64_t BuildLinkedHqSelectionFingerprint(
    std::string_view pairKey,
    const LinkedHqFrustumUnion& frustumUnion,
    std::span<const LinkedHqSourceFingerprint> sources) {
    std::uint64_t hash = 1469598103934665603ULL;
    HashBytes(&hash, pairKey.data(), pairKey.size());
    HashValue(&hash, frustumUnion.borderFraction);
    for (const auto& matrix : frustumUnion.midpointViewProjections) {
        for (std::size_t column = 0U; column < 4U; ++column) {
            for (std::size_t row = 0U; row < 4U; ++row) {
                HashValue(&hash, matrix[column][row]);
            }
        }
    }
    for (const auto& source : sources) {
        const auto path = source.path.generic_string();
        HashBytes(&hash, path.data(), path.size());
        HashValue(&hash, source.byteSize);
        HashValue(&hash, source.writeTimeTicks);
    }
    return hash;
}

const char* LinkedHqPreparationStageName(
    LinkedHqPreparationStage stage) {
    switch (stage) {
        case LinkedHqPreparationStage::Waiting:
            return "Waiting";
        case LinkedHqPreparationStage::Scanning:
            return "Scanning";
        case LinkedHqPreparationStage::Organising:
            return "Organising";
        case LinkedHqPreparationStage::Uploading:
            return "Uploading";
        case LinkedHqPreparationStage::Ready:
            return "Ready";
        case LinkedHqPreparationStage::Failed:
            return "Failed";
    }
    return "Waiting";
}

float LinkedHqOverallProgress(
    LinkedHqPreparationStage stage,
    float stageProgress) {
    const float fraction = std::clamp(stageProgress, 0.0F, 1.0F);
    switch (stage) {
        case LinkedHqPreparationStage::Waiting:
            return 0.0F;
        case LinkedHqPreparationStage::Scanning:
            return 0.02F + (0.78F * fraction);
        case LinkedHqPreparationStage::Organising:
            return 0.80F + (0.10F * fraction);
        case LinkedHqPreparationStage::Uploading:
            return 0.90F + (0.10F * fraction);
        case LinkedHqPreparationStage::Ready:
            return 1.0F;
        case LinkedHqPreparationStage::Failed:
            return std::clamp(fraction, 0.0F, 0.99F);
    }
    return 0.0F;
}

}  // namespace invisible_places::app
