#include "app/LinkedHighQualityPreview.hpp"

#include "io/AdaptiveHqCache.hpp"
#include "io/SceneDisplayDensityCache.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <limits>
#include <system_error>
#include <unordered_set>

#include <glm/vec4.hpp>
#include <glm/geometric.hpp>
#include <glm/matrix.hpp>

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

LinkedHqPatchDrawPolicy ResolveLinkedHqPatchDrawPolicy(
    bool linkedHqEnabled,
    bool adaptiveHqEnabled,
    bool adaptiveGuardCoversView,
    bool adaptiveTransitionValid) {
    if (!linkedHqEnabled) {
        return {};
    }
    if (!adaptiveHqEnabled) {
        return {
            .renderFinePatches = true,
        };
    }

    return {
        .renderFinePatches = true,
        .applyFineAdaptiveDensity = adaptiveTransitionValid,
        .applyCoarseAdaptiveDensity =
            adaptiveTransitionValid && adaptiveGuardCoversView,
        .retainingFineDuringRefresh = !adaptiveGuardCoversView,
    };
}

std::uint64_t AdaptiveHqResidentBlockPayloadBytes(
    const invisible_places::io::AdaptiveHqResidentBlock& block) {
    if (block.points == nullptr) {
        return 0U;
    }
    const auto& subset = *block.points;
    const auto& cloud = subset.cloud;
    return
        static_cast<std::uint64_t>(cloud.positions.size()) *
            sizeof(invisible_places::io::Float3) +
        static_cast<std::uint64_t>(cloud.normals.size()) *
            sizeof(invisible_places::io::Float3) +
        static_cast<std::uint64_t>(cloud.packedColors.size()) *
            sizeof(std::uint32_t) +
        static_cast<std::uint64_t>(cloud.scalarFieldValues.size()) *
            sizeof(float) +
        static_cast<std::uint64_t>(subset.sourcePointIndices.size()) *
            sizeof(std::uint32_t);
}

AdaptiveHqResidentRetention RetainAdaptiveHqResidentBlocks(
    std::span<const invisible_places::io::AdaptiveHqResidentBlock> candidates,
    std::span<const std::uint32_t> activeBlockIndices,
    std::uint64_t inactiveByteBudget) {
    AdaptiveHqResidentRetention retained;
    retained.blocks.reserve(candidates.size());

    std::unordered_set<std::uint32_t> activeSet;
    activeSet.reserve(activeBlockIndices.size());
    for (const auto blockIndex : activeBlockIndices) {
        activeSet.insert(blockIndex);
        const auto found = std::find_if(
            candidates.begin(),
            candidates.end(),
            [&](const auto& candidate) {
                return candidate.blockIndex == blockIndex &&
                    candidate.points != nullptr;
            });
        if (found == candidates.end()) {
            continue;
        }
        retained.activePayloadBytes +=
            AdaptiveHqResidentBlockPayloadBytes(*found);
        retained.blocks.push_back(*found);
    }

    std::vector<const invisible_places::io::AdaptiveHqResidentBlock*> fringe;
    fringe.reserve(candidates.size());
    for (const auto& candidate : candidates) {
        if (candidate.points != nullptr &&
            !activeSet.contains(candidate.blockIndex)) {
            fringe.push_back(&candidate);
        }
    }
    std::sort(
        fringe.begin(),
        fringe.end(),
        [](const auto* left, const auto* right) {
            if (left->lastUsedSerial != right->lastUsedSerial) {
                return left->lastUsedSerial > right->lastUsedSerial;
            }
            return left->blockIndex < right->blockIndex;
        });
    for (const auto* candidate : fringe) {
        const auto payloadBytes =
            AdaptiveHqResidentBlockPayloadBytes(*candidate);
        if (payloadBytes >
            inactiveByteBudget - retained.inactivePayloadBytes) {
            continue;
        }
        retained.inactivePayloadBytes += payloadBytes;
        ++retained.inactiveBlockCount;
        retained.blocks.push_back(*candidate);
    }
    return retained;
}

bool LinkedHqFrustumUnion::Contains(
    const invisible_places::io::Float3& point) const {
    const float safeBorder = std::max(0.0F, borderFraction);
    const float lateralLimit = 1.0F + (2.0F * safeBorder);
    for (const auto& viewProjection : viewProjections) {
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
        if (std::isfinite(maximumViewDepthMeters) &&
            maximumViewDepthMeters > 0.0F &&
            clip.w > maximumViewDepthMeters) {
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

bool LinkedHqFrustumUnion::IntersectsBounds(
    const invisible_places::io::Bounds3f& bounds) const {
    if (!bounds.valid || viewProjections.empty()) {
        return false;
    }
    const std::array<glm::vec4, 8U> corners{
        glm::vec4{bounds.minimum.x, bounds.minimum.y, bounds.minimum.z, 1.0F},
        glm::vec4{bounds.maximum.x, bounds.minimum.y, bounds.minimum.z, 1.0F},
        glm::vec4{bounds.minimum.x, bounds.maximum.y, bounds.minimum.z, 1.0F},
        glm::vec4{bounds.maximum.x, bounds.maximum.y, bounds.minimum.z, 1.0F},
        glm::vec4{bounds.minimum.x, bounds.minimum.y, bounds.maximum.z, 1.0F},
        glm::vec4{bounds.maximum.x, bounds.minimum.y, bounds.maximum.z, 1.0F},
        glm::vec4{bounds.minimum.x, bounds.maximum.y, bounds.maximum.z, 1.0F},
        glm::vec4{bounds.maximum.x, bounds.maximum.y, bounds.maximum.z, 1.0F},
    };
    const float lateralLimit =
        1.0F + (2.0F * std::max(0.0F, borderFraction));
    for (const auto& viewProjection : viewProjections) {
        std::array<glm::vec4, 8U> clipCorners;
        for (std::size_t corner = 0U; corner < corners.size(); ++corner) {
            clipCorners[corner] = viewProjection * corners[corner];
        }
        const auto allOutside = [&](const auto& signedDistance) {
            return std::all_of(
                clipCorners.begin(),
                clipCorners.end(),
                [&](const glm::vec4& clip) {
                    return !std::isfinite(clip.x) ||
                           !std::isfinite(clip.y) ||
                           !std::isfinite(clip.z) ||
                           !std::isfinite(clip.w) ||
                           signedDistance(clip) < 0.0F;
                });
        };
        if (allOutside([&](const glm::vec4& clip) {
                return clip.x + lateralLimit * clip.w;
            }) ||
            allOutside([&](const glm::vec4& clip) {
                return lateralLimit * clip.w - clip.x;
            }) ||
            allOutside([&](const glm::vec4& clip) {
                return clip.y + lateralLimit * clip.w;
            }) ||
            allOutside([&](const glm::vec4& clip) {
                return lateralLimit * clip.w - clip.y;
            }) ||
            allOutside([](const glm::vec4& clip) {
                return clip.z + clip.w;
            }) ||
            allOutside([](const glm::vec4& clip) {
                return clip.w - clip.z;
            }) ||
            allOutside([](const glm::vec4& clip) {
                return clip.w - 1.0e-7F;
            })) {
            continue;
        }
        if (std::isfinite(maximumViewDepthMeters) &&
            maximumViewDepthMeters > 0.0F &&
            allOutside([&](const glm::vec4& clip) {
                return maximumViewDepthMeters - clip.w;
            })) {
            continue;
        }
        return true;
    }
    return false;
}

LinkedHqFrustumUnion BuildAnimationHqFrustumUnion(
    std::span<const glm::mat4> viewProjections,
    float borderFraction) {
    LinkedHqFrustumUnion result;
    result.borderFraction = std::max(0.0F, borderFraction);
    result.viewProjections.reserve(viewProjections.size());
    const auto matricesEqual = [](const glm::mat4& left,
                                  const glm::mat4& right) {
        for (std::size_t column = 0U; column < 4U; ++column) {
            for (std::size_t row = 0U; row < 4U; ++row) {
                if (left[column][row] != right[column][row]) {
                    return false;
                }
            }
        }
        return true;
    };
    for (const auto& viewProjection : viewProjections) {
        const bool duplicate = std::any_of(
            result.viewProjections.begin(),
            result.viewProjections.end(),
            [&](const glm::mat4& existing) {
                return matricesEqual(existing, viewProjection);
            });
        if (!duplicate) {
            result.viewProjections.push_back(viewProjection);
        }
    }
    return result;
}

bool LinkedHqFrustumUnionCoversView(
    const LinkedHqFrustumUnion& frustumUnion,
    const glm::mat4& view,
    const glm::mat4& projection,
    float requiredDepthMeters,
    float requiredBorderFraction) {
    if (frustumUnion.viewProjections.empty() ||
        !std::isfinite(requiredDepthMeters) || requiredDepthMeters <= 0.0F) {
        return false;
    }
    const glm::mat4 inverseProjection = glm::inverse(projection);
    const glm::mat4 inverseView = glm::inverse(view);
    const float requiredLateralLimit =
        1.0F + 2.0F * std::max(0.0F, requiredBorderFraction);
    const std::array<float, 3U> coordinates{
        -requiredLateralLimit,
        0.0F,
        requiredLateralLimit,
    };
    const std::array<float, 2U> depths{
        std::max(0.01F, requiredDepthMeters * 0.20F),
        requiredDepthMeters,
    };
    for (const float depth : depths) {
        for (const float ndcY : coordinates) {
            for (const float ndcX : coordinates) {
                glm::vec4 nearView =
                    inverseProjection * glm::vec4{ndcX, ndcY, -1.0F, 1.0F};
                glm::vec4 farView =
                    inverseProjection * glm::vec4{ndcX, ndcY, 1.0F, 1.0F};
                if (std::abs(nearView.w) <= 1.0e-7F ||
                    std::abs(farView.w) <= 1.0e-7F) {
                    return false;
                }
                nearView /= nearView.w;
                farView /= farView.w;
                const glm::vec3 ray = glm::vec3{farView - nearView};
                if (!std::isfinite(ray.z) || std::abs(ray.z) <= 1.0e-7F) {
                    return false;
                }
                const float distanceAlongRay =
                    (-depth - nearView.z) / ray.z;
                const glm::vec3 viewPoint =
                    glm::vec3{nearView} + ray * distanceAlongRay;
                const glm::vec4 world =
                    inverseView * glm::vec4{viewPoint, 1.0F};
                if (std::abs(world.w) <= 1.0e-7F ||
                    !frustumUnion.Contains({
                        world.x / world.w,
                        world.y / world.w,
                        world.z / world.w,
                    })) {
                    return false;
                }
            }
        }
    }
    return true;
}

std::vector<float> BuildUnlinkedAnimationHqSampleTimes(
    std::span<const float> authoredKeyTimesSeconds,
    float durationSeconds) {
    const float duration = std::isfinite(durationSeconds)
        ? std::max(0.0F, durationSeconds)
        : 0.0F;
    std::vector<float> times;
    times.reserve(
        kUnlinkedAnimationHqUniformViewCount +
        authoredKeyTimesSeconds.size());
    if (duration <= 0.0F) {
        times.push_back(0.0F);
        return times;
    }
    for (std::size_t sample = 0U;
         sample < kUnlinkedAnimationHqUniformViewCount;
         ++sample) {
        const float position = static_cast<float>(sample) /
            static_cast<float>(kUnlinkedAnimationHqUniformViewCount - 1U);
        times.push_back(duration * position);
    }
    for (const float keyTime : authoredKeyTimesSeconds) {
        if (std::isfinite(keyTime)) {
            times.push_back(std::clamp(keyTime, 0.0F, duration));
        }
    }
    std::sort(times.begin(), times.end());
    const float duplicateTolerance = std::max(1.0e-6F, duration * 1.0e-6F);
    times.erase(
        std::unique(
            times.begin(),
            times.end(),
            [duplicateTolerance](float left, float right) {
                return std::abs(left - right) <= duplicateTolerance;
            }),
        times.end());
    return times;
}

std::vector<float> BuildAnimationSectionFrustumSampleTimes(
    float durationSeconds,
    float normalizedStart,
    float normalizedEnd,
    std::uint32_t framesPerSecond,
    std::uint32_t paddingFrames) {
    const float duration = std::isfinite(durationSeconds)
        ? std::max(0.0F, durationSeconds)
        : 0.0F;
    const std::uint32_t rate = std::max(1U, framesPerSecond);
    float start = std::isfinite(normalizedStart)
        ? std::clamp(normalizedStart, 0.0F, 1.0F)
        : 0.0F;
    float end = std::isfinite(normalizedEnd)
        ? std::clamp(normalizedEnd, 0.0F, 1.0F)
        : 1.0F;
    if (end < start) {
        std::swap(start, end);
    }
    if (duration <= 0.0F) {
        return {0.0F};
    }

    const double totalFrames =
        static_cast<double>(duration) * static_cast<double>(rate);
    const auto firstFrame = static_cast<std::int64_t>(std::max(
        0.0,
        std::floor(static_cast<double>(start) * totalFrames) -
            static_cast<double>(paddingFrames)));
    const auto lastFrame = static_cast<std::int64_t>(std::min(
        std::ceil(totalFrames),
        std::ceil(static_cast<double>(end) * totalFrames) +
            static_cast<double>(paddingFrames)));

    std::vector<float> times;
    times.reserve(static_cast<std::size_t>(
        std::max<std::int64_t>(1, lastFrame - firstFrame + 3)));
    times.push_back(duration * start);
    for (std::int64_t frame = firstFrame; frame <= lastFrame; ++frame) {
        times.push_back(std::clamp(
            static_cast<float>(frame) / static_cast<float>(rate),
            0.0F,
            duration));
    }
    times.push_back(duration * end);
    std::sort(times.begin(), times.end());
    const float duplicateTolerance = std::max(1.0e-6F, duration * 1.0e-7F);
    times.erase(
        std::unique(
            times.begin(),
            times.end(),
            [duplicateTolerance](float left, float right) {
                return std::abs(left - right) <= duplicateTolerance;
            }),
        times.end());
    return times;
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
    const auto inspected = invisible_places::io::InspectAdaptiveHqSource(
        payloadPath);
    if (!inspected.success) {
        if (errorMessage != nullptr) {
            *errorMessage = "Unable to fingerprint HQ source: " +
                inspected.errorMessage;
        }
        return false;
    }
    *fingerprint = {
        .path = inspected.identity.path,
        .byteSize = inspected.identity.byteSize,
        .writeTimeTicks = inspected.identity.writeTimeTicks,
        .schemaFingerprint = inspected.identity.schemaFingerprint,
        .contentFingerprint = inspected.identity.contentFingerprint,
        .pointCount = inspected.identity.pointCount,
        .recordSize = inspected.identity.recordSize,
    };
    return true;
}

std::uint64_t BuildLinkedHqSelectionFingerprint(
    std::string_view pairKey,
    const LinkedHqFrustumUnion& frustumUnion,
    std::span<const LinkedHqSourceFingerprint> sources) {
    std::uint64_t hash = 1469598103934665603ULL;
    HashBytes(&hash, pairKey.data(), pairKey.size());
    HashValue(&hash, frustumUnion.borderFraction);
    HashValue(&hash, frustumUnion.maximumViewDepthMeters);
    HashValue(&hash, frustumUnion.viewProjections.size());
    for (const auto& matrix : frustumUnion.viewProjections) {
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
        HashBytes(
            &hash,
            source.schemaFingerprint.data(),
            source.schemaFingerprint.size());
        HashBytes(
            &hash,
            source.contentFingerprint.data(),
            source.contentFingerprint.size());
        HashValue(&hash, source.pointCount);
        HashValue(&hash, source.recordSize);
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
