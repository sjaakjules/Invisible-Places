#include "camera/AnimationPath.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>
#include <unordered_set>

#include <glm/geometric.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/mat3x3.hpp>
#include <glm/trigonometric.hpp>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

namespace invisible_places::camera {

namespace {

constexpr float kAnimationFramesPerSecond = 30.0F;

float EvaluatePreparedScalarSpline(
    const std::vector<float>& knots,
    const AnimationPreparedScalarSpline& spline,
    float x) {
    if (knots.empty() || spline.values.empty()) {
        return 0.0F;
    }
    if (knots.size() == 1U || spline.values.size() == 1U) {
        return spline.values.front();
    }

    const float clampedX = std::clamp(x, knots.front(), knots.back());
    if (clampedX <= knots.front()) {
        return spline.values.front();
    }
    if (clampedX >= knots.back()) {
        return spline.values.back();
    }

    const auto upper = std::upper_bound(knots.begin(), knots.end(), clampedX);
    const std::size_t rightIndex =
        std::clamp<std::size_t>(static_cast<std::size_t>(upper - knots.begin()), 1U, knots.size() - 1U);
    const std::size_t leftIndex = rightIndex - 1U;
    const float interval = knots[rightIndex] - knots[leftIndex];
    if (interval <= 1.0e-6F) {
        return spline.values[leftIndex];
    }

    const float a = (knots[rightIndex] - clampedX) / interval;
    const float b = (clampedX - knots[leftIndex]) / interval;
    const float leftCurve = ((a * a * a) - a) * spline.secondDerivatives[leftIndex];
    const float rightCurve = ((b * b * b) - b) * spline.secondDerivatives[rightIndex];
    return (a * spline.values[leftIndex]) +
           (b * spline.values[rightIndex]) +
           ((leftCurve + rightCurve) * interval * interval / 6.0F);
}

AnimationPreparedScalarSpline BuildClampedCubicSpline(
    const std::vector<float>& knots,
    const std::vector<float>& values,
    std::optional<float> authoredFirstDerivative = std::nullopt,
    std::optional<float> authoredLastDerivative = std::nullopt) {
    AnimationPreparedScalarSpline spline{
        .values = values,
        .secondDerivatives = std::vector<float>(values.size(), 0.0F),
    };

    if (knots.size() < 2U || knots.size() != values.size()) {
        return spline;
    }

    const std::size_t systemSize = knots.size();
    std::vector<float> lower(systemSize, 0.0F);
    std::vector<float> diagonal(systemSize, 0.0F);
    std::vector<float> upper(systemSize, 0.0F);
    std::vector<float> rhs(systemSize, 0.0F);

    const float firstInterval = knots[1U] - knots[0U];
    const float lastInterval =
        knots.back() - knots[knots.size() - 2U];
    if (firstInterval <= 1.0e-6F || lastInterval <= 1.0e-6F) {
        return spline;
    }
    // One-sided chord slopes keep endpoints travelling toward the adjacent
    // key instead of allowing the global cubic solve to point backwards.
    // Unlike an ease curve, this does not force endpoint velocity to zero.
    const float firstDerivative = authoredFirstDerivative.value_or(
        (values[1U] - values[0U]) / firstInterval);
    const float lastDerivative = authoredLastDerivative.value_or(
        (values.back() - values[values.size() - 2U]) / lastInterval);
    diagonal.front() = 2.0F * firstInterval;
    upper.front() = firstInterval;
    rhs.front() = 6.0F *
        (((values[1U] - values[0U]) / firstInterval) -
         firstDerivative);
    lower.back() = lastInterval;
    diagonal.back() = 2.0F * lastInterval;
    rhs.back() = 6.0F *
        (lastDerivative -
         ((values.back() - values[values.size() - 2U]) /
          lastInterval));

    for (std::size_t knotIndex = 1U;
         knotIndex + 1U < systemSize;
         ++knotIndex) {
        const float previousInterval = knots[knotIndex] - knots[knotIndex - 1U];
        const float nextInterval = knots[knotIndex + 1U] - knots[knotIndex];
        if (previousInterval <= 1.0e-6F || nextInterval <= 1.0e-6F) {
            continue;
        }

        lower[knotIndex] = previousInterval;
        diagonal[knotIndex] =
            2.0F * (previousInterval + nextInterval);
        upper[knotIndex] = nextInterval;
        rhs[knotIndex] =
            6.0F *
            (((values[knotIndex + 1U] - values[knotIndex]) / nextInterval) -
             ((values[knotIndex] - values[knotIndex - 1U]) / previousInterval));
    }

    for (std::size_t row = 1; row < systemSize; ++row) {
        if (std::abs(diagonal[row - 1U]) <= 1.0e-6F) {
            continue;
        }
        const float factor = lower[row] / diagonal[row - 1U];
        diagonal[row] -= factor * upper[row - 1U];
        rhs[row] -= factor * rhs[row - 1U];
    }

    std::vector<float> solution(systemSize, 0.0F);
    for (std::size_t reverseRow = systemSize; reverseRow > 0U; --reverseRow) {
        const std::size_t row = reverseRow - 1U;
        if (std::abs(diagonal[row]) <= 1.0e-6F) {
            solution[row] = 0.0F;
            continue;
        }
        const float nextValue = (row + 1U) < systemSize ? solution[row + 1U] : 0.0F;
        solution[row] = (rhs[row] - (upper[row] * nextValue)) / diagonal[row];
    }

    spline.secondDerivatives = std::move(solution);
    return spline;
}

float EvaluatePreparedScalarSplineEndpointDerivative(
    const std::vector<float>& knots,
    const AnimationPreparedScalarSpline& spline,
    bool firstEndpoint) {
    if (knots.size() < 2U || spline.values.size() != knots.size() ||
        spline.secondDerivatives.size() != knots.size()) {
        return 0.0F;
    }
    if (firstEndpoint) {
        const float interval = knots[1U] - knots[0U];
        if (interval <= 1.0e-6F) {
            return 0.0F;
        }
        return (spline.values[1U] - spline.values[0U]) / interval -
               interval *
                   (2.0F * spline.secondDerivatives[0U] +
                    spline.secondDerivatives[1U]) /
                   6.0F;
    }
    const std::size_t last = knots.size() - 1U;
    const float interval = knots[last] - knots[last - 1U];
    if (interval <= 1.0e-6F) {
        return 0.0F;
    }
    return (spline.values[last] - spline.values[last - 1U]) / interval +
           interval *
               (spline.secondDerivatives[last - 1U] +
                2.0F * spline.secondDerivatives[last]) /
               6.0F;
}

glm::vec3 ToGlm(const std::array<float, 3>& value) {
    return {value[0], value[1], value[2]};
}

std::uint32_t MinimumAnimationDurationFrames(const AnimationPath& path) {
    return path.keys.size() > 1U ? static_cast<std::uint32_t>(path.keys.size() - 1U) : 1U;
}

float Distance(const std::array<float, 3>& left, const std::array<float, 3>& right) {
    return glm::length(ToGlm(right) - ToGlm(left));
}

std::array<float, 3> Difference(
    const std::array<float, 3>& value,
    const std::array<float, 3>& origin) {
    return {
        value[0] - origin[0],
        value[1] - origin[1],
        value[2] - origin[2],
    };
}

bool ValidLocalizedKeyCorrections(const AnimationPath& path) {
    std::unordered_set<std::string> correctionIds;
    for (const auto& correction : path.localizedKeyCorrections) {
        if (correction.keyId.empty() ||
            !correctionIds.insert(correction.keyId).second ||
            std::none_of(
                path.keys.begin(),
                path.keys.end(),
                [&](const AnimationPathKey& key) {
                    return key.id == correction.keyId;
                })) {
            return false;
        }
    }
    return true;
}

std::vector<std::array<float, 3>> BuildLoopCorrectionTangents(
    const std::vector<float>& knots,
    const std::vector<std::array<float, 3>>& corrections,
    const std::vector<std::uint8_t>& enabled) {
    std::vector<std::array<float, 3>> tangents(
        corrections.size(),
        {0.0F, 0.0F, 0.0F});
    if (corrections.size() < 2U || knots.size() != corrections.size() ||
        enabled.size() != corrections.size()) {
        return tangents;
    }
    for (std::size_t keyIndex = 0U;
         keyIndex < corrections.size();
         ++keyIndex) {
        if (enabled[keyIndex] == 0U) {
            continue;
        }
        const std::size_t leftIndex =
            keyIndex == 0U ? 0U : keyIndex - 1U;
        const std::size_t rightIndex =
            keyIndex + 1U < corrections.size()
                ? keyIndex + 1U
                : corrections.size() - 1U;
        const float interval = knots[rightIndex] - knots[leftIndex];
        if (interval <= 1.0e-6F) {
            continue;
        }
        for (std::size_t component = 0U; component < 3U; ++component) {
            tangents[keyIndex][component] =
                (corrections[rightIndex][component] -
                 corrections[leftIndex][component]) /
                interval;
        }
    }
    return tangents;
}

std::array<float, 3> EvaluateLoopKeyCorrection(
    const PreparedAnimationPathEvaluationContext& context,
    const std::vector<std::array<float, 3>>& corrections,
    const std::vector<std::array<float, 3>>& tangents,
    float timeSeconds) {
    std::array<float, 3> correction{0.0F, 0.0F, 0.0F};
    if (!context.hasLoopKeyCorrections || context.knots.size() < 2U ||
        corrections.size() != context.knots.size() ||
        tangents.size() != context.knots.size() ||
        context.loopCorrectionKeyEnabled.size() != context.knots.size()) {
        return correction;
    }
    const float clampedTime = std::clamp(
        timeSeconds,
        context.knots.front(),
        context.knots.back());
    const auto upper = std::upper_bound(
        context.knots.begin(),
        context.knots.end(),
        clampedTime);
    const std::size_t rightIndex = std::clamp<std::size_t>(
        static_cast<std::size_t>(upper - context.knots.begin()),
        1U,
        context.knots.size() - 1U);
    const std::size_t leftIndex = rightIndex - 1U;
    if (context.loopCorrectionKeyEnabled[leftIndex] == 0U &&
        context.loopCorrectionKeyEnabled[rightIndex] == 0U) {
        return correction;
    }
    const float interval =
        context.knots[rightIndex] - context.knots[leftIndex];
    const float amount = interval <= 1.0e-6F
                             ? 0.0F
                             : std::clamp(
                                   (clampedTime - context.knots[leftIndex]) /
                                       interval,
                                   0.0F,
                                   1.0F);
    const float amount2 = amount * amount;
    const float amount3 = amount2 * amount;
    const float amount4 = amount3 * amount;
    const float amount5 = amount4 * amount;
    // Quintic Hermite with zero correction acceleration at every key. The
    // stored first derivatives still change seam velocity as intended, while
    // shared zero second derivatives make the correction layer C2. A segment
    // bounded by two locked zero corrections remains exactly untouched.
    const float h00 =
        1.0F - (10.0F * amount3) + (15.0F * amount4) -
        (6.0F * amount5);
    const float h10 =
        amount - (6.0F * amount3) + (8.0F * amount4) -
        (3.0F * amount5);
    const float h01 =
        (10.0F * amount3) - (15.0F * amount4) +
        (6.0F * amount5);
    const float h11 =
        (-4.0F * amount3) + (7.0F * amount4) -
        (3.0F * amount5);
    for (std::size_t component = 0U; component < 3U; ++component) {
        correction[component] =
            h00 * corrections[leftIndex][component] +
            h10 * interval * tangents[leftIndex][component] +
            h01 * corrections[rightIndex][component] +
            h11 * interval * tangents[rightIndex][component];
    }
    return correction;
}

std::array<float, 3> FocusPointFromShot(const CameraShot& shot) {
    return shot.state.target;
}

std::vector<std::uint32_t> BuildSegmentDurations(const AnimationPath& path) {
    if (path.keys.size() < 2U) {
        return {};
    }

    const auto segmentCount = path.keys.size() - 1U;
    std::vector<std::uint32_t> durations(segmentCount, 1U);
    std::uint32_t sourceTotalFrames = 0U;
    for (std::size_t segmentIndex = 0; segmentIndex < segmentCount; ++segmentIndex) {
        durations[segmentIndex] = std::max<std::uint32_t>(1U, path.keys[segmentIndex + 1U].durationFrames);
        sourceTotalFrames += durations[segmentIndex];
    }

    const auto targetTotalFrames = std::max<std::uint32_t>(
        path.durationFrames,
        static_cast<std::uint32_t>(segmentCount));
    if (sourceTotalFrames == targetTotalFrames) {
        return durations;
    }

    if (sourceTotalFrames <= segmentCount) {
        const auto baseFrames = targetTotalFrames / static_cast<std::uint32_t>(segmentCount);
        const auto leftoverFrames = targetTotalFrames % static_cast<std::uint32_t>(segmentCount);
        for (std::size_t segmentIndex = 0; segmentIndex < segmentCount; ++segmentIndex) {
            durations[segmentIndex] = baseFrames + (segmentIndex < leftoverFrames ? 1U : 0U);
        }
        return durations;
    }

    std::vector<float> remainders(segmentCount, 0.0F);
    std::vector<std::uint32_t> retimed(segmentCount, 1U);
    std::uint32_t assignedFrames = static_cast<std::uint32_t>(segmentCount);
    const std::uint32_t extraFrames = targetTotalFrames - assignedFrames;
    const std::uint32_t sourceExtraFrames = sourceTotalFrames - static_cast<std::uint32_t>(segmentCount);

    for (std::size_t segmentIndex = 0; segmentIndex < segmentCount; ++segmentIndex) {
        const auto segmentExtraFrames = durations[segmentIndex] - 1U;
        const float idealExtraFrames =
            static_cast<float>(extraFrames) *
            (static_cast<float>(segmentExtraFrames) / static_cast<float>(sourceExtraFrames));
        const auto wholeFrames = static_cast<std::uint32_t>(std::floor(idealExtraFrames));
        retimed[segmentIndex] += wholeFrames;
        assignedFrames += wholeFrames;
        remainders[segmentIndex] = idealExtraFrames - static_cast<float>(wholeFrames);
    }

    while (assignedFrames < targetTotalFrames) {
        const auto nextIt = std::max_element(remainders.begin(), remainders.end());
        if (nextIt == remainders.end()) {
            break;
        }
        const auto index = static_cast<std::size_t>(nextIt - remainders.begin());
        ++retimed[index];
        *nextIt = -1.0F;
        ++assignedFrames;
    }

    return retimed;
}

std::vector<float> BuildAnimationKnots(const AnimationPath& path) {
    std::vector<float> knots;
    if (path.keys.size() < 2U) {
        return knots;
    }

    const auto segmentDurations = BuildSegmentDurations(path);
    knots.reserve(path.keys.size());
    knots.push_back(0.0F);
    float elapsedSeconds = 0.0F;
    for (const auto durationFrames : segmentDurations) {
        elapsedSeconds += static_cast<float>(durationFrames) / kAnimationFramesPerSecond;
        knots.push_back(elapsedSeconds);
    }
    return knots;
}

std::vector<float> BuildKeySamples(
    const AnimationPath& path,
    float (*readValue)(const AnimationPathKey& key)) {
    std::vector<float> samples;
    samples.reserve(path.keys.size());
    for (const auto& key : path.keys) {
        samples.push_back(readValue(key));
    }
    return samples;
}

float ReadCameraX(const AnimationPathKey& key) {
    return key.cameraPosition[0];
}

float ReadCameraY(const AnimationPathKey& key) {
    return key.cameraPosition[1];
}

float ReadCameraZ(const AnimationPathKey& key) {
    return key.cameraPosition[2];
}

float ReadFocusX(const AnimationPathKey& key) {
    return key.focusPoint[0];
}

float ReadFocusY(const AnimationPathKey& key) {
    return key.focusPoint[1];
}

float ReadFocusZ(const AnimationPathKey& key) {
    return key.focusPoint[2];
}

float ReadFovDegrees(const AnimationPathKey& key) {
    return key.fovDegrees;
}

float ReadNearPlane(const AnimationPathKey& key) {
    return key.nearPlane;
}

float ReadFarPlane(const AnimationPathKey& key) {
    return key.farPlane;
}

glm::quat LookAtOrientation(glm::vec3 cameraPosition, glm::vec3 focusPoint);

float ReadFocusDistance(const AnimationPathKey& key) {
    return key.hasFocusDistance
               ? std::max(0.001F, key.focusDistance)
               : std::max(0.001F, Distance(key.cameraPosition, key.focusPoint));
}

float ReadApertureFStops(const AnimationPathKey& key) {
    return key.hasApertureFStops ? std::max(0.1F, key.apertureFStops) : 8.0F;
}

bool AnyKeyHasOrientation(const AnimationPath& path) {
    return std::any_of(path.keys.begin(), path.keys.end(), [](const AnimationPathKey& key) {
        return key.hasOrientation;
    });
}

bool AnyKeyHasFocusDistance(const AnimationPath& path) {
    return std::any_of(path.keys.begin(), path.keys.end(), [](const AnimationPathKey& key) {
        return key.hasFocusDistance;
    });
}

bool AnyKeyHasApertureFStops(const AnimationPath& path) {
    return std::any_of(path.keys.begin(), path.keys.end(), [](const AnimationPathKey& key) {
        return key.hasApertureFStops;
    });
}

glm::quat OrientationFromKey(const AnimationPathKey& key) {
    if (key.hasOrientation) {
        const glm::quat orientation{
            key.orientation[3],
            key.orientation[0],
            key.orientation[1],
            key.orientation[2],
        };
        const float lengthSquared =
            (orientation.w * orientation.w) +
            (orientation.x * orientation.x) +
            (orientation.y * orientation.y) +
            (orientation.z * orientation.z);
        if (lengthSquared > 1.0e-8F) {
            return glm::normalize(orientation);
        }
    }
    return LookAtOrientation(ToGlm(key.cameraPosition), ToGlm(key.focusPoint));
}

glm::quat LookAtOrientation(glm::vec3 cameraPosition, glm::vec3 focusPoint) {
    glm::vec3 forward = focusPoint - cameraPosition;
    if (glm::length(forward) <= 1.0e-5F) {
        forward = glm::vec3{0.0F, 0.0F, -1.0F};
        focusPoint = cameraPosition + forward;
    }

    glm::vec3 up{0.0F, 0.0F, 1.0F};
    if (std::abs(glm::dot(glm::normalize(forward), up)) > 0.995F) {
        up = glm::vec3{0.0F, 1.0F, 0.0F};
    }

    const auto view = glm::lookAtRH(cameraPosition, focusPoint, up);
    const auto cameraToWorld = glm::inverse(glm::mat3{view});
    return glm::normalize(glm::quat_cast(cameraToWorld));
}

std::array<float, 4> QuaternionToArray(const glm::quat& orientation) {
    return {orientation.x, orientation.y, orientation.z, orientation.w};
}

glm::quat QuaternionFromArray(const std::array<float, 4>& orientation) {
    return glm::quat{orientation[3], orientation[0], orientation[1], orientation[2]};
}

std::vector<float> BuildAnimationGeometryKnots(
    const std::vector<float>& cameraX,
    const std::vector<float>& cameraY,
    const std::vector<float>& cameraZ,
    const std::vector<float>& focusX,
    const std::vector<float>& focusY,
    const std::vector<float>& focusZ,
    const std::vector<std::array<float, 4>>& orientations,
    const std::vector<AnimationPathKey>& splineKeys) {
    const std::size_t count = cameraX.size();
    if (count < 2U || cameraY.size() != count || cameraZ.size() != count ||
        focusX.size() != count || focusY.size() != count ||
        focusZ.size() != count || orientations.size() != count ||
        splineKeys.size() != count) {
        return {};
    }

    const bool hasMaterializedWeights = std::all_of(
        splineKeys.begin() + 1,
        splineKeys.end(),
        [](const AnimationPathKey& key) {
            return std::isfinite(key.splineParameterWeight) &&
                   key.splineParameterWeight > 1.0e-6F;
        });
    if (hasMaterializedWeights) {
        std::vector<float> knots(count, 0.0F);
        for (std::size_t index = 1U; index < count; ++index) {
            knots[index] = knots[index - 1U] +
                splineKeys[index].splineParameterWeight;
        }
        return knots;
    }

    std::vector<float> spans(count - 1U, 0.0F);
    float positiveSpanTotal = 0.0F;
    std::size_t positiveSpanCount = 0U;
    for (std::size_t index = 0U; index + 1U < count; ++index) {
        const glm::vec3 cameraFrom{
            cameraX[index], cameraY[index], cameraZ[index]};
        const glm::vec3 cameraTo{
            cameraX[index + 1U],
            cameraY[index + 1U],
            cameraZ[index + 1U]};
        const glm::vec3 focusFrom{
            focusX[index], focusY[index], focusZ[index]};
        const glm::vec3 focusTo{
            focusX[index + 1U],
            focusY[index + 1U],
            focusZ[index + 1U]};
        const float cameraDistance = glm::length(cameraTo - cameraFrom);
        const float focusDistance = glm::length(focusTo - focusFrom);
        const auto fromOrientation = QuaternionFromArray(
            orientations[index]);
        const auto toOrientation = QuaternionFromArray(
            orientations[index + 1U]);
        const float orientationDot = std::clamp(
            std::abs(glm::dot(fromOrientation, toOrientation)),
            0.0F,
            1.0F);
        const float orientationAngle = 2.0F * std::acos(orientationDot);
        const float viewScale = std::max(
            1.0e-3F,
            0.5F *
                (glm::length(focusFrom - cameraFrom) +
                 glm::length(focusTo - cameraTo)));
        const float angularDistance = viewScale * orientationAngle;
        const float span = std::hypot(
            cameraDistance,
            std::hypot(focusDistance, angularDistance));
        if (std::isfinite(span) && span > 1.0e-6F) {
            spans[index] = span;
            positiveSpanTotal += span;
            ++positiveSpanCount;
        }
    }

    const float fallbackSpan = positiveSpanCount > 0U
        ? std::max(
              1.0e-4F,
              1.0e-3F * positiveSpanTotal /
                  static_cast<float>(positiveSpanCount))
        : 1.0F;
    std::vector<float> knots(count, 0.0F);
    for (std::size_t index = 0U; index < spans.size(); ++index) {
        knots[index + 1U] = knots[index] +
            std::max(spans[index], fallbackSpan);
    }
    return knots;
}

std::vector<float> BuildGeometryTimeVelocities(
    const std::vector<float>& timeKnots,
    const std::vector<float>& geometryKnots) {
    if (timeKnots.size() < 2U ||
        timeKnots.size() != geometryKnots.size()) {
        return {};
    }
    const std::size_t count = timeKnots.size();
    std::vector<float> secants(count - 1U, 0.0F);
    for (std::size_t index = 0U; index < secants.size(); ++index) {
        const float duration = timeKnots[index + 1U] - timeKnots[index];
        const float distance =
            geometryKnots[index + 1U] - geometryKnots[index];
        if (duration <= 1.0e-6F || distance <= 1.0e-6F) {
            return {};
        }
        secants[index] = distance / duration;
    }

    std::vector<float> velocities(count, 0.0F);
    velocities.front() = secants.front();
    velocities.back() = secants.back();
    for (std::size_t index = 1U; index + 1U < count; ++index) {
        const float previousDuration =
            timeKnots[index] - timeKnots[index - 1U];
        const float nextDuration =
            timeKnots[index + 1U] - timeKnots[index];
        const float previousSlope = secants[index - 1U];
        const float nextSlope = secants[index];
        const float firstWeight =
            (2.0F * nextDuration) + previousDuration;
        const float secondWeight =
            nextDuration + (2.0F * previousDuration);
        const float denominator =
            (firstWeight / previousSlope) +
            (secondWeight / nextSlope);
        const float harmonic = denominator > 1.0e-8F
            ? (firstWeight + secondWeight) / denominator
            : std::min(previousSlope, nextSlope);
        // With zero endpoint acceleration, normalized quintic slopes up to
        // two are monotone. This cap prevents timing weights from making the
        // path run backwards while retaining a positive through-key speed.
        velocities[index] = std::clamp(
            harmonic,
            1.0e-6F,
            2.0F * std::min(previousSlope, nextSlope));
    }
    return velocities;
}

float EvaluateGeometryTimeMap(
    const PreparedAnimationPathEvaluationContext& context,
    float timeSeconds) {
    if (context.knots.size() < 2U ||
        context.geometryKnots.size() != context.knots.size() ||
        context.geometryVelocities.size() != context.knots.size()) {
        return context.geometryKnots.empty()
            ? 0.0F
            : context.geometryKnots.front();
    }
    const float clampedTime = std::clamp(
        timeSeconds,
        context.knots.front(),
        context.knots.back());
    if (clampedTime <= context.knots.front()) {
        return context.geometryKnots.front();
    }
    if (clampedTime >= context.knots.back()) {
        return context.geometryKnots.back();
    }
    const auto upper = std::upper_bound(
        context.knots.begin(),
        context.knots.end(),
        clampedTime);
    const std::size_t rightIndex = std::clamp<std::size_t>(
        static_cast<std::size_t>(upper - context.knots.begin()),
        1U,
        context.knots.size() - 1U);
    const std::size_t leftIndex = rightIndex - 1U;
    const float interval =
        context.knots[rightIndex] - context.knots[leftIndex];
    if (interval <= 1.0e-6F) {
        return context.geometryKnots[leftIndex];
    }
    const float amount = std::clamp(
        (clampedTime - context.knots[leftIndex]) / interval,
        0.0F,
        1.0F);
    const float amount2 = amount * amount;
    const float amount3 = amount2 * amount;
    const float amount4 = amount3 * amount;
    const float amount5 = amount4 * amount;
    const float positionLeft =
        1.0F - (10.0F * amount3) + (15.0F * amount4) -
        (6.0F * amount5);
    const float velocityLeft =
        amount - (6.0F * amount3) + (8.0F * amount4) -
        (3.0F * amount5);
    const float positionRight =
        (10.0F * amount3) - (15.0F * amount4) +
        (6.0F * amount5);
    const float velocityRight =
        (-4.0F * amount3) + (7.0F * amount4) -
        (3.0F * amount5);
    const float geometry =
        positionLeft * context.geometryKnots[leftIndex] +
        velocityLeft * interval *
            context.geometryVelocities[leftIndex] +
        positionRight * context.geometryKnots[rightIndex] +
        velocityRight * interval *
            context.geometryVelocities[rightIndex];
    return std::clamp(
        geometry,
        context.geometryKnots[leftIndex],
        context.geometryKnots[rightIndex]);
}

glm::quat EvaluatePreparedOrientation(
    const PreparedAnimationPathEvaluationContext& context,
    float geometryPosition) {
    if (context.orientationQuaternions.empty()) {
        return glm::quat{1.0F, 0.0F, 0.0F, 0.0F};
    }
    if (context.orientationQuaternions.size() == 1U ||
        context.geometryKnots.size() !=
            context.orientationQuaternions.size()) {
        return QuaternionFromArray(context.orientationQuaternions.front());
    }
    const glm::quat orientation{
        EvaluatePreparedScalarSpline(
            context.geometryKnots,
            context.orientationW,
            geometryPosition),
        EvaluatePreparedScalarSpline(
            context.geometryKnots,
            context.orientationX,
            geometryPosition),
        EvaluatePreparedScalarSpline(
            context.geometryKnots,
            context.orientationY,
            geometryPosition),
        EvaluatePreparedScalarSpline(
            context.geometryKnots,
            context.orientationZ,
            geometryPosition),
    };
    const float lengthSquared = glm::dot(orientation, orientation);
    if (lengthSquared > 1.0e-12F && std::isfinite(lengthSquared)) {
        return glm::normalize(orientation);
    }
    const auto nearest = std::lower_bound(
        context.geometryKnots.begin(),
        context.geometryKnots.end(),
        geometryPosition);
    const std::size_t nearestIndex = nearest == context.geometryKnots.end()
        ? context.orientationQuaternions.size() - 1U
        : static_cast<std::size_t>(
              nearest - context.geometryKnots.begin());
    return QuaternionFromArray(
        context.orientationQuaternions[nearestIndex]);
}

glm::vec3 ViewDirectionFromEvaluation(
    const PreparedAnimationPathEvaluationContext& context,
    const AnimationPathEvaluation& evaluation) {
    if (!context.hasOrientation) {
        const glm::vec3 toFocus =
            ToGlm(evaluation.focusPoint) - ToGlm(evaluation.camera.position);
        if (glm::dot(toFocus, toFocus) > 1.0e-10F) {
            return glm::normalize(toFocus);
        }
    }
    return QuaternionFromCameraState(evaluation.camera) * glm::vec3{0.0F, 0.0F, -1.0F};
}

struct PerceivedFlowProbe {
    float screenSpeed = 0.0F;
    float fullScreenSpeed = 0.0F;
    std::array<float, 2> screenVelocity{0.0F, 0.0F};
    std::array<float, 2> focusPlaneMeanVelocity{0.0F, 0.0F};
    float focusPlaneVelocityVariation = 0.0F;
    std::array<float, 2> topScreenVelocity{0.0F, 0.0F};
    std::array<float, 2> middleScreenVelocity{0.0F, 0.0F};
    std::array<float, 2> bottomScreenVelocity{0.0F, 0.0F};
    float imageRotationDegreesPerSecond = 0.0F;
};

bool ProjectWorldPointToScreenHeightCoordinates(
    const AnimationPathEvaluation& evaluation,
    const glm::vec3& worldPoint,
    std::array<float, 2>* projected) {
    if (projected == nullptr) {
        return false;
    }
    const glm::quat orientation =
        QuaternionFromCameraState(evaluation.camera);
    const glm::vec3 cameraLocal =
        glm::inverse(orientation) *
        (worldPoint - ToGlm(evaluation.camera.position));
    const float depth = -cameraLocal.z;
    const float tangentHalfVerticalFov = std::tan(
        0.5F * std::max(
                   glm::radians(evaluation.camera.fovDegrees),
                   0.01F));
    const float denominator =
        2.0F * depth * tangentHalfVerticalFov;
    if (!std::isfinite(denominator) || denominator <= 1.0e-6F) {
        return false;
    }
    *projected = {
        cameraLocal.x / denominator,
        cameraLocal.y / denominator,
    };
    return std::isfinite((*projected)[0U]) &&
           std::isfinite((*projected)[1U]);
}

PerceivedFlowProbe ProbePerceivedFlow(
    const PreparedAnimationPathEvaluationContext& context,
    float timeSeconds,
    float deltaSeconds,
    std::optional<std::array<float, 2>> horizontalRegion =
        std::nullopt) {
    const float leftTime = std::max(0.0F, timeSeconds - deltaSeconds);
    const float rightTime = std::min(context.durationSeconds, timeSeconds + deltaSeconds);
    const float spanSeconds = rightTime - leftTime;
    if (spanSeconds <= 1.0e-6F) {
        return {};
    }

    const auto left = EvaluatePreparedAnimationPath(context, leftTime);
    const auto right = EvaluatePreparedAnimationPath(context, rightTime);
    const auto current = EvaluatePreparedAnimationPath(context, timeSeconds);

    const glm::vec3 cameraVelocity =
        (ToGlm(right.camera.position) - ToGlm(left.camera.position)) / spanSeconds;
    const glm::vec3 leftView = ViewDirectionFromEvaluation(context, left);
    const glm::vec3 rightView = ViewDirectionFromEvaluation(context, right);
    const glm::vec3 currentView = ViewDirectionFromEvaluation(context, current);
    const float viewAlignment = std::clamp(glm::dot(leftView, rightView), -1.0F, 1.0F);
    const float angularSpeed = std::acos(viewAlignment) / spanSeconds;

    const float subjectDistance = std::max(current.focusDistance, 0.05F);
    const glm::vec3 perpendicularVelocity =
        cameraVelocity - (currentView * glm::dot(cameraVelocity, currentView));
    const float flowRadiansPerSecond =
        angularSpeed + (glm::length(perpendicularVelocity) / subjectDistance);
    const float verticalFovRadians = std::max(glm::radians(current.camera.fovDegrees), 0.01F);
    const float screenSpeed = flowRadiansPerSecond / verticalFovRadians;
    PerceivedFlowProbe probe;
    probe.screenSpeed = std::isfinite(screenSpeed) ? std::max(screenSpeed, 0.0F) : 0.0F;
    probe.fullScreenSpeed = probe.screenSpeed;

    const glm::vec3 screenRight =
        QuaternionFromCameraState(current.camera) * glm::vec3{1.0F, 0.0F, 0.0F};
    const glm::vec3 screenUp =
        QuaternionFromCameraState(current.camera) * glm::vec3{0.0F, 1.0F, 0.0F};
    const glm::vec3 viewVelocity = (rightView - leftView) / spanSeconds;
    const std::array<float, 2> rotationDirection{
        -glm::dot(viewVelocity, screenRight),
        -glm::dot(viewVelocity, screenUp),
    };
    const std::array<float, 2> translationDirection{
        -glm::dot(perpendicularVelocity, screenRight) / subjectDistance,
        -glm::dot(perpendicularVelocity, screenUp) / subjectDistance,
    };
    std::array<float, 2> direction{
        rotationDirection[0] + translationDirection[0],
        rotationDirection[1] + translationDirection[1],
    };
    float directionLength = std::hypot(direction[0], direction[1]);
    if (directionLength <= 1.0e-8F) {
        const float rotationLength = std::hypot(rotationDirection[0], rotationDirection[1]);
        const float translationLength = std::hypot(translationDirection[0], translationDirection[1]);
        direction = rotationLength >= translationLength
                        ? rotationDirection
                        : translationDirection;
        directionLength = std::hypot(direction[0], direction[1]);
    }
    if (probe.screenSpeed > 1.0e-8F &&
        directionLength > 1.0e-8F && std::isfinite(directionLength)) {
        probe.screenVelocity = {
            probe.screenSpeed * direction[0] / directionLength,
            probe.screenSpeed * direction[1] / directionLength,
        };
    }

    // Diagnostic flow is sampled independently from the established scalar
    // proxy above. Keeping these paths separate guarantees that adding X/Y,
    // vertical variation, and image rotation cannot retime an animation or
    // change the velocity-blend objective.
    const glm::quat currentOrientation =
        QuaternionFromCameraState(current.camera);
    const float tangentHalfVerticalFov = std::tan(
        0.5F * std::max(
                   glm::radians(current.camera.fovDegrees),
                   0.01F));
    const float aspectRatio =
        std::clamp(context.aspectRatio, 0.25F, 8.0F);
    const float subjectDepth = std::max(current.focusDistance, 0.05F);
    struct DiagnosticGridSample {
        std::array<float, 2> currentScreen{};
        std::array<float, 2> velocity{};
        bool valid = false;
    };
    constexpr std::array<float, 3> kDiagnosticCoordinates{
        -2.0F / 3.0F,
        0.0F,
        2.0F / 3.0F,
    };
    std::array<std::array<DiagnosticGridSample, 3>, 3> diagnosticGrid{};
    for (std::size_t yIndex = 0U; yIndex < 3U; ++yIndex) {
        for (std::size_t xIndex = 0U; xIndex < 3U; ++xIndex) {
            const glm::vec3 cameraLocalPoint{
                kDiagnosticCoordinates[xIndex] * aspectRatio *
                    tangentHalfVerticalFov * subjectDepth,
                kDiagnosticCoordinates[yIndex] * tangentHalfVerticalFov *
                    subjectDepth,
                -subjectDepth,
            };
            const glm::vec3 worldPoint =
                ToGlm(current.camera.position) +
                currentOrientation * cameraLocalPoint;
            auto& gridSample = diagnosticGrid[yIndex][xIndex];
            std::array<float, 2> leftScreen{};
            std::array<float, 2> rightScreen{};
            gridSample.valid =
                ProjectWorldPointToScreenHeightCoordinates(
                    current,
                    worldPoint,
                    &gridSample.currentScreen) &&
                ProjectWorldPointToScreenHeightCoordinates(
                    left,
                    worldPoint,
                    &leftScreen) &&
                ProjectWorldPointToScreenHeightCoordinates(
                    right,
                    worldPoint,
                    &rightScreen);
            if (!gridSample.valid) {
                continue;
            }
            gridSample.velocity = {
                (rightScreen[0U] - leftScreen[0U]) / spanSeconds,
                (rightScreen[1U] - leftScreen[1U]) / spanSeconds,
            };
        }
    }
    std::uint32_t validGridSamples = 0U;
    for (const auto& row : diagnosticGrid) {
        for (const auto& sample : row) {
            if (!sample.valid) {
                continue;
            }
            probe.focusPlaneMeanVelocity[0U] += sample.velocity[0U];
            probe.focusPlaneMeanVelocity[1U] += sample.velocity[1U];
            ++validGridSamples;
        }
    }
    if (validGridSamples > 0U) {
        const float inverseCount =
            1.0F / static_cast<float>(validGridSamples);
        probe.focusPlaneMeanVelocity[0U] *= inverseCount;
        probe.focusPlaneMeanVelocity[1U] *= inverseCount;
        float squaredVariation = 0.0F;
        for (const auto& row : diagnosticGrid) {
            for (const auto& sample : row) {
                if (!sample.valid) {
                    continue;
                }
                const float x = sample.velocity[0U] -
                    probe.focusPlaneMeanVelocity[0U];
                const float y = sample.velocity[1U] -
                    probe.focusPlaneMeanVelocity[1U];
                squaredVariation += x * x + y * y;
            }
        }
        probe.focusPlaneVelocityVariation = std::sqrt(
            squaredVariation * inverseCount);
    }
    const auto diagnosticVelocity = [&](std::size_t yIndex) {
        return diagnosticGrid[yIndex][1U].valid
                   ? diagnosticGrid[yIndex][1U].velocity
                   : std::array<float, 2>{0.0F, 0.0F};
    };
    probe.bottomScreenVelocity = diagnosticVelocity(0U);
    probe.middleScreenVelocity = diagnosticVelocity(1U);
    probe.topScreenVelocity = diagnosticVelocity(2U);
    const auto& leftMiddle = diagnosticGrid[1U][0U];
    const auto& rightMiddle = diagnosticGrid[1U][2U];
    const auto& bottomMiddle = diagnosticGrid[0U][1U];
    const auto& topMiddle = diagnosticGrid[2U][1U];
    if (leftMiddle.valid && rightMiddle.valid &&
        bottomMiddle.valid && topMiddle.valid) {
        const float horizontalSpan =
            rightMiddle.currentScreen[0U] -
            leftMiddle.currentScreen[0U];
        const float verticalSpan =
            topMiddle.currentScreen[1U] -
            bottomMiddle.currentScreen[1U];
        if (std::abs(horizontalSpan) > 1.0e-6F &&
            std::abs(verticalSpan) > 1.0e-6F) {
            const float dVyDx =
                (rightMiddle.velocity[1U] -
                 leftMiddle.velocity[1U]) /
                horizontalSpan;
            const float dVxDy =
                (topMiddle.velocity[0U] -
                 bottomMiddle.velocity[0U]) /
                verticalSpan;
            const float rotationRadiansPerSecond =
                0.5F * (dVyDx - dVxDy);
            if (std::isfinite(rotationRadiansPerSecond)) {
                probe.imageRotationDegreesPerSecond =
                    glm::degrees(rotationRadiansPerSecond);
            }
        }
    }

    if (!horizontalRegion.has_value()) {
        return probe;
    }

    const float regionMinimum = std::clamp(
        std::min(
            horizontalRegion->at(0U),
            horizontalRegion->at(1U)),
        -1.0F,
        1.0F);
    const float regionMaximum = std::clamp(
        std::max(
            horizontalRegion->at(0U),
            horizontalRegion->at(1U)),
        -1.0F,
        1.0F);
    if (regionMaximum - regionMinimum <= 1.0e-5F) {
        return probe;
    }

    // Three stratified horizontal probes and three vertical probes capture
    // yaw/pitch perspective changes across the visible wipe region without
    // rendering either animation. The world samples lie on the current
    // focus plane, which also makes focus edits influence the estimate.
    constexpr std::array<float, 3> kHorizontalStrata{
        1.0F / 6.0F,
        0.5F,
        5.0F / 6.0F,
    };
    constexpr std::array<float, 3> kVerticalSamples{
        -2.0F / 3.0F,
        0.0F,
        2.0F / 3.0F,
    };
    std::array<float, 2> regionalVelocity{0.0F, 0.0F};
    float regionalSquaredSpeed = 0.0F;
    std::uint32_t validSamples = 0U;
    for (const float horizontalAmount : kHorizontalStrata) {
        const float horizontal = std::lerp(
            regionMinimum,
            regionMaximum,
            horizontalAmount);
        for (const float vertical : kVerticalSamples) {
            const glm::vec3 cameraLocalPoint{
                horizontal * aspectRatio *
                    tangentHalfVerticalFov * subjectDepth,
                vertical * tangentHalfVerticalFov * subjectDepth,
                -subjectDepth,
            };
            const glm::vec3 worldPoint =
                ToGlm(current.camera.position) +
                currentOrientation * cameraLocalPoint;
            std::array<float, 2> leftScreen{};
            std::array<float, 2> rightScreen{};
            if (!ProjectWorldPointToScreenHeightCoordinates(
                    left,
                    worldPoint,
                    &leftScreen) ||
                !ProjectWorldPointToScreenHeightCoordinates(
                    right,
                    worldPoint,
                    &rightScreen)) {
                continue;
            }
            const std::array<float, 2> velocity{
                (rightScreen[0U] - leftScreen[0U]) / spanSeconds,
                (rightScreen[1U] - leftScreen[1U]) / spanSeconds,
            };
            const float squaredSpeed =
                velocity[0U] * velocity[0U] +
                velocity[1U] * velocity[1U];
            if (!std::isfinite(squaredSpeed)) {
                continue;
            }
            regionalVelocity[0U] += velocity[0U];
            regionalVelocity[1U] += velocity[1U];
            regionalSquaredSpeed += squaredSpeed;
            ++validSamples;
        }
    }
    if (validSamples > 0U) {
        const float inverseCount =
            1.0F / static_cast<float>(validSamples);
        probe.screenVelocity = {
            regionalVelocity[0U] * inverseCount,
            regionalVelocity[1U] * inverseCount,
        };
        probe.screenSpeed =
            std::sqrt(regionalSquaredSpeed * inverseCount);
    }
    return probe;
}

struct DominantScreenPanAxis {
    std::array<float, 2> direction{1.0F, 0.0F};
    bool valid = false;
};

DominantScreenPanAxis EstimateDominantScreenPanAxis(
    const PreparedAnimationPathEvaluationContext& context,
    std::uint32_t sampleCount,
    float deltaSeconds) {
    DominantScreenPanAxis result;
    if (!context.valid || context.singleKey ||
        context.durationSeconds <= 1.0e-6F) {
        return result;
    }
    const auto samples = std::clamp<std::uint32_t>(
        sampleCount,
        8U,
        2048U);
    std::array<float, 2> signedVelocity{0.0F, 0.0F};
    float totalVelocity = 0.0F;
    float xx = 0.0F;
    float xy = 0.0F;
    float yy = 0.0F;
    for (std::uint32_t sampleIndex = 0U;
         sampleIndex < samples;
         ++sampleIndex) {
        const float timeSeconds = context.durationSeconds *
            (static_cast<float>(sampleIndex) + 0.5F) /
            static_cast<float>(samples);
        const auto probe = ProbePerceivedFlow(
            context,
            timeSeconds,
            deltaSeconds);
        const float x = probe.middleScreenVelocity[0U];
        const float y = probe.middleScreenVelocity[1U];
        if (!std::isfinite(x) || !std::isfinite(y)) {
            continue;
        }
        const float length = std::hypot(x, y);
        signedVelocity[0U] += x;
        signedVelocity[1U] += y;
        totalVelocity += length;
        xx += x * x;
        xy += x * y;
        yy += y * y;
    }
    const float signedLength = std::hypot(
        signedVelocity[0U],
        signedVelocity[1U]);
    if (signedLength > 1.0e-7F &&
        signedLength >= 0.10F * totalVelocity) {
        result.direction = {
            signedVelocity[0U] / signedLength,
            signedVelocity[1U] / signedLength,
        };
        result.valid = true;
        return result;
    }
    if (xx + yy <= 1.0e-10F) {
        return result;
    }
    const float angle = 0.5F * std::atan2(
        2.0F * xy,
        xx - yy);
    result.direction = {std::cos(angle), std::sin(angle)};
    result.valid = true;
    return result;
}

float EqualizationScreenSpeed(
    const PerceivedFlowProbe& probe,
    const AnimationSpeedEqualizationOptions& options,
    const DominantScreenPanAxis& panAxis) {
    if (options.mode ==
        AnimationSpeedEqualizationMode::PerceivedMotion) {
        return probe.screenSpeed;
    }
    const float centerX = probe.middleScreenVelocity[0U];
    const float centerY = probe.middleScreenVelocity[1U];
    const float centerSpeed = std::hypot(centerX, centerY);
    if (options.mode ==
        AnimationSpeedEqualizationMode::CenterScreenPan) {
        return std::isfinite(centerSpeed)
            ? std::max(centerSpeed, 0.0F)
            : 0.0F;
    }

    const float meanSpeed = std::hypot(
        probe.focusPlaneMeanVelocity[0U],
        probe.focusPlaneMeanVelocity[1U]);
    const float baseSpeed = std::max(
        centerSpeed,
        0.5F * meanSpeed);
    const float offAxisSpeed = panAxis.valid
        ? std::abs(
              centerX * panAxis.direction[1U] -
              centerY * panAxis.direction[0U])
        : 0.0F;
    // A roll moves image content in proportion to its distance from the
    // centre. 0.35 screen heights is a representative mid-frame radius.
    const float rollScreenSpeed = 0.35F * std::abs(
        glm::radians(probe.imageRotationDegreesPerSecond));
    const float speed =
        baseSpeed +
        std::max(options.offAxisWeight, 0.0F) * offAxisSpeed +
        std::max(options.flowVariationWeight, 0.0F) *
            probe.focusPlaneVelocityVariation +
        std::max(options.rollWeight, 0.0F) * rollScreenSpeed;
    return std::isfinite(speed) ? std::max(speed, 0.0F) : 0.0F;
}

}  // namespace

AnimationPath BuildAnimationPathFromCameraShots(
    const std::string& name,
    const std::vector<CameraShot>& orderedShots,
    std::uint32_t durationFrames,
    float apertureFStops) {
    AnimationPath path;
    path.name = name.empty() ? "Animation" : name;
    path.durationFrames = std::max<std::uint32_t>(
        durationFrames,
        orderedShots.size() > 1U ? static_cast<std::uint32_t>(orderedShots.size() - 1U) : 1U);
    path.apertureFStops = std::max(0.1F, apertureFStops);
    path.depthOfFieldMaxBlurPixels = std::max(0.0F, path.depthOfFieldMaxBlurPixels);
    path.keys.reserve(orderedShots.size());

    std::size_t keyIndex = 0;
    for (const auto& shot : orderedShots) {
        AnimationPathKey key;
        key.id = "key_" + std::to_string(++keyIndex);
        key.cameraPosition = shot.state.position;
        key.focusPoint = FocusPointFromShot(shot);
        key.fovDegrees = shot.state.fovDegrees;
        key.nearPlane = shot.state.nearPlane;
        key.farPlane = shot.state.farPlane;
        key.durationFrames = std::max<std::uint32_t>(1U, shot.durationFrames);
        key.sourceShotName = shot.name;
        key.linkedCameraId = shot.id;
        key.linkedCameraName = shot.name;
        path.keys.push_back(std::move(key));
    }

    return path;
}

float AnimationPathDurationSeconds(const AnimationPath& path) {
    return static_cast<float>(std::max(path.durationFrames, MinimumAnimationDurationFrames(path))) /
           kAnimationFramesPerSecond;
}

PreparedAnimationPathEvaluationContext PrepareAnimationPathEvaluation(const AnimationPath& path) {
    PreparedAnimationPathEvaluationContext context;
    if (path.keys.empty()) {
        return context;
    }

    context.valid = true;
    context.singleKey = path.keys.size() == 1U;
    context.durationSeconds = AnimationPathDurationSeconds(path);
    context.aspectRatio =
        path.exportSettings.height > 0U
            ? std::clamp(
                  static_cast<float>(path.exportSettings.width) /
                      static_cast<float>(path.exportSettings.height),
                  0.25F,
                  8.0F)
            : 16.0F / 9.0F;
    context.depthOfFieldEnabled = path.depthOfFieldEnabled;
    context.apertureFStops = std::max(0.1F, path.apertureFStops);
    context.depthOfFieldMaxBlurPixels = std::max(0.0F, path.depthOfFieldMaxBlurPixels);
    context.hasOrientation = AnyKeyHasOrientation(path);
    context.hasFocusDistance = AnyKeyHasFocusDistance(path);
    context.hasApertureFStops = AnyKeyHasApertureFStops(path);

    if (context.singleKey) {
        context.singleKeySnapshot = path.keys.front();
        return context;
    }

    context.knots = BuildAnimationKnots(path);
    if (context.knots.size() != path.keys.size() || context.knots.empty()) {
        context.valid = false;
        return context;
    }

    auto cameraX = BuildKeySamples(path, ReadCameraX);
    auto cameraY = BuildKeySamples(path, ReadCameraY);
    auto cameraZ = BuildKeySamples(path, ReadCameraZ);
    auto focusX = BuildKeySamples(path, ReadFocusX);
    auto focusY = BuildKeySamples(path, ReadFocusY);
    auto focusZ = BuildKeySamples(path, ReadFocusZ);
    auto focusDistances = BuildKeySamples(path, ReadFocusDistance);

    auto splineKeys = path.keys;
    if (!path.localizedKeyCorrections.empty() &&
        ValidLocalizedKeyCorrections(path)) {
        context.hasLoopKeyCorrections = true;
        context.loopCorrectionKeyEnabled.assign(path.keys.size(), 0U);
        context.loopCameraCorrections.assign(
            path.keys.size(),
            {0.0F, 0.0F, 0.0F});
        context.loopFocusCorrections.assign(
            path.keys.size(),
            {0.0F, 0.0F, 0.0F});
        for (const auto& correction : path.localizedKeyCorrections) {
            const auto keyIt = std::find_if(
                path.keys.begin(),
                path.keys.end(),
                [&](const AnimationPathKey& key) {
                    return key.id == correction.keyId;
                });
            if (keyIt == path.keys.end()) {
                continue;
            }
            const std::size_t keyIndex = static_cast<std::size_t>(
                std::distance(path.keys.begin(), keyIt));
            context.loopCorrectionKeyEnabled[keyIndex] = 1U;
            context.loopCameraCorrections[keyIndex] = Difference(
                keyIt->cameraPosition,
                correction.splineCameraPosition);
            context.loopFocusCorrections[keyIndex] = Difference(
                keyIt->focusPoint,
                correction.splineFocusPoint);
            cameraX[keyIndex] = correction.splineCameraPosition[0U];
            cameraY[keyIndex] = correction.splineCameraPosition[1U];
            cameraZ[keyIndex] = correction.splineCameraPosition[2U];
            focusX[keyIndex] = correction.splineFocusPoint[0U];
            focusY[keyIndex] = correction.splineFocusPoint[1U];
            focusZ[keyIndex] = correction.splineFocusPoint[2U];
            splineKeys[keyIndex].cameraPosition =
                correction.splineCameraPosition;
            splineKeys[keyIndex].focusPoint =
                correction.splineFocusPoint;
            focusDistances[keyIndex] = ReadFocusDistance(
                splineKeys[keyIndex]);
        }
        context.loopCameraCorrectionTangents =
            BuildLoopCorrectionTangents(
                context.knots,
                context.loopCameraCorrections,
                context.loopCorrectionKeyEnabled);
        context.loopFocusCorrectionTangents =
            BuildLoopCorrectionTangents(
                context.knots,
                context.loopFocusCorrections,
                context.loopCorrectionKeyEnabled);
    }

    context.orientationQuaternions.reserve(path.keys.size());
    for (const auto& key : splineKeys) {
        auto orientation = OrientationFromKey(key);
        if (!context.orientationQuaternions.empty()) {
            const auto previous = QuaternionFromArray(
                context.orientationQuaternions.back());
            if (glm::dot(previous, orientation) < 0.0F) {
                orientation = -orientation;
            }
        }
        context.orientationQuaternions.push_back(
            QuaternionToArray(orientation));
    }
    context.geometryKnots = BuildAnimationGeometryKnots(
        cameraX,
        cameraY,
        cameraZ,
        focusX,
        focusY,
        focusZ,
        context.orientationQuaternions,
        splineKeys);
    context.geometryVelocities = BuildGeometryTimeVelocities(
        context.knots,
        context.geometryKnots);
    if (context.geometryKnots.size() != path.keys.size() ||
        context.geometryVelocities.size() != path.keys.size()) {
        context.valid = false;
        return context;
    }

    const auto authoredEndpointDerivative = [](
        const AnimationPathKey& endpoint,
        float derivative) -> std::optional<float> {
        if (!endpoint.hasSplineEndpointTangent ||
            !std::isfinite(derivative)) {
            return std::nullopt;
        }
        return derivative;
    };
    const auto buildPositionSpline = [&](
        std::vector<float> values,
        std::size_t component,
        bool cameraTrack) {
        const auto endpointDerivative = [&](const AnimationPathKey& endpoint) {
            return authoredEndpointDerivative(
                endpoint,
                cameraTrack
                    ? endpoint.splineCameraEndpointTangent[component]
                    : endpoint.splineFocusEndpointTangent[component]);
        };
        return BuildClampedCubicSpline(
            context.geometryKnots,
            values,
            endpointDerivative(splineKeys.front()),
            endpointDerivative(splineKeys.back()));
    };
    context.cameraX = buildPositionSpline(std::move(cameraX), 0U, true);
    context.cameraY = buildPositionSpline(std::move(cameraY), 1U, true);
    context.cameraZ = buildPositionSpline(std::move(cameraZ), 2U, true);
    context.focusX = buildPositionSpline(std::move(focusX), 0U, false);
    context.focusY = buildPositionSpline(std::move(focusY), 1U, false);
    context.focusZ = buildPositionSpline(std::move(focusZ), 2U, false);
    const auto buildLensSpline = [&](
        std::vector<float> values,
        std::size_t component) {
        return BuildClampedCubicSpline(
            context.geometryKnots,
            values,
            authoredEndpointDerivative(
                splineKeys.front(),
                splineKeys.front()
                    .splineLensEndpointTangent[component]),
            authoredEndpointDerivative(
                splineKeys.back(),
                splineKeys.back()
                    .splineLensEndpointTangent[component]));
    };
    context.fovDegrees = buildLensSpline(
        BuildKeySamples(path, ReadFovDegrees),
        0U);
    context.nearPlane = buildLensSpline(
        BuildKeySamples(path, ReadNearPlane),
        1U);
    context.farPlane = buildLensSpline(
        BuildKeySamples(path, ReadFarPlane),
        2U);
    context.focusDistance = buildLensSpline(
        std::move(focusDistances),
        3U);
    context.apertureFStopsSpline = buildLensSpline(
        BuildKeySamples(path, ReadApertureFStops),
        4U);
    std::vector<float> orientationX;
    std::vector<float> orientationY;
    std::vector<float> orientationZ;
    std::vector<float> orientationW;
    orientationX.reserve(context.orientationQuaternions.size());
    orientationY.reserve(context.orientationQuaternions.size());
    orientationZ.reserve(context.orientationQuaternions.size());
    orientationW.reserve(context.orientationQuaternions.size());
    for (const auto& orientation : context.orientationQuaternions) {
        orientationX.push_back(orientation[0U]);
        orientationY.push_back(orientation[1U]);
        orientationZ.push_back(orientation[2U]);
        orientationW.push_back(orientation[3U]);
    }
    const auto buildOrientationSpline = [&](
        std::vector<float> values,
        std::size_t component) {
        return BuildClampedCubicSpline(
            context.geometryKnots,
            values,
            authoredEndpointDerivative(
                splineKeys.front(),
                splineKeys.front()
                    .splineOrientationEndpointTangent[component]),
            authoredEndpointDerivative(
                splineKeys.back(),
                splineKeys.back()
                    .splineOrientationEndpointTangent[component]));
    };
    context.orientationX = buildOrientationSpline(
        std::move(orientationX),
        0U);
    context.orientationY = buildOrientationSpline(
        std::move(orientationY),
        1U);
    context.orientationZ = buildOrientationSpline(
        std::move(orientationZ),
        2U);
    context.orientationW = buildOrientationSpline(
        std::move(orientationW),
        3U);

    return context;
}

AnimationPathEvaluation EvaluatePreparedAnimationPath(
    const PreparedAnimationPathEvaluationContext& context,
    float timeSeconds) {
    AnimationPathEvaluation evaluation;
    if (!context.valid) {
        return evaluation;
    }

    if (context.singleKey) {
        const auto& key = context.singleKeySnapshot;
        evaluation.focusPoint = key.focusPoint;
        evaluation.focusDistance = glm::length(ToGlm(key.focusPoint) - ToGlm(key.cameraPosition));
        if (key.hasFocusDistance) {
            evaluation.focusDistance = std::max(0.001F, key.focusDistance);
        }
        evaluation.camera.position = key.cameraPosition;
        evaluation.camera.target = key.focusPoint;
        evaluation.camera.orbitCenter = key.focusPoint;
        evaluation.camera.hasOrbitCenter = true;
        WriteQuaternionToCameraState(OrientationFromKey(key), &evaluation.camera);
        evaluation.camera.fovDegrees = key.fovDegrees;
        evaluation.camera.nearPlane = key.nearPlane;
        evaluation.camera.farPlane = key.farPlane;
        evaluation.camera.hasDepthOfField = context.depthOfFieldEnabled;
        evaluation.camera.focusDistance = evaluation.focusDistance;
        evaluation.camera.apertureFStops = key.hasApertureFStops
                                               ? std::max(0.1F, key.apertureFStops)
                                               : context.apertureFStops;
        evaluation.camera.depthOfFieldMaxBlurPixels = context.depthOfFieldMaxBlurPixels;
        return evaluation;
    }

    if (context.knots.empty()) {
        return evaluation;
    }

    const float clampedTimeSeconds = std::clamp(
        timeSeconds,
        context.knots.front(),
        context.knots.back());
    const float geometryPosition = EvaluateGeometryTimeMap(
        context,
        clampedTimeSeconds);
    evaluation.camera.position = {
        EvaluatePreparedScalarSpline(context.geometryKnots, context.cameraX, geometryPosition),
        EvaluatePreparedScalarSpline(context.geometryKnots, context.cameraY, geometryPosition),
        EvaluatePreparedScalarSpline(context.geometryKnots, context.cameraZ, geometryPosition),
    };
    evaluation.focusPoint = {
        EvaluatePreparedScalarSpline(context.geometryKnots, context.focusX, geometryPosition),
        EvaluatePreparedScalarSpline(context.geometryKnots, context.focusY, geometryPosition),
        EvaluatePreparedScalarSpline(context.geometryKnots, context.focusZ, geometryPosition),
    };
    if (context.hasLoopKeyCorrections && context.knots.size() >= 2U) {
        const auto cameraCorrection = EvaluateLoopKeyCorrection(
            context,
            context.loopCameraCorrections,
            context.loopCameraCorrectionTangents,
            clampedTimeSeconds);
        const auto focusCorrection = EvaluateLoopKeyCorrection(
            context,
            context.loopFocusCorrections,
            context.loopFocusCorrectionTangents,
            clampedTimeSeconds);
        for (std::size_t component = 0U; component < 3U; ++component) {
            evaluation.camera.position[component] +=
                cameraCorrection[component];
            evaluation.focusPoint[component] += focusCorrection[component];
        }
    }
    evaluation.camera.target = evaluation.focusPoint;
    evaluation.camera.orbitCenter = evaluation.focusPoint;
    evaluation.camera.hasOrbitCenter = true;

    const auto cameraPosition = ToGlm(evaluation.camera.position);
    const auto focusPoint = ToGlm(evaluation.focusPoint);
    evaluation.focusDistance = glm::length(focusPoint - cameraPosition);
    if (context.hasFocusDistance) {
        evaluation.focusDistance =
            EvaluatePreparedScalarSpline(context.geometryKnots, context.focusDistance, geometryPosition);
    }
    if (context.hasOrientation) {
        WriteQuaternionToCameraState(
            EvaluatePreparedOrientation(context, geometryPosition),
            &evaluation.camera);
    } else {
        WriteQuaternionToCameraState(LookAtOrientation(cameraPosition, focusPoint), &evaluation.camera);
    }

    evaluation.camera.fovDegrees =
        EvaluatePreparedScalarSpline(context.geometryKnots, context.fovDegrees, geometryPosition);
    evaluation.camera.nearPlane =
        EvaluatePreparedScalarSpline(context.geometryKnots, context.nearPlane, geometryPosition);
    evaluation.camera.farPlane =
        EvaluatePreparedScalarSpline(context.geometryKnots, context.farPlane, geometryPosition);
    evaluation.camera.hasDepthOfField = context.depthOfFieldEnabled;
    evaluation.camera.focusDistance = evaluation.focusDistance;
    evaluation.camera.apertureFStops =
        context.hasApertureFStops
            ? EvaluatePreparedScalarSpline(context.geometryKnots, context.apertureFStopsSpline, geometryPosition)
            : context.apertureFStops;
    evaluation.camera.depthOfFieldMaxBlurPixels = context.depthOfFieldMaxBlurPixels;
    return evaluation;
}

AnimationPathMotionStats MeasurePreparedAnimationPathMotion(
    const PreparedAnimationPathEvaluationContext& context,
    float normalizedTime,
    std::uint32_t sampleCount) {
    AnimationPathMotionStats stats;
    if (!context.valid) {
        return stats;
    }

    stats.durationSeconds = context.durationSeconds;
    if (stats.durationSeconds <= 1.0e-6F) {
        return stats;
    }

    const std::uint32_t steps = std::max<std::uint32_t>(1U, sampleCount);
    auto previous = EvaluatePreparedAnimationPath(context, 0.0F);
    for (std::uint32_t step = 1U; step <= steps; ++step) {
        const float timeSeconds =
            stats.durationSeconds * (static_cast<float>(step) / static_cast<float>(steps));
        const auto current = EvaluatePreparedAnimationPath(context, timeSeconds);
        stats.cameraDistance += Distance(previous.camera.position, current.camera.position);
        stats.targetDistance += Distance(previous.focusPoint, current.focusPoint);
        previous = current;
    }

    stats.averageCameraSpeed = stats.cameraDistance / stats.durationSeconds;
    stats.averageTargetSpeed = stats.targetDistance / stats.durationSeconds;

    const float timeSeconds = stats.durationSeconds * std::clamp(normalizedTime, 0.0F, 1.0F);
    const float deltaSeconds = std::min(
        std::max(stats.durationSeconds / static_cast<float>(std::max<std::uint32_t>(steps, 30U)), 1.0F / 240.0F),
        stats.durationSeconds);
    const float leftTime = std::max(0.0F, timeSeconds - deltaSeconds);
    const float rightTime = std::min(stats.durationSeconds, timeSeconds + deltaSeconds);
    const float spanSeconds = rightTime - leftTime;
    if (spanSeconds > 1.0e-6F) {
        const auto left = EvaluatePreparedAnimationPath(context, leftTime);
        const auto right = EvaluatePreparedAnimationPath(context, rightTime);
        stats.currentCameraSpeed = Distance(left.camera.position, right.camera.position) / spanSeconds;
        stats.currentTargetSpeed = Distance(left.focusPoint, right.focusPoint) / spanSeconds;
    }

    return stats;
}

AnimationPathMotionStats MeasureAnimationPathMotion(
    const AnimationPath& path,
    float normalizedTime,
    std::uint32_t sampleCount) {
    return MeasurePreparedAnimationPathMotion(
        PrepareAnimationPathEvaluation(path),
        normalizedTime,
        sampleCount);
}

std::uint32_t AnimationDurationFramesForAverageSpeed(
    const AnimationPath& path,
    AnimationPathMotionTarget target,
    float worldUnitsPerSecond,
    std::uint32_t sampleCount) {
    const std::uint32_t minimumFrames = MinimumAnimationDurationFrames(path);
    if (worldUnitsPerSecond <= 1.0e-5F || path.keys.empty()) {
        return minimumFrames;
    }

    const auto stats = MeasurePreparedAnimationPathMotion(
        PrepareAnimationPathEvaluation(path),
        0.0F,
        sampleCount);
    const float distance =
        target == AnimationPathMotionTarget::Camera ? stats.cameraDistance : stats.targetDistance;
    if (distance <= 1.0e-5F) {
        return minimumFrames;
    }

    const double requestedFrames =
        std::ceil(static_cast<double>(distance) / static_cast<double>(worldUnitsPerSecond) *
                  static_cast<double>(kAnimationFramesPerSecond));
    if (requestedFrames >= static_cast<double>(std::numeric_limits<std::uint32_t>::max())) {
        return std::numeric_limits<std::uint32_t>::max();
    }
    return std::max<std::uint32_t>(minimumFrames, static_cast<std::uint32_t>(requestedFrames));
}

std::vector<AnimationPerceivedFlowSample> MeasurePreparedAnimationPathPerceivedFlow(
    const PreparedAnimationPathEvaluationContext& context,
    std::uint32_t sampleCount) {
    const std::uint32_t samples = std::clamp<std::uint32_t>(sampleCount, 2U, 4096U);
    std::vector<AnimationPerceivedFlowSample> flow(samples);
    for (std::uint32_t sampleIndex = 0U; sampleIndex < samples; ++sampleIndex) {
        flow[sampleIndex].normalizedPosition =
            static_cast<float>(sampleIndex) / static_cast<float>(samples - 1U);
    }
    if (!context.valid || context.singleKey || context.durationSeconds <= 1.0e-6F) {
        return flow;
    }

    const float deltaSeconds = std::min(
        std::max(
            context.durationSeconds / static_cast<float>(std::max<std::uint32_t>(samples, 30U)),
            1.0F / 240.0F),
        context.durationSeconds);
    for (auto& sample : flow) {
        const auto probe = ProbePerceivedFlow(
            context,
            context.durationSeconds * sample.normalizedPosition,
            deltaSeconds);
        sample.screenSpeed = probe.screenSpeed;
        sample.screenVelocity = probe.screenVelocity;
        sample.topScreenVelocity = probe.topScreenVelocity;
        sample.middleScreenVelocity = probe.middleScreenVelocity;
        sample.bottomScreenVelocity = probe.bottomScreenVelocity;
        sample.imageRotationDegreesPerSecond =
            probe.imageRotationDegreesPerSecond;
    }
    return flow;
}

namespace {

AnimationVelocitySpatialEqualizationResult
RedistributeAnimationPathKeysForConstantScreenVelocity(
    AnimationPath* path,
    std::size_t component,
    std::uint32_t sampleCount) {
    AnimationVelocitySpatialEqualizationResult result;
    const char axisName = component == 0U ? 'X' : 'Y';
    if (path == nullptr) {
        result.errorMessage = "The animation path is unavailable.";
        return result;
    }
    if (path->keys.size() < 3U) {
        result.errorMessage =
            "At least three keys are required to redistribute interior "
            "camera and focus positions.";
        return result;
    }

    // Work on a copy so a failed or already-even operation cannot bake
    // correction metadata or otherwise modify the authored path.
    AnimationPath candidate = *path;
    BakeAnimationPathLocalizedCorrections(&candidate);
    const auto context = PrepareAnimationPathEvaluation(candidate);
    if (!context.valid || context.singleKey ||
        context.durationSeconds <= 1.0e-6F ||
        context.knots.size() != candidate.keys.size()) {
        result.errorMessage =
            std::string{"The animation path could not be evaluated for "} +
            axisName + " velocity.";
        return result;
    }

    const auto samples = std::clamp<std::uint32_t>(
        std::max<std::uint32_t>(sampleCount, 65U),
        65U,
        4096U);
    const auto flow = MeasurePreparedAnimationPathPerceivedFlow(
        context,
        samples);
    if (flow.size() < 2U) {
        result.errorMessage =
            std::string{"The animation path produced no usable "} +
            axisName + "-velocity samples.";
        return result;
    }

    std::vector<float> cumulativeTravel(flow.size(), 0.0F);
    for (std::size_t sampleIndex = 1U;
         sampleIndex < flow.size();
         ++sampleIndex) {
        const float intervalSeconds = context.durationSeconds *
            std::max(
                flow[sampleIndex].normalizedPosition -
                    flow[sampleIndex - 1U].normalizedPosition,
                0.0F);
        const float previousSpeed = std::abs(
            flow[sampleIndex - 1U].middleScreenVelocity[component]);
        const float currentSpeed = std::abs(
            flow[sampleIndex].middleScreenVelocity[component]);
        const float intervalTravel =
            std::isfinite(previousSpeed) &&
                    std::isfinite(currentSpeed)
                ? 0.5F * intervalSeconds *
                      (previousSpeed + currentSpeed)
                : 0.0F;
        cumulativeTravel[sampleIndex] =
            cumulativeTravel[sampleIndex - 1U] +
            std::max(intervalTravel, 0.0F);
    }
    result.totalScreenTravel = cumulativeTravel.back();
    if (!std::isfinite(result.totalScreenTravel) ||
        result.totalScreenTravel <= 1.0e-6F) {
        result.errorMessage =
            std::string{"The animation has no measurable centre-screen "} +
            axisName + " travel.";
        return result;
    }

    float previousSourcePosition = 0.0F;
    for (std::size_t keyIndex = 1U;
         keyIndex + 1U < candidate.keys.size();
         ++keyIndex) {
        const float keyTimeFraction = std::clamp(
            context.knots[keyIndex] / context.durationSeconds,
            0.0F,
            1.0F);
        const float targetTravel =
            result.totalScreenTravel * keyTimeFraction;
        const auto rightIt = std::lower_bound(
            cumulativeTravel.begin(),
            cumulativeTravel.end(),
            targetTravel);
        const std::size_t rightIndex =
            rightIt == cumulativeTravel.end()
                ? cumulativeTravel.size() - 1U
                : static_cast<std::size_t>(
                      rightIt - cumulativeTravel.begin());
        const std::size_t leftIndex =
            rightIndex > 0U ? rightIndex - 1U : 0U;
        const float leftTravel = cumulativeTravel[leftIndex];
        const float rightTravel = cumulativeTravel[rightIndex];
        const float amount = rightTravel - leftTravel > 1.0e-8F
            ? std::clamp(
                  (targetTravel - leftTravel) /
                      (rightTravel - leftTravel),
                  0.0F,
                  1.0F)
            : 0.0F;
        float sourcePosition = std::lerp(
            flow[leftIndex].normalizedPosition,
            flow[rightIndex].normalizedPosition,
            amount);
        constexpr float kMinimumSourceSpacing = 1.0e-5F;
        const float maximumSourcePosition =
            1.0F -
            kMinimumSourceSpacing * static_cast<float>(
                candidate.keys.size() - 1U - keyIndex);
        sourcePosition = std::clamp(
            sourcePosition,
            previousSourcePosition + kMinimumSourceSpacing,
            maximumSourcePosition);
        previousSourcePosition = sourcePosition;

        const auto sampled = EvaluatePreparedAnimationPath(
            context,
            context.durationSeconds * sourcePosition);
        auto& key = candidate.keys[keyIndex];
        const float poseScale = std::max(
            1.0F,
            std::max(
                Distance(key.cameraPosition, key.focusPoint),
                Distance(
                    sampled.camera.position,
                    sampled.focusPoint)));
        const bool moved =
            Distance(key.cameraPosition, sampled.camera.position) >
                1.0e-5F * poseScale ||
            Distance(key.focusPoint, sampled.focusPoint) >
                1.0e-5F * poseScale;
        key.cameraPosition = sampled.camera.position;
        key.focusPoint = sampled.focusPoint;
        if (moved) {
            ++result.movedKeyCount;
        }
    }

    result.succeeded = true;
    result.changed = result.movedKeyCount > 0U;
    if (!result.changed) {
        return result;
    }
    RebuildAnimationPathGeometryFromKeys(&candidate);
    *path = std::move(candidate);
    return result;
}

struct CameraSpatialFlowMetrics {
    bool valid = false;
    float xMean = 0.0F;
    float xMeanAbsolute = 0.0F;
    float xDeviationMeanSquare = 0.0F;
    float xDeviationRms = 0.0F;
    float rotationMeanSquare = 0.0F;
    float rotationRms = 0.0F;
    float rotationMinorityMeanSquare = 0.0F;
    float rotationDifferenceMeanSquare = 0.0F;
    std::size_t rotationDirectionChanges = 0U;
};

CameraSpatialFlowMetrics CalculateCameraSpatialFlowMetrics(
    const std::vector<AnimationPerceivedFlowSample>& flow,
    float targetXVelocity,
    float rotationDirectionThreshold) {
    CameraSpatialFlowMetrics metrics;
    double xSum = 0.0;
    double xAbsoluteSum = 0.0;
    double rotationSquareSum = 0.0;
    double positiveRotationSquareSum = 0.0;
    double negativeRotationSquareSum = 0.0;
    std::size_t validCount = 0U;
    for (const auto& sample : flow) {
        const float xVelocity = sample.middleScreenVelocity[0U];
        const float rotation = sample.imageRotationDegreesPerSecond;
        if (!std::isfinite(xVelocity) || !std::isfinite(rotation)) {
            continue;
        }
        xSum += xVelocity;
        xAbsoluteSum += std::abs(xVelocity);
        const double rotationSquare =
            static_cast<double>(rotation) * rotation;
        rotationSquareSum += rotationSquare;
        if (rotation > 0.0F) {
            positiveRotationSquareSum += rotationSquare;
        } else if (rotation < 0.0F) {
            negativeRotationSquareSum += rotationSquare;
        }
        ++validCount;
    }
    if (validCount < 2U) {
        return metrics;
    }

    const double divisor = static_cast<double>(validCount);
    metrics.xMean = static_cast<float>(xSum / divisor);
    metrics.xMeanAbsolute = static_cast<float>(xAbsoluteSum / divisor);
    metrics.rotationMeanSquare = static_cast<float>(
        rotationSquareSum / divisor);
    metrics.rotationRms = std::sqrt(std::max(
        metrics.rotationMeanSquare,
        0.0F));
    metrics.rotationMinorityMeanSquare = static_cast<float>(
        std::min(
            positiveRotationSquareSum,
            negativeRotationSquareSum) /
        divisor);

    const float resolvedTargetX = std::isfinite(targetXVelocity)
        ? targetXVelocity
        : metrics.xMean;
    double xDeviationSquareSum = 0.0;
    double rotationDifferenceSquareSum = 0.0;
    std::size_t rotationDifferenceCount = 0U;
    const float directionThreshold =
        std::isfinite(rotationDirectionThreshold)
            ? std::max(rotationDirectionThreshold, 1.0e-4F)
            : std::max(1.0e-4F, 0.05F * metrics.rotationRms);
    int previousRotationDirection = 0;
    bool havePreviousRotation = false;
    float previousRotation = 0.0F;
    for (const auto& sample : flow) {
        const float xVelocity = sample.middleScreenVelocity[0U];
        const float rotation = sample.imageRotationDegreesPerSecond;
        if (!std::isfinite(xVelocity) || !std::isfinite(rotation)) {
            continue;
        }
        const double xDifference =
            static_cast<double>(xVelocity - resolvedTargetX);
        xDeviationSquareSum += xDifference * xDifference;
        if (havePreviousRotation) {
            const double difference =
                static_cast<double>(rotation - previousRotation);
            rotationDifferenceSquareSum += difference * difference;
            ++rotationDifferenceCount;
        }
        previousRotation = rotation;
        havePreviousRotation = true;

        const int direction = rotation > directionThreshold
            ? 1
            : rotation < -directionThreshold ? -1 : 0;
        if (direction != 0) {
            if (previousRotationDirection != 0 &&
                direction != previousRotationDirection) {
                ++metrics.rotationDirectionChanges;
            }
            previousRotationDirection = direction;
        }
    }
    metrics.xDeviationMeanSquare = static_cast<float>(
        xDeviationSquareSum / divisor);
    metrics.xDeviationRms = std::sqrt(std::max(
        metrics.xDeviationMeanSquare,
        0.0F));
    if (rotationDifferenceCount > 0U) {
        metrics.rotationDifferenceMeanSquare = static_cast<float>(
            rotationDifferenceSquareSum /
            static_cast<double>(rotationDifferenceCount));
    }
    metrics.valid = true;
    return metrics;
}

CameraSpatialFlowMetrics MeasureCameraSpatialFlowMetrics(
    const AnimationPath& path,
    std::uint32_t sampleCount,
    float targetXVelocity,
    float rotationDirectionThreshold =
        std::numeric_limits<float>::quiet_NaN()) {
    const auto context = PrepareAnimationPathEvaluation(path);
    if (!context.valid || context.singleKey ||
        context.durationSeconds <= 1.0e-6F) {
        return {};
    }
    return CalculateCameraSpatialFlowMetrics(
        MeasurePreparedAnimationPathPerceivedFlow(
            context,
            sampleCount),
        targetXVelocity,
        rotationDirectionThreshold);
}

AnimationPath CameraRedistributedPath(
    const AnimationPath& base,
    const PreparedAnimationPathEvaluationContext& sourceContext,
    const std::vector<float>& sourcePositions) {
    AnimationPath candidate = base;
    for (std::size_t keyIndex = 1U;
         keyIndex + 1U < candidate.keys.size();
         ++keyIndex) {
        const auto sampled = EvaluatePreparedAnimationPath(
            sourceContext,
            sourceContext.durationSeconds * sourcePositions[keyIndex]);
        candidate.keys[keyIndex].cameraPosition =
            sampled.camera.position;
    }
    RebuildAnimationPathGeometryFromKeys(&candidate);
    return candidate;
}

float CameraSourceMovementPenalty(
    const std::vector<float>& positions,
    const std::vector<float>& initialPositions) {
    if (positions.size() < 3U ||
        positions.size() != initialPositions.size()) {
        return 0.0F;
    }
    const float typicalSpacing =
        1.0F / static_cast<float>(positions.size() - 1U);
    double squareSum = 0.0;
    for (std::size_t index = 1U;
         index + 1U < positions.size();
         ++index) {
        const double normalizedDifference =
            static_cast<double>(positions[index] - initialPositions[index]) /
            std::max(typicalSpacing, 1.0e-6F);
        squareSum += normalizedDifference * normalizedDifference;
    }
    return static_cast<float>(
        squareSum /
        static_cast<double>(positions.size() - 2U));
}

float CameraSpatialSmoothingObjective(
    const CameraSpatialFlowMetrics& metrics,
    const CameraSpatialFlowMetrics& baseline,
    float rotationScale,
    float xDeviationScale,
    bool equalizeScreenXVelocity,
    float movementPenalty) {
    if (!metrics.valid) {
        return std::numeric_limits<float>::infinity();
    }
    if (metrics.rotationDirectionChanges >
        baseline.rotationDirectionChanges) {
        return std::numeric_limits<float>::infinity();
    }
    const float rotationScaleSquared =
        rotationScale * rotationScale;
    float objective =
        metrics.rotationMeanSquare / rotationScaleSquared +
        1.75F * metrics.rotationMinorityMeanSquare /
            rotationScaleSquared +
        0.10F * metrics.rotationDifferenceMeanSquare /
            rotationScaleSquared +
        0.75F * static_cast<float>(
            metrics.rotationDirectionChanges);
    if (equalizeScreenXVelocity) {
        const float xDeviationScaleSquared =
            xDeviationScale * xDeviationScale;
        objective += 6.0F * metrics.xDeviationMeanSquare /
            xDeviationScaleSquared;

        // Combined mode is Pareto-like: it never accepts a meaningful
        // regression in either graph merely because the other one improved.
        const float allowedRotation =
            baseline.rotationRms + 1.0e-6F;
        if (metrics.rotationRms > allowedRotation) {
            return std::numeric_limits<float>::infinity();
        }
        const float allowedXDeviation =
            baseline.xDeviationRms +
            std::max(
                {1.0e-7F,
                 0.02F * baseline.xDeviationRms,
                 0.0002F * baseline.xMeanAbsolute});
        if (metrics.xDeviationRms > allowedXDeviation) {
            return std::numeric_limits<float>::infinity();
        }
    }
    return objective + 0.002F * movementPenalty;
}

}  // namespace

AnimationVelocitySpatialEqualizationResult
RedistributeAnimationPathKeysForConstantScreenXVelocity(
    AnimationPath* path,
    std::uint32_t sampleCount) {
    return RedistributeAnimationPathKeysForConstantScreenVelocity(
        path,
        0U,
        sampleCount);
}

AnimationVelocitySpatialEqualizationResult
RedistributeAnimationPathKeysForConstantScreenYVelocity(
    AnimationPath* path,
    std::uint32_t sampleCount) {
    return RedistributeAnimationPathKeysForConstantScreenVelocity(
        path,
        1U,
        sampleCount);
}

AnimationCameraSpatialSmoothingResult
OptimizeAnimationCameraKeysForSmoothRotation(
    AnimationPath* path,
    const AnimationCameraSpatialSmoothingOptions& options) {
    AnimationCameraSpatialSmoothingResult result;
    if (path == nullptr) {
        result.errorMessage = "The animation path is unavailable.";
        return result;
    }
    if (path->keys.size() < 3U) {
        result.errorMessage =
            "At least three keys are required to redistribute interior "
            "camera positions.";
        return result;
    }

    AnimationPath base = *path;
    BakeAnimationPathLocalizedCorrections(&base);
    const auto sourceContext = PrepareAnimationPathEvaluation(base);
    if (!sourceContext.valid || sourceContext.singleKey ||
        sourceContext.durationSeconds <= 1.0e-6F ||
        sourceContext.knots.size() != base.keys.size()) {
        result.errorMessage =
            "The animation path could not be evaluated for rotation "
            "smoothing.";
        return result;
    }

    const std::uint32_t sampleCount = std::clamp<std::uint32_t>(
        std::max<std::uint32_t>(options.sampleCount, 65U),
        65U,
        513U);
    const auto baselineWithoutTarget = MeasureCameraSpatialFlowMetrics(
        base,
        sampleCount,
        std::numeric_limits<float>::quiet_NaN());
    if (!baselineWithoutTarget.valid) {
        result.errorMessage =
            "The animation produced no usable rotation samples.";
        return result;
    }
    const float targetXVelocity = baselineWithoutTarget.xMean;
    const float rotationDirectionThreshold = std::max(
        1.0e-4F,
        0.05F * baselineWithoutTarget.rotationRms);
    const auto baseline = MeasureCameraSpatialFlowMetrics(
        base,
        sampleCount,
        targetXVelocity,
        rotationDirectionThreshold);
    if (!baseline.valid) {
        result.errorMessage =
            "The animation produced no usable motion samples.";
        return result;
    }
    if (options.equalizeScreenXVelocity &&
        baseline.xMeanAbsolute <= 1.0e-6F) {
        result.errorMessage =
            "The animation has no measurable centre-screen X velocity.";
        return result;
    }

    result.beforeRotationRmsDegreesPerSecond = baseline.rotationRms;
    result.afterRotationRmsDegreesPerSecond = baseline.rotationRms;
    result.beforeXVelocityDeviation = baseline.xDeviationRms;
    result.afterXVelocityDeviation = baseline.xDeviationRms;
    result.beforeRotationDirectionChanges =
        baseline.rotationDirectionChanges;
    result.afterRotationDirectionChanges =
        baseline.rotationDirectionChanges;
    result.succeeded = true;
    if (baseline.rotationRms <= 1.0e-5F &&
        !options.equalizeScreenXVelocity) {
        return result;
    }

    std::vector<float> initialPositions(base.keys.size(), 0.0F);
    for (std::size_t keyIndex = 0U;
         keyIndex < initialPositions.size();
         ++keyIndex) {
        initialPositions[keyIndex] = std::clamp(
            sourceContext.knots[keyIndex] /
                sourceContext.durationSeconds,
            0.0F,
            1.0F);
    }
    std::vector<float> positions = initialPositions;
    AnimationPath bestPath = base;
    auto bestMetrics = baseline;
    const float rotationScale = std::max(
        baseline.rotationRms,
        0.01F);
    const float xDeviationScale = std::max(
        {baseline.xDeviationRms,
         0.002F * baseline.xMeanAbsolute,
         1.0e-6F});
    float bestObjective = CameraSpatialSmoothingObjective(
        bestMetrics,
        baseline,
        rotationScale,
        xDeviationScale,
        options.equalizeScreenXVelocity,
        0.0F);

    float step = 0.75F /
        static_cast<float>(base.keys.size() - 1U);
    const float minimumStep = 0.0025F /
        static_cast<float>(base.keys.size() - 1U);
    const std::uint32_t sweeps = std::clamp<std::uint32_t>(
        options.optimizationSweeps,
        1U,
        48U);
    constexpr float kMinimumSourceSpacing = 1.0e-5F;
    for (std::uint32_t sweep = 0U;
         sweep < sweeps && step >= minimumStep;
         ++sweep) {
        bool sweepImproved = false;
        for (std::size_t keyOffset = 0U;
             keyOffset + 2U < base.keys.size();
             ++keyOffset) {
            const std::size_t keyIndex = sweep % 2U == 0U
                ? keyOffset + 1U
                : base.keys.size() - 2U - keyOffset;
            const float lower =
                positions[keyIndex - 1U] + kMinimumSourceSpacing;
            const float upper =
                positions[keyIndex + 1U] - kMinimumSourceSpacing;
            if (upper <= lower) {
                continue;
            }

            float keyBestObjective = bestObjective;
            std::vector<float> keyBestPositions = positions;
            AnimationPath keyBestPath = bestPath;
            CameraSpatialFlowMetrics keyBestMetrics = bestMetrics;
            for (const float direction : {-1.0F, 1.0F}) {
                std::vector<float> trialPositions = positions;
                trialPositions[keyIndex] = std::clamp(
                    positions[keyIndex] + direction * step,
                    lower,
                    upper);
                if (std::abs(
                        trialPositions[keyIndex] -
                        positions[keyIndex]) <= 1.0e-7F) {
                    continue;
                }
                auto trialPath = CameraRedistributedPath(
                    base,
                    sourceContext,
                    trialPositions);
                const auto trialMetrics =
                    MeasureCameraSpatialFlowMetrics(
                        trialPath,
                        sampleCount,
                        targetXVelocity,
                        rotationDirectionThreshold);
                const float trialObjective =
                    CameraSpatialSmoothingObjective(
                        trialMetrics,
                        baseline,
                        rotationScale,
                        xDeviationScale,
                        options.equalizeScreenXVelocity,
                        CameraSourceMovementPenalty(
                            trialPositions,
                            initialPositions));
                if (trialObjective + 1.0e-5F <
                    keyBestObjective) {
                    keyBestObjective = trialObjective;
                    keyBestPositions = std::move(trialPositions);
                    keyBestPath = std::move(trialPath);
                    keyBestMetrics = trialMetrics;
                }
            }
            if (keyBestObjective + 1.0e-5F < bestObjective) {
                positions = std::move(keyBestPositions);
                bestPath = std::move(keyBestPath);
                bestMetrics = keyBestMetrics;
                bestObjective = keyBestObjective;
                sweepImproved = true;
            }
        }
        step *= sweepImproved ? 0.82F : 0.50F;
    }

    result.afterRotationRmsDegreesPerSecond = bestMetrics.rotationRms;
    result.afterXVelocityDeviation = bestMetrics.xDeviationRms;
    result.afterRotationDirectionChanges =
        bestMetrics.rotationDirectionChanges;
    for (std::size_t keyIndex = 1U;
         keyIndex + 1U < base.keys.size();
         ++keyIndex) {
        const float poseScale = std::max(
            1.0F,
            Distance(
                base.keys[keyIndex].cameraPosition,
                base.keys[keyIndex].focusPoint));
        if (Distance(
                base.keys[keyIndex].cameraPosition,
                bestPath.keys[keyIndex].cameraPosition) >
            1.0e-5F * poseScale) {
            ++result.movedKeyCount;
        }
    }
    result.changed = result.movedKeyCount > 0U &&
        bestObjective + 1.0e-5F <
            CameraSpatialSmoothingObjective(
                baseline,
                baseline,
                rotationScale,
                xDeviationScale,
                options.equalizeScreenXVelocity,
                0.0F);
    if (result.changed) {
        *path = std::move(bestPath);
    } else {
        result.movedKeyCount = 0U;
        result.afterRotationRmsDegreesPerSecond =
            result.beforeRotationRmsDegreesPerSecond;
        result.afterXVelocityDeviation =
            result.beforeXVelocityDeviation;
        result.afterRotationDirectionChanges =
            result.beforeRotationDirectionChanges;
    }
    return result;
}

std::vector<std::uint32_t> ComputeEqualizedAnimationSegmentFrames(
    const AnimationPath& path,
    const AnimationSpeedEqualizationOptions& options) {
    if (path.keys.size() < 2U || path.durationFrames < 1U) {
        return {};
    }

    const auto segmentCount = path.keys.size() - 1U;
    const auto targetTotalFrames = std::max<std::uint32_t>(
        path.durationFrames,
        static_cast<std::uint32_t>(segmentCount));

    // Midpoint-rule integral of perceived flow per segment; each segment's
    // share of the total frames then follows its share of the total flow.
    std::vector<float> integratedFlow(segmentCount, 0.0F);
    const auto context = PrepareAnimationPathEvaluation(path);
    if (context.valid && !context.singleKey &&
        context.knots.size() == path.keys.size() &&
        context.durationSeconds > 1.0e-6F) {
        const std::uint32_t interiorSamples =
            std::clamp<std::uint32_t>(
                options.samplesPerSegment,
                4U,
                256U);
        const auto totalSamples = interiorSamples * static_cast<std::uint32_t>(segmentCount);
        const float deltaSeconds = std::min(
            std::max(
                context.durationSeconds /
                    static_cast<float>(std::max<std::uint32_t>(totalSamples, 30U)),
                1.0F / 240.0F),
            context.durationSeconds);
        const auto panAxis = options.mode ==
                AnimationSpeedEqualizationMode::StabilizedPan
            ? EstimateDominantScreenPanAxis(
                  context,
                  totalSamples,
                  deltaSeconds)
            : DominantScreenPanAxis{};
        for (std::size_t segmentIndex = 0; segmentIndex < segmentCount; ++segmentIndex) {
            const float stepSeconds =
                (context.knots[segmentIndex + 1U] - context.knots[segmentIndex]) /
                static_cast<float>(interiorSamples);
            if (stepSeconds <= 0.0F) {
                continue;
            }
            float segmentFlow = 0.0F;
            for (std::uint32_t step = 0U; step < interiorSamples; ++step) {
                const float timeSeconds =
                    context.knots[segmentIndex] +
                    ((static_cast<float>(step) + 0.5F) * stepSeconds);
                segmentFlow += EqualizationScreenSpeed(
                    ProbePerceivedFlow(
                        context,
                        timeSeconds,
                        deltaSeconds),
                    options,
                    panAxis);
            }
            integratedFlow[segmentIndex] = segmentFlow * stepSeconds;
        }
    }

    std::vector<std::uint32_t> frames(segmentCount, 1U);
    const float totalFlow =
        std::accumulate(integratedFlow.begin(), integratedFlow.end(), 0.0F);
    if (!std::isfinite(totalFlow) || totalFlow <= 1.0e-6F) {
        // Static camera (or unusable evaluation): fall back to an even split.
        const auto baseFrames = targetTotalFrames / static_cast<std::uint32_t>(segmentCount);
        const auto leftoverFrames = targetTotalFrames % static_cast<std::uint32_t>(segmentCount);
        for (std::size_t segmentIndex = 0; segmentIndex < segmentCount; ++segmentIndex) {
            frames[segmentIndex] = baseFrames + (segmentIndex < leftoverFrames ? 1U : 0U);
        }
        return frames;
    }

    std::vector<float> remainders(segmentCount, 0.0F);
    std::uint32_t assignedFrames = static_cast<std::uint32_t>(segmentCount);
    const std::uint32_t extraFrames = targetTotalFrames - assignedFrames;
    for (std::size_t segmentIndex = 0; segmentIndex < segmentCount; ++segmentIndex) {
        const float idealExtraFrames =
            static_cast<float>(extraFrames) * (integratedFlow[segmentIndex] / totalFlow);
        const auto wholeFrames = static_cast<std::uint32_t>(std::floor(idealExtraFrames));
        frames[segmentIndex] += wholeFrames;
        assignedFrames += wholeFrames;
        remainders[segmentIndex] = idealExtraFrames - static_cast<float>(wholeFrames);
    }
    while (assignedFrames < targetTotalFrames) {
        const auto nextIt = std::max_element(remainders.begin(), remainders.end());
        if (nextIt == remainders.end()) {
            break;
        }
        ++frames[static_cast<std::size_t>(nextIt - remainders.begin())];
        *nextIt = -1.0F;
        ++assignedFrames;
    }
    return frames;
}

std::vector<std::uint32_t> ComputeConstantPerceivedSpeedSegmentFrames(
    const AnimationPath& path,
    std::uint32_t samplesPerSegment) {
    return ComputeEqualizedAnimationSegmentFrames(
        path,
        {
            .mode = AnimationSpeedEqualizationMode::PerceivedMotion,
            .samplesPerSegment = samplesPerSegment,
        });
}

float AnimationPathKeyNormalizedPosition(
    const AnimationPath& path,
    std::size_t keyIndex) {
    if (path.keys.empty()) {
        return 0.0F;
    }
    if (keyIndex + 1U >= path.keys.size()) {
        return 1.0F;
    }
    const auto knots = BuildAnimationKnots(path);
    if (keyIndex >= knots.size() || knots.empty() ||
        knots.back() <= 1.0e-6F) {
        return 0.0F;
    }
    return std::clamp(knots[keyIndex] / knots.back(), 0.0F, 1.0F);
}

namespace {

struct LoopEndpointOccurrence {
    std::size_t pathIndex = 0U;
    std::size_t keyIndex = 0U;
    std::array<float, 3> originalCamera{0.0F, 0.0F, 0.0F};
    std::array<float, 3> originalFocus{0.0F, 0.0F, 0.0F};
    float cameraCap = 0.0F;
    float focusCap = 0.0F;
};

struct LoopEndpointGroup {
    std::string linkedCameraId;
    std::vector<LoopEndpointOccurrence> occurrences;
    std::array<float, 3> cameraPosition{0.0F, 0.0F, 0.0F};
    std::array<float, 3> focusPoint{0.0F, 0.0F, 0.0F};
    float cameraCap = 0.0F;
    float focusCap = 0.0F;
};

void EnsureLocalizedKeyCorrections(
    AnimationPath* path,
    const std::vector<std::size_t>& movableKeyIndices) {
    if (path == nullptr) {
        return;
    }
    for (const auto keyIndex : movableKeyIndices) {
        if (keyIndex >= path->keys.size()) {
            continue;
        }
        const auto& key = path->keys[keyIndex];
        const bool alreadyLocalized = std::any_of(
            path->localizedKeyCorrections.begin(),
            path->localizedKeyCorrections.end(),
            [&](const AnimationLocalizedKeyCorrection& correction) {
                return correction.keyId == key.id;
            });
        if (alreadyLocalized) {
            continue;
        }
        path->localizedKeyCorrections.push_back({
            .keyId = key.id,
            .splineCameraPosition = key.cameraPosition,
            .splineFocusPoint = key.focusPoint,
        });
    }
}

AnimationPath BuildSplineBasePath(const AnimationPath& path) {
    AnimationPath result = path;
    for (const auto& correction : result.localizedKeyCorrections) {
        const auto keyIt = std::find_if(
            result.keys.begin(),
            result.keys.end(),
            [&](const AnimationPathKey& key) {
                return key.id == correction.keyId;
            });
        if (keyIt == result.keys.end()) {
            continue;
        }
        keyIt->cameraPosition = correction.splineCameraPosition;
        keyIt->focusPoint = correction.splineFocusPoint;
    }
    result.localizedKeyCorrections.clear();
    return result;
}

std::optional<std::vector<std::size_t>> ResolveLoopMovableKeyIndices(
    const AnimationPath& path,
    const std::vector<std::string>& requestedKeyIds,
    bool useExplicitSelection,
    std::string* errorMessage) {
    if (!useExplicitSelection) {
        return std::vector<std::size_t>{0U, path.keys.size() - 1U};
    }

    std::vector<std::size_t> indices;
    indices.reserve(requestedKeyIds.size());
    std::unordered_set<std::string> requestedIds;
    for (const auto& keyId : requestedKeyIds) {
        if (keyId.empty() || !requestedIds.insert(keyId).second) {
            if (errorMessage != nullptr) {
                *errorMessage =
                    "Each velocity-enabled key needs a unique persistent ID.";
            }
            return std::nullopt;
        }
        const auto keyIt = std::find_if(
            path.keys.begin(),
            path.keys.end(),
            [&](const AnimationPathKey& key) { return key.id == keyId; });
        if (keyIt == path.keys.end()) {
            if (errorMessage != nullptr) {
                *errorMessage =
                    "A velocity-enabled key no longer exists in its animation.";
            }
            return std::nullopt;
        }
        if (std::count_if(
                path.keys.begin(),
                path.keys.end(),
                [&](const AnimationPathKey& key) { return key.id == keyId; }) !=
            1) {
            if (errorMessage != nullptr) {
                *errorMessage =
                    "Velocity-enabled keys must have unique persistent IDs.";
            }
            return std::nullopt;
        }
        indices.push_back(
            static_cast<std::size_t>(keyIt - path.keys.begin()));
    }
    std::sort(indices.begin(), indices.end());
    return indices;
}

float LocalLoopKeyTrackLength(
    const AnimationPath& path,
    std::size_t keyIndex,
    bool cameraTrack) {
    if (path.keys.size() < 2U || keyIndex >= path.keys.size()) {
        return 0.0F;
    }
    const auto& current = cameraTrack
                              ? path.keys[keyIndex].cameraPosition
                              : path.keys[keyIndex].focusPoint;
    std::array<float, 2> adjacentLengths{
        std::numeric_limits<float>::infinity(),
        std::numeric_limits<float>::infinity(),
    };
    if (keyIndex > 0U) {
        adjacentLengths[0U] = Distance(
            current,
            cameraTrack ? path.keys[keyIndex - 1U].cameraPosition
                        : path.keys[keyIndex - 1U].focusPoint);
    }
    if (keyIndex + 1U < path.keys.size()) {
        adjacentLengths[1U] = Distance(
            current,
            cameraTrack ? path.keys[keyIndex + 1U].cameraPosition
                        : path.keys[keyIndex + 1U].focusPoint);
    }
    float localLength = std::numeric_limits<float>::infinity();
    for (const float length : adjacentLengths) {
        if (std::isfinite(length)) {
            localLength = std::min(localLength, length);
        }
    }
    return std::isfinite(localLength) ? localLength : 0.0F;
}

std::array<float, 3> AverageLoopEndpointPosition(
    const LoopEndpointGroup& group,
    bool cameraTrack) {
    std::array<float, 3> average{0.0F, 0.0F, 0.0F};
    if (group.occurrences.empty()) {
        return average;
    }
    for (const auto& occurrence : group.occurrences) {
        const auto& position =
            cameraTrack ? occurrence.originalCamera : occurrence.originalFocus;
        for (std::size_t component = 0U; component < 3U; ++component) {
            average[component] += position[component];
        }
    }
    const float inverseCount =
        1.0F / static_cast<float>(group.occurrences.size());
    for (auto& component : average) {
        component *= inverseCount;
    }
    return average;
}

bool ProjectLoopEndpointPositionToCaps(
    const LoopEndpointGroup& group,
    bool cameraTrack,
    std::array<float, 3>* position) {
    if (position == nullptr || group.occurrences.empty()) {
        return false;
    }
    constexpr float kCapTolerance = 1.0e-5F;
    for (std::uint32_t iteration = 0U; iteration < 128U; ++iteration) {
        bool adjusted = false;
        for (const auto& occurrence : group.occurrences) {
            const auto& origin =
                cameraTrack ? occurrence.originalCamera : occurrence.originalFocus;
            const float cap =
                cameraTrack ? occurrence.cameraCap : occurrence.focusCap;
            const glm::vec3 offset = ToGlm(*position) - ToGlm(origin);
            const float distance = glm::length(offset);
            if (distance <= cap + kCapTolerance) {
                continue;
            }
            if (cap <= 1.0e-8F || distance <= 1.0e-8F) {
                *position = origin;
            } else {
                const glm::vec3 projected =
                    ToGlm(origin) + offset * (cap / distance);
                *position = {projected.x, projected.y, projected.z};
            }
            adjusted = true;
        }
        if (!adjusted) {
            return true;
        }
    }
    return std::all_of(
        group.occurrences.begin(),
        group.occurrences.end(),
        [&](const LoopEndpointOccurrence& occurrence) {
            const auto& origin =
                cameraTrack ? occurrence.originalCamera : occurrence.originalFocus;
            const float cap =
                cameraTrack ? occurrence.cameraCap : occurrence.focusCap;
            return Distance(origin, *position) <= cap + kCapTolerance;
        });
}

void ApplyLoopEndpointGroups(
    AnimationPath* first,
    AnimationPath* second,
    const std::vector<LoopEndpointGroup>& groups) {
    AnimationPath* paths[] = {first, second};
    for (const auto& group : groups) {
        for (const auto& occurrence : group.occurrences) {
            auto& key = paths[occurrence.pathIndex]->keys[occurrence.keyIndex];
            key.cameraPosition = group.cameraPosition;
            key.focusPoint = group.focusPoint;
        }
    }
}

float LoopProbeDeltaSeconds(
    const PreparedAnimationPathEvaluationContext& context,
    float segmentSeconds) {
    return std::min(
        std::max(segmentSeconds / 64.0F, 1.0F / 240.0F),
        std::max(context.durationSeconds, 1.0F / 240.0F));
}

struct LoopOverlapDurations {
    std::array<float, 2> startSeconds{0.0F, 0.0F};
    std::array<float, 2> endSeconds{0.0F, 0.0F};
};

float ResolveLoopOverlapSeconds(
    float requestedSeconds,
    float defaultSeconds,
    float durationSeconds) {
    const float requested =
        std::isfinite(requestedSeconds) && requestedSeconds > 0.0F
            ? requestedSeconds
            : defaultSeconds;
    return std::clamp(requested, 0.0F, durationSeconds);
}

LoopOverlapDurations ResolveLoopOverlapDurations(
    const PreparedAnimationPathEvaluationContext& first,
    const PreparedAnimationPathEvaluationContext& second,
    const AnimationLoopSmoothingOptions& options) {
    const auto defaultStart = [](const auto& context) {
        return context.knots.size() >= 2U
                   ? context.knots[1U] - context.knots.front()
                   : 0.0F;
    };
    const auto defaultEnd = [](const auto& context) {
        return context.knots.size() >= 2U
                   ? context.knots.back() -
                         context.knots[context.knots.size() - 2U]
                   : 0.0F;
    };
    return {
        .startSeconds = {
            ResolveLoopOverlapSeconds(
                options.firstStartOverlapSeconds,
                defaultStart(first),
                first.durationSeconds),
            ResolveLoopOverlapSeconds(
                options.secondStartOverlapSeconds,
                defaultStart(second),
                second.durationSeconds),
        },
        .endSeconds = {
            ResolveLoopOverlapSeconds(
                options.firstEndOverlapSeconds,
                defaultEnd(first),
                first.durationSeconds),
            ResolveLoopOverlapSeconds(
                options.secondEndOverlapSeconds,
                defaultEnd(second),
                second.durationSeconds),
        },
    };
}

struct LoopScore {
    bool cancelled = false;
    float objective = 0.0F;
    float mismatch = 0.0F;
    std::array<float, 2> seamMismatch{0.0F, 0.0F};
    std::array<float, 2> terminalSpeedRmsChange{0.0F, 0.0F};
};

std::array<float, 2> HorizontalWipeRegion(
    bool outgoing,
    bool panRight,
    float visibleFraction) {
    const float fraction =
        std::clamp(visibleFraction, 0.0F, 1.0F);
    const bool keepLeft = outgoing == panRight;
    return keepLeft
               ? std::array<float, 2>{-1.0F, -1.0F + 2.0F * fraction}
               : std::array<float, 2>{1.0F - 2.0F * fraction, 1.0F};
}

LoopScore ScoreLoopPair(
    const AnimationPath& first,
    const AnimationPath& second,
    const PreparedAnimationPathEvaluationContext& originalFirst,
    const PreparedAnimationPathEvaluationContext& originalSecond,
    const std::vector<LoopEndpointGroup>& groups,
    const LoopOverlapDurations& overlaps,
    const AnimationLoopSmoothingOptions& options) {
    constexpr std::uint32_t kTerminalSampleIntervals = 12U;
    // Keep the already-authored terminal speed curve dominant enough that
    // the optimizer primarily rotates screen motion instead of retuning it.
    constexpr float kOriginalSpeedCurveWeight = 1.50F;
    constexpr float kEndpointMovementWeight = 0.02F;
    const auto candidateFirst = PrepareAnimationPathEvaluation(first);
    const auto candidateSecond = PrepareAnimationPathEvaluation(second);
    const PreparedAnimationPathEvaluationContext* candidates[] = {
        &candidateFirst,
        &candidateSecond,
    };
    const PreparedAnimationPathEvaluationContext* originals[] = {
        &originalFirst,
        &originalSecond,
    };
    LoopScore score;
    if (options.stopToken.stop_requested()) {
        score.cancelled = true;
        return score;
    }
    float totalWeight = 0.0F;
    std::array<float, 2> seamWeights{0.0F, 0.0F};
    std::array<float, 2> speedErrorWeights{0.0F, 0.0F};
    const std::array<std::pair<std::size_t, std::size_t>, 2> seams{{
        {0U, 1U},
        {1U, 0U},
    }};
    for (std::size_t seamIndex = 0U; seamIndex < seams.size(); ++seamIndex) {
        if (options.stopToken.stop_requested()) {
            score.cancelled = true;
            return score;
        }
        const auto [outgoingIndex, incomingIndex] = seams[seamIndex];
        const auto& outgoing = *candidates[outgoingIndex];
        const auto& incoming = *candidates[incomingIndex];
        const auto& originalOutgoing = *originals[outgoingIndex];
        const auto& originalIncoming = *originals[incomingIndex];
        const float overlapSeconds = std::min(
            overlaps.endSeconds[outgoingIndex],
            overlaps.startSeconds[incomingIndex]);
        if (overlapSeconds <= 1.0e-6F) {
            continue;
        }
        const float outgoingDelta =
            LoopProbeDeltaSeconds(outgoing, overlapSeconds);
        const float incomingDelta =
            LoopProbeDeltaSeconds(incoming, overlapSeconds);
        for (std::uint32_t sampleIndex = 0U;
             sampleIndex <= kTerminalSampleIntervals;
             ++sampleIndex) {
            if (options.stopToken.stop_requested()) {
                score.cancelled = true;
                return score;
            }
            const float sampleAmount =
                static_cast<float>(sampleIndex) /
                static_cast<float>(kTerminalSampleIntervals);
            const auto wipeRegions =
                ResolveAnimationLoopHorizontalBlendRegions(
                    sampleAmount,
                    options.panRight);
            const float weight = options.horizontalBlend
                                     ? 4.0F *
                                           wipeRegions.outgoingVisibleFraction *
                                           wipeRegions.incomingVisibleFraction
                                     : (1.0F - sampleAmount) *
                                           (1.0F - sampleAmount);
            if (weight <= 0.0F) {
                continue;
            }
            const float outgoingTime = options.horizontalBlend
                                           ? outgoing.knots.back() -
                                                 overlapSeconds +
                                                 sampleAmount * overlapSeconds
                                           : outgoing.knots.back() -
                                                 sampleAmount * overlapSeconds;
            const float incomingTime =
                incoming.knots.front() + sampleAmount * overlapSeconds;
            const std::optional<std::array<float, 2>> outgoingRegion =
                options.horizontalBlend
                    ? std::optional<std::array<float, 2>>{
                          wipeRegions.outgoingRange}
                    : std::nullopt;
            const std::optional<std::array<float, 2>> incomingRegion =
                options.horizontalBlend
                    ? std::optional<std::array<float, 2>>{
                          wipeRegions.incomingRange}
                    : std::nullopt;
            const auto outgoingFlow = ProbePerceivedFlow(
                outgoing,
                outgoingTime,
                outgoingDelta,
                outgoingRegion);
            const auto incomingFlow = ProbePerceivedFlow(
                incoming,
                incomingTime,
                incomingDelta,
                incomingRegion);
            const auto originalOutgoingFlow = ProbePerceivedFlow(
                originalOutgoing,
                outgoingTime,
                outgoingDelta,
                outgoingRegion);
            const auto originalIncomingFlow = ProbePerceivedFlow(
                originalIncoming,
                incomingTime,
                incomingDelta,
                incomingRegion);
            const float scale = std::max(
                0.5F * (originalOutgoingFlow.screenSpeed + originalIncomingFlow.screenSpeed),
                1.0e-3F);
            const float fullScreenScale = std::max(
                0.5F *
                    (originalOutgoingFlow.fullScreenSpeed +
                     originalIncomingFlow.fullScreenSpeed),
                1.0e-3F);
            const float flowX = outgoingFlow.screenVelocity[0] - incomingFlow.screenVelocity[0];
            const float flowY = outgoingFlow.screenVelocity[1] - incomingFlow.screenVelocity[1];
            const float normalizedMismatch =
                ((flowX * flowX) + (flowY * flowY)) / (scale * scale);
            const float outgoingSpeedError =
                (outgoingFlow.fullScreenSpeed -
                 originalOutgoingFlow.fullScreenSpeed) /
                fullScreenScale;
            const float incomingSpeedError =
                (incomingFlow.fullScreenSpeed -
                 originalIncomingFlow.fullScreenSpeed) /
                fullScreenScale;
            score.seamMismatch[seamIndex] += weight * normalizedMismatch;
            score.terminalSpeedRmsChange[outgoingIndex] +=
                weight * outgoingSpeedError * outgoingSpeedError;
            score.terminalSpeedRmsChange[incomingIndex] +=
                weight * incomingSpeedError * incomingSpeedError;
            speedErrorWeights[outgoingIndex] += weight;
            speedErrorWeights[incomingIndex] += weight;
            score.objective +=
                weight *
                (normalizedMismatch +
                 kOriginalSpeedCurveWeight *
                     ((outgoingSpeedError * outgoingSpeedError) +
                      (incomingSpeedError * incomingSpeedError)));
            totalWeight += weight;
            seamWeights[seamIndex] += weight;
        }
    }
    for (std::size_t seamIndex = 0U;
         seamIndex < score.seamMismatch.size();
         ++seamIndex) {
        if (seamWeights[seamIndex] > 0.0F) {
            score.seamMismatch[seamIndex] /= seamWeights[seamIndex];
        }
    }
    score.mismatch =
        0.5F * (score.seamMismatch[0U] + score.seamMismatch[1U]);
    for (std::size_t pathIndex = 0U;
         pathIndex < score.terminalSpeedRmsChange.size();
         ++pathIndex) {
        if (speedErrorWeights[pathIndex] > 0.0F) {
            score.terminalSpeedRmsChange[pathIndex] = std::sqrt(
                score.terminalSpeedRmsChange[pathIndex] /
                speedErrorWeights[pathIndex]);
        }
    }
    if (totalWeight > 0.0F) {
        score.objective /= totalWeight;
    }
    for (const auto& group : groups) {
        float movementPenalty = 0.0F;
        for (const auto& occurrence : group.occurrences) {
            if (occurrence.cameraCap > 1.0e-8F) {
                const float normalized =
                    Distance(group.cameraPosition, occurrence.originalCamera) /
                    occurrence.cameraCap;
                movementPenalty += normalized * normalized;
            }
            if (occurrence.focusCap > 1.0e-8F) {
                const float normalized =
                    Distance(group.focusPoint, occurrence.originalFocus) /
                    occurrence.focusCap;
                movementPenalty += normalized * normalized;
            }
        }
        if (!group.occurrences.empty()) {
            score.objective +=
                kEndpointMovementWeight * movementPenalty /
                static_cast<float>(group.occurrences.size());
        }
    }
    return score;
}

struct LoopMovementSummary {
    float maxCameraMove = 0.0F;
    float maxFocusMove = 0.0F;
    float maxCameraCapUsage = 0.0F;
    float maxFocusCapUsage = 0.0F;
    std::vector<AnimationLoopKeyMovement> keys;
};

LoopMovementSummary MeasureLoopKeyMovements(
    const AnimationPath& before,
    const AnimationPath& after,
    const std::vector<std::size_t>& movableKeyIndices,
    float requestedMoveFraction) {
    LoopMovementSummary summary;
    if (before.keys.size() < 3U ||
        before.keys.size() != after.keys.size()) {
        return summary;
    }
    const float moveFraction = std::clamp(
        requestedMoveFraction,
        0.01F,
        0.25F);
    summary.keys.reserve(movableKeyIndices.size());
    for (const auto keyIndex : movableKeyIndices) {
        if (keyIndex >= before.keys.size()) {
            continue;
        }
        const auto& originalKey = before.keys[keyIndex];
        const auto& currentKey = after.keys[keyIndex];
        if (currentKey.id != originalKey.id) {
            continue;
        }
        const float cameraMove = Distance(
            currentKey.cameraPosition,
            originalKey.cameraPosition);
        const float focusMove = Distance(
            currentKey.focusPoint,
            originalKey.focusPoint);
        const float cameraCap = moveFraction *
                                LocalLoopKeyTrackLength(
                                    before,
                                    keyIndex,
                                    true);
        const float focusCap = moveFraction *
                               LocalLoopKeyTrackLength(
                                   before,
                                   keyIndex,
                                   false);
        AnimationLoopKeyMovement movement{
            .keyId = originalKey.id,
            .cameraMove = cameraMove,
            .focusMove = focusMove,
            .cameraCapUsage = cameraCap > 1.0e-8F
                                  ? cameraMove / cameraCap
                                  : 0.0F,
            .focusCapUsage = focusCap > 1.0e-8F
                                 ? focusMove / focusCap
                                 : 0.0F,
        };
        summary.maxCameraMove =
            std::max(summary.maxCameraMove, movement.cameraMove);
        summary.maxFocusMove =
            std::max(summary.maxFocusMove, movement.focusMove);
        summary.maxCameraCapUsage = std::max(
            summary.maxCameraCapUsage,
            movement.cameraCapUsage);
        summary.maxFocusCapUsage = std::max(
            summary.maxFocusCapUsage,
            movement.focusCapUsage);
        summary.keys.push_back(std::move(movement));
    }
    return summary;
}

std::vector<AnimationLoopScreenDisplacementSample>
MeasureLoopScreenDisplacement(
    const PreparedAnimationPathEvaluationContext& original,
    const PreparedAnimationPathEvaluationContext& candidate,
    float* maximumDisplacement) {
    std::vector<AnimationLoopScreenDisplacementSample> samples;
    if (!original.valid || !candidate.valid ||
        original.durationSeconds <= 1.0e-6F) {
        return samples;
    }
    const std::uint32_t totalFrames = std::max<std::uint32_t>(
        1U,
        static_cast<std::uint32_t>(
            std::round(original.durationSeconds * kAnimationFramesPerSecond)));
    for (std::uint32_t sampledFrame = 0U;;) {
        const float normalizedPosition =
            static_cast<float>(sampledFrame) /
            static_cast<float>(totalFrames);
        const float timeSeconds =
            normalizedPosition * original.durationSeconds;
        const auto before =
            EvaluatePreparedAnimationPath(original, timeSeconds);
        const auto after =
            EvaluatePreparedAnimationPath(candidate, timeSeconds);
        const glm::vec3 beforeView =
            ViewDirectionFromEvaluation(original, before);
        const glm::vec3 afterView =
            ViewDirectionFromEvaluation(candidate, after);
        const float viewAngle = std::acos(std::clamp(
            glm::dot(beforeView, afterView),
            -1.0F,
            1.0F));
        const glm::vec3 cameraOffset =
            ToGlm(after.camera.position) -
            ToGlm(before.camera.position);
        const glm::vec3 perpendicularOffset =
            cameraOffset - beforeView * glm::dot(cameraOffset, beforeView);
        const float translationAngle =
            glm::length(perpendicularOffset) /
            std::max(before.focusDistance, 0.05F);
        const float verticalFovRadians =
            std::max(glm::radians(before.camera.fovDegrees), 0.01F);
        const float displacement =
            (viewAngle + translationAngle) / verticalFovRadians;
        samples.push_back({
            .normalizedPosition = normalizedPosition,
            .screenDisplacement =
                std::isfinite(displacement) ? displacement : 0.0F,
        });
        if (maximumDisplacement != nullptr) {
            *maximumDisplacement = std::max(
                *maximumDisplacement,
                samples.back().screenDisplacement);
        }
        if (sampledFrame == totalFrames) {
            break;
        }
        sampledFrame = std::min(totalFrames, sampledFrame + 10U);
    }
    return samples;
}

}  // namespace

AnimationLoopHorizontalBlendRegions
ResolveAnimationLoopHorizontalBlendRegions(
    float blendProgress,
    bool panRight) {
    const float progress = std::clamp(
        std::isfinite(blendProgress) ? blendProgress : 0.0F,
        0.0F,
        1.0F);
    const float outgoingVisibleFraction =
        std::lerp(2.0F / 3.0F, 1.0F / 3.0F, progress);
    const float incomingVisibleFraction =
        1.0F - outgoingVisibleFraction;
    return {
        .outgoingVisibleFraction = outgoingVisibleFraction,
        .incomingVisibleFraction = incomingVisibleFraction,
        .outgoingRange = HorizontalWipeRegion(
            true,
            panRight,
            outgoingVisibleFraction),
        .incomingRange = HorizontalWipeRegion(
            false,
            panRight,
            incomingVisibleFraction),
    };
}

AnimationLoopSmoothingResult SmoothAnimationLoopTransitions(
    AnimationPath* first,
    AnimationPath* second,
    const AnimationLoopSmoothingOptions& options) {
    AnimationLoopSmoothingResult result;
    if (options.stopToken.stop_requested()) {
        result.errorMessage = "Velocity-alignment preview cancelled.";
        return result;
    }
    if (first == nullptr || second == nullptr || first == second) {
        result.errorMessage = "Choose two different animations.";
        return result;
    }
    if (first->keys.size() < 3U || second->keys.size() < 3U) {
        result.errorMessage =
            "Velocity alignment requires at least three camera keys in each animation.";
        return result;
    }
    const auto hasStableEndpointIds = [](const AnimationPath& path) {
        return !path.keys.front().id.empty() &&
               !path.keys.back().id.empty() &&
               path.keys.front().id != path.keys.back().id;
    };
    if (!hasStableEndpointIds(*first) || !hasStableEndpointIds(*second)) {
        result.errorMessage =
            "Velocity alignment requires distinct persistent IDs on each path's endpoint keys.";
        return result;
    }
    const float requestedMoveFraction =
        std::isfinite(options.maxEndMoveFraction)
            ? options.maxEndMoveFraction
            : 0.10F;
    const float moveFraction =
        std::clamp(requestedMoveFraction, 0.01F, 0.25F);
    const auto originalFirst = PrepareAnimationPathEvaluation(*first);
    const auto originalSecond = PrepareAnimationPathEvaluation(*second);
    if (!originalFirst.valid || !originalSecond.valid ||
        originalFirst.knots.size() < 3U || originalSecond.knots.size() < 3U) {
        result.errorMessage = "One of the animation paths cannot be evaluated.";
        return result;
    }
    const auto firstMovableIndices = ResolveLoopMovableKeyIndices(
        *first,
        options.firstMovableKeyIds,
        options.useExplicitKeySelection,
        &result.errorMessage);
    const auto secondMovableIndices = ResolveLoopMovableKeyIndices(
        *second,
        options.secondMovableKeyIds,
        options.useExplicitKeySelection,
        &result.errorMessage);
    if (!firstMovableIndices.has_value() ||
        !secondMovableIndices.has_value()) {
        return result;
    }
    if (firstMovableIndices->empty() && secondMovableIndices->empty()) {
        result.errorMessage =
            "Enable at least one eligible camera key on either timeline.";
        return result;
    }
    const auto overlaps = ResolveLoopOverlapDurations(
        originalFirst,
        originalSecond,
        options);

    AnimationPath originalFirstPath = *first;
    AnimationPath originalSecondPath = *second;
    AnimationPath* paths[] = {first, second};
    std::vector<LoopEndpointGroup> groups;
    for (std::size_t pathIndex = 0U; pathIndex < 2U; ++pathIndex) {
        auto& path = *paths[pathIndex];
        const auto& movableIndices = pathIndex == 0U
                                         ? firstMovableIndices.value()
                                         : secondMovableIndices.value();
        for (const std::size_t keyIndex : movableIndices) {
            const auto& key = path.keys[keyIndex];
            LoopEndpointOccurrence occurrence{
                .pathIndex = pathIndex,
                .keyIndex = keyIndex,
                .originalCamera = key.cameraPosition,
                .originalFocus = key.focusPoint,
                .cameraCap = moveFraction *
                             LocalLoopKeyTrackLength(
                                 path,
                                 keyIndex,
                                 true),
                .focusCap = moveFraction *
                            LocalLoopKeyTrackLength(
                                path,
                                keyIndex,
                                false),
            };
            auto groupIt = key.linkedCameraId.empty()
                               ? groups.end()
                               : std::find_if(
                                     groups.begin(),
                                     groups.end(),
                                     [&](const LoopEndpointGroup& group) {
                                         return group.linkedCameraId == key.linkedCameraId;
                                     });
            if (groupIt == groups.end()) {
                groups.push_back({
                    .linkedCameraId = key.linkedCameraId,
                    .occurrences = {occurrence},
                    .cameraPosition = occurrence.originalCamera,
                    .focusPoint = occurrence.originalFocus,
                    .cameraCap = occurrence.cameraCap,
                    .focusCap = occurrence.focusCap,
                });
                continue;
            }
            groupIt->occurrences.push_back(occurrence);
            groupIt->cameraCap = std::min(groupIt->cameraCap, occurrence.cameraCap);
            groupIt->focusCap = std::min(groupIt->focusCap, occurrence.focusCap);
        }
    }
    for (auto& group : groups) {
        group.cameraPosition = AverageLoopEndpointPosition(group, true);
        group.focusPoint = AverageLoopEndpointPosition(group, false);
        if (!ProjectLoopEndpointPositionToCaps(group, true, &group.cameraPosition) ||
            !ProjectLoopEndpointPositionToCaps(group, false, &group.focusPoint)) {
            *first = std::move(originalFirstPath);
            *second = std::move(originalSecondPath);
            result.errorMessage =
                "A shared movable camera's saved poses are farther apart "
                "than the selected Max End Move allows.";
            return result;
        }
    }

    // Each visible adjusted key is evaluated as a compact correction over
    // the preserved distance-parameterized C2 spline. Adding a zero-offset
    // record here changes no current frame, but keeps every interval that
    // does not touch a movable key exactly on its pre-alignment spline
    // throughout search.
    EnsureLocalizedKeyCorrections(first, firstMovableIndices.value());
    EnsureLocalizedKeyCorrections(second, secondMovableIndices.value());

    const std::vector<LoopEndpointGroup> noMovementGroups;
    const auto beforeScore = ScoreLoopPair(
        *first,
        *second,
        originalFirst,
        originalSecond,
        noMovementGroups,
        overlaps,
        options);
    if (beforeScore.cancelled) {
        *first = std::move(originalFirstPath);
        *second = std::move(originalSecondPath);
        result.errorMessage = "Velocity-alignment preview cancelled.";
        return result;
    }
    result.beforeMismatch = beforeScore.mismatch;
    result.beforeSeamMismatch = beforeScore.seamMismatch;
    ApplyLoopEndpointGroups(first, second, groups);
    LoopScore bestScore = ScoreLoopPair(
        *first,
        *second,
        originalFirst,
        originalSecond,
        groups,
        overlaps,
        options);
    if (bestScore.cancelled) {
        *first = std::move(originalFirstPath);
        *second = std::move(originalSecondPath);
        result.errorMessage = "Velocity-alignment preview cancelled.";
        return result;
    }
    float stepFraction = 0.25F;
    const std::uint32_t maxSweeps = std::clamp<std::uint32_t>(
        options.maxOptimizationSweeps,
        1U,
        80U);
    const float minimumStepFraction = std::clamp(
        std::isfinite(options.minimumStepFraction)
            ? options.minimumStepFraction
            : 0.005F,
        0.001F,
        0.25F);
    for (std::uint32_t sweep = 0U;
         sweep < maxSweeps && stepFraction >= minimumStepFraction;
         ++sweep) {
        if (options.stopToken.stop_requested()) {
            *first = std::move(originalFirstPath);
            *second = std::move(originalSecondPath);
            result.errorMessage = "Velocity-alignment preview cancelled.";
            return result;
        }
        bool improved = false;
        for (std::size_t groupIndex = 0U; groupIndex < groups.size(); ++groupIndex) {
            for (const bool cameraTrack : {true, false}) {
                const float cap = cameraTrack ? groups[groupIndex].cameraCap : groups[groupIndex].focusCap;
                if (cap <= 1.0e-8F) {
                    continue;
                }
                for (std::size_t component = 0U; component < 3U; ++component) {
                    auto& position = cameraTrack
                                         ? groups[groupIndex].cameraPosition
                                         : groups[groupIndex].focusPoint;
                    const auto savedPosition = position;
                    auto selectedPosition = savedPosition;
                    LoopScore selectedScore = bestScore;
                    for (const float direction : {-1.0F, 1.0F}) {
                        position = savedPosition;
                        position[component] += direction * cap * stepFraction;
                        if (!ProjectLoopEndpointPositionToCaps(
                                groups[groupIndex],
                                cameraTrack,
                                &position)) {
                            continue;
                        }
                        ApplyLoopEndpointGroups(first, second, groups);
                        const auto candidateScore =
                            ScoreLoopPair(
                                *first,
                                *second,
                                originalFirst,
                                originalSecond,
                                groups,
                                overlaps,
                                options);
                        if (candidateScore.cancelled) {
                            *first = std::move(originalFirstPath);
                            *second = std::move(originalSecondPath);
                            result.errorMessage =
                                "Velocity-alignment preview cancelled.";
                            return result;
                        }
                        if (candidateScore.objective + 1.0e-7F < selectedScore.objective) {
                            selectedPosition = position;
                            selectedScore = candidateScore;
                        }
                    }
                    position = selectedPosition;
                    ApplyLoopEndpointGroups(first, second, groups);
                    if (selectedScore.objective + 1.0e-7F < bestScore.objective) {
                        bestScore = selectedScore;
                        improved = true;
                    }
                }
            }
        }
        if (!improved) {
            stepFraction *= 0.5F;
        }
    }

    ApplyLoopEndpointGroups(first, second, groups);
    const auto finalScore = ScoreLoopPair(
        *first,
        *second,
        originalFirst,
        originalSecond,
        groups,
        overlaps,
        options);
    if (finalScore.cancelled) {
        *first = std::move(originalFirstPath);
        *second = std::move(originalSecondPath);
        result.errorMessage = "Velocity-alignment preview cancelled.";
        return result;
    }
    result.afterMismatch = finalScore.mismatch;
    result.afterSeamMismatch = finalScore.seamMismatch;
    result.terminalSpeedRmsChange = finalScore.terminalSpeedRmsChange;
    const AnimationPath* smoothedPaths[] = {first, second};
    const PreparedAnimationPathEvaluationContext* originals[] = {
        &originalFirst,
        &originalSecond,
    };
    for (std::size_t pathIndex = 0U; pathIndex < 2U; ++pathIndex) {
        const auto movement = MeasureLoopKeyMovements(
            pathIndex == 0U ? originalFirstPath : originalSecondPath,
            *smoothedPaths[pathIndex],
            pathIndex == 0U
                ? firstMovableIndices.value()
                : secondMovableIndices.value(),
            moveFraction);
        result.maxCameraMove =
            std::max(result.maxCameraMove, movement.maxCameraMove);
        result.maxFocusMove =
            std::max(result.maxFocusMove, movement.maxFocusMove);
        result.maxCameraCapUsage = std::max(
            result.maxCameraCapUsage,
            movement.maxCameraCapUsage);
        result.maxFocusCapUsage = std::max(
            result.maxFocusCapUsage,
            movement.maxFocusCapUsage);
        result.keyMovements[pathIndex] = movement.keys;
        const auto candidateContext =
            PrepareAnimationPathEvaluation(*smoothedPaths[pathIndex]);
        result.screenDisplacementSamples[pathIndex] =
            MeasureLoopScreenDisplacement(
                *originals[pathIndex],
                candidateContext,
                &result.maxScreenDisplacement[pathIndex]);
    }
    const auto seamDidNotWorsen = [&](std::size_t seamIndex) {
        const float tolerance = std::max(
            1.0e-7F,
            1.0e-4F * result.beforeSeamMismatch[seamIndex]);
        return result.afterSeamMismatch[seamIndex] <=
               result.beforeSeamMismatch[seamIndex] + tolerance;
    };
    const bool neitherSeamWorsened =
        seamDidNotWorsen(0U) && seamDidNotWorsen(1U);
    const float requiredOverallImprovement = std::max(
        1.0e-7F,
        1.0e-4F * result.beforeMismatch);
    const bool overallImproved =
        result.afterMismatch + requiredOverallImprovement <
        result.beforeMismatch;
    const bool selectedKeysMoved =
        result.maxCameraMove > 1.0e-6F ||
        result.maxFocusMove > 1.0e-6F;
    result.changed =
        overallImproved && neitherSeamWorsened && selectedKeysMoved;
    result.succeeded = true;
    if (!result.changed) {
        *first = std::move(originalFirstPath);
        *second = std::move(originalSecondPath);
        if (!selectedKeysMoved) {
            result.errorMessage =
                "No enabled-key movement produced a measurable improvement "
                "within the selected limit.";
        } else if (!overallImproved) {
            result.errorMessage =
                "The best bounded key movement did not measurably lower "
                "the combined screen-flow mismatch.";
        } else {
            result.errorMessage =
                "The best combined candidate was rejected because it made "
                "one seam direction worse.";
        }
    }
    return result;
}

AnimationLoopTransitionMetrics MeasureAnimationLoopTransitions(
    const AnimationPath& first,
    const AnimationPath& second,
    const AnimationLoopSmoothingOptions& options) {
    AnimationLoopTransitionMetrics metrics;
    if (&first == &second) {
        metrics.errorMessage = "Choose two different animations.";
        return metrics;
    }
    if (first.keys.size() < 3U || second.keys.size() < 3U) {
        metrics.errorMessage =
            "Velocity-blend validation requires at least three camera keys in each animation.";
        return metrics;
    }

    const bool firstHasLink = first.velocityBlendLink.has_value();
    const bool secondHasLink = second.velocityBlendLink.has_value();
    if (firstHasLink != secondHasLink) {
        metrics.errorMessage =
            "Only one animation contains velocity-blend link metadata.";
        return metrics;
    }
    if (firstHasLink &&
        (first.velocityBlendLink->pairId.empty() ||
         first.velocityBlendLink->pairId != second.velocityBlendLink->pairId ||
         first.velocityBlendLink->horizontalBlend !=
             second.velocityBlendLink->horizontalBlend ||
         first.velocityBlendLink->panRight !=
             second.velocityBlendLink->panRight)) {
        metrics.errorMessage =
            "The animations do not contain the same valid velocity-blend pair.";
        return metrics;
    }
    if (!ValidLocalizedKeyCorrections(first) ||
        !ValidLocalizedKeyCorrections(second)) {
        metrics.errorMessage =
            "One animation contains invalid localized key corrections.";
        return metrics;
    }

    AnimationPath originalFirst = BuildSplineBasePath(first);
    AnimationPath originalSecond = BuildSplineBasePath(second);
    const auto originalFirstContext =
        PrepareAnimationPathEvaluation(originalFirst);
    const auto originalSecondContext =
        PrepareAnimationPathEvaluation(originalSecond);
    if (!originalFirstContext.valid || !originalSecondContext.valid ||
        originalFirstContext.knots.size() < 3U ||
        originalSecondContext.knots.size() < 3U) {
        metrics.errorMessage = "One of the animation paths cannot be evaluated.";
        return metrics;
    }
    AnimationLoopSmoothingOptions measurementOptions = options;
    if (firstHasLink) {
        measurementOptions.firstStartOverlapSeconds =
            first.velocityBlendLink->startOverlapSeconds;
        measurementOptions.firstEndOverlapSeconds =
            first.velocityBlendLink->endOverlapSeconds;
        measurementOptions.secondStartOverlapSeconds =
            second.velocityBlendLink->startOverlapSeconds;
        measurementOptions.secondEndOverlapSeconds =
            second.velocityBlendLink->endOverlapSeconds;
        measurementOptions.horizontalBlend =
            first.velocityBlendLink->horizontalBlend;
        measurementOptions.panRight =
            first.velocityBlendLink->panRight;
    }
    const auto overlaps = ResolveLoopOverlapDurations(
        originalFirstContext,
        originalSecondContext,
        measurementOptions);

    const std::vector<LoopEndpointGroup> noMovementGroups;
    const auto beforeScore = ScoreLoopPair(
        originalFirst,
        originalSecond,
        originalFirstContext,
        originalSecondContext,
        noMovementGroups,
        overlaps,
        measurementOptions);
    const bool hasLocalizedCorrections =
        !first.localizedKeyCorrections.empty() ||
        !second.localizedKeyCorrections.empty();
    const auto afterScore = hasLocalizedCorrections
                                ? ScoreLoopPair(
                                      first,
                                      second,
                                      originalFirstContext,
                                      originalSecondContext,
                                      noMovementGroups,
                                      overlaps,
                                      measurementOptions)
                                : beforeScore;
    metrics.valid = true;
    metrics.hasAppliedSmoothing = hasLocalizedCorrections;
    metrics.beforeMismatch = beforeScore.mismatch;
    metrics.afterMismatch = afterScore.mismatch;
    metrics.beforeSeamMismatch = beforeScore.seamMismatch;
    metrics.afterSeamMismatch = afterScore.seamMismatch;
    metrics.terminalSpeedRmsChange =
        afterScore.terminalSpeedRmsChange;
    if (hasLocalizedCorrections) {
        const AnimationPath* paths[] = {&first, &second};
        const AnimationPath* basePaths[] = {
            &originalFirst,
            &originalSecond,
        };
        const PreparedAnimationPathEvaluationContext* originals[] = {
            &originalFirstContext,
            &originalSecondContext,
        };
        for (std::size_t pathIndex = 0U; pathIndex < 2U; ++pathIndex) {
            std::vector<std::size_t> correctedKeyIndices;
            correctedKeyIndices.reserve(
                paths[pathIndex]->localizedKeyCorrections.size());
            for (const auto& correction :
                 paths[pathIndex]->localizedKeyCorrections) {
                const auto keyIt = std::find_if(
                    paths[pathIndex]->keys.begin(),
                    paths[pathIndex]->keys.end(),
                    [&](const AnimationPathKey& key) {
                        return key.id == correction.keyId;
                    });
                if (keyIt != paths[pathIndex]->keys.end()) {
                    correctedKeyIndices.push_back(
                        static_cast<std::size_t>(std::distance(
                            paths[pathIndex]->keys.begin(),
                            keyIt)));
                }
            }
            const float moveFraction =
                paths[pathIndex]->velocityBlendLink.has_value()
                    ? paths[pathIndex]
                          ->velocityBlendLink->maxEndMoveFraction
                    : measurementOptions.maxEndMoveFraction;
            const auto movement = MeasureLoopKeyMovements(
                *basePaths[pathIndex],
                *paths[pathIndex],
                correctedKeyIndices,
                moveFraction);
            metrics.maxCameraMove = std::max(
                metrics.maxCameraMove,
                movement.maxCameraMove);
            metrics.maxFocusMove = std::max(
                metrics.maxFocusMove,
                movement.maxFocusMove);
            metrics.maxCameraCapUsage = std::max(
                metrics.maxCameraCapUsage,
                movement.maxCameraCapUsage);
            metrics.maxFocusCapUsage = std::max(
                metrics.maxFocusCapUsage,
                movement.maxFocusCapUsage);
            metrics.keyMovements[pathIndex] = movement.keys;
            const auto candidateContext =
                PrepareAnimationPathEvaluation(*paths[pathIndex]);
            metrics.screenDisplacementSamples[pathIndex] =
                MeasureLoopScreenDisplacement(
                    *originals[pathIndex],
                    candidateContext,
                    &metrics.maxScreenDisplacement[pathIndex]);
        }
    }
    return metrics;
}

namespace {

struct StrongScreenProjection {
    glm::vec2 uv{0.0F, 0.0F};
    float depth = 0.0F;
    bool visible = false;
};

StrongScreenProjection ProjectStrongAlignmentPoint(
    const AnimationPathEvaluation& evaluation,
    const glm::vec3& point,
    float aspectRatio) {
    StrongScreenProjection projection;
    const glm::vec3 local =
        glm::inverse(QuaternionFromCameraState(evaluation.camera)) *
        (point - ToGlm(evaluation.camera.position));
    projection.depth = -local.z;
    const float tangent = std::tan(
        0.5F * std::max(
                   glm::radians(evaluation.camera.fovDegrees),
                   0.01F));
    const float aspect = std::clamp(aspectRatio, 0.25F, 8.0F);
    if (!std::isfinite(projection.depth) ||
        projection.depth <= std::max(evaluation.camera.nearPlane, 1.0e-5F) ||
        projection.depth >= evaluation.camera.farPlane || tangent <= 1.0e-6F) {
        return projection;
    }
    const glm::vec2 ndc{
        local.x / (projection.depth * tangent * aspect),
        local.y / (projection.depth * tangent),
    };
    projection.uv = 0.5F * (ndc + glm::vec2{1.0F, 1.0F});
    projection.visible =
        std::isfinite(projection.uv.x) && std::isfinite(projection.uv.y) &&
        projection.uv.x >= 0.0F && projection.uv.x <= 1.0F &&
        projection.uv.y >= 0.0F && projection.uv.y <= 1.0F;
    return projection;
}

struct StrongAlignmentSample {
    glm::vec3 world{0.0F};
    glm::vec2 referenceUv{0.0F};
};

struct StrongAlignmentSampleSet {
    std::vector<StrongAlignmentSample> samples;
    std::size_t destinationOccupiedCellCount = 0U;
    std::size_t referenceOccupiedCellCount = 0U;
    float destinationCoverage = 0.0F;
    float referenceCoverage = 0.0F;
};

StrongAlignmentSampleSet BuildStrongAlignmentSamples(
    const AnimationPathEvaluation& destination,
    const AnimationPathEvaluation& reference,
    std::span<const invisible_places::io::Float3> residentPoints,
    const AnimationStrongAlignmentOptions& options) {
    StrongAlignmentSampleSet result;
    if (residentPoints.empty()) {
        return result;
    }
    const std::size_t maximumSamples = std::max<std::size_t>(
        1U,
        options.maximumPointSamples);
    const std::size_t stride = std::max<std::size_t>(
        1U,
        (residentPoints.size() + maximumSamples - 1U) / maximumSamples);
    std::vector<glm::vec3> points;
    for (std::size_t index = 0U;
         index < residentPoints.size();
         index += stride) {
        if (options.stopToken.stop_requested()) {
            return result;
        }
        const auto& point = residentPoints[index];
        points.push_back({point.x, point.y, point.z});
    }

    constexpr std::size_t kGridWidth = 64U;
    constexpr std::size_t kGridHeight = 36U;
    constexpr std::size_t kGridSize = kGridWidth * kGridHeight;
    std::array<std::vector<float>, 2> nearestDepth{
        std::vector<float>(kGridSize, std::numeric_limits<float>::infinity()),
        std::vector<float>(kGridSize, std::numeric_limits<float>::infinity()),
    };
    std::array<std::vector<StrongScreenProjection>, 2> projections{
        std::vector<StrongScreenProjection>(points.size()),
        std::vector<StrongScreenProjection>(points.size()),
    };
    const std::array<const AnimationPathEvaluation*, 2> evaluations{
        &destination,
        &reference,
    };
    for (std::size_t view = 0U; view < 2U; ++view) {
        for (std::size_t pointIndex = 0U;
             pointIndex < points.size();
             ++pointIndex) {
            if (options.stopToken.stop_requested()) {
                return {};
            }
            auto& projection = projections[view][pointIndex];
            projection = ProjectStrongAlignmentPoint(
                *evaluations[view],
                points[pointIndex],
                options.aspectRatio);
            if (!projection.visible) {
                continue;
            }
            const std::size_t x = std::min<std::size_t>(
                kGridWidth - 1U,
                static_cast<std::size_t>(
                    projection.uv.x * static_cast<float>(kGridWidth)));
            const std::size_t y = std::min<std::size_t>(
                kGridHeight - 1U,
                static_cast<std::size_t>(
                    projection.uv.y * static_cast<float>(kGridHeight)));
            nearestDepth[view][y * kGridWidth + x] = std::min(
                nearestDepth[view][y * kGridWidth + x],
                projection.depth);
        }
    }

    const float lowerFrameFraction = std::clamp(
        options.lowerFrameFraction,
        0.25F,
        0.75F);
    std::array<std::vector<std::uint8_t>, 2> occupiedCells{
        std::vector<std::uint8_t>(kGridSize, 0U),
        std::vector<std::uint8_t>(kGridSize, 0U),
    };
    std::vector<std::uint8_t> referenceCellSamples(kGridSize, 0U);
    result.samples.reserve(kGridSize * 2U);
    const auto gridIndex = [&](const StrongScreenProjection& projection) {
        const std::size_t x = std::min<std::size_t>(
            kGridWidth - 1U,
            static_cast<std::size_t>(
                projection.uv.x * static_cast<float>(kGridWidth)));
        const std::size_t y = std::min<std::size_t>(
            kGridHeight - 1U,
            static_cast<std::size_t>(
                projection.uv.y * static_cast<float>(kGridHeight)));
        return y * kGridWidth + x;
    };
    for (std::size_t pointIndex = 0U;
         pointIndex < points.size();
         ++pointIndex) {
        if (options.stopToken.stop_requested()) {
            return {};
        }
        const auto& destinationProjection = projections[0U][pointIndex];
        const auto& referenceProjection = projections[1U][pointIndex];
        if (!destinationProjection.visible || !referenceProjection.visible) {
            continue;
        }
        if (destinationProjection.uv.y > lowerFrameFraction ||
            referenceProjection.uv.y > lowerFrameFraction) {
            continue;
        }
        const auto isFrontmost = [&](std::size_t view) {
            const auto& projection = projections[view][pointIndex];
            const float nearest = nearestDepth[view][gridIndex(projection)];
            return projection.depth <= nearest * 1.02F + 1.0e-4F;
        };
        if (!isFrontmost(0U) || !isFrontmost(1U)) {
            continue;
        }
        const std::size_t referenceCell = gridIndex(referenceProjection);
        // Keep the objective spatially balanced. Dense nearby sand otherwise
        // overwhelms the rock transition and sparse far side of the image.
        if (referenceCellSamples[referenceCell] >= 2U) {
            continue;
        }
        ++referenceCellSamples[referenceCell];
        result.samples.push_back({
            .world = points[pointIndex],
            .referenceUv = referenceProjection.uv,
        });
        occupiedCells[0U][gridIndex(destinationProjection)] = 1U;
        occupiedCells[1U][referenceCell] = 1U;
    }
    result.destinationOccupiedCellCount = static_cast<std::size_t>(
        std::count(occupiedCells[0U].begin(), occupiedCells[0U].end(), 1U));
    result.referenceOccupiedCellCount = static_cast<std::size_t>(
        std::count(occupiedCells[1U].begin(), occupiedCells[1U].end(), 1U));
    const std::size_t availableLowerCells = std::max<std::size_t>(
        1U,
        kGridWidth * static_cast<std::size_t>(std::ceil(
                         lowerFrameFraction *
                         static_cast<float>(kGridHeight))));
    result.destinationCoverage = std::clamp(
        static_cast<float>(result.destinationOccupiedCellCount) /
            static_cast<float>(availableLowerCells),
        0.0F,
        1.0F);
    result.referenceCoverage = std::clamp(
        static_cast<float>(result.referenceOccupiedCellCount) /
            static_cast<float>(availableLowerCells),
        0.0F,
        1.0F);
    return result;
}

struct StrongObjective {
    bool valid = false;
    float reprojectionMeanSquared = 0.0F;
    float horizontalOffset = 0.0F;
    float verticalOffset = 0.0F;
    float scaleMismatchPercent = 0.0F;
    float rotationMismatchDegrees = 0.0F;
    float robustReprojection = 0.0F;
    float shapeMeanSquared = 0.0F;
    float combined = 0.0F;
};

struct StrongScreenDistribution {
    glm::vec2 centroid{0.0F};
    float varianceX = 0.0F;
    float varianceY = 0.0F;
    float covariance = 0.0F;
    float radius = 0.0F;
    float angle = 0.0F;
};

StrongScreenDistribution StrongDistribution(
    std::span<const glm::vec2> points) {
    StrongScreenDistribution result;
    if (points.empty()) {
        return result;
    }
    for (const auto& point : points) {
        result.centroid += point;
    }
    result.centroid /= static_cast<float>(points.size());
    for (const auto& point : points) {
        const glm::vec2 offset = point - result.centroid;
        result.varianceX += offset.x * offset.x;
        result.covariance += offset.x * offset.y;
        result.varianceY += offset.y * offset.y;
    }
    const float inverseCount = 1.0F / static_cast<float>(points.size());
    result.varianceX *= inverseCount;
    result.varianceY *= inverseCount;
    result.covariance *= inverseCount;
    result.radius = std::sqrt(std::max(
        result.varianceX + result.varianceY,
        0.0F));
    result.angle = 0.5F * std::atan2(
        2.0F * result.covariance,
        result.varianceX - result.varianceY);
    return result;
}

float StrongHuberLoss(float distance, float delta) {
    const float absolute = std::abs(distance);
    return absolute <= delta
               ? absolute * absolute
               : 2.0F * delta * absolute - delta * delta;
}

StrongObjective EvaluateStrongAlignmentObjective(
    const AnimationPathEvaluation& destination,
    const StrongAlignmentSampleSet& samples,
    float aspectRatio,
    float regularization,
    float regularizationScale) {
    StrongObjective result;
    if (samples.samples.size() < 24U) {
        return result;
    }
    std::vector<glm::vec2> destinationScreen;
    std::vector<glm::vec2> referenceScreen;
    destinationScreen.reserve(samples.samples.size());
    referenceScreen.reserve(samples.samples.size());
    std::size_t lostSamples = 0U;
    constexpr float kHuberDelta = 0.05F;
    for (const auto& sample : samples.samples) {
        const auto projection = ProjectStrongAlignmentPoint(
            destination,
            sample.world,
            aspectRatio);
        if (!projection.visible) {
            ++lostSamples;
            continue;
        }
        const glm::vec2 destinationPoint{
            projection.uv.x * aspectRatio,
            projection.uv.y,
        };
        const glm::vec2 referencePoint{
            sample.referenceUv.x * aspectRatio,
            sample.referenceUv.y,
        };
        const glm::vec2 difference = destinationPoint - referencePoint;
        const float squared = glm::dot(difference, difference);
        result.reprojectionMeanSquared += squared;
        result.robustReprojection += StrongHuberLoss(
            std::sqrt(std::max(squared, 0.0F)),
            kHuberDelta);
        destinationScreen.push_back(destinationPoint);
        referenceScreen.push_back(referencePoint);
    }
    const std::size_t minimumVisible = std::max<std::size_t>(
        24U,
        (4U * samples.samples.size() + 4U) / 5U);
    if (destinationScreen.size() < minimumVisible) {
        return result;
    }
    const float inverseVisible =
        1.0F / static_cast<float>(destinationScreen.size());
    result.reprojectionMeanSquared *= inverseVisible;
    result.robustReprojection =
        (result.robustReprojection +
         0.05F * static_cast<float>(lostSamples)) /
        static_cast<float>(samples.samples.size());
    const auto destinationDistribution =
        StrongDistribution(destinationScreen);
    const auto referenceDistribution =
        StrongDistribution(referenceScreen);
    const glm::vec2 centroidDifference =
        destinationDistribution.centroid -
        referenceDistribution.centroid;
    result.horizontalOffset = centroidDifference.x;
    result.verticalOffset = centroidDifference.y;
    const float referenceRadius = std::max(
        referenceDistribution.radius,
        1.0e-6F);
    result.scaleMismatchPercent = 100.0F * std::abs(
        destinationDistribution.radius / referenceRadius - 1.0F);
    float angleDifference =
        destinationDistribution.angle - referenceDistribution.angle;
    while (angleDifference > 0.5F * glm::pi<float>()) {
        angleDifference -= glm::pi<float>();
    }
    while (angleDifference < -0.5F * glm::pi<float>()) {
        angleDifference += glm::pi<float>();
    }
    result.rotationMismatchDegrees =
        std::abs(glm::degrees(angleDifference));
    const float varianceXDifference =
        destinationDistribution.varianceX -
        referenceDistribution.varianceX;
    const float varianceYDifference =
        destinationDistribution.varianceY -
        referenceDistribution.varianceY;
    const float covarianceDifference =
        destinationDistribution.covariance -
        referenceDistribution.covariance;
    result.shapeMeanSquared =
        varianceXDifference * varianceXDifference +
        varianceYDifference * varianceYDifference +
        2.0F * covarianceDifference * covarianceDifference;
    const float centroidMeanSquared =
        glm::dot(centroidDifference, centroidDifference);
    result.combined =
        0.65F * result.robustReprojection +
        0.20F * centroidMeanSquared +
        0.10F * result.shapeMeanSquared +
        0.05F * regularizationScale * std::max(regularization, 0.0F);
    result.valid = std::isfinite(result.combined);
    return result;
}

float StrongTrackMovementCap(
    const AnimationPath& path,
    std::size_t keyIndex,
    bool cameraTrack,
    float fraction) {
    if (path.keys.size() < 2U || keyIndex >= path.keys.size()) {
        return 0.0F;
    }
    const auto value = [&](std::size_t index) -> const std::array<float, 3>& {
        return cameraTrack ? path.keys[index].cameraPosition
                           : path.keys[index].focusPoint;
    };
    float shortest = std::numeric_limits<float>::infinity();
    if (keyIndex > 0U) {
        shortest = std::min(
            shortest,
            Distance(value(keyIndex), value(keyIndex - 1U)));
    }
    if (keyIndex + 1U < path.keys.size()) {
        shortest = std::min(
            shortest,
            Distance(value(keyIndex), value(keyIndex + 1U)));
    }
    return std::isfinite(shortest)
               ? std::max(0.0F, fraction) * shortest
               : 0.0F;
}

std::array<float, 3> AddStrongOffset(
    const std::array<float, 3>& value,
    const glm::vec3& offset) {
    return {
        value[0U] + offset.x,
        value[1U] + offset.y,
        value[2U] + offset.z,
    };
}

float StrongScreenMovementRms(
    const AnimationPathEvaluation& before,
    const AnimationPathEvaluation& after,
    const StrongAlignmentSampleSet& samples,
    float aspectRatio) {
    if (samples.samples.empty()) {
        return 0.0F;
    }
    constexpr std::size_t kMaximumProbeSamples = 512U;
    const std::size_t stride = std::max<std::size_t>(
        1U,
        (samples.samples.size() + kMaximumProbeSamples - 1U) /
            kMaximumProbeSamples);
    float squaredMovement = 0.0F;
    std::size_t visibleCount = 0U;
    for (std::size_t sampleIndex = 0U;
         sampleIndex < samples.samples.size();
         sampleIndex += stride) {
        const auto beforeProjection = ProjectStrongAlignmentPoint(
            before,
            samples.samples[sampleIndex].world,
            aspectRatio);
        const auto afterProjection = ProjectStrongAlignmentPoint(
            after,
            samples.samples[sampleIndex].world,
            aspectRatio);
        if (!beforeProjection.visible || !afterProjection.visible) {
            continue;
        }
        const glm::vec2 movement{
            (afterProjection.uv.x - beforeProjection.uv.x) * aspectRatio,
            afterProjection.uv.y - beforeProjection.uv.y,
        };
        squaredMovement += glm::dot(movement, movement);
        ++visibleCount;
    }
    return visibleCount > 0U
               ? std::sqrt(
                     squaredMovement /
                     static_cast<float>(visibleCount))
               : 0.0F;
}

struct HorizontalAnimationPathFrame {
    glm::vec3 forward{1.0F, 0.0F, 0.0F};
    glm::vec3 lateral{0.0F, -1.0F, 0.0F};
};

std::optional<HorizontalAnimationPathFrame>
ResolveHorizontalAnimationPathFrame(
    const PreparedAnimationPathEvaluationContext& context,
    float normalizedPosition) {
    if (!context.valid || context.singleKey ||
        context.durationSeconds <= 1.0e-6F) {
        return std::nullopt;
    }

    const float centerSeconds =
        std::clamp(normalizedPosition, 0.0F, 1.0F) *
        context.durationSeconds;
    float sampleHalfSpan = std::clamp(
        context.durationSeconds / 600.0F,
        1.0F / 240.0F,
        1.0F / 30.0F);
    sampleHalfSpan = std::min(
        sampleHalfSpan,
        0.25F * context.durationSeconds);
    const float leftSeconds = std::max(
        0.0F,
        centerSeconds - sampleHalfSpan);
    const float rightSeconds = std::min(
        context.durationSeconds,
        centerSeconds + sampleHalfSpan);
    if (rightSeconds - leftSeconds <= 1.0e-7F) {
        return std::nullopt;
    }

    const auto left = EvaluatePreparedAnimationPath(
        context,
        leftSeconds);
    const auto right = EvaluatePreparedAnimationPath(
        context,
        rightSeconds);
    auto horizontalDelta = [](const std::array<float, 3>& from,
                              const std::array<float, 3>& to) {
        return glm::vec3{
            to[0U] - from[0U],
            to[1U] - from[1U],
            0.0F,
        };
    };
    glm::vec3 travel = horizontalDelta(
        left.focusPoint,
        right.focusPoint);
    if (glm::dot(travel, travel) <= 1.0e-12F) {
        travel = horizontalDelta(
            left.camera.position,
            right.camera.position);
    }
    if (!std::isfinite(travel.x) || !std::isfinite(travel.y) ||
        glm::dot(travel, travel) <= 1.0e-12F) {
        return std::nullopt;
    }

    const glm::vec3 forward = glm::normalize(travel);
    const glm::vec3 lateral = glm::cross(
        forward,
        glm::vec3{0.0F, 0.0F, 1.0F});
    if (!std::isfinite(lateral.x) || !std::isfinite(lateral.y) ||
        glm::dot(lateral, lateral) <= 1.0e-12F) {
        return std::nullopt;
    }
    return HorizontalAnimationPathFrame{
        .forward = forward,
        .lateral = glm::normalize(lateral),
    };
}

}  // namespace

AnimationFocusRelativeCameraAlignmentResult
AlignAnimationKeyCameraToReferenceRig(
    AnimationPath* destination,
    const AnimationPath& reference,
    const AnimationFocusRelativeCameraAlignmentOptions& options) {
    AnimationFocusRelativeCameraAlignmentResult result;
    if (destination == nullptr || destination == &reference) {
        result.errorMessage =
            "Choose different destination and reference animations.";
        return result;
    }
    const auto keyIt = std::find_if(
        destination->keys.begin(),
        destination->keys.end(),
        [&](const AnimationPathKey& key) {
            return key.id == options.destinationKeyId;
        });
    if (keyIt == destination->keys.end()) {
        result.errorMessage =
            "The selected camera-rig alignment key no longer exists.";
        return result;
    }

    const std::size_t keyIndex = static_cast<std::size_t>(
        std::distance(destination->keys.begin(), keyIt));
    const float destinationPosition = AnimationPathKeyNormalizedPosition(
        *destination,
        keyIndex);
    const float referencePosition = std::clamp(
        options.referenceNormalizedPosition,
        0.0F,
        1.0F);
    const auto destinationContext = PrepareAnimationPathEvaluation(
        *destination);
    const auto referenceContext = PrepareAnimationPathEvaluation(reference);
    if (!destinationContext.valid || !referenceContext.valid) {
        result.errorMessage = "One selected animation cannot be evaluated.";
        return result;
    }
    const auto destinationFrame = ResolveHorizontalAnimationPathFrame(
        destinationContext,
        destinationPosition);
    const auto referenceFrame = ResolveHorizontalAnimationPathFrame(
        referenceContext,
        referencePosition);
    if (!referenceFrame.has_value()) {
        result.errorMessage =
            "The matching animation has no horizontal camera or focus travel at that frame to define forward along its path.";
        return result;
    }
    if (!destinationFrame.has_value()) {
        result.errorMessage =
            "The selected animation has no horizontal camera or focus travel at that key to define its local path direction.";
        return result;
    }

    const auto referenceEvaluation = EvaluatePreparedAnimationPath(
        referenceContext,
        referenceContext.durationSeconds * referencePosition);
    const glm::vec3 referenceCamera = ToGlm(
        referenceEvaluation.camera.position);
    const glm::vec3 referenceFocus = ToGlm(
        referenceEvaluation.focusPoint);
    const glm::vec3 referenceOffset =
        referenceCamera - referenceFocus;
    result.referenceFocusDistance = glm::length(referenceOffset);
    if (!std::isfinite(result.referenceFocusDistance) ||
        result.referenceFocusDistance <= 1.0e-5F) {
        result.errorMessage =
            "The matching frame's camera is too close to its focus point to define a reusable camera rig.";
        return result;
    }

    result.referenceAlongPathOffset = glm::dot(
        referenceOffset,
        referenceFrame->forward);
    result.referenceLateralOffset = glm::dot(
        referenceOffset,
        referenceFrame->lateral);
    result.referenceHeightOffset = referenceOffset.z;
    const glm::vec3 destinationFocus = ToGlm(keyIt->focusPoint);
    const glm::vec3 targetCamera =
        destinationFocus +
        result.referenceAlongPathOffset * destinationFrame->forward +
        result.referenceLateralOffset * destinationFrame->lateral +
        result.referenceHeightOffset * glm::vec3{0.0F, 0.0F, 1.0F};
    if (!std::isfinite(targetCamera.x) || !std::isfinite(targetCamera.y) ||
        !std::isfinite(targetCamera.z)) {
        result.errorMessage =
            "The matching frame produced an invalid focus-relative camera position.";
        return result;
    }

    const std::array<float, 3> target{
        targetCamera.x,
        targetCamera.y,
        targetCamera.z,
    };
    result.cameraMove = Distance(keyIt->cameraPosition, target);
    result.succeeded = true;
    if (result.cameraMove <= 1.0e-6F) {
        return result;
    }

    EnsureLocalizedKeyCorrections(destination, {keyIndex});
    destination->keys[keyIndex].cameraPosition = target;
    result.changed = true;
    return result;
}

AnimationStrongAlignmentResult StrongAlignAnimationKeyToReference(
    AnimationPath* destination,
    const AnimationPath& reference,
    std::span<const invisible_places::io::Float3> residentPoints,
    const AnimationStrongAlignmentOptions& options) {
    AnimationStrongAlignmentResult result;
    if (options.stopToken.stop_requested()) {
        result.errorMessage = "Lower-frame alignment job cancelled.";
        return result;
    }
    if (destination == nullptr || destination == &reference) {
        result.errorMessage = "Choose different destination and reference animations.";
        return result;
    }
    const auto keyIt = std::find_if(
        destination->keys.begin(),
        destination->keys.end(),
        [&](const AnimationPathKey& key) {
            return key.id == options.destinationKeyId;
        });
    if (keyIt == destination->keys.end()) {
        result.errorMessage = "The selected lower-frame alignment key no longer exists.";
        return result;
    }
    if (residentPoints.size() < 32U) {
        result.errorMessage =
            "The common scene has too few resident position samples for geometry alignment.";
        return result;
    }
    const std::size_t keyIndex = static_cast<std::size_t>(
        std::distance(destination->keys.begin(), keyIt));
    const float destinationPosition = AnimationPathKeyNormalizedPosition(
        *destination,
        keyIndex);
    const auto destinationContext = PrepareAnimationPathEvaluation(*destination);
    const auto referenceContext = PrepareAnimationPathEvaluation(reference);
    if (!destinationContext.valid || !referenceContext.valid) {
        result.errorMessage = "One selected animation cannot be evaluated.";
        return result;
    }
    const auto destinationEvaluation = EvaluatePreparedAnimationPath(
        destinationContext,
        destinationContext.durationSeconds * destinationPosition);
    const auto referenceEvaluation = EvaluatePreparedAnimationPath(
        referenceContext,
        referenceContext.durationSeconds *
            std::clamp(options.referenceNormalizedPosition, 0.0F, 1.0F));
    const auto samples = BuildStrongAlignmentSamples(
        destinationEvaluation,
        referenceEvaluation,
        residentPoints,
        options);
    if (options.stopToken.stop_requested()) {
        result.errorMessage = "Lower-frame alignment job cancelled.";
        return result;
    }
    result.metrics.foregroundSampleCount = samples.samples.size();
    result.metrics.destinationOccupiedCellCount =
        samples.destinationOccupiedCellCount;
    result.metrics.referenceOccupiedCellCount =
        samples.referenceOccupiedCellCount;
    result.metrics.destinationCoverage = samples.destinationCoverage;
    result.metrics.referenceCoverage = samples.referenceCoverage;
    if (samples.samples.size() < 24U ||
        samples.destinationOccupiedCellCount < 12U ||
        samples.referenceOccupiedCellCount < 12U) {
        result.errorMessage =
            "Too few common frontmost points occupy the lower part of both frames. Try a closer matching frame or wait for the common scene's preview positions to become resident.";
        return result;
    }

    const auto original = *destination;
    auto candidate = original;
    EnsureLocalizedKeyCorrections(&candidate, {keyIndex});
    const auto originalCamera = original.keys[keyIndex].cameraPosition;
    const auto originalFocus = original.keys[keyIndex].focusPoint;
    const float moveFraction = std::clamp(
        options.maxMoveFraction,
        0.01F,
        1.0F);
    const float cameraCap = StrongTrackMovementCap(
        original,
        keyIndex,
        true,
        moveFraction);
    const float focusCap = StrongTrackMovementCap(
        original,
        keyIndex,
        false,
        moveFraction);
    if (cameraCap <= 1.0e-8F && focusCap <= 1.0e-8F) {
        result.errorMessage =
            "The selected key has no non-zero neighboring camera or focus segment to define a movement cap.";
        return result;
    }

    const glm::quat destinationOrientation =
        QuaternionFromCameraState(destinationEvaluation.camera);
    const std::array<glm::vec3, 3> axes{
        glm::normalize(
            destinationOrientation * glm::vec3{1.0F, 0.0F, 0.0F}),
        glm::normalize(
            destinationOrientation * glm::vec3{0.0F, 1.0F, 0.0F}),
        glm::normalize(
            destinationOrientation * glm::vec3{0.0F, 0.0F, -1.0F}),
    };

    const auto evaluateCandidate = [&](const AnimationPath& path) {
        const auto context = PrepareAnimationPathEvaluation(path);
        return EvaluatePreparedAnimationPath(
            context,
            context.durationSeconds * destinationPosition);
    };
    const float destinationTime =
        destinationContext.durationSeconds * destinationPosition;
    const float flowDelta = std::min(
        std::max(
            destinationContext.durationSeconds / 240.0F,
            1.0F / 240.0F),
        std::max(
            destinationContext.durationSeconds,
            1.0F / 240.0F));
    const float originalScreenSpeed = ProbePerceivedFlow(
        destinationContext,
        destinationTime,
        flowDelta)
                                          .fullScreenSpeed;
    const auto regularization = [&](const AnimationPath& path) {
        const float cameraUsage = cameraCap > 1.0e-8F
                                      ? Distance(
                                            originalCamera,
                                            path.keys[keyIndex].cameraPosition) /
                                            cameraCap
                                      : 0.0F;
        const float focusUsage = focusCap > 1.0e-8F
                                     ? Distance(
                                           originalFocus,
                                           path.keys[keyIndex].focusPoint) /
                                           focusCap
                                     : 0.0F;
        const float poseRegularization =
            0.5F * (cameraUsage * cameraUsage +
                    focusUsage * focusUsage);
        const auto pathContext = PrepareAnimationPathEvaluation(path);
        const float candidateSpeed = ProbePerceivedFlow(
            pathContext,
            pathContext.durationSeconds * destinationPosition,
            flowDelta)
                                         .fullScreenSpeed;
        const float speedScale = std::max(originalScreenSpeed, 1.0e-3F);
        const float speedDrift =
            (candidateSpeed - originalScreenSpeed) / speedScale;
        return 0.75F * poseRegularization +
               0.25F * speedDrift * speedDrift;
    };
    const auto baseline = EvaluateStrongAlignmentObjective(
        destinationEvaluation,
        samples,
        options.aspectRatio,
        0.0F,
        1.0F);
    if (!baseline.valid) {
        result.errorMessage =
            "The resident common-scene samples could not be compared in both views.";
        return result;
    }
    const float regularizationScale = std::max(
        1.0e-7F,
        0.65F * baseline.robustReprojection +
            0.20F * (baseline.horizontalOffset * baseline.horizontalOffset +
                     baseline.verticalOffset * baseline.verticalOffset) +
            0.10F * baseline.shapeMeanSquared);
    StrongObjective best = baseline;
    const float jointCap = std::min(cameraCap, focusCap);
    // The first triplet translates camera and focus together. The other two
    // move only camera or focus. Joint translation is important: it can fix
    // centring/parallax without forcing the coordinate descent through a
    // temporarily worse camera-only or focus-only pose.
    const std::array<float, 9> variableCaps{
        jointCap,
        jointCap,
        jointCap,
        cameraCap,
        cameraCap,
        cameraCap,
        focusCap,
        focusCap,
        focusCap,
    };
    const auto applyVariable = [&](AnimationPath* path,
                                   std::size_t variable,
                                   float amount) {
        auto& key = path->keys[keyIndex];
        const glm::vec3 offset = amount * axes[variable % 3U];
        const std::size_t trackGroup = variable / 3U;
        if (trackGroup == 0U || trackGroup == 1U) {
            key.cameraPosition = AddStrongOffset(key.cameraPosition, offset);
        }
        if (trackGroup == 0U || trackGroup == 2U) {
            key.focusPoint = AddStrongOffset(key.focusPoint, offset);
        }
    };
    const auto clampMovement = [&](AnimationPath* path) {
        auto clampTrack = [](std::array<float, 3>* value,
                             const std::array<float, 3>& origin,
                             float cap) {
            glm::vec3 offset = ToGlm(*value) - ToGlm(origin);
            const float length = glm::length(offset);
            if (cap > 0.0F && length > cap) {
                offset *= cap / length;
                *value = AddStrongOffset(origin, offset);
            }
        };
        auto& key = path->keys[keyIndex];
        clampTrack(&key.cameraPosition, originalCamera, cameraCap);
        clampTrack(&key.focusPoint, originalFocus, focusCap);
    };
    std::array<float, 9> fullCapScreenEffects{};
    float screenEffectSum = 0.0F;
    std::size_t screenEffectCount = 0U;
    constexpr float kSensitivityProbeFraction = 0.05F;
    for (std::size_t variable = 0U;
         variable < variableCaps.size();
         ++variable) {
        if (options.stopToken.stop_requested()) {
            result.errorMessage = "Lower-frame alignment job cancelled.";
            return result;
        }
        const float cap = variableCaps[variable];
        if (cap <= 1.0e-8F) {
            continue;
        }
        auto probe = candidate;
        applyVariable(
            &probe,
            variable,
            kSensitivityProbeFraction * cap);
        clampMovement(&probe);
        const float effect = StrongScreenMovementRms(
            destinationEvaluation,
            evaluateCandidate(probe),
            samples,
            options.aspectRatio) /
            kSensitivityProbeFraction;
        if (std::isfinite(effect) && effect > 1.0e-8F) {
            fullCapScreenEffects[variable] = effect;
            screenEffectSum += effect;
            ++screenEffectCount;
        }
    }
    const float targetScreenEffect = screenEffectCount > 0U
                                         ? screenEffectSum /
                                               static_cast<float>(
                                                   screenEffectCount)
                                         : 0.0F;
    std::array<float, 9> step{};
    for (std::size_t variable = 0U;
         variable < step.size();
         ++variable) {
        const float cap = variableCaps[variable];
        float sensitivityScale = 1.0F;
        if (targetScreenEffect > 1.0e-8F &&
            fullCapScreenEffects[variable] > 1.0e-8F) {
            sensitivityScale = std::clamp(
                targetScreenEffect /
                    fullCapScreenEffects[variable],
                0.25F,
                2.0F);
        }
        step[variable] = std::min(
            0.50F * cap,
            0.30F * cap * sensitivityScale);
    }
    for (std::size_t sweep = 0U; sweep < 12U; ++sweep) {
        if (options.stopToken.stop_requested()) {
            result.errorMessage = "Lower-frame alignment job cancelled.";
            return result;
        }
        bool improved = false;
        for (std::size_t variable = 0U; variable < step.size(); ++variable) {
            if (step[variable] <= 1.0e-8F) {
                continue;
            }
            for (const float sign : {-1.0F, 1.0F}) {
                auto trial = candidate;
                applyVariable(&trial, variable, sign * step[variable]);
                clampMovement(&trial);
                const auto objective = EvaluateStrongAlignmentObjective(
                    evaluateCandidate(trial),
                    samples,
                    options.aspectRatio,
                    regularization(trial),
                    regularizationScale);
                if (objective.valid &&
                    objective.combined + 1.0e-10F < best.combined) {
                    candidate = std::move(trial);
                    best = objective;
                    improved = true;
                }
            }
        }
        if (!improved || sweep % 2U == 1U) {
            for (auto& value : step) {
                value *= 0.5F;
            }
        }
    }

    const float foregroundTolerance = std::max(
        baseline.reprojectionMeanSquared * 0.01F,
        std::pow(0.25F / 1080.0F, 2.0F));
    const float requiredImprovement = std::max(
        1.0e-10F,
        baseline.combined * 1.0e-4F);
    const bool accepted =
        best.valid &&
        best.combined + requiredImprovement < baseline.combined &&
        best.reprojectionMeanSquared <=
            baseline.reprojectionMeanSquared + foregroundTolerance;
    result.succeeded = true;
    result.changed = accepted;
    const auto rms1080 = [](float meanSquared) {
        return 1080.0F * std::sqrt(std::max(meanSquared, 0.0F));
    };
    const StrongObjective& finalObjective = accepted ? best : baseline;
    result.metrics.beforeForegroundReprojectionRms1080 =
        rms1080(baseline.reprojectionMeanSquared);
    result.metrics.afterForegroundReprojectionRms1080 =
        rms1080(finalObjective.reprojectionMeanSquared);
    result.metrics.beforeHorizontalOffset1080 =
        1080.0F * std::abs(baseline.horizontalOffset);
    result.metrics.afterHorizontalOffset1080 =
        1080.0F * std::abs(finalObjective.horizontalOffset);
    result.metrics.beforeVerticalOffset1080 =
        1080.0F * std::abs(baseline.verticalOffset);
    result.metrics.afterVerticalOffset1080 =
        1080.0F * std::abs(finalObjective.verticalOffset);
    result.metrics.beforeScaleMismatchPercent =
        baseline.scaleMismatchPercent;
    result.metrics.afterScaleMismatchPercent =
        finalObjective.scaleMismatchPercent;
    result.metrics.beforeRotationMismatchDegrees =
        baseline.rotationMismatchDegrees;
    result.metrics.afterRotationMismatchDegrees =
        finalObjective.rotationMismatchDegrees;
    if (!accepted) {
        result.errorMessage =
            "No pose inside the selected key's movement cap improved the common lower-frame reprojection.";
        return result;
    }
    result.metrics.cameraMove = Distance(
        originalCamera,
        candidate.keys[keyIndex].cameraPosition);
    result.metrics.focusMove = Distance(
        originalFocus,
        candidate.keys[keyIndex].focusPoint);
    result.metrics.cameraCapUsage = cameraCap > 1.0e-8F
                                        ? result.metrics.cameraMove / cameraCap
                                        : 0.0F;
    result.metrics.focusCapUsage = focusCap > 1.0e-8F
                                       ? result.metrics.focusMove / focusCap
                                       : 0.0F;
    *destination = std::move(candidate);
    return result;
}

AnimationMatchingFrameGhostResult BuildAnimationMatchingFrameGhost(
    const AnimationPathEvaluation& destination,
    const AnimationPathEvaluation& reference,
    std::span<const invisible_places::io::Float3> residentPoints,
    const AnimationMatchingFrameGhostOptions& options) {
    AnimationMatchingFrameGhostResult result;
    result.inputPointCount = residentPoints.size();
    if (options.stopToken.stop_requested()) {
        result.errorMessage = "Matching-frame point capture cancelled.";
        return result;
    }
    if (residentPoints.empty()) {
        result.errorMessage =
            "The linked animations have no resident point positions to capture.";
        return result;
    }

    const std::size_t maximumSamples = std::max<std::size_t>(
        1U,
        options.maximumPointSamples);
    const std::size_t stride = std::max<std::size_t>(
        1U,
        (residentPoints.size() + maximumSamples - 1U) / maximumSamples);
    const std::size_t gridWidth = std::clamp<std::size_t>(
        options.screenGridWidth,
        8U,
        1024U);
    const std::size_t gridHeight = std::clamp<std::size_t>(
        options.screenGridHeight,
        8U,
        1024U);
    const std::size_t gridSize = gridWidth * gridHeight;
    std::vector<float> nearestDepth(
        gridSize,
        std::numeric_limits<float>::infinity());

    struct Sample {
        glm::vec3 world{0.0F};
        StrongScreenProjection projection{};
        std::size_t cellIndex = 0U;
    };
    std::vector<Sample> visibleSamples;
    visibleSamples.reserve(std::min(residentPoints.size(), maximumSamples));
    const auto cellForProjection = [&](const StrongScreenProjection& projection) {
        const std::size_t x = std::min<std::size_t>(
            gridWidth - 1U,
            static_cast<std::size_t>(
                projection.uv.x * static_cast<float>(gridWidth)));
        const std::size_t y = std::min<std::size_t>(
            gridHeight - 1U,
            static_cast<std::size_t>(
                projection.uv.y * static_cast<float>(gridHeight)));
        return y * gridWidth + x;
    };
    for (std::size_t pointIndex = 0U;
         pointIndex < residentPoints.size();
         pointIndex += stride) {
        if ((result.sampledPointCount & 1023U) == 0U &&
            options.stopToken.stop_requested()) {
            result.errorMessage = "Matching-frame point capture cancelled.";
            return result;
        }
        ++result.sampledPointCount;
        const auto& source = residentPoints[pointIndex];
        const glm::vec3 world{source.x, source.y, source.z};
        if (!std::isfinite(world.x) || !std::isfinite(world.y) ||
            !std::isfinite(world.z)) {
            continue;
        }
        const auto projection = ProjectStrongAlignmentPoint(
            reference,
            world,
            options.aspectRatio);
        if (!projection.visible) {
            continue;
        }
        ++result.frustumVisiblePointCount;
        const std::size_t cellIndex = cellForProjection(projection);
        nearestDepth[cellIndex] = std::min(
            nearestDepth[cellIndex],
            projection.depth);
        visibleSamples.push_back({
            .world = world,
            .projection = projection,
            .cellIndex = cellIndex,
        });
    }
    if (visibleSamples.empty()) {
        result.errorMessage =
            "No resident points are visible from the linked animation's matching frame.";
        return result;
    }

    const float absoluteTolerance = std::max(
        options.frontDepthToleranceMeters,
        0.0F);
    const float relativeTolerance = std::max(
        options.frontDepthToleranceFraction,
        0.0F);
    const auto frameTransform = BuildAnimationCameraFrameTransform(
        destination,
        reference);
    const glm::vec3 referencePosition =
        ToGlm(frameTransform.sourceCameraPosition);
    const glm::vec3 destinationPosition =
        ToGlm(frameTransform.destinationCameraPosition);
    const glm::quat referenceToDestination = QuaternionFromArray(
        frameTransform.sourceToDestinationRotation);
    result.positions.reserve(visibleSamples.size());
    std::size_t visibleIndex = 0U;
    for (const auto& sample : visibleSamples) {
        if ((visibleIndex & 1023U) == 0U &&
            options.stopToken.stop_requested()) {
            result.positions.clear();
            result.errorMessage = "Matching-frame point capture cancelled.";
            return result;
        }
        ++visibleIndex;
        const float frontDepth = nearestDepth[sample.cellIndex];
        const float tolerance = std::max(
            absoluteTolerance,
            relativeTolerance * frontDepth);
        if (sample.projection.depth > frontDepth + tolerance) {
            continue;
        }
        const glm::vec3 transformed =
            destinationPosition +
            referenceToDestination *
                (sample.world - referencePosition);
        result.positions.push_back({
            transformed.x,
            transformed.y,
            transformed.z,
        });
    }
    if (result.positions.empty()) {
        result.errorMessage =
            "The linked frame's visibility filter did not retain a front surface.";
        return result;
    }
    result.succeeded = true;
    return result;
}

AnimationPathEvaluation EvaluateAnimationPath(
    const AnimationPath& path,
    float timeSeconds) {
    return EvaluatePreparedAnimationPath(
        PrepareAnimationPathEvaluation(path),
        timeSeconds);
}

AnimationCameraFrameTransform BuildAnimationCameraFrameTransform(
    const AnimationPathEvaluation& destination,
    const AnimationPathEvaluation& source) {
    const glm::quat rotation = glm::normalize(
        QuaternionFromCameraState(destination.camera) *
        glm::inverse(QuaternionFromCameraState(source.camera)));
    return {
        .sourceCameraPosition = source.camera.position,
        .destinationCameraPosition = destination.camera.position,
        .sourceToDestinationRotation = {
            rotation.x,
            rotation.y,
            rotation.z,
            rotation.w,
        },
    };
}

std::array<float, 3> ApplyAnimationCameraFrameTransform(
    const AnimationCameraFrameTransform& transform,
    const std::array<float, 3>& sourcePoint) {
    const glm::vec3 sourceOrigin =
        ToGlm(transform.sourceCameraPosition);
    const glm::vec3 destinationOrigin =
        ToGlm(transform.destinationCameraPosition);
    const glm::quat rotation =
        QuaternionFromArray(transform.sourceToDestinationRotation);
    const glm::vec3 result = destinationOrigin +
        rotation * (ToGlm(sourcePoint) - sourceOrigin);
    return {result.x, result.y, result.z};
}

std::array<float, 3> InvertAnimationCameraFrameTransform(
    const AnimationCameraFrameTransform& transform,
    const std::array<float, 3>& destinationPoint) {
    const glm::vec3 sourceOrigin =
        ToGlm(transform.sourceCameraPosition);
    const glm::vec3 destinationOrigin =
        ToGlm(transform.destinationCameraPosition);
    const glm::quat rotation =
        QuaternionFromArray(transform.sourceToDestinationRotation);
    const glm::vec3 result = sourceOrigin +
        glm::inverse(rotation) *
            (ToGlm(destinationPoint) - destinationOrigin);
    return {result.x, result.y, result.z};
}

std::optional<std::size_t> InsertAnimationPathKeyAtFrame(
    AnimationPath* path,
    AnimationPathKey key,
    std::uint32_t frame,
    std::string* errorMessage) {
    const auto fail = [&](std::string message) {
        if (errorMessage != nullptr) {
            *errorMessage = std::move(message);
        }
        return std::optional<std::size_t>{};
    };
    if (path == nullptr || path->keys.size() < 2U) {
        return fail("At least two existing keys are required to insert at a playhead frame.");
    }
    const auto durations = BuildSegmentDurations(*path);
    const std::uint32_t totalFrames = std::accumulate(
        durations.begin(),
        durations.end(),
        0U);
    if (frame == 0U || frame >= totalFrames) {
        return fail("Move the playhead between the first and last animation frames before inserting a key.");
    }

    std::uint32_t segmentStart = 0U;
    std::size_t segmentIndex = 0U;
    bool foundSegment = false;
    for (; segmentIndex < durations.size(); ++segmentIndex) {
        const std::uint32_t segmentEnd =
            segmentStart + durations[segmentIndex];
        if (frame == segmentStart || frame == segmentEnd) {
            return fail(
                "A key already occupies animation frame " +
                std::to_string(frame) + ".");
        }
        if (frame > segmentStart && frame < segmentEnd) {
            foundSegment = true;
            break;
        }
        segmentStart = segmentEnd;
    }
    if (!foundSegment) {
        return fail("The requested animation frame is not inside a key segment.");
    }
    BakeAnimationPathLocalizedCorrections(path);

    // Capture the clean curve before changing its topology. An evaluated
    // playhead key can then split both the existing spatial spline and its
    // geometry coordinate without re-solving a different curve merely
    // because another control was inserted on it.
    const auto previousContext = PrepareAnimationPathEvaluation(*path);
    const float insertionTime =
        static_cast<float>(frame) / kAnimationFramesPerSecond;
    const auto previousEvaluation = EvaluatePreparedAnimationPath(
        previousContext,
        insertionTime);
    const float insertionGeometry = EvaluateGeometryTimeMap(
        previousContext,
        insertionTime);
    const auto pointDistance = [](const std::array<float, 3>& left,
                                  const std::array<float, 3>& right) {
        return glm::length(ToGlm(left) - ToGlm(right));
    };
    const float poseScale = std::max(
        1.0F,
        std::max(
            pointDistance(
                previousEvaluation.camera.position,
                previousEvaluation.focusPoint),
            pointDistance(key.cameraPosition, key.focusPoint)));
    bool preservesEvaluatedCurve =
        previousContext.valid &&
        pointDistance(
            key.cameraPosition,
            previousEvaluation.camera.position) <= 2.0e-4F * poseScale &&
        pointDistance(
            key.focusPoint,
            previousEvaluation.focusPoint) <= 2.0e-4F * poseScale &&
        std::abs(key.fovDegrees -
                 previousEvaluation.camera.fovDegrees) <= 1.0e-3F &&
        std::abs(key.nearPlane -
                 previousEvaluation.camera.nearPlane) <= 1.0e-4F &&
        std::abs(key.farPlane -
                 previousEvaluation.camera.farPlane) <= 1.0e-2F;
    if (preservesEvaluatedCurve && previousContext.hasOrientation) {
        const auto keyOrientation = OrientationFromKey(key);
        const auto evaluatedOrientation = QuaternionFromArray(
            previousEvaluation.camera.orientation);
        preservesEvaluatedCurve =
            std::abs(glm::dot(keyOrientation, evaluatedOrientation)) >=
            1.0F - 1.0e-5F;
    }
    if (preservesEvaluatedCurve && previousContext.hasFocusDistance) {
        preservesEvaluatedCurve =
            std::abs(ReadFocusDistance(key) -
                     previousEvaluation.focusDistance) <=
            2.0e-4F * poseScale;
    }
    if (preservesEvaluatedCurve && previousContext.hasApertureFStops) {
        preservesEvaluatedCurve =
            std::abs(ReadApertureFStops(key) -
                     previousEvaluation.camera.apertureFStops) <= 1.0e-3F;
    }
    preservesEvaluatedCurve =
        preservesEvaluatedCurve &&
        previousContext.geometryKnots.size() == path->keys.size();

    if (preservesEvaluatedCurve) {
        const auto materializeEndpointTangents = [&](
            AnimationPathKey* endpoint,
            bool firstEndpoint) {
            endpoint->hasSplineEndpointTangent = true;
            const AnimationPreparedScalarSpline* cameraSplines[] = {
                &previousContext.cameraX,
                &previousContext.cameraY,
                &previousContext.cameraZ,
            };
            const AnimationPreparedScalarSpline* focusSplines[] = {
                &previousContext.focusX,
                &previousContext.focusY,
                &previousContext.focusZ,
            };
            for (std::size_t component = 0U; component < 3U; ++component) {
                endpoint->splineCameraEndpointTangent[component] =
                    EvaluatePreparedScalarSplineEndpointDerivative(
                        previousContext.geometryKnots,
                        *cameraSplines[component],
                        firstEndpoint);
                endpoint->splineFocusEndpointTangent[component] =
                    EvaluatePreparedScalarSplineEndpointDerivative(
                        previousContext.geometryKnots,
                        *focusSplines[component],
                        firstEndpoint);
            }
            const AnimationPreparedScalarSpline* orientationSplines[] = {
                &previousContext.orientationX,
                &previousContext.orientationY,
                &previousContext.orientationZ,
                &previousContext.orientationW,
            };
            for (std::size_t component = 0U; component < 4U; ++component) {
                endpoint->splineOrientationEndpointTangent[component] =
                    EvaluatePreparedScalarSplineEndpointDerivative(
                        previousContext.geometryKnots,
                        *orientationSplines[component],
                        firstEndpoint);
            }
            const AnimationPreparedScalarSpline* lensSplines[] = {
                &previousContext.fovDegrees,
                &previousContext.nearPlane,
                &previousContext.farPlane,
                &previousContext.focusDistance,
                &previousContext.apertureFStopsSpline,
            };
            for (std::size_t component = 0U; component < 5U; ++component) {
                endpoint->splineLensEndpointTangent[component] =
                    EvaluatePreparedScalarSplineEndpointDerivative(
                        previousContext.geometryKnots,
                        *lensSplines[component],
                        firstEndpoint);
            }
        };
        materializeEndpointTangents(&path->keys.front(), true);
        materializeEndpointTangents(&path->keys.back(), false);
        path->keys.front().splineParameterWeight = 0.0F;
        for (std::size_t index = 1U; index < path->keys.size(); ++index) {
            path->keys[index].splineParameterWeight =
                previousContext.geometryKnots[index] -
                previousContext.geometryKnots[index - 1U];
        }
    } else {
        RebuildAnimationPathGeometryFromKeys(path);
    }

    const auto idExists = [&](std::string_view candidate) {
        return std::any_of(
            path->keys.begin(),
            path->keys.end(),
            [&](const auto& existing) { return existing.id == candidate; });
    };
    std::string requestedId = key.id;
    if (requestedId.empty()) {
        std::size_t suffix = 1U;
        do {
            requestedId = "key_" + std::to_string(suffix++);
        } while (idExists(requestedId));
    } else if (idExists(requestedId)) {
        std::size_t suffix = 1U;
        const std::string base = requestedId;
        do {
            requestedId = base + "_" + std::to_string(suffix++);
        } while (idExists(requestedId));
    }
    key.id = std::move(requestedId);

    // Materialize the effective segment frames before editing. This makes
    // the inserted key land on the requested integer frame even when the
    // authored weights previously required duration rescaling.
    for (std::size_t index = 1U; index < path->keys.size(); ++index) {
        path->keys[index].durationFrames = durations[index - 1U];
    }
    const std::uint32_t segmentEnd =
        segmentStart + durations[segmentIndex];
    key.durationFrames = frame - segmentStart;
    const std::size_t insertionIndex = segmentIndex + 1U;
    if (preservesEvaluatedCurve) {
        const float leftGeometry =
            previousContext.geometryKnots[segmentIndex];
        const float rightGeometry =
            previousContext.geometryKnots[segmentIndex + 1U];
        const float geometrySpan = rightGeometry - leftGeometry;
        const float edgeInset = std::min(
            1.0e-5F,
            0.25F * geometrySpan);
        const float splitGeometry = std::clamp(
            insertionGeometry,
            leftGeometry + edgeInset,
            rightGeometry - edgeInset);
        key.splineParameterWeight = splitGeometry - leftGeometry;
        path->keys[insertionIndex].splineParameterWeight =
            rightGeometry - splitGeometry;
    }
    path->keys.insert(
        path->keys.begin() + static_cast<std::ptrdiff_t>(insertionIndex),
        std::move(key));
    path->keys[insertionIndex + 1U].durationFrames =
        segmentEnd - frame;
    if (errorMessage != nullptr) {
        errorMessage->clear();
    }
    return insertionIndex;
}

bool RemoveAnimationPathKey(
    AnimationPath* path,
    std::size_t keyIndex,
    std::string* errorMessage) {
    if (path == nullptr || keyIndex >= path->keys.size()) {
        if (errorMessage != nullptr) {
            *errorMessage = "The selected animation key no longer exists.";
        }
        return false;
    }
    if (path->keys.size() <= 2U) {
        if (errorMessage != nullptr) {
            *errorMessage = "An animation must retain at least two keys.";
        }
        return false;
    }
    BakeAnimationPathLocalizedCorrections(path);

    const bool hasMaterializedGeometry = std::all_of(
        path->keys.begin() + 1,
        path->keys.end(),
        [](const AnimationPathKey& key) {
            return std::isfinite(key.splineParameterWeight) &&
                   key.splineParameterWeight > 1.0e-6F;
        });
    auto durations = BuildSegmentDurations(*path);
    if (keyIndex == 0U) {
        durations.erase(durations.begin());
    } else if (keyIndex + 1U == path->keys.size()) {
        durations.pop_back();
    } else {
        durations[keyIndex - 1U] += durations[keyIndex];
        durations.erase(
            durations.begin() + static_cast<std::ptrdiff_t>(keyIndex));
    }

    if (hasMaterializedGeometry && keyIndex > 0U &&
        keyIndex + 1U < path->keys.size()) {
        path->keys[keyIndex + 1U].splineParameterWeight +=
            path->keys[keyIndex].splineParameterWeight;
    }
    const std::string removedId = path->keys[keyIndex].id;
    path->keys.erase(
        path->keys.begin() + static_cast<std::ptrdiff_t>(keyIndex));
    if (hasMaterializedGeometry) {
        path->keys.front().splineParameterWeight = 0.0F;
    } else {
        RebuildAnimationPathGeometryFromKeys(path);
    }
    for (std::size_t index = 1U; index < path->keys.size(); ++index) {
        path->keys[index].durationFrames = durations[index - 1U];
    }
    std::erase_if(
        path->localizedKeyCorrections,
        [&](const auto& correction) {
            return correction.keyId == removedId;
        });
    if (path->velocityBlendLink.has_value()) {
        std::erase(
            path->velocityBlendLink->movableKeyIds,
            removedId);
    }
    if (errorMessage != nullptr) {
        errorMessage->clear();
    }
    return true;
}

bool ReorderAnimationPathKey(
    AnimationPath* path,
    std::size_t sourceIndex,
    std::size_t destinationIndex) {
    if (path == nullptr || sourceIndex >= path->keys.size() ||
        destinationIndex >= path->keys.size() ||
        sourceIndex == destinationIndex) {
        return false;
    }
    BakeAnimationPathLocalizedCorrections(path);
    const auto durations = BuildSegmentDurations(*path);
    auto moved = std::move(path->keys[sourceIndex]);
    path->keys.erase(
        path->keys.begin() + static_cast<std::ptrdiff_t>(sourceIndex));
    path->keys.insert(
        path->keys.begin() +
            static_cast<std::ptrdiff_t>(destinationIndex),
        std::move(moved));
    for (std::size_t index = 1U; index < path->keys.size(); ++index) {
        path->keys[index].durationFrames = durations[index - 1U];
    }
    RebuildAnimationPathGeometryFromKeys(path);
    return true;
}

void RebuildAnimationPathGeometryFromKeys(AnimationPath* path) {
    if (path == nullptr) {
        return;
    }
    for (auto& key : path->keys) {
        key.splineParameterWeight = 0.0F;
        key.hasSplineEndpointTangent = false;
        key.splineCameraEndpointTangent = {0.0F, 0.0F, 0.0F};
        key.splineFocusEndpointTangent = {0.0F, 0.0F, 0.0F};
        key.splineOrientationEndpointTangent = {
            0.0F,
            0.0F,
            0.0F,
            0.0F,
        };
        key.splineLensEndpointTangent = {
            0.0F,
            0.0F,
            0.0F,
            0.0F,
            0.0F,
        };
    }
}

bool BakeAnimationPathLocalizedCorrections(AnimationPath* path) {
    if (path == nullptr || path->localizedKeyCorrections.empty()) {
        return false;
    }
    // Adjusted key poses already contain the visible correction. Removing
    // only the preserved base metadata makes those poses the new clean C2
    // spline controls instead of stacking old local weighting onto edits.
    path->localizedKeyCorrections.clear();
    RebuildAnimationPathGeometryFromKeys(path);
    return true;
}

bool ResetAnimationPathTimingWeights(AnimationPath* path) {
    if (path == nullptr || path->keys.size() < 2U) {
        return false;
    }

    const std::size_t segmentCount = path->keys.size() - 1U;
    const std::uint32_t targetTotalFrames = std::max<std::uint32_t>(
        path->durationFrames,
        static_cast<std::uint32_t>(segmentCount));
    const std::uint32_t baseFrames =
        targetTotalFrames / static_cast<std::uint32_t>(segmentCount);
    const std::uint32_t leftoverFrames =
        targetTotalFrames % static_cast<std::uint32_t>(segmentCount);

    bool changed = BakeAnimationPathLocalizedCorrections(path);
    for (std::size_t segmentIndex = 0U;
         segmentIndex < segmentCount;
         ++segmentIndex) {
        const std::uint32_t segmentFrames =
            baseFrames + (segmentIndex < leftoverFrames ? 1U : 0U);
        auto& storedFrames =
            path->keys[segmentIndex + 1U].durationFrames;
        if (storedFrames != segmentFrames) {
            storedFrames = segmentFrames;
            changed = true;
        }
    }
    return changed;
}

void MoveAnimationCameraKey(
    AnimationPath* path,
    std::size_t keyIndex,
    const std::array<float, 3>& cameraPosition) {
    if (path == nullptr || keyIndex >= path->keys.size()) {
        return;
    }
    BakeAnimationPathLocalizedCorrections(path);
    path->keys[keyIndex].cameraPosition = cameraPosition;
    RebuildAnimationPathGeometryFromKeys(path);
}

void MoveAnimationFocusKey(
    AnimationPath* path,
    std::size_t keyIndex,
    const std::array<float, 3>& focusPoint) {
    if (path == nullptr || keyIndex >= path->keys.size()) {
        return;
    }
    BakeAnimationPathLocalizedCorrections(path);
    path->keys[keyIndex].focusPoint = focusPoint;
    RebuildAnimationPathGeometryFromKeys(path);
}

void CollectRayHitDistancesAlongRay(
    std::span<const invisible_places::io::Float3> points,
    const std::array<float, 3>& origin,
    const std::array<float, 3>& normalizedDirection,
    float perpendicularRadiusMeters,
    float minDistanceMeters,
    float maxDistanceMeters,
    std::vector<float>* distances) {
    if (distances == nullptr || perpendicularRadiusMeters <= 0.0F ||
        maxDistanceMeters <= minDistanceMeters) {
        return;
    }
    const glm::vec3 rayOrigin{origin[0], origin[1], origin[2]};
    const glm::vec3 rayDirection{
        normalizedDirection[0],
        normalizedDirection[1],
        normalizedDirection[2]};
    const float radiusSquared = perpendicularRadiusMeters * perpendicularRadiusMeters;
    for (const auto& point : points) {
        const glm::vec3 offset =
            glm::vec3{point.x, point.y, point.z} - rayOrigin;
        const float alongRay = glm::dot(offset, rayDirection);
        if (alongRay < minDistanceMeters || alongRay > maxDistanceMeters) {
            continue;
        }
        const glm::vec3 perpendicular = offset - rayDirection * alongRay;
        if (glm::dot(perpendicular, perpendicular) > radiusSquared) {
            continue;
        }
        distances->push_back(alongRay);
    }
}

std::optional<float> ResolveFirstRayHitCluster(
    std::vector<float> distances,
    float clusterDepthMeters,
    std::size_t minimumClusterSamples) {
    if (distances.empty()) {
        return std::nullopt;
    }
    std::sort(distances.begin(), distances.end());
    const float clusterDepth = std::max(0.0F, clusterDepthMeters);
    for (std::size_t index = 0U; index < distances.size(); ++index) {
        const float clusterStart = distances[index];
        std::size_t clusterEnd = index;
        float clusterSum = 0.0F;
        while (clusterEnd < distances.size() &&
               distances[clusterEnd] - clusterStart <= clusterDepth) {
            clusterSum += distances[clusterEnd];
            ++clusterEnd;
        }
        const auto clusterCount = clusterEnd - index;
        if (clusterCount >= minimumClusterSamples) {
            return clusterSum / static_cast<float>(clusterCount);
        }
    }
    // No dense cluster; a sparse surface (thin vegetation) still deserves the
    // nearest candidate rather than no focus at all.
    return distances.front();
}

}  // namespace invisible_places::camera
