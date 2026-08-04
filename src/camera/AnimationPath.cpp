#include "camera/AnimationPath.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>

#include <glm/geometric.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/mat3x3.hpp>
#include <glm/trigonometric.hpp>
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

AnimationPreparedScalarSpline BuildNaturalCubicSpline(
    const std::vector<float>& knots,
    const std::vector<float>& values) {
    AnimationPreparedScalarSpline spline{
        .values = values,
        .secondDerivatives = std::vector<float>(values.size(), 0.0F),
    };

    if (knots.size() < 3U || knots.size() != values.size()) {
        return spline;
    }

    const std::size_t systemSize = knots.size() - 2U;
    std::vector<float> lower(systemSize, 0.0F);
    std::vector<float> diagonal(systemSize, 0.0F);
    std::vector<float> upper(systemSize, 0.0F);
    std::vector<float> rhs(systemSize, 0.0F);

    for (std::size_t row = 0; row < systemSize; ++row) {
        const std::size_t knotIndex = row + 1U;
        const float previousInterval = knots[knotIndex] - knots[knotIndex - 1U];
        const float nextInterval = knots[knotIndex + 1U] - knots[knotIndex];
        if (previousInterval <= 1.0e-6F || nextInterval <= 1.0e-6F) {
            continue;
        }

        lower[row] = row > 0U ? previousInterval : 0.0F;
        diagonal[row] = 2.0F * (previousInterval + nextInterval);
        upper[row] = (row + 1U) < systemSize ? nextInterval : 0.0F;
        rhs[row] =
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

    for (std::size_t row = 0; row < systemSize; ++row) {
        spline.secondDerivatives[row + 1U] = solution[row];
    }
    return spline;
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

bool ValidLoopSmoothingMetadata(const AnimationPath& path) {
    if (!path.loopTransitionSmoothing.has_value() || path.keys.size() < 3U) {
        return false;
    }
    const auto& smoothing = path.loopTransitionSmoothing.value();
    return !smoothing.pairId.empty() &&
           !smoothing.firstKeyId.empty() &&
           !smoothing.lastKeyId.empty() &&
           path.keys.front().id == smoothing.firstKeyId &&
           path.keys.back().id == smoothing.lastKeyId;
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

glm::quat EvaluatePreparedOrientation(
    const PreparedAnimationPathEvaluationContext& context,
    float timeSeconds) {
    if (context.orientationQuaternions.empty()) {
        return glm::quat{1.0F, 0.0F, 0.0F, 0.0F};
    }
    if (context.orientationQuaternions.size() == 1U ||
        context.knots.size() != context.orientationQuaternions.size()) {
        return QuaternionFromArray(context.orientationQuaternions.front());
    }

    const float clampedTime = std::clamp(timeSeconds, context.knots.front(), context.knots.back());
    if (clampedTime <= context.knots.front()) {
        return QuaternionFromArray(context.orientationQuaternions.front());
    }
    if (clampedTime >= context.knots.back()) {
        return QuaternionFromArray(context.orientationQuaternions.back());
    }

    const auto upper = std::upper_bound(context.knots.begin(), context.knots.end(), clampedTime);
    const std::size_t rightIndex =
        std::clamp<std::size_t>(
            static_cast<std::size_t>(upper - context.knots.begin()),
            1U,
            context.knots.size() - 1U);
    const std::size_t leftIndex = rightIndex - 1U;
    const float interval = context.knots[rightIndex] - context.knots[leftIndex];
    const float amount = interval <= 1.0e-6F ? 0.0F : (clampedTime - context.knots[leftIndex]) / interval;

    auto left = QuaternionFromArray(context.orientationQuaternions[leftIndex]);
    auto right = QuaternionFromArray(context.orientationQuaternions[rightIndex]);
    if (glm::dot(left, right) < 0.0F) {
        right = -right;
    }
    return glm::normalize(glm::slerp(left, right, std::clamp(amount, 0.0F, 1.0F)));
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
    std::array<float, 2> screenVelocity{0.0F, 0.0F};
};

PerceivedFlowProbe ProbePerceivedFlow(
    const PreparedAnimationPathEvaluationContext& context,
    float timeSeconds,
    float deltaSeconds) {
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
    if (probe.screenSpeed <= 1.0e-8F) {
        return probe;
    }

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
    if (directionLength > 1.0e-8F && std::isfinite(directionLength)) {
        probe.screenVelocity = {
            probe.screenSpeed * direction[0] / directionLength,
            probe.screenSpeed * direction[1] / directionLength,
        };
    }
    return probe;
}

float PerceivedFlowScreenHeightsPerSecond(
    const PreparedAnimationPathEvaluationContext& context,
    float timeSeconds,
    float deltaSeconds) {
    return ProbePerceivedFlow(context, timeSeconds, deltaSeconds).screenSpeed;
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

    AnimationPathKey originalFirstKey = path.keys.front();
    AnimationPathKey originalLastKey = path.keys.back();
    if (ValidLoopSmoothingMetadata(path)) {
        const auto& smoothing = path.loopTransitionSmoothing.value();
        originalFirstKey.cameraPosition = smoothing.originalFirstCameraPosition;
        originalFirstKey.focusPoint = smoothing.originalFirstFocusPoint;
        originalLastKey.cameraPosition = smoothing.originalLastCameraPosition;
        originalLastKey.focusPoint = smoothing.originalLastFocusPoint;

        cameraX.front() = originalFirstKey.cameraPosition[0];
        cameraY.front() = originalFirstKey.cameraPosition[1];
        cameraZ.front() = originalFirstKey.cameraPosition[2];
        focusX.front() = originalFirstKey.focusPoint[0];
        focusY.front() = originalFirstKey.focusPoint[1];
        focusZ.front() = originalFirstKey.focusPoint[2];
        cameraX.back() = originalLastKey.cameraPosition[0];
        cameraY.back() = originalLastKey.cameraPosition[1];
        cameraZ.back() = originalLastKey.cameraPosition[2];
        focusX.back() = originalLastKey.focusPoint[0];
        focusY.back() = originalLastKey.focusPoint[1];
        focusZ.back() = originalLastKey.focusPoint[2];
        focusDistances.front() = ReadFocusDistance(originalFirstKey);
        focusDistances.back() = ReadFocusDistance(originalLastKey);

        context.hasLoopEndpointCorrections = true;
        context.firstCameraCorrection = Difference(
            path.keys.front().cameraPosition,
            smoothing.originalFirstCameraPosition);
        context.firstFocusCorrection = Difference(
            path.keys.front().focusPoint,
            smoothing.originalFirstFocusPoint);
        context.lastCameraCorrection = Difference(
            path.keys.back().cameraPosition,
            smoothing.originalLastCameraPosition);
        context.lastFocusCorrection = Difference(
            path.keys.back().focusPoint,
            smoothing.originalLastFocusPoint);
    }

    // Linked-loop keys are already per-frame samples of the source clips and
    // crossfades. Linear interpolation preserves holds and avoids natural-
    // cubic ringing before a cut; ordinary authored paths retain their cubic
    // pass-through interpolation.
    const auto buildSpline = [&](std::vector<float> values) {
        if (path.linkedLoop.has_value()) {
            return AnimationPreparedScalarSpline{
                .values = std::move(values),
                .secondDerivatives =
                    std::vector<float>(path.keys.size(), 0.0F),
            };
        }
        return BuildNaturalCubicSpline(context.knots, values);
    };
    context.cameraX = buildSpline(std::move(cameraX));
    context.cameraY = buildSpline(std::move(cameraY));
    context.cameraZ = buildSpline(std::move(cameraZ));
    context.focusX = buildSpline(std::move(focusX));
    context.focusY = buildSpline(std::move(focusY));
    context.focusZ = buildSpline(std::move(focusZ));
    context.fovDegrees = buildSpline(BuildKeySamples(path, ReadFovDegrees));
    context.nearPlane = buildSpline(BuildKeySamples(path, ReadNearPlane));
    context.farPlane = buildSpline(BuildKeySamples(path, ReadFarPlane));
    context.focusDistance = buildSpline(std::move(focusDistances));
    context.apertureFStopsSpline =
        buildSpline(BuildKeySamples(path, ReadApertureFStops));
    context.orientationQuaternions.reserve(path.keys.size());
    for (std::size_t keyIndex = 0; keyIndex < path.keys.size(); ++keyIndex) {
        const auto& key = keyIndex == 0U
                              ? originalFirstKey
                              : (keyIndex + 1U == path.keys.size()
                                     ? originalLastKey
                                     : path.keys[keyIndex]);
        context.orientationQuaternions.push_back(QuaternionToArray(OrientationFromKey(key)));
    }

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

    const float clampedTimeSeconds = std::clamp(timeSeconds, 0.0F, context.knots.back());
    evaluation.camera.position = {
        EvaluatePreparedScalarSpline(context.knots, context.cameraX, clampedTimeSeconds),
        EvaluatePreparedScalarSpline(context.knots, context.cameraY, clampedTimeSeconds),
        EvaluatePreparedScalarSpline(context.knots, context.cameraZ, clampedTimeSeconds),
    };
    evaluation.focusPoint = {
        EvaluatePreparedScalarSpline(context.knots, context.focusX, clampedTimeSeconds),
        EvaluatePreparedScalarSpline(context.knots, context.focusY, clampedTimeSeconds),
        EvaluatePreparedScalarSpline(context.knots, context.focusZ, clampedTimeSeconds),
    };
    if (context.hasLoopEndpointCorrections && context.knots.size() >= 3U) {
        std::array<float, 3> cameraCorrection{0.0F, 0.0F, 0.0F};
        std::array<float, 3> focusCorrection{0.0F, 0.0F, 0.0F};
        if (clampedTimeSeconds <= context.knots[1U]) {
            const float interval = context.knots[1U] - context.knots[0U];
            const float amount = interval <= 1.0e-6F
                                     ? 1.0F
                                     : std::clamp(
                                           (clampedTimeSeconds - context.knots[0U]) / interval,
                                           0.0F,
                                           1.0F);
            const float weight = (1.0F - amount) * (1.0F - amount);
            for (std::size_t component = 0U; component < 3U; ++component) {
                cameraCorrection[component] = context.firstCameraCorrection[component] * weight;
                focusCorrection[component] = context.firstFocusCorrection[component] * weight;
            }
        } else if (clampedTimeSeconds >= context.knots[context.knots.size() - 2U]) {
            const std::size_t leftIndex = context.knots.size() - 2U;
            const float interval = context.knots.back() - context.knots[leftIndex];
            const float amount = interval <= 1.0e-6F
                                     ? 1.0F
                                     : std::clamp(
                                           (clampedTimeSeconds - context.knots[leftIndex]) / interval,
                                           0.0F,
                                           1.0F);
            const float weight = amount * amount;
            for (std::size_t component = 0U; component < 3U; ++component) {
                cameraCorrection[component] = context.lastCameraCorrection[component] * weight;
                focusCorrection[component] = context.lastFocusCorrection[component] * weight;
            }
        }
        for (std::size_t component = 0U; component < 3U; ++component) {
            evaluation.camera.position[component] += cameraCorrection[component];
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
            EvaluatePreparedScalarSpline(context.knots, context.focusDistance, clampedTimeSeconds);
    }
    if (context.hasOrientation) {
        WriteQuaternionToCameraState(
            EvaluatePreparedOrientation(context, clampedTimeSeconds),
            &evaluation.camera);
    } else {
        WriteQuaternionToCameraState(LookAtOrientation(cameraPosition, focusPoint), &evaluation.camera);
    }

    evaluation.camera.fovDegrees =
        EvaluatePreparedScalarSpline(context.knots, context.fovDegrees, clampedTimeSeconds);
    evaluation.camera.nearPlane =
        EvaluatePreparedScalarSpline(context.knots, context.nearPlane, clampedTimeSeconds);
    evaluation.camera.farPlane =
        EvaluatePreparedScalarSpline(context.knots, context.farPlane, clampedTimeSeconds);
    evaluation.camera.hasDepthOfField = context.depthOfFieldEnabled;
    evaluation.camera.focusDistance = evaluation.focusDistance;
    evaluation.camera.apertureFStops =
        context.hasApertureFStops
            ? EvaluatePreparedScalarSpline(context.knots, context.apertureFStopsSpline, clampedTimeSeconds)
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
    }
    return flow;
}

std::vector<std::uint32_t> ComputeConstantPerceivedSpeedSegmentFrames(
    const AnimationPath& path,
    std::uint32_t samplesPerSegment) {
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
            std::clamp<std::uint32_t>(samplesPerSegment, 4U, 256U);
        const auto totalSamples = interiorSamples * static_cast<std::uint32_t>(segmentCount);
        const float deltaSeconds = std::min(
            std::max(
                context.durationSeconds /
                    static_cast<float>(std::max<std::uint32_t>(totalSamples, 30U)),
                1.0F / 240.0F),
            context.durationSeconds);
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
                segmentFlow +=
                    PerceivedFlowScreenHeightsPerSecond(context, timeSeconds, deltaSeconds);
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

std::int32_t ClampLinkedLoopPaddingFrames(
    const AnimationPath& first,
    const AnimationPath& second,
    std::int32_t requestedPaddingFrames) {
    constexpr std::int64_t kMaximumPositivePaddingFrames = 3600LL;
    const auto firstFrames = std::max(
        first.durationFrames,
        MinimumAnimationDurationFrames(first));
    const auto secondFrames = std::max(
        second.durationFrames,
        MinimumAnimationDurationFrames(second));
    const auto firstTerminalStart = static_cast<std::int64_t>(std::lround(
        AnimationPathKeyNormalizedPosition(
            first,
            first.keys.size() >= 2U ? first.keys.size() - 2U : 0U) *
        static_cast<float>(firstFrames)));
    const auto secondTerminalStart = static_cast<std::int64_t>(std::lround(
        AnimationPathKeyNormalizedPosition(
            second,
            second.keys.size() >= 2U ? second.keys.size() - 2U : 0U) *
        static_cast<float>(secondFrames)));
    const std::int64_t basePeriod =
        firstTerminalStart + secondTerminalStart;
    const std::int64_t largestSource = std::max<std::int64_t>(
        firstFrames,
        secondFrames);
    const auto ceilHalf = [](std::int64_t value) {
        return value >= 0LL ? (value + 1LL) / 2LL : value / 2LL;
    };
    // A and B each recur once per period. Keep the period at least as long
    // as either source so copies of the same source never overlap; that caps
    // the live blend at one A sample and one B sample.
    const std::int64_t minimumPadding = std::max({
        -firstTerminalStart,
        -secondTerminalStart,
        ceilHalf(largestSource - basePeriod),
        ceilHalf(2LL - basePeriod),
    });
    const std::int64_t maximumPadding = std::max(
        kMaximumPositivePaddingFrames,
        minimumPadding);
    return static_cast<std::int32_t>(std::clamp<std::int64_t>(
        requestedPaddingFrames,
        minimumPadding,
        maximumPadding));
}

AnimationLinkedLoopTiming ResolveLinkedLoopTiming(
    const AnimationPath& first,
    const AnimationPath& second,
    std::int32_t requestedPaddingFrames) {
    AnimationLinkedLoopTiming timing;
    timing.firstDurationFrames = std::max(
        first.durationFrames,
        MinimumAnimationDurationFrames(first));
    timing.secondDurationFrames = std::max(
        second.durationFrames,
        MinimumAnimationDurationFrames(second));
    timing.firstTerminalStartFrame = static_cast<std::uint32_t>(std::clamp<long>(
        std::lround(
            AnimationPathKeyNormalizedPosition(
                first,
                first.keys.size() >= 2U
                    ? first.keys.size() - 2U
                    : 0U) *
            static_cast<float>(timing.firstDurationFrames)),
        0L,
        static_cast<long>(timing.firstDurationFrames)));
    timing.secondTerminalStartFrame = static_cast<std::uint32_t>(std::clamp<long>(
        std::lround(
            AnimationPathKeyNormalizedPosition(
                second,
                second.keys.size() >= 2U
                    ? second.keys.size() - 2U
                    : 0U) *
            static_cast<float>(timing.secondDurationFrames)),
        0L,
        static_cast<long>(timing.secondDurationFrames)));
    timing.paddingFrames = ClampLinkedLoopPaddingFrames(
        first,
        second,
        requestedPaddingFrames);
    const std::int64_t periodFrames =
        static_cast<std::int64_t>(timing.firstTerminalStartFrame) +
        static_cast<std::int64_t>(timing.secondTerminalStartFrame) +
        (2LL * static_cast<std::int64_t>(timing.paddingFrames));
    timing.periodFrames = static_cast<std::uint32_t>(std::max<std::int64_t>(
        2LL,
        periodFrames));
    timing.secondStartFrame =
        static_cast<float>(timing.firstTerminalStartFrame) +
        static_cast<float>(timing.paddingFrames);
    return timing;
}

namespace {

struct ActiveLinkedClip {
    const PreparedAnimationPathEvaluationContext* context = nullptr;
    bool first = false;
    float startFrame = 0.0F;
    float durationFrames = 0.0F;
};

AnimationPathEvaluation EvaluateActiveLinkedClip(
    const ActiveLinkedClip& clip,
    float frame) {
    const float localFrame = std::clamp(
        frame - clip.startFrame,
        0.0F,
        clip.durationFrames);
    return EvaluatePreparedAnimationPath(
        *clip.context,
        localFrame / kAnimationFramesPerSecond);
}

AnimationPathEvaluation BlendLinkedEvaluations(
    const AnimationPathEvaluation& outgoing,
    const AnimationPathEvaluation& incoming,
    float amount) {
    const float t = std::clamp(amount, 0.0F, 1.0F);
    AnimationPathEvaluation result;
    result.camera = InterpolateCameraStates(
        outgoing.camera,
        incoming.camera,
        t);
    for (std::size_t component = 0U; component < 3U; ++component) {
        result.focusPoint[component] = LerpFloat(
            outgoing.focusPoint[component],
            incoming.focusPoint[component],
            t);
    }
    result.focusDistance = LerpFloat(
        outgoing.focusDistance,
        incoming.focusDistance,
        t);
    result.camera.target = result.focusPoint;
    result.camera.orbitCenter = result.focusPoint;
    result.camera.hasOrbitCenter = true;
    result.camera.focusDistance = result.focusDistance;
    return result;
}

AnimationLinkedLoopSourceSample EvaluatePreparedLinkedLoopSourceSample(
    const PreparedAnimationPathEvaluationContext& firstContext,
    const PreparedAnimationPathEvaluationContext& secondContext,
    const AnimationLinkedLoopTiming& timing,
    float firstStartPosition,
    float linkedNormalizedPosition) {
    AnimationLinkedLoopSourceSample sample;
    sample.valid = true;
    const float periodFrameCount = static_cast<float>(timing.periodFrames);
    const float firstStartFrame =
        std::clamp(firstStartPosition, 0.0F, 1.0F) *
        static_cast<float>(timing.firstDurationFrames);
    float frame = std::fmod(
        std::clamp(linkedNormalizedPosition, 0.0F, 1.0F) *
                periodFrameCount +
            firstStartFrame,
        periodFrameCount);
    if (frame < 0.0F) {
        frame += periodFrameCount;
    }

    std::vector<ActiveLinkedClip> active;
    active.reserve(2U);
    std::optional<ActiveLinkedClip> latestCompleted;
    float latestEndFrame = -std::numeric_limits<float>::infinity();
    for (int cycleOffset = -1; cycleOffset <= 1; ++cycleOffset) {
        const float cycleStart =
            static_cast<float>(cycleOffset) * periodFrameCount;
        const std::array<ActiveLinkedClip, 2> clips{{
            {.context = &firstContext,
             .first = true,
             .startFrame = cycleStart,
             .durationFrames =
                 static_cast<float>(timing.firstDurationFrames)},
            {.context = &secondContext,
             .first = false,
             .startFrame = cycleStart + timing.secondStartFrame,
             .durationFrames =
                 static_cast<float>(timing.secondDurationFrames)},
        }};
        for (const auto& clip : clips) {
            const float clipEnd = clip.startFrame + clip.durationFrames;
            if (frame >= clip.startFrame && frame < clipEnd) {
                active.push_back(clip);
            }
            if (clipEnd <= frame && clipEnd > latestEndFrame) {
                latestEndFrame = clipEnd;
                latestCompleted = clip;
            }
        }
    }

    if (active.empty()) {
        if (latestCompleted.has_value()) {
            sample.blended = EvaluateActiveLinkedClip(
                latestCompleted.value(),
                latestCompleted->startFrame +
                    latestCompleted->durationFrames);
        } else {
            sample.blended = EvaluatePreparedAnimationPath(
                firstContext,
                0.0F);
        }
        return sample;
    }

    const auto assignSource = [&](const ActiveLinkedClip& clip,
                                  const AnimationPathEvaluation& evaluation,
                                  float weight,
                                  AnimationLinkedLoopSourceSample* output) {
        if (clip.first) {
            output->first = evaluation;
            output->firstActive = true;
            output->firstWeight = weight;
        } else {
            output->second = evaluation;
            output->secondActive = true;
            output->secondWeight = weight;
        }
    };
    if (active.size() == 1U) {
        const auto evaluation = EvaluateActiveLinkedClip(
            active.front(),
            frame);
        assignSource(active.front(), evaluation, 1.0F, &sample);
        sample.blended = evaluation;
        return sample;
    }

    std::sort(
        active.begin(),
        active.end(),
        [](const auto& left, const auto& right) {
            return left.startFrame < right.startFrame;
        });
    const auto& outgoing = active[active.size() - 2U];
    const auto& incoming = active.back();
    const float overlapEnd = std::min(
        outgoing.startFrame + outgoing.durationFrames,
        incoming.startFrame + incoming.durationFrames);
    const float overlapFrames = std::max(
        1.0F,
        overlapEnd - incoming.startFrame);
    const float linearAmount = std::clamp(
        (frame - incoming.startFrame) / overlapFrames,
        0.0F,
        1.0F);
    const float smoothAmount =
        linearAmount * linearAmount * (3.0F - (2.0F * linearAmount));
    const auto outgoingEvaluation = EvaluateActiveLinkedClip(
        outgoing,
        frame);
    const auto incomingEvaluation = EvaluateActiveLinkedClip(
        incoming,
        frame);
    assignSource(outgoing, outgoingEvaluation, 1.0F - smoothAmount, &sample);
    assignSource(incoming, incomingEvaluation, smoothAmount, &sample);
    sample.blended = BlendLinkedEvaluations(
        outgoingEvaluation,
        incomingEvaluation,
        smoothAmount);
    return sample;
}

}  // namespace

AnimationLinkedLoopSourceSample EvaluateLinkedLoopSourceSample(
    const AnimationPath& first,
    const AnimationPath& second,
    float firstStartPosition,
    std::int32_t paddingFrames,
    float linkedNormalizedPosition) {
    const auto firstContext = PrepareAnimationPathEvaluation(first);
    const auto secondContext = PrepareAnimationPathEvaluation(second);
    if (!firstContext.valid || !secondContext.valid) {
        return {};
    }
    return EvaluatePreparedLinkedLoopSourceSample(
        firstContext,
        secondContext,
        ResolveLinkedLoopTiming(first, second, paddingFrames),
        firstStartPosition,
        linkedNormalizedPosition);
}

std::optional<AnimationPath> BuildLinkedLoopAnimation(
    const AnimationPath& first,
    const AnimationPath& second,
    const AnimationLinkedLoopBuildOptions& options,
    std::string* errorMessage) {
    const auto fail = [&](std::string message)
        -> std::optional<AnimationPath> {
        if (errorMessage != nullptr) {
            *errorMessage = std::move(message);
        }
        return std::nullopt;
    };
    if (first.keys.size() < 3U || second.keys.size() < 3U) {
        return fail(
            "A linked loop requires at least three keys in each source animation so each terminal edge is defined.");
    }
    if (options.firstStartKeyIndex >= first.keys.size()) {
        return fail("The linked-loop start key is no longer present in the first animation.");
    }

    const auto firstContext = PrepareAnimationPathEvaluation(first);
    const auto secondContext = PrepareAnimationPathEvaluation(second);
    if (!firstContext.valid || firstContext.singleKey ||
        !secondContext.valid || secondContext.singleKey) {
        return fail("One of the linked-loop source animations cannot be evaluated.");
    }

    const auto timing = ResolveLinkedLoopTiming(
        first,
        second,
        options.paddingFrames);
    constexpr std::int64_t kMaximumGeneratedLinkedFrames = 36000LL;
    if (timing.periodFrames > kMaximumGeneratedLinkedFrames) {
        return fail(
            "The linked loop would exceed 36,000 generated frames. Shorten the sources or padding.");
    }

    const float firstStartPosition = AnimationPathKeyNormalizedPosition(
        first,
        options.firstStartKeyIndex);

    AnimationPath linked = first;
    linked.name = options.name.empty()
                      ? first.name + " + " + second.name + " Linked"
                      : options.name;
    linked.durationFrames = timing.periodFrames;
    linked.keys.clear();
    linked.keys.reserve(static_cast<std::size_t>(timing.periodFrames) + 1U);
    linked.loopTransitionSmoothing.reset();
    linked.linkedLoop = AnimationLinkedLoopMetadata{
        .firstFileName = options.firstFileName.empty()
                             ? first.name
                             : options.firstFileName,
        .secondFileName = options.secondFileName.empty()
                              ? second.name
                              : options.secondFileName,
        .firstStartKeyId = first.keys[options.firstStartKeyIndex].id,
        .firstStartPosition = firstStartPosition,
        .paddingFrames = timing.paddingFrames,
    };
    linked.depthOfFieldEnabled =
        first.depthOfFieldEnabled || second.depthOfFieldEnabled;
    for (const auto& layerPath : second.associatedLayerPaths) {
        if (std::find(
                linked.associatedLayerPaths.begin(),
                linked.associatedLayerPaths.end(),
                layerPath) == linked.associatedLayerPaths.end()) {
            linked.associatedLayerPaths.push_back(layerPath);
        }
    }

    for (std::uint32_t frameIndex = 0U;
         frameIndex <= timing.periodFrames;
         ++frameIndex) {
        const auto sourceSample = EvaluatePreparedLinkedLoopSourceSample(
            firstContext,
            secondContext,
            timing,
            firstStartPosition,
            static_cast<float>(frameIndex) /
                static_cast<float>(timing.periodFrames));
        const auto& evaluation = sourceSample.blended;
        AnimationPathKey key;
        key.id = "linked_frame_" + std::to_string(frameIndex + 1U);
        key.cameraPosition = evaluation.camera.position;
        key.focusPoint = evaluation.focusPoint;
        key.hasOrientation = true;
        key.orientation = evaluation.camera.orientation;
        key.hasFocusDistance = true;
        key.focusDistance = std::max(0.001F, evaluation.focusDistance);
        key.hasApertureFStops = true;
        key.apertureFStops = std::max(
            0.1F,
            evaluation.camera.apertureFStops);
        key.fovDegrees = evaluation.camera.fovDegrees;
        key.nearPlane = evaluation.camera.nearPlane;
        key.farPlane = evaluation.camera.farPlane;
        key.durationFrames = 1U;
        key.sourceShotName = "Linked frame " +
                             std::to_string(frameIndex + 1U);
        linked.keys.push_back(std::move(key));
    }

    if (errorMessage != nullptr) {
        errorMessage->clear();
    }
    return linked;
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

AnimationLoopSmoothingMetadata MakeLoopSmoothingMetadata(
    const AnimationPath& path,
    std::string pairId,
    std::string partnerFileName,
    std::uint32_t sequenceIndex,
    float maxEndMoveFraction) {
    return {
        .pairId = std::move(pairId),
        .partnerFileName = std::move(partnerFileName),
        .sequenceIndex = sequenceIndex,
        .maxEndMoveFraction = maxEndMoveFraction,
        .firstKeyId = path.keys.front().id,
        .lastKeyId = path.keys.back().id,
        .originalFirstCameraPosition = path.keys.front().cameraPosition,
        .originalFirstFocusPoint = path.keys.front().focusPoint,
        .originalLastCameraPosition = path.keys.back().cameraPosition,
        .originalLastFocusPoint = path.keys.back().focusPoint,
    };
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

struct LoopScore {
    float objective = 0.0F;
    float mismatch = 0.0F;
    std::array<float, 2> seamMismatch{0.0F, 0.0F};
    std::array<float, 2> terminalSpeedRmsChange{0.0F, 0.0F};
};

LoopScore ScoreLoopPair(
    const AnimationPath& first,
    const AnimationPath& second,
    const PreparedAnimationPathEvaluationContext& originalFirst,
    const PreparedAnimationPathEvaluationContext& originalSecond,
    const std::vector<LoopEndpointGroup>& groups) {
    constexpr std::uint32_t kTerminalSampleIntervals = 8U;
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
    float totalWeight = 0.0F;
    std::array<float, 2> seamWeights{0.0F, 0.0F};
    std::array<float, 2> speedErrorWeights{0.0F, 0.0F};
    const std::array<std::pair<std::size_t, std::size_t>, 2> seams{{
        {0U, 1U},
        {1U, 0U},
    }};
    for (std::size_t seamIndex = 0U; seamIndex < seams.size(); ++seamIndex) {
        const auto [outgoingIndex, incomingIndex] = seams[seamIndex];
        const auto& outgoing = *candidates[outgoingIndex];
        const auto& incoming = *candidates[incomingIndex];
        const auto& originalOutgoing = *originals[outgoingIndex];
        const auto& originalIncoming = *originals[incomingIndex];
        const float outgoingSegment = outgoing.knots.back() - outgoing.knots[outgoing.knots.size() - 2U];
        const float incomingSegment = incoming.knots[1U] - incoming.knots[0U];
        const float outgoingDelta = LoopProbeDeltaSeconds(outgoing, outgoingSegment);
        const float incomingDelta = LoopProbeDeltaSeconds(incoming, incomingSegment);
        for (std::uint32_t sampleIndex = 0U;
             sampleIndex <= kTerminalSampleIntervals;
             ++sampleIndex) {
            const float inward =
                static_cast<float>(sampleIndex) /
                static_cast<float>(kTerminalSampleIntervals);
            const float weight = (1.0F - inward) * (1.0F - inward);
            if (weight <= 0.0F) {
                continue;
            }
            const float outgoingTime = outgoing.knots.back() - inward * outgoingSegment;
            const float incomingTime = incoming.knots.front() + inward * incomingSegment;
            const auto outgoingFlow = ProbePerceivedFlow(outgoing, outgoingTime, outgoingDelta);
            const auto incomingFlow = ProbePerceivedFlow(incoming, incomingTime, incomingDelta);
            const auto originalOutgoingFlow = ProbePerceivedFlow(
                originalOutgoing,
                outgoingTime,
                outgoingDelta);
            const auto originalIncomingFlow = ProbePerceivedFlow(
                originalIncoming,
                incomingTime,
                incomingDelta);
            const float scale = std::max(
                0.5F * (originalOutgoingFlow.screenSpeed + originalIncomingFlow.screenSpeed),
                1.0e-3F);
            const float flowX = outgoingFlow.screenVelocity[0] - incomingFlow.screenVelocity[0];
            const float flowY = outgoingFlow.screenVelocity[1] - incomingFlow.screenVelocity[1];
            const float normalizedMismatch =
                ((flowX * flowX) + (flowY * flowY)) / (scale * scale);
            const float outgoingSpeedError =
                (outgoingFlow.screenSpeed - originalOutgoingFlow.screenSpeed) / scale;
            const float incomingSpeedError =
                (incomingFlow.screenSpeed - originalIncomingFlow.screenSpeed) / scale;
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

void RestoreLoopSmoothingOriginalEndpoints(AnimationPath* path) {
    if (path == nullptr || !ValidLoopSmoothingMetadata(*path)) {
        return;
    }
    const auto smoothing = path->loopTransitionSmoothing.value();
    path->keys.front().cameraPosition = smoothing.originalFirstCameraPosition;
    path->keys.front().focusPoint = smoothing.originalFirstFocusPoint;
    path->keys.back().cameraPosition = smoothing.originalLastCameraPosition;
    path->keys.back().focusPoint = smoothing.originalLastFocusPoint;
    path->loopTransitionSmoothing.reset();
}

void AccumulateLoopEndpointMovement(
    const AnimationPath& path,
    const AnimationLoopSmoothingMetadata& smoothing,
    AnimationLoopTransitionMetrics* metrics) {
    if (metrics == nullptr || path.keys.size() < 3U) {
        return;
    }
    const float moveFraction = std::clamp(
        smoothing.maxEndMoveFraction,
        0.01F,
        0.25F);
    const auto accumulateTrack = [](
                                     const std::array<float, 3>& current,
                                     const std::array<float, 3>& original,
                                     const std::array<float, 3>& adjacent,
                                     float fraction,
                                     float* maximumMove,
                                     float* maximumCapUsage) {
        const float movement = Distance(current, original);
        const float cap = fraction * Distance(original, adjacent);
        *maximumMove = std::max(*maximumMove, movement);
        if (cap > 1.0e-8F) {
            *maximumCapUsage = std::max(
                *maximumCapUsage,
                movement / cap);
        }
    };
    accumulateTrack(
        path.keys.front().cameraPosition,
        smoothing.originalFirstCameraPosition,
        path.keys[1U].cameraPosition,
        moveFraction,
        &metrics->maxCameraMove,
        &metrics->maxCameraCapUsage);
    accumulateTrack(
        path.keys.back().cameraPosition,
        smoothing.originalLastCameraPosition,
        path.keys[path.keys.size() - 2U].cameraPosition,
        moveFraction,
        &metrics->maxCameraMove,
        &metrics->maxCameraCapUsage);
    accumulateTrack(
        path.keys.front().focusPoint,
        smoothing.originalFirstFocusPoint,
        path.keys[1U].focusPoint,
        moveFraction,
        &metrics->maxFocusMove,
        &metrics->maxFocusCapUsage);
    accumulateTrack(
        path.keys.back().focusPoint,
        smoothing.originalLastFocusPoint,
        path.keys[path.keys.size() - 2U].focusPoint,
        moveFraction,
        &metrics->maxFocusMove,
        &metrics->maxFocusCapUsage);
}

}  // namespace

AnimationLoopSmoothingResult SmoothAnimationLoopTransitions(
    AnimationPath* first,
    AnimationPath* second,
    const AnimationLoopSmoothingOptions& options) {
    AnimationLoopSmoothingResult result;
    if (first == nullptr || second == nullptr || first == second) {
        result.errorMessage = "Choose two different animations.";
        return result;
    }
    if (first->keys.size() < 3U || second->keys.size() < 3U) {
        result.errorMessage = "Loop smoothing requires at least three camera keys in each animation.";
        return result;
    }
    const auto hasStableEndpointIds = [](const AnimationPath& path) {
        return !path.keys.front().id.empty() &&
               !path.keys.back().id.empty() &&
               path.keys.front().id != path.keys.back().id;
    };
    if (!hasStableEndpointIds(*first) || !hasStableEndpointIds(*second)) {
        result.errorMessage =
            "Loop smoothing requires distinct persistent IDs on each path's endpoint keys.";
        return result;
    }
    if (first->loopTransitionSmoothing.has_value() ||
        second->loopTransitionSmoothing.has_value()) {
        result.errorMessage = "Unapply the existing end smoothing before smoothing this pair again.";
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

    AnimationPath originalFirstPath = *first;
    AnimationPath originalSecondPath = *second;
    AnimationPath* paths[] = {first, second};
    std::vector<LoopEndpointGroup> groups;
    for (std::size_t pathIndex = 0U; pathIndex < 2U; ++pathIndex) {
        auto& path = *paths[pathIndex];
        for (const std::size_t keyIndex : {std::size_t{0U}, path.keys.size() - 1U}) {
            const bool start = keyIndex == 0U;
            const std::size_t adjacentIndex = start ? 1U : keyIndex - 1U;
            const auto& key = path.keys[keyIndex];
            LoopEndpointOccurrence occurrence{
                .pathIndex = pathIndex,
                .keyIndex = keyIndex,
                .originalCamera = key.cameraPosition,
                .originalFocus = key.focusPoint,
                .cameraCap = moveFraction * Distance(key.cameraPosition, path.keys[adjacentIndex].cameraPosition),
                .focusCap = moveFraction * Distance(key.focusPoint, path.keys[adjacentIndex].focusPoint),
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
                "A shared endpoint camera's saved poses are farther apart "
                "than the selected Max End Move allows.";
            return result;
        }
    }

    const std::string pairId = options.pairId.empty()
                                   ? first->name + "::" + second->name
                                   : options.pairId;
    first->loopTransitionSmoothing = MakeLoopSmoothingMetadata(
        *first,
        pairId,
        options.secondFileName,
        0U,
        moveFraction);
    second->loopTransitionSmoothing = MakeLoopSmoothingMetadata(
        *second,
        pairId,
        options.firstFileName,
        1U,
        moveFraction);

    const std::vector<LoopEndpointGroup> noMovementGroups;
    const auto beforeScore = ScoreLoopPair(
        *first,
        *second,
        originalFirst,
        originalSecond,
        noMovementGroups);
    result.beforeMismatch = beforeScore.mismatch;
    result.beforeSeamMismatch = beforeScore.seamMismatch;
    ApplyLoopEndpointGroups(first, second, groups);
    LoopScore bestScore = ScoreLoopPair(*first, *second, originalFirst, originalSecond, groups);
    float stepFraction = 0.25F;
    for (std::uint32_t sweep = 0U; sweep < 40U && stepFraction >= 0.005F; ++sweep) {
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
                            ScoreLoopPair(*first, *second, originalFirst, originalSecond, groups);
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
    const auto finalScore = ScoreLoopPair(*first, *second, originalFirst, originalSecond, groups);
    result.afterMismatch = finalScore.mismatch;
    result.afterSeamMismatch = finalScore.seamMismatch;
    result.terminalSpeedRmsChange = finalScore.terminalSpeedRmsChange;
    for (const auto& group : groups) {
        for (const auto& occurrence : group.occurrences) {
            const float cameraMove =
                Distance(group.cameraPosition, occurrence.originalCamera);
            const float focusMove =
                Distance(group.focusPoint, occurrence.originalFocus);
            result.maxCameraMove = std::max(result.maxCameraMove, cameraMove);
            result.maxFocusMove = std::max(result.maxFocusMove, focusMove);
            if (occurrence.cameraCap > 1.0e-8F) {
                result.maxCameraCapUsage = std::max(
                    result.maxCameraCapUsage,
                    cameraMove / occurrence.cameraCap);
            }
            if (occurrence.focusCap > 1.0e-8F) {
                result.maxFocusCapUsage = std::max(
                    result.maxFocusCapUsage,
                    focusMove / occurrence.focusCap);
            }
        }
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
    const bool endpointsMoved =
        result.maxCameraMove > 1.0e-6F ||
        result.maxFocusMove > 1.0e-6F;
    result.changed = overallImproved && neitherSeamWorsened && endpointsMoved;
    result.succeeded = true;
    if (!result.changed) {
        *first = std::move(originalFirstPath);
        *second = std::move(originalSecondPath);
        if (!endpointsMoved) {
            result.errorMessage =
                "No endpoint movement produced a measurable improvement "
                "within the selected limit.";
        } else if (!overallImproved) {
            result.errorMessage =
                "The best bounded endpoint movement did not measurably lower "
                "the combined screen-flow mismatch.";
        } else {
            result.errorMessage =
                "The best combined candidate was rejected because it made "
                "one loop direction worse.";
        }
    }
    return result;
}

AnimationLoopTransitionMetrics MeasureAnimationLoopTransitions(
    const AnimationPath& first,
    const AnimationPath& second) {
    AnimationLoopTransitionMetrics metrics;
    if (&first == &second) {
        metrics.errorMessage = "Choose two different animations.";
        return metrics;
    }
    if (first.keys.size() < 3U || second.keys.size() < 3U) {
        metrics.errorMessage =
            "Loop validation requires at least three camera keys in each animation.";
        return metrics;
    }

    const bool firstHasSmoothing = first.loopTransitionSmoothing.has_value();
    const bool secondHasSmoothing = second.loopTransitionSmoothing.has_value();
    if (firstHasSmoothing != secondHasSmoothing) {
        metrics.errorMessage =
            "Only one animation contains reversible loop-smoothing metadata.";
        return metrics;
    }
    if (firstHasSmoothing &&
        (!ValidLoopSmoothingMetadata(first) ||
         !ValidLoopSmoothingMetadata(second) ||
         first.loopTransitionSmoothing->pairId !=
             second.loopTransitionSmoothing->pairId)) {
        metrics.errorMessage =
            "The animations do not contain the same valid reversible smoothing pair.";
        return metrics;
    }

    AnimationPath originalFirst = first;
    AnimationPath originalSecond = second;
    if (firstHasSmoothing) {
        RestoreLoopSmoothingOriginalEndpoints(&originalFirst);
        RestoreLoopSmoothingOriginalEndpoints(&originalSecond);
    }
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

    const std::vector<LoopEndpointGroup> noMovementGroups;
    const auto beforeScore = ScoreLoopPair(
        originalFirst,
        originalSecond,
        originalFirstContext,
        originalSecondContext,
        noMovementGroups);
    const auto afterScore = firstHasSmoothing
                                ? ScoreLoopPair(
                                      first,
                                      second,
                                      originalFirstContext,
                                      originalSecondContext,
                                      noMovementGroups)
                                : beforeScore;
    metrics.valid = true;
    metrics.hasAppliedSmoothing = firstHasSmoothing;
    metrics.beforeMismatch = beforeScore.mismatch;
    metrics.afterMismatch = afterScore.mismatch;
    metrics.beforeSeamMismatch = beforeScore.seamMismatch;
    metrics.afterSeamMismatch = afterScore.seamMismatch;
    metrics.terminalSpeedRmsChange =
        afterScore.terminalSpeedRmsChange;
    if (firstHasSmoothing) {
        AccumulateLoopEndpointMovement(
            first,
            first.loopTransitionSmoothing.value(),
            &metrics);
        AccumulateLoopEndpointMovement(
            second,
            second.loopTransitionSmoothing.value(),
            &metrics);
    }
    return metrics;
}

bool UnapplyAnimationLoopSmoothing(
    AnimationPath* path,
    std::string* errorMessage) {
    if (path == nullptr || !path->loopTransitionSmoothing.has_value()) {
        if (errorMessage != nullptr) {
            *errorMessage = "This animation has no applied end smoothing.";
        }
        return false;
    }
    const auto smoothing = path->loopTransitionSmoothing.value();
    if (path->keys.size() < 3U ||
        path->keys.front().id != smoothing.firstKeyId ||
        path->keys.back().id != smoothing.lastKeyId) {
        if (errorMessage != nullptr) {
            *errorMessage = "The endpoint key IDs changed after smoothing; the saved endpoints were not restored.";
        }
        return false;
    }
    path->keys.front().cameraPosition = smoothing.originalFirstCameraPosition;
    path->keys.front().focusPoint = smoothing.originalFirstFocusPoint;
    path->keys.back().cameraPosition = smoothing.originalLastCameraPosition;
    path->keys.back().focusPoint = smoothing.originalLastFocusPoint;
    path->loopTransitionSmoothing.reset();
    if (errorMessage != nullptr) {
        errorMessage->clear();
    }
    return true;
}

AnimationPathEvaluation EvaluateAnimationPath(
    const AnimationPath& path,
    float timeSeconds) {
    return EvaluatePreparedAnimationPath(
        PrepareAnimationPathEvaluation(path),
        timeSeconds);
}

void MoveAnimationCameraKey(
    AnimationPath* path,
    std::size_t keyIndex,
    const std::array<float, 3>& cameraPosition) {
    if (path == nullptr || keyIndex >= path->keys.size()) {
        return;
    }
    path->keys[keyIndex].cameraPosition = cameraPosition;
}

void MoveAnimationFocusKey(
    AnimationPath* path,
    std::size_t keyIndex,
    const std::array<float, 3>& focusPoint) {
    if (path == nullptr || keyIndex >= path->keys.size()) {
        return;
    }
    path->keys[keyIndex].focusPoint = focusPoint;
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
