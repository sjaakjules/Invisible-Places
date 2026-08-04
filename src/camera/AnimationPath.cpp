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

float PerceivedFlowScreenHeightsPerSecond(
    const PreparedAnimationPathEvaluationContext& context,
    float timeSeconds,
    float deltaSeconds) {
    const float leftTime = std::max(0.0F, timeSeconds - deltaSeconds);
    const float rightTime = std::min(context.durationSeconds, timeSeconds + deltaSeconds);
    const float spanSeconds = rightTime - leftTime;
    if (spanSeconds <= 1.0e-6F) {
        return 0.0F;
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
    return std::isfinite(screenSpeed) ? std::max(screenSpeed, 0.0F) : 0.0F;
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

    context.cameraX = BuildNaturalCubicSpline(context.knots, BuildKeySamples(path, ReadCameraX));
    context.cameraY = BuildNaturalCubicSpline(context.knots, BuildKeySamples(path, ReadCameraY));
    context.cameraZ = BuildNaturalCubicSpline(context.knots, BuildKeySamples(path, ReadCameraZ));
    context.focusX = BuildNaturalCubicSpline(context.knots, BuildKeySamples(path, ReadFocusX));
    context.focusY = BuildNaturalCubicSpline(context.knots, BuildKeySamples(path, ReadFocusY));
    context.focusZ = BuildNaturalCubicSpline(context.knots, BuildKeySamples(path, ReadFocusZ));
    context.fovDegrees = BuildNaturalCubicSpline(context.knots, BuildKeySamples(path, ReadFovDegrees));
    context.nearPlane = BuildNaturalCubicSpline(context.knots, BuildKeySamples(path, ReadNearPlane));
    context.farPlane = BuildNaturalCubicSpline(context.knots, BuildKeySamples(path, ReadFarPlane));
    context.focusDistance = BuildNaturalCubicSpline(context.knots, BuildKeySamples(path, ReadFocusDistance));
    context.apertureFStopsSpline =
        BuildNaturalCubicSpline(context.knots, BuildKeySamples(path, ReadApertureFStops));
    context.orientationQuaternions.reserve(path.keys.size());
    for (const auto& key : path.keys) {
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
        sample.screenSpeed = PerceivedFlowScreenHeightsPerSecond(
            context,
            context.durationSeconds * sample.normalizedPosition,
            deltaSeconds);
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
