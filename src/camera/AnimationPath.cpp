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
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

namespace invisible_places::camera {

namespace {

constexpr float kAnimationFramesPerSecond = 30.0F;

struct NaturalCubicSpline {
    std::vector<float> knots;
    std::vector<float> values;
    std::vector<float> secondDerivatives;

    [[nodiscard]] float Evaluate(float x) const {
        if (knots.empty() || values.empty()) {
            return 0.0F;
        }
        if (knots.size() == 1U || values.size() == 1U) {
            return values.front();
        }

        const float clampedX = std::clamp(x, knots.front(), knots.back());
        if (clampedX <= knots.front()) {
            return values.front();
        }
        if (clampedX >= knots.back()) {
            return values.back();
        }

        const auto upper = std::upper_bound(knots.begin(), knots.end(), clampedX);
        const std::size_t rightIndex =
            std::clamp<std::size_t>(static_cast<std::size_t>(upper - knots.begin()), 1U, knots.size() - 1U);
        const std::size_t leftIndex = rightIndex - 1U;
        const float interval = knots[rightIndex] - knots[leftIndex];
        if (interval <= 1.0e-6F) {
            return values[leftIndex];
        }

        const float a = (knots[rightIndex] - clampedX) / interval;
        const float b = (clampedX - knots[leftIndex]) / interval;
        const float leftCurve = ((a * a * a) - a) * secondDerivatives[leftIndex];
        const float rightCurve = ((b * b * b) - b) * secondDerivatives[rightIndex];
        return (a * values[leftIndex]) +
               (b * values[rightIndex]) +
               ((leftCurve + rightCurve) * interval * interval / 6.0F);
    }
};

NaturalCubicSpline BuildNaturalCubicSpline(
    const std::vector<float>& knots,
    const std::vector<float>& values) {
    NaturalCubicSpline spline{
        .knots = knots,
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

std::array<float, 3> FromGlm(const glm::vec3& value) {
    return {value.x, value.y, value.z};
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

float EvaluateScalar(
    const AnimationPath& path,
    const std::vector<float>& knots,
    float timeSeconds,
    float (*readValue)(const AnimationPathKey& key)) {
    return BuildNaturalCubicSpline(knots, BuildKeySamples(path, readValue)).Evaluate(timeSeconds);
}

std::array<float, 3> EvaluateVector3(
    const AnimationPath& path,
    const std::vector<float>& knots,
    float timeSeconds,
    const std::array<float (*)(const AnimationPathKey& key), 3>& readers) {
    return {
        EvaluateScalar(path, knots, timeSeconds, readers[0]),
        EvaluateScalar(path, knots, timeSeconds, readers[1]),
        EvaluateScalar(path, knots, timeSeconds, readers[2]),
    };
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

glm::quat EvaluateOrientation(
    const AnimationPath& path,
    const std::vector<float>& knots,
    float timeSeconds) {
    if (path.keys.empty()) {
        return glm::quat{1.0F, 0.0F, 0.0F, 0.0F};
    }
    if (path.keys.size() == 1U || knots.size() != path.keys.size()) {
        return OrientationFromKey(path.keys.front());
    }

    const float clampedTime = std::clamp(timeSeconds, knots.front(), knots.back());
    if (clampedTime <= knots.front()) {
        return OrientationFromKey(path.keys.front());
    }
    if (clampedTime >= knots.back()) {
        return OrientationFromKey(path.keys.back());
    }

    const auto upper = std::upper_bound(knots.begin(), knots.end(), clampedTime);
    const std::size_t rightIndex =
        std::clamp<std::size_t>(static_cast<std::size_t>(upper - knots.begin()), 1U, knots.size() - 1U);
    const std::size_t leftIndex = rightIndex - 1U;
    const float interval = knots[rightIndex] - knots[leftIndex];
    const float amount = interval <= 1.0e-6F ? 0.0F : (clampedTime - knots[leftIndex]) / interval;

    auto left = OrientationFromKey(path.keys[leftIndex]);
    auto right = OrientationFromKey(path.keys[rightIndex]);
    if (glm::dot(left, right) < 0.0F) {
        right = -right;
    }
    return glm::normalize(glm::slerp(left, right, std::clamp(amount, 0.0F, 1.0F)));
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

AnimationPathMotionStats MeasureAnimationPathMotion(
    const AnimationPath& path,
    float normalizedTime,
    std::uint32_t sampleCount) {
    AnimationPathMotionStats stats;
    if (path.keys.empty()) {
        return stats;
    }

    stats.durationSeconds = AnimationPathDurationSeconds(path);
    if (stats.durationSeconds <= 1.0e-6F) {
        return stats;
    }

    const std::uint32_t steps = std::max<std::uint32_t>(1U, sampleCount);
    auto previous = EvaluateAnimationPath(path, 0.0F);
    for (std::uint32_t step = 1U; step <= steps; ++step) {
        const float timeSeconds =
            stats.durationSeconds * (static_cast<float>(step) / static_cast<float>(steps));
        const auto current = EvaluateAnimationPath(path, timeSeconds);
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
        const auto left = EvaluateAnimationPath(path, leftTime);
        const auto right = EvaluateAnimationPath(path, rightTime);
        stats.currentCameraSpeed = Distance(left.camera.position, right.camera.position) / spanSeconds;
        stats.currentTargetSpeed = Distance(left.focusPoint, right.focusPoint) / spanSeconds;
    }

    return stats;
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

    const auto stats = MeasureAnimationPathMotion(path, 0.0F, sampleCount);
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

AnimationPathEvaluation EvaluateAnimationPath(
    const AnimationPath& path,
    float timeSeconds) {
    AnimationPathEvaluation evaluation;
    if (path.keys.empty()) {
        return evaluation;
    }

    if (path.keys.size() == 1U) {
        const auto& key = path.keys.front();
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
        evaluation.camera.hasDepthOfField = path.depthOfFieldEnabled;
        evaluation.camera.focusDistance = evaluation.focusDistance;
        evaluation.camera.apertureFStops = key.hasApertureFStops
                                               ? std::max(0.1F, key.apertureFStops)
                                               : std::max(0.1F, path.apertureFStops);
        evaluation.camera.depthOfFieldMaxBlurPixels = std::max(0.0F, path.depthOfFieldMaxBlurPixels);
        return evaluation;
    }

    const auto knots = BuildAnimationKnots(path);
    if (knots.size() != path.keys.size() || knots.empty()) {
        return evaluation;
    }

    const float clampedTimeSeconds = std::clamp(timeSeconds, 0.0F, knots.back());
    evaluation.camera.position = EvaluateVector3(
        path,
        knots,
        clampedTimeSeconds,
        {ReadCameraX, ReadCameraY, ReadCameraZ});
    evaluation.focusPoint = EvaluateVector3(
        path,
        knots,
        clampedTimeSeconds,
        {ReadFocusX, ReadFocusY, ReadFocusZ});
    evaluation.camera.target = evaluation.focusPoint;
    evaluation.camera.orbitCenter = evaluation.focusPoint;
    evaluation.camera.hasOrbitCenter = true;

    const auto cameraPosition = ToGlm(evaluation.camera.position);
    const auto focusPoint = ToGlm(evaluation.focusPoint);
    evaluation.focusDistance = glm::length(focusPoint - cameraPosition);
    if (AnyKeyHasFocusDistance(path)) {
        evaluation.focusDistance = EvaluateScalar(path, knots, clampedTimeSeconds, ReadFocusDistance);
    }
    if (AnyKeyHasOrientation(path)) {
        WriteQuaternionToCameraState(EvaluateOrientation(path, knots, clampedTimeSeconds), &evaluation.camera);
    } else {
        WriteQuaternionToCameraState(LookAtOrientation(cameraPosition, focusPoint), &evaluation.camera);
    }

    evaluation.camera.fovDegrees = EvaluateScalar(path, knots, clampedTimeSeconds, ReadFovDegrees);
    evaluation.camera.nearPlane = EvaluateScalar(path, knots, clampedTimeSeconds, ReadNearPlane);
    evaluation.camera.farPlane = EvaluateScalar(path, knots, clampedTimeSeconds, ReadFarPlane);
    evaluation.camera.hasDepthOfField = path.depthOfFieldEnabled;
    evaluation.camera.focusDistance = evaluation.focusDistance;
    evaluation.camera.apertureFStops = AnyKeyHasApertureFStops(path)
                                           ? EvaluateScalar(path, knots, clampedTimeSeconds, ReadApertureFStops)
                                           : std::max(0.1F, path.apertureFStops);
    evaluation.camera.depthOfFieldMaxBlurPixels = std::max(0.0F, path.depthOfFieldMaxBlurPixels);
    return evaluation;
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

}  // namespace invisible_places::camera
