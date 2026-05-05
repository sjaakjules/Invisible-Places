#include "camera/CameraPath.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

#include <glm/geometric.hpp>
#include <glm/gtc/quaternion.hpp>

namespace invisible_places::camera {

namespace {

constexpr float kShotTimebaseFramesPerSecond = 30.0F;

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

std::vector<float> BuildScalarSamples(
    const std::vector<CameraShot>& shots,
    const CameraPathTiming& timing,
    float (*readValue)(const CameraState& state)) {
    std::vector<float> samples;
    samples.reserve(timing.knotSeconds.size());
    for (std::size_t index = timing.fromIndex; index <= timing.toIndex; ++index) {
        samples.push_back(readValue(shots[index].state));
    }
    return samples;
}

float ReadPositionX(const CameraState& state) {
    return state.position[0];
}

float ReadPositionY(const CameraState& state) {
    return state.position[1];
}

float ReadPositionZ(const CameraState& state) {
    return state.position[2];
}

float ReadTargetX(const CameraState& state) {
    return state.target[0];
}

float ReadTargetY(const CameraState& state) {
    return state.target[1];
}

float ReadTargetZ(const CameraState& state) {
    return state.target[2];
}

float ReadOrbitCenterX(const CameraState& state) {
    return state.hasOrbitCenter ? state.orbitCenter[0] : state.target[0];
}

float ReadOrbitCenterY(const CameraState& state) {
    return state.hasOrbitCenter ? state.orbitCenter[1] : state.target[1];
}

float ReadOrbitCenterZ(const CameraState& state) {
    return state.hasOrbitCenter ? state.orbitCenter[2] : state.target[2];
}

float ReadFovDegrees(const CameraState& state) {
    return state.fovDegrees;
}

float ReadNearPlane(const CameraState& state) {
    return state.nearPlane;
}

float ReadFarPlane(const CameraState& state) {
    return state.farPlane;
}

glm::vec3 PositionFromState(const CameraState& state) {
    return {state.position[0], state.position[1], state.position[2]};
}

glm::vec3 TargetFromState(const CameraState& state) {
    return {state.target[0], state.target[1], state.target[2]};
}

glm::vec3 OrbitCenterFromState(const CameraState& state) {
    return state.hasOrbitCenter
               ? glm::vec3{state.orbitCenter[0], state.orbitCenter[1], state.orbitCenter[2]}
               : TargetFromState(state);
}

float SegmentTimingWeight(const CameraShot& from, const CameraShot& to) {
    const float positionDistance =
        glm::length(PositionFromState(to.state) - PositionFromState(from.state));
    const float targetDistance =
        glm::length(TargetFromState(to.state) - TargetFromState(from.state));
    const float orbitDistance =
        glm::length(OrbitCenterFromState(to.state) - OrbitCenterFromState(from.state));

    const auto fromOrientation = QuaternionFromCameraState(from.state);
    const auto toOrientation = QuaternionFromCameraState(to.state);
    const float orientationDot = std::clamp(std::abs(glm::dot(fromOrientation, toOrientation)), 0.0F, 1.0F);
    const float rotationAngle = 2.0F * std::acos(orientationDot);

    return positionDistance + (0.25F * targetDistance) + (0.25F * orbitDistance) + rotationAngle;
}

std::vector<std::uint32_t> DistributeSegmentFrames(
    const std::vector<float>& weights,
    std::uint32_t totalDurationFrames) {
    if (weights.empty()) {
        return {};
    }

    const auto segmentCount = weights.size();
    const std::uint32_t clampedTotalFrames =
        std::max<std::uint32_t>(totalDurationFrames, static_cast<std::uint32_t>(segmentCount));
    std::vector<std::uint32_t> durations(segmentCount, 1U);
    std::uint32_t remainingFrames = clampedTotalFrames - static_cast<std::uint32_t>(segmentCount);
    if (remainingFrames == 0U) {
        return durations;
    }

    float totalWeight = 0.0F;
    for (const auto weight : weights) {
        totalWeight += std::max(0.0F, weight);
    }

    std::vector<float> remainders(segmentCount, 0.0F);
    if (totalWeight <= std::numeric_limits<float>::epsilon()) {
        const auto baseExtra = remainingFrames / static_cast<std::uint32_t>(segmentCount);
        const auto leftover = remainingFrames % static_cast<std::uint32_t>(segmentCount);
        for (std::size_t index = 0; index < segmentCount; ++index) {
            durations[index] += baseExtra + (index < leftover ? 1U : 0U);
        }
        return durations;
    }

    std::uint32_t assignedExtraFrames = 0U;
    for (std::size_t index = 0; index < segmentCount; ++index) {
        const float idealFrames =
            static_cast<float>(remainingFrames) * (std::max(0.0F, weights[index]) / totalWeight);
        const auto wholeFrames = static_cast<std::uint32_t>(std::floor(idealFrames));
        durations[index] += wholeFrames;
        assignedExtraFrames += wholeFrames;
        remainders[index] = idealFrames - static_cast<float>(wholeFrames);
    }

    while (assignedExtraFrames < remainingFrames) {
        const auto nextIt = std::max_element(remainders.begin(), remainders.end());
        if (nextIt == remainders.end()) {
            break;
        }
        const auto index = static_cast<std::size_t>(nextIt - remainders.begin());
        ++durations[index];
        *nextIt = -1.0F;
        ++assignedExtraFrames;
    }

    return durations;
}

std::array<float, 3> EvaluateVector3(
    const std::vector<CameraShot>& shots,
    const CameraPathTiming& timing,
    float timeSeconds,
    const std::array<float (*)(const CameraState& state), 3>& readers) {
    return {
        BuildNaturalCubicSpline(timing.knotSeconds, BuildScalarSamples(shots, timing, readers[0])).Evaluate(timeSeconds),
        BuildNaturalCubicSpline(timing.knotSeconds, BuildScalarSamples(shots, timing, readers[1])).Evaluate(timeSeconds),
        BuildNaturalCubicSpline(timing.knotSeconds, BuildScalarSamples(shots, timing, readers[2])).Evaluate(timeSeconds),
    };
}

std::vector<glm::quat> BuildAlignedQuaternions(
    const std::vector<CameraShot>& shots,
    const CameraPathTiming& timing) {
    std::vector<glm::quat> quaternions;
    quaternions.reserve(timing.knotSeconds.size());
    for (std::size_t index = timing.fromIndex; index <= timing.toIndex; ++index) {
        auto orientation = QuaternionFromCameraState(shots[index].state);
        if (!quaternions.empty() && glm::dot(quaternions.back(), orientation) < 0.0F) {
            orientation = -orientation;
        }
        quaternions.push_back(glm::normalize(orientation));
    }
    return quaternions;
}

glm::quat EvaluateOrientation(
    const std::vector<CameraShot>& shots,
    const CameraPathTiming& timing,
    float timeSeconds) {
    const auto quaternions = BuildAlignedQuaternions(shots, timing);
    if (quaternions.empty()) {
        return glm::quat{1.0F, 0.0F, 0.0F, 0.0F};
    }
    if (quaternions.size() == 1U) {
        return quaternions.front();
    }
    if (quaternions.size() == 2U) {
        const float durationSeconds = std::max(1.0e-6F, timing.DurationSeconds());
        const float t = std::clamp(timeSeconds / durationSeconds, 0.0F, 1.0F);
        return glm::normalize(glm::slerp(quaternions[0], quaternions[1], t));
    }

    std::vector<float> xValues;
    std::vector<float> yValues;
    std::vector<float> zValues;
    std::vector<float> wValues;
    xValues.reserve(quaternions.size());
    yValues.reserve(quaternions.size());
    zValues.reserve(quaternions.size());
    wValues.reserve(quaternions.size());
    for (const auto& orientation : quaternions) {
        xValues.push_back(orientation.x);
        yValues.push_back(orientation.y);
        zValues.push_back(orientation.z);
        wValues.push_back(orientation.w);
    }

    const glm::quat orientation{
        BuildNaturalCubicSpline(timing.knotSeconds, wValues).Evaluate(timeSeconds),
        BuildNaturalCubicSpline(timing.knotSeconds, xValues).Evaluate(timeSeconds),
        BuildNaturalCubicSpline(timing.knotSeconds, yValues).Evaluate(timeSeconds),
        BuildNaturalCubicSpline(timing.knotSeconds, zValues).Evaluate(timeSeconds),
    };
    const float lengthSquared =
        (orientation.w * orientation.w) +
        (orientation.x * orientation.x) +
        (orientation.y * orientation.y) +
        (orientation.z * orientation.z);
    if (lengthSquared <= 1.0e-12F) {
        const auto nearest = std::lower_bound(timing.knotSeconds.begin(), timing.knotSeconds.end(), timeSeconds);
        const auto nearestIndex = nearest == timing.knotSeconds.end()
                                      ? quaternions.size() - 1U
                                      : static_cast<std::size_t>(nearest - timing.knotSeconds.begin());
        return quaternions[nearestIndex];
    }
    const float inverseLength = 1.0F / std::sqrt(lengthSquared);
    return glm::quat{
        orientation.w * inverseLength,
        orientation.x * inverseLength,
        orientation.y * inverseLength,
        orientation.z * inverseLength,
    };
}

float EvaluateScalar(
    const std::vector<CameraShot>& shots,
    const CameraPathTiming& timing,
    float timeSeconds,
    float (*readValue)(const CameraState& state)) {
    return BuildNaturalCubicSpline(timing.knotSeconds, BuildScalarSamples(shots, timing, readValue)).Evaluate(timeSeconds);
}

CameraState FallbackCameraState(const std::vector<CameraShot>& shots, const CameraPathTiming& timing) {
    if (!shots.empty() && timing.fromIndex < shots.size()) {
        return shots[timing.fromIndex].state;
    }
    return {};
}

}  // namespace

CameraPathTiming BuildCameraPathTiming(
    const std::vector<CameraShot>& shots,
    std::size_t fromIndex,
    std::size_t toIndex) {
    CameraPathTiming timing;
    if (shots.size() < 2U) {
        return timing;
    }

    timing.fromIndex = std::min(fromIndex, shots.size() - 1U);
    timing.toIndex = std::min(toIndex, shots.size() - 1U);
    if (timing.fromIndex >= timing.toIndex) {
        timing.knotSeconds.clear();
        return timing;
    }

    timing.knotSeconds.reserve((timing.toIndex - timing.fromIndex) + 1U);
    timing.knotSeconds.push_back(0.0F);
    float elapsedSeconds = 0.0F;
    for (std::size_t shotIndex = timing.fromIndex + 1U; shotIndex <= timing.toIndex; ++shotIndex) {
        const auto durationFrames = std::max<std::uint32_t>(1U, shots[shotIndex].durationFrames);
        elapsedSeconds += static_cast<float>(durationFrames) / kShotTimebaseFramesPerSecond;
        timing.knotSeconds.push_back(elapsedSeconds);
    }
    return timing;
}

std::vector<CameraShot> BuildWeightedCameraPathShots(
    const std::vector<CameraShot>& orderedShots,
    std::uint32_t totalDurationFrames) {
    if (orderedShots.size() < 2U) {
        return orderedShots;
    }

    std::vector<CameraShot> weightedShots = orderedShots;
    std::vector<float> weights;
    weights.reserve(weightedShots.size() - 1U);
    for (std::size_t index = 1; index < weightedShots.size(); ++index) {
        weights.push_back(SegmentTimingWeight(weightedShots[index - 1U], weightedShots[index]));
    }

    const auto segmentDurations = DistributeSegmentFrames(weights, totalDurationFrames);
    for (std::size_t segmentIndex = 0; segmentIndex < segmentDurations.size(); ++segmentIndex) {
        weightedShots[segmentIndex + 1U].durationFrames = segmentDurations[segmentIndex];
    }
    return weightedShots;
}

CameraState EvaluateCameraPath(
    const std::vector<CameraShot>& shots,
    const CameraPathTiming& timing,
    float timeSeconds) {
    if (!timing.IsValid() || timing.toIndex >= shots.size()) {
        return FallbackCameraState(shots, timing);
    }

    const float clampedTimeSeconds = std::clamp(timeSeconds, 0.0F, timing.DurationSeconds());
    CameraState state;
    state.position = EvaluateVector3(
        shots,
        timing,
        clampedTimeSeconds,
        {ReadPositionX, ReadPositionY, ReadPositionZ});
    state.target = EvaluateVector3(
        shots,
        timing,
        clampedTimeSeconds,
        {ReadTargetX, ReadTargetY, ReadTargetZ});
    state.orbitCenter = EvaluateVector3(
        shots,
        timing,
        clampedTimeSeconds,
        {ReadOrbitCenterX, ReadOrbitCenterY, ReadOrbitCenterZ});
    state.hasOrbitCenter = true;
    WriteQuaternionToCameraState(EvaluateOrientation(shots, timing, clampedTimeSeconds), &state);
    state.fovDegrees = EvaluateScalar(shots, timing, clampedTimeSeconds, ReadFovDegrees);
    state.nearPlane = EvaluateScalar(shots, timing, clampedTimeSeconds, ReadNearPlane);
    state.farPlane = EvaluateScalar(shots, timing, clampedTimeSeconds, ReadFarPlane);
    return state;
}

}  // namespace invisible_places::camera
