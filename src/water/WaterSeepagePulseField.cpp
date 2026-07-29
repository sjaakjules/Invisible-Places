#include "water/WaterSeepagePulseField.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace invisible_places::water {
namespace {

constexpr float kMinimumSpacingMeters = 0.005F;
constexpr float kMaximumSpacingMeters = 20.0F;
constexpr float kMinimumWidthMeters = 0.001F;
constexpr float kMaximumWidthMeters = 10.0F;
constexpr float kMaximumSpeedMetersPerSecond = 4.0F;
constexpr float kMaximumEvolution = 4.0F;
constexpr float kMinimumStableSpanMeters = 0.005F;
constexpr float kMaximumStableSpanMeters = 1000.0F;
constexpr float kTwoPi = 6.28318530717958647692F;

[[nodiscard]] float FiniteOr(float value, float fallback) noexcept {
    return std::isfinite(value) ? value : fallback;
}

[[nodiscard]] float Clamp01(float value) noexcept {
    return std::clamp(value, 0.0F, 1.0F);
}

[[nodiscard]] float SmoothStep(
    float edge0,
    float edge1,
    float value) noexcept {
    const float amount = Clamp01(
        (value - edge0) / std::max(1.0e-6F, edge1 - edge0));
    return amount * amount * (3.0F - 2.0F * amount);
}

// Keep this bit-for-bit integer recipe aligned with SeepageSpatialHash and
// SeepageHash01 in WaterFlow.cpp.  The field can therefore be prepared by a
// CPU/offline path without assigning a second random identity to each wave.
[[nodiscard]] std::uint32_t SpatialHash(
    std::int32_t x,
    std::int32_t y,
    std::int32_t z) noexcept {
    std::uint32_t hash =
        (static_cast<std::uint32_t>(x) * 0x8da6b343U) ^
        (static_cast<std::uint32_t>(y) * 0xd8163841U) ^
        (static_cast<std::uint32_t>(z) * 0xcb1ab31fU);
    hash ^= hash >> 16U;
    hash *= 0x7feb352dU;
    hash ^= hash >> 15U;
    hash *= 0x846ca68bU;
    hash ^= hash >> 16U;
    return hash;
}

[[nodiscard]] float Hash01(
    std::int32_t x,
    std::int32_t y,
    std::uint32_t seed) noexcept {
    std::uint32_t hash =
        SpatialHash(x, y, static_cast<std::int32_t>(seed));
    hash ^= seed * 0x9e3779b9U;
    hash ^= hash >> 16U;
    hash *= 0x7feb352dU;
    hash ^= hash >> 15U;
    return static_cast<float>(hash & 0x00ffffffU) /
           static_cast<float>(0x01000000U);
}

// This intentionally remains a broad, graduated wave rather than a thin
// line.  It is the longitudinal equivalent of SeepageContourWave.
[[nodiscard]] float ContourWave(
    float distanceMeters,
    float widthMeters) noexcept {
    const float width = std::max(kMinimumWidthMeters, widthMeters);
    return Clamp01(
        1.0F -
        SmoothStep(
            width * 0.15F,
            width * 2.40F,
            std::abs(distanceMeters)));
}

struct ResolvedPulseSettings {
    float spacingMeters = 0.18F;
    float widthMeters = 0.055F;
    float speedMetersPerSecond = 0.12F;
    float irregularity = 0.38F;
    float evolution = 0.06F;
    float waveCount = 7.0F;
    float speedVariation = 0.55F;
    float stableSpanMeters = 1.0F;
    float timeSeconds = 0.0F;
    std::uint32_t proceduralSeed = 0U;
    std::uint32_t sampleCount = 64U;
};

[[nodiscard]] ResolvedPulseSettings ResolveSettings(
    const WaterSeepagePulseFieldSettings& settings) noexcept {
    ResolvedPulseSettings resolved;
    resolved.spacingMeters = std::clamp(
        FiniteOr(settings.spacingMeters, resolved.spacingMeters),
        kMinimumSpacingMeters,
        kMaximumSpacingMeters);
    resolved.widthMeters = std::clamp(
        FiniteOr(settings.widthMeters, resolved.widthMeters),
        kMinimumWidthMeters,
        std::min(kMaximumWidthMeters, resolved.spacingMeters * 0.49F));
    resolved.speedMetersPerSecond = std::clamp(
        FiniteOr(
            settings.speedMetersPerSecond,
            resolved.speedMetersPerSecond),
        0.0F,
        kMaximumSpeedMetersPerSecond);
    resolved.irregularity =
        Clamp01(FiniteOr(settings.irregularity, resolved.irregularity));
    resolved.evolution = std::clamp(
        FiniteOr(settings.evolution, resolved.evolution),
        0.0F,
        kMaximumEvolution);
    resolved.waveCount = std::clamp(
        FiniteOr(settings.waveCount, resolved.waveCount),
        1.0F,
        static_cast<float>(kWaterSeepagePulseMaximumWaveCount));
    resolved.speedVariation =
        Clamp01(FiniteOr(settings.speedVariation, resolved.speedVariation));
    resolved.stableSpanMeters = std::clamp(
        FiniteOr(settings.stableSpanMeters, resolved.stableSpanMeters),
        kMinimumStableSpanMeters,
        kMaximumStableSpanMeters);
    resolved.timeSeconds =
        std::max(0.0F, FiniteOr(settings.timeSeconds, resolved.timeSeconds));
    resolved.proceduralSeed =
        settings.seed ^ (settings.nodeId * 0x9e3779b9U);
    resolved.sampleCount =
        WaterSeepagePulseFieldSampleCount(settings.quality);
    return resolved;
}

[[nodiscard]] float PositiveModulo(
    float timeSeconds,
    float speedMetersPerSecond,
    float phaseDistanceMeters,
    float cycleDistanceMeters) noexcept {
    // Do the potentially large time multiplication in double precision so a
    // finite long-running timeline cannot overflow the field to NaN.
    const double cycle = static_cast<double>(cycleDistanceMeters);
    const double travelled =
        static_cast<double>(timeSeconds) *
            static_cast<double>(speedMetersPerSecond) +
        static_cast<double>(phaseDistanceMeters);
    double wrapped = std::fmod(travelled, cycle);
    if (wrapped < 0.0) {
        wrapped += cycle;
    }
    return static_cast<float>(wrapped);
}

[[nodiscard]] float EvaluatePulse(
    const ResolvedPulseSettings& settings,
    float downstreamMeters) noexcept {
    const auto fullWaveCount = static_cast<std::uint32_t>(
        std::floor(settings.waveCount));
    const float finalWaveWeight =
        settings.waveCount - static_cast<float>(fullWaveCount);
    const std::uint32_t evaluatedWaveCount = std::min(
        kWaterSeepagePulseMaximumWaveCount,
        fullWaveCount + (finalWaveWeight > 0.0F ? 1U : 0U));

    float accumulatedWaves = 0.0F;
    for (std::uint32_t waveIndex = 0U;
         waveIndex < evaluatedWaveCount;
         ++waveIndex) {
        const auto index = static_cast<std::int32_t>(waveIndex);
        const float speedHash =
            Hash01(index, 11, settings.proceduralSeed + 19009U);
        const float startHash =
            Hash01(index, 23, settings.proceduralSeed + 27143U);
        const float widthHash =
            Hash01(index, 37, settings.proceduralSeed + 31847U);
        const float shapeHash =
            Hash01(index, 53, settings.proceduralSeed + 45119U);
        const float amplitudeHash =
            Hash01(index, 71, settings.proceduralSeed + 57203U);

        const float waveSpeed =
            settings.speedMetersPerSecond *
            std::max(
                0.15F,
                1.0F +
                    (speedHash * 2.0F - 1.0F) *
                        settings.speedVariation);
        const float idleGap = std::max(
            settings.spacingMeters * (0.45F + shapeHash * 0.55F),
            settings.widthMeters * 4.0F);
        const float cycleDistance = settings.stableSpanMeters + idleGap;

        // Wave zero is the reference launch: at animation time zero it sits
        // on the source so selecting Contour Pulses gives immediate visual
        // feedback even when the current Strength reveals only a short run.
        // The remaining waves keep stable random launch phases. Every wave
        // still has its own idle interval and speed, allowing genuine
        // catch-up overlap after the reference front moves downhill.
        const float launchPhaseDistance =
            waveIndex == 0U
                ? idleGap
                : startHash * cycleDistance;
        const float travel =
            PositiveModulo(
                settings.timeSeconds,
                waveSpeed,
                launchPhaseDistance,
                cycleDistance) -
            idleGap;
        const float waveWidth =
            settings.widthMeters * (0.72F + widthHash * 0.78F);

        // This is the centre-line component of the existing front-shape
        // formula.  Stationary cross-surface bow/noise remains outside this
        // one-dimensional field, while seeded slow flex stays coherent.
        const double shapePhase = std::fmod(
            static_cast<double>(settings.timeSeconds) *
                    static_cast<double>(settings.evolution) *
                    static_cast<double>(0.15F + widthHash * 0.25F) +
                static_cast<double>(shapeHash * kTwoPi),
            static_cast<double>(kTwoPi));
        const float slowShapeChange =
            static_cast<float>(std::sin(shapePhase)) *
            settings.widthMeters * settings.irregularity * 0.55F;
        const float distanceToWave =
            downstreamMeters + slowShapeChange - travel;
        const float amplitude = 0.58F + amplitudeHash * 0.42F;
        const float waveWeight =
            waveIndex < fullWaveCount ? 1.0F : finalWaveWeight;
        accumulatedWaves +=
            ContourWave(distanceToWave, waveWidth) *
            amplitude *
            waveWeight;
    }

    // A broad isolated wave stays subtle; additive catch-up overlap ramps
    // much faster into a local high-intensity region.
    return Clamp01(
        accumulatedWaves * 0.22F +
        std::max(0.0F, accumulatedWaves - 0.90F) * 0.42F);
}

}  // namespace

WaterSeepagePulseField BuildWaterSeepagePulseField(
    const WaterSeepagePulseFieldSettings& settings) noexcept {
    const ResolvedPulseSettings resolved = ResolveSettings(settings);
    WaterSeepagePulseField field;
    field.sampleCount = resolved.sampleCount;
    field.stableSpanMeters = resolved.stableSpanMeters;

    const float intervalCount =
        static_cast<float>(field.sampleCount - 1U);
    for (std::uint32_t sampleIndex = 0U;
         sampleIndex < field.sampleCount;
         ++sampleIndex) {
        const float downstreamMeters =
            field.stableSpanMeters *
            (static_cast<float>(sampleIndex) / intervalCount);
        field.samples[sampleIndex] =
            EvaluatePulse(resolved, downstreamMeters);
    }
    return field;
}

float SampleWaterSeepagePulseField(
    const WaterSeepagePulseField& field,
    float downstreamMeters) noexcept {
    const std::uint32_t count = std::min(
        field.sampleCount,
        kWaterSeepagePulseFieldCapacity);
    if (count == 0U ||
        !std::isfinite(field.stableSpanMeters) ||
        field.stableSpanMeters <= 0.0F ||
        !std::isfinite(downstreamMeters)) {
        return 0.0F;
    }
    if (count == 1U) {
        return std::isfinite(field.samples[0]) ? field.samples[0] : 0.0F;
    }

    const float unitDistance = std::clamp(
        downstreamMeters / field.stableSpanMeters,
        0.0F,
        1.0F);
    const float samplePosition =
        unitDistance * static_cast<float>(count - 1U);
    const auto leftIndex =
        static_cast<std::uint32_t>(std::floor(samplePosition));
    const auto rightIndex = std::min(leftIndex + 1U, count - 1U);
    const float amount =
        samplePosition - static_cast<float>(leftIndex);
    const float left = std::isfinite(field.samples[leftIndex])
                           ? field.samples[leftIndex]
                           : 0.0F;
    const float right = std::isfinite(field.samples[rightIndex])
                            ? field.samples[rightIndex]
                            : 0.0F;
    return Clamp01(left + (right - left) * amount);
}

float WaterSeepagePulseField::Sample(float downstreamMeters) const noexcept {
    return SampleWaterSeepagePulseField(*this, downstreamMeters);
}

}  // namespace invisible_places::water
