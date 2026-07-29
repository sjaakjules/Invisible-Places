#pragma once

#include <array>
#include <cstdint>

namespace invisible_places::water {

inline constexpr std::uint32_t kWaterSeepagePulseFieldCapacity = 128U;
inline constexpr std::uint32_t kWaterSeepagePulseMaximumWaveCount = 12U;

// This quality is deliberately independent of WaterSeepageQuality.  The
// pulse field is a compact longitudinal lookup table, so its quality controls
// sample density only; surface-noise quality remains a renderer concern.
enum class WaterSeepagePulseFieldQuality : std::uint32_t {
    Low,
    Balanced,
    High,
};

struct WaterSeepagePulseFieldSettings {
    float spacingMeters = 0.18F;
    float widthMeters = 0.055F;
    float speedMetersPerSecond = 0.12F;
    float irregularity = 0.38F;
    float evolution = 0.06F;
    // Animation interpolation may make this fractional.  Whole waves remain
    // stable and only the final wave fades in or out.
    float waveCount = 7.0F;
    float speedVariation = 0.55F;
    std::uint32_t seed = 0U;
    std::uint32_t nodeId = 0U;
    WaterSeepagePulseFieldQuality quality =
        WaterSeepagePulseFieldQuality::Balanced;
    // Invariant: callers derive this from the node's immutable maximum
    // support extent, never its live Strength/effective reach.  The builder
    // intentionally has no Strength input, so animation cannot change wave
    // launch phases by changing their cycle distance.
    float stableSpanMeters = 1.0F;
    float timeSeconds = 0.0F;
};

struct WaterSeepagePulseField {
    // The storage capacity never changes.  Only the first sampleCount entries
    // are live; all remaining entries are guaranteed to be zero.
    std::array<float, kWaterSeepagePulseFieldCapacity> samples{};
    std::uint32_t sampleCount = 0U;
    float stableSpanMeters = 0.0F;

    // Samples the inclusive [0, stableSpanMeters] domain.  Non-finite input,
    // an empty field, or an invalid span returns zero; finite out-of-range
    // distances clamp to the nearest endpoint without indexing past storage.
    [[nodiscard]] float Sample(float downstreamMeters) const noexcept;
};

[[nodiscard]] constexpr std::uint32_t WaterSeepagePulseFieldSampleCount(
    WaterSeepagePulseFieldQuality quality) noexcept {
    switch (quality) {
        case WaterSeepagePulseFieldQuality::Low:
            return 32U;
        case WaterSeepagePulseFieldQuality::High:
            return 128U;
        case WaterSeepagePulseFieldQuality::Balanced:
        default:
            return 64U;
    }
}

[[nodiscard]] WaterSeepagePulseField BuildWaterSeepagePulseField(
    const WaterSeepagePulseFieldSettings& settings) noexcept;

[[nodiscard]] float SampleWaterSeepagePulseField(
    const WaterSeepagePulseField& field,
    float downstreamMeters) noexcept;

}  // namespace invisible_places::water
