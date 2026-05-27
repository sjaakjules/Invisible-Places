#include "renderer/pointcloud/PointCloudPerformanceGovernor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace invisible_places::renderer::pointcloud {

namespace {

constexpr float kDefaultBudgetScale = 1.0F;
constexpr double kEwmaAlpha = 0.18;
constexpr float kMaxScaleDownPerUpdate = 0.80F;
constexpr float kMaxScaleUpPerUpdate = 1.15F;
constexpr std::uint64_t kMinimumUploadBytesPerFrame = 16ULL * 1024ULL * 1024ULL;

[[nodiscard]] bool ValidGpuTiming(double milliseconds) {
    return std::isfinite(milliseconds) && milliseconds > 0.0;
}

[[nodiscard]] float ClampBudgetScale(float scale, float minimumScale, float maximumScale) {
    const float minimum = std::clamp(minimumScale, 0.05F, 1.0F);
    const float maximum = std::max(minimum, std::clamp(maximumScale, 1.0F, 4.0F));
    return std::clamp(scale, minimum, maximum);
}

[[nodiscard]] PointCloudPerformanceGovernorOutput MakeOutput(
    const PointCloudPerformanceGovernor& governor,
    const PointCloudPerformanceGovernorSample& sample) {
    PointCloudPerformanceGovernorOutput output;
    output.timestampSupported = sample.timestampSupported;
    output.timingValid = sample.timingValid && ValidGpuTiming(sample.gpuPointPassMs);
    output.sceneRendered = sample.sceneRendered;
    output.budgetScale = ClampBudgetScale(
        governor.budgetScale,
        sample.minimumBudgetScale,
        sample.maximumBudgetScale);
    output.targetSpacingScale = std::sqrt(1.0F / std::max(0.05F, output.budgetScale));
    output.gpuPointPassMs = output.timingValid ? sample.gpuPointPassMs : 0.0;
    output.gpuPointPassEwmaMs = governor.ewmaInitialized ? governor.gpuPointPassEwmaMs : 0.0;
    output.gpuCompositeMs =
        output.timingValid && ValidGpuTiming(sample.gpuCompositeMs) ? sample.gpuCompositeMs : 0.0;
    output.gpuCompositeEwmaMs = governor.ewmaInitialized ? governor.gpuCompositeEwmaMs : 0.0;
    output.targetPointPassMs = std::max(0.1, sample.targetPointPassMs);
    output.targetCompositeMs = std::max(0.1, sample.targetCompositeMs);
    output.maxUploadBytesPerFrame = std::max<std::uint64_t>(
        kMinimumUploadBytesPerFrame,
        static_cast<std::uint64_t>(
            std::llround(static_cast<double>(std::max<std::uint64_t>(1ULL, sample.baseUploadBytesPerFrame)) *
                         static_cast<double>(output.budgetScale))));

    if (!sample.timestampSupported) {
        output.timestampState = "unavailable";
        output.status = "timestamp unavailable; conservative fixed budget";
        output.activeLimit = "conservative fixed fallback";
        output.visuallyLossless = true;
        return output;
    }
    if (!sample.sceneRendered) {
        output.timestampState = "cached frame";
        output.status = "cached frame; governor held";
        output.activeLimit = "previous measured budget";
        output.visuallyLossless = true;
        return output;
    }
    if (!output.timingValid) {
        output.timestampState = "invalid";
        output.status = "timestamp invalid; governor held";
        output.activeLimit = "previous measured budget";
        output.visuallyLossless = true;
        return output;
    }

    output.timestampState = "valid";
    const double ratio = output.gpuPointPassEwmaMs > 0.0
                             ? output.gpuPointPassEwmaMs / output.targetPointPassMs
                             : output.gpuPointPassMs / output.targetPointPassMs;
    output.overTarget = ratio > 1.12;
    output.underTarget = ratio < 0.72 && output.budgetScale >= 0.98F;
    output.performanceLimited =
        output.overTarget &&
        output.budgetScale <= sample.minimumBudgetScale + 0.01F &&
        (sample.representativeBudgetReached || sample.fragmentBudgetReached || sample.blendedFragmentBudgetReached);
    output.visuallyLossless = !output.performanceLimited && output.budgetScale >= 0.98F && ratio <= 1.05;
    if (output.performanceLimited) {
        if (sample.blendedFragmentBudgetReached) {
            output.activeLimit = "blended fragments";
        } else if (sample.fragmentBudgetReached) {
            output.activeLimit = "fragments";
        } else {
            output.activeLimit = "vertices";
        }
        output.status = "performance-limited";
    } else if (output.budgetScale < 0.98F) {
        output.activeLimit = sample.blendedFragmentBudgetReached
                                 ? "blended fragments"
                                 : (sample.fragmentBudgetReached
                                        ? "fragments"
                                        : (sample.representativeBudgetReached ? "vertices" : "point-pass time"));
        output.status = "measured adaptive budget";
    } else {
        output.activeLimit = "visual budget";
        output.status = "visually lossless";
    }
    return output;
}

}  // namespace

std::size_t PointCloudLodRendererCostProfileIndex(PointCloudLodRendererCostProfile profile) {
    const auto index = static_cast<std::size_t>(profile);
    return index < kPointCloudLodRendererCostProfileCount ? index : 0U;
}

double DefaultPointCloudGovernorTargetPointPassMs(
    PointCloudLodRendererCostProfile profile,
    bool interactive) {
    switch (profile) {
        case PointCloudLodRendererCostProfile::FastBasicSquare:
            return interactive ? 8.0 : 12.0;
        case PointCloudLodRendererCostProfile::BeautyScreenSprite:
        case PointCloudLodRendererCostProfile::BeautyWorldScreenSprite:
            return interactive ? 11.0 : 16.0;
        case PointCloudLodRendererCostProfile::BeautyWorldSurfel:
            return interactive ? 12.5 : 18.0;
    }
    return interactive ? 10.0 : 16.0;
}

double DefaultPointCloudGovernorTargetCompositeMs(bool interactive) {
    return interactive ? 3.0 : 5.0;
}

std::uint64_t DefaultPointCloudGovernorUploadBudgetBytes(PointCloudLodRendererCostProfile profile) {
    switch (profile) {
        case PointCloudLodRendererCostProfile::FastBasicSquare:
            return 128ULL * 1024ULL * 1024ULL;
        case PointCloudLodRendererCostProfile::BeautyScreenSprite:
        case PointCloudLodRendererCostProfile::BeautyWorldScreenSprite:
        case PointCloudLodRendererCostProfile::BeautyWorldSurfel:
            return 96ULL * 1024ULL * 1024ULL;
    }
    return 96ULL * 1024ULL * 1024ULL;
}

PointCloudPerformanceGovernorOutput UpdatePointCloudPerformanceGovernor(
    PointCloudPerformanceGovernor* governor,
    const PointCloudPerformanceGovernorSample& sample) {
    if (governor == nullptr) {
        PointCloudPerformanceGovernor fallback;
        return MakeOutput(fallback, sample);
    }

    governor->timestampSupported = sample.timestampSupported;
    governor->lastTimingValid = false;

    if (!sample.timestampSupported) {
        governor->budgetScale = kDefaultBudgetScale;
        governor->ewmaInitialized = false;
        return MakeOutput(*governor, sample);
    }

    const bool validSample =
        sample.sceneRendered &&
        sample.timingValid &&
        ValidGpuTiming(sample.gpuPointPassMs);
    if (!validSample) {
        ++governor->invalidSampleCount;
        return MakeOutput(*governor, sample);
    }

    const double compositeMs = ValidGpuTiming(sample.gpuCompositeMs) ? sample.gpuCompositeMs : 0.0;
    if (!governor->ewmaInitialized) {
        governor->gpuPointPassEwmaMs = sample.gpuPointPassMs;
        governor->gpuCompositeEwmaMs = compositeMs;
        governor->ewmaInitialized = true;
    } else {
        governor->gpuPointPassEwmaMs =
            kEwmaAlpha * sample.gpuPointPassMs + (1.0 - kEwmaAlpha) * governor->gpuPointPassEwmaMs;
        governor->gpuCompositeEwmaMs =
            kEwmaAlpha * compositeMs + (1.0 - kEwmaAlpha) * governor->gpuCompositeEwmaMs;
    }
    ++governor->validSampleCount;
    governor->lastTimingValid = true;

    const double targetPointPassMs = std::max(0.1, sample.targetPointPassMs);
    const double ratio = governor->gpuPointPassEwmaMs / targetPointPassMs;
    float desiredScale = governor->budgetScale;
    if (ratio > 1.0) {
        desiredScale *= static_cast<float>(std::pow(1.0 / ratio, 0.70));
    } else {
        desiredScale *= static_cast<float>(1.0 + std::min(0.15, (1.0 - ratio) * 0.20));
    }
    desiredScale = ClampBudgetScale(desiredScale, sample.minimumBudgetScale, sample.maximumBudgetScale);
    desiredScale = std::clamp(
        desiredScale,
        governor->budgetScale * kMaxScaleDownPerUpdate,
        governor->budgetScale * kMaxScaleUpPerUpdate);
    governor->budgetScale = ClampBudgetScale(desiredScale, sample.minimumBudgetScale, sample.maximumBudgetScale);

    return MakeOutput(*governor, sample);
}

PointCloudPerformanceGovernorOutput CurrentPointCloudPerformanceGovernorOutput(
    const PointCloudPerformanceGovernor& governor,
    const PointCloudPerformanceGovernorSample& sample) {
    return MakeOutput(governor, sample);
}

}  // namespace invisible_places::renderer::pointcloud
