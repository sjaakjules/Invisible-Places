#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace invisible_places::app::point_density_parity {

// Non-owning view over one linear render. Every channel must contain exactly
// width * height samples. Depth samples that are non-finite or not greater
// than Options::minimumValidDepth are excluded from depth comparisons.
struct LinearFrameView {
    std::uint32_t width = 0U;
    std::uint32_t height = 0U;
    std::span<const float> red;
    std::span<const float> green;
    std::span<const float> blue;
    std::span<const float> alpha;
    std::span<const float> depth;
};

struct Options {
    // Pixels at or above this alpha are foreground support.
    float foregroundAlphaThreshold = 0.01F;
    // Missing/excess support uses a square (Chebyshev) neighbourhood.
    std::uint32_t supportTolerancePixels = 1U;
    std::uint32_t tileColumns = 32U;
    std::uint32_t tileRows = 18U;
    float rgbBlurSigmaPixels = 1.0F;
    float opticalDepthEpsilon = 1.0e-6F;
    float minimumValidDepth = 0.0F;
};

struct FrameStatistics {
    std::size_t foregroundPixelCount = 0U;
    double foregroundCoverage = 0.0;
    double opticalDepthSum = 0.0;
};

struct SupportStatistics {
    std::size_t missingReferencePixelCount = 0U;
    std::size_t excessCandidatePixelCount = 0U;
    double missingFractionOfReference = 0.0;
    double excessFractionOfCandidate = 0.0;
};

struct TileCoverageDeltaStatistics {
    std::uint32_t columns = 0U;
    std::uint32_t rows = 0U;
    std::size_t sampleCount = 0U;
    double p50AbsoluteDelta = 0.0;
    double p95AbsoluteDelta = 0.0;
    double maximumAbsoluteDelta = 0.0;
};

struct RgbErrorStatistics {
    std::size_t sampleCount = 0U;
    double meanAbsoluteError = 0.0;
    double rootMeanSquareError = 0.0;
};

struct DepthErrorStatistics {
    std::size_t sampleCount = 0U;
    double medianAbsoluteError = 0.0;
    double p95AbsoluteError = 0.0;
};

struct Report {
    std::uint32_t width = 0U;
    std::uint32_t height = 0U;
    Options options;
    FrameStatistics reference;
    FrameStatistics candidate;
    double candidateToReferenceCoverageRatio = 1.0;
    double candidateToReferenceOpticalDepthRatio = 1.0;
    SupportStatistics support;
    TileCoverageDeltaStatistics tileCoverage;
    RgbErrorStatistics blurredLinearRgb;
    DepthErrorStatistics mutualSupportDepth;
};

struct DiagnosticImages {
    std::uint32_t width = 0U;
    std::uint32_t height = 0U;

    // Interleaved linear RGBA. RGB is the unblurred absolute colour
    // difference and A is absolute alpha difference. Values are finite but
    // deliberately not clamped, preserving HDR differences for the caller.
    std::vector<float> absoluteDifferenceRgba;

    // Interleaved RGBA8 support classification:
    // transparent = background, green = exact mutual support,
    // yellow = reference support matched only within tolerance,
    // cyan = candidate support matched only within tolerance,
    // red = missing reference support, blue = excess candidate support.
    std::vector<std::uint8_t> supportRgba;
};

struct ComparisonResult {
    bool success = false;
    std::string errorMessage;
    Report report;
    DiagnosticImages diagnostics;
};

// The candidate is the render being evaluated (for example, a compensated
// 5 mm source) against a frozen high-density reference (for example, the
// equivalent 1 mm source).
// Ratios are candidate/reference. A 0/0 ratio is reported as 1; a positive
// candidate divided by zero is reported as positive infinity.
[[nodiscard]] ComparisonResult Compare(
    const LinearFrameView& reference,
    const LinearFrameView& candidate,
    const Options& options = {});

}  // namespace invisible_places::app::point_density_parity
