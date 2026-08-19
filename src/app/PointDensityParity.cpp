#include "app/PointDensityParity.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <utility>

namespace invisible_places::app::point_density_parity {

namespace {

constexpr std::array<std::uint8_t, 4U> kMutualSupport{64U, 200U, 96U, 255U};
constexpr std::array<std::uint8_t, 4U> kToleratedReference{255U, 190U, 0U, 255U};
constexpr std::array<std::uint8_t, 4U> kToleratedCandidate{0U, 190U, 255U, 255U};
constexpr std::array<std::uint8_t, 4U> kMissingReference{255U, 48U, 48U, 255U};
constexpr std::array<std::uint8_t, 4U> kExcessCandidate{48U, 112U, 255U, 255U};

[[nodiscard]] float FiniteOrZero(float value) {
    return std::isfinite(value) ? value : 0.0F;
}

[[nodiscard]] float ClampedAlpha(float value) {
    return std::clamp(FiniteOrZero(value), 0.0F, 1.0F);
}

[[nodiscard]] bool ValidateFrame(
    const LinearFrameView& frame,
    std::size_t expectedPixelCount,
    const char* label,
    std::string* errorMessage) {
    const auto validChannel = [expectedPixelCount](std::span<const float> channel) {
        return channel.size() == expectedPixelCount;
    };
    if (validChannel(frame.red) && validChannel(frame.green) &&
        validChannel(frame.blue) && validChannel(frame.alpha) &&
        validChannel(frame.depth)) {
        return true;
    }

    if (errorMessage != nullptr) {
        *errorMessage = std::string{label} +
                        " frame channels must each contain width * height samples";
    }
    return false;
}

[[nodiscard]] double SafeRatio(double numerator, double denominator) {
    if (denominator != 0.0) {
        return numerator / denominator;
    }
    return numerator == 0.0 ? 1.0 : std::numeric_limits<double>::infinity();
}

[[nodiscard]] double SafeFraction(double numerator, double denominator) {
    return denominator == 0.0 ? 0.0 : numerator / denominator;
}

[[nodiscard]] double NearestRankPercentile(
    std::vector<double> values,
    double percentile) {
    if (values.empty()) {
        return 0.0;
    }
    std::sort(values.begin(), values.end());
    const double rank = std::ceil(percentile * static_cast<double>(values.size()));
    const auto index = static_cast<std::size_t>(std::max(1.0, rank)) - 1U;
    return values[std::min(index, values.size() - 1U)];
}

[[nodiscard]] double Median(std::vector<double> values) {
    if (values.empty()) {
        return 0.0;
    }
    std::sort(values.begin(), values.end());
    const auto middle = values.size() / 2U;
    if ((values.size() % 2U) != 0U) {
        return values[middle];
    }
    return (values[middle - 1U] + values[middle]) * 0.5;
}

[[nodiscard]] bool HasSupportWithinTolerance(
    std::span<const std::uint8_t> support,
    std::uint32_t width,
    std::uint32_t height,
    std::uint32_t x,
    std::uint32_t y,
    std::uint32_t tolerance) {
    const auto minX = x > tolerance ? x - tolerance : 0U;
    const auto minY = y > tolerance ? y - tolerance : 0U;
    const auto maxX = std::min(width - 1U, x + std::min(tolerance, width - 1U - x));
    const auto maxY = std::min(height - 1U, y + std::min(tolerance, height - 1U - y));
    for (std::uint32_t candidateY = minY; candidateY <= maxY; ++candidateY) {
        for (std::uint32_t candidateX = minX; candidateX <= maxX; ++candidateX) {
            const auto index = static_cast<std::size_t>(candidateY) * width + candidateX;
            if (support[index] != 0U) {
                return true;
            }
        }
    }
    return false;
}

void SetDiagnosticPixel(
    std::vector<std::uint8_t>* rgba,
    std::size_t pixelIndex,
    const std::array<std::uint8_t, 4U>& color) {
    if (rgba == nullptr) {
        return;
    }
    const auto offset = pixelIndex * 4U;
    std::copy(color.begin(), color.end(), rgba->begin() + static_cast<std::ptrdiff_t>(offset));
}

[[nodiscard]] std::vector<float> MakeGaussianKernel(float sigma) {
    if (sigma <= 0.0F) {
        return {1.0F};
    }

    const auto radius = static_cast<int>(std::ceil(3.0F * sigma));
    std::vector<float> kernel(static_cast<std::size_t>(radius * 2 + 1));
    double sum = 0.0;
    for (int offset = -radius; offset <= radius; ++offset) {
        const auto normalizedOffset = static_cast<double>(offset) / sigma;
        const auto value = std::exp(-0.5 * normalizedOffset * normalizedOffset);
        kernel[static_cast<std::size_t>(offset + radius)] = static_cast<float>(value);
        sum += value;
    }
    for (auto& value : kernel) {
        value = static_cast<float>(static_cast<double>(value) / sum);
    }
    return kernel;
}

[[nodiscard]] std::vector<float> GaussianBlur(
    std::span<const float> source,
    std::uint32_t width,
    std::uint32_t height,
    std::span<const float> kernel) {
    const auto pixelCount = static_cast<std::size_t>(width) * height;
    std::vector<float> horizontal(pixelCount, 0.0F);
    std::vector<float> output(pixelCount, 0.0F);
    const auto radius = static_cast<int>(kernel.size() / 2U);

    for (std::uint32_t y = 0U; y < height; ++y) {
        for (std::uint32_t x = 0U; x < width; ++x) {
            double sum = 0.0;
            for (int offset = -radius; offset <= radius; ++offset) {
                const auto sampleX = static_cast<std::uint32_t>(std::clamp(
                    static_cast<int>(x) + offset,
                    0,
                    static_cast<int>(width) - 1));
                const auto index = static_cast<std::size_t>(y) * width + sampleX;
                sum += static_cast<double>(FiniteOrZero(source[index])) *
                       kernel[static_cast<std::size_t>(offset + radius)];
            }
            horizontal[static_cast<std::size_t>(y) * width + x] = static_cast<float>(sum);
        }
    }

    for (std::uint32_t y = 0U; y < height; ++y) {
        for (std::uint32_t x = 0U; x < width; ++x) {
            double sum = 0.0;
            for (int offset = -radius; offset <= radius; ++offset) {
                const auto sampleY = static_cast<std::uint32_t>(std::clamp(
                    static_cast<int>(y) + offset,
                    0,
                    static_cast<int>(height) - 1));
                const auto index = static_cast<std::size_t>(sampleY) * width + x;
                sum += static_cast<double>(horizontal[index]) *
                       kernel[static_cast<std::size_t>(offset + radius)];
            }
            output[static_cast<std::size_t>(y) * width + x] = static_cast<float>(sum);
        }
    }
    return output;
}

[[nodiscard]] FrameStatistics ComputeFrameStatistics(
    std::span<const float> alpha,
    float foregroundThreshold,
    float opticalDepthEpsilon) {
    FrameStatistics statistics;
    for (const float sourceAlpha : alpha) {
        const auto value = ClampedAlpha(sourceAlpha);
        statistics.foregroundPixelCount += value >= foregroundThreshold ? 1U : 0U;
        statistics.opticalDepthSum +=
            -std::log(std::max(1.0 - static_cast<double>(value),
                               static_cast<double>(opticalDepthEpsilon)));
    }
    statistics.foregroundCoverage =
        static_cast<double>(statistics.foregroundPixelCount) /
        static_cast<double>(alpha.size());
    return statistics;
}

}  // namespace

ComparisonResult Compare(
    const LinearFrameView& reference,
    const LinearFrameView& candidate,
    const Options& options) {
    ComparisonResult result;
    if (reference.width == 0U || reference.height == 0U) {
        result.errorMessage = "reference frame dimensions must be non-zero";
        return result;
    }
    if (reference.width != candidate.width || reference.height != candidate.height) {
        result.errorMessage = "reference and candidate frame dimensions must match";
        return result;
    }
    if (options.tileColumns == 0U || options.tileRows == 0U) {
        result.errorMessage = "tile grid dimensions must be non-zero";
        return result;
    }
    if (!std::isfinite(options.foregroundAlphaThreshold) ||
        options.foregroundAlphaThreshold < 0.0F ||
        options.foregroundAlphaThreshold > 1.0F) {
        result.errorMessage = "foreground alpha threshold must be within [0, 1]";
        return result;
    }
    if (!std::isfinite(options.rgbBlurSigmaPixels) ||
        options.rgbBlurSigmaPixels < 0.0F || options.rgbBlurSigmaPixels > 16.0F) {
        result.errorMessage = "RGB blur sigma must be finite and within [0, 16]";
        return result;
    }
    if (!std::isfinite(options.opticalDepthEpsilon) ||
        options.opticalDepthEpsilon <= 0.0F ||
        options.opticalDepthEpsilon >= 1.0F) {
        result.errorMessage = "optical-depth epsilon must be within (0, 1)";
        return result;
    }
    if (!std::isfinite(options.minimumValidDepth)) {
        result.errorMessage = "minimum valid depth must be finite";
        return result;
    }

    const auto maxSize = std::numeric_limits<std::size_t>::max();
    if (reference.height > maxSize / reference.width) {
        result.errorMessage = "frame dimensions exceed addressable storage";
        return result;
    }
    const auto pixelCount = static_cast<std::size_t>(reference.width) * reference.height;
    if (pixelCount > maxSize / 4U) {
        result.errorMessage = "frame dimensions exceed RGBA diagnostic storage";
        return result;
    }
    if (!ValidateFrame(reference, pixelCount, "reference", &result.errorMessage) ||
        !ValidateFrame(candidate, pixelCount, "candidate", &result.errorMessage)) {
        return result;
    }

    result.report.width = reference.width;
    result.report.height = reference.height;
    result.report.options = options;
    result.diagnostics.width = reference.width;
    result.diagnostics.height = reference.height;
    result.diagnostics.absoluteDifferenceRgba.resize(pixelCount * 4U, 0.0F);
    result.diagnostics.supportRgba.resize(pixelCount * 4U, 0U);

    std::vector<std::uint8_t> referenceSupport(pixelCount, 0U);
    std::vector<std::uint8_t> candidateSupport(pixelCount, 0U);
    for (std::size_t index = 0U; index < pixelCount; ++index) {
        referenceSupport[index] =
            ClampedAlpha(reference.alpha[index]) >= options.foregroundAlphaThreshold ? 1U : 0U;
        candidateSupport[index] =
            ClampedAlpha(candidate.alpha[index]) >= options.foregroundAlphaThreshold ? 1U : 0U;

        const auto outputOffset = index * 4U;
        result.diagnostics.absoluteDifferenceRgba[outputOffset] =
            std::abs(FiniteOrZero(candidate.red[index]) - FiniteOrZero(reference.red[index]));
        result.diagnostics.absoluteDifferenceRgba[outputOffset + 1U] =
            std::abs(FiniteOrZero(candidate.green[index]) - FiniteOrZero(reference.green[index]));
        result.diagnostics.absoluteDifferenceRgba[outputOffset + 2U] =
            std::abs(FiniteOrZero(candidate.blue[index]) - FiniteOrZero(reference.blue[index]));
        result.diagnostics.absoluteDifferenceRgba[outputOffset + 3U] =
            std::abs(ClampedAlpha(candidate.alpha[index]) - ClampedAlpha(reference.alpha[index]));
    }

    result.report.reference = ComputeFrameStatistics(
        reference.alpha,
        options.foregroundAlphaThreshold,
        options.opticalDepthEpsilon);
    result.report.candidate = ComputeFrameStatistics(
        candidate.alpha,
        options.foregroundAlphaThreshold,
        options.opticalDepthEpsilon);
    result.report.candidateToReferenceCoverageRatio = SafeRatio(
        result.report.candidate.foregroundCoverage,
        result.report.reference.foregroundCoverage);
    result.report.candidateToReferenceOpticalDepthRatio = SafeRatio(
        result.report.candidate.opticalDepthSum,
        result.report.reference.opticalDepthSum);

    std::vector<double> depthErrors;
    depthErrors.reserve(std::min(
        result.report.reference.foregroundPixelCount,
        result.report.candidate.foregroundPixelCount));
    for (std::uint32_t y = 0U; y < reference.height; ++y) {
        for (std::uint32_t x = 0U; x < reference.width; ++x) {
            const auto index = static_cast<std::size_t>(y) * reference.width + x;
            const bool referenceForeground = referenceSupport[index] != 0U;
            const bool candidateForeground = candidateSupport[index] != 0U;
            if (referenceForeground && candidateForeground) {
                SetDiagnosticPixel(
                    &result.diagnostics.supportRgba,
                    index,
                    kMutualSupport);
                const auto referenceDepth = reference.depth[index];
                const auto candidateDepth = candidate.depth[index];
                if (std::isfinite(referenceDepth) && std::isfinite(candidateDepth) &&
                    referenceDepth > options.minimumValidDepth &&
                    candidateDepth > options.minimumValidDepth) {
                    depthErrors.push_back(std::abs(
                        static_cast<double>(candidateDepth) - referenceDepth));
                }
                continue;
            }

            if (referenceForeground) {
                const bool tolerated = HasSupportWithinTolerance(
                    candidateSupport,
                    reference.width,
                    reference.height,
                    x,
                    y,
                    options.supportTolerancePixels);
                if (!tolerated) {
                    ++result.report.support.missingReferencePixelCount;
                }
                SetDiagnosticPixel(
                    &result.diagnostics.supportRgba,
                    index,
                    tolerated ? kToleratedReference : kMissingReference);
            } else if (candidateForeground) {
                const bool tolerated = HasSupportWithinTolerance(
                    referenceSupport,
                    reference.width,
                    reference.height,
                    x,
                    y,
                    options.supportTolerancePixels);
                if (!tolerated) {
                    ++result.report.support.excessCandidatePixelCount;
                }
                SetDiagnosticPixel(
                    &result.diagnostics.supportRgba,
                    index,
                    tolerated ? kToleratedCandidate : kExcessCandidate);
            }
        }
    }
    result.report.support.missingFractionOfReference = SafeFraction(
        static_cast<double>(result.report.support.missingReferencePixelCount),
        static_cast<double>(result.report.reference.foregroundPixelCount));
    result.report.support.excessFractionOfCandidate = SafeFraction(
        static_cast<double>(result.report.support.excessCandidatePixelCount),
        static_cast<double>(result.report.candidate.foregroundPixelCount));

    auto& tileStatistics = result.report.tileCoverage;
    tileStatistics.columns = std::min(options.tileColumns, reference.width);
    tileStatistics.rows = std::min(options.tileRows, reference.height);
    std::vector<double> tileDeltas;
    tileDeltas.reserve(
        static_cast<std::size_t>(tileStatistics.columns) * tileStatistics.rows);
    for (std::uint32_t tileY = 0U; tileY < tileStatistics.rows; ++tileY) {
        const auto beginY = tileY * reference.height / tileStatistics.rows;
        const auto endY = (tileY + 1U) * reference.height / tileStatistics.rows;
        for (std::uint32_t tileX = 0U; tileX < tileStatistics.columns; ++tileX) {
            const auto beginX = tileX * reference.width / tileStatistics.columns;
            const auto endX = (tileX + 1U) * reference.width / tileStatistics.columns;
            std::size_t referenceCount = 0U;
            std::size_t candidateCount = 0U;
            for (auto y = beginY; y < endY; ++y) {
                for (auto x = beginX; x < endX; ++x) {
                    const auto index = static_cast<std::size_t>(y) * reference.width + x;
                    referenceCount += referenceSupport[index] != 0U ? 1U : 0U;
                    candidateCount += candidateSupport[index] != 0U ? 1U : 0U;
                }
            }
            const auto tilePixelCount =
                static_cast<double>(endX - beginX) *
                static_cast<double>(endY - beginY);
            tileDeltas.push_back(std::abs(
                static_cast<double>(candidateCount) / tilePixelCount -
                static_cast<double>(referenceCount) / tilePixelCount));
        }
    }
    tileStatistics.sampleCount = tileDeltas.size();
    tileStatistics.p50AbsoluteDelta = NearestRankPercentile(tileDeltas, 0.50);
    tileStatistics.p95AbsoluteDelta = NearestRankPercentile(tileDeltas, 0.95);
    tileStatistics.maximumAbsoluteDelta =
        tileDeltas.empty() ? 0.0 : *std::max_element(tileDeltas.begin(), tileDeltas.end());

    const auto kernel = MakeGaussianKernel(options.rgbBlurSigmaPixels);
    const std::array referenceChannels{
        GaussianBlur(reference.red, reference.width, reference.height, kernel),
        GaussianBlur(reference.green, reference.width, reference.height, kernel),
        GaussianBlur(reference.blue, reference.width, reference.height, kernel),
    };
    const std::array candidateChannels{
        GaussianBlur(candidate.red, candidate.width, candidate.height, kernel),
        GaussianBlur(candidate.green, candidate.width, candidate.height, kernel),
        GaussianBlur(candidate.blue, candidate.width, candidate.height, kernel),
    };
    double absoluteErrorSum = 0.0;
    double squaredErrorSum = 0.0;
    for (std::size_t channel = 0U; channel < referenceChannels.size(); ++channel) {
        for (std::size_t index = 0U; index < pixelCount; ++index) {
            const auto error = std::abs(
                static_cast<double>(candidateChannels[channel][index]) -
                referenceChannels[channel][index]);
            absoluteErrorSum += error;
            squaredErrorSum += error * error;
        }
    }
    auto& rgbStatistics = result.report.blurredLinearRgb;
    rgbStatistics.sampleCount = pixelCount * 3U;
    rgbStatistics.meanAbsoluteError =
        absoluteErrorSum / static_cast<double>(rgbStatistics.sampleCount);
    rgbStatistics.rootMeanSquareError = std::sqrt(
        squaredErrorSum / static_cast<double>(rgbStatistics.sampleCount));

    auto& depthStatistics = result.report.mutualSupportDepth;
    depthStatistics.sampleCount = depthErrors.size();
    depthStatistics.medianAbsoluteError = Median(depthErrors);
    depthStatistics.p95AbsoluteError = NearestRankPercentile(std::move(depthErrors), 0.95);

    result.success = true;
    return result;
}

}  // namespace invisible_places::app::point_density_parity
