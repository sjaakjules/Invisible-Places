#include "app/PointDensityParity.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace {

using Catch::Approx;
using invisible_places::app::point_density_parity::LinearFrameView;

struct OwnedFrame {
    std::uint32_t width = 0U;
    std::uint32_t height = 0U;
    std::vector<float> red;
    std::vector<float> green;
    std::vector<float> blue;
    std::vector<float> alpha;
    std::vector<float> depth;

    [[nodiscard]] LinearFrameView View() const {
        return {
            .width = width,
            .height = height,
            .red = red,
            .green = green,
            .blue = blue,
            .alpha = alpha,
            .depth = depth,
        };
    }
};

OwnedFrame MakeFrame(std::uint32_t width, std::uint32_t height) {
    const auto count = static_cast<std::size_t>(width) * height;
    return {
        .width = width,
        .height = height,
        .red = std::vector<float>(count, 0.0F),
        .green = std::vector<float>(count, 0.0F),
        .blue = std::vector<float>(count, 0.0F),
        .alpha = std::vector<float>(count, 0.0F),
        .depth = std::vector<float>(count, std::numeric_limits<float>::infinity()),
    };
}

void SetPixel(
    OwnedFrame* frame,
    std::size_t index,
    float color,
    float alpha,
    float depth) {
    frame->red[index] = color;
    frame->green[index] = color;
    frame->blue[index] = color;
    frame->alpha[index] = alpha;
    frame->depth[index] = depth;
}

}  // namespace

TEST_CASE(
    "Point density parity reports identical frames without error",
    "[pointcloud][density][parity]") {
    auto frame = MakeFrame(2U, 2U);
    SetPixel(&frame, 0U, 0.25F, 0.5F, 3.0F);
    SetPixel(&frame, 3U, 1.5F, 1.0F, 8.0F);

    const auto result = invisible_places::app::point_density_parity::Compare(
        frame.View(),
        frame.View());

    REQUIRE(result.success);
    CHECK(result.errorMessage.empty());
    CHECK(result.report.reference.foregroundPixelCount == 2U);
    CHECK(result.report.reference.foregroundCoverage == Approx(0.5));
    CHECK(result.report.candidateToReferenceCoverageRatio == Approx(1.0));
    CHECK(result.report.candidateToReferenceOpticalDepthRatio == Approx(1.0));
    CHECK(result.report.support.missingReferencePixelCount == 0U);
    CHECK(result.report.support.excessCandidatePixelCount == 0U);
    CHECK(result.report.tileCoverage.p95AbsoluteDelta == Approx(0.0));
    CHECK(result.report.blurredLinearRgb.meanAbsoluteError == Approx(0.0));
    CHECK(result.report.blurredLinearRgb.rootMeanSquareError == Approx(0.0));
    CHECK(result.report.mutualSupportDepth.sampleCount == 2U);
    CHECK(result.report.mutualSupportDepth.medianAbsoluteError == Approx(0.0));
    CHECK(result.diagnostics.absoluteDifferenceRgba.size() == 16U);
    CHECK(result.diagnostics.supportRgba.size() == 16U);
    const std::array<std::uint8_t, 4U> expectedMutual{64U, 200U, 96U, 255U};
    CHECK(std::equal(
        expectedMutual.begin(),
        expectedMutual.end(),
        result.diagnostics.supportRgba.begin()));
}

TEST_CASE(
    "Point density parity tolerates a one-pixel support shift",
    "[pointcloud][density][parity][support]") {
    auto reference = MakeFrame(3U, 1U);
    auto candidate = MakeFrame(3U, 1U);
    SetPixel(&reference, 0U, 0.5F, 1.0F, 4.0F);
    SetPixel(&candidate, 1U, 0.5F, 1.0F, 4.0F);

    const auto result = invisible_places::app::point_density_parity::Compare(
        reference.View(),
        candidate.View());

    REQUIRE(result.success);
    CHECK(result.report.support.missingReferencePixelCount == 0U);
    CHECK(result.report.support.excessCandidatePixelCount == 0U);
    CHECK(result.report.mutualSupportDepth.sampleCount == 0U);
    const auto& diagnostic = result.diagnostics.supportRgba;
    const std::array<std::uint8_t, 4U> expectedReference{255U, 190U, 0U, 255U};
    const std::array<std::uint8_t, 4U> expectedCandidate{0U, 190U, 255U, 255U};
    CHECK(std::equal(
        expectedReference.begin(),
        expectedReference.end(),
        diagnostic.begin()));
    CHECK(std::equal(
        expectedCandidate.begin(),
        expectedCandidate.end(),
        diagnostic.begin() + 4));
}

TEST_CASE(
    "Point density parity exposes missing and excess support beyond tolerance",
    "[pointcloud][density][parity][support]") {
    auto reference = MakeFrame(5U, 1U);
    auto candidate = MakeFrame(5U, 1U);
    SetPixel(&reference, 0U, 0.2F, 0.5F, 1.0F);
    SetPixel(&candidate, 3U, 0.2F, 0.5F, 1.0F);

    const auto result = invisible_places::app::point_density_parity::Compare(
        reference.View(),
        candidate.View());

    REQUIRE(result.success);
    CHECK(result.report.support.missingReferencePixelCount == 1U);
    CHECK(result.report.support.excessCandidatePixelCount == 1U);
    CHECK(result.report.support.missingFractionOfReference == Approx(1.0));
    CHECK(result.report.support.excessFractionOfCandidate == Approx(1.0));
    CHECK(result.report.candidateToReferenceCoverageRatio == Approx(1.0));
}

TEST_CASE(
    "Point density parity measures optical depth colour and valid mutual depth",
    "[pointcloud][density][parity][metrics]") {
    auto reference = MakeFrame(2U, 1U);
    auto candidate = MakeFrame(2U, 1U);
    for (std::size_t index = 0U; index < 2U; ++index) {
        SetPixel(&reference, index, 0.2F, 0.5F, 2.0F + static_cast<float>(index));
        SetPixel(&candidate, index, 0.4F, 0.75F, 4.0F + static_cast<float>(index) * 3.0F);
    }
    candidate.depth[1] = std::numeric_limits<float>::quiet_NaN();

    const auto result = invisible_places::app::point_density_parity::Compare(
        reference.View(),
        candidate.View());

    REQUIRE(result.success);
    CHECK(result.report.candidateToReferenceOpticalDepthRatio == Approx(2.0));
    CHECK(result.report.blurredLinearRgb.sampleCount == 6U);
    CHECK(result.report.blurredLinearRgb.meanAbsoluteError == Approx(0.2));
    CHECK(result.report.blurredLinearRgb.rootMeanSquareError == Approx(0.2));
    CHECK(result.report.mutualSupportDepth.sampleCount == 1U);
    CHECK(result.report.mutualSupportDepth.medianAbsoluteError == Approx(2.0));
    CHECK(result.report.mutualSupportDepth.p95AbsoluteError == Approx(2.0));
    CHECK(result.diagnostics.absoluteDifferenceRgba[0U] == Approx(0.2F));
    CHECK(result.diagnostics.absoluteDifferenceRgba[3U] == Approx(0.25F));
}

TEST_CASE(
    "Point density parity tile statistics use bounded non-empty tiles",
    "[pointcloud][density][parity][tiles]") {
    auto reference = MakeFrame(4U, 1U);
    auto candidate = MakeFrame(4U, 1U);
    SetPixel(&candidate, 0U, 1.0F, 1.0F, 1.0F);

    invisible_places::app::point_density_parity::Options options;
    options.tileColumns = 32U;
    options.tileRows = 18U;
    const auto result = invisible_places::app::point_density_parity::Compare(
        reference.View(),
        candidate.View(),
        options);

    REQUIRE(result.success);
    CHECK(result.report.tileCoverage.columns == 4U);
    CHECK(result.report.tileCoverage.rows == 1U);
    CHECK(result.report.tileCoverage.sampleCount == 4U);
    CHECK(result.report.tileCoverage.p50AbsoluteDelta == Approx(0.0));
    CHECK(result.report.tileCoverage.p95AbsoluteDelta == Approx(1.0));
    CHECK(result.report.tileCoverage.maximumAbsoluteDelta == Approx(1.0));
}

TEST_CASE(
    "Point density parity uses a conventional even-sample depth median",
    "[pointcloud][density][parity][depth]") {
    auto reference = MakeFrame(2U, 1U);
    auto candidate = MakeFrame(2U, 1U);
    SetPixel(&reference, 0U, 0.2F, 1.0F, 2.0F);
    SetPixel(&reference, 1U, 0.2F, 1.0F, 3.0F);
    SetPixel(&candidate, 0U, 0.2F, 1.0F, 3.0F);
    SetPixel(&candidate, 1U, 0.2F, 1.0F, 6.0F);

    const auto result = invisible_places::app::point_density_parity::Compare(
        reference.View(),
        candidate.View());

    REQUIRE(result.success);
    CHECK(result.report.mutualSupportDepth.sampleCount == 2U);
    CHECK(result.report.mutualSupportDepth.medianAbsoluteError == Approx(2.0));
    CHECK(result.report.mutualSupportDepth.p95AbsoluteError == Approx(3.0));
}

TEST_CASE(
    "Point density parity rejects mismatched frame storage",
    "[pointcloud][density][parity][validation]") {
    auto reference = MakeFrame(2U, 2U);
    auto candidate = MakeFrame(2U, 2U);
    candidate.depth.pop_back();

    const auto result = invisible_places::app::point_density_parity::Compare(
        reference.View(),
        candidate.View());

    CHECK_FALSE(result.success);
    CHECK(result.errorMessage.find("candidate frame channels") != std::string::npos);
    CHECK(result.diagnostics.absoluteDifferenceRgba.empty());
    CHECK(result.diagnostics.supportRgba.empty());
}

TEST_CASE(
    "Point density parity defines empty support as matched without missing fractions",
    "[pointcloud][density][parity][support]") {
    const auto reference = MakeFrame(2U, 1U);
    const auto candidate = MakeFrame(2U, 1U);

    const auto result = invisible_places::app::point_density_parity::Compare(
        reference.View(),
        candidate.View());

    REQUIRE(result.success);
    CHECK(result.report.candidateToReferenceCoverageRatio == Approx(1.0));
    CHECK(result.report.candidateToReferenceOpticalDepthRatio == Approx(1.0));
    CHECK(result.report.support.missingFractionOfReference == Approx(0.0));
    CHECK(result.report.support.excessFractionOfCandidate == Approx(0.0));
}
