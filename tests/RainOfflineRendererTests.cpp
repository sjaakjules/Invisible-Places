#include "output/OfflinePointRenderer.hpp"
#include "style/RenderParameterBinding.hpp"
#include "water/RainSimulation.hpp"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace {

invisible_places::io::LoadedPointCloud MakeRockRainReadbackPatch() {
    invisible_places::io::LoadedPointCloud cloud;
    constexpr int kHalfWidth = 18;
    constexpr int kHalfHeight = 12;
    constexpr float kSpacingMeters = 0.01F;
    cloud.positions.reserve(
        static_cast<std::size_t>((kHalfWidth * 2) + 1) *
        static_cast<std::size_t>((kHalfHeight * 2) + 1));
    for (int y = -kHalfHeight; y <= kHalfHeight; ++y) {
        for (int x = -kHalfWidth; x <= kHalfWidth; ++x) {
            cloud.positions.push_back({
                static_cast<float>(x) * kSpacingMeters,
                static_cast<float>(y) * kSpacingMeters,
                0.0F,
            });
            cloud.normals.push_back({0.0F, 0.0F, 1.0F});
            cloud.packedColors.push_back(0xff101010U);
            cloud.bounds.Expand(cloud.positions.back());
        }
    }
    cloud.hasNormals = true;
    cloud.hasSourceRgb = true;
    return cloud;
}

std::vector<std::uint8_t> RenderRockRainReadback(
    const invisible_places::io::LoadedPointCloud& cloud,
    const invisible_places::water::RainImpactGrid* impactGrid) {
    invisible_places::renderer::pointcloud::PointCloudStyleState style;
    style.colorMode =
        invisible_places::renderer::pointcloud::PointCloudColorMode::SolidColor;
    style.solidColor = {0.035F, 0.045F, 0.055F, 1.0F};
    style.falloffProfile =
        invisible_places::renderer::pointcloud::PointCloudFalloffProfile::HardDisc;
    style.solidCenters = true;
    invisible_places::style::SetScalarConstant(&style.pointSize, 3.0F);
    invisible_places::style::SetScalarConstant(&style.opacity, 0.38F);
    invisible_places::style::SetScalarConstant(&style.emissiveStrength, 0.0F);

    const invisible_places::output::OfflinePointLayer layer{
        .cloud = &cloud,
        .style = style,
        .hasSourceRgb = true,
        .localToWorld = glm::mat4{1.0F},
        .rainCollisionRole =
            invisible_places::water::WaterSurfaceRole::Rock,
        .rainImpactGrid = impactGrid,
    };
    invisible_places::camera::CameraState camera;
    camera.position = {0.0F, 0.0F, 1.0F};
    camera.target = {0.0F, 0.0F, 0.0F};
    camera.fovDegrees = 28.0F;
    camera.nearPlane = 0.01F;
    camera.farPlane = 10.0F;

    constexpr std::uint32_t kWidth = 192U;
    constexpr std::uint32_t kHeight = 128U;
    invisible_places::output::ExrImage image;
    invisible_places::output::InitializeExrImage(
        &image,
        kWidth,
        kHeight);
    invisible_places::output::RenderPointCloudTile(
        {layer},
        camera,
        {0U, 0U, kWidth, kHeight},
        &image,
        nullptr,
        nullptr,
        1.0F);
    const auto toSrgb8 = [](float linear) {
        const float clamped = std::clamp(linear, 0.0F, 1.0F);
        const float encoded =
            clamped <= 0.0031308F
                ? 12.92F * clamped
                : (1.055F * std::pow(clamped, 1.0F / 2.4F)) - 0.055F;
        return static_cast<std::uint8_t>(
            std::clamp(std::lround(encoded * 255.0F), 0L, 255L));
    };
    std::vector<std::uint8_t> rgba;
    rgba.reserve(static_cast<std::size_t>(kWidth) * kHeight * 4U);
    for (std::size_t pixel = 0U; pixel < image.alpha.size(); ++pixel) {
        rgba.push_back(toSrgb8(image.beautyR[pixel]));
        rgba.push_back(toSrgb8(image.beautyG[pixel]));
        rgba.push_back(toSrgb8(image.beautyB[pixel]));
        rgba.push_back(toSrgb8(image.alpha[pixel]));
    }
    return rgba;
}

std::uint32_t PixelLuma8(
    const std::vector<std::uint8_t>& rgba,
    std::size_t pixelIndex) {
    const auto offset = pixelIndex * 4U;
    // Integer Rec.709 weights keep the comparison in the same one-step
    // 8-bit domain used by the native acceptance readback.
    return (
        54U * rgba[offset] +
        183U * rgba[offset + 1U] +
        19U * rgba[offset + 2U] +
        128U) >>
        8U;
}

}  // namespace

TEST_CASE("offline rain advances once before all tiles consume the frame", "[water][rain][offline]") {
    std::vector<invisible_places::water::WaterSurfaceSample> samples;
    for (int y = -10; y <= 10; ++y) {
        for (int x = -10; x <= 10; ++x) {
            samples.push_back({
                {x * 0.02F, y * 0.02F, 0.0F},
                {0.0F, 0.0F, 1.0F},
                invisible_places::water::WaterSurfaceRole::Rock,
            });
        }
    }
    const auto cache = invisible_places::water::BuildWaterSurfaceCacheFromSamples(samples);
    auto settings = invisible_places::water::DefaultRainRuntimeSettings();
    settings.enabled = true;
    settings.impactEffectsEnabled = true;
    settings.activeParticleCount = 128U;
    settings.rainLevel = 1.0F;
    settings.density = 1.0F;
    settings.weatherFrontStrength = 0.0F;
    settings.spawnRadiusMeters = 0.15F;
    settings.spawnHeightMeters = 0.35F;
    settings.cameraDeathDistanceMeters = 100.0F;

    invisible_places::camera::CameraState camera;
    camera.position = {0.0F, 0.0F, 2.0F};
    camera.target = {0.0F, 0.0F, 0.0F};
    camera.orbitCenter = camera.target;
    camera.hasOrbitCenter = true;
    camera.nearPlane = 0.01F;
    camera.farPlane = 20.0F;

    invisible_places::output::OfflineRainSimulationState rainState{128U};
    constexpr float deltaSeconds = 1.0F / 30.0F;
    for (int frameIndex = 0; frameIndex < 8; ++frameIndex) {
        invisible_places::output::AdvanceOfflineRainFrame(
            &rainState,
            cache,
            settings,
            invisible_places::water::RainVisualPreset("Rain Fine Lines"),
            camera,
            frameIndex * deltaSeconds,
            deltaSeconds);
    }

    REQUIRE(rainState.frame.particles.size() == 128U);
    CHECK(rainState.frame.particles.data() == rainState.simulator.Particles().data());
    CHECK_FALSE(rainState.impactGrid.cells.empty());
    CHECK(rainState.diagnostics.activeParticles > 0U);

    const auto firstParticleBefore = rainState.frame.particles.front();
    invisible_places::output::ExrImage image;
    invisible_places::output::InitializeExrImage(&image, 64U, 64U);
    const auto tiles = invisible_places::output::BuildOfflineRenderTiles(64U, 64U, 32U);
    for (const auto& tile : tiles) {
        invisible_places::output::RenderPointCloudTile(
            {}, camera, tile, &image, nullptr, nullptr, 7.0F * deltaSeconds, &rainState.frame);
    }
    const auto firstParticleAfter = rainState.frame.particles.front();
    CHECK(firstParticleAfter.generation == firstParticleBefore.generation);
    CHECK(firstParticleAfter.position.x == firstParticleBefore.position.x);
    CHECK(firstParticleAfter.position.y == firstParticleBefore.position.y);
    CHECK(firstParticleAfter.position.z == firstParticleBefore.position.z);
    CHECK(std::any_of(image.alpha.begin(), image.alpha.end(), [](float alpha) { return alpha > 0.0F; }));

    settings.impactEffectsEnabled = false;
    invisible_places::output::AdvanceOfflineRainFrame(
        &rainState,
        cache,
        settings,
        invisible_places::water::RainVisualPreset("Rain Fine Lines"),
        camera,
        8.0F * deltaSeconds,
        deltaSeconds);
    CHECK(rainState.impactGrid.cells.empty());
    CHECK(rainState.diagnostics.emittedEvents == 0U);
}

TEST_CASE(
    "two overlapping ROCK drops never darken the offline readback",
    "[water][rain][offline][rock][union][readback]") {
    auto rockSettings =
        invisible_places::water::RainRockImpactSettings{};
    rockSettings.edgeBreakup = 0.0F;
    rockSettings.spreadSpeed = 6.0F;
    rockSettings.centreFalloff = 1.0F;
    rockSettings.heightBias = 0.0F;

    const invisible_places::water::RainImpactEvent first{
        .position = {-0.025F, 0.0F, 0.0F},
        .birthTimeSeconds = 0.0F,
        .normal = {0.0F, 0.0F, 1.0F},
        .radiusMeters = 0.14F,
        .role = invisible_places::water::WaterSurfaceRole::Rock,
        .lifetimeSeconds = 5.0F,
        .energy = 0.55F,
        .seed = 7U,
    };
    auto second = first;
    second.position.x = 0.025F;
    second.seed = 11U;

    const std::array firstEvents{first};
    const std::array secondEvents{second};
    const std::array combinedEvents{first, second};
    const auto firstGrid =
        invisible_places::water::BuildRainImpactGrid(
            firstEvents,
            {},
            1.0F,
            2.0F,
            rockSettings);
    const auto secondGrid =
        invisible_places::water::BuildRainImpactGrid(
            secondEvents,
            {},
            1.0F,
            2.0F,
            rockSettings);
    const auto combinedGrid =
        invisible_places::water::BuildRainImpactGrid(
            combinedEvents,
            {},
            1.0F,
            2.0F,
            rockSettings);

    const auto cloud = MakeRockRainReadbackPatch();
    const auto baseline = RenderRockRainReadback(cloud, nullptr);
    const auto firstOnly = RenderRockRainReadback(cloud, &firstGrid);
    const auto secondOnly = RenderRockRainReadback(cloud, &secondGrid);
    const auto combined = RenderRockRainReadback(cloud, &combinedGrid);
    REQUIRE_FALSE(baseline.empty());
    REQUIRE(firstOnly.size() == baseline.size());
    REQUIRE(secondOnly.size() == baseline.size());
    REQUIRE(combined.size() == baseline.size());

    std::size_t overlapPixelCount = 0U;
    std::size_t strengthenedPixelCount = 0U;
    std::uint32_t worstDarkeningSteps = 0U;
    std::uint32_t minimumPositiveCombinedLift =
        std::numeric_limits<std::uint32_t>::max();
    std::uint32_t maximumCombinedLift = 0U;
    for (std::size_t pixel = 0U;
         pixel < baseline.size() / 4U;
         ++pixel) {
        const auto baselineLuma = PixelLuma8(baseline, pixel);
        const auto firstLuma = PixelLuma8(firstOnly, pixel);
        const auto secondLuma = PixelLuma8(secondOnly, pixel);
        const auto combinedLuma = PixelLuma8(combined, pixel);
        if (combinedLuma > baselineLuma) {
            const auto lift = combinedLuma - baselineLuma;
            minimumPositiveCombinedLift =
                std::min(minimumPositiveCombinedLift, lift);
            maximumCombinedLift = std::max(maximumCombinedLift, lift);
        }
        if (firstLuma <= baselineLuma + 1U ||
            secondLuma <= baselineLuma + 1U) {
            continue;
        }
        ++overlapPixelCount;
        const auto strongerSingle = std::max(firstLuma, secondLuma);
        if (combinedLuma > strongerSingle) {
            ++strengthenedPixelCount;
        } else if (strongerSingle > combinedLuma) {
            worstDarkeningSteps = std::max(
                worstDarkeningSteps,
                strongerSingle - combinedLuma);
        }
    }

    REQUIRE(overlapPixelCount >= 64U);
    CHECK(worstDarkeningSteps <= 1U);
    CHECK(strengthenedPixelCount > 0U);
    REQUIRE(
        minimumPositiveCombinedLift !=
        std::numeric_limits<std::uint32_t>::max());
    CHECK(maximumCombinedLift > minimumPositiveCombinedLift + 1U);
}
