#include "output/OfflinePointRenderer.hpp"
#include "water/RainSimulation.hpp"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <vector>

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
