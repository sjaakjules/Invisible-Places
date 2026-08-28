#include "app/BackgroundRenderPreparation.hpp"

#include <catch2/catch_test_macros.hpp>

TEST_CASE(
    "Background render preparation waits for derived Seepage and Flow resources",
    "[export][background-render-preparation][water]") {
    using invisible_places::app::BackgroundRenderPreparationObservation;
    using invisible_places::app::BackgroundRenderPreparationReady;

    const BackgroundRenderPreparationObservation ready{
        .initialFlowRefreshRequested = true,
        .flowJobsSettled = true,
        .requiredSeepageTopologyCount = 3U,
        .readySeepageTopologyCount = 3U,
    };
    CHECK(BackgroundRenderPreparationReady(ready));

    auto observation = ready;
    observation.waterSurfaceWarmupActive = true;
    CHECK_FALSE(BackgroundRenderPreparationReady(observation));

    observation = ready;
    observation.waterSurfacePreprocessPending = true;
    CHECK_FALSE(BackgroundRenderPreparationReady(observation));

    observation = ready;
    observation.initialFlowRefreshRequested = false;
    CHECK_FALSE(BackgroundRenderPreparationReady(observation));

    observation = ready;
    observation.flowJobsSettled = false;
    CHECK_FALSE(BackgroundRenderPreparationReady(observation));

    observation = ready;
    observation.seepageSupportActive = true;
    CHECK_FALSE(BackgroundRenderPreparationReady(observation));

    observation = ready;
    observation.readySeepageTopologyCount = 2U;
    CHECK_FALSE(BackgroundRenderPreparationReady(observation));

    observation = ready;
    observation.layerLoadActive = true;
    CHECK_FALSE(BackgroundRenderPreparationReady(observation));

    observation = ready;
    observation.scalarFieldLoadActive = true;
    CHECK_FALSE(BackgroundRenderPreparationReady(observation));

    observation = ready;
    observation.queuedLayerLoadCount = 1U;
    CHECK_FALSE(BackgroundRenderPreparationReady(observation));
}
