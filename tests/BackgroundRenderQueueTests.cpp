#include "app/BackgroundRenderQueue.hpp"

#include <catch2/catch_test_macros.hpp>

#include <array>

TEST_CASE(
    "Background render dependencies serialize live workers and release terminal or stale dead workers",
    "[export][background-queue]") {
    using invisible_places::app::BackgroundRenderDependencyObservation;
    using invisible_places::app::BackgroundRenderDependencyBlocks;

    CHECK_FALSE(BackgroundRenderDependencyBlocks({
        .statusLoaded = true,
        .terminal = true,
        .processId = 10,
        .processAlive = false,
        .statusFresh = true,
    }));
    CHECK(BackgroundRenderDependencyBlocks({
        .statusLoaded = true,
        .terminal = false,
        .processId = 11,
        .processAlive = true,
        .statusFresh = true,
    }));
    CHECK(BackgroundRenderDependencyBlocks({
        .statusLoaded = true,
        .terminal = false,
        .processId = 12,
        .processAlive = false,
        .statusFresh = true,
    }));
    CHECK_FALSE(BackgroundRenderDependencyBlocks({
        .statusLoaded = true,
        .terminal = false,
        .processId = 13,
        .processAlive = false,
        .statusFresh = false,
    }));
    CHECK(BackgroundRenderDependencyBlocks({}));

    const std::array dependencies{
        BackgroundRenderDependencyObservation{
            .statusLoaded = true,
            .terminal = true,
        },
        BackgroundRenderDependencyObservation{
            .statusLoaded = true,
            .processId = 20,
            .processAlive = true,
            .statusFresh = true,
        },
        BackgroundRenderDependencyObservation{
            .statusLoaded = true,
            .processId = 21,
            .processAlive = false,
            .statusFresh = false,
        },
        BackgroundRenderDependencyObservation{},
    };
    CHECK(invisible_places::app::CountBlockingBackgroundRenderDependencies(
              dependencies) == 2U);
}
