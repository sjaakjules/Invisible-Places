#include "camera/AnimationPath.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace {

using Catch::Approx;
using invisible_places::camera::CollectRayHitDistancesAlongRay;
using invisible_places::camera::ResolveFirstRayHitCluster;
using invisible_places::camera::ResolveSurfaceFocusDistance;
using invisible_places::io::Float3;

std::vector<Float3> PlaneOfPoints(float x, float extent, float spacing) {
    std::vector<Float3> points;
    for (float y = -extent; y <= extent; y += spacing) {
        for (float z = -extent; z <= extent; z += spacing) {
            points.push_back({x, y, z});
        }
    }
    return points;
}

invisible_places::camera::AnimationPath MakeReciprocalPanTestPath(
    std::string name,
    float direction,
    std::uint32_t firstSegmentFrames,
    std::uint32_t secondSegmentFrames) {
    invisible_places::camera::AnimationPath path;
    path.name = std::move(name);
    path.durationFrames = firstSegmentFrames + secondSegmentFrames;
    path.exportSettings.startFrame = 7U;
    path.exportSettings.endFrame = 91U;
    path.keys = {
        {.id = "start",
         .cameraPosition = {-1.4F * direction, -8.0F, 2.0F},
         .focusPoint = {-0.3F * direction, 0.0F, 1.0F}},
        {.id = "middle",
         .cameraPosition = {-0.4F * direction, -8.0F, 2.0F},
         .focusPoint = {0.0F, 0.0F, 1.0F},
         .durationFrames = firstSegmentFrames},
        {.id = "end",
         .cameraPosition = {0.6F * direction, -8.0F, 2.0F},
         .focusPoint = {0.3F * direction, 0.0F, 1.0F},
         .durationFrames = secondSegmentFrames},
    };
    for (std::size_t index = 0U; index < path.keys.size(); ++index) {
        path.keys[index].id = path.name + "_" + path.keys[index].id;
    }
    return path;
}

void AddLegacyWaterTracks(
    invisible_places::camera::AnimationPath* path) {
    REQUIRE(path != nullptr);
    invisible_places::water::WaterScenarioTrack scenarioTrack;
    scenarioTrack.scenarioId = "legacy-water";
    scenarioTrack.keys = {
        {.id = "scenario-quarter", .position = 0.25F},
        {.id = "scenario-terminal", .position = 1.0F},
    };
    invisible_places::water::WaterSeepageNodeTrack nodeTrack;
    nodeTrack.nodeId = 42U;
    nodeTrack.keys = {
        {.id = "node-half", .position = 0.50F},
        {.id = "node-terminal", .position = 1.0F},
    };
    scenarioTrack.seepageNodeTracks.push_back(std::move(nodeTrack));
    invisible_places::water::WaterTimingRunAssignment assignment;
    assignment.feature = invisible_places::water::WaterTimingFeature::Rain;
    assignment.fallbackRun.id = "fallback-rain";
    assignment.fallbackRun.keys = {
        {.id = "rain-three-quarter", .position = 0.75F, .level = 0.4F},
        {.id = "rain-terminal", .position = 1.0F, .level = 0.8F},
    };
    scenarioTrack.timingAssignments.push_back(std::move(assignment));
    path->waterScenarioTracks = {std::move(scenarioTrack)};
}

void CheckLegacyWaterTracksRetimed(
    const invisible_places::camera::AnimationPath& path,
    float scale) {
    REQUIRE(path.waterScenarioTracks.size() == 1U);
    const auto& scenarioTrack = path.waterScenarioTracks.front();
    REQUIRE(scenarioTrack.keys.size() == 2U);
    CHECK(scenarioTrack.keys[0U].id == "scenario-quarter");
    CHECK(scenarioTrack.keys[0U].position == Approx(0.25F * scale));
    CHECK(scenarioTrack.keys[1U].position == Approx(scale));
    REQUIRE(scenarioTrack.seepageNodeTracks.size() == 1U);
    const auto& nodeTrack = scenarioTrack.seepageNodeTracks.front();
    CHECK(nodeTrack.nodeId == 42U);
    REQUIRE(nodeTrack.keys.size() == 2U);
    CHECK(nodeTrack.keys[0U].position == Approx(0.50F * scale));
    CHECK(nodeTrack.keys[1U].position == Approx(scale));
    REQUIRE(scenarioTrack.timingAssignments.size() == 1U);
    const auto& fallback =
        scenarioTrack.timingAssignments.front().fallbackRun;
    CHECK(fallback.id == "fallback-rain");
    REQUIRE(fallback.keys.size() == 2U);
    CHECK(fallback.keys[0U].position == Approx(0.75F * scale));
    CHECK(fallback.keys[0U].level == Approx(0.4F));
    CHECK(fallback.keys[1U].position == Approx(scale));
}

invisible_places::camera::AnimationSurfacePatchObservation
MakeReciprocalPanTestPatch() {
    invisible_places::camera::AnimationSurfacePatchObservation patch;
    patch.pointCount = 3U;
    patch.worldPoints[0U] = {0.0F, 0.0F, 1.0F};
    patch.worldPoints[1U] = {0.25F, 0.0F, 1.0F};
    patch.worldPoints[2U] = {0.0F, 0.0F, 1.25F};
    return patch;
}

invisible_places::camera::AnimationReciprocalPanExtensionOptions
MakeReciprocalPanTestOptions() {
    invisible_places::camera::AnimationReciprocalPanExtensionOptions options;
    const auto patch = MakeReciprocalPanTestPatch();
    options.firstDrivesSecond.sourceTailFrame = 35U;
    options.firstDrivesSecond.sourcePatch = patch;
    auto actualBTerminalPatch = patch;
    actualBTerminalPatch.worldPoints[0U][0U] = 2.0F;
    actualBTerminalPatch.worldPoints[1U][0U] = 3.0F;
    actualBTerminalPatch.worldPoints[2U][0U] = 2.0F;
    options.firstDrivesSecond.destinationEndPatch = actualBTerminalPatch;
    options.secondDrivesFirst.sourceTailFrame = 44U;
    options.secondDrivesFirst.sourcePatch = patch;
    auto actualATerminalPatch = patch;
    actualATerminalPatch.worldPoints[0U][0U] = 1.5F;
    actualATerminalPatch.worldPoints[1U][0U] = 2.5F;
    actualATerminalPatch.worldPoints[2U][0U] = 1.5F;
    options.secondDrivesFirst.destinationEndPatch = actualATerminalPatch;
    options.sampleCount = 9U;
    options.optimizationSweeps = 12U;
    return options;
}

void CheckPanEvaluationNear(
    const invisible_places::camera::AnimationPathEvaluation& left,
    const invisible_places::camera::AnimationPathEvaluation& right,
    float margin = 2.0e-4F) {
    for (std::size_t component = 0U; component < 3U; ++component) {
        CHECK(left.camera.position[component] ==
              Approx(right.camera.position[component]).margin(margin));
        CHECK(left.focusPoint[component] ==
              Approx(right.focusPoint[component]).margin(margin));
    }
    CHECK(left.camera.fovDegrees ==
          Approx(right.camera.fovDegrees).margin(1.0e-5F));
    CHECK(left.camera.nearPlane ==
          Approx(right.camera.nearPlane).margin(1.0e-6F));
    CHECK(left.camera.farPlane ==
          Approx(right.camera.farPlane).margin(1.0e-4F));
    CHECK(left.camera.apertureFStops ==
          Approx(right.camera.apertureFStops).margin(1.0e-5F));
}

}  // namespace

TEST_CASE("Ray collection keeps points near the ray inside the range", "[camera][focus]") {
    const auto plane = PlaneOfPoints(10.0F, 0.4F, 0.1F);
    std::vector<float> distances;
    CollectRayHitDistancesAlongRay(
        plane,
        {0.0F, 0.0F, 0.0F},
        {1.0F, 0.0F, 0.0F},
        0.2F,
        0.25F,
        500.0F,
        &distances);
    REQUIRE(!distances.empty());
    for (const float distance : distances) {
        CHECK(distance == Approx(10.0F));
    }

    // Points beyond the perpendicular radius or outside the distance range
    // never contribute.
    std::vector<float> filtered;
    const std::vector<Float3> outliers{
        {5.0F, 0.5F, 0.0F},   // 0.5m off axis
        {-3.0F, 0.0F, 0.0F},  // behind the origin
        {0.1F, 0.0F, 0.0F},   // closer than the minimum distance
    };
    CollectRayHitDistancesAlongRay(
        outliers,
        {0.0F, 0.0F, 0.0F},
        {1.0F, 0.0F, 0.0F},
        0.2F,
        0.25F,
        500.0F,
        &filtered);
    CHECK(filtered.empty());
}

TEST_CASE("Cluster resolution ignores a stray point in front of a dense surface", "[camera][focus]") {
    std::vector<float> distances{3.0F};
    for (int i = 0; i < 20; ++i) {
        distances.push_back(10.0F + 0.01F * static_cast<float>(i));
    }
    const auto hit = ResolveFirstRayHitCluster(distances, 0.5F, 3U);
    REQUIRE(hit.has_value());
    CHECK(hit.value() == Approx(10.095F).margin(0.05F));
}

TEST_CASE("Cluster resolution falls back to the nearest sparse candidate", "[camera][focus]") {
    const auto hit = ResolveFirstRayHitCluster({7.5F}, 0.5F, 3U);
    REQUIRE(hit.has_value());
    CHECK(hit.value() == Approx(7.5F));

    CHECK_FALSE(ResolveFirstRayHitCluster({}, 0.5F, 3U).has_value());
}

TEST_CASE("Nearest qualifying cluster wins over a farther denser one", "[camera][focus]") {
    std::vector<float> distances;
    for (int i = 0; i < 4; ++i) {
        distances.push_back(6.0F + 0.05F * static_cast<float>(i));
    }
    for (int i = 0; i < 40; ++i) {
        distances.push_back(20.0F + 0.01F * static_cast<float>(i));
    }
    const auto hit = ResolveFirstRayHitCluster(distances, 0.5F, 3U);
    REQUIRE(hit.has_value());
    CHECK(hit.value() == Approx(6.075F).margin(0.05F));
}

TEST_CASE("Surface focus preserves the previous distance when the ray misses",
          "[camera][focus][key-editing]") {
    CHECK(ResolveSurfaceFocusDistance(std::nullopt, 7.25F) ==
          Approx(7.25F));
    CHECK(ResolveSurfaceFocusDistance(3.5F, 7.25F) == Approx(3.5F));
    CHECK(ResolveSurfaceFocusDistance(-2.0F, 7.25F) ==
          Approx(7.25F));
}

TEST_CASE("Matching-frame ghost keeps the linked view's front surface and freezes it at the destination pose",
          "[camera][animation][matching-frame-ghost]") {
    invisible_places::camera::AnimationPathEvaluation reference;
    reference.camera.position = {0.0F, 0.0F, 0.0F};
    reference.camera.orientation = {0.0F, 0.0F, 0.0F, 1.0F};
    reference.camera.fovDegrees = 60.0F;
    reference.camera.nearPlane = 0.01F;
    reference.camera.farPlane = 100.0F;

    invisible_places::camera::AnimationPathEvaluation destination = reference;
    destination.camera.position = {10.0F, 2.0F, 3.0F};
    constexpr float kSqrtHalf = 0.70710678118F;
    destination.camera.orientation = {
        0.0F,
        kSqrtHalf,
        0.0F,
        kSqrtHalf,
    };

    const std::vector<Float3> points{
        {0.0F, 0.0F, -2.0F},   // front centre surface
        {0.0F, 0.0F, -5.0F},   // occluded in the same screen cell
        {0.5F, 0.0F, -2.0F},   // front surface in another cell
        {10.0F, 0.0F, -2.0F},  // outside the reference frustum
        {0.0F, 0.0F, 1.0F},    // behind the reference camera
    };
    const auto ghost = invisible_places::camera::
        BuildAnimationMatchingFrameGhost(
            destination,
            reference,
            points,
            {
                .aspectRatio = 1.0F,
                .screenGridWidth = 8U,
                .screenGridHeight = 8U,
                .frontDepthToleranceMeters = 0.0F,
                .frontDepthToleranceFraction = 0.0F,
                .maximumPointSamples = points.size(),
            });

    INFO(ghost.errorMessage);
    REQUIRE(ghost.succeeded);
    CHECK(ghost.inputPointCount == points.size());
    CHECK(ghost.sampledPointCount == points.size());
    CHECK(ghost.frustumVisiblePointCount == 3U);
    REQUIRE(ghost.positions.size() == 2U);
    CHECK(ghost.positions[0U].x == Approx(8.0F));
    CHECK(ghost.positions[0U].y == Approx(2.0F));
    CHECK(ghost.positions[0U].z == Approx(3.0F));
    CHECK(ghost.positions[1U].x == Approx(8.0F));
    CHECK(ghost.positions[1U].y == Approx(2.0F));
    CHECK(ghost.positions[1U].z == Approx(2.5F));
}

TEST_CASE("Matching-frame ghost reports an empty resident scene without producing points",
          "[camera][animation][matching-frame-ghost]") {
    const invisible_places::camera::AnimationPathEvaluation evaluation;
    const auto ghost = invisible_places::camera::
        BuildAnimationMatchingFrameGhost(
            evaluation,
            evaluation,
            {});
    CHECK_FALSE(ghost.succeeded);
    CHECK(ghost.positions.empty());
    CHECK_FALSE(ghost.errorMessage.empty());
}

TEST_CASE("Matching-frame ghost honours background cancellation without publishing points",
          "[camera][animation][matching-frame-ghost]") {
    const invisible_places::camera::AnimationPathEvaluation evaluation;
    const std::vector<Float3> points{{0.0F, 0.0F, -2.0F}};
    std::stop_source cancellation;
    cancellation.request_stop();

    const auto ghost = invisible_places::camera::
        BuildAnimationMatchingFrameGhost(
            evaluation,
            evaluation,
            points,
            {.stopToken = cancellation.get_token()});

    CHECK_FALSE(ghost.succeeded);
    CHECK(ghost.positions.empty());
    CHECK(ghost.errorMessage.find("cancelled") != std::string::npos);
}

TEST_CASE("Matching camera-frame transform rigidly maps B controls into A space and is invertible",
          "[camera][animation][matching-frame-ghost]") {
    invisible_places::camera::AnimationPathEvaluation source;
    source.camera.position = {2.0F, 3.0F, 4.0F};
    source.camera.orientation = {0.0F, 0.0F, 0.0F, 1.0F};
    invisible_places::camera::AnimationPathEvaluation destination;
    destination.camera.position = {10.0F, 20.0F, 30.0F};
    constexpr float kSqrtHalf = 0.70710678118F;
    destination.camera.orientation = {
        0.0F,
        kSqrtHalf,
        0.0F,
        kSqrtHalf,
    };

    const auto transform = invisible_places::camera::
        BuildAnimationCameraFrameTransform(destination, source);
    const auto mappedCamera = invisible_places::camera::
        ApplyAnimationCameraFrameTransform(
            transform,
            source.camera.position);
    CHECK(mappedCamera[0U] == Approx(10.0F));
    CHECK(mappedCamera[1U] == Approx(20.0F));
    CHECK(mappedCamera[2U] == Approx(30.0F));

    const std::array<float, 3> sourceControl{3.0F, 5.0F, 1.0F};
    const auto mappedControl = invisible_places::camera::
        ApplyAnimationCameraFrameTransform(transform, sourceControl);
    CHECK(mappedControl[0U] == Approx(7.0F));
    CHECK(mappedControl[1U] == Approx(22.0F));
    CHECK(mappedControl[2U] == Approx(29.0F));
    const auto restoredControl = invisible_places::camera::
        InvertAnimationCameraFrameTransform(transform, mappedControl);
    CHECK(restoredControl[0U] == Approx(sourceControl[0U]));
    CHECK(restoredControl[1U] == Approx(sourceControl[1U]));
    CHECK(restoredControl[2U] == Approx(sourceControl[2U]));
}

TEST_CASE("Animation key insertion splits the exact playhead segment without retiming later keys",
          "[camera][animation][key-editing]") {
    invisible_places::camera::AnimationPath path;
    path.durationFrames = 100U;
    path.keys.resize(3U);
    path.keys[0U].id = "A";
    path.keys[1U].id = "B";
    path.keys[1U].durationFrames = 40U;
    path.keys[2U].id = "C";
    path.keys[2U].durationFrames = 60U;

    invisible_places::camera::AnimationPathKey inserted;
    inserted.id = "B";
    inserted.cameraPosition = {2.5F, 0.0F, 0.0F};
    std::string error;
    const auto insertedIndex = invisible_places::camera::
        InsertAnimationPathKeyAtFrame(
            &path,
            std::move(inserted),
            25U,
            &error);

    INFO(error);
    REQUIRE(insertedIndex == 1U);
    REQUIRE(path.keys.size() == 4U);
    CHECK(path.durationFrames == 100U);
    CHECK(path.keys[1U].id == "B_1");
    CHECK(path.keys[1U].durationFrames == 25U);
    CHECK(path.keys[2U].durationFrames == 15U);
    CHECK(path.keys[3U].durationFrames == 60U);
    CHECK(
        invisible_places::camera::AnimationPathKeyNormalizedPosition(
            path,
            1U) == Approx(0.25F));
    CHECK(
        invisible_places::camera::AnimationPathKeyNormalizedPosition(
            path,
            2U) == Approx(0.40F));
}

TEST_CASE("Removing an interior animation key merges timing and clears key metadata",
          "[camera][animation][key-editing]") {
    invisible_places::camera::AnimationPath path;
    path.durationFrames = 100U;
    path.keys.resize(4U);
    path.keys[0U].id = "A";
    path.keys[1U].id = "inserted";
    path.keys[1U].durationFrames = 25U;
    path.keys[2U].id = "B";
    path.keys[2U].durationFrames = 15U;
    path.keys[3U].id = "C";
    path.keys[3U].durationFrames = 60U;
    path.localizedKeyCorrections.push_back({.keyId = "inserted"});
    path.velocityBlendLink =
        invisible_places::camera::AnimationVelocityBlendLinkMetadata{};
    path.velocityBlendLink->movableKeyIds = {"inserted", "C"};

    std::string error;
    REQUIRE(invisible_places::camera::RemoveAnimationPathKey(
        &path,
        1U,
        &error));
    INFO(error);
    REQUIRE(path.keys.size() == 3U);
    CHECK(path.durationFrames == 100U);
    CHECK(path.keys[1U].id == "B");
    CHECK(path.keys[1U].durationFrames == 40U);
    CHECK(path.keys[2U].durationFrames == 60U);
    CHECK(path.localizedKeyCorrections.empty());
    REQUIRE(path.velocityBlendLink.has_value());
    CHECK(path.velocityBlendLink->movableKeyIds ==
          std::vector<std::string>{"C"});
}

TEST_CASE("Dragging animation keys changes pose order while retaining timing slots",
          "[camera][animation][key-editing]") {
    invisible_places::camera::AnimationPath path;
    path.durationFrames = 100U;
    path.keys.resize(3U);
    path.keys[0U].id = "A";
    path.keys[1U].id = "B";
    path.keys[1U].durationFrames = 40U;
    path.keys[2U].id = "C";
    path.keys[2U].durationFrames = 60U;

    REQUIRE(invisible_places::camera::ReorderAnimationPathKey(
        &path,
        0U,
        2U));
    CHECK(path.keys[0U].id == "B");
    CHECK(path.keys[1U].id == "C");
    CHECK(path.keys[2U].id == "A");
    CHECK(path.keys[1U].durationFrames == 40U);
    CHECK(path.keys[2U].durationFrames == 60U);
    CHECK(
        invisible_places::camera::AnimationPathKeyNormalizedPosition(
            path,
            1U) == Approx(0.40F));
    CHECK(path.durationFrames == 100U);
}

TEST_CASE("Resetting animation timing weights clears retiming retained by key edits",
          "[camera][animation][key-editing][timing-reset]") {
    invisible_places::camera::AnimationPath path;
    path.durationFrames = 100U;
    path.keys.resize(3U);
    path.keys[0U].id = "A";
    path.keys[1U].id = "B";
    path.keys[1U].durationFrames = 40U;
    path.keys[2U].id = "C";
    path.keys[2U].durationFrames = 60U;

    invisible_places::camera::AnimationPathKey inserted;
    inserted.id = "inserted";
    REQUIRE(invisible_places::camera::InsertAnimationPathKeyAtFrame(
                &path,
                std::move(inserted),
                25U)
                .has_value());
    REQUIRE(invisible_places::camera::RemoveAnimationPathKey(
        &path,
        1U));
    REQUIRE(path.keys.size() == 3U);
    CHECK(path.keys[1U].durationFrames == 40U);
    CHECK(path.keys[2U].durationFrames == 60U);

    REQUIRE(invisible_places::camera::ResetAnimationPathTimingWeights(
        &path));
    CHECK(path.durationFrames == 100U);
    CHECK(path.keys[1U].durationFrames == 50U);
    CHECK(path.keys[2U].durationFrames == 50U);
    CHECK_FALSE(invisible_places::camera::ResetAnimationPathTimingWeights(
        &path));
}

TEST_CASE("Reset timing uses distance geometry and C2 traversal without endpoint stop easing",
          "[camera][animation][timing-reset]") {
    invisible_places::camera::AnimationPath path;
    path.durationFrames = 90U;
    path.keys = {
        {.id = "A",
         .cameraPosition = {0.0F, 0.0F, 0.0F},
         .focusPoint = {0.0F, 1.0F, 0.0F},
         .durationFrames = 90U},
        {.id = "B",
         .cameraPosition = {1.0F, 2.0F, 0.5F},
         .focusPoint = {2.0F, 2.5F, 0.5F},
         .durationFrames = 8U},
        {.id = "C",
         .cameraPosition = {4.0F, -1.0F, 1.0F},
         .focusPoint = {4.5F, -0.5F, 1.5F},
         .durationFrames = 70U},
        {.id = "D",
         .cameraPosition = {7.0F, 1.0F, 0.0F},
         .focusPoint = {7.0F, 1.0F, 1.0F},
         .durationFrames = 12U},
    };
    const auto originalKeys = path.keys;

    REQUIRE(invisible_places::camera::ResetAnimationPathTimingWeights(
        &path));
    REQUIRE(path.keys.size() == originalKeys.size());
    for (std::size_t index = 0U; index < path.keys.size(); ++index) {
        CHECK(path.keys[index].cameraPosition ==
              originalKeys[index].cameraPosition);
        CHECK(path.keys[index].focusPoint ==
              originalKeys[index].focusPoint);
        if (index > 0U) {
            CHECK(path.keys[index].durationFrames == 30U);
        }
    }

    const auto coordinateAt = [&path](
                                  float seconds,
                                  bool focus,
                                  std::size_t component) {
        const auto evaluation = invisible_places::camera::
            EvaluateAnimationPath(path, seconds);
        return focus
            ? evaluation.focusPoint[component]
            : evaluation.camera.position[component];
    };
    constexpr float kStepSeconds = 0.005F;
    constexpr float kInteriorKnotSeconds = 1.0F;
    for (const bool focus : {false, true}) {
        float endpointSpeedSquared = 0.0F;
        for (std::size_t component = 0U; component < 3U; ++component) {
            const float startVelocity =
                (coordinateAt(kStepSeconds, focus, component) -
                 coordinateAt(0.0F, focus, component)) /
                kStepSeconds;
            endpointSpeedSquared += startVelocity * startVelocity;

            const float velocityIn =
                (coordinateAt(kInteriorKnotSeconds, focus, component) -
                 coordinateAt(
                     kInteriorKnotSeconds - kStepSeconds,
                     focus,
                     component)) /
                kStepSeconds;
            const float velocityOut =
                (coordinateAt(
                     kInteriorKnotSeconds + kStepSeconds,
                     focus,
                     component) -
                 coordinateAt(kInteriorKnotSeconds, focus, component)) /
                kStepSeconds;
            const float accelerationIn =
                (coordinateAt(kInteriorKnotSeconds, focus, component) -
                 2.0F * coordinateAt(
                            kInteriorKnotSeconds - kStepSeconds,
                            focus,
                            component) +
                 coordinateAt(
                     kInteriorKnotSeconds - 2.0F * kStepSeconds,
                     focus,
                     component)) /
                (kStepSeconds * kStepSeconds);
            const float accelerationOut =
                (coordinateAt(
                     kInteriorKnotSeconds + 2.0F * kStepSeconds,
                     focus,
                     component) -
                 2.0F * coordinateAt(
                            kInteriorKnotSeconds + kStepSeconds,
                            focus,
                            component) +
                 coordinateAt(kInteriorKnotSeconds, focus, component)) /
                (kStepSeconds * kStepSeconds);

            CHECK(velocityOut == Approx(velocityIn).margin(0.08F));
            CHECK(accelerationOut == Approx(accelerationIn).margin(0.8F));
        }
        CHECK(endpointSpeedSquared > 1.0e-4F);
    }
}

TEST_CASE("Timing reset cannot bend camera or focus geometry and bakes stale corrections",
          "[camera][animation][key-editing][timing-reset][smooth-spline]") {
    invisible_places::camera::AnimationPath path;
    path.durationFrames = 90U;
    path.keys = {
        {.id = "A",
         .cameraPosition = {0.0F, 0.0F, 0.0F},
         .focusPoint = {0.0F, -2.0F, 0.0F},
         .durationFrames = 90U},
        {.id = "B",
         .cameraPosition = {1.0F, 0.4F, 0.2F},
         .focusPoint = {0.8F, -1.8F, 0.1F},
         .durationFrames = 8U},
        {.id = "C",
         .cameraPosition = {8.0F, -3.0F, 1.0F},
         .focusPoint = {9.0F, 4.0F, 1.5F},
         .durationFrames = 82U},
    };
    const auto before = invisible_places::camera::
        PrepareAnimationPathEvaluation(path);
    REQUIRE(before.valid);

    REQUIRE(invisible_places::camera::ResetAnimationPathTimingWeights(
        &path));
    const auto after = invisible_places::camera::
        PrepareAnimationPathEvaluation(path);
    REQUIRE(after.valid);
    CHECK(after.geometryKnots == before.geometryKnots);
    CHECK(after.cameraX.values == before.cameraX.values);
    CHECK(after.cameraX.secondDerivatives ==
          before.cameraX.secondDerivatives);
    CHECK(after.cameraY.secondDerivatives ==
          before.cameraY.secondDerivatives);
    CHECK(after.focusX.secondDerivatives ==
          before.focusX.secondDerivatives);
    CHECK(after.focusY.secondDerivatives ==
          before.focusY.secondDerivatives);

    constexpr float kProbeSeconds = 1.0F / 300.0F;
    const auto start = invisible_places::camera::EvaluateAnimationPath(
        path,
        0.0F);
    const auto next = invisible_places::camera::EvaluateAnimationPath(
        path,
        kProbeSeconds);
    const auto forwardDot = [](const auto& movement, const auto& chord) {
        return movement[0U] * chord[0U] +
               movement[1U] * chord[1U] +
               movement[2U] * chord[2U];
    };
    const auto difference = [](const auto& to, const auto& from) {
        return std::array<float, 3>{
            to[0U] - from[0U],
            to[1U] - from[1U],
            to[2U] - from[2U],
        };
    };
    CHECK(forwardDot(
              difference(next.camera.position, start.camera.position),
              difference(
                  path.keys[1U].cameraPosition,
                  path.keys[0U].cameraPosition)) > 0.0F);
    CHECK(forwardDot(
              difference(next.focusPoint, start.focusPoint),
              difference(
                  path.keys[1U].focusPoint,
                  path.keys[0U].focusPoint)) > 0.0F);

    path.localizedKeyCorrections.push_back({
        .keyId = "A",
        .splineCameraPosition = path.keys[0U].cameraPosition,
        .splineFocusPoint = path.keys[0U].focusPoint,
    });
    path.keys[0U].cameraPosition[2U] += 0.25F;
    const auto adjustedPose = path.keys[0U].cameraPosition;
    REQUIRE(invisible_places::camera::ResetAnimationPathTimingWeights(
        &path));
    CHECK(path.localizedKeyCorrections.empty());
    CHECK(path.keys[0U].cameraPosition == adjustedPose);
}

TEST_CASE("Inserted then promoted endpoint travels toward its next camera and focus keys",
          "[camera][animation][key-editing][smooth-spline]") {
    invisible_places::camera::AnimationPath path;
    path.durationFrames = 120U;
    path.keys = {
        {.id = "old-start",
         .cameraPosition = {-3.0F, -1.0F, 0.0F},
         .focusPoint = {-2.0F, 1.0F, 0.5F},
         .durationFrames = 20U},
        {.id = "next",
         .cameraPosition = {1.0F, 0.0F, 0.4F},
         .focusPoint = {1.5F, 1.2F, 0.8F},
         .durationFrames = 35U},
        {.id = "far",
         .cameraPosition = {11.0F, -5.0F, 1.5F},
         .focusPoint = {13.0F, 7.0F, 2.0F},
         .durationFrames = 65U},
    };
    path.localizedKeyCorrections.push_back({
        .keyId = "next",
        .splineCameraPosition = path.keys[1U].cameraPosition,
        .splineFocusPoint = path.keys[1U].focusPoint,
    });
    path.keys[1U].cameraPosition[1U] += 0.2F;
    path.keys[1U].focusPoint[2U] += 0.15F;
    REQUIRE(invisible_places::camera::
                BakeAnimationPathLocalizedCorrections(&path));

    constexpr std::uint32_t kInsertFrame = 10U;
    const auto beforeInsertion = invisible_places::camera::
        PrepareAnimationPathEvaluation(path);
    const auto sampled = invisible_places::camera::EvaluateAnimationPath(
        path,
        static_cast<float>(kInsertFrame) / 30.0F);
    invisible_places::camera::AnimationPathKey inserted{
        .id = "inserted",
        .cameraPosition = sampled.camera.position,
        .focusPoint = sampled.focusPoint,
    };
    const auto insertedIndex = invisible_places::camera::
        InsertAnimationPathKeyAtFrame(
            &path,
            std::move(inserted),
            kInsertFrame);
    REQUIRE(insertedIndex == 1U);
    const auto afterInsertion = invisible_places::camera::
        PrepareAnimationPathEvaluation(path);
    REQUIRE(afterInsertion.geometryKnots.size() ==
            beforeInsertion.geometryKnots.size() + 1U);
    CHECK(afterInsertion.geometryKnots.front() ==
          Approx(beforeInsertion.geometryKnots.front()));
    for (std::size_t oldIndex = 1U;
         oldIndex < beforeInsertion.geometryKnots.size();
         ++oldIndex) {
        CHECK(afterInsertion.geometryKnots[oldIndex + 1U] ==
              Approx(beforeInsertion.geometryKnots[oldIndex]));
    }
    // A cubic already passing through the inserted evaluated point is the
    // unique clamped C2 solution on the split knot set. Matching second
    // derivatives at every original knot verifies that adding the key did
    // not reshape either spatial track.
    const auto checkOriginalSplineDerivatives = [](
        const invisible_places::camera::AnimationPreparedScalarSpline& before,
        const invisible_places::camera::AnimationPreparedScalarSpline& after) {
        REQUIRE(after.secondDerivatives.size() ==
                before.secondDerivatives.size() + 1U);
        CHECK(after.secondDerivatives.front() ==
              Approx(before.secondDerivatives.front()).margin(2.0e-4F));
        for (std::size_t oldIndex = 1U;
             oldIndex < before.secondDerivatives.size();
             ++oldIndex) {
            CHECK(after.secondDerivatives[oldIndex + 1U] ==
                  Approx(before.secondDerivatives[oldIndex]).margin(2.0e-4F));
        }
    };
    checkOriginalSplineDerivatives(
        beforeInsertion.cameraX,
        afterInsertion.cameraX);
    checkOriginalSplineDerivatives(
        beforeInsertion.cameraY,
        afterInsertion.cameraY);
    checkOriginalSplineDerivatives(
        beforeInsertion.cameraZ,
        afterInsertion.cameraZ);
    checkOriginalSplineDerivatives(
        beforeInsertion.focusX,
        afterInsertion.focusX);
    checkOriginalSplineDerivatives(
        beforeInsertion.focusY,
        afterInsertion.focusY);
    checkOriginalSplineDerivatives(
        beforeInsertion.focusZ,
        afterInsertion.focusZ);
    CHECK(path.keys[1U].splineParameterWeight > 0.0F);
    CHECK(path.keys[2U].splineParameterWeight > 0.0F);
    REQUIRE(invisible_places::camera::RemoveAnimationPathKey(
        &path,
        0U));
    REQUIRE(path.keys.front().id == "inserted");
    CHECK(path.localizedKeyCorrections.empty());

    const auto start = invisible_places::camera::EvaluateAnimationPath(
        path,
        0.0F);
    const auto justAfter = invisible_places::camera::EvaluateAnimationPath(
        path,
        1.0F / 300.0F);
    const auto movesToward = [](const auto& from,
                                const auto& to,
                                const auto& nextKey) {
        float dot = 0.0F;
        for (std::size_t component = 0U; component < 3U; ++component) {
            dot += (to[component] - from[component]) *
                   (nextKey[component] - from[component]);
        }
        return dot;
    };
    CHECK(movesToward(
              start.camera.position,
              justAfter.camera.position,
              path.keys[1U].cameraPosition) > 0.0F);
    CHECK(movesToward(
              start.focusPoint,
              justAfter.focusPoint,
              path.keys[1U].focusPoint) > 0.0F);
}

TEST_CASE("Explicit orientation has continuous angular velocity and acceleration at keys",
          "[camera][animation][orientation][smooth-spline]") {
    invisible_places::camera::AnimationPath path;
    path.durationFrames = 120U;
    const auto orientationAroundZ = [](float degrees) {
        const float halfRadians =
            0.5F * degrees * 3.14159265358979323846F / 180.0F;
        return std::array<float, 4>{
            0.0F,
            0.0F,
            std::sin(halfRadians),
            std::cos(halfRadians),
        };
    };
    path.keys = {
        {.id = "A",
         .cameraPosition = {0.0F, 0.0F, 0.0F},
         .focusPoint = {0.0F, 0.0F, -4.0F},
         .hasOrientation = true,
         .orientation = orientationAroundZ(0.0F),
         .durationFrames = 20U},
        {.id = "B",
         .cameraPosition = {1.0F, 0.2F, 0.0F},
         .focusPoint = {1.0F, 0.2F, -4.0F},
         .hasOrientation = true,
         .orientation = orientationAroundZ(25.0F),
         .durationFrames = 25U},
        {.id = "C",
         .cameraPosition = {5.0F, 1.0F, 0.5F},
         .focusPoint = {5.0F, 1.0F, -3.5F},
         .hasOrientation = true,
         .orientation = orientationAroundZ(105.0F),
         .durationFrames = 55U},
        {.id = "D",
         .cameraPosition = {8.0F, 2.0F, 1.0F},
         .focusPoint = {8.0F, 2.0F, -3.0F},
         .hasOrientation = true,
         .orientation = orientationAroundZ(145.0F),
         .durationFrames = 40U},
    };
    const auto context = invisible_places::camera::
        PrepareAnimationPathEvaluation(path);
    REQUIRE(context.valid);
    REQUIRE(context.knots.size() == path.keys.size());
    const float knot = context.knots[1U];
    constexpr float kStep = 0.004F;
    const auto orientationAt = [&](float seconds) {
        return invisible_places::camera::EvaluatePreparedAnimationPath(
                   context,
                   seconds)
            .camera.orientation;
    };
    const auto centre = orientationAt(knot);
    const auto aligned = [&](std::array<float, 4> value) {
        float dot = 0.0F;
        for (std::size_t component = 0U; component < 4U; ++component) {
            dot += value[component] * centre[component];
        }
        if (dot < 0.0F) {
            for (auto& component : value) {
                component = -component;
            }
        }
        return value;
    };
    const auto minusTwo = aligned(orientationAt(knot - 2.0F * kStep));
    const auto minusOne = aligned(orientationAt(knot - kStep));
    const auto plusOne = aligned(orientationAt(knot + kStep));
    const auto plusTwo = aligned(orientationAt(knot + 2.0F * kStep));
    for (std::size_t component = 0U; component < 4U; ++component) {
        const float velocityIn =
            (centre[component] - minusOne[component]) / kStep;
        const float velocityOut =
            (plusOne[component] - centre[component]) / kStep;
        const float accelerationIn =
            (centre[component] - 2.0F * minusOne[component] +
             minusTwo[component]) /
            (kStep * kStep);
        const float accelerationOut =
            (plusTwo[component] - 2.0F * plusOne[component] +
             centre[component]) /
            (kStep * kStep);
        CHECK(velocityOut == Approx(velocityIn).margin(0.03F));
        CHECK(accelerationOut == Approx(accelerationIn).margin(0.35F));
    }
}

TEST_CASE("Focus-relative rig alignment rotates one matching camera into the destination path frame",
          "[camera][animation][camera-rig-alignment]") {
    invisible_places::camera::AnimationPath reference;
    reference.durationFrames = 60U;
    reference.keys = {
        {.id = "reference-a",
         .cameraPosition = {-2.0F, -6.0F, 3.0F},
         .focusPoint = {0.0F, 0.0F, 0.0F},
         .durationFrames = 30U},
        {.id = "reference-b",
         .cameraPosition = {8.0F, -6.0F, 3.0F},
         .focusPoint = {10.0F, 0.0F, 0.0F},
         .durationFrames = 30U},
        {.id = "reference-c",
         .cameraPosition = {18.0F, -6.0F, 3.0F},
         .focusPoint = {20.0F, 0.0F, 0.0F},
         .durationFrames = 30U},
    };
    invisible_places::camera::AnimationPath destination;
    destination.durationFrames = 60U;
    destination.keys = {
        {.id = "destination-a",
         .cameraPosition = {100.0F, -4.0F, 11.0F},
         .focusPoint = {100.0F, 0.0F, 10.0F},
         .durationFrames = 30U},
        {.id = "destination-b",
         .cameraPosition = {100.0F, 6.0F, 11.0F},
         .focusPoint = {100.0F, 10.0F, 10.0F},
         .durationFrames = 30U},
        {.id = "destination-c",
         .cameraPosition = {100.0F, 16.0F, 11.0F},
         .focusPoint = {100.0F, 20.0F, 10.0F},
         .durationFrames = 30U},
    };
    const auto originalReference = reference;
    const auto originalDestination = destination;

    const auto result = invisible_places::camera::
        AlignAnimationKeyCameraToReferenceRig(
            &destination,
            reference,
            {
                .destinationKeyId = "destination-b",
                .referenceNormalizedPosition = 0.5F,
            });

    INFO(result.errorMessage);
    REQUIRE(result.succeeded);
    REQUIRE(result.changed);
    CHECK(result.referenceFocusDistance == Approx(7.0F));
    CHECK(result.referenceAlongPathOffset == Approx(-2.0F));
    CHECK(result.referenceLateralOffset == Approx(6.0F));
    CHECK(result.referenceHeightOffset == Approx(3.0F));
    CHECK(destination.keys[1U].cameraPosition[0U] == Approx(106.0F));
    CHECK(destination.keys[1U].cameraPosition[1U] == Approx(8.0F));
    CHECK(destination.keys[1U].cameraPosition[2U] == Approx(13.0F));
    CHECK(destination.keys[1U].focusPoint ==
          originalDestination.keys[1U].focusPoint);
    CHECK(destination.keys[0U].cameraPosition ==
          originalDestination.keys[0U].cameraPosition);
    CHECK(destination.keys[2U].cameraPosition ==
          originalDestination.keys[2U].cameraPosition);
    REQUIRE(destination.localizedKeyCorrections.size() == 1U);
    CHECK(destination.localizedKeyCorrections.front().keyId ==
          "destination-b");
    CHECK(destination.localizedKeyCorrections.front().splineCameraPosition ==
          originalDestination.keys[1U].cameraPosition);
    CHECK(reference.keys[0U].cameraPosition ==
          originalReference.keys[0U].cameraPosition);
    CHECK(reference.keys[1U].cameraPosition ==
          originalReference.keys[1U].cameraPosition);
    CHECK(reference.keys[2U].cameraPosition ==
          originalReference.keys[2U].cameraPosition);
}

TEST_CASE("Focus-relative rig alignment rejects paths without a travel direction",
          "[camera][animation][camera-rig-alignment]") {
    invisible_places::camera::AnimationPath destination;
    destination.durationFrames = 30U;
    destination.keys = {
        {.id = "destination",
         .cameraPosition = {0.0F, -5.0F, 2.0F},
         .focusPoint = {0.0F, 0.0F, 0.0F},
         .durationFrames = 30U},
        {.id = "destination-end",
         .cameraPosition = {0.0F, -5.0F, 2.0F},
         .focusPoint = {0.0F, 0.0F, 0.0F},
         .durationFrames = 30U},
    };
    auto reference = destination;
    reference.keys[0U].id = "reference";
    reference.keys[1U].id = "reference-end";
    const auto originalCamera = destination.keys[0U].cameraPosition;

    const auto result = invisible_places::camera::
        AlignAnimationKeyCameraToReferenceRig(
            &destination,
            reference,
            {
                .destinationKeyId = "destination",
                .referenceNormalizedPosition = 0.0F,
            });

    CHECK_FALSE(result.succeeded);
    CHECK_FALSE(result.changed);
    CHECK_FALSE(result.errorMessage.empty());
    CHECK(destination.keys[0U].cameraPosition == originalCamera);
    CHECK(destination.localizedKeyCorrections.empty());
}

TEST_CASE(
    "Pair clip normalization preserves animation motion and keeps the widest visible range",
    "[camera][animation][clip-planes][pan-extension]") {
    auto first = MakeReciprocalPanTestPath("A", 1.0F, 41U, 79U);
    auto second = MakeReciprocalPanTestPath("B", -1.0F, 54U, 96U);
    first.keys[0U].nearPlane = 0.020F;
    first.keys[1U].nearPlane = 0.010F;
    first.keys[2U].nearPlane = 0.015F;
    first.keys[0U].farPlane = 90.0F;
    first.keys[1U].farPlane = 120.0F;
    first.keys[2U].farPlane = 100.0F;
    second.keys[0U].nearPlane = 0.005F;
    second.keys[1U].nearPlane = 0.008F;
    second.keys[2U].nearPlane = 0.006F;
    second.keys[0U].farPlane = 80.0F;
    second.keys[1U].farPlane = 150.0F;
    second.keys[2U].farPlane = 130.0F;
    first.keys.front().hasSplineEndpointTangent = true;
    first.keys.front().splineLensEndpointTangent = {
        0.25F,
        0.01F,
        -2.0F,
        0.5F,
        0.75F,
    };
    first.keys.back().hasSplineEndpointTangent = true;
    first.keys.back().splineLensEndpointTangent = {
        -0.20F,
        -0.01F,
        3.0F,
        -0.4F,
        -0.6F,
    };
    first.keys.front().linkedCameraId = "camera-a";
    first.keys.back().linkedCameraId = "camera-a";
    second.keys[1U].linkedCameraId = "camera-b";

    const auto originalFirst = first;
    const auto originalSecond = second;
    const auto firstBefore = invisible_places::camera::
        PrepareAnimationPathEvaluation(first);
    const auto secondBefore = invisible_places::camera::
        PrepareAnimationPathEvaluation(second);
    REQUIRE(firstBefore.valid);
    REQUIRE(secondBefore.valid);

    const auto result = invisible_places::camera::
        BuildConservativeAnimationClipPlaneNormalization(first, second);
    INFO(result.errorMessage);
    REQUIRE(result.succeeded);
    REQUIRE(result.changed);
    CHECK(result.nearPlane == Approx(0.005F));
    CHECK(result.farPlane == Approx(150.0F));
    CHECK(result.linkedCameraIds ==
          std::vector<std::string>{"camera-a", "camera-b"});
    CHECK(first.keys.front().nearPlane ==
          Approx(originalFirst.keys.front().nearPlane));
    CHECK(second.keys[1U].farPlane ==
          Approx(originalSecond.keys[1U].farPlane));

    for (const auto* path :
         {&result.firstCandidate, &result.secondCandidate}) {
        for (const auto& key : path->keys) {
            CHECK(key.nearPlane == Approx(0.005F));
            CHECK(key.farPlane == Approx(150.0F));
            if (key.hasSplineEndpointTangent) {
                CHECK(key.splineLensEndpointTangent[1U] == Approx(0.0F));
                CHECK(key.splineLensEndpointTangent[2U] == Approx(0.0F));
            }
        }
    }
    CHECK(result.firstCandidate.durationFrames == first.durationFrames);
    CHECK(result.firstCandidate.exportSettings.startFrame ==
          first.exportSettings.startFrame);
    CHECK(result.firstCandidate.exportSettings.endFrame ==
          first.exportSettings.endFrame);
    CHECK(result.firstCandidate.keys.front().splineLensEndpointTangent[0U] ==
          Approx(0.25F));
    CHECK(result.firstCandidate.keys.front().splineLensEndpointTangent[3U] ==
          Approx(0.5F));
    CHECK(result.firstCandidate.keys.front().splineLensEndpointTangent[4U] ==
          Approx(0.75F));
    CHECK(result.firstCandidate.keys.back().splineLensEndpointTangent[0U] ==
          Approx(-0.20F));
    CHECK(result.firstCandidate.keys.back().splineLensEndpointTangent[3U] ==
          Approx(-0.4F));
    CHECK(result.firstCandidate.keys.back().splineLensEndpointTangent[4U] ==
          Approx(-0.6F));

    const auto firstAfter = invisible_places::camera::
        PrepareAnimationPathEvaluation(result.firstCandidate);
    const auto secondAfter = invisible_places::camera::
        PrepareAnimationPathEvaluation(result.secondCandidate);
    REQUIRE(firstAfter.valid);
    REQUIRE(secondAfter.valid);
    for (std::uint32_t sample = 0U; sample <= 64U; ++sample) {
        const float amount = static_cast<float>(sample) / 64.0F;
        for (const auto& [before, after] :
             {std::pair{&firstBefore, &firstAfter},
              std::pair{&secondBefore, &secondAfter}}) {
            const auto beforeValue = invisible_places::camera::
                EvaluatePreparedAnimationPath(
                    *before,
                    before->durationSeconds * amount);
            const auto afterValue = invisible_places::camera::
                EvaluatePreparedAnimationPath(
                    *after,
                    after->durationSeconds * amount);
            CHECK(afterValue.camera.position == beforeValue.camera.position);
            CHECK(afterValue.focusPoint == beforeValue.focusPoint);
            CHECK(afterValue.camera.fovDegrees ==
                  Approx(beforeValue.camera.fovDegrees).margin(1.0e-6F));
            CHECK(afterValue.camera.nearPlane == Approx(0.005F));
            CHECK(afterValue.camera.farPlane == Approx(150.0F));
        }
    }

    const auto alreadyNormalized = invisible_places::camera::
        BuildConservativeAnimationClipPlaneNormalization(
            result.firstCandidate,
            result.secondCandidate);
    REQUIRE(alreadyNormalized.succeeded);
    CHECK_FALSE(alreadyNormalized.changed);

    auto invalid = first;
    invalid.keys[1U].farPlane = invalid.keys[1U].nearPlane;
    const auto invalidResult = invisible_places::camera::
        BuildConservativeAnimationClipPlaneNormalization(invalid, second);
    CHECK_FALSE(invalidResult.succeeded);
    CHECK_FALSE(invalidResult.changed);
    CHECK(invalidResult.firstCandidate.keys.empty());
    CHECK(invalid.keys[1U].farPlane ==
          Approx(first.keys[1U].nearPlane));
}

TEST_CASE("Reciprocal pan extension appends a short unlinked tail without retiming the old prefix",
          "[camera][animation][pan-extension]") {
    auto first = MakeReciprocalPanTestPath("A", 1.0F, 41U, 79U);
    auto second = MakeReciprocalPanTestPath("B", -1.0F, 54U, 96U);
    AddLegacyWaterTracks(&first);
    AddLegacyWaterTracks(&second);
    first.keys.back().sourceShotName = "terminal shot";
    first.keys.back().linkedCameraId = "camera-a";
    first.keys.back().linkedCameraName = "Camera A";
    second.keys.back().sourceShotName = "partner terminal shot";
    second.keys.back().linkedCameraId = "camera-b";
    second.keys.back().linkedCameraName = "Camera B";

    const auto middleSplineCamera = first.keys[1U].cameraPosition;
    const auto middleSplineFocus = first.keys[1U].focusPoint;
    first.keys[1U].cameraPosition[0U] += 0.12F;
    first.keys[1U].focusPoint[0U] += 0.04F;
    first.localizedKeyCorrections.push_back({
        .keyId = first.keys[1U].id,
        .splineCameraPosition = middleSplineCamera,
        .splineFocusPoint = middleSplineFocus,
    });

    const auto firstBefore = invisible_places::camera::
        PrepareAnimationPathEvaluation(first);
    const auto secondBefore = invisible_places::camera::
        PrepareAnimationPathEvaluation(second);
    REQUIRE(firstBefore.valid);
    REQUIRE(secondBefore.valid);
    const auto result = invisible_places::camera::
        BuildAnimationReciprocalPanExtension(
            first,
            second,
            MakeReciprocalPanTestOptions());

    INFO(result.errorMessage);
    REQUIRE(result.succeeded);
    REQUIRE(result.changed);
    REQUIRE(result.firstCandidate.keys.size() == first.keys.size() + 2U);
    REQUIRE(result.secondCandidate.keys.size() == second.keys.size() + 2U);
    CHECK(result.firstCandidate.durationFrames == 164U);
    CHECK(result.secondCandidate.durationFrames == 185U);
    CHECK(result.metrics.extensionFrames[0U] == 44U);
    CHECK(result.metrics.extensionFrames[1U] == 35U);
    CHECK(result.metrics.appendedKeyCount[0U] == 2U);
    CHECK(result.metrics.appendedKeyCount[1U] == 2U);
    CHECK(result.metrics.sourceInteriorKeyCount[0U] == 0U);
    CHECK(result.metrics.sourceInteriorKeyCount[1U] == 0U);
    CHECK(result.firstCandidate.authoredTrackDurationFrames == 0U);
    CHECK(result.secondCandidate.authoredTrackDurationFrames == 0U);
    CheckLegacyWaterTracksRetimed(
        result.firstCandidate,
        120.0F / 164.0F);
    CheckLegacyWaterTracksRetimed(
        result.secondCandidate,
        150.0F / 185.0F);

    CHECK(result.firstCandidate.keys[1U].durationFrames == 41U);
    CHECK(result.firstCandidate.keys[2U].durationFrames == 79U);
    CHECK(result.firstCandidate.keys[3U].durationFrames == 22U);
    CHECK(result.firstCandidate.keys[4U].durationFrames == 22U);
    CHECK(result.secondCandidate.keys[1U].durationFrames == 54U);
    CHECK(result.secondCandidate.keys[2U].durationFrames == 96U);
    CHECK(result.secondCandidate.keys[3U].durationFrames == 18U);
    CHECK(result.secondCandidate.keys[4U].durationFrames == 17U);
    CHECK(result.firstCandidate.exportSettings.startFrame == 7U);
    CHECK(result.firstCandidate.exportSettings.endFrame == 91U);
    CHECK(result.secondCandidate.exportSettings.startFrame == 7U);
    CHECK(result.secondCandidate.exportSettings.endFrame == 91U);

    for (std::size_t index = first.keys.size();
         index < result.firstCandidate.keys.size();
         ++index) {
        CHECK(result.firstCandidate.keys[index].sourceShotName.empty());
        CHECK(result.firstCandidate.keys[index].linkedCameraId.empty());
        CHECK(result.firstCandidate.keys[index].linkedCameraName.empty());
        CHECK(result.firstCandidate.keys[index].id != first.keys.back().id);
    }
    std::unordered_set<std::string> firstAppendedIds;
    for (std::size_t index = first.keys.size();
         index < result.firstCandidate.keys.size();
         ++index) {
        CHECK(firstAppendedIds.insert(
                  result.firstCandidate.keys[index].id).second);
    }
    for (std::size_t index = second.keys.size();
         index < result.secondCandidate.keys.size();
         ++index) {
        CHECK(result.secondCandidate.keys[index].sourceShotName.empty());
        CHECK(result.secondCandidate.keys[index].linkedCameraId.empty());
        CHECK(result.secondCandidate.keys[index].linkedCameraName.empty());
        CHECK(result.secondCandidate.keys[index].id != second.keys.back().id);
    }
    std::unordered_set<std::string> secondAppendedIds;
    for (std::size_t index = second.keys.size();
         index < result.secondCandidate.keys.size();
         ++index) {
        CHECK(secondAppendedIds.insert(
                  result.secondCandidate.keys[index].id).second);
    }

    REQUIRE(result.firstCandidate.localizedKeyCorrections.size() >= 3U);
    const auto preservedMiddleCorrection = std::find_if(
        result.firstCandidate.localizedKeyCorrections.begin(),
        result.firstCandidate.localizedKeyCorrections.end(),
        [&](const auto& correction) {
            return correction.keyId == first.keys[1U].id;
        });
    REQUIRE(preservedMiddleCorrection !=
            result.firstCandidate.localizedKeyCorrections.end());
    CHECK(preservedMiddleCorrection->hasCameraCorrectionTangent);
    CHECK(preservedMiddleCorrection->hasFocusCorrectionTangent);

    const auto firstAfter = invisible_places::camera::
        PrepareAnimationPathEvaluation(result.firstCandidate);
    const auto secondAfter = invisible_places::camera::
        PrepareAnimationPathEvaluation(result.secondCandidate);
    REQUIRE(firstAfter.valid);
    REQUIRE(secondAfter.valid);
    const float firstPrefixEnd = 41.0F / 30.0F;
    const float secondPrefixEnd = 54.0F / 30.0F;
    for (std::uint32_t sample = 0U; sample <= 120U; ++sample) {
        const float amount = static_cast<float>(sample) / 120.0F;
        CheckPanEvaluationNear(
            invisible_places::camera::EvaluatePreparedAnimationPath(
                firstBefore,
                firstPrefixEnd * amount),
            invisible_places::camera::EvaluatePreparedAnimationPath(
                firstAfter,
                firstPrefixEnd * amount));
        CheckPanEvaluationNear(
            invisible_places::camera::EvaluatePreparedAnimationPath(
                secondBefore,
                secondPrefixEnd * amount),
            invisible_places::camera::EvaluatePreparedAnimationPath(
                secondAfter,
                secondPrefixEnd * amount));
    }
    CHECK(result.metrics.maxPrefixPositionError[0U] < 2.0e-4F);
    CHECK(result.metrics.maxPrefixPositionError[1U] < 2.0e-4F);
    CHECK(result.metrics.rotationConstrained[0U]);
    CHECK(result.metrics.rotationConstrained[1U]);
    CHECK(std::isfinite(result.metrics.anchorOverlayRmsScreenHeights[0U]));
    CHECK(std::isfinite(result.metrics.anchorOverlayRmsScreenHeights[1U]));
    CHECK(std::isfinite(
        result.metrics.patchNodeOverlayRmsScreenHeights[0U]));
    CHECK(std::isfinite(
        result.metrics.patchNodeOverlayRmsScreenHeights[1U]));
    CHECK(std::isfinite(
        result.metrics.perspectiveScaleResidualPercent[0U]));
    CHECK(std::isfinite(
        result.metrics.perspectiveScaleResidualPercent[1U]));
}

TEST_CASE("Bidirectional reciprocal pan extension generates pre-roll and tail from each seam triangle pair",
          "[camera][animation][pan-extension][bidirectional]") {
    auto first = MakeReciprocalPanTestPath("full-A", 1.0F, 41U, 79U);
    auto second = MakeReciprocalPanTestPath("full-B", -1.0F, 54U, 96U);
    AddLegacyWaterTracks(&first);
    AddLegacyWaterTracks(&second);
    const auto options = MakeReciprocalPanTestOptions();

    const auto firstSeam = invisible_places::camera::
        BuildAnimationPanBidirectionalSeamPreview(
            first,
            second,
            options.firstDrivesSecond,
            options.aspectRatio,
            options.sampleCount,
            options.optimizationSweeps);
    INFO(firstSeam.errorMessage);
    REQUIRE(firstSeam.succeeded);
    REQUIRE(firstSeam.changed);
    CHECK(firstSeam.sourceHead.candidate.durationFrames == 155U);
    CHECK(firstSeam.destinationTail.candidate.durationFrames == 185U);
    CHECK(firstSeam.sourceHead.appendedKeyCount == 2U);
    CHECK(firstSeam.destinationTail.appendedKeyCount == 2U);
    CHECK(firstSeam.sourceHead.candidate.exportSettings.startFrame == 42U);
    CHECK(firstSeam.sourceHead.candidate.exportSettings.endFrame == 126U);
    CHECK(firstSeam.destinationTail.candidate.exportSettings.startFrame == 7U);
    CHECK(firstSeam.destinationTail.candidate.exportSettings.endFrame == 91U);

    const auto result = invisible_places::camera::
        BuildAnimationBidirectionalReciprocalPanExtension(
            first,
            second,
            options);
    INFO(result.errorMessage);
    REQUIRE(result.succeeded);
    REQUIRE(result.changed);
    REQUIRE(result.firstCandidate.keys.size() == first.keys.size() + 4U);
    REQUIRE(result.secondCandidate.keys.size() == second.keys.size() + 4U);
    CHECK(result.firstCandidate.durationFrames == 199U);
    CHECK(result.secondCandidate.durationFrames == 229U);
    CHECK(result.metrics.incoming.extensionFrames[0U] == 35U);
    CHECK(result.metrics.incoming.extensionFrames[1U] == 44U);
    CHECK(result.metrics.outgoing.extensionFrames[0U] == 44U);
    CHECK(result.metrics.outgoing.extensionFrames[1U] == 35U);
    CHECK(result.metrics.incoming.appendedKeyCount[0U] == 2U);
    CHECK(result.metrics.incoming.appendedKeyCount[1U] == 2U);
    CHECK(result.metrics.outgoing.appendedKeyCount[0U] == 2U);
    CHECK(result.metrics.outgoing.appendedKeyCount[1U] == 2U);

    const auto secondSeam = invisible_places::camera::
        BuildAnimationPanBidirectionalSeamPreview(
            second,
            first,
            options.secondDrivesFirst,
            options.aspectRatio,
            options.sampleCount,
            options.optimizationSweeps);
    INFO(secondSeam.errorMessage);
    REQUIRE(secondSeam.succeeded);
    const auto mergedFirst = invisible_places::camera::
        PrepareAnimationPathEvaluation(result.firstCandidate);
    const auto mergedSecond = invisible_places::camera::
        PrepareAnimationPathEvaluation(result.secondCandidate);
    const auto expectedFirstHead = invisible_places::camera::
        PrepareAnimationPathEvaluation(firstSeam.sourceHead.candidate);
    const auto expectedFirstTail = invisible_places::camera::
        PrepareAnimationPathEvaluation(secondSeam.destinationTail.candidate);
    const auto expectedSecondHead = invisible_places::camera::
        PrepareAnimationPathEvaluation(secondSeam.sourceHead.candidate);
    const auto expectedSecondTail = invisible_places::camera::
        PrepareAnimationPathEvaluation(firstSeam.destinationTail.candidate);
    REQUIRE(mergedFirst.valid);
    REQUIRE(mergedSecond.valid);
    REQUIRE(expectedFirstHead.valid);
    REQUIRE(expectedFirstTail.valid);
    REQUIRE(expectedSecondHead.valid);
    REQUIRE(expectedSecondTail.valid);
    for (std::uint32_t sample = 0U; sample <= 12U; ++sample) {
        const float amount = static_cast<float>(sample) / 12.0F;
        const float firstHeadTime = amount * 35.0F / 30.0F;
        const float firstTailTime =
            (120.0F + amount * 44.0F) / 30.0F;
        const float secondHeadTime = amount * 44.0F / 30.0F;
        const float secondTailTime =
            (150.0F + amount * 35.0F) / 30.0F;
        CheckPanEvaluationNear(
            invisible_places::camera::EvaluatePreparedAnimationPath(
                mergedFirst,
                firstHeadTime),
            invisible_places::camera::EvaluatePreparedAnimationPath(
                expectedFirstHead,
                firstHeadTime),
            3.0e-4F);
        CheckPanEvaluationNear(
            invisible_places::camera::EvaluatePreparedAnimationPath(
                mergedFirst,
                35.0F / 30.0F + firstTailTime),
            invisible_places::camera::EvaluatePreparedAnimationPath(
                expectedFirstTail,
                firstTailTime),
            3.0e-4F);
        CheckPanEvaluationNear(
            invisible_places::camera::EvaluatePreparedAnimationPath(
                mergedSecond,
                secondHeadTime),
            invisible_places::camera::EvaluatePreparedAnimationPath(
                expectedSecondHead,
                secondHeadTime),
            3.0e-4F);
        CheckPanEvaluationNear(
            invisible_places::camera::EvaluatePreparedAnimationPath(
                mergedSecond,
                44.0F / 30.0F + secondTailTime),
            invisible_places::camera::EvaluatePreparedAnimationPath(
                expectedSecondTail,
                secondTailTime),
            3.0e-4F);
    }
    CHECK(result.firstCandidate.exportSettings.startFrame == 42U);
    CHECK(result.firstCandidate.exportSettings.endFrame == 126U);
    CHECK(result.secondCandidate.exportSettings.startFrame == 51U);
    CHECK(result.secondCandidate.exportSettings.endFrame == 135U);

    const auto keyFrame = [](const auto& path, std::string_view id) {
        std::uint32_t frame = 0U;
        for (std::size_t index = 0U; index < path.keys.size(); ++index) {
            if (path.keys[index].id == id) {
                return frame;
            }
            if (index + 1U < path.keys.size()) {
                frame += path.keys[index + 1U].durationFrames;
            }
        }
        return std::numeric_limits<std::uint32_t>::max();
    };
    CHECK(keyFrame(result.firstCandidate, first.keys[0U].id) == 35U);
    CHECK(keyFrame(result.firstCandidate, first.keys[1U].id) == 76U);
    CHECK(keyFrame(result.firstCandidate, first.keys[2U].id) == 155U);
    CHECK(keyFrame(result.secondCandidate, second.keys[0U].id) == 44U);
    CHECK(keyFrame(result.secondCandidate, second.keys[1U].id) == 98U);
    CHECK(keyFrame(result.secondCandidate, second.keys[2U].id) == 194U);

    std::unordered_set<std::string> originalIds;
    for (const auto& key : first.keys) {
        originalIds.insert(key.id);
    }
    std::size_t generatedFirstKeys = 0U;
    for (const auto& key : result.firstCandidate.keys) {
        if (!originalIds.contains(key.id)) {
            ++generatedFirstKeys;
            CHECK(key.linkedCameraId.empty());
            CHECK(key.linkedCameraName.empty());
            CHECK(key.sourceShotName.empty());
        }
    }
    CHECK(generatedFirstKeys == 4U);

    REQUIRE(result.firstCandidate.waterScenarioTracks.size() == 1U);
    REQUIRE(result.secondCandidate.waterScenarioTracks.size() == 1U);
    const auto& firstWater =
        result.firstCandidate.waterScenarioTracks.front();
    const auto& secondWater =
        result.secondCandidate.waterScenarioTracks.front();
    CHECK(firstWater.keys[0U].position ==
          Approx((35.0F + 0.25F * 120.0F) / 199.0F));
    CHECK(firstWater.keys[1U].position ==
          Approx((35.0F + 120.0F) / 199.0F));
    CHECK(secondWater.keys[0U].position ==
          Approx((44.0F + 0.25F * 150.0F) / 229.0F));
    CHECK(secondWater.keys[1U].position ==
          Approx((44.0F + 150.0F) / 229.0F));

    auto migratedFirst = first;
    auto migratedSecond = second;
    migratedFirst.authoredTrackDurationFrames = 90U;
    migratedSecond.authoredTrackDurationFrames = 110U;
    const auto migrated = invisible_places::camera::
        BuildAnimationBidirectionalReciprocalPanExtension(
            migratedFirst,
            migratedSecond,
            options);
    INFO(migrated.errorMessage);
    REQUIRE(migrated.succeeded);
    CHECK(migrated.firstCandidate.authoredTrackDurationFrames == 0U);
    CHECK(migrated.secondCandidate.authoredTrackDurationFrames == 0U);
    CHECK(migrated.firstCandidate.waterScenarioTracks.front()
              .keys.front()
              .position == Approx((35.0F + 0.25F * 90.0F) / 199.0F));
    CHECK(migrated.secondCandidate.waterScenarioTracks.front()
              .keys.front()
              .position == Approx((44.0F + 0.25F * 110.0F) / 229.0F));

    auto swappedOptions = options;
    swappedOptions.firstDrivesSecond = options.secondDrivesFirst;
    swappedOptions.secondDrivesFirst = options.firstDrivesSecond;
    const auto swapped = invisible_places::camera::
        BuildAnimationBidirectionalReciprocalPanExtension(
            second,
            first,
            swappedOptions);
    INFO(swapped.errorMessage);
    REQUIRE(swapped.succeeded);
    REQUIRE(swapped.firstCandidate.durationFrames ==
            result.secondCandidate.durationFrames);
    REQUIRE(swapped.secondCandidate.durationFrames ==
            result.firstCandidate.durationFrames);
    const auto forwardFirst = invisible_places::camera::
        PrepareAnimationPathEvaluation(result.firstCandidate);
    const auto forwardSecond = invisible_places::camera::
        PrepareAnimationPathEvaluation(result.secondCandidate);
    const auto swappedFirst = invisible_places::camera::
        PrepareAnimationPathEvaluation(swapped.firstCandidate);
    const auto swappedSecond = invisible_places::camera::
        PrepareAnimationPathEvaluation(swapped.secondCandidate);
    REQUIRE(forwardFirst.valid);
    REQUIRE(forwardSecond.valid);
    REQUIRE(swappedFirst.valid);
    REQUIRE(swappedSecond.valid);
    for (std::uint32_t sample = 0U; sample <= 24U; ++sample) {
        const float amount = static_cast<float>(sample) / 24.0F;
        CheckPanEvaluationNear(
            invisible_places::camera::EvaluatePreparedAnimationPath(
                forwardFirst,
                amount * forwardFirst.durationSeconds),
            invisible_places::camera::EvaluatePreparedAnimationPath(
                swappedSecond,
                amount * swappedSecond.durationSeconds),
            3.0e-4F);
        CheckPanEvaluationNear(
            invisible_places::camera::EvaluatePreparedAnimationPath(
                forwardSecond,
                amount * forwardSecond.durationSeconds),
            invisible_places::camera::EvaluatePreparedAnimationPath(
                swappedFirst,
                amount * swappedFirst.durationSeconds),
            3.0e-4F);
    }
}

TEST_CASE(
    "Destination seam axes preserve source dimensions in the destination camera frame",
    "[camera][animation][pan-extension][correspondence]") {
    invisible_places::camera::AnimationPathEvaluation source;
    source.camera.orientation = {0.0F, 0.0F, 0.0F, 1.0F};
    invisible_places::camera::AnimationPathEvaluation destination;
    constexpr float kHalfSqrtTwo = 0.70710678118F;
    destination.camera.orientation = {
        0.0F,
        0.0F,
        kHalfSqrtTwo,
        kHalfSqrtTwo,
    };
    invisible_places::camera::AnimationSurfacePatchObservation patch;
    patch.pointCount = 3U;
    patch.worldPoints[0U] = {10.0F, 10.0F, 10.0F};
    patch.worldPoints[1U] = {11.0F, 10.0F, 10.0F};
    patch.worldPoints[2U] = {10.0F, 12.0F, 10.0F};

    const auto generated = invisible_places::camera::
        BuildAnimationCameraLocalSurfacePatch(
            source,
            destination,
            patch,
            {2.0F, 3.0F, 4.0F});
    REQUIRE(generated.pointCount == 3U);
    CHECK(generated.worldPoints[0U][0U] == Approx(2.0F));
    CHECK(generated.worldPoints[0U][1U] == Approx(3.0F));
    CHECK(generated.worldPoints[0U][2U] == Approx(4.0F));
    CHECK(generated.worldPoints[1U][0U] == Approx(2.0F).margin(1.0e-5F));
    CHECK(generated.worldPoints[1U][1U] == Approx(4.0F).margin(1.0e-5F));
    CHECK(generated.worldPoints[1U][2U] == Approx(4.0F).margin(1.0e-5F));
    CHECK(generated.worldPoints[2U][0U] == Approx(0.0F).margin(1.0e-5F));
    CHECK(generated.worldPoints[2U][1U] == Approx(3.0F).margin(1.0e-5F));
    CHECK(generated.worldPoints[2U][2U] == Approx(4.0F).margin(1.0e-5F));

    patch.worldPoints[2U] = patch.worldPoints[1U];
    CHECK(invisible_places::camera::BuildAnimationCameraLocalSurfacePatch(
              source,
              destination,
              patch,
              {2.0F, 3.0F, 4.0F})
              .pointCount == 0U);
}

TEST_CASE(
    "One-seam pan preview uses the same fitted destination as the reciprocal build",
    "[camera][animation][pan-extension]") {
    const auto first = MakeReciprocalPanTestPath("A", 1.0F, 41U, 79U);
    const auto second = MakeReciprocalPanTestPath("B", -1.0F, 54U, 96U);
    const auto options = MakeReciprocalPanTestOptions();
    const auto reciprocal = invisible_places::camera::
        BuildAnimationReciprocalPanExtension(first, second, options);
    INFO(reciprocal.errorMessage);
    REQUIRE(reciprocal.succeeded);

    const auto preview = invisible_places::camera::
        BuildAnimationPanTerminalExtensionPreview(
            second,
            first,
            options.firstDrivesSecond,
            options.aspectRatio,
            options.sampleCount,
            options.optimizationSweeps);
    INFO(preview.errorMessage);
    REQUIRE(preview.succeeded);
    REQUIRE(preview.changed);
    CHECK(preview.extensionFrames ==
          reciprocal.metrics.extensionFrames[1U]);
    CHECK(preview.appendedKeyCount ==
          reciprocal.metrics.appendedKeyCount[1U]);
    CHECK(preview.candidate.durationFrames ==
          reciprocal.secondCandidate.durationFrames);
    CHECK(preview.candidate.keys.size() ==
          reciprocal.secondCandidate.keys.size());
    CHECK(preview.anchorOverlayRmsScreenHeights ==
          Approx(reciprocal.metrics.anchorOverlayRmsScreenHeights[1U]));
    CHECK(preview.patchNodeOverlayRmsScreenHeights == Approx(
          reciprocal.metrics.patchNodeOverlayRmsScreenHeights[1U]));

    const auto previewContext = invisible_places::camera::
        PrepareAnimationPathEvaluation(preview.candidate);
    const auto reciprocalContext = invisible_places::camera::
        PrepareAnimationPathEvaluation(reciprocal.secondCandidate);
    REQUIRE(previewContext.valid);
    REQUIRE(reciprocalContext.valid);
    for (std::uint32_t sample = 0U; sample <= 90U; ++sample) {
        const float time = previewContext.durationSeconds *
            static_cast<float>(sample) / 90.0F;
        CheckPanEvaluationNear(
            invisible_places::camera::EvaluatePreparedAnimationPath(
                previewContext,
                time),
            invisible_places::camera::EvaluatePreparedAnimationPath(
                reciprocalContext,
                time),
            1.0e-6F);
    }
}

TEST_CASE("Reciprocal pan extension is argument-order independent and atomic on invalid input",
          "[camera][animation][pan-extension]") {
    const auto first = MakeReciprocalPanTestPath("A", 1.0F, 41U, 79U);
    const auto second = MakeReciprocalPanTestPath("B", -1.0F, 54U, 96U);
    const auto options = MakeReciprocalPanTestOptions();
    const auto forward = invisible_places::camera::
        BuildAnimationReciprocalPanExtension(first, second, options);
    INFO(forward.errorMessage);
    REQUIRE(forward.succeeded);

    auto previouslyExtendedFirst = first;
    auto previouslyExtendedSecond = second;
    AddLegacyWaterTracks(&previouslyExtendedFirst);
    AddLegacyWaterTracks(&previouslyExtendedSecond);
    previouslyExtendedFirst.authoredTrackDurationFrames = 90U;
    previouslyExtendedSecond.authoredTrackDurationFrames = 110U;
    const auto retainedTrackDurations = invisible_places::camera::
        BuildAnimationReciprocalPanExtension(
            previouslyExtendedFirst,
            previouslyExtendedSecond,
            options);
    INFO(retainedTrackDurations.errorMessage);
    REQUIRE(retainedTrackDurations.succeeded);
    CHECK(retainedTrackDurations.firstCandidate.authoredTrackDurationFrames ==
          0U);
    CHECK(retainedTrackDurations.secondCandidate.authoredTrackDurationFrames ==
          0U);
    CheckLegacyWaterTracksRetimed(
        retainedTrackDurations.firstCandidate,
        90.0F / 164.0F);
    CheckLegacyWaterTracksRetimed(
        retainedTrackDurations.secondCandidate,
        110.0F / 185.0F);

    auto swappedOptions = options;
    swappedOptions.firstDrivesSecond = options.secondDrivesFirst;
    swappedOptions.secondDrivesFirst = options.firstDrivesSecond;
    const auto swapped = invisible_places::camera::
        BuildAnimationReciprocalPanExtension(
            second,
            first,
            swappedOptions);
    INFO(swapped.errorMessage);
    REQUIRE(swapped.succeeded);
    CHECK(swapped.firstCandidate.durationFrames ==
          forward.secondCandidate.durationFrames);
    CHECK(swapped.secondCandidate.durationFrames ==
          forward.firstCandidate.durationFrames);
    REQUIRE(swapped.firstCandidate.keys.size() ==
            forward.secondCandidate.keys.size());
    REQUIRE(swapped.secondCandidate.keys.size() ==
            forward.firstCandidate.keys.size());
    const auto swappedFirst = invisible_places::camera::
        PrepareAnimationPathEvaluation(swapped.firstCandidate);
    const auto swappedSecond = invisible_places::camera::
        PrepareAnimationPathEvaluation(swapped.secondCandidate);
    const auto forwardFirst = invisible_places::camera::
        PrepareAnimationPathEvaluation(forward.firstCandidate);
    const auto forwardSecond = invisible_places::camera::
        PrepareAnimationPathEvaluation(forward.secondCandidate);
    REQUIRE(swappedFirst.valid);
    REQUIRE(swappedSecond.valid);
    REQUIRE(forwardFirst.valid);
    REQUIRE(forwardSecond.valid);
    CheckPanEvaluationNear(
        invisible_places::camera::EvaluatePreparedAnimationPath(
            swappedFirst,
            swappedFirst.durationSeconds),
        invisible_places::camera::EvaluatePreparedAnimationPath(
            forwardSecond,
            forwardSecond.durationSeconds));
    CheckPanEvaluationNear(
        invisible_places::camera::EvaluatePreparedAnimationPath(
            swappedSecond,
            swappedSecond.durationSeconds),
        invisible_places::camera::EvaluatePreparedAnimationPath(
            forwardFirst,
            forwardFirst.durationSeconds));

    auto animatedLens = second;
    animatedLens.keys[1U].fovDegrees += 5.0F;
    const auto rejected = invisible_places::camera::
        BuildAnimationReciprocalPanExtension(
            first,
            animatedLens,
            options);
    CHECK_FALSE(rejected.succeeded);
    CHECK_FALSE(rejected.changed);
    CHECK(rejected.firstCandidate.keys.empty());
    CHECK(rejected.secondCandidate.keys.empty());
    CHECK_FALSE(rejected.errorMessage.empty());
    CHECK(first.durationFrames == 120U);
    CHECK(second.durationFrames == 150U);

    auto authoredOrientation = second;
    authoredOrientation.keys[1U].hasOrientation = true;
    const auto orientationRejected = invisible_places::camera::
        BuildAnimationReciprocalPanExtension(
            first,
            authoredOrientation,
            options);
    CHECK_FALSE(orientationRejected.succeeded);
    CHECK(orientationRejected.firstCandidate.keys.empty());
    CHECK(orientationRejected.secondCandidate.keys.empty());

    auto hiddenLensMotion = second;
    hiddenLensMotion.keys.front().hasSplineEndpointTangent = true;
    hiddenLensMotion.keys.front().splineLensEndpointTangent[0U] = 0.25F;
    const auto tangentRejected = invisible_places::camera::
        BuildAnimationReciprocalPanExtension(
            first,
            hiddenLensMotion,
            options);
    CHECK_FALSE(tangentRejected.succeeded);
    CHECK(tangentRejected.firstCandidate.keys.empty());
    CHECK(tangentRejected.secondCandidate.keys.empty());

    auto inactiveFocusTangent = second;
    inactiveFocusTangent.keys.front().hasSplineEndpointTangent = true;
    inactiveFocusTangent.keys.front().splineLensEndpointTangent[3U] = 0.25F;
    const auto inactiveTangentAccepted = invisible_places::camera::
        BuildAnimationReciprocalPanExtension(
            first,
            inactiveFocusTangent,
            options);
    INFO(inactiveTangentAccepted.errorMessage);
    CHECK(inactiveTangentAccepted.succeeded);

    auto overflowing = first;
    overflowing.durationFrames =
        std::numeric_limits<std::uint32_t>::max() - 10U;
    overflowing.keys[1U].durationFrames = 1U;
    overflowing.keys[2U].durationFrames =
        std::numeric_limits<std::uint32_t>::max() - 11U;
    const auto overflowRejected = invisible_places::camera::
        BuildAnimationReciprocalPanExtension(
            overflowing,
            second,
            options);
    CHECK_FALSE(overflowRejected.succeeded);
    CHECK(overflowRejected.firstCandidate.keys.empty());
    CHECK(overflowRejected.secondCandidate.keys.empty());

    auto incomingDurationOverflow = first;
    incomingDurationOverflow.keys[1U].durationFrames =
        std::numeric_limits<std::uint32_t>::max();
    incomingDurationOverflow.keys[2U].durationFrames =
        std::numeric_limits<std::uint32_t>::max();
    const auto durationSumRejected = invisible_places::camera::
        BuildAnimationReciprocalPanExtension(
            incomingDurationOverflow,
            second,
            options);
    CHECK_FALSE(durationSumRejected.succeeded);
    CHECK(durationSumRejected.firstCandidate.keys.empty());
    CHECK(durationSumRejected.secondCandidate.keys.empty());
    CHECK(durationSumRejected.errorMessage.find("overflow") !=
          std::string::npos);
}

TEST_CASE("Reciprocal pan extension chooses three keys for an interior source key and caps unstable source tails",
          "[camera][animation][pan-extension]") {
    const auto makeFourKeyPan = [](std::string name, float offset) {
        invisible_places::camera::AnimationPath path;
        path.name = name;
        path.durationFrames = 60U;
        path.keys = {
            {.id = name + "-0",
             .cameraPosition = {offset - 1.5F, -8.0F, 2.0F},
             .focusPoint = {-0.4F, 0.0F, 1.0F}},
            {.id = name + "-1",
             .cameraPosition = {offset - 1.0F, -8.0F, 2.0F},
             .focusPoint = {-0.2F, 0.0F, 1.0F},
             .durationFrames = 10U},
            {.id = name + "-2",
             .cameraPosition = {offset, -8.0F, 2.0F},
             .focusPoint = {0.0F, 0.0F, 1.0F},
             .durationFrames = 20U},
            {.id = name + "-3",
             .cameraPosition = {offset + 1.0F, -8.0F, 2.0F},
             .focusPoint = {0.3F, 0.0F, 1.0F},
             .durationFrames = 30U},
        };
        return path;
    };
    const auto first = makeFourKeyPan("four-a", 0.0F);
    const auto second = makeFourKeyPan("four-b", 0.5F);
    const auto firstBefore = invisible_places::camera::
        PrepareAnimationPathEvaluation(first);
    const auto secondBefore = invisible_places::camera::
        PrepareAnimationPathEvaluation(second);
    REQUIRE(firstBefore.valid);
    REQUIRE(secondBefore.valid);

    invisible_places::camera::AnimationReciprocalPanExtensionOptions options;
    options.firstDrivesSecond.sourceTailFrame = 29U;
    options.secondDrivesFirst.sourceTailFrame = 29U;
    options.firstDrivesSecond.sourcePatch = MakeReciprocalPanTestPatch();
    options.firstDrivesSecond.destinationEndPatch =
        MakeReciprocalPanTestPatch();
    options.secondDrivesFirst.sourcePatch = MakeReciprocalPanTestPatch();
    options.secondDrivesFirst.destinationEndPatch =
        MakeReciprocalPanTestPatch();
    options.sampleCount = 9U;
    options.optimizationSweeps = 8U;

    const auto result = invisible_places::camera::
        BuildAnimationReciprocalPanExtension(first, second, options);
    INFO(result.errorMessage);
    REQUIRE(result.succeeded);
    REQUIRE(result.firstCandidate.keys.size() == first.keys.size() + 3U);
    REQUIRE(result.secondCandidate.keys.size() == second.keys.size() + 3U);
    CHECK(result.metrics.appendedKeyCount[0U] == 3U);
    CHECK(result.metrics.appendedKeyCount[1U] == 3U);
    CHECK(result.metrics.sourceInteriorKeyCount[0U] == 1U);
    CHECK(result.metrics.sourceInteriorKeyCount[1U] == 1U);
    CHECK(result.firstCandidate.durationFrames == 89U);
    CHECK(result.secondCandidate.durationFrames == 89U);
    const std::array<std::uint32_t, 3> expectedTailDurations{10U, 9U, 10U};
    std::uint32_t firstTailFrameSum = 0U;
    std::uint32_t secondTailFrameSum = 0U;
    for (std::size_t index = 0U; index < expectedTailDurations.size(); ++index) {
        const auto firstDuration =
            result.firstCandidate.keys[first.keys.size() + index]
                .durationFrames;
        const auto secondDuration =
            result.secondCandidate.keys[second.keys.size() + index]
                .durationFrames;
        CHECK(firstDuration == expectedTailDurations[index]);
        CHECK(secondDuration == expectedTailDurations[index]);
        CHECK(firstDuration > 0U);
        CHECK(secondDuration > 0U);
        firstTailFrameSum += firstDuration;
        secondTailFrameSum += secondDuration;
    }
    CHECK(firstTailFrameSum == 29U);
    CHECK(secondTailFrameSum == 29U);
    const auto firstAfter = invisible_places::camera::
        PrepareAnimationPathEvaluation(result.firstCandidate);
    const auto secondAfter = invisible_places::camera::
        PrepareAnimationPathEvaluation(result.secondCandidate);
    REQUIRE(firstAfter.valid);
    REQUIRE(secondAfter.valid);
    for (std::uint32_t sample = 0U; sample <= 60U; ++sample) {
        const float time = static_cast<float>(sample) / 60.0F;
        CheckPanEvaluationNear(
            invisible_places::camera::EvaluatePreparedAnimationPath(
                firstBefore,
                time),
            invisible_places::camera::EvaluatePreparedAnimationPath(
                firstAfter,
                time));
        CheckPanEvaluationNear(
            invisible_places::camera::EvaluatePreparedAnimationPath(
                secondBefore,
                time),
            invisible_places::camera::EvaluatePreparedAnimationPath(
                secondAfter,
                time));
    }

    auto unstable = options;
    unstable.firstDrivesSecond.sourceTailFrame = 31U;
    const auto rejected = invisible_places::camera::
        BuildAnimationReciprocalPanExtension(first, second, unstable);
    CHECK_FALSE(rejected.succeeded);
    CHECK(rejected.firstCandidate.keys.empty());
    CHECK(rejected.secondCandidate.keys.empty());
    CHECK(rejected.errorMessage.find("penultimate") != std::string::npos);
}

TEST_CASE("Reciprocal two-key pans allow a pre-terminal best-fit source tail",
          "[camera][animation][pan-extension]") {
    const auto makeTwoKeyPan = [](std::string name, float offset) {
        invisible_places::camera::AnimationPath path;
        path.name = name;
        path.durationFrames = 40U;
        path.keys = {
            {.id = name + "-start",
             .cameraPosition = {offset - 1.0F, -8.0F, 2.0F},
             .focusPoint = {-0.2F, 0.0F, 1.0F}},
            {.id = name + "-end",
             .cameraPosition = {offset + 1.0F, -8.0F, 2.0F},
             .focusPoint = {0.2F, 0.0F, 1.0F},
             .durationFrames = 40U},
        };
        return path;
    };
    invisible_places::camera::AnimationReciprocalPanExtensionOptions options;
    options.firstDrivesSecond.sourceTailFrame = 20U;
    options.secondDrivesFirst.sourceTailFrame = 20U;
    options.firstDrivesSecond.sourcePatch = MakeReciprocalPanTestPatch();
    options.firstDrivesSecond.destinationEndPatch =
        MakeReciprocalPanTestPatch();
    options.secondDrivesFirst.sourcePatch = MakeReciprocalPanTestPatch();
    options.secondDrivesFirst.destinationEndPatch =
        MakeReciprocalPanTestPatch();
    options.sampleCount = 7U;
    options.optimizationSweeps = 6U;
    const auto result = invisible_places::camera::
        BuildAnimationReciprocalPanExtension(
            makeTwoKeyPan("two-a", 0.0F),
            makeTwoKeyPan("two-b", 0.5F),
            options);
    INFO(result.errorMessage);
    REQUIRE(result.succeeded);
    CHECK(result.metrics.appendedKeyCount[0U] == 2U);
    CHECK(result.metrics.appendedKeyCount[1U] == 2U);
    CHECK(result.metrics.patchDiagnostic[0U].find("best fit") !=
          std::string::npos);
    CHECK(result.metrics.patchDiagnostic[1U].find("best fit") !=
          std::string::npos);
}

TEST_CASE("Reciprocal pan extension rejects incomplete, degenerate, and behind-camera triangles",
          "[camera][animation][pan-extension]") {
    const auto first = MakeReciprocalPanTestPath("A", 1.0F, 41U, 79U);
    const auto second = MakeReciprocalPanTestPath("B", -1.0F, 54U, 96U);
    auto pointOptions = MakeReciprocalPanTestOptions();
    pointOptions.firstDrivesSecond.sourcePatch.pointCount = 1U;
    const auto incomplete = invisible_places::camera::
        BuildAnimationReciprocalPanExtension(
            first,
            second,
            pointOptions);
    CHECK_FALSE(incomplete.succeeded);
    CHECK(incomplete.firstCandidate.keys.empty());

    auto degenerateOptions = MakeReciprocalPanTestOptions();
    degenerateOptions.firstDrivesSecond.sourcePatch.worldPoints[2U] =
        degenerateOptions.firstDrivesSecond.sourcePatch.worldPoints[1U];
    const auto degenerate = invisible_places::camera::
        BuildAnimationReciprocalPanExtension(
            first,
            second,
            degenerateOptions);
    CHECK_FALSE(degenerate.succeeded);
    CHECK(degenerate.firstCandidate.keys.empty());

    auto behindOptions = MakeReciprocalPanTestOptions();
    invisible_places::camera::AnimationSurfacePatchObservation behind;
    behind.pointCount = 3U;
    behind.worldPoints[0U] = {0.0F, -20.0F, 1.0F};
    behind.worldPoints[1U] = {0.25F, -20.0F, 1.0F};
    behind.worldPoints[2U] = {0.0F, -20.0F, 1.25F};
    behindOptions.firstDrivesSecond.sourcePatch = behind;
    behindOptions.firstDrivesSecond.destinationEndPatch = behind;
    const auto rejected = invisible_places::camera::
        BuildAnimationReciprocalPanExtension(
            first,
            second,
            behindOptions);
    CHECK_FALSE(rejected.succeeded);
    CHECK_FALSE(rejected.changed);
    CHECK(rejected.firstCandidate.keys.empty());
    CHECK(rejected.secondCandidate.keys.empty());
    CHECK_FALSE(rejected.errorMessage.empty());

    auto tangentBehindOptions = MakeReciprocalPanTestOptions();
    auto tangentBehind = MakeReciprocalPanTestPatch();
    tangentBehind.worldPoints[1U] = {0.25F, -20.0F, 1.0F};
    tangentBehindOptions.firstDrivesSecond.sourcePatch = tangentBehind;
    const auto tangentRejected = invisible_places::camera::
        BuildAnimationReciprocalPanExtension(
            first,
            second,
            tangentBehindOptions);
    CHECK_FALSE(tangentRejected.succeeded);
    CHECK(tangentRejected.firstCandidate.keys.empty());
    CHECK(tangentRejected.secondCandidate.keys.empty());
}

TEST_CASE("Reciprocal parallel pans match signed patch velocity through both appended seams",
          "[camera][animation][pan-extension][velocity]") {
    const auto makeParallelPan = [](std::string name, float offset) {
        invisible_places::camera::AnimationPath path;
        path.name = name;
        path.durationFrames = 60U;
        for (std::size_t index = 0U; index < 3U; ++index) {
            const float x = offset + static_cast<float>(index);
            path.keys.push_back({
                .id = name + "-" + std::to_string(index),
                .cameraPosition = {x, 0.0F, 0.0F},
                .focusPoint = {x, 10.0F, 0.0F},
                .durationFrames = 30U,
            });
        }
        return path;
    };
    invisible_places::camera::AnimationSurfacePatchObservation patch;
    patch.pointCount = 3U;
    patch.worldPoints[0U] = {0.0F, 10.0F, 0.0F};
    patch.worldPoints[1U] = {1.0F, 10.0F, 0.0F};
    patch.worldPoints[2U] = {0.0F, 10.0F, 1.0F};
    invisible_places::camera::AnimationReciprocalPanExtensionOptions options;
    options.firstDrivesSecond.sourceTailFrame = 20U;
    options.firstDrivesSecond.sourcePatch = patch;
    auto actualBTerminalPatch = patch;
    actualBTerminalPatch.worldPoints[0U][0U] = 2.0F;
    actualBTerminalPatch.worldPoints[1U][0U] = 3.0F;
    actualBTerminalPatch.worldPoints[2U][0U] = 2.0F;
    options.firstDrivesSecond.destinationEndPatch = actualBTerminalPatch;
    options.secondDrivesFirst.sourceTailFrame = 30U;
    options.secondDrivesFirst.sourcePatch = patch;
    auto actualATerminalPatch = patch;
    actualATerminalPatch.worldPoints[0U][0U] = 1.5F;
    actualATerminalPatch.worldPoints[1U][0U] = 2.5F;
    actualATerminalPatch.worldPoints[2U][0U] = 1.5F;
    options.secondDrivesFirst.destinationEndPatch = actualATerminalPatch;
    options.sampleCount = 9U;
    options.optimizationSweeps = 4U;

    const auto result = invisible_places::camera::
        BuildAnimationReciprocalPanExtension(
            makeParallelPan("A", 0.0F),
            makeParallelPan("B", 0.5F),
            options);
    INFO(result.errorMessage);
    REQUIRE(result.succeeded);
    CHECK(result.firstCandidate.durationFrames == 90U);
    CHECK(result.secondCandidate.durationFrames == 80U);
    for (std::size_t side = 0U; side < 2U; ++side) {
        CHECK(result.metrics.afterVelocityRmsScreenHeightsPerSecond[side] <
              1.0e-5F);
        CHECK(std::abs(
                  result.metrics
                      .signedVelocityResidualScreenHeightsPerSecond[side][0U]) <
              1.0e-5F);
        CHECK(std::abs(
                  result.metrics
                      .signedVelocityResidualScreenHeightsPerSecond[side][1U]) <
              1.0e-5F);
        CHECK(result.metrics.anchorOverlayRmsScreenHeights[side] <
              1.0e-4F);
    }

    SECTION("in-front correspondence triangles remain valid beyond the image") {
        auto offscreenOptions = options;
        for (auto* specification : {
                 &offscreenOptions.firstDrivesSecond,
                 &offscreenOptions.secondDrivesFirst}) {
            for (auto* observation : {
                     &specification->sourcePatch,
                     &specification->destinationEndPatch}) {
                for (auto& point : observation->worldPoints) {
                    // At depth 10 and the default 60-degree FOV this lies
                    // several screen widths outside a 16:9 image, while
                    // remaining fully in front of the camera.
                    point[0U] += 20.0F;
                }
            }
        }
        const auto offscreenResult = invisible_places::camera::
            BuildAnimationReciprocalPanExtension(
                makeParallelPan("offscreen-A", 0.0F),
                makeParallelPan("offscreen-B", 0.5F),
                offscreenOptions);
        INFO(offscreenResult.errorMessage);
        REQUIRE(offscreenResult.succeeded);
        CHECK(offscreenResult.firstCandidate.durationFrames == 90U);
        CHECK(offscreenResult.secondCandidate.durationFrames == 80U);
        for (std::size_t side = 0U; side < 2U; ++side) {
            CHECK(std::isfinite(
                offscreenResult.metrics
                    .afterVelocityRmsScreenHeightsPerSecond[side]));
            CHECK(std::isfinite(
                offscreenResult.metrics
                    .anchorOverlayRmsScreenHeights[side]));
            CHECK(offscreenResult.metrics.patchConfidence[side] > 0.0F);
        }
    }
}

TEST_CASE("Reciprocal off-centre orbit pans continue projected patch rotation",
          "[camera][animation][pan-extension][rotation]") {
    const auto makeOrbitPan = [](std::string name) {
        invisible_places::camera::AnimationPath path;
        path.name = name;
        path.durationFrames = 60U;
        path.keys = {
            {.id = name + "-0",
             .cameraPosition = {-4.0F, -6.0F, 2.0F},
             .focusPoint = {0.0F, 0.0F, 1.0F}},
            {.id = name + "-1",
             .cameraPosition = {0.0F, -7.0F, 3.0F},
             .focusPoint = {0.0F, 0.0F, 1.0F},
             .durationFrames = 30U},
            {.id = name + "-2",
             .cameraPosition = {4.0F, -6.0F, 2.0F},
             .focusPoint = {0.0F, 0.0F, 1.0F},
             .durationFrames = 30U},
        };
        return path;
    };
    invisible_places::camera::AnimationSurfacePatchObservation patch;
    patch.pointCount = 3U;
    patch.worldPoints[0U] = {2.0F, 0.0F, 1.0F};
    patch.worldPoints[1U] = {2.5F, 0.0F, 1.0F};
    patch.worldPoints[2U] = {2.0F, 0.0F, 1.5F};
    invisible_places::camera::AnimationReciprocalPanExtensionOptions options;
    options.firstDrivesSecond.sourceTailFrame = 20U;
    options.firstDrivesSecond.sourcePatch = patch;
    options.firstDrivesSecond.destinationEndPatch = patch;
    options.secondDrivesFirst = options.firstDrivesSecond;
    options.sampleCount = 17U;
    options.optimizationSweeps = 24U;

    const auto result = invisible_places::camera::
        BuildAnimationReciprocalPanExtension(
            makeOrbitPan("left"),
            makeOrbitPan("right"),
            options);
    INFO(result.errorMessage);
    REQUIRE(result.succeeded);
    for (std::size_t side = 0U; side < 2U; ++side) {
        CHECK(result.metrics.rotationConstrained[side]);
        CHECK(std::isfinite(
            result.metrics.rotationRateResidualDegreesPerSecond[side]));
        CHECK(result.metrics.rotationRateResidualDegreesPerSecond[side] <
              0.5F);
        CHECK(result.metrics.afterRotationRmsDegreesPerSecond[side] <=
              result.metrics.beforeRotationRmsDegreesPerSecond[side]);
    }
}
