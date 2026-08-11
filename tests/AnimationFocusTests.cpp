#include "camera/AnimationPath.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cmath>
#include <vector>

namespace {

using Catch::Approx;
using invisible_places::camera::CollectRayHitDistancesAlongRay;
using invisible_places::camera::ResolveFirstRayHitCluster;
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
