#include "camera/AnimationPath.hpp"
#include "camera/OrbitCamera.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <glm/gtc/quaternion.hpp>
#include <glm/trigonometric.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <tuple>
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

TEST_CASE("World camera segments clip to the visible view-frustum interval",
          "[camera][projection][frustum-clip]") {
    const glm::mat4 identity{1.0F};

    SECTION("an on-screen segment is unchanged") {
        const auto segment = invisible_places::camera::
            ProjectWorldSegmentToNdc(
                identity,
                {-0.5F, 0.25F, -0.25F},
                {0.5F, -0.25F, 0.25F});
        REQUIRE(segment.has_value());
        CHECK(segment->start.x == Approx(-0.5F));
        CHECK(segment->start.y == Approx(0.25F));
        CHECK(segment->start.z == Approx(-0.25F));
        CHECK(segment->end.x == Approx(0.5F));
        CHECK(segment->end.y == Approx(-0.25F));
        CHECK(segment->end.z == Approx(0.25F));
    }

    SECTION("one off-screen endpoint clips at the viewport edge") {
        const auto segment = invisible_places::camera::
            ProjectWorldSegmentToNdc(
                identity,
                {0.0F, 0.0F, 0.0F},
                {2.0F, 0.5F, 0.0F});
        REQUIRE(segment.has_value());
        CHECK(segment->start.x == Approx(0.0F));
        CHECK(segment->end.x == Approx(1.0F));
        CHECK(segment->end.y == Approx(0.25F));
    }

    SECTION("two off-screen endpoints retain a crossing segment") {
        const auto segment = invisible_places::camera::
            ProjectWorldSegmentToNdc(
                identity,
                {-2.0F, 0.0F, 0.0F},
                {2.0F, 0.0F, 0.0F});
        REQUIRE(segment.has_value());
        CHECK(segment->start.x == Approx(-1.0F));
        CHECK(segment->end.x == Approx(1.0F));
    }

    SECTION("an endpoint behind the viewer clips at the near plane") {
        invisible_places::camera::OrbitCamera camera;
        const auto matrices = camera.Matrices(1.0F);
        const glm::vec3 awayFromTarget = glm::normalize(
            matrices.position - camera.Target());
        const glm::vec3 behindViewer =
            matrices.position + awayFromTarget;
        const auto segment = invisible_places::camera::
            ProjectWorldSegmentToNdc(
                matrices.viewProjection,
                camera.Target(),
                behindViewer);
        REQUIRE(segment.has_value());
        CHECK(segment->end.x == Approx(0.0F).margin(1.0e-4F));
        CHECK(segment->end.y == Approx(0.0F).margin(1.0e-4F));
        CHECK(segment->end.z == Approx(-1.0F).margin(1.0e-4F));
    }

    SECTION("a segment outside one frustum side is rejected") {
        CHECK_FALSE(invisible_places::camera::ProjectWorldSegmentToNdc(
            identity,
            {1.5F, -0.5F, 0.0F},
            {2.0F, 0.5F, 0.0F}));
    }
}

TEST_CASE("Timeline camera following preserves an orbited inspection view",
          "[camera][animation][feature-timeline][playback][scrub]") {
    CHECK_FALSE(invisible_places::camera::AnimationCameraMatchesFrame(
        {},
        {},
        0.5F));
    CHECK_FALSE(invisible_places::camera::
                    FeatureTimelineScrubShouldMoveCamera(
                        {},
                        {},
                        0.5F,
                        true));

    invisible_places::camera::AnimationPath path;
    path.durationFrames = 90U;
    path.keys = {
        {.cameraPosition = {-2.0F, -6.0F, 2.0F},
         .focusPoint = {0.0F, 0.0F, 1.0F},
         .fovDegrees = 52.0F,
         .durationFrames = 90U},
        {.cameraPosition = {2.0F, -5.0F, 2.5F},
         .focusPoint = {0.5F, 0.0F, 1.2F},
         .fovDegrees = 58.0F,
         .durationFrames = 90U},
    };
    constexpr float kPosition = 0.4F;
    const auto evaluation = invisible_places::camera::EvaluateAnimationPath(
        path,
        invisible_places::camera::AnimationPathDurationSeconds(path) *
            kPosition);
    invisible_places::camera::OrbitCamera liveCamera;
    liveCamera.ApplyState(evaluation.camera);
    const auto attached = liveCamera.CaptureState();

    CHECK(invisible_places::camera::AnimationCameraMatchesFrame(
        attached,
        path,
        kPosition));
    CHECK(invisible_places::camera::AnimationPlaybackShouldFollowCamera(
        attached,
        path,
        kPosition,
        true));
    CHECK(invisible_places::camera::
              FeatureTimelineScrubShouldMoveCamera(
                  attached,
                  path,
                  kPosition,
                  false));

    liveCamera.Orbit(24.0F, -8.0F);
    const auto orbited = liveCamera.CaptureState();
    CHECK_FALSE(invisible_places::camera::AnimationCameraMatchesFrame(
        orbited,
        path,
        kPosition));
    CHECK_FALSE(invisible_places::camera::AnimationPlaybackShouldFollowCamera(
        orbited,
        path,
        kPosition,
        true));
    CHECK_FALSE(invisible_places::camera::AnimationPlaybackShouldFollowCamera(
        attached,
        path,
        kPosition,
        false));
    CHECK_FALSE(invisible_places::camera::
                    FeatureTimelineScrubShouldMoveCamera(
                        orbited,
                        path,
                        kPosition,
                        false));
    CHECK(invisible_places::camera::
              FeatureTimelineScrubShouldMoveCamera(
                  orbited,
                  path,
                  kPosition,
                  true));

    auto changedLens = attached;
    changedLens.fovDegrees += 0.1F;
    CHECK_FALSE(invisible_places::camera::
                    FeatureTimelineScrubShouldMoveCamera(
                        changedLens,
                        path,
                        kPosition,
                        false));

    auto rolled = attached;
    const glm::quat originalOrientation{
        rolled.orientation[3U],
        rolled.orientation[0U],
        rolled.orientation[1U],
        rolled.orientation[2U],
    };
    const glm::quat rolledOrientation = glm::normalize(
        glm::angleAxis(
            glm::radians(2.0F),
            glm::vec3{0.0F, 0.0F, 1.0F}) *
        originalOrientation);
    rolled.orientation = {
        rolledOrientation.x,
        rolledOrientation.y,
        rolledOrientation.z,
        rolledOrientation.w,
    };
    CHECK_FALSE(invisible_places::camera::
                    FeatureTimelineScrubShouldMoveCamera(
                        rolled,
                        path,
                        kPosition,
                        false));

    auto freeOrientationPath = path;
    for (auto& key : freeOrientationPath.keys) {
        key.hasOrientation = true;
        key.orientation = {0.0F, 0.0F, 0.0F, 1.0F};
    }
    const auto freeEvaluation = invisible_places::camera::
        EvaluateAnimationPath(
            freeOrientationPath,
            invisible_places::camera::AnimationPathDurationSeconds(
                freeOrientationPath) *
                kPosition);
    invisible_places::camera::OrbitCamera freeCamera;
    freeCamera.ApplyState(freeEvaluation.camera);
    CHECK(invisible_places::camera::
              FeatureTimelineScrubShouldMoveCamera(
                  freeCamera.CaptureState(),
                  freeOrientationPath,
                  kPosition,
                  false));
}

TEST_CASE("Focus Z rotation orbits the camera on XY without changing height or distance",
          "[camera][animation][key-editing][focus-orbit]") {
    const std::array<float, 3> focus{1.0F, 2.0F, 3.0F};
    const std::array<float, 3> camera{1.0F, 4.0F, 8.0F};
    const auto rotated = invisible_places::camera::
        RotateAnimationCameraPositionAboutFocusZ(
            camera,
            focus,
            0.5F * 3.14159265358979323846F);

    CHECK(rotated[0U] == Approx(-1.0F));
    CHECK(rotated[1U] == Approx(2.0F));
    CHECK(rotated[2U] == Approx(camera[2U]));
    const auto distance = [](const auto& left, const auto& right) {
        return std::hypot(
            left[0U] - right[0U],
            std::hypot(
                left[1U] - right[1U],
                left[2U] - right[2U]));
    };
    CHECK(distance(rotated, focus) == Approx(distance(camera, focus)));
}

TEST_CASE("Live camera snapshots update a key's authored pose and lens channels",
          "[camera][animation][key-editing][live-camera]") {
    invisible_places::camera::AnimationPath path;
    path.keys = {
        {.cameraPosition = {0.0F, 0.0F, 4.0F},
         .focusPoint = {0.0F, 0.0F, 0.0F}},
        {.cameraPosition = {2.0F, 0.0F, 4.0F},
         .focusPoint = {1.0F, 0.0F, 0.0F},
         .hasOrientation = true,
         .hasFocusDistance = true,
         .focusDistance = 4.0F,
         .hasApertureFStops = true,
         .apertureFStops = 5.6F},
    };
    invisible_places::camera::CameraState cameraState;
    cameraState.position = {5.0F, 6.0F, 7.0F};
    cameraState.orientation = {
        0.0F,
        0.0F,
        0.70710677F,
        0.70710677F,
    };
    cameraState.fovDegrees = 38.0F;
    cameraState.nearPlane = 0.025F;
    cameraState.farPlane = 420.0F;
    cameraState.focusDistance = 9.5F;
    cameraState.apertureFStops = 2.8F;
    const std::array<float, 3> focus{8.0F, 9.0F, 10.0F};

    invisible_places::camera::UpdateAnimationPathKeyFromCameraState(
        &path,
        0U,
        cameraState,
        focus,
        cameraState.focusDistance);

    const auto& updated = path.keys[0U];
    CHECK(updated.cameraPosition == cameraState.position);
    CHECK(updated.focusPoint == focus);
    CHECK(updated.hasOrientation);
    CHECK(updated.orientation == cameraState.orientation);
    CHECK(updated.hasFocusDistance);
    CHECK(updated.focusDistance == Approx(9.5F));
    CHECK(updated.hasApertureFStops);
    CHECK(updated.apertureFStops == Approx(2.8F));
    CHECK(updated.fovDegrees == Approx(38.0F));
    CHECK(updated.nearPlane == Approx(0.025F));
    CHECK(updated.farPlane == Approx(420.0F));
}

TEST_CASE("Camera alignment paste keeps the destination focus and copies spherical framing",
          "[camera][animation][key-editing][alignment-clipboard]") {
    invisible_places::camera::AnimationPath source;
    source.keys = {
        {.id = "source",
         .cameraPosition = {3.0F, 4.0F, 12.0F},
         .focusPoint = {0.0F, 0.0F, 0.0F},
         .hasOrientation = true,
         .orientation = {0.0F, 0.0F, 0.38268343F, 0.92387950F}},
    };
    const auto alignment = invisible_places::camera::
        CaptureAnimationCameraAlignment(source, 0U);
    REQUIRE(alignment.has_value());
    CHECK(alignment->cameraToFocusDistance == Approx(13.0F));
    CHECK(alignment->worldAzimuthRadians ==
          Approx(std::atan2(4.0F, 3.0F)));
    CHECK(alignment->polarAngleRadians ==
          Approx(std::atan2(5.0F, 12.0F)));

    // Clipboard records are values, not a live link back to the source key.
    source.keys[0U].cameraPosition = {-20.0F, -30.0F, -40.0F};

    invisible_places::camera::AnimationPath destination;
    destination.keys = {
        {.id = "destination",
         .cameraPosition = {9.0F, 8.0F, 7.0F},
         .focusPoint = {10.0F, 20.0F, 30.0F},
         .hasOrientation = true,
         .orientation = {0.0F, 0.0F, 0.0F, 1.0F}},
        {.id = "orientation-channel",
         .cameraPosition = {1.0F, 0.0F, 0.0F},
         .focusPoint = {0.0F, 0.0F, 0.0F},
         .hasOrientation = true},
    };
    const auto focusBefore = destination.keys[0U].focusPoint;
    REQUIRE(invisible_places::camera::ApplyAnimationCameraAlignment(
        &destination,
        0U,
        alignment.value()));

    const auto& pasted = destination.keys[0U];
    CHECK(pasted.focusPoint == focusBefore);
    CHECK(pasted.cameraPosition[0U] == Approx(13.0F));
    CHECK(pasted.cameraPosition[1U] == Approx(24.0F));
    CHECK(pasted.cameraPosition[2U] == Approx(42.0F));
    CHECK(pasted.orientation[2U] == Approx(0.38268343F));
    CHECK(pasted.orientation[3U] == Approx(0.92387950F));
}

TEST_CASE("Camera alignment paste keeps target-driven paths target-driven",
          "[camera][animation][key-editing][alignment-clipboard]") {
    invisible_places::camera::AnimationPath path;
    path.keys = {
        {.cameraPosition = {0.0F, 2.0F, 1.0F},
         .focusPoint = {1.0F, 1.0F, 1.0F}},
    };
    const invisible_places::camera::AnimationCameraAlignment alignment{
        .cameraToFocusDistance = 5.0F,
        .polarAngleRadians = 1.57079632679F,
        .worldAzimuthRadians = 0.0F,
        .orientation = {0.0F, 0.0F, 0.70710677F, 0.70710677F},
    };
    REQUIRE(invisible_places::camera::ApplyAnimationCameraAlignment(
        &path,
        0U,
        alignment));
    const std::array<float, 3> expectedCamera{6.0F, 1.0F, 1.0F};
    CHECK(path.keys[0U].cameraPosition == expectedCamera);
    CHECK_FALSE(path.keys[0U].hasOrientation);
}

TEST_CASE("Camera alignment normalizes orbit to path tangent and local ground",
          "[camera][animation][key-editing][alignment-clipboard]") {
    invisible_places::camera::AnimationPath source;
    source.keys = {
        {.id = "source-0",
         .cameraPosition = {0.0F, -4.0F, 5.0F},
         .focusPoint = {0.0F, 0.0F, 2.0F}},
        {.id = "source-1",
         .cameraPosition = {10.0F, -4.0F, 5.0F},
         .focusPoint = {10.0F, 0.0F, 2.0F}},
        {.id = "source-2",
         .cameraPosition = {20.0F, -4.0F, 5.0F},
         .focusPoint = {20.0F, 0.0F, 2.0F}},
    };
    const auto alignment = invisible_places::camera::
        CaptureAnimationCameraAlignment(source, 1U, 1.0F);
    REQUIRE(alignment.has_value());
    REQUIRE(alignment->hasPathTangentAngle);
    REQUIRE(alignment->hasGroundHeight);
    CHECK(alignment->cameraToFocusDistance == Approx(5.0F));
    CHECK(alignment->polarAngleRadians ==
          Approx(std::atan2(4.0F, 3.0F)));
    CHECK(alignment->pathTangentAngleRadians ==
          Approx(1.57079632679F));
    CHECK(alignment->cameraHeightAboveGround == Approx(4.0F));
    CHECK(alignment->focusHeightAboveGround == Approx(1.0F));

    invisible_places::camera::AnimationPath destination;
    destination.keys = {
        {.id = "destination-0",
         .cameraPosition = {-4.0F, 0.0F, 4.0F},
         .focusPoint = {0.0F, 0.0F, 1.0F}},
        {.id = "destination-1",
         .cameraPosition = {-4.0F, 10.0F, 4.0F},
         .focusPoint = {0.0F, 10.0F, 1.0F}},
        {.id = "destination-2",
         .cameraPosition = {-4.0F, 20.0F, 4.0F},
         .focusPoint = {0.0F, 20.0F, 1.0F}},
    };
    const invisible_places::camera::AnimationCameraAlignmentPasteOptions
        options{
            .cameraToFocusDistance = true,
            .polarAngleToFocus = true,
            .cameraHeightAboveGround = true,
            .focusHeightAboveGround = true,
            .angleFromPathTangent = true,
            .horizonAndRoll = false,
            .destinationGroundHeight = 10.0F,
        };
    REQUIRE(invisible_places::camera::ApplyAnimationCameraAlignment(
        &destination,
        1U,
        alignment.value(),
        options));

    const auto& pasted = destination.keys[1U];
    CHECK(pasted.focusPoint[0U] == Approx(0.0F));
    CHECK(pasted.focusPoint[1U] == Approx(10.0F));
    CHECK(pasted.focusPoint[2U] == Approx(11.0F));
    CHECK(pasted.cameraPosition[0U] == Approx(4.0F));
    CHECK(pasted.cameraPosition[1U] == Approx(10.0F));
    CHECK(pasted.cameraPosition[2U] == Approx(14.0F));
}

TEST_CASE("Camera alignment paste changes only selected framing components",
          "[camera][animation][key-editing][alignment-clipboard]") {
    invisible_places::camera::AnimationPath destination;
    destination.keys = {
        {.id = "destination",
         .cameraPosition = {3.0F, 4.0F, 12.0F},
         .focusPoint = {0.0F, 0.0F, 0.0F}},
    };
    const auto before = invisible_places::camera::
        CaptureAnimationCameraAlignment(destination, 0U);
    REQUIRE(before.has_value());
    const invisible_places::camera::AnimationCameraAlignment source{
        .cameraToFocusDistance = 10.0F,
        .polarAngleRadians = 0.25F,
        .worldAzimuthRadians = -1.0F,
    };
    const invisible_places::camera::AnimationCameraAlignmentPasteOptions
        options{
            .cameraToFocusDistance = true,
            .polarAngleToFocus = false,
            .cameraHeightAboveGround = false,
            .focusHeightAboveGround = false,
            .angleFromPathTangent = false,
            .horizonAndRoll = false,
        };
    REQUIRE(invisible_places::camera::ApplyAnimationCameraAlignment(
        &destination,
        0U,
        source,
        options));
    const auto after = invisible_places::camera::
        CaptureAnimationCameraAlignment(destination, 0U);
    REQUIRE(after.has_value());
    CHECK(after->cameraToFocusDistance == Approx(10.0F));
    CHECK(after->polarAngleRadians ==
          Approx(before->polarAngleRadians));
    CHECK(after->worldAzimuthRadians ==
          Approx(before->worldAzimuthRadians));
    CHECK(destination.keys[0U].focusPoint ==
          std::array<float, 3>{0.0F, 0.0F, 0.0F});
}

TEST_CASE("Shared camera rig handle sits one third of the way from focus to camera",
          "[camera][animation][key-editing][focus-rig-gizmo]") {
    const auto handle = invisible_places::camera::
        ResolveAnimationCameraRigHandlePoint(
            {4.0F, 8.0F, 12.0F},
            {1.0F, 2.0F, 3.0F});
    REQUIRE(handle.has_value());
    CHECK(handle->at(0U) == Approx(2.0F));
    CHECK(handle->at(1U) == Approx(4.0F));
    CHECK(handle->at(2U) == Approx(6.0F));

    CHECK_FALSE(invisible_places::camera::
        ResolveAnimationCameraRigHandlePoint(
            {1.0F, 2.0F, 3.0F},
            {1.0F, 2.0F, 3.0F})
            .has_value());
}

TEST_CASE("Focus rig translation moves camera and focus together without changing framing",
          "[camera][animation][key-editing][focus-rig-gizmo]") {
    invisible_places::camera::AnimationPath path;
    path.keys = {
        {.id = "rig-key",
         .cameraPosition = {1.0F, 2.0F, 3.0F},
         .focusPoint = {4.0F, 5.0F, 6.0F},
         .hasOrientation = true,
         .orientation = {0.0F, 0.0F, 0.38268343F, 0.92387950F},
         .hasFocusDistance = true,
         .focusDistance = 7.0F},
    };
    path.localizedKeyCorrections = {{
        .keyId = "rig-key",
        .splineCameraPosition = {0.5F, 1.5F, 2.5F},
        .splineFocusPoint = {3.5F, 4.5F, 5.5F},
    }};
    const auto offsetBefore = std::array<float, 3>{
        path.keys[0U].cameraPosition[0U] - path.keys[0U].focusPoint[0U],
        path.keys[0U].cameraPosition[1U] - path.keys[0U].focusPoint[1U],
        path.keys[0U].cameraPosition[2U] - path.keys[0U].focusPoint[2U],
    };

    invisible_places::camera::TranslateAnimationCameraAndFocusKey(
        &path,
        0U,
        {-2.0F, 3.0F, 1.5F});

    const std::array<float, 3> expectedCamera{-1.0F, 5.0F, 4.5F};
    const std::array<float, 3> expectedFocus{2.0F, 8.0F, 7.5F};
    CHECK(path.keys[0U].cameraPosition == expectedCamera);
    CHECK(path.keys[0U].focusPoint == expectedFocus);
    CHECK(path.keys[0U].orientation[2U] == Approx(0.38268343F));
    CHECK(path.keys[0U].orientation[3U] == Approx(0.92387950F));
    CHECK(path.keys[0U].focusDistance == Approx(7.0F));
    CHECK(path.localizedKeyCorrections.empty());
    const std::array<float, 3> offsetAfter{
        path.keys[0U].cameraPosition[0U] - path.keys[0U].focusPoint[0U],
        path.keys[0U].cameraPosition[1U] - path.keys[0U].focusPoint[1U],
        path.keys[0U].cameraPosition[2U] - path.keys[0U].focusPoint[2U],
    };
    CHECK(offsetAfter == offsetBefore);
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

TEST_CASE(
    "Pair fixed-lens repair normalizes every lens channel without moving the paths",
    "[camera][animation][fixed-lens][pan-extension]") {
    auto first = MakeReciprocalPanTestPath("lens-A", 1.0F, 41U, 79U);
    auto second = MakeReciprocalPanTestPath("lens-B", -1.0F, 54U, 96U);
    const std::array<float, 3U> firstFov{54.0F, 56.0F, 55.0F};
    const std::array<float, 3U> secondFov{62.0F, 60.0F, 61.0F};
    for (std::size_t index = 0U; index < first.keys.size(); ++index) {
        first.keys[index].fovDegrees = firstFov[index];
        second.keys[index].fovDegrees = secondFov[index];
    }
    first.keys[0U].hasFocusDistance = true;
    first.keys[0U].focusDistance = 8.0F;
    first.keys[2U].hasFocusDistance = true;
    first.keys[2U].focusDistance = 12.0F;
    first.keys[0U].hasApertureFStops = true;
    first.keys[0U].apertureFStops = 4.0F;
    first.keys[2U].hasApertureFStops = true;
    first.keys[2U].apertureFStops = 8.0F;
    first.keys[0U].nearPlane = 0.02F;
    second.keys[1U].nearPlane = 0.005F;
    for (auto& key : first.keys) {
        key.farPlane = 120.0F;
    }
    for (auto& key : second.keys) {
        key.farPlane = 140.0F;
    }
    first.keys[1U].farPlane = 150.0F;
    second.keys[2U].farPlane = 220.0F;
    for (auto* path : {&first, &second}) {
        path->keys.front().hasSplineEndpointTangent = true;
        path->keys.front().splineLensEndpointTangent = {
            0.25F,
            0.01F,
            -2.0F,
            0.5F,
            0.75F,
        };
        path->keys.back().hasSplineEndpointTangent = true;
        path->keys.back().splineLensEndpointTangent = {
            -0.20F,
            -0.01F,
            3.0F,
            -0.4F,
            -0.6F,
        };
    }
    const auto firstBefore = first;
    const auto secondBefore = second;
    const auto firstBeforeEvaluation = invisible_places::camera::
        PrepareAnimationPathEvaluation(firstBefore);
    const auto secondBeforeEvaluation = invisible_places::camera::
        PrepareAnimationPathEvaluation(secondBefore);
    REQUIRE(firstBeforeEvaluation.valid);
    REQUIRE(secondBeforeEvaluation.valid);

    const auto repaired = invisible_places::camera::
        BuildAnimationFixedLensNormalization(first, second);
    INFO(repaired.errorMessage);
    REQUIRE(repaired.succeeded);
    REQUIRE(repaired.changed);
    CHECK(repaired.nearPlane == Approx(0.005F));
    CHECK(repaired.farPlane == Approx(220.0F));
    CHECK(repaired.profiles[0U].fovDegrees == Approx(55.0F));
    CHECK(repaired.profiles[0U].hasFocusDistance);
    CHECK(repaired.profiles[0U].focusDistance == Approx(10.0F));
    CHECK(repaired.profiles[0U].hasApertureFStops);
    CHECK(repaired.profiles[0U].apertureFStops == Approx(6.0F));
    CHECK(repaired.profiles[1U].fovDegrees == Approx(61.0F));
    CHECK_FALSE(repaired.profiles[1U].hasFocusDistance);
    CHECK_FALSE(repaired.profiles[1U].hasApertureFStops);

    for (std::size_t role = 0U; role < 2U; ++role) {
        const auto& candidate = role == 0U
            ? repaired.firstCandidate
            : repaired.secondCandidate;
        const auto& original = role == 0U ? firstBefore : secondBefore;
        REQUIRE(candidate.keys.size() == original.keys.size());
        CHECK(candidate.durationFrames == original.durationFrames);
        CHECK(candidate.exportSettings.startFrame ==
              original.exportSettings.startFrame);
        CHECK(candidate.exportSettings.endFrame ==
              original.exportSettings.endFrame);
        for (std::size_t index = 0U; index < candidate.keys.size(); ++index) {
            const auto& key = candidate.keys[index];
            CHECK(key.cameraPosition == original.keys[index].cameraPosition);
            CHECK(key.focusPoint == original.keys[index].focusPoint);
            CHECK(key.durationFrames == original.keys[index].durationFrames);
            CHECK(key.fovDegrees ==
                  Approx(repaired.profiles[role].fovDegrees));
            CHECK(key.nearPlane == Approx(0.005F));
            CHECK(key.farPlane == Approx(220.0F));
            CHECK(std::all_of(
                key.splineLensEndpointTangent.begin(),
                key.splineLensEndpointTangent.end(),
                [](float value) { return value == 0.0F; }));
        }
    }
    for (const auto& key : repaired.firstCandidate.keys) {
        CHECK(key.hasFocusDistance);
        CHECK(key.focusDistance == Approx(10.0F));
        CHECK(key.hasApertureFStops);
        CHECK(key.apertureFStops == Approx(6.0F));
    }
    for (const auto& key : repaired.secondCandidate.keys) {
        CHECK_FALSE(key.hasFocusDistance);
        CHECK_FALSE(key.hasApertureFStops);
    }
    const auto firstAfterEvaluation = invisible_places::camera::
        PrepareAnimationPathEvaluation(repaired.firstCandidate);
    const auto secondAfterEvaluation = invisible_places::camera::
        PrepareAnimationPathEvaluation(repaired.secondCandidate);
    REQUIRE(firstAfterEvaluation.valid);
    REQUIRE(secondAfterEvaluation.valid);
    for (std::uint32_t sample = 0U; sample <= 64U; ++sample) {
        const float amount = static_cast<float>(sample) / 64.0F;
        for (const auto& [before, after] : {
                 std::pair{&firstBeforeEvaluation, &firstAfterEvaluation},
                 std::pair{&secondBeforeEvaluation, &secondAfterEvaluation},
             }) {
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
        }
    }
    const auto extension = invisible_places::camera::
        BuildAnimationBidirectionalReciprocalPanExtension(
            repaired.firstCandidate,
            repaired.secondCandidate,
            MakeReciprocalPanTestOptions());
    INFO(extension.errorMessage);
    REQUIRE(extension.succeeded);

    const auto alreadyFixed = invisible_places::camera::
        BuildAnimationFixedLensNormalization(
            repaired.firstCandidate,
            repaired.secondCandidate);
    REQUIRE(alreadyFixed.succeeded);
    CHECK_FALSE(alreadyFixed.changed);

    const auto swapped = invisible_places::camera::
        BuildAnimationFixedLensNormalization(second, first);
    REQUIRE(swapped.succeeded);
    CHECK(swapped.nearPlane == Approx(repaired.nearPlane));
    CHECK(swapped.farPlane == Approx(repaired.farPlane));
    CHECK(swapped.profiles[0U].fovDegrees ==
          Approx(repaired.profiles[1U].fovDegrees));
    CHECK(swapped.profiles[1U].fovDegrees ==
          Approx(repaired.profiles[0U].fovDegrees));

    auto invalid = first;
    invalid.keys[1U].fovDegrees =
        std::numeric_limits<float>::quiet_NaN();
    const auto invalidResult = invisible_places::camera::
        BuildAnimationFixedLensNormalization(invalid, second);
    CHECK_FALSE(invalidResult.succeeded);
    CHECK_FALSE(invalidResult.changed);
    CHECK(invalidResult.firstCandidate.keys.empty());
    CHECK(std::isnan(invalid.keys[1U].fovDegrees));
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
    "Reciprocal pan loop retime preserves aligned geometry and makes an exact four-minute cycle",
    "[camera][animation][pan-extension][loop-retime]") {
    auto firstBaseline =
        MakeReciprocalPanTestPath("retime-A", 1.0F, 41U, 79U);
    auto secondBaseline =
        MakeReciprocalPanTestPath("retime-B", -1.0F, 54U, 96U);
    AddLegacyWaterTracks(&firstBaseline);
    AddLegacyWaterTracks(&secondBaseline);
    const auto extended = invisible_places::camera::
        BuildAnimationBidirectionalReciprocalPanExtension(
            firstBaseline,
            secondBaseline,
            MakeReciprocalPanTestOptions());
    INFO(extended.errorMessage);
    REQUIRE(extended.succeeded);

    invisible_places::camera::AnimationReciprocalLoopDurationRetimeOptions
        options;
    options.targetCycleFrames = 4U * 60U * 30U;
    options.seamHalfFrames = {35U, 44U};
    const auto result = invisible_places::camera::
        BuildAnimationReciprocalLoopDurationRetime(
            extended.firstCandidate,
            extended.secondCandidate,
            options);
    INFO(result.errorMessage);
    REQUIRE(result.succeeded);
    REQUIRE(result.changed);
    CHECK(result.metrics.originalCycleFrames == 270U);
    CHECK(result.metrics.targetCycleFrames == 7200U);
    CHECK(result.metrics.timeScale == Approx(7200.0F / 270.0F));
    CHECK(result.metrics.originalBulkFrames ==
          std::array<std::uint32_t, 2U>{120U, 150U});
    CHECK(result.metrics.retimedBulkFrames ==
          std::array<std::uint32_t, 2U>{3200U, 4000U});
    CHECK(result.metrics.retimedSeamHalfFrames ==
          std::array<std::uint32_t, 2U>{933U, 1173U});
    CHECK(result.firstCandidate.durationFrames == 5306U);
    CHECK(result.secondCandidate.durationFrames == 6106U);
    CHECK(result.firstCandidate.exportSettings.startFrame == 1120U);
    CHECK(result.firstCandidate.exportSettings.endFrame == 3360U);
    CHECK(result.secondCandidate.exportSettings.startFrame == 1360U);
    CHECK(result.secondCandidate.exportSettings.endFrame == 3600U);
    CHECK(
        result.firstCandidate.durationFrames +
            result.secondCandidate.durationFrames -
            2U * (result.metrics.retimedSeamHalfFrames[0U] +
                  result.metrics.retimedSeamHalfFrames[1U]) ==
        7200U);

    const auto checkPath = [](
                               const auto& original,
                               const auto& retimed,
                               std::uint32_t expectedDuration) {
        REQUIRE(retimed.keys.size() == original.keys.size());
        CHECK(retimed.durationFrames == expectedDuration);
        std::uint64_t durationSum = 0U;
        for (std::size_t index = 0U; index < original.keys.size(); ++index) {
            const auto& before = original.keys[index];
            const auto& after = retimed.keys[index];
            CHECK(after.id == before.id);
            CHECK(after.cameraPosition == before.cameraPosition);
            CHECK(after.focusPoint == before.focusPoint);
            CHECK(after.orientation == before.orientation);
            CHECK(after.fovDegrees == before.fovDegrees);
            CHECK(after.nearPlane == before.nearPlane);
            CHECK(after.farPlane == before.farPlane);
            CHECK(after.splineParameterWeight ==
                  before.splineParameterWeight);
            CHECK(after.splineCameraEndpointTangent ==
                  before.splineCameraEndpointTangent);
            CHECK(after.splineFocusEndpointTangent ==
                  before.splineFocusEndpointTangent);
            if (index > 0U) {
                CHECK(after.durationFrames >= 1U);
                durationSum += after.durationFrames;
            }
            const auto beforeAtKey = invisible_places::camera::
                EvaluateAnimationPath(
                    original,
                    invisible_places::camera::AnimationPathDurationSeconds(
                        original) *
                        invisible_places::camera::
                            AnimationPathKeyNormalizedPosition(
                                original,
                                index));
            const auto afterAtKey = invisible_places::camera::
                EvaluateAnimationPath(
                    retimed,
                    invisible_places::camera::AnimationPathDurationSeconds(
                        retimed) *
                        invisible_places::camera::
                            AnimationPathKeyNormalizedPosition(
                                retimed,
                                index));
            CheckPanEvaluationNear(beforeAtKey, afterAtKey, 4.0e-4F);
        }
        CHECK(durationSum == expectedDuration);
    };
    checkPath(
        extended.firstCandidate,
        result.firstCandidate,
        5306U);
    checkPath(
        extended.secondCandidate,
        result.secondCandidate,
        6106U);
    REQUIRE(result.firstCandidate.waterScenarioTracks.size() == 1U);
    REQUIRE(result.secondCandidate.waterScenarioTracks.size() == 1U);
    const auto& firstWater =
        result.firstCandidate.waterScenarioTracks.front();
    const auto& secondWater =
        result.secondCandidate.waterScenarioTracks.front();
    CHECK(firstWater.keys[0U].position ==
          Approx((933.0F + 0.25F * 3200.0F) / 5306.0F));
    CHECK(firstWater.keys[1U].position ==
          Approx((933.0F + 3200.0F) / 5306.0F));
    CHECK(secondWater.keys[0U].position ==
          Approx((1173.0F + 0.25F * 4000.0F) / 6106.0F));
    CHECK(secondWater.keys[1U].position ==
          Approx((1173.0F + 4000.0F) / 6106.0F));

    const auto sumPrefix = [](const auto& path, std::size_t segmentCount) {
        std::uint32_t frames = 0U;
        for (std::size_t segment = 0U; segment < segmentCount; ++segment) {
            frames += path.keys[segment + 1U].durationFrames;
        }
        return frames;
    };
    const auto sumSuffix = [](const auto& path, std::size_t segmentCount) {
        std::uint32_t frames = 0U;
        for (std::size_t segment = 0U; segment < segmentCount; ++segment) {
            frames += path.keys[path.keys.size() - 1U - segment]
                          .durationFrames;
        }
        return frames;
    };
    CHECK(sumPrefix(
              result.firstCandidate,
              extended.metrics.incoming.appendedKeyCount[0U]) == 933U);
    CHECK(sumSuffix(
              result.firstCandidate,
              extended.metrics.outgoing.appendedKeyCount[0U]) == 1173U);
    CHECK(sumPrefix(
              result.secondCandidate,
              extended.metrics.incoming.appendedKeyCount[1U]) == 1173U);
    CHECK(sumSuffix(
              result.secondCandidate,
              extended.metrics.outgoing.appendedKeyCount[1U]) == 933U);

    auto swappedOptions = options;
    swappedOptions.seamHalfFrames = {44U, 35U};
    const auto swapped = invisible_places::camera::
        BuildAnimationReciprocalLoopDurationRetime(
            extended.secondCandidate,
            extended.firstCandidate,
            swappedOptions);
    INFO(swapped.errorMessage);
    REQUIRE(swapped.succeeded);
    CHECK(swapped.firstCandidate.durationFrames ==
          result.secondCandidate.durationFrames);
    CHECK(swapped.secondCandidate.durationFrames ==
          result.firstCandidate.durationFrames);
    for (std::size_t index = 1U;
         index < result.firstCandidate.keys.size();
         ++index) {
        CHECK(swapped.secondCandidate.keys[index].durationFrames ==
              result.firstCandidate.keys[index].durationFrames);
    }
    for (std::size_t index = 1U;
         index < result.secondCandidate.keys.size();
         ++index) {
        CHECK(swapped.firstCandidate.keys[index].durationFrames ==
              result.secondCandidate.keys[index].durationFrames);
    }

    auto fasterOptions = options;
    fasterOptions.targetCycleFrames = 135U;
    const auto faster = invisible_places::camera::
        BuildAnimationReciprocalLoopDurationRetime(
            extended.firstCandidate,
            extended.secondCandidate,
            fasterOptions);
    INFO(faster.errorMessage);
    REQUIRE(faster.succeeded);
    CHECK(faster.metrics.timeScale == Approx(0.5F));
    CHECK(
        faster.firstCandidate.durationFrames +
            faster.secondCandidate.durationFrames -
            2U * (faster.metrics.retimedSeamHalfFrames[0U] +
                  faster.metrics.retimedSeamHalfFrames[1U]) ==
        135U);
    CHECK(faster.metrics.retimedBulkFrames[0U] == 60U);
    CHECK(faster.metrics.retimedBulkFrames[1U] == 75U);

    auto fullExportFirst = extended.firstCandidate;
    fullExportFirst.exportSettings.endFrame = 0U;
    const auto fullExport = invisible_places::camera::
        BuildAnimationReciprocalLoopDurationRetime(
            fullExportFirst,
            extended.secondCandidate,
            options);
    REQUIRE(fullExport.succeeded);
    CHECK(fullExport.firstCandidate.exportSettings.endFrame == 0U);

    auto impossibleOptions = options;
    impossibleOptions.targetCycleFrames = 1U;
    const auto impossible = invisible_places::camera::
        BuildAnimationReciprocalLoopDurationRetime(
            extended.firstCandidate,
            extended.secondCandidate,
            impossibleOptions);
    CHECK_FALSE(impossible.succeeded);
    CHECK(impossible.firstCandidate.keys.empty());
    CHECK(impossible.secondCandidate.keys.empty());
}

TEST_CASE(
    "Selected transition smoothing moves paired midpoint cameras while preserving triangle alignment",
    "[camera][animation][pan-extension][transition-smoothing]") {
    const auto firstBaseline =
        MakeReciprocalPanTestPath("smooth-A", 1.0F, 41U, 79U);
    const auto secondBaseline =
        MakeReciprocalPanTestPath("smooth-B", -1.0F, 54U, 96U);
    const auto extensionOptions = MakeReciprocalPanTestOptions();
    const auto extension = invisible_places::camera::
        BuildAnimationBidirectionalReciprocalPanExtension(
            firstBaseline,
            secondBaseline,
            extensionOptions);
    INFO(extension.errorMessage);
    REQUIRE(extension.succeeded);
    auto first = extension.firstCandidate;
    auto second = extension.secondCandidate;
    const auto firstBefore = first;
    const auto secondBefore = second;

    const auto movableIds = [](const auto& baseline, const auto& candidate) {
        std::unordered_set<std::string> originalIds;
        for (const auto& key : baseline.keys) {
            originalIds.insert(key.id);
        }
        std::vector<std::string> result;
        for (const auto& key : candidate.keys) {
            if (!originalIds.contains(key.id)) {
                result.push_back(key.id);
            }
        }
        // The sole interior authored key is the nearest available control on
        // both sides of this compact test path.
        result.push_back(baseline.keys.front().id);
        result.push_back(baseline.keys[1U].id);
        result.push_back(baseline.keys.back().id);
        return result;
    };
    invisible_places::camera::AnimationLoopSmoothingOptions options;
    options.maxEndMoveFraction = 0.15F;
    options.useExplicitKeySelection = true;
    options.firstMovableKeyIds = movableIds(firstBaseline, first);
    options.secondMovableKeyIds = movableIds(secondBaseline, second);
    options.firstStartOverlapSeconds = 70.0F / 30.0F;
    options.firstEndOverlapSeconds = 88.0F / 30.0F;
    options.secondStartOverlapSeconds = 88.0F / 30.0F;
    options.secondEndOverlapSeconds = 70.0F / 30.0F;
    options.horizontalBlend = true;
    options.panRight = true;
    options.maxOptimizationSweeps = 16U;
    options.minimumStepFraction = 0.0125F;
    options.imageRotationMismatchWeight = 0.35F;
    options.selectedNeighborhoodSmoothnessWeight = 1.50F;
    options.triangleAlignmentConstraints = {
        {
            .firstKeyId = firstBaseline.keys.front().id,
            .secondKeyId = secondBaseline.keys.back().id,
            .firstPatch = extensionOptions.firstDrivesSecond.sourcePatch,
            .secondPatch =
                extensionOptions.firstDrivesSecond.destinationEndPatch,
        },
        {
            .firstKeyId = firstBaseline.keys.back().id,
            .secondKeyId = secondBaseline.keys.front().id,
            .firstPatch =
                extensionOptions.secondDrivesFirst.destinationEndPatch,
            .secondPatch = extensionOptions.secondDrivesFirst.sourcePatch,
        },
    };
    options.triangleAlignmentWeight = 8.0F;

    const auto smoothing = invisible_places::camera::
        SmoothAnimationLoopTransitions(&first, &second, options);
    INFO(smoothing.errorMessage);
    REQUIRE(smoothing.succeeded);
    REQUIRE(smoothing.changed);
    CHECK(smoothing.afterObjective < smoothing.beforeObjective);
    CHECK(smoothing.afterNeighborhoodRoughness[0U] +
              smoothing.afterNeighborhoodRoughness[1U] <
          smoothing.beforeNeighborhoodRoughness[0U] +
              smoothing.beforeNeighborhoodRoughness[1U]);
    CHECK(smoothing.afterSeamMismatch[0U] <=
          smoothing.beforeSeamMismatch[0U] + 1.0e-5F);
    CHECK(smoothing.afterSeamMismatch[1U] <=
          smoothing.beforeSeamMismatch[1U] + 1.0e-5F);
    CHECK(smoothing.afterSeamRotationMismatch[0U] <=
          smoothing.beforeSeamRotationMismatch[0U] + 1.0e-5F);
    CHECK(smoothing.afterSeamRotationMismatch[1U] <=
          smoothing.beforeSeamRotationMismatch[1U] + 1.0e-5F);
    for (std::size_t seam = 0U; seam < 2U; ++seam) {
        CHECK(smoothing.afterTriangleAlignmentRms[seam] <=
              smoothing.beforeTriangleAlignmentRms[seam] + 2.1e-5F);
        CHECK(smoothing.afterTriangleAlignmentMax[seam] <=
              smoothing.beforeTriangleAlignmentMax[seam] + 3.1e-5F);
    }

    const auto findKey = [](const auto& path, std::string_view id)
        -> const invisible_places::camera::AnimationPathKey& {
        const auto found = std::find_if(
            path.keys.begin(),
            path.keys.end(),
            [&](const auto& key) { return key.id == id; });
        REQUIRE(found != path.keys.end());
        return *found;
    };
    std::size_t movedMidpointCount = 0U;
    for (const auto& [before, after, baseline] : {
             std::tuple{&firstBefore, &first, &firstBaseline},
             std::tuple{&secondBefore, &second, &secondBaseline}}) {
        for (const auto* midpoint : {
                 &baseline->keys.front(),
                 &baseline->keys.back()}) {
            const auto& beforeKey = findKey(*before, midpoint->id);
            const auto& afterKey = findKey(*after, midpoint->id);
            const bool moved =
                afterKey.cameraPosition != beforeKey.cameraPosition ||
                afterKey.focusPoint != beforeKey.focusPoint;
            movedMidpointCount += moved ? 1U : 0U;
        }
        CHECK(after->keys.size() == before->keys.size());
        CHECK(after->durationFrames == before->durationFrames);
    }
    CHECK(movedMidpointCount >= 2U);
    const auto keyMoved = [&](const auto& before,
                              const auto& after,
                              std::string_view id) {
        const auto& beforeKey = findKey(before, id);
        const auto& afterKey = findKey(after, id);
        return beforeKey.cameraPosition != afterKey.cameraPosition ||
               beforeKey.focusPoint != afterKey.focusPoint;
    };
    const bool firstSeamDriverMoved = keyMoved(
        firstBefore,
        first,
        firstBaseline.keys.front().id);
    const bool firstSeamFollowerMoved = keyMoved(
        secondBefore,
        second,
        secondBaseline.keys.back().id);
    const bool secondSeamDriverMoved = keyMoved(
        firstBefore,
        first,
        firstBaseline.keys.back().id);
    const bool secondSeamFollowerMoved = keyMoved(
        secondBefore,
        second,
        secondBaseline.keys.front().id);
    CHECK(firstSeamDriverMoved == firstSeamFollowerMoved);
    CHECK(secondSeamDriverMoved == secondSeamFollowerMoved);
    CHECK((firstSeamDriverMoved || secondSeamDriverMoved));

    // The guided pre-roll review can smooth the B-end -> A-start seam before
    // the opposite correspondence exists. The disabled A-end -> B-start seam
    // must contribute no score or implicit default endpoint neighborhood.
    auto reviewFirst = extension.firstCandidate;
    auto reviewSecond = extension.secondCandidate;
    auto reviewOptions = options;
    reviewOptions.enabledSeams = {false, true};
    reviewOptions.triangleAlignmentConstraints = {
        options.triangleAlignmentConstraints.front()};
    const auto review = invisible_places::camera::
        SmoothAnimationLoopTransitions(
            &reviewFirst,
            &reviewSecond,
            reviewOptions);
    INFO(review.errorMessage);
    REQUIRE(review.succeeded);
    REQUIRE(review.changed);
    CHECK(review.beforeSeamMismatch[0U] == Catch::Approx(0.0F));
    CHECK(review.afterSeamMismatch[0U] == Catch::Approx(0.0F));
    CHECK(review.beforeSeamRotationMismatch[0U] == Catch::Approx(0.0F));
    CHECK(review.afterSeamRotationMismatch[0U] == Catch::Approx(0.0F));
    CHECK(review.afterObjective < review.beforeObjective);
    CHECK(review.afterSeamMismatch[1U] <=
          review.beforeSeamMismatch[1U] + 1.0e-5F);
    CHECK(review.afterTriangleAlignmentRms[0U] <=
          review.beforeTriangleAlignmentRms[0U] + 2.1e-5F);

    auto disabledFirst = extension.firstCandidate;
    auto disabledSecond = extension.secondCandidate;
    const auto disabledFirstBefore = disabledFirst;
    const auto disabledSecondBefore = disabledSecond;
    auto disabledOptions = options;
    disabledOptions.enabledSeams = {false, false};
    const auto disabled = invisible_places::camera::
        SmoothAnimationLoopTransitions(
            &disabledFirst,
            &disabledSecond,
            disabledOptions);
    CHECK_FALSE(disabled.succeeded);
    CHECK_FALSE(disabled.changed);
    const auto unchanged = [](const auto& before, const auto& after) {
        if (before.durationFrames != after.durationFrames ||
            before.keys.size() != after.keys.size()) {
            return false;
        }
        for (std::size_t index = 0U; index < before.keys.size(); ++index) {
            if (before.keys[index].id != after.keys[index].id ||
                before.keys[index].cameraPosition !=
                    after.keys[index].cameraPosition ||
                before.keys[index].focusPoint !=
                    after.keys[index].focusPoint ||
                before.keys[index].durationFrames !=
                    after.keys[index].durationFrames) {
                return false;
            }
        }
        return true;
    };
    CHECK(unchanged(disabledFirstBefore, disabledFirst));
    CHECK(unchanged(disabledSecondBefore, disabledSecond));
}

TEST_CASE(
    "Manual reciprocal smoothing moves only green spatial controls and carries the paired triangle key",
    "[camera][animation][pan-extension][transition-smoothing][manual]") {
    const auto firstBaseline =
        MakeReciprocalPanTestPath("manual-A", 1.0F, 41U, 79U);
    const auto secondBaseline =
        MakeReciprocalPanTestPath("manual-B", -1.0F, 54U, 96U);
    const auto extensionOptions = MakeReciprocalPanTestOptions();
    const auto extension = invisible_places::camera::
        BuildAnimationBidirectionalReciprocalPanExtension(
            firstBaseline,
            secondBaseline,
            extensionOptions);
    INFO(extension.errorMessage);
    REQUIRE(extension.succeeded);
    auto first = extension.firstCandidate;
    auto second = extension.secondCandidate;

    invisible_places::camera::AnimationLoopSmoothingOptions options;
    options.useExplicitKeySelection = true;
    for (const auto& key : first.keys) {
        options.firstMovableKeyIds.push_back(key.id);
    }
    for (const auto& key : second.keys) {
        options.secondMovableKeyIds.push_back(key.id);
    }
    options.triangleAlignmentConstraints = {
        {
            .firstKeyId = firstBaseline.keys.front().id,
            .secondKeyId = secondBaseline.keys.back().id,
            .firstPatch = extensionOptions.firstDrivesSecond.sourcePatch,
            .secondPatch =
                extensionOptions.firstDrivesSecond.destinationEndPatch,
        },
        {
            .firstKeyId = firstBaseline.keys.back().id,
            .secondKeyId = secondBaseline.keys.front().id,
            .firstPatch =
                extensionOptions.secondDrivesFirst.destinationEndPatch,
            .secondPatch = extensionOptions.secondDrivesFirst.sourcePatch,
        },
    };

    const auto findKey = [](auto& path, std::string_view id) -> auto& {
        const auto found = std::find_if(
            path.keys.begin(),
            path.keys.end(),
            [&](const auto& key) { return key.id == id; });
        REQUIRE(found != path.keys.end());
        return *found;
    };
    const auto firstDurations = [&] {
        std::vector<std::uint32_t> result;
        for (const auto& key : first.keys) {
            result.push_back(key.durationFrames);
        }
        return result;
    }();
    const auto secondDurations = [&] {
        std::vector<std::uint32_t> result;
        for (const auto& key : second.keys) {
            result.push_back(key.durationFrames);
        }
        return result;
    }();
    const auto firstDuration = first.durationFrames;
    const auto secondDuration = second.durationFrames;
    const auto triangleScreenRms = [&]() {
        const auto firstKeyIndex = static_cast<std::size_t>(
            std::find_if(
                first.keys.begin(),
                first.keys.end(),
                [&](const auto& key) {
                    return key.id == firstBaseline.keys.front().id;
                }) - first.keys.begin());
        const auto secondKeyIndex = static_cast<std::size_t>(
            std::find_if(
                second.keys.begin(),
                second.keys.end(),
                [&](const auto& key) {
                    return key.id == secondBaseline.keys.back().id;
                }) - second.keys.begin());
        const auto firstEvaluation = invisible_places::camera::
            EvaluateAnimationPath(
                first,
                invisible_places::camera::AnimationPathKeyNormalizedPosition(
                    first,
                    firstKeyIndex) *
                    invisible_places::camera::AnimationPathDurationSeconds(
                        first));
        const auto secondEvaluation = invisible_places::camera::
            EvaluateAnimationPath(
                second,
                invisible_places::camera::AnimationPathKeyNormalizedPosition(
                    second,
                    secondKeyIndex) *
                    invisible_places::camera::AnimationPathDurationSeconds(
                        second));
        invisible_places::camera::OrbitCamera firstCamera;
        invisible_places::camera::OrbitCamera secondCamera;
        firstCamera.ApplyState(firstEvaluation.camera);
        secondCamera.ApplyState(secondEvaluation.camera);
        const auto firstMatrices = firstCamera.Matrices(16.0F / 9.0F);
        const auto secondMatrices = secondCamera.Matrices(16.0F / 9.0F);
        const auto project = [](const auto& matrices,
                                const std::array<float, 3>& point) {
            const glm::vec4 clip = matrices.viewProjection * glm::vec4{
                point[0U], point[1U], point[2U], 1.0F};
            REQUIRE(clip.w > 1.0e-6F);
            return glm::vec2{clip.x / clip.w, clip.y / clip.w};
        };
        float squared = 0.0F;
        for (std::size_t node = 0U; node < 3U; ++node) {
            const auto firstPoint = project(
                firstMatrices,
                extensionOptions.firstDrivesSecond.sourcePatch
                    .worldPoints[node]);
            const auto secondPoint = project(
                secondMatrices,
                extensionOptions.firstDrivesSecond.destinationEndPatch
                    .worldPoints[node]);
            const auto difference = firstPoint - secondPoint;
            squared += glm::dot(difference, difference);
        }
        return std::sqrt(squared / 3.0F);
    };
    const float alignmentBefore = triangleScreenRms();
    const auto driverBefore = findKey(
        first,
        firstBaseline.keys.front().id).cameraPosition;
    const auto followerBefore = findKey(
        second,
        secondBaseline.keys.back().id).cameraPosition;
    const std::array<float, 3> target{
        driverBefore[0U] + 0.04F,
        driverBefore[1U] - 0.02F,
        driverBefore[2U] + 0.01F,
    };
    const auto moved = invisible_places::camera::
        MoveAnimationLoopSelectedKeySpatially(
            &first,
            &second,
            options,
            0U,
            firstBaseline.keys.front().id,
            true,
            target);
    INFO(moved.errorMessage);
    REQUIRE(moved.succeeded);
    REQUIRE(moved.changed);
    CHECK(moved.movedPairedTriangleKey);
    CHECK(findKey(first, firstBaseline.keys.front().id).cameraPosition ==
          target);
    CHECK(findKey(second, secondBaseline.keys.back().id).cameraPosition !=
          followerBefore);
    CHECK(triangleScreenRms() <=
          Catch::Approx(alignmentBefore).margin(2.0e-5F));
    CHECK(first.durationFrames == firstDuration);
    CHECK(second.durationFrames == secondDuration);
    for (std::size_t index = 0U; index < first.keys.size(); ++index) {
        CHECK(first.keys[index].durationFrames == firstDurations[index]);
    }
    for (std::size_t index = 0U; index < second.keys.size(); ++index) {
        CHECK(second.keys[index].durationFrames == secondDurations[index]);
    }

    const auto lockedId = firstBaseline.keys[1U].id;
    options.firstMovableKeyIds.erase(
        std::remove(
            options.firstMovableKeyIds.begin(),
            options.firstMovableKeyIds.end(),
            lockedId),
        options.firstMovableKeyIds.end());
    const auto lockedBefore = findKey(first, lockedId).focusPoint;
    auto lockedTarget = lockedBefore;
    lockedTarget[0U] += 1.0F;
    const auto locked = invisible_places::camera::
        MoveAnimationLoopSelectedKeySpatially(
            &first,
            &second,
            options,
            0U,
            lockedId,
            false,
            lockedTarget);
    CHECK_FALSE(locked.succeeded);
    CHECK_FALSE(locked.changed);
    CHECK(findKey(first, lockedId).focusPoint == lockedBefore);

    // Iterative spatial smoothing or a manual rig edit can leave the two
    // projected midpoint triangles slightly separated. Force alignment keeps
    // the chosen reference path byte-for-byte fixed and re-solves the paired
    // camera/focus controls on the other path without retiming either path.
    const auto firstReferenceBefore = first;
    const float registeredBeforeDrift = triangleScreenRms();
    auto& driftedFollower = findKey(
        second,
        secondBaseline.keys.back().id);
    driftedFollower.cameraPosition[0U] += 0.30F;
    driftedFollower.focusPoint[0U] += 0.30F;
    const float driftedRms = triangleScreenRms();
    REQUIRE(driftedRms > registeredBeforeDrift + 1.0e-4F);

    const auto realigned = invisible_places::camera::
        ForceAlignAnimationLoopSelectedTriangles(
            &first,
            &second,
            options,
            0U);
    INFO(realigned.errorMessage);
    REQUIRE(realigned.succeeded);
    REQUIRE(realigned.changed);
    CHECK(realigned.alignedPairCount == 2U);
    CHECK(realigned.afterRmsScreenHeights <
          realigned.beforeRmsScreenHeights);
    CHECK(triangleScreenRms() < driftedRms);
    REQUIRE(first.keys.size() == firstReferenceBefore.keys.size());
    CHECK(first.localizedKeyCorrections.size() ==
          firstReferenceBefore.localizedKeyCorrections.size());
    for (std::size_t index = 0U; index < first.keys.size(); ++index) {
        CHECK(first.keys[index].id == firstReferenceBefore.keys[index].id);
        CHECK(first.keys[index].cameraPosition ==
              firstReferenceBefore.keys[index].cameraPosition);
        CHECK(first.keys[index].focusPoint ==
              firstReferenceBefore.keys[index].focusPoint);
        CHECK(first.keys[index].durationFrames ==
              firstReferenceBefore.keys[index].durationFrames);
    }
    CHECK(first.durationFrames == firstDuration);
    CHECK(second.durationFrames == secondDuration);
    for (std::size_t index = 0U; index < first.keys.size(); ++index) {
        CHECK(first.keys[index].durationFrames == firstDurations[index]);
    }
    for (std::size_t index = 0U; index < second.keys.size(); ++index) {
        CHECK(second.keys[index].durationFrames == secondDurations[index]);
    }

    const auto secondReferenceBefore = second;
    auto& reverseFollower = findKey(
        first,
        firstBaseline.keys.back().id);
    reverseFollower.cameraPosition[1U] -= 0.25F;
    reverseFollower.focusPoint[1U] -= 0.25F;
    const auto reverseRealigned = invisible_places::camera::
        ForceAlignAnimationLoopSelectedTriangles(
            &first,
            &second,
            options,
            1U);
    INFO(reverseRealigned.errorMessage);
    REQUIRE(reverseRealigned.succeeded);
    REQUIRE(reverseRealigned.changed);
    CHECK(reverseRealigned.afterRmsScreenHeights <
          reverseRealigned.beforeRmsScreenHeights);
    REQUIRE(second.keys.size() == secondReferenceBefore.keys.size());
    for (std::size_t index = 0U; index < second.keys.size(); ++index) {
        CHECK(second.keys[index].cameraPosition ==
              secondReferenceBefore.keys[index].cameraPosition);
        CHECK(second.keys[index].focusPoint ==
              secondReferenceBefore.keys[index].focusPoint);
        CHECK(second.keys[index].durationFrames ==
              secondReferenceBefore.keys[index].durationFrames);
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

TEST_CASE("Linked seam sampling follows both reciprocal overlap bands",
          "[camera][animation][linked-seam]") {
    const auto makePath = [](
                              std::string name,
                              std::string pairId,
                              float startOverlap,
                              float endOverlap) {
        invisible_places::camera::AnimationPath path;
        path.name = std::move(name);
        path.durationFrames = 300U;
        path.keys = {
            {.id = path.name + "-start",
             .cameraPosition = {0.0F, 0.0F, 0.0F},
             .focusPoint = {0.0F, 1.0F, 0.0F}},
            {.id = path.name + "-end",
             .cameraPosition = {1.0F, 0.0F, 0.0F},
             .focusPoint = {1.0F, 1.0F, 0.0F},
             .durationFrames = 300U},
        };
        path.velocityBlendLink = invisible_places::camera::
            AnimationVelocityBlendLinkMetadata{
                .pairId = std::move(pairId),
                .partnerFileName = "partner.ipanim.json",
                .startOverlapSeconds = startOverlap,
                .endOverlapSeconds = endOverlap,
                .horizontalBlend = true,
            };
        return path;
    };
    const auto first = makePath("A", "pair", 2.0F, 3.0F);
    const auto second = makePath("B", "pair", 3.0F, 2.0F);

    const auto start = invisible_places::camera::
        ResolveAnimationLinkedSeamSample(first, second, 0.0F);
    REQUIRE(start.has_value());
    CHECK(start->currentSeamIndex == 0U);
    CHECK(start->partnerNormalizedPosition == Approx(0.8F));
    CHECK(start->overlapProgress == Approx(0.0F));
    CHECK_FALSE(start->currentOnLeft);

    const auto firstMidpoint = invisible_places::camera::
        ResolveAnimationLinkedSeamSample(first, second, 0.1F);
    REQUIRE(firstMidpoint.has_value());
    CHECK(firstMidpoint->partnerNormalizedPosition == Approx(0.9F));
    CHECK(firstMidpoint->overlapProgress == Approx(0.5F));
    CHECK_FALSE(firstMidpoint->currentOnLeft);

    const auto startEnd = invisible_places::camera::
        ResolveAnimationLinkedSeamSample(first, second, 0.2F);
    REQUIRE(startEnd.has_value());
    CHECK(startEnd->partnerNormalizedPosition == Approx(1.0F));
    CHECK(startEnd->overlapProgress == Approx(1.0F));
    CHECK_FALSE(startEnd->currentOnLeft);
    CHECK_FALSE(invisible_places::camera::ResolveAnimationLinkedSeamSample(
                    first,
                    second,
                    0.5F)
                    .has_value());

    const auto endStart = invisible_places::camera::
        ResolveAnimationLinkedSeamSample(first, second, 0.7F);
    REQUIRE(endStart.has_value());
    CHECK(endStart->currentSeamIndex == 1U);
    CHECK(endStart->partnerNormalizedPosition == Approx(0.0F));
    CHECK(endStart->overlapProgress == Approx(0.0F));
    CHECK(endStart->currentOnLeft);

    const auto secondMidpoint = invisible_places::camera::
        ResolveAnimationLinkedSeamSample(first, second, 0.85F);
    REQUIRE(secondMidpoint.has_value());
    CHECK(secondMidpoint->partnerNormalizedPosition == Approx(0.15F));
    CHECK(secondMidpoint->overlapProgress == Approx(0.5F));
    CHECK(secondMidpoint->currentOnLeft);

    const auto end = invisible_places::camera::
        ResolveAnimationLinkedSeamSample(first, second, 1.0F);
    REQUIRE(end.has_value());
    CHECK(end->partnerNormalizedPosition == Approx(0.3F));
    CHECK(end->overlapProgress == Approx(1.0F));
    CHECK(end->currentOnLeft);

    const auto reverseStart = invisible_places::camera::
        ResolveAnimationLinkedSeamSample(second, first, 0.0F);
    REQUIRE(reverseStart.has_value());
    CHECK(reverseStart->partnerNormalizedPosition == Approx(0.7F));
    CHECK(reverseStart->overlapProgress == Approx(0.0F));
    CHECK_FALSE(reverseStart->currentOnLeft);

    auto broken = second;
    broken.velocityBlendLink->pairId = "other-pair";
    CHECK_FALSE(invisible_places::camera::ResolveAnimationLinkedSeamSample(
                    first,
                    broken,
                    0.1F)
                    .has_value());
}

TEST_CASE("Reciprocal timing windows map both camera paths onto one exact loop",
          "[camera][animation][linked-seam][timing-loop]") {
    const auto makePath = [](std::string name,
                             float startOverlap,
                             float endOverlap) {
        invisible_places::camera::AnimationPath path;
        path.name = std::move(name);
        path.durationFrames = 300U;
        path.keys = {
            {.id = path.name + "-start"},
            {.id = path.name + "-end", .durationFrames = 300U},
        };
        path.velocityBlendLink = invisible_places::camera::
            AnimationVelocityBlendLinkMetadata{
                .pairId = "shared-clock",
                .partnerFileName = "partner.ipanim.json",
                .startOverlapSeconds = startOverlap,
                .endOverlapSeconds = endOverlap,
            };
        return path;
    };
    auto first = makePath("A", 2.0F, 3.0F);
    auto second = makePath("B", 3.0F, 2.0F);
    REQUIRE(invisible_places::camera::
                ConfigureAnimationReciprocalTimingLoopWindows(
                    &first,
                    &second));
    REQUIRE(first.velocityBlendLink.has_value());
    REQUIRE(second.velocityBlendLink.has_value());
    CHECK(first.velocityBlendLink->timingCycleFrames == 450U);
    CHECK(second.velocityBlendLink->timingCycleFrames == 450U);
    CHECK(first.velocityBlendLink->timingWindowStartFrame == 0);
    CHECK(second.velocityBlendLink->timingWindowStartFrame == 210);

    const auto phase = [](const auto& path, float local) {
        return invisible_places::camera::AnimationLocalToTimingLoopPosition(
            path,
            local);
    };
    CHECK(phase(first, 0.0F) ==
          Approx(phase(second, 0.8F)).margin(1.0e-6F));
    CHECK(phase(first, 0.2F) ==
          Approx(phase(second, 1.0F)).margin(1.0e-6F));
    CHECK(phase(first, 0.7F) ==
          Approx(phase(second, 0.0F)).margin(1.0e-6F));
    CHECK(phase(first, 1.0F) ==
          Approx(phase(second, 0.3F)).margin(1.0e-6F));

    const auto atLoopZero = invisible_places::camera::
        AnimationTimingLoopPositionToLocalPositions(first, 0.0F);
    const auto partnerAtLoopZero = invisible_places::camera::
        AnimationTimingLoopPositionToLocalPositions(second, 0.0F);
    REQUIRE(atLoopZero.size() == 1U);
    REQUIRE(partnerAtLoopZero.size() == 1U);
    CHECK(atLoopZero.front() == Approx(0.0F));
    CHECK(partnerAtLoopZero.front() == Approx(0.8F));

    auto invalid = second;
    invalid.velocityBlendLink->endOverlapSeconds = 1.0F;
    const auto originalFirst = first.velocityBlendLink.value();
    CHECK_FALSE(invisible_places::camera::
                    ConfigureAnimationReciprocalTimingLoopWindows(
                        &first,
                        &invalid));
    CHECK(first.velocityBlendLink->timingCycleFrames ==
          originalFirst.timingCycleFrames);
}

namespace {

// A pan with several authored keys close to its end so a pre-roll alignment
// ramp has interior controls to ease across.
invisible_places::camera::AnimationPath MakeDenseReciprocalPanTestPath(
    std::string name,
    float direction) {
    invisible_places::camera::AnimationPath path;
    path.name = std::move(name);
    path.durationFrames = 120U;
    path.keys = {
        {.id = "k0",
         .cameraPosition = {-1.6F * direction, -8.0F, 2.0F},
         .focusPoint = {-0.4F * direction, 0.0F, 1.0F}},
        {.id = "k1",
         .cameraPosition = {-1.0F * direction, -8.0F, 2.0F},
         .focusPoint = {-0.2F * direction, 0.0F, 1.0F},
         .durationFrames = 24U},
        {.id = "k2",
         .cameraPosition = {-0.4F * direction, -8.0F, 2.0F},
         .focusPoint = {0.0F, 0.0F, 1.0F},
         .durationFrames = 24U},
        {.id = "k3",
         .cameraPosition = {0.2F * direction, -8.0F, 2.0F},
         .focusPoint = {0.1F * direction, 0.0F, 1.0F},
         .durationFrames = 24U},
        {.id = "k4",
         .cameraPosition = {0.8F * direction, -8.0F, 2.0F},
         .focusPoint = {0.2F * direction, 0.0F, 1.0F},
         .durationFrames = 24U},
        {.id = "k5",
         .cameraPosition = {1.4F * direction, -8.0F, 2.0F},
         .focusPoint = {0.3F * direction, 0.0F, 1.0F},
         .durationFrames = 24U},
    };
    for (auto& key : path.keys) {
        key.id = path.name + "_" + key.id;
    }
    return path;
}

}  // namespace

TEST_CASE(
    "Pre-roll alignment ramp spreads the terminal seam move across authored keys",
    "[camera][animation][pan-extension]") {
    const auto source = MakeDenseReciprocalPanTestPath("ramp-src", 1.0F);
    const auto destination = MakeDenseReciprocalPanTestPath(
        "ramp-dst",
        -1.0F);
    invisible_places::camera::AnimationTerminalExtensionSpec spec;
    spec.sourceTailFrame = 24U;
    spec.sourcePatch = MakeReciprocalPanTestPatch();
    auto destinationPatch = MakeReciprocalPanTestPatch();
    destinationPatch.worldPoints[0U][0U] = 2.0F;
    destinationPatch.worldPoints[1U][0U] = 3.0F;
    destinationPatch.worldPoints[2U][0U] = 2.0F;
    spec.destinationEndPatch = destinationPatch;

    const auto legacy = invisible_places::camera::
        BuildAnimationPanTerminalExtensionPreview(
            destination,
            source,
            spec,
            16.0F / 9.0F,
            9U,
            12U);
    INFO(legacy.errorMessage);
    REQUIRE(legacy.succeeded);
    CHECK(legacy.alignmentSpreadKeyCount == 0U);
    CHECK(legacy.formerTerminalCameraMove > 1.0e-3F);

    auto rampedSpec = spec;
    rampedSpec.alignmentRampFrames = 60U;
    const auto ramped = invisible_places::camera::
        BuildAnimationPanTerminalExtensionPreview(
            destination,
            source,
            rampedSpec,
            16.0F / 9.0F,
            9U,
            12U);
    INFO(ramped.errorMessage);
    REQUIRE(ramped.succeeded);
    // Keys 24 and 48 frames before the terminal sit strictly inside the
    // 60-frame window.
    CHECK(ramped.alignmentSpreadKeyCount == 2U);

    // Both fits land the terminal on the identical aligned pose.
    const auto& legacyTerminal =
        legacy.candidate.keys[destination.keys.size() - 1U];
    const auto& rampedTerminal =
        ramped.candidate.keys[destination.keys.size() - 1U];
    for (std::size_t component = 0U; component < 3U; ++component) {
        CHECK(rampedTerminal.cameraPosition[component] ==
              Approx(legacyTerminal.cameraPosition[component])
                  .margin(1.0e-4F));
    }

    // The ramp trades the one-segment lurch for a gentler approach: the peak
    // extra camera speed over the incoming window must drop.
    CHECK(ramped.incomingTransientPeakSpeed <
          legacy.incomingTransientPeakSpeed);

    // Before the window (its first eased key is 48 frames before the end,
    // so exactness holds through the preceding key at 48/30 s) the ramped
    // candidate still evaluates exactly like the original animation.
    const auto originalContext = invisible_places::camera::
        PrepareAnimationPathEvaluation(destination);
    const auto rampedContext = invisible_places::camera::
        PrepareAnimationPathEvaluation(ramped.candidate);
    REQUIRE(originalContext.valid);
    REQUIRE(rampedContext.valid);
    const float exactPrefixEnd = 48.0F / 30.0F;
    for (std::uint32_t sample = 0U; sample <= 48U; ++sample) {
        const float time = exactPrefixEnd *
            static_cast<float>(sample) / 48.0F;
        CheckPanEvaluationNear(
            invisible_places::camera::EvaluatePreparedAnimationPath(
                originalContext,
                time),
            invisible_places::camera::EvaluatePreparedAnimationPath(
                rampedContext,
                time),
            2.0e-4F);
    }

    // The eased keys hold fractional shares of the terminal move, pinned as
    // localized corrections over untouched base spline poses.
    for (const auto keyOffset : {std::size_t{3U}, std::size_t{4U}}) {
        const auto& key = ramped.candidate.keys[keyOffset];
        const auto correction = std::find_if(
            ramped.candidate.localizedKeyCorrections.begin(),
            ramped.candidate.localizedKeyCorrections.end(),
            [&](const auto& value) { return value.keyId == key.id; });
        REQUIRE(correction !=
                ramped.candidate.localizedKeyCorrections.end());
        CHECK(correction->hasCameraCorrectionTangent);
        for (std::size_t component = 0U; component < 3U; ++component) {
            CHECK(correction->splineCameraPosition[component] ==
                  Approx(destination.keys[keyOffset]
                             .cameraPosition[component])
                      .margin(1.0e-6F));
        }
        const float movedDistance = std::hypot(
            key.cameraPosition[0U] -
                destination.keys[keyOffset].cameraPosition[0U],
            key.cameraPosition[1U] -
                destination.keys[keyOffset].cameraPosition[1U],
            key.cameraPosition[2U] -
                destination.keys[keyOffset].cameraPosition[2U]);
        CHECK(movedDistance > 0.0F);
        CHECK(movedDistance < legacy.formerTerminalCameraMove + 1.0e-4F);
    }

    // The appended tail still tracks the source patch as well as before.
    CHECK(ramped.anchorOverlayRmsScreenHeights <
          legacy.anchorOverlayRmsScreenHeights + 5.0e-3F);
}

TEST_CASE(
    "Balanced seam alignment moves the source start toward the destination terminal",
    "[camera][animation][pan-extension]") {
    const auto source = MakeDenseReciprocalPanTestPath("bias-src", 1.0F);
    const auto destination = MakeDenseReciprocalPanTestPath(
        "bias-dst",
        -1.0F);
    invisible_places::camera::AnimationTerminalExtensionSpec spec;
    spec.sourceTailFrame = 24U;
    spec.sourcePatch = MakeReciprocalPanTestPatch();
    auto destinationPatch = MakeReciprocalPanTestPatch();
    destinationPatch.worldPoints[0U][0U] = 2.0F;
    destinationPatch.worldPoints[1U][0U] = 3.0F;
    destinationPatch.worldPoints[2U][0U] = 2.0F;
    spec.destinationEndPatch = destinationPatch;
    spec.alignmentRampFrames = 48U;

    const auto unbiased = invisible_places::camera::
        BuildAnimationPanBidirectionalSeamPreview(
            source,
            destination,
            spec,
            16.0F / 9.0F,
            9U,
            12U);
    INFO(unbiased.errorMessage);
    REQUIRE(unbiased.succeeded);

    auto balancedSpec = spec;
    balancedSpec.sourceAlignmentFraction = 0.5F;
    const auto balanced = invisible_places::camera::
        BuildAnimationPanBidirectionalSeamPreview(
            source,
            destination,
            balancedSpec,
            16.0F / 9.0F,
            9U,
            12U);
    INFO(balanced.errorMessage);
    REQUIRE(balanced.succeeded);

    // The destination terminal carries a smaller share of the alignment.
    CHECK(balanced.destinationTail.formerTerminalCameraMove <
          unbiased.destinationTail.formerTerminalCameraMove);

    // The source's former frame-zero key moved as a localized correction
    // while its base spline pose stayed authored.
    const std::size_t headKeys = balanced.sourceHead.appendedKeyCount;
    const auto& movedStart =
        balanced.sourceHead.candidate.keys[headKeys];
    CHECK(movedStart.id == source.keys.front().id);
    const float startMove = std::hypot(
        movedStart.cameraPosition[0U] -
            source.keys.front().cameraPosition[0U],
        movedStart.cameraPosition[1U] -
            source.keys.front().cameraPosition[1U],
        movedStart.cameraPosition[2U] -
            source.keys.front().cameraPosition[2U]);
    CHECK(startMove > 1.0e-4F);
    const auto startCorrection = std::find_if(
        balanced.sourceHead.candidate.localizedKeyCorrections.begin(),
        balanced.sourceHead.candidate.localizedKeyCorrections.end(),
        [&](const auto& value) {
            return value.keyId == source.keys.front().id;
        });
    REQUIRE(startCorrection !=
            balanced.sourceHead.candidate.localizedKeyCorrections.end());

    // A zero fraction reproduces the unbiased candidates exactly.
    auto zeroSpec = spec;
    zeroSpec.sourceAlignmentFraction = 0.0F;
    const auto zero = invisible_places::camera::
        BuildAnimationPanBidirectionalSeamPreview(
            source,
            destination,
            zeroSpec,
            16.0F / 9.0F,
            9U,
            12U);
    REQUIRE(zero.succeeded);
    const auto zeroContext = invisible_places::camera::
        PrepareAnimationPathEvaluation(zero.destinationTail.candidate);
    const auto unbiasedContext = invisible_places::camera::
        PrepareAnimationPathEvaluation(unbiased.destinationTail.candidate);
    REQUIRE(zeroContext.valid);
    REQUIRE(unbiasedContext.valid);
    for (std::uint32_t sample = 0U; sample <= 24U; ++sample) {
        const float time = zeroContext.durationSeconds *
            static_cast<float>(sample) / 24.0F;
        CheckPanEvaluationNear(
            invisible_places::camera::EvaluatePreparedAnimationPath(
                zeroContext,
                time),
            invisible_places::camera::EvaluatePreparedAnimationPath(
                unbiasedContext,
                time),
            1.0e-6F);
    }
}

TEST_CASE(
    "Bidirectional reciprocal extension composes both seams' pre-roll ramps",
    "[camera][animation][pan-extension]") {
    const auto first = MakeDenseReciprocalPanTestPath("compose-A", 1.0F);
    const auto second = MakeDenseReciprocalPanTestPath("compose-B", -1.0F);
    invisible_places::camera::AnimationReciprocalPanExtensionOptions options;
    const auto patch = MakeReciprocalPanTestPatch();
    options.firstDrivesSecond.sourceTailFrame = 24U;
    options.firstDrivesSecond.sourcePatch = patch;
    auto bPatch = patch;
    bPatch.worldPoints[0U][0U] = 2.0F;
    bPatch.worldPoints[1U][0U] = 3.0F;
    bPatch.worldPoints[2U][0U] = 2.0F;
    options.firstDrivesSecond.destinationEndPatch = bPatch;
    options.firstDrivesSecond.alignmentRampFrames = 48U;
    options.secondDrivesFirst.sourceTailFrame = 24U;
    options.secondDrivesFirst.sourcePatch = patch;
    auto aPatch = patch;
    aPatch.worldPoints[0U][0U] = 1.5F;
    aPatch.worldPoints[1U][0U] = 2.5F;
    aPatch.worldPoints[2U][0U] = 1.5F;
    options.secondDrivesFirst.destinationEndPatch = aPatch;
    options.secondDrivesFirst.alignmentRampFrames = 48U;
    options.sampleCount = 9U;
    options.optimizationSweeps = 12U;

    const auto result = invisible_places::camera::
        BuildAnimationBidirectionalReciprocalPanExtension(
            first,
            second,
            options);
    INFO(result.errorMessage);
    REQUIRE(result.succeeded);
    CHECK(result.metrics.outgoing.alignmentSpreadKeyCount[0U] > 0U);
    CHECK(result.metrics.outgoing.alignmentSpreadKeyCount[1U] > 0U);

    // Both merged candidates still evaluate and keep every original key id
    // exactly once between the generated head and tail.
    for (const auto* candidate :
         {&result.firstCandidate, &result.secondCandidate}) {
        const auto context = invisible_places::camera::
            PrepareAnimationPathEvaluation(*candidate);
        REQUIRE(context.valid);
        std::unordered_set<std::string> ids;
        for (const auto& key : candidate->keys) {
            CHECK(ids.insert(key.id).second);
        }
    }
    for (const auto& key : first.keys) {
        CHECK(std::count_if(
                  result.firstCandidate.keys.begin(),
                  result.firstCandidate.keys.end(),
                  [&](const auto& candidateKey) {
                      return candidateKey.id == key.id;
                  }) == 1);
    }

    // The bulk centre, outside both 48-frame windows, is preserved: the
    // authored middle key pose survives the merge untouched.
    const auto& middle = first.keys[2U];
    const auto merged = std::find_if(
        result.firstCandidate.keys.begin(),
        result.firstCandidate.keys.end(),
        [&](const auto& key) { return key.id == middle.id; });
    REQUIRE(merged != result.firstCandidate.keys.end());
    for (std::size_t component = 0U; component < 3U; ++component) {
        CHECK(merged->cameraPosition[component] ==
              Approx(middle.cameraPosition[component]).margin(1.0e-5F));
    }
}

TEST_CASE(
    "Bidirectional merge preserves a pre-existing correction's effective tangent",
    "[camera][animation][pan-extension]") {
    auto first = MakeDenseReciprocalPanTestPath("carry-A", 1.0F);
    const auto second = MakeDenseReciprocalPanTestPath("carry-B", -1.0F);
    // Pre-existing localized corrections without materialized tangents
    // evaluate with derived central-difference tangents. Two adjacent
    // corrections give the middle one a NONZERO derived tangent; both end
    // fits materialize that same value, so the merge must not double it.
    for (const auto [keyIndex, offset] :
         {std::pair<std::size_t, float>{2U, 0.02F},
          std::pair<std::size_t, float>{3U, 0.045F}}) {
        auto& key = first.keys[keyIndex];
        invisible_places::camera::AnimationLocalizedKeyCorrection correction;
        correction.keyId = key.id;
        correction.splineCameraPosition = key.cameraPosition;
        correction.splineFocusPoint = key.focusPoint;
        key.cameraPosition[2U] += offset;
        first.localizedKeyCorrections.push_back(correction);
    }
    const auto& correctedKey = first.keys[2U];

    const auto originalContext = invisible_places::camera::
        PrepareAnimationPathEvaluation(first);
    REQUIRE(originalContext.valid);
    REQUIRE(originalContext.hasLoopKeyCorrections);
    const auto expectedTangent =
        originalContext.loopCameraCorrectionTangents[2U];
    REQUIRE(std::abs(expectedTangent[2U]) > 1.0e-4F);

    invisible_places::camera::AnimationReciprocalPanExtensionOptions options;
    const auto patch = MakeReciprocalPanTestPatch();
    options.firstDrivesSecond.sourceTailFrame = 24U;
    options.firstDrivesSecond.sourcePatch = patch;
    auto bPatch = patch;
    bPatch.worldPoints[0U][0U] = 0.4F;
    bPatch.worldPoints[1U][0U] = 0.65F;
    bPatch.worldPoints[2U][0U] = 0.4F;
    options.firstDrivesSecond.destinationEndPatch = bPatch;
    options.firstDrivesSecond.alignmentRampFrames = 30U;
    options.secondDrivesFirst.sourceTailFrame = 24U;
    options.secondDrivesFirst.sourcePatch = patch;
    auto aPatch = patch;
    aPatch.worldPoints[0U][0U] = 0.3F;
    aPatch.worldPoints[1U][0U] = 0.55F;
    aPatch.worldPoints[2U][0U] = 0.3F;
    options.secondDrivesFirst.destinationEndPatch = aPatch;
    options.secondDrivesFirst.alignmentRampFrames = 30U;
    options.sampleCount = 9U;
    options.optimizationSweeps = 12U;

    const auto result = invisible_places::camera::
        BuildAnimationBidirectionalReciprocalPanExtension(
            first,
            second,
            options);
    INFO(result.errorMessage);
    REQUIRE(result.succeeded);
    const auto merged = std::find_if(
        result.firstCandidate.localizedKeyCorrections.begin(),
        result.firstCandidate.localizedKeyCorrections.end(),
        [&](const auto& value) { return value.keyId == correctedKey.id; });
    REQUIRE(merged != result.firstCandidate.localizedKeyCorrections.end());
    REQUIRE(merged->hasCameraCorrectionTangent);
    // The 30-frame ramps do not reach the middle key (48 frames from either
    // end), so its effective tangent must survive the merge unchanged.
    for (std::size_t component = 0U; component < 3U; ++component) {
        CHECK(merged->cameraCorrectionTangent[component] ==
              Approx(expectedTangent[component]).margin(1.0e-5F));
    }
    const auto mergedKey = std::find_if(
        result.firstCandidate.keys.begin(),
        result.firstCandidate.keys.end(),
        [&](const auto& key) { return key.id == correctedKey.id; });
    REQUIRE(mergedKey != result.firstCandidate.keys.end());
    CHECK(mergedKey->cameraPosition[2U] ==
          Approx(correctedKey.cameraPosition[2U]).margin(1.0e-5F));
}
