#include "camera/AnimationPath.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

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
