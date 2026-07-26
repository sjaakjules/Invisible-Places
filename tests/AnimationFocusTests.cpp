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
