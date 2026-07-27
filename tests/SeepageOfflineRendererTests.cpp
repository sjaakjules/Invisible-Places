#include "camera/CameraState.hpp"
#include "io/PointCloudData.hpp"
#include "output/OfflinePointRenderer.hpp"
#include "renderer/pointcloud/PointCloudPreviewState.hpp"
#include "style/RenderParameterBinding.hpp"
#include "water/WaterFlow.hpp"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

namespace {

void WriteLookAtOrientation(invisible_places::camera::CameraState* state) {
    const glm::vec3 position{state->position[0], state->position[1], state->position[2]};
    const glm::vec3 target{state->target[0], state->target[1], state->target[2]};
    const auto view = glm::lookAtRH(position, target, glm::vec3{0.0F, 0.0F, 1.0F});
    const auto orientation = glm::normalize(glm::quat_cast(glm::inverse(glm::mat3{view})));
    state->orientation = {orientation.x, orientation.y, orientation.z, orientation.w};
}

invisible_places::io::LoadedPointCloud MakeSeepageCloud() {
    invisible_places::io::LoadedPointCloud cloud;
    cloud.positions = {{0.0F, 0.0F, -0.55F}};
    cloud.normals = {{0.0F, 1.0F, 0.0F}};
    cloud.packedColors = {0xffffffffU};
    cloud.hasNormals = true;
    cloud.hasSourceRgb = true;
    cloud.bounds.Expand(cloud.positions.front());
    return cloud;
}

invisible_places::renderer::pointcloud::PointCloudStyleState MakeSeepageStyle() {
    invisible_places::renderer::pointcloud::PointCloudStyleState style;
    style.colorMode = invisible_places::renderer::pointcloud::PointCloudColorMode::SolidColor;
    style.solidColor = {0.72F, 0.34F, 0.20F, 1.0F};
    style.falloffProfile = invisible_places::renderer::pointcloud::PointCloudFalloffProfile::HardDisc;
    invisible_places::style::SetScalarConstant(&style.pointSize, 7.0F);
    invisible_places::style::SetScalarConstant(&style.surfelDiameter, 0.08F);
    invisible_places::style::SetScalarConstant(&style.opacity, 0.45F);
    invisible_places::style::SetScalarConstant(&style.emissiveStrength, 0.0F);
    return style;
}

invisible_places::water::WaterSeepageSpatialGrid MakeSeepageGrid(bool attenuateOpacity = false) {
    invisible_places::water::WaterSeepageNode node;
    node.id = 2U;
    node.position = {0.0F, 0.0F, 0.0F};
    node.surfaceNormal = {0.0F, 1.0F, 0.0F};
    node.downAxis = {0.0F, 0.0F, -1.0F};
    node.seed = 91U;
    auto look = invisible_places::water::DefaultWaterSeepageLookSettings();
    look.density = 1.0F;
    look.glisten = 1.0F;
    look.response.colouriseAmount = 0.85F;
    look.response.emissionAdd = 0.50F;
    if (attenuateOpacity) {
        look.response.opacityAdd = 0.0F;
        look.response.opacityMultiply = 0.45F;
    }
    // Per-node overrides were replaced by named profiles; the custom look is
    // supplied as the default look the node resolves to.
    const std::vector<invisible_places::water::WaterSeepageNode> nodes{node};
    return invisible_places::water::BuildWaterSeepageSpatialGrid(
        nodes,
        {},
        look,
        "ROCK",
        true,
        invisible_places::water::DefaultWaterRainSettings(),
        100'000'000ULL);
}

invisible_places::output::ExrImage Render(
    const invisible_places::output::OfflinePointLayer& layer,
    float timeSeconds) {
    invisible_places::camera::CameraState camera;
    camera.position = {0.0F, -5.0F, -0.55F};
    camera.target = {0.0F, 0.0F, -0.55F};
    camera.nearPlane = 0.1F;
    camera.farPlane = 20.0F;
    WriteLookAtOrientation(&camera);

    invisible_places::output::ExrImage image;
    invisible_places::output::InitializeExrImage(&image, 15U, 15U);
    invisible_places::output::RenderPointCloudTile(
        {layer},
        camera,
        {0U, 0U, 15U, 15U},
        &image,
        nullptr,
        nullptr,
        timeSeconds);
    return image;
}

float ImageDifference(
    const invisible_places::output::ExrImage& left,
    const invisible_places::output::ExrImage& right) {
    float result = 0.0F;
    for (std::size_t index = 0; index < left.alpha.size(); ++index) {
        result += std::abs(left.beautyR[index] - right.beautyR[index]);
        result += std::abs(left.beautyG[index] - right.beautyG[index]);
        result += std::abs(left.beautyB[index] - right.beautyB[index]);
        result += std::abs(left.alpha[index] - right.alpha[index]);
    }
    return result;
}

}  // namespace

TEST_CASE("Offline export evaluates animated Seepage without point memberships", "[water][seepage][offline][export]") {
    const auto cloud = MakeSeepageCloud();
    const auto style = MakeSeepageStyle();
    const invisible_places::output::OfflinePointLayer dryLayer{
        .cloud = &cloud,
        .style = style,
        .hasSourceRgb = true,
        .localToWorld = glm::mat4{1.0F},
    };
    const invisible_places::output::OfflinePointLayer seepageLayer{
        .cloud = &cloud,
        .style = style,
        .hasSourceRgb = true,
        .localToWorld = glm::mat4{1.0F},
        .seepageGrid = MakeSeepageGrid(),
    };

    CHECK(seepageLayer.seepageGrid.nodes.size() == 1U);
    CHECK(seepageLayer.seepageGrid.nodeReferences.size() > 0U);
    const auto dry = Render(dryLayer, 0.0F);
    const auto wet = Render(seepageLayer, 0.0F);
    const auto animated = Render(seepageLayer, 0.37F);
    const std::size_t center = 7U * 15U + 7U;
    REQUIRE(dry.alpha[center] > 0.0F);
    REQUIRE(wet.alpha[center] > 0.0F);
    CHECK(wet.alpha[center] > dry.alpha[center]);
    CHECK(ImageDifference(dry, wet) > 0.01F);
    CHECK(ImageDifference(wet, animated) > 1.0e-5F);
}

TEST_CASE("Fast Basic offline export applies the full Seepage response", "[water][seepage][offline][fast-basic]") {
    const auto cloud = MakeSeepageCloud();
    const auto style = MakeSeepageStyle();
    const invisible_places::output::OfflinePointLayer dryLayer{
        .cloud = &cloud,
        .style = style,
        .hasSourceRgb = true,
        .fastBasic = true,
        .drawPointCount = 1U,
        .localToWorld = glm::mat4{1.0F},
    };
    auto wetLayer = dryLayer;
    wetLayer.seepageGrid = MakeSeepageGrid(true);

    const auto dry = Render(dryLayer, 0.13F);
    const auto wet = Render(wetLayer, 0.13F);
    const std::size_t center = 7U * 15U + 7U;
    REQUIRE(dry.alpha[center] > 0.0F);
    REQUIRE(wet.alpha[center] > 0.0F);
    CHECK(wet.alpha[center] < dry.alpha[center]);
    CHECK(ImageDifference(dry, wet) > 0.01F);
}
