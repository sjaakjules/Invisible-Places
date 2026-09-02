#include "renderer/core/VulkanViewportShell.hpp"

#include "InvisiblePlacesBuildConfig.hpp"
#include "io/SceneDisplayDensityCache.hpp"
#include "water/WaterSeepagePulseField.hpp"

#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>

#include <Imath/half.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_vulkan.h>

#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstddef>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <limits>
#include <numeric>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

#include <glm/mat4x4.hpp>
#include <glm/matrix.hpp>
#include <glm/vec4.hpp>

namespace invisible_places::renderer::core {

namespace {

#ifndef VK_KHR_PORTABILITY_SUBSET_EXTENSION_NAME
#define VK_KHR_PORTABILITY_SUBSET_EXTENSION_NAME "VK_KHR_portability_subset"
#endif

constexpr float kPointCloudAntialiasFeatherPixels = 1.0F;

[[nodiscard]] bool PointCloudLayerUsesFastBasicRenderer(
    renderer::pointcloud::PointCloudRendererMode requestedMode,
    const SceneRenderState::PointCloudLayerState& layer) {
    return renderer::pointcloud::ResolvePointCloudLayerRendererMode(
               requestedMode,
               layer.generatedWaterOverlay,
               layer.style) ==
           renderer::pointcloud::PointCloudRendererMode::FastBasic;
}

struct QueueFamilySelection {
    std::optional<std::uint32_t> graphicsFamily;
    std::optional<std::uint32_t> presentFamily;

    [[nodiscard]] bool IsComplete() const {
        return graphicsFamily.has_value() && presentFamily.has_value();
    }
};

struct SwapchainSupport {
    VkSurfaceCapabilitiesKHR capabilities{};
    std::vector<VkSurfaceFormatKHR> formats;
    std::vector<VkPresentModeKHR> presentModes;
};

struct alignas(16) FrameUniforms {
    glm::mat4 viewProjection{1.0F};
    glm::mat4 view{1.0F};
    glm::mat4 projection{1.0F};
    glm::vec4 cameraPosition{0.0F, 0.0F, 1.0F, 0.0F};
    glm::vec4 depthParameters{0.0F, 0.05F, 1000.0F, 0.0F};
    glm::vec4 viewportParameters{1.0F, 1.0F, 2.0F, 2.0F};
    glm::vec4 depthOfFieldParameters{0.0F, 1.0F, 8.0F, 24.0F};
    // xyz: aHQ start/switch/end view depth; w: prepared fine depth.
    glm::vec4 adaptiveDensityParameters{0.0F};
    glm::mat4 inverseViewProjection{1.0F};
};

double MillisecondsBetween(
    std::chrono::steady_clock::time_point start,
    std::chrono::steady_clock::time_point end) {
    return std::chrono::duration<double, std::milli>(end - start).count();
}

const char* PresentModeName(VkPresentModeKHR presentMode) {
    switch (presentMode) {
        case VK_PRESENT_MODE_IMMEDIATE_KHR:
            return "Immediate";
        case VK_PRESENT_MODE_MAILBOX_KHR:
            return "Mailbox";
        case VK_PRESENT_MODE_FIFO_KHR:
            return "FIFO";
        case VK_PRESENT_MODE_FIFO_RELAXED_KHR:
            return "FIFO relaxed";
        default:
            return "Unknown";
    }
}

struct alignas(16) PointCloudBindingGpu {
    glm::vec4 constantValue{0.0F, 0.0F, 0.0F, 0.0F};
    glm::vec4 range{0.0F, 1.0F, 0.0F, 1.0F};
    glm::vec4 extra{1.0F, 0.0F, 0.0F, 0.0F};
    glm::uvec4 control{0U, 0U, 0U, 0U};
};

struct alignas(16) PointCloudStyleGpu {
    glm::vec4 solidColor{0.93F, 0.88F, 0.72F, 1.0F};
    glm::uvec4 globalControl{0U, 0U, 0U, 1U};
    glm::uvec4 pointMeta{0U, 0U, 0U, 0U};
    glm::uvec4 renderControl{0U, 1U, 0U, 0U};
    glm::vec4 renderParams0{1.0F, 0.55F, 4.0F, 1.6F};
    // x: trail-style geometry, y: density footprint scale, z: density coverage
    // correction, w: runtime Water Flow activity.
    glm::vec4 renderParams1{0.0F, 1.0F, 1.0F, 1.0F};
    glm::vec4 renderParams2{1.0F, 0.0F, 1.0F, 0.0F};
    glm::vec4 renderParams3{0.0F, 1.0F, 64.0F, 0.0F};
    PointCloudBindingGpu pointSize{};
    PointCloudBindingGpu opacity{};
    PointCloudBindingGpu emissive{};
    PointCloudBindingGpu depthFade{};
    PointCloudBindingGpu colormapPosition{};
    PointCloudBindingGpu surfelDiameter{};
    glm::vec4 colorize{0.95F, 0.68F, 0.28F, 0.0F};
    glm::uvec4 stylisationControl{0U, 0U, 0U, 0U};
    glm::vec4 stylisationParams0{1.0F, 5.0F, 0.35F, 0.35F};
    glm::vec4 stylisationParams1{0.45F, 2.2F, 0.35F, 0.0F};
    glm::vec4 stylisationParams2{0.25F, 0.0F, 0.0F, 0.0F};
    glm::vec4 surfaceMotionParams{0.0F, 1.5F, 0.35F, 0.58F};
    glm::vec4 surfaceMotionStats{0.0F, 1.0F, 1.0F, 0.25F};
    glm::uvec4 shorelineWaveControl{0U, 1U, 0U, 0U};
    glm::vec4 shorelineWaveParams0{1.55F, 0.45F, 0.05F, 1.15F};
    glm::vec4 shorelineWaveParams1{1.0F, 0.0F, 1.0F, 0.25F};
    glm::vec4 shorelineWaveParams2{0.55F, 0.35F, 0.06F, 0.55F};
    glm::vec4 shorelineWaveParams3{0.0F, 0.65F, 0.08F, 1.25F};
    glm::vec4 shorelineWaveParams4{0.0F, 1.35F, 0.75F, 0.0F};
    glm::vec4 shorelineWaveParams5{1.30F, 0.30F, 1.0F, 0.30F};
    glm::vec4 shorelineWaveTint{0.62F, 0.88F, 1.0F, 1.0F};
    glm::vec4 gradientStartColor{0.05F, 0.28F, 0.95F, 1.0F};
    glm::vec4 gradientEndColor{0.96F, 0.94F, 0.58F, 1.0F};
    // x: enabled, y: node count, z: hash-cell capacity, w: node-reference count.
    glm::uvec4 seepageControl{0U, 0U, 0U, 0U};
    // x: cell size, y: inverse cell size, z: maximum linear probes.
    glm::vec4 seepageGridParams{0.50F, 2.0F, 1.0F, 0.0F};
    glm::vec4 seepageBoundsMin{0.0F, 0.0F, 0.0F, 0.0F};
    glm::vec4 seepageBoundsMax{0.0F, 0.0F, 0.0F, 0.0F};
    // x: enabled, y: scene role, z: grid dimension, w: event capacity.
    glm::uvec4 rainImpactControl{0U, 0U, 0U, 0U};
    // xy: grid origin, z: cell size, w: animation time.
    glm::vec4 rainImpactGrid{0.0F, 0.0F, 0.125F, 0.0F};
    glm::vec4 rainImpactRock0{0.35F, 1.60F, 0.65F, 0.75F};
    // x: persistence; yzw: ROCK height band [minZ, maxZ, fadeMeters].
    glm::vec4 rainImpactRock1{1.35F, 1.5F, 2.4F, 0.30F};
    glm::vec4 rainImpactVegetation0{1.80F, 0.65F, 0.070F, 0.010F};
    // x: stream spread; yzw: VEG height band [minZ, maxZ, fadeMeters].
    glm::vec4 rainImpactVegetation1{0.65F, 2.5F, 1.0e8F, 0.30F};
    // xyz: SAND height band [minZ, maxZ, fadeMeters], w: Wetness downhill
    // stretch.
    glm::vec4 rainImpactSandBand{-1.0e8F, 2.0F, 0.30F, 1.0F};
    // xyz: Rings/Wetness/Droplets response, w: expanding-ring thickness.
    glm::vec4 rainImpactResponse{1.0F};
    // x: active effect count. Effects are applied from index zero upward, so
    // the last entry is visually topmost.
    glm::uvec4 timingColouriseControl{0U, 0U, 0U, 0U};
    // x: enabled, y: TimingColouriseSource, z: scalar slot + 1 (zero means
    // unavailable), w: TimingColouriseOutput. Normal sources do not consume
    // z.
    std::array<glm::uvec4, renderer::pointcloud::kTimingColouriseMaxEffects>
        timingColouriseSources{};
    // x/y: lower/upper bounds; z: unused since the per-edge fade split
    // (schema 85) moved fades into timingColouriseFades; w: signed
    // emissive/darkening level.
    std::array<glm::vec4, renderer::pointcloud::kTimingColouriseMaxEffects>
        timingColouriseRanges{};
    // x/y: signed lower/upper edge-fade fractions. Positive inward values
    // stop at one span; negative outward values may cover many spans. z/w:
    // spare.
    std::array<glm::vec4, renderer::pointcloud::kTimingColouriseMaxEffects>
        timingColouriseFades{};
    // Effect-major 64-sample RGBA lookup tables. RGB is tint; A is mix only.
    std::array<
        glm::vec4,
        renderer::pointcloud::kTimingColouriseMaxEffects *
            renderer::pointcloud::kTimingColouriseLutSamples>
        timingColouriseLut{};
    // Additional shoreline instances (x = packed count) with the same lane
    // layout as the primary shorelineWave* fields above. Appended after the
    // Timing Colourise block so every existing std140 offset is preserved.
    glm::uvec4 additionalShorelineCount{0U, 0U, 0U, 0U};
    std::array<glm::uvec4, renderer::pointcloud::kMaxAdditionalShorelineInstances>
        additionalShorelineControl{};
    std::array<glm::vec4, renderer::pointcloud::kMaxAdditionalShorelineInstances>
        additionalShorelineParams0{};
    std::array<glm::vec4, renderer::pointcloud::kMaxAdditionalShorelineInstances>
        additionalShorelineParams1{};
    std::array<glm::vec4, renderer::pointcloud::kMaxAdditionalShorelineInstances>
        additionalShorelineParams2{};
    std::array<glm::vec4, renderer::pointcloud::kMaxAdditionalShorelineInstances>
        additionalShorelineParams3{};
    std::array<glm::vec4, renderer::pointcloud::kMaxAdditionalShorelineInstances>
        additionalShorelineParams4{};
    std::array<glm::vec4, renderer::pointcloud::kMaxAdditionalShorelineInstances>
        additionalShorelineParams5{};
    std::array<glm::vec4, renderer::pointcloud::kMaxAdditionalShorelineInstances>
        additionalShorelineTint{};
    // Appended so every established std140 offset remains stable. x: core
    // alpha threshold, y: view-space tolerance metres, z: depth-weight
    // strength, w: GPU back-to-front sorting enabled.
    glm::vec4 depthCompositingParams{0.35F, 0.02F, 1.0F, 0.0F};
    // x: emission response (0 accumulated, 1 saturated per-fragment fold),
    // y: back-face fade enabled, z/w: cosine of the fade start/end angles
    // between the point normal and the direction to the camera.
    glm::vec4 emissionNormalControl{0.0F, 0.0F, 1.0F, -1.0F};
    // x: PointCloudSurfaceStabilityMode, y: packed RGBA8 scalar slot (or
    // UINT_MAX when the validated sidecar is unavailable). z/w: spare.
    glm::uvec4 surfaceStabilityControl{
        0U,
        std::numeric_limits<std::uint32_t>::max(),
        0U,
        0U};
    // x: influence between the historical point opacity and the selected
    // sidecar channel. yzw: spare.
    glm::vec4 surfaceStabilityParams{1.0F, 0.0F, 0.0F, 0.0F};
};

// PointStyleData is mirrored verbatim by the point-cloud GLSL uniform blocks.
// Keep these checks beside the CPU definition so a capacity or field-layout
// change cannot silently shift the fixed std140 ABI.
static_assert(offsetof(PointCloudStyleGpu, timingColouriseControl) == 976U);
static_assert(offsetof(PointCloudStyleGpu, timingColouriseSources) == 992U);
static_assert(offsetof(PointCloudStyleGpu, timingColouriseRanges) == 1120U);
static_assert(offsetof(PointCloudStyleGpu, timingColouriseFades) == 1248U);
static_assert(offsetof(PointCloudStyleGpu, timingColouriseLut) == 1376U);
static_assert(
    offsetof(PointCloudStyleGpu, additionalShorelineCount) == 9568U);
static_assert(offsetof(PointCloudStyleGpu, depthCompositingParams) == 10096U);
static_assert(offsetof(PointCloudStyleGpu, emissionNormalControl) == 10112U);
static_assert(offsetof(PointCloudStyleGpu, surfaceStabilityControl) == 10128U);
static_assert(offsetof(PointCloudStyleGpu, surfaceStabilityParams) == 10144U);
static_assert(sizeof(PointCloudStyleGpu) == 10160U);

struct alignas(16) PointDepthSortPushConstants {
    glm::mat4 view{1.0F};
    glm::uvec4 control{0U, 0U, 0U, 0U};
    glm::vec4 depth{0.05F, 1000.0F, 0.0F, 0.0F};
};

// 2^17 logarithmic depth buckets keep same-key ties to near-coplanar points
// (sub-millimetre at survey working distances), and the low 3 bits carry a
// per-point hash so even those ties resolve to one fixed order instead of
// the scatter phase's scheduler order. The hash width must match
// kIndexHashBucketBits in pointcloud_depth_sort.comp.
constexpr std::uint32_t kPointDepthSortBucketCount = 1'048'576U;
constexpr std::uint32_t kPointDepthSortWorkgroupSize = 256U;
// Vulkan guarantees at least 65,535 compute groups in both X and Y. Use the
// portable X limit and spill larger source-cloud sorts into Y.
constexpr std::uint32_t kPointDepthSortDispatchGroupsX = 65'535U;
static_assert(sizeof(PointDepthSortPushConstants) == 96U);

// Snaps the sort identity below visible-order granularity: direction to
// ~0.2 degree steps and the along-axis offset to a sliver of the depth
// range. A slow, slightly wobbling pan then reuses the cached ordering for
// many consecutive frames — bit-stable output and no per-frame compute —
// while any deliberate view change still refreshes the sort promptly.
glm::mat4 QuantizePointDepthSortView(
    const glm::mat4& view,
    float nearPlane,
    float farPlane) {
    constexpr float kDirectionQuantum = 1.0F / 256.0F;
    const float offsetQuantum = std::max(
        1.0e-5F,
        (std::max(farPlane, nearPlane) - nearPlane) * 1.0e-4F);
    const auto snap = [](float value, float quantum) {
        return std::round(value / quantum) * quantum;
    };
    glm::mat4 quantized = view;
    quantized[0U][2U] = snap(view[0U][2U], kDirectionQuantum);
    quantized[1U][2U] = snap(view[1U][2U], kDirectionQuantum);
    quantized[2U][2U] = snap(view[2U][2U], kDirectionQuantum);
    quantized[3U][2U] = snap(view[3U][2U], offsetQuantum);
    return quantized;
}

void HashPointDepthSortWord(std::uint64_t* hash, std::uint32_t value) {
    if (hash == nullptr) {
        return;
    }
    // FNV-1a is sufficient here: this is a render-cache identity, not a
    // persisted content fingerprint.
    *hash ^= value;
    *hash *= 1099511628211ULL;
}

std::uint64_t PointDepthSortCacheKey(
    const glm::mat4& view,
    float nearPlane,
    float farPlane,
    std::uint32_t pointCount,
    std::uint32_t sourceMode,
    std::uint32_t dynamicPositionRevision) {
    std::uint64_t hash = 1469598103934665603ULL;
    // The compute shader consumes only the Z row of the view transform. By
    // hashing just that row, a fixed-direction lateral pan reuses exactly the
    // same ordering even in Per Frame mode.
    for (std::size_t column = 0U; column < 4U; ++column) {
        HashPointDepthSortWord(
            &hash,
            std::bit_cast<std::uint32_t>(view[column][2]));
    }
    HashPointDepthSortWord(
        &hash,
        std::bit_cast<std::uint32_t>(nearPlane));
    HashPointDepthSortWord(
        &hash,
        std::bit_cast<std::uint32_t>(farPlane));
    HashPointDepthSortWord(&hash, pointCount);
    HashPointDepthSortWord(&hash, sourceMode);
    HashPointDepthSortWord(&hash, dynamicPositionRevision);
    return hash;
}

// The surfel/EXR pair has the largest point-cloud stage interface: 20 scalar
// components plus one flat vec4 per Timing Colourise effect.
constexpr std::uint32_t kPointCloudMaximumInterstageComponents =
    20U +
    (4U * static_cast<std::uint32_t>(
              renderer::pointcloud::kTimingColouriseMaxEffects));

struct alignas(16) RainUniformsGpu {
    glm::mat4 viewProjection{1.0F};
    glm::mat4 view{1.0F};
    glm::mat4 projection{1.0F};
    glm::vec4 cameraTime{0.0F, 0.0F, 1.0F, 0.0F};
    glm::vec4 spawnCentreRadius{0.0F, 0.0F, 4.0F, 6.0F};
    glm::vec4 cacheBoundsMinResolution{0.0F, 0.0F, 0.0F, invisible_places::water::kWaterSurfaceResolutionMeters};
    glm::vec4 cacheBoundsMaxDeathDistance{0.0F, 0.0F, 0.0F, 28.0F};
    glm::vec4 weather0{1.0F, 0.0F, 0.3F, 0.45F};
    glm::vec4 weather1{0.35F, 8.0F, 2.5F, 0.4F};
    glm::vec4 weather2{12.0F, 1.5F, 1.0F / 30.0F, 8.0F};
    glm::vec4 visual0{0.003F, 0.16F, 0.42F, 0.58F};
    glm::vec4 visual1{0.68F, 0.82F, 0.92F, 0.16F};
    glm::vec4 visual2{0.65F, 4.0F, 1.0F, 1.0F};
    glm::uvec4 simulation0{0U, invisible_places::water::kRainParticleCapacity,
                           invisible_places::water::kRainImpactEventCapacity, 101U};
    glm::uvec4 simulation1{0U, 0U, 1U, 1U};
    glm::uvec4 collision0{0U, 0U, 1U, invisible_places::water::kRainImpactGridDimension};
    glm::vec4 impactGrid{0.0F, 0.0F, 0.125F, 32.0F};
    glm::uvec4 effectToggles{1U, 1U, 1U, 0U};
    glm::vec4 effectScales{1.0F, 1.0F, 1.0F, 0.0F};
    glm::vec4 nearSurface{0.18F, 0.30F, 0.65F, 0.75F};
    glm::vec4 viewport{1.0F, 1.0F, 1.0F, 1.0F};
};

struct alignas(16) RainParticleGpu {
    glm::vec4 positionAge{0.0F};
    glm::vec4 previousActivity{0.0F};
    glm::vec4 velocity{0.0F};
    glm::uvec4 state{0U};
};

struct alignas(16) RainImpactEventGpu {
    glm::vec4 positionBirth{0.0F};
    glm::vec4 normalRadius{0.0F, 0.0F, 1.0F, 0.04F};
    glm::vec4 lifetimeEnergy{0.0F};
    glm::uvec4 control{0U};
};

struct alignas(16) RainCountersGpu {
    glm::uvec4 values{0U};
    glm::uvec4 roleValues{0U};
};

struct alignas(16) WaterSurfacePreprocessUniformsGpu {
    glm::uvec4 table{0U, 1U, 0U, 0U};
    glm::vec4 parameters{
        invisible_places::water::kWaterSurfaceResolutionMeters,
        0.50F,
        0.45F,
        0.0F};
};

static_assert(sizeof(RainParticleGpu) == 64U);
static_assert(sizeof(RainImpactEventGpu) == 64U);
static_assert(sizeof(RainCountersGpu) == 32U);
static_assert(sizeof(invisible_places::water::RainGpuSurfaceSlot) == 32U);
static_assert(sizeof(invisible_places::water::RainGpuVegetationSlot) == 16U);
static_assert(sizeof(invisible_places::water::WaterGpuSurfaceSurfelSlot) == 32U);

std::uint32_t ActiveRainParticleCount(const SceneRenderState& state) {
    const auto intensity = invisible_places::water::RainIntensityValues(
        state.rainSettings.intensityPreset);
    return std::min<std::uint32_t>(
        invisible_places::water::kRainParticleCapacity,
        static_cast<std::uint32_t>(std::lround(
            state.rainSettings.activeParticleCount *
            std::clamp(state.rainSettings.rainLevel, 0.0F, 1.0F) *
            std::clamp(state.rainSettings.density * intensity.density, 0.0F, 1.0F))));
}

struct alignas(16) WaterSeepageLookGpu {
    // x: pattern, y: blend mode.
    glm::uvec4 control{0U, 1U, 0U, 0U};
    // Contour Pulses: x spacing, y width, z speed, w irregularity.
    glm::vec4 legacy0{0.18F, 0.055F, 0.12F, 0.38F};
    // Contour Pulses: x wave count, y speed variation. z/w retain the
    // shared density/response-intensity lanes.
    glm::vec4 legacy1{7.0F, 0.55F, 0.45F, 0.85F};
    glm::vec4 response0{0.35F, 0.04F, 1.12F, 0.0F};
    glm::vec4 response1{1.08F, 0.28F, 0.42F, 0.46F};
    glm::vec4 response2{0.22F, 0.35F, 0.55F, 0.50F};
    glm::vec4 organic0{0.20F, 0.55F, 0.06F, 0.45F};
    glm::vec4 organic1{0.70F, 0.18F, 0.30F, 0.40F};
    glm::vec4 organic2{0.45F, 0.025F, 0.35F, 0.018F};
    glm::vec4 organic3{0.10F, 0.0F, 0.0F, 0.0F};
    glm::vec4 environmentDirection{0.0F, 0.0F, 1.0F, 0.0F};
};

// Geometry, orientation and surface-guide samples only change when Seepage
// topology changes. Keeping them out of the live parameter record avoids
// copying guide arrays for every animation key or rain-level update.
struct alignas(16) WaterSeepageNodeTopologyGpu {
    static constexpr std::size_t kGuideSampleCapacity = 8U;

    // x: stable node id. Seed is live because authored seed edits are
    // parameter-only and must not rebuild guides or the spatial hash.
    glm::uvec4 control{0U, 0U, 0U, 0U};
    glm::vec4 positionReach{0.0F, 0.0F, 0.0F, 1.25F};
    glm::vec4 normalSurface{0.0F, 1.0F, 0.0F, 0.15F};
    glm::vec4 downEdge{0.0F, 0.0F, -1.0F, 0.10F};
    glm::vec4 lateralStart{1.0F, 0.0F, 0.0F, 0.06F};
    // x: selection half-width, y: maximum resident upstream run.
    glm::vec4 geometry{0.375F, 0.0F, 0.0F, 0.0F};
    glm::uvec4 guideControl{0U, 0U, 0U, 0U};
    std::array<glm::vec4, kGuideSampleCapacity> guidePositionStation{};
    std::array<glm::vec4, kGuideSampleCapacity> guideNormalConfidence{};
};

// This is the only Seepage record rewritten while an animation is playing.
// It intentionally retains the two complete looks required for deterministic
// point-level pattern cross-dissolves, but contains no guide/topology data.
struct alignas(16) WaterSeepageNodeParamsGpu {
    static constexpr std::size_t kPulseFieldVec4Capacity = 32U;

    // x: stable node id, y: quality, z: blend mode, w: seed.
    glm::uvec4 control{0U, 1U, 1U, 1U};
    // x: viewport/export enabled factor, y: normal-alignment weight,
    // z: effective strength,
    // w: rain visual strength.
    glm::vec4 geometry{0.0F, 0.20F, 1.0F, 0.0F};
    WaterSeepageLookGpu look{};
    WaterSeepageLookGpu transitionLook{};
    // x: scenario/local spread, y: transition amount, z: wetting progress.
    glm::vec4 scenario{0.0F, 0.0F, 1.0F, 0.0F};
    // x: live reach, y: live full width, z: live prominence,
    // w: least-resistance travel budget. These values never rebuild topology.
    glm::vec4 liveGeometry{1.25F, 0.75F, 1.0F, 1.5F};
    // Seed-derived basis is live so changing a node seed immediately changes
    // both noise orientation and procedural hashes.
    std::array<glm::vec4, 3> noiseBasis{};
    // x/y: current/transition sample counts, z/w: their stable spans as float
    // bits. The reference fields change only with authored pulse shape;
    // renderer time advances their shader lookups, so clock-only frames do
    // not rewrite this record or touch topology/descriptors.
    glm::uvec4 pulseFieldControl{0U, 0U, 0U, 0U};
    std::array<glm::vec4, kPulseFieldVec4Capacity> pulseField{};
    std::array<glm::vec4, kPulseFieldVec4Capacity> transitionPulseField{};
};

static_assert(sizeof(WaterSeepageLookGpu) == 176U);
static_assert(sizeof(WaterSeepageNodeTopologyGpu) == 368U);
static_assert(sizeof(WaterSeepageNodeParamsGpu) == 1504U);
static_assert(offsetof(WaterSeepageNodeParamsGpu, pulseFieldControl) == 464U);
static_assert(offsetof(WaterSeepageNodeParamsGpu, pulseField) == 480U);
static_assert(offsetof(WaterSeepageNodeParamsGpu, transitionPulseField) == 992U);

WaterSeepageLookGpu MakeWaterSeepageLookGpu(
    const invisible_places::water::WaterSeepageLookSettings& look) {
    const invisible_places::water::WaterSeepageLookSettings fallback{};
    const auto finiteOr = [](float value, float fallback) {
        return std::isfinite(value) ? value : fallback;
    };
    const auto finiteClamp = [&finiteOr](float value, float minimum, float maximum, float fallback) {
        return std::clamp(finiteOr(value, fallback), minimum, maximum);
    };
    WaterSeepageLookGpu gpu;
    gpu.control = glm::uvec4{
        static_cast<std::uint32_t>(look.pattern),
        static_cast<std::uint32_t>(look.blendMode),
        0U,
        0U,
    };
    gpu.legacy0 = glm::vec4{
        finiteClamp(
            look.pulseSpacingMeters,
            0.005F,
            20.0F,
            fallback.pulseSpacingMeters),
        finiteClamp(
            look.pulseWidthMeters,
            0.001F,
            10.0F,
            fallback.pulseWidthMeters),
        finiteClamp(
            look.pulseSpeedMetersPerSecond,
            0.0F,
            4.0F,
            fallback.pulseSpeedMetersPerSecond),
        finiteClamp(
            look.pulseIrregularity,
            0.0F,
            1.0F,
            fallback.pulseIrregularity),
    };
    gpu.legacy1 = glm::vec4{
        finiteClamp(
            look.pulseWaveCount,
            1.0F,
            12.0F,
            fallback.pulseWaveCount),
        finiteClamp(
            look.pulseSpeedVariation,
            0.0F,
            1.0F,
            fallback.pulseSpeedVariation),
        finiteClamp(look.density, 0.0F, 1.0F, fallback.density),
        finiteClamp(
            look.response.intensity,
            0.0F,
            8.0F,
            fallback.response.intensity),
    };
    gpu.response0 = glm::vec4{
        finiteClamp(
            look.response.emissionAdd,
            0.0F,
            16.0F,
            fallback.response.emissionAdd),
        finiteClamp(
            look.response.opacityAdd,
            -1.0F,
            4.0F,
            fallback.response.opacityAdd),
        finiteClamp(
            look.response.opacityMultiply,
            0.0F,
            16.0F,
            fallback.response.opacityMultiply),
        finiteClamp(
            look.response.pointSizeAdd,
            -256.0F,
            512.0F,
            fallback.response.pointSizeAdd),
    };
    gpu.response1 = glm::vec4{
        finiteClamp(
            look.response.pointSizeMultiply,
            0.0F,
            16.0F,
            fallback.response.pointSizeMultiply),
        finiteClamp(
            look.response.colouriseRed,
            0.0F,
            1.0F,
            fallback.response.colouriseRed),
        finiteClamp(
            look.response.colouriseGreen,
            0.0F,
            1.0F,
            fallback.response.colouriseGreen),
        finiteClamp(
            look.response.colouriseBlue,
            0.0F,
            1.0F,
            fallback.response.colouriseBlue),
    };
    gpu.response2 = glm::vec4{
        finiteClamp(
            look.response.colouriseAmount,
            0.0F,
            1.0F,
            fallback.response.colouriseAmount),
        finiteClamp(look.baseWetness, 0.0F, 1.0F, fallback.baseWetness),
        finiteClamp(look.glisten, 0.0F, 1.0F, fallback.glisten),
        finiteClamp(look.rainResponse, 0.0F, 1.0F, fallback.rainResponse),
    };
    const float organicFeatureSize =
        look.pattern == invisible_places::water::WaterSeepagePattern::WettingTrickle
            ? look.tricklePatchSizeMeters
            : look.featureSizeMeters;
    const float organicFeatureFallback =
        look.pattern == invisible_places::water::WaterSeepagePattern::WettingTrickle
            ? fallback.tricklePatchSizeMeters
            : fallback.featureSizeMeters;
    gpu.organic0 = glm::vec4{
        finiteClamp(organicFeatureSize, 0.005F, 20.0F, organicFeatureFallback),
        finiteClamp(look.contrast, 0.0F, 1.0F, fallback.contrast),
        finiteClamp(look.evolution, 0.0F, 4.0F, fallback.evolution),
        finiteClamp(look.roughness, 0.02F, 1.0F, fallback.roughness),
    };
    gpu.organic1 = glm::vec4{
        finiteClamp(look.angleResponse, 0.0F, 1.0F, fallback.angleResponse),
        finiteClamp(
            look.microNormalStrength,
            0.0F,
            1.0F,
            fallback.microNormalStrength),
        finiteClamp(look.glintDensity, 0.0F, 1.0F, fallback.glintDensity),
        finiteClamp(look.curl, 0.0F, 2.0F, fallback.curl),
    };
    gpu.organic2 = glm::vec4{
        finiteClamp(look.breakup, 0.0F, 1.0F, fallback.breakup),
        finiteClamp(
            look.downhillDriftMetersPerSecond,
            0.0F,
            4.0F,
            fallback.downhillDriftMetersPerSecond),
        // Spare lane (was trickle length; the wetting front now anchors to
        // the node's effective reach).
        0.0F,
        finiteClamp(
            look.trickleWidthMeters,
            0.001F,
            100.0F,
            fallback.trickleWidthMeters),
    };
    gpu.organic3 = glm::vec4{
        finiteClamp(
            look.trickleFrontSoftness,
            0.001F,
            10.0F,
            fallback.trickleFrontSoftness),
        0.0F,
        0.0F,
        0.0F,
    };
    constexpr float degreesToRadians = 0.01745329251994329577F;
    const float azimuth = std::remainder(
                              finiteOr(
                                  look.environmentAzimuthDegrees,
                                  fallback.environmentAzimuthDegrees),
                              360.0F) *
                          degreesToRadians;
    const float elevation = finiteClamp(
                                look.environmentElevationDegrees,
                                -89.0F,
                                89.0F,
                                fallback.environmentElevationDegrees) *
                            degreesToRadians;
    glm::vec3 environmentDirection{
        std::cos(elevation) * std::cos(azimuth),
        std::cos(elevation) * std::sin(azimuth),
        std::sin(elevation),
    };
    if (glm::dot(environmentDirection, environmentDirection) <= 1.0e-8F) {
        environmentDirection = {0.0F, 0.0F, 1.0F};
    } else {
        environmentDirection = glm::normalize(environmentDirection);
    }
    gpu.environmentDirection = glm::vec4{environmentDirection, 0.0F};
    return gpu;
}

struct alignas(16) WaterSeepageHashCellGpu {
    glm::ivec4 coordinate{0, 0, 0, 0};
    glm::uvec4 range{0U, 0U, 0U, 0U};
};

struct alignas(16) DynamicMeshFlowUniformsGpu {
    glm::uvec4 capacities{0U, 0U, 0U, 0U};
    glm::uvec4 ground{0U, 1U, 0U, 0U};
    glm::uvec4 control{1U, 0U, 1U, 0U};
    glm::uvec4 contactGrid{0U, 0U, 0U, 0U};
    glm::vec4 timing{0.0F, 1.0F / 30.0F, 1.0F, 0.0F};
    glm::vec4 boundsMinimumResolution{0.0F, 0.0F, 0.0F, 0.010F};
    glm::vec4 boundsMaximumBand{0.0F, 0.0F, 0.0F, 0.35F};
    glm::vec4 motion0{0.22F, 1.35F, 0.76F, 0.006F};
    glm::vec4 particleNoise{0.32F, 0.18F, 0.40F, 0.75F};
    glm::vec4 sharedWind{0.18F, 2.4F, 0.06F, 0.80F};
    glm::vec4 visual{0.005F, 0.18F, invisible_places::water::kWaterTrailFeatureTypeDynamicMesh, 0.8F};
    glm::vec4 responsePersistence{2.5F, 3.0F, 0.0F, 0.0F};
    glm::vec4 rockResponse{0.12F, 0.16F, 0.35F, 2.5F};
    glm::vec4 rockColour{0.18F, 0.42F, 0.55F, 0.45F};
    glm::vec4 vegetationResponse{0.18F, 0.14F, 0.55F, 3.0F};
    glm::vec4 vegetationColour{0.18F, 0.55F, 0.48F, 0.50F};
    glm::vec4 vegetationExtra{1.4F, 0.45F, 0.0F, 0.0F};
};

struct alignas(16) DynamicMeshFlowParticleGpu {
    glm::vec4 positionAge{0.0F, 0.0F, 0.0F, 0.0F};
    glm::vec4 velocityDistance{1.0F, 0.0F, 0.0F, 0.0F};
    glm::vec4 normalConfidence{0.0F, 0.0F, 1.0F, 0.0F};
    glm::uvec4 metadata{0U, 0U, 0U, 0U};
};

struct alignas(16) DynamicMeshFlowContactEventGpu {
    glm::vec4 positionTime{0.0F, 0.0F, 0.0F, -1.0F};
    glm::vec4 normalRadius{0.0F, 0.0F, 1.0F, 0.0F};
    glm::uvec4 metadata{0U, 0U, 0U, 0U};
    glm::vec4 response{0.0F, 0.0F, 0.0F, 0.0F};
    glm::vec4 vegetationExtra{0.0F, 0.0F, 0.0F, 0.0F};
};

struct alignas(16) DynamicMeshFlowRainSeedGpu {
    glm::vec4 positionBirth{0.0F, 0.0F, 0.0F, -1.0F};
    glm::uvec4 metadata{0U, 0U, 0U, 0U};
};

struct alignas(16) DynamicMeshFlowRainSeedHeaderGpu {
    std::uint32_t writeCount = 0U;
    std::uint32_t reserved0 = 0U;
    std::uint32_t reserved1 = 0U;
    std::uint32_t reserved2 = 0U;
};

constexpr std::uint32_t kDynamicMeshFlowRainSeedCapacity = 1'024U;

struct alignas(16) DynamicMeshFlowCountersGpu {
    std::uint32_t activeParticleCount = 0U;
    std::uint32_t contactEventCount = 0U;
    std::uint32_t terminalContactCount = 0U;
    std::uint32_t rainSeedParticleCount = 0U;
    std::uint32_t availableRainSeedCount = 0U;
    std::uint32_t routeSampleCount = 0U;
    std::uint32_t routeWithinBoundsCount = 0U;
    std::uint32_t routeNearSurfaceCount = 0U;
    std::uint32_t longSegmentCount = 0U;
    std::uint32_t verticalJumpCount = 0U;
    std::uint32_t maximumSegmentBits = 0U;
    std::array<std::uint32_t, 8> alignmentHistogram{};
    std::uint32_t reserved0 = 0U;
};

struct alignas(16) WaterFlowSourceUniformsGpu {
    glm::uvec4 counts0{0U, 0U, 0U, 0U};
    glm::uvec4 counts1{0U, 0U, 0U, 0U};
    glm::uvec4 surface0{0U, 1U, 0U, 0U};
    glm::uvec4 metadata{1U, 0U, 0U, 0U};
    glm::uvec4 identity{0U, 1U, 0U, 0U};
    glm::vec4 route0{0.0F, 0.010F, 0.020F, 0.004F};
    glm::vec4 lane0{0.12F, 0.006F, 0.06F, 0.18F};
    glm::vec4 guide0{0.85F, 0.35F, 0.65F, 0.85F};
    glm::vec4 trail0{0.75F, 0.010F, 0.45F, 0.045F};
    glm::vec4 shape0{0.22F, 0.85F, 0.08F, 0.0F};
    glm::vec4 fade0{0.25F, 0.10F, 0.25F, 0.10F};
};

struct alignas(16) WaterFlowInputPointGpu {
    glm::vec4 positionDistance{0.0F, 0.0F, 0.0F, 0.0F};
    glm::vec4 normalConfidence{0.0F, 0.0F, 1.0F, 1.0F};
    glm::vec4 outgoingArcDistances{0.0F, 0.0F, 0.0F, 0.0F};
    glm::vec4 laneWidth{0.0F, 1.0F, 0.0F, 0.0F};
};

// std430-compatible, source-local branch table. Route/trail output offsets are
// precomputed on the CPU so both compute passes can dispatch one Y slice per
// branch without searching or joining branch endpoints.
struct alignas(16) WaterFlowBranchGpu {
    glm::uvec4 inputRoute{0U, 0U, 0U, 0U};
    glm::uvec4 trailLane{0U, 0U, 0U, 1U};
    glm::uvec4 identity{0U, 0U, 1U, 0U};
    glm::vec4 metrics{0.0F, 0.010F, 0.0F, 0.0F};
};

static_assert(sizeof(WaterFlowSourceUniformsGpu) == 176U);
static_assert(sizeof(WaterFlowInputPointGpu) == 64U);
static_assert(sizeof(WaterFlowBranchGpu) == 64U);

struct alignas(16) DynamicMeshSurfaceCellGpu {
    glm::vec4 positionConfidence{0.0F, 0.0F, 0.0F, 0.0F};
    glm::vec4 normalAmbiguous{0.0F, 0.0F, 1.0F, 0.0F};
    glm::vec4 downhillSampleCount{1.0F, 0.0F, 0.0F, 0.0F};
};

struct alignas(16) DynamicMeshFlowEmitterGpu {
    glm::vec4 positionRadius{0.0F, 0.0F, 0.0F, 0.0F};
    glm::vec4 strengthSpeedIdSeed{1.0F, 1.0F, 0.0F, 0.0F};
};

static_assert(sizeof(DynamicMeshFlowUniformsGpu) == 272U);
static_assert(sizeof(DynamicMeshFlowParticleGpu) == 64U);
static_assert(sizeof(DynamicMeshFlowContactEventGpu) == 80U);
static_assert(sizeof(DynamicMeshFlowRainSeedGpu) == 32U);
static_assert(sizeof(DynamicMeshFlowRainSeedHeaderGpu) == 16U);
static_assert(sizeof(DynamicMeshFlowCountersGpu) == 80U);

struct alignas(16) DynamicMeshFlowAttractorGpu {
    glm::vec4 positionRadius{0.0F, 0.0F, 0.0F, 0.0F};
    glm::vec4 strengthEnabled{0.0F, 0.0F, 0.0F, 0.0F};
};

std::uint32_t WaterSeepageQualityGpu(invisible_places::water::WaterSeepageQuality quality) {
    using Quality = invisible_places::water::WaterSeepageQuality;
    switch (quality) {
        case Quality::Low:
            return 0U;
        case Quality::High:
            return 2U;
        case Quality::Auto:
        case Quality::Balanced:
            return 1U;
    }
    return 1U;
}

glm::vec3 SafeSeepageDirection(glm::vec3 value, glm::vec3 fallback) {
    if (!std::isfinite(value.x) || !std::isfinite(value.y) || !std::isfinite(value.z)) {
        value = fallback;
    }
    if (!std::isfinite(fallback.x) || !std::isfinite(fallback.y) || !std::isfinite(fallback.z)) {
        fallback = {0.0F, 0.0F, 1.0F};
    }
    if (glm::dot(value, value) <= 1.0e-8F) {
        value = fallback;
    }
    return glm::dot(value, value) > 1.0e-8F ? glm::normalize(value) : glm::vec3{0.0F, 0.0F, 1.0F};
}

WaterSeepageNodeTopologyGpu MakeWaterSeepageNodeTopologyGpu(
    const invisible_places::water::WaterSeepageRuntimeNode& node) {
    const auto finiteOr = [](float value, float fallback) {
        return std::isfinite(value) ? value : fallback;
    };
    const glm::vec3 safePosition{
        finiteOr(node.position.x, 0.0F),
        finiteOr(node.position.y, 0.0F),
        finiteOr(node.position.z, 0.0F),
    };
    WaterSeepageNodeTopologyGpu gpu;
    const glm::vec3 normal = SafeSeepageDirection(node.surfaceNormal, {0.0F, 1.0F, 0.0F});
    glm::vec3 down = node.downAxis - normal * glm::dot(node.downAxis, normal);
    down = SafeSeepageDirection(down, {0.0F, 0.0F, -1.0F});
    glm::vec3 lateral = node.lateralAxis;
    lateral -= normal * glm::dot(lateral, normal);
    lateral -= down * glm::dot(lateral, down);
    lateral = SafeSeepageDirection(lateral, glm::cross(normal, down));

    gpu.control = glm::uvec4{
        node.id,
        0U,
        0U,
        0U,
    };
    gpu.positionReach = glm::vec4{
        safePosition,
        std::max(0.001F, finiteOr(node.selectionReachLimitMeters, 2.34375F))};
    gpu.normalSurface = glm::vec4{normal, std::max(0.001F, finiteOr(node.depthToleranceMeters, 0.15F))};
    gpu.downEdge = glm::vec4{down, std::max(0.001F, finiteOr(node.edgeFeatherMeters, 0.10F))};
    gpu.lateralStart = glm::vec4{lateral, std::max(0.001F, finiteOr(node.startHalfWidthMeters, 0.06F))};
    gpu.geometry = glm::vec4{
        std::max(
            0.001F,
            finiteOr(node.selectionWidthLimitMeters, 1.215F) * 0.5F),
        std::max(
            0.0F,
            finiteOr(node.maximumUpstreamRunMeters, 0.0F)),
        0.0F,
        0.0F,
    };
    const auto guideSampleCount = node.guideValid
                                      ? std::min<std::size_t>(
                                            static_cast<std::size_t>(node.guideSampleCount),
                                            std::min(
                                                node.guideSamples.size(),
                                                WaterSeepageNodeTopologyGpu::kGuideSampleCapacity))
                                      : 0U;
    const float guideAchievedReach = std::max(
        0.0F,
        std::isfinite(node.guideAchievedReachMeters)
            ? node.guideAchievedReachMeters
            : 0.0F);
    gpu.guideControl = glm::uvec4{
        static_cast<std::uint32_t>(guideSampleCount),
        std::bit_cast<std::uint32_t>(guideAchievedReach),
        node.guideValid ? 1U : 0U,
        node.guideComplete ? 1U : 0U,
    };
    glm::vec3 previousGuideNormal = normal;
    glm::vec3 previousGuidePosition = safePosition;
    float previousStation = 0.0F;
    for (std::size_t sampleIndex = 0U; sampleIndex < guideSampleCount; ++sampleIndex) {
        const auto& sample = node.guideSamples[sampleIndex];
        const glm::vec3 position{
            finiteOr(sample.position.x, previousGuidePosition.x),
            finiteOr(sample.position.y, previousGuidePosition.y),
            finiteOr(sample.position.z, previousGuidePosition.z),
        };
        glm::vec3 sampleNormal{
            sample.normal.x,
            sample.normal.y,
            sample.normal.z,
        };
        sampleNormal = SafeSeepageDirection(sampleNormal, previousGuideNormal);
        if (glm::dot(sampleNormal, previousGuideNormal) < 0.0F) {
            sampleNormal = -sampleNormal;
        }
        const float station = std::max(
            previousStation,
            std::isfinite(sample.station) ? sample.station : previousStation);
        const float confidence = std::clamp(
            std::isfinite(sample.confidence) ? sample.confidence : 0.0F,
            0.0F,
            1.0F);
        gpu.guidePositionStation[sampleIndex] = glm::vec4{position, station};
        gpu.guideNormalConfidence[sampleIndex] = glm::vec4{sampleNormal, confidence};
        previousGuidePosition = position;
        previousGuideNormal = sampleNormal;
        previousStation = station;
    }
    return gpu;
}

struct PackedWaterSeepagePulseField {
    std::uint32_t sampleCount = 0U;
    std::uint32_t stableSpanBits = 0U;
    std::array<
        glm::vec4,
        WaterSeepageNodeParamsGpu::kPulseFieldVec4Capacity> samples{};
};

PackedWaterSeepagePulseField PackWaterSeepagePulseField(
    const invisible_places::water::WaterSeepagePulseField& field) {
    static_assert(
        WaterSeepageNodeParamsGpu::kPulseFieldVec4Capacity * 4U ==
        invisible_places::water::kWaterSeepagePulseFieldCapacity);

    PackedWaterSeepagePulseField packed;
    const float stableSpan = std::isfinite(field.stableSpanMeters)
                                 ? std::clamp(
                                       field.stableSpanMeters,
                                       0.005F,
                                       1000.0F)
                                 : 0.0F;
    const auto sampleCount = std::min<std::uint32_t>(
        field.sampleCount,
        invisible_places::water::kWaterSeepagePulseFieldCapacity);
    if (sampleCount == 0U || stableSpan <= 0.0F) {
        return packed;
    }

    packed.sampleCount = sampleCount;
    packed.stableSpanBits = std::bit_cast<std::uint32_t>(stableSpan);
    for (std::uint32_t sampleIndex = 0U;
         sampleIndex < sampleCount;
         ++sampleIndex) {
        const float sample = std::isfinite(field.samples[sampleIndex])
                                 ? std::clamp(
                                       field.samples[sampleIndex],
                                       0.0F,
                                       1.0F)
                                 : 0.0F;
        packed.samples[sampleIndex / 4U][sampleIndex % 4U] = sample;
    }
    return packed;
}

WaterSeepageNodeParamsGpu MakeWaterSeepageNodeParamsGpu(
    const invisible_places::water::WaterSeepageRuntimeNode& node) {
    const auto finiteOr = [](float value, float fallback) {
        return std::isfinite(value) ? value : fallback;
    };
    const auto finiteClamp = [&finiteOr](float value, float minimum, float maximum, float fallback) {
        return std::clamp(finiteOr(value, fallback), minimum, maximum);
    };
    WaterSeepageNodeParamsGpu gpu;
    gpu.control = glm::uvec4{
        node.id,
        WaterSeepageQualityGpu(node.resolvedQuality),
        static_cast<std::uint32_t>(node.look.blendMode),
        node.seed,
    };
    gpu.geometry = glm::vec4{
        finiteClamp(node.enabledFactor, 0.0F, 1.0F, 1.0F),
        finiteClamp(node.normalAlignment, 0.0F, 1.0F, 0.20F),
        finiteClamp(node.strength, 0.0F, 8.0F, 0.0F),
        finiteClamp(node.rainVisualStrength, 0.0F, 1.0F, 0.0F),
    };
    gpu.look = MakeWaterSeepageLookGpu(node.look);
    gpu.transitionLook = MakeWaterSeepageLookGpu(
        node.transitionLook.value_or(node.look));
    gpu.scenario = glm::vec4{
        finiteClamp(node.scenarioSpread, 0.0F, 1.0F, 0.0F),
        finiteClamp(node.transitionAmount, 0.0F, 1.0F, 0.0F),
        finiteClamp(node.wettingProgress, 0.0F, 1.0F, 1.0F),
        0.0F,
    };
    gpu.liveGeometry = glm::vec4{
        finiteClamp(node.reachMeters, 0.0F, 1000.0F, 1.25F),
        finiteClamp(node.widthMeters, 0.0F, 1000.0F, 0.75F),
        finiteClamp(node.prominence, 0.0F, 8.0F, 1.0F),
        // Least-resistance travel budget for connected support (replaces the
        // unused source half-width lane).
        finiteClamp(node.budgetMeters, 0.0F, 1000.0F, 1.5F),
    };
    for (std::size_t basisIndex = 0U; basisIndex < gpu.noiseBasis.size(); ++basisIndex) {
        glm::vec3 basis = node.noiseRotation[basisIndex];
        if (!std::isfinite(basis.x) || !std::isfinite(basis.y) || !std::isfinite(basis.z)) {
            basis = glm::vec3{0.0F};
            basis[basisIndex] = 1.0F;
        }
        gpu.noiseBasis[basisIndex] = glm::vec4{basis, 0.0F};
    }
    const auto pulseField = PackWaterSeepagePulseField(node.pulseField);
    auto transitionPulseField =
        PackWaterSeepagePulseField(node.transitionPulseField);
    // A missing transition field is safe to treat as the current field. This
    // keeps a partially prepared frame deterministic while still allowing
    // the point-level pattern selector to choose either look.
    if (transitionPulseField.sampleCount == 0U &&
        pulseField.sampleCount != 0U) {
        transitionPulseField = pulseField;
    }
    gpu.pulseFieldControl = glm::uvec4{
        pulseField.sampleCount,
        transitionPulseField.sampleCount,
        pulseField.stableSpanBits,
        transitionPulseField.stableSpanBits,
    };
    gpu.pulseField = pulseField.samples;
    gpu.transitionPulseField = transitionPulseField.samples;
    return gpu;
}

std::uint32_t SeepageHashUint(std::uint32_t value) {
    value ^= value >> 16U;
    value *= 0x7feb352dU;
    value ^= value >> 15U;
    value *= 0x846ca68bU;
    value ^= value >> 16U;
    return value;
}

std::uint32_t SeepageCellHash(std::int32_t x, std::int32_t y, std::int32_t z) {
    std::uint32_t value = static_cast<std::uint32_t>(x) * 0x8da6b343U;
    value ^= static_cast<std::uint32_t>(y) * 0xd8163841U;
    value ^= static_cast<std::uint32_t>(z) * 0xcb1ab31fU;
    return SeepageHashUint(value);
}

std::uint32_t NextSeepageHashCapacity(std::size_t occupiedCellCount) {
    const std::size_t requested = std::max<std::size_t>(2U, occupiedCellCount * 2U);
    std::size_t capacity = 1U;
    while (capacity < requested && capacity <= (std::numeric_limits<std::uint32_t>::max() / 2U)) {
        capacity *= 2U;
    }
    if (capacity < requested || capacity > std::numeric_limits<std::uint32_t>::max()) {
        throw std::runtime_error{"Seepage spatial hash exceeds the current 32-bit capacity."};
    }
    return static_cast<std::uint32_t>(capacity);
}

struct WaterSeepageGpuTopology {
    std::vector<WaterSeepageNodeTopologyGpu> nodes;
    std::vector<WaterSeepageNodeParamsGpu> params;
    std::vector<WaterSeepageHashCellGpu> hashCells;
    std::vector<invisible_places::water::WaterSeepageSupportReference> nodeReferences;
    std::uint32_t occupiedCellCount = 0U;
    std::uint32_t probeLimit = 1U;
    bool usesConnectedSupport = false;
    float cellSizeMeters = 0.50F;
    invisible_places::io::Bounds3f unionBounds{};
};

WaterSeepageGpuTopology MakeWaterSeepageGpuTopology(
    const invisible_places::water::WaterSeepageSpatialGrid& grid) {
    WaterSeepageGpuTopology result;
    const bool usesConnectedSupport = !grid.supportHashCells.empty() &&
                                      !grid.supportReferences.empty();
    const auto& sourceHashCells = usesConnectedSupport
                                      ? grid.supportHashCells
                                      : grid.hashCells;
    if (grid.nodes.size() > std::numeric_limits<std::uint32_t>::max() ||
        (usesConnectedSupport ? grid.supportReferences.size()
                              : grid.nodeReferences.size()) >
            std::numeric_limits<std::uint32_t>::max()) {
        throw std::runtime_error{"Seepage GPU payload exceeds the current 32-bit limit."};
    }
    result.usesConnectedSupport = usesConnectedSupport;
    result.cellSizeMeters = usesConnectedSupport
                                ? grid.supportCellSizeMeters
                                : grid.cellSizeMeters;
    result.unionBounds = usesConnectedSupport
                             ? grid.supportUnionBounds
                             : grid.unionBounds;
    result.nodes.reserve(grid.nodes.size());
    result.params.reserve(grid.nodes.size());
    for (const auto& node : grid.nodes) {
        result.nodes.push_back(MakeWaterSeepageNodeTopologyGpu(node));
        result.params.push_back(MakeWaterSeepageNodeParamsGpu(node));
    }

    struct OccupiedCell {
        std::int32_t x = 0;
        std::int32_t y = 0;
        std::int32_t z = 0;
        std::uint32_t referenceOffset = 0U;
        std::uint32_t referenceCount = 0U;
    };
    std::vector<OccupiedCell> occupiedCells;
    occupiedCells.reserve(sourceHashCells.size());
    result.nodeReferences.reserve(
        usesConnectedSupport ? grid.supportReferences.size()
                             : grid.nodeReferences.size());
    for (const auto& cell : sourceHashCells) {
        if (!cell.occupied || cell.referenceCount == 0U ||
            cell.referenceOffset >= (usesConnectedSupport
                                         ? grid.supportReferences.size()
                                         : grid.nodeReferences.size())) {
            continue;
        }
        const std::size_t sourceEnd = std::min<std::size_t>(
            usesConnectedSupport ? grid.supportReferences.size()
                                 : grid.nodeReferences.size(),
            static_cast<std::size_t>(cell.referenceOffset) + static_cast<std::size_t>(cell.referenceCount));
        const auto targetOffset = static_cast<std::uint32_t>(result.nodeReferences.size());
        for (std::size_t referenceIndex = cell.referenceOffset; referenceIndex < sourceEnd; ++referenceIndex) {
            const auto nodeIndex = usesConnectedSupport
                                       ? grid.supportReferences[referenceIndex].nodeIndex
                                       : grid.nodeReferences[referenceIndex];
            if (nodeIndex < result.nodes.size()) {
                if (usesConnectedSupport) {
                    result.nodeReferences.push_back(
                        grid.supportReferences[referenceIndex]);
                } else {
                    result.nodeReferences.push_back({.nodeIndex = nodeIndex});
                }
            }
        }
        const auto targetCount = static_cast<std::uint32_t>(result.nodeReferences.size()) - targetOffset;
        if (targetCount > 0U) {
            occupiedCells.push_back({
                .x = cell.x,
                .y = cell.y,
                .z = cell.z,
                .referenceOffset = targetOffset,
                .referenceCount = targetCount,
            });
        }
    }

    const auto capacity = NextSeepageHashCapacity(occupiedCells.size());
    result.hashCells.assign(capacity, WaterSeepageHashCellGpu{});
    for (const auto& cell : occupiedCells) {
        const std::uint32_t initialSlot = SeepageCellHash(cell.x, cell.y, cell.z) & (capacity - 1U);
        bool inserted = false;
        for (std::uint32_t probe = 0U; probe < capacity; ++probe) {
            const std::uint32_t slot = (initialSlot + probe) & (capacity - 1U);
            auto& target = result.hashCells[slot];
            if (target.coordinate.w != 0) {
                continue;
            }
            target.coordinate = glm::ivec4{cell.x, cell.y, cell.z, 1};
            target.range = glm::uvec4{cell.referenceOffset, cell.referenceCount, 0U, 0U};
            result.probeLimit = std::max(result.probeLimit, probe + 1U);
            inserted = true;
            break;
        }
        if (!inserted) {
            throw std::runtime_error{"Seepage spatial hash insertion failed."};
        }
    }
    result.occupiedCellCount = static_cast<std::uint32_t>(occupiedCells.size());

    if (result.nodes.empty()) {
        result.nodes.push_back(WaterSeepageNodeTopologyGpu{});
        result.params.push_back(WaterSeepageNodeParamsGpu{});
    }
    if (result.nodeReferences.empty()) {
        result.nodeReferences.push_back({});
    }
    return result;
}

struct alignas(16) GaussianSplatPushConstants {
    glm::mat4 localToWorld{1.0F};
    glm::vec4 layerTint{0.94F, 0.82F, 0.60F, 1.0F};
    glm::vec4 style{1.0F, 1.0F, 1.0F, 1.0F};
    glm::uvec4 control{0U, 0U, 1U, 0U};
    glm::vec4 extra{1.5F, 0.0F, 0.0F, 0.0F};
};

struct alignas(16) HighQualityGaussianLayerStyle {
    glm::mat4 localToWorld{1.0F};
    glm::vec4 layerTint{0.94F, 0.82F, 0.60F, 1.0F};
    glm::vec4 style{1.0F, 1.0F, 1.0F, 1.0F};
    glm::uvec4 control{0U, 0U, 1U, 0U};
};

struct alignas(16) HighQualityGaussianPushConstants {
    glm::vec4 extra{1.5F, 0.0F, 0.0F, 0.0F};
};

struct alignas(16) PostProcessPushConstants {
    glm::vec4 edl{0.0F, 24.0F, 0.35F, 1.0F};
    glm::vec4 preview{0.0F, 0.0F, 0.0F, 0.0F};
    glm::vec4 temporalOverlay{0.0F, 1.0F, 0.0F, 0.0F};
    // x: stale source (1=A, 2=B, 0=off); y/z: cached projection scale;
    // w: cached focus depth used where sparse point depth has no sample.
    glm::vec4 temporalReprojection{0.0F, 1.0F, 1.0F, 1.0F};
    glm::mat4 staleViewToCurrentClip{1.0F};
};

static_assert(
    sizeof(PostProcessPushConstants) == 128U,
    "Post-process push constants must fit Vulkan's portable 128-byte floor.");

struct alignas(16) EyeDomeLightingExportPushConstants {
    glm::vec4 edl{1.0F, 24.0F, 0.35F, 1.0F};
};

std::string NormalizeScalarFieldName(std::string_view name) {
    std::string normalized;
    normalized.reserve(name.size());
    for (const char character : name) {
        const auto byte = static_cast<unsigned char>(character);
        if (std::isalnum(byte) == 0) {
            continue;
        }
        normalized.push_back(static_cast<char>(std::tolower(byte)));
    }
    return normalized;
}

std::optional<std::uint32_t> FindScalarFieldSlotByNormalizedName(
    const std::vector<invisible_places::io::ScalarFieldStats>& scalarFields,
    std::initializer_list<std::string_view> exactNames,
    std::string_view containsName) {
    std::optional<std::uint32_t> containsMatch;
    for (std::size_t index = 0; index < scalarFields.size(); ++index) {
        const auto normalized = NormalizeScalarFieldName(scalarFields[index].name);
        for (const auto exactName : exactNames) {
            if (normalized == exactName) {
                return static_cast<std::uint32_t>(index);
            }
        }
        if (!containsMatch.has_value() && normalized.find(containsName) != std::string::npos) {
            containsMatch = static_cast<std::uint32_t>(index);
        }
    }
    return containsMatch;
}

std::optional<std::uint32_t> FindRoughnessScalarFieldSlot(
    const std::vector<invisible_places::io::ScalarFieldStats>& scalarFields) {
    return FindScalarFieldSlotByNormalizedName(
        scalarFields,
        {"roughness", "scalarroughness"},
        "roughness");
}

std::optional<std::uint32_t> FindGroundIdScalarFieldSlot(
    const std::vector<invisible_places::io::ScalarFieldStats>& scalarFields) {
    return FindScalarFieldSlotByNormalizedName(
        scalarFields,
        {"groundid", "scalargroundid"},
        "groundid");
}

std::optional<std::uint32_t> FindSurfaceStabilityScalarFieldSlot(
    const std::vector<invisible_places::io::ScalarFieldStats>& scalarFields) {
    // The packed column is appended verbatim by the sidecar loader, so exact
    // identity is the correct predicate — and it must match the offline
    // renderer's resolver, or preview and export can disagree about whether
    // surface selection applies.
    for (std::size_t index = 0U; index < scalarFields.size(); ++index) {
        if (scalarFields[index].name ==
            invisible_places::io::kPointCloudSurfaceStabilityPackedFieldName) {
            return static_cast<std::uint32_t>(index);
        }
    }
    return std::nullopt;
}

constexpr std::uint32_t kSurfelVerticesPerPoint = 6U;
constexpr std::uint32_t kMaxSurfelEncodedPointCount =
    std::numeric_limits<std::uint32_t>::max() / kSurfelVerticesPerPoint;

std::vector<std::uint32_t> SanitizePointIndices(
    const std::vector<std::uint32_t>& indices,
    std::uint32_t pointCount) {
    std::vector<std::uint32_t> sanitized;
    sanitized.reserve(indices.size());
    for (const auto index : indices) {
        if (index < pointCount) {
            sanitized.push_back(index);
        }
    }
    std::sort(sanitized.begin(), sanitized.end());
    sanitized.erase(std::unique(sanitized.begin(), sanitized.end()), sanitized.end());
    return sanitized;
}

bool MatricesApproximatelyEqual(const glm::mat4& left, const glm::mat4& right, float epsilon = 1.0e-6F) {
    for (int column = 0; column < 4; ++column) {
        for (int row = 0; row < 4; ++row) {
            if (std::abs(left[column][row] - right[column][row]) > epsilon) {
                return false;
            }
        }
    }

    return true;
}

constexpr std::array<const char*, 1> kRequiredDeviceExtensions = {
    VK_KHR_SWAPCHAIN_EXTENSION_NAME,
};

[[nodiscard]] bool HasExtension(
    const std::vector<VkExtensionProperties>& extensions,
    std::string_view name) {
    return std::any_of(extensions.begin(), extensions.end(), [name](const VkExtensionProperties& extension) {
        return name == extension.extensionName;
    });
}

std::vector<VkExtensionProperties> EnumerateDeviceExtensions(VkPhysicalDevice device) {
    std::uint32_t count = 0;
    vkEnumerateDeviceExtensionProperties(device, nullptr, &count, nullptr);
    std::vector<VkExtensionProperties> extensions(count);
    vkEnumerateDeviceExtensionProperties(device, nullptr, &count, extensions.data());
    return extensions;
}

QueueFamilySelection FindQueueFamilies(VkPhysicalDevice device, VkSurfaceKHR surface) {
    QueueFamilySelection selection;

    std::uint32_t queueFamilyCount = 0;
    vkGetPhysicalDeviceQueueFamilyProperties(device, &queueFamilyCount, nullptr);
    std::vector<VkQueueFamilyProperties> queueFamilies(queueFamilyCount);
    vkGetPhysicalDeviceQueueFamilyProperties(device, &queueFamilyCount, queueFamilies.data());

    for (std::uint32_t index = 0; index < queueFamilyCount; ++index) {
        const bool supportsGraphics = (queueFamilies[index].queueFlags & VK_QUEUE_GRAPHICS_BIT) != 0;
        if (supportsGraphics && !selection.graphicsFamily.has_value()) {
            selection.graphicsFamily = index;
        }

        VkBool32 presentSupported = VK_FALSE;
        vkGetPhysicalDeviceSurfaceSupportKHR(device, index, surface, &presentSupported);
        if (presentSupported == VK_TRUE) {
            selection.presentFamily = index;
        }

        if (selection.IsComplete()) {
            break;
        }
    }

    return selection;
}

SwapchainSupport QuerySwapchainSupport(VkPhysicalDevice device, VkSurfaceKHR surface) {
    SwapchainSupport support;
    vkGetPhysicalDeviceSurfaceCapabilitiesKHR(device, surface, &support.capabilities);

    std::uint32_t formatCount = 0;
    vkGetPhysicalDeviceSurfaceFormatsKHR(device, surface, &formatCount, nullptr);
    support.formats.resize(formatCount);
    if (formatCount > 0) {
        vkGetPhysicalDeviceSurfaceFormatsKHR(device, surface, &formatCount, support.formats.data());
    }

    std::uint32_t presentModeCount = 0;
    vkGetPhysicalDeviceSurfacePresentModesKHR(device, surface, &presentModeCount, nullptr);
    support.presentModes.resize(presentModeCount);
    if (presentModeCount > 0) {
        vkGetPhysicalDeviceSurfacePresentModesKHR(
            device, surface, &presentModeCount, support.presentModes.data());
    }

    return support;
}

bool IsDeviceSuitable(VkPhysicalDevice device, VkSurfaceKHR surface, bool* portabilitySubsetEnabled) {
    const auto queueFamilies = FindQueueFamilies(device, surface);
    if (!queueFamilies.IsComplete()) {
        return false;
    }

    const auto extensions = EnumerateDeviceExtensions(device);
    for (const char* requiredExtension : kRequiredDeviceExtensions) {
        if (!HasExtension(extensions, requiredExtension)) {
            return false;
        }
    }

    const auto swapchainSupport = QuerySwapchainSupport(device, surface);
    if (swapchainSupport.formats.empty() || swapchainSupport.presentModes.empty()) {
        return false;
    }

    VkPhysicalDeviceFeatures features{};
    vkGetPhysicalDeviceFeatures(device, &features);
    if (features.largePoints == VK_FALSE) {
        return false;
    }

    VkPhysicalDeviceProperties properties{};
    vkGetPhysicalDeviceProperties(device, &properties);
    if (properties.limits.maxUniformBufferRange < sizeof(PointCloudStyleGpu) ||
        properties.limits.maxVertexOutputComponents <
            kPointCloudMaximumInterstageComponents ||
        properties.limits.maxFragmentInputComponents <
            kPointCloudMaximumInterstageComponents) {
        return false;
    }

    if (portabilitySubsetEnabled != nullptr) {
        *portabilitySubsetEnabled = HasExtension(extensions, VK_KHR_PORTABILITY_SUBSET_EXTENSION_NAME);
    }

    return true;
}

VkSurfaceFormatKHR ChooseSurfaceFormat(const std::vector<VkSurfaceFormatKHR>& formats) {
    for (const auto& format : formats) {
        if (format.format == VK_FORMAT_B8G8R8A8_SRGB &&
            format.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR) {
            return format;
        }
    }

    return formats.front();
}

VkPresentModeKHR ChoosePresentMode(const std::vector<VkPresentModeKHR>& presentModes) {
    for (const auto& mode : presentModes) {
        if (mode == VK_PRESENT_MODE_MAILBOX_KHR) {
            return mode;
        }
    }

    return VK_PRESENT_MODE_FIFO_KHR;
}

VkExtent2D ChooseExtent(GLFWwindow* window, const VkSurfaceCapabilitiesKHR& capabilities) {
    if (capabilities.currentExtent.width != std::numeric_limits<std::uint32_t>::max()) {
        return capabilities.currentExtent;
    }

    int width = 0;
    int height = 0;
    glfwGetFramebufferSize(window, &width, &height);

    VkExtent2D extent{
        static_cast<std::uint32_t>(width),
        static_cast<std::uint32_t>(height),
    };

    extent.width = std::clamp(
        extent.width, capabilities.minImageExtent.width, capabilities.maxImageExtent.width);
    extent.height = std::clamp(
        extent.height, capabilities.minImageExtent.height, capabilities.maxImageExtent.height);
    return extent;
}

void Check(VkResult result, std::string_view context) {
    if (result != VK_SUCCESS) {
        throw std::runtime_error(
            std::string{context} + " failed with VkResult " + std::to_string(static_cast<int>(result)));
    }
}

void CheckImGuiResult(VkResult result) {
    Check(result, "ImGui Vulkan backend");
}

void ApplyImGuiStyle() {
    ImGui::StyleColorsLight();
    auto& style = ImGui::GetStyle();
    style.WindowRounding = 18.0F;
    style.FrameRounding = 10.0F;
    style.GrabRounding = 10.0F;
    style.PopupRounding = 12.0F;
    style.ScrollbarRounding = 10.0F;
    style.WindowPadding = ImVec2{16.0F, 14.0F};
    style.FramePadding = ImVec2{10.0F, 7.0F};
    style.ItemSpacing = ImVec2{10.0F, 8.0F};

    auto& colors = style.Colors;
    colors[ImGuiCol_WindowBg] = ImVec4{0.95F, 0.94F, 0.90F, 0.93F};
    colors[ImGuiCol_ChildBg] = ImVec4{0.97F, 0.96F, 0.93F, 0.86F};
    colors[ImGuiCol_Border] = ImVec4{0.63F, 0.58F, 0.47F, 0.35F};
    colors[ImGuiCol_FrameBg] = ImVec4{0.90F, 0.88F, 0.82F, 0.92F};
    colors[ImGuiCol_FrameBgHovered] = ImVec4{0.83F, 0.80F, 0.72F, 0.94F};
    colors[ImGuiCol_FrameBgActive] = ImVec4{0.78F, 0.74F, 0.63F, 0.96F};
    colors[ImGuiCol_TitleBg] = ImVec4{0.89F, 0.86F, 0.76F, 0.95F};
    colors[ImGuiCol_TitleBgActive] = ImVec4{0.85F, 0.80F, 0.64F, 0.98F};
    colors[ImGuiCol_Button] = ImVec4{0.71F, 0.63F, 0.44F, 0.86F};
    colors[ImGuiCol_ButtonHovered] = ImVec4{0.76F, 0.67F, 0.46F, 0.92F};
    colors[ImGuiCol_ButtonActive] = ImVec4{0.62F, 0.54F, 0.37F, 0.95F};
    colors[ImGuiCol_Header] = ImVec4{0.78F, 0.71F, 0.53F, 0.75F};
    colors[ImGuiCol_HeaderHovered] = ImVec4{0.81F, 0.74F, 0.55F, 0.87F};
    colors[ImGuiCol_HeaderActive] = ImVec4{0.70F, 0.62F, 0.46F, 0.94F};
    colors[ImGuiCol_SliderGrab] = ImVec4{0.53F, 0.44F, 0.24F, 0.92F};
    colors[ImGuiCol_SliderGrabActive] = ImVec4{0.45F, 0.37F, 0.19F, 0.98F};
}

const invisible_places::io::ScalarFieldStats* ResolveBindingScalarFieldStats(
    const invisible_places::style::RenderParameterBinding& binding,
    const std::vector<invisible_places::io::ScalarFieldStats>& scalarFields) {
    if (binding.fieldMap.fieldSlot < 0 ||
        static_cast<std::size_t>(binding.fieldMap.fieldSlot) >= scalarFields.size()) {
        return nullptr;
    }

    return &scalarFields[static_cast<std::size_t>(binding.fieldMap.fieldSlot)];
}

PointCloudBindingGpu MakePointCloudBindingGpu(
    const invisible_places::style::RenderParameterBinding& binding,
    const std::vector<invisible_places::io::ScalarFieldStats>& scalarFields,
    float inactiveDefault) {
    const auto* fieldStats = ResolveBindingScalarFieldStats(binding, scalarFields);
    const bool useFieldMapping =
        binding.active &&
        binding.mode == invisible_places::style::ParameterSourceMode::FieldMapped &&
        fieldStats != nullptr;
    PointCloudBindingGpu gpuBinding;
    gpuBinding.constantValue = glm::vec4{
        binding.active ? binding.constantValue[0] : inactiveDefault,
        binding.constantValue[1],
        binding.constantValue[2],
        binding.constantValue[3],
    };
    gpuBinding.range = glm::vec4{
        invisible_places::style::ResolveBindingInputMinimum(binding, fieldStats),
        invisible_places::style::ResolveBindingInputMaximum(binding, fieldStats),
        binding.fieldMap.outputMin,
        binding.fieldMap.outputMax,
    };
    gpuBinding.extra = glm::vec4{binding.fieldMap.gamma, 0.0F, 0.0F, 0.0F};
    gpuBinding.control = glm::uvec4{
        useFieldMapping
            ? static_cast<std::uint32_t>(invisible_places::style::ParameterSourceMode::FieldMapped)
            : static_cast<std::uint32_t>(invisible_places::style::ParameterSourceMode::Constant),
        useFieldMapping
            ? static_cast<std::uint32_t>(binding.fieldMap.fieldSlot)
            : 0xFFFFFFFFU,
        useFieldMapping ? binding.fieldMap.flags : 0U,
        binding.active ? 1U : 0U,
    };
    return gpuBinding;
}

std::uint32_t InactivePointBindingCount(const renderer::pointcloud::PointCloudStyleState& style) {
    std::uint32_t count = 0;
    count += style.pointSize.active ? 0U : 1U;
    count += style.surfelDiameter.active ? 0U : 1U;
    count += style.opacity.active ? 0U : 1U;
    count += style.emissiveStrength.active ? 0U : 1U;
    count += style.depthFade.active ? 0U : 1U;
    count += style.colormapPosition.active ? 0U : 1U;
    return count;
}

void UpdateImGuiPlatformWindowsIfNeeded() {
    if (ImGui::GetCurrentContext() == nullptr ||
        (ImGui::GetIO().ConfigFlags & ImGuiConfigFlags_ViewportsEnable) == 0) {
        return;
    }

    GLFWwindow* backupContext = glfwGetCurrentContext();
    ImGui::UpdatePlatformWindows();
    ImGui::RenderPlatformWindowsDefault();
    glfwMakeContextCurrent(backupContext);
}

void ScalePointCloudBindingGpu(PointCloudBindingGpu* binding, float scale) {
    if (binding == nullptr) {
        return;
    }

    const float safeScale = std::max(0.001F, scale);
    binding->constantValue.x *= safeScale;
    binding->range.z *= safeScale;
    binding->range.w *= safeScale;
}

std::uint16_t FloatToHalfBits(float value) {
    return Imath::half{std::max(0.0F, value)}.bits();
}

VkDescriptorPoolSize MakePoolSize(VkDescriptorType type, std::uint32_t descriptorCount) {
    return VkDescriptorPoolSize{type, descriptorCount};
}

VkPipelineColorBlendAttachmentState MakeAlphaBlendAttachment() {
    VkPipelineColorBlendAttachmentState attachment{};
    attachment.colorWriteMask =
        VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
    attachment.blendEnable = VK_TRUE;
    attachment.srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
    attachment.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
    attachment.colorBlendOp = VK_BLEND_OP_ADD;
    attachment.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
    attachment.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
    attachment.alphaBlendOp = VK_BLEND_OP_ADD;
    return attachment;
}

VkPipelineColorBlendAttachmentState MakeAdditiveBlendAttachment() {
    VkPipelineColorBlendAttachmentState attachment{};
    attachment.colorWriteMask =
        VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
    attachment.blendEnable = VK_TRUE;
    attachment.srcColorBlendFactor = VK_BLEND_FACTOR_ONE;
    attachment.dstColorBlendFactor = VK_BLEND_FACTOR_ONE;
    attachment.colorBlendOp = VK_BLEND_OP_ADD;
    attachment.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
    attachment.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
    attachment.alphaBlendOp = VK_BLEND_OP_ADD;
    return attachment;
}

VkPipelineColorBlendAttachmentState MakeRevealageBlendAttachment() {
    VkPipelineColorBlendAttachmentState attachment{};
    attachment.colorWriteMask = VK_COLOR_COMPONENT_R_BIT;
    attachment.blendEnable = VK_TRUE;
    attachment.srcColorBlendFactor = VK_BLEND_FACTOR_ZERO;
    attachment.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_COLOR;
    attachment.colorBlendOp = VK_BLEND_OP_ADD;
    attachment.srcAlphaBlendFactor = VK_BLEND_FACTOR_ZERO;
    attachment.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
    attachment.alphaBlendOp = VK_BLEND_OP_ADD;
    return attachment;
}

VkPipelineColorBlendAttachmentState MakePremultipliedAlphaBlendAttachment() {
    VkPipelineColorBlendAttachmentState attachment{};
    attachment.colorWriteMask =
        VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
    attachment.blendEnable = VK_TRUE;
    attachment.srcColorBlendFactor = VK_BLEND_FACTOR_ONE;
    attachment.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
    attachment.colorBlendOp = VK_BLEND_OP_ADD;
    attachment.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
    attachment.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
    attachment.alphaBlendOp = VK_BLEND_OP_ADD;
    return attachment;
}

VkShaderModule CreateShaderModule(VkDevice device, const std::vector<char>& code, const char* label) {
    VkShaderModuleCreateInfo moduleInfo{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};
    moduleInfo.codeSize = code.size();
    moduleInfo.pCode = reinterpret_cast<const std::uint32_t*>(code.data());

    VkShaderModule module = VK_NULL_HANDLE;
    Check(vkCreateShaderModule(device, &moduleInfo, nullptr, &module), label);
    return module;
}

bool FormatSupportsOptimalFeatures(
    VkPhysicalDevice physicalDevice,
    VkFormat format,
    VkFormatFeatureFlags requiredFeatures) {
    VkFormatProperties properties{};
    vkGetPhysicalDeviceFormatProperties(physicalDevice, format, &properties);
    return (properties.optimalTilingFeatures & requiredFeatures) == requiredFeatures;
}

ImTextureID TextureIdFromDescriptorSet(VkDescriptorSet descriptorSet) {
    return static_cast<ImTextureID>(reinterpret_cast<std::uintptr_t>(descriptorSet));
}

}  // namespace

VulkanViewportShell::VulkanViewportShell(GLFWwindow* window) : window_(window) {
    if (window_ == nullptr) {
        throw std::runtime_error{"Vulkan viewport requires a valid GLFW window."};
    }

    glfwSetWindowUserPointer(window_, this);
    glfwSetFramebufferSizeCallback(window_, &VulkanViewportShell::FramebufferResizeCallback);

    CreateInstance();
    CreateSurface();
    PickPhysicalDevice();
    CreateLogicalDevice();
    CreateSwapchain();
    CreateImageViews();
    CreateRenderPass();
    CreatePresentRenderPass();
    CreatePointDescriptorSetLayout();
    CreatePointDepthSortDescriptorSetLayout();
    CreateDynamicMeshFlowDescriptorSetLayout();
    CreateWaterFlowSourceDescriptorSetLayout();
    CreateRainDescriptorSetLayout();
    CreateWaterSurfacePreprocessDescriptorSetLayout();
    CreateGaussianSplatDescriptorSetLayout();
    CreateHighQualityGaussianSplatDescriptorSetLayout();
    CreateCompositeDescriptorSetLayout();
    CreatePostProcessDescriptorSetLayout();
    CreateEyeDomeLightingExportDescriptorSetLayout();
    CreateDescriptorPools();
    CreatePostProcessSampler();
    CreateUniformResources();
    CreateRainResources();
    CreateSceneColorResources();
    CreateDepthResources();
    CreateAccumulationResources();
    CreateLinearDepthResources();
    CreatePointPipelines();
    CreatePointDepthSortPipeline();
    CreateDynamicMeshFlowComputePipeline();
    CreateWaterFlowSourceComputePipelines();
    CreateRainPipelines();
    CreateWaterSurfacePreprocessPipeline();
    CreateGaussianSplatPipeline();
    CreateHighQualityGaussianSplatPipeline();
    CreateCompositePipeline();
    CreatePostProcessPipeline();
    CreateEyeDomeLightingExportPipeline();
    CreateFramebuffers();
    CreatePresentFramebuffers();
    CreateCommandPool();
    CreateCommandBuffers();
    CreateSyncObjects();
    CreateOrUpdateCompositeDescriptorSet();
    CreateOrUpdatePostProcessDescriptorSets();
    CreateImGuiResources();
    UploadImGuiFonts();
}

VulkanViewportShell::~VulkanViewportShell() {
    if (device_ != VK_NULL_HANDLE) {
        vkDeviceWaitIdle(device_);
    }

    ClearImGuiPreviewImageTexture();

    if (ImGui::GetCurrentContext() != nullptr) {
        ImGui_ImplVulkan_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();
    }

    ResetPointCloudReferencingCommandBuffers();
    FlushDeferredPointCloudDestroys(true);
    for (auto& resources : pointCloudResources_) {
        RetirePointCloudRenderDescriptors(&resources);
    }
    for (auto& resources : pointCloudResources_) {
        DestroyPointCloudResources(&resources);
    }
    pointCloudResources_.clear();

    for (auto& resources : gaussianSplatResources_) {
        CleanupGaussianSplatResources(&resources);
    }
    gaussianSplatResources_.clear();
    CleanupHighQualityGaussianScene();
    CleanupExrExportResources();
    CleanupRainResources();

    CleanupSwapchain();

    for (auto& frame : frameResources_) {
        DestroyBuffer(&frame.uniformBuffer);
        if (frame.imageAvailableSemaphore != VK_NULL_HANDLE) {
            vkDestroySemaphore(device_, frame.imageAvailableSemaphore, nullptr);
            frame.imageAvailableSemaphore = VK_NULL_HANDLE;
        }
        if (frame.fence != VK_NULL_HANDLE) {
            vkDestroyFence(device_, frame.fence, nullptr);
            frame.fence = VK_NULL_HANDLE;
        }
        if (frame.timestampQueryPool != VK_NULL_HANDLE) {
            vkDestroyQueryPool(device_, frame.timestampQueryPool, nullptr);
            frame.timestampQueryPool = VK_NULL_HANDLE;
        }
    }

    if (commandPool_ != VK_NULL_HANDLE) {
        vkDestroyCommandPool(device_, commandPool_, nullptr);
        commandPool_ = VK_NULL_HANDLE;
    }

    if (pointDepthPrepassPipeline_ != VK_NULL_HANDLE) {
        vkDestroyPipeline(device_, pointDepthPrepassPipeline_, nullptr);
    }
    if (pointAccumulationPipeline_ != VK_NULL_HANDLE) {
        vkDestroyPipeline(device_, pointAccumulationPipeline_, nullptr);
    }
    if (pointAccumulationDepthTestedPipeline_ != VK_NULL_HANDLE) {
        vkDestroyPipeline(device_, pointAccumulationDepthTestedPipeline_, nullptr);
    }
    if (pointConstantSimpleAccumulationPipeline_ != VK_NULL_HANDLE) {
        vkDestroyPipeline(device_, pointConstantSimpleAccumulationPipeline_, nullptr);
    }
    if (pointOpaqueHardDiscPipeline_ != VK_NULL_HANDLE) {
        vkDestroyPipeline(device_, pointOpaqueHardDiscPipeline_, nullptr);
    }
    if (pointFastBasicPipeline_ != VK_NULL_HANDLE) {
        vkDestroyPipeline(device_, pointFastBasicPipeline_, nullptr);
    }
    if (pointSortedDepthAovPipeline_ != VK_NULL_HANDLE) {
        vkDestroyPipeline(device_, pointSortedDepthAovPipeline_, nullptr);
    }
    if (pointSortedAlphaPipeline_ != VK_NULL_HANDLE) {
        vkDestroyPipeline(device_, pointSortedAlphaPipeline_, nullptr);
    }
    if (pointSortedAlphaHybridPipeline_ != VK_NULL_HANDLE) {
        vkDestroyPipeline(
            device_,
            pointSortedAlphaHybridPipeline_,
            nullptr);
    }
    if (pointSortedEmissionCompositePipeline_ != VK_NULL_HANDLE) {
        vkDestroyPipeline(
            device_,
            pointSortedEmissionCompositePipeline_,
            nullptr);
    }
    if (pointDepthSortPipeline_ != VK_NULL_HANDLE) {
        vkDestroyPipeline(device_, pointDepthSortPipeline_, nullptr);
    }
    if (dynamicMeshFlowComputePipeline_ != VK_NULL_HANDLE) {
        vkDestroyPipeline(device_, dynamicMeshFlowComputePipeline_, nullptr);
    }
    if (waterFlowRouteComputePipeline_ != VK_NULL_HANDLE) {
        vkDestroyPipeline(device_, waterFlowRouteComputePipeline_, nullptr);
    }
    if (waterFlowTrailComputePipeline_ != VK_NULL_HANDLE) {
        vkDestroyPipeline(device_, waterFlowTrailComputePipeline_, nullptr);
    }
    if (rainComputePipeline_ != VK_NULL_HANDLE) {
        vkDestroyPipeline(device_, rainComputePipeline_, nullptr);
    }
    if (waterSurfacePreprocessPipeline_ != VK_NULL_HANDLE) {
        vkDestroyPipeline(device_, waterSurfacePreprocessPipeline_, nullptr);
    }
    if (rainPipeline_ != VK_NULL_HANDLE) {
        vkDestroyPipeline(device_, rainPipeline_, nullptr);
    }
    if (surfelDepthPrepassPipeline_ != VK_NULL_HANDLE) {
        vkDestroyPipeline(device_, surfelDepthPrepassPipeline_, nullptr);
    }
    if (surfelAccumulationPipeline_ != VK_NULL_HANDLE) {
        vkDestroyPipeline(device_, surfelAccumulationPipeline_, nullptr);
    }
    if (surfelAccumulationDepthTestedPipeline_ != VK_NULL_HANDLE) {
        vkDestroyPipeline(device_, surfelAccumulationDepthTestedPipeline_, nullptr);
    }
    if (surfelConstantSimpleAccumulationPipeline_ != VK_NULL_HANDLE) {
        vkDestroyPipeline(device_, surfelConstantSimpleAccumulationPipeline_, nullptr);
    }
    if (surfelOpaqueHardDiscPipeline_ != VK_NULL_HANDLE) {
        vkDestroyPipeline(device_, surfelOpaqueHardDiscPipeline_, nullptr);
    }
    if (gaussianSplatPipeline_ != VK_NULL_HANDLE) {
        vkDestroyPipeline(device_, gaussianSplatPipeline_, nullptr);
    }
    if (highQualityGaussianSplatPipeline_ != VK_NULL_HANDLE) {
        vkDestroyPipeline(device_, highQualityGaussianSplatPipeline_, nullptr);
    }
    if (compositePipeline_ != VK_NULL_HANDLE) {
        vkDestroyPipeline(device_, compositePipeline_, nullptr);
    }
    if (postProcessPipeline_ != VK_NULL_HANDLE) {
        vkDestroyPipeline(device_, postProcessPipeline_, nullptr);
    }
    if (eyeDomeLightingExportPipeline_ != VK_NULL_HANDLE) {
        vkDestroyPipeline(device_, eyeDomeLightingExportPipeline_, nullptr);
    }
    if (pointPipelineLayout_ != VK_NULL_HANDLE) {
        vkDestroyPipelineLayout(device_, pointPipelineLayout_, nullptr);
    }
    if (pointDepthSortPipelineLayout_ != VK_NULL_HANDLE) {
        vkDestroyPipelineLayout(device_, pointDepthSortPipelineLayout_, nullptr);
    }
    if (dynamicMeshFlowPipelineLayout_ != VK_NULL_HANDLE) {
        vkDestroyPipelineLayout(device_, dynamicMeshFlowPipelineLayout_, nullptr);
    }
    if (waterFlowSourcePipelineLayout_ != VK_NULL_HANDLE) {
        vkDestroyPipelineLayout(device_, waterFlowSourcePipelineLayout_, nullptr);
    }
    if (rainPipelineLayout_ != VK_NULL_HANDLE) {
        vkDestroyPipelineLayout(device_, rainPipelineLayout_, nullptr);
    }
    if (waterSurfacePreprocessPipelineLayout_ != VK_NULL_HANDLE) {
        vkDestroyPipelineLayout(device_, waterSurfacePreprocessPipelineLayout_, nullptr);
    }
    if (gaussianSplatPipelineLayout_ != VK_NULL_HANDLE) {
        vkDestroyPipelineLayout(device_, gaussianSplatPipelineLayout_, nullptr);
    }
    if (highQualityGaussianSplatPipelineLayout_ != VK_NULL_HANDLE) {
        vkDestroyPipelineLayout(device_, highQualityGaussianSplatPipelineLayout_, nullptr);
    }
    if (compositePipelineLayout_ != VK_NULL_HANDLE) {
        vkDestroyPipelineLayout(device_, compositePipelineLayout_, nullptr);
    }
    if (postProcessPipelineLayout_ != VK_NULL_HANDLE) {
        vkDestroyPipelineLayout(device_, postProcessPipelineLayout_, nullptr);
    }
    if (eyeDomeLightingExportPipelineLayout_ != VK_NULL_HANDLE) {
        vkDestroyPipelineLayout(device_, eyeDomeLightingExportPipelineLayout_, nullptr);
    }
    if (postProcessSampler_ != VK_NULL_HANDLE) {
        vkDestroySampler(device_, postProcessSampler_, nullptr);
    }
    if (descriptorPool_ != VK_NULL_HANDLE) {
        vkDestroyDescriptorPool(device_, descriptorPool_, nullptr);
    }
    if (gaussianSplatDescriptorPool_ != VK_NULL_HANDLE) {
        vkDestroyDescriptorPool(device_, gaussianSplatDescriptorPool_, nullptr);
    }
    if (imguiDescriptorPool_ != VK_NULL_HANDLE) {
        vkDestroyDescriptorPool(device_, imguiDescriptorPool_, nullptr);
    }
    if (pointDescriptorSetLayout_ != VK_NULL_HANDLE) {
        vkDestroyDescriptorSetLayout(device_, pointDescriptorSetLayout_, nullptr);
    }
    if (pointDepthSortDescriptorSetLayout_ != VK_NULL_HANDLE) {
        vkDestroyDescriptorSetLayout(
            device_,
            pointDepthSortDescriptorSetLayout_,
            nullptr);
    }
    if (dynamicMeshFlowDescriptorSetLayout_ != VK_NULL_HANDLE) {
        vkDestroyDescriptorSetLayout(device_, dynamicMeshFlowDescriptorSetLayout_, nullptr);
    }
    if (waterFlowSourceDescriptorSetLayout_ != VK_NULL_HANDLE) {
        vkDestroyDescriptorSetLayout(device_, waterFlowSourceDescriptorSetLayout_, nullptr);
    }
    if (rainDescriptorSetLayout_ != VK_NULL_HANDLE) {
        vkDestroyDescriptorSetLayout(device_, rainDescriptorSetLayout_, nullptr);
    }
    if (waterSurfacePreprocessDescriptorSetLayout_ != VK_NULL_HANDLE) {
        vkDestroyDescriptorSetLayout(device_, waterSurfacePreprocessDescriptorSetLayout_, nullptr);
    }
    if (gaussianSplatDescriptorSetLayout_ != VK_NULL_HANDLE) {
        vkDestroyDescriptorSetLayout(device_, gaussianSplatDescriptorSetLayout_, nullptr);
    }
    if (highQualityGaussianSplatDescriptorSetLayout_ != VK_NULL_HANDLE) {
        vkDestroyDescriptorSetLayout(device_, highQualityGaussianSplatDescriptorSetLayout_, nullptr);
    }
    if (compositeDescriptorSetLayout_ != VK_NULL_HANDLE) {
        vkDestroyDescriptorSetLayout(device_, compositeDescriptorSetLayout_, nullptr);
    }
    if (postProcessDescriptorSetLayout_ != VK_NULL_HANDLE) {
        vkDestroyDescriptorSetLayout(device_, postProcessDescriptorSetLayout_, nullptr);
    }
    if (eyeDomeLightingExportDescriptorSetLayout_ != VK_NULL_HANDLE) {
        vkDestroyDescriptorSetLayout(device_, eyeDomeLightingExportDescriptorSetLayout_, nullptr);
    }
    if (renderPass_ != VK_NULL_HANDLE) {
        vkDestroyRenderPass(device_, renderPass_, nullptr);
        renderPass_ = VK_NULL_HANDLE;
    }
    if (presentRenderPass_ != VK_NULL_HANDLE) {
        vkDestroyRenderPass(device_, presentRenderPass_, nullptr);
        presentRenderPass_ = VK_NULL_HANDLE;
    }

    if (device_ != VK_NULL_HANDLE) {
        vkDestroyDevice(device_, nullptr);
        device_ = VK_NULL_HANDLE;
    }
    if (surface_ != VK_NULL_HANDLE) {
        vkDestroySurfaceKHR(instance_, surface_, nullptr);
        surface_ = VK_NULL_HANDLE;
    }
    if (instance_ != VK_NULL_HANDLE) {
        vkDestroyInstance(instance_, nullptr);
        instance_ = VK_NULL_HANDLE;
    }
}

void VulkanViewportShell::BeginUiFrame() {
    if (ImGui::GetCurrentContext() == nullptr || uiFrameBegun_) {
        return;
    }

    ImGui_ImplVulkan_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    // GLFW can create a Vulkan surface while macOS monitor enumeration is
    // unavailable (remote/headless smoke sessions are the common case). ImGui
    // asserts if multi-viewport mode remains enabled with no platform monitor;
    // retain the main viewport and degrade gracefully instead.
    if (ImGui::GetPlatformIO().Monitors.empty()) {
        ImGui::GetIO().ConfigFlags &= ~ImGuiConfigFlags_ViewportsEnable;
    }
    // ImGui clears every mouse button when any of its windows reports OS
    // focus loss. macOS system screenshots and multi-viewport focus churn do
    // exactly that mid-drag, releasing slider and window drags the user is
    // still physically holding — Cocoa keeps delivering the eventual mouse
    // release to the originating window, so ignoring focus loss while a
    // button is held is safe and drops no release event.
    ImGui::GetIO().ConfigDebugIgnoreFocusLoss = ImGui::IsAnyMouseDown();
    ImGui::NewFrame();
    uiFrameBegun_ = true;
}

bool VulkanViewportShell::SceneImageNeedsRender(std::uint32_t imageIndex) const {
    if (!liveSceneRenderingEnabled_) {
        return false;
    }
    if (!sceneCachingEnabled_) {
        return true;
    }
    return imageIndex >= sceneImageRevisions_.size() ||
           sceneImageRevisions_[imageIndex] != sceneRevision_;
}

bool VulkanViewportShell::AnySceneImageNeedsRender() const {
    if (!liveSceneRenderingEnabled_) {
        return false;
    }
    if (!sceneCachingEnabled_) {
        return true;
    }
    if (sceneImageRevisions_.size() != swapchainImages_.size()) {
        return true;
    }
    return std::any_of(
        sceneImageRevisions_.begin(),
        sceneImageRevisions_.end(),
        [this](std::uint64_t imageRevision) { return imageRevision != sceneRevision_; });
}

void VulkanViewportShell::DrawFrame() {
    const bool collectDiagnostics = diagnosticsEnabled_;
    if (dynamicMeshFlowTimingPublishedGeneration_ ==
        dynamicMeshFlowTimingResultGeneration_) {
        diagnostics_.gpuDynamicMeshFlow.active = false;
    } else {
        dynamicMeshFlowTimingPublishedGeneration_ =
            dynamicMeshFlowTimingResultGeneration_;
    }
    const auto frameStart = collectDiagnostics ? std::chrono::steady_clock::now()
                                               : std::chrono::steady_clock::time_point{};

    if (uiFrameBegun_) {
        ImGui::Render();
        uiFrameBegun_ = false;
    }
    const auto uiEnd = collectDiagnostics ? std::chrono::steady_clock::now()
                                          : std::chrono::steady_clock::time_point{};

    auto& frame = frameResources_[currentFrameIndex_];
    Check(
        vkWaitForFences(device_, 1, &frame.fence, VK_TRUE, UINT64_MAX),
        "vkWaitForFences(frame slot)");
    ++pointCloudFrameCounter_;
    FlushDeferredPointCloudDestroys(false);
    const auto fenceWaitEnd = collectDiagnostics
                                  ? std::chrono::steady_clock::now()
                                  : std::chrono::steady_clock::time_point{};
    if (frame.activeSubmissionSerial != 0U) {
        frame.completedSubmissionSerial = frame.activeSubmissionSerial;
        ReadCompletedFrameTimestamps(currentFrameIndex_);
        ClearCompletedImageOwners(
            &swapchainImageOwners_,
            static_cast<std::uint32_t>(currentFrameIndex_),
            frame.completedSubmissionSerial);
        frame.activeSubmissionSerial = 0U;
    }
    PollWaterFlowSourceDispatches();
    // This frame slot is no longer referenced by the GPU, so it is safe to
    // publish the newest compact water-effect snapshots without waiting for
    // any other in-flight frame.
    FlushWaterSeepageParamsForFrame(currentFrameIndex_);
    const auto maintenanceEnd = collectDiagnostics
                                    ? std::chrono::steady_clock::now()
                                    : std::chrono::steady_clock::time_point{};
    if (AnySceneImageNeedsRender()) {
        RefreshHighQualityGaussianScene(currentFrameIndex_);
    }
    const auto highQualityPrepareEnd = collectDiagnostics
                                           ? std::chrono::steady_clock::now()
                                           : std::chrono::steady_clock::time_point{};
    if (AnySceneImageNeedsRender()) {
        UpdateUniformBuffer(currentFrameIndex_);
    }
    const auto prepareEnd = collectDiagnostics ? std::chrono::steady_clock::now()
                                               : std::chrono::steady_clock::time_point{};

    diagnostics_.imageOwnerWaited = false;
    diagnostics_.imageWaitImageIndex = 0U;
    diagnostics_.imageWaitOwnerFrameSlot = 0U;
    diagnostics_.imageWaitOwnerSubmissionSerial = 0U;
    std::uint32_t imageIndex = 0;
    const VkResult acquireResult =
        vkAcquireNextImageKHR(
            device_,
            swapchain_,
            UINT64_MAX,
            frame.imageAvailableSemaphore,
            VK_NULL_HANDLE,
            &imageIndex);
    const auto acquireEnd = collectDiagnostics ? std::chrono::steady_clock::now()
                                               : std::chrono::steady_clock::time_point{};

    if (acquireResult == VK_ERROR_OUT_OF_DATE_KHR) {
        UpdateImGuiPlatformWindowsIfNeeded();
        RecreateSwapchain();
        // Exclude the aborted frame plus the swapchain rebuild from the next
        // cadence sample so the always-on FPS readout is not distorted.
        lastFrameCadenceTimestamp_ = std::chrono::steady_clock::now();
        return;
    }
    if (acquireResult != VK_SUCCESS && acquireResult != VK_SUBOPTIMAL_KHR) {
        Check(acquireResult, "vkAcquireNextImageKHR");
    }

    if (imageIndex < swapchainImageOwners_.size()) {
        const auto owner = swapchainImageOwners_[imageIndex];
        if (owner.Valid() && owner.frameSlot < frameResources_.size()) {
            auto& ownerFrame = frameResources_[owner.frameSlot];
            const auto state = ClassifySwapchainImageOwner(
                owner,
                ownerFrame.activeSubmissionSerial,
                ownerFrame.completedSubmissionSerial);
            if (state == SwapchainImageOwnerState::Active) {
                if (collectDiagnostics) {
                    diagnostics_.imageOwnerWaited = true;
                    diagnostics_.imageWaitImageIndex = imageIndex;
                    diagnostics_.imageWaitOwnerFrameSlot =
                        owner.frameSlot;
                    diagnostics_.imageWaitOwnerSubmissionSerial =
                        owner.submissionSerial;
                }
                Check(
                    vkWaitForFences(
                        device_,
                        1,
                        &ownerFrame.fence,
                        VK_TRUE,
                        UINT64_MAX),
                    "vkWaitForFences(swapchain image owner)");
                ownerFrame.completedSubmissionSerial =
                    ownerFrame.activeSubmissionSerial;
                ReadCompletedFrameTimestamps(owner.frameSlot);
                ClearCompletedImageOwners(
                    &swapchainImageOwners_,
                    owner.frameSlot,
                    owner.submissionSerial);
                ownerFrame.activeSubmissionSerial = 0U;
            } else {
                if (state == SwapchainImageOwnerState::Stale) {
                    ++diagnostics_.staleImageOwnerCount;
                }
                swapchainImageOwners_[imageIndex] = {};
            }
        } else if (owner.Valid()) {
            ++diagnostics_.staleImageOwnerCount;
            swapchainImageOwners_[imageIndex] = {};
        }
    }
    const auto imageWaitEnd = collectDiagnostics ? std::chrono::steady_clock::now()
                                                 : std::chrono::steady_clock::time_point{};
    Check(vkResetFences(device_, 1, &frame.fence), "vkResetFences");
    Check(vkResetCommandBuffer(frame.commandBuffer, 0), "vkResetCommandBuffer");
    // Surface descriptor generations can be reclaimed only after MoltenVK's
    // executable command buffer for this slot has been reset, not merely once
    // its fence has signalled.
    PollWaterSurfacePreprocess();
    const auto resetPollEnd = collectDiagnostics
                                  ? std::chrono::steady_clock::now()
                                  : std::chrono::steady_clock::time_point{};
    RecordCommandBuffer(frame.commandBuffer, imageIndex, currentFrameIndex_);
    const auto recordEnd = collectDiagnostics ? std::chrono::steady_clock::now()
                                              : std::chrono::steady_clock::time_point{};

    VkPipelineStageFlags waitStage = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    VkSubmitInfo submitInfo{VK_STRUCTURE_TYPE_SUBMIT_INFO};
    submitInfo.waitSemaphoreCount = 1;
    submitInfo.pWaitSemaphores = &frame.imageAvailableSemaphore;
    submitInfo.pWaitDstStageMask = &waitStage;
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &frame.commandBuffer;
    submitInfo.signalSemaphoreCount = 1;
    VkSemaphore renderFinishedSemaphore =
        imageIndex < swapchainRenderFinishedSemaphores_.size()
            ? swapchainRenderFinishedSemaphores_[imageIndex]
            : VK_NULL_HANDLE;
    if (renderFinishedSemaphore == VK_NULL_HANDLE) {
        throw std::runtime_error{
            "Acquired swapchain image has no render-finished semaphore."};
    }
    submitInfo.pSignalSemaphores = &renderFinishedSemaphore;

    Check(vkQueueSubmit(graphicsQueue_, 1, &submitInfo, frame.fence), "vkQueueSubmit");
    const std::uint64_t submissionSerial = nextSubmissionSerial_++;
    frame.activeSubmissionSerial = submissionSerial;
    frame.submittedImageIndex = imageIndex;
    frame.timestampPending =
        collectDiagnostics && gpuTimestampsSupported_ &&
        frame.timestampQueryPool != VK_NULL_HANDLE;
    if (imageIndex < swapchainImageOwners_.size()) {
        swapchainImageOwners_[imageIndex] = SwapchainImageOwner{
            static_cast<std::uint32_t>(currentFrameIndex_),
            submissionSerial};
    }
    diagnostics_.submittedFrameSlot =
        static_cast<std::uint32_t>(currentFrameIndex_);
    diagnostics_.submittedImageIndex = imageIndex;
    diagnostics_.submittedSubmissionSerial = submissionSerial;
    const auto submitEnd = collectDiagnostics ? std::chrono::steady_clock::now()
                                              : std::chrono::steady_clock::time_point{};

    VkPresentInfoKHR presentInfo{VK_STRUCTURE_TYPE_PRESENT_INFO_KHR};
    presentInfo.waitSemaphoreCount = 1;
    presentInfo.pWaitSemaphores = &renderFinishedSemaphore;
    presentInfo.swapchainCount = 1;
    presentInfo.pSwapchains = &swapchain_;
    presentInfo.pImageIndices = &imageIndex;

    const VkResult presentResult = vkQueuePresentKHR(presentQueue_, &presentInfo);
    const auto presentEnd = collectDiagnostics ? std::chrono::steady_clock::now()
                                               : std::chrono::steady_clock::time_point{};
    if (presentResult == VK_ERROR_OUT_OF_DATE_KHR || presentResult == VK_SUBOPTIMAL_KHR || framebufferResized_) {
        framebufferResized_ = false;
        UpdateImGuiPlatformWindowsIfNeeded();
        RecreateSwapchain();
        // Exclude the swapchain rebuild from the next cadence sample.
        lastFrameCadenceTimestamp_ = std::chrono::steady_clock::now();
        return;
    }

    Check(presentResult, "vkQueuePresentKHR");
    lastSubmittedFrameSlot_ = static_cast<std::uint32_t>(currentFrameIndex_);
    lastSubmittedImageIndex_ = imageIndex;
    lastSubmittedSceneImageValid_ =
        liveSceneRenderingEnabled_ &&
        imageIndex < sceneImageRevisions_.size() &&
        imageIndex < sceneColorImages_.size() &&
        imageIndex < linearDepthImages_.size() &&
        sceneImageRevisions_[imageIndex] == sceneRevision_;
    UpdateImGuiPlatformWindowsIfNeeded();
    const auto frameEnd = collectDiagnostics ? std::chrono::steady_clock::now()
                                             : std::chrono::steady_clock::time_point{};
    if (collectDiagnostics) {
        diagnostics_.framesInFlight = static_cast<std::uint32_t>(kFramesInFlight);
        diagnostics_.swapchainImageCount = static_cast<std::uint32_t>(swapchainImages_.size());
        diagnostics_.currentFrameIndex = static_cast<std::uint32_t>(currentFrameIndex_);
        diagnostics_.frameUiRenderMs = MillisecondsBetween(frameStart, uiEnd);
        diagnostics_.frameFenceWaitMs =
            MillisecondsBetween(uiEnd, fenceWaitEnd);
        diagnostics_.frameCompletedMaintenanceMs =
            MillisecondsBetween(fenceWaitEnd, maintenanceEnd);
        diagnostics_.frameHighQualityGaussianPrepareMs =
            MillisecondsBetween(maintenanceEnd, highQualityPrepareEnd);
        diagnostics_.frameUniformResourceUploadMs =
            MillisecondsBetween(highQualityPrepareEnd, prepareEnd);
        diagnostics_.framePrepareMs =
            MillisecondsBetween(maintenanceEnd, prepareEnd);
        diagnostics_.frameAcquireMs = MillisecondsBetween(prepareEnd, acquireEnd);
        diagnostics_.frameImageWaitMs = MillisecondsBetween(acquireEnd, imageWaitEnd);
        diagnostics_.frameResetRetirePollingMs =
            MillisecondsBetween(imageWaitEnd, resetPollEnd);
        diagnostics_.frameCommandBufferMs =
            MillisecondsBetween(resetPollEnd, recordEnd);
        diagnostics_.frameSubmitMs = MillisecondsBetween(recordEnd, submitEnd);
        diagnostics_.framePresentMs = MillisecondsBetween(submitEnd, presentEnd);
        diagnostics_.framePlatformWindowsMs = MillisecondsBetween(presentEnd, frameEnd);
        diagnostics_.frameRenderMs = MillisecondsBetween(frameStart, frameEnd);
        UpdateCpuTimingAverages();
        if (!diagnosticsTimingInitialized_) {
            diagnostics_.minFrameRenderMs = diagnostics_.frameRenderMs;
            diagnostics_.maxFrameRenderMs = diagnostics_.frameRenderMs;
            diagnosticsTimingInitialized_ = true;
        } else {
            diagnostics_.minFrameRenderMs =
                std::min(diagnostics_.minFrameRenderMs, diagnostics_.frameRenderMs);
            diagnostics_.maxFrameRenderMs =
                std::max(diagnostics_.maxFrameRenderMs, diagnostics_.frameRenderMs);
        }
    }
    // Frame cadence is the wall-clock time between successive DrawFrame
    // calls and is always measured: the Controls-window FPS readout must not
    // depend on the Debug window enabling full diagnostics collection, and
    // cadence — unlike frameRenderMs — includes the app-side work done
    // between frames, so it reflects the frame rate the user actually sees.
    const auto cadenceEnd = std::chrono::steady_clock::now();
    if (frameCadenceInitialized_) {
        constexpr double kFrameAverageWindowMs = 500.0;
        const double wallMs =
            MillisecondsBetween(lastFrameCadenceTimestamp_, cadenceEnd);
        diagnostics_.frameAverageWindowSeconds =
            kFrameAverageWindowMs / 1000.0;
        diagnostics_.frameFps = wallMs > 0.0 ? 1000.0 / wallMs : 0.0;
        diagnosticsFpsWindowMs_ += wallMs;
        ++diagnosticsFpsWindowFrames_;
        if (diagnosticsFpsWindowMs_ >= kFrameAverageWindowMs) {
            diagnostics_.averageFrameRenderMs =
                diagnosticsFpsWindowMs_ /
                static_cast<double>(diagnosticsFpsWindowFrames_);
            diagnostics_.averageFrameFps =
                (1000.0 *
                 static_cast<double>(diagnosticsFpsWindowFrames_)) /
                diagnosticsFpsWindowMs_;
            diagnosticsFpsWindowMs_ = 0.0;
            diagnosticsFpsWindowFrames_ = 0;
        }
    }
    frameCadenceInitialized_ = true;
    lastFrameCadenceTimestamp_ = cadenceEnd;
    currentFrameIndex_ = (currentFrameIndex_ + 1U) % kFramesInFlight;
}

void VulkanViewportShell::WaitIdle() const {
    if (device_ != VK_NULL_HANDLE) {
        vkDeviceWaitIdle(device_);
    }
}

void VulkanViewportShell::SetLiveSceneReadbackCaptureEnabled(bool enabled) {
    if (liveSceneReadbackCaptureEnabled_ == enabled) {
        return;
    }
    liveSceneReadbackCaptureEnabled_ = enabled;
    ++sceneRevision_;
    lastSubmittedSceneImageValid_ = false;
}

LiveSceneReadback VulkanViewportShell::ReadLiveSceneFrame() {
    if (device_ == VK_NULL_HANDLE || commandPool_ == VK_NULL_HANDLE) {
        throw std::runtime_error{"Live scene readback requires an initialized Vulkan device."};
    }
    if (!lastSubmittedSceneImageValid_ ||
        lastSubmittedImageIndex_ >= sceneColorImages_.size() ||
        lastSubmittedImageIndex_ >= linearDepthImages_.size() ||
        lastSubmittedImageIndex_ >= sceneImageRevisions_.size() ||
        sceneImageRevisions_[lastSubmittedImageIndex_] != sceneRevision_) {
        throw std::runtime_error{"No current live scene presentation is available for readback."};
    }
    if (swapchainWidth_ == 0U || swapchainHeight_ == 0U) {
        throw std::runtime_error{"Live scene readback has an empty swapchain extent."};
    }

    const bool bgra =
        swapchainImageFormat_ == VK_FORMAT_B8G8R8A8_SRGB ||
        swapchainImageFormat_ == VK_FORMAT_B8G8R8A8_UNORM;
    const bool rgba =
        swapchainImageFormat_ == VK_FORMAT_R8G8B8A8_SRGB ||
        swapchainImageFormat_ == VK_FORMAT_R8G8B8A8_UNORM;
    if (!bgra && !rgba) {
        throw std::runtime_error{
            "Live scene readback supports only four-channel 8-bit swapchain formats."};
    }
    if (linearDepthFormat_ != VK_FORMAT_R32_SFLOAT) {
        throw std::runtime_error{"Live scene readback requires R32_SFLOAT linear depth."};
    }

    const VkDeviceSize pixelCount =
        static_cast<VkDeviceSize>(swapchainWidth_) *
        static_cast<VkDeviceSize>(swapchainHeight_);
    constexpr VkDeviceSize kBytesPerPixel = 4U;
    if (pixelCount > std::numeric_limits<VkDeviceSize>::max() / (2U * kBytesPerPixel)) {
        throw std::runtime_error{"Live scene readback extent exceeds the addressable buffer size."};
    }
    const VkDeviceSize colorBytes = pixelCount * kBytesPerPixel;
    const VkDeviceSize depthBytes = pixelCount * sizeof(float);
    const VkDeviceSize requiredBytes = colorBytes + depthBytes;

    Check(vkDeviceWaitIdle(device_), "vkDeviceWaitIdle(live scene readback)");
    if (liveSceneReadbackBuffer_.buffer == VK_NULL_HANDLE ||
        liveSceneReadbackBuffer_.size < requiredBytes) {
        auto replacement = CreateHostVisibleBuffer(
            requiredBytes,
            VK_BUFFER_USAGE_TRANSFER_DST_BIT);
        DestroyBuffer(&liveSceneReadbackBuffer_);
        liveSceneReadbackBuffer_ = replacement;
    }

    VkCommandBuffer commandBuffer = VK_NULL_HANDLE;
    VkCommandBufferAllocateInfo allocationInfo{
        VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
    allocationInfo.commandPool = commandPool_;
    allocationInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    allocationInfo.commandBufferCount = 1U;
    Check(
        vkAllocateCommandBuffers(device_, &allocationInfo, &commandBuffer),
        "vkAllocateCommandBuffers(live scene readback)");

    bool submitted = false;
    bool submissionCompleted = false;
    try {
        VkCommandBufferBeginInfo beginInfo{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
        beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
        Check(
            vkBeginCommandBuffer(commandBuffer, &beginInfo),
            "vkBeginCommandBuffer(live scene readback)");

        std::array<VkImageMemoryBarrier, 2> copyBarriers{};
        for (auto& barrier : copyBarriers) {
            barrier = {VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER};
            // The scene colour was sampled by the present pass, while linear
            // depth may have last been either written or sampled depending on
            // post-process settings. Cover both settled access histories.
            barrier.srcAccessMask =
                VK_ACCESS_MEMORY_READ_BIT | VK_ACCESS_MEMORY_WRITE_BIT;
            barrier.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
            barrier.oldLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
            barrier.newLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
            barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
            barrier.subresourceRange.baseMipLevel = 0U;
            barrier.subresourceRange.levelCount = 1U;
            barrier.subresourceRange.baseArrayLayer = 0U;
            barrier.subresourceRange.layerCount = 1U;
        }
        copyBarriers[0].image = sceneColorImages_[lastSubmittedImageIndex_].image;
        copyBarriers[1].image = linearDepthImages_[lastSubmittedImageIndex_].image;
        vkCmdPipelineBarrier(
            commandBuffer,
            VK_PIPELINE_STAGE_ALL_COMMANDS_BIT,
            VK_PIPELINE_STAGE_TRANSFER_BIT,
            0U,
            0U,
            nullptr,
            0U,
            nullptr,
            static_cast<std::uint32_t>(copyBarriers.size()),
            copyBarriers.data());

        VkBufferImageCopy colorCopy{};
        colorCopy.bufferOffset = 0U;
        colorCopy.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        colorCopy.imageSubresource.mipLevel = 0U;
        colorCopy.imageSubresource.baseArrayLayer = 0U;
        colorCopy.imageSubresource.layerCount = 1U;
        colorCopy.imageExtent = {swapchainWidth_, swapchainHeight_, 1U};
        vkCmdCopyImageToBuffer(
            commandBuffer,
            sceneColorImages_[lastSubmittedImageIndex_].image,
            VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
            liveSceneReadbackBuffer_.buffer,
            1U,
            &colorCopy);

        VkBufferImageCopy depthCopy = colorCopy;
        depthCopy.bufferOffset = colorBytes;
        vkCmdCopyImageToBuffer(
            commandBuffer,
            linearDepthImages_[lastSubmittedImageIndex_].image,
            VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
            liveSceneReadbackBuffer_.buffer,
            1U,
            &depthCopy);

        std::array<VkImageMemoryBarrier, 2> restoreBarriers = copyBarriers;
        for (auto& barrier : restoreBarriers) {
            barrier.srcAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
            barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
            barrier.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
            barrier.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        }
        VkBufferMemoryBarrier hostReadBarrier{VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER};
        hostReadBarrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
        hostReadBarrier.dstAccessMask = VK_ACCESS_HOST_READ_BIT;
        hostReadBarrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        hostReadBarrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        hostReadBarrier.buffer = liveSceneReadbackBuffer_.buffer;
        hostReadBarrier.offset = 0U;
        hostReadBarrier.size = requiredBytes;
        vkCmdPipelineBarrier(
            commandBuffer,
            VK_PIPELINE_STAGE_TRANSFER_BIT,
            VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT | VK_PIPELINE_STAGE_HOST_BIT,
            0U,
            0U,
            nullptr,
            1U,
            &hostReadBarrier,
            static_cast<std::uint32_t>(restoreBarriers.size()),
            restoreBarriers.data());

        Check(
            vkEndCommandBuffer(commandBuffer),
            "vkEndCommandBuffer(live scene readback)");
        VkSubmitInfo submitInfo{VK_STRUCTURE_TYPE_SUBMIT_INFO};
        submitInfo.commandBufferCount = 1U;
        submitInfo.pCommandBuffers = &commandBuffer;
        Check(
            vkQueueSubmit(graphicsQueue_, 1U, &submitInfo, VK_NULL_HANDLE),
            "vkQueueSubmit(live scene readback)");
        submitted = true;
        Check(
            vkQueueWaitIdle(graphicsQueue_),
            "vkQueueWaitIdle(live scene readback)");
        submissionCompleted = true;
    } catch (...) {
        if (commandBuffer != VK_NULL_HANDLE && (!submitted || submissionCompleted)) {
            vkFreeCommandBuffers(device_, commandPool_, 1U, &commandBuffer);
        }
        throw;
    }
    vkFreeCommandBuffers(device_, commandPool_, 1U, &commandBuffer);

    LiveSceneReadback readback{
        .width = swapchainWidth_,
        .height = swapchainHeight_,
        .submittedFrameSlot = lastSubmittedFrameSlot_,
        .swapchainImageIndex = lastSubmittedImageIndex_,
    };
    const auto* bytes = static_cast<const std::uint8_t*>(liveSceneReadbackBuffer_.mapped);
    readback.colorRgba8.resize(static_cast<std::size_t>(colorBytes));
    if (rgba) {
        std::memcpy(
            readback.colorRgba8.data(),
            bytes,
            static_cast<std::size_t>(colorBytes));
    } else {
        for (std::size_t offset = 0U;
             offset < static_cast<std::size_t>(colorBytes);
             offset += 4U) {
            readback.colorRgba8[offset] = bytes[offset + 2U];
            readback.colorRgba8[offset + 1U] = bytes[offset + 1U];
            readback.colorRgba8[offset + 2U] = bytes[offset];
            readback.colorRgba8[offset + 3U] = bytes[offset + 3U];
        }
    }
    readback.linearDepth.resize(static_cast<std::size_t>(pixelCount));
    std::memcpy(
        readback.linearDepth.data(),
        bytes + static_cast<std::size_t>(colorBytes),
        static_cast<std::size_t>(depthBytes));
    return readback;
}

void VulkanViewportShell::SetDiagnosticsEnabled(bool enabled) {
    if (diagnosticsEnabled_ == enabled) {
        return;
    }
    diagnosticsEnabled_ = enabled;
    if (enabled) {
        diagnosticsTimingInitialized_ = false;
        // The FPS/cadence fields are always-on and deliberately survive this
        // reset — opening the Debug window must not blank the persistent
        // frame-rate readout.
        diagnostics_.frameRenderMs = 0.0;
        diagnostics_.minFrameRenderMs = 0.0;
        diagnostics_.maxFrameRenderMs = 0.0;
        diagnostics_.frameUiRenderMs = 0.0;
        diagnostics_.averageFrameUiRenderMs = 0.0;
        diagnostics_.frameFenceWaitMs = 0.0;
        diagnostics_.averageFrameFenceWaitMs = 0.0;
        diagnostics_.frameCompletedMaintenanceMs = 0.0;
        diagnostics_.averageFrameCompletedMaintenanceMs = 0.0;
        diagnostics_.frameAcquireMs = 0.0;
        diagnostics_.averageFrameAcquireMs = 0.0;
        diagnostics_.frameImageWaitMs = 0.0;
        diagnostics_.averageFrameImageWaitMs = 0.0;
        diagnostics_.framePrepareMs = 0.0;
        diagnostics_.frameHighQualityGaussianPrepareMs = 0.0;
        diagnostics_.averageFrameHighQualityGaussianPrepareMs = 0.0;
        diagnostics_.frameUniformResourceUploadMs = 0.0;
        diagnostics_.averageFrameUniformResourceUploadMs = 0.0;
        diagnostics_.frameResetRetirePollingMs = 0.0;
        diagnostics_.averageFrameResetRetirePollingMs = 0.0;
        diagnostics_.frameCommandBufferMs = 0.0;
        diagnostics_.averageFrameCommandBufferMs = 0.0;
        diagnostics_.frameSubmitMs = 0.0;
        diagnostics_.averageFrameSubmitMs = 0.0;
        diagnostics_.framePresentMs = 0.0;
        diagnostics_.averageFramePresentMs = 0.0;
        diagnostics_.framePlatformWindowsMs = 0.0;
        diagnostics_.averageFramePlatformWindowsMs = 0.0;
        diagnostics_.gpuTimestampResultsAvailable = false;
        diagnostics_.gpuTimestampUnavailableResultCount = 0U;
        diagnostics_.gpuTotal = {};
        diagnostics_.gpuRainCompute = {};
        diagnostics_.gpuDepth = {};
        diagnostics_.gpuWeightedAccumulation = {};
        diagnostics_.gpuComposite = {};
        diagnostics_.gpuOpaqueAndHighQuality = {};
        diagnostics_.gpuPostProcess = {};
        diagnostics_.gpuImGui = {};
        diagnostics_.gpuDynamicMeshFlow = {};
        cpuTimingWindow_.Reset();
        gpuTimingWindow_.Reset();
        dynamicMeshFlowTimingWindow_.Reset();
        lastGpuTimingSampleAt_ = {};
        lastDynamicMeshFlowTimingSampleAt_ = {};
    }
}

void VulkanViewportShell::SetSceneCachingEnabled(bool enabled) {
    if (sceneCachingEnabled_ == enabled) {
        return;
    }
    sceneCachingEnabled_ = enabled;
    if (!sceneCachingEnabled_) {
        ++sceneRevision_;
    }
}

void VulkanViewportShell::SetTemporalCameraOverlay(
    bool enabled,
    std::uint32_t renderedSourceIndex,
    float firstWeight,
    float secondWeight,
    const std::array<glm::mat4, 2U>* currentSourceViewProjections,
    TemporalCameraCompositionMode compositionMode,
    float splitCenterNormalized,
    bool firstSourcePrimary) {
    if (enabled &&
        (temporalCameraOverlayColorImages_[0U].image == VK_NULL_HANDLE ||
         temporalCameraOverlayDepthImages_[0U].image == VK_NULL_HANDLE)) {
        // The optional history costs four viewport-sized images. Allocate it
        // only on first use, after every descriptor that may still reference
        // the fallback scene images has finished.
        Check(
            vkDeviceWaitIdle(device_),
            "vkDeviceWaitIdle(temporal camera overlay allocation)");
        CreateTemporalCameraOverlayResources();
        CreateOrUpdatePostProcessDescriptorSets();
    }
    const std::uint32_t sourceIndex = std::min(renderedSourceIndex, 1U);
    const bool modeChanged = temporalCameraOverlayEnabled_ != enabled;
    const bool sourceChanged =
        enabled &&
        temporalCameraOverlayRenderedSourceIndex_ != sourceIndex;
    const bool primarySourceChanged =
        enabled &&
        temporalCameraOverlayFirstSourcePrimary_ != firstSourcePrimary;
    const bool layoutChanged =
        enabled &&
        (temporalCameraOverlayCompositionMode_ != compositionMode ||
         std::abs(temporalCameraOverlaySplitCenterNormalized_ -
                  splitCenterNormalized) > 1.0e-5F ||
         primarySourceChanged);
    if (modeChanged || primarySourceChanged) {
        // A current/partner role flip changes what history slots 0 and 1
        // mean. Clearing both avoids displaying one or two stale frames with
        // the newly selected left/top ordering.
        temporalCameraOverlayHistoryValid_ = {false, false};
    }
    temporalCameraOverlayEnabled_ = enabled;
    temporalCameraOverlayRenderedSourceIndex_ = sourceIndex;
    temporalCameraOverlayWeights_[0U] =
        std::clamp(firstWeight, 0.0F, 1.0F);
    temporalCameraOverlayWeights_[1U] =
        std::clamp(secondWeight, 0.0F, 1.0F);
    temporalCameraOverlayCompositionMode_ = compositionMode;
    temporalCameraOverlaySplitCenterNormalized_ = std::clamp(
        splitCenterNormalized,
        -2.0F,
        3.0F);
    temporalCameraOverlayFirstSourcePrimary_ = firstSourcePrimary;
    temporalCameraOverlayCurrentViewProjectionsValid_ =
        enabled && currentSourceViewProjections != nullptr;
    if (temporalCameraOverlayCurrentViewProjectionsValid_) {
        temporalCameraOverlayCurrentViewProjections_ =
            *currentSourceViewProjections;
    }
    if (!enabled) {
        temporalCameraOverlayHistoryValid_ = {false, false};
    }
    if (modeChanged || sourceChanged || layoutChanged) {
        ++sceneRevision_;
        lastSubmittedSceneImageValid_ = false;
    }
}

void VulkanViewportShell::ResetRainRuntime() {
    ++rainResources_.resetEpoch;
    if (rainResources_.resetEpoch == 0U) {
        rainResources_.resetEpoch = 1U;
    }
    rainResources_.previousTimeSeconds =
        -std::numeric_limits<float>::infinity();
    rainResources_.previousRainEnabled = false;
    rainResources_.lastPotentialImpactTimeSeconds =
        -std::numeric_limits<float>::infinity();
    rainResources_.lastInFlightParticleCount = 0U;
    rainResources_.frameDeltaSeconds = 1.0F / 30.0F;
    ++sceneRevision_;
}

void VulkanViewportShell::SetLiveRainSimulationEnabled(bool enabled) {
    if (liveRainSimulationEnabled_ == enabled) {
        return;
    }
    liveRainSimulationEnabled_ = enabled;
    if (enabled) {
        // The export owned the shared buffers while live Rain was suspended.
        // Begin the resumed preview from its own deterministic initial state.
        ResetRainRuntime();
    } else {
        ++sceneRevision_;
    }
}

void VulkanViewportShell::UpdateRainRuntimeTiming(const SceneRenderState& state) {
    const bool timeMovedBack =
        std::isfinite(rainResources_.previousTimeSeconds) &&
        state.flowTimeSeconds < rainResources_.previousTimeSeconds;
    if (std::isfinite(rainResources_.previousTimeSeconds) &&
        state.flowTimeSeconds >= rainResources_.previousTimeSeconds) {
        rainResources_.frameDeltaSeconds = std::clamp(
            state.flowTimeSeconds - rainResources_.previousTimeSeconds,
            1.0F / 240.0F,
            0.25F);
    } else {
        rainResources_.frameDeltaSeconds = 1.0F / 30.0F;
    }
    if ((!rainResources_.previousRainEnabled && state.rainSettings.enabled) ||
        state.rainSettings.seed != rainResources_.lastSeed ||
        timeMovedBack) {
        ++rainResources_.resetEpoch;
        if (rainResources_.resetEpoch == 0U) {
            rainResources_.resetEpoch = 1U;
        }
    }
    if (!state.rainSettings.enabled || timeMovedBack) {
        rainResources_.lastPotentialImpactTimeSeconds =
            -std::numeric_limits<float>::infinity();
        rainResources_.lastInFlightParticleCount = 0U;
    } else if (
        rainResources_.collisionReady &&
        state.rainSettings.impactEffectsEnabled &&
        (state.rainSettings.sandEffectsEnabled ||
         state.rainSettings.rockEffectsEnabled ||
         state.rainSettings.vegetationEffectsEnabled) &&
        state.rainSettings.rainLevel > 1.0e-5F &&
        std::isfinite(state.flowTimeSeconds)) {
        // A positive level may emit an impact this frame. At level zero the
        // timestamp intentionally remains unchanged so existing GPU events
        // receive their full natural lifetime without permitting new drops.
        rainResources_.lastPotentialImpactTimeSeconds =
            state.flowTimeSeconds;
    }
    rainResources_.lastSeed = state.rainSettings.seed;
    rainResources_.previousTimeSeconds = state.flowTimeSeconds;
    rainResources_.previousRainEnabled = state.rainSettings.enabled;
}

bool VulkanViewportShell::RainImpactEffectsRequireRedraw(
    float currentTimeSeconds) const {
    return rainResources_.lastInFlightParticleCount > 0U ||
           invisible_places::water::RainImpactTailIsActive(
               rainResources_.lastPotentialImpactTimeSeconds,
               currentTimeSeconds);
}

void VulkanViewportShell::UpdateRenderState(const SceneRenderState& state) {
    if (liveRainSimulationEnabled_) {
        UpdateRainRuntimeTiming(state);
    }
    renderState_ = state;
    // A keyed Rain track keeps rainSettings.enabled true even at level zero so
    // the rain clock and tails stay continuous. Every visible rain artefact
    // (rings, wetness, droplets) ages out within kRainMaximumImpactLifetimeSeconds
    // and airborne drops re-anchor the tail when they land, so once the level
    // is zero and the tail has drained there is provably nothing rain-related
    // to shade and the layers can keep their cheaper material variants.
    const bool rainCanShadeThisFrame =
        renderState_.rainSettings.rainLevel > 1.0e-5F ||
        RainImpactEffectsRequireRedraw(renderState_.flowTimeSeconds);
    // The impact models shade any layer's points inside their world-Z height
    // bands, independent of the layer's collision role — but a layer whose Z
    // extent no enabled band can reach evaluates every model to exactly zero,
    // so it may keep its cheaper (early-z) material variant. The margin
    // covers surface-motion vertical offsets on top of each band's fade
    // feather.
    constexpr float kRainImpactLayerBoundsMarginMeters = 1.0F;
    for (auto& layer : renderState_.pointCloudLayers) {
        const bool impactCanReachLayer =
            !layer.worldZBoundsValid ||
            invisible_places::water::RainImpactEffectsCanReachZRange(
                renderState_.rainSettings,
                layer.worldMinZ,
                layer.worldMaxZ,
                kRainImpactLayerBoundsMarginMeters);
        layer.style.rainImpactEffects =
            liveRainSimulationEnabled_ &&
            renderState_.rainSettings.enabled &&
            renderState_.rainSettings.impactEffectsEnabled &&
            rainCanShadeThisFrame &&
            impactCanReachLayer &&
            // A rain feature toggled off the water sheet leaves the
            // gap-fill layer dry while every scanned cloud keeps shading.
            (!layer.waterFillLayer || renderState_.rainAppliesToWaterFill);
    }
    diagnostics_.previewPerformanceModeEnabled =
        renderState_.previewPerformanceMode;
    diagnostics_.previewPerformanceEligibleLayerCount =
        static_cast<std::uint32_t>(std::count_if(
            renderState_.pointCloudLayers.begin(),
            renderState_.pointCloudLayers.end(),
            [&](const auto& layer) {
                return !PointCloudLayerUsesFastBasicRenderer(
                           renderState_.pointCloudRendererMode,
                           layer) &&
                       LayerUsesDepthTestedAccumulation(
                    layer,
                    ResolvePointCloudLayerMaterialVariant(layer));
            }));
    ++sceneRevision_;

    std::uint64_t pointCount = 0;
    double pointSizeSum = 0.0;
    std::uint64_t pointSizeWeight = 0;
    for (const auto& layer : renderState_.pointCloudLayers) {
        const auto densityCompensation =
            renderer::pointcloud::SanitizePointCloudDensityCompensation(layer.densityCompensation);
        pointCount += layer.drawPointCount;
        pointSizeSum +=
            static_cast<double>(
                layer.style.pointSize.constantValue[0] *
                renderState_.pointSizeScale *
                densityCompensation.footprintScale) *
            static_cast<double>(std::max<std::uint32_t>(1U, layer.drawPointCount));
        pointSizeWeight += std::max<std::uint32_t>(1U, layer.drawPointCount);
    }

    diagnostics_.pointCount = pointCount;
    if (pointCount == 0) {
        diagnostics_.pointSubmittedCount = 0;
        diagnostics_.pointPassSubmittedCount = 0;
    }
    diagnostics_.averagePointSizePx =
        pointSizeWeight > 0 ? static_cast<float>(pointSizeSum / static_cast<double>(pointSizeWeight)) : 0.0F;
    diagnostics_.accumulationWidth = swapchainWidth_;
    diagnostics_.accumulationHeight = swapchainHeight_;
    diagnostics_.rainActiveParticleCount = ActiveRainParticleCount(renderState_);
    const bool hasFastBasicLayers = std::any_of(
        renderState_.pointCloudLayers.begin(),
        renderState_.pointCloudLayers.end(),
        [&](const auto& layer) {
            return PointCloudLayerUsesFastBasicRenderer(
                renderState_.pointCloudRendererMode,
                layer);
        });
    const bool hasBeautyLayers = std::any_of(
        renderState_.pointCloudLayers.begin(),
        renderState_.pointCloudLayers.end(),
        [&](const auto& layer) {
            return !PointCloudLayerUsesFastBasicRenderer(
                renderState_.pointCloudRendererMode,
                layer);
        });
    diagnostics_.pointRenderModes = pointCount == 0
                                        ? ""
                                        : (hasFastBasicLayers && hasBeautyLayers
                                               ? "fast-basic-square + beauty-overlay"
                                               : (hasFastBasicLayers
                                                      ? "fast-basic-square"
                                                      : "beauty-material"));

    std::ostringstream summary;
    summary << "Renderer: " << diagnostics_.rendererName << " | " << swapchainWidth_ << "x"
            << swapchainHeight_ << " | mixed scene Vulkan viewport";
    if (pointCount > 0) {
        summary << " | points: " << pointCount << " | point px avg: " << diagnostics_.averagePointSizePx
                << " | accumulation: " << diagnostics_.accumulationWidth << "x" << diagnostics_.accumulationHeight
                << " | point material: " << diagnostics_.pointRenderModes;
    }
    diagnostics_.summary = summary.str();
}

void VulkanViewportShell::BeginPointCloudMutationBatch(
    bool requiresQuiescentGpu) {
    if (pointCloudMutationBatchDepth_ == 0U) {
        pointCloudMutationBatchWaitCount_ = 0U;
        pointCloudMutationBatchPeakBytes_ = PointCloudResidentBytes();
        pointCloudMutationBatchQuiesced_ = false;
        if (requiresQuiescentGpu) {
            WaitIdle();
            ResetPointCloudReferencingCommandBuffers();
            pointCloudMutationBatchQuiesced_ = true;
            ++pointCloudMutationBatchWaitCount_;
        }
    } else if (requiresQuiescentGpu &&
               !pointCloudMutationBatchQuiesced_) {
        // A nested batch that needs the settled device upgrades the outer
        // creation-only batch exactly once.
        SettlePointCloudMutation();
    }
    ++pointCloudMutationBatchDepth_;
}

void VulkanViewportShell::EndPointCloudMutationBatch() {
    if (pointCloudMutationBatchDepth_ == 0U) {
        throw std::logic_error{"Point-cloud mutation batch end has no matching begin."};
    }
    --pointCloudMutationBatchDepth_;
    if (pointCloudMutationBatchDepth_ != 0U) {
        return;
    }
    TrackPointCloudResidentPeak();
    lastPointCloudMutationBatchPeakBytes_ = pointCloudMutationBatchPeakBytes_;
    lastPointCloudMutationBatchWaitCount_ = pointCloudMutationBatchWaitCount_;
    pointCloudMutationBatchWaitCount_ = 0U;
}

void VulkanViewportShell::SetSmokeFailNextPointCloudUploadAfterPartialAllocation(bool enabled) {
    smokeFailNextPointCloudUploadAfterPartialAllocation_ = enabled;
}

void VulkanViewportShell::SetSmokeFailNextWaterFlowUploadAfterPartialAllocation(bool enabled) {
    smokeFailNextWaterFlowUploadAfterPartialAllocation_ = enabled;
}

bool VulkanViewportShell::HasPointCloudResources(std::size_t layerId) const {
    return FindPointCloudResources(layerId) != nullptr;
}

std::uint64_t VulkanViewportShell::PointCloudResourceResidentBytes(
    const ActivePointCloudResources& resources) const {
    const auto bufferBytes = [](const BufferAllocation& buffer) -> std::uint64_t {
        return buffer.buffer != VK_NULL_HANDLE ? static_cast<std::uint64_t>(buffer.size) : 0U;
    };
    std::uint64_t bytes = 0U;
    const auto add = [&bytes, &bufferBytes](const BufferAllocation& buffer) {
        const auto size = bufferBytes(buffer);
        bytes = size > std::numeric_limits<std::uint64_t>::max() - bytes
                    ? std::numeric_limits<std::uint64_t>::max()
                    : bytes + size;
    };
    add(resources.positionBuffer);
    add(resources.positionStorageBuffer);
    add(resources.colorBuffer);
    add(resources.normalBuffer);
    add(resources.scalarFieldBuffer);
    add(resources.sampledIndexBuffer);
    add(resources.sampledSurfelIndexBuffer);
    add(resources.interactiveSampledIndexBuffer);
    add(resources.interactiveSurfelIndexBuffer);
    add(resources.depthSortedIndexBuffer);
    add(resources.depthSortScratchBuffer);
    add(resources.dynamicMeshFlowParticleBuffer);
    add(resources.dynamicMeshFlowContactEventBuffer);
    add(resources.dynamicMeshFlowContactGridBuffer);
    add(resources.seepageNodeBuffer);
    for (const auto& paramsBuffer : resources.seepageParamsBuffers) {
        add(paramsBuffer);
    }
    add(resources.seepageExrParamsBuffer);
    add(resources.seepageHashCellBuffer);
    add(resources.seepageNodeReferenceBuffer);
    add(resources.waterFlowPendingPositionBuffer);
    add(resources.waterFlowPendingPositionStorageBuffer);
    add(resources.waterFlowPendingColorBuffer);
    add(resources.waterFlowPendingNormalBuffer);
    add(resources.waterFlowPendingScalarFieldBuffer);
    for (const auto& retired : resources.waterFlowRetiredOutputs) {
        add(retired.positionBuffer);
        add(retired.positionStorageBuffer);
        add(retired.colorBuffer);
        add(retired.normalBuffer);
        add(retired.scalarFieldBuffer);
    }
    return bytes;
}

std::uint64_t VulkanViewportShell::PointCloudLayerResidentBytes(std::size_t layerId) const {
    const auto* resources = FindPointCloudResources(layerId);
    return resources != nullptr ? PointCloudResourceResidentBytes(*resources) : 0U;
}

std::uint64_t VulkanViewportShell::PointCloudResidentBytes() const {
    std::uint64_t bytes = 0U;
    for (const auto& resources : pointCloudResources_) {
        const auto layerBytes = PointCloudResourceResidentBytes(resources);
        bytes = layerBytes > std::numeric_limits<std::uint64_t>::max() - bytes
                    ? std::numeric_limits<std::uint64_t>::max()
                    : bytes + layerBytes;
    }
    return bytes;
}

std::uint64_t VulkanViewportShell::PointCloudPeakResidentBytes() const {
    return pointCloudResidentPeakBytes_;
}

std::uint64_t VulkanViewportShell::LastPointCloudMutationBatchPeakResidentBytes() const {
    return lastPointCloudMutationBatchPeakBytes_;
}

std::uint32_t VulkanViewportShell::LastPointCloudMutationBatchWaitCount() const {
    return lastPointCloudMutationBatchWaitCount_;
}

void VulkanViewportShell::TrackPointCloudResidentPeak(std::uint64_t unpublishedBytes) {
    const auto residentBytes = PointCloudResidentBytes();
    const auto combinedBytes =
        unpublishedBytes > std::numeric_limits<std::uint64_t>::max() - residentBytes
            ? std::numeric_limits<std::uint64_t>::max()
            : residentBytes + unpublishedBytes;
    pointCloudResidentPeakBytes_ = std::max(pointCloudResidentPeakBytes_, combinedBytes);
    if (pointCloudMutationBatchDepth_ > 0U) {
        pointCloudMutationBatchPeakBytes_ =
            std::max(pointCloudMutationBatchPeakBytes_, combinedBytes);
    }
}

void VulkanViewportShell::SettlePointCloudMutation() {
    if (pointCloudMutationBatchDepth_ > 0U) {
        if (pointCloudMutationBatchQuiesced_) {
            return;
        }
        // A batch opened without requiring a quiescent device settles
        // lazily: the first mutation that destroys or replaces live
        // resources pays the one drain, while pure creations before it
        // (new-layer uploads are host-visible buffer fills) overlap
        // whatever the GPU still has in flight.
        WaitIdle();
        ResetPointCloudReferencingCommandBuffers();
        pointCloudMutationBatchQuiesced_ = true;
        ++pointCloudMutationBatchWaitCount_;
        return;
    }
    WaitIdle();
    ResetPointCloudReferencingCommandBuffers();
}

VulkanViewportShell::ActivePointCloudResources
VulkanViewportShell::BuildPointCloudUploadResources(
    const invisible_places::io::LoadedPointCloud& cloud,
    const std::vector<std::uint32_t>& sampledIndices,
    bool onRenderThread) {
    ActivePointCloudResources resources;
        resources.pointCount = static_cast<std::uint32_t>(cloud.PointCount());
    resources.activePointCount = resources.pointCount;
    resources.scalarFieldCount = static_cast<std::uint32_t>(cloud.ScalarFieldCount());
    resources.hasSourceRgb = cloud.hasSourceRgb;
    resources.hasNormals = cloud.hasNormals && cloud.normals.size() == cloud.positions.size();
    resources.worldBoundsValid = cloud.bounds.valid;
    resources.worldBoundsMinZ = cloud.bounds.minimum.z;
    resources.worldBoundsMaxZ = cloud.bounds.maximum.z;

    try {
        resources.positionBuffer = CreateHostVisibleBuffer(
            static_cast<VkDeviceSize>(cloud.positions.size() * sizeof(invisible_places::io::Float3)),
            VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
        UploadBufferData(resources.positionBuffer, cloud.positions.data(), resources.positionBuffer.size);
        if (onRenderThread) {
            TrackPointCloudResidentPeak(PointCloudResourceResidentBytes(resources));
        }

        resources.positionStorageBuffer = CreateHostVisibleBuffer(
            static_cast<VkDeviceSize>(cloud.positions.size() * sizeof(glm::vec4)), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        auto* storagePositions = static_cast<glm::vec4*>(resources.positionStorageBuffer.mapped);
        if (storagePositions == nullptr) {
            throw std::runtime_error{"Point-cloud position storage buffer is not host visible."};
        }
        for (std::size_t pointIndex = 0; pointIndex < cloud.positions.size(); ++pointIndex) {
            const auto& position = cloud.positions[pointIndex];
            storagePositions[pointIndex] = glm::vec4{position.x, position.y, position.z, 1.0F};
        }
        if (onRenderThread) {
            TrackPointCloudResidentPeak(PointCloudResourceResidentBytes(resources));
        }
        if (onRenderThread &&
            smokeFailNextPointCloudUploadAfterPartialAllocation_) {
            smokeFailNextPointCloudUploadAfterPartialAllocation_ = false;
            throw std::runtime_error{"Injected point-cloud upload failure after partial allocation."};
        }

        resources.colorBuffer =
            CreateHostVisibleBuffer(static_cast<VkDeviceSize>(cloud.packedColors.size() * sizeof(std::uint32_t)),
                                    VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        UploadBufferData(resources.colorBuffer, cloud.packedColors.data(), resources.colorBuffer.size);
        if (onRenderThread) {
            TrackPointCloudResidentPeak(PointCloudResourceResidentBytes(resources));
        }

        if (resources.hasNormals) {
            resources.normalBuffer =
                CreateHostVisibleBuffer(static_cast<VkDeviceSize>(cloud.normals.size() * sizeof(glm::vec4)),
                                        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
            auto* storageNormals = static_cast<glm::vec4*>(resources.normalBuffer.mapped);
            if (storageNormals == nullptr) {
                throw std::runtime_error{"Point-cloud normal storage buffer is not host visible."};
            }
            for (std::size_t pointIndex = 0; pointIndex < cloud.normals.size(); ++pointIndex) {
                const auto& normal = cloud.normals[pointIndex];
                storageNormals[pointIndex] = glm::vec4{normal.x, normal.y, normal.z, 0.0F};
            }
        } else {
            const glm::vec4 fallbackNormal{0.0F, 0.0F, 0.0F, 0.0F};
            resources.normalBuffer =
                CreateHostVisibleBuffer(sizeof(fallbackNormal), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
            UploadBufferData(resources.normalBuffer, &fallbackNormal, sizeof(fallbackNormal));
        }
        if (onRenderThread) {
            TrackPointCloudResidentPeak(PointCloudResourceResidentBytes(resources));
        }

        if (!cloud.scalarFieldValues.empty()) {
            resources.scalarFieldBuffer =
                CreateHostVisibleBuffer(static_cast<VkDeviceSize>(cloud.scalarFieldValues.size() * sizeof(float)),
                                        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
            UploadBufferData(resources.scalarFieldBuffer, cloud.scalarFieldValues.data(),
                             resources.scalarFieldBuffer.size);
        } else {
            const float fallbackScalar = 0.0F;
            resources.scalarFieldBuffer = CreateHostVisibleBuffer(sizeof(float), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
            UploadBufferData(resources.scalarFieldBuffer, &fallbackScalar, sizeof(float));
        }
        if (onRenderThread) {
            TrackPointCloudResidentPeak(PointCloudResourceResidentBytes(resources));
        }
        const WaterSeepageNodeTopologyGpu emptySeepageNode{};
        resources.seepageNodeBuffer =
            CreateHostVisibleBuffer(sizeof(emptySeepageNode), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        UploadBufferData(resources.seepageNodeBuffer, &emptySeepageNode, sizeof(emptySeepageNode));
        const WaterSeepageNodeParamsGpu emptySeepageParams{};
        for (auto& paramsBuffer : resources.seepageParamsBuffers) {
            paramsBuffer = CreateHostVisibleBuffer(sizeof(emptySeepageParams), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
            UploadBufferData(paramsBuffer, &emptySeepageParams, sizeof(emptySeepageParams));
        }
        resources.seepageExrParamsBuffer =
            CreateHostVisibleBuffer(sizeof(emptySeepageParams), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        UploadBufferData(resources.seepageExrParamsBuffer, &emptySeepageParams, sizeof(emptySeepageParams));
        resources.pendingSeepageParams.resize(sizeof(emptySeepageParams));
        std::memcpy(resources.pendingSeepageParams.data(), &emptySeepageParams, sizeof(emptySeepageParams));
        const WaterSeepageHashCellGpu emptySeepageHashCell{};
        resources.seepageHashCellBuffer =
            CreateHostVisibleBuffer(sizeof(emptySeepageHashCell), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        UploadBufferData(resources.seepageHashCellBuffer, &emptySeepageHashCell, sizeof(emptySeepageHashCell));
        const invisible_places::water::WaterSeepageSupportReference
            emptySeepageNodeReference{};
        resources.seepageNodeReferenceBuffer =
            CreateHostVisibleBuffer(sizeof(emptySeepageNodeReference), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        UploadBufferData(resources.seepageNodeReferenceBuffer, &emptySeepageNodeReference,
                         sizeof(emptySeepageNodeReference));

        for (auto& styleBuffer : resources.styleBuffers) {
            styleBuffer = CreateHostVisibleBuffer(sizeof(PointCloudStyleGpu), VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT);
        }
        resources.exrStyleBuffer =
            CreateHostVisibleBuffer(sizeof(PointCloudStyleGpu), VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT);

        const auto sanitized = SanitizePointIndices(sampledIndices, resources.pointCount);
        if (!sanitized.empty() && sanitized.size() < resources.pointCount) {
            resources.sampledIndexBuffer = CreateHostVisibleBuffer(
                static_cast<VkDeviceSize>(sanitized.size() * sizeof(std::uint32_t)),
                VK_BUFFER_USAGE_INDEX_BUFFER_BIT |
                    VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
            UploadBufferData(resources.sampledIndexBuffer, sanitized.data(), resources.sampledIndexBuffer.size);
            resources.usingSampledIndices = true;
            resources.activePointCount = static_cast<std::uint32_t>(sanitized.size());
            const auto surfelIndices =
                invisible_places::renderer::pointcloud::GenerateSurfelEncodedSampleIndices(sanitized);
            if (!surfelIndices.empty()) {
                resources.sampledSurfelIndexBuffer =
                    CreateHostVisibleBuffer(static_cast<VkDeviceSize>(surfelIndices.size() * sizeof(std::uint32_t)),
                                            VK_BUFFER_USAGE_INDEX_BUFFER_BIT);
                UploadBufferData(resources.sampledSurfelIndexBuffer, surfelIndices.data(),
                                 resources.sampledSurfelIndexBuffer.size);
            }
            if (onRenderThread) {
            TrackPointCloudResidentPeak(PointCloudResourceResidentBytes(resources));
        }
        }

        return resources;
    } catch (...) {
        DestroyStagedPointCloudBuffers(&resources);
        throw;
    }
}

void VulkanViewportShell::DestroyStagedPointCloudBuffers(
    ActivePointCloudResources* resources) {
    if (resources == nullptr) {
        return;
    }
    DestroyBuffer(&resources->positionBuffer);
    DestroyBuffer(&resources->positionStorageBuffer);
    DestroyBuffer(&resources->colorBuffer);
    DestroyBuffer(&resources->normalBuffer);
    DestroyBuffer(&resources->scalarFieldBuffer);
    DestroyBuffer(&resources->seepageNodeBuffer);
    for (auto& paramsBuffer : resources->seepageParamsBuffers) {
        DestroyBuffer(&paramsBuffer);
    }
    DestroyBuffer(&resources->seepageExrParamsBuffer);
    DestroyBuffer(&resources->seepageHashCellBuffer);
    DestroyBuffer(&resources->seepageNodeReferenceBuffer);
    for (auto& styleBuffer : resources->styleBuffers) {
        DestroyBuffer(&styleBuffer);
    }
    DestroyBuffer(&resources->exrStyleBuffer);
    DestroyBuffer(&resources->sampledIndexBuffer);
    DestroyBuffer(&resources->sampledSurfelIndexBuffer);
}

struct VulkanViewportShell::StagedPointCloudUpload {
    ActivePointCloudResources resources;
    std::uint64_t pointCount = 0U;
    std::uint32_t scalarFieldCount = 0U;
    bool hasNormals = false;
};

std::shared_ptr<VulkanViewportShell::StagedPointCloudUpload>
VulkanViewportShell::StagePointCloudUpload(
    const invisible_places::io::LoadedPointCloud& cloud,
    const std::vector<std::uint32_t>& sampledIndices) {
    if (cloud.PointCount() == 0) {
        throw std::runtime_error{"Cannot stage an empty point cloud."};
    }
    if (cloud.PointCount() > std::numeric_limits<std::uint32_t>::max()) {
        throw std::runtime_error{"Point cloud exceeds the current 32-bit draw-count limit."};
    }
    auto staged = std::shared_ptr<StagedPointCloudUpload>(
        new StagedPointCloudUpload{},
        [this](StagedPointCloudUpload* payload) {
            if (payload != nullptr) {
                DestroyStagedPointCloudBuffers(&payload->resources);
                delete payload;
            }
        });
    staged->resources =
        BuildPointCloudUploadResources(cloud, sampledIndices, false);
    staged->pointCount = cloud.PointCount();
    staged->scalarFieldCount =
        static_cast<std::uint32_t>(cloud.ScalarFieldCount());
    staged->hasNormals =
        cloud.hasNormals && cloud.normals.size() == cloud.positions.size();
    return staged;
}

void VulkanViewportShell::UploadPointCloud(std::size_t layerId, const invisible_places::io::LoadedPointCloud& cloud,
                                           const std::vector<std::uint32_t>& sampledIndices,
                                           std::shared_ptr<StagedPointCloudUpload> stagedUpload) {
    if (cloud.PointCount() == 0) {
        throw std::runtime_error{"Cannot upload an empty point cloud."};
    }
    if (cloud.PointCount() > std::numeric_limits<std::uint32_t>::max()) {
        throw std::runtime_error{"Point cloud exceeds the current 32-bit draw-count limit."};
    }

    // Adopt a worker-staged payload when it matches the cloud exactly;
    // otherwise build the buffers inline as before.
    const bool adoptable = stagedUpload != nullptr &&
        stagedUpload->pointCount == cloud.PointCount() &&
        stagedUpload->scalarFieldCount ==
            static_cast<std::uint32_t>(cloud.ScalarFieldCount()) &&
        stagedUpload->hasNormals ==
            (cloud.hasNormals &&
             cloud.normals.size() == cloud.positions.size()) &&
        sampledIndices.empty();
    ActivePointCloudResources resources;
    if (adoptable) {
        resources = std::move(stagedUpload->resources);
        stagedUpload->resources = ActivePointCloudResources{};
        TrackPointCloudResidentPeak(
            PointCloudResourceResidentBytes(resources));
    } else {
        resources =
            BuildPointCloudUploadResources(cloud, sampledIndices, true);
    }
    resources.layerId = layerId;
    try {
        resources.pointDescriptorPool = CreatePointDescriptorPool(
            0U,
            exrExportResources_.depthImage.view != VK_NULL_HANDLE);
        UpdatePointCloudDescriptorSets(&resources, resources.pointDescriptorPool);
        if (exrExportResources_.depthImage.view != VK_NULL_HANDLE) {
            UpdatePointCloudExrDescriptorSet(&resources, exrExportResources_.depthImage.view,
                                             exrExportResources_.uniformBuffer.buffer,
                                             resources.pointDescriptorPool);
        }
    } catch (...) {
        DestroyPointCloudResources(&resources);
        throw;
    }

    try {
        // A fresh layer id is a pure creation: nothing in flight can
        // reference buffers that did not exist, so only a replace needs the
        // device settled before the old resources are destroyed below.
        if (FindPointCloudResources(layerId) != nullptr) {
            SettlePointCloudMutation();
        }
    } catch (...) {
        DestroyPointCloudResources(&resources);
        throw;
    }
    auto* existingResources = FindPointCloudResources(layerId);
    resources.pointDescriptorGeneration =
        existingResources != nullptr ? existingResources->pointDescriptorGeneration + 1U : 1U;
    if (resources.pointDescriptorGeneration == 0U) {
        resources.pointDescriptorGeneration = 1U;
    }
    if (existingResources != nullptr) {
        ActivePointCloudResources retired = std::move(*existingResources);
        *existingResources = std::move(resources);
        DestroyPointCloudResources(&retired);
    } else {
        try {
            pointCloudResources_.push_back(std::move(resources));
        } catch (...) {
            DestroyPointCloudResources(&resources);
            throw;
        }
    }
    TrackPointCloudResidentPeak();
    ++sceneRevision_;
}

void VulkanViewportShell::UploadPointCloudScalarFields(
    std::size_t layerId, const std::vector<invisible_places::io::ScalarFieldStats>& scalarFields,
    const std::vector<float>& scalarFieldValues) {
    auto* resources = FindPointCloudResources(layerId);
    if (resources == nullptr || resources->pointCount == 0U) {
        throw std::runtime_error{"Cannot update scalar fields for an unloaded point cloud."};
    }
    const auto pointCount = static_cast<std::size_t>(resources->pointCount);
    const auto scalarFieldCount = scalarFields.size();
    const auto expectedValueCount = scalarFieldCount * pointCount;
    if (scalarFieldCount > 0U && scalarFieldValues.size() != expectedValueCount) {
        throw std::runtime_error{"Point-cloud scalar field buffer has an unexpected value count."};
    }
    if (scalarFieldCount > std::numeric_limits<std::uint32_t>::max()) {
        throw std::runtime_error{"Point-cloud scalar field count exceeds the current 32-bit limit."};
    }

    BufferAllocation pendingScalarField;
    try {
        if (!scalarFieldValues.empty()) {
            pendingScalarField =
                CreateHostVisibleBuffer(static_cast<VkDeviceSize>(scalarFieldValues.size() * sizeof(float)),
                                        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
            UploadBufferData(pendingScalarField, scalarFieldValues.data(), pendingScalarField.size);
        } else {
            const float fallbackScalar = 0.0F;
            pendingScalarField = CreateHostVisibleBuffer(sizeof(float), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
            UploadBufferData(pendingScalarField, &fallbackScalar, sizeof(float));
        }
    } catch (...) {
        DestroyBuffer(&pendingScalarField);
        throw;
    }
    // The replacement scalar payload overlaps the settled payload until the
    // fresh point descriptor generation is installed. Include that transient
    // overlap in the point-buffer high-water diagnostic.
    TrackPointCloudResidentPeak(
        static_cast<std::uint64_t>(pendingScalarField.size));

    std::swap(resources->scalarFieldBuffer, pendingScalarField);
    const auto settledScalarFieldCount = resources->scalarFieldCount;
    resources->scalarFieldCount = static_cast<std::uint32_t>(scalarFieldCount);
    try {
        ReplacePointCloudDescriptorSets(resources);
    } catch (...) {
        resources->scalarFieldCount = settledScalarFieldCount;
        std::swap(resources->scalarFieldBuffer, pendingScalarField);
        DestroyBuffer(&pendingScalarField);
        throw;
    }
    DestroyBuffer(&pendingScalarField);
    ++sceneRevision_;
}

void VulkanViewportShell::UploadPointCloudScalarFieldValues(
    std::size_t layerId,
    std::size_t scalarFieldIndex,
    std::span<const float> values) {
    auto* resources = FindPointCloudResources(layerId);
    if (resources == nullptr || resources->pointCount == 0U) {
        throw std::runtime_error{"Cannot update a scalar field for an unloaded point cloud."};
    }
    if (scalarFieldIndex >= resources->scalarFieldCount ||
        values.size() != static_cast<std::size_t>(resources->pointCount)) {
        throw std::runtime_error{"Point-cloud scalar field update has an unexpected shape."};
    }
    const auto byteOffset = static_cast<VkDeviceSize>(
        scalarFieldIndex * static_cast<std::size_t>(resources->pointCount) * sizeof(float));
    const auto byteCount = static_cast<VkDeviceSize>(values.size() * sizeof(float));
    if (resources->scalarFieldBuffer.mapped == nullptr ||
        byteOffset + byteCount > resources->scalarFieldBuffer.size) {
        throw std::runtime_error{"Point-cloud scalar field buffer is not directly writable."};
    }

    SettlePointCloudMutation();
    auto* destination = static_cast<std::byte*>(resources->scalarFieldBuffer.mapped) + byteOffset;
    std::memcpy(destination, values.data(), static_cast<std::size_t>(byteCount));
    ++sceneRevision_;
}

DynamicMeshFlowGpuUploadResult VulkanViewportShell::UploadDynamicMeshFlowPreviewPointCloud(
    std::size_t layerId,
    const invisible_places::water::MeshSurfaceCache& cache,
    const std::vector<invisible_places::water::WaterEmitter>& emitters,
    const invisible_places::water::WaterDynamicMeshFlowSettings& settings,
    invisible_places::water::WaterTrailBuildQuality quality) {
    (void)cache;
    (void)emitters;
    (void)quality;
    DynamicMeshFlowGpuFrameRequest request;
    request.settings = settings;
    request.activity = settings.enabled ? 1.0F : 0.0F;
    request.moisture = 0.0F;
    request.resetSimulation = false;
    const auto update = UpdateDynamicMeshFlowGpuSimulation(layerId, request);
    return {
        .pointCount = update.pointCount,
        .particleCount = update.particleCapacity,
        .routeAnchorCount = update.historyLength,
        .visibleSampleCount = update.particleCapacity,
        .solveMilliseconds =
            update.allocationMilliseconds +
            update.parameterUploadMilliseconds +
            update.dispatchMilliseconds,
        .staticUploadMilliseconds = update.allocationMilliseconds,
        .liveUploadMilliseconds = update.parameterUploadMilliseconds,
        .dispatchMilliseconds = update.dispatchMilliseconds,
        .reusedStaticBuffers = !update.allocatedThisUpdate,
        .asynchronousDispatch = update.asynchronousDispatch,
    };
#if 0
    const auto startedAt = std::chrono::steady_clock::now();
    if (!settings.enabled || cache.cells.empty()) {
        throw std::runtime_error{"Dynamic mesh flow GPU upload requires an enabled settings object and a warm cache."};
    }
    if (dynamicMeshFlowComputePipeline_ == VK_NULL_HANDLE ||
        dynamicMeshFlowPipelineLayout_ == VK_NULL_HANDLE ||
        dynamicMeshFlowDescriptorSetLayout_ == VK_NULL_HANDLE) {
        throw std::runtime_error{"Dynamic mesh flow compute pipeline is not available."};
    }

    std::vector<DynamicMeshFlowEmitterGpu> activeEmitters;
    activeEmitters.reserve(emitters.size());
    for (const auto& emitter : emitters) {
        if (emitter.status == invisible_places::water::WaterEmitterStatus::Disabled ||
            emitter.strength <= 0.0F) {
            continue;
        }
        DynamicMeshFlowEmitterGpu gpuEmitter;
        gpuEmitter.positionRadius = glm::vec4{
            emitter.position.x,
            emitter.position.y,
            emitter.position.z,
            std::max(0.0F, emitter.radius),
        };
        gpuEmitter.strengthSpeedIdSeed = glm::vec4{
            std::max(0.0F, emitter.strength),
            std::max(0.01F, emitter.speed),
            static_cast<float>(emitter.id),
            static_cast<float>(settings.seed ^ (emitter.id * 2654435761U)),
        };
        activeEmitters.push_back(gpuEmitter);
    }
    if (activeEmitters.empty()) {
        throw std::runtime_error{"Dynamic mesh flow GPU upload needs at least one active emitter."};
    }

    std::vector<DynamicMeshFlowAttractorGpu> gpuAttractors;
    gpuAttractors.reserve(std::max<std::size_t>(1U, settings.attractors.size()));
    for (const auto& attractor : settings.attractors) {
        DynamicMeshFlowAttractorGpu gpuAttractor;
        gpuAttractor.positionRadius = glm::vec4{
            attractor.position.x,
            attractor.position.y,
            attractor.position.z,
            std::max(0.0F, attractor.radiusMeters),
        };
        gpuAttractor.strengthEnabled = glm::vec4{
            std::max(0.0F, attractor.strength),
            attractor.enabled ? 1.0F : 0.0F,
            static_cast<float>(attractor.id),
            0.0F,
        };
        gpuAttractors.push_back(gpuAttractor);
    }
    if (gpuAttractors.empty()) {
        gpuAttractors.push_back({});
    }

    const std::uint32_t requestedParticleCount = std::max<std::uint32_t>(
        1U,
        quality == invisible_places::water::WaterTrailBuildQuality::Preview
            ? settings.previewParticleLimit
            : settings.finalParticleLimit);
    const std::uint32_t routeAnchorLimit =
        quality == invisible_places::water::WaterTrailBuildQuality::Preview ? 192U : 384U;
    const std::uint32_t routeAnchorCount = std::clamp<std::uint32_t>(
        static_cast<std::uint32_t>(
            std::ceil(std::max(0.02F, settings.trailLengthMeters) /
                      std::max(0.002F, settings.stepMeters))) + 1U,
        2U,
        routeAnchorLimit);
    const std::uint32_t visibleSampleCount = routeAnchorCount;
    const std::uint32_t routeStride = routeAnchorCount + visibleSampleCount;
    constexpr std::uint32_t kMaxGpuMeshFlowPreviewPoints = 1'200'000U;
    const std::uint32_t particleCount = std::max<std::uint32_t>(
        1U,
        std::min(requestedParticleCount, kMaxGpuMeshFlowPreviewPoints / std::max(1U, routeStride)));
    const std::uint64_t pointCount64 =
        static_cast<std::uint64_t>(particleCount) * static_cast<std::uint64_t>(routeStride);
    if (pointCount64 == 0U || pointCount64 > std::numeric_limits<std::uint32_t>::max()) {
        throw std::runtime_error{"Dynamic mesh flow GPU output exceeds the current 32-bit draw-count limit."};
    }
    const auto pointCount = static_cast<std::uint32_t>(pointCount64);

    auto* resources = FindPointCloudResources(layerId);
    if (resources == nullptr) {
        pointCloudResources_.push_back(ActivePointCloudResources{});
        resources = &pointCloudResources_.back();
    }
    const auto emitterCapacityNeeded = static_cast<std::uint32_t>(activeEmitters.size());
    const auto attractorCapacityNeeded = static_cast<std::uint32_t>(gpuAttractors.size());
    const bool hasStaticDynamicMeshBuffers =
        resources->dynamicMeshFlowCellBuffer.buffer != VK_NULL_HANDLE &&
        resources->dynamicMeshFlowGridBuffer.buffer != VK_NULL_HANDLE &&
        resources->positionStorageBuffer.buffer != VK_NULL_HANDLE &&
        resources->normalBuffer.buffer != VK_NULL_HANDLE &&
        resources->scalarFieldBuffer.buffer != VK_NULL_HANDLE;
    const bool hasLiveDynamicMeshBuffers =
        resources->dynamicMeshFlowUniformBuffers.front().buffer != VK_NULL_HANDLE &&
        resources->dynamicMeshFlowEmitterBuffers.front().buffer != VK_NULL_HANDLE &&
        resources->dynamicMeshFlowAttractorBuffers.front().buffer != VK_NULL_HANDLE;
    const bool rebuildStaticBuffers =
        !hasStaticDynamicMeshBuffers ||
        resources->layerId != layerId ||
        resources->dynamicMeshFlowCacheIdentity != &cache ||
        resources->pointCount != pointCount ||
        resources->dynamicMeshFlowParticleCount != particleCount ||
        resources->dynamicMeshFlowRouteAnchorCount != routeAnchorCount ||
        resources->dynamicMeshFlowVisibleSampleCount != visibleSampleCount ||
        resources->scalarFieldCount != 31U;
    const bool rebuildLiveBuffers =
        rebuildStaticBuffers ||
        !hasLiveDynamicMeshBuffers ||
        resources->dynamicMeshFlowEmitterCapacity < emitterCapacityNeeded ||
        resources->dynamicMeshFlowAttractorCapacity < attractorCapacityNeeded;

    double staticUploadMilliseconds = 0.0;
    if (rebuildStaticBuffers) {
        const auto staticStartedAt = std::chrono::steady_clock::now();
        CleanupPointCloudResources(resources);

        const float cellSize = std::clamp(cache.settings.cacheCellSizeMeters, 0.005F, 5.0F);
        int minCellX = std::numeric_limits<int>::max();
        int minCellY = std::numeric_limits<int>::max();
        int maxCellX = std::numeric_limits<int>::min();
        int maxCellY = std::numeric_limits<int>::min();
        std::vector<DynamicMeshSurfaceCellGpu> gpuCells;
        gpuCells.reserve(cache.cells.size());
        for (const auto& cell : cache.cells) {
            const int cellX = cell.cellX;
            const int cellY = cell.cellY;
            minCellX = std::min(minCellX, cellX);
            minCellY = std::min(minCellY, cellY);
            maxCellX = std::max(maxCellX, cellX);
            maxCellY = std::max(maxCellY, cellY);

            DynamicMeshSurfaceCellGpu gpuCell;
            gpuCell.positionConfidence = glm::vec4{
                cell.position.x,
                cell.position.y,
                cell.position.z,
                std::clamp(cell.confidence, 0.0F, 1.0F),
            };
            gpuCell.normalAmbiguous = glm::vec4{
                cell.normal.x,
                cell.normal.y,
                cell.normal.z,
                cell.ambiguous ? 1.0F : 0.0F,
            };
            gpuCell.downhillSampleCount = glm::vec4{
                cell.downhill.x,
                cell.downhill.y,
                cell.downhill.z,
                static_cast<float>(cell.sampleCount),
            };
            gpuCells.push_back(gpuCell);
        }
        if (gpuCells.empty() || minCellX > maxCellX || minCellY > maxCellY) {
            throw std::runtime_error{"Dynamic mesh flow GPU cache has no uploadable cells."};
        }
        const std::uint64_t gridWidth64 = static_cast<std::uint64_t>(
            static_cast<long long>(maxCellX) - static_cast<long long>(minCellX) + 1LL);
        const std::uint64_t gridHeight64 = static_cast<std::uint64_t>(
            static_cast<long long>(maxCellY) - static_cast<long long>(minCellY) + 1LL);
        constexpr std::uint64_t kMaxDynamicMeshFlowGridEntries = 32'000'000ULL;
        if (gridWidth64 == 0U ||
            gridHeight64 == 0U ||
            gridWidth64 > std::numeric_limits<std::uint32_t>::max() ||
            gridHeight64 > std::numeric_limits<std::uint32_t>::max() ||
            gridWidth64 * gridHeight64 > kMaxDynamicMeshFlowGridEntries) {
            throw std::runtime_error{
                "Dynamic mesh flow GPU cache grid is too large; increase the mesh cache cell size."};
        }
        const auto gridWidth = static_cast<std::uint32_t>(gridWidth64);
        const auto gridHeight = static_cast<std::uint32_t>(gridHeight64);
        std::vector<std::uint32_t> denseGrid(static_cast<std::size_t>(gridWidth64 * gridHeight64), 0U);
        for (std::size_t cellIndex = 0; cellIndex < cache.cells.size(); ++cellIndex) {
            const auto& cell = cache.cells[cellIndex];
            const int cellX = cell.cellX;
            const int cellY = cell.cellY;
            const auto localX = static_cast<std::uint32_t>(cellX - minCellX);
            const auto localY = static_cast<std::uint32_t>(cellY - minCellY);
            denseGrid[static_cast<std::size_t>(localY) * gridWidth + localX] =
                static_cast<std::uint32_t>(std::min<std::size_t>(
                    cellIndex + 1U,
                    static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())));
        }

        resources->layerId = layerId;
        resources->pointCount = pointCount;
        resources->activePointCount = pointCount;
        resources->scalarFieldCount = 31U;
        resources->hasSourceRgb = true;
        resources->hasNormals = true;
        resources->dynamicMeshFlowCacheIdentity = &cache;
        resources->dynamicMeshFlowParticleCount = particleCount;
        resources->dynamicMeshFlowRouteAnchorCount = routeAnchorCount;
        resources->dynamicMeshFlowVisibleSampleCount = visibleSampleCount;
        resources->dynamicMeshFlowGridWidth = gridWidth;
        resources->dynamicMeshFlowGridHeight = gridHeight;
        resources->dynamicMeshFlowEmitterCapacity = emitterCapacityNeeded;
        resources->dynamicMeshFlowAttractorCapacity = attractorCapacityNeeded;
        resources->dynamicMeshFlowCellCount = cache.cells.size();
        resources->dynamicMeshFlowMinCellX = minCellX;
        resources->dynamicMeshFlowMinCellY = minCellY;
        resources->dynamicMeshFlowCellSize = cellSize;
        resources->dynamicMeshFlowNextLiveSlot = 0;

        resources->positionBuffer = CreateHostVisibleBuffer(
            static_cast<VkDeviceSize>(pointCount) * sizeof(invisible_places::io::Float3),
            VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
        if (resources->positionBuffer.mapped != nullptr) {
            std::memset(resources->positionBuffer.mapped, 0, static_cast<std::size_t>(resources->positionBuffer.size));
        }
        resources->positionStorageBuffer = CreateHostVisibleBuffer(
            static_cast<VkDeviceSize>(pointCount) * sizeof(glm::vec4),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        resources->normalBuffer = CreateHostVisibleBuffer(
            static_cast<VkDeviceSize>(pointCount) * sizeof(glm::vec4),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        resources->scalarFieldBuffer = CreateHostVisibleBuffer(
            static_cast<VkDeviceSize>(pointCount) * 31U * sizeof(float),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        if (resources->positionStorageBuffer.mapped != nullptr) {
            std::memset(resources->positionStorageBuffer.mapped, 0, static_cast<std::size_t>(resources->positionStorageBuffer.size));
        }
        if (resources->normalBuffer.mapped != nullptr) {
            std::memset(resources->normalBuffer.mapped, 0, static_cast<std::size_t>(resources->normalBuffer.size));
        }
        if (resources->scalarFieldBuffer.mapped != nullptr) {
            std::memset(resources->scalarFieldBuffer.mapped, 0, static_cast<std::size_t>(resources->scalarFieldBuffer.size));
        }

        const std::uint32_t waterColor =
            static_cast<std::uint32_t>(190U) |
            (static_cast<std::uint32_t>(230U) << 8U) |
            (static_cast<std::uint32_t>(255U) << 16U) |
            (0xFFU << 24U);
        std::vector<std::uint32_t> packedColors(pointCount, waterColor);
        resources->colorBuffer = CreateHostVisibleBuffer(
            static_cast<VkDeviceSize>(packedColors.size() * sizeof(std::uint32_t)),
            VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        UploadBufferData(resources->colorBuffer, packedColors.data(), resources->colorBuffer.size);

        const WaterSeepageNodeTopologyGpu emptySeepageNode{};
        resources->seepageNodeBuffer = CreateHostVisibleBuffer(
            sizeof(emptySeepageNode),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        UploadBufferData(resources->seepageNodeBuffer, &emptySeepageNode, sizeof(emptySeepageNode));
        const WaterSeepageNodeParamsGpu emptySeepageParams{};
        for (auto& paramsBuffer : resources->seepageParamsBuffers) {
            paramsBuffer = CreateHostVisibleBuffer(
                sizeof(emptySeepageParams),
                VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
            UploadBufferData(paramsBuffer, &emptySeepageParams, sizeof(emptySeepageParams));
        }
        resources->seepageExrParamsBuffer = CreateHostVisibleBuffer(
            sizeof(emptySeepageParams),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        UploadBufferData(
            resources->seepageExrParamsBuffer,
            &emptySeepageParams,
            sizeof(emptySeepageParams));
        resources->pendingSeepageParams.resize(sizeof(emptySeepageParams));
        std::memcpy(
            resources->pendingSeepageParams.data(),
            &emptySeepageParams,
            sizeof(emptySeepageParams));
        const WaterSeepageHashCellGpu emptySeepageHashCell{};
        resources->seepageHashCellBuffer = CreateHostVisibleBuffer(
            sizeof(emptySeepageHashCell),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        UploadBufferData(
            resources->seepageHashCellBuffer,
            &emptySeepageHashCell,
            sizeof(emptySeepageHashCell));
        const invisible_places::water::WaterSeepageSupportReference
            emptySeepageNodeReference{};
        resources->seepageNodeReferenceBuffer = CreateHostVisibleBuffer(
            sizeof(emptySeepageNodeReference),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        UploadBufferData(
            resources->seepageNodeReferenceBuffer,
            &emptySeepageNodeReference,
            sizeof(emptySeepageNodeReference));

        resources->dynamicMeshFlowCellBuffer = CreateHostVisibleBuffer(
            static_cast<VkDeviceSize>(gpuCells.size() * sizeof(DynamicMeshSurfaceCellGpu)),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        UploadBufferData(resources->dynamicMeshFlowCellBuffer, gpuCells.data(), resources->dynamicMeshFlowCellBuffer.size);
        resources->dynamicMeshFlowGridBuffer = CreateHostVisibleBuffer(
            static_cast<VkDeviceSize>(denseGrid.size() * sizeof(std::uint32_t)),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        UploadBufferData(resources->dynamicMeshFlowGridBuffer, denseGrid.data(), resources->dynamicMeshFlowGridBuffer.size);

        for (std::size_t liveSlot = 0; liveSlot < kDynamicMeshFlowLiveSlots; ++liveSlot) {
            resources->dynamicMeshFlowUniformBuffers[liveSlot] = CreateHostVisibleBuffer(
                sizeof(DynamicMeshFlowUniformsGpu),
                VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT);
            resources->dynamicMeshFlowEmitterBuffers[liveSlot] = CreateHostVisibleBuffer(
                static_cast<VkDeviceSize>(emitterCapacityNeeded * sizeof(DynamicMeshFlowEmitterGpu)),
                VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
            resources->dynamicMeshFlowAttractorBuffers[liveSlot] = CreateHostVisibleBuffer(
                static_cast<VkDeviceSize>(attractorCapacityNeeded * sizeof(DynamicMeshFlowAttractorGpu)),
                VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        }

        for (auto& styleBuffer : resources->styleBuffers) {
            styleBuffer = CreateHostVisibleBuffer(sizeof(PointCloudStyleGpu), VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT);
        }
        resources->exrStyleBuffer = CreateHostVisibleBuffer(sizeof(PointCloudStyleGpu), VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT);

        ReplacePointCloudDescriptorSets(resources);
        for (std::size_t liveSlot = 0; liveSlot < kDynamicMeshFlowLiveSlots; ++liveSlot) {
            UpdateDynamicMeshFlowDescriptorSet(resources, liveSlot);
        }
        staticUploadMilliseconds = MillisecondsBetween(staticStartedAt, std::chrono::steady_clock::now());
    }
    if (!rebuildStaticBuffers && rebuildLiveBuffers) {
        SettlePointCloudMutation();
        for (std::size_t liveSlot = 0; liveSlot < kDynamicMeshFlowLiveSlots; ++liveSlot) {
            auto& commandBuffer = resources->dynamicMeshFlowCommandBuffers[liveSlot];
            if (commandBuffer != VK_NULL_HANDLE && commandPool_ != VK_NULL_HANDLE) {
                vkFreeCommandBuffers(device_, commandPool_, 1, &commandBuffer);
                commandBuffer = VK_NULL_HANDLE;
            }
            auto& fence = resources->dynamicMeshFlowDispatchFences[liveSlot];
            if (fence != VK_NULL_HANDLE) {
                vkDestroyFence(device_, fence, nullptr);
                fence = VK_NULL_HANDLE;
            }
            auto& descriptorSet = resources->dynamicMeshFlowDescriptorSets[liveSlot];
            descriptorSet = VK_NULL_HANDLE;
            DestroyBuffer(&resources->dynamicMeshFlowUniformBuffers[liveSlot]);
            DestroyBuffer(&resources->dynamicMeshFlowEmitterBuffers[liveSlot]);
            DestroyBuffer(&resources->dynamicMeshFlowAttractorBuffers[liveSlot]);
        }
        resources->dynamicMeshFlowEmitterCapacity = emitterCapacityNeeded;
        resources->dynamicMeshFlowAttractorCapacity = attractorCapacityNeeded;
        resources->dynamicMeshFlowNextLiveSlot = 0;
        for (std::size_t liveSlot = 0; liveSlot < kDynamicMeshFlowLiveSlots; ++liveSlot) {
            resources->dynamicMeshFlowUniformBuffers[liveSlot] = CreateHostVisibleBuffer(
                sizeof(DynamicMeshFlowUniformsGpu),
                VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT);
            resources->dynamicMeshFlowEmitterBuffers[liveSlot] = CreateHostVisibleBuffer(
                static_cast<VkDeviceSize>(emitterCapacityNeeded * sizeof(DynamicMeshFlowEmitterGpu)),
                VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
            resources->dynamicMeshFlowAttractorBuffers[liveSlot] = CreateHostVisibleBuffer(
                static_cast<VkDeviceSize>(attractorCapacityNeeded * sizeof(DynamicMeshFlowAttractorGpu)),
                VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
            UpdateDynamicMeshFlowDescriptorSet(resources, liveSlot);
        }
    }

    DynamicMeshFlowUniformsGpu uniforms;
    uniforms.counts0 = glm::uvec4{particleCount, routeAnchorCount, visibleSampleCount, pointCount};
    uniforms.counts1 = glm::uvec4{
        31U,
        static_cast<std::uint32_t>(activeEmitters.size()),
        static_cast<std::uint32_t>(settings.attractors.size()),
        settings.seed,
    };
    uniforms.surface0 = glm::vec4{
        resources->dynamicMeshFlowCellSize,
        std::max(settings.projectionSearchRadiusMeters, resources->dynamicMeshFlowCellSize),
        settings.surfaceOffsetMeters,
        invisible_places::water::kWaterTrailFeatureTypeDynamicMesh,
    };
    uniforms.grid0 = glm::ivec4{
        resources->dynamicMeshFlowMinCellX,
        resources->dynamicMeshFlowMinCellY,
        std::clamp(
            static_cast<int>(
                std::ceil(
                    std::max(settings.projectionSearchRadiusMeters, resources->dynamicMeshFlowCellSize) /
                    resources->dynamicMeshFlowCellSize)),
            1,
            24),
        0,
    };
    uniforms.grid1 = glm::uvec4{
        resources->dynamicMeshFlowGridWidth,
        resources->dynamicMeshFlowGridHeight,
        0U,
        0U,
    };
    uniforms.flow0 = glm::vec4{
        std::max(0.02F, settings.trailLengthMeters),
        std::clamp(settings.stepMeters, 0.002F, 5.0F),
        std::max(0.01F, settings.speedMetersPerSecond),
        std::max(0.0F, settings.animationDurationSeconds),
    };
    uniforms.flow1 = glm::vec4{
        std::max(0.0F, settings.downhillWeight),
        std::max(0.0F, settings.attractorWeight),
        std::max(0.0F, settings.sourceVelocityWeight),
        std::max(0.0F, settings.curlStrength),
    };
    uniforms.flow2 = glm::vec4{
        std::clamp(settings.inertia, 0.0F, 0.98F),
        std::max(0.0F, settings.branchingStrength),
        std::max(0.0F, settings.eddyStrength),
        std::max(0.0F, settings.topologyResponse),
    };
    uniforms.visual0 = glm::vec4{
        std::max(0.0005F, settings.trailWidthMeters),
        std::max(0.001F, settings.trailStreakLengthMeters),
        std::max(settings.trailWidthMeters * 8.0F, settings.stepMeters * 0.75F),
        std::clamp(settings.attractorWeight * 0.22F, 0.0F, 2.0F),
    };

    const auto liveUploadStartedAt = std::chrono::steady_clock::now();
    const auto liveSlot = resources->dynamicMeshFlowNextLiveSlot % kDynamicMeshFlowLiveSlots;
    resources->dynamicMeshFlowNextLiveSlot = (resources->dynamicMeshFlowNextLiveSlot + 1U) % kDynamicMeshFlowLiveSlots;
    PrepareDynamicMeshFlowDispatchSlot(resources, liveSlot);
    UploadBufferData(resources->dynamicMeshFlowUniformBuffers[liveSlot], &uniforms, sizeof(uniforms));
    UploadBufferData(
        resources->dynamicMeshFlowEmitterBuffers[liveSlot],
        activeEmitters.data(),
        static_cast<VkDeviceSize>(activeEmitters.size() * sizeof(DynamicMeshFlowEmitterGpu)));
    UploadBufferData(
        resources->dynamicMeshFlowAttractorBuffers[liveSlot],
        gpuAttractors.data(),
        static_cast<VkDeviceSize>(gpuAttractors.size() * sizeof(DynamicMeshFlowAttractorGpu)));
    if (resources->dynamicMeshFlowDescriptorSets[liveSlot] == VK_NULL_HANDLE) {
        UpdateDynamicMeshFlowDescriptorSet(resources, liveSlot);
    }
    const double liveUploadMilliseconds = MillisecondsBetween(liveUploadStartedAt, std::chrono::steady_clock::now());
    const auto dispatchStartedAt = std::chrono::steady_clock::now();
    DispatchDynamicMeshFlowCompute(resources, particleCount, liveSlot);
    const double dispatchMilliseconds = MillisecondsBetween(dispatchStartedAt, std::chrono::steady_clock::now());

    ++sceneRevision_;
    return {
        .pointCount = pointCount,
        .particleCount = particleCount,
        .routeAnchorCount = routeAnchorCount,
        .visibleSampleCount = visibleSampleCount,
        .solveMilliseconds = MillisecondsBetween(startedAt, std::chrono::steady_clock::now()),
        .staticUploadMilliseconds = staticUploadMilliseconds,
        .liveUploadMilliseconds = liveUploadMilliseconds,
        .dispatchMilliseconds = dispatchMilliseconds,
        .reusedStaticBuffers = !rebuildStaticBuffers,
        .asynchronousDispatch = true,
    };
#endif
}

DynamicMeshFlowGpuUpdateResult VulkanViewportShell::UpdateDynamicMeshFlowGpuSimulation(
    std::size_t layerId,
    const DynamicMeshFlowGpuFrameRequest& request) {
    constexpr std::uint32_t kScalarFieldCount = 31U;
    constexpr std::uint32_t kMaximumParticleCapacity = 65'536U;
    constexpr std::uint32_t kMaximumHistoryLength = 128U;
    constexpr std::uint32_t kMaximumPointCount = 4'000'000U;

    DynamicMeshFlowGpuUpdateResult result;
    const auto groundView = WaterGroundFlowView();
    result.sharedGroundReady = groundView.valid;
    result.sharedGroundUploadRevision = groundView.uploadRevision;
    if (!groundView.valid ||
        dynamicMeshFlowComputePipeline_ == VK_NULL_HANDLE ||
        dynamicMeshFlowPipelineLayout_ == VK_NULL_HANDLE ||
        dynamicMeshFlowDescriptorSetLayout_ == VK_NULL_HANDLE) {
        return result;
    }

    const auto particleCapacity = std::clamp(
        request.settings.particleCapacity,
        1U,
        kMaximumParticleCapacity);
    const auto historyLength = std::clamp(
        request.settings.historyLength,
        2U,
        kMaximumHistoryLength);
    const std::uint64_t pointCount64 =
        static_cast<std::uint64_t>(particleCapacity) *
        static_cast<std::uint64_t>(historyLength + 1U);
    if (pointCount64 > kMaximumPointCount ||
        pointCount64 > std::numeric_limits<std::uint32_t>::max()) {
        throw std::runtime_error{
            "Dynamic Mesh Flow fixed history exceeds the GPU point budget."};
    }
    const auto pointCount = static_cast<std::uint32_t>(pointCount64);
    // Splat registration multiplies bucket references, so the grid is sized
    // well past the particle count to keep hash-collision eviction rare.
    std::uint32_t contactGridCapacity = 1U;
    while (contactGridCapacity < particleCapacity * 8U) {
        contactGridCapacity <<= 1U;
    }

    auto* resources = FindPointCloudResources(layerId);
    if (resources == nullptr) {
        pointCloudResources_.push_back(ActivePointCloudResources{});
        resources = &pointCloudResources_.back();
    }
    const bool groundIdentityChanged =
        resources->dynamicMeshFlowGroundUploadRevision != groundView.uploadRevision ||
        resources->dynamicMeshFlowGroundIdentity != *groundView.cacheIdentity;
    const bool needsAllocation =
        resources->dynamicMeshFlowAllocationRevision == 0U ||
        resources->layerId != layerId ||
        resources->dynamicMeshFlowParticleCount != particleCapacity ||
        resources->dynamicMeshFlowRouteAnchorCount != historyLength ||
        resources->pointCount != pointCount ||
        groundIdentityChanged ||
        resources->dynamicMeshFlowParticleBuffer.buffer == VK_NULL_HANDLE ||
        resources->dynamicMeshFlowContactEventBuffer.buffer == VK_NULL_HANDLE ||
        resources->dynamicMeshFlowContactGridBuffer.buffer == VK_NULL_HANDLE ||
        resources->dynamicMeshFlowDescriptorPool == VK_NULL_HANDLE ||
        resources->positionStorageBuffer.buffer == VK_NULL_HANDLE ||
        resources->normalBuffer.buffer == VK_NULL_HANDLE ||
        resources->scalarFieldBuffer.buffer == VK_NULL_HANDLE;

    if (needsAllocation) {
        const auto allocationStartedAt = std::chrono::steady_clock::now();
        ActivePointCloudResources staged;
        staged.layerId = layerId;
        staged.pointCount = pointCount;
        staged.activePointCount = pointCount;
        staged.scalarFieldCount = kScalarFieldCount;
        staged.hasSourceRgb = true;
        staged.hasNormals = true;
        staged.dynamicMeshFlowGroundIdentity = *groundView.cacheIdentity;
        staged.dynamicMeshFlowGroundUploadRevision = groundView.uploadRevision;
        staged.dynamicMeshFlowParticleCount = particleCapacity;
        staged.dynamicMeshFlowRouteAnchorCount = historyLength;
        staged.dynamicMeshFlowVisibleSampleCount = particleCapacity;
        staged.dynamicMeshFlowEventCapacity = particleCapacity;
        staged.dynamicMeshFlowContactGridMask =
            contactGridCapacity - 1U;
        staged.dynamicMeshFlowCellSize = groundView.resolutionMeters;
        staged.dynamicMeshFlowAllocationRevision =
            resources->dynamicMeshFlowAllocationRevision + 1U;
        staged.dynamicMeshFlowDescriptorGeneration =
            resources->dynamicMeshFlowDescriptorGeneration + 1U;
        staged.dynamicMeshFlowResetEpoch = 1U;

        try {
            staged.positionBuffer = CreateHostVisibleBuffer(
                static_cast<VkDeviceSize>(pointCount) *
                    sizeof(invisible_places::io::Float3),
                VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
            staged.positionStorageBuffer = CreateHostVisibleBuffer(
                static_cast<VkDeviceSize>(pointCount) * sizeof(glm::vec4),
                VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
            staged.normalBuffer = CreateHostVisibleBuffer(
                static_cast<VkDeviceSize>(pointCount) * sizeof(glm::vec4),
                VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
            staged.scalarFieldBuffer = CreateHostVisibleBuffer(
                static_cast<VkDeviceSize>(pointCount) *
                    kScalarFieldCount * sizeof(float),
                VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
            staged.dynamicMeshFlowParticleBuffer = CreateHostVisibleBuffer(
                static_cast<VkDeviceSize>(particleCapacity) *
                    sizeof(DynamicMeshFlowParticleGpu),
                VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
            staged.dynamicMeshFlowContactEventBuffer = CreateHostVisibleBuffer(
                static_cast<VkDeviceSize>(particleCapacity) *
                    sizeof(DynamicMeshFlowContactEventGpu),
                VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
            staged.dynamicMeshFlowContactGridBuffer =
                CreateHostVisibleBuffer(
                    static_cast<VkDeviceSize>(contactGridCapacity) *
                        sizeof(glm::uvec4),
                    VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);

            for (const auto* allocation : {
                     &staged.positionBuffer,
                     &staged.positionStorageBuffer,
                     &staged.normalBuffer,
                     &staged.scalarFieldBuffer,
                     &staged.dynamicMeshFlowParticleBuffer,
                     &staged.dynamicMeshFlowContactEventBuffer,
                     &staged.dynamicMeshFlowContactGridBuffer}) {
                if (allocation->mapped != nullptr) {
                    std::memset(
                        allocation->mapped,
                        0,
                        static_cast<std::size_t>(allocation->size));
                }
            }

            const std::uint32_t waterColor =
                static_cast<std::uint32_t>(190U) |
                (static_cast<std::uint32_t>(230U) << 8U) |
                (static_cast<std::uint32_t>(255U) << 16U) |
                (0xFFU << 24U);
            std::vector<std::uint32_t> packedColors(pointCount, waterColor);
            staged.colorBuffer = CreateHostVisibleBuffer(
                static_cast<VkDeviceSize>(packedColors.size()) *
                    sizeof(std::uint32_t),
                VK_BUFFER_USAGE_VERTEX_BUFFER_BIT |
                    VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
            UploadBufferData(
                staged.colorBuffer,
                packedColors.data(),
                staged.colorBuffer.size);

            const WaterSeepageNodeTopologyGpu emptySeepageNode{};
            staged.seepageNodeBuffer = CreateHostVisibleBuffer(
                sizeof(emptySeepageNode),
                VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
            UploadBufferData(
                staged.seepageNodeBuffer,
                &emptySeepageNode,
                sizeof(emptySeepageNode));
            const WaterSeepageNodeParamsGpu emptySeepageParams{};
            for (auto& paramsBuffer : staged.seepageParamsBuffers) {
                paramsBuffer = CreateHostVisibleBuffer(
                    sizeof(emptySeepageParams),
                    VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
                UploadBufferData(
                    paramsBuffer,
                    &emptySeepageParams,
                    sizeof(emptySeepageParams));
            }
            staged.seepageExrParamsBuffer = CreateHostVisibleBuffer(
                sizeof(emptySeepageParams),
                VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
            UploadBufferData(
                staged.seepageExrParamsBuffer,
                &emptySeepageParams,
                sizeof(emptySeepageParams));
            staged.pendingSeepageParams.resize(sizeof(emptySeepageParams));
            std::memcpy(
                staged.pendingSeepageParams.data(),
                &emptySeepageParams,
                sizeof(emptySeepageParams));
            const WaterSeepageHashCellGpu emptySeepageHashCell{};
            staged.seepageHashCellBuffer = CreateHostVisibleBuffer(
                sizeof(emptySeepageHashCell),
                VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
            UploadBufferData(
                staged.seepageHashCellBuffer,
                &emptySeepageHashCell,
                sizeof(emptySeepageHashCell));
            const invisible_places::water::WaterSeepageSupportReference
                emptySeepageNodeReference{};
            staged.seepageNodeReferenceBuffer = CreateHostVisibleBuffer(
                sizeof(emptySeepageNodeReference),
                VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
            UploadBufferData(
                staged.seepageNodeReferenceBuffer,
                &emptySeepageNodeReference,
                sizeof(emptySeepageNodeReference));

            for (auto& styleBuffer : staged.styleBuffers) {
                styleBuffer = CreateHostVisibleBuffer(
                    sizeof(PointCloudStyleGpu),
                    VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT);
            }
            staged.exrStyleBuffer = CreateHostVisibleBuffer(
                sizeof(PointCloudStyleGpu),
                VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT);

            for (std::size_t liveSlot = 0U;
                 liveSlot < kDynamicMeshFlowLiveSlots;
                 ++liveSlot) {
                staged.dynamicMeshFlowUniformBuffers[liveSlot] =
                    CreateHostVisibleBuffer(
                        sizeof(DynamicMeshFlowUniformsGpu),
                        VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT);
                staged.dynamicMeshFlowCounterBuffers[liveSlot] =
                    CreateHostVisibleBuffer(
                        sizeof(DynamicMeshFlowCountersGpu),
                        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
            }
            staged.dynamicMeshFlowDescriptorPool =
                CreateDynamicMeshFlowDescriptorPool();
            for (std::size_t liveSlot = 0U;
                 liveSlot < kDynamicMeshFlowLiveSlots;
                 ++liveSlot) {
                UpdateDynamicMeshFlowDescriptorSet(
                    &staged,
                    liveSlot,
                    groundView);
            }
            auto stagedPointGeneration = BuildPointDescriptorGeneration(
                &staged,
                exrExportResources_.depthImage.view,
                exrExportResources_.uniformBuffer.buffer);
            InstallPointDescriptorGeneration(
                &staged,
                &stagedPointGeneration);
        } catch (...) {
            DestroyPointCloudResources(&staged);
            throw;
        }

        struct PendingBaseContactGeneration {
            std::size_t resourceIndex = 0U;
            PendingPointDescriptorGeneration generation{};
        };
        std::vector<PendingBaseContactGeneration> pendingBaseGenerations;
        pointDescriptorDynamicMeshFlowContactOverride_ =
            PointDescriptorDynamicMeshFlowContactView(&staged);
        try {
            for (std::size_t resourceIndex = 0U;
                 resourceIndex < pointCloudResources_.size();
                 ++resourceIndex) {
                auto& candidate = pointCloudResources_[resourceIndex];
                if (&candidate == resources ||
                    candidate.dynamicMeshFlowAllocationRevision != 0U ||
                    candidate.pointDescriptorPool == VK_NULL_HANDLE) {
                    continue;
                }
                PendingBaseContactGeneration pendingBase;
                pendingBase.resourceIndex = resourceIndex;
                pendingBase.generation = BuildPointDescriptorGeneration(
                    &candidate,
                    exrExportResources_.depthImage.view,
                    exrExportResources_.uniformBuffer.buffer);
                pendingBaseGenerations.push_back(
                    std::move(pendingBase));
            }
        } catch (...) {
            pointDescriptorDynamicMeshFlowContactOverride_.reset();
            for (auto& pendingBase : pendingBaseGenerations) {
                DestroyPointDescriptorGeneration(
                    &pendingBase.generation);
            }
            DestroyPointCloudResources(&staged);
            throw;
        }
        pointDescriptorDynamicMeshFlowContactOverride_.reset();

        // Descriptor and buffer generations become visible together. The old
        // generation remains fully bound until the one settle immediately
        // before the swap, so failed staging cannot disturb the last frame.
        TrackPointCloudResidentPeak(PointCloudResourceResidentBytes(staged));
        try {
            SettlePointCloudMutation();
        } catch (...) {
            for (auto& pendingBase : pendingBaseGenerations) {
                DestroyPointDescriptorGeneration(
                    &pendingBase.generation);
            }
            DestroyPointCloudResources(&staged);
            throw;
        }
        std::swap(*resources, staged);
        for (auto& pendingBase : pendingBaseGenerations) {
            InstallPointDescriptorGeneration(
                &pointCloudResources_[pendingBase.resourceIndex],
                &pendingBase.generation);
        }
        DestroyPointCloudResources(&staged);

        // The replacement owner is now fully installed and every base-cloud
        // descriptor generation points at it. Only at this point may an old
        // scene's simulation be retired; doing this earlier leaves base
        // descriptors borrowing dead contact buffers during cache changes.
        std::vector<std::size_t> staleMeshFlowLayerIds;
        for (const auto& candidate : pointCloudResources_) {
            if (candidate.layerId == layerId ||
                candidate.dynamicMeshFlowAllocationRevision == 0U) {
                continue;
            }
            if (candidate.dynamicMeshFlowGroundUploadRevision !=
                    groundView.uploadRevision ||
                candidate.dynamicMeshFlowGroundIdentity !=
                    *groundView.cacheIdentity) {
                staleMeshFlowLayerIds.push_back(candidate.layerId);
            }
        }
        for (const auto staleLayerId : staleMeshFlowLayerIds) {
            ClearDynamicMeshFlowGpuSimulation(staleLayerId);
        }
        resources = FindPointCloudResources(layerId);
        if (resources == nullptr) {
            throw std::runtime_error{
                "Installed Mesh Flow resources disappeared during stale-owner retirement."};
        }
        result.allocatedThisUpdate = true;
        result.allocationMilliseconds = MillisecondsBetween(
            allocationStartedAt,
            std::chrono::steady_clock::now());
    }

    if (request.resetSimulation) {
        ++resources->dynamicMeshFlowResetEpoch;
        if (resources->dynamicMeshFlowResetEpoch == 0U) {
            resources->dynamicMeshFlowResetEpoch = 1U;
        }
    }

    const auto parameterStartedAt = std::chrono::steady_clock::now();
    const auto liveSlot =
        resources->dynamicMeshFlowNextLiveSlot % kDynamicMeshFlowLiveSlots;
    resources->dynamicMeshFlowNextLiveSlot =
        (resources->dynamicMeshFlowNextLiveSlot + 1U) %
        kDynamicMeshFlowLiveSlots;
    PrepareDynamicMeshFlowDispatchSlot(resources, liveSlot);

    if (resources->dynamicMeshFlowCounterBuffers[liveSlot].mapped != nullptr) {
        DynamicMeshFlowCountersGpu completedCounters;
        std::memcpy(
            &completedCounters,
            resources->dynamicMeshFlowCounterBuffers[liveSlot].mapped,
            sizeof(completedCounters));
        resources->dynamicMeshFlowLastActiveParticleCount =
            std::min(completedCounters.activeParticleCount, particleCapacity);
        resources->dynamicMeshFlowLastContactEventCount =
            std::min(
                completedCounters.contactEventCount,
                resources->dynamicMeshFlowEventCapacity);
        resources->dynamicMeshFlowLastRainSeedCount =
            std::min(
                completedCounters.availableRainSeedCount,
                kDynamicMeshFlowRainSeedCapacity);
        resources->dynamicMeshFlowLastRainSeedParticleCount =
            std::min(
                completedCounters.rainSeedParticleCount,
                particleCapacity);
        resources->dynamicMeshFlowLastRouteSampleCount =
            completedCounters.routeSampleCount;
        resources->dynamicMeshFlowLastRouteWithinBoundsCount =
            std::min(
                completedCounters.routeWithinBoundsCount,
                completedCounters.routeSampleCount);
        resources->dynamicMeshFlowLastRouteNearSurfaceCount =
            std::min(
                completedCounters.routeNearSurfaceCount,
                completedCounters.routeSampleCount);
        resources->dynamicMeshFlowLastLongSegmentCount =
            completedCounters.longSegmentCount;
        resources->dynamicMeshFlowLastVerticalJumpCount =
            completedCounters.verticalJumpCount;
        const float completedMaximumSegment =
            std::bit_cast<float>(
                completedCounters.maximumSegmentBits);
        resources->dynamicMeshFlowLastMaximumSegmentMeters =
            std::isfinite(completedMaximumSegment)
                ? std::max(0.0F, completedMaximumSegment)
                : 0.0F;
        resources->dynamicMeshFlowLastAlignmentHistogram =
            completedCounters.alignmentHistogram;
    }

    DynamicMeshFlowUniformsGpu uniforms;
    uniforms.capacities = glm::uvec4{
        particleCapacity,
        historyLength,
        pointCount,
        resources->dynamicMeshFlowEventCapacity,
    };
    uniforms.ground = glm::uvec4{
        groundView.tableMask,
        std::max(1U, groundView.maximumProbeCount),
        groundView.occupiedCellCount,
        request.settings.seed,
    };
    uniforms.control = glm::uvec4{
        resources->dynamicMeshFlowResetEpoch,
        request.settings.showTrails ? 1U : 0U,
        groundView.entryCandidateCount,
        rainResources_.resetEpoch,
    };
    const float activity =
        request.settings.enabled
            ? std::clamp(
                  std::isfinite(request.activity) ? request.activity : 0.0F,
                  0.0F,
                  1.0F)
            : 0.0F;
    const float moisture = std::clamp(
        std::isfinite(request.moisture) ? request.moisture : 0.0F,
        0.0F,
        1.0F);
    const float populationFraction =
        activity * activity *
        std::lerp(0.125F, 0.50F, moisture);
    uniforms.timing = glm::vec4{
        std::isfinite(request.timeSeconds) ? request.timeSeconds : 0.0F,
        std::clamp(
            std::isfinite(request.deltaSeconds)
                ? request.deltaSeconds
                : 1.0F / 30.0F,
            0.0F,
            0.10F),
        activity,
        moisture,
    };
    // Contact events splat into every bucket their radius overlaps, so the
    // cell size sets activation granularity, not reach. Consumers derive
    // table lengths from SSBO runtime arrays and can hash contacts without
    // another mutable control/uniform binding. Must match
    // kMeshFlowContactCellSizeMeters in pointcloud_mesh_flow_contact.glsl.
    constexpr float kContactGridCellSizeMeters = 0.25F;
    const float contactGridCellSize = kContactGridCellSizeMeters;
    resources->dynamicMeshFlowContactGridCellSizeMeters =
        contactGridCellSize;
    uniforms.contactGrid = glm::uvec4{
        resources->dynamicMeshFlowContactGridMask,
        resources->dynamicMeshFlowContactGridMask + 1U,
        std::bit_cast<std::uint32_t>(
            std::clamp(request.settings.surfaceSurge, 0.0F, 1.0F)),
        0U,
    };
    uniforms.boundsMinimumResolution = glm::vec4{
        groundView.bounds.minimum.x,
        groundView.bounds.minimum.y,
        groundView.bounds.minimum.z,
        std::max(0.001F, groundView.resolutionMeters),
    };
    uniforms.boundsMaximumBand = glm::vec4{
        groundView.bounds.maximum.x,
        groundView.bounds.maximum.y,
        groundView.bounds.maximum.z,
        std::clamp(request.settings.edgeCoverage, 0.0F, 1.0F),
    };
    uniforms.motion0 = glm::vec4{
        std::max(0.001F, request.settings.speedMetersPerSecond),
        std::max(0.0F, request.settings.downhillWeight),
        std::clamp(request.settings.inertia, 0.0F, 0.98F),
        std::clamp(request.settings.surfaceOffsetMeters, -0.05F, 0.10F),
    };
    uniforms.particleNoise = glm::vec4{
        std::max(0.0F, request.settings.particleNoiseStrength),
        std::max(0.001F, request.settings.particleNoiseScaleMeters),
        std::max(0.0F, request.settings.particleNoiseSpeed),
        std::clamp(request.settings.dryConcavityFocus, 0.0F, 2.0F),
    };
    uniforms.sharedWind = glm::vec4{
        std::max(0.0F, request.settings.sharedWindStrength),
        std::max(0.001F, request.settings.sharedWindScaleMeters),
        std::max(0.0F, request.settings.sharedWindSpeed),
        std::clamp(request.settings.rainSpawnSpread, 0.0F, 2.0F),
    };
    uniforms.visual = glm::vec4{
        std::max(0.0005F, request.settings.trailWidthMeters),
        std::max(0.001F, request.settings.trailStreakLengthMeters),
        invisible_places::water::kWaterTrailFeatureTypeDynamicMesh,
        std::max(0.01F, request.settings.contactFadeSeconds),
    };
    uniforms.responsePersistence = glm::vec4{
        std::max(
            request.settings.contactFadeSeconds,
            request.settings.rockResponse.persistenceSeconds),
        std::max(
            request.settings.contactFadeSeconds,
            request.settings.vegetationResponse.persistenceSeconds),
        contactGridCellSize,
        std::clamp(request.settings.trailWetnessFloor, 0.0F, 1.0F),
    };
    uniforms.rockResponse = glm::vec4{
        std::max(0.0F, request.settings.rockResponse.radiusMeters),
        std::max(0.0F, request.settings.rockResponse.opacityAdd),
        std::max(0.0F, request.settings.rockResponse.emissionAdd),
        std::max(0.01F, request.settings.rockResponse.persistenceSeconds),
    };
    uniforms.rockColour = glm::vec4{
        request.settings.rockResponse.colourise.x,
        request.settings.rockResponse.colourise.y,
        request.settings.rockResponse.colourise.z,
        request.settings.rockResponse.colouriseAmount,
    };
    uniforms.vegetationResponse = glm::vec4{
        std::max(0.0F, request.settings.vegetationResponse.radiusMeters),
        std::max(0.0F, request.settings.vegetationResponse.opacityAdd),
        std::max(0.0F, request.settings.vegetationResponse.emissionAdd),
        std::max(
            0.01F,
            request.settings.vegetationResponse.persistenceSeconds),
    };
    uniforms.vegetationColour = glm::vec4{
        request.settings.vegetationResponse.colourise.x,
        request.settings.vegetationResponse.colourise.y,
        request.settings.vegetationResponse.colourise.z,
        request.settings.vegetationResponse.colouriseAmount,
    };
    uniforms.vegetationExtra = glm::vec4{
        std::max(0.0F, request.settings.vegetationResponse.twinkle),
        std::max(
            0.0F,
            request.settings.vegetationResponse.streamDepthMeters),
        std::clamp(request.settings.contactUpwardReachMeters, 0.0F, 3.0F),
        std::clamp(
            request.settings.rainDistributedSourceFraction,
            0.0F,
            1.0F),
    };

    const DynamicMeshFlowCountersGpu emptyCounters{};
    UploadBufferData(
        resources->dynamicMeshFlowUniformBuffers[liveSlot],
        &uniforms,
        sizeof(uniforms));
    UploadBufferData(
        resources->dynamicMeshFlowCounterBuffers[liveSlot],
        &emptyCounters,
        sizeof(emptyCounters));
    ++resources->dynamicMeshFlowParameterRevision;
    result.parameterUploadMilliseconds = MillisecondsBetween(
        parameterStartedAt,
        std::chrono::steady_clock::now());

    const auto dispatchStartedAt = std::chrono::steady_clock::now();
    DispatchDynamicMeshFlowCompute(resources, particleCapacity, liveSlot);
    ++resources->dynamicMeshFlowDispatchCount;
    result.dispatchMilliseconds = MillisecondsBetween(
        dispatchStartedAt,
        std::chrono::steady_clock::now());

    ++sceneRevision_;
    result.pointCount = pointCount;
    result.particleCapacity = particleCapacity;
    result.historyLength = historyLength;
    // Expose the deterministic hashed population budget, not a transient
    // count of particles currently between their staggered birth/fade states.
    // The latter legitimately oscillates as short routes recycle and made the
    // UI/smoke appear to flash even though the stable eligible set did not.
    result.activeParticles =
        static_cast<std::uint32_t>(
            std::floor(
                static_cast<float>(particleCapacity) *
                populationFraction));
    result.contactEvents = resources->dynamicMeshFlowLastContactEventCount;
    result.entryCandidateCount = groundView.entryCandidateCount;
    result.rainSeedCount =
        resources->dynamicMeshFlowLastRainSeedCount;
    result.usedRainSeedParticleCount =
        resources->dynamicMeshFlowLastRainSeedParticleCount;
    result.routeSampleCount =
        resources->dynamicMeshFlowLastRouteSampleCount;
    if (result.routeSampleCount > 0U) {
        const float routeSampleCount =
            static_cast<float>(result.routeSampleCount);
        result.routeWithinGroundBoundsFraction =
            static_cast<float>(
                resources->
                    dynamicMeshFlowLastRouteWithinBoundsCount) /
            routeSampleCount;
        result.routeWithinSurfaceToleranceFraction =
            static_cast<float>(
                resources->
                    dynamicMeshFlowLastRouteNearSurfaceCount) /
            routeSampleCount;
    }
    result.maximumRenderedSegmentMeters =
        resources->dynamicMeshFlowLastMaximumSegmentMeters;
    result.longSegmentCount =
        resources->dynamicMeshFlowLastLongSegmentCount;
    result.unexplainedVerticalJumpCount =
        resources->dynamicMeshFlowLastVerticalJumpCount;
    const auto alignmentSampleCount = std::accumulate(
        resources->dynamicMeshFlowLastAlignmentHistogram.begin(),
        resources->dynamicMeshFlowLastAlignmentHistogram.end(),
        std::uint64_t{0U});
    if (alignmentSampleCount > 0U) {
        const auto medianRank =
            (alignmentSampleCount + 1U) / 2U;
        std::uint64_t cumulative = 0U;
        for (std::size_t binIndex = 0U;
             binIndex <
                 resources->
                     dynamicMeshFlowLastAlignmentHistogram.size();
             ++binIndex) {
            cumulative +=
                resources->
                    dynamicMeshFlowLastAlignmentHistogram[binIndex];
            if (cumulative >= medianRank) {
                result.medianTangentDownhillAlignment =
                    (static_cast<float>(binIndex) + 0.5F) /
                    static_cast<float>(
                        resources->
                            dynamicMeshFlowLastAlignmentHistogram.size());
                break;
            }
        }
    }
    result.allocationRevision =
        resources->dynamicMeshFlowAllocationRevision;
    result.parameterRevision =
        resources->dynamicMeshFlowParameterRevision;
    result.descriptorGeneration =
        resources->dynamicMeshFlowDescriptorGeneration;
    result.dispatchCount = resources->dynamicMeshFlowDispatchCount;
    result.parametersOnly = !result.allocatedThisUpdate;
    result.asynchronousDispatch = true;
    return result;
}

DynamicMeshFlowContactGpuView
VulkanViewportShell::DynamicMeshFlowContactView(
    std::size_t layerId) const {
    const auto* resources = FindPointCloudResources(layerId);
    if (resources == nullptr ||
        !rainResources_.collisionReady ||
        resources->dynamicMeshFlowGroundUploadRevision !=
            rainResources_.groundResidentUploadRevision ||
        resources->dynamicMeshFlowGroundIdentity !=
            rainResources_.collisionCacheIdentity) {
        return {};
    }
    return {
        .eventBuffer =
            resources->dynamicMeshFlowContactEventBuffer.buffer,
        .eventOffset = 0U,
        .eventRange =
            resources->dynamicMeshFlowContactEventBuffer.size,
        .gridBuffer =
            resources->dynamicMeshFlowContactGridBuffer.buffer,
        .gridOffset = 0U,
        .gridRange =
            resources->dynamicMeshFlowContactGridBuffer.size,
        .eventCapacity = resources->dynamicMeshFlowEventCapacity,
        .gridMask = resources->dynamicMeshFlowContactGridMask,
        .gridCellSizeMeters =
            resources->dynamicMeshFlowContactGridCellSizeMeters,
        .allocationRevision =
            resources->dynamicMeshFlowAllocationRevision,
        .descriptorGeneration =
            resources->dynamicMeshFlowDescriptorGeneration,
        .valid =
            resources->dynamicMeshFlowAllocationRevision != 0U &&
            resources->dynamicMeshFlowDescriptorGeneration != 0U &&
            resources->dynamicMeshFlowContactEventBuffer.buffer !=
                VK_NULL_HANDLE &&
            resources->dynamicMeshFlowContactGridBuffer.buffer !=
                VK_NULL_HANDLE,
    };
}

void VulkanViewportShell::ClearDynamicMeshFlowGpuSimulation(
    std::size_t layerId) {
    const auto found = std::find_if(
        pointCloudResources_.begin(),
        pointCloudResources_.end(),
        [layerId](const ActivePointCloudResources& resources) {
            return resources.layerId == layerId &&
                   resources.dynamicMeshFlowAllocationRevision != 0U;
        });
    if (found == pointCloudResources_.end()) {
        return;
    }

    struct PendingBaseContactGeneration {
        std::size_t resourceIndex = 0U;
        PendingPointDescriptorGeneration generation{};
    };
    std::vector<PendingBaseContactGeneration> pendingBaseGenerations;
    DynamicMeshFlowContactGpuView replacementContactView{
        .eventBuffer =
            rainResources_
                .dynamicMeshFlowDummyContactEventBuffer.buffer,
        .eventOffset = 0U,
        .eventRange =
            rainResources_
                .dynamicMeshFlowDummyContactEventBuffer.size,
        .gridBuffer =
            rainResources_
                .dynamicMeshFlowDummyContactGridBuffer.buffer,
        .gridOffset = 0U,
        .gridRange =
            rainResources_
                .dynamicMeshFlowDummyContactGridBuffer.size,
        .eventCapacity = 1U,
        .gridMask = 0U,
        .gridCellSizeMeters = 0.75F,
        .valid = false,
    };
    for (const auto& candidate : pointCloudResources_) {
        if (&candidate == &(*found) ||
            candidate.dynamicMeshFlowAllocationRevision == 0U) {
            continue;
        }
        const auto candidateView =
            PointDescriptorDynamicMeshFlowContactView(&candidate);
        if (candidateView.valid) {
            replacementContactView = candidateView;
            break;
        }
    }
    pointDescriptorDynamicMeshFlowContactOverride_ =
        replacementContactView;
    try {
        for (std::size_t resourceIndex = 0U;
             resourceIndex < pointCloudResources_.size();
             ++resourceIndex) {
            auto& candidate = pointCloudResources_[resourceIndex];
            if (&candidate == &(*found) ||
                candidate.dynamicMeshFlowAllocationRevision != 0U ||
                candidate.pointDescriptorPool == VK_NULL_HANDLE) {
                continue;
            }
            PendingBaseContactGeneration pendingBase;
            pendingBase.resourceIndex = resourceIndex;
            pendingBase.generation = BuildPointDescriptorGeneration(
                &candidate,
                exrExportResources_.depthImage.view,
                exrExportResources_.uniformBuffer.buffer);
            pendingBaseGenerations.push_back(
                std::move(pendingBase));
        }
    } catch (...) {
        pointDescriptorDynamicMeshFlowContactOverride_.reset();
        for (auto& pendingBase : pendingBaseGenerations) {
            DestroyPointDescriptorGeneration(
                &pendingBase.generation);
        }
        throw;
    }
    pointDescriptorDynamicMeshFlowContactOverride_.reset();
    try {
        SettlePointCloudMutation();
    } catch (...) {
        for (auto& pendingBase : pendingBaseGenerations) {
            DestroyPointDescriptorGeneration(
                &pendingBase.generation);
        }
        throw;
    }
    for (auto& pendingBase : pendingBaseGenerations) {
        InstallPointDescriptorGeneration(
            &pointCloudResources_[pendingBase.resourceIndex],
            &pendingBase.generation);
    }
    DestroyPointCloudResources(&(*found));
    pointCloudResources_.erase(found);
    ++sceneRevision_;
}

WaterFlowGpuSourceUploadResult VulkanViewportShell::UploadWaterFlowGpuSource(
    std::size_t layerId,
    const WaterFlowGpuSourceRequest& request) {
    WaterFlowGpuSourceUploadResult result;
    if (waterFlowRouteComputePipeline_ == VK_NULL_HANDLE ||
        waterFlowTrailComputePipeline_ == VK_NULL_HANDLE ||
        waterFlowSourcePipelineLayout_ == VK_NULL_HANDLE ||
        waterFlowSourceDescriptorSetLayout_ == VK_NULL_HANDLE) {
        return result;
    }
    // New cache preprocessing is submitted on the same queue. Deferring guided
    // requests here guarantees every dispatch using the old immutable table is
    // ordered before that table can enter the cache retirement path.
    if (request.useSurfaceGuide && WaterSurfaceUploadPending()) {
        result.diagnostics = WaterFlowGpuSourceState(layerId);
        result.diagnostics.pending = true;
        return result;
    }

    auto compactSourceInput = request.compactSourceInput;
    if (compactSourceInput == nullptr || !compactSourceInput->Valid()) {
        auto builtInput =
            request.inputKind ==
                    invisible_places::water::WaterFlowGpuInputKind::ManualCatmullRomControlPoints
                ? invisible_places::water::BuildWaterFlowGpuManualSplineSourceInput(
                      request.controlPoints,
                      request.sourceId,
                      1U)
                : invisible_places::water::BuildWaterFlowGpuSampledSourceInput(
                      request.sampledAnchors,
                      request.settings,
                      nullptr);
        compactSourceInput =
            std::make_shared<const invisible_places::water::WaterFlowGpuCompactSourceInput>(
                std::move(builtInput));
    }
    if (!compactSourceInput->Valid()) {
        return result;
    }
    std::vector<WaterFlowInputPointGpu> inputPoints;
    inputPoints.reserve(compactSourceInput->points.size());
    for (const auto& sourcePoint : compactSourceInput->points) {
        WaterFlowInputPointGpu point;
        point.positionDistance = glm::vec4{
            sourcePoint.position.x,
            sourcePoint.position.y,
            sourcePoint.position.z,
            sourcePoint.cumulativeDistanceMeters,
        };
        point.normalConfidence = glm::vec4{
            sourcePoint.normal.x,
            sourcePoint.normal.y,
            sourcePoint.normal.z,
            sourcePoint.confidence,
        };
        point.outgoingArcDistances = glm::vec4{
            sourcePoint.outgoingSegmentArcDistancesMeters[0U],
            sourcePoint.outgoingSegmentArcDistancesMeters[1U],
            sourcePoint.outgoingSegmentArcDistancesMeters[2U],
            sourcePoint.outgoingSegmentArcDistancesMeters[3U],
        };
        point.laneWidth = glm::vec4{
            static_cast<float>(sourcePoint.laneWidth.mode),
            sourcePoint.laneWidth.value,
            0.0F,
            0.0F,
        };
        inputPoints.push_back(point);
    }

    auto* resources = FindPointCloudResources(layerId);
    const std::uint32_t spareCapacity =
        resources != nullptr && resources->waterFlowSourceActive
            ? resources->waterFlowSparePointCapacity
            : 0U;
    result.layout = invisible_places::water::BuildWaterFlowGpuOutputLayout(
        *compactSourceInput,
        request.settings,
        spareCapacity);
    if (!result.layout.Valid()) {
        return result;
    }
    VkPhysicalDeviceProperties physicalDeviceProperties{};
    vkGetPhysicalDeviceProperties(physicalDevice_, &physicalDeviceProperties);
    if (result.layout.branchCount >
        physicalDeviceProperties.limits.maxComputeWorkGroupCount[1]) {
        return result;
    }

    const std::uint64_t newestRevision = std::max({
        resources != nullptr ? resources->waterFlowSourceRequestedRevision : 0U,
        resources != nullptr ? resources->waterFlowPendingRevision : 0U,
        resources != nullptr ? resources->waterFlowQueuedRevision : 0U,
    });
    if (resources != nullptr && resources->waterFlowSourceActive &&
        request.sourceRevision <= newestRevision) {
        result.diagnostics = WaterFlowGpuSourceState(layerId);
        return result;
    }

    if (resources != nullptr && resources->waterFlowSourceActive &&
        resources->waterFlowSourceDispatchPending) {
        resources->waterFlowSourceRequestedRevision = request.sourceRevision;
        resources->waterFlowSourceId = request.sourceId;
        resources->waterFlowDeletePending = false;
        resources->waterFlowDeleteOutstandingFrameMask = 0U;
        resources->waterFlowQueuedInputKind = request.inputKind;
        resources->waterFlowQueuedCompactSourceInput = std::move(compactSourceInput);
        resources->waterFlowQueuedSettings = request.settings;
        resources->waterFlowQueuedRevision = request.sourceRevision;
        resources->waterFlowQueuedSourceId = request.sourceId;
        resources->waterFlowQueuedUseSurfaceGuide = request.useSurfaceGuide;
        resources->waterFlowSourceQueued = true;
        result.accepted = true;
        result.asynchronousDispatch = true;
        result.diagnostics = WaterFlowGpuSourceState(layerId);
        return result;
    }

    const std::uint32_t requiredInputCapacity = std::bit_ceil(std::max<std::uint32_t>(
        2U,
        static_cast<std::uint32_t>(inputPoints.size())));
    std::vector<WaterFlowBranchGpu> gpuBranches;
    gpuBranches.reserve(result.layout.branches.size());
    for (const auto& branch : result.layout.branches) {
        WaterFlowBranchGpu gpuBranch;
        gpuBranch.inputRoute = glm::uvec4{
            branch.inputStart,
            branch.inputCount,
            branch.routeStart,
            branch.routePointsPerLane,
        };
        gpuBranch.trailLane = glm::uvec4{
            branch.activeRouteLaneCount,
            branch.trailOutputStart,
            branch.trailCount,
            branch.firstTrailId,
        };
        gpuBranch.identity = glm::uvec4{
            branch.potentialLaneCount,
            branch.branchId,
            branch.pathId,
            static_cast<std::uint32_t>(request.inputKind),
        };
        gpuBranch.metrics = glm::vec4{
            branch.routeLengthMeters,
            branch.routeSpacingMeters,
            0.0F,
            0.0F,
        };
        gpuBranches.push_back(gpuBranch);
    }
    const std::uint32_t requiredBranchCapacity = std::bit_ceil(
        std::max<std::uint32_t>(1U, static_cast<std::uint32_t>(gpuBranches.size())));

    const auto surfaceView = WaterSurfaceFlowView();
    const bool useSurfaceGuide =
        request.useSurfaceGuide && surfaceView.valid && surfaceView.preprocessingComplete;
    WaterFlowSourceUniformsGpu uniforms;
    uniforms.counts0 = glm::uvec4{
        result.layout.inputPointCount,
        result.layout.branchCount,
        result.layout.maxActiveRouteLaneCount,
        result.layout.routePointCountTotal,
    };
    uniforms.counts1 = glm::uvec4{
        result.layout.trailCount,
        result.layout.samplesPerTrail,
        result.layout.pointCount,
        result.layout.pointCapacity,
    };
    uniforms.surface0 = glm::uvec4{
        surfaceView.tableMask,
        std::max(1U, surfaceView.maximumProbeCount),
        useSurfaceGuide ? 1U : 0U,
        request.sourceId,
    };
    uniforms.metadata = glm::uvec4{
        request.settings.seed,
        static_cast<std::uint32_t>(request.inputKind),
        useSurfaceGuide ? 1U : 0U,
        static_cast<std::uint32_t>(request.sourceRevision),
    };
    const std::uint32_t endpointFadeFlags =
        (request.settings.startFadeEnabled ? 1U : 0U) |
        (request.settings.endFadeEnabled ? 2U : 0U);
    uniforms.identity = glm::uvec4{request.sourceId, endpointFadeFlags, 0U, 0U};
    uniforms.route0 = glm::vec4{
        result.layout.routeLengthMeters,
        std::max(0.001F, request.settings.trailPointSpacingMeters),
        surfaceView.valid
            ? surfaceView.resolutionMeters
            : invisible_places::water::kWaterSurfaceResolutionMeters,
        std::isfinite(request.settings.surfaceOffsetMeters)
            ? request.settings.surfaceOffsetMeters
            : 0.004F,
    };
    uniforms.lane0 = glm::vec4{
        std::max(0.0F, request.settings.laneSpreadMeters),
        std::max(0.0005F, request.settings.trailWidthMeters),
        std::clamp(request.settings.turbulence, 0.0F, 1.0F),
        std::max(0.005F, request.settings.turbulenceScaleMeters),
    };
    uniforms.guide0 = glm::vec4{
        std::clamp(request.settings.surfaceFollow, 0.0F, 1.0F),
        std::clamp(request.settings.downhillPull, 0.0F, 1.0F),
        std::clamp(request.settings.terrainWidthResponse, 0.0F, 1.0F),
        std::clamp(request.settings.pathAttraction, 0.0F, 1.0F),
    };
    uniforms.trail0 = glm::vec4{
        std::max(0.02F, request.settings.trailLengthMeters),
        std::max(0.001F, request.settings.trailPointSpacingMeters),
        std::max(0.0F, request.settings.speedMetersPerSecond),
        std::max(0.001F, request.settings.trailStreakLengthMeters),
    };
    uniforms.shape0 = glm::vec4{
        std::clamp(request.settings.laneCrossing, 0.0F, 1.0F),
        std::clamp(request.settings.trailSmoothness, 0.0F, 1.0F),
        std::clamp(request.settings.trailLooseness, 0.0F, 1.0F),
        0.0F,
    };
    uniforms.fade0 = glm::vec4{
        std::clamp(request.settings.startFadeFullDistanceMeters, 0.0F, 50.0F),
        std::clamp(
            request.settings.startFadeRandomBeginDistanceMeters,
            0.0F,
            50.0F),
        std::clamp(request.settings.endFadeFullDistanceMeters, 0.0F, 50.0F),
        std::clamp(
            request.settings.endFadeRandomBeginDistanceMeters,
            0.0F,
            50.0F),
    };

    const bool reusedOutputCapacity =
        resources != nullptr && resources->waterFlowSourceActive &&
        resources->waterFlowSparePointCapacity >= result.layout.pointCapacity &&
        resources->waterFlowPendingPositionBuffer.buffer != VK_NULL_HANDLE &&
        resources->waterFlowPendingPositionStorageBuffer.buffer != VK_NULL_HANDLE &&
        resources->waterFlowPendingColorBuffer.buffer != VK_NULL_HANDLE &&
        resources->waterFlowPendingNormalBuffer.buffer != VK_NULL_HANDLE &&
        resources->waterFlowPendingScalarFieldBuffer.buffer != VK_NULL_HANDLE;

    // Build the entire compute generation off to the side. No published Flow
    // state or descriptor binding changes until every allocation, upload,
    // descriptor write, command recording, and queue submission succeeds.
    ActivePointCloudResources staging;
    staging.layerId = layerId;
    staging.waterFlowSparePointCapacity = result.layout.pointCapacity;
    staging.waterFlowSourceInputCapacity = requiredInputCapacity;
    staging.waterFlowSourceBranchCapacity = requiredBranchCapacity;
    bool stagingBorrowsSpareOutput = false;
    if (reusedOutputCapacity) {
        staging.waterFlowPendingPositionBuffer = resources->waterFlowPendingPositionBuffer;
        staging.waterFlowPendingPositionStorageBuffer =
            resources->waterFlowPendingPositionStorageBuffer;
        staging.waterFlowPendingColorBuffer = resources->waterFlowPendingColorBuffer;
        staging.waterFlowPendingNormalBuffer = resources->waterFlowPendingNormalBuffer;
        staging.waterFlowPendingScalarFieldBuffer =
            resources->waterFlowPendingScalarFieldBuffer;
        stagingBorrowsSpareOutput = true;
    }

    const auto clearBorrowedStagingOutput = [&]() {
        if (!stagingBorrowsSpareOutput) {
            return;
        }
        staging.waterFlowPendingPositionBuffer = {};
        staging.waterFlowPendingPositionStorageBuffer = {};
        staging.waterFlowPendingColorBuffer = {};
        staging.waterFlowPendingNormalBuffer = {};
        staging.waterFlowPendingScalarFieldBuffer = {};
    };
    try {
        if (!reusedOutputCapacity) {
            const VkDeviceSize capacity = result.layout.pointCapacity;
            staging.waterFlowPendingPositionBuffer = CreateDeviceLocalBuffer(
                capacity * sizeof(invisible_places::io::Float3),
                VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
            staging.waterFlowPendingPositionStorageBuffer = CreateDeviceLocalBuffer(
                capacity * sizeof(glm::vec4),
                VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
            if (smokeFailNextWaterFlowUploadAfterPartialAllocation_) {
                smokeFailNextWaterFlowUploadAfterPartialAllocation_ = false;
                throw std::runtime_error{
                    "Injected Water Flow upload failure after partial allocation."};
            }
            staging.waterFlowPendingNormalBuffer = CreateDeviceLocalBuffer(
                capacity * sizeof(glm::vec4),
                VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
            staging.waterFlowPendingScalarFieldBuffer = CreateDeviceLocalBuffer(
                capacity * invisible_places::water::kWaterTrailScalarFieldCount * sizeof(float),
                VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
            staging.waterFlowPendingColorBuffer = CreateHostVisibleBuffer(
                capacity * sizeof(std::uint32_t),
                VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
            const std::uint32_t waterColor = static_cast<std::uint32_t>(190U) |
                                             (static_cast<std::uint32_t>(230U) << 8U) |
                                             (static_cast<std::uint32_t>(255U) << 16U) |
                                             (0xFFU << 24U);
            auto* packedColors =
                static_cast<std::uint32_t*>(staging.waterFlowPendingColorBuffer.mapped);
            if (packedColors != nullptr) {
                std::fill_n(packedColors, result.layout.pointCapacity, waterColor);
                staging.waterFlowSourceBytesTransferred +=
                    static_cast<std::uint64_t>(result.layout.pointCapacity) *
                    sizeof(std::uint32_t);
            }
        }

        staging.waterFlowSourceInputBuffer = CreateHostVisibleBuffer(
            static_cast<VkDeviceSize>(requiredInputCapacity) * sizeof(WaterFlowInputPointGpu),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        if (reusedOutputCapacity && smokeFailNextWaterFlowUploadAfterPartialAllocation_) {
            // The failure hook remains deterministic when an existing spare
            // output generation is large enough for this request.
            smokeFailNextWaterFlowUploadAfterPartialAllocation_ = false;
            throw std::runtime_error{
                "Injected Water Flow upload failure after partial allocation."};
        }
        staging.waterFlowSourceBranchBuffer = CreateHostVisibleBuffer(
            static_cast<VkDeviceSize>(requiredBranchCapacity) * sizeof(WaterFlowBranchGpu),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        staging.waterFlowSourceUniformBuffer = CreateHostVisibleBuffer(
            sizeof(WaterFlowSourceUniformsGpu),
            VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT);
        CreateWaterFlowDummyPointResources(&staging);

        UploadBufferData(
            staging.waterFlowSourceInputBuffer,
            inputPoints.data(),
            static_cast<VkDeviceSize>(inputPoints.size() * sizeof(WaterFlowInputPointGpu)));
        UploadBufferData(
            staging.waterFlowSourceBranchBuffer,
            gpuBranches.data(),
            static_cast<VkDeviceSize>(gpuBranches.size() * sizeof(WaterFlowBranchGpu)));
        UploadBufferData(staging.waterFlowSourceUniformBuffer, &uniforms, sizeof(uniforms));
        staging.waterFlowSourceBytesTransferred +=
            static_cast<std::uint64_t>(inputPoints.size() * sizeof(WaterFlowInputPointGpu)) +
            static_cast<std::uint64_t>(gpuBranches.size() * sizeof(WaterFlowBranchGpu)) +
            sizeof(uniforms);

        // Unguided sources bind their private input allocation as the harmless
        // table dummy, so the descriptor generation never retains an unrelated
        // scene-cache buffer across a cache swap.
        UpdateWaterFlowSourceDescriptorSet(
            &staging,
            useSurfaceGuide ? surfaceView : WaterSurfaceFlowGpuView{});
        DispatchWaterFlowSourceCompute(&staging, result.layout);
    } catch (...) {
        clearBorrowedStagingOutput();
        DestroyPointCloudResources(&staging);
        throw;
    }

    try {
        if (resources == nullptr) {
            pointCloudResources_.reserve(pointCloudResources_.size() + 1U);
            pointCloudResources_.push_back(ActivePointCloudResources{});
            resources = &pointCloudResources_.back();
            resources->layerId = layerId;
        } else if (!resources->waterFlowSourceActive) {
            // The old layer remains renderable until the replacement compute
            // generation is complete and submitted. Only then is it settled
            // and replaced, preserving it on every fallible staging path.
            CleanupPointCloudResources(resources);
            resources->layerId = layerId;
        } else {
            RetireWaterFlowSourceComputeGeneration(resources);
            DestroyBuffer(&resources->waterFlowSourceUniformBuffer);
            DestroyBuffer(&resources->waterFlowSourceInputBuffer);
            DestroyBuffer(&resources->waterFlowSourceBranchBuffer);
            if (reusedOutputCapacity) {
                resources->waterFlowPendingPositionBuffer = {};
                resources->waterFlowPendingPositionStorageBuffer = {};
                resources->waterFlowPendingColorBuffer = {};
                resources->waterFlowPendingNormalBuffer = {};
                resources->waterFlowPendingScalarFieldBuffer = {};
            } else {
                DestroyBuffer(&resources->waterFlowPendingPositionBuffer);
                DestroyBuffer(&resources->waterFlowPendingPositionStorageBuffer);
                DestroyBuffer(&resources->waterFlowPendingColorBuffer);
                DestroyBuffer(&resources->waterFlowPendingNormalBuffer);
                DestroyBuffer(&resources->waterFlowPendingScalarFieldBuffer);
            }
        }
    } catch (...) {
        clearBorrowedStagingOutput();
        DestroyPointCloudResources(&staging);
        throw;
    }

    const auto moveBuffer = [](BufferAllocation* destination, BufferAllocation* source) {
        std::swap(*destination, *source);
    };
    moveBuffer(
        &resources->waterFlowPendingPositionBuffer,
        &staging.waterFlowPendingPositionBuffer);
    moveBuffer(
        &resources->waterFlowPendingPositionStorageBuffer,
        &staging.waterFlowPendingPositionStorageBuffer);
    moveBuffer(
        &resources->waterFlowPendingColorBuffer,
        &staging.waterFlowPendingColorBuffer);
    moveBuffer(
        &resources->waterFlowPendingNormalBuffer,
        &staging.waterFlowPendingNormalBuffer);
    moveBuffer(
        &resources->waterFlowPendingScalarFieldBuffer,
        &staging.waterFlowPendingScalarFieldBuffer);
    moveBuffer(
        &resources->waterFlowSourceUniformBuffer,
        &staging.waterFlowSourceUniformBuffer);
    moveBuffer(
        &resources->waterFlowSourceInputBuffer,
        &staging.waterFlowSourceInputBuffer);
    moveBuffer(
        &resources->waterFlowSourceBranchBuffer,
        &staging.waterFlowSourceBranchBuffer);
    std::swap(
        resources->waterFlowSourceDescriptorPool,
        staging.waterFlowSourceDescriptorPool);
    std::swap(
        resources->waterFlowSourceDescriptorSet,
        staging.waterFlowSourceDescriptorSet);
    ++resources->waterFlowSourceDescriptorGeneration;
    if (resources->waterFlowSourceDescriptorGeneration == 0U) {
        resources->waterFlowSourceDescriptorGeneration = 1U;
    }
    std::swap(
        resources->waterFlowSourceDispatchFence,
        staging.waterFlowSourceDispatchFence);
    std::swap(
        resources->waterFlowSourceCommandBuffer,
        staging.waterFlowSourceCommandBuffer);
    std::swap(
        resources->waterFlowSourceDispatchPending,
        staging.waterFlowSourceDispatchPending);

    // A newly-created Flow layer adopts the completely staged dummy point
    // effect/style bundle. An existing settled Flow keeps its already-bound
    // immutable bundle; the unused staged copy is released below.
    if (resources->seepageNodeBuffer.buffer == VK_NULL_HANDLE) {
        for (std::size_t frameIndex = 0U; frameIndex < kFramesInFlight; ++frameIndex) {
            moveBuffer(
                &resources->seepageParamsBuffers[frameIndex],
                &staging.seepageParamsBuffers[frameIndex]);
            moveBuffer(
                &resources->styleBuffers[frameIndex],
                &staging.styleBuffers[frameIndex]);
        }
        moveBuffer(&resources->seepageNodeBuffer, &staging.seepageNodeBuffer);
        moveBuffer(&resources->seepageExrParamsBuffer, &staging.seepageExrParamsBuffer);
        moveBuffer(&resources->seepageHashCellBuffer, &staging.seepageHashCellBuffer);
        moveBuffer(
            &resources->seepageNodeReferenceBuffer,
            &staging.seepageNodeReferenceBuffer);
        moveBuffer(&resources->exrStyleBuffer, &staging.exrStyleBuffer);
        resources->pendingSeepageParams = std::move(staging.pendingSeepageParams);
    }

    resources->waterFlowSourceActive = true;
    resources->waterFlowDeletePending = false;
    resources->waterFlowDeleteOutstandingFrameMask = 0U;
    resources->waterFlowSourceId = request.sourceId;
    resources->waterFlowSourceRequestedRevision = request.sourceRevision;
    resources->waterFlowPendingLayout = result.layout;
    resources->waterFlowPendingRevision = request.sourceRevision;
    resources->waterFlowPendingSourceId = request.sourceId;
    resources->waterFlowSparePointCapacity = result.layout.pointCapacity;
    resources->waterFlowSourceInputCapacity = requiredInputCapacity;
    resources->waterFlowSourceBranchCapacity = requiredBranchCapacity;
    resources->waterFlowSourceSurfaceUploadRevision =
        useSurfaceGuide ? surfaceView.uploadRevision : 0U;
    resources->waterFlowSourceUseSurfaceGuide = useSurfaceGuide;
    resources->waterFlowSourceBytesTransferred += staging.waterFlowSourceBytesTransferred;
    ++resources->waterFlowSourceDispatchCount;

    DestroyPointCloudResources(&staging);
    TrackPointCloudResidentPeak();
    ++sceneRevision_;

    result.accepted = true;
    result.reusedOutputCapacity = reusedOutputCapacity;
    result.asynchronousDispatch = true;
    result.diagnostics = WaterFlowGpuSourceState(layerId);
    return result;
}

WaterFlowGpuSourceDiagnostics VulkanViewportShell::WaterFlowGpuSourceState(
    std::size_t layerId) const {
    WaterFlowGpuSourceDiagnostics diagnostics;
    const auto* resources = FindPointCloudResources(layerId);
    if (resources == nullptr || !resources->waterFlowSourceActive) {
        return diagnostics;
    }
    diagnostics.sourceId = resources->waterFlowSourceId;
    diagnostics.requestedRevision = resources->waterFlowSourceRequestedRevision;
    diagnostics.completedRevision = resources->waterFlowSourceCompletedRevision;
    diagnostics.surfaceUploadRevision = resources->waterFlowSourceSurfaceUploadRevision;
    diagnostics.bytesTransferred = resources->waterFlowSourceBytesTransferred;
    diagnostics.computeDescriptorGeneration =
        resources->waterFlowSourceDescriptorGeneration;
    diagnostics.pointDescriptorGeneration = resources->pointDescriptorGeneration;
    diagnostics.residentBytes = PointCloudResourceResidentBytes(*resources);
    diagnostics.computeDispatchCount = resources->waterFlowSourceDispatchCount;
    diagnostics.pointCapacity = std::max(
        resources->pointCount,
        resources->waterFlowPendingLayout.pointCapacity);
    diagnostics.activePointCount = resources->waterFlowSettledPointCount;
    diagnostics.pending =
        resources->waterFlowSourceDispatchPending || resources->waterFlowSourceQueued;
    diagnostics.usingSurfaceGuide = resources->waterFlowSourceUseSurfaceGuide;
    return diagnostics;
}

void VulkanViewportShell::RemoveWaterFlowGpuSource(std::size_t layerId) {
    auto* resources = FindPointCloudResources(layerId);
    if (resources == nullptr || !resources->waterFlowSourceActive) {
        return;
    }
    resources->waterFlowDeletePending = true;
    resources->waterFlowDeleteOutstandingFrameMask =
        (1U << static_cast<std::uint32_t>(kFramesInFlight)) - 1U;
    resources->waterFlowSourceQueued = false;
    resources->waterFlowQueuedRevision = 0U;
    resources->waterFlowQueuedCompactSourceInput.reset();
    resources->waterFlowSettledPointCount = 0U;
    resources->activePointCount = 0U;
    ++sceneRevision_;
}

void VulkanViewportShell::UploadWaterSeepageTopology(
    std::size_t layerId,
    const invisible_places::water::WaterSeepageSpatialGrid& grid) {
    auto* resources = FindPointCloudResources(layerId);
    if (resources == nullptr || resources->pointCount == 0U) {
        throw std::runtime_error{"Cannot upload Seepage topology for an unloaded point cloud."};
    }
    if (grid.nodes.size() > std::numeric_limits<std::uint32_t>::max()) {
        throw std::runtime_error{"Seepage node count exceeds the current 32-bit limit."};
    }
    const auto finitePoint = [](const invisible_places::io::Float3& point) {
        return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
    };
    const auto orderedBounds = [](const invisible_places::io::Bounds3f& bounds) {
        return bounds.minimum.x <= bounds.maximum.x &&
               bounds.minimum.y <= bounds.maximum.y &&
               bounds.minimum.z <= bounds.maximum.z;
    };
    const bool usesConnectedSupport = !grid.supportHashCells.empty() &&
                                      !grid.supportReferences.empty();
    const float selectedCellSize = usesConnectedSupport
                                       ? grid.supportCellSizeMeters
                                       : grid.cellSizeMeters;
    const auto& selectedBounds = usesConnectedSupport
                                     ? grid.supportUnionBounds
                                     : grid.unionBounds;
    if (!grid.nodes.empty() &&
        (!std::isfinite(selectedCellSize) || selectedCellSize <= 0.0F ||
         !selectedBounds.valid || !finitePoint(selectedBounds.minimum) ||
         !finitePoint(selectedBounds.maximum) || !orderedBounds(selectedBounds))) {
        // Keep the previously settled topology active instead of publishing a
        // malformed startup grid into every point-cloud descriptor.
        throw std::runtime_error{"Seepage topology contains invalid bounds or cell spacing."};
    }

    const auto payload = MakeWaterSeepageGpuTopology(grid);
    const auto paramsBufferSize = static_cast<VkDeviceSize>(
        payload.params.size() * sizeof(WaterSeepageNodeParamsGpu));
    std::vector<std::byte> pendingParams(
        static_cast<std::size_t>(paramsBufferSize));
    std::memcpy(
        pendingParams.data(),
        payload.params.data(),
        static_cast<std::size_t>(paramsBufferSize));
    std::vector<std::uint32_t> pendingTopologyNodeIds;
    pendingTopologyNodeIds.reserve(grid.nodes.size());
    for (const auto& node : grid.nodes) {
        pendingTopologyNodeIds.push_back(node.id);
    }

    struct SeepageBufferSet {
        BufferAllocation node{};
        std::array<BufferAllocation, kFramesInFlight> params{};
        BufferAllocation exrParams{};
        BufferAllocation hashCells{};
        BufferAllocation nodeReferences{};
    } pending;
    const auto destroyBufferSet = [this](SeepageBufferSet* buffers) {
        if (buffers == nullptr) {
            return;
        }
        DestroyBuffer(&buffers->node);
        for (auto& paramsBuffer : buffers->params) {
            DestroyBuffer(&paramsBuffer);
        }
        DestroyBuffer(&buffers->exrParams);
        DestroyBuffer(&buffers->hashCells);
        DestroyBuffer(&buffers->nodeReferences);
    };

    try {
        pending.node = CreateHostVisibleBuffer(
            static_cast<VkDeviceSize>(
                payload.nodes.size() * sizeof(WaterSeepageNodeTopologyGpu)),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        UploadBufferData(pending.node, payload.nodes.data(), pending.node.size);
        for (auto& paramsBuffer : pending.params) {
            paramsBuffer = CreateHostVisibleBuffer(
                paramsBufferSize,
                VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
            UploadBufferData(paramsBuffer, payload.params.data(), paramsBufferSize);
        }
        pending.exrParams = CreateHostVisibleBuffer(
            paramsBufferSize,
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        UploadBufferData(
            pending.exrParams,
            payload.params.data(),
            paramsBufferSize);
        pending.hashCells = CreateHostVisibleBuffer(
            static_cast<VkDeviceSize>(
                payload.hashCells.size() * sizeof(WaterSeepageHashCellGpu)),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        UploadBufferData(
            pending.hashCells,
            payload.hashCells.data(),
            pending.hashCells.size);
        pending.nodeReferences = CreateHostVisibleBuffer(
            static_cast<VkDeviceSize>(
                payload.nodeReferences.size() *
                sizeof(invisible_places::water::WaterSeepageSupportReference)),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        UploadBufferData(
            pending.nodeReferences,
            payload.nodeReferences.data(),
            pending.nodeReferences.size);
    } catch (...) {
        destroyBufferSet(&pending);
        throw;
    }

    // Point the staging descriptor generation at the new buffers while all
    // settled descriptors and their old buffers remain alive. If descriptor
    // allocation fails, swap back without changing the visible generation.
    std::swap(resources->seepageNodeBuffer, pending.node);
    std::swap(resources->seepageParamsBuffers, pending.params);
    std::swap(resources->seepageExrParamsBuffer, pending.exrParams);
    std::swap(resources->seepageHashCellBuffer, pending.hashCells);
    std::swap(resources->seepageNodeReferenceBuffer, pending.nodeReferences);
    const auto settledTopologyGeneration = resources->seepageTopologyGeneration;
    resources->seepageTopologyGeneration = settledTopologyGeneration + 1U;
    if (resources->seepageTopologyGeneration == 0U) {
        resources->seepageTopologyGeneration = 1U;
    }
    try {
        ReplacePointCloudDescriptorSets(resources);
    } catch (...) {
        resources->seepageTopologyGeneration = settledTopologyGeneration;
        std::swap(resources->seepageNodeBuffer, pending.node);
        std::swap(resources->seepageParamsBuffers, pending.params);
        std::swap(resources->seepageExrParamsBuffer, pending.exrParams);
        std::swap(resources->seepageHashCellBuffer, pending.hashCells);
        std::swap(
            resources->seepageNodeReferenceBuffer,
            pending.nodeReferences);
        destroyBufferSet(&pending);
        throw;
    }
    // ReplacePointCloudDescriptorSets retires the old descriptor generation;
    // the buffers it referenced can now be destroyed safely.
    destroyBufferSet(&pending);

    resources->pendingSeepageParams = std::move(pendingParams);
    ++resources->seepageParamsGeneration;
    resources->seepageParamsFrameGenerations.fill(
        resources->seepageParamsGeneration);
    resources->seepageParamsExrGeneration =
        resources->seepageParamsGeneration;

    resources->seepageNodeCount = static_cast<std::uint32_t>(grid.nodes.size());
    resources->seepageTopologyNodeIds = std::move(pendingTopologyNodeIds);
    resources->seepageHashCellCapacity =
        resources->seepageNodeCount > 0U && payload.occupiedCellCount > 0U
            ? static_cast<std::uint32_t>(payload.hashCells.size())
            : 0U;
    resources->seepageOccupiedCellCount = payload.occupiedCellCount;
    resources->seepageNodeReferenceCount =
        payload.occupiedCellCount > 0U
            ? static_cast<std::uint32_t>(payload.nodeReferences.size())
            : 0U;
    resources->seepageHashProbeLimit = payload.probeLimit;
    resources->seepageUsesConnectedSupport = payload.usesConnectedSupport;
    resources->seepageCellSizeMeters = std::max(0.001F, payload.cellSizeMeters);
    resources->seepageUnionBounds = payload.unionBounds;
    resources->seepageCanContribute =
        invisible_places::water::WaterSeepageGridHasActiveViewportEffect(grid);
    ++resources->seepageTopologyUploadRevision;
    ++resources->seepageParamsUploadRevision;
    ++sceneRevision_;
}

void VulkanViewportShell::UpdateWaterSeepageParams(
    std::size_t layerId,
    const invisible_places::water::WaterSeepageSpatialGrid& grid) {
    auto* resources = FindPointCloudResources(layerId);
    if (resources == nullptr || resources->pointCount == 0U) {
        throw std::runtime_error{"Cannot update Seepage params for an unloaded point cloud."};
    }
    if (grid.nodes.size() != resources->seepageNodeCount) {
        throw std::runtime_error{"Seepage node count changed without rebuilding topology."};
    }
    for (std::size_t nodeIndex = 0U; nodeIndex < grid.nodes.size(); ++nodeIndex) {
        if (nodeIndex >= resources->seepageTopologyNodeIds.size() ||
            grid.nodes[nodeIndex].id != resources->seepageTopologyNodeIds[nodeIndex]) {
            throw std::runtime_error{"Seepage node order changed without rebuilding topology."};
        }
    }

    std::vector<WaterSeepageNodeParamsGpu> params;
    params.reserve(std::max<std::size_t>(grid.nodes.size(), 1U));
    for (const auto& node : grid.nodes) {
        params.push_back(MakeWaterSeepageNodeParamsGpu(node));
    }
    if (params.empty()) {
        params.push_back(WaterSeepageNodeParamsGpu{});
    }
    const auto expectedSize =
        static_cast<VkDeviceSize>(params.size() * sizeof(WaterSeepageNodeParamsGpu));
    const bool hasExpectedBuffers = std::all_of(
        resources->seepageParamsBuffers.begin(),
        resources->seepageParamsBuffers.end(),
        [expectedSize](const BufferAllocation& buffer) { return buffer.size == expectedSize; });
    if (!hasExpectedBuffers || resources->seepageExrParamsBuffer.size != expectedSize) {
        throw std::runtime_error{"Seepage params buffer has an unexpected size."};
    }
    resources->pendingSeepageParams.resize(static_cast<std::size_t>(expectedSize));
    std::memcpy(
        resources->pendingSeepageParams.data(),
        params.data(),
        static_cast<std::size_t>(expectedSize));
    resources->seepageCanContribute =
        invisible_places::water::WaterSeepageGridHasActiveViewportEffect(grid);
    ++resources->seepageParamsGeneration;
    ++resources->seepageParamsUploadRevision;
    ++sceneRevision_;
}

void VulkanViewportShell::FlushWaterSeepageParamsForFrame(std::size_t frameIndex) {
    if (frameIndex >= kFramesInFlight) {
        return;
    }
    for (auto& resources : pointCloudResources_) {
        if (resources.seepageParamsFrameGenerations[frameIndex] ==
                resources.seepageParamsGeneration ||
            resources.pendingSeepageParams.empty()) {
            continue;
        }
        auto& target = resources.seepageParamsBuffers[frameIndex];
        if (target.size != resources.pendingSeepageParams.size()) {
            throw std::runtime_error{"Seepage frame params buffer has an unexpected size."};
        }
        UploadBufferData(
            target,
            resources.pendingSeepageParams.data(),
            target.size);
        resources.seepageParamsFrameGenerations[frameIndex] =
            resources.seepageParamsGeneration;
    }
}

void VulkanViewportShell::FlushWaterSeepageParamsForExr() {
    for (auto& resources : pointCloudResources_) {
        if (resources.seepageParamsExrGeneration == resources.seepageParamsGeneration ||
            resources.pendingSeepageParams.empty()) {
            continue;
        }
        if (resources.seepageExrParamsBuffer.size != resources.pendingSeepageParams.size()) {
            throw std::runtime_error{"Seepage EXR params buffer has an unexpected size."};
        }
        UploadBufferData(
            resources.seepageExrParamsBuffer,
            resources.pendingSeepageParams.data(),
            resources.seepageExrParamsBuffer.size);
        resources.seepageParamsExrGeneration = resources.seepageParamsGeneration;
    }
}

std::size_t VulkanViewportShell::WaterSeepageNodeCount(std::size_t layerId) const {
    const auto* resources = FindPointCloudResources(layerId);
    return resources != nullptr ? static_cast<std::size_t>(resources->seepageNodeCount) : 0U;
}

std::size_t VulkanViewportShell::WaterSeepageOccupiedCellCount(std::size_t layerId) const {
    const auto* resources = FindPointCloudResources(layerId);
    return resources != nullptr ? static_cast<std::size_t>(resources->seepageOccupiedCellCount) : 0U;
}

std::size_t VulkanViewportShell::WaterSeepageNodeReferenceCount(std::size_t layerId) const {
    const auto* resources = FindPointCloudResources(layerId);
    return resources != nullptr ? static_cast<std::size_t>(resources->seepageNodeReferenceCount) : 0U;
}

std::uint64_t VulkanViewportShell::WaterSeepageResidentBytes(
    std::size_t layerId) const {
    const auto* resources = FindPointCloudResources(layerId);
    if (resources == nullptr) {
        return 0U;
    }
    const auto bufferBytes = [](const BufferAllocation& buffer) {
        return buffer.buffer != VK_NULL_HANDLE
                   ? static_cast<std::uint64_t>(buffer.size)
                   : 0U;
    };
    std::uint64_t bytes = 0U;
    const auto add = [&](const BufferAllocation& buffer) {
        const auto size = bufferBytes(buffer);
        bytes = size >
                        std::numeric_limits<std::uint64_t>::max() - bytes
                    ? std::numeric_limits<std::uint64_t>::max()
                    : bytes + size;
    };
    add(resources->seepageNodeBuffer);
    for (const auto& paramsBuffer : resources->seepageParamsBuffers) {
        add(paramsBuffer);
    }
    add(resources->seepageExrParamsBuffer);
    add(resources->seepageHashCellBuffer);
    add(resources->seepageNodeReferenceBuffer);
    return bytes;
}

std::uint64_t VulkanViewportShell::WaterSeepageTopologyUploadRevision(std::size_t layerId) const {
    const auto* resources = FindPointCloudResources(layerId);
    return resources != nullptr ? resources->seepageTopologyUploadRevision : 0U;
}

std::uint64_t VulkanViewportShell::WaterSeepageParamsUploadRevision(std::size_t layerId) const {
    const auto* resources = FindPointCloudResources(layerId);
    return resources != nullptr ? resources->seepageParamsUploadRevision : 0U;
}

WaterEffectFramePublicationDiagnostics
VulkanViewportShell::WaterSeepageParamsPublicationState(std::size_t layerId) const {
    WaterEffectFramePublicationDiagnostics diagnostics;
    const auto* resources = FindPointCloudResources(layerId);
    if (resources == nullptr) {
        return diagnostics;
    }
    diagnostics.requestedGeneration = resources->seepageParamsGeneration;
    diagnostics.liveFrameGenerations = resources->seepageParamsFrameGenerations;
    diagnostics.exrGeneration = resources->seepageParamsExrGeneration;
    diagnostics.topologyGeneration = resources->seepageTopologyGeneration;
    diagnostics.liveDescriptorGenerations =
        resources->seepageDescriptorGenerations;
    diagnostics.exrDescriptorGeneration =
        resources->seepageExrDescriptorGeneration;
    const auto expectedImageCount = depthImages_.size();
    const auto inspectDescriptorRing =
        [resources, expectedImageCount, &diagnostics](
            const std::array<std::vector<VkDescriptorSet>, kFramesInFlight>& descriptors,
            const std::array<std::vector<std::uint64_t>, kFramesInFlight>& generations) {
            for (std::size_t frameIndex = 0U; frameIndex < kFramesInFlight; ++frameIndex) {
                const auto& frameDescriptors = descriptors[frameIndex];
                const auto& frameGenerations = generations[frameIndex];
                diagnostics.descriptorGenerationMismatchCount +=
                    static_cast<std::uint32_t>(
                        frameDescriptors.size() > expectedImageCount
                            ? frameDescriptors.size() - expectedImageCount
                            : expectedImageCount - frameDescriptors.size());
                diagnostics.descriptorGenerationMismatchCount +=
                    static_cast<std::uint32_t>(
                        frameGenerations.size() > expectedImageCount
                            ? frameGenerations.size() - expectedImageCount
                            : expectedImageCount - frameGenerations.size());
                const auto commonCount = std::min({
                    expectedImageCount,
                    frameDescriptors.size(),
                    frameGenerations.size(),
                });
                for (std::size_t imageIndex = 0U; imageIndex < commonCount; ++imageIndex) {
                    if (frameDescriptors[imageIndex] == VK_NULL_HANDLE ||
                        frameGenerations[imageIndex] != resources->seepageTopologyGeneration) {
                        ++diagnostics.descriptorGenerationMismatchCount;
                    }
                }
            }
        };
    inspectDescriptorRing(
        resources->descriptorSets,
        resources->seepageDescriptorGenerations);
    for (const auto& highlight : resources->highlights) {
        // A retained zero-cardinality highlight is not submitted and therefore
        // has no live descriptor publication requirement. Active highlights
        // must cover the same complete frame-slot/image ring as the base.
        if (highlight.indexCount == 0U) {
            continue;
        }
        inspectDescriptorRing(
            highlight.descriptorSets,
            highlight.seepageDescriptorGenerations);
    }
    diagnostics.exrParamsPublicationRequired = exrExportFrameInFlight_;
    const bool exrDescriptorPresent =
        resources->exrDescriptorSet != VK_NULL_HANDLE;
    const bool exrTopologyReady =
        !exrDescriptorPresent ||
        resources->seepageExrDescriptorGeneration ==
            resources->seepageTopologyGeneration;
    if (!exrTopologyReady ||
        (diagnostics.exrParamsPublicationRequired && !exrDescriptorPresent)) {
        ++diagnostics.descriptorGenerationMismatchCount;
    }
    const bool exrParamsReady =
        !diagnostics.exrParamsPublicationRequired ||
        (exrDescriptorPresent &&
         resources->seepageParamsExrGeneration ==
             resources->seepageParamsGeneration);
    diagnostics.descriptorGenerationReady =
        diagnostics.descriptorGenerationMismatchCount == 0U &&
        expectedImageCount > 0U &&
        std::all_of(
            resources->seepageParamsFrameGenerations.begin(),
            resources->seepageParamsFrameGenerations.end(),
            [resources](std::uint64_t generation) {
                return generation == resources->seepageParamsGeneration;
            }) &&
        exrTopologyReady && exrParamsReady;
    diagnostics.activationFramesRemaining = 0U;
    const auto firstLiveBuffer = resources->seepageParamsBuffers[0].buffer;
    const auto secondLiveBuffer = resources->seepageParamsBuffers[1].buffer;
    diagnostics.liveBuffersDistinct =
        firstLiveBuffer != VK_NULL_HANDLE && secondLiveBuffer != VK_NULL_HANDLE &&
        firstLiveBuffer != secondLiveBuffer;
    const auto exrBuffer = resources->seepageExrParamsBuffer.buffer;
    diagnostics.exrBufferDistinct =
        exrBuffer != VK_NULL_HANDLE && exrBuffer != firstLiveBuffer &&
        exrBuffer != secondLiveBuffer;
    return diagnostics;
}

void VulkanViewportShell::UpdatePointBudget(
    std::size_t layerId,
    const std::vector<std::uint32_t>& sampledIndices,
    bool indicesAlreadySortedUnique,
    bool retainBufferCapacity) {
    SettlePointCloudMutation();

    auto* resources = FindPointCloudResources(layerId);
    if (resources == nullptr) {
        return;
    }

    const bool rebuildDepthSortDescriptors =
        resources->depthSortDescriptorPool != VK_NULL_HANDLE;
    if (rebuildDepthSortDescriptors) {
        vkDestroyDescriptorPool(
            device_,
            resources->depthSortDescriptorPool,
            nullptr);
        resources->depthSortDescriptorPool = VK_NULL_HANDLE;
        resources->depthSortDescriptorSet = VK_NULL_HANDLE;
    }
    const auto rebuildDepthSortDescriptorSet = [&]() {
        if (rebuildDepthSortDescriptors) {
            UpdatePointDepthSortDescriptorSet(resources);
        }
    };

    if (!retainBufferCapacity) {
        DestroyBuffer(&resources->sampledIndexBuffer);
        DestroyBuffer(&resources->sampledSurfelIndexBuffer);
    }
    DestroyBuffer(&resources->interactiveSampledIndexBuffer);
    DestroyBuffer(&resources->interactiveSurfelIndexBuffer);
    resources->usingSampledIndices = false;
    const std::uint32_t validPointCount = resources->waterFlowSourceActive
                                              ? resources->waterFlowSettledPointCount
                                              : resources->pointCount;
    resources->activePointCount = validPointCount;
    resources->interactiveSampledIndexCount = 0;

    const bool trustedIndicesValid =
        indicesAlreadySortedUnique &&
        (sampledIndices.empty() ||
         (sampledIndices.back() < validPointCount &&
          std::is_sorted(sampledIndices.begin(), sampledIndices.end()) &&
          std::adjacent_find(
              sampledIndices.begin(),
              sampledIndices.end()) == sampledIndices.end()));
    std::vector<std::uint32_t> sanitizedStorage;
    const std::vector<std::uint32_t>* sanitized = &sampledIndices;
    if (!trustedIndicesValid) {
        sanitizedStorage =
            SanitizePointIndices(sampledIndices, validPointCount);
        sanitized = &sanitizedStorage;
    }
    if (validPointCount == 0 ||
        sanitized->empty() ||
        sanitized->size() >= validPointCount) {
        if (!retainBufferCapacity) {
            DestroyBuffer(&resources->sampledIndexBuffer);
            DestroyBuffer(&resources->sampledSurfelIndexBuffer);
        }
        rebuildDepthSortDescriptorSet();
        return;
    }

    const auto ensureCapacity = [&] (
        BufferAllocation* buffer,
        VkDeviceSize requiredBytes,
        VkDeviceSize maximumUsefulBytes) {
        if (buffer->buffer != VK_NULL_HANDLE &&
            buffer->size >= requiredBytes) {
            return;
        }
        DestroyBuffer(buffer);
        VkDeviceSize capacityBytes = requiredBytes;
        if (retainBufferCapacity &&
            requiredBytes <=
                std::numeric_limits<VkDeviceSize>::max() -
                    requiredBytes / 2U) {
            capacityBytes = std::min(
                maximumUsefulBytes,
                requiredBytes + requiredBytes / 2U);
        }
        *buffer = CreateHostVisibleBuffer(
            std::max(requiredBytes, capacityBytes),
            VK_BUFFER_USAGE_INDEX_BUFFER_BIT |
                VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
    };
    const auto pointIndexBytes = static_cast<VkDeviceSize>(
        sanitized->size() * sizeof(std::uint32_t));
    ensureCapacity(
        &resources->sampledIndexBuffer,
        pointIndexBytes,
        static_cast<VkDeviceSize>(validPointCount) *
            sizeof(std::uint32_t));
    UploadBufferData(
        resources->sampledIndexBuffer,
        sanitized->data(),
        pointIndexBytes);
    resources->usingSampledIndices = true;
    resources->activePointCount = static_cast<std::uint32_t>(sanitized->size());

    if (sanitized->back() <= kMaxSurfelEncodedPointCount) {
        const auto surfelIndexCount =
            sanitized->size() * kSurfelVerticesPerPoint;
        const auto surfelIndexBytes = static_cast<VkDeviceSize>(
            surfelIndexCount * sizeof(std::uint32_t));
        ensureCapacity(
            &resources->sampledSurfelIndexBuffer,
            surfelIndexBytes,
            static_cast<VkDeviceSize>(validPointCount) *
                kSurfelVerticesPerPoint * sizeof(std::uint32_t));
        auto* destination = static_cast<std::uint32_t*>(
            resources->sampledSurfelIndexBuffer.mapped);
        if (destination == nullptr) {
            throw std::runtime_error{
                "Point-budget surfel index storage is not host visible."};
        }
        for (const auto pointIndex : *sanitized) {
            const auto encodedBase = pointIndex * kSurfelVerticesPerPoint;
            for (std::uint32_t corner = 0U;
                 corner < kSurfelVerticesPerPoint;
                 ++corner) {
                *destination++ = encodedBase + corner;
            }
        }
    } else {
        DestroyBuffer(&resources->sampledSurfelIndexBuffer);
    }
    rebuildDepthSortDescriptorSet();
    TrackPointCloudResidentPeak();
}

void VulkanViewportShell::UpdateInteractivePointSampleBuffer(
    std::size_t layerId,
    const std::vector<std::uint32_t>& sampledIndices,
    bool includeSurfelIndices,
    bool indicesAlreadySortedUnique) {
    SettlePointCloudMutation();

    auto* resources = FindPointCloudResources(layerId);
    if (resources == nullptr) {
        return;
    }

    const bool rebuildDepthSortDescriptors =
        resources->depthSortDescriptorPool != VK_NULL_HANDLE;
    if (rebuildDepthSortDescriptors) {
        vkDestroyDescriptorPool(
            device_,
            resources->depthSortDescriptorPool,
            nullptr);
        resources->depthSortDescriptorPool = VK_NULL_HANDLE;
        resources->depthSortDescriptorSet = VK_NULL_HANDLE;
    }
    const auto rebuildDepthSortDescriptorSet = [&]() {
        if (rebuildDepthSortDescriptors) {
            UpdatePointDepthSortDescriptorSet(resources);
        }
    };

    DestroyBuffer(&resources->interactiveSampledIndexBuffer);
    DestroyBuffer(&resources->interactiveSurfelIndexBuffer);
    resources->interactiveSampledIndexCount = 0;

    const std::uint32_t validPointCount = resources->waterFlowSourceActive
                                              ? resources->waterFlowSettledPointCount
                                              : resources->pointCount;
    const bool trustedIndicesValid =
        indicesAlreadySortedUnique &&
        (sampledIndices.empty() ||
         (sampledIndices.back() < validPointCount &&
          std::is_sorted(sampledIndices.begin(), sampledIndices.end()) &&
          std::adjacent_find(
              sampledIndices.begin(),
              sampledIndices.end()) == sampledIndices.end()));
    std::vector<std::uint32_t> sanitizedStorage;
    const std::vector<std::uint32_t>* sanitized = &sampledIndices;
    if (!trustedIndicesValid) {
        sanitizedStorage =
            SanitizePointIndices(sampledIndices, validPointCount);
        sanitized = &sanitizedStorage;
    }
    if (sanitized->empty() || sanitized->size() >= validPointCount) {
        rebuildDepthSortDescriptorSet();
        return;
    }

    resources->interactiveSampledIndexBuffer = CreateHostVisibleBuffer(
        static_cast<VkDeviceSize>(sanitized->size() * sizeof(std::uint32_t)),
        VK_BUFFER_USAGE_INDEX_BUFFER_BIT |
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
    UploadBufferData(
        resources->interactiveSampledIndexBuffer,
        sanitized->data(),
        resources->interactiveSampledIndexBuffer.size);
    resources->interactiveSampledIndexCount =
        static_cast<std::uint32_t>(sanitized->size());

    if (includeSurfelIndices) {
        const auto surfelIndices =
            invisible_places::renderer::pointcloud::GenerateSurfelEncodedSampleIndices(*sanitized);
        if (!surfelIndices.empty()) {
            resources->interactiveSurfelIndexBuffer = CreateHostVisibleBuffer(
                static_cast<VkDeviceSize>(surfelIndices.size() * sizeof(std::uint32_t)),
                VK_BUFFER_USAGE_INDEX_BUFFER_BIT);
            UploadBufferData(
                resources->interactiveSurfelIndexBuffer,
                surfelIndices.data(),
                resources->interactiveSurfelIndexBuffer.size);
        }
    }
    rebuildDepthSortDescriptorSet();
}

void VulkanViewportShell::UploadPointHighlightIndices(
    std::size_t layerId,
    std::uint64_t key,
    const std::vector<std::uint32_t>& indices,
    const PointHighlightStyle& style) {
    auto* resources = FindPointCloudResources(layerId);
    if (resources == nullptr) {
        return;
    }

    const std::uint32_t validPointCount = resources->waterFlowSourceActive
                                              ? resources->waterFlowSettledPointCount
                                              : resources->pointCount;
    if (validPointCount == 0U) {
        return;
    }
    const auto sanitized = SanitizePointIndices(indices, validPointCount);
    auto existingIt = std::find_if(
        resources->highlights.begin(),
        resources->highlights.end(),
        [key](const ActivePointCloudResources::PointHighlightResources& highlight) {
            return highlight.key == key;
        });
    if (sanitized.empty()) {
        if (existingIt == resources->highlights.end()) {
            return;
        }
        const auto existingIndex = static_cast<std::size_t>(
            std::distance(resources->highlights.begin(), existingIt));
        auto retiredHighlight = std::move(resources->highlights[existingIndex]);
        resources->highlights.erase(resources->highlights.begin() + existingIndex);
        try {
            ReplacePointCloudDescriptorSets(resources);
        } catch (...) {
            resources->highlights.insert(
                resources->highlights.begin() + existingIndex,
                std::move(retiredHighlight));
            throw;
        }
        CleanupPointHighlightResources(&retiredHighlight);
        ++sceneRevision_;
        return;
    }

    ActivePointCloudResources::PointHighlightResources highlight;
    try {
        highlight.key = key;
        highlight.style = style;
        highlight.indexCount = static_cast<std::uint32_t>(sanitized.size());
        highlight.indexBuffer = CreateHostVisibleBuffer(
            static_cast<VkDeviceSize>(sanitized.size() * sizeof(std::uint32_t)),
            VK_BUFFER_USAGE_INDEX_BUFFER_BIT);
        UploadBufferData(highlight.indexBuffer, sanitized.data(), highlight.indexBuffer.size);

        const auto surfelIndices =
            invisible_places::renderer::pointcloud::GenerateSurfelEncodedSampleIndices(sanitized);
        if (!surfelIndices.empty()) {
            highlight.surfelIndexBuffer = CreateHostVisibleBuffer(
                static_cast<VkDeviceSize>(surfelIndices.size() * sizeof(std::uint32_t)),
                VK_BUFFER_USAGE_INDEX_BUFFER_BIT);
            UploadBufferData(
                highlight.surfelIndexBuffer,
                surfelIndices.data(),
                highlight.surfelIndexBuffer.size);
        }

        for (auto& styleBuffer : highlight.styleBuffers) {
            styleBuffer = CreateHostVisibleBuffer(
                sizeof(PointCloudStyleGpu),
                VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT);
        }
    } catch (...) {
        CleanupPointHighlightResources(&highlight);
        throw;
    }

    if (existingIt != resources->highlights.end()) {
        const auto existingIndex = static_cast<std::size_t>(
            std::distance(resources->highlights.begin(), existingIt));
        auto retiredHighlight = std::move(resources->highlights[existingIndex]);
        resources->highlights[existingIndex] = std::move(highlight);
        try {
            ReplacePointCloudDescriptorSets(resources);
        } catch (...) {
            highlight = std::move(resources->highlights[existingIndex]);
            resources->highlights[existingIndex] = std::move(retiredHighlight);
            CleanupPointHighlightResources(&highlight);
            throw;
        }
        CleanupPointHighlightResources(&retiredHighlight);
    } else {
        try {
            // Reserve before transferring ownership of raw Vulkan handles. A
            // host allocation failure must leave them in `highlight`, where
            // the catch path can release them deterministically.
            resources->highlights.reserve(resources->highlights.size() + 1U);
            resources->highlights.push_back(std::move(highlight));
        } catch (...) {
            CleanupPointHighlightResources(&highlight);
            throw;
        }
        try {
            ReplacePointCloudDescriptorSets(resources);
        } catch (...) {
            highlight = std::move(resources->highlights.back());
            resources->highlights.pop_back();
            CleanupPointHighlightResources(&highlight);
            throw;
        }
    }
    ++sceneRevision_;
}

void VulkanViewportShell::ClearPointHighlightIndices(std::size_t layerId, std::uint64_t key) {
    auto* resources = FindPointCloudResources(layerId);
    if (resources == nullptr) {
        return;
    }
    const auto highlightIt = std::find_if(
        resources->highlights.begin(),
        resources->highlights.end(),
        [key](const ActivePointCloudResources::PointHighlightResources& highlight) {
            return highlight.key == key;
        });
    if (highlightIt == resources->highlights.end()) {
        return;
    }
    const auto highlightIndex = static_cast<std::size_t>(
        std::distance(resources->highlights.begin(), highlightIt));
    auto retiredHighlight = std::move(resources->highlights[highlightIndex]);
    resources->highlights.erase(resources->highlights.begin() + highlightIndex);
    try {
        ReplacePointCloudDescriptorSets(resources);
    } catch (...) {
        resources->highlights.insert(
            resources->highlights.begin() + highlightIndex,
            std::move(retiredHighlight));
        throw;
    }
    CleanupPointHighlightResources(&retiredHighlight);
    ++sceneRevision_;
}

void VulkanViewportShell::ClearPointHighlights(std::size_t layerId) {
    auto* resources = FindPointCloudResources(layerId);
    if (resources == nullptr || resources->highlights.empty()) {
        return;
    }
    auto retiredHighlights = std::move(resources->highlights);
    resources->highlights.clear();
    try {
        ReplacePointCloudDescriptorSets(resources);
    } catch (...) {
        resources->highlights = std::move(retiredHighlights);
        throw;
    }
    for (auto& highlight : retiredHighlights) {
        CleanupPointHighlightResources(&highlight);
    }
    ++sceneRevision_;
}

void VulkanViewportShell::RemovePointCloudDeferred(std::size_t layerId) {
    auto resourcesIt = std::find_if(
        pointCloudResources_.begin(),
        pointCloudResources_.end(),
        [layerId](const ActivePointCloudResources& resources) {
            return resources.layerId == layerId;
        });
    if (resourcesIt == pointCloudResources_.end()) {
        return;
    }
    if (resourcesIt->dynamicMeshFlowAllocationRevision != 0U) {
        RemovePointCloud(layerId);
        return;
    }
    deferredPointCloudDestroys_.push_back({
        .detachFrame = pointCloudFrameCounter_,
        .resources = std::move(*resourcesIt),
    });
    pointCloudResources_.erase(resourcesIt);
    ++sceneRevision_;
}

void VulkanViewportShell::FlushDeferredPointCloudDestroys(bool force) {
    for (std::size_t entry = 0U;
         entry < deferredPointCloudDestroys_.size();) {
        auto& deferred = deferredPointCloudDestroys_[entry];
        const bool aged =
            pointCloudFrameCounter_ - deferred.detachFrame >
            static_cast<std::uint64_t>(kFramesInFlight);
        if (!force && !aged) {
            ++entry;
            continue;
        }
        DestroyPointCloudResources(&deferred.resources);
        deferredPointCloudDestroys_.erase(
            deferredPointCloudDestroys_.begin() +
            static_cast<std::ptrdiff_t>(entry));
    }
}

void VulkanViewportShell::RemovePointCloud(std::size_t layerId) {
    auto resourcesIt = std::find_if(
        pointCloudResources_.begin(),
        pointCloudResources_.end(),
        [layerId](const ActivePointCloudResources& resources) { return resources.layerId == layerId; });
    if (resourcesIt == pointCloudResources_.end()) {
        return;
    }
    if (resourcesIt->dynamicMeshFlowAllocationRevision != 0U) {
        ClearDynamicMeshFlowGpuSimulation(layerId);
        return;
    }

    CleanupPointCloudResources(&(*resourcesIt));
    pointCloudResources_.erase(resourcesIt);
}

void VulkanViewportShell::ClearPointClouds() {
    SettlePointCloudMutation();
    FlushDeferredPointCloudDestroys(true);
    // Base layers may bind a Mesh Flow contact generation owned by another
    // point resource. Retire every render descriptor before destroying any
    // resource so teardown cannot leave a cross-resource buffer reference.
    for (auto& resources : pointCloudResources_) {
        RetirePointCloudRenderDescriptors(&resources);
    }
    for (auto& resources : pointCloudResources_) {
        DestroyPointCloudResources(&resources);
    }
    pointCloudResources_.clear();
}

void VulkanViewportShell::UploadWaterSurfaceCache(
    const invisible_places::water::WaterSurfaceCache& cache) {
    auto& resources = rainResources_;
    auto cacheIdentity = cache.cacheIdentity;
    if (!cacheIdentity.Valid() ||
        cacheIdentity.sourceSignature != cache.signature ||
        cache.gpuData.sourceRevision != cache.revision ||
        cache.gpuData.sourceIdentity != cacheIdentity) {
        cacheIdentity = invisible_places::water::BuildWaterSurfaceCacheIdentity(cache);
    }

    // The shared cache is immutable for a scene signature. Avoid hash rebuilds,
    // allocation, descriptor writes, and submissions when Application polls the
    // same warmed cache more than once.
    if (resources.collisionUploadRevision != 0U &&
        resources.collisionCacheIdentity == cacheIdentity &&
        resources.pendingSurfaceUpload.surfaceTableBuffer.buffer == VK_NULL_HANDLE) {
        return;
    }
    if (resources.pendingSurfaceUpload.surfaceTableBuffer.buffer != VK_NULL_HANDLE &&
        resources.pendingSurfaceUpload.cacheIdentity == cacheIdentity) {
        return;
    }

    invisible_places::water::WaterSurfaceGpuData generatedGpuData;
    const invisible_places::water::WaterSurfaceGpuData* gpuData = &cache.gpuData;
    const bool hasPersistedGpuTables = cache.gpuData.persistedTables.Valid();
    const bool hasPersistedGroundTable =
        cache.gpuData.persistedTables.GroundValid();
    if (cache.gpuData.sourceRevision != cache.revision ||
        cache.gpuData.sourceIdentity != cacheIdentity ||
        (!hasPersistedGpuTables &&
         (cache.gpuData.surfaceTable.empty() ||
          cache.gpuData.vegetationTable.empty() ||
          cache.gpuData.flowSurfaceTable.empty() ||
          (!hasPersistedGroundTable &&
           cache.gpuData.groundTable.empty())))) {
        generatedGpuData = invisible_places::water::BuildWaterSurfaceGpuData(cache);
        gpuData = &generatedGpuData;
    }
    const bool streamsPersistedTables = gpuData->persistedTables.Valid();
    const bool streamsPersistedGroundTable =
        gpuData->persistedTables.GroundValid();
    const auto surfaceTableCount = streamsPersistedTables
                                       ? gpuData->persistedTables.surfaceCount
                                       : static_cast<std::uint64_t>(gpuData->surfaceTable.size());
    const auto vegetationTableCount = streamsPersistedTables
                                          ? gpuData->persistedTables.vegetationCount
                                          : static_cast<std::uint64_t>(gpuData->vegetationTable.size());
    const auto flowSurfaceTableCount = streamsPersistedTables
                                           ? gpuData->persistedTables.flowSurfaceCount
                                           : static_cast<std::uint64_t>(gpuData->flowSurfaceTable.size());
    const auto groundTableCount = streamsPersistedGroundTable
                                      ? gpuData->persistedTables.groundCount
                                      : static_cast<std::uint64_t>(gpuData->groundTable.size());
    if (surfaceTableCount == 0U || vegetationTableCount == 0U ||
        flowSurfaceTableCount == 0U || groundTableCount == 0U) {
        throw std::runtime_error{"Water surface cache did not produce GPU hash tables."};
    }
    if (surfaceTableCount > std::numeric_limits<std::uint32_t>::max() ||
        vegetationTableCount > std::numeric_limits<std::uint32_t>::max() ||
        flowSurfaceTableCount > std::numeric_limits<std::uint32_t>::max() ||
        groundTableCount > std::numeric_limits<std::uint32_t>::max() ||
        cache.flowSurfaceSurfels.size() > std::numeric_limits<std::uint32_t>::max() ||
        cache.groundCells.size() > std::numeric_limits<std::uint32_t>::max()) {
        throw std::runtime_error{"Water surface cache exceeds the 32-bit GPU table limit."};
    }
    const auto groundEntryCandidates =
        invisible_places::water::BuildWaterDynamicMeshFlowGroundEntries(cache);
    if (groundEntryCandidates.size() >
        std::numeric_limits<std::uint32_t>::max()) {
        throw std::runtime_error{
            "Water surface cache exceeds the 32-bit Mesh Flow entry limit."};
    }

    const VkDeviceSize rainSurfaceBytes =
        static_cast<VkDeviceSize>(surfaceTableCount) *
        sizeof(invisible_places::water::RainGpuSurfaceSlot);
    const VkDeviceSize vegetationBytes =
        static_cast<VkDeviceSize>(vegetationTableCount) *
        sizeof(invisible_places::water::RainGpuVegetationSlot);
    const VkDeviceSize flowSurfaceBytes =
        static_cast<VkDeviceSize>(flowSurfaceTableCount) *
        sizeof(invisible_places::water::WaterGpuSurfaceSurfelSlot);
    const VkDeviceSize groundBytes =
        static_cast<VkDeviceSize>(groundTableCount) *
        sizeof(invisible_places::water::WaterGpuGroundSlot);
    const VkDeviceSize groundEntryBytes =
        static_cast<VkDeviceSize>(
            std::max<std::size_t>(1U, groundEntryCandidates.size())) *
        sizeof(invisible_places::water::WaterDynamicMeshFlowGroundEntry);
    const auto persistedFileMatchesSnapshot = [&]() {
        if (!streamsPersistedTables) {
            return true;
        }
        std::error_code sizeError;
        const auto fileSize = std::filesystem::file_size(
            gpuData->persistedTables.filePath,
            sizeError);
        std::error_code writeTimeError;
        const auto writeTime = std::filesystem::last_write_time(
            gpuData->persistedTables.filePath,
            writeTimeError);
        return !sizeError && !writeTimeError &&
               static_cast<std::uint64_t>(fileSize) ==
                   gpuData->persistedTables.fileSize &&
               static_cast<std::int64_t>(writeTime.time_since_epoch().count()) ==
                   gpuData->persistedTables.modificationTicks;
    };
    if (!persistedFileMatchesSnapshot()) {
        throw std::runtime_error{
            "Validated water surface sidecar changed before GPU upload."};
    }

    // A newer scene may arrive while the previous preprocessing dispatch is in
    // flight. Keep that job alive until its own fence signals instead of
    // blocking or invalidating resources referenced by the queue.
    if (resources.pendingSurfaceUpload.surfaceTableBuffer.buffer != VK_NULL_HANDLE) {
        resources.abandonedSurfaceUploads.push_back(resources.pendingSurfaceUpload);
        resources.pendingSurfaceUpload = {};
    }
    auto& pending = resources.pendingSurfaceUpload;
    // Keep host-visible transient memory bounded even for multi-gigabyte scene
    // caches. A single mapped buffer is reused after each queue-local transfer
    // fence; resident buffers remain device-local. This startup-only path may
    // briefly wait for copies, but it avoids retaining three full cache-sized
    // staging allocations alongside the CPU and device payloads.
    BufferAllocation uploadStagingBuffer{};
    VkCommandBuffer uploadCommandBuffer = VK_NULL_HANDLE;
    VkFence uploadFence = VK_NULL_HANDLE;
    const auto cleanupUpload = [&]() {
        if (uploadCommandBuffer != VK_NULL_HANDLE && commandPool_ != VK_NULL_HANDLE) {
            vkFreeCommandBuffers(device_, commandPool_, 1U, &uploadCommandBuffer);
            uploadCommandBuffer = VK_NULL_HANDLE;
        }
        if (uploadFence != VK_NULL_HANDLE) {
            vkDestroyFence(device_, uploadFence, nullptr);
            uploadFence = VK_NULL_HANDLE;
        }
        DestroyBuffer(&uploadStagingBuffer);
    };
    try {
        pending.surfaceTableBuffer = CreateDeviceLocalBuffer(
            rainSurfaceBytes,
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        pending.vegetationTableBuffer = CreateDeviceLocalBuffer(
            vegetationBytes,
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        pending.flowSurfaceInputBuffer = CreateDeviceLocalBuffer(
            flowSurfaceBytes,
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        pending.flowSurfaceTableBuffer = CreateDeviceLocalBuffer(
            flowSurfaceBytes,
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        pending.groundTableBuffer = CreateDeviceLocalBuffer(
            groundBytes,
            VK_BUFFER_USAGE_TRANSFER_DST_BIT |
                VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        pending.groundEntryCandidateBuffer = CreateDeviceLocalBuffer(
            groundEntryBytes,
            VK_BUFFER_USAGE_TRANSFER_DST_BIT |
                VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);

        const VkDeviceSize largestUpload =
            std::max(
                {rainSurfaceBytes,
                 vegetationBytes,
                 flowSurfaceBytes,
                 groundBytes,
                 groundEntryBytes});
        uploadStagingBuffer = CreateHostVisibleBuffer(
            std::min(largestUpload, kWaterSurfaceUploadStagingLimitBytes),
            VK_BUFFER_USAGE_TRANSFER_SRC_BIT);
        pending.peakStagingBytes =
            static_cast<std::uint64_t>(uploadStagingBuffer.size);

        VkCommandBufferAllocateInfo uploadAllocInfo{
            VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
        uploadAllocInfo.commandPool = commandPool_;
        uploadAllocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        uploadAllocInfo.commandBufferCount = 1U;
        Check(
            vkAllocateCommandBuffers(
                device_,
                &uploadAllocInfo,
                &uploadCommandBuffer),
            "vkAllocateCommandBuffers(water surface chunk upload)");
        VkFenceCreateInfo uploadFenceInfo{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};
        Check(
            vkCreateFence(device_, &uploadFenceInfo, nullptr, &uploadFence),
            "vkCreateFence(water surface chunk upload)");

        bool commandBufferSubmitted = false;
        const auto submitStagedChunk = [&](VkDeviceSize chunkBytes,
                                           VkDeviceSize destinationOffset,
                                           const BufferAllocation& destination) {
                if (commandBufferSubmitted) {
                    Check(
                        vkResetFences(device_, 1U, &uploadFence),
                        "vkResetFences(water surface chunk upload)");
                    Check(
                        vkResetCommandBuffer(uploadCommandBuffer, 0U),
                        "vkResetCommandBuffer(water surface chunk upload)");
                }

                VkCommandBufferBeginInfo beginInfo{
                    VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
                beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
                Check(
                    vkBeginCommandBuffer(uploadCommandBuffer, &beginInfo),
                    "vkBeginCommandBuffer(water surface chunk upload)");
                VkBufferCopy copy{};
                copy.dstOffset = destinationOffset;
                copy.size = chunkBytes;
                vkCmdCopyBuffer(
                    uploadCommandBuffer,
                    uploadStagingBuffer.buffer,
                    destination.buffer,
                    1U,
                    &copy);
                Check(
                    vkEndCommandBuffer(uploadCommandBuffer),
                    "vkEndCommandBuffer(water surface chunk upload)");
                VkSubmitInfo submitInfo{VK_STRUCTURE_TYPE_SUBMIT_INFO};
                submitInfo.commandBufferCount = 1U;
                submitInfo.pCommandBuffers = &uploadCommandBuffer;
                Check(
                    vkQueueSubmit(graphicsQueue_, 1U, &submitInfo, uploadFence),
                    "vkQueueSubmit(water surface chunk upload)");
                commandBufferSubmitted = true;
                Check(
                    vkWaitForFences(
                        device_,
                        1U,
                        &uploadFence,
                        VK_TRUE,
                        std::numeric_limits<std::uint64_t>::max()),
                    "vkWaitForFences(water surface chunk upload)");
        };
        const auto uploadInChunks = [&](const void* sourceData,
                                        VkDeviceSize sourceBytes,
                                        const BufferAllocation& destination) {
            const auto* source = static_cast<const std::byte*>(sourceData);
            VkDeviceSize destinationOffset = 0U;
            while (destinationOffset < sourceBytes) {
                const auto chunkBytes = std::min(
                    uploadStagingBuffer.size,
                    sourceBytes - destinationOffset);
                UploadBufferData(
                    uploadStagingBuffer,
                    source + static_cast<std::size_t>(destinationOffset),
                    chunkBytes);
                submitStagedChunk(chunkBytes, destinationOffset, destination);
                destinationOffset += chunkBytes;
            }
        };
        const auto uploadPersistedSection = [&](std::ifstream* source,
                                                std::uint64_t fileOffset,
                                                std::uint64_t sectionTag,
                                                std::uint64_t elementCount,
                                                VkDeviceSize sourceBytes,
                                                const BufferAllocation& destination,
                                                invisible_places::water::WaterSurfaceGpuStreamChecksumBuilder*
                                                    streamChecksum) {
            if (source == nullptr || !source->is_open()) {
                throw std::runtime_error{"Water surface sidecar stream is unavailable."};
            }
            if (streamChecksum == nullptr) {
                throw std::runtime_error{"Water surface sidecar verifier is unavailable."};
            }
            source->clear();
            source->seekg(static_cast<std::streamoff>(fileOffset), std::ios::beg);
            if (!source->good()) {
                throw std::runtime_error{"Water surface sidecar table offset is invalid."};
            }
            streamChecksum->AddPod(sectionTag);
            streamChecksum->AddPod(elementCount);
            VkDeviceSize destinationOffset = 0U;
            while (destinationOffset < sourceBytes) {
                const auto chunkBytes = std::min(
                    uploadStagingBuffer.size,
                    sourceBytes - destinationOffset);
                source->read(
                    static_cast<char*>(uploadStagingBuffer.mapped),
                    static_cast<std::streamsize>(chunkBytes));
                if (!source->good()) {
                    throw std::runtime_error{"Water surface sidecar GPU table is truncated."};
                }
                streamChecksum->AddBytes(
                    uploadStagingBuffer.mapped,
                    static_cast<std::size_t>(chunkBytes));
                submitStagedChunk(chunkBytes, destinationOffset, destination);
                destinationOffset += chunkBytes;
            }
        };
        if (streamsPersistedTables) {
            invisible_places::water::WaterSurfaceGpuStreamChecksumBuilder
                streamChecksum =
                    gpuData->persistedTables.payloadValidationDeferred
                        ? gpuData->persistedTables.payloadChecksumPrefix
                        : invisible_places::water::
                              WaterSurfaceGpuStreamChecksumBuilder{};
            std::ifstream persistedInput{
                gpuData->persistedTables.filePath,
                std::ios::binary};
            if (!persistedInput.is_open()) {
                throw std::runtime_error{"Validated Water surface sidecar could not be reopened for GPU upload."};
            }
            uploadPersistedSection(
                &persistedInput,
                gpuData->persistedTables.surfaceOffset,
                4U,
                surfaceTableCount,
                rainSurfaceBytes,
                pending.surfaceTableBuffer,
                &streamChecksum);
            uploadPersistedSection(
                &persistedInput,
                gpuData->persistedTables.vegetationOffset,
                5U,
                vegetationTableCount,
                vegetationBytes,
                pending.vegetationTableBuffer,
                &streamChecksum);
            uploadPersistedSection(
                &persistedInput,
                gpuData->persistedTables.flowSurfaceOffset,
                6U,
                flowSurfaceTableCount,
                flowSurfaceBytes,
                pending.flowSurfaceInputBuffer,
                &streamChecksum);
            if (streamsPersistedGroundTable) {
                uploadPersistedSection(
                    &persistedInput,
                    gpuData->persistedTables.groundOffset,
                    8U,
                    groundTableCount,
                    groundBytes,
                    pending.groundTableBuffer,
                    &streamChecksum);
            }
            const auto expectedStreamChecksum =
                gpuData->persistedTables.payloadValidationDeferred
                    ? gpuData->payloadChecksum
                    : gpuData->persistedTables.streamChecksum;
            if (streamChecksum.Finish() != expectedStreamChecksum ||
                !persistedFileMatchesSnapshot()) {
                throw std::runtime_error{
                    "Water surface sidecar changed during GPU upload."};
            }
        } else {
            uploadInChunks(
                gpuData->surfaceTable.data(),
                rainSurfaceBytes,
                pending.surfaceTableBuffer);
            uploadInChunks(
                gpuData->vegetationTable.data(),
                vegetationBytes,
                pending.vegetationTableBuffer);
            uploadInChunks(
                gpuData->flowSurfaceTable.data(),
                flowSurfaceBytes,
                pending.flowSurfaceInputBuffer);
        }
        if (streamsPersistedTables && !streamsPersistedGroundTable) {
            uploadInChunks(
                gpuData->groundTable.data(),
                groundBytes,
                pending.groundTableBuffer);
        } else if (!streamsPersistedTables) {
            uploadInChunks(
                gpuData->groundTable.data(),
                groundBytes,
                pending.groundTableBuffer);
        }
        if (groundEntryCandidates.empty()) {
            const invisible_places::water::WaterDynamicMeshFlowGroundEntry
                emptyEntry{};
            uploadInChunks(
                &emptyEntry,
                sizeof(emptyEntry),
                pending.groundEntryCandidateBuffer);
        } else {
            uploadInChunks(
                groundEntryCandidates.data(),
                groundEntryBytes,
                pending.groundEntryCandidateBuffer);
        }
        cleanupUpload();

        pending.preprocessUniformBuffer = CreateHostVisibleBuffer(
            sizeof(WaterSurfacePreprocessUniformsGpu),
            VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT);

        pending.bounds = cache.bounds;
        pending.surfaceMask = gpuData->surfaceMask;
        pending.vegetationMask = gpuData->vegetationMask;
        pending.maximumProbeCount = std::max(1U, gpuData->maximumProbeCount);
        pending.flowSurfaceMask = gpuData->flowSurfaceMask;
        pending.flowMaximumProbeCount = std::max(1U, gpuData->flowMaximumProbeCount);
        pending.flowSurfaceCellCount =
            static_cast<std::uint32_t>(cache.flowSurfaceSurfels.size());
        pending.flowSurfaceTableCapacity =
            static_cast<std::uint32_t>(flowSurfaceTableCount);
        pending.groundMask = gpuData->groundMask;
        pending.groundMaximumProbeCount =
            std::max(1U, gpuData->groundMaximumProbeCount);
        pending.groundCellCount =
            static_cast<std::uint32_t>(cache.groundCells.size());
        pending.groundEntryCandidateCount =
            static_cast<std::uint32_t>(groundEntryCandidates.size());
        pending.resolutionMeters = std::max(0.001F, cache.resolutionMeters);
        pending.cacheRevision = cache.revision;
        pending.cacheIdentity = cacheIdentity;
        pending.tableBytes = static_cast<std::uint64_t>(
            rainSurfaceBytes + vegetationBytes + flowSurfaceBytes +
            groundBytes + groundEntryBytes);
        pending.uploadRevision = resources.collisionUploadRevision + 1U;

        WaterSurfacePreprocessUniformsGpu preprocessUniforms;
        preprocessUniforms.table = glm::uvec4{
            pending.flowSurfaceMask,
            pending.flowMaximumProbeCount,
            pending.flowSurfaceTableCapacity,
            pending.flowSurfaceCellCount,
        };
        preprocessUniforms.parameters.x = pending.resolutionMeters;
        UploadBufferData(
            pending.preprocessUniformBuffer,
            &preprocessUniforms,
            sizeof(preprocessUniforms));

        std::array<VkDescriptorSetLayout, kFramesInFlight> rainLayouts{};
        rainLayouts.fill(rainDescriptorSetLayout_);
        const std::array<VkDescriptorPoolSize, 2> rainPoolSizes = {
            MakePoolSize(
                VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,
                static_cast<std::uint32_t>(kFramesInFlight)),
            MakePoolSize(
                VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                static_cast<std::uint32_t>(kFramesInFlight * 8U)),
        };
        VkDescriptorPoolCreateInfo rainPoolInfo{
            VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
        rainPoolInfo.maxSets = static_cast<std::uint32_t>(kFramesInFlight);
        rainPoolInfo.poolSizeCount = static_cast<std::uint32_t>(rainPoolSizes.size());
        rainPoolInfo.pPoolSizes = rainPoolSizes.data();
        Check(
            vkCreateDescriptorPool(
                device_,
                &rainPoolInfo,
                nullptr,
                &pending.rainDescriptorPool),
            "vkCreateDescriptorPool(pending water surface rain)");
        VkDescriptorSetAllocateInfo rainAllocInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
        rainAllocInfo.descriptorPool = pending.rainDescriptorPool;
        rainAllocInfo.descriptorSetCount = static_cast<std::uint32_t>(rainLayouts.size());
        rainAllocInfo.pSetLayouts = rainLayouts.data();
        Check(
            vkAllocateDescriptorSets(
                device_,
                &rainAllocInfo,
                pending.rainDescriptorSets.data()),
            "vkAllocateDescriptorSets(pending water surface rain)");
        UpdateRainDescriptorSets(
            pending.rainDescriptorSets,
            pending.surfaceTableBuffer,
            pending.vegetationTableBuffer);

        const std::array<VkDescriptorPoolSize, 2> preprocessPoolSizes = {
            MakePoolSize(VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1U),
            MakePoolSize(VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 2U),
        };
        VkDescriptorPoolCreateInfo preprocessPoolInfo{
            VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
        preprocessPoolInfo.maxSets = 1U;
        preprocessPoolInfo.poolSizeCount =
            static_cast<std::uint32_t>(preprocessPoolSizes.size());
        preprocessPoolInfo.pPoolSizes = preprocessPoolSizes.data();
        Check(
            vkCreateDescriptorPool(
                device_,
                &preprocessPoolInfo,
                nullptr,
                &pending.preprocessDescriptorPool),
            "vkCreateDescriptorPool(pending water surface preprocess)");
        VkDescriptorSetAllocateInfo preprocessAllocInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
        preprocessAllocInfo.descriptorPool = pending.preprocessDescriptorPool;
        preprocessAllocInfo.descriptorSetCount = 1U;
        preprocessAllocInfo.pSetLayouts = &waterSurfacePreprocessDescriptorSetLayout_;
        Check(
            vkAllocateDescriptorSets(
                device_,
                &preprocessAllocInfo,
                &pending.preprocessDescriptorSet),
            "vkAllocateDescriptorSets(pending water surface preprocess)");
        UpdateWaterSurfacePreprocessDescriptorSet(&pending);
        if (!DispatchWaterSurfacePreprocess(&pending)) {
            throw std::runtime_error{
                "Water surface preprocess pipeline was unavailable."};
        }
        resources.collisionUploadRevision = pending.uploadRevision;
    } catch (...) {
        cleanupUpload();
        CleanupWaterSurfacePendingUpload(&pending);
        throw;
    }

    diagnostics_.rainCollisionUploadRevision = resources.collisionUploadRevision;
    diagnostics_.waterSurfaceUploadRevision = resources.collisionUploadRevision;
    diagnostics_.waterSurfaceGpuBytes = resources.residentTableBytes +
        pending.tableBytes + static_cast<std::uint64_t>(flowSurfaceBytes);
    diagnostics_.waterSurfacePeakStagingBytes = pending.peakStagingBytes;
    diagnostics_.waterSurfacePreprocessDispatchCount = resources.preprocessDispatchCount;
    diagnostics_.waterSurfacePreprocessPending = true;
}

void VulkanViewportShell::ClearWaterSurfaceCache() {
    auto& resources = rainResources_;
    if (resources.pendingSurfaceUpload.surfaceTableBuffer.buffer != VK_NULL_HANDLE) {
        resources.abandonedSurfaceUploads.push_back(resources.pendingSurfaceUpload);
        resources.pendingSurfaceUpload = {};
    }
    resources.collisionReady = false;
    resources.flowSurfaceReady = false;
    resources.collisionBounds = {};
    resources.collisionCacheRevision = 0U;
    resources.collisionCacheIdentity = {};
    resources.peakStagingBytes = 0U;
    ++resources.resetEpoch;
    diagnostics_.rainCollisionCacheRevision = 0U;
    diagnostics_.waterSurfaceCacheRevision = 0U;
    diagnostics_.waterSurfacePeakStagingBytes = 0U;
    diagnostics_.waterSurfaceSurfelCellCount = 0U;
    diagnostics_.waterSurfacePreprocessPending = !resources.abandonedSurfaceUploads.empty();
    ++sceneRevision_;
}

WaterSurfaceFlowGpuView VulkanViewportShell::WaterSurfaceFlowView() const {
    const auto& resources = rainResources_;
    return {
        .buffer = resources.flowSurfaceTableBuffer.buffer,
        .offset = 0U,
        .range = resources.flowSurfaceTableBuffer.size,
        .tableMask = resources.flowSurfaceMask,
        .maximumProbeCount = resources.flowMaximumProbeCount,
        .occupiedCellCount = resources.flowSurfaceCellCount,
        .resolutionMeters = resources.surfaceResolutionMeters,
        .cacheRevision = resources.collisionCacheRevision,
        .uploadRevision = resources.flowSurfaceResidentUploadRevision,
        .cacheIdentity = &resources.collisionCacheIdentity,
        .valid = resources.flowSurfaceReady,
        .preprocessingComplete = resources.flowSurfaceReady,
    };
}

WaterGroundFlowGpuView VulkanViewportShell::WaterGroundFlowView() const {
    const auto& resources = rainResources_;
    return {
        .buffer = resources.groundTableBuffer.buffer,
        .offset = 0U,
        .range = resources.groundTableBuffer.size,
        .entryCandidateBuffer =
            resources.groundEntryCandidateBuffer.buffer,
        .entryCandidateOffset = 0U,
        .entryCandidateRange =
            resources.groundEntryCandidateBuffer.size,
        .entryCandidateCount =
            resources.groundEntryCandidateCount,
        .tableMask = resources.groundMask,
        .maximumProbeCount = resources.groundMaximumProbeCount,
        .occupiedCellCount = resources.groundCellCount,
        .resolutionMeters = resources.surfaceResolutionMeters,
        .cacheRevision = resources.collisionCacheRevision,
        .uploadRevision = resources.groundResidentUploadRevision,
        .cacheIdentity = &resources.collisionCacheIdentity,
        .bounds = resources.collisionBounds,
        .valid =
            resources.collisionReady &&
            resources.groundTableBuffer.buffer != VK_NULL_HANDLE &&
            resources.groundEntryCandidateBuffer.buffer != VK_NULL_HANDLE &&
            resources.groundCellCount > 0U,
    };
}

bool VulkanViewportShell::WaterSurfaceUploadPending() const {
    return rainResources_.pendingSurfaceUpload.surfaceTableBuffer.buffer != VK_NULL_HANDLE;
}

bool VulkanViewportShell::DispatchWaterSurfacePreprocess(
    WaterSurfacePendingGpuUpload* pending) {
    if (pending == nullptr || pending->flowSurfaceTableCapacity == 0U ||
        pending->preprocessDescriptorSet == VK_NULL_HANDLE ||
        waterSurfacePreprocessPipeline_ == VK_NULL_HANDLE ||
        waterSurfacePreprocessPipelineLayout_ == VK_NULL_HANDLE ||
        commandPool_ == VK_NULL_HANDLE) {
        return false;
    }

    VkCommandBufferAllocateInfo allocInfo{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
    allocInfo.commandPool = commandPool_;
    allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    allocInfo.commandBufferCount = 1U;
    Check(
        vkAllocateCommandBuffers(
            device_,
            &allocInfo,
            &pending->preprocessCommandBuffer),
        "vkAllocateCommandBuffers(water surface preprocess)");

    VkFenceCreateInfo fenceInfo{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};
    Check(
        vkCreateFence(
            device_,
            &fenceInfo,
            nullptr,
            &pending->preprocessFence),
        "vkCreateFence(water surface preprocess)");

    VkCommandBufferBeginInfo beginInfo{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
    beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    Check(
        vkBeginCommandBuffer(pending->preprocessCommandBuffer, &beginInfo),
        "vkBeginCommandBuffer(water surface preprocess)");

    // Chunked transfers were completed on this same queue before this command
    // buffer was recorded. Establish visibility for Rain reads and the Flow
    // preprocessing dispatch without keeping cache-sized staging buffers alive.
    std::array<VkBufferMemoryBarrier, 5> uploadBarriers{};
    const std::array<VkBuffer, 5> uploadedBuffers = {
        pending->surfaceTableBuffer.buffer,
        pending->vegetationTableBuffer.buffer,
        pending->flowSurfaceInputBuffer.buffer,
        pending->groundTableBuffer.buffer,
        pending->groundEntryCandidateBuffer.buffer,
    };
    for (std::size_t index = 0U; index < uploadBarriers.size(); ++index) {
        auto& barrier = uploadBarriers[index];
        barrier = {VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER};
        barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
        barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
        barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barrier.buffer = uploadedBuffers[index];
        barrier.offset = 0U;
        barrier.size = VK_WHOLE_SIZE;
    }
    vkCmdPipelineBarrier(
        pending->preprocessCommandBuffer,
        VK_PIPELINE_STAGE_TRANSFER_BIT,
        VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT |
            VK_PIPELINE_STAGE_VERTEX_SHADER_BIT |
            VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
        0U,
        0U,
        nullptr,
        static_cast<std::uint32_t>(uploadBarriers.size()),
        uploadBarriers.data(),
        0U,
        nullptr);

    vkCmdBindPipeline(
        pending->preprocessCommandBuffer,
        VK_PIPELINE_BIND_POINT_COMPUTE,
        waterSurfacePreprocessPipeline_);
    vkCmdBindDescriptorSets(
        pending->preprocessCommandBuffer,
        VK_PIPELINE_BIND_POINT_COMPUTE,
        waterSurfacePreprocessPipelineLayout_,
        0U,
        1U,
        &pending->preprocessDescriptorSet,
        0U,
        nullptr);
    vkCmdDispatch(
        pending->preprocessCommandBuffer,
        (pending->flowSurfaceTableCapacity + 63U) / 64U,
        1U,
        1U);

    VkBufferMemoryBarrier outputBarrier{VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER};
    outputBarrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
    outputBarrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
    outputBarrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    outputBarrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    outputBarrier.buffer = pending->flowSurfaceTableBuffer.buffer;
    outputBarrier.offset = 0U;
    outputBarrier.size = VK_WHOLE_SIZE;
    vkCmdPipelineBarrier(
        pending->preprocessCommandBuffer,
        VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
        VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT |
            VK_PIPELINE_STAGE_VERTEX_SHADER_BIT |
            VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
        0U,
        0U,
        nullptr,
        1U,
        &outputBarrier,
        0U,
        nullptr);
    Check(
        vkEndCommandBuffer(pending->preprocessCommandBuffer),
        "vkEndCommandBuffer(water surface preprocess)");

    VkSubmitInfo submitInfo{VK_STRUCTURE_TYPE_SUBMIT_INFO};
    submitInfo.commandBufferCount = 1U;
    submitInfo.pCommandBuffers = &pending->preprocessCommandBuffer;
    Check(
        vkQueueSubmit(
            graphicsQueue_,
            1U,
            &submitInfo,
            pending->preprocessFence),
        "vkQueueSubmit(water surface preprocess)");

    ++rainResources_.preprocessDispatchCount;
    return true;
}

void VulkanViewportShell::PollWaterSurfacePreprocess() {
    auto& resources = rainResources_;

    // This method is called immediately after currentFrameIndex_'s fence has
    // signalled and before it is reset. That makes retired resource reclamation
    // deterministic without a device-wide wait.
    const std::uint32_t completedFrameBit = 1U << currentFrameIndex_;
    for (auto& retired : resources.retiredSurfaceResources) {
        retired.outstandingFrameMask &= ~completedFrameBit;
        if (retired.outstandingExr && exrExportResources_.fence != VK_NULL_HANDLE) {
            const VkResult exrStatus = vkGetFenceStatus(device_, exrExportResources_.fence);
            if (exrStatus == VK_SUCCESS) {
                if (exrExportResources_.commandBuffer != VK_NULL_HANDLE) {
                    Check(
                        vkResetCommandBuffer(exrExportResources_.commandBuffer, 0U),
                        "vkResetCommandBuffer(retired water surface EXR)");
                }
                retired.outstandingExr = false;
            } else if (exrStatus != VK_NOT_READY) {
                Check(exrStatus, "vkGetFenceStatus(retired water surface EXR)");
            }
        }
    }
    for (auto retiredIt = resources.retiredSurfaceResources.begin();
         retiredIt != resources.retiredSurfaceResources.end();) {
        const bool borrowedByMeshFlow = std::any_of(
            pointCloudResources_.begin(),
            pointCloudResources_.end(),
            [retiredIt](const ActivePointCloudResources& pointResources) {
                return retiredIt->uploadRevision != 0U &&
                       pointResources.dynamicMeshFlowAllocationRevision != 0U &&
                       pointResources.dynamicMeshFlowGroundUploadRevision ==
                           retiredIt->uploadRevision;
            });
        if (retiredIt->outstandingFrameMask != 0U ||
            retiredIt->outstandingExr ||
            borrowedByMeshFlow) {
            ++retiredIt;
            continue;
        }
        CleanupWaterSurfaceRetiredResources(&(*retiredIt));
        retiredIt = resources.retiredSurfaceResources.erase(retiredIt);
    }

    for (auto abandonedIt = resources.abandonedSurfaceUploads.begin();
         abandonedIt != resources.abandonedSurfaceUploads.end();) {
        const VkResult status = abandonedIt->preprocessFence == VK_NULL_HANDLE
            ? VK_SUCCESS
            : vkGetFenceStatus(device_, abandonedIt->preprocessFence);
        if (status == VK_NOT_READY) {
            ++abandonedIt;
            continue;
        }
        Check(status, "vkGetFenceStatus(abandoned water surface preprocess)");
        CleanupWaterSurfacePendingUpload(&(*abandonedIt));
        abandonedIt = resources.abandonedSurfaceUploads.erase(abandonedIt);
    }

    auto& pending = resources.pendingSurfaceUpload;
    if (pending.surfaceTableBuffer.buffer == VK_NULL_HANDLE ||
        pending.preprocessFence == VK_NULL_HANDLE) {
        diagnostics_.waterSurfacePreprocessPending =
            pending.surfaceTableBuffer.buffer != VK_NULL_HANDLE ||
            !resources.abandonedSurfaceUploads.empty();
        return;
    }

    const VkResult status = vkGetFenceStatus(device_, pending.preprocessFence);
    if (status == VK_NOT_READY) {
        diagnostics_.waterSurfacePreprocessPending = true;
        return;
    }
    Check(status, "vkGetFenceStatus(water surface preprocess)");

    struct PendingBaseContactGeneration {
        std::size_t resourceIndex = 0U;
        PendingPointDescriptorGeneration generation{};
    };
    std::vector<PendingBaseContactGeneration>
        pendingBaseContactGenerations;
    std::vector<std::size_t> staleMeshFlowLayerIds;
    for (const auto& pointResources : pointCloudResources_) {
        if (pointResources.dynamicMeshFlowAllocationRevision != 0U &&
            (pointResources.dynamicMeshFlowGroundUploadRevision !=
                 pending.uploadRevision ||
             pointResources.dynamicMeshFlowGroundIdentity !=
                 pending.cacheIdentity)) {
            staleMeshFlowLayerIds.push_back(pointResources.layerId);
        }
    }
    const bool replacesMeshFlowGroundOwner =
        !staleMeshFlowLayerIds.empty();
    if (replacesMeshFlowGroundOwner) {
        // Base layers must stop sampling the outgoing owner's contact buffers
        // as soon as the new Ground becomes resident. Keep the old simulation
        // alive through staging for rollback, then retire it only after these
        // replacement descriptor generations have committed.
        pointDescriptorDynamicMeshFlowContactOverride_ =
            DynamicMeshFlowContactGpuView{
                .eventBuffer =
                    resources.dynamicMeshFlowDummyContactEventBuffer.buffer,
                .eventOffset = 0U,
                .eventRange =
                    resources.dynamicMeshFlowDummyContactEventBuffer.size,
                .gridBuffer =
                    resources.dynamicMeshFlowDummyContactGridBuffer.buffer,
                .gridOffset = 0U,
                .gridRange =
                    resources.dynamicMeshFlowDummyContactGridBuffer.size,
                .eventCapacity = 1U,
                .gridMask = 0U,
                .gridCellSizeMeters = 0.75F,
                .valid = false,
            };
        try {
            for (std::size_t resourceIndex = 0U;
                 resourceIndex < pointCloudResources_.size();
                 ++resourceIndex) {
                auto& pointResources =
                    pointCloudResources_[resourceIndex];
                if (pointResources.dynamicMeshFlowAllocationRevision != 0U ||
                    pointResources.pointDescriptorPool == VK_NULL_HANDLE) {
                    continue;
                }
                PendingBaseContactGeneration pendingBase;
                pendingBase.resourceIndex = resourceIndex;
                pendingBase.generation =
                    BuildPointDescriptorGeneration(
                        &pointResources,
                        exrExportResources_.depthImage.view,
                        exrExportResources_.uniformBuffer.buffer);
                pendingBaseContactGenerations.push_back(
                    std::move(pendingBase));
            }
        } catch (...) {
            pointDescriptorDynamicMeshFlowContactOverride_.reset();
            for (auto& pendingBase :
                 pendingBaseContactGenerations) {
                DestroyPointDescriptorGeneration(
                    &pendingBase.generation);
            }
            throw;
        }
        pointDescriptorDynamicMeshFlowContactOverride_.reset();
        try {
            SettlePointCloudMutation();
        } catch (...) {
            for (auto& pendingBase :
                 pendingBaseContactGenerations) {
                DestroyPointDescriptorGeneration(
                    &pendingBase.generation);
            }
            throw;
        }
    }

    if (pending.preprocessCommandBuffer != VK_NULL_HANDLE && commandPool_ != VK_NULL_HANDLE) {
        vkFreeCommandBuffers(device_, commandPool_, 1U, &pending.preprocessCommandBuffer);
        pending.preprocessCommandBuffer = VK_NULL_HANDLE;
    }
    vkDestroyFence(device_, pending.preprocessFence, nullptr);
    pending.preprocessFence = VK_NULL_HANDLE;
    pending.preprocessDescriptorSet = VK_NULL_HANDLE;
    if (pending.preprocessDescriptorPool != VK_NULL_HANDLE) {
        vkDestroyDescriptorPool(device_, pending.preprocessDescriptorPool, nullptr);
        pending.preprocessDescriptorPool = VK_NULL_HANDLE;
    }
    DestroyBuffer(&pending.flowSurfaceInputBuffer);
    DestroyBuffer(&pending.preprocessUniformBuffer);

    WaterSurfaceRetiredGpuResources retired;
    retired.surfaceTableBuffer = resources.surfaceTableBuffer;
    retired.vegetationTableBuffer = resources.vegetationTableBuffer;
    retired.flowSurfaceTableBuffer = resources.flowSurfaceTableBuffer;
    retired.groundTableBuffer = resources.groundTableBuffer;
    retired.groundEntryCandidateBuffer =
        resources.groundEntryCandidateBuffer;
    retired.uploadRevision = resources.groundResidentUploadRevision;
    retired.rainDescriptorSets = resources.descriptorSets;
    retired.rainDescriptorPool = resources.descriptorPool;
    // A signalled fence is not enough on MoltenVK: an executable command
    // buffer can retain its resolved descriptor objects. Require every live
    // frame slot to be reset once after promotion before retiring this pool.
    retired.outstandingFrameMask =
        (1U << static_cast<std::uint32_t>(kFramesInFlight)) - 1U;
    if (exrExportResources_.fence != VK_NULL_HANDLE) {
        // The EXR command buffer likewise needs one explicit reset, whether
        // its fence was already signalled at the instant of promotion or not.
        retired.outstandingExr = exrExportResources_.commandBuffer != VK_NULL_HANDLE;
    }

    resources.surfaceTableBuffer = pending.surfaceTableBuffer;
    resources.vegetationTableBuffer = pending.vegetationTableBuffer;
    resources.flowSurfaceTableBuffer = pending.flowSurfaceTableBuffer;
    resources.groundTableBuffer = pending.groundTableBuffer;
    resources.groundEntryCandidateBuffer =
        pending.groundEntryCandidateBuffer;
    resources.descriptorSets = pending.rainDescriptorSets;
    resources.descriptorPool = pending.rainDescriptorPool;
    pending.surfaceTableBuffer = {};
    pending.vegetationTableBuffer = {};
    pending.flowSurfaceTableBuffer = {};
    pending.groundTableBuffer = {};
    pending.groundEntryCandidateBuffer = {};
    pending.rainDescriptorSets.fill(VK_NULL_HANDLE);
    pending.rainDescriptorPool = VK_NULL_HANDLE;

    resources.collisionBounds = pending.bounds;
    resources.surfaceMask = pending.surfaceMask;
    resources.vegetationMask = pending.vegetationMask;
    resources.maximumProbeCount = pending.maximumProbeCount;
    resources.flowSurfaceMask = pending.flowSurfaceMask;
    resources.flowMaximumProbeCount = pending.flowMaximumProbeCount;
    resources.flowSurfaceCellCount = pending.flowSurfaceCellCount;
    resources.flowSurfaceTableCapacity = pending.flowSurfaceTableCapacity;
    resources.groundMask = pending.groundMask;
    resources.groundMaximumProbeCount =
        pending.groundMaximumProbeCount;
    resources.groundCellCount = pending.groundCellCount;
    resources.groundEntryCandidateCount =
        pending.groundEntryCandidateCount;
    resources.surfaceResolutionMeters = pending.resolutionMeters;
    resources.collisionCacheRevision = pending.cacheRevision;
    resources.collisionCacheIdentity = pending.cacheIdentity;
    resources.flowSurfaceResidentUploadRevision = pending.uploadRevision;
    resources.groundResidentUploadRevision = pending.uploadRevision;
    resources.residentTableBytes = pending.tableBytes;
    resources.peakStagingBytes = pending.peakStagingBytes;
    resources.collisionReady = pending.bounds.valid;
    resources.flowSurfaceReady =
        pending.bounds.valid && pending.flowSurfaceCellCount > 0U;
    ++resources.resetEpoch;

    for (auto& pendingBase :
         pendingBaseContactGenerations) {
        InstallPointDescriptorGeneration(
            &pointCloudResources_[pendingBase.resourceIndex],
            &pendingBase.generation);
    }

    // The new Ground and dummy-contact descriptor generations are now
    // committed. Retire every outgoing Mesh owner here, even when Mesh Flow is
    // disabled and no replacement dispatch will follow. The clear operation
    // settles and replaces all cross-resource point descriptors before
    // destroying the owner's buffers, so the retired Ground can be reclaimed
    // without leaving a descriptor bound to freed storage.
    for (const auto staleLayerId : staleMeshFlowLayerIds) {
        ClearDynamicMeshFlowGpuSimulation(staleLayerId);
    }

    resources.retiredSurfaceResources.push_back(retired);
    pending = {};
    if (resources.retiredSurfaceResources.back().outstandingFrameMask == 0U &&
        !resources.retiredSurfaceResources.back().outstandingExr &&
        !std::any_of(
            pointCloudResources_.begin(),
            pointCloudResources_.end(),
            [&resources](const ActivePointCloudResources& pointResources) {
                return pointResources.dynamicMeshFlowAllocationRevision != 0U &&
                       pointResources.dynamicMeshFlowGroundUploadRevision ==
                           resources.retiredSurfaceResources.back().uploadRevision;
            })) {
        CleanupWaterSurfaceRetiredResources(&resources.retiredSurfaceResources.back());
        resources.retiredSurfaceResources.pop_back();
    }

    diagnostics_.rainCollisionCacheRevision = resources.collisionCacheRevision;
    diagnostics_.rainCollisionUploadRevision = resources.collisionUploadRevision;
    diagnostics_.waterSurfaceCacheRevision = resources.collisionCacheRevision;
    diagnostics_.waterSurfaceUploadRevision = resources.collisionUploadRevision;
    diagnostics_.waterSurfaceGpuBytes = resources.residentTableBytes;
    diagnostics_.waterSurfacePeakStagingBytes = resources.peakStagingBytes;
    diagnostics_.waterSurfaceSurfelCellCount = resources.flowSurfaceCellCount;
    diagnostics_.waterSurfaceTableCapacity = resources.flowSurfaceTableCapacity;
    diagnostics_.waterSurfaceMaximumProbeCount = resources.flowMaximumProbeCount;
    diagnostics_.waterSurfacePreprocessDispatchCount = resources.preprocessDispatchCount;
    diagnostics_.waterSurfacePreprocessPending = !resources.abandonedSurfaceUploads.empty();
    ++sceneRevision_;
}

std::uint64_t VulkanViewportShell::WaterSurfaceUploadRevision() const {
    return rainResources_.collisionUploadRevision;
}

void VulkanViewportShell::UploadGaussianSplats(
    std::size_t layerId,
    const invisible_places::io::LoadedGaussianSplat& splats) {
    if (splats.SplatCount() == 0) {
        throw std::runtime_error{"Cannot upload an empty Gaussian splat layer."};
    }
    if (splats.SplatCount() > std::numeric_limits<std::uint32_t>::max()) {
        throw std::runtime_error{"Gaussian splat layer exceeds the current 32-bit draw-count limit."};
    }

    WaitIdle();
    ResetPointCloudReferencingCommandBuffers();

    auto* existingResources = FindGaussianSplatResources(layerId);
    const auto nextRevision = existingResources != nullptr ? (existingResources->revision + 1U) : 1U;
    if (existingResources == nullptr) {
        gaussianSplatResources_.push_back(ActiveGaussianSplatResources{});
        existingResources = &gaussianSplatResources_.back();
    } else {
        CleanupGaussianSplatResources(existingResources);
    }

    auto& resources = *existingResources;
    resources.layerId = layerId;
    resources.splatCount = static_cast<std::uint32_t>(splats.SplatCount());
    resources.cpuCenters = splats.centers;
    resources.cpuScales = splats.scales;
    resources.cpuRotations = splats.rotations;
    resources.cpuOpacities = splats.opacities;
    resources.cpuShCoefficients = splats.shCoefficients;
    resources.revision = nextRevision;

    std::vector<glm::vec4> centers;
    std::vector<glm::vec4> scales;
    std::vector<glm::vec4> rotations;
    centers.reserve(splats.SplatCount());
    scales.reserve(splats.SplatCount());
    rotations.reserve(splats.SplatCount());

    for (std::size_t index = 0; index < splats.SplatCount(); ++index) {
        const auto& center = splats.centers[index];
        const auto& scale = splats.scales[index];
        const auto& rotation = splats.rotations[index];
        centers.emplace_back(center.x, center.y, center.z, 1.0F);
        scales.emplace_back(scale[0], scale[1], scale[2], 0.0F);
        rotations.emplace_back(rotation[0], rotation[1], rotation[2], rotation[3]);
    }

    resources.centerBuffer = CreateHostVisibleBuffer(
        static_cast<VkDeviceSize>(centers.size() * sizeof(glm::vec4)),
        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
    UploadBufferData(resources.centerBuffer, centers.data(), resources.centerBuffer.size);

    resources.scaleBuffer = CreateHostVisibleBuffer(
        static_cast<VkDeviceSize>(scales.size() * sizeof(glm::vec4)),
        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
    UploadBufferData(resources.scaleBuffer, scales.data(), resources.scaleBuffer.size);

    resources.rotationBuffer = CreateHostVisibleBuffer(
        static_cast<VkDeviceSize>(rotations.size() * sizeof(glm::vec4)),
        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
    UploadBufferData(resources.rotationBuffer, rotations.data(), resources.rotationBuffer.size);

    resources.opacityBuffer = CreateHostVisibleBuffer(
        static_cast<VkDeviceSize>(splats.opacities.size() * sizeof(float)),
        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
    UploadBufferData(resources.opacityBuffer, splats.opacities.data(), resources.opacityBuffer.size);

    resources.shBuffer = CreateHostVisibleBuffer(
        static_cast<VkDeviceSize>(splats.shCoefficients.size() * sizeof(float)),
        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
    UploadBufferData(resources.shBuffer, splats.shCoefficients.data(), resources.shBuffer.size);

    UpdateGaussianSplatDescriptorSets(&resources);
    highQualityGaussianSceneDirty_ = true;
}

void VulkanViewportShell::RemoveGaussianSplats(std::size_t layerId) {
    WaitIdle();
    ResetPointCloudReferencingCommandBuffers();

    auto resourcesIt = std::find_if(
        gaussianSplatResources_.begin(),
        gaussianSplatResources_.end(),
        [layerId](const ActiveGaussianSplatResources& resources) { return resources.layerId == layerId; });
    if (resourcesIt == gaussianSplatResources_.end()) {
        return;
    }

    CleanupGaussianSplatResources(&(*resourcesIt));
    gaussianSplatResources_.erase(resourcesIt);
    highQualityGaussianSceneDirty_ = true;
}

void VulkanViewportShell::ClearGaussianSplats() {
    WaitIdle();
    ResetPointCloudReferencingCommandBuffers();
    for (auto& resources : gaussianSplatResources_) {
        CleanupGaussianSplatResources(&resources);
    }
    gaussianSplatResources_.clear();
    highQualityGaussianSceneDirty_ = true;
}

bool VulkanViewportShell::UiWantsMouseCapture() const {
    return ImGui::GetCurrentContext() != nullptr && ImGui::GetIO().WantCaptureMouse;
}

bool VulkanViewportShell::UiWantsKeyboardCapture() const {
    return ImGui::GetCurrentContext() != nullptr && ImGui::GetIO().WantCaptureKeyboard;
}

bool VulkanViewportShell::HasPointClouds() const {
    return std::any_of(
        pointCloudResources_.begin(),
        pointCloudResources_.end(),
        [](const ActivePointCloudResources& resources) { return resources.activePointCount > 0; });
}

bool VulkanViewportShell::HasGaussianSplats() const {
    return std::any_of(
        gaussianSplatResources_.begin(),
        gaussianSplatResources_.end(),
        [](const ActiveGaussianSplatResources& resources) { return resources.splatCount > 0; });
}

invisible_places::output::HalfRgbaExrImage VulkanViewportShell::RenderPointCloudExrFrame(
    const PointCloudExrFrameRequest& request) {
    if (!BeginPointCloudExrFrame(request)) {
        // A cancelled or failed export job can abandon a submitted frame.
        // Synchronous rendering only runs when no job owns the EXR pipeline,
        // so drain the stale frame and retry once instead of latching every
        // future preview shut.
        CancelPointCloudExrFrame();
        if (!BeginPointCloudExrFrame(request)) {
            throw std::runtime_error{"GPU EXR export already has a frame in flight."};
        }
    }
    Check(vkWaitForFences(device_, 1, &exrExportResources_.fence, VK_TRUE, UINT64_MAX), "vkWaitForFences(exr complete)");
    return CompletePointCloudExrFrame();
}

bool VulkanViewportShell::BeginPointCloudExrFrame(const PointCloudExrFrameRequest& request) {
    if (exrExportFrameInFlight_) {
        return false;
    }
    if (request.width == 0 || request.height == 0) {
        throw std::runtime_error{"GPU EXR export requires a non-zero frame size."};
    }
    if (request.renderState.pointCloudLayers.empty()) {
        throw std::runtime_error{"GPU EXR export requires at least one visible point-cloud layer."};
    }

    const bool needsEyeDomeLightingResources =
        request.renderState.eyeDomeLightingEnabled &&
        HasPointCloudExrReadback(request.readbackMask, PointCloudExrReadbackMask::Color);
    if (exrExportResources_.framebuffer == VK_NULL_HANDLE ||
        exrExportResources_.width != request.width ||
        exrExportResources_.height != request.height ||
        !ContainsPointCloudExrReadbacks(
            exrExportResources_.readbackMask,
            request.readbackMask) ||
        (needsEyeDomeLightingResources &&
         exrExportResources_.eyeDomeLightingDescriptorSet == VK_NULL_HANDLE)) {
        CreateExrExportResources(
            request.width,
            request.height,
            needsEyeDomeLightingResources,
            request.readbackMask);
    }

    if (exrExportResources_.fence == VK_NULL_HANDLE ||
        exrExportResources_.commandBuffer == VK_NULL_HANDLE) {
        throw std::runtime_error{"GPU EXR export resources are not initialized."};
    }
    const VkResult existingFenceStatus = vkGetFenceStatus(device_, exrExportResources_.fence);
    if (existingFenceStatus == VK_NOT_READY) {
        if (exrExportFrameInFlight_) {
            // A genuinely running asynchronous export frame.
            return false;
        }
        // Unsignalled with no submission pending: a previous Begin threw
        // between vkResetFences and vkQueueSubmit. The fence is inert, so
        // treat it as idle (the wait below is skipped) instead of refusing
        // every future export until the resources are recreated.
    } else {
        Check(existingFenceStatus, "vkGetFenceStatus(exr idle)");
    }
    // EXR owns a separate parameter snapshot. The fence above guarantees a
    // previous asynchronous export is no longer reading it.
    FlushWaterSeepageParamsForExr();
    // The EXR descriptor is separate from both live frame descriptors. It is
    // rebound only after its own fence so surface-cache promotion cannot
    // mutate descriptor state used by an asynchronous export.
    UpdateRainExrDescriptorSet(
        rainResources_.surfaceTableBuffer,
        rainResources_.vegetationTableBuffer);

    const auto previousRenderState = renderState_;
    auto restoreRenderState = [&]() { renderState_ = previousRenderState; };

    try {
        UpdateRainRuntimeTiming(request.renderState);
        renderState_ = request.renderState;
        for (auto& layer : renderState_.pointCloudLayers) {
            // Match the live viewport: the impact effects shade every
            // layer's points inside their bands, independent of the layer's
            // collision role.
            layer.style.rainImpactEffects =
                renderState_.rainSettings.enabled &&
                renderState_.rainSettings.impactEffectsEnabled;
        }
        for (auto& resources : pointCloudResources_) {
            if (resources.exrDescriptorSet == VK_NULL_HANDLE) {
                ReplacePointCloudDescriptorSets(&resources);
            }
        }

        if (existingFenceStatus == VK_SUCCESS) {
            Check(vkWaitForFences(device_, 1, &exrExportResources_.fence, VK_TRUE, UINT64_MAX), "vkWaitForFences(exr)");
        }
        Check(vkResetFences(device_, 1, &exrExportResources_.fence), "vkResetFences(exr)");
        Check(
            vkResetCommandBuffer(exrExportResources_.commandBuffer, 0),
            "vkResetCommandBuffer(exr)");
        UploadFrameUniformsToBuffer(exrExportResources_.uniformBuffer, request.width, request.height);
        RecordExrExportCommandBuffer(request);

        VkSubmitInfo submitInfo{VK_STRUCTURE_TYPE_SUBMIT_INFO};
        submitInfo.commandBufferCount = 1;
        submitInfo.pCommandBuffers = &exrExportResources_.commandBuffer;
        Check(vkQueueSubmit(graphicsQueue_, 1, &submitInfo, exrExportResources_.fence), "vkQueueSubmit(exr)");
        exrExportFrameInFlight_ = true;
        exrExportInFlightWidth_ = request.width;
        exrExportInFlightHeight_ = request.height;
        exrExportInFlightReadbackMask_ = request.readbackMask;
        restoreRenderState();
        return true;
    } catch (...) {
        restoreRenderState();
        throw;
    }
}

PointCloudExrFrameStatus VulkanViewportShell::PollPointCloudExrFrame() {
    if (!exrExportFrameInFlight_) {
        return PointCloudExrFrameStatus::Idle;
    }
    const VkResult status = vkGetFenceStatus(device_, exrExportResources_.fence);
    if (status == VK_SUCCESS) {
        return PointCloudExrFrameStatus::Ready;
    }
    if (status == VK_NOT_READY) {
        return PointCloudExrFrameStatus::Running;
    }
    Check(status, "vkGetFenceStatus(exr)");
    return PointCloudExrFrameStatus::Running;
}

invisible_places::output::HalfRgbaExrImage VulkanViewportShell::CompletePointCloudExrFrame() {
    if (!exrExportFrameInFlight_) {
        return {};
    }
    const VkResult status = vkGetFenceStatus(device_, exrExportResources_.fence);
    if (status == VK_NOT_READY) {
        throw std::runtime_error{"GPU EXR export frame is still running."};
    }
    Check(status, "vkGetFenceStatus(exr complete)");

    const auto width = exrExportInFlightWidth_;
    const auto height = exrExportInFlightHeight_;
    const auto mask = exrExportInFlightReadbackMask_;
    exrExportFrameInFlight_ = false;
    exrExportInFlightWidth_ = 0;
    exrExportInFlightHeight_ = 0;
    exrExportInFlightReadbackMask_ = PointCloudExrReadbackMask::All;
    return ReadCompletedExrExportFrame(mask, width, height);
}

void VulkanViewportShell::CancelPointCloudExrFrame() {
    if (!exrExportFrameInFlight_) {
        return;
    }
    if (exrExportResources_.fence != VK_NULL_HANDLE) {
        Check(vkWaitForFences(device_, 1, &exrExportResources_.fence, VK_TRUE, UINT64_MAX), "vkWaitForFences(exr cancel)");
    }
    exrExportFrameInFlight_ = false;
    exrExportInFlightWidth_ = 0;
    exrExportInFlightHeight_ = 0;
    exrExportInFlightReadbackMask_ = PointCloudExrReadbackMask::All;
}

invisible_places::output::HalfRgbaExrImage VulkanViewportShell::ReadCompletedExrExportFrame(
    PointCloudExrReadbackMask readbackMask,
    std::uint32_t width,
    std::uint32_t height) {
    invisible_places::output::HalfRgbaExrImage image;
    image.width = width;
    image.height = height;
    const auto pixelCount = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);

    if (HasPointCloudExrReadback(readbackMask, PointCloudExrReadbackMask::Color)) {
        image.rgbaHalf.resize(pixelCount * 4U);
        void* mappedColor = exrExportResources_.colorReadbackBuffer.mapped;
        bool unmapColor = false;
        if (mappedColor == nullptr) {
            Check(
                vkMapMemory(
                    device_,
                    exrExportResources_.colorReadbackBuffer.memory,
                    0,
                    exrExportResources_.colorReadbackBuffer.size,
                    0,
                    &mappedColor),
                "vkMapMemory(exr color)");
            unmapColor = true;
        }
        std::memcpy(
            image.rgbaHalf.data(),
            mappedColor,
            image.rgbaHalf.size() * sizeof(std::uint16_t));
        if (unmapColor) {
            vkUnmapMemory(device_, exrExportResources_.colorReadbackBuffer.memory);
        }
    }

    if (HasPointCloudExrReadback(readbackMask, PointCloudExrReadbackMask::Depth)) {
        image.depth.resize(pixelCount);
        void* mappedDepth = exrExportResources_.depthReadbackBuffer.mapped;
        bool unmapDepth = false;
        if (mappedDepth == nullptr) {
            Check(
                vkMapMemory(
                    device_,
                    exrExportResources_.depthReadbackBuffer.memory,
                    0,
                    exrExportResources_.depthReadbackBuffer.size,
                    0,
                    &mappedDepth),
                "vkMapMemory(exr depth)");
            unmapDepth = true;
        }
        std::memcpy(
            image.depth.data(),
            mappedDepth,
            image.depth.size() * sizeof(float));
        if (unmapDepth) {
            vkUnmapMemory(device_, exrExportResources_.depthReadbackBuffer.memory);
        }
    }

    auto copyRgbHalfReadback = [&](const BufferAllocation& buffer,
                                   std::vector<std::uint16_t>* destination,
                                   const char* mapLabel) {
        if (destination == nullptr || destination->size() != pixelCount * 3U) {
            return;
        }

        void* mapped = buffer.mapped;
        bool unmap = false;
        if (mapped == nullptr) {
            Check(
                vkMapMemory(device_, buffer.memory, 0, buffer.size, 0, &mapped),
                mapLabel);
            unmap = true;
        }

        const auto* source = static_cast<const std::uint16_t*>(mapped);
        for (std::size_t pixelIndex = 0; pixelIndex < pixelCount; ++pixelIndex) {
            const std::size_t sourceOffset = pixelIndex * 4U;
            const std::size_t destinationOffset = pixelIndex * 3U;
            (*destination)[destinationOffset + 0U] = source[sourceOffset + 0U];
            (*destination)[destinationOffset + 1U] = source[sourceOffset + 1U];
            (*destination)[destinationOffset + 2U] = source[sourceOffset + 2U];
        }
        if (unmap) {
            vkUnmapMemory(device_, buffer.memory);
        }
    };
    if (HasPointCloudExrReadback(readbackMask, PointCloudExrReadbackMask::Normal)) {
        image.normalHalf.resize(pixelCount * 3U);
        copyRgbHalfReadback(exrExportResources_.normalReadbackBuffer, &image.normalHalf, "vkMapMemory(exr normal)");
    }
    if (HasPointCloudExrReadback(readbackMask, PointCloudExrReadbackMask::Albedo)) {
        image.albedoHalf.resize(pixelCount * 3U);
        copyRgbHalfReadback(exrExportResources_.albedoReadbackBuffer, &image.albedoHalf, "vkMapMemory(exr albedo)");
    }
    return image;
}

VulkanViewportShell::ImGuiPreviewImageTexture VulkanViewportShell::UploadImGuiPreviewImageTexture(
    std::uint32_t width,
    std::uint32_t height,
    const std::vector<std::uint8_t>& rgba) {
    if (width == 0 || height == 0) {
        throw std::runtime_error{"Preview image upload requires a non-zero image size."};
    }
    const auto byteCount =
        static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 4U;
    if (rgba.size() != byteCount) {
        throw std::runtime_error{"Preview image upload received an invalid RGBA buffer size."};
    }
    if (device_ == VK_NULL_HANDLE || commandPool_ == VK_NULL_HANDLE) {
        throw std::runtime_error{"Preview image upload requires an active Vulkan viewport."};
    }

    BufferAllocation stagingBuffer{};
    ImageAllocation image{};
    VkSampler sampler = VK_NULL_HANDLE;
    VkCommandBuffer commandBuffer = VK_NULL_HANDLE;
    VkDescriptorSet descriptorSet = VK_NULL_HANDLE;

    try {
        stagingBuffer = CreateHostVisibleBuffer(
            static_cast<VkDeviceSize>(byteCount),
            VK_BUFFER_USAGE_TRANSFER_SRC_BIT);
        UploadBufferData(stagingBuffer, rgba.data(), static_cast<VkDeviceSize>(byteCount));

        image = CreateAttachmentImage(
            width,
            height,
            VK_FORMAT_R8G8B8A8_UNORM,
            VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
            VK_IMAGE_ASPECT_COLOR_BIT);

        VkCommandBufferAllocateInfo allocInfo{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
        allocInfo.commandPool = commandPool_;
        allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        allocInfo.commandBufferCount = 1;
        Check(vkAllocateCommandBuffers(device_, &allocInfo, &commandBuffer), "vkAllocateCommandBuffers(preview texture)");

        VkCommandBufferBeginInfo beginInfo{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
        beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
        Check(vkBeginCommandBuffer(commandBuffer, &beginInfo), "vkBeginCommandBuffer(preview texture)");

        VkImageMemoryBarrier transferBarrier{VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER};
        transferBarrier.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        transferBarrier.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
        transferBarrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        transferBarrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        transferBarrier.image = image.image;
        transferBarrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        transferBarrier.subresourceRange.baseMipLevel = 0;
        transferBarrier.subresourceRange.levelCount = 1;
        transferBarrier.subresourceRange.baseArrayLayer = 0;
        transferBarrier.subresourceRange.layerCount = 1;
        transferBarrier.srcAccessMask = 0;
        transferBarrier.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
        vkCmdPipelineBarrier(
            commandBuffer,
            VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,
            VK_PIPELINE_STAGE_TRANSFER_BIT,
            0,
            0,
            nullptr,
            0,
            nullptr,
            1,
            &transferBarrier);

        VkBufferImageCopy copyRegion{};
        copyRegion.bufferOffset = 0;
        copyRegion.bufferRowLength = 0;
        copyRegion.bufferImageHeight = 0;
        copyRegion.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        copyRegion.imageSubresource.mipLevel = 0;
        copyRegion.imageSubresource.baseArrayLayer = 0;
        copyRegion.imageSubresource.layerCount = 1;
        copyRegion.imageOffset = {0, 0, 0};
        copyRegion.imageExtent = {width, height, 1};
        vkCmdCopyBufferToImage(
            commandBuffer,
            stagingBuffer.buffer,
            image.image,
            VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
            1,
            &copyRegion);

        VkImageMemoryBarrier shaderReadBarrier{VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER};
        shaderReadBarrier.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
        shaderReadBarrier.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        shaderReadBarrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        shaderReadBarrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        shaderReadBarrier.image = image.image;
        shaderReadBarrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        shaderReadBarrier.subresourceRange.baseMipLevel = 0;
        shaderReadBarrier.subresourceRange.levelCount = 1;
        shaderReadBarrier.subresourceRange.baseArrayLayer = 0;
        shaderReadBarrier.subresourceRange.layerCount = 1;
        shaderReadBarrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
        shaderReadBarrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
        vkCmdPipelineBarrier(
            commandBuffer,
            VK_PIPELINE_STAGE_TRANSFER_BIT,
            VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
            0,
            0,
            nullptr,
            0,
            nullptr,
            1,
            &shaderReadBarrier);

        Check(vkEndCommandBuffer(commandBuffer), "vkEndCommandBuffer(preview texture)");

        VkSubmitInfo submitInfo{VK_STRUCTURE_TYPE_SUBMIT_INFO};
        submitInfo.commandBufferCount = 1;
        submitInfo.pCommandBuffers = &commandBuffer;
        Check(vkQueueSubmit(graphicsQueue_, 1, &submitInfo, VK_NULL_HANDLE), "vkQueueSubmit(preview texture)");
        Check(vkQueueWaitIdle(graphicsQueue_), "vkQueueWaitIdle(preview texture)");
        vkFreeCommandBuffers(device_, commandPool_, 1, &commandBuffer);
        commandBuffer = VK_NULL_HANDLE;

        VkSamplerCreateInfo samplerInfo{VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO};
        samplerInfo.magFilter = VK_FILTER_LINEAR;
        samplerInfo.minFilter = VK_FILTER_LINEAR;
        samplerInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
        samplerInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        samplerInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        samplerInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        samplerInfo.mipLodBias = 0.0F;
        samplerInfo.anisotropyEnable = VK_FALSE;
        samplerInfo.maxAnisotropy = 1.0F;
        samplerInfo.compareEnable = VK_FALSE;
        samplerInfo.compareOp = VK_COMPARE_OP_ALWAYS;
        samplerInfo.minLod = 0.0F;
        samplerInfo.maxLod = 0.0F;
        samplerInfo.borderColor = VK_BORDER_COLOR_INT_OPAQUE_BLACK;
        samplerInfo.unnormalizedCoordinates = VK_FALSE;
        Check(vkCreateSampler(device_, &samplerInfo, nullptr, &sampler), "vkCreateSampler(preview texture)");

        descriptorSet =
            ImGui_ImplVulkan_AddTexture(sampler, image.view, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);

        ClearImGuiPreviewImageTexture();
        imguiPreviewImageTexture_ = {
            .width = width,
            .height = height,
            .image = image,
            .sampler = sampler,
            .descriptorSet = descriptorSet,
        };
        image = {};
        sampler = VK_NULL_HANDLE;
        descriptorSet = VK_NULL_HANDLE;
        DestroyBuffer(&stagingBuffer);

        return {
            .textureId = TextureIdFromDescriptorSet(imguiPreviewImageTexture_.descriptorSet),
            .width = width,
            .height = height,
        };
    } catch (...) {
        if (commandBuffer != VK_NULL_HANDLE && commandPool_ != VK_NULL_HANDLE) {
            vkFreeCommandBuffers(device_, commandPool_, 1, &commandBuffer);
        }
        if (descriptorSet != VK_NULL_HANDLE && ImGui::GetCurrentContext() != nullptr) {
            ImGui_ImplVulkan_RemoveTexture(descriptorSet);
        }
        if (sampler != VK_NULL_HANDLE) {
            vkDestroySampler(device_, sampler, nullptr);
        }
        DestroyImage(&image);
        DestroyBuffer(&stagingBuffer);
        throw;
    }
}

void VulkanViewportShell::ClearImGuiPreviewImageTexture() {
    if (device_ == VK_NULL_HANDLE) {
        imguiPreviewImageTexture_ = {};
        return;
    }

    if (imguiPreviewImageTexture_.descriptorSet != VK_NULL_HANDLE &&
        ImGui::GetCurrentContext() != nullptr) {
        vkDeviceWaitIdle(device_);
        ImGui_ImplVulkan_RemoveTexture(imguiPreviewImageTexture_.descriptorSet);
    }
    if (imguiPreviewImageTexture_.sampler != VK_NULL_HANDLE) {
        vkDestroySampler(device_, imguiPreviewImageTexture_.sampler, nullptr);
    }
    DestroyImage(&imguiPreviewImageTexture_.image);
    imguiPreviewImageTexture_ = {};
}

void VulkanViewportShell::CreateInstance() {
    std::uint32_t glfwExtensionCount = 0;
    const char** glfwExtensions = glfwGetRequiredInstanceExtensions(&glfwExtensionCount);
    if (glfwExtensions == nullptr) {
        throw std::runtime_error{"GLFW did not provide required Vulkan instance extensions."};
    }

    std::vector<const char*> extensions(glfwExtensions, glfwExtensions + glfwExtensionCount);
#if defined(__APPLE__)
    if (std::find(extensions.begin(), extensions.end(), VK_KHR_PORTABILITY_ENUMERATION_EXTENSION_NAME) ==
        extensions.end()) {
        extensions.push_back(VK_KHR_PORTABILITY_ENUMERATION_EXTENSION_NAME);
    }
#endif

    VkApplicationInfo appInfo{VK_STRUCTURE_TYPE_APPLICATION_INFO};
    appInfo.pApplicationName = "Invisible Places";
    appInfo.applicationVersion = VK_MAKE_VERSION(0, 1, 0);
    appInfo.pEngineName = "InvisiblePlacesScenePreview";
    appInfo.engineVersion = VK_MAKE_VERSION(0, 1, 0);
    appInfo.apiVersion = VK_API_VERSION_1_2;

    VkInstanceCreateInfo createInfo{VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO};
    createInfo.pApplicationInfo = &appInfo;
    createInfo.enabledExtensionCount = static_cast<std::uint32_t>(extensions.size());
    createInfo.ppEnabledExtensionNames = extensions.data();
#if defined(__APPLE__)
    createInfo.flags |= VK_INSTANCE_CREATE_ENUMERATE_PORTABILITY_BIT_KHR;
#endif

    Check(vkCreateInstance(&createInfo, nullptr, &instance_), "vkCreateInstance");
}

void VulkanViewportShell::CreateSurface() {
    Check(glfwCreateWindowSurface(instance_, window_, nullptr, &surface_), "glfwCreateWindowSurface");
}

void VulkanViewportShell::PickPhysicalDevice() {
    std::uint32_t deviceCount = 0;
    vkEnumeratePhysicalDevices(instance_, &deviceCount, nullptr);
    if (deviceCount == 0) {
        throw std::runtime_error{"No Vulkan physical devices were found."};
    }

    std::vector<VkPhysicalDevice> devices(deviceCount);
    vkEnumeratePhysicalDevices(instance_, &deviceCount, devices.data());

    for (const auto device : devices) {
        bool portabilitySubsetEnabled = false;
        if (!IsDeviceSuitable(device, surface_, &portabilitySubsetEnabled)) {
            continue;
        }

        physicalDevice_ = device;
        enablePortabilitySubset_ = portabilitySubsetEnabled;

        VkPhysicalDeviceProperties properties{};
        vkGetPhysicalDeviceProperties(device, &properties);
        diagnostics_.rendererName = properties.deviceName;
        diagnostics_.driverName = "Vulkan";
        pointSizeRangeMin_ = std::max(1.0F, properties.limits.pointSizeRange[0]);
        pointSizeRangeMax_ = std::max(pointSizeRangeMin_, properties.limits.pointSizeRange[1]);
        gpuTimestampPeriodNanoseconds_ =
            static_cast<double>(properties.limits.timestampPeriod);
        break;
    }

    if (physicalDevice_ == VK_NULL_HANDLE) {
        throw std::runtime_error{"No suitable Vulkan device was found for presentation."};
    }

    const auto selection = FindQueueFamilies(physicalDevice_, surface_);
    graphicsQueueFamily_ = selection.graphicsFamily.value();
    presentQueueFamily_ = selection.presentFamily.value();

    std::uint32_t queueFamilyCount = 0U;
    vkGetPhysicalDeviceQueueFamilyProperties(
        physicalDevice_,
        &queueFamilyCount,
        nullptr);
    std::vector<VkQueueFamilyProperties> queueFamilies(queueFamilyCount);
    vkGetPhysicalDeviceQueueFamilyProperties(
        physicalDevice_,
        &queueFamilyCount,
        queueFamilies.data());
    if (graphicsQueueFamily_ < queueFamilies.size()) {
        gpuTimestampValidBits_ =
            queueFamilies[graphicsQueueFamily_].timestampValidBits;
    }
    gpuTimestampsSupported_ =
        gpuTimestampValidBits_ > 0U &&
        gpuTimestampPeriodNanoseconds_ > 0.0;
    diagnostics_.gpuTimestampsSupported = gpuTimestampsSupported_;
    diagnostics_.gpuTimestampValidBits = gpuTimestampValidBits_;
    diagnostics_.gpuTimestampPeriodNanoseconds =
        gpuTimestampPeriodNanoseconds_;
}

void VulkanViewportShell::CreateLogicalDevice() {
    const std::set<std::uint32_t> uniqueFamilies = {graphicsQueueFamily_, presentQueueFamily_};

    float queuePriority = 1.0F;
    std::vector<VkDeviceQueueCreateInfo> queueCreateInfos;
    for (const auto family : uniqueFamilies) {
        VkDeviceQueueCreateInfo queueCreateInfo{VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO};
        queueCreateInfo.queueFamilyIndex = family;
        queueCreateInfo.queueCount = 1;
        queueCreateInfo.pQueuePriorities = &queuePriority;
        queueCreateInfos.push_back(queueCreateInfo);
    }

    std::vector<const char*> deviceExtensions(kRequiredDeviceExtensions.begin(), kRequiredDeviceExtensions.end());
    if (enablePortabilitySubset_) {
        deviceExtensions.push_back(VK_KHR_PORTABILITY_SUBSET_EXTENSION_NAME);
    }

    VkPhysicalDeviceFeatures features{};
    features.largePoints = VK_TRUE;
    features.shaderInt64 = VK_TRUE;

    VkDeviceCreateInfo createInfo{VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO};
    createInfo.queueCreateInfoCount = static_cast<std::uint32_t>(queueCreateInfos.size());
    createInfo.pQueueCreateInfos = queueCreateInfos.data();
    createInfo.pEnabledFeatures = &features;
    createInfo.enabledExtensionCount = static_cast<std::uint32_t>(deviceExtensions.size());
    createInfo.ppEnabledExtensionNames = deviceExtensions.data();

    Check(vkCreateDevice(physicalDevice_, &createInfo, nullptr, &device_), "vkCreateDevice");

    vkGetDeviceQueue(device_, graphicsQueueFamily_, 0, &graphicsQueue_);
    vkGetDeviceQueue(device_, presentQueueFamily_, 0, &presentQueue_);
}

void VulkanViewportShell::CreateSwapchain() {
    const auto support = QuerySwapchainSupport(physicalDevice_, surface_);
    const auto surfaceFormat = ChooseSurfaceFormat(support.formats);
    const auto presentMode = ChoosePresentMode(support.presentModes);
    const auto extent = ChooseExtent(window_, support.capabilities);

    std::uint32_t imageCount = support.capabilities.minImageCount + 1;
    if (support.capabilities.maxImageCount > 0 && imageCount > support.capabilities.maxImageCount) {
        imageCount = support.capabilities.maxImageCount;
    }
    requestedSwapchainImageCount_ = imageCount;
    selectedPresentMode_ = presentMode;

    const std::uint32_t queueFamilyIndices[] = {graphicsQueueFamily_, presentQueueFamily_};

    VkSwapchainCreateInfoKHR createInfo{VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR};
    createInfo.surface = surface_;
    createInfo.minImageCount = imageCount;
    createInfo.imageFormat = surfaceFormat.format;
    createInfo.imageColorSpace = surfaceFormat.colorSpace;
    createInfo.imageExtent = extent;
    createInfo.imageArrayLayers = 1;
    createInfo.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;

    if (graphicsQueueFamily_ != presentQueueFamily_) {
        createInfo.imageSharingMode = VK_SHARING_MODE_CONCURRENT;
        createInfo.queueFamilyIndexCount = 2;
        createInfo.pQueueFamilyIndices = queueFamilyIndices;
    } else {
        createInfo.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
    }

    createInfo.preTransform = support.capabilities.currentTransform;
    createInfo.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
    createInfo.presentMode = presentMode;
    createInfo.clipped = VK_TRUE;
    createInfo.oldSwapchain = VK_NULL_HANDLE;

    Check(vkCreateSwapchainKHR(device_, &createInfo, nullptr, &swapchain_), "vkCreateSwapchainKHR");

    std::uint32_t actualImageCount = 0;
    vkGetSwapchainImagesKHR(device_, swapchain_, &actualImageCount, nullptr);
    swapchainImages_.resize(actualImageCount);
    vkGetSwapchainImagesKHR(device_, swapchain_, &actualImageCount, swapchainImages_.data());

    swapchainImageFormat_ = surfaceFormat.format;
    swapchainWidth_ = extent.width;
    swapchainHeight_ = extent.height;
    swapchainImageOwners_.assign(swapchainImages_.size(), {});
    CreateSwapchainRenderFinishedSemaphores();

    std::ostringstream summary;
    summary << "Renderer: " << diagnostics_.rendererName << " | " << swapchainWidth_ << "x"
            << swapchainHeight_ << " | mixed scene Vulkan viewport";
    diagnostics_.summary = summary.str();
    diagnostics_.width = swapchainWidth_;
    diagnostics_.height = swapchainHeight_;
    diagnostics_.requestedSwapchainImageCount =
        requestedSwapchainImageCount_;
    diagnostics_.swapchainImageCount =
        static_cast<std::uint32_t>(swapchainImages_.size());
    diagnostics_.presentMode = PresentModeName(selectedPresentMode_);
}

void VulkanViewportShell::CreateImageViews() {
    imageViews_.clear();
    imageViews_.reserve(swapchainImages_.size());

    for (const auto image : swapchainImages_) {
        VkImageViewCreateInfo createInfo{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
        createInfo.image = image;
        createInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
        createInfo.format = swapchainImageFormat_;
        createInfo.components.r = VK_COMPONENT_SWIZZLE_IDENTITY;
        createInfo.components.g = VK_COMPONENT_SWIZZLE_IDENTITY;
        createInfo.components.b = VK_COMPONENT_SWIZZLE_IDENTITY;
        createInfo.components.a = VK_COMPONENT_SWIZZLE_IDENTITY;
        createInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        createInfo.subresourceRange.baseMipLevel = 0;
        createInfo.subresourceRange.levelCount = 1;
        createInfo.subresourceRange.baseArrayLayer = 0;
        createInfo.subresourceRange.layerCount = 1;

        VkImageView imageView = VK_NULL_HANDLE;
        Check(vkCreateImageView(device_, &createInfo, nullptr, &imageView), "vkCreateImageView");
        imageViews_.push_back(imageView);
    }
}

void VulkanViewportShell::CreateRenderPass() {
    accumulationFormat_ = SelectAccumulationFormat();
    revealageFormat_ = SelectRevealageFormat();
    linearDepthFormat_ = VK_FORMAT_R32_SFLOAT;

    VkAttachmentDescription colorAttachment{};
    colorAttachment.format = swapchainImageFormat_;
    colorAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
    colorAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    colorAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    colorAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    colorAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    colorAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    colorAttachment.finalLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

    VkAttachmentDescription depthAttachment{};
    depthAttachment.format = SelectDepthFormat();
    depthAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
    depthAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    depthAttachment.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    depthAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    depthAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    depthAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    depthAttachment.finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

    VkAttachmentDescription accumulationAttachment{};
    accumulationAttachment.format = accumulationFormat_;
    accumulationAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
    accumulationAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    accumulationAttachment.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    accumulationAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    accumulationAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    accumulationAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    accumulationAttachment.finalLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

    VkAttachmentDescription revealageAttachment{};
    revealageAttachment.format = revealageFormat_;
    revealageAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
    revealageAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    revealageAttachment.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    revealageAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    revealageAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    revealageAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    revealageAttachment.finalLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

    VkAttachmentDescription emissiveAttachment{};
    emissiveAttachment.format = accumulationFormat_;
    emissiveAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
    emissiveAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    emissiveAttachment.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    emissiveAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    emissiveAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    emissiveAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    emissiveAttachment.finalLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

    VkAttachmentDescription linearDepthAttachment{};
    linearDepthAttachment.format = linearDepthFormat_;
    linearDepthAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
    linearDepthAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    linearDepthAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    linearDepthAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    linearDepthAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    linearDepthAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    linearDepthAttachment.finalLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

    VkAttachmentReference subpass0ColorRef{5, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};

    VkAttachmentReference depthAttachmentRef{1, VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL};
    VkAttachmentReference depthReadOnlyAttachmentRef{1, VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL};

    VkSubpassDescription subpass0{};
    subpass0.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    subpass0.colorAttachmentCount = 1;
    subpass0.pColorAttachments = &subpass0ColorRef;
    subpass0.pDepthStencilAttachment = &depthAttachmentRef;

    VkAttachmentReference subpass1ColorRefs[3]{};
    subpass1ColorRefs[0] = {2, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
    subpass1ColorRefs[1] = {3, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
    subpass1ColorRefs[2] = {4, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
    VkAttachmentReference subpass1InputRef{1, VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL};

    VkSubpassDescription subpass1{};
    subpass1.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    subpass1.colorAttachmentCount = 3;
    subpass1.pColorAttachments = subpass1ColorRefs;
    subpass1.inputAttachmentCount = 1;
    subpass1.pInputAttachments = &subpass1InputRef;
    subpass1.pDepthStencilAttachment = &depthReadOnlyAttachmentRef;

    VkAttachmentReference subpass2ColorRef{0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
    VkAttachmentReference subpass2InputRefs[3]{};
    subpass2InputRefs[0] = {2, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL};
    subpass2InputRefs[1] = {3, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL};
    subpass2InputRefs[2] = {4, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL};

    VkSubpassDescription subpass2{};
    subpass2.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    subpass2.colorAttachmentCount = 1;
    subpass2.pColorAttachments = &subpass2ColorRef;
    subpass2.inputAttachmentCount = 3;
    subpass2.pInputAttachments = subpass2InputRefs;

    VkAttachmentReference subpass3ColorRef{0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
    VkSubpassDescription subpass3{};
    subpass3.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    subpass3.colorAttachmentCount = 1;
    subpass3.pColorAttachments = &subpass3ColorRef;
    subpass3.pDepthStencilAttachment = &depthAttachmentRef;

    // Sorted layers write colour conventionally and add emission into the
    // shared emission attachment (cleared at subpass start); the following
    // subpass applies the weighted path's exponential emission response on
    // top, so emissive levels are identical whichever transparency mode a
    // visual selects.
    VkAttachmentReference subpass4ColorRefs[2]{};
    subpass4ColorRefs[0] = {0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
    subpass4ColorRefs[1] = {4, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
    VkAttachmentReference subpass4DepthInputRef{
        1,
        VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL};
    VkSubpassDescription subpass4{};
    subpass4.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    subpass4.colorAttachmentCount = 2;
    subpass4.pColorAttachments = subpass4ColorRefs;
    subpass4.inputAttachmentCount = 1;
    subpass4.pInputAttachments = &subpass4DepthInputRef;
    subpass4.pDepthStencilAttachment = &depthReadOnlyAttachmentRef;

    VkAttachmentReference subpass5ColorRef{
        0,
        VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
    VkAttachmentReference subpass5EmissionInputRef{
        4,
        VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL};
    VkSubpassDescription subpass5{};
    subpass5.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    subpass5.colorAttachmentCount = 1;
    subpass5.pColorAttachments = &subpass5ColorRef;
    subpass5.inputAttachmentCount = 1;
    subpass5.pInputAttachments = &subpass5EmissionInputRef;

    std::array<VkSubpassDependency, 14> dependencies{};
    dependencies[0].srcSubpass = VK_SUBPASS_EXTERNAL;
    dependencies[0].dstSubpass = 0;
    dependencies[0].srcStageMask =
        VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
    dependencies[0].dstStageMask =
        VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
    dependencies[0].dstAccessMask =
        VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;

    dependencies[1].srcSubpass = 0;
    dependencies[1].dstSubpass = 1;
    dependencies[1].srcStageMask =
        VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT | VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;
    dependencies[1].dstStageMask =
        VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT |
        VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;
    dependencies[1].srcAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
    dependencies[1].dstAccessMask =
        VK_ACCESS_INPUT_ATTACHMENT_READ_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT;

    dependencies[2].srcSubpass = 0;
    dependencies[2].dstSubpass = 3;
    dependencies[2].srcStageMask =
        VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT | VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;
    dependencies[2].dstStageMask =
        VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT | VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;
    dependencies[2].srcAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
    dependencies[2].dstAccessMask =
        VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;

    dependencies[3].srcSubpass = 1;
    dependencies[3].dstSubpass = 2;
    dependencies[3].srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dependencies[3].dstStageMask =
        VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT | VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dependencies[3].srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
    dependencies[3].dstAccessMask =
        VK_ACCESS_INPUT_ATTACHMENT_READ_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;

    dependencies[4].srcSubpass = 2;
    dependencies[4].dstSubpass = 3;
    dependencies[4].srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dependencies[4].dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dependencies[4].srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
    dependencies[4].dstAccessMask =
        VK_ACCESS_COLOR_ATTACHMENT_READ_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;

    dependencies[5].srcSubpass = 3;
    dependencies[5].dstSubpass = VK_SUBPASS_EXTERNAL;
    dependencies[5].srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dependencies[5].dstStageMask = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
    dependencies[5].srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
    dependencies[5].dstAccessMask = VK_ACCESS_SHADER_READ_BIT;

    dependencies[6].srcSubpass = 0;
    dependencies[6].dstSubpass = VK_SUBPASS_EXTERNAL;
    dependencies[6].srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dependencies[6].dstStageMask = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
    dependencies[6].srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
    dependencies[6].dstAccessMask = VK_ACCESS_SHADER_READ_BIT;

    dependencies[7].srcSubpass = 1;
    dependencies[7].dstSubpass = 3;
    dependencies[7].srcStageMask =
        VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT | VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;
    dependencies[7].dstStageMask =
        VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT | VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;
    dependencies[7].srcAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT;
    dependencies[7].dstAccessMask =
        VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;

    dependencies[8].srcSubpass = 3;
    dependencies[8].dstSubpass = 4;
    dependencies[8].srcStageMask =
        VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT |
        VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT |
        VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;
    dependencies[8].dstStageMask =
        VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT |
        VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT |
        VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
    dependencies[8].srcAccessMask =
        VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT |
        VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
    dependencies[8].dstAccessMask =
        VK_ACCESS_INPUT_ATTACHMENT_READ_BIT |
        VK_ACCESS_COLOR_ATTACHMENT_READ_BIT |
        VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT |
        VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT;

    // Subpass 3 may be empty when the scene has no Fast Basic/opaque layer.
    // Synchronize the original depth prepass directly as well, so the sorted
    // fragment shader always sees its core-depth writes as an input attachment.
    dependencies[9].srcSubpass = 0;
    dependencies[9].dstSubpass = 4;
    dependencies[9].srcStageMask =
        VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT |
        VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;
    dependencies[9].dstStageMask =
        VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT |
        VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
    dependencies[9].srcAccessMask =
        VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
    dependencies[9].dstAccessMask =
        VK_ACCESS_INPUT_ATTACHMENT_READ_BIT |
        VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT;

    dependencies[10].srcSubpass = 4;
    dependencies[10].dstSubpass = VK_SUBPASS_EXTERNAL;
    dependencies[10].srcStageMask =
        VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dependencies[10].dstStageMask = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
    dependencies[10].srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
    dependencies[10].dstAccessMask = VK_ACCESS_SHADER_READ_BIT;

    // The weighted composite consumed the emission attachment as an input in
    // subpass 2; the sorted subpass clears and rewrites it as colour output.
    dependencies[11].srcSubpass = 2;
    dependencies[11].dstSubpass = 4;
    dependencies[11].srcStageMask = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
    dependencies[11].dstStageMask =
        VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dependencies[11].srcAccessMask = VK_ACCESS_INPUT_ATTACHMENT_READ_BIT;
    dependencies[11].dstAccessMask =
        VK_ACCESS_COLOR_ATTACHMENT_READ_BIT |
        VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;

    dependencies[12].srcSubpass = 4;
    dependencies[12].dstSubpass = 5;
    dependencies[12].srcStageMask =
        VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dependencies[12].dstStageMask =
        VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT |
        VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dependencies[12].srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
    dependencies[12].dstAccessMask =
        VK_ACCESS_INPUT_ATTACHMENT_READ_BIT |
        VK_ACCESS_COLOR_ATTACHMENT_READ_BIT |
        VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;

    dependencies[13].srcSubpass = 5;
    dependencies[13].dstSubpass = VK_SUBPASS_EXTERNAL;
    dependencies[13].srcStageMask =
        VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dependencies[13].dstStageMask = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
    dependencies[13].srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
    dependencies[13].dstAccessMask = VK_ACCESS_SHADER_READ_BIT;

    const std::array<VkAttachmentDescription, 6> attachments = {
        colorAttachment,
        depthAttachment,
        accumulationAttachment,
        revealageAttachment,
        emissiveAttachment,
        linearDepthAttachment,
    };
    const std::array<VkSubpassDescription, 6> subpasses = {
        subpass0,
        subpass1,
        subpass2,
        subpass3,
        subpass4,
        subpass5};

    VkRenderPassCreateInfo renderPassInfo{VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO};
    renderPassInfo.attachmentCount = static_cast<std::uint32_t>(attachments.size());
    renderPassInfo.pAttachments = attachments.data();
    renderPassInfo.subpassCount = static_cast<std::uint32_t>(subpasses.size());
    renderPassInfo.pSubpasses = subpasses.data();
    renderPassInfo.dependencyCount = static_cast<std::uint32_t>(dependencies.size());
    renderPassInfo.pDependencies = dependencies.data();

    Check(vkCreateRenderPass(device_, &renderPassInfo, nullptr, &renderPass_), "vkCreateRenderPass");
}

void VulkanViewportShell::CreatePresentRenderPass() {
    VkAttachmentDescription colorAttachment{};
    colorAttachment.format = swapchainImageFormat_;
    colorAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
    colorAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    colorAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    colorAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    colorAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    colorAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    colorAttachment.finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;

    VkAttachmentReference colorRef{0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};

    VkSubpassDescription subpass{};
    subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    subpass.colorAttachmentCount = 1;
    subpass.pColorAttachments = &colorRef;

    std::array<VkSubpassDependency, 2> dependencies{};
    dependencies[0].srcSubpass = VK_SUBPASS_EXTERNAL;
    dependencies[0].dstSubpass = 0;
    dependencies[0].srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dependencies[0].dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dependencies[0].dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;

    dependencies[1].srcSubpass = 0;
    dependencies[1].dstSubpass = VK_SUBPASS_EXTERNAL;
    dependencies[1].srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dependencies[1].dstStageMask = VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT;
    dependencies[1].srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;

    VkRenderPassCreateInfo renderPassInfo{VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO};
    renderPassInfo.attachmentCount = 1;
    renderPassInfo.pAttachments = &colorAttachment;
    renderPassInfo.subpassCount = 1;
    renderPassInfo.pSubpasses = &subpass;
    renderPassInfo.dependencyCount = static_cast<std::uint32_t>(dependencies.size());
    renderPassInfo.pDependencies = dependencies.data();

    Check(vkCreateRenderPass(device_, &renderPassInfo, nullptr, &presentRenderPass_), "vkCreateRenderPass(present)");
}

void VulkanViewportShell::CreatePointDescriptorSetLayout() {
    // Storage-buffer bindings: 4-6 point data, 10-12/16 Seepage, 13-15 rain
    // impact, 17-18 Mesh Flow contact. Bindings 7-9 were the removed sparse
    // Ripple buffers and are intentionally left unbound so the Seepage, rain
    // and Mesh Flow GLSL binding numbers stay unchanged.
    constexpr std::array<std::uint32_t, 12> kPointStorageBindings{
        4U, 5U, 6U, 10U, 11U, 12U, 13U, 14U, 15U, 16U, 17U, 18U};
    std::array<VkDescriptorSetLayoutBinding, 4U + kPointStorageBindings.size()> bindings{};
    bindings[0].binding = 0;
    bindings[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    bindings[0].descriptorCount = 1;
    bindings[0].stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
    bindings[1].binding = 1;
    bindings[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    bindings[1].descriptorCount = 1;
    bindings[1].stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
    bindings[2].binding = 2;
    bindings[2].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    bindings[2].descriptorCount = 1;
    bindings[2].stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
    bindings[3].binding = 3;
    bindings[3].descriptorType = VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT;
    bindings[3].descriptorCount = 1;
    bindings[3].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
    for (std::size_t storageIndex = 0; storageIndex < kPointStorageBindings.size(); ++storageIndex) {
        auto& binding = bindings[4U + storageIndex];
        binding.binding = kPointStorageBindings[storageIndex];
        binding.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        binding.descriptorCount = 1;
        binding.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
    }

    VkDescriptorSetLayoutCreateInfo layoutInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
    layoutInfo.bindingCount = static_cast<std::uint32_t>(bindings.size());
    layoutInfo.pBindings = bindings.data();

    Check(
        vkCreateDescriptorSetLayout(device_, &layoutInfo, nullptr, &pointDescriptorSetLayout_),
        "vkCreateDescriptorSetLayout(point)");
}

void VulkanViewportShell::CreatePointDepthSortDescriptorSetLayout() {
    std::array<VkDescriptorSetLayoutBinding, 5> bindings{};
    for (std::uint32_t bindingIndex = 0U;
         bindingIndex < bindings.size();
         ++bindingIndex) {
        bindings[bindingIndex].binding = bindingIndex;
        bindings[bindingIndex].descriptorType =
            VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        bindings[bindingIndex].descriptorCount = 1U;
        bindings[bindingIndex].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    }
    VkDescriptorSetLayoutCreateInfo layoutInfo{
        VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
    layoutInfo.bindingCount = static_cast<std::uint32_t>(bindings.size());
    layoutInfo.pBindings = bindings.data();
    Check(
        vkCreateDescriptorSetLayout(
            device_,
            &layoutInfo,
            nullptr,
            &pointDepthSortDescriptorSetLayout_),
        "vkCreateDescriptorSetLayout(point depth sort)");
}

void VulkanViewportShell::CreateDynamicMeshFlowDescriptorSetLayout() {
    std::array<VkDescriptorSetLayoutBinding, 11> bindings{};
    for (std::uint32_t bindingIndex = 0; bindingIndex < bindings.size(); ++bindingIndex) {
        bindings[bindingIndex].binding = bindingIndex;
        bindings[bindingIndex].descriptorCount = 1;
        bindings[bindingIndex].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
        bindings[bindingIndex].descriptorType =
            bindingIndex == 0 ? VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER : VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    }

    VkDescriptorSetLayoutCreateInfo layoutInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
    layoutInfo.bindingCount = static_cast<std::uint32_t>(bindings.size());
    layoutInfo.pBindings = bindings.data();
    Check(
        vkCreateDescriptorSetLayout(device_, &layoutInfo, nullptr, &dynamicMeshFlowDescriptorSetLayout_),
        "vkCreateDescriptorSetLayout(dynamic mesh flow)");
}

void VulkanViewportShell::CreateWaterFlowSourceDescriptorSetLayout() {
    std::array<VkDescriptorSetLayoutBinding, 7> bindings{};
    for (std::uint32_t bindingIndex = 0U; bindingIndex < bindings.size(); ++bindingIndex) {
        bindings[bindingIndex].binding = bindingIndex;
        bindings[bindingIndex].descriptorCount = 1U;
        bindings[bindingIndex].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
        bindings[bindingIndex].descriptorType =
            bindingIndex == 0U ? VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER : VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    }

    VkDescriptorSetLayoutCreateInfo layoutInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
    layoutInfo.bindingCount = static_cast<std::uint32_t>(bindings.size());
    layoutInfo.pBindings = bindings.data();
    Check(
        vkCreateDescriptorSetLayout(device_, &layoutInfo, nullptr, &waterFlowSourceDescriptorSetLayout_),
        "vkCreateDescriptorSetLayout(water flow source)");
}

void VulkanViewportShell::CreateRainDescriptorSetLayout() {
    std::array<VkDescriptorSetLayoutBinding, 9> bindings{};
    for (std::uint32_t bindingIndex = 0; bindingIndex < bindings.size(); ++bindingIndex) {
        bindings[bindingIndex].binding = bindingIndex;
        bindings[bindingIndex].descriptorCount = 1;
        bindings[bindingIndex].descriptorType =
            bindingIndex == 0U ? VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER : VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        bindings[bindingIndex].stageFlags =
            VK_SHADER_STAGE_COMPUTE_BIT | VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
    }

    VkDescriptorSetLayoutCreateInfo layoutInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
    layoutInfo.bindingCount = static_cast<std::uint32_t>(bindings.size());
    layoutInfo.pBindings = bindings.data();
    Check(
        vkCreateDescriptorSetLayout(device_, &layoutInfo, nullptr, &rainDescriptorSetLayout_),
        "vkCreateDescriptorSetLayout(rain)");
}

void VulkanViewportShell::CreateWaterSurfacePreprocessDescriptorSetLayout() {
    std::array<VkDescriptorSetLayoutBinding, 3> bindings{};
    bindings[0] = {
        0U,
        VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,
        1U,
        VK_SHADER_STAGE_COMPUTE_BIT,
        nullptr};
    for (std::uint32_t binding = 1U; binding < bindings.size(); ++binding) {
        bindings[binding] = {
            binding,
            VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
            1U,
            VK_SHADER_STAGE_COMPUTE_BIT,
            nullptr};
    }

    VkDescriptorSetLayoutCreateInfo layoutInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
    layoutInfo.bindingCount = static_cast<std::uint32_t>(bindings.size());
    layoutInfo.pBindings = bindings.data();
    Check(
        vkCreateDescriptorSetLayout(
            device_,
            &layoutInfo,
            nullptr,
            &waterSurfacePreprocessDescriptorSetLayout_),
        "vkCreateDescriptorSetLayout(water surface preprocess)");
}

void VulkanViewportShell::CreateGaussianSplatDescriptorSetLayout() {
    std::array<VkDescriptorSetLayoutBinding, 7> bindings{};
    bindings[0] = {0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, nullptr};
    for (std::uint32_t index = 1; index < 6; ++index) {
        bindings[index].binding = index;
        bindings[index].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        bindings[index].descriptorCount = 1;
        bindings[index].stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
    }
    bindings[6] = {6, VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr};

    VkDescriptorSetLayoutCreateInfo layoutInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
    layoutInfo.bindingCount = static_cast<std::uint32_t>(bindings.size());
    layoutInfo.pBindings = bindings.data();

    Check(
        vkCreateDescriptorSetLayout(device_, &layoutInfo, nullptr, &gaussianSplatDescriptorSetLayout_),
        "vkCreateDescriptorSetLayout(gsplat)");
}

void VulkanViewportShell::CreateHighQualityGaussianSplatDescriptorSetLayout() {
    std::array<VkDescriptorSetLayoutBinding, 9> bindings{};
    for (std::uint32_t bindingIndex = 0; bindingIndex < bindings.size(); ++bindingIndex) {
        bindings[bindingIndex].binding = bindingIndex;
        bindings[bindingIndex].descriptorCount = 1;
        bindings[bindingIndex].stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
        bindings[bindingIndex].descriptorType =
            bindingIndex == 0 ? VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER : VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    }

    VkDescriptorSetLayoutCreateInfo layoutInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
    layoutInfo.bindingCount = static_cast<std::uint32_t>(bindings.size());
    layoutInfo.pBindings = bindings.data();

    Check(
        vkCreateDescriptorSetLayout(
            device_,
            &layoutInfo,
            nullptr,
            &highQualityGaussianSplatDescriptorSetLayout_),
        "vkCreateDescriptorSetLayout(gsplat_hq)");
}

void VulkanViewportShell::CreateCompositeDescriptorSetLayout() {
    std::array<VkDescriptorSetLayoutBinding, 5> bindings{};
    for (std::uint32_t bindingIndex = 0; bindingIndex < bindings.size(); ++bindingIndex) {
        bindings[bindingIndex] = {
            bindingIndex,
            VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT,
            1,
            VK_SHADER_STAGE_FRAGMENT_BIT,
            nullptr};
    }

    VkDescriptorSetLayoutCreateInfo layoutInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
    layoutInfo.bindingCount = static_cast<std::uint32_t>(bindings.size());
    layoutInfo.pBindings = bindings.data();

    Check(
        vkCreateDescriptorSetLayout(device_, &layoutInfo, nullptr, &compositeDescriptorSetLayout_),
        "vkCreateDescriptorSetLayout(composite)");
}

void VulkanViewportShell::CreatePostProcessDescriptorSetLayout() {
    std::array<VkDescriptorSetLayoutBinding, 6> bindings{};
    for (std::uint32_t bindingIndex = 0U;
         bindingIndex < bindings.size();
         ++bindingIndex) {
        bindings[bindingIndex] = {
            bindingIndex,
            VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
            1,
            VK_SHADER_STAGE_FRAGMENT_BIT,
            nullptr};
    }

    VkDescriptorSetLayoutCreateInfo layoutInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
    layoutInfo.bindingCount = static_cast<std::uint32_t>(bindings.size());
    layoutInfo.pBindings = bindings.data();

    Check(
        vkCreateDescriptorSetLayout(device_, &layoutInfo, nullptr, &postProcessDescriptorSetLayout_),
        "vkCreateDescriptorSetLayout(postprocess)");
}

void VulkanViewportShell::CreateEyeDomeLightingExportDescriptorSetLayout() {
    const std::array<VkDescriptorSetLayoutBinding, 2> bindings = {
        VkDescriptorSetLayoutBinding{
            0,
            VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,
            1,
            VK_SHADER_STAGE_COMPUTE_BIT,
            nullptr},
        VkDescriptorSetLayoutBinding{
            1,
            VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
            1,
            VK_SHADER_STAGE_COMPUTE_BIT,
            nullptr},
    };

    VkDescriptorSetLayoutCreateInfo layoutInfo{
        VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
    layoutInfo.bindingCount = static_cast<std::uint32_t>(bindings.size());
    layoutInfo.pBindings = bindings.data();

    Check(
        vkCreateDescriptorSetLayout(
            device_,
            &layoutInfo,
            nullptr,
            &eyeDomeLightingExportDescriptorSetLayout_),
        "vkCreateDescriptorSetLayout(export EDL)");
}

void VulkanViewportShell::CreateDescriptorPools() {
    const std::array<VkDescriptorPoolSize, 4> poolSizes = {
        MakePoolSize(VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 8192),
        MakePoolSize(VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 32768),
        MakePoolSize(VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT, 8192),
        MakePoolSize(VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 8192),
    };

    VkDescriptorPoolCreateInfo poolInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
    poolInfo.flags = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;
    // Non-point compute/composite generations still share this long-lived
    // pool. Point rendering uses short-lived generation-owned pools below, so
    // density switches no longer consume this headroom.
    poolInfo.maxSets = 8192;
    poolInfo.poolSizeCount = static_cast<std::uint32_t>(poolSizes.size());
    poolInfo.pPoolSizes = poolSizes.data();
    Check(vkCreateDescriptorPool(device_, &poolInfo, nullptr, &descriptorPool_), "vkCreateDescriptorPool");

    const std::array<VkDescriptorPoolSize, 3> gsplatPoolSizes = {
        MakePoolSize(VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1024),
        MakePoolSize(VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 4096),
        MakePoolSize(VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT, 1024),
    };

    VkDescriptorPoolCreateInfo gsplatPoolInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
    gsplatPoolInfo.flags = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;
    gsplatPoolInfo.maxSets = 1024;
    gsplatPoolInfo.poolSizeCount = static_cast<std::uint32_t>(gsplatPoolSizes.size());
    gsplatPoolInfo.pPoolSizes = gsplatPoolSizes.data();
    Check(
        vkCreateDescriptorPool(device_, &gsplatPoolInfo, nullptr, &gaussianSplatDescriptorPool_),
        "vkCreateDescriptorPool(gsplat)");
}

void VulkanViewportShell::CreatePostProcessSampler() {
    VkSamplerCreateInfo samplerInfo{VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO};
    samplerInfo.magFilter = VK_FILTER_NEAREST;
    samplerInfo.minFilter = VK_FILTER_NEAREST;
    samplerInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
    samplerInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    samplerInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    samplerInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    samplerInfo.minLod = 0.0F;
    samplerInfo.maxLod = 0.0F;
    samplerInfo.borderColor = VK_BORDER_COLOR_FLOAT_OPAQUE_BLACK;

    Check(vkCreateSampler(device_, &samplerInfo, nullptr, &postProcessSampler_), "vkCreateSampler(postprocess)");
}

void VulkanViewportShell::CreateUniformResources() {
    for (auto& frame : frameResources_) {
        DestroyBuffer(&frame.uniformBuffer);
        frame.uniformBuffer = CreateHostVisibleBuffer(sizeof(FrameUniforms), VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT);
    }
}

void VulkanViewportShell::CreateRainResources() {
    auto& resources = rainResources_;
    resources.surfaceTableBuffer = CreateHostVisibleBuffer(
        2U * sizeof(invisible_places::water::RainGpuSurfaceSlot),
        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
    resources.vegetationTableBuffer = CreateHostVisibleBuffer(
        2U * sizeof(invisible_places::water::RainGpuVegetationSlot),
        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
    resources.flowSurfaceTableBuffer = CreateHostVisibleBuffer(
        2U * sizeof(invisible_places::water::WaterGpuSurfaceSurfelSlot),
        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
    resources.dynamicMeshFlowRainSeedBuffer =
        CreateHostVisibleBuffer(
            sizeof(DynamicMeshFlowRainSeedHeaderGpu) +
                static_cast<VkDeviceSize>(
                    kDynamicMeshFlowRainSeedCapacity) *
                    sizeof(DynamicMeshFlowRainSeedGpu),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
    resources.dynamicMeshFlowDummyContactEventBuffer =
        CreateHostVisibleBuffer(
            sizeof(DynamicMeshFlowContactEventGpu),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
    resources.dynamicMeshFlowDummyContactGridBuffer =
        CreateHostVisibleBuffer(
            sizeof(glm::uvec4),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
    resources.particleBuffer = CreateHostVisibleBuffer(
        static_cast<VkDeviceSize>(invisible_places::water::kRainParticleCapacity) * sizeof(RainParticleGpu),
        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
    resources.eventBuffer = CreateHostVisibleBuffer(
        static_cast<VkDeviceSize>(invisible_places::water::kRainImpactEventCapacity) * sizeof(RainImpactEventGpu),
        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
    resources.counterBuffer = CreateHostVisibleBuffer(
        sizeof(RainCountersGpu),
        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT);
    for (auto& counterReadback : resources.counterReadbackBuffers) {
        counterReadback = CreateHostVisibleBuffer(
            sizeof(RainCountersGpu),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT);
    }
    resources.impactCountBuffer = CreateHostVisibleBuffer(
        static_cast<VkDeviceSize>(invisible_places::water::kRainImpactGridDimension) *
            invisible_places::water::kRainImpactGridDimension * sizeof(glm::uvec4),
        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
    resources.impactReferenceBuffer = CreateHostVisibleBuffer(
        static_cast<VkDeviceSize>(invisible_places::water::kRainImpactGridDimension) *
            invisible_places::water::kRainImpactGridDimension *
            (invisible_places::water::kRainSandEventsPerCell +
             invisible_places::water::kRainRockEventsPerCell +
             invisible_places::water::kRainVegetationEventsPerCell) * sizeof(std::uint32_t),
        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
    for (auto& uniformBuffer : resources.uniformBuffers) {
        uniformBuffer = CreateHostVisibleBuffer(sizeof(RainUniformsGpu), VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT);
    }
    resources.exrUniformBuffer = CreateHostVisibleBuffer(
        sizeof(RainUniformsGpu),
        VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT);

    std::memset(resources.surfaceTableBuffer.mapped, 0, static_cast<std::size_t>(resources.surfaceTableBuffer.size));
    std::memset(resources.vegetationTableBuffer.mapped, 0, static_cast<std::size_t>(resources.vegetationTableBuffer.size));
    std::memset(
        resources.flowSurfaceTableBuffer.mapped,
        0,
        static_cast<std::size_t>(resources.flowSurfaceTableBuffer.size));
    std::memset(
        resources.dynamicMeshFlowRainSeedBuffer.mapped,
        0,
        static_cast<std::size_t>(
            resources.dynamicMeshFlowRainSeedBuffer.size));
    std::memset(
        resources.dynamicMeshFlowDummyContactEventBuffer.mapped,
        0,
        static_cast<std::size_t>(
            resources.dynamicMeshFlowDummyContactEventBuffer.size));
    std::memset(
        resources.dynamicMeshFlowDummyContactGridBuffer.mapped,
        0,
        static_cast<std::size_t>(
            resources.dynamicMeshFlowDummyContactGridBuffer.size));
    std::memset(resources.particleBuffer.mapped, 0, static_cast<std::size_t>(resources.particleBuffer.size));
    std::memset(resources.eventBuffer.mapped, 0, static_cast<std::size_t>(resources.eventBuffer.size));
    std::memset(resources.counterBuffer.mapped, 0, static_cast<std::size_t>(resources.counterBuffer.size));
    for (auto& counterReadback : resources.counterReadbackBuffers) {
        std::memset(
            counterReadback.mapped,
            0,
            static_cast<std::size_t>(counterReadback.size));
    }
    std::memset(resources.impactCountBuffer.mapped, 0, static_cast<std::size_t>(resources.impactCountBuffer.size));
    std::memset(
        resources.impactReferenceBuffer.mapped,
        0,
        static_cast<std::size_t>(resources.impactReferenceBuffer.size));

    std::array<VkDescriptorSetLayout, kFramesInFlight> layouts{};
    layouts.fill(rainDescriptorSetLayout_);
    const std::array<VkDescriptorPoolSize, 2> livePoolSizes = {
        MakePoolSize(
            VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,
            static_cast<std::uint32_t>(kFramesInFlight)),
        MakePoolSize(
            VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
            static_cast<std::uint32_t>(kFramesInFlight * 8U)),
    };
    VkDescriptorPoolCreateInfo livePoolInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
    livePoolInfo.maxSets = static_cast<std::uint32_t>(kFramesInFlight);
    livePoolInfo.poolSizeCount = static_cast<std::uint32_t>(livePoolSizes.size());
    livePoolInfo.pPoolSizes = livePoolSizes.data();
    Check(
        vkCreateDescriptorPool(
            device_,
            &livePoolInfo,
            nullptr,
            &resources.descriptorPool),
        "vkCreateDescriptorPool(rain generation)");
    VkDescriptorSetAllocateInfo allocInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
    allocInfo.descriptorPool = resources.descriptorPool;
    allocInfo.descriptorSetCount = static_cast<std::uint32_t>(layouts.size());
    allocInfo.pSetLayouts = layouts.data();
    Check(
        vkAllocateDescriptorSets(device_, &allocInfo, resources.descriptorSets.data()),
        "vkAllocateDescriptorSets(rain)");

    const std::array<VkDescriptorPoolSize, 2> exrPoolSizes = {
        MakePoolSize(VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1U),
        MakePoolSize(VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 8U),
    };
    VkDescriptorPoolCreateInfo exrPoolInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
    exrPoolInfo.maxSets = 1U;
    exrPoolInfo.poolSizeCount = static_cast<std::uint32_t>(exrPoolSizes.size());
    exrPoolInfo.pPoolSizes = exrPoolSizes.data();
    Check(
        vkCreateDescriptorPool(
            device_,
            &exrPoolInfo,
            nullptr,
            &resources.exrDescriptorPool),
        "vkCreateDescriptorPool(rain EXR generation)");
    VkDescriptorSetAllocateInfo exrAllocInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
    exrAllocInfo.descriptorPool = resources.exrDescriptorPool;
    exrAllocInfo.descriptorSetCount = 1U;
    exrAllocInfo.pSetLayouts = &rainDescriptorSetLayout_;
    Check(
        vkAllocateDescriptorSets(device_, &exrAllocInfo, &resources.exrDescriptorSet),
        "vkAllocateDescriptorSets(rain exr)");

    UpdateRainDescriptorSets();
    UpdateRainExrDescriptorSet(
        resources.surfaceTableBuffer,
        resources.vegetationTableBuffer);

    diagnostics_.rainParticleCapacity = invisible_places::water::kRainParticleCapacity;
    diagnostics_.rainEventCapacity = invisible_places::water::kRainImpactEventCapacity;
}

void VulkanViewportShell::CleanupRainResources() {
    auto& resources = rainResources_;
    CleanupWaterSurfacePendingUpload(&resources.pendingSurfaceUpload);
    for (auto& abandoned : resources.abandonedSurfaceUploads) {
        CleanupWaterSurfacePendingUpload(&abandoned);
    }
    resources.abandonedSurfaceUploads.clear();
    for (auto& retired : resources.retiredSurfaceResources) {
        CleanupWaterSurfaceRetiredResources(&retired);
    }
    resources.retiredSurfaceResources.clear();
    resources.descriptorSets.fill(VK_NULL_HANDLE);
    resources.exrDescriptorSet = VK_NULL_HANDLE;
    if (resources.descriptorPool != VK_NULL_HANDLE) {
        vkDestroyDescriptorPool(device_, resources.descriptorPool, nullptr);
        resources.descriptorPool = VK_NULL_HANDLE;
    }
    if (resources.exrDescriptorPool != VK_NULL_HANDLE) {
        vkDestroyDescriptorPool(device_, resources.exrDescriptorPool, nullptr);
        resources.exrDescriptorPool = VK_NULL_HANDLE;
    }
    for (auto& uniformBuffer : resources.uniformBuffers) {
        DestroyBuffer(&uniformBuffer);
    }
    DestroyBuffer(&resources.exrUniformBuffer);
    for (auto& counterReadback : resources.counterReadbackBuffers) {
        DestroyBuffer(&counterReadback);
    }
    DestroyBuffer(&resources.surfaceTableBuffer);
    DestroyBuffer(&resources.vegetationTableBuffer);
    DestroyBuffer(&resources.flowSurfaceTableBuffer);
    DestroyBuffer(&resources.groundEntryCandidateBuffer);
    DestroyBuffer(&resources.dynamicMeshFlowRainSeedBuffer);
    DestroyBuffer(&resources.dynamicMeshFlowDummyContactEventBuffer);
    DestroyBuffer(&resources.dynamicMeshFlowDummyContactGridBuffer);
    DestroyBuffer(&resources.groundTableBuffer);
    DestroyBuffer(&resources.particleBuffer);
    DestroyBuffer(&resources.eventBuffer);
    DestroyBuffer(&resources.counterBuffer);
    DestroyBuffer(&resources.impactCountBuffer);
    DestroyBuffer(&resources.impactReferenceBuffer);
    resources = {};
}

void VulkanViewportShell::CleanupWaterSurfacePendingUpload(
    WaterSurfacePendingGpuUpload* pending) {
    if (pending == nullptr) {
        return;
    }
    if (pending->preprocessCommandBuffer != VK_NULL_HANDLE &&
        commandPool_ != VK_NULL_HANDLE) {
        vkFreeCommandBuffers(
            device_,
            commandPool_,
            1U,
            &pending->preprocessCommandBuffer);
    }
    if (pending->preprocessFence != VK_NULL_HANDLE) {
        vkDestroyFence(device_, pending->preprocessFence, nullptr);
    }
    pending->preprocessDescriptorSet = VK_NULL_HANDLE;
    pending->rainDescriptorSets.fill(VK_NULL_HANDLE);
    if (pending->preprocessDescriptorPool != VK_NULL_HANDLE) {
        vkDestroyDescriptorPool(device_, pending->preprocessDescriptorPool, nullptr);
        pending->preprocessDescriptorPool = VK_NULL_HANDLE;
    }
    if (pending->rainDescriptorPool != VK_NULL_HANDLE) {
        vkDestroyDescriptorPool(device_, pending->rainDescriptorPool, nullptr);
        pending->rainDescriptorPool = VK_NULL_HANDLE;
    }
    DestroyBuffer(&pending->surfaceTableBuffer);
    DestroyBuffer(&pending->vegetationTableBuffer);
    DestroyBuffer(&pending->flowSurfaceInputBuffer);
    DestroyBuffer(&pending->flowSurfaceTableBuffer);
    DestroyBuffer(&pending->groundTableBuffer);
    DestroyBuffer(&pending->groundEntryCandidateBuffer);
    DestroyBuffer(&pending->preprocessUniformBuffer);
    *pending = {};
}

void VulkanViewportShell::CleanupWaterSurfaceRetiredResources(
    WaterSurfaceRetiredGpuResources* retired) {
    if (retired == nullptr) {
        return;
    }
    retired->rainDescriptorSets.fill(VK_NULL_HANDLE);
    if (retired->rainDescriptorPool != VK_NULL_HANDLE) {
        vkDestroyDescriptorPool(device_, retired->rainDescriptorPool, nullptr);
        retired->rainDescriptorPool = VK_NULL_HANDLE;
    }
    DestroyBuffer(&retired->surfaceTableBuffer);
    DestroyBuffer(&retired->vegetationTableBuffer);
    DestroyBuffer(&retired->flowSurfaceTableBuffer);
    DestroyBuffer(&retired->groundTableBuffer);
    DestroyBuffer(&retired->groundEntryCandidateBuffer);
    *retired = {};
}

void VulkanViewportShell::UpdateRainDescriptorSets() {
    UpdateRainDescriptorSets(
        rainResources_.descriptorSets,
        rainResources_.surfaceTableBuffer,
        rainResources_.vegetationTableBuffer);
}

void VulkanViewportShell::UpdateRainDescriptorSets(
    const std::array<VkDescriptorSet, kFramesInFlight>& descriptorSets,
    const BufferAllocation& surfaceTableBuffer,
    const BufferAllocation& vegetationTableBuffer) {
    const auto& resources = rainResources_;
    for (std::size_t frameIndex = 0; frameIndex < kFramesInFlight; ++frameIndex) {
        const auto descriptorSet = descriptorSets[frameIndex];
        if (descriptorSet == VK_NULL_HANDLE) {
            continue;
        }
        std::array<VkDescriptorBufferInfo, 9> infos{};
        infos[0] = {resources.uniformBuffers[frameIndex].buffer, 0, sizeof(RainUniformsGpu)};
        infos[1] = {resources.particleBuffer.buffer, 0, resources.particleBuffer.size};
        infos[2] = {resources.eventBuffer.buffer, 0, resources.eventBuffer.size};
        infos[3] = {resources.counterBuffer.buffer, 0, resources.counterBuffer.size};
        infos[4] = {surfaceTableBuffer.buffer, 0, surfaceTableBuffer.size};
        infos[5] = {vegetationTableBuffer.buffer, 0, vegetationTableBuffer.size};
        infos[6] = {resources.impactCountBuffer.buffer, 0, resources.impactCountBuffer.size};
        infos[7] = {resources.impactReferenceBuffer.buffer, 0, resources.impactReferenceBuffer.size};
        infos[8] = {
            resources.dynamicMeshFlowRainSeedBuffer.buffer,
            0,
            resources.dynamicMeshFlowRainSeedBuffer.size};
        std::array<VkWriteDescriptorSet, 9> writes{};
        for (std::uint32_t binding = 0U; binding < writes.size(); ++binding) {
            writes[binding] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
            writes[binding].dstSet = descriptorSet;
            writes[binding].dstBinding = binding;
            writes[binding].descriptorType =
                binding == 0U ? VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER : VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
            writes[binding].descriptorCount = 1U;
            writes[binding].pBufferInfo = &infos[binding];
        }
        vkUpdateDescriptorSets(
            device_,
            static_cast<std::uint32_t>(writes.size()),
            writes.data(),
            0U,
            nullptr);
    }
}

void VulkanViewportShell::UpdateRainExrDescriptorSet(
    const BufferAllocation& surfaceTableBuffer,
    const BufferAllocation& vegetationTableBuffer) {
    const auto& resources = rainResources_;
    if (resources.exrDescriptorSet == VK_NULL_HANDLE) {
        return;
    }
    std::array<VkDescriptorBufferInfo, 9> infos{};
    infos[0] = {resources.exrUniformBuffer.buffer, 0, sizeof(RainUniformsGpu)};
    infos[1] = {resources.particleBuffer.buffer, 0, resources.particleBuffer.size};
    infos[2] = {resources.eventBuffer.buffer, 0, resources.eventBuffer.size};
    infos[3] = {resources.counterBuffer.buffer, 0, resources.counterBuffer.size};
    infos[4] = {surfaceTableBuffer.buffer, 0, surfaceTableBuffer.size};
    infos[5] = {vegetationTableBuffer.buffer, 0, vegetationTableBuffer.size};
    infos[6] = {resources.impactCountBuffer.buffer, 0, resources.impactCountBuffer.size};
    infos[7] = {resources.impactReferenceBuffer.buffer, 0, resources.impactReferenceBuffer.size};
    infos[8] = {
        resources.dynamicMeshFlowRainSeedBuffer.buffer,
        0,
        resources.dynamicMeshFlowRainSeedBuffer.size};
    std::array<VkWriteDescriptorSet, 9> writes{};
    for (std::uint32_t binding = 0U; binding < writes.size(); ++binding) {
        writes[binding] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
        writes[binding].dstSet = resources.exrDescriptorSet;
        writes[binding].dstBinding = binding;
        writes[binding].descriptorType =
            binding == 0U ? VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER : VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        writes[binding].descriptorCount = 1U;
        writes[binding].pBufferInfo = &infos[binding];
    }
    vkUpdateDescriptorSets(
        device_,
        static_cast<std::uint32_t>(writes.size()),
        writes.data(),
        0U,
        nullptr);
}

void VulkanViewportShell::UpdateWaterSurfacePreprocessDescriptorSet(
    WaterSurfacePendingGpuUpload* pending) {
    if (pending == nullptr || pending->preprocessDescriptorSet == VK_NULL_HANDLE ||
        pending->preprocessUniformBuffer.buffer == VK_NULL_HANDLE ||
        pending->flowSurfaceInputBuffer.buffer == VK_NULL_HANDLE ||
        pending->flowSurfaceTableBuffer.buffer == VK_NULL_HANDLE) {
        return;
    }

    const std::array<VkDescriptorBufferInfo, 3> infos = {
        VkDescriptorBufferInfo{
            pending->preprocessUniformBuffer.buffer,
            0U,
            sizeof(WaterSurfacePreprocessUniformsGpu)},
        VkDescriptorBufferInfo{
            pending->flowSurfaceInputBuffer.buffer,
            0U,
            pending->flowSurfaceInputBuffer.size},
        VkDescriptorBufferInfo{
            pending->flowSurfaceTableBuffer.buffer,
            0U,
            pending->flowSurfaceTableBuffer.size},
    };
    std::array<VkWriteDescriptorSet, 3> writes{};
    for (std::uint32_t binding = 0U; binding < writes.size(); ++binding) {
        writes[binding] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
        writes[binding].dstSet = pending->preprocessDescriptorSet;
        writes[binding].dstBinding = binding;
        writes[binding].descriptorType =
            binding == 0U ? VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER : VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        writes[binding].descriptorCount = 1U;
        writes[binding].pBufferInfo = &infos[binding];
    }
    vkUpdateDescriptorSets(
        device_,
        static_cast<std::uint32_t>(writes.size()),
        writes.data(),
        0U,
        nullptr);
}

void VulkanViewportShell::CreatePointPipelines() {
    const auto vertexShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "pointcloud_preview.vert.spv").string());
    // The depth prepass reuses the preview vertex source compiled with
    // DEPTH_PREPASS, which strips colour-only work (timing colourise stack,
    // water colour chains, colormap, AOV outputs) while producing
    // bit-identical position, point size, opacity, and depth outputs.
    const auto prepassVertexShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "pointcloud_preview_depth_prepass.vert.spv").string());
    const auto depthFragmentShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "pointcloud_export_depth.frag.spv").string());
    const auto accumulationFragmentShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "pointcloud_accumulation.frag.spv").string());
    const auto sortedAlphaFragmentShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "pointcloud_sorted_alpha.frag.spv").string());
    const auto constantSimpleVertexShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "pointcloud_constant_simple.vert.spv").string());
    const auto constantSimpleFragmentShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "pointcloud_constant_simple_accumulation.frag.spv").string());
    const auto opaqueHardDiscFragmentShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "pointcloud_opaque_hard_disc.frag.spv").string());
    const auto fastBasicVertexShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "pointcloud_fast_basic.vert.spv").string());
    const auto fastBasicFragmentShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "pointcloud_fast_basic.frag.spv").string());
    const auto surfelVertexShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "pointcloud_surfel.vert.spv").string());
    const auto surfelPrepassVertexShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "pointcloud_surfel_depth_prepass.vert.spv").string());
    const auto surfelDepthFragmentShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "pointcloud_surfel_export_depth.frag.spv").string());
    const auto surfelAccumulationFragmentShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "pointcloud_surfel_accumulation.frag.spv").string());
    const auto surfelConstantSimpleVertexShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "pointcloud_surfel_constant_simple.vert.spv").string());
    const auto surfelConstantSimpleFragmentShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "pointcloud_surfel_constant_simple_accumulation.frag.spv").string());
    const auto surfelOpaqueHardDiscFragmentShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "pointcloud_surfel_opaque_hard_disc.frag.spv").string());

    const auto vertexModule = CreateShaderModule(device_, vertexShaderCode, "vkCreateShaderModule(point vertex)");
    const auto prepassVertexModule =
        CreateShaderModule(device_, prepassVertexShaderCode, "vkCreateShaderModule(point prepass vertex)");
    const auto depthFragmentModule =
        CreateShaderModule(device_, depthFragmentShaderCode, "vkCreateShaderModule(point depth fragment)");
    const auto accumulationFragmentModule =
        CreateShaderModule(device_, accumulationFragmentShaderCode, "vkCreateShaderModule(point accumulation fragment)");
    const auto sortedAlphaFragmentModule =
        CreateShaderModule(device_, sortedAlphaFragmentShaderCode, "vkCreateShaderModule(point sorted alpha fragment)");
    const auto constantSimpleVertexModule =
        CreateShaderModule(device_, constantSimpleVertexShaderCode, "vkCreateShaderModule(point simple vertex)");
    const auto constantSimpleFragmentModule =
        CreateShaderModule(device_, constantSimpleFragmentShaderCode, "vkCreateShaderModule(point simple accumulation fragment)");
    const auto opaqueHardDiscFragmentModule =
        CreateShaderModule(device_, opaqueHardDiscFragmentShaderCode, "vkCreateShaderModule(point opaque hard disc fragment)");
    const auto fastBasicVertexModule =
        CreateShaderModule(device_, fastBasicVertexShaderCode, "vkCreateShaderModule(point fast basic vertex)");
    const auto fastBasicFragmentModule =
        CreateShaderModule(device_, fastBasicFragmentShaderCode, "vkCreateShaderModule(point fast basic fragment)");
    const auto surfelVertexModule =
        CreateShaderModule(device_, surfelVertexShaderCode, "vkCreateShaderModule(surfel vertex)");
    const auto surfelPrepassVertexModule =
        CreateShaderModule(device_, surfelPrepassVertexShaderCode, "vkCreateShaderModule(surfel prepass vertex)");
    const auto surfelDepthFragmentModule =
        CreateShaderModule(device_, surfelDepthFragmentShaderCode, "vkCreateShaderModule(surfel depth fragment)");
    const auto surfelAccumulationFragmentModule =
        CreateShaderModule(device_, surfelAccumulationFragmentShaderCode, "vkCreateShaderModule(surfel accumulation fragment)");
    const auto surfelConstantSimpleVertexModule =
        CreateShaderModule(device_, surfelConstantSimpleVertexShaderCode, "vkCreateShaderModule(surfel simple vertex)");
    const auto surfelConstantSimpleFragmentModule =
        CreateShaderModule(
            device_,
            surfelConstantSimpleFragmentShaderCode,
            "vkCreateShaderModule(surfel simple accumulation fragment)");
    const auto surfelOpaqueHardDiscFragmentModule =
        CreateShaderModule(
            device_,
            surfelOpaqueHardDiscFragmentShaderCode,
            "vkCreateShaderModule(surfel opaque hard disc fragment)");

    VkPipelineShaderStageCreateInfo vertexStage{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    vertexStage.stage = VK_SHADER_STAGE_VERTEX_BIT;
    vertexStage.module = vertexModule;
    vertexStage.pName = "main";

    VkPipelineShaderStageCreateInfo prepassVertexStage{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    prepassVertexStage.stage = VK_SHADER_STAGE_VERTEX_BIT;
    prepassVertexStage.module = prepassVertexModule;
    prepassVertexStage.pName = "main";

    VkPipelineShaderStageCreateInfo surfelVertexStage{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    surfelVertexStage.stage = VK_SHADER_STAGE_VERTEX_BIT;
    surfelVertexStage.module = surfelVertexModule;
    surfelVertexStage.pName = "main";

    VkPipelineShaderStageCreateInfo surfelPrepassVertexStage{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    surfelPrepassVertexStage.stage = VK_SHADER_STAGE_VERTEX_BIT;
    surfelPrepassVertexStage.module = surfelPrepassVertexModule;
    surfelPrepassVertexStage.pName = "main";

    VkPipelineShaderStageCreateInfo constantSimpleVertexStage{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    constantSimpleVertexStage.stage = VK_SHADER_STAGE_VERTEX_BIT;
    constantSimpleVertexStage.module = constantSimpleVertexModule;
    constantSimpleVertexStage.pName = "main";

    VkPipelineShaderStageCreateInfo fastBasicVertexStage{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    fastBasicVertexStage.stage = VK_SHADER_STAGE_VERTEX_BIT;
    fastBasicVertexStage.module = fastBasicVertexModule;
    fastBasicVertexStage.pName = "main";

    VkPipelineShaderStageCreateInfo surfelConstantSimpleVertexStage{
        VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    surfelConstantSimpleVertexStage.stage = VK_SHADER_STAGE_VERTEX_BIT;
    surfelConstantSimpleVertexStage.module = surfelConstantSimpleVertexModule;
    surfelConstantSimpleVertexStage.pName = "main";

    const std::array<VkVertexInputBindingDescription, 2> bindingDescriptions = {
        VkVertexInputBindingDescription{0, sizeof(invisible_places::io::Float3), VK_VERTEX_INPUT_RATE_VERTEX},
        VkVertexInputBindingDescription{1, sizeof(std::uint32_t), VK_VERTEX_INPUT_RATE_VERTEX},
    };

    const std::array<VkVertexInputAttributeDescription, 2> attributeDescriptions = {
        VkVertexInputAttributeDescription{0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0},
        VkVertexInputAttributeDescription{1, 1, VK_FORMAT_R8G8B8A8_UNORM, 0},
    };

    VkPipelineVertexInputStateCreateInfo vertexInputInfo{
        VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO};
    vertexInputInfo.vertexBindingDescriptionCount = static_cast<std::uint32_t>(bindingDescriptions.size());
    vertexInputInfo.pVertexBindingDescriptions = bindingDescriptions.data();
    vertexInputInfo.vertexAttributeDescriptionCount = static_cast<std::uint32_t>(attributeDescriptions.size());
    vertexInputInfo.pVertexAttributeDescriptions = attributeDescriptions.data();

    VkPipelineInputAssemblyStateCreateInfo inputAssembly{
        VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO};
    inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_POINT_LIST;

    VkPipelineVertexInputStateCreateInfo surfelVertexInputInfo{
        VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO};

    VkPipelineInputAssemblyStateCreateInfo surfelInputAssembly{
        VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO};
    surfelInputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;

    VkPipelineViewportStateCreateInfo viewportState{VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO};
    viewportState.viewportCount = 1;
    viewportState.scissorCount = 1;

    VkPipelineRasterizationStateCreateInfo rasterizer{
        VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO};
    rasterizer.polygonMode = VK_POLYGON_MODE_FILL;
    rasterizer.cullMode = VK_CULL_MODE_NONE;
    rasterizer.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
    rasterizer.lineWidth = 1.0F;

    VkPipelineMultisampleStateCreateInfo multisampling{
        VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO};
    multisampling.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
    multisampling.minSampleShading = 1.0F;

    const std::array<VkDynamicState, 2> dynamicStates = {VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR};
    VkPipelineDynamicStateCreateInfo dynamicState{VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO};
    dynamicState.dynamicStateCount = static_cast<std::uint32_t>(dynamicStates.size());
    dynamicState.pDynamicStates = dynamicStates.data();

    VkPipelineLayoutCreateInfo pipelineLayoutInfo{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
    pipelineLayoutInfo.setLayoutCount = 1;
    pipelineLayoutInfo.pSetLayouts = &pointDescriptorSetLayout_;

    Check(vkCreatePipelineLayout(device_, &pipelineLayoutInfo, nullptr, &pointPipelineLayout_), "vkCreatePipelineLayout(point)");

    auto createPointPipeline =
        [&](const VkPipelineShaderStageCreateInfo& selectedVertexStage,
            const VkPipelineVertexInputStateCreateInfo& selectedVertexInputInfo,
            const VkPipelineInputAssemblyStateCreateInfo& selectedInputAssembly,
            VkShaderModule fragmentModule,
            std::uint32_t subpass,
            const std::vector<VkPipelineColorBlendAttachmentState>& blendAttachments,
            bool depthTest,
            bool depthWrite,
            VkCompareOp depthCompare,
            const char* label,
            VkPipeline* pipeline) {
            VkPipelineShaderStageCreateInfo fragmentStage{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
            fragmentStage.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
            fragmentStage.module = fragmentModule;
            fragmentStage.pName = "main";
            const std::array<VkPipelineShaderStageCreateInfo, 2> shaderStages = {
                selectedVertexStage,
                fragmentStage};

            VkPipelineDepthStencilStateCreateInfo depthStencil{
                VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO};
            depthStencil.depthTestEnable = depthTest ? VK_TRUE : VK_FALSE;
            depthStencil.depthWriteEnable = depthWrite ? VK_TRUE : VK_FALSE;
            depthStencil.depthCompareOp = depthCompare;

            VkPipelineColorBlendStateCreateInfo colorBlending{
                VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO};
            colorBlending.attachmentCount = static_cast<std::uint32_t>(blendAttachments.size());
            colorBlending.pAttachments = blendAttachments.data();

            VkGraphicsPipelineCreateInfo pipelineInfo{VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO};
            pipelineInfo.stageCount = static_cast<std::uint32_t>(shaderStages.size());
            pipelineInfo.pStages = shaderStages.data();
            pipelineInfo.pVertexInputState = &selectedVertexInputInfo;
            pipelineInfo.pInputAssemblyState = &selectedInputAssembly;
            pipelineInfo.pViewportState = &viewportState;
            pipelineInfo.pRasterizationState = &rasterizer;
            pipelineInfo.pMultisampleState = &multisampling;
            pipelineInfo.pDepthStencilState = &depthStencil;
            pipelineInfo.pColorBlendState = &colorBlending;
            pipelineInfo.pDynamicState = &dynamicState;
            pipelineInfo.layout = pointPipelineLayout_;
            pipelineInfo.renderPass = renderPass_;
            pipelineInfo.subpass = subpass;

            Check(vkCreateGraphicsPipelines(device_, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, pipeline), label);
        };

    VkPipelineColorBlendAttachmentState linearDepthBlend{};
    linearDepthBlend.colorWriteMask = VK_COLOR_COMPONENT_R_BIT;
    linearDepthBlend.blendEnable = VK_FALSE;

    VkPipelineColorBlendAttachmentState opaqueColorBlend{};
    opaqueColorBlend.colorWriteMask =
        VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
    opaqueColorBlend.blendEnable = VK_FALSE;

    createPointPipeline(
        prepassVertexStage,
        vertexInputInfo,
        inputAssembly,
        depthFragmentModule,
        0,
        std::vector<VkPipelineColorBlendAttachmentState>{linearDepthBlend},
        true,
        true,
        VK_COMPARE_OP_LESS,
        "vkCreateGraphicsPipelines(point depth prepass)",
        &pointDepthPrepassPipeline_);

    createPointPipeline(
        vertexStage,
        vertexInputInfo,
        inputAssembly,
        accumulationFragmentModule,
        1,
        std::vector<VkPipelineColorBlendAttachmentState>{
            MakeAdditiveBlendAttachment(),
            MakeRevealageBlendAttachment(),
            MakeAdditiveBlendAttachment()},
        false,
        false,
        VK_COMPARE_OP_ALWAYS,
        "vkCreateGraphicsPipelines(point accumulation)",
        &pointAccumulationPipeline_);

    // Preview performance mode: opaque-styled layers forced onto the unified
    // shader accumulate against the read-only prepass depth so occluded
    // fragments never shade. LESS_OR_EQUAL admits the surface the prepass
    // itself wrote (positions are bit-identical between the two pipelines).
    createPointPipeline(
        vertexStage,
        vertexInputInfo,
        inputAssembly,
        accumulationFragmentModule,
        1,
        std::vector<VkPipelineColorBlendAttachmentState>{
            MakeAdditiveBlendAttachment(),
            MakeRevealageBlendAttachment(),
            MakeAdditiveBlendAttachment()},
        true,
        false,
        VK_COMPARE_OP_LESS_OR_EQUAL,
        "vkCreateGraphicsPipelines(point accumulation depth tested)",
        &pointAccumulationDepthTestedPipeline_);

    createPointPipeline(
        constantSimpleVertexStage,
        vertexInputInfo,
        inputAssembly,
        constantSimpleFragmentModule,
        1,
        std::vector<VkPipelineColorBlendAttachmentState>{
            MakeAdditiveBlendAttachment(),
            MakeRevealageBlendAttachment(),
            MakeAdditiveBlendAttachment()},
        false,
        false,
        VK_COMPARE_OP_ALWAYS,
        "vkCreateGraphicsPipelines(point simple accumulation)",
        &pointConstantSimpleAccumulationPipeline_);

    createPointPipeline(
        constantSimpleVertexStage,
        vertexInputInfo,
        inputAssembly,
        opaqueHardDiscFragmentModule,
        3,
        std::vector<VkPipelineColorBlendAttachmentState>{MakeAlphaBlendAttachment()},
        true,
        false,
        VK_COMPARE_OP_LESS_OR_EQUAL,
        "vkCreateGraphicsPipelines(point opaque hard disc)",
        &pointOpaqueHardDiscPipeline_);

    createPointPipeline(
        fastBasicVertexStage,
        vertexInputInfo,
        inputAssembly,
        fastBasicFragmentModule,
        3,
        std::vector<VkPipelineColorBlendAttachmentState>{opaqueColorBlend},
        true,
        true,
        VK_COMPARE_OP_LESS,
        "vkCreateGraphicsPipelines(point fast basic)",
        &pointFastBasicPipeline_);

    // Sorted transparency must not populate hardware depth with its own
    // nearest point before the back-to-front colour pass. When EDL or a
    // depth readback needs this layer, the already-sorted draw instead
    // overwrites only the linear-depth AOV; its final fragment is the nearest
    // one while existing opaque/prepass geometry remains the hardware-depth
    // occluder.
    createPointPipeline(
        prepassVertexStage,
        vertexInputInfo,
        inputAssembly,
        depthFragmentModule,
        0,
        std::vector<VkPipelineColorBlendAttachmentState>{linearDepthBlend},
        true,
        false,
        VK_COMPARE_OP_LESS_OR_EQUAL,
        "vkCreateGraphicsPipelines(point sorted depth AOV)",
        &pointSortedDepthAovPipeline_);

    createPointPipeline(
        vertexStage,
        vertexInputInfo,
        inputAssembly,
        sortedAlphaFragmentModule,
        4,
        std::vector<VkPipelineColorBlendAttachmentState>{
            MakeAlphaBlendAttachment(),
            MakeAdditiveBlendAttachment()},
        true,
        false,
        VK_COMPARE_OP_LESS_OR_EQUAL,
        "vkCreateGraphicsPipelines(point sorted alpha)",
        &pointSortedAlphaPipeline_);
    createPointPipeline(
        vertexStage,
        vertexInputInfo,
        inputAssembly,
        sortedAlphaFragmentModule,
        4,
        std::vector<VkPipelineColorBlendAttachmentState>{
            MakeAlphaBlendAttachment(),
            MakeAdditiveBlendAttachment()},
        false,
        false,
        VK_COMPARE_OP_ALWAYS,
        "vkCreateGraphicsPipelines(point sorted alpha hybrid depth)",
        &pointSortedAlphaHybridPipeline_);

    createPointPipeline(
        surfelPrepassVertexStage,
        surfelVertexInputInfo,
        surfelInputAssembly,
        surfelDepthFragmentModule,
        0,
        std::vector<VkPipelineColorBlendAttachmentState>{linearDepthBlend},
        true,
        true,
        VK_COMPARE_OP_LESS,
        "vkCreateGraphicsPipelines(surfel depth prepass)",
        &surfelDepthPrepassPipeline_);

    createPointPipeline(
        surfelVertexStage,
        surfelVertexInputInfo,
        surfelInputAssembly,
        surfelAccumulationFragmentModule,
        1,
        std::vector<VkPipelineColorBlendAttachmentState>{
            MakeAdditiveBlendAttachment(),
            MakeRevealageBlendAttachment(),
            MakeAdditiveBlendAttachment()},
        false,
        false,
        VK_COMPARE_OP_ALWAYS,
        "vkCreateGraphicsPipelines(surfel accumulation)",
        &surfelAccumulationPipeline_);

    createPointPipeline(
        surfelVertexStage,
        surfelVertexInputInfo,
        surfelInputAssembly,
        surfelAccumulationFragmentModule,
        1,
        std::vector<VkPipelineColorBlendAttachmentState>{
            MakeAdditiveBlendAttachment(),
            MakeRevealageBlendAttachment(),
            MakeAdditiveBlendAttachment()},
        true,
        false,
        VK_COMPARE_OP_LESS_OR_EQUAL,
        "vkCreateGraphicsPipelines(surfel accumulation depth tested)",
        &surfelAccumulationDepthTestedPipeline_);

    createPointPipeline(
        surfelConstantSimpleVertexStage,
        surfelVertexInputInfo,
        surfelInputAssembly,
        surfelConstantSimpleFragmentModule,
        1,
        std::vector<VkPipelineColorBlendAttachmentState>{
            MakeAdditiveBlendAttachment(),
            MakeRevealageBlendAttachment(),
            MakeAdditiveBlendAttachment()},
        false,
        false,
        VK_COMPARE_OP_ALWAYS,
        "vkCreateGraphicsPipelines(surfel simple accumulation)",
        &surfelConstantSimpleAccumulationPipeline_);

    createPointPipeline(
        surfelConstantSimpleVertexStage,
        surfelVertexInputInfo,
        surfelInputAssembly,
        surfelOpaqueHardDiscFragmentModule,
        3,
        std::vector<VkPipelineColorBlendAttachmentState>{MakeAlphaBlendAttachment()},
        true,
        false,
        VK_COMPARE_OP_LESS_OR_EQUAL,
        "vkCreateGraphicsPipelines(surfel opaque hard disc)",
        &surfelOpaqueHardDiscPipeline_);

    vkDestroyShaderModule(device_, surfelOpaqueHardDiscFragmentModule, nullptr);
    vkDestroyShaderModule(device_, surfelConstantSimpleFragmentModule, nullptr);
    vkDestroyShaderModule(device_, surfelConstantSimpleVertexModule, nullptr);
    vkDestroyShaderModule(device_, surfelAccumulationFragmentModule, nullptr);
    vkDestroyShaderModule(device_, surfelDepthFragmentModule, nullptr);
    vkDestroyShaderModule(device_, surfelPrepassVertexModule, nullptr);
    vkDestroyShaderModule(device_, surfelVertexModule, nullptr);
    vkDestroyShaderModule(device_, constantSimpleFragmentModule, nullptr);
    vkDestroyShaderModule(device_, fastBasicFragmentModule, nullptr);
    vkDestroyShaderModule(device_, fastBasicVertexModule, nullptr);
    vkDestroyShaderModule(device_, opaqueHardDiscFragmentModule, nullptr);
    vkDestroyShaderModule(device_, constantSimpleVertexModule, nullptr);
    vkDestroyShaderModule(device_, sortedAlphaFragmentModule, nullptr);
    vkDestroyShaderModule(device_, accumulationFragmentModule, nullptr);
    vkDestroyShaderModule(device_, depthFragmentModule, nullptr);
    vkDestroyShaderModule(device_, prepassVertexModule, nullptr);
    vkDestroyShaderModule(device_, vertexModule, nullptr);
}

void VulkanViewportShell::CreatePointDepthSortPipeline() {
    const auto shaderCode = ReadBinaryFile(
        (std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} /
         "pointcloud_depth_sort.comp.spv")
            .string());
    const auto shaderModule = CreateShaderModule(
        device_,
        shaderCode,
        "vkCreateShaderModule(point depth sort)");

    VkPushConstantRange pushConstantRange{};
    pushConstantRange.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    pushConstantRange.offset = 0U;
    pushConstantRange.size = sizeof(PointDepthSortPushConstants);
    VkPipelineLayoutCreateInfo layoutInfo{
        VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
    layoutInfo.setLayoutCount = 1U;
    layoutInfo.pSetLayouts = &pointDepthSortDescriptorSetLayout_;
    layoutInfo.pushConstantRangeCount = 1U;
    layoutInfo.pPushConstantRanges = &pushConstantRange;
    Check(
        vkCreatePipelineLayout(
            device_,
            &layoutInfo,
            nullptr,
            &pointDepthSortPipelineLayout_),
        "vkCreatePipelineLayout(point depth sort)");

    VkPipelineShaderStageCreateInfo stageInfo{
        VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    stageInfo.stage = VK_SHADER_STAGE_COMPUTE_BIT;
    stageInfo.module = shaderModule;
    stageInfo.pName = "main";
    VkComputePipelineCreateInfo pipelineInfo{
        VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO};
    pipelineInfo.stage = stageInfo;
    pipelineInfo.layout = pointDepthSortPipelineLayout_;
    Check(
        vkCreateComputePipelines(
            device_,
            VK_NULL_HANDLE,
            1U,
            &pipelineInfo,
            nullptr,
            &pointDepthSortPipeline_),
        "vkCreateComputePipelines(point depth sort)");
    vkDestroyShaderModule(device_, shaderModule, nullptr);
}

void VulkanViewportShell::CreateDynamicMeshFlowComputePipeline() {
    const auto shaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "dynamic_mesh_flow.comp.spv").string());
    const auto shaderModule =
        CreateShaderModule(device_, shaderCode, "vkCreateShaderModule(dynamic mesh flow compute)");

    VkPipelineShaderStageCreateInfo stageInfo{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    stageInfo.stage = VK_SHADER_STAGE_COMPUTE_BIT;
    stageInfo.module = shaderModule;
    stageInfo.pName = "main";

    VkPipelineLayoutCreateInfo layoutInfo{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
    layoutInfo.setLayoutCount = 1;
    layoutInfo.pSetLayouts = &dynamicMeshFlowDescriptorSetLayout_;
    Check(
        vkCreatePipelineLayout(device_, &layoutInfo, nullptr, &dynamicMeshFlowPipelineLayout_),
        "vkCreatePipelineLayout(dynamic mesh flow)");

    VkComputePipelineCreateInfo pipelineInfo{VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO};
    pipelineInfo.stage = stageInfo;
    pipelineInfo.layout = dynamicMeshFlowPipelineLayout_;
    Check(
        vkCreateComputePipelines(
            device_,
            VK_NULL_HANDLE,
            1,
            &pipelineInfo,
            nullptr,
            &dynamicMeshFlowComputePipeline_),
        "vkCreateComputePipelines(dynamic mesh flow)");

    vkDestroyShaderModule(device_, shaderModule, nullptr);
}

void VulkanViewportShell::CreateWaterFlowSourceComputePipelines() {
    const auto routeCode = ReadBinaryFile(
        (std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} /
         "water_flow_routes.comp.spv")
            .string());
    const auto trailCode = ReadBinaryFile(
        (std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} /
         "water_flow_trails.comp.spv")
            .string());
    const auto routeModule =
        CreateShaderModule(device_, routeCode, "vkCreateShaderModule(water flow routes)");
    const auto trailModule =
        CreateShaderModule(device_, trailCode, "vkCreateShaderModule(water flow trails)");

    VkPipelineLayoutCreateInfo layoutInfo{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
    layoutInfo.setLayoutCount = 1U;
    layoutInfo.pSetLayouts = &waterFlowSourceDescriptorSetLayout_;
    Check(
        vkCreatePipelineLayout(device_, &layoutInfo, nullptr, &waterFlowSourcePipelineLayout_),
        "vkCreatePipelineLayout(water flow source)");

    const auto createPipeline = [this](
                                    VkShaderModule module,
                                    VkPipeline* pipeline,
                                    const char* label) {
        VkPipelineShaderStageCreateInfo stageInfo{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
        stageInfo.stage = VK_SHADER_STAGE_COMPUTE_BIT;
        stageInfo.module = module;
        stageInfo.pName = "main";
        VkComputePipelineCreateInfo pipelineInfo{VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO};
        pipelineInfo.stage = stageInfo;
        pipelineInfo.layout = waterFlowSourcePipelineLayout_;
        Check(
            vkCreateComputePipelines(device_, VK_NULL_HANDLE, 1U, &pipelineInfo, nullptr, pipeline),
            label);
    };
    createPipeline(
        routeModule,
        &waterFlowRouteComputePipeline_,
        "vkCreateComputePipelines(water flow routes)");
    createPipeline(
        trailModule,
        &waterFlowTrailComputePipeline_,
        "vkCreateComputePipelines(water flow trails)");

    vkDestroyShaderModule(device_, trailModule, nullptr);
    vkDestroyShaderModule(device_, routeModule, nullptr);
}

void VulkanViewportShell::CreateRainPipelines() {
    const auto computeCode = ReadBinaryFile(
        (std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "rain_simulation.comp.spv").string());
    const auto vertexCode = ReadBinaryFile(
        (std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "rain_particle.vert.spv").string());
    const auto fragmentCode = ReadBinaryFile(
        (std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "rain_particle.frag.spv").string());
    const auto computeModule = CreateShaderModule(device_, computeCode, "vkCreateShaderModule(rain compute)");
    const auto vertexModule = CreateShaderModule(device_, vertexCode, "vkCreateShaderModule(rain vertex)");
    const auto fragmentModule = CreateShaderModule(device_, fragmentCode, "vkCreateShaderModule(rain fragment)");

    VkPushConstantRange pushConstantRange{};
    pushConstantRange.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    pushConstantRange.offset = 0U;
    pushConstantRange.size = sizeof(std::uint32_t);
    VkPipelineLayoutCreateInfo layoutInfo{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
    layoutInfo.setLayoutCount = 1U;
    layoutInfo.pSetLayouts = &rainDescriptorSetLayout_;
    layoutInfo.pushConstantRangeCount = 1U;
    layoutInfo.pPushConstantRanges = &pushConstantRange;
    Check(
        vkCreatePipelineLayout(device_, &layoutInfo, nullptr, &rainPipelineLayout_),
        "vkCreatePipelineLayout(rain)");

    VkPipelineShaderStageCreateInfo computeStage{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    computeStage.stage = VK_SHADER_STAGE_COMPUTE_BIT;
    computeStage.module = computeModule;
    computeStage.pName = "main";
    VkComputePipelineCreateInfo computeInfo{VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO};
    computeInfo.stage = computeStage;
    computeInfo.layout = rainPipelineLayout_;
    Check(
        vkCreateComputePipelines(device_, VK_NULL_HANDLE, 1U, &computeInfo, nullptr, &rainComputePipeline_),
        "vkCreateComputePipelines(rain)");

    std::array<VkPipelineShaderStageCreateInfo, 2> stages{};
    stages[0] = {VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    stages[0].stage = VK_SHADER_STAGE_VERTEX_BIT;
    stages[0].module = vertexModule;
    stages[0].pName = "main";
    stages[1] = {VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    stages[1].stage = VK_SHADER_STAGE_FRAGMENT_BIT;
    stages[1].module = fragmentModule;
    stages[1].pName = "main";
    VkPipelineVertexInputStateCreateInfo vertexInput{VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO};
    VkPipelineInputAssemblyStateCreateInfo inputAssembly{VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO};
    inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
    VkPipelineViewportStateCreateInfo viewportState{VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO};
    viewportState.viewportCount = 1U;
    viewportState.scissorCount = 1U;
    VkPipelineRasterizationStateCreateInfo rasterization{VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO};
    rasterization.polygonMode = VK_POLYGON_MODE_FILL;
    rasterization.cullMode = VK_CULL_MODE_NONE;
    rasterization.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
    rasterization.lineWidth = 1.0F;
    VkPipelineMultisampleStateCreateInfo multisample{VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO};
    multisample.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
    VkPipelineDepthStencilStateCreateInfo depthStencil{VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO};
    depthStencil.depthTestEnable = VK_TRUE;
    depthStencil.depthWriteEnable = VK_FALSE;
    depthStencil.depthCompareOp = VK_COMPARE_OP_LESS_OR_EQUAL;
    const std::array blendAttachments = {
        MakeAdditiveBlendAttachment(),
        MakeRevealageBlendAttachment(),
        MakeAdditiveBlendAttachment(),
    };
    VkPipelineColorBlendStateCreateInfo colourBlend{VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO};
    colourBlend.attachmentCount = static_cast<std::uint32_t>(blendAttachments.size());
    colourBlend.pAttachments = blendAttachments.data();
    constexpr std::array dynamicStates = {VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR};
    VkPipelineDynamicStateCreateInfo dynamicState{VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO};
    dynamicState.dynamicStateCount = static_cast<std::uint32_t>(dynamicStates.size());
    dynamicState.pDynamicStates = dynamicStates.data();
    VkGraphicsPipelineCreateInfo pipelineInfo{VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO};
    pipelineInfo.stageCount = static_cast<std::uint32_t>(stages.size());
    pipelineInfo.pStages = stages.data();
    pipelineInfo.pVertexInputState = &vertexInput;
    pipelineInfo.pInputAssemblyState = &inputAssembly;
    pipelineInfo.pViewportState = &viewportState;
    pipelineInfo.pRasterizationState = &rasterization;
    pipelineInfo.pMultisampleState = &multisample;
    pipelineInfo.pDepthStencilState = &depthStencil;
    pipelineInfo.pColorBlendState = &colourBlend;
    pipelineInfo.pDynamicState = &dynamicState;
    pipelineInfo.layout = rainPipelineLayout_;
    pipelineInfo.renderPass = renderPass_;
    pipelineInfo.subpass = 1U;
    Check(
        vkCreateGraphicsPipelines(device_, VK_NULL_HANDLE, 1U, &pipelineInfo, nullptr, &rainPipeline_),
        "vkCreateGraphicsPipelines(rain)");

    vkDestroyShaderModule(device_, fragmentModule, nullptr);
    vkDestroyShaderModule(device_, vertexModule, nullptr);
    vkDestroyShaderModule(device_, computeModule, nullptr);
}

void VulkanViewportShell::CreateWaterSurfacePreprocessPipeline() {
    const auto shaderCode = ReadBinaryFile(
        (std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} /
         "water_surface_preprocess.comp.spv")
            .string());
    const auto shaderModule = CreateShaderModule(
        device_,
        shaderCode,
        "vkCreateShaderModule(water surface preprocess)");

    VkPipelineLayoutCreateInfo layoutInfo{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
    layoutInfo.setLayoutCount = 1U;
    layoutInfo.pSetLayouts = &waterSurfacePreprocessDescriptorSetLayout_;
    Check(
        vkCreatePipelineLayout(
            device_,
            &layoutInfo,
            nullptr,
            &waterSurfacePreprocessPipelineLayout_),
        "vkCreatePipelineLayout(water surface preprocess)");

    VkPipelineShaderStageCreateInfo stageInfo{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    stageInfo.stage = VK_SHADER_STAGE_COMPUTE_BIT;
    stageInfo.module = shaderModule;
    stageInfo.pName = "main";
    VkComputePipelineCreateInfo pipelineInfo{VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO};
    pipelineInfo.stage = stageInfo;
    pipelineInfo.layout = waterSurfacePreprocessPipelineLayout_;
    Check(
        vkCreateComputePipelines(device_, VK_NULL_HANDLE, 1U, &pipelineInfo, nullptr, &waterSurfacePreprocessPipeline_),
        "vkCreateComputePipelines(water surface preprocess)");

    vkDestroyShaderModule(device_, shaderModule, nullptr);
}

void VulkanViewportShell::CreateExrExportResources(
    std::uint32_t width,
    std::uint32_t height,
    bool enableEyeDomeLighting,
    PointCloudExrReadbackMask readbackMask) {
    if (width == 0 || height == 0) {
        throw std::runtime_error{"GPU EXR export requires a non-zero frame size."};
    }

    constexpr VkFormat kExportColorFormat = VK_FORMAT_R16G16B16A16_SFLOAT;
    constexpr VkFormat kExportLinearDepthFormat = VK_FORMAT_R32_SFLOAT;
    VkFormatFeatureFlags requiredColorFeatures =
        VK_FORMAT_FEATURE_COLOR_ATTACHMENT_BIT | VK_FORMAT_FEATURE_TRANSFER_SRC_BIT;
    VkFormatFeatureFlags requiredDepthFeatures =
        VK_FORMAT_FEATURE_COLOR_ATTACHMENT_BIT | VK_FORMAT_FEATURE_TRANSFER_SRC_BIT;
    if (enableEyeDomeLighting) {
        requiredColorFeatures |= VK_FORMAT_FEATURE_STORAGE_IMAGE_BIT;
        requiredDepthFeatures |= VK_FORMAT_FEATURE_SAMPLED_IMAGE_BIT;
    }
    if (!FormatSupportsOptimalFeatures(
            physicalDevice_,
            kExportColorFormat,
            requiredColorFeatures)) {
        throw std::runtime_error{"GPU EXR export requires RGBA16F color attachment readback support."};
    }
    if (!FormatSupportsOptimalFeatures(
            physicalDevice_,
            kExportLinearDepthFormat,
            requiredDepthFeatures)) {
        throw std::runtime_error{"GPU EXR export requires R32F depth attachment readback support."};
    }

    // Build the entire replacement while the current export and every live
    // point descriptor generation remain usable. No settled state is mutated
    // until all fallible Vulkan allocations and writes have succeeded.
    ExrExportResources stagedResources;
    auto& resources = stagedResources;
    resources.width = width;
    resources.height = height;
    resources.readbackMask = readbackMask;

    std::vector<PendingPointDescriptorGeneration> stagedPointGenerations;
    try {
        CreateExrExportRenderPass(&resources);
        CreateExrExportPipelines(&resources);

        const VkImageUsageFlags colorPostProcessUsage =
            enableEyeDomeLighting ? VK_IMAGE_USAGE_STORAGE_BIT : 0U;
        const VkImageUsageFlags depthPostProcessUsage =
            enableEyeDomeLighting ? VK_IMAGE_USAGE_SAMPLED_BIT : 0U;
        resources.colorImage = CreateAttachmentImage(
            width,
            height,
            kExportColorFormat,
            VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT |
                colorPostProcessUsage,
            VK_IMAGE_ASPECT_COLOR_BIT);
        resources.depthImage =
            CreateAttachmentImage(width, height, depthFormat_,
                                  VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_INPUT_ATTACHMENT_BIT,
                                  VK_IMAGE_ASPECT_DEPTH_BIT);
        resources.accumulationImage = CreateAttachmentImage(
            width, height, accumulationFormat_,
            VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_INPUT_ATTACHMENT_BIT, VK_IMAGE_ASPECT_COLOR_BIT);
        resources.revealageImage = CreateAttachmentImage(
            width, height, revealageFormat_, VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_INPUT_ATTACHMENT_BIT,
            VK_IMAGE_ASPECT_COLOR_BIT);
        resources.emissiveImage = CreateAttachmentImage(
            width, height, accumulationFormat_,
            VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_INPUT_ATTACHMENT_BIT, VK_IMAGE_ASPECT_COLOR_BIT);
        resources.normalAccumulationImage = CreateAttachmentImage(
            width, height, accumulationFormat_,
            VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_INPUT_ATTACHMENT_BIT, VK_IMAGE_ASPECT_COLOR_BIT);
        resources.albedoAccumulationImage = CreateAttachmentImage(
            width, height, accumulationFormat_,
            VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_INPUT_ATTACHMENT_BIT, VK_IMAGE_ASPECT_COLOR_BIT);
        resources.linearDepthImage = CreateAttachmentImage(
            width, height, kExportLinearDepthFormat,
            VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT |
                depthPostProcessUsage,
            VK_IMAGE_ASPECT_COLOR_BIT);
        resources.normalImage = CreateAttachmentImage(
            width, height, kExportColorFormat, VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT,
            VK_IMAGE_ASPECT_COLOR_BIT);
        resources.albedoImage = CreateAttachmentImage(
            width, height, kExportColorFormat, VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT,
            VK_IMAGE_ASPECT_COLOR_BIT);

        const std::array<VkImageView, 10> attachments = {
            resources.colorImage.view,
            resources.depthImage.view,
            resources.accumulationImage.view,
            resources.revealageImage.view,
            resources.emissiveImage.view,
            resources.linearDepthImage.view,
            resources.normalAccumulationImage.view,
            resources.albedoAccumulationImage.view,
            resources.normalImage.view,
            resources.albedoImage.view,
        };

        VkFramebufferCreateInfo framebufferInfo{VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO};
        framebufferInfo.renderPass = resources.renderPass;
        framebufferInfo.attachmentCount = static_cast<std::uint32_t>(attachments.size());
        framebufferInfo.pAttachments = attachments.data();
        framebufferInfo.width = width;
        framebufferInfo.height = height;
        framebufferInfo.layers = 1;
        Check(vkCreateFramebuffer(device_, &framebufferInfo, nullptr, &resources.framebuffer),
              "vkCreateFramebuffer(exr)");

        if (enableEyeDomeLighting) {
            CreateExrExportEyeDomeLightingResources(&resources);
        }

        if (HasPointCloudExrReadback(
                readbackMask,
                PointCloudExrReadbackMask::Color)) {
            resources.colorReadbackBuffer = CreateHostVisibleBuffer(
                static_cast<VkDeviceSize>(width) *
                    static_cast<VkDeviceSize>(height) * 4U *
                    sizeof(std::uint16_t),
                VK_BUFFER_USAGE_TRANSFER_DST_BIT);
        }
        if (HasPointCloudExrReadback(
                readbackMask,
                PointCloudExrReadbackMask::Depth)) {
            resources.depthReadbackBuffer = CreateHostVisibleBuffer(
                static_cast<VkDeviceSize>(width) *
                    static_cast<VkDeviceSize>(height) * sizeof(float),
                VK_BUFFER_USAGE_TRANSFER_DST_BIT);
        }
        if (HasPointCloudExrReadback(
                readbackMask,
                PointCloudExrReadbackMask::Normal)) {
            resources.normalReadbackBuffer = CreateHostVisibleBuffer(
                static_cast<VkDeviceSize>(width) *
                    static_cast<VkDeviceSize>(height) * 4U *
                    sizeof(std::uint16_t),
                VK_BUFFER_USAGE_TRANSFER_DST_BIT);
        }
        if (HasPointCloudExrReadback(
                readbackMask,
                PointCloudExrReadbackMask::Albedo)) {
            resources.albedoReadbackBuffer = CreateHostVisibleBuffer(
                static_cast<VkDeviceSize>(width) *
                    static_cast<VkDeviceSize>(height) * 4U *
                    sizeof(std::uint16_t),
                VK_BUFFER_USAGE_TRANSFER_DST_BIT);
        }
        resources.uniformBuffer = CreateHostVisibleBuffer(sizeof(FrameUniforms), VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT);

        VkCommandBufferAllocateInfo commandBufferInfo{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
        commandBufferInfo.commandPool = commandPool_;
        commandBufferInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        commandBufferInfo.commandBufferCount = 1;
        Check(vkAllocateCommandBuffers(device_, &commandBufferInfo, &resources.commandBuffer),
              "vkAllocateCommandBuffers(exr)");

        VkFenceCreateInfo fenceInfo{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};
        fenceInfo.flags = VK_FENCE_CREATE_SIGNALED_BIT;
        Check(vkCreateFence(device_, &fenceInfo, nullptr, &resources.fence), "vkCreateFence(exr)");

        const VkDescriptorPoolSize compositePoolSize = MakePoolSize(VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT, 5U);
        VkDescriptorPoolCreateInfo compositePoolInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
        compositePoolInfo.maxSets = 1U;
        compositePoolInfo.poolSizeCount = 1U;
        compositePoolInfo.pPoolSizes = &compositePoolSize;
        Check(vkCreateDescriptorPool(device_, &compositePoolInfo, nullptr, &resources.compositeDescriptorPool),
              "vkCreateDescriptorPool(exr composite generation)");
        VkDescriptorSetAllocateInfo compositeAllocInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
        compositeAllocInfo.descriptorPool = resources.compositeDescriptorPool;
        compositeAllocInfo.descriptorSetCount = 1U;
        compositeAllocInfo.pSetLayouts = &compositeDescriptorSetLayout_;
        Check(vkAllocateDescriptorSets(device_, &compositeAllocInfo, &resources.compositeDescriptorSet),
              "vkAllocateDescriptorSets(exr composite generation)");
        CreateOrUpdateCompositeDescriptorSet(&resources.compositeDescriptorSet, resources.accumulationImage.view,
                                             resources.revealageImage.view, resources.emissiveImage.view,
                                             resources.normalAccumulationImage.view,
                                             resources.albedoAccumulationImage.view);

        stagedPointGenerations.reserve(pointCloudResources_.size());
        for (auto& pointResources : pointCloudResources_) {
            stagedPointGenerations.push_back(
                BuildPointDescriptorGeneration(
                    &pointResources,
                    resources.depthImage.view,
                    resources.uniformBuffer.buffer));
        }
        for (std::size_t index = 0U; index < pointCloudResources_.size(); ++index) {
            if (stagedPointGenerations[index].descriptorPool == VK_NULL_HANDLE ||
                stagedPointGenerations[index].highlightDescriptorSets.size() !=
                    pointCloudResources_[index].highlights.size() ||
                stagedPointGenerations[index].highlightSeepageDescriptorGenerations.size() !=
                    pointCloudResources_[index].highlights.size()) {
                throw std::runtime_error{"Staged EXR point descriptor generation is incomplete."};
            }
        }
    } catch (...) {
        for (auto& generation : stagedPointGenerations) {
            DestroyPointDescriptorGeneration(&generation);
        }
        DestroyExrExportResources(&stagedResources);
        throw;
    }

    try {
        // This is the only device-wide settle in EXR recreation. It also
        // resets MoltenVK command buffers that retain resolved descriptors.
        SettlePointCloudMutation();
    } catch (...) {
        for (auto& generation : stagedPointGenerations) {
            DestroyPointDescriptorGeneration(&generation);
        }
        DestroyExrExportResources(&stagedResources);
        throw;
    }

    exrExportFrameInFlight_ = false;
    exrExportInFlightWidth_ = 0U;
    exrExportInFlightHeight_ = 0U;
    exrExportInFlightReadbackMask_ = PointCloudExrReadbackMask::All;

    std::swap(exrExportResources_, stagedResources);
    for (std::size_t index = 0U; index < pointCloudResources_.size(); ++index) {
        InstallPointDescriptorGeneration(&pointCloudResources_[index], &stagedPointGenerations[index]);
    }
    // Old point pools are gone before their old EXR depth attachment. The
    // replaced EXR generation can now be destroyed without a stale binding.
    DestroyExrExportResources(&stagedResources);
}

void VulkanViewportShell::CreateExrExportEyeDomeLightingResources(
    ExrExportResources* resources) {
    if (resources == nullptr ||
        resources->colorImage.view == VK_NULL_HANDLE ||
        resources->linearDepthImage.view == VK_NULL_HANDLE) {
        throw std::runtime_error{
            "GPU export EDL requires initialized color and linear-depth images."};
    }

    const std::array<VkDescriptorPoolSize, 2> poolSizes = {
        MakePoolSize(VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1U),
        MakePoolSize(VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1U),
    };
    VkDescriptorPoolCreateInfo poolInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
    poolInfo.maxSets = 1U;
    poolInfo.poolSizeCount = static_cast<std::uint32_t>(poolSizes.size());
    poolInfo.pPoolSizes = poolSizes.data();
    Check(
        vkCreateDescriptorPool(
            device_,
            &poolInfo,
            nullptr,
            &resources->eyeDomeLightingDescriptorPool),
        "vkCreateDescriptorPool(export EDL generation)");

    VkDescriptorSetAllocateInfo allocInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
    allocInfo.descriptorPool = resources->eyeDomeLightingDescriptorPool;
    allocInfo.descriptorSetCount = 1U;
    allocInfo.pSetLayouts = &eyeDomeLightingExportDescriptorSetLayout_;
    Check(
        vkAllocateDescriptorSets(
            device_,
            &allocInfo,
            &resources->eyeDomeLightingDescriptorSet),
        "vkAllocateDescriptorSets(export EDL generation)");

    VkDescriptorImageInfo colorInfo{};
    colorInfo.imageView = resources->colorImage.view;
    colorInfo.imageLayout = VK_IMAGE_LAYOUT_GENERAL;

    VkDescriptorImageInfo depthInfo{};
    depthInfo.sampler = postProcessSampler_;
    depthInfo.imageView = resources->linearDepthImage.view;
    depthInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

    std::array<VkWriteDescriptorSet, 2> writes{};
    writes[0] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[0].dstSet = resources->eyeDomeLightingDescriptorSet;
    writes[0].dstBinding = 0U;
    writes[0].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
    writes[0].descriptorCount = 1U;
    writes[0].pImageInfo = &colorInfo;

    writes[1] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[1].dstSet = resources->eyeDomeLightingDescriptorSet;
    writes[1].dstBinding = 1U;
    writes[1].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    writes[1].descriptorCount = 1U;
    writes[1].pImageInfo = &depthInfo;

    vkUpdateDescriptorSets(
        device_,
        static_cast<std::uint32_t>(writes.size()),
        writes.data(),
        0U,
        nullptr);
}

void VulkanViewportShell::CreateExrExportRenderPass(ExrExportResources* resources) {
    if (resources == nullptr) {
        return;
    }

    constexpr VkFormat kExportColorFormat = VK_FORMAT_R16G16B16A16_SFLOAT;
    constexpr VkFormat kExportLinearDepthFormat = VK_FORMAT_R32_SFLOAT;

    VkAttachmentDescription colorAttachment{};
    colorAttachment.format = kExportColorFormat;
    colorAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
    colorAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    colorAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    colorAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    colorAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    colorAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    colorAttachment.finalLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;

    VkAttachmentDescription depthAttachment{};
    depthAttachment.format = depthFormat_;
    depthAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
    depthAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    depthAttachment.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    depthAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    depthAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    depthAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    depthAttachment.finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

    VkAttachmentDescription accumulationAttachment{};
    accumulationAttachment.format = accumulationFormat_;
    accumulationAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
    accumulationAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    accumulationAttachment.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    accumulationAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    accumulationAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    accumulationAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    accumulationAttachment.finalLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

    VkAttachmentDescription revealageAttachment{};
    revealageAttachment.format = revealageFormat_;
    revealageAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
    revealageAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    revealageAttachment.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    revealageAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    revealageAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    revealageAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    revealageAttachment.finalLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

    VkAttachmentDescription emissiveAttachment{};
    emissiveAttachment.format = accumulationFormat_;
    emissiveAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
    emissiveAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    emissiveAttachment.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    emissiveAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    emissiveAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    emissiveAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    emissiveAttachment.finalLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

    VkAttachmentDescription aovAccumulationAttachment = emissiveAttachment;

    VkAttachmentDescription linearDepthAttachment{};
    linearDepthAttachment.format = kExportLinearDepthFormat;
    linearDepthAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
    linearDepthAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    linearDepthAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    linearDepthAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    linearDepthAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    linearDepthAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    linearDepthAttachment.finalLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;

    VkAttachmentDescription aovOutputAttachment = colorAttachment;

    VkAttachmentReference linearDepthColorRef{5, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
    VkAttachmentReference finalColorRef{0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
    VkAttachmentReference normalOutputRef{8, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
    VkAttachmentReference albedoOutputRef{9, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
    VkAttachmentReference depthAttachmentRef{1, VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL};
    VkAttachmentReference depthReadOnlyAttachmentRef{1, VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL};

    VkSubpassDescription depthSubpass{};
    depthSubpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    depthSubpass.colorAttachmentCount = 1;
    depthSubpass.pColorAttachments = &linearDepthColorRef;
    depthSubpass.pDepthStencilAttachment = &depthAttachmentRef;

    VkAttachmentReference accumulationColorRefs[5]{};
    accumulationColorRefs[0] = {2, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
    accumulationColorRefs[1] = {3, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
    accumulationColorRefs[2] = {4, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
    accumulationColorRefs[3] = {6, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
    accumulationColorRefs[4] = {7, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
    VkAttachmentReference depthInputRef{1, VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL};

    VkSubpassDescription accumulationSubpass{};
    accumulationSubpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    accumulationSubpass.colorAttachmentCount = 5;
    accumulationSubpass.pColorAttachments = accumulationColorRefs;
    accumulationSubpass.inputAttachmentCount = 1;
    accumulationSubpass.pInputAttachments = &depthInputRef;
    accumulationSubpass.pDepthStencilAttachment = &depthReadOnlyAttachmentRef;

    VkAttachmentReference compositeInputRefs[5]{};
    compositeInputRefs[0] = {2, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL};
    compositeInputRefs[1] = {3, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL};
    compositeInputRefs[2] = {4, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL};
    compositeInputRefs[3] = {6, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL};
    compositeInputRefs[4] = {7, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL};

    VkAttachmentReference compositeColorRefs[3]{};
    compositeColorRefs[0] = finalColorRef;
    compositeColorRefs[1] = normalOutputRef;
    compositeColorRefs[2] = albedoOutputRef;

    VkSubpassDescription compositeSubpass{};
    compositeSubpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    compositeSubpass.colorAttachmentCount = 3;
    compositeSubpass.pColorAttachments = compositeColorRefs;
    compositeSubpass.inputAttachmentCount = 5;
    compositeSubpass.pInputAttachments = compositeInputRefs;

    VkAttachmentReference fastBasicColorRefs[3]{};
    fastBasicColorRefs[0] = finalColorRef;
    fastBasicColorRefs[1] = normalOutputRef;
    fastBasicColorRefs[2] = albedoOutputRef;

    VkSubpassDescription fastBasicSubpass{};
    fastBasicSubpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    fastBasicSubpass.colorAttachmentCount = 3;
    fastBasicSubpass.pColorAttachments = fastBasicColorRefs;
    fastBasicSubpass.pDepthStencilAttachment = &depthReadOnlyAttachmentRef;

    // Sorted layers add emission into the shared emission attachment
    // (cleared at subpass start) so the sorted emission composite subpass
    // can reproduce the weighted path's exponential response exactly.
    VkAttachmentReference sortedColorRefs[4]{};
    sortedColorRefs[0] = finalColorRef;
    sortedColorRefs[1] = normalOutputRef;
    sortedColorRefs[2] = albedoOutputRef;
    sortedColorRefs[3] = {4, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
    VkAttachmentReference sortedDepthInputRef{
        1,
        VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL};
    VkSubpassDescription sortedSubpass{};
    sortedSubpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    sortedSubpass.colorAttachmentCount = 4;
    sortedSubpass.pColorAttachments = sortedColorRefs;
    sortedSubpass.inputAttachmentCount = 1;
    sortedSubpass.pInputAttachments = &sortedDepthInputRef;
    sortedSubpass.pDepthStencilAttachment = &depthReadOnlyAttachmentRef;

    VkAttachmentReference sortedEmissionColorRef{
        0,
        VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
    VkAttachmentReference sortedEmissionInputRef{
        4,
        VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL};
    VkSubpassDescription sortedEmissionSubpass{};
    sortedEmissionSubpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    sortedEmissionSubpass.colorAttachmentCount = 1;
    sortedEmissionSubpass.pColorAttachments = &sortedEmissionColorRef;
    sortedEmissionSubpass.inputAttachmentCount = 1;
    sortedEmissionSubpass.pInputAttachments = &sortedEmissionInputRef;

    std::array<VkSubpassDependency, 13> dependencies{};
    dependencies[0].srcSubpass = VK_SUBPASS_EXTERNAL;
    dependencies[0].dstSubpass = 0;
    dependencies[0].srcStageMask =
        VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
    dependencies[0].dstStageMask =
        VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
    dependencies[0].dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;

    dependencies[1].srcSubpass = 0;
    dependencies[1].dstSubpass = 1;
    dependencies[1].srcStageMask =
        VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT | VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;
    dependencies[1].dstStageMask = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT |
                                   VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;
    dependencies[1].srcAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
    dependencies[1].dstAccessMask = VK_ACCESS_INPUT_ATTACHMENT_READ_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT;

    dependencies[2].srcSubpass = 1;
    dependencies[2].dstSubpass = 2;
    dependencies[2].srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dependencies[2].dstStageMask =
        VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT | VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dependencies[2].srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
    dependencies[2].dstAccessMask = VK_ACCESS_INPUT_ATTACHMENT_READ_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;

    dependencies[3].srcSubpass = 0;
    dependencies[3].dstSubpass = VK_SUBPASS_EXTERNAL;
    dependencies[3].srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dependencies[3].dstStageMask = VK_PIPELINE_STAGE_TRANSFER_BIT;
    dependencies[3].srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
    dependencies[3].dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;

    dependencies[4].srcSubpass = 2;
    dependencies[4].dstSubpass = 3;
    dependencies[4].srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dependencies[4].dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dependencies[4].srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
    dependencies[4].dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;

    dependencies[5].srcSubpass = 0;
    dependencies[5].dstSubpass = 3;
    dependencies[5].srcStageMask =
        VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT | VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;
    dependencies[5].dstStageMask =
        VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT | VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;
    dependencies[5].srcAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
    dependencies[5].dstAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT;

    dependencies[6].srcSubpass = 3;
    dependencies[6].dstSubpass = VK_SUBPASS_EXTERNAL;
    dependencies[6].srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dependencies[6].dstStageMask = VK_PIPELINE_STAGE_TRANSFER_BIT;
    dependencies[6].srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
    dependencies[6].dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;

    dependencies[7].srcSubpass = 3;
    dependencies[7].dstSubpass = 4;
    dependencies[7].srcStageMask =
        VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT |
        VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT |
        VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;
    dependencies[7].dstStageMask =
        VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT |
        VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT |
        VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
    dependencies[7].srcAccessMask =
        VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT |
        VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT;
    dependencies[7].dstAccessMask =
        VK_ACCESS_INPUT_ATTACHMENT_READ_BIT |
        VK_ACCESS_COLOR_ATTACHMENT_READ_BIT |
        VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT |
        VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT;

    dependencies[8].srcSubpass = 0;
    dependencies[8].dstSubpass = 4;
    dependencies[8].srcStageMask =
        VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT |
        VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;
    dependencies[8].dstStageMask =
        VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT |
        VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
    dependencies[8].srcAccessMask =
        VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
    dependencies[8].dstAccessMask =
        VK_ACCESS_INPUT_ATTACHMENT_READ_BIT |
        VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT;

    dependencies[9].srcSubpass = 4;
    dependencies[9].dstSubpass = VK_SUBPASS_EXTERNAL;
    dependencies[9].srcStageMask =
        VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dependencies[9].dstStageMask = VK_PIPELINE_STAGE_TRANSFER_BIT;
    dependencies[9].srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
    dependencies[9].dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;

    // The weighted composite consumed the emission attachment as an input in
    // subpass 2; the sorted subpass clears and rewrites it as colour output.
    dependencies[10].srcSubpass = 2;
    dependencies[10].dstSubpass = 4;
    dependencies[10].srcStageMask = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
    dependencies[10].dstStageMask =
        VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dependencies[10].srcAccessMask = VK_ACCESS_INPUT_ATTACHMENT_READ_BIT;
    dependencies[10].dstAccessMask =
        VK_ACCESS_COLOR_ATTACHMENT_READ_BIT |
        VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;

    dependencies[11].srcSubpass = 4;
    dependencies[11].dstSubpass = 5;
    dependencies[11].srcStageMask =
        VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dependencies[11].dstStageMask =
        VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT |
        VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dependencies[11].srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
    dependencies[11].dstAccessMask =
        VK_ACCESS_INPUT_ATTACHMENT_READ_BIT |
        VK_ACCESS_COLOR_ATTACHMENT_READ_BIT |
        VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;

    dependencies[12].srcSubpass = 5;
    dependencies[12].dstSubpass = VK_SUBPASS_EXTERNAL;
    dependencies[12].srcStageMask =
        VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dependencies[12].dstStageMask = VK_PIPELINE_STAGE_TRANSFER_BIT;
    dependencies[12].srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
    dependencies[12].dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;

    const std::array<VkAttachmentDescription, 10> attachments = {
        colorAttachment,     depthAttachment,       accumulationAttachment,    revealageAttachment,
        emissiveAttachment,  linearDepthAttachment, aovAccumulationAttachment, aovAccumulationAttachment,
        aovOutputAttachment, aovOutputAttachment,
    };
    const std::array<VkSubpassDescription, 6> subpasses = {
        depthSubpass,
        accumulationSubpass,
        compositeSubpass,
        fastBasicSubpass,
        sortedSubpass,
        sortedEmissionSubpass,
    };

    VkRenderPassCreateInfo renderPassInfo{VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO};
    renderPassInfo.attachmentCount = static_cast<std::uint32_t>(attachments.size());
    renderPassInfo.pAttachments = attachments.data();
    renderPassInfo.subpassCount = static_cast<std::uint32_t>(subpasses.size());
    renderPassInfo.pSubpasses = subpasses.data();
    renderPassInfo.dependencyCount = static_cast<std::uint32_t>(dependencies.size());
    renderPassInfo.pDependencies = dependencies.data();
    Check(vkCreateRenderPass(device_, &renderPassInfo, nullptr, &resources->renderPass), "vkCreateRenderPass(exr)");
}

void VulkanViewportShell::CreateExrExportPipelines(ExrExportResources* resources) {
    if (resources == nullptr || resources->renderPass == VK_NULL_HANDLE) {
        return;
    }

    const auto vertexShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "pointcloud_preview.vert.spv").string());
    // Depth passes reuse the DEPTH_PREPASS-specialised vertex variants; they
    // produce bit-identical position/size/opacity/depth without the
    // colour-only work, so the exported depth AOV is unchanged.
    const auto prepassVertexShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "pointcloud_preview_depth_prepass.vert.spv").string());
    const auto accumulationFragmentShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "pointcloud_exr_accumulation.frag.spv").string());
    const auto sortedAlphaFragmentShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "pointcloud_exr_sorted_alpha.frag.spv").string());
    const auto constantSimpleVertexShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "pointcloud_constant_simple.vert.spv").string());
    const auto constantSimpleFragmentShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "pointcloud_exr_constant_simple_accumulation.frag.spv").string());
    const auto fastBasicVertexShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "pointcloud_fast_basic.vert.spv").string());
    const auto fastBasicFragmentShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "pointcloud_fast_basic.frag.spv").string());
    const auto fastBasicDepthFragmentShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "pointcloud_fast_basic_depth.frag.spv").string());
    const auto depthFragmentShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "pointcloud_export_depth.frag.spv").string());
    const auto surfelVertexShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "pointcloud_surfel.vert.spv").string());
    const auto surfelPrepassVertexShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "pointcloud_surfel_depth_prepass.vert.spv").string());
    const auto surfelAccumulationFragmentShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "pointcloud_surfel_exr_accumulation.frag.spv").string());
    const auto surfelConstantSimpleVertexShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "pointcloud_surfel_constant_simple.vert.spv").string());
    const auto surfelConstantSimpleFragmentShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "pointcloud_surfel_exr_constant_simple_accumulation.frag.spv").string());
    const auto surfelDepthFragmentShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "pointcloud_surfel_export_depth.frag.spv").string());
    const auto rainVertexShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "rain_particle.vert.spv").string());
    const auto rainFragmentShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "rain_particle.frag.spv").string());

    const auto vertexModule = CreateShaderModule(device_, vertexShaderCode, "vkCreateShaderModule(exr point vertex)");
    const auto prepassVertexModule =
        CreateShaderModule(device_, prepassVertexShaderCode, "vkCreateShaderModule(exr point prepass vertex)");
    const auto accumulationFragmentModule =
        CreateShaderModule(device_, accumulationFragmentShaderCode, "vkCreateShaderModule(exr point accumulation fragment)");
    const auto sortedAlphaFragmentModule =
        CreateShaderModule(device_, sortedAlphaFragmentShaderCode, "vkCreateShaderModule(exr point sorted alpha fragment)");
    const auto constantSimpleVertexModule =
        CreateShaderModule(device_, constantSimpleVertexShaderCode, "vkCreateShaderModule(exr point simple vertex)");
    const auto constantSimpleFragmentModule =
        CreateShaderModule(
            device_,
            constantSimpleFragmentShaderCode,
            "vkCreateShaderModule(exr point simple accumulation fragment)");
    const auto fastBasicVertexModule =
        CreateShaderModule(device_, fastBasicVertexShaderCode, "vkCreateShaderModule(exr point fast basic vertex)");
    const auto fastBasicFragmentModule =
        CreateShaderModule(device_, fastBasicFragmentShaderCode, "vkCreateShaderModule(exr point fast basic fragment)");
    const auto fastBasicDepthFragmentModule =
        CreateShaderModule(
            device_,
            fastBasicDepthFragmentShaderCode,
            "vkCreateShaderModule(exr point fast basic depth fragment)");
    const auto depthFragmentModule =
        CreateShaderModule(device_, depthFragmentShaderCode, "vkCreateShaderModule(exr point depth fragment)");
    const auto surfelVertexModule =
        CreateShaderModule(device_, surfelVertexShaderCode, "vkCreateShaderModule(exr surfel vertex)");
    const auto surfelPrepassVertexModule =
        CreateShaderModule(device_, surfelPrepassVertexShaderCode, "vkCreateShaderModule(exr surfel prepass vertex)");
    const auto surfelAccumulationFragmentModule =
        CreateShaderModule(device_, surfelAccumulationFragmentShaderCode, "vkCreateShaderModule(exr surfel accumulation fragment)");
    const auto surfelConstantSimpleVertexModule =
        CreateShaderModule(device_, surfelConstantSimpleVertexShaderCode, "vkCreateShaderModule(exr surfel simple vertex)");
    const auto surfelConstantSimpleFragmentModule =
        CreateShaderModule(
            device_,
            surfelConstantSimpleFragmentShaderCode,
            "vkCreateShaderModule(exr surfel simple accumulation fragment)");
    const auto surfelDepthFragmentModule =
        CreateShaderModule(device_, surfelDepthFragmentShaderCode, "vkCreateShaderModule(exr surfel depth fragment)");
    const auto rainVertexModule =
        CreateShaderModule(device_, rainVertexShaderCode, "vkCreateShaderModule(exr rain vertex)");
    const auto rainFragmentModule =
        CreateShaderModule(device_, rainFragmentShaderCode, "vkCreateShaderModule(exr rain fragment)");

    VkPipelineShaderStageCreateInfo vertexStage{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    vertexStage.stage = VK_SHADER_STAGE_VERTEX_BIT;
    vertexStage.module = vertexModule;
    vertexStage.pName = "main";

    VkPipelineShaderStageCreateInfo prepassVertexStage{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    prepassVertexStage.stage = VK_SHADER_STAGE_VERTEX_BIT;
    prepassVertexStage.module = prepassVertexModule;
    prepassVertexStage.pName = "main";

    VkPipelineShaderStageCreateInfo surfelVertexStage{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    surfelVertexStage.stage = VK_SHADER_STAGE_VERTEX_BIT;
    surfelVertexStage.module = surfelVertexModule;
    surfelVertexStage.pName = "main";

    VkPipelineShaderStageCreateInfo surfelPrepassVertexStage{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    surfelPrepassVertexStage.stage = VK_SHADER_STAGE_VERTEX_BIT;
    surfelPrepassVertexStage.module = surfelPrepassVertexModule;
    surfelPrepassVertexStage.pName = "main";

    VkPipelineShaderStageCreateInfo constantSimpleVertexStage{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    constantSimpleVertexStage.stage = VK_SHADER_STAGE_VERTEX_BIT;
    constantSimpleVertexStage.module = constantSimpleVertexModule;
    constantSimpleVertexStage.pName = "main";

    VkPipelineShaderStageCreateInfo fastBasicVertexStage{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    fastBasicVertexStage.stage = VK_SHADER_STAGE_VERTEX_BIT;
    fastBasicVertexStage.module = fastBasicVertexModule;
    fastBasicVertexStage.pName = "main";

    VkPipelineShaderStageCreateInfo surfelConstantSimpleVertexStage{
        VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    surfelConstantSimpleVertexStage.stage = VK_SHADER_STAGE_VERTEX_BIT;
    surfelConstantSimpleVertexStage.module = surfelConstantSimpleVertexModule;
    surfelConstantSimpleVertexStage.pName = "main";

    const std::array<VkVertexInputBindingDescription, 2> bindingDescriptions = {
        VkVertexInputBindingDescription{0, sizeof(invisible_places::io::Float3), VK_VERTEX_INPUT_RATE_VERTEX},
        VkVertexInputBindingDescription{1, sizeof(std::uint32_t), VK_VERTEX_INPUT_RATE_VERTEX},
    };
    const std::array<VkVertexInputAttributeDescription, 2> attributeDescriptions = {
        VkVertexInputAttributeDescription{0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0},
        VkVertexInputAttributeDescription{1, 1, VK_FORMAT_R8G8B8A8_UNORM, 0},
    };

    VkPipelineVertexInputStateCreateInfo vertexInputInfo{
        VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO};
    vertexInputInfo.vertexBindingDescriptionCount = static_cast<std::uint32_t>(bindingDescriptions.size());
    vertexInputInfo.pVertexBindingDescriptions = bindingDescriptions.data();
    vertexInputInfo.vertexAttributeDescriptionCount = static_cast<std::uint32_t>(attributeDescriptions.size());
    vertexInputInfo.pVertexAttributeDescriptions = attributeDescriptions.data();

    VkPipelineInputAssemblyStateCreateInfo inputAssembly{
        VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO};
    inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_POINT_LIST;

    VkPipelineVertexInputStateCreateInfo surfelVertexInputInfo{
        VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO};

    VkPipelineInputAssemblyStateCreateInfo surfelInputAssembly{
        VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO};
    surfelInputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;

    VkPipelineViewportStateCreateInfo viewportState{VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO};
    viewportState.viewportCount = 1;
    viewportState.scissorCount = 1;

    VkPipelineRasterizationStateCreateInfo rasterizer{
        VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO};
    rasterizer.polygonMode = VK_POLYGON_MODE_FILL;
    rasterizer.cullMode = VK_CULL_MODE_NONE;
    rasterizer.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
    rasterizer.lineWidth = 1.0F;

    VkPipelineMultisampleStateCreateInfo multisampling{
        VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO};
    multisampling.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
    multisampling.minSampleShading = 1.0F;

    const std::array<VkDynamicState, 2> dynamicStates = {VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR};
    VkPipelineDynamicStateCreateInfo dynamicState{VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO};
    dynamicState.dynamicStateCount = static_cast<std::uint32_t>(dynamicStates.size());
    dynamicState.pDynamicStates = dynamicStates.data();

    auto createPointPipeline =
        [&](const VkPipelineShaderStageCreateInfo& selectedVertexStage,
            const VkPipelineVertexInputStateCreateInfo& selectedVertexInputInfo,
            const VkPipelineInputAssemblyStateCreateInfo& selectedInputAssembly,
            VkShaderModule fragmentModule,
            std::uint32_t subpass,
            const std::vector<VkPipelineColorBlendAttachmentState>& blendAttachments,
            bool depthTest,
            bool depthWrite,
            VkCompareOp depthCompare,
            const char* label,
            VkPipeline* pipeline) {
            VkPipelineShaderStageCreateInfo fragmentStage{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
            fragmentStage.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
            fragmentStage.module = fragmentModule;
            fragmentStage.pName = "main";
            const std::array<VkPipelineShaderStageCreateInfo, 2> shaderStages = {
                selectedVertexStage,
                fragmentStage};

            VkPipelineDepthStencilStateCreateInfo depthStencil{
                VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO};
            depthStencil.depthTestEnable = depthTest ? VK_TRUE : VK_FALSE;
            depthStencil.depthWriteEnable = depthWrite ? VK_TRUE : VK_FALSE;
            depthStencil.depthCompareOp = depthCompare;

            VkPipelineColorBlendStateCreateInfo colorBlending{
                VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO};
            colorBlending.attachmentCount = static_cast<std::uint32_t>(blendAttachments.size());
            colorBlending.pAttachments = blendAttachments.data();

            VkGraphicsPipelineCreateInfo pipelineInfo{VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO};
            pipelineInfo.stageCount = static_cast<std::uint32_t>(shaderStages.size());
            pipelineInfo.pStages = shaderStages.data();
            pipelineInfo.pVertexInputState = &selectedVertexInputInfo;
            pipelineInfo.pInputAssemblyState = &selectedInputAssembly;
            pipelineInfo.pViewportState = &viewportState;
            pipelineInfo.pRasterizationState = &rasterizer;
            pipelineInfo.pMultisampleState = &multisampling;
            pipelineInfo.pDepthStencilState = &depthStencil;
            pipelineInfo.pColorBlendState = &colorBlending;
            pipelineInfo.pDynamicState = &dynamicState;
            pipelineInfo.layout = pointPipelineLayout_;
            pipelineInfo.renderPass = resources->renderPass;
            pipelineInfo.subpass = subpass;
            Check(vkCreateGraphicsPipelines(device_, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, pipeline), label);
        };

    VkPipelineColorBlendAttachmentState linearDepthBlend{};
    linearDepthBlend.colorWriteMask = VK_COLOR_COMPONENT_R_BIT;
    linearDepthBlend.blendEnable = VK_FALSE;

    VkPipelineColorBlendAttachmentState opaqueColorBlend{};
    opaqueColorBlend.colorWriteMask =
        VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
    opaqueColorBlend.blendEnable = VK_FALSE;

    createPointPipeline(
        prepassVertexStage,
        vertexInputInfo,
        inputAssembly,
        depthFragmentModule,
        0,
        std::vector<VkPipelineColorBlendAttachmentState>{linearDepthBlend},
        true,
        true,
        VK_COMPARE_OP_LESS,
        "vkCreateGraphicsPipelines(exr point depth)",
        &resources->pointDepthPipeline);
    createPointPipeline(
        vertexStage,
        vertexInputInfo,
        inputAssembly,
        accumulationFragmentModule,
        1,
        std::vector<VkPipelineColorBlendAttachmentState>{
            MakeAdditiveBlendAttachment(),
            MakeRevealageBlendAttachment(),
            MakeAdditiveBlendAttachment(),
            MakeAdditiveBlendAttachment(),
            MakeAdditiveBlendAttachment()},
        false,
        false,
        VK_COMPARE_OP_ALWAYS,
        "vkCreateGraphicsPipelines(exr point accumulation)",
        &resources->pointAccumulationPipeline);

    createPointPipeline(
        constantSimpleVertexStage,
        vertexInputInfo,
        inputAssembly,
        constantSimpleFragmentModule,
        1,
        std::vector<VkPipelineColorBlendAttachmentState>{
            MakeAdditiveBlendAttachment(),
            MakeRevealageBlendAttachment(),
            MakeAdditiveBlendAttachment(),
            MakeAdditiveBlendAttachment(),
            MakeAdditiveBlendAttachment()},
        false,
        false,
        VK_COMPARE_OP_ALWAYS,
        "vkCreateGraphicsPipelines(exr point simple accumulation)",
        &resources->pointConstantSimpleAccumulationPipeline);

    createPointPipeline(
        fastBasicVertexStage,
        vertexInputInfo,
        inputAssembly,
        fastBasicDepthFragmentModule,
        0,
        std::vector<VkPipelineColorBlendAttachmentState>{linearDepthBlend},
        true,
        true,
        VK_COMPARE_OP_LESS,
        "vkCreateGraphicsPipelines(exr point fast basic depth)",
        &resources->pointFastBasicDepthPipeline);

    createPointPipeline(
        fastBasicVertexStage,
        vertexInputInfo,
        inputAssembly,
        fastBasicFragmentModule,
        3,
        std::vector<VkPipelineColorBlendAttachmentState>{
            opaqueColorBlend,
            VkPipelineColorBlendAttachmentState{},
            VkPipelineColorBlendAttachmentState{}},
        true,
        false,
        VK_COMPARE_OP_LESS_OR_EQUAL,
        "vkCreateGraphicsPipelines(exr point fast basic)",
        &resources->pointFastBasicPipeline);

    createPointPipeline(
        prepassVertexStage,
        vertexInputInfo,
        inputAssembly,
        depthFragmentModule,
        0,
        std::vector<VkPipelineColorBlendAttachmentState>{linearDepthBlend},
        true,
        false,
        VK_COMPARE_OP_LESS_OR_EQUAL,
        "vkCreateGraphicsPipelines(exr point sorted depth AOV)",
        &resources->pointSortedDepthAovPipeline);

    createPointPipeline(
        vertexStage,
        vertexInputInfo,
        inputAssembly,
        sortedAlphaFragmentModule,
        4,
        std::vector<VkPipelineColorBlendAttachmentState>{
            MakeAlphaBlendAttachment(),
            MakeAlphaBlendAttachment(),
            MakeAlphaBlendAttachment(),
            MakeAdditiveBlendAttachment()},
        true,
        false,
        VK_COMPARE_OP_LESS_OR_EQUAL,
        "vkCreateGraphicsPipelines(exr point sorted alpha)",
        &resources->pointSortedAlphaPipeline);
    createPointPipeline(
        vertexStage,
        vertexInputInfo,
        inputAssembly,
        sortedAlphaFragmentModule,
        4,
        std::vector<VkPipelineColorBlendAttachmentState>{
            MakeAlphaBlendAttachment(),
            MakeAlphaBlendAttachment(),
            MakeAlphaBlendAttachment(),
            MakeAdditiveBlendAttachment()},
        false,
        false,
        VK_COMPARE_OP_ALWAYS,
        "vkCreateGraphicsPipelines(exr point sorted alpha hybrid depth)",
        &resources->pointSortedAlphaHybridPipeline);

    createPointPipeline(
        surfelPrepassVertexStage,
        surfelVertexInputInfo,
        surfelInputAssembly,
        surfelDepthFragmentModule,
        0,
        std::vector<VkPipelineColorBlendAttachmentState>{linearDepthBlend},
        true,
        true,
        VK_COMPARE_OP_LESS,
        "vkCreateGraphicsPipelines(exr surfel depth)",
        &resources->surfelDepthPipeline);
    createPointPipeline(
        surfelVertexStage,
        surfelVertexInputInfo,
        surfelInputAssembly,
        surfelAccumulationFragmentModule,
        1,
        std::vector<VkPipelineColorBlendAttachmentState>{
            MakeAdditiveBlendAttachment(),
            MakeRevealageBlendAttachment(),
            MakeAdditiveBlendAttachment(),
            MakeAdditiveBlendAttachment(),
            MakeAdditiveBlendAttachment()},
        false,
        false,
        VK_COMPARE_OP_ALWAYS,
        "vkCreateGraphicsPipelines(exr surfel accumulation)",
        &resources->surfelAccumulationPipeline);

    createPointPipeline(
        surfelConstantSimpleVertexStage,
        surfelVertexInputInfo,
        surfelInputAssembly,
        surfelConstantSimpleFragmentModule,
        1,
        std::vector<VkPipelineColorBlendAttachmentState>{
            MakeAdditiveBlendAttachment(),
            MakeRevealageBlendAttachment(),
            MakeAdditiveBlendAttachment(),
            MakeAdditiveBlendAttachment(),
            MakeAdditiveBlendAttachment()},
        false,
        false,
        VK_COMPARE_OP_ALWAYS,
        "vkCreateGraphicsPipelines(exr surfel simple accumulation)",
        &resources->surfelConstantSimpleAccumulationPipeline);

    VkPipelineShaderStageCreateInfo rainVertexStage{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    rainVertexStage.stage = VK_SHADER_STAGE_VERTEX_BIT;
    rainVertexStage.module = rainVertexModule;
    rainVertexStage.pName = "main";
    VkPipelineShaderStageCreateInfo rainFragmentStage{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    rainFragmentStage.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
    rainFragmentStage.module = rainFragmentModule;
    rainFragmentStage.pName = "main";
    const std::array rainStages = {rainVertexStage, rainFragmentStage};
    VkPipelineVertexInputStateCreateInfo rainVertexInput{
        VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO};
    VkPipelineInputAssemblyStateCreateInfo rainInputAssembly{
        VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO};
    rainInputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
    VkPipelineDepthStencilStateCreateInfo rainDepthStencil{
        VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO};
    rainDepthStencil.depthTestEnable = VK_TRUE;
    rainDepthStencil.depthWriteEnable = VK_FALSE;
    rainDepthStencil.depthCompareOp = VK_COMPARE_OP_LESS_OR_EQUAL;
    const std::array rainBlendAttachments = {
        MakeAdditiveBlendAttachment(),
        MakeRevealageBlendAttachment(),
        MakeAdditiveBlendAttachment(),
        MakeAdditiveBlendAttachment(),
        MakeAdditiveBlendAttachment(),
    };
    VkPipelineColorBlendStateCreateInfo rainColourBlend{
        VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO};
    rainColourBlend.attachmentCount = static_cast<std::uint32_t>(rainBlendAttachments.size());
    rainColourBlend.pAttachments = rainBlendAttachments.data();
    VkGraphicsPipelineCreateInfo rainPipelineInfo{VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO};
    rainPipelineInfo.stageCount = static_cast<std::uint32_t>(rainStages.size());
    rainPipelineInfo.pStages = rainStages.data();
    rainPipelineInfo.pVertexInputState = &rainVertexInput;
    rainPipelineInfo.pInputAssemblyState = &rainInputAssembly;
    rainPipelineInfo.pViewportState = &viewportState;
    rainPipelineInfo.pRasterizationState = &rasterizer;
    rainPipelineInfo.pMultisampleState = &multisampling;
    rainPipelineInfo.pDepthStencilState = &rainDepthStencil;
    rainPipelineInfo.pColorBlendState = &rainColourBlend;
    rainPipelineInfo.pDynamicState = &dynamicState;
    rainPipelineInfo.layout = rainPipelineLayout_;
    rainPipelineInfo.renderPass = resources->renderPass;
    rainPipelineInfo.subpass = 1U;
    Check(
        vkCreateGraphicsPipelines(
            device_,
            VK_NULL_HANDLE,
            1U,
            &rainPipelineInfo,
            nullptr,
            &resources->rainPipeline),
        "vkCreateGraphicsPipelines(exr rain)");

    vkDestroyShaderModule(device_, rainFragmentModule, nullptr);
    vkDestroyShaderModule(device_, rainVertexModule, nullptr);
    vkDestroyShaderModule(device_, surfelConstantSimpleFragmentModule, nullptr);
    vkDestroyShaderModule(device_, surfelConstantSimpleVertexModule, nullptr);
    vkDestroyShaderModule(device_, surfelDepthFragmentModule, nullptr);
    vkDestroyShaderModule(device_, surfelAccumulationFragmentModule, nullptr);
    vkDestroyShaderModule(device_, surfelPrepassVertexModule, nullptr);
    vkDestroyShaderModule(device_, surfelVertexModule, nullptr);
    vkDestroyShaderModule(device_, fastBasicDepthFragmentModule, nullptr);
    vkDestroyShaderModule(device_, fastBasicFragmentModule, nullptr);
    vkDestroyShaderModule(device_, fastBasicVertexModule, nullptr);
    vkDestroyShaderModule(device_, depthFragmentModule, nullptr);
    vkDestroyShaderModule(device_, constantSimpleFragmentModule, nullptr);
    vkDestroyShaderModule(device_, constantSimpleVertexModule, nullptr);
    vkDestroyShaderModule(device_, sortedAlphaFragmentModule, nullptr);
    vkDestroyShaderModule(device_, accumulationFragmentModule, nullptr);
    vkDestroyShaderModule(device_, prepassVertexModule, nullptr);
    vkDestroyShaderModule(device_, vertexModule, nullptr);

    const auto compositeVertexShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "gsplat_composite.vert.spv").string());
    const auto compositeFragmentShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "pointcloud_exr_composite.frag.spv").string());
    const auto compositeVertexModule =
        CreateShaderModule(device_, compositeVertexShaderCode, "vkCreateShaderModule(exr composite vertex)");
    const auto compositeFragmentModule =
        CreateShaderModule(device_, compositeFragmentShaderCode, "vkCreateShaderModule(exr composite fragment)");

    VkPipelineShaderStageCreateInfo compositeVertexStage{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    compositeVertexStage.stage = VK_SHADER_STAGE_VERTEX_BIT;
    compositeVertexStage.module = compositeVertexModule;
    compositeVertexStage.pName = "main";
    VkPipelineShaderStageCreateInfo compositeFragmentStage{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    compositeFragmentStage.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
    compositeFragmentStage.module = compositeFragmentModule;
    compositeFragmentStage.pName = "main";
    const std::array<VkPipelineShaderStageCreateInfo, 2> compositeStages = {
        compositeVertexStage,
        compositeFragmentStage,
    };

    VkPipelineVertexInputStateCreateInfo compositeVertexInput{
        VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO};
    VkPipelineInputAssemblyStateCreateInfo compositeInputAssembly{
        VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO};
    compositeInputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
    VkPipelineDepthStencilStateCreateInfo compositeDepthStencil{
        VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO};
    compositeDepthStencil.depthTestEnable = VK_FALSE;
    compositeDepthStencil.depthWriteEnable = VK_FALSE;
    const std::array<VkPipelineColorBlendAttachmentState, 3> compositeBlendAttachments = {
        MakeAlphaBlendAttachment(),
        VkPipelineColorBlendAttachmentState{
            VK_FALSE,
            VK_BLEND_FACTOR_ZERO,
            VK_BLEND_FACTOR_ZERO,
            VK_BLEND_OP_ADD,
            VK_BLEND_FACTOR_ZERO,
            VK_BLEND_FACTOR_ZERO,
            VK_BLEND_OP_ADD,
            VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT |
                VK_COLOR_COMPONENT_A_BIT},
        VkPipelineColorBlendAttachmentState{
            VK_FALSE,
            VK_BLEND_FACTOR_ZERO,
            VK_BLEND_FACTOR_ZERO,
            VK_BLEND_OP_ADD,
            VK_BLEND_FACTOR_ZERO,
            VK_BLEND_FACTOR_ZERO,
            VK_BLEND_OP_ADD,
            VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT |
                VK_COLOR_COMPONENT_A_BIT},
    };
    VkPipelineColorBlendStateCreateInfo compositeColorBlending{
        VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO};
    compositeColorBlending.attachmentCount = static_cast<std::uint32_t>(compositeBlendAttachments.size());
    compositeColorBlending.pAttachments = compositeBlendAttachments.data();

    VkGraphicsPipelineCreateInfo compositePipelineInfo{VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO};
    compositePipelineInfo.stageCount = static_cast<std::uint32_t>(compositeStages.size());
    compositePipelineInfo.pStages = compositeStages.data();
    compositePipelineInfo.pVertexInputState = &compositeVertexInput;
    compositePipelineInfo.pInputAssemblyState = &compositeInputAssembly;
    compositePipelineInfo.pViewportState = &viewportState;
    compositePipelineInfo.pRasterizationState = &rasterizer;
    compositePipelineInfo.pMultisampleState = &multisampling;
    compositePipelineInfo.pDepthStencilState = &compositeDepthStencil;
    compositePipelineInfo.pColorBlendState = &compositeColorBlending;
    compositePipelineInfo.pDynamicState = &dynamicState;
    compositePipelineInfo.layout = compositePipelineLayout_;
    compositePipelineInfo.renderPass = resources->renderPass;
    compositePipelineInfo.subpass = 2;
    Check(
        vkCreateGraphicsPipelines(
            device_,
            VK_NULL_HANDLE,
            1,
            &compositePipelineInfo,
            nullptr,
            &resources->compositePipeline),
        "vkCreateGraphicsPipelines(exr composite)");

    vkDestroyShaderModule(device_, compositeFragmentModule, nullptr);

    // Sorted emission composite: applies the weighted exponential emission
    // response over the sorted colour output (subpass 5), reusing the shared
    // composite descriptor set's emission slot.
    const auto sortedEmissionFragmentShaderCode = ReadBinaryFile(
        (std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} /
         "pointcloud_sorted_emission_composite.frag.spv")
            .string());
    const auto sortedEmissionFragmentModule = CreateShaderModule(
        device_,
        sortedEmissionFragmentShaderCode,
        "vkCreateShaderModule(exr sorted emission composite fragment)");
    VkPipelineShaderStageCreateInfo sortedEmissionFragmentStage{
        VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    sortedEmissionFragmentStage.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
    sortedEmissionFragmentStage.module = sortedEmissionFragmentModule;
    sortedEmissionFragmentStage.pName = "main";
    const std::array<VkPipelineShaderStageCreateInfo, 2>
        sortedEmissionStages = {
            compositeVertexStage,
            sortedEmissionFragmentStage,
        };
    VkPipelineColorBlendAttachmentState sortedEmissionBlend{};
    sortedEmissionBlend.colorWriteMask =
        VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT |
        VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
    sortedEmissionBlend.blendEnable = VK_TRUE;
    sortedEmissionBlend.srcColorBlendFactor = VK_BLEND_FACTOR_ONE;
    sortedEmissionBlend.dstColorBlendFactor = VK_BLEND_FACTOR_ONE;
    sortedEmissionBlend.colorBlendOp = VK_BLEND_OP_ADD;
    sortedEmissionBlend.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
    sortedEmissionBlend.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
    sortedEmissionBlend.alphaBlendOp = VK_BLEND_OP_MAX;
    VkPipelineColorBlendStateCreateInfo sortedEmissionBlending{
        VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO};
    sortedEmissionBlending.attachmentCount = 1;
    sortedEmissionBlending.pAttachments = &sortedEmissionBlend;
    compositePipelineInfo.stageCount =
        static_cast<std::uint32_t>(sortedEmissionStages.size());
    compositePipelineInfo.pStages = sortedEmissionStages.data();
    compositePipelineInfo.pColorBlendState = &sortedEmissionBlending;
    compositePipelineInfo.subpass = 5;
    Check(
        vkCreateGraphicsPipelines(
            device_,
            VK_NULL_HANDLE,
            1,
            &compositePipelineInfo,
            nullptr,
            &resources->pointSortedEmissionCompositePipeline),
        "vkCreateGraphicsPipelines(exr sorted emission composite)");
    vkDestroyShaderModule(device_, sortedEmissionFragmentModule, nullptr);
    vkDestroyShaderModule(device_, compositeVertexModule, nullptr);
}

void VulkanViewportShell::CreateGaussianSplatPipeline() {
    const auto vertexShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "gsplat_accumulation.vert.spv").string());
    const auto fragmentShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "gsplat_accumulation.frag.spv").string());

    const auto vertexModule = CreateShaderModule(device_, vertexShaderCode, "vkCreateShaderModule(gsplat vertex)");
    const auto fragmentModule = CreateShaderModule(device_, fragmentShaderCode, "vkCreateShaderModule(gsplat fragment)");

    VkPipelineShaderStageCreateInfo vertexStage{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    vertexStage.stage = VK_SHADER_STAGE_VERTEX_BIT;
    vertexStage.module = vertexModule;
    vertexStage.pName = "main";

    VkPipelineShaderStageCreateInfo fragmentStage{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    fragmentStage.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
    fragmentStage.module = fragmentModule;
    fragmentStage.pName = "main";

    const std::array<VkPipelineShaderStageCreateInfo, 2> shaderStages = {vertexStage, fragmentStage};

    VkPipelineVertexInputStateCreateInfo vertexInputInfo{
        VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO};

    VkPipelineInputAssemblyStateCreateInfo inputAssembly{
        VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO};
    inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;

    VkPipelineViewportStateCreateInfo viewportState{VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO};
    viewportState.viewportCount = 1;
    viewportState.scissorCount = 1;

    VkPipelineRasterizationStateCreateInfo rasterizer{
        VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO};
    rasterizer.polygonMode = VK_POLYGON_MODE_FILL;
    rasterizer.cullMode = VK_CULL_MODE_NONE;
    rasterizer.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
    rasterizer.lineWidth = 1.0F;

    VkPipelineMultisampleStateCreateInfo multisampling{
        VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO};
    multisampling.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
    multisampling.minSampleShading = 1.0F;

    VkPipelineDepthStencilStateCreateInfo depthStencil{
        VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO};
    depthStencil.depthTestEnable = VK_TRUE;
    depthStencil.depthWriteEnable = VK_FALSE;
    depthStencil.depthCompareOp = VK_COMPARE_OP_LESS_OR_EQUAL;

    const std::array<VkPipelineColorBlendAttachmentState, 3> colorBlendAttachments = {
        MakeAdditiveBlendAttachment(),
        MakeRevealageBlendAttachment(),
        MakeAdditiveBlendAttachment(),
    };
    VkPipelineColorBlendStateCreateInfo colorBlending{
        VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO};
    colorBlending.attachmentCount = static_cast<std::uint32_t>(colorBlendAttachments.size());
    colorBlending.pAttachments = colorBlendAttachments.data();

    const std::array<VkDynamicState, 2> dynamicStates = {VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR};
    VkPipelineDynamicStateCreateInfo dynamicState{VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO};
    dynamicState.dynamicStateCount = static_cast<std::uint32_t>(dynamicStates.size());
    dynamicState.pDynamicStates = dynamicStates.data();

    VkPushConstantRange pushConstantRange{};
    pushConstantRange.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
    pushConstantRange.size = sizeof(GaussianSplatPushConstants);

    VkPipelineLayoutCreateInfo pipelineLayoutInfo{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
    pipelineLayoutInfo.setLayoutCount = 1;
    pipelineLayoutInfo.pSetLayouts = &gaussianSplatDescriptorSetLayout_;
    pipelineLayoutInfo.pushConstantRangeCount = 1;
    pipelineLayoutInfo.pPushConstantRanges = &pushConstantRange;

    Check(vkCreatePipelineLayout(device_, &pipelineLayoutInfo, nullptr, &gaussianSplatPipelineLayout_), "vkCreatePipelineLayout(gsplat)");

    VkGraphicsPipelineCreateInfo pipelineInfo{VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO};
    pipelineInfo.stageCount = static_cast<std::uint32_t>(shaderStages.size());
    pipelineInfo.pStages = shaderStages.data();
    pipelineInfo.pVertexInputState = &vertexInputInfo;
    pipelineInfo.pInputAssemblyState = &inputAssembly;
    pipelineInfo.pViewportState = &viewportState;
    pipelineInfo.pRasterizationState = &rasterizer;
    pipelineInfo.pMultisampleState = &multisampling;
    pipelineInfo.pDepthStencilState = &depthStencil;
    pipelineInfo.pColorBlendState = &colorBlending;
    pipelineInfo.pDynamicState = &dynamicState;
    pipelineInfo.layout = gaussianSplatPipelineLayout_;
    pipelineInfo.renderPass = renderPass_;
    pipelineInfo.subpass = 1;

    Check(vkCreateGraphicsPipelines(device_, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &gaussianSplatPipeline_), "vkCreateGraphicsPipelines(gsplat)");

    vkDestroyShaderModule(device_, fragmentModule, nullptr);
    vkDestroyShaderModule(device_, vertexModule, nullptr);
}

void VulkanViewportShell::CreateHighQualityGaussianSplatPipeline() {
    const auto vertexShaderCode =
        ReadBinaryFile(
            (std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "gsplat_high_quality.vert.spv").string());
    const auto fragmentShaderCode =
        ReadBinaryFile(
            (std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "gsplat_high_quality.frag.spv").string());

    const auto vertexModule =
        CreateShaderModule(device_, vertexShaderCode, "vkCreateShaderModule(gsplat_hq vertex)");
    const auto fragmentModule =
        CreateShaderModule(device_, fragmentShaderCode, "vkCreateShaderModule(gsplat_hq fragment)");

    VkPipelineShaderStageCreateInfo vertexStage{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    vertexStage.stage = VK_SHADER_STAGE_VERTEX_BIT;
    vertexStage.module = vertexModule;
    vertexStage.pName = "main";

    VkPipelineShaderStageCreateInfo fragmentStage{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    fragmentStage.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
    fragmentStage.module = fragmentModule;
    fragmentStage.pName = "main";

    const std::array<VkPipelineShaderStageCreateInfo, 2> shaderStages = {vertexStage, fragmentStage};

    VkPipelineVertexInputStateCreateInfo vertexInputInfo{
        VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO};

    VkPipelineInputAssemblyStateCreateInfo inputAssembly{
        VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO};
    inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;

    VkPipelineViewportStateCreateInfo viewportState{VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO};
    viewportState.viewportCount = 1;
    viewportState.scissorCount = 1;

    VkPipelineRasterizationStateCreateInfo rasterizer{
        VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO};
    rasterizer.polygonMode = VK_POLYGON_MODE_FILL;
    rasterizer.cullMode = VK_CULL_MODE_NONE;
    rasterizer.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
    rasterizer.lineWidth = 1.0F;

    VkPipelineMultisampleStateCreateInfo multisampling{
        VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO};
    multisampling.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
    multisampling.minSampleShading = 1.0F;

    VkPipelineDepthStencilStateCreateInfo depthStencil{
        VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO};
    depthStencil.depthTestEnable = VK_TRUE;
    depthStencil.depthWriteEnable = VK_FALSE;
    depthStencil.depthCompareOp = VK_COMPARE_OP_LESS_OR_EQUAL;

    const auto colorBlendAttachment = MakePremultipliedAlphaBlendAttachment();
    VkPipelineColorBlendStateCreateInfo colorBlending{
        VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO};
    colorBlending.attachmentCount = 1;
    colorBlending.pAttachments = &colorBlendAttachment;

    const std::array<VkDynamicState, 2> dynamicStates = {VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR};
    VkPipelineDynamicStateCreateInfo dynamicState{VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO};
    dynamicState.dynamicStateCount = static_cast<std::uint32_t>(dynamicStates.size());
    dynamicState.pDynamicStates = dynamicStates.data();

    VkPushConstantRange pushConstantRange{};
    pushConstantRange.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
    pushConstantRange.size = sizeof(HighQualityGaussianPushConstants);

    VkPipelineLayoutCreateInfo pipelineLayoutInfo{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
    pipelineLayoutInfo.setLayoutCount = 1;
    pipelineLayoutInfo.pSetLayouts = &highQualityGaussianSplatDescriptorSetLayout_;
    pipelineLayoutInfo.pushConstantRangeCount = 1;
    pipelineLayoutInfo.pPushConstantRanges = &pushConstantRange;

    Check(
        vkCreatePipelineLayout(
            device_,
            &pipelineLayoutInfo,
            nullptr,
            &highQualityGaussianSplatPipelineLayout_),
        "vkCreatePipelineLayout(gsplat_hq)");

    VkGraphicsPipelineCreateInfo pipelineInfo{VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO};
    pipelineInfo.stageCount = static_cast<std::uint32_t>(shaderStages.size());
    pipelineInfo.pStages = shaderStages.data();
    pipelineInfo.pVertexInputState = &vertexInputInfo;
    pipelineInfo.pInputAssemblyState = &inputAssembly;
    pipelineInfo.pViewportState = &viewportState;
    pipelineInfo.pRasterizationState = &rasterizer;
    pipelineInfo.pMultisampleState = &multisampling;
    pipelineInfo.pDepthStencilState = &depthStencil;
    pipelineInfo.pColorBlendState = &colorBlending;
    pipelineInfo.pDynamicState = &dynamicState;
    pipelineInfo.layout = highQualityGaussianSplatPipelineLayout_;
    pipelineInfo.renderPass = renderPass_;
    pipelineInfo.subpass = 3;

    Check(
        vkCreateGraphicsPipelines(
            device_,
            VK_NULL_HANDLE,
            1,
            &pipelineInfo,
            nullptr,
            &highQualityGaussianSplatPipeline_),
        "vkCreateGraphicsPipelines(gsplat_hq)");

    vkDestroyShaderModule(device_, fragmentModule, nullptr);
    vkDestroyShaderModule(device_, vertexModule, nullptr);
}

void VulkanViewportShell::CreateCompositePipeline() {
    const auto vertexShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "gsplat_composite.vert.spv").string());
    const auto fragmentShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "gsplat_composite.frag.spv").string());

    const auto vertexModule = CreateShaderModule(device_, vertexShaderCode, "vkCreateShaderModule(composite vertex)");
    const auto fragmentModule = CreateShaderModule(device_, fragmentShaderCode, "vkCreateShaderModule(composite fragment)");

    VkPipelineShaderStageCreateInfo vertexStage{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    vertexStage.stage = VK_SHADER_STAGE_VERTEX_BIT;
    vertexStage.module = vertexModule;
    vertexStage.pName = "main";

    VkPipelineShaderStageCreateInfo fragmentStage{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    fragmentStage.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
    fragmentStage.module = fragmentModule;
    fragmentStage.pName = "main";

    const std::array<VkPipelineShaderStageCreateInfo, 2> shaderStages = {vertexStage, fragmentStage};

    VkPipelineVertexInputStateCreateInfo vertexInputInfo{
        VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO};

    VkPipelineInputAssemblyStateCreateInfo inputAssembly{
        VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO};
    inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;

    VkPipelineViewportStateCreateInfo viewportState{VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO};
    viewportState.viewportCount = 1;
    viewportState.scissorCount = 1;

    VkPipelineRasterizationStateCreateInfo rasterizer{
        VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO};
    rasterizer.polygonMode = VK_POLYGON_MODE_FILL;
    rasterizer.cullMode = VK_CULL_MODE_NONE;
    rasterizer.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
    rasterizer.lineWidth = 1.0F;

    VkPipelineMultisampleStateCreateInfo multisampling{
        VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO};
    multisampling.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

    VkPipelineDepthStencilStateCreateInfo depthStencil{
        VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO};
    depthStencil.depthTestEnable = VK_FALSE;
    depthStencil.depthWriteEnable = VK_FALSE;

    const auto colorBlendAttachment = MakeAlphaBlendAttachment();
    VkPipelineColorBlendStateCreateInfo colorBlending{
        VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO};
    colorBlending.attachmentCount = 1;
    colorBlending.pAttachments = &colorBlendAttachment;

    const std::array<VkDynamicState, 2> dynamicStates = {VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR};
    VkPipelineDynamicStateCreateInfo dynamicState{VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO};
    dynamicState.dynamicStateCount = static_cast<std::uint32_t>(dynamicStates.size());
    dynamicState.pDynamicStates = dynamicStates.data();

    VkPipelineLayoutCreateInfo pipelineLayoutInfo{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
    pipelineLayoutInfo.setLayoutCount = 1;
    pipelineLayoutInfo.pSetLayouts = &compositeDescriptorSetLayout_;

    Check(vkCreatePipelineLayout(device_, &pipelineLayoutInfo, nullptr, &compositePipelineLayout_), "vkCreatePipelineLayout(composite)");

    VkGraphicsPipelineCreateInfo pipelineInfo{VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO};
    pipelineInfo.stageCount = static_cast<std::uint32_t>(shaderStages.size());
    pipelineInfo.pStages = shaderStages.data();
    pipelineInfo.pVertexInputState = &vertexInputInfo;
    pipelineInfo.pInputAssemblyState = &inputAssembly;
    pipelineInfo.pViewportState = &viewportState;
    pipelineInfo.pRasterizationState = &rasterizer;
    pipelineInfo.pMultisampleState = &multisampling;
    pipelineInfo.pDepthStencilState = &depthStencil;
    pipelineInfo.pColorBlendState = &colorBlending;
    pipelineInfo.pDynamicState = &dynamicState;
    pipelineInfo.layout = compositePipelineLayout_;
    pipelineInfo.renderPass = renderPass_;
    pipelineInfo.subpass = 2;

    Check(vkCreateGraphicsPipelines(device_, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &compositePipeline_), "vkCreateGraphicsPipelines(composite)");

    vkDestroyShaderModule(device_, fragmentModule, nullptr);

    // Sorted emission composite: same fullscreen geometry and descriptor set,
    // reading only the emission slot after the sorted-alpha subpass rewrote
    // it. Colour adds; alpha takes the maximum, mirroring the weighted
    // composite's max(alpha, emissionAlpha) coverage fold.
    const auto sortedEmissionFragmentShaderCode = ReadBinaryFile(
        (std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} /
         "pointcloud_sorted_emission_composite.frag.spv")
            .string());
    const auto sortedEmissionFragmentModule = CreateShaderModule(
        device_,
        sortedEmissionFragmentShaderCode,
        "vkCreateShaderModule(sorted emission composite fragment)");
    VkPipelineShaderStageCreateInfo sortedEmissionFragmentStage{
        VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    sortedEmissionFragmentStage.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
    sortedEmissionFragmentStage.module = sortedEmissionFragmentModule;
    sortedEmissionFragmentStage.pName = "main";
    const std::array<VkPipelineShaderStageCreateInfo, 2>
        sortedEmissionStages = {vertexStage, sortedEmissionFragmentStage};

    VkPipelineColorBlendAttachmentState sortedEmissionBlend{};
    sortedEmissionBlend.colorWriteMask =
        VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT |
        VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
    sortedEmissionBlend.blendEnable = VK_TRUE;
    sortedEmissionBlend.srcColorBlendFactor = VK_BLEND_FACTOR_ONE;
    sortedEmissionBlend.dstColorBlendFactor = VK_BLEND_FACTOR_ONE;
    sortedEmissionBlend.colorBlendOp = VK_BLEND_OP_ADD;
    sortedEmissionBlend.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
    sortedEmissionBlend.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
    sortedEmissionBlend.alphaBlendOp = VK_BLEND_OP_MAX;
    VkPipelineColorBlendStateCreateInfo sortedEmissionBlending{
        VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO};
    sortedEmissionBlending.attachmentCount = 1;
    sortedEmissionBlending.pAttachments = &sortedEmissionBlend;

    pipelineInfo.stageCount =
        static_cast<std::uint32_t>(sortedEmissionStages.size());
    pipelineInfo.pStages = sortedEmissionStages.data();
    pipelineInfo.pColorBlendState = &sortedEmissionBlending;
    pipelineInfo.subpass = 5;
    Check(
        vkCreateGraphicsPipelines(
            device_,
            VK_NULL_HANDLE,
            1,
            &pipelineInfo,
            nullptr,
            &pointSortedEmissionCompositePipeline_),
        "vkCreateGraphicsPipelines(sorted emission composite)");
    vkDestroyShaderModule(device_, sortedEmissionFragmentModule, nullptr);
    vkDestroyShaderModule(device_, vertexModule, nullptr);
}

void VulkanViewportShell::CreatePostProcessPipeline() {
    const auto vertexShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "gsplat_composite.vert.spv").string());
    const auto fragmentShaderCode =
        ReadBinaryFile((std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} / "edl_postprocess.frag.spv").string());

    const auto vertexModule =
        CreateShaderModule(device_, vertexShaderCode, "vkCreateShaderModule(postprocess vertex)");
    const auto fragmentModule =
        CreateShaderModule(device_, fragmentShaderCode, "vkCreateShaderModule(postprocess fragment)");

    VkPipelineShaderStageCreateInfo vertexStage{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    vertexStage.stage = VK_SHADER_STAGE_VERTEX_BIT;
    vertexStage.module = vertexModule;
    vertexStage.pName = "main";

    VkPipelineShaderStageCreateInfo fragmentStage{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    fragmentStage.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
    fragmentStage.module = fragmentModule;
    fragmentStage.pName = "main";

    const std::array<VkPipelineShaderStageCreateInfo, 2> shaderStages = {vertexStage, fragmentStage};

    VkPipelineVertexInputStateCreateInfo vertexInputInfo{
        VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO};
    VkPipelineInputAssemblyStateCreateInfo inputAssembly{
        VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO};
    inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;

    VkPipelineViewportStateCreateInfo viewportState{VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO};
    viewportState.viewportCount = 1;
    viewportState.scissorCount = 1;

    VkPipelineRasterizationStateCreateInfo rasterizer{
        VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO};
    rasterizer.polygonMode = VK_POLYGON_MODE_FILL;
    rasterizer.cullMode = VK_CULL_MODE_NONE;
    rasterizer.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
    rasterizer.lineWidth = 1.0F;

    VkPipelineMultisampleStateCreateInfo multisampling{
        VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO};
    multisampling.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

    VkPipelineDepthStencilStateCreateInfo depthStencil{
        VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO};
    depthStencil.depthTestEnable = VK_FALSE;
    depthStencil.depthWriteEnable = VK_FALSE;

    VkPipelineColorBlendAttachmentState colorBlendAttachment{};
    colorBlendAttachment.colorWriteMask =
        VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
    colorBlendAttachment.blendEnable = VK_FALSE;

    VkPipelineColorBlendStateCreateInfo colorBlending{
        VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO};
    colorBlending.attachmentCount = 1;
    colorBlending.pAttachments = &colorBlendAttachment;

    const std::array<VkDynamicState, 2> dynamicStates = {VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR};
    VkPipelineDynamicStateCreateInfo dynamicState{VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO};
    dynamicState.dynamicStateCount = static_cast<std::uint32_t>(dynamicStates.size());
    dynamicState.pDynamicStates = dynamicStates.data();

    VkPushConstantRange pushConstantRange{};
    pushConstantRange.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
    pushConstantRange.offset = 0;
    pushConstantRange.size = sizeof(PostProcessPushConstants);

    VkPipelineLayoutCreateInfo pipelineLayoutInfo{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
    pipelineLayoutInfo.setLayoutCount = 1;
    pipelineLayoutInfo.pSetLayouts = &postProcessDescriptorSetLayout_;
    pipelineLayoutInfo.pushConstantRangeCount = 1;
    pipelineLayoutInfo.pPushConstantRanges = &pushConstantRange;

    Check(
        vkCreatePipelineLayout(device_, &pipelineLayoutInfo, nullptr, &postProcessPipelineLayout_),
        "vkCreatePipelineLayout(postprocess)");

    VkGraphicsPipelineCreateInfo pipelineInfo{VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO};
    pipelineInfo.stageCount = static_cast<std::uint32_t>(shaderStages.size());
    pipelineInfo.pStages = shaderStages.data();
    pipelineInfo.pVertexInputState = &vertexInputInfo;
    pipelineInfo.pInputAssemblyState = &inputAssembly;
    pipelineInfo.pViewportState = &viewportState;
    pipelineInfo.pRasterizationState = &rasterizer;
    pipelineInfo.pMultisampleState = &multisampling;
    pipelineInfo.pDepthStencilState = &depthStencil;
    pipelineInfo.pColorBlendState = &colorBlending;
    pipelineInfo.pDynamicState = &dynamicState;
    pipelineInfo.layout = postProcessPipelineLayout_;
    pipelineInfo.renderPass = presentRenderPass_;
    pipelineInfo.subpass = 0;

    Check(
        vkCreateGraphicsPipelines(device_, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &postProcessPipeline_),
        "vkCreateGraphicsPipelines(postprocess)");

    vkDestroyShaderModule(device_, fragmentModule, nullptr);
    vkDestroyShaderModule(device_, vertexModule, nullptr);
}

void VulkanViewportShell::CreateEyeDomeLightingExportPipeline() {
    const auto shaderCode = ReadBinaryFile(
        (std::filesystem::path{INVISIBLE_PLACES_SHADER_OUTPUT_DIR} /
         "edl_export.comp.spv")
            .string());
    const auto shaderModule =
        CreateShaderModule(device_, shaderCode, "vkCreateShaderModule(export EDL compute)");

    VkPushConstantRange pushConstantRange{};
    pushConstantRange.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    pushConstantRange.offset = 0U;
    pushConstantRange.size = sizeof(EyeDomeLightingExportPushConstants);

    VkPipelineLayoutCreateInfo layoutInfo{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
    layoutInfo.setLayoutCount = 1U;
    layoutInfo.pSetLayouts = &eyeDomeLightingExportDescriptorSetLayout_;
    layoutInfo.pushConstantRangeCount = 1U;
    layoutInfo.pPushConstantRanges = &pushConstantRange;
    Check(
        vkCreatePipelineLayout(
            device_,
            &layoutInfo,
            nullptr,
            &eyeDomeLightingExportPipelineLayout_),
        "vkCreatePipelineLayout(export EDL)");

    VkPipelineShaderStageCreateInfo stageInfo{
        VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    stageInfo.stage = VK_SHADER_STAGE_COMPUTE_BIT;
    stageInfo.module = shaderModule;
    stageInfo.pName = "main";

    VkComputePipelineCreateInfo pipelineInfo{VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO};
    pipelineInfo.stage = stageInfo;
    pipelineInfo.layout = eyeDomeLightingExportPipelineLayout_;
    Check(
        vkCreateComputePipelines(
            device_,
            VK_NULL_HANDLE,
            1U,
            &pipelineInfo,
            nullptr,
            &eyeDomeLightingExportPipeline_),
        "vkCreateComputePipelines(export EDL)");

    vkDestroyShaderModule(device_, shaderModule, nullptr);
}

void VulkanViewportShell::CreateFramebuffers() {
    framebuffers_.clear();
    framebuffers_.reserve(sceneColorImages_.size());

    for (std::size_t imageIndex = 0; imageIndex < sceneColorImages_.size(); ++imageIndex) {
        const std::array<VkImageView, 6> attachments = {
            sceneColorImages_[imageIndex].view,
            depthImages_[imageIndex].view,
            accumulationImages_[imageIndex].view,
            revealageImages_[imageIndex].view,
            emissiveImages_[imageIndex].view,
            linearDepthImages_[imageIndex].view,
        };

        VkFramebufferCreateInfo framebufferInfo{VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO};
        framebufferInfo.renderPass = renderPass_;
        framebufferInfo.attachmentCount = static_cast<std::uint32_t>(attachments.size());
        framebufferInfo.pAttachments = attachments.data();
        framebufferInfo.width = swapchainWidth_;
        framebufferInfo.height = swapchainHeight_;
        framebufferInfo.layers = 1;

        VkFramebuffer framebuffer = VK_NULL_HANDLE;
        Check(vkCreateFramebuffer(device_, &framebufferInfo, nullptr, &framebuffer), "vkCreateFramebuffer");
        framebuffers_.push_back(framebuffer);
    }
}

void VulkanViewportShell::CreatePresentFramebuffers() {
    presentFramebuffers_.clear();
    presentFramebuffers_.reserve(imageViews_.size());

    for (const auto imageView : imageViews_) {
        VkFramebufferCreateInfo framebufferInfo{VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO};
        framebufferInfo.renderPass = presentRenderPass_;
        framebufferInfo.attachmentCount = 1;
        framebufferInfo.pAttachments = &imageView;
        framebufferInfo.width = swapchainWidth_;
        framebufferInfo.height = swapchainHeight_;
        framebufferInfo.layers = 1;

        VkFramebuffer framebuffer = VK_NULL_HANDLE;
        Check(vkCreateFramebuffer(device_, &framebufferInfo, nullptr, &framebuffer), "vkCreateFramebuffer(present)");
        presentFramebuffers_.push_back(framebuffer);
    }
}

void VulkanViewportShell::CreateSceneColorResources() {
    for (auto& image : sceneColorImages_) {
        DestroyImage(&image);
    }
    sceneColorImages_.clear();
    sceneImageRevisions_.clear();
    sceneColorImages_.reserve(swapchainImages_.size());
    sceneImageRevisions_.reserve(swapchainImages_.size());
    for (std::size_t imageIndex = 0; imageIndex < swapchainImages_.size(); ++imageIndex) {
        sceneColorImages_.push_back(CreateAttachmentImage(
            swapchainImageFormat_,
            VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT |
                VK_IMAGE_USAGE_TRANSFER_SRC_BIT,
            VK_IMAGE_ASPECT_COLOR_BIT));
        sceneImageRevisions_.push_back(0);
    }
}

void VulkanViewportShell::CreateDepthResources() {
    for (auto& image : depthImages_) {
        DestroyImage(&image);
    }
    depthImages_.clear();
    depthFormat_ = SelectDepthFormat();
    depthImages_.reserve(swapchainImages_.size());
    for (std::size_t imageIndex = 0; imageIndex < swapchainImages_.size(); ++imageIndex) {
        depthImages_.push_back(CreateAttachmentImage(
            depthFormat_,
            VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_INPUT_ATTACHMENT_BIT,
            VK_IMAGE_ASPECT_DEPTH_BIT));
    }
}

void VulkanViewportShell::CreateAccumulationResources() {
    for (auto& image : accumulationImages_) {
        DestroyImage(&image);
    }
    for (auto& image : revealageImages_) {
        DestroyImage(&image);
    }
    for (auto& image : emissiveImages_) {
        DestroyImage(&image);
    }
    accumulationImages_.clear();
    revealageImages_.clear();
    emissiveImages_.clear();

    accumulationImages_.reserve(swapchainImages_.size());
    revealageImages_.reserve(swapchainImages_.size());
    emissiveImages_.reserve(swapchainImages_.size());
    for (std::size_t imageIndex = 0; imageIndex < swapchainImages_.size(); ++imageIndex) {
        accumulationImages_.push_back(CreateAttachmentImage(
            accumulationFormat_,
            VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_INPUT_ATTACHMENT_BIT,
            VK_IMAGE_ASPECT_COLOR_BIT));
        revealageImages_.push_back(CreateAttachmentImage(
            revealageFormat_,
            VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_INPUT_ATTACHMENT_BIT,
            VK_IMAGE_ASPECT_COLOR_BIT));
        emissiveImages_.push_back(CreateAttachmentImage(
            accumulationFormat_,
            VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_INPUT_ATTACHMENT_BIT,
            VK_IMAGE_ASPECT_COLOR_BIT));
    }
}

void VulkanViewportShell::CreateLinearDepthResources() {
    for (auto& image : linearDepthImages_) {
        DestroyImage(&image);
    }
    for (auto& image : temporalCameraOverlayColorImages_) {
        DestroyImage(&image);
    }
    for (auto& image : temporalCameraOverlayDepthImages_) {
        DestroyImage(&image);
    }
    linearDepthImages_.clear();
    linearDepthImages_.reserve(swapchainImages_.size());
    for (std::size_t imageIndex = 0; imageIndex < swapchainImages_.size(); ++imageIndex) {
        linearDepthImages_.push_back(CreateAttachmentImage(
            linearDepthFormat_,
            VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT |
                VK_IMAGE_USAGE_TRANSFER_SRC_BIT,
            VK_IMAGE_ASPECT_COLOR_BIT));
    }
}

void VulkanViewportShell::CreateTemporalCameraOverlayResources() {
    for (auto& image : temporalCameraOverlayColorImages_) {
        DestroyImage(&image);
        image = CreateAttachmentImage(
            swapchainImageFormat_,
            VK_IMAGE_USAGE_TRANSFER_DST_BIT |
                VK_IMAGE_USAGE_SAMPLED_BIT,
            VK_IMAGE_ASPECT_COLOR_BIT);
    }
    for (auto& image : temporalCameraOverlayDepthImages_) {
        DestroyImage(&image);
        image = CreateAttachmentImage(
            linearDepthFormat_,
            VK_IMAGE_USAGE_TRANSFER_DST_BIT |
                VK_IMAGE_USAGE_SAMPLED_BIT,
            VK_IMAGE_ASPECT_COLOR_BIT);
    }
    temporalCameraOverlayResourcesInitialized_ = false;
    temporalCameraOverlayHistoryValid_ = {false, false};
}

void VulkanViewportShell::CreateCommandPool() {
    VkCommandPoolCreateInfo poolInfo{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
    poolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    poolInfo.queueFamilyIndex = graphicsQueueFamily_;

    Check(vkCreateCommandPool(device_, &poolInfo, nullptr, &commandPool_), "vkCreateCommandPool");
}

void VulkanViewportShell::CreateCommandBuffers() {
    for (auto& frame : frameResources_) {
        if (frame.commandBuffer != VK_NULL_HANDLE) {
            vkFreeCommandBuffers(device_, commandPool_, 1, &frame.commandBuffer);
            frame.commandBuffer = VK_NULL_HANDLE;
        }
    }

    std::array<VkCommandBuffer, kFramesInFlight> commandBuffers{};

    VkCommandBufferAllocateInfo allocInfo{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
    allocInfo.commandPool = commandPool_;
    allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    allocInfo.commandBufferCount = static_cast<std::uint32_t>(commandBuffers.size());

    Check(vkAllocateCommandBuffers(device_, &allocInfo, commandBuffers.data()), "vkAllocateCommandBuffers");
    for (std::size_t frameIndex = 0; frameIndex < kFramesInFlight; ++frameIndex) {
        frameResources_[frameIndex].commandBuffer = commandBuffers[frameIndex];
    }
}

void VulkanViewportShell::CreateSyncObjects() {
    VkSemaphoreCreateInfo semaphoreInfo{VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO};
    VkFenceCreateInfo fenceInfo{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};
    fenceInfo.flags = VK_FENCE_CREATE_SIGNALED_BIT;

    for (auto& frame : frameResources_) {
        Check(
            vkCreateSemaphore(device_, &semaphoreInfo, nullptr, &frame.imageAvailableSemaphore),
            "vkCreateSemaphore(imageAvailable)");
        Check(vkCreateFence(device_, &fenceInfo, nullptr, &frame.fence), "vkCreateFence");
    }

    if (!gpuTimestampsSupported_) {
        return;
    }

    VkQueryPoolCreateInfo queryPoolInfo{
        VK_STRUCTURE_TYPE_QUERY_POOL_CREATE_INFO};
    queryPoolInfo.queryType = VK_QUERY_TYPE_TIMESTAMP;
    queryPoolInfo.queryCount = kFrameTimestampQueryCount;
    for (auto& frame : frameResources_) {
        const VkResult queryResult = vkCreateQueryPool(
            device_,
            &queryPoolInfo,
            nullptr,
            &frame.timestampQueryPool);
        if (queryResult == VK_SUCCESS) {
            continue;
        }
        for (auto& createdFrame : frameResources_) {
            if (createdFrame.timestampQueryPool != VK_NULL_HANDLE) {
                vkDestroyQueryPool(
                    device_,
                    createdFrame.timestampQueryPool,
                    nullptr);
                createdFrame.timestampQueryPool = VK_NULL_HANDLE;
            }
        }
        gpuTimestampsSupported_ = false;
        diagnostics_.gpuTimestampsSupported = false;
        break;
    }
}

void VulkanViewportShell::CreateSwapchainRenderFinishedSemaphores() {
    DestroySwapchainRenderFinishedSemaphores();
    swapchainRenderFinishedSemaphores_.resize(
        swapchainImages_.size(),
        VK_NULL_HANDLE);
    VkSemaphoreCreateInfo semaphoreInfo{
        VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO};
    for (auto& semaphore : swapchainRenderFinishedSemaphores_) {
        Check(
            vkCreateSemaphore(device_, &semaphoreInfo, nullptr, &semaphore),
            "vkCreateSemaphore(swapchain render finished)");
    }
}

void VulkanViewportShell::DestroySwapchainRenderFinishedSemaphores() {
    for (auto& semaphore : swapchainRenderFinishedSemaphores_) {
        if (semaphore != VK_NULL_HANDLE && device_ != VK_NULL_HANDLE) {
            vkDestroySemaphore(device_, semaphore, nullptr);
            semaphore = VK_NULL_HANDLE;
        }
    }
    swapchainRenderFinishedSemaphores_.clear();
}

void VulkanViewportShell::ReadCompletedFrameTimestamps(
    std::size_t frameIndex) {
    if (frameIndex >= frameResources_.size()) {
        return;
    }
    auto& frame = frameResources_[frameIndex];
    diagnostics_.completedFrameSlot =
        static_cast<std::uint32_t>(frameIndex);
    diagnostics_.completedImageIndex = frame.submittedImageIndex;
    diagnostics_.completedSubmissionSerial =
        frame.completedSubmissionSerial;
    diagnostics_.gpuTimestampResultsAvailable = false;
    if (!gpuTimestampsSupported_ || !frame.timestampPending ||
        frame.timestampQueryPool == VK_NULL_HANDLE) {
        frame.timestampPending = false;
        return;
    }

    std::array<TimestampQueryResult, kFrameTimestampQueryCount> results{};
    const VkResult result = vkGetQueryPoolResults(
        device_,
        frame.timestampQueryPool,
        0U,
        kFrameTimestampQueryCount,
        sizeof(results),
        results.data(),
        sizeof(TimestampQueryResult),
        VK_QUERY_RESULT_64_BIT | VK_QUERY_RESULT_WITH_AVAILABILITY_BIT);
    frame.timestampPending = false;
    if (result != VK_SUCCESS ||
        !TimestampResultsAvailable(results)) {
        ++diagnostics_.gpuTimestampUnavailableResultCount;
        return;
    }

    const auto elapsed = [&](std::uint32_t start, std::uint32_t end) {
        return TimestampDeltaMilliseconds(
            results[start].value,
            results[end].value,
            gpuTimestampValidBits_,
            gpuTimestampPeriodNanoseconds_);
    };
    const auto setPhase = [](
                              GpuPhaseDiagnostics* phase,
                              double milliseconds,
                              bool active) {
        phase->milliseconds = active ? milliseconds : 0.0;
        phase->active = active;
    };
    const double totalMilliseconds =
        elapsed(kTimestampFrameStart, kTimestampFrameEnd);
    std::array<double, 7> phaseMilliseconds = {
        elapsed(kTimestampFrameStart, kTimestampRainEnd),
        elapsed(kTimestampRainEnd, kTimestampDepthEnd),
        elapsed(kTimestampDepthEnd, kTimestampWeightedEnd),
        elapsed(kTimestampWeightedEnd, kTimestampCompositeEnd),
        elapsed(kTimestampCompositeEnd, kTimestampOpaqueEnd),
        elapsed(kTimestampOpaqueEnd, kTimestampPostProcessEnd),
        // Include render-pass completion in the final active band so the
        // published phase sum reconciles exactly with total GPU time.
        elapsed(kTimestampPostProcessEnd, kTimestampImGuiEnd) +
            elapsed(kTimestampImGuiEnd, kTimestampFrameEnd),
    };
    const std::array<bool, 7> phaseActive = {
        (frame.timestampActiveMask & kGpuPhaseRainActive) != 0U,
        (frame.timestampActiveMask & kGpuPhaseDepthActive) != 0U,
        (frame.timestampActiveMask & kGpuPhaseWeightedActive) != 0U,
        (frame.timestampActiveMask & kGpuPhaseCompositeActive) != 0U,
        (frame.timestampActiveMask & kGpuPhaseOpaqueActive) != 0U,
        (frame.timestampActiveMask & kGpuPhasePostProcessActive) != 0U,
        (frame.timestampActiveMask & kGpuPhaseImGuiActive) != 0U,
    };
    // Empty phases still contain render-pass/subpass transition cost. Keep
    // them visibly inactive, but attribute that overhead to the nearest active
    // band so active phase totals remain trustworthy and reconcilable.
    for (std::size_t index = 0U; index < phaseActive.size(); ++index) {
        if (phaseActive[index] || phaseMilliseconds[index] == 0.0) {
            continue;
        }
        std::size_t target = phaseActive.size();
        for (std::size_t candidate = index + 1U;
             candidate < phaseActive.size();
             ++candidate) {
            if (phaseActive[candidate]) {
                target = candidate;
                break;
            }
        }
        if (target == phaseActive.size()) {
            for (std::size_t candidate = index; candidate > 0U;
                 --candidate) {
                if (phaseActive[candidate - 1U]) {
                    target = candidate - 1U;
                    break;
                }
            }
        }
        if (target < phaseActive.size()) {
            phaseMilliseconds[target] += phaseMilliseconds[index];
        }
        phaseMilliseconds[index] = 0.0;
    }
    const std::array<GpuPhaseDiagnostics*, 7> phaseOutputs = {
        &diagnostics_.gpuRainCompute,
        &diagnostics_.gpuDepth,
        &diagnostics_.gpuWeightedAccumulation,
        &diagnostics_.gpuComposite,
        &diagnostics_.gpuOpaqueAndHighQuality,
        &diagnostics_.gpuPostProcess,
        &diagnostics_.gpuImGui,
    };
    setPhase(&diagnostics_.gpuTotal, totalMilliseconds, true);
    for (std::size_t index = 0U; index < phaseOutputs.size(); ++index) {
        setPhase(
            phaseOutputs[index],
            phaseMilliseconds[index],
            phaseActive[index]);
    }
    diagnostics_.gpuTimestampResultsAvailable = true;
    diagnostics_.gpuResultLatencyFrames = static_cast<std::uint32_t>(
        std::min<std::uint64_t>(
            kFramesInFlight,
            nextSubmissionSerial_ > frame.completedSubmissionSerial
                ? nextSubmissionSerial_ -
                      frame.completedSubmissionSerial - 1U
                : 0U));
    UpdateGpuTimingAverages();
}

void VulkanViewportShell::UpdateCpuTimingAverages() {
    const std::array<double, 12> milliseconds = {
        diagnostics_.frameUiRenderMs,
        diagnostics_.frameFenceWaitMs,
        diagnostics_.frameCompletedMaintenanceMs,
        diagnostics_.frameHighQualityGaussianPrepareMs,
        diagnostics_.frameUniformResourceUploadMs,
        diagnostics_.frameAcquireMs,
        diagnostics_.frameImageWaitMs,
        diagnostics_.frameResetRetirePollingMs,
        diagnostics_.frameCommandBufferMs,
        diagnostics_.frameSubmitMs,
        diagnostics_.framePresentMs,
        diagnostics_.framePlatformWindowsMs,
    };
    std::array<bool, 12> active{};
    active.fill(true);
    cpuTimingWindow_.AddFrame(
        milliseconds,
        active,
        diagnostics_.frameRenderMs);
    if (!cpuTimingWindow_.PublishIfReady(500.0)) {
        return;
    }
    const std::array<double*, 12> averages = {
        &diagnostics_.averageFrameUiRenderMs,
        &diagnostics_.averageFrameFenceWaitMs,
        &diagnostics_.averageFrameCompletedMaintenanceMs,
        &diagnostics_.averageFrameHighQualityGaussianPrepareMs,
        &diagnostics_.averageFrameUniformResourceUploadMs,
        &diagnostics_.averageFrameAcquireMs,
        &diagnostics_.averageFrameImageWaitMs,
        &diagnostics_.averageFrameResetRetirePollingMs,
        &diagnostics_.averageFrameCommandBufferMs,
        &diagnostics_.averageFrameSubmitMs,
        &diagnostics_.averageFramePresentMs,
        &diagnostics_.averageFramePlatformWindowsMs,
    };
    for (std::size_t index = 0U; index < averages.size(); ++index) {
        *averages[index] = cpuTimingWindow_.PublishedAverage(index);
    }
}

void VulkanViewportShell::UpdateGpuTimingAverages() {
    const std::array<GpuPhaseDiagnostics*, 8> phases = {
        &diagnostics_.gpuTotal,
        &diagnostics_.gpuRainCompute,
        &diagnostics_.gpuDepth,
        &diagnostics_.gpuWeightedAccumulation,
        &diagnostics_.gpuComposite,
        &diagnostics_.gpuOpaqueAndHighQuality,
        &diagnostics_.gpuPostProcess,
        &diagnostics_.gpuImGui,
    };
    std::array<double, 8> milliseconds{};
    std::array<bool, 8> active{};
    for (std::size_t index = 0U; index < phases.size(); ++index) {
        milliseconds[index] = phases[index]->milliseconds;
        active[index] = phases[index]->active;
    }
    const auto sampleTime = std::chrono::steady_clock::now();
    const double elapsedMilliseconds =
        lastGpuTimingSampleAt_ == std::chrono::steady_clock::time_point{}
            ? diagnostics_.gpuTotal.milliseconds
            : std::chrono::duration<double, std::milli>(
                  sampleTime - lastGpuTimingSampleAt_)
                  .count();
    lastGpuTimingSampleAt_ = sampleTime;
    gpuTimingWindow_.AddFrame(
        milliseconds,
        active,
        elapsedMilliseconds);
    if (!gpuTimingWindow_.PublishIfReady(500.0)) {
        return;
    }
    for (std::size_t index = 0U; index < phases.size(); ++index) {
        if (gpuTimingWindow_.PublishedActive(index)) {
            phases[index]->averageMilliseconds =
                gpuTimingWindow_.PublishedAverage(index);
        }
    }
}

void VulkanViewportShell::CreateImGuiResources() {
    const std::array<VkDescriptorPoolSize, 11> poolSizes = {
        MakePoolSize(VK_DESCRIPTOR_TYPE_SAMPLER, 1000),
        MakePoolSize(VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1000),
        MakePoolSize(VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, 1000),
        MakePoolSize(VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1000),
        MakePoolSize(VK_DESCRIPTOR_TYPE_UNIFORM_TEXEL_BUFFER, 1000),
        MakePoolSize(VK_DESCRIPTOR_TYPE_STORAGE_TEXEL_BUFFER, 1000),
        MakePoolSize(VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1000),
        MakePoolSize(VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1000),
        MakePoolSize(VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER_DYNAMIC, 1000),
        MakePoolSize(VK_DESCRIPTOR_TYPE_STORAGE_BUFFER_DYNAMIC, 1000),
        MakePoolSize(VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT, 1000),
    };

    VkDescriptorPoolCreateInfo poolInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
    poolInfo.flags = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;
    poolInfo.maxSets = 1000 * static_cast<std::uint32_t>(poolSizes.size());
    poolInfo.poolSizeCount = static_cast<std::uint32_t>(poolSizes.size());
    poolInfo.pPoolSizes = poolSizes.data();
    Check(vkCreateDescriptorPool(device_, &poolInfo, nullptr, &imguiDescriptorPool_), "vkCreateDescriptorPool(imgui)");

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ApplyImGuiStyle();

    auto& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;
    io.ConfigViewportsNoDecoration = false;

    // Keep a real italic face in the shared atlas for regenerative Water
    // setting labels. Loading is conditional so packaged/headless builds
    // retain the ordinary default font when a platform font is unavailable.
    // Application code locates this face by its stable debug name.
    io.FontDefault = io.Fonts->AddFontDefault();
    constexpr std::string_view kWaterItalicFontName =
        "Water Settings Italic";
#if defined(__APPLE__)
    constexpr std::array<std::string_view, 2> kItalicFontCandidates{
        "/System/Library/Fonts/Supplemental/Arial Italic.ttf",
        "/System/Library/Fonts/Supplemental/Helvetica Oblique.ttf",
    };
#elif defined(_WIN32)
    constexpr std::array<std::string_view, 2> kItalicFontCandidates{
        "C:/Windows/Fonts/segoeuii.ttf",
        "C:/Windows/Fonts/ariali.ttf",
    };
#else
    constexpr std::array<std::string_view, 2> kItalicFontCandidates{
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Oblique.ttf",
        "/usr/share/fonts/truetype/liberation2/LiberationSans-Italic.ttf",
    };
#endif
    for (const auto candidate : kItalicFontCandidates) {
        std::error_code fontError;
        if (!std::filesystem::is_regular_file(candidate, fontError)) {
            continue;
        }
        ImFontConfig italicConfig;
        std::strncpy(
            italicConfig.Name,
            kWaterItalicFontName.data(),
            sizeof(italicConfig.Name) - 1U);
        italicConfig.Name[sizeof(italicConfig.Name) - 1U] = '\0';
        if (io.Fonts->AddFontFromFileTTF(
                std::string{candidate}.c_str(),
                13.0F,
                &italicConfig) != nullptr) {
            break;
        }
    }

    ImGui_ImplGlfw_InitForVulkan(window_, true);

    ImGui_ImplVulkan_InitInfo initInfo{};
    initInfo.Instance = instance_;
    initInfo.PhysicalDevice = physicalDevice_;
    initInfo.Device = device_;
    initInfo.QueueFamily = graphicsQueueFamily_;
    initInfo.Queue = graphicsQueue_;
    initInfo.DescriptorPool = imguiDescriptorPool_;
    initInfo.MinImageCount = std::max<std::uint32_t>(2U, static_cast<std::uint32_t>(swapchainImages_.size()));
    initInfo.ImageCount = static_cast<std::uint32_t>(swapchainImages_.size());
    initInfo.PipelineInfoMain.RenderPass = presentRenderPass_;
    initInfo.PipelineInfoMain.Subpass = 0;
    initInfo.PipelineInfoMain.MSAASamples = VK_SAMPLE_COUNT_1_BIT;
    initInfo.CheckVkResultFn = &CheckImGuiResult;

    if (!ImGui_ImplVulkan_Init(&initInfo)) {
        throw std::runtime_error{"ImGui Vulkan backend initialization failed."};
    }
}

void VulkanViewportShell::UploadImGuiFonts() {}

VkDescriptorPool VulkanViewportShell::CreatePointDescriptorPool(std::size_t highlightCount, bool includeExrDescriptor) {
    const std::uint64_t layerCount = 1U + static_cast<std::uint64_t>(highlightCount);
    const std::uint64_t liveSetCount =
        layerCount * static_cast<std::uint64_t>(kFramesInFlight) * static_cast<std::uint64_t>(depthImages_.size());
    const std::uint64_t exrSetCount = includeExrDescriptor ? 1U : 0U;
    const std::uint64_t setCount64 = std::max<std::uint64_t>(1U, liveSetCount + exrSetCount);
    if (setCount64 > std::numeric_limits<std::uint32_t>::max() / 13U) {
        throw std::runtime_error{"Point descriptor generation exceeds Vulkan pool limits."};
    }
    const auto setCount = static_cast<std::uint32_t>(setCount64);
    const std::array<VkDescriptorPoolSize, 3> poolSizes = {
        MakePoolSize(VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, setCount * 2U),
        MakePoolSize(VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, setCount * 13U),
        MakePoolSize(VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT, setCount),
    };
    VkDescriptorPoolCreateInfo poolInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
    poolInfo.maxSets = setCount;
    poolInfo.poolSizeCount = static_cast<std::uint32_t>(poolSizes.size());
    poolInfo.pPoolSizes = poolSizes.data();
    VkDescriptorPool descriptorPool = VK_NULL_HANDLE;
    Check(vkCreateDescriptorPool(device_, &poolInfo, nullptr, &descriptorPool),
          "vkCreateDescriptorPool(point generation)");
    ++diagnostics_.pointDescriptorGenerationsAllocated;
    return descriptorPool;
}

void VulkanViewportShell::DestroyPointDescriptorPool(VkDescriptorPool* descriptorPool) {
    if (descriptorPool == nullptr || *descriptorPool == VK_NULL_HANDLE) {
        return;
    }
    vkDestroyDescriptorPool(device_, *descriptorPool, nullptr);
    *descriptorPool = VK_NULL_HANDLE;
    ++diagnostics_.pointDescriptorGenerationsRetired;
}

void VulkanViewportShell::UpdatePointCloudDescriptorSets(ActivePointCloudResources* resources,
                                                         VkDescriptorPool allocationPool) {
    if (resources == nullptr) {
        return;
    }
    for (std::size_t frameIndex = 0; frameIndex < kFramesInFlight; ++frameIndex) {
        resources->descriptorSets[frameIndex].resize(depthImages_.size(), VK_NULL_HANDLE);
        resources->seepageDescriptorGenerations[frameIndex].resize(depthImages_.size(), 0U);
        for (std::uint32_t imageIndex = 0; imageIndex < depthImages_.size(); ++imageIndex) {
            UpdatePointCloudDescriptorSet(resources, frameIndex, imageIndex, depthImages_[imageIndex].view,
                                          allocationPool);
            resources->seepageDescriptorGenerations[frameIndex][imageIndex] = resources->seepageTopologyGeneration;
        }
    }
}

void VulkanViewportShell::RetirePointCloudDescriptorSets(
    std::array<std::vector<VkDescriptorSet>, kFramesInFlight>* descriptorSets) {
    if (descriptorSets == nullptr) {
        return;
    }
    // Handles are cleared before the generation-owned pool is destroyed. The
    // caller has already settled and reset every command buffer that could
    // retain MoltenVK's resolved descriptor objects.
    for (auto& frameDescriptorSets : *descriptorSets) {
        std::fill(frameDescriptorSets.begin(), frameDescriptorSets.end(), VK_NULL_HANDLE);
        frameDescriptorSets.clear();
    }
}

void VulkanViewportShell::ResetPointCloudReferencingCommandBuffers() {
    if (device_ == VK_NULL_HANDLE || commandPool_ == VK_NULL_HANDLE) {
        return;
    }
    // MoltenVK retains resolved descriptor objects in executable command
    // buffers. Even after device idle, retire those recordings before freeing
    // a descriptor generation or its bound buffers.
    for (auto& frame : frameResources_) {
        if (frame.commandBuffer != VK_NULL_HANDLE) {
            Check(vkResetCommandBuffer(frame.commandBuffer, 0U), "vkResetCommandBuffer(point descriptor retirement)");
        }
    }
    if (exrExportResources_.commandBuffer != VK_NULL_HANDLE) {
        Check(vkResetCommandBuffer(exrExportResources_.commandBuffer, 0U),
              "vkResetCommandBuffer(point EXR descriptor retirement)");
    }
}

void VulkanViewportShell::ReplacePointCloudDescriptorSets(ActivePointCloudResources* resources) {
    if (resources == nullptr) {
        return;
    }

    auto pending = BuildPointDescriptorGeneration(
        resources,
        exrExportResources_.depthImage.view,
        exrExportResources_.uniformBuffer.buffer);
    try {
        SettlePointCloudMutation();
    } catch (...) {
        DestroyPointDescriptorGeneration(&pending);
        throw;
    }
    InstallPointDescriptorGeneration(resources, &pending);
}

VulkanViewportShell::PendingPointDescriptorGeneration
VulkanViewportShell::BuildPointDescriptorGeneration(ActivePointCloudResources* resources, VkImageView exrDepthView,
                                                    VkBuffer exrFrameUniformBuffer) {
    if (resources == nullptr) {
        throw std::runtime_error{"Cannot build descriptors for a null point cloud."};
    }

    PendingPointDescriptorGeneration pending;
    pending.highlightDescriptorSets.resize(resources->highlights.size());
    pending.highlightSeepageDescriptorGenerations.resize(resources->highlights.size());
    pending.descriptorPool = CreatePointDescriptorPool(resources->highlights.size(), exrDepthView != VK_NULL_HANDLE);

    auto settledBase = std::move(resources->descriptorSets);
    auto settledBaseGenerations = std::move(resources->seepageDescriptorGenerations);
    const auto settledExr = resources->exrDescriptorSet;
    const auto settledExrGeneration = resources->seepageExrDescriptorGeneration;
    std::vector<std::array<std::vector<VkDescriptorSet>, kFramesInFlight>> settledHighlights(
        resources->highlights.size());
    std::vector<std::array<std::vector<std::uint64_t>, kFramesInFlight>> settledHighlightGenerations(
        resources->highlights.size());
    for (std::size_t index = 0U; index < resources->highlights.size(); ++index) {
        settledHighlights[index] = std::move(resources->highlights[index].descriptorSets);
        settledHighlightGenerations[index] = std::move(resources->highlights[index].seepageDescriptorGenerations);
    }
    resources->descriptorSets = {};
    resources->seepageDescriptorGenerations = {};
    resources->exrDescriptorSet = VK_NULL_HANDLE;
    resources->seepageExrDescriptorGeneration = 0U;

    const auto restoreSettled = [&]() {
        resources->descriptorSets = std::move(settledBase);
        resources->seepageDescriptorGenerations = std::move(settledBaseGenerations);
        resources->exrDescriptorSet = settledExr;
        resources->seepageExrDescriptorGeneration = settledExrGeneration;
        for (std::size_t index = 0U; index < resources->highlights.size(); ++index) {
            resources->highlights[index].descriptorSets = std::move(settledHighlights[index]);
            resources->highlights[index].seepageDescriptorGenerations = std::move(settledHighlightGenerations[index]);
        }
    };

    try {
        UpdatePointCloudDescriptorSets(resources, pending.descriptorPool);
        pending.descriptorSets = std::move(resources->descriptorSets);
        pending.seepageDescriptorGenerations = std::move(resources->seepageDescriptorGenerations);
        for (std::size_t index = 0U; index < resources->highlights.size(); ++index) {
            auto& highlight = resources->highlights[index];
            UpdatePointHighlightDescriptorSets(resources, &highlight, pending.descriptorPool);
            pending.highlightDescriptorSets[index] = std::move(highlight.descriptorSets);
            pending.highlightSeepageDescriptorGenerations[index] = std::move(highlight.seepageDescriptorGenerations);
        }
        if (exrDepthView != VK_NULL_HANDLE) {
            UpdatePointCloudExrDescriptorSet(
                resources, exrDepthView, exrFrameUniformBuffer, pending.descriptorPool);
            pending.exrDescriptorSet = resources->exrDescriptorSet;
            pending.seepageExrDescriptorGeneration = resources->seepageExrDescriptorGeneration;
        }
    } catch (...) {
        resources->descriptorSets = {};
        resources->seepageDescriptorGenerations = {};
        resources->exrDescriptorSet = VK_NULL_HANDLE;
        resources->seepageExrDescriptorGeneration = 0U;
        for (auto& highlight : resources->highlights) {
            highlight.descriptorSets = {};
            highlight.seepageDescriptorGenerations = {};
        }
        restoreSettled();
        DestroyPointDescriptorGeneration(&pending);
        throw;
    }

    resources->descriptorSets = {};
    resources->seepageDescriptorGenerations = {};
    resources->exrDescriptorSet = VK_NULL_HANDLE;
    resources->seepageExrDescriptorGeneration = 0U;
    for (auto& highlight : resources->highlights) {
        highlight.descriptorSets = {};
        highlight.seepageDescriptorGenerations = {};
    }
    restoreSettled();
    return pending;
}

void VulkanViewportShell::InstallPointDescriptorGeneration(ActivePointCloudResources* resources,
                                                           PendingPointDescriptorGeneration* generation) {
    if (resources == nullptr || generation == nullptr || generation->descriptorPool == VK_NULL_HANDLE) {
        throw std::runtime_error{"Cannot install an incomplete point descriptor generation."};
    }
    if (generation->highlightDescriptorSets.size() != resources->highlights.size() ||
        generation->highlightSeepageDescriptorGenerations.size() != resources->highlights.size()) {
        throw std::runtime_error{"Point descriptor highlight cardinality changed before install."};
    }

    // Every operation after validation is a no-throw handle/container swap.
    // This matters for the global EXR transaction: once the device is settled,
    // an allocator failure must no longer be able to produce a partial commit.
    std::swap(resources->pointDescriptorPool, generation->descriptorPool);
    resources->descriptorSets.swap(generation->descriptorSets);
    resources->seepageDescriptorGenerations.swap(generation->seepageDescriptorGenerations);
    for (std::size_t index = 0U; index < resources->highlights.size(); ++index) {
        resources->highlights[index].descriptorSets.swap(generation->highlightDescriptorSets[index]);
        resources->highlights[index].seepageDescriptorGenerations.swap(
            generation->highlightSeepageDescriptorGenerations[index]);
    }
    std::swap(resources->exrDescriptorSet, generation->exrDescriptorSet);
    std::swap(resources->seepageExrDescriptorGeneration, generation->seepageExrDescriptorGeneration);

    ++resources->pointDescriptorGeneration;
    if (resources->pointDescriptorGeneration == 0U) {
        resources->pointDescriptorGeneration = 1U;
    }
    DestroyPointDescriptorGeneration(generation);
}

void VulkanViewportShell::DestroyPointDescriptorGeneration(PendingPointDescriptorGeneration* generation) {
    if (generation == nullptr) {
        return;
    }
    RetirePointCloudDescriptorSets(&generation->descriptorSets);
    for (auto& descriptorSets : generation->highlightDescriptorSets) {
        RetirePointCloudDescriptorSets(&descriptorSets);
    }
    generation->exrDescriptorSet = VK_NULL_HANDLE;
    DestroyPointDescriptorPool(&generation->descriptorPool);
    *generation = {};
}

DynamicMeshFlowContactGpuView
VulkanViewportShell::PointDescriptorDynamicMeshFlowContactView(
    const ActivePointCloudResources* targetResources) const {
    if (pointDescriptorDynamicMeshFlowContactOverride_.has_value()) {
        return pointDescriptorDynamicMeshFlowContactOverride_.value();
    }
    const auto ownsInstalledGround =
        [this](const ActivePointCloudResources& resources) {
            return rainResources_.collisionReady &&
                   resources.dynamicMeshFlowGroundUploadRevision != 0U &&
                   resources.dynamicMeshFlowGroundUploadRevision ==
                       rainResources_.groundResidentUploadRevision &&
                   resources.dynamicMeshFlowGroundIdentity ==
                       rainResources_.collisionCacheIdentity;
        };
    const auto makeView =
        [&ownsInstalledGround](const ActivePointCloudResources& resources) {
            return DynamicMeshFlowContactGpuView{
                .eventBuffer =
                    resources.dynamicMeshFlowContactEventBuffer.buffer,
                .eventOffset = 0U,
                .eventRange =
                    resources.dynamicMeshFlowContactEventBuffer.size,
                .gridBuffer =
                    resources.dynamicMeshFlowContactGridBuffer.buffer,
                .gridOffset = 0U,
                .gridRange =
                    resources.dynamicMeshFlowContactGridBuffer.size,
                .eventCapacity =
                    resources.dynamicMeshFlowEventCapacity,
                .gridMask =
                    resources.dynamicMeshFlowContactGridMask,
                .gridCellSizeMeters =
                    resources.dynamicMeshFlowContactGridCellSizeMeters,
                .allocationRevision =
                    resources.dynamicMeshFlowAllocationRevision,
                .descriptorGeneration =
                    resources.dynamicMeshFlowDescriptorGeneration,
                .valid =
                    resources.dynamicMeshFlowAllocationRevision != 0U &&
                    resources.dynamicMeshFlowDescriptorGeneration != 0U &&
                    ownsInstalledGround(resources) &&
                    resources.dynamicMeshFlowContactEventBuffer.buffer !=
                        VK_NULL_HANDLE &&
                    resources.dynamicMeshFlowContactGridBuffer.buffer !=
                        VK_NULL_HANDLE,
            };
        };
    if (targetResources != nullptr &&
        targetResources->dynamicMeshFlowAllocationRevision != 0U) {
        const auto targetView = makeView(*targetResources);
        if (targetView.valid) {
            return targetView;
        }
    }
    for (const auto& resources : pointCloudResources_) {
        const auto activeView = makeView(resources);
        if (activeView.valid) {
            return activeView;
        }
    }
    return {
        .eventBuffer =
            rainResources_.dynamicMeshFlowDummyContactEventBuffer.buffer,
        .eventOffset = 0U,
        .eventRange =
            rainResources_.dynamicMeshFlowDummyContactEventBuffer.size,
        .gridBuffer =
            rainResources_.dynamicMeshFlowDummyContactGridBuffer.buffer,
        .gridOffset = 0U,
        .gridRange =
            rainResources_.dynamicMeshFlowDummyContactGridBuffer.size,
        .eventCapacity = 1U,
        .gridMask = 0U,
        .gridCellSizeMeters = 0.75F,
        .valid = false,
    };
}

void VulkanViewportShell::UpdatePointCloudDescriptorSet(ActivePointCloudResources* resources, std::size_t frameIndex,
                                                        std::uint32_t imageIndex, VkImageView sceneDepthView,
                                                        VkDescriptorPool allocationPool) {
    if (resources == nullptr || frameIndex >= kFramesInFlight || imageIndex >= depthImages_.size()) {
        return;
    }

    auto& descriptorSet = resources->descriptorSets[frameIndex][imageIndex];
    if (descriptorSet == VK_NULL_HANDLE) {
        const auto descriptorPool = allocationPool != VK_NULL_HANDLE
                                        ? allocationPool
                                        : resources->pointDescriptorPool;
        if (descriptorPool == VK_NULL_HANDLE) {
            throw std::runtime_error{"Point descriptor generation has no allocation pool."};
        }
        VkDescriptorSetAllocateInfo allocInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
        allocInfo.descriptorPool = descriptorPool;
        allocInfo.descriptorSetCount = 1;
        allocInfo.pSetLayouts = &pointDescriptorSetLayout_;
        Check(
            vkAllocateDescriptorSets(device_, &allocInfo, &descriptorSet),
            "vkAllocateDescriptorSets(point)");
    }

    VkDescriptorBufferInfo uniformInfo{};
    uniformInfo.buffer = frameResources_[frameIndex].uniformBuffer.buffer;
    uniformInfo.offset = 0;
    uniformInfo.range = sizeof(FrameUniforms);

    VkDescriptorBufferInfo scalarInfo{};
    scalarInfo.buffer = resources->scalarFieldBuffer.buffer;
    scalarInfo.offset = 0;
    scalarInfo.range = resources->scalarFieldBuffer.size;

    VkDescriptorBufferInfo styleInfo{};
    styleInfo.buffer = resources->styleBuffers[frameIndex].buffer;
    styleInfo.offset = 0;
    styleInfo.range = sizeof(PointCloudStyleGpu);

    VkDescriptorBufferInfo positionStorageInfo{};
    positionStorageInfo.buffer = resources->positionStorageBuffer.buffer;
    positionStorageInfo.offset = 0;
    positionStorageInfo.range = resources->positionStorageBuffer.size;

    VkDescriptorBufferInfo colorStorageInfo{};
    colorStorageInfo.buffer = resources->colorBuffer.buffer;
    colorStorageInfo.offset = 0;
    colorStorageInfo.range = resources->colorBuffer.size;

    VkDescriptorBufferInfo normalInfo{};
    normalInfo.buffer = resources->normalBuffer.buffer;
    normalInfo.offset = 0;
    normalInfo.range = resources->normalBuffer.size;

    VkDescriptorBufferInfo seepageNodeInfo{
        resources->seepageNodeBuffer.buffer,
        0,
        resources->seepageNodeBuffer.size};
    VkDescriptorBufferInfo seepageHashCellInfo{
        resources->seepageHashCellBuffer.buffer,
        0,
        resources->seepageHashCellBuffer.size};
    VkDescriptorBufferInfo seepageNodeReferenceInfo{
        resources->seepageNodeReferenceBuffer.buffer,
        0,
        resources->seepageNodeReferenceBuffer.size};
    VkDescriptorBufferInfo seepageParamsInfo{
        resources->seepageParamsBuffers[frameIndex].buffer,
        0,
        resources->seepageParamsBuffers[frameIndex].size};
    VkDescriptorBufferInfo rainImpactCountInfo{
        rainResources_.impactCountBuffer.buffer,
        0,
        rainResources_.impactCountBuffer.size};
    VkDescriptorBufferInfo rainImpactReferenceInfo{
        rainResources_.impactReferenceBuffer.buffer,
        0,
        rainResources_.impactReferenceBuffer.size};
    VkDescriptorBufferInfo rainImpactEventInfo{
        rainResources_.eventBuffer.buffer,
        0,
        rainResources_.eventBuffer.size};
    const auto meshFlowContactView =
        PointDescriptorDynamicMeshFlowContactView(resources);
    VkDescriptorBufferInfo meshFlowContactEventInfo{
        meshFlowContactView.eventBuffer,
        meshFlowContactView.eventOffset,
        meshFlowContactView.eventRange};
    VkDescriptorBufferInfo meshFlowContactGridInfo{
        meshFlowContactView.gridBuffer,
        meshFlowContactView.gridOffset,
        meshFlowContactView.gridRange};

    VkDescriptorImageInfo sceneDepthInfo{};
    sceneDepthInfo.imageView = sceneDepthView;
    sceneDepthInfo.imageLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL;

    std::array<VkWriteDescriptorSet, 16> writes{};
    writes[0] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[0].dstSet = descriptorSet;
    writes[0].dstBinding = 0;
    writes[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    writes[0].descriptorCount = 1;
    writes[0].pBufferInfo = &uniformInfo;

    writes[1] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[1].dstSet = descriptorSet;
    writes[1].dstBinding = 1;
    writes[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[1].descriptorCount = 1;
    writes[1].pBufferInfo = &scalarInfo;

    writes[2] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[2].dstSet = descriptorSet;
    writes[2].dstBinding = 2;
    writes[2].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    writes[2].descriptorCount = 1;
    writes[2].pBufferInfo = &styleInfo;

    writes[3] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[3].dstSet = descriptorSet;
    writes[3].dstBinding = 3;
    writes[3].descriptorType = VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT;
    writes[3].descriptorCount = 1;
    writes[3].pImageInfo = &sceneDepthInfo;

    writes[4] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[4].dstSet = descriptorSet;
    writes[4].dstBinding = 4;
    writes[4].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[4].descriptorCount = 1;
    writes[4].pBufferInfo = &positionStorageInfo;

    writes[5] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[5].dstSet = descriptorSet;
    writes[5].dstBinding = 5;
    writes[5].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[5].descriptorCount = 1;
    writes[5].pBufferInfo = &colorStorageInfo;

    writes[6] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[6].dstSet = descriptorSet;
    writes[6].dstBinding = 6;
    writes[6].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[6].descriptorCount = 1;
    writes[6].pBufferInfo = &normalInfo;

    writes[7] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[7].dstSet = descriptorSet;
    writes[7].dstBinding = 10;
    writes[7].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[7].descriptorCount = 1;
    writes[7].pBufferInfo = &seepageNodeInfo;

    writes[8] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[8].dstSet = descriptorSet;
    writes[8].dstBinding = 11;
    writes[8].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[8].descriptorCount = 1;
    writes[8].pBufferInfo = &seepageHashCellInfo;

    writes[9] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[9].dstSet = descriptorSet;
    writes[9].dstBinding = 12;
    writes[9].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[9].descriptorCount = 1;
    writes[9].pBufferInfo = &seepageNodeReferenceInfo;
    writes[10] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[10].dstSet = descriptorSet;
    writes[10].dstBinding = 13;
    writes[10].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[10].descriptorCount = 1;
    writes[10].pBufferInfo = &rainImpactCountInfo;
    writes[11] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[11].dstSet = descriptorSet;
    writes[11].dstBinding = 14;
    writes[11].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[11].descriptorCount = 1;
    writes[11].pBufferInfo = &rainImpactReferenceInfo;
    writes[12] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[12].dstSet = descriptorSet;
    writes[12].dstBinding = 15;
    writes[12].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[12].descriptorCount = 1;
    writes[12].pBufferInfo = &rainImpactEventInfo;
    writes[13] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[13].dstSet = descriptorSet;
    writes[13].dstBinding = 16;
    writes[13].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[13].descriptorCount = 1;
    writes[13].pBufferInfo = &seepageParamsInfo;
    writes[14] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[14].dstSet = descriptorSet;
    writes[14].dstBinding = 17;
    writes[14].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[14].descriptorCount = 1;
    writes[14].pBufferInfo = &meshFlowContactEventInfo;
    writes[15] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[15].dstSet = descriptorSet;
    writes[15].dstBinding = 18;
    writes[15].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[15].descriptorCount = 1;
    writes[15].pBufferInfo = &meshFlowContactGridInfo;

    vkUpdateDescriptorSets(device_, static_cast<std::uint32_t>(writes.size()), writes.data(), 0, nullptr);
}

void VulkanViewportShell::UpdateDynamicMeshFlowDescriptorSet(
    ActivePointCloudResources* resources,
    std::size_t liveSlot,
    const WaterGroundFlowGpuView& groundView) {
    if (resources == nullptr ||
        liveSlot >= kDynamicMeshFlowLiveSlots ||
        !groundView.valid ||
        resources->dynamicMeshFlowDescriptorPool == VK_NULL_HANDLE) {
        return;
    }
    auto& descriptorSet = resources->dynamicMeshFlowDescriptorSets[liveSlot];
    if (descriptorSet == VK_NULL_HANDLE) {
        VkDescriptorSetAllocateInfo allocInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
        allocInfo.descriptorPool = resources->dynamicMeshFlowDescriptorPool;
        allocInfo.descriptorSetCount = 1;
        allocInfo.pSetLayouts = &dynamicMeshFlowDescriptorSetLayout_;
        Check(
            vkAllocateDescriptorSets(device_, &allocInfo, &descriptorSet),
            "vkAllocateDescriptorSets(dynamic mesh flow)");
    }

    VkDescriptorBufferInfo uniformInfo{
        resources->dynamicMeshFlowUniformBuffers[liveSlot].buffer,
        0,
        resources->dynamicMeshFlowUniformBuffers[liveSlot].size};
    VkDescriptorBufferInfo groundInfo{
        groundView.buffer,
        groundView.offset,
        groundView.range};
    VkDescriptorBufferInfo particleInfo{
        resources->dynamicMeshFlowParticleBuffer.buffer,
        0,
        resources->dynamicMeshFlowParticleBuffer.size};
    VkDescriptorBufferInfo entryCandidateInfo{
        groundView.entryCandidateBuffer,
        groundView.entryCandidateOffset,
        groundView.entryCandidateRange};
    VkDescriptorBufferInfo contactEventInfo{
        resources->dynamicMeshFlowContactEventBuffer.buffer,
        0,
        resources->dynamicMeshFlowContactEventBuffer.size};
    VkDescriptorBufferInfo contactGridInfo{
        resources->dynamicMeshFlowContactGridBuffer.buffer,
        0,
        resources->dynamicMeshFlowContactGridBuffer.size};
    VkDescriptorBufferInfo counterInfo{
        resources->dynamicMeshFlowCounterBuffers[liveSlot].buffer,
        0,
        resources->dynamicMeshFlowCounterBuffers[liveSlot].size};
    VkDescriptorBufferInfo positionInfo{
        resources->positionStorageBuffer.buffer,
        0,
        resources->positionStorageBuffer.size};
    VkDescriptorBufferInfo normalInfo{
        resources->normalBuffer.buffer,
        0,
        resources->normalBuffer.size};
    VkDescriptorBufferInfo scalarInfo{
        resources->scalarFieldBuffer.buffer,
        0,
        resources->scalarFieldBuffer.size};
    VkDescriptorBufferInfo rainSeedInfo{
        rainResources_.dynamicMeshFlowRainSeedBuffer.buffer,
        0,
        rainResources_.dynamicMeshFlowRainSeedBuffer.size};

    std::array<VkWriteDescriptorSet, 11> writes{};
    VkDescriptorBufferInfo* infos[] = {
        &uniformInfo,
        &groundInfo,
        &particleInfo,
        &entryCandidateInfo,
        &contactEventInfo,
        &contactGridInfo,
        &counterInfo,
        &positionInfo,
        &normalInfo,
        &scalarInfo,
        &rainSeedInfo,
    };
    for (std::uint32_t bindingIndex = 0; bindingIndex < writes.size(); ++bindingIndex) {
        writes[bindingIndex] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
        writes[bindingIndex].dstSet = descriptorSet;
        writes[bindingIndex].dstBinding = bindingIndex;
        writes[bindingIndex].descriptorType =
            bindingIndex == 0U ? VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER : VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        writes[bindingIndex].descriptorCount = 1;
        writes[bindingIndex].pBufferInfo = infos[bindingIndex];
    }
    vkUpdateDescriptorSets(device_, static_cast<std::uint32_t>(writes.size()), writes.data(), 0, nullptr);
}

VkDescriptorPool VulkanViewportShell::CreateDynamicMeshFlowDescriptorPool() {
    const std::array<VkDescriptorPoolSize, 2> poolSizes = {
        MakePoolSize(
            VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,
            static_cast<std::uint32_t>(kDynamicMeshFlowLiveSlots)),
        MakePoolSize(
            VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
            static_cast<std::uint32_t>(kDynamicMeshFlowLiveSlots * 10U)),
    };
    VkDescriptorPoolCreateInfo poolInfo{
        VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
    poolInfo.maxSets =
        static_cast<std::uint32_t>(kDynamicMeshFlowLiveSlots);
    poolInfo.poolSizeCount =
        static_cast<std::uint32_t>(poolSizes.size());
    poolInfo.pPoolSizes = poolSizes.data();
    VkDescriptorPool descriptorPool = VK_NULL_HANDLE;
    Check(
        vkCreateDescriptorPool(
            device_,
            &poolInfo,
            nullptr,
            &descriptorPool),
        "vkCreateDescriptorPool(dynamic mesh flow generation)");
    return descriptorPool;
}

void VulkanViewportShell::PrepareDynamicMeshFlowDispatchSlot(
    ActivePointCloudResources* resources,
    std::size_t liveSlot) {
    if (resources == nullptr || liveSlot >= kDynamicMeshFlowLiveSlots || commandPool_ == VK_NULL_HANDLE) {
        return;
    }

    auto& fence = resources->dynamicMeshFlowDispatchFences[liveSlot];
    if (fence == VK_NULL_HANDLE) {
        VkFenceCreateInfo fenceInfo{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};
        fenceInfo.flags = VK_FENCE_CREATE_SIGNALED_BIT;
        Check(vkCreateFence(device_, &fenceInfo, nullptr, &fence), "vkCreateFence(dynamic mesh flow)");
    } else {
        const VkResult status = vkGetFenceStatus(device_, fence);
        if (status == VK_NOT_READY) {
            Check(vkWaitForFences(device_, 1, &fence, VK_TRUE, UINT64_MAX), "vkWaitForFences(dynamic mesh flow)");
        } else if (status != VK_SUCCESS) {
            Check(status, "vkGetFenceStatus(dynamic mesh flow)");
        }
    }

    auto& timestampQueryPool =
        resources->dynamicMeshFlowTimestampQueryPools[liveSlot];
    auto& timestampPending =
        resources->dynamicMeshFlowTimestampPending[liveSlot];
    if (timestampPending && timestampQueryPool != VK_NULL_HANDLE) {
        std::array<TimestampQueryResult, 2> results{};
        const VkResult queryResult = vkGetQueryPoolResults(
            device_,
            timestampQueryPool,
            0U,
            2U,
            sizeof(results),
            results.data(),
            sizeof(TimestampQueryResult),
            VK_QUERY_RESULT_64_BIT |
                VK_QUERY_RESULT_WITH_AVAILABILITY_BIT);
        timestampPending = false;
        if (queryResult == VK_SUCCESS &&
            TimestampResultsAvailable(results)) {
            const double milliseconds =
                TimestampDeltaMilliseconds(
                    results[0].value,
                    results[1].value,
                    gpuTimestampValidBits_,
                    gpuTimestampPeriodNanoseconds_);
            diagnostics_.gpuDynamicMeshFlow.milliseconds =
                milliseconds;
            diagnostics_.gpuDynamicMeshFlow.active = true;
            ++dynamicMeshFlowTimingResultGeneration_;
            const auto sampleTime = std::chrono::steady_clock::now();
            const double elapsedMilliseconds =
                lastDynamicMeshFlowTimingSampleAt_ ==
                        std::chrono::steady_clock::time_point{}
                    ? milliseconds
                    : std::chrono::duration<double, std::milli>(
                          sampleTime -
                          lastDynamicMeshFlowTimingSampleAt_)
                          .count();
            lastDynamicMeshFlowTimingSampleAt_ = sampleTime;
            dynamicMeshFlowTimingWindow_.AddFrame(
                {milliseconds},
                {true},
                elapsedMilliseconds);
            if (dynamicMeshFlowTimingWindow_.PublishIfReady(500.0)) {
                diagnostics_.gpuDynamicMeshFlow.averageMilliseconds =
                    dynamicMeshFlowTimingWindow_.PublishedAverage(0U);
            }
        } else {
            ++diagnostics_.gpuTimestampUnavailableResultCount;
        }
    }
    if (gpuTimestampsSupported_ && timestampQueryPool == VK_NULL_HANDLE) {
        VkQueryPoolCreateInfo queryPoolInfo{
            VK_STRUCTURE_TYPE_QUERY_POOL_CREATE_INFO};
        queryPoolInfo.queryType = VK_QUERY_TYPE_TIMESTAMP;
        queryPoolInfo.queryCount = 2U;
        if (vkCreateQueryPool(
                device_,
                &queryPoolInfo,
                nullptr,
                &timestampQueryPool) != VK_SUCCESS) {
            timestampQueryPool = VK_NULL_HANDLE;
        }
    }

    auto& commandBuffer = resources->dynamicMeshFlowCommandBuffers[liveSlot];
    if (commandBuffer != VK_NULL_HANDLE) {
        vkFreeCommandBuffers(device_, commandPool_, 1, &commandBuffer);
        commandBuffer = VK_NULL_HANDLE;
    }
}

void VulkanViewportShell::DispatchDynamicMeshFlowCompute(
    ActivePointCloudResources* resources,
    std::uint32_t particleCount,
    std::size_t liveSlot) {
    if (resources == nullptr ||
        particleCount == 0U ||
        liveSlot >= kDynamicMeshFlowLiveSlots ||
        resources->dynamicMeshFlowDescriptorSets[liveSlot] == VK_NULL_HANDLE ||
        resources->dynamicMeshFlowDispatchFences[liveSlot] == VK_NULL_HANDLE ||
        commandPool_ == VK_NULL_HANDLE) {
        return;
    }

    VkCommandBufferAllocateInfo allocInfo{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
    allocInfo.commandPool = commandPool_;
    allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    allocInfo.commandBufferCount = 1;
    VkCommandBuffer commandBuffer = VK_NULL_HANDLE;
    Check(vkAllocateCommandBuffers(device_, &allocInfo, &commandBuffer), "vkAllocateCommandBuffers(dynamic mesh flow)");
    resources->dynamicMeshFlowCommandBuffers[liveSlot] = commandBuffer;

    VkCommandBufferBeginInfo beginInfo{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
    beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    Check(vkBeginCommandBuffer(commandBuffer, &beginInfo), "vkBeginCommandBuffer(dynamic mesh flow)");

    const VkQueryPool timestampQueryPool =
        diagnosticsEnabled_ && gpuTimestampsSupported_
            ? resources->dynamicMeshFlowTimestampQueryPools[liveSlot]
            : VK_NULL_HANDLE;
    if (timestampQueryPool != VK_NULL_HANDLE) {
        vkCmdResetQueryPool(commandBuffer, timestampQueryPool, 0U, 2U);
    }

    vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, dynamicMeshFlowComputePipeline_);
    vkCmdBindDescriptorSets(
        commandBuffer,
        VK_PIPELINE_BIND_POINT_COMPUTE,
        dynamicMeshFlowPipelineLayout_,
        0,
        1,
        &resources->dynamicMeshFlowDescriptorSets[liveSlot],
        0,
        nullptr);

    const std::array<VkBuffer, 6> outputBuffers = {
        resources->dynamicMeshFlowParticleBuffer.buffer,
        resources->dynamicMeshFlowContactEventBuffer.buffer,
        resources->dynamicMeshFlowContactGridBuffer.buffer,
        resources->positionStorageBuffer.buffer,
        resources->normalBuffer.buffer,
        resources->scalarFieldBuffer.buffer,
    };
    std::array<VkBufferMemoryBarrier, 7> priorFrameBarriers{};
    for (std::size_t index = 0; index < priorFrameBarriers.size(); ++index) {
        auto& bufferBarrier = priorFrameBarriers[index];
        bufferBarrier.sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER;
        bufferBarrier.srcAccessMask = VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT;
        bufferBarrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT;
        bufferBarrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        bufferBarrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        bufferBarrier.buffer =
            index < outputBuffers.size()
                ? outputBuffers[index]
                : rainResources_.dynamicMeshFlowRainSeedBuffer.buffer;
        bufferBarrier.offset = 0U;
        bufferBarrier.size = VK_WHOLE_SIZE;
    }
    // Live tuning reuses settled output buffers. Serialize only those buffers
    // against older frame reads before this source-local compute update writes
    // them; the input-slot fence alone does not protect render consumers.
    vkCmdPipelineBarrier(
        commandBuffer,
        VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT |
            VK_PIPELINE_STAGE_VERTEX_SHADER_BIT |
            VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
        VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
        0U,
        0U,
        nullptr,
        static_cast<std::uint32_t>(priorFrameBarriers.size()),
        priorFrameBarriers.data(),
        0U,
        nullptr);
    // Written after the prior-frame drain barrier, at the compute stage so
    // the write is ordered after the barrier's compute-stage dependency
    // resolves (a top-of-pipe write here could still latch before the drain
    // completes). This keeps the timing band on the dispatch itself instead
    // of absorbing the queue-drain wait, which previously made this timer
    // read large while compute cost was small.
    if (timestampQueryPool != VK_NULL_HANDLE) {
        vkCmdWriteTimestamp(
            commandBuffer,
            VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
            timestampQueryPool,
            0U);
    }
    vkCmdDispatch(commandBuffer, (particleCount + 63U) / 64U, 1U, 1U);

    std::array<VkBufferMemoryBarrier, 6> barriers{};
    for (std::size_t index = 0; index < barriers.size(); ++index) {
        barriers[index] = {VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER};
        barriers[index].srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
        barriers[index].dstAccessMask = VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_VERTEX_ATTRIBUTE_READ_BIT;
        barriers[index].srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barriers[index].dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barriers[index].buffer = outputBuffers[index];
        barriers[index].offset = 0;
        barriers[index].size = VK_WHOLE_SIZE;
    }
    vkCmdPipelineBarrier(
        commandBuffer,
        VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
        VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT |
            VK_PIPELINE_STAGE_VERTEX_SHADER_BIT |
            VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT |
            VK_PIPELINE_STAGE_VERTEX_INPUT_BIT,
        0,
        0,
        nullptr,
        static_cast<std::uint32_t>(barriers.size()),
        barriers.data(),
        0,
        nullptr);
    if (timestampQueryPool != VK_NULL_HANDLE) {
        vkCmdWriteTimestamp(
            commandBuffer,
            VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT,
            timestampQueryPool,
            1U);
    }

    Check(vkEndCommandBuffer(commandBuffer), "vkEndCommandBuffer(dynamic mesh flow)");

    VkSubmitInfo submitInfo{VK_STRUCTURE_TYPE_SUBMIT_INFO};
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &commandBuffer;
    Check(
        vkResetFences(device_, 1, &resources->dynamicMeshFlowDispatchFences[liveSlot]),
        "vkResetFences(dynamic mesh flow)");
    Check(
        vkQueueSubmit(graphicsQueue_, 1, &submitInfo, resources->dynamicMeshFlowDispatchFences[liveSlot]),
        "vkQueueSubmit(dynamic mesh flow)");
    resources->dynamicMeshFlowTimestampPending[liveSlot] =
        timestampQueryPool != VK_NULL_HANDLE;
}

VkDescriptorPool VulkanViewportShell::CreateWaterFlowSourceDescriptorPool() {
    const std::array<VkDescriptorPoolSize, 2> poolSizes = {
        MakePoolSize(VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1U),
        MakePoolSize(VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 6U),
    };
    VkDescriptorPoolCreateInfo poolInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
    poolInfo.maxSets = 1U;
    poolInfo.poolSizeCount = static_cast<std::uint32_t>(poolSizes.size());
    poolInfo.pPoolSizes = poolSizes.data();
    VkDescriptorPool descriptorPool = VK_NULL_HANDLE;
    Check(
        vkCreateDescriptorPool(device_, &poolInfo, nullptr, &descriptorPool),
        "vkCreateDescriptorPool(water flow source generation)");
    return descriptorPool;
}

void VulkanViewportShell::CreateWaterFlowDummyPointResources(
    ActivePointCloudResources* resources) {
    if (resources == nullptr) {
        return;
    }

    const WaterSeepageNodeTopologyGpu emptySeepageNode{};
    resources->seepageNodeBuffer = CreateHostVisibleBuffer(
        sizeof(emptySeepageNode), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
    UploadBufferData(resources->seepageNodeBuffer, &emptySeepageNode, sizeof(emptySeepageNode));
    const WaterSeepageNodeParamsGpu emptySeepageParams{};
    for (auto& paramsBuffer : resources->seepageParamsBuffers) {
        paramsBuffer = CreateHostVisibleBuffer(
            sizeof(emptySeepageParams), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        UploadBufferData(paramsBuffer, &emptySeepageParams, sizeof(emptySeepageParams));
    }
    resources->seepageExrParamsBuffer = CreateHostVisibleBuffer(
        sizeof(emptySeepageParams), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
    UploadBufferData(
        resources->seepageExrParamsBuffer,
        &emptySeepageParams,
        sizeof(emptySeepageParams));
    resources->pendingSeepageParams.resize(sizeof(emptySeepageParams));
    std::memcpy(
        resources->pendingSeepageParams.data(),
        &emptySeepageParams,
        sizeof(emptySeepageParams));
    const WaterSeepageHashCellGpu emptySeepageHashCell{};
    resources->seepageHashCellBuffer = CreateHostVisibleBuffer(
        sizeof(emptySeepageHashCell), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
    UploadBufferData(
        resources->seepageHashCellBuffer,
        &emptySeepageHashCell,
        sizeof(emptySeepageHashCell));
    const invisible_places::water::WaterSeepageSupportReference emptyReference{};
    resources->seepageNodeReferenceBuffer = CreateHostVisibleBuffer(
        sizeof(emptyReference), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
    UploadBufferData(
        resources->seepageNodeReferenceBuffer,
        &emptyReference,
        sizeof(emptyReference));
    for (auto& styleBuffer : resources->styleBuffers) {
        styleBuffer = CreateHostVisibleBuffer(
            sizeof(PointCloudStyleGpu), VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT);
    }
    resources->exrStyleBuffer = CreateHostVisibleBuffer(
        sizeof(PointCloudStyleGpu), VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT);
}

void VulkanViewportShell::RetireWaterFlowSourceComputeGeneration(
    ActivePointCloudResources* resources) {
    if (resources == nullptr) {
        return;
    }
    if (resources->waterFlowSourceDispatchPending &&
        resources->waterFlowSourceDispatchFence != VK_NULL_HANDLE && device_ != VK_NULL_HANDLE) {
        const VkResult status = vkGetFenceStatus(device_, resources->waterFlowSourceDispatchFence);
        if (status == VK_NOT_READY) {
            vkWaitForFences(
                device_, 1U, &resources->waterFlowSourceDispatchFence, VK_TRUE, UINT64_MAX);
        }
    }
    if (resources->waterFlowSourceCommandBuffer != VK_NULL_HANDLE &&
        commandPool_ != VK_NULL_HANDLE && device_ != VK_NULL_HANDLE) {
        vkFreeCommandBuffers(
            device_, commandPool_, 1U, &resources->waterFlowSourceCommandBuffer);
        resources->waterFlowSourceCommandBuffer = VK_NULL_HANDLE;
    }
    resources->waterFlowSourceDispatchPending = false;

    // Destroying the generation pool frees the descriptor before any of its
    // input or output buffers can enter their retirement path.
    resources->waterFlowSourceDescriptorSet = VK_NULL_HANDLE;
    if (resources->waterFlowSourceDescriptorPool != VK_NULL_HANDLE &&
        device_ != VK_NULL_HANDLE) {
        vkDestroyDescriptorPool(device_, resources->waterFlowSourceDescriptorPool, nullptr);
        resources->waterFlowSourceDescriptorPool = VK_NULL_HANDLE;
    }
    if (resources->waterFlowSourceDispatchFence != VK_NULL_HANDLE && device_ != VK_NULL_HANDLE) {
        vkDestroyFence(device_, resources->waterFlowSourceDispatchFence, nullptr);
        resources->waterFlowSourceDispatchFence = VK_NULL_HANDLE;
    }
}

void VulkanViewportShell::UpdateWaterFlowSourceDescriptorSet(
    ActivePointCloudResources* resources,
    const WaterSurfaceFlowGpuView& surfaceView) {
    if (resources == nullptr) {
        return;
    }
    if (resources->waterFlowSourceDescriptorSet != VK_NULL_HANDLE ||
        resources->waterFlowSourceDescriptorPool != VK_NULL_HANDLE) {
        throw std::logic_error{
            "Water Flow source descriptors are immutable generations and cannot be rewritten."};
    }
    resources->waterFlowSourceDescriptorPool = CreateWaterFlowSourceDescriptorPool();
    auto& descriptorSet = resources->waterFlowSourceDescriptorSet;
    VkDescriptorSetAllocateInfo allocInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
    allocInfo.descriptorPool = resources->waterFlowSourceDescriptorPool;
    allocInfo.descriptorSetCount = 1U;
    allocInfo.pSetLayouts = &waterFlowSourceDescriptorSetLayout_;
    Check(
        vkAllocateDescriptorSets(device_, &allocInfo, &descriptorSet),
        "vkAllocateDescriptorSets(water flow source generation)");

    const bool hasSurface = surfaceView.valid && surfaceView.buffer != VK_NULL_HANDLE &&
                            surfaceView.range > 0U;
    std::array<VkDescriptorBufferInfo, 7> infos = {
        VkDescriptorBufferInfo{
            resources->waterFlowSourceUniformBuffer.buffer,
            0U,
            resources->waterFlowSourceUniformBuffer.size},
        VkDescriptorBufferInfo{
            resources->waterFlowSourceInputBuffer.buffer,
            0U,
            resources->waterFlowSourceInputBuffer.size},
        VkDescriptorBufferInfo{
            hasSurface ? surfaceView.buffer : resources->waterFlowSourceInputBuffer.buffer,
            hasSurface ? surfaceView.offset : 0U,
            hasSurface ? surfaceView.range : resources->waterFlowSourceInputBuffer.size},
        VkDescriptorBufferInfo{
            resources->waterFlowPendingPositionStorageBuffer.buffer,
            0U,
            resources->waterFlowPendingPositionStorageBuffer.size},
        VkDescriptorBufferInfo{
            resources->waterFlowPendingNormalBuffer.buffer,
            0U,
            resources->waterFlowPendingNormalBuffer.size},
        VkDescriptorBufferInfo{
            resources->waterFlowPendingScalarFieldBuffer.buffer,
            0U,
            resources->waterFlowPendingScalarFieldBuffer.size},
        VkDescriptorBufferInfo{
            resources->waterFlowSourceBranchBuffer.buffer,
            0U,
            resources->waterFlowSourceBranchBuffer.size},
    };
    std::array<VkWriteDescriptorSet, 7> writes{};
    for (std::uint32_t bindingIndex = 0U; bindingIndex < writes.size(); ++bindingIndex) {
        writes[bindingIndex] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
        writes[bindingIndex].dstSet = descriptorSet;
        writes[bindingIndex].dstBinding = bindingIndex;
        writes[bindingIndex].descriptorType =
            bindingIndex == 0U ? VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER : VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        writes[bindingIndex].descriptorCount = 1U;
        writes[bindingIndex].pBufferInfo = &infos[bindingIndex];
    }
    vkUpdateDescriptorSets(
        device_,
        static_cast<std::uint32_t>(writes.size()),
        writes.data(),
        0U,
        nullptr);
    ++resources->waterFlowSourceDescriptorGeneration;
}

void VulkanViewportShell::DispatchWaterFlowSourceCompute(
    ActivePointCloudResources* resources,
    const invisible_places::water::WaterFlowGpuOutputLayout& layout) {
    if (resources == nullptr || !layout.Valid() ||
        resources->waterFlowSourceDescriptorSet == VK_NULL_HANDLE ||
        commandPool_ == VK_NULL_HANDLE) {
        return;
    }
    if (resources->waterFlowSourceDispatchFence == VK_NULL_HANDLE) {
        VkFenceCreateInfo fenceInfo{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};
        fenceInfo.flags = VK_FENCE_CREATE_SIGNALED_BIT;
        Check(
            vkCreateFence(device_, &fenceInfo, nullptr, &resources->waterFlowSourceDispatchFence),
            "vkCreateFence(water flow source)");
    }
    if (resources->waterFlowSourceCommandBuffer != VK_NULL_HANDLE) {
        vkFreeCommandBuffers(
            device_,
            commandPool_,
            1U,
            &resources->waterFlowSourceCommandBuffer);
        resources->waterFlowSourceCommandBuffer = VK_NULL_HANDLE;
    }

    VkCommandBufferAllocateInfo allocInfo{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
    allocInfo.commandPool = commandPool_;
    allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    allocInfo.commandBufferCount = 1U;
    Check(
        vkAllocateCommandBuffers(device_, &allocInfo, &resources->waterFlowSourceCommandBuffer),
        "vkAllocateCommandBuffers(water flow source)");
    VkCommandBufferBeginInfo beginInfo{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
    beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    Check(
        vkBeginCommandBuffer(resources->waterFlowSourceCommandBuffer, &beginInfo),
        "vkBeginCommandBuffer(water flow source)");

    vkCmdBindDescriptorSets(
        resources->waterFlowSourceCommandBuffer,
        VK_PIPELINE_BIND_POINT_COMPUTE,
        waterFlowSourcePipelineLayout_,
        0U,
        1U,
        &resources->waterFlowSourceDescriptorSet,
        0U,
        nullptr);
    vkCmdBindPipeline(
        resources->waterFlowSourceCommandBuffer,
        VK_PIPELINE_BIND_POINT_COMPUTE,
        waterFlowRouteComputePipeline_);
    vkCmdDispatch(
        resources->waterFlowSourceCommandBuffer,
        (layout.maxActiveRouteLaneCount + 31U) / 32U,
        layout.branchCount,
        1U);

    std::array<VkBufferMemoryBarrier, 3> barriers{};
    const std::array<VkBuffer, 3> outputBuffers = {
        resources->waterFlowPendingPositionStorageBuffer.buffer,
        resources->waterFlowPendingNormalBuffer.buffer,
        resources->waterFlowPendingScalarFieldBuffer.buffer,
    };
    for (std::size_t index = 0U; index < barriers.size(); ++index) {
        barriers[index] = {VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER};
        barriers[index].srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
        barriers[index].dstAccessMask = VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT;
        barriers[index].srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barriers[index].dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barriers[index].buffer = outputBuffers[index];
        barriers[index].offset = 0U;
        barriers[index].size = VK_WHOLE_SIZE;
    }
    vkCmdPipelineBarrier(
        resources->waterFlowSourceCommandBuffer,
        VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
        VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
        0U,
        0U,
        nullptr,
        static_cast<std::uint32_t>(barriers.size()),
        barriers.data(),
        0U,
        nullptr);

    vkCmdBindPipeline(
        resources->waterFlowSourceCommandBuffer,
        VK_PIPELINE_BIND_POINT_COMPUTE,
        waterFlowTrailComputePipeline_);
    vkCmdDispatch(
        resources->waterFlowSourceCommandBuffer,
        (layout.maxTrailsPerBranch + 63U) / 64U,
        layout.branchCount,
        1U);
    for (auto& barrier : barriers) {
        barrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
        barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_VERTEX_ATTRIBUTE_READ_BIT;
    }
    vkCmdPipelineBarrier(
        resources->waterFlowSourceCommandBuffer,
        VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
        VK_PIPELINE_STAGE_VERTEX_SHADER_BIT | VK_PIPELINE_STAGE_VERTEX_INPUT_BIT,
        0U,
        0U,
        nullptr,
        static_cast<std::uint32_t>(barriers.size()),
        barriers.data(),
        0U,
        nullptr);
    Check(
        vkEndCommandBuffer(resources->waterFlowSourceCommandBuffer),
        "vkEndCommandBuffer(water flow source)");

    VkSubmitInfo submitInfo{VK_STRUCTURE_TYPE_SUBMIT_INFO};
    submitInfo.commandBufferCount = 1U;
    submitInfo.pCommandBuffers = &resources->waterFlowSourceCommandBuffer;
    Check(
        vkResetFences(device_, 1U, &resources->waterFlowSourceDispatchFence),
        "vkResetFences(water flow source)");
    Check(
        vkQueueSubmit(
            graphicsQueue_,
            1U,
            &submitInfo,
            resources->waterFlowSourceDispatchFence),
        "vkQueueSubmit(water flow source)");
    resources->waterFlowSourceDispatchPending = true;
}

void VulkanViewportShell::PollWaterFlowSourceDispatches() {
    struct QueuedRequest {
        std::size_t layerId = 0U;
        std::uint32_t sourceId = 0U;
        std::uint64_t revision = 0U;
        invisible_places::water::WaterFlowGpuInputKind inputKind =
            invisible_places::water::WaterFlowGpuInputKind::SampledAnchors;
        std::shared_ptr<const invisible_places::water::WaterFlowGpuCompactSourceInput>
            compactSourceInput;
        invisible_places::water::WaterFlowTrailSettings settings{};
        bool useSurfaceGuide = false;
    };
    std::vector<QueuedRequest> queuedRequests;
    std::vector<std::size_t> completedDeletes;

    bool exrPending = false;
    if (exrExportResources_.fence != VK_NULL_HANDLE) {
        const VkResult exrStatus = vkGetFenceStatus(device_, exrExportResources_.fence);
        exrPending = exrStatus == VK_NOT_READY;
        if (!exrPending && exrStatus != VK_SUCCESS) {
            Check(exrStatus, "vkGetFenceStatus(water flow retirement EXR)");
        }
    }

    const std::uint32_t currentFrameBit = 1U << static_cast<std::uint32_t>(currentFrameIndex_);
    for (auto& resources : pointCloudResources_) {
        for (auto& retired : resources.waterFlowRetiredOutputs) {
            if ((retired.outstandingFrameMask & currentFrameBit) != 0U) {
                retired.outstandingFrameMask &= ~currentFrameBit;
            }
            if (retired.outstandingExr && !exrPending) {
                retired.outstandingExr = false;
            }
        }
        for (auto retiredIt = resources.waterFlowRetiredOutputs.begin();
             retiredIt != resources.waterFlowRetiredOutputs.end();) {
            if (retiredIt->outstandingFrameMask != 0U || retiredIt->outstandingExr) {
                ++retiredIt;
                continue;
            }
            if (!resources.waterFlowSourceDispatchPending &&
                resources.waterFlowPendingPositionStorageBuffer.buffer == VK_NULL_HANDLE) {
                resources.waterFlowPendingPositionBuffer = retiredIt->positionBuffer;
                resources.waterFlowPendingPositionStorageBuffer = retiredIt->positionStorageBuffer;
                resources.waterFlowPendingColorBuffer = retiredIt->colorBuffer;
                resources.waterFlowPendingNormalBuffer = retiredIt->normalBuffer;
                resources.waterFlowPendingScalarFieldBuffer = retiredIt->scalarFieldBuffer;
                resources.waterFlowSparePointCapacity = retiredIt->pointCapacity;
                retiredIt->positionBuffer = {};
                retiredIt->positionStorageBuffer = {};
                retiredIt->colorBuffer = {};
                retiredIt->normalBuffer = {};
                retiredIt->scalarFieldBuffer = {};
            } else {
                DestroyBuffer(&retiredIt->positionBuffer);
                DestroyBuffer(&retiredIt->positionStorageBuffer);
                DestroyBuffer(&retiredIt->colorBuffer);
                DestroyBuffer(&retiredIt->normalBuffer);
                DestroyBuffer(&retiredIt->scalarFieldBuffer);
            }
            retiredIt = resources.waterFlowRetiredOutputs.erase(retiredIt);
        }

        if (resources.waterFlowDeletePending) {
            resources.waterFlowDeleteOutstandingFrameMask &= ~currentFrameBit;
            const bool computeComplete =
                resources.waterFlowSourceDispatchFence == VK_NULL_HANDLE ||
                vkGetFenceStatus(device_, resources.waterFlowSourceDispatchFence) == VK_SUCCESS;
            if (resources.waterFlowDeleteOutstandingFrameMask == 0U &&
                computeComplete &&
                !exrPending) {
                completedDeletes.push_back(resources.layerId);
            }
        }
    }

    for (auto& resources : pointCloudResources_) {
        if (!resources.waterFlowSourceActive || !resources.waterFlowSourceDispatchPending ||
            resources.waterFlowSourceDispatchFence == VK_NULL_HANDLE) {
            continue;
        }
        const VkResult status = vkGetFenceStatus(device_, resources.waterFlowSourceDispatchFence);
        if (status == VK_NOT_READY) {
            continue;
        }
        Check(status, "vkGetFenceStatus(water flow source)");
        RetireWaterFlowSourceComputeGeneration(&resources);
        DestroyBuffer(&resources.waterFlowSourceUniformBuffer);
        DestroyBuffer(&resources.waterFlowSourceInputBuffer);
        DestroyBuffer(&resources.waterFlowSourceBranchBuffer);
        resources.waterFlowSourceInputCapacity = 0U;
        resources.waterFlowSourceBranchCapacity = 0U;

        if (resources.waterFlowSourceQueued &&
            resources.waterFlowQueuedRevision > resources.waterFlowPendingRevision) {
            QueuedRequest queued;
            queued.layerId = resources.layerId;
            queued.sourceId = resources.waterFlowQueuedSourceId;
            queued.revision = resources.waterFlowQueuedRevision;
            queued.inputKind = resources.waterFlowQueuedInputKind;
            queued.compactSourceInput =
                std::move(resources.waterFlowQueuedCompactSourceInput);
            queued.settings = resources.waterFlowQueuedSettings;
            queued.useSurfaceGuide = resources.waterFlowQueuedUseSurfaceGuide;
            queuedRequests.push_back(std::move(queued));
            resources.waterFlowSourceQueued = false;
            resources.waterFlowQueuedRevision = 0U;
            resources.waterFlowPendingRevision = 0U;
            resources.waterFlowPendingLayout = {};
            resources.waterFlowSourceRequestedRevision = resources.waterFlowSourceCompletedRevision;
            continue;
        }

        if (resources.waterFlowDeletePending) {
            continue;
        }
        // Promote all five shape-changing outputs through one complete point
        // descriptor generation. The old generation remains drawable until
        // allocation, binding, queue settling, and command-buffer retirement
        // have all succeeded.
        const auto settledPointCount = resources.pointCount;
        const auto settledFlowPointCount = resources.waterFlowSettledPointCount;
        const auto settledActivePointCount = resources.activePointCount;
        const auto settledScalarFieldCount = resources.scalarFieldCount;
        const auto settledHasSourceRgb = resources.hasSourceRgb;
        const auto settledHasNormals = resources.hasNormals;
        const auto settledUsingSampledIndices = resources.usingSampledIndices;
        const auto settledInteractiveSampledIndexCount =
            resources.interactiveSampledIndexCount;
        std::swap(resources.positionBuffer, resources.waterFlowPendingPositionBuffer);
        std::swap(
            resources.positionStorageBuffer,
            resources.waterFlowPendingPositionStorageBuffer);
        std::swap(resources.colorBuffer, resources.waterFlowPendingColorBuffer);
        std::swap(resources.normalBuffer, resources.waterFlowPendingNormalBuffer);
        std::swap(resources.scalarFieldBuffer, resources.waterFlowPendingScalarFieldBuffer);
        resources.pointCount = resources.waterFlowPendingLayout.pointCapacity;
        resources.waterFlowSettledPointCount = resources.waterFlowPendingLayout.pointCount;
        resources.activePointCount = resources.waterFlowPendingLayout.pointCount;
        resources.scalarFieldCount = invisible_places::water::kWaterTrailScalarFieldCount;
        resources.hasSourceRgb = true;
        resources.hasNormals = true;
        resources.usingSampledIndices = false;
        resources.interactiveSampledIndexCount = 0U;
        try {
            ReplacePointCloudDescriptorSets(&resources);
            if (resources.depthSortDescriptorSet != VK_NULL_HANDLE) {
                UpdatePointDepthSortDescriptorSet(&resources);
            }
        } catch (...) {
            resources.pointCount = settledPointCount;
            resources.waterFlowSettledPointCount = settledFlowPointCount;
            resources.activePointCount = settledActivePointCount;
            resources.scalarFieldCount = settledScalarFieldCount;
            resources.hasSourceRgb = settledHasSourceRgb;
            resources.hasNormals = settledHasNormals;
            resources.usingSampledIndices = settledUsingSampledIndices;
            resources.interactiveSampledIndexCount =
                settledInteractiveSampledIndexCount;
            std::swap(resources.positionBuffer, resources.waterFlowPendingPositionBuffer);
            std::swap(
                resources.positionStorageBuffer,
                resources.waterFlowPendingPositionStorageBuffer);
            std::swap(resources.colorBuffer, resources.waterFlowPendingColorBuffer);
            std::swap(resources.normalBuffer, resources.waterFlowPendingNormalBuffer);
            std::swap(resources.scalarFieldBuffer, resources.waterFlowPendingScalarFieldBuffer);
            RetireWaterFlowSourceComputeGeneration(&resources);
            DestroyBuffer(&resources.waterFlowPendingPositionBuffer);
            DestroyBuffer(&resources.waterFlowPendingPositionStorageBuffer);
            DestroyBuffer(&resources.waterFlowPendingColorBuffer);
            DestroyBuffer(&resources.waterFlowPendingNormalBuffer);
            DestroyBuffer(&resources.waterFlowPendingScalarFieldBuffer);
            DestroyBuffer(&resources.waterFlowSourceUniformBuffer);
            DestroyBuffer(&resources.waterFlowSourceInputBuffer);
            DestroyBuffer(&resources.waterFlowSourceBranchBuffer);
            resources.waterFlowSourceInputCapacity = 0U;
            resources.waterFlowSourceBranchCapacity = 0U;
            resources.waterFlowSparePointCapacity = 0U;
            resources.waterFlowPendingRevision = 0U;
            resources.waterFlowPendingSourceId = 0U;
            resources.waterFlowPendingLayout = {};
            resources.waterFlowSourceRequestedRevision =
                resources.waterFlowSourceCompletedRevision;
            TrackPointCloudResidentPeak();
            continue;
        }
        for (auto& highlight : resources.highlights) {
            // Highlight indices describe the prior Flow topology; the fresh
            // descriptor generation keeps their resources valid, while a zero
            // count prevents stale indices from being submitted.
            highlight.indexCount = 0U;
        }
        DestroyBuffer(&resources.sampledIndexBuffer);
        DestroyBuffer(&resources.sampledSurfelIndexBuffer);
        DestroyBuffer(&resources.interactiveSampledIndexBuffer);
        DestroyBuffer(&resources.interactiveSurfelIndexBuffer);
        resources.waterFlowSparePointCapacity = settledPointCount;
        resources.waterFlowSourceId = resources.waterFlowPendingSourceId;
        resources.waterFlowSourceCompletedRevision = resources.waterFlowPendingRevision;
        resources.waterFlowPendingRevision = 0U;
        resources.waterFlowPendingLayout = {};
        TrackPointCloudResidentPeak();
        ++sceneRevision_;
    }

    for (auto& queued : queuedRequests) {
        WaterFlowGpuSourceRequest request;
        request.sourceId = queued.sourceId;
        request.sourceRevision = queued.revision;
        request.inputKind = queued.inputKind;
        request.compactSourceInput = std::move(queued.compactSourceInput);
        request.settings = queued.settings;
        request.useSurfaceGuide = queued.useSurfaceGuide;
        (void)UploadWaterFlowGpuSource(queued.layerId, request);
    }

    for (const auto layerId : completedDeletes) {
        auto resourcesIt = std::find_if(
            pointCloudResources_.begin(),
            pointCloudResources_.end(),
            [layerId](const ActivePointCloudResources& resources) {
                return resources.layerId == layerId && resources.waterFlowDeletePending;
            });
        if (resourcesIt == pointCloudResources_.end()) {
            continue;
        }
        CleanupPointCloudResources(&(*resourcesIt));
        pointCloudResources_.erase(resourcesIt);
    }
}

void VulkanViewportShell::UpdatePointHighlightDescriptorSets(
    ActivePointCloudResources* resources,
    ActivePointCloudResources::PointHighlightResources* highlight,
    VkDescriptorPool allocationPool) {
    if (resources == nullptr || highlight == nullptr) {
        return;
    }
    for (std::size_t frameIndex = 0; frameIndex < kFramesInFlight; ++frameIndex) {
        highlight->descriptorSets[frameIndex].resize(depthImages_.size(), VK_NULL_HANDLE);
        highlight->seepageDescriptorGenerations[frameIndex].resize(
            depthImages_.size(),
            0U);
        for (std::uint32_t imageIndex = 0; imageIndex < depthImages_.size(); ++imageIndex) {
            UpdatePointHighlightDescriptorSet(
                resources,
                highlight,
                frameIndex,
                imageIndex,
                depthImages_[imageIndex].view,
                allocationPool);
            highlight->seepageDescriptorGenerations[frameIndex][imageIndex] =
                resources->seepageTopologyGeneration;
        }
    }
}

void VulkanViewportShell::UpdatePointHighlightDescriptorSet(
    ActivePointCloudResources* resources,
    ActivePointCloudResources::PointHighlightResources* highlight,
    std::size_t frameIndex,
    std::uint32_t imageIndex,
    VkImageView sceneDepthView,
    VkDescriptorPool allocationPool) {
    if (resources == nullptr ||
        highlight == nullptr ||
        frameIndex >= kFramesInFlight ||
        imageIndex >= depthImages_.size()) {
        return;
    }

    auto& descriptorSet = highlight->descriptorSets[frameIndex][imageIndex];
    if (descriptorSet == VK_NULL_HANDLE) {
        const auto descriptorPool = allocationPool != VK_NULL_HANDLE
                                        ? allocationPool
                                        : resources->pointDescriptorPool;
        if (descriptorPool == VK_NULL_HANDLE) {
            throw std::runtime_error{"Point highlight descriptor generation has no allocation pool."};
        }
        VkDescriptorSetAllocateInfo allocInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
        allocInfo.descriptorPool = descriptorPool;
        allocInfo.descriptorSetCount = 1;
        allocInfo.pSetLayouts = &pointDescriptorSetLayout_;
        Check(
            vkAllocateDescriptorSets(device_, &allocInfo, &descriptorSet),
            "vkAllocateDescriptorSets(point highlight)");
    }

    VkDescriptorBufferInfo uniformInfo{};
    uniformInfo.buffer = frameResources_[frameIndex].uniformBuffer.buffer;
    uniformInfo.offset = 0;
    uniformInfo.range = sizeof(FrameUniforms);

    VkDescriptorBufferInfo scalarInfo{};
    scalarInfo.buffer = resources->scalarFieldBuffer.buffer;
    scalarInfo.offset = 0;
    scalarInfo.range = resources->scalarFieldBuffer.size;

    VkDescriptorBufferInfo styleInfo{};
    styleInfo.buffer = highlight->styleBuffers[frameIndex].buffer;
    styleInfo.offset = 0;
    styleInfo.range = sizeof(PointCloudStyleGpu);

    VkDescriptorBufferInfo positionStorageInfo{};
    positionStorageInfo.buffer = resources->positionStorageBuffer.buffer;
    positionStorageInfo.offset = 0;
    positionStorageInfo.range = resources->positionStorageBuffer.size;

    VkDescriptorBufferInfo colorStorageInfo{};
    colorStorageInfo.buffer = resources->colorBuffer.buffer;
    colorStorageInfo.offset = 0;
    colorStorageInfo.range = resources->colorBuffer.size;

    VkDescriptorBufferInfo normalInfo{};
    normalInfo.buffer = resources->normalBuffer.buffer;
    normalInfo.offset = 0;
    normalInfo.range = resources->normalBuffer.size;

    VkDescriptorBufferInfo seepageNodeInfo{
        resources->seepageNodeBuffer.buffer,
        0,
        resources->seepageNodeBuffer.size};
    VkDescriptorBufferInfo seepageHashCellInfo{
        resources->seepageHashCellBuffer.buffer,
        0,
        resources->seepageHashCellBuffer.size};
    VkDescriptorBufferInfo seepageNodeReferenceInfo{
        resources->seepageNodeReferenceBuffer.buffer,
        0,
        resources->seepageNodeReferenceBuffer.size};
    VkDescriptorBufferInfo seepageParamsInfo{
        resources->seepageParamsBuffers[frameIndex].buffer,
        0,
        resources->seepageParamsBuffers[frameIndex].size};
    VkDescriptorBufferInfo rainImpactCountInfo{
        rainResources_.impactCountBuffer.buffer,
        0,
        rainResources_.impactCountBuffer.size};
    VkDescriptorBufferInfo rainImpactReferenceInfo{
        rainResources_.impactReferenceBuffer.buffer,
        0,
        rainResources_.impactReferenceBuffer.size};
    VkDescriptorBufferInfo rainImpactEventInfo{
        rainResources_.eventBuffer.buffer,
        0,
        rainResources_.eventBuffer.size};
    const auto meshFlowContactView =
        PointDescriptorDynamicMeshFlowContactView(resources);
    VkDescriptorBufferInfo meshFlowContactEventInfo{
        meshFlowContactView.eventBuffer,
        meshFlowContactView.eventOffset,
        meshFlowContactView.eventRange};
    VkDescriptorBufferInfo meshFlowContactGridInfo{
        meshFlowContactView.gridBuffer,
        meshFlowContactView.gridOffset,
        meshFlowContactView.gridRange};

    VkDescriptorImageInfo sceneDepthInfo{};
    sceneDepthInfo.imageView = sceneDepthView;
    sceneDepthInfo.imageLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL;

    std::array<VkWriteDescriptorSet, 16> writes{};
    writes[0] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[0].dstSet = descriptorSet;
    writes[0].dstBinding = 0;
    writes[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    writes[0].descriptorCount = 1;
    writes[0].pBufferInfo = &uniformInfo;

    writes[1] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[1].dstSet = descriptorSet;
    writes[1].dstBinding = 1;
    writes[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[1].descriptorCount = 1;
    writes[1].pBufferInfo = &scalarInfo;

    writes[2] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[2].dstSet = descriptorSet;
    writes[2].dstBinding = 2;
    writes[2].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    writes[2].descriptorCount = 1;
    writes[2].pBufferInfo = &styleInfo;

    writes[3] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[3].dstSet = descriptorSet;
    writes[3].dstBinding = 3;
    writes[3].descriptorType = VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT;
    writes[3].descriptorCount = 1;
    writes[3].pImageInfo = &sceneDepthInfo;

    writes[4] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[4].dstSet = descriptorSet;
    writes[4].dstBinding = 4;
    writes[4].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[4].descriptorCount = 1;
    writes[4].pBufferInfo = &positionStorageInfo;

    writes[5] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[5].dstSet = descriptorSet;
    writes[5].dstBinding = 5;
    writes[5].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[5].descriptorCount = 1;
    writes[5].pBufferInfo = &colorStorageInfo;

    writes[6] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[6].dstSet = descriptorSet;
    writes[6].dstBinding = 6;
    writes[6].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[6].descriptorCount = 1;
    writes[6].pBufferInfo = &normalInfo;

    writes[7] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[7].dstSet = descriptorSet;
    writes[7].dstBinding = 10;
    writes[7].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[7].descriptorCount = 1;
    writes[7].pBufferInfo = &seepageNodeInfo;

    writes[8] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[8].dstSet = descriptorSet;
    writes[8].dstBinding = 11;
    writes[8].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[8].descriptorCount = 1;
    writes[8].pBufferInfo = &seepageHashCellInfo;

    writes[9] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[9].dstSet = descriptorSet;
    writes[9].dstBinding = 12;
    writes[9].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[9].descriptorCount = 1;
    writes[9].pBufferInfo = &seepageNodeReferenceInfo;
    writes[10] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[10].dstSet = descriptorSet;
    writes[10].dstBinding = 13;
    writes[10].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[10].descriptorCount = 1;
    writes[10].pBufferInfo = &rainImpactCountInfo;
    writes[11] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[11].dstSet = descriptorSet;
    writes[11].dstBinding = 14;
    writes[11].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[11].descriptorCount = 1;
    writes[11].pBufferInfo = &rainImpactReferenceInfo;
    writes[12] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[12].dstSet = descriptorSet;
    writes[12].dstBinding = 15;
    writes[12].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[12].descriptorCount = 1;
    writes[12].pBufferInfo = &rainImpactEventInfo;
    writes[13] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[13].dstSet = descriptorSet;
    writes[13].dstBinding = 16;
    writes[13].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[13].descriptorCount = 1;
    writes[13].pBufferInfo = &seepageParamsInfo;
    writes[14] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[14].dstSet = descriptorSet;
    writes[14].dstBinding = 17;
    writes[14].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[14].descriptorCount = 1;
    writes[14].pBufferInfo = &meshFlowContactEventInfo;
    writes[15] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[15].dstSet = descriptorSet;
    writes[15].dstBinding = 18;
    writes[15].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[15].descriptorCount = 1;
    writes[15].pBufferInfo = &meshFlowContactGridInfo;

    vkUpdateDescriptorSets(device_, static_cast<std::uint32_t>(writes.size()), writes.data(), 0, nullptr);
}

void VulkanViewportShell::UpdatePointCloudExrDescriptorSet(
    ActivePointCloudResources* resources,
    VkImageView sceneDepthView,
    VkBuffer exrFrameUniformBuffer,
    VkDescriptorPool allocationPool) {
    if (resources == nullptr) {
        return;
    }
    // The frame uniform buffer must be passed explicitly: during EXR resource
    // recreation the settled exrExportResources_ member still holds the OLD
    // (or empty) uniform buffer while the staged descriptor generations are
    // built. Binding the member here silently attached every EXR descriptor
    // set to a destroyed buffer, and the export then rendered every point
    // layer with garbage view-projection uniforms - empty frames while rain
    // (whose descriptor rebinds each Begin) kept rendering.
    if (exrFrameUniformBuffer == VK_NULL_HANDLE) {
        throw std::runtime_error{
            "Point EXR descriptor update requires the frame uniform buffer."};
    }

    if (resources->exrDescriptorSet == VK_NULL_HANDLE) {
        const auto descriptorPool = allocationPool != VK_NULL_HANDLE
                                        ? allocationPool
                                        : resources->pointDescriptorPool;
        if (descriptorPool == VK_NULL_HANDLE) {
            throw std::runtime_error{"Point EXR descriptor generation has no allocation pool."};
        }
        VkDescriptorSetAllocateInfo allocInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
        allocInfo.descriptorPool = descriptorPool;
        allocInfo.descriptorSetCount = 1;
        allocInfo.pSetLayouts = &pointDescriptorSetLayout_;
        Check(
            vkAllocateDescriptorSets(device_, &allocInfo, &resources->exrDescriptorSet),
            "vkAllocateDescriptorSets(point exr)");
    }

    VkDescriptorBufferInfo uniformInfo{exrFrameUniformBuffer, 0, sizeof(FrameUniforms)};
    VkDescriptorBufferInfo scalarInfo{resources->scalarFieldBuffer.buffer, 0, resources->scalarFieldBuffer.size};
    VkDescriptorBufferInfo styleInfo{resources->exrStyleBuffer.buffer, 0, sizeof(PointCloudStyleGpu)};
    VkDescriptorBufferInfo positionStorageInfo{
        resources->positionStorageBuffer.buffer,
        0,
        resources->positionStorageBuffer.size};
    VkDescriptorBufferInfo colorStorageInfo{resources->colorBuffer.buffer, 0, resources->colorBuffer.size};
    VkDescriptorBufferInfo normalInfo{resources->normalBuffer.buffer, 0, resources->normalBuffer.size};
    VkDescriptorBufferInfo seepageNodeInfo{
        resources->seepageNodeBuffer.buffer,
        0,
        resources->seepageNodeBuffer.size};
    VkDescriptorBufferInfo seepageHashCellInfo{
        resources->seepageHashCellBuffer.buffer,
        0,
        resources->seepageHashCellBuffer.size};
    VkDescriptorBufferInfo seepageNodeReferenceInfo{
        resources->seepageNodeReferenceBuffer.buffer,
        0,
        resources->seepageNodeReferenceBuffer.size};
    VkDescriptorBufferInfo seepageParamsInfo{
        resources->seepageExrParamsBuffer.buffer,
        0,
        resources->seepageExrParamsBuffer.size};
    VkDescriptorBufferInfo rainImpactCountInfo{
        rainResources_.impactCountBuffer.buffer,
        0,
        rainResources_.impactCountBuffer.size};
    VkDescriptorBufferInfo rainImpactReferenceInfo{
        rainResources_.impactReferenceBuffer.buffer,
        0,
        rainResources_.impactReferenceBuffer.size};
    VkDescriptorBufferInfo rainImpactEventInfo{
        rainResources_.eventBuffer.buffer,
        0,
        rainResources_.eventBuffer.size};
    const auto meshFlowContactView =
        PointDescriptorDynamicMeshFlowContactView(resources);
    VkDescriptorBufferInfo meshFlowContactEventInfo{
        meshFlowContactView.eventBuffer,
        meshFlowContactView.eventOffset,
        meshFlowContactView.eventRange};
    VkDescriptorBufferInfo meshFlowContactGridInfo{
        meshFlowContactView.gridBuffer,
        meshFlowContactView.gridOffset,
        meshFlowContactView.gridRange};
    VkDescriptorImageInfo sceneDepthInfo{};
    sceneDepthInfo.imageView = sceneDepthView;
    sceneDepthInfo.imageLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL;

    std::array<VkWriteDescriptorSet, 16> writes{};
    writes[0] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[0].dstSet = resources->exrDescriptorSet;
    writes[0].dstBinding = 0;
    writes[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    writes[0].descriptorCount = 1;
    writes[0].pBufferInfo = &uniformInfo;

    writes[1] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[1].dstSet = resources->exrDescriptorSet;
    writes[1].dstBinding = 1;
    writes[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[1].descriptorCount = 1;
    writes[1].pBufferInfo = &scalarInfo;

    writes[2] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[2].dstSet = resources->exrDescriptorSet;
    writes[2].dstBinding = 2;
    writes[2].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    writes[2].descriptorCount = 1;
    writes[2].pBufferInfo = &styleInfo;

    writes[3] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[3].dstSet = resources->exrDescriptorSet;
    writes[3].dstBinding = 3;
    writes[3].descriptorType = VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT;
    writes[3].descriptorCount = 1;
    writes[3].pImageInfo = &sceneDepthInfo;

    writes[4] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[4].dstSet = resources->exrDescriptorSet;
    writes[4].dstBinding = 4;
    writes[4].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[4].descriptorCount = 1;
    writes[4].pBufferInfo = &positionStorageInfo;

    writes[5] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[5].dstSet = resources->exrDescriptorSet;
    writes[5].dstBinding = 5;
    writes[5].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[5].descriptorCount = 1;
    writes[5].pBufferInfo = &colorStorageInfo;

    writes[6] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[6].dstSet = resources->exrDescriptorSet;
    writes[6].dstBinding = 6;
    writes[6].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[6].descriptorCount = 1;
    writes[6].pBufferInfo = &normalInfo;

    writes[7] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[7].dstSet = resources->exrDescriptorSet;
    writes[7].dstBinding = 10;
    writes[7].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[7].descriptorCount = 1;
    writes[7].pBufferInfo = &seepageNodeInfo;

    writes[8] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[8].dstSet = resources->exrDescriptorSet;
    writes[8].dstBinding = 11;
    writes[8].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[8].descriptorCount = 1;
    writes[8].pBufferInfo = &seepageHashCellInfo;

    writes[9] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[9].dstSet = resources->exrDescriptorSet;
    writes[9].dstBinding = 12;
    writes[9].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[9].descriptorCount = 1;
    writes[9].pBufferInfo = &seepageNodeReferenceInfo;
    writes[10] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[10].dstSet = resources->exrDescriptorSet;
    writes[10].dstBinding = 13;
    writes[10].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[10].descriptorCount = 1;
    writes[10].pBufferInfo = &rainImpactCountInfo;
    writes[11] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[11].dstSet = resources->exrDescriptorSet;
    writes[11].dstBinding = 14;
    writes[11].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[11].descriptorCount = 1;
    writes[11].pBufferInfo = &rainImpactReferenceInfo;
    writes[12] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[12].dstSet = resources->exrDescriptorSet;
    writes[12].dstBinding = 15;
    writes[12].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[12].descriptorCount = 1;
    writes[12].pBufferInfo = &rainImpactEventInfo;
    writes[13] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[13].dstSet = resources->exrDescriptorSet;
    writes[13].dstBinding = 16;
    writes[13].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[13].descriptorCount = 1;
    writes[13].pBufferInfo = &seepageParamsInfo;
    writes[14] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[14].dstSet = resources->exrDescriptorSet;
    writes[14].dstBinding = 17;
    writes[14].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[14].descriptorCount = 1;
    writes[14].pBufferInfo = &meshFlowContactEventInfo;
    writes[15] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[15].dstSet = resources->exrDescriptorSet;
    writes[15].dstBinding = 18;
    writes[15].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[15].descriptorCount = 1;
    writes[15].pBufferInfo = &meshFlowContactGridInfo;

    vkUpdateDescriptorSets(device_, static_cast<std::uint32_t>(writes.size()), writes.data(), 0, nullptr);
    resources->seepageExrDescriptorGeneration =
        resources->seepageTopologyGeneration;
}

void VulkanViewportShell::CreateOrUpdateCompositeDescriptorSet() {
    compositeDescriptorSets_.resize(accumulationImages_.size(), VK_NULL_HANDLE);
    for (std::uint32_t imageIndex = 0; imageIndex < accumulationImages_.size(); ++imageIndex) {
        CreateOrUpdateCompositeDescriptorSet(
            &compositeDescriptorSets_[imageIndex],
            accumulationImages_[imageIndex].view,
            revealageImages_[imageIndex].view,
            emissiveImages_[imageIndex].view);
    }
}

void VulkanViewportShell::CreateOrUpdateCompositeDescriptorSet(
    VkDescriptorSet* descriptorSet,
    VkImageView accumulationView,
    VkImageView revealageView,
    VkImageView emissiveView) {
    CreateOrUpdateCompositeDescriptorSet(
        descriptorSet,
        accumulationView,
        revealageView,
        emissiveView,
        VK_NULL_HANDLE,
        VK_NULL_HANDLE);
}

void VulkanViewportShell::CreateOrUpdateCompositeDescriptorSet(
    VkDescriptorSet* descriptorSet,
    VkImageView accumulationView,
    VkImageView revealageView,
    VkImageView emissiveView,
    VkImageView normalAccumulationView,
    VkImageView albedoAccumulationView) {
    if (descriptorSet == nullptr) {
        return;
    }

    if (*descriptorSet == VK_NULL_HANDLE) {
        VkDescriptorSetAllocateInfo allocInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
        allocInfo.descriptorPool = descriptorPool_;
        allocInfo.descriptorSetCount = 1;
        allocInfo.pSetLayouts = &compositeDescriptorSetLayout_;
        Check(vkAllocateDescriptorSets(device_, &allocInfo, descriptorSet), "vkAllocateDescriptorSets(composite)");
    }

    VkDescriptorImageInfo accumulationInfo{};
    accumulationInfo.imageView = accumulationView;
    accumulationInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

    VkDescriptorImageInfo revealageInfo{};
    revealageInfo.imageView = revealageView;
    revealageInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

    VkDescriptorImageInfo emissiveInfo{};
    emissiveInfo.imageView = emissiveView;
    emissiveInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

    VkDescriptorImageInfo normalAccumulationInfo{};
    normalAccumulationInfo.imageView = normalAccumulationView;
    normalAccumulationInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

    VkDescriptorImageInfo albedoAccumulationInfo{};
    albedoAccumulationInfo.imageView = albedoAccumulationView;
    albedoAccumulationInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

    std::array<VkWriteDescriptorSet, 5> writes{};
    writes[0] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[0].dstSet = *descriptorSet;
    writes[0].dstBinding = 0;
    writes[0].descriptorType = VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT;
    writes[0].descriptorCount = 1;
    writes[0].pImageInfo = &accumulationInfo;

    writes[1] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[1].dstSet = *descriptorSet;
    writes[1].dstBinding = 1;
    writes[1].descriptorType = VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT;
    writes[1].descriptorCount = 1;
    writes[1].pImageInfo = &revealageInfo;

    writes[2] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[2].dstSet = *descriptorSet;
    writes[2].dstBinding = 2;
    writes[2].descriptorType = VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT;
    writes[2].descriptorCount = 1;
    writes[2].pImageInfo = &emissiveInfo;

    std::uint32_t writeCount = 3U;
    if (normalAccumulationView != VK_NULL_HANDLE && albedoAccumulationView != VK_NULL_HANDLE) {
        writes[3] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
        writes[3].dstSet = *descriptorSet;
        writes[3].dstBinding = 3;
        writes[3].descriptorType = VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT;
        writes[3].descriptorCount = 1;
        writes[3].pImageInfo = &normalAccumulationInfo;

        writes[4] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
        writes[4].dstSet = *descriptorSet;
        writes[4].dstBinding = 4;
        writes[4].descriptorType = VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT;
        writes[4].descriptorCount = 1;
        writes[4].pImageInfo = &albedoAccumulationInfo;
        writeCount = 5U;
    }

    vkUpdateDescriptorSets(device_, writeCount, writes.data(), 0, nullptr);
}

void VulkanViewportShell::CreateOrUpdatePostProcessDescriptorSets() {
    postProcessDescriptorSets_.resize(sceneColorImages_.size(), VK_NULL_HANDLE);
    for (std::uint32_t imageIndex = 0; imageIndex < sceneColorImages_.size(); ++imageIndex) {
        auto& descriptorSet = postProcessDescriptorSets_[imageIndex];
        if (descriptorSet == VK_NULL_HANDLE) {
            VkDescriptorSetAllocateInfo allocInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
            allocInfo.descriptorPool = descriptorPool_;
            allocInfo.descriptorSetCount = 1;
            allocInfo.pSetLayouts = &postProcessDescriptorSetLayout_;
            Check(
                vkAllocateDescriptorSets(device_, &allocInfo, &descriptorSet),
                "vkAllocateDescriptorSets(postprocess)");
        }

        VkDescriptorImageInfo sceneColorInfo{};
        sceneColorInfo.sampler = postProcessSampler_;
        sceneColorInfo.imageView = sceneColorImages_[imageIndex].view;
        sceneColorInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

        VkDescriptorImageInfo linearDepthInfo{};
        linearDepthInfo.sampler = postProcessSampler_;
        linearDepthInfo.imageView = linearDepthImages_[imageIndex].view;
        linearDepthInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

        std::array<VkDescriptorImageInfo, 2U> temporalColorInfos{};
        std::array<VkDescriptorImageInfo, 2U> temporalDepthInfos{};
        const bool temporalResourcesReady =
            temporalCameraOverlayColorImages_[0U].view != VK_NULL_HANDLE &&
            temporalCameraOverlayColorImages_[1U].view != VK_NULL_HANDLE &&
            temporalCameraOverlayDepthImages_[0U].view != VK_NULL_HANDLE &&
            temporalCameraOverlayDepthImages_[1U].view != VK_NULL_HANDLE;
        for (std::size_t sourceIndex = 0U;
             sourceIndex < temporalColorInfos.size();
             ++sourceIndex) {
            temporalColorInfos[sourceIndex].sampler = postProcessSampler_;
            temporalColorInfos[sourceIndex].imageView = temporalResourcesReady
                                                            ? temporalCameraOverlayColorImages_[sourceIndex].view
                                                            : sceneColorImages_[imageIndex].view;
            temporalColorInfos[sourceIndex].imageLayout =
                VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
            temporalDepthInfos[sourceIndex].sampler = postProcessSampler_;
            temporalDepthInfos[sourceIndex].imageView = temporalResourcesReady
                                                            ? temporalCameraOverlayDepthImages_[sourceIndex].view
                                                            : linearDepthImages_[imageIndex].view;
            temporalDepthInfos[sourceIndex].imageLayout =
                VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        }

        std::array<VkWriteDescriptorSet, 6> writes{};
        writes[0] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
        writes[0].dstSet = descriptorSet;
        writes[0].dstBinding = 0;
        writes[0].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        writes[0].descriptorCount = 1;
        writes[0].pImageInfo = &sceneColorInfo;

        writes[1] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
        writes[1].dstSet = descriptorSet;
        writes[1].dstBinding = 1;
        writes[1].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        writes[1].descriptorCount = 1;
        writes[1].pImageInfo = &linearDepthInfo;

        for (std::size_t sourceIndex = 0U;
             sourceIndex < temporalColorInfos.size();
             ++sourceIndex) {
            const std::size_t colorWriteIndex = 2U + sourceIndex * 2U;
            const std::size_t depthWriteIndex = colorWriteIndex + 1U;
            writes[colorWriteIndex] = {
                VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
            writes[colorWriteIndex].dstSet = descriptorSet;
            writes[colorWriteIndex].dstBinding =
                static_cast<std::uint32_t>(colorWriteIndex);
            writes[colorWriteIndex].descriptorType =
                VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
            writes[colorWriteIndex].descriptorCount = 1;
            writes[colorWriteIndex].pImageInfo =
                &temporalColorInfos[sourceIndex];
            writes[depthWriteIndex] = {
                VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
            writes[depthWriteIndex].dstSet = descriptorSet;
            writes[depthWriteIndex].dstBinding =
                static_cast<std::uint32_t>(depthWriteIndex);
            writes[depthWriteIndex].descriptorType =
                VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
            writes[depthWriteIndex].descriptorCount = 1;
            writes[depthWriteIndex].pImageInfo =
                &temporalDepthInfos[sourceIndex];
        }

        vkUpdateDescriptorSets(device_, static_cast<std::uint32_t>(writes.size()), writes.data(), 0, nullptr);
    }
}

void VulkanViewportShell::UpdateGaussianSplatDescriptorSets(ActiveGaussianSplatResources* resources) {
    if (resources == nullptr) {
        return;
    }
    for (std::size_t frameIndex = 0; frameIndex < kFramesInFlight; ++frameIndex) {
        resources->descriptorSets[frameIndex].resize(depthImages_.size(), VK_NULL_HANDLE);
        for (std::uint32_t imageIndex = 0; imageIndex < depthImages_.size(); ++imageIndex) {
            UpdateGaussianSplatDescriptorSet(resources, frameIndex, imageIndex);
        }
    }
}

void VulkanViewportShell::UpdateGaussianSplatDescriptorSet(
    ActiveGaussianSplatResources* resources,
    std::size_t frameIndex,
    std::uint32_t imageIndex) {
    if (resources == nullptr || frameIndex >= kFramesInFlight || imageIndex >= depthImages_.size()) {
        return;
    }

    auto& descriptorSet = resources->descriptorSets[frameIndex][imageIndex];
    if (descriptorSet == VK_NULL_HANDLE) {
        VkDescriptorSetAllocateInfo allocInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
        allocInfo.descriptorPool = gaussianSplatDescriptorPool_;
        allocInfo.descriptorSetCount = 1;
        allocInfo.pSetLayouts = &gaussianSplatDescriptorSetLayout_;
        Check(vkAllocateDescriptorSets(device_, &allocInfo, &descriptorSet), "vkAllocateDescriptorSets(gsplat)");
    }

    VkDescriptorBufferInfo uniformInfo{};
    uniformInfo.buffer = frameResources_[frameIndex].uniformBuffer.buffer;
    uniformInfo.offset = 0;
    uniformInfo.range = sizeof(FrameUniforms);

    VkDescriptorBufferInfo centerInfo{resources->centerBuffer.buffer, 0, resources->centerBuffer.size};
    VkDescriptorBufferInfo scaleInfo{resources->scaleBuffer.buffer, 0, resources->scaleBuffer.size};
    VkDescriptorBufferInfo rotationInfo{resources->rotationBuffer.buffer, 0, resources->rotationBuffer.size};
    VkDescriptorBufferInfo opacityInfo{resources->opacityBuffer.buffer, 0, resources->opacityBuffer.size};
    VkDescriptorBufferInfo shInfo{resources->shBuffer.buffer, 0, resources->shBuffer.size};
    VkDescriptorImageInfo sceneDepthInfo{};
    sceneDepthInfo.imageView = depthImages_[imageIndex].view;
    sceneDepthInfo.imageLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL;

    std::array<VkWriteDescriptorSet, 7> writes{};
    writes[0] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[0].dstSet = descriptorSet;
    writes[0].dstBinding = 0;
    writes[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    writes[0].descriptorCount = 1;
    writes[0].pBufferInfo = &uniformInfo;

    writes[1] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[1].dstSet = descriptorSet;
    writes[1].dstBinding = 1;
    writes[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[1].descriptorCount = 1;
    writes[1].pBufferInfo = &centerInfo;

    writes[2] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[2].dstSet = descriptorSet;
    writes[2].dstBinding = 2;
    writes[2].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[2].descriptorCount = 1;
    writes[2].pBufferInfo = &scaleInfo;

    writes[3] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[3].dstSet = descriptorSet;
    writes[3].dstBinding = 3;
    writes[3].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[3].descriptorCount = 1;
    writes[3].pBufferInfo = &rotationInfo;

    writes[4] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[4].dstSet = descriptorSet;
    writes[4].dstBinding = 4;
    writes[4].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[4].descriptorCount = 1;
    writes[4].pBufferInfo = &opacityInfo;

    writes[5] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[5].dstSet = descriptorSet;
    writes[5].dstBinding = 5;
    writes[5].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[5].descriptorCount = 1;
    writes[5].pBufferInfo = &shInfo;

    writes[6] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[6].dstSet = descriptorSet;
    writes[6].dstBinding = 6;
    writes[6].descriptorType = VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT;
    writes[6].descriptorCount = 1;
    writes[6].pImageInfo = &sceneDepthInfo;

    vkUpdateDescriptorSets(device_, static_cast<std::uint32_t>(writes.size()), writes.data(), 0, nullptr);
}

void VulkanViewportShell::UpdateHighQualityGaussianDescriptorSet(std::size_t frameIndex) {
    auto& scene = highQualityGaussianScene_;
    if (scene.splatCount == 0 || scene.layerCount == 0 || frameIndex >= kFramesInFlight) {
        return;
    }

    auto& descriptorSet = scene.descriptorSets[frameIndex];
    if (descriptorSet == VK_NULL_HANDLE) {
        VkDescriptorSetAllocateInfo allocInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
        allocInfo.descriptorPool = gaussianSplatDescriptorPool_;
        allocInfo.descriptorSetCount = 1;
        allocInfo.pSetLayouts = &highQualityGaussianSplatDescriptorSetLayout_;
        Check(
            vkAllocateDescriptorSets(device_, &allocInfo, &descriptorSet),
            "vkAllocateDescriptorSets(gsplat_hq)");
    }

    VkDescriptorBufferInfo uniformInfo{};
    uniformInfo.buffer = frameResources_[frameIndex].uniformBuffer.buffer;
    uniformInfo.offset = 0;
    uniformInfo.range = sizeof(FrameUniforms);

    VkDescriptorBufferInfo centerInfo{scene.centerBuffer.buffer, 0, scene.centerBuffer.size};
    VkDescriptorBufferInfo scaleInfo{scene.scaleBuffer.buffer, 0, scene.scaleBuffer.size};
    VkDescriptorBufferInfo rotationInfo{scene.rotationBuffer.buffer, 0, scene.rotationBuffer.size};
    VkDescriptorBufferInfo opacityInfo{scene.opacityBuffer.buffer, 0, scene.opacityBuffer.size};
    VkDescriptorBufferInfo shInfo{scene.shBuffer.buffer, 0, scene.shBuffer.size};
    VkDescriptorBufferInfo layerStyleIndexInfo{
        scene.layerStyleIndexBuffer.buffer,
        0,
        scene.layerStyleIndexBuffer.size};
    VkDescriptorBufferInfo layerStyleInfo{
        scene.layerStyleBuffers[frameIndex].buffer,
        0,
        scene.layerStyleBuffers[frameIndex].size};
    VkDescriptorBufferInfo sortedIndexInfo{
        scene.sortedIndexBuffers[frameIndex].buffer,
        0,
        scene.sortedIndexBuffers[frameIndex].size};

    std::array<VkWriteDescriptorSet, 9> writes{};
    writes[0] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[0].dstSet = descriptorSet;
    writes[0].dstBinding = 0;
    writes[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    writes[0].descriptorCount = 1;
    writes[0].pBufferInfo = &uniformInfo;

    writes[1] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[1].dstSet = descriptorSet;
    writes[1].dstBinding = 1;
    writes[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[1].descriptorCount = 1;
    writes[1].pBufferInfo = &centerInfo;

    writes[2] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[2].dstSet = descriptorSet;
    writes[2].dstBinding = 2;
    writes[2].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[2].descriptorCount = 1;
    writes[2].pBufferInfo = &scaleInfo;

    writes[3] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[3].dstSet = descriptorSet;
    writes[3].dstBinding = 3;
    writes[3].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[3].descriptorCount = 1;
    writes[3].pBufferInfo = &rotationInfo;

    writes[4] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[4].dstSet = descriptorSet;
    writes[4].dstBinding = 4;
    writes[4].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[4].descriptorCount = 1;
    writes[4].pBufferInfo = &opacityInfo;

    writes[5] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[5].dstSet = descriptorSet;
    writes[5].dstBinding = 5;
    writes[5].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[5].descriptorCount = 1;
    writes[5].pBufferInfo = &shInfo;

    writes[6] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[6].dstSet = descriptorSet;
    writes[6].dstBinding = 6;
    writes[6].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[6].descriptorCount = 1;
    writes[6].pBufferInfo = &layerStyleIndexInfo;

    writes[7] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[7].dstSet = descriptorSet;
    writes[7].dstBinding = 7;
    writes[7].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[7].descriptorCount = 1;
    writes[7].pBufferInfo = &layerStyleInfo;

    writes[8] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    writes[8].dstSet = descriptorSet;
    writes[8].dstBinding = 8;
    writes[8].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[8].descriptorCount = 1;
    writes[8].pBufferInfo = &sortedIndexInfo;

    vkUpdateDescriptorSets(device_, static_cast<std::uint32_t>(writes.size()), writes.data(), 0, nullptr);
}

void VulkanViewportShell::RefreshHighQualityGaussianScene(std::size_t frameIndex) {
    struct ActiveHighLayer {
        const SceneRenderState::GaussianSplatLayerState* renderLayer = nullptr;
        const ActiveGaussianSplatResources* resources = nullptr;
    };

    std::vector<ActiveHighLayer> activeHighLayers;
    activeHighLayers.reserve(renderState_.gaussianSplatLayers.size());
    const auto retireHighQualityScene = [this]() {
        WaitIdle();
        ResetPointCloudReferencingCommandBuffers();
        CleanupHighQualityGaussianScene();
    };

    for (const auto& layer : renderState_.gaussianSplatLayers) {
        if (layer.style.qualityMode != renderer::gsplat::GaussianSplatQualityMode::High) {
            continue;
        }

        const auto* resources = FindGaussianSplatResources(layer.layerId);
        if (resources == nullptr || resources->splatCount == 0) {
            continue;
        }

        ActiveHighLayer activeLayer;
        activeLayer.renderLayer = &layer;
        activeLayer.resources = resources;
        activeHighLayers.push_back(activeLayer);
    }

    if (activeHighLayers.empty()) {
        const bool hasDescriptor = std::any_of(
            highQualityGaussianScene_.descriptorSets.begin(),
            highQualityGaussianScene_.descriptorSets.end(),
            [](VkDescriptorSet descriptorSet) { return descriptorSet != VK_NULL_HANDLE; });
        if (highQualityGaussianScene_.splatCount > 0 || hasDescriptor) {
            retireHighQualityScene();
        }
        highQualityGaussianSceneDirty_ = false;
        return;
    }

    std::vector<renderer::gsplat::HighQualityGaussianLayerInput> hqLayerInputs;
    hqLayerInputs.reserve(activeHighLayers.size());
    for (const auto& activeLayer : activeHighLayers) {
        renderer::gsplat::HighQualityGaussianLayerInput input;
        input.layerId = activeLayer.renderLayer->layerId;
        input.revision = activeLayer.resources->revision;
        input.splatCount = activeLayer.resources->splatCount;
        input.localToWorld = activeLayer.renderLayer->localToWorld;
        input.transformEnabled = activeLayer.renderLayer->style.transformEnabled;
        hqLayerInputs.push_back(input);
    }

    const auto signatures = renderer::gsplat::BuildHighQualityGaussianLayerSignatures(hqLayerInputs);

    const bool needsRebuild =
        highQualityGaussianSceneDirty_ ||
        highQualityGaussianScene_.splatCount == 0 ||
        !renderer::gsplat::HighQualityGaussianLayerSignaturesMatch(
            highQualityGaussianScene_.layerSignatures,
            signatures);

    if (needsRebuild) {
        retireHighQualityScene();

        std::vector<glm::vec4> mergedCenters;
        std::vector<glm::vec4> mergedScales;
        std::vector<glm::vec4> mergedRotations;
        std::vector<float> mergedOpacities;
        std::vector<float> mergedShCoefficients;
        std::vector<std::uint32_t> mergedLayerStyleIndices;

        highQualityGaussianScene_.layerRanges =
            renderer::gsplat::BuildHighQualityGaussianLayerRanges(hqLayerInputs);

        for (const auto& activeLayer : activeHighLayers) {
            const auto& resources = *activeLayer.resources;
            const auto splatCount = resources.splatCount;

            mergedCenters.reserve(mergedCenters.size() + splatCount);
            for (const auto& center : resources.cpuCenters) {
                mergedCenters.emplace_back(center.x, center.y, center.z, 1.0F);
                highQualityGaussianScene_.mergedLocalCenters.push_back(center);
            }

            mergedScales.reserve(mergedScales.size() + splatCount);
            for (const auto& scale : resources.cpuScales) {
                mergedScales.emplace_back(scale[0], scale[1], scale[2], 0.0F);
            }

            mergedRotations.reserve(mergedRotations.size() + splatCount);
            for (const auto& rotation : resources.cpuRotations) {
                mergedRotations.emplace_back(rotation[0], rotation[1], rotation[2], rotation[3]);
            }

            mergedOpacities.insert(
                mergedOpacities.end(),
                resources.cpuOpacities.begin(),
                resources.cpuOpacities.end());
            mergedShCoefficients.insert(
                mergedShCoefficients.end(),
                resources.cpuShCoefficients.begin(),
                resources.cpuShCoefficients.end());
        }

        highQualityGaussianScene_.layerSignatures = signatures;
        highQualityGaussianScene_.splatCount =
            highQualityGaussianScene_.layerRanges.empty()
                ? 0U
                : highQualityGaussianScene_.layerRanges.back().mergedStart +
                      highQualityGaussianScene_.layerRanges.back().splatCount;
        highQualityGaussianScene_.layerCount = static_cast<std::uint32_t>(highQualityGaussianScene_.layerRanges.size());

        mergedLayerStyleIndices.clear();
        for (const auto& layerRange : highQualityGaussianScene_.layerRanges) {
            mergedLayerStyleIndices.insert(
                mergedLayerStyleIndices.end(),
                layerRange.splatCount,
                layerRange.styleIndex);
        }

        highQualityGaussianScene_.centerBuffer = CreateHostVisibleBuffer(
            static_cast<VkDeviceSize>(mergedCenters.size() * sizeof(glm::vec4)),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        UploadBufferData(
            highQualityGaussianScene_.centerBuffer,
            mergedCenters.data(),
            highQualityGaussianScene_.centerBuffer.size);

        highQualityGaussianScene_.scaleBuffer = CreateHostVisibleBuffer(
            static_cast<VkDeviceSize>(mergedScales.size() * sizeof(glm::vec4)),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        UploadBufferData(
            highQualityGaussianScene_.scaleBuffer,
            mergedScales.data(),
            highQualityGaussianScene_.scaleBuffer.size);

        highQualityGaussianScene_.rotationBuffer = CreateHostVisibleBuffer(
            static_cast<VkDeviceSize>(mergedRotations.size() * sizeof(glm::vec4)),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        UploadBufferData(
            highQualityGaussianScene_.rotationBuffer,
            mergedRotations.data(),
            highQualityGaussianScene_.rotationBuffer.size);

        highQualityGaussianScene_.opacityBuffer = CreateHostVisibleBuffer(
            static_cast<VkDeviceSize>(mergedOpacities.size() * sizeof(float)),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        UploadBufferData(
            highQualityGaussianScene_.opacityBuffer,
            mergedOpacities.data(),
            highQualityGaussianScene_.opacityBuffer.size);

        highQualityGaussianScene_.shBuffer = CreateHostVisibleBuffer(
            static_cast<VkDeviceSize>(mergedShCoefficients.size() * sizeof(float)),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        UploadBufferData(
            highQualityGaussianScene_.shBuffer,
            mergedShCoefficients.data(),
            highQualityGaussianScene_.shBuffer.size);

        highQualityGaussianScene_.layerStyleIndexBuffer = CreateHostVisibleBuffer(
            static_cast<VkDeviceSize>(mergedLayerStyleIndices.size() * sizeof(std::uint32_t)),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        UploadBufferData(
            highQualityGaussianScene_.layerStyleIndexBuffer,
            mergedLayerStyleIndices.data(),
            highQualityGaussianScene_.layerStyleIndexBuffer.size);

        for (std::size_t resourceFrameIndex = 0; resourceFrameIndex < kFramesInFlight; ++resourceFrameIndex) {
            highQualityGaussianScene_.layerStyleBuffers[resourceFrameIndex] = CreateHostVisibleBuffer(
                static_cast<VkDeviceSize>(
                    std::max<std::uint32_t>(1U, highQualityGaussianScene_.layerCount) *
                    sizeof(HighQualityGaussianLayerStyle)),
                VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
            highQualityGaussianScene_.sortedIndexBuffers[resourceFrameIndex] = CreateHostVisibleBuffer(
                static_cast<VkDeviceSize>(
                    std::max<std::uint32_t>(1U, highQualityGaussianScene_.splatCount) * sizeof(std::uint32_t)),
                VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
            UpdateHighQualityGaussianDescriptorSet(resourceFrameIndex);
        }
        highQualityGaussianSceneDirty_ = false;
    }

    if (highQualityGaussianScene_.splatCount == 0 || highQualityGaussianScene_.layerCount == 0) {
        return;
    }

    std::vector<HighQualityGaussianLayerStyle> layerStyles(
        highQualityGaussianScene_.layerCount);
    auto findActiveLayer = [&](std::size_t layerId) -> const ActiveHighLayer* {
        const auto it = std::find_if(
            activeHighLayers.begin(),
            activeHighLayers.end(),
            [layerId](const ActiveHighLayer& activeLayer) {
                return activeLayer.renderLayer->layerId == layerId;
            });
        return it != activeHighLayers.end() ? &(*it) : nullptr;
    };

    for (const auto& layerRange : highQualityGaussianScene_.layerRanges) {
        const auto* activeLayer = findActiveLayer(layerRange.layerId);
        if (activeLayer == nullptr) {
            continue;
        }

        const auto& renderLayer = *activeLayer->renderLayer;
        auto& layerStyle = layerStyles[layerRange.styleIndex];
        layerStyle.localToWorld = renderLayer.localToWorld;
        layerStyle.layerTint = glm::vec4{
            renderLayer.style.layerTint[0],
            renderLayer.style.layerTint[1],
            renderLayer.style.layerTint[2],
            renderLayer.style.layerTint[3]};
        layerStyle.style = glm::vec4{
            renderLayer.style.opacityMultiplier,
            renderLayer.style.scaleMultiplier,
            renderLayer.style.exposure,
            renderLayer.style.saturation};
        layerStyle.control = glm::uvec4{
            static_cast<std::uint32_t>(renderLayer.style.colorMode),
            static_cast<std::uint32_t>(renderLayer.style.debugMode),
            renderLayer.style.transformEnabled ? 1U : 0U,
            0U};

    }

    UploadBufferData(
        highQualityGaussianScene_.layerStyleBuffers[frameIndex],
        layerStyles.data(),
        static_cast<VkDeviceSize>(layerStyles.size() * sizeof(HighQualityGaussianLayerStyle)));

    const bool needsResort =
        !highQualityGaussianScene_.hasSortedView ||
        !MatricesApproximatelyEqual(highQualityGaussianScene_.lastSortedView, renderState_.view);
    if (needsResort) {
        highQualityGaussianScene_.sortedIndices = renderer::gsplat::SortHighQualityGaussianIndices(
            highQualityGaussianScene_.mergedLocalCenters,
            hqLayerInputs,
            highQualityGaussianScene_.layerRanges,
            renderState_.view);
        highQualityGaussianScene_.lastSortedView = renderState_.view;
        highQualityGaussianScene_.hasSortedView = true;
    }
    UploadBufferData(
        highQualityGaussianScene_.sortedIndexBuffers[frameIndex],
        highQualityGaussianScene_.sortedIndices.data(),
        static_cast<VkDeviceSize>(highQualityGaussianScene_.sortedIndices.size() * sizeof(std::uint32_t)));
}

void VulkanViewportShell::CleanupSwapchain() {
    lastSubmittedSceneImageValid_ = false;
    DestroyBuffer(&liveSceneReadbackBuffer_);

    if (commandPool_ != VK_NULL_HANDLE) {
        for (auto& frame : frameResources_) {
            if (frame.commandBuffer != VK_NULL_HANDLE) {
                vkFreeCommandBuffers(device_, commandPool_, 1, &frame.commandBuffer);
                frame.commandBuffer = VK_NULL_HANDLE;
            }
        }
        if (exrExportResources_.commandBuffer != VK_NULL_HANDLE) {
            Check(
                vkResetCommandBuffer(exrExportResources_.commandBuffer, 0U),
                "vkResetCommandBuffer(swapchain point descriptor retirement)");
        }
    }

    // CleanupSwapchain is entered only after the device has settled. Release
    // shared-pool descriptors before the image views they reference instead
    // of quarantining their slots for the lifetime of the application.
    const auto freeSharedDescriptorSets = [&](std::vector<VkDescriptorSet>* descriptorSets) {
        if (descriptorSets == nullptr) {
            return;
        }
        for (auto& descriptorSet : *descriptorSets) {
            if (descriptorSet != VK_NULL_HANDLE && descriptorPool_ != VK_NULL_HANDLE) {
                Check(
                    vkFreeDescriptorSets(device_, descriptorPool_, 1U, &descriptorSet),
                    "vkFreeDescriptorSets(swapchain generation)");
                descriptorSet = VK_NULL_HANDLE;
            }
        }
        descriptorSets->clear();
    };
    freeSharedDescriptorSets(&compositeDescriptorSets_);
    freeSharedDescriptorSets(&postProcessDescriptorSets_);

    for (auto& resources : pointCloudResources_) {
        RetirePointCloudDescriptorSets(&resources.descriptorSets);
        resources.seepageDescriptorGenerations = {};
        for (auto& highlight : resources.highlights) {
            RetirePointCloudDescriptorSets(&highlight.descriptorSets);
            highlight.seepageDescriptorGenerations = {};
        }
        resources.exrDescriptorSet = VK_NULL_HANDLE;
        resources.seepageExrDescriptorGeneration = 0U;
        DestroyPointDescriptorPool(&resources.pointDescriptorPool);
    }
    for (auto& resources : gaussianSplatResources_) {
        for (auto& descriptorSets : resources.descriptorSets) {
            for (auto& descriptorSet : descriptorSets) {
                if (descriptorSet != VK_NULL_HANDLE && gaussianSplatDescriptorPool_ != VK_NULL_HANDLE) {
                    vkFreeDescriptorSets(device_, gaussianSplatDescriptorPool_, 1, &descriptorSet);
                    descriptorSet = VK_NULL_HANDLE;
                }
            }
            descriptorSets.clear();
        }
    }

    for (const auto framebuffer : framebuffers_) {
        vkDestroyFramebuffer(device_, framebuffer, nullptr);
    }
    framebuffers_.clear();
    for (const auto framebuffer : presentFramebuffers_) {
        vkDestroyFramebuffer(device_, framebuffer, nullptr);
    }
    presentFramebuffers_.clear();

    for (auto& image : sceneColorImages_) {
        DestroyImage(&image);
    }
    for (auto& image : depthImages_) {
        DestroyImage(&image);
    }
    for (auto& image : accumulationImages_) {
        DestroyImage(&image);
    }
    for (auto& image : revealageImages_) {
        DestroyImage(&image);
    }
    for (auto& image : emissiveImages_) {
        DestroyImage(&image);
    }
    for (auto& image : linearDepthImages_) {
        DestroyImage(&image);
    }
    sceneColorImages_.clear();
    depthImages_.clear();
    accumulationImages_.clear();
    revealageImages_.clear();
    emissiveImages_.clear();
    linearDepthImages_.clear();
    temporalCameraOverlayResourcesInitialized_ = false;
    temporalCameraOverlayHistoryValid_ = {false, false};

    for (const auto imageView : imageViews_) {
        vkDestroyImageView(device_, imageView, nullptr);
    }
    imageViews_.clear();

    DestroySwapchainRenderFinishedSemaphores();
    if (swapchain_ != VK_NULL_HANDLE) {
        vkDestroySwapchainKHR(device_, swapchain_, nullptr);
        swapchain_ = VK_NULL_HANDLE;
    }

    swapchainImages_.clear();
    swapchainImageOwners_.clear();
}

void VulkanViewportShell::CleanupPointCloudResources(ActivePointCloudResources* resources) {
    if (resources == nullptr) {
        return;
    }

    SettlePointCloudMutation();
    DestroyPointCloudResources(resources);
}

void VulkanViewportShell::RetirePointCloudRenderDescriptors(
    ActivePointCloudResources* resources) {
    if (resources == nullptr) {
        return;
    }
    RetirePointCloudDescriptorSets(&resources->descriptorSets);
    resources->seepageDescriptorGenerations = {};
    for (auto& highlight : resources->highlights) {
        RetirePointCloudDescriptorSets(&highlight.descriptorSets);
        highlight.seepageDescriptorGenerations = {};
    }
    resources->exrDescriptorSet = VK_NULL_HANDLE;
    resources->seepageExrDescriptorGeneration = 0U;
    DestroyPointDescriptorPool(&resources->pointDescriptorPool);
}

void VulkanViewportShell::DestroyPointCloudResources(ActivePointCloudResources* resources) {
    if (resources == nullptr) {
        return;
    }

    RetirePointCloudRenderDescriptors(resources);
    if (resources->depthSortDescriptorPool != VK_NULL_HANDLE) {
        vkDestroyDescriptorPool(
            device_,
            resources->depthSortDescriptorPool,
            nullptr);
        resources->depthSortDescriptorPool = VK_NULL_HANDLE;
        resources->depthSortDescriptorSet = VK_NULL_HANDLE;
    }

    for (std::size_t liveSlot = 0; liveSlot < kDynamicMeshFlowLiveSlots; ++liveSlot) {
        auto& fence = resources->dynamicMeshFlowDispatchFences[liveSlot];
        if (fence != VK_NULL_HANDLE && device_ != VK_NULL_HANDLE) {
            const VkResult status = vkGetFenceStatus(device_, fence);
            if (status == VK_NOT_READY) {
                vkWaitForFences(device_, 1, &fence, VK_TRUE, UINT64_MAX);
            }
        }
        auto& commandBuffer = resources->dynamicMeshFlowCommandBuffers[liveSlot];
        if (commandBuffer != VK_NULL_HANDLE &&
            commandPool_ != VK_NULL_HANDLE &&
            device_ != VK_NULL_HANDLE) {
            vkFreeCommandBuffers(device_, commandPool_, 1, &commandBuffer);
            commandBuffer = VK_NULL_HANDLE;
        }
        if (fence != VK_NULL_HANDLE && device_ != VK_NULL_HANDLE) {
            vkDestroyFence(device_, fence, nullptr);
            fence = VK_NULL_HANDLE;
        }
        auto& timestampQueryPool =
            resources->dynamicMeshFlowTimestampQueryPools[liveSlot];
        if (timestampQueryPool != VK_NULL_HANDLE &&
            device_ != VK_NULL_HANDLE) {
            vkDestroyQueryPool(device_, timestampQueryPool, nullptr);
            timestampQueryPool = VK_NULL_HANDLE;
        }
        resources->dynamicMeshFlowTimestampPending[liveSlot] = false;
    }
    RetireWaterFlowSourceComputeGeneration(resources);

    // Descriptor sets retain references to every point-cloud and water-effect
    // buffer. Retire them before releasing any of those allocations; doing the
    // reverse leaves MoltenVK with stale buffer bookkeeping during a later
    // descriptor allocation/update.
    if (resources->dynamicMeshFlowDescriptorPool != VK_NULL_HANDLE) {
        vkDestroyDescriptorPool(
            device_,
            resources->dynamicMeshFlowDescriptorPool,
            nullptr);
        resources->dynamicMeshFlowDescriptorPool = VK_NULL_HANDLE;
        resources->dynamicMeshFlowDescriptorSets.fill(VK_NULL_HANDLE);
    } else {
        for (auto& descriptorSet : resources->dynamicMeshFlowDescriptorSets) {
            if (descriptorSet != VK_NULL_HANDLE &&
                descriptorPool_ != VK_NULL_HANDLE) {
                Check(
                    vkFreeDescriptorSets(
                        device_,
                        descriptorPool_,
                        1U,
                        &descriptorSet),
                    "vkFreeDescriptorSets(dynamic mesh flow retirement)");
                descriptorSet = VK_NULL_HANDLE;
            }
        }
    }

    for (auto& highlight : resources->highlights) {
        CleanupPointHighlightResources(&highlight);
    }
    resources->highlights.clear();

    DestroyBuffer(&resources->positionBuffer);
    DestroyBuffer(&resources->positionStorageBuffer);
    DestroyBuffer(&resources->colorBuffer);
    DestroyBuffer(&resources->normalBuffer);
    DestroyBuffer(&resources->scalarFieldBuffer);
    DestroyBuffer(&resources->seepageNodeBuffer);
    for (auto& paramsBuffer : resources->seepageParamsBuffers) {
        DestroyBuffer(&paramsBuffer);
    }
    DestroyBuffer(&resources->seepageExrParamsBuffer);
    DestroyBuffer(&resources->seepageHashCellBuffer);
    DestroyBuffer(&resources->seepageNodeReferenceBuffer);
    for (auto& uniformBuffer : resources->dynamicMeshFlowUniformBuffers) {
        DestroyBuffer(&uniformBuffer);
    }
    DestroyBuffer(&resources->dynamicMeshFlowParticleBuffer);
    DestroyBuffer(&resources->dynamicMeshFlowContactEventBuffer);
    DestroyBuffer(&resources->dynamicMeshFlowContactGridBuffer);
    DestroyBuffer(&resources->dynamicMeshFlowCellBuffer);
    DestroyBuffer(&resources->dynamicMeshFlowGridBuffer);
    for (auto& emitterBuffer : resources->dynamicMeshFlowEmitterBuffers) {
        DestroyBuffer(&emitterBuffer);
    }
    for (auto& attractorBuffer : resources->dynamicMeshFlowAttractorBuffers) {
        DestroyBuffer(&attractorBuffer);
    }
    for (auto& counterBuffer : resources->dynamicMeshFlowCounterBuffers) {
        DestroyBuffer(&counterBuffer);
    }
    DestroyBuffer(&resources->waterFlowSourceUniformBuffer);
    DestroyBuffer(&resources->waterFlowSourceInputBuffer);
    DestroyBuffer(&resources->waterFlowSourceBranchBuffer);
    DestroyBuffer(&resources->waterFlowPendingPositionBuffer);
    DestroyBuffer(&resources->waterFlowPendingPositionStorageBuffer);
    DestroyBuffer(&resources->waterFlowPendingColorBuffer);
    DestroyBuffer(&resources->waterFlowPendingNormalBuffer);
    DestroyBuffer(&resources->waterFlowPendingScalarFieldBuffer);
    for (auto& retired : resources->waterFlowRetiredOutputs) {
        DestroyBuffer(&retired.positionBuffer);
        DestroyBuffer(&retired.positionStorageBuffer);
        DestroyBuffer(&retired.colorBuffer);
        DestroyBuffer(&retired.normalBuffer);
        DestroyBuffer(&retired.scalarFieldBuffer);
    }
    resources->waterFlowRetiredOutputs.clear();
    for (auto& styleBuffer : resources->styleBuffers) {
        DestroyBuffer(&styleBuffer);
    }
    DestroyBuffer(&resources->exrStyleBuffer);
    DestroyBuffer(&resources->sampledIndexBuffer);
    DestroyBuffer(&resources->sampledSurfelIndexBuffer);
    DestroyBuffer(&resources->interactiveSampledIndexBuffer);
    DestroyBuffer(&resources->interactiveSurfelIndexBuffer);
    DestroyBuffer(&resources->depthSortedIndexBuffer);
    DestroyBuffer(&resources->depthSortScratchBuffer);
    *resources = ActivePointCloudResources{};
}

void VulkanViewportShell::CleanupPointHighlightResources(
    ActivePointCloudResources::PointHighlightResources* highlight) {
    if (highlight == nullptr) {
        return;
    }

    RetirePointCloudDescriptorSets(&highlight->descriptorSets);
    highlight->seepageDescriptorGenerations = {};
    DestroyBuffer(&highlight->indexBuffer);
    DestroyBuffer(&highlight->surfelIndexBuffer);
    for (auto& styleBuffer : highlight->styleBuffers) {
        DestroyBuffer(&styleBuffer);
    }
    *highlight = ActivePointCloudResources::PointHighlightResources{};
}

void VulkanViewportShell::CleanupGaussianSplatResources(ActiveGaussianSplatResources* resources) {
    if (resources == nullptr) {
        return;
    }

    for (auto& descriptorSets : resources->descriptorSets) {
        for (auto& descriptorSet : descriptorSets) {
            if (descriptorSet != VK_NULL_HANDLE &&
                gaussianSplatDescriptorPool_ != VK_NULL_HANDLE &&
                device_ != VK_NULL_HANDLE) {
                vkFreeDescriptorSets(device_, gaussianSplatDescriptorPool_, 1, &descriptorSet);
            }
        }
        descriptorSets.clear();
    }

    DestroyBuffer(&resources->centerBuffer);
    DestroyBuffer(&resources->scaleBuffer);
    DestroyBuffer(&resources->rotationBuffer);
    DestroyBuffer(&resources->opacityBuffer);
    DestroyBuffer(&resources->shBuffer);

    *resources = ActiveGaussianSplatResources{};
}

void VulkanViewportShell::CleanupHighQualityGaussianScene() {
    for (auto& descriptorSet : highQualityGaussianScene_.descriptorSets) {
        if (descriptorSet != VK_NULL_HANDLE &&
            gaussianSplatDescriptorPool_ != VK_NULL_HANDLE &&
            device_ != VK_NULL_HANDLE) {
            vkFreeDescriptorSets(device_, gaussianSplatDescriptorPool_, 1, &descriptorSet);
        }
    }

    DestroyBuffer(&highQualityGaussianScene_.centerBuffer);
    DestroyBuffer(&highQualityGaussianScene_.scaleBuffer);
    DestroyBuffer(&highQualityGaussianScene_.rotationBuffer);
    DestroyBuffer(&highQualityGaussianScene_.opacityBuffer);
    DestroyBuffer(&highQualityGaussianScene_.shBuffer);
    DestroyBuffer(&highQualityGaussianScene_.layerStyleIndexBuffer);
    for (auto& buffer : highQualityGaussianScene_.layerStyleBuffers) {
      DestroyBuffer(&buffer);
    }
    for (auto &buffer : highQualityGaussianScene_.sortedIndexBuffers) {
      DestroyBuffer(&buffer);
    }

    highQualityGaussianScene_ = HighQualityGaussianSceneResources{};
}

void VulkanViewportShell::CleanupExrExportResources() {
    CancelPointCloudExrFrame();
    DestroyExrExportResources(&exrExportResources_);
}

void VulkanViewportShell::DestroyExrExportResources(ExrExportResources* resources) {
    if (resources == nullptr) {
        return;
    }

    if (resources->commandBuffer != VK_NULL_HANDLE && commandPool_ != VK_NULL_HANDLE) {
        vkFreeCommandBuffers(device_, commandPool_, 1, &resources->commandBuffer);
        resources->commandBuffer = VK_NULL_HANDLE;
    }

    // The composite descriptor is generation-owned and must be retired before
    // the image views it binds. Point EXR descriptors are owned by their point
    // pools and are retired by the caller before an installed EXR generation.
    resources->eyeDomeLightingDescriptorSet = VK_NULL_HANDLE;
    if (resources->eyeDomeLightingDescriptorPool != VK_NULL_HANDLE) {
        vkDestroyDescriptorPool(
            device_,
            resources->eyeDomeLightingDescriptorPool,
            nullptr);
        resources->eyeDomeLightingDescriptorPool = VK_NULL_HANDLE;
    }
    resources->compositeDescriptorSet = VK_NULL_HANDLE;
    if (resources->compositeDescriptorPool != VK_NULL_HANDLE) {
        vkDestroyDescriptorPool(device_, resources->compositeDescriptorPool, nullptr);
        resources->compositeDescriptorPool = VK_NULL_HANDLE;
    }
    if (resources->fence != VK_NULL_HANDLE) {
        vkDestroyFence(device_, resources->fence, nullptr);
    }
    if (resources->framebuffer != VK_NULL_HANDLE) {
        vkDestroyFramebuffer(device_, resources->framebuffer, nullptr);
    }
    if (resources->pointDepthPipeline != VK_NULL_HANDLE) {
        vkDestroyPipeline(device_, resources->pointDepthPipeline, nullptr);
    }
    if (resources->pointAccumulationPipeline != VK_NULL_HANDLE) {
        vkDestroyPipeline(device_, resources->pointAccumulationPipeline, nullptr);
    }
    if (resources->pointConstantSimpleAccumulationPipeline != VK_NULL_HANDLE) {
        vkDestroyPipeline(device_, resources->pointConstantSimpleAccumulationPipeline, nullptr);
    }
    if (resources->pointFastBasicDepthPipeline != VK_NULL_HANDLE) {
        vkDestroyPipeline(device_, resources->pointFastBasicDepthPipeline, nullptr);
    }
    if (resources->pointFastBasicPipeline != VK_NULL_HANDLE) {
        vkDestroyPipeline(device_, resources->pointFastBasicPipeline, nullptr);
    }
    if (resources->pointSortedDepthAovPipeline != VK_NULL_HANDLE) {
        vkDestroyPipeline(
            device_,
            resources->pointSortedDepthAovPipeline,
            nullptr);
    }
    if (resources->pointSortedAlphaPipeline != VK_NULL_HANDLE) {
        vkDestroyPipeline(device_, resources->pointSortedAlphaPipeline, nullptr);
    }
    if (resources->pointSortedAlphaHybridPipeline != VK_NULL_HANDLE) {
        vkDestroyPipeline(
            device_,
            resources->pointSortedAlphaHybridPipeline,
            nullptr);
    }
    if (resources->pointSortedEmissionCompositePipeline != VK_NULL_HANDLE) {
        vkDestroyPipeline(
            device_,
            resources->pointSortedEmissionCompositePipeline,
            nullptr);
    }
    if (resources->surfelDepthPipeline != VK_NULL_HANDLE) {
        vkDestroyPipeline(device_, resources->surfelDepthPipeline, nullptr);
    }
    if (resources->surfelAccumulationPipeline != VK_NULL_HANDLE) {
        vkDestroyPipeline(device_, resources->surfelAccumulationPipeline, nullptr);
    }
    if (resources->surfelConstantSimpleAccumulationPipeline != VK_NULL_HANDLE) {
        vkDestroyPipeline(device_, resources->surfelConstantSimpleAccumulationPipeline, nullptr);
    }
    if (resources->rainPipeline != VK_NULL_HANDLE) {
        vkDestroyPipeline(device_, resources->rainPipeline, nullptr);
    }
    if (resources->compositePipeline != VK_NULL_HANDLE) {
        vkDestroyPipeline(device_, resources->compositePipeline, nullptr);
    }
    if (resources->renderPass != VK_NULL_HANDLE) {
        vkDestroyRenderPass(device_, resources->renderPass, nullptr);
    }

    DestroyImage(&resources->colorImage);
    DestroyImage(&resources->depthImage);
    DestroyImage(&resources->accumulationImage);
    DestroyImage(&resources->revealageImage);
    DestroyImage(&resources->emissiveImage);
    DestroyImage(&resources->normalAccumulationImage);
    DestroyImage(&resources->albedoAccumulationImage);
    DestroyImage(&resources->linearDepthImage);
    DestroyImage(&resources->normalImage);
    DestroyImage(&resources->albedoImage);
    DestroyBuffer(&resources->colorReadbackBuffer);
    DestroyBuffer(&resources->depthReadbackBuffer);
    DestroyBuffer(&resources->normalReadbackBuffer);
    DestroyBuffer(&resources->albedoReadbackBuffer);
    DestroyBuffer(&resources->uniformBuffer);

    *resources = ExrExportResources{};
}

void VulkanViewportShell::RecreateSwapchain() {
    int width = 0;
    int height = 0;
    glfwGetFramebufferSize(window_, &width, &height);
    if (width <= 0 || height <= 0) {
        return;
    }

    vkDeviceWaitIdle(device_);
    for (auto& frame : frameResources_) {
        if (frame.activeSubmissionSerial != 0U) {
            frame.completedSubmissionSerial =
                frame.activeSubmissionSerial;
        }
        frame.activeSubmissionSerial = 0U;
        frame.timestampPending = false;
        frame.timestampActiveMask = 0U;
    }
    diagnostics_.gpuTimestampResultsAvailable = false;

    CleanupSwapchain();
    CreateSwapchain();
    CreateImageViews();
    CreateSceneColorResources();
    CreateDepthResources();
    CreateAccumulationResources();
    CreateLinearDepthResources();
    if (temporalCameraOverlayEnabled_) {
        CreateTemporalCameraOverlayResources();
    }
    CreateFramebuffers();
    CreatePresentFramebuffers();
    CreateCommandBuffers();
    CreateOrUpdateCompositeDescriptorSet();
    CreateOrUpdatePostProcessDescriptorSets();
    BeginPointCloudMutationBatch();
    try {
        for (auto& resources : pointCloudResources_) {
            ReplacePointCloudDescriptorSets(&resources);
        }
        EndPointCloudMutationBatch();
    } catch (...) {
        EndPointCloudMutationBatch();
        throw;
    }
    for (auto& resources : gaussianSplatResources_) {
        UpdateGaussianSplatDescriptorSets(&resources);
    }

    if (ImGui::GetCurrentContext() != nullptr) {
        ImGui_ImplVulkan_SetMinImageCount(static_cast<int>(std::max<std::size_t>(2, swapchainImages_.size())));
    }
}

bool VulkanViewportShell::ResolvePointCloudDrawPlan(const SceneRenderState::PointCloudLayerState& layer,
                                                    bool forceFullSource, bool requireLiveDescriptors,
                                                    PointCloudDrawPlan* plan) {
    if (plan == nullptr) {
        return false;
    }
    *plan = {};

    auto* resources = FindPointCloudResources(layer.layerId);
    if (resources == nullptr || resources->activePointCount == 0) {
        return false;
    }
    // The live per-frame descriptor array tracks the swapchain, which can be
    // empty while the window is hidden or the live view is suspended during a
    // long export. The EXR pass binds its own exrDescriptorSet (checked at
    // draw time), so only live recording may treat an empty array as
    // "not drawable" — otherwise an export silently drops every layer.
    if (requireLiveDescriptors && resources->descriptorSets[0].empty()) {
        return false;
    }
    if (resources->dynamicMeshFlowAllocationRevision != 0U &&
        (!rainResources_.collisionReady ||
         resources->dynamicMeshFlowGroundUploadRevision !=
             rainResources_.groundResidentUploadRevision ||
         resources->dynamicMeshFlowGroundIdentity !=
             rainResources_.collisionCacheIdentity)) {
        // The old simulation remains resident until a replacement owner has
        // installed successfully, but it must never draw against the newly
        // active scene/cache identity.
        return false;
    }

    std::uint32_t drawPointCount = 0;
    if (forceFullSource) {
        drawPointCount =
            resources->waterFlowSourceActive ? resources->waterFlowSettledPointCount : resources->pointCount;
    } else {
        drawPointCount = layer.drawPointCount > 0 ? std::min(layer.drawPointCount, resources->activePointCount)
                                                  : resources->activePointCount;
    }
    if (drawPointCount == 0) {
        return false;
    }

    const bool worldSurfels = layer.style.geometryMode != renderer::pointcloud::PointCloudGeometryMode::ScreenSprites;
    if (worldSurfels) {
        drawPointCount = std::min(drawPointCount, kMaxSurfelEncodedPointCount);
        if (drawPointCount == 0) {
            return false;
        }
    }

    const bool sampledBudgetReady = !forceFullSource && resources->usingSampledIndices &&
                                    resources->sampledIndexBuffer.buffer != VK_NULL_HANDLE &&
                                    (!worldSurfels || resources->sampledSurfelIndexBuffer.buffer != VK_NULL_HANDLE);
    const bool interactiveSampleReady =
        !forceFullSource &&
        !sampledBudgetReady &&
        drawPointCount < resources->activePointCount &&
        resources->interactiveSampledIndexBuffer.buffer != VK_NULL_HANDLE &&
        (!worldSurfels || resources->interactiveSurfelIndexBuffer.buffer != VK_NULL_HANDLE) &&
        resources->interactiveSampledIndexCount == drawPointCount;
    if (!forceFullSource &&
        !sampledBudgetReady &&
        drawPointCount < resources->activePointCount &&
        !interactiveSampleReady) {
        drawPointCount = resources->activePointCount;
    }
    if (worldSurfels) {
        drawPointCount = std::min(drawPointCount, kMaxSurfelEncodedPointCount);
        if (drawPointCount == 0) {
            return false;
        }
    }

    plan->resources = resources;
    plan->drawPointCount = drawPointCount;
    plan->worldSurfels = worldSurfels;
    plan->sampledBudgetReady = sampledBudgetReady;
    plan->interactiveSampleReady = interactiveSampleReady;
    return true;
}

void VulkanViewportShell::UpdatePointDepthSortDescriptorSet(
    ActivePointCloudResources* resources) {
    if (resources == nullptr ||
        resources->positionStorageBuffer.buffer == VK_NULL_HANDLE ||
        resources->depthSortedIndexBuffer.buffer == VK_NULL_HANDLE ||
        resources->depthSortScratchBuffer.buffer == VK_NULL_HANDLE) {
        return;
    }
    resources->depthSortCacheValid = false;
    if (resources->depthSortDescriptorPool == VK_NULL_HANDLE) {
        const VkDescriptorPoolSize poolSize{
            VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
            5U};
        VkDescriptorPoolCreateInfo poolInfo{
            VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
        poolInfo.maxSets = 1U;
        poolInfo.poolSizeCount = 1U;
        poolInfo.pPoolSizes = &poolSize;
        Check(
            vkCreateDescriptorPool(
                device_,
                &poolInfo,
                nullptr,
                &resources->depthSortDescriptorPool),
            "vkCreateDescriptorPool(point depth sort)");
    }
    if (resources->depthSortDescriptorSet == VK_NULL_HANDLE) {
        VkDescriptorSetAllocateInfo allocateInfo{
            VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
        allocateInfo.descriptorPool = resources->depthSortDescriptorPool;
        allocateInfo.descriptorSetCount = 1U;
        allocateInfo.pSetLayouts = &pointDepthSortDescriptorSetLayout_;
        Check(
            vkAllocateDescriptorSets(
                device_,
                &allocateInfo,
                &resources->depthSortDescriptorSet),
            "vkAllocateDescriptorSets(point depth sort)");
    }

    const auto& budgetSource =
        resources->sampledIndexBuffer.buffer != VK_NULL_HANDLE
            ? resources->sampledIndexBuffer
            : resources->depthSortedIndexBuffer;
    const auto& interactiveSource =
        resources->interactiveSampledIndexBuffer.buffer != VK_NULL_HANDLE
            ? resources->interactiveSampledIndexBuffer
            : resources->depthSortedIndexBuffer;
    const std::array<VkDescriptorBufferInfo, 5> infos = {
        VkDescriptorBufferInfo{
            resources->positionStorageBuffer.buffer,
            0U,
            resources->positionStorageBuffer.size},
        VkDescriptorBufferInfo{
            budgetSource.buffer,
            0U,
            budgetSource.size},
        VkDescriptorBufferInfo{
            interactiveSource.buffer,
            0U,
            interactiveSource.size},
        VkDescriptorBufferInfo{
            resources->depthSortedIndexBuffer.buffer,
            0U,
            resources->depthSortedIndexBuffer.size},
        VkDescriptorBufferInfo{
            resources->depthSortScratchBuffer.buffer,
            0U,
            resources->depthSortScratchBuffer.size},
    };
    std::array<VkWriteDescriptorSet, 5> writes{};
    for (std::uint32_t binding = 0U; binding < writes.size(); ++binding) {
        writes[binding] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
        writes[binding].dstSet = resources->depthSortDescriptorSet;
        writes[binding].dstBinding = binding;
        writes[binding].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        writes[binding].descriptorCount = 1U;
        writes[binding].pBufferInfo = &infos[binding];
    }
    vkUpdateDescriptorSets(
        device_,
        static_cast<std::uint32_t>(writes.size()),
        writes.data(),
        0U,
        nullptr);
}

void VulkanViewportShell::EnsurePointDepthSortResources(
    ActivePointCloudResources* resources,
    std::uint32_t requiredPointCount) {
    if (resources == nullptr || resources->pointCount == 0U ||
        requiredPointCount == 0U) {
        return;
    }
    requiredPointCount =
        std::min(requiredPointCount, resources->pointCount);
    const VkDeviceSize requiredIndexBytes =
        static_cast<VkDeviceSize>(requiredPointCount) *
        sizeof(std::uint32_t);
    const VkDeviceSize requiredScratchBytes =
        static_cast<VkDeviceSize>(kPointDepthSortBucketCount) * 2U *
        sizeof(std::uint32_t);
    if (resources->depthSortedIndexBuffer.buffer != VK_NULL_HANDLE &&
        resources->depthSortedIndexBuffer.size >= requiredIndexBytes &&
        resources->depthSortScratchBuffer.buffer != VK_NULL_HANDLE &&
        resources->depthSortScratchBuffer.size >= requiredScratchBytes &&
        resources->depthSortDescriptorSet != VK_NULL_HANDLE) {
        return;
    }
    if (resources->depthSortedIndexBuffer.buffer != VK_NULL_HANDLE ||
        resources->depthSortScratchBuffer.buffer != VK_NULL_HANDLE ||
        resources->depthSortDescriptorPool != VK_NULL_HANDLE) {
        SettlePointCloudMutation();
    }
    if (resources->depthSortDescriptorPool != VK_NULL_HANDLE) {
        vkDestroyDescriptorPool(
            device_,
            resources->depthSortDescriptorPool,
            nullptr);
        resources->depthSortDescriptorPool = VK_NULL_HANDLE;
        resources->depthSortDescriptorSet = VK_NULL_HANDLE;
    }
    DestroyBuffer(&resources->depthSortedIndexBuffer);
    DestroyBuffer(&resources->depthSortScratchBuffer);
    resources->depthSortedIndexBuffer = CreateDeviceLocalBuffer(
        requiredIndexBytes,
        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
            VK_BUFFER_USAGE_INDEX_BUFFER_BIT);
    resources->depthSortScratchBuffer = CreateDeviceLocalBuffer(
        requiredScratchBytes,
        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
            VK_BUFFER_USAGE_TRANSFER_DST_BIT);
    UpdatePointDepthSortDescriptorSet(resources);
    TrackPointCloudResidentPeak();
}

bool VulkanViewportShell::RecordPointDepthSort(
    VkCommandBuffer commandBuffer,
    const SceneRenderState::PointCloudLayerState& layer,
    bool forceFullSource) {
    PointCloudDrawPlan plan;
    if (commandBuffer == VK_NULL_HANDLE ||
        pointDepthSortPipeline_ == VK_NULL_HANDLE ||
        pointDepthSortPipelineLayout_ == VK_NULL_HANDLE ||
        layer.style.geometryMode !=
            renderer::pointcloud::PointCloudGeometryMode::ScreenSprites ||
        !layer.style.gpuBackToFrontSorting ||
        !ResolvePointCloudDrawPlan(
            layer,
            forceFullSource,
            false,
            &plan)) {
        return false;
    }
    auto* resources = plan.resources;
    if (resources == nullptr ||
        resources->depthSortDescriptorSet == VK_NULL_HANDLE ||
        resources->depthSortedIndexBuffer.buffer == VK_NULL_HANDLE ||
        resources->depthSortScratchBuffer.buffer == VK_NULL_HANDLE) {
        if (resources != nullptr) {
            // A sorting layer whose dispatch cannot be recorded must not let
            // the draw bind a stale (or never-written) sorted index buffer
            // for a different draw count.
            resources->depthSortCacheValid = false;
        }
        return false;
    }

    const bool usesFixedVertical =
        layer.style.gpuSortMode ==
            renderer::pointcloud::PointCloudGpuSortMode::FixedVertical &&
        resources->worldBoundsValid &&
        resources->worldBoundsMaxZ - resources->worldBoundsMinZ > 1.0e-4F;
    const bool usesAnimationReference =
        !usesFixedVertical &&
        layer.depthSortViewValid &&
        layer.style.gpuSortMode !=
            renderer::pointcloud::PointCloudGpuSortMode::PerFrame;
    glm::mat4 sortView =
        usesAnimationReference ? layer.depthSortView : renderState_.view;
    float sortNearPlane = std::max(
        1.0e-5F,
        usesAnimationReference
            ? layer.depthSortNearPlane
            : renderState_.nearPlane);
    float sortFarPlane = std::max(
        sortNearPlane + 1.0e-5F,
        usesAnimationReference
            ? layer.depthSortFarPlane
            : renderState_.farPlane);
    if (usesFixedVertical) {
        // Camera-independent straight-down axis: depth is metres below the
        // cloud's top, with a one-metre margin so logarithmic buckets stay
        // fine across the whole occupied range. The identity never changes
        // with the camera, so a pan or gentle orbit reuses one cached
        // ordering for the entire session.
        constexpr float kVerticalMarginMeters = 1.0F;
        sortView = glm::mat4{1.0F};
        sortView[0U][2U] = 0.0F;
        sortView[1U][2U] = 0.0F;
        sortView[2U][2U] = 1.0F;
        sortView[3U][2U] =
            -(resources->worldBoundsMaxZ + kVerticalMarginMeters);
        sortNearPlane = kVerticalMarginMeters;
        sortFarPlane = kVerticalMarginMeters +
                       (resources->worldBoundsMaxZ -
                        resources->worldBoundsMinZ);
    }
    sortView = QuantizePointDepthSortView(
        sortView,
        sortNearPlane,
        sortFarPlane);
    const std::uint32_t sourceMode = plan.sampledBudgetReady
                                         ? 1U
                                         : (plan.interactiveSampleReady
                                                ? 2U
                                                : 0U);
    const auto sortKey = PointDepthSortCacheKey(
        sortView,
        sortNearPlane,
        sortFarPlane,
        plan.drawPointCount,
        sourceMode,
        resources->dynamicMeshFlowDispatchCount);
    if (resources->depthSortCacheValid &&
        resources->depthSortCacheKey == sortKey) {
        ++diagnostics_.pointDepthSortCacheReuseCount;
        return true;
    }

    ++diagnostics_.pointDepthSortDispatchCount;

    vkCmdFillBuffer(
        commandBuffer,
        resources->depthSortScratchBuffer.buffer,
        0U,
        resources->depthSortScratchBuffer.size,
        0U);
    std::array<VkBufferMemoryBarrier, 2> beginBarriers{};
    beginBarriers[0] = {VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER};
    beginBarriers[0].srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
    beginBarriers[0].dstAccessMask =
        VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT;
    beginBarriers[0].srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    beginBarriers[0].dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    beginBarriers[0].buffer = resources->depthSortScratchBuffer.buffer;
    beginBarriers[0].offset = 0U;
    beginBarriers[0].size = VK_WHOLE_SIZE;
    beginBarriers[1] = {VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER};
    beginBarriers[1].srcAccessMask = VK_ACCESS_INDEX_READ_BIT;
    beginBarriers[1].dstAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
    beginBarriers[1].srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    beginBarriers[1].dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    beginBarriers[1].buffer = resources->depthSortedIndexBuffer.buffer;
    beginBarriers[1].offset = 0U;
    beginBarriers[1].size = VK_WHOLE_SIZE;
    vkCmdPipelineBarrier(
        commandBuffer,
        VK_PIPELINE_STAGE_TRANSFER_BIT |
            VK_PIPELINE_STAGE_VERTEX_INPUT_BIT,
        VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
        0U,
        0U,
        nullptr,
        static_cast<std::uint32_t>(beginBarriers.size()),
        beginBarriers.data(),
        0U,
        nullptr);

    vkCmdBindPipeline(
        commandBuffer,
        VK_PIPELINE_BIND_POINT_COMPUTE,
        pointDepthSortPipeline_);
    vkCmdBindDescriptorSets(
        commandBuffer,
        VK_PIPELINE_BIND_POINT_COMPUTE,
        pointDepthSortPipelineLayout_,
        0U,
        1U,
        &resources->depthSortDescriptorSet,
        0U,
        nullptr);
    PointDepthSortPushConstants pushConstants{};
    pushConstants.view = sortView;
    pushConstants.control = glm::uvec4{
        0U,
        plan.drawPointCount,
        sourceMode,
        kPointDepthSortBucketCount,
    };
    pushConstants.depth = glm::vec4{
        sortNearPlane,
        sortFarPlane,
        0.0F,
        0.0F,
    };
    const auto dispatchPhase = [&](std::uint32_t phase,
                                   std::uint32_t groupCount) {
        pushConstants.control.x = phase;
        vkCmdPushConstants(
            commandBuffer,
            pointDepthSortPipelineLayout_,
            VK_SHADER_STAGE_COMPUTE_BIT,
            0U,
            sizeof(pushConstants),
            &pushConstants);
        const std::uint32_t groupCountX =
            std::min(groupCount, kPointDepthSortDispatchGroupsX);
        const std::uint32_t groupCountY =
            (groupCount + groupCountX - 1U) / groupCountX;
        vkCmdDispatch(commandBuffer, groupCountX, groupCountY, 1U);
    };
    const std::uint32_t pointGroups =
        (plan.drawPointCount + kPointDepthSortWorkgroupSize - 1U) /
        kPointDepthSortWorkgroupSize;
    dispatchPhase(0U, pointGroups);

    VkMemoryBarrier computeBarrier{VK_STRUCTURE_TYPE_MEMORY_BARRIER};
    computeBarrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
    computeBarrier.dstAccessMask =
        VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT;
    vkCmdPipelineBarrier(
        commandBuffer,
        VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
        VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
        0U,
        1U,
        &computeBarrier,
        0U,
        nullptr,
        0U,
        nullptr);
    dispatchPhase(1U, 1U);
    vkCmdPipelineBarrier(
        commandBuffer,
        VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
        VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
        0U,
        1U,
        &computeBarrier,
        0U,
        nullptr,
        0U,
        nullptr);
    dispatchPhase(2U, pointGroups);

    VkBufferMemoryBarrier drawBarrier{
        VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER};
    drawBarrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
    drawBarrier.dstAccessMask = VK_ACCESS_INDEX_READ_BIT;
    drawBarrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    drawBarrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    drawBarrier.buffer = resources->depthSortedIndexBuffer.buffer;
    drawBarrier.offset = 0U;
    drawBarrier.size = VK_WHOLE_SIZE;
    vkCmdPipelineBarrier(
        commandBuffer,
        VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
        VK_PIPELINE_STAGE_VERTEX_INPUT_BIT,
        0U,
        0U,
        nullptr,
        1U,
        &drawBarrier,
        0U,
        nullptr);
    resources->depthSortCacheKey = sortKey;
    resources->depthSortCacheValid = true;
    return true;
}

renderer::pointcloud::PointCloudMaterialVariant
VulkanViewportShell::ResolvePointCloudLayerMaterialVariant(
    const SceneRenderState::PointCloudLayerState& layer) const {
    const auto* resources = FindPointCloudResources(layer.layerId);
    // ConstantSimple and OpaqueHardDisc deliberately omit the procedural
    // water evaluator, so the layer needs the unified material whenever the
    // attached Seepage topology's current params can put the effect on
    // screen. When every node contributes nothing (seepageCanContribute
    // false) the unified shader would render identically to the optimized
    // shaders, so those stay selectable. Mid-flight parameter updates need
    // no extra guard: until a frame slot's params generation settles, the
    // style readiness gate disables Seepage for that individual frame on
    // the unified path too, so both variants draw the same image.
    const bool hasAttachedSeepage =
        resources != nullptr &&
        resources->seepageNodeCount > 0U &&
        resources->seepageHashCellCapacity > 0U &&
        resources->seepageNodeReferenceCount > 0U &&
        resources->seepageUnionBounds.valid &&
        resources->seepageCanContribute;
    const bool hasTimingColourise =
        renderer::pointcloud::TimingColouriseStackHasActiveEffects(
            layer.timingColourise);
    // Shoreline waves are evaluated only by the unified procedural shader,
    // and historically rode along on the rain forcing that put every layer
    // on it. Rain forcing is now band-gated per layer, so shoreline must
    // claim the unified material explicitly.
    const bool hasShorelineWaves =
        renderer::pointcloud::PointCloudStyleHasShorelineWaveRegion(
            layer.style) ||
        (layer.shorelineInstancesEligible &&
         !renderState_.additionalShorelines.empty());
    const bool hasSurfaceStabilityOpacity =
        static_cast<std::uint32_t>(
            layer.style.effectiveSurfaceStabilityMode) >=
            static_cast<std::uint32_t>(
                renderer::pointcloud::PointCloudSurfaceStabilityMode::
                    DensityContinuity) &&
        FindSurfaceStabilityScalarFieldSlot(layer.scalarFields).has_value();
    return renderer::pointcloud::ResolvePointCloudMaterialVariant(
        layer.style,
        layer.densityCompensation,
        hasAttachedSeepage || hasTimingColourise || hasShorelineWaves ||
            hasSurfaceStabilityOpacity);
}

bool VulkanViewportShell::LayerUsesDepthTestedAccumulation(
    const SceneRenderState::PointCloudLayerState& layer,
    renderer::pointcloud::PointCloudMaterialVariant materialVariant) const {
    // Preview performance mode targets layers that would render on the
    // opaque early-z fast path if the procedural effects (timing colourise,
    // seepage, rain shading) were not forcing the unified material. Those
    // layers are opaque-styled, so depth-testing their weighted accumulation
    // against the prepass depth culls only fragments an opaque pass would
    // hide anyway. Genuinely translucent layers never qualify. Shoreline
    // waves author partially transparent points (opacity multipliers below
    // one), so a wave layer's prepass depth would wrongly cull geometry
    // behind its translucent crests — shoreline-forced layers stay on plain
    // weighted accumulation.
    const bool hasShorelineWaves =
        renderer::pointcloud::PointCloudStyleHasShorelineWaveRegion(
            layer.style) ||
        (layer.shorelineInstancesEligible &&
         !renderState_.additionalShorelines.empty());
    // Density compensation normally keeps coarse display sources on weighted
    // accumulation so their coverage matches the 1 mm reference. Performance
    // mode does not replace that material: it only adds a matching read-only
    // depth test. Classify the authored style with neutral compensation so an
    // otherwise opaque 5 mm layer can benefit too. This is the preview-only
    // blending trade-off advertised by the setting.
    return renderState_.previewPerformanceMode &&
           !hasShorelineWaves &&
           materialVariant ==
               renderer::pointcloud::PointCloudMaterialVariant::Unified &&
           renderer::pointcloud::PointCloudStyleSupportsPreviewDepthCulling(
               layer.style);
}

bool VulkanViewportShell::UploadPointCloudLayerStyle(
    const SceneRenderState::PointCloudLayerState& layer,
    const PointCloudDrawPlan& plan,
    std::size_t frameIndex,
    std::uint32_t imageIndex,
    bool exrStyle,
    const BufferAllocation* styleBufferOverride,
    const bool* seepageDescriptorReadyOverride) {
    auto* resources = plan.resources;
    if (resources == nullptr || plan.drawPointCount == 0 || frameIndex >= kFramesInFlight) {
        return false;
    }

    PointCloudStyleGpu styleGpu;
    styleGpu.solidColor = glm::vec4{
        layer.style.solidColor[0],
        layer.style.solidColor[1],
        layer.style.solidColor[2],
        layer.style.solidColor[3],
    };
    styleGpu.gradientStartColor = glm::vec4{
        layer.style.gradientStartColor[0],
        layer.style.gradientStartColor[1],
        layer.style.gradientStartColor[2],
        1.0F,
    };
    styleGpu.gradientEndColor = glm::vec4{
        layer.style.gradientEndColor[0],
        layer.style.gradientEndColor[1],
        layer.style.gradientEndColor[2],
        1.0F,
    };
    styleGpu.colorize = glm::vec4{
        layer.style.colorizeColor[0],
        layer.style.colorizeColor[1],
        layer.style.colorizeColor[2],
        std::clamp(layer.style.colorizeAmount, 0.0F, 1.0F),
    };
    const std::size_t timingColouriseCount = std::min<std::size_t>(
        layer.timingColourise.effectCount,
        layer.timingColourise.effects.size());
    std::uint32_t packedTimingColouriseCount = 0U;
    for (std::size_t effectIndex = 0;
         effectIndex < timingColouriseCount;
         ++effectIndex) {
        const auto& effect = layer.timingColourise.effects[effectIndex];
        const bool scalarSource =
            effect.source ==
            renderer::pointcloud::TimingColouriseSource::ScalarField;
        const bool scalarAvailable =
            effect.scalarFieldSlot >= 0 &&
            static_cast<std::uint32_t>(effect.scalarFieldSlot) <
                resources->scalarFieldCount;
        const bool normalSource =
            effect.source ==
                renderer::pointcloud::TimingColouriseSource::NormalX ||
            effect.source ==
                renderer::pointcloud::TimingColouriseSource::NormalY ||
            effect.source ==
                renderer::pointcloud::TimingColouriseSource::NormalZ;
        const bool boundsValid =
            std::isfinite(effect.lowerBound) &&
            std::isfinite(effect.upperBound) &&
            effect.upperBound > effect.lowerBound;
        const bool outputValid =
            effect.output ==
                renderer::pointcloud::TimingColouriseOutput::Colourise ||
            effect.output ==
                renderer::pointcloud::TimingColouriseOutput::Emissive;
        if (!effect.enabled ||
            !boundsValid ||
            !outputValid ||
            !((scalarSource && scalarAvailable) ||
              (normalSource && resources->hasNormals))) {
            continue;
        }

        const std::size_t packedEffectIndex =
            static_cast<std::size_t>(packedTimingColouriseCount);
        styleGpu.timingColouriseSources[packedEffectIndex] = glm::uvec4{
            1U,
            static_cast<std::uint32_t>(effect.source),
            scalarAvailable
                ? static_cast<std::uint32_t>(effect.scalarFieldSlot) + 1U
                : 0U,
            static_cast<std::uint32_t>(effect.output),
        };
        styleGpu.timingColouriseRanges[packedEffectIndex] = glm::vec4{
            std::isfinite(effect.lowerBound) ? effect.lowerBound : 0.0F,
            std::isfinite(effect.upperBound) ? effect.upperBound : 0.0F,
            // z/w carry the primary/secondary colourise blend modes for the
            // per-channel fold in the shader; emissive slots blend nothing.
            effect.output ==
                    renderer::pointcloud::TimingColouriseOutput::Colourise
                ? static_cast<float>(effect.blendMode)
                : 0.0F,
            effect.output ==
                    renderer::pointcloud::TimingColouriseOutput::Colourise
                ? static_cast<float>(effect.secondaryBlendMode)
                : 0.0F,
        };
        styleGpu.timingColouriseFades[packedEffectIndex] = glm::vec4{
            std::clamp(
                std::isfinite(effect.edgeFadeLowerFraction)
                    ? effect.edgeFadeLowerFraction
                    : 0.10F,
                -renderer::pointcloud::
                    kTimingColouriseMaximumOutwardEdgeFade,
                renderer::pointcloud::
                    kTimingColouriseMaximumInwardEdgeFade),
            std::clamp(
                std::isfinite(effect.edgeFadeUpperFraction)
                    ? effect.edgeFadeUpperFraction
                    : 0.10F,
                -renderer::pointcloud::
                    kTimingColouriseMaximumOutwardEdgeFade,
                renderer::pointcloud::
                    kTimingColouriseMaximumInwardEdgeFade),
            effect.output ==
                    renderer::pointcloud::TimingColouriseOutput::Colourise
                ? static_cast<float>(effect.blendCompositionMode)
                : 0.0F,
            effect.output ==
                    renderer::pointcloud::TimingColouriseOutput::Colourise
                ? std::clamp(
                      std::isfinite(effect.blendMix)
                          ? effect.blendMix
                          : 1.0F,
                      0.0F,
                      1.0F)
                : 0.0F,
        };
        if (effect.output ==
            renderer::pointcloud::TimingColouriseOutput::Colourise) {
            for (std::size_t sampleIndex = 0;
                 sampleIndex <
                 renderer::pointcloud::kTimingColouriseLutSamples;
                 ++sampleIndex) {
                const auto& sample = effect.rgbaLut[sampleIndex];
                const std::size_t packedIndex =
                    packedEffectIndex *
                        renderer::pointcloud::kTimingColouriseLutSamples +
                    sampleIndex;
                styleGpu.timingColouriseLut[packedIndex] = glm::vec4{
                    std::clamp(
                        std::isfinite(sample[0]) ? sample[0] : 0.0F,
                        0.0F,
                        1.0F),
                    std::clamp(
                        std::isfinite(sample[1]) ? sample[1] : 0.0F,
                        0.0F,
                        1.0F),
                    std::clamp(
                        std::isfinite(sample[2]) ? sample[2] : 0.0F,
                        0.0F,
                        1.0F),
                    std::clamp(
                        std::isfinite(sample[3]) ? sample[3] : 0.0F,
                        0.0F,
                        1.0F),
                };
            }
        } else if (
            effect.output ==
            renderer::pointcloud::TimingColouriseOutput::Emissive) {
            // Emissive slots carry their falloff-shaped signed level in
            // LUT channel r, unclamped to [0, 1] because emission may
            // exceed one and darkening is negative.
            for (std::size_t sampleIndex = 0;
                 sampleIndex <
                 renderer::pointcloud::kTimingColouriseLutSamples;
                 ++sampleIndex) {
                const float level = effect.rgbaLut[sampleIndex][0];
                const std::size_t packedIndex =
                    packedEffectIndex *
                        renderer::pointcloud::kTimingColouriseLutSamples +
                    sampleIndex;
                styleGpu.timingColouriseLut[packedIndex] = glm::vec4{
                    std::clamp(
                        std::isfinite(level) ? level : 0.0F,
                        -64.0F,
                        64.0F),
                    0.0F,
                    0.0F,
                    0.0F,
                };
            }
        }
        ++packedTimingColouriseCount;
    }
    styleGpu.timingColouriseControl.x = packedTimingColouriseCount;
    styleGpu.globalControl = glm::uvec4{
        static_cast<std::uint32_t>(layer.style.colorMode),
        static_cast<std::uint32_t>(layer.style.colormap),
        resources->scalarFieldCount,
        layer.hasSourceRgb ? 1U : 0U,
    };
    // Source-local GPU Flow owns positions in its vec4 storage output; the
    // legacy Float3 vertex allocation is intentionally never populated.
    // Preserve that resource invariant even if a caller supplies a plain
    // point style while a topology update is settling.
    const bool waterTrailOverlay =
        layer.generatedWaterOverlay &&
        (layer.style.waterTrailOverlay || resources->waterFlowSourceActive);
    const std::uint32_t waterContentKind =
        !layer.generatedWaterOverlay
            ? 0U
            : (waterTrailOverlay
                   ? 3U
                   : (layer.style.flowAnimation
                          ? (layer.style.waterPathView ? 2U : 1U)
                          : 0U));
    styleGpu.pointMeta = glm::uvec4{
        resources->pointCount,
        static_cast<std::uint32_t>(layer.adaptiveDensityRole),
        resources->hasNormals ? 1U : 0U,
        waterContentKind,
    };
    const auto densityCompensation =
        renderer::pointcloud::SanitizePointCloudDensityCompensation(layer.densityCompensation);
    const auto materialVariant = ResolvePointCloudLayerMaterialVariant(layer);
    // Depth-tested accumulation only culls if the prepass actually wrote
    // this layer's depth, which the depth fragment shaders gate on
    // renderControl.x; without this the perf-mode prepass draw discards
    // every fragment and the depth test passes everywhere. Live-only: the
    // EXR depth pass must stay independent of the preview toggle.
    const bool softDepthWrites =
        renderer::pointcloud::PointCloudDepthPrepassWrites(layer.style);
    const bool softDepthTests =
        renderer::pointcloud::PointCloudDepthPrepassTests(layer.style);
    const bool sortedDepthAovContribution =
        exrStyle && layer.style.gpuBackToFrontSorting &&
        layer.style.geometryMode ==
            renderer::pointcloud::PointCloudGeometryMode::ScreenSprites &&
        !layer.style.depthPrepassEnabled;
    const bool forceDepthContribution =
        renderState_.eyeDomeLightingEnabled || liveSceneReadbackCaptureEnabled_ ||
        materialVariant ==
            renderer::pointcloud::PointCloudMaterialVariant::OpaqueHardDisc ||
        sortedDepthAovContribution ||
        (!exrStyle && LayerUsesDepthTestedAccumulation(layer, materialVariant));
    // 0: no soft-depth participation; 1: soft write+test; 2: forced hard
    // coverage (performance/AOV paths); 3: test the shared depth but do not
    // write it. Test-only is what lets SAND receive ROCK occlusion without
    // allowing overlapping sand scans to replace the nearest surface.
    const std::uint32_t depthContributionMode = softDepthWrites
                                                    ? 1U
                                                : softDepthTests
                                                    ? 3U
                                                : (!layer.style.depthPrepassEnabled &&
                                                           forceDepthContribution
                                                       ? 2U
                                                       : 0U);
    styleGpu.renderControl = glm::uvec4{
        depthContributionMode,
        static_cast<std::uint32_t>(layer.style.falloffProfile),
        static_cast<std::uint32_t>(layer.style.geometryMode),
        layer.style.solidCenters ? 1U : 0U,
    };
    styleGpu.renderParams0 = glm::vec4{
        layer.style.exposure,
        layer.style.innerRadius,
        layer.style.gaussianSharpness,
        layer.style.featherPower,
    };
    styleGpu.renderParams1 = glm::vec4{
        layer.style.waterTrailStyleGeometry ? 1.0F : 0.0F,
        densityCompensation.footprintScale,
        densityCompensation.coverageCorrection,
        std::clamp(
            std::isfinite(layer.style.waterFlowActivity) ? layer.style.waterFlowActivity : 1.0F,
            0.0F,
            1.0F),
    };
    styleGpu.renderParams2 = glm::vec4{
        kPointCloudAntialiasFeatherPixels,
        renderer::pointcloud::SanitizeWaterFlowSpeedScale(layer.style.waterFlowSpeedScale),
        std::clamp(layer.style.waterStreakAspect, 1.0F, 32.0F),
        renderer::pointcloud::PointCloudStyleUsesWorldSizedScreenSprites(layer.style) ? 1.0F : 0.0F,
    };
    styleGpu.renderParams3 = glm::vec4{
        std::clamp(layer.style.depthPrepassAlphaThreshold, 0.0F, 1.0F),
        pointSizeRangeMin_,
        pointSizeRangeMax_,
        std::max(0.0F, renderState_.flowTimeSeconds),
    };
    styleGpu.stylisationControl = glm::uvec4{
        static_cast<std::uint32_t>(layer.style.stylisationMode),
        static_cast<std::uint32_t>(layer.style.nprPreset),
        0U,
        0U,
    };
    styleGpu.stylisationParams0 = glm::vec4{
        std::clamp(layer.style.stylisationStrength, 0.0F, 1.0F),
        std::clamp(layer.style.stylisationColorLevels, 2.0F, 16.0F),
        std::clamp(layer.style.stylisationInkStrength, 0.0F, 1.0F),
        std::clamp(layer.style.stylisationPaperGrain, 0.0F, 1.0F),
    };
    styleGpu.stylisationParams1 = glm::vec4{
        std::clamp(layer.style.stylisationPigmentBleed, 0.0F, 1.0F),
        std::clamp(layer.style.brushAspect, 0.25F, 6.0F),
        std::clamp(layer.style.strokeJitter, 0.0F, 1.0F),
        std::clamp(layer.style.hatchStrength, 0.0F, 1.0F),
    };
    styleGpu.stylisationParams2 = glm::vec4{
        std::clamp(layer.style.strokeOpacityVariance, 0.0F, 1.0F),
        std::clamp(layer.style.pigmentVariation, 0.0F, 1.0F),
        std::clamp(layer.style.pigmentAnimationSpeed, 0.0F, 4.0F),
        std::clamp(layer.style.granulationAngleStrength, 0.0F, 1.0F),
    };
    styleGpu.depthCompositingParams = glm::vec4{
        std::clamp(layer.style.depthPrepassAlphaThreshold, 0.0F, 1.0F),
        std::clamp(layer.style.depthPrepassToleranceMeters, 0.0F, 10.0F),
        std::clamp(layer.style.depthWeightStrength, 1.0F, 8.0F),
        layer.style.gpuBackToFrontSorting ? 1.0F : 0.0F,
    };
    constexpr float kDegreesToRadians = 0.01745329252F;
    const float normalCullStartDegrees =
        std::clamp(layer.style.normalCullStartDegrees, 0.0F, 179.0F);
    const float normalCullEndDegrees = std::clamp(
        std::max(
            layer.style.normalCullEndDegrees,
            normalCullStartDegrees + 1.0F),
        1.0F,
        180.0F);
    styleGpu.emissionNormalControl = glm::vec4{
        layer.style.emissionResponse ==
                renderer::pointcloud::PointCloudEmissionResponse::Saturated
            ? 1.0F
            : 0.0F,
        // Layers without stored normals cannot be facing-culled; gating here
        // keeps the vertex path free of the extra work for them.
        layer.style.normalCullEnabled && resources->hasNormals ? 1.0F : 0.0F,
        std::cos(normalCullStartDegrees * kDegreesToRadians),
        std::cos(normalCullEndDegrees * kDegreesToRadians),
    };
    const auto surfaceStabilitySlot =
        FindSurfaceStabilityScalarFieldSlot(layer.scalarFields);
    styleGpu.surfaceStabilityControl = glm::uvec4{
        static_cast<std::uint32_t>(
            layer.style.effectiveSurfaceStabilityMode),
        surfaceStabilitySlot.has_value() &&
                surfaceStabilitySlot.value() < resources->scalarFieldCount
            ? surfaceStabilitySlot.value()
            : std::numeric_limits<std::uint32_t>::max(),
        0U,
        0U,
    };
    styleGpu.surfaceStabilityParams = glm::vec4{
        std::clamp(
            std::isfinite(layer.style.surfaceStabilityInfluence)
                ? layer.style.surfaceStabilityInfluence
                : 1.0F,
            0.0F,
            1.0F),
        0.0F,
        0.0F,
        0.0F,
    };
    if (layer.style.roughnessMotionStrength > 1.0e-5F) {
        auto setSurfaceMotionParams = [&]() {
            styleGpu.surfaceMotionParams = glm::vec4{
                std::clamp(layer.style.roughnessMotionStrength, 0.0F, 1.0F),
                std::clamp(layer.style.roughnessMotionScale, 0.01F, 50.0F),
                std::clamp(layer.style.roughnessMotionSpeed, 0.0F, 8.0F),
                std::clamp(layer.style.roughnessMotionThreshold, 0.0F, 1.0F),
            };
        };
        if (layer.style.roughnessMotionFullLayer) {
            styleGpu.stylisationControl.z = std::numeric_limits<std::uint32_t>::max();
            setSurfaceMotionParams();
            styleGpu.surfaceMotionStats = glm::vec4{
                0.0F,
                1.0F,
                std::clamp(layer.style.roughnessMotionGroundId, 0.0F, 1.0F),
                0.25F,
            };
        } else {
            const auto roughnessSlot = FindRoughnessScalarFieldSlot(layer.scalarFields);
            if (roughnessSlot.has_value() && roughnessSlot.value() < layer.scalarFields.size()) {
                const auto& roughnessStats = layer.scalarFields[roughnessSlot.value()];
                const float roughnessRange =
                    std::max(1.0e-6F, roughnessStats.maximum - roughnessStats.minimum);
                styleGpu.stylisationControl.z = roughnessSlot.value() + 1U;
                if (const auto groundSlot = FindGroundIdScalarFieldSlot(layer.scalarFields);
                    groundSlot.has_value() && groundSlot.value() < layer.scalarFields.size()) {
                    styleGpu.stylisationControl.w = groundSlot.value() + 1U;
                }
                setSurfaceMotionParams();
                styleGpu.surfaceMotionStats = glm::vec4{
                    roughnessStats.minimum,
                    1.0F / roughnessRange,
                    std::clamp(layer.style.roughnessMotionGroundId, 0.0F, 1.0F),
                    0.25F,
                };
            }
        }
    }
    if (layer.style.shorelineWaveEnabled) {
        const bool heightFoam =
            layer.style.shorelineWaveAlgorithm ==
            invisible_places::renderer::pointcloud::PointCloudShorelineWaveAlgorithm::HeightFoam;
        const bool continuousBands =
            layer.style.shorelineWaveAlgorithm ==
            invisible_places::renderer::pointcloud::PointCloudShorelineWaveAlgorithm::ContinuousBands;
        const auto& foam = layer.style.shorelineHeightFoam;
        const glm::vec2 directionInput = heightFoam
                                             ? glm::vec2{foam.directionX, foam.directionY}
                                             : glm::vec2{
                                                   layer.style.shorelineDirectionX,
                                                   layer.style.shorelineDirectionY,
                                               };
        const glm::vec2 shorelineDirection =
            glm::dot(directionInput, directionInput) > 1.0e-8F
                ? glm::normalize(directionInput)
                : glm::vec2{1.0F, 0.0F};
        const float boundaryZ = heightFoam ? foam.runupZ : layer.style.shorelineBoundaryZ;
        const float reachMeters = heightFoam
                                      ? std::clamp(foam.offshoreReachMeters, 0.001F, 50.0F)
                                      : std::clamp(layer.style.shorelineHeightReachMeters, 0.001F, 50.0F);
        const float edgeFadeMeters = heightFoam
                                         ? std::clamp(foam.edgeFadeMeters, 0.0F, 10.0F)
                                         : std::clamp(layer.style.shorelineEdgeFadeMeters, 0.0F, 10.0F);
        const float breakZ = heightFoam
                                 ? invisible_places::renderer::pointcloud::NormalizeHeightFoamBreakZ(
                                       boundaryZ,
                                       reachMeters,
                                       edgeFadeMeters,
                                       foam.breakZ)
                                 : boundaryZ;
        // shorelineWaveControl.z is the shader algorithm ABI: Foam Fronts =
        // 0, Height Foam = 1, and Continuous Bands = 2. Keep this explicit so
        // the UI enum may remain independently ordered.
        styleGpu.shorelineWaveControl = glm::uvec4{
            1U,
            heightFoam ? foam.seed : layer.style.shorelineSeed,
            heightFoam ? 1U : (continuousBands ? 2U : 0U),
            0U,
        };
        styleGpu.shorelineWaveParams0 = glm::vec4{
            boundaryZ,
            reachMeters,
            edgeFadeMeters,
            heightFoam
                ? std::clamp(foam.intensity, 0.0F, 5.0F)
                : std::clamp(layer.style.shorelineIntensity, 0.0F, 5.0F),
        };
        styleGpu.shorelineWaveParams1 = glm::vec4{
            shorelineDirection.x,
            shorelineDirection.y,
            heightFoam
                ? std::clamp(foam.patternScale, 0.01F, 50.0F)
                : std::clamp(layer.style.shorelinePatternScale, 0.01F, 50.0F),
            heightFoam
                ? std::clamp(foam.wavelengthMeters, 0.002F, 10.0F)
                : std::clamp(layer.style.shorelineWavelengthMeters, 0.002F, 10.0F),
        };
        styleGpu.shorelineWaveParams2 = glm::vec4{
            heightFoam ? std::clamp(foam.speed, 0.0F, 10.0F)
                       : std::clamp(layer.style.shorelineSpeed, 0.0F, 10.0F),
            heightFoam ? std::clamp(foam.warp, 0.0F, 3.0F)
                       : std::clamp(layer.style.shorelineWarp, 0.0F, 3.0F),
            heightFoam ? std::clamp(foam.turbulence, 0.0F, 1.0F)
                       : std::clamp(layer.style.shorelineTurbulence, 0.0F, 1.0F),
            heightFoam ? std::clamp(foam.density, 0.0F, 1.0F)
                       : std::clamp(layer.style.shorelineDensity, 0.0F, 1.0F),
        };
        styleGpu.shorelineWaveParams3 = glm::vec4{
            heightFoam ? foam.phase : layer.style.shorelinePhase,
            heightFoam ? std::clamp(foam.emissionAdd, 0.0F, 8.0F)
                       : std::clamp(layer.style.shorelineEmissionAdd, 0.0F, 8.0F),
            heightFoam ? std::clamp(foam.opacityAdd, -1.0F, 2.0F)
                       : std::clamp(layer.style.shorelineOpacityAdd, -1.0F, 2.0F),
            heightFoam ? std::clamp(foam.opacityMultiply, 0.0F, 8.0F)
                       : std::clamp(layer.style.shorelineOpacityMultiply, 0.0F, 8.0F),
        };
        styleGpu.shorelineWaveParams4 = glm::vec4{
            heightFoam ? std::clamp(foam.pointSizeAdd, -256.0F, 512.0F)
                       : std::clamp(layer.style.shorelinePointSizeAdd, -256.0F, 512.0F),
            heightFoam ? std::clamp(foam.pointSizeMultiply, 0.0F, 8.0F)
                       : std::clamp(layer.style.shorelinePointSizeMultiply, 0.0F, 8.0F),
            heightFoam ? std::clamp(foam.colourMix, 0.0F, 1.0F)
                       : std::clamp(layer.style.shorelineColourMix, 0.0F, 1.0F),
            // Foam Fronts Background Wash; the shader treats zero (older
            // documents and Height Foam) as the neutral 1.
            heightFoam ? 0.0F
                       : std::clamp(layer.style.shorelineBackgroundWash, 0.05F, 2.0F),
        };
        styleGpu.shorelineWaveParams5 = glm::vec4{
            breakZ,
            heightFoam ? std::clamp(foam.offshoreFoamStrength, 0.0F, 3.0F) : 0.0F,
            heightFoam ? std::clamp(foam.incomingStrength, 0.0F, 5.0F) : 1.0F,
            heightFoam ? std::clamp(foam.returnStrength, 0.0F, 1.0F) : 0.0F,
        };
        const auto& shorelineColour = heightFoam ? foam.colour : layer.style.shorelineColour;
        styleGpu.shorelineWaveTint = glm::vec4{
            std::clamp(shorelineColour[0], 0.0F, 4.0F),
            std::clamp(shorelineColour[1], 0.0F, 4.0F),
            std::clamp(shorelineColour[2], 0.0F, 4.0F),
            1.0F,
        };
    }
    // The spare Y lane is a frame-local Mesh Flow contact visibility gate.
    // Contact buffers remain resident and continue aging while hidden.
    styleGpu.additionalShorelineCount = glm::uvec4{
        0U,
        renderState_.meshFlowContactEffectsEnabled ? 1U : 0U,
        0U,
        0U};
    if (layer.shorelineInstancesEligible) {
        std::uint32_t packedShorelineCount = 0U;
        for (const auto& instanceSettings : renderState_.additionalShorelines) {
            if (packedShorelineCount >=
                renderer::pointcloud::kMaxAdditionalShorelineInstances) {
                break;
            }
            if (!instanceSettings.enabled) {
                continue;
            }
            // Instances that opted out of the water sheet skip the gap-fill
            // layer; later instances compact upward exactly like disabled
            // ones so the shader sees a dense slot range.
            if (layer.waterFillLayer && !instanceSettings.applyToWaterFill) {
                continue;
            }
            // Same lane recipe as the primary block above, read from the
            // profile-shaped settings banks instead of the flattened style.
            const bool instanceHeightFoam =
                instanceSettings.algorithm ==
                invisible_places::renderer::pointcloud::
                    PointCloudShorelineWaveAlgorithm::HeightFoam;
            const bool instanceContinuousBands =
                instanceSettings.algorithm ==
                invisible_places::renderer::pointcloud::
                    PointCloudShorelineWaveAlgorithm::ContinuousBands;
            const auto& ff = instanceSettings.foamFronts;
            const auto& hf = instanceSettings.heightFoam;
            const glm::vec2 instanceDirectionInput =
                instanceHeightFoam
                    ? glm::vec2{hf.directionX, hf.directionY}
                    : glm::vec2{ff.directionX, ff.directionY};
            const glm::vec2 instanceDirection =
                glm::dot(instanceDirectionInput, instanceDirectionInput) >
                        1.0e-8F
                    ? glm::normalize(instanceDirectionInput)
                    : glm::vec2{1.0F, 0.0F};
            const float instanceBoundaryZ =
                instanceHeightFoam ? hf.runupZ : ff.boundaryZ;
            const float instanceReach =
                instanceHeightFoam
                    ? std::clamp(hf.offshoreReachMeters, 0.001F, 50.0F)
                    : std::clamp(ff.heightReachMeters, 0.001F, 50.0F);
            const float instanceEdgeFade =
                instanceHeightFoam
                    ? std::clamp(hf.edgeFadeMeters, 0.0F, 10.0F)
                    : std::clamp(ff.edgeFadeMeters, 0.0F, 10.0F);
            const float instanceBreakZ =
                instanceHeightFoam
                    ? invisible_places::renderer::pointcloud::
                          NormalizeHeightFoamBreakZ(
                              instanceBoundaryZ,
                              instanceReach,
                              instanceEdgeFade,
                              hf.breakZ)
                    : instanceBoundaryZ;
            const std::size_t slot = packedShorelineCount;
            styleGpu.additionalShorelineControl[slot] = glm::uvec4{
                1U,
                static_cast<std::uint32_t>(
                    instanceHeightFoam ? hf.seed : ff.seed),
                instanceHeightFoam ? 1U : (instanceContinuousBands ? 2U : 0U),
                0U,
            };
            styleGpu.additionalShorelineParams0[slot] = glm::vec4{
                instanceBoundaryZ,
                instanceReach,
                instanceEdgeFade,
                instanceHeightFoam ? std::clamp(hf.intensity, 0.0F, 5.0F)
                                   : std::clamp(ff.intensity, 0.0F, 5.0F),
            };
            styleGpu.additionalShorelineParams1[slot] = glm::vec4{
                instanceDirection.x,
                instanceDirection.y,
                instanceHeightFoam
                    ? std::clamp(hf.patternScale, 0.01F, 50.0F)
                    : std::clamp(ff.patternScale, 0.01F, 50.0F),
                instanceHeightFoam
                    ? std::clamp(hf.wavelengthMeters, 0.002F, 10.0F)
                    : std::clamp(ff.wavelengthMeters, 0.002F, 10.0F),
            };
            styleGpu.additionalShorelineParams2[slot] = glm::vec4{
                instanceHeightFoam ? std::clamp(hf.speed, 0.0F, 10.0F)
                                   : std::clamp(ff.speed, 0.0F, 10.0F),
                instanceHeightFoam ? std::clamp(hf.warp, 0.0F, 3.0F)
                                   : std::clamp(ff.warp, 0.0F, 3.0F),
                instanceHeightFoam ? std::clamp(hf.turbulence, 0.0F, 1.0F)
                                   : std::clamp(ff.turbulence, 0.0F, 1.0F),
                instanceHeightFoam ? std::clamp(hf.density, 0.0F, 1.0F)
                                   : std::clamp(ff.density, 0.0F, 1.0F),
            };
            styleGpu.additionalShorelineParams3[slot] = glm::vec4{
                instanceHeightFoam ? hf.phase : ff.phase,
                instanceHeightFoam ? std::clamp(hf.emissionAdd, 0.0F, 8.0F)
                                   : std::clamp(ff.emissionAdd, 0.0F, 8.0F),
                instanceHeightFoam ? std::clamp(hf.opacityAdd, -1.0F, 2.0F)
                                   : std::clamp(ff.opacityAdd, -1.0F, 2.0F),
                instanceHeightFoam
                    ? std::clamp(hf.opacityMultiply, 0.0F, 8.0F)
                    : std::clamp(ff.opacityMultiply, 0.0F, 8.0F),
            };
            styleGpu.additionalShorelineParams4[slot] = glm::vec4{
                instanceHeightFoam
                    ? std::clamp(hf.pointSizeAdd, -256.0F, 512.0F)
                    : std::clamp(ff.pointSizeAdd, -256.0F, 512.0F),
                instanceHeightFoam
                    ? std::clamp(hf.pointSizeMultiply, 0.0F, 8.0F)
                    : std::clamp(ff.pointSizeMultiply, 0.0F, 8.0F),
                instanceHeightFoam ? std::clamp(hf.colourMix, 0.0F, 1.0F)
                                   : std::clamp(ff.colourMix, 0.0F, 1.0F),
                instanceHeightFoam
                    ? 0.0F
                    : std::clamp(ff.backgroundWash, 0.05F, 2.0F),
            };
            styleGpu.additionalShorelineParams5[slot] = glm::vec4{
                instanceBreakZ,
                instanceHeightFoam
                    ? std::clamp(hf.offshoreFoamStrength, 0.0F, 3.0F)
                    : 0.0F,
                instanceHeightFoam
                    ? std::clamp(hf.incomingStrength, 0.0F, 5.0F)
                    : 1.0F,
                instanceHeightFoam
                    ? std::clamp(hf.returnStrength, 0.0F, 1.0F)
                    : 0.0F,
            };
            const auto& instanceColour =
                instanceHeightFoam ? hf.colour : ff.colour;
            styleGpu.additionalShorelineTint[slot] = glm::vec4{
                std::clamp(instanceColour[0], 0.0F, 4.0F),
                std::clamp(instanceColour[1], 0.0F, 4.0F),
                std::clamp(instanceColour[2], 0.0F, 4.0F),
                1.0F,
            };
            ++packedShorelineCount;
        }
        styleGpu.additionalShorelineCount.x = packedShorelineCount;
    }
    const bool seepageDescriptorReady =
        exrStyle
            ? resources->exrDescriptorSet != VK_NULL_HANDLE &&
                  resources->seepageExrDescriptorGeneration ==
                      resources->seepageTopologyGeneration &&
                  resources->seepageParamsExrGeneration ==
                      resources->seepageParamsGeneration
            : seepageDescriptorReadyOverride != nullptr
                  ? *seepageDescriptorReadyOverride
                  : frameIndex < kFramesInFlight &&
                        imageIndex < resources->descriptorSets[frameIndex].size() &&
                        imageIndex <
                            resources->seepageDescriptorGenerations[frameIndex].size() &&
                        resources->descriptorSets[frameIndex][imageIndex] != VK_NULL_HANDLE &&
                        resources->seepageDescriptorGenerations[frameIndex][imageIndex] ==
                            resources->seepageTopologyGeneration &&
                        resources->seepageParamsFrameGenerations[frameIndex] ==
                            resources->seepageParamsGeneration;
    if (seepageDescriptorReady &&
        resources->seepageNodeCount > 0U &&
        resources->seepageHashCellCapacity > 0U &&
        resources->seepageNodeReferenceCount > 0U &&
        resources->seepageUnionBounds.valid) {
        const float cellSize = std::max(0.001F, resources->seepageCellSizeMeters);
        styleGpu.seepageControl = glm::uvec4{
            1U,
            resources->seepageNodeCount,
            resources->seepageHashCellCapacity,
            resources->seepageNodeReferenceCount,
        };
        styleGpu.seepageGridParams = glm::vec4{
            cellSize,
            1.0F / cellSize,
            static_cast<float>(std::max(1U, resources->seepageHashProbeLimit)),
            resources->seepageUsesConnectedSupport ? 1.0F : 0.0F,
        };
        styleGpu.seepageBoundsMin = glm::vec4{
            resources->seepageUnionBounds.minimum.x,
            resources->seepageUnionBounds.minimum.y,
            resources->seepageUnionBounds.minimum.z,
            0.0F,
        };
        styleGpu.seepageBoundsMax = glm::vec4{
            resources->seepageUnionBounds.maximum.x,
            resources->seepageUnionBounds.maximum.y,
            resources->seepageUnionBounds.maximum.z,
            0.0F,
        };
    }
    const auto rainRole = static_cast<std::uint32_t>(layer.rainCollisionRole);
    // The impact effects are decoupled from the layer's cloud role: EVERY
    // rendered point-cloud layer receives the bitmask of enabled effects
    // (1 Rings, 2 Wetness, 4 Droplets) regardless of its collision role, and
    // the shader gates each model by its own world-Z height band instead.
    // rainImpactControl.y still carries the layer role because the mesh-flow
    // contact include (pointcloud_mesh_flow_contact.glsl) selects ROCK/VEG
    // point styles through it.
    const std::uint32_t rainEffectMask =
        rainResources_.collisionReady
            ? invisible_places::water::RainImpactEffectMask(renderState_.rainSettings)
            : 0U;
    const float rainGridWorldSpan = invisible_places::water::RainImpactGridWorldSpan(
        renderState_.rainSettings);
    styleGpu.rainImpactControl = glm::uvec4{
        rainEffectMask,
        rainRole,
        invisible_places::water::kRainImpactGridDimension,
        invisible_places::water::kRainImpactEventCapacity,
    };
    styleGpu.rainImpactGrid = glm::vec4{
        renderState_.cameraPosition.x - rainGridWorldSpan * 0.5F,
        renderState_.cameraPosition.y - rainGridWorldSpan * 0.5F,
        rainGridWorldSpan / static_cast<float>(invisible_places::water::kRainImpactGridDimension),
        std::max(0.0F, renderState_.flowTimeSeconds),
    };
    const auto& rockImpact = renderState_.rainSettings.rockImpact;
    styleGpu.rainImpactRock0 = glm::vec4{
        std::clamp(rockImpact.edgeBreakup, 0.0F, 1.0F),
        std::clamp(rockImpact.spreadSpeed, 0.10F, 6.0F),
        std::clamp(rockImpact.centreFalloff, 0.0F, 1.0F),
        std::clamp(rockImpact.heightBias, 0.0F, 2.0F),
    };
    const auto rockBand = invisible_places::water::SanitizeRainImpactHeightBand(
        renderState_.rainSettings.rockImpactBand);
    styleGpu.rainImpactRock1 = glm::vec4{
        std::clamp(rockImpact.persistence, 0.25F, 3.0F),
        rockBand.minZ,
        rockBand.maxZ,
        rockBand.fadeMeters,
    };
    const auto& vegetationImpact = renderState_.rainSettings.vegetationImpact;
    styleGpu.rainImpactVegetation0 = glm::vec4{
        std::clamp(vegetationImpact.twinkle, 0.0F, 4.0F),
        std::clamp(vegetationImpact.propagationMetersPerSecond, 0.05F, 6.0F),
        std::clamp(vegetationImpact.hopSpacingMeters, 0.010F, 0.30F),
        std::max(0.001F, vegetationImpact.streamWidthMeters),
    };
    const auto vegetationBand = invisible_places::water::SanitizeRainImpactHeightBand(
        renderState_.rainSettings.vegetationImpactBand);
    styleGpu.rainImpactVegetation1 = glm::vec4{
        std::clamp(vegetationImpact.streamSpread, 0.0F, 2.0F),
        vegetationBand.minZ,
        vegetationBand.maxZ,
        vegetationBand.fadeMeters,
    };
    const auto sandBand = invisible_places::water::SanitizeRainImpactHeightBand(
        renderState_.rainSettings.sandImpactBand);
    styleGpu.rainImpactSandBand = glm::vec4{
        sandBand.minZ,
        sandBand.maxZ,
        sandBand.fadeMeters,
        std::clamp(renderState_.rainSettings.rockImpact.downhillStretch, 0.0F, 2.0F),
    };
    styleGpu.rainImpactResponse = glm::vec4{
        std::max(0.0F, renderState_.rainSettings.sandEffectScale),
        std::max(0.0F, renderState_.rainSettings.rockEffectScale),
        std::max(0.0F, renderState_.rainSettings.vegetationEffectScale),
        std::isfinite(renderState_.rainSettings.ringImpact.thicknessScale)
            ? std::clamp(
                  renderState_.rainSettings.ringImpact.thicknessScale,
                  0.25F,
                  2.0F)
            : 1.0F,
    };
    styleGpu.pointSize = MakePointCloudBindingGpu(
        layer.style.pointSize,
        layer.scalarFields,
        renderer::pointcloud::kInactivePointSizeDefault);
    ScalePointCloudBindingGpu(&styleGpu.pointSize, renderState_.pointSizeScale);
    styleGpu.opacity = MakePointCloudBindingGpu(
        layer.style.opacity,
        layer.scalarFields,
        renderer::pointcloud::kInactiveOpacityDefault);
    styleGpu.emissive = MakePointCloudBindingGpu(
        layer.style.emissiveStrength,
        layer.scalarFields,
        renderer::pointcloud::kInactiveEmissionDefault);
    styleGpu.depthFade = MakePointCloudBindingGpu(
        layer.style.depthFade,
        layer.scalarFields,
        renderer::pointcloud::kInactiveDepthFadeDefault);
    styleGpu.colormapPosition = MakePointCloudBindingGpu(
        layer.style.colormapPosition,
        layer.scalarFields,
        renderer::pointcloud::kInactiveColormapPositionDefault);
    styleGpu.surfelDiameter = MakePointCloudBindingGpu(
        layer.style.surfelDiameter,
        layer.scalarFields,
        renderer::pointcloud::kInactiveSurfelDiameterDefault);

    const auto& styleBuffer =
        styleBufferOverride != nullptr
            ? *styleBufferOverride
            : (exrStyle ? resources->exrStyleBuffer : resources->styleBuffers[frameIndex]);
    UploadBufferData(styleBuffer, &styleGpu, sizeof(styleGpu));
    return true;
}

bool VulkanViewportShell::RecordPointCloudLayerDraw(
    VkCommandBuffer commandBuffer,
    const SceneRenderState::PointCloudLayerState& layer,
    bool forceFullSource,
    VkPipeline spritePipeline,
    VkPipeline surfelPipeline,
    bool uploadStyle,
    std::size_t frameIndex,
    std::uint32_t imageIndex,
    bool exrStyle,
    std::uint32_t* recordedDrawPointCount) {
    if (recordedDrawPointCount != nullptr) {
        *recordedDrawPointCount = 0;
    }
    PointCloudDrawPlan plan;
    if (!ResolvePointCloudDrawPlan(layer, forceFullSource, !exrStyle, &plan)) {
        return false;
    }
    auto* resources = plan.resources;
    if (plan.worldSurfels) {
        if (surfelPipeline == VK_NULL_HANDLE) {
            return false;
        }
    } else if (spritePipeline == VK_NULL_HANDLE) {
        return false;
    }

    if (uploadStyle) {
        if (!UploadPointCloudLayerStyle(
                layer,
                plan,
                frameIndex,
                imageIndex,
                exrStyle)) {
            return false;
        }
    }

    vkCmdBindPipeline(
        commandBuffer,
        VK_PIPELINE_BIND_POINT_GRAPHICS,
        plan.worldSurfels ? surfelPipeline : spritePipeline);

    VkDescriptorSet descriptorSet = VK_NULL_HANDLE;
    if (exrStyle) {
        descriptorSet = resources->exrDescriptorSet;
    } else if (frameIndex < kFramesInFlight &&
               imageIndex < resources->descriptorSets[frameIndex].size()) {
        descriptorSet = resources->descriptorSets[frameIndex][imageIndex];
    }
    if (descriptorSet == VK_NULL_HANDLE) {
        return false;
    }

    vkCmdBindDescriptorSets(
        commandBuffer,
        VK_PIPELINE_BIND_POINT_GRAPHICS,
        pointPipelineLayout_,
        0,
        1,
        &descriptorSet,
        0,
        nullptr);

    if (recordedDrawPointCount != nullptr) {
        *recordedDrawPointCount = plan.drawPointCount;
    }

    if (plan.worldSurfels) {
        const std::uint32_t surfelVertexCount = plan.drawPointCount * kSurfelVerticesPerPoint;
        if (plan.sampledBudgetReady) {
            vkCmdBindIndexBuffer(
                commandBuffer,
                resources->sampledSurfelIndexBuffer.buffer,
                0,
                VK_INDEX_TYPE_UINT32);
            vkCmdDrawIndexed(commandBuffer, surfelVertexCount, 1, 0, 0, 0);
        } else if (plan.interactiveSampleReady) {
            vkCmdBindIndexBuffer(
                commandBuffer,
                resources->interactiveSurfelIndexBuffer.buffer,
                0,
                VK_INDEX_TYPE_UINT32);
            vkCmdDrawIndexed(commandBuffer, surfelVertexCount, 1, 0, 0, 0);
        } else {
            vkCmdDraw(commandBuffer, surfelVertexCount, 1, 0, 0);
        }
        return true;
    }

    const std::array<VkBuffer, 2> vertexBuffers = {
        resources->positionBuffer.buffer,
        resources->colorBuffer.buffer,
    };
    constexpr std::array<VkDeviceSize, 2> offsets = {0, 0};
    vkCmdBindVertexBuffers(
        commandBuffer,
        0,
        static_cast<std::uint32_t>(vertexBuffers.size()),
        vertexBuffers.data(),
        offsets.data());

    if (layer.style.gpuBackToFrontSorting &&
        resources->depthSortedIndexBuffer.buffer != VK_NULL_HANDLE &&
        resources->depthSortCacheValid) {
        // depthSortCacheValid proves a sort dispatch for the current cache
        // key was recorded (or validly reused); without it the sorted index
        // buffer may hold undefined or differently-sized contents, and the
        // unsorted fallbacks below stay correct.
        vkCmdBindIndexBuffer(
            commandBuffer,
            resources->depthSortedIndexBuffer.buffer,
            0,
            VK_INDEX_TYPE_UINT32);
        vkCmdDrawIndexed(
            commandBuffer,
            plan.drawPointCount,
            1,
            0,
            0,
            0);
    } else if (plan.sampledBudgetReady) {
        vkCmdBindIndexBuffer(
            commandBuffer,
            resources->sampledIndexBuffer.buffer,
            0,
            VK_INDEX_TYPE_UINT32);
        vkCmdDrawIndexed(commandBuffer, plan.drawPointCount, 1, 0, 0, 0);
    } else if (plan.interactiveSampleReady) {
        vkCmdBindIndexBuffer(
            commandBuffer,
            resources->interactiveSampledIndexBuffer.buffer,
            0,
            VK_INDEX_TYPE_UINT32);
        vkCmdDrawIndexed(commandBuffer, plan.drawPointCount, 1, 0, 0, 0);
    } else {
        vkCmdDraw(commandBuffer, plan.drawPointCount, 1, 0, 0);
    }
    return true;
}

bool VulkanViewportShell::RecordPointCloudHighlightDraw(
    VkCommandBuffer commandBuffer,
    const SceneRenderState::PointCloudLayerState& layer,
    const ActivePointCloudResources::PointHighlightResources& highlight,
    VkPipeline spritePipeline,
    VkPipeline surfelPipeline,
    std::size_t frameIndex,
    std::uint32_t imageIndex,
    std::uint32_t* recordedDrawPointCount) {
    if (recordedDrawPointCount != nullptr) {
        *recordedDrawPointCount = 0;
    }
    auto* resources = FindPointCloudResources(layer.layerId);
    if (resources == nullptr ||
        highlight.indexCount == 0 ||
        highlight.indexBuffer.buffer == VK_NULL_HANDLE ||
        frameIndex >= kFramesInFlight ||
        imageIndex >= highlight.descriptorSets[frameIndex].size()) {
        return false;
    }

    SceneRenderState::PointCloudLayerState highlightLayer = layer;
    highlightLayer.scalarFields.clear();
    highlightLayer.hasSourceRgb = false;
    highlightLayer.drawPointCount = highlight.indexCount;
    highlightLayer.style.colorMode = renderer::pointcloud::PointCloudColorMode::SolidColor;
    const float pulse =
        highlight.style.pulseAlpha
            ? static_cast<float>(0.68 + (0.32 * std::sin(std::max(0.0F, renderState_.flowTimeSeconds) * 5.4F)))
            : 1.0F;
    const float alpha = std::clamp(highlight.style.color[3] * pulse, 0.05F, 1.0F);
    highlightLayer.style.solidColor = {
        highlight.style.color[0],
        highlight.style.color[1],
        highlight.style.color[2],
        alpha,
    };
    highlightLayer.style.colorizeAmount = 0.0F;
    highlightLayer.style.flowAnimation = false;
    highlightLayer.style.waterPathView = false;
    highlightLayer.style.waterTrailOverlay = false;
    highlightLayer.style.roughnessMotionStrength = 0.0F;
    highlightLayer.style.stylisationMode = renderer::pointcloud::PointCloudStylisationMode::Off;
    invisible_places::style::SetScalarConstant(
        &highlightLayer.style.pointSize,
        std::max(0.25F, layer.style.pointSize.constantValue[0]));
    invisible_places::style::SetScalarConstant(
        &highlightLayer.style.surfelDiameter,
        std::max(0.0001F, layer.style.surfelDiameter.constantValue[0]));
    invisible_places::style::SetScalarConstant(&highlightLayer.style.opacity, alpha);
    invisible_places::style::SetScalarConstant(&highlightLayer.style.emissiveStrength, 0.12F);
    invisible_places::style::SetScalarConstant(&highlightLayer.style.depthFade, 0.0F);
    invisible_places::style::SetScalarConstant(&highlightLayer.style.colormapPosition, 0.5F);

    const bool worldSurfels =
        highlightLayer.style.geometryMode != renderer::pointcloud::PointCloudGeometryMode::ScreenSprites;
    if (worldSurfels) {
        if (surfelPipeline == VK_NULL_HANDLE || highlight.surfelIndexBuffer.buffer == VK_NULL_HANDLE) {
            return false;
        }
        highlightLayer.drawPointCount = std::min(highlight.indexCount, kMaxSurfelEncodedPointCount);
    } else if (spritePipeline == VK_NULL_HANDLE) {
        return false;
    }

    PointCloudDrawPlan plan;
    plan.resources = resources;
    plan.drawPointCount = highlightLayer.drawPointCount;
    plan.worldSurfels = worldSurfels;
    const VkDescriptorSet descriptorSet =
        highlight.descriptorSets[frameIndex][imageIndex];
    if (descriptorSet == VK_NULL_HANDLE) {
        return false;
    }
    const bool highlightSeepageDescriptorReady =
        imageIndex <
            highlight.seepageDescriptorGenerations[frameIndex].size() &&
        highlight.seepageDescriptorGenerations[frameIndex][imageIndex] ==
            resources->seepageTopologyGeneration &&
        resources->seepageParamsFrameGenerations[frameIndex] ==
            resources->seepageParamsGeneration;
    if (!UploadPointCloudLayerStyle(
            highlightLayer,
            plan,
            frameIndex,
            imageIndex,
            false,
            &highlight.styleBuffers[frameIndex],
            &highlightSeepageDescriptorReady)) {
        return false;
    }

    vkCmdBindPipeline(
        commandBuffer,
        VK_PIPELINE_BIND_POINT_GRAPHICS,
        worldSurfels ? surfelPipeline : spritePipeline);

    vkCmdBindDescriptorSets(
        commandBuffer,
        VK_PIPELINE_BIND_POINT_GRAPHICS,
        pointPipelineLayout_,
        0,
        1,
        &descriptorSet,
        0,
        nullptr);

    if (recordedDrawPointCount != nullptr) {
        *recordedDrawPointCount = highlightLayer.drawPointCount;
    }

    if (worldSurfels) {
        const std::uint32_t surfelVertexCount = highlightLayer.drawPointCount * kSurfelVerticesPerPoint;
        vkCmdBindIndexBuffer(
            commandBuffer,
            highlight.surfelIndexBuffer.buffer,
            0,
            VK_INDEX_TYPE_UINT32);
        vkCmdDrawIndexed(commandBuffer, surfelVertexCount, 1, 0, 0, 0);
        return true;
    }

    const std::array<VkBuffer, 2> vertexBuffers = {
        resources->positionBuffer.buffer,
        resources->colorBuffer.buffer,
    };
    constexpr std::array<VkDeviceSize, 2> offsets = {0, 0};
    vkCmdBindVertexBuffers(
        commandBuffer,
        0,
        static_cast<std::uint32_t>(vertexBuffers.size()),
        vertexBuffers.data(),
        offsets.data());
    vkCmdBindIndexBuffer(commandBuffer, highlight.indexBuffer.buffer, 0, VK_INDEX_TYPE_UINT32);
    vkCmdDrawIndexed(commandBuffer, highlightLayer.drawPointCount, 1, 0, 0, 0);
    return true;
}

void VulkanViewportShell::RecordExrExportCommandBuffer(const PointCloudExrFrameRequest& request) {
    auto& resources = exrExportResources_;
    if (resources.commandBuffer == VK_NULL_HANDLE ||
        resources.renderPass == VK_NULL_HANDLE ||
        resources.framebuffer == VK_NULL_HANDLE) {
        throw std::runtime_error{"GPU EXR export resources are not initialized."};
    }
    const auto forceFullSourceForLayer = [&](const auto& layer) {
        return !request.previewDensity &&
               !PointCloudLayerUsesFastBasicRenderer(
                   renderState_.pointCloudRendererMode,
                   layer);
    };
    for (const auto& layer : renderState_.pointCloudLayers) {
        if (layer.style.gpuBackToFrontSorting &&
            layer.style.geometryMode ==
                renderer::pointcloud::PointCloudGeometryMode::ScreenSprites &&
            !PointCloudLayerUsesFastBasicRenderer(
                renderState_.pointCloudRendererMode,
                layer)) {
            PointCloudDrawPlan plan;
            if (ResolvePointCloudDrawPlan(
                    layer,
                    forceFullSourceForLayer(layer),
                    false,
                    &plan)) {
                EnsurePointDepthSortResources(
                    plan.resources,
                    plan.drawPointCount);
            }
        }
    }
    exrLastRecordedLayerCount_ = 0U;
    exrLastRecordedPointCount_ = 0U;

    VkCommandBufferBeginInfo beginInfo{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
    Check(vkBeginCommandBuffer(resources.commandBuffer, &beginInfo), "vkBeginCommandBuffer(exr)");

    std::array<VkClearValue, 10> clearValues{};
    clearValues[0].color = {{
        request.renderState.backgroundColor.r,
        request.renderState.backgroundColor.g,
        request.renderState.backgroundColor.b,
        request.renderState.backgroundColor.a,
    }};
    clearValues[1].depthStencil = {1.0F, 0};
    clearValues[2].color = {{0.0F, 0.0F, 0.0F, 0.0F}};
    clearValues[3].color = {{1.0F, 0.0F, 0.0F, 0.0F}};
    clearValues[4].color = {{0.0F, 0.0F, 0.0F, 0.0F}};
    clearValues[5].color = {{0.0F, 0.0F, 0.0F, 0.0F}};
    clearValues[6].color = {{0.0F, 0.0F, 0.0F, 0.0F}};
    clearValues[7].color = {{0.0F, 0.0F, 0.0F, 0.0F}};
    clearValues[8].color = {{0.0F, 0.0F, 0.0F, 0.0F}};
    clearValues[9].color = {{0.0F, 0.0F, 0.0F, 0.0F}};

    std::uint32_t drawablePointLayerCount = 0U;
    for (const auto& layer : renderState_.pointCloudLayers) {
        PointCloudDrawPlan plan;
        if (ResolvePointCloudDrawPlan(
                layer,
                forceFullSourceForLayer(layer),
                false,
                &plan)) {
            ++drawablePointLayerCount;
            static_cast<void>(UploadPointCloudLayerStyle(layer, plan, 0U, 0U, true));
        }
    }
    // A frame that would draw no point-cloud layer at all must fail loudly:
    // rain and other overlay passes record independently, so silently
    // dropping every layer produces effect-only frames that look like a
    // successful export.
    if (drawablePointLayerCount == 0U) {
        throw std::runtime_error{
            "GPU EXR export: none of the " +
            std::to_string(renderState_.pointCloudLayers.size()) +
            " point-cloud layers is drawable (GPU resources missing or "
            "empty)."};
    }
    UploadRainUniformsToBuffer(
        rainResources_.exrUniformBuffer,
        request.width,
        request.height);
    RecordRainComputeWithDescriptor(
        resources.commandBuffer,
        rainResources_.exrDescriptorSet,
        nullptr);
    for (const auto& layer : renderState_.pointCloudLayers) {
        if (!PointCloudLayerUsesFastBasicRenderer(
                renderState_.pointCloudRendererMode,
                layer)) {
            static_cast<void>(RecordPointDepthSort(
                resources.commandBuffer,
                layer,
                forceFullSourceForLayer(layer)));
        }
    }

    VkRenderPassBeginInfo renderPassInfo{VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO};
    renderPassInfo.renderPass = resources.renderPass;
    renderPassInfo.framebuffer = resources.framebuffer;
    renderPassInfo.renderArea.offset = {0, 0};
    renderPassInfo.renderArea.extent = {request.width, request.height};
    renderPassInfo.clearValueCount = static_cast<std::uint32_t>(clearValues.size());
    renderPassInfo.pClearValues = clearValues.data();

    vkCmdBeginRenderPass(resources.commandBuffer, &renderPassInfo, VK_SUBPASS_CONTENTS_INLINE);

    const VkViewport viewport{
        0.0F,
        0.0F,
        static_cast<float>(request.width),
        static_cast<float>(request.height),
        0.0F,
        1.0F,
    };
    const VkRect2D scissor{{0, 0}, {request.width, request.height}};
    vkCmdSetViewport(resources.commandBuffer, 0, 1, &viewport);
    vkCmdSetScissor(resources.commandBuffer, 0, 1, &scissor);

    for (const auto& layer : renderState_.pointCloudLayers) {
        if (PointCloudLayerUsesFastBasicRenderer(
                renderState_.pointCloudRendererMode,
                layer)) {
            static_cast<void>(RecordPointCloudLayerDraw(
                resources.commandBuffer,
                layer,
                forceFullSourceForLayer(layer),
                resources.pointFastBasicDepthPipeline,
                VK_NULL_HANDLE,
                false,
                0U,
                0U,
                true));
            continue;
        }
        const auto materialVariant =
            ResolvePointCloudLayerMaterialVariant(layer);
        const bool opaqueHardDisc =
            materialVariant == renderer::pointcloud::PointCloudMaterialVariant::OpaqueHardDisc;
        const bool sortedWithoutPrepass =
            layer.style.gpuBackToFrontSorting &&
            layer.style.geometryMode ==
                renderer::pointcloud::PointCloudGeometryMode::ScreenSprites &&
            !layer.style.depthPrepassEnabled;
        const bool softDepthWrites =
            renderer::pointcloud::PointCloudDepthPrepassWrites(layer.style);
        if (opaqueHardDisc || softDepthWrites ||
            (!layer.style.depthPrepassEnabled &&
             renderState_.eyeDomeLightingEnabled) ||
            sortedWithoutPrepass) {
            static_cast<void>(RecordPointCloudLayerDraw(
                resources.commandBuffer,
                layer,
                forceFullSourceForLayer(layer),
                sortedWithoutPrepass
                    ? resources.pointSortedDepthAovPipeline
                    : resources.pointDepthPipeline,
                resources.surfelDepthPipeline,
                false,
                0U,
                0U,
                true));
        }
    }

    vkCmdNextSubpass(resources.commandBuffer, VK_SUBPASS_CONTENTS_INLINE);
    vkCmdSetViewport(resources.commandBuffer, 0, 1, &viewport);
    vkCmdSetScissor(resources.commandBuffer, 0, 1, &scissor);

    for (const auto& layer : renderState_.pointCloudLayers) {
        if (PointCloudLayerUsesFastBasicRenderer(
                renderState_.pointCloudRendererMode,
                layer)) {
            continue;
        }
        if (layer.style.gpuBackToFrontSorting &&
            layer.style.geometryMode ==
                renderer::pointcloud::PointCloudGeometryMode::ScreenSprites) {
            continue;
        }
        VkPipeline spritePipeline = resources.pointAccumulationPipeline;
        VkPipeline surfelPipeline = resources.surfelAccumulationPipeline;
        if (ResolvePointCloudLayerMaterialVariant(layer) ==
            renderer::pointcloud::PointCloudMaterialVariant::ConstantSimple) {
            spritePipeline = resources.pointConstantSimpleAccumulationPipeline;
            surfelPipeline = resources.surfelConstantSimpleAccumulationPipeline;
        }
        std::uint32_t recordedDrawPointCount = 0U;
        if (RecordPointCloudLayerDraw(
                resources.commandBuffer,
                layer,
                forceFullSourceForLayer(layer),
                spritePipeline,
                surfelPipeline,
                false,
                0U,
                0U,
                true,
                &recordedDrawPointCount)) {
            ++exrLastRecordedLayerCount_;
            exrLastRecordedPointCount_ += recordedDrawPointCount;
        }
    }
    RecordRainDrawWithDescriptor(
        resources.commandBuffer,
        rainResources_.exrDescriptorSet,
        resources.rainPipeline);

    vkCmdNextSubpass(resources.commandBuffer, VK_SUBPASS_CONTENTS_INLINE);
    vkCmdSetViewport(resources.commandBuffer, 0, 1, &viewport);
    vkCmdSetScissor(resources.commandBuffer, 0, 1, &scissor);

    if (resources.compositeDescriptorSet != VK_NULL_HANDLE) {
        vkCmdBindPipeline(resources.commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, resources.compositePipeline);
        vkCmdBindDescriptorSets(
            resources.commandBuffer,
            VK_PIPELINE_BIND_POINT_GRAPHICS,
            compositePipelineLayout_,
            0,
            1,
            &resources.compositeDescriptorSet,
            0,
            nullptr);
        vkCmdDraw(resources.commandBuffer, 3, 1, 0, 0);
    }

    vkCmdNextSubpass(resources.commandBuffer, VK_SUBPASS_CONTENTS_INLINE);
    vkCmdSetViewport(resources.commandBuffer, 0, 1, &viewport);
    vkCmdSetScissor(resources.commandBuffer, 0, 1, &scissor);

    for (const auto& layer : renderState_.pointCloudLayers) {
        if (!PointCloudLayerUsesFastBasicRenderer(
                renderState_.pointCloudRendererMode,
                layer)) {
            continue;
        }
        std::uint32_t recordedDrawPointCount = 0U;
        if (RecordPointCloudLayerDraw(
                resources.commandBuffer,
                layer,
                forceFullSourceForLayer(layer),
                resources.pointFastBasicPipeline,
                VK_NULL_HANDLE,
                false,
                0U,
                0U,
                true,
                &recordedDrawPointCount)) {
            ++exrLastRecordedLayerCount_;
            exrLastRecordedPointCount_ += recordedDrawPointCount;
        }
    }

    vkCmdNextSubpass(resources.commandBuffer, VK_SUBPASS_CONTENTS_INLINE);
    vkCmdSetViewport(resources.commandBuffer, 0, 1, &viewport);
    vkCmdSetScissor(resources.commandBuffer, 0, 1, &scissor);

    const bool hasSortedPointLayers = std::any_of(
        renderState_.pointCloudLayers.begin(),
        renderState_.pointCloudLayers.end(),
        [&](const auto& layer) {
            return !PointCloudLayerUsesFastBasicRenderer(
                       renderState_.pointCloudRendererMode,
                       layer) &&
                   layer.style.gpuBackToFrontSorting &&
                   layer.style.geometryMode ==
                       renderer::pointcloud::PointCloudGeometryMode::
                           ScreenSprites;
        });
    if (hasSortedPointLayers) {
        // The emission attachment still holds subpass 1's weighted values,
        // which the weighted composite already applied. Reset it so the
        // sorted emission composite adds only sorted-layer emission.
        VkClearAttachment emissionClear{};
        emissionClear.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        emissionClear.colorAttachment = 3U;
        emissionClear.clearValue.color = {{0.0F, 0.0F, 0.0F, 0.0F}};
        VkClearRect emissionClearRect{};
        emissionClearRect.rect = scissor;
        emissionClearRect.baseArrayLayer = 0U;
        emissionClearRect.layerCount = 1U;
        vkCmdClearAttachments(
            resources.commandBuffer,
            1U,
            &emissionClear,
            1U,
            &emissionClearRect);
    }
    for (const auto& layer : renderState_.pointCloudLayers) {
        if (PointCloudLayerUsesFastBasicRenderer(
                renderState_.pointCloudRendererMode,
                layer) ||
            !layer.style.gpuBackToFrontSorting ||
            layer.style.geometryMode !=
                renderer::pointcloud::PointCloudGeometryMode::ScreenSprites) {
            continue;
        }
        std::uint32_t recordedDrawPointCount = 0U;
        const VkPipeline sortedPipeline =
            renderer::pointcloud::PointCloudDepthPrepassTests(layer.style)
                ? resources.pointSortedAlphaHybridPipeline
                : resources.pointSortedAlphaPipeline;
        if (RecordPointCloudLayerDraw(
                resources.commandBuffer,
                layer,
                forceFullSourceForLayer(layer),
                sortedPipeline,
                VK_NULL_HANDLE,
                false,
                0U,
                0U,
                true,
                &recordedDrawPointCount)) {
            ++exrLastRecordedLayerCount_;
            exrLastRecordedPointCount_ += recordedDrawPointCount;
        }
    }

    vkCmdNextSubpass(resources.commandBuffer, VK_SUBPASS_CONTENTS_INLINE);
    vkCmdSetViewport(resources.commandBuffer, 0, 1, &viewport);
    vkCmdSetScissor(resources.commandBuffer, 0, 1, &scissor);
    if (hasSortedPointLayers &&
        resources.pointSortedEmissionCompositePipeline != VK_NULL_HANDLE &&
        resources.compositeDescriptorSet != VK_NULL_HANDLE) {
        vkCmdBindPipeline(
            resources.commandBuffer,
            VK_PIPELINE_BIND_POINT_GRAPHICS,
            resources.pointSortedEmissionCompositePipeline);
        vkCmdBindDescriptorSets(
            resources.commandBuffer,
            VK_PIPELINE_BIND_POINT_GRAPHICS,
            compositePipelineLayout_,
            0,
            1,
            &resources.compositeDescriptorSet,
            0,
            nullptr);
        vkCmdDraw(resources.commandBuffer, 3, 1, 0, 0);
    }

    vkCmdEndRenderPass(resources.commandBuffer);

    const bool applyEyeDomeLighting =
        request.renderState.eyeDomeLightingEnabled &&
        HasPointCloudExrReadback(request.readbackMask, PointCloudExrReadbackMask::Color);
    if (applyEyeDomeLighting) {
        if (eyeDomeLightingExportPipeline_ == VK_NULL_HANDLE ||
            eyeDomeLightingExportPipelineLayout_ == VK_NULL_HANDLE ||
            resources.eyeDomeLightingDescriptorSet == VK_NULL_HANDLE) {
            throw std::runtime_error{"GPU export EDL resources are not initialized."};
        }

        std::array<VkImageMemoryBarrier, 2> shaderReadBarriers{};
        shaderReadBarriers[0] = {VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER};
        shaderReadBarriers[0].srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
        shaderReadBarriers[0].dstAccessMask =
            VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT;
        shaderReadBarriers[0].oldLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
        shaderReadBarriers[0].newLayout = VK_IMAGE_LAYOUT_GENERAL;
        shaderReadBarriers[0].srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        shaderReadBarriers[0].dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        shaderReadBarriers[0].image = resources.colorImage.image;
        shaderReadBarriers[0].subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        shaderReadBarriers[0].subresourceRange.baseMipLevel = 0U;
        shaderReadBarriers[0].subresourceRange.levelCount = 1U;
        shaderReadBarriers[0].subresourceRange.baseArrayLayer = 0U;
        shaderReadBarriers[0].subresourceRange.layerCount = 1U;

        shaderReadBarriers[1] = {VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER};
        shaderReadBarriers[1].srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
        shaderReadBarriers[1].dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
        shaderReadBarriers[1].oldLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
        shaderReadBarriers[1].newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        shaderReadBarriers[1].srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        shaderReadBarriers[1].dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        shaderReadBarriers[1].image = resources.linearDepthImage.image;
        shaderReadBarriers[1].subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        shaderReadBarriers[1].subresourceRange.baseMipLevel = 0U;
        shaderReadBarriers[1].subresourceRange.levelCount = 1U;
        shaderReadBarriers[1].subresourceRange.baseArrayLayer = 0U;
        shaderReadBarriers[1].subresourceRange.layerCount = 1U;

        vkCmdPipelineBarrier(
            resources.commandBuffer,
            VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT,
            VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
            0U,
            0U,
            nullptr,
            0U,
            nullptr,
            static_cast<std::uint32_t>(shaderReadBarriers.size()),
            shaderReadBarriers.data());

        vkCmdBindPipeline(
            resources.commandBuffer,
            VK_PIPELINE_BIND_POINT_COMPUTE,
            eyeDomeLightingExportPipeline_);
        vkCmdBindDescriptorSets(
            resources.commandBuffer,
            VK_PIPELINE_BIND_POINT_COMPUTE,
            eyeDomeLightingExportPipelineLayout_,
            0U,
            1U,
            &resources.eyeDomeLightingDescriptorSet,
            0U,
            nullptr);
        const EyeDomeLightingExportPushConstants pushConstants{
            .edl = glm::vec4{
                1.0F,
                24.0F,
                0.35F,
                std::clamp(
                    request.renderState.eyeDomeLightingThickness,
                    0.0F,
                    24.0F),
            },
        };
        vkCmdPushConstants(
            resources.commandBuffer,
            eyeDomeLightingExportPipelineLayout_,
            VK_SHADER_STAGE_COMPUTE_BIT,
            0U,
            sizeof(EyeDomeLightingExportPushConstants),
            &pushConstants);
        constexpr std::uint32_t kWorkgroupSize = 16U;
        vkCmdDispatch(
            resources.commandBuffer,
            (request.width + kWorkgroupSize - 1U) / kWorkgroupSize,
            (request.height + kWorkgroupSize - 1U) / kWorkgroupSize,
            1U);

        std::array<VkImageMemoryBarrier, 2> transferBarriers{};
        transferBarriers[0] = {VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER};
        transferBarriers[0].srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
        transferBarriers[0].dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
        transferBarriers[0].oldLayout = VK_IMAGE_LAYOUT_GENERAL;
        transferBarriers[0].newLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
        transferBarriers[0].srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        transferBarriers[0].dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        transferBarriers[0].image = resources.colorImage.image;
        transferBarriers[0].subresourceRange = shaderReadBarriers[0].subresourceRange;

        std::uint32_t transferBarrierCount = 1U;
        if (HasPointCloudExrReadback(
                request.readbackMask,
                PointCloudExrReadbackMask::Depth)) {
            transferBarriers[1] = {VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER};
            transferBarriers[1].srcAccessMask = VK_ACCESS_SHADER_READ_BIT;
            transferBarriers[1].dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
            transferBarriers[1].oldLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
            transferBarriers[1].newLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
            transferBarriers[1].srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            transferBarriers[1].dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            transferBarriers[1].image = resources.linearDepthImage.image;
            transferBarriers[1].subresourceRange = shaderReadBarriers[1].subresourceRange;
            transferBarrierCount = 2U;
        }

        vkCmdPipelineBarrier(
            resources.commandBuffer,
            VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
            VK_PIPELINE_STAGE_TRANSFER_BIT,
            0U,
            0U,
            nullptr,
            0U,
            nullptr,
            transferBarrierCount,
            transferBarriers.data());
    }

    VkBufferImageCopy colorCopyRegion{};
    colorCopyRegion.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    colorCopyRegion.imageSubresource.mipLevel = 0;
    colorCopyRegion.imageSubresource.baseArrayLayer = 0;
    colorCopyRegion.imageSubresource.layerCount = 1;
    colorCopyRegion.imageExtent = {request.width, request.height, 1};
    if (HasPointCloudExrReadback(request.readbackMask, PointCloudExrReadbackMask::Color)) {
        vkCmdCopyImageToBuffer(
            resources.commandBuffer,
            resources.colorImage.image,
            VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
            resources.colorReadbackBuffer.buffer,
            1,
            &colorCopyRegion);
    }

    VkBufferImageCopy depthCopyRegion{};
    depthCopyRegion.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    depthCopyRegion.imageSubresource.mipLevel = 0;
    depthCopyRegion.imageSubresource.baseArrayLayer = 0;
    depthCopyRegion.imageSubresource.layerCount = 1;
    depthCopyRegion.imageExtent = {request.width, request.height, 1};
    if (HasPointCloudExrReadback(request.readbackMask, PointCloudExrReadbackMask::Depth)) {
        vkCmdCopyImageToBuffer(
            resources.commandBuffer,
            resources.linearDepthImage.image,
            VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
            resources.depthReadbackBuffer.buffer,
            1,
            &depthCopyRegion);
    }

    VkBufferImageCopy normalCopyRegion{};
    normalCopyRegion.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    normalCopyRegion.imageSubresource.mipLevel = 0;
    normalCopyRegion.imageSubresource.baseArrayLayer = 0;
    normalCopyRegion.imageSubresource.layerCount = 1;
    normalCopyRegion.imageExtent = {request.width, request.height, 1};
    if (HasPointCloudExrReadback(request.readbackMask, PointCloudExrReadbackMask::Normal)) {
        vkCmdCopyImageToBuffer(
            resources.commandBuffer,
            resources.normalImage.image,
            VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
            resources.normalReadbackBuffer.buffer,
            1,
            &normalCopyRegion);
    }

    VkBufferImageCopy albedoCopyRegion{};
    albedoCopyRegion.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    albedoCopyRegion.imageSubresource.mipLevel = 0;
    albedoCopyRegion.imageSubresource.baseArrayLayer = 0;
    albedoCopyRegion.imageSubresource.layerCount = 1;
    albedoCopyRegion.imageExtent = {request.width, request.height, 1};
    if (HasPointCloudExrReadback(request.readbackMask, PointCloudExrReadbackMask::Albedo)) {
        vkCmdCopyImageToBuffer(
            resources.commandBuffer,
            resources.albedoImage.image,
            VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
            resources.albedoReadbackBuffer.buffer,
            1,
            &albedoCopyRegion);
    }

    Check(vkEndCommandBuffer(resources.commandBuffer), "vkEndCommandBuffer(exr)");
}

void VulkanViewportShell::RecordTemporalCameraOverlayHistory(
    VkCommandBuffer commandBuffer,
    std::uint32_t imageIndex,
    bool sceneRendered) {
    const bool resourcesReady =
        temporalCameraOverlayColorImages_[0U].image != VK_NULL_HANDLE &&
        temporalCameraOverlayColorImages_[1U].image != VK_NULL_HANDLE &&
        temporalCameraOverlayDepthImages_[0U].image != VK_NULL_HANDLE &&
        temporalCameraOverlayDepthImages_[1U].image != VK_NULL_HANDLE;
    if (!resourcesReady) {
        return;
    }
    if (!temporalCameraOverlayResourcesInitialized_) {
        std::array<VkImageMemoryBarrier, 4U> initializeBarriers{};
        for (std::size_t sourceIndex = 0U;
             sourceIndex < temporalCameraOverlayColorImages_.size();
             ++sourceIndex) {
            auto initializeImage = [&](VkImage image,
                                       VkImageMemoryBarrier* barrier) {
                *barrier = {VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER};
                barrier->srcAccessMask = 0U;
                barrier->dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
                barrier->oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
                barrier->newLayout =
                    VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
                barrier->srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
                barrier->dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
                barrier->image = image;
                barrier->subresourceRange.aspectMask =
                    VK_IMAGE_ASPECT_COLOR_BIT;
                barrier->subresourceRange.baseMipLevel = 0U;
                barrier->subresourceRange.levelCount = 1U;
                barrier->subresourceRange.baseArrayLayer = 0U;
                barrier->subresourceRange.layerCount = 1U;
            };
            initializeImage(
                temporalCameraOverlayColorImages_[sourceIndex].image,
                &initializeBarriers[sourceIndex * 2U]);
            initializeImage(
                temporalCameraOverlayDepthImages_[sourceIndex].image,
                &initializeBarriers[sourceIndex * 2U + 1U]);
        }
        vkCmdPipelineBarrier(
            commandBuffer,
            VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,
            VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT |
                VK_PIPELINE_STAGE_TRANSFER_BIT,
            0U,
            0U,
            nullptr,
            0U,
            nullptr,
            static_cast<std::uint32_t>(initializeBarriers.size()),
            initializeBarriers.data());
        temporalCameraOverlayResourcesInitialized_ = true;
    }

    if (!temporalCameraOverlayEnabled_ || !sceneRendered ||
        imageIndex >= sceneColorImages_.size() ||
        imageIndex >= linearDepthImages_.size()) {
        return;
    }

    const std::size_t sourceIndex = std::min<std::size_t>(
        temporalCameraOverlayRenderedSourceIndex_,
        temporalCameraOverlayColorImages_.size() - 1U);
    std::array<VkImageMemoryBarrier, 4U> copyBarriers{};
    const auto prepareCopy = [](VkImage image,
                                VkImageLayout newLayout,
                                VkAccessFlags destinationAccess,
                                VkImageMemoryBarrier* barrier) {
        *barrier = {VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER};
        barrier->srcAccessMask =
            VK_ACCESS_MEMORY_READ_BIT | VK_ACCESS_MEMORY_WRITE_BIT;
        barrier->dstAccessMask = destinationAccess;
        barrier->oldLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        barrier->newLayout = newLayout;
        barrier->srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barrier->dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barrier->image = image;
        barrier->subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        barrier->subresourceRange.baseMipLevel = 0U;
        barrier->subresourceRange.levelCount = 1U;
        barrier->subresourceRange.baseArrayLayer = 0U;
        barrier->subresourceRange.layerCount = 1U;
    };
    prepareCopy(
        sceneColorImages_[imageIndex].image,
        VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
        VK_ACCESS_TRANSFER_READ_BIT,
        &copyBarriers[0U]);
    prepareCopy(
        linearDepthImages_[imageIndex].image,
        VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
        VK_ACCESS_TRANSFER_READ_BIT,
        &copyBarriers[1U]);
    prepareCopy(
        temporalCameraOverlayColorImages_[sourceIndex].image,
        VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
        VK_ACCESS_TRANSFER_WRITE_BIT,
        &copyBarriers[2U]);
    prepareCopy(
        temporalCameraOverlayDepthImages_[sourceIndex].image,
        VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
        VK_ACCESS_TRANSFER_WRITE_BIT,
        &copyBarriers[3U]);
    vkCmdPipelineBarrier(
        commandBuffer,
        VK_PIPELINE_STAGE_ALL_COMMANDS_BIT,
        VK_PIPELINE_STAGE_TRANSFER_BIT,
        0U,
        0U,
        nullptr,
        0U,
        nullptr,
        static_cast<std::uint32_t>(copyBarriers.size()),
        copyBarriers.data());

    VkImageCopy copyRegion{};
    copyRegion.srcSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    copyRegion.srcSubresource.mipLevel = 0U;
    copyRegion.srcSubresource.baseArrayLayer = 0U;
    copyRegion.srcSubresource.layerCount = 1U;
    copyRegion.dstSubresource = copyRegion.srcSubresource;
    copyRegion.extent = {swapchainWidth_, swapchainHeight_, 1U};
    vkCmdCopyImage(
        commandBuffer,
        sceneColorImages_[imageIndex].image,
        VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
        temporalCameraOverlayColorImages_[sourceIndex].image,
        VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
        1U,
        &copyRegion);
    vkCmdCopyImage(
        commandBuffer,
        linearDepthImages_[imageIndex].image,
        VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
        temporalCameraOverlayDepthImages_[sourceIndex].image,
        VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
        1U,
        &copyRegion);

    for (std::size_t barrierIndex = 0U;
         barrierIndex < copyBarriers.size();
         ++barrierIndex) {
        auto& barrier = copyBarriers[barrierIndex];
        const bool sourceImage = barrierIndex < 2U;
        barrier.srcAccessMask = sourceImage
                                    ? VK_ACCESS_TRANSFER_READ_BIT
                                    : VK_ACCESS_TRANSFER_WRITE_BIT;
        barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
        barrier.oldLayout = sourceImage
                                ? VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL
                                : VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
        barrier.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    }
    vkCmdPipelineBarrier(
        commandBuffer,
        VK_PIPELINE_STAGE_TRANSFER_BIT,
        VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
        0U,
        0U,
        nullptr,
        0U,
        nullptr,
        static_cast<std::uint32_t>(copyBarriers.size()),
        copyBarriers.data());
    temporalCameraOverlayHistoryViews_[sourceIndex] = renderState_.view;
    temporalCameraOverlayHistoryProjections_[sourceIndex] =
        renderState_.projection;
    temporalCameraOverlayHistoryReferenceDepths_[sourceIndex] =
        std::max(0.001F, renderState_.focusDistance);
    temporalCameraOverlayHistoryValid_[sourceIndex] = true;
}

void VulkanViewportShell::RecordCommandBuffer(
    VkCommandBuffer commandBuffer,
    std::uint32_t imageIndex,
    std::size_t frameIndex) {
    const bool collectDiagnostics = diagnosticsEnabled_;
    const auto recordStart = collectDiagnostics ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point{};
    std::uint32_t pointDrawCalls = 0;
    std::uint32_t pointDepthLayerCount = 0;
    std::uint32_t pointAccumulationLayerCount = 0;
    std::uint32_t pointStyleUploadCount = 0;
    std::uint32_t pointSkippedInactiveBindings = 0;
    std::uint32_t pointOpaqueHardDiscDrawCalls = 0;
    std::uint32_t pointConstantSimpleDrawCalls = 0;
    std::uint32_t pointUnifiedDrawCalls = 0;
    std::uint32_t pointFastBasicDrawCalls = 0;
    std::uint64_t pointFastBasicDrawnPoints = 0;
    std::uint64_t pointSubmittedCount = diagnostics_.pointSubmittedCount;
    std::uint64_t pointPassSubmittedCount = diagnostics_.pointPassSubmittedCount;

    for (const auto& layer : renderState_.pointCloudLayers) {
        if (layer.style.gpuBackToFrontSorting &&
            layer.style.geometryMode ==
                renderer::pointcloud::PointCloudGeometryMode::ScreenSprites &&
            !PointCloudLayerUsesFastBasicRenderer(
                renderState_.pointCloudRendererMode,
                layer)) {
            PointCloudDrawPlan plan;
            if (ResolvePointCloudDrawPlan(
                    layer,
                    false,
                    false,
                    &plan)) {
                EnsurePointDepthSortResources(
                    plan.resources,
                    plan.drawPointCount);
            }
        }
    }

    VkCommandBufferBeginInfo beginInfo{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
    Check(vkBeginCommandBuffer(commandBuffer, &beginInfo), "vkBeginCommandBuffer");

    const bool drawLiveScene = SceneImageNeedsRender(imageIndex);
    const bool collectGpuTimestamps =
        collectDiagnostics && gpuTimestampsSupported_ &&
        frameIndex < frameResources_.size() &&
        frameResources_[frameIndex].timestampQueryPool != VK_NULL_HANDLE;
    const VkQueryPool timestampQueryPool =
        collectGpuTimestamps
            ? frameResources_[frameIndex].timestampQueryPool
            : VK_NULL_HANDLE;
    if (collectGpuTimestamps) {
        vkCmdResetQueryPool(
            commandBuffer,
            timestampQueryPool,
            0U,
            kFrameTimestampQueryCount);
        vkCmdWriteTimestamp(
            commandBuffer,
            VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,
            timestampQueryPool,
            kTimestampFrameStart);
    }
    if (drawLiveScene) {
        pointSubmittedCount = 0;
        pointPassSubmittedCount = 0;
    }
    const bool hasFastBasicPointLayers = std::any_of(
        renderState_.pointCloudLayers.begin(),
        renderState_.pointCloudLayers.end(),
        [&](const auto& layer) {
            return PointCloudLayerUsesFastBasicRenderer(
                renderState_.pointCloudRendererMode,
                layer);
        });
    const bool hasBeautyPointLayers = std::any_of(
        renderState_.pointCloudLayers.begin(),
        renderState_.pointCloudLayers.end(),
        [&](const auto& layer) {
            return !PointCloudLayerUsesFastBasicRenderer(
                renderState_.pointCloudRendererMode,
                layer);
        });
    if (collectGpuTimestamps) {
        frameResources_[frameIndex].timestampActiveMask = 0U;
    }
    const auto markGpuPhaseActive = [&](std::uint32_t phase) {
        if (collectGpuTimestamps) {
            frameResources_[frameIndex].timestampActiveMask |= phase;
        }
    };
    const VkViewport viewport{
        0.0F,
        0.0F,
        static_cast<float>(swapchainWidth_),
        static_cast<float>(swapchainHeight_),
        0.0F,
        1.0F,
    };
    const VkRect2D scissor{{0, 0}, {swapchainWidth_, swapchainHeight_}};

    if (drawLiveScene) {
        for (const auto& layer : renderState_.pointCloudLayers) {
            PointCloudDrawPlan plan;
            if (ResolvePointCloudDrawPlan(layer, false, true, &plan) &&
                UploadPointCloudLayerStyle(layer, plan, frameIndex, imageIndex, false)) {
                if (collectDiagnostics) {
                    ++pointStyleUploadCount;
                    pointSkippedInactiveBindings += InactivePointBindingCount(layer.style);
                }
            }
        }
        if (liveRainSimulationEnabled_) {
            UploadRainUniforms(
                frameIndex,
                swapchainWidth_,
                swapchainHeight_);
        }
        const bool recordsRainCompute =
            liveRainSimulationEnabled_ &&
            frameIndex < rainResources_.descriptorSets.size() &&
            renderState_.rainSettings.enabled &&
            rainResources_.collisionReady &&
            rainComputePipeline_ != VK_NULL_HANDLE &&
            rainResources_.descriptorSets[frameIndex] != VK_NULL_HANDLE;
        if (liveRainSimulationEnabled_) {
            RecordRainCompute(commandBuffer, frameIndex);
        }
        if (recordsRainCompute) {
            markGpuPhaseActive(kGpuPhaseRainActive);
        }
    }
    if (collectGpuTimestamps) {
        vkCmdWriteTimestamp(
            commandBuffer,
            VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT,
            timestampQueryPool,
            kTimestampRainEnd);
    }
    if (collectGpuTimestamps && !drawLiveScene) {
        for (std::uint32_t query = kTimestampDepthEnd;
             query <= kTimestampOpaqueEnd;
             ++query) {
            vkCmdWriteTimestamp(
                commandBuffer,
                VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT,
                timestampQueryPool,
                query);
        }
    }

    if (drawLiveScene) {
        for (const auto& layer : renderState_.pointCloudLayers) {
            if (!PointCloudLayerUsesFastBasicRenderer(
                    renderState_.pointCloudRendererMode,
                    layer)) {
                static_cast<void>(RecordPointDepthSort(
                    commandBuffer,
                    layer,
                    false));
            }
        }

        std::array<VkClearValue, 6> clearValues{};
    clearValues[0].color = renderState_.proResAlphaPreviewEnabled
                                ? VkClearColorValue{{0.0F, 0.0F, 0.0F, 0.0F}}
                                : VkClearColorValue{{
                                      renderState_.backgroundColor.r,
                                      renderState_.backgroundColor.g,
                                      renderState_.backgroundColor.b,
                                      renderState_.backgroundColor.a,
                                  }};
    clearValues[1].depthStencil = {1.0F, 0};
    clearValues[2].color = {{0.0F, 0.0F, 0.0F, 0.0F}};
    clearValues[3].color = {{1.0F, 0.0F, 0.0F, 0.0F}};
    clearValues[4].color = {{0.0F, 0.0F, 0.0F, 0.0F}};
    clearValues[5].color = {{0.0F, 0.0F, 0.0F, 0.0F}};

    VkRenderPassBeginInfo renderPassInfo{VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO};
    renderPassInfo.renderPass = renderPass_;
    renderPassInfo.framebuffer = framebuffers_[imageIndex];
    renderPassInfo.renderArea.offset = {0, 0};
    renderPassInfo.renderArea.extent = {swapchainWidth_, swapchainHeight_};
    renderPassInfo.clearValueCount = static_cast<std::uint32_t>(clearValues.size());
    renderPassInfo.pClearValues = clearValues.data();

    vkCmdBeginRenderPass(commandBuffer, &renderPassInfo, VK_SUBPASS_CONTENTS_INLINE);

    vkCmdSetViewport(commandBuffer, 0, 1, &viewport);
    vkCmdSetScissor(commandBuffer, 0, 1, &scissor);

    if (drawLiveScene &&
        (hasBeautyPointLayers || liveSceneReadbackCaptureEnabled_) &&
        !renderState_.pointCloudLayers.empty()) {
        for (const auto& layer : renderState_.pointCloudLayers) {
            const bool layerUsesFastBasic =
                PointCloudLayerUsesFastBasicRenderer(
                    renderState_.pointCloudRendererMode,
                    layer);
            if (layerUsesFastBasic &&
                !liveSceneReadbackCaptureEnabled_) {
                continue;
            }
            const auto materialVariant =
                ResolvePointCloudLayerMaterialVariant(layer);
            const bool opaqueHardDisc =
                materialVariant == renderer::pointcloud::PointCloudMaterialVariant::OpaqueHardDisc;
            const bool sortedWithoutPrepass =
                layer.style.gpuBackToFrontSorting &&
                layer.style.geometryMode ==
                    renderer::pointcloud::PointCloudGeometryMode::ScreenSprites &&
                !layer.style.depthPrepassEnabled;
            const bool softDepthWrites =
                renderer::pointcloud::PointCloudDepthPrepassWrites(
                    layer.style);
            // Depth-tested accumulation needs this layer's depth in the
            // prepass even when eye-dome lighting is off.
            if (opaqueHardDisc || softDepthWrites ||
                (!layer.style.depthPrepassEnabled &&
                 (renderState_.eyeDomeLightingEnabled ||
                  liveSceneReadbackCaptureEnabled_ ||
                  LayerUsesDepthTestedAccumulation(
                      layer, materialVariant)))) {
                if (collectDiagnostics) {
                    ++pointDepthLayerCount;
                }
                std::uint32_t recordedDrawPointCount = 0;
                if (RecordPointCloudLayerDraw(
                    commandBuffer,
                    layer,
                    false,
                    sortedWithoutPrepass
                        ? pointSortedDepthAovPipeline_
                        : pointDepthPrepassPipeline_,
                    surfelDepthPrepassPipeline_,
                    false,
                    frameIndex,
                    imageIndex,
                    false,
                    &recordedDrawPointCount)) {
                    markGpuPhaseActive(kGpuPhaseDepthActive);
                    if (collectDiagnostics) {
                        ++pointDrawCalls;
                        pointPassSubmittedCount += recordedDrawPointCount;
                    }
                }
            }
        }
    }

    if (collectGpuTimestamps) {
        vkCmdWriteTimestamp(
            commandBuffer,
            VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT,
            timestampQueryPool,
            kTimestampDepthEnd);
    }
    vkCmdNextSubpass(commandBuffer, VK_SUBPASS_CONTENTS_INLINE);
    vkCmdSetViewport(commandBuffer, 0, 1, &viewport);
    vkCmdSetScissor(commandBuffer, 0, 1, &scissor);

    if (drawLiveScene && hasBeautyPointLayers && !renderState_.pointCloudLayers.empty()) {
        for (const auto& layer : renderState_.pointCloudLayers) {
            if (PointCloudLayerUsesFastBasicRenderer(
                    renderState_.pointCloudRendererMode,
                    layer)) {
                continue;
            }
            if (layer.style.gpuBackToFrontSorting &&
                layer.style.geometryMode ==
                    renderer::pointcloud::PointCloudGeometryMode::ScreenSprites) {
                continue;
            }
            const auto materialVariant =
                ResolvePointCloudLayerMaterialVariant(layer);
            if (materialVariant == renderer::pointcloud::PointCloudMaterialVariant::OpaqueHardDisc) {
                continue;
            }
            VkPipeline spritePipeline = pointAccumulationPipeline_;
            VkPipeline surfelPipeline = surfelAccumulationPipeline_;
            if (materialVariant == renderer::pointcloud::PointCloudMaterialVariant::ConstantSimple) {
                spritePipeline = pointConstantSimpleAccumulationPipeline_;
                surfelPipeline = surfelConstantSimpleAccumulationPipeline_;
            } else if (LayerUsesDepthTestedAccumulation(layer, materialVariant)) {
                spritePipeline = pointAccumulationDepthTestedPipeline_;
                surfelPipeline = surfelAccumulationDepthTestedPipeline_;
            }
            if (collectDiagnostics) {
                ++pointAccumulationLayerCount;
            }
            std::uint32_t recordedDrawPointCount = 0;
            if (RecordPointCloudLayerDraw(
                commandBuffer,
                layer,
                false,
                spritePipeline,
                surfelPipeline,
                false,
                frameIndex,
                imageIndex,
                false,
                &recordedDrawPointCount)) {
                markGpuPhaseActive(kGpuPhaseWeightedActive);
                if (collectDiagnostics) {
                    ++pointDrawCalls;
                    pointSubmittedCount += recordedDrawPointCount;
                    pointPassSubmittedCount += recordedDrawPointCount;
                    if (materialVariant == renderer::pointcloud::PointCloudMaterialVariant::ConstantSimple) {
                        ++pointConstantSimpleDrawCalls;
                    } else {
                        ++pointUnifiedDrawCalls;
                    }
                }
            }
        }
    }

    if (drawLiveScene && hasBeautyPointLayers && !renderState_.pointCloudLayers.empty()) {
        for (const auto& layer : renderState_.pointCloudLayers) {
            if (PointCloudLayerUsesFastBasicRenderer(
                    renderState_.pointCloudRendererMode,
                    layer)) {
                continue;
            }
            const auto* resources = FindPointCloudResources(layer.layerId);
            if (resources == nullptr || resources->highlights.empty()) {
                continue;
            }
            for (const auto& highlight : resources->highlights) {
                std::uint32_t recordedDrawPointCount = 0;
                if (RecordPointCloudHighlightDraw(
                    commandBuffer,
                    layer,
                    highlight,
                    pointConstantSimpleAccumulationPipeline_,
                    surfelConstantSimpleAccumulationPipeline_,
                    frameIndex,
                    imageIndex,
                    &recordedDrawPointCount)) {
                    markGpuPhaseActive(kGpuPhaseWeightedActive);
                    if (collectDiagnostics) {
                        ++pointDrawCalls;
                        ++pointConstantSimpleDrawCalls;
                        pointSubmittedCount += recordedDrawPointCount;
                        pointPassSubmittedCount += recordedDrawPointCount;
                    }
                }
            }
        }
    }

    if (drawLiveScene) {
        const bool recordsRainDraw =
            liveRainSimulationEnabled_ &&
            frameIndex < rainResources_.descriptorSets.size() &&
            renderState_.rainSettings.enabled &&
            rainResources_.collisionReady &&
            rainPipeline_ != VK_NULL_HANDLE &&
            rainResources_.descriptorSets[frameIndex] != VK_NULL_HANDLE;
        if (liveRainSimulationEnabled_) {
            RecordRainDraw(
                commandBuffer,
                frameIndex,
                rainPipeline_);
        }
        if (recordsRainDraw) {
            markGpuPhaseActive(kGpuPhaseWeightedActive);
        }
    }

    if (drawLiveScene && !renderState_.gaussianSplatLayers.empty()) {
        vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, gaussianSplatPipeline_);

        for (const auto& layer : renderState_.gaussianSplatLayers) {
            if (layer.style.qualityMode == renderer::gsplat::GaussianSplatQualityMode::High) {
                continue;
            }

            const auto* resources = FindGaussianSplatResources(layer.layerId);
            if (resources == nullptr ||
                resources->splatCount == 0 ||
                frameIndex >= kFramesInFlight ||
                imageIndex >= resources->descriptorSets[frameIndex].size()) {
                continue;
            }
            VkDescriptorSet descriptorSet = resources->descriptorSets[frameIndex][imageIndex];
            if (descriptorSet == VK_NULL_HANDLE) {
                continue;
            }

            std::uint32_t weightedQualityMode = 0U;
            if (layer.style.qualityMode == renderer::gsplat::GaussianSplatQualityMode::Fast) {
                weightedQualityMode = 1U;
            } else if (layer.style.qualityMode == renderer::gsplat::GaussianSplatQualityMode::SurfaceGuided) {
                weightedQualityMode = 2U;
            }

            GaussianSplatPushConstants pushConstants;
            pushConstants.localToWorld = layer.localToWorld;
            pushConstants.layerTint = glm::vec4{
                layer.style.layerTint[0],
                layer.style.layerTint[1],
                layer.style.layerTint[2],
                layer.style.layerTint[3],
            };
            pushConstants.style = glm::vec4{
                layer.style.opacityMultiplier,
                layer.style.scaleMultiplier,
                layer.style.exposure,
                layer.style.saturation,
            };
            pushConstants.control = glm::uvec4{
                static_cast<std::uint32_t>(layer.style.colorMode),
                static_cast<std::uint32_t>(layer.style.debugMode),
                layer.style.transformEnabled ? 1U : 0U,
                weightedQualityMode,
            };
            pushConstants.extra = glm::vec4{
                renderState_.gaussianSplatFootprintBoost,
                0.0F,
                0.0F,
                0.0F,
            };

            vkCmdBindDescriptorSets(
                commandBuffer,
                VK_PIPELINE_BIND_POINT_GRAPHICS,
                gaussianSplatPipelineLayout_,
                0,
                1,
                &descriptorSet,
                0,
                nullptr);
            vkCmdPushConstants(
                commandBuffer,
                gaussianSplatPipelineLayout_,
                VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
                0,
                sizeof(GaussianSplatPushConstants),
                &pushConstants);
            vkCmdDraw(commandBuffer, 6, resources->splatCount, 0, 0);
            markGpuPhaseActive(kGpuPhaseWeightedActive);
        }
    }

    if (collectGpuTimestamps) {
        vkCmdWriteTimestamp(
            commandBuffer,
            VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT,
            timestampQueryPool,
            kTimestampWeightedEnd);
    }
    vkCmdNextSubpass(commandBuffer, VK_SUBPASS_CONTENTS_INLINE);
    vkCmdSetViewport(commandBuffer, 0, 1, &viewport);
    vkCmdSetScissor(commandBuffer, 0, 1, &scissor);

    if (drawLiveScene &&
        imageIndex < compositeDescriptorSets_.size() &&
        compositeDescriptorSets_[imageIndex] != VK_NULL_HANDLE) {
        VkDescriptorSet descriptorSet = compositeDescriptorSets_[imageIndex];
        vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, compositePipeline_);
        vkCmdBindDescriptorSets(
            commandBuffer,
            VK_PIPELINE_BIND_POINT_GRAPHICS,
            compositePipelineLayout_,
            0,
            1,
            &descriptorSet,
            0,
            nullptr);
        vkCmdDraw(commandBuffer, 3, 1, 0, 0);
        markGpuPhaseActive(kGpuPhaseCompositeActive);
    }

    if (collectGpuTimestamps) {
        vkCmdWriteTimestamp(
            commandBuffer,
            VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT,
            timestampQueryPool,
            kTimestampCompositeEnd);
    }
    vkCmdNextSubpass(commandBuffer, VK_SUBPASS_CONTENTS_INLINE);
    vkCmdSetViewport(commandBuffer, 0, 1, &viewport);
    vkCmdSetScissor(commandBuffer, 0, 1, &scissor);

    if (drawLiveScene && hasFastBasicPointLayers && !renderState_.pointCloudLayers.empty()) {
        for (const auto& layer : renderState_.pointCloudLayers) {
            if (!PointCloudLayerUsesFastBasicRenderer(
                    renderState_.pointCloudRendererMode,
                    layer)) {
                continue;
            }
            std::uint32_t recordedDrawPointCount = 0;
            if (RecordPointCloudLayerDraw(
                commandBuffer,
                layer,
                false,
                pointFastBasicPipeline_,
                VK_NULL_HANDLE,
                false,
                frameIndex,
                imageIndex,
                false,
                &recordedDrawPointCount)) {
                markGpuPhaseActive(kGpuPhaseOpaqueActive);
                if (collectDiagnostics) {
                    ++pointDrawCalls;
                    ++pointFastBasicDrawCalls;
                    pointFastBasicDrawnPoints += recordedDrawPointCount;
                    pointSubmittedCount += recordedDrawPointCount;
                    pointPassSubmittedCount += recordedDrawPointCount;
                }
            }
            const auto* resources = FindPointCloudResources(layer.layerId);
            if (resources == nullptr || resources->highlights.empty()) {
                continue;
            }
            for (const auto& highlight : resources->highlights) {
                std::uint32_t highlightedPointCount = 0;
                if (RecordPointCloudHighlightDraw(
                    commandBuffer,
                    layer,
                    highlight,
                    pointFastBasicPipeline_,
                    VK_NULL_HANDLE,
                    frameIndex,
                    imageIndex,
                    &highlightedPointCount)) {
                    markGpuPhaseActive(kGpuPhaseOpaqueActive);
                    if (collectDiagnostics) {
                        ++pointDrawCalls;
                        ++pointFastBasicDrawCalls;
                        pointFastBasicDrawnPoints += highlightedPointCount;
                        pointSubmittedCount += highlightedPointCount;
                        pointPassSubmittedCount += highlightedPointCount;
                    }
                }
            }
        }
    }
    if (drawLiveScene && hasBeautyPointLayers && !renderState_.pointCloudLayers.empty()) {
        for (const auto& layer : renderState_.pointCloudLayers) {
            if (PointCloudLayerUsesFastBasicRenderer(
                    renderState_.pointCloudRendererMode,
                    layer)) {
                continue;
            }
            const auto materialVariant =
                ResolvePointCloudLayerMaterialVariant(layer);
            if (materialVariant != renderer::pointcloud::PointCloudMaterialVariant::OpaqueHardDisc) {
                continue;
            }
            std::uint32_t recordedDrawPointCount = 0;
            if (RecordPointCloudLayerDraw(
                commandBuffer,
                layer,
                false,
                pointOpaqueHardDiscPipeline_,
                surfelOpaqueHardDiscPipeline_,
                false,
                frameIndex,
                imageIndex,
                false,
                &recordedDrawPointCount)) {
                markGpuPhaseActive(kGpuPhaseOpaqueActive);
                if (collectDiagnostics) {
                    ++pointDrawCalls;
                    ++pointOpaqueHardDiscDrawCalls;
                    pointSubmittedCount += recordedDrawPointCount;
                    pointPassSubmittedCount += recordedDrawPointCount;
                }
            }
        }
    }

    if (drawLiveScene &&
        frameIndex < kFramesInFlight &&
        highQualityGaussianScene_.descriptorSets[frameIndex] != VK_NULL_HANDLE &&
        highQualityGaussianScene_.splatCount > 0) {
        VkDescriptorSet descriptorSet = highQualityGaussianScene_.descriptorSets[frameIndex];
        HighQualityGaussianPushConstants pushConstants;
        pushConstants.extra = glm::vec4{
            renderState_.gaussianSplatFootprintBoost,
            0.0F,
            0.0F,
            0.0F,
        };

        vkCmdBindPipeline(
            commandBuffer,
            VK_PIPELINE_BIND_POINT_GRAPHICS,
            highQualityGaussianSplatPipeline_);
        vkCmdBindDescriptorSets(
            commandBuffer,
            VK_PIPELINE_BIND_POINT_GRAPHICS,
            highQualityGaussianSplatPipelineLayout_,
            0,
            1,
            &descriptorSet,
            0,
            nullptr);
        vkCmdPushConstants(
            commandBuffer,
            highQualityGaussianSplatPipelineLayout_,
            VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
            0,
            sizeof(HighQualityGaussianPushConstants),
            &pushConstants);
        vkCmdDraw(commandBuffer, 6, highQualityGaussianScene_.splatCount, 0, 0);
        markGpuPhaseActive(kGpuPhaseOpaqueActive);
    }

    vkCmdNextSubpass(commandBuffer, VK_SUBPASS_CONTENTS_INLINE);
    vkCmdSetViewport(commandBuffer, 0, 1, &viewport);
    vkCmdSetScissor(commandBuffer, 0, 1, &scissor);
    const bool hasSortedPointLayers =
        drawLiveScene && hasBeautyPointLayers &&
        std::any_of(
            renderState_.pointCloudLayers.begin(),
            renderState_.pointCloudLayers.end(),
            [&](const auto& layer) {
                return !PointCloudLayerUsesFastBasicRenderer(
                           renderState_.pointCloudRendererMode,
                           layer) &&
                       layer.style.gpuBackToFrontSorting &&
                       layer.style.geometryMode ==
                           renderer::pointcloud::PointCloudGeometryMode::
                               ScreenSprites;
            });
    if (hasSortedPointLayers) {
        // The emission attachment still holds subpass 1's weighted values,
        // which the weighted composite already applied. Reset it so the
        // sorted emission composite adds only sorted-layer emission.
        VkClearAttachment emissionClear{};
        emissionClear.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        emissionClear.colorAttachment = 1U;
        emissionClear.clearValue.color = {{0.0F, 0.0F, 0.0F, 0.0F}};
        VkClearRect emissionClearRect{};
        emissionClearRect.rect = scissor;
        emissionClearRect.baseArrayLayer = 0U;
        emissionClearRect.layerCount = 1U;
        vkCmdClearAttachments(
            commandBuffer,
            1U,
            &emissionClear,
            1U,
            &emissionClearRect);
    }
    if (hasSortedPointLayers) {
        for (const auto& layer : renderState_.pointCloudLayers) {
            if (PointCloudLayerUsesFastBasicRenderer(
                    renderState_.pointCloudRendererMode,
                    layer) ||
                !layer.style.gpuBackToFrontSorting ||
                layer.style.geometryMode !=
                    renderer::pointcloud::PointCloudGeometryMode::ScreenSprites) {
                continue;
            }
            std::uint32_t recordedDrawPointCount = 0U;
            const VkPipeline sortedPipeline =
                renderer::pointcloud::PointCloudDepthPrepassTests(layer.style)
                    ? pointSortedAlphaHybridPipeline_
                    : pointSortedAlphaPipeline_;
            if (RecordPointCloudLayerDraw(
                    commandBuffer,
                    layer,
                    false,
                    sortedPipeline,
                    VK_NULL_HANDLE,
                    false,
                    frameIndex,
                    imageIndex,
                    false,
                    &recordedDrawPointCount)) {
                markGpuPhaseActive(kGpuPhaseOpaqueActive);
                if (collectDiagnostics) {
                    ++pointDrawCalls;
                    ++pointUnifiedDrawCalls;
                    pointSubmittedCount += recordedDrawPointCount;
                    pointPassSubmittedCount += recordedDrawPointCount;
                }
            }
        }
    }

    vkCmdNextSubpass(commandBuffer, VK_SUBPASS_CONTENTS_INLINE);
    vkCmdSetViewport(commandBuffer, 0, 1, &viewport);
    vkCmdSetScissor(commandBuffer, 0, 1, &scissor);
    if (hasSortedPointLayers &&
        pointSortedEmissionCompositePipeline_ != VK_NULL_HANDLE &&
        imageIndex < compositeDescriptorSets_.size() &&
        compositeDescriptorSets_[imageIndex] != VK_NULL_HANDLE) {
        VkDescriptorSet descriptorSet = compositeDescriptorSets_[imageIndex];
        vkCmdBindPipeline(
            commandBuffer,
            VK_PIPELINE_BIND_POINT_GRAPHICS,
            pointSortedEmissionCompositePipeline_);
        vkCmdBindDescriptorSets(
            commandBuffer,
            VK_PIPELINE_BIND_POINT_GRAPHICS,
            compositePipelineLayout_,
            0,
            1,
            &descriptorSet,
            0,
            nullptr);
        vkCmdDraw(commandBuffer, 3, 1, 0, 0);
        markGpuPhaseActive(kGpuPhaseOpaqueActive);
    }

    if (collectGpuTimestamps) {
        vkCmdWriteTimestamp(
            commandBuffer,
            VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT,
            timestampQueryPool,
            kTimestampOpaqueEnd);
    }
        vkCmdEndRenderPass(commandBuffer);
        if (imageIndex < sceneImageRevisions_.size()) {
            sceneImageRevisions_[imageIndex] = sceneRevision_;
        }
    }

    RecordTemporalCameraOverlayHistory(
        commandBuffer,
        imageIndex,
        drawLiveScene);

    const bool presentScene =
        liveSceneRenderingEnabled_ &&
        imageIndex < sceneImageRevisions_.size() &&
        sceneImageRevisions_[imageIndex] == sceneRevision_;

    std::array<VkClearValue, 1> presentClearValues{};
    presentClearValues[0].color = {
        {
            renderState_.backgroundColor.r,
            renderState_.backgroundColor.g,
            renderState_.backgroundColor.b,
            renderState_.backgroundColor.a,
        }};

    VkRenderPassBeginInfo presentPassInfo{VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO};
    presentPassInfo.renderPass = presentRenderPass_;
    presentPassInfo.framebuffer = presentFramebuffers_[imageIndex];
    presentPassInfo.renderArea.offset = {0, 0};
    presentPassInfo.renderArea.extent = {swapchainWidth_, swapchainHeight_};
    presentPassInfo.clearValueCount = static_cast<std::uint32_t>(presentClearValues.size());
    presentPassInfo.pClearValues = presentClearValues.data();

    vkCmdBeginRenderPass(commandBuffer, &presentPassInfo, VK_SUBPASS_CONTENTS_INLINE);
    // This timing band covers the always-recorded presentation render pass as
    // well as optional ImGui draws, so it remains valid with an empty UI.
    markGpuPhaseActive(kGpuPhaseImGuiActive);
    vkCmdSetViewport(commandBuffer, 0, 1, &viewport);
    vkCmdSetScissor(commandBuffer, 0, 1, &scissor);

    if (presentScene &&
        imageIndex < postProcessDescriptorSets_.size() &&
        postProcessDescriptorSets_[imageIndex] != VK_NULL_HANDLE) {
        VkDescriptorSet descriptorSet = postProcessDescriptorSets_[imageIndex];
        PostProcessPushConstants pushConstants;
        pushConstants.edl = glm::vec4{
            renderState_.eyeDomeLightingEnabled ? 1.0F : 0.0F,
            24.0F,
            0.35F,
            std::clamp(renderState_.eyeDomeLightingThickness, 1.0F, 24.0F),
        };
        pushConstants.preview = glm::vec4{
            renderState_.proResAlphaPreviewEnabled ? 1.0F : 0.0F,
            0.0F,
            0.0F,
            0.0F,
        };
        const std::uint32_t temporalValidityMask =
            (temporalCameraOverlayHistoryValid_[0U] ? 1U : 0U) |
            (temporalCameraOverlayHistoryValid_[1U] ? 2U : 0U);
        pushConstants.temporalOverlay = glm::vec4{
            temporalCameraOverlayEnabled_
                ? static_cast<float>(static_cast<std::uint32_t>(
                      temporalCameraOverlayCompositionMode_))
                : 0.0F,
            temporalCameraOverlayWeights_[0U],
            temporalCameraOverlayWeights_[1U],
            static_cast<float>(temporalValidityMask),
        };
        if (temporalCameraOverlayEnabled_ &&
            temporalCameraOverlayCompositionMode_ !=
                TemporalCameraCompositionMode::ReprojectedBlend) {
            // Split and alpha-over modes sample both complete cached camera
            // frames in their own image space. Reprojection is unused, so
            // z carries the split location and w identifies source A as the
            // left or top image, depending on the selected composition.
            pushConstants.temporalReprojection = glm::vec4{
                0.0F,
                0.0F,
                temporalCameraOverlaySplitCenterNormalized_,
                temporalCameraOverlayFirstSourcePrimary_ ? 1.0F : 0.0F,
            };
        }
        if (temporalCameraOverlayEnabled_ &&
            temporalCameraOverlayCompositionMode_ ==
                TemporalCameraCompositionMode::ReprojectedBlend &&
            temporalCameraOverlayCurrentViewProjectionsValid_) {
            const std::size_t staleSourceIndex =
                1U - std::min<std::size_t>(
                         temporalCameraOverlayRenderedSourceIndex_,
                         1U);
            const float projectionX =
                temporalCameraOverlayHistoryProjections_
                    [staleSourceIndex][0U][0U];
            const float projectionY =
                temporalCameraOverlayHistoryProjections_
                    [staleSourceIndex][1U][1U];
            if (temporalCameraOverlayHistoryValid_[staleSourceIndex] &&
                std::isfinite(projectionX) &&
                std::isfinite(projectionY) &&
                std::abs(projectionX) > 1.0e-6F &&
                std::abs(projectionY) > 1.0e-6F) {
                pushConstants.temporalReprojection = glm::vec4{
                    static_cast<float>(staleSourceIndex + 1U),
                    projectionX,
                    projectionY,
                    temporalCameraOverlayHistoryReferenceDepths_
                        [staleSourceIndex],
                };
                pushConstants.staleViewToCurrentClip =
                    temporalCameraOverlayCurrentViewProjections_
                        [staleSourceIndex] *
                    glm::inverse(
                        temporalCameraOverlayHistoryViews_
                            [staleSourceIndex]);
            }
        }
        vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, postProcessPipeline_);
        vkCmdBindDescriptorSets(
            commandBuffer,
            VK_PIPELINE_BIND_POINT_GRAPHICS,
            postProcessPipelineLayout_,
            0,
            1,
            &descriptorSet,
            0,
            nullptr);
        vkCmdPushConstants(
            commandBuffer,
            postProcessPipelineLayout_,
            VK_SHADER_STAGE_FRAGMENT_BIT,
            0,
            sizeof(PostProcessPushConstants),
            &pushConstants);
        vkCmdDraw(commandBuffer, 3, 1, 0, 0);
        markGpuPhaseActive(kGpuPhasePostProcessActive);
    }
    if (collectGpuTimestamps) {
        vkCmdWriteTimestamp(
            commandBuffer,
            VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT,
            timestampQueryPool,
            kTimestampPostProcessEnd);
    }

    if (ImGui::GetCurrentContext() != nullptr) {
        ImDrawData* drawData = ImGui::GetDrawData();
        ImGui_ImplVulkan_RenderDrawData(drawData, commandBuffer);
    }
    if (collectGpuTimestamps) {
        vkCmdWriteTimestamp(
            commandBuffer,
            VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT,
            timestampQueryPool,
            kTimestampImGuiEnd);
    }

    vkCmdEndRenderPass(commandBuffer);
    if (collectGpuTimestamps) {
        vkCmdWriteTimestamp(
            commandBuffer,
            VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT,
            timestampQueryPool,
            kTimestampFrameEnd);
    }
    Check(vkEndCommandBuffer(commandBuffer), "vkEndCommandBuffer");

    const auto recordEnd = collectDiagnostics ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point{};
    diagnostics_.pointDrawCalls = pointDrawCalls;
    diagnostics_.pointDepthLayerCount = pointDepthLayerCount;
    diagnostics_.pointAccumulationLayerCount = pointAccumulationLayerCount;
    diagnostics_.pointStyleUploadCount = pointStyleUploadCount;
    diagnostics_.pointSkippedInactiveBindings = pointSkippedInactiveBindings;
    diagnostics_.pointOpaqueHardDiscDrawCalls = pointOpaqueHardDiscDrawCalls;
    diagnostics_.pointConstantSimpleDrawCalls = pointConstantSimpleDrawCalls;
    diagnostics_.pointUnifiedDrawCalls = pointUnifiedDrawCalls;
    diagnostics_.pointFastBasicDrawCalls = pointFastBasicDrawCalls;
    diagnostics_.pointFastBasicDrawnPoints = pointFastBasicDrawnPoints;
    diagnostics_.pointSubmittedCount = pointSubmittedCount;
    diagnostics_.pointPassSubmittedCount = pointPassSubmittedCount;
    diagnostics_.sceneRenderedThisFrame = drawLiveScene;
    diagnostics_.sceneCacheActive = sceneCachingEnabled_;
    diagnostics_.pointCommandRecordMs =
        collectDiagnostics ? std::chrono::duration<double, std::milli>(recordEnd - recordStart).count() : 0.0;
}

void VulkanViewportShell::UpdateUniformBuffer(std::size_t frameIndex) {
    UploadFrameUniforms(frameIndex, swapchainWidth_, swapchainHeight_);
}

void VulkanViewportShell::UploadFrameUniforms(
    std::size_t frameIndex,
    std::uint32_t width,
    std::uint32_t height) {
    if (frameIndex >= kFramesInFlight ||
        frameResources_[frameIndex].uniformBuffer.buffer == VK_NULL_HANDLE) {
        return;
    }

    UploadFrameUniformsToBuffer(frameResources_[frameIndex].uniformBuffer, width, height);
}

void VulkanViewportShell::UploadFrameUniformsToBuffer(
    const BufferAllocation& buffer,
    std::uint32_t width,
    std::uint32_t height) {
    if (buffer.buffer == VK_NULL_HANDLE) {
        return;
    }

    FrameUniforms uniforms;
    uniforms.viewProjection = renderState_.viewProjection;
    uniforms.view = renderState_.view;
    uniforms.projection = renderState_.projection;
    uniforms.cameraPosition = glm::vec4{renderState_.cameraPosition, 0.0F};
    uniforms.depthParameters = glm::vec4{
        std::max(0.0F, renderState_.flowTimeSeconds),
        renderState_.nearPlane,
        renderState_.farPlane,
        std::max(0.001F, renderState_.pointSizeScale),
    };
    const float viewportWidth = std::max(1.0F, static_cast<float>(width));
    const float viewportHeight = std::max(1.0F, static_cast<float>(height));
    uniforms.viewportParameters = glm::vec4{
        viewportWidth,
        viewportHeight,
        2.0F / viewportWidth,
        2.0F / viewportHeight,
    };
    uniforms.depthOfFieldParameters = glm::vec4{
        renderState_.hasDepthOfField ? 1.0F : 0.0F,
        std::max(0.001F, renderState_.focusDistance),
        std::max(0.1F, renderState_.apertureFStops),
        std::max(0.0F, renderState_.depthOfFieldMaxBlurPixels),
    };
    uniforms.inverseViewProjection = glm::inverse(renderState_.viewProjection);
    const auto& adaptive = renderState_.adaptiveDensityTransition;
    // Shaders read x (start depth), z (end depth), and w (the publish
    // crossfade's coarse-engage factor). preparedFineDepthMeters is a
    // CPU-side preparation bound and is deliberately not uploaded.
    uniforms.adaptiveDensityParameters = adaptive.Valid()
        ? glm::vec4{
              adaptive.startDepthMeters,
              adaptive.switchDepthMeters,
              adaptive.endDepthMeters,
              std::clamp(adaptive.coarseEngage, 0.0F, 1.0F),
          }
        : glm::vec4{0.0F};

    UploadBufferData(buffer, &uniforms, sizeof(uniforms));
}

void VulkanViewportShell::UploadRainUniforms(
    std::size_t frameIndex,
    std::uint32_t width,
    std::uint32_t height) {
    auto& resources = rainResources_;
    if (frameIndex >= resources.uniformBuffers.size() ||
        resources.uniformBuffers[frameIndex].buffer == VK_NULL_HANDLE) {
        return;
    }

    const RainCountersGpu* completedCounters = nullptr;
    if (resources.counterReadbackBuffers[frameIndex].mapped != nullptr) {
        // DrawFrame has already waited this frame slot's fence. Read the
        // slot-owned transfer snapshot rather than racing the shared compute
        // counter buffer used by another live or EXR submission.
        completedCounters = static_cast<const RainCountersGpu*>(
            resources.counterReadbackBuffers[frameIndex].mapped);
        resources.lastInFlightParticleCount =
            renderState_.rainSettings.enabled
                ? completedCounters->roleValues.w
                : 0U;
        if (renderState_.rainSettings.enabled &&
            completedCounters->values.z > 0U &&
            std::isfinite(renderState_.flowTimeSeconds)) {
            // A draining airborne drop can land after the keyed Rain amount
            // reaches zero. Anchor the wetness/ring tail to that real impact.
            resources.lastPotentialImpactTimeSeconds =
                renderState_.flowTimeSeconds;
        }
    }
    if (!renderState_.rainSettings.impactEffectsEnabled) {
        diagnostics_.rainImpactOverflowCount = 0U;
        diagnostics_.rainEventsEmittedThisFrame = 0U;
        diagnostics_.rainSandImpactsThisFrame = 0U;
        diagnostics_.rainRockImpactsThisFrame = 0U;
        diagnostics_.rainVegetationImpactsThisFrame = 0U;
    } else if (completedCounters != nullptr) {
        diagnostics_.rainImpactOverflowCount =
            completedCounters->values.y;
        diagnostics_.rainEventsEmittedThisFrame =
            completedCounters->values.z;
        diagnostics_.rainSandImpactsThisFrame =
            completedCounters->roleValues.x;
        diagnostics_.rainRockImpactsThisFrame =
            completedCounters->roleValues.y;
        diagnostics_.rainVegetationImpactsThisFrame =
            completedCounters->roleValues.z;
    }
    UploadRainUniformsToBuffer(resources.uniformBuffers[frameIndex], width, height);
}

void VulkanViewportShell::UploadRainUniformsToBuffer(
    const BufferAllocation& target,
    std::uint32_t width,
    std::uint32_t height) {
    if (target.buffer == VK_NULL_HANDLE) {
        return;
    }
    auto& resources = rainResources_;

    const auto& settings = renderState_.rainSettings;
    const auto& visual = renderState_.rainVisual;
    const auto intensity = invisible_places::water::RainIntensityValues(settings.intensityPreset);
    const auto activeParticleCount = ActiveRainParticleCount(renderState_);
    const float worldSpan = invisible_places::water::RainImpactGridWorldSpan(settings);

    RainUniformsGpu uniforms;
    uniforms.viewProjection = renderState_.viewProjection;
    uniforms.view = renderState_.view;
    uniforms.projection = renderState_.projection;
    uniforms.cameraTime = glm::vec4{renderState_.cameraPosition, std::max(0.0F, renderState_.flowTimeSeconds)};
    const float collisionTop = resources.collisionBounds.valid
                                   ? resources.collisionBounds.maximum.z
                                   : renderState_.rainSpawnCentre.z;
    uniforms.spawnCentreRadius = glm::vec4{
        renderState_.rainSpawnCentre.x,
        renderState_.rainSpawnCentre.y,
        collisionTop + settings.spawnHeightMeters,
        std::max(0.1F, settings.spawnRadiusMeters),
    };
    if (resources.collisionBounds.valid) {
        uniforms.cacheBoundsMinResolution = glm::vec4{
            resources.collisionBounds.minimum.x,
            resources.collisionBounds.minimum.y,
            resources.collisionBounds.minimum.z,
            std::max(
                1.0e-6F,
                resources.surfaceResolutionMeters),
        };
        uniforms.cacheBoundsMaxDeathDistance = glm::vec4{
            resources.collisionBounds.maximum.x,
            resources.collisionBounds.maximum.y,
            resources.collisionBounds.maximum.z,
            std::max(1.0F, settings.cameraDeathDistanceMeters),
        };
    }
    uniforms.weather0 = glm::vec4{
        settings.windDirectionX,
        settings.windDirectionY,
        std::max(0.0F, settings.windSpeedMetersPerSecond) * intensity.windResponse,
        std::max(0.0F, settings.turbulence) * intensity.windResponse,
    };
    uniforms.weather1 = glm::vec4{
        std::max(0.0F, settings.gustStrength),
        std::max(0.2F, settings.gustScaleMeters),
        std::max(0.0F, settings.gustSpeedMetersPerSecond),
        std::clamp(settings.weatherFrontStrength, 0.0F, 1.0F),
    };
    uniforms.weather2 = glm::vec4{
        std::max(0.2F, settings.weatherFrontScaleMeters),
        std::max(0.0F, settings.weatherFrontSpeedMetersPerSecond),
        resources.frameDeltaSeconds,
        std::max(0.1F, settings.fallSpeedMetersPerSecond) * intensity.speed,
    };
    uniforms.visual0 = glm::vec4{
        std::max(0.0001F, visual.widthMeters) * intensity.width,
        std::max(0.001F, visual.streakLengthMeters) * intensity.length,
        std::clamp(visual.softness, 0.0F, 1.0F),
        std::clamp(visual.opacity * settings.opacityScale * intensity.opacity, 0.0F, 1.0F),
    };
    uniforms.visual1 = glm::vec4{
        visual.colour[0],
        visual.colour[1],
        visual.colour[2],
        std::max(0.0F, visual.emission * settings.emissionScale * intensity.emission),
    };
    uniforms.visual2 = glm::vec4{
        std::max(0.0F, visual.minimumScreenPixels),
        std::max(visual.minimumScreenPixels, visual.maximumScreenPixels),
        std::max(0.05F, settings.dropletSizeScale),
        intensity.effectEnergy,
    };
    uniforms.nearSurface = glm::vec4{
        std::clamp(settings.nearSurface.approachDistanceMeters, 0.0F, 1.0F),
        std::clamp(settings.nearSurface.minimumSpeedFactor, 0.05F, 1.0F),
        std::clamp(settings.nearSurface.squish, 0.0F, 1.0F),
        std::clamp(settings.nearSurface.normalAlignment, 0.0F, 1.0F),
    };
    uniforms.simulation0 = glm::uvec4{
        activeParticleCount,
        invisible_places::water::kRainParticleCapacity,
        invisible_places::water::kRainImpactEventCapacity,
        settings.seed,
    };
    uniforms.simulation1 = glm::uvec4{
        settings.enabled && resources.collisionReady ? 1U : 0U,
        settings.impactEffectsEnabled ? 1U : 0U,
        resources.resetEpoch,
        static_cast<std::uint32_t>(settings.intensityPreset),
    };
    uniforms.collision0 = glm::uvec4{
        resources.surfaceMask,
        resources.vegetationMask,
        resources.maximumProbeCount,
        invisible_places::water::kRainImpactGridDimension,
    };
    uniforms.impactGrid = glm::vec4{
        renderState_.cameraPosition.x - worldSpan * 0.5F,
        renderState_.cameraPosition.y - worldSpan * 0.5F,
        worldSpan / static_cast<float>(invisible_places::water::kRainImpactGridDimension),
        worldSpan,
    };
    uniforms.effectToggles = glm::uvec4{
        settings.sandEffectsEnabled ? 1U : 0U,
        settings.rockEffectsEnabled ? 1U : 0U,
        settings.vegetationEffectsEnabled ? 1U : 0U,
        0U,
    };
    uniforms.effectScales = glm::vec4{
        // Response is consumed by the point shaders, not baked into newly
        // emitted events. Keep the retired lanes neutral for ABI stability.
        1.0F,
        1.0F,
        1.0F,
        // Wetness downhill stretch: the GPU simulator boosts steep rock
        // impact radii by steepness x min(stretch, 1) at spawn.
        std::clamp(settings.rockImpact.downhillStretch, 0.0F, 2.0F),
    };
    uniforms.viewport = glm::vec4{
        std::max(1U, width),
        std::max(1U, height),
        1.0F / static_cast<float>(std::max(1U, width)),
        1.0F / static_cast<float>(std::max(1U, height)),
    };
    UploadBufferData(target, &uniforms, sizeof(uniforms));
}

void VulkanViewportShell::RecordRainCompute(VkCommandBuffer commandBuffer, std::size_t frameIndex) {
    if (frameIndex >= rainResources_.descriptorSets.size()) {
        return;
    }
    RecordRainComputeWithDescriptor(
        commandBuffer,
        rainResources_.descriptorSets[frameIndex],
        &rainResources_.counterReadbackBuffers[frameIndex]);
}

void VulkanViewportShell::RecordRainComputeWithDescriptor(
    VkCommandBuffer commandBuffer,
    VkDescriptorSet descriptorSet,
    const BufferAllocation* counterReadback) {
    if (!renderState_.rainSettings.enabled || !rainResources_.collisionReady ||
        rainComputePipeline_ == VK_NULL_HANDLE || descriptorSet == VK_NULL_HANDLE) {
        return;
    }
    vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, rainComputePipeline_);
    vkCmdBindDescriptorSets(
        commandBuffer,
        VK_PIPELINE_BIND_POINT_COMPUTE,
        rainPipelineLayout_,
        0U,
        1U,
        &descriptorSet,
        0U,
        nullptr);

    // Rain simulation and impact grids intentionally persist across frames.
    // A later submission may otherwise begin its compute writes while the
    // previous frame is still reading the same buffers for ROCK/SAND/VEG
    // vertices or fragments. Keep the dependency buffer-local so unrelated
    // point-cloud and cache resources can continue independently.
    const std::array<const BufferAllocation*, 6> sharedRainBuffers = {
        &rainResources_.particleBuffer,
        &rainResources_.eventBuffer,
        &rainResources_.counterBuffer,
        &rainResources_.impactCountBuffer,
        &rainResources_.impactReferenceBuffer,
        &rainResources_.dynamicMeshFlowRainSeedBuffer,
    };
    std::array<VkBufferMemoryBarrier, 6> priorFrameBarriers{};
    for (std::size_t index = 0; index < sharedRainBuffers.size(); ++index) {
        auto& bufferBarrier = priorFrameBarriers[index];
        bufferBarrier.sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER;
        bufferBarrier.srcAccessMask =
            VK_ACCESS_SHADER_READ_BIT |
            VK_ACCESS_SHADER_WRITE_BIT |
            VK_ACCESS_TRANSFER_READ_BIT;
        bufferBarrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT;
        bufferBarrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        bufferBarrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        bufferBarrier.buffer = sharedRainBuffers[index]->buffer;
        bufferBarrier.offset = 0U;
        bufferBarrier.size = sharedRainBuffers[index]->size;
    }
    vkCmdPipelineBarrier(
        commandBuffer,
        VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT |
            VK_PIPELINE_STAGE_VERTEX_SHADER_BIT |
            VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT |
            VK_PIPELINE_STAGE_TRANSFER_BIT,
        VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
        0U,
        0U,
        nullptr,
        static_cast<std::uint32_t>(priorFrameBarriers.size()),
        priorFrameBarriers.data(),
        0U,
        nullptr);

    const auto barrier = [&](VkPipelineStageFlags destinationStage, VkAccessFlags destinationAccess) {
        VkMemoryBarrier memoryBarrier{VK_STRUCTURE_TYPE_MEMORY_BARRIER};
        memoryBarrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
        memoryBarrier.dstAccessMask = destinationAccess;
        vkCmdPipelineBarrier(
            commandBuffer,
            VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
            destinationStage,
            0U,
            1U,
            &memoryBarrier,
            0U,
            nullptr,
            0U,
            nullptr);
    };

    // Always reset the compact per-frame counters, including the number of
    // airborne particles still draining after emission stops. The shader
    // skips the larger impact-grid clear when impact effects are disabled.
    constexpr std::uint32_t clearPhase = 0U;
    vkCmdPushConstants(
        commandBuffer,
        rainPipelineLayout_,
        VK_SHADER_STAGE_COMPUTE_BIT,
        0U,
        sizeof(clearPhase),
        &clearPhase);
    vkCmdDispatch(
        commandBuffer,
        (invisible_places::water::kRainImpactGridDimension *
             invisible_places::water::kRainImpactGridDimension +
         63U) /
            64U,
        1U,
        1U);
    barrier(
        VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
        VK_ACCESS_SHADER_READ_BIT |
            VK_ACCESS_SHADER_WRITE_BIT);

    constexpr std::uint32_t simulationPhase = 1U;
    vkCmdPushConstants(
        commandBuffer,
        rainPipelineLayout_,
        VK_SHADER_STAGE_COMPUTE_BIT,
        0U,
        sizeof(simulationPhase),
        &simulationPhase);
    vkCmdDispatch(
        commandBuffer,
        (invisible_places::water::kRainParticleCapacity + 63U) / 64U,
        1U,
        1U);

    if (renderState_.rainSettings.impactEffectsEnabled) {
        barrier(VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT);
        constexpr std::uint32_t binPhase = 2U;
        vkCmdPushConstants(
            commandBuffer,
            rainPipelineLayout_,
            VK_SHADER_STAGE_COMPUTE_BIT,
            0U,
            sizeof(binPhase),
            &binPhase);
        vkCmdDispatch(
            commandBuffer,
            (invisible_places::water::kRainImpactEventCapacity + 63U) / 64U,
            1U,
            1U);
    }
    if (counterReadback != nullptr &&
        counterReadback->buffer != VK_NULL_HANDLE &&
        counterReadback->size >= sizeof(RainCountersGpu)) {
        VkBufferMemoryBarrier counterToTransfer{VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER};
        counterToTransfer.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
        counterToTransfer.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
        counterToTransfer.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        counterToTransfer.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        counterToTransfer.buffer = rainResources_.counterBuffer.buffer;
        counterToTransfer.offset = 0U;
        counterToTransfer.size = sizeof(RainCountersGpu);
        vkCmdPipelineBarrier(
            commandBuffer,
            VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
            VK_PIPELINE_STAGE_TRANSFER_BIT,
            0U,
            0U,
            nullptr,
            1U,
            &counterToTransfer,
            0U,
            nullptr);

        VkBufferCopy counterCopy{};
        counterCopy.size = sizeof(RainCountersGpu);
        vkCmdCopyBuffer(
            commandBuffer,
            rainResources_.counterBuffer.buffer,
            counterReadback->buffer,
            1U,
            &counterCopy);

        VkBufferMemoryBarrier readbackToHost{VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER};
        readbackToHost.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
        readbackToHost.dstAccessMask = VK_ACCESS_HOST_READ_BIT;
        readbackToHost.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        readbackToHost.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        readbackToHost.buffer = counterReadback->buffer;
        readbackToHost.offset = 0U;
        readbackToHost.size = sizeof(RainCountersGpu);
        vkCmdPipelineBarrier(
            commandBuffer,
            VK_PIPELINE_STAGE_TRANSFER_BIT,
            VK_PIPELINE_STAGE_HOST_BIT,
            0U,
            0U,
            nullptr,
            1U,
            &readbackToHost,
            0U,
            nullptr);
    }
    barrier(
        VK_PIPELINE_STAGE_VERTEX_SHADER_BIT | VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
        VK_ACCESS_SHADER_READ_BIT);
}

void VulkanViewportShell::RecordRainDraw(
    VkCommandBuffer commandBuffer,
    std::size_t frameIndex,
    VkPipeline pipeline) {
    if (frameIndex >= rainResources_.descriptorSets.size()) {
        return;
    }
    RecordRainDrawWithDescriptor(
        commandBuffer,
        rainResources_.descriptorSets[frameIndex],
        pipeline);
}

void VulkanViewportShell::RecordRainDrawWithDescriptor(
    VkCommandBuffer commandBuffer,
    VkDescriptorSet descriptorSet,
    VkPipeline pipeline) {
    if (!renderState_.rainSettings.enabled || !rainResources_.collisionReady ||
        pipeline == VK_NULL_HANDLE || descriptorSet == VK_NULL_HANDLE) {
        return;
    }
    vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);
    vkCmdBindDescriptorSets(
        commandBuffer,
        VK_PIPELINE_BIND_POINT_GRAPHICS,
        rainPipelineLayout_,
        0U,
        1U,
        &descriptorSet,
        0U,
        nullptr);
    // The spawn target can drop immediately, but already-active slots above
    // it remain visible until they land or leave the collision volume.
    vkCmdDraw(
        commandBuffer,
        6U,
        invisible_places::water::kRainParticleCapacity,
        0U,
        0U);
}

VulkanViewportShell::BufferAllocation VulkanViewportShell::CreateHostVisibleBuffer(
    VkDeviceSize size,
    VkBufferUsageFlags usage) const {
    if (size == 0) {
        size = 4;
    }

    BufferAllocation allocation;
    allocation.size = size;

    VkBufferCreateInfo bufferInfo{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
    bufferInfo.size = size;
    bufferInfo.usage = usage;
    bufferInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

    try {
        Check(vkCreateBuffer(device_, &bufferInfo, nullptr, &allocation.buffer), "vkCreateBuffer");

        VkMemoryRequirements memoryRequirements{};
        vkGetBufferMemoryRequirements(device_, allocation.buffer, &memoryRequirements);

        VkMemoryAllocateInfo allocInfo{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
        allocInfo.allocationSize = memoryRequirements.size;
        allocInfo.memoryTypeIndex = FindMemoryType(
            memoryRequirements.memoryTypeBits,
            VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
            VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT | VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);

        Check(vkAllocateMemory(device_, &allocInfo, nullptr, &allocation.memory), "vkAllocateMemory");
        Check(vkBindBufferMemory(device_, allocation.buffer, allocation.memory, 0), "vkBindBufferMemory");
        Check(
            vkMapMemory(device_, allocation.memory, 0, allocation.size, 0, &allocation.mapped),
            "vkMapMemory(persistent buffer)");
        return allocation;
    } catch (...) {
        if (allocation.memory != VK_NULL_HANDLE && allocation.mapped != nullptr) {
            vkUnmapMemory(device_, allocation.memory);
        }
        if (allocation.buffer != VK_NULL_HANDLE) {
            vkDestroyBuffer(device_, allocation.buffer, nullptr);
        }
        if (allocation.memory != VK_NULL_HANDLE) {
            vkFreeMemory(device_, allocation.memory, nullptr);
        }
        throw;
    }
}

VulkanViewportShell::BufferAllocation VulkanViewportShell::CreateDeviceLocalBuffer(
    VkDeviceSize size,
    VkBufferUsageFlags usage) const {
    if (size == 0) {
        size = 4;
    }

    BufferAllocation allocation;
    allocation.size = size;

    VkBufferCreateInfo bufferInfo{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
    bufferInfo.size = size;
    bufferInfo.usage = usage;
    bufferInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    try {
        Check(
            vkCreateBuffer(device_, &bufferInfo, nullptr, &allocation.buffer),
            "vkCreateBuffer(device local)");

        VkMemoryRequirements memoryRequirements{};
        vkGetBufferMemoryRequirements(device_, allocation.buffer, &memoryRequirements);

        VkMemoryAllocateInfo allocInfo{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
        allocInfo.allocationSize = memoryRequirements.size;
        allocInfo.memoryTypeIndex = FindMemoryType(
            memoryRequirements.memoryTypeBits,
            VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
            VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
        Check(
            vkAllocateMemory(device_, &allocInfo, nullptr, &allocation.memory),
            "vkAllocateMemory(device local)");
        Check(
            vkBindBufferMemory(device_, allocation.buffer, allocation.memory, 0),
            "vkBindBufferMemory(device local)");
        return allocation;
    } catch (...) {
        if (allocation.buffer != VK_NULL_HANDLE) {
            vkDestroyBuffer(device_, allocation.buffer, nullptr);
        }
        if (allocation.memory != VK_NULL_HANDLE) {
            vkFreeMemory(device_, allocation.memory, nullptr);
        }
        throw;
    }
}

void VulkanViewportShell::UploadBufferData(
    const BufferAllocation& buffer,
    const void* data,
    VkDeviceSize size) const {
    if (buffer.memory == VK_NULL_HANDLE || data == nullptr || size == 0) {
        return;
    }

    if (buffer.mapped != nullptr) {
        std::memcpy(buffer.mapped, data, static_cast<std::size_t>(size));
        return;
    }

    void* mapped = nullptr;
    Check(vkMapMemory(device_, buffer.memory, 0, size, 0, &mapped), "vkMapMemory");
    std::memcpy(mapped, data, static_cast<std::size_t>(size));
    vkUnmapMemory(device_, buffer.memory);
}

void VulkanViewportShell::DestroyBuffer(BufferAllocation* buffer) {
    if (buffer == nullptr) {
        return;
    }

    if (buffer->memory != VK_NULL_HANDLE) {
        if (buffer->mapped != nullptr) {
            vkUnmapMemory(device_, buffer->memory);
            buffer->mapped = nullptr;
        }
    }
    if (buffer->buffer != VK_NULL_HANDLE) {
        vkDestroyBuffer(device_, buffer->buffer, nullptr);
    }
    if (buffer->memory != VK_NULL_HANDLE) {
        vkFreeMemory(device_, buffer->memory, nullptr);
    }

    *buffer = BufferAllocation{};
}

void VulkanViewportShell::DestroyImage(ImageAllocation* image) {
    if (image == nullptr) {
        return;
    }

    if (image->view != VK_NULL_HANDLE) {
        vkDestroyImageView(device_, image->view, nullptr);
    }
    if (image->image != VK_NULL_HANDLE) {
        vkDestroyImage(device_, image->image, nullptr);
    }
    if (image->memory != VK_NULL_HANDLE) {
        vkFreeMemory(device_, image->memory, nullptr);
    }

    *image = ImageAllocation{};
}

std::uint32_t VulkanViewportShell::FindMemoryType(
    std::uint32_t typeFilter,
    VkMemoryPropertyFlags requiredFlags,
    VkMemoryPropertyFlags preferredFlags) const {
    VkPhysicalDeviceMemoryProperties memoryProperties{};
    vkGetPhysicalDeviceMemoryProperties(physicalDevice_, &memoryProperties);

    std::optional<std::uint32_t> fallback;
    for (std::uint32_t index = 0; index < memoryProperties.memoryTypeCount; ++index) {
        const bool supported = (typeFilter & (1U << index)) != 0;
        const auto propertyFlags = memoryProperties.memoryTypes[index].propertyFlags;
        if (!supported || (propertyFlags & requiredFlags) != requiredFlags) {
            continue;
        }

        if ((propertyFlags & preferredFlags) == preferredFlags) {
            return index;
        }

        if (!fallback.has_value()) {
            fallback = index;
        }
    }

    if (!fallback.has_value()) {
        throw std::runtime_error{"Failed to find a compatible Vulkan memory type."};
    }

    return fallback.value();
}

VkFormat VulkanViewportShell::SelectDepthFormat() const {
    constexpr std::array<VkFormat, 3> candidates = {
        VK_FORMAT_D32_SFLOAT,
        VK_FORMAT_D32_SFLOAT_S8_UINT,
        VK_FORMAT_D24_UNORM_S8_UINT,
    };

    for (const auto format : candidates) {
        VkFormatProperties properties{};
        vkGetPhysicalDeviceFormatProperties(physicalDevice_, format, &properties);
        if ((properties.optimalTilingFeatures & VK_FORMAT_FEATURE_DEPTH_STENCIL_ATTACHMENT_BIT) != 0) {
            return format;
        }
    }

    throw std::runtime_error{"No compatible Vulkan depth format is available."};
}

VkFormat VulkanViewportShell::SelectAccumulationFormat() const {
    constexpr std::array<VkFormat, 2> candidates = {
        VK_FORMAT_R16G16B16A16_SFLOAT,
        VK_FORMAT_R32G32B32A32_SFLOAT,
    };

    for (const auto format : candidates) {
        VkFormatProperties properties{};
        vkGetPhysicalDeviceFormatProperties(physicalDevice_, format, &properties);
        if ((properties.optimalTilingFeatures & VK_FORMAT_FEATURE_COLOR_ATTACHMENT_BIT) != 0) {
            return format;
        }
    }

    throw std::runtime_error{"No compatible accumulation color format is available."};
}

VkFormat VulkanViewportShell::SelectRevealageFormat() const {
    constexpr std::array<VkFormat, 2> candidates = {
        VK_FORMAT_R16_SFLOAT,
        VK_FORMAT_R32_SFLOAT,
    };

    for (const auto format : candidates) {
        VkFormatProperties properties{};
        vkGetPhysicalDeviceFormatProperties(physicalDevice_, format, &properties);
        if ((properties.optimalTilingFeatures & VK_FORMAT_FEATURE_COLOR_ATTACHMENT_BIT) != 0) {
            return format;
        }
    }

    throw std::runtime_error{"No compatible revealage color format is available."};
}

std::vector<char> VulkanViewportShell::ReadBinaryFile(const std::string& filePath) const {
    std::ifstream input{filePath, std::ios::ate | std::ios::binary};
    if (!input.is_open()) {
        throw std::runtime_error{"Failed to open shader file: " + filePath};
    }

    const auto fileSize = static_cast<std::size_t>(input.tellg());
    std::vector<char> bytes(fileSize);
    input.seekg(0, std::ios::beg);
    input.read(bytes.data(), static_cast<std::streamsize>(fileSize));
    if (!input.good() && !input.eof()) {
        throw std::runtime_error{"Failed to read shader file: " + filePath};
    }

    return bytes;
}

VulkanViewportShell::ImageAllocation VulkanViewportShell::CreateAttachmentImage(
    VkFormat format,
    VkImageUsageFlags usage,
    VkImageAspectFlags aspectFlags) const {
    return CreateAttachmentImage(swapchainWidth_, swapchainHeight_, format, usage, aspectFlags);
}

VulkanViewportShell::ImageAllocation VulkanViewportShell::CreateAttachmentImage(
    std::uint32_t width,
    std::uint32_t height,
    VkFormat format,
    VkImageUsageFlags usage,
    VkImageAspectFlags aspectFlags) const {
    ImageAllocation image;
    image.format = format;

    VkImageCreateInfo imageInfo{VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO};
    imageInfo.imageType = VK_IMAGE_TYPE_2D;
    imageInfo.extent.width = width;
    imageInfo.extent.height = height;
    imageInfo.extent.depth = 1;
    imageInfo.mipLevels = 1;
    imageInfo.arrayLayers = 1;
    imageInfo.format = format;
    imageInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
    imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    imageInfo.usage = usage;
    imageInfo.samples = VK_SAMPLE_COUNT_1_BIT;
    imageInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

    Check(vkCreateImage(device_, &imageInfo, nullptr, &image.image), "vkCreateImage(attachment)");

    VkMemoryRequirements memoryRequirements{};
    vkGetImageMemoryRequirements(device_, image.image, &memoryRequirements);

    VkMemoryAllocateInfo allocInfo{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
    allocInfo.allocationSize = memoryRequirements.size;
    allocInfo.memoryTypeIndex = FindMemoryType(memoryRequirements.memoryTypeBits, 0, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

    Check(vkAllocateMemory(device_, &allocInfo, nullptr, &image.memory), "vkAllocateMemory(attachment)");
    Check(vkBindImageMemory(device_, image.image, image.memory, 0), "vkBindImageMemory(attachment)");

    VkImageViewCreateInfo viewInfo{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
    viewInfo.image = image.image;
    viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
    viewInfo.format = format;
    viewInfo.subresourceRange.aspectMask = aspectFlags;
    viewInfo.subresourceRange.baseMipLevel = 0;
    viewInfo.subresourceRange.levelCount = 1;
    viewInfo.subresourceRange.baseArrayLayer = 0;
    viewInfo.subresourceRange.layerCount = 1;

    Check(vkCreateImageView(device_, &viewInfo, nullptr, &image.view), "vkCreateImageView(attachment)");
    return image;
}

VulkanViewportShell::ActivePointCloudResources* VulkanViewportShell::FindPointCloudResources(std::size_t layerId) {
    auto resourcesIt = std::find_if(
        pointCloudResources_.begin(),
        pointCloudResources_.end(),
        [layerId](const ActivePointCloudResources& resources) { return resources.layerId == layerId; });
    return resourcesIt != pointCloudResources_.end() ? &(*resourcesIt) : nullptr;
}

const VulkanViewportShell::ActivePointCloudResources* VulkanViewportShell::FindPointCloudResources(
    std::size_t layerId) const {
    auto resourcesIt = std::find_if(
        pointCloudResources_.begin(),
        pointCloudResources_.end(),
        [layerId](const ActivePointCloudResources& resources) { return resources.layerId == layerId; });
    return resourcesIt != pointCloudResources_.end() ? &(*resourcesIt) : nullptr;
}

VulkanViewportShell::ActiveGaussianSplatResources* VulkanViewportShell::FindGaussianSplatResources(
    std::size_t layerId) {
    auto resourcesIt = std::find_if(
        gaussianSplatResources_.begin(),
        gaussianSplatResources_.end(),
        [layerId](const ActiveGaussianSplatResources& resources) { return resources.layerId == layerId; });
    return resourcesIt != gaussianSplatResources_.end() ? &(*resourcesIt) : nullptr;
}

const VulkanViewportShell::ActiveGaussianSplatResources* VulkanViewportShell::FindGaussianSplatResources(
    std::size_t layerId) const {
    auto resourcesIt = std::find_if(
        gaussianSplatResources_.begin(),
        gaussianSplatResources_.end(),
        [layerId](const ActiveGaussianSplatResources& resources) { return resources.layerId == layerId; });
    return resourcesIt != gaussianSplatResources_.end() ? &(*resourcesIt) : nullptr;
}

void VulkanViewportShell::FramebufferResizeCallback(GLFWwindow* window, int /*width*/, int /*height*/) {
    auto* shell = static_cast<VulkanViewportShell*>(glfwGetWindowUserPointer(window));
    if (shell != nullptr) {
        shell->framebufferResized_ = true;
    }
}

}  // namespace invisible_places::renderer::core
