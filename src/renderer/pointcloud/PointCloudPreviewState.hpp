#pragma once

#include "io/PointCloudData.hpp"
#include "style/RenderParameterBinding.hpp"

#include <glm/mat4x4.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace invisible_places::renderer::pointcloud {

inline constexpr float kInactivePointSizeDefault = 1.0F;
inline constexpr float kInactiveSurfelDiameterDefault = 0.005F;
inline constexpr float kInactiveOpacityDefault = 1.0F;
inline constexpr float kInactiveEmissionDefault = 0.0F;
inline constexpr float kInactiveDepthFadeDefault = 0.0F;
inline constexpr float kInactiveColormapPositionDefault = 0.5F;
inline constexpr std::size_t kTimingColouriseMaxEffects = 5U;
inline constexpr std::size_t kTimingColouriseLutSamples = 64U;

// Renderer-facing timing colourise data is deliberately independent from the
// authored timing model. Callers resolve field names to each layer's local
// scalar slot before populating this fixed-capacity payload.
enum class TimingColouriseSource : std::uint32_t {
    ScalarField = 0U,
    NormalX = 1U,
    NormalY = 2U,
    NormalZ = 3U
};

struct ResolvedTimingColouriseEffect {
    bool enabled = false;
    TimingColouriseSource source = TimingColouriseSource::ScalarField;
    std::int32_t scalarFieldSlot = -1;
    float lowerBound = 0.0F;
    float upperBound = 1.0F;
    // Fraction of the selected [lower, upper] span used by each inward edge
    // fade. Values are sanitized to [0, 0.5] by the render backends.
    float edgeFadeFraction = 0.0F;
    // RGB is the tint and A is colourise amount. Alpha never changes point
    // opacity.
    std::array<std::array<float, 4>, kTimingColouriseLutSamples> rgbaLut{};
};

struct ResolvedTimingColouriseStack {
    std::uint32_t effectCount = 0U;
    // Effects are applied in array order; later entries are visually topmost.
    std::array<ResolvedTimingColouriseEffect, kTimingColouriseMaxEffects> effects{};
};

enum class PointCloudColorMode {
    SourceRgb,
    SolidColor,
    ScalarColormap
};

enum class PointCloudColormapId {
    Viridis,
    Plasma,
    Inferno,
    Magma,
    Cividis,
    Turbo,
    Topographic,
    LandSurface,
    ExponentialFire,
    ExponentialIce,
    HighContrast,
    CustomGradient
};

enum class PointCloudGeometryMode {
    ScreenSprites,
    WorldSurfels,
    CameraFacingWorldSprites
};

enum class PointCloudScreenSpriteSizeMode {
    Pixels,
    WorldMillimeters
};

enum class PointCloudFalloffProfile {
    HardDisc,
    SoftDisc,
    Gaussian,
    Rim
};

enum class PointCloudStylisationMode {
    Off,
    NprStylisation,
    BrushParticles
};

enum class PointCloudNprPreset {
    Watercolor,
    Cartoon
};

enum class PointCloudPreviewLodMode {
    FullResolution,
    AutoCameraLod,
    ForceLod
};

enum class PointCloudRendererMode {
    Beauty,
    FastBasic
};

enum class PointCloudMaterialVariant {
    OpaqueHardDisc,
    ConstantSimple,
    Unified
};

enum class PointCloudShorelineWaveAlgorithm {
    FoamFronts,
    HeightFoam
};

struct PointCloudHeightFoamShorelineSettings {
    float runupZ = 1.55F;
    float breakZ = 1.30F;
    float offshoreReachMeters = 0.55F;
    float edgeFadeMeters = 0.05F;
    float directionX = 1.0F;
    float directionY = 0.0F;
    float patternScale = 1.0F;
    float wavelengthMeters = 0.25F;
    float speed = 0.55F;
    float warp = 0.35F;
    float turbulence = 0.06F;
    float density = 0.55F;
    float phase = 0.0F;
    float intensity = 1.15F;
    float offshoreFoamStrength = 0.30F;
    float incomingStrength = 1.0F;
    float returnStrength = 0.30F;
    float emissionAdd = 0.65F;
    float opacityAdd = 0.08F;
    float opacityMultiply = 1.25F;
    float pointSizeAdd = 0.0F;
    float pointSizeMultiply = 1.35F;
    float colourMix = 0.75F;
    std::array<float, 3> colour{0.62F, 0.88F, 1.0F};
    std::uint32_t seed = 1U;
};

// A profile-sized copy of the legacy Foam Fronts fields that remain flattened
// on PointCloudStyleState for shader/persistence compatibility. Keeping this
// bank separate lets named Shoreline profiles preserve both algorithms while
// changing only the water settings on an existing point visual.
struct PointCloudFoamFrontsShorelineSettings {
    float boundaryZ = 1.55F;
    float heightReachMeters = 0.45F;
    float edgeFadeMeters = 0.05F;
    float directionX = 1.0F;
    float directionY = 0.0F;
    float patternScale = 1.0F;
    float wavelengthMeters = 0.25F;
    float speed = 0.55F;
    float warp = 0.35F;
    float turbulence = 0.06F;
    float density = 0.55F;
    float phase = 0.0F;
    float intensity = 1.15F;
    float emissionAdd = 0.65F;
    float opacityAdd = 0.08F;
    float opacityMultiply = 1.25F;
    float pointSizeAdd = 0.0F;
    float pointSizeMultiply = 1.35F;
    float colourMix = 0.75F;
    std::array<float, 3> colour{0.62F, 0.88F, 1.0F};
    std::uint32_t seed = 1U;
};

struct PointCloudShorelineWaveSettings {
    bool enabled = false;
    PointCloudShorelineWaveAlgorithm algorithm =
        PointCloudShorelineWaveAlgorithm::FoamFronts;
    PointCloudFoamFrontsShorelineSettings foamFronts{};
    PointCloudHeightFoamShorelineSettings heightFoam{};
};

struct PointCloudShorelineWaveProfile {
    std::string name = "Default";
    PointCloudShorelineWaveSettings settings{};
};

struct PointCloudDensityCompensation {
    float footprintScale = 1.0F;
    float coverageCorrection = 1.0F;
};

struct WaterFlowActivityScales {
    float activity = 1.0F;
    // A continuous whole-population fade; trail seeds never threshold it.
    float trailVisibility = 1.0F;
    float appearance = 1.0F;
    float width = 1.0F;
    // Motion stays independent of strength because phase uses absolute time.
    float speed = 1.0F;
    float visibleLength = 1.0F;
    float lateralMotion = 0.15F;
};

struct PointCloudStyleState {
    PointCloudStyleState();

    PointCloudGeometryMode geometryMode = PointCloudGeometryMode::ScreenSprites;
    PointCloudScreenSpriteSizeMode screenSpriteSizeMode = PointCloudScreenSpriteSizeMode::Pixels;
    PointCloudFalloffProfile falloffProfile = PointCloudFalloffProfile::HardDisc;
    PointCloudStylisationMode stylisationMode = PointCloudStylisationMode::Off;
    PointCloudNprPreset nprPreset = PointCloudNprPreset::Watercolor;
    PointCloudColorMode colorMode = PointCloudColorMode::SourceRgb;
    PointCloudColormapId colormap = PointCloudColormapId::Viridis;
    std::array<float, 4> solidColor{0.93F, 0.88F, 0.72F, 1.0F};
    std::array<float, 3> gradientStartColor{0.05F, 0.28F, 0.95F};
    std::array<float, 3> gradientEndColor{0.96F, 0.94F, 0.58F};
    std::array<float, 3> colorizeColor{0.95F, 0.68F, 0.28F};
    float colorizeAmount = 0.0F;
    float stylisationStrength = 1.0F;
    float stylisationColorLevels = 5.0F;
    float stylisationInkStrength = 0.35F;
    float stylisationPaperGrain = 0.35F;
    float stylisationPigmentBleed = 0.45F;
    float brushAspect = 2.2F;
    float strokeJitter = 0.35F;
    float hatchStrength = 0.0F;
    float strokeOpacityVariance = 0.25F;
    float pigmentVariation = 0.0F;
    float pigmentAnimationSpeed = 0.0F;
    float granulationAngleStrength = 0.0F;
    float roughnessMotionStrength = 0.0F;
    float roughnessMotionScale = 1.5F;
    float roughnessMotionSpeed = 0.35F;
    float roughnessMotionThreshold = 0.58F;
    float roughnessMotionGroundId = 1.0F;
    bool roughnessMotionFullLayer = false;
    float exposure = 1.0F;
    float innerRadius = 0.55F;
    float gaussianSharpness = 4.0F;
    float featherPower = 1.6F;
    float waterStreakAspect = 1.0F;
    bool waterTrailStyleGeometry = false;
    // Runtime-only effective activity for one Water Flow trail source.
    float waterFlowActivity = 1.0F;
    // Runtime-only multiplier for authored Water Flow trail speed. This is kept
    // out of the stream scalar payload so speed-only edits can be uniform-only.
    float waterFlowSpeedScale = 1.0F;
    bool solidCenters = true;
    bool flowAnimation = false;
    bool waterPathView = false;
    bool waterTrailOverlay = false;
    // Runtime-only: role-scoped rain reactions require the unified material path.
    bool rainImpactEffects = false;
    bool causticAnimation = false;
    float causticIntensity = 0.0F;
    float causticScale = 4.0F;
    float causticSpeed = 0.55F;
    float causticLineSharpness = 0.72F;
    float causticWarp = 0.35F;
    float causticCellSizeMeters = 0.20F;
    float causticLineWidthMeters = 0.015F;
    float causticFeatherMeters = 0.006F;
    float causticSurfacePointSpacingMeters = 0.005F;
    float causticWarpAmplitudeMeters = 0.045F;
    std::array<float, 3> causticTint{0.62F, 0.88F, 1.0F};
    float causticEmissionBoost = 1.15F;
    float causticOpacityBoost = 0.08F;
    float causticPointSizeBoost = 0.0F;
    float causticPreviewTintAmount = 0.0F;
    float causticPreviewTintRegionId = 0.0F;
    std::int32_t causticMaskFieldSlot = -1;
    std::int32_t causticEdgeFieldSlot = -1;
    std::int32_t causticSeedFieldSlot = -1;
    bool shorelineWaveEnabled = false;
    PointCloudShorelineWaveAlgorithm shorelineWaveAlgorithm =
        PointCloudShorelineWaveAlgorithm::FoamFronts;
    PointCloudHeightFoamShorelineSettings shorelineHeightFoam{};
    float shorelineBoundaryZ = 1.55F;
    float shorelineHeightReachMeters = 0.45F;
    float shorelineEdgeFadeMeters = 0.05F;
    float shorelineDirectionX = 1.0F;
    float shorelineDirectionY = 0.0F;
    float shorelinePatternScale = 1.0F;
    float shorelineWavelengthMeters = 0.25F;
    float shorelineSpeed = 0.55F;
    float shorelineWarp = 0.35F;
    float shorelineTurbulence = 0.06F;
    float shorelineDensity = 0.55F;
    float shorelinePhase = 0.0F;
    float shorelineIntensity = 1.15F;
    float shorelineEmissionAdd = 0.65F;
    float shorelineOpacityAdd = 0.08F;
    float shorelineOpacityMultiply = 1.25F;
    float shorelinePointSizeAdd = 0.0F;
    float shorelinePointSizeMultiply = 1.35F;
    float shorelineColourMix = 0.75F;
    std::array<float, 3> shorelineColour{0.62F, 0.88F, 1.0F};
    std::uint32_t shorelineSeed = 1U;
    invisible_places::style::RenderParameterBinding pointSize;
    invisible_places::style::RenderParameterBinding surfelDiameter;
    invisible_places::style::RenderParameterBinding opacity;
    invisible_places::style::RenderParameterBinding emissiveStrength;
    invisible_places::style::RenderParameterBinding depthFade;
    invisible_places::style::RenderParameterBinding colormapPosition;
};

struct PointBudgetState {
    std::uint64_t totalPoints = 0;
    std::uint64_t activePoints = 0;
    float activeFraction = 1.0F;
    std::vector<std::uint32_t> sampledIndices;

    [[nodiscard]] bool UsesSampledIndices() const { return !sampledIndices.empty(); }
};

struct PointCloudPreviewLodDecision {
    std::uint64_t drawPointCount = 0;
    bool usesPreviewLod = false;
};

struct PointCloudSessionState {
    std::filesystem::path sourcePath;
    std::string displayName;
    bool loaded = false;
    bool active = false;
    bool hasSourceRgb = false;
    bool hasFocusPoint = false;
    std::uint64_t totalPoints = 0;
    invisible_places::io::Bounds3f bounds{};
    invisible_places::io::Float3 focusPoint{};
    std::vector<invisible_places::io::ScalarFieldStats> scalarFields;
    PointBudgetState budget{};
    PointCloudStyleState style{};
};

std::uint64_t ClampPointBudget(std::uint64_t totalPoints, std::uint64_t requestedPoints);
[[nodiscard]] bool PointCloudAlphaContributesDepth(float alpha);
[[nodiscard]] bool PointCloudStyleHasActiveRoughnessMotion(const PointCloudStyleState& style);
[[nodiscard]] bool TimingColouriseStackHasActiveEffects(
    const ResolvedTimingColouriseStack& stack);
[[nodiscard]] bool PointCloudSceneRoleAllowsRoughnessMotion(std::string_view sceneRole);
[[nodiscard]] PointCloudStyleState MakePointCloudStyleForSceneRole(
    PointCloudStyleState style,
    std::string_view sceneRole);
[[nodiscard]] bool PointCloudStyleHasActiveShorelineWaves(const PointCloudStyleState& style);
[[nodiscard]] PointCloudShorelineWaveSettings ExtractPointCloudShorelineWaveSettings(
    const PointCloudStyleState& style);
void ApplyPointCloudShorelineWaveSettings(
    PointCloudStyleState* style,
    const PointCloudShorelineWaveSettings& settings);
[[nodiscard]] PointCloudShorelineWaveSettings CalmPointCloudShorelineWaveSettings();
// True when an authored shoreline still defines a flooded/dry spatial region,
// even if its animation speed is paused. Rain uses this predicate so viewport
// and offline SAND impacts agree at a held shoreline frame.
[[nodiscard]] bool PointCloudStyleHasShorelineWaveRegion(
    const PointCloudStyleState& style);
[[nodiscard]] bool PointCloudStyleHasActiveCaustics(const PointCloudStyleState& style);
[[nodiscard]] bool PointCloudStyleUsesWorldSizedScreenSprites(const PointCloudStyleState& style);
[[nodiscard]] WaterFlowActivityScales ResolveWaterFlowActivityScales(
    float effectiveActivity,
    float trailSeed);
[[nodiscard]] float SanitizeWaterFlowSpeedScale(float speedScale);
[[nodiscard]] float ShorelineWaveHeightMask(
    float boundaryZ,
    float reachMeters,
    float edgeFadeMeters,
    float worldZ);
[[nodiscard]] float NormalizeHeightFoamBreakZ(
    float runupZ,
    float offshoreReachMeters,
    float edgeFadeMeters,
    float breakZ);
[[nodiscard]] float WorldDiameterToScreenPointSizePixels(
    float diameterMeters,
    float viewDepth,
    float projectionScaleY,
    float viewportHeight);
[[nodiscard]] PointCloudStyleState MakeFastBasicPointCloudStyle(
    const PointCloudStyleState& sourceStyle,
    bool hasSourceRgb);
[[nodiscard]] PointCloudDensityCompensation ResolvePointCloudDensityCompensation(
    float displaySpacingMeters,
    std::uint64_t displayPointCount,
    float referenceSpacingMeters,
    std::uint64_t referencePointCount);
[[nodiscard]] PointCloudDensityCompensation SanitizePointCloudDensityCompensation(
    PointCloudDensityCompensation compensation);
[[nodiscard]] PointCloudMaterialVariant ResolvePointCloudMaterialVariant(const PointCloudStyleState& style);
[[nodiscard]] PointCloudMaterialVariant ResolvePointCloudMaterialVariant(
    const PointCloudStyleState& style,
    PointCloudDensityCompensation densityCompensation);
[[nodiscard]] PointCloudMaterialVariant ResolvePointCloudMaterialVariant(
    const PointCloudStyleState& style,
    PointCloudDensityCompensation densityCompensation,
    bool requiresUnifiedProceduralEffects);
[[nodiscard]] const char* PointCloudMaterialVariantName(PointCloudMaterialVariant variant);
PointBudgetState MakePointBudgetState(std::uint64_t totalPoints, std::uint64_t requestedPoints);
PointBudgetState MakePointBudgetState(
    const invisible_places::io::LoadedPointCloud& cloud,
    std::uint64_t requestedPoints);
std::uint64_t ResolveInteractivePointBudget(
    const PointBudgetState& budget,
    bool interactionActive,
    std::uint64_t interactivePointCap);
PointCloudPreviewLodDecision ResolvePointCloudPreviewLod(
    const PointBudgetState& budget,
    PointCloudPreviewLodMode mode,
    bool cameraNavigationActive,
    bool cameraPlaybackActive,
    std::uint64_t lodTargetPoints);
std::vector<std::uint32_t> GenerateDeterministicSampleIndices(
    std::uint64_t totalPoints,
    std::uint64_t requestedPoints);
std::vector<std::uint32_t> GenerateSpatialSampleIndices(
    const std::vector<invisible_places::io::Float3>& positions,
    const invisible_places::io::Bounds3f& bounds,
    std::uint64_t requestedPoints);
std::vector<std::uint32_t> GenerateFrustumUnionPointIndices(
    const std::vector<invisible_places::io::Float3>& positions,
    const invisible_places::io::Bounds3f& bounds,
    std::span<const glm::mat4> viewProjections,
    std::uint32_t gridDimension = 64U);
std::vector<std::uint32_t> GenerateSurfelEncodedSampleIndices(
    const std::vector<std::uint32_t>& sampledPointIndices);

}  // namespace invisible_places::renderer::pointcloud
