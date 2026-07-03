#pragma once

#include "io/PointCloudData.hpp"
#include "style/RenderParameterBinding.hpp"

#include <glm/mat4x4.hpp>

#include <array>
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
    float exposure = 1.0F;
    float innerRadius = 0.55F;
    float gaussianSharpness = 4.0F;
    float featherPower = 1.6F;
    float waterStreakAspect = 1.0F;
    bool solidCenters = true;
    bool flowAnimation = false;
    bool waterPathView = false;
    bool waterTrailOverlay = false;
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
[[nodiscard]] bool PointCloudSceneRoleAllowsRoughnessMotion(std::string_view sceneRole);
[[nodiscard]] PointCloudStyleState MakePointCloudStyleForSceneRole(
    PointCloudStyleState style,
    std::string_view sceneRole);
[[nodiscard]] bool PointCloudStyleHasActiveShorelineWaves(const PointCloudStyleState& style);
[[nodiscard]] bool PointCloudStyleHasActiveCaustics(const PointCloudStyleState& style);
[[nodiscard]] bool PointCloudStyleUsesWorldSizedScreenSprites(const PointCloudStyleState& style);
[[nodiscard]] float ShorelineWaveHeightMask(
    float boundaryZ,
    float reachMeters,
    float edgeFadeMeters,
    float worldZ);
[[nodiscard]] float WorldDiameterToScreenPointSizePixels(
    float diameterMeters,
    float viewDepth,
    float projectionScaleY,
    float viewportHeight);
[[nodiscard]] PointCloudStyleState MakeFastBasicPointCloudStyle(
    const PointCloudStyleState& sourceStyle,
    bool hasSourceRgb);
[[nodiscard]] PointCloudMaterialVariant ResolvePointCloudMaterialVariant(const PointCloudStyleState& style);
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
