#pragma once

#include "io/PointCloudData.hpp"
#include "style/RenderParameterBinding.hpp"

#include <array>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace invisible_places::renderer::pointcloud {

inline constexpr float kInactivePointSizeDefault = 1.0F;
inline constexpr float kInactiveSurfelDiameterDefault = 0.005F;
inline constexpr float kInactiveOpacityDefault = 1.0F;
inline constexpr float kInactiveEmissionDefault = 0.0F;
inline constexpr float kInactiveXrayDefault = 0.0F;
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
    HighContrast
};

enum class PointCloudGeometryMode {
    ScreenSprites,
    WorldSurfels,
    CameraFacingWorldSprites
};

enum class PointCloudDepthContribution {
    None,
    AlphaThreshold,
    Always
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
    FastBasic,
    Raytraced
};

enum class PointCloudRaycastPrimitiveMode {
    StyleSurfels,
    SoftDensitySpheres
};

enum class PointCloudMaterialVariant {
    OpaqueHardDisc,
    ConstantSimple,
    Unified
};

struct PointCloudStyleState {
    PointCloudStyleState();

    PointCloudGeometryMode geometryMode = PointCloudGeometryMode::ScreenSprites;
    PointCloudDepthContribution depthContribution = PointCloudDepthContribution::None;
    PointCloudFalloffProfile falloffProfile = PointCloudFalloffProfile::HardDisc;
    PointCloudStylisationMode stylisationMode = PointCloudStylisationMode::Off;
    PointCloudNprPreset nprPreset = PointCloudNprPreset::Watercolor;
    PointCloudColorMode colorMode = PointCloudColorMode::SourceRgb;
    PointCloudColormapId colormap = PointCloudColormapId::Viridis;
    std::array<float, 4> solidColor{0.93F, 0.88F, 0.72F, 1.0F};
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
    float depthFalloff = 80.0F;
    float depthBias = 0.0005F;
    float frontAlpha = 0.16F;
    float hiddenAlpha = 0.08F;
    float densityScale = 1.0F;
    float densityClamp = 64.0F;
    float depthAlphaThreshold = 0.5F;
    bool solidCenters = true;
    bool flowAnimation = false;
    bool waterPathView = false;
    invisible_places::style::RenderParameterBinding pointSize;
    invisible_places::style::RenderParameterBinding surfelDiameter;
    invisible_places::style::RenderParameterBinding opacity;
    invisible_places::style::RenderParameterBinding emissiveStrength;
    invisible_places::style::RenderParameterBinding xrayStrength;
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
[[nodiscard]] bool PointCloudStyleUsesDepthPrepass(const PointCloudStyleState& style);
[[nodiscard]] bool PointCloudStyleUsesDepthPrepass(const PointCloudStyleState& style, bool sceneHasActiveXray);
[[nodiscard]] bool PointCloudAlphaContributesDepth(const PointCloudStyleState& style, float alpha);
[[nodiscard]] bool PointCloudStyleHasActiveXray(const PointCloudStyleState& style);
[[nodiscard]] bool PointCloudStyleHasActiveRoughnessMotion(const PointCloudStyleState& style);
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
std::vector<std::uint32_t> GenerateSurfelEncodedSampleIndices(
    const std::vector<std::uint32_t>& sampledPointIndices);

}  // namespace invisible_places::renderer::pointcloud
