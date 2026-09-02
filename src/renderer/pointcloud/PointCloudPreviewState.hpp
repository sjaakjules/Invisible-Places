#pragma once

#include "io/PointCloudData.hpp"
#include "style/RenderParameterBinding.hpp"

#include <glm/mat4x4.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <span>
#include <stop_token>
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
inline constexpr std::size_t kTimingColouriseMaxEffects = 8U;
inline constexpr std::size_t kTimingColouriseLutSamples = 64U;
// Keep these renderer payload limits in lockstep with the authored timing
// model and pointcloud_timing_colourise.glsl. Inward fades cannot exceed the
// selected span; outward fades may cover many selected spans.
inline constexpr float kTimingColouriseMaximumInwardEdgeFade = 1.0F;
inline constexpr float kTimingColouriseMaximumOutwardEdgeFade = 1'000'000.0F;

// Renderer-facing timing colourise data is deliberately independent from the
// authored timing model. Callers resolve field names to each layer's local
// scalar slot before populating this fixed-capacity payload.
enum class TimingColouriseSource : std::uint32_t {
    ScalarField = 0U,
    NormalX = 1U,
    NormalY = 2U,
    NormalZ = 3U
};

enum class TimingColouriseOutput : std::uint32_t {
    Colourise = 0U,
    Emissive = 1U,
};

// Compositing mode for a colourise effect against the colour beneath it
// (earlier slots, or the base cloud colour). Values match
// timing::TimingColouriseBlendMode and the constants in
// shaders/pointcloud_timing_colourise.glsl.
enum class TimingColouriseBlendMode : std::uint32_t {
    Normal = 0U,
    Multiply = 1U,
    Screen = 2U,
    Add = 3U,
    Divide = 4U,
    VividLight = 5U,
    ColorBurn = 6U,
};

enum class TimingColouriseBlendCompositionMode : std::uint32_t {
    PrimaryOnly = 0U,
    Crossfade = 1U,
    ApplyAfter = 2U,
};

// Divisors in Divide, Vivid Light, and Color Burn are floored here so the
// fold stays finite; the hard clip this produces at extreme wash values
// matches the After Effects float-space look once the output clamps.
inline constexpr float kTimingColouriseBlendDivisorFloor = 1.0e-3F;

// One colourise step is blended = base * scale + offset for every supported
// mode, because each mode is linear in the base colour once the wash colour
// is known (Vivid Light branches on the wash, never the base). That keeps a
// whole stack foldable into one per-channel scale/offset pair across the
// vertex->fragment interface. Mirrored by FoldTimingColouriseBlendStep in
// shaders/pointcloud_timing_colourise.glsl; the two must stay identical.
inline void ResolveTimingColouriseBlendCoefficients(
    TimingColouriseBlendMode mode,
    const std::array<float, 3>& wash,
    std::array<float, 3>* branchScale,
    std::array<float, 3>* branchOffset) {
    for (std::size_t channel = 0U; channel < 3U; ++channel) {
        const float washValue = wash[channel];
        float resolvedScale = 0.0F;
        float resolvedOffset = washValue;
        switch (mode) {
            case TimingColouriseBlendMode::Normal:
                break;
            case TimingColouriseBlendMode::Multiply:
                resolvedScale = washValue;
                resolvedOffset = 0.0F;
                break;
            case TimingColouriseBlendMode::Screen:
                resolvedScale = 1.0F - washValue;
                resolvedOffset = washValue;
                break;
            case TimingColouriseBlendMode::Add:
                resolvedScale = 1.0F;
                resolvedOffset = washValue;
                break;
            case TimingColouriseBlendMode::Divide:
                resolvedScale = 1.0F /
                              std::max(
                                  washValue,
                                  kTimingColouriseBlendDivisorFloor);
                resolvedOffset = 0.0F;
                break;
            case TimingColouriseBlendMode::VividLight:
                if (washValue < 0.5F) {
                    resolvedScale =
                        1.0F /
                        std::max(
                            2.0F * washValue,
                            kTimingColouriseBlendDivisorFloor);
                    resolvedOffset = 1.0F - resolvedScale;
                } else {
                    resolvedScale =
                        1.0F /
                        std::max(
                            2.0F * (1.0F - washValue),
                            kTimingColouriseBlendDivisorFloor);
                    resolvedOffset = 0.0F;
                }
                break;
            case TimingColouriseBlendMode::ColorBurn:
                resolvedScale =
                    1.0F /
                    std::max(
                        washValue,
                        kTimingColouriseBlendDivisorFloor);
                resolvedOffset = 1.0F - resolvedScale;
                break;
        }
        (*branchScale)[channel] = resolvedScale;
        (*branchOffset)[channel] = resolvedOffset;
    }
}

inline void ComposeTimingColouriseResolvedBlendStep(
    const std::array<float, 3>& branchScale,
    const std::array<float, 3>& branchOffset,
    float amount,
    std::array<float, 3>* scale,
    std::array<float, 3>* offset) {
    for (std::size_t channel = 0U; channel < 3U; ++channel) {
        const float stepScale =
            (1.0F - amount) + amount * branchScale[channel];
        const float stepOffset = amount * branchOffset[channel];
        (*scale)[channel] *= stepScale;
        (*offset)[channel] =
            (*offset)[channel] * stepScale + stepOffset;
    }
}

inline void ComposeTimingColouriseBlendStep(
    TimingColouriseBlendMode mode,
    const std::array<float, 3>& wash,
    float amount,
    std::array<float, 3>* scale,
    std::array<float, 3>* offset) {
    std::array<float, 3> branchScale{};
    std::array<float, 3> branchOffset{};
    ResolveTimingColouriseBlendCoefficients(
        mode, wash, &branchScale, &branchOffset);
    ComposeTimingColouriseResolvedBlendStep(
        branchScale, branchOffset, amount, scale, offset);
}

inline void ComposeTimingColouriseBlendStep(
    TimingColouriseBlendMode primaryMode,
    TimingColouriseBlendMode secondaryMode,
    TimingColouriseBlendCompositionMode compositionMode,
    float blendMix,
    const std::array<float, 3>& wash,
    float amount,
    std::array<float, 3>* scale,
    std::array<float, 3>* offset) {
    const float safeMix = std::clamp(blendMix, 0.0F, 1.0F);
    if (compositionMode ==
        TimingColouriseBlendCompositionMode::PrimaryOnly) {
        ComposeTimingColouriseBlendStep(
            primaryMode, wash, amount, scale, offset);
        return;
    }
    if (compositionMode ==
        TimingColouriseBlendCompositionMode::ApplyAfter) {
        ComposeTimingColouriseBlendStep(
            primaryMode, wash, amount, scale, offset);
        ComposeTimingColouriseBlendStep(
            secondaryMode, wash, amount * safeMix, scale, offset);
        return;
    }

    std::array<float, 3> primaryScale{};
    std::array<float, 3> primaryOffset{};
    std::array<float, 3> secondaryScale{};
    std::array<float, 3> secondaryOffset{};
    ResolveTimingColouriseBlendCoefficients(
        primaryMode, wash, &primaryScale, &primaryOffset);
    ResolveTimingColouriseBlendCoefficients(
        secondaryMode, wash, &secondaryScale, &secondaryOffset);
    for (std::size_t channel = 0U; channel < 3U; ++channel) {
        primaryScale[channel] = std::lerp(
            primaryScale[channel], secondaryScale[channel], safeMix);
        primaryOffset[channel] = std::lerp(
            primaryOffset[channel], secondaryOffset[channel], safeMix);
    }
    ComposeTimingColouriseResolvedBlendStep(
        primaryScale, primaryOffset, amount, scale, offset);
}

// Sequential form of the same step for callers that hold the base colour
// (the offline renderer and tests).
inline std::array<float, 3> ApplyTimingColouriseBlendStep(
    TimingColouriseBlendMode mode,
    const std::array<float, 3>& base,
    const std::array<float, 3>& wash,
    float amount) {
    std::array<float, 3> scale{1.0F, 1.0F, 1.0F};
    std::array<float, 3> offset{0.0F, 0.0F, 0.0F};
    ComposeTimingColouriseBlendStep(mode, wash, amount, &scale, &offset);
    return {
        base[0] * scale[0] + offset[0],
        base[1] * scale[1] + offset[1],
        base[2] * scale[2] + offset[2],
    };
}

inline std::array<float, 3> ApplyTimingColouriseBlendStep(
    TimingColouriseBlendMode primaryMode,
    TimingColouriseBlendMode secondaryMode,
    TimingColouriseBlendCompositionMode compositionMode,
    float blendMix,
    const std::array<float, 3>& base,
    const std::array<float, 3>& wash,
    float amount) {
    std::array<float, 3> scale{1.0F, 1.0F, 1.0F};
    std::array<float, 3> offset{0.0F, 0.0F, 0.0F};
    ComposeTimingColouriseBlendStep(
        primaryMode,
        secondaryMode,
        compositionMode,
        blendMix,
        wash,
        amount,
        &scale,
        &offset);
    return {
        base[0] * scale[0] + offset[0],
        base[1] * scale[1] + offset[1],
        base[2] * scale[2] + offset[2],
    };
}

struct ResolvedTimingColouriseEffect {
    bool enabled = false;
    TimingColouriseSource source = TimingColouriseSource::ScalarField;
    TimingColouriseOutput output = TimingColouriseOutput::Colourise;
    std::int32_t scalarFieldSlot = -1;
    float lowerBound = 0.0F;
    float upperBound = 1.0F;
    // Signed fraction of the selected [lower, upper] span used by each edge
    // fade, independently per edge. Positive values fade inward up to one
    // span; negative values fade outward and may exceed one span.
    float edgeFadeLowerFraction = 0.10F;
    float edgeFadeUpperFraction = 0.10F;
    // A signed scalar applied after the field bounds/fade mask. Positive
    // values add emission; negative values darken point colour, with -1 fully
    // dark at full mask. Colourise effects ignore this value.
    float emissiveLevel = 0.0F;
    // Compositing mode against the colour beneath this slot. Emissive
    // output ignores it.
    TimingColouriseBlendMode blendMode = TimingColouriseBlendMode::Normal;
    TimingColouriseBlendMode secondaryBlendMode =
        TimingColouriseBlendMode::Multiply;
    TimingColouriseBlendCompositionMode blendCompositionMode =
        TimingColouriseBlendCompositionMode::PrimaryOnly;
    float blendMix = 1.0F;
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

// Chooses the camera direction used by the optional GPU transparency sort.
// PerFrame preserves the original behaviour. FullAnimation and MovingAverage
// are resolved by animation-aware callers and fall back to PerFrame when no
// animation path is available. FixedVertical sorts bottom-up along world Z
// from the cloud's own bounds — camera-independent, so aerial/top-down work
// reuses a single cached ordering with no animation path required.
enum class PointCloudGpuSortMode : std::uint32_t {
    PerFrame = 0U,
    FullAnimation = 1U,
    MovingAverage = 2U,
    FixedVertical = 3U,
};

// Saved policy for deciding which semantic scene roles contribute to and
// consume the soft-edge depth surface. Uniform preserves the historical
// behaviour. RockOccluder keeps ROCK self-occlusion, lets SAND receive that
// occlusion without allowing overlapping sand scans to fight for ownership,
// and leaves sparse VEG as a transparent overlay. Custom exposes every role.
enum class PointCloudDepthRolePolicy : std::uint32_t {
    Uniform = 0U,
    RockOccluder = 1U,
    Custom = 2U,
};

enum class PointCloudDepthParticipation : std::uint32_t {
    Disabled = 0U,
    TestOnly = 1U,
    WriteAndTest = 2U,
};

// Optional cache-backed opacity selection for overlapping survey passes.
// DrawAll is an explicit comparison mode; Original also avoids a sidecar
// lookup when an older cache or an ordinary point cloud is used.
enum class PointCloudSurfaceStabilityMode : std::uint32_t {
    Original = 0U,
    DrawAll = 1U,
    DensityContinuity = 2U,
    PreferLower = 3U,
    PreferUpper = 4U,
    SoftSeparation = 5U,
};

enum class PointCloudSurfaceStabilityPolicy : std::uint32_t {
    Uniform = 0U,
    StableRoles = 1U,
    Custom = 2U,
};

// How emission resolves against overlapping fragments. Accumulated preserves
// the established weighted-blended behaviour: every fragment's linear
// emission energy (including density compensation) sums before one
// exponential response, so deep stacks keep brightening. Saturated applies
// the exponential response per fragment and folds it into the blended
// colour, bounding glow at the front surface — the response the GPU sorted
// path originally shipped with. Both transparency paths honour the choice
// identically.
enum class PointCloudEmissionResponse : std::uint32_t {
    Accumulated = 0U,
    Saturated = 1U,
};

enum class PointCloudMaterialVariant {
    OpaqueHardDisc,
    ConstantSimple,
    Unified
};

enum class PointCloudShorelineWaveAlgorithm {
    FoamFronts,
    HeightFoam,
    ContinuousBands
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
// on PointCloudStyleState for shader/persistence compatibility. Continuous
// Bands intentionally shares this control bank because it is an alternate
// motion model for the same foam look. Keeping the bank separate from Height
// Foam lets named Shoreline profiles preserve both control families while
// changing only the settings of a Water-owned Shoreline profile or effect.
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
    // Response curve for the faded body wash between crests: 1 keeps the
    // authored look, lower values sharpen it toward line-like peaks, higher
    // values lift it. Foam Fronts only.
    float backgroundWash = 1.0F;
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
    // Schema 84: resolved from the timing-run timelines. When false, the
    // standing-water gap-fill layer skips this instance (both the primary
    // style bake and the additional-instance uniform array).
    bool applyToWaterFill = true;
    PointCloudShorelineWaveAlgorithm algorithm =
        PointCloudShorelineWaveAlgorithm::FoamFronts;
    PointCloudFoamFrontsShorelineSettings foamFronts{};
    PointCloudHeightFoamShorelineSettings heightFoam{};
};

struct PointCloudShorelineWaveProfile {
    std::string name;
    PointCloudShorelineWaveSettings settings{};
    // Object edits are ordinary project-owned Water profiles, but retain
    // explicit ownership metadata so an object's next edit overwrites its
    // one derived <base>_<object name> copy instead of creating a chain of
    // point-visual-style `_edited` profiles.
    bool objectOverride = false;
    std::uint32_t shorelineInstanceId = 0U;
    std::string baseProfileName;
};

// One shoreline effect in the project-owned Water list. `profileName` is the
// currently applied saved profile; `baseProfileName` remains the named source
// when `profileName` is this object's derived override. Settings are retained
// as the resolved snapshot used by rendering and legacy recovery, never as
// part of a saved point visual.
struct PointCloudShorelineInstance {
    std::uint32_t id = 0U;
    std::string name = "Shoreline";
    bool enabled = true;
    std::string profileName;
    std::string baseProfileName;
    PointCloudShorelineWaveSettings settings{};
};

// The renderer retains one legacy primary uniform bank plus four additive
// banks. Water authoring treats all five identically in one ordered list; the
// first resolved effect is packed through the primary bank only as an
// internal transport detail.
inline constexpr std::size_t kMaxAdditionalShorelineInstances = 4U;
inline constexpr std::size_t kMaxShorelineInstances =
    kMaxAdditionalShorelineInstances + 1U;

// Lower bound for the per-fragment coverage correction: a display bundle is
// never assumed to over-cover its appearance reference by more than 5x, so a
// sparse pseudo-canonical reference (Site1's split "1 mm" clouds) cannot drive
// per-role alpha to extremes that make roles diverge in the live view. The
// floor sits below every measured Scene3 correction (smallest: VEG 0.2456),
// so the validated Scene3 live/export parity is bit-for-bit unchanged.
inline constexpr float kPointCloudCoverageCorrectionFloor = 0.2F;

struct PointCloudDensityCompensation {
    float footprintScale = 1.0F;
    float coverageCorrection = 1.0F;
};

// Live-preview-only density roles used by adaptive HQ. The complete coarse
// layer and its compact fine companion share one depth transition, but each
// applies the opposite stable point-selection policy on the GPU. Disabled is
// the default for every ordinary/export layer.
enum class PointCloudAdaptiveDensityRole : std::uint32_t {
    Disabled = 0U,
    Fine = 1U,
    Coarse = 2U,
    // The previous publish's fine patches while a publish crossfade runs.
    // Each point hands off to the incoming Fine layer exactly once during
    // the ramp via a shared position-keyed hash, so the pair never double
    // draws and never opens a gap. Fully handed off (drawing nothing) at
    // steady state.
    FineOutgoing = 3U,
};

struct PointCloudAdaptiveDensityTransition {
    float startDepthMeters = 0.0F;
    float switchDepthMeters = 0.0F;
    float endDepthMeters = 0.0F;
    // Fine source data is prepared slightly beyond the visible transition so
    // resize jitter and the cache guard cannot reveal an unloaded edge.
    float preparedFineDepthMeters = 0.0F;
    // Runtime-only publish crossfade. While a refresh is in flight the
    // complete coarse layer draws ungated for coverage; at 1 the published
    // steady state removes the redundant near-zone coarse points entirely.
    // Ramping 0 -> 1 after a publish spreads that removal across a short
    // window instead of one visible frame. Exports never carry an adaptive
    // transition, so this never affects rendered output.
    float coarseEngage = 1.0F;

    [[nodiscard]] bool Valid() const {
        return startDepthMeters > 0.0F &&
               switchDepthMeters >= startDepthMeters &&
               endDepthMeters > switchDepthMeters &&
               preparedFineDepthMeters >= endDepthMeters;
    }
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
    // Advanced transparency controls are deliberately opt-in so existing
    // named visuals retain their established weighted-blended appearance and
    // render cost. GPU sorting currently applies to screen-sprite geometry;
    // surfel geometry keeps weighted blended transparency.
    bool gpuBackToFrontSorting = false;
    PointCloudGpuSortMode gpuSortMode = PointCloudGpuSortMode::PerFrame;
    // Symmetric animation-time window used by MovingAverage. Seconds keep the
    // authored result independent of preview/export frame rate.
    float gpuSortWindowSeconds = 1.0F;
    // Writes only sufficiently opaque point cores to the depth prepass, then
    // admits nearby fragments within depthPrepassToleranceMeters. This keeps
    // soft/Gaussian neighbours blendable while rejecting genuinely deeper
    // surfaces (for example the ground below an overhang).
    bool depthPrepassEnabled = false;
    float depthPrepassAlphaThreshold = 0.35F;
    float depthPrepassToleranceMeters = 0.02F;
    PointCloudDepthRolePolicy depthRolePolicy =
        PointCloudDepthRolePolicy::Uniform;
    PointCloudDepthParticipation rockDepthParticipation =
        PointCloudDepthParticipation::WriteAndTest;
    PointCloudDepthParticipation sandDepthParticipation =
        PointCloudDepthParticipation::WriteAndTest;
    PointCloudDepthParticipation vegetationDepthParticipation =
        PointCloudDepthParticipation::WriteAndTest;
    // Runtime-only role resolution. Saved visuals persist the policy and
    // custom role values above; MakePointCloudStyleForSceneRole resolves this
    // field for each ROCK/SAND/VEG layer before it reaches a renderer.
    PointCloudDepthParticipation effectiveDepthParticipation =
        PointCloudDepthParticipation::WriteAndTest;
    // Stable Roles softly culls ROCK, selects a density/continuity SAND
    // surface, and leaves sparse VEG untouched. Custom exposes all roles.
    PointCloudSurfaceStabilityPolicy surfaceStabilityPolicy =
        PointCloudSurfaceStabilityPolicy::Uniform;
    PointCloudSurfaceStabilityMode uniformSurfaceStabilityMode =
        PointCloudSurfaceStabilityMode::Original;
    PointCloudSurfaceStabilityMode rockSurfaceStabilityMode =
        PointCloudSurfaceStabilityMode::SoftSeparation;
    PointCloudSurfaceStabilityMode sandSurfaceStabilityMode =
        PointCloudSurfaceStabilityMode::DensityContinuity;
    PointCloudSurfaceStabilityMode vegetationSurfaceStabilityMode =
        PointCloudSurfaceStabilityMode::DrawAll;
    // Runtime-only role resolution.
    PointCloudSurfaceStabilityMode effectiveSurfaceStabilityMode =
        PointCloudSurfaceStabilityMode::Original;
    // Zero preserves authored opacity; one applies the complete sidecar
    // weight, with intermediate values useful for safe A/B comparisons.
    float surfaceStabilityInfluence = 1.0F;
    // 1.0 exactly preserves the historical WBOIT depth weighting. Higher
    // values progressively favour nearer fragments using logarithmic depth.
    float depthWeightStrength = 1.0F;
    PointCloudEmissionResponse emissionResponse =
        PointCloudEmissionResponse::Accumulated;
    // Fades points whose stored normal faces away from the camera — for
    // example back-facing survey returns seen through an overhang from
    // above. Fully visible up to the start angle (normal vs the direction to
    // the camera), hidden beyond the end angle, smooth between. Points
    // without normals are never culled.
    bool normalCullEnabled = false;
    float normalCullStartDegrees = 75.0F;
    float normalCullEndDegrees = 105.0F;
    bool flowAnimation = false;
    bool waterPathView = false;
    bool waterTrailOverlay = false;
    // Runtime-only: role-scoped rain reactions require the unified material path.
    bool rainImpactEffects = false;
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
    float shorelineBackgroundWash = 1.0F;
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
[[nodiscard]] bool PointCloudDepthPrepassWrites(
    const PointCloudStyleState& style);
[[nodiscard]] bool PointCloudDepthPrepassTests(
    const PointCloudStyleState& style);
[[nodiscard]] float ResolvePointCloudSurfaceStabilityWeight(
    PointCloudSurfaceStabilityMode mode,
    std::uint32_t packedWeights,
    float influence = 1.0F);
[[nodiscard]] bool PointCloudStyleHasActiveShorelineWaves(const PointCloudStyleState& style);
[[nodiscard]] bool PointCloudShorelineWaveSettingsHasActiveMotion(
    const PointCloudShorelineWaveSettings& settings);
[[nodiscard]] PointCloudShorelineWaveSettings ExtractPointCloudShorelineWaveSettings(
    const PointCloudStyleState& style);
void ApplyPointCloudShorelineWaveSettings(
    PointCloudStyleState* style,
    const PointCloudShorelineWaveSettings& settings);
[[nodiscard]] PointCloudShorelineWaveSettings CalmPointCloudShorelineWaveSettings();
// Resolves a saved profile name for one Shoreline object. Shared named bases
// are available to every object; an object-specific copy is available only to
// its owner. Whitespace around authored names is ignored.
[[nodiscard]] const PointCloudShorelineWaveProfile*
FindPointCloudShorelineWaveProfile(
    std::span<const PointCloudShorelineWaveProfile> profiles,
    std::uint32_t shorelineInstanceId,
    std::string_view profileName);
// Returns the stable human-facing name for one Shoreline object's derived
// profile without colliding with a saved base or another object's copy. An
// existing copy owned by the same object and base is ignored so ordinary
// rename/update operations keep their current unsuffixed name when possible.
[[nodiscard]] std::string UniquePointCloudShorelineObjectProfileName(
    std::span<const PointCloudShorelineWaveProfile> profiles,
    std::string_view baseProfileName,
    std::string_view objectName,
    std::uint32_t shorelineInstanceId);
// True when an authored shoreline still defines a flooded/dry spatial region,
// even if its animation speed is paused. Rain uses this predicate so viewport
// and offline SAND impacts agree at a held shoreline frame.
[[nodiscard]] bool PointCloudStyleHasShorelineWaveRegion(
    const PointCloudStyleState& style);
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
// Fast Basic is the scene/default renderer, but generated Flow trails retain
// their authored Beauty material. This keeps the expensive material path
// scoped to the overlay instead of silently promoting the entire scene.
[[nodiscard]] PointCloudRendererMode ResolvePointCloudLayerRendererMode(
    PointCloudRendererMode requestedMode,
    bool generatedWaterOverlay,
    const PointCloudStyleState& style);
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
// Chooses the depth at which a 5 mm sample projects to targetCoarsePixels.
// Fine points remain complete before the blend band, the complete 5 mm layer
// takes over after it, and a stable stochastic transition avoids a hard ring.
[[nodiscard]] PointCloudAdaptiveDensityTransition
ResolvePointCloudAdaptiveDensityTransition(
    float projectionScaleY,
    float viewportHeightPixels,
    float coarseSpacingMeters = 0.005F,
    float targetCoarsePixels = 1.0F);
[[nodiscard]] float PointCloudAdaptiveDensityCoarseWeight(
    float viewDepthMeters,
    const PointCloudAdaptiveDensityTransition& transition);
[[nodiscard]] float PointCloudAdaptiveDensityKeepProbability(
    PointCloudAdaptiveDensityRole role,
    float viewDepthMeters,
    const PointCloudAdaptiveDensityTransition& transition);
// Density scaling applies to the signed, composed authored geometry only.
// The antialias support is a fixed screen-space margin added after the scale
// (identical across display densities), and camera depth-of-field is a
// post-density image effect added after that. The caller applies its
// geometry-specific final clamp.
[[nodiscard]] float ResolvePointCloudDensityAdjustedFootprint(
    float authoredFootprint,
    float antialiasFootprint,
    float postDensityExpansion,
    PointCloudDensityCompensation compensation);
[[nodiscard]] float ClampPointCloudResolvedSurfelDiameter(
    float resolvedDiameter,
    float maximumDiameter);
[[nodiscard]] PointCloudMaterialVariant ResolvePointCloudMaterialVariant(const PointCloudStyleState& style);
[[nodiscard]] PointCloudMaterialVariant ResolvePointCloudMaterialVariant(
    const PointCloudStyleState& style,
    PointCloudDensityCompensation densityCompensation);
[[nodiscard]] PointCloudMaterialVariant ResolvePointCloudMaterialVariant(
    const PointCloudStyleState& style,
    PointCloudDensityCompensation densityCompensation,
    bool requiresUnifiedProceduralEffects);
// Preview performance mode may retain weighted density compensation while
// depth-culling an authored style that is otherwise fully opaque.
[[nodiscard]] bool PointCloudStyleSupportsPreviewDepthCulling(
    const PointCloudStyleState& style);
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
struct FrustumPointGridLookup {
    const invisible_places::io::Float3* positionsIdentity = nullptr;
    std::size_t pointCount = 0U;
    std::uint32_t dimension = 0U;
    invisible_places::io::Bounds3f bounds{};
    // One precomputed 3-D cell id per source point, retained in source order.
    // Reusing it turns later guard refreshes into a compact integer scan and
    // preserves the sorted-unique point-index contract without redoing voxel
    // coordinates from float positions.
    std::vector<std::uint32_t> pointCellIndices;

    [[nodiscard]] bool Matches(
        const std::vector<invisible_places::io::Float3>& positions,
        std::uint32_t requestedDimension) const;
};
[[nodiscard]] FrustumPointGridLookup BuildFrustumPointGridLookup(
    const std::vector<invisible_places::io::Float3>& positions,
    const invisible_places::io::Bounds3f& bounds,
    std::uint32_t gridDimension = 64U,
    std::stop_token stopToken = {});
std::vector<std::uint32_t> GenerateFrustumUnionPointIndices(
    const std::vector<invisible_places::io::Float3>& positions,
    const invisible_places::io::Bounds3f& bounds,
    std::span<const glm::mat4> viewProjections,
    std::uint32_t gridDimension = 64U,
    std::stop_token stopToken = {},
    const FrustumPointGridLookup* pointGridLookup = nullptr);
std::vector<std::uint32_t> GenerateSurfelEncodedSampleIndices(
    const std::vector<std::uint32_t>& sampledPointIndices);

}  // namespace invisible_places::renderer::pointcloud
