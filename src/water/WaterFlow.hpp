#pragma once

#include "io/MeshData.hpp"
#include "io/PointCloudData.hpp"
#include "water/RainSimulation.hpp"
#include "water/WaterSeepagePulseField.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <stop_token>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include <glm/mat3x3.hpp>
#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>

namespace invisible_places::water {

enum class WaterScaleMode {
    Aerial,
    Mid,
    Detail
};

enum class WaterEmitterOrigin {
    Manual,
    AutoSuggested,
    Propagated
};

enum class WaterEmitterStatus {
    Candidate,
    Accepted,
    Disabled
};

enum class WaterSourceSettingsAssignment {
    Default,
    Custom,
    LinkedEmitter
};

enum class WaterEffectBlendMode {
    Add,
    Max,
    Multiply,
    Screen,
    Override
};

enum class WaterSeepageQuality {
    Auto,
    Low,
    Balanced,
    High
};

enum class WaterSeepagePattern {
    WetRockSheen,
    ChaoticBloom,
    WettingTrickle,
    ContourPulses
};

enum class WaterScenarioInterpolation {
    Smooth,
    Linear,
    Hold,
    // Monotone cubic interpolation that carries velocity through keys while
    // the authored values continue in the same direction. Unlike Smooth,
    // this only comes to rest at reversals, flat holds, or a mode boundary.
    SmoothVelocity,
    // Chord-length-aware Catmull-Rom interpolation using alpha = 0.5. This
    // follows a fluid C1 curve through every key without the uniform
    // Catmull-Rom loops and cusps caused by unevenly spaced control points.
    CentripetalCatmullRom,
    // Keyed-setting tracks only: a cubic Bezier segment whose outgoing and
    // incoming control points are authored directly on the value graph.
    SplineHandles,
    // Keyed-setting tracks only: the key follows its track's
    // defaultInterpolation, so restyling the whole setting is one edit while
    // explicitly overridden keys keep their own mode. Never valid on
    // scenario or node keys; evaluation resolves it before segment math.
    TrackDefault
};

struct WaterPathGenerationSettings {
    WaterScaleMode legacyScaleMode = WaterScaleMode::Mid;
    bool autoTune = true;
    float supportVoxelSize = 0.008F;
    float maxBridgeDistance = 0.080F;
    float smoothing = 0.45F;
    float pathLength = 14.0F;
    float pathSampleSpacing = 0.008F;
    float branching = 0.70F;
    float coverage = 0.65F;
    float gapTolerance = 0.60F;
    bool attractorEnabled = false;
    invisible_places::io::Float3 attractorPosition{};
    float attractorStrength = 0.0F;
    std::uint32_t maxSteps = 2200;
    std::uint32_t supportSampleLimit = 450000;
};

struct WaterParticleTrailShapeSettings {
    float particleJitter = 0.35F;
    float splineAnchorSpacing = 0.5F;
    std::uint32_t trailLaneCount = 7;
    float trailLooseness = 0.45F;
    float trailSmoothness = 0.55F;
    float trailTurbulence = 0.45F;
    float trailMomentum = 0.60F;
    float normalTurbulenceResponse = 0.60F;
};

struct WaterAnimationTrailSettings {
    float particleDensity = 1.0F;
    float particleSpeed = 1.0F;
    float colorVariation = 0.65F;
    float trailLengthMeters = 0.75F;
    float trailSampleSpacingMeters = 0.0F;
};

struct WaterEffectResponseSettings {
    float intensity = 0.75F;
    float emissionAdd = 0.85F;
    float opacityAdd = 0.0F;
    float opacityMultiply = 1.0F;
    float pointSizeAdd = 0.0F;
    float pointSizeMultiply = 1.0F;
    float hueShift = 0.0F;
    float colouriseRed = 0.62F;
    float colouriseGreen = 0.88F;
    float colouriseBlue = 1.0F;
    float colouriseAmount = 0.35F;
    float gaussianSharpnessBias = 0.0F;
};

struct WaterSeepageLookSettings {
    WaterSeepageQuality quality = WaterSeepageQuality::Auto;
    WaterSeepagePattern pattern = WaterSeepagePattern::ChaoticBloom;
    float baseWetness = 0.35F;
    float density = 0.45F;
    float glisten = 0.55F;
    float rainResponse = 0.50F;
    float featureSizeMeters = 0.20F;
    float contrast = 0.55F;
    float evolution = 0.06F;
    float roughness = 0.45F;
    float angleResponse = 0.70F;
    float microNormalStrength = 0.18F;
    float glintDensity = 0.30F;
    float environmentAzimuthDegrees = 225.0F;
    float environmentElevationDegrees = 55.0F;
    float curl = 0.40F;
    float breakup = 0.45F;
    float downhillDriftMetersPerSecond = 0.025F;
    float tricklePatchSizeMeters = 0.08F;
    float trickleWidthMeters = 0.018F;
    float trickleFrontSoftness = 0.10F;
    float pulseSpacingMeters = 0.18F;
    float pulseWidthMeters = 0.055F;
    float pulseSpeedMetersPerSecond = 0.12F;
    float pulseIrregularity = 0.38F;
    float pulseWaveCount = 7.0F;
    float pulseSpeedVariation = 0.55F;
    WaterEffectResponseSettings response{
        .intensity = 0.85F,
        .emissionAdd = 0.35F,
        .opacityAdd = 0.04F,
        .opacityMultiply = 1.12F,
        .pointSizeAdd = 0.0F,
        .pointSizeMultiply = 1.08F,
        .hueShift = 0.0F,
        .colouriseRed = 0.28F,
        .colouriseGreen = 0.42F,
        .colouriseBlue = 0.46F,
        .colouriseAmount = 0.22F,
        .gaussianSharpnessBias = 0.0F,
    };
    WaterEffectBlendMode blendMode = WaterEffectBlendMode::Max;
};

// Named seepage settings profile (pattern, motion, reflection — everything
// except the visual response). The response half of the stored settings is
// ignored at resolve time; nodes pair a settings profile with an independent
// response profile so the effect and how much it changes the underlying
// cloud can be mixed and matched.
// Object-specific copies ("<base>_<node name>") carry their owning node id
// and base profile; any node may reference another node's copy by name, but
// only the owner's edits rewrite it.
struct WaterSeepageLookProfile {
    std::string name = "Default";
    WaterSeepageLookSettings settings{};
    bool objectOverride = false;
    std::uint32_t ownerObjectId = 0U;
    std::string baseProfileName;
};

// Named visual-response profile: how strongly seepage changes the underlying
// cloud (emission/opacity/size/colour response and blend), independent of
// which pattern produces the effect. Object-copy fields follow the look
// profile's convention.
struct WaterSeepageResponseProfile {
    std::string name = "Default";
    WaterEffectResponseSettings response{};
    WaterEffectBlendMode blendMode = WaterEffectBlendMode::Max;
    bool objectOverride = false;
    std::uint32_t ownerObjectId = 0U;
    std::string baseProfileName;
};

// Reusable physical/behavioural settings for a Seepage node. Placement,
// identity, viewport/export visibility, and the per-node variation seed stay
// on WaterSeepageNode; this profile owns the footprint, strength, delayed Rain
// response, and authored-role targeting shown in the Node Settings section.
struct WaterSeepageNodeSettings {
    float widthMeters = 0.10F;
    float prominence = 1.0F;
    float selectionReachLimitMeters = 2.34375F;
    float selectionWidthLimitMeters = 1.215F;
    float edgeFeatherMeters = 0.10F;
    float depthToleranceMeters = 0.15F;
    float normalAlignment = 0.20F;
    float strength = 1.0F;
    float rainDelaySeconds = 0.0F;
    float rainRiseSeconds = 0.0F;
    float rainRecessionSeconds = 0.0F;
    std::vector<std::string> targetSceneRoles{"ROCK", "VEG"};

    friend auto operator<=>(
        const WaterSeepageNodeSettings&,
        const WaterSeepageNodeSettings&) = default;
};

// Named Node Settings profile. Object-copy metadata follows the same
// convention as Seepage Look and Visual Response profiles.
struct WaterSeepageNodeSettingsProfile {
    std::string name = "Default";
    WaterSeepageNodeSettings settings{};
    bool objectOverride = false;
    std::uint32_t ownerObjectId = 0U;
    std::string baseProfileName;
};

// Pure description of the editable layer behind a Seepage Node Settings,
// Look, or Response assignment. Modern object-copy metadata is authoritative;
// the legacy `<base>_edited` convention remains readable for migration.
struct WaterObjectProfileIdentity {
    std::string name;
    std::uint32_t ownerObjectId = 0U;
    std::string baseProfileName;
};

struct WaterObjectProfileEditDescriptor {
    std::string assignedProfileName = "Default";
    // Exact resolvable base: `_preset` is deliberately retained for discard.
    std::string exactBaseProfileName = "Default";
    // Human-facing reusable save target: `_preset`/`_edited` are stripped.
    std::string suggestedSaveProfileName = "Default";
    // Empty for ordinary assignments and another object's copy.
    std::string removableWorkingProfileName;
    bool assignedObjectCopy = false;
    bool ownedObjectCopy = false;
    bool legacyEditedShadow = false;
};

// Modern owner/base metadata wins over the legacy `<base>_edited` spelling:
// an object can legitimately end in "edited", so its owned copy must survive
// project-library sanitization.
[[nodiscard]] bool WaterObjectProfileNameIsLegacyEditedShadow(
    std::string_view profileName,
    bool objectOverride);

[[nodiscard]] WaterObjectProfileEditDescriptor
DescribeWaterObjectProfileEdit(
    std::string_view assignedProfileName,
    std::uint32_t selectedOwnerObjectId,
    const std::optional<WaterObjectProfileIdentity>& assignedObjectCopy =
        std::nullopt);

enum class WaterObjectProfilePromotionOperation : std::uint8_t {
    Save = 0,
    SaveAs,
    Discard,
};

enum class WaterObjectProfileBaseKind : std::uint8_t {
    Default = 0,
    Shared,
    Protected,
    Missing,
    ObjectCopy,
};

enum class WaterObjectProfilePromotionFailure : std::uint8_t {
    None = 0,
    NotOwnedWorkingCopy,
    ProtectedBaseRequiresSaveAs,
    MissingBaseRequiresSaveAs,
    ObjectCopyBaseRequiresSaveAs,
    MissingDiscardBase,
    ObjectCopyDiscardBase,
};

struct WaterObjectProfilePromotionPlan {
    WaterObjectProfilePromotionOperation operation =
        WaterObjectProfilePromotionOperation::Save;
    WaterObjectProfilePromotionFailure failure =
        WaterObjectProfilePromotionFailure::None;
    std::string workingProfileName;
    std::string targetProfileName;
    bool overwriteExisting = false;
    bool createShared = false;
    bool eraseWorkingCopy = false;

    [[nodiscard]] bool allowed() const {
        return failure == WaterObjectProfilePromotionFailure::None;
    }
};

// Plans one owner-copy transaction without mutating the profile library.
// Save As never overwrites: requested/reserved collisions receive a stable
// numeric suffix. Save retains its allowed exact reusable base; Discard may
// retain a protected `_preset` base because it does not overwrite its values.
[[nodiscard]] WaterObjectProfilePromotionPlan
PlanWaterObjectProfilePromotion(
    const WaterObjectProfileEditDescriptor& descriptor,
    WaterObjectProfilePromotionOperation operation,
    WaterObjectProfileBaseKind baseKind,
    std::string_view requestedProfileName,
    std::span<const std::string> reservedProfileNames);

// Returns the preferred name when it is free, otherwise the first free
// numeric suffix. The code-owned `_preset` namespace is reserved even when
// the caller's stored library does not contain those regenerated profiles.
// The finite reservation set guarantees one of the next N+1 candidates is
// available, so callers never receive an unchecked fallback.
[[nodiscard]] std::string AllocateUniqueWaterObjectProfileName(
    std::string_view preferredProfileName,
    std::span<const std::string> reservedProfileNames);

struct WaterObjectProfilePromotionTransactionCallbacks {
    // Save and Save As write the snapshotted working values first. Discard
    // intentionally skips this callback.
    std::function<bool(std::string_view targetProfileName)> writeTarget;
    std::function<bool(
        std::string_view previousProfileName,
        std::string_view nextProfileName)> rewriteReferences;
    // The caller must erase only the exact owner-tagged working copy.
    std::function<bool(std::string_view workingProfileName)> eraseWorkingCopy;
};

// Executes the mutation ordering shared by Path/Lane/Trail promotion:
// target write, exact reference rewrite, then owner-copy erase. A rejected
// plan or missing required callback fails before mutation; a callback that
// returns false stops before the next phase. Completed callbacks are not
// rolled back, so callers preflight owner/target identity and keep these
// small in-memory callbacks effectively infallible.
[[nodiscard]] bool RunWaterObjectProfilePromotionTransaction(
    const WaterObjectProfilePromotionPlan& plan,
    const WaterObjectProfilePromotionTransactionCallbacks& callbacks);

struct WaterSeepageNode;

enum class WaterSeepageProfileHalf : std::uint8_t {
    NodeSettings = 0,
    Look,
    Response,
};

// Rebinds the persisted assignment on every matching node. Matching is
// trimmed but case-preserving so legacy `_Edited` names remain addressable
// until their promotion transaction completes.
[[nodiscard]] std::size_t ReplaceWaterSeepageNodeProfileReferences(
    std::span<WaterSeepageNode> nodes,
    WaterSeepageProfileHalf half,
    std::string_view previousProfileName,
    std::string_view nextProfileName);

struct WaterScenarioState {
    WaterSeepageLookSettings seepageLook{};
    float seepageLevel = 1.0F;
    float seepageSpread = 0.0F;
    float rainLevel = 0.0F;
    float flowLevel = 1.0F;
    // Shoreline waves are a point-style shader effect, so this level scales
    // the authored style non-destructively at render time. One preserves
    // projects written before schema 46 / animation schema 13.
    float shorelineLevel = 1.0F;
    // Mesh Flow has an independent dry baseline and Rain gain. Keeping the
    // legacy defaults at one/zero preserves projects written before schema 44.
    float meshFlowLevel = 1.0F;
    float meshFlowRainGain = 0.0F;
    float meshFlowPersistenceScale = 1.0F;
    float meshFlowRainRiseSeconds = 0.0F;
    float meshFlowRainRecessionSeconds = 0.0F;
    float seepageRainDelaySeconds = 0.0F;
    float seepageRainRiseSeconds = 0.0F;
    float seepageRainRecessionSeconds = 0.0F;
    std::optional<WaterSeepageLookSettings> transitionLook;
    float transitionAmount = 0.0F;
};

struct WaterScenarioDefinition {
    std::string id;
    std::string name;
    WaterScenarioState state{};
};

struct WaterScenarioKey {
    std::string id;
    float position = 0.0F;
    WaterScenarioState state{};
    WaterScenarioInterpolation interpolation = WaterScenarioInterpolation::Smooth;
};

struct WaterSeepageNodeAnimationState {
    float activity = 1.0F;
    float localSpread = 0.0F;
    float wettingProgress = 1.0F;
    // Live node dimensions and prominence are normalized multipliers. They
    // are deliberately parameters rather than topology inputs so playback
    // never rebuilds cache-derived support.
    float reachScale = 1.0F;
    float widthScale = 1.0F;
    float prominence = 1.0F;
    // Timings v2 keyed absolute values. Negative means "not keyed": the
    // authored value times the multiplier lanes above applies. When set,
    // the keyed value replaces the authored base but the scenario level,
    // activity, and spread/rain gains still scale on top — all parameter
    // lanes, never topology.
    float strengthOverride = -1.0F;
    float prominenceOverride = -1.0F;
    float sourceWidthOverride = -1.0F;
    // Resolved per-node Rain envelope value. Negative keeps the legacy
    // fallback (scenario Rain when present, otherwise the authored Rain
    // control). This is a transient frame value, never an authored setting.
    float rainLevelOverride = -1.0F;
    // Complete transient look produced by active scalar profile tracks. The
    // frame resolver starts from the otherwise active scenario/profile look,
    // applies keyed values, and never writes the result into the saved base.
    std::optional<WaterSeepageLookSettings> lookOverride;
};

struct WaterSeepageNodeKey {
    std::string id;
    float position = 0.0F;
    WaterSeepageNodeAnimationState state{};
    WaterScenarioInterpolation interpolation = WaterScenarioInterpolation::Smooth;
};

struct WaterSeepageNodeTrack {
    std::uint32_t nodeId = 0U;
    std::vector<WaterSeepageNodeKey> keys;
};

struct WaterSeepageNodeAnimationStateEntry {
    std::uint32_t nodeId = 0U;
    WaterSeepageNodeAnimationState state{};
};

// Water timing runs are reusable per-feature key sequences authored in the
// Timings panel. Positions use the active normalized timing domain: one path
// for an ordinary animation, or the unique cyclic duration for a reciprocal
// pair. Extension workflows remap existing positions so they retain their
// intended camera frames. Applied legacy runs compile into the owning track's
// complete-snapshot keys; runs are never evaluated per frame.
enum class WaterTimingFeature {
    Shoreline,
    Seepage,
    Rain,
    Flow,
    MeshFlow
};

struct WaterTimingKey {
    std::string id;
    float position = 0.0F;
    float level = 1.0F;
    WaterScenarioInterpolation interpolation = WaterScenarioInterpolation::Smooth;
};

struct WaterTimingRun {
    std::string id;
    std::string name;
    WaterTimingFeature feature = WaterTimingFeature::Rain;
    std::vector<WaterTimingKey> keys;
};

struct WaterTimingRunAssignment {
    WaterTimingFeature feature = WaterTimingFeature::Rain;
    std::string runId;
    std::string runName;
    // Embedded snapshot so a track stays reproducible when its project run
    // library entry is missing, mirroring fallbackScenario.
    WaterTimingRun fallbackRun{};
};

struct WaterScenarioTrack {
    std::string scenarioId;
    std::string scenarioName;
    WaterScenarioDefinition fallbackScenario{};
    std::vector<WaterScenarioKey> keys;
    std::vector<WaterSeepageNodeTrack> seepageNodeTracks;
    std::vector<WaterTimingRunAssignment> timingAssignments;
};

// ---- Timings v2: per-feature, per-setting keyframing ----
// A run groups scene water features whose individual settings are keyed in
// the active Timing Take. Positions are normalized 0..1 in its path-local or
// shared cyclic domain; camera-tail extensions remap them to retain existing
// camera frames. Runs are scenario-scoped and evaluated directly per frame;
// within one scenario a feature belongs to at most one run. Key
// display names are derived ("<run name> <n>" in time order), never stored,
// so inserting a key between two others renumbers the later ones for free.
enum class WaterKeyedFeatureKind : std::uint8_t {
    Rain = 0,
    MeshFlow,
    Shoreline,
    SeepageGlobal,
    FlowGlobal,
    SeepageNode,
    FlowSource,
    FlowPath,
    // Additional shoreline instance (objectId = instance id). The primary
    // style shoreline stays the global Shoreline kind above.
    ShorelineInstance,
};

struct WaterKeyedFeatureId {
    WaterKeyedFeatureKind kind = WaterKeyedFeatureKind::Rain;
    // Seepage node / flow emitter / manual path id; zero for scene globals.
    std::uint32_t objectId = 0U;

    friend auto operator<=>(
        const WaterKeyedFeatureId&,
        const WaterKeyedFeatureId&) = default;
};

struct WaterSettingKey {
    float position = 0.0F;
    float value = 0.0F;
    WaterScenarioInterpolation interpolation =
        WaterScenarioInterpolation::TrackDefault;
    // Explicit settings-clip ownership. Zero means the key is loose on the
    // feature timeline. Keeping ownership on the key (rather than inferring
    // it from a time window) lets clips overlap without stealing each
    // other's keys.
    std::uint32_t clipId = 0U;
    // Handle times are fractions of the adjacent segment measured away from
    // this key. Handle values are offsets in the setting's authored units.
    // The one-third/zero defaults reproduce Smooth Step exactly.
    float incomingHandleTime = 1.0F / 3.0F;
    float incomingHandleValue = 0.0F;
    float outgoingHandleTime = 1.0F / 3.0F;
    float outgoingHandleValue = 0.0F;
};

enum class WaterSettingSplineHandleSide : std::uint8_t {
    Incoming = 0,
    Outgoing,
};

struct WaterSettingSplineHandlePoint {
    float anchorPosition = 0.0F;
    float anchorValue = 0.0F;
    float controlPosition = 0.0F;
    float controlValue = 0.0F;
};

struct WaterKeyedSettingTrack {
    std::string settingId;
    // Dormant tracks retain their keys in the scenario but do not evaluate,
    // draw markers, or participate in navigation until enabled again.
    bool active = true;
    // Dynamic profile fields use stored display metadata instead of growing
    // the fixed feature-setting registry for every scalar profile member.
    std::string label;
    std::string profileGroup;
    std::string profileName;
    // Curve style applied to every key set to TrackDefault. New tracks use a
    // monotone spline: speed carries through monotonic runs without scalar
    // overshoot and comes to rest at reversals. Pre-existing tracks retain
    // their serialized style, and legacy tracks migrate to Smooth so their
    // saved motion remains unchanged until deliberately restyled.
    WaterScenarioInterpolation defaultInterpolation =
        WaterScenarioInterpolation::SmoothVelocity;
    std::vector<WaterSettingKey> keys;
};

// Project-owned reusable key tracks. Saved profiles are immutable templates;
// editing an applied profile creates an object-owned `_edited` shadow whose
// sourceProfileName points back to the saved template for discard/restore.
// Keys stay normalized to their reusable timing domain (0..1), so a profile
// can be applied to another Flow Path Source without depending on frame count.
// Captured clip packages additionally record the normalized length of the
// span they came from, so applying one reproduces its authored duration by
// default while remaining freely stretchable afterwards.
struct WaterKeyedSettingsProfile {
    std::string name;
    std::string baseProfileName = "Default";
    std::string ownerObjectName;
    std::string sourceProfileName;
    std::uint32_t ownerObjectId = 0U;
    WaterKeyedFeatureKind featureKind = WaterKeyedFeatureKind::FlowPath;
    bool edited = false;
    float nativeLengthFraction = 1.0F;
    std::vector<WaterKeyedSettingTrack> settings;
};

// The shortest span a settings clip may occupy, in normalized timing
// positions. It stays well above the 1e-4 key identity tolerance so a fully
// shrunk clip still keeps its keys distinct.
inline constexpr float kWaterFeatureClipMinimumLength = 1.0e-3F;

// Shared key-identity tolerance for clip span maths (membership, collisions,
// seam comparisons).
inline constexpr float kWaterFeatureClipPositionTolerance = 1.0e-4F;

// ---- Wrapped clip contract (W1) ----
// A clip span is `start` in [0,1) and `end` in (start, start+1]. `end > 1`
// means the clip wraps through loop phase 0: it occupies [start,1] and
// [0,end-1]. Keys are always stored in [0,1]; a key p belongs to a wrapped
// span when p or p+1 lies in [start,end]. `end - start` is the clip length
// everywhere, so an unwrapped clip is exactly the pre-W1 0<=start<=end<=1
// shape and its maths are unchanged. Wrapping is only ever produced by
// linked-cyclic authoring gestures (callers pass allowWrap); unlinked
// editing keeps clamping to 0..1.

// Wraps any phase into [0,1). Values within the shared tolerance below 1
// wrap to 0 so seam float noise cannot store a key at 0.99999 instead of 0.
[[nodiscard]] inline float WrapWaterClipPhase(float value) {
    if (!std::isfinite(value)) {
        return 0.0F;
    }
    float wrapped = value - std::floor(value);
    if (wrapped >= 1.0F - kWaterFeatureClipPositionTolerance) {
        wrapped = 0.0F;
    }
    return wrapped;
}

[[nodiscard]] inline bool WaterClipIsWrapped(float start, float end) {
    (void)start;
    return end > 1.0F + kWaterFeatureClipPositionTolerance;
}

[[nodiscard]] inline float WaterClipLength(float start, float end) {
    return end - start;
}

// Stored phase of a clip's end: the wrapped end lands back in (0,1].
[[nodiscard]] inline float WaterClipCanonicalEnd(float start, float end) {
    return WaterClipIsWrapped(start, end) ? end - 1.0F : end;
}

// Unwraps a stored key position onto the clip's own [start, start+1]
// coordinate line: keys ahead of the wrap seam gain one cycle.
[[nodiscard]] inline float UnwrapWaterClipPosition(
    float position,
    float start) {
    return position < start - kWaterFeatureClipPositionTolerance
               ? position + 1.0F
               : position;
}

// Membership: p inside [start-tol, end+tol], or p+1 inside it when the span
// wraps. An unwrapped span keeps the pre-W1 linear test on purpose: on the
// 0..1 rail a stored key at 0 is not a member of a clip ending at 1 (they are
// distinct times when unlinked), and only a span that actually crosses phase
// 0 identifies the two phases.
[[nodiscard]] inline bool WaterClipContainsPosition(
    float start,
    float end,
    float position) {
    constexpr float kTolerance = kWaterFeatureClipPositionTolerance;
    const auto inside = [&](float candidate) {
        return candidate >= start - kTolerance &&
               candidate <= end + kTolerance;
    };
    return inside(position) ||
           (WaterClipIsWrapped(start, end) && inside(position + 1.0F));
}

// Normalizes a derived display span to the same minimum width as a stored
// clip. A wrapped clip span is kept (end up to start+1); a plain loose-key
// span never wraps. The clip lane allocator and UI share this so a loose-key
// ghost can never overlap a bar in pixels while being considered disjoint by
// lane assignment.
[[nodiscard]] std::pair<float, float> WaterFeatureClipDisplaySpan(
    float start,
    float end);

// A named group on one feature's keyed-setting timeline. Clips are authoring
// metadata only: the keys stay the single evaluated source of truth, and
// carry this clip's id explicitly so overlapping clip windows remain
// unambiguous. A keyed clip's first and last key define its displayed bounds;
// an empty clip retains its authored window. Positions are normalized 0..1
// in the active path-local or shared-loop timing domain, exactly like the keys
// they contain.
struct WaterFeatureSettingsClip {
    // Unique within its timeline; zero is never a valid stored id (the UI
    // uses it for the derived loose-keys block).
    std::uint32_t id = 0U;
    std::string name = "Clip";
    // W1 span: start in [0,1), end in (start, start+1]; end > 1 wraps
    // through phase 0 (see the wrapped clip contract above).
    float start = 0.0F;
    float end = 1.0F;
    // Package provenance (a WaterKeyedSettingsProfile name); metadata only.
    std::string sourceProfileName;
};

struct WaterFeatureTimeline {
    WaterKeyedFeatureId feature{};
    std::vector<WaterKeyedSettingTrack> settings;
    std::vector<WaterFeatureSettingsClip> clips;
    // Load/runtime migration marker. Older documents inferred membership
    // from non-overlapping clip spans; current documents serialize clip_id
    // on every timeline key, including zero for deliberately loose keys.
    bool clipMembershipExplicit = false;
};

// A short authored event note shared by every feature assigned to one run.
// The id is stable within that run; position uses the same normalized timing
// domain as the run's keys and clips.
struct WaterFeatureRunMark {
    std::uint32_t id = 0U;
    std::string text = "Mark";
    float position = 0.0F;
};

struct WaterFeatureTimingRun {
    std::uint32_t id = 0U;
    std::string name = "Run";
    // Unchecked runs keep their feature timelines and keys but stop driving
    // the water features: the overlay, rain envelopes, and keyed sliders all
    // treat their features as unkeyed until the run is enabled again.
    bool enabled = true;
    std::vector<WaterFeatureTimeline> features;
    std::vector<WaterFeatureRunMark> marks;
};

struct WaterScenarioFeatureRuns {
    std::string scenarioId;
    std::vector<WaterFeatureTimingRun> runs;
};

// Registry of the settings each feature kind can key. The id is the stable
// serialization identity; label and range drive the keyed sliders. Global
// kinds key one "level" that overrides the matching scenario channel.
struct WaterKeyableSettingInfo {
    const char* id = "";
    const char* label = "";
    float minimum = 0.0F;
    float maximum = 1.0F;
    float defaultValue = 1.0F;
    // Large parameter surfaces such as Rain stay discoverable beside their
    // authored controls without filling an empty timeline with dozens of
    // guides. Once keyed, these settings appear in the graph normally.
    bool showUnauthoredInTimeline = true;
};

[[nodiscard]] std::span<const WaterKeyableSettingInfo> WaterKeyableSettings(
    WaterKeyedFeatureKind kind);
[[nodiscard]] const WaterKeyableSettingInfo* FindWaterKeyableSetting(
    WaterKeyedFeatureKind kind,
    std::string_view settingId);
// Canonical presentation order for one keyable setting. Registry settings
// keep their fixed registry ordinal even when an unauthored setting is hidden
// from a timeline. Dynamic profile tracks follow every registry setting in
// lexicographic id order, independent of their storage/merge order. Returning
// one shared index lets Water controls, graphs, markers, pins, and clips use a
// stable colour identity without depending on an ImGui palette here.
[[nodiscard]] std::optional<std::size_t> WaterKeyedSettingDisplayIndex(
    WaterKeyedFeatureKind kind,
    std::string_view settingId,
    std::span<const WaterKeyedSettingTrack> timelineSettings = {});
// Lowest canonical setting index among every key explicitly owned by clipId.
// Inactive/dormant tracks still participate so toggling a track cannot recolour
// its clip. clipId zero resolves the derived loose-keys block; an empty clip or
// loose timeline has no setting identity.
[[nodiscard]] std::optional<std::size_t>
WaterFeatureClipPrimarySettingDisplayIndex(
    const WaterFeatureTimeline& timeline,
    std::uint32_t clipId);
// Exact, presentation-stable setting ownership for one stored clip, or for
// the derived loose-keys block when clipId is zero. Every authored key
// participates, including keys on dormant tracks. Empty stored clips retain
// an empty signature so they receive their own semantic lane band.
struct WaterFeatureClipSettingSignature {
    std::vector<std::string> settingIds;
    std::optional<std::size_t> minimumDisplayIndex;

    friend bool operator==(
        const WaterFeatureClipSettingSignature&,
        const WaterFeatureClipSettingSignature&) = default;
};

[[nodiscard]] WaterFeatureClipSettingSignature
WaterFeatureClipSettingSignatureForId(
    const WaterFeatureTimeline& timeline,
    std::uint32_t clipId);

// Deterministic semantic lane allocation for a feature timeline. Signatures
// form stable base bands ordered by canonical setting identity; overlapping
// or exactly touching spans of the same signature greedily spill into the
// lowest available sublane. The result depends on authored spans and ids,
// never clip-vector order or the current timeline zoom.
struct WaterFeatureClipLaneAssignment {
    std::uint32_t clipId = 0U;
    std::size_t signatureBandIndex = 0U;
    std::size_t spillLaneIndex = 0U;
    std::size_t laneIndex = 0U;

    friend bool operator==(
        const WaterFeatureClipLaneAssignment&,
        const WaterFeatureClipLaneAssignment&) = default;
};

struct WaterFeatureClipLaneLayout {
    std::vector<WaterFeatureClipLaneAssignment> assignments;
    std::size_t signatureBandCount = 0U;
    std::size_t laneCount = 0U;

    friend bool operator==(
        const WaterFeatureClipLaneLayout&,
        const WaterFeatureClipLaneLayout&) = default;
};

// cyclicLooseKeys derives the loose-key ghost span cyclically (the linked
// whole-loop lens); stored clip spans always carry their own wrapping.
[[nodiscard]] WaterFeatureClipLaneLayout
BuildWaterFeatureClipLaneLayout(
    const WaterFeatureTimeline& timeline,
    bool cyclicLooseKeys = false);

// Concise body-hover label for a clip. Conventional package names such as
// "Object - Start - Strength" become "Start - Strength"; otherwise only an
// exact focused-object prefix followed by a separator is removed.
[[nodiscard]] std::string WaterFeatureClipConciseDisplayName(
    std::string_view clipName,
    std::string_view focusedObjectName);
// Resolves the authored scalar behind every entry in Rain's fixed keyable
// setting registry. UI/profile comparisons use this rather than maintaining a
// second member map that can silently drift when a Rain setting is added.
[[nodiscard]] std::optional<float> ResolveWaterRainSettingValue(
    const RainRuntimeSettings& settings,
    const WaterRainVisualSettings& visual,
    std::string_view settingId);
[[nodiscard]] std::string_view WaterKeyedFeatureKindLabel(
    WaterKeyedFeatureKind kind);
[[nodiscard]] std::string_view WaterKeyedFeatureKindName(
    WaterKeyedFeatureKind kind);
[[nodiscard]] std::optional<WaterKeyedFeatureKind>
ParseWaterKeyedFeatureKindName(std::string_view name);
[[nodiscard]] bool WaterKeyedFeatureKindIsGlobal(WaterKeyedFeatureKind kind);

[[nodiscard]] WaterKeyedSettingTrack SanitizeWaterKeyedSettingTrack(
    WaterKeyedSettingTrack track);
// Profile equality includes curve defaults and manual handle geometry, but
// deliberately ignores clip ownership because whole-timeline profiles do not
// carry settings-clip grouping.
[[nodiscard]] bool WaterKeyedSettingTrackProfileEqual(
    const WaterKeyedSettingTrack& left,
    const WaterKeyedSettingTrack& right);
// Rebinds keyed tracks carrying explicit profile-group metadata. Use the
// feature-aware overload below for persisted stores that may still contain
// blank-group Seepage or manual Flow Path tracks.
[[nodiscard]] std::size_t ReplaceWaterKeyedSettingProfileReferences(
    std::span<WaterKeyedSettingTrack> settings,
    std::string_view profileGroup,
    std::string_view previousProfileName,
    std::string_view nextProfileName);
// Feature-aware overload used for whole-project transactions. Explicit
// profile metadata remains authoritative; legacy blank-group tracks are
// classified only inside their matching feature kind so overlapping ids such
// as `strength` cannot capture an unrelated Water feature.
[[nodiscard]] std::size_t ReplaceWaterKeyedSettingProfileReferences(
    std::span<WaterKeyedSettingTrack> settings,
    WaterKeyedFeatureKind featureKind,
    std::string_view profileGroup,
    std::string_view previousProfileName,
    std::string_view nextProfileName);
// Rebinds the saved-base mirror carried by reusable keyed settings profiles.
// Package identity and provenance fields are deliberately left untouched.
[[nodiscard]] std::size_t ReplaceWaterKeyedSettingsProfileBaseReferences(
    std::span<WaterKeyedSettingsProfile> profiles,
    WaterKeyedFeatureKind featureKind,
    std::string_view previousProfileName,
    std::string_view nextProfileName);
// Rebinds every matching profile-backed track on the requested feature
// timelines. Dormant tracks are intentionally included; legacy blank-group
// Seepage and manual Flow Path tracks are classified by stable setting ids.
[[nodiscard]] std::size_t CanonicalizeWaterFeatureProfileMetadata(
    std::span<WaterFeatureTimingRun> runs,
    std::span<const WaterKeyedFeatureId> features,
    std::string_view profileGroup,
    std::string_view profileName);
[[nodiscard]] WaterKeyedSettingsProfile SanitizeWaterKeyedSettingsProfile(
    WaterKeyedSettingsProfile profile);
// Package/profile names are scoped to their feature kind. This lets a Seepage
// package and a Flow Path package share a user-facing name without replacing
// one another in the project-owned library.
[[nodiscard]] std::optional<std::size_t>
FindWaterKeyedSettingsProfileIndex(
    std::span<const WaterKeyedSettingsProfile> profiles,
    WaterKeyedFeatureKind featureKind,
    std::string_view name);
// Sanitizes the project-owned keyed-settings library without narrowing it to
// one feature kind. Clip packages are shared by Rain, Flow, Shoreline, and
// Seepage authoring, while full-length Flow Path profiles use the same store.
[[nodiscard]] std::vector<WaterKeyedSettingsProfile>
SanitizeWaterKeyedSettingsProfileLibrary(
    std::vector<WaterKeyedSettingsProfile> profiles);
[[nodiscard]] std::string WaterKeyedSettingsProfileSavedName(
    std::string_view baseProfileName,
    std::string_view objectName);
[[nodiscard]] std::string WaterKeyedSettingsProfileEditedName(
    std::string_view baseProfileName,
    std::string_view objectName);
[[nodiscard]] WaterFeatureTimingRun SanitizeWaterFeatureTimingRun(
    WaterFeatureTimingRun run);
// Mark ids are unique within a run. Default names are unique across the
// supplied scene's runs and use the user-facing Mark 00, Mark 01 sequence.
[[nodiscard]] std::uint32_t AllocateWaterFeatureRunMarkId(
    const WaterFeatureTimingRun& run);
[[nodiscard]] std::string AllocateWaterFeatureRunMarkName(
    std::span<const WaterFeatureTimingRun> runs);
[[nodiscard]] WaterFeatureRunMark* FindWaterFeatureRunMark(
    WaterFeatureTimingRun* run,
    std::uint32_t markId);
[[nodiscard]] const WaterFeatureRunMark* FindWaterFeatureRunMark(
    const WaterFeatureTimingRun* run,
    std::uint32_t markId);
[[nodiscard]] bool MoveWaterFeatureRunMark(
    WaterFeatureTimingRun* run,
    std::uint32_t markId,
    float position);
[[nodiscard]] bool RenameWaterFeatureRunMark(
    WaterFeatureTimingRun* run,
    std::uint32_t markId,
    std::string_view text);
[[nodiscard]] bool RemoveWaterFeatureRunMark(
    WaterFeatureTimingRun* run,
    std::uint32_t markId);
// Moves an existing feature timeline (including dormant tracks and keys) to
// the target run, or adds a new empty timeline when it is not assigned.
[[nodiscard]] bool AssignWaterFeatureToTimingRun(
    WaterScenarioFeatureRuns* entry,
    const WaterKeyedFeatureId& feature,
    std::uint32_t targetRunId);
// Removes every timeline for one deleted scene feature while retaining the
// authored runs themselves (including runs that become empty). Returns the
// number of timelines removed, which also exposes malformed duplicates.
[[nodiscard]] std::size_t RemoveWaterFeatureFromTimingRuns(
    std::span<WaterFeatureTimingRun> runs,
    const WaterKeyedFeatureId& feature);
// Endpoint-hold sampling across the concrete per-segment modes, including
// automatic and manually handled splines. An exact interior key belongs to
// the segment it starts, so a Hold step reads its new value at the key
// position itself. Empty tracks return nullopt.
[[nodiscard]] std::optional<float> EvaluateWaterKeyedSettingTrack(
    const WaterKeyedSettingTrack& track,
    float normalizedPosition);
// Whole-loop sampling. The last authored key connects directly to the first;
// virtual neighbouring copies participate in spline tangents, so no endpoint
// key is inserted and Smooth/SmoothVelocity behaviour is preserved.
[[nodiscard]] std::optional<float> EvaluateWaterKeyedSettingTrackCyclic(
    const WaterKeyedSettingTrack& track,
    float normalizedLoopPosition);
// Inserts, or replaces any key within 1e-4 of the position. Passing clipId
// explicitly attaches (or reattaches) the key; omitting it preserves an
// existing key's membership and creates a new loose key.
void AddOrUpdateWaterSettingKey(
    WaterKeyedSettingTrack* track,
    float position,
    float value,
    WaterScenarioInterpolation interpolation =
        WaterScenarioInterpolation::TrackDefault,
    std::optional<std::uint32_t> clipId = std::nullopt);
// Timeline-aware UI authoring: one selected clip owns genuinely new keys,
// while editing a key already at this time preserves its current clip.
void AddOrUpdateWaterTimelineSettingKey(
    WaterFeatureTimeline* timeline,
    WaterKeyedSettingTrack* track,
    float position,
    float value,
    WaterScenarioInterpolation interpolation =
        WaterScenarioInterpolation::TrackDefault,
    std::optional<std::uint32_t> selectedClipId = std::nullopt);
// Moves one key without overwriting another key in the same setting track.
// Source matching and destination occupancy use the same 1e-4 tolerance as
// insertion and navigation.
[[nodiscard]] bool MoveWaterSettingKey(
    WaterKeyedSettingTrack* track,
    float sourcePosition,
    float destinationPosition);
// Resolves or moves one manual Bezier control point. Incoming handles are
// active when the preceding segment uses Spline Handles; outgoing handles are
// active when the selected key's segment does. Absolute graph coordinates are
// converted to segment-relative storage so ordinary key/clip retiming carries
// the curve shape with it.
[[nodiscard]] std::optional<WaterSettingSplineHandlePoint>
ResolveWaterSettingSplineHandlePoint(
    const WaterKeyedSettingTrack& track,
    float keyPosition,
    WaterSettingSplineHandleSide side,
    bool cyclic = false);
[[nodiscard]] bool MoveWaterSettingSplineHandlePoint(
    WaterKeyedSettingTrack* track,
    float keyPosition,
    WaterSettingSplineHandleSide side,
    float controlPosition,
    float controlValue,
    bool cyclic = false);
[[nodiscard]] std::size_t WaterSettingKeyCountAtPosition(
    const WaterKeyedSettingTrack& track,
    float position);
[[nodiscard]] std::size_t RemoveWaterSettingKeysAtPosition(
    WaterKeyedSettingTrack* track,
    float position);
[[nodiscard]] std::size_t WaterFeatureKeyCountAtPosition(
    const WaterFeatureTimeline& timeline,
    float position);
[[nodiscard]] std::size_t RemoveWaterFeatureKeysAtPosition(
    WaterFeatureTimeline* timeline,
    float position);
[[nodiscard]] std::optional<float> PreviousWaterSettingKeyPosition(
    const WaterKeyedSettingTrack& track,
    float position);
[[nodiscard]] std::optional<float> NextWaterSettingKeyPosition(
    const WaterKeyedSettingTrack& track,
    float position);
[[nodiscard]] std::optional<float> PreviousWaterFeatureKeyPosition(
    const WaterFeatureTimeline& timeline,
    float position);
[[nodiscard]] std::optional<float> NextWaterFeatureKeyPosition(
    const WaterFeatureTimeline& timeline,
    float position);
// Ordered union of active keys in one profile group. UI names are derived
// from this order, so insertion and moves renumber <profile>_Run01 states
// without rewriting the saved key data.
[[nodiscard]] std::vector<float> WaterFeatureProfileKeyPositions(
    const WaterFeatureTimeline& timeline,
    std::string_view profileGroup);

// ---- Settings clips: grouped manipulation of a timeline span ----
// Every operation below moves or copies raw keys (including dormant tracks,
// so re-enabling a track stays aligned with its clip); evaluation and
// serialization of the keys themselves are untouched.

[[nodiscard]] WaterFeatureSettingsClip SanitizeWaterFeatureSettingsClip(
    WaterFeatureSettingsClip clip);
[[nodiscard]] std::uint32_t AllocateWaterFeatureClipId(
    const WaterFeatureTimeline& timeline);
[[nodiscard]] WaterFeatureSettingsClip* FindWaterFeatureClip(
    WaterFeatureTimeline* timeline,
    std::uint32_t clipId);
[[nodiscard]] const WaterFeatureSettingsClip* FindWaterFeatureClip(
    const WaterFeatureTimeline* timeline,
    std::uint32_t clipId);
// Span of every explicitly loose key on the timeline (active and dormant
// tracks), or nullopt when there are none. Backs the derived loose-keys block
// shown alongside stored clips. The default is the pre-W1 linear min/max on
// the 0..1 rail; with cyclic (the linked whole-loop lens) it is the shortest
// covering arc -- the complement of the largest circular gap between the
// keys, possibly wrapped (end > 1) -- so a cluster pushed across phase 0
// stays one block instead of flipping to the complement-length rail hull.
// The linear hull is the complement of the seam gap and remains the
// tie-breaking choice, so the two lenses agree wherever both are valid.
[[nodiscard]] std::optional<std::pair<float, float>>
WaterFeatureLooseKeySpan(
    const WaterFeatureTimeline& timeline,
    bool cyclic = false);

// Recomputes one stored clip's bounds from its owned keys. An unwrapped clip
// takes the exact pre-W1 linear min/max of its members (so unlinked key
// drags, reorders, and deletes never make it wrap); a wrapped clip takes the
// hull of its members unwrapped relative to the member cyclically nearest
// its current start (keys ahead of that anchor gain a cycle), so it stays
// wrapped while either end key is nudged and unwraps once every member sits
// on one side of phase 0. A clip that unwrapped onto [start, 1] by a member
// reaching phase 0 keeps reading that member as 1 so the bounds are a fixed
// point of this call (and survive save/load). A single time keeps the
// minimum manipulable width around that key: a clip that was wrapped may
// straddle phase 0, an unwrapped one keeps the pre-W1 0..1 clamped marker.
// Empty clips retain their authored span.
// Known limitation: a key gesture can never make an unwrapped clip wrap.
// Dragging the head key of {0.02,0.3} backwards across phase 0 to 0.98 in
// the linked view yields the linear hull {0.3,0.98} (pre-W1 behaviour);
// only clip drags (TransformWaterFeatureClip with allowWrap) cross the seam.
[[nodiscard]] bool SynchronizeWaterFeatureClipBounds(
    WaterFeatureTimeline* timeline,
    std::uint32_t clipId);

// Interval the span [rangeStart, rangeEnd] may occupy without leaving 0..1.
// Other clips do not constrain it: overlapping clip windows are supported.
// For linked-cyclic editing (cyclic = true) the rail has no ends, so the
// limits are unbounded and only the end - start <= 1 rule applies.
struct WaterFeatureSpanLimits {
    float minimumStart = 0.0F;
    float maximumEnd = 1.0F;
};
[[nodiscard]] WaterFeatureSpanLimits WaterFeatureTimelineSpanLimits(
    const WaterFeatureTimeline& timeline,
    float rangeStart,
    float rangeEnd,
    bool cyclic = false);

// Affinely remaps every key inside [rangeStart, rangeEnd] (inclusive, with
// the shared 1e-4 tolerance) onto [newStart, newEnd], on every track. Clips
// contained by the range remap with their keys. Returns false without
// mutation for invalid ranges, a destination outside 0..1, a destination
// shorter than the clip minimum while the range holds keys or clips, or a
// remapped key colliding with a key outside the range on the same track.
[[nodiscard]] bool TransformWaterFeatureTimelineSpan(
    WaterFeatureTimeline* timeline,
    float rangeStart,
    float rangeEnd,
    float newStart,
    float newEnd);

// Affinely transforms only the explicitly selected clip ids. Zero selects
// loose keys inside the source range. This is the overlap-safe primitive
// used by single and grouped UI drags. The source range may be a wrapped
// clip span (end > 1). With allowWrap the destination may be any span of
// length <= 1 expressed in unwrapped coordinates (newStart is normalized
// into [0,1) and keys wrap around phase 0), and same-track keys of one
// selected clip (or two selected loose keys) that share one loop phase
// (stored 0 and 1) are merged onto the linear-later one before the move,
// mirroring cyclic evaluation. Keys of two different selected clips are
// never merged: a group landing that would collide them is refused. A
// destination equal to the source span on the loop is the identity and
// succeeds without touching the document. Without allowWrap the
// destination must lie inside 0..1 exactly as before, so unlinked editing
// never wraps or merges.
[[nodiscard]] bool TransformWaterFeatureClipSelection(
    WaterFeatureTimeline* timeline,
    std::span<const std::uint32_t> clipIds,
    float rangeStart,
    float rangeEnd,
    float newStart,
    float newEnd,
    bool allowWrap = false);

// Translates one clip/group window on the canonical cyclic 0..1 timeline
// with period 1: the start rolls around phase 0 and the returned span is
// {start in [0,1), start + length}, so a span crossing the loop origin is
// returned wrapped (end > 1) rather than jumping to the other side. Any
// length rotates here, including a full-length span; the stored clip then
// re-derives its bounds from its moved keys (except that a full-length
// wrapped clip with a member on its boundary phase keeps its span, see
// SynchronizeWaterFeatureClipBounds). A clip keyed at both 0 and 1
// holds two stored keys on a single loop phase; TransformWaterFeatureClip-
// Selection (with allowWrap) coalesces such a pair onto the linear-later
// key, exactly as cyclic evaluation already reads it, so the clip rotates
// instead of being refused. Exact boundary placements are stable until the
// pointer actually crosses them.
[[nodiscard]] std::pair<float, float> CyclicWaterFeatureClipMoveSpan(
    float rangeStart,
    float rangeEnd,
    float delta);

// Convenience form for one stored clip.
[[nodiscard]] bool TransformWaterFeatureClip(
    WaterFeatureTimeline* timeline,
    std::uint32_t clipId,
    float newStart,
    float newEnd,
    bool allowWrap = false);

// Captures the keys inside [rangeStart, rangeEnd] as a reusable package
// whose key positions are renormalized to 0..1 across the span. Tracks with
// no keys in range are omitted; dormant tracks keep active = false. The
// package records the span length as nativeLengthFraction. Identity fields
// (name, owner, kind) are left for the caller to fill.
[[nodiscard]] WaterKeyedSettingsProfile CaptureWaterKeyedSettingsClip(
    const WaterFeatureTimeline& timeline,
    float rangeStart,
    float rangeEnd);

// Captures only keys explicitly owned by clipId, irrespective of overlapping
// windows. The package is normalized across the clip's current bounds.
[[nodiscard]] WaterKeyedSettingsProfile CaptureWaterKeyedSettingsClipById(
    const WaterFeatureTimeline& timeline,
    std::uint32_t clipId);

// Applies a package into [windowStart, windowEnd] as a new explicitly owned
// clip; the window may wrap (windowEnd up to windowStart + 1), in which case
// the stored keys wrap into [0,1). Existing clips and loose keys survive,
// including inside the target window. Same-track/time collisions are nudged minimally within the window.
// Missing tracks are created with the package's display metadata. When the
// package kind differs from the timeline's feature kind only registry
// settings of the target kind apply. Returns the new clip id, or nullopt
// without mutation for invalid arguments or when any applicable package key
// cannot be placed collision-free. Package application is transactional: it
// never leaves a partial or empty clip behind.
[[nodiscard]] std::optional<std::uint32_t> ApplyWaterKeyedSettingsClip(
    WaterFeatureTimeline* timeline,
    const WaterKeyedSettingsProfile& profile,
    float windowStart,
    float windowEnd,
    std::string clipName = {});

// Creates a clip over a span and attaches the loose keys inside it. Existing
// clips, including overlapping ones, are untouched. Returns the new clip id.
[[nodiscard]] std::optional<std::uint32_t> CreateWaterFeatureClipFromSpan(
    WaterFeatureTimeline* timeline,
    float start,
    float end,
    std::string name,
    bool attachLooseKeys = true);

// Copies one clip (bounds and explicitly owned keys) onto the same timeline
// with its start at targetStart. Overlapping destinations are supported.
// Without allowWrap the start is clamped so the copy stays inside 0..1;
// with it (linked-cyclic authoring) the start wraps with period 1 and the
// copy may wrap through phase 0.
[[nodiscard]] std::optional<std::uint32_t> DuplicateWaterFeatureClip(
    WaterFeatureTimeline* timeline,
    std::uint32_t clipId,
    float targetStart,
    bool allowWrap = false);

// Copies or moves one explicitly owned clip onto another feature's timeline
// over the same window. Both features must share one feature kind; existing
// destination clips and loose keys survive. Returns the destination clip id.
[[nodiscard]] std::optional<std::uint32_t> TransferWaterFeatureClip(
    WaterFeatureTimeline* source,
    std::uint32_t clipId,
    WaterFeatureTimeline* destination,
    bool removeFromSource);

// Removes the clip entry, and either deletes its explicitly owned keys or
// leaves them behind as loose keys.
[[nodiscard]] bool RemoveWaterFeatureClip(
    WaterFeatureTimeline* timeline,
    std::uint32_t clipId,
    bool removeKeys);

struct WaterFeatureTimingSampleEntry {
    WaterKeyedFeatureId feature{};
    std::string settingId;
    float value = 0.0F;
};

// Every keyed (feature, setting) evaluated at one normalized position.
struct WaterFeatureTimingOverlay {
    std::vector<WaterFeatureTimingSampleEntry> samples;
    // Membership includes enabled and disabled runs. The toggle decides
    // whether that membership is merely informational or a visibility
    // allow-list for runtime consumers.
    std::vector<WaterKeyedFeatureId> assignedRunFeatures;
    bool onlyShowRunFeatures = false;

    [[nodiscard]] const float* Find(
        const WaterKeyedFeatureId& feature,
        std::string_view settingId) const;
    // Global Shoreline/Seepage/Flow ids are category umbrellas: assigning a
    // global permits every matching object, while assigning any object keeps
    // its category-level master available without permitting its siblings.
    [[nodiscard]] bool Allows(const WaterKeyedFeatureId& feature) const;
};

[[nodiscard]] WaterFeatureTimingOverlay BuildWaterFeatureTimingOverlay(
    std::span<const WaterFeatureTimingRun> runs,
    float normalizedPosition,
    bool cyclic = false,
    bool onlyShowRunFeatures = false);
// Applies active singleton "level" samples onto the legacy frame-control
// carrier (Rain, Mesh Flow, Shoreline). Parsed Seepage/Flow global samples
// are deliberately ignored.
void ApplyWaterFeatureTimingOverlayToScenario(
    const WaterFeatureTimingOverlay& overlay,
    WaterScenarioState* state);

// By default resolves only enabled runs, so callers asking "which run drives
// this feature right now" see disabled runs as absent. Membership checks that
// must also respect muted runs (assignment lists, import conflicts) pass
// includeDisabled = true.
[[nodiscard]] const WaterFeatureTimingRun* FindWaterFeatureRunContaining(
    std::span<const WaterFeatureTimingRun> runs,
    const WaterKeyedFeatureId& feature,
    bool includeDisabled = false);
[[nodiscard]] WaterFeatureTimeline* FindWaterFeatureTimeline(
    WaterFeatureTimingRun* run,
    const WaterKeyedFeatureId& feature);
[[nodiscard]] const WaterFeatureTimeline* FindWaterFeatureTimeline(
    const WaterFeatureTimingRun* run,
    const WaterKeyedFeatureId& feature);

struct WaterSeepageRainEnvelope {
    float sampleRateHz = 120.0F;
    float durationSeconds = 0.0F;
    std::vector<float> samples;
    std::string fingerprint;
};

struct WaterMeshFlowRainEnvelope {
    float sampleRateHz = 120.0F;
    float durationSeconds = 0.0F;
    std::vector<float> samples;
    std::string fingerprint;
};

inline constexpr float kWaterRainEnvelopeSampleRateHz = 120.0F;
inline constexpr std::size_t kWaterRainEnvelopeMaximumSamples = 1'000'000U;

// A bounded, deterministic sampling domain shared by authored Seepage and
// Mesh Flow Rain responses. When the requested duration would exceed the
// sample budget, the effective rate is reduced while retaining both
// endpoints.
struct WaterRainEnvelopeDomain {
    float durationSeconds = 0.0F;
    float sampleRateHz = kWaterRainEnvelopeSampleRateHz;
    float stepSeconds = 0.0F;
    std::size_t sampleCount = 1U;
};

struct WaterRainResponseSettings {
    float delaySeconds = 0.0F;
    float riseSeconds = 0.0F;
    float recessionSeconds = 0.0F;

    friend auto operator<=>(
        const WaterRainResponseSettings&,
        const WaterRainResponseSettings&) = default;
};

// Bounded fixed-step schedule used when a stateful GPU Mesh Flow simulation is
// sampled by an offline render. Sequential samples advance from the settled
// state; seeks rebuild only the visible history window.
struct WaterMeshFlowTimelineStep {
    float timeSeconds = 0.0F;
    float deltaSeconds = 0.0F;
    bool resetSimulation = false;
};

struct WaterSeepageViewContext {
    invisible_places::io::Float3 cameraPosition{};
    bool hasCameraPosition = false;
};

struct WaterSeepageNode {
    std::uint32_t id = 0U;
    std::string name = "Seepage";
    invisible_places::io::Float3 position{};
    invisible_places::io::Float3 surfaceNormal{0.0F, 1.0F, 0.0F};
    invisible_places::io::Float3 downAxis{0.0F, 0.0F, -1.0F};
    // Legacy nominal travel, kept for the cache-less guided/planar fallback
    // and old documents. Connected support derives its travel budget from
    // Node Strength alone (kWaterSeepageRunMetersPerStrength).
    float reachMeters = 1.25F;
    // Width is the live full width of the always-wet source patch around the
    // node; everywhere beyond it the affected area comes from the
    // least-resistance flood budget (Node Strength). The legacy start/end
    // fields remain readable during the staged migration.
    float widthMeters = 0.10F;
    float startWidthMeters = 0.12F;
    float endWidthMeters = 0.75F;
    // Prominence scales only how strongly the effect is applied, never where
    // it applies; strength shapes the affected area.
    float prominence = 1.0F;
    // Cache-cell support is authored to fixed limits. Reach/width edits inside
    // these limits remain parameter-only; increasing a limit is the explicit
    // topology-changing operation.
    float selectionReachLimitMeters = 2.34375F;
    float selectionWidthLimitMeters = 1.215F;
    float edgeFeatherMeters = 0.10F;
    float depthToleranceMeters = 0.15F;
    float normalAlignment = 0.20F;
    float strength = 1.0F;
    // Authored, node-specific response to the global Rain amount. These
    // parameters affect only the live wetness envelope and never rebuild
    // connected support.
    float rainDelaySeconds = 0.0F;
    float rainRiseSeconds = 0.0F;
    float rainRecessionSeconds = 0.0F;
    std::uint32_t seed = 1U;
    bool enabledInViewport = true;
    bool enabledInExport = true;
    std::vector<std::string> targetSceneRoles{"ROCK", "VEG"};
    std::string settingsProfileName = "Default";
    std::string lookProfileName = "Default";
    std::string responseProfileName = "Default";
    // Legacy migration only: older documents stored per-node look copies.
    // Loading materializes them as named profiles and clears these; the
    // resolve path never reads them and they are no longer serialized.
    std::optional<WaterSeepageLookSettings> lookOverride;
    std::optional<WaterSeepageLookSettings> tempLookOverride;
};

inline constexpr std::size_t kWaterSeepageMaximumGuideSamples = 8U;

struct WaterSeepageGuideSample {
    invisible_places::io::Float3 position{};
    invisible_places::io::Float3 normal{0.0F, 1.0F, 0.0F};
    float station = 0.0F;
    float confidence = 1.0F;
};

struct WaterSeepageSurfaceGuide {
    std::uint32_t nodeId = 0U;
    std::uint32_t sampleCount = 0U;
    std::array<WaterSeepageGuideSample, kWaterSeepageMaximumGuideSamples> samples{};
    float requestedReachMeters = 0.0F;
    float achievedReachMeters = 0.0F;
    bool valid = false;
    bool complete = false;
};

struct WaterSeepageRuntimeNode {
    std::uint32_t id = 0U;
    std::uint32_t seed = 1U;
    glm::vec3 position{0.0F, 0.0F, 0.0F};
    glm::vec3 surfaceNormal{0.0F, 1.0F, 0.0F};
    glm::vec3 downAxis{0.0F, 0.0F, -1.0F};
    glm::vec3 lateralAxis{1.0F, 0.0F, 0.0F};
    glm::mat3 noiseRotation{1.0F};
    glm::vec3 environmentDirection{0.0F, 0.0F, 1.0F};
    float reachMeters = 1.25F;
    float widthMeters = 0.75F;
    float selectionReachLimitMeters = 2.34375F;
    float selectionWidthLimitMeters = 1.215F;
    // Immutable extent derived from the connected upstream support. It lets
    // the live mask reveal the real highest resident cell back toward
    // the node instead of assuming that every surface reaches the authored
    // selection limit.
    float maximumUpstreamRunMeters = 0.0F;
    float prominence = 1.0F;
    float authoredReachMeters = 1.25F;
    float authoredWidthMeters = 0.75F;
    float authoredProminence = 1.0F;
    float authoredStartHalfWidthMeters = 0.06F;
    float startHalfWidthMeters = 0.06F;
    float endHalfWidthMeters = 0.375F;
    float edgeFeatherMeters = 0.10F;
    float depthToleranceMeters = 0.15F;
    float normalAlignment = 0.20F;
    float strength = 1.0F;
    // Live least-resistance travel budget in cost-metres, derived from
    // strength (and reach-scale animation, spread, and rain gains), clamped
    // to the selection reach limit.
    float budgetMeters = 1.5F;
    float rainVisualStrength = 0.0F;
    float scenarioSpread = 0.0F;
    float effectiveActivity = 1.0F;
    // Viewport visibility is a compact live parameter. Disabled viewport
    // nodes remain in the spatial topology so toggling them cannot require a
    // descriptor or hash-grid rebuild.
    float enabledFactor = 1.0F;
    float localSpread = 0.0F;
    float wettingProgress = 1.0F;
    float authoredStrength = 1.0F;
    WaterSeepageQuality resolvedQuality = WaterSeepageQuality::Balanced;
    WaterSeepageLookSettings authoredLook{};
    WaterSeepageLookSettings look{};
    std::optional<WaterSeepageLookSettings> transitionLook;
    float transitionAmount = 0.0F;
    // Contour Pulses are composed into a compact reference field only when
    // their authored shape changes. Rendering advances through that field
    // from the frame time on the GPU (and with the same sampler offline), so
    // a clock-only frame never rebuilds or uploads Seepage parameters. The
    // span comes from immutable maximum support limits, never live
    // Strength/Rain, so revealing more support cannot teleport wave phase.
    float pulseStableSpanMeters = 1.0F;
    std::uint64_t pulseFieldPreparationFingerprint = 0U;
    WaterSeepagePulseField pulseField{};
    WaterSeepagePulseField transitionPulseField{};
    std::uint32_t guideSampleCount = 0U;
    std::array<WaterSeepageGuideSample, kWaterSeepageMaximumGuideSamples> guideSamples{};
    float guideRequestedReachMeters = 0.0F;
    float guideAchievedReachMeters = 0.0F;
    bool guideValid = false;
    bool guideComplete = false;
    bool usesConnectedSupport = false;
};

inline constexpr float kWaterSeepageSupportCellSizeMeters = 0.010F;
inline constexpr std::size_t kWaterSeepageMaximumSupportCellsPerNode = 262'144U;

// Least-resistance flood model: support is selected by labelled downstream
// and upstream Dijkstra floods over connected cache surfels. Steps are
// decomposed in each surfel's tangent plane, so steep descent is cheap,
// cross-contour creep is expensive, and the short upstream wick cannot enter
// through a cheaper downstream detour. Node Strength converts to a live
// travel budget in cost-metres; full-precision run/cross metrics then reshape
// the selected support per frame without topology work.
inline constexpr float kWaterSeepageDescentCostFactor = 0.75F;
inline constexpr float kWaterSeepageContourCostFactor = 2.2F;
inline constexpr float kWaterSeepageAscentCostFactor = 5.0F;
inline constexpr float kWaterSeepageUpstreamCostFactor = 1.0F;
// Reveal the high wick from its resident tip back toward the node before
// releasing the downhill front. Keep this aligned with
// SeepageConnectedSupportMask in pointcloud_sparse_ripple.glsl.
inline constexpr float kWaterSeepageUpstreamLeadFraction = 0.12F;
inline constexpr float kWaterSeepageSteepContourCostFactor = 8.8F;
inline constexpr float kWaterSeepageSteepnessBlendStart = 0.35F;
inline constexpr float kWaterSeepageSteepnessBlendEnd = 0.75F;
inline constexpr float kWaterSeepageRunMetersPerStrength = 1.5F;
// Vegetation more than this far above its connected substrate is hovering
// canopy and stays dry.
inline constexpr float kWaterSeepageVegetationRiseMeters = 0.15F;
inline constexpr std::uint32_t kWaterSeepageSupportConnectedFlag = 1U;
inline constexpr std::uint32_t kWaterSeepageSupportUpstreamFlag = 2U;

// One density-independent cache cell selected beneath an authored node. The
// metrics are evaluated against live node parameters and therefore do not
// change while an animation is playing.
struct WaterSeepageSupportCell {
    std::int32_t x = 0;
    std::int32_t y = 0;
    std::int32_t z = 0;
    // Accumulated least-resistance cost from the node (cost-metres).
    // The field name predates the tangent-decomposed flood and remains for
    // compatibility with diagnostics and existing callers.
    float downwardDistanceMeters = 0.0F;
    // Full-precision tangent-plane metrics used by the CPU Structure Overlay.
    // Downstream cross-contour distance is accumulated from the node. For an
    // upstream cell it is the excess above the cheapest route in the same
    // flow-run station, retaining branch width without hiding a winding tip.
    // Runtime references pack both non-negative values as half-floats.
    float flowRunMeters = 0.0F;
    float crossContourMeters = 0.0F;
    invisible_places::io::Float3 surfaceNormal{0.0F, 0.0F, 1.0F};
    float confidence = 0.0F;
    bool upstream = false;
};

struct WaterSeepageSupportSelectionDiagnostics {
    std::uint32_t visitedSurfelCount = 0U;
    std::uint32_t acceptedSurfelCount = 0U;
    std::uint32_t emittedCellCount = 0U;
    std::uint32_t rejectedAboveNodeCount = 0U;
    std::uint32_t rejectedReachCount = 0U;
    std::uint32_t rejectedWidthCount = 0U;
    std::uint32_t rejectedContinuityCount = 0U;
    bool cellLimitExceeded = false;
};

struct WaterSeepageSupportSelection {
    std::uint32_t nodeId = 0U;
    std::string targetSceneRole;
    WaterSurfaceRole sourceRole = WaterSurfaceRole::None;
    float cellSizeMeters = kWaterSeepageSupportCellSizeMeters;
    float reachLimitMeters = 0.0F;
    float widthLimitMeters = 0.0F;
    std::vector<WaterSeepageSupportCell> cells;
    invisible_places::io::Bounds3f bounds{};
    std::string fingerprint;
};

struct WaterSeepageSupportBuildOptions {
    std::size_t maximumSupportCells = kWaterSeepageMaximumSupportCellsPerNode;
    std::size_t maximumVisitedSurfels = kWaterSeepageMaximumSupportCellsPerNode;
    const std::stop_token* stopToken = nullptr;
};

struct WaterSeepageSupportBuildResult {
    WaterSeepageSupportSelection selection;
    WaterSeepageSupportSelectionDiagnostics diagnostics{};
    std::string errorMessage;
    bool success = false;
    bool cancelled = false;
    // The node is valid but this authored role has no local substrate/occupancy
    // to shade. Callers settle this as an empty node-role result rather than
    // poisoning the complete role draft or retrying it every frame.
    bool surfaceUnavailable = false;
};

struct alignas(16) WaterSeepageSupportReference {
    std::uint32_t nodeIndex = 0U;
    float downwardDistanceMeters = 0.0F;
    // x: accumulated local flow-run; y: downstream cross-contour travel or
    // station-relative upstream branch excess.
    // Both are non-negative IEEE-754 half-floats packed into one 32-bit lane.
    std::uint32_t packedRunCrossMeters = 0U;
    // Octahedral normal (10+10 bits), confidence (8 bits), authored terrain
    // role (2 bits), and flags (2 bits). Keeping this record exactly 16 bytes
    // lets the CPU and std430 GPU paths share one bounded reference payload.
    std::uint32_t packedNormalRoleConfidenceFlags = 0U;
};

static_assert(sizeof(WaterSeepageSupportReference) == 16U);

struct WaterSeepageSupportRunCrossMetrics {
    float flowRunMeters = 0.0F;
    float crossContourMeters = 0.0F;
};

struct WaterSeepageSupportReferenceMetadata {
    invisible_places::io::Float3 surfaceNormal{0.0F, 0.0F, 1.0F};
    WaterSurfaceRole sourceRole = WaterSurfaceRole::None;
    float confidence = 0.0F;
    // kWaterSeepageSupportConnectedFlag and
    // kWaterSeepageSupportUpstreamFlag.
    std::uint32_t flags = 0U;
};

[[nodiscard]] std::uint32_t PackWaterSeepageSupportReferenceMetadata(
    const invisible_places::io::Float3& surfaceNormal,
    WaterSurfaceRole sourceRole,
    float confidence,
    std::uint32_t flags = 0U);
[[nodiscard]] WaterSeepageSupportReferenceMetadata
UnpackWaterSeepageSupportReferenceMetadata(std::uint32_t packed);
[[nodiscard]] std::uint32_t PackWaterSeepageSupportRunCrossMetrics(
    float flowRunMeters,
    float crossContourMeters);
[[nodiscard]] WaterSeepageSupportRunCrossMetrics
UnpackWaterSeepageSupportRunCrossMetrics(std::uint32_t packed);

struct WaterSeepageSpatialHashCell {
    std::int32_t x = 0;
    std::int32_t y = 0;
    std::int32_t z = 0;
    std::uint32_t referenceOffset = 0U;
    std::uint32_t referenceCount = 0U;
    bool occupied = false;
};

struct WaterSeepageSpatialGridDiagnostics {
    std::uint32_t inputNodeCount = 0U;
    std::uint32_t activeNodeCount = 0U;
    std::uint32_t occupiedCellCount = 0U;
    std::uint32_t hashCellCapacity = 0U;
    std::uint32_t nodeReferenceCount = 0U;
    std::uint32_t overflowCellCount = 0U;
    std::uint32_t droppedNodeReferenceCount = 0U;
    std::uint32_t maxReferencesPerCell = 8U;
    std::uint32_t supportSelectionCount = 0U;
    std::uint32_t supportOccupiedCellCount = 0U;
    std::uint32_t supportHashCellCapacity = 0U;
    std::uint32_t supportReferenceCount = 0U;
    std::uint32_t supportOverflowCellCount = 0U;
    std::uint32_t droppedSupportReferenceCount = 0U;
};

struct WaterSeepageSpatialGrid {
    static constexpr std::uint32_t kMaxReferencesPerCell = 8U;

    std::vector<WaterSeepageRuntimeNode> nodes;
    std::vector<WaterSeepageSpatialHashCell> hashCells;
    std::vector<std::uint32_t> nodeReferences;
    // When populated, this exact 10 mm hash replaces fan membership. The
    // coarse node grid remains available as the legacy/fallback path while
    // callers migrate to connected support selections.
    std::vector<WaterSeepageSpatialHashCell> supportHashCells;
    std::vector<WaterSeepageSupportReference> supportReferences;
    invisible_places::io::Bounds3f unionBounds{};
    invisible_places::io::Bounds3f supportUnionBounds{};
    float cellSizeMeters = 0.50F;
    float supportCellSizeMeters = kWaterSeepageSupportCellSizeMeters;
    WaterSeepageSpatialGridDiagnostics diagnostics{};
};

struct WaterSeepageRuntimeContribution {
    float mask = 0.0F;
    float damp = 0.0F;
    float ripple = 0.0F;
    float glint = 0.0F;
    float scale = 0.0F;
    float colourMix = 0.0F;
    float emissionAdd = 0.0F;
    float opacityAdd = 0.0F;
    float opacityMultiply = 1.0F;
    float pointSizeAdd = 0.0F;
    float pointSizeMultiply = 1.0F;
    glm::vec3 colour{0.28F, 0.42F, 0.46F};
};

struct WaterFlowTrailSettings {
    bool enabled = true;
    std::uint32_t trailCountTotal = 700;
    std::uint32_t laneCount = 0;
    float trailLengthMeters = 0.75F;
    float trailPointSpacingMeters = 0.010F;
    float trailWidthMeters = 0.006F;
    float trailStreakLengthMeters = 0.045F;
    bool startFadeEnabled = false;
    float startFadeFullDistanceMeters = 0.25F;
    float startFadeRandomBeginDistanceMeters = 0.10F;
    bool endFadeEnabled = false;
    float endFadeFullDistanceMeters = 0.25F;
    float endFadeRandomBeginDistanceMeters = 0.10F;
    float surfaceOffsetMeters = 0.004F;
    float pathAttraction = 0.85F;
    float laneSpreadMeters = 0.12F;
    float laneCrossing = 0.22F;
    float trailSmoothness = 0.85F;
    float trailLooseness = 0.08F;
    float turbulence = 0.06F;
    float surfaceFollow = 0.85F;
    float downhillPull = 0.35F;
    float terrainWidthResponse = 0.65F;
    float turbulenceScaleMeters = 0.18F;
    float speedMetersPerSecond = 0.45F;
    std::uint32_t seed = 1;
};

struct WaterFlowTrailBuildOptions {
    const std::stop_token* stopToken = nullptr;
    // CPU reference/fallback hook. GPU Flow uses the resident cache directly.
    const WaterSurfaceCache* surfaceCache = nullptr;
    bool useSurfaceGuide = false;
};

// Keep this value in lock-step with WaterTrailSample serialization and every
// GPU Flow writer. Scalar storage is field-major: field * pointCapacity + point.
inline constexpr std::uint32_t kWaterTrailScalarFieldCount = 36U;

enum class WaterFlowGpuInputKind : std::uint32_t {
    SampledAnchors = 0U,
    ManualCatmullRomControlPoints = 1U,
};

// Per-node lane-cover control for authored Flow paths. Inherit follows the
// selected Lane profile, Absolute stores metres independently of that
// profile, and Relative multiplies it (0.2 = 20%, 2.3 = 230%).
enum class WaterManualFlowPathLaneWidthMode : std::uint32_t {
    Inherit = 0U,
    Absolute = 1U,
    Relative = 2U,
};

struct WaterManualFlowPathLaneWidth {
    WaterManualFlowPathLaneWidthMode mode =
        WaterManualFlowPathLaneWidthMode::Inherit;
    float value = 1.0F;
};

struct WaterOverlayPoint;
struct WaterPathAnalysisCache;

// Compact, deterministic source input shared by the CPU reference path and
// the GPU route pass. Manual splines retain only four cumulative arc-length
// checkpoints per outgoing segment, avoiding a dense sampled-curve upload on
// every control-point edit while still allowing distance-based evaluation.
struct WaterFlowGpuCompactInputPoint {
    invisible_places::io::Float3 position{};
    invisible_places::io::Float3 normal{0.0F, 0.0F, 1.0F};
    float confidence = 1.0F;
    float cumulativeDistanceMeters = 0.0F;
    std::array<float, 4> outgoingSegmentArcDistancesMeters{};
    WaterManualFlowPathLaneWidth laneWidth{};
};

struct WaterFlowGpuCompactInput {
    std::vector<WaterFlowGpuCompactInputPoint> points;
    float routeLengthMeters = 0.0F;

    [[nodiscard]] bool Valid() const {
        return points.size() >= 2U && routeLengthMeters > 1.0e-5F;
    }
};

// One authored/generated route inside a source-local compact input. Points for
// every branch live in one flat allocation, while this table retains the CPU
// path grouping and scalar identities used by the trail builder.
struct WaterFlowGpuCompactBranch {
    std::uint32_t inputStart = 0U;
    std::uint32_t inputCount = 0U;
    std::uint32_t branchId = 0U;
    std::uint32_t pathId = 0U;
    float routeLengthMeters = 0.0F;
    float allocationWeight = 0.0F;
};

struct WaterFlowGpuCompactSourceInput {
    std::vector<WaterFlowGpuCompactInputPoint> points;
    std::vector<WaterFlowGpuCompactBranch> branches;

    [[nodiscard]] bool Valid() const;
};

[[nodiscard]] WaterFlowGpuCompactInput BuildWaterFlowGpuSampledInput(
    std::span<const WaterOverlayPoint> anchors);
[[nodiscard]] WaterFlowGpuCompactInput BuildWaterFlowGpuManualSplineInput(
    std::span<const invisible_places::io::Float3> controlPoints);
[[nodiscard]] WaterFlowGpuCompactInput BuildWaterFlowGpuManualSplineInput(
    std::span<const invisible_places::io::Float3> controlPoints,
    std::span<const WaterManualFlowPathLaneWidth> laneWidths);

// Generated point sources can contain several consecutive path branches. This
// contract flattens them without joining branch endpoints and carries the same
// exact trail-allocation weights as the deterministic CPU builder.
[[nodiscard]] WaterFlowGpuCompactSourceInput BuildWaterFlowGpuSampledSourceInput(
    std::span<const WaterOverlayPoint> anchors,
    const WaterFlowTrailSettings& settings,
    const WaterPathAnalysisCache* analysis = nullptr);
[[nodiscard]] WaterFlowGpuCompactSourceInput BuildWaterFlowGpuManualSplineSourceInput(
    std::span<const invisible_places::io::Float3> controlPoints,
    std::uint32_t branchId = 0U,
    std::uint32_t pathId = 1U);
[[nodiscard]] WaterFlowGpuCompactSourceInput BuildWaterFlowGpuManualSplineSourceInput(
    std::span<const invisible_places::io::Float3> controlPoints,
    std::span<const WaterManualFlowPathLaneWidth> laneWidths,
    std::uint32_t branchId = 0U,
    std::uint32_t pathId = 1U);

struct WaterFlowGpuBranchLayout {
    std::uint32_t inputStart = 0U;
    std::uint32_t inputCount = 0U;
    std::uint32_t routeStart = 0U;
    std::uint32_t routePointsPerLane = 0U;
    std::uint32_t activeRouteLaneCount = 0U;
    std::uint32_t trailOutputStart = 0U;
    std::uint32_t trailCount = 0U;
    std::uint32_t firstTrailId = 1U;
    std::uint32_t potentialLaneCount = 0U;
    std::uint32_t branchId = 0U;
    std::uint32_t pathId = 0U;
    float routeLengthMeters = 0.0F;
    float routeSpacingMeters = 0.0F;
};

struct WaterFlowGpuOutputLayout {
    std::vector<WaterFlowGpuBranchLayout> branches;
    std::uint32_t branchCount = 0U;
    std::uint32_t inputPointCount = 0U;
    std::uint32_t laneCount = 0U;
    std::uint32_t maxActiveRouteLaneCount = 0U;
    std::uint32_t maxTrailsPerBranch = 0U;
    std::uint32_t routePointCountPerLane = 0U;
    std::uint32_t routePointCountTotal = 0U;
    std::uint32_t trailCount = 0U;
    std::uint32_t samplesPerTrail = 0U;
    std::uint32_t trailPointCountTotal = 0U;
    std::uint32_t pointCount = 0U;
    std::uint32_t pointCapacity = 0U;
    float routeLengthMeters = 0.0F;

    [[nodiscard]] bool Valid() const {
        if (inputPointCount < 2U || laneCount == 0U || routePointCountPerLane < 2U ||
            trailCount == 0U || samplesPerTrail < 2U || pointCount == 0U ||
            pointCapacity < pointCount || branchCount == 0U ||
            branchCount != branches.size()) {
            return false;
        }
        std::uint64_t expectedTrailId = 1U;
        std::uint64_t allocatedTrails = 0U;
        std::uint64_t expectedRouteStart = 0U;
        std::uint64_t expectedTrailStart = routePointCountTotal;
        for (const auto& branch : branches) {
            const std::uint64_t inputEnd =
                static_cast<std::uint64_t>(branch.inputStart) + branch.inputCount;
            const std::uint64_t routeEnd =
                static_cast<std::uint64_t>(branch.routeStart) +
                static_cast<std::uint64_t>(branch.routePointsPerLane) *
                    branch.activeRouteLaneCount;
            const std::uint64_t trailEnd =
                static_cast<std::uint64_t>(branch.trailOutputStart) +
                static_cast<std::uint64_t>(branch.trailCount) * samplesPerTrail;
            if (branch.inputCount < 2U || inputEnd > inputPointCount ||
                branch.routePointsPerLane < 2U ||
                branch.routeStart != expectedRouteStart || routeEnd > routePointCountTotal ||
                branch.trailOutputStart != expectedTrailStart || trailEnd > pointCount ||
                branch.firstTrailId != expectedTrailId ||
                branch.activeRouteLaneCount > branch.potentialLaneCount ||
                branch.activeRouteLaneCount > branch.trailCount) {
                return false;
            }
            expectedTrailId += branch.trailCount;
            allocatedTrails += branch.trailCount;
            expectedRouteStart = routeEnd;
            expectedTrailStart = trailEnd;
        }
        return allocatedTrails == trailCount && expectedRouteStart == routePointCountTotal &&
               expectedTrailStart == pointCount;
    }
};

// Deterministic sizing contract shared by the renderer and tests. Capacity is
// geometric and never shrinks while currentPointCapacity can satisfy the edit.
[[nodiscard]] WaterFlowGpuOutputLayout BuildWaterFlowGpuOutputLayout(
    const WaterFlowGpuCompactSourceInput& input,
    const WaterFlowTrailSettings& settings,
    std::uint32_t currentPointCapacity = 0U,
    std::uint32_t maximumPointCapacity = 1'200'000U);
[[nodiscard]] WaterFlowGpuOutputLayout BuildWaterFlowGpuOutputLayout(
    std::uint32_t inputPointCount,
    float routeLengthMeters,
    const WaterFlowTrailSettings& settings,
    std::uint32_t currentPointCapacity = 0U,
    std::uint32_t maximumPointCapacity = 1'200'000U);

[[nodiscard]] bool WaterFlowLaneRouteInputsEqual(
    const WaterFlowTrailSettings& left,
    const WaterFlowTrailSettings& right);
[[nodiscard]] bool WaterFlowLaneSpeedOnlyEdit(
    const WaterFlowTrailSettings& before,
    const WaterFlowTrailSettings& after);

struct WaterTrailGeometrySettings {
    float trailLengthMeters = 0.75F;
    float pointSpacingMeters = 0.010F;
    float widthMeters = 0.006F;
    float streakLengthMeters = 0.045F;
    bool startFadeEnabled = false;
    float startFadeFullDistanceMeters = 0.25F;
    float startFadeRandomBeginDistanceMeters = 0.10F;
    bool endFadeEnabled = false;
    float endFadeFullDistanceMeters = 0.25F;
    float endFadeRandomBeginDistanceMeters = 0.10F;
};

[[nodiscard]] WaterTrailGeometrySettings DefaultWaterTrailGeometrySettings();
[[nodiscard]] float AutoWaterTrailPointSpacingMeters(float trailLengthMeters, float widthMeters);
[[nodiscard]] float AutoWaterTrailStreakLengthMeters(float pointSpacingMeters, float widthMeters);
[[nodiscard]] WaterTrailGeometrySettings FitWaterTrailGeometryForContinuousLines(
    WaterTrailGeometrySettings geometry);
[[nodiscard]] WaterTrailGeometrySettings WaterTrailGeometryFromFlowTrailSettings(
    const WaterFlowTrailSettings& settings);
[[nodiscard]] WaterFlowTrailSettings ApplyWaterTrailGeometryToFlowTrailSettings(
    WaterFlowTrailSettings settings,
    const WaterTrailGeometrySettings& geometry);
[[nodiscard]] bool WaterTrailGeometryGenerationInputsEqual(
    const WaterTrailGeometrySettings& left,
    const WaterTrailGeometrySettings& right);
[[nodiscard]] bool WaterTrailGeometryLiveVisualOnlyEdit(
    const WaterTrailGeometrySettings& before,
    const WaterTrailGeometrySettings& after);

using WaterRainIntensityPreset = RainIntensityPreset;
using WaterRainSettings = RainRuntimeSettings;

inline constexpr float kWaterTrailFeatureTypeDynamicMesh = 5.0F;

struct WaterDynamicMeshMotionKeyframe {
    float timeSeconds = 0.0F;
    invisible_places::io::Float3 position{};
};

struct WaterDynamicMeshAttractor {
    std::uint32_t id = 1;
    std::string name = "Attractor";
    invisible_places::io::Float3 position{};
    float radiusMeters = 0.65F;
    float strength = 0.75F;
    bool enabled = true;
    std::vector<WaterDynamicMeshMotionKeyframe> keyframes;
};

struct WaterDynamicMeshEmitterMotion {
    std::uint32_t emitterId = 0;
    std::string name = "Emitter Motion";
    bool enabled = true;
    std::vector<WaterDynamicMeshMotionKeyframe> keyframes;
};

struct WaterDynamicMeshParticlePreset {
    std::string_view name;
    std::string_view label;
};

struct WaterDynamicMeshRockResponseSettings {
    float radiusMeters = 0.12F;
    float opacityAdd = 0.16F;
    float emissionAdd = 0.35F;
    invisible_places::io::Float3 colourise{0.18F, 0.42F, 0.55F};
    float colouriseAmount = 0.45F;
    float persistenceSeconds = 2.5F;
};

struct WaterDynamicMeshVegetationResponseSettings {
    float radiusMeters = 0.18F;
    float opacityAdd = 0.14F;
    float emissionAdd = 0.55F;
    invisible_places::io::Float3 colourise{0.18F, 0.55F, 0.48F};
    float colouriseAmount = 0.50F;
    float persistenceSeconds = 3.0F;
    float twinkle = 1.4F;
    float streamDepthMeters = 0.45F;
};

struct WaterDynamicMeshFlowSettings {
    bool enabled = false;
    bool gpuPreviewEnabled = true;
    bool showTrails = true;
    // Retained only so schema-44 projects can be parsed. Mesh Flow is
    // automatic in schema 45 and never consumes ordinary Flow emitters.
    bool automaticSources = true;
    std::filesystem::path meshPath{};
    float cacheCellSizeMeters = 0.08F;
    float projectionSearchRadiusMeters = 1.25F;
    float ambiguityHeightMeters = 0.18F;
    // The fixed-capacity runtime changes active particle/history counts through
    // parameters; it does not resize buffers while a frame is in flight.
    std::uint32_t particleCapacity = 4096U;
    std::uint32_t historyLength = 24U;
    float sourceBandWidthMeters = 0.75F;
    float sourceBandFraction = 0.04F;
    float dryConcavityFocus = 0.90F;
    // 0 keeps dry spawns concentrated at the most likely (convergent) rim
    // cells; 1 accepts candidates along the entire sampled-Ground +X rim
    // regardless of moisture, so trails arrive along the whole rock edge.
    float edgeCoverage = 0.0F;
    // Authored moisture response. Activity is the dry baseline; Rain Gain
    // fills the remaining activity from the filtered global Rain amount.
    float activity = 1.0F;
    float rainGain = 0.0F;
    float moisturePersistenceMultiplier = 1.0F;
    float rainRiseSeconds = 0.0F;
    float rainRecessionSeconds = 0.0F;
    float rainSpawnSpread = 0.75F;
    float rainDistributedSourceFraction = 0.55F;
    std::uint32_t previewParticleLimit = 560;
    std::uint32_t finalParticleLimit = 2400;
    float trailLengthMeters = 18.0F;
    float stepMeters = 0.12F;
    float trailWidthMeters = 0.0025F;
    float trailStreakLengthMeters = 0.030F;
    float surfaceOffsetMeters = 0.003F;
    float trailOpacityDry = 0.025F;
    float trailOpacityWet = 0.14F;
    float trailEmissionDry = 0.04F;
    float trailEmissionWet = 0.45F;
    float trailExposure = 1.25F;
    float speedMetersPerSecond = 0.26F;
    float downhillWeight = 1.75F;
    float attractorWeight = 1.0F;
    float sourceVelocityWeight = 0.35F;
    float curlStrength = 0.18F;
    float branchingStrength = 0.36F;
    float eddyStrength = 0.08F;
    float topologyResponse = 0.65F;
    float inertia = 0.88F;
    float particleNoiseStrength = 0.10F;
    float particleNoiseScaleMeters = 0.45F;
    float particleNoiseSpeed = 0.18F;
    // Depth of the surface population's stall-and-rush speed cycle (0 keeps
    // every trail steady). Roughly half the particles flow as slow, steady
    // aquifer filaments; the rest read as surface water pooling against
    // debris and surging over it.
    float surfaceSurge = 0.6F;
    float sharedWindStrength = 0.035F;
    float sharedWindScaleMeters = 3.0F;
    float sharedWindSpeed = 0.025F;
    float contactFadeSeconds = 0.8F;
    // How far above a terrain contact the wetting response blends (both ROCK
    // and VEG contacts), so overhanging points wet instead of cutting off at
    // the collision height.
    float contactUpwardReachMeters = 0.6F;
    // Trails read at least this wet regardless of scenario moisture, keeping
    // the underground-water filaments visible in dry scenes.
    float trailWetnessFloor = 0.75F;
    WaterDynamicMeshRockResponseSettings rockResponse{};
    WaterDynamicMeshVegetationResponseSettings vegetationResponse{};
    float animationDurationSeconds = 4.0F;
    std::uint32_t seed = 29U;
    std::string particlePresetName = "Default";
    std::string trailProfileName = "Default";
    std::vector<WaterDynamicMeshAttractor> attractors;
    std::vector<WaterDynamicMeshEmitterMotion> emitterMotions;
};

// CPU mirror of the bounded GPU regime weights.  This is deliberately compact:
// timeline changes evaluate these scalars and rotate the live uniform ring; they
// never rebuild Ground topology or resize the fixed particle/history buffers.
struct WaterDynamicMeshFlowVisualWeights {
    float automaticSpawnAcceptance = 1.0F;
    float directionalNoiseScale = 1.0F;
    float trailProminence = 1.0F;
    float trailWidthScale = 1.0F;
    float trailStreakScale = 1.0F;
};

// Cache-owned automatic source on the vegetation-supported Ground surface,
// ordered by geodesic distance from the component's +X rim (hash-shuffled
// within 0.10 m bands so the GPU's index bias spreads spawns along the whole
// curved edge). The 16-byte layout is also the std430 GPU entry ABI; live
// Mesh Flow parameters only select from this immutable table.
struct alignas(16) WaterDynamicMeshFlowGroundEntry {
    std::int32_t cellX = 0;
    std::int32_t cellY = 0;
    float edgeDistanceMeters = 0.0F;
    float edgeDistanceFraction = 0.0F;
};

static_assert(sizeof(WaterDynamicMeshFlowGroundEntry) == 16U);

struct MeshSurfaceCacheCell {
    invisible_places::io::Float3 position{};
    invisible_places::io::Float3 normal{0.0F, 0.0F, 1.0F};
    invisible_places::io::Float3 downhill{1.0F, 0.0F, 0.0F};
    int cellX = 0;
    int cellY = 0;
    float minZ = 0.0F;
    float maxZ = 0.0F;
    float confidence = 1.0F;
    std::uint32_t sampleCount = 0;
    bool ambiguous = false;
};

struct MeshSurfaceCache {
    std::uint32_t schemaVersion = 1;
    std::filesystem::path meshPath;
    std::string meshSignature;
    WaterDynamicMeshFlowSettings settings{};
    std::vector<MeshSurfaceCacheCell> cells;
    std::unordered_map<std::uint64_t, std::uint32_t> cellLookup;
    invisible_places::io::Bounds3f bounds{};
    std::uint64_t sourceVertexCount = 0;
    std::uint64_t sourceTriangleCount = 0;
    double buildMilliseconds = 0.0;
    bool stale = false;
};

struct MeshSurfaceProjection {
    invisible_places::io::Float3 position{};
    invisible_places::io::Float3 normal{0.0F, 0.0F, 1.0F};
    invisible_places::io::Float3 downhill{1.0F, 0.0F, 0.0F};
    float confidence = 0.0F;
    bool ambiguous = false;
    bool hit = false;
};

struct WaterDynamicMeshFlowDiagnostics {
    double meshLoadMilliseconds = 0.0;
    double cacheBuildMilliseconds = 0.0;
    double solveMilliseconds = 0.0;
    double gpuStaticUploadMilliseconds = 0.0;
    double gpuLiveUploadMilliseconds = 0.0;
    double gpuDispatchMilliseconds = 0.0;
    std::uint64_t sourceVertexCount = 0;
    std::uint64_t sourceTriangleCount = 0;
    std::uint32_t cacheCellCount = 0;
    std::uint32_t emittedPathCount = 0;
    std::uint32_t emittedSampleCount = 0;
    std::uint32_t automaticEntryCandidateCount = 0;
    std::uint32_t availableRainSeedCount = 0;
    std::uint32_t rainSeedParticleCount = 0;
    std::uint32_t routeSampleCount = 0;
    float routeWithinGroundBoundsFraction = 0.0F;
    float routeWithinSurfaceToleranceFraction = 0.0F;
    float maximumRenderedSegmentMeters = 0.0F;
    std::uint32_t longSegmentCount = 0;
    std::uint32_t unexplainedVerticalJumpCount = 0;
    float medianTangentDownhillAlignment = 0.0F;
    std::uint32_t projectionMissCount = 0;
    std::uint32_t ambiguousHitCount = 0;
    bool gpuStaticBuffersReused = false;
    bool gpuAsynchronousDispatch = false;
};

[[nodiscard]] std::array<WaterRainIntensityPreset, 3> AllWaterRainIntensityPresets();
[[nodiscard]] std::string_view WaterRainIntensityPresetLabel(WaterRainIntensityPreset preset);
[[nodiscard]] std::string_view WaterRainIntensityPresetNameForStorage(WaterRainIntensityPreset preset);
[[nodiscard]] std::optional<WaterRainIntensityPreset> ParseWaterRainIntensityPresetName(std::string_view value);
[[nodiscard]] WaterRainSettings DefaultWaterRainSettings();
// Applies Rain's active scalar tracks to a per-frame copy. Every supported
// value maps to an existing GPU/offline runtime parameter; authored settings,
// collision caches, particle capacities, and topology remain untouched.
void ApplyWaterFeatureTimingOverlayToRainSettings(
    const WaterFeatureTimingOverlay& overlay,
    WaterRainSettings* settings,
    WaterRainVisualSettings* visual);
[[nodiscard]] WaterDynamicMeshFlowSettings DefaultWaterDynamicMeshFlowSettings();
[[nodiscard]] WaterDynamicMeshFlowSettings SanitizeWaterDynamicMeshFlowSettings(
    WaterDynamicMeshFlowSettings settings);
void ApplyWaterFeatureTimingOverlayToDynamicMeshFlowSettings(
    const WaterFeatureTimingOverlay& overlay,
    WaterDynamicMeshFlowSettings* settings);
[[nodiscard]] WaterRainResponseSettings
ResolveWaterDynamicMeshFlowRainResponse(
    const WaterDynamicMeshFlowSettings& settings,
    const WaterFeatureTimingOverlay* overlay = nullptr);
[[nodiscard]] WaterDynamicMeshFlowVisualWeights
EvaluateWaterDynamicMeshFlowVisualWeights(
    const WaterDynamicMeshFlowSettings& settings,
    float convergence,
    float moisture,
    float surfaceConfidence = 1.0F);
[[nodiscard]] std::vector<WaterDynamicMeshFlowGroundEntry>
BuildWaterDynamicMeshFlowGroundEntries(const WaterSurfaceCache& cache);
[[nodiscard]] std::string WaterDynamicMeshFlowSettingsFingerprint(
    const WaterDynamicMeshFlowSettings& settings);
[[nodiscard]] std::array<WaterDynamicMeshParticlePreset, 4> AllWaterDynamicMeshParticlePresets();
[[nodiscard]] WaterDynamicMeshFlowSettings ApplyWaterDynamicMeshParticlePreset(
    WaterDynamicMeshFlowSettings settings,
    std::string_view presetName);
[[nodiscard]] std::string_view NormalizeWaterDynamicMeshParticlePresetName(std::string_view presetName);
[[nodiscard]] float WaterRainPresetVisualStrength(WaterRainIntensityPreset preset);

[[nodiscard]] WaterSeepageLookSettings DefaultWaterSeepageLookSettings();
[[nodiscard]] WaterSeepageNodeSettings ExtractWaterSeepageNodeSettings(
    const WaterSeepageNode& node);
void ApplyWaterSeepageNodeSettings(
    const WaterSeepageNodeSettings& settings,
    WaterSeepageNode* node);
// Returns the saved settings beneath an object-owned copy or a legacy
// `<base>_edited` shadow. Ordinary saved profiles have no comparison base.
// A missing/Default base resolves to `defaultSettings`, matching runtime
// profile fallback behaviour.
[[nodiscard]] std::optional<WaterSeepageNodeSettings>
ResolveWaterSeepageNodeSettingsProfileBaseline(
    const WaterSeepageNodeSettings& defaultSettings,
    std::span<const WaterSeepageNodeSettingsProfile> profiles,
    std::string_view assignedProfileName);
// Applies one scalar Timings-v2 profile sample. IDs beginning with "look."
// address the look half; "response." addresses the visual-response half.
// Returns false for an unknown/non-scalar setting.
[[nodiscard]] bool ApplyWaterSeepageLookTimingValue(
    WaterSeepageLookSettings* look,
    std::string_view settingId,
    float value);
[[nodiscard]] std::vector<WaterScenarioDefinition> DefaultWaterScenarioDefinitions();
[[nodiscard]] WaterScenarioState SanitizeWaterScenarioState(WaterScenarioState state);
[[nodiscard]] WaterScenarioState EvaluateWaterScenarioTrack(
    const WaterScenarioTrack& track,
    const WaterScenarioDefinition& definition,
    float normalizedPosition);
[[nodiscard]] float EffectiveWaterDynamicMeshFlowLevel(
    const WaterScenarioState& state,
    float effectiveRainLevel);
[[nodiscard]] float EffectiveWaterDynamicMeshFlowLevel(
    const WaterDynamicMeshFlowSettings& settings,
    float effectiveRainLevel,
    const WaterFeatureTimingOverlay* overlay = nullptr);
[[nodiscard]] float EffectiveWaterDynamicMeshPersistenceSeconds(
    float authoredPersistenceSeconds,
    const WaterScenarioState& state);
[[nodiscard]] float EffectiveWaterDynamicMeshPersistenceSeconds(
    float authoredPersistenceSeconds,
    const WaterDynamicMeshFlowSettings& settings,
    const WaterFeatureTimingOverlay* overlay = nullptr);
[[nodiscard]] std::string WaterDynamicMeshFlowScenarioFingerprint(
    const WaterScenarioTrack& track,
    const WaterScenarioDefinition& definition);
[[nodiscard]] WaterMeshFlowRainEnvelope BuildWaterMeshFlowRainEnvelope(
    const WaterScenarioTrack& track,
    const WaterScenarioDefinition& definition,
    float durationSeconds,
    float sampleRateHz = 120.0F,
    std::size_t maxSamples = 1'000'000U);
[[nodiscard]] WaterMeshFlowRainEnvelope BuildWaterMeshFlowRainEnvelope(
    const WaterDynamicMeshFlowSettings& settings,
    std::span<const WaterFeatureTimingRun> runs,
    float authoredRainLevel,
    float durationSeconds,
    float sampleRateHz = kWaterRainEnvelopeSampleRateHz,
    std::size_t maxSamples = kWaterRainEnvelopeMaximumSamples);
[[nodiscard]] float EvaluateWaterMeshFlowRainEnvelope(
    const WaterMeshFlowRainEnvelope& envelope,
    float timeSeconds);
[[nodiscard]] std::string WaterMeshFlowRainEnvelopeFingerprint(
    const WaterScenarioTrack& track,
    const WaterScenarioDefinition& definition,
    float durationSeconds);
[[nodiscard]] std::string WaterMeshFlowRainEnvelopeFingerprint(
    const WaterDynamicMeshFlowSettings& settings,
    std::span<const WaterFeatureTimingRun> runs,
    float authoredRainLevel,
    float durationSeconds);
[[nodiscard]] std::uint64_t WaterMeshFlowSampleTick(
    float sampleTimeSeconds,
    float fixedStepSeconds = 1.0F / 30.0F);
[[nodiscard]] std::vector<WaterMeshFlowTimelineStep>
BuildWaterMeshFlowSampleTimeline(
    std::optional<std::uint64_t> previousCompletedTick,
    float targetSampleTimeSeconds,
    std::uint32_t historyLength,
    float fixedStepSeconds = 1.0F / 30.0F);
[[nodiscard]] WaterSeepageNodeAnimationState EvaluateWaterSeepageNodeAnimationTrack(
    const WaterScenarioTrack& track,
    std::uint32_t nodeId,
    float normalizedPosition);
[[nodiscard]] std::vector<WaterSeepageNodeAnimationStateEntry>
EvaluateWaterSeepageNodeAnimationTracks(
    const WaterScenarioTrack& track,
    float normalizedPosition);
void AddOrUpdateWaterSeepageNodeKey(
    WaterScenarioTrack* track,
    std::uint32_t nodeId,
    WaterSeepageNodeKey key,
    float replacementTolerance = 0.0001F);
[[nodiscard]] WaterSeepageRainEnvelope BuildWaterSeepageRainEnvelope(
    const WaterScenarioTrack& track,
    const WaterScenarioDefinition& definition,
    float durationSeconds,
    float sampleRateHz = 120.0F,
    std::size_t maxSamples = 1'000'000U);
[[nodiscard]] WaterSeepageRainEnvelope BuildWaterSeepageNodeRainEnvelope(
    const WaterSeepageNode& node,
    std::span<const WaterFeatureTimingRun> runs,
    float authoredRainLevel,
    float durationSeconds,
    float sampleRateHz = kWaterRainEnvelopeSampleRateHz,
    std::size_t maxSamples = kWaterRainEnvelopeMaximumSamples);
[[nodiscard]] float EvaluateWaterSeepageRainEnvelope(
    const WaterSeepageRainEnvelope& envelope,
    float timeSeconds);
[[nodiscard]] std::string WaterSeepageRainEnvelopeFingerprint(
    const WaterScenarioTrack& track,
    const WaterScenarioDefinition& definition,
    float durationSeconds);
[[nodiscard]] std::string WaterSeepageNodeRainEnvelopeFingerprint(
    const WaterSeepageNode& node,
    std::span<const WaterFeatureTimingRun> runs,
    float authoredRainLevel,
    float durationSeconds);
[[nodiscard]] WaterRainEnvelopeDomain MakeWaterRainEnvelopeDomain(
    float durationSeconds,
    float sampleRateHz = kWaterRainEnvelopeSampleRateHz,
    std::size_t maxSamples = kWaterRainEnvelopeMaximumSamples);
[[nodiscard]] WaterRainResponseSettings SanitizeWaterRainResponseSettings(
    WaterRainResponseSettings settings);
[[nodiscard]] WaterRainResponseSettings ResolveWaterSeepageNodeRainResponse(
    const WaterSeepageNode& node,
    const WaterFeatureTimingOverlay* overlay = nullptr);
void ApplyWaterFeatureTimingOverlayToSeepageNode(
    const WaterFeatureTimingOverlay& overlay,
    WaterSeepageNode* node);
[[nodiscard]] float EffectiveWaterFlowActivity(
    const WaterScenarioState& state,
    float maximumFlowStrength,
    float rainResponse,
    bool sourceShowTrail = true,
    bool globalShowTrails = true);
// Timing Takes have no aggregate Flow level. The authored source strength is
// therefore the wet maximum, while Rain Response controls how much of that
// strength is rain-fed (zero keeps the authored strength constant; one fades
// from dry to the authored maximum).
[[nodiscard]] float EffectiveAuthoredWaterFlowActivity(
    float effectiveRainLevel,
    float maximumFlowStrength,
    float rainResponse,
    bool sourceShowTrail = true,
    bool globalShowTrails = true);
void AddOrUpdateWaterScenarioKey(
    WaterScenarioTrack* track,
    WaterScenarioKey key,
    float replacementTolerance = 0.0001F);
[[nodiscard]] const char* WaterTimingFeatureLabel(WaterTimingFeature feature);
[[nodiscard]] WaterTimingRun SanitizeWaterTimingRun(WaterTimingRun run);
[[nodiscard]] float EvaluateWaterTimingRun(
    const WaterTimingRun& run,
    float normalizedPosition,
    float fallbackLevel);
void AddOrUpdateWaterTimingKey(
    WaterTimingRun* run,
    WaterTimingKey key,
    float replacementTolerance = 0.0001F);
void ApplyWaterTimingLevelToScenarioState(
    WaterTimingFeature feature,
    float level,
    WaterScenarioState* state);
[[nodiscard]] float WaterTimingLevelFromScenarioState(
    WaterTimingFeature feature,
    const WaterScenarioState& state);
[[nodiscard]] std::vector<WaterScenarioKey> CompileWaterTimingScenarioKeys(
    const WaterScenarioState& baseState,
    std::span<const WaterTimingRun> runs);
[[nodiscard]] WaterSeepageQuality ResolveWaterSeepageQuality(
    WaterSeepageQuality quality,
    std::uint64_t effectivePointInvocations);
[[nodiscard]] WaterSeepageLookSettings ResolveWaterSeepageLook(
    std::span<const WaterSeepageLookProfile> profiles,
    std::span<const WaterSeepageResponseProfile> responseProfiles,
    const WaterSeepageLookSettings& defaultLook,
    std::string_view profileName,
    std::string_view responseProfileName);
[[nodiscard]] WaterSeepageLookSettings ResolveWaterSeepageLook(
    const WaterSeepageNode& node,
    std::span<const WaterSeepageLookProfile> profiles,
    std::span<const WaterSeepageResponseProfile> responseProfiles,
    const WaterSeepageLookSettings& defaultLook);
// Scalar timing keys need a complete transient look as their base. The node's
// fully resolved authored settings/response pair always wins; the scenario
// argument is retained only for source compatibility with legacy callers.
[[nodiscard]] WaterSeepageLookSettings ResolveWaterSeepageTimingLookBase(
    const WaterSeepageLookSettings& resolvedAuthoredLook,
    const std::optional<WaterScenarioState>& scenarioState);
[[nodiscard]] std::string WaterSeepageLocalLookName(std::string_view baseName, std::uint32_t nodeId);
[[nodiscard]] invisible_places::io::Float3 DeriveWaterSeepageDownAxis(
    const invisible_places::io::Float3& surfaceNormal,
    const invisible_places::io::Float3& fallbackAxis = {0.0F, 0.0F, -1.0F});
[[nodiscard]] WaterSeepageSpatialGrid BuildWaterSeepageSpatialGrid(
    std::span<const WaterSeepageNode> nodes,
    std::span<const WaterSeepageLookProfile> profiles,
    const WaterSeepageLookSettings& defaultLook,
    std::string_view targetSceneRole,
    bool forExport,
    const WaterRainSettings& rainSettings,
    std::uint64_t effectivePointInvocations,
    std::span<const WaterSeepageSurfaceGuide> guides = {},
    const std::optional<WaterScenarioState>& scenarioState = std::nullopt,
    std::span<const WaterSeepageNodeAnimationStateEntry> nodeAnimationStates = {},
    std::span<const WaterSeepageSupportSelection> supportSelections = {},
    // Appended (rather than beside `profiles`) so existing positional
    // callers keep compiling; empty means every node uses the default
    // response.
    std::span<const WaterSeepageResponseProfile> responseProfiles = {});
[[nodiscard]] WaterSeepageSupportBuildResult BuildWaterSeepageSupportSelection(
    const WaterSeepageNode& node,
    std::string_view targetSceneRole,
    const WaterSurfaceCache& surfaceCache,
    const WaterSeepageSupportBuildOptions& options = {});
// Builds independent node selections concurrently while preserving authored
// result order. The worker count is deliberately bounded: each Dijkstra owns
// temporary visited/queued maps, so unrestricted fan-out can multiply memory
// pressure when a full-site cache contains many authored nodes.
[[nodiscard]] std::vector<WaterSeepageSupportBuildResult>
BuildWaterSeepageSupportSelections(
    std::span<const WaterSeepageNode> nodes,
    std::string_view targetSceneRole,
    const WaterSurfaceCache& surfaceCache,
    const WaterSeepageSupportBuildOptions& options = {},
    std::size_t maximumParallelBuilds = 3U);
// A failed/cancelled/capped candidate never replaces the last settled
// selection. This explicit commit seam keeps asynchronous callers atomic.
[[nodiscard]] bool CommitWaterSeepageSupportSelection(
    const WaterSeepageSupportBuildResult& candidate,
    WaterSeepageSupportSelection* settledSelection);
// The live membership weight of one selected support cell — the exact mask
// the renderer applies (minus the per-point normal-agreement term), so the
// Structure Overlay can show the truly affected area.
[[nodiscard]] float EvaluateWaterSeepageSupportCellMask(
    const WaterSeepageRuntimeNode& node,
    const WaterSeepageSupportCell& cell);
void ApplyWaterSeepageRuntimeParameters(
    WaterSeepageSpatialGrid* grid,
    std::span<const WaterSeepageNode> nodes,
    std::span<const WaterSeepageLookProfile> profiles,
    const WaterSeepageLookSettings& defaultLook,
    const std::optional<WaterScenarioState>& scenarioState,
    const WaterRainSettings& rainSettings,
    std::uint64_t effectivePointInvocations,
    std::span<const WaterSeepageNodeAnimationStateEntry> nodeAnimationStates = {},
    std::span<const WaterSeepageResponseProfile> responseProfiles = {});
void ApplyWaterSeepageScenarioParameters(
    WaterSeepageSpatialGrid* grid,
    const std::optional<WaterScenarioState>& scenarioState,
    const WaterRainSettings& rainSettings,
    std::uint64_t effectivePointInvocations,
    std::span<const WaterSeepageNodeAnimationStateEntry> nodeAnimationStates = {});
// Prepares the compact Contour Pulse reference samples from authored shape
// settings (unchanged inputs are reused). sampleTimeSeconds is retained for
// source compatibility but deliberately does not participate in preparation:
// rendering advances the reference field from its own frame time. Preparation
// changes no support, descriptors, point data, or topology.
void PrepareWaterSeepagePulseFields(
    WaterSeepageSpatialGrid* grid,
    float sampleTimeSeconds);
[[nodiscard]] WaterSeepageRuntimeContribution EvaluateWaterSeepageRuntimeContribution(
    const WaterSeepageRuntimeNode& node,
    const invisible_places::io::Float3& position,
    const invisible_places::io::Float3& normal,
    float timeSeconds,
    const WaterSeepageViewContext& viewContext = {});
[[nodiscard]] WaterSeepageRuntimeContribution EvaluateWaterSeepageGridContribution(
    const WaterSeepageSpatialGrid& grid,
    const invisible_places::io::Float3& position,
    const invisible_places::io::Float3& normal,
    float timeSeconds,
    const WaterSeepageViewContext& viewContext = {});
// Stable authored identity for the compact topology of one terrain role.
// Live look, visibility, strength, seed, and animation parameters are
// deliberately excluded so they remain parameter-ring updates.
[[nodiscard]] std::string WaterSeepageAuthoredTopologyFingerprint(
    std::span<const WaterSeepageNode> nodes,
    std::string_view targetSceneRole);
[[nodiscard]] bool WaterSeepageGridHasActiveViewportEffect(
    const WaterSeepageSpatialGrid& grid);
// True when the look's GPU pattern output depends on the shader clock, i.e.
// re-rendering at a later time can produce different pixels. Static looks
// (all motion parameters zero) render identically at any time, so a cached
// scene image stays correct until a parameter edit re-uploads the grid.
[[nodiscard]] bool WaterSeepageLookIsTimeAnimating(
    const WaterSeepageLookSettings& look);
// WaterSeepageGridHasActiveViewportEffect restricted to nodes whose
// contributing look is also time-animating: only those need a per-frame
// scene redraw, everything else is covered by the params-fingerprint
// invalidation path.
[[nodiscard]] bool WaterSeepageGridHasTimeAnimatingViewportEffect(
    const WaterSeepageSpatialGrid& grid);
[[nodiscard]] std::string WaterSeepageTopologyFingerprint(const WaterSeepageSpatialGrid& grid);
[[nodiscard]] std::string WaterSeepageParamsFingerprint(const WaterSeepageSpatialGrid& grid);

struct WaterTrailSample {
    invisible_places::io::Float3 position{};
    invisible_places::io::Float3 normal{0.0F, 0.0F, 1.0F};
    invisible_places::io::Float3 tangent{1.0F, 0.0F, 0.0F};
    std::uint8_t red = 40;
    std::uint8_t green = 210;
    std::uint8_t blue = 255;
    float trailId = 0.0F;
    float sourceId = 0.0F;
    float pathId = 0.0F;
    float branchId = 0.0F;
    float trailSeed = 0.0F;
    float pointSeed = 0.0F;
    float trailDistance = 0.0F;
    float trailLength = 1.0F;
    float pointAge = 0.0F;
    float trailAge = 0.0F;
    float trailSpeed = 1.0F;
    float trailWidth = 0.006F;
    float trailStreakLength = 0.045F;
    float trailConfidence = 1.0F;
    float wetness = 1.0F;
    float featureType = 0.0F;
    float trailRole = 1.0F;
    float routeStartIndex = 0.0F;
    float routePointCount = 0.0F;
    float routeLength = 1.0F;
    float trailStartPhase = 0.0F;
    float trailLateralOffset = 0.0F;
    float trailLaneIndex = 0.0F;
    float trailLaneCount = 1.0F;
    float trailLanePitch = 0.00025F;
    float trailLaneSpan = 0.0F;
    float trailLaneCrossing = 0.22F;
    float trailCrossSeed = 0.0F;
    float endpointFadeFlags = 0.0F;
    float startFadeFullDistanceMeters = 0.0F;
    float startFadeRandomBeginDistanceMeters = 0.0F;
    float endFadeFullDistanceMeters = 0.0F;
    float endFadeRandomBeginDistanceMeters = 0.0F;
};

struct WaterTrailOverlay {
    std::vector<WaterTrailSample> samples;
    invisible_places::io::Bounds3f bounds{};
};

struct WaterParticleTrailSettings {
    float particleDensity = 1.0F;
    float particleJitter = 0.35F;
    float particleSpeed = 1.0F;
    float splineAnchorSpacing = 0.5F;
};

struct WaterParticleVisualSettings {
    float particleSizePixels = 14.0F;
    float particleOpacity = 0.24F;
    float colorVariation = 0.65F;
    float glow = 0.35F;
};

struct WaterSourceSettings {
    WaterPathGenerationSettings path{};
    WaterParticleTrailShapeSettings trailShape{};
};

using WaterVisualSettings = WaterParticleVisualSettings;

struct WaterSettingsBundle {
    WaterPathGenerationSettings path{};
    WaterParticleTrailSettings trail{};
    WaterParticleVisualSettings visual{};
};

using WaterBakeSettings = WaterPathGenerationSettings;
using WaterRenderSettings = WaterSettingsBundle;

struct WaterEmitter {
    std::uint32_t id = 0;
    std::string name = "Water Source";
    invisible_places::io::Float3 position{};
    float radius = 0.35F;
    float strength = 1.0F;
    float speed = 1.0F;
    WaterScaleMode scope = WaterScaleMode::Mid;
    WaterEmitterOrigin origin = WaterEmitterOrigin::Manual;
    WaterEmitterStatus status = WaterEmitterStatus::Accepted;
    float confidence = 1.0F;
    std::optional<std::uint32_t> parentId;
    WaterSourceSettingsAssignment sourceSettingsAssignment = WaterSourceSettingsAssignment::Default;
    std::optional<std::uint32_t> linkedSourceSettingsEmitterId;
    std::optional<WaterSourceSettings> sourceSettings;
    std::optional<WaterSourceSettings> tempSourceSettings;
    std::string pathProfileName = "Global";
    std::string laneProfileName = "Global";
    std::string trailProfileName = "Global";
    // Locked assignments resolve the saved profile instead of its matching temporary edit.
    bool pathProfileLocked = false;
    bool laneProfileLocked = false;
    bool trailProfileLocked = false;
    float maximumFlowStrength = 1.0F;
    float rainResponse = 0.0F;
    bool showTrail = true;
};

struct WaterManualFlowPathSource {
    std::uint32_t id = 0;
    std::string name = "Path Source";
    std::vector<invisible_places::io::Float3> controlPoints;
    // Parallel to controlPoints. Missing legacy entries inherit the global
    // Lane Cover Width and are normalized by editors/serialization on write.
    std::vector<WaterManualFlowPathLaneWidth> controlPointLaneWidths;
    std::string laneProfileName = "Global";
    std::string trailProfileName = "Global";
    // Manual paths share the same saved-versus-live profile assignment rule.
    bool laneProfileLocked = false;
    bool trailProfileLocked = false;
    // New sources opt in. Project/source schema migration disables this for
    // legacy manual paths until the user explicitly enables it.
    bool useSurfaceGuide = true;
    // Empty means the source owns only its live timeline tracks. A saved name
    // applies an immutable project profile; an `_edited` name references the
    // object-owned working shadow created on the first keyed edit.
    std::string keyedSettingsProfileName;
    float maximumFlowStrength = 1.0F;
    float rainResponse = 0.0F;
    bool showTrail = true;
};

enum class WaterFlowProfileKind : std::uint8_t {
    Path = 0,
    Lane,
    Trail,
};

// Exact by-name assignment replacements are reported separately for point
// and manual-path sources because their ids share one allocator but their
// runtime invalidation paths differ. Lock state is deliberately untouched.
struct WaterFlowProfileAssignmentRewrite {
    std::vector<std::uint32_t> pointSourceIds;
    std::vector<std::uint32_t> manualPathSourceIds;
    bool dynamicMeshTrailChanged = false;
    // True only when the supplied mesh edit shadow (or its `_edited`
    // assignment) derives from the replaced Trail profile. The Application
    // may then safely discard that shadow without touching an unrelated edit.
    bool dynamicMeshEditedTrailProfileMatched = false;

    [[nodiscard]] bool changed() const {
        return !pointSourceIds.empty() ||
               !manualPathSourceIds.empty() ||
               dynamicMeshTrailChanged;
    }
};

// Rebinds one Flow profile name through every persisted source assignment.
// Matching trims surrounding whitespace and treats a blank source assignment
// as Global, mirroring runtime profile resolution. Empty previous/replacement
// names are rejected so a cleanup transaction cannot become ambiguous or
// create a dangling assignment.
[[nodiscard]] WaterFlowProfileAssignmentRewrite
ReplaceWaterFlowProfileAssignments(
    std::span<WaterEmitter> emitters,
    std::span<WaterManualFlowPathSource> manualPaths,
    WaterDynamicMeshFlowSettings* dynamicMesh,
    WaterFlowProfileKind kind,
    std::string_view previousProfileName,
    std::string_view nextProfileName,
    std::string_view dynamicMeshEditedTrailProfileName = {});

struct WaterSceneSupportLayer {
    const invisible_places::io::LoadedPointCloud* cloud = nullptr;
    std::string role;
    float pointSpacingMeters = 0.0F;
    float samplingMultiplier = 1.0F;
};

struct WaterOverlayPoint {
    invisible_places::io::Float3 position{};
    invisible_places::io::Float3 normal{0.0F, 0.0F, 1.0F};
    std::uint8_t red = 40;
    std::uint8_t green = 210;
    std::uint8_t blue = 255;
    float flowId = 0.0F;
    float emitterId = 0.0F;
    float pathDistance = 0.0F;
    float phase = 0.0F;
    float speed = 1.0F;
    float width = 1.0F;
    // Negative means use the global Lane Cover Width. Manual spline anchors
    // carry a resolved local width in metres so the CPU reference follows the
    // same per-node lane envelope as the GPU route pass.
    float laneSpanMeters = -1.0F;
    float confidence = 1.0F;
    float accumulation = 0.0F;
    float pooling = 0.0F;
    float particleRole = 0.0F;
    float pathStartIndex = 0.0F;
    float pathPointCount = 0.0F;
    float jitterSeed = 0.0F;
    float trailAge = 0.0F;
    float trailLength = 0.0F;
    float featureType = 0.0F;
    float regionId = 0.0F;
    float surfaceSteepness = 0.0F;
    float trailLaneId = 0.0F;
    float trailLateralOffset = 0.0F;
};

struct WaterOverlay {
    std::vector<WaterOverlayPoint> points;
    invisible_places::io::Bounds3f bounds{};
};

enum class WaterAnimatedTrailMotionMode {
    Path,
    VectorField
};

struct WaterAnimatedTrailPath {
    WaterAnimatedTrailMotionMode motionMode = WaterAnimatedTrailMotionMode::Path;
    std::uint32_t sourceId = 0;
    std::vector<WaterOverlayPoint> anchors;
};

struct WaterAnimatedTrailBuildSettings {
    std::uint32_t trailCountTotal = 700;
    std::uint32_t laneCount = 0;
    float trailLengthMeters = 0.75F;
    float trailPointSpacingMeters = 0.010F;
    float trailWidthMeters = 0.006F;
    float trailStreakLengthMeters = 0.045F;
    bool startFadeEnabled = false;
    float startFadeFullDistanceMeters = 0.25F;
    float startFadeRandomBeginDistanceMeters = 0.10F;
    bool endFadeEnabled = false;
    float endFadeFullDistanceMeters = 0.25F;
    float endFadeRandomBeginDistanceMeters = 0.10F;
    float surfaceOffsetMeters = 0.004F;
    float pathAttraction = 0.85F;
    float laneSpreadMeters = 0.12F;
    float turbulence = 0.06F;
    float laneCrossing = 0.22F;
    float trailSmoothness = 0.85F;
    float trailLooseness = 0.08F;
    float surfaceFollow = 0.85F;
    float downhillPull = 0.35F;
    float terrainWidthResponse = 0.65F;
    float turbulenceScaleMeters = 0.18F;
    float speedMetersPerSecond = 0.45F;
    std::uint32_t seed = 1;
    float featureType = 0.0F;
};

enum class WaterTrailBuildQuality {
    Preview,
    Final
};

struct WaterTrailBuildDiagnostics {
    double surfaceIndexBuildMs = 0.0;
    double routeMs = 0.0;
    double laneMs = 0.0;
    double particleMs = 0.0;
    std::uint64_t surfaceSampleCount = 0;
    std::uint32_t routedPathCount = 0;
    std::uint32_t emittedLaneCount = 0;
    std::uint32_t emittedParticleCount = 0;
};

struct TrailSurfaceIndexCell {
    invisible_places::io::Float3 position{};
    invisible_places::io::Float3 normal{0.0F, 0.0F, 1.0F};
    float minZ = 0.0F;
    float maxZ = 0.0F;
    float confidence = 1.0F;
    std::uint32_t count = 0;
    bool hasNormal = false;
};

struct TrailSurfaceIndex {
    std::vector<TrailSurfaceIndexCell> cells;
    std::unordered_map<std::uint64_t, std::uint32_t> cellLookup;
    float cellSize = 0.05F;
    float searchRadius = 0.12F;
    float surfaceLift = 0.004F;
    std::uint64_t sampledPointCount = 0;
    double buildMilliseconds = 0.0;
};

enum class WaterPathBranchRole {
    Main,
    Secondary,
    Spread
};

enum class WaterPathTerminationReason {
    ReachedLength,
    NoSupport,
    MaxSteps,
    Loop,
    Duplicate,
    Empty
};

struct WaterPathAutoTuneDiagnostics {
    float estimatedPointSpacing = 0.0F;
    float supportVoxelSize = 0.0F;
    float maxBridgeDistance = 0.0F;
    float pathSampleSpacing = 0.0F;
    float branchSearchRadius = 0.0F;
    float averageConfidence = 0.0F;
    std::uint32_t iterationCount = 0;
    std::uint32_t pilotTraceCount = 0;
    std::uint32_t branchCount = 0;
    std::uint32_t lowConfidenceBranchCount = 0;
    std::string summary;
};

struct WaterPathBranch {
    std::uint32_t id = 0;
    std::optional<std::uint32_t> parentId;
    std::uint32_t emitterId = 0;
    WaterPathBranchRole role = WaterPathBranchRole::Main;
    WaterPathTerminationReason terminationReason = WaterPathTerminationReason::Empty;
    float confidence = 1.0F;
    float length = 0.0F;
    float flatness = 0.0F;
    std::uint32_t gapCount = 0;
    std::string bakeFingerprint;
    std::vector<WaterOverlayPoint> rawAnchors;
};

struct WaterPathAnalysisSample {
    std::uint32_t branchId = 0;
    std::uint32_t sampleIndex = 0;
    float pathDistance = 0.0F;
    float slope = 0.0F;
    float flatness = 0.0F;
    float curvature = 0.0F;
    float neighborDensity = 0.0F;
    float nearestPathDistance = 0.0F;
    float confluence = 0.0F;
    float channelWidth = 0.0F;
    float speed = 0.0F;
    float turbulence = 0.0F;
    float eddyPotential = 0.0F;
    float ripplePotential = 0.0F;
};

struct WaterPathBranchAnalysis {
    std::uint32_t branchId = 0;
    std::vector<WaterPathAnalysisSample> samples;
};

struct WaterPathAnalysisCache {
    std::uint32_t schemaVersion = 1;
    float analysisRadiusMeters = 0.0F;
    std::vector<WaterPathBranchAnalysis> branches;
};

struct WaterPathCache {
    std::uint32_t schemaVersion = 2;
    std::filesystem::path supportLayerPath;
    std::string supportSignature;
    std::string emitterSettingsFingerprint;
    WaterPathGenerationSettings requestedSettings{};
    WaterPathGenerationSettings tunedSettings{};
    WaterPathAutoTuneDiagnostics diagnostics{};
    std::vector<WaterPathBranch> branches;
    std::vector<std::uint32_t> hiddenBranchIds;
    std::optional<WaterPathAnalysisCache> analysis;
    bool stale = false;
};

[[nodiscard]] const char* WaterScaleModeName(WaterScaleMode mode);
[[nodiscard]] const char* WaterEmitterOriginName(WaterEmitterOrigin origin);
[[nodiscard]] const char* WaterEmitterStatusName(WaterEmitterStatus status);
[[nodiscard]] WaterPathGenerationSettings DefaultWaterPathGenerationSettings(WaterScaleMode mode = WaterScaleMode::Mid);
[[nodiscard]] WaterSourceSettings DefaultWaterSourceSettings(WaterScaleMode mode = WaterScaleMode::Mid);
[[nodiscard]] WaterAnimationTrailSettings DefaultWaterAnimationTrailSettings();
[[nodiscard]] WaterVisualSettings DefaultWaterVisualSettings();
[[nodiscard]] WaterSettingsBundle DefaultWaterSettingsBundle(WaterScaleMode mode = WaterScaleMode::Mid);
[[nodiscard]] WaterBakeSettings DefaultWaterBakeSettings(WaterScaleMode mode);
[[nodiscard]] bool WaterPathBakeInputsEqual(
    const WaterPathGenerationSettings& left,
    const WaterPathGenerationSettings& right);
[[nodiscard]] bool WaterSourceBakeInputsEqual(
    const WaterSourceSettings& left,
    const WaterSourceSettings& right);
[[nodiscard]] const WaterSourceSettings& ResolveWaterSourceSettings(
    const WaterEmitter& emitter,
    const WaterSourceSettings& defaultSettings);
[[nodiscard]] const WaterSourceSettings& ResolveWaterSourceSettings(
    const WaterEmitter& emitter,
    const std::vector<WaterEmitter>& emitters,
    const WaterSourceSettings& defaultSettings);

[[nodiscard]] std::vector<WaterEmitter> SuggestWaterEmitters(
    const invisible_places::io::LoadedPointCloud& cloud,
    const std::vector<WaterEmitter>& existingEmitters,
    const WaterPathGenerationSettings& settings,
    std::uint32_t firstEmitterId,
    std::uint32_t maxSuggestions);

[[nodiscard]] std::optional<invisible_places::io::Float3> SnapEmitterToCloud(
    const invisible_places::io::LoadedPointCloud& cloud,
    const invisible_places::io::Float3& position,
    const WaterPathGenerationSettings& settings);

[[nodiscard]] WaterOverlay GenerateWaterPathAnchors(
    const invisible_places::io::LoadedPointCloud& cloud,
    const std::vector<WaterEmitter>& emitters,
    const WaterPathGenerationSettings& settings);

[[nodiscard]] WaterOverlay GenerateWaterPathAnchors(
    const invisible_places::io::LoadedPointCloud& cloud,
    const std::vector<WaterEmitter>& emitters,
    const WaterSourceSettings& defaultSettings);

[[nodiscard]] WaterPathCache GenerateWaterPathCache(
    const invisible_places::io::LoadedPointCloud& cloud,
    const std::vector<WaterEmitter>& emitters,
    const WaterPathGenerationSettings& settings);

[[nodiscard]] WaterPathCache GenerateWaterPathCache(
    const invisible_places::io::LoadedPointCloud& cloud,
    const std::vector<WaterEmitter>& emitters,
    const WaterSourceSettings& defaultSettings);

[[nodiscard]] invisible_places::io::LoadedPointCloud BuildCombinedWaterSupportCloud(
    std::span<const WaterSceneSupportLayer> layers,
    const WaterPathGenerationSettings& settings);
[[nodiscard]] invisible_places::io::LoadedPointCloud BuildCombinedWaterSupportCloud(
    std::span<const WaterSceneSupportLayer> layers,
    const WaterPathGenerationSettings& settings,
    std::span<const invisible_places::io::Float3> priorityPoints);

[[nodiscard]] std::vector<WaterSeepageSurfaceGuide> BuildWaterSeepageSurfaceGuides(
    std::span<const WaterSeepageNode> nodes,
    std::span<const WaterSceneSupportLayer> supportLayers,
    std::size_t sampleLimit = 300'000U);
[[nodiscard]] std::vector<WaterSeepageSurfaceGuide> BuildWaterSeepageSurfaceGuides(
    std::span<const WaterSeepageNode> nodes,
    const WaterSurfaceCache& surfaceCache);

[[nodiscard]] WaterPathAnalysisCache BuildWaterPathAnalysis(const WaterPathCache& cache);
[[nodiscard]] bool WaterPathAnalysisCacheCompatible(const WaterPathCache& cache);
void EnsureWaterPathAnalysis(WaterPathCache* cache);

[[nodiscard]] std::shared_ptr<const TrailSurfaceIndex> BuildTrailSurfaceIndex(
    const invisible_places::io::LoadedPointCloud& cloud);
[[nodiscard]] std::shared_ptr<const TrailSurfaceIndex> BuildTrailSurfaceIndex(
    const invisible_places::io::LoadedPointCloud* cloud);
[[nodiscard]] std::uint64_t TrailSurfaceIndexSampleCount(const TrailSurfaceIndex& index);
[[nodiscard]] double TrailSurfaceIndexBuildMilliseconds(const TrailSurfaceIndex& index);

[[nodiscard]] WaterOverlay BuildWaterPathAnchorsFromCache(
    const WaterPathCache& cache,
    const std::vector<WaterEmitter>& emitters,
    const WaterSourceSettings& defaultSettings);

[[nodiscard]] WaterOverlay BuildWaterOverlayFromPathAnchors(
    const WaterOverlay& pathAnchors,
    const WaterParticleTrailShapeSettings& trailShapeSettings,
    const WaterAnimationTrailSettings& animationTrailSettings);

[[nodiscard]] WaterOverlay BuildWaterOverlayFromPathAnchors(
    const WaterOverlay& pathAnchors,
    const WaterParticleTrailShapeSettings& trailShapeSettings,
    const WaterAnimationTrailSettings& animationTrailSettings,
    const invisible_places::io::LoadedPointCloud* supportCloud,
    WaterTrailBuildQuality quality = WaterTrailBuildQuality::Final,
    WaterTrailBuildDiagnostics* diagnostics = nullptr);

[[nodiscard]] WaterOverlay BuildWaterOverlayFromPathAnchors(
    const WaterOverlay& pathAnchors,
    const WaterParticleTrailShapeSettings& trailShapeSettings,
    const WaterAnimationTrailSettings& animationTrailSettings,
    const TrailSurfaceIndex* surfaceIndex,
    WaterTrailBuildQuality quality = WaterTrailBuildQuality::Final,
    WaterTrailBuildDiagnostics* diagnostics = nullptr);

[[nodiscard]] WaterOverlay BuildWaterOverlayFromPathAnchors(
    const WaterOverlay& pathAnchors,
    const WaterParticleTrailSettings& legacyTrailSettings,
    const WaterParticleVisualSettings& legacyVisualSettings);

[[nodiscard]] WaterOverlay BuildWaterOverlayFromPathAnchors(
    const WaterOverlay& pathAnchors,
    const std::vector<WaterEmitter>& emitters,
    const WaterSourceSettings& defaultSettings,
    const WaterAnimationTrailSettings& animationTrailSettings);

[[nodiscard]] WaterOverlay BuildWaterOverlayFromPathAnchors(
    const WaterOverlay& pathAnchors,
    const std::vector<WaterEmitter>& emitters,
    const WaterSourceSettings& defaultSettings,
    const WaterAnimationTrailSettings& animationTrailSettings,
    const invisible_places::io::LoadedPointCloud* supportCloud,
    WaterTrailBuildQuality quality = WaterTrailBuildQuality::Final,
    WaterTrailBuildDiagnostics* diagnostics = nullptr);

[[nodiscard]] WaterOverlay BuildWaterOverlayFromPathAnchors(
    const WaterOverlay& pathAnchors,
    const std::vector<WaterEmitter>& emitters,
    const WaterSourceSettings& defaultSettings,
    const WaterAnimationTrailSettings& animationTrailSettings,
    const TrailSurfaceIndex* surfaceIndex,
    WaterTrailBuildQuality quality = WaterTrailBuildQuality::Final,
    WaterTrailBuildDiagnostics* diagnostics = nullptr);

[[nodiscard]] WaterOverlay GenerateWaterOverlay(
    const invisible_places::io::LoadedPointCloud& cloud,
    const std::vector<WaterEmitter>& emitters,
    const WaterSourceSettings& defaultSourceSettings,
    const WaterAnimationTrailSettings& animationTrailSettings);

[[nodiscard]] WaterOverlay GenerateWaterOverlay(
    const invisible_places::io::LoadedPointCloud& cloud,
    const std::vector<WaterEmitter>& emitters,
    const WaterSettingsBundle& settings);
[[nodiscard]] WaterTrailOverlay BuildAnimatedWaterTrailOverlay(
    const std::vector<WaterAnimatedTrailPath>& paths,
    const WaterAnimatedTrailBuildSettings& settings);
[[nodiscard]] WaterTrailOverlay BuildFlowTrailOverlayFromPathAnchors(
    const WaterOverlay& pathAnchors,
    const WaterFlowTrailSettings& settings);
[[nodiscard]] WaterTrailOverlay BuildFlowTrailOverlayFromPathAnchors(
    const WaterOverlay& pathAnchors,
    const WaterFlowTrailSettings& settings,
    const WaterPathAnalysisCache* analysis);
[[nodiscard]] WaterTrailOverlay BuildFlowTrailOverlayFromPathAnchors(
    const WaterOverlay& pathAnchors,
    const WaterFlowTrailSettings& settings,
    const WaterPathAnalysisCache* analysis,
    const WaterFlowTrailBuildOptions& options);
[[nodiscard]] WaterOverlay BuildManualFlowPathAnchors(
    const WaterManualFlowPathSource& source,
    float sampleSpacingMeters = 0.025F,
    float globalLaneSpanMeters = 0.12F);
[[nodiscard]] float ResolveWaterManualFlowPathLaneWidth(
    const WaterManualFlowPathLaneWidth& laneWidth,
    float globalLaneSpanMeters);
// Applies a viewport cross-bar drag without changing an existing override's
// mode. A Standard node becomes Relative on its first customization; when the
// global width is zero, Absolute is the only representation of a non-zero
// customized width.
[[nodiscard]] WaterManualFlowPathLaneWidth
ApplyWaterManualFlowPathLaneWidthHandleDrag(
    const WaterManualFlowPathLaneWidth& laneWidth,
    float resolvedWidthMeters,
    float globalLaneSpanMeters);
[[nodiscard]] MeshSurfaceCache BuildMeshSurfaceCache(
    const invisible_places::io::LoadedTriangleMesh& mesh,
    const WaterDynamicMeshFlowSettings& settings);
[[nodiscard]] MeshSurfaceProjection ProjectToMeshSurface(
    const MeshSurfaceCache& cache,
    const invisible_places::io::Float3& position);
[[nodiscard]] MeshSurfaceProjection ProjectRayToMeshSurface(
    const MeshSurfaceCache& cache,
    const invisible_places::io::Float3& rayOrigin,
    const invisible_places::io::Float3& rayDirection);
[[nodiscard]] WaterTrailOverlay BuildDynamicMeshWaterTrailOverlay(
    const MeshSurfaceCache& cache,
    const std::vector<WaterEmitter>& emitters,
    const WaterDynamicMeshFlowSettings& settings,
    WaterTrailBuildQuality quality = WaterTrailBuildQuality::Final,
    WaterDynamicMeshFlowDiagnostics* diagnostics = nullptr);
[[nodiscard]] invisible_places::io::LoadedPointCloud BuildWaterTrailOverlayPointCloud(
    const WaterTrailOverlay& overlay,
    const std::filesystem::path& sourcePath,
    std::string_view layerName);
[[nodiscard]] invisible_places::io::LoadedPointCloud BuildWaterTrailOverlayPointCloud(
    const WaterTrailOverlay& overlay,
    const std::filesystem::path& sourcePath,
    std::string_view layerName,
    const std::stop_token* stopToken);
[[nodiscard]] std::vector<invisible_places::io::ScalarFieldStats> WaterTrailOverlayScalarFieldsForPointCount(
    std::uint64_t pointCount,
    bool includeEndpointFadeFields = false);
[[nodiscard]] invisible_places::io::LoadedPointCloud BuildWaterOverlayPointCloud(
    const WaterOverlay& overlay,
    const std::filesystem::path& sourcePath,
    std::string_view layerName);
bool WriteWaterOverlayPly(
    const WaterOverlay& overlay,
    const std::filesystem::path& outputPath,
    std::string* errorMessage);

}  // namespace invisible_places::water
