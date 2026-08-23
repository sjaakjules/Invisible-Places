#pragma once

#include "water/WaterFlow.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace invisible_places::timing {

inline constexpr std::string_view kAuthoredTimingTakeId = "authored-timing";
inline constexpr std::string_view kAuthoredTimingTakeName = "Authored Timing";
// Authoring remains unrestricted. This is only the shared responsiveness
// recommendation for how many effects should overlap at one animation time.
inline constexpr std::size_t kTimingColouriseSoftActiveEffectLimit = 5U;
inline constexpr std::size_t kMaximumTimingColourisePaletteStops = 8U;
inline constexpr std::size_t kTimingColouriseLutSampleCount = 64U;
inline constexpr float kTimingColouriseKeyTolerance = 1.0e-4F;

enum class TimingColouriseFieldSource : std::uint8_t {
    Scalar = 0,
    NormalX,
    NormalY,
    NormalZ,
};

struct TimingColouriseFieldSelector {
    TimingColouriseFieldSource source = TimingColouriseFieldSource::Scalar;
    // Stable, exact field name used by each point-cloud layer. Synthetic
    // normal components leave this empty.
    std::string scalarFieldName;

    friend auto operator<=>(
        const TimingColouriseFieldSelector&,
        const TimingColouriseFieldSelector&) = default;
};

struct TimingColourisePaletteStop {
    // Stable within one palette/effect. Animation tracks refer to this id so
    // sorting or moving stops never exchanges their authored properties.
    std::string id;
    float position = 0.0F;
    std::array<float, 3> colour{1.0F, 1.0F, 1.0F};
    // Mix amount for the sampled colour. This never changes point opacity.
    float colouriseAmount = 1.0F;
};

struct TimingColourisePalette {
    std::vector<TimingColourisePaletteStop> stops{
        TimingColourisePaletteStop{}};
};

// An effect-local alternative derived from a built-in preset. It is not part
// of the shared project palette library and remains owned by one Colourise
// effect while that effect switches between presets.
struct TimingColouriseLocalPaletteEdit {
    std::string presetId;
    std::string presetName;
    TimingColourisePalette palette{};
};

// Whole-palette snapshots are retained for projects authored before
// independent stop-property keying. New effects use StopParameters: their
// stop topology is static and each stop property owns its own curve.
enum class TimingColourisePaletteKeyModel : std::uint8_t {
    LegacySnapshots = 0,
    StopParameters,
};

enum class TimingColourisePaletteSourceKind : std::uint8_t {
    Custom = 0,
    Preset,
    Saved,
};

enum class TimingColouriseAmountOverrideMode : std::uint8_t {
    // Clamp every sampled stop amount to the authored override ceiling.
    Maximum = 0,
    // Multiply every sampled stop amount by the authored override value.
    Scale,
};

// Legacy discriminator retained only for parsing documents written before
// Visual Features carried independent colourise/emissive aspects. Runtime
// effects no longer store a kind; they enable either or both aspects.
enum class TimingEffectKind : std::uint8_t {
    Colourise = 0,
    Emissive,
};

enum class TimingColouriseEffectParameter : std::uint8_t {
    PalettePhase = 0,
    AmountOverride,
    EmissiveLevel,
};

enum class TimingColourisePaletteStopParameter : std::uint8_t {
    Position = 0,
    Colour,
    ColouriseAmount,
};

using TimingColouriseLut =
    std::array<std::array<float, 4>, kTimingColouriseLutSampleCount>;

struct TimingColouriseBounds {
    float lower = 0.0F;
    float upper = 1.0F;
    // Signed fraction of the selected span faded at each edge, in [-0.5,
    // 0.5]. Positive values fade inward; negative values fade outward.
    float edgeFade = 0.10F;
};

// An effect chooses exactly one bounds parameterisation. This makes the two
// geometric controls unambiguous while still allowing either control to be
// keyed independently.
enum class TimingColouriseBoundsKeyMode : std::uint8_t {
    LowerUpper = 0,
    CentreSpread,
    LowerSpread,
    UpperSpread,
};

enum class TimingColouriseBoundsParameter : std::uint8_t {
    Lower = 0,
    Upper,
    Centre,
    Spread,
    EdgeFade,
};

enum class TimingColouriseBoundsHandle : std::uint8_t {
    Lower = 0,
    Upper,
    Centre,
};

struct TimingColouriseBoundsHandleEdit {
    TimingColouriseBounds bounds{};
    std::array<TimingColouriseBoundsParameter, 2> parameters{
        TimingColouriseBoundsParameter::EdgeFade,
        TimingColouriseBoundsParameter::EdgeFade};
    std::size_t parameterCount = 0U;
};

struct TimingColouriseBoundsParameterKey {
    TimingColouriseBoundsParameter parameter =
        TimingColouriseBoundsParameter::Lower;
    float position = 0.0F;
    float value = 0.0F;
    invisible_places::water::WaterScenarioInterpolation interpolation =
        invisible_places::water::WaterScenarioInterpolation::Smooth;
};

struct TimingColouriseEffectParameterKey {
    TimingColouriseEffectParameter parameter =
        TimingColouriseEffectParameter::PalettePhase;
    float position = 0.0F;
    float value = 0.0F;
    invisible_places::water::WaterScenarioInterpolation interpolation =
        invisible_places::water::WaterScenarioInterpolation::Smooth;
};

struct TimingColourisePaletteKey {
    float position = 0.0F;
    TimingColourisePalette palette{};
    invisible_places::water::WaterScenarioInterpolation interpolation =
        invisible_places::water::WaterScenarioInterpolation::Smooth;
};

struct TimingColourisePaletteStopParameterKey {
    std::string stopId;
    TimingColourisePaletteStopParameter parameter =
        TimingColourisePaletteStopParameter::Position;
    // Normalized animation position, not the stop's position in the palette.
    float position = 0.0F;
    // Position and ColouriseAmount use scalarValue; Colour uses colourValue.
    float scalarValue = 0.0F;
    std::array<float, 3> colourValue{1.0F, 1.0F, 1.0F};
    invisible_places::water::WaterScenarioInterpolation interpolation =
        invisible_places::water::WaterScenarioInterpolation::Smooth;
};

struct TimingColouriseBoundsKey {
    float position = 0.0F;
    TimingColouriseBounds bounds{};
    invisible_places::water::WaterScenarioInterpolation interpolation =
        invisible_places::water::WaterScenarioInterpolation::Smooth;
};

struct TimingColouriseActivationRange {
    // Inclusive normalized animation positions. This window controls only
    // whether the effect contributes to rendering; authored tracks remain
    // global and continue to evaluate from every key.
    float start = 0.0F;
    float end = 1.0F;
};

// Exact first/last animation-time coordinates owned by one Visual Feature's
// setting keys. This is derived authoring state, independent of the feature's
// activation window, and is therefore never serialized.
struct TimingColouriseSettingsKeySpan {
    float start = 0.0F;
    float end = 0.0F;
};

// The same derived clip on the whole-loop circle: start is a canonical
// [0, 1) phase and length a forward cycle fraction, so a clip that straddles
// loop zero (start 0.90, length 0.15) stays one contiguous interval instead
// of flipping into its complement. Never serialized.
struct TimingColouriseCyclicSettingsKeySpan {
    float start = 0.0F;
    float length = 0.0F;
};

// Remembered bounds authoring for one scalar field selector, so switching a
// Visual Feature between families/variants and back never loses edits. An
// unedited entry keeps following the latest globally edited bounds for its
// selector; the first local edit detaches it.
struct TimingColouriseFieldBoundsMemory {
    TimingColouriseFieldSelector selector{};
    TimingColouriseBounds bounds{};
    TimingColouriseBoundsKeyMode boundsKeyMode =
        TimingColouriseBoundsKeyMode::LowerUpper;
    std::vector<TimingColouriseBoundsParameterKey> boundsParameterKeys;
    std::vector<TimingColouriseBoundsKey> boundsKeys;
    bool edited = false;
    // Global-store revision this entry last adopted; unedited entries with
    // an older revision refresh from the shared store.
    std::uint64_t adoptedGlobalRevision = 0U;
};

// A named, shareable bounds state for one scalar field selector.
struct TimingScalarBoundsProfile {
    std::string name;
    TimingColouriseBounds bounds{};
};

// Project-wide bounds knowledge for one scalar field selector: the latest
// edited bounds (the "Global" profile) plus named saved states.
struct TimingScalarBoundsStore {
    TimingColouriseFieldSelector selector{};
    TimingColouriseBounds globalBounds{};
    std::uint64_t revision = 0U;
    std::vector<TimingScalarBoundsProfile> profiles;
};

// One Visual Feature: a single scalar-field binding with an activation
// window that can drive colourise output, emissive output, or both.
struct TimingColouriseEffect {
    std::string id;
    std::string name = "Visual Feature";
    // Independent output aspects. Disabling an aspect keeps its authored
    // settings and keys dormant, exactly as the former Colourise/Emissive
    // kind split preserved the other kind's data. Sanitize guarantees at
    // least one aspect stays enabled, and that the emissive aspect only
    // pairs with a scalar field source.
    bool colouriseEnabled = true;
    bool emissiveEnabled = false;
    bool enabled = true;
    TimingColouriseActivationRange activationRange{};
    TimingColouriseFieldSelector field{};
    TimingColourisePalette basePalette{};
    TimingColouriseBounds baseBounds{};
    TimingColourisePaletteKeyModel paletteKeyModel =
        TimingColourisePaletteKeyModel::StopParameters;
    TimingColourisePaletteSourceKind paletteSourceKind =
        TimingColourisePaletteSourceKind::Custom;
    // Provenance is authoring metadata only. Evaluation always uses the
    // copied palette stored on this effect, never a live library reference.
    std::string paletteSourceId;
    std::string paletteSourceName;
    // Private preset variants owned only by this effect. basePalette remains
    // the active evaluation snapshot; paletteEdited identifies when the
    // matching local variant is the active snapshot.
    std::vector<TimingColouriseLocalPaletteEdit> localPaletteEdits;
    bool paletteEdited = false;
    TimingColouriseAmountOverrideMode colouriseAmountOverrideMode =
        TimingColouriseAmountOverrideMode::Maximum;
    // Applied after palette/key evaluation. This changes colour mixing only,
    // never point opacity or the authored per-stop amounts.
    float colouriseAmountOverride = 1.0F;
    // Base cyclic offset in palette turns. Palette Phase keys below store a
    // signed delta from the preceding phase key (or this base for the first
    // key), constrained to one turn in either direction. Evaluation
    // accumulates those deltas without wrapping; wrapping happens only while
    // sampling the LUT. Stops and their authored positions remain fixed.
    float palettePhaseOffset = 0.0F;
    // Independent animated tracks for phase and the amount override value.
    // EmissiveLevel shares this scalar-key representation. Positive values
    // add emission; negative values darken masked points without changing
    // opacity. The Max/Scale mode itself remains an authored, non-animated
    // choice.
    float emissiveLevel = 1.0F;
    std::vector<TimingColouriseEffectParameterKey> effectParameterKeys;
    // Legacy whole-palette snapshots remain active only in LegacySnapshots
    // mode. New authoring writes the independent tracks below.
    std::vector<TimingColourisePaletteKey> paletteKeys;
    std::vector<TimingColourisePaletteStopParameterKey>
        paletteStopParameterKeys;
    TimingColouriseBoundsKeyMode boundsKeyMode =
        TimingColouriseBoundsKeyMode::LowerUpper;
    // New independently keyed bounds controls. Legacy whole-bounds keys remain
    // below so existing projects retain their exact animation and edge fade.
    // Parameter keys override the corresponding evaluated legacy/base value.
    std::vector<TimingColouriseBoundsParameterKey> boundsParameterKeys;
    // Legacy v1 snapshot keys. New authoring should use boundsParameterKeys.
    std::vector<TimingColouriseBoundsKey> boundsKeys;
    // Whether the live bounds above were locally edited for the current
    // field selector (detaching them from the shared Global bounds).
    bool boundsEdited = false;
    std::uint64_t boundsAdoptedGlobalRevision = 0U;
    // Per-selector authoring memory for every field this feature visited.
    std::vector<TimingColouriseFieldBoundsMemory> fieldBoundsMemory;
};

struct TimingColourisePaletteDefinition {
    std::string id;
    std::string name = "Palette";
    TimingColourisePalette palette{};
};

[[nodiscard]] const TimingColouriseLocalPaletteEdit*
FindTimingColouriseLocalPaletteEdit(
    const TimingColouriseEffect& effect,
    std::string_view presetId);
// Stores an effect-local variant for the effect's current built-in preset and
// makes that variant the active base palette.
[[nodiscard]] bool UpsertTimingColouriseLocalPaletteEdit(
    TimingColouriseEffect* effect,
    TimingColourisePalette palette);
// Selects a built-in source snapshot without deleting any private variants
// already owned by this effect.
[[nodiscard]] bool ActivateTimingColouriseOriginalPreset(
    TimingColouriseEffect* effect,
    const TimingColourisePaletteDefinition& preset);
[[nodiscard]] bool ActivateTimingColouriseLocalPaletteEdit(
    TimingColouriseEffect* effect,
    std::string_view presetId);
// Removes the private variant derived from `originalPreset`. If that variant
// is active, the supplied original becomes active in its place.
[[nodiscard]] bool DiscardTimingColouriseLocalPaletteEdit(
    TimingColouriseEffect* effect,
    const TimingColourisePaletteDefinition& originalPreset);
// Converts a private variant into a shared Saved definition, activates that
// saved snapshot on the effect, and removes only the promoted private entry.
[[nodiscard]] std::optional<TimingColourisePaletteDefinition>
PromoteTimingColouriseLocalPaletteEdit(
    TimingColouriseEffect* effect,
    std::string_view presetId,
    std::string savedPaletteId,
    std::string savedPaletteName);

struct TimingTakeDefinition {
    std::string id;
    std::string name = "Timing Take";
    // Stable profile ids are authoritative. Names are persisted mirrors for
    // readable JSON, recovery from a missing id, and explicit base provenance.
    std::string assignedRainProfileId;
    std::string assignedRainProfileName;
    std::string baseRainProfileId;
    std::string baseRainProfileName;
};

struct TimingTakeSceneState {
    std::string takeId = std::string{kAuthoredTimingTakeId};
    std::string sceneGroupName = "Default";
    std::vector<invisible_places::water::WaterFeatureTimingRun>
        waterFeatureTimingRuns;
    // When enabled, run membership is also an explicit Water visibility
    // allow-list for this Timing Take and scene. Disabled runs retain their
    // membership while contributing no keyed samples.
    bool onlyShowWaterFeaturesInRuns = false;
    // List order is visual priority: index zero is the top layer and is
    // applied last.
    std::vector<TimingColouriseEffect> colouriseEffects;
    std::uint32_t waterFeatureTimingRunSequence = 1U;
    std::uint32_t colouriseEffectSequence = 1U;
};

struct TimingColouriseLayerSample {
    std::array<float, 3> colour{1.0F, 1.0F, 1.0F};
    float colouriseAmount = 0.0F;
};

struct TimingWaterProfileReferenceRewriteCounts {
    std::size_t legacyScenarioTracks = 0U;
    std::size_t timingTakeTracks = 0U;
    std::size_t keyedPackageTracks = 0U;

    [[nodiscard]] std::size_t total() const {
        return legacyScenarioTracks + timingTakeTracks +
               keyedPackageTracks;
    }
};

// Rebinds one dynamic Water profile through every persisted timing store.
// Dormant tracks and reusable keyed packages are deliberately retained and
// rewritten with the active tracks.
[[nodiscard]] TimingWaterProfileReferenceRewriteCounts
ReplaceTimingWaterProfileReferences(
    std::span<invisible_places::water::WaterScenarioFeatureRuns>
        legacyScenarios,
    std::span<TimingTakeSceneState> timingTakeStates,
    std::span<invisible_places::water::WaterKeyedSettingsProfile>
        keyedPackages,
    std::string_view profileGroup,
    std::string_view previousProfileName,
    std::string_view nextProfileName);

// Canonicalizes the selected features' matching profile tracks across both
// live stores without changing reusable packages or unrelated features.
[[nodiscard]] std::size_t CanonicalizeTimingWaterFeatureProfileMetadata(
    std::span<invisible_places::water::WaterScenarioFeatureRuns>
        legacyScenarios,
    std::span<TimingTakeSceneState> timingTakeStates,
    std::span<const invisible_places::water::WaterKeyedFeatureId> features,
    std::string_view profileGroup,
    std::string_view profileName);

[[nodiscard]] std::string NormalizeTimingTakeId(std::string_view takeId);
[[nodiscard]] TimingTakeDefinition AuthoredTimingTakeDefinition();

inline constexpr std::string_view kLegacyWaterRainProfileId =
    "rain-profile-project";
inline constexpr std::string_view kLegacyWaterRainProfileName =
    "Project Rain";
inline constexpr std::string_view kTimingTakeRainTrackProfileGroup =
    "rain";

// Resolves one take's effective/base Rain profiles by stable id first and its
// persisted name mirror second. A missing assignment falls back to the first
// shared base profile; object-owned copies are never chosen as that fallback.
[[nodiscard]] const invisible_places::water::WaterRainProfile*
ResolveTimingTakeRainProfile(
    std::span<const invisible_places::water::WaterRainProfile> profiles,
    const TimingTakeDefinition& take);
[[nodiscard]] const invisible_places::water::WaterRainProfile*
ResolveTimingTakeRainBaseProfile(
    std::span<const invisible_places::water::WaterRainProfile> profiles,
    const TimingTakeDefinition& take);

// Captures one take's exact effective Rain profile by value. Stable identity,
// the full runtime settings, and edited/non-preset visual values remain
// immutable if the source library is changed after an export request.
[[nodiscard]] std::optional<invisible_places::water::WaterRainProfile>
CaptureTimingTakeRainProfileSnapshot(
    std::span<const invisible_places::water::WaterRainProfile> profiles,
    std::span<const TimingTakeDefinition> takes,
    std::string_view takeId);

// Converts one frozen effective profile into the authored Rain level consumed
// by Timing/Seepage exports. Disabled Rain is always exactly dry.
[[nodiscard]] float TimingTakeRainAuthoredLevel(
    const invisible_places::water::RainRuntimeSettings& settings);

// Retains the queue-time scenario's other authored lanes while replacing its
// Rain mirror with the selected Timing Take's immutable profile value.
[[nodiscard]] std::optional<invisible_places::water::WaterScenarioState>
ProjectTimingTakeRainToScenarioSnapshot(
    const std::optional<invisible_places::water::WaterScenarioState>& scenario,
    const invisible_places::water::RainRuntimeSettings& settings);

struct TimingTakeRainLiveSyncDecision {
    bool copyProfile = false;
    bool resetRuntime = false;

    friend bool operator==(
        const TimingTakeRainLiveSyncDecision&,
        const TimingTakeRainLiveSyncDecision&) = default;
};

// Decides whether the single live Rain renderer projection must change. A
// value edit within an already-bound owner profile deliberately returns a
// no-op; take/profile identity changes and explicit project-load refreshes
// copy the resolved snapshot and begin one new simulation epoch.
[[nodiscard]] TimingTakeRainLiveSyncDecision
ResolveTimingTakeRainLiveSyncDecision(
    std::string_view boundTakeId,
    std::string_view boundProfileId,
    std::string_view resolvedTakeId,
    std::string_view resolvedProfileId,
    bool forceRefresh = false);

// Canonicalizes every Rain setting track owned by a Timing Take to one
// metadata group and the take's currently resolved profile name. Legacy RGB
// tracks used `rain_visual`, while scalar tracks often had empty metadata.
[[nodiscard]] std::size_t RewriteTimingTakeRainTrackProfileMetadata(
    std::vector<TimingTakeSceneState>* states,
    std::string_view takeId,
    std::string_view profileName);

// Canonicalizes pre-Timing-Take scenario storage without touching tracks or
// keys. Scenario ids that are stable take ids use that take's effective Rain
// profile; older scenario ids use Authored Timing's effective profile, then
// the first shared profile, and finally the readable literal fallback.
[[nodiscard]] std::size_t RewriteLegacyScenarioRainTrackProfileMetadata(
    std::vector<invisible_places::water::WaterScenarioFeatureRuns>* scenarios,
    std::span<const invisible_places::water::WaterRainProfile> profiles,
    std::span<const TimingTakeDefinition> takes,
    std::string_view legacyProfileName = kLegacyWaterRainProfileName);

// Normalizes ids/names in file order and rewrites profile/take references.
// The first duplicate keeps its identity; later entries receive deterministic
// suffixes, with stable-id plus name mirrors disambiguating legacy collisions.
void SanitizeWaterRainProfileLibrary(
    std::vector<invisible_places::water::WaterRainProfile>* profiles,
    std::vector<TimingTakeDefinition>* takes = nullptr);

// Upgrades a singleton legacy Rain snapshot into exactly one shared base and
// assigns it only to takes that do not already resolve. Returns the base id.
// The full runtime and visual snapshots are copied verbatim.
[[nodiscard]] std::string EnsureLegacyWaterRainProfile(
    std::vector<invisible_places::water::WaterRainProfile>* profiles,
    std::vector<TimingTakeDefinition>* takes,
    const invisible_places::water::RainRuntimeSettings& legacySettings,
    const invisible_places::water::WaterRainVisualSettings& legacyVisual,
    std::string_view preferredName = kLegacyWaterRainProfileName);

struct TimingTakeRainStandaloneExportState {
    std::vector<invisible_places::water::WaterRainProfile> profiles;
    std::vector<TimingTakeDefinition> assignments;
    invisible_places::water::RainRuntimeSettings compatibilitySettings{};
    invisible_places::water::WaterRainVisualSettings compatibilityVisual{};
};

// Captures the complete reusable library and definition-only assignment
// records for Water Sources. The legacy singleton pair mirrors the requested
// take's effective authored profile so older readers see the same Rain.
[[nodiscard]] TimingTakeRainStandaloneExportState
BuildTimingTakeRainStandaloneExportState(
    std::span<const invisible_places::water::WaterRainProfile> profiles,
    std::span<const TimingTakeDefinition> takes,
    std::string_view activeTakeId,
    const invisible_places::water::RainRuntimeSettings& fallbackSettings,
    const invisible_places::water::WaterRainVisualSettings& fallbackVisual);

struct TimingTakeRainImportResult {
    std::size_t profilesInserted = 0U;
    std::size_t profilesUpdated = 0U;
    std::size_t orphanOwnerProfilesSkipped = 0U;
    std::size_t assignmentsApplied = 0U;

    [[nodiscard]] bool changed() const {
        return profilesInserted > 0U || profilesUpdated > 0U ||
               assignmentsApplied > 0U;
    }
};

// Merges a standalone Water Sources Rain library into a project without
// importing Timing Takes. Stable profile ids are authoritative: reused names
// never overwrite another id, and imported collisions receive a unique
// display name. Assignment mirrors are applied only to already-existing take
// ids. Owner copies that neither belong to an existing take nor serve a
// matched observer assignment are skipped as unreachable temporary state.
[[nodiscard]] TimingTakeRainImportResult
MergeImportedTimingTakeRainProfiles(
    std::vector<invisible_places::water::WaterRainProfile>* profiles,
    std::vector<TimingTakeDefinition>* takes,
    std::span<const invisible_places::water::WaterRainProfile>
        importedProfiles,
    std::span<const TimingTakeDefinition> importedAssignments,
    // Schema <=30 carried only one compatibility snapshot. Supplying the
    // current take id preserves its historical replace-Rain behavior without
    // letting a standalone file manufacture a take.
    std::string_view legacyCompatibilityTakeId = {});

// Selects a shared base profile and clears any prior owner-copy assignment.
bool AssignTimingTakeRainBaseProfile(
    TimingTakeDefinition* take,
    std::span<const invisible_places::water::WaterRainProfile> profiles,
    std::string_view baseProfileId);

// Returns the exact collision-safe name that the next authored Rain edit will
// use. An existing owner copy for this take/base pair keeps its current name,
// so UI previews cannot drift from UpsertTimingTakeRainOwnerProfile.
[[nodiscard]] std::string PredictTimingTakeRainOwnerProfileName(
    std::span<const invisible_places::water::WaterRainProfile> profiles,
    const TimingTakeDefinition& take);

// Commits edited Rain into the take-owned `(owner, base)` copy, creating a
// collision-safe `<base>_<take>` profile on first edit. The take is reassigned
// to that copy and its base mirrors remain stable.
[[nodiscard]] invisible_places::water::WaterRainProfile*
UpsertTimingTakeRainOwnerProfile(
    std::vector<invisible_places::water::WaterRainProfile>* profiles,
    TimingTakeDefinition* take,
    const invisible_places::water::RainRuntimeSettings& settings,
    const invisible_places::water::WaterRainVisualSettings& visual);

// Saves the active take's own temporary edit over its exact shared base. The
// owner-copy base id is authoritative; an unresolved supplied id fails closed
// instead of falling back by name. The base keeps its identity/name, the
// temporary owner copy is removed, and references to it follow the base.
[[nodiscard]] invisible_places::water::WaterRainProfile*
SaveTimingTakeRainOwnerProfileToBase(
    std::vector<invisible_places::water::WaterRainProfile>* profiles,
    std::vector<TimingTakeDefinition>* takes,
    std::string_view takeId);

// Promotes the take's effective snapshot into a nameable shared base. Save As
// always allocates a collision-safe new name/id. The explicit overwrite mode
// is retained for callers transitioning to SaveTimingTakeRainOwnerProfileToBase:
// it ignores requestedName and overwrites only the take's resolved base by id.
// Any references to the promoted temporary owner copy follow the saved base
// before it is removed.
[[nodiscard]] invisible_places::water::WaterRainProfile*
SaveTimingTakeRainOwnerProfileAsShared(
    std::vector<invisible_places::water::WaterRainProfile>* profiles,
    std::vector<TimingTakeDefinition>* takes,
    std::string_view takeId,
    std::string_view requestedName,
    bool overwriteResolvedBase = false);

// Returns a take to the base behind its current owner copy and removes that
// temporary copy. Any other take referencing the copy follows the same base.
bool DiscardTimingTakeRainOwnerProfile(
    std::vector<invisible_places::water::WaterRainProfile>* profiles,
    std::vector<TimingTakeDefinition>* takes,
    std::string_view takeId);

// Lifecycle helpers used by Timing Take Rename/Duplicate/Delete transactions.
// Rename updates every assignment-name mirror for the stable owner profile id.
bool RenameTimingTakeRainOwnerProfile(
    std::vector<invisible_places::water::WaterRainProfile>* profiles,
    std::vector<TimingTakeDefinition>* takes,
    std::string_view takeId);
bool DuplicateTimingTakeRainProfileAssignment(
    std::vector<invisible_places::water::WaterRainProfile>* profiles,
    const TimingTakeDefinition& source,
    TimingTakeDefinition* duplicate);
// Surviving takes that referenced a removed owner copy are returned to its
// shared base before the copy is erased.
[[nodiscard]] std::size_t RemoveTimingTakeRainOwnerProfiles(
    std::vector<invisible_places::water::WaterRainProfile>* profiles,
    std::vector<TimingTakeDefinition>* takes,
    std::string_view deletedTakeId);

[[nodiscard]] TimingColourisePalette SanitizeTimingColourisePalette(
    TimingColourisePalette palette);
// Mirrors the palette left-to-right without changing stop identities or
// authored colour/amount values. The returned stops are sanitized and sorted.
[[nodiscard]] TimingColourisePalette ReverseTimingColourisePalette(
    TimingColourisePalette palette);
[[nodiscard]] bool CanReverseTimingColourisePaletteAtPosition(
    const TimingColouriseEffect& effect,
    float position);
// Reverses an unkeyed base palette, or authors only stop-position values at
// the requested animation position. Legacy snapshot tracks can be reversed
// only while positioned exactly on one of their keys.
[[nodiscard]] bool ReverseTimingColourisePaletteAtPosition(
    TimingColouriseEffect* effect,
    float position);
[[nodiscard]] std::string AllocateTimingColourisePaletteStopId(
    const TimingColourisePalette& palette);
[[nodiscard]] TimingColouriseBounds SanitizeTimingColouriseBounds(
    TimingColouriseBounds bounds);
[[nodiscard]] bool TimingColouriseBoundsParameterIsAllowed(
    TimingColouriseBoundsKeyMode mode,
    TimingColouriseBoundsParameter parameter);
[[nodiscard]] std::array<TimingColouriseBoundsParameter, 2>
TimingColouriseBoundsParametersForMode(TimingColouriseBoundsKeyMode mode);
[[nodiscard]] float TimingColouriseBoundsParameterValue(
    const TimingColouriseBounds& bounds,
    TimingColouriseBoundsParameter parameter);
// Transfers a scalar-bounds key between fields with different numeric
// domains. Absolute coordinates preserve their normalized percentile,
// Spread preserves its fraction of the full field range, and Edge Fade is
// dimensionless so it is retained verbatim (subject to normal sanitizing).
[[nodiscard]] float RemapTimingColouriseBoundsParameterValueToRange(
    TimingColouriseBoundsParameter parameter,
    float value,
    float sourceMinimum,
    float sourceMaximum,
    float destinationMinimum,
    float destinationMaximum);
// Places the earliest copied key at destinationAnchor while preserving all
// relative spacing. Near either animation edge, the complete group shifts
// just enough to remain inside the normalized [0, 1] authored domain.
[[nodiscard]] std::vector<float> OffsetTimingColouriseKeyPositionsForPaste(
    std::span<const float> sourcePositions,
    float destinationAnchor);
// Resolves a direct histogram-handle drag into the coordinate track(s) owned
// by the selected keying mode. Translation handles preserve spacing; a
// Centre+Spacing endpoint mirrors its partner around the fixed centre.
[[nodiscard]] std::optional<TimingColouriseBoundsHandleEdit>
ResolveTimingColouriseBoundsHandleEdit(
    TimingColouriseBoundsKeyMode mode,
    TimingColouriseBoundsHandle handle,
    TimingColouriseBounds currentBounds,
    float targetValue,
    float rangeMinimum,
    float rangeMaximum);
// Refuses a mode change while it would invalidate an existing geometric
// parameter track. Edge Fade is independent and never blocks a mode change.
[[nodiscard]] bool SetTimingColouriseBoundsKeyMode(
    TimingColouriseEffect* effect,
    TimingColouriseBoundsKeyMode mode);
// Palette phase and amount override belong to the colourise aspect; the
// emissive level belongs to the emissive aspect. Keys for a disabled
// aspect's parameters stay stored but are never counted, moved, or
// evaluated.
[[nodiscard]] bool TimingEffectParameterIsSupported(
    bool colouriseEnabled,
    bool emissiveEnabled,
    TimingColouriseEffectParameter parameter);
[[nodiscard]] bool TimingEffectParameterIsSupported(
    const TimingColouriseEffect& effect,
    TimingColouriseEffectParameter parameter);
[[nodiscard]] TimingColouriseActivationRange
SanitizeTimingColouriseActivationRange(
    TimingColouriseActivationRange range);
// Uses an inclusive [start, end] interval. Non-finite sample positions are
// never active; finite positions are clamped to the normalized domain.
[[nodiscard]] bool TimingColouriseActivationRangeContains(
    TimingColouriseActivationRange range,
    float normalizedPosition);
// Combines the persistent enabled toggle with the activation window.
[[nodiscard]] bool TimingColouriseEffectIsActiveAt(
    const TimingColouriseEffect& effect,
    float normalizedPosition,
    bool cyclic = false);
// Derives one feature-wide settings clip from every authoritative animation
// key, including keys for disabled aspects and remembered non-current field
// bounds. A remembered entry matching effect.field is a cache of the live
// bounds tracks and is deliberately not counted twice. The positions helper
// returns a sorted, tolerance-deduplicated union for clip ticks; the span is
// nullopt when the feature owns no setting keys.
[[nodiscard]] std::vector<float>
TimingColouriseEffectSettingsKeyPositions(
    const TimingColouriseEffect& effect);
[[nodiscard]] std::optional<TimingColouriseSettingsKeySpan>
TimingColouriseEffectSettingsKeySpan(
    const TimingColouriseEffect& effect);
// Affinely maps every key represented by source onto destination without
// changing the activation window, values, interpolation, or other settings.
// Dormant aspect and non-current field-memory tracks move with the live keys.
// A zero-width source may only translate to another zero-width destination.
// Invalid bounds, a source that does not contain every owned key, or a
// same-lane collision rejects the complete operation without mutation.
[[nodiscard]] bool TransformTimingColouriseEffectSettingsKeys(
    TimingColouriseEffect* effect,
    TimingColouriseSettingsKeySpan source,
    TimingColouriseSettingsKeySpan destination);
// Canonicalises a whole-loop phase into [0, 1): 1.0 wraps to 0.0 because loop
// zero and loop one are the same cyclic instant. Non-finite input maps to 0.
[[nodiscard]] float WrapTimingColouriseLoopPosition(float position);
// Shortest distance around the loop between two wrapped phases, so 0.0 and
// 1.0 are coincident and 0.05 sits 0.10 from 0.95.
[[nodiscard]] float TimingColouriseCyclicKeyDistance(float a, float b);
// Cyclic counterpart of TimingColouriseEffectSettingsKeySpan. The clip is the
// complement of the largest circular gap between owned keys, so keys that
// were dragged across loop zero remain one clip. When the wrap gap ties the
// largest interior gap the canonical min..max span wins, which keeps every
// non-wrapping layout identical to the linear span.
[[nodiscard]] std::optional<TimingColouriseCyclicSettingsKeySpan>
TimingColouriseEffectCyclicSettingsKeySpan(
    const TimingColouriseEffect& effect);
// Cyclic counterpart of TransformTimingColouriseEffectSettingsKeys: every
// key is unwrapped relative to source.start, affinely mapped onto destination
// (whose start may be any finite real), and wrapped back into [0, 1). Keys
// outside the forward source span, a point/non-point mismatch, a length
// above one cycle, or a same-lane collision measured around the loop reject
// the whole operation without mutation.
// Same-lane keys that sit on one cyclic instant (0.0 and 1.0, the usual
// first/last layout of a linear-authored feature) are merged the way cyclic
// evaluation already merges them: the linear-later key survives. Returns the
// number of keys removed. The cyclic transform applies this itself; the key
// lane drag applies it to a selection that wraps both keys together.
std::size_t CoalesceTimingColouriseEffectCyclicallyCoincidentKeys(
    TimingColouriseEffect* effect);
[[nodiscard]] bool TransformTimingColouriseEffectSettingsKeysCyclic(
    TimingColouriseEffect* effect,
    TimingColouriseCyclicSettingsKeySpan source,
    TimingColouriseCyclicSettingsKeySpan destination);
// Palette Phase keys store deltas accumulated from palettePhaseOffset in
// time order, so a cyclic move that carries keys across loop zero re-orders
// the accumulation and changes the phase every key lands on. Given the
// effect before the move and the same effect with only key positions changed
// (keys index-aligned, not yet sorted), re-encodes the deltas so every key
// keeps the accumulated phase it had, making the move a pure retime. Tracks
// whose time order did not change are left bit-identical. Deltas that would
// leave the [-1, 1] range wrap by whole turns, which the palette cannot see.
void PreserveTimingColourisePalettePhaseTargetsAfterMove(
    const TimingColouriseEffect& original,
    TimingColouriseEffect* moved);
[[nodiscard]] TimingColouriseEffect SanitizeTimingColouriseEffect(
    TimingColouriseEffect effect);
[[nodiscard]] TimingColourisePaletteDefinition
SanitizeTimingColourisePaletteDefinition(
    TimingColourisePaletteDefinition definition);
[[nodiscard]] TimingTakeDefinition SanitizeTimingTakeDefinition(
    TimingTakeDefinition definition);
[[nodiscard]] TimingTakeSceneState SanitizeTimingTakeSceneState(
    TimingTakeSceneState state);
// Moves every animation-time coordinate owned by one Timing Take scene from
// sourceDurationFrames onto destinationDurationFrames, optionally after a
// prepended destinationStartFrame, while preserving its authored camera frame.
// An activation start of exactly zero and terminal end of exactly one remain
// "from animation start" / "through animation end" sentinels. Returns false
// without mutation for invalid state, duration, or range arguments.
[[nodiscard]] bool RetimeTimingTakeSceneStateNormalizedPositions(
    TimingTakeSceneState* state,
    std::uint32_t sourceDurationFrames,
    std::uint32_t destinationDurationFrames,
    std::uint32_t destinationStartFrame = 0U);
// Merges a second mapped Timing Take scene into the first. Exact
// same-lane/time collisions keep the destination value, while unrelated
// state from the source retains its run, clip ownership, and curve mode.
// Run and clip ids are remapped where their state-local identities collide,
// and a feature remains assigned to exactly one run.
void MergeTimingTakeSceneStateKeepingFirst(
    TimingTakeSceneState* destination,
    const TimingTakeSceneState& source);
// Upserts the live bounds authoring (base bounds, key mode, both key
// vectors, edited flag) into the effect's per-selector memory under its
// current field selector.
void StashTimingColouriseFieldBounds(TimingColouriseEffect* effect);
// Switches the effect's field selector without losing authoring: the
// current selector's bounds state is stashed first, then the new selector
// restores its remembered state when one exists. Otherwise the effect
// adopts the shared store's Global bounds (when provided) or the supplied
// fallback range, with cleared keys and an unedited state.
void ApplyTimingColouriseFieldSelection(
    TimingColouriseEffect* effect,
    const TimingColouriseFieldSelector& selector,
    const TimingColouriseBounds& fallbackBounds,
    const TimingScalarBoundsStore* globalStore);
[[nodiscard]] TimingScalarBoundsStore* FindTimingScalarBoundsStore(
    std::vector<TimingScalarBoundsStore>* stores,
    const TimingColouriseFieldSelector& selector);
[[nodiscard]] const TimingScalarBoundsStore* FindTimingScalarBoundsStore(
    const std::vector<TimingScalarBoundsStore>& stores,
    const TimingColouriseFieldSelector& selector);
// Records a local base-bounds edit: marks the effect detached ("edited")
// and publishes the bounds as the selector's latest Global state so other
// features with unedited bounds for this selector follow along.
void RecordTimingScalarBoundsEdit(
    std::vector<TimingScalarBoundsStore>* stores,
    TimingColouriseEffect* effect);
// Adopts the selector's latest Global bounds when this effect has no local
// edit and the store has advanced past the revision it last adopted.
// Returns true when the bounds changed.
bool RefreshTimingColouriseBoundsFromGlobal(
    TimingColouriseEffect* effect,
    const std::vector<TimingScalarBoundsStore>& stores);
// Merges legacy single-aspect effect pairs into combined Visual Features:
// an enabled colourise-only effect adopts an enabled emissive-only partner
// only when the pair provably evaluated identically as separate objects —
// exact same field, exactly equal activation window, identical complete
// bounds authoring (base bounds, key mode, both key vectors; bounds gate
// emissive output just as they gate colourise), and no renderer slot-cap
// pressure anywhere inside the window (cap selection is position
// dependent). Pairing is first-match in list order; unmatched effects are
// left untouched. mergeEligible (index-aligned when provided) restricts
// which effects may participate, so aspect-authored features in a mixed
// document are never re-merged. Returns the number of merges performed.
std::size_t MergeLegacyTimingEffectAspects(
    std::vector<TimingColouriseEffect>* effects,
    const std::vector<bool>* mergeEligible = nullptr,
    std::size_t rendererSlotCapacity = 8U);
// Moves an effect to its new final list index while keeping the effect and all
// of its owned animation tracks together.
[[nodiscard]] bool MoveTimingColouriseEffect(
    std::vector<TimingColouriseEffect>* effects,
    std::size_t fromIndex,
    std::size_t toIndex);

[[nodiscard]] TimingColouriseLut CompileTimingColourisePaletteLut(
    const TimingColourisePalette& palette);
[[nodiscard]] TimingColouriseLut ApplyTimingColouriseAmountOverride(
    TimingColouriseLut lut,
    TimingColouriseAmountOverrideMode mode,
    float value);
[[nodiscard]] TimingColouriseLut ApplyTimingColourisePalettePhase(
    const TimingColouriseLut& lut,
    float phaseOffset);
[[nodiscard]] float EvaluateTimingColouriseEffectParameter(
    const TimingColouriseEffect& effect,
    TimingColouriseEffectParameter parameter,
    float normalizedPosition,
    bool cyclic = false);
// Returns the authored Palette Phase delta at an exact key. Between keys it
// returns the currently evaluated phase relative to the preceding accumulated
// key target, which makes inserting a key preserve the visible phase.
[[nodiscard]] float TimingColourisePalettePhaseDeltaFromPrevious(
    const TimingColouriseEffect& effect,
    float normalizedPosition);
[[nodiscard]] float EvaluateTimingEmissiveLevel(
    const TimingColouriseEffect& effect,
    float normalizedPosition,
    bool cyclic = false);
[[nodiscard]] TimingColourisePalette EvaluateTimingColourisePalette(
    const TimingColouriseEffect& effect,
    float normalizedPosition,
    bool cyclic = false);
[[nodiscard]] TimingColouriseLut EvaluateTimingColourisePaletteLut(
    const TimingColouriseEffect& effect,
    float normalizedPosition,
    bool cyclic = false);
[[nodiscard]] TimingColouriseBounds EvaluateTimingColouriseBounds(
    const TimingColouriseEffect& effect,
    float normalizedPosition,
    bool cyclic = false);
[[nodiscard]] TimingColouriseLayerSample SampleTimingColouriseLut(
    const TimingColouriseLut& lut,
    float normalizedFieldValue);
[[nodiscard]] float TimingColouriseBoundsMask(
    const TimingColouriseBounds& bounds,
    float fieldValue);

// Samples are supplied in list order (top first). The helper composites from
// the bottom toward index zero, so the top list item has highest priority.
[[nodiscard]] std::array<float, 3> ApplyTimingColouriseStack(
    std::array<float, 3> baseColour,
    std::span<const TimingColouriseLayerSample> samples);

void AddOrUpdateTimingColourisePaletteKey(
    TimingColouriseEffect* effect,
    float position,
    TimingColourisePalette palette,
    invisible_places::water::WaterScenarioInterpolation interpolation =
        invisible_places::water::WaterScenarioInterpolation::Smooth);
[[nodiscard]] bool AddOrUpdateTimingColourisePaletteStopScalarKey(
    TimingColouriseEffect* effect,
    std::string_view stopId,
    TimingColourisePaletteStopParameter parameter,
    float position,
    float value,
    invisible_places::water::WaterScenarioInterpolation interpolation =
        invisible_places::water::WaterScenarioInterpolation::Smooth);
[[nodiscard]] bool AddOrUpdateTimingColourisePaletteStopColourKey(
    TimingColouriseEffect* effect,
    std::string_view stopId,
    float position,
    std::array<float, 3> colour,
    invisible_places::water::WaterScenarioInterpolation interpolation =
        invisible_places::water::WaterScenarioInterpolation::Smooth);
void AddOrUpdateTimingColouriseBoundsKey(
    TimingColouriseEffect* effect,
    float position,
    TimingColouriseBounds bounds,
    invisible_places::water::WaterScenarioInterpolation interpolation =
        invisible_places::water::WaterScenarioInterpolation::Smooth);
[[nodiscard]] bool AddOrUpdateTimingColouriseBoundsParameterKey(
    TimingColouriseEffect* effect,
    TimingColouriseBoundsParameter parameter,
    float position,
    float value,
    invisible_places::water::WaterScenarioInterpolation interpolation =
        invisible_places::water::WaterScenarioInterpolation::Smooth);
[[nodiscard]] bool AddOrUpdateTimingColouriseEffectParameterKey(
    TimingColouriseEffect* effect,
    TimingColouriseEffectParameter parameter,
    float position,
    float value,
    invisible_places::water::WaterScenarioInterpolation interpolation =
        invisible_places::water::WaterScenarioInterpolation::Smooth);
[[nodiscard]] bool MoveTimingColourisePaletteKey(
    TimingColouriseEffect* effect,
    float sourcePosition,
    float destinationPosition);
[[nodiscard]] bool CanMoveTimingColourisePaletteKeysAtPosition(
    const TimingColouriseEffect& effect,
    float sourcePosition,
    float destinationPosition);
[[nodiscard]] bool MoveTimingColouriseBoundsKey(
    TimingColouriseEffect* effect,
    float sourcePosition,
    float destinationPosition);
// Checks the complete move group for collisions. The requested parameter and
// a coincident geometric partner selected by Bounds Keying form one group;
// Edge Fade remains independent.
[[nodiscard]] bool CanMoveTimingColouriseBoundsParameterKeysAtPosition(
    const TimingColouriseEffect& effect,
    TimingColouriseBoundsParameter parameter,
    float sourcePosition,
    float destinationPosition);
// Moves the complete bounds-parameter group described above atomically.
[[nodiscard]] bool MoveTimingColouriseBoundsParameterKey(
    TimingColouriseEffect* effect,
    TimingColouriseBoundsParameter parameter,
    float sourcePosition,
    float destinationPosition);
[[nodiscard]] bool MoveTimingColouriseEffectParameterKey(
    TimingColouriseEffect* effect,
    TimingColouriseEffectParameter parameter,
    float sourcePosition,
    float destinationPosition);
[[nodiscard]] bool MoveTimingColouriseEffectParameterKeys(
    TimingColouriseEffect* effect,
    float sourcePosition,
    float destinationPosition);
[[nodiscard]] bool CanMoveTimingColouriseEffectParameterKeysAtPosition(
    const TimingColouriseEffect& effect,
    float sourcePosition,
    float destinationPosition);
[[nodiscard]] std::size_t TimingColourisePaletteKeyCountAtPosition(
    const TimingColouriseEffect& effect,
    float position);
[[nodiscard]] std::size_t
TimingColourisePaletteStopParameterKeyCountAtPosition(
    const TimingColouriseEffect& effect,
    std::string_view stopId,
    TimingColourisePaletteStopParameter parameter,
    float position);
[[nodiscard]] std::size_t
TimingColouriseBoundsParameterKeyCountAtPosition(
    const TimingColouriseEffect& effect,
    TimingColouriseBoundsParameter parameter,
    float position);
[[nodiscard]] std::size_t
TimingColouriseEffectParameterKeyCountAtPosition(
    const TimingColouriseEffect& effect,
    TimingColouriseEffectParameter parameter,
    float position);
[[nodiscard]] std::size_t
TimingColouriseEffectParameterUnionKeyCountAtPosition(
    const TimingColouriseEffect& effect,
    float position);
[[nodiscard]] std::size_t TimingColouriseBoundsKeyCountAtPosition(
    const TimingColouriseEffect& effect,
    float position);
[[nodiscard]] std::size_t TimingColouriseEffectKeyCountAtPosition(
    const TimingColouriseEffect& effect,
    float position);
[[nodiscard]] std::size_t RemoveTimingColourisePaletteKeysAtPosition(
    TimingColouriseEffect* effect,
    float position);
[[nodiscard]] std::size_t
RemoveTimingColourisePaletteStopParameterKeysAtPosition(
    TimingColouriseEffect* effect,
    std::string_view stopId,
    TimingColourisePaletteStopParameter parameter,
    float position);
[[nodiscard]] std::size_t RemoveTimingColouriseBoundsKeysAtPosition(
    TimingColouriseEffect* effect,
    float position);
[[nodiscard]] std::size_t
RemoveTimingColouriseBoundsParameterKeysAtPosition(
    TimingColouriseEffect* effect,
    TimingColouriseBoundsParameter parameter,
    float position);
[[nodiscard]] std::size_t
RemoveTimingColouriseEffectParameterKeysAtPosition(
    TimingColouriseEffect* effect,
    TimingColouriseEffectParameter parameter,
    float position);
[[nodiscard]] std::size_t
RemoveTimingColouriseEffectParameterKeysAtPosition(
    TimingColouriseEffect* effect,
    float position);
[[nodiscard]] std::size_t RemoveTimingColouriseEffectKeysAtPosition(
    TimingColouriseEffect* effect,
    float position);
[[nodiscard]] std::optional<float>
PreviousTimingColourisePaletteKeyPosition(
    const TimingColouriseEffect& effect,
    float position);
[[nodiscard]] std::optional<float> NextTimingColourisePaletteKeyPosition(
    const TimingColouriseEffect& effect,
    float position);
[[nodiscard]] std::vector<float> TimingColourisePaletteKeyPositions(
    const TimingColouriseEffect& effect);
// Stop topology is static for independent property tracks. Removing a stop
// is refused while it has keys, preserving dormant animation data.
[[nodiscard]] bool CanRemoveTimingColourisePaletteStop(
    const TimingColouriseEffect& effect,
    std::string_view stopId);
[[nodiscard]] bool RemoveTimingColourisePaletteStop(
    TimingColouriseEffect* effect,
    std::string_view stopId);
[[nodiscard]] std::string TimingColourisePaletteKeyStateName(
    std::string_view paletteName,
    std::size_t orderedPosition);
[[nodiscard]] std::optional<float>
PreviousTimingColouriseBoundsKeyPosition(
    const TimingColouriseEffect& effect,
    float position);
[[nodiscard]] std::optional<float> NextTimingColouriseBoundsKeyPosition(
    const TimingColouriseEffect& effect,
    float position);
[[nodiscard]] std::optional<float>
PreviousTimingColouriseBoundsParameterKeyPosition(
    const TimingColouriseEffect& effect,
    TimingColouriseBoundsParameter parameter,
    float position);
[[nodiscard]] std::optional<float>
NextTimingColouriseBoundsParameterKeyPosition(
    const TimingColouriseEffect& effect,
    TimingColouriseBoundsParameter parameter,
    float position);
[[nodiscard]] std::optional<float>
PreviousTimingColouriseEffectParameterKeyPosition(
    const TimingColouriseEffect& effect,
    TimingColouriseEffectParameter parameter,
    float position);
[[nodiscard]] std::optional<float>
NextTimingColouriseEffectParameterKeyPosition(
    const TimingColouriseEffect& effect,
    TimingColouriseEffectParameter parameter,
    float position);
[[nodiscard]] std::vector<float>
TimingColouriseEffectParameterKeyPositions(
    const TimingColouriseEffect& effect,
    TimingColouriseEffectParameter parameter);
[[nodiscard]] std::vector<float>
TimingColouriseEffectParameterKeyPositions(
    const TimingColouriseEffect& effect);
[[nodiscard]] std::optional<float>
PreviousTimingColouriseAnyEffectParameterKeyPosition(
    const TimingColouriseEffect& effect,
    float position);
[[nodiscard]] std::optional<float>
NextTimingColouriseAnyEffectParameterKeyPosition(
    const TimingColouriseEffect& effect,
    float position);
[[nodiscard]] std::optional<float> PreviousTimingColouriseEffectKeyPosition(
    const TimingColouriseEffect& effect,
    float position);
[[nodiscard]] std::optional<float> NextTimingColouriseEffectKeyPosition(
    const TimingColouriseEffect& effect,
    float position);

[[nodiscard]] const TimingTakeDefinition* FindTimingTakeDefinition(
    std::span<const TimingTakeDefinition> takes,
    std::string_view takeId);
[[nodiscard]] TimingTakeDefinition* FindTimingTakeDefinition(
    std::vector<TimingTakeDefinition>* takes,
    std::string_view takeId);
[[nodiscard]] const TimingTakeSceneState* FindTimingTakeSceneState(
    std::span<const TimingTakeSceneState> states,
    std::string_view takeId,
    std::string_view sceneGroupName);
[[nodiscard]] TimingTakeSceneState* FindTimingTakeSceneState(
    std::vector<TimingTakeSceneState>* states,
    std::string_view takeId,
    std::string_view sceneGroupName);
[[nodiscard]] TimingTakeSceneState* EnsureTimingTakeSceneState(
    std::vector<TimingTakeSceneState>* states,
    std::string_view takeId,
    std::string_view sceneGroupName);
[[nodiscard]] bool AssignWaterFeatureToTimingRun(
    TimingTakeSceneState* state,
    const invisible_places::water::WaterKeyedFeatureId& feature,
    std::uint32_t targetRunId);

[[nodiscard]] std::string AllocateTimingTakeId(
    std::span<const TimingTakeDefinition> takes,
    std::uint32_t* nextSequence);
[[nodiscard]] std::string AllocateTimingColouriseEffectId(
    std::span<const TimingColouriseEffect> effects,
    std::uint32_t* nextSequence);
[[nodiscard]] std::string AllocateTimingColourisePaletteId(
    std::span<const TimingColourisePaletteDefinition> palettes,
    std::uint32_t* nextSequence);

}  // namespace invisible_places::timing
