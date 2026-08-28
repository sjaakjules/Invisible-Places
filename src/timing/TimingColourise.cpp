#include "timing/TimingColourise.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <iomanip>
#include <limits>
#include <numeric>
#include <optional>
#include <sstream>
#include <tuple>
#include <unordered_set>
#include <utility>

namespace invisible_places::timing {
namespace {

std::string TrimRainProfileText(std::string_view value) {
    const auto isSpace = [](unsigned char character) {
        return std::isspace(character) != 0;
    };
    auto first = value.begin();
    while (first != value.end() &&
           isSpace(static_cast<unsigned char>(*first))) {
        ++first;
    }
    auto last = value.end();
    while (last != first &&
           isSpace(static_cast<unsigned char>(*(last - 1)))) {
        --last;
    }
    return std::string{first, last};
}

std::string RainProfileNameKey(std::string_view value) {
    auto key = TrimRainProfileText(value);
    std::transform(
        key.begin(),
        key.end(),
        key.begin(),
        [](unsigned char character) {
            return static_cast<char>(std::tolower(character));
        });
    return key;
}

const invisible_places::water::WaterRainProfile* FindAssignedRainProfile(
    std::span<const invisible_places::water::WaterRainProfile> profiles,
    const TimingTakeDefinition& take) {
    if (const auto* byId = invisible_places::water::FindWaterRainProfileById(
            profiles,
            take.assignedRainProfileId);
        byId != nullptr) {
        return byId;
    }
    return invisible_places::water::FindWaterRainProfileByName(
        profiles,
        take.assignedRainProfileName);
}

const invisible_places::water::WaterRainProfile* FindExplicitRainBaseProfile(
    std::span<const invisible_places::water::WaterRainProfile> profiles,
    std::string_view id,
    std::string_view name) {
    const auto* profile = invisible_places::water::FindWaterRainProfileById(
        profiles,
        id);
    if (profile == nullptr) {
        profile = invisible_places::water::FindWaterRainProfileByName(
            profiles,
            name);
    }
    return profile != nullptr && !profile->objectOverride ? profile : nullptr;
}

const invisible_places::water::WaterRainProfile* FirstSharedRainProfile(
    std::span<const invisible_places::water::WaterRainProfile> profiles) {
    const auto found = std::find_if(
        profiles.begin(),
        profiles.end(),
        [](const invisible_places::water::WaterRainProfile& profile) {
            return !profile.objectOverride;
        });
    return found != profiles.end() ? &*found : nullptr;
}

void SetTimingTakeRainAssignment(
    TimingTakeDefinition* take,
    const invisible_places::water::WaterRainProfile& assigned,
    const invisible_places::water::WaterRainProfile& base) {
    if (take == nullptr) {
        return;
    }
    take->assignedRainProfileId = assigned.id;
    take->assignedRainProfileName = assigned.name;
    take->baseRainProfileId = base.id;
    take->baseRainProfileName = base.name;
}

std::string UniqueRainProfileName(
    std::span<const invisible_places::water::WaterRainProfile> profiles,
    std::string_view preferredName,
    std::string_view excludedId = {}) {
    auto preferred = TrimRainProfileText(preferredName);
    if (preferred.empty()) {
        preferred = "Project Rain";
    }
    const auto available = [&](std::string_view candidate) {
        const auto wanted = RainProfileNameKey(candidate);
        return std::none_of(
            profiles.begin(),
            profiles.end(),
            [&](const invisible_places::water::WaterRainProfile& profile) {
                return profile.id != excludedId &&
                       RainProfileNameKey(profile.name) == wanted;
            });
    };
    if (available(preferred)) {
        return preferred;
    }
    for (std::uint32_t suffix = 2U; suffix < 100'000U; ++suffix) {
        const auto candidate = preferred + " " + std::to_string(suffix);
        if (available(candidate)) {
            return candidate;
        }
    }
    return preferred + " Copy";
}

std::string TimingTakeRainOwnerProfileName(
    const invisible_places::water::WaterRainProfile& base,
    const TimingTakeDefinition& take) {
    const auto takeName = TrimRainProfileText(take.name);
    return base.name + "_" +
           (takeName.empty() ? std::string{"Timing Take"} : takeName);
}

std::optional<std::size_t> TimingTakeRainOwnerProfileIndex(
    std::span<const invisible_places::water::WaterRainProfile> profiles,
    const TimingTakeDefinition& take,
    const invisible_places::water::WaterRainProfile& base) {
    const auto baseNameKey = RainProfileNameKey(base.name);
    const auto existing = std::find_if(
        profiles.begin(),
        profiles.end(),
        [&](const invisible_places::water::WaterRainProfile& profile) {
            if (!profile.objectOverride ||
                profile.ownerTimingTakeId != take.id) {
                return false;
            }
            return (!base.id.empty() &&
                    profile.baseProfileId == base.id) ||
                   (profile.baseProfileId.empty() &&
                    RainProfileNameKey(profile.baseProfileName) ==
                        baseNameKey);
        });
    if (existing == profiles.end()) {
        return std::nullopt;
    }
    return static_cast<std::size_t>(existing - profiles.begin());
}

float FiniteOr(float value, float fallback) {
    return std::isfinite(value) ? value : fallback;
}

float Clamp01(float value) {
    return std::clamp(FiniteOr(value, 0.0F), 0.0F, 1.0F);
}

float ClampPalettePhaseDelta(float value) {
    return std::clamp(FiniteOr(value, 0.0F), -1.0F, 1.0F);
}

float SrgbChannelToLinear(float value) {
    value = Clamp01(value);
    return value <= 0.04045F
               ? value / 12.92F
               : std::pow((value + 0.055F) / 1.055F, 2.4F);
}

float LinearChannelToSrgb(float value) {
    value = std::max(0.0F, FiniteOr(value, 0.0F));
    return Clamp01(
        value <= 0.0031308F
            ? value * 12.92F
            : 1.055F * std::pow(value, 1.0F / 2.4F) - 0.055F);
}

bool IsValidColourSpace(TimingColouriseColourSpace space) {
    switch (space) {
        case TimingColouriseColourSpace::Srgb:
        case TimingColouriseColourSpace::LinearRgb:
        case TimingColouriseColourSpace::OkLab:
            return true;
    }
    return false;
}

float InterpolationAmount(
    invisible_places::water::WaterScenarioInterpolation interpolation,
    float amount) {
    amount = Clamp01(amount);
    if (interpolation ==
        invisible_places::water::WaterScenarioInterpolation::Hold) {
        return 0.0F;
    }
    if (interpolation ==
            invisible_places::water::WaterScenarioInterpolation::Smooth ||
        interpolation == invisible_places::water::
                             WaterScenarioInterpolation::SplineHandles ||
        interpolation == invisible_places::water::
                             WaterScenarioInterpolation::SmoothVelocity ||
        interpolation == invisible_places::water::
                             WaterScenarioInterpolation::
                                 CentripetalCatmullRom) {
        return amount * amount * (3.0F - 2.0F * amount);
    }
    return amount;
}

bool IsValidInterpolation(
    invisible_places::water::WaterScenarioInterpolation interpolation) {
    using invisible_places::water::WaterScenarioInterpolation;
    switch (interpolation) {
        case WaterScenarioInterpolation::Smooth:
        case WaterScenarioInterpolation::Linear:
        case WaterScenarioInterpolation::Hold:
        case WaterScenarioInterpolation::SmoothVelocity:
        case WaterScenarioInterpolation::CentripetalCatmullRom:
            return true;
        // Water-track-only styles; colourise keys never carry them.
        case WaterScenarioInterpolation::SplineHandles:
        case WaterScenarioInterpolation::TrackDefault:
            return false;
    }
    return false;
}

template <typename Key>
void SortAndCoalesceKeys(std::vector<Key>* keys) {
    std::stable_sort(
        keys->begin(),
        keys->end(),
        [](const Key& left, const Key& right) {
            return left.position < right.position;
        });
    std::vector<Key> unique;
    unique.reserve(keys->size());
    for (auto& key : *keys) {
        if (!unique.empty() &&
            std::abs(unique.back().position - key.position) <=
                kTimingColouriseKeyTolerance) {
            unique.back() = std::move(key);
        } else {
            unique.push_back(std::move(key));
        }
    }
    *keys = std::move(unique);
}

bool IsValidBoundsKeyMode(TimingColouriseBoundsKeyMode mode) {
    switch (mode) {
        case TimingColouriseBoundsKeyMode::LowerUpper:
        case TimingColouriseBoundsKeyMode::CentreSpread:
        case TimingColouriseBoundsKeyMode::LowerSpread:
        case TimingColouriseBoundsKeyMode::UpperSpread:
            return true;
    }
    return false;
}

bool IsValidBoundsParameter(TimingColouriseBoundsParameter parameter) {
    switch (parameter) {
        case TimingColouriseBoundsParameter::Lower:
        case TimingColouriseBoundsParameter::Upper:
        case TimingColouriseBoundsParameter::Centre:
        case TimingColouriseBoundsParameter::Spread:
        // Legacy shared-fade keys stay valid through parsing so sanitize can
        // split them into the two per-edge tracks.
        case TimingColouriseBoundsParameter::EdgeFade:
        case TimingColouriseBoundsParameter::EdgeFadeLower:
        case TimingColouriseBoundsParameter::EdgeFadeUpper:
            return true;
    }
    return false;
}

// Splits every legacy shared EdgeFade key into coincident EdgeFadeLower and
// EdgeFadeUpper keys. Runs before per-key sanitizing, so runtime tracks
// never hold the legacy parameter.
void SplitLegacyEdgeFadeKeys(
    std::vector<TimingColouriseBoundsParameterKey>* keys) {
    if (keys == nullptr) {
        return;
    }
    std::vector<TimingColouriseBoundsParameterKey> split;
    split.reserve(keys->size());
    for (auto& key : *keys) {
        if (key.parameter != TimingColouriseBoundsParameter::EdgeFade) {
            split.push_back(std::move(key));
            continue;
        }
        auto lower = key;
        lower.parameter = TimingColouriseBoundsParameter::EdgeFadeLower;
        auto upper = std::move(key);
        upper.parameter = TimingColouriseBoundsParameter::EdgeFadeUpper;
        split.push_back(std::move(lower));
        split.push_back(std::move(upper));
    }
    *keys = std::move(split);
}

bool IsValidPaletteKeyModel(TimingColourisePaletteKeyModel model) {
    switch (model) {
        case TimingColourisePaletteKeyModel::LegacySnapshots:
        case TimingColourisePaletteKeyModel::StopParameters:
            return true;
    }
    return false;
}

bool IsValidPaletteSourceKind(TimingColourisePaletteSourceKind kind) {
    switch (kind) {
        case TimingColourisePaletteSourceKind::Custom:
        case TimingColourisePaletteSourceKind::Preset:
        case TimingColourisePaletteSourceKind::Saved:
            return true;
    }
    return false;
}

bool IsValidAmountOverrideMode(
    TimingColouriseAmountOverrideMode mode) {
    switch (mode) {
        case TimingColouriseAmountOverrideMode::Maximum:
        case TimingColouriseAmountOverrideMode::Scale:
            return true;
    }
    return false;
}

bool IsValidEffectParameter(
    TimingColouriseEffectParameter parameter) {
    switch (parameter) {
        case TimingColouriseEffectParameter::PalettePhase:
        case TimingColouriseEffectParameter::AmountOverride:
        case TimingColouriseEffectParameter::EmissiveLevel:
        case TimingColouriseEffectParameter::PaletteSkewCentre:
        case TimingColouriseEffectParameter::PaletteSkewLower:
        case TimingColouriseEffectParameter::PaletteSkewUpper:
        case TimingColouriseEffectParameter::PaletteSkewSpread:
        case TimingColouriseEffectParameter::EmissiveSkewCentre:
        case TimingColouriseEffectParameter::EmissiveSkewSpread:
            return true;
    }
    return false;
}

float ClampPaletteSkew(float value) {
    return std::clamp(FiniteOr(value, 0.0F), -1.0F, 1.0F);
}

bool IsValidPaletteStopParameter(
    TimingColourisePaletteStopParameter parameter) {
    switch (parameter) {
        case TimingColourisePaletteStopParameter::Position:
        case TimingColourisePaletteStopParameter::Colour:
        case TimingColourisePaletteStopParameter::ColouriseAmount:
            return true;
    }
    return false;
}

float SanitizePaletteStopScalarValue(
    TimingColourisePaletteStopParameter parameter,
    float value) {
    value = FiniteOr(value, 0.0F);
    if (parameter == TimingColourisePaletteStopParameter::Position ||
        parameter ==
            TimingColourisePaletteStopParameter::ColouriseAmount) {
        return Clamp01(value);
    }
    return value;
}

std::array<float, 3> SanitizePaletteStopColour(
    std::array<float, 3> colour) {
    for (auto& channel : colour) {
        channel = Clamp01(channel);
    }
    return colour;
}

void SortAndCoalescePaletteStopParameterKeys(
    std::vector<TimingColourisePaletteStopParameterKey>* keys) {
    std::stable_sort(
        keys->begin(),
        keys->end(),
        [](const TimingColourisePaletteStopParameterKey& left,
           const TimingColourisePaletteStopParameterKey& right) {
            if (left.stopId != right.stopId) {
                return left.stopId < right.stopId;
            }
            if (left.parameter != right.parameter) {
                return static_cast<std::uint8_t>(left.parameter) <
                       static_cast<std::uint8_t>(right.parameter);
            }
            return left.position < right.position;
        });
    std::vector<TimingColourisePaletteStopParameterKey> unique;
    unique.reserve(keys->size());
    for (auto& key : *keys) {
        if (!unique.empty() && unique.back().stopId == key.stopId &&
            unique.back().parameter == key.parameter &&
            std::abs(unique.back().position - key.position) <=
                kTimingColouriseKeyTolerance) {
            unique.back() = std::move(key);
        } else {
            unique.push_back(std::move(key));
        }
    }
    *keys = std::move(unique);
}

float SanitizeBoundsParameterValue(
    TimingColouriseBoundsParameter parameter,
    float value) {
    if (parameter == TimingColouriseBoundsParameter::EdgeFade) {
        return std::clamp(FiniteOr(value, 0.10F), -0.5F, 0.5F);
    }
    if (parameter == TimingColouriseBoundsParameter::EdgeFadeLower ||
        parameter == TimingColouriseBoundsParameter::EdgeFadeUpper) {
        // The split fades reach a whole-span fade in either direction.
        return std::clamp(FiniteOr(value, 0.10F), -1.0F, 1.0F);
    }
    value = FiniteOr(value, 0.0F);
    if (parameter == TimingColouriseBoundsParameter::Spread) {
        return std::max(0.0F, value);
    }
    return value;
}

void SortAndCoalesceBoundsParameterKeys(
    std::vector<TimingColouriseBoundsParameterKey>* keys) {
    std::stable_sort(
        keys->begin(),
        keys->end(),
        [](const TimingColouriseBoundsParameterKey& left,
           const TimingColouriseBoundsParameterKey& right) {
            if (left.parameter != right.parameter) {
                return static_cast<std::uint8_t>(left.parameter) <
                       static_cast<std::uint8_t>(right.parameter);
            }
            return left.position < right.position;
        });
    std::vector<TimingColouriseBoundsParameterKey> unique;
    unique.reserve(keys->size());
    for (auto& key : *keys) {
        if (!unique.empty() &&
            unique.back().parameter == key.parameter &&
            std::abs(unique.back().position - key.position) <=
                kTimingColouriseKeyTolerance) {
            unique.back() = std::move(key);
        } else {
            unique.push_back(std::move(key));
        }
    }
    *keys = std::move(unique);
}

void SortAndCoalesceEffectParameterKeys(
    std::vector<TimingColouriseEffectParameterKey>* keys) {
    std::stable_sort(
        keys->begin(),
        keys->end(),
        [](const TimingColouriseEffectParameterKey& left,
           const TimingColouriseEffectParameterKey& right) {
            if (left.parameter != right.parameter) {
                return static_cast<std::uint8_t>(left.parameter) <
                       static_cast<std::uint8_t>(right.parameter);
            }
            return left.position < right.position;
        });
    std::vector<TimingColouriseEffectParameterKey> unique;
    unique.reserve(keys->size());
    for (auto& key : *keys) {
        if (!unique.empty() &&
            unique.back().parameter == key.parameter &&
            std::abs(unique.back().position - key.position) <=
                kTimingColouriseKeyTolerance) {
            unique.back() = std::move(key);
        } else {
            unique.push_back(std::move(key));
        }
    }
    *keys = std::move(unique);
}

float EffectParameterBaseValue(
    const TimingColouriseEffect& effect,
    TimingColouriseEffectParameter parameter) {
    switch (parameter) {
        case TimingColouriseEffectParameter::PalettePhase:
            return effect.palettePhaseOffset;
        case TimingColouriseEffectParameter::AmountOverride:
            return effect.colouriseAmountOverride;
        case TimingColouriseEffectParameter::EmissiveLevel:
            return effect.emissiveLevel;
        case TimingColouriseEffectParameter::PaletteSkewCentre:
            return effect.paletteSkewCentre;
        case TimingColouriseEffectParameter::PaletteSkewSpread:
            return effect.paletteSkewSpread;
        case TimingColouriseEffectParameter::EmissiveSkewCentre:
            return effect.emissiveSkewCentre;
        case TimingColouriseEffectParameter::EmissiveSkewSpread:
            return effect.emissiveSkewSpread;
        // Legacy per-side skews; sanitize retags their keys onto Spread,
        // so nothing evaluates them.
        case TimingColouriseEffectParameter::PaletteSkewLower:
        case TimingColouriseEffectParameter::PaletteSkewUpper:
            return 0.0F;
    }
    return 0.0F;
}

float SanitizeEffectParameterValue(
    TimingColouriseEffectParameter parameter,
    float value) {
    switch (parameter) {
        case TimingColouriseEffectParameter::PalettePhase:
            return ClampPalettePhaseDelta(value);
        case TimingColouriseEffectParameter::AmountOverride:
            return Clamp01(value);
        case TimingColouriseEffectParameter::EmissiveLevel:
            return FiniteOr(value, 1.0F);
        case TimingColouriseEffectParameter::PaletteSkewCentre:
        case TimingColouriseEffectParameter::EmissiveSkewCentre:
            return std::clamp(FiniteOr(value, 0.5F), 0.0F, 1.0F);
        case TimingColouriseEffectParameter::PaletteSkewLower:
        case TimingColouriseEffectParameter::PaletteSkewUpper:
        case TimingColouriseEffectParameter::PaletteSkewSpread:
        case TimingColouriseEffectParameter::EmissiveSkewSpread:
            return ClampPaletteSkew(value);
    }
    return 0.0F;
}

template <typename Iterator, typename ValueAt>
std::optional<float> EvaluateScalarKeyTrack(
    Iterator trackBegin,
    Iterator trackEnd,
    float normalizedPosition,
    ValueAt valueAt) {
    using invisible_places::water::WaterScenarioInterpolation;
    if (trackBegin == trackEnd) {
        return std::nullopt;
    }
    const auto keyCount = static_cast<std::size_t>(
        std::distance(trackBegin, trackEnd));
    const auto& first = *trackBegin;
    const auto& last = *(trackEnd - 1);
    if (normalizedPosition <= first.position || keyCount == 1U) {
        return valueAt(first);
    }
    if (normalizedPosition >= last.position) {
        return valueAt(last);
    }

    const auto right = std::lower_bound(
        trackBegin,
        trackEnd,
        normalizedPosition,
        [](const auto& key, float position) {
            return key.position < position;
        });
    if (right == trackEnd) {
        return valueAt(last);
    }
    // A key is authoritative at its own instant for every curve style. In
    // particular a Hold segment ending here must show this key's value, not
    // hold the previous one through it.
    if (std::abs(right->position - normalizedPosition) <=
        kTimingColouriseKeyTolerance) {
        return valueAt(*right);
    }
    const auto left = right - 1;
    const double segmentDuration =
        static_cast<double>(right->position) -
        static_cast<double>(left->position);
    if (segmentDuration <=
        static_cast<double>(kTimingColouriseKeyTolerance)) {
        return valueAt(*right);
    }
    const double amount = std::clamp(
        (static_cast<double>(normalizedPosition) -
         static_cast<double>(left->position)) /
            segmentDuration,
        0.0,
        1.0);
    if (left->interpolation ==
        WaterScenarioInterpolation::CentripetalCatmullRom) {
        struct SplinePoint {
            double x = 0.0;
            double y = 0.0;
        };
        double valueMinimum = std::numeric_limits<double>::max();
        double valueMaximum = std::numeric_limits<double>::lowest();
        for (auto key = trackBegin; key != trackEnd; ++key) {
            const double value = static_cast<double>(valueAt(*key));
            valueMinimum = std::min(valueMinimum, value);
            valueMaximum = std::max(valueMaximum, value);
        }
        const double valueSpan = valueMaximum - valueMinimum;
        if (!std::isfinite(valueSpan) ||
            valueSpan <= std::numeric_limits<double>::epsilon()) {
            return valueAt(*left);
        }
        const auto pointForKey = [&](const auto& key) {
            return SplinePoint{
                .x = static_cast<double>(key.position),
                .y = (static_cast<double>(valueAt(key)) - valueMinimum) /
                     valueSpan,
            };
        };
        const auto leftIndex = static_cast<std::size_t>(
            std::distance(trackBegin, left));
        const SplinePoint point1 = pointForKey(*left);
        const SplinePoint point2 = pointForKey(*right);
        const SplinePoint point0 =
            leftIndex > 0U
                ? pointForKey(*(left - 1))
                : SplinePoint{
                      .x = 2.0 * point1.x - point2.x,
                      .y = 2.0 * point1.y - point2.y,
                  };
        const SplinePoint point3 =
            leftIndex + 2U < keyCount
                ? pointForKey(*(right + 1))
                : SplinePoint{
                      .x = 2.0 * point2.x - point1.x,
                      .y = 2.0 * point2.y - point1.y,
                  };
        const auto nextKnot = [](double knot,
                                 const SplinePoint& firstPoint,
                                 const SplinePoint& secondPoint) {
            // The Euclidean chord length is raised to alpha = 0.5.
            // A small floor keeps coincident control points evaluable.
            const double increment = std::max(
                1.0e-7,
                std::sqrt(std::hypot(
                    secondPoint.x - firstPoint.x,
                    secondPoint.y - firstPoint.y)));
            return knot + increment;
        };
        const double knot0 = 0.0;
        const double knot1 = nextKnot(knot0, point0, point1);
        const double knot2 = nextKnot(knot1, point1, point2);
        const double knot3 = nextKnot(knot2, point2, point3);
        const auto mixPoint = [](const SplinePoint& firstPoint,
                                 const SplinePoint& secondPoint,
                                 double firstWeight,
                                 double secondWeight,
                                 double denominator) {
            denominator = std::max(denominator, 1.0e-12);
            return SplinePoint{
                .x = (firstWeight * firstPoint.x +
                      secondWeight * secondPoint.x) /
                     denominator,
                .y = (firstWeight * firstPoint.y +
                      secondWeight * secondPoint.y) /
                     denominator,
            };
        };
        const auto evaluatePoint = [&](double knot) {
            const auto a1 = mixPoint(
                point0,
                point1,
                knot1 - knot,
                knot - knot0,
                knot1 - knot0);
            const auto a2 = mixPoint(
                point1,
                point2,
                knot2 - knot,
                knot - knot1,
                knot2 - knot1);
            const auto a3 = mixPoint(
                point2,
                point3,
                knot3 - knot,
                knot - knot2,
                knot3 - knot2);
            const auto b1 = mixPoint(
                a1,
                a2,
                knot2 - knot,
                knot - knot0,
                knot2 - knot0);
            const auto b2 = mixPoint(
                a2,
                a3,
                knot3 - knot,
                knot - knot1,
                knot3 - knot1);
            return mixPoint(
                b1,
                b2,
                knot2 - knot,
                knot - knot1,
                knot2 - knot1);
        };

        // The centripetal spline is a 2D curve in animation-position/value
        // space. Invert its x coordinate so evaluation remains keyed to the
        // animation timeline and dy/dx stays continuous at shared nodes.
        const double targetX = static_cast<double>(normalizedPosition);
        constexpr int kBracketSamples = 32;
        double bracketLow = knot1;
        double bracketHigh = knot2;
        bool bracketed = false;
        double previousKnot = knot1;
        double previousX = evaluatePoint(previousKnot).x;
        for (int sample = 1; sample <= kBracketSamples; ++sample) {
            const double candidateKnot = std::lerp(
                knot1,
                knot2,
                static_cast<double>(sample) /
                    static_cast<double>(kBracketSamples));
            const double candidateX = evaluatePoint(candidateKnot).x;
            if ((previousX - targetX) * (candidateX - targetX) <= 0.0) {
                bracketLow = previousKnot;
                bracketHigh = candidateKnot;
                bracketed = true;
                break;
            }
            previousKnot = candidateKnot;
            previousX = candidateX;
        }
        double evaluatedKnot = std::lerp(knot1, knot2, amount);
        if (bracketed) {
            double lowX = evaluatePoint(bracketLow).x;
            for (int iteration = 0; iteration < 36; ++iteration) {
                const double middle =
                    std::midpoint(bracketLow, bracketHigh);
                const double middleX = evaluatePoint(middle).x;
                if ((lowX - targetX) * (middleX - targetX) <= 0.0) {
                    bracketHigh = middle;
                } else {
                    bracketLow = middle;
                    lowX = middleX;
                }
            }
            evaluatedKnot = std::midpoint(bracketLow, bracketHigh);
        }
        const double evaluated =
            valueMinimum + evaluatePoint(evaluatedKnot).y * valueSpan;
        const float result = static_cast<float>(evaluated);
        return std::isfinite(result)
                   ? result
                   : std::lerp(
                         valueAt(*left),
                         valueAt(*right),
                         static_cast<float>(amount));
    }
    if (left->interpolation !=
        WaterScenarioInterpolation::SmoothVelocity) {
        return std::lerp(
            valueAt(*left),
            valueAt(*right),
            InterpolationAmount(
                left->interpolation,
                static_cast<float>(amount)));
    }

    const auto keyAt = [&](std::size_t index) -> const auto& {
        return *(trackBegin + static_cast<std::ptrdiff_t>(index));
    };
    const auto intervalDuration = [&](std::size_t index) {
        return static_cast<double>(keyAt(index + 1U).position) -
               static_cast<double>(keyAt(index).position);
    };
    const auto intervalVelocity = [&](std::size_t index) {
        const double duration = intervalDuration(index);
        return duration >
                       static_cast<double>(
                           kTimingColouriseKeyTolerance)
                   ? (static_cast<double>(valueAt(keyAt(index + 1U))) -
                      static_cast<double>(valueAt(keyAt(index)))) /
                         duration
                   : 0.0;
    };
    const auto continuesInSameDirection =
        [](double leftVelocity, double rightVelocity) {
            return leftVelocity != 0.0 && rightVelocity != 0.0 &&
                   std::isfinite(leftVelocity) &&
                   std::isfinite(rightVelocity) &&
                   std::signbit(leftVelocity) ==
                       std::signbit(rightVelocity);
        };
    const auto endpointVelocity =
        [&](double adjacentDuration,
            double nextDuration,
            double adjacentVelocity,
            double nextVelocity) {
            const double denominator =
                adjacentDuration + nextDuration;
            if (denominator <= 0.0 ||
                !std::isfinite(denominator)) {
                return adjacentVelocity;
            }
            double velocity =
                ((2.0 * adjacentDuration + nextDuration) *
                     adjacentVelocity -
                 adjacentDuration * nextVelocity) /
                denominator;
            if (!continuesInSameDirection(
                    velocity,
                    adjacentVelocity)) {
                return 0.0;
            }
            if (!continuesInSameDirection(
                    adjacentVelocity,
                    nextVelocity) &&
                std::abs(velocity) >
                    3.0 * std::abs(adjacentVelocity)) {
                velocity = 3.0 * adjacentVelocity;
            }
            return velocity;
        };
    const auto tangentAt = [&](std::size_t index) {
        if (keyCount == 2U) {
            return intervalVelocity(0U);
        }
        if (index == 0U) {
            if (keyAt(0U).interpolation !=
                WaterScenarioInterpolation::SmoothVelocity) {
                return 0.0;
            }
            return endpointVelocity(
                intervalDuration(0U),
                intervalDuration(1U),
                intervalVelocity(0U),
                intervalVelocity(1U));
        }
        if (index + 1U == keyCount) {
            if (keyAt(keyCount - 2U).interpolation !=
                WaterScenarioInterpolation::SmoothVelocity) {
                return 0.0;
            }
            return endpointVelocity(
                intervalDuration(keyCount - 2U),
                intervalDuration(keyCount - 3U),
                intervalVelocity(keyCount - 2U),
                intervalVelocity(keyCount - 3U));
        }
        if (keyAt(index - 1U).interpolation !=
                WaterScenarioInterpolation::SmoothVelocity ||
            keyAt(index).interpolation !=
                WaterScenarioInterpolation::SmoothVelocity) {
            // Changing interpolation styles is an authored velocity break.
            return 0.0;
        }
        const double previousVelocity =
            intervalVelocity(index - 1U);
        const double nextVelocity = intervalVelocity(index);
        if (!continuesInSameDirection(
                previousVelocity,
                nextVelocity)) {
            // Local extrema and flat holds deliberately come to rest.
            return 0.0;
        }
        const double previousDuration =
            intervalDuration(index - 1U);
        const double nextDuration = intervalDuration(index);
        const double previousWeight =
            2.0 * nextDuration + previousDuration;
        const double nextWeight =
            nextDuration + 2.0 * previousDuration;
        return (previousWeight + nextWeight) /
               (previousWeight / previousVelocity +
                nextWeight / nextVelocity);
    };

    const auto leftIndex = static_cast<std::size_t>(
        std::distance(trackBegin, left));
    const double leftTangent = tangentAt(leftIndex);
    const double rightTangent = tangentAt(leftIndex + 1U);
    const double amountSquared = amount * amount;
    const double amountCubed = amountSquared * amount;
    const double leftValue = static_cast<double>(valueAt(*left));
    const double rightValue = static_cast<double>(valueAt(*right));
    const double evaluated =
        (2.0 * amountCubed - 3.0 * amountSquared + 1.0) *
            leftValue +
        (amountCubed - 2.0 * amountSquared + amount) *
            segmentDuration * leftTangent +
        (-2.0 * amountCubed + 3.0 * amountSquared) *
            rightValue +
        (amountCubed - amountSquared) * segmentDuration *
            rightTangent;
    const float result = static_cast<float>(evaluated);
    return std::isfinite(result)
               ? result
               : std::lerp(
                     valueAt(*left),
                     valueAt(*right),
                     static_cast<float>(amount));
}

std::optional<float> EvaluateEffectParameterTrack(
    const std::vector<TimingColouriseEffectParameterKey>& keys,
    TimingColouriseEffectParameter parameter,
    float normalizedPosition,
    float palettePhaseBaseValue,
    bool palettePhaseKeysAreAbsolute = false) {
    // Sanitization groups each parameter's keys and orders them by time.
    const auto trackBegin = std::find_if(
        keys.begin(),
        keys.end(),
        [&](const auto& key) { return key.parameter == parameter; });
    const auto trackEnd = std::find_if(
        trackBegin,
        keys.end(),
        [&](const auto& key) { return key.parameter != parameter; });
    if (parameter == TimingColouriseEffectParameter::PalettePhase &&
        !palettePhaseKeysAreAbsolute) {
        std::vector<TimingColouriseEffectParameterKey> accumulated;
        accumulated.reserve(static_cast<std::size_t>(
            std::distance(trackBegin, trackEnd)));
        float phase = FiniteOr(palettePhaseBaseValue, 0.0F);
        for (auto key = trackBegin; key != trackEnd; ++key) {
            phase += ClampPalettePhaseDelta(key->value);
            auto absoluteKey = *key;
            absoluteKey.value = phase;
            accumulated.push_back(std::move(absoluteKey));
        }
        return EvaluateScalarKeyTrack(
            accumulated.begin(),
            accumulated.end(),
            normalizedPosition,
            [](const auto& key) { return key.value; });
    }
    return EvaluateScalarKeyTrack(
        trackBegin,
        trackEnd,
        normalizedPosition,
        [](const auto& key) { return key.value; });
}

template <typename Key, typename Value, typename Sanitize>
void AddOrUpdateKey(
    std::vector<Key>* keys,
    float position,
    Value value,
    invisible_places::water::WaterScenarioInterpolation interpolation,
    Sanitize sanitize) {
    if (keys == nullptr) {
        return;
    }
    Key key{
        .position = Clamp01(position),
        .interpolation = interpolation,
    };
    if constexpr (requires { key.palette; }) {
        key.palette = sanitize(std::move(value));
    } else {
        key.bounds = sanitize(std::move(value));
    }
    const auto existing = std::find_if(
        keys->begin(),
        keys->end(),
        [&](const Key& candidate) {
            return std::abs(candidate.position - key.position) <=
                   kTimingColouriseKeyTolerance;
        });
    if (existing != keys->end()) {
        *existing = std::move(key);
    } else {
        keys->push_back(std::move(key));
    }
    SortAndCoalesceKeys(keys);
}

template <typename Key>
std::pair<const Key*, const Key*> BracketingKeys(
    const std::vector<Key>& keys,
    float normalizedPosition);

std::optional<float> EvaluateBoundsParameterTrack(
    const std::vector<TimingColouriseBoundsParameterKey>& keys,
    TimingColouriseBoundsParameter parameter,
    float normalizedPosition) {
    // Sanitization groups each parameter's keys and orders them by time.
    const auto trackBegin = std::find_if(
        keys.begin(),
        keys.end(),
        [&](const auto& key) { return key.parameter == parameter; });
    const auto trackEnd = std::find_if(
        trackBegin,
        keys.end(),
        [&](const auto& key) { return key.parameter != parameter; });
    return EvaluateScalarKeyTrack(
        trackBegin,
        trackEnd,
        normalizedPosition,
        [](const auto& key) { return key.value; });
}

// Sanitized keys are sorted by (stop, parameter, position), so one track is
// a contiguous range; find it so the shared scalar evaluator (and its
// Monotone Spline / Centripetal Catmull-Rom modes) can run over iterators.
std::pair<
    std::vector<TimingColourisePaletteStopParameterKey>::const_iterator,
    std::vector<TimingColourisePaletteStopParameterKey>::const_iterator>
PaletteStopParameterTrackRange(
    const std::vector<TimingColourisePaletteStopParameterKey>& keys,
    std::string_view stopId,
    TimingColourisePaletteStopParameter parameter) {
    const auto matches =
        [&](const TimingColourisePaletteStopParameterKey& key) {
            return key.stopId == stopId && key.parameter == parameter;
        };
    const auto begin = std::find_if(keys.begin(), keys.end(), matches);
    auto end = begin;
    while (end != keys.end() && matches(*end)) {
        ++end;
    }
    return {begin, end};
}

std::optional<float> EvaluatePaletteStopScalarTrack(
    const std::vector<TimingColourisePaletteStopParameterKey>& keys,
    std::string_view stopId,
    TimingColourisePaletteStopParameter parameter,
    float normalizedPosition) {
    const auto [begin, end] = PaletteStopParameterTrackRange(
        keys,
        stopId,
        parameter);
    if (begin == end) {
        return std::nullopt;
    }
    return EvaluateScalarKeyTrack(
        begin,
        end,
        normalizedPosition,
        [](const TimingColourisePaletteStopParameterKey& key) {
            return key.scalarValue;
        });
}

std::optional<std::array<float, 3>> EvaluatePaletteStopColourTrack(
    const std::vector<TimingColourisePaletteStopParameterKey>& keys,
    std::string_view stopId,
    float normalizedPosition,
    TimingColouriseColourSpace space) {
    const auto [begin, end] = PaletteStopParameterTrackRange(
        keys,
        stopId,
        TimingColourisePaletteStopParameter::Colour);
    if (begin == end) {
        return std::nullopt;
    }
    // Each channel follows the shared scalar evaluator in the chosen colour
    // space, so keyed colours honour every curve style and blend along an
    // sRGB, linear-light, or perceptual OkLab path.
    std::array<float, 3> coordinates{};
    for (std::size_t channel = 0U;
         channel < coordinates.size();
         ++channel) {
        coordinates[channel] =
            EvaluateScalarKeyTrack(
                begin,
                end,
                normalizedPosition,
                [&](const TimingColourisePaletteStopParameterKey& key) {
                    return TimingColouriseColourToSpace(
                        key.colourValue,
                        space)[channel];
                })
                .value_or(0.0F);
    }
    return TimingColouriseColourFromSpace(coordinates, space);
}

TimingColouriseBounds EvaluateLegacyTimingColouriseBounds(
    const TimingColouriseEffect& effect,
    float normalizedPosition) {
    if (effect.boundsKeys.empty()) {
        return effect.baseBounds;
    }
    const auto [left, right] =
        BracketingKeys(effect.boundsKeys, normalizedPosition);
    if (left == right) {
        return left->bounds;
    }
    const float span =
        std::max(1.0e-6F, right->position - left->position);
    const float amount = InterpolationAmount(
        left->interpolation,
        (normalizedPosition - left->position) / span);
    return SanitizeTimingColouriseBounds(TimingColouriseBounds{
        .lower = std::lerp(left->bounds.lower, right->bounds.lower, amount),
        .upper = std::lerp(left->bounds.upper, right->bounds.upper, amount),
        .edgeFadeLower = std::lerp(
            left->bounds.edgeFadeLower,
            right->bounds.edgeFadeLower,
            amount),
        .edgeFadeUpper = std::lerp(
            left->bounds.edgeFadeUpper,
            right->bounds.edgeFadeUpper,
            amount),
    });
}

template <typename Key>
bool MoveKey(
    std::vector<Key>* keys,
    float sourcePosition,
    float destinationPosition) {
    if (keys == nullptr || !std::isfinite(sourcePosition) ||
        !std::isfinite(destinationPosition) || destinationPosition < 0.0F ||
        destinationPosition > 1.0F) {
        return false;
    }
    const auto source = std::find_if(
        keys->begin(),
        keys->end(),
        [&](const Key& key) {
            return std::abs(key.position - sourcePosition) <=
                   kTimingColouriseKeyTolerance;
        });
    if (source == keys->end()) {
        return false;
    }
    const bool occupied = std::any_of(
        keys->begin(),
        keys->end(),
        [&](const Key& key) {
            return &key != &*source &&
                   std::abs(key.position - destinationPosition) <=
                       kTimingColouriseKeyTolerance;
        });
    if (occupied) {
        return false;
    }
    source->position = destinationPosition;
    std::stable_sort(
        keys->begin(),
        keys->end(),
        [](const Key& left, const Key& right) {
            return left.position < right.position;
        });
    return true;
}

template <typename Key>
std::size_t KeyCountAtPosition(
    const std::vector<Key>& keys,
    float position) {
    if (!std::isfinite(position)) {
        return 0U;
    }
    return static_cast<std::size_t>(std::count_if(
        keys.begin(),
        keys.end(),
        [&](const Key& key) {
            return std::abs(key.position - position) <=
                   kTimingColouriseKeyTolerance;
        }));
}

template <typename Key>
std::size_t RemoveKeysAtPosition(
    std::vector<Key>* keys,
    float position) {
    if (keys == nullptr || !std::isfinite(position)) {
        return 0U;
    }
    const auto previousSize = keys->size();
    std::erase_if(
        *keys,
        [&](const Key& key) {
            return std::abs(key.position - position) <=
                   kTimingColouriseKeyTolerance;
        });
    return previousSize - keys->size();
}

struct PaletteKeyPositionCluster {
    float representative = 0.0F;
    float minimum = 0.0F;
    float maximum = 0.0F;
};

std::vector<PaletteKeyPositionCluster> PaletteKeyPositionClusters(
    const TimingColouriseEffect& effect) {
    std::vector<float> positions;
    positions.reserve(
        effect.paletteKeys.size() +
        effect.paletteStopParameterKeys.size());
    for (const auto& key : effect.paletteKeys) {
        if (std::isfinite(key.position)) {
            positions.push_back(Clamp01(key.position));
        }
    }
    for (const auto& key : effect.paletteStopParameterKeys) {
        if (std::isfinite(key.position)) {
            positions.push_back(Clamp01(key.position));
        }
    }
    std::stable_sort(positions.begin(), positions.end());

    std::vector<PaletteKeyPositionCluster> clusters;
    clusters.reserve(positions.size());
    for (const float position : positions) {
        if (clusters.empty() ||
            position - clusters.back().maximum >
                kTimingColouriseKeyTolerance) {
            clusters.push_back({
                .representative = position,
                .minimum = position,
                .maximum = position,
            });
        } else {
            // Use adjacent-distance (single-linkage) clustering so a chain of
            // mutually tolerant keys cannot be split between two markers.
            clusters.back().maximum = position;
        }
    }
    return clusters;
}

float DistanceFromPaletteKeyPositionCluster(
    const PaletteKeyPositionCluster& cluster,
    float position) {
    if (position < cluster.minimum) {
        return cluster.minimum - position;
    }
    if (position > cluster.maximum) {
        return position - cluster.maximum;
    }
    return 0.0F;
}

std::optional<std::size_t> PaletteKeyPositionClusterIndexAtPosition(
    std::span<const PaletteKeyPositionCluster> clusters,
    float position) {
    if (!std::isfinite(position)) {
        return std::nullopt;
    }
    std::optional<std::size_t> best;
    float bestDistance = std::numeric_limits<float>::infinity();
    for (std::size_t index = 0U; index < clusters.size(); ++index) {
        const float distance =
            DistanceFromPaletteKeyPositionCluster(clusters[index], position);
        if (distance <= kTimingColouriseKeyTolerance &&
            distance < bestDistance) {
            best = index;
            bestDistance = distance;
        }
    }
    return best;
}

bool PaletteKeyPositionBelongsToCluster(
    float position,
    const PaletteKeyPositionCluster& cluster) {
    if (!std::isfinite(position)) {
        return false;
    }
    position = Clamp01(position);
    return position >= cluster.minimum && position <= cluster.maximum;
}

template <typename Key>
std::optional<float> PreviousKeyPosition(
    const std::vector<Key>& keys,
    float position) {
    if (!std::isfinite(position)) {
        return std::nullopt;
    }
    std::optional<float> best;
    for (const auto& key : keys) {
        if (key.position < position - kTimingColouriseKeyTolerance &&
            (!best.has_value() || key.position > *best)) {
            best = key.position;
        }
    }
    return best;
}

template <typename Key>
std::optional<float> NextKeyPosition(
    const std::vector<Key>& keys,
    float position) {
    if (!std::isfinite(position)) {
        return std::nullopt;
    }
    std::optional<float> best;
    for (const auto& key : keys) {
        if (key.position > position + kTimingColouriseKeyTolerance &&
            (!best.has_value() || key.position < *best)) {
            best = key.position;
        }
    }
    return best;
}

template <typename Key>
std::pair<const Key*, const Key*> BracketingKeys(
    const std::vector<Key>& keys,
    float normalizedPosition) {
    if (keys.empty()) {
        return {nullptr, nullptr};
    }
    if (normalizedPosition <= keys.front().position) {
        return {&keys.front(), &keys.front()};
    }
    if (normalizedPosition >= keys.back().position) {
        return {&keys.back(), &keys.back()};
    }
    for (std::size_t index = 0U; index + 1U < keys.size(); ++index) {
        if (normalizedPosition < keys[index + 1U].position) {
            return {&keys[index], &keys[index + 1U]};
        }
    }
    return {&keys.back(), &keys.back()};
}

std::array<float, 4> SampleSanitizedPalette(
    const TimingColourisePalette& palette,
    float position) {
    const auto& stops = palette.stops;
    if (stops.size() == 1U || position <= stops.front().position) {
        return {
            stops.front().colour[0],
            stops.front().colour[1],
            stops.front().colour[2],
            stops.front().colouriseAmount};
    }
    if (position >= stops.back().position) {
        return {
            stops.back().colour[0],
            stops.back().colour[1],
            stops.back().colour[2],
            stops.back().colouriseAmount};
    }
    for (std::size_t index = 0U; index + 1U < stops.size(); ++index) {
        const auto& left = stops[index];
        const auto& right = stops[index + 1U];
        if (position > right.position) {
            continue;
        }
        const float span = right.position - left.position;
        if (span <= std::numeric_limits<float>::epsilon()) {
            return {
                right.colour[0],
                right.colour[1],
                right.colour[2],
                right.colouriseAmount};
        }
        const float amount = Clamp01((position - left.position) / span);
        return {
            std::lerp(left.colour[0], right.colour[0], amount),
            std::lerp(left.colour[1], right.colour[1], amount),
            std::lerp(left.colour[2], right.colour[2], amount),
            std::lerp(
                left.colouriseAmount,
                right.colouriseAmount,
                amount)};
    }
    return {
        stops.back().colour[0],
        stops.back().colour[1],
        stops.back().colour[2],
        stops.back().colouriseAmount};
}

template <typename Item>
std::string AllocateSequentialId(
    std::span<const Item> items,
    std::uint32_t* nextSequence,
    std::string_view prefix) {
    std::uint32_t sequence =
        nextSequence == nullptr ? 1U : std::max(1U, *nextSequence);
    while (true) {
        const std::string candidate =
            std::string{prefix} + std::to_string(sequence++);
        const bool exists = std::any_of(
            items.begin(),
            items.end(),
            [&](const Item& item) {
                return item.id == candidate;
            });
        if (!exists) {
            if (nextSequence != nullptr) {
                *nextSequence = sequence;
            }
            return candidate;
        }
    }
}

}  // namespace

std::string NormalizeTimingTakeId(std::string_view takeId) {
    if (takeId.empty()) {
        return std::string{kAuthoredTimingTakeId};
    }
    return std::string{takeId};
}

TimingTakeDefinition AuthoredTimingTakeDefinition() {
    return TimingTakeDefinition{
        .id = std::string{kAuthoredTimingTakeId},
        .name = std::string{kAuthoredTimingTakeName},
    };
}

const invisible_places::water::WaterRainProfile*
ResolveTimingTakeRainProfile(
    std::span<const invisible_places::water::WaterRainProfile> profiles,
    const TimingTakeDefinition& take) {
    if (const auto* assigned = FindAssignedRainProfile(profiles, take);
        assigned != nullptr) {
        return assigned;
    }
    if (const auto* base = FindExplicitRainBaseProfile(
            profiles,
            take.baseRainProfileId,
            take.baseRainProfileName);
        base != nullptr) {
        return base;
    }
    return FirstSharedRainProfile(profiles);
}

const invisible_places::water::WaterRainProfile*
ResolveTimingTakeRainBaseProfile(
    std::span<const invisible_places::water::WaterRainProfile> profiles,
    const TimingTakeDefinition& take) {
    if (const auto* base = FindExplicitRainBaseProfile(
            profiles,
            take.baseRainProfileId,
            take.baseRainProfileName);
        base != nullptr) {
        return base;
    }
    if (const auto* assigned = FindAssignedRainProfile(profiles, take);
        assigned != nullptr) {
        if (!assigned->objectOverride) {
            return assigned;
        }
        if (const auto* base = FindExplicitRainBaseProfile(
                profiles,
                assigned->baseProfileId,
                assigned->baseProfileName);
            base != nullptr) {
            return base;
        }
    }
    return FirstSharedRainProfile(profiles);
}

std::optional<invisible_places::water::WaterRainProfile>
CaptureTimingTakeRainProfileSnapshot(
    std::span<const invisible_places::water::WaterRainProfile> profiles,
    std::span<const TimingTakeDefinition> takes,
    std::string_view takeId) {
    const auto* take = FindTimingTakeDefinition(takes, takeId);
    if (take == nullptr) {
        return std::nullopt;
    }
    const auto* effective = ResolveTimingTakeRainProfile(profiles, *take);
    return effective != nullptr
               ? std::optional<invisible_places::water::WaterRainProfile>{
                     *effective}
               : std::nullopt;
}

float TimingTakeRainAuthoredLevel(
    const invisible_places::water::RainRuntimeSettings& settings) {
    return settings.enabled
               ? std::clamp(settings.rainLevel, 0.0F, 1.0F)
               : 0.0F;
}

std::optional<invisible_places::water::WaterScenarioState>
ProjectTimingTakeRainToScenarioSnapshot(
    const std::optional<invisible_places::water::WaterScenarioState>& scenario,
    const invisible_places::water::RainRuntimeSettings& settings) {
    auto result = scenario;
    if (result.has_value()) {
        result->rainLevel = TimingTakeRainAuthoredLevel(settings);
    }
    return result;
}

TimingTakeRainLiveSyncDecision ResolveTimingTakeRainLiveSyncDecision(
    std::string_view boundTakeId,
    std::string_view boundProfileId,
    std::string_view resolvedTakeId,
    std::string_view resolvedProfileId,
    bool forceRefresh) {
    const bool identityChanged =
        boundTakeId != resolvedTakeId ||
        boundProfileId != resolvedProfileId;
    const bool synchronize = forceRefresh || identityChanged;
    return {
        .copyProfile = synchronize,
        .resetRuntime = synchronize,
    };
}

TimingWaterProfileReferenceRewriteCounts
ReplaceTimingWaterProfileReferences(
    std::span<invisible_places::water::WaterScenarioFeatureRuns>
        legacyScenarios,
    std::span<TimingTakeSceneState> timingTakeStates,
    std::span<invisible_places::water::WaterKeyedSettingsProfile>
        keyedPackages,
    std::string_view profileGroup,
    std::string_view previousProfileName,
    std::string_view nextProfileName) {
    TimingWaterProfileReferenceRewriteCounts counts;
    const auto replaceInRuns = [&](auto& runs, std::size_t* count) {
        for (auto& run : runs) {
            for (auto& feature : run.features) {
                *count += invisible_places::water::
                    ReplaceWaterKeyedSettingProfileReferences(
                        feature.settings,
                        feature.feature.kind,
                        profileGroup,
                        previousProfileName,
                        nextProfileName);
            }
        }
    };
    for (auto& scenario : legacyScenarios) {
        replaceInRuns(
            scenario.runs,
            &counts.legacyScenarioTracks);
    }
    for (auto& state : timingTakeStates) {
        replaceInRuns(
            state.waterFeatureTimingRuns,
            &counts.timingTakeTracks);
    }
    for (auto& package : keyedPackages) {
        counts.keyedPackageTracks += invisible_places::water::
            ReplaceWaterKeyedSettingProfileReferences(
                package.settings,
                package.featureKind,
                profileGroup,
                previousProfileName,
                nextProfileName);
    }
    return counts;
}

std::size_t CanonicalizeTimingWaterFeatureProfileMetadata(
    std::span<invisible_places::water::WaterScenarioFeatureRuns>
        legacyScenarios,
    std::span<TimingTakeSceneState> timingTakeStates,
    std::span<const invisible_places::water::WaterKeyedFeatureId> features,
    std::string_view profileGroup,
    std::string_view profileName) {
    std::size_t changed = 0U;
    for (auto& scenario : legacyScenarios) {
        changed += invisible_places::water::
            CanonicalizeWaterFeatureProfileMetadata(
                scenario.runs,
                features,
                profileGroup,
                profileName);
    }
    for (auto& state : timingTakeStates) {
        changed += invisible_places::water::
            CanonicalizeWaterFeatureProfileMetadata(
                state.waterFeatureTimingRuns,
                features,
                profileGroup,
                profileName);
    }
    return changed;
}

std::size_t RewriteTimingTakeRainTrackProfileMetadata(
    std::vector<TimingTakeSceneState>* states,
    std::string_view takeId,
    std::string_view profileName) {
    if (states == nullptr || profileName.empty()) {
        return 0U;
    }
    const auto normalizedTakeId = NormalizeTimingTakeId(takeId);
    std::size_t changed = 0U;
    for (auto& state : *states) {
        if (NormalizeTimingTakeId(state.takeId) != normalizedTakeId) {
            continue;
        }
        for (auto& run : state.waterFeatureTimingRuns) {
            for (auto& timeline : run.features) {
                if (timeline.feature.kind !=
                    invisible_places::water::WaterKeyedFeatureKind::Rain) {
                    continue;
                }
                for (auto& track : timeline.settings) {
                    if (track.profileGroup !=
                            kTimingTakeRainTrackProfileGroup ||
                        track.profileName != profileName) {
                        track.profileGroup = std::string{
                            kTimingTakeRainTrackProfileGroup};
                        track.profileName = std::string{profileName};
                        ++changed;
                    }
                }
            }
        }
    }
    return changed;
}

std::size_t RewriteLegacyScenarioRainTrackProfileMetadata(
    std::vector<invisible_places::water::WaterScenarioFeatureRuns>* scenarios,
    std::span<const invisible_places::water::WaterRainProfile> profiles,
    std::span<const TimingTakeDefinition> takes,
    std::string_view legacyProfileName) {
    if (scenarios == nullptr || legacyProfileName.empty()) {
        return 0U;
    }
    std::string legacyResolvedProfileName{legacyProfileName};
    const invisible_places::water::WaterRainProfile* legacyProfile = nullptr;
    if (const auto* authored = FindTimingTakeDefinition(
            takes,
            kAuthoredTimingTakeId);
        authored != nullptr) {
        legacyProfile = ResolveTimingTakeRainProfile(
            profiles,
            *authored);
    }
    if (legacyProfile == nullptr) {
        legacyProfile = FirstSharedRainProfile(profiles);
    }
    if (legacyProfile != nullptr) {
        legacyResolvedProfileName = legacyProfile->name;
    }
    std::size_t changed = 0U;
    for (auto& scenario : *scenarios) {
        std::string profileName{legacyResolvedProfileName};
        if (const auto* take = FindTimingTakeDefinition(
                takes,
                scenario.scenarioId);
            take != nullptr) {
            if (const auto* profile = ResolveTimingTakeRainProfile(
                    profiles,
                    *take);
                profile != nullptr) {
                profileName = profile->name;
            }
        }
        for (auto& run : scenario.runs) {
            for (auto& timeline : run.features) {
                if (timeline.feature.kind !=
                    invisible_places::water::WaterKeyedFeatureKind::Rain) {
                    continue;
                }
                for (auto& track : timeline.settings) {
                    if (track.profileGroup !=
                            kTimingTakeRainTrackProfileGroup ||
                        track.profileName != profileName) {
                        track.profileGroup = std::string{
                            kTimingTakeRainTrackProfileGroup};
                        track.profileName = profileName;
                        ++changed;
                    }
                }
            }
        }
    }
    return changed;
}

bool AssignTimingTakeRainBaseProfile(
    TimingTakeDefinition* take,
    std::span<const invisible_places::water::WaterRainProfile> profiles,
    std::string_view baseProfileId) {
    if (take == nullptr) {
        return false;
    }
    const auto* base = invisible_places::water::FindWaterRainProfileById(
        profiles,
        baseProfileId);
    if (base == nullptr || base->objectOverride) {
        return false;
    }
    SetTimingTakeRainAssignment(take, *base, *base);
    return true;
}

void SanitizeWaterRainProfileLibrary(
    std::vector<invisible_places::water::WaterRainProfile>* profiles,
    std::vector<TimingTakeDefinition>* takes) {
    if (profiles == nullptr) {
        return;
    }
    struct NormalizationRecord {
        std::string oldId;
        std::string oldName;
        std::string oldBaseId;
        std::string oldBaseName;
        bool shared = true;
    };
    auto input = std::move(*profiles);
    profiles->clear();
    profiles->reserve(input.size());
    for (auto& candidate : input) {
        candidate = invisible_places::water::SanitizeWaterRainProfile(
            std::move(candidate));
    }
    // Do not let an earlier duplicate steal a suffix that is already the
    // stable id/name of a later entry in the file.
    const auto reservedInput = input;
    std::vector<NormalizationRecord> records;
    records.reserve(input.size());
    for (auto& candidate : input) {
        NormalizationRecord record{
            .oldId = candidate.id,
            .oldName = candidate.name,
            .oldBaseId = candidate.baseProfileId,
            .oldBaseName = candidate.baseProfileName,
            .shared = !candidate.objectOverride,
        };
        if (candidate.id.empty() ||
            invisible_places::water::FindWaterRainProfileById(
                *profiles,
                candidate.id) != nullptr) {
            auto allocationScope = *profiles;
            allocationScope.insert(
                allocationScope.end(),
                reservedInput.begin(),
                reservedInput.end());
            candidate.id =
                invisible_places::water::AllocateWaterRainProfileId(
                    allocationScope,
                    candidate.id.empty() ? candidate.name : candidate.id);
        }
        if (invisible_places::water::FindWaterRainProfileByName(
                *profiles,
                candidate.name) != nullptr) {
            auto allocationScope = *profiles;
            allocationScope.insert(
                allocationScope.end(),
                reservedInput.begin(),
                reservedInput.end());
            candidate.name = UniqueRainProfileName(
                allocationScope,
                candidate.name);
        }
        profiles->push_back(std::move(candidate));
        records.push_back(std::move(record));
    }

    const auto resolveRecord = [&records, profiles](
                                   std::string_view oldId,
                                   std::string_view oldName,
                                   bool sharedOnly)
        -> const invisible_places::water::WaterRainProfile* {
        std::vector<std::size_t> candidates;
        if (!TrimRainProfileText(oldId).empty()) {
            for (std::size_t index = 0U; index < records.size(); ++index) {
                if ((!sharedOnly || records[index].shared) &&
                    records[index].oldId == oldId) {
                    candidates.push_back(index);
                }
            }
        }
        if (candidates.size() > 1U &&
            !TrimRainProfileText(oldName).empty()) {
            const auto wanted = RainProfileNameKey(oldName);
            const auto matchingName = std::find_if(
                candidates.begin(),
                candidates.end(),
                [&](std::size_t index) {
                    return RainProfileNameKey(records[index].oldName) ==
                           wanted;
                });
            if (matchingName != candidates.end()) {
                return &profiles->at(*matchingName);
            }
        }
        if (!candidates.empty()) {
            return &profiles->at(candidates.front());
        }
        if (!TrimRainProfileText(oldName).empty()) {
            const auto wanted = RainProfileNameKey(oldName);
            for (std::size_t index = 0U; index < records.size(); ++index) {
                if ((!sharedOnly || records[index].shared) &&
                    RainProfileNameKey(records[index].oldName) == wanted) {
                    return &profiles->at(index);
                }
            }
        }
        return nullptr;
    };

    for (std::size_t index = 0U; index < profiles->size(); ++index) {
        auto& profile = profiles->at(index);
        if (!profile.objectOverride) {
            continue;
        }
        if (const auto* base = resolveRecord(
                records[index].oldBaseId,
                records[index].oldBaseName,
                true);
            base != nullptr) {
            profile.baseProfileId = base->id;
            profile.baseProfileName = base->name;
        }
    }

    if (takes == nullptr) {
        return;
    }
    for (auto& take : *takes) {
        take = SanitizeTimingTakeDefinition(std::move(take));
        if (const auto* assigned = resolveRecord(
                take.assignedRainProfileId,
                take.assignedRainProfileName,
                false);
            assigned != nullptr) {
            take.assignedRainProfileId = assigned->id;
            take.assignedRainProfileName = assigned->name;
        }
        if (const auto* base = resolveRecord(
                take.baseRainProfileId,
                take.baseRainProfileName,
                true);
            base != nullptr) {
            take.baseRainProfileId = base->id;
            take.baseRainProfileName = base->name;
        } else if (const auto* resolvedBase =
                       ResolveTimingTakeRainBaseProfile(*profiles, take);
                   resolvedBase != nullptr) {
            take.baseRainProfileId = resolvedBase->id;
            take.baseRainProfileName = resolvedBase->name;
        }
    }
}

std::string EnsureLegacyWaterRainProfile(
    std::vector<invisible_places::water::WaterRainProfile>* profiles,
    std::vector<TimingTakeDefinition>* takes,
    const invisible_places::water::RainRuntimeSettings& legacySettings,
    const invisible_places::water::WaterRainVisualSettings& legacyVisual,
    std::string_view preferredName) {
    if (profiles == nullptr) {
        return {};
    }
    SanitizeWaterRainProfileLibrary(profiles, takes);

    const invisible_places::water::WaterRainProfile* shared =
        invisible_places::water::FindWaterRainProfileById(
            *profiles,
            kLegacyWaterRainProfileId);
    if (shared != nullptr && shared->objectOverride) {
        shared = nullptr;
    }
    if (shared == nullptr) {
        shared = FirstSharedRainProfile(*profiles);
    }
    if (shared == nullptr) {
        invisible_places::water::WaterRainProfile migrated;
        migrated.id = invisible_places::water::AllocateWaterRainProfileId(
            *profiles,
            kLegacyWaterRainProfileId);
        migrated.name = UniqueRainProfileName(*profiles, preferredName);
        // Preserve the legacy authored snapshot exactly, including edited
        // visual values whose visualProfileName still names a built-in.
        migrated.settings = legacySettings;
        migrated.visual = legacyVisual;
        profiles->push_back(std::move(migrated));
        shared = &profiles->back();
    }

    const std::string sharedId = shared->id;
    if (takes != nullptr) {
        for (auto& take : *takes) {
            take = SanitizeTimingTakeDefinition(std::move(take));
            if (FindAssignedRainProfile(*profiles, take) == nullptr) {
                AssignTimingTakeRainBaseProfile(
                    &take,
                    *profiles,
                    sharedId);
            }
        }
    }
    return sharedId;
}

TimingTakeRainStandaloneExportState
BuildTimingTakeRainStandaloneExportState(
    std::span<const invisible_places::water::WaterRainProfile> profiles,
    std::span<const TimingTakeDefinition> takes,
    std::string_view activeTakeId,
    const invisible_places::water::RainRuntimeSettings& fallbackSettings,
    const invisible_places::water::WaterRainVisualSettings& fallbackVisual) {
    TimingTakeRainStandaloneExportState result{
        .profiles = {profiles.begin(), profiles.end()},
        .assignments = {takes.begin(), takes.end()},
        .compatibilitySettings = fallbackSettings,
        .compatibilityVisual = fallbackVisual,
    };
    (void)EnsureLegacyWaterRainProfile(
        &result.profiles,
        &result.assignments,
        fallbackSettings,
        fallbackVisual);
    const auto* active = FindTimingTakeDefinition(
        result.assignments,
        activeTakeId);
    if (active == nullptr) {
        active = FindTimingTakeDefinition(
            result.assignments,
            kAuthoredTimingTakeId);
    }
    if (active == nullptr && !result.assignments.empty()) {
        active = &result.assignments.front();
    }
    if (active != nullptr) {
        if (const auto* effective = ResolveTimingTakeRainProfile(
                result.profiles,
                *active);
            effective != nullptr) {
            result.compatibilitySettings = effective->settings;
            result.compatibilityVisual = effective->visual;
        }
    }
    return result;
}

TimingTakeRainImportResult MergeImportedTimingTakeRainProfiles(
    std::vector<invisible_places::water::WaterRainProfile>* profiles,
    std::vector<TimingTakeDefinition>* takes,
    std::span<const invisible_places::water::WaterRainProfile>
        importedProfiles,
    std::span<const TimingTakeDefinition> importedAssignments,
    std::string_view legacyCompatibilityTakeId) {
    TimingTakeRainImportResult result;
    if (profiles == nullptr || takes == nullptr) {
        return result;
    }

    auto normalizedProfiles =
        std::vector<invisible_places::water::WaterRainProfile>{
            importedProfiles.begin(),
            importedProfiles.end()};
    // Normalize only the incoming payload. Existing project entries are
    // deliberately left byte-for-byte alone unless their stable id is
    // explicitly imported below.
    SanitizeWaterRainProfileLibrary(&normalizedProfiles, nullptr);

    // Schema <= 30 carried only one compatibility Rain snapshot and no take
    // assignments. Its synthesized legacy id is not an authoritative stable
    // identity: replacing an existing profile with that id would also change
    // every other take that shares the destination profile. Isolate a
    // collision before synthesizing the active-take assignment below.
    if (importedAssignments.empty() &&
        !legacyCompatibilityTakeId.empty()) {
        const auto legacyShared = std::find_if(
            normalizedProfiles.begin(),
            normalizedProfiles.end(),
            [](const invisible_places::water::WaterRainProfile& profile) {
                return !profile.objectOverride;
            });
        if (legacyShared != normalizedProfiles.end() &&
            invisible_places::water::FindWaterRainProfileById(
                *profiles,
                legacyShared->id) != nullptr) {
            const auto previousId = legacyShared->id;
            legacyShared->id =
                invisible_places::water::AllocateWaterRainProfileId(
                    *profiles,
                    previousId + "-" +
                        std::string{legacyCompatibilityTakeId});
            for (auto& profile : normalizedProfiles) {
                if (profile.objectOverride &&
                    profile.baseProfileId == previousId) {
                    profile.baseProfileId = legacyShared->id;
                    profile.baseProfileName = legacyShared->name;
                }
            }
        }
    }

    std::vector<TimingTakeDefinition> matchedAssignments;
    std::unordered_set<std::string> matchedTakeIds;
    const auto appendMatchedAssignment = [&](TimingTakeDefinition imported) {
        auto assignment = SanitizeTimingTakeDefinition(imported);
        if (FindTimingTakeDefinition(takes, assignment.id) == nullptr ||
            !matchedTakeIds.insert(assignment.id).second) {
            return;
        }
        matchedAssignments.push_back(std::move(assignment));
    };
    for (const auto& imported : importedAssignments) {
        appendMatchedAssignment(imported);
    }
    if (importedAssignments.empty() &&
        !legacyCompatibilityTakeId.empty()) {
        if (const auto* legacyShared =
                FirstSharedRainProfile(normalizedProfiles);
            legacyShared != nullptr) {
            appendMatchedAssignment({
                .id = std::string{legacyCompatibilityTakeId},
                .assignedRainProfileId = legacyShared->id,
                .assignedRainProfileName = legacyShared->name,
                .baseRainProfileId = legacyShared->id,
                .baseRainProfileName = legacyShared->name,
            });
        }
    }

    const auto assignmentReferences =
        [&](const invisible_places::water::WaterRainProfile& profile) {
            return std::any_of(
                matchedAssignments.begin(),
                matchedAssignments.end(),
                [&](const TimingTakeDefinition& assignment) {
                    if (!assignment.assignedRainProfileId.empty()) {
                        return assignment.assignedRainProfileId ==
                               profile.id;
                    }
                    return RainProfileNameKey(
                               assignment.assignedRainProfileName) ==
                           RainProfileNameKey(profile.name);
                });
        };

    std::vector<std::string> mergedProfileIds;
    mergedProfileIds.reserve(normalizedProfiles.size());
    std::unordered_set<std::string> insertedProfileIds;
    std::unordered_set<std::string> updatedProfileIds;
    for (auto candidate : normalizedProfiles) {
        if (candidate.objectOverride &&
            FindTimingTakeDefinition(
                *takes,
                candidate.ownerTimingTakeId) == nullptr &&
            !assignmentReferences(candidate)) {
            ++result.orphanOwnerProfilesSkipped;
            continue;
        }
        candidate.name = UniqueRainProfileName(
            *profiles,
            candidate.name,
            candidate.id);
        const auto candidateId = candidate.id;
        if (auto* existing =
                invisible_places::water::FindWaterRainProfileById(
                    profiles,
                    candidate.id);
            existing != nullptr) {
            if (*existing != candidate) {
                *existing = candidate;
                updatedProfileIds.insert(candidateId);
                ++result.profilesUpdated;
            }
        } else {
            profiles->push_back(std::move(candidate));
            insertedProfileIds.insert(candidateId);
            ++result.profilesInserted;
        }
        mergedProfileIds.push_back(candidateId);
    }

    const auto findStrictProfile =
        [&](std::string_view id,
            std::string_view name)
        -> const invisible_places::water::WaterRainProfile* {
            if (!TrimRainProfileText(id).empty()) {
                return invisible_places::water::FindWaterRainProfileById(
                    *profiles,
                    id);
            }
            return invisible_places::water::FindWaterRainProfileByName(
                *profiles,
                name);
        };

    const std::unordered_set<std::string> mergedProfileIdSet{
        mergedProfileIds.begin(),
        mergedProfileIds.end()};
    const auto recordProfileUpdate = [&](std::string_view profileId) {
        if (!insertedProfileIds.contains(std::string{profileId}) &&
            updatedProfileIds.insert(std::string{profileId}).second) {
            ++result.profilesUpdated;
        }
    };

    // Imported base-name mirrors follow the final stable-id destination,
    // including any collision-safe display-name suffix assigned above.
    for (const auto& profileId : mergedProfileIds) {
        auto* profile = invisible_places::water::FindWaterRainProfileById(
            profiles,
            profileId);
        if (profile == nullptr || !profile->objectOverride) {
            continue;
        }
        const auto* base = findStrictProfile(
            profile->baseProfileId,
            profile->baseProfileName);
        if (base == nullptr || base->objectOverride) {
            continue;
        }
        const bool changed =
            profile->baseProfileId != base->id ||
            profile->baseProfileName != base->name;
        profile->baseProfileId = base->id;
        profile->baseProfileName = base->name;
        if (changed) {
            recordProfileUpdate(profile->id);
        }
    }

    // A stable-id update may rename a shared base that already had local
    // owner copies. Repair only those dependent mirrors; unrelated profiles
    // remain untouched.
    for (auto& profile : *profiles) {
        if (!profile.objectOverride || profile.baseProfileId.empty() ||
            !mergedProfileIdSet.contains(profile.baseProfileId)) {
            continue;
        }
        const auto* base =
            invisible_places::water::FindWaterRainProfileById(
                *profiles,
                profile.baseProfileId);
        if (base == nullptr || base->objectOverride ||
            profile.baseProfileName == base->name) {
            continue;
        }
        profile.baseProfileName = base->name;
        recordProfileUpdate(profile.id);
    }

    std::unordered_set<std::string> changedAssignmentTakeIds;
    const auto recordAssignmentUpdate = [&](std::string_view takeId) {
        if (changedAssignmentTakeIds.insert(std::string{takeId}).second) {
            ++result.assignmentsApplied;
        }
    };
    // The same stable-id rename also repairs readable mirrors on local takes,
    // including observers not present in the standalone assignment list.
    for (auto& take : *takes) {
        bool changed = false;
        if (!take.assignedRainProfileId.empty() &&
            mergedProfileIdSet.contains(take.assignedRainProfileId)) {
            if (const auto* assigned =
                    invisible_places::water::FindWaterRainProfileById(
                        *profiles,
                        take.assignedRainProfileId);
                assigned != nullptr &&
                take.assignedRainProfileName != assigned->name) {
                take.assignedRainProfileName = assigned->name;
                changed = true;
            }
        }
        if (!take.baseRainProfileId.empty() &&
            mergedProfileIdSet.contains(take.baseRainProfileId)) {
            if (const auto* base =
                    invisible_places::water::FindWaterRainProfileById(
                        *profiles,
                        take.baseRainProfileId);
                base != nullptr && !base->objectOverride &&
                take.baseRainProfileName != base->name) {
                take.baseRainProfileName = base->name;
                changed = true;
            }
        }
        if (changed) {
            recordAssignmentUpdate(take.id);
        }
    }

    for (const auto& assignment : matchedAssignments) {
        auto* destination = FindTimingTakeDefinition(
            takes,
            assignment.id);
        if (destination == nullptr) {
            continue;
        }
        const auto* assigned = findStrictProfile(
            assignment.assignedRainProfileId,
            assignment.assignedRainProfileName);
        const auto* base = findStrictProfile(
            assignment.baseRainProfileId,
            assignment.baseRainProfileName);
        if (assigned != nullptr && !assigned->objectOverride) {
            base = assigned;
        } else if (assigned != nullptr && assigned->objectOverride &&
                   (base == nullptr || base->objectOverride)) {
            base = findStrictProfile(
                assigned->baseProfileId,
                assigned->baseProfileName);
        } else if (assigned == nullptr && base != nullptr &&
                   !base->objectOverride) {
            assigned = base;
        }
        if (assigned == nullptr || base == nullptr ||
            base->objectOverride) {
            continue;
        }

        const bool changed =
            destination->assignedRainProfileId != assigned->id ||
            destination->assignedRainProfileName != assigned->name ||
            destination->baseRainProfileId != base->id ||
            destination->baseRainProfileName != base->name;
        destination->assignedRainProfileId = assigned->id;
        destination->assignedRainProfileName = assigned->name;
        destination->baseRainProfileId = base->id;
        destination->baseRainProfileName = base->name;
        if (changed) {
            recordAssignmentUpdate(destination->id);
        }
    }
    return result;
}

std::string PredictTimingTakeRainOwnerProfileName(
    std::span<const invisible_places::water::WaterRainProfile> profiles,
    const TimingTakeDefinition& take) {
    const auto sanitizedTake = SanitizeTimingTakeDefinition(take);
    const auto* base = ResolveTimingTakeRainBaseProfile(
        profiles,
        sanitizedTake);
    if (base == nullptr) {
        return {};
    }
    if (const auto existing =
            TimingTakeRainOwnerProfileIndex(profiles, sanitizedTake, *base);
        existing.has_value()) {
        return profiles[existing.value()].name;
    }
    return UniqueRainProfileName(
        profiles,
        TimingTakeRainOwnerProfileName(*base, sanitizedTake));
}

invisible_places::water::WaterRainProfile*
UpsertTimingTakeRainOwnerProfile(
    std::vector<invisible_places::water::WaterRainProfile>* profiles,
    TimingTakeDefinition* take,
    const invisible_places::water::RainRuntimeSettings& settings,
    const invisible_places::water::WaterRainVisualSettings& visual) {
    if (profiles == nullptr || take == nullptr) {
        return nullptr;
    }
    *take = SanitizeTimingTakeDefinition(std::move(*take));
    const auto* resolvedBase = ResolveTimingTakeRainBaseProfile(
        *profiles,
        *take);
    if (resolvedBase == nullptr) {
        return nullptr;
    }
    const auto base = *resolvedBase;
    const auto existingIndex =
        TimingTakeRainOwnerProfileIndex(*profiles, *take, base);
    if (!existingIndex.has_value()) {
        invisible_places::water::WaterRainProfile copy;
        copy.id = invisible_places::water::AllocateWaterRainProfileId(
            *profiles,
            base.id + "-" + take->id);
        copy.name = PredictTimingTakeRainOwnerProfileName(
            *profiles,
            *take);
        copy.objectOverride = true;
        copy.ownerTimingTakeId = take->id;
        copy.baseProfileId = base.id;
        copy.baseProfileName = base.name;
        profiles->push_back(std::move(copy));
    }
    auto& owner = existingIndex.has_value()
                      ? (*profiles)[existingIndex.value()]
                      : profiles->back();
    owner.settings = settings;
    owner.visual = visual;
    owner.objectOverride = true;
    owner.ownerTimingTakeId = take->id;
    owner.baseProfileId = base.id;
    owner.baseProfileName = base.name;
    SetTimingTakeRainAssignment(take, owner, base);
    return &owner;
}

invisible_places::water::WaterRainProfile*
SaveTimingTakeRainOwnerProfileToBase(
    std::vector<invisible_places::water::WaterRainProfile>* profiles,
    std::vector<TimingTakeDefinition>* takes,
    std::string_view takeId) {
    return SaveTimingTakeRainOwnerProfileAsShared(
        profiles,
        takes,
        takeId,
        {},
        true);
}

invisible_places::water::WaterRainProfile*
SaveTimingTakeRainOwnerProfileAsShared(
    std::vector<invisible_places::water::WaterRainProfile>* profiles,
    std::vector<TimingTakeDefinition>* takes,
    std::string_view takeId,
    std::string_view requestedName,
    bool overwriteResolvedBase) {
    if (profiles == nullptr || takes == nullptr) {
        return nullptr;
    }
    auto* take = FindTimingTakeDefinition(takes, takeId);
    if (take == nullptr) {
        return nullptr;
    }
    const auto* effective = ResolveTimingTakeRainProfile(*profiles, *take);
    if (effective == nullptr) {
        return nullptr;
    }
    const invisible_places::water::WaterRainProfile* resolvedBase = nullptr;
    if (overwriteResolvedBase) {
        if (!effective->objectOverride ||
            effective->ownerTimingTakeId != take->id) {
            return nullptr;
        }
        const auto findSharedById = [&](std::string_view profileId) {
            const auto* profile =
                invisible_places::water::FindWaterRainProfileById(
                    *profiles,
                    profileId);
            return profile != nullptr && !profile->objectOverride
                       ? profile
                       : nullptr;
        };
        if (!effective->baseProfileId.empty()) {
            resolvedBase = findSharedById(effective->baseProfileId);
            if (resolvedBase == nullptr) {
                return nullptr;
            }
        } else if (!take->baseRainProfileId.empty()) {
            resolvedBase = findSharedById(take->baseRainProfileId);
            if (resolvedBase == nullptr) {
                return nullptr;
            }
        } else {
            const auto baseName =
                !TrimRainProfileText(effective->baseProfileName).empty()
                    ? effective->baseProfileName
                    : take->baseRainProfileName;
            resolvedBase =
                invisible_places::water::FindWaterRainProfileByName(
                    profiles,
                    baseName);
            if (resolvedBase == nullptr || resolvedBase->objectOverride) {
                return nullptr;
            }
        }
    } else {
        resolvedBase = ResolveTimingTakeRainBaseProfile(
            *profiles,
            *take);
    }
    const auto resolvedBaseId =
        resolvedBase != nullptr ? resolvedBase->id : std::string{};
    const auto resolvedBaseName =
        resolvedBase != nullptr ? resolvedBase->name : std::string{};
    const auto savedSettings = effective->settings;
    const auto savedVisual = effective->visual;
    const auto promotedOwnerId =
        effective->objectOverride &&
                effective->ownerTimingTakeId == take->id
            ? effective->id
            : std::string{};
    const auto promotedOwnerName =
        promotedOwnerId.empty() ? std::string{} : effective->name;

    invisible_places::water::WaterRainProfile* saved = nullptr;
    if (overwriteResolvedBase) {
        // Save is identity-based: an editable name field must never redirect
        // an overwrite to an unrelated shared profile with the same text.
        saved = invisible_places::water::FindWaterRainProfileById(
            profiles,
            resolvedBaseId);
        if (saved == nullptr || saved->objectOverride) {
            return nullptr;
        }
    } else {
        auto preferredName = TrimRainProfileText(requestedName);
        if (preferredName.empty()) {
            preferredName = !resolvedBaseName.empty()
                                ? resolvedBaseName
                                : effective->objectOverride
                                      ? effective->baseProfileName
                                      : effective->name;
        }
        invisible_places::water::WaterRainProfile shared;
        shared.name = UniqueRainProfileName(*profiles, preferredName);
        shared.id = invisible_places::water::AllocateWaterRainProfileId(
            *profiles,
            shared.name);
        profiles->push_back(std::move(shared));
        saved = &profiles->back();
    }
    saved->settings = savedSettings;
    saved->visual = savedVisual;
    saved->objectOverride = false;
    saved->ownerTimingTakeId.clear();
    saved->baseProfileId.clear();
    saved->baseProfileName.clear();
    const auto savedId = saved->id;

    // Repoint every observer before erasing the promoted owner copy, so no
    // stable assignment is left dangling.
    for (auto& definition : *takes) {
        if (definition.id == take->id ||
            (!promotedOwnerId.empty() &&
             (definition.assignedRainProfileId == promotedOwnerId ||
              (definition.assignedRainProfileId.empty() &&
               RainProfileNameKey(definition.assignedRainProfileName) ==
                   RainProfileNameKey(promotedOwnerName))))) {
            AssignTimingTakeRainBaseProfile(
                &definition,
                *profiles,
                savedId);
        }
    }
    if (!promotedOwnerId.empty()) {
        std::erase_if(
            *profiles,
            [&](const invisible_places::water::WaterRainProfile& profile) {
                return profile.id == promotedOwnerId;
            });
    }
    return invisible_places::water::FindWaterRainProfileById(
        profiles,
        savedId);
}

bool DiscardTimingTakeRainOwnerProfile(
    std::vector<invisible_places::water::WaterRainProfile>* profiles,
    std::vector<TimingTakeDefinition>* takes,
    std::string_view takeId) {
    if (profiles == nullptr || takes == nullptr) {
        return false;
    }
    auto* take = FindTimingTakeDefinition(takes, takeId);
    if (take == nullptr) {
        return false;
    }
    const auto* effective = ResolveTimingTakeRainProfile(*profiles, *take);
    if (effective == nullptr || !effective->objectOverride ||
        effective->ownerTimingTakeId != take->id) {
        return false;
    }
    const auto ownerId = effective->id;
    const auto ownerName = effective->name;
    const auto* base = FindExplicitRainBaseProfile(
        *profiles,
        effective->baseProfileId,
        effective->baseProfileName);
    if (base == nullptr) {
        return false;
    }
    const auto baseId = base->id;
    for (auto& definition : *takes) {
        if (definition.assignedRainProfileId == ownerId ||
            (definition.assignedRainProfileId.empty() &&
             RainProfileNameKey(definition.assignedRainProfileName) ==
                 RainProfileNameKey(ownerName))) {
            AssignTimingTakeRainBaseProfile(
                &definition,
                *profiles,
                baseId);
        }
    }
    std::erase_if(
        *profiles,
        [&](const invisible_places::water::WaterRainProfile& profile) {
            return profile.id == ownerId;
        });
    return true;
}

bool RenameTimingTakeRainOwnerProfile(
    std::vector<invisible_places::water::WaterRainProfile>* profiles,
    std::vector<TimingTakeDefinition>* takes,
    std::string_view takeId) {
    if (profiles == nullptr || takes == nullptr) {
        return false;
    }
    auto* take = FindTimingTakeDefinition(takes, takeId);
    if (take == nullptr) {
        return false;
    }
    bool renamed = false;
    for (auto& profile : *profiles) {
        if (!profile.objectOverride ||
            profile.ownerTimingTakeId != take->id) {
            continue;
        }
        const auto* base = FindExplicitRainBaseProfile(
            *profiles,
            profile.baseProfileId,
            profile.baseProfileName);
        if (base == nullptr) {
            continue;
        }
        const auto oldName = profile.name;
        const auto profileId = profile.id;
        profile.name = UniqueRainProfileName(
            *profiles,
            TimingTakeRainOwnerProfileName(*base, *take),
            profileId);
        for (auto& definition : *takes) {
            if (definition.assignedRainProfileId == profileId ||
                (definition.assignedRainProfileId.empty() &&
                 RainProfileNameKey(definition.assignedRainProfileName) ==
                     RainProfileNameKey(oldName))) {
                definition.assignedRainProfileId = profileId;
                definition.assignedRainProfileName = profile.name;
            }
        }
        renamed = true;
    }
    return renamed;
}

bool DuplicateTimingTakeRainProfileAssignment(
    std::vector<invisible_places::water::WaterRainProfile>* profiles,
    const TimingTakeDefinition& source,
    TimingTakeDefinition* duplicate) {
    if (profiles == nullptr || duplicate == nullptr) {
        return false;
    }
    const auto* sourceProfile = ResolveTimingTakeRainProfile(
        *profiles,
        source);
    const auto* sourceBase = ResolveTimingTakeRainBaseProfile(
        *profiles,
        source);
    if (sourceProfile == nullptr || sourceBase == nullptr) {
        return false;
    }
    const auto effective = *sourceProfile;
    const auto base = *sourceBase;
    if (!AssignTimingTakeRainBaseProfile(
            duplicate,
            *profiles,
            base.id)) {
        return false;
    }
    if (!effective.objectOverride) {
        return true;
    }
    return UpsertTimingTakeRainOwnerProfile(
               profiles,
               duplicate,
               effective.settings,
               effective.visual) != nullptr;
}

std::size_t RemoveTimingTakeRainOwnerProfiles(
    std::vector<invisible_places::water::WaterRainProfile>* profiles,
    std::vector<TimingTakeDefinition>* takes,
    std::string_view deletedTakeId) {
    if (profiles == nullptr) {
        return 0U;
    }
    const auto normalizedTakeId = NormalizeTimingTakeId(deletedTakeId);
    std::vector<invisible_places::water::WaterRainProfile> removed;
    for (const auto& profile : *profiles) {
        if (profile.objectOverride &&
            profile.ownerTimingTakeId == normalizedTakeId) {
            removed.push_back(profile);
        }
    }
    if (removed.empty()) {
        return 0U;
    }
    if (takes != nullptr) {
        for (auto& take : *takes) {
            if (take.id == normalizedTakeId) {
                continue;
            }
            const auto removedProfile = std::find_if(
                removed.begin(),
                removed.end(),
                [&](const invisible_places::water::WaterRainProfile& profile) {
                    return take.assignedRainProfileId == profile.id ||
                           (take.assignedRainProfileId.empty() &&
                            RainProfileNameKey(
                                take.assignedRainProfileName) ==
                                RainProfileNameKey(profile.name));
                });
            if (removedProfile == removed.end()) {
                continue;
            }
            const auto* base = FindExplicitRainBaseProfile(
                *profiles,
                removedProfile->baseProfileId,
                removedProfile->baseProfileName);
            if (base != nullptr) {
                SetTimingTakeRainAssignment(&take, *base, *base);
            } else {
                take.assignedRainProfileId.clear();
                take.assignedRainProfileName.clear();
                take.baseRainProfileId.clear();
                take.baseRainProfileName.clear();
            }
        }
    }
    std::erase_if(
        *profiles,
        [&](const invisible_places::water::WaterRainProfile& profile) {
            return profile.objectOverride &&
                   profile.ownerTimingTakeId == normalizedTakeId;
        });
    return removed.size();
}

std::array<float, 3> TimingColouriseColourToSpace(
    std::array<float, 3> colour,
    TimingColouriseColourSpace space) {
    for (auto& channel : colour) {
        channel = Clamp01(channel);
    }
    if (space == TimingColouriseColourSpace::Srgb) {
        return colour;
    }
    const std::array<float, 3> linear{
        SrgbChannelToLinear(colour[0]),
        SrgbChannelToLinear(colour[1]),
        SrgbChannelToLinear(colour[2]),
    };
    if (space == TimingColouriseColourSpace::LinearRgb) {
        return linear;
    }
    // OkLab (Bjorn Ottosson's reference matrices) from linear sRGB.
    const float l = std::cbrt(
        0.4122214708F * linear[0] + 0.5363325363F * linear[1] +
        0.0514459929F * linear[2]);
    const float m = std::cbrt(
        0.2119034982F * linear[0] + 0.6806995451F * linear[1] +
        0.1073969566F * linear[2]);
    const float s = std::cbrt(
        0.0883024619F * linear[0] + 0.2817188376F * linear[1] +
        0.6299787005F * linear[2]);
    return {
        0.2104542553F * l + 0.7936177850F * m - 0.0040720468F * s,
        1.9779984951F * l - 2.4285922050F * m + 0.4505937099F * s,
        0.0259040371F * l + 0.7827717662F * m - 0.8086757660F * s,
    };
}

std::array<float, 3> TimingColouriseColourFromSpace(
    std::array<float, 3> coordinates,
    TimingColouriseColourSpace space) {
    if (space == TimingColouriseColourSpace::Srgb) {
        for (auto& channel : coordinates) {
            channel = Clamp01(channel);
        }
        return coordinates;
    }
    std::array<float, 3> linear = coordinates;
    if (space == TimingColouriseColourSpace::OkLab) {
        const float l = coordinates[0] + 0.3963377774F * coordinates[1] +
                        0.2158037573F * coordinates[2];
        const float m = coordinates[0] - 0.1055613458F * coordinates[1] -
                        0.0638541728F * coordinates[2];
        const float s = coordinates[0] - 0.0894841775F * coordinates[1] -
                        1.2914855480F * coordinates[2];
        linear = {
            4.0767416621F * (l * l * l) - 3.3077115913F * (m * m * m) +
                0.2309699292F * (s * s * s),
            -1.2684380046F * (l * l * l) + 2.6097574011F * (m * m * m) -
                0.3413193965F * (s * s * s),
            -0.0041960863F * (l * l * l) - 0.7034186147F * (m * m * m) +
                1.7076147010F * (s * s * s),
        };
    }
    return {
        LinearChannelToSrgb(linear[0]),
        LinearChannelToSrgb(linear[1]),
        LinearChannelToSrgb(linear[2]),
    };
}

TimingColourisePalette SanitizeTimingColourisePalette(
    TimingColourisePalette palette) {
    if (palette.stops.size() > kMaximumTimingColourisePaletteStops) {
        palette.stops.resize(kMaximumTimingColourisePaletteStops);
    }
    for (auto& stop : palette.stops) {
        stop.position = Clamp01(stop.position);
        for (float& channel : stop.colour) {
            channel = Clamp01(channel);
        }
        stop.colouriseAmount = Clamp01(stop.colouriseAmount);
    }
    std::stable_sort(
        palette.stops.begin(),
        palette.stops.end(),
        [](const TimingColourisePaletteStop& left,
           const TimingColourisePaletteStop& right) {
            return left.position < right.position;
        });
    if (palette.stops.empty()) {
        palette.stops.push_back(TimingColourisePaletteStop{});
    }
    // Reserve every authored id before filling gaps. This keeps a missing id
    // that sorts before "palette-stop-1" from stealing that existing id.
    std::unordered_set<std::string> reservedIds;
    for (const auto& stop : palette.stops) {
        if (!stop.id.empty()) {
            reservedIds.insert(stop.id);
        }
    }
    std::unordered_set<std::string> usedIds;
    std::uint32_t sequence = 1U;
    for (auto& stop : palette.stops) {
        if (!stop.id.empty() && usedIds.insert(stop.id).second) {
            continue;
        }
        std::string candidate;
        do {
            candidate = "palette-stop-" +
                        std::to_string(sequence++);
        } while (reservedIds.contains(candidate) ||
                 usedIds.contains(candidate));
        stop.id = std::move(candidate);
        reservedIds.insert(stop.id);
        usedIds.insert(stop.id);
    }
    return palette;
}

TimingColourisePalette ReverseTimingColourisePalette(
    TimingColourisePalette palette) {
    palette = SanitizeTimingColourisePalette(std::move(palette));
    // Reversing equal-position stops as a group would preserve the direction
    // of their discontinuity. Reverse authored order first so sampling the
    // result at 1-x is the mathematical mirror of sampling the input at x.
    std::reverse(palette.stops.begin(), palette.stops.end());
    for (auto& stop : palette.stops) {
        stop.position = 1.0F - stop.position;
    }
    return SanitizeTimingColourisePalette(std::move(palette));
}

bool CanReverseTimingColourisePaletteAtPosition(
    const TimingColouriseEffect& effect,
    float position) {
    if (!std::isfinite(position) || position < 0.0F ||
        position > 1.0F) {
        return false;
    }
    const auto sanitized = SanitizeTimingColouriseEffect(effect);
    if (sanitized.paletteKeyModel ==
        TimingColourisePaletteKeyModel::StopParameters) {
        return true;
    }
    return sanitized.paletteKeys.empty() ||
           std::any_of(
               sanitized.paletteKeys.begin(),
               sanitized.paletteKeys.end(),
               [&](const TimingColourisePaletteKey& key) {
                   return std::abs(key.position - position) <=
                          kTimingColouriseKeyTolerance;
               });
}

bool ReverseTimingColourisePaletteAtPosition(
    TimingColouriseEffect* effect,
    float position) {
    if (effect == nullptr ||
        !CanReverseTimingColourisePaletteAtPosition(*effect, position)) {
        return false;
    }

    auto updated = SanitizeTimingColouriseEffect(*effect);
    if (updated.paletteKeyModel ==
        TimingColourisePaletteKeyModel::LegacySnapshots) {
        if (updated.paletteKeys.empty()) {
            updated.basePalette = ReverseTimingColourisePalette(
                std::move(updated.basePalette));
            if (updated.paletteSourceKind !=
                TimingColourisePaletteSourceKind::Preset) {
                updated.paletteEdited = true;
            }
        } else {
            const auto exact = std::find_if(
                updated.paletteKeys.begin(),
                updated.paletteKeys.end(),
                [&](const TimingColourisePaletteKey& key) {
                    return std::abs(key.position - position) <=
                           kTimingColouriseKeyTolerance;
                });
            if (exact == updated.paletteKeys.end()) {
                return false;
            }
            exact->palette = ReverseTimingColourisePalette(
                std::move(exact->palette));
        }
        *effect = SanitizeTimingColouriseEffect(std::move(updated));
        return true;
    }

    const bool hasPaletteKeys = !updated.paletteKeys.empty() ||
                                !updated.paletteStopParameterKeys.empty();
    if (!hasPaletteKeys) {
        updated.basePalette = ReverseTimingColourisePalette(
            std::move(updated.basePalette));
        if (updated.paletteSourceKind !=
            TimingColourisePaletteSourceKind::Preset) {
            updated.paletteEdited = true;
        }
        *effect = SanitizeTimingColouriseEffect(std::move(updated));
        return true;
    }

    const auto reversed = ReverseTimingColourisePalette(
        EvaluateTimingColourisePalette(updated, position));
    for (const auto& stop : reversed.stops) {
        const auto existing = std::find_if(
            updated.paletteStopParameterKeys.begin(),
            updated.paletteStopParameterKeys.end(),
            [&](const TimingColourisePaletteStopParameterKey& key) {
                return key.stopId == stop.id &&
                       key.parameter ==
                           TimingColourisePaletteStopParameter::Position &&
                       std::abs(key.position - position) <=
                           kTimingColouriseKeyTolerance;
            });
        const auto interpolation =
            existing != updated.paletteStopParameterKeys.end()
                ? existing->interpolation
                : invisible_places::water::
                      WaterScenarioInterpolation::SmoothVelocity;
        if (!AddOrUpdateTimingColourisePaletteStopScalarKey(
                &updated,
                stop.id,
                TimingColourisePaletteStopParameter::Position,
                position,
                stop.position,
                interpolation)) {
            return false;
        }
    }
    *effect = SanitizeTimingColouriseEffect(std::move(updated));
    return true;
}

std::string AllocateTimingColourisePaletteStopId(
    const TimingColourisePalette& palette) {
    std::unordered_set<std::string> ids;
    ids.reserve(palette.stops.size());
    for (const auto& stop : palette.stops) {
        if (!stop.id.empty()) {
            ids.insert(stop.id);
        }
    }
    for (std::uint32_t sequence = 1U;; ++sequence) {
        const std::string candidate =
            "palette-stop-" + std::to_string(sequence);
        if (!ids.contains(candidate)) {
            return candidate;
        }
    }
}

TimingColouriseBounds SanitizeTimingColouriseBounds(
    TimingColouriseBounds bounds) {
    bounds.lower = FiniteOr(bounds.lower, 0.0F);
    bounds.upper = FiniteOr(bounds.upper, 1.0F);
    if (bounds.lower > bounds.upper) {
        std::swap(bounds.lower, bounds.upper);
    }
    bounds.edgeFadeLower =
        std::clamp(FiniteOr(bounds.edgeFadeLower, 0.10F), -1.0F, 1.0F);
    bounds.edgeFadeUpper =
        std::clamp(FiniteOr(bounds.edgeFadeUpper, 0.10F), -1.0F, 1.0F);
    return bounds;
}

bool TimingColouriseBoundsParameterIsAllowed(
    TimingColouriseBoundsKeyMode mode,
    TimingColouriseBoundsParameter parameter) {
    if (!IsValidBoundsKeyMode(mode) ||
        !IsValidBoundsParameter(parameter)) {
        return false;
    }
    if (parameter == TimingColouriseBoundsParameter::EdgeFadeLower ||
        parameter == TimingColouriseBoundsParameter::EdgeFadeUpper) {
        return true;
    }
    if (parameter == TimingColouriseBoundsParameter::EdgeFade) {
        // The legacy shared lane only exists between parsing and sanitize,
        // which splits it; no live track may keep it.
        return false;
    }
    const auto pair = TimingColouriseBoundsParametersForMode(mode);
    return parameter == pair[0] || parameter == pair[1];
}

std::array<TimingColouriseBoundsParameter, 2>
TimingColouriseBoundsParametersForMode(
    TimingColouriseBoundsKeyMode mode) {
    switch (mode) {
        case TimingColouriseBoundsKeyMode::CentreSpread:
            return {
                TimingColouriseBoundsParameter::Centre,
                TimingColouriseBoundsParameter::Spread};
        case TimingColouriseBoundsKeyMode::LowerSpread:
            return {
                TimingColouriseBoundsParameter::Lower,
                TimingColouriseBoundsParameter::Spread};
        case TimingColouriseBoundsKeyMode::UpperSpread:
            return {
                TimingColouriseBoundsParameter::Upper,
                TimingColouriseBoundsParameter::Spread};
        case TimingColouriseBoundsKeyMode::LowerUpper:
        default:
            return {
                TimingColouriseBoundsParameter::Lower,
                TimingColouriseBoundsParameter::Upper};
    }
}

float TimingColouriseBoundsParameterValue(
    const TimingColouriseBounds& bounds,
    TimingColouriseBoundsParameter parameter) {
    const auto sanitized = SanitizeTimingColouriseBounds(bounds);
    switch (parameter) {
        case TimingColouriseBoundsParameter::Lower:
            return sanitized.lower;
        case TimingColouriseBoundsParameter::Upper:
            return sanitized.upper;
        case TimingColouriseBoundsParameter::Centre:
            return std::midpoint(sanitized.lower, sanitized.upper);
        case TimingColouriseBoundsParameter::Spread:
            return sanitized.upper - sanitized.lower;
        case TimingColouriseBoundsParameter::EdgeFade:
            // Legacy readback: the closest single value is the mean.
            return std::midpoint(
                sanitized.edgeFadeLower,
                sanitized.edgeFadeUpper);
        case TimingColouriseBoundsParameter::EdgeFadeLower:
            return sanitized.edgeFadeLower;
        case TimingColouriseBoundsParameter::EdgeFadeUpper:
            return sanitized.edgeFadeUpper;
    }
    return 0.0F;
}

float RemapTimingColouriseBoundsParameterValueToRange(
    TimingColouriseBoundsParameter parameter,
    float value,
    float sourceMinimum,
    float sourceMaximum,
    float destinationMinimum,
    float destinationMaximum) {
    value = SanitizeBoundsParameterValue(parameter, value);
    if (parameter == TimingColouriseBoundsParameter::EdgeFade ||
        parameter == TimingColouriseBoundsParameter::EdgeFadeLower ||
        parameter == TimingColouriseBoundsParameter::EdgeFadeUpper) {
        // Fades are dimensionless fractions of the span.
        return value;
    }

    if (!std::isfinite(sourceMinimum) ||
        !std::isfinite(sourceMaximum) ||
        !std::isfinite(destinationMinimum) ||
        !std::isfinite(destinationMaximum)) {
        return value;
    }
    const auto [sourceLow, sourceHigh] = std::minmax(
        sourceMinimum,
        sourceMaximum);
    const auto [destinationLow, destinationHigh] = std::minmax(
        destinationMinimum,
        destinationMaximum);
    const float sourceWidth = sourceHigh - sourceLow;
    const float destinationWidth = destinationHigh - destinationLow;
    if (destinationWidth <= std::numeric_limits<float>::epsilon()) {
        return parameter == TimingColouriseBoundsParameter::Spread
                   ? 0.0F
                   : destinationLow;
    }
    if (sourceWidth <= std::numeric_limits<float>::epsilon()) {
        return parameter == TimingColouriseBoundsParameter::Spread
                   ? value
                   : destinationLow;
    }
    if (parameter == TimingColouriseBoundsParameter::Spread) {
        return std::max(
            0.0F,
            value * destinationWidth / sourceWidth);
    }
    const float percentile = std::clamp(
        (value - sourceLow) / sourceWidth,
        0.0F,
        1.0F);
    return std::lerp(
        destinationLow,
        destinationHigh,
        percentile);
}

std::vector<float> OffsetTimingColouriseKeyPositionsForPaste(
    std::span<const float> sourcePositions,
    float destinationAnchor) {
    if (sourcePositions.empty()) {
        return {};
    }
    std::vector<float> positions;
    positions.reserve(sourcePositions.size());
    float sourceMinimum = 1.0F;
    float sourceMaximum = 0.0F;
    for (const float sourcePosition : sourcePositions) {
        const float position = Clamp01(FiniteOr(sourcePosition, 0.0F));
        positions.push_back(position);
        sourceMinimum = std::min(sourceMinimum, position);
        sourceMaximum = std::max(sourceMaximum, position);
    }

    float offset =
        Clamp01(FiniteOr(destinationAnchor, 0.0F)) - sourceMinimum;
    if (sourceMaximum + offset > 1.0F) {
        offset -= sourceMaximum + offset - 1.0F;
    }
    if (sourceMinimum + offset < 0.0F) {
        offset -= sourceMinimum + offset;
    }
    for (float& position : positions) {
        position = Clamp01(position + offset);
    }
    return positions;
}

std::optional<TimingColouriseBoundsHandleEdit>
ResolveTimingColouriseBoundsHandleEdit(
    TimingColouriseBoundsKeyMode mode,
    TimingColouriseBoundsHandle handle,
    TimingColouriseBounds currentBounds,
    float targetValue,
    float rangeMinimum,
    float rangeMaximum) {
    if (!IsValidBoundsKeyMode(mode) ||
        !std::isfinite(targetValue) ||
        !std::isfinite(rangeMinimum) ||
        !std::isfinite(rangeMaximum) ||
        rangeMaximum <= rangeMinimum) {
        return std::nullopt;
    }
    switch (handle) {
        case TimingColouriseBoundsHandle::Lower:
        case TimingColouriseBoundsHandle::Upper:
        case TimingColouriseBoundsHandle::Centre:
            break;
        default:
            return std::nullopt;
    }

    currentBounds =
        SanitizeTimingColouriseBounds(currentBounds);
    const float spread =
        currentBounds.upper - currentBounds.lower;
    const float halfSpread = spread * 0.5F;
    const float rangeSpread =
        rangeMaximum - rangeMinimum;
    const auto translatedToCentre =
        [&](float requestedCentre) {
            float centre = requestedCentre;
            if (spread >= rangeSpread) {
                centre =
                    std::midpoint(rangeMinimum, rangeMaximum);
            } else {
                centre = std::clamp(
                    centre,
                    rangeMinimum + halfSpread,
                    rangeMaximum - halfSpread);
            }
            TimingColouriseBounds translated =
                currentBounds;
            translated.lower = centre - halfSpread;
            translated.upper = centre + halfSpread;
            return translated;
        };
    const auto oneParameter =
        [](TimingColouriseBounds bounds,
           TimingColouriseBoundsParameter parameter) {
            TimingColouriseBoundsHandleEdit edit;
            edit.bounds = bounds;
            edit.parameters[0] = parameter;
            edit.parameterCount = 1U;
            return edit;
        };

    switch (mode) {
        case TimingColouriseBoundsKeyMode::CentreSpread: {
            if (handle ==
                TimingColouriseBoundsHandle::Centre) {
                return oneParameter(
                    translatedToCentre(targetValue),
                    TimingColouriseBoundsParameter::Centre);
            }
            const float centre = std::midpoint(
                currentBounds.lower,
                currentBounds.upper);
            if (centre < rangeMinimum ||
                centre > rangeMaximum) {
                return std::nullopt;
            }
            const float maximumHalfSpread = std::max(
                0.0F,
                std::min(
                    centre - rangeMinimum,
                    rangeMaximum - centre));
            const float requestedHalfSpread =
                handle == TimingColouriseBoundsHandle::Lower
                    ? centre - targetValue
                    : targetValue - centre;
            const float resolvedHalfSpread = std::clamp(
                requestedHalfSpread,
                0.0F,
                maximumHalfSpread);
            auto bounds = currentBounds;
            bounds.lower = centre - resolvedHalfSpread;
            bounds.upper = centre + resolvedHalfSpread;
            return oneParameter(
                bounds,
                TimingColouriseBoundsParameter::Spread);
        }
        case TimingColouriseBoundsKeyMode::LowerSpread: {
            if (handle ==
                TimingColouriseBoundsHandle::Upper) {
                if (currentBounds.lower > rangeMaximum) {
                    return std::nullopt;
                }
                auto bounds = currentBounds;
                bounds.upper = std::clamp(
                    targetValue,
                    std::max(
                        currentBounds.lower,
                        rangeMinimum),
                    rangeMaximum);
                return oneParameter(
                    bounds,
                    TimingColouriseBoundsParameter::Spread);
            }
            const float requestedCentre =
                handle == TimingColouriseBoundsHandle::Centre
                    ? targetValue
                    : targetValue + halfSpread;
            return oneParameter(
                translatedToCentre(requestedCentre),
                TimingColouriseBoundsParameter::Lower);
        }
        case TimingColouriseBoundsKeyMode::UpperSpread: {
            if (handle ==
                TimingColouriseBoundsHandle::Lower) {
                if (currentBounds.upper < rangeMinimum) {
                    return std::nullopt;
                }
                auto bounds = currentBounds;
                bounds.lower = std::clamp(
                    targetValue,
                    rangeMinimum,
                    std::min(
                        currentBounds.upper,
                        rangeMaximum));
                return oneParameter(
                    bounds,
                    TimingColouriseBoundsParameter::Spread);
            }
            const float requestedCentre =
                handle == TimingColouriseBoundsHandle::Centre
                    ? targetValue
                    : targetValue - halfSpread;
            return oneParameter(
                translatedToCentre(requestedCentre),
                TimingColouriseBoundsParameter::Upper);
        }
        case TimingColouriseBoundsKeyMode::LowerUpper:
        default:
            if (handle ==
                TimingColouriseBoundsHandle::Lower) {
                if (currentBounds.upper < rangeMinimum) {
                    return std::nullopt;
                }
                auto bounds = currentBounds;
                bounds.lower = std::clamp(
                    targetValue,
                    rangeMinimum,
                    std::min(
                        currentBounds.upper,
                        rangeMaximum));
                return oneParameter(
                    bounds,
                    TimingColouriseBoundsParameter::Lower);
            }
            if (handle ==
                TimingColouriseBoundsHandle::Upper) {
                if (currentBounds.lower > rangeMaximum) {
                    return std::nullopt;
                }
                auto bounds = currentBounds;
                bounds.upper = std::clamp(
                    targetValue,
                    std::max(
                        currentBounds.lower,
                        rangeMinimum),
                    rangeMaximum);
                return oneParameter(
                    bounds,
                    TimingColouriseBoundsParameter::Upper);
            }
            TimingColouriseBoundsHandleEdit edit;
            edit.bounds = translatedToCentre(targetValue);
            // Move the leading endpoint first so the existing independent
            // endpoint clamp cannot truncate a translation.
            if (edit.bounds.lower > currentBounds.lower) {
                edit.parameters = {
                    TimingColouriseBoundsParameter::Upper,
                    TimingColouriseBoundsParameter::Lower};
            } else {
                edit.parameters = {
                    TimingColouriseBoundsParameter::Lower,
                    TimingColouriseBoundsParameter::Upper};
            }
            edit.parameterCount = 2U;
            return edit;
    }
}

bool SetTimingColouriseBoundsKeyMode(
    TimingColouriseEffect* effect,
    TimingColouriseBoundsKeyMode mode) {
    if (effect == nullptr || !IsValidBoundsKeyMode(mode)) {
        return false;
    }
    const bool hasConflictingTrack = std::any_of(
        effect->boundsParameterKeys.begin(),
        effect->boundsParameterKeys.end(),
        [&](const TimingColouriseBoundsParameterKey& key) {
            return !TimingColouriseBoundsParameterIsAllowed(
                mode,
                key.parameter);
        });
    if (hasConflictingTrack) {
        return false;
    }
    effect->boundsKeyMode = mode;
    return true;
}

bool TimingEffectParameterIsSupported(
    bool colouriseEnabled,
    bool emissiveEnabled,
    TimingColouriseEffectParameter parameter) {
    if (!IsValidEffectParameter(parameter)) {
        return false;
    }
    switch (parameter) {
        case TimingColouriseEffectParameter::PalettePhase:
        case TimingColouriseEffectParameter::AmountOverride:
        case TimingColouriseEffectParameter::PaletteSkewCentre:
        case TimingColouriseEffectParameter::PaletteSkewSpread:
            return colouriseEnabled;
        case TimingColouriseEffectParameter::EmissiveLevel:
        case TimingColouriseEffectParameter::EmissiveSkewCentre:
        case TimingColouriseEffectParameter::EmissiveSkewSpread:
            return emissiveEnabled;
        // Legacy per-side skews exist only between parsing and sanitize,
        // which retags their keys onto Spread.
        case TimingColouriseEffectParameter::PaletteSkewLower:
        case TimingColouriseEffectParameter::PaletteSkewUpper:
            return false;
    }
    return false;
}

bool TimingEffectParameterIsSupported(
    const TimingColouriseEffect& effect,
    TimingColouriseEffectParameter parameter) {
    return TimingEffectParameterIsSupported(
        effect.colouriseEnabled,
        effect.emissiveEnabled,
        parameter);
}

TimingColouriseActivationRange
SanitizeTimingColouriseActivationRange(
    TimingColouriseActivationRange range) {
    range.start = std::clamp(
        FiniteOr(range.start, 0.0F),
        0.0F,
        1.0F);
    range.end = std::clamp(
        FiniteOr(range.end, 1.0F),
        0.0F,
        1.0F);
    if (range.start > range.end) {
        std::swap(range.start, range.end);
    }
    return range;
}

bool TimingColouriseActivationRangeContains(
    TimingColouriseActivationRange range,
    float normalizedPosition) {
    if (!std::isfinite(normalizedPosition)) {
        return false;
    }
    range = SanitizeTimingColouriseActivationRange(range);
    normalizedPosition = std::clamp(normalizedPosition, 0.0F, 1.0F);
    return normalizedPosition >= range.start &&
           normalizedPosition <= range.end;
}

bool TimingColouriseEffectIsActiveAt(
    const TimingColouriseEffect& effect,
    float normalizedPosition,
    bool cyclic) {
    if (cyclic && std::isfinite(normalizedPosition)) {
        normalizedPosition -= std::floor(normalizedPosition);
    }
    return effect.enabled &&
           (effect.colouriseEnabled || effect.emissiveEnabled) &&
           TimingColouriseActivationRangeContains(
               effect.activationRange,
               normalizedPosition);
}

TimingColouriseEffect SanitizeTimingColouriseEffect(
    TimingColouriseEffect effect) {
    if (!effect.colouriseEnabled && !effect.emissiveEnabled) {
        effect.colouriseEnabled = true;
    }
    if (effect.name.empty()) {
        effect.name = "Visual Feature";
    }
    effect.activationRange = SanitizeTimingColouriseActivationRange(
        effect.activationRange);
    if (effect.emissiveEnabled &&
        effect.field.source != TimingColouriseFieldSource::Scalar) {
        if (effect.colouriseEnabled) {
            // Colourise legitimately reads normal sources; the emissive
            // aspect simply cannot follow it there.
            effect.emissiveEnabled = false;
        } else {
            // A pure emissive feature keeps the legacy behaviour of being
            // forced back onto a scalar source.
            effect.field.source = TimingColouriseFieldSource::Scalar;
            effect.field.scalarFieldName.clear();
        }
    }
    if (effect.field.source != TimingColouriseFieldSource::Scalar) {
        effect.field.scalarFieldName.clear();
    }
    effect.basePalette =
        SanitizeTimingColourisePalette(std::move(effect.basePalette));
    effect.baseBounds = SanitizeTimingColouriseBounds(effect.baseBounds);
    if (!IsValidPaletteKeyModel(effect.paletteKeyModel)) {
        effect.paletteKeyModel = effect.paletteKeys.empty()
                                     ? TimingColourisePaletteKeyModel::
                                           StopParameters
                                     : TimingColourisePaletteKeyModel::
                                           LegacySnapshots;
    }
    if (!IsValidPaletteSourceKind(effect.paletteSourceKind)) {
        effect.paletteSourceKind =
            TimingColourisePaletteSourceKind::Custom;
    }
    if (effect.paletteSourceKind ==
        TimingColourisePaletteSourceKind::Custom) {
        effect.paletteSourceId.clear();
    }
    std::vector<TimingColouriseLocalPaletteEdit> localPaletteEdits;
    localPaletteEdits.reserve(effect.localPaletteEdits.size() + 1U);
    for (auto& edit : effect.localPaletteEdits) {
        // Only library-backed variants are meaningful; a Custom source has
        // nothing to shadow.
        if (edit.presetId.empty() ||
            (edit.sourceKind !=
                 TimingColourisePaletteSourceKind::Preset &&
             edit.sourceKind !=
                 TimingColourisePaletteSourceKind::Saved)) {
            continue;
        }
        if (edit.presetName.empty()) {
            edit.presetName = edit.presetId;
        }
        edit.palette =
            SanitizeTimingColourisePalette(std::move(edit.palette));
        const auto existing = std::find_if(
            localPaletteEdits.begin(),
            localPaletteEdits.end(),
            [&](const TimingColouriseLocalPaletteEdit& candidate) {
                return candidate.sourceKind == edit.sourceKind &&
                       candidate.presetId == edit.presetId;
            });
        if (existing == localPaletteEdits.end()) {
            localPaletteEdits.push_back(std::move(edit));
        } else {
            *existing = std::move(edit);
        }
    }
    effect.localPaletteEdits = std::move(localPaletteEdits);
    // Projects authored before effect-local variants stored an active
    // Preset_edited (and, before saved-source variants, a Saved_edited)
    // palette only in basePalette. A non-empty source id makes that
    // provenance safe to synthesize, and also keeps direct active edits and
    // Flip Palette synchronized with their private snapshot.
    if ((effect.paletteSourceKind ==
             TimingColourisePaletteSourceKind::Preset ||
         effect.paletteSourceKind ==
             TimingColourisePaletteSourceKind::Saved) &&
        effect.paletteEdited && !effect.paletteSourceId.empty()) {
        if (effect.paletteSourceName.empty()) {
            effect.paletteSourceName = effect.paletteSourceId;
        }
        auto activeEdit = std::find_if(
            effect.localPaletteEdits.begin(),
            effect.localPaletteEdits.end(),
            [&](const TimingColouriseLocalPaletteEdit& candidate) {
                return candidate.sourceKind ==
                           effect.paletteSourceKind &&
                       candidate.presetId == effect.paletteSourceId;
            });
        if (activeEdit == effect.localPaletteEdits.end()) {
            effect.localPaletteEdits.push_back(
                TimingColouriseLocalPaletteEdit{
                    .sourceKind = effect.paletteSourceKind,
                    .presetId = effect.paletteSourceId,
                    .presetName = effect.paletteSourceName,
                    .palette = effect.basePalette,
                });
        } else {
            if (effect.paletteSourceName.empty()) {
                effect.paletteSourceName = activeEdit->presetName;
            }
            activeEdit->presetName =
                effect.paletteSourceName;
            activeEdit->palette = effect.basePalette;
        }
    }
    if (!IsValidAmountOverrideMode(
            effect.colouriseAmountOverrideMode)) {
        effect.colouriseAmountOverrideMode =
            TimingColouriseAmountOverrideMode::Maximum;
    }
    if (!IsValidColourSpace(effect.colourKeyInterpolationSpace)) {
        effect.colourKeyInterpolationSpace =
            TimingColouriseColourSpace::Srgb;
    }
    effect.colouriseAmountOverride = std::clamp(
        FiniteOr(effect.colouriseAmountOverride, 1.0F),
        0.0F,
        1.0F);
    effect.palettePhaseOffset =
        FiniteOr(effect.palettePhaseOffset, 0.0F);
    effect.paletteSkewCentre =
        std::clamp(FiniteOr(effect.paletteSkewCentre, 0.5F), 0.0F, 1.0F);
    effect.paletteSkewSpread = ClampPaletteSkew(effect.paletteSkewSpread);
    effect.emissiveSkewCentre =
        std::clamp(FiniteOr(effect.emissiveSkewCentre, 0.5F), 0.0F, 1.0F);
    effect.emissiveSkewSpread =
        ClampPaletteSkew(effect.emissiveSkewSpread);
    const auto sanitizeSkewNodes =
        [](std::vector<TimingColourisePaletteSkewNode>* nodes) {
            std::vector<TimingColourisePaletteSkewNode> kept;
            kept.reserve(nodes->size());
            for (auto& node : *nodes) {
                if (node.id.empty() ||
                    std::any_of(
                        kept.begin(),
                        kept.end(),
                        [&](const TimingColourisePaletteSkewNode&
                                existing) {
                            return existing.id == node.id;
                        })) {
                    continue;
                }
                node.palettePosition = std::clamp(
                    FiniteOr(node.palettePosition, 0.5F),
                    0.02F,
                    0.98F);
                node.fieldPosition = Clamp01(node.fieldPosition);
                node.spread = ClampPaletteSkew(node.spread);
                kept.push_back(std::move(node));
            }
            std::stable_sort(
                kept.begin(),
                kept.end(),
                [](const TimingColourisePaletteSkewNode& left,
                   const TimingColourisePaletteSkewNode& right) {
                    return left.palettePosition < right.palettePosition;
                });
            *nodes = std::move(kept);
        };
    sanitizeSkewNodes(&effect.paletteSkewNodes);
    sanitizeSkewNodes(&effect.emissiveSkewNodes);
    // Legacy per-side skew keys become centre-spread keys: the node warp
    // has no per-side lanes, and the retag preserves each key's timing and
    // approximate intent.
    for (auto& key : effect.effectParameterKeys) {
        if (key.parameter ==
                TimingColouriseEffectParameter::PaletteSkewLower ||
            key.parameter ==
                TimingColouriseEffectParameter::PaletteSkewUpper) {
            key.parameter =
                TimingColouriseEffectParameter::PaletteSkewSpread;
        }
    }
    const auto sanitizeFalloff =
        [](std::vector<TimingColouriseEmissiveFalloffNode>* nodes,
           std::vector<TimingColouriseEmissiveFalloffKey>* keys) {
            std::vector<TimingColouriseEmissiveFalloffNode> keptNodes;
            keptNodes.reserve(nodes->size());
            for (auto& node : *nodes) {
                if (node.id.empty() ||
                    std::any_of(
                        keptNodes.begin(),
                        keptNodes.end(),
                        [&](const TimingColouriseEmissiveFalloffNode&
                                existing) {
                            return existing.id == node.id;
                        })) {
                    continue;
                }
                node.position = Clamp01(node.position);
                node.level = Clamp01(node.level);
                keptNodes.push_back(std::move(node));
            }
            std::stable_sort(
                keptNodes.begin(),
                keptNodes.end(),
                [](const TimingColouriseEmissiveFalloffNode& left,
                   const TimingColouriseEmissiveFalloffNode& right) {
                    return left.position < right.position;
                });
            *nodes = std::move(keptNodes);
            std::erase_if(
                *keys,
                [&](const TimingColouriseEmissiveFalloffKey& key) {
                    return key.nodeId.empty() ||
                           (key.parameter !=
                                TimingColouriseEmissiveFalloffParameter::
                                    Position &&
                            key.parameter !=
                                TimingColouriseEmissiveFalloffParameter::
                                    Level) ||
                           std::none_of(
                               nodes->begin(),
                               nodes->end(),
                               [&](const auto& node) {
                                   return node.id == key.nodeId;
                               });
                });
            for (auto& key : *keys) {
                key.position = Clamp01(key.position);
                key.value = Clamp01(key.value);
                if (!IsValidInterpolation(key.interpolation)) {
                    key.interpolation = invisible_places::water::
                        WaterScenarioInterpolation::Smooth;
                }
            }
            std::stable_sort(
                keys->begin(),
                keys->end(),
                [](const TimingColouriseEmissiveFalloffKey& left,
                   const TimingColouriseEmissiveFalloffKey& right) {
                    if (left.nodeId != right.nodeId) {
                        return left.nodeId < right.nodeId;
                    }
                    if (left.parameter != right.parameter) {
                        return static_cast<std::uint8_t>(
                                   left.parameter) <
                               static_cast<std::uint8_t>(
                                   right.parameter);
                    }
                    return left.position < right.position;
                });
            std::vector<TimingColouriseEmissiveFalloffKey> unique;
            unique.reserve(keys->size());
            for (auto& key : *keys) {
                if (!unique.empty() &&
                    unique.back().nodeId == key.nodeId &&
                    unique.back().parameter == key.parameter &&
                    std::abs(unique.back().position - key.position) <=
                        kTimingColouriseKeyTolerance) {
                    unique.back() = std::move(key);
                } else {
                    unique.push_back(std::move(key));
                }
            }
            *keys = std::move(unique);
        };
    sanitizeFalloff(
        &effect.emissiveFalloffNodes,
        &effect.emissiveFalloffKeys);
    effect.emissiveLevel =
        FiniteOr(effect.emissiveLevel, 1.0F);
    std::erase_if(
        effect.effectParameterKeys,
        [](const TimingColouriseEffectParameterKey& key) {
            return !IsValidEffectParameter(key.parameter);
        });
    for (auto& key : effect.effectParameterKeys) {
        key.position = Clamp01(key.position);
        key.value = SanitizeEffectParameterValue(
            key.parameter,
            key.value);
        if (!IsValidInterpolation(key.interpolation)) {
            key.interpolation = invisible_places::water::
                WaterScenarioInterpolation::Smooth;
        }
    }
    for (auto& key : effect.paletteKeys) {
        key.position = Clamp01(key.position);
        key.palette =
            SanitizeTimingColourisePalette(std::move(key.palette));
        if (!IsValidInterpolation(key.interpolation)) {
            key.interpolation = invisible_places::water::
                WaterScenarioInterpolation::Smooth;
        }
    }
    std::unordered_set<std::string> stopIds;
    std::size_t stopIdCapacity = effect.basePalette.stops.size();
    for (const auto& edit : effect.localPaletteEdits) {
        stopIdCapacity += edit.palette.stops.size();
    }
    stopIds.reserve(stopIdCapacity);
    for (const auto& stop : effect.basePalette.stops) {
        stopIds.insert(stop.id);
    }
    for (const auto& edit : effect.localPaletteEdits) {
        for (const auto& stop : edit.palette.stops) {
            stopIds.insert(stop.id);
        }
    }
    std::erase_if(
        effect.paletteStopParameterKeys,
        [&](const TimingColourisePaletteStopParameterKey& key) {
            return key.stopId.empty() || !stopIds.contains(key.stopId) ||
                   !IsValidPaletteStopParameter(key.parameter);
        });
    for (auto& key : effect.paletteStopParameterKeys) {
        key.position = Clamp01(key.position);
        key.scalarValue = SanitizePaletteStopScalarValue(
            key.parameter,
            key.scalarValue);
        key.colourValue =
            SanitizePaletteStopColour(key.colourValue);
        if (!IsValidInterpolation(key.interpolation)) {
            key.interpolation = invisible_places::water::
                WaterScenarioInterpolation::Smooth;
        }
    }
    for (auto& key : effect.boundsKeys) {
        key.position = Clamp01(key.position);
        key.bounds = SanitizeTimingColouriseBounds(key.bounds);
        if (!IsValidInterpolation(key.interpolation)) {
            key.interpolation = invisible_places::water::
                WaterScenarioInterpolation::Smooth;
        }
    }
    if (!IsValidBoundsKeyMode(effect.boundsKeyMode)) {
        effect.boundsKeyMode =
            TimingColouriseBoundsKeyMode::LowerUpper;
    }
    SplitLegacyEdgeFadeKeys(&effect.boundsParameterKeys);
    std::erase_if(
        effect.boundsParameterKeys,
        [&](const TimingColouriseBoundsParameterKey& key) {
            return !IsValidBoundsParameter(key.parameter) ||
                   !TimingColouriseBoundsParameterIsAllowed(
                       effect.boundsKeyMode,
                       key.parameter);
        });
    for (auto& key : effect.boundsParameterKeys) {
        key.position = Clamp01(key.position);
        key.value =
            SanitizeBoundsParameterValue(key.parameter, key.value);
        if (!IsValidInterpolation(key.interpolation)) {
            key.interpolation = invisible_places::water::
                WaterScenarioInterpolation::Smooth;
        }
    }
    SortAndCoalesceKeys(&effect.paletteKeys);
    SortAndCoalescePaletteStopParameterKeys(
        &effect.paletteStopParameterKeys);
    SortAndCoalesceEffectParameterKeys(
        &effect.effectParameterKeys);
    SortAndCoalesceKeys(&effect.boundsKeys);
    SortAndCoalesceBoundsParameterKeys(&effect.boundsParameterKeys);
    std::vector<TimingColouriseFieldBoundsMemory> fieldBoundsMemory;
    fieldBoundsMemory.reserve(effect.fieldBoundsMemory.size());
    for (auto& memory : effect.fieldBoundsMemory) {
        if (memory.selector.source ==
                TimingColouriseFieldSource::Scalar &&
            memory.selector.scalarFieldName.empty()) {
            continue;
        }
        const bool duplicate = std::any_of(
            fieldBoundsMemory.begin(),
            fieldBoundsMemory.end(),
            [&](const TimingColouriseFieldBoundsMemory& kept) {
                return kept.selector == memory.selector;
            });
        if (duplicate) {
            continue;
        }
        memory.bounds = SanitizeTimingColouriseBounds(memory.bounds);
        if (!IsValidBoundsKeyMode(memory.boundsKeyMode)) {
            memory.boundsKeyMode =
                TimingColouriseBoundsKeyMode::LowerUpper;
        }
        SplitLegacyEdgeFadeKeys(&memory.boundsParameterKeys);
        std::erase_if(
            memory.boundsParameterKeys,
            [&](const TimingColouriseBoundsParameterKey& key) {
                return !IsValidBoundsParameter(key.parameter) ||
                       !TimingColouriseBoundsParameterIsAllowed(
                           memory.boundsKeyMode,
                           key.parameter);
            });
        for (auto& key : memory.boundsKeys) {
            key.position = Clamp01(key.position);
            key.bounds = SanitizeTimingColouriseBounds(key.bounds);
            if (!IsValidInterpolation(key.interpolation)) {
                key.interpolation = invisible_places::water::
                    WaterScenarioInterpolation::Smooth;
            }
        }
        for (auto& key : memory.boundsParameterKeys) {
            key.position = Clamp01(key.position);
            key.value =
                SanitizeBoundsParameterValue(key.parameter, key.value);
            if (!IsValidInterpolation(key.interpolation)) {
                key.interpolation = invisible_places::water::
                    WaterScenarioInterpolation::Smooth;
            }
        }
        SortAndCoalesceKeys(&memory.boundsKeys);
        SortAndCoalesceBoundsParameterKeys(&memory.boundsParameterKeys);
        fieldBoundsMemory.push_back(std::move(memory));
    }
    effect.fieldBoundsMemory = std::move(fieldBoundsMemory);
    std::vector<TimingColouriseFieldVisualMemory> fieldVisualMemory;
    fieldVisualMemory.reserve(effect.fieldVisualMemory.size());
    for (auto& memory : effect.fieldVisualMemory) {
        if (memory.selector.source ==
                TimingColouriseFieldSource::Scalar &&
            memory.selector.scalarFieldName.empty()) {
            continue;
        }
        const bool duplicate = std::any_of(
            fieldVisualMemory.begin(),
            fieldVisualMemory.end(),
            [&](const TimingColouriseFieldVisualMemory& kept) {
                return kept.selector == memory.selector;
            });
        if (duplicate) {
            continue;
        }
        memory.basePalette =
            SanitizeTimingColourisePalette(std::move(memory.basePalette));
        if (!IsValidPaletteKeyModel(memory.paletteKeyModel)) {
            memory.paletteKeyModel = memory.paletteKeys.empty()
                ? TimingColourisePaletteKeyModel::StopParameters
                : TimingColourisePaletteKeyModel::LegacySnapshots;
        }
        if (!IsValidPaletteSourceKind(memory.paletteSourceKind)) {
            memory.paletteSourceKind =
                TimingColourisePaletteSourceKind::Custom;
        }
        if (!IsValidColourSpace(memory.colourKeyInterpolationSpace)) {
            memory.colourKeyInterpolationSpace =
                TimingColouriseColourSpace::Srgb;
        }
        if (!IsValidAmountOverrideMode(
                memory.colouriseAmountOverrideMode)) {
            memory.colouriseAmountOverrideMode =
                TimingColouriseAmountOverrideMode::Maximum;
        }
        memory.colouriseAmountOverride = std::clamp(
            FiniteOr(memory.colouriseAmountOverride, 1.0F),
            0.0F,
            1.0F);
        memory.palettePhaseOffset =
            FiniteOr(memory.palettePhaseOffset, 0.0F);
        memory.paletteSkewCentre = std::clamp(
            FiniteOr(memory.paletteSkewCentre, 0.5F),
            0.0F,
            1.0F);
        memory.paletteSkewSpread =
            ClampPaletteSkew(memory.paletteSkewSpread);
        memory.emissiveSkewCentre = std::clamp(
            FiniteOr(memory.emissiveSkewCentre, 0.5F),
            0.0F,
            1.0F);
        memory.emissiveSkewSpread =
            ClampPaletteSkew(memory.emissiveSkewSpread);
        sanitizeSkewNodes(&memory.paletteSkewNodes);
        sanitizeSkewNodes(&memory.emissiveSkewNodes);
        sanitizeFalloff(
            &memory.emissiveFalloffNodes,
            &memory.emissiveFalloffKeys);
        memory.emissiveLevel = FiniteOr(memory.emissiveLevel, 1.0F);
        std::erase_if(
            memory.effectParameterKeys,
            [](const TimingColouriseEffectParameterKey& key) {
                return !IsValidEffectParameter(key.parameter);
            });
        for (auto& key : memory.effectParameterKeys) {
            key.position = Clamp01(key.position);
            key.value =
                SanitizeEffectParameterValue(key.parameter, key.value);
            if (!IsValidInterpolation(key.interpolation)) {
                key.interpolation = invisible_places::water::
                    WaterScenarioInterpolation::Smooth;
            }
        }
        for (auto& key : memory.paletteKeys) {
            key.position = Clamp01(key.position);
            key.palette =
                SanitizeTimingColourisePalette(std::move(key.palette));
            if (!IsValidInterpolation(key.interpolation)) {
                key.interpolation = invisible_places::water::
                    WaterScenarioInterpolation::Smooth;
            }
        }
        std::unordered_set<std::string> memoryStopIds;
        memoryStopIds.reserve(memory.basePalette.stops.size());
        for (const auto& stop : memory.basePalette.stops) {
            memoryStopIds.insert(stop.id);
        }
        std::erase_if(
            memory.paletteStopParameterKeys,
            [&](const TimingColourisePaletteStopParameterKey& key) {
                return key.stopId.empty() ||
                       !memoryStopIds.contains(key.stopId) ||
                       !IsValidPaletteStopParameter(key.parameter);
            });
        for (auto& key : memory.paletteStopParameterKeys) {
            key.position = Clamp01(key.position);
            key.scalarValue = SanitizePaletteStopScalarValue(
                key.parameter,
                key.scalarValue);
            key.colourValue =
                SanitizePaletteStopColour(key.colourValue);
            if (!IsValidInterpolation(key.interpolation)) {
                key.interpolation = invisible_places::water::
                    WaterScenarioInterpolation::Smooth;
            }
        }
        SortAndCoalesceKeys(&memory.paletteKeys);
        SortAndCoalescePaletteStopParameterKeys(
            &memory.paletteStopParameterKeys);
        SortAndCoalesceEffectParameterKeys(
            &memory.effectParameterKeys);
        fieldVisualMemory.push_back(std::move(memory));
    }
    effect.fieldVisualMemory = std::move(fieldVisualMemory);
    return effect;
}

TimingColourisePaletteDefinition SanitizeTimingColourisePaletteDefinition(
    TimingColourisePaletteDefinition definition) {
    if (definition.name.empty()) {
        definition.name = "Palette";
    }
    definition.palette =
        SanitizeTimingColourisePalette(std::move(definition.palette));
    return definition;
}

const TimingColouriseLocalPaletteEdit*
FindTimingColouriseLocalPaletteEdit(
    const TimingColouriseEffect& effect,
    TimingColourisePaletteSourceKind sourceKind,
    std::string_view sourceId) {
    const auto found = std::find_if(
        effect.localPaletteEdits.begin(),
        effect.localPaletteEdits.end(),
        [&](const TimingColouriseLocalPaletteEdit& edit) {
            return edit.sourceKind == sourceKind &&
                   edit.presetId == sourceId;
        });
    return found == effect.localPaletteEdits.end() ? nullptr : &*found;
}

const TimingColouriseLocalPaletteEdit*
FindTimingColouriseLocalPaletteEdit(
    const TimingColouriseEffect& effect,
    std::string_view presetId) {
    return FindTimingColouriseLocalPaletteEdit(
        effect,
        TimingColourisePaletteSourceKind::Preset,
        presetId);
}

bool UpsertTimingColouriseLocalPaletteEdit(
    TimingColouriseEffect* effect,
    TimingColourisePalette palette) {
    if (effect == nullptr) {
        return false;
    }
    // Only library-backed sources own "_edited" variants; a Custom palette
    // is already effect-local state.
    if ((effect->paletteSourceKind !=
             TimingColourisePaletteSourceKind::Preset &&
         effect->paletteSourceKind !=
             TimingColourisePaletteSourceKind::Saved) ||
        effect->paletteSourceId.empty()) {
        return false;
    }
    palette = SanitizeTimingColourisePalette(std::move(palette));
    const auto existing = std::find_if(
        effect->localPaletteEdits.begin(),
        effect->localPaletteEdits.end(),
        [&](const TimingColouriseLocalPaletteEdit& edit) {
            return edit.sourceKind == effect->paletteSourceKind &&
                   edit.presetId == effect->paletteSourceId;
        });
    TimingColouriseLocalPaletteEdit localEdit{
        .sourceKind = effect->paletteSourceKind,
        .presetId = effect->paletteSourceId,
        .presetName = effect->paletteSourceName.empty()
                          ? effect->paletteSourceId
                          : effect->paletteSourceName,
        .palette = palette,
    };
    if (existing == effect->localPaletteEdits.end()) {
        effect->localPaletteEdits.push_back(std::move(localEdit));
    } else {
        *existing = std::move(localEdit);
    }
    effect->basePalette = std::move(palette);
    effect->paletteEdited = true;
    return true;
}

bool ActivateTimingColouriseOriginalSource(
    TimingColouriseEffect* effect,
    TimingColourisePaletteSourceKind sourceKind,
    const TimingColourisePaletteDefinition& definition) {
    if (effect == nullptr || definition.id.empty() ||
        (sourceKind != TimingColourisePaletteSourceKind::Preset &&
         sourceKind != TimingColourisePaletteSourceKind::Saved)) {
        return false;
    }
    auto updated = SanitizeTimingColouriseEffect(*effect);
    const auto sanitizedDefinition =
        SanitizeTimingColourisePaletteDefinition(definition);
    updated.basePalette = sanitizedDefinition.palette;
    updated.paletteSourceKind = sourceKind;
    updated.paletteSourceId = sanitizedDefinition.id;
    updated.paletteSourceName = sanitizedDefinition.name;
    updated.paletteEdited = false;
    *effect = SanitizeTimingColouriseEffect(std::move(updated));
    return true;
}

bool ActivateTimingColouriseOriginalPreset(
    TimingColouriseEffect* effect,
    const TimingColourisePaletteDefinition& preset) {
    return ActivateTimingColouriseOriginalSource(
        effect,
        TimingColourisePaletteSourceKind::Preset,
        preset);
}

bool ActivateTimingColouriseLocalPaletteEdit(
    TimingColouriseEffect* effect,
    TimingColourisePaletteSourceKind sourceKind,
    std::string_view sourceId) {
    if (effect == nullptr || sourceId.empty()) {
        return false;
    }
    auto updated = SanitizeTimingColouriseEffect(*effect);
    const auto localEdit = std::find_if(
        updated.localPaletteEdits.begin(),
        updated.localPaletteEdits.end(),
        [&](const TimingColouriseLocalPaletteEdit& edit) {
            return edit.sourceKind == sourceKind &&
                   edit.presetId == sourceId;
        });
    if (localEdit == updated.localPaletteEdits.end()) {
        return false;
    }
    updated.basePalette = localEdit->palette;
    updated.paletteSourceKind = sourceKind;
    updated.paletteSourceId = localEdit->presetId;
    updated.paletteSourceName = localEdit->presetName;
    updated.paletteEdited = true;
    *effect = SanitizeTimingColouriseEffect(std::move(updated));
    return true;
}

bool ActivateTimingColouriseLocalPaletteEdit(
    TimingColouriseEffect* effect,
    std::string_view presetId) {
    return ActivateTimingColouriseLocalPaletteEdit(
        effect,
        TimingColourisePaletteSourceKind::Preset,
        presetId);
}

bool DiscardTimingColouriseLocalPaletteEdit(
    TimingColouriseEffect* effect,
    TimingColourisePaletteSourceKind sourceKind,
    const TimingColourisePaletteDefinition& originalDefinition) {
    if (effect == nullptr || originalDefinition.id.empty()) {
        return false;
    }
    auto updated = SanitizeTimingColouriseEffect(*effect);
    const auto localEdit = std::find_if(
        updated.localPaletteEdits.begin(),
        updated.localPaletteEdits.end(),
        [&](const TimingColouriseLocalPaletteEdit& edit) {
            return edit.sourceKind == sourceKind &&
                   edit.presetId == originalDefinition.id;
        });
    if (localEdit == updated.localPaletteEdits.end()) {
        return false;
    }
    const bool discardingActiveEdit =
        updated.paletteSourceKind == sourceKind &&
        updated.paletteEdited &&
        updated.paletteSourceId == originalDefinition.id;
    updated.localPaletteEdits.erase(localEdit);
    if (discardingActiveEdit) {
        const auto sanitizedDefinition =
            SanitizeTimingColourisePaletteDefinition(originalDefinition);
        updated.basePalette = sanitizedDefinition.palette;
        updated.paletteSourceId = sanitizedDefinition.id;
        updated.paletteSourceName = sanitizedDefinition.name;
        updated.paletteEdited = false;
    }
    *effect = SanitizeTimingColouriseEffect(std::move(updated));
    return true;
}

bool DiscardTimingColouriseLocalPaletteEdit(
    TimingColouriseEffect* effect,
    const TimingColourisePaletteDefinition& originalPreset) {
    return DiscardTimingColouriseLocalPaletteEdit(
        effect,
        TimingColourisePaletteSourceKind::Preset,
        originalPreset);
}

std::optional<TimingColourisePaletteDefinition>
PromoteTimingColouriseLocalPaletteEdit(
    TimingColouriseEffect* effect,
    TimingColourisePaletteSourceKind sourceKind,
    std::string_view sourceId,
    std::string savedPaletteId,
    std::string savedPaletteName) {
    if (effect == nullptr || sourceId.empty() ||
        savedPaletteId.empty()) {
        return std::nullopt;
    }
    auto updated = SanitizeTimingColouriseEffect(*effect);
    const auto localEdit = std::find_if(
        updated.localPaletteEdits.begin(),
        updated.localPaletteEdits.end(),
        [&](const TimingColouriseLocalPaletteEdit& edit) {
            return edit.sourceKind == sourceKind &&
                   edit.presetId == sourceId;
        });
    if (localEdit == updated.localPaletteEdits.end()) {
        return std::nullopt;
    }
    auto definition = SanitizeTimingColourisePaletteDefinition(
        TimingColourisePaletteDefinition{
            .id = std::move(savedPaletteId),
            .name = std::move(savedPaletteName),
            .palette = localEdit->palette,
        });
    updated.localPaletteEdits.erase(localEdit);
    updated.basePalette = definition.palette;
    updated.paletteSourceKind = TimingColourisePaletteSourceKind::Saved;
    updated.paletteSourceId = definition.id;
    updated.paletteSourceName = definition.name;
    updated.paletteEdited = false;
    *effect = SanitizeTimingColouriseEffect(std::move(updated));
    return definition;
}

std::optional<TimingColourisePaletteDefinition>
PromoteTimingColouriseLocalPaletteEdit(
    TimingColouriseEffect* effect,
    std::string_view presetId,
    std::string savedPaletteId,
    std::string savedPaletteName) {
    return PromoteTimingColouriseLocalPaletteEdit(
        effect,
        TimingColourisePaletteSourceKind::Preset,
        presetId,
        std::move(savedPaletteId),
        std::move(savedPaletteName));
}

TimingTakeDefinition SanitizeTimingTakeDefinition(
    TimingTakeDefinition definition) {
    definition.id = NormalizeTimingTakeId(definition.id);
    if (definition.id == kAuthoredTimingTakeId) {
        definition.name = std::string{kAuthoredTimingTakeName};
        // The built-in take is the fallback context for every scene, so it
        // never carries a scene scope.
        definition.sceneGroup.clear();
    } else if (definition.name.empty()) {
        definition.name = "Timing Take";
    }
    definition.assignedRainProfileId = TrimRainProfileText(
        definition.assignedRainProfileId);
    definition.assignedRainProfileName = TrimRainProfileText(
        definition.assignedRainProfileName);
    definition.baseRainProfileId = TrimRainProfileText(
        definition.baseRainProfileId);
    definition.baseRainProfileName = TrimRainProfileText(
        definition.baseRainProfileName);
    return definition;
}

TimingTakeSceneState SanitizeTimingTakeSceneState(
    TimingTakeSceneState state) {
    state.takeId = NormalizeTimingTakeId(state.takeId);
    if (state.sceneGroupName.empty()) {
        state.sceneGroupName = "Default";
    }
    for (auto& run : state.waterFeatureTimingRuns) {
        run = invisible_places::water::SanitizeWaterFeatureTimingRun(
            std::move(run));
    }
    for (auto& effect : state.colouriseEffects) {
        effect = SanitizeTimingColouriseEffect(std::move(effect));
    }
    state.waterFeatureTimingRunSequence =
        std::max(1U, state.waterFeatureTimingRunSequence);
    state.colouriseEffectSequence =
        std::max(1U, state.colouriseEffectSequence);
    return state;
}

namespace {

bool IsCurrentTimingColouriseFieldMemory(
    const TimingColouriseEffect& effect,
    const TimingColouriseFieldBoundsMemory& memory) {
    return memory.selector == effect.field;
}

template <typename Key>
void AppendTimingColouriseSettingsKeyPositions(
    std::vector<float>* positions,
    const std::vector<Key>& keys) {
    for (const auto& key : keys) {
        if (!std::isfinite(key.position)) {
            continue;
        }
        positions->push_back(Clamp01(key.position));
    }
}

template <typename Key>
void ExtendTimingColouriseSettingsKeySpan(
    std::optional<TimingColouriseSettingsKeySpan>* span,
    const std::vector<Key>& keys) {
    for (const auto& key : keys) {
        if (!std::isfinite(key.position)) {
            continue;
        }
        const float position = Clamp01(key.position);
        if (!span->has_value()) {
            *span = TimingColouriseSettingsKeySpan{
                .start = position,
                .end = position,
            };
        } else {
            (*span)->start = std::min((*span)->start, position);
            (*span)->end = std::max((*span)->end, position);
        }
    }
}

template <typename Key, typename SameLane>
bool TimingColouriseKeysHaveNoLaneCollisions(
    const std::vector<Key>& keys,
    SameLane sameLane) {
    for (std::size_t left = 0U; left < keys.size(); ++left) {
        for (std::size_t right = left + 1U;
             right < keys.size();
             ++right) {
            if (sameLane(keys[left], keys[right]) &&
                std::abs(
                    keys[left].position - keys[right].position) <=
                    kTimingColouriseKeyTolerance) {
                return false;
            }
        }
    }
    return true;
}

bool TimingColouriseEffectSettingsKeysHaveNoLaneCollisions(
    const TimingColouriseEffect& effect) {
    const auto sameSingleLane = [](const auto&, const auto&) {
        return true;
    };
    if (!TimingColouriseKeysHaveNoLaneCollisions(
            effect.paletteKeys,
            sameSingleLane) ||
        !TimingColouriseKeysHaveNoLaneCollisions(
            effect.paletteStopParameterKeys,
            [](const auto& left, const auto& right) {
                return left.stopId == right.stopId &&
                       left.parameter == right.parameter;
            }) ||
        !TimingColouriseKeysHaveNoLaneCollisions(
            effect.effectParameterKeys,
            [](const auto& left, const auto& right) {
                return left.parameter == right.parameter;
            }) ||
        !TimingColouriseKeysHaveNoLaneCollisions(
            effect.boundsKeys,
            sameSingleLane) ||
        !TimingColouriseKeysHaveNoLaneCollisions(
            effect.boundsParameterKeys,
            [](const auto& left, const auto& right) {
                return left.parameter == right.parameter;
            }) ||
        !TimingColouriseKeysHaveNoLaneCollisions(
            effect.emissiveFalloffKeys,
            [](const auto& left, const auto& right) {
                return left.nodeId == right.nodeId &&
                       left.parameter == right.parameter;
            })) {
        return false;
    }
    for (const auto& memory : effect.fieldBoundsMemory) {
        if (IsCurrentTimingColouriseFieldMemory(effect, memory)) {
            continue;
        }
        if (!TimingColouriseKeysHaveNoLaneCollisions(
                memory.boundsKeys,
                sameSingleLane) ||
            !TimingColouriseKeysHaveNoLaneCollisions(
                memory.boundsParameterKeys,
                [](const auto& left, const auto& right) {
                    return left.parameter == right.parameter;
                })) {
            return false;
        }
    }
    return true;
}

template <typename Key, typename SameLane>
bool TimingColouriseKeysHaveNoCyclicLaneCollisions(
    const std::vector<Key>& keys,
    SameLane sameLane) {
    for (std::size_t left = 0U; left < keys.size(); ++left) {
        for (std::size_t right = left + 1U;
             right < keys.size();
             ++right) {
            if (sameLane(keys[left], keys[right]) &&
                TimingColouriseKeyPositionsCoincideCyclically(
                    keys[left].position,
                    keys[right].position)) {
                return false;
            }
        }
    }
    return true;
}

// Same lane rules as the linear check, but 0.0 and 1.0 count as one instant
// because cyclic evaluation coalesces them before triplicating the keys.
bool TimingColouriseEffectSettingsKeysHaveNoCyclicLaneCollisions(
    const TimingColouriseEffect& effect) {
    const auto sameSingleLane = [](const auto&, const auto&) {
        return true;
    };
    if (!TimingColouriseKeysHaveNoCyclicLaneCollisions(
            effect.paletteKeys,
            sameSingleLane) ||
        !TimingColouriseKeysHaveNoCyclicLaneCollisions(
            effect.paletteStopParameterKeys,
            [](const auto& left, const auto& right) {
                return left.stopId == right.stopId &&
                       left.parameter == right.parameter;
            }) ||
        !TimingColouriseKeysHaveNoCyclicLaneCollisions(
            effect.effectParameterKeys,
            [](const auto& left, const auto& right) {
                return left.parameter == right.parameter;
            }) ||
        !TimingColouriseKeysHaveNoCyclicLaneCollisions(
            effect.boundsKeys,
            sameSingleLane) ||
        !TimingColouriseKeysHaveNoCyclicLaneCollisions(
            effect.boundsParameterKeys,
            [](const auto& left, const auto& right) {
                return left.parameter == right.parameter;
            }) ||
        !TimingColouriseKeysHaveNoCyclicLaneCollisions(
            effect.emissiveFalloffKeys,
            [](const auto& left, const auto& right) {
                return left.nodeId == right.nodeId &&
                       left.parameter == right.parameter;
            })) {
        return false;
    }
    for (const auto& memory : effect.fieldBoundsMemory) {
        if (IsCurrentTimingColouriseFieldMemory(effect, memory)) {
            continue;
        }
        if (!TimingColouriseKeysHaveNoCyclicLaneCollisions(
                memory.boundsKeys,
                sameSingleLane) ||
            !TimingColouriseKeysHaveNoCyclicLaneCollisions(
                memory.boundsParameterKeys,
                [](const auto& left, const auto& right) {
                    return left.parameter == right.parameter;
                })) {
            return false;
        }
    }
    return true;
}

void SynchronizeCurrentTimingColouriseFieldMemory(
    TimingColouriseEffect* effect) {
    for (auto& memory : effect->fieldBoundsMemory) {
        if (!IsCurrentTimingColouriseFieldMemory(*effect, memory)) {
            continue;
        }
        memory.bounds = effect->baseBounds;
        memory.boundsKeyMode = effect->boundsKeyMode;
        memory.boundsParameterKeys = effect->boundsParameterKeys;
        memory.boundsKeys = effect->boundsKeys;
        memory.edited = effect->boundsEdited;
        memory.adoptedGlobalRevision =
            effect->boundsAdoptedGlobalRevision;
    }
}

template <typename Key, typename SameLane>
std::size_t CoalesceCyclicallyCoincidentKeys(
    std::vector<Key>* keys,
    SameLane sameLane) {
    // Mirror ExpandTimingKeysForCyclicEvaluation, which wraps 1.0 onto 0.0
    // and keeps the later key of a coincident pair: the linear-earlier key
    // is the one evaluation was already ignoring in the cyclic lens.
    std::vector<bool> drop(keys->size(), false);
    for (std::size_t left = 0U; left < keys->size(); ++left) {
        for (std::size_t right = left + 1U;
             right < keys->size();
             ++right) {
            if (!sameLane((*keys)[left], (*keys)[right]) ||
                !TimingColouriseKeyPositionsCoincideCyclically(
                    (*keys)[left].position,
                    (*keys)[right].position)) {
                continue;
            }
            const bool leftEarlier =
                (*keys)[left].position <= (*keys)[right].position;
            drop[leftEarlier ? left : right] = true;
        }
    }
    std::vector<Key> kept;
    kept.reserve(keys->size());
    for (std::size_t index = 0U; index < keys->size(); ++index) {
        if (!drop[index]) {
            kept.push_back(std::move((*keys)[index]));
        }
    }
    const std::size_t removed = keys->size() - kept.size();
    *keys = std::move(kept);
    return removed;
}

// Indices of the Palette Phase keys in the order evaluation accumulates
// their deltas (time order; ties keep the stored order like a stable sort).
std::vector<std::size_t> PalettePhaseKeyAccumulationOrder(
    const std::vector<TimingColouriseEffectParameterKey>& keys) {
    std::vector<std::size_t> order;
    for (std::size_t index = 0U; index < keys.size(); ++index) {
        if (keys[index].parameter ==
            TimingColouriseEffectParameter::PalettePhase) {
            order.push_back(index);
        }
    }
    std::stable_sort(
        order.begin(),
        order.end(),
        [&](std::size_t left, std::size_t right) {
            return keys[left].position < keys[right].position;
        });
    return order;
}

// Rewrites every Palette Phase key's value as its accumulated target, the
// value cyclic evaluation works with once it has merged coincident keys.
void EncodePalettePhaseKeysAsTargets(TimingColouriseEffect* effect) {
    float phase = FiniteOr(effect->palettePhaseOffset, 0.0F);
    for (const std::size_t index :
         PalettePhaseKeyAccumulationOrder(effect->effectParameterKeys)) {
        auto& key = effect->effectParameterKeys[index];
        phase += ClampPalettePhaseDelta(key.value);
        key.value = phase;
    }
}

// Inverse of EncodePalettePhaseKeysAsTargets over whichever keys survived.
void EncodePalettePhaseKeysAsDeltas(TimingColouriseEffect* effect) {
    float previous = FiniteOr(effect->palettePhaseOffset, 0.0F);
    for (const std::size_t index :
         PalettePhaseKeyAccumulationOrder(effect->effectParameterKeys)) {
        auto& key = effect->effectParameterKeys[index];
        const float target = key.value;
        key.value = WrapTimingColourisePalettePhaseDelta(target - previous);
        previous = target;
    }
}

void SortTimingColouriseEffectSettingsKeys(
    TimingColouriseEffect* effect) {
    SortAndCoalesceKeys(&effect->paletteKeys);
    SortAndCoalescePaletteStopParameterKeys(
        &effect->paletteStopParameterKeys);
    SortAndCoalesceEffectParameterKeys(
        &effect->effectParameterKeys);
    SortAndCoalesceKeys(&effect->boundsKeys);
    SortAndCoalesceBoundsParameterKeys(
        &effect->boundsParameterKeys);
    for (auto& memory : effect->fieldBoundsMemory) {
        SortAndCoalesceKeys(&memory.boundsKeys);
        SortAndCoalesceBoundsParameterKeys(
            &memory.boundsParameterKeys);
    }
}

}  // namespace

std::size_t CoalesceTimingColouriseEffectCyclicallyCoincidentKeys(
    TimingColouriseEffect* effect) {
    if (effect == nullptr) {
        return 0U;
    }
    const auto sameSingleLane = [](const auto&, const auto&) {
        return true;
    };
    std::size_t removed = 0U;
    removed += CoalesceCyclicallyCoincidentKeys(
        &effect->paletteKeys,
        sameSingleLane);
    removed += CoalesceCyclicallyCoincidentKeys(
        &effect->paletteStopParameterKeys,
        [](const auto& left, const auto& right) {
            return left.stopId == right.stopId &&
                   left.parameter == right.parameter;
        });
    // Palette Phase is delta-encoded and every key's delta, including the
    // one at 0.0 that cyclic evaluation replaces with the 1.0 key, has
    // already been accumulated into the later keys by the time evaluation
    // merges the pair. Merge on absolute targets so dropping a key never
    // drops its delta, then re-difference the survivors. Only a merged phase
    // lane is re-encoded; untouched tracks keep their authored floats.
    const auto sameParameter = [](const auto& left, const auto& right) {
        return left.parameter == right.parameter;
    };
    const auto phaseKeyCount = [&]() {
        return PalettePhaseKeyAccumulationOrder(effect->effectParameterKeys)
            .size();
    };
    auto authoredEffectKeys = effect->effectParameterKeys;
    const std::size_t authoredPhaseKeys = phaseKeyCount();
    EncodePalettePhaseKeysAsTargets(effect);
    const std::size_t removedEffectKeys = CoalesceCyclicallyCoincidentKeys(
        &effect->effectParameterKeys,
        sameParameter);
    if (phaseKeyCount() == authoredPhaseKeys) {
        // No phase key merged, so the authored deltas are still right;
        // coalescing depends only on positions, so replaying it on the
        // authored copy drops the same (non-phase) keys bit-for-bit.
        effect->effectParameterKeys = std::move(authoredEffectKeys);
        (void)CoalesceCyclicallyCoincidentKeys(
            &effect->effectParameterKeys,
            sameParameter);
    } else {
        EncodePalettePhaseKeysAsDeltas(effect);
    }
    removed += removedEffectKeys;
    removed += CoalesceCyclicallyCoincidentKeys(
        &effect->boundsKeys,
        sameSingleLane);
    removed += CoalesceCyclicallyCoincidentKeys(
        &effect->boundsParameterKeys,
        [](const auto& left, const auto& right) {
            return left.parameter == right.parameter;
        });
    for (auto& memory : effect->fieldBoundsMemory) {
        if (IsCurrentTimingColouriseFieldMemory(*effect, memory)) {
            continue;
        }
        removed += CoalesceCyclicallyCoincidentKeys(
            &memory.boundsKeys,
            sameSingleLane);
        removed += CoalesceCyclicallyCoincidentKeys(
            &memory.boundsParameterKeys,
            [](const auto& left, const auto& right) {
                return left.parameter == right.parameter;
            });
    }
    if (removed != 0U) {
        SynchronizeCurrentTimingColouriseFieldMemory(effect);
    }
    return removed;
}

std::vector<float> TimingColouriseEffectSettingsKeyPositions(
    const TimingColouriseEffect& effect) {
    std::vector<float> positions;
    positions.reserve(
        effect.effectParameterKeys.size() +
        effect.paletteKeys.size() +
        effect.paletteStopParameterKeys.size() +
        effect.boundsParameterKeys.size() +
        effect.boundsKeys.size());
    AppendTimingColouriseSettingsKeyPositions(
        &positions,
        effect.effectParameterKeys);
    AppendTimingColouriseSettingsKeyPositions(
        &positions,
        effect.paletteKeys);
    AppendTimingColouriseSettingsKeyPositions(
        &positions,
        effect.paletteStopParameterKeys);
    AppendTimingColouriseSettingsKeyPositions(
        &positions,
        effect.boundsParameterKeys);
    AppendTimingColouriseSettingsKeyPositions(
        &positions,
        effect.boundsKeys);
    AppendTimingColouriseSettingsKeyPositions(
        &positions,
        effect.emissiveFalloffKeys);
    for (const auto& memory : effect.fieldBoundsMemory) {
        // The live vectors above are authoritative for the selected field.
        // Its remembered entry is only the snapshot from the last switch and
        // may legitimately lag until the next Stash call.
        if (IsCurrentTimingColouriseFieldMemory(effect, memory)) {
            continue;
        }
        AppendTimingColouriseSettingsKeyPositions(
            &positions,
            memory.boundsParameterKeys);
        AppendTimingColouriseSettingsKeyPositions(
            &positions,
            memory.boundsKeys);
    }
    std::stable_sort(positions.begin(), positions.end());
    positions.erase(
        std::unique(
            positions.begin(),
            positions.end(),
            [](float left, float right) {
                return std::abs(left - right) <=
                       kTimingColouriseKeyTolerance;
            }),
        positions.end());
    return positions;
}

std::optional<TimingColouriseSettingsKeySpan>
TimingColouriseEffectSettingsKeySpan(
    const TimingColouriseEffect& effect) {
    std::optional<TimingColouriseSettingsKeySpan> span;
    ExtendTimingColouriseSettingsKeySpan(
        &span,
        effect.effectParameterKeys);
    ExtendTimingColouriseSettingsKeySpan(&span, effect.paletteKeys);
    ExtendTimingColouriseSettingsKeySpan(
        &span,
        effect.paletteStopParameterKeys);
    ExtendTimingColouriseSettingsKeySpan(
        &span,
        effect.boundsParameterKeys);
    ExtendTimingColouriseSettingsKeySpan(&span, effect.boundsKeys);
    ExtendTimingColouriseSettingsKeySpan(
        &span,
        effect.emissiveFalloffKeys);
    for (const auto& memory : effect.fieldBoundsMemory) {
        if (IsCurrentTimingColouriseFieldMemory(effect, memory)) {
            continue;
        }
        ExtendTimingColouriseSettingsKeySpan(
            &span,
            memory.boundsParameterKeys);
        ExtendTimingColouriseSettingsKeySpan(
            &span,
            memory.boundsKeys);
    }
    return span;
}

bool TransformTimingColouriseEffectSettingsKeys(
    TimingColouriseEffect* effect,
    TimingColouriseSettingsKeySpan source,
    TimingColouriseSettingsKeySpan destination) {
    if (effect == nullptr || !std::isfinite(source.start) ||
        !std::isfinite(source.end) ||
        !std::isfinite(destination.start) ||
        !std::isfinite(destination.end) ||
        source.start < -kTimingColouriseKeyTolerance ||
        source.end > 1.0F + kTimingColouriseKeyTolerance ||
        source.end < source.start ||
        destination.start < -kTimingColouriseKeyTolerance ||
        destination.end > 1.0F + kTimingColouriseKeyTolerance ||
        destination.end < destination.start) {
        return false;
    }
    source.start = Clamp01(source.start);
    source.end = Clamp01(source.end);
    destination.start = Clamp01(destination.start);
    destination.end = Clamp01(destination.end);

    const float sourceLength = source.end - source.start;
    const float destinationLength = destination.end - destination.start;
    const bool pointSource = source.start == source.end;
    if (pointSource && destination.start != destination.end) {
        // One key time has no internal timing that can be stretched.
        return false;
    }
    if (!pointSource && destination.start == destination.end) {
        return false;
    }

    TimingColouriseEffect candidate = *effect;
    bool foundKey = false;
    const auto mapKeys = [&](auto* keys) {
        for (auto& key : *keys) {
            foundKey = true;
            if (!std::isfinite(key.position) ||
                key.position <
                    source.start - kTimingColouriseKeyTolerance ||
                key.position >
                    source.end + kTimingColouriseKeyTolerance) {
                return false;
            }
            const double remapped = pointSource
                ? static_cast<double>(key.position) +
                      static_cast<double>(destination.start - source.start)
                : static_cast<double>(destination.start) +
                      (static_cast<double>(key.position) -
                       static_cast<double>(source.start)) /
                          static_cast<double>(sourceLength) *
                          static_cast<double>(destinationLength);
            if (!std::isfinite(remapped) ||
                remapped <
                    -static_cast<double>(kTimingColouriseKeyTolerance) ||
                remapped >
                    1.0 +
                        static_cast<double>(
                            kTimingColouriseKeyTolerance) ||
                remapped <
                    static_cast<double>(destination.start) -
                        static_cast<double>(
                            kTimingColouriseKeyTolerance) ||
                remapped >
                    static_cast<double>(destination.end) +
                        static_cast<double>(
                            kTimingColouriseKeyTolerance)) {
                return false;
            }
            key.position = Clamp01(static_cast<float>(remapped));
        }
        return true;
    };

    if (!mapKeys(&candidate.effectParameterKeys) ||
        !mapKeys(&candidate.paletteKeys) ||
        !mapKeys(&candidate.paletteStopParameterKeys) ||
        !mapKeys(&candidate.boundsParameterKeys) ||
        !mapKeys(&candidate.boundsKeys) ||
        !mapKeys(&candidate.emissiveFalloffKeys)) {
        return false;
    }
    for (auto& memory : candidate.fieldBoundsMemory) {
        if (IsCurrentTimingColouriseFieldMemory(candidate, memory)) {
            continue;
        }
        if (!mapKeys(&memory.boundsParameterKeys) ||
            !mapKeys(&memory.boundsKeys)) {
            return false;
        }
    }
    if (!foundKey ||
        !TimingColouriseEffectSettingsKeysHaveNoLaneCollisions(
            candidate)) {
        return false;
    }

    // Bring an existing selected-field cache forward from the transformed
    // live tracks. Do not manufacture a cache for effects that never needed
    // one merely because their clip moved.
    SynchronizeCurrentTimingColouriseFieldMemory(&candidate);
    SortTimingColouriseEffectSettingsKeys(&candidate);
    *effect = std::move(candidate);
    return true;
}

std::optional<float> TimingColouriseSnapKeyDragOffset(
    std::span<const float> draggedPositions,
    std::span<const float> targetPositions,
    float rawOffset,
    float tolerance) {
    if (!std::isfinite(rawOffset) || !std::isfinite(tolerance) ||
        tolerance <= 0.0F) {
        return std::nullopt;
    }
    std::optional<float> best;
    float bestDistance = tolerance;
    for (const float dragged : draggedPositions) {
        if (!std::isfinite(dragged)) {
            continue;
        }
        for (const float target : targetPositions) {
            if (!std::isfinite(target)) {
                continue;
            }
            const float candidate = target - dragged;
            const float distance = std::abs(candidate - rawOffset);
            if (distance <= bestDistance) {
                bestDistance = distance;
                best = candidate;
            }
        }
    }
    if (best.has_value() &&
        std::abs(best.value() - rawOffset) <=
            std::numeric_limits<float>::epsilon()) {
        return std::nullopt;
    }
    return best;
}

float WrapTimingColouriseLoopPosition(float position) {
    position = FiniteOr(position, 0.0F);
    position -= std::floor(position);
    return position >= 1.0F ? 0.0F : position;
}

float TimingColouriseCyclicKeyDistance(float a, float b) {
    const float forward = std::abs(
        WrapTimingColouriseLoopPosition(a) -
        WrapTimingColouriseLoopPosition(b));
    return std::min(forward, 1.0F - forward);
}

bool TimingColouriseKeyPositionsCoincideCyclically(float a, float b) {
    // Deliberately not TimingColouriseCyclicKeyDistance: evaluation wraps
    // and then compares linearly, so a pair straddling loop zero by a hair
    // (0.99998 and 0.00005) is two instants there and must be two here.
    return std::abs(
               WrapTimingColouriseLoopPosition(a) -
               WrapTimingColouriseLoopPosition(b)) <=
           kTimingColouriseKeyTolerance;
}

float WrapTimingColourisePalettePhaseDelta(float delta) {
    if (!std::isfinite(delta)) {
        return 0.0F;
    }
    if (delta > 1.0F || delta < -1.0F) {
        delta -= std::floor(delta);
    }
    return ClampPalettePhaseDelta(delta);
}

std::optional<TimingColouriseCyclicSettingsKeySpan>
TimingColouriseEffectCyclicSettingsKeySpan(
    const TimingColouriseEffect& effect) {
    auto positions = TimingColouriseEffectSettingsKeyPositions(effect);
    if (positions.empty()) {
        return std::nullopt;
    }
    // The linear positions keep 1.0 distinct from 0.0; on the loop they are
    // the same instant, so wrap and re-coalesce before measuring gaps. The
    // coalescing rule is the evaluator's (wrap, then compare linearly), so
    // keys a hair either side of loop zero stay two instants here exactly as
    // the cyclic transform and collision check treat them; merging them
    // would derive a span whose transform then maps both onto one position.
    for (auto& position : positions) {
        position = WrapTimingColouriseLoopPosition(position);
    }
    std::sort(positions.begin(), positions.end());
    std::vector<float> unique;
    unique.reserve(positions.size());
    for (const float position : positions) {
        if (unique.empty() ||
            !TimingColouriseKeyPositionsCoincideCyclically(
                unique.back(),
                position)) {
            unique.push_back(position);
        }
    }
    if (unique.size() == 1U) {
        return TimingColouriseCyclicSettingsKeySpan{
            .start = unique.front(),
            .length = 0.0F,
        };
    }

    float largestInteriorGap = -1.0F;
    for (std::size_t index = 0U; index + 1U < unique.size(); ++index) {
        largestInteriorGap =
            std::max(largestInteriorGap, unique[index + 1U] - unique[index]);
    }
    // Interior gaps that tie within tolerance (evenly spaced keys offset from
    // loop zero) must resolve the same way every frame, or float rounding of
    // the differences would pick a different cluster as the keys translate
    // and the derived clip would flip between arcs mid-drag. Take the first
    // tied gap in sorted order: the cluster whose start key is nearest after
    // loop zero.
    std::size_t largestInteriorGapIndex = 0U;
    for (std::size_t index = 0U; index + 1U < unique.size(); ++index) {
        if (unique[index + 1U] - unique[index] >=
            largestInteriorGap - kTimingColouriseKeyTolerance) {
            largestInteriorGapIndex = index;
            break;
        }
    }
    const float wrapGap = unique.front() + 1.0F - unique.back();
    // Ties (two keys half a loop apart, evenly spaced keys) resolve to the
    // canonical min..max so the cyclic clip agrees with the linear one for
    // every layout that does not actually straddle loop zero.
    if (wrapGap >= largestInteriorGap - kTimingColouriseKeyTolerance) {
        return TimingColouriseCyclicSettingsKeySpan{
            .start = unique.front(),
            .length = unique.back() - unique.front(),
        };
    }
    const float chosenGap = unique[largestInteriorGapIndex + 1U] -
                            unique[largestInteriorGapIndex];
    return TimingColouriseCyclicSettingsKeySpan{
        .start = unique[largestInteriorGapIndex + 1U],
        .length = 1.0F - chosenGap,
    };
}

bool TransformTimingColouriseEffectSettingsKeysCyclic(
    TimingColouriseEffect* effect,
    TimingColouriseCyclicSettingsKeySpan source,
    TimingColouriseCyclicSettingsKeySpan destination) {
    if (effect == nullptr || !std::isfinite(source.start) ||
        !std::isfinite(source.length) ||
        !std::isfinite(destination.start) ||
        !std::isfinite(destination.length) ||
        source.length < -kTimingColouriseKeyTolerance ||
        source.length > 1.0F + kTimingColouriseKeyTolerance ||
        destination.length < -kTimingColouriseKeyTolerance ||
        destination.length > 1.0F + kTimingColouriseKeyTolerance) {
        return false;
    }
    source.length = Clamp01(source.length);
    destination.length = Clamp01(destination.length);
    const bool pointSource = source.length == 0.0F;
    if (pointSource && destination.length != 0.0F) {
        // One key time has no internal timing that can be stretched.
        return false;
    }
    if (!pointSource && destination.length == 0.0F) {
        return false;
    }

    // Keys at 0.0 and 1.0 (the usual first/last layout authored in the
    // linear timeline) are one cyclic instant, so every destination maps
    // them onto the same wrapped position. Coalesce them first, exactly as
    // evaluation does, instead of letting the collision check below reject
    // every move of such a clip. The original is coalesced the same way so
    // the Palette Phase re-encode still sees index-aligned keys.
    TimingColouriseEffect original = *effect;
    (void)CoalesceTimingColouriseEffectCyclicallyCoincidentKeys(&original);
    TimingColouriseEffect candidate = original;
    bool foundKey = false;
    const auto mapKeys = [&](auto* keys) {
        for (auto& key : *keys) {
            foundKey = true;
            if (!std::isfinite(key.position)) {
                return false;
            }
            // Forward distance from the clip start; a key a hair before the
            // start (u ~ 1) is the start key seen from the other side.
            float u = WrapTimingColouriseLoopPosition(
                key.position - source.start);
            if (u >= 1.0F - kTimingColouriseKeyTolerance) {
                u = 0.0F;
            }
            if (u > source.length + kTimingColouriseKeyTolerance) {
                return false;
            }
            u = std::min(u, source.length);
            const double remapped = pointSource
                ? static_cast<double>(destination.start)
                : static_cast<double>(destination.start) +
                      static_cast<double>(u) /
                          static_cast<double>(source.length) *
                          static_cast<double>(destination.length);
            if (!std::isfinite(remapped)) {
                return false;
            }
            key.position = WrapTimingColouriseLoopPosition(
                static_cast<float>(remapped));
        }
        return true;
    };

    if (!mapKeys(&candidate.effectParameterKeys) ||
        !mapKeys(&candidate.paletteKeys) ||
        !mapKeys(&candidate.paletteStopParameterKeys) ||
        !mapKeys(&candidate.boundsParameterKeys) ||
        !mapKeys(&candidate.boundsKeys) ||
        !mapKeys(&candidate.emissiveFalloffKeys)) {
        return false;
    }
    for (auto& memory : candidate.fieldBoundsMemory) {
        if (IsCurrentTimingColouriseFieldMemory(candidate, memory)) {
            continue;
        }
        if (!mapKeys(&memory.boundsParameterKeys) ||
            !mapKeys(&memory.boundsKeys)) {
            return false;
        }
    }
    if (!foundKey ||
        !TimingColouriseEffectSettingsKeysHaveNoCyclicLaneCollisions(
            candidate)) {
        return false;
    }

    PreserveTimingColourisePalettePhaseTargetsAfterMove(original, &candidate);
    SynchronizeCurrentTimingColouriseFieldMemory(&candidate);
    SortTimingColouriseEffectSettingsKeys(&candidate);
    *effect = std::move(candidate);
    return true;
}

void PreserveTimingColourisePalettePhaseTargetsAfterMove(
    const TimingColouriseEffect& original,
    TimingColouriseEffect* moved) {
    if (moved == nullptr ||
        moved->effectParameterKeys.size() !=
            original.effectParameterKeys.size()) {
        return;
    }
    std::vector<std::size_t> phaseKeys;
    for (std::size_t index = 0U;
         index < original.effectParameterKeys.size();
         ++index) {
        if (original.effectParameterKeys[index].parameter ==
                TimingColouriseEffectParameter::PalettePhase &&
            moved->effectParameterKeys[index].parameter ==
                TimingColouriseEffectParameter::PalettePhase) {
            phaseKeys.push_back(index);
        }
    }
    if (phaseKeys.size() < 2U) {
        return;
    }
    const auto timeOrder = [&](const std::vector<
                               TimingColouriseEffectParameterKey>& keys) {
        auto order = phaseKeys;
        std::stable_sort(
            order.begin(),
            order.end(),
            [&](std::size_t left, std::size_t right) {
                return keys[left].position < keys[right].position;
            });
        return order;
    };
    const auto originalOrder = timeOrder(original.effectParameterKeys);
    const auto movedOrder = timeOrder(moved->effectParameterKeys);
    if (originalOrder == movedOrder) {
        return;
    }

    // Accumulated targets in the original time order, exactly as evaluation
    // sees them (EvaluateEffectParameterTrack and the cyclic preparation).
    std::vector<float> absolute(original.effectParameterKeys.size(), 0.0F);
    float phase = FiniteOr(original.palettePhaseOffset, 0.0F);
    for (const std::size_t index : originalOrder) {
        phase += ClampPalettePhaseDelta(
            original.effectParameterKeys[index].value);
        absolute[index] = phase;
    }
    // Re-difference along the new order. A delta outside the stored [-1, 1]
    // range drops whole turns rather than clamping, so the keys still land
    // on the same palette rotation.
    float previous = FiniteOr(original.palettePhaseOffset, 0.0F);
    for (const std::size_t index : movedOrder) {
        moved->effectParameterKeys[index].value =
            WrapTimingColourisePalettePhaseDelta(absolute[index] - previous);
        previous = absolute[index];
    }
}

bool RetimeTimingTakeSceneStateNormalizedPositions(
    TimingTakeSceneState* state,
    std::uint32_t sourceDurationFrames,
    std::uint32_t destinationDurationFrames,
    std::uint32_t destinationStartFrame) {
    if (state == nullptr || sourceDurationFrames == 0U ||
        destinationDurationFrames == 0U ||
        static_cast<std::uint64_t>(destinationStartFrame) +
                sourceDurationFrames >
            destinationDurationFrames) {
        return false;
    }

    const double sourceFrames = static_cast<double>(sourceDurationFrames);
    const double destinationFrames =
        static_cast<double>(destinationDurationFrames);
    const double destinationStart =
        static_cast<double>(destinationStartFrame);
    const auto retimePosition = [sourceFrames,
                                 destinationFrames,
                                 destinationStart](float position) {
        const double finitePosition = std::isfinite(position)
                                          ? static_cast<double>(position)
                                          : 0.0;
        return static_cast<float>(std::clamp(
            (destinationStart + finitePosition * sourceFrames) /
                destinationFrames,
            0.0,
            1.0));
    };
    const auto retimeKeys = [&](auto* keys) {
        for (auto& key : *keys) {
            key.position = retimePosition(key.position);
        }
    };

    for (auto& run : state->waterFeatureTimingRuns) {
        for (auto& mark : run.marks) {
            mark.position = retimePosition(mark.position);
        }
        for (auto& feature : run.features) {
            // A clip ending on 1 whose head sits above 0 may own a member
            // stored at time 0: the seam state SynchronizeWaterFeatureClip-
            // Bounds documents (a linked tail key dragged onto phase 0,
            // measured as 1 while the clip ends on 1). Off the loop that
            // member is the clip's end, not its head, so it must retime as
            // 1 -- mapping the stored 0 through the position map would land
            // it at the destination start, outside the retimed span, and
            // the next Synchronize would flip the clip onto {0, start}.
            // A member stored at 1 alongside one at 0 means the 0 is a real
            // head key (the unlinked drag to the rail's left edge); those
            // retime linearly as before.
            constexpr float kSeamTolerance =
                water::kWaterFeatureClipPositionTolerance;
            for (const auto& clip : feature.clips) {
                if (water::WaterClipIsWrapped(clip.start, clip.end) ||
                    std::abs(clip.end - 1.0F) > kSeamTolerance ||
                    clip.start <= kSeamTolerance) {
                    continue;
                }
                bool memberOnPhaseOne = false;
                for (const auto& setting : feature.settings) {
                    for (const auto& key : setting.keys) {
                        if (key.clipId == clip.id &&
                            key.position >= 1.0F - kSeamTolerance) {
                            memberOnPhaseOne = true;
                        }
                    }
                }
                if (memberOnPhaseOne) {
                    continue;
                }
                for (auto& setting : feature.settings) {
                    bool rewrote = false;
                    for (auto& key : setting.keys) {
                        if (key.clipId == clip.id &&
                            std::isfinite(key.position) &&
                            key.position <= kSeamTolerance) {
                            key.position = 1.0F;
                            rewrote = true;
                        }
                    }
                    if (rewrote) {
                        // The seam member moves from the front of the track
                        // to its end; keys stay stored ascending.
                        std::stable_sort(
                            setting.keys.begin(),
                            setting.keys.end(),
                            [](const auto& left, const auto& right) {
                                return left.position < right.position;
                            });
                    }
                }
            }
            for (auto& setting : feature.settings) {
                retimeKeys(&setting.keys);
            }
            // Settings clips are spans over the same normalized domain, so
            // they retime exactly with the keys they group. A wrapped clip
            // (end > 1) retimes as start plus scaled length on its unwrapped
            // line: mapping the end through the clamping map would pin it to
            // 1 and silently unwrap the clip.
            const double lengthScale = sourceFrames / destinationFrames;
            std::vector<std::uint32_t> wrappedClipIds;
            for (auto& clip : feature.clips) {
                const float length = std::isfinite(clip.end) &&
                                             std::isfinite(clip.start)
                    ? std::clamp(clip.end - clip.start, 0.0F, 1.0F)
                    : 0.0F;
                if (water::WaterClipIsWrapped(clip.start, clip.end)) {
                    wrappedClipIds.push_back(clip.id);
                }
                clip.start = retimePosition(clip.start);
                clip.end = clip.start +
                           static_cast<float>(length * lengthScale);
            }
            // The source's phase 0 is no longer a seam once the take is a
            // sub-range of a longer animation, so a wrapped clip's members
            // (retimed individually above) no longer sit inside the scaled
            // span. Bounds are derived data: take the linear hull of the
            // retimed keys for clips that wrapped, and only those, so
            // documents whose clips never wrap retime bit-identically. The
            // hull is computed here rather than through
            // SynchronizeWaterFeatureClipBounds because a scaled span that
            // lands late in the destination can still exceed 1, which
            // would route that call into its cyclic branch and wrap the
            // clip through the destination's start/end -- covering the
            // complement of its keys. Keyless wrapped clips keep the
            // scaled span as their only information.
            for (const auto clipId : wrappedClipIds) {
                auto* clip = water::FindWaterFeatureClip(&feature, clipId);
                if (clip == nullptr) {
                    continue;
                }
                std::optional<std::pair<float, float>> hull;
                for (const auto& setting : feature.settings) {
                    for (const auto& key : setting.keys) {
                        if (key.clipId != clipId ||
                            !std::isfinite(key.position)) {
                            continue;
                        }
                        if (!hull.has_value()) {
                            hull = {key.position, key.position};
                        } else {
                            hull->first =
                                std::min(hull->first, key.position);
                            hull->second =
                                std::max(hull->second, key.position);
                        }
                    }
                }
                if (!hull.has_value()) {
                    continue;
                }
                clip->start = hull->first;
                clip->end = hull->second;
                // An unwrapped clip is the pre-W1 shape; its single-key
                // marker and minimum width follow the same rules.
                (void)water::SynchronizeWaterFeatureClipBounds(
                    &feature,
                    clipId);
            }
        }
    }

    for (auto& effect : state->colouriseEffects) {
        effect.activationRange = SanitizeTimingColouriseActivationRange(
            effect.activationRange);
        const bool beginsAtAnimationStart =
            effect.activationRange.start == 0.0F;
        const bool extendsThroughAnimationEnd =
            effect.activationRange.end == 1.0F;
        effect.activationRange.start = beginsAtAnimationStart
                                           ? 0.0F
                                           : retimePosition(
                                                 effect.activationRange.start);
        effect.activationRange.end = extendsThroughAnimationEnd
                                         ? 1.0F
                                         : retimePosition(
                                               effect.activationRange.end);
        retimeKeys(&effect.effectParameterKeys);
        retimeKeys(&effect.paletteKeys);
        retimeKeys(&effect.paletteStopParameterKeys);
        retimeKeys(&effect.boundsParameterKeys);
        retimeKeys(&effect.boundsKeys);
        retimeKeys(&effect.emissiveFalloffKeys);
        for (auto& memory : effect.fieldBoundsMemory) {
            retimeKeys(&memory.boundsParameterKeys);
            retimeKeys(&memory.boundsKeys);
        }
    }
    return true;
}

namespace {

template <typename Key, typename SameLane>
void MergeTimingKeysKeepingFirst(
    std::vector<Key>* destination,
    const std::vector<Key>& source,
    SameLane sameLane) {
    if (destination == nullptr) {
        return;
    }
    for (const auto& key : source) {
        const bool occupied = std::any_of(
            destination->begin(),
            destination->end(),
            [&](const Key& existing) {
                return sameLane(existing, key) &&
                       std::abs(existing.position - key.position) <=
                           kTimingColouriseKeyTolerance;
            });
        if (!occupied) {
            destination->push_back(key);
        }
    }
}

invisible_places::water::WaterScenarioInterpolation
ConcreteWaterTrackDefault(
    const invisible_places::water::WaterKeyedSettingTrack& track) {
    return track.defaultInterpolation ==
                   invisible_places::water::
                       WaterScenarioInterpolation::TrackDefault
               ? invisible_places::water::
                     WaterScenarioInterpolation::SmoothVelocity
               : track.defaultInterpolation;
}

void MergeWaterFeatureTimelineKeepingFirst(
    invisible_places::water::WaterFeatureTimeline* destination,
    const invisible_places::water::WaterFeatureTimeline& source) {
    using invisible_places::water::WaterKeyedSettingTrack;
    using invisible_places::water::WaterScenarioInterpolation;
    if (destination == nullptr ||
        destination->feature != source.feature) {
        return;
    }

    std::vector<std::pair<std::uint32_t, std::uint32_t>> clipIdMap;
    clipIdMap.reserve(source.clips.size());
    std::vector<std::uint32_t> appendedClipIds;
    appendedClipIds.reserve(source.clips.size());
    for (const auto& sourceClip : source.clips) {
        auto copied = sourceClip;
        const bool idUnavailable =
            copied.id == 0U ||
            std::any_of(
                destination->clips.begin(),
                destination->clips.end(),
                [&](const auto& existing) {
                    return existing.id == copied.id;
                });
        if (idUnavailable) {
            copied.id =
                invisible_places::water::AllocateWaterFeatureClipId(
                    *destination);
        }
        clipIdMap.emplace_back(sourceClip.id, copied.id);
        appendedClipIds.push_back(copied.id);
        destination->clips.push_back(std::move(copied));
    }
    const auto remapClipId = [&](std::uint32_t sourceId) {
        if (sourceId == 0U) {
            return 0U;
        }
        const auto mapped = std::find_if(
            clipIdMap.begin(),
            clipIdMap.end(),
            [&](const auto& entry) {
                return entry.first == sourceId;
            });
        return mapped != clipIdMap.end() ? mapped->second : 0U;
    };

    for (const auto& sourceSetting : source.settings) {
        auto destinationSetting = std::find_if(
            destination->settings.begin(),
            destination->settings.end(),
            [&](const WaterKeyedSettingTrack& setting) {
                return setting.settingId == sourceSetting.settingId;
            });
        if (destinationSetting == destination->settings.end()) {
            auto copied = sourceSetting;
            for (auto& key : copied.keys) {
                key.clipId = remapClipId(key.clipId);
            }
            destination->settings.push_back(std::move(copied));
            continue;
        }

        destinationSetting->active =
            destinationSetting->active || sourceSetting.active;
        const bool differingDefaults =
            ConcreteWaterTrackDefault(*destinationSetting) !=
            ConcreteWaterTrackDefault(sourceSetting);
        for (auto key : sourceSetting.keys) {
            const bool occupied = std::any_of(
                destinationSetting->keys.begin(),
                destinationSetting->keys.end(),
                [&](const auto& existing) {
                    return std::abs(existing.position - key.position) <=
                           kTimingColouriseKeyTolerance;
                });
            if (occupied) {
                continue;
            }
            key.clipId = remapClipId(key.clipId);
            if (differingDefaults &&
                key.interpolation ==
                    WaterScenarioInterpolation::TrackDefault) {
                key.interpolation =
                    ConcreteWaterTrackDefault(sourceSetting);
            }
            destinationSetting->keys.push_back(std::move(key));
        }
        std::stable_sort(
            destinationSetting->keys.begin(),
            destinationSetting->keys.end(),
            [](const auto& left, const auto& right) {
                return left.position < right.position;
            });
    }
    destination->clipMembershipExplicit =
        destination->clipMembershipExplicit ||
        source.clipMembershipExplicit || !source.clips.empty();
    for (const auto clipId : appendedClipIds) {
        (void)invisible_places::water::SynchronizeWaterFeatureClipBounds(
            destination,
            clipId);
    }
}

std::uint32_t AllocateMergedWaterRunId(
    TimingTakeSceneState* destination,
    std::uint32_t preferred) {
    const auto used = [&](std::uint32_t id) {
        return id == 0U || std::any_of(
            destination->waterFeatureTimingRuns.begin(),
            destination->waterFeatureTimingRuns.end(),
            [&](const auto& run) { return run.id == id; });
    };
    if (!used(preferred)) {
        return preferred;
    }
    std::uint32_t candidate = std::max(
        1U,
        destination->waterFeatureTimingRunSequence);
    while (used(candidate)) {
        ++candidate;
    }
    destination->waterFeatureTimingRunSequence = candidate + 1U;
    return candidate;
}

void MergeWaterFeatureRunMarksKeepingFirst(
    invisible_places::water::WaterFeatureTimingRun* destination,
    std::span<const invisible_places::water::WaterFeatureRunMark> source) {
    if (destination == nullptr) {
        return;
    }
    for (const auto& sourceMark : source) {
        const bool duplicate = std::any_of(
            destination->marks.begin(),
            destination->marks.end(),
            [&](const auto& existing) {
                return existing.text == sourceMark.text &&
                       std::abs(existing.position - sourceMark.position) <=
                           kTimingColouriseKeyTolerance;
            });
        if (duplicate) {
            continue;
        }
        auto copied = sourceMark;
        if (copied.id == 0U ||
            invisible_places::water::FindWaterFeatureRunMark(
                destination,
                copied.id) != nullptr) {
            copied.id = invisible_places::water::
                AllocateWaterFeatureRunMarkId(*destination);
        }
        if (copied.id != 0U) {
            destination->marks.push_back(std::move(copied));
        }
    }
}

void MergeWaterFeatureRunVariantsKeepingFirst(
    invisible_places::water::WaterFeatureTimingRun* destination,
    const invisible_places::water::WaterFeatureTimingRun& source,
    std::optional<invisible_places::water::WaterKeyedFeatureId>
        onlyFeature = std::nullopt) {
    if (destination == nullptr) {
        return;
    }
    destination->nextVariantId = std::max(
        destination->nextVariantId,
        source.nextVariantId);
    for (const auto& sourceVariant : source.variants) {
        auto destinationVariant = std::find_if(
            destination->variants.begin(),
            destination->variants.end(),
            [&](const auto& candidate) {
                return candidate.name == sourceVariant.name;
            });
        if (destinationVariant == destination->variants.end()) {
            invisible_places::water::WaterFeatureRunVariant copied{
                .id = sourceVariant.id,
                .name = sourceVariant.name,
            };
            if (copied.id == 0U ||
                invisible_places::water::FindWaterFeatureRunVariant(
                    destination,
                    copied.id) != nullptr) {
                copied.id = invisible_places::water::
                    AllocateWaterFeatureRunVariantId(destination);
            }
            if (copied.id == 0U) {
                continue;
            }
            destination->variants.push_back(std::move(copied));
            destinationVariant = std::prev(destination->variants.end());
        }
        for (const auto& overrideValue : sourceVariant.overrides) {
            if (onlyFeature.has_value() &&
                overrideValue.feature != onlyFeature.value()) {
                continue;
            }
            if (invisible_places::water::
                    FindWaterFeatureRunVariantOverride(
                        &*destinationVariant,
                        overrideValue.feature,
                        overrideValue.settingId) == nullptr) {
                destinationVariant->overrides.push_back(overrideValue);
            }
        }
    }
}

}  // namespace

void MergeTimingTakeSceneStateKeepingFirst(
    TimingTakeSceneState* destination,
    const TimingTakeSceneState& inputSource) {
    if (destination == nullptr) {
        return;
    }
    *destination = SanitizeTimingTakeSceneState(
        std::move(*destination));
    const auto source = SanitizeTimingTakeSceneState(inputSource);
    destination->onlyShowWaterFeaturesInRuns =
        destination->onlyShowWaterFeaturesInRuns ||
        source.onlyShowWaterFeaturesInRuns;

    // Older linked-loop merges could leave duplicate/zero run ids and the
    // same feature assigned to multiple runs. Repair that historical state
    // before adding the next loop so subsequent run and clip remapping has a
    // deterministic first owner.
    for (std::size_t runIndex = 0U;
         runIndex < destination->waterFeatureTimingRuns.size();
         ++runIndex) {
        auto& run = destination->waterFeatureTimingRuns[runIndex];
        const bool idUnavailable =
            run.id == 0U ||
            std::any_of(
                destination->waterFeatureTimingRuns.begin(),
                destination->waterFeatureTimingRuns.begin() +
                    static_cast<std::ptrdiff_t>(runIndex),
                [&](const auto& earlier) {
                    return earlier.id == run.id;
                });
        if (idUnavailable) {
            run.id = AllocateMergedWaterRunId(destination, 0U);
        }
    }
    for (std::size_t runIndex = 0U;
         runIndex < destination->waterFeatureTimingRuns.size();
         ++runIndex) {
        auto& run = destination->waterFeatureTimingRuns[runIndex];
        const bool originallyHadFeatures = !run.features.empty();
        for (std::size_t featureIndex = 0U;
             featureIndex < run.features.size();) {
            auto* firstOwner =
                static_cast<invisible_places::water::
                                WaterFeatureTimeline*>(nullptr);
            auto* firstOwnerRun =
                static_cast<invisible_places::water::
                                WaterFeatureTimingRun*>(nullptr);
            for (std::size_t earlierRunIndex = 0U;
                 earlierRunIndex <= runIndex && firstOwner == nullptr;
                 ++earlierRunIndex) {
                auto& earlierRun =
                    destination->waterFeatureTimingRuns[earlierRunIndex];
                const std::size_t limit =
                    earlierRunIndex == runIndex
                        ? featureIndex
                        : earlierRun.features.size();
                const auto earlier = std::find_if(
                    earlierRun.features.begin(),
                    earlierRun.features.begin() +
                        static_cast<std::ptrdiff_t>(limit),
                    [&](const auto& candidate) {
                        return candidate.feature ==
                               run.features[featureIndex].feature;
                    });
                if (earlier !=
                    earlierRun.features.begin() +
                        static_cast<std::ptrdiff_t>(limit)) {
                    firstOwner = &*earlier;
                    firstOwnerRun = &earlierRun;
                }
            }
            if (firstOwner == nullptr) {
                ++featureIndex;
                continue;
            }
            MergeWaterFeatureTimelineKeepingFirst(
                firstOwner,
                run.features[featureIndex]);
            MergeWaterFeatureRunMarksKeepingFirst(
                firstOwnerRun,
                run.marks);
            MergeWaterFeatureRunVariantsKeepingFirst(
                firstOwnerRun,
                run,
                run.features[featureIndex].feature);
            firstOwnerRun->enabled =
                firstOwnerRun->enabled || run.enabled;
            run.features.erase(
                run.features.begin() +
                static_cast<std::ptrdiff_t>(featureIndex));
        }
        if (originallyHadFeatures && run.features.empty()) {
            // Every feature that gave these marks meaning was repaired into
            // its first owning run above. Do not leave the same annotations
            // on an empty organizational shell where a future feature could
            // inherit them accidentally.
            run.marks.clear();
        }
    }

    const auto findFeature = [&](const auto& featureId) {
        using Result = std::pair<
            invisible_places::water::WaterFeatureTimingRun*,
            invisible_places::water::WaterFeatureTimeline*>;
        for (auto& run : destination->waterFeatureTimingRuns) {
            const auto feature = std::find_if(
                run.features.begin(),
                run.features.end(),
                [&](const auto& timeline) {
                    return timeline.feature == featureId;
                });
            if (feature != run.features.end()) {
                return Result{&run, &*feature};
            }
        }
        return Result{nullptr, nullptr};
    };
    const auto findRunByName = [&](std::string_view name) {
        const auto run = std::find_if(
            destination->waterFeatureTimingRuns.begin(),
            destination->waterFeatureTimingRuns.end(),
            [&](const auto& candidate) {
                return candidate.name == name;
            });
        return run != destination->waterFeatureTimingRuns.end()
                   ? &*run
                   : nullptr;
    };
    for (const auto& sourceRun : source.waterFeatureTimingRuns) {
        auto* namedDestinationRun = findRunByName(sourceRun.name);
        const auto ensureDestinationRun = [&]() {
            if (namedDestinationRun != nullptr) {
                return namedDestinationRun;
            }
            destination->waterFeatureTimingRuns.push_back({
                .id = AllocateMergedWaterRunId(
                    destination,
                    sourceRun.id),
                .name = sourceRun.name,
                .enabled = sourceRun.enabled,
                .features = {},
            });
            namedDestinationRun =
                &destination->waterFeatureTimingRuns.back();
            return namedDestinationRun;
        };
        // Preserve the source run even when all of its features already have
        // a first owner in another run. In that case the named run remains an
        // empty organizational group instead of duplicating the feature.
        auto* targetRun = ensureDestinationRun();
        targetRun->enabled = targetRun->enabled || sourceRun.enabled;
        MergeWaterFeatureRunVariantsKeepingFirst(
            targetRun,
            sourceRun);
        bool targetReceivedFeature = sourceRun.features.empty();
        for (const auto& sourceFeature : sourceRun.features) {
            auto [owningRun, destinationFeature] =
                findFeature(sourceFeature.feature);
            if (destinationFeature != nullptr) {
                owningRun->enabled = owningRun->enabled || sourceRun.enabled;
                MergeWaterFeatureRunMarksKeepingFirst(
                    owningRun,
                    sourceRun.marks);
                MergeWaterFeatureTimelineKeepingFirst(
                    destinationFeature,
                    sourceFeature);
                if (owningRun != targetRun) {
                    MergeWaterFeatureRunVariantsKeepingFirst(
                        owningRun,
                        sourceRun,
                        sourceFeature.feature);
                }
                continue;
            }
            invisible_places::water::WaterFeatureTimeline merged{
                .feature = sourceFeature.feature};
            MergeWaterFeatureTimelineKeepingFirst(
                &merged,
                sourceFeature);
            targetRun->features.push_back(std::move(merged));
            targetReceivedFeature = true;
        }
        if (targetReceivedFeature) {
            MergeWaterFeatureRunMarksKeepingFirst(
                targetRun,
                sourceRun.marks);
        }
    }

    for (const auto& sourceEffect : source.colouriseEffects) {
        auto destinationEffect = std::find_if(
            destination->colouriseEffects.begin(),
            destination->colouriseEffects.end(),
            [&](const auto& effect) {
                return !sourceEffect.id.empty() &&
                       effect.id == sourceEffect.id;
            });
        if (destinationEffect == destination->colouriseEffects.end()) {
            destination->colouriseEffects.push_back(sourceEffect);
            continue;
        }
        destinationEffect->activationRange.start = std::min(
            destinationEffect->activationRange.start,
            sourceEffect.activationRange.start);
        destinationEffect->activationRange.end = std::max(
            destinationEffect->activationRange.end,
            sourceEffect.activationRange.end);
        MergeTimingKeysKeepingFirst(
            &destinationEffect->effectParameterKeys,
            sourceEffect.effectParameterKeys,
            [](const auto& left, const auto& right) {
                return left.parameter == right.parameter;
            });
        MergeTimingKeysKeepingFirst(
            &destinationEffect->paletteKeys,
            sourceEffect.paletteKeys,
            [](const auto&, const auto&) { return true; });
        MergeTimingKeysKeepingFirst(
            &destinationEffect->paletteStopParameterKeys,
            sourceEffect.paletteStopParameterKeys,
            [](const auto& left, const auto& right) {
                return left.stopId == right.stopId &&
                       left.parameter == right.parameter;
            });
        MergeTimingKeysKeepingFirst(
            &destinationEffect->boundsParameterKeys,
            sourceEffect.boundsParameterKeys,
            [](const auto& left, const auto& right) {
                return left.parameter == right.parameter;
            });
        MergeTimingKeysKeepingFirst(
            &destinationEffect->boundsKeys,
            sourceEffect.boundsKeys,
            [](const auto&, const auto&) { return true; });
        MergeTimingKeysKeepingFirst(
            &destinationEffect->emissiveFalloffKeys,
            sourceEffect.emissiveFalloffKeys,
            [](const auto& left, const auto& right) {
                return left.nodeId == right.nodeId &&
                       left.parameter == right.parameter;
            });
        for (const auto& sourceMemory : sourceEffect.fieldBoundsMemory) {
            auto destinationMemory = std::find_if(
                destinationEffect->fieldBoundsMemory.begin(),
                destinationEffect->fieldBoundsMemory.end(),
                [&](const auto& memory) {
                    return memory.selector == sourceMemory.selector;
                });
            if (destinationMemory ==
                destinationEffect->fieldBoundsMemory.end()) {
                destinationEffect->fieldBoundsMemory.push_back(sourceMemory);
                continue;
            }
            MergeTimingKeysKeepingFirst(
                &destinationMemory->boundsParameterKeys,
                sourceMemory.boundsParameterKeys,
                [](const auto& left, const auto& right) {
                    return left.parameter == right.parameter;
                });
            MergeTimingKeysKeepingFirst(
                &destinationMemory->boundsKeys,
                sourceMemory.boundsKeys,
                [](const auto&, const auto&) { return true; });
        }
    }
    destination->waterFeatureTimingRunSequence = std::max(
        destination->waterFeatureTimingRunSequence,
        source.waterFeatureTimingRunSequence);
    destination->colouriseEffectSequence = std::max(
        destination->colouriseEffectSequence,
        source.colouriseEffectSequence);
    std::uint32_t maximumRunId = 0U;
    for (const auto& run : destination->waterFeatureTimingRuns) {
        maximumRunId = std::max(maximumRunId, run.id);
    }
    destination->waterFeatureTimingRunSequence = std::max(
        destination->waterFeatureTimingRunSequence,
        maximumRunId + 1U);
    *destination = SanitizeTimingTakeSceneState(
        std::move(*destination));
}

void StashTimingColouriseFieldBounds(TimingColouriseEffect* effect) {
    if (effect == nullptr) {
        return;
    }
    auto entry = std::find_if(
        effect->fieldBoundsMemory.begin(),
        effect->fieldBoundsMemory.end(),
        [&](const TimingColouriseFieldBoundsMemory& memory) {
            return memory.selector == effect->field;
        });
    if (entry == effect->fieldBoundsMemory.end()) {
        effect->fieldBoundsMemory.emplace_back();
        entry = std::prev(effect->fieldBoundsMemory.end());
        entry->selector = effect->field;
    }
    entry->bounds = effect->baseBounds;
    entry->boundsKeyMode = effect->boundsKeyMode;
    entry->boundsParameterKeys = effect->boundsParameterKeys;
    entry->boundsKeys = effect->boundsKeys;
    entry->edited = effect->boundsEdited;
    entry->edgeFadesLinked = effect->edgeFadesLinked;
    entry->adoptedGlobalRevision = effect->boundsAdoptedGlobalRevision;
}

void StashTimingColouriseFieldVisuals(TimingColouriseEffect* effect) {
    if (effect == nullptr) {
        return;
    }
    auto entry = std::find_if(
        effect->fieldVisualMemory.begin(),
        effect->fieldVisualMemory.end(),
        [&](const TimingColouriseFieldVisualMemory& memory) {
            return memory.selector == effect->field;
        });
    if (entry == effect->fieldVisualMemory.end()) {
        effect->fieldVisualMemory.emplace_back();
        entry = std::prev(effect->fieldVisualMemory.end());
        entry->selector = effect->field;
    }
    entry->basePalette = effect->basePalette;
    entry->paletteKeyModel = effect->paletteKeyModel;
    entry->paletteSourceKind = effect->paletteSourceKind;
    entry->paletteSourceId = effect->paletteSourceId;
    entry->paletteSourceName = effect->paletteSourceName;
    entry->paletteEdited = effect->paletteEdited;
    entry->paletteLooped = effect->paletteLooped;
    entry->colourKeyInterpolationSpace =
        effect->colourKeyInterpolationSpace;
    entry->colouriseAmountOverrideMode =
        effect->colouriseAmountOverrideMode;
    entry->colouriseAmountOverride = effect->colouriseAmountOverride;
    entry->blendMode = effect->blendMode;
    entry->palettePhaseOffset = effect->palettePhaseOffset;
    entry->paletteSkewCentre = effect->paletteSkewCentre;
    entry->paletteSkewSpread = effect->paletteSkewSpread;
    entry->paletteSkewNodes = effect->paletteSkewNodes;
    entry->emissiveSkewCentre = effect->emissiveSkewCentre;
    entry->emissiveSkewSpread = effect->emissiveSkewSpread;
    entry->emissiveSkewNodes = effect->emissiveSkewNodes;
    entry->emissiveFalloffNodes = effect->emissiveFalloffNodes;
    entry->emissiveFalloffKeys = effect->emissiveFalloffKeys;
    entry->emissiveLevel = effect->emissiveLevel;
    entry->effectParameterKeys = effect->effectParameterKeys;
    entry->paletteKeys = effect->paletteKeys;
    entry->paletteStopParameterKeys = effect->paletteStopParameterKeys;
}

void ApplyTimingColouriseFieldSelection(
    TimingColouriseEffect* effect,
    const TimingColouriseFieldSelector& selector,
    const TimingColouriseBounds& fallbackBounds,
    const TimingScalarBoundsStore* globalStore) {
    if (effect == nullptr || effect->field == selector) {
        return;
    }
    StashTimingColouriseFieldBounds(effect);
    // Visual settings follow the field unless the user chose Global. A
    // field visited for the first time keeps the current settings, so the
    // link only ever restores state this feature actually authored there.
    if (effect->fieldScopedVisualSettings) {
        StashTimingColouriseFieldVisuals(effect);
        const auto visualEntry = std::find_if(
            effect->fieldVisualMemory.begin(),
            effect->fieldVisualMemory.end(),
            [&](const TimingColouriseFieldVisualMemory& memory) {
                return memory.selector == selector;
            });
        if (visualEntry != effect->fieldVisualMemory.end()) {
            effect->basePalette = visualEntry->basePalette;
            effect->paletteKeyModel = visualEntry->paletteKeyModel;
            effect->paletteSourceKind = visualEntry->paletteSourceKind;
            effect->paletteSourceId = visualEntry->paletteSourceId;
            effect->paletteSourceName = visualEntry->paletteSourceName;
            effect->paletteEdited = visualEntry->paletteEdited;
            effect->paletteLooped = visualEntry->paletteLooped;
            effect->colourKeyInterpolationSpace =
                visualEntry->colourKeyInterpolationSpace;
            effect->colouriseAmountOverrideMode =
                visualEntry->colouriseAmountOverrideMode;
            effect->colouriseAmountOverride =
                visualEntry->colouriseAmountOverride;
            effect->blendMode = visualEntry->blendMode;
            effect->palettePhaseOffset =
                visualEntry->palettePhaseOffset;
            effect->paletteSkewCentre = visualEntry->paletteSkewCentre;
            effect->paletteSkewSpread = visualEntry->paletteSkewSpread;
            effect->paletteSkewNodes = visualEntry->paletteSkewNodes;
            effect->emissiveSkewCentre =
                visualEntry->emissiveSkewCentre;
            effect->emissiveSkewSpread =
                visualEntry->emissiveSkewSpread;
            effect->emissiveSkewNodes = visualEntry->emissiveSkewNodes;
            effect->emissiveFalloffNodes =
                visualEntry->emissiveFalloffNodes;
            effect->emissiveFalloffKeys =
                visualEntry->emissiveFalloffKeys;
            effect->emissiveLevel = visualEntry->emissiveLevel;
            effect->effectParameterKeys =
                visualEntry->effectParameterKeys;
            effect->paletteKeys = visualEntry->paletteKeys;
            effect->paletteStopParameterKeys =
                visualEntry->paletteStopParameterKeys;
        }
    }
    effect->field = selector;
    const auto entry = std::find_if(
        effect->fieldBoundsMemory.begin(),
        effect->fieldBoundsMemory.end(),
        [&](const TimingColouriseFieldBoundsMemory& memory) {
            return memory.selector == selector;
        });
    if (entry != effect->fieldBoundsMemory.end()) {
        effect->baseBounds = entry->bounds;
        effect->boundsKeyMode = entry->boundsKeyMode;
        effect->boundsParameterKeys = entry->boundsParameterKeys;
        effect->boundsKeys = entry->boundsKeys;
        effect->boundsEdited = entry->edited;
        effect->edgeFadesLinked = entry->edgeFadesLinked;
        effect->boundsAdoptedGlobalRevision =
            entry->adoptedGlobalRevision;
    } else {
        effect->baseBounds =
            globalStore != nullptr && globalStore->revision > 0U
                ? globalStore->globalBounds
                : fallbackBounds;
        effect->boundsKeyMode = TimingColouriseBoundsKeyMode::LowerUpper;
        effect->boundsParameterKeys.clear();
        effect->boundsKeys.clear();
        effect->boundsEdited = false;
        effect->edgeFadesLinked = true;
        effect->boundsAdoptedGlobalRevision =
            globalStore != nullptr ? globalStore->revision : 0U;
    }
    if (!effect->boundsEdited && globalStore != nullptr &&
        globalStore->revision > effect->boundsAdoptedGlobalRevision) {
        effect->baseBounds = globalStore->globalBounds;
        effect->boundsAdoptedGlobalRevision = globalStore->revision;
    }
    *effect = SanitizeTimingColouriseEffect(std::move(*effect));
}

TimingScalarBoundsStore* FindTimingScalarBoundsStore(
    std::vector<TimingScalarBoundsStore>* stores,
    const TimingColouriseFieldSelector& selector) {
    if (stores == nullptr) {
        return nullptr;
    }
    const auto store = std::find_if(
        stores->begin(),
        stores->end(),
        [&](const TimingScalarBoundsStore& candidate) {
            return candidate.selector == selector;
        });
    return store != stores->end() ? &*store : nullptr;
}

const TimingScalarBoundsStore* FindTimingScalarBoundsStore(
    const std::vector<TimingScalarBoundsStore>& stores,
    const TimingColouriseFieldSelector& selector) {
    const auto store = std::find_if(
        stores.begin(),
        stores.end(),
        [&](const TimingScalarBoundsStore& candidate) {
            return candidate.selector == selector;
        });
    return store != stores.end() ? &*store : nullptr;
}

void RecordTimingScalarBoundsEdit(
    std::vector<TimingScalarBoundsStore>* stores,
    TimingColouriseEffect* effect) {
    if (effect == nullptr) {
        return;
    }
    effect->boundsEdited = true;
    if (stores == nullptr) {
        return;
    }
    auto* store = FindTimingScalarBoundsStore(stores, effect->field);
    if (store == nullptr) {
        stores->emplace_back();
        store = &stores->back();
        store->selector = effect->field;
    }
    store->globalBounds = SanitizeTimingColouriseBounds(effect->baseBounds);
    ++store->revision;
    effect->boundsAdoptedGlobalRevision = store->revision;
}

bool RefreshTimingColouriseBoundsFromGlobal(
    TimingColouriseEffect* effect,
    const std::vector<TimingScalarBoundsStore>& stores) {
    if (effect == nullptr || effect->boundsEdited) {
        return false;
    }
    const auto* store = FindTimingScalarBoundsStore(stores, effect->field);
    if (store == nullptr ||
        store->revision <= effect->boundsAdoptedGlobalRevision) {
        return false;
    }
    effect->baseBounds = store->globalBounds;
    effect->boundsAdoptedGlobalRevision = store->revision;
    return true;
}

std::size_t MergeLegacyTimingEffectAspects(
    std::vector<TimingColouriseEffect>* effects,
    const std::vector<bool>* mergeEligible,
    std::size_t rendererSlotCapacity) {
    if (effects == nullptr || effects->size() < 2U) {
        return 0U;
    }
    // Local copy kept in lockstep with erasures so eligibility stays
    // aligned with shifting indices.
    std::vector<bool> eligibility;
    if (mergeEligible != nullptr) {
        eligibility = *mergeEligible;
        eligibility.resize(effects->size(), false);
    }
    const auto eligible = [&](std::size_t index) {
        return eligibility.empty() || eligibility[index];
    };
    const auto rangesMatch = [](const TimingColouriseActivationRange& a,
                                const TimingColouriseActivationRange& b) {
        // Exact equality: authored duplicates serialize identically, and a
        // merged object with one window cannot reproduce even a sliver of
        // activation difference.
        const auto left = SanitizeTimingColouriseActivationRange(a);
        const auto right = SanitizeTimingColouriseActivationRange(b);
        return left.start == right.start && left.end == right.end;
    };
    const auto boundsParameterKeysMatch =
        [](const std::vector<TimingColouriseBoundsParameterKey>& a,
           const std::vector<TimingColouriseBoundsParameterKey>& b) {
            return a.size() == b.size() &&
                   std::equal(
                       a.begin(),
                       a.end(),
                       b.begin(),
                       [](const TimingColouriseBoundsParameterKey& x,
                          const TimingColouriseBoundsParameterKey& y) {
                           return x.parameter == y.parameter &&
                                  x.position == y.position &&
                                  x.value == y.value &&
                                  x.interpolation == y.interpolation;
                       });
        };
    const auto boundsKeysMatch =
        [](const std::vector<TimingColouriseBoundsKey>& a,
           const std::vector<TimingColouriseBoundsKey>& b) {
            return a.size() == b.size() &&
                   std::equal(
                       a.begin(),
                       a.end(),
                       b.begin(),
                       [](const TimingColouriseBoundsKey& x,
                          const TimingColouriseBoundsKey& y) {
                           return x.position == y.position &&
                                  x.bounds.lower == y.bounds.lower &&
                                  x.bounds.upper == y.bounds.upper &&
                                  x.bounds.edgeFadeLower ==
                                      y.bounds.edgeFadeLower &&
                                  x.bounds.edgeFadeUpper ==
                                      y.bounds.edgeFadeUpper &&
                                  x.interpolation == y.interpolation;
                       });
        };
    // Bounds gate and select the emissive contribution exactly as they do
    // colourise, so a pair only evaluates identically when its complete
    // bounds authoring matches; a single merged object carries one bounds.
    const auto boundsAuthoringMatches =
        [&](const TimingColouriseEffect& a,
            const TimingColouriseEffect& b) {
            return a.baseBounds.lower == b.baseBounds.lower &&
                   a.baseBounds.upper == b.baseBounds.upper &&
                   a.baseBounds.edgeFadeLower ==
                       b.baseBounds.edgeFadeLower &&
                   a.baseBounds.edgeFadeUpper ==
                       b.baseBounds.edgeFadeUpper &&
                   a.boundsKeyMode == b.boundsKeyMode &&
                   boundsParameterKeysMatch(
                       a.boundsParameterKeys,
                       b.boundsParameterKeys) &&
                   boundsKeysMatch(a.boundsKeys, b.boundsKeys);
        };
    // With more slots concurrently active than the renderer retains, the
    // topmost-first cap selection depends on list positions, and merging
    // moves the emissive slot to its partner's position — only merge where
    // the cap provably never engages inside the pair's window.
    const auto concurrencyWithinCapacity =
        [&](const TimingColouriseActivationRange& window) {
            std::vector<float> samples{window.start, window.end};
            for (const auto& effect : *effects) {
                const auto range = SanitizeTimingColouriseActivationRange(
                    effect.activationRange);
                for (const float boundary : {range.start, range.end}) {
                    if (boundary >= window.start &&
                        boundary <= window.end) {
                        samples.push_back(boundary);
                    }
                }
            }
            for (const float sample : samples) {
                std::size_t slots = 0U;
                for (const auto& effect : *effects) {
                    if (TimingColouriseEffectIsActiveAt(effect, sample)) {
                        slots +=
                            (effect.colouriseEnabled ? 1U : 0U) +
                            (effect.emissiveEnabled ? 1U : 0U);
                    }
                }
                if (slots > rendererSlotCapacity) {
                    return false;
                }
            }
            return true;
        };
    std::size_t merged = 0U;
    for (std::size_t index = 0U; index < effects->size(); ++index) {
        if (!eligible(index) ||
            !(*effects)[index].enabled ||
            !(*effects)[index].colouriseEnabled ||
            (*effects)[index].emissiveEnabled) {
            continue;
        }
        std::size_t partnerIndex = effects->size();
        for (std::size_t candidate = 0U; candidate < effects->size();
             ++candidate) {
            if (candidate == index || !eligible(candidate)) {
                continue;
            }
            const auto& other = (*effects)[candidate];
            if (other.emissiveEnabled && !other.colouriseEnabled &&
                other.enabled && (*effects)[index].enabled &&
                other.field == (*effects)[index].field &&
                rangesMatch(
                    other.activationRange,
                    (*effects)[index].activationRange) &&
                boundsAuthoringMatches(other, (*effects)[index])) {
                partnerIndex = candidate;
                break;
            }
        }
        if (partnerIndex >= effects->size() ||
            !concurrencyWithinCapacity(
                SanitizeTimingColouriseActivationRange(
                    (*effects)[index].activationRange))) {
            continue;
        }
        // The colourise object absorbs the emissive aspect. The partner's
        // dormant colourise-side data (palette, phase/amount keys) never
        // contributed to rendering and is dropped with it; its bounds are
        // identical to the target's by the gate above. The target's own
        // emissive-level keys were provably dormant while it was
        // colourise-only, so the partner's authored track replaces them.
        auto& target = (*effects)[index];
        const auto& partner = (*effects)[partnerIndex];
        target.emissiveEnabled = true;
        target.emissiveLevel = partner.emissiveLevel;
        std::erase_if(
            target.effectParameterKeys,
            [](const TimingColouriseEffectParameterKey& key) {
                return key.parameter ==
                       TimingColouriseEffectParameter::EmissiveLevel;
            });
        for (const auto& key : partner.effectParameterKeys) {
            if (key.parameter ==
                TimingColouriseEffectParameter::EmissiveLevel) {
                target.effectParameterKeys.push_back(key);
            }
        }
        target = SanitizeTimingColouriseEffect(std::move(target));
        effects->erase(
            effects->begin() +
            static_cast<std::ptrdiff_t>(partnerIndex));
        if (!eligibility.empty()) {
            eligibility.erase(
                eligibility.begin() +
                static_cast<std::ptrdiff_t>(partnerIndex));
        }
        if (partnerIndex < index) {
            --index;
        }
        ++merged;
    }
    return merged;
}

bool MoveTimingColouriseEffect(
    std::vector<TimingColouriseEffect>* effects,
    std::size_t fromIndex,
    std::size_t toIndex) {
    if (effects == nullptr ||
        fromIndex >= effects->size() ||
        toIndex >= effects->size() ||
        fromIndex == toIndex) {
        return false;
    }

    auto effect = std::move((*effects)[fromIndex]);
    effects->erase(
        effects->begin() + static_cast<std::ptrdiff_t>(fromIndex));
    effects->insert(
        effects->begin() + static_cast<std::ptrdiff_t>(toIndex),
        std::move(effect));
    return true;
}

TimingColouriseLut CompileTimingColourisePaletteLut(
    const TimingColourisePalette& palette) {
    const auto sanitized = SanitizeTimingColourisePalette(palette);
    TimingColouriseLut lut{};
    for (std::size_t index = 0U; index < lut.size(); ++index) {
        const float position =
            static_cast<float>(index) /
            static_cast<float>(lut.size() - 1U);
        lut[index] = SampleSanitizedPalette(sanitized, position);
    }
    return lut;
}

TimingColouriseLut ApplyTimingColouriseAmountOverride(
    TimingColouriseLut lut,
    TimingColouriseAmountOverrideMode mode,
    float value) {
    if (!IsValidAmountOverrideMode(mode)) {
        mode = TimingColouriseAmountOverrideMode::Maximum;
    }
    value = std::clamp(FiniteOr(value, 1.0F), 0.0F, 1.0F);
    for (auto& sample : lut) {
        const float amount = Clamp01(sample[3]);
        sample[3] =
            mode == TimingColouriseAmountOverrideMode::Scale
                ? amount * value
                : std::min(amount, value);
    }
    return lut;
}

TimingColouriseLut ApplyTimingColourisePalettePhase(
    const TimingColouriseLut& lut,
    float phaseOffset) {
    phaseOffset = FiniteOr(phaseOffset, 0.0F);
    phaseOffset -= std::floor(phaseOffset);
    if (phaseOffset <= std::numeric_limits<float>::epsilon() ||
        phaseOffset >=
            1.0F - std::numeric_limits<float>::epsilon()) {
        return lut;
    }
    TimingColouriseLut shifted{};
    for (std::size_t index = 0U; index < shifted.size(); ++index) {
        const float destination =
            static_cast<float>(index) /
            static_cast<float>(shifted.size() - 1U);
        float source = destination - phaseOffset;
        source -= std::floor(source);
        const auto sample = SampleTimingColouriseLut(lut, source);
        shifted[index] = {
            sample.colour[0],
            sample.colour[1],
            sample.colour[2],
            sample.colouriseAmount,
        };
    }
    return shifted;
}

namespace {

// The forward warp curve: knots in palette (or falloff-curve) space, their
// bounds-fraction targets, and monotone-limited Hermite tangents scaled by
// each node's 4^spread density factor.
struct TimingColouriseWarpCurve {
    std::vector<float> palette;
    std::vector<float> field;
    std::vector<float> tangent;
};

TimingColouriseWarpCurve BuildTimingColouriseWarpCurve(
    std::span<const TimingColouriseWarpPoint> points) {
    TimingColouriseWarpCurve curve;
    const std::size_t knotCount = points.size() + 2U;
    curve.palette.reserve(knotCount);
    curve.field.reserve(knotCount);
    curve.tangent.assign(knotCount, 0.0F);
    curve.palette.push_back(0.0F);
    curve.field.push_back(0.0F);
    std::vector<float> spreads;
    spreads.assign(knotCount, 0.0F);
    for (std::size_t index = 0U; index < points.size(); ++index) {
        curve.palette.push_back(points[index].palettePosition);
        curve.field.push_back(points[index].fieldPosition);
        spreads[index + 1U] = points[index].spread;
    }
    curve.palette.push_back(1.0F);
    curve.field.push_back(1.0F);

    const std::size_t segmentCount = knotCount - 1U;
    std::vector<float> secant(segmentCount, 1.0F);
    for (std::size_t index = 0U; index < segmentCount; ++index) {
        const float paletteSpan = std::max(
            curve.palette[index + 1U] - curve.palette[index],
            1.0e-4F);
        secant[index] = std::max(
            curve.field[index + 1U] - curve.field[index],
            0.0F) / paletteSpan;
    }
    for (std::size_t index = 0U; index < knotCount; ++index) {
        const float left = index > 0U ? secant[index - 1U] : secant[0];
        const float right =
            index + 1U < knotCount ? secant[index] : secant.back();
        float tangent = index == 0U
            ? right
            : index + 1U == knotCount
                ? left
                : 0.5F * (left + right);
        // Local pinch/spread: 4^spread keeps 0 exactly neutral and gives a
        // symmetric quarter-to-four-times density range.
        tangent *= std::pow(4.0F, ClampPaletteSkew(spreads[index]));
        // Fritsch-Carlson limiting keeps the warp monotone whatever the
        // spread asked for.
        const float limit = 3.0F * std::min(left, right);
        curve.tangent[index] = std::clamp(tangent, 0.0F, limit);
    }
    return curve;
}

float EvaluateTimingColouriseWarpCurve(
    const TimingColouriseWarpCurve& curve,
    float palettePosition) {
    const float p = Clamp01(palettePosition);
    std::size_t segment = 0U;
    while (segment + 2U < curve.palette.size() &&
           p > curve.palette[segment + 1U]) {
        ++segment;
    }
    const float p0 = curve.palette[segment];
    const float p1 = curve.palette[segment + 1U];
    const float span = std::max(p1 - p0, 1.0e-6F);
    const float t = Clamp01((p - p0) / span);
    const float t2 = t * t;
    const float t3 = t2 * t;
    const float x0 = curve.field[segment];
    const float x1 = curve.field[segment + 1U];
    const float m0 = curve.tangent[segment];
    const float m1 = curve.tangent[segment + 1U];
    const float value =
        (2.0F * t3 - 3.0F * t2 + 1.0F) * x0 +
        (t3 - 2.0F * t2 + t) * span * m0 +
        (-2.0F * t3 + 3.0F * t2) * x1 +
        (t3 - t2) * span * m1;
    // Monotone limiting keeps the segment inside its endpoints; the clamp
    // only absorbs floating-point residue.
    return std::clamp(value, std::min(x0, x1), std::max(x0, x1));
}

}  // namespace

std::vector<TimingColouriseWarpPoint> BuildTimingColouriseWarpPoints(
    float centreFieldPosition,
    float centreSpread,
    std::span<const TimingColourisePaletteSkewNode> extraNodes) {
    std::vector<TimingColouriseWarpPoint> points;
    points.reserve(extraNodes.size() + 1U);
    points.push_back(TimingColouriseWarpPoint{
        .palettePosition = 0.5F,
        .fieldPosition =
            std::clamp(FiniteOr(centreFieldPosition, 0.5F), 0.0F, 1.0F),
        .spread = ClampPaletteSkew(centreSpread),
    });
    for (const auto& node : extraNodes) {
        points.push_back(TimingColouriseWarpPoint{
            .palettePosition = std::clamp(
                FiniteOr(node.palettePosition, 0.5F),
                0.02F,
                0.98F),
            .fieldPosition = Clamp01(node.fieldPosition),
            .spread = ClampPaletteSkew(node.spread),
        });
    }
    std::stable_sort(
        points.begin(),
        points.end(),
        [](const TimingColouriseWarpPoint& left,
           const TimingColouriseWarpPoint& right) {
            return left.palettePosition < right.palettePosition;
        });
    // Coincident anchors keep their first (centre-first) representative,
    // and the bounds targets are clamped strictly increasing so the map
    // can always be inverted.
    constexpr float kMargin = 1.0e-3F;
    std::vector<TimingColouriseWarpPoint> sanitized;
    sanitized.reserve(points.size());
    for (auto& point : points) {
        if (!sanitized.empty() &&
            point.palettePosition - sanitized.back().palettePosition <=
                2.0F * kMargin) {
            continue;
        }
        sanitized.push_back(point);
    }
    float previousField = 0.0F;
    for (std::size_t index = 0U; index < sanitized.size(); ++index) {
        const float remaining =
            static_cast<float>(sanitized.size() - index);
        sanitized[index].fieldPosition = std::clamp(
            sanitized[index].fieldPosition,
            previousField + kMargin,
            1.0F - kMargin * remaining);
        previousField = sanitized[index].fieldPosition;
    }
    return sanitized;
}

float EvaluateTimingColouriseWarpFieldPosition(
    std::span<const TimingColouriseWarpPoint> points,
    float palettePosition) {
    return EvaluateTimingColouriseWarpCurve(
        BuildTimingColouriseWarpCurve(points),
        palettePosition);
}

float EvaluateTimingColouriseWarpPaletteCoordinate(
    std::span<const TimingColouriseWarpPoint> points,
    float fieldFraction) {
    const float target = Clamp01(fieldFraction);
    const auto curve = BuildTimingColouriseWarpCurve(points);
    float low = 0.0F;
    float high = 1.0F;
    for (int iteration = 0; iteration < 28; ++iteration) {
        const float middle = std::midpoint(low, high);
        if (EvaluateTimingColouriseWarpCurve(curve, middle) < target) {
            low = middle;
        } else {
            high = middle;
        }
    }
    return std::midpoint(low, high);
}

bool TimingColouriseWarpIsIdentity(
    std::span<const TimingColouriseWarpPoint> points) {
    constexpr float kEpsilon = 1.0e-5F;
    return std::all_of(
        points.begin(),
        points.end(),
        [](const TimingColouriseWarpPoint& point) {
            return std::abs(
                       point.fieldPosition - point.palettePosition) <=
                       kEpsilon &&
                   std::abs(point.spread) <= kEpsilon;
        });
}

std::string AllocateTimingColourisePaletteSkewNodeId(
    std::span<const TimingColourisePaletteSkewNode> nodes) {
    for (std::size_t candidate = 1U;; ++candidate) {
        std::string id = "skew-node-" + std::to_string(candidate);
        if (std::none_of(
                nodes.begin(),
                nodes.end(),
                [&](const TimingColourisePaletteSkewNode& node) {
                    return node.id == id;
                })) {
            return id;
        }
    }
}

TimingColouriseLut ApplyTimingColourisePaletteSkew(
    const TimingColouriseLut& lut,
    std::span<const TimingColouriseWarpPoint> warp) {
    if (TimingColouriseWarpIsIdentity(warp)) {
        return lut;
    }
    const auto curve = BuildTimingColouriseWarpCurve(warp);
    TimingColouriseLut skewed{};
    for (std::size_t index = 0U; index < skewed.size(); ++index) {
        const float destination =
            static_cast<float>(index) /
            static_cast<float>(skewed.size() - 1U);
        // Invert the forward warp per sample: which palette coordinate
        // lands on this bounds fraction.
        float low = 0.0F;
        float high = 1.0F;
        for (int iteration = 0; iteration < 28; ++iteration) {
            const float middle = std::midpoint(low, high);
            if (EvaluateTimingColouriseWarpCurve(curve, middle) <
                destination) {
                low = middle;
            } else {
                high = middle;
            }
        }
        const auto sample = SampleTimingColouriseLut(
            lut,
            std::midpoint(low, high));
        skewed[index] = {
            sample.colour[0],
            sample.colour[1],
            sample.colour[2],
            sample.colouriseAmount,
        };
    }
    return skewed;
}

TimingColouriseLut ApplyTimingColourisePaletteLoop(
    const TimingColouriseLut& lut) {
    TimingColouriseLut mirrored{};
    for (std::size_t index = 0U; index < mirrored.size(); ++index) {
        const float destination =
            static_cast<float>(index) /
            static_cast<float>(mirrored.size() - 1U);
        const float source = std::abs(2.0F * destination - 1.0F);
        const auto sample = SampleTimingColouriseLut(lut, source);
        mirrored[index] = {
            sample.colour[0],
            sample.colour[1],
            sample.colour[2],
            sample.colouriseAmount,
        };
    }
    return mirrored;
}

namespace {

template <typename Key, typename SameTrack, typename TrackLess>
void ExpandTimingKeysForCyclicEvaluation(
    std::vector<Key>* keys,
    SameTrack sameTrack,
    TrackLess trackLess) {
    if (keys == nullptr || keys->empty()) {
        return;
    }
    for (auto& key : *keys) {
        key.position = WrapTimingColouriseLoopPosition(key.position);
    }
    std::stable_sort(
        keys->begin(),
        keys->end(),
        [&](const Key& left, const Key& right) {
            if (trackLess(left, right)) {
                return true;
            }
            if (trackLess(right, left)) {
                return false;
            }
            return left.position < right.position;
        });
    std::vector<Key> canonical;
    canonical.reserve(keys->size());
    for (auto& key : *keys) {
        if (!canonical.empty() &&
            sameTrack(canonical.back(), key) &&
            TimingColouriseKeyPositionsCoincideCyclically(
                canonical.back().position,
                key.position)) {
            canonical.back() = std::move(key);
        } else {
            canonical.push_back(std::move(key));
        }
    }
    keys->clear();
    keys->reserve(canonical.size() * 3U);
    for (const float shift : {-1.0F, 0.0F, 1.0F}) {
        for (const auto& key : canonical) {
            auto copy = key;
            copy.position += shift;
            keys->push_back(std::move(copy));
        }
    }
    std::stable_sort(
        keys->begin(),
        keys->end(),
        [&](const Key& left, const Key& right) {
            if (trackLess(left, right)) {
                return true;
            }
            if (trackLess(right, left)) {
                return false;
            }
            return left.position < right.position;
        });
}

TimingColouriseEffect PrepareTimingColouriseEffectForEvaluation(
    const TimingColouriseEffect& effect,
    bool cyclic) {
    auto prepared = SanitizeTimingColouriseEffect(effect);
    if (!cyclic) {
        return prepared;
    }

    // Palette Phase storage is delta-based. Convert one canonical cycle to
    // absolute targets before making virtual neighbours, otherwise tripling
    // the keys would incorrectly accumulate another phase turn per copy.
    float accumulatedPhase = prepared.palettePhaseOffset;
    for (auto& key : prepared.effectParameterKeys) {
        if (key.parameter == TimingColouriseEffectParameter::PalettePhase) {
            accumulatedPhase += ClampPalettePhaseDelta(key.value);
            key.value = accumulatedPhase;
        }
    }
    ExpandTimingKeysForCyclicEvaluation(
        &prepared.effectParameterKeys,
        [](const auto& left, const auto& right) {
            return left.parameter == right.parameter;
        },
        [](const auto& left, const auto& right) {
            return static_cast<std::uint8_t>(left.parameter) <
                   static_cast<std::uint8_t>(right.parameter);
        });
    ExpandTimingKeysForCyclicEvaluation(
        &prepared.paletteKeys,
        [](const auto&, const auto&) { return true; },
        [](const auto&, const auto&) { return false; });
    ExpandTimingKeysForCyclicEvaluation(
        &prepared.paletteStopParameterKeys,
        [](const auto& left, const auto& right) {
            return left.stopId == right.stopId &&
                   left.parameter == right.parameter;
        },
        [](const auto& left, const auto& right) {
            return std::tie(left.stopId, left.parameter) <
                   std::tie(right.stopId, right.parameter);
        });
    ExpandTimingKeysForCyclicEvaluation(
        &prepared.emissiveFalloffKeys,
        [](const auto& left, const auto& right) {
            return left.nodeId == right.nodeId &&
                   left.parameter == right.parameter;
        },
        [](const auto& left, const auto& right) {
            return std::tie(left.nodeId, left.parameter) <
                   std::tie(right.nodeId, right.parameter);
        });
    ExpandTimingKeysForCyclicEvaluation(
        &prepared.boundsParameterKeys,
        [](const auto& left, const auto& right) {
            return left.parameter == right.parameter;
        },
        [](const auto& left, const auto& right) {
            return static_cast<std::uint8_t>(left.parameter) <
                   static_cast<std::uint8_t>(right.parameter);
        });
    ExpandTimingKeysForCyclicEvaluation(
        &prepared.boundsKeys,
        [](const auto&, const auto&) { return true; },
        [](const auto&, const auto&) { return false; });
    return prepared;
}

TimingColourisePalette EvaluatePreparedTimingColourisePalette(
    const TimingColouriseEffect& prepared,
    float normalizedPosition) {
    if (prepared.paletteKeyModel ==
        TimingColourisePaletteKeyModel::LegacySnapshots) {
        return prepared.basePalette;
    }
    auto palette = prepared.basePalette;
    for (auto& stop : palette.stops) {
        stop.position = EvaluatePaletteStopScalarTrack(
                            prepared.paletteStopParameterKeys,
                            stop.id,
                            TimingColourisePaletteStopParameter::Position,
                            normalizedPosition)
                            .value_or(stop.position);
        stop.colour = EvaluatePaletteStopColourTrack(
                          prepared.paletteStopParameterKeys,
                          stop.id,
                          normalizedPosition,
                          prepared.colourKeyInterpolationSpace)
                          .value_or(stop.colour);
        stop.colouriseAmount = EvaluatePaletteStopScalarTrack(
                                  prepared.paletteStopParameterKeys,
                                  stop.id,
                                  TimingColourisePaletteStopParameter::
                                      ColouriseAmount,
                                  normalizedPosition)
                                  .value_or(stop.colouriseAmount);
    }
    return SanitizeTimingColourisePalette(std::move(palette));
}

std::vector<TimingColouriseEvaluatedFalloffNode>
EvaluatePreparedEmissiveFalloffNodes(
    const TimingColouriseEffect& prepared,
    float normalizedPosition) {
    std::vector<TimingColouriseEvaluatedFalloffNode> nodes;
    nodes.reserve(prepared.emissiveFalloffNodes.size());
    for (const auto& node : prepared.emissiveFalloffNodes) {
        const auto trackValue =
            [&](TimingColouriseEmissiveFalloffParameter parameter,
                float fallback) {
                const auto matches =
                    [&](const TimingColouriseEmissiveFalloffKey& key) {
                        return key.nodeId == node.id &&
                               key.parameter == parameter;
                    };
                const auto begin = std::find_if(
                    prepared.emissiveFalloffKeys.begin(),
                    prepared.emissiveFalloffKeys.end(),
                    matches);
                auto end = begin;
                while (end != prepared.emissiveFalloffKeys.end() &&
                       matches(*end)) {
                    ++end;
                }
                if (begin == end) {
                    return fallback;
                }
                return Clamp01(
                    EvaluateScalarKeyTrack(
                        begin,
                        end,
                        normalizedPosition,
                        [](const TimingColouriseEmissiveFalloffKey&
                               key) { return key.value; })
                        .value_or(fallback));
            };
        nodes.push_back(TimingColouriseEvaluatedFalloffNode{
            .id = node.id,
            .position = trackValue(
                TimingColouriseEmissiveFalloffParameter::Position,
                node.position),
            .level = trackValue(
                TimingColouriseEmissiveFalloffParameter::Level,
                node.level),
        });
    }
    std::stable_sort(
        nodes.begin(),
        nodes.end(),
        [](const TimingColouriseEvaluatedFalloffNode& left,
           const TimingColouriseEvaluatedFalloffNode& right) {
            return left.position < right.position;
        });
    return nodes;
}

}  // namespace

float EvaluateTimingColouriseEffectParameter(
    const TimingColouriseEffect& effect,
    TimingColouriseEffectParameter parameter,
    float normalizedPosition,
    bool cyclic) {
    const auto sanitized = PrepareTimingColouriseEffectForEvaluation(
        effect,
        cyclic);
    if (!TimingEffectParameterIsSupported(
            sanitized,
            parameter)) {
        return 0.0F;
    }
    normalizedPosition = cyclic
        ? WrapTimingColouriseLoopPosition(normalizedPosition)
        : Clamp01(normalizedPosition);
    return EvaluateEffectParameterTrack(
               sanitized.effectParameterKeys,
               parameter,
               normalizedPosition,
               sanitized.palettePhaseOffset,
               cyclic)
        .value_or(EffectParameterBaseValue(sanitized, parameter));
}

float TimingColourisePalettePhaseDeltaFromPrevious(
    const TimingColouriseEffect& effect,
    float normalizedPosition) {
    const auto sanitized = SanitizeTimingColouriseEffect(effect);
    if (!sanitized.colouriseEnabled) {
        return 0.0F;
    }
    normalizedPosition = Clamp01(normalizedPosition);
    float previousPhase = sanitized.palettePhaseOffset;
    for (const auto& key : sanitized.effectParameterKeys) {
        if (key.parameter !=
            TimingColouriseEffectParameter::PalettePhase) {
            continue;
        }
        if (std::abs(key.position - normalizedPosition) <=
            kTimingColouriseKeyTolerance) {
            return ClampPalettePhaseDelta(key.value);
        }
        if (key.position < normalizedPosition) {
            previousPhase += ClampPalettePhaseDelta(key.value);
            continue;
        }
        break;
    }
    const float evaluated = EvaluateEffectParameterTrack(
                                sanitized.effectParameterKeys,
                                TimingColouriseEffectParameter::PalettePhase,
                                normalizedPosition,
                                sanitized.palettePhaseOffset)
                                .value_or(sanitized.palettePhaseOffset);
    return ClampPalettePhaseDelta(evaluated - previousPhase);
}

float EvaluateTimingEmissiveLevel(
    const TimingColouriseEffect& effect,
    float normalizedPosition,
    bool cyclic) {
    return EvaluateTimingColouriseEffectParameter(
        effect,
        TimingColouriseEffectParameter::EmissiveLevel,
        normalizedPosition,
        cyclic);
}

std::vector<TimingColouriseEvaluatedFalloffNode>
EvaluateTimingColouriseEmissiveFalloffNodes(
    const TimingColouriseEffect& effect,
    float normalizedPosition,
    bool cyclic) {
    const auto prepared = PrepareTimingColouriseEffectForEvaluation(
        effect,
        cyclic);
    normalizedPosition = cyclic
        ? WrapTimingColouriseLoopPosition(normalizedPosition)
        : Clamp01(normalizedPosition);
    return EvaluatePreparedEmissiveFalloffNodes(
        prepared,
        normalizedPosition);
}

float EvaluateTimingColouriseEmissiveFalloffMultiplier(
    std::span<const TimingColouriseEvaluatedFalloffNode> nodes,
    float boundsFraction) {
    if (nodes.empty()) {
        // The flat historical response: emission applies evenly.
        return 1.0F;
    }
    struct CurveKey {
        float position = 0.0F;
        float value = 1.0F;
        invisible_places::water::WaterScenarioInterpolation
            interpolation = invisible_places::water::
                WaterScenarioInterpolation::SmoothVelocity;
    };
    std::vector<CurveKey> curve;
    curve.reserve(nodes.size());
    for (const auto& node : nodes) {
        curve.push_back(CurveKey{
            .position = Clamp01(node.position),
            .value = Clamp01(node.level),
        });
    }
    // The profile is a Monotone Spline through the nodes, held flat past
    // the outermost ones (the shared evaluator's endpoint rule).
    return Clamp01(
        EvaluateScalarKeyTrack(
            curve.begin(),
            curve.end(),
            Clamp01(boundsFraction),
            [](const CurveKey& key) { return key.value; })
            .value_or(1.0F));
}

std::array<float, kTimingColouriseLutSampleCount>
EvaluateTimingEmissiveFalloffProfile(
    const TimingColouriseEffect& effect,
    float normalizedPosition,
    bool cyclic) {
    const auto prepared = PrepareTimingColouriseEffectForEvaluation(
        effect,
        cyclic);
    normalizedPosition = cyclic
        ? WrapTimingColouriseLoopPosition(normalizedPosition)
        : Clamp01(normalizedPosition);
    const auto parameterValue =
        [&](TimingColouriseEffectParameter parameter) {
            return EvaluateEffectParameterTrack(
                       prepared.effectParameterKeys,
                       parameter,
                       normalizedPosition,
                       prepared.palettePhaseOffset,
                       cyclic)
                .value_or(
                    EffectParameterBaseValue(prepared, parameter));
        };
    const float level = parameterValue(
        TimingColouriseEffectParameter::EmissiveLevel);
    std::array<float, kTimingColouriseLutSampleCount> profile{};
    const auto nodes = EvaluatePreparedEmissiveFalloffNodes(
        prepared,
        normalizedPosition);
    const auto warp = BuildTimingColouriseWarpPoints(
        parameterValue(
            TimingColouriseEffectParameter::EmissiveSkewCentre),
        parameterValue(
            TimingColouriseEffectParameter::EmissiveSkewSpread),
        prepared.emissiveSkewNodes);
    const bool identityWarp = TimingColouriseWarpIsIdentity(warp);
    if (nodes.empty()) {
        profile.fill(level);
        return profile;
    }
    for (std::size_t index = 0U; index < profile.size(); ++index) {
        const float fraction =
            static_cast<float>(index) /
            static_cast<float>(profile.size() - 1U);
        const float curvePosition = identityWarp
            ? fraction
            : EvaluateTimingColouriseWarpPaletteCoordinate(
                  warp,
                  fraction);
        profile[index] =
            level * EvaluateTimingColouriseEmissiveFalloffMultiplier(
                        nodes,
                        curvePosition);
    }
    return profile;
}

std::string AllocateTimingColouriseEmissiveFalloffNodeId(
    std::span<const TimingColouriseEmissiveFalloffNode> nodes) {
    for (std::size_t candidate = 1U;; ++candidate) {
        std::string id = "falloff-node-" + std::to_string(candidate);
        if (std::none_of(
                nodes.begin(),
                nodes.end(),
                [&](const TimingColouriseEmissiveFalloffNode& node) {
                    return node.id == id;
                })) {
            return id;
        }
    }
}

bool AddOrUpdateTimingColouriseEmissiveFalloffKey(
    TimingColouriseEffect* effect,
    std::string_view nodeId,
    TimingColouriseEmissiveFalloffParameter parameter,
    float position,
    float value,
    invisible_places::water::WaterScenarioInterpolation interpolation) {
    if (effect == nullptr || nodeId.empty() ||
        !std::isfinite(position) || !std::isfinite(value) ||
        std::none_of(
            effect->emissiveFalloffNodes.begin(),
            effect->emissiveFalloffNodes.end(),
            [&](const TimingColouriseEmissiveFalloffNode& node) {
                return node.id == nodeId;
            })) {
        return false;
    }
    TimingColouriseEmissiveFalloffKey key{
        .nodeId = std::string{nodeId},
        .parameter = parameter,
        .position = Clamp01(position),
        .value = Clamp01(value),
        .interpolation = IsValidInterpolation(interpolation)
                             ? interpolation
                             : invisible_places::water::
                                   WaterScenarioInterpolation::
                                       SmoothVelocity,
    };
    const auto existing = std::find_if(
        effect->emissiveFalloffKeys.begin(),
        effect->emissiveFalloffKeys.end(),
        [&](const TimingColouriseEmissiveFalloffKey& candidate) {
            return candidate.nodeId == key.nodeId &&
                   candidate.parameter == key.parameter &&
                   std::abs(candidate.position - key.position) <=
                       kTimingColouriseKeyTolerance;
        });
    if (existing == effect->emissiveFalloffKeys.end()) {
        effect->emissiveFalloffKeys.push_back(std::move(key));
    } else {
        *existing = std::move(key);
    }
    *effect = SanitizeTimingColouriseEffect(std::move(*effect));
    return true;
}

std::size_t RemoveTimingColouriseEmissiveFalloffKeysAtPosition(
    TimingColouriseEffect* effect,
    std::string_view nodeId,
    TimingColouriseEmissiveFalloffParameter parameter,
    float position) {
    if (effect == nullptr || !std::isfinite(position)) {
        return 0U;
    }
    return std::erase_if(
        effect->emissiveFalloffKeys,
        [&](const TimingColouriseEmissiveFalloffKey& key) {
            return key.nodeId == nodeId &&
                   key.parameter == parameter &&
                   std::abs(key.position - position) <=
                       kTimingColouriseKeyTolerance;
        });
}

TimingColourisePalette EvaluateTimingColourisePalette(
    const TimingColouriseEffect& effect,
    float normalizedPosition,
    bool cyclic) {
    const auto prepared = PrepareTimingColouriseEffectForEvaluation(
        effect,
        cyclic);
    normalizedPosition = cyclic
        ? WrapTimingColouriseLoopPosition(normalizedPosition)
        : Clamp01(normalizedPosition);
    return EvaluatePreparedTimingColourisePalette(
        prepared,
        normalizedPosition);
}

TimingColouriseLut EvaluateTimingColourisePaletteLut(
    const TimingColouriseEffect& effect,
    float normalizedPosition,
    bool cyclic) {
    const auto sanitized = PrepareTimingColouriseEffectForEvaluation(
        effect,
        cyclic);
    normalizedPosition = cyclic
        ? WrapTimingColouriseLoopPosition(normalizedPosition)
        : Clamp01(normalizedPosition);
    const auto evaluatedParameter =
        [&](TimingColouriseEffectParameter parameter) {
            return EvaluateEffectParameterTrack(
                       sanitized.effectParameterKeys,
                       parameter,
                       normalizedPosition,
                       sanitized.palettePhaseOffset,
                       cyclic)
                .value_or(
                    EffectParameterBaseValue(
                        sanitized,
                        parameter));
        };
    const float phaseOffset = evaluatedParameter(
        TimingColouriseEffectParameter::PalettePhase);
    const float amountOverride = evaluatedParameter(
        TimingColouriseEffectParameter::AmountOverride);
    const float skewCentre = evaluatedParameter(
        TimingColouriseEffectParameter::PaletteSkewCentre);
    const float skewSpread = evaluatedParameter(
        TimingColouriseEffectParameter::PaletteSkewSpread);
    const auto warp = BuildTimingColouriseWarpPoints(
        skewCentre,
        skewSpread,
        sanitized.paletteSkewNodes);
    const auto finalize = [&](TimingColouriseLut lut) {
        // Loop first so the phase rotation cycles a seamless, end-matched
        // palette instead of dragging the mirror seam through the output.
        // Skew is a bounds-fraction remap, so it wraps the palette-space
        // transforms: the skew centre stays pinned while phase animates.
        if (sanitized.paletteLooped) {
            lut = ApplyTimingColourisePaletteLoop(lut);
        }
        return ApplyTimingColouriseAmountOverride(
            ApplyTimingColourisePaletteSkew(
                ApplyTimingColourisePalettePhase(lut, phaseOffset),
                warp),
            sanitized.colouriseAmountOverrideMode,
            amountOverride);
    };
    if (sanitized.paletteKeyModel ==
        TimingColourisePaletteKeyModel::StopParameters) {
        return finalize(CompileTimingColourisePaletteLut(
            EvaluatePreparedTimingColourisePalette(
                sanitized,
                normalizedPosition)));
    }
    if (sanitized.paletteKeys.empty()) {
        return finalize(
            CompileTimingColourisePaletteLut(
                sanitized.basePalette));
    }
    normalizedPosition = Clamp01(normalizedPosition);
    const auto [left, right] =
        BracketingKeys(sanitized.paletteKeys, normalizedPosition);
    auto result = CompileTimingColourisePaletteLut(left->palette);
    if (left == right) {
        return finalize(std::move(result));
    }
    const float span =
        std::max(1.0e-6F, right->position - left->position);
    const float amount = InterpolationAmount(
        left->interpolation,
        (normalizedPosition - left->position) / span);
    if (amount <= 0.0F) {
        return finalize(std::move(result));
    }
    const auto rightLut = CompileTimingColourisePaletteLut(right->palette);
    for (std::size_t index = 0U; index < result.size(); ++index) {
        for (std::size_t channel = 0U; channel < 4U; ++channel) {
            result[index][channel] =
                std::lerp(result[index][channel], rightLut[index][channel], amount);
        }
    }
    return finalize(std::move(result));
}

TimingColouriseBounds EvaluateTimingColouriseBounds(
    const TimingColouriseEffect& effect,
    float normalizedPosition,
    bool cyclic) {
    const auto sanitized = PrepareTimingColouriseEffectForEvaluation(
        effect,
        cyclic);
    normalizedPosition = cyclic
        ? WrapTimingColouriseLoopPosition(normalizedPosition)
        : Clamp01(normalizedPosition);
    const auto fallback = EvaluateLegacyTimingColouriseBounds(
        sanitized,
        normalizedPosition);
    const auto value = [&](TimingColouriseBoundsParameter parameter) {
        const auto evaluated = EvaluateBoundsParameterTrack(
            sanitized.boundsParameterKeys,
            parameter,
            normalizedPosition);
        return evaluated.value_or(
            TimingColouriseBoundsParameterValue(fallback, parameter));
    };

    TimingColouriseBounds result{
        .edgeFadeLower =
            value(TimingColouriseBoundsParameter::EdgeFadeLower),
        .edgeFadeUpper =
            value(TimingColouriseBoundsParameter::EdgeFadeUpper),
    };
    switch (sanitized.boundsKeyMode) {
        case TimingColouriseBoundsKeyMode::CentreSpread: {
            const float centre =
                value(TimingColouriseBoundsParameter::Centre);
            const float halfSpread =
                0.5F * value(TimingColouriseBoundsParameter::Spread);
            result.lower = centre - halfSpread;
            result.upper = centre + halfSpread;
            break;
        }
        case TimingColouriseBoundsKeyMode::LowerSpread:
            result.lower = value(TimingColouriseBoundsParameter::Lower);
            result.upper =
                result.lower +
                value(TimingColouriseBoundsParameter::Spread);
            break;
        case TimingColouriseBoundsKeyMode::UpperSpread:
            result.upper = value(TimingColouriseBoundsParameter::Upper);
            result.lower =
                result.upper -
                value(TimingColouriseBoundsParameter::Spread);
            break;
        case TimingColouriseBoundsKeyMode::LowerUpper:
        default:
            result.lower = value(TimingColouriseBoundsParameter::Lower);
            result.upper = value(TimingColouriseBoundsParameter::Upper);
            break;
    }
    if (result.lower > result.upper) {
        // Independent endpoint curves can cross between otherwise-valid keys.
        // Collapse at the crossing instead of swapping endpoint identities,
        // which would make the authored Lower curve suddenly drive Upper.
        const float centre = std::midpoint(result.lower, result.upper);
        result.lower = centre;
        result.upper = centre;
    }
    return SanitizeTimingColouriseBounds(result);
}

TimingColouriseLayerSample SampleTimingColouriseLut(
    const TimingColouriseLut& lut,
    float normalizedFieldValue) {
    const float scaled =
        Clamp01(normalizedFieldValue) * static_cast<float>(lut.size() - 1U);
    const auto leftIndex = static_cast<std::size_t>(std::floor(scaled));
    const auto rightIndex = std::min(leftIndex + 1U, lut.size() - 1U);
    const float amount = scaled - static_cast<float>(leftIndex);
    TimingColouriseLayerSample sample;
    for (std::size_t channel = 0U; channel < 3U; ++channel) {
        sample.colour[channel] =
            std::lerp(lut[leftIndex][channel], lut[rightIndex][channel], amount);
    }
    sample.colouriseAmount =
        std::lerp(lut[leftIndex][3], lut[rightIndex][3], amount);
    return sample;
}

float TimingColouriseBoundsMask(
    const TimingColouriseBounds& bounds,
    float fieldValue) {
    // Mirrors the per-edge smoothstep mask the renderer computes in
    // shaders/pointcloud_timing_colourise.glsl and the offline port in
    // OfflinePointRenderer, so model-level answers match the shipped image.
    if (!std::isfinite(fieldValue)) {
        return 0.0F;
    }
    const auto sanitized = SanitizeTimingColouriseBounds(bounds);
    const float span = sanitized.upper - sanitized.lower;
    if (span <= std::numeric_limits<float>::epsilon()) {
        return 0.0F;
    }
    const auto smoothStep = [](float edge0, float edge1, float value) {
        const float amount = Clamp01(
            (value - edge0) / std::max(
                                  edge1 - edge0,
                                  std::numeric_limits<float>::epsilon()));
        return amount * amount * (3.0F - 2.0F * amount);
    };
    const float lowerOutward =
        span * std::max(-sanitized.edgeFadeLower, 0.0F);
    const float upperOutward =
        span * std::max(-sanitized.edgeFadeUpper, 0.0F);
    if (fieldValue < sanitized.lower - lowerOutward ||
        fieldValue > sanitized.upper + upperOutward) {
        return 0.0F;
    }
    const float normalized =
        Clamp01((fieldValue - sanitized.lower) / span);
    float lowerAmount = 1.0F;
    if (sanitized.edgeFadeLower > 1.0e-6F) {
        lowerAmount = smoothStep(
            0.0F,
            sanitized.edgeFadeLower,
            normalized);
    } else if (sanitized.edgeFadeLower < -1.0e-6F) {
        lowerAmount = smoothStep(
            sanitized.lower - lowerOutward,
            sanitized.lower,
            fieldValue);
    }
    float upperAmount = 1.0F;
    if (sanitized.edgeFadeUpper > 1.0e-6F) {
        upperAmount = smoothStep(
            0.0F,
            sanitized.edgeFadeUpper,
            1.0F - normalized);
    } else if (sanitized.edgeFadeUpper < -1.0e-6F) {
        upperAmount = 1.0F - smoothStep(
                                 sanitized.upper,
                                 sanitized.upper + upperOutward,
                                 fieldValue);
    }
    return Clamp01(std::min(lowerAmount, upperAmount));
}

std::array<float, 3> ApplyTimingColouriseStack(
    std::array<float, 3> baseColour,
    std::span<const TimingColouriseLayerSample> samples) {
    for (auto it = samples.rbegin(); it != samples.rend(); ++it) {
        const float amount = Clamp01(it->colouriseAmount);
        for (std::size_t channel = 0U; channel < baseColour.size(); ++channel) {
            baseColour[channel] =
                std::lerp(baseColour[channel], Clamp01(it->colour[channel]), amount);
        }
    }
    return baseColour;
}

void AddOrUpdateTimingColourisePaletteKey(
    TimingColouriseEffect* effect,
    float position,
    TimingColourisePalette palette,
    invisible_places::water::WaterScenarioInterpolation interpolation) {
    if (effect == nullptr) {
        return;
    }
    effect->paletteKeyModel =
        TimingColourisePaletteKeyModel::LegacySnapshots;
    AddOrUpdateKey(
        &effect->paletteKeys,
        position,
        std::move(palette),
        IsValidInterpolation(interpolation)
            ? interpolation
            : invisible_places::water::
                  WaterScenarioInterpolation::Smooth,
        SanitizeTimingColourisePalette);
}

bool AddOrUpdateTimingColourisePaletteStopScalarKey(
    TimingColouriseEffect* effect,
    std::string_view stopId,
    TimingColourisePaletteStopParameter parameter,
    float position,
    float value,
    invisible_places::water::WaterScenarioInterpolation interpolation) {
    if (effect == nullptr || stopId.empty() ||
        (parameter != TimingColourisePaletteStopParameter::Position &&
         parameter !=
             TimingColourisePaletteStopParameter::ColouriseAmount) ||
        !std::isfinite(position) || !std::isfinite(value)) {
        return false;
    }
    effect->basePalette =
        SanitizeTimingColourisePalette(std::move(effect->basePalette));
    const bool stopExists = std::any_of(
        effect->basePalette.stops.begin(),
        effect->basePalette.stops.end(),
        [&](const TimingColourisePaletteStop& stop) {
            return stop.id == stopId;
        });
    if (!stopExists ||
        (effect->paletteKeyModel ==
             TimingColourisePaletteKeyModel::LegacySnapshots &&
         !effect->paletteKeys.empty())) {
        return false;
    }
    effect->paletteKeyModel =
        TimingColourisePaletteKeyModel::StopParameters;
    TimingColourisePaletteStopParameterKey key{
        .stopId = std::string{stopId},
        .parameter = parameter,
        .position = Clamp01(position),
        .scalarValue =
            SanitizePaletteStopScalarValue(parameter, value),
        .interpolation = IsValidInterpolation(interpolation)
                             ? interpolation
                             : invisible_places::water::
                                   WaterScenarioInterpolation::Smooth,
    };
    const auto existing = std::find_if(
        effect->paletteStopParameterKeys.begin(),
        effect->paletteStopParameterKeys.end(),
        [&](const TimingColourisePaletteStopParameterKey& candidate) {
            return candidate.stopId == key.stopId &&
                   candidate.parameter == parameter &&
                   std::abs(candidate.position - key.position) <=
                       kTimingColouriseKeyTolerance;
        });
    if (existing == effect->paletteStopParameterKeys.end()) {
        effect->paletteStopParameterKeys.push_back(std::move(key));
    } else {
        *existing = std::move(key);
    }
    SortAndCoalescePaletteStopParameterKeys(
        &effect->paletteStopParameterKeys);
    return true;
}

bool AddOrUpdateTimingColourisePaletteStopColourKey(
    TimingColouriseEffect* effect,
    std::string_view stopId,
    float position,
    std::array<float, 3> colour,
    invisible_places::water::WaterScenarioInterpolation interpolation) {
    if (effect == nullptr || stopId.empty() ||
        !std::isfinite(position) ||
        std::any_of(
            colour.begin(),
            colour.end(),
            [](float channel) { return !std::isfinite(channel); })) {
        return false;
    }
    effect->basePalette =
        SanitizeTimingColourisePalette(std::move(effect->basePalette));
    const bool stopExists = std::any_of(
        effect->basePalette.stops.begin(),
        effect->basePalette.stops.end(),
        [&](const TimingColourisePaletteStop& stop) {
            return stop.id == stopId;
        });
    if (!stopExists ||
        (effect->paletteKeyModel ==
             TimingColourisePaletteKeyModel::LegacySnapshots &&
         !effect->paletteKeys.empty())) {
        return false;
    }
    effect->paletteKeyModel =
        TimingColourisePaletteKeyModel::StopParameters;
    TimingColourisePaletteStopParameterKey key{
        .stopId = std::string{stopId},
        .parameter = TimingColourisePaletteStopParameter::Colour,
        .position = Clamp01(position),
        .colourValue = SanitizePaletteStopColour(colour),
        .interpolation = IsValidInterpolation(interpolation)
                             ? interpolation
                             : invisible_places::water::
                                   WaterScenarioInterpolation::Smooth,
    };
    const auto existing = std::find_if(
        effect->paletteStopParameterKeys.begin(),
        effect->paletteStopParameterKeys.end(),
        [&](const TimingColourisePaletteStopParameterKey& candidate) {
            return candidate.stopId == key.stopId &&
                   candidate.parameter == key.parameter &&
                   std::abs(candidate.position - key.position) <=
                       kTimingColouriseKeyTolerance;
        });
    if (existing == effect->paletteStopParameterKeys.end()) {
        effect->paletteStopParameterKeys.push_back(std::move(key));
    } else {
        *existing = std::move(key);
    }
    SortAndCoalescePaletteStopParameterKeys(
        &effect->paletteStopParameterKeys);
    return true;
}

void AddOrUpdateTimingColouriseBoundsKey(
    TimingColouriseEffect* effect,
    float position,
    TimingColouriseBounds bounds,
    invisible_places::water::WaterScenarioInterpolation interpolation) {
    if (effect == nullptr) {
        return;
    }
    AddOrUpdateKey(
        &effect->boundsKeys,
        position,
        bounds,
        IsValidInterpolation(interpolation)
            ? interpolation
            : invisible_places::water::
                  WaterScenarioInterpolation::Smooth,
        SanitizeTimingColouriseBounds);
}

bool AddOrUpdateTimingColouriseBoundsParameterKey(
    TimingColouriseEffect* effect,
    TimingColouriseBoundsParameter parameter,
    float position,
    float value,
    invisible_places::water::WaterScenarioInterpolation interpolation) {
    if (effect == nullptr || !std::isfinite(position) ||
        !std::isfinite(value) ||
        !TimingColouriseBoundsParameterIsAllowed(
            effect->boundsKeyMode,
            parameter)) {
        return false;
    }
    const bool firstKeyForParameter = std::none_of(
        effect->boundsParameterKeys.begin(),
        effect->boundsParameterKeys.end(),
        [&](const TimingColouriseBoundsParameterKey& candidate) {
            return candidate.parameter == parameter;
        });
    if (firstKeyForParameter && !effect->boundsKeys.empty()) {
        // Project the legacy snapshot track into this scalar track before
        // adding its first independently authored key. Without this seeding,
        // first/last hold semantics would replace the entire legacy component
        // with the one new value outside that key.
        effect->boundsParameterKeys.reserve(
            effect->boundsParameterKeys.size() +
            effect->boundsKeys.size() + 1U);
        for (const auto& legacy : effect->boundsKeys) {
            effect->boundsParameterKeys.push_back({
                .parameter = parameter,
                .position = legacy.position,
                .value = TimingColouriseBoundsParameterValue(
                    legacy.bounds,
                    parameter),
                .interpolation =
                    IsValidInterpolation(legacy.interpolation)
                        ? legacy.interpolation
                        : invisible_places::water::
                              WaterScenarioInterpolation::Smooth,
            });
        }
    }
    TimingColouriseBoundsParameterKey key{
        .parameter = parameter,
        .position = Clamp01(position),
        .value = SanitizeBoundsParameterValue(parameter, value),
        .interpolation = IsValidInterpolation(interpolation)
                             ? interpolation
                             : invisible_places::water::
                                   WaterScenarioInterpolation::Smooth,
    };
    const auto existing = std::find_if(
        effect->boundsParameterKeys.begin(),
        effect->boundsParameterKeys.end(),
        [&](const TimingColouriseBoundsParameterKey& candidate) {
            return candidate.parameter == parameter &&
                   std::abs(candidate.position - key.position) <=
                       kTimingColouriseKeyTolerance;
        });
    if (existing == effect->boundsParameterKeys.end()) {
        effect->boundsParameterKeys.push_back(key);
    } else {
        *existing = key;
    }
    SortAndCoalesceBoundsParameterKeys(&effect->boundsParameterKeys);
    return true;
}

bool AddOrUpdateTimingColouriseEffectParameterKey(
    TimingColouriseEffect* effect,
    TimingColouriseEffectParameter parameter,
    float position,
    float value,
    invisible_places::water::WaterScenarioInterpolation interpolation) {
    if (effect == nullptr ||
        !TimingEffectParameterIsSupported(*effect, parameter) ||
        !std::isfinite(position) || !std::isfinite(value)) {
        return false;
    }
    TimingColouriseEffectParameterKey key{
        .parameter = parameter,
        .position = Clamp01(position),
        .value = SanitizeEffectParameterValue(parameter, value),
        .interpolation = IsValidInterpolation(interpolation)
                             ? interpolation
                             : invisible_places::water::
                                   WaterScenarioInterpolation::Smooth,
    };
    const auto existing = std::find_if(
        effect->effectParameterKeys.begin(),
        effect->effectParameterKeys.end(),
        [&](const TimingColouriseEffectParameterKey& candidate) {
            return candidate.parameter == parameter &&
                   std::abs(candidate.position - key.position) <=
                       kTimingColouriseKeyTolerance;
        });
    if (existing == effect->effectParameterKeys.end()) {
        // A phase key stores a delta from its predecessor. When inserting
        // between two existing phase keys, split the following delta so the
        // next accumulated target (and everything after it) stays put. If an
        // extreme authored edit would require more than one turn on the next
        // key, keep the requested delta and let the later path shift instead.
        if (parameter == TimingColouriseEffectParameter::PalettePhase) {
            auto next = effect->effectParameterKeys.end();
            for (auto candidate = effect->effectParameterKeys.begin();
                 candidate != effect->effectParameterKeys.end();
                 ++candidate) {
                if (candidate->parameter == parameter &&
                    candidate->position >
                        key.position + kTimingColouriseKeyTolerance &&
                    (next == effect->effectParameterKeys.end() ||
                     candidate->position < next->position)) {
                    next = candidate;
                }
            }
            if (next != effect->effectParameterKeys.end()) {
                const float splitDelta = next->value - key.value;
                if (splitDelta >= -1.0F && splitDelta <= 1.0F) {
                    next->value = ClampPalettePhaseDelta(splitDelta);
                }
            }
        }
        effect->effectParameterKeys.push_back(key);
    } else {
        *existing = key;
    }
    SortAndCoalesceEffectParameterKeys(
        &effect->effectParameterKeys);
    return true;
}

bool MoveTimingColourisePaletteKey(
    TimingColouriseEffect* effect,
    float sourcePosition,
    float destinationPosition) {
    if (effect == nullptr ||
        !CanMoveTimingColourisePaletteKeysAtPosition(
            *effect,
            sourcePosition,
            destinationPosition)) {
        return false;
    }
    if (std::abs(destinationPosition - sourcePosition) <=
        kTimingColouriseKeyTolerance) {
        return true;
    }
    const auto clusters = PaletteKeyPositionClusters(*effect);
    const auto sourceClusterIndex =
        PaletteKeyPositionClusterIndexAtPosition(
            clusters,
            sourcePosition);
    if (!sourceClusterIndex.has_value()) {
        return false;
    }
    const auto sourceCluster = clusters[*sourceClusterIndex];
    for (auto& key : effect->paletteKeys) {
        if (PaletteKeyPositionBelongsToCluster(
                key.position,
                sourceCluster)) {
            key.position = destinationPosition;
        }
    }
    for (auto& key : effect->paletteStopParameterKeys) {
        if (PaletteKeyPositionBelongsToCluster(
                key.position,
                sourceCluster)) {
            key.position = destinationPosition;
        }
    }
    SortAndCoalesceKeys(&effect->paletteKeys);
    SortAndCoalescePaletteStopParameterKeys(
        &effect->paletteStopParameterKeys);
    return true;
}

bool CanMoveTimingColourisePaletteKeysAtPosition(
    const TimingColouriseEffect& effect,
    float sourcePosition,
    float destinationPosition) {
    if (!std::isfinite(sourcePosition) ||
        !std::isfinite(destinationPosition) ||
        destinationPosition < 0.0F || destinationPosition > 1.0F) {
        return false;
    }
    const auto clusters = PaletteKeyPositionClusters(effect);
    const auto sourceClusterIndex =
        PaletteKeyPositionClusterIndexAtPosition(
            clusters,
            sourcePosition);
    if (!sourceClusterIndex.has_value()) {
        return false;
    }
    if (std::abs(destinationPosition - sourcePosition) <=
        kTimingColouriseKeyTolerance) {
        return true;
    }
    const auto& sourceCluster = clusters[*sourceClusterIndex];
    const auto isSourcePosition = [&](float position) {
        return PaletteKeyPositionBelongsToCluster(
            position,
            sourceCluster);
    };
    const auto isDestinationPosition = [&](float position) {
        if (!std::isfinite(position) || isSourcePosition(position)) {
            return false;
        }
        for (std::size_t index = 0U; index < clusters.size(); ++index) {
            if (index == *sourceClusterIndex ||
                DistanceFromPaletteKeyPositionCluster(
                    clusters[index],
                    destinationPosition) >
                    kTimingColouriseKeyTolerance) {
                continue;
            }
            if (PaletteKeyPositionBelongsToCluster(
                    position,
                    clusters[index])) {
                return true;
            }
        }
        return false;
    };
    const bool hasLegacySource = std::any_of(
        effect.paletteKeys.begin(),
        effect.paletteKeys.end(),
        [&](const TimingColourisePaletteKey& key) {
            return isSourcePosition(key.position);
        });
    const bool hasLegacyDestination = std::any_of(
        effect.paletteKeys.begin(),
        effect.paletteKeys.end(),
        [&](const TimingColourisePaletteKey& key) {
            return isDestinationPosition(key.position);
        });
    if (hasLegacyDestination) {
        // A legacy snapshot owns every stop property at its position.
        return false;
    }
    if (hasLegacySource &&
        std::any_of(
            effect.paletteStopParameterKeys.begin(),
            effect.paletteStopParameterKeys.end(),
            [&](const TimingColourisePaletteStopParameterKey& key) {
                return isDestinationPosition(key.position);
            })) {
        return false;
    }
    for (const auto& source : effect.paletteStopParameterKeys) {
        if (!isSourcePosition(source.position)) {
            continue;
        }
        const bool sameTrackOccupied = std::any_of(
            effect.paletteStopParameterKeys.begin(),
            effect.paletteStopParameterKeys.end(),
            [&](const TimingColourisePaletteStopParameterKey& candidate) {
                return candidate.stopId == source.stopId &&
                       candidate.parameter == source.parameter &&
                       isDestinationPosition(candidate.position);
            });
        if (sameTrackOccupied) {
            return false;
        }
    }
    return true;
}

bool MoveTimingColouriseBoundsKey(
    TimingColouriseEffect* effect,
    float sourcePosition,
    float destinationPosition) {
    if (effect == nullptr || !std::isfinite(sourcePosition) ||
        !std::isfinite(destinationPosition) ||
        destinationPosition < 0.0F || destinationPosition > 1.0F) {
        return false;
    }
    const bool hasLegacySource =
        KeyCountAtPosition(effect->boundsKeys, sourcePosition) != 0U;
    const bool hasParameterSource = std::any_of(
        effect->boundsParameterKeys.begin(),
        effect->boundsParameterKeys.end(),
        [&](const TimingColouriseBoundsParameterKey& key) {
            return std::abs(key.position - sourcePosition) <=
                   kTimingColouriseKeyTolerance;
        });
    if (!hasLegacySource && !hasParameterSource) {
        return false;
    }
    if (std::abs(destinationPosition - sourcePosition) <=
        kTimingColouriseKeyTolerance) {
        return true;
    }
    const bool unionDestinationOccupied =
        KeyCountAtPosition(
            effect->boundsKeys,
            destinationPosition) != 0U ||
        std::any_of(
            effect->boundsParameterKeys.begin(),
            effect->boundsParameterKeys.end(),
            [&](const TimingColouriseBoundsParameterKey& key) {
                return std::abs(
                           key.position - destinationPosition) <=
                       kTimingColouriseKeyTolerance;
            });
    if (unionDestinationOccupied) {
        return false;
    }
    if (hasLegacySource &&
        std::any_of(
            effect->boundsKeys.begin(),
            effect->boundsKeys.end(),
            [&](const TimingColouriseBoundsKey& key) {
                return std::abs(key.position - sourcePosition) >
                           kTimingColouriseKeyTolerance &&
                       std::abs(key.position - destinationPosition) <=
                           kTimingColouriseKeyTolerance;
            })) {
        return false;
    }
    for (const auto& source : effect->boundsParameterKeys) {
        if (std::abs(source.position - sourcePosition) >
            kTimingColouriseKeyTolerance) {
            continue;
        }
        const bool occupied = std::any_of(
            effect->boundsParameterKeys.begin(),
            effect->boundsParameterKeys.end(),
            [&](const TimingColouriseBoundsParameterKey& candidate) {
                return candidate.parameter == source.parameter &&
                       std::abs(candidate.position - sourcePosition) >
                           kTimingColouriseKeyTolerance &&
                       std::abs(
                           candidate.position - destinationPosition) <=
                           kTimingColouriseKeyTolerance;
            });
        if (occupied) {
            return false;
        }
    }
    for (auto& key : effect->boundsKeys) {
        if (std::abs(key.position - sourcePosition) <=
            kTimingColouriseKeyTolerance) {
            key.position = destinationPosition;
        }
    }
    for (auto& key : effect->boundsParameterKeys) {
        if (std::abs(key.position - sourcePosition) <=
            kTimingColouriseKeyTolerance) {
            key.position = destinationPosition;
        }
    }
    SortAndCoalesceKeys(&effect->boundsKeys);
    SortAndCoalesceBoundsParameterKeys(&effect->boundsParameterKeys);
    return true;
}

bool CanMoveTimingColouriseBoundsParameterKeysAtPosition(
    const TimingColouriseEffect& effect,
    TimingColouriseBoundsParameter parameter,
    float sourcePosition,
    float destinationPosition) {
    if (!TimingColouriseBoundsParameterIsAllowed(
            effect.boundsKeyMode,
            parameter) ||
        !std::isfinite(sourcePosition) ||
        !std::isfinite(destinationPosition) ||
        destinationPosition < 0.0F || destinationPosition > 1.0F) {
        return false;
    }
    const auto geometricParameters =
        TimingColouriseBoundsParametersForMode(effect.boundsKeyMode);
    const bool moveGeometricPair =
        std::find(
            geometricParameters.begin(),
            geometricParameters.end(),
            parameter) != geometricParameters.end();
    const auto movesParameter =
        [&](TimingColouriseBoundsParameter candidate) {
            return candidate == parameter ||
                   (moveGeometricPair &&
                    std::find(
                        geometricParameters.begin(),
                        geometricParameters.end(),
                        candidate) != geometricParameters.end());
        };
    const bool hasRequestedSource = std::any_of(
        effect.boundsParameterKeys.begin(),
        effect.boundsParameterKeys.end(),
        [&](const TimingColouriseBoundsParameterKey& key) {
            return key.parameter == parameter &&
                   std::abs(key.position - sourcePosition) <=
                       kTimingColouriseKeyTolerance;
        });
    if (!hasRequestedSource) {
        return false;
    }
    if (std::abs(destinationPosition - sourcePosition) <=
        kTimingColouriseKeyTolerance) {
        return true;
    }
    for (const auto& source : effect.boundsParameterKeys) {
        if (!movesParameter(source.parameter) ||
            std::abs(source.position - sourcePosition) >
                kTimingColouriseKeyTolerance) {
            continue;
        }
        const bool occupied = std::any_of(
            effect.boundsParameterKeys.begin(),
            effect.boundsParameterKeys.end(),
            [&](const TimingColouriseBoundsParameterKey& candidate) {
                return candidate.parameter == source.parameter &&
                       std::abs(candidate.position - sourcePosition) >
                           kTimingColouriseKeyTolerance &&
                       std::abs(
                           candidate.position - destinationPosition) <=
                           kTimingColouriseKeyTolerance;
            });
        if (occupied) {
            return false;
        }
    }
    return true;
}

bool MoveTimingColouriseBoundsParameterKey(
    TimingColouriseEffect* effect,
    TimingColouriseBoundsParameter parameter,
    float sourcePosition,
    float destinationPosition) {
    if (effect == nullptr ||
        !CanMoveTimingColouriseBoundsParameterKeysAtPosition(
            *effect,
            parameter,
            sourcePosition,
            destinationPosition)) {
        return false;
    }
    const auto geometricParameters =
        TimingColouriseBoundsParametersForMode(effect->boundsKeyMode);
    const bool moveGeometricPair =
        std::find(
            geometricParameters.begin(),
            geometricParameters.end(),
            parameter) != geometricParameters.end();
    for (auto& key : effect->boundsParameterKeys) {
        const bool parameterMoves =
            key.parameter == parameter ||
            (moveGeometricPair &&
             std::find(
                 geometricParameters.begin(),
                 geometricParameters.end(),
                 key.parameter) != geometricParameters.end());
        if (parameterMoves &&
            std::abs(key.position - sourcePosition) <=
                kTimingColouriseKeyTolerance) {
            key.position = destinationPosition;
        }
    }
    SortAndCoalesceBoundsParameterKeys(&effect->boundsParameterKeys);
    return true;
}

bool MoveTimingColouriseEffectParameterKey(
    TimingColouriseEffect* effect,
    TimingColouriseEffectParameter parameter,
    float sourcePosition,
    float destinationPosition) {
    if (effect == nullptr ||
        !TimingEffectParameterIsSupported(*effect, parameter) ||
        !std::isfinite(destinationPosition) ||
        destinationPosition < 0.0F || destinationPosition > 1.0F) {
        return false;
    }
    const auto source = std::find_if(
        effect->effectParameterKeys.begin(),
        effect->effectParameterKeys.end(),
        [&](const TimingColouriseEffectParameterKey& key) {
            return key.parameter == parameter &&
                   std::abs(key.position - sourcePosition) <=
                       kTimingColouriseKeyTolerance;
        });
    if (source == effect->effectParameterKeys.end()) {
        return false;
    }
    const bool occupied = std::any_of(
        effect->effectParameterKeys.begin(),
        effect->effectParameterKeys.end(),
        [&](const TimingColouriseEffectParameterKey& key) {
            return &key != &*source && key.parameter == parameter &&
                   std::abs(key.position - destinationPosition) <=
                       kTimingColouriseKeyTolerance;
        });
    if (occupied) {
        return false;
    }
    source->position = destinationPosition;
    SortAndCoalesceEffectParameterKeys(
        &effect->effectParameterKeys);
    return true;
}

bool CanMoveTimingColouriseEffectParameterKeysAtPosition(
    const TimingColouriseEffect& effect,
    float sourcePosition,
    float destinationPosition) {
    if (!std::isfinite(sourcePosition) ||
        !std::isfinite(destinationPosition) ||
        destinationPosition < 0.0F || destinationPosition > 1.0F) {
        return false;
    }
    const bool hasSource = std::any_of(
        effect.effectParameterKeys.begin(),
        effect.effectParameterKeys.end(),
        [&](const TimingColouriseEffectParameterKey& key) {
            return TimingEffectParameterIsSupported(
                       effect,
                       key.parameter) &&
                   std::abs(key.position - sourcePosition) <=
                       kTimingColouriseKeyTolerance;
        });
    if (!hasSource) {
        return false;
    }
    if (std::abs(destinationPosition - sourcePosition) <=
        kTimingColouriseKeyTolerance) {
        return true;
    }
    for (const auto& source : effect.effectParameterKeys) {
        if (!TimingEffectParameterIsSupported(
                effect,
                source.parameter) ||
            std::abs(source.position - sourcePosition) >
                kTimingColouriseKeyTolerance) {
            continue;
        }
        const bool occupied = std::any_of(
            effect.effectParameterKeys.begin(),
            effect.effectParameterKeys.end(),
            [&](const TimingColouriseEffectParameterKey& candidate) {
                return candidate.parameter == source.parameter &&
                       std::abs(candidate.position - sourcePosition) >
                           kTimingColouriseKeyTolerance &&
                       std::abs(candidate.position - destinationPosition) <=
                           kTimingColouriseKeyTolerance;
            });
        if (occupied) {
            return false;
        }
    }
    return true;
}

bool MoveTimingColouriseEffectParameterKeys(
    TimingColouriseEffect* effect,
    float sourcePosition,
    float destinationPosition) {
    if (effect == nullptr ||
        !CanMoveTimingColouriseEffectParameterKeysAtPosition(
            *effect,
            sourcePosition,
            destinationPosition)) {
        return false;
    }
    for (auto& key : effect->effectParameterKeys) {
        if (TimingEffectParameterIsSupported(
                *effect,
                key.parameter) &&
            std::abs(key.position - sourcePosition) <=
                kTimingColouriseKeyTolerance) {
            key.position = destinationPosition;
        }
    }
    SortAndCoalesceEffectParameterKeys(
        &effect->effectParameterKeys);
    return true;
}

std::size_t TimingColourisePaletteKeyCountAtPosition(
    const TimingColouriseEffect& effect,
    float position) {
    const auto clusters = PaletteKeyPositionClusters(effect);
    const auto clusterIndex =
        PaletteKeyPositionClusterIndexAtPosition(clusters, position);
    if (!clusterIndex.has_value()) {
        return 0U;
    }
    const auto& cluster = clusters[*clusterIndex];
    const auto belongs = [&](const auto& key) {
        return PaletteKeyPositionBelongsToCluster(
            key.position,
            cluster);
    };
    return static_cast<std::size_t>(
               std::count_if(
                   effect.paletteKeys.begin(),
                   effect.paletteKeys.end(),
                   belongs)) +
           static_cast<std::size_t>(
               std::count_if(
                   effect.paletteStopParameterKeys.begin(),
                   effect.paletteStopParameterKeys.end(),
                   belongs));
}

std::size_t TimingColourisePaletteStopParameterKeyCountAtPosition(
    const TimingColouriseEffect& effect,
    std::string_view stopId,
    TimingColourisePaletteStopParameter parameter,
    float position) {
    if (!std::isfinite(position)) {
        return 0U;
    }
    return static_cast<std::size_t>(std::count_if(
        effect.paletteStopParameterKeys.begin(),
        effect.paletteStopParameterKeys.end(),
        [&](const TimingColourisePaletteStopParameterKey& key) {
            return key.stopId == stopId && key.parameter == parameter &&
                   std::abs(key.position - position) <=
                       kTimingColouriseKeyTolerance;
        }));
}

std::size_t TimingColouriseBoundsParameterKeyCountAtPosition(
    const TimingColouriseEffect& effect,
    TimingColouriseBoundsParameter parameter,
    float position) {
    if (!std::isfinite(position)) {
        return 0U;
    }
    return static_cast<std::size_t>(std::count_if(
        effect.boundsParameterKeys.begin(),
        effect.boundsParameterKeys.end(),
        [&](const TimingColouriseBoundsParameterKey& key) {
            return key.parameter == parameter &&
                   std::abs(key.position - position) <=
                       kTimingColouriseKeyTolerance;
        }));
}

std::size_t TimingColouriseEffectParameterKeyCountAtPosition(
    const TimingColouriseEffect& effect,
    TimingColouriseEffectParameter parameter,
    float position) {
    if (!TimingEffectParameterIsSupported(effect, parameter) ||
        !std::isfinite(position)) {
        return 0U;
    }
    return static_cast<std::size_t>(std::count_if(
        effect.effectParameterKeys.begin(),
        effect.effectParameterKeys.end(),
        [&](const TimingColouriseEffectParameterKey& key) {
            return key.parameter == parameter &&
                   std::abs(key.position - position) <=
                       kTimingColouriseKeyTolerance;
        }));
}

std::size_t TimingColouriseEffectParameterUnionKeyCountAtPosition(
    const TimingColouriseEffect& effect,
    float position) {
    if (!std::isfinite(position)) {
        return 0U;
    }
    return static_cast<std::size_t>(std::count_if(
        effect.effectParameterKeys.begin(),
        effect.effectParameterKeys.end(),
        [&](const TimingColouriseEffectParameterKey& key) {
            return TimingEffectParameterIsSupported(
                       effect,
                       key.parameter) &&
                   std::abs(key.position - position) <=
                       kTimingColouriseKeyTolerance;
        }));
}

std::size_t TimingColouriseBoundsKeyCountAtPosition(
    const TimingColouriseEffect& effect,
    float position) {
    return KeyCountAtPosition(effect.boundsKeys, position) +
           static_cast<std::size_t>(std::count_if(
               effect.boundsParameterKeys.begin(),
               effect.boundsParameterKeys.end(),
               [&](const TimingColouriseBoundsParameterKey& key) {
                   return std::isfinite(position) &&
                          std::abs(key.position - position) <=
                              kTimingColouriseKeyTolerance;
               }));
}

std::size_t TimingColouriseEffectKeyCountAtPosition(
    const TimingColouriseEffect& effect,
    float position) {
    const auto paletteKeyCount =
        effect.colouriseEnabled
            ? TimingColourisePaletteKeyCountAtPosition(effect, position)
            : 0U;
    return paletteKeyCount +
           TimingColouriseEffectParameterUnionKeyCountAtPosition(
               effect,
               position) +
           TimingColouriseBoundsKeyCountAtPosition(effect, position);
}

std::size_t RemoveTimingColourisePaletteKeysAtPosition(
    TimingColouriseEffect* effect,
    float position) {
    if (effect == nullptr || !std::isfinite(position)) {
        return 0U;
    }
    const auto clusters = PaletteKeyPositionClusters(*effect);
    const auto clusterIndex =
        PaletteKeyPositionClusterIndexAtPosition(clusters, position);
    if (!clusterIndex.has_value()) {
        return 0U;
    }
    const auto cluster = clusters[*clusterIndex];
    const auto previousLegacySize = effect->paletteKeys.size();
    std::erase_if(
        effect->paletteKeys,
        [&](const TimingColourisePaletteKey& key) {
            return PaletteKeyPositionBelongsToCluster(
                key.position,
                cluster);
        });
    const auto previousParameterSize =
        effect->paletteStopParameterKeys.size();
    std::erase_if(
        effect->paletteStopParameterKeys,
        [&](const TimingColourisePaletteStopParameterKey& key) {
            return PaletteKeyPositionBelongsToCluster(
                key.position,
                cluster);
        });
    return previousLegacySize - effect->paletteKeys.size() +
           previousParameterSize -
               effect->paletteStopParameterKeys.size();
}

std::size_t RemoveTimingColourisePaletteStopParameterKeysAtPosition(
    TimingColouriseEffect* effect,
    std::string_view stopId,
    TimingColourisePaletteStopParameter parameter,
    float position) {
    if (effect == nullptr || !std::isfinite(position)) {
        return 0U;
    }
    const auto previousSize =
        effect->paletteStopParameterKeys.size();
    std::erase_if(
        effect->paletteStopParameterKeys,
        [&](const TimingColourisePaletteStopParameterKey& key) {
            return key.stopId == stopId && key.parameter == parameter &&
                   std::abs(key.position - position) <=
                       kTimingColouriseKeyTolerance;
        });
    return previousSize - effect->paletteStopParameterKeys.size();
}

std::size_t RemoveTimingColouriseBoundsKeysAtPosition(
    TimingColouriseEffect* effect,
    float position) {
    if (effect == nullptr) {
        return 0U;
    }
    const auto legacy =
        RemoveKeysAtPosition(&effect->boundsKeys, position);
    if (!std::isfinite(position)) {
        return legacy;
    }
    const auto previousSize = effect->boundsParameterKeys.size();
    std::erase_if(
        effect->boundsParameterKeys,
        [&](const TimingColouriseBoundsParameterKey& key) {
            return std::abs(key.position - position) <=
                   kTimingColouriseKeyTolerance;
        });
    return legacy + previousSize - effect->boundsParameterKeys.size();
}

std::size_t RemoveTimingColouriseBoundsParameterKeysAtPosition(
    TimingColouriseEffect* effect,
    TimingColouriseBoundsParameter parameter,
    float position) {
    if (effect == nullptr || !std::isfinite(position)) {
        return 0U;
    }
    const auto previousSize = effect->boundsParameterKeys.size();
    std::erase_if(
        effect->boundsParameterKeys,
        [&](const TimingColouriseBoundsParameterKey& key) {
            return key.parameter == parameter &&
                   std::abs(key.position - position) <=
                       kTimingColouriseKeyTolerance;
        });
    return previousSize - effect->boundsParameterKeys.size();
}

std::size_t RemoveTimingColouriseEffectParameterKeysAtPosition(
    TimingColouriseEffect* effect,
    TimingColouriseEffectParameter parameter,
    float position) {
    if (effect == nullptr ||
        !TimingEffectParameterIsSupported(*effect, parameter) ||
        !std::isfinite(position)) {
        return 0U;
    }
    const auto removed = std::find_if(
        effect->effectParameterKeys.begin(),
        effect->effectParameterKeys.end(),
        [&](const TimingColouriseEffectParameterKey& key) {
            return key.parameter == parameter &&
                   std::abs(key.position - position) <=
                       kTimingColouriseKeyTolerance;
        });
    if (removed == effect->effectParameterKeys.end()) {
        return 0U;
    }
    if (parameter == TimingColouriseEffectParameter::PalettePhase) {
        auto next = effect->effectParameterKeys.end();
        for (auto candidate = effect->effectParameterKeys.begin();
             candidate != effect->effectParameterKeys.end();
             ++candidate) {
            if (candidate->parameter == parameter &&
                candidate->position >
                    removed->position + kTimingColouriseKeyTolerance &&
                (next == effect->effectParameterKeys.end() ||
                 candidate->position < next->position)) {
                next = candidate;
            }
        }
        if (next != effect->effectParameterKeys.end()) {
            // Whole turns are invisible to the palette, so a joined delta
            // outside the stored range folds back into one turn rather than
            // being skipped (which silently lost the removed key's delta).
            next->value = WrapTimingColourisePalettePhaseDelta(
                next->value + removed->value);
        }
    }
    const auto previousSize = effect->effectParameterKeys.size();
    std::erase_if(
        effect->effectParameterKeys,
        [&](const TimingColouriseEffectParameterKey& key) {
            return key.parameter == parameter &&
                   std::abs(key.position - position) <=
                       kTimingColouriseKeyTolerance;
        });
    return previousSize - effect->effectParameterKeys.size();
}

std::size_t RemoveTimingColouriseEffectParameterKeysAtPosition(
    TimingColouriseEffect* effect,
    float position) {
    if (effect == nullptr) {
        return 0U;
    }
    if (!std::isfinite(position)) {
        return 0U;
    }
    const auto phaseRemoved =
        effect->colouriseEnabled
            ? RemoveTimingColouriseEffectParameterKeysAtPosition(
                  effect,
                  TimingColouriseEffectParameter::PalettePhase,
                  position)
            : 0U;
    const auto previousSize = effect->effectParameterKeys.size();
    std::erase_if(
        effect->effectParameterKeys,
        [&](const TimingColouriseEffectParameterKey& key) {
            return key.parameter !=
                       TimingColouriseEffectParameter::PalettePhase &&
                   TimingEffectParameterIsSupported(
                       *effect,
                       key.parameter) &&
                   std::abs(key.position - position) <=
                       kTimingColouriseKeyTolerance;
        });
    return phaseRemoved + previousSize -
                              effect->effectParameterKeys.size();
}

std::size_t RemoveTimingColouriseEffectKeysAtPosition(
    TimingColouriseEffect* effect,
    float position) {
    const auto removedPaletteKeys =
        effect != nullptr && effect->colouriseEnabled
            ? RemoveTimingColourisePaletteKeysAtPosition(effect, position)
            : 0U;
    return removedPaletteKeys +
           RemoveTimingColouriseEffectParameterKeysAtPosition(
               effect,
               position) +
           RemoveTimingColouriseBoundsKeysAtPosition(effect, position);
}

std::optional<float> PreviousTimingColourisePaletteKeyPosition(
    const TimingColouriseEffect& effect,
    float position) {
    if (!std::isfinite(position)) {
        return std::nullopt;
    }
    const auto clusters = PaletteKeyPositionClusters(effect);
    if (const auto current =
            PaletteKeyPositionClusterIndexAtPosition(
                clusters,
                position);
        current.has_value()) {
        if (*current == 0U) {
            return std::nullopt;
        }
        return clusters[*current - 1U].representative;
    }
    for (auto cluster = clusters.rbegin();
         cluster != clusters.rend();
         ++cluster) {
        if (cluster->maximum <
            position - kTimingColouriseKeyTolerance) {
            return cluster->representative;
        }
    }
    return std::nullopt;
}

std::optional<float> NextTimingColourisePaletteKeyPosition(
    const TimingColouriseEffect& effect,
    float position) {
    if (!std::isfinite(position)) {
        return std::nullopt;
    }
    const auto clusters = PaletteKeyPositionClusters(effect);
    if (const auto current =
            PaletteKeyPositionClusterIndexAtPosition(
                clusters,
                position);
        current.has_value()) {
        if (*current + 1U >= clusters.size()) {
            return std::nullopt;
        }
        return clusters[*current + 1U].representative;
    }
    for (const auto& cluster : clusters) {
        if (cluster.minimum >
            position + kTimingColouriseKeyTolerance) {
            return cluster.representative;
        }
    }
    return std::nullopt;
}

std::vector<float> TimingColourisePaletteKeyPositions(
    const TimingColouriseEffect& effect) {
    const auto clusters = PaletteKeyPositionClusters(effect);
    std::vector<float> positions;
    positions.reserve(clusters.size());
    for (const auto& cluster : clusters) {
        positions.push_back(cluster.representative);
    }
    return positions;
}

bool CanRemoveTimingColourisePaletteStop(
    const TimingColouriseEffect& effect,
    std::string_view stopId) {
    if (stopId.empty() || effect.basePalette.stops.size() <= 1U ||
        !effect.paletteKeys.empty()) {
        return false;
    }
    const bool stopExists = std::any_of(
        effect.basePalette.stops.begin(),
        effect.basePalette.stops.end(),
        [&](const TimingColourisePaletteStop& stop) {
            return stop.id == stopId;
        });
    const bool hasKeys = std::any_of(
        effect.paletteStopParameterKeys.begin(),
        effect.paletteStopParameterKeys.end(),
        [&](const TimingColourisePaletteStopParameterKey& key) {
            return key.stopId == stopId;
        });
    return stopExists && !hasKeys;
}

bool RemoveTimingColourisePaletteStop(
    TimingColouriseEffect* effect,
    std::string_view stopId) {
    if (effect == nullptr ||
        !CanRemoveTimingColourisePaletteStop(*effect, stopId)) {
        return false;
    }
    std::erase_if(
        effect->basePalette.stops,
        [&](const TimingColourisePaletteStop& stop) {
            return stop.id == stopId;
        });
    effect->basePalette = SanitizeTimingColourisePalette(
        std::move(effect->basePalette));
    return true;
}

std::string TimingColourisePaletteKeyStateName(
    std::string_view paletteName,
    std::size_t orderedPosition) {
    std::string base =
        paletteName.empty() ? "Palette" : std::string{paletteName};
    constexpr std::string_view editedSuffix = "_edited";
    if (base.size() >= editedSuffix.size() &&
        std::string_view{base}.substr(
            base.size() - editedSuffix.size()) == editedSuffix) {
        base.resize(base.size() - editedSuffix.size());
    }
    std::ostringstream name;
    name << base << "_Run" << std::setfill('0') << std::setw(2)
         << orderedPosition + 1U;
    return name.str();
}

std::optional<float> PreviousTimingColouriseBoundsKeyPosition(
    const TimingColouriseEffect& effect,
    float position) {
    const auto legacy = PreviousKeyPosition(effect.boundsKeys, position);
    std::optional<float> parameter;
    for (const auto& key : effect.boundsParameterKeys) {
        if (key.position < position - kTimingColouriseKeyTolerance &&
            (!parameter.has_value() || key.position > *parameter)) {
            parameter = key.position;
        }
    }
    if (!legacy.has_value()) {
        return parameter;
    }
    if (!parameter.has_value()) {
        return legacy;
    }
    return std::max(*legacy, *parameter);
}

std::optional<float> NextTimingColouriseBoundsKeyPosition(
    const TimingColouriseEffect& effect,
    float position) {
    const auto legacy = NextKeyPosition(effect.boundsKeys, position);
    std::optional<float> parameter;
    for (const auto& key : effect.boundsParameterKeys) {
        if (key.position > position + kTimingColouriseKeyTolerance &&
            (!parameter.has_value() || key.position < *parameter)) {
            parameter = key.position;
        }
    }
    if (!legacy.has_value()) {
        return parameter;
    }
    if (!parameter.has_value()) {
        return legacy;
    }
    return std::min(*legacy, *parameter);
}

std::optional<float>
PreviousTimingColouriseBoundsParameterKeyPosition(
    const TimingColouriseEffect& effect,
    TimingColouriseBoundsParameter parameter,
    float position) {
    if (!std::isfinite(position)) {
        return std::nullopt;
    }
    std::optional<float> best;
    for (const auto& key : effect.boundsParameterKeys) {
        if (key.parameter == parameter &&
            key.position < position - kTimingColouriseKeyTolerance &&
            (!best.has_value() || key.position > *best)) {
            best = key.position;
        }
    }
    return best;
}

std::optional<float>
NextTimingColouriseBoundsParameterKeyPosition(
    const TimingColouriseEffect& effect,
    TimingColouriseBoundsParameter parameter,
    float position) {
    if (!std::isfinite(position)) {
        return std::nullopt;
    }
    std::optional<float> best;
    for (const auto& key : effect.boundsParameterKeys) {
        if (key.parameter == parameter &&
            key.position > position + kTimingColouriseKeyTolerance &&
            (!best.has_value() || key.position < *best)) {
            best = key.position;
        }
    }
    return best;
}

std::optional<float>
PreviousTimingColouriseEffectParameterKeyPosition(
    const TimingColouriseEffect& effect,
    TimingColouriseEffectParameter parameter,
    float position) {
    if (!TimingEffectParameterIsSupported(effect, parameter) ||
        !std::isfinite(position)) {
        return std::nullopt;
    }
    std::optional<float> best;
    for (const auto& key : effect.effectParameterKeys) {
        if (key.parameter == parameter &&
            key.position < position - kTimingColouriseKeyTolerance &&
            (!best.has_value() || key.position > *best)) {
            best = key.position;
        }
    }
    return best;
}

std::optional<float>
NextTimingColouriseEffectParameterKeyPosition(
    const TimingColouriseEffect& effect,
    TimingColouriseEffectParameter parameter,
    float position) {
    if (!TimingEffectParameterIsSupported(effect, parameter) ||
        !std::isfinite(position)) {
        return std::nullopt;
    }
    std::optional<float> best;
    for (const auto& key : effect.effectParameterKeys) {
        if (key.parameter == parameter &&
            key.position > position + kTimingColouriseKeyTolerance &&
            (!best.has_value() || key.position < *best)) {
            best = key.position;
        }
    }
    return best;
}

std::vector<float> TimingColouriseEffectParameterKeyPositions(
    const TimingColouriseEffect& effect,
    TimingColouriseEffectParameter parameter) {
    std::vector<float> result;
    if (!TimingEffectParameterIsSupported(effect, parameter)) {
        return result;
    }
    for (const auto& key : effect.effectParameterKeys) {
        if (key.parameter == parameter) {
            result.push_back(key.position);
        }
    }
    std::stable_sort(result.begin(), result.end());
    return result;
}

std::vector<float> TimingColouriseEffectParameterKeyPositions(
    const TimingColouriseEffect& effect) {
    std::vector<float> result;
    result.reserve(effect.effectParameterKeys.size());
    for (const auto& key : effect.effectParameterKeys) {
        if (TimingEffectParameterIsSupported(
                effect,
                key.parameter)) {
            result.push_back(key.position);
        }
    }
    std::stable_sort(result.begin(), result.end());
    result.erase(
        std::unique(
            result.begin(),
            result.end(),
            [](float left, float right) {
                return std::abs(left - right) <=
                       kTimingColouriseKeyTolerance;
            }),
        result.end());
    return result;
}

std::optional<float>
PreviousTimingColouriseAnyEffectParameterKeyPosition(
    const TimingColouriseEffect& effect,
    float position) {
    if (!std::isfinite(position)) {
        return std::nullopt;
    }
    std::optional<float> best;
    for (const auto& key : effect.effectParameterKeys) {
        if (TimingEffectParameterIsSupported(
                effect,
                key.parameter) &&
            key.position < position - kTimingColouriseKeyTolerance &&
            (!best.has_value() || key.position > *best)) {
            best = key.position;
        }
    }
    return best;
}

std::optional<float>
NextTimingColouriseAnyEffectParameterKeyPosition(
    const TimingColouriseEffect& effect,
    float position) {
    if (!std::isfinite(position)) {
        return std::nullopt;
    }
    std::optional<float> best;
    for (const auto& key : effect.effectParameterKeys) {
        if (TimingEffectParameterIsSupported(
                effect,
                key.parameter) &&
            key.position > position + kTimingColouriseKeyTolerance &&
            (!best.has_value() || key.position < *best)) {
            best = key.position;
        }
    }
    return best;
}

std::optional<float> PreviousTimingColouriseEffectKeyPosition(
    const TimingColouriseEffect& effect,
    float position) {
    const auto palette = effect.colouriseEnabled
                             ? PreviousTimingColourisePaletteKeyPosition(
                                   effect,
                                   position)
                             : std::nullopt;
    const auto bounds =
        PreviousTimingColouriseBoundsKeyPosition(effect, position);
    const auto controls =
        PreviousTimingColouriseAnyEffectParameterKeyPosition(
            effect,
            position);
    std::optional<float> result = palette;
    if (bounds.has_value() &&
        (!result.has_value() || *bounds > *result)) {
        result = bounds;
    }
    if (controls.has_value() &&
        (!result.has_value() || *controls > *result)) {
        result = controls;
    }
    return result;
}

std::optional<float> NextTimingColouriseEffectKeyPosition(
    const TimingColouriseEffect& effect,
    float position) {
    const auto palette = effect.colouriseEnabled
                             ? NextTimingColourisePaletteKeyPosition(
                                   effect,
                                   position)
                             : std::nullopt;
    const auto bounds =
        NextTimingColouriseBoundsKeyPosition(effect, position);
    const auto controls =
        NextTimingColouriseAnyEffectParameterKeyPosition(
            effect,
            position);
    std::optional<float> result = palette;
    if (bounds.has_value() &&
        (!result.has_value() || *bounds < *result)) {
        result = bounds;
    }
    if (controls.has_value() &&
        (!result.has_value() || *controls < *result)) {
        result = controls;
    }
    return result;
}

const TimingTakeDefinition* FindTimingTakeDefinition(
    std::span<const TimingTakeDefinition> takes,
    std::string_view takeId) {
    const auto normalized = NormalizeTimingTakeId(takeId);
    const auto found = std::find_if(
        takes.begin(),
        takes.end(),
        [&](const TimingTakeDefinition& take) {
            return take.id == normalized;
        });
    return found == takes.end() ? nullptr : &*found;
}

TimingTakeDefinition* FindTimingTakeDefinition(
    std::vector<TimingTakeDefinition>* takes,
    std::string_view takeId) {
    if (takes == nullptr) {
        return nullptr;
    }
    return const_cast<TimingTakeDefinition*>(FindTimingTakeDefinition(
        std::span<const TimingTakeDefinition>{*takes},
        takeId));
}

const TimingTakeSceneState* FindTimingTakeSceneState(
    std::span<const TimingTakeSceneState> states,
    std::string_view takeId,
    std::string_view sceneGroupName) {
    const auto normalized = NormalizeTimingTakeId(takeId);
    const auto found = std::find_if(
        states.begin(),
        states.end(),
        [&](const TimingTakeSceneState& state) {
            return state.takeId == normalized &&
                   state.sceneGroupName == sceneGroupName;
        });
    return found == states.end() ? nullptr : &*found;
}

TimingTakeSceneState* FindTimingTakeSceneState(
    std::vector<TimingTakeSceneState>* states,
    std::string_view takeId,
    std::string_view sceneGroupName) {
    if (states == nullptr) {
        return nullptr;
    }
    return const_cast<TimingTakeSceneState*>(FindTimingTakeSceneState(
        std::span<const TimingTakeSceneState>{*states},
        takeId,
        sceneGroupName));
}

TimingTakeSceneState* EnsureTimingTakeSceneState(
    std::vector<TimingTakeSceneState>* states,
    std::string_view takeId,
    std::string_view sceneGroupName) {
    if (states == nullptr) {
        return nullptr;
    }
    if (auto* existing =
            FindTimingTakeSceneState(states, takeId, sceneGroupName)) {
        return existing;
    }
    states->push_back(SanitizeTimingTakeSceneState(TimingTakeSceneState{
        .takeId = NormalizeTimingTakeId(takeId),
        .sceneGroupName =
            sceneGroupName.empty() ? "Default" : std::string{sceneGroupName},
    }));
    return &states->back();
}

bool AssignWaterFeatureToTimingRun(
    TimingTakeSceneState* state,
    const invisible_places::water::WaterKeyedFeatureId& feature,
    std::uint32_t targetRunId) {
    if (state == nullptr) {
        return false;
    }
    invisible_places::water::WaterScenarioFeatureRuns adapter{
        .scenarioId = state->takeId,
        .runs = std::move(state->waterFeatureTimingRuns),
    };
    const bool assigned =
        invisible_places::water::AssignWaterFeatureToTimingRun(
            &adapter,
            feature,
            targetRunId);
    state->waterFeatureTimingRuns = std::move(adapter.runs);
    return assigned;
}

std::string AllocateTimingTakeId(
    std::span<const TimingTakeDefinition> takes,
    std::uint32_t* nextSequence) {
    return AllocateSequentialId(takes, nextSequence, "timing-take-");
}

std::string AllocateTimingColouriseEffectId(
    std::span<const TimingColouriseEffect> effects,
    std::uint32_t* nextSequence) {
    return AllocateSequentialId(
        effects,
        nextSequence,
        "colourise-effect-");
}

std::string AllocateTimingColourisePaletteId(
    std::span<const TimingColourisePaletteDefinition> palettes,
    std::uint32_t* nextSequence) {
    return AllocateSequentialId(
        palettes,
        nextSequence,
        "colourise-palette-");
}

}  // namespace invisible_places::timing
