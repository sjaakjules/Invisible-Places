#include "timing/TimingColourise.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <iomanip>
#include <limits>
#include <numeric>
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

float FiniteOr(float value, float fallback) {
    return std::isfinite(value) ? value : fallback;
}

float Clamp01(float value) {
    return std::clamp(FiniteOr(value, 0.0F), 0.0F, 1.0F);
}

float ClampPalettePhaseDelta(float value) {
    return std::clamp(FiniteOr(value, 0.0F), -1.0F, 1.0F);
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
        case WaterScenarioInterpolation::SmoothVelocity:
        case WaterScenarioInterpolation::CentripetalCatmullRom:
            return true;
        case WaterScenarioInterpolation::Linear:
        case WaterScenarioInterpolation::Hold:
        case WaterScenarioInterpolation::SplineHandles:
        // Setting-track-only sentinel; colourise keys never carry it.
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
        case TimingColouriseBoundsParameter::EdgeFade:
            return true;
    }
    return false;
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
            return true;
    }
    return false;
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
            return std::max(0.0F, FiniteOr(value, 1.0F));
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

struct PaletteStopParameterBrackets {
    const TimingColourisePaletteStopParameterKey* first = nullptr;
    const TimingColourisePaletteStopParameterKey* last = nullptr;
    const TimingColourisePaletteStopParameterKey* left = nullptr;
    const TimingColourisePaletteStopParameterKey* right = nullptr;
};

PaletteStopParameterBrackets PaletteStopParameterKeysAround(
    const std::vector<TimingColourisePaletteStopParameterKey>& keys,
    std::string_view stopId,
    TimingColourisePaletteStopParameter parameter,
    float normalizedPosition) {
    PaletteStopParameterBrackets result;
    for (const auto& key : keys) {
        if (key.stopId != stopId || key.parameter != parameter) {
            continue;
        }
        if (result.first == nullptr) {
            result.first = &key;
        }
        result.last = &key;
        if (key.position <= normalizedPosition) {
            result.left = &key;
        }
        if (result.right == nullptr && key.position >= normalizedPosition) {
            result.right = &key;
        }
    }
    if (result.first == nullptr) {
        return result;
    }
    if (normalizedPosition <= result.first->position) {
        result.left = result.first;
        result.right = result.first;
    } else if (normalizedPosition >= result.last->position) {
        result.left = result.last;
        result.right = result.last;
    } else {
        if (result.left == nullptr) {
            result.left = result.first;
        }
        if (result.right == nullptr) {
            result.right = result.last;
        }
    }
    return result;
}

std::optional<float> EvaluatePaletteStopScalarTrack(
    const std::vector<TimingColourisePaletteStopParameterKey>& keys,
    std::string_view stopId,
    TimingColourisePaletteStopParameter parameter,
    float normalizedPosition) {
    const auto brackets = PaletteStopParameterKeysAround(
        keys,
        stopId,
        parameter,
        normalizedPosition);
    if (brackets.first == nullptr) {
        return std::nullopt;
    }
    if (brackets.left == brackets.right ||
        std::abs(brackets.right->position - brackets.left->position) <=
            kTimingColouriseKeyTolerance) {
        return brackets.right->scalarValue;
    }
    const float amount = InterpolationAmount(
        brackets.left->interpolation,
        (normalizedPosition - brackets.left->position) /
            (brackets.right->position - brackets.left->position));
    return std::lerp(
        brackets.left->scalarValue,
        brackets.right->scalarValue,
        amount);
}

std::optional<std::array<float, 3>> EvaluatePaletteStopColourTrack(
    const std::vector<TimingColourisePaletteStopParameterKey>& keys,
    std::string_view stopId,
    float normalizedPosition) {
    const auto brackets = PaletteStopParameterKeysAround(
        keys,
        stopId,
        TimingColourisePaletteStopParameter::Colour,
        normalizedPosition);
    if (brackets.first == nullptr) {
        return std::nullopt;
    }
    if (brackets.left == brackets.right ||
        std::abs(brackets.right->position - brackets.left->position) <=
            kTimingColouriseKeyTolerance) {
        return brackets.right->colourValue;
    }
    const float amount = InterpolationAmount(
        brackets.left->interpolation,
        (normalizedPosition - brackets.left->position) /
            (brackets.right->position - brackets.left->position));
    std::array<float, 3> colour{};
    for (std::size_t channel = 0U; channel < colour.size(); ++channel) {
        colour[channel] = std::lerp(
            brackets.left->colourValue[channel],
            brackets.right->colourValue[channel],
            amount);
    }
    return colour;
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
        .edgeFade =
            std::lerp(left->bounds.edgeFade, right->bounds.edgeFade, amount),
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
    const auto baseNameKey = RainProfileNameKey(base.name);
    auto existing = std::find_if(
        profiles->begin(),
        profiles->end(),
        [&](const invisible_places::water::WaterRainProfile& profile) {
            if (!profile.objectOverride ||
                profile.ownerTimingTakeId != take->id) {
                return false;
            }
            return (!base.id.empty() &&
                    profile.baseProfileId == base.id) ||
                   (profile.baseProfileId.empty() &&
                    RainProfileNameKey(profile.baseProfileName) ==
                        baseNameKey);
        });
    if (existing == profiles->end()) {
        invisible_places::water::WaterRainProfile copy;
        copy.id = invisible_places::water::AllocateWaterRainProfileId(
            *profiles,
            base.id + "-" + take->id);
        copy.name = UniqueRainProfileName(
            *profiles,
            TimingTakeRainOwnerProfileName(base, *take));
        copy.objectOverride = true;
        copy.ownerTimingTakeId = take->id;
        copy.baseProfileId = base.id;
        copy.baseProfileName = base.name;
        profiles->push_back(std::move(copy));
        existing = profiles->end() - 1;
    }
    existing->settings = settings;
    existing->visual = visual;
    existing->objectOverride = true;
    existing->ownerTimingTakeId = take->id;
    existing->baseProfileId = base.id;
    existing->baseProfileName = base.name;
    SetTimingTakeRainAssignment(take, *existing, base);
    return &*existing;
}

invisible_places::water::WaterRainProfile*
SaveTimingTakeRainOwnerProfileAsShared(
    std::vector<invisible_places::water::WaterRainProfile>* profiles,
    std::vector<TimingTakeDefinition>* takes,
    std::string_view takeId,
    std::string_view requestedName,
    bool overwriteExisting) {
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
    const auto savedSettings = effective->settings;
    const auto savedVisual = effective->visual;
    const auto promotedOwnerId =
        effective->objectOverride &&
                effective->ownerTimingTakeId == take->id
            ? effective->id
            : std::string{};
    const auto promotedOwnerName =
        promotedOwnerId.empty() ? std::string{} : effective->name;

    auto preferredName = TrimRainProfileText(requestedName);
    if (preferredName.empty()) {
        preferredName = effective->objectOverride
                            ? effective->baseProfileName
                            : effective->name;
    }
    auto* existingByName =
        invisible_places::water::FindWaterRainProfileByName(
            profiles,
            preferredName);
    invisible_places::water::WaterRainProfile* saved = nullptr;
    if (overwriteExisting && existingByName != nullptr &&
        !existingByName->objectOverride) {
        saved = existingByName;
    } else {
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
                      WaterScenarioInterpolation::Smooth;
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
    bounds.edgeFade =
        std::clamp(FiniteOr(bounds.edgeFade, 0.10F), -0.5F, 0.5F);
    return bounds;
}

bool TimingColouriseBoundsParameterIsAllowed(
    TimingColouriseBoundsKeyMode mode,
    TimingColouriseBoundsParameter parameter) {
    if (!IsValidBoundsKeyMode(mode) ||
        !IsValidBoundsParameter(parameter)) {
        return false;
    }
    if (parameter == TimingColouriseBoundsParameter::EdgeFade) {
        return true;
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
            return sanitized.edgeFade;
    }
    return 0.0F;
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
            return colouriseEnabled;
        case TimingColouriseEffectParameter::EmissiveLevel:
            return emissiveEnabled;
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
        if (edit.presetId.empty()) {
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
                return candidate.presetId == edit.presetId;
            });
        if (existing == localPaletteEdits.end()) {
            localPaletteEdits.push_back(std::move(edit));
        } else {
            *existing = std::move(edit);
        }
    }
    effect.localPaletteEdits = std::move(localPaletteEdits);
    // Projects authored before effect-local variants stored an active
    // Preset_edited palette only in basePalette. A non-empty preset id makes
    // that provenance safe to synthesize, and also keeps direct active edits
    // and Flip Palette synchronized with their private snapshot.
    if (effect.paletteSourceKind ==
            TimingColourisePaletteSourceKind::Preset &&
        effect.paletteEdited && !effect.paletteSourceId.empty()) {
        if (effect.paletteSourceName.empty()) {
            effect.paletteSourceName = effect.paletteSourceId;
        }
        auto activeEdit = std::find_if(
            effect.localPaletteEdits.begin(),
            effect.localPaletteEdits.end(),
            [&](const TimingColouriseLocalPaletteEdit& candidate) {
                return candidate.presetId == effect.paletteSourceId;
            });
        if (activeEdit == effect.localPaletteEdits.end()) {
            effect.localPaletteEdits.push_back(
                TimingColouriseLocalPaletteEdit{
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
    effect.colouriseAmountOverride = std::clamp(
        FiniteOr(effect.colouriseAmountOverride, 1.0F),
        0.0F,
        1.0F);
    effect.palettePhaseOffset =
        FiniteOr(effect.palettePhaseOffset, 0.0F);
    effect.emissiveLevel = std::max(
        0.0F,
        FiniteOr(effect.emissiveLevel, 1.0F));
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
    std::string_view presetId) {
    const auto found = std::find_if(
        effect.localPaletteEdits.begin(),
        effect.localPaletteEdits.end(),
        [&](const TimingColouriseLocalPaletteEdit& edit) {
            return edit.presetId == presetId;
        });
    return found == effect.localPaletteEdits.end() ? nullptr : &*found;
}

bool UpsertTimingColouriseLocalPaletteEdit(
    TimingColouriseEffect* effect,
    TimingColourisePalette palette) {
    if (effect == nullptr) {
        return false;
    }
    if (effect->paletteSourceKind !=
            TimingColourisePaletteSourceKind::Preset ||
        effect->paletteSourceId.empty()) {
        return false;
    }
    palette = SanitizeTimingColourisePalette(std::move(palette));
    const auto existing = std::find_if(
        effect->localPaletteEdits.begin(),
        effect->localPaletteEdits.end(),
        [&](const TimingColouriseLocalPaletteEdit& edit) {
            return edit.presetId == effect->paletteSourceId;
        });
    TimingColouriseLocalPaletteEdit localEdit{
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

bool ActivateTimingColouriseOriginalPreset(
    TimingColouriseEffect* effect,
    const TimingColourisePaletteDefinition& preset) {
    if (effect == nullptr || preset.id.empty()) {
        return false;
    }
    auto updated = SanitizeTimingColouriseEffect(*effect);
    const auto sanitizedPreset =
        SanitizeTimingColourisePaletteDefinition(preset);
    updated.basePalette = sanitizedPreset.palette;
    updated.paletteSourceKind = TimingColourisePaletteSourceKind::Preset;
    updated.paletteSourceId = sanitizedPreset.id;
    updated.paletteSourceName = sanitizedPreset.name;
    updated.paletteEdited = false;
    *effect = SanitizeTimingColouriseEffect(std::move(updated));
    return true;
}

bool ActivateTimingColouriseLocalPaletteEdit(
    TimingColouriseEffect* effect,
    std::string_view presetId) {
    if (effect == nullptr || presetId.empty()) {
        return false;
    }
    auto updated = SanitizeTimingColouriseEffect(*effect);
    const auto localEdit = std::find_if(
        updated.localPaletteEdits.begin(),
        updated.localPaletteEdits.end(),
        [&](const TimingColouriseLocalPaletteEdit& edit) {
            return edit.presetId == presetId;
        });
    if (localEdit == updated.localPaletteEdits.end()) {
        return false;
    }
    updated.basePalette = localEdit->palette;
    updated.paletteSourceKind = TimingColourisePaletteSourceKind::Preset;
    updated.paletteSourceId = localEdit->presetId;
    updated.paletteSourceName = localEdit->presetName;
    updated.paletteEdited = true;
    *effect = SanitizeTimingColouriseEffect(std::move(updated));
    return true;
}

bool DiscardTimingColouriseLocalPaletteEdit(
    TimingColouriseEffect* effect,
    const TimingColourisePaletteDefinition& originalPreset) {
    if (effect == nullptr || originalPreset.id.empty()) {
        return false;
    }
    auto updated = SanitizeTimingColouriseEffect(*effect);
    const auto localEdit = std::find_if(
        updated.localPaletteEdits.begin(),
        updated.localPaletteEdits.end(),
        [&](const TimingColouriseLocalPaletteEdit& edit) {
            return edit.presetId == originalPreset.id;
        });
    if (localEdit == updated.localPaletteEdits.end()) {
        return false;
    }
    const bool discardingActiveEdit =
        updated.paletteSourceKind ==
            TimingColourisePaletteSourceKind::Preset &&
        updated.paletteEdited &&
        updated.paletteSourceId == originalPreset.id;
    updated.localPaletteEdits.erase(localEdit);
    if (discardingActiveEdit) {
        const auto sanitizedPreset =
            SanitizeTimingColourisePaletteDefinition(originalPreset);
        updated.basePalette = sanitizedPreset.palette;
        updated.paletteSourceId = sanitizedPreset.id;
        updated.paletteSourceName = sanitizedPreset.name;
        updated.paletteEdited = false;
    }
    *effect = SanitizeTimingColouriseEffect(std::move(updated));
    return true;
}

std::optional<TimingColourisePaletteDefinition>
PromoteTimingColouriseLocalPaletteEdit(
    TimingColouriseEffect* effect,
    std::string_view presetId,
    std::string savedPaletteId,
    std::string savedPaletteName) {
    if (effect == nullptr || presetId.empty() ||
        savedPaletteId.empty()) {
        return std::nullopt;
    }
    auto updated = SanitizeTimingColouriseEffect(*effect);
    const auto localEdit = std::find_if(
        updated.localPaletteEdits.begin(),
        updated.localPaletteEdits.end(),
        [&](const TimingColouriseLocalPaletteEdit& edit) {
            return edit.presetId == presetId;
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

TimingTakeDefinition SanitizeTimingTakeDefinition(
    TimingTakeDefinition definition) {
    definition.id = NormalizeTimingTakeId(definition.id);
    if (definition.id == kAuthoredTimingTakeId) {
        definition.name = std::string{kAuthoredTimingTakeName};
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
        for (auto& feature : run.features) {
            for (auto& setting : feature.settings) {
                retimeKeys(&setting.keys);
            }
            // Settings clips are spans over the same normalized domain, so
            // they retime exactly with the keys they group.
            for (auto& clip : feature.clips) {
                clip.start = retimePosition(clip.start);
                clip.end = retimePosition(clip.end);
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
            firstOwnerRun->enabled =
                firstOwnerRun->enabled || run.enabled;
            run.features.erase(
                run.features.begin() +
                static_cast<std::ptrdiff_t>(featureIndex));
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
        for (const auto& sourceFeature : sourceRun.features) {
            auto [owningRun, destinationFeature] =
                findFeature(sourceFeature.feature);
            if (destinationFeature != nullptr) {
                owningRun->enabled = owningRun->enabled || sourceRun.enabled;
                MergeWaterFeatureTimelineKeepingFirst(
                    destinationFeature,
                    sourceFeature);
                continue;
            }
            invisible_places::water::WaterFeatureTimeline merged{
                .feature = sourceFeature.feature};
            MergeWaterFeatureTimelineKeepingFirst(
                &merged,
                sourceFeature);
            targetRun->features.push_back(std::move(merged));
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
    entry->adoptedGlobalRevision = effect->boundsAdoptedGlobalRevision;
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
                                  x.bounds.edgeFade == y.bounds.edgeFade &&
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
                   a.baseBounds.edgeFade == b.baseBounds.edgeFade &&
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

float WrapTimingLoopPosition(float position) {
    position = FiniteOr(position, 0.0F);
    position -= std::floor(position);
    return position >= 1.0F ? 0.0F : position;
}

template <typename Key, typename SameTrack, typename TrackLess>
void ExpandTimingKeysForCyclicEvaluation(
    std::vector<Key>* keys,
    SameTrack sameTrack,
    TrackLess trackLess) {
    if (keys == nullptr || keys->empty()) {
        return;
    }
    for (auto& key : *keys) {
        key.position = WrapTimingLoopPosition(key.position);
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
            std::abs(canonical.back().position - key.position) <=
                kTimingColouriseKeyTolerance) {
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
                          normalizedPosition)
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
        ? WrapTimingLoopPosition(normalizedPosition)
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

TimingColourisePalette EvaluateTimingColourisePalette(
    const TimingColouriseEffect& effect,
    float normalizedPosition,
    bool cyclic) {
    const auto prepared = PrepareTimingColouriseEffectForEvaluation(
        effect,
        cyclic);
    normalizedPosition = cyclic
        ? WrapTimingLoopPosition(normalizedPosition)
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
        ? WrapTimingLoopPosition(normalizedPosition)
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
    const auto finalize = [&](TimingColouriseLut lut) {
        return ApplyTimingColouriseAmountOverride(
            ApplyTimingColourisePalettePhase(lut, phaseOffset),
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
        ? WrapTimingLoopPosition(normalizedPosition)
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
        .edgeFade = value(TimingColouriseBoundsParameter::EdgeFade),
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
    if (!std::isfinite(fieldValue)) {
        return 0.0F;
    }
    const auto sanitized = SanitizeTimingColouriseBounds(bounds);
    const float span = sanitized.upper - sanitized.lower;
    if (span <= std::numeric_limits<float>::epsilon()) {
        return 0.0F;
    }
    const float fadeWidth = span * std::abs(sanitized.edgeFade);
    if (fadeWidth <= std::numeric_limits<float>::epsilon()) {
        return fieldValue >= sanitized.lower &&
                       fieldValue <= sanitized.upper
                   ? 1.0F
                   : 0.0F;
    }
    if (sanitized.edgeFade < 0.0F) {
        if (fieldValue < sanitized.lower - fadeWidth ||
            fieldValue > sanitized.upper + fadeWidth) {
            return 0.0F;
        }
        const float lowerAmount = Clamp01(
            (fieldValue - (sanitized.lower - fadeWidth)) / fadeWidth);
        const float upperAmount = Clamp01(
            ((sanitized.upper + fadeWidth) - fieldValue) / fadeWidth);
        return std::min(lowerAmount, upperAmount);
    }
    if (fieldValue < sanitized.lower || fieldValue > sanitized.upper) {
        return 0.0F;
    }
    const float lowerAmount =
        Clamp01((fieldValue - sanitized.lower) / fadeWidth);
    const float upperAmount =
        Clamp01((sanitized.upper - fieldValue) / fadeWidth);
    return std::min(lowerAmount, upperAmount);
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
            const float joinedDelta = next->value + removed->value;
            if (joinedDelta >= -1.0F && joinedDelta <= 1.0F) {
                next->value = ClampPalettePhaseDelta(joinedDelta);
            }
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
