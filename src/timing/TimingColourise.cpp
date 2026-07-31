#include "timing/TimingColourise.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <numeric>
#include <sstream>
#include <unordered_set>
#include <utility>

namespace invisible_places::timing {
namespace {

float FiniteOr(float value, float fallback) {
    return std::isfinite(value) ? value : fallback;
}

float Clamp01(float value) {
    return std::clamp(FiniteOr(value, 0.0F), 0.0F, 1.0F);
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
        invisible_places::water::WaterScenarioInterpolation::Smooth) {
        return amount * amount * (3.0F - 2.0F * amount);
    }
    return amount;
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
    value = FiniteOr(value, 0.0F);
    if (parameter == TimingColouriseBoundsParameter::Spread) {
        return std::max(0.0F, value);
    }
    if (parameter == TimingColouriseBoundsParameter::EdgeFade) {
        return std::clamp(value, 0.0F, 0.5F);
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
    const TimingColouriseBoundsParameterKey* first = nullptr;
    const TimingColouriseBoundsParameterKey* last = nullptr;
    const TimingColouriseBoundsParameterKey* left = nullptr;
    const TimingColouriseBoundsParameterKey* right = nullptr;
    for (const auto& key : keys) {
        if (key.parameter != parameter) {
            continue;
        }
        if (first == nullptr) {
            first = &key;
        }
        last = &key;
        if (key.position <= normalizedPosition) {
            left = &key;
        }
        if (right == nullptr && key.position >= normalizedPosition) {
            right = &key;
        }
    }
    if (first == nullptr) {
        return std::nullopt;
    }
    if (normalizedPosition <= first->position) {
        return first->value;
    }
    if (normalizedPosition >= last->position) {
        return last->value;
    }
    if (left == nullptr) {
        left = first;
    }
    if (right == nullptr) {
        right = last;
    }
    if (left == right ||
        std::abs(right->position - left->position) <=
            kTimingColouriseKeyTolerance) {
        return right->value;
    }
    const float amount = InterpolationAmount(
        left->interpolation,
        (normalizedPosition - left->position) /
            (right->position - left->position));
    return std::lerp(left->value, right->value, amount);
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
        std::clamp(FiniteOr(bounds.edgeFade, 0.0F), 0.0F, 0.5F);
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

TimingColouriseEffect SanitizeTimingColouriseEffect(
    TimingColouriseEffect effect) {
    if (effect.name.empty()) {
        effect.name = "Colourise";
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
    for (auto& key : effect.paletteKeys) {
        key.position = Clamp01(key.position);
        key.palette =
            SanitizeTimingColourisePalette(std::move(key.palette));
    }
    std::unordered_set<std::string> stopIds;
    stopIds.reserve(effect.basePalette.stops.size());
    for (const auto& stop : effect.basePalette.stops) {
        stopIds.insert(stop.id);
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
    }
    for (auto& key : effect.boundsKeys) {
        key.position = Clamp01(key.position);
        key.bounds = SanitizeTimingColouriseBounds(key.bounds);
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
    }
    SortAndCoalesceKeys(&effect.paletteKeys);
    SortAndCoalescePaletteStopParameterKeys(
        &effect.paletteStopParameterKeys);
    SortAndCoalesceKeys(&effect.boundsKeys);
    SortAndCoalesceBoundsParameterKeys(&effect.boundsParameterKeys);
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

TimingTakeDefinition SanitizeTimingTakeDefinition(
    TimingTakeDefinition definition) {
    definition.id = NormalizeTimingTakeId(definition.id);
    if (definition.id == kAuthoredTimingTakeId) {
        definition.name = std::string{kAuthoredTimingTakeName};
    } else if (definition.name.empty()) {
        definition.name = "Timing Take";
    }
    return definition;
}

TimingTakeSceneState SanitizeTimingTakeSceneState(
    TimingTakeSceneState state) {
    state.takeId = NormalizeTimingTakeId(state.takeId);
    if (state.sceneGroupName.empty()) {
        state.sceneGroupName = "Default";
    }
    if (state.colouriseEffects.size() > kMaximumTimingColouriseEffects) {
        state.colouriseEffects.resize(kMaximumTimingColouriseEffects);
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

TimingColourisePalette EvaluateTimingColourisePalette(
    const TimingColouriseEffect& effect,
    float normalizedPosition) {
    const auto sanitized = SanitizeTimingColouriseEffect(effect);
    // A legacy snapshot track is evaluated directly as LUTs by
    // EvaluateTimingColourisePaletteLut. There is no single stop topology
    // capable of representing its sample-wise interpolation, so this helper
    // intentionally exposes its authored base palette only.
    if (sanitized.paletteKeyModel ==
        TimingColourisePaletteKeyModel::LegacySnapshots) {
        return sanitized.basePalette;
    }
    normalizedPosition = Clamp01(normalizedPosition);
    auto palette = sanitized.basePalette;
    for (auto& stop : palette.stops) {
        stop.position = EvaluatePaletteStopScalarTrack(
                            sanitized.paletteStopParameterKeys,
                            stop.id,
                            TimingColourisePaletteStopParameter::Position,
                            normalizedPosition)
                            .value_or(stop.position);
        stop.colour = EvaluatePaletteStopColourTrack(
                          sanitized.paletteStopParameterKeys,
                          stop.id,
                          normalizedPosition)
                          .value_or(stop.colour);
        stop.colouriseAmount =
            EvaluatePaletteStopScalarTrack(
                sanitized.paletteStopParameterKeys,
                stop.id,
                TimingColourisePaletteStopParameter::ColouriseAmount,
                normalizedPosition)
                .value_or(stop.colouriseAmount);
    }
    return SanitizeTimingColourisePalette(std::move(palette));
}

TimingColouriseLut EvaluateTimingColourisePaletteLut(
    const TimingColouriseEffect& effect,
    float normalizedPosition) {
    const auto sanitized = SanitizeTimingColouriseEffect(effect);
    if (sanitized.paletteKeyModel ==
        TimingColourisePaletteKeyModel::StopParameters) {
        return CompileTimingColourisePaletteLut(
            EvaluateTimingColourisePalette(
                sanitized,
                normalizedPosition));
    }
    if (sanitized.paletteKeys.empty()) {
        return CompileTimingColourisePaletteLut(sanitized.basePalette);
    }
    normalizedPosition = Clamp01(normalizedPosition);
    const auto [left, right] =
        BracketingKeys(sanitized.paletteKeys, normalizedPosition);
    auto result = CompileTimingColourisePaletteLut(left->palette);
    if (left == right) {
        return result;
    }
    const float span =
        std::max(1.0e-6F, right->position - left->position);
    const float amount = InterpolationAmount(
        left->interpolation,
        (normalizedPosition - left->position) / span);
    if (amount <= 0.0F) {
        return result;
    }
    const auto rightLut = CompileTimingColourisePaletteLut(right->palette);
    for (std::size_t index = 0U; index < result.size(); ++index) {
        for (std::size_t channel = 0U; channel < 4U; ++channel) {
            result[index][channel] =
                std::lerp(result[index][channel], rightLut[index][channel], amount);
        }
    }
    return result;
}

TimingColouriseBounds EvaluateTimingColouriseBounds(
    const TimingColouriseEffect& effect,
    float normalizedPosition) {
    const auto sanitized = SanitizeTimingColouriseEffect(effect);
    normalizedPosition = Clamp01(normalizedPosition);
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
    if (span <= std::numeric_limits<float>::epsilon() ||
        fieldValue < sanitized.lower || fieldValue > sanitized.upper) {
        return 0.0F;
    }
    const float fadeWidth = span * sanitized.edgeFade;
    if (fadeWidth <= std::numeric_limits<float>::epsilon()) {
        return 1.0F;
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
        interpolation,
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
        .interpolation = interpolation,
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
        .interpolation = interpolation,
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
        interpolation,
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
                .interpolation = legacy.interpolation,
            });
        }
    }
    TimingColouriseBoundsParameterKey key{
        .parameter = parameter,
        .position = Clamp01(position),
        .value = SanitizeBoundsParameterValue(parameter, value),
        .interpolation = interpolation,
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
    for (auto& key : effect->paletteKeys) {
        if (std::abs(key.position - sourcePosition) <=
            kTimingColouriseKeyTolerance) {
            key.position = destinationPosition;
        }
    }
    for (auto& key : effect->paletteStopParameterKeys) {
        if (std::abs(key.position - sourcePosition) <=
            kTimingColouriseKeyTolerance) {
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
    const bool hasLegacySource =
        KeyCountAtPosition(effect.paletteKeys, sourcePosition) != 0U;
    const bool hasParameterSource = std::any_of(
        effect.paletteStopParameterKeys.begin(),
        effect.paletteStopParameterKeys.end(),
        [&](const TimingColourisePaletteStopParameterKey& key) {
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
    const bool hasLegacyDestination =
        KeyCountAtPosition(effect.paletteKeys, destinationPosition) != 0U;
    if (hasLegacyDestination) {
        // A legacy snapshot owns every stop property at its position.
        return false;
    }
    if (hasLegacySource &&
        std::any_of(
            effect.paletteStopParameterKeys.begin(),
            effect.paletteStopParameterKeys.end(),
            [&](const TimingColourisePaletteStopParameterKey& key) {
                return std::abs(key.position - destinationPosition) <=
                       kTimingColouriseKeyTolerance;
            })) {
        return false;
    }
    for (const auto& source : effect.paletteStopParameterKeys) {
        if (std::abs(source.position - sourcePosition) >
            kTimingColouriseKeyTolerance) {
            continue;
        }
        const bool sameTrackOccupied = std::any_of(
            effect.paletteStopParameterKeys.begin(),
            effect.paletteStopParameterKeys.end(),
            [&](const TimingColourisePaletteStopParameterKey& candidate) {
                return candidate.stopId == source.stopId &&
                       candidate.parameter == source.parameter &&
                       std::abs(candidate.position - sourcePosition) >
                           kTimingColouriseKeyTolerance &&
                       std::abs(
                           candidate.position - destinationPosition) <=
                           kTimingColouriseKeyTolerance;
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

bool MoveTimingColouriseBoundsParameterKey(
    TimingColouriseEffect* effect,
    TimingColouriseBoundsParameter parameter,
    float sourcePosition,
    float destinationPosition) {
    if (effect == nullptr ||
        !TimingColouriseBoundsParameterIsAllowed(
            effect->boundsKeyMode,
            parameter)) {
        return false;
    }
    const auto source = std::find_if(
        effect->boundsParameterKeys.begin(),
        effect->boundsParameterKeys.end(),
        [&](const TimingColouriseBoundsParameterKey& key) {
            return key.parameter == parameter &&
                   std::abs(key.position - sourcePosition) <=
                       kTimingColouriseKeyTolerance;
        });
    if (source == effect->boundsParameterKeys.end() ||
        !std::isfinite(destinationPosition) ||
        destinationPosition < 0.0F || destinationPosition > 1.0F) {
        return false;
    }
    const bool occupied = std::any_of(
        effect->boundsParameterKeys.begin(),
        effect->boundsParameterKeys.end(),
        [&](const TimingColouriseBoundsParameterKey& key) {
            return &key != &*source && key.parameter == parameter &&
                   std::abs(key.position - destinationPosition) <=
                       kTimingColouriseKeyTolerance;
        });
    if (occupied) {
        return false;
    }
    source->position = destinationPosition;
    SortAndCoalesceBoundsParameterKeys(&effect->boundsParameterKeys);
    return true;
}

std::size_t TimingColourisePaletteKeyCountAtPosition(
    const TimingColouriseEffect& effect,
    float position) {
    if (!std::isfinite(position)) {
        return 0U;
    }
    return KeyCountAtPosition(effect.paletteKeys, position) +
           static_cast<std::size_t>(std::count_if(
               effect.paletteStopParameterKeys.begin(),
               effect.paletteStopParameterKeys.end(),
               [&](const TimingColourisePaletteStopParameterKey& key) {
                   return std::abs(key.position - position) <=
                          kTimingColouriseKeyTolerance;
               }));
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
    return TimingColourisePaletteKeyCountAtPosition(effect, position) +
           TimingColouriseBoundsKeyCountAtPosition(effect, position);
}

std::size_t RemoveTimingColourisePaletteKeysAtPosition(
    TimingColouriseEffect* effect,
    float position) {
    if (effect == nullptr) {
        return 0U;
    }
    const auto legacy =
        RemoveKeysAtPosition(&effect->paletteKeys, position);
    if (!std::isfinite(position)) {
        return legacy;
    }
    const auto previousSize =
        effect->paletteStopParameterKeys.size();
    std::erase_if(
        effect->paletteStopParameterKeys,
        [&](const TimingColourisePaletteStopParameterKey& key) {
            return std::abs(key.position - position) <=
                   kTimingColouriseKeyTolerance;
        });
    return legacy + previousSize -
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

std::size_t RemoveTimingColouriseEffectKeysAtPosition(
    TimingColouriseEffect* effect,
    float position) {
    return RemoveTimingColourisePaletteKeysAtPosition(effect, position) +
           RemoveTimingColouriseBoundsKeysAtPosition(effect, position);
}

std::optional<float> PreviousTimingColourisePaletteKeyPosition(
    const TimingColouriseEffect& effect,
    float position) {
    if (!std::isfinite(position)) {
        return std::nullopt;
    }
    std::optional<float> best =
        PreviousKeyPosition(effect.paletteKeys, position);
    for (const auto& key : effect.paletteStopParameterKeys) {
        if (key.position < position - kTimingColouriseKeyTolerance &&
            (!best.has_value() || key.position > best.value())) {
            best = key.position;
        }
    }
    return best;
}

std::optional<float> NextTimingColourisePaletteKeyPosition(
    const TimingColouriseEffect& effect,
    float position) {
    if (!std::isfinite(position)) {
        return std::nullopt;
    }
    std::optional<float> best =
        NextKeyPosition(effect.paletteKeys, position);
    for (const auto& key : effect.paletteStopParameterKeys) {
        if (key.position > position + kTimingColouriseKeyTolerance &&
            (!best.has_value() || key.position < best.value())) {
            best = key.position;
        }
    }
    return best;
}

std::vector<float> TimingColourisePaletteKeyPositions(
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
    std::vector<float> unique;
    unique.reserve(positions.size());
    for (const float position : positions) {
        if (unique.empty() ||
            std::abs(unique.back() - position) >
                kTimingColouriseKeyTolerance) {
            unique.push_back(position);
        }
    }
    return unique;
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

std::optional<float> PreviousTimingColouriseEffectKeyPosition(
    const TimingColouriseEffect& effect,
    float position) {
    const auto palette =
        PreviousTimingColourisePaletteKeyPosition(effect, position);
    const auto bounds =
        PreviousTimingColouriseBoundsKeyPosition(effect, position);
    if (!palette.has_value()) {
        return bounds;
    }
    if (!bounds.has_value()) {
        return palette;
    }
    return std::max(*palette, *bounds);
}

std::optional<float> NextTimingColouriseEffectKeyPosition(
    const TimingColouriseEffect& effect,
    float position) {
    const auto palette =
        NextTimingColourisePaletteKeyPosition(effect, position);
    const auto bounds =
        NextTimingColouriseBoundsKeyPosition(effect, position);
    if (!palette.has_value()) {
        return bounds;
    }
    if (!bounds.has_value()) {
        return palette;
    }
    return std::min(*palette, *bounds);
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
