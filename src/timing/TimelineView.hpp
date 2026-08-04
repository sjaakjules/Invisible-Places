#pragma once

#include <algorithm>
#include <cmath>
#include <numeric>
#include <utility>

namespace invisible_places::timing {

inline constexpr float kMinimumTimelineViewSpan = 1.0e-4F;

// A UI-only lens over normalized animation time. It never changes authored
// key positions, playback duration, or export sampling.
struct TimelineViewRange {
    float start = 0.0F;
    float end = 1.0F;
};

inline TimelineViewRange SanitizeTimelineViewRange(
    TimelineViewRange range,
    float minimumSpan = kMinimumTimelineViewSpan) noexcept {
    if (!std::isfinite(range.start)) {
        range.start = 0.0F;
    }
    if (!std::isfinite(range.end)) {
        range.end = 1.0F;
    }
    range.start = std::clamp(range.start, 0.0F, 1.0F);
    range.end = std::clamp(range.end, 0.0F, 1.0F);
    if (range.start > range.end) {
        std::swap(range.start, range.end);
    }

    const float safeMinimumSpan =
        std::isfinite(minimumSpan)
            ? std::clamp(minimumSpan, kMinimumTimelineViewSpan, 1.0F)
            : kMinimumTimelineViewSpan;
    if (range.end - range.start >= safeMinimumSpan) {
        return range;
    }

    const float midpoint = std::midpoint(range.start, range.end);
    range.start = std::clamp(
        midpoint - safeMinimumSpan * 0.5F,
        0.0F,
        1.0F - safeMinimumSpan);
    range.end = range.start + safeMinimumSpan;
    return range;
}

inline bool TimelineViewRangeIsFull(
    TimelineViewRange range,
    float tolerance = kMinimumTimelineViewSpan) noexcept {
    range = SanitizeTimelineViewRange(range);
    return range.start <= tolerance && range.end >= 1.0F - tolerance;
}

inline bool TimelinePositionIsInView(
    TimelineViewRange range,
    float position,
    float tolerance = kMinimumTimelineViewSpan) noexcept {
    if (!std::isfinite(position)) {
        return false;
    }
    range = SanitizeTimelineViewRange(range);
    return position >= range.start - tolerance &&
           position <= range.end + tolerance;
}

inline float TimelinePositionToViewFraction(
    TimelineViewRange range,
    float position) noexcept {
    range = SanitizeTimelineViewRange(range);
    return std::clamp(
        (position - range.start) / (range.end - range.start),
        0.0F,
        1.0F);
}

inline float TimelineViewFractionToPosition(
    TimelineViewRange range,
    float fraction) noexcept {
    range = SanitizeTimelineViewRange(range);
    return std::lerp(
        range.start,
        range.end,
        std::clamp(fraction, 0.0F, 1.0F));
}

}  // namespace invisible_places::timing
