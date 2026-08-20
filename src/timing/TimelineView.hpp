#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
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

// A cyclic view is intentionally a separate model from TimelineViewRange.
// Existing unlinked animation timelines retain their bounded 0..1 endpoint
// semantics, while linked timelines can use a period-2 signed circle.
struct CyclicTimelineViewDomain {
    float origin = -1.0F;
    float period = 2.0F;
};

inline constexpr CyclicTimelineViewDomain
    kLinkedSignedCyclicTimelineViewDomain{};

// `start` is canonical and `span` is a positive unwrapped distance. A short
// view from +0.9 through the signed seam to -0.9 is therefore {+0.9, 0.2},
// with an unwrapped end of +1.1.
struct CyclicTimelineViewRange {
    float start = -1.0F;
    float span = 2.0F;
};

struct CyclicTimelineViewSegment {
    float start = -1.0F;
    float end = -1.0F;
};

struct CyclicTimelineViewSegmentList {
    std::array<CyclicTimelineViewSegment, 2U> values{};
    std::size_t count = 0U;

    [[nodiscard]] const CyclicTimelineViewSegment& operator[](
        std::size_t index) const noexcept {
        return values[index];
    }

    [[nodiscard]] auto begin() const noexcept {
        return values.begin();
    }

    [[nodiscard]] auto end() const noexcept {
        return values.begin() + static_cast<std::ptrdiff_t>(count);
    }
};

inline CyclicTimelineViewDomain CyclicTimelineSanitizeDomain(
    CyclicTimelineViewDomain domain) noexcept {
    if (!std::isfinite(domain.origin)) {
        domain.origin = -1.0F;
    }
    if (!std::isfinite(domain.period) || domain.period <= 0.0F ||
        !std::isfinite(domain.origin + domain.period)) {
        domain.origin = -1.0F;
        domain.period = 2.0F;
    }
    return domain;
}

inline CyclicTimelineViewRange CyclicTimelineFullViewRange(
    CyclicTimelineViewDomain domain =
        kLinkedSignedCyclicTimelineViewDomain) noexcept {
    domain = CyclicTimelineSanitizeDomain(domain);
    return CyclicTimelineViewRange{
        .start = domain.origin,
        .span = domain.period,
    };
}

inline float CyclicTimelineViewRangeEnd(
    CyclicTimelineViewRange range) noexcept {
    return range.start + range.span;
}

// Returns the half-open representative in
// [domain.origin, domain.origin + domain.period).
inline float CyclicTimelineWrapPosition(
    float position,
    CyclicTimelineViewDomain domain =
        kLinkedSignedCyclicTimelineViewDomain) noexcept {
    domain = CyclicTimelineSanitizeDomain(domain);
    if (!std::isfinite(position)) {
        return domain.origin;
    }
    const double origin = static_cast<double>(domain.origin);
    const double period = static_cast<double>(domain.period);
    double offset = std::fmod(static_cast<double>(position) - origin, period);
    if (offset < 0.0) {
        offset += period;
    }
    if (offset >= period) {
        offset = 0.0;
    }
    return static_cast<float>(origin + offset);
}

// Lifts a position by whole periods to the copy nearest `reference`. At an
// exact half-period tie, the smallest adjustment wins. This preserves -1 and
// +1 as distinct overview handles even though they are cyclically equivalent.
inline float CyclicTimelineEquivalentPositionNear(
    float position,
    float reference,
    CyclicTimelineViewDomain domain =
        kLinkedSignedCyclicTimelineViewDomain) noexcept {
    domain = CyclicTimelineSanitizeDomain(domain);
    if (!std::isfinite(position)) {
        position = domain.origin;
    }
    if (!std::isfinite(reference)) {
        reference = domain.origin;
    }

    const double value = static_cast<double>(position);
    const double target = static_cast<double>(reference);
    const double period = static_cast<double>(domain.period);
    const double quotient = (target - value) / period;
    const double lowerCopy = std::floor(quotient);
    const double upperCopy = std::ceil(quotient);
    const double lowerValue = value + lowerCopy * period;
    const double upperValue = value + upperCopy * period;
    const double lowerDistance = std::abs(target - lowerValue);
    const double upperDistance = std::abs(target - upperValue);
    const double tieTolerance =
        std::numeric_limits<double>::epsilon() *
        std::max({1.0, std::abs(target), period}) * 8.0;

    double selected = lowerValue;
    if (upperDistance + tieTolerance < lowerDistance) {
        selected = upperValue;
    } else if (std::abs(lowerDistance - upperDistance) <= tieTolerance &&
               std::abs(upperCopy) < std::abs(lowerCopy)) {
        selected = upperValue;
    }
    return static_cast<float>(selected);
}

inline float CyclicTimelineForwardDistance(
    float from,
    float to,
    CyclicTimelineViewDomain domain =
        kLinkedSignedCyclicTimelineViewDomain) noexcept {
    domain = CyclicTimelineSanitizeDomain(domain);
    if (!std::isfinite(from) || !std::isfinite(to)) {
        return 0.0F;
    }
    const double period = static_cast<double>(domain.period);
    double distance = std::fmod(
        static_cast<double>(to) - static_cast<double>(from),
        period);
    if (distance < 0.0) {
        distance += period;
    }
    if (distance >= period) {
        distance = 0.0;
    }
    return static_cast<float>(distance);
}

inline CyclicTimelineViewRange CyclicTimelineSanitizeViewRange(
    CyclicTimelineViewRange range,
    CyclicTimelineViewDomain domain =
        kLinkedSignedCyclicTimelineViewDomain,
    float minimumSpan = kMinimumTimelineViewSpan) noexcept {
    domain = CyclicTimelineSanitizeDomain(domain);
    const float defaultMinimumSpan = std::min(
        kMinimumTimelineViewSpan,
        domain.period);
    const float safeMinimumSpan = std::isfinite(minimumSpan)
        ? std::clamp(minimumSpan, defaultMinimumSpan, domain.period)
        : defaultMinimumSpan;

    if (!std::isfinite(range.start)) {
        range.start = domain.origin;
    }
    if (!std::isfinite(range.span)) {
        range.span = domain.period;
    } else {
        range.span = std::clamp(
            range.span,
            safeMinimumSpan,
            domain.period);
    }
    if (range.span >= domain.period) {
        return CyclicTimelineFullViewRange(domain);
    }
    range.start = CyclicTimelineWrapPosition(range.start, domain);
    return range;
}

// Constructs the forward interval between two displayed endpoints. Thus
// +0.9 -> -0.9 has span 0.2, while -0.9 -> +0.9 has span 1.8. Endpoints that
// are explicitly a whole period apart construct a full view.
inline CyclicTimelineViewRange CyclicTimelineViewRangeFromEndpoints(
    float start,
    float end,
    CyclicTimelineViewDomain domain =
        kLinkedSignedCyclicTimelineViewDomain,
    float minimumSpan = kMinimumTimelineViewSpan) noexcept {
    domain = CyclicTimelineSanitizeDomain(domain);
    if (!std::isfinite(start) || !std::isfinite(end)) {
        return CyclicTimelineFullViewRange(domain);
    }
    const double rawDistance =
        static_cast<double>(end) - static_cast<double>(start);
    const float span = std::abs(rawDistance) >=
            static_cast<double>(domain.period)
        ? domain.period
        : CyclicTimelineForwardDistance(start, end, domain);
    return CyclicTimelineSanitizeViewRange(
        CyclicTimelineViewRange{
            .start = start,
            .span = span,
        },
        domain,
        minimumSpan);
}

inline bool CyclicTimelineViewRangeIsFull(
    CyclicTimelineViewRange range,
    CyclicTimelineViewDomain domain =
        kLinkedSignedCyclicTimelineViewDomain,
    float tolerance = kMinimumTimelineViewSpan) noexcept {
    domain = CyclicTimelineSanitizeDomain(domain);
    range = CyclicTimelineSanitizeViewRange(range, domain);
    const float safeTolerance = std::isfinite(tolerance)
        ? std::clamp(tolerance, 0.0F, domain.period)
        : kMinimumTimelineViewSpan;
    return range.span >= domain.period - safeTolerance;
}

inline bool CyclicTimelinePositionIsInView(
    CyclicTimelineViewRange range,
    float position,
    CyclicTimelineViewDomain domain =
        kLinkedSignedCyclicTimelineViewDomain,
    float tolerance = kMinimumTimelineViewSpan) noexcept {
    if (!std::isfinite(position)) {
        return false;
    }
    domain = CyclicTimelineSanitizeDomain(domain);
    range = CyclicTimelineSanitizeViewRange(range, domain);
    const float safeTolerance = std::isfinite(tolerance)
        ? std::max(0.0F, tolerance)
        : kMinimumTimelineViewSpan;
    if (CyclicTimelineViewRangeIsFull(range, domain, safeTolerance)) {
        return true;
    }
    const float unwrapped = CyclicTimelineEquivalentPositionNear(
        position,
        range.start + range.span * 0.5F,
        domain);
    return unwrapped >= range.start - safeTolerance &&
           unwrapped <= CyclicTimelineViewRangeEnd(range) + safeTolerance;
}

inline float CyclicTimelinePositionToViewFraction(
    CyclicTimelineViewRange range,
    float position,
    CyclicTimelineViewDomain domain =
        kLinkedSignedCyclicTimelineViewDomain) noexcept {
    domain = CyclicTimelineSanitizeDomain(domain);
    range = CyclicTimelineSanitizeViewRange(range, domain);
    if (!std::isfinite(position)) {
        return 0.0F;
    }
    const float unwrapped = CyclicTimelineEquivalentPositionNear(
        position,
        range.start + range.span * 0.5F,
        domain);
    return std::clamp(
        (unwrapped - range.start) / range.span,
        0.0F,
        1.0F);
}

// The inverse intentionally remains unwrapped so drawing and handle drags are
// continuous through a seam. Wrap only at an authored/display boundary.
inline float CyclicTimelineViewFractionToUnwrappedPosition(
    CyclicTimelineViewRange range,
    float fraction,
    CyclicTimelineViewDomain domain =
        kLinkedSignedCyclicTimelineViewDomain) noexcept {
    domain = CyclicTimelineSanitizeDomain(domain);
    range = CyclicTimelineSanitizeViewRange(range, domain);
    const float safeFraction = std::isfinite(fraction)
        ? std::clamp(fraction, 0.0F, 1.0F)
        : 0.0F;
    return range.start + range.span * safeFraction;
}

inline CyclicTimelineViewSegmentList CyclicTimelineViewRangeSegments(
    CyclicTimelineViewRange range,
    CyclicTimelineViewDomain domain =
        kLinkedSignedCyclicTimelineViewDomain) noexcept {
    domain = CyclicTimelineSanitizeDomain(domain);
    range = CyclicTimelineSanitizeViewRange(range, domain);
    CyclicTimelineViewSegmentList result;
    const float domainEnd = domain.origin + domain.period;
    if (CyclicTimelineViewRangeIsFull(range, domain)) {
        result.values[0U] = CyclicTimelineViewSegment{
            .start = domain.origin,
            .end = domainEnd,
        };
        result.count = 1U;
        return result;
    }

    const float end = CyclicTimelineViewRangeEnd(range);
    if (end <= domainEnd) {
        result.values[0U] = CyclicTimelineViewSegment{
            .start = range.start,
            .end = end,
        };
        result.count = 1U;
        return result;
    }

    result.values[0U] = CyclicTimelineViewSegment{
        .start = range.start,
        .end = domainEnd,
    };
    result.values[1U] = CyclicTimelineViewSegment{
        .start = domain.origin,
        .end = domain.origin + (end - domainEnd),
    };
    result.count = 2U;
    return result;
}

// Moves the complete lens without changing its span. Full views stay at the
// canonical reset, while narrower views wrap their start through the period.
inline CyclicTimelineViewRange CyclicTimelinePanViewRange(
    CyclicTimelineViewRange range,
    float delta,
    CyclicTimelineViewDomain domain =
        kLinkedSignedCyclicTimelineViewDomain) noexcept {
    domain = CyclicTimelineSanitizeDomain(domain);
    range = CyclicTimelineSanitizeViewRange(range, domain);
    if (!std::isfinite(delta) ||
        CyclicTimelineViewRangeIsFull(range, domain)) {
        return range;
    }
    range.start = CyclicTimelineWrapPosition(range.start + delta, domain);
    return CyclicTimelineSanitizeViewRange(range, domain);
}

}  // namespace invisible_places::timing
