#include "rain_impact_rock_model.glsl"

struct RainImpactEventGpu {
    vec4 positionBirth;
    vec4 normalRadius;
    vec4 lifetimeEnergy;
    uvec4 control;
};

struct RainImpactComposite {
    float opacityAdd;
    float emissionAdd;
    float pointSizeMultiply;
    float colourMix;
};

layout(set = 0, binding = 13, std430) readonly buffer RainImpactCountsBuffer {
    uvec4 rainImpactCounts[];
};

layout(set = 0, binding = 14, std430) readonly buffer RainImpactReferencesBuffer {
    uint rainImpactReferences[];
};

layout(set = 0, binding = 15, std430) readonly buffer RainImpactEventsBuffer {
    RainImpactEventGpu rainImpactEvents[];
};

const uint kRainRoleRock = 1u;
const uint kRainRoleSand = 2u;
const uint kRainRoleVegetation = 3u;
const uint kRainReferencesPerCell = 20u;

uint RainImpactHashBits(uint value) {
    value ^= value >> 16u;
    value *= 0x7feb352du;
    value ^= value >> 15u;
    value *= 0x846ca68bu;
    return value ^ (value >> 16u);
}

float RainImpactRandom01(uint value) {
    return float(RainImpactHashBits(value) & 0x00ffffffu) / float(0x01000000u);
}

uint RainImpactHashCell(ivec3 coordinate) {
    uint hash = RainImpactHashBits(uint(coordinate.x));
    const uint y = RainImpactHashBits(uint(coordinate.y));
    const uint z = RainImpactHashBits(uint(coordinate.z));
    hash ^= (y << 11u) | (y >> 21u);
    hash ^= (z << 22u) | (z >> 10u);
    return RainImpactHashBits(hash);
}

vec2 RainImpactRandomDirection2D(uint seed) {
    const vec2 direction = vec2(
        RainImpactRandom01(seed + 0x68bc21ebu),
        RainImpactRandom01(seed + 0x02e5be93u)) * 2.0 - 1.0;
    return dot(direction, direction) > 1e-8 ? normalize(direction) : vec2(1.0, 0.0);
}

vec2 RainVegetationAnchorOffset(
    uint streamSeed,
    vec2 branchDirection,
    uint bandIndex,
    float hopSpacing,
    float maximumDepth,
    float eventRadius,
    float streamSpread,
    float maximumPathOffset) {
    const float anchorDepth = float(bandIndex) * hopSpacing;
    const float anchorDepthFraction = clamp(anchorDepth / maximumDepth, 0.0, 1.0);
    const float branchScale = eventRadius * streamSpread *
        (0.06 + 0.76 * anchorDepthFraction) *
        (0.65 + 0.35 * RainImpactRandom01(streamSeed + 5u));
    const vec2 wander = RainImpactRandomDirection2D(
        streamSeed ^ RainImpactHashBits(bandIndex + 0xa511e9b3u));
    const float wanderScale = eventRadius * 0.16 *
        smoothstep(0.0, hopSpacing * 2.0, anchorDepth);
    vec2 result = branchDirection * branchScale + wander * wanderScale;
    const float offsetLength = length(result);
    if (offsetLength > maximumPathOffset && offsetLength > 1e-7) {
        result *= maximumPathOffset / offsetLength;
    }
    return result;
}

float RainVegetationSprinkleValue(
    RainImpactEventGpu event,
    vec3 point,
    vec3 pointNormal,
    float age) {
    if (point.z > event.positionBirth.z + 0.01) {
        return 0.0;
    }

    const float downwardSpeed = clamp(styleData.rainImpactVegetation0.y, 0.05, 6.0);
    const float hopSpacing = clamp(styleData.rainImpactVegetation0.z, 0.010, 0.30);
    const float streamSpread = clamp(styleData.rainImpactVegetation1.x, 0.0, 2.0);
    const float twinkle = clamp(styleData.rainImpactVegetation0.x, 0.0, 4.0);
    const float authoredWidth = clamp(
        styleData.rainImpactVegetation0.w,
        0.001,
        max(0.001, event.normalRadius.w * 0.20));
    // This includes crown widening and the 2x smooth feather, keeping every
    // visible stream sample inside the fixed event broad phase.
    const float maximumPathOffset = max(
        0.0,
        event.normalRadius.w - authoredWidth * 4.10);
    const float verticalDistance = max(0.0, event.positionBirth.z - point.z);
    const float maximumDepth = max(0.18, event.lifetimeEnergy.x * downwardSpeed);
    if (verticalDistance > maximumDepth) {
        return 0.0;
    }

    const float bandCoordinate = verticalDistance / hopSpacing;
    const uint bandIndex = uint(floor(bandCoordinate));
    const float bandMix = smoothstep(0.0, 1.0, fract(bandCoordinate));
    const uint pointHash = RainImpactHashCell(ivec3(floor(point / 0.005)));
    // Point-to-stream assignment keeps four crown paths at one path evaluation per event.
    const uint streamIndex = RainImpactHashBits(
        event.control.y ^ pointHash ^ 0x27d4eb2du) % 4u;
    const uint streamSeed = event.control.y ^
        RainImpactHashBits(0x9e3779b9u * (streamIndex + 1u));
    const vec2 branchDirection = RainImpactRandomDirection2D(streamSeed);

    vec3 eventNormal = event.normalRadius.xyz;
    eventNormal = dot(eventNormal, eventNormal) > 1e-8
        ? normalize(eventNormal)
        : vec3(0.0, 0.0, 1.0);
    const vec3 basisReference = abs(eventNormal.z) < 0.90
        ? vec3(0.0, 0.0, 1.0)
        : vec3(1.0, 0.0, 0.0);
    const vec3 tangentXCandidate = cross(basisReference, eventNormal);
    const vec3 tangentX = dot(tangentXCandidate, tangentXCandidate) > 1e-8
        ? normalize(tangentXCandidate)
        : vec3(1.0, 0.0, 0.0);
    const vec3 tangentYCandidate = cross(eventNormal, tangentX);
    const vec3 tangentY = dot(tangentYCandidate, tangentYCandidate) > 1e-8
        ? normalize(tangentYCandidate)
        : vec3(0.0, 1.0, 0.0);
    const vec2 anchorA = RainVegetationAnchorOffset(
        streamSeed,
        branchDirection,
        bandIndex,
        hopSpacing,
        maximumDepth,
        event.normalRadius.w,
        streamSpread,
        maximumPathOffset);
    const vec2 anchorB = RainVegetationAnchorOffset(
        streamSeed,
        branchDirection,
        bandIndex + 1u,
        hopSpacing,
        maximumDepth,
        event.normalRadius.w,
        streamSpread,
        maximumPathOffset);
    vec2 pathOffset = mix(anchorA, anchorB, bandMix);
    const float hopSide = (streamIndex & 1u) == 0u ? 1.0 : -1.0;
    const float hopArc = sin(3.14159265359 * clamp(fract(bandCoordinate), 0.0, 1.0)) *
        event.normalRadius.w * 0.055 * hopSide;
    pathOffset += vec2(-branchDirection.y, branchDirection.x) * hopArc;
    const float pathOffsetLength = length(pathOffset);
    if (pathOffsetLength > maximumPathOffset && pathOffsetLength > 1e-7) {
        pathOffset *= maximumPathOffset / pathOffsetLength;
    }
    const vec3 pathCentre =
        vec3(event.positionBirth.xy, event.positionBirth.z - verticalDistance) +
        tangentX * pathOffset.x + tangentY * pathOffset.y;
    const vec3 pointOffset = point - pathCentre;
    const vec3 tangentPointOffset = pointOffset - eventNormal * dot(pointOffset, eventNormal);
    const float crownWidth = mix(
        1.75,
        1.0,
        smoothstep(0.0, hopSpacing * 2.0, verticalDistance));
    const float streamWidth = authoredWidth * crownWidth *
        (0.85 + 0.30 * RainImpactRandom01(streamSeed + 7u));
    const float path = 1.0 - smoothstep(
        streamWidth,
        streamWidth * 2.0,
        length(tangentPointOffset));
    if (path <= 0.0) {
        return 0.0;
    }

    const uint sparkleSeed = event.control.y ^ pointHash ^
        RainImpactHashBits(0x85ebca6bu * (streamIndex + 1u));
    const float selectedPoint = smoothstep(
        0.52,
        0.92,
        RainImpactRandom01(sparkleSeed));
    if (selectedPoint <= 0.0) {
        return 0.0;
    }
    const float timeJitter =
        (RainImpactRandom01(sparkleSeed + 0xc2b2ae35u) - 0.5) * 0.10;
    const float streamDelay = float(streamIndex) * 0.026 +
        RainImpactRandom01(streamSeed + 19u) * 0.035;
    const float localAge = age - verticalDistance / downwardSpeed -
        streamDelay - timeJitter;
    const float pulse = smoothstep(0.0, 0.030, localAge) *
        (1.0 - smoothstep(0.12, 0.31, localAge));
    const float delayedGlint = smoothstep(0.15, 0.20, localAge) *
        (1.0 - smoothstep(0.28, 0.47, localAge)) * 0.62;
    vec3 normalizedPointNormal = pointNormal;
    normalizedPointNormal = dot(normalizedPointNormal, normalizedPointNormal) > 1e-8
        ? normalize(normalizedPointNormal)
        : vec3(0.0, 0.0, 1.0);
    const float leafFacing = 0.72 + 0.28 * abs(dot(normalizedPointNormal, eventNormal));
    return path * selectedPoint * max(pulse, delayedGlint) * twinkle * leafFacing;
}

RainImpactComposite ResolveRainImpactComposite(vec3 point, vec3 pointNormal) {
    RainImpactComposite composite = RainImpactComposite(0.0, 0.0, 1.0, 0.0);
    const uint role = styleData.rainImpactControl.y;
    const uint dimension = styleData.rainImpactControl.z;
    if (styleData.rainImpactControl.x == 0u || role == 0u || dimension == 0u ||
        styleData.rainImpactGrid.z <= 1e-6) {
        return composite;
    }
    const ivec2 coordinate = ivec2(floor(
        (point.xy - styleData.rainImpactGrid.xy) / styleData.rainImpactGrid.z));
    if (any(lessThan(coordinate, ivec2(0))) || any(greaterThanEqual(coordinate, ivec2(int(dimension))))) {
        return composite;
    }
    const uint cellIndex = uint(coordinate.y) * dimension + uint(coordinate.x);
    if (cellIndex >= uint(rainImpactCounts.length())) {
        return composite;
    }
    const uvec4 counts = rainImpactCounts[cellIndex];
    uint count = 0u;
    uint offset = 0u;
    if (role == kRainRoleSand) {
        count = min(counts.x, 8u);
    } else if (role == kRainRoleRock) {
        count = min(counts.y, 8u);
        offset = 8u;
    } else if (role == kRainRoleVegetation) {
        count = min(counts.z, 4u);
        offset = 16u;
    } else {
        return composite;
    }

    const float time = styleData.rainImpactGrid.w;
    for (uint localIndex = 0u; localIndex < count; ++localIndex) {
        const uint referenceIndex =
            cellIndex * kRainReferencesPerCell + offset + localIndex;
        if (referenceIndex >= uint(rainImpactReferences.length())) {
            break;
        }
        const uint eventIndex = rainImpactReferences[referenceIndex];
        if (eventIndex >= styleData.rainImpactControl.w ||
            eventIndex >= uint(rainImpactEvents.length())) {
            continue;
        }
        const RainImpactEventGpu event = rainImpactEvents[eventIndex];
        if (event.control.x != role) {
            continue;
        }
        const float age = time - event.positionBirth.w;
        const float lifetime = max(0.001, event.lifetimeEnergy.x);
        if (age < 0.0 || age > lifetime) {
            continue;
        }
        const float life = clamp(age / lifetime, 0.0, 1.0);
        float value = 0.0;
        if (role == kRainRoleSand) {
            const float distance = length(point.xy - event.positionBirth.xy);
            const float ringRadius = event.normalRadius.w * (0.12 + 0.88 * life);
            const float thickness = max(0.003, event.normalRadius.w * 0.14);
            const float ringDistance = abs(distance - ringRadius);
            value = exp(-(ringDistance * ringDistance) / (thickness * thickness)) *
                    (1.0 - smoothstep(0.72, 1.0, life));
        } else if (role == kRainRoleRock) {
            value = EvaluateRockRainImpactValue(
                event.positionBirth.xyz,
                event.normalRadius.xyz,
                event.normalRadius.w,
                event.lifetimeEnergy.x,
                event.control.y,
                styleData.rainImpactRock0,
                styleData.rainImpactRock1,
                point,
                pointNormal,
                age);
        } else {
            value = RainVegetationSprinkleValue(event, point, pointNormal, age);
        }
        value *= event.lifetimeEnergy.y;
        const bool vegetation = role == kRainRoleVegetation;
        composite.opacityAdd = max(
            composite.opacityAdd,
            value * (vegetation ? 0.14 : 0.18));
        composite.emissionAdd = max(
            composite.emissionAdd,
            value * (vegetation ? 0.48 : 0.11));
        composite.pointSizeMultiply = max(
            composite.pointSizeMultiply,
            1.0 + value * (vegetation ? 0.12 : 0.16));
        composite.colourMix = max(
            composite.colourMix,
            value * (role == kRainRoleRock ? 0.42 : (vegetation ? 0.18 : 0.20)));
    }
    return composite;
}

vec3 ApplyRainImpactColour(vec3 colour, RainImpactComposite impact) {
    const vec3 wetColour = styleData.rainImpactControl.y == kRainRoleVegetation
        ? vec3(0.54, 0.80, 0.82)
        : vec3(0.24, 0.48, 0.62);
    return mix(colour, wetColour, clamp(impact.colourMix, 0.0, 0.72));
}
