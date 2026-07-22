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

float RainVegetationSprinkleValue(
    RainImpactEventGpu event,
    vec3 point,
    float age) {
    if (point.z > event.positionBirth.z + 0.01) {
        return 0.0;
    }

    const float downwardSpeed = 1.35;
    const float verticalDistance = max(0.0, event.positionBirth.z - point.z);
    const float maximumDepth = max(0.18, event.lifetimeEnergy.x * downwardSpeed);
    if (verticalDistance > maximumDepth) {
        return 0.0;
    }

    const float depthFraction = clamp(verticalDistance / maximumDepth, 0.0, 1.0);
    const float bandCoordinate = verticalDistance / 0.09;
    const uint bandIndex = uint(floor(bandCoordinate));
    const float bandMix = smoothstep(0.0, 1.0, fract(bandCoordinate));
    const uint pointHash = RainImpactHashCell(ivec3(floor(point / 0.005)));
    // Point-to-stream assignment keeps three visible paths at one path evaluation per event.
    const uint streamIndex = RainImpactHashBits(
        event.control.y ^ pointHash ^ 0x27d4eb2du) % 3u;
    const uint streamSeed = event.control.y ^
        RainImpactHashBits(0x9e3779b9u * (streamIndex + 1u));
    const vec2 branchDirection = RainImpactRandomDirection2D(streamSeed);
    const float spread = event.normalRadius.w * (0.08 + 0.52 * depthFraction) *
        (0.65 + 0.35 * RainImpactRandom01(streamSeed + 5u));
    const vec2 wanderA = RainImpactRandomDirection2D(
        streamSeed ^ RainImpactHashBits(bandIndex + 0xa511e9b3u));
    const vec2 wanderB = RainImpactRandomDirection2D(
        streamSeed ^ RainImpactHashBits(bandIndex + 1u + 0xa511e9b3u));
    const vec2 wanderDirection = mix(wanderA, wanderB, bandMix);
    const float wanderScale = event.normalRadius.w * 0.12 *
        smoothstep(0.0, 0.18, verticalDistance);
    const vec2 pathCentre = event.positionBirth.xy +
        branchDirection * spread + wanderDirection * wanderScale;
    const float streamWidth = max(
        0.0022,
        event.normalRadius.w * (0.08 + 0.015 * RainImpactRandom01(streamSeed + 7u)));
    const float path = 1.0 - smoothstep(
        streamWidth,
        streamWidth * 2.0,
        length(point.xy - pathCentre));
    if (path <= 0.0) {
        return 0.0;
    }

    const uint sparkleSeed = event.control.y ^ pointHash ^
        RainImpactHashBits(0x85ebca6bu * (streamIndex + 1u));
    const float selectedPoint = smoothstep(
        0.62,
        0.94,
        RainImpactRandom01(sparkleSeed));
    if (selectedPoint <= 0.0) {
        return 0.0;
    }
    const float timeJitter =
        (RainImpactRandom01(sparkleSeed + 0xc2b2ae35u) - 0.5) * 0.12;
    const float streamDelay = float(streamIndex) * 0.035 +
        RainImpactRandom01(streamSeed + 19u) * 0.04;
    const float localAge = age - verticalDistance / downwardSpeed -
        streamDelay - timeJitter;
    const float pulse = smoothstep(0.0, 0.035, localAge) *
        (1.0 - smoothstep(0.085, 0.24, localAge));
    return path * selectedPoint * pulse;
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
        const uint eventIndex = rainImpactReferences[cellIndex * kRainReferencesPerCell + offset + localIndex];
        if (eventIndex >= styleData.rainImpactControl.w) {
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
            vec3 eventNormal = event.normalRadius.xyz;
            eventNormal = dot(eventNormal, eventNormal) > 1e-8 ? normalize(eventNormal) : normalize(pointNormal);
            const vec3 delta = point - event.positionBirth.xyz;
            const float normalDistance = abs(dot(delta, eventNormal));
            const vec3 tangent = delta - eventNormal * dot(delta, eventNormal);
            const float normalizedDistance =
                sqrt(dot(tangent, tangent) + normalDistance * normalDistance * 4.0) /
                max(0.001, event.normalRadius.w);
            const float growthSeconds = clamp(lifetime * 0.18, 0.55, 0.95);
            const float growth = smoothstep(0.0, growthSeconds, age);
            const float edgeWidth = 0.07 + (1.0 - growth) * 0.09;
            value = (1.0 - smoothstep(
                         max(0.0, growth - edgeWidth),
                         growth + edgeWidth,
                         normalizedDistance)) *
                    (1.0 - smoothstep(0.55, 1.0, life));
        } else {
            value = RainVegetationSprinkleValue(event, point, age);
        }
        value *= event.lifetimeEnergy.y;
        const bool vegetation = role == kRainRoleVegetation;
        composite.opacityAdd = max(
            composite.opacityAdd,
            value * (vegetation ? 0.08 : 0.18));
        composite.emissionAdd = max(
            composite.emissionAdd,
            value * (vegetation ? 0.24 : 0.11));
        composite.pointSizeMultiply = max(
            composite.pointSizeMultiply,
            1.0 + value * (vegetation ? 0.07 : 0.16));
        composite.colourMix = max(
            composite.colourMix,
            value * (role == kRainRoleRock ? 0.42 : (vegetation ? 0.09 : 0.20)));
    }
    return composite;
}

vec3 ApplyRainImpactColour(vec3 colour, RainImpactComposite impact) {
    const vec3 wetColour = styleData.rainImpactControl.y == kRainRoleVegetation
        ? vec3(0.54, 0.80, 0.82)
        : vec3(0.24, 0.48, 0.62);
    return mix(colour, wetColour, clamp(impact.colourMix, 0.0, 0.72));
}
