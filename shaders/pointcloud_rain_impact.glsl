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
            value = (1.0 - smoothstep(0.45, 1.0, normalizedDistance)) *
                    (1.0 - smoothstep(0.55, 1.0, life));
        } else if (point.z <= event.positionBirth.z + 0.01) {
            const float xyDistance = length(point.xy - event.positionBirth.xy);
            const float column = 1.0 - smoothstep(
                event.normalRadius.w * 0.55,
                event.normalRadius.w,
                xyDistance);
            const float verticalDistance = max(0.0, event.positionBirth.z - point.z);
            const ivec3 variationCell = ivec3(floor(point / 0.04));
            const float variation = (
                RainImpactRandom01(event.control.y ^ RainImpactHashCell(variationCell)) - 0.5) * 0.12;
            const float localAge = age - verticalDistance / 1.6 - variation;
            if (localAge >= 0.0) {
                const float localLife = localAge / max(0.15, lifetime * 0.65);
                value = column * smoothstep(0.0, 0.12, localLife) *
                        (1.0 - smoothstep(0.35, 1.0, localLife));
            }
        }
        value *= event.lifetimeEnergy.y;
        composite.opacityAdd = max(composite.opacityAdd, value * 0.18);
        composite.emissionAdd = max(
            composite.emissionAdd,
            value * (role == kRainRoleVegetation ? 0.24 : 0.11));
        composite.pointSizeMultiply = max(composite.pointSizeMultiply, 1.0 + value * 0.16);
        composite.colourMix = max(composite.colourMix, value * (role == kRainRoleRock ? 0.42 : 0.20));
    }
    return composite;
}

vec3 ApplyRainImpactColour(vec3 colour, RainImpactComposite impact) {
    const vec3 wetColour = styleData.rainImpactControl.y == kRainRoleVegetation
        ? vec3(0.54, 0.80, 0.82)
        : vec3(0.24, 0.48, 0.62);
    return mix(colour, wetColour, clamp(impact.colourMix, 0.0, 0.72));
}
