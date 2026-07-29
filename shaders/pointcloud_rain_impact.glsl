#include "rain_impact_rock_model.glsl"
#include "rain_impact_sand_model.glsl"

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
    // Colour mix contributed by the Droplets model; it tints brighter than
    // the shared wet colour used by Rings and Wetness.
    float dropletMix;
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
const uint kRainSandReferenceCapacity = 8u;
const uint kRainRockReferenceCapacity = 16u;
const uint kRainVegetationReferenceCapacity = 4u;
const uint kRainReferencesPerCell = 28u;

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

// Full strength inside [minZ, maxZ], fading linearly to zero over fadeMeters
// beyond each bounded edge. Open edges carry a huge finite sentinel so their
// clamp term stays 1.
float RainImpactBandWeight(vec3 band, float pointZ) {
    const float bandFade = max(1.0e-3, band.z);
    return clamp((pointZ - (band.x - bandFade)) / bandFade, 0.0, 1.0) *
           clamp(((band.y + bandFade) - pointZ) / bandFade, 0.0, 1.0);
}

// The three impact effects are decoupled from the cloud roles: every point
// layer evaluates every enabled model, each gated only by its own world-Z
// height band. Events spawn from rain striking the collision surfaces, and
// GROUND strikes (ROCK and SAND) feed BOTH the Rings and Wetness models —
// each model timing the event with its own lifetime lane — while VEG
// strikes feed Droplets only. Every event shades all clouds' points within
// range and band.
// rainImpactControl.x is an effect bitmask: 1 Rings, 2 Wetness, 4 Droplets.
//
// Each model first accumulates its own scalar value (Rings and Droplets take
// the maximum over their events, Wetness uses the peak-preserving soft
// union), then the three models compose additively so overlapping bands show
// every effect at once instead of the locally strongest erasing the others:
// opacity/emission adds sum, point-size multipliers multiply, and the two
// tint weights sum per target colour (their application clamps at 0.72).
// CPU mirror: EvaluateRainImpact in src/water/RainSimulation.cpp.
RainImpactComposite ResolveRainImpactComposite(vec3 point, vec3 pointNormal) {
    RainImpactComposite composite = RainImpactComposite(0.0, 0.0, 1.0, 0.0, 0.0);
    const uint effectMask = styleData.rainImpactControl.x;
    const uint dimension = styleData.rainImpactControl.z;
    if (effectMask == 0u || dimension == 0u ||
        styleData.rainImpactGrid.z <= 1e-6) {
        return composite;
    }
    const float ringsBand = (effectMask & 1u) != 0u
        ? RainImpactBandWeight(styleData.rainImpactSandBand.xyz, point.z)
        : 0.0;
    const float wetnessBand = (effectMask & 2u) != 0u
        ? RainImpactBandWeight(styleData.rainImpactRock1.yzw, point.z)
        : 0.0;
    const float dropletsBand = (effectMask & 4u) != 0u
        ? RainImpactBandWeight(styleData.rainImpactVegetation1.yzw, point.z)
        : 0.0;
    if (ringsBand <= 0.0 && wetnessBand <= 0.0 && dropletsBand <= 0.0) {
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
    const float time = styleData.rainImpactGrid.w;
    // Hoisted once per point so the reference loop below never re-reads
    // styleData vectors per event.
    const uint eventCount = min(
        styleData.rainImpactControl.w,
        uint(rainImpactEvents.length()));
    const uint referenceCount = uint(rainImpactReferences.length());
    const vec4 rockParams0 = styleData.rainImpactRock0;
    const vec4 rockParams1 = styleData.rainImpactRock1;
    const float rockDownhillStretch = styleData.rainImpactSandBand.w;
    const vec4 responses = max(styleData.rainImpactResponse, vec4(0.0));

    float sandWaterMask = 1.0;
    if (ringsBand > 0.0 && HasShorelineWaveEffect()) {
        const float boundaryZ = styleData.shorelineWaveParams0.x;
        const float edgeFade = max(
            0.001,
            styleData.shorelineWaveParams0.z);
        // The shoreline boundary is the authored high-water wavefront. Only
        // its flooded/downhill side receives the broad expanding ring.
        sandWaterMask = smoothstep(
            -edgeFade,
            edgeFade,
            boundaryZ - point.z);
    }

    float ringsValue = 0.0;
    float dropletsValue = 0.0;
    float rockPeak = 0.0;
    float rockRemaining = 1.0;
    for (uint modelIndex = 0u; modelIndex < 3u; ++modelIndex) {
        uint role = kRainRoleSand;
        uint occupiedMask = counts.x;
        uint capacity = kRainSandReferenceCapacity;
        uint offset = 0u;
        float bandWeight = ringsBand;
        if (modelIndex == 1u) {
            role = kRainRoleRock;
            occupiedMask = counts.y;
            capacity = kRainRockReferenceCapacity;
            offset = kRainSandReferenceCapacity;
            bandWeight = wetnessBand;
        } else if (modelIndex == 2u) {
            role = kRainRoleVegetation;
            occupiedMask = counts.z;
            capacity = kRainVegetationReferenceCapacity;
            offset = kRainSandReferenceCapacity + kRainRockReferenceCapacity;
            bandWeight = dropletsBand;
        }
        if (bandWeight <= 0.0 || occupiedMask == 0u) {
            continue;
        }
        for (uint localIndex = 0u; localIndex < capacity; ++localIndex) {
            if ((occupiedMask & (1u << localIndex)) == 0u) {
                continue;
            }
            const uint referenceIndex =
                cellIndex * kRainReferencesPerCell + offset + localIndex;
            if (referenceIndex >= referenceCount) {
                break;
            }
            const uint packedReference = rainImpactReferences[referenceIndex];
            if (packedReference == 0xffffffffu) {
                continue;
            }
            const uint eventIndex = packedReference & 0xffffu;
            if (eventIndex >= eventCount) {
                continue;
            }
            const RainImpactEventGpu event = rainImpactEvents[eventIndex];
            // The two ground lists reference BOTH ground strike roles (the
            // simulator bins ROCK and SAND strikes into each). A model
            // always times an event with ITS OWN lifetime: the struck
            // role's model lifetime sits in lifetimeEnergy.x and the other
            // ground model's in the .z lane, so Rings uses the ring timing
            // of a ROCK strike and Wetness the wet timing of a SAND strike.
            // Energy (lifetimeEnergy.y) stays shared. VEG stays veg-only.
            float modelLifetime = event.lifetimeEnergy.x;
            if (role == kRainRoleSand || role == kRainRoleRock) {
                if (event.control.x != kRainRoleSand &&
                    event.control.x != kRainRoleRock) {
                    continue;
                }
                if (event.control.x != role) {
                    modelLifetime = event.lifetimeEnergy.z;
                }
            } else if (event.control.x != role) {
                continue;
            }
            const float age = time - event.positionBirth.w;
            const float lifetime = max(0.001, modelLifetime);
            if (age < 0.0 || age > lifetime) {
                continue;
            }
            float value = 0.0;
            if (role == kRainRoleSand) {
                value = EvaluateSandRainImpactValue(
                    event.positionBirth.xyz,
                    event.normalRadius.w,
                    modelLifetime,
                    point,
                    age,
                    sandWaterMask,
                    responses.w);
            } else if (role == kRainRoleRock) {
                value = EvaluateRockRainImpactValue(
                    event.positionBirth.xyz,
                    event.normalRadius.xyz,
                    event.normalRadius.w,
                    modelLifetime,
                    event.control.y,
                    rockParams0,
                    rockParams1,
                    rockDownhillStretch,
                    point,
                    pointNormal,
                    age);
            } else {
                value = RainVegetationSprinkleValue(event, point, pointNormal, age);
            }
            // Response is per consuming effect, not the event's collision
            // role. It therefore affects every point and ground-impact role
            // inside this model's band and updates existing events instantly.
            const float response =
                role == kRainRoleSand ? responses.x :
                (role == kRainRoleRock ? responses.y : responses.z);
            value *= event.lifetimeEnergy.y * bandWeight * response;
            if (role == kRainRoleRock) {
                // Preserve the strongest individual contribution while softly
                // filling overlap. Adding another retained impact therefore
                // cannot cut a darker boundary through an existing wet region.
                rockPeak = max(rockPeak, value);
                rockRemaining *= 1.0 - clamp(value, 0.0, 1.0);
            } else if (role == kRainRoleVegetation) {
                dropletsValue = max(dropletsValue, value);
            } else {
                ringsValue = max(ringsValue, value);
            }
        }
    }
    const float wetnessValue = max(rockPeak, 1.0 - rockRemaining);
    // Cross-effect composition: the three models coexist. Adds sum, size
    // multipliers multiply, and both tint weights sum per target colour, so a
    // point inside overlapping bands shows Rings AND Wetness AND Droplets.
    // A model whose value is zero contributes exactly nothing, keeping every
    // single-effect response identical to its standalone evaluation.
    composite.opacityAdd =
        ringsValue * 0.18 + wetnessValue * 0.18 + dropletsValue * 0.14;
    composite.emissionAdd =
        ringsValue * 0.11 + wetnessValue * 0.11 + dropletsValue * 0.48;
    composite.pointSizeMultiply =
        (1.0 + ringsValue * 0.16) *
        (1.0 + wetnessValue * 0.16) *
        (1.0 + dropletsValue * 0.12);
    composite.colourMix = ringsValue * 0.20 + wetnessValue * 0.42;
    composite.dropletMix = dropletsValue * 0.18;
    return composite;
}

vec3 ApplyRainImpactColour(vec3 colour, RainImpactComposite impact) {
    const vec3 result = mix(
        colour,
        vec3(0.24, 0.48, 0.62),
        clamp(impact.colourMix, 0.0, 0.72));
    return mix(
        result,
        vec3(0.54, 0.80, 0.82),
        clamp(impact.dropletMix, 0.0, 0.72));
}
