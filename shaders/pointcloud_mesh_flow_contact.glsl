// Mesh Flow contact events are produced by the fixed-capacity Ground-flow
// compute pass. The hash contains four one-based event indices per 0.75 m XY
// bucket so a displayed terrain point examines at most 36 references.
struct MeshFlowContactEventGpu {
    vec4 positionTime;       // xyz contact, w birth time
    vec4 normalRadius;       // xyz averaged normal, w response radius
    uvec4 metadata;          // particle, Ground flags, packed tint, ROCK/VEG
    vec4 response;           // opacity, emission, persistence, VEG twinkle
    vec4 vegetationExtra;    // x downward stream depth
};

struct MeshFlowContactComposite {
    vec3 tint;
    float colourMix;
    float opacityAdd;
    float emissionAdd;
    float pointSizeMultiply;
};

layout(set = 0, binding = 17, std430) readonly buffer MeshFlowContactEventsBuffer {
    MeshFlowContactEventGpu meshFlowContactEvents[];
};

layout(set = 0, binding = 18, std430) readonly buffer MeshFlowContactGridBuffer {
    uvec4 meshFlowContactIndicesPlusOne[];
};

const float kMeshFlowContactCellSizeMeters = 0.75;
const uint kMeshFlowContactRoleRock = 0u;
const uint kMeshFlowContactRoleVegetation = 1u;
const uint kMeshFlowGroundVegetationSupportedFlag = 1u << 2u;

uint MeshFlowContactHashBits(uint value) {
    value ^= value >> 16u;
    value *= 0x7feb352du;
    value ^= value >> 15u;
    value *= 0x846ca68bu;
    return value ^ (value >> 16u);
}

uint MeshFlowContactRotateLeft(uint value, uint count) {
    return (value << count) | (value >> (32u - count));
}

uint MeshFlowContactHashCell(ivec2 coordinate) {
    uint hash = MeshFlowContactHashBits(uint(coordinate.x));
    hash ^= MeshFlowContactRotateLeft(
        MeshFlowContactHashBits(uint(coordinate.y)),
        11u);
    hash ^= MeshFlowContactRotateLeft(MeshFlowContactHashBits(0u), 22u);
    return MeshFlowContactHashBits(hash);
}

float MeshFlowContactHash01(uint value) {
    return float(MeshFlowContactHashBits(value) & 0x00ffffffu) /
           float(0x01000000u);
}

MeshFlowContactComposite EmptyMeshFlowContactComposite() {
    MeshFlowContactComposite result;
    result.tint = vec3(0.0);
    result.colourMix = 0.0;
    result.opacityAdd = 0.0;
    result.emissionAdd = 0.0;
    result.pointSizeMultiply = 1.0;
    return result;
}

bool MeshFlowContactRoleMatches(uint eventRole) {
    // The point style uses Rain's authored terrain role numbering:
    // ROCK = 1, SAND = 2, VEG = 3. Ground contacts intentionally ignore SAND.
    const uint pointRole = styleData.rainImpactControl.y;
    return (eventRole == kMeshFlowContactRoleRock && pointRole == 1u) ||
           (eventRole == kMeshFlowContactRoleVegetation && pointRole == 3u);
}

void BlendMeshFlowContact(
    inout MeshFlowContactComposite composite,
    MeshFlowContactEventGpu event,
    vec3 point,
    vec3 pointNormal,
    float timeSeconds) {
    if (!MeshFlowContactRoleMatches(event.metadata.w)) {
        return;
    }

    const float birthTime = event.positionTime.w;
    const float persistence = event.response.z;
    const float radius = event.normalRadius.w;
    if (isnan(birthTime) || isinf(birthTime) || birthTime < -1.0e30 ||
        persistence <= 0.0 || radius <= 0.0) {
        return;
    }
    const float age = timeSeconds - birthTime;
    if (age < 0.0 || age > persistence) {
        return;
    }

    const vec3 delta = point - event.positionTime.xyz;
    const float planarDistance = length(delta.xy);
    const bool vegetation =
        event.metadata.w == kMeshFlowContactRoleVegetation &&
        (event.metadata.y & kMeshFlowGroundVegetationSupportedFlag) != 0u;
    float spatial = 0.0;
    float pointTwinkle = 0.0;
    if (!vegetation) {
        const float distanceToContact = length(delta);
        if (distanceToContact > radius) {
            return;
        }
        const float radial = 1.0 - smoothstep(radius * 0.20, radius, distanceToContact);
        const float belowWeight = mix(
            0.72,
            1.18,
            smoothstep(-radius * 0.20, radius * 0.65, -delta.z));
        spatial = radial * belowWeight;
        pointTwinkle = spatial * 0.08;
    } else {
        if (planarDistance > radius || delta.z > radius * 0.10) {
            return;
        }
        const float streamDepth = max(radius, event.vegetationExtra.x);
        const float downwardDepth = max(0.0, -delta.z);
        if (downwardDepth > streamDepth) {
            return;
        }
        const float streamWidth = max(0.006, radius * mix(
            0.44,
            0.18,
            clamp(downwardDepth / streamDepth, 0.0, 1.0)));
        const uint pointSeed = MeshFlowContactHashBits(
            event.metadata.x ^
            MeshFlowContactHashBits(uint(floor(point.x / 0.01))) ^
            MeshFlowContactHashBits(uint(floor(point.y / 0.01))) ^
            MeshFlowContactHashBits(uint(floor(point.z / 0.01))));
        const vec2 wanderDirection = normalize(
            vec2(
                MeshFlowContactHash01(pointSeed + 0x68bc21ebu),
                MeshFlowContactHash01(pointSeed + 0x02e5be93u)) *
                2.0 -
            1.0 +
            vec2(1.0e-4, 0.0));
        const vec2 wanderingCentre =
            wanderDirection *
            sin(downwardDepth * 31.0 + float(event.metadata.x) * 0.73) *
            radius * 0.16;
        const float streamDistance = length(delta.xy - wanderingCentre);
        const float streamMask =
            1.0 - smoothstep(streamWidth, streamWidth * 2.5, streamDistance);
        const float crownMask =
            (1.0 - smoothstep(radius * 0.20, radius, planarDistance)) *
            (1.0 - smoothstep(0.0, radius * 0.50, downwardDepth));
        const float depthFade =
            1.0 - smoothstep(streamDepth * 0.72, streamDepth, downwardDepth);
        const float twinklePhase =
            timeSeconds * 7.0 -
            downwardDepth * 24.0 +
            MeshFlowContactHash01(pointSeed + 0x9e3779b9u) * 6.28318530718;
        const float pulse = pow(
            clamp(0.5 + 0.5 * sin(twinklePhase), 0.0, 1.0),
            3.0);
        const float twinkleStrength = max(0.0, event.response.w);
        spatial = max(crownMask, streamMask * depthFade * (0.38 + 0.62 * pulse));
        pointTwinkle =
            streamMask * depthFade * pulse * min(2.5, twinkleStrength) * 0.16;
    }

    const float lifeFade =
        1.0 - smoothstep(persistence * 0.48, persistence, age);
    vec3 eventNormal = event.normalRadius.xyz;
    eventNormal = dot(eventNormal, eventNormal) > 1.0e-8
        ? normalize(eventNormal)
        : vec3(0.0, 0.0, 1.0);
    vec3 normalizedPointNormal = pointNormal;
    normalizedPointNormal = dot(normalizedPointNormal, normalizedPointNormal) > 1.0e-8
        ? normalize(normalizedPointNormal)
        : vec3(0.0, 0.0, 1.0);
    const float facing = 0.76 + 0.24 * abs(dot(normalizedPointNormal, eventNormal));
    const float value = clamp(spatial * lifeFade * facing, 0.0, 2.0);
    if (value <= 0.0) {
        return;
    }

    const vec4 packedTint = unpackUnorm4x8(event.metadata.z);
    const float colourMix = clamp(packedTint.a * value, 0.0, 0.92);
    if (colourMix > composite.colourMix) {
        composite.tint = packedTint.rgb;
        composite.colourMix = colourMix;
    }
    composite.opacityAdd = max(
        composite.opacityAdd,
        max(0.0, event.response.x) * value);
    composite.emissionAdd = max(
        composite.emissionAdd,
        max(0.0, event.response.y) * value);
    composite.pointSizeMultiply = max(
        composite.pointSizeMultiply,
        1.0 + pointTwinkle * lifeFade);
}

MeshFlowContactComposite ResolveMeshFlowContactComposite(
    vec3 point,
    vec3 pointNormal,
    float timeSeconds) {
    MeshFlowContactComposite composite = EmptyMeshFlowContactComposite();
    const uint eventCapacity = uint(meshFlowContactEvents.length());
    const uint gridCapacity = uint(meshFlowContactIndicesPlusOne.length());
    if (eventCapacity == 0u || gridCapacity == 0u ||
        (gridCapacity & (gridCapacity - 1u)) != 0u) {
        return composite;
    }

    const uint gridMask = gridCapacity - 1u;
    const ivec2 centre = ivec2(floor(point.xy / kMeshFlowContactCellSizeMeters));
    for (int offsetY = -1; offsetY <= 1; ++offsetY) {
        for (int offsetX = -1; offsetX <= 1; ++offsetX) {
            const ivec2 coordinate = centre + ivec2(offsetX, offsetY);
            const uint bucket =
                MeshFlowContactHashCell(coordinate) & gridMask;
            const uvec4 references =
                meshFlowContactIndicesPlusOne[bucket];
            for (uint lane = 0u; lane < 4u; ++lane) {
                const uint indexPlusOne = references[lane];
                if (indexPlusOne == 0u || indexPlusOne > eventCapacity) {
                    continue;
                }
                BlendMeshFlowContact(
                    composite,
                    meshFlowContactEvents[indexPlusOne - 1u],
                    point,
                    pointNormal,
                    timeSeconds);
            }
        }
    }
    return composite;
}

vec3 ApplyMeshFlowContactColour(
    vec3 colour,
    MeshFlowContactComposite contact) {
    return mix(
        colour,
        contact.tint,
        clamp(contact.colourMix, 0.0, 0.92));
}
