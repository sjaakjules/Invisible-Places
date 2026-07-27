// Mesh Flow contact events are produced by the fixed-capacity Ground-flow
// compute pass. Events are splatted into every 0.25 m XY bucket their
// response radius overlaps, so a displayed terrain point examines at most 36
// references and never sees cell-aligned activation edges.
struct MeshFlowContactEventGpu {
    vec4 positionTime;       // xyz contact, w birth time
    vec4 normalRadius;       // xyz averaged normal, w response radius
    uvec4 metadata;          // particle, Ground flags, packed tint, ROCK/VEG
    vec4 response;           // opacity, emission, persistence, VEG twinkle
    vec4 vegetationExtra;    // stream depth, moisture, convergence, seed
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

const float kMeshFlowContactCellSizeMeters = 0.25;
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
    // Both event roles shade ROCK and VEG points: water striking a rock edge
    // also wets the understory hanging above it.
    const uint pointRole = styleData.rainImpactControl.y;
    return pointRole == 1u || pointRole == 3u;
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
    // Radius and the tunable upward reach share one lane as half floats.
    const vec2 radiusReach =
        unpackHalf2x16(floatBitsToUint(event.normalRadius.w));
    const float radius = radiusReach.x;
    const float upwardReach = max(radiusReach.y, radius * 0.12);
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
    const float streamDepth = max(radius, event.vegetationExtra.x);
    const float downwardDepth = max(0.0, -delta.z);
    const float upwardHeight = max(0.0, delta.z);
    if (planarDistance > radius || upwardHeight > upwardReach ||
        downwardDepth > streamDepth) {
        return;
    }

    // Use event-level hashes for the filament centres. Point-level randomness
    // is reserved for twinkle, so neighbouring cloud points describe coherent
    // gravity-led fingers rather than unrelated speckles.
    const uint eventSeed = MeshFlowContactHashBits(
        event.metadata.x ^
        floatBitsToUint(event.vegetationExtra.w) ^
        0x9e3779b9u);
    vec3 eventNormal = event.normalRadius.xyz;
    eventNormal = dot(eventNormal, eventNormal) > 1.0e-8
        ? normalize(eventNormal)
        : vec3(0.0, 0.0, 1.0);
    const vec3 projectedGravity =
        vec3(0.0, 0.0, -1.0) -
        eventNormal * dot(vec3(0.0, 0.0, -1.0), eventNormal);
    const float fallbackAngle =
        MeshFlowContactHash01(eventSeed + 0x68bc21ebu) * 6.28318530718;
    const vec2 projectedDirection = projectedGravity.xy;
    const vec2 mainDirection =
        dot(projectedDirection, projectedDirection) > 1.0e-8
            ? normalize(projectedDirection)
            : vec2(cos(fallbackAngle), sin(fallbackAngle));
    const float branchOffset =
        mix(
            -0.20943951024,
            0.20943951024,
            MeshFlowContactHash01(eventSeed + 0x02e5be93u));
    const float branchCos = cos(branchOffset);
    const float branchSin = sin(branchOffset);
    const vec2 crossDirection = vec2(-mainDirection.y, mainDirection.x);
    const vec2 branchDirection = vec2(
        mainDirection.x * branchCos -
            mainDirection.y * branchSin,
        mainDirection.x * branchSin +
            mainDirection.y * branchCos);
    const float depthT = clamp(downwardDepth / streamDepth, 0.0, 1.0);
    const float moisture = clamp(event.vegetationExtra.y, 0.0, 1.0);
    const float convergence = clamp(event.vegetationExtra.z, 0.0, 1.0);
    const float flowEnergy =
        clamp(0.42 + 0.30 * moisture + 0.28 * convergence, 0.0, 1.0);
    const float eventPhase =
        MeshFlowContactHash01(eventSeed + 0x85ebca6bu) * 6.28318530718;
    const float wanderAmplitude =
        radius * mix(0.025, 0.075, flowEnergy) *
        (0.25 + 0.75 * depthT);
    const vec2 mainCentre =
        mainDirection *
            sin(downwardDepth * mix(19.0, 29.0, flowEnergy) + eventPhase) *
            wanderAmplitude +
        crossDirection *
            sin(downwardDepth * 13.0 + eventPhase * 1.37) *
            wanderAmplitude * 0.28;
    const float branchProbability = mix(0.20, 0.55, moisture);
    const float branchEnabled =
        MeshFlowContactHash01(eventSeed + 0x27d4eb2fu) <
                branchProbability
            ? 1.0
            : 0.0;
    const float branchGate =
        branchEnabled * smoothstep(0.18, 0.42, depthT);
    const vec2 branchCentre =
        mainCentre +
        branchDirection *
            (downwardDepth - streamDepth * 0.12) *
            0.08 * branchGate;

    const float mainWidth = max(
        0.005,
        radius * mix(
            vegetation ? 0.24 : 0.17,
            vegetation ? 0.10 : 0.065,
            depthT));
    const float branchWidth = max(0.004, mainWidth * 0.70);
    const float mainDistance = length(delta.xy - mainCentre);
    const float branchDistance = length(delta.xy - branchCentre);
    const float mainStream =
        1.0 - smoothstep(mainWidth, mainWidth * 2.15, mainDistance);
    const float branchStream =
        (1.0 - smoothstep(
            branchWidth,
            branchWidth * 2.20,
            branchDistance)) *
        branchGate;
    const float streamMask = max(mainStream, branchStream * 0.82);

    // The crown blooms symmetrically: downward it hands over to the stream
    // filaments, upward it fades across the tunable reach so overhanging
    // points wet smoothly instead of cutting off at the contact height.
    const float crownVertical = delta.z > 0.0
        ? 1.0 - smoothstep(0.0, max(upwardReach, 1.0e-4), upwardHeight)
        : 1.0 - smoothstep(radius * 0.08, radius * 0.72, downwardDepth);
    const float crownMask =
        (1.0 - smoothstep(radius * 0.16, radius, planarDistance)) *
        crownVertical;
    const float belowWeight = mix(
        0.95,
        1.18,
        smoothstep(-radius * 0.12, radius * 0.72, -delta.z));
    const float depthFade =
        1.0 - smoothstep(streamDepth * 0.72, streamDepth, downwardDepth);
    const float pulsePhase =
        timeSeconds * mix(5.2, 7.2, moisture) -
        downwardDepth * mix(19.0, 28.0, flowEnergy) +
        eventPhase;
    const float pulse = pow(
        clamp(0.5 + 0.5 * sin(pulsePhase), 0.0, 1.0),
        3.0);
    const float descendingFront =
        smoothstep(
            depthT * persistence * 0.32,
            depthT * persistence * 0.32 + max(0.08, persistence * 0.10),
            age);
    const float streamSignal =
        streamMask * depthFade * descendingFront *
        (0.26 + 0.74 * pulse) * mix(0.72, 1.0, flowEnergy);
    const float spatial = max(crownMask * belowWeight, streamSignal);

    const uint pointSeed = MeshFlowContactHashBits(
        eventSeed ^
        MeshFlowContactHashBits(uint(floor(point.x / 0.01))) ^
        MeshFlowContactHashBits(uint(floor(point.y / 0.01))) ^
        MeshFlowContactHashBits(uint(floor(point.z / 0.01))));
    const float pointVariation =
        0.78 + 0.22 * MeshFlowContactHash01(pointSeed + 0xc2b2ae35u);
    const float twinkleStrength = vegetation
        ? max(0.0, event.response.w)
        : 0.65;
    const float pointTwinkle =
        streamMask * depthFade * descendingFront * pulse *
        min(2.5, twinkleStrength) * 0.16 * pointVariation;

    const float lifeFade =
        1.0 - smoothstep(persistence * 0.48, persistence, age);
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
