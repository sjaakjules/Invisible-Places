const float kRippleTwoPi = 6.28318530718;

struct SparseRippleComposite {
    float scale;
    float colourMix;
    float emissionAdd;
    float opacityAdd;
    float opacityMultiply;
    float pointSizeAdd;
    float pointSizeMultiply;
    vec3 colour;
};

struct SeepageLook {
    // x: pattern (0 wet rock, 1 chaotic bloom, 2 wetting trickle,
    // 3 contour pulses), y: blend mode.
    uvec4 control;
    // Contour Pulses: x spacing, y width, z speed, w irregularity.
    // legacy1 carries x: wave count, y: speed variation,
    // z: density, w: response intensity.
    vec4 legacy0;
    vec4 legacy1;
    vec4 response0;
    vec4 response1;
    vec4 response2;
    // x: feature size, y: contrast, z: evolution, w: roughness.
    vec4 organic0;
    // x: angle response, y: micro-normal strength, z: glint density, w: curl.
    vec4 organic1;
    // x: breakup, y: downhill drift, z: spare (was trickle length),
    // w: finger width.
    vec4 organic2;
    // x: trickle-front softness.
    vec4 organic3;
    vec4 environmentDirection;
};

struct SeepageNodeTopology {
    // x: stable node id. Seed and noise rotation are live parameters.
    uvec4 control;
    // xyz: clicked surface position, w: topology selection reach limit.
    vec4 positionReach;
    // xyz: clicked surface normal, w: accepted surface-plane thickness in metres.
    vec4 normalSurface;
    // xyz: gravity projected onto the clicked surface, w: fan edge feather in metres.
    vec4 downEdge;
    // xyz: lateral fan direction, w: legacy topology source half-width.
    vec4 lateralStart;
    // x: topology selection half-width limit,
    // y: maximum resident upstream flow-run distance.
    vec4 geometry;
    // x: sample count, y: achieved reach as float bits, z: valid, w: complete.
    uvec4 guideControl;
    // xyz: surface-following guide position, w: cumulative downstream station.
    vec4 guidePositionStation[8];
    // xyz: surface normal, w: guide confidence.
    vec4 guideNormalConfidence[8];
};

struct SeepageNodeParams {
    // x: stable node id, y: quality (0 low, 1 balanced, 2 high), z: blend mode, w: seed.
    uvec4 control;
    // x: viewport enabled factor, y: normal-alignment weight,
    // z: animation-composed strength,
    // w: rain visual strength.
    vec4 geometry;
    SeepageLook look;
    SeepageLook transitionLook;
    // x: scenario/local spread, y: pattern-transition amount, z: wetting progress.
    vec4 scenario;
    // x: live reach (guided/planar fallback), y: live source width,
    // z: live prominence, w: least-resistance travel budget (connected).
    vec4 liveGeometry;
    // Seed-derived, orientation-independent world-noise rotation.
    vec4 noiseBasis[3];
    // x/y: current/transition sample counts, z/w: stable spans as float bits.
    // The two fixed-capacity fields remain inside the frame-ringed parameter
    // record, so animated pulses never alter topology or descriptors.
    uvec4 pulseFieldControl;
    vec4 pulseField[32];
    vec4 transitionPulseField[32];
};

struct SeepageHashCell {
    // xyz: signed world-grid coordinate, w: occupied flag.
    ivec4 coordinate;
    // x: first node-reference index, y: node-reference count.
    uvec4 range;
};

struct SeepageNodeReference {
    uint nodeIndex;
    float leastResistanceCost;
    // x: accumulated local flow-run; y: downstream cross-contour travel or
    // station-relative upstream branch excess, packed as IEEE-754 half
    // floats to preserve the 16-byte std430 reference ABI.
    uint packedRunCross;
    uint packedNormalRoleConfidenceFlags;
};

layout(set = 0, binding = 10, std430) readonly buffer SeepageNodesBuffer {
    SeepageNodeTopology seepageNodes[];
} seepageNodeData;

layout(set = 0, binding = 11, std430) readonly buffer SeepageHashCellsBuffer {
    SeepageHashCell seepageHashCells[];
} seepageHashCellData;

layout(set = 0, binding = 12, std430) readonly buffer SeepageNodeReferencesBuffer {
    SeepageNodeReference seepageNodeReferences[];
} seepageNodeReferenceData;

// Live Seepage parameters are separate from the immutable guide topology and
// are backed by a frame-safe ring on the renderer side.
layout(set = 0, binding = 16, std430) readonly buffer SeepageNodeParamsBuffer {
    SeepageNodeParams seepageParams[];
} seepageParamData;

bool RippleFiniteFloat(float value) {
    return !isnan(value) && !isinf(value);
}

bool RippleFiniteVec3(vec3 value) {
    return !any(isnan(value)) && !any(isinf(value));
}

bool HasShorelineWaveEffect() {
    return styleData.shorelineWaveControl.x != 0u &&
           styleData.shorelineWaveParams0.y > 1e-5 &&
           styleData.shorelineWaveParams1.w > 1e-5 &&
           styleData.shorelineWaveParams0.w > 1e-5;
}

bool HasSeepageEffect() {
    const uint capacity = styleData.seepageControl.z;
    return styleData.seepageControl.x != 0u &&
           styleData.seepageControl.y != 0u &&
           capacity != 0u &&
           (capacity & (capacity - 1u)) == 0u &&
           styleData.seepageControl.w != 0u &&
           styleData.seepageControl.y <= uint(seepageNodeData.seepageNodes.length()) &&
           styleData.seepageControl.y <= uint(seepageParamData.seepageParams.length()) &&
           capacity <= uint(seepageHashCellData.seepageHashCells.length()) &&
           styleData.seepageControl.w <=
               uint(seepageNodeReferenceData.seepageNodeReferences.length()) &&
           RippleFiniteFloat(styleData.seepageGridParams.x) &&
           styleData.seepageGridParams.x > 1e-6 &&
           RippleFiniteVec3(styleData.seepageBoundsMin.xyz) &&
           RippleFiniteVec3(styleData.seepageBoundsMax.xyz) &&
           all(lessThanEqual(
               styleData.seepageBoundsMin.xyz,
               styleData.seepageBoundsMax.xyz));
}

uint SeepageHashUint(uint value) {
    value ^= value >> 16u;
    value *= 0x7feb352du;
    value ^= value >> 15u;
    value *= 0x846ca68bu;
    value ^= value >> 16u;
    return value;
}

uint SeepageCellHash(ivec3 coordinate) {
    uint value = uint(coordinate.x) * 0x8da6b343u;
    value ^= uint(coordinate.y) * 0xd8163841u;
    value ^= uint(coordinate.z) * 0xcb1ab31fu;
    return SeepageHashUint(value);
}

float RippleHash(float value) {
    return fract(sin(value) * 43758.5453123);
}

float RippleCellHash(int cellX, int cellY, float seed, float salt) {
    return RippleHash(
        float(cellX) * 12.9898 +
        float(cellY) * 78.233 +
        seed * 37.719 +
        salt * 19.371);
}

float RippleSmoothBlockNoise(vec2 uv, float cellSize, float seed, float salt) {
    const float safeCellSize = max(0.001, cellSize);
    const vec2 p = uv / safeCellSize;
    const int cellX = int(floor(p.x));
    const int cellY = int(floor(p.y));
    const vec2 f = fract(p);
    const vec2 u = f * f * (3.0 - 2.0 * f);
    const float a = RippleCellHash(cellX, cellY, seed, salt);
    const float b = RippleCellHash(cellX + 1, cellY, seed, salt);
    const float c = RippleCellHash(cellX, cellY + 1, seed, salt);
    const float d = RippleCellHash(cellX + 1, cellY + 1, seed, salt);
    return mix(mix(a, b, u.x), mix(c, d, u.x), u.y);
}

float RippleLine(float distance, float width) {
    return 1.0 - smoothstep(0.0, max(1e-5, width), abs(distance));
}

vec3 RippleSafeNormal(vec3 normal) {
    return dot(normal, normal) > 1e-8 ? normalize(normal) : vec3(0.0, 0.0, 1.0);
}

vec3 RippleSafeLateral(vec3 direction) {
    vec3 lateral = abs(direction.z) < 0.999
        ? cross(vec3(0.0, 0.0, 1.0), direction)
        : cross(vec3(0.0, 1.0, 0.0), direction);
    return dot(lateral, lateral) > 1e-8 ? normalize(lateral) : vec3(1.0, 0.0, 0.0);
}

float SandCloudShorelineWaveValue(vec2 uv, float shoreDistance, float edgeBlendWidth, float wavelength, float warp, float turbulence, float density, float seed, float phase) {
    const float travelDistance = max(wavelength, 0.015);
    const float t = phase;
    const float density01 = clamp(density, 0.0, 1.0);
    const float turbulence01 = clamp(turbulence, 0.0, 1.0);
    const float clampedWarp = clamp(warp, 0.0, 2.0);
    const float alongShore = uv.y;
    const float lateralScale = max(wavelength * 1.35, 0.012);
    const float frontWidth = max(wavelength * (0.046 + turbulence01 * 0.026), 0.003);
    const float trailLength = max(wavelength * (1.05 + turbulence01 * 0.68), frontWidth * 7.0);
    const float incomingShare = 0.58;
    const float returnShare = 0.30;
    const float waveRate = 0.070 + density01 * 0.045;
    const float warpGuard = wavelength * (0.18 + clampedWarp * 0.16 + turbulence01 * 0.08);
    const float finishOffset = max(edgeBlendWidth, edgeBlendWidth + warpGuard);
    float combined = 0.0;

    // Band Density chooses how many of the four fronts run: two at 0, four
    // at 1. The fronts share one cycle rate on fixed staggered offsets with
    // only a bounded wobble, so their brief quiet windows can never drift
    // into alignment and the shore never pauses: at least one front is
    // always inbound with two fronts, and effectively two once all four run.
    const float frontCount = mix(2.0, 4.0, density01);

    for (int waveIndex = 0; waveIndex < 4; ++waveIndex) {
        const float slot = float(waveIndex);
        // Activation order 0, 2, 1, 3: the first two fronts sit half a
        // cycle apart, so the always-inbound guarantee holds from Density 0.
        const float activationRank =
            waveIndex == 0 ? 0.0
                           : (waveIndex == 2 ? 1.0
                                             : (waveIndex == 1 ? 2.0 : 3.0));
        const float frontWeight = 1.0 - smoothstep(
            frontCount - 0.35,
            frontCount + 0.15,
            activationRank + 0.5);
        if (frontWeight <= 1.0e-4) {
            continue;
        }
        const float slotSeed = seed + slot * 53.17;
        const float timingNoise = RippleHash(slotSeed * 0.071 + 11.0);

        const float offset =
            slot * 0.25 +
            (timingNoise - 0.5) * 0.05 +
            RippleSmoothBlockNoise(vec2(t * 0.018, slotSeed), 0.23, seed, 181.0) * 0.04;
        const float cycle = fract(t * waveRate + offset);
        const float activeEnd = incomingShare + returnShare;
        if (cycle >= activeEnd) {
            continue;
        }

        const float scallopNoise = RippleSmoothBlockNoise(
            vec2(alongShore + slot * wavelength * 0.37, seed * 0.13 + slot * 0.41),
            max(wavelength * 0.48, 0.008),
            seed,
            151.0 + slot * 19.0);
        const float frontWarp =
            (sin((alongShore / lateralScale) + seed * 1.17 + slot * 1.91) * 0.62 +
             sin((alongShore / max(wavelength * 0.58, 0.006)) - seed * 0.73 + slot * 2.37) * 0.28 +
             (scallopNoise - 0.5) * 1.15) *
            wavelength * (0.045 + clampedWarp * 0.060 + turbulence01 * 0.035);
        const float x = finishOffset - max(0.0, shoreDistance) - frontWarp;
        const float waveTravel = travelDistance * mix(1.38, 1.82, RippleHash(slotSeed * 0.061 + 47.0));
        const float offshoreStart = -waveTravel * (0.72 + timingNoise * 0.18);
        const float shoreEnd = 0.0;
        const float shoreBreakup = smoothstep(0.24, 0.92, scallopNoise + turbulence01 * 0.22);
        const float shorewardMask = 1.0 - smoothstep(frontWidth * 0.45, frontWidth * 2.20, x - shoreEnd);
        // Brightness the incoming front holds when it stops at the shore.
        // The return branch's carry starts from this same level so the
        // incoming-to-return handoff never dims in a single frame.
        const float peakArrivalLevel = 0.86;

        if (cycle < incomingShare) {
            const float incomingProgress = smoothstep(0.0, 1.0, cycle / incomingShare);
            const float frontPosition = mix(offshoreStart, shoreEnd, incomingProgress);
            const float front = x - frontPosition;
            const float frontLocalCoordinate = front;
            // Stretch breakup along the moving wave front so foam grain travels with the crest.
            const vec2 waveFrontStreakUv = vec2(
                frontLocalCoordinate * (2.05 + turbulence01 * 0.45) + slot * 0.19,
                alongShore * (0.11 + turbulence01 * 0.05) +
                    sin(frontLocalCoordinate / max(wavelength * 0.55, 0.006) + slotSeed * 0.17) * wavelength * 0.045 +
                    slot * 0.31);
            const float waveFrontStreakNoise = RippleSmoothBlockNoise(
                waveFrontStreakUv,
                max(wavelength * 0.20, 0.006),
                seed,
                203.0 + slot * 23.0);
            const float foamMottleNoise = RippleSmoothBlockNoise(
                vec2(frontLocalCoordinate * 0.54 + slot * 0.19, alongShore * 0.62 + slot * 0.31),
                max(wavelength * 0.42, 0.007),
                seed,
                227.0 + slot * 29.0);
            const float foamNoise = RippleSmoothBlockNoise(
                mix(vec2(frontLocalCoordinate * 0.54 + slot * 0.19, alongShore * 0.62 + slot * 0.31), waveFrontStreakUv, 0.78),
                max(wavelength * 0.24, 0.006),
                seed,
                251.0 + slot * 31.0);
            const float breakup = smoothstep(
                0.18,
                0.96,
                waveFrontStreakNoise * 0.52 +
                    foamNoise * 0.28 +
                    foamMottleNoise * 0.18 +
                    shoreBreakup * 0.24 +
                    turbulence01 * 0.18);
            const float foamContinuity = mix(0.72, 1.0, breakup);
            const float incomingDeepSideMask = 1.0 - smoothstep(-frontWidth * 0.35, frontWidth * 0.15, front);
            const float incomingStartMask =
                smoothstep(offshoreStart - frontWidth * 1.25, offshoreStart + frontWidth * 1.25, x);
            const float incomingMask = incomingStartMask * incomingDeepSideMask;
            const float crest = RippleLine(front, frontWidth);
            const float crestHalo = RippleLine(front, frontWidth * 1.70);
            const float frontTrailDistance = max(0.0, -front);
            const float incomingFollowMask =
                1.0 - smoothstep(trailLength * 0.18, trailLength * 0.50, frontTrailDistance);
            const float arrivingFoam =
                exp(-frontTrailDistance / max(trailLength * 0.42, 1.0e-4)) *
                smoothstep(frontWidth * 0.35, frontWidth * 1.70, frontTrailDistance) *
                incomingFollowMask;
            const float incomingBodyDistance = max(0.0, frontPosition - x);
            const float incomingBodyFade =
                1.0 - smoothstep(waveTravel * 0.72, waveTravel * 1.02, incomingBodyDistance);
            const float incomingBodyFoam =
                incomingMask * incomingBodyFade * (0.10 + density01 * 0.12) * foamContinuity;
            const float arrivalFade = smoothstep(0.01, 0.12, cycle);
            // Arriving fronts must stay the brighter half of the cycle, so
            // the arrival dip stays mild compared to the receding crest.
            const float peakSoftening =
                1.0 - smoothstep(0.84, 1.0, incomingProgress) * (1.0 - peakArrivalLevel);
            const float incomingEdgeFoam =
                (crest * (0.70 + shoreBreakup * 0.22) * foamContinuity +
                 crestHalo * (0.08 + density01 * 0.06) * foamContinuity +
                 arrivingFoam * (0.42 + density01 * 0.28) * breakup) *
                incomingMask;
            const float value = max(incomingBodyFoam, incomingEdgeFoam) * arrivalFade * peakSoftening;
            combined = max(combined, value * shorewardMask * frontWeight);
        } else {
            const float returnProgress = smoothstep(0.0, 1.0, (cycle - incomingShare) / returnShare);
            const float returnDistance = waveTravel * 0.50;
            const float clearFront = shoreEnd - returnDistance * returnProgress;
            const float front = x - clearFront;
            const float frontLocalCoordinate = front;
            // Reuse the same front-local grain as the incoming phase so the crest does not resample at peak.
            const vec2 waveFrontStreakUv = vec2(
                frontLocalCoordinate * (2.05 + turbulence01 * 0.45) + slot * 0.19,
                alongShore * (0.11 + turbulence01 * 0.05) +
                    sin(frontLocalCoordinate / max(wavelength * 0.55, 0.006) + slotSeed * 0.17) * wavelength * 0.045 +
                    slot * 0.31);
            const float waveFrontStreakNoise = RippleSmoothBlockNoise(
                waveFrontStreakUv,
                max(wavelength * 0.20, 0.006),
                seed,
                203.0 + slot * 23.0);
            const float foamMottleNoise = RippleSmoothBlockNoise(
                vec2(frontLocalCoordinate * 0.54 + slot * 0.19, alongShore * 0.62 + slot * 0.31),
                max(wavelength * 0.42, 0.007),
                seed,
                227.0 + slot * 29.0);
            const float foamNoise = RippleSmoothBlockNoise(
                mix(vec2(frontLocalCoordinate * 0.54 + slot * 0.19, alongShore * 0.62 + slot * 0.31), waveFrontStreakUv, 0.78),
                max(wavelength * 0.24, 0.006),
                seed,
                251.0 + slot * 31.0);
            const float breakup = smoothstep(
                0.18,
                0.96,
                waveFrontStreakNoise * 0.52 +
                    foamNoise * 0.28 +
                    foamMottleNoise * 0.18 +
                    shoreBreakup * 0.24 +
                    turbulence01 * 0.18);
            const float foamContinuity = mix(0.72, 1.0, breakup);
            const float remainingMask = 1.0 - smoothstep(-frontWidth * 1.25, frontWidth * 1.55, front);
            const float foamDistance = max(0.0, shoreEnd - x);
            const float heldFoam =
                exp(-foamDistance / max(trailLength, 1.0e-4)) *
                smoothstep(frontWidth * 0.35, frontWidth * 1.70, foamDistance) *
                (1.0 - smoothstep(trailLength * 0.30, trailLength * 0.72, foamDistance));
            const float returnFade = 1.0 - smoothstep(0.10, 0.46, returnProgress);
            const float heldValue =
                heldFoam * remainingMask * (0.08 + density01 * 0.08) * foamContinuity * returnFade;
            const float crest = RippleLine(front, frontWidth);
            const float crestHalo = RippleLine(front, frontWidth * 1.70);
            const float trailDistance = max(0.0, front);
            const float returnFollowMask =
                1.0 - smoothstep(trailLength * 0.18, trailLength * 0.50, trailDistance);
            const float trailingFoam =
                exp(-trailDistance / max(trailLength * 0.42, 1.0e-4)) *
                smoothstep(frontWidth * 0.35, frontWidth * 1.70, trailDistance) *
                returnFollowMask;
            const float edgeFade = 1.0 - smoothstep(0.78, 1.0, returnProgress);
            // Mirror of incomingDeepSideMask: the receding crest keeps only
            // its shoreward half, matching the incoming front's lit width so
            // the outgoing wave reads slightly dimmer, never brighter.
            const float returnCrestSideMask =
                smoothstep(-frontWidth * 0.15, frontWidth * 0.35, front);
            const float edgeValue =
                ((crest * (0.62 + shoreBreakup * 0.20) * foamContinuity +
                  crestHalo * (0.10 + density01 * 0.06) * foamContinuity) *
                     returnCrestSideMask +
                 trailingFoam * (0.38 + density01 * 0.24) * breakup) *
                edgeFade;
            const float edgeIntroduce = smoothstep(0.0, 0.18, returnProgress);
            const float peakFront = x - shoreEnd;
            const float peakTrailDistance = max(0.0, -peakFront);
            const float peakCarryMask = 1.0 - smoothstep(-frontWidth * 0.35, frontWidth * 0.15, peakFront);
            const float peakCarryFoam =
                exp(-peakTrailDistance / max(trailLength * 0.42, 1.0e-4)) *
                smoothstep(frontWidth * 0.35, frontWidth * 1.70, peakTrailDistance) *
                (1.0 - smoothstep(trailLength * 0.18, trailLength * 0.50, peakTrailDistance));
            const float peakCarryFade = 1.0 - smoothstep(0.0, 0.20, returnProgress);
            // The carry reproduces the incoming front's exact end state —
            // crest, trailing foam, and body wash at the arrival level — and
            // only then fades, so the wave settles without a one-frame dip
            // while the dimmer receding front introduces underneath.
            const float peakCarryStartMask =
                smoothstep(offshoreStart - frontWidth * 1.25, offshoreStart + frontWidth * 1.25, x);
            const float peakCarryBodyFade =
                1.0 - smoothstep(waveTravel * 0.72, waveTravel * 1.02, peakTrailDistance);
            const float peakCarryBodyFoam =
                peakCarryMask * peakCarryStartMask * peakCarryBodyFade *
                (0.10 + density01 * 0.12) * foamContinuity;
            const float peakCarryEdgeFoam =
                (RippleLine(peakFront, frontWidth) * (0.70 + shoreBreakup * 0.22) * foamContinuity +
                 RippleLine(peakFront, frontWidth * 1.70) * (0.08 + density01 * 0.06) * foamContinuity +
                 peakCarryFoam * (0.42 + density01 * 0.28) * breakup) *
                peakCarryMask * peakCarryStartMask;
            const float peakCarryValue =
                max(peakCarryBodyFoam, peakCarryEdgeFoam) * peakArrivalLevel * peakCarryFade;
            const float value = max(max(heldValue, peakCarryValue), edgeValue * edgeIntroduce);
            combined = max(combined, value * shorewardMask * frontWeight);
        }
    }

    return clamp(combined, 0.0, 1.0);
}

float ContinuousBandShorelineWaveValue(
    vec2 uv,
    float shoreDistance,
    float reachMeters,
    float wavelength,
    float warp,
    float turbulence,
    float density,
    float seed,
    float phase) {
    const float safeReach = max(0.001, reachMeters);
    const float safeWavelength = max(0.002, wavelength);
    const float density01 = clamp(density, 0.0, 1.0);
    const float turbulence01 = clamp(turbulence, 0.0, 1.0);
    const float clampedWarp = clamp(warp, 0.0, 2.0);
    const float alongShore = uv.y;

    // Bay-beach jostle built from the Foam Fronts wave character: two to four
    // overlapping fronts share the whole reach instead of oscillating inside
    // fixed lanes. Every front runs a smooth periodic gather-push-recede
    // cycle with no inactive window and no phase reset, so somewhere along
    // the beach foam is always moving. A slow per-front vigour swell decides
    // how far each cycle reaches: vigorous cycles crash beside the waterline
    // and wash into the upper run-in zone, tired ones dissolve inside the
    // deeper jostling water while their neighbours keep moving.
    const float activeBandCount = mix(2.0, 4.0, density01);
    // Upper share of the reach where finished crashes wash out; everything
    // below it is the jostling body of the bay.
    const float runInDepth = safeReach * 0.30;
    const float frontWidth = max(
        safeWavelength * (0.052 + turbulence01 * 0.030),
        0.003);
    const float foamReach = max(
        safeWavelength * (0.55 + turbulence01 * 0.30),
        frontWidth * 5.0);
    const float lateralScale = max(safeWavelength * 1.35, 0.012);
    const float waveRate = 0.062 + density01 * 0.034;
    const float seedAngle = RippleHash(seed * 0.071 + 19.0) * kRippleTwoPi;
    float combined = 0.0;

    for (int bandIndex = 0; bandIndex < 4; ++bandIndex) {
        const float slot = float(bandIndex);
        const float bandWeight = 1.0 - smoothstep(
            activeBandCount - 0.35,
            activeBandCount + 0.15,
            slot + 0.5);
        const float slotSeed = seed + slot * 53.17;
        // Incommensurate cycle speeds plus a golden-angle stagger keep the
        // fronts from clumping into one shared quiet moment.
        const float speedScale =
            mix(0.72, 1.37, RippleHash(slotSeed * 0.097 + 23.0));
        const float waveAngle =
            phase * waveRate * speedScale * kRippleTwoPi +
            seedAngle + slot * 2.399963;
        // Alongshore lead and lag: one stretch of beach crashes while a
        // neighbouring stretch is still gathering, so fronts cross and
        // jostle instead of arriving as unbroken parallel bands.
        const float lateralPhase =
            sin((alongShore / (lateralScale * 3.1)) + seedAngle + slot * 1.91) *
                (0.55 + clampedWarp * 0.50) +
            sin(
                (alongShore / max(safeWavelength * 2.3, 0.02)) -
                seedAngle * 0.73 + slot * 2.37) *
                (0.22 + turbulence01 * 0.30);
        const float cycleAngle = waveAngle + lateralPhase;
        // Smooth asymmetric cycle: long gather, quicker push, lingering
        // recede. The second harmonic sharpens the push without introducing
        // any lifecycle reset, and the value and velocity agree everywhere.
        const float approach = clamp(
            0.5 - 0.5 * cos(cycleAngle) - 0.12 * sin(cycleAngle * 2.0),
            0.0,
            1.0);
        // Slow vigour swell, also varying along the shore, chooses between
        // crashing near the waterline and fading mid-band this cycle.
        const float vigor = 0.5 + 0.5 * sin(
            waveAngle * 0.237 + slotSeed * 0.61 +
            sin(alongShore / (lateralScale * 5.7) + slotSeed) * 0.8);
        const float crashDepth =
            mix(runInDepth * 1.30, runInDepth * 0.12, vigor);
        const float startDepth =
            safeReach * mix(0.66, 0.98, RippleHash(slotSeed * 0.061 + 47.0));
        const float waveCenter = mix(startDepth, crashDepth, approach);

        const float scallopNoise = RippleSmoothBlockNoise(
            vec2(
                alongShore + slot * safeWavelength * 0.37,
                seed * 0.13 + slot * 0.41),
            max(safeWavelength * 0.48, 0.008),
            seed,
            151.0 + slot * 19.0);
        const float frontWarpSignal = clamp(
            sin((alongShore / lateralScale) + seed * 1.17 + slot * 1.91) * 0.62 +
                sin(
                    (alongShore / max(safeWavelength * 0.58, 0.006)) -
                    seed * 0.73 + slot * 2.37) *
                    0.28 +
                (scallopNoise - 0.5) * 1.15,
            -1.0,
            1.0);
        const float frontWarp = frontWarpSignal *
            safeWavelength * (0.045 + clampedWarp * 0.060 + turbulence01 * 0.035);
        const float front = max(0.0, shoreDistance) + frontWarp - waveCenter;
        const float frontLocalCoordinate = front;

        // Front-local grain follows each band through both directions of the
        // oscillation instead of being resampled at a lifecycle boundary.
        const vec2 waveFrontStreakUv = vec2(
            frontLocalCoordinate * (2.05 + turbulence01 * 0.45) + slot * 0.19,
            alongShore * (0.11 + turbulence01 * 0.05) +
                sin(
                    frontLocalCoordinate / max(safeWavelength * 0.55, 0.006) +
                    slotSeed * 0.17) *
                    safeWavelength * 0.045 +
                slot * 0.31);
        const float waveFrontStreakNoise = RippleSmoothBlockNoise(
            waveFrontStreakUv,
            max(safeWavelength * 0.20, 0.006),
            seed,
            203.0 + slot * 23.0);
        const float foamMottleNoise = RippleSmoothBlockNoise(
            vec2(
                frontLocalCoordinate * 0.54 + slot * 0.19,
                alongShore * 0.62 + slot * 0.31),
            max(safeWavelength * 0.42, 0.007),
            seed,
            227.0 + slot * 29.0);
        const float foamNoise = RippleSmoothBlockNoise(
            mix(
                vec2(
                    frontLocalCoordinate * 0.54 + slot * 0.19,
                    alongShore * 0.62 + slot * 0.31),
                waveFrontStreakUv,
                0.78),
            max(safeWavelength * 0.24, 0.006),
            seed,
            251.0 + slot * 31.0);
        const float shoreBreakup = smoothstep(
            0.24,
            0.92,
            scallopNoise + turbulence01 * 0.22);
        const float breakup = smoothstep(
            0.18,
            0.96,
            waveFrontStreakNoise * 0.52 +
                foamNoise * 0.28 +
                foamMottleNoise * 0.18 +
                shoreBreakup * 0.24 +
                turbulence01 * 0.18);
        const float foamContinuity = mix(0.72, 1.0, breakup);
        const float crest = RippleLine(front, frontWidth);
        const float crestHalo = RippleLine(front, frontWidth * 1.70);
        const float foamDistance = abs(front);
        const float surroundingFoam =
            exp(-foamDistance / max(foamReach * 0.42, 1.0e-4)) *
            smoothstep(frontWidth * 0.35, frontWidth * 1.70, foamDistance) *
            (1.0 - smoothstep(foamReach * 0.40, foamReach, foamDistance));
        // Bright beside the waterline, faint while gathering offshore, and
        // never fully absent so the bay always carries moving foam.
        const float crashProximity = smoothstep(
            runInDepth * 1.8,
            runInDepth * 0.5,
            waveCenter);
        const float crestGain = mix(0.26, 1.0, crashProximity);
        // Crash wash: foam trailing shoreward of a front that reached the
        // run-in zone, blooming at the push's peak and draining as the front
        // recedes (the asymmetric cycle keeps `approach` high just after the
        // crash, so the wash lingers before it fades).
        const float washLength = foamReach * (0.7 + 0.8 * vigor);
        const float washFoam =
            exp(min(0.0, front) / max(washLength, 1.0e-4)) *
            smoothstep(frontWidth * 0.35, frontWidth * 1.55, -front) *
            crashProximity *
            smoothstep(0.55, 0.90, approach);
        const float value =
            (crest * (0.70 + shoreBreakup * 0.22) * foamContinuity * crestGain +
             crestHalo * (0.08 + density01 * 0.06) * foamContinuity * crestGain +
             surroundingFoam * (0.24 + density01 * 0.22) * breakup *
                 mix(0.55, 1.0, crashProximity) +
             washFoam * (0.34 + density01 * 0.24) * breakup) *
            bandWeight;
        combined = max(combined, value);
    }

    return clamp(combined, 0.0, 1.0);
}

float HeightFoamShorelineWaveValue(
    vec2 shoreUv,
    float worldZ,
    float runupZ,
    float breakZ,
    float offshoreReachMeters,
    float edgeFadeMeters,
    float patternScale,
    float wavelength,
    float warp,
    float turbulence,
    float density,
    float offshoreFoamStrength,
    float incomingStrength,
    float returnStrength,
    float seed,
    float phase) {
    const float safeReach = max(0.001, offshoreReachMeters);
    const float offshoreZ = runupZ - safeReach;
    const float safeWavelength = max(0.002, wavelength);
    const float turbulence01 = clamp(turbulence, 0.0, 1.0);
    const float density01 = clamp(density, 0.0, 1.0);
    const float clampedWarp = clamp(warp, 0.0, 3.0);
    const float safePatternScale = max(0.01, patternScale);
    const vec2 patternUv = shoreUv * safePatternScale;
    const float alongShore = patternUv.y;
    const float heightCoordinate = patternUv.x;
    const float breakFeather = max(edgeFadeMeters, safeWavelength * 0.16);
    const float seaFoamMask =
        1.0 - smoothstep(breakZ - breakFeather, breakZ + breakFeather, worldZ);
    const float offshoreEdgeMask =
        smoothstep(offshoreZ - max(edgeFadeMeters, 1.0e-5), offshoreZ + max(edgeFadeMeters, 1.0e-5), worldZ);

    const float reservoirNoise = RippleSmoothBlockNoise(
        vec2(
            heightCoordinate * 0.58 + phase * (0.030 + turbulence01 * 0.018),
            alongShore * 0.71 - phase * (0.019 + clampedWarp * 0.010)),
        max(safeWavelength * 0.42, 0.006),
        seed,
        311.0);
    const float reservoirMottle = RippleSmoothBlockNoise(
        vec2(
            heightCoordinate * 0.31 - phase * 0.017,
            alongShore * 0.43 + phase * 0.013),
        max(safeWavelength * 0.68, 0.008),
        seed,
        337.0);
    const float reservoirWarp =
        (sin(alongShore / max(safeWavelength * 1.35, 0.008) + seed * 0.71) * 0.62 +
         sin(alongShore / max(safeWavelength * 0.63, 0.006) - seed * 0.37) * 0.28 +
         (reservoirNoise - 0.5) * 0.85) *
        safeWavelength * (0.08 + clampedWarp * 0.12 + turbulence01 * 0.08);
    const float ridgePhaseA =
        ((heightCoordinate + reservoirWarp) / safeWavelength) * 6.28318530718 - phase * 0.16;
    const float ridgePhaseB =
        ((heightCoordinate * 0.61 - reservoirWarp * 0.72) / safeWavelength) * 6.28318530718 + phase * 0.11 + 1.73;
    const float ridgeThreshold = mix(0.78, 0.48, density01);
    const float ridgeA = smoothstep(
        ridgeThreshold,
        0.98,
        0.5 + 0.5 * sin(ridgePhaseA) + (reservoirNoise - 0.5) * (0.18 + turbulence01 * 0.18));
    const float ridgeB = smoothstep(
        min(0.92, ridgeThreshold + 0.08),
        0.99,
        0.5 + 0.5 * sin(ridgePhaseB) + (reservoirMottle - 0.5) * 0.22);
    const float foamPotential =
        max(ridgeA * 0.82, ridgeB * 0.58) * mix(0.55, 1.0, reservoirMottle);
    const float persistentFoam =
        foamPotential * seaFoamMask * offshoreEdgeMask * max(0.0, offshoreFoamStrength);

    const float incomingShare = 0.60;
    const float returnShare = 0.25;
    const float activeEnd = incomingShare + returnShare;
    const float waveRate = 0.058 + density01 * 0.044;
    const float frontWidth = max(safeWavelength * (0.055 + turbulence01 * 0.035), 0.003);
    const float trailLength = max(safeWavelength * (0.72 + turbulence01 * 0.58), frontWidth * 7.0);
    float movingBands = 0.0;
    float wakeClear = 0.0;

    for (int waveIndex = 0; waveIndex < 4; ++waveIndex) {
        const float slot = float(waveIndex);
        const float slotSeed = seed + slot * 59.31;
        const float timingNoise = RippleHash(slotSeed * 0.071 + 17.0);
        const float speedNoise = mix(0.91, 1.09, RippleHash(slotSeed * 0.097 + 29.0));
        const float waveGate = RippleHash(slotSeed * 0.113 + 43.0);
        if (waveGate > mix(0.62, 1.0, density01)) {
            continue;
        }

        const float offset =
            slot * 0.235 +
            (timingNoise - 0.5) * (0.10 + turbulence01 * 0.10) +
            RippleSmoothBlockNoise(vec2(phase * 0.018, slotSeed), 0.23, seed, 359.0) * 0.08;
        const float cycle = fract(phase * waveRate * speedNoise + offset);
        if (cycle >= activeEnd) {
            continue;
        }

        const float lateralNoise = RippleSmoothBlockNoise(
            vec2(alongShore + slot * safeWavelength * 0.41, slotSeed * 0.19),
            max(safeWavelength * 0.44, 0.007),
            seed,
            383.0 + slot * 17.0);
        const float frontWarpZ =
            (sin(alongShore / max(safeWavelength * 1.28, 0.008) + slotSeed * 0.13) * 0.58 +
             sin(alongShore / max(safeWavelength * 0.57, 0.006) - slotSeed * 0.09) * 0.26 +
             (lateralNoise - 0.5) * 1.08) *
            safeWavelength * (0.05 + clampedWarp * 0.075 + turbulence01 * 0.045);

        if (cycle < incomingShare) {
            const float incomingProgress = smoothstep(0.0, 1.0, cycle / incomingShare);
            const float frontZ = mix(offshoreZ, runupZ, incomingProgress) + frontWarpZ;
            const float signedFrontDistance = worldZ - frontZ;
            const float crest = RippleLine(signedFrontDistance, frontWidth);
            const float crestHalo = RippleLine(signedFrontDistance, frontWidth * 1.75);
            const float seaSideDistance = max(0.0, frontZ - worldZ);
            const float foamTail =
                exp(-seaSideDistance / max(trailLength * 0.48, 1.0e-4)) *
                smoothstep(frontWidth * 0.30, frontWidth * 1.55, seaSideDistance) *
                (1.0 - smoothstep(trailLength * 0.55, trailLength, seaSideDistance));
            const float breakGain = smoothstep(
                breakZ - breakFeather,
                breakZ + breakFeather,
                frontZ);
            const float arrivalFade = smoothstep(0.01, 0.12, cycle);
            const float finishFade = 1.0 - smoothstep(0.90, 1.0, incomingProgress) * 0.24;
            const float gatherPotential = mix(0.62, 1.0, foamPotential);
            const float incomingBand =
                (crest * mix(0.48, 0.94, breakGain) * gatherPotential +
                 crestHalo * mix(0.08, 0.16, breakGain) +
                 foamTail * mix(0.20, 0.48, breakGain) * mix(0.70, 1.0, lateralNoise)) *
                max(0.0, incomingStrength) * arrivalFade * finishFade;
            movingBands = max(movingBands, incomingBand);

            const float passedFront =
                1.0 - smoothstep(-frontWidth * 0.25, frontWidth * 0.80, signedFrontDistance);
            const float recentWake =
                1.0 - smoothstep(frontWidth * 1.5, trailLength * 1.35, seaSideDistance);
            wakeClear = max(wakeClear, passedFront * recentWake * smoothstep(0.08, 0.42, incomingProgress));
        } else {
            const float returnProgress =
                smoothstep(0.0, 1.0, (cycle - incomingShare) / returnShare);
            const float frontZ = mix(runupZ, offshoreZ, returnProgress) + frontWarpZ;
            const float signedFrontDistance = worldZ - frontZ;
            const float crest = RippleLine(signedFrontDistance, frontWidth * 1.08);
            const float landSideDistance = max(0.0, worldZ - frontZ);
            const float returnTail =
                exp(-landSideDistance / max(trailLength * 0.45, 1.0e-4)) *
                smoothstep(frontWidth * 0.30, frontWidth * 1.55, landSideDistance) *
                (1.0 - smoothstep(trailLength * 0.48, trailLength * 0.92, landSideDistance));
            const float returnFade = 1.0 - smoothstep(0.05, 1.0, returnProgress);
            const float returnBand =
                (crest * 0.52 + returnTail * 0.28) *
                max(0.0, incomingStrength) * clamp(returnStrength, 0.0, 1.0) * returnFade;
            movingBands = max(movingBands, returnBand);
            wakeClear = max(
                wakeClear,
                (1.0 - smoothstep(-frontWidth, trailLength, signedFrontDistance)) *
                    (1.0 - returnProgress) * 0.45);
        }
    }

    const float recoveredReservoir = persistentFoam * (1.0 - clamp(wakeClear, 0.0, 1.0) * 0.68);
    return clamp(max(recoveredReservoir, movingBands), 0.0, 1.0);
}

SparseRippleComposite EmptySparseRippleComposite() {
    SparseRippleComposite result;
    result.scale = 0.0;
    result.colourMix = 0.0;
    result.emissionAdd = 0.0;
    result.opacityAdd = 0.0;
    result.opacityMultiply = 1.0;
    result.pointSizeAdd = 0.0;
    result.pointSizeMultiply = 1.0;
    result.colour = vec3(0.62, 0.88, 1.0);
    return result;
}

SparseRippleComposite SanitizeSparseRippleComposite(SparseRippleComposite value) {
    // A malformed or only-partially-published effect must never poison the
    // base point pass. NaN point sizes and opacity values have undefined
    // rasterisation/blending behaviour and can make an otherwise valid ROCK
    // cloud disappear for an entire in-flight frame.
    if (!RippleFiniteFloat(value.scale) ||
        !RippleFiniteFloat(value.colourMix) ||
        !RippleFiniteFloat(value.emissionAdd) ||
        !RippleFiniteFloat(value.opacityAdd) ||
        !RippleFiniteFloat(value.opacityMultiply) ||
        !RippleFiniteFloat(value.pointSizeAdd) ||
        !RippleFiniteFloat(value.pointSizeMultiply) ||
        !RippleFiniteVec3(value.colour)) {
        return EmptySparseRippleComposite();
    }
    // Preserve the established finite composition ranges. This helper is a
    // failure-containment boundary, not a new visual limiter.
    value.colourMix = clamp(value.colourMix, 0.0, 1.0);
    value.opacityMultiply = max(0.0, value.opacityMultiply);
    value.pointSizeMultiply = max(0.0, value.pointSizeMultiply);
    return value;
}

float SeepageNoiseHash01(int x, int y, uint seed) {
    uint hash = SeepageCellHash(ivec3(x, y, int(seed)));
    hash ^= seed * 0x9e3779b9u;
    hash ^= hash >> 16u;
    hash *= 0x7feb352du;
    hash ^= hash >> 15u;
    return float(hash & 0x00ffffffu) / float(0x01000000u);
}

// Area envelope shared by every membership path (CPU mirror:
// EvaluateSeepageAreaEnvelope in src/water/WaterFlow.cpp — keep the
// constants in sync). Node strength shapes WHERE seepage lives: the run
// scales with strength and travels further on near-vertical surfaces, while
// the half-width starts at the authored node width and spreads outward with
// travelled distance — faster on flat surfaces, slower on walls. Prominence
// only scales how strongly the effect is applied.
float SeepageResolvedFanMask(
    uint nodeIndex,
    vec3 pointNormal,
    float downstreamDistance,
    float signedLateralDistance,
    float planeDistance,
    vec3 surfaceNormal,
    float reachBound,
    out float effectiveReach,
    out float effectiveHalfWidth,
    out float lateralNormalised) {
    effectiveReach = 0.0;
    effectiveHalfWidth = 0.0;
    lateralNormalised = 0.0;
    const float surfaceThickness = max(
        1e-4,
        seepageNodeData.seepageNodes[nodeIndex].normalSurface.w);
    const float edgeFeather = max(1e-4, seepageNodeData.seepageNodes[nodeIndex].downEdge.w);
    const float steep = clamp(1.0 - abs(surfaceNormal.z), 0.0, 1.0);
    const float strength = clamp(
        seepageParamData.seepageParams[nodeIndex].geometry.z,
        0.0,
        8.0);
    const float liveReach = max(
        0.0,
        seepageParamData.seepageParams[nodeIndex].liveGeometry.x);
    const float reachLimit = max(
        0.0,
        seepageNodeData.seepageNodes[nodeIndex].positionReach.w);
    const float reachRun = min(
        clamp(liveReach * strength * mix(0.45, 1.15, steep), 0.0, reachLimit),
        reachBound);
    const float spreadRate = 0.18 * strength * mix(1.60, 0.45, steep);
    const float halfWidthLimit = max(
        0.0,
        seepageNodeData.seepageNodes[nodeIndex].geometry.x);
    const float halfWidth = clamp(
        seepageParamData.seepageParams[nodeIndex].liveGeometry.y * 0.5 +
            max(downstreamDistance, 0.0) * spreadRate,
        0.0,
        halfWidthLimit);
    const float endFeather = max(edgeFeather, reachRun * 0.10);
    const float lateralFeather = max(edgeFeather, halfWidth * 0.30);
    effectiveReach = reachRun;
    effectiveHalfWidth = halfWidth;
    // A run collapsed by the guide bound or animation contributes nothing
    // (CPU parity: the guided evaluator early-outs the same way).
    if (reachRun <= 1e-5) {
        return 0.0;
    }
    if (downstreamDistance < -edgeFeather || downstreamDistance > reachRun + endFeather) {
        return 0.0;
    }
    if (planeDistance > surfaceThickness + edgeFeather) {
        return 0.0;
    }
    if (abs(signedLateralDistance) > halfWidth + lateralFeather) {
        return 0.0;
    }
    lateralNormalised = signedLateralDistance / max(halfWidth, 1e-4);

    const float headMask = smoothstep(-edgeFeather, 0.0, downstreamDistance);
    const float tailMask = 1.0 - smoothstep(
        reachRun - endFeather,
        reachRun + endFeather,
        downstreamDistance);
    const float sideMask = 1.0 - smoothstep(
        halfWidth,
        halfWidth + lateralFeather,
        abs(signedLateralDistance));
    const float surfaceMask =
        1.0 - smoothstep(surfaceThickness, surfaceThickness + edgeFeather, planeDistance);
    const vec3 resolvedPointNormal =
        styleData.pointMeta.z != 0u && dot(pointNormal, pointNormal) > 1e-8
            ? normalize(pointNormal)
            : surfaceNormal;
    const float normalAlignment = abs(dot(resolvedPointNormal, surfaceNormal));
    const float aligned = smoothstep(0.15, 0.85, normalAlignment);
    const float normalMask = mix(
        1.0,
        aligned,
        clamp(seepageParamData.seepageParams[nodeIndex].geometry.y, 0.0, 1.0));
    return clamp(headMask * tailMask * sideMask * surfaceMask * normalMask, 0.0, 1.0);
}

float SeepagePlanarFanMask(
    uint nodeIndex,
    vec3 worldPosition,
    vec3 pointNormal,
    out float downstreamDistance,
    out float lateralDistance,
    out float effectiveReach,
    out float effectiveHalfWidth,
    out float lateralNormalised,
    out vec3 resolvedSurfaceNormal,
    out vec3 resolvedDownTangent) {
    const vec3 nodeNormal = RippleSafeNormal(
        seepageNodeData.seepageNodes[nodeIndex].normalSurface.xyz);
    vec3 down = seepageNodeData.seepageNodes[nodeIndex].downEdge.xyz;
    down -= nodeNormal * dot(down, nodeNormal);
    down = dot(down, down) > 1e-8 ? normalize(down) : vec3(0.0, 0.0, -1.0);
    resolvedSurfaceNormal = nodeNormal;
    resolvedDownTangent = down;
    vec3 lateral = seepageNodeData.seepageNodes[nodeIndex].lateralStart.xyz;
    lateral -= nodeNormal * dot(lateral, nodeNormal);
    lateral -= down * dot(lateral, down);
    if (dot(lateral, lateral) <= 1e-8) {
        lateral = cross(nodeNormal, down);
    }
    lateral = dot(lateral, lateral) > 1e-8 ? normalize(lateral) : RippleSafeLateral(down);

    const vec3 relative =
        worldPosition - seepageNodeData.seepageNodes[nodeIndex].positionReach.xyz;
    downstreamDistance = dot(relative, down);
    lateralDistance = dot(relative, lateral);
    const float planeDistance = abs(dot(relative, nodeNormal));
    return SeepageResolvedFanMask(
        nodeIndex,
        pointNormal,
        downstreamDistance,
        lateralDistance,
        planeDistance,
        nodeNormal,
        3.402823466e+38,
        effectiveReach,
        effectiveHalfWidth,
        lateralNormalised);
}

float SeepageFanMask(
    uint nodeIndex,
    vec3 worldPosition,
    vec3 pointNormal,
    out float downstreamDistance,
    out float lateralDistance,
    out float effectiveReach,
    out float effectiveHalfWidth,
    out float lateralNormalised,
    out vec3 resolvedSurfaceNormal,
    out vec3 resolvedDownTangent) {
    downstreamDistance = 0.0;
    lateralDistance = 0.0;
    effectiveReach = 0.0;
    effectiveHalfWidth = 0.0;
    lateralNormalised = 0.0;
    resolvedSurfaceNormal = RippleSafeNormal(
        seepageNodeData.seepageNodes[nodeIndex].normalSurface.xyz);
    resolvedDownTangent = RippleSafeNormal(
        seepageNodeData.seepageNodes[nodeIndex].downEdge.xyz);

    const float edgeFeather = max(
        1e-4,
        seepageNodeData.seepageNodes[nodeIndex].downEdge.w);
    if (worldPosition.z >
        seepageNodeData.seepageNodes[nodeIndex].positionReach.z + edgeFeather) {
        return 0.0;
    }

    const uint guideSampleCount = min(
        seepageNodeData.seepageNodes[nodeIndex].guideControl.x,
        8u);
    if (guideSampleCount < 2u) {
        return SeepagePlanarFanMask(
            nodeIndex,
            worldPosition,
            pointNormal,
            downstreamDistance,
            lateralDistance,
            effectiveReach,
            effectiveHalfWidth,
            lateralNormalised,
            resolvedSurfaceNormal,
            resolvedDownTangent);
    }

    float bestDistanceSquared = 3.402823466e+38;
    float bestRawAmount = 0.0;
    float bestAmount = 0.0;
    float bestPlaneDistance = 0.0;
    float bestLateralDistance = 0.0;
    vec3 bestSurfaceNormal = RippleSafeNormal(
        seepageNodeData.seepageNodes[nodeIndex].normalSurface.xyz);
    uint bestSegmentIndex = 0u;
    bool foundSegment = false;

    for (uint segmentIndex = 0u; segmentIndex < 7u; ++segmentIndex) {
        if (segmentIndex + 1u >= guideSampleCount) {
            break;
        }
        const vec4 startSample =
            seepageNodeData.seepageNodes[nodeIndex].guidePositionStation[segmentIndex];
        const vec4 endSample =
            seepageNodeData.seepageNodes[nodeIndex].guidePositionStation[segmentIndex + 1u];
        const vec3 segment = endSample.xyz - startSample.xyz;
        const float segmentLengthSquared = dot(segment, segment);
        if (segmentLengthSquared <= 1e-10) {
            continue;
        }

        const float rawAmount = dot(worldPosition - startSample.xyz, segment) / segmentLengthSquared;
        const float amount = clamp(rawAmount, 0.0, 1.0);
        const vec3 center = mix(startSample.xyz, endSample.xyz, amount);
        const vec3 centerDelta = worldPosition - center;
        const float distanceSquared = dot(centerDelta, centerDelta);
        if (foundSegment && distanceSquared + 1e-8 >= bestDistanceSquared) {
            continue;
        }

        vec3 startNormal = RippleSafeNormal(
            seepageNodeData.seepageNodes[nodeIndex].guideNormalConfidence[segmentIndex].xyz);
        vec3 endNormal = RippleSafeNormal(
            seepageNodeData.seepageNodes[nodeIndex].guideNormalConfidence[segmentIndex + 1u].xyz);
        if (dot(startNormal, endNormal) < 0.0) {
            endNormal = -endNormal;
        }
        const vec3 surfaceNormal = RippleSafeNormal(mix(startNormal, endNormal, amount));
        const vec3 tangent = normalize(segment);
        vec3 lateral = cross(surfaceNormal, tangent);
        if (dot(lateral, lateral) <= 1e-8) {
            lateral = seepageNodeData.seepageNodes[nodeIndex].lateralStart.xyz;
        }
        lateral = dot(lateral, lateral) > 1e-8
                      ? normalize(lateral)
                      : RippleSafeLateral(tangent);
        if (dot(
                lateral,
                seepageNodeData.seepageNodes[nodeIndex].lateralStart.xyz) < 0.0) {
            lateral = -lateral;
        }

        foundSegment = true;
        bestDistanceSquared = distanceSquared;
        bestRawAmount = rawAmount;
        bestAmount = amount;
        bestPlaneDistance = abs(dot(centerDelta, surfaceNormal));
        bestLateralDistance = dot(centerDelta, lateral);
        bestSurfaceNormal = surfaceNormal;
        bestSegmentIndex = segmentIndex;
    }

    if (!foundSegment) {
        return SeepagePlanarFanMask(
            nodeIndex,
            worldPosition,
            pointNormal,
            downstreamDistance,
            lateralDistance,
            effectiveReach,
            effectiveHalfWidth,
            lateralNormalised,
            resolvedSurfaceNormal,
            resolvedDownTangent);
    }

    float stationAmount = bestAmount;
    if ((bestSegmentIndex == 0u && bestRawAmount < 0.0) ||
        (bestSegmentIndex + 2u == guideSampleCount && bestRawAmount > 1.0)) {
        stationAmount = bestRawAmount;
    }
    const vec4 stationStart =
        seepageNodeData.seepageNodes[nodeIndex].guidePositionStation[bestSegmentIndex];
    const vec4 stationEnd =
        seepageNodeData.seepageNodes[nodeIndex].guidePositionStation[bestSegmentIndex + 1u];
    downstreamDistance = mix(stationStart.w, stationEnd.w, stationAmount);
    lateralDistance = bestLateralDistance;
    resolvedSurfaceNormal = bestSurfaceNormal;
    resolvedDownTangent = RippleSafeNormal(
        stationEnd.xyz - stationStart.xyz);
    // The guide cannot represent stations beyond what it traced, so the
    // envelope run is additionally bounded by the achieved guide extent.
    const float achievedReach = max(
        0.0,
        uintBitsToFloat(seepageNodeData.seepageNodes[nodeIndex].guideControl.y));
    const float lastStation = max(
        0.0,
        seepageNodeData.seepageNodes[nodeIndex]
            .guidePositionStation[guideSampleCount - 1u]
            .w);
    return SeepageResolvedFanMask(
        nodeIndex,
        pointNormal,
        downstreamDistance,
        lateralDistance,
        bestPlaneDistance,
        bestSurfaceNormal,
        min(achievedReach, lastStation),
        effectiveReach,
        effectiveHalfWidth,
        lateralNormalised);
}

vec3 DecodeSeepageSupportNormal(uint packed) {
    vec2 octahedral = vec2(
        float(packed & 0x3ffu),
        float((packed >> 10u) & 0x3ffu)) / 1023.0 * 2.0 - 1.0;
    vec3 normal = vec3(
        octahedral,
        1.0 - abs(octahedral.x) - abs(octahedral.y));
    if (normal.z < 0.0) {
        const vec2 original = normal.xy;
        normal.x = (1.0 - abs(original.y)) * (original.x < 0.0 ? -1.0 : 1.0);
        normal.y = (1.0 - abs(original.x)) * (original.y < 0.0 ? -1.0 : 1.0);
    }
    return RippleSafeNormal(normal);
}

// Least-resistance membership: immutable support stores flood cost plus
// geometric flow-run/cross-contour distances. Live Strength changes only the
// compact budget and width masks. CPU mirror:
// EvaluateConnectedSeepageLiveMask in src/water/WaterFlow.cpp.
float SeepageConnectedSupportMask(
    uint nodeIndex,
    SeepageNodeReference reference,
    vec3 worldPosition,
    vec3 pointNormal,
    out float downstreamDistance,
    out float lateralDistance,
    out float effectiveReach,
    out float effectiveHalfWidth,
    out float lateralNormalised,
    out vec3 resolvedSurfaceNormal,
    out vec3 resolvedDownTangent) {
    const float rawCost = reference.leastResistanceCost;
    const float cost = RippleFiniteFloat(rawCost) ? max(0.0, rawCost) : 0.0;
    vec2 runCross = unpackHalf2x16(reference.packedRunCross);
    runCross = vec2(
        RippleFiniteFloat(runCross.x) ? max(0.0, runCross.x) : 0.0,
        RippleFiniteFloat(runCross.y) ? max(0.0, runCross.y) : 0.0);
    const float run = runCross.x;
    const float crossContour = runCross.y;
    const uint supportFlags =
        (reference.packedNormalRoleConfidenceFlags >> 30u) & 0x3u;
    const bool upstream = (supportFlags & 0x2u) != 0u;
    // The upstream wick is a persistent damp source reveal, not a travelling
    // wave lane. Retain its run for taper/membership but keep procedural
    // downstream animation at the source.
    downstreamDistance = upstream ? 0.0 : run;
    resolvedSurfaceNormal = DecodeSeepageSupportNormal(
        reference.packedNormalRoleConfidenceFlags);

    // Re-project gravity at every support cell so the animation follows
    // ledges and protrusions rather than remaining vertical in the node's
    // original tangent plane.
    const vec3 gravity = vec3(0.0, 0.0, -1.0);
    vec3 localDown =
        gravity -
        resolvedSurfaceNormal * dot(gravity, resolvedSurfaceNormal);
    if (!RippleFiniteVec3(localDown) || dot(localDown, localDown) <= 1e-8) {
        const vec3 nodeDown =
            seepageNodeData.seepageNodes[nodeIndex].downEdge.xyz;
        localDown =
            nodeDown -
            resolvedSurfaceNormal * dot(nodeDown, resolvedSurfaceNormal);
    }
    if (!RippleFiniteVec3(localDown) || dot(localDown, localDown) <= 1e-8) {
        const vec3 helper =
            abs(resolvedSurfaceNormal.x) < 0.8
                ? vec3(1.0, 0.0, 0.0)
                : vec3(0.0, 1.0, 0.0);
        localDown = cross(resolvedSurfaceNormal, helper);
    }
    resolvedDownTangent = RippleSafeNormal(localDown);
    vec3 localLateral = RippleSafeNormal(
        cross(resolvedSurfaceNormal, resolvedDownTangent));
    const vec3 nodeLateral = RippleSafeNormal(
        seepageNodeData.seepageNodes[nodeIndex].lateralStart.xyz);
    if (dot(localLateral, nodeLateral) < 0.0) {
        localLateral = -localLateral;
    }
    lateralDistance = dot(
        worldPosition - seepageNodeData.seepageNodes[nodeIndex].positionReach.xyz,
        localLateral);

    const float budget = max(0.0, seepageParamData.seepageParams[nodeIndex].liveGeometry.w);
    const float sourceRadius =
        max(0.0, seepageParamData.seepageParams[nodeIndex].liveGeometry.y) * 0.5;
    if (budget <= 1e-5) {
        effectiveReach = 0.0;
        effectiveHalfWidth = 0.0;
        lateralNormalised = 0.0;
        return 0.0;
    }

    const float cellSize = max(1e-5, styleData.seepageGridParams.x);
    const float selectionHalfWidth = max(
        0.0,
        seepageNodeData.seepageNodes[nodeIndex].geometry.x);
    const float authoredFeather = max(
        0.0,
        seepageNodeData.seepageNodes[nodeIndex].downEdge.w);
    // Reveal the real resident high tip back toward the node over the first
    // 12% of the immutable budget range, then release the downhill front.
    // This mirrors EvaluateConnectedSeepageLiveMask exactly.
    const float maximumBudget = max(
        cellSize,
        max(
            0.0,
            seepageNodeData.seepageNodes[nodeIndex].positionReach.w));
    const float sequenceProgress = clamp(
        budget / maximumBudget,
        0.0,
        1.0);
    const float maximumUpstreamRun = max(
        0.0,
        seepageNodeData.seepageNodes[nodeIndex].geometry.y);
    const float upstreamLeadFraction =
        maximumUpstreamRun > cellSize * 0.5
            ? 0.12
            : 0.0;
    const float routeProgress =
        upstream
            ? (upstreamLeadFraction > 0.0
                   ? clamp(
                         sequenceProgress / upstreamLeadFraction,
                         0.0,
                         1.0)
                   : 0.0)
            : (upstreamLeadFraction > 0.0
                   ? clamp(
                         (sequenceProgress - upstreamLeadFraction) /
                             (1.0 - upstreamLeadFraction),
                         0.0,
                         1.0)
                   : sequenceProgress);
    const float routeBudget =
        maximumBudget * routeProgress;
    const float sourceFeather = max(
        1e-5,
        min(authoredFeather, cellSize * 2.0));

    const vec3 projectedGravity =
        gravity -
        resolvedSurfaceNormal * dot(gravity, resolvedSurfaceNormal);
    const float steepness = clamp(length(projectedGravity), 0.0, 1.0);
    const float steepBlend = smoothstep(0.35, 0.75, steepness);

    float budgetMask = 0.0;
    float allowedHalfWidth = sourceRadius;
    if (upstream) {
        const float upstreamExtent =
            max(cellSize, maximumUpstreamRun);
        const float reverseFrontRun =
            upstreamExtent * (1.0 - routeProgress);
        const float reverseFrontFeather = max(
            cellSize,
            min(
                max(cellSize, authoredFeather),
                upstreamExtent * 0.15));
        budgetMask =
            reverseFrontRun <= 1e-5
                ? 1.0
                : smoothstep(
                      max(
                          0.0,
                          reverseFrontRun - reverseFrontFeather),
                      reverseFrontRun,
                      run);
        // Intensity builds with the reveal instead of saturating behind the
        // front (CPU parity: EvaluateConnectedSeepageLiveMask), reaching one
        // exactly when the front meets the node.
        budgetMask *= smoothstep(0.0, 1.0, routeProgress);

        const float nodewardStation = clamp(
            1.0 - run / upstreamExtent,
            0.0,
            1.0);
        const float tipHalfWidth =
            min(sourceRadius, cellSize * 0.5);
        const float fullTaperHalfWidth = mix(
            tipHalfWidth,
            sourceRadius,
            nodewardStation);
        const float centrelineHalfWidth = mix(
            tipHalfWidth,
            sourceRadius,
            nodewardStation * nodewardStation);
        float lateralFill = 0.0;
        if (routeProgress >= 1.0 - 1e-5) {
            const float effectiveStrength = clamp(
                seepageParamData.seepageParams[nodeIndex].geometry.z,
                0.0,
                1.0);
            const float completionStrength =
                sequenceProgress > 1e-5
                    ? clamp(
                          effectiveStrength *
                              upstreamLeadFraction /
                              sequenceProgress,
                          0.0,
                          1.0)
                    : effectiveStrength;
            lateralFill =
                completionStrength >= 1.0 - 1e-6
                    ? (effectiveStrength >= 1.0 - 1e-6
                           ? 1.0
                           : 0.0)
                    : smoothstep(
                          completionStrength,
                          1.0,
                          effectiveStrength);
        }
        allowedHalfWidth = mix(
            centrelineHalfWidth,
            fullTaperHalfWidth,
            lateralFill);
        effectiveReach = upstreamExtent;
    } else {
        const float frontFeather = max(
            max(1e-5, authoredFeather),
            routeBudget * 0.15);
        budgetMask =
            routeBudget > 1e-5
                ? 1.0 - smoothstep(
                      max(0.0, routeBudget - frontFeather),
                      routeBudget,
                      cost)
                : 0.0;
        allowedHalfWidth +=
            run * mix(0.55, 0.10, steepBlend);
        // The live budget is in least-resistance cost-metres. Patterns need
        // the corresponding physical pure-downstream run so keyed fronts
        // traverse steep support continuously instead of stopping at 75%.
        const float pureRouteCostPerMeter =
            mix(2.2, 0.75, steepBlend);
        effectiveReach =
            routeBudget / max(1e-5, pureRouteCostPerMeter);
    }
    allowedHalfWidth = clamp(
        allowedHalfWidth,
        0.0,
        selectionHalfWidth);
    effectiveHalfWidth = allowedHalfWidth;
    lateralNormalised =
        lateralDistance / max(effectiveHalfWidth, 1e-4);
    const float lateralFeather = max(
        1e-5,
        min(
            authoredFeather,
            max(cellSize * 2.0, allowedHalfWidth * 0.25)));
    const float widthMask = 1.0 - smoothstep(
        allowedHalfWidth,
        allowedHalfWidth + lateralFeather,
        crossContour);
    const float sourceDistance = length(vec2(run, crossContour));
    const float sourceMask = 1.0 - smoothstep(
        sourceRadius,
        sourceRadius + sourceFeather,
        sourceDistance);
    const bool upstreamReachedNode =
        upstreamLeadFraction <= 0.0 ||
        sequenceProgress + 1e-6 >= upstreamLeadFraction;
    // The below-node source disk eases in after the upstream front arrives
    // (CPU parity: EvaluateConnectedSeepageLiveMask) instead of switching on
    // in a single step at the release strength.
    const float releaseEase =
        upstreamLeadFraction > 0.0
            ? smoothstep(
                  upstreamLeadFraction,
                  upstreamLeadFraction * 1.5,
                  sequenceProgress)
            : smoothstep(0.0, 0.04, sequenceProgress);
    const float releasedSourceMask =
        !upstream && upstreamReachedNode
            ? sourceMask * releaseEase
            : 0.0;
    const float coreMask = clamp(
        max(releasedSourceMask, budgetMask * widthMask),
        0.0,
        1.0);
    if (coreMask <= 1e-6) {
        return 0.0;
    }
    const vec3 resolvedPointNormal =
        styleData.pointMeta.z != 0u && dot(pointNormal, pointNormal) > 1e-8
            ? normalize(pointNormal)
            : resolvedSurfaceNormal;
    const float normalAgreement = abs(dot(resolvedPointNormal, resolvedSurfaceNormal));
    const float aligned = smoothstep(0.15, 0.85, normalAgreement);
    const float normalMask = mix(
        1.0,
        aligned,
        clamp(seepageParamData.seepageParams[nodeIndex].geometry.y, 0.0, 1.0));
    const float confidence =
        float((reference.packedNormalRoleConfidenceFlags >> 20u) & 0xffu) / 255.0;
    return clamp(coreMask * normalMask * mix(0.65, 1.0, confidence), 0.0, 1.0);
}

uvec4 SeepageLookControl(uint nodeIndex, bool transition) {
    return transition
               ? seepageParamData.seepageParams[nodeIndex].transitionLook.control
               : seepageParamData.seepageParams[nodeIndex].look.control;
}

vec4 SeepageLookLegacy1(uint nodeIndex, bool transition) {
    return transition
               ? seepageParamData.seepageParams[nodeIndex].transitionLook.legacy1
               : seepageParamData.seepageParams[nodeIndex].look.legacy1;
}

vec4 SeepageLookLegacy0(uint nodeIndex, bool transition) {
    return transition
               ? seepageParamData.seepageParams[nodeIndex].transitionLook.legacy0
               : seepageParamData.seepageParams[nodeIndex].look.legacy0;
}

vec4 SeepageLookResponse0(uint nodeIndex, bool transition) {
    return transition
               ? seepageParamData.seepageParams[nodeIndex].transitionLook.response0
               : seepageParamData.seepageParams[nodeIndex].look.response0;
}

vec4 SeepageLookResponse1(uint nodeIndex, bool transition) {
    return transition
               ? seepageParamData.seepageParams[nodeIndex].transitionLook.response1
               : seepageParamData.seepageParams[nodeIndex].look.response1;
}

vec4 SeepageLookResponse2(uint nodeIndex, bool transition) {
    return transition
               ? seepageParamData.seepageParams[nodeIndex].transitionLook.response2
               : seepageParamData.seepageParams[nodeIndex].look.response2;
}

vec4 SeepageLookOrganic0(uint nodeIndex, bool transition) {
    return transition
               ? seepageParamData.seepageParams[nodeIndex].transitionLook.organic0
               : seepageParamData.seepageParams[nodeIndex].look.organic0;
}

vec4 SeepageLookOrganic1(uint nodeIndex, bool transition) {
    return transition
               ? seepageParamData.seepageParams[nodeIndex].transitionLook.organic1
               : seepageParamData.seepageParams[nodeIndex].look.organic1;
}

vec4 SeepageLookOrganic2(uint nodeIndex, bool transition) {
    return transition
               ? seepageParamData.seepageParams[nodeIndex].transitionLook.organic2
               : seepageParamData.seepageParams[nodeIndex].look.organic2;
}

vec4 SeepageLookOrganic3(uint nodeIndex, bool transition) {
    return transition
               ? seepageParamData.seepageParams[nodeIndex].transitionLook.organic3
               : seepageParamData.seepageParams[nodeIndex].look.organic3;
}

vec3 SeepageLookEnvironment(uint nodeIndex, bool transition) {
    const vec3 direction = transition
                               ? seepageParamData.seepageParams[nodeIndex]
                                     .transitionLook.environmentDirection.xyz
                               : seepageParamData.seepageParams[nodeIndex]
                                     .look.environmentDirection.xyz;
    return RippleSafeNormal(direction);
}

uint SeepageHash3(ivec3 coordinate, uint seed) {
    uint hash = SeepageCellHash(coordinate);
    hash ^= seed * 0x9e3779b9u;
    hash ^= hash >> 16u;
    hash *= 0x7feb352du;
    hash ^= hash >> 15u;
    hash *= 0x846ca68bu;
    hash ^= hash >> 16u;
    return hash;
}

vec3 SeepageGradient3(uint hash) {
    const uint index = hash % 12u;
    vec3 gradient;
    if (index == 0u) gradient = vec3(1.0, 1.0, 0.0);
    else if (index == 1u) gradient = vec3(-1.0, 1.0, 0.0);
    else if (index == 2u) gradient = vec3(1.0, -1.0, 0.0);
    else if (index == 3u) gradient = vec3(-1.0, -1.0, 0.0);
    else if (index == 4u) gradient = vec3(1.0, 0.0, 1.0);
    else if (index == 5u) gradient = vec3(-1.0, 0.0, 1.0);
    else if (index == 6u) gradient = vec3(1.0, 0.0, -1.0);
    else if (index == 7u) gradient = vec3(-1.0, 0.0, -1.0);
    else if (index == 8u) gradient = vec3(0.0, 1.0, 1.0);
    else if (index == 9u) gradient = vec3(0.0, -1.0, 1.0);
    else if (index == 10u) gradient = vec3(0.0, 1.0, -1.0);
    else gradient = vec3(0.0, -1.0, -1.0);
    return gradient * 0.70710678118;
}

struct SeepageNoise3Sample {
    float value;
    vec3 gradient;
};

void AccumulateSeepageSimplexCorner(
    ivec3 lattice,
    ivec3 offset,
    vec3 delta,
    uint seed,
    inout float value,
    inout vec3 derivative) {
    const float attenuation = 0.60 - dot(delta, delta);
    if (attenuation <= 0.0) {
        return;
    }
    const vec3 gradient = SeepageGradient3(SeepageHash3(lattice + offset, seed));
    const float gradientDot = dot(gradient, delta);
    const float attenuation2 = attenuation * attenuation;
    const float attenuation3 = attenuation2 * attenuation;
    const float attenuation4 = attenuation2 * attenuation2;
    value += attenuation4 * gradientDot;
    derivative += attenuation4 * gradient - 8.0 * attenuation3 * gradientDot * delta;
}

SeepageNoise3Sample SeepageSimplexNoise3(vec3 coordinate, uint seed) {
    const float skew = 1.0 / 3.0;
    const float unskew = 1.0 / 6.0;
    const float skewAmount = (coordinate.x + coordinate.y + coordinate.z) * skew;
    const ivec3 lattice = ivec3(floor(coordinate + skewAmount));
    const float unskewAmount = float(lattice.x + lattice.y + lattice.z) * unskew;
    const vec3 x0 = coordinate - (vec3(lattice) - unskewAmount);
    ivec3 i1;
    ivec3 i2;
    if (x0.x >= x0.y) {
        if (x0.y >= x0.z) {
            i1 = ivec3(1, 0, 0); i2 = ivec3(1, 1, 0);
        } else if (x0.x >= x0.z) {
            i1 = ivec3(1, 0, 0); i2 = ivec3(1, 0, 1);
        } else {
            i1 = ivec3(0, 0, 1); i2 = ivec3(1, 0, 1);
        }
    } else if (x0.y < x0.z) {
        i1 = ivec3(0, 0, 1); i2 = ivec3(0, 1, 1);
    } else if (x0.x < x0.z) {
        i1 = ivec3(0, 1, 0); i2 = ivec3(0, 1, 1);
    } else {
        i1 = ivec3(0, 1, 0); i2 = ivec3(1, 1, 0);
    }
    float value = 0.0;
    vec3 derivative = vec3(0.0);
    AccumulateSeepageSimplexCorner(lattice, ivec3(0), x0, seed, value, derivative);
    AccumulateSeepageSimplexCorner(lattice, i1, x0 - vec3(i1) + unskew, seed, value, derivative);
    AccumulateSeepageSimplexCorner(lattice, i2, x0 - vec3(i2) + 2.0 * unskew, seed, value, derivative);
    AccumulateSeepageSimplexCorner(lattice, ivec3(1), x0 - vec3(1.0) + 3.0 * unskew, seed, value, derivative);
    SeepageNoise3Sample result;
    result.value = clamp(0.5 + 0.5 * value * 32.0, 0.0, 1.0);
    result.gradient = derivative * 16.0;
    return result;
}

SeepageNoise3Sample SeepageFractalNoise3(vec3 coordinate, uint seed, uint quality) {
    const uint octaveCount = quality == 0u ? 1u : (quality >= 2u ? 3u : 2u);
    SeepageNoise3Sample result;
    result.value = 0.0;
    result.gradient = vec3(0.0);
    float amplitude = 1.0;
    float frequency = 1.0;
    float weight = 0.0;
    for (uint octave = 0u; octave < 3u; ++octave) {
        if (octave >= octaveCount) break;
        const SeepageNoise3Sample noiseSample = SeepageSimplexNoise3(
            coordinate,
            seed + octave * 1013u);
        result.value += noiseSample.value * amplitude;
        result.gradient += noiseSample.gradient * (amplitude * frequency);
        weight += amplitude;
        coordinate = coordinate * 2.03 + vec3(17.0, 31.0, 47.0);
        frequency *= 2.03;
        amplitude *= 0.5;
    }
    if (weight > 0.0) {
        result.value /= weight;
        result.gradient /= weight;
    }
    return result;
}

bool SeepageUsesTransition(uint nodeIndex, vec3 worldPosition) {
    const float amount = clamp(
        seepageParamData.seepageParams[nodeIndex].scenario.y,
        0.0,
        1.0);
    if (amount <= 1e-6) return false;
    if (amount >= 1.0 - 1e-6) return true;
    const float featureSize = max(
        0.005,
        min(
            seepageParamData.seepageParams[nodeIndex].look.organic0.x,
            seepageParamData.seepageParams[nodeIndex].transitionLook.organic0.x) * 0.18);
    const ivec3 cell = ivec3(floor(worldPosition / featureSize));
    const uint selectorBits = SeepageHash3(
        cell,
        seepageParamData.seepageParams[nodeIndex].control.w ^
            seepageNodeData.seepageNodes[nodeIndex].control.x) & 0x00ffffffu;
    return float(selectorBits) / float(0x01000000u) < amount;
}

float SeepageReflectionSignal(
    uint nodeIndex,
    bool transition,
    vec3 worldPosition,
    vec3 baseNormal,
    vec3 microGradient,
    float sparseGate) {
    const vec4 organic0 = SeepageLookOrganic0(nodeIndex, transition);
    const vec4 organic1 = SeepageLookOrganic1(nodeIndex, transition);
    vec3 tangentGradient = microGradient - baseNormal * dot(microGradient, baseNormal);
    tangentGradient = dot(tangentGradient, tangentGradient) > 1e-10
                          ? normalize(tangentGradient)
                          : vec3(0.0);
    const vec3 microNormal = RippleSafeNormal(
        baseNormal + tangentGradient * (organic1.y * 0.42));
    vec3 viewDirection = uniforms.cameraPosition.xyz - worldPosition;
    viewDirection = dot(viewDirection, viewDirection) > 1e-10
                        ? normalize(viewDirection)
                        : -SeepageLookEnvironment(nodeIndex, transition);
    const vec3 environmentDirection = SeepageLookEnvironment(nodeIndex, transition);
    vec3 halfVector = viewDirection + environmentDirection;
    halfVector = dot(halfVector, halfVector) > 1e-10
                     ? normalize(halfVector)
                     : environmentDirection;
    const float roughness = clamp(organic0.w, 0.02, 1.0);
    const float exponent = 2.0 + (1.0 - roughness) * (1.0 - roughness) * 126.0;
    const float specular = pow(max(0.0, dot(microNormal, halfVector)), exponent);
    const float fresnel = pow(clamp(1.0 - abs(dot(microNormal, viewDirection)), 0.0, 1.0), 5.0);
    const float angleResponse = clamp(organic1.x, 0.0, 1.0);
    return clamp(
        specular * (0.35 + angleResponse * 1.15) * (0.28 + sparseGate * 0.72) +
            fresnel * (0.08 + angleResponse * 0.32),
        0.0,
        1.0);
}

float SeepagePulseFieldValue(
    uint nodeIndex,
    bool transition,
    uint sampleIndex) {
    const uint vecIndex = sampleIndex >> 2u;
    const int component = int(sampleIndex & 3u);
    const vec4 packedSamples =
        transition
            ? seepageParamData.seepageParams[nodeIndex]
                  .transitionPulseField[vecIndex]
            : seepageParamData.seepageParams[nodeIndex]
                  .pulseField[vecIndex];
    const float value = packedSamples[component];
    return RippleFiniteFloat(value) ? clamp(value, 0.0, 1.0) : 0.0;
}

float SampleSeepagePulseField(
    uint nodeIndex,
    bool transition,
    float downstreamMeters) {
    const uvec4 control =
        seepageParamData.seepageParams[nodeIndex].pulseFieldControl;
    const uint sampleCount = min(
        transition ? control.y : control.x,
        128u);
    const float stableSpan = uintBitsToFloat(
        transition ? control.w : control.z);
    if (sampleCount == 0u ||
        !RippleFiniteFloat(stableSpan) ||
        stableSpan <= 1e-6) {
        return 0.0;
    }
    if (sampleCount == 1u) {
        return SeepagePulseFieldValue(
            nodeIndex,
            transition,
            0u);
    }

    const float normalised = clamp(
        RippleFiniteFloat(downstreamMeters)
            ? downstreamMeters / stableSpan
            : 0.0,
        0.0,
        1.0);
    const float sampleCoordinate =
        normalised * float(sampleCount - 1u);
    const uint firstIndex = min(
        uint(floor(sampleCoordinate)),
        sampleCount - 1u);
    const uint secondIndex = min(
        firstIndex + 1u,
        sampleCount - 1u);
    return mix(
        SeepagePulseFieldValue(
            nodeIndex,
            transition,
            firstIndex),
        SeepagePulseFieldValue(
            nodeIndex,
            transition,
            secondIndex),
        fract(sampleCoordinate));
}

float WrapSeepagePulseDistance(
    float distanceMeters,
    float stableSpanMeters) {
    if (!RippleFiniteFloat(distanceMeters) ||
        !RippleFiniteFloat(stableSpanMeters) ||
        stableSpanMeters <= 1e-6) {
        return 0.0;
    }
    return mod(distanceMeters, stableSpanMeters);
}

float SampleAnimatedSeepagePulseField(
    uint nodeIndex,
    bool transition,
    float fieldDistanceMeters,
    float timeSeconds,
    vec4 legacy0,
    vec4 legacy1,
    vec4 organic0,
    uint quality,
    uint proceduralSeed) {
    const uvec4 fieldControl =
        seepageParamData.seepageParams[nodeIndex].pulseFieldControl;
    const float stableSpan = uintBitsToFloat(
        transition ? fieldControl.w : fieldControl.z);
    if (!RippleFiniteFloat(stableSpan) || stableSpan <= 1e-6) {
        return 0.0;
    }

    const float time = max(
        0.0,
        RippleFiniteFloat(timeSeconds) ? timeSeconds : 0.0);
    const float speed = max(0.0, legacy0.z);
    const float speedVariation = clamp(legacy1.y, 0.0, 1.0);
    const float irregularity = clamp(legacy0.w, 0.0, 1.0);
    const float evolution = max(0.0, organic0.z);
    const float evolutionHash =
        SeepageNoiseHash01(7, 19, proceduralSeed + 49201u);
    const float evolutionPhase = mod(
        time * evolution * (0.15 + evolutionHash * 0.25) +
            evolutionHash * kRippleTwoPi,
        kRippleTwoPi);
    const float evolutionShift =
        sin(evolutionPhase) *
        max(0.001, legacy0.y) *
        irregularity * 0.55;

    // The compact field is built only when authored shape settings change.
    // Frame time advances the lookup here, so camera scrubbing and a stalled
    // support worker cannot freeze the visible wave phase.
    const float primaryDistance = WrapSeepagePulseDistance(
        fieldDistanceMeters - time * speed + evolutionShift,
        stableSpan);
    float pulse = SampleSeepagePulseField(
        nodeIndex,
        transition,
        primaryDistance);

    // Quality-scaled neighbouring phase populations preserve irregular
    // catch-up and additive overlap with at most three interpolated lookups.
    if (quality >= 2u) {
        const float phaseHash =
            SeepageNoiseHash01(23, 41, proceduralSeed + 58309u);
        const float speedHash =
            SeepageNoiseHash01(31, 59, proceduralSeed + 61487u);
        const float secondarySpeed =
            speed *
            max(
                0.15,
                1.0 +
                    (0.35 + speedHash * 0.45) *
                        speedVariation);
        const float secondaryDistance = WrapSeepagePulseDistance(
            fieldDistanceMeters + phaseHash * stableSpan -
                time * secondarySpeed -
                evolutionShift * 0.65,
            stableSpan);
        pulse +=
            SampleSeepagePulseField(
                nodeIndex,
                transition,
                secondaryDistance) *
            speedVariation * 0.82;
    }
    if (quality >= 3u) {
        const float phaseHash =
            SeepageNoiseHash01(47, 71, proceduralSeed + 64763u);
        const float speedHash =
            SeepageNoiseHash01(61, 83, proceduralSeed + 68371u);
        const float tertiarySpeed =
            speed *
            max(
                0.15,
                1.0 -
                    (0.25 + speedHash * 0.40) *
                        speedVariation);
        const float tertiaryDistance = WrapSeepagePulseDistance(
            fieldDistanceMeters + phaseHash * stableSpan -
                time * tertiarySpeed +
                evolutionShift * 0.40,
            stableSpan);
        pulse +=
            SampleSeepagePulseField(
                nodeIndex,
                transition,
                tertiaryDistance) *
            speedVariation * 0.50;
    }
    return clamp(
        pulse + max(0.0, pulse - 0.82) * 0.22,
        0.0,
        1.0);
}

vec3 SeepagePatternSignals(
    uint nodeIndex,
    bool transition,
    vec3 worldPosition,
    vec3 pointNormal,
    vec3 surfaceNormal,
    vec3 downTangent,
    float timeSeconds,
    float downstreamDistance,
    float signedLateralDistance,
    float effectiveReach,
    float effectiveHalfWidth,
    float fanMask) {
    const uvec4 lookControl = SeepageLookControl(nodeIndex, transition);
    const vec4 legacy0 = SeepageLookLegacy0(nodeIndex, transition);
    const vec4 legacy1 = SeepageLookLegacy1(nodeIndex, transition);
    const vec4 response2 = SeepageLookResponse2(nodeIndex, transition);
    const vec4 organic0 = SeepageLookOrganic0(nodeIndex, transition);
    const vec4 organic1 = SeepageLookOrganic1(nodeIndex, transition);
    const vec4 organic2 = SeepageLookOrganic2(nodeIndex, transition);
    const vec4 organic3 = SeepageLookOrganic3(nodeIndex, transition);
    const float rainGain = clamp(
        seepageParamData.seepageParams[nodeIndex].geometry.w * response2.w,
        0.0,
        1.0);
    const float wetness = clamp(response2.y + rainGain * 0.30, 0.0, 1.0);
    const float density = clamp(legacy1.z + rainGain * 0.25, 0.0, 1.0);
    // Strength shapes the area envelope inside the fan mask (see
    // SeepageResolvedFanMask); the signal amplitude is intensity territory
    // and belongs to prominence alone.
    const float strengthMask = fanMask;
    const uint proceduralSeed =
        seepageParamData.seepageParams[nodeIndex].control.w ^
        (seepageNodeData.seepageNodes[nodeIndex].control.x * 0x9e3779b9u);
    const float featureSize = max(0.005, organic0.x);
    const float time = max(0.0, timeSeconds);
    const mat3 noiseRotation = mat3(
        seepageParamData.seepageParams[nodeIndex].noiseBasis[0].xyz,
        seepageParamData.seepageParams[nodeIndex].noiseBasis[1].xyz,
        seepageParamData.seepageParams[nodeIndex].noiseBasis[2].xyz);
    if (lookControl.x == 3u) {
        const float spacing = max(0.005, legacy0.x);
        const float irregularity = clamp(legacy0.w, 0.0, 1.0);
        const float downstream = max(0.0, downstreamDistance);
        const float lateralNormalised = clamp(
            signedLateralDistance / max(0.001, effectiveHalfWidth),
            -1.5,
            1.5);
        // Keep the surface field fixed and move only the independent fronts
        // over it, avoiding whole-pattern texture advection.
        const vec3 coordinate =
            noiseRotation * (worldPosition / (spacing * 1.65));
        const SeepageNoise3Sample frontNoise = SeepageFractalNoise3(
            coordinate,
            proceduralSeed + 17431u,
            seepageParamData.seepageParams[nodeIndex].control.y);
        const float centredNoise = frontNoise.value * 2.0 - 1.0;
        const float baseBowedFront =
            pow(abs(lateralNormalised), 1.35) *
            spacing * (0.10 + irregularity * 0.28);
        // Authored launches and fractional Wave Count are composed only when
        // shape settings change. Renderer time supplies motion/evolution,
        // while stationary world noise bows/roughens each lookup coordinate.
        // Strength/Rain reveals cannot teleport phase, and the former
        // 7–12-wave inner loop becomes at most three interpolations.
        const float frontWarp =
            centredNoise * spacing * irregularity * 0.25;
        const float fieldDistance =
            downstream + baseBowedFront + frontWarp;
        const float pulse = clamp(
            SampleAnimatedSeepagePulseField(
                nodeIndex,
                transition,
                fieldDistance,
                time,
                legacy0,
                legacy1,
                organic0,
                seepageParamData.seepageParams[nodeIndex].control.y,
                proceduralSeed) *
                mix(0.82, 1.16, frontNoise.value),
            0.0,
            1.0);
        const float coverageThreshold = 0.72 - density * 0.38;
        const float dampPatch = smoothstep(
            coverageThreshold,
            coverageThreshold + 0.24,
            frontNoise.value);
        const vec3 resolvedNormal =
            styleData.pointMeta.z != 0u && dot(pointNormal, pointNormal) > 1e-8
                ? normalize(pointNormal)
                : RippleSafeNormal(surfaceNormal);
        const vec3 worldGradient = transpose(noiseRotation) * frontNoise.gradient;
        const float reflection = SeepageReflectionSignal(
            nodeIndex,
            transition,
            worldPosition,
            resolvedNormal,
            worldGradient,
            max(pulse, dampPatch * 0.15));
        return vec3(
            clamp(
                strengthMask *
                    (wetness * (0.06 + dampPatch * 0.10) +
                     pulse *
                         (0.62 + wetness * 0.14 +
                          rainGain * 0.12)),
                0.0,
                1.0),
            clamp(
                strengthMask * pulse * (0.30 + dampPatch * 0.24),
                0.0,
                1.0),
            clamp(
                strengthMask *
                    (0.015 + pulse * 0.78 + dampPatch * 0.06) *
                    response2.z * reflection,
                0.0,
                1.0));
    }
    if (lookControl.x == 2u) {
        const uint quality = seepageParamData.seepageParams[nodeIndex].control.y;
        // The trickle run is the strength- and slope-shaped envelope reach;
        // the pattern no longer declares its own length.
        const float trickleLength = max(0.005, effectiveReach);
        const float trickleWidth = max(0.002, organic2.w);
        const float frontSoftness = max(0.002, organic3.x);
        const float downstream = max(0.0, downstreamDistance);
        const float distanceProgress = clamp(downstream / trickleLength, 0.0, 1.0);
        const float wettingProgress = clamp(
            seepageParamData.seepageParams[nodeIndex].scenario.z,
            0.0,
            1.0);
        const vec3 resolvedDown = RippleSafeNormal(downTangent);
        const vec3 advectedPosition = worldPosition -
            resolvedDown * (time * max(0.0, organic2.y));
        vec3 coordinate = noiseRotation * (advectedPosition / featureSize);
        coordinate += vec3(0.037, -0.029, 0.043) * (time * organic0.z);

        // The quality-controlled fractal is deliberately the only sampled noise
        // stack here: Low has one scale, Balanced adds breakup, and High adds detail.
        const SeepageNoise3Sample trickleNoise = SeepageFractalNoise3(
            coordinate,
            proceduralSeed + 6151u,
            quality);
        const float breakup = clamp(organic2.x, 0.0, 1.0);
        const float patchThreshold = 0.70 - density * 0.34 + breakup * 0.07;
        const float patchField = smoothstep(
            patchThreshold,
            patchThreshold + mix(0.22, 0.12, breakup),
            trickleNoise.value);

        const float sourceDistance = length(vec2(downstream, signedLateralDistance));
        const float sourceEnvelope = 1.0 - smoothstep(
            featureSize * 0.30,
            featureSize * 1.35,
            sourceDistance);
        const float sourcePatch = sourceEnvelope * mix(0.38, 1.0, patchField);

        // Finger lanes spread across the envelope's local half-width so they
        // track the widening run.
        const float fanHalfWidth = max(1e-4, effectiveHalfWidth);
        const float lateralWander =
            (trickleNoise.value - 0.5) * featureSize * mix(0.16, 0.85, breakup);
        const float laneHash0 = SeepageNoiseHash01(0, 0, proceduralSeed + 7013u);
        const float laneHash1 = SeepageNoiseHash01(1, 0, proceduralSeed + 7013u);
        const float laneHash2 = SeepageNoiseHash01(2, 0, proceduralSeed + 7013u);
        const float lane0 = (laneHash0 - 0.5) * fanHalfWidth * 0.55 + lateralWander;
        const float lane1 = (laneHash1 - 0.5) * fanHalfWidth * 1.25 - lateralWander * 0.58;
        const float lane2 = (laneHash2 - 0.5) * fanHalfWidth * 1.55 + lateralWander * 0.36;
        const float softWidth = trickleWidth * mix(1.65, 2.65, breakup);
        float fingers = 1.0 - smoothstep(
            trickleWidth,
            softWidth,
            abs(signedLateralDistance - lane0));
        const float secondaryGate = smoothstep(0.22, 0.72, density + laneHash1 * 0.24);
        const float tertiaryGate = smoothstep(0.48, 0.90, density + laneHash2 * 0.18);
        fingers = max(
            fingers,
            (1.0 - smoothstep(
                trickleWidth * 0.82,
                softWidth * 0.86,
                abs(signedLateralDistance - lane1))) * secondaryGate);
        fingers = max(
            fingers,
            (1.0 - smoothstep(
                trickleWidth * 0.68,
                softWidth * 0.74,
                abs(signedLateralDistance - lane2))) * tertiaryGate);

        const int delayCellDown = int(floor(downstream / featureSize));
        const int delayCellLateral = int(floor(signedLateralDistance / featureSize));
        const float saturationDelay = SeepageNoiseHash01(
            delayCellDown,
            delayCellLateral,
            proceduralSeed + 8089u);
        const float onset = clamp(
            0.03 + distanceProgress * 0.68 + saturationDelay * 0.14 +
                (1.0 - patchField) * breakup * 0.06,
            0.0,
            0.92);
        const float progressFeather = clamp(
            frontSoftness / trickleLength,
            0.008,
            0.32);
        const float wetReveal = smoothstep(
            onset - progressFeather,
            onset + progressFeather,
            wettingProgress);
        const float frontPulse = 1.0 - smoothstep(
            progressFeather,
            progressFeather * 3.2,
            abs(wettingProgress - onset));
        // No separate length mask: the fan mask's end feather already fades
        // the run out at the effective reach.
        const float breakupGate = mix(
            1.0,
            smoothstep(0.20, 0.78, trickleNoise.value + patchField * 0.22),
            breakup);
        const float trickleBody = fingers * breakupGate;
        const float persistentDamp = max(
            sourcePatch,
            trickleBody * (0.50 + patchField * 0.50));
        const float activeWet = wetReveal * persistentDamp;

        const vec3 resolvedNormal =
            styleData.pointMeta.z != 0u && dot(pointNormal, pointNormal) > 1e-8
                ? normalize(pointNormal)
                : RippleSafeNormal(surfaceNormal);
        const vec3 worldGradient = transpose(noiseRotation) * trickleNoise.gradient;
        const float sparseGate = smoothstep(
            0.90 - organic1.z * 0.58,
            0.98,
            trickleNoise.value);
        const float reflection = SeepageReflectionSignal(
            nodeIndex,
            transition,
            worldPosition,
            resolvedNormal,
            worldGradient,
            max(sparseGate, activeWet * 0.42));
        return vec3(
            clamp(
                strengthMask * activeWet *
                    (wetness * (0.70 + patchField * 0.22) + rainGain * 0.12),
                0.0,
                1.0),
            clamp(
                strengthMask * wetReveal *
                    (trickleBody * (0.16 + patchField * 0.20) +
                     frontPulse * fingers * 0.52 + sourcePatch * 0.08),
                0.0,
                1.0),
            clamp(
                strengthMask * activeWet * response2.z * reflection,
                0.0,
                1.0));
    }
    vec3 coordinate;
    SeepageNoise3Sample bodyNoise;
    SeepageNoise3Sample warpNoise;
    float patchOrBloom;
    float sparseGate;
    if (lookControl.x == 0u) {
        coordinate = noiseRotation * (worldPosition / featureSize);
        coordinate += vec3(0.071, -0.053, 0.037) * (time * organic0.z);
        bodyNoise = SeepageFractalNoise3(
            coordinate,
            proceduralSeed,
            seepageParamData.seepageParams[nodeIndex].control.y);
        const float feather = mix(0.24, 0.035, organic0.y);
        patchOrBloom = smoothstep(1.0 - density - feather, 1.0 - density + feather, bodyNoise.value);
        sparseGate = bodyNoise.value;
        if (seepageParamData.seepageParams[nodeIndex].control.y != 0u) {
            sparseGate = SeepageSimplexNoise3(
                coordinate * 3.73 + vec3(13.0, -7.0, 19.0),
                proceduralSeed + 4099u).value;
        }
        sparseGate = smoothstep(0.92 - organic1.z * 0.62, 0.98, sparseGate);
    } else {
        const vec3 advectedPosition = worldPosition -
            RippleSafeNormal(downTangent) * (time * organic2.y);
        coordinate = noiseRotation * (advectedPosition / featureSize);
        warpNoise = SeepageFractalNoise3(
            coordinate * 0.53 + vec3(0.031, -0.047, 0.023) * (time * organic0.z),
            proceduralSeed + 2053u,
            seepageParamData.seepageParams[nodeIndex].control.y);
        coordinate += warpNoise.gradient * (organic1.w * 0.22);
        coordinate += vec3(-0.029, 0.041, 0.017) * (time * organic0.z);
        bodyNoise = SeepageFractalNoise3(
            coordinate,
            proceduralSeed,
            seepageParamData.seepageParams[nodeIndex].control.y);
        const float ridge = 1.0 - abs(bodyNoise.value * 2.0 - 1.0);
        const float organic = clamp(ridge * 0.68 + bodyNoise.value * 0.32, 0.0, 1.0);
        const float threshold = 0.76 - density * 0.48 + organic2.x * 0.16;
        patchOrBloom = smoothstep(threshold, threshold + 0.18, organic);
        sparseGate = smoothstep(0.90 - organic1.z * 0.58, 0.98, warpNoise.value);
    }
    const vec3 resolvedNormal =
        styleData.pointMeta.z != 0u && dot(pointNormal, pointNormal) > 1e-8
            ? normalize(pointNormal)
            : RippleSafeNormal(surfaceNormal);
    const vec3 worldGradient = transpose(noiseRotation) * bodyNoise.gradient;
    const float reflection = SeepageReflectionSignal(
        nodeIndex,
        transition,
        worldPosition,
        resolvedNormal,
        worldGradient,
        sparseGate);
    if (lookControl.x == 0u) {
        return vec3(
            clamp(strengthMask * wetness * (0.68 + bodyNoise.value * 0.22 + patchOrBloom * 0.20), 0.0, 1.0),
            clamp(strengthMask * patchOrBloom * (0.08 + bodyNoise.value * 0.14), 0.0, 1.0),
            clamp(strengthMask * patchOrBloom * response2.z * reflection, 0.0, 1.0));
    }
    const float ridge = 1.0 - abs(bodyNoise.value * 2.0 - 1.0);
    const float organic = clamp(ridge * 0.68 + bodyNoise.value * 0.32, 0.0, 1.0);
    return vec3(
        clamp(strengthMask * (wetness * (0.60 + bodyNoise.value * 0.22) + patchOrBloom * (0.16 + rainGain * 0.14)), 0.0, 1.0),
        clamp(strengthMask * patchOrBloom * (0.18 + organic * 0.42), 0.0, 1.0),
        clamp(strengthMask * patchOrBloom * response2.z * reflection, 0.0, 1.0));
}

SparseRippleComposite EvaluateSeepageContribution(
    uint nodeIndex,
    bool usesConnectedSupport,
    SeepageNodeReference supportReference,
    vec3 worldPosition,
    vec3 pointNormal,
    uint pointIndex,
    float timeSeconds) {
    SparseRippleComposite contribution = EmptySparseRippleComposite();
    // Visibility, strength, reach, width, and prominence are animation-updated
    // scalars. Reject inactive nodes before the surface-guide search and all
    // procedural work (matching the CPU query gates in
    // EvaluateWaterSeepageGridContribution), keeping a Visible toggle
    // parameter-only.
    if (seepageParamData.seepageParams[nodeIndex].geometry.x <= 1e-5 ||
        seepageParamData.seepageParams[nodeIndex].geometry.z <= 1e-5 ||
        seepageParamData.seepageParams[nodeIndex].liveGeometry.x <= 1e-5 ||
        seepageParamData.seepageParams[nodeIndex].liveGeometry.y <= 1e-5 ||
        seepageParamData.seepageParams[nodeIndex].liveGeometry.z <= 1e-5) {
        return contribution;
    }
    float downstreamDistance;
    float lateralDistance;
    float effectiveReach;
    float effectiveHalfWidth;
    float lateralNormalised;
    vec3 surfaceNormal;
    vec3 downTangent;
    const float fanMask = usesConnectedSupport
        ? SeepageConnectedSupportMask(
              nodeIndex,
              supportReference,
              worldPosition,
              pointNormal,
              downstreamDistance,
              lateralDistance,
              effectiveReach,
              effectiveHalfWidth,
              lateralNormalised,
              surfaceNormal,
              downTangent)
        : SeepageFanMask(
              nodeIndex,
              worldPosition,
              pointNormal,
              downstreamDistance,
              lateralDistance,
              effectiveReach,
              effectiveHalfWidth,
              lateralNormalised,
              surfaceNormal,
              downTangent);
    if (fanMask <= 1e-5) {
        return contribution;
    }

    const bool transition = SeepageUsesTransition(nodeIndex, worldPosition);
    const vec4 legacy1 = SeepageLookLegacy1(nodeIndex, transition);
    const vec4 response0 = SeepageLookResponse0(nodeIndex, transition);
    const vec4 response1 = SeepageLookResponse1(nodeIndex, transition);
    const vec4 response2 = SeepageLookResponse2(nodeIndex, transition);
    float wettingFanMask = fanMask;
    const float wettingProgress = clamp(
        seepageParamData.seepageParams[nodeIndex].scenario.z,
        0.0,
        1.0);
    if (wettingProgress <= 1e-6) {
        return contribution;
    }
    if (wettingProgress < 1.0 - 1e-6) {
        // The front travels the same strength- and slope-shaped run the fan
        // mask used, so the reveal reaches exactly as far as the effect does.
        const float frontReach = max(1e-4, effectiveReach);
        const float frontSoftness = max(
            seepageNodeData.seepageNodes[nodeIndex].downEdge.w,
            SeepageLookOrganic3(nodeIndex, transition).x);
        const float frontDistance = frontReach * wettingProgress;
        const float frontMask = 1.0 - smoothstep(
            frontDistance - frontSoftness,
            frontDistance + frontSoftness,
            max(0.0, downstreamDistance));
        wettingFanMask *= frontMask * smoothstep(0.0, 0.04, wettingProgress);
        if (wettingFanMask <= 1e-5) {
            return contribution;
        }
    }
    const vec3 signals = SeepagePatternSignals(
        nodeIndex,
        transition,
        worldPosition,
        pointNormal,
        surfaceNormal,
        downTangent,
        timeSeconds,
        downstreamDistance,
        lateralDistance,
        effectiveReach,
        effectiveHalfWidth,
        wettingFanMask);
    const float scale = clamp(
        (signals.x * 0.58 + signals.y * 0.34 + signals.z * 0.46) *
            max(0.0, legacy1.w) *
            max(0.0, seepageParamData.seepageParams[nodeIndex].liveGeometry.z) *
            (1.0 + clamp(
                 seepageParamData.seepageParams[nodeIndex].geometry.w *
                     response2.w,
                 0.0,
                 1.0) * 0.65),
        0.0,
        1.0);
    if (scale <= 1e-5) {
        return contribution;
    }

    contribution.scale = scale;
    contribution.colourMix = clamp(
        response2.x * scale,
        0.0,
        1.0);
    contribution.emissionAdd = max(
        0.0,
        response0.x) * scale;
    contribution.opacityAdd =
        response0.y * scale;
    contribution.opacityMultiply = mix(
        1.0,
        max(0.0, response0.z),
        scale);
    contribution.pointSizeAdd =
        response0.w * scale;
    contribution.pointSizeMultiply = mix(
        1.0,
        max(0.0, response1.x),
        scale);
    contribution.colour = clamp(
        response1.yzw,
        vec3(0.0),
        vec3(1.0));
    return contribution;
}

float ShorelineHeightMask(float boundaryZ, float reachMeters, float edgeFadeMeters, float worldZ) {
    const float shoreDistance = boundaryZ - worldZ;
    if (shoreDistance < -edgeFadeMeters || shoreDistance > reachMeters + edgeFadeMeters) {
        return 0.0;
    }
    const float waterSide = smoothstep(-edgeFadeMeters, max(edgeFadeMeters, 1e-5), shoreDistance);
    const float reachFade =
        1.0 - smoothstep(max(0.0, reachMeters), reachMeters + max(edgeFadeMeters, 1e-5), shoreDistance);
    return clamp(waterSide * reachFade, 0.0, 1.0);
}

// Shared shoreline evaluation over one explicit lane set. The primary style
// shoreline and every additional shoreline instance use identical lane
// semantics, so the same body serves both.
SparseRippleComposite EvaluateShorelineWaveContributionFrom(
    uvec4 waveControl,
    vec4 waveParams0,
    vec4 waveParams1,
    vec4 waveParams2,
    vec4 waveParams3,
    vec4 waveParams4,
    vec4 waveParams5,
    vec4 waveTint,
    vec3 worldPosition,
    vec3 pointNormal,
    float timeSeconds) {
    SparseRippleComposite contribution = EmptySparseRippleComposite();
    const bool waveActive =
        waveControl.x != 0u &&
        waveParams0.y > 1e-5 &&
        waveParams1.w > 1e-5 &&
        waveParams0.w > 1e-5;
    if (!waveActive) {
        return contribution;
    }

    const float boundaryZ = waveParams0.x;
    const float reachMeters = max(0.001, waveParams0.y);
    const float edgeFadeMeters = max(0.0, waveParams0.z);
    const float heightMask = ShorelineHeightMask(boundaryZ, reachMeters, edgeFadeMeters, worldPosition.z);
    if (heightMask <= 1e-5) {
        return contribution;
    }

    vec2 tangent = vec2(waveParams1.x, waveParams1.y);
    tangent = dot(tangent, tangent) > 1e-8 ? normalize(tangent) : vec2(1.0, 0.0);
    const float tangentCoordinate = dot(worldPosition.xy, tangent);
    const float shoreDistance = max(0.0, boundaryZ - worldPosition.z);
    const float patternScale = max(0.01, waveParams1.z);
    const float wavelength = max(0.002, waveParams1.w);
    const float shorelineEdgeBlendWidth =
        max(0.001, min(max(edgeFadeMeters, wavelength * 0.20), reachMeters * 0.45));
    const float timePhase =
        waveParams3.x + (timeSeconds * max(0.0, waveParams2.x));
    float pattern = 0.0;
    if (waveControl.z == 1u) {
        pattern = HeightFoamShorelineWaveValue(
            vec2(shoreDistance, tangentCoordinate),
            worldPosition.z,
            boundaryZ,
            waveParams5.x,
            reachMeters,
            edgeFadeMeters,
            patternScale,
            wavelength,
            waveParams2.y,
            waveParams2.z,
            waveParams2.w,
            waveParams5.y,
            waveParams5.z,
            waveParams5.w,
            float(waveControl.y),
            timePhase);
    } else if (waveControl.z == 2u) {
        pattern = ContinuousBandShorelineWaveValue(
            vec2(shoreDistance, tangentCoordinate) * patternScale,
            shoreDistance,
            reachMeters,
            wavelength,
            waveParams2.y,
            waveParams2.z,
            waveParams2.w,
            float(waveControl.y),
            timePhase);
    } else {
        pattern = SandCloudShorelineWaveValue(
            vec2(shoreDistance, tangentCoordinate) * patternScale,
            shoreDistance,
            shorelineEdgeBlendWidth,
            wavelength,
            waveParams2.y,
            waveParams2.z,
            waveParams2.w,
            float(waveControl.y),
            timePhase);
        // Background Wash (waveParams4.w): one keeps the authored look; below
        // one the faded body wash between crests sharpens toward line-like
        // peaks, above one it lifts. Zero means a document from before the
        // control existed and stays neutral.
        const float backgroundWash = waveParams4.w;
        if (backgroundWash > 1.0e-4 && abs(backgroundWash - 1.0) > 1.0e-4) {
            pattern = pow(
                clamp(pattern, 0.0, 1.0),
                1.0 / clamp(backgroundWash, 0.05, 2.0));
        }
    }

    const float normalMask =
        clamp(0.62 + 0.38 * abs(dot(RippleSafeNormal(pointNormal), vec3(0.0, 0.0, 1.0))), 0.0, 1.0);
    const float scale = clamp(pattern * heightMask * normalMask * waveParams0.w, 0.0, 1.0);
    if (scale <= 1e-5) {
        return contribution;
    }

    contribution.scale = scale;
    contribution.colourMix = clamp(waveParams4.z * scale, 0.0, 1.0);
    contribution.emissionAdd = max(0.0, waveParams3.y) * scale;
    contribution.opacityAdd = waveParams3.z * scale;
    contribution.opacityMultiply = mix(1.0, max(0.0, waveParams3.w), scale);
    contribution.pointSizeAdd = waveParams4.x * scale;
    contribution.pointSizeMultiply = mix(1.0, max(0.0, waveParams4.y), scale);
    contribution.colour = clamp(waveTint.rgb, vec3(0.0), vec3(1.0));
    return contribution;
}

SparseRippleComposite EvaluateShorelineWaveContribution(vec3 worldPosition, vec3 pointNormal, float timeSeconds) {
    if (!HasShorelineWaveEffect()) {
        return EmptySparseRippleComposite();
    }
    return EvaluateShorelineWaveContributionFrom(
        styleData.shorelineWaveControl,
        styleData.shorelineWaveParams0,
        styleData.shorelineWaveParams1,
        styleData.shorelineWaveParams2,
        styleData.shorelineWaveParams3,
        styleData.shorelineWaveParams4,
        styleData.shorelineWaveParams5,
        styleData.shorelineWaveTint,
        worldPosition,
        pointNormal,
        timeSeconds);
}

float RippleScreen(float baseValue, float contribution) {
    const float a = clamp(baseValue, 0.0, 1.0);
    const float b = clamp(contribution, 0.0, 1.0);
    return 1.0 - ((1.0 - a) * (1.0 - b));
}

void BlendSparseRippleContribution(inout SparseRippleComposite target, SparseRippleComposite contribution, uint blendMode) {
    if (contribution.scale <= 1e-5) {
        return;
    }
    if (blendMode == 1u) {
        target.scale = max(target.scale, contribution.scale);
        target.emissionAdd = max(target.emissionAdd, contribution.emissionAdd);
        target.opacityAdd = max(target.opacityAdd, contribution.opacityAdd);
        target.opacityMultiply = max(target.opacityMultiply, contribution.opacityMultiply);
        target.pointSizeAdd = max(target.pointSizeAdd, contribution.pointSizeAdd);
        target.pointSizeMultiply = max(target.pointSizeMultiply, contribution.pointSizeMultiply);
        if (contribution.colourMix >= target.colourMix) {
            target.colourMix = contribution.colourMix;
            target.colour = contribution.colour;
        }
        return;
    }
    if (blendMode == 2u) {
        target.scale = max(target.scale, contribution.scale);
        target.opacityMultiply *= contribution.opacityMultiply;
        target.pointSizeMultiply *= contribution.pointSizeMultiply;
        target.emissionAdd += contribution.emissionAdd;
        target.opacityAdd += contribution.opacityAdd;
        target.pointSizeAdd += contribution.pointSizeAdd;
    } else if (blendMode == 3u) {
        target.scale = RippleScreen(target.scale, contribution.scale);
        target.emissionAdd = RippleScreen(target.emissionAdd, contribution.emissionAdd);
        target.opacityAdd = RippleScreen(target.opacityAdd, contribution.opacityAdd);
        target.opacityMultiply *= contribution.opacityMultiply;
        target.pointSizeAdd = RippleScreen(target.pointSizeAdd, contribution.pointSizeAdd);
        target.pointSizeMultiply *= contribution.pointSizeMultiply;
    } else if (blendMode == 4u) {
        target = contribution;
        return;
    } else {
        target.scale = clamp(target.scale + contribution.scale, 0.0, 1.0);
        target.emissionAdd += contribution.emissionAdd;
        target.opacityAdd += contribution.opacityAdd;
        target.opacityMultiply *= contribution.opacityMultiply;
        target.pointSizeAdd += contribution.pointSizeAdd;
        target.pointSizeMultiply *= contribution.pointSizeMultiply;
    }
    const float nextMix = clamp(target.colourMix + contribution.colourMix, 0.0, 1.0);
    if (nextMix > 1e-5) {
        target.colour = mix(target.colour, contribution.colour, contribution.colourMix / nextMix);
    }
    target.colourMix = nextMix;
}

void BlendSeepageContributions(
    inout SparseRippleComposite target,
    vec3 worldPosition,
    vec3 pointNormal,
    uint pointIndex,
    float timeSeconds) {
    if (!HasSeepageEffect() ||
        !RippleFiniteVec3(worldPosition) ||
        any(lessThan(worldPosition, styleData.seepageBoundsMin.xyz)) ||
        any(greaterThan(worldPosition, styleData.seepageBoundsMax.xyz))) {
        return;
    }

    const float cellSize = max(1e-6, styleData.seepageGridParams.x);
    const ivec3 coordinate = ivec3(floor(worldPosition / cellSize));
    const uint capacity = styleData.seepageControl.z;
    if (capacity == 0u) {
        return;
    }
    const uint probeLimit = min(
        capacity,
        max(1u, uint(max(1.0, styleData.seepageGridParams.z))));
    const uint initialSlot = SeepageCellHash(coordinate) & (capacity - 1u);
    for (uint probe = 0u; probe < probeLimit; ++probe) {
        const uint slot = (initialSlot + probe) & (capacity - 1u);
        const SeepageHashCell cell = seepageHashCellData.seepageHashCells[slot];
        if (cell.coordinate.w == 0) {
            return;
        }
        if (any(notEqual(cell.coordinate.xyz, coordinate))) {
            continue;
        }

        const uint start = cell.range.x;
        const uint count = cell.range.y;
        if (count == 0u || start >= styleData.seepageControl.w) {
            return;
        }
        const uint available = styleData.seepageControl.w - start;
        const uint cappedEnd = start + min(count, available);
        const bool usesConnectedSupport = styleData.seepageGridParams.w > 0.5;
        for (uint referenceIndex = start; referenceIndex < cappedEnd; ++referenceIndex) {
            const SeepageNodeReference supportReference =
                seepageNodeReferenceData.seepageNodeReferences[referenceIndex];
            const uint nodeIndex = supportReference.nodeIndex;
            if (nodeIndex >= styleData.seepageControl.y) {
                continue;
            }
            const SparseRippleComposite contribution = EvaluateSeepageContribution(
                nodeIndex,
                usesConnectedSupport,
                supportReference,
                worldPosition,
                pointNormal,
                pointIndex,
                timeSeconds);
            BlendSparseRippleContribution(
                target,
                contribution,
                SeepageLookControl(
                    nodeIndex,
                    SeepageUsesTransition(nodeIndex, worldPosition)).y);
        }
        return;
    }
}

SparseRippleComposite ResolveSparseRippleComposite(vec3 worldPosition, vec3 pointNormal, uint pointIndex, float timeSeconds) {
    SparseRippleComposite result = EvaluateShorelineWaveContribution(worldPosition, pointNormal, timeSeconds);
    // Additional shoreline instances blend additively over the primary
    // shoreline. Each instance early-outs on its own height mask, so points
    // outside an instance's band pay only the mask test.
    const uint additionalShorelineCount = min(styleData.additionalShorelineCount.x, 4u);
    for (uint shorelineIndex = 0u; shorelineIndex < additionalShorelineCount; ++shorelineIndex) {
        SparseRippleComposite instanceContribution = EvaluateShorelineWaveContributionFrom(
            styleData.additionalShorelineControl[shorelineIndex],
            styleData.additionalShorelineParams0[shorelineIndex],
            styleData.additionalShorelineParams1[shorelineIndex],
            styleData.additionalShorelineParams2[shorelineIndex],
            styleData.additionalShorelineParams3[shorelineIndex],
            styleData.additionalShorelineParams4[shorelineIndex],
            styleData.additionalShorelineParams5[shorelineIndex],
            styleData.additionalShorelineTint[shorelineIndex],
            worldPosition,
            pointNormal,
            timeSeconds);
        BlendSparseRippleContribution(result, instanceContribution, 0u);
    }
    BlendSeepageContributions(result, worldPosition, pointNormal, pointIndex, timeSeconds);
    return SanitizeSparseRippleComposite(result);
}

vec3 ApplySparseRippleColor(vec3 baseColor, SparseRippleComposite ripple) {
    if (ripple.colourMix <= 1e-5) {
        return baseColor;
    }
    return mix(baseColor, ripple.colour, clamp(ripple.colourMix, 0.0, 1.0));
}
