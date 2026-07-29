const float kRipplePi = 3.14159265359;
const float kRippleTwoPi = 6.28318530718;

struct SparseRippleRange {
    uvec2 range;
};

struct SparseRippleMembership {
    uvec4 control;
    vec4 data;
};

struct SparseRippleParams {
    uvec4 control;
    vec4 region0;
    vec4 region1;
    vec4 pattern0;
    vec4 pattern1;
    vec4 response0;
    vec4 response1;
    vec4 response2;
};

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

layout(set = 0, binding = 7, std430) readonly buffer SparseRippleRanges {
    SparseRippleRange sparseRippleRanges[];
} sparseRippleRangeData;

layout(set = 0, binding = 8, std430) readonly buffer SparseRippleMemberships {
    SparseRippleMembership sparseRippleMemberships[];
} sparseRippleMembershipData;

layout(set = 0, binding = 9, std430) readonly buffer SparseRippleParamsBuffer {
    SparseRippleParams sparseRippleParams[];
} sparseRippleParamData;

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

bool HasSparseRippleEffects() {
    return styleData.rippleEffectSlots3.x != 0u &&
           styleData.rippleEffectSlots3.y != 0u &&
           styleData.rippleEffectSlots3.z != 0u &&
           styleData.pointMeta.x <= uint(sparseRippleRangeData.sparseRippleRanges.length()) &&
           styleData.rippleEffectSlots3.y <=
               uint(sparseRippleMembershipData.sparseRippleMemberships.length()) &&
           styleData.rippleEffectSlots3.z <=
               uint(sparseRippleParamData.sparseRippleParams.length());
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

float RippleOverlaySalt(uint overlayType) {
    if (overlayType == 0u) {
        return 101.0;
    }
    if (overlayType == 1u) {
        return 211.0;
    }
    if (overlayType == 2u) {
        return 307.0;
    }
    if (overlayType == 3u) {
        return 401.0;
    }
    if (overlayType == 4u) {
        return 503.0;
    }
    if (overlayType == 5u) {
        return 601.0;
    }
    if (overlayType == 6u) {
        return 701.0;
    }
    if (overlayType == 7u) {
        return 809.0;
    }
    if (overlayType == 8u) {
        return 907.0;
    }
    if (overlayType == 9u) {
        return 1009.0;
    }
    return 1103.0;
}

float RippleCellHash(int cellX, int cellY, float seed, float salt) {
    return RippleHash(
        float(cellX) * 12.9898 +
        float(cellY) * 78.233 +
        seed * 37.719 +
        salt * 19.371);
}

vec2 RippleCellHash2(int cellX, int cellY, float seed, float salt) {
    return vec2(
        RippleCellHash(cellX, cellY, seed, salt),
        RippleCellHash(cellX, cellY, seed, salt + 17.0));
}

float RippleBlockNoise(vec2 uv, float cellSize, float seed, float salt) {
    const float safeCellSize = max(0.001, cellSize);
    const int cellX = int(floor(uv.x / safeCellSize));
    const int cellY = int(floor(uv.y / safeCellSize));
    return RippleCellHash(cellX, cellY, seed, salt);
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

float RippleWavePeak(float phase, float sharpness) {
    const float wave = 0.5 + 0.5 * cos(phase * kRippleTwoPi);
    return pow(clamp(wave, 0.0, 1.0), max(0.01, sharpness));
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

float RippleCausticLaceValue(vec2 uv, float wavelength, float warp, float turbulence, float density, float seed, float phase) {
    const float cellSize = max(0.005, wavelength * 0.78);
    const float t = -phase;
    const float density01 = clamp(density, 0.0, 1.0);
    const float turbulence01 = clamp(turbulence, 0.0, 1.0);
    vec2 p = uv / cellSize;
    const float warpAmount = clamp(warp, 0.0, 8.0);
    p += vec2(
             sin((p.y * 0.81 + seed * 1.37 + t * 0.22) * 2.19) +
                 0.5 * sin((p.y * 1.73 - seed * 0.61 - t * 0.15) * 1.31),
             cos((p.x * 0.88 + seed * 1.91 - t * 0.24) * 2.41) +
                 0.5 * sin((p.x * 1.57 + seed * 0.47 + t * 0.18) * 1.67)) *
         (0.08 + warpAmount * 0.18);

    const int baseX = int(floor(p.x));
    const int baseY = int(floor(p.y));
    float nearest = 1.0e20;
    float secondNearest = 1.0e20;
    for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
            const int cx = baseX + dx;
            const int cy = baseY + dy;
            const vec2 h = RippleCellHash2(cx, cy, seed, 31.0);
            const float angle = (h.x * 1.73 + h.y * 2.41 + t * (0.055 + h.x * 0.050)) * kRippleTwoPi;
            const vec2 wobble = vec2(cos(angle), sin(angle * 1.13 + h.y * kRippleTwoPi)) *
                                 (0.10 + h.y * 0.11);
            const vec2 feature = vec2(float(cx), float(cy)) + h + wobble;
            const float d = length(p - feature);
            if (d < nearest) {
                secondNearest = nearest;
                nearest = d;
            } else if (d < secondNearest) {
                secondNearest = d;
            }
        }
    }
    const float ridgeDistance = secondNearest - nearest;
    const float lineWidth = clamp(0.010 + turbulence01 * 0.022 + density01 * 0.012, 0.008, 0.085);
    const float ridge = 1.0 - smoothstep(lineWidth, lineWidth * 3.7, ridgeDistance);
    const float broadRidge = 1.0 - smoothstep(lineWidth * 1.8, lineWidth * 6.4, ridgeDistance);
    const float filamentA = RippleWavePeak((p.x * 0.23 + p.y * 0.71) + t * 0.045 + seed * 0.17, 7.0);
    const float filamentB = RippleWavePeak((p.x * -0.52 + p.y * 0.34) - t * 0.038 + seed * 0.11, 9.0);
    const float filament = max(filamentA, filamentB);
    const float shimmer = 0.80 + 0.20 * RippleSmoothBlockNoise(
                                           uv + vec2(t * 0.021, -t * 0.017),
                                           cellSize * 0.33,
                                           seed,
                                           43.0);
    const float ridgeEnergy = ridge * 0.88 + broadRidge * ridge * 0.18 + filament * ridge * 0.18;
    const float coverage = smoothstep(0.30 - density01 * 0.16, 0.94, ridgeEnergy);
    const float activeEnvelope = smoothstep(0.08, 0.62, ridge) * coverage;
    const float lace = pow(clamp(ridge, 0.0, 1.0), 1.65) * (0.78 + filament * 0.22);
    const float filamentLift = filament * pow(clamp(ridge, 0.0, 1.0), 1.15) * (0.14 + turbulence01 * 0.10);
    const float softGlow = broadRidge * ridge * (0.025 + density01 * 0.045);
    return clamp((lace + filamentLift + softGlow) * activeEnvelope * shimmer, 0.0, 1.0);
}

float RippleRainRingValue(vec2 uv, float wavelength, float warp, float turbulence, float density, float seed, float phase) {
    const float density01 = clamp(density, 0.0, 1.0);
    const float densityCurve = sqrt(density01);
    const float cellSize = max(wavelength * 1.45, mix(0.34, 0.115, densityCurve));
    const vec2 p = uv / cellSize;
    const int baseX = int(floor(p.x));
    const int baseY = int(floor(p.y));
    const float t = -phase;
    const float rainDensity = clamp(0.10 + density01 * 0.78, 0.06, 0.92);
    const float width = max(wavelength * (0.026 + turbulence * 0.024), 0.0022);
    const float closeSpacing = max(wavelength * (0.15 + turbulence * 0.055), width * 3.0);
    const float maxRadius = max(wavelength * (1.72 + turbulence * 0.38 + clamp(warp, 0.0, 2.0) * 0.13), cellSize * 0.46);
    float best = 0.0;
    float blend = 0.0;
    for (int dy = -2; dy <= 2; ++dy) {
        for (int dx = -2; dx <= 2; ++dx) {
            const int cx = baseX + dx;
            const int cy = baseY + dy;
            const float dropGate = RippleCellHash(cx, cy, seed, 71.0);
            if (dropGate > rainDensity) {
                continue;
            }
            vec2 center = (vec2(float(cx), float(cy)) + RippleCellHash2(cx, cy, seed, 59.0)) * cellSize;
            center += (RippleCellHash2(cx, cy, seed, 67.0) - 0.5) * cellSize * clamp(warp, 0.0, 2.0) * 0.13;
            const float distance = length(uv - center);
            const float dropSeed = RippleCellHash(cx, cy, seed, 79.0);
            const float life = fract(t * (0.16 + dropSeed * 0.07 + density01 * 0.025) + dropSeed);
            const float radius = maxRadius * smoothstep(0.0, 1.0, life);
            const float fade = pow(1.0 - life, 1.22) * smoothstep(0.025, 0.13, life);
            const float reachEnvelope = 1.0 - smoothstep(maxRadius * 0.86, maxRadius * 1.10, distance);
            const float innerVisible = smoothstep(closeSpacing * 1.1, maxRadius * 0.82, radius);
            const float outerVisible = smoothstep(closeSpacing * 0.6, maxRadius * 0.92, radius);
            const float primary = RippleLine(distance - radius, width);
            const float inner = RippleLine(distance - max(0.0, radius - closeSpacing * 0.92), width * 0.72) * innerVisible * 0.54;
            const float outer = RippleLine(distance - (radius + closeSpacing * 0.78), width * 0.82) * outerVisible * 0.34;
            const float wave = RippleWavePeak(
                (distance - radius) / max(closeSpacing, 0.002) + dropSeed * 0.37,
                2.4);
            const float interference = wave *
                                       (1.0 - smoothstep(width * 2.0, closeSpacing * 3.2, abs(distance - radius))) *
                                       0.22;
            const float amplitude = (0.66 + dropSeed * 0.24 + (rainDensity - dropGate) * 0.10) * fade * reachEnvelope;
            const float drop = (primary + inner + outer + interference) * amplitude;
            best = max(best, drop);
            blend += drop;
        }
    }
    return clamp(max(best, blend * (0.26 + density01 * 0.18)), 0.0, 1.0);
}

float RippleTideBandsValue(vec2 uv, float shoreDistance, float edgeBlendWidth, float wavelength, float warp, float turbulence, float density, float seed, float phase) {
    const float travelDistance = max(wavelength, 0.015);
    const float t = -phase;
    const float density01 = clamp(density, 0.0, 1.0);
    const float turbulence01 = clamp(turbulence, 0.0, 1.0);
    const float clampedWarp = clamp(warp, 0.0, 2.0);
    const float lateralScale = max(wavelength * 1.35, 0.012);
    const float frontWidth = max(wavelength * (0.046 + turbulence01 * 0.026), 0.003);
    const float trailLength = max(wavelength * (1.05 + turbulence01 * 0.68), frontWidth * 7.0);
    const float incomingShare = 0.58;
    const float returnShare = 0.30;
    const float waveRate = 0.070 + density01 * 0.045;
    const float warpGuard = wavelength * (0.18 + clampedWarp * 0.16 + turbulence01 * 0.08);
    const float finishOffset = max(edgeBlendWidth, edgeBlendWidth + warpGuard);
    float combined = 0.0;

    for (int waveIndex = 0; waveIndex < 4; ++waveIndex) {
        const float slot = float(waveIndex);
        const float slotSeed = seed + slot * 53.17;
        const float timingNoise = RippleHash(slotSeed * 0.071 + 11.0);
        const float speedNoise = mix(0.91, 1.09, RippleHash(slotSeed * 0.097 + 23.0));
        const float waveGate = RippleHash(slotSeed * 0.113 + 31.0);
        if (waveGate > mix(0.62, 1.0, density01)) {
            continue;
        }

        const float offset =
            slot * 0.235 +
            (timingNoise - 0.5) * (0.12 + turbulence01 * 0.10) +
            RippleSmoothBlockNoise(vec2(t * 0.018, slotSeed), 0.23, seed, 181.0) * 0.10;
        const float cycle = fract(t * waveRate * speedNoise + offset);
        const float activeEnd = incomingShare + returnShare;
        if (cycle >= activeEnd) {
            continue;
        }

        const float scallopNoise = RippleSmoothBlockNoise(
            vec2(uv.y + slot * wavelength * 0.37, seed * 0.13 + slot * 0.41),
            max(wavelength * 0.48, 0.008),
            seed,
            151.0 + slot * 19.0);
        const float frontWarp =
            (sin((uv.y / lateralScale) + seed * 1.17 + slot * 1.91) * 0.62 +
             sin((uv.y / max(wavelength * 0.58, 0.006)) - seed * 0.73 + slot * 2.37) * 0.28 +
             (scallopNoise - 0.5) * 1.15) *
            wavelength * (0.08 + clampedWarp * 0.10 + turbulence01 * 0.06);
        const float x = finishOffset - max(0.0, shoreDistance) - frontWarp;
        const float waveTravel = travelDistance * mix(1.38, 1.82, RippleHash(slotSeed * 0.061 + 47.0));
        const float offshoreStart = -waveTravel * (0.72 + timingNoise * 0.18);
        const float shoreEnd = 0.0;
        const float shoreBreakup = smoothstep(0.24, 0.92, scallopNoise + turbulence01 * 0.22);
        const float foamNoise = RippleSmoothBlockNoise(
            vec2(x * 0.37 + slot * 0.19, uv.y + slot * 0.31),
            max(wavelength * 0.35, 0.006),
            seed,
            203.0 + slot * 23.0);
        const float breakup = smoothstep(
            0.18,
            0.96,
            foamNoise + shoreBreakup * 0.45 + turbulence01 * 0.18);
        const float shorewardMask = 1.0 - smoothstep(frontWidth * 0.45, frontWidth * 2.20, x - shoreEnd);

        if (cycle < incomingShare) {
            const float incomingProgress = smoothstep(0.0, 1.0, cycle / incomingShare);
            const float frontPosition = mix(offshoreStart, shoreEnd, incomingProgress);
            const float front = x - frontPosition;
            const float crest = RippleLine(front, frontWidth);
            const float trailDistance = max(0.0, -front);
            const float trailingFoam =
                exp(-trailDistance / max(trailLength, 1.0e-4)) *
                smoothstep(frontWidth * 0.35, frontWidth * 1.70, trailDistance) *
                (1.0 - smoothstep(trailLength * 1.08, trailLength * 2.15, trailDistance));
            const float crestFade = smoothstep(0.02, 0.18, cycle) * (1.0 - smoothstep(0.91, 1.0, incomingProgress));
            const float value =
                crest * (0.78 + shoreBreakup * 0.24) * crestFade +
                trailingFoam * (0.46 + density01 * 0.34) * breakup;
            combined = max(combined, value * shorewardMask);
        } else {
            const float returnProgress = smoothstep(0.0, 1.0, (cycle - incomingShare) / returnShare);
            const float returnDistance = waveTravel * 0.50;
            const float clearFront = shoreEnd - returnDistance * returnProgress;
            const float front = x - clearFront;
            const float remainingMask = 1.0 - smoothstep(-frontWidth * 1.25, frontWidth * 1.55, front);
            const float trailDistance = max(0.0, shoreEnd - x);
            const float heldFoam =
                exp(-trailDistance / max(trailLength, 1.0e-4)) *
                smoothstep(frontWidth * 0.35, frontWidth * 1.70, trailDistance) *
                (1.0 - smoothstep(trailLength * 1.08, trailLength * 2.15, trailDistance));
            const float returnFade = 1.0 - smoothstep(0.45, 1.0, returnProgress);
            const float value =
                heldFoam * remainingMask * (0.50 + density01 * 0.30) * breakup * returnFade;
            combined = max(combined, value * shorewardMask);
        }
    }

    return clamp(combined, 0.0, 1.0);
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

    for (int waveIndex = 0; waveIndex < 4; ++waveIndex) {
        const float slot = float(waveIndex);
        const float slotSeed = seed + slot * 53.17;
        const float timingNoise = RippleHash(slotSeed * 0.071 + 11.0);
        const float speedNoise = mix(0.91, 1.09, RippleHash(slotSeed * 0.097 + 23.0));
        const float waveGate = RippleHash(slotSeed * 0.113 + 31.0);
        if (waveGate > mix(0.62, 1.0, density01)) {
            continue;
        }

        const float offset =
            slot * 0.235 +
            (timingNoise - 0.5) * (0.12 + turbulence01 * 0.10) +
            RippleSmoothBlockNoise(vec2(t * 0.018, slotSeed), 0.23, seed, 181.0) * 0.10;
        const float cycle = fract(t * waveRate * speedNoise + offset);
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
            const float peakSoftening = 1.0 - smoothstep(0.84, 1.0, incomingProgress) * 0.28;
            const float incomingEdgeFoam =
                (crest * (0.70 + shoreBreakup * 0.22) * foamContinuity +
                 crestHalo * (0.08 + density01 * 0.06) * foamContinuity +
                 arrivingFoam * (0.42 + density01 * 0.28) * breakup) *
                incomingMask;
            const float value = max(incomingBodyFoam, incomingEdgeFoam) * arrivalFade * peakSoftening;
            combined = max(combined, value * shorewardMask);
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
            const float edgeValue =
                (crest * (0.62 + shoreBreakup * 0.20) * foamContinuity +
                 crestHalo * (0.10 + density01 * 0.06) * foamContinuity +
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
            const float peakCarryValue =
                (RippleLine(peakFront, frontWidth) * (0.62 + shoreBreakup * 0.20) * foamContinuity +
                 RippleLine(peakFront, frontWidth * 1.70) * (0.10 + density01 * 0.06) * foamContinuity +
                 peakCarryFoam * (0.38 + density01 * 0.24) * breakup) *
                peakCarryMask * peakCarryFade * 0.72;
            const float value = max(max(heldValue, peakCarryValue), edgeValue * edgeIntroduce);
            combined = max(combined, value * shorewardMask);
        }
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

float RippleWetSheenValue(vec2 uv, vec3 normal, float wavelength, float warp, float turbulence, float density, float seed, float phase) {
    const float slope = clamp(1.0 - abs(normal.z), 0.0, 1.0);
    const float normalGrain = clamp(length(normal.xy), 0.0, 1.0);
    const float t = -phase;
    const float safeWavelength = max(wavelength, 0.005);
    const float clampedWarp = clamp(warp, 0.0, 2.0);
    const vec2 normalBias = normal.xy * safeWavelength * (0.30 + clampedWarp * 0.38);
    const vec2 driftA = vec2(t * (0.034 + turbulence * 0.018), -t * (0.021 + clampedWarp * 0.012));
    const vec2 driftB = vec2(-t * (0.018 + clampedWarp * 0.016), t * (0.029 + turbulence * 0.020));
    const float warpWave =
        sin((uv.y / max(safeWavelength * 1.15, 0.010)) + seed * 0.021 + t * 0.31) *
        safeWavelength * (0.045 + clampedWarp * 0.075 + turbulence * 0.035);
    const vec2 warpedUv = uv + vec2(warpWave, -warpWave * 0.58) + normalBias;
    const float lowA = RippleSmoothBlockNoise(
        warpedUv + driftA,
        max(safeWavelength * 1.70, 0.018),
        seed,
        163.0);
    const float lowB = RippleSmoothBlockNoise(
        uv * 0.73 + normal.yx * safeWavelength * (0.50 + clamp(warp, 0.0, 2.0) * 0.35) + driftB,
        max(safeWavelength * 2.45, 0.024),
        seed,
        173.0);
    const float fine = RippleSmoothBlockNoise(
        warpedUv + vec2(-t * 0.046, t * 0.039),
        max(safeWavelength * (0.30 - turbulence * 0.10), 0.005),
        seed,
        167.0);
    const float micro = RippleBlockNoise(
        warpedUv + normal.yx * safeWavelength * 0.22 + vec2(t * 0.062, -t * 0.047),
        max(safeWavelength * (0.105 - turbulence * 0.030), 0.0025),
        seed,
        181.0);
    const float sheenPatch = smoothstep(
        0.40 - density * 0.22 - turbulence * 0.08,
        0.90,
        lowA * 0.56 + lowB * 0.36 + fine * 0.12);
    const float normalLift = slope * (0.34 + clampedWarp * 0.20) +
                             normalGrain * (0.08 + clampedWarp * 0.05);
    const float patchGate = smoothstep(0.14, 0.70, sheenPatch + lowB * 0.24 + fine * 0.10);
    const float coverage = smoothstep(
        0.14 - density * 0.20,
        0.90,
        sheenPatch * (0.78 + normalLift * 0.44) + lowA * 0.18 + fine * 0.14 +
            normalLift * patchGate * (0.16 + clampedWarp * 0.06));
    const float grain = smoothstep(0.39 - turbulence * 0.20, 0.95, fine * 0.66 + micro * 0.34 + lowA * 0.12) *
                        patchGate;
    const float shimmerWave = 0.50 + 0.50 * sin(
        (uv.x + uv.y * 0.41) / max(safeWavelength * 3.6, 0.020) +
        t * (0.36 + turbulence * 0.22) +
        lowB * kRippleTwoPi);
    const float glint = grain * (0.44 + shimmerWave * (0.36 + turbulence * 0.24));
    const float wetCycle = RippleWavePeak(
        t * (0.24 + turbulence * 0.14) + lowA * 0.73 + lowB * 0.41 + normalGrain * 0.19,
        1.35);
    const float temporalGate = 0.62 + 0.38 * wetCycle;
    const float normalResponse = 0.82 + slope * (0.55 + clampedWarp * 0.12) + normalGrain * 0.16;
    const float sheen =
        (sheenPatch * (0.18 + slope * 0.42 + normalGrain * 0.12) +
         patchGate * slope * clampedWarp * 0.08 +
         glint * (0.24 + turbulence * 0.48)) *
        coverage *
        normalResponse;
    return clamp(sheen * temporalGate, 0.0, 1.0);
}

float RippleDripTrailValue(vec2 uv, vec3 normal, float wavelength, float warp, float turbulence, float density, float seed, float phase) {
    const float density01 = clamp(density, 0.0, 1.0);
    const float densityCurve = sqrt(density01);
    const float cellSize = max(wavelength * 1.45, mix(0.34, 0.115, densityCurve));
    const vec2 p = uv / cellSize;
    const int baseX = int(floor(p.x));
    const int baseY = int(floor(p.y));
    const float t = -phase;
    const float originDensity = clamp(0.10 + density01 * 0.78, 0.06, 0.92);
    const float originSoftMargin = 0.12 + clamp(turbulence, 0.0, 1.0) * 0.14;
    vec2 flowDir = normal.xy * normal.z;
    if (dot(flowDir, flowDir) <= 1.0e-6) {
        flowDir = vec2(1.0, 0.0);
    } else {
        flowDir = normalize(flowDir);
    }
    const vec2 sideDir = vec2(-flowDir.y, flowDir.x);
    float best = 0.0;
    for (int dy = -2; dy <= 2; ++dy) {
        for (int dx = -2; dx <= 2; ++dx) {
            const int cx = baseX + dx;
            const int cy = baseY + dy;
            const float originGate = RippleCellHash(cx, cy, seed, 71.0);
            if (originGate > originDensity + originSoftMargin) {
                continue;
            }
            const float originWeight = 1.0 - smoothstep(originDensity, originDensity + originSoftMargin, originGate);
            vec2 origin = (vec2(float(cx), float(cy)) + RippleCellHash2(cx, cy, seed, 59.0)) * cellSize;
            origin += (RippleCellHash2(cx, cy, seed, 67.0) - 0.5) * cellSize * clamp(warp, 0.0, 2.0) * 0.13;
            const float trailSeed = RippleCellHash(cx, cy, seed, 79.0);
            const float life = fract(t * (0.12 + trailSeed * 0.055 + density01 * 0.018) + trailSeed);
            const float travel = max(wavelength * (1.35 + clamp(warp, 0.0, 8.0) * 0.32 + turbulence * 0.22), 0.040);
            const float head = travel * smoothstep(0.0, 1.0, life);
            const float ageFade = pow(1.0 - life, 0.74) * smoothstep(0.015, 0.12, life);
            const vec2 local = uv - origin;
            const float along = dot(local, flowDir);
            const float cross = dot(local, sideDir);
            const float tailLength = travel * (0.28 + 0.72 * smoothstep(0.05, 0.80, life));
            const float behindHead = head - along;
            const float tail = clamp(behindHead / max(tailLength, 1.0e-5), 0.0, 1.0);
            const float inLength = smoothstep(0.0, wavelength * 0.08, along) *
                                   smoothstep(0.0, wavelength * 0.10, behindHead) *
                                   (1.0 - smoothstep(tailLength * 0.72, tailLength, behindHead));
            const float wiggle =
                sin((along / max(wavelength * 0.40, 0.006)) + trailSeed * kRippleTwoPi + t * (0.42 + turbulence * 0.22)) *
                wavelength * (0.050 + clamp(warp, 0.0, 8.0) * 0.030 + turbulence * 0.050) *
                smoothstep(0.0, travel * 0.52, along);
            const float width = max(wavelength * (0.038 + turbulence * 0.052), 0.0038);
            const float activeWidth = width * (1.0 + tail * (0.65 + turbulence * 0.35));
            const float lateral = RippleLine(cross - wiggle, activeWidth);
            const float wetWidth = activeWidth * (2.0 + turbulence * 1.1) + wavelength * 0.010;
            const float wetTrail = RippleLine(cross - wiggle * 0.55, wetWidth) *
                                   inLength *
                                   (0.18 + tail * (0.28 + turbulence * 0.16));
            const float wakeLength = travel * (0.16 + 0.74 * smoothstep(0.03, 0.92, life));
            const float inWake = smoothstep(-wavelength * 0.035, wavelength * 0.055, along) *
                                 (1.0 - smoothstep(wakeLength * 0.82, wakeLength, along));
            const float wakeWidth = max(wavelength * (0.052 + turbulence * 0.050), 0.0075) *
                                    (1.0 + tail * 0.70);
            const float wake = RippleLine(cross - wiggle * 0.35, wakeWidth) *
                               inWake *
                               (0.14 + turbulence * 0.16) *
                               (0.35 + originWeight * 0.65);
            const float taper = 1.0 - tail * 0.78;
            const float headDrop = RippleLine(length(vec2(along - head, cross - wiggle)), width * (3.1 + turbulence * 1.4));
            const float bead = RippleLine(length(local), width * 2.7) * (1.0 - smoothstep(0.18, 0.42, life));
            const float trail = (lateral * inLength * taper + wetTrail + wake + headDrop * 0.42 + bead * 0.20) *
                                ageFade *
                                (0.50 + originWeight * 0.36 + trailSeed * 0.18);
            best = max(best, trail);
        }
    }
    return clamp(best, 0.0, 1.0);
}

float RippleSaltMineralShimmerValue(vec2 regionUv, vec3 normal, float wavelength, float warp, float turbulence, float density, float seed, float phase) {
    const float t = -phase;
    const float normalBias = clamp(length(normal.xy), 0.0, 1.0);
    const vec2 normalFlow = normalBias > 1.0e-4 ? normal.xy / normalBias : vec2(0.37, -0.21);
    const vec2 mineralAcross = vec2(-normalFlow.y, normalFlow.x);
    const float veinCell = max(wavelength * (1.05 + warp * 0.20 - density * 0.18), 0.018);
    const vec2 lowWarp = vec2(
        RippleSmoothBlockNoise(regionUv + vec2(t * 0.012, -t * 0.009), max(wavelength * 1.35, 0.018), seed, 113.0),
        RippleSmoothBlockNoise(regionUv + vec2(-t * 0.010, t * 0.014), max(wavelength * 1.35, 0.018), seed, 127.0)) -
        vec2(0.5);
    const vec2 mineralUv =
        regionUv +
        normalFlow * wavelength * (0.50 + normalBias * 0.36 + warp * 0.20) +
        mineralAcross * wavelength * normalBias * 0.22 +
        lowWarp * wavelength * (0.32 + warp * 0.30 + turbulence * 0.18);

    const float coarse = RippleSmoothBlockNoise(
        mineralUv + normalFlow * wavelength * 0.35,
        max(wavelength * 1.80, 0.020),
        seed,
        131.0);
    const float splitPhase = 0.5 + 0.5 * sin((t * (0.075 + turbulence * 0.045) + coarse * 0.62 + seed * 0.013) * kRippleTwoPi);
    const float splitBlend = smoothstep(0.18, 0.82, splitPhase);
    const float reconnect = 1.0 - abs(splitBlend * 2.0 - 1.0);

    const vec2 pA = mineralUv / veinCell;
    const vec2 pB =
        (mineralUv +
         mineralAcross * wavelength * (0.42 + normalBias * 0.26) * (splitBlend * 2.0 - 1.0) +
         normalFlow * wavelength * 0.16 * reconnect) /
        veinCell;
    const int baseAX = int(floor(pA.x));
    const int baseAY = int(floor(pA.y));
    const int baseBX = int(floor(pB.x));
    const int baseBY = int(floor(pB.y));
    float nearestA = 1.0e20;
    float secondA = 1.0e20;
    float nearestB = 1.0e20;
    float secondB = 1.0e20;
    float veinSeedA = 0.0;
    float veinSeedB = 0.0;
    for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
            const int ax = baseAX + dx;
            const int ay = baseAY + dy;
            const vec2 hA = RippleCellHash2(ax, ay, seed, 149.0);
            const float angleA = (hA.x * 1.51 + hA.y * 2.07 + t * (0.025 + hA.y * 0.035)) * kRippleTwoPi;
            const vec2 featureA =
                vec2(float(ax), float(ay)) +
                hA +
                vec2(cos(angleA), sin(angleA * 1.17 + hA.x * kRippleTwoPi)) * (0.07 + turbulence * 0.045) +
                normalFlow * normalBias * (hA.x - 0.5) * 0.22;
            const float distanceA = length(pA - featureA);
            if (distanceA < nearestA) {
                secondA = nearestA;
                nearestA = distanceA;
                veinSeedA = hA.x;
            } else if (distanceA < secondA) {
                secondA = distanceA;
            }

            const int bx = baseBX + dx;
            const int by = baseBY + dy;
            const vec2 hB = RippleCellHash2(bx, by, seed, 181.0);
            const float angleB = (hB.x * 1.73 + hB.y * 1.39 - t * (0.030 + hB.x * 0.030)) * kRippleTwoPi;
            const vec2 featureB =
                vec2(float(bx), float(by)) +
                hB +
                vec2(sin(angleB * 1.11 + hB.y * kRippleTwoPi), cos(angleB)) * (0.08 + turbulence * 0.050) -
                mineralAcross * normalBias * (hB.y - 0.5) * 0.20;
            const float distanceB = length(pB - featureB);
            if (distanceB < nearestB) {
                secondB = nearestB;
                nearestB = distanceB;
                veinSeedB = hB.y;
            } else if (distanceB < secondB) {
                secondB = distanceB;
            }
        }
    }

    const float veinWidth = clamp(0.024 + turbulence * 0.018 + density * 0.014 + normalBias * 0.010, 0.014, 0.085);
    const float veinA = 1.0 - smoothstep(veinWidth, veinWidth * 4.0, secondA - nearestA);
    const float veinB = 1.0 - smoothstep(veinWidth * 0.82, veinWidth * 3.8, secondB - nearestB);
    const float bridge = sqrt(max(0.0, veinA * veinB)) * (0.20 + reconnect * 0.48);
    const float veinNetwork = clamp(max(max(veinA * (0.90 - splitBlend * 0.18), veinB * (0.54 + splitBlend * 0.46)), bridge), 0.0, 1.0);
    const float alongVein =
        (mineralUv.x * (0.43 + normalFlow.x * 0.15) + mineralUv.y * (0.31 + normalFlow.y * 0.15)) /
        max(wavelength * 0.36, 0.004);
    const float shimmerWave = RippleWavePeak(
        alongVein + t * (0.22 + turbulence * 0.15) + veinSeedA * 1.37 + veinSeedB * 0.71,
        2.2);
    const float crystal = RippleSmoothBlockNoise(
        mineralUv + normalFlow * t * 0.010 + mineralAcross * t * 0.006,
        max(wavelength * 0.18, 0.004),
        seed,
        193.0);
    const float veinCoverage = smoothstep(
        0.50 - density * 0.24 - normalBias * 0.14,
        0.98,
        veinNetwork + coarse * 0.20);
    const float activityNoise = RippleSmoothBlockNoise(
        mineralUv + normalFlow * t * 0.018 - mineralAcross * t * 0.013,
        max(wavelength * 0.30, 0.006),
        seed,
        197.0);
    const float veinActivity = smoothstep(
        0.18,
        0.92,
        shimmerWave + activityNoise * (0.20 + turbulence * 0.12) + reconnect * 0.08);
    const float brightSplit = 0.36 + 0.64 * RippleWavePeak(splitPhase + crystal * 0.35 + t * 0.035, 1.8);
    const float fineGlint = smoothstep(0.76 - turbulence * 0.16 - density * 0.10, 1.0, crystal + veinNetwork * 0.20);
    const float softVein = veinNetwork * (0.035 + coarse * 0.025) * (0.42 + veinActivity * 0.36);
    const float brightVein = veinNetwork *
                             veinActivity *
                             (0.16 + shimmerWave * 0.46 + fineGlint * 0.20) *
                             brightSplit;
    return clamp(veinCoverage * (softVein + brightVein) * (0.72 + normalBias * 0.38), 0.0, 1.0);
}

float RippleDropletValue(vec2 uv, vec3 normal, float wavelength, float warp, float turbulence, float density, float seed, float phase) {
    const float safeWavelength = max(wavelength, 0.005);
    const float cellSize = max(safeWavelength * 1.45, 0.018);
    const vec2 p = uv / cellSize;
    const int baseX = int(floor(p.x));
    const int baseY = int(floor(p.y));
    const float t = -phase;
    const float normalBias = clamp(length(normal.xy), 0.0, 1.0);
    const float geometryBias = 0.64 + normalBias * (0.24 + clamp(warp, 0.0, 2.0) * 0.06);
    float best = 0.0;
    for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
            const int cellX = baseX + dx;
            const int cellY = baseY + dy;
            const float sparseGate = RippleCellHash(cellX, cellY, seed, 89.0);
            const float keep = smoothstep(
                0.94 - density * 0.70 - clamp(turbulence, 0.0, 1.0) * 0.16,
                1.0,
                sparseGate);
            const float clusterSeed = RippleCellHash(cellX, cellY, seed, 91.0);
            const vec2 center =
                (vec2(float(cellX), float(cellY)) + RippleCellHash2(cellX, cellY, seed, 83.0)) *
                cellSize;
            const vec2 clusterOffset =
                (RippleCellHash2(cellX, cellY, seed, 97.0) - 0.5) *
                cellSize *
                clamp(warp, 0.0, 2.0) *
                0.24;
            const vec2 anchor = center + clusterOffset;
            const float clusterRadius =
                max(safeWavelength * (0.17 + clusterSeed * 0.16 + turbulence * 0.08), 0.0035);
            const float distance = length(uv - anchor);
            const float core = 1.0 - smoothstep(0.0, clusterRadius, distance);
            const vec2 satelliteA =
                anchor + (RippleCellHash2(cellX, cellY, seed, 101.0) - 0.5) * clusterRadius * 2.45;
            const vec2 satelliteB =
                anchor + (RippleCellHash2(cellX, cellY, seed, 103.0) - 0.5) * clusterRadius * 3.10;
            const float satellite = (1.0 - smoothstep(
                                         0.0,
                                         clusterRadius * (0.42 + turbulence * 0.12),
                                         length(uv - satelliteA))) *
                                    0.55 +
                                    (1.0 - smoothstep(
                                         0.0,
                                         clusterRadius * (0.30 + clusterSeed * 0.18),
                                         length(uv - satelliteB))) *
                                    0.34;
            const float waveA = RippleWavePeak(
                t * (0.82 + clusterSeed * 0.52) +
                    (anchor.x * 0.67 + anchor.y * 0.31) / max(safeWavelength * 1.85, 0.012) +
                    sparseGate * 2.1,
                2.4);
            const float waveB = RippleWavePeak(
                t * (1.22 - clusterSeed * 0.32) +
                    (anchor.x * -0.28 + anchor.y * 0.81) / max(safeWavelength * 2.60, 0.018) +
                    clusterSeed * 2.7,
                3.4);
            const float twinkle = RippleWavePeak(t * (1.75 + clusterSeed * 0.65) + sparseGate * 3.6, 5.0);
            const float pulse = 0.18 + 0.58 * (waveA * 0.62 + waveB * 0.38) + 0.24 * twinkle;
            const float cluster = pow(clamp(core, 0.0, 1.0), 2.0) + satellite;
            best = max(best, cluster * keep * pulse * geometryBias);
        }
    }
    return clamp(best, 0.0, 1.0);
}

float RippleCurrentThreadsValue(vec2 uv, vec3 normal, float wavelength, float warp, float turbulence, float density, float seed, float phase) {
    const float t = -phase;
    const float normalBias = clamp(length(normal.xy), 0.0, 1.0);
    const float cellXSize = max(wavelength * (2.10 + warp * 0.35), 0.036);
    const float cellYSize = max(wavelength * (1.05 + turbulence * 0.35), 0.024);
    vec2 currentUv = uv;
    currentUv.x += normalBias * wavelength * (0.10 + warp * 0.06);
    currentUv.y += sin(uv.x / max(wavelength * 1.25, 0.010) + seed * 1.17 + t * 0.13) *
                  wavelength * (0.055 + warp * 0.060 + normalBias * 0.035);
    currentUv.y += sin(uv.x / max(wavelength * 0.47, 0.006) - seed * 0.73 - t * 0.19) *
                  wavelength * turbulence * 0.055;
    const vec2 p = vec2(currentUv.x / cellXSize, currentUv.y / cellYSize);
    const int baseX = int(floor(p.x));
    const int baseY = int(floor(p.y));
    const float originDensity = clamp(0.16 + density * 0.66 + normalBias * 0.16, 0.10, 0.92);
    float best = 0.0;
    for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -2; dx <= 1; ++dx) {
            const int cx = baseX + dx;
            const int cy = baseY + dy;
            const float originGate = RippleCellHash(cx, cy, seed, 199.0);
            if (originGate > originDensity) {
                continue;
            }
            vec2 origin =
                (vec2(float(cx), float(cy)) + RippleCellHash2(cx, cy, seed, 211.0)) *
                vec2(cellXSize, cellYSize);
            const float pulseSeed = RippleCellHash(cx, cy, seed, 223.0);
            origin.y += (pulseSeed - 0.5) * cellYSize * (0.22 + turbulence * 0.18);
            const float life = fract(t * (0.095 + pulseSeed * 0.075 + normalBias * 0.030) + pulseSeed);
            const float travelRange = cellXSize * (0.82 + warp * 0.10 + normalBias * 0.34);
            const float head = life * travelRange;
            const float trailLength = max(wavelength * (1.15 + warp * 0.34 + normalBias * 0.44), 0.035);
            const vec2 local = currentUv - origin;
            const float forward = local.x;
            const float tail = head - forward;
            const float inPulse =
                smoothstep(0.0, wavelength * 0.11, forward) *
                smoothstep(0.0, wavelength * 0.08, tail) *
                (1.0 - smoothstep(trailLength * 0.78, trailLength, tail));
            const float fan = clamp(forward / max(trailLength, 1.0e-4), 0.0, 1.0);
            const float wiggle =
                sin(forward / max(wavelength * 0.42, 0.006) + pulseSeed * kRippleTwoPi + t * 0.37) *
                wavelength * (0.045 + turbulence * 0.055 + warp * 0.022);
            const float spread =
                wavelength * (0.026 + turbulence * 0.026 + normalBias * 0.020) +
                fan * wavelength * (0.072 + warp * 0.048 + normalBias * 0.064);
            const float lateral = local.y - wiggle;
            const float trunk = RippleLine(lateral, spread) * inPulse * (1.0 - fan * 0.42);
            const float headDrop = RippleLine(length(vec2((forward - head) * 0.72, lateral)), spread * 2.75) * 0.36;
            const float branchSeed = RippleCellHash(cx, cy, seed, 227.0);
            const float branchGate = smoothstep(0.54 - density * 0.30, 1.0, branchSeed + normalBias * 0.12);
            const float splitWindow = smoothstep(0.18, 0.48, fan) * (1.0 - smoothstep(0.70, 1.0, fan));
            const float branchSlope = mix(-0.72, 0.72, RippleCellHash(cx, cy, seed, 229.0));
            const float branchOffset = (fan - 0.20) * branchSlope * wavelength * (0.72 + warp * 0.38);
            const float branch =
                max(
                    RippleLine(lateral - branchOffset, spread * 0.58),
                    RippleLine(lateral + branchOffset * 0.68, spread * 0.48)) *
                splitWindow *
                branchGate *
                inPulse;
            const float breakupNoise = RippleSmoothBlockNoise(
                currentUv + vec2(t * 0.018 + pulseSeed, -t * 0.011),
                max(wavelength * (0.24 + turbulence * 0.16), 0.006),
                seed,
                233.0);
            const float breakupPulse = RippleWavePeak(
                forward / max(wavelength * 0.76, 0.008) - t * (0.12 + pulseSeed * 0.05),
                1.8);
            const float breakup = smoothstep(
                0.18 - turbulence * 0.10,
                0.88,
                breakupNoise + breakupPulse * (0.20 + turbulence * 0.16));
            const float pulseCore = RippleLine(
                length(vec2(forward - head, lateral * 0.72)),
                spread * (2.1 + turbulence * 0.7));
            const float pulse = (trunk + headDrop + branch * 0.68 + pulseCore * (0.16 + density * 0.08)) *
                                breakup *
                                inPulse *
                                (0.62 + originGate * 0.22 + normalBias * 0.22);
            best = max(best, pulse);
        }
    }
    const float fallbackNoise = RippleSmoothBlockNoise(
        currentUv + vec2(t * 0.014, -t * 0.009),
        max(wavelength * 0.42, 0.006),
        seed,
        239.0);
    const float fallbackPulse =
        RippleWavePeak(
            currentUv.x / max(wavelength * 1.9, 0.012) - t * 0.22 + fallbackNoise,
            2.1) *
        smoothstep(0.42 - density * 0.16, 0.96, fallbackNoise);
    const float softFallback =
        fallbackPulse *
        (0.025 + density * 0.020) *
        (0.35 + 0.65 * RippleWavePeak(t * 0.41 + fallbackNoise * 1.7, 1.6));
    return clamp((best + softFallback) * (0.78 + normalBias * 0.30), 0.0, 1.0);
}

float RippleFoamSparkleValue(vec2 regionUv, float edge, float wavelength, float warp, float turbulence, float density, float seed, float phase) {
    const float t = -phase;
    const float density01 = clamp(density, 0.0, 1.0);
    const float turbulence01 = clamp(turbulence, 0.0, 1.0);
    const float driftAmount = clamp(warp, 0.0, 2.0);
    const vec2 drift = vec2(t * (0.018 + driftAmount * 0.010), -t * (0.012 + turbulence01 * 0.008)) * driftAmount;
    const vec2 lowWarp = vec2(
        RippleSmoothBlockNoise(regionUv + vec2(t * 0.017, -t * 0.013) * driftAmount, max(wavelength * 1.85, 0.024), seed, 109.0),
        RippleSmoothBlockNoise(regionUv + vec2(-t * 0.014, t * 0.019) * driftAmount, max(wavelength * 1.85, 0.024), seed, 113.0)) -
        vec2(0.5);
    const vec2 foamUv = regionUv + drift + lowWarp * wavelength * driftAmount * (0.28 + driftAmount * 0.24 + turbulence01 * 0.18);
    const float patchCellSize = max(wavelength * (0.98 + density01 * 0.56), 0.018);
    const vec2 p = foamUv / patchCellSize;
    const int baseX = int(floor(p.x));
    const int baseY = int(floor(p.y));
    float nearest = 1.0e20;
    float secondNearest = 1.0e20;
    float thirdNearest = 1.0e20;
    float nearestSeed = 0.0;
    float secondSeed = 0.0;
    float nearestPresence = 0.0;
    float secondPresence = 0.0;
    float nearestLife = 0.0;
    float secondLife = 0.0;
    for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
            const int cx = baseX + dx;
            const int cy = baseY + dy;
            const vec2 h = RippleCellHash2(cx, cy, seed, 131.0);
            const float cycle = t * (0.026 + turbulence01 * 0.034) + h.x * 2.17 + h.y * 0.83;
            const float cycleIndex = floor(cycle);
            const float life = fract(cycle);
            const vec2 cycleH = RippleCellHash2(cx, cy, seed, 157.0 + cycleIndex * 29.0);
            const float rise = smoothstep(0.05, 0.24, life);
            const float fall = 1.0 - smoothstep(0.58 + density01 * 0.20, 0.98, life);
            const float presence = rise * fall;
            const vec2 site = mix(h, cycleH, 0.48 + turbulence01 * 0.18);
            const vec2 cycleOffset = (cycleH - 0.5) * (0.13 + density01 * 0.08 + turbulence01 * 0.10);
            const float angle =
                (h.x * 1.91 + h.y * 2.37 + cycleH.x * 1.11 + t * (0.020 + h.x * 0.025) * driftAmount) *
                kRippleTwoPi;
            const vec2 cellWobble = vec2(cos(angle), sin(angle * 1.07 + h.y * kRippleTwoPi)) *
                                    driftAmount *
                                    (0.026 + turbulence01 * 0.018);
            const vec2 center = vec2(float(cx), float(cy)) + site + cycleOffset + cellWobble;
            const float distance =
                length(p - center) +
                (1.0 - presence) * (0.36 + turbulence01 * 0.20 + (1.0 - density01) * 0.10);
            const float cellSeed = RippleCellHash(cx, cy, seed, 137.0 + cycleIndex * 13.0);
            if (distance < nearest) {
                thirdNearest = secondNearest;
                secondNearest = nearest;
                secondSeed = nearestSeed;
                secondPresence = nearestPresence;
                secondLife = nearestLife;
                nearest = distance;
                nearestSeed = cellSeed;
                nearestPresence = presence;
                nearestLife = life;
            } else if (distance < secondNearest) {
                thirdNearest = secondNearest;
                secondNearest = distance;
                secondSeed = cellSeed;
                secondPresence = presence;
                secondLife = life;
            } else if (distance < thirdNearest) {
                thirdNearest = distance;
            }
        }
    }
    const float ridgeDistance = secondNearest - nearest;
    const float ridgeWidth = 0.050 + density01 * 0.060 + turbulence01 * 0.036;
    const float ridgePresence = smoothstep(0.04, 0.62, nearestPresence * secondPresence);
    const float ridgeAge = max(nearestLife, secondLife);
    const float cellRidge =
        (1.0 - smoothstep(ridgeWidth, ridgeWidth * (3.8 + turbulence01 * 1.2), ridgeDistance)) *
        ridgePresence;
    const float junction =
        (1.0 - smoothstep(ridgeWidth * 2.0, ridgeWidth * 7.0, thirdNearest - nearest)) *
        ridgePresence;
    const float foamNoise = RippleSmoothBlockNoise(
        foamUv + vec2(t * 0.026, -t * 0.018) * driftAmount,
        max(wavelength * (0.36 + turbulence01 * 0.16), 0.007),
        seed,
        149.0);
    const float fineA = RippleSmoothBlockNoise(
        foamUv + vec2(foamNoise, -foamNoise) * wavelength * 0.31 + vec2(t * 0.041, t * 0.027) * driftAmount,
        max(wavelength * (0.13 + turbulence01 * 0.06), 0.0035),
        seed,
        107.0);
    const float fineB = RippleSmoothBlockNoise(
        foamUv * 1.37 + vec2(-foamNoise, foamNoise) * wavelength * 0.19 + vec2(-t * 0.033, t * 0.022) * driftAmount,
        max(wavelength * (0.095 + turbulence01 * 0.045), 0.003),
        seed,
        151.0);
    const float fineFleck = fineA * 0.62 + fineB * 0.38;
    const float breakupPulse = RippleWavePeak(
        t * (0.10 + turbulence01 * 0.16) + nearestSeed * 0.71 + secondSeed * 0.29 + foamNoise * 0.35,
        1.7);
    const float ageBreakup = smoothstep(0.30, 0.90, ridgeAge);
    const float breakup = smoothstep(
        0.24 - density01 * 0.16,
        0.92,
        foamNoise + breakupPulse * (0.18 + turbulence01 * 0.16) + cellRidge * 0.18);
    const float chips = smoothstep(
        0.74 - density01 * 0.12 - turbulence01 * 0.18,
        1.0,
        fineFleck + breakupPulse * 0.18 + ageBreakup * (0.28 + turbulence01 * 0.22));
    const float brokenRidge = cellRidge * breakup * (1.0 - chips * (0.22 + turbulence01 * 0.32));
    const float sparkle = smoothstep(
        0.62 - density01 * 0.22 - turbulence01 * 0.14,
        1.0,
        fineFleck + brokenRidge * 0.25 + junction * 0.18);
    const float pulse = 0.76 + 0.24 * RippleWavePeak(nearestSeed + secondSeed * 0.37 + fineFleck + t * (0.17 + turbulence01 * 0.18), 2.4);
    const float edgeFade = smoothstep(0.05, 0.34, edge);
    const float foam = brokenRidge * (0.82 + junction * 0.42) + sparkle * (0.14 + turbulence01 * 0.08);
    return clamp(edgeFade * foam * pulse, 0.0, 1.0);
}

float RipplePatternValue(uint overlayType, vec2 uv, vec2 regionUv, vec3 normal, float edge, float shoreDistance, float edgeBlendWidth, float wavelength, float warp, float turbulence, float density, float seed, float phase) {
    const float radialDistance = length(uv);
    const float regionRadialDistance = length(regionUv);
    if (overlayType == 0u) {
        return RippleCausticLaceValue(uv, wavelength, warp, turbulence, density, seed, phase);
    }
    if (overlayType == 1u) {
        return RippleWavePeak((uv.x / wavelength) + phase, 4.0);
    }
    if (overlayType == 2u) {
        return RippleWavePeak((regionRadialDistance / wavelength) + phase, 6.0);
    }
    if (overlayType == 3u) {
        return RippleRainRingValue(regionUv, wavelength, warp, turbulence, density, seed, phase);
    }
    if (overlayType == 4u) {
        return RippleTideBandsValue(uv, shoreDistance, edgeBlendWidth, wavelength, warp, turbulence, density, seed, phase);
    }
    if (overlayType == 5u) {
        return RippleWetSheenValue(uv, normal, wavelength, warp, turbulence, density, seed, phase);
    }
    if (overlayType == 6u) {
        return RippleCurrentThreadsValue(uv, normal, wavelength, warp, turbulence, density, seed, phase);
    }
    if (overlayType == 7u) {
        return RippleDropletValue(regionUv, normal, wavelength, warp, turbulence, density, seed, phase);
    }
    if (overlayType == 8u) {
        return RippleDripTrailValue(regionUv, normal, wavelength, warp, turbulence, density, seed, phase);
    }
    if (overlayType == 9u) {
        return RippleFoamSparkleValue(regionUv, edge, wavelength, warp, turbulence, density, seed, phase);
    }
    return RippleSaltMineralShimmerValue(regionUv, normal, wavelength, warp, turbulence, density, seed, phase);
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
    const float releasedSourceMask =
        !upstream && upstreamReachedNode
            ? sourceMask
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
        // All authored wave launches and their slow evolution were composed
        // once for this node into a fixed-span longitudinal field. Per point,
        // stationary world noise only bows/roughens the lookup coordinate.
        // This keeps Strength/Rain reveals from teleporting wave phase and
        // replaces the former 7–12-wave inner loop with one interpolation.
        const float frontWarp =
            centredNoise * spacing * irregularity * 0.25;
        const float fieldDistance =
            downstream + baseBowedFront + frontWarp;
        const float pulse = clamp(
            SampleSeepagePulseField(
                nodeIndex,
                transition,
                fieldDistance) *
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

SparseRippleComposite EvaluateShorelineWaveContribution(vec3 worldPosition, vec3 pointNormal, float timeSeconds) {
    SparseRippleComposite contribution = EmptySparseRippleComposite();
    if (!HasShorelineWaveEffect()) {
        return contribution;
    }

    const float boundaryZ = styleData.shorelineWaveParams0.x;
    const float reachMeters = max(0.001, styleData.shorelineWaveParams0.y);
    const float edgeFadeMeters = max(0.0, styleData.shorelineWaveParams0.z);
    const float heightMask = ShorelineHeightMask(boundaryZ, reachMeters, edgeFadeMeters, worldPosition.z);
    if (heightMask <= 1e-5) {
        return contribution;
    }

    vec2 tangent = vec2(styleData.shorelineWaveParams1.x, styleData.shorelineWaveParams1.y);
    tangent = dot(tangent, tangent) > 1e-8 ? normalize(tangent) : vec2(1.0, 0.0);
    const float tangentCoordinate = dot(worldPosition.xy, tangent);
    const float shoreDistance = max(0.0, boundaryZ - worldPosition.z);
    const float patternScale = max(0.01, styleData.shorelineWaveParams1.z);
    const float wavelength = max(0.002, styleData.shorelineWaveParams1.w);
    const float shorelineEdgeBlendWidth =
        max(0.001, min(max(edgeFadeMeters, wavelength * 0.20), reachMeters * 0.45));
    const float timePhase =
        styleData.shorelineWaveParams3.x + (timeSeconds * max(0.0, styleData.shorelineWaveParams2.x));
    float pattern = 0.0;
    if (styleData.shorelineWaveControl.z == 1u) {
        pattern = HeightFoamShorelineWaveValue(
            vec2(shoreDistance, tangentCoordinate),
            worldPosition.z,
            boundaryZ,
            styleData.shorelineWaveParams5.x,
            reachMeters,
            edgeFadeMeters,
            patternScale,
            wavelength,
            styleData.shorelineWaveParams2.y,
            styleData.shorelineWaveParams2.z,
            styleData.shorelineWaveParams2.w,
            styleData.shorelineWaveParams5.y,
            styleData.shorelineWaveParams5.z,
            styleData.shorelineWaveParams5.w,
            float(styleData.shorelineWaveControl.y),
            timePhase);
    } else {
        pattern = SandCloudShorelineWaveValue(
            vec2(shoreDistance, tangentCoordinate) * patternScale,
            shoreDistance,
            shorelineEdgeBlendWidth,
            wavelength,
            styleData.shorelineWaveParams2.y,
            styleData.shorelineWaveParams2.z,
            styleData.shorelineWaveParams2.w,
            float(styleData.shorelineWaveControl.y),
            timePhase);
    }

    const float normalMask =
        clamp(0.62 + 0.38 * abs(dot(RippleSafeNormal(pointNormal), vec3(0.0, 0.0, 1.0))), 0.0, 1.0);
    const float scale = clamp(pattern * heightMask * normalMask * styleData.shorelineWaveParams0.w, 0.0, 1.0);
    if (scale <= 1e-5) {
        return contribution;
    }

    contribution.scale = scale;
    contribution.colourMix = clamp(styleData.shorelineWaveParams4.z * scale, 0.0, 1.0);
    contribution.emissionAdd = max(0.0, styleData.shorelineWaveParams3.y) * scale;
    contribution.opacityAdd = styleData.shorelineWaveParams3.z * scale;
    contribution.opacityMultiply = mix(1.0, max(0.0, styleData.shorelineWaveParams3.w), scale);
    contribution.pointSizeAdd = styleData.shorelineWaveParams4.x * scale;
    contribution.pointSizeMultiply = mix(1.0, max(0.0, styleData.shorelineWaveParams4.y), scale);
    contribution.colour = clamp(styleData.shorelineWaveTint.rgb, vec3(0.0), vec3(1.0));
    return contribution;
}

SparseRippleComposite EvaluateSparseRippleContribution(SparseRippleMembership membership, SparseRippleParams params, vec3 worldPosition, vec3 pointNormal, float timeSeconds) {
    SparseRippleComposite contribution = EmptySparseRippleComposite();
    const vec3 normal = RippleSafeNormal(pointNormal);
    vec3 direction = params.region1.xyz;
    direction = dot(direction, direction) > 1e-8 ? normalize(direction) : vec3(1.0, 0.0, 0.0);
    vec3 tangent = direction - normal * dot(direction, normal);
    tangent = dot(tangent, tangent) > 1e-8 ? normalize(tangent) : RippleSafeLateral(normal);
    vec3 lateral = cross(normal, tangent);
    lateral = dot(lateral, lateral) > 1e-8 ? normalize(lateral) : RippleSafeLateral(tangent);

    const float edgeDistance = max(0.0, membership.data.x);
    const float shoreDistance = max(0.0, membership.data.z);
    const float edge = smoothstep(0.0, max(1e-5, params.region1.w), edgeDistance);
    const float strength = clamp(params.region0.w, 0.0, 1.0);
    const bool causticLace = params.control.x == 0u;
    if (strength <= 1e-5 || (!causticLace && edge <= 1e-5)) {
        return contribution;
    }

    const float patternScale = clamp(params.pattern0.x, 0.05, 100.0);
    const float scaledShoreDistance = shoreDistance * patternScale;
    const float scaledEdgeBlendWidth = params.region1.w * patternScale;
    const float wavelength = max(0.005, params.pattern0.y);
    const float speed = max(0.0, params.pattern0.z);
    const float warp = max(0.0, params.pattern0.w);
    const float turbulence = max(0.0, params.pattern1.x);
    const float density = clamp(params.pattern1.w, 0.0, 1.0);
    const float phase = params.pattern1.y - max(0.0, timeSeconds) * speed;
    const float intensity = max(0.0, params.pattern1.z);
    const vec3 relative = worldPosition - params.region0.xyz;
    const vec2 uv = vec2(dot(relative, tangent), dot(relative, lateral)) * patternScale;
    const vec2 regionUv = relative.xy * patternScale;
    const float seed =
        float(params.control.z) * 0.013 +
        float(params.control.w) * 0.017 +
        RippleOverlaySalt(params.control.x) * 0.011;
    const float value = RipplePatternValue(
        params.control.x,
        uv,
        regionUv,
        normal,
        edge,
        scaledShoreDistance,
        scaledEdgeBlendWidth,
        wavelength,
        warp,
        turbulence,
        density,
        seed,
        phase);
    const float edgeFactor = causticLace ? 1.0 : edge;
    const float scale = clamp(value * intensity * strength * edgeFactor, 0.0, 1.0);
    contribution.scale = scale;
    contribution.colourMix = clamp(params.response2.x * scale, 0.0, 1.0);
    contribution.emissionAdd = max(0.0, params.response0.x) * scale;
    contribution.opacityAdd = params.response0.y * scale;
    contribution.opacityMultiply = mix(1.0, max(0.0, params.response0.z), scale);
    contribution.pointSizeAdd = params.response0.w * scale;
    contribution.pointSizeMultiply = mix(1.0, max(0.0, params.response1.x), scale);
    contribution.colour = clamp(params.response1.yzw, vec3(0.0), vec3(1.0));
    return contribution;
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
    BlendSeepageContributions(result, worldPosition, pointNormal, pointIndex, timeSeconds);
    if (!HasSparseRippleEffects()) {
        return SanitizeSparseRippleComposite(result);
    }
    if (styleData.pointMeta.x == 0u || pointIndex >= styleData.pointMeta.x) {
        return SanitizeSparseRippleComposite(result);
    }
    const SparseRippleRange pointRange = sparseRippleRangeData.sparseRippleRanges[pointIndex];
    const uint start = pointRange.range.x;
    const uint count = pointRange.range.y;
    if (count == 0u || start >= styleData.rippleEffectSlots3.y) {
        return SanitizeSparseRippleComposite(result);
    }
    const uint available = styleData.rippleEffectSlots3.y - start;
    const uint cappedEnd = start + min(count, available);
    for (uint entryIndex = start; entryIndex < cappedEnd; ++entryIndex) {
        const SparseRippleMembership membership = sparseRippleMembershipData.sparseRippleMemberships[entryIndex];
        const uint paramIndex = membership.control.x;
        if (paramIndex >= styleData.rippleEffectSlots3.z) {
            continue;
        }
        const SparseRippleParams params = sparseRippleParamData.sparseRippleParams[paramIndex];
        const SparseRippleComposite contribution =
            EvaluateSparseRippleContribution(membership, params, worldPosition, pointNormal, timeSeconds);
        BlendSparseRippleContribution(result, contribution, params.control.y);
    }
    return SanitizeSparseRippleComposite(result);
}

vec3 ApplySparseRippleColor(vec3 baseColor, SparseRippleComposite ripple) {
    if (ripple.colourMix <= 1e-5) {
        return baseColor;
    }
    return mix(baseColor, ripple.colour, clamp(ripple.colourMix, 0.0, 1.0));
}
