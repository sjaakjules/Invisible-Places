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

layout(set = 0, binding = 7, std430) readonly buffer SparseRippleRanges {
    SparseRippleRange sparseRippleRanges[];
} sparseRippleRangeData;

layout(set = 0, binding = 8, std430) readonly buffer SparseRippleMemberships {
    SparseRippleMembership sparseRippleMemberships[];
} sparseRippleMembershipData;

layout(set = 0, binding = 9, std430) readonly buffer SparseRippleParamsBuffer {
    SparseRippleParams sparseRippleParams[];
} sparseRippleParamData;

bool HasSparseRippleEffects() {
    return styleData.rippleEffectSlots3.x != 0u &&
           styleData.rippleEffectSlots3.y != 0u &&
           styleData.rippleEffectSlots3.z != 0u;
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
    const float lineWidth = clamp(0.012 + turbulence * 0.026 + density * 0.014, 0.008, 0.10);
    const float ridge = 1.0 - smoothstep(lineWidth, lineWidth * 4.0, ridgeDistance);
    const float filamentA = RippleWavePeak((p.x * 0.23 + p.y * 0.71) + t * 0.045 + seed * 0.17, 7.0);
    const float filamentB = RippleWavePeak((p.x * -0.52 + p.y * 0.34) - t * 0.038 + seed * 0.11, 9.0);
    const float shimmer = 0.80 + 0.20 * RippleSmoothBlockNoise(
                                           uv + vec2(t * 0.021, -t * 0.017),
                                           cellSize * 0.33,
                                           seed,
                                           43.0);
    const float coverage = smoothstep(0.84 - density * 0.42, 1.0, ridge + max(filamentA, filamentB) * 0.22);
    const float lace = max(pow(clamp(ridge, 0.0, 1.0), 1.85), max(filamentA, filamentB) * 0.32 * ridge);
    const float softRidge = (1.0 - smoothstep(lineWidth * 2.2, lineWidth * 8.0, ridgeDistance)) * density * 0.10;
    return clamp(max(lace * coverage, softRidge) * shimmer, 0.0, 1.0);
}

float RippleRainRingValue(vec2 uv, float wavelength, float warp, float turbulence, float density, float seed, float phase) {
    const float cellSize = max(wavelength * 2.8, 0.025);
    const vec2 p = uv / cellSize;
    const int baseX = int(floor(p.x));
    const int baseY = int(floor(p.y));
    const float t = -phase;
    const float rainDensity = clamp(0.05 + density * 0.75, 0.02, 0.88);
    const float width = max(wavelength * (0.030 + turbulence * 0.018), 0.0025);
    const float maxRadius = cellSize * 0.86;
    float best = 0.0;
    for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
            const int cx = baseX + dx;
            const int cy = baseY + dy;
            const float dropGate = RippleCellHash(cx, cy, seed, 71.0);
            if (dropGate > rainDensity) {
                continue;
            }
            vec2 center = (vec2(float(cx), float(cy)) + RippleCellHash2(cx, cy, seed, 59.0)) * cellSize;
            center += (RippleCellHash2(cx, cy, seed, 67.0) - 0.5) * cellSize * clamp(warp, 0.0, 2.0) * 0.18;
            const float distance = length(uv - center);
            const float dropSeed = RippleCellHash(cx, cy, seed, 79.0);
            const float life = fract(t * (0.18 + dropSeed * 0.08) + dropSeed);
            const float radius = maxRadius * smoothstep(0.0, 1.0, life);
            const float fade = pow(1.0 - life, 1.35) * smoothstep(0.025, 0.14, life);
            const float ring = RippleLine(distance - radius, width);
            const float outer = RippleLine(distance - radius * 0.68, width * 0.72) * 0.24;
            best = max(best, (ring + outer) * fade * (0.72 + dropGate * 0.28));
        }
    }
    return clamp(best, 0.0, 1.0);
}

float RippleTideBandsValue(vec2 uv, float wavelength, float warp, float turbulence, float seed, float phase) {
    const float travelDistance = max(wavelength, 0.015);
    const float t = -phase;
    const float cycle = 0.5 - 0.5 * cos(t * 0.095 * kRippleTwoPi);
    const float travel = (cycle * 2.0 - 1.0) * travelDistance;
    const float lateralScale = max(wavelength * 1.35, 0.012);
    const float frontWarp =
        (sin((uv.y / lateralScale) + seed * 1.17 + t * 0.040) +
         0.52 * sin((uv.y / max(wavelength * 0.43, 0.006)) - seed * 0.73 + t * 0.027)) *
        wavelength * (0.14 + clamp(warp, 0.0, 8.0) * 0.12 + turbulence * 0.08);
    const float front = uv.x - travel - frontWarp;
    const float wash = 1.0 - smoothstep(wavelength * 0.06, wavelength * 1.08, abs(front));
    const float crest = RippleLine(front, wavelength * (0.16 + turbulence * 0.06));
    const float scallop = 0.65 + 0.35 * RippleSmoothBlockNoise(
                                         vec2(uv.y, travel + frontWarp),
                                         max(wavelength * 0.52, 0.008),
                                         seed,
                                         151.0);
    const float shoreFoam = smoothstep(0.18, 1.0, scallop + turbulence * 0.22);
    return clamp((wash * 0.22 + crest * 0.78) * shoreFoam, 0.0, 1.0);
}

float RippleWetSheenValue(vec2 uv, vec3 normal, float wavelength, float warp, float turbulence, float density, float seed, float phase) {
    const float slope = clamp(1.0 - abs(normal.z), 0.0, 1.0);
    const float normalGrain = clamp(length(normal.xy), 0.0, 1.0);
    const float t = -phase;
    const float low = RippleSmoothBlockNoise(
        uv + normal.xy * wavelength * 0.65 + vec2(t * 0.012, -t * 0.009),
        max(wavelength * 1.80, 0.018),
        seed,
        163.0);
    const float fine = RippleSmoothBlockNoise(
        uv + normal.yx * wavelength * 0.35 + vec2(-t * 0.019, t * 0.015),
        max(wavelength * 0.34, 0.006),
        seed,
        167.0);
    const float coverage = smoothstep(
        0.50 - density * 0.26,
        0.99,
        low * 0.36 + slope * (0.78 + warp * 0.16) + normalGrain * 0.24 + fine * 0.16);
    const float glint = smoothstep(0.76 - turbulence * 0.10, 0.98, fine) *
                        (0.72 + 0.28 * RippleWavePeak(t * 0.18 + low, 2.0));
    const float pulse = 0.84 + 0.16 * sin(t * 0.42 + low * kRippleTwoPi + normalGrain * 2.0);
    const float sheen = (0.12 + slope * 0.78 + glint * (0.18 + turbulence * 0.16)) * coverage;
    return clamp(sheen * pulse, 0.0, 1.0);
}

float RippleDripTrailValue(vec2 uv, float wavelength, float warp, float turbulence, float density, float seed, float phase) {
    const float cellXSize = max(wavelength * 2.35, 0.040);
    const float cellYSize = max(wavelength * 1.25, 0.024);
    const vec2 p = vec2(uv.x / cellXSize, uv.y / cellYSize);
    const int baseX = int(floor(p.x));
    const int baseY = int(floor(p.y));
    const float t = -phase;
    const float originDensity = clamp(0.04 + density * 0.74, 0.02, 0.86);
    float best = 0.0;
    for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
            const int cx = baseX + dx;
            const int cy = baseY + dy;
            const float originGate = RippleCellHash(cx, cy, seed, 137.0);
            if (originGate > originDensity) {
                continue;
            }
            const vec2 origin =
                (vec2(float(cx), float(cy)) + RippleCellHash2(cx, cy, seed, 139.0)) *
                vec2(cellXSize, cellYSize);
            const float trailSeed = RippleCellHash(cx, cy, seed, 149.0);
            const float life = fract(t * (0.11 + trailSeed * 0.07) + trailSeed);
            const float trailLength = max(wavelength * (1.55 + clamp(warp, 0.0, 8.0) * 0.35), 0.035);
            const float head = life * trailLength;
            const vec2 local = uv - origin;
            const float behindHead = head - local.x;
            const float tail = clamp(behindHead / trailLength, 0.0, 1.0);
            const float inLength = smoothstep(0.0, 0.08, behindHead) *
                                   (1.0 - smoothstep(trailLength * 0.78, trailLength, behindHead));
            const float wiggle =
                sin((local.x / max(wavelength * 0.42, 0.006)) + trailSeed * kRippleTwoPi + t * 0.58) *
                wavelength * (0.08 + clamp(warp, 0.0, 8.0) * 0.035 + turbulence * 0.04);
            const float width = max(wavelength * (0.030 + turbulence * 0.030), 0.0025);
            const float lateral = RippleLine(local.y - wiggle, width);
            const float taper = 1.0 - tail * 0.72;
            const float headDrop = RippleLine(length(vec2(local.x - head, local.y - wiggle)), width * 2.6);
            best = max(best, lateral * inLength * taper + headDrop * 0.30);
        }
    }
    return clamp(best, 0.0, 1.0);
}

float RippleSaltMineralShimmerValue(vec2 regionUv, vec3 normal, float wavelength, float warp, float turbulence, float density, float seed, float phase) {
    const float t = -phase;
    const float normalBias = clamp(length(normal.xy), 0.0, 1.0);
    const float coarse = RippleSmoothBlockNoise(
        regionUv + normal.xy * wavelength * (0.45 + warp * 0.15) + vec2(t * 0.010, -t * 0.007),
        max(wavelength * 1.65, 0.020),
        seed,
        113.0);
    const float mineralPatch =
        0.12 + 0.88 * smoothstep(0.66 - density * 0.44, 0.98, coarse + normalBias * (0.16 + warp * 0.05));
    const float grain = RippleBlockNoise(
        regionUv + vec2(t * 0.009, -t * 0.006),
        max(wavelength * 0.18, 0.004),
        seed,
        127.0);
    const float micro = RippleBlockNoise(
        regionUv + vec2(-t * 0.014, t * 0.011),
        max(wavelength * 0.060, 0.0025),
        seed,
        131.0);
    const float crystal = smoothstep(0.58, 0.96, grain);
    const float fleck = smoothstep(0.80 - turbulence * 0.12 - density * 0.10, 1.0, micro) *
                        (0.70 + 0.30 * RippleWavePeak(t * 0.21 + grain, 2.0));
    return clamp(mineralPatch * (0.06 + crystal * 0.52 + fleck * 0.68), 0.0, 1.0);
}

float RippleDropletValue(vec2 uv, vec3 normal, float wavelength, float warp, float turbulence, float density, float seed, float phase) {
    const float cellSize = max(wavelength * 1.65, 0.025);
    const vec2 p = uv / cellSize;
    const int cellX = int(floor(p.x));
    const int cellY = int(floor(p.y));
    const vec2 center = (vec2(float(cellX), float(cellY)) + RippleCellHash2(cellX, cellY, seed, 83.0)) * cellSize;
    const float sparseGate = RippleCellHash(cellX, cellY, seed, 89.0);
    const float keep = smoothstep(0.96 - density * 0.72 - clamp(turbulence, 0.0, 1.0) * 0.14, 1.0, sparseGate);
    const vec2 clusterOffset = (RippleCellHash2(cellX, cellY, seed, 91.0) - 0.5) * cellSize * clamp(warp, 0.0, 2.0) * 0.18;
    const float distance = length(uv - center - clusterOffset);
    const float geometryBias = 0.72 + 0.28 * clamp(length(normal.xy), 0.0, 1.0);
    const float glint = 1.0 - smoothstep(0.0, cellSize * (0.18 + turbulence * 0.05), distance);
    const float satellite = 1.0 - smoothstep(
        0.0,
        cellSize * 0.11,
        length(uv - center - (RippleCellHash2(cellX, cellY, seed, 93.0) - 0.5) * cellSize * 0.55));
    const float pulse = 0.55 + 0.45 * RippleWavePeak(phase + sparseGate * 1.73, 2.0);
    return clamp((pow(clamp(glint, 0.0, 1.0), 2.2) + satellite * 0.42) * keep * pulse * geometryBias, 0.0, 1.0);
}

float RippleCurrentThreadsValue(vec2 uv, float wavelength, float warp, float turbulence, float density, float seed, float phase) {
    const float spacing = max(wavelength * 0.90, 0.010);
    const float t = -phase * 0.45;
    vec2 warped = uv;
    warped.y += sin(uv.x / max(wavelength * 1.8, 0.012) + seed * 1.2 + t * 0.18) *
                wavelength * (0.10 + warp * 0.18);
    warped.y += sin(uv.x / max(wavelength * 0.62, 0.006) - seed * 0.7 - t * 0.11) *
                wavelength * turbulence * 0.13;
    const float lane = floor(warped.y / spacing);
    const float laneSeed = RippleCellHash(int(lane), int(floor(warped.x / max(spacing * 6.0, 0.02))), seed, 97.0);
    const float laneCenter = (lane + 0.18 + laneSeed * 0.64) * spacing;
    const float keep = 0.55 + 0.45 * smoothstep(0.78 - density * 0.55, 1.0, laneSeed);
    const float width = spacing * (0.080 + turbulence * 0.045);
    const float mainLine = RippleLine(warped.y - laneCenter, width);
    const float branchPhase = floor(warped.x / max(wavelength * 1.55, 0.012));
    const float branchSeed = RippleCellHash(int(lane), int(branchPhase), seed, 101.0);
    const float branchGate = smoothstep(0.70 - density * 0.42, 1.0, branchSeed);
    const float branchSlope = mix(-0.42, 0.42, RippleCellHash(int(lane), int(branchPhase), seed, 103.0));
    const float branchX = fract(warped.x / max(wavelength * 1.55, 0.012));
    const float branchEnvelope = smoothstep(0.05, 0.28, branchX) * (1.0 - smoothstep(0.58, 0.98, branchX));
    const float branch = RippleLine(
        warped.y - laneCenter - (branchX - 0.18) * branchSlope * wavelength,
        width * 0.72) * branchEnvelope * branchGate;
    const float broken = smoothstep(
        0.22,
        0.94,
        0.5 + 0.5 * sin((warped.x / max(wavelength * 2.4, 0.012)) + laneSeed * kRippleTwoPi - phase * 0.55));
    return clamp((mainLine * broken + branch * 0.85) * keep, 0.0, 1.0);
}

float RippleFoamSparkleValue(vec2 regionUv, float edge, float wavelength, float warp, float turbulence, float density, float seed, float phase) {
    const float t = -phase;
    const float edgeBand = 0.12 + smoothstep(0.01, 0.10, edge) * (1.0 - smoothstep(0.78, 1.0, edge));
    const vec2 foamUv = regionUv + vec2(t * 0.018, -t * 0.011) * (0.4 + warp);
    const float foamPatch = RippleSmoothBlockNoise(foamUv, max(wavelength * 2.2, 0.018), seed, 109.0);
    const float fleck = RippleBlockNoise(foamUv, max(wavelength * 0.26, 0.004), seed, 107.0);
    const float keep = 0.20 + 0.80 * smoothstep(0.74 - density * 0.54 - turbulence * 0.12, 1.0, fleck + foamPatch * 0.18);
    const float pulse = 0.45 + 0.55 * smoothstep(0.20, 1.0, fract(fleck + t * (0.23 + turbulence * 0.19)));
    return clamp(edgeBand * keep * pulse * (0.62 + foamPatch * 0.38), 0.0, 1.0);
}

float RipplePatternValue(uint overlayType, vec2 uv, vec2 regionUv, vec3 normal, float edge, float wavelength, float warp, float turbulence, float density, float seed, float phase) {
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
        return RippleTideBandsValue(uv, wavelength, warp, turbulence, seed, phase);
    }
    if (overlayType == 5u) {
        return RippleWetSheenValue(uv, normal, wavelength, warp, turbulence, density, seed, phase);
    }
    if (overlayType == 6u) {
        return RippleCurrentThreadsValue(uv, wavelength, warp, turbulence, density, seed, phase);
    }
    if (overlayType == 7u) {
        return RippleDropletValue(regionUv, normal, wavelength, warp, turbulence, density, seed, phase);
    }
    if (overlayType == 8u) {
        return RippleDripTrailValue(uv, wavelength, warp, turbulence, density, seed, phase);
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
    const float edge = smoothstep(0.0, max(1e-5, params.region1.w), edgeDistance);
    const float strength = clamp(params.region0.w, 0.0, 1.0);
    if (strength <= 1e-5 || edge <= 1e-5) {
        return contribution;
    }

    const float patternScale = clamp(params.pattern0.x, 0.05, 100.0);
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
        membership.data.y +
        float(params.control.z) * 0.013 +
        float(params.control.w) * 0.017;
    const float value = RipplePatternValue(
        params.control.x,
        uv,
        regionUv,
        normal,
        edge,
        wavelength,
        warp,
        turbulence,
        density,
        seed,
        phase);
    const float scale = clamp(value * intensity * strength * edge, 0.0, 1.0);
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

SparseRippleComposite ResolveSparseRippleComposite(vec3 worldPosition, vec3 pointNormal, uint pointIndex, float timeSeconds) {
    SparseRippleComposite result = EmptySparseRippleComposite();
    if (!HasSparseRippleEffects() ||
        styleData.pointMeta.x == 0u ||
        pointIndex >= styleData.pointMeta.x) {
        return result;
    }
    const SparseRippleRange pointRange = sparseRippleRangeData.sparseRippleRanges[pointIndex];
    const uint start = pointRange.range.x;
    const uint count = pointRange.range.y;
    if (count == 0u || start >= styleData.rippleEffectSlots3.y) {
        return result;
    }
    const uint cappedEnd = min(start + count, styleData.rippleEffectSlots3.y);
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
    result.opacityMultiply = max(0.0, result.opacityMultiply);
    result.pointSizeMultiply = max(0.0, result.pointSizeMultiply);
    result.colourMix = clamp(result.colourMix, 0.0, 1.0);
    return result;
}

vec3 ApplySparseRippleColor(vec3 baseColor, SparseRippleComposite ripple) {
    if (ripple.colourMix <= 1e-5) {
        return baseColor;
    }
    return mix(baseColor, ripple.colour, clamp(ripple.colourMix, 0.0, 1.0));
}
