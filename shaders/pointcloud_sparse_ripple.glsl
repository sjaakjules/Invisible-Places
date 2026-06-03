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

float RippleTideBandsValue(vec2 uv, float wavelength, float warp, float turbulence, float seed, float phase) {
    const float travelDistance = max(wavelength, 0.015);
    const float t = -phase;
    const float tidePhase = t * 0.082 * kRippleTwoPi;
    const float cycle = 0.5 - 0.5 * cos(tidePhase);
    const float motion = sin(tidePhase);
    const float motionDirection = motion >= 0.0 ? 1.0 : -1.0;
    const float travel = (cycle * 2.0 - 1.0) * travelDistance;
    const float lateralScale = max(wavelength * 1.35, 0.012);
    const float scallopNoise = RippleSmoothBlockNoise(
        vec2(uv.y, t * 0.035 + seed * 0.13),
        max(wavelength * 0.48, 0.008),
        seed,
        151.0);
    const float frontWarp =
        (sin((uv.y / lateralScale) + seed * 1.17 + t * 0.028) * 0.62 +
         sin((uv.y / max(wavelength * 0.58, 0.006)) - seed * 0.73 + t * 0.019) * 0.28 +
         (scallopNoise - 0.5) * 1.15) *
        wavelength * (0.08 + clamp(warp, 0.0, 8.0) * 0.10 + turbulence * 0.06);
    const float front = uv.x - travel - frontWarp;
    const float frontWidth = max(wavelength * (0.050 + turbulence * 0.028), 0.003);
    const float crest = RippleLine(front, frontWidth);
    const float trailDistance = max(0.0, -front * motionDirection);
    const float trailLength = max(wavelength * (0.42 + turbulence * 0.34), frontWidth * 5.0);
    const float trailingFoam =
        exp(-trailDistance / max(trailLength, 1.0e-4)) *
        smoothstep(frontWidth * 0.40, frontWidth * 1.80, trailDistance) *
        (1.0 - smoothstep(trailLength, trailLength * 1.65, trailDistance));
    const float shoreBreakup = smoothstep(0.24, 0.92, scallopNoise + turbulence * 0.22);
    const float calmMotion = 0.62 + 0.28 * abs(motion);
    return clamp((crest * (0.72 + shoreBreakup * 0.28) + trailingFoam * shoreBreakup * 0.32) * calmMotion, 0.0, 1.0);
}

float RippleWetSheenValue(vec2 uv, vec3 normal, float wavelength, float warp, float turbulence, float density, float seed, float phase) {
    const float slope = clamp(1.0 - abs(normal.z), 0.0, 1.0);
    const float normalGrain = clamp(length(normal.xy), 0.0, 1.0);
    const float t = -phase;
    const float safeWavelength = max(wavelength, 0.005);
    const vec2 normalBias = normal.xy * safeWavelength * (0.44 + clamp(warp, 0.0, 2.0) * 0.52);
    const vec2 driftA = vec2(t * (0.034 + turbulence * 0.018), -t * (0.021 + clamp(warp, 0.0, 2.0) * 0.012));
    const vec2 driftB = vec2(-t * (0.018 + clamp(warp, 0.0, 2.0) * 0.016), t * (0.029 + turbulence * 0.020));
    const float warpWave =
        sin((uv.y / max(safeWavelength * 1.15, 0.010)) + seed * 0.021 + t * 0.31) *
        safeWavelength * (0.055 + clamp(warp, 0.0, 2.0) * 0.085 + turbulence * 0.035);
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
    const float normalLift = slope * (0.52 + clamp(warp, 0.0, 2.0) * 0.30) +
                             normalGrain * (0.12 + clamp(warp, 0.0, 2.0) * 0.08);
    const float coverage = smoothstep(
        0.22 - density * 0.16,
        0.94,
        sheenPatch * 0.58 + lowA * 0.12 + normalLift);
    const float grain = smoothstep(0.50 - turbulence * 0.20, 0.98, fine * 0.68 + micro * 0.32);
    const float shimmerWave = 0.50 + 0.50 * sin(
        (uv.x + uv.y * 0.41) / max(safeWavelength * 3.6, 0.020) +
        t * (0.36 + turbulence * 0.22) +
        lowB * kRippleTwoPi);
    const float glint = grain * (0.44 + shimmerWave * (0.36 + turbulence * 0.24));
    const float pulse = 0.86 + 0.14 * sin(t * 0.58 + lowA * kRippleTwoPi + normalGrain * 2.4);
    const float sheen =
        (0.07 + sheenPatch * 0.20 + slope * (0.66 + clamp(warp, 0.0, 2.0) * 0.10) +
         normalGrain * 0.10 + glint * (0.14 + turbulence * 0.32)) *
        coverage;
    return clamp(sheen * pulse, 0.0, 1.0);
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
    vec2 flowDir = normal.xy;
    if (dot(flowDir, flowDir) <= 1.0e-6) {
        flowDir = vec2(1.0, 0.0);
    } else {
        flowDir = normalize(flowDir);
    }
    const float directionBlend = clamp(length(normal.xy) * 1.35 + warp * 0.08, 0.0, 1.0);
    flowDir = normalize(mix(vec2(1.0, 0.0), flowDir, directionBlend));
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
    const float shimmer = 0.58 + 0.42 * RippleWavePeak(
                                         alongVein + t * (0.18 + turbulence * 0.12) + veinSeedA * 1.37 + veinSeedB * 0.71,
                                         2.4);
    const float crystal = RippleSmoothBlockNoise(
        mineralUv + normalFlow * t * 0.010 + mineralAcross * t * 0.006,
        max(wavelength * 0.18, 0.004),
        seed,
        193.0);
    const float veinCoverage = smoothstep(
        0.50 - density * 0.24 - normalBias * 0.14,
        0.98,
        veinNetwork + coarse * 0.20);
    const float brightSplit = 0.72 + 0.28 * RippleWavePeak(splitPhase + crystal * 0.35, 1.8);
    const float fineGlint = smoothstep(0.76 - turbulence * 0.16 - density * 0.10, 1.0, crystal + veinNetwork * 0.20);
    return clamp(veinCoverage * veinNetwork * (0.34 + shimmer * 0.48 + fineGlint * 0.22) * brightSplit * (0.72 + normalBias * 0.38), 0.0, 1.0);
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
    vec2 streamUv = uv;
    streamUv.x += normalBias * wavelength * (0.10 + warp * 0.06);
    streamUv.y += sin(uv.x / max(wavelength * 1.25, 0.010) + seed * 1.17 + t * 0.13) *
                  wavelength * (0.055 + warp * 0.060 + normalBias * 0.035);
    streamUv.y += sin(uv.x / max(wavelength * 0.47, 0.006) - seed * 0.73 - t * 0.19) *
                  wavelength * turbulence * 0.055;
    const vec2 p = vec2(streamUv.x / cellXSize, streamUv.y / cellYSize);
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
            const vec2 local = streamUv - origin;
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
                streamUv + vec2(t * 0.018 + pulseSeed, -t * 0.011),
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
            const float pulse = (trunk + headDrop + branch * 0.68) *
                                    breakup *
                                    (0.62 + originGate * 0.22 + normalBias * 0.22) +
                                pulseCore * (0.08 + density * 0.10);
            best = max(best, pulse);
        }
    }
    const float fallbackNoise = RippleSmoothBlockNoise(
        streamUv + vec2(t * 0.014, -t * 0.009),
        max(wavelength * 0.42, 0.006),
        seed,
        239.0);
    const float fallbackPulse =
        RippleWavePeak(
            streamUv.x / max(wavelength * 1.9, 0.012) - t * 0.22 + fallbackNoise,
            2.1) *
        smoothstep(0.42 - density * 0.16, 0.96, fallbackNoise);
    return clamp(max(best, fallbackPulse * 0.22) * (0.78 + normalBias * 0.30), 0.0, 1.0);
}

float RippleFoamSparkleValue(vec2 regionUv, float edge, float wavelength, float warp, float turbulence, float density, float seed, float phase) {
    const float t = -phase;
    const vec2 foamUv = regionUv + vec2(t * 0.018, -t * 0.011) * (0.4 + warp);
    const float patchCellSize = max(wavelength * (1.10 + density * 0.72), 0.018);
    const vec2 p = foamUv / patchCellSize;
    const int baseX = int(floor(p.x));
    const int baseY = int(floor(p.y));
    const float warpAmount = clamp(warp, 0.0, 8.0);
    float nearestPatch = 1.0e20;
    float patchSeed = 0.0;
    for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
            const int cx = baseX + dx;
            const int cy = baseY + dy;
            const vec2 h = RippleCellHash2(cx, cy, seed, 131.0);
            const float angle = (h.x * 1.91 + h.y * 2.37 + t * (0.020 + h.x * 0.025)) * kRippleTwoPi;
            const vec2 drift = vec2(cos(angle), sin(angle * 1.07 + h.y * kRippleTwoPi)) *
                               (0.04 + warpAmount * 0.035);
            const float radius = 0.34 + h.y * 0.24 + density * 0.12;
            const vec2 center = vec2(float(cx), float(cy)) + h + drift;
            const float patchDistance = length(p - center) / max(radius, 0.05);
            if (patchDistance < nearestPatch) {
                nearestPatch = patchDistance;
                patchSeed = RippleCellHash(cx, cy, seed, 137.0);
            }
        }
    }
    const float cellularPatch = 1.0 - smoothstep(0.62, 1.16 + turbulence * 0.20, nearestPatch);
    const float foamNoise = RippleSmoothBlockNoise(
        foamUv + vec2(t * 0.010, t * 0.007),
        max(wavelength * 0.62, 0.010),
        seed,
        109.0);
    const float fineFleck = RippleBlockNoise(
        foamUv + vec2(foamNoise, -foamNoise) * wavelength * 0.36,
        max(wavelength * 0.18, 0.004),
        seed,
        107.0);
    const float patchKeep = smoothstep(
        0.34 - density * 0.18,
        0.92,
        cellularPatch * 0.72 + foamNoise * 0.44);
    const float sparkle = smoothstep(
        0.68 - density * 0.26 - turbulence * 0.16,
        1.0,
        fineFleck + cellularPatch * 0.22);
    const float pulse = 0.64 + 0.36 * RippleWavePeak(patchSeed + fineFleck + t * (0.17 + turbulence * 0.20), 2.4);
    const float edgeFade = smoothstep(0.05, 0.34, edge);
    const float foam = patchKeep * (0.54 + cellularPatch * 0.46) + sparkle * 0.22;
    return clamp(edgeFade * foam * pulse, 0.0, 1.0);
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
    const bool causticLace = params.control.x == 0u;
    if (strength <= 1e-5 || (!causticLace && edge <= 1e-5)) {
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
        float(params.control.z) * 0.013 +
        float(params.control.w) * 0.017 +
        RippleOverlaySalt(params.control.x) * 0.011;
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
