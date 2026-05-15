const uint kPointStylisationOff = 0u;
const uint kPointStylisationNpr = 1u;
const uint kPointStylisationBrush = 2u;
const uint kPointNprWatercolor = 0u;
const uint kPointNprCartoon = 1u;

bool PointStylisationActive() {
    return styleData.stylisationControl.x != kPointStylisationOff &&
           styleData.stylisationParams0.x > 1e-5;
}

float PointHash01(uint value) {
    value ^= value >> 16u;
    value *= 0x7feb352du;
    value ^= value >> 15u;
    value *= 0x846ca68bu;
    value ^= value >> 16u;
    return float(value & 0x00ffffffu) / 16777215.0;
}

float PointCoordNoise(vec2 coord, uint pointIndex) {
    const vec2 shifted = clamp(coord, vec2(-2.0), vec2(2.0)) + vec2(2.0);
    const uvec2 cell = uvec2(floor(shifted * 31.0));
    return PointHash01((cell.x * 1973u) ^ (cell.y * 9277u) ^ (pointIndex * 26699u));
}

float PointTemporalPigmentNoise(uint pointIndex, uint salt) {
    const float speed = clamp(styleData.stylisationParams2.z, 0.0, 4.0);
    if (speed <= 1e-5) {
        return PointHash01((pointIndex * 747796405u) ^ salt);
    }

    const float temporal = max(0.0, uniforms.depthParameters.x) * speed * 12.0;
    const uint frame = uint(floor(temporal));
    const float blend = smoothstep(0.0, 1.0, fract(temporal));
    const float current = PointHash01((pointIndex * 747796405u) ^ (frame * 2891336453u) ^ salt);
    const float next = PointHash01((pointIndex * 747796405u) ^ ((frame + 1u) * 2891336453u) ^ salt);
    return mix(current, next, blend);
}

float PointWatercolorGranulationMask(vec3 color, float surfaceAngleMask) {
    const float luma = dot(clamp(color, 0.0, 1.0), vec3(0.299, 0.587, 0.114));
    const float luminanceMask = 1.0 - smoothstep(0.18, 0.92, luma);
    const float angleStrength = clamp(styleData.stylisationParams2.w, 0.0, 1.0);
    const float grazingMask = 0.35 + (0.65 * smoothstep(0.05, 0.85, clamp(surfaceAngleMask, 0.0, 1.0)));
    return clamp((0.25 + (0.75 * luminanceMask)) * mix(1.0, grazingMask, angleStrength), 0.0, 1.0);
}

vec2 PointBrushCoord(vec2 coord, uint pointIndex) {
    const float jitter = clamp(styleData.stylisationParams1.z, 0.0, 1.0);
    const vec2 jitterOffset = vec2(
        PointHash01(pointIndex * 1664525u + 1013904223u),
        PointHash01(pointIndex * 22695477u + 1u)) - vec2(0.5);
    coord -= jitterOffset * (jitter * 0.38);

    const float angle = PointHash01(pointIndex * 747796405u + 2891336453u) * 6.28318530718;
    const float c = cos(angle);
    const float s = sin(angle);
    return vec2((c * coord.x) - (s * coord.y), (s * coord.x) + (c * coord.y));
}

float PointBrushRadius(vec2 brushCoord) {
    const float aspect = max(0.25, styleData.stylisationParams1.y);
    const vec2 ellipse = vec2(brushCoord.x / aspect, brushCoord.y * aspect);
    return length(ellipse);
}

float PointStylisationCoverage(vec2 coord, float radius, float radiusSquared, uint pointIndex) {
    if (!PointStylisationActive()) {
        return 1.0;
    }

    const float strength = clamp(styleData.stylisationParams0.x, 0.0, 1.0);
    const float bleed = clamp(styleData.stylisationParams1.x, 0.0, 1.0);
    const float grainAmount = clamp(styleData.stylisationParams0.w, 0.0, 1.0);
    const float grain = mix(
        PointCoordNoise(coord, pointIndex),
        PointTemporalPigmentNoise(pointIndex, 0x9e3779b9u),
        clamp(styleData.stylisationParams2.y, 0.0, 1.0) * 0.45);
    float coverage = 1.0;

    if (styleData.stylisationControl.x == kPointStylisationNpr) {
        if (styleData.stylisationControl.y == kPointNprWatercolor) {
            const float edgeDryness = smoothstep(0.58, 1.0, radius) * bleed;
            const float pigmentGap = clamp(0.78 + (0.44 * grain), 0.0, 1.25);
            coverage *= mix(1.0, 1.0 - (edgeDryness * pigmentGap * 0.55), strength);
        }
        return clamp(coverage, 0.0, 1.0);
    }

    const vec2 brushCoord = PointBrushCoord(coord, pointIndex);
    const float brushRadius = PointBrushRadius(brushCoord);
    if (brushRadius > 1.0) {
        return 0.0;
    }

    const float edgeWidth = max(0.04, 0.48 * bleed);
    coverage *= smoothstep(1.0, 1.0 - edgeWidth, brushRadius);
    const float brushGrain = mix(
        PointCoordNoise(brushCoord, pointIndex + 17u),
        PointTemporalPigmentNoise(pointIndex, 0x85ebca6bu),
        clamp(styleData.stylisationParams2.y, 0.0, 1.0) * 0.55);
    coverage *= mix(1.0, 0.68 + (0.64 * brushGrain), grainAmount * strength);
    coverage *= 1.0 - (clamp(styleData.stylisationParams2.x, 0.0, 1.0) *
                        PointTemporalPigmentNoise(pointIndex, 0xc2b2ae35u) * 0.55 * strength);
    return clamp(coverage, 0.0, 1.0);
}

vec3 PointQuantizeColor(vec3 color) {
    const float levels = max(2.0, floor(styleData.stylisationParams0.y + 0.5));
    return floor(clamp(color, 0.0, 1.0) * levels + vec3(0.5)) / levels;
}

vec3 PointApplyHatch(vec3 color, vec2 coord, uint pointIndex, float strength) {
    const float hatchStrength = clamp(styleData.stylisationParams1.w, 0.0, 1.0) * strength;
    if (hatchStrength <= 1e-5) {
        return color;
    }

    const float phase = PointHash01(pointIndex * 374761393u + 668265263u);
    const float stripe = abs(fract(((coord.x + coord.y) * 8.0) + phase) - 0.5);
    const float line = 1.0 - smoothstep(0.035, 0.12, stripe);
    return color * (1.0 - (line * hatchStrength * 0.55));
}

vec3 PointStyliseWatercolor(
    vec3 color,
    vec2 coord,
    uint pointIndex,
    float surfaceAngleMask,
    float strength) {
    const float luma = dot(color, vec3(0.299, 0.587, 0.114));
    vec3 stylised = mix(vec3(luma), color, 0.72);
    stylised = mix(stylised, vec3(1.0), 0.08);
    const float variation = clamp(styleData.stylisationParams2.y, 0.0, 1.0);
    const float granulationMask = PointWatercolorGranulationMask(color, surfaceAngleMask);
    const float spatialGrain = PointCoordNoise(coord, pointIndex);
    const float temporalGrain = PointTemporalPigmentNoise(pointIndex, 0x27d4eb2du);
    const float grain = mix(spatialGrain, temporalGrain, variation);
    const float paperGrain = clamp(styleData.stylisationParams0.w, 0.0, 1.0) * granulationMask;
    const float pigmentShift = ((temporalGrain - 0.5) * 2.0) * variation * granulationMask;
    stylised *= 1.0 + (pigmentShift * 0.18);
    stylised *= mix(1.0, 0.80 + (0.42 * grain), paperGrain);
    return mix(color, clamp(stylised, 0.0, 1.0), strength);
}

vec3 PointStyliseCartoon(vec3 color, vec2 coord, float radius, uint pointIndex, float strength) {
    vec3 stylised = PointQuantizeColor(color);
    const float ink = smoothstep(0.58, 1.0, radius) * clamp(styleData.stylisationParams0.z, 0.0, 1.0);
    stylised *= 1.0 - (ink * 0.78);
    stylised = PointApplyHatch(stylised, coord, pointIndex, strength);
    return mix(color, clamp(stylised, 0.0, 1.0), strength);
}

vec3 PointStylisationColor(vec3 color, vec2 coord, float radius, uint pointIndex, float surfaceAngleMask) {
    if (!PointStylisationActive()) {
        return color;
    }

    const float strength = clamp(styleData.stylisationParams0.x, 0.0, 1.0);
    vec2 styleCoord = coord;
    float styleRadius = radius;
    if (styleData.stylisationControl.x == kPointStylisationBrush) {
        styleCoord = PointBrushCoord(coord, pointIndex);
        styleRadius = PointBrushRadius(styleCoord);
    }

    vec3 stylised = color;
    if (styleData.stylisationControl.y == kPointNprCartoon) {
        stylised = PointStyliseCartoon(color, styleCoord, styleRadius, pointIndex, strength);
    } else {
        stylised = PointStyliseWatercolor(color, styleCoord, pointIndex, surfaceAngleMask, strength);
    }
    stylised = PointApplyHatch(stylised, styleCoord, pointIndex, strength);
    return clamp(stylised, 0.0, 1.0);
}
