#ifndef POINTCLOUD_CAUSTICS_GLSL
#define POINTCLOUD_CAUSTICS_GLSL

const float kCausticPi2 = 6.28318530718;

float CausticHash(vec2 p) {
    vec3 p3 = fract(vec3(p.xyx) * 0.1031);
    p3 += dot(p3, p3.yzx + 33.33);
    return fract((p3.x + p3.y) * p3.z);
}

vec2 CausticHash2(vec2 p) {
    return vec2(CausticHash(p), CausticHash(p + 19.19));
}

float CausticStableSeed(float regionOrSeed) {
    if (regionOrSeed >= 1.0) {
        return CausticHash(vec2(floor(regionOrSeed + 0.5), 401.0));
    }
    return 0.38196601125;
}

vec2 CausticSurfaceUv(vec3 worldPosition, vec3 normal) {
    if (dot(normal, normal) <= 1e-8) {
        return worldPosition.xy;
    }

    vec3 n = normalize(normal);
    if (n.z < 0.0) {
        n = -n;
    }

    const float steepness = smoothstep(0.18, 0.82, 1.0 - clamp(n.z, 0.0, 1.0));
    const vec2 slope = clamp(-n.xy / max(0.35, n.z), vec2(-1.75), vec2(1.75));
    return worldPosition.xy + slope * worldPosition.z * steepness * 0.32;
}

float CausticVoronoiRidge(vec2 metersUv, float seed, float time, float edge) {
    const float stableSeed = CausticStableSeed(seed);
    const float cellSize = max(0.005, styleData.causticParams0.y);
    const float lineWidth = max(0.0005, styleData.causticParams0.w);
    const float warpAmplitude = clamp(styleData.causticParams1.x, 0.0, 2.0);
    const float feather = max(0.0005, styleData.causticParams2.x);
    const float pointSpacing = max(0.0005, styleData.causticParams2.y);
    const float edgeCore = smoothstep(0.08, 0.82, clamp(edge, 0.0, 1.0));

    vec2 warpedMeters = metersUv;
    const vec2 warpUv = metersUv / max(0.001, cellSize * 1.7);
    warpedMeters += warpAmplitude * vec2(
        sin(warpUv.y * 1.73 + time * 0.71 + stableSeed * kCausticPi2) +
            0.35 * sin(warpUv.x * 3.19 - time * 0.23 + stableSeed * 5.31),
        cos(warpUv.x * 1.31 - time * 0.63 + stableSeed * 4.398229715) +
            0.35 * cos(warpUv.y * 2.17 + time * 0.29 + stableSeed * 7.13));

    const vec2 uv = warpedMeters / cellSize;
    const vec2 base = floor(uv);
    const vec2 f = fract(uv);
    float first = 16.0;
    float second = 16.0;
    for (int y = -1; y <= 1; ++y) {
        for (int x = -1; x <= 1; ++x) {
            const vec2 cell = vec2(float(x), float(y));
            vec2 feature = CausticHash2(base + cell + stableSeed * 37.0);
            feature = 0.5 + 0.5 * sin((feature + time * vec2(0.17, -0.13) + stableSeed) * kCausticPi2);
            const float distance = length(cell + feature - f);
            if (distance < first) {
                second = first;
                first = distance;
            } else if (distance < second) {
                second = distance;
            }
        }
    }

    const float ridgeDistanceMeters = max(0.0, second - first) * cellSize;
    const float edgeWidthScale = mix(0.55, 1.0, edgeCore);
    const float resolvedWidth = max(lineWidth, pointSpacing * 1.15) * edgeWidthScale;
    const float resolvedFeather = max(feather, pointSpacing * 0.5) * mix(0.75, 1.0, edgeCore);
    const float core = 1.0 - smoothstep(resolvedWidth, resolvedWidth + resolvedFeather, ridgeDistanceMeters);
    const float halo = 1.0 - smoothstep(
        resolvedWidth + resolvedFeather * 2.0,
        resolvedWidth + resolvedFeather * 7.0,
        ridgeDistanceMeters);
    const float dissolveNoise = CausticHash(
        floor((metersUv / max(0.001, cellSize * 2.0)) + stableSeed * 23.0));
    const float dissolve = smoothstep(0.04, 0.74, edgeCore + (dissolveNoise - 0.5) * 0.18);
    const float shimmer = 0.82 + 0.18 * sin(time * 1.7 + stableSeed * 17.0 + first * 11.0);
    return clamp((core * 1.12 + halo * 0.32) * shimmer * dissolve, 0.0, 1.0);
}

float CausticEdgeGate(vec2 metersUv, float edge, float seed) {
    const float stableSeed = CausticStableSeed(seed);
    const float cellSize = max(0.005, styleData.causticParams0.y);
    const float broadBreakup = CausticHash(floor((metersUv / max(0.001, cellSize * 2.5)) + stableSeed * 11.0));
    return smoothstep(0.04, 0.92, edge + broadBreakup * 0.14 - 0.06);
}

float CausticPreviewTint(float mask, float edge, float regionOrSeed) {
    const float amount = clamp(styleData.causticParams2.z, 0.0, 1.0);
    if (amount <= 1e-5) {
        return 0.0;
    }
    const float targetRegion = styleData.causticParams2.w;
    if (targetRegion >= 0.5 && abs(floor(regionOrSeed + 0.5) - floor(targetRegion + 0.5)) > 0.5) {
        return 0.0;
    }
    return clamp(mask * edge * amount, 0.0, 1.0);
}

float CausticColorMixAmount(float causticStrength, float previewTint) {
    return clamp(causticStrength * 0.55 + previewTint * 0.24, 0.0, 1.0);
}

float CausticColorSignal(float causticStrength, float previewTint) {
    return causticStrength + previewTint * (0.24 / 0.55);
}

#endif
