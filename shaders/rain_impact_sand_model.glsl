#ifndef INVISIBLE_PLACES_RAIN_IMPACT_SAND_MODEL_GLSL
#define INVISIBLE_PLACES_RAIN_IMPACT_SAND_MODEL_GLSL

// Pure SAND evaluator shared by the viewport shaders and the deterministic
// CPU/offline path. waterMask is one on the flooded side of the authored
// shoreline and zero on dry, uphill sand.
float EvaluateSandRainImpactValue(
    vec3 eventPosition,
    float eventRadius,
    float lifetime,
    vec3 point,
    float age,
    float waterMask,
    float thicknessScale) {
    if (age < 0.0 || age > lifetime) {
        return 0.0;
    }

    const float safeLifetime = max(0.001, lifetime);
    const float life = clamp(age / safeLifetime, 0.0, 1.0);
    const float planarDistance = length(point.xy - eventPosition.xy);

    // Flooded sand keeps the expanding ripple, but its response now rolls off
    // continuously from the impact centre instead of ending as a flat band.
    const float wetRadius = max(0.001, eventRadius);
    const float wetNormalizedDistance = planarDistance / wetRadius;
    const float ringRadius = wetRadius * (0.12 + 0.88 * life);
    const float ringThickness = max(
        0.00075,
        max(0.003, wetRadius * 0.14) * clamp(thicknessScale, 0.25, 2.0));
    const float ringDistance = abs(planarDistance - ringRadius);
    const float wetRadialFade =
        1.0 - smoothstep(0.0, 1.0, wetNormalizedDistance);
    const float wetValue =
        exp(-(ringDistance * ringDistance) /
            (ringThickness * ringThickness)) *
        (1.0 - smoothstep(0.72, 1.0, life)) *
        sqrt(max(0.0, wetRadialFade));

    // Dry sand receives a compact filled splash. The 5--25 mm radius produces
    // the requested 0.01--0.05 m diameter without changing the event broad
    // phase used by the fixed-capacity grid.
    const float dryRadius = clamp(eventRadius * 0.22, 0.005, 0.025);
    const float dryNormalizedDistance = planarDistance / dryRadius;
    const float dryGrowthSeconds = min(0.14, safeLifetime * 0.28);
    const float dryGrowth = smoothstep(0.0, dryGrowthSeconds, age);
    const float dryEdgeWidth = 0.10 + (1.0 - dryGrowth) * 0.18;
    const float dryFootprint = 1.0 - smoothstep(
        max(0.0, dryGrowth - dryEdgeWidth),
        dryGrowth + dryEdgeWidth,
        dryNormalizedDistance);
    const float dryRadialFade =
        1.0 - smoothstep(0.0, 1.0, dryNormalizedDistance);
    const float lowerPointWeight = smoothstep(
        -0.35,
        0.35,
        (eventPosition.z - point.z) / dryRadius);
    const float dryHeightGain = mix(0.78, 1.16, lowerPointWeight);
    const float dryValue =
        dryFootprint *
        dryRadialFade *
        dryHeightGain *
        (1.0 - smoothstep(0.55, 1.0, life));

    return mix(dryValue, wetValue, clamp(waterMask, 0.0, 1.0));
}

#endif
