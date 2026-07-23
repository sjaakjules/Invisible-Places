#ifndef INVISIBLE_PLACES_RAIN_IMPACT_ROCK_MODEL_GLSL
#define INVISIBLE_PLACES_RAIN_IMPACT_ROCK_MODEL_GLSL

// Pure ROCK evaluator shared by every point-cloud shader and the deterministic
// Vulkan equivalence harness. Keeping grid lookup and material composition out
// of this function makes CPU/GPU comparisons independent of broad-phase order.
uint RockRainHashBits(uint value) {
    value ^= value >> 16u;
    value *= 0x7feb352du;
    value ^= value >> 15u;
    value *= 0x846ca68bu;
    return value ^ (value >> 16u);
}

float RockRainRandom01(uint value) {
    return float(RockRainHashBits(value) & 0x00ffffffu) / float(0x01000000u);
}

float EvaluateRockRainImpactValue(
    vec3 eventPosition,
    vec3 eventNormalCandidate,
    float eventRadius,
    float lifetime,
    uint eventSeed,
    vec4 rockImpact0,
    vec4 rockImpact1,
    vec3 point,
    vec3 pointNormal,
    float age) {
    if (age < 0.0 || age > lifetime) {
        return 0.0;
    }

    vec3 eventNormal = eventNormalCandidate;
    if (dot(eventNormal, eventNormal) > 1e-8) {
        eventNormal = normalize(eventNormal);
    } else if (dot(pointNormal, pointNormal) > 1e-8) {
        eventNormal = normalize(pointNormal);
    } else {
        eventNormal = vec3(0.0, 0.0, 1.0);
    }

    const float safeLifetime = max(0.001, lifetime);
    const float life = clamp(age / safeLifetime, 0.0, 1.0);
    const float effectiveRadius = max(0.001, eventRadius * sqrt(2.0 / 3.0));
    const float growthSeconds =
        clamp(lifetime * 0.18, 0.55, 0.95) *
        (3.20 / clamp(rockImpact0.y, 0.10, 6.0));
    const float growth = smoothstep(0.0, growthSeconds, age);

    const vec3 gravity = vec3(0.0, 0.0, -1.0);
    const vec3 projectedGravity = gravity - eventNormal * dot(gravity, eventNormal);
    const float projectedGravityLengthSquared = dot(projectedGravity, projectedGravity);
    const vec3 downhill = projectedGravityLengthSquared > 1e-8
        ? projectedGravity * inversesqrt(projectedGravityLengthSquared)
        : vec3(0.0);
    const float driftStartLife = clamp(growthSeconds / safeLifetime, 0.0, 0.9);
    const float drift = driftStartLife < 0.9
        ? smoothstep(driftStartLife, 0.9, life)
        : 0.0;
    const vec3 impactCentre = eventPosition + downhill * (effectiveRadius * 0.2 * drift);

    const vec3 delta = point - impactCentre;
    const float normalDistance = abs(dot(delta, eventNormal));
    const vec3 tangent = delta - eventNormal * dot(delta, eventNormal);
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
    const float angle = atan(dot(tangent, tangentY), dot(tangent, tangentX));
    const float breakupPhase = RockRainRandom01(eventSeed ^ 0xd1b54a35u) * 2.0 * 3.14159265359;
    const float breakupNoise = clamp(
        0.5 + 0.31 * sin(angle * 5.0 + breakupPhase) +
            0.19 * sin(angle * 9.0 - breakupPhase * 1.7),
        0.0,
        1.0);
    // Breakup is inward-only so the original event radius remains the exact
    // broad phase used by the fixed-capacity impact grid.
    const float irregularRadiusScale = 1.0 -
        0.35 * clamp(rockImpact0.x, 0.0, 1.0) * breakupNoise;
    const float normalizedDistance = (
        sqrt(dot(tangent, tangent) + normalDistance * normalDistance * 4.0) /
        effectiveRadius) / max(0.65, irregularRadiusScale);
    // Retain the original broad-phase cells: once downhill drift begins, the
    // late feather plus the 20% centre travel still fits inside eventRadius.
    const float edgeWidth = 0.02 + (1.0 - growth) * 0.14;
    const float lowerPointWeight = smoothstep(
        -0.45,
        0.45,
        (eventPosition.z - point.z) / effectiveRadius);
    const float heightBias = clamp(rockImpact0.w * (0.20 / 0.75), 0.0, 0.55);
    const float heightGain = mix(1.0 - heightBias, 1.0 + heightBias, lowerPointWeight);
    const float fadeDifference = clamp(heightBias * 0.75, 0.0, 0.30);
    const float fadeStart = clamp(
        mix(0.55 - fadeDifference, 0.55 + fadeDifference, lowerPointWeight) *
            clamp(rockImpact1.x / 1.35, 0.10, 2.25),
        0.05,
        0.95);
    const float footprint = 1.0 - smoothstep(
        max(0.0, growth - edgeWidth),
        growth + edgeWidth,
        normalizedDistance);
    const float centreWeight = mix(
        1.0,
        sqrt(clamp(1.0 - normalizedDistance, 0.0, 1.0)),
        clamp(rockImpact0.z, 0.0, 1.0));
    return footprint *
           (1.0 - smoothstep(fadeStart, 1.0, life)) *
           heightGain * centreWeight;
}

#endif
