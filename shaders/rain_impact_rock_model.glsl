#ifndef INVISIBLE_PLACES_RAIN_IMPACT_ROCK_MODEL_GLSL
#define INVISIBLE_PLACES_RAIN_IMPACT_ROCK_MODEL_GLSL

// Pure ROCK evaluator shared by every point-cloud shader and the deterministic
// Vulkan equivalence harness. Keeping grid lookup and material composition out
// of this function makes CPU/GPU comparisons independent of broad-phase order.
float EvaluateRockRainImpactValue(
    vec3 eventPosition,
    vec3 eventNormalCandidate,
    float eventRadius,
    float lifetime,
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
    const float growthSeconds = 2.0 * clamp(lifetime * 0.18, 0.55, 0.95);
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
    const float normalizedDistance =
        sqrt(dot(tangent, tangent) + normalDistance * normalDistance * 4.0) /
        effectiveRadius;
    // Retain the original broad-phase cells: once downhill drift begins, the
    // late feather plus the 20% centre travel still fits inside eventRadius.
    const float edgeWidth = 0.02 + (1.0 - growth) * 0.14;
    const float lowerPointWeight = smoothstep(
        -0.45,
        0.45,
        (eventPosition.z - point.z) / effectiveRadius);
    const float heightGain = mix(0.8, 1.2, lowerPointWeight);
    const float fadeStart = mix(0.4, 0.7, lowerPointWeight);
    return (1.0 - smoothstep(
                max(0.0, growth - edgeWidth),
                growth + edgeWidth,
                normalizedDistance)) *
           (1.0 - smoothstep(fadeStart, 1.0, life)) *
           heightGain;
}

#endif
