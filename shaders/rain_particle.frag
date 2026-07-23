#version 450

layout(location = 0) in vec2 inUv;
layout(location = 1) in vec4 inColourOpacity;
layout(location = 2) in float inEmission;
layout(location = 3) in float inViewDepth;
layout(location = 4) in float inSoftness;
layout(location = 5) in float inEllipseBlend;

layout(location = 0) out vec4 outAccumulation;
layout(location = 1) out float outRevealage;
layout(location = 2) out vec4 outEmission;

void main() {
    const float lateral = abs(inUv.x);
    const float featherStart = mix(0.90, 0.05, clamp(inSoftness, 0.0, 1.0));
    const float lateralSoftness = 1.0 - smoothstep(featherStart, 1.0, lateral);
    const float headFade = smoothstep(0.0, 0.08, inUv.y);
    const float tailFade = 1.0 - smoothstep(0.72, 1.0, inUv.y);
    const float streakCoverage = lateralSoftness * headFade * tailFade;
    const vec2 ellipseUv = vec2(inUv.x, inUv.y * 2.0 - 1.0);
    const float ellipseCoverage = 1.0 - smoothstep(
        featherStart,
        1.0,
        length(ellipseUv));
    const float coverage = mix(
        streakCoverage,
        ellipseCoverage,
        clamp(inEllipseBlend, 0.0, 1.0));
    const float alpha = clamp(inColourOpacity.a * coverage, 0.0, 0.995);
    if (alpha <= 1e-5) {
        discard;
    }
    const float depthWeight = clamp(1.0 / (0.02 + inViewDepth * inViewDepth * 0.002), 0.01, 64.0);
    const float weight = (min(1.0, alpha * 8.0) + 0.01) * depthWeight;
    outAccumulation = vec4(inColourOpacity.rgb * alpha * weight, alpha * weight);
    outRevealage = alpha;
    outEmission = vec4(inColourOpacity.rgb * alpha * max(0.0, inEmission), alpha * max(0.0, inEmission));
}
