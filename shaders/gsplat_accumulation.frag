#version 450

layout(location = 0) in vec2 inUv;
layout(location = 1) in float inOpacity;
layout(location = 2) in vec3 inWorldCenter;
layout(location = 3) flat in uint inSplatIndex;
layout(location = 4) in float inViewDepth;
layout(location = 5) in float inScaleMetric;

layout(location = 0) out vec4 outAccumulation;
layout(location = 1) out float outRevealage;
layout(location = 2) out vec4 outEmission;

layout(set = 0, binding = 0) uniform FrameUniforms {
    mat4 viewProjection;
    mat4 view;
    mat4 projection;
    vec4 cameraPosition;
    vec4 depthParameters;
    vec4 viewportParameters;
    vec4 depthOfFieldParameters;
} uniforms;

layout(set = 0, binding = 5, std430) readonly buffer ShCoefficients {
    float values[];
} shCoefficients;

layout(input_attachment_index = 0, set = 0, binding = 6) uniform subpassInput sceneDepthInput;

layout(push_constant) uniform GaussianSplatStyle {
    mat4 localToWorld;
    vec4 layerTint;
    vec4 style;
    uvec4 control;
    vec4 extra;
} styleData;

const float kShC0 = 0.28209479177387814;
const float kShC1 = 0.4886025119029199;
const float kShC2[5] = float[](
    1.0925484305920792,
    -1.0925484305920792,
    0.31539156525252005,
    -1.0925484305920792,
    0.5462742152960396
);
const float kShC3[7] = float[](
    -0.5900435899266435,
    2.890611442640554,
    -0.4570457994644658,
    0.3731763325901154,
    -0.4570457994644658,
    1.445305721320277,
    -0.5900435899266435
);

vec3 LoadShCoefficient(uint splatIndex, uint coefficientIndex) {
    const uint base = (splatIndex * 48u) + (coefficientIndex * 3u);
    return vec3(
        shCoefficients.values[base + 0u],
        shCoefficients.values[base + 1u],
        shCoefficients.values[base + 2u]
    );
}

vec3 EvaluateFullSh(uint splatIndex, vec3 direction) {
    vec3 result = kShC0 * LoadShCoefficient(splatIndex, 0u);

    const float x = direction.x;
    const float y = direction.y;
    const float z = direction.z;

    result += (-kShC1 * y) * LoadShCoefficient(splatIndex, 1u);
    result += ( kShC1 * z) * LoadShCoefficient(splatIndex, 2u);
    result += (-kShC1 * x) * LoadShCoefficient(splatIndex, 3u);

    result += (kShC2[0] * x * y) * LoadShCoefficient(splatIndex, 4u);
    result += (kShC2[1] * y * z) * LoadShCoefficient(splatIndex, 5u);
    result += (kShC2[2] * (2.0 * z * z - x * x - y * y)) * LoadShCoefficient(splatIndex, 6u);
    result += (kShC2[3] * x * z) * LoadShCoefficient(splatIndex, 7u);
    result += (kShC2[4] * (x * x - y * y)) * LoadShCoefficient(splatIndex, 8u);

    result += (kShC3[0] * y * (3.0 * x * x - y * y)) * LoadShCoefficient(splatIndex, 9u);
    result += (kShC3[1] * x * y * z) * LoadShCoefficient(splatIndex, 10u);
    result += (kShC3[2] * y * (4.0 * z * z - x * x - y * y)) * LoadShCoefficient(splatIndex, 11u);
    result += (kShC3[3] * z * (2.0 * z * z - 3.0 * x * x - 3.0 * y * y)) * LoadShCoefficient(splatIndex, 12u);
    result += (kShC3[4] * x * (4.0 * z * z - x * x - y * y)) * LoadShCoefficient(splatIndex, 13u);
    result += (kShC3[5] * z * (x * x - y * y)) * LoadShCoefficient(splatIndex, 14u);
    result += (kShC3[6] * x * (x * x - 3.0 * y * y)) * LoadShCoefficient(splatIndex, 15u);

    return max(result + vec3(0.5), vec3(0.0));
}

vec3 ApplySaturation(vec3 color, float saturation) {
    const float luminance = dot(color, vec3(0.2126, 0.7152, 0.0722));
    return mix(vec3(luminance), color, saturation);
}

void main() {
    const float sigmaRadius = 3.0;
    const float radiusSquared = dot(inUv, inUv);
    if (radiusSquared > (sigmaRadius * sigmaRadius)) {
        discard;
    }

    const float gaussian = exp(-0.5 * radiusSquared);
    float alpha = clamp(inOpacity * styleData.style.x * gaussian, 0.0, 0.995);
    if (alpha <= 1e-4) {
        discard;
    }

    const vec3 viewDirection = normalize(inWorldCenter - uniforms.cameraPosition.xyz);
    const uint weightedMode = styleData.control.w;
    const bool forceDcOnly = weightedMode == 1u;
    vec3 color = EvaluateFullSh(inSplatIndex, viewDirection);
    if (forceDcOnly || styleData.control.x == 1u) {
        color = max(vec3(0.0), vec3(0.5) + (kShC0 * LoadShCoefficient(inSplatIndex, 0u)));
    }

    color *= max(styleData.style.z, 0.0);
    color = ApplySaturation(color, max(styleData.style.w, 0.0));

    if (styleData.control.y == 1u) {
        color = vec3(alpha);
    } else if (styleData.control.y == 2u) {
        const float scaleDebug = clamp(log2(inScaleMetric + 1.0) / 6.0, 0.0, 1.0);
        color = mix(vec3(0.08, 0.12, 0.18), vec3(0.96, 0.62, 0.12), scaleDebug);
    } else if (styleData.control.y == 3u) {
        const float depthNorm = clamp(
            (inViewDepth - uniforms.depthParameters.y) /
            max(1e-5, uniforms.depthParameters.z - uniforms.depthParameters.y),
            0.0,
            1.0);
        color = vec3(1.0 - depthNorm, 0.55 * (1.0 - depthNorm), depthNorm);
    } else if (styleData.control.y == 4u) {
        color = styleData.layerTint.rgb;
    } else {
        color *= styleData.layerTint.rgb;
    }

    const float normalizedDepth = clamp(
        (inViewDepth - uniforms.depthParameters.y) /
        max(1e-5, uniforms.depthParameters.z - uniforms.depthParameters.y),
        0.0,
        1.0);
    const float opacityBase = min(1.0, alpha * 8.0) + 0.01;
    const float opacityWeight = opacityBase * opacityBase * opacityBase;
    const float frontBase = 1.0 - normalizedDepth;
    const float frontSquared = frontBase * frontBase;
    const float frontWeight = frontSquared * frontSquared;
    float weight = clamp(
        (opacityWeight * 0.5) + (opacityWeight * frontWeight * 128.0),
        1e-3,
        256.0);

    if (weightedMode == 2u) {
        const float pointDepth = subpassLoad(sceneDepthInput).r;
        if (pointDepth < 0.999999) {
            const float depthDelta = pointDepth - gl_FragCoord.z;
            const float frontGap = max(0.0, depthDelta);
            const float backGap = max(0.0, -depthDelta);
            const float surfaceAffinity = exp(-abs(depthDelta) * 380.0);
            const float frontFade = mix(1.0, 0.08, clamp(frontGap * 640.0, 0.0, 1.0));
            const float backFade = mix(1.0, 0.35, clamp(backGap * 260.0, 0.0, 1.0));
            alpha *= max(0.12, surfaceAffinity) * frontFade * backFade;
            if (alpha <= 1e-4) {
                discard;
            }
            weight *= mix(0.35, 2.6, surfaceAffinity) * frontFade * mix(1.0, 0.65, clamp(backGap * 180.0, 0.0, 1.0));
        }
    }

    outAccumulation = vec4(color * alpha * weight, alpha * weight);
    outRevealage = alpha;
    outEmission = vec4(0.0);
}
