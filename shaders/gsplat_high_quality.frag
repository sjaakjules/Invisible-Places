#version 450

layout(location = 0) in vec2 inUv;
layout(location = 1) in float inOpacity;
layout(location = 2) in vec3 inWorldCenter;
layout(location = 3) flat in uint inSplatIndex;
layout(location = 4) in float inViewDepth;
layout(location = 5) in float inScaleMetric;
layout(location = 6) flat in uint inLayerStyleIndex;

layout(location = 0) out vec4 outColor;

layout(set = 0, binding = 0) uniform FrameUniforms {
    mat4 viewProjection;
    mat4 view;
    mat4 projection;
    vec4 cameraPosition;
    vec4 depthParameters;
    vec4 viewportParameters;
} uniforms;

layout(set = 0, binding = 5, std430) readonly buffer ShCoefficients {
    float values[];
} shCoefficients;

struct LayerStyleGpu {
    mat4 localToWorld;
    vec4 layerTint;
    vec4 style;
    uvec4 control;
};

layout(set = 0, binding = 7, std430) readonly buffer LayerStyles {
    LayerStyleGpu values[];
} layerStyles;

layout(push_constant) uniform HighQualityGaussianStyle {
    vec4 extra;
} pushConstants;

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
    const LayerStyleGpu layerStyle = layerStyles.values[inLayerStyleIndex];
    const float sigmaRadius = 3.0;
    const float radiusSquared = dot(inUv, inUv);
    if (radiusSquared > (sigmaRadius * sigmaRadius)) {
        discard;
    }

    const float gaussian = exp(-0.5 * radiusSquared);
    const float alpha = clamp(inOpacity * layerStyle.style.x * gaussian, 0.0, 0.995);
    if (alpha <= 1e-4) {
        discard;
    }

    const vec3 viewDirection = normalize(inWorldCenter - uniforms.cameraPosition.xyz);
    vec3 color = EvaluateFullSh(inSplatIndex, viewDirection);
    if (layerStyle.control.x == 1u) {
        color = max(vec3(0.0), vec3(0.5) + (kShC0 * LoadShCoefficient(inSplatIndex, 0u)));
    }

    color *= max(layerStyle.style.z, 0.0);
    color = ApplySaturation(color, max(layerStyle.style.w, 0.0));

    if (layerStyle.control.y == 1u) {
        color = vec3(alpha);
    } else if (layerStyle.control.y == 2u) {
        const float scaleDebug = clamp(log2(inScaleMetric + 1.0) / 6.0, 0.0, 1.0);
        color = mix(vec3(0.08, 0.12, 0.18), vec3(0.96, 0.62, 0.12), scaleDebug);
    } else if (layerStyle.control.y == 3u) {
        const float depthNorm = clamp(
            (inViewDepth - uniforms.depthParameters.y) /
            max(1e-5, uniforms.depthParameters.z - uniforms.depthParameters.y),
            0.0,
            1.0);
        color = vec3(1.0 - depthNorm, 0.55 * (1.0 - depthNorm), depthNorm);
    } else if (layerStyle.control.y == 4u) {
        color = layerStyle.layerTint.rgb;
    } else {
        color *= layerStyle.layerTint.rgb;
    }

    outColor = vec4(color * alpha, alpha);
}
