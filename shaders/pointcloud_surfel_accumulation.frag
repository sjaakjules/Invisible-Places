#version 450
#extension GL_GOOGLE_include_directive : require
#include "pointcloud_colormaps.glsl"

layout(location = 0) in vec4 inSourceColor;
layout(location = 1) in float inColormapValue;
layout(location = 2) in float inOpacity;
layout(location = 3) in float inEmissive;
layout(location = 4) in float inXray;
layout(location = 5) in float inDepthFade;
layout(location = 6) in float inViewDepth;
layout(location = 7) in vec2 inDiscCoord;

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

struct RenderParameterBindingGpu {
    vec4 constantValue;
    vec4 range;
    vec4 extra;
    uvec4 control;
};

layout(set = 0, binding = 2, std140) uniform PointStyleData {
    vec4 solidColor;
    uvec4 globalControl;
    uvec4 pointMeta;
    uvec4 renderControl;
    vec4 renderParams0;
    vec4 renderParams1;
    vec4 renderParams2;
    RenderParameterBindingGpu pointSizeBinding;
    RenderParameterBindingGpu opacityBinding;
    RenderParameterBindingGpu emissiveBinding;
    RenderParameterBindingGpu xrayBinding;
    RenderParameterBindingGpu depthFadeBinding;
    RenderParameterBindingGpu colormapPositionBinding;
    RenderParameterBindingGpu surfelDiameterBinding;
} styleData;

layout(input_attachment_index = 0, set = 0, binding = 3) uniform subpassInput sceneDepthInput;

vec3 ResolveBaseColor() {
    vec3 baseColor = inSourceColor.rgb;
    if (styleData.globalControl.x == 1u) {
        baseColor = styleData.solidColor.rgb;
    } else if (styleData.globalControl.x == 2u) {
        baseColor = ApplyPointCloudColormap(styleData.globalControl.y, clamp(inColormapValue, 0.0, 1.0));
    } else if (styleData.globalControl.w == 0u) {
        baseColor = styleData.solidColor.rgb;
    }
    return baseColor;
}

float ResolveFalloff(float radius, float radiusSquared, uint mode) {
    uint profile = styleData.renderControl.y;
    if (mode == 1u) {
        profile = 0u;
    } else if (mode == 6u) {
        profile = 2u;
    }

    if (profile == 0u) {
        return 1.0;
    }
    if (profile == 2u) {
        return exp(-radiusSquared * max(0.001, styleData.renderParams0.z));
    }
    if (profile == 3u) {
        return pow(max(0.0, 1.0 - radius), max(0.001, styleData.renderParams0.w));
    }
    return smoothstep(1.0, clamp(styleData.renderParams0.y, 0.0, 0.99), radius);
}

float WeightedAlphaWeight(float alpha) {
    const float depthNorm = clamp(
        (inViewDepth - uniforms.depthParameters.y) /
        max(1e-5, uniforms.depthParameters.z - uniforms.depthParameters.y),
        0.0,
        1.0);
    const float opacityWeight = pow(min(1.0, alpha * 8.0) + 0.01, 3.0);
    const float frontWeight = pow(1.0 - depthNorm, 4.0);
    return clamp(
        (opacityWeight * 0.5) + (opacityWeight * frontWeight * 128.0),
        1e-3,
        256.0);
}

void main() {
    float radiusSquared = dot(inDiscCoord, inDiscCoord);
    if (radiusSquared > 1.0) {
        discard;
    }

    uint mode = styleData.renderControl.x;
    if (mode == 5u) {
        mode = 4u;
    }

    const float radius = sqrt(radiusSquared);
    const float falloff = ResolveFalloff(radius, radiusSquared, mode);
    const float opacity = clamp(inOpacity, 0.0, 1.0);
    const float alpha = clamp(opacity * falloff, 0.0, 0.995);
    if (alpha <= 1e-5) {
        discard;
    }

    const vec3 baseColor = ResolveBaseColor();
    outAccumulation = vec4(0.0);
    outRevealage = 0.0;
    outEmission = vec4(0.0);

    if (mode == 4u || mode == 6u) {
        const float densityScale = max(0.0, styleData.renderParams2.x);
        const float densityClamp = max(0.0, styleData.renderParams2.y);
        const float weightedAlpha = densityClamp > 0.0 ? min(alpha * max(1.0, densityScale), densityClamp) : alpha;
        const float weight = WeightedAlphaWeight(weightedAlpha);
        outAccumulation = vec4(baseColor * weightedAlpha * weight, weightedAlpha * weight);
        outRevealage = weightedAlpha;
        return;
    }

    if (mode == 3u) {
        const float sceneDepth = subpassLoad(sceneDepthInput).r;
        const float xrayStrength = clamp(inXray, 0.0, 1.0);
        if (sceneDepth >= 0.999999 || xrayStrength <= 1e-5) {
            discard;
        }

        const float depthBias = max(0.0, styleData.renderParams1.y);
        const float behind = max(gl_FragCoord.z - sceneDepth - depthBias, 0.0);
        const float hiddenFade = exp(-behind * max(0.0, styleData.renderParams1.x));
        const float frontMask = gl_FragCoord.z <= sceneDepth + depthBias ? 1.0 : 0.0;
        const float xrayAlpha =
            alpha * xrayStrength * mix(styleData.renderParams1.w * hiddenFade, styleData.renderParams1.z, frontMask);
        if (xrayAlpha <= 1e-5) {
            discard;
        }

        const float gain = max(1.0, inEmissive) * max(0.0, styleData.renderParams0.x);
        outEmission = vec4(baseColor * xrayAlpha * gain, xrayAlpha * gain);
        return;
    }

    const float gain = max(0.0, inEmissive) * max(0.0, styleData.renderParams0.x);
    if (gain <= 1e-5) {
        discard;
    }
    outEmission = vec4(baseColor * alpha * gain, alpha * gain);
}
