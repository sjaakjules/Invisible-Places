#version 450
#extension GL_GOOGLE_include_directive : require

layout(location = 2) in float inOpacity;
layout(location = 5) in float inDepthFade;
layout(location = 6) in float inViewDepth;
layout(location = 7) flat in uint inPointIndex;

layout(location = 0) out float outLinearDepth;

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
    vec4 renderParams3;
    RenderParameterBindingGpu pointSizeBinding;
    RenderParameterBindingGpu opacityBinding;
    RenderParameterBindingGpu emissiveBinding;
    RenderParameterBindingGpu xrayBinding;
    RenderParameterBindingGpu depthFadeBinding;
    RenderParameterBindingGpu colormapPositionBinding;
    RenderParameterBindingGpu surfelDiameterBinding;
    vec4 colorize;
    uvec4 stylisationControl;
    vec4 stylisationParams0;
    vec4 stylisationParams1;
    vec4 stylisationParams2;
} styleData;

#include "pointcloud_stylisation.glsl"

float ResolveFalloff(float radius, float radiusSquared) {
    uint profile = styleData.renderControl.y;

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

float AlphaClampMax() {
    return styleData.renderControl.w != 0u ? 1.0 : 0.995;
}

float ResolveDepthFadeAlpha(float depthFade) {
    const float depthNorm = clamp(
        (inViewDepth - uniforms.depthParameters.y) /
        max(1e-5, uniforms.depthParameters.z - uniforms.depthParameters.y),
        0.0,
        1.0);
    return mix(1.0, 1.0 - depthNorm, clamp(depthFade, 0.0, 1.0));
}

void main() {
    const vec2 centered = (gl_PointCoord * 2.0) - 1.0;
    const float radiusSquared = dot(centered, centered);
    if (radiusSquared > 1.0) {
        discard;
    }

    const float radius = sqrt(radiusSquared);
    const float alpha =
        clamp(
            clamp(inOpacity, 0.0, 1.0) *
                ResolveFalloff(radius, radiusSquared) *
                PointStylisationCoverage(centered, radius, radiusSquared, inPointIndex) *
                ResolveDepthFadeAlpha(inDepthFade),
            0.0,
            AlphaClampMax());
    if (alpha <= 1e-5 ||
        styleData.renderControl.x == 0u ||
        (styleData.renderControl.x == 1u && alpha < clamp(styleData.renderParams3.x, 0.0, 1.0))) {
        discard;
    }

    outLinearDepth = inViewDepth;
}
