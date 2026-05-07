#version 450

layout(location = 2) in float inOpacity;
layout(location = 5) in float inDepthFade;
layout(location = 6) in float inViewDepth;
layout(location = 7) in vec2 inDiscCoord;

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
} styleData;

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

float ResolveDepthFadeAlpha(float depthFade) {
    const float depthNorm = clamp(
        (inViewDepth - uniforms.depthParameters.y) /
        max(1e-5, uniforms.depthParameters.z - uniforms.depthParameters.y),
        0.0,
        1.0);
    return mix(1.0, 1.0 - depthNorm, clamp(depthFade, 0.0, 1.0));
}

void main() {
    const float radiusSquared = dot(inDiscCoord, inDiscCoord);
    if (radiusSquared > 1.0) {
        discard;
    }

    const float radius = sqrt(radiusSquared);
    const float alpha =
        clamp(inOpacity, 0.0, 1.0) * ResolveFalloff(radius, radiusSquared) * ResolveDepthFadeAlpha(inDepthFade);
    if (alpha <= 1e-5 ||
        styleData.renderControl.x == 0u ||
        (styleData.renderControl.x == 1u && alpha < clamp(styleData.renderParams3.x, 0.0, 1.0))) {
        discard;
    }

    outLinearDepth = inViewDepth;
}
