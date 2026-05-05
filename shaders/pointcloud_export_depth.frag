#version 450

layout(location = 2) in float inOpacity;
layout(location = 6) in float inViewDepth;

layout(location = 0) out float outLinearDepth;

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

float ResolveFalloff(float radius, float radiusSquared) {
    const uint renderMode = styleData.renderControl.x;
    uint profile = styleData.renderControl.y;
    if (renderMode == 1u) {
        profile = 0u;
    } else if (renderMode == 6u) {
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

void main() {
    const vec2 centered = (gl_PointCoord * 2.0) - 1.0;
    const float radiusSquared = dot(centered, centered);
    if (radiusSquared > 1.0) {
        discard;
    }

    const float radius = sqrt(radiusSquared);
    const float alpha = clamp(inOpacity, 0.0, 1.0) * ResolveFalloff(radius, radiusSquared);
    if (alpha <= 1e-5) {
        discard;
    }

    outLinearDepth = inViewDepth;
}
