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

layout(location = 0) out vec4 outColor;

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

vec3 ResolveSolidColor(vec3 baseColor) {
    float emissive = max(0.0, inEmissive);
    float xray = clamp(inXray, 0.0, 1.0);
    float depthFade = clamp(inDepthFade, 0.0, 1.0);
    float depthNorm = clamp(
        (inViewDepth - uniforms.depthParameters.y) /
        max(1e-5, uniforms.depthParameters.z - uniforms.depthParameters.y),
        0.0,
        1.0);
    float fade = mix(1.0, 1.0 - (depthNorm * 0.65), depthFade);

    vec3 shadedColor = baseColor * fade;
    shadedColor = mix(shadedColor, vec3(1.0), xray * 0.45);
    shadedColor += emissive * 0.35 * baseColor;
    return clamp(shadedColor, 0.0, 1.0);
}

vec4 ResolveBlendOutput(vec3 color, float alpha) {
    const uint blendMode = styleData.renderControl.w;
    if (blendMode == 3u) {
        return vec4(mix(vec3(1.0), color, alpha), alpha);
    }
    return vec4(color * alpha, alpha);
}

void main() {
    vec2 centered = (gl_PointCoord * 2.0) - 1.0;
    float radiusSquared = dot(centered, centered);
    if (radiusSquared > 1.0) {
        discard;
    }
    float radius = sqrt(radiusSquared);

    vec3 baseColor = ResolveBaseColor();
    float opacity = clamp(inOpacity, 0.0, 1.0);
    float edge = ResolveFalloff(radius, radiusSquared);
    if (opacity * edge <= 1e-5) {
        discard;
    }

    float alpha = opacity * edge;
    outColor = ResolveBlendOutput(ResolveSolidColor(baseColor), alpha);
}
