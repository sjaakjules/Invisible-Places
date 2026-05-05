#version 450

layout(location = 0) in vec4 inSourceColor;
layout(location = 1) in float inColormapValue;
layout(location = 2) in float inOpacity;
layout(location = 3) in float inEmissive;
layout(location = 4) in float inXray;
layout(location = 5) in float inDepthFade;
layout(location = 6) in float inViewDepth;
layout(location = 7) in vec2 inDiscCoord;

layout(location = 0) out vec4 outColor;

layout(set = 0, binding = 0) uniform FrameUniforms {
    mat4 viewProjection;
    mat4 view;
    mat4 projection;
    vec4 cameraPosition;
    vec4 depthParameters;
    vec4 viewportParameters;
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

vec3 Viridis(float t) {
    return clamp(vec3(
        0.277727 + 0.520420 * t + 0.117231 * t * t - 0.219384 * t * t * t,
        0.005407 + 1.404613 * t - 1.653928 * t * t + 0.743293 * t * t * t,
        0.334099 + 1.384590 * t - 1.584386 * t * t + 0.630205 * t * t * t), 0.0, 1.0);
}

vec3 Plasma(float t) {
    return clamp(vec3(
        0.058732 + 2.176514 * t - 2.689460 * t * t + 1.466006 * t * t * t,
        0.023336 + 0.238383 * t + 1.118022 * t * t - 0.905937 * t * t * t,
        0.543817 + 0.753960 * t - 1.308172 * t * t + 0.806940 * t * t * t), 0.0, 1.0);
}

vec3 Inferno(float t) {
    return clamp(vec3(
        0.000218 + 1.538871 * t - 1.908164 * t * t + 0.873490 * t * t * t,
        0.001651 - 0.117089 * t + 2.064416 * t * t - 1.277355 * t * t * t,
        0.013866 + 0.635041 * t + 0.618582 * t * t - 0.700491 * t * t * t), 0.0, 1.0);
}

vec3 Magma(float t) {
    return clamp(vec3(
        0.001462 + 1.384825 * t - 1.875795 * t * t + 0.850876 * t * t * t,
        0.000466 - 0.251373 * t + 1.927205 * t * t - 1.035104 * t * t * t,
        0.013866 + 0.873807 * t + 0.143707 * t * t - 0.282206 * t * t * t), 0.0, 1.0);
}

vec3 Cividis(float t) {
    return clamp(vec3(
        0.000000 + 0.975500 * t - 0.676400 * t * t + 0.187000 * t * t * t,
        0.126200 + 0.662500 * t - 0.360900 * t * t + 0.079200 * t * t * t,
        0.301500 + 0.539600 * t - 0.540700 * t * t + 0.219900 * t * t * t), 0.0, 1.0);
}

vec3 Turbo(float t) {
    return clamp(vec3(
        0.13572138 + 4.61539260 * t - 42.66032258 * t * t + 132.13108234 * t * t * t - 152.94239396 * t * t * t * t + 59.28637943 * t * t * t * t * t,
        0.09140261 + 2.19418839 * t + 4.84296658 * t * t - 14.18503333 * t * t * t + 4.27729857 * t * t * t * t + 2.82956604 * t * t * t * t * t,
        0.10667330 + 12.64194608 * t - 60.58204836 * t * t + 110.36276771 * t * t * t - 89.90310912 * t * t * t * t + 27.34824973 * t * t * t * t * t), 0.0, 1.0);
}

vec3 ApplyColormap(uint colormapId, float t) {
    if (colormapId == 1u) {
        return Plasma(t);
    }
    if (colormapId == 2u) {
        return Inferno(t);
    }
    if (colormapId == 3u) {
        return Magma(t);
    }
    if (colormapId == 4u) {
        return Cividis(t);
    }
    if (colormapId == 5u) {
        return Turbo(t);
    }
    return Viridis(t);
}

vec3 ResolveBaseColor() {
    vec3 baseColor = inSourceColor.rgb;
    if (styleData.globalControl.x == 1u) {
        baseColor = styleData.solidColor.rgb;
    } else if (styleData.globalControl.x == 2u) {
        baseColor = ApplyColormap(styleData.globalControl.y, clamp(inColormapValue, 0.0, 1.0));
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

void main() {
    float radiusSquared = dot(inDiscCoord, inDiscCoord);
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

    outColor = vec4(ResolveSolidColor(baseColor), opacity * edge);
}
