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
layout(location = 7) flat in uint inPointIndex;
layout(location = 8) in float inSurfaceAngleMask;
layout(location = 9) in vec3 inAovNormal;
layout(location = 10) in float inCaustic;

layout(location = 0) out vec4 outAccumulation;
layout(location = 1) out float outRevealage;
layout(location = 2) out vec4 outEmission;
layout(location = 3) out vec4 outNormalAccumulation;
layout(location = 4) out vec4 outAlbedoAccumulation;

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
    vec4 surfaceMotionParams;
    vec4 surfaceMotionStats;
    uvec4 causticControl;
    vec4 causticParams0;
    vec4 causticParams1;
    vec4 causticParams2;
    vec4 causticTint;
    uvec4 waterEffectControl;
    uvec4 waterEffectSlots0;
    uvec4 waterEffectSlots1;
    uvec4 rippleEffectSlots0;
    uvec4 rippleEffectSlots1;
    uvec4 rippleEffectSlots2;
    uvec4 rippleEffectSlots3;
    vec4 gradientStartColor;
    vec4 gradientEndColor;
} styleData;

#include "pointcloud_stylisation.glsl"

layout(input_attachment_index = 0, set = 0, binding = 3) uniform subpassInput sceneDepthInput;

vec3 ResolveBaseColor() {
    vec3 baseColor = inSourceColor.rgb;
    if (styleData.globalControl.x == 1u) {
        baseColor = styleData.solidColor.rgb;
    } else if (styleData.globalControl.x == 2u) {
        baseColor = ApplyPointCloudColormapOrGradient(
            styleData.globalControl.y,
            clamp(inColormapValue, 0.0, 1.0),
            styleData.gradientStartColor.rgb,
            styleData.gradientEndColor.rgb);
    } else if (styleData.globalControl.w == 0u) {
        baseColor = styleData.solidColor.rgb;
    }
    return baseColor;
}

vec3 RgbToHsl(vec3 color) {
    const float maxChannel = max(max(color.r, color.g), color.b);
    const float minChannel = min(min(color.r, color.g), color.b);
    const float delta = maxChannel - minChannel;
    const float lightness = (maxChannel + minChannel) * 0.5;
    if (delta <= 1e-5) {
        return vec3(0.0, 0.0, lightness);
    }

    const float saturation =
        lightness > 0.5
            ? delta / max(1e-5, 2.0 - maxChannel - minChannel)
            : delta / max(1e-5, maxChannel + minChannel);
    float hue = 0.0;
    if (maxChannel == color.r) {
        hue = (color.g - color.b) / delta + (color.g < color.b ? 6.0 : 0.0);
    } else if (maxChannel == color.g) {
        hue = ((color.b - color.r) / delta) + 2.0;
    } else {
        hue = ((color.r - color.g) / delta) + 4.0;
    }
    return vec3(hue / 6.0, saturation, lightness);
}

float HueToRgb(float p, float q, float t) {
    if (t < 0.0) {
        t += 1.0;
    }
    if (t > 1.0) {
        t -= 1.0;
    }
    if (t < (1.0 / 6.0)) {
        return p + ((q - p) * 6.0 * t);
    }
    if (t < 0.5) {
        return q;
    }
    if (t < (2.0 / 3.0)) {
        return p + ((q - p) * ((2.0 / 3.0) - t) * 6.0);
    }
    return p;
}

vec3 HslToRgb(vec3 hsl) {
    if (hsl.y <= 1e-5) {
        return vec3(hsl.z);
    }
    const float q = hsl.z < 0.5 ? hsl.z * (1.0 + hsl.y) : hsl.z + hsl.y - (hsl.z * hsl.y);
    const float p = (2.0 * hsl.z) - q;
    return vec3(
        HueToRgb(p, q, hsl.x + (1.0 / 3.0)),
        HueToRgb(p, q, hsl.x),
        HueToRgb(p, q, hsl.x - (1.0 / 3.0)));
}

vec3 ApplyColorize(vec3 baseColor) {
    const float amount = clamp(styleData.colorize.a, 0.0, 1.0);
    if (amount <= 1e-5) {
        return baseColor;
    }
    const vec3 sourceHsl = RgbToHsl(clamp(baseColor, 0.0, 1.0));
    const vec3 tintHsl = RgbToHsl(clamp(styleData.colorize.rgb, 0.0, 1.0));
    const vec3 colorized = HslToRgb(vec3(tintHsl.x, tintHsl.y, sourceHsl.z));
    return mix(baseColor, colorized, amount);
}

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

float WeightedAlphaWeight(float alpha) {
    const float depthNorm = clamp(
        (inViewDepth - uniforms.depthParameters.y) /
        max(1e-5, uniforms.depthParameters.z - uniforms.depthParameters.y),
        0.0,
        1.0);
    const float opacityBase = min(1.0, alpha * 8.0) + 0.01;
    const float opacityWeight = opacityBase * opacityBase * opacityBase;
    const float frontBase = 1.0 - depthNorm;
    const float frontSquared = frontBase * frontBase;
    const float frontWeight = frontSquared * frontSquared;
    return clamp(
        (opacityWeight * 0.5) + (opacityWeight * frontWeight * 128.0),
        1e-3,
        256.0);
}

float ResolveDepthFadeAlpha(float depthFade) {
    const float depthNorm = clamp(
        (inViewDepth - uniforms.depthParameters.y) /
        max(1e-5, uniforms.depthParameters.z - uniforms.depthParameters.y),
        0.0,
        1.0);
    return mix(1.0, 1.0 - depthNorm, clamp(depthFade, 0.0, 1.0));
}

void WriteAovs(vec3 albedo, vec3 normal, float aovWeight) {
    outNormalAccumulation = vec4(0.0);
    outAlbedoAccumulation = vec4(albedo * aovWeight, aovWeight);
    if (dot(normal, normal) > 1e-8) {
        outNormalAccumulation = vec4(normalize(normal) * aovWeight, aovWeight);
    }
}

void main() {
    vec2 centered = (gl_PointCoord * 2.0) - 1.0;
    float radiusSquared = dot(centered, centered);
    if (radiusSquared > 1.0) {
        discard;
    }

    const float radius = sqrt(radiusSquared);
    const float falloff = ResolveFalloff(radius, radiusSquared);
    const float opacity = clamp(inOpacity, 0.0, 1.0);
    const float stylisedCoverage = PointStylisationCoverage(centered, radius, radiusSquared, inPointIndex);
    const float alpha =
        clamp(opacity * falloff * stylisedCoverage * ResolveDepthFadeAlpha(inDepthFade), 0.0, AlphaClampMax());
    if (alpha <= 1e-5) {
        discard;
    }

    vec3 baseColor =
        PointStylisationColor(ApplyColorize(ResolveBaseColor()), centered, radius, inPointIndex, inSurfaceAngleMask);
    baseColor = mix(baseColor, styleData.causticTint.rgb, clamp(inCaustic * 0.55, 0.0, 1.0));
    outAccumulation = vec4(0.0);
    outRevealage = 0.0;
    outEmission = vec4(0.0);
    outNormalAccumulation = vec4(0.0);
    outAlbedoAccumulation = vec4(0.0);

    const float densityScale = max(0.0, styleData.renderParams2.x);
    const float densityClamp = max(0.0, styleData.renderParams2.y);
    const float densityAlpha = densityClamp > 0.0 ? min(alpha * max(1.0, densityScale), densityClamp) : alpha;
    const float weightedAlpha = clamp(densityAlpha, 0.0, AlphaClampMax());
    const float weight = WeightedAlphaWeight(weightedAlpha);
    const float aovWeight = weightedAlpha * weight;
    outAccumulation = vec4(baseColor * aovWeight, aovWeight);
    outRevealage = weightedAlpha;
    WriteAovs(baseColor, inAovNormal, aovWeight);

    const float emissionGain = max(0.0, inEmissive) * max(0.0, styleData.renderParams0.x);
    if (emissionGain > 1e-5) {
        outEmission += vec4(baseColor * alpha * emissionGain, alpha * emissionGain);
    }

    if (inXray > 1e-5) {
        const float sceneDepth = subpassLoad(sceneDepthInput).r;
        const float xrayStrength = clamp(inXray, 0.0, 1.0);
        if (sceneDepth < 0.999999 && xrayStrength > 1e-5) {
            const float depthBias = max(0.0, styleData.renderParams1.y);
            const float behind = max(gl_FragCoord.z - sceneDepth - depthBias, 0.0);
            const float hiddenFade = exp(-behind * max(0.0, styleData.renderParams1.x));
            const float frontMask = gl_FragCoord.z <= sceneDepth + depthBias ? 1.0 : 0.0;
            const float xrayAlpha =
                alpha * xrayStrength * mix(styleData.renderParams1.w * hiddenFade, styleData.renderParams1.z, frontMask);
            if (xrayAlpha > 1e-5) {
                const float xrayGain = max(0.0, styleData.renderParams0.x);
                outEmission += vec4(baseColor * xrayAlpha * xrayGain, xrayAlpha * xrayGain);
            }
        }
    }
}
