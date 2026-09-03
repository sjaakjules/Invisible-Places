#version 450
#extension GL_GOOGLE_include_directive : require
#include "pointcloud_colormaps.glsl"
#include "pointcloud_coverage.glsl"

layout(location = 0) in vec4 inSourceColor;
layout(location = 1) in float inColormapValue;
layout(location = 2) in float inOpacity;
layout(location = 3) in float inEmissive;
layout(location = 5) in float inDepthFade;
layout(location = 6) in float inViewDepth;
layout(location = 7) in vec2 inDiscCoord;
layout(location = 8) flat in uint inPointIndex;
layout(location = 9) in float inSurfaceAngleMask;
layout(location = 12) in vec4 inWaterColourTransform;
layout(location = 13) flat in vec4 inTimingColouriseTransform;
layout(location = 14) flat in vec4 inTimingColouriseScale;

#ifdef POINTCLOUD_SORTED_ALPHA
layout(location = 0) out vec4 outSortedColor;
layout(location = 1) out vec4 outSortedEmission;
#else
layout(location = 0) out vec4 outAccumulation;
layout(location = 1) out float outRevealage;
layout(location = 2) out vec4 outEmission;
#endif

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
    uvec4 shorelineWaveControl;
    vec4 shorelineWaveParams0;
    vec4 shorelineWaveParams1;
    vec4 shorelineWaveParams2;
    vec4 shorelineWaveParams3;
    vec4 shorelineWaveParams4;
    vec4 shorelineWaveParams5;
    vec4 shorelineWaveTint;
    vec4 gradientStartColor;
    vec4 gradientEndColor;
    uvec4 seepageControl;
    vec4 seepageGridParams;
    vec4 seepageBoundsMin;
    vec4 seepageBoundsMax;
    uvec4 rainImpactControl;
    vec4 rainImpactGrid;
    vec4 rainImpactRock0;
    vec4 rainImpactRock1;
    vec4 rainImpactVegetation0;
    vec4 rainImpactVegetation1;
    vec4 rainImpactSandBand;
    vec4 rainImpactResponse;
    uvec4 timingColouriseControl;
    uvec4 timingColouriseSources[8];
    vec4 timingColouriseRanges[8];
    vec4 timingColouriseFades[8];
    vec4 timingColouriseLut[512];
    uvec4 additionalShorelineCount;
    uvec4 additionalShorelineControl[4];
    vec4 additionalShorelineParams0[4];
    vec4 additionalShorelineParams1[4];
    vec4 additionalShorelineParams2[4];
    vec4 additionalShorelineParams3[4];
    vec4 additionalShorelineParams4[4];
    vec4 additionalShorelineParams5[4];
    vec4 additionalShorelineTint[4];
    vec4 depthCompositingParams;
    vec4 emissionNormalControl;
    uvec4 surfaceStabilityControl;
    vec4 surfaceStabilityParams;
} styleData;

#include "pointcloud_stylisation.glsl"
#define POINTCLOUD_TIMING_COLOURISE_FRAGMENT
#include "pointcloud_timing_colourise.glsl"

layout(input_attachment_index = 0, set = 0, binding = 3) uniform subpassInput sceneDepthInput;
#include "pointcloud_depth_compositing.glsl"

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
    // Carry most of the selected colour's lightness (keeping some of the
    // point's own shading detail) so a brighter or more vivid selection
    // genuinely brightens and saturates the result instead of only
    // re-hueing it at the point's original lightness.
    const float colorizedLightness =
        clamp(mix(sourceHsl.z, tintHsl.z, 0.65), 0.0, 1.0);
    const vec3 colorized =
        HslToRgb(vec3(tintHsl.x, tintHsl.y, colorizedLightness));
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

float ResolveDepthFadeAlpha(float depthFade) {
    const float depthNorm = clamp(
        (inViewDepth - uniforms.depthParameters.y) /
        max(1e-5, uniforms.depthParameters.z - uniforms.depthParameters.y),
        0.0,
        1.0);
    return mix(1.0, 1.0 - depthNorm, clamp(depthFade, 0.0, 1.0));
}

void main() {
    const float coverage = PointCloudDiscCoverage(inDiscCoord, styleData.renderParams2.x);
    if (coverage <= 1.0e-5) {
        discard;
    }

    float radiusSquared = dot(inDiscCoord, inDiscCoord);
    const float radius = sqrt(radiusSquared);
    const float falloff = ResolveFalloff(radius, radiusSquared);
    const float opacity = clamp(inOpacity, 0.0, 1.0);
    const float stylisedCoverage = PointStylisationCoverage(inDiscCoord, radius, radiusSquared, inPointIndex);
    const float rawAlpha =
        opacity * falloff * stylisedCoverage * ResolveDepthFadeAlpha(inDepthFade) * coverage;
    const float compensatedRawAlpha = rawAlpha * max(0.0, styleData.renderParams1.z);
    const float alpha = clamp(compensatedRawAlpha, 0.0, AlphaClampMax());
    if (alpha <= 1e-5) {
        discard;
    }
    if (PointCloudDepthPrepassRejectsFragment()) {
        discard;
    }

    const vec3 timingColour =
        ApplyTimingColouriseStack(ApplyColorize(ResolveBaseColor()));
    vec3 baseColor = PointStylisationColor(
        timingColour * inWaterColourTransform.w +
            inWaterColourTransform.rgb,
        inDiscCoord,
        radius,
        inPointIndex,
        inSurfaceAngleMask);
#ifdef POINTCLOUD_SORTED_ALPHA
    outSortedEmission = vec4(0.0);
    vec3 sortedColor = baseColor;
    float resolvedEmissive = max(0.0, inEmissive);
    const float timingEmissionAdd = ResolveTimingColouriseEmissionAdd();
    if (timingEmissionAdd > 0.0) {
        resolvedEmissive += timingEmissionAdd;
    }
    const float emissionGain =
        resolvedEmissive * max(0.0, styleData.renderParams0.x);
    if (emissionGain > 1e-5) {
        if (PointCloudSaturatedEmissionEnabled()) {
            sortedColor = PointCloudSaturatedEmissionColor(
                baseColor,
                compensatedRawAlpha,
                alpha,
                emissionGain);
        } else {
            outSortedEmission = vec4(
                baseColor * compensatedRawAlpha * emissionGain,
                compensatedRawAlpha * emissionGain);
        }
    }
    outSortedColor = vec4(sortedColor, alpha);
#else
    outAccumulation = vec4(0.0);
    outRevealage = 0.0;
    outEmission = vec4(0.0);
    float resolvedEmissive = max(0.0, inEmissive);
    const float timingEmissionAdd = ResolveTimingColouriseEmissionAdd();
    if (timingEmissionAdd > 0.0) {
        resolvedEmissive += timingEmissionAdd;
    }
    const float emissionGain = resolvedEmissive * max(0.0, styleData.renderParams0.x);
    vec3 accumulationColor = baseColor;
    if (emissionGain > 1e-5) {
        if (PointCloudSaturatedEmissionEnabled()) {
            accumulationColor = PointCloudSaturatedEmissionColor(
                baseColor,
                compensatedRawAlpha,
                alpha,
                emissionGain);
        } else {
            outEmission = vec4(
                baseColor * compensatedRawAlpha * emissionGain,
                compensatedRawAlpha * emissionGain);
        }
    }
    const float weightedAlpha = clamp(alpha, 0.0, AlphaClampMax());
    const float weight = PointCloudWeightedAlphaWeight(weightedAlpha);
    outAccumulation = vec4(accumulationColor * weightedAlpha * weight, weightedAlpha * weight);
    outRevealage = weightedAlpha;
#endif
}
