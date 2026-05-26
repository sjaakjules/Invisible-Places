#version 450
#extension GL_GOOGLE_include_directive : require
#include "pointcloud_colormaps.glsl"

layout(location = 0) in vec4 inSourceColor;
layout(location = 2) flat in uint inPointIndex;
layout(location = 3) in vec3 inWorldPosition;
layout(location = 4) in vec3 inPointNormal;

layout(location = 0) out vec4 outColor;

layout(set = 0, binding = 1, std430) readonly buffer ScalarFieldValues {
    float values[];
} scalarFieldValues;

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
    vec4 gradientStartColor;
    vec4 gradientEndColor;
} styleData;

const uint kFieldMapFlagClamp = 1u;
const uint kFieldMapFlagInvert = 2u;
const uint kWaterParticleRoleFieldSlot = 9u;
const uint kWaterJitterSeedFieldSlot = 12u;
const uint kWaterTrailAgeFieldSlot = 13u;

float LoadScalarFieldValue(uint fieldSlot) {
    if (fieldSlot == 0xFFFFFFFFu ||
        fieldSlot >= styleData.globalControl.z ||
        styleData.pointMeta.x == 0u ||
        inPointIndex >= styleData.pointMeta.x) {
        return 0.0;
    }
    return scalarFieldValues.values[(fieldSlot * styleData.pointMeta.x) + inPointIndex];
}

#include "pointcloud_caustics.glsl"

float ResolveCausticStrength(out float previewTint) {
    previewTint = 0.0;
    if (styleData.causticControl.x == 0u ||
        styleData.causticControl.y == 0u ||
        styleData.causticControl.z == 0u ||
        styleData.causticControl.w == 0u) {
        return 0.0;
    }
    const float mask = clamp(LoadScalarFieldValue(styleData.causticControl.y - 1u), 0.0, 1.0);
    if (mask <= 1e-5) {
        return 0.0;
    }
    const float edge = clamp(LoadScalarFieldValue(styleData.causticControl.z - 1u), 0.0, 1.0);
    const float seed = LoadScalarFieldValue(styleData.causticControl.w - 1u);
    previewTint = CausticPreviewTint(mask, edge, seed);
    const float time = styleData.renderParams3.w * max(0.0, styleData.causticParams0.z);
    const vec2 metersUv = CausticSurfaceUv(inWorldPosition, inPointNormal);
    const float ridge = CausticVoronoiRidge(metersUv, seed, time, edge);
    const float edgeGate = CausticEdgeGate(metersUv, edge, seed);
    return clamp(ridge * mask * edgeGate * max(0.0, styleData.causticParams0.x), 0.0, 6.0);
}

float EvaluateBinding(RenderParameterBindingGpu binding) {
    if (binding.control.x == 0u) {
        return binding.constantValue.x;
    }

    float normalized =
        (LoadScalarFieldValue(binding.control.y) - binding.range.x) /
        max(1e-5, binding.range.y - binding.range.x);
    if ((binding.control.z & kFieldMapFlagInvert) != 0u) {
        normalized = 1.0 - normalized;
    }
    if ((binding.control.z & kFieldMapFlagClamp) != 0u) {
        normalized = clamp(normalized, 0.0, 1.0);
        normalized = pow(normalized, max(0.0001, binding.extra.x));
    } else {
        normalized = sign(normalized) * pow(abs(normalized), max(0.0001, binding.extra.x));
    }
    return binding.range.z + ((binding.range.w - binding.range.z) * normalized);
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
        lightness > 0.5 ? delta / max(1e-5, 2.0 - maxChannel - minChannel)
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
    return mix(baseColor, HslToRgb(vec3(tintHsl.x, tintHsl.y, sourceHsl.z)), amount);
}

vec3 ResolveBaseColor() {
    vec3 baseColor = styleData.solidColor.rgb;
    if (styleData.globalControl.x == 0u && styleData.globalControl.w != 0u) {
        baseColor = inSourceColor.rgb;
    } else if (styleData.globalControl.x == 2u) {
        baseColor = ApplyPointCloudColormapOrGradient(
            styleData.globalControl.y,
            clamp(EvaluateBinding(styleData.colormapPositionBinding), 0.0, 1.0),
            styleData.gradientStartColor.rgb,
            styleData.gradientEndColor.rgb);
    }
    baseColor = ApplyColorize(baseColor);
    float previewTint = 0.0;
    const float caustic = ResolveCausticStrength(previewTint);
    return mix(baseColor, styleData.causticTint.rgb, CausticColorMixAmount(caustic, previewTint));
}

void main() {
    float waterTrailFade = 1.0;
    if (styleData.pointMeta.w != 0u && styleData.globalControl.z > kWaterJitterSeedFieldSlot) {
        const float role = LoadScalarFieldValue(kWaterParticleRoleFieldSlot);
        if (styleData.pointMeta.w == 2u) {
            if (!((role >= 0.5 && role < 1.5) || (role >= 1.5 && role < 2.5) || (role >= 2.5 && role < 3.5))) {
                discard;
            }
            if (role >= 2.5 && role < 3.5) {
                waterTrailFade = 0.28;
            } else if (role >= 0.5 && role < 1.5) {
                waterTrailFade = 0.18;
            }
        } else if (role < 0.5 || role >= 1.5) {
            discard;
        }
        if (styleData.pointMeta.w != 2u && styleData.globalControl.z > kWaterTrailAgeFieldSlot) {
            const float age = clamp(LoadScalarFieldValue(kWaterTrailAgeFieldSlot), 0.0, 1.0);
            waterTrailFade = 0.35 + 0.65 * pow(1.0 - smoothstep(0.0, 1.0, age), 1.35);
        }
    }
    outColor = vec4(ResolveBaseColor() * waterTrailFade, 1.0);
}
