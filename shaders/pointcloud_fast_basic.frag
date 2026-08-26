#version 450
#extension GL_GOOGLE_include_directive : require
#include "pointcloud_colormaps.glsl"

layout(location = 0) in vec4 inSourceColor;
layout(location = 2) flat in uint inPointIndex;
layout(location = 3) in vec3 inWorldPosition;
layout(location = 4) in vec3 inPointNormal;
layout(location = 5) in float inFlowCoverage;
layout(location = 6) flat in vec4 inTimingColouriseTransform;
layout(location = 7) flat in float inTimingColouriseEmissionAdd;

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
} styleData;

#include "pointcloud_sparse_ripple.glsl"
#include "pointcloud_rain_impact.glsl"
#include "pointcloud_mesh_flow_contact.glsl"
#define POINTCLOUD_TIMING_COLOURISE_FRAGMENT
#include "pointcloud_timing_colourise.glsl"

const uint kFieldMapFlagClamp = 1u;
const uint kFieldMapFlagInvert = 2u;
const uint kWaterParticleRoleFieldSlot = 9u;
const uint kWaterJitterSeedFieldSlot = 12u;
const uint kWaterAgeFieldSlot = 13u;
const uint kWaterTrailRoleFieldSlot = 0u;
const uint kWaterTrailTangentZFieldSlot = 24u;

float LoadScalarFieldValue(uint fieldSlot) {
    if (fieldSlot == 0xFFFFFFFFu ||
        fieldSlot >= styleData.globalControl.z ||
        styleData.pointMeta.x == 0u ||
        inPointIndex >= styleData.pointMeta.x) {
        return 0.0;
    }
    return scalarFieldValues.values[(fieldSlot * styleData.pointMeta.x) + inPointIndex];
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
    // Carry most of the selected colour's lightness so brightness and
    // vividness of the selection participate in the blend.
    const float colorizedLightness =
        clamp(mix(sourceHsl.z, tintHsl.z, 0.65), 0.0, 1.0);
    return mix(
        baseColor,
        HslToRgb(vec3(tintHsl.x, tintHsl.y, colorizedLightness)),
        amount);
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
    baseColor = ApplyTimingColouriseStack(baseColor);
    const SparseRippleComposite sparseRipple =
        ResolveSparseRippleComposite(inWorldPosition, inPointNormal, inPointIndex, styleData.renderParams3.w);
    const RainImpactComposite rainImpact = ResolveRainImpactComposite(inWorldPosition, inPointNormal);
    const MeshFlowContactComposite meshFlowContact =
        ResolveMeshFlowContactComposite(
            inWorldPosition,
            inPointNormal,
            uniforms.depthParameters.x);
    const vec3 composedColour =
        ApplyMeshFlowContactColour(
            ApplyRainImpactColour(
                ApplySparseRippleColor(baseColor, sparseRipple),
                rainImpact),
            meshFlowContact);
    // Fast Basic is intentionally opaque and has no separate emission target;
    // represent emission as a bounded colour lift.
    const vec3 finalColour =
        composedColour +
        meshFlowContact.tint *
            min(1.0, meshFlowContact.emissionAdd) *
            0.35;
    const float timingEmissionAdd = ResolveTimingColouriseEmissionAdd();
    if (timingEmissionAdd <= 0.0) {
        return finalColour;
    }
    return finalColour *
           (1.0 + 0.35 * min(1.0, timingEmissionAdd));
}

float WaterFlowCoverageNoise() {
    const vec2 spriteCell = floor(gl_PointCoord * 16.0);
    const float hashInput =
        float(inPointIndex) * 0.754877666 +
        spriteCell.x * 12.9898 +
        spriteCell.y * 78.233;
    return fract(sin(hashInput) * 43758.5453);
}

void main() {
    float waterTrailFade = 1.0;
    if (styleData.pointMeta.w == 3u && styleData.globalControl.z > kWaterTrailTangentZFieldSlot) {
        if (LoadScalarFieldValue(kWaterTrailRoleFieldSlot) < 0.5) {
            discard;
        }
        if (inFlowCoverage <= 0.0 || WaterFlowCoverageNoise() > clamp(inFlowCoverage, 0.0, 1.0)) {
            discard;
        }
        outColor = vec4(ResolveBaseColor(), 1.0);
        return;
    }
    if (styleData.pointMeta.w == 3u) {
        outColor = vec4(ResolveBaseColor(), 1.0);
        return;
    }
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
        if (styleData.pointMeta.w != 2u && styleData.globalControl.z > kWaterAgeFieldSlot) {
            const float age = clamp(LoadScalarFieldValue(kWaterAgeFieldSlot), 0.0, 1.0);
            waterTrailFade = 0.35 + 0.65 * pow(1.0 - smoothstep(0.0, 1.0, age), 1.35);
        }
    }
    outColor = vec4(ResolveBaseColor() * waterTrailFade, 1.0);
}
