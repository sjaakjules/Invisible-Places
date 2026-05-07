#version 450

layout(location = 0) in vec4 inSourceColor;
layout(location = 1) in float inViewDepth;
layout(location = 2) in vec2 inDiscCoord;

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

vec3 ResolveBaseColor() {
    if (styleData.globalControl.x == 1u || styleData.globalControl.w == 0u) {
        return styleData.solidColor.rgb;
    }
    return inSourceColor.rgb;
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

void main() {
    float radiusSquared = dot(inDiscCoord, inDiscCoord);
    if (radiusSquared > 1.0) {
        discard;
    }

    const float radius = sqrt(radiusSquared);
    const float falloff = ResolveFalloff(radius, radiusSquared);
    const float opacity = clamp(styleData.opacityBinding.constantValue.x, 0.0, 1.0);
    const float alpha = clamp(opacity * falloff, 0.0, 0.995);
    if (alpha <= 1e-5) {
        discard;
    }

    const vec3 baseColor = ApplyColorize(ResolveBaseColor());
    const float densityScale = max(0.0, styleData.renderParams2.x);
    const float densityClamp = max(0.0, styleData.renderParams2.y);
    const float densityAlpha = densityClamp > 0.0 ? min(alpha * max(1.0, densityScale), densityClamp) : alpha;
    const float weightedAlpha = clamp(densityAlpha, 0.0, 0.995);
    const float weight = WeightedAlphaWeight(weightedAlpha);

    outAccumulation = vec4(baseColor * weightedAlpha * weight, weightedAlpha * weight);
    outRevealage = weightedAlpha;
    outEmission = vec4(0.0);

    const float emissionGain =
        max(0.0, styleData.emissiveBinding.constantValue.x) * max(0.0, styleData.renderParams0.x);
    if (emissionGain > 1e-5) {
        outEmission += vec4(baseColor * alpha * emissionGain, alpha * emissionGain);
    }
}
