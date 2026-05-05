#version 450

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec4 inColor;

layout(location = 0) out vec4 outSourceColor;
layout(location = 1) out float outColormapValue;
layout(location = 2) out float outOpacity;
layout(location = 3) out float outEmissive;
layout(location = 4) out float outXray;
layout(location = 5) out float outDepthFade;
layout(location = 6) out float outViewDepth;

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
    RenderParameterBindingGpu pointSizeBinding;
    RenderParameterBindingGpu opacityBinding;
    RenderParameterBindingGpu emissiveBinding;
    RenderParameterBindingGpu xrayBinding;
    RenderParameterBindingGpu depthFadeBinding;
    RenderParameterBindingGpu colormapPositionBinding;
    RenderParameterBindingGpu surfelDiameterBinding;
} styleData;

const uint kFieldMapFlagClamp = 1u;
const uint kFieldMapFlagInvert = 2u;

float LoadScalarFieldValue(uint fieldSlot) {
    if (fieldSlot == 0xFFFFFFFFu ||
        fieldSlot >= styleData.globalControl.z ||
        styleData.pointMeta.x == 0u) {
        return 0.0;
    }

    const uint pointIndex = uint(gl_VertexIndex);
    if (pointIndex >= styleData.pointMeta.x) {
        return 0.0;
    }

    const uint scalarIndex = (fieldSlot * styleData.pointMeta.x) + pointIndex;
    return scalarFieldValues.values[scalarIndex];
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

float ResolveDepthOfFieldBlurPixels(float viewDepth) {
    if (uniforms.depthOfFieldParameters.x <= 0.5) {
        return 0.0;
    }

    const float focusDistance = max(0.001, uniforms.depthOfFieldParameters.y);
    const float apertureFStops = max(0.1, uniforms.depthOfFieldParameters.z);
    const float maxBlurPixels = max(0.0, uniforms.depthOfFieldParameters.w);
    const float distanceFromFocus = abs(viewDepth - focusDistance) / max(max(viewDepth, focusDistance), 0.001);
    return clamp(distanceFromFocus * (8.0 / apertureFStops) * maxBlurPixels, 0.0, maxBlurPixels);
}

void main() {
    vec4 worldPosition = vec4(inPosition, 1.0);
    vec4 viewPosition = uniforms.view * worldPosition;
    const float viewDepth = -viewPosition.z;
    gl_Position = uniforms.viewProjection * worldPosition;
    const float basePointSize = clamp(
        EvaluateBinding(styleData.pointSizeBinding),
        max(1.0, styleData.renderParams2.z),
        max(max(1.0, styleData.renderParams2.z), styleData.renderParams2.w));
    gl_PointSize = clamp(
        basePointSize + ResolveDepthOfFieldBlurPixels(viewDepth),
        max(1.0, styleData.renderParams2.z),
        max(max(1.0, styleData.renderParams2.z), styleData.renderParams2.w));

    outSourceColor = inColor;
    outColormapValue = EvaluateBinding(styleData.colormapPositionBinding);
    outOpacity = EvaluateBinding(styleData.opacityBinding);
    outEmissive = EvaluateBinding(styleData.emissiveBinding);
    outXray = EvaluateBinding(styleData.xrayBinding);
    outDepthFade = EvaluateBinding(styleData.depthFadeBinding);
    outViewDepth = viewDepth;
}
