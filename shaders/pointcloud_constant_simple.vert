#version 450
#extension GL_GOOGLE_include_directive : require

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec4 inColor;

layout(location = 0) out vec4 outSourceColor;
layout(location = 1) out float outViewDepth;
layout(location = 2) out vec3 outAovNormal;
layout(location = 3) out float outKernelEnergy;
layout(location = 4) out float outKernelSpriteRatio;

layout(set = 0, binding = 0) uniform FrameUniforms {
    mat4 viewProjection;
    mat4 view;
    mat4 projection;
    vec4 cameraPosition;
    vec4 depthParameters;
    vec4 viewportParameters;
    vec4 depthOfFieldParameters;
    vec4 adaptiveDensityParameters;
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
} styleData;

#include "pointcloud_adaptive_density.glsl"

layout(set = 0, binding = 6, std430) readonly buffer PointNormals {
    vec4 normals[];
} pointNormals;

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

float WorldDiameterToScreenPointSizePixels(float diameterMeters, float viewDepth) {
    return max(0.0, diameterMeters) *
           abs(uniforms.projection[1][1]) *
           max(1.0, uniforms.viewportParameters.y) /
           (2.0 * max(0.001, viewDepth));
}

vec3 ResolveAovNormal(uint pointIndex) {
    if (styleData.pointMeta.z == 0u || pointIndex >= styleData.pointMeta.x) {
        return vec3(0.0);
    }
    vec3 normal = pointNormals.normals[pointIndex].xyz;
    if (dot(normal, normal) <= 1e-8) {
        return vec3(0.0);
    }
    return normalize(normal);
}

void main() {
    const uint pointIndex = uint(gl_VertexIndex);
    vec4 worldPosition = vec4(inPosition, 1.0);
    vec4 viewPosition = uniforms.view * worldPosition;
    const float viewDepth = -viewPosition.z;
    gl_Position = uniforms.viewProjection * worldPosition;
    if (!AdaptiveDensityKeepsPoint(
            worldPosition.xyz,
            pointIndex,
            viewDepth)) {
        gl_Position = vec4(2.0, 2.0, 2.0, 1.0);
        gl_PointSize = 0.0;
        outSourceColor = inColor;
        outViewDepth = viewDepth;
        outAovNormal = vec3(0.0);
        outKernelEnergy = 0.0;
        outKernelSpriteRatio = 1.0;
        return;
    }

    const bool worldSizedScreenSprites = styleData.renderParams2.w > 0.5;
    const float footprintScale = max(1.0e-6, styleData.renderParams1.y);
    const float basePointSize =
        worldSizedScreenSprites
            ? WorldDiameterToScreenPointSizePixels(
                  max(0.0, styleData.surfelDiameterBinding.constantValue.x),
                  viewDepth)
            : max(0.0, styleData.pointSizeBinding.constantValue.x);
    const float minPointSize = max(1.0, styleData.renderParams3.y);
    const float maxPointSize = max(minPointSize, styleData.renderParams3.z);
    // The antialias support is a fixed screen-space margin (see
    // pointcloud_preview.vert): only the authored kernel scales with the
    // density footprint. Camera blur remains a post-density image effect.
    const float densityAdjustedPointSize =
        basePointSize * footprintScale + max(0.0, styleData.renderParams2.x);
    gl_PointSize = clamp(
        densityAdjustedPointSize + ResolveDepthOfFieldBlurPixels(viewDepth),
        minPointSize,
        maxPointSize);
    // Floor-only energy conservation and kernel-true falloff normalisation;
    // see pointcloud_preview.vert for the rules.
    const float floorEnergyRatio =
        clamp(densityAdjustedPointSize / minPointSize, 0.0, 1.0);
    outKernelEnergy = floorEnergyRatio * floorEnergyRatio;
    const float dofKernelSize = basePointSize * footprintScale +
                                ResolveDepthOfFieldBlurPixels(viewDepth);
    const float spriteSize = clamp(
        densityAdjustedPointSize + ResolveDepthOfFieldBlurPixels(viewDepth),
        minPointSize,
        maxPointSize);
    outKernelSpriteRatio =
        densityAdjustedPointSize + ResolveDepthOfFieldBlurPixels(viewDepth) <=
                minPointSize
            ? 1.0
            : clamp(dofKernelSize / spriteSize, 1.0e-3, 1.0);

    outSourceColor = inColor;
    outViewDepth = viewDepth;
    outAovNormal = ResolveAovNormal(pointIndex);
}
