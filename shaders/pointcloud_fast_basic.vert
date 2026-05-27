#version 450

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec4 inColor;

layout(location = 0) out vec4 outSourceColor;
layout(location = 1) out float outViewDepth;
layout(location = 2) flat out uint outPointIndex;
layout(location = 3) out vec3 outWorldPosition;
layout(location = 4) out vec3 outPointNormal;

layout(set = 0, binding = 0) uniform FrameUniforms {
    mat4 viewProjection;
    mat4 view;
    mat4 projection;
    vec4 cameraPosition;
    vec4 depthParameters;
    vec4 viewportParameters;
    vec4 depthOfFieldParameters;
} uniforms;

layout(set = 0, binding = 4, std430) readonly buffer PointPositions {
    vec4 positions[];
} pointPositions;

layout(set = 0, binding = 5, std430) readonly buffer PointColors {
    uint colors[];
} pointColors;

layout(set = 0, binding = 6, std430) readonly buffer PointNormals {
    vec4 normals[];
} pointNormals;

struct PointDrawItemGpu {
    uvec4 indices;
    vec4 params;
};

layout(set = 0, binding = 7, std430) readonly buffer PointDrawItems {
    PointDrawItemGpu drawItems[];
} pointDrawItems;

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
} styleData;

const uint kRenderControlUseDrawItems = 2u;

bool UseDrawItems() {
    return (styleData.renderControl.w & kRenderControlUseDrawItems) != 0u;
}

uint SourcePointIndex(uint drawIndex) {
    return UseDrawItems() ? pointDrawItems.drawItems[drawIndex].indices.x : drawIndex;
}

float DrawItemFootprintDiameterPixels(uint drawIndex) {
    return UseDrawItems() ? sqrt(max(1.0, pointDrawItems.drawItems[drawIndex].params.w)) : 1.0;
}

vec4 UnpackRgba8(uint packedColor) {
    return vec4(
        float(packedColor & 0xFFu) / 255.0,
        float((packedColor >> 8u) & 0xFFu) / 255.0,
        float((packedColor >> 16u) & 0xFFu) / 255.0,
        float((packedColor >> 24u) & 0xFFu) / 255.0);
}

float WorldDiameterToScreenPointSizePixels(float diameterMeters, float viewDepth) {
    return max(0.0, diameterMeters) *
           abs(uniforms.projection[1][1]) *
           max(1.0, uniforms.viewportParameters.y) /
           (2.0 * max(0.001, viewDepth));
}

void main() {
    const uint drawIndex = uint(gl_VertexIndex);
    const uint pointIndex = SourcePointIndex(drawIndex);
    const vec3 sourcePosition = UseDrawItems() ? pointPositions.positions[pointIndex].xyz : inPosition;
    const vec4 sourceColor = UseDrawItems() ? UnpackRgba8(pointColors.colors[pointIndex]) : inColor;
    vec4 worldPosition = vec4(sourcePosition, 1.0);
    vec4 viewPosition = uniforms.view * worldPosition;
    gl_Position = uniforms.viewProjection * worldPosition;
    const bool worldSizedScreenSprites = styleData.renderParams2.w > 0.5;
    const float basePointSize =
        worldSizedScreenSprites
            ? WorldDiameterToScreenPointSizePixels(styleData.surfelDiameterBinding.constantValue.x, -viewPosition.z)
            : styleData.pointSizeBinding.constantValue.x;
    gl_PointSize = clamp(
        max(basePointSize, DrawItemFootprintDiameterPixels(drawIndex)),
        max(1.0, styleData.renderParams3.y),
        max(max(1.0, styleData.renderParams3.y), styleData.renderParams3.z));
    outSourceColor = sourceColor;
    outViewDepth = -viewPosition.z;
    outPointIndex = pointIndex;
    outWorldPosition = sourcePosition;
    outPointNormal =
        styleData.pointMeta.z != 0u && pointIndex < styleData.pointMeta.x
            ? pointNormals.normals[pointIndex].xyz
            : vec3(0.0);
}
