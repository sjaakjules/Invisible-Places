#version 450

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec4 inColor;
layout(location = 2) in float inScalarValue;

layout(location = 0) out vec4 outSourceColor;
layout(location = 1) out float outScalarValue;
layout(location = 2) out float outViewDepth;

layout(set = 0, binding = 0) uniform FrameUniforms {
    mat4 viewProjection;
    mat4 view;
    vec4 cameraPosition;
    vec4 depthParameters;
} uniforms;

layout(push_constant) uniform PointCloudStyle {
    vec4 solidColor;
    vec4 scalarRange;
    vec4 shading;
    uvec4 control;
    vec4 extra;
} styleData;

void main() {
    vec4 worldPosition = vec4(inPosition, 1.0);
    vec4 viewPosition = uniforms.view * worldPosition;
    gl_Position = uniforms.viewProjection * worldPosition;
    gl_PointSize = max(1.0, styleData.shading.x);

    outSourceColor = inColor;
    outScalarValue = inScalarValue;
    outViewDepth = -viewPosition.z;
}
