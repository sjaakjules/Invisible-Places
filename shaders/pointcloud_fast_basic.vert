#version 450

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec4 inColor;

layout(location = 0) out vec4 outSourceColor;
layout(location = 1) out float outViewDepth;

layout(set = 0, binding = 0) uniform FrameUniforms {
    mat4 viewProjection;
    mat4 view;
    mat4 projection;
    vec4 cameraPosition;
    vec4 depthParameters;
    vec4 viewportParameters;
    vec4 depthOfFieldParameters;
} uniforms;

void main() {
    vec4 worldPosition = vec4(inPosition, 1.0);
    vec4 viewPosition = uniforms.view * worldPosition;
    gl_Position = uniforms.viewProjection * worldPosition;
    gl_PointSize = 1.0;
    outSourceColor = inColor;
    outViewDepth = -viewPosition.z;
}
