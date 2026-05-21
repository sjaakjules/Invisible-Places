#version 450

layout(location = 0) out vec4 outSourceColor;
layout(location = 1) out float outViewDepth;
layout(location = 2) out vec2 outDiscCoord;
layout(location = 3) out vec3 outAovNormal;

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
} styleData;

layout(set = 0, binding = 4, std430) readonly buffer SurfelPositions {
    vec4 positions[];
} surfelPositions;

layout(set = 0, binding = 5, std430) readonly buffer SurfelColors {
    uint colors[];
} surfelColors;

layout(set = 0, binding = 6, std430) readonly buffer SurfelNormals {
    vec4 normals[];
} surfelNormals;

const uint kSurfelVerticesPerPoint = 6u;

const vec2 kSurfelCorners[6] = vec2[](
    vec2(-1.0, -1.0),
    vec2(1.0, -1.0),
    vec2(1.0, 1.0),
    vec2(-1.0, -1.0),
    vec2(1.0, 1.0),
    vec2(-1.0, 1.0));

vec4 UnpackRgba8(uint packedColor) {
    return vec4(
        float(packedColor & 0xFFu) / 255.0,
        float((packedColor >> 8u) & 0xFFu) / 255.0,
        float((packedColor >> 16u) & 0xFFu) / 255.0,
        float((packedColor >> 24u) & 0xFFu) / 255.0);
}

vec3 CameraRight() {
    return normalize(vec3(uniforms.view[0][0], uniforms.view[1][0], uniforms.view[2][0]));
}

vec3 CameraUp() {
    return normalize(vec3(uniforms.view[0][1], uniforms.view[1][1], uniforms.view[2][1]));
}

void ResolveBasis(vec3 center, uint pointIndex, out vec3 tangent, out vec3 bitangent) {
    const vec3 cameraRight = CameraRight();
    const vec3 cameraUp = CameraUp();
    const bool forceCameraFacing = styleData.renderControl.z == 2u;
    bool useNormal = styleData.pointMeta.z != 0u && !forceCameraFacing;
    vec3 normal = useNormal ? surfelNormals.normals[pointIndex].xyz : vec3(0.0);
    useNormal = useNormal && dot(normal, normal) > 1e-8;

    if (!useNormal) {
        tangent = cameraRight;
        bitangent = cameraUp;
        return;
    }

    normal = normalize(normal);
    tangent = cameraRight - (normal * dot(cameraRight, normal));
    if (dot(tangent, tangent) <= 1e-8) {
        tangent = cameraUp - (normal * dot(cameraUp, normal));
    }
    if (dot(tangent, tangent) <= 1e-8) {
        tangent = abs(normal.z) < 0.999 ? cross(vec3(0.0, 0.0, 1.0), normal)
                                        : cross(vec3(0.0, 1.0, 0.0), normal);
    }

    tangent = normalize(tangent);
    bitangent = normalize(cross(normal, tangent));
}

vec3 ResolveAovNormal(uint pointIndex) {
    if (styleData.pointMeta.z == 0u || pointIndex >= styleData.pointMeta.x) {
        return vec3(0.0);
    }
    vec3 normal = surfelNormals.normals[pointIndex].xyz;
    if (dot(normal, normal) <= 1e-8) {
        return vec3(0.0);
    }
    return normalize(normal);
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

float ResolveDepthOfFieldWorldRadius(float viewDepth) {
    const float blurPixels = ResolveDepthOfFieldBlurPixels(viewDepth);
    const float blurNdcY = blurPixels * uniforms.viewportParameters.w;
    return max(0.0, blurNdcY * max(0.001, viewDepth) / max(abs(uniforms.projection[1][1]), 1e-5));
}

void main() {
    const uint encodedVertexIndex = uint(gl_VertexIndex);
    const uint pointIndex = encodedVertexIndex / kSurfelVerticesPerPoint;
    const uint cornerIndex = encodedVertexIndex - (pointIndex * kSurfelVerticesPerPoint);
    const vec2 corner = kSurfelCorners[int(cornerIndex)];

    const vec3 center = surfelPositions.positions[pointIndex].xyz;
    vec3 tangent;
    vec3 bitangent;
    ResolveBasis(center, pointIndex, tangent, bitangent);

    const vec4 centerViewPosition = uniforms.view * vec4(center, 1.0);
    const float centerDepth = -centerViewPosition.z;
    const float diameter =
        max(0.0, styleData.surfelDiameterBinding.constantValue.x) +
        (ResolveDepthOfFieldWorldRadius(centerDepth) * 2.0);
    const vec3 offset = (tangent * corner.x + bitangent * corner.y) * (diameter * 0.5);
    const vec4 worldPosition = vec4(center + offset, 1.0);
    const vec4 viewPosition = uniforms.view * worldPosition;

    gl_Position = uniforms.viewProjection * worldPosition;

    outSourceColor = UnpackRgba8(surfelColors.colors[pointIndex]);
    outViewDepth = -viewPosition.z;
    outDiscCoord = corner;
    outAovNormal = ResolveAovNormal(pointIndex);
}
