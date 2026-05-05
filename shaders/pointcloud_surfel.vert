#version 450

layout(location = 0) out vec4 outSourceColor;
layout(location = 1) out float outColormapValue;
layout(location = 2) out float outOpacity;
layout(location = 3) out float outEmissive;
layout(location = 4) out float outXray;
layout(location = 5) out float outDepthFade;
layout(location = 6) out float outViewDepth;
layout(location = 7) out vec2 outDiscCoord;

layout(set = 0, binding = 0) uniform FrameUniforms {
    mat4 viewProjection;
    mat4 view;
    mat4 projection;
    vec4 cameraPosition;
    vec4 depthParameters;
    vec4 viewportParameters;
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

layout(set = 0, binding = 4, std430) readonly buffer SurfelPositions {
    vec4 positions[];
} surfelPositions;

layout(set = 0, binding = 5, std430) readonly buffer SurfelColors {
    uint colors[];
} surfelColors;

layout(set = 0, binding = 6, std430) readonly buffer SurfelNormals {
    vec4 normals[];
} surfelNormals;

const uint kFieldMapFlagClamp = 1u;
const uint kFieldMapFlagInvert = 2u;
const uint kSurfelVerticesPerPoint = 6u;

const vec2 kSurfelCorners[6] = vec2[](
    vec2(-1.0, -1.0),
    vec2(1.0, -1.0),
    vec2(1.0, 1.0),
    vec2(-1.0, -1.0),
    vec2(1.0, 1.0),
    vec2(-1.0, 1.0));

float LoadScalarFieldValue(uint fieldSlot, uint pointIndex) {
    if (fieldSlot == 0xFFFFFFFFu ||
        fieldSlot >= styleData.globalControl.z ||
        styleData.pointMeta.x == 0u ||
        pointIndex >= styleData.pointMeta.x) {
        return 0.0;
    }

    const uint scalarIndex = (fieldSlot * styleData.pointMeta.x) + pointIndex;
    return scalarFieldValues.values[scalarIndex];
}

float EvaluateBinding(RenderParameterBindingGpu binding, uint pointIndex) {
    if (binding.control.x == 0u) {
        return binding.constantValue.x;
    }

    float normalized =
        (LoadScalarFieldValue(binding.control.y, pointIndex) - binding.range.x) /
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
    bool useNormal = styleData.pointMeta.z != 0u;
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

void main() {
    const uint encodedVertexIndex = uint(gl_VertexIndex);
    const uint pointIndex = encodedVertexIndex / kSurfelVerticesPerPoint;
    const uint cornerIndex = encodedVertexIndex - (pointIndex * kSurfelVerticesPerPoint);
    const vec2 corner = kSurfelCorners[int(cornerIndex)];

    const vec3 center = surfelPositions.positions[pointIndex].xyz;
    vec3 tangent;
    vec3 bitangent;
    ResolveBasis(center, pointIndex, tangent, bitangent);

    const float diameter = max(0.0, EvaluateBinding(styleData.surfelDiameterBinding, pointIndex));
    const vec3 offset = (tangent * corner.x + bitangent * corner.y) * (diameter * 0.5);
    const vec4 worldPosition = vec4(center + offset, 1.0);
    const vec4 viewPosition = uniforms.view * worldPosition;

    gl_Position = uniforms.viewProjection * worldPosition;

    outSourceColor = UnpackRgba8(surfelColors.colors[pointIndex]);
    outColormapValue = EvaluateBinding(styleData.colormapPositionBinding, pointIndex);
    outOpacity = EvaluateBinding(styleData.opacityBinding, pointIndex);
    outEmissive = EvaluateBinding(styleData.emissiveBinding, pointIndex);
    outXray = EvaluateBinding(styleData.xrayBinding, pointIndex);
    outDepthFade = EvaluateBinding(styleData.depthFadeBinding, pointIndex);
    outViewDepth = -viewPosition.z;
    outDiscCoord = corner;
}
