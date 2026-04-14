#version 450

layout(location = 0) out vec2 outUv;
layout(location = 1) out float outOpacity;
layout(location = 2) out vec3 outWorldCenter;
layout(location = 3) flat out uint outSplatIndex;
layout(location = 4) out float outViewDepth;
layout(location = 5) out float outScaleMetric;
layout(location = 6) flat out uint outLayerStyleIndex;

layout(set = 0, binding = 0) uniform FrameUniforms {
    mat4 viewProjection;
    mat4 view;
    mat4 projection;
    vec4 cameraPosition;
    vec4 depthParameters;
    vec4 viewportParameters;
} uniforms;

layout(set = 0, binding = 1, std430) readonly buffer Centers {
    vec4 values[];
} centers;

layout(set = 0, binding = 2, std430) readonly buffer Scales {
    vec4 values[];
} scales;

layout(set = 0, binding = 3, std430) readonly buffer Rotations {
    vec4 values[];
} rotations;

layout(set = 0, binding = 4, std430) readonly buffer Opacities {
    float values[];
} opacities;

layout(set = 0, binding = 6, std430) readonly buffer LayerStyleIndices {
    uint values[];
} layerStyleIndices;

struct LayerStyleGpu {
    mat4 localToWorld;
    vec4 layerTint;
    vec4 style;
    uvec4 control;
};

layout(set = 0, binding = 7, std430) readonly buffer LayerStyles {
    LayerStyleGpu values[];
} layerStyles;

layout(set = 0, binding = 8, std430) readonly buffer SortedIndices {
    uint values[];
} sortedIndices;

layout(push_constant) uniform HighQualityGaussianStyle {
    vec4 extra;
} pushConstants;

vec2 CornerPosition(uint vertexIndex) {
    const vec2 corners[6] = vec2[](
        vec2(-1.0, -1.0),
        vec2( 1.0, -1.0),
        vec2( 1.0,  1.0),
        vec2(-1.0, -1.0),
        vec2( 1.0,  1.0),
        vec2(-1.0,  1.0)
    );
    return corners[vertexIndex];
}

vec3 RotateVector(vec4 quaternionWxyz, vec3 value) {
    vec3 q = quaternionWxyz.yzw;
    vec3 t = 2.0 * cross(q, value);
    return value + (quaternionWxyz.x * t) + cross(q, t);
}

vec2 ProjectAxisToNdc(vec3 viewAxis, vec3 viewCenter, float invDepth, float projectionX, float projectionY) {
    const vec3 jacobianRowX = vec3(
        projectionX * invDepth,
        0.0,
        projectionX * viewCenter.x * invDepth * invDepth
    );
    const vec3 jacobianRowY = vec3(
        0.0,
        projectionY * invDepth,
        projectionY * viewCenter.y * invDepth * invDepth
    );
    return vec2(dot(jacobianRowX, viewAxis), dot(jacobianRowY, viewAxis));
}

vec2 PrincipalEigenvector(float covXX, float covXY, float covYY, float lambda) {
    if (abs(covXY) > 1e-8) {
        return normalize(vec2(lambda - covYY, covXY));
    }
    return covXX >= covYY ? vec2(1.0, 0.0) : vec2(0.0, 1.0);
}

void main() {
    const uint mergedIndex = sortedIndices.values[uint(gl_InstanceIndex)];
    const uint layerStyleIndex = layerStyleIndices.values[mergedIndex];
    const LayerStyleGpu layerStyle = layerStyles.values[layerStyleIndex];
    const vec2 corner = CornerPosition(uint(gl_VertexIndex));

    const vec3 localCenter = centers.values[mergedIndex].xyz;
    const vec4 rotation = rotations.values[mergedIndex];
    const vec3 decodedScale = max(scales.values[mergedIndex].xyz * layerStyle.style.y, vec3(1e-4));
    const float footprintBoost = max(pushConstants.extra.x, 1e-3);

    const mat4 localToWorld = layerStyle.control.z != 0u ? layerStyle.localToWorld : mat4(1.0);
    const vec4 worldCenter4 = localToWorld * vec4(localCenter, 1.0);
    const vec3 worldCenter = worldCenter4.xyz / max(worldCenter4.w, 1e-6);
    const vec4 viewPosition = uniforms.view * vec4(worldCenter, 1.0);
    const vec3 viewCenter = viewPosition.xyz;
    const float depth = -viewCenter.z;

    if (depth <= 1e-5) {
        gl_Position = vec4(2.0, 2.0, 2.0, 1.0);
        outUv = vec2(0.0);
        outOpacity = 0.0;
        outWorldCenter = worldCenter;
        outSplatIndex = mergedIndex;
        outViewDepth = 0.0;
        outScaleMetric = 0.0;
        outLayerStyleIndex = layerStyleIndex;
        return;
    }

    const mat3 viewLinear = mat3(uniforms.view) * mat3(localToWorld);
    const vec3 axis0 = viewLinear * RotateVector(rotation, vec3(decodedScale.x, 0.0, 0.0));
    const vec3 axis1 = viewLinear * RotateVector(rotation, vec3(0.0, decodedScale.y, 0.0));
    const vec3 axis2 = viewLinear * RotateVector(rotation, vec3(0.0, 0.0, decodedScale.z));

    const float invDepth = 1.0 / depth;
    const float projectionX = uniforms.projection[0][0];
    const float projectionY = uniforms.projection[1][1];
    const vec2 projectedAxis0 = ProjectAxisToNdc(axis0, viewCenter, invDepth, projectionX, projectionY);
    const vec2 projectedAxis1 = ProjectAxisToNdc(axis1, viewCenter, invDepth, projectionX, projectionY);
    const vec2 projectedAxis2 = ProjectAxisToNdc(axis2, viewCenter, invDepth, projectionX, projectionY);

    float covXX =
        dot(vec3(projectedAxis0.x, projectedAxis1.x, projectedAxis2.x),
            vec3(projectedAxis0.x, projectedAxis1.x, projectedAxis2.x));
    float covXY =
        (projectedAxis0.x * projectedAxis0.y) +
        (projectedAxis1.x * projectedAxis1.y) +
        (projectedAxis2.x * projectedAxis2.y);
    float covYY =
        dot(vec3(projectedAxis0.y, projectedAxis1.y, projectedAxis2.y),
            vec3(projectedAxis0.y, projectedAxis1.y, projectedAxis2.y));

    covXX += uniforms.viewportParameters.z * uniforms.viewportParameters.z;
    covYY += uniforms.viewportParameters.w * uniforms.viewportParameters.w;

    const float trace = covXX + covYY;
    const float determinant = max((covXX * covYY) - (covXY * covXY), 1e-12);
    const float eigenOffset = sqrt(max(0.0, (trace * trace * 0.25) - determinant));
    const float lambdaMajor = max((trace * 0.5) + eigenOffset, 1e-10);
    const float lambdaMinor = max((trace * 0.5) - eigenOffset, 1e-10);

    const vec2 majorDirection = PrincipalEigenvector(covXX, covXY, covYY, lambdaMajor);
    const vec2 minorDirection = vec2(-majorDirection.y, majorDirection.x);
    const vec2 majorAxis = majorDirection * sqrt(lambdaMajor);
    const vec2 minorAxis = minorDirection * sqrt(lambdaMinor);

    const float sigmaRadius = 3.0;
    const vec2 sampleCoord = corner * sigmaRadius;
    const vec2 offsetNdc =
        ((majorAxis * sampleCoord.x) + (minorAxis * sampleCoord.y)) * footprintBoost;
    const vec4 clipCenter = uniforms.projection * vec4(viewCenter, 1.0);
    gl_Position = clipCenter + vec4(offsetNdc * clipCenter.w, 0.0, 0.0);

    outUv = sampleCoord;
    outOpacity = opacities.values[mergedIndex];
    outWorldCenter = worldCenter;
    outSplatIndex = mergedIndex;
    outViewDepth = depth;
    outScaleMetric = max(decodedScale.x, max(decodedScale.y, decodedScale.z));
    outLayerStyleIndex = layerStyleIndex;
}
