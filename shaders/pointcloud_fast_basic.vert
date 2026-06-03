#version 450
#extension GL_GOOGLE_include_directive : require

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

layout(set = 0, binding = 1, std430) readonly buffer ScalarFieldValues {
    float values[];
} scalarFieldValues;

layout(set = 0, binding = 4, std430) readonly buffer PointPositions {
    vec4 positions[];
} pointPositions;

layout(set = 0, binding = 6, std430) readonly buffer PointNormals {
    vec4 normals[];
} pointNormals;

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
    vec4 surfaceMotionParams;
    vec4 surfaceMotionStats;
    uvec4 causticControl;
    vec4 causticParams0;
    vec4 causticParams1;
    vec4 causticParams2;
    vec4 causticTint;
    uvec4 waterEffectControl;
    uvec4 waterEffectSlots0;
    uvec4 waterEffectSlots1;
    uvec4 rippleEffectSlots0;
    uvec4 rippleEffectSlots1;
    uvec4 rippleEffectSlots2;
    uvec4 rippleEffectSlots3;
} styleData;

#include "pointcloud_sparse_ripple.glsl"

const uint kWaterTrailRoleFieldSlot = 0u;
const uint kWaterTrailDistanceFieldSlot = 7u;
const uint kWaterTrailLengthFieldSlot = 8u;
const uint kWaterTrailRouteStartFieldSlot = 9u;
const uint kWaterTrailRouteCountFieldSlot = 10u;
const uint kWaterTrailRouteLengthFieldSlot = 11u;
const uint kWaterTrailStartPhaseFieldSlot = 12u;
const uint kWaterTrailLateralOffsetFieldSlot = 13u;
const uint kWaterTrailPointAgeFieldSlot = 14u;
const uint kWaterTrailAgeFieldSlot = 15u;
const uint kWaterTrailSpeedFieldSlot = 16u;
const uint kWaterTrailStreakLengthFieldSlot = 18u;
const uint kWaterTrailTangentZFieldSlot = 24u;
const uint kWaterTrailLaneIndexFieldSlot = 25u;
const uint kWaterTrailLaneCountFieldSlot = 26u;
const uint kWaterTrailLanePitchFieldSlot = 27u;
const uint kWaterTrailLaneSpanFieldSlot = 28u;
const uint kWaterTrailLaneCrossingFieldSlot = 29u;
const uint kWaterTrailCrossSeedFieldSlot = 30u;

float LoadScalarFieldValueForPoint(uint fieldSlot, uint pointIndex) {
    if (fieldSlot == 0xFFFFFFFFu ||
        fieldSlot >= styleData.globalControl.z ||
        styleData.pointMeta.x == 0u ||
        pointIndex >= styleData.pointMeta.x) {
        return 0.0;
    }
    return scalarFieldValues.values[(fieldSlot * styleData.pointMeta.x) + pointIndex];
}

bool WaterTrailOverlayEnabled() {
    return styleData.pointMeta.w == 3u && styleData.globalControl.z > kWaterTrailTangentZFieldSlot;
}

vec3 SafeWaterLateral(vec3 tangent, vec3 fallback) {
    vec3 lateral = cross(tangent, vec3(0.0, 0.0, 1.0));
    if (dot(lateral, lateral) <= 1e-8) {
        lateral = cross(tangent, vec3(0.0, 1.0, 0.0));
    }
    if (dot(lateral, lateral) <= 1e-8) {
        lateral = fallback;
    }
    return normalize(lateral);
}

vec3 CatmullRomWater(vec3 p0, vec3 p1, vec3 p2, vec3 p3, float t) {
    const float t2 = t * t;
    const float t3 = t2 * t;
    return 0.5 * (
        (2.0 * p1) +
        (-p0 + p2) * t +
        ((2.0 * p0) - (5.0 * p1) + (4.0 * p2) - p3) * t2 +
        (-p0 + (3.0 * p1) - (3.0 * p2) + p3) * t3);
}

uint WaterTrailRouteStart(uint pointIndex) {
    return uint(max(0.0, floor(LoadScalarFieldValueForPoint(kWaterTrailRouteStartFieldSlot, pointIndex) + 0.5)));
}

uint WaterTrailRouteCount(uint pointIndex) {
    return uint(max(0.0, floor(LoadScalarFieldValueForPoint(kWaterTrailRouteCountFieldSlot, pointIndex) + 0.5)));
}

float WaterTrailTravelPhase(uint pointIndex) {
    const float routeLength = max(0.001, LoadScalarFieldValueForPoint(kWaterTrailRouteLengthFieldSlot, pointIndex));
    const float trailDistance = max(0.0, LoadScalarFieldValueForPoint(kWaterTrailDistanceFieldSlot, pointIndex));
    const float trailAge = LoadScalarFieldValueForPoint(kWaterTrailAgeFieldSlot, pointIndex);
    const float baseStartPhase = LoadScalarFieldValueForPoint(kWaterTrailStartPhaseFieldSlot, pointIndex);
    const float speed = max(0.0, LoadScalarFieldValueForPoint(kWaterTrailSpeedFieldSlot, pointIndex));
    const float trailStartPhase = fract(
        baseStartPhase +
        trailAge +
        max(0.0, uniforms.depthParameters.x) * speed / routeLength);
    return trailStartPhase + trailDistance / routeLength;
}

float WaterTrailVisibility(uint pointIndex) {
    if (LoadScalarFieldValueForPoint(kWaterTrailRoleFieldSlot, pointIndex) < 0.5) {
        return 0.0;
    }
    const float phase = WaterTrailTravelPhase(pointIndex);
    const float routeLength = max(0.001, LoadScalarFieldValueForPoint(kWaterTrailRouteLengthFieldSlot, pointIndex));
    const float trailStreakLength = max(0.001, LoadScalarFieldValueForPoint(kWaterTrailStreakLengthFieldSlot, pointIndex));
    const float endFeather = clamp(trailStreakLength / routeLength, 0.001, 0.08);
    return 1.0 - smoothstep(1.0 - endFeather, 1.0, phase);
}

vec3 WaterTrailRoutePosition(uint pointIndex, float phase, vec3 fallbackPosition) {
    const uint routeStart = WaterTrailRouteStart(pointIndex);
    const uint routeCount = WaterTrailRouteCount(pointIndex);
    if (routeCount < 2u || routeStart >= styleData.pointMeta.x || routeStart + routeCount > styleData.pointMeta.x) {
        return fallbackPosition;
    }
    const float routePosition = fract(phase) * float(routeCount - 1u);
    const uint anchorOffset = min(uint(floor(routePosition)), routeCount - 1u);
    const float t = fract(routePosition);
    const uint p0Offset = anchorOffset > 0u ? anchorOffset - 1u : anchorOffset;
    const uint p1Offset = anchorOffset;
    const uint p2Offset = min(anchorOffset + 1u, routeCount - 1u);
    const uint p3Offset = min(anchorOffset + 2u, routeCount - 1u);
    return CatmullRomWater(
        pointPositions.positions[routeStart + p0Offset].xyz,
        pointPositions.positions[routeStart + p1Offset].xyz,
        pointPositions.positions[routeStart + p2Offset].xyz,
        pointPositions.positions[routeStart + p3Offset].xyz,
        t);
}

vec3 WaterTrailRouteTangent(uint pointIndex, float phase) {
    const uint routeStart = WaterTrailRouteStart(pointIndex);
    const uint routeCount = WaterTrailRouteCount(pointIndex);
    if (routeCount < 2u || routeStart >= styleData.pointMeta.x || routeStart + routeCount > styleData.pointMeta.x) {
        return vec3(1.0, 0.0, 0.0);
    }
    const float routePosition = fract(phase) * float(routeCount - 1u);
    const uint anchorOffset = min(uint(floor(routePosition)), routeCount - 1u);
    const uint prevOffset = anchorOffset > 0u ? anchorOffset - 1u : anchorOffset;
    const uint nextOffset = min(anchorOffset + 1u, routeCount - 1u);
    const vec3 tangent =
        pointPositions.positions[routeStart + nextOffset].xyz -
        pointPositions.positions[routeStart + prevOffset].xyz;
    return dot(tangent, tangent) > 1e-8 ? normalize(tangent) : vec3(1.0, 0.0, 0.0);
}

vec3 WaterTrailRouteNormal(uint pointIndex, float phase) {
    if (styleData.pointMeta.z == 0u) {
        return vec3(0.0, 0.0, 1.0);
    }
    const uint routeStart = WaterTrailRouteStart(pointIndex);
    const uint routeCount = WaterTrailRouteCount(pointIndex);
    if (routeCount < 2u || routeStart >= styleData.pointMeta.x || routeStart + routeCount > styleData.pointMeta.x) {
        const vec3 normal = pointNormals.normals[pointIndex].xyz;
        return dot(normal, normal) > 1e-8 ? normalize(normal) : vec3(0.0, 0.0, 1.0);
    }
    const float routePosition = fract(phase) * float(routeCount - 1u);
    const uint anchorOffset = min(uint(floor(routePosition)), routeCount - 1u);
    const float t = fract(routePosition);
    const uint p1Offset = anchorOffset;
    const uint p2Offset = min(anchorOffset + 1u, routeCount - 1u);
    const vec3 normal = mix(
        pointNormals.normals[routeStart + p1Offset].xyz,
        pointNormals.normals[routeStart + p2Offset].xyz,
        t);
    return dot(normal, normal) > 1e-8 ? normalize(normal) : vec3(0.0, 0.0, 1.0);
}

vec3 ResolveWaterTrailPosition(vec3 basePosition, uint pointIndex) {
    if (!WaterTrailOverlayEnabled() ||
        LoadScalarFieldValueForPoint(kWaterTrailRoleFieldSlot, pointIndex) < 0.5) {
        return basePosition;
    }
    const float phase = WaterTrailTravelPhase(pointIndex);
    const vec3 routePosition = WaterTrailRoutePosition(pointIndex, phase, basePosition);
    const vec3 routeTangent = WaterTrailRouteTangent(pointIndex, phase);
    const vec3 routeNormal = WaterTrailRouteNormal(pointIndex, phase);
    vec3 lateral = cross(routeNormal, routeTangent);
    if (dot(lateral, lateral) <= 1e-8) {
        lateral = SafeWaterLateral(routeTangent, vec3(1.0, 0.0, 0.0));
    } else {
        lateral = normalize(lateral);
    }
    const float lateralOffset = LoadScalarFieldValueForPoint(kWaterTrailLateralOffsetFieldSlot, pointIndex);
    return routePosition + lateral * lateralOffset;
}

float WorldDiameterToScreenPointSizePixels(float diameterMeters, float viewDepth) {
    return max(0.0, diameterMeters) *
           abs(uniforms.projection[1][1]) *
           max(1.0, uniforms.viewportParameters.y) /
           (2.0 * max(0.001, viewDepth));
}

void main() {
    const uint pointIndex = uint(gl_VertexIndex);
    const float waterTrailVisibility = WaterTrailOverlayEnabled()
        ? WaterTrailVisibility(pointIndex)
        : 1.0;
    const vec3 resolvedPosition = ResolveWaterTrailPosition(inPosition, pointIndex);
    vec4 worldPosition = vec4(resolvedPosition, 1.0);
    vec4 viewPosition = uniforms.view * worldPosition;
    gl_Position = uniforms.viewProjection * worldPosition;
    const vec3 pointNormal =
        styleData.pointMeta.z != 0u && pointIndex < styleData.pointMeta.x
            ? pointNormals.normals[pointIndex].xyz
            : vec3(0.0, 0.0, 1.0);
    const SparseRippleComposite sparseRipple =
        ResolveSparseRippleComposite(resolvedPosition, pointNormal, pointIndex, uniforms.depthParameters.x);
    const bool worldSizedScreenSprites = styleData.renderParams2.w > 0.5;
    const float basePointSize =
        worldSizedScreenSprites
            ? WorldDiameterToScreenPointSizePixels(styleData.surfelDiameterBinding.constantValue.x, -viewPosition.z)
            : styleData.pointSizeBinding.constantValue.x;
    gl_PointSize = waterTrailVisibility <= 0.0
        ? 0.0
        : clamp(
              (basePointSize * sparseRipple.pointSizeMultiply) + sparseRipple.pointSizeAdd,
              max(1.0, styleData.renderParams3.y),
              max(max(1.0, styleData.renderParams3.y), styleData.renderParams3.z));
    outSourceColor = inColor;
    outViewDepth = -viewPosition.z;
    outPointIndex = pointIndex;
    outWorldPosition = resolvedPosition;
    outPointNormal =
        dot(pointNormal, pointNormal) > 1e-8 ? normalize(pointNormal) : vec3(0.0, 0.0, 1.0);
}
