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

const uint kWaterStreamRoleFieldSlot = 0u;
const uint kWaterStreamLengthFieldSlot = 8u;
const uint kWaterStreamRouteStartFieldSlot = 9u;
const uint kWaterStreamRouteCountFieldSlot = 10u;
const uint kWaterStreamRouteLengthFieldSlot = 11u;
const uint kWaterStreamStartPhaseFieldSlot = 12u;
const uint kWaterStreamLateralOffsetFieldSlot = 13u;
const uint kWaterStreamPointAgeFieldSlot = 14u;
const uint kWaterStreamAgeFieldSlot = 15u;
const uint kWaterStreamSpeedFieldSlot = 16u;
const uint kWaterStreamTangentZFieldSlot = 24u;
const uint kWaterStreamLaneIndexFieldSlot = 25u;
const uint kWaterStreamLaneCountFieldSlot = 26u;
const uint kWaterStreamLanePitchFieldSlot = 27u;
const uint kWaterStreamLaneSpanFieldSlot = 28u;
const uint kWaterStreamLaneCrossingFieldSlot = 29u;
const uint kWaterStreamCrossSeedFieldSlot = 30u;

float LoadScalarFieldValueForPoint(uint fieldSlot, uint pointIndex) {
    if (fieldSlot == 0xFFFFFFFFu ||
        fieldSlot >= styleData.globalControl.z ||
        styleData.pointMeta.x == 0u ||
        pointIndex >= styleData.pointMeta.x) {
        return 0.0;
    }
    return scalarFieldValues.values[(fieldSlot * styleData.pointMeta.x) + pointIndex];
}

bool WaterStreamOverlayEnabled() {
    return styleData.pointMeta.w == 3u && styleData.globalControl.z > kWaterStreamTangentZFieldSlot;
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

uint WaterStreamRouteStart(uint pointIndex) {
    return uint(max(0.0, floor(LoadScalarFieldValueForPoint(kWaterStreamRouteStartFieldSlot, pointIndex) + 0.5)));
}

uint WaterStreamRouteCount(uint pointIndex) {
    return uint(max(0.0, floor(LoadScalarFieldValueForPoint(kWaterStreamRouteCountFieldSlot, pointIndex) + 0.5)));
}

float WaterStreamTravelPhase(uint pointIndex) {
    const float routeLength = max(0.001, LoadScalarFieldValueForPoint(kWaterStreamRouteLengthFieldSlot, pointIndex));
    const float streamLength = max(0.0, LoadScalarFieldValueForPoint(kWaterStreamLengthFieldSlot, pointIndex));
    const float pointAge = clamp(LoadScalarFieldValueForPoint(kWaterStreamPointAgeFieldSlot, pointIndex), 0.0, 1.0);
    const float streamAge = LoadScalarFieldValueForPoint(kWaterStreamAgeFieldSlot, pointIndex);
    const float streamStartPhase = LoadScalarFieldValueForPoint(kWaterStreamStartPhaseFieldSlot, pointIndex);
    const float speed = max(0.0, LoadScalarFieldValueForPoint(kWaterStreamSpeedFieldSlot, pointIndex));
    return fract(
        streamStartPhase +
        streamAge +
        max(0.0, uniforms.depthParameters.x) * speed / routeLength -
        pointAge * streamLength / routeLength);
}

float WaterStreamHash(float a, float b, float c) {
    return fract(sin(dot(vec3(a, b, c), vec3(12.9898, 78.233, 37.719))) * 43758.5453123);
}

float WaterStreamLaneCenter(float laneIndex, float laneCount, float laneSpan) {
    const float count = max(1.0, floor(laneCount + 0.5));
    if (count <= 1.0 || laneSpan <= 0.00001) {
        return 0.0;
    }
    const float clampedIndex = clamp(laneIndex, 0.0, count - 1.0);
    return (((clampedIndex + 0.5) / count) - 0.5) * laneSpan;
}

float WaterStreamRouteTurnBias(uint pointIndex, float travelPhase, vec3 routeNormal) {
    const uint routeStart = WaterStreamRouteStart(pointIndex);
    const uint routeCount = WaterStreamRouteCount(pointIndex);
    if (routeCount < 3u || routeStart >= styleData.pointMeta.x || routeStart + routeCount > styleData.pointMeta.x) {
        return 0.0;
    }

    const float routePosition = fract(travelPhase) * float(routeCount - 1u);
    const uint centerOffset = min(max(uint(floor(routePosition)), 1u), routeCount - 2u);
    const vec3 previous = pointPositions.positions[routeStart + centerOffset - 1u].xyz;
    const vec3 center = pointPositions.positions[routeStart + centerOffset].xyz;
    const vec3 next = pointPositions.positions[routeStart + centerOffset + 1u].xyz;
    vec3 previousTangent = center - previous;
    vec3 nextTangent = next - center;
    if (dot(previousTangent, previousTangent) <= 1e-8 || dot(nextTangent, nextTangent) <= 1e-8) {
        return 0.0;
    }

    previousTangent = normalize(previousTangent);
    nextTangent = normalize(nextTangent);
    const float signedTurn = dot(cross(previousTangent, nextTangent), routeNormal);
    return clamp(-signedTurn * 8.0, -1.0, 1.0);
}

float WaterStreamApplyLaneJump(
    float currentLane,
    float laneCount,
    float jumpChance,
    float turnBias,
    float crossSeed,
    float segmentIndex) {
    if (WaterStreamHash(crossSeed, segmentIndex, 17.0) >= jumpChance) {
        return currentLane;
    }

    const float outerLaneProbability = clamp(0.5 + turnBias * 0.42, 0.08, 0.92);
    const float direction =
        WaterStreamHash(crossSeed, segmentIndex, 29.0) < outerLaneProbability ? 1.0 : -1.0;
    return clamp(currentLane + direction, 0.0, laneCount - 1.0);
}

float ResolveWaterStreamLateralOffset(
    uint pointIndex,
    float travelPhase,
    vec3 routeTangent,
    float turnBias) {
    const float baseOffset = LoadScalarFieldValueForPoint(kWaterStreamLateralOffsetFieldSlot, pointIndex);
    if (styleData.globalControl.z <= kWaterStreamCrossSeedFieldSlot) {
        return baseOffset;
    }

    const float crossing = clamp(LoadScalarFieldValueForPoint(kWaterStreamLaneCrossingFieldSlot, pointIndex), 0.0, 1.0);
    const float laneCount = max(1.0, floor(LoadScalarFieldValueForPoint(kWaterStreamLaneCountFieldSlot, pointIndex) + 0.5));
    const float lanePitch = max(0.0, LoadScalarFieldValueForPoint(kWaterStreamLanePitchFieldSlot, pointIndex));
    const float laneSpan = max(0.0, LoadScalarFieldValueForPoint(kWaterStreamLaneSpanFieldSlot, pointIndex));
    if (crossing <= 0.0001 || laneCount <= 1.0 || lanePitch <= 0.0 || laneSpan <= 0.00001) {
        return baseOffset;
    }

    const float baseLane = clamp(
        floor(LoadScalarFieldValueForPoint(kWaterStreamLaneIndexFieldSlot, pointIndex) + 0.5),
        0.0,
        laneCount - 1.0);
    const float baseCenter = WaterStreamLaneCenter(baseLane, laneCount, laneSpan);
    const float offsetJitter = baseOffset - baseCenter;
    const float crossSeed = LoadScalarFieldValueForPoint(kWaterStreamCrossSeedFieldSlot, pointIndex);
    const float routeProgress = fract(travelPhase);
    const float sourceProgress = smoothstep(0.03, 0.55, routeProgress);
    const float flatness = 1.0 - smoothstep(0.05, 0.45, abs(routeTangent.z));
    const float jumpChance = clamp(
        crossing *
            sourceProgress *
            mix(0.30, 1.45, flatness) *
            (1.0 + abs(turnBias) * 0.75),
        0.0,
        1.0);
    const float segmentCount = mix(2.0, 12.0, crossing);
    const float segmentCoord = routeProgress * segmentCount;
    const float segmentIndex = floor(segmentCoord);
    const float localPhase = fract(segmentCoord);
    float currentLane = baseLane;
    for (int segment = 0; segment < 12; ++segment) {
        const float segmentValue = float(segment);
        if (segmentValue >= segmentIndex || segmentValue >= segmentCount) {
            break;
        }
        currentLane = WaterStreamApplyLaneJump(
            currentLane,
            laneCount,
            jumpChance,
            turnBias,
            crossSeed,
            segmentValue);
    }

    const float targetLane = WaterStreamApplyLaneJump(
        currentLane,
        laneCount,
        jumpChance,
        turnBias,
        crossSeed,
        segmentIndex);
    const float envelope = smoothstep(0.18, 0.92, localPhase);
    const float resolvedLane = mix(currentLane, targetLane, envelope);
    return WaterStreamLaneCenter(resolvedLane, laneCount, laneSpan) + offsetJitter;
}

vec3 WaterStreamRoutePosition(uint pointIndex, float phase, vec3 fallbackPosition) {
    const uint routeStart = WaterStreamRouteStart(pointIndex);
    const uint routeCount = WaterStreamRouteCount(pointIndex);
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

vec3 WaterStreamRouteTangent(uint pointIndex, float phase) {
    const uint routeStart = WaterStreamRouteStart(pointIndex);
    const uint routeCount = WaterStreamRouteCount(pointIndex);
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

vec3 WaterStreamRouteNormal(uint pointIndex, float phase) {
    if (styleData.pointMeta.z == 0u) {
        return vec3(0.0, 0.0, 1.0);
    }
    const uint routeStart = WaterStreamRouteStart(pointIndex);
    const uint routeCount = WaterStreamRouteCount(pointIndex);
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

vec3 ResolveWaterStreamPosition(vec3 basePosition, uint pointIndex) {
    if (!WaterStreamOverlayEnabled() ||
        LoadScalarFieldValueForPoint(kWaterStreamRoleFieldSlot, pointIndex) < 0.5) {
        return basePosition;
    }
    const float phase = WaterStreamTravelPhase(pointIndex);
    const vec3 routePosition = WaterStreamRoutePosition(pointIndex, phase, basePosition);
    const vec3 routeTangent = WaterStreamRouteTangent(pointIndex, phase);
    const vec3 routeNormal = WaterStreamRouteNormal(pointIndex, phase);
    vec3 lateral = cross(routeNormal, routeTangent);
    if (dot(lateral, lateral) <= 1e-8) {
        lateral = SafeWaterLateral(routeTangent, vec3(1.0, 0.0, 0.0));
    } else {
        lateral = normalize(lateral);
    }
    const float turnBias = WaterStreamRouteTurnBias(pointIndex, phase, routeNormal);
    return routePosition + lateral * ResolveWaterStreamLateralOffset(pointIndex, phase, routeTangent, turnBias);
}

float WorldDiameterToScreenPointSizePixels(float diameterMeters, float viewDepth) {
    return max(0.0, diameterMeters) *
           abs(uniforms.projection[1][1]) *
           max(1.0, uniforms.viewportParameters.y) /
           (2.0 * max(0.001, viewDepth));
}

void main() {
    const uint pointIndex = uint(gl_VertexIndex);
    const vec3 resolvedPosition = ResolveWaterStreamPosition(inPosition, pointIndex);
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
    gl_PointSize = clamp(
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
