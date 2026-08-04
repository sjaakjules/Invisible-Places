#version 450
#extension GL_GOOGLE_include_directive : require

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec4 inColor;

layout(location = 0) out vec4 outSourceColor;
layout(location = 1) out float outViewDepth;
layout(location = 2) flat out uint outPointIndex;
layout(location = 3) out vec3 outWorldPosition;
layout(location = 4) out vec3 outPointNormal;
layout(location = 5) out float outFlowCoverage;
layout(location = 6) flat out vec4 outTimingColouriseTransform;
layout(location = 7) flat out float outTimingColouriseEmissionAdd;

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
    uvec4 shorelineWaveControl;
    vec4 shorelineWaveParams0;
    vec4 shorelineWaveParams1;
    vec4 shorelineWaveParams2;
    vec4 shorelineWaveParams3;
    vec4 shorelineWaveParams4;
    vec4 shorelineWaveParams5;
    vec4 shorelineWaveTint;
    vec4 gradientStartColor;
    vec4 gradientEndColor;
    uvec4 seepageControl;
    vec4 seepageGridParams;
    vec4 seepageBoundsMin;
    vec4 seepageBoundsMax;
    uvec4 rainImpactControl;
    vec4 rainImpactGrid;
    vec4 rainImpactRock0;
    vec4 rainImpactRock1;
    vec4 rainImpactVegetation0;
    vec4 rainImpactVegetation1;
    vec4 rainImpactSandBand;
    vec4 rainImpactResponse;
    uvec4 timingColouriseControl;
    uvec4 timingColouriseSources[8];
    vec4 timingColouriseRanges[8];
    vec4 timingColouriseLut[512];
    uvec4 additionalShorelineCount;
    uvec4 additionalShorelineControl[4];
    vec4 additionalShorelineParams0[4];
    vec4 additionalShorelineParams1[4];
    vec4 additionalShorelineParams2[4];
    vec4 additionalShorelineParams3[4];
    vec4 additionalShorelineParams4[4];
    vec4 additionalShorelineParams5[4];
    vec4 additionalShorelineTint[4];
} styleData;

#include "pointcloud_sparse_ripple.glsl"
#include "pointcloud_rain_impact.glsl"
#include "pointcloud_mesh_flow_contact.glsl"
#define POINTCLOUD_TIMING_COLOURISE_VERTEX
#include "pointcloud_timing_colourise.glsl"

const uint kWaterTrailRoleFieldSlot = 0u;
const uint kWaterTrailSeedFieldSlot = 5u;
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
const uint kWaterTrailWidthFieldSlot = 17u;
const uint kWaterTrailStreakLengthFieldSlot = 18u;
const uint kWaterTrailConfidenceFieldSlot = 19u;
const uint kWaterTrailFeatureTypeFieldSlot = 21u;
const uint kWaterTrailTangentZFieldSlot = 24u;
const uint kWaterTrailLaneIndexFieldSlot = 25u;
const uint kWaterTrailLaneCountFieldSlot = 26u;
const uint kWaterTrailLanePitchFieldSlot = 27u;
const uint kWaterTrailLaneSpanFieldSlot = 28u;
const uint kWaterTrailLaneCrossingFieldSlot = 29u;
const uint kWaterTrailCrossSeedFieldSlot = 30u;
const uint kWaterTrailEndpointFadeFlagsFieldSlot = 31u;
const uint kWaterTrailStartFadeFullDistanceFieldSlot = 32u;
const uint kWaterTrailStartFadeRandomBeginDistanceFieldSlot = 33u;
const uint kWaterTrailEndFadeFullDistanceFieldSlot = 34u;
const uint kWaterTrailEndFadeRandomBeginDistanceFieldSlot = 35u;

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

vec3 SafeCentripetalWaterMix(vec3 a, vec3 b, float ta, float tb, float t) {
    const float denominator = tb - ta;
    return abs(denominator) > 1e-6
        // A1 and A3 must extrapolate on the P1-P2 knot interval. Clamping
        // here degenerates the spline into its piecewise-linear control hull.
        ? mix(a, b, (t - ta) / denominator)
        : a;
}

vec3 CentripetalCatmullRomWater(vec3 p0, vec3 p1, vec3 p2, vec3 p3, float u) {
    const float t0 = 0.0;
    const float t1 = t0 + sqrt(max(length(p1 - p0), 1e-6));
    const float t2 = t1 + sqrt(max(length(p2 - p1), 1e-6));
    const float t3 = t2 + sqrt(max(length(p3 - p2), 1e-6));
    const float t = mix(t1, t2, clamp(u, 0.0, 1.0));
    const vec3 a1 = SafeCentripetalWaterMix(p0, p1, t0, t1, t);
    const vec3 a2 = SafeCentripetalWaterMix(p1, p2, t1, t2, t);
    const vec3 a3 = SafeCentripetalWaterMix(p2, p3, t2, t3, t);
    const vec3 b1 = SafeCentripetalWaterMix(a1, a2, t0, t2, t);
    const vec3 b2 = SafeCentripetalWaterMix(a2, a3, t1, t3, t);
    return SafeCentripetalWaterMix(b1, b2, t1, t2, t);
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
    const float speed =
        max(0.0, LoadScalarFieldValueForPoint(kWaterTrailSpeedFieldSlot, pointIndex)) *
        max(0.0, styleData.renderParams2.y);
    const float trailStartPhase = fract(
        baseStartPhase +
        trailAge +
        max(0.0, uniforms.depthParameters.x) * speed / routeLength);
    return trailStartPhase + trailDistance / routeLength;
}

bool WaterTrailStyleGeometryAvailable() {
    return styleData.renderParams1.x > 0.5 &&
           styleData.surfelDiameterBinding.control.w != 0u &&
           styleData.surfelDiameterBinding.control.x == 0u &&
           styleData.surfelDiameterBinding.constantValue.x > 0.0 &&
           styleData.renderParams2.z > 0.0;
}

float WaterTrailStyleWidth() {
    return max(0.0001, styleData.surfelDiameterBinding.constantValue.x);
}

float WaterFlowActivity() {
    return clamp(styleData.renderParams1.w, 0.0, 1.0);
}

float WaterFlowAppearanceScale() {
    return mix(0.30, 1.0, WaterFlowActivity());
}

float WaterTrailActivityGate() {
    // Fade every settled trail continuously. Per-trail seed variation remains
    // available to motion, but never gates an entire trail during keying.
    return WaterFlowActivity();
}

float WaterTrailBaseWidth(uint pointIndex) {
    return WaterTrailStyleGeometryAvailable()
        ? WaterTrailStyleWidth()
        : max(0.0001, LoadScalarFieldValueForPoint(kWaterTrailWidthFieldSlot, pointIndex));
}

float WaterTrailStreakLength(uint pointIndex) {
    float streakLength = 0.0;
    if (WaterTrailStyleGeometryAvailable()) {
        streakLength = WaterTrailStyleWidth() * max(1.0, styleData.renderParams2.z);
    } else {
        streakLength = max(0.001, LoadScalarFieldValueForPoint(kWaterTrailStreakLengthFieldSlot, pointIndex));
    }
    return streakLength * mix(0.55, 1.0, WaterFlowActivity());
}

float WaterTrailEndpointFadeRamp(
    float distanceFromEndpoint,
    float fullDistance,
    float randomBeginDistance,
    float randomAmount) {
    const float full = max(0.0, fullDistance);
    if (full <= 1e-5) {
        return 1.0;
    }
    const float begin = min(
        max(0.0, randomBeginDistance) * clamp(randomAmount, 0.0, 1.0),
        max(0.0, full - 1e-5));
    return smoothstep(begin, full, max(0.0, distanceFromEndpoint));
}

float WaterTrailEndpointFadeVisibility(
    uint pointIndex,
    float phase,
    float routeLength) {
    if (styleData.globalControl.z <= kWaterTrailEndFadeRandomBeginDistanceFieldSlot) {
        return 1.0;
    }
    const uint flags = uint(max(
        0.0,
        floor(LoadScalarFieldValueForPoint(
                  kWaterTrailEndpointFadeFlagsFieldSlot,
                  pointIndex) +
              0.5)));
    if (flags == 0u) {
        return 1.0;
    }
    const float trailSeed = clamp(
        LoadScalarFieldValueForPoint(kWaterTrailSeedFieldSlot, pointIndex),
        0.0,
        1.0);
    const float distanceFromStart = fract(phase) * routeLength;
    float visibility = 1.0;
    if ((flags & 1u) != 0u) {
        visibility *= WaterTrailEndpointFadeRamp(
            distanceFromStart,
            LoadScalarFieldValueForPoint(
                kWaterTrailStartFadeFullDistanceFieldSlot,
                pointIndex),
            LoadScalarFieldValueForPoint(
                kWaterTrailStartFadeRandomBeginDistanceFieldSlot,
                pointIndex),
            fract(trailSeed * 0.754877666 + 0.137));
    }
    if ((flags & 2u) != 0u) {
        visibility *= WaterTrailEndpointFadeRamp(
            routeLength - distanceFromStart,
            LoadScalarFieldValueForPoint(
                kWaterTrailEndFadeFullDistanceFieldSlot,
                pointIndex),
            LoadScalarFieldValueForPoint(
                kWaterTrailEndFadeRandomBeginDistanceFieldSlot,
                pointIndex),
            fract(trailSeed * 0.569840291 + 0.731));
    }
    return visibility;
}

float WaterTrailVisibility(uint pointIndex) {
    if (LoadScalarFieldValueForPoint(kWaterTrailRoleFieldSlot, pointIndex) < 0.5) {
        return 0.0;
    }
    const float phase = WaterTrailTravelPhase(pointIndex);
    const float routeLength = max(0.001, LoadScalarFieldValueForPoint(kWaterTrailRouteLengthFieldSlot, pointIndex));
    const float trailStreakLength = WaterTrailStreakLength(pointIndex);
    const float endFeather = clamp(trailStreakLength / routeLength, 0.001, 0.10);
    const bool meshTrail =
        abs(LoadScalarFieldValueForPoint(kWaterTrailFeatureTypeFieldSlot, pointIndex) - 5.0) < 0.5;
    // Mesh trails are continuous streams whose anchors accumulate a start
    // phase up to 1.0, so the raw phase spends most of each cycle above the
    // route end and an unwrapped fade hides the tail almost permanently.
    // The drawn position already wraps via fract; fade on the same wrapped
    // phase, with a short fade-in so the wrap does not pop at the seed.
    const float fadePhase = meshTrail ? fract(phase) : phase;
    const float routeEndFade =
        (1.0 - smoothstep(1.0 - endFeather, 1.0, fadePhase)) *
        (meshTrail ? smoothstep(0.0, endFeather * 0.5, fadePhase) : 1.0);
    const float meshTerminalFade =
        meshTrail
            ? clamp(
                  LoadScalarFieldValueForPoint(
                      kWaterTrailConfidenceFieldSlot,
                      pointIndex),
                  0.0,
                  1.0)
            : 1.0;
    return WaterTrailActivityGate() * routeEndFade * meshTerminalFade *
           WaterTrailEndpointFadeVisibility(pointIndex, phase, routeLength);
}

vec3 WaterTrailRoutePosition(uint pointIndex, float phase, vec3 fallbackPosition) {
    const uint routeStart = WaterTrailRouteStart(pointIndex);
    const uint routeCount = WaterTrailRouteCount(pointIndex);
    if (routeCount < 2u || routeStart >= styleData.pointMeta.x || routeStart + routeCount > styleData.pointMeta.x) {
        return fallbackPosition;
    }
    const float routePosition = fract(phase) * float(routeCount - 1u);
    const uint anchorOffset = min(uint(floor(routePosition)), routeCount - 2u);
    const float t = clamp(routePosition - float(anchorOffset), 0.0, 1.0);
    const vec3 p1 = pointPositions.positions[routeStart + anchorOffset].xyz;
    const vec3 p2 = pointPositions.positions[routeStart + anchorOffset + 1u].xyz;
    const vec3 p0 = anchorOffset > 0u
        ? pointPositions.positions[routeStart + anchorOffset - 1u].xyz
        : p1 + (p1 - p2);
    const vec3 p3 = anchorOffset + 2u < routeCount
        ? pointPositions.positions[routeStart + anchorOffset + 2u].xyz
        : p2 + (p2 - p1);
    return CentripetalCatmullRomWater(p0, p1, p2, p3, t);
}

vec3 WaterTrailRouteTangent(uint pointIndex, float phase) {
    const uint routeStart = WaterTrailRouteStart(pointIndex);
    const uint routeCount = WaterTrailRouteCount(pointIndex);
    if (routeCount < 2u || routeStart >= styleData.pointMeta.x || routeStart + routeCount > styleData.pointMeta.x) {
        return vec3(1.0, 0.0, 0.0);
    }
    const float routePosition = fract(phase) * float(routeCount - 1u);
    const uint anchorOffset = min(uint(floor(routePosition)), routeCount - 2u);
    const float t = clamp(routePosition - float(anchorOffset), 0.0, 1.0);
    const vec3 p1 = pointPositions.positions[routeStart + anchorOffset].xyz;
    const vec3 p2 = pointPositions.positions[routeStart + anchorOffset + 1u].xyz;
    const vec3 p0 = anchorOffset > 0u
        ? pointPositions.positions[routeStart + anchorOffset - 1u].xyz
        : p1 + (p1 - p2);
    const vec3 p3 = anchorOffset + 2u < routeCount
        ? pointPositions.positions[routeStart + anchorOffset + 2u].xyz
        : p2 + (p2 - p1);
    const float tangentProbe = 0.01;
    const vec3 tangent =
        CentripetalCatmullRomWater(p0, p1, p2, p3, min(1.0, t + tangentProbe)) -
        CentripetalCatmullRomWater(p0, p1, p2, p3, max(0.0, t - tangentProbe));
    const vec3 fallbackTangent = p2 - p1;
    return dot(tangent, tangent) > 1e-8
        ? normalize(tangent)
        : (dot(fallbackTangent, fallbackTangent) > 1e-8
            ? normalize(fallbackTangent)
            : vec3(1.0, 0.0, 0.0));
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
    const float trailSeed = clamp(
        LoadScalarFieldValueForPoint(kWaterTrailSeedFieldSlot, pointIndex),
        0.0,
        1.0);
    const float motionPhase =
        (phase * 1.70 + trailSeed * 3.17 + max(0.0, uniforms.depthParameters.x) * 0.35) *
        6.28318530718;
    const float microMotion =
        sin(motionPhase) * WaterTrailBaseWidth(pointIndex) * 0.15;
    return routePosition + lateral * (lateralOffset + microMotion);
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
    // Source-local GPU Flow owns positions in the storage buffer. In
    // particular, route-role points return their base unchanged from the
    // animation resolver and therefore cannot use the unpopulated Float3
    // compatibility vertex allocation.
    const vec3 basePosition =
        WaterTrailOverlayEnabled() && pointIndex < styleData.pointMeta.x
            ? pointPositions.positions[pointIndex].xyz
            : inPosition;
    const vec3 resolvedPosition = ResolveWaterTrailPosition(basePosition, pointIndex);
    vec4 worldPosition = vec4(resolvedPosition, 1.0);
    vec4 viewPosition = uniforms.view * worldPosition;
    gl_Position = uniforms.viewProjection * worldPosition;
    const vec3 pointNormal =
        styleData.pointMeta.z != 0u && pointIndex < styleData.pointMeta.x
            ? pointNormals.normals[pointIndex].xyz
            : vec3(0.0, 0.0, 1.0);
    const SparseRippleComposite sparseRipple =
        ResolveSparseRippleComposite(resolvedPosition, pointNormal, pointIndex, uniforms.depthParameters.x);
    const RainImpactComposite rainImpact = ResolveRainImpactComposite(resolvedPosition, pointNormal);
    const MeshFlowContactComposite meshFlowContact =
        ResolveMeshFlowContactComposite(
            resolvedPosition,
            pointNormal,
            uniforms.depthParameters.x);
    const bool worldSizedScreenSprites = styleData.renderParams2.w > 0.5;
    const float basePointSize =
        worldSizedScreenSprites
            ? WorldDiameterToScreenPointSizePixels(styleData.surfelDiameterBinding.constantValue.x, -viewPosition.z)
            : styleData.pointSizeBinding.constantValue.x;
    const float footprintScale = max(1.0e-6, styleData.renderParams1.y);
    const float flowWidthScale = WaterTrailOverlayEnabled()
        ? mix(0.65, 1.0, WaterFlowActivity())
        : 1.0;
    const float minPointSize = max(1.0, styleData.renderParams3.y);
    const float maxPointSize = max(minPointSize, styleData.renderParams3.z);
    const float meshTerminalSizeFade =
        WaterTrailOverlayEnabled() &&
                abs(
                    LoadScalarFieldValueForPoint(
                        kWaterTrailFeatureTypeFieldSlot,
                        pointIndex) -
                    5.0) <
                    0.5
            ? sqrt(
                  clamp(
                      LoadScalarFieldValueForPoint(
                          kWaterTrailConfidenceFieldSlot,
                          pointIndex),
                      0.0,
                      1.0))
            : 1.0;
    const float resolvedPointSize =
        ((basePointSize * flowWidthScale * sparseRipple.pointSizeMultiply *
          rainImpact.pointSizeMultiply *
          meshFlowContact.pointSizeMultiply) + sparseRipple.pointSizeAdd) *
        footprintScale * meshTerminalSizeFade;
    gl_PointSize = waterTrailVisibility <= 0.0
        ? 0.0
        : RippleFiniteFloat(resolvedPointSize)
              ? clamp(resolvedPointSize, minPointSize, maxPointSize)
              : minPointSize;
    outSourceColor = inColor;
    outViewDepth = -viewPosition.z;
    outPointIndex = pointIndex;
    outWorldPosition = resolvedPosition;
    outPointNormal =
        dot(pointNormal, pointNormal) > 1e-8 ? normalize(pointNormal) : vec3(0.0, 0.0, 1.0);
    outFlowCoverage = WaterTrailOverlayEnabled()
        ? waterTrailVisibility * WaterFlowAppearanceScale()
        : 1.0;
    ResolveTimingColouriseTransform(
        pointIndex,
        outTimingColouriseTransform,
        outTimingColouriseEmissionAdd);
}
