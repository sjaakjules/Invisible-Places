#version 450
#extension GL_GOOGLE_include_directive : require

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec4 inColor;

#ifndef DEPTH_PREPASS
layout(location = 0) out vec4 outSourceColor;
layout(location = 1) out float outColormapValue;
#endif
layout(location = 2) out float outOpacity;
#ifndef DEPTH_PREPASS
layout(location = 3) out float outEmissive;
#endif
layout(location = 5) out float outDepthFade;
layout(location = 6) out float outViewDepth;
layout(location = 7) flat out uint outPointIndex;
#ifndef DEPTH_PREPASS
layout(location = 8) out float outSurfaceAngleMask;
layout(location = 9) out vec3 outAovNormal;
layout(location = 11) out vec4 outWaterColourTransform;
layout(location = 12) flat out vec4 outTimingColouriseTransform;
layout(location = 13) flat out float outTimingColouriseEmissionAdd;
#endif

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

const uint kFieldMapFlagClamp = 1u;
const uint kFieldMapFlagInvert = 2u;
const uint kWaterPhaseFieldSlot = 3u;
const uint kWaterSpeedFieldSlot = 4u;
const uint kWaterWidthFieldSlot = 5u;
const uint kWaterConfidenceFieldSlot = 6u;
const uint kWaterAccumulationFieldSlot = 7u;
const uint kWaterPoolingFieldSlot = 8u;
const uint kWaterParticleRoleFieldSlot = 9u;
const uint kWaterPathStartFieldSlot = 10u;
const uint kWaterPathCountFieldSlot = 11u;
const uint kWaterJitterSeedFieldSlot = 12u;
const uint kWaterAgeFieldSlot = 13u;
const uint kWaterFeatureTypeFieldSlot = 15u;
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
const uint kWaterTrailTangentXFieldSlot = 22u;
const uint kWaterTrailTangentYFieldSlot = 23u;
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
const float kWaterParticleSpeedScale = 0.12;

float LoadScalarFieldValueForPoint(uint fieldSlot, uint pointIndex) {
    if (fieldSlot == 0xFFFFFFFFu ||
        fieldSlot >= styleData.globalControl.z ||
        styleData.pointMeta.x == 0u ||
        pointIndex >= styleData.pointMeta.x) {
        return 0.0;
    }

    const uint scalarIndex = (fieldSlot * styleData.pointMeta.x) + pointIndex;
    return scalarFieldValues.values[scalarIndex];
}

float LoadScalarFieldValue(uint fieldSlot) {
    return LoadScalarFieldValueForPoint(fieldSlot, uint(gl_VertexIndex));
}

bool HasWaterParticleFields() {
    if (styleData.pointMeta.w == 3u) {
        return styleData.globalControl.z > kWaterTrailTangentZFieldSlot;
    }
    return styleData.pointMeta.w != 0u && styleData.globalControl.z > kWaterJitterSeedFieldSlot;
}

bool WaterTrailOverlayEnabled() {
    return styleData.pointMeta.w == 3u && styleData.globalControl.z > kWaterTrailTangentZFieldSlot;
}

float WaterFeatureType(uint pointIndex) {
    if (WaterTrailOverlayEnabled()) {
        return LoadScalarFieldValueForPoint(kWaterTrailFeatureTypeFieldSlot, pointIndex);
    }
    return styleData.globalControl.z > kWaterFeatureTypeFieldSlot
        ? LoadScalarFieldValueForPoint(kWaterFeatureTypeFieldSlot, pointIndex)
        : 0.0;
}

float WaterTrailFade(uint pointIndex) {
    if (WaterTrailOverlayEnabled()) {
        return 1.0;
    }
    if (styleData.globalControl.z <= kWaterAgeFieldSlot) {
        return 1.0;
    }
    const float age = clamp(LoadScalarFieldValueForPoint(kWaterAgeFieldSlot, pointIndex), 0.0, 1.0);
    return pow(1.0 - smoothstep(0.0, 1.0, age), 1.35);
}

bool WaterPathViewEnabled() {
    return styleData.pointMeta.w == 2u;
}

float WaterPathPointSizeScale(uint pointIndex) {
    if (!WaterPathViewEnabled() || !HasWaterParticleFields()) {
        return 1.0;
    }
    const float role = LoadScalarFieldValueForPoint(kWaterParticleRoleFieldSlot, pointIndex);
    if (role >= 2.5 && role < 3.5) {
        return 0.48;
    }
    if (role >= 0.5 && role < 1.5) {
        return 0.58;
    }
    return 1.0;
}

float WaterParticleTravel(uint pointIndex) {
    const float phase = LoadScalarFieldValueForPoint(kWaterPhaseFieldSlot, pointIndex);
    const float speed = max(0.02, LoadScalarFieldValueForPoint(kWaterSpeedFieldSlot, pointIndex));
    return fract(phase + max(0.0, uniforms.depthParameters.x) * speed * kWaterParticleSpeedScale);
}

bool IsWaterSteam(uint pointIndex) {
    const float featureType = WaterFeatureType(pointIndex);
    return featureType > 0.5 && featureType < 1.5;
}

float WaterSteamFade(uint pointIndex) {
    const float travel = WaterParticleTravel(pointIndex);
    const float seed = LoadScalarFieldValueForPoint(kWaterJitterSeedFieldSlot, pointIndex);
    const float birth = smoothstep(0.0, 0.10, travel);
    const float dissipate = 1.0 - smoothstep(0.70, 1.0, travel);
    const float turbulence = 0.82 + 0.18 * sin((travel + seed * 1.618) * 6.28318530718);
    return birth * dissipate * turbulence;
}

float WaterSteamSizeScale(uint pointIndex) {
    if (!IsWaterSteam(pointIndex)) {
        return 1.0;
    }
    const float travel = WaterParticleTravel(pointIndex);
    const float seed = LoadScalarFieldValueForPoint(kWaterJitterSeedFieldSlot, pointIndex);
    return mix(0.72, 2.15 + seed * 0.35, smoothstep(0.0, 1.0, travel));
}

float HashWater01(uint value) {
    value ^= value >> 16u;
    value *= 0x7feb352du;
    value ^= value >> 15u;
    value *= 0x846ca68bu;
    value ^= value >> 16u;
    return float(value & 0x00ffffffu) / 16777215.0;
}

float SurfaceHash13(vec3 value) {
    return fract(sin(dot(value, vec3(127.1, 311.7, 74.7))) * 43758.5453123);
}

float SurfaceValueNoise(vec3 value) {
    const vec3 cell = floor(value);
    const vec3 local = fract(value);
    const vec3 blend = local * local * (3.0 - 2.0 * local);
    const float c000 = SurfaceHash13(cell + vec3(0.0, 0.0, 0.0));
    const float c100 = SurfaceHash13(cell + vec3(1.0, 0.0, 0.0));
    const float c010 = SurfaceHash13(cell + vec3(0.0, 1.0, 0.0));
    const float c110 = SurfaceHash13(cell + vec3(1.0, 1.0, 0.0));
    const float c001 = SurfaceHash13(cell + vec3(0.0, 0.0, 1.0));
    const float c101 = SurfaceHash13(cell + vec3(1.0, 0.0, 1.0));
    const float c011 = SurfaceHash13(cell + vec3(0.0, 1.0, 1.0));
    const float c111 = SurfaceHash13(cell + vec3(1.0, 1.0, 1.0));
    const float x00 = mix(c000, c100, blend.x);
    const float x10 = mix(c010, c110, blend.x);
    const float x01 = mix(c001, c101, blend.x);
    const float x11 = mix(c011, c111, blend.x);
    const float y0 = mix(x00, x10, blend.y);
    const float y1 = mix(x01, x11, blend.y);
    return mix(y0, y1, blend.z);
}

float SurfaceFbm(vec3 value) {
    float sum = 0.0;
    float amplitude = 0.5;
    float normalizer = 0.0;
    for (int octave = 0; octave < 3; ++octave) {
        sum += SurfaceValueNoise(value) * amplitude;
        normalizer += amplitude;
        value = value * 2.03 + vec3(17.1, 31.7, 11.3);
        amplitude *= 0.5;
    }
    return sum / max(0.0001, normalizer);
}

vec3 SurfaceMotionNoiseVector(vec3 position, float time) {
    return vec3(
        SurfaceFbm(position + vec3(13.1, 0.0, time)),
        SurfaceFbm(position + vec3(0.0, 29.7, time * 1.13)),
        SurfaceFbm(position + vec3(41.3, 19.1, time * 0.83))) * 2.0 - 1.0;
}

float SurfaceMotionMask(uint pointIndex) {
    if (styleData.surfaceMotionParams.x <= 1e-5) {
        return 0.0;
    }
    if (styleData.stylisationControl.z == 0xffffffffu) {
        return 1.0;
    }
    if (styleData.stylisationControl.z == 0u) {
        return 0.0;
    }

    const uint roughnessSlot = styleData.stylisationControl.z - 1u;
    const float roughness = LoadScalarFieldValueForPoint(roughnessSlot, pointIndex);
    const float roughnessNormalized =
        clamp((roughness - styleData.surfaceMotionStats.x) * styleData.surfaceMotionStats.y, 0.0, 1.0);
    float mask = smoothstep(clamp(styleData.surfaceMotionParams.w, 0.0, 1.0), 1.0, roughnessNormalized);
    if (styleData.stylisationControl.w != 0u) {
        const uint groundSlot = styleData.stylisationControl.w - 1u;
        const float groundId = LoadScalarFieldValueForPoint(groundSlot, pointIndex);
        const float distanceToTarget = abs(groundId - styleData.surfaceMotionStats.z);
        const float tolerance = max(0.001, styleData.surfaceMotionStats.w);
        mask *= 1.0 - smoothstep(tolerance, tolerance + 0.25, distanceToTarget);
    }
    return mask;
}

vec3 ResolveSurfaceMotionPosition(vec3 basePosition, uint pointIndex) {
    const float mask = SurfaceMotionMask(pointIndex);
    if (mask <= 1e-5) {
        return basePosition;
    }

    const float scale = max(0.01, styleData.surfaceMotionParams.y);
    const float speed = max(0.0, styleData.surfaceMotionParams.z);
    const float time = max(0.0, uniforms.depthParameters.x) * speed;
    const vec3 noisePosition = basePosition * scale;
    vec3 animatedNoise = SurfaceMotionNoiseVector(noisePosition, time);
    vec3 restNoise = SurfaceMotionNoiseVector(noisePosition, 0.0);
    vec3 offset = (animatedNoise - restNoise) * styleData.surfaceMotionParams.x * mask;
    offset.z *= 0.35;
    return basePosition + offset;
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

vec3 JitteredWaterAnchorPosition(
    uint pathStart,
    uint pathCount,
    uint anchorOffset,
    float particleSeed,
    float pathJitter) {
    const uint clampedOffset = min(anchorOffset, pathCount - 1u);
    const uint anchorIndex = pathStart + clampedOffset;
    const vec3 basePosition = pointPositions.positions[anchorIndex].xyz;
    if (pathJitter <= 0.0001) {
        return basePosition;
    }

    const uint prevOffset = clampedOffset > 0u ? clampedOffset - 1u : clampedOffset;
    const uint nextOffset = min(clampedOffset + 1u, pathCount - 1u);
    const vec3 prevPosition = pointPositions.positions[pathStart + prevOffset].xyz;
    const vec3 nextPosition = pointPositions.positions[pathStart + nextOffset].xyz;
    vec3 tangent = nextPosition - prevPosition;
    if (dot(tangent, tangent) <= 1e-8) {
        return basePosition;
    }

    tangent = normalize(tangent);
    const vec3 lateral = SafeWaterLateral(tangent, vec3(1.0, 0.0, 0.0));
    vec3 secondary = cross(tangent, lateral);
    if (dot(secondary, secondary) <= 1e-8) {
        secondary = vec3(0.0, 0.0, 1.0);
    }
    secondary = normalize(secondary);

    const uint seedBits = uint(clamp(particleSeed, 0.0, 1.0) * 16777215.0);
    const uint hashBase = seedBits ^ (clampedOffset * 747796405u);
    const float lateralNoise = (HashWater01(hashBase ^ 0x9e3779b9u) - 0.5) * 2.0;
    const float secondaryNoise = (HashWater01(hashBase ^ 0x85ebca6bu) - 0.5) * 2.0;
    const float startFade = smoothstep(0.0, 2.0, float(clampedOffset));
    const float endFade = smoothstep(0.0, 2.0, float((pathCount - 1u) - clampedOffset));
    const float endpointFade = min(startFade, endFade);
    const float anchorWidth = clamp(LoadScalarFieldValueForPoint(kWaterWidthFieldSlot, anchorIndex), 0.001, 100.0);
    const float amplitude = anchorWidth * clamp(pathJitter, 0.0, 3.0) * 0.45 * endpointFade;
    return basePosition + (lateral * lateralNoise + secondary * secondaryNoise * 0.22) * amplitude;
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

float WaterFlowWidthScale() {
    return mix(0.65, 1.0, WaterFlowActivity());
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

vec2 WaterTrailRouteSegment(uint pointIndex, float phase) {
    const uint routeStart = WaterTrailRouteStart(pointIndex);
    const uint routeCount = WaterTrailRouteCount(pointIndex);
    if (routeCount < 2u || routeStart >= styleData.pointMeta.x || routeStart + routeCount > styleData.pointMeta.x) {
        return vec2(0.0);
    }

    const float routePhase = fract(phase);
    const float routePosition = routePhase * float(routeCount - 1u);
    const uint anchorOffset = min(uint(floor(routePosition)), routeCount - 2u);
    return vec2(float(anchorOffset), fract(routePosition));
}

float WaterTrailVisibility(uint pointIndex, float phase) {
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

    const vec2 segment = WaterTrailRouteSegment(pointIndex, phase);
    const uint anchorOffset = min(uint(segment.x), routeCount - 2u);
    const float t = segment.y;
    const uint p1Offset = anchorOffset;
    const uint p2Offset = min(anchorOffset + 1u, routeCount - 1u);
    const vec3 p1 = pointPositions.positions[routeStart + p1Offset].xyz;
    const vec3 p2 = pointPositions.positions[routeStart + p2Offset].xyz;
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
        const vec3 tangent = vec3(
            LoadScalarFieldValueForPoint(kWaterTrailTangentXFieldSlot, pointIndex),
            LoadScalarFieldValueForPoint(kWaterTrailTangentYFieldSlot, pointIndex),
            LoadScalarFieldValueForPoint(kWaterTrailTangentZFieldSlot, pointIndex));
        return dot(tangent, tangent) > 1e-8 ? normalize(tangent) : vec3(1.0, 0.0, 0.0);
    }

    const vec2 segment = WaterTrailRouteSegment(pointIndex, phase);
    const uint anchorOffset = min(uint(segment.x), routeCount - 2u);
    const float t = segment.y;
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

    const vec2 segment = WaterTrailRouteSegment(pointIndex, phase);
    const uint anchorOffset = min(uint(segment.x), routeCount - 2u);
    const float t = segment.y;
    const uint p1Offset = anchorOffset;
    const uint p2Offset = min(anchorOffset + 1u, routeCount - 1u);
    const vec3 normal = mix(
        pointNormals.normals[routeStart + p1Offset].xyz,
        pointNormals.normals[routeStart + p2Offset].xyz,
        t);
    return dot(normal, normal) > 1e-8 ? normalize(normal) : vec3(0.0, 0.0, 1.0);
}

vec3 ResolveWaterTrailPosition(
    vec3 basePosition,
    uint pointIndex,
    float trailRole,
    float phase) {
    if (trailRole < 0.5) {
        return basePosition;
    }

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

vec3 ResolveWaterFlowPosition(
    vec3 basePosition,
    uint pointIndex,
    float trailRole,
    float trailPhase) {
    if (!HasWaterParticleFields()) {
        return basePosition;
    }
    if (WaterTrailOverlayEnabled()) {
        return ResolveWaterTrailPosition(
            basePosition,
            pointIndex,
            trailRole,
            trailPhase);
    }
    if (WaterPathViewEnabled()) {
        return basePosition;
    }

    const float role = LoadScalarFieldValueForPoint(kWaterParticleRoleFieldSlot, pointIndex);
    if (role < 0.5 || role >= 1.5) {
        return basePosition;
    }

    const uint pathStart = uint(max(0.0, floor(LoadScalarFieldValueForPoint(kWaterPathStartFieldSlot, pointIndex) + 0.5)));
    const uint pathCount = uint(max(0.0, floor(LoadScalarFieldValueForPoint(kWaterPathCountFieldSlot, pointIndex) + 0.5)));
    if (pathCount < 2u || pathStart >= styleData.pointMeta.x || pathStart + pathCount > styleData.pointMeta.x) {
        return basePosition;
    }

    const float pathPosition = WaterParticleTravel(pointIndex) * float(pathCount - 1u);
    const uint anchorOffset = min(uint(floor(pathPosition)), pathCount - 1u);
    const float t = fract(pathPosition);
    const uint p0Offset = anchorOffset > 0u ? anchorOffset - 1u : anchorOffset;
    const uint p1Offset = anchorOffset;
    const uint p2Offset = min(anchorOffset + 1u, pathCount - 1u);
    const uint p3Offset = min(anchorOffset + 2u, pathCount - 1u);
    const float seed = LoadScalarFieldValueForPoint(kWaterJitterSeedFieldSlot, pointIndex);
    const float pathJitter = clamp(LoadScalarFieldValueForPoint(kWaterWidthFieldSlot, pointIndex), 0.0, 3.0);
    const vec3 p0 = JitteredWaterAnchorPosition(pathStart, pathCount, p0Offset, seed, pathJitter);
    const vec3 p1 = JitteredWaterAnchorPosition(pathStart, pathCount, p1Offset, seed, pathJitter);
    const vec3 p2 = JitteredWaterAnchorPosition(pathStart, pathCount, p2Offset, seed, pathJitter);
    const vec3 p3 = JitteredWaterAnchorPosition(pathStart, pathCount, p3Offset, seed, pathJitter);
    return CatmullRomWater(p0, p1, p2, p3, t);
}

float EvaluateBinding(RenderParameterBindingGpu binding) {
    if (binding.control.x == 0u) {
        return binding.constantValue.x;
    }

    float normalized =
        (LoadScalarFieldValue(binding.control.y) - binding.range.x) /
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

vec2 ApplyWaterFlowAnimation(
    float opacity,
    float emissive,
    uint pointIndex,
    float trailRole,
    float trailVisibility) {
    if (styleData.pointMeta.w == 0u || styleData.globalControl.z <= kWaterSpeedFieldSlot) {
        return vec2(opacity, emissive);
    }

    if (WaterTrailOverlayEnabled()) {
        if (trailRole < 0.5) {
            return vec2(0.0);
        }
        const float appearance = WaterFlowAppearanceScale();
        return vec2(
            opacity * trailVisibility * appearance,
            emissive * trailVisibility * appearance);
    }

    if (styleData.pointMeta.w == 3u) {
        return vec2(opacity, emissive);
    }

    if (HasWaterParticleFields()) {
        const float role = LoadScalarFieldValueForPoint(kWaterParticleRoleFieldSlot, pointIndex);
        if (WaterPathViewEnabled()) {
            if (role >= 1.5 && role < 2.5) {
                const float confidence =
                    styleData.globalControl.z > kWaterConfidenceFieldSlot
                        ? clamp(LoadScalarFieldValueForPoint(kWaterConfidenceFieldSlot, pointIndex), 0.0, 1.0)
                        : 1.0;
                const float accumulation =
                    styleData.globalControl.z > kWaterAccumulationFieldSlot
                        ? clamp(LoadScalarFieldValueForPoint(kWaterAccumulationFieldSlot, pointIndex), 0.0, 1.0)
                        : 0.0;
                return vec2(opacity * (0.45 + confidence * 0.55), emissive * (0.45 + accumulation * 0.75));
            }
            if (role >= 2.5 && role < 3.5) {
                const float confidence =
                    styleData.globalControl.z > kWaterConfidenceFieldSlot
                        ? clamp(LoadScalarFieldValueForPoint(kWaterConfidenceFieldSlot, pointIndex), 0.0, 1.0)
                        : 1.0;
                const float shimmer =
                    0.72 + 0.28 * sin((LoadScalarFieldValueForPoint(kWaterJitterSeedFieldSlot, pointIndex) + uniforms.depthParameters.x * 0.12) * 6.28318530718);
                return vec2(opacity * confidence * 0.18 * shimmer, emissive * confidence * 0.22 * shimmer);
            }
            if (role >= 0.5 && role < 1.5) {
                const float trailFade = WaterTrailFade(pointIndex);
                return vec2(opacity * 0.10 * trailFade, emissive * 0.12 * trailFade);
            }
            if (role < 1.5) {
                return vec2(0.0);
            }
            return vec2(0.0);
        }
        if (role < 0.5 || role >= 1.5) {
            return vec2(0.0);
        }

        const float featureType = WaterFeatureType(pointIndex);
        const float trailFade = WaterTrailFade(pointIndex);
        if (featureType > 0.5 && featureType < 1.5) {
            const float steamFade = WaterSteamFade(pointIndex);
            const float lift = WaterParticleTravel(pointIndex);
            return vec2(
                opacity * trailFade * steamFade,
                emissive * trailFade * steamFade * (0.35 + (1.0 - lift) * 0.65));
        }
        const float travel = WaterParticleTravel(pointIndex);
        const float seed = LoadScalarFieldValueForPoint(kWaterJitterSeedFieldSlot, pointIndex);
        const float endFade = smoothstep(0.0, 0.08, travel) * (1.0 - smoothstep(0.92, 1.0, travel));
        const float shimmer = 0.78 + 0.22 * sin((travel + seed * 1.618) * 6.28318530718);
        return vec2(opacity * endFade * shimmer * trailFade, emissive * endFade * shimmer * trailFade);
    }

    const float phase = LoadScalarFieldValueForPoint(kWaterPhaseFieldSlot, pointIndex);
    const float speed = max(0.02, LoadScalarFieldValueForPoint(kWaterSpeedFieldSlot, pointIndex));
    const float confidence =
        styleData.globalControl.z > kWaterConfidenceFieldSlot
            ? clamp(LoadScalarFieldValueForPoint(kWaterConfidenceFieldSlot, pointIndex), 0.0, 1.0)
            : 1.0;
    const float accumulation =
        styleData.globalControl.z > kWaterAccumulationFieldSlot
            ? clamp(LoadScalarFieldValueForPoint(kWaterAccumulationFieldSlot, pointIndex), 0.0, 1.0)
            : 0.0;
    const float pooling =
        styleData.globalControl.z > kWaterPoolingFieldSlot
            ? clamp(LoadScalarFieldValueForPoint(kWaterPoolingFieldSlot, pointIndex), 0.0, 1.0)
            : 0.0;
    const float wave = 0.5 + (0.5 * sin((phase - uniforms.depthParameters.x * speed) * 6.28318530718));
    const float crest = smoothstep(0.58, 1.0, wave);
    const float alphaPulse = clamp((0.24 + crest * 0.82 + pooling * 0.24) * confidence, 0.0, 1.25);
    const float emissivePulse =
        clamp((0.70 + crest * 2.10 + accumulation * 1.25 + pooling * 0.55) * confidence, 0.0, 4.5);
    return vec2(opacity * alphaPulse, emissive * emissivePulse);
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

float WorldDiameterToScreenPointSizePixels(float diameterMeters, float viewDepth) {
    return max(0.0, diameterMeters) *
           abs(uniforms.projection[1][1]) *
           max(1.0, uniforms.viewportParameters.y) /
           (2.0 * max(0.001, viewDepth));
}

float ResolveSurfaceAngleMask(vec3 worldPosition, uint pointIndex) {
    if (styleData.pointMeta.z == 0u || pointIndex >= styleData.pointMeta.x) {
        return 0.0;
    }

    vec3 normal = pointNormals.normals[pointIndex].xyz;
    if (dot(normal, normal) <= 1e-8) {
        return 0.0;
    }

    normal = normalize(normal);
    const vec3 viewDirection = normalize(uniforms.cameraPosition.xyz - worldPosition);
    return clamp(1.0 - abs(dot(normal, viewDirection)), 0.0, 1.0);
}

vec3 ResolveAovNormal(uint pointIndex) {
    if (styleData.pointMeta.z == 0u || pointIndex >= styleData.pointMeta.x) {
        return vec3(0.0);
    }
    if (WaterTrailOverlayEnabled()) {
        return vec3(0.0);
    }
    if (HasWaterParticleFields()) {
        const float role = LoadScalarFieldValueForPoint(kWaterParticleRoleFieldSlot, pointIndex);
        if (role >= 0.5) {
            return vec3(0.0);
        }
    }

    vec3 normal = pointNormals.normals[pointIndex].xyz;
    if (dot(normal, normal) <= 1e-8) {
        return vec3(0.0);
    }
    return normalize(normal);
}

vec3 ApplyResolvedWaterColour(
    vec3 baseColor,
    SparseRippleComposite sparseRipple,
    RainImpactComposite rainImpact,
    MeshFlowContactComposite meshFlowContact) {
    return ApplyMeshFlowContactColour(
        ApplyRainImpactColour(
            ApplySparseRippleColor(baseColor, sparseRipple),
            rainImpact),
        meshFlowContact);
}

// Every stage above mixes with an amount that does not depend on the input
// colour, so resolving two probe colours only needs the field reads once.
// The per-stage maths must stay identical to ApplyResolvedWaterColour.
void ApplyResolvedWaterColourPair(
    vec3 baseColorA,
    vec3 baseColorB,
    SparseRippleComposite sparseRipple,
    RainImpactComposite rainImpact,
    MeshFlowContactComposite meshFlowContact,
    out vec3 resolvedA,
    out vec3 resolvedB) {
    vec3 colorA = ApplySparseRippleColor(baseColorA, sparseRipple);
    vec3 colorB = ApplySparseRippleColor(baseColorB, sparseRipple);
    colorA = ApplyRainImpactColour(colorA, rainImpact);
    colorB = ApplyRainImpactColour(colorB, rainImpact);
    resolvedA = ApplyMeshFlowContactColour(colorA, meshFlowContact);
    resolvedB = ApplyMeshFlowContactColour(colorB, meshFlowContact);
}

void main() {
    const uint pointIndex = uint(gl_VertexIndex);
    // Trail role, travel phase, and visibility feed the flow position, the
    // opacity/emissive animation, and the effect-visibility gate below.
    // Resolve them once: each re-derivation costs the same strided scalar
    // column reads.
    const bool waterTrailOverlay = WaterTrailOverlayEnabled();
    float waterTrailRole = 0.0;
    float waterTrailPhase = 0.0;
    float waterTrailVisibility = 1.0;
    if (waterTrailOverlay) {
        waterTrailRole =
            LoadScalarFieldValueForPoint(kWaterTrailRoleFieldSlot, pointIndex);
        waterTrailPhase = WaterTrailTravelPhase(pointIndex);
        waterTrailVisibility = WaterTrailVisibility(pointIndex, waterTrailPhase);
    }
    // GPU Flow writes its settled route/trail geometry directly to the vec4
    // position SSBO. Its legacy Float3 vertex allocation is intentionally not
    // read back or copied, so route-role points (which are not animated by
    // ResolveWaterFlowPosition) must also start from the storage position.
    const vec3 basePosition =
        waterTrailOverlay && pointIndex < styleData.pointMeta.x
            ? pointPositions.positions[pointIndex].xyz
            : inPosition;
    const vec3 flowPosition = ResolveWaterFlowPosition(
        basePosition,
        pointIndex,
        waterTrailRole,
        waterTrailPhase);
    vec4 worldPosition = vec4(ResolveSurfaceMotionPosition(flowPosition, pointIndex), 1.0);
    vec4 viewPosition = uniforms.view * worldPosition;
    const float viewDepth = -viewPosition.z;
    gl_Position = uniforms.viewProjection * worldPosition;
    // Point primitives are clipped by their centre, so a centre outside the
    // clip volume never rasterises and its effect resolution is
    // unobservable. Skip the whole procedural stack for those points: with
    // the camera inside the scene, a large share of the cloud sits behind
    // or beside the frustum yet runs this shader every frame. The 5% slack
    // keeps partially conformant point-clipping implementations exact at
    // the viewport edge; visible points resolve identically to before.
    const float effectCullLimit = gl_Position.w * 1.05;
    if (gl_Position.w <= 0.0 ||
        abs(gl_Position.x) > effectCullLimit ||
        abs(gl_Position.y) > effectCullLimit ||
        gl_Position.z < -0.05 * gl_Position.w ||
        gl_Position.z > effectCullLimit) {
        gl_PointSize = max(1.0, styleData.renderParams3.y);
        outOpacity = 0.0;
        outDepthFade = 0.0;
        outViewDepth = viewDepth;
        outPointIndex = pointIndex;
#ifndef DEPTH_PREPASS
        outSourceColor = inColor;
        outColormapValue = 0.0;
        outEmissive = 0.0;
        outSurfaceAngleMask = 0.0;
        outAovNormal = vec3(0.0);
        outWaterColourTransform = vec4(0.0, 0.0, 0.0, 1.0);
        outTimingColouriseTransform = vec4(0.0, 0.0, 0.0, 1.0);
        outTimingColouriseEmissionAdd = 0.0;
#endif
        return;
    }
    const vec3 aovNormal = ResolveAovNormal(pointIndex);
    const SparseRippleComposite sparseRipple =
        ResolveSparseRippleComposite(worldPosition.xyz, aovNormal, pointIndex, uniforms.depthParameters.x);
    const RainImpactComposite rainImpact = ResolveRainImpactComposite(
        worldPosition.xyz,
        aovNormal);
    const MeshFlowContactComposite meshFlowContact =
        ResolveMeshFlowContactComposite(
            worldPosition.xyz,
            aovNormal,
            uniforms.depthParameters.x);
    const float sparseRipplePointSizeAdd = sparseRipple.pointSizeAdd;
    const float sparseRipplePointSizeMultiply = sparseRipple.pointSizeMultiply;
    const float sparseRippleOpacityAdd = sparseRipple.opacityAdd;
    const float sparseRippleOpacityMultiply = sparseRipple.opacityMultiply;
    const float sparseRippleEmissionAdd = sparseRipple.emissionAdd;
    const bool worldSizedScreenSprites = styleData.renderParams2.w > 0.5;
    const float footprintScale = max(1.0e-6, styleData.renderParams1.y);
    const float flowWidthScale = WaterTrailOverlayEnabled() ? WaterFlowWidthScale() : 1.0;
    const float pointSizeBeforeDepthOfField =
        worldSizedScreenSprites
            ? WorldDiameterToScreenPointSizePixels(
                  ((max(0.0, EvaluateBinding(styleData.surfelDiameterBinding)) * flowWidthScale *
                        WaterPathPointSizeScale(pointIndex) *
                        WaterSteamSizeScale(pointIndex) *
                        sparseRipplePointSizeMultiply *
                        rainImpact.pointSizeMultiply *
                        meshFlowContact.pointSizeMultiply) +
                       sparseRipplePointSizeAdd) *
                      footprintScale,
                  viewDepth)
            : (((max(0.0, EvaluateBinding(styleData.pointSizeBinding)) * flowWidthScale *
                    WaterPathPointSizeScale(pointIndex) *
                    WaterSteamSizeScale(pointIndex) *
                    sparseRipplePointSizeMultiply *
                    rainImpact.pointSizeMultiply *
                    meshFlowContact.pointSizeMultiply) +
                   sparseRipplePointSizeAdd) *
                  footprintScale);
    const float minPointSize = max(1.0, styleData.renderParams3.y);
    const float maxPointSize = max(minPointSize, styleData.renderParams3.z);
    // Scale the authored kernel and its antialias support together. Depth of
    // field belongs to the camera and remains outside density compensation.
    const float resolvedPointSize =
        pointSizeBeforeDepthOfField +
        max(0.0, styleData.renderParams2.x) * footprintScale +
        ResolveDepthOfFieldBlurPixels(viewDepth);
    gl_PointSize = RippleFiniteFloat(resolvedPointSize)
        ? clamp(resolvedPointSize, minPointSize, maxPointSize)
        : minPointSize;

#ifndef DEPTH_PREPASS
    ResolveTimingColouriseTransform(
        pointIndex,
        outTimingColouriseTransform,
        outTimingColouriseEmissionAdd);
    if (styleData.timingColouriseControl.x != 0u) {
        vec3 waterFromZero;
        vec3 waterFromOne;
        ApplyResolvedWaterColourPair(
            vec3(0.0),
            vec3(1.0),
            sparseRipple,
            rainImpact,
            meshFlowContact,
            waterFromZero,
            waterFromOne);
        const float retainedBase = dot(
            waterFromOne - waterFromZero,
            vec3(1.0 / 3.0));
        outWaterColourTransform =
            vec4(waterFromZero, retainedBase);
        // Resolve Source RGB, Solid and Scalar Colormap through one fragment
        // path before applying timing colourise and this water transform.
        outSourceColor = inColor;
    } else {
        outWaterColourTransform = vec4(0.0, 0.0, 0.0, 1.0);
        outSourceColor = vec4(
            ApplyResolvedWaterColour(
                inColor.rgb,
                sparseRipple,
                rainImpact,
                meshFlowContact),
            inColor.a);
    }
    outColormapValue = EvaluateBinding(styleData.colormapPositionBinding);
#endif
    const vec2 animatedFlow = ApplyWaterFlowAnimation(
        EvaluateBinding(styleData.opacityBinding),
        EvaluateBinding(styleData.emissiveBinding),
        pointIndex,
        waterTrailRole,
        waterTrailVisibility);
    const float safeBaseOpacity = RippleFiniteFloat(animatedFlow.x)
        ? clamp(animatedFlow.x, 0.0, 4.0)
        : 1.0;
#ifndef DEPTH_PREPASS
    const float safeBaseEmissive = RippleFiniteFloat(animatedFlow.y)
        ? max(0.0, animatedFlow.y)
        : 0.0;
#endif
    const float flowEffectVisibility = waterTrailOverlay
        ? waterTrailVisibility * WaterFlowAppearanceScale()
        : 1.0;
    const float resolvedOpacity =
        (animatedFlow.x * sparseRippleOpacityMultiply) +
            (sparseRippleOpacityAdd +
             rainImpact.opacityAdd +
             meshFlowContact.opacityAdd) * flowEffectVisibility;
    outOpacity = RippleFiniteFloat(resolvedOpacity)
        ? clamp(resolvedOpacity, 0.0, 4.0)
        : safeBaseOpacity;
#ifndef DEPTH_PREPASS
    const float resolvedEmissive =
        animatedFlow.y +
        (sparseRippleEmissionAdd +
         rainImpact.emissionAdd +
         meshFlowContact.emissionAdd) * flowEffectVisibility;
    outEmissive = RippleFiniteFloat(resolvedEmissive)
        ? max(0.0, resolvedEmissive)
        : safeBaseEmissive;
#endif
    outDepthFade = EvaluateBinding(styleData.depthFadeBinding);
    outViewDepth = viewDepth;
    outPointIndex = pointIndex;
#ifndef DEPTH_PREPASS
    outSurfaceAngleMask = ResolveSurfaceAngleMask(worldPosition.xyz, pointIndex);
    outAovNormal = aovNormal;
#endif
}
