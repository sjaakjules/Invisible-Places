#version 450
#extension GL_GOOGLE_include_directive : require

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec4 inColor;

layout(location = 0) out vec4 outSourceColor;
layout(location = 1) out float outColormapValue;
layout(location = 2) out float outOpacity;
layout(location = 3) out float outEmissive;
layout(location = 4) out float outXray;
layout(location = 5) out float outDepthFade;
layout(location = 6) out float outViewDepth;
layout(location = 7) flat out uint outPointIndex;
layout(location = 8) out float outSurfaceAngleMask;
layout(location = 9) out vec3 outAovNormal;
layout(location = 10) out float outCaustic;

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

#include "pointcloud_caustics.glsl"

float ResolveCausticStrength(vec3 worldPosition, uint pointIndex, out float previewTint) {
    previewTint = 0.0;
    if (styleData.causticControl.x == 0u ||
        styleData.causticControl.y == 0u ||
        styleData.causticControl.z == 0u ||
        styleData.causticControl.w == 0u) {
        return 0.0;
    }
    const uint maskSlot = styleData.causticControl.y - 1u;
    const uint edgeSlot = styleData.causticControl.z - 1u;
    const uint seedSlot = styleData.causticControl.w - 1u;
    const float mask = clamp(LoadScalarFieldValueForPoint(maskSlot, pointIndex), 0.0, 1.0);
    if (mask <= 1e-5) {
        return 0.0;
    }
    const float edge = clamp(LoadScalarFieldValueForPoint(edgeSlot, pointIndex), 0.0, 1.0);
    const float seed = LoadScalarFieldValueForPoint(seedSlot, pointIndex);
    previewTint = CausticPreviewTint(mask, edge, seed);
    const float time = uniforms.depthParameters.x * max(0.0, styleData.causticParams0.z);
    const vec3 normal =
        styleData.pointMeta.z != 0u && pointIndex < styleData.pointMeta.x
            ? pointNormals.normals[pointIndex].xyz
            : vec3(0.0);
    const vec2 metersUv = CausticSurfaceUv(worldPosition, normal);
    const float ridge = CausticVoronoiRidge(metersUv, seed, time, edge);
    const float edgeGate = CausticEdgeGate(metersUv, edge, seed);
    return clamp(ridge * mask * edgeGate * max(0.0, styleData.causticParams0.x), 0.0, 6.0);
}

bool HasWaterEffectComposition() {
    return styleData.waterEffectControl.x != 0u &&
           styleData.waterEffectControl.y != 0u &&
           styleData.waterEffectControl.z != 0u &&
           styleData.waterEffectControl.w != 0u &&
           styleData.waterEffectSlots0.x != 0u &&
           styleData.waterEffectSlots0.y != 0u &&
           styleData.waterEffectSlots0.z != 0u &&
           styleData.waterEffectSlots0.w != 0u &&
           styleData.waterEffectSlots1.x != 0u &&
           styleData.waterEffectSlots1.y != 0u;
}

float WaterEffectField(uint slotPlusOne, uint pointIndex, float fallback) {
    if (!HasWaterEffectComposition() || slotPlusOne == 0u) {
        return fallback;
    }
    return LoadScalarFieldValueForPoint(slotPlusOne - 1u, pointIndex);
}

bool HasRippleEffectFields() {
    return styleData.rippleEffectSlots0.x != 0u &&
           styleData.rippleEffectSlots0.y != 0u &&
           styleData.rippleEffectSlots0.z != 0u &&
           styleData.rippleEffectSlots0.w != 0u &&
           styleData.rippleEffectSlots1.x != 0u &&
           styleData.rippleEffectSlots1.y != 0u &&
           styleData.rippleEffectSlots1.w != 0u &&
           styleData.rippleEffectSlots2.x != 0u &&
           styleData.rippleEffectSlots2.y != 0u &&
           styleData.rippleEffectSlots2.z != 0u &&
           styleData.rippleEffectSlots2.w != 0u;
}

float RippleEffectField(uint slotPlusOne, uint pointIndex, float fallback) {
    if (!HasRippleEffectFields() || slotPlusOne == 0u) {
        return fallback;
    }
    return LoadScalarFieldValueForPoint(slotPlusOne - 1u, pointIndex);
}

float ResolveRippleEffectScale(uint pointIndex) {
    if (!HasRippleEffectFields()) {
        return 1.0;
    }
    const float mask = clamp(RippleEffectField(styleData.rippleEffectSlots0.x, pointIndex, 0.0), 0.0, 1.0);
    if (mask <= 1e-5) {
        return 1.0;
    }
    const float edge = clamp(RippleEffectField(styleData.rippleEffectSlots0.y, pointIndex, 0.0), 0.0, 1.0);
    const float value = clamp(RippleEffectField(styleData.rippleEffectSlots0.z, pointIndex, 0.0), 0.0, 1.0);
    const float seed = RippleEffectField(styleData.rippleEffectSlots0.w, pointIndex, 0.0);
    const float distance = RippleEffectField(styleData.rippleEffectSlots1.x, pointIndex, 0.0);
    const float linearCoord = RippleEffectField(styleData.rippleEffectSlots1.y, pointIndex, 0.0);
    const float speed = max(0.0, RippleEffectField(styleData.rippleEffectSlots1.w, pointIndex, 0.0));
    const float confidence = clamp(RippleEffectField(styleData.rippleEffectSlots2.x, pointIndex, 0.0), 0.0, 1.0);
    const float wavelength = max(0.005, RippleEffectField(styleData.rippleEffectSlots2.y, pointIndex, 0.25));
    const float warp = max(0.0, RippleEffectField(styleData.rippleEffectSlots2.z, pointIndex, 0.0));
    const float phaseOffset = RippleEffectField(styleData.rippleEffectSlots2.w, pointIndex, 0.0);
    const float time = max(0.0, uniforms.depthParameters.x);
    const float ripplePhase =
        (linearCoord / wavelength) -
        (time * speed) +
        phaseOffset +
        (seed * 0.173);
    const float warpPhase =
        sin(((distance / wavelength) + time * 0.37 + seed) * 6.28318530718) * warp;
    const float wave = 0.5 + 0.5 * cos((ripplePhase + warpPhase) * 6.28318530718);
    const float crest = smoothstep(0.42, 1.0, wave);
    return clamp(value * mask * edge * confidence * (0.18 + 0.82 * crest), 0.0, 1.0);
}

vec3 ApplyWaterEffectColor(vec3 baseColor, uint pointIndex, float waterEffectScale) {
    const float mixAmount =
        clamp(WaterEffectField(styleData.waterEffectSlots0.z, pointIndex, 0.0) * waterEffectScale, 0.0, 1.0);
    if (mixAmount <= 1e-5) {
        return baseColor;
    }
    const vec3 effectColor = vec3(
        clamp(WaterEffectField(styleData.waterEffectSlots0.w, pointIndex, 0.62), 0.0, 1.0),
        clamp(WaterEffectField(styleData.waterEffectSlots1.x, pointIndex, 0.88), 0.0, 1.0),
        clamp(WaterEffectField(styleData.waterEffectSlots1.y, pointIndex, 1.0), 0.0, 1.0));
    return mix(baseColor, effectColor, mixAmount);
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
    if (styleData.stylisationControl.z == 0u || styleData.surfaceMotionParams.x <= 1e-5) {
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
    const float speed = max(0.0, LoadScalarFieldValueForPoint(kWaterTrailSpeedFieldSlot, pointIndex));
    const float trailStartPhase = fract(
        baseStartPhase +
        trailAge +
        max(0.0, uniforms.depthParameters.x) * speed / routeLength);
    return trailStartPhase + trailDistance / routeLength;
}

float WaterTrailVisibility(uint pointIndex) {
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
    const vec3 p0 = pointPositions.positions[routeStart + p0Offset].xyz;
    const vec3 p1 = pointPositions.positions[routeStart + p1Offset].xyz;
    const vec3 p2 = pointPositions.positions[routeStart + p2Offset].xyz;
    const vec3 p3 = pointPositions.positions[routeStart + p3Offset].xyz;
    return CatmullRomWater(p0, p1, p2, p3, t);
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

    const float routePosition = fract(phase) * float(routeCount - 1u);
    const uint anchorOffset = min(uint(floor(routePosition)), routeCount - 1u);
    const uint prevOffset = anchorOffset > 0u ? anchorOffset - 1u : anchorOffset;
    const uint nextOffset = min(anchorOffset + 1u, routeCount - 1u);
    const vec3 previous = pointPositions.positions[routeStart + prevOffset].xyz;
    const vec3 next = pointPositions.positions[routeStart + nextOffset].xyz;
    const vec3 tangent = next - previous;
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
    const float trailRole = LoadScalarFieldValueForPoint(kWaterTrailRoleFieldSlot, pointIndex);
    if (trailRole < 0.5) {
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

vec3 ResolveWaterFlowPosition(vec3 basePosition, uint pointIndex) {
    if (!HasWaterParticleFields()) {
        return basePosition;
    }
    if (WaterTrailOverlayEnabled()) {
        return ResolveWaterTrailPosition(basePosition, pointIndex);
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

vec2 ApplyWaterFlowAnimation(float opacity, float emissive, uint pointIndex) {
    if (styleData.pointMeta.w == 0u || styleData.globalControl.z <= kWaterSpeedFieldSlot) {
        return vec2(opacity, emissive);
    }

    if (WaterTrailOverlayEnabled()) {
        const float trailRole = LoadScalarFieldValueForPoint(kWaterTrailRoleFieldSlot, pointIndex);
        if (trailRole < 0.5) {
            return vec2(0.0);
        }
        const float trailVisibility = WaterTrailVisibility(pointIndex);
        return vec2(opacity * trailVisibility, emissive * trailVisibility);
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

void main() {
    const uint pointIndex = uint(gl_VertexIndex);
    const vec3 flowPosition = ResolveWaterFlowPosition(inPosition, pointIndex);
    vec4 worldPosition = vec4(ResolveSurfaceMotionPosition(flowPosition, pointIndex), 1.0);
    vec4 viewPosition = uniforms.view * worldPosition;
    const float viewDepth = -viewPosition.z;
    gl_Position = uniforms.viewProjection * worldPosition;
    float previewTint = 0.0;
    const float caustic = ResolveCausticStrength(worldPosition.xyz, pointIndex, previewTint);
    const float waterEffectScale = ResolveRippleEffectScale(pointIndex);
    const SparseRippleComposite sparseRipple =
        ResolveSparseRippleComposite(worldPosition.xyz, ResolveAovNormal(pointIndex), pointIndex, uniforms.depthParameters.x);
    const float waterEffectPointSizeAdd =
        HasWaterEffectComposition() ? WaterEffectField(styleData.waterEffectSlots0.x, pointIndex, 0.0) * waterEffectScale : 0.0;
    const float sparseRipplePointSizeAdd = sparseRipple.pointSizeAdd;
    const float waterEffectPointSizeMultiply =
        HasWaterEffectComposition()
            ? mix(1.0, max(0.0, WaterEffectField(styleData.waterEffectSlots0.y, pointIndex, 1.0)), waterEffectScale)
            : 1.0;
    const float sparseRipplePointSizeMultiply = sparseRipple.pointSizeMultiply;
    const float waterEffectOpacityAdd =
        HasWaterEffectComposition() ? WaterEffectField(styleData.waterEffectControl.z, pointIndex, 0.0) * waterEffectScale : 0.0;
    const float sparseRippleOpacityAdd = sparseRipple.opacityAdd;
    const float waterEffectOpacityMultiply =
        HasWaterEffectComposition()
            ? mix(1.0, max(0.0, WaterEffectField(styleData.waterEffectControl.w, pointIndex, 1.0)), waterEffectScale)
            : 1.0;
    const float sparseRippleOpacityMultiply = sparseRipple.opacityMultiply;
    const float waterEffectEmissionAdd =
        HasWaterEffectComposition()
            ? max(0.0, WaterEffectField(styleData.waterEffectControl.y, pointIndex, 0.0)) * waterEffectScale
            : 0.0;
    const float sparseRippleEmissionAdd = sparseRipple.emissionAdd;
    const bool worldSizedScreenSprites = styleData.renderParams2.w > 0.5;
    const float pointSizeBeforeDepthOfField =
        worldSizedScreenSprites
            ? WorldDiameterToScreenPointSizePixels(
                  (max(0.0, EvaluateBinding(styleData.surfelDiameterBinding)) *
                       WaterPathPointSizeScale(pointIndex) *
                       WaterSteamSizeScale(pointIndex) *
                       (1.0 + caustic * max(0.0, styleData.causticParams1.w)) *
                       waterEffectPointSizeMultiply *
                       sparseRipplePointSizeMultiply) +
                      waterEffectPointSizeAdd +
                      sparseRipplePointSizeAdd,
                  viewDepth)
            : ((clamp(
                    EvaluateBinding(styleData.pointSizeBinding),
                    max(1.0, styleData.renderParams3.y),
                    max(max(1.0, styleData.renderParams3.y), styleData.renderParams3.z)) *
                    WaterPathPointSizeScale(pointIndex) *
                    WaterSteamSizeScale(pointIndex) *
                    (1.0 + caustic * max(0.0, styleData.causticParams1.w)) *
                    waterEffectPointSizeMultiply *
                    sparseRipplePointSizeMultiply) +
                   waterEffectPointSizeAdd +
                   sparseRipplePointSizeAdd);
    gl_PointSize = clamp(
        pointSizeBeforeDepthOfField + ResolveDepthOfFieldBlurPixels(viewDepth),
        max(1.0, styleData.renderParams3.y),
        max(max(1.0, styleData.renderParams3.y), styleData.renderParams3.z));

    const float causticColorSignal = CausticColorSignal(caustic, previewTint);
    outSourceColor =
        vec4(
            ApplySparseRippleColor(
                ApplyWaterEffectColor(
                    mix(inColor.rgb, styleData.causticTint.rgb, CausticColorMixAmount(caustic, previewTint)),
                    pointIndex,
                    waterEffectScale),
                sparseRipple),
            inColor.a);
    outColormapValue = EvaluateBinding(styleData.colormapPositionBinding);
    const vec2 animatedFlow = ApplyWaterFlowAnimation(
        EvaluateBinding(styleData.opacityBinding),
        EvaluateBinding(styleData.emissiveBinding),
        pointIndex);
    outOpacity = clamp(
        (animatedFlow.x * (1.0 + caustic * max(0.0, styleData.causticParams1.z)) *
             waterEffectOpacityMultiply *
             sparseRippleOpacityMultiply) +
            waterEffectOpacityAdd +
            sparseRippleOpacityAdd,
        0.0,
        4.0);
    outEmissive =
        animatedFlow.y +
        caustic * max(0.0, styleData.causticParams1.y) +
        waterEffectEmissionAdd +
        sparseRippleEmissionAdd;
    outXray = EvaluateBinding(styleData.xrayBinding);
    outDepthFade = EvaluateBinding(styleData.depthFadeBinding);
    outViewDepth = viewDepth;
    outPointIndex = pointIndex;
    outSurfaceAngleMask = ResolveSurfaceAngleMask(worldPosition.xyz, pointIndex);
    outAovNormal = ResolveAovNormal(pointIndex);
    outCaustic = causticColorSignal;
}
