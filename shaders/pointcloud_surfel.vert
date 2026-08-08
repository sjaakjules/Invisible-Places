#version 450
#extension GL_GOOGLE_include_directive : require

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
layout(location = 7) out vec2 outDiscCoord;
layout(location = 8) flat out uint outPointIndex;
#ifndef DEPTH_PREPASS
layout(location = 9) out float outSurfaceAngleMask;
layout(location = 10) out vec3 outAovNormal;
layout(location = 11) out float outCaustic;
layout(location = 12) out vec4 outWaterColourTransform;
layout(location = 13) flat out vec4 outTimingColouriseTransform;
layout(location = 14) flat out float outTimingColouriseEmissionAdd;
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

layout(set = 0, binding = 4, std430) readonly buffer SurfelPositions {
    vec4 positions[];
} surfelPositions;

layout(set = 0, binding = 5, std430) readonly buffer SurfelColors {
    uint colors[];
} surfelColors;

layout(set = 0, binding = 6, std430) readonly buffer SurfelNormals {
    vec4 normals[];
} surfelNormals;

// Keep the shared timing evaluator independent of the resource's historical
// sprite/surfel naming.
#define pointNormals surfelNormals
#define POINTCLOUD_TIMING_COLOURISE_VERTEX
#include "pointcloud_timing_colourise.glsl"
#undef pointNormals

const uint kFieldMapFlagClamp = 1u;
const uint kFieldMapFlagInvert = 2u;
const uint kSurfelVerticesPerPoint = 6u;
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
    const float mask = clamp(LoadScalarFieldValue(maskSlot, pointIndex), 0.0, 1.0);
    if (mask <= 1e-5) {
        return 0.0;
    }
    const float edge = clamp(LoadScalarFieldValue(edgeSlot, pointIndex), 0.0, 1.0);
    const float seed = LoadScalarFieldValue(seedSlot, pointIndex);
    previewTint = CausticPreviewTint(mask, edge, seed);
    const float time = uniforms.depthParameters.x * max(0.0, styleData.causticParams0.z);
    const vec3 normal =
        styleData.pointMeta.z != 0u && pointIndex < styleData.pointMeta.x
            ? surfelNormals.normals[pointIndex].xyz
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
    return LoadScalarFieldValue(slotPlusOne - 1u, pointIndex);
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
    return LoadScalarFieldValue(slotPlusOne - 1u, pointIndex);
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
        return LoadScalarFieldValue(kWaterTrailFeatureTypeFieldSlot, pointIndex);
    }
    return styleData.globalControl.z > kWaterFeatureTypeFieldSlot
        ? LoadScalarFieldValue(kWaterFeatureTypeFieldSlot, pointIndex)
        : 0.0;
}

float WaterTrailFade(uint pointIndex) {
    if (WaterTrailOverlayEnabled()) {
        return 1.0;
    }
    if (styleData.globalControl.z <= kWaterAgeFieldSlot) {
        return 1.0;
    }
    const float age = clamp(LoadScalarFieldValue(kWaterAgeFieldSlot, pointIndex), 0.0, 1.0);
    return pow(1.0 - smoothstep(0.0, 1.0, age), 1.35);
}

bool WaterPathViewEnabled() {
    return styleData.pointMeta.w == 2u;
}

float WaterPathPointSizeScale(uint pointIndex) {
    if (!WaterPathViewEnabled() || !HasWaterParticleFields()) {
        return 1.0;
    }
    const float role = LoadScalarFieldValue(kWaterParticleRoleFieldSlot, pointIndex);
    if (role >= 2.5 && role < 3.5) {
        return 0.48;
    }
    if (role >= 0.5 && role < 1.5) {
        return 0.58;
    }
    return 1.0;
}

float WaterParticleTravel(uint pointIndex) {
    const float phase = LoadScalarFieldValue(kWaterPhaseFieldSlot, pointIndex);
    const float speed = max(0.02, LoadScalarFieldValue(kWaterSpeedFieldSlot, pointIndex));
    return fract(phase + max(0.0, uniforms.depthParameters.x) * speed * kWaterParticleSpeedScale);
}

bool IsWaterSteam(uint pointIndex) {
    const float featureType = WaterFeatureType(pointIndex);
    return featureType > 0.5 && featureType < 1.5;
}

float WaterSteamFade(uint pointIndex) {
    const float travel = WaterParticleTravel(pointIndex);
    const float seed = LoadScalarFieldValue(kWaterJitterSeedFieldSlot, pointIndex);
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
    const float seed = LoadScalarFieldValue(kWaterJitterSeedFieldSlot, pointIndex);
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
    const float roughness = LoadScalarFieldValue(roughnessSlot, pointIndex);
    const float roughnessNormalized =
        clamp((roughness - styleData.surfaceMotionStats.x) * styleData.surfaceMotionStats.y, 0.0, 1.0);
    float mask = smoothstep(clamp(styleData.surfaceMotionParams.w, 0.0, 1.0), 1.0, roughnessNormalized);
    if (styleData.stylisationControl.w != 0u) {
        const uint groundSlot = styleData.stylisationControl.w - 1u;
        const float groundId = LoadScalarFieldValue(groundSlot, pointIndex);
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
    const vec3 basePosition = surfelPositions.positions[anchorIndex].xyz;
    if (pathJitter <= 0.0001) {
        return basePosition;
    }

    const uint prevOffset = clampedOffset > 0u ? clampedOffset - 1u : clampedOffset;
    const uint nextOffset = min(clampedOffset + 1u, pathCount - 1u);
    const vec3 prevPosition = surfelPositions.positions[pathStart + prevOffset].xyz;
    const vec3 nextPosition = surfelPositions.positions[pathStart + nextOffset].xyz;
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
    const float anchorWidth = clamp(LoadScalarFieldValue(kWaterWidthFieldSlot, anchorIndex), 0.001, 100.0);
    const float amplitude = anchorWidth * clamp(pathJitter, 0.0, 3.0) * 0.45 * endpointFade;
    return basePosition + (lateral * lateralNoise + secondary * secondaryNoise * 0.22) * amplitude;
}

uint WaterTrailRouteStart(uint pointIndex) {
    return uint(max(0.0, floor(LoadScalarFieldValue(kWaterTrailRouteStartFieldSlot, pointIndex) + 0.5)));
}

uint WaterTrailRouteCount(uint pointIndex) {
    return uint(max(0.0, floor(LoadScalarFieldValue(kWaterTrailRouteCountFieldSlot, pointIndex) + 0.5)));
}

float WaterTrailTravelPhase(uint pointIndex) {
    const float routeLength = max(0.001, LoadScalarFieldValue(kWaterTrailRouteLengthFieldSlot, pointIndex));
    const float trailDistance = max(0.0, LoadScalarFieldValue(kWaterTrailDistanceFieldSlot, pointIndex));
    const float trailAge = LoadScalarFieldValue(kWaterTrailAgeFieldSlot, pointIndex);
    const float baseStartPhase = LoadScalarFieldValue(kWaterTrailStartPhaseFieldSlot, pointIndex);
    const float speed =
        max(0.0, LoadScalarFieldValue(kWaterTrailSpeedFieldSlot, pointIndex)) *
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
    if (WaterTrailStyleGeometryAvailable()) {
        return WaterTrailStyleWidth();
    }
    return max(0.0001, LoadScalarFieldValue(kWaterTrailWidthFieldSlot, pointIndex));
}

float WaterTrailWidth(uint pointIndex) {
    return WaterTrailBaseWidth(pointIndex) * mix(0.65, 1.0, WaterFlowActivity());
}

float WaterTrailStreakLength(uint pointIndex) {
    float streakLength = 0.0;
    if (WaterTrailStyleGeometryAvailable()) {
        streakLength = WaterTrailStyleWidth() * max(1.0, styleData.renderParams2.z);
    } else {
        streakLength = max(0.001, LoadScalarFieldValue(kWaterTrailStreakLengthFieldSlot, pointIndex));
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
        floor(LoadScalarFieldValue(
                  kWaterTrailEndpointFadeFlagsFieldSlot,
                  pointIndex) +
              0.5)));
    if (flags == 0u) {
        return 1.0;
    }
    const float trailSeed = clamp(
        LoadScalarFieldValue(kWaterTrailSeedFieldSlot, pointIndex),
        0.0,
        1.0);
    const float distanceFromStart = fract(phase) * routeLength;
    float visibility = 1.0;
    if ((flags & 1u) != 0u) {
        visibility *= WaterTrailEndpointFadeRamp(
            distanceFromStart,
            LoadScalarFieldValue(
                kWaterTrailStartFadeFullDistanceFieldSlot,
                pointIndex),
            LoadScalarFieldValue(
                kWaterTrailStartFadeRandomBeginDistanceFieldSlot,
                pointIndex),
            fract(trailSeed * 0.754877666 + 0.137));
    }
    if ((flags & 2u) != 0u) {
        visibility *= WaterTrailEndpointFadeRamp(
            routeLength - distanceFromStart,
            LoadScalarFieldValue(
                kWaterTrailEndFadeFullDistanceFieldSlot,
                pointIndex),
            LoadScalarFieldValue(
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
    const float routeLength = max(0.001, LoadScalarFieldValue(kWaterTrailRouteLengthFieldSlot, pointIndex));
    const float trailStreakLength = WaterTrailStreakLength(pointIndex);
    const float endFeather = clamp(trailStreakLength / routeLength, 0.001, 0.10);
    const bool meshTrail =
        abs(LoadScalarFieldValue(kWaterTrailFeatureTypeFieldSlot, pointIndex) - 5.0) < 0.5;
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
                  LoadScalarFieldValue(
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
    const vec3 p1 = surfelPositions.positions[routeStart + anchorOffset].xyz;
    const vec3 p2 = surfelPositions.positions[routeStart + anchorOffset + 1u].xyz;
    const vec3 p0 = anchorOffset > 0u
        ? surfelPositions.positions[routeStart + anchorOffset - 1u].xyz
        : p1 + (p1 - p2);
    const vec3 p3 = anchorOffset + 2u < routeCount
        ? surfelPositions.positions[routeStart + anchorOffset + 2u].xyz
        : p2 + (p2 - p1);
    return CentripetalCatmullRomWater(p0, p1, p2, p3, t);
}

vec3 WaterTrailRouteTangent(uint pointIndex, float phase) {
    const uint routeStart = WaterTrailRouteStart(pointIndex);
    const uint routeCount = WaterTrailRouteCount(pointIndex);
    if (routeCount < 2u || routeStart >= styleData.pointMeta.x || routeStart + routeCount > styleData.pointMeta.x) {
        const vec3 tangent = vec3(
            LoadScalarFieldValue(kWaterTrailTangentXFieldSlot, pointIndex),
            LoadScalarFieldValue(kWaterTrailTangentYFieldSlot, pointIndex),
            LoadScalarFieldValue(kWaterTrailTangentZFieldSlot, pointIndex));
        return dot(tangent, tangent) > 1e-8 ? normalize(tangent) : vec3(1.0, 0.0, 0.0);
    }

    const vec2 segment = WaterTrailRouteSegment(pointIndex, phase);
    const uint anchorOffset = min(uint(segment.x), routeCount - 2u);
    const float t = segment.y;
    const vec3 p1 = surfelPositions.positions[routeStart + anchorOffset].xyz;
    const vec3 p2 = surfelPositions.positions[routeStart + anchorOffset + 1u].xyz;
    const vec3 p0 = anchorOffset > 0u
        ? surfelPositions.positions[routeStart + anchorOffset - 1u].xyz
        : p1 + (p1 - p2);
    const vec3 p3 = anchorOffset + 2u < routeCount
        ? surfelPositions.positions[routeStart + anchorOffset + 2u].xyz
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
        const vec3 normal = surfelNormals.normals[pointIndex].xyz;
        return dot(normal, normal) > 1e-8 ? normalize(normal) : vec3(0.0, 0.0, 1.0);
    }

    const vec2 segment = WaterTrailRouteSegment(pointIndex, phase);
    const uint anchorOffset = min(uint(segment.x), routeCount - 2u);
    const float t = segment.y;
    const uint p1Offset = anchorOffset;
    const uint p2Offset = min(anchorOffset + 1u, routeCount - 1u);
    const vec3 normal = mix(
        surfelNormals.normals[routeStart + p1Offset].xyz,
        surfelNormals.normals[routeStart + p2Offset].xyz,
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
    const float lateralOffset = LoadScalarFieldValue(kWaterTrailLateralOffsetFieldSlot, pointIndex);
    const float trailSeed = clamp(LoadScalarFieldValue(kWaterTrailSeedFieldSlot, pointIndex), 0.0, 1.0);
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

    const float role = LoadScalarFieldValue(kWaterParticleRoleFieldSlot, pointIndex);
    if (role < 0.5 || role >= 1.5) {
        return basePosition;
    }

    const uint pathStart = uint(max(0.0, floor(LoadScalarFieldValue(kWaterPathStartFieldSlot, pointIndex) + 0.5)));
    const uint pathCount = uint(max(0.0, floor(LoadScalarFieldValue(kWaterPathCountFieldSlot, pointIndex) + 0.5)));
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
    const float seed = LoadScalarFieldValue(kWaterJitterSeedFieldSlot, pointIndex);
    const float pathJitter = clamp(LoadScalarFieldValue(kWaterWidthFieldSlot, pointIndex), 0.0, 3.0);
    const vec3 p0 = JitteredWaterAnchorPosition(pathStart, pathCount, p0Offset, seed, pathJitter);
    const vec3 p1 = JitteredWaterAnchorPosition(pathStart, pathCount, p1Offset, seed, pathJitter);
    const vec3 p2 = JitteredWaterAnchorPosition(pathStart, pathCount, p2Offset, seed, pathJitter);
    const vec3 p3 = JitteredWaterAnchorPosition(pathStart, pathCount, p3Offset, seed, pathJitter);
    return CatmullRomWater(p0, p1, p2, p3, t);
}

vec3 ResolveWaterFlowTangent(uint pointIndex, float trailPhase) {
    if (!HasWaterParticleFields()) {
        return vec3(0.0);
    }
    if (WaterTrailOverlayEnabled()) {
        return WaterTrailRouteTangent(pointIndex, trailPhase);
    }

    const uint pathStart = uint(max(0.0, floor(LoadScalarFieldValue(kWaterPathStartFieldSlot, pointIndex) + 0.5)));
    const uint pathCount = uint(max(0.0, floor(LoadScalarFieldValue(kWaterPathCountFieldSlot, pointIndex) + 0.5)));
    if (pathCount < 2u || pathStart >= styleData.pointMeta.x || pathStart + pathCount > styleData.pointMeta.x) {
        return vec3(0.0);
    }

    const float role = LoadScalarFieldValue(kWaterParticleRoleFieldSlot, pointIndex);
    uint anchorOffset = min(pointIndex > pathStart ? pointIndex - pathStart : 0u, pathCount - 1u);
    if (!WaterPathViewEnabled() && role >= 0.5 && role < 1.5) {
        const float pathPosition = WaterParticleTravel(pointIndex) * float(pathCount - 1u);
        anchorOffset = min(uint(floor(pathPosition)), pathCount - 1u);
    }

    const uint prevOffset = anchorOffset > 0u ? anchorOffset - 1u : anchorOffset;
    const uint nextOffset = min(anchorOffset + 1u, pathCount - 1u);
    const float seed = LoadScalarFieldValue(kWaterJitterSeedFieldSlot, pointIndex);
    const float pathJitter = clamp(LoadScalarFieldValue(kWaterWidthFieldSlot, pointIndex), 0.0, 3.0);
    const vec3 previous = JitteredWaterAnchorPosition(pathStart, pathCount, prevOffset, seed, pathJitter);
    const vec3 next = JitteredWaterAnchorPosition(pathStart, pathCount, nextOffset, seed, pathJitter);
    const vec3 tangent = next - previous;
    return dot(tangent, tangent) > 1e-8 ? normalize(tangent) : vec3(0.0);
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
        const float role = LoadScalarFieldValue(kWaterParticleRoleFieldSlot, pointIndex);
        if (WaterPathViewEnabled()) {
            if (role >= 1.5 && role < 2.5) {
                const float confidence =
                    styleData.globalControl.z > kWaterConfidenceFieldSlot
                        ? clamp(LoadScalarFieldValue(kWaterConfidenceFieldSlot, pointIndex), 0.0, 1.0)
                        : 1.0;
                const float accumulation =
                    styleData.globalControl.z > kWaterAccumulationFieldSlot
                        ? clamp(LoadScalarFieldValue(kWaterAccumulationFieldSlot, pointIndex), 0.0, 1.0)
                        : 0.0;
                return vec2(opacity * (0.45 + confidence * 0.55), emissive * (0.45 + accumulation * 0.75));
            }
            if (role >= 2.5 && role < 3.5) {
                const float confidence =
                    styleData.globalControl.z > kWaterConfidenceFieldSlot
                        ? clamp(LoadScalarFieldValue(kWaterConfidenceFieldSlot, pointIndex), 0.0, 1.0)
                        : 1.0;
                const float shimmer =
                    0.72 + 0.28 * sin((LoadScalarFieldValue(kWaterJitterSeedFieldSlot, pointIndex) + uniforms.depthParameters.x * 0.12) * 6.28318530718);
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
        const float seed = LoadScalarFieldValue(kWaterJitterSeedFieldSlot, pointIndex);
        const float endFade = smoothstep(0.0, 0.08, travel) * (1.0 - smoothstep(0.92, 1.0, travel));
        const float shimmer = 0.78 + 0.22 * sin((travel + seed * 1.618) * 6.28318530718);
        return vec2(opacity * endFade * shimmer * trailFade, emissive * endFade * shimmer * trailFade);
    }

    const float phase = LoadScalarFieldValue(kWaterPhaseFieldSlot, pointIndex);
    const float speed = max(0.02, LoadScalarFieldValue(kWaterSpeedFieldSlot, pointIndex));
    const float confidence =
        styleData.globalControl.z > kWaterConfidenceFieldSlot
            ? clamp(LoadScalarFieldValue(kWaterConfidenceFieldSlot, pointIndex), 0.0, 1.0)
            : 1.0;
    const float accumulation =
        styleData.globalControl.z > kWaterAccumulationFieldSlot
            ? clamp(LoadScalarFieldValue(kWaterAccumulationFieldSlot, pointIndex), 0.0, 1.0)
            : 0.0;
    const float pooling =
        styleData.globalControl.z > kWaterPoolingFieldSlot
            ? clamp(LoadScalarFieldValue(kWaterPoolingFieldSlot, pointIndex), 0.0, 1.0)
            : 0.0;
    const float wave = 0.5 + (0.5 * sin((phase - uniforms.depthParameters.x * speed) * 6.28318530718));
    const float crest = smoothstep(0.58, 1.0, wave);
    const float alphaPulse = clamp((0.24 + crest * 0.82 + pooling * 0.24) * confidence, 0.0, 1.25);
    const float emissivePulse =
        clamp((0.70 + crest * 2.10 + accumulation * 1.25 + pooling * 0.55) * confidence, 0.0, 4.5);
    return vec2(opacity * alphaPulse, emissive * emissivePulse);
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

void ResolveBasis(
    vec3 center,
    uint pointIndex,
    float trailPhase,
    out vec3 tangent,
    out vec3 bitangent,
    out float surfaceAngleMask) {
    const vec3 cameraRight = CameraRight();
    const vec3 cameraUp = CameraUp();
    const bool forceCameraFacing = styleData.renderControl.z == 2u;
    bool useNormal = styleData.pointMeta.z != 0u && !forceCameraFacing;
    vec3 normal = useNormal ? surfelNormals.normals[pointIndex].xyz : vec3(0.0);
    useNormal = useNormal && dot(normal, normal) > 1e-8;
    const float waterStreakAspect = styleData.pointMeta.w != 0u ? max(1.0, styleData.renderParams2.z) : 1.0;

    if (!useNormal) {
        normal = uniforms.cameraPosition.xyz - center;
        if (dot(normal, normal) <= 1e-8) {
            normal = normalize(cross(cameraRight, cameraUp));
        } else {
            normal = normalize(normal);
        }
        if (waterStreakAspect > 1.0001) {
            tangent = ResolveWaterFlowTangent(pointIndex, trailPhase);
            tangent -= normal * dot(tangent, normal);
            if (dot(tangent, tangent) > 1e-8) {
                tangent = normalize(tangent);
                bitangent = normalize(cross(normal, tangent));
                surfaceAngleMask = 0.0;
                return;
            }
        }
        tangent = cameraRight;
        bitangent = cameraUp;
        surfaceAngleMask = 0.0;
        return;
    }

    normal = normalize(normal);
    surfaceAngleMask = clamp(1.0 - abs(dot(normal, normalize(uniforms.cameraPosition.xyz - center))), 0.0, 1.0);
    if (waterStreakAspect > 1.0001) {
        tangent = ResolveWaterFlowTangent(pointIndex, trailPhase);
        tangent -= normal * dot(tangent, normal);
        if (dot(tangent, tangent) > 1e-8) {
            tangent = normalize(tangent);
            bitangent = normalize(cross(normal, tangent));
            return;
        }
    }
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
    if (WaterTrailOverlayEnabled()) {
        return vec3(0.0);
    }
    if (HasWaterParticleFields()) {
        const float role = LoadScalarFieldValue(kWaterParticleRoleFieldSlot, pointIndex);
        if (role >= 0.5) {
            return vec3(0.0);
        }
    }

    vec3 normal = surfelNormals.normals[pointIndex].xyz;
    if (dot(normal, normal) <= 1e-8) {
        return vec3(0.0);
    }
    return normalize(normal);
}

vec3 ApplyResolvedWaterColour(
    vec3 baseColor,
    uint pointIndex,
    float caustic,
    float previewTint,
    float waterEffectScale,
    SparseRippleComposite sparseRipple,
    RainImpactComposite rainImpact,
    MeshFlowContactComposite meshFlowContact) {
    return ApplyMeshFlowContactColour(
        ApplyRainImpactColour(
            ApplySparseRippleColor(
                ApplyWaterEffectColor(
                    mix(
                        baseColor,
                        styleData.causticTint.rgb,
                        CausticColorMixAmount(caustic, previewTint)),
                    pointIndex,
                    waterEffectScale),
                sparseRipple),
            rainImpact),
        meshFlowContact);
}

// Every stage above mixes with an amount that does not depend on the input
// colour, so resolving two probe colours only needs the field reads once.
// The per-stage maths must stay identical to ApplyResolvedWaterColour.
void ApplyWaterEffectColorPair(
    inout vec3 colorA,
    inout vec3 colorB,
    uint pointIndex,
    float waterEffectScale) {
    const float mixAmount =
        clamp(WaterEffectField(styleData.waterEffectSlots0.z, pointIndex, 0.0) * waterEffectScale, 0.0, 1.0);
    if (mixAmount <= 1e-5) {
        return;
    }
    const vec3 effectColor = vec3(
        clamp(WaterEffectField(styleData.waterEffectSlots0.w, pointIndex, 0.62), 0.0, 1.0),
        clamp(WaterEffectField(styleData.waterEffectSlots1.x, pointIndex, 0.88), 0.0, 1.0),
        clamp(WaterEffectField(styleData.waterEffectSlots1.y, pointIndex, 1.0), 0.0, 1.0));
    colorA = mix(colorA, effectColor, mixAmount);
    colorB = mix(colorB, effectColor, mixAmount);
}

void ApplyResolvedWaterColourPair(
    vec3 baseColorA,
    vec3 baseColorB,
    uint pointIndex,
    float caustic,
    float previewTint,
    float waterEffectScale,
    SparseRippleComposite sparseRipple,
    RainImpactComposite rainImpact,
    MeshFlowContactComposite meshFlowContact,
    out vec3 resolvedA,
    out vec3 resolvedB) {
    const float causticMix = CausticColorMixAmount(caustic, previewTint);
    vec3 colorA = mix(baseColorA, styleData.causticTint.rgb, causticMix);
    vec3 colorB = mix(baseColorB, styleData.causticTint.rgb, causticMix);
    ApplyWaterEffectColorPair(colorA, colorB, pointIndex, waterEffectScale);
    colorA = ApplySparseRippleColor(colorA, sparseRipple);
    colorB = ApplySparseRippleColor(colorB, sparseRipple);
    colorA = ApplyRainImpactColour(colorA, rainImpact);
    colorB = ApplyRainImpactColour(colorB, rainImpact);
    resolvedA = ApplyMeshFlowContactColour(colorA, meshFlowContact);
    resolvedB = ApplyMeshFlowContactColour(colorB, meshFlowContact);
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

float ScreenPixelWorldSpan(float viewDepth, float pixels) {
    const float spanNdcY = max(0.0, pixels) * uniforms.viewportParameters.w;
    return max(0.0, spanNdcY * max(0.001, viewDepth) / max(abs(uniforms.projection[1][1]), 1e-5));
}

void main() {
    const uint encodedVertexIndex = uint(gl_VertexIndex);
    const uint pointIndex = encodedVertexIndex / kSurfelVerticesPerPoint;
    const uint cornerIndex = encodedVertexIndex - (pointIndex * kSurfelVerticesPerPoint);
    const vec2 corner = kSurfelCorners[int(cornerIndex)];

    // Trail role, travel phase, and visibility feed the flow position, the
    // streak basis, the opacity/emissive animation, and the effect-visibility
    // gate below. Resolve them once per invocation: each re-derivation costs
    // the same strided scalar column reads.
    const bool waterTrailOverlay = WaterTrailOverlayEnabled();
    float waterTrailRole = 0.0;
    float waterTrailPhase = 0.0;
    float waterTrailVisibility = 1.0;
    if (waterTrailOverlay) {
        waterTrailRole =
            LoadScalarFieldValue(kWaterTrailRoleFieldSlot, pointIndex);
        waterTrailPhase = WaterTrailTravelPhase(pointIndex);
        waterTrailVisibility = WaterTrailVisibility(pointIndex, waterTrailPhase);
    }

    const vec3 flowPosition = ResolveWaterFlowPosition(
        surfelPositions.positions[pointIndex].xyz,
        pointIndex,
        waterTrailRole,
        waterTrailPhase);
    const vec3 center = ResolveSurfaceMotionPosition(flowPosition, pointIndex);
    vec3 tangent;
    vec3 bitangent;
    float surfaceAngleMask;
    ResolveBasis(center, pointIndex, waterTrailPhase, tangent, bitangent, surfaceAngleMask);

    const vec4 centerViewPosition = uniforms.view * vec4(center, 1.0);
    const float centerDepth = -centerViewPosition.z;
    float centerPreviewTint = 0.0;
    const float centerCaustic = ResolveCausticStrength(center, pointIndex, centerPreviewTint);
    const float waterEffectScale = ResolveRippleEffectScale(pointIndex);
    vec3 rippleNormal =
        styleData.pointMeta.z != 0u && pointIndex < styleData.pointMeta.x
            ? surfelNormals.normals[pointIndex].xyz
            : vec3(0.0, 0.0, 1.0);
    rippleNormal = dot(rippleNormal, rippleNormal) > 1e-8 ? normalize(rippleNormal) : vec3(0.0, 0.0, 1.0);
    const SparseRippleComposite sparseRipple =
        ResolveSparseRippleComposite(center, rippleNormal, pointIndex, uniforms.depthParameters.x);
    const RainImpactComposite rainImpact = ResolveRainImpactComposite(center, rippleNormal);
    const MeshFlowContactComposite meshFlowContact =
        ResolveMeshFlowContactComposite(
            center,
            rippleNormal,
            uniforms.depthParameters.x);
    const float waterEffectPointSizeAdd =
        HasWaterEffectComposition() ? WaterEffectField(styleData.waterEffectSlots0.x, pointIndex, 0.0) * waterEffectScale : 0.0;
    const float sparseRipplePointSizeAdd = sparseRipple.pointSizeAdd;
    const float waterEffectPointSizeMultiply =
        HasWaterEffectComposition()
            ? mix(1.0, max(0.0, WaterEffectField(styleData.waterEffectSlots0.y, pointIndex, 1.0)), waterEffectScale)
            : 1.0;
    const float sparseRipplePointSizeMultiply = sparseRipple.pointSizeMultiply;
    float authoredDiameter =
        ((WaterTrailOverlayEnabled()
              ? WaterTrailWidth(pointIndex)
              : max(0.0, EvaluateBinding(styleData.surfelDiameterBinding, pointIndex))) *
             WaterPathPointSizeScale(pointIndex) *
             WaterSteamSizeScale(pointIndex) *
             (1.0 + centerCaustic * max(0.0, styleData.causticParams1.w)) *
             waterEffectPointSizeMultiply *
             sparseRipplePointSizeMultiply *
             rainImpact.pointSizeMultiply *
             meshFlowContact.pointSizeMultiply) +
        waterEffectPointSizeAdd +
        sparseRipplePointSizeAdd;
    const float unboundedDiameter =
        authoredDiameter * max(1.0e-6, styleData.renderParams1.y) +
        (ResolveDepthOfFieldWorldRadius(centerDepth) * 2.0) +
        ScreenPixelWorldSpan(centerDepth, styleData.renderParams2.x);
    const float maximumDiameter = max(
        1.0e-6,
        ScreenPixelWorldSpan(
            centerDepth,
            max(1.0, styleData.renderParams3.z)));
    // Invalid effect data must not expand one surfel into a screen-sized quad.
    // Screen-sprite paths already apply this maximum through gl_PointSize.
    const float diameter =
        (isnan(unboundedDiameter) || isinf(unboundedDiameter))
            ? 0.0
            : clamp(unboundedDiameter, 0.0, maximumDiameter);
    float waterStreakAspect = styleData.pointMeta.w != 0u ? max(1.0, styleData.renderParams2.z) : 1.0;
    if (WaterTrailOverlayEnabled()) {
        const float trailStreakLength = WaterTrailStreakLength(pointIndex);
        waterStreakAspect = max(1.0, trailStreakLength / max(diameter, 0.0001));
        waterStreakAspect = min(waterStreakAspect, 64.0);
    }
    const vec3 offset = (tangent * corner.x * waterStreakAspect + bitangent * corner.y) * (diameter * 0.5);
    const vec4 worldPosition = vec4(center + offset, 1.0);
    const vec4 viewPosition = uniforms.view * worldPosition;
    float previewTint = 0.0;
    const float caustic = ResolveCausticStrength(worldPosition.xyz, pointIndex, previewTint);

    gl_Position = uniforms.viewProjection * worldPosition;

#ifndef DEPTH_PREPASS
    const vec4 sourceColor = UnpackRgba8(surfelColors.colors[pointIndex]);
#endif
    const float waterEffectOpacityAdd =
        HasWaterEffectComposition() ? WaterEffectField(styleData.waterEffectControl.z, pointIndex, 0.0) * waterEffectScale : 0.0;
    const float sparseRippleOpacityAdd = sparseRipple.opacityAdd;
    const float waterEffectOpacityMultiply =
        HasWaterEffectComposition()
            ? mix(1.0, max(0.0, WaterEffectField(styleData.waterEffectControl.w, pointIndex, 1.0)), waterEffectScale)
            : 1.0;
    const float sparseRippleOpacityMultiply = sparseRipple.opacityMultiply;
#ifndef DEPTH_PREPASS
    const float waterEffectEmissionAdd =
        HasWaterEffectComposition()
            ? max(0.0, WaterEffectField(styleData.waterEffectControl.y, pointIndex, 0.0)) * waterEffectScale
            : 0.0;
    const float sparseRippleEmissionAdd = sparseRipple.emissionAdd;
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
            pointIndex,
            caustic,
            previewTint,
            waterEffectScale,
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
        outSourceColor = sourceColor;
    } else {
        outWaterColourTransform = vec4(0.0, 0.0, 0.0, 1.0);
        outSourceColor = vec4(
            ApplyResolvedWaterColour(
                sourceColor.rgb,
                pointIndex,
                caustic,
                previewTint,
                waterEffectScale,
                sparseRipple,
                rainImpact,
                meshFlowContact),
            sourceColor.a);
    }
    outColormapValue = EvaluateBinding(styleData.colormapPositionBinding, pointIndex);
#endif
    const vec2 animatedFlow = ApplyWaterFlowAnimation(
        EvaluateBinding(styleData.opacityBinding, pointIndex),
        EvaluateBinding(styleData.emissiveBinding, pointIndex),
        pointIndex,
        waterTrailRole,
        waterTrailVisibility);
    const float flowEffectVisibility = waterTrailOverlay
        ? waterTrailVisibility * WaterFlowAppearanceScale()
        : 1.0;
    outOpacity = clamp(
        (animatedFlow.x * (1.0 + caustic * max(0.0, styleData.causticParams1.z)) *
             waterEffectOpacityMultiply *
             sparseRippleOpacityMultiply) +
            (waterEffectOpacityAdd +
             sparseRippleOpacityAdd +
             rainImpact.opacityAdd +
             meshFlowContact.opacityAdd) * flowEffectVisibility,
        0.0,
        4.0);
#ifndef DEPTH_PREPASS
    outEmissive =
        animatedFlow.y +
        (caustic * max(0.0, styleData.causticParams1.y) +
         waterEffectEmissionAdd +
         sparseRippleEmissionAdd +
         rainImpact.emissionAdd +
         meshFlowContact.emissionAdd) * flowEffectVisibility;
#endif
    outDepthFade = EvaluateBinding(styleData.depthFadeBinding, pointIndex);
    outViewDepth = -viewPosition.z;
    outDiscCoord = corner;
    outPointIndex = pointIndex;
#ifndef DEPTH_PREPASS
    outSurfaceAngleMask = surfaceAngleMask;
    outAovNormal = ResolveAovNormal(pointIndex);
    outCaustic = CausticColorSignal(caustic, previewTint);
#endif
}
