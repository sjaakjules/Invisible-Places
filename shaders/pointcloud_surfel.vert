#version 450

layout(location = 0) out vec4 outSourceColor;
layout(location = 1) out float outColormapValue;
layout(location = 2) out float outOpacity;
layout(location = 3) out float outEmissive;
layout(location = 4) out float outXray;
layout(location = 5) out float outDepthFade;
layout(location = 6) out float outViewDepth;
layout(location = 7) out vec2 outDiscCoord;
layout(location = 8) flat out uint outPointIndex;
layout(location = 9) out float outSurfaceAngleMask;
layout(location = 10) out vec3 outAovNormal;

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

bool HasWaterParticleFields() {
    return styleData.pointMeta.w != 0u && styleData.globalControl.z > kWaterJitterSeedFieldSlot;
}

bool WaterPathViewEnabled() {
    return styleData.pointMeta.w == 2u;
}

float WaterParticleTravel(uint pointIndex) {
    const float phase = LoadScalarFieldValue(kWaterPhaseFieldSlot, pointIndex);
    const float speed = max(0.02, LoadScalarFieldValue(kWaterSpeedFieldSlot, pointIndex));
    return fract(phase + max(0.0, uniforms.depthParameters.x) * speed * kWaterParticleSpeedScale);
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

vec3 ResolveWaterFlowPosition(vec3 basePosition, uint pointIndex) {
    if (!HasWaterParticleFields()) {
        return basePosition;
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

vec2 ApplyWaterFlowAnimation(float opacity, float emissive, uint pointIndex) {
    if (styleData.pointMeta.w == 0u || styleData.globalControl.z <= kWaterSpeedFieldSlot) {
        return vec2(opacity, emissive);
    }

    if (HasWaterParticleFields()) {
        const float role = LoadScalarFieldValue(kWaterParticleRoleFieldSlot, pointIndex);
        if (WaterPathViewEnabled()) {
            if (role < 1.5) {
                return vec2(0.0);
            }
            const float confidence =
                styleData.globalControl.z > kWaterConfidenceFieldSlot
                    ? clamp(LoadScalarFieldValue(kWaterConfidenceFieldSlot, pointIndex), 0.0, 1.0)
                    : 1.0;
            const float accumulation =
                styleData.globalControl.z > kWaterAccumulationFieldSlot
                    ? clamp(LoadScalarFieldValue(kWaterAccumulationFieldSlot, pointIndex), 0.0, 1.0)
                    : 0.0;
            return vec2(opacity * (0.35 + confidence * 0.65), emissive * (0.35 + accumulation * 0.65));
        }
        if (role < 0.5 || role >= 1.5) {
            return vec2(0.0);
        }

        const float travel = WaterParticleTravel(pointIndex);
        const float seed = LoadScalarFieldValue(kWaterJitterSeedFieldSlot, pointIndex);
        const float endFade = smoothstep(0.0, 0.08, travel) * (1.0 - smoothstep(0.92, 1.0, travel));
        const float shimmer = 0.78 + 0.22 * sin((travel + seed * 1.618) * 6.28318530718);
        return vec2(opacity * endFade * shimmer, emissive * endFade * shimmer);
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
    out vec3 tangent,
    out vec3 bitangent,
    out float surfaceAngleMask) {
    const vec3 cameraRight = CameraRight();
    const vec3 cameraUp = CameraUp();
    const bool forceCameraFacing = styleData.renderControl.z == 2u;
    bool useNormal = styleData.pointMeta.z != 0u && !forceCameraFacing;
    vec3 normal = useNormal ? surfelNormals.normals[pointIndex].xyz : vec3(0.0);
    useNormal = useNormal && dot(normal, normal) > 1e-8;

    if (!useNormal) {
        tangent = cameraRight;
        bitangent = cameraUp;
        surfaceAngleMask = 0.0;
        return;
    }

    normal = normalize(normal);
    surfaceAngleMask = clamp(1.0 - abs(dot(normal, normalize(uniforms.cameraPosition.xyz - center))), 0.0, 1.0);
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

    const vec3 flowPosition = ResolveWaterFlowPosition(surfelPositions.positions[pointIndex].xyz, pointIndex);
    const vec3 center = ResolveSurfaceMotionPosition(flowPosition, pointIndex);
    vec3 tangent;
    vec3 bitangent;
    float surfaceAngleMask;
    ResolveBasis(center, pointIndex, tangent, bitangent, surfaceAngleMask);

    const vec4 centerViewPosition = uniforms.view * vec4(center, 1.0);
    const float centerDepth = -centerViewPosition.z;
    const float diameter =
        max(0.0, EvaluateBinding(styleData.surfelDiameterBinding, pointIndex)) +
        (ResolveDepthOfFieldWorldRadius(centerDepth) * 2.0);
    const vec3 offset = (tangent * corner.x + bitangent * corner.y) * (diameter * 0.5);
    const vec4 worldPosition = vec4(center + offset, 1.0);
    const vec4 viewPosition = uniforms.view * worldPosition;

    gl_Position = uniforms.viewProjection * worldPosition;

    outSourceColor = UnpackRgba8(surfelColors.colors[pointIndex]);
    outColormapValue = EvaluateBinding(styleData.colormapPositionBinding, pointIndex);
    const vec2 animatedFlow = ApplyWaterFlowAnimation(
        EvaluateBinding(styleData.opacityBinding, pointIndex),
        EvaluateBinding(styleData.emissiveBinding, pointIndex),
        pointIndex);
    outOpacity = animatedFlow.x;
    outEmissive = animatedFlow.y;
    outXray = EvaluateBinding(styleData.xrayBinding, pointIndex);
    outDepthFade = EvaluateBinding(styleData.depthFadeBinding, pointIndex);
    outViewDepth = -viewPosition.z;
    outDiscCoord = corner;
    outPointIndex = pointIndex;
    outSurfaceAngleMask = surfaceAngleMask;
    outAovNormal = ResolveAovNormal(pointIndex);
}
