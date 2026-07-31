#ifndef POINTCLOUD_TIMING_COLOURISE_GLSL
#define POINTCLOUD_TIMING_COLOURISE_GLSL

const uint kTimingColouriseMaxEffects = 5u;
const uint kTimingColouriseLutSamples = 64u;
const uint kTimingColouriseScalarField = 0u;
const uint kTimingColouriseNormalX = 1u;
const uint kTimingColouriseNormalY = 2u;
const uint kTimingColouriseNormalZ = 3u;

#ifdef POINTCLOUD_TIMING_COLOURISE_VERTEX

vec4 ResolveTimingColouriseEffect(uint effectIndex, uint pointIndex) {
    if (effectIndex >= min(
            styleData.timingColouriseControl.x,
            kTimingColouriseMaxEffects)) {
        return vec4(0.0);
    }
    const uvec4 source = styleData.timingColouriseSources[effectIndex];
    if (source.x == 0u ||
        styleData.pointMeta.x == 0u ||
        pointIndex >= styleData.pointMeta.x) {
        return vec4(0.0);
    }

    float value = 0.0;
    if (source.y == kTimingColouriseScalarField) {
        if (source.z == 0u) {
            return vec4(0.0);
        }
        const uint fieldSlot = source.z - 1u;
        if (fieldSlot >= styleData.globalControl.z) {
            return vec4(0.0);
        }
        value = scalarFieldValues.values[
            fieldSlot * styleData.pointMeta.x + pointIndex];
    } else {
        if (styleData.pointMeta.z == 0u) {
            return vec4(0.0);
        }
        const vec3 normal = pointNormals.normals[pointIndex].xyz;
        if (source.y == kTimingColouriseNormalX) {
            value = normal.x;
        } else if (source.y == kTimingColouriseNormalY) {
            value = normal.y;
        } else if (source.y == kTimingColouriseNormalZ) {
            value = normal.z;
        } else {
            return vec4(0.0);
        }
    }
    if (isnan(value) || isinf(value)) {
        return vec4(0.0);
    }

    const vec4 range = styleData.timingColouriseRanges[effectIndex];
    const float span = range.y - range.x;
    if (!(span > 1.0e-12) || value < range.x || value > range.y) {
        return vec4(0.0);
    }
    const float normalized = clamp((value - range.x) / span, 0.0, 1.0);
    float edgeMask = 1.0;
    const float fadeFraction = clamp(range.z, 0.0, 0.5);
    if (fadeFraction > 1.0e-6) {
        edgeMask = min(
            smoothstep(0.0, fadeFraction, normalized),
            smoothstep(0.0, fadeFraction, 1.0 - normalized));
    }

    const float scaled =
        normalized * float(kTimingColouriseLutSamples - 1u);
    const uint lowerSample = min(
        uint(floor(scaled)),
        kTimingColouriseLutSamples - 1u);
    const uint upperSample = min(
        lowerSample + 1u,
        kTimingColouriseLutSamples - 1u);
    const uint lutOffset =
        effectIndex * kTimingColouriseLutSamples;
    const vec4 lut = mix(
        styleData.timingColouriseLut[lutOffset + lowerSample],
        styleData.timingColouriseLut[lutOffset + upperSample],
        scaled - float(lowerSample));
    return vec4(
        clamp(lut.rgb, 0.0, 1.0),
        clamp(lut.a, 0.0, 1.0) * clamp(edgeMask, 0.0, 1.0));
}

void ResolveTimingColouriseStack(
    uint pointIndex,
    out vec4 resolvedEffects[5]) {
    for (uint effectIndex = 0u;
         effectIndex < kTimingColouriseMaxEffects;
         ++effectIndex) {
        resolvedEffects[effectIndex] =
            ResolveTimingColouriseEffect(effectIndex, pointIndex);
    }
}

#endif

#ifdef POINTCLOUD_TIMING_COLOURISE_FRAGMENT

vec3 ApplyTimingColouriseStack(vec3 baseColor) {
    vec3 colour = baseColor;
    for (uint effectIndex = 0u;
         effectIndex < kTimingColouriseMaxEffects;
         ++effectIndex) {
        const vec4 effect = inTimingColourise[effectIndex];
        colour = mix(
            colour,
            clamp(effect.rgb, 0.0, 1.0),
            clamp(effect.a, 0.0, 1.0));
    }
    return colour;
}

#endif

#endif
