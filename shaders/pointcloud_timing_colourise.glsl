#ifndef POINTCLOUD_TIMING_COLOURISE_GLSL
#define POINTCLOUD_TIMING_COLOURISE_GLSL

const uint kTimingColouriseMaxEffects = 8u;
const uint kTimingColouriseLutSamples = 64u;
const uint kTimingColouriseScalarField = 0u;
const uint kTimingColouriseNormalX = 1u;
const uint kTimingColouriseNormalY = 2u;
const uint kTimingColouriseNormalZ = 3u;
const uint kTimingColouriseColouriseOutput = 0u;
const uint kTimingColouriseEmissiveOutput = 1u;

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
    const vec4 fades = styleData.timingColouriseFades[effectIndex];
    const float span = range.y - range.x;
    // Each edge owns a signed fade fraction of the span (positive inward,
    // negative outward), independently clamped to a whole span.
    const float lowerFade = clamp(fades.x, -1.0, 1.0);
    const float upperFade = clamp(fades.y, -1.0, 1.0);
    const float lowerOutward = span * max(-lowerFade, 0.0);
    const float upperOutward = span * max(-upperFade, 0.0);
    if (!(span > 1.0e-12) ||
        value < range.x - lowerOutward ||
        value > range.y + upperOutward) {
        return vec4(0.0);
    }
    const float normalized = clamp((value - range.x) / span, 0.0, 1.0);
    float lowerMask = 1.0;
    if (lowerFade > 1.0e-6) {
        lowerMask = smoothstep(0.0, lowerFade, normalized);
    } else if (lowerFade < -1.0e-6) {
        lowerMask = smoothstep(range.x - lowerOutward, range.x, value);
    }
    float upperMask = 1.0;
    if (upperFade > 1.0e-6) {
        upperMask = smoothstep(0.0, upperFade, 1.0 - normalized);
    } else if (upperFade < -1.0e-6) {
        upperMask =
            1.0 - smoothstep(range.y, range.y + upperOutward, value);
    }
    const float edgeMask = min(lowerMask, upperMask);

    const float safeEdgeMask = clamp(edgeMask, 0.0, 1.0);
    if (source.w == kTimingColouriseEmissiveOutput) {
        // A single resolved vec4 carries both effect kinds. Negative alpha
        // is an emissive sentinel consumed by
        // ResolveTimingColouriseTransform; x stores the signed masked level.
        return vec4(
            range.w * safeEdgeMask,
            0.0,
            0.0,
            -1.0);
    }
    if (source.w != kTimingColouriseColouriseOutput) {
        return vec4(0.0);
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
        clamp(lut.a, 0.0, 1.0) * safeEdgeMask);
}

void ResolveTimingColouriseTransform(
    uint pointIndex,
    out vec4 transform,
    out float emissionAdd) {
    // Compose the per-effect mix chain into one affine transform so the
    // interstage payload is a single vec4 + float instead of eight vec4s:
    // finalColour = baseColour * transform.a + transform.rgb.
    vec3 blended = vec3(0.0);
    float retained = 1.0;
    float emission = 0.0;
    const uint effectCount = min(
        styleData.timingColouriseControl.x,
        kTimingColouriseMaxEffects);
    for (uint effectIndex = 0u;
         effectIndex < effectCount;
         ++effectIndex) {
        const vec4 effect =
            ResolveTimingColouriseEffect(effectIndex, pointIndex);
        if (effect.a < 0.0) {
            if (effect.x < 0.0) {
                // A level of -1 fully darkens the masked colour. Folding the
                // attenuation into the affine transform preserves stack
                // order with neighbouring colourise effects.
                const float retainedLight = clamp(1.0 + effect.x, 0.0, 1.0);
                blended *= retainedLight;
                retained *= retainedLight;
            } else {
                emission += effect.x;
            }
            continue;
        }
        const float alpha = clamp(effect.a, 0.0, 1.0);
        blended = mix(blended, clamp(effect.rgb, 0.0, 1.0), alpha);
        retained *= 1.0 - alpha;
    }
    transform = vec4(blended, retained);
    emissionAdd = emission;
}

#endif

#ifdef POINTCLOUD_TIMING_COLOURISE_FRAGMENT

vec3 ApplyTimingColouriseStack(vec3 baseColor) {
    return baseColor * inTimingColouriseTransform.a +
        inTimingColouriseTransform.rgb;
}

float ResolveTimingColouriseEmissionAdd() {
    return max(0.0, inTimingColouriseEmissionAdd);
}

#endif

#endif
