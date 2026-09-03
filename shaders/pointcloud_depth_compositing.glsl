#ifndef INVISIBLE_PLACES_POINTCLOUD_DEPTH_COMPOSITING_GLSL
#define INVISIBLE_PLACES_POINTCLOUD_DEPTH_COMPOSITING_GLSL

// Strength 1.0 is byte-for-byte the historical linear near/far weighting.
// Higher strengths progressively substitute logarithmic camera depth, which
// gives useful separation to nearby survey surfaces even when the camera far
// plane is hundreds or thousands of metres away.
float PointCloudWeightedAlphaWeight(float alpha) {
    const float nearDepth = max(1.0e-5, uniforms.depthParameters.y);
    const float farDepth = max(nearDepth + 1.0e-5, uniforms.depthParameters.z);
    const float linearDepthNorm = clamp(
        (inViewDepth - nearDepth) / (farDepth - nearDepth),
        0.0,
        1.0);
    const float opacityBase = min(1.0, alpha * 8.0) + 0.01;
    const float opacityWeight = opacityBase * opacityBase * opacityBase;
    const float linearFrontBase = 1.0 - linearDepthNorm;
    const float linearFrontSquared = linearFrontBase * linearFrontBase;
    const float linearFrontWeight = linearFrontSquared * linearFrontSquared;
    const float strength = clamp(styleData.depthCompositingParams.z, 1.0, 8.0);
    if (strength <= 1.0001) {
        return clamp(
            (opacityWeight * 0.5) +
                (opacityWeight * linearFrontWeight * 128.0),
            1.0e-3,
            256.0);
    }
    const float logarithmicDepthNorm = clamp(
        log2(max(1.0, inViewDepth / nearDepth)) /
            max(1.0e-5, log2(farDepth / nearDepth)),
        0.0,
        1.0);
    const float strengthMix = (strength - 1.0) / 7.0;
    const float depthNorm = mix(
        linearDepthNorm,
        logarithmicDepthNorm,
        strengthMix);
    const float frontBase = 1.0 - depthNorm;
    const float frontSquared = frontBase * frontBase;
    const float frontWeight = frontSquared * frontSquared;
    return clamp(
        (opacityWeight * 0.5) + (opacityWeight * frontWeight * 128.0),
        1.0e-3,
        256.0);
}

// Saturated emission applies the exponential response per fragment and
// folds it into the blended colour, so glow is bounded at the front surface
// instead of accumulating through the overlap stack. This is the response
// the GPU sorted path originally shipped with; Accumulated (the default)
// preserves the established linear accumulation into the emission
// attachment.
bool PointCloudSaturatedEmissionEnabled() {
    return styleData.emissionNormalControl.x > 0.5;
}

vec3 PointCloudSaturatedEmissionColor(
    vec3 baseColor,
    float compensatedRawAlpha,
    float alpha,
    float emissionGain) {
    const vec3 emissionContribution = vec3(1.0) - exp(
        -max(baseColor, vec3(0.0)) *
        compensatedRawAlpha * emissionGain);
    return baseColor + emissionContribution / max(alpha, 1.0e-5);
}

// The input is the hardware depth attachment. Reconstruct its positive
// view-space depth so tolerance remains an intuitive world-metre value.
// A clear value of one means no prepass core covered this pixel.
bool PointCloudDepthPrepassRejectsFragment() {
    if (styleData.renderControl.x != 1u &&
        styleData.renderControl.x != 3u) {
        return false;
    }
    const float hardwareDepth = subpassLoad(sceneDepthInput).r;
    if (hardwareDepth >= 1.0 - 1.0e-7) {
        return false;
    }
    // Solve projection(viewZ).z / projection(viewZ).w for viewZ. This
    // general form supports both perspective and parallel cameras.
    const float denominator =
        hardwareDepth * uniforms.projection[2][3] -
        uniforms.projection[2][2];
    if (abs(denominator) <= 1.0e-7) {
        return false;
    }
    // This rearrangement directly reconstructs positive -viewZ for the
    // right-handed OrbitCamera projection (the numerator is the negated
    // conventional view-Z solution), matching inViewDepth from the vertex.
    const float nearestCoreDepth =
        (hardwareDepth * uniforms.projection[3][3] -
         uniforms.projection[3][2]) /
        denominator;
    if (!(nearestCoreDepth > 0.0) || isnan(nearestCoreDepth) ||
        isinf(nearestCoreDepth)) {
        return false;
    }
    const float toleranceMeters = max(0.0, styleData.depthCompositingParams.y);
    return inViewDepth > nearestCoreDepth + toleranceMeters;
}

#endif
