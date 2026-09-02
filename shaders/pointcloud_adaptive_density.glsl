// Stable live-preview aHQ selection. pointMeta.y carries the per-layer role:
// 0 ordinary, 1 fine, 2 coarse. The hash is camera-independent, so a paused
// view and small camera moves do not shimmer through different point sets.
const uint kAdaptiveDensityDisabled = 0u;
const uint kAdaptiveDensityFine = 1u;
const uint kAdaptiveDensityCoarse = 2u;
const uint kAdaptiveDensityFineOutgoing = 3u;

uint AdaptiveDensityHash(uint value) {
    value ^= value >> 16u;
    value *= 0x7feb352du;
    value ^= value >> 15u;
    value *= 0x846ca68bu;
    value ^= value >> 16u;
    return value;
}

float AdaptiveDensityNoise(vec3 worldPosition, uint pointIndex) {
    const uvec3 bits = floatBitsToUint(worldPosition);
    uint hash = AdaptiveDensityHash(pointIndex ^ bits.x);
    hash = AdaptiveDensityHash(hash ^ bits.y);
    hash = AdaptiveDensityHash(hash ^ bits.z);
    return float(hash & 0x00ffffffu) / 16777216.0;
}

bool AdaptiveDensityKeepsPoint(
    vec3 worldPosition,
    uint pointIndex,
    float viewDepth) {
    const uint role = styleData.pointMeta.y;
    if (role == kAdaptiveDensityDisabled) {
        return true;
    }
    const float startDepth = uniforms.adaptiveDensityParameters.x;
    const float endDepth = uniforms.adaptiveDensityParameters.z;
    if (!(startDepth > 0.0 && endDepth > startDepth)) {
        // Without a valid transition the incoming layers draw in full, so a
        // lingering outgoing layer must not double-draw the same points.
        return role != kAdaptiveDensityFineOutgoing;
    }
    const float coarseWeight = smoothstep(startDepth, endDepth, viewDepth);
    const float fineWeight = 1.0 - coarseWeight;
    // w is the publish crossfade's engage factor: during a refresh the
    // complete coarse layer draws for coverage, and after the publish the
    // redundant near-zone coarse points ramp out over a short window
    // instead of vanishing in one frame. The same ramp hands the fine
    // points over from the outgoing to the incoming patch layers point by
    // point, so the near zone never swaps in a single frame either.
    const float coarseEngage = clamp(
        uniforms.adaptiveDensityParameters.w,
        0.0,
        1.0);
    // The publish crossfade blends the outgoing and incoming fine banks by
    // complementary per-layer opacity (applied CPU-side to the layer style),
    // not by partitioning points: a per-point handoff scrambles per-pixel
    // blend order under sorted transparency and measured strictly worse.
    // Both banks therefore keep the same depth-band probability here.
    const float probability =
        (role == kAdaptiveDensityFine ||
         role == kAdaptiveDensityFineOutgoing)
        ? fineWeight * fineWeight
        : (role == kAdaptiveDensityCoarse
               ? mix(1.0, coarseWeight, coarseEngage)
               : 1.0);
    if (probability <= 0.0) {
        return false;
    }
    if (probability >= 1.0) {
        return true;
    }
    return AdaptiveDensityNoise(worldPosition, pointIndex) < probability;
}
