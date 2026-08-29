// Stable live-preview aHQ selection. pointMeta.y carries the per-layer role:
// 0 ordinary, 1 fine, 2 coarse. The hash is camera-independent, so a paused
// view and small camera moves do not shimmer through different point sets.
const uint kAdaptiveDensityDisabled = 0u;
const uint kAdaptiveDensityFine = 1u;
const uint kAdaptiveDensityCoarse = 2u;

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
        return true;
    }
    const float coarseWeight = smoothstep(startDepth, endDepth, viewDepth);
    const float fineWeight = 1.0 - coarseWeight;
    const float probability = role == kAdaptiveDensityFine
        ? fineWeight * fineWeight
        : (role == kAdaptiveDensityCoarse ? coarseWeight : 1.0);
    if (probability <= 0.0) {
        return false;
    }
    if (probability >= 1.0) {
        return true;
    }
    return AdaptiveDensityNoise(worldPosition, pointIndex) < probability;
}
