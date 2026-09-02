#ifndef INVISIBLE_PLACES_POINTCLOUD_SURFACE_STABILITY_GLSL
#define INVISIBLE_PLACES_POINTCLOUD_SURFACE_STABILITY_GLSL

// The validated display-density sidecar stores four alternative opacity
// weights in the bit pattern of one float scalar column. Keeping it in the
// existing field-major scalar buffer avoids another descriptor and makes the
// lookup identical for full-resolution, 5 mm, subset, and aHQ block loads.
float ResolvePointCloudSurfaceStabilityWeight(uint pointIndex) {
    const uint mode = styleData.surfaceStabilityControl.x;
    const uint fieldSlot = styleData.surfaceStabilityControl.y;
    // Original (0) and Draw All (1) are deliberately free of scalar reads.
    // An out-of-range point index must degrade exactly like an absent
    // column: LoadScalarFieldValueForPoint would return 0.0, whose bit
    // pattern decodes as zero weight and silently hides the point.
    if (mode < 2u ||
        mode > 5u ||
        fieldSlot == 0xFFFFFFFFu ||
        fieldSlot >= styleData.globalControl.z ||
        pointIndex >= styleData.pointMeta.x) {
        return 1.0;
    }

    const uint packed = floatBitsToUint(
        LoadScalarFieldValueForPoint(fieldSlot, pointIndex));
    const uint channel = mode - 2u;
    const float selected =
        float((packed >> (channel * 8u)) & 0xFFu) * (1.0 / 255.0);
    return mix(
        1.0,
        selected,
        clamp(styleData.surfaceStabilityParams.x, 0.0, 1.0));
}

// Fast Basic is intentionally an opaque early-z renderer. It represents a
// fractional sidecar weight as a fixed per-point coverage decision rather
// than frame-varying alpha, preserving its fast path without introducing
// temporal shimmer. The decision hashes the quantized cloud-space position,
// not the point index: aHQ republishes and density switches renumber local
// indices, and the same physical point must keep the same verdict across
// layouts and in the CPU export. floor() avoids round-half ambiguity and
// fp32 multiply/floor are exact for shared inputs, so the GLSL and C++
// mirrors agree bit-for-bit (SurfaceStabilityPositionHash01 in
// OfflinePointRenderer).
float PointCloudSurfaceStabilityHash01(vec3 cloudPosition) {
    const ivec3 cell = ivec3(floor(cloudPosition * 2048.0));
    uint value = (uint(cell.x) * 0x8da6b343u) ^
                 (uint(cell.y) * 0xd8163841u) ^
                 (uint(cell.z) * 0xcb1ab31fu);
    value ^= value >> 16u;
    value *= 0x7feb352du;
    value ^= value >> 15u;
    value *= 0x846ca68bu;
    value ^= value >> 16u;
    return float(value & 0x00FFFFFFu) * (1.0 / 16777215.0);
}

bool PointCloudSurfaceStabilityKeepsOpaquePoint(
    uint pointIndex,
    vec3 cloudPosition) {
    const float weight = ResolvePointCloudSurfaceStabilityWeight(pointIndex);
    return weight >= 0.99999 ||
           (weight > 0.0 &&
            PointCloudSurfaceStabilityHash01(cloudPosition) < weight);
}

#endif
