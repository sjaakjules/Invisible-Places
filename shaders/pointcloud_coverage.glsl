#ifndef POINTCLOUD_COVERAGE_GLSL
#define POINTCLOUD_COVERAGE_GLSL

float PointCloudFootprintDiameterPixels(vec2 discCoord) {
    const vec2 coordWidth = fwidth(discCoord);
    const float coordUnitsPerPixel = max(max(coordWidth.x, coordWidth.y), 1.0e-6);
    return clamp(2.0 / coordUnitsPerPixel, 1.0, 8192.0);
}

float PointCloudDiscCoverage(vec2 discCoord, float featherPixels) {
    const float radius = length(discCoord);
    if (radius >= 1.0) {
        return 0.0;
    }

    const float clampedFeatherPixels = clamp(featherPixels, 0.0, 2.0);
    if (clampedFeatherPixels <= 1.0e-5) {
        return 1.0;
    }

    const float diameterPixels = PointCloudFootprintDiameterPixels(discCoord);
    const float coreRadius = clamp(
        (diameterPixels - clampedFeatherPixels) / max(diameterPixels, 1.0e-5),
        0.0,
        1.0);
    return 1.0 - smoothstep(coreRadius, 1.0, radius);
}

bool PointCloudDepthCoveragePasses(vec2 discCoord, float featherPixels) {
    return PointCloudDiscCoverage(discCoord, featherPixels) >= 0.5;
}

#endif
