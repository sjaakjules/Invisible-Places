#version 450

layout(location = 0) in vec2 inUv;
layout(location = 0) out vec4 outColor;

layout(set = 0, binding = 0) uniform sampler2D sceneColorInput;
layout(set = 0, binding = 1) uniform sampler2D linearDepthInput;
layout(set = 0, binding = 2) uniform sampler2D temporalColorAInput;
layout(set = 0, binding = 3) uniform sampler2D temporalDepthAInput;
layout(set = 0, binding = 4) uniform sampler2D temporalColorBInput;
layout(set = 0, binding = 5) uniform sampler2D temporalDepthBInput;

layout(push_constant) uniform PostProcessData {
    vec4 edl;
    vec4 preview;
    // x: 0 off, 1 reprojected blend, 2 feature-following split,
    // 3 straight-alpha A-over-B/B-over-A; y/z: A/B weights or opacity;
    // w: validity mask.
    vec4 temporalOverlay;
    // x: stale source (1=A, 2=B, 0=off); y/z: cached projection X/Y;
    // w: reference view depth for sparse/background reprojection.
    vec4 temporalReprojection;
    // Takes a position in the stale source's cached view space to that
    // source camera's clip space at the current linked-animation time.
    mat4 staleViewToCurrentClip;
} postProcess;

bool ValidDepth(float depth) {
    return !isnan(depth) && !isinf(depth) && depth > 0.0;
}

float LogDepth(float depth) {
    return log2(max(depth, 1.0e-6));
}

float DepthAt(ivec2 coord, int source) {
    if (source == 1) {
        return texelFetch(temporalDepthAInput, coord, 0).r;
    }
    if (source == 2) {
        return texelFetch(temporalDepthBInput, coord, 0).r;
    }
    return texelFetch(linearDepthInput, coord, 0).r;
}

float EyeDomeLightingShade(ivec2 coord, ivec2 size, int source) {
    if (postProcess.edl.x <= 0.5) {
        return 1.0;
    }

    const float centerDepth = DepthAt(coord, source);
    if (!ValidDepth(centerDepth)) {
        return 1.0;
    }

    const ivec2 offsets[8] = ivec2[](
        ivec2(-1, 0),
        ivec2(1, 0),
        ivec2(0, -1),
        ivec2(0, 1),
        ivec2(-1, -1),
        ivec2(1, -1),
        ivec2(-1, 1),
        ivec2(1, 1)
    );

    const float centerLogDepth = LogDepth(centerDepth);
    float response = 0.0;
    int sampleCount = 0;
    const int radiusPixels = clamp(int(round(postProcess.edl.w)), 1, 24);
    for (int radius = 1; radius <= radiusPixels; ++radius) {
        for (int index = 0; index < 8; ++index) {
            const ivec2 sampleCoord = coord + (offsets[index] * radius);
            if (sampleCoord.x < 0 || sampleCoord.y < 0 || sampleCoord.x >= size.x || sampleCoord.y >= size.y) {
                continue;
            }

            const float neighborDepth = DepthAt(sampleCoord, source);
            if (!ValidDepth(neighborDepth)) {
                continue;
            }

            response += max(0.0, LogDepth(neighborDepth) - centerLogDepth);
            ++sampleCount;
        }
    }

    if (sampleCount == 0 || response <= 1.0e-6) {
        return 1.0;
    }

    const float shade = exp(-max(0.0, postProcess.edl.y) * response / float(sampleCount));
    return clamp(shade, clamp(postProcess.edl.z, 0.0, 1.0), 1.0);
}

vec4 FinishSceneColor(vec4 sceneColor, float shade) {
    vec3 color = sceneColor.rgb;
    if (postProcess.preview.x > 0.5) {
        color *= clamp(sceneColor.a, 0.0, 1.0);
    }
    return vec4(color * shade, sceneColor.a);
}

vec4 ShadeSceneColor(vec4 sceneColor, float shade) {
    return vec4(sceneColor.rgb * shade, sceneColor.a);
}

vec4 ApplyAlphaPreview(vec4 sceneColor) {
    if (postProcess.preview.x > 0.5) {
        sceneColor.rgb *= clamp(sceneColor.a, 0.0, 1.0);
    }
    return sceneColor;
}

vec4 CompositeStraightAlphaOver(
    vec4 top,
    vec4 bottom,
    float topOpacity,
    float bottomOpacity) {
    const float topAlpha = clamp(
        top.a * clamp(topOpacity, 0.0, 1.0),
        0.0,
        1.0);
    const float bottomAlpha = clamp(
        bottom.a * clamp(bottomOpacity, 0.0, 1.0),
        0.0,
        1.0);
    const float outputAlpha =
        topAlpha + (bottomAlpha * (1.0 - topAlpha));
    if (outputAlpha <= 1.0e-6) {
        return vec4(0.0);
    }
    const vec3 premultiplied =
        (top.rgb * topAlpha) +
        (bottom.rgb * bottomAlpha * (1.0 - topAlpha));
    return vec4(premultiplied / outputAlpha, outputAlpha);
}

ivec2 TemporalCoord(vec2 uv, ivec2 size) {
    return clamp(
        ivec2(floor(uv * vec2(size))),
        ivec2(0),
        size - ivec2(1));
}

vec4 TemporalColorTexel(ivec2 coord, int source) {
    if (source == 1) {
        return texelFetch(temporalColorAInput, coord, 0);
    }
    return texelFetch(temporalColorBInput, coord, 0);
}

vec4 TemporalColorAt(vec2 uv, ivec2 size, int source) {
    const int staleSource = int(round(postProcess.temporalReprojection.x));
    if (source != staleSource) {
        return TemporalColorTexel(TemporalCoord(uv, size), source);
    }

    // The shared post sampler stays nearest for portable R32 depth support;
    // manually filter only the colour history that moves by sub-pixels.
    const vec2 pixel = (uv * vec2(size)) - 0.5;
    const ivec2 base = ivec2(floor(pixel));
    const vec2 amount = fract(pixel);
    const ivec2 maximum = size - ivec2(1);
    const ivec2 p00 = clamp(base, ivec2(0), maximum);
    const ivec2 p10 = clamp(base + ivec2(1, 0), ivec2(0), maximum);
    const ivec2 p01 = clamp(base + ivec2(0, 1), ivec2(0), maximum);
    const ivec2 p11 = clamp(base + ivec2(1, 1), ivec2(0), maximum);
    return mix(
        mix(
            TemporalColorTexel(p00, source),
            TemporalColorTexel(p10, source),
            amount.x),
        mix(
            TemporalColorTexel(p01, source),
            TemporalColorTexel(p11, source),
            amount.x),
        amount.y);
}

bool CachedUvToCurrentUv(
    vec2 cachedUv,
    float viewDepth,
    out vec2 currentUv) {
    const float projectionX = postProcess.temporalReprojection.y;
    const float projectionY = postProcess.temporalReprojection.z;
    if (!ValidDepth(viewDepth) ||
        abs(projectionX) <= 1.0e-6 ||
        abs(projectionY) <= 1.0e-6) {
        return false;
    }

    const vec2 cachedNdc = (cachedUv * 2.0) - 1.0;
    const vec4 cachedView = vec4(
        cachedNdc.x * viewDepth / projectionX,
        cachedNdc.y * viewDepth / projectionY,
        -viewDepth,
        1.0);
    const vec4 currentClip =
        postProcess.staleViewToCurrentClip * cachedView;
    if (any(isnan(currentClip)) || any(isinf(currentClip)) ||
        currentClip.w <= 1.0e-6) {
        return false;
    }
    currentUv = ((currentClip.xy / currentClip.w) * 0.5) + 0.5;
    return !any(isnan(currentUv)) && !any(isinf(currentUv));
}

vec2 ReprojectTemporalUv(vec2 targetUv, ivec2 size, int source) {
    const int staleSource = int(round(postProcess.temporalReprojection.x));
    if (source != staleSource || staleSource == 0) {
        return targetUv;
    }

    // Solve the inverse warp iteratively. Linear depth reconstructs the
    // cached view-space point; where the point cloud leaves a sparse hole,
    // focus depth supplies a stable low-frequency camera-motion estimate.
    vec2 cachedUv = targetUv;
    const float referenceDepth =
        max(0.001, postProcess.temporalReprojection.w);
    float initialDepth = DepthAt(TemporalCoord(cachedUv, size), source);
    const bool initialDepthValid = ValidDepth(initialDepth);
    if (!initialDepthValid) {
        initialDepth = referenceDepth;
    }

    for (int iteration = 0; iteration < 3; ++iteration) {
        float depth = DepthAt(TemporalCoord(cachedUv, size), source);
        if (!ValidDepth(depth)) {
            depth = iteration == 0 ? initialDepth : referenceDepth;
        }
        vec2 projectedUv;
        if (!CachedUvToCurrentUv(cachedUv, depth, projectedUv)) {
            return targetUv;
        }
        const vec2 correction = clamp(
            targetUv - projectedUv,
            vec2(-0.08),
            vec2(0.08));
        cachedUv += correction;
        if (any(lessThan(cachedUv, vec2(-0.20))) ||
            any(greaterThan(cachedUv, vec2(1.20)))) {
            return targetUv;
        }
    }

    float finalDepth = DepthAt(TemporalCoord(cachedUv, size), source);
    const bool finalDepthValid = ValidDepth(finalDepth);
    if (!finalDepthValid) {
        finalDepth = referenceDepth;
    }
    vec2 finalProjectedUv;
    if (!CachedUvToCurrentUv(cachedUv, finalDepth, finalProjectedUv)) {
        return targetUv;
    }

    const vec2 imageSize = vec2(size);
    const float residualPixels =
        length((targetUv - finalProjectedUv) * imageSize);
    float confidence =
        1.0 - smoothstep(0.75, 4.0, residualPixels);
    if (initialDepthValid && finalDepthValid) {
        const float depthDiscontinuity =
            abs(LogDepth(finalDepth) - LogDepth(initialDepth));
        confidence *=
            1.0 - smoothstep(0.35, 1.50, depthDiscontinuity);
    }

    vec2 offset = cachedUv - targetUv;
    const float offsetPixels = length(offset * imageSize);
    const float maxMotionPixels =
        max(2.0, min(imageSize.x, imageSize.y) * 0.06);
    if (offsetPixels > maxMotionPixels) {
        offset *= maxMotionPixels / max(offsetPixels, 1.0e-6);
    }
    const vec2 halfTexel = 0.5 / imageSize;
    const vec2 warpedUv = clamp(
        targetUv + offset,
        halfTexel,
        vec2(1.0) - halfTexel);
    const vec2 edgeDistancePixels =
        min(warpedUv, vec2(1.0) - warpedUv) * imageSize;
    confidence *= smoothstep(
        0.5,
        3.0,
        min(edgeDistancePixels.x, edgeDistancePixels.y));
    return mix(targetUv, warpedUv, clamp(confidence, 0.0, 1.0));
}

void main() {
    const ivec2 size = textureSize(sceneColorInput, 0);
    const ivec2 coord = clamp(ivec2(gl_FragCoord.xy), ivec2(0), size - ivec2(1));
    const vec4 sceneColor = texelFetch(sceneColorInput, coord, 0);
    const vec4 current = FinishSceneColor(
        sceneColor,
        EyeDomeLightingShade(coord, size, 0));
    if (postProcess.temporalOverlay.x <= 0.5) {
        outColor = current;
        return;
    }

    const int validityMask = int(round(postProcess.temporalOverlay.w));
    const bool firstValid = (validityMask & 1) != 0;
    const bool secondValid = (validityMask & 2) != 0;
    if (!firstValid && !secondValid) {
        outColor = current;
        return;
    }

    const int overlayMode = int(round(postProcess.temporalOverlay.x));
    const bool splitView = overlayMode == 2;
    const bool alphaOverView = overlayMode == 3;
    const bool imageSpaceView = splitView || alphaOverView;
    const vec2 targetUv =
        (vec2(coord) + vec2(0.5)) / vec2(size);
    vec4 first = vec4(0.0);
    vec4 second = vec4(0.0);
    if (firstValid) {
        const vec2 firstUv = imageSpaceView
            ? targetUv
            : ReprojectTemporalUv(targetUv, size, 1);
        const ivec2 firstCoord = TemporalCoord(firstUv, size);
        const vec4 firstScene = TemporalColorAt(firstUv, size, 1);
        const float firstShade =
            EyeDomeLightingShade(firstCoord, size, 1);
        first = alphaOverView
            ? ShadeSceneColor(firstScene, firstShade)
            : FinishSceneColor(firstScene, firstShade);
    }
    if (secondValid) {
        const vec2 secondUv = imageSpaceView
            ? targetUv
            : ReprojectTemporalUv(targetUv, size, 2);
        const ivec2 secondCoord = TemporalCoord(secondUv, size);
        const vec4 secondScene = TemporalColorAt(secondUv, size, 2);
        const float secondShade =
            EyeDomeLightingShade(secondCoord, size, 2);
        second = alphaOverView
            ? ShadeSceneColor(secondScene, secondShade)
            : FinishSceneColor(secondScene, secondShade);
    }
    if (!firstValid) {
        outColor = alphaOverView
            ? ApplyAlphaPreview(
                  CompositeStraightAlphaOver(
                      second,
                      vec4(0.0),
                      postProcess.temporalOverlay.z,
                      0.0))
            : second;
        return;
    }
    if (!secondValid) {
        outColor = alphaOverView
            ? ApplyAlphaPreview(
                  CompositeStraightAlphaOver(
                      first,
                      vec4(0.0),
                      postProcess.temporalOverlay.y,
                      0.0))
            : first;
        return;
    }

    if (splitView) {
        const float centre = postProcess.temporalReprojection.z;
        const bool useLeft = targetUv.x < centre;
        const bool firstOnLeft =
            postProcess.temporalReprojection.w > 0.5;
        outColor = firstOnLeft
            ? (useLeft ? first : second)
            : (useLeft ? second : first);
        return;
    }

    if (alphaOverView) {
        const bool firstOnTop =
            postProcess.temporalReprojection.w > 0.5;
        outColor = ApplyAlphaPreview(
            firstOnTop
                ? CompositeStraightAlphaOver(
                      first,
                      second,
                      postProcess.temporalOverlay.y,
                      postProcess.temporalOverlay.z)
                : CompositeStraightAlphaOver(
                      second,
                      first,
                      postProcess.temporalOverlay.z,
                      postProcess.temporalOverlay.y));
        return;
    }

    float firstWeight = max(0.0, postProcess.temporalOverlay.y);
    float secondWeight = max(0.0, postProcess.temporalOverlay.z);
    const float weightSum = firstWeight + secondWeight;
    if (weightSum <= 1.0e-6) {
        firstWeight = 0.5;
        secondWeight = 0.5;
    } else {
        firstWeight /= weightSum;
        secondWeight /= weightSum;
    }
    outColor = first * firstWeight + second * secondWeight;
}
