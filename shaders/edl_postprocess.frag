#version 450

layout(location = 0) in vec2 inUv;
layout(location = 0) out vec4 outColor;

layout(set = 0, binding = 0) uniform sampler2D sceneColorInput;
layout(set = 0, binding = 1) uniform sampler2D linearDepthInput;

layout(push_constant) uniform PostProcessData {
    vec4 edl;
} postProcess;

bool ValidDepth(float depth) {
    return !isnan(depth) && !isinf(depth) && depth > 0.0;
}

float LogDepth(float depth) {
    return log2(max(depth, 1.0e-6));
}

float EyeDomeLightingShade(ivec2 coord, ivec2 size) {
    if (postProcess.edl.x <= 0.5) {
        return 1.0;
    }

    const float centerDepth = texelFetch(linearDepthInput, coord, 0).r;
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

            const float neighborDepth = texelFetch(linearDepthInput, sampleCoord, 0).r;
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

void main() {
    const ivec2 size = textureSize(sceneColorInput, 0);
    const ivec2 coord = clamp(ivec2(gl_FragCoord.xy), ivec2(0), size - ivec2(1));
    const vec4 sceneColor = texelFetch(sceneColorInput, coord, 0);
    const float shade = EyeDomeLightingShade(coord, size);
    outColor = vec4(sceneColor.rgb * shade, sceneColor.a);
}
