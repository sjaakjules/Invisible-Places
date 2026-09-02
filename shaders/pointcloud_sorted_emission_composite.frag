#version 450

layout(location = 0) in vec2 inUv;
layout(location = 0) out vec4 outColor;

// Runs after the sorted-alpha colour subpass. Sorted layers accumulate their
// emission additively — the same write the weighted-blended path performs —
// and this pass applies the identical exponential response on top of the
// already-composited colour, so toggling GPU sorting never changes emissive
// levels. Binding 2 matches the emission slot of the shared weighted
// composite descriptor set; input attachment 0 is this subpass's only input.
layout(input_attachment_index = 0, set = 0, binding = 2) uniform subpassInput emissionInput;

void main() {
    const vec4 emissionRaw = max(subpassLoad(emissionInput), vec4(0.0));
    const vec3 emission = vec3(1.0) - exp(-emissionRaw.rgb);
    const float emissionAlpha = clamp(1.0 - exp(-emissionRaw.a), 0.0, 1.0);
    // The pipeline adds colour (ONE/ONE) and takes MAX on alpha, mirroring
    // how the weighted composite folds emission into its output coverage.
    outColor = vec4(emission, emissionAlpha);
}
