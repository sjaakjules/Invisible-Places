#version 450

layout(location = 0) in vec2 inUv;
layout(location = 0) out vec4 outColor;

layout(input_attachment_index = 0, set = 0, binding = 0) uniform subpassInput accumulationInput;
layout(input_attachment_index = 1, set = 0, binding = 1) uniform subpassInput revealageInput;

void main() {
    const vec4 accumulation = subpassLoad(accumulationInput);
    const float revealage = subpassLoad(revealageInput).r;
    const float alpha = clamp(1.0 - revealage, 0.0, 1.0);

    if (alpha <= 1e-5) {
        outColor = vec4(0.0);
        return;
    }

    const vec3 color = accumulation.rgb / max(accumulation.a, 1e-5);
    outColor = vec4(color, alpha);
}
