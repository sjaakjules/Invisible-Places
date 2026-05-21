#version 450

layout(location = 0) in vec2 inUv;
layout(location = 0) out vec4 outColor;
layout(location = 1) out vec4 outNormal;
layout(location = 2) out vec4 outAlbedo;

layout(input_attachment_index = 0, set = 0, binding = 0) uniform subpassInput accumulationInput;
layout(input_attachment_index = 1, set = 0, binding = 1) uniform subpassInput revealageInput;
layout(input_attachment_index = 2, set = 0, binding = 2) uniform subpassInput emissionInput;
layout(input_attachment_index = 3, set = 0, binding = 3) uniform subpassInput normalAccumulationInput;
layout(input_attachment_index = 4, set = 0, binding = 4) uniform subpassInput albedoAccumulationInput;

vec3 ResolveNormal(vec4 accumulation) {
    if (accumulation.a <= 1e-5) {
        return vec3(0.0);
    }

    vec3 normal = accumulation.rgb / accumulation.a;
    if (dot(normal, normal) <= 1e-8) {
        return vec3(0.0);
    }
    return normalize(normal);
}

vec3 ResolveAlbedo(vec4 accumulation) {
    if (accumulation.a <= 1e-5) {
        return vec3(0.0);
    }
    return max(accumulation.rgb / accumulation.a, vec3(0.0));
}

void main() {
    const vec4 accumulation = subpassLoad(accumulationInput);
    const float revealage = subpassLoad(revealageInput).r;
    const float alpha = clamp(1.0 - revealage, 0.0, 1.0);
    const vec4 emissionRaw = max(subpassLoad(emissionInput), vec4(0.0));
    const vec3 emission = vec3(1.0) - exp(-emissionRaw.rgb);
    const float emissionAlpha = clamp(1.0 - exp(-emissionRaw.a), 0.0, 1.0);

    const vec3 normal = ResolveNormal(subpassLoad(normalAccumulationInput));
    const vec3 albedo = ResolveAlbedo(subpassLoad(albedoAccumulationInput));
    outNormal = vec4(normal, alpha);
    outAlbedo = vec4(albedo, alpha);

    if (alpha <= 1e-5 && emissionAlpha <= 1e-5) {
        outColor = vec4(0.0);
        outNormal = vec4(0.0);
        outAlbedo = vec4(0.0);
        return;
    }

    const vec3 transparentColor = accumulation.rgb / max(accumulation.a, 1e-5);
    const vec3 desiredContribution = (transparentColor * alpha) + emission;
    const float outputAlpha = max(alpha, emissionAlpha);
    outColor = vec4(desiredContribution / max(outputAlpha, 1e-5), outputAlpha);
}
