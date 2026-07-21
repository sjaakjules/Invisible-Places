#version 450

layout(location = 0) out vec2 outUv;
layout(location = 1) out vec4 outColourOpacity;
layout(location = 2) out float outEmission;
layout(location = 3) out float outViewDepth;
layout(location = 4) out float outSoftness;

struct RainParticle {
    vec4 positionAge;
    vec4 previousActivity;
    vec4 velocity;
    uvec4 state;
};

layout(set = 0, binding = 0, std140) uniform RainUniforms {
    mat4 viewProjection;
    mat4 view;
    mat4 projection;
    vec4 cameraTime;
    vec4 spawnCentreRadius;
    vec4 cacheBoundsMinResolution;
    vec4 cacheBoundsMaxDeathDistance;
    vec4 weather0;
    vec4 weather1;
    vec4 weather2;
    vec4 visual0;
    vec4 visual1;
    vec4 visual2;
    uvec4 simulation0;
    uvec4 simulation1;
    uvec4 collision0;
    vec4 impactGrid;
    uvec4 effectToggles;
    vec4 effectScales;
    vec4 viewport;
} rain;

layout(set = 0, binding = 1, std430) readonly buffer RainParticles {
    RainParticle particles[];
};

const vec2 kCorners[6] = vec2[](
    vec2(-1.0, 0.0),
    vec2(1.0, 0.0),
    vec2(1.0, 1.0),
    vec2(-1.0, 0.0),
    vec2(1.0, 1.0),
    vec2(-1.0, 1.0));

void main() {
    const uint particleIndex = uint(gl_InstanceIndex);
    const RainParticle particle = particles[particleIndex];
    const vec2 corner = kCorners[gl_VertexIndex];
    if (particle.state.x == 0u || particleIndex >= rain.simulation0.x || rain.simulation1.x == 0u) {
        gl_Position = vec4(2.0, 2.0, 2.0, 1.0);
        outUv = corner;
        outColourOpacity = vec4(0.0);
        outEmission = 0.0;
        outViewDepth = 1.0;
        outSoftness = 0.0;
        return;
    }

    const vec3 centre = particle.positionAge.xyz;
    vec3 direction = particle.velocity.xyz;
    direction = dot(direction, direction) > 1e-8 ? normalize(direction) : vec3(0.0, 0.0, -1.0);
    vec3 viewDirection = centre - rain.cameraTime.xyz;
    viewDirection = dot(viewDirection, viewDirection) > 1e-8 ? normalize(viewDirection) : vec3(0.0, 1.0, 0.0);
    vec3 side = cross(viewDirection, direction);
    if (dot(side, side) <= 1e-8) {
        side = vec3(rain.view[0][0], rain.view[1][0], rain.view[2][0]);
    }
    side = normalize(side);
    const float viewDepth = max(0.001, -(rain.view * vec4(centre, 1.0)).z);
    const float pixelWorldSpan =
        2.0 * viewDepth / max(1e-5, abs(rain.projection[1][1])) / max(1.0, rain.viewport.y);
    const float authoredWidth = rain.visual0.x * rain.visual2.z;
    const float width = clamp(
        authoredWidth,
        rain.visual2.x * pixelWorldSpan,
        max(rain.visual2.x, rain.visual2.y) * pixelWorldSpan);
    const float length = max(width, rain.visual0.y * rain.visual2.z);
    const vec3 worldPosition =
        centre - direction * length * corner.y + side * width * corner.x * 0.5;
    gl_Position = rain.viewProjection * vec4(worldPosition, 1.0);
    outUv = corner;
    outColourOpacity = vec4(rain.visual1.rgb, rain.visual0.w * particle.previousActivity.w);
    outEmission = rain.visual1.w;
    outViewDepth = viewDepth;
    outSoftness = rain.visual0.z;
}
