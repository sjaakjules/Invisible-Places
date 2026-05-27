#version 450

layout(location = 1) in float inViewDepth;
layout(location = 2) flat in uint inPointIndex;

layout(location = 0) out float outLinearDepth;

layout(set = 0, binding = 1, std430) readonly buffer ScalarFieldValues {
    float values[];
} scalarFieldValues;

struct RenderParameterBindingGpu {
    vec4 constantValue;
    vec4 range;
    vec4 extra;
    uvec4 control;
};

layout(set = 0, binding = 2, std140) uniform PointStyleData {
    vec4 solidColor;
    uvec4 globalControl;
    uvec4 pointMeta;
    uvec4 renderControl;
    vec4 renderParams0;
    vec4 renderParams1;
    vec4 renderParams2;
    vec4 renderParams3;
    RenderParameterBindingGpu pointSizeBinding;
    RenderParameterBindingGpu opacityBinding;
    RenderParameterBindingGpu emissiveBinding;
    RenderParameterBindingGpu xrayBinding;
    RenderParameterBindingGpu depthFadeBinding;
    RenderParameterBindingGpu colormapPositionBinding;
    RenderParameterBindingGpu surfelDiameterBinding;
    vec4 colorize;
    uvec4 stylisationControl;
    vec4 stylisationParams0;
    vec4 stylisationParams1;
    vec4 stylisationParams2;
} styleData;

const uint kWaterParticleRoleFieldSlot = 9u;
const uint kWaterJitterSeedFieldSlot = 12u;
const uint kWaterStreamTangentZFieldSlot = 18u;

float LoadScalarFieldValue(uint fieldSlot) {
    if (fieldSlot == 0xFFFFFFFFu ||
        fieldSlot >= styleData.globalControl.z ||
        styleData.pointMeta.x == 0u ||
        inPointIndex >= styleData.pointMeta.x) {
        return 0.0;
    }
    return scalarFieldValues.values[(fieldSlot * styleData.pointMeta.x) + inPointIndex];
}

void main() {
    if (styleData.pointMeta.w == 3u && styleData.globalControl.z > kWaterStreamTangentZFieldSlot) {
        outLinearDepth = inViewDepth;
        return;
    }
    if (styleData.pointMeta.w != 0u && styleData.globalControl.z > kWaterJitterSeedFieldSlot) {
        const float role = LoadScalarFieldValue(kWaterParticleRoleFieldSlot);
        if (styleData.pointMeta.w == 2u) {
            if (!((role >= 0.5 && role < 1.5) || (role >= 1.5 && role < 2.5) || (role >= 2.5 && role < 3.5))) {
                discard;
            }
        } else if (role < 0.5 || role >= 1.5) {
            discard;
        }
    }
    outLinearDepth = inViewDepth;
}
