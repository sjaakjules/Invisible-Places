#version 450

layout(location = 1) in float inViewDepth;

layout(location = 0) out float outLinearDepth;

void main() {
    outLinearDepth = inViewDepth;
}
