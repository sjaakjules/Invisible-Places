// Scientific palettes are generated from Matplotlib v3.10.8 lib/matplotlib/_cm_listed.py data.
// Shader tables are downsampled to 32 entries; terrain and contrast palettes are procedural stops.
const int kPointCloudColormapSampleCount = 32;

const uint kViridisColormap[kPointCloudColormapSampleCount] = uint[](
    0x440154u, 0x470D60u, 0x48196Bu, 0x482474u, 0x472E7Cu, 0x453882u, 0x414286u, 0x3E4B89u,
    0x3A548Cu, 0x365D8Du, 0x32658Eu, 0x2E6D8Eu, 0x2B758Eu, 0x287D8Eu, 0x25858Eu, 0x228C8Du,
    0x20948Cu, 0x1E9C89u, 0x20A386u, 0x25AB82u, 0x2DB27Du, 0x39BA76u, 0x48C16Eu, 0x58C765u,
    0x6ACD5Bu, 0x7ED34Fu, 0x92D742u, 0xA8DB34u, 0xBEDF26u, 0xD4E21Bu, 0xE9E41Au, 0xFDE725u
);

const uint kPlasmaColormap[kPointCloudColormapSampleCount] = uint[](
    0x0D0887u, 0x220690u, 0x320597u, 0x40049Du, 0x4E02A2u, 0x5B01A5u, 0x6800A8u, 0x7501A8u,
    0x8104A7u, 0x8D0BA5u, 0x9814A0u, 0xA31D9Au, 0xAD2693u, 0xB6308Bu, 0xBF3984u, 0xC7427Cu,
    0xCF4C74u, 0xD6556Du, 0xDD5E66u, 0xE3685Fu, 0xE97258u, 0xEE7C51u, 0xF3874Au, 0xF79243u,
    0xFA9D3Bu, 0xFCA935u, 0xFDB52Eu, 0xFDC229u, 0xFCCF25u, 0xF9DD24u, 0xF5EB27u, 0xF0F921u
);

const uint kInfernoColormap[kPointCloudColormapSampleCount] = uint[](
    0x000004u, 0x040313u, 0x0B0725u, 0x160B39u, 0x220C4Cu, 0x310A5Cu, 0x3F0A66u, 0x4D0C6Bu,
    0x5A116Eu, 0x67166Eu, 0x741B6Eu, 0x811F6Cu, 0x8E2469u, 0x9B2964u, 0xA82E5Fu, 0xB53358u,
    0xC13A51u, 0xCC4248u, 0xD74B3Fu, 0xE05536u, 0xE8612Cu, 0xEF6D22u, 0xF57B17u, 0xF8890Cu,
    0xFB9806u, 0xFCA80Du, 0xFBB81Cu, 0xF9C830u, 0xF6D847u, 0xF2E763u, 0xF3F585u, 0xFCFFA4u
);

const uint kMagmaColormap[kPointCloudColormapSampleCount] = uint[](
    0x000004u, 0x040312u, 0x0B0823u, 0x140E35u, 0x1E1149u, 0x2A115Du, 0x38106Du, 0x461077u,
    0x54137Du, 0x601880u, 0x6D1E81u, 0x7A2382u, 0x872781u, 0x942C80u, 0xA2307Eu, 0xAF347Bu,
    0xBD3977u, 0xCA3E72u, 0xD6456Cu, 0xE24D66u, 0xEC5860u, 0xF3655Cu, 0xF8745Cu, 0xFB8360u,
    0xFD9366u, 0xFEA26Fu, 0xFEB27Au, 0xFEC185u, 0xFED093u, 0xFDDFA1u, 0xFCEEB0u, 0xFCFDBFu
);

const uint kCividisColormap[kPointCloudColormapSampleCount] = uint[](
    0x00224Eu, 0x00285Cu, 0x002E6Bu, 0x073370u, 0x1C396Fu, 0x293F6Eu, 0x33446Cu, 0x3D4A6Cu,
    0x45506Cu, 0x4D556Cu, 0x555B6Du, 0x5C616Eu, 0x636770u, 0x6B6D72u, 0x727274u, 0x797877u,
    0x807E78u, 0x888578u, 0x908B78u, 0x989177u, 0xA09775u, 0xA89E73u, 0xB0A471u, 0xB9AB6Du,
    0xC1B26Au, 0xCAB965u, 0xD3C060u, 0xDCC859u, 0xE5CF52u, 0xEED748u, 0xF8DF3Cu, 0xFEE838u
);

const uint kTurboColormap[kPointCloudColormapSampleCount] = uint[](
    0x30123Bu, 0x3A2A74u, 0x4042A4u, 0x4558CAu, 0x476EE6u, 0x4683F8u, 0x4097FFu, 0x34ACF8u,
    0x25C0E7u, 0x1AD2D2u, 0x18E0BDu, 0x24ECA8u, 0x3BF58Fu, 0x59FB73u, 0x79FE59u, 0x97FE44u,
    0xAEFA37u, 0xC3F234u, 0xD7E535u, 0xE8D639u, 0xF4C63Au, 0xFCB437u, 0xFE9E2Fu, 0xFC8624u,
    0xF76D1Au, 0xEE5610u, 0xE2430Au, 0xD43305u, 0xC22403u, 0xAD1801u, 0x950D01u, 0x7A0403u
);

vec3 UnpackPointCloudColormapRgb(uint packedRgb) {
    return vec3(
        float((packedRgb >> 16u) & 0xFFu),
        float((packedRgb >> 8u) & 0xFFu),
        float(packedRgb & 0xFFu)) / 255.0;
}

vec3 SamplePointCloudColormapLut(const uint samples[kPointCloudColormapSampleCount], float value) {
    const float scaled = clamp(value, 0.0, 1.0) * float(kPointCloudColormapSampleCount - 1);
    const int lowerIndex = int(floor(scaled));
    const int upperIndex = min(lowerIndex + 1, kPointCloudColormapSampleCount - 1);
    const float mixAmount = scaled - float(lowerIndex);
    return mix(
        UnpackPointCloudColormapRgb(samples[lowerIndex]),
        UnpackPointCloudColormapRgb(samples[upperIndex]),
        mixAmount);
}

vec3 SamplePointCloudColormapSegment(float value, float lower, float upper, vec3 lowerColor, vec3 upperColor) {
    const float mixAmount = clamp((value - lower) / max(upper - lower, 1.0e-6), 0.0, 1.0);
    return mix(lowerColor, upperColor, mixAmount);
}

float PointCloudExponentialColormapCurve(float value, float exponent) {
    const float clamped = clamp(value, 0.0, 1.0);
    const float denominator = exp(exponent) - 1.0;
    if (denominator <= 1.0e-6) {
        return clamped;
    }
    return (exp(exponent * clamped) - 1.0) / denominator;
}

vec3 SampleTopographicColormap(float value) {
    const float v = clamp(value, 0.0, 1.0);
    if (v <= 0.16) {
        return SamplePointCloudColormapSegment(v, 0.00, 0.16, vec3(0.03, 0.12, 0.28), vec3(0.10, 0.36, 0.58));
    }
    if (v <= 0.28) {
        return SamplePointCloudColormapSegment(v, 0.16, 0.28, vec3(0.10, 0.36, 0.58), vec3(0.50, 0.76, 0.69));
    }
    if (v <= 0.34) {
        return SamplePointCloudColormapSegment(v, 0.28, 0.34, vec3(0.50, 0.76, 0.69), vec3(0.78, 0.70, 0.45));
    }
    if (v <= 0.48) {
        return SamplePointCloudColormapSegment(v, 0.34, 0.48, vec3(0.78, 0.70, 0.45), vec3(0.28, 0.55, 0.24));
    }
    if (v <= 0.64) {
        return SamplePointCloudColormapSegment(v, 0.48, 0.64, vec3(0.28, 0.55, 0.24), vec3(0.58, 0.50, 0.30));
    }
    if (v <= 0.80) {
        return SamplePointCloudColormapSegment(v, 0.64, 0.80, vec3(0.58, 0.50, 0.30), vec3(0.50, 0.42, 0.38));
    }
    return SamplePointCloudColormapSegment(v, 0.80, 1.00, vec3(0.50, 0.42, 0.38), vec3(0.96, 0.95, 0.90));
}

vec3 SampleLandSurfaceColormap(float value) {
    const float v = clamp(value, 0.0, 1.0);
    if (v <= 0.18) {
        return SamplePointCloudColormapSegment(v, 0.00, 0.18, vec3(0.10, 0.13, 0.06), vec3(0.23, 0.31, 0.12));
    }
    if (v <= 0.38) {
        return SamplePointCloudColormapSegment(v, 0.18, 0.38, vec3(0.23, 0.31, 0.12), vec3(0.38, 0.52, 0.20));
    }
    if (v <= 0.58) {
        return SamplePointCloudColormapSegment(v, 0.38, 0.58, vec3(0.38, 0.52, 0.20), vec3(0.65, 0.58, 0.32));
    }
    if (v <= 0.76) {
        return SamplePointCloudColormapSegment(v, 0.58, 0.76, vec3(0.65, 0.58, 0.32), vec3(0.72, 0.45, 0.25));
    }
    if (v <= 0.90) {
        return SamplePointCloudColormapSegment(v, 0.76, 0.90, vec3(0.72, 0.45, 0.25), vec3(0.55, 0.50, 0.45));
    }
    return SamplePointCloudColormapSegment(v, 0.90, 1.00, vec3(0.55, 0.50, 0.45), vec3(0.86, 0.82, 0.72));
}

vec3 SampleExponentialFireColormap(float value) {
    const float v = PointCloudExponentialColormapCurve(value, 3.0);
    if (v <= 0.30) {
        return SamplePointCloudColormapSegment(v, 0.00, 0.30, vec3(0.00, 0.00, 0.00), vec3(0.25, 0.00, 0.06));
    }
    if (v <= 0.55) {
        return SamplePointCloudColormapSegment(v, 0.30, 0.55, vec3(0.25, 0.00, 0.06), vec3(0.85, 0.10, 0.02));
    }
    if (v <= 0.76) {
        return SamplePointCloudColormapSegment(v, 0.55, 0.76, vec3(0.85, 0.10, 0.02), vec3(1.00, 0.48, 0.00));
    }
    if (v <= 0.92) {
        return SamplePointCloudColormapSegment(v, 0.76, 0.92, vec3(1.00, 0.48, 0.00), vec3(1.00, 0.92, 0.20));
    }
    return SamplePointCloudColormapSegment(v, 0.92, 1.00, vec3(1.00, 0.92, 0.20), vec3(1.00, 1.00, 0.92));
}

vec3 SampleExponentialIceColormap(float value) {
    const float v = PointCloudExponentialColormapCurve(value, 2.7);
    if (v <= 0.35) {
        return SamplePointCloudColormapSegment(v, 0.00, 0.35, vec3(0.00, 0.02, 0.08), vec3(0.02, 0.14, 0.36));
    }
    if (v <= 0.60) {
        return SamplePointCloudColormapSegment(v, 0.35, 0.60, vec3(0.02, 0.14, 0.36), vec3(0.04, 0.48, 0.78));
    }
    if (v <= 0.82) {
        return SamplePointCloudColormapSegment(v, 0.60, 0.82, vec3(0.04, 0.48, 0.78), vec3(0.38, 0.86, 0.95));
    }
    return SamplePointCloudColormapSegment(v, 0.82, 1.00, vec3(0.38, 0.86, 0.95), vec3(0.96, 1.00, 1.00));
}

vec3 SampleHighContrastColormap(float value) {
    const float v = clamp(value, 0.0, 1.0);
    if (v <= 0.12) {
        return SamplePointCloudColormapSegment(v, 0.00, 0.12, vec3(0.00, 0.00, 0.00), vec3(0.10, 0.00, 0.32));
    }
    if (v <= 0.32) {
        return SamplePointCloudColormapSegment(v, 0.12, 0.32, vec3(0.10, 0.00, 0.32), vec3(0.00, 0.12, 0.85));
    }
    if (v <= 0.50) {
        return SamplePointCloudColormapSegment(v, 0.32, 0.50, vec3(0.00, 0.12, 0.85), vec3(0.00, 0.82, 0.95));
    }
    if (v <= 0.68) {
        return SamplePointCloudColormapSegment(v, 0.50, 0.68, vec3(0.00, 0.82, 0.95), vec3(0.98, 0.92, 0.00));
    }
    if (v <= 0.84) {
        return SamplePointCloudColormapSegment(v, 0.68, 0.84, vec3(0.98, 0.92, 0.00), vec3(1.00, 0.26, 0.00));
    }
    return SamplePointCloudColormapSegment(v, 0.84, 1.00, vec3(1.00, 0.26, 0.00), vec3(1.00, 1.00, 1.00));
}

vec3 ApplyPointCloudColormap(uint colormapId, float value) {
    if (colormapId == 1u) {
        return SamplePointCloudColormapLut(kPlasmaColormap, value);
    }
    if (colormapId == 2u) {
        return SamplePointCloudColormapLut(kInfernoColormap, value);
    }
    if (colormapId == 3u) {
        return SamplePointCloudColormapLut(kMagmaColormap, value);
    }
    if (colormapId == 4u) {
        return SamplePointCloudColormapLut(kCividisColormap, value);
    }
    if (colormapId == 5u) {
        return SamplePointCloudColormapLut(kTurboColormap, value);
    }
    if (colormapId == 6u) {
        return SampleTopographicColormap(value);
    }
    if (colormapId == 7u) {
        return SampleLandSurfaceColormap(value);
    }
    if (colormapId == 8u) {
        return SampleExponentialFireColormap(value);
    }
    if (colormapId == 9u) {
        return SampleExponentialIceColormap(value);
    }
    if (colormapId == 10u) {
        return SampleHighContrastColormap(value);
    }
    return SamplePointCloudColormapLut(kViridisColormap, value);
}

vec3 ApplyPointCloudColormapOrGradient(uint colormapId, float value, vec3 gradientStart, vec3 gradientEnd) {
    if (colormapId == 11u) {
        return mix(gradientStart, gradientEnd, clamp(value, 0.0, 1.0));
    }
    return ApplyPointCloudColormap(colormapId, value);
}
