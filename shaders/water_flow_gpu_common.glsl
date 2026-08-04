// Shared contract for source-local Flow route and trail compute passes.
// Output scalars are field-major and preserve WaterTrailSample's 31-slot order.

const uint kTrailRoleFieldSlot = 0u;
const uint kTrailIdFieldSlot = 1u;
const uint kSourceIdFieldSlot = 2u;
const uint kPathIdFieldSlot = 3u;
const uint kBranchIdFieldSlot = 4u;
const uint kTrailSeedFieldSlot = 5u;
const uint kPointSeedFieldSlot = 6u;
const uint kTrailDistanceFieldSlot = 7u;
const uint kTrailLengthFieldSlot = 8u;
const uint kRouteStartFieldSlot = 9u;
const uint kRouteCountFieldSlot = 10u;
const uint kRouteLengthFieldSlot = 11u;
const uint kTrailStartPhaseFieldSlot = 12u;
const uint kTrailLateralOffsetFieldSlot = 13u;
const uint kPointAgeFieldSlot = 14u;
const uint kTrailAgeFieldSlot = 15u;
const uint kTrailSpeedFieldSlot = 16u;
const uint kTrailWidthFieldSlot = 17u;
const uint kTrailStreakLengthFieldSlot = 18u;
const uint kTrailConfidenceFieldSlot = 19u;
const uint kWetnessFieldSlot = 20u;
const uint kFeatureTypeFieldSlot = 21u;
const uint kTangentXFieldSlot = 22u;
const uint kTangentYFieldSlot = 23u;
const uint kTangentZFieldSlot = 24u;
const uint kLaneIndexFieldSlot = 25u;
const uint kLaneCountFieldSlot = 26u;
const uint kLanePitchFieldSlot = 27u;
const uint kLaneSpanFieldSlot = 28u;
const uint kLaneCrossingFieldSlot = 29u;
const uint kCrossSeedFieldSlot = 30u;
const uint kEndpointFadeFlagsFieldSlot = 31u;
const uint kStartFadeFullDistanceFieldSlot = 32u;
const uint kStartFadeRandomBeginDistanceFieldSlot = 33u;
const uint kEndFadeFullDistanceFieldSlot = 34u;
const uint kEndFadeRandomBeginDistanceFieldSlot = 35u;
const uint kWaterTrailScalarFieldCount = 36u;
const int kEmptyCoordinate = -2147483648;

struct WaterFlowInputPoint {
    vec4 positionDistance;
    vec4 normalConfidence;
    vec4 outgoingArcDistances;
    vec4 laneWidth; // mode, value, reserved, reserved
};

struct WaterFlowBranch {
    uvec4 inputRoute; // input start/count, route start, route points/lane
    uvec4 trailLane;  // active route lanes, trail output start/count, first trail id
    uvec4 identity;   // potential lanes, branch id, one-based path id, input kind
    vec4 metrics;     // route length, route spacing, allocation weight, reserved
};

// Matches water::WaterGpuSurfaceSurfelSlot (32 bytes).
struct WaterSurfaceSurfelSlot {
    ivec3 coordinate;
    uint roleAndSampleCount;
    uvec4 packed;
};

layout(set = 0, binding = 0, std140) uniform WaterFlowSourceUniforms {
    uvec4 counts0;  // input points, branches, max active route lanes, total route points
    uvec4 counts1;  // trails, samples/trail, active points, point capacity
    uvec4 surface0; // table mask, max probe, valid, source id
    uvec4 metadata; // seed, input kind, use surface guide, source revision low
    uvec4 identity; // source id, endpoint fade flags, reserved
    vec4 route0;    // aggregate route length, nominal spacing, surface resolution, surface offset
    vec4 lane0;     // lane span, trail width, turbulence, turbulence scale
    vec4 guide0;    // surface follow, downhill pull, terrain width response, path attraction
    vec4 trail0;    // trail length, point spacing, speed, streak length
    vec4 shape0;    // lane crossing, smoothness, looseness, reserved
    vec4 fade0;     // start full, start random begin, end full, end random begin
} flow;

layout(set = 0, binding = 1, std430) readonly buffer WaterFlowInputPoints {
    WaterFlowInputPoint points[];
} flowInput;

layout(set = 0, binding = 2, std430) readonly buffer WaterFlowSurfaceTable {
    WaterSurfaceSurfelSlot slots[];
} flowSurface;

// Bindings 3 and 4 intentionally mirror the Dynamic Mesh Flow output layout.
// This lets both source-local passes share one compact descriptor/pipeline layout.
layout(set = 0, binding = 3, std430) buffer WaterFlowOutputPositions {
    vec4 positions[];
} flowPositions;

layout(set = 0, binding = 4, std430) buffer WaterFlowOutputNormals {
    vec4 normals[];
} flowNormals;

layout(set = 0, binding = 5, std430) buffer WaterFlowOutputScalars {
    float values[];
} flowScalars;

layout(set = 0, binding = 6, std430) readonly buffer WaterFlowBranches {
    WaterFlowBranch branches[];
} flowBranches;

uint FlowHashBits(uint value) {
    value ^= value >> 16u;
    value *= 0x7feb352du;
    value ^= value >> 15u;
    value *= 0x846ca68bu;
    return value ^ (value >> 16u);
}

uint FlowRotateLeft(uint value, uint count) {
    return (value << count) | (value >> (32u - count));
}

uint FlowHashCell(ivec3 coordinate) {
    uint hash = FlowHashBits(uint(coordinate.x));
    hash ^= FlowRotateLeft(FlowHashBits(uint(coordinate.y)), 11u);
    hash ^= FlowRotateLeft(FlowHashBits(uint(coordinate.z)), 22u);
    return FlowHashBits(hash);
}

float FlowHash01(uint a, uint b, uint c) {
    // Byte-for-byte equivalent integer mixing to CPU RegionHash01/Hash01.
    const uint value = (a * 747796405u) ^ (b * 2891336453u) ^ (c * 277803737u);
    return float(FlowHashBits(value) & 0x00ffffffu) / 16777215.0;
}

uint FlowCenteredLaneIndex(uint ordinal, uint laneCount, bool positiveSideFirst) {
    if (laneCount <= 1u) {
        return 0u;
    }
    const uint slot = ordinal % laneCount;
    const uint centreLow = (laneCount - 1u) / 2u;
    const uint canonical = slot == 0u
        ? centreLow
        : ((slot & 1u) != 0u
               ? centreLow + ((slot + 1u) / 2u)
               : centreLow - (slot / 2u));
    return positiveSideFirst ? canonical : (laneCount - 1u) - canonical;
}

vec3 FlowSafeNormalize(vec3 value, vec3 fallback) {
    const float lengthSquared = dot(value, value);
    return lengthSquared > 1e-10 ? value * inversesqrt(lengthSquared) : fallback;
}

void FlowStoreField(uint slot, uint pointIndex, float value) {
    if (slot < kWaterTrailScalarFieldCount && pointIndex < flow.counts1.w) {
        flowScalars.values[slot * flow.counts1.w + pointIndex] = value;
    }
}

float FlowLoadField(uint slot, uint pointIndex) {
    if (slot < kWaterTrailScalarFieldCount && pointIndex < flow.counts1.w) {
        return flowScalars.values[slot * flow.counts1.w + pointIndex];
    }
    return 0.0;
}

void FlowClearFields(uint pointIndex) {
    for (uint slot = 0u; slot < kWaterTrailScalarFieldCount; ++slot) {
        FlowStoreField(slot, pointIndex, 0.0);
    }
}

vec3 FlowDecodeNormal(uint packedNormal) {
    vec3 value = vec3(
        float(packedNormal & 1023u),
        float((packedNormal >> 10u) & 1023u),
        float((packedNormal >> 20u) & 1023u)) / 1023.0;
    return FlowSafeNormalize(value * 2.0 - 1.0, vec3(0.0, 0.0, 1.0));
}

vec3 FlowDecodeCentroid(WaterSurfaceSurfelSlot slot) {
    const uint packed = slot.packed.x;
    const vec3 localPosition = vec3(
        float(packed & 1023u),
        float((packed >> 10u) & 1023u),
        float((packed >> 20u) & 1023u)) / 1023.0;
    return (vec3(slot.coordinate) + localPosition) * max(flow.route0.z, 0.001);
}

bool FlowFindSurfel(ivec3 coordinate, uint role, out WaterSurfaceSurfelSlot result) {
    if (flow.surface0.z == 0u) {
        return false;
    }
    const uint initial =
        FlowHashBits(FlowHashCell(coordinate) ^ FlowHashBits(role)) & flow.surface0.x;
    const uint probeLimit = min(32u, max(1u, flow.surface0.y));
    for (uint probe = 0u; probe < probeLimit; ++probe) {
        const WaterSurfaceSurfelSlot candidate =
            flowSurface.slots[(initial + probe) & flow.surface0.x];
        if (candidate.coordinate.x == kEmptyCoordinate) {
            return false;
        }
        if (all(equal(candidate.coordinate, coordinate)) &&
            (candidate.roleAndSampleCount & 0xffu) == role) {
            result = candidate;
            return true;
        }
    }
    return false;
}

struct FlowSurfaceQuery {
    vec3 position;
    vec3 normal;
    vec3 projectedGravity;
    float confidence;
    float coherence;
    float roughness;
    float planarity;
    float transverseSupport;
    float concavity;
    float gravityStrength;
    ivec3 coordinate;
    uint role;
    bool hit;
};

// Query the already edge-preserving, preprocessed neighbour field. The
// persisted surfel stays 32 bytes: planarity, projected gravity, transverse
// support and a concavity-like groove hint are derived here from its filtered
// normal/roughness pair and immediate same-role neighbours.
FlowSurfaceQuery FlowQuerySurface(
    vec3 position,
    vec3 previousSurfacePosition,
    vec3 referenceNormal,
    vec3 routeTangent,
    vec3 routeLateral,
    uint previousRole,
    bool hasPreviousSurface,
    float radius) {
    FlowSurfaceQuery best;
    best.position = position;
    const vec3 queryNormal = FlowSafeNormalize(referenceNormal, vec3(0.0, 0.0, 1.0));
    best.normal = queryNormal;
    best.projectedGravity = vec3(0.0);
    best.confidence = 0.0;
    best.coherence = 0.0;
    best.roughness = 1.0;
    best.planarity = 0.0;
    best.transverseSupport = 0.0;
    best.concavity = 0.0;
    best.gravityStrength = 0.0;
    best.coordinate = ivec3(0);
    best.role = 0u;
    best.hit = false;
    float bestScore = 3.402823e38;
    const float resolution = max(flow.route0.z, 0.001);
    const ivec3 centre = ivec3(floor(position / resolution));
    const int cellRadius = int(clamp(ceil(radius / resolution), 1.0, 2.0));
    const vec3 safeTangent = FlowSafeNormalize(routeTangent, vec3(1.0, 0.0, 0.0));
    const vec3 predictedSurfacePosition = hasPreviousSurface
        ? previousSurfacePosition + safeTangent * max(flow.route0.y, resolution * 0.25)
        : position;
    for (int z = -2; z <= 2; ++z) {
        if (abs(z) > cellRadius) { continue; }
        for (int y = -2; y <= 2; ++y) {
            if (abs(y) > cellRadius) { continue; }
            for (int x = -2; x <= 2; ++x) {
                if (abs(x) > cellRadius) { continue; }
                const ivec3 coordinate = centre + ivec3(x, y, z);
                for (uint role = 1u; role <= 2u; ++role) {
                    WaterSurfaceSurfelSlot slot;
                    if (!FlowFindSurfel(coordinate, role, slot)) { continue; }
                    const vec3 candidatePosition = FlowDecodeCentroid(slot);
                    const vec3 delta = candidatePosition - position;
                    const float distanceToCandidate = length(delta);
                    if (distanceToCandidate > radius) { continue; }
                    vec3 candidateNormal = FlowDecodeNormal(slot.packed.y);
                    if (dot(candidateNormal, queryNormal) < 0.0) {
                        candidateNormal = -candidateNormal;
                    }
                    const vec2 confidenceCoherence = unpackUnorm2x16(slot.packed.z);
                    const vec2 roughnessVariance = unpackHalf2x16(slot.packed.w);
                    const float normalDisagreement = 1.0 - abs(dot(candidateNormal, queryNormal));
                    const float planeResidual = abs(dot(delta, candidateNormal));
                    const vec3 continuityDelta = candidatePosition - predictedSurfacePosition;
                    const float sheetSeparation = abs(dot(continuityDelta, queryNormal));
                    const float normalAgreement = 1.0 - normalDisagreement;

                    // A close, nearly parallel sheet is precisely the case in
                    // which nearest-point queries tend to flicker. Once a
                    // route has accepted a sheet, reject a large normal-axis
                    // jump unless the normal turns enough to represent a real
                    // crease rather than a parallel layer.
                    const float maximumContinuousSeparation = max(
                        resolution * 1.35,
                        min(radius * 0.55, max(flow.route0.y * 2.0, resolution)));
                    if (hasPreviousSurface && normalAgreement > 0.72 &&
                        sheetSeparation > maximumContinuousSeparation) {
                        continue;
                    }

                    const float backwardDistance = hasPreviousSurface
                        ? max(0.0, -dot(continuityDelta, safeTangent))
                        : 0.0;
                    // Role changes are possible at a genuine ROCK/SAND
                    // boundary, but the old role wins close decisions. This
                    // hysteresis prevents alternating roles in overlapping
                    // hash cells without making boundaries impassable.
                    const float roleHysteresis =
                        hasPreviousSurface && previousRole != 0u && role != previousRole
                            ? resolution * (0.85 + confidenceCoherence.y * 0.45)
                            : 0.0;
                    const float continuityPenalty = hasPreviousSurface
                        ? length(continuityDelta) * 0.30 + sheetSeparation * 2.20 +
                              backwardDistance * 1.35
                        : 0.0;
                    const float score = distanceToCandidate * 0.75 + planeResidual * 0.85 +
                                        normalDisagreement * resolution * 2.5 +
                                        continuityPenalty + roleHysteresis -
                                        confidenceCoherence.x * resolution * 0.25;
                    if (score < bestScore) {
                        bestScore = score;
                        best.position = candidatePosition;
                        best.normal = candidateNormal;
                        best.confidence = confidenceCoherence.x;
                        best.coherence = confidenceCoherence.y;
                        best.roughness = clamp(
                            max(roughnessVariance.x, roughnessVariance.y),
                            0.0,
                            1.0);
                        best.planarity = clamp(
                            confidenceCoherence.y * (1.0 - clamp(roughnessVariance.y, 0.0, 1.0)),
                            0.0,
                            1.0);
                        best.coordinate = coordinate;
                        best.role = role;
                        best.hit = true;
                    }
                }
            }
        }
    }

    if (!best.hit) {
        return best;
    }

    vec3 surfaceTangent = safeTangent - best.normal * dot(safeTangent, best.normal);
    surfaceTangent = FlowSafeNormalize(
        surfaceTangent,
        FlowSafeNormalize(cross(routeLateral, best.normal), safeTangent));
    vec3 transverseAxis = FlowSafeNormalize(
        cross(best.normal, surfaceTangent),
        FlowSafeNormalize(routeLateral, vec3(0.0, 1.0, 0.0)));
    if (dot(transverseAxis, routeLateral) < 0.0) {
        transverseAxis = -transverseAxis;
    }

    float negativeSupport = 0.0;
    float positiveSupport = 0.0;
    float neighbourWeightSum = 0.0;
    float planarWeightSum = 0.0;
    float concavityWeightSum = 0.0;
    for (int z = -1; z <= 1; ++z) {
        for (int y = -1; y <= 1; ++y) {
            for (int x = -1; x <= 1; ++x) {
                const ivec3 offset = ivec3(x, y, z);
                if (all(equal(offset, ivec3(0)))) { continue; }
                WaterSurfaceSurfelSlot neighbour;
                if (!FlowFindSurfel(best.coordinate + offset, best.role, neighbour)) { continue; }

                vec3 neighbourNormal = FlowDecodeNormal(neighbour.packed.y);
                float agreement = dot(neighbourNormal, best.normal);
                if (agreement < 0.0) {
                    neighbourNormal = -neighbourNormal;
                    agreement = -agreement;
                }
                // Same-role is not enough at a crease: bilateral agreement and
                // plane residual keep the hint field on this surface sheet.
                if (agreement < 0.48) { continue; }
                const vec3 neighbourDelta = FlowDecodeCentroid(neighbour) - best.position;
                const float distanceCells = length(neighbourDelta) / resolution;
                const float planeResidualCells = abs(dot(neighbourDelta, best.normal)) / resolution;
                const vec2 neighbourConfidenceCoherence = unpackUnorm2x16(neighbour.packed.z);
                const float distanceWeight = exp(-0.5 * distanceCells * distanceCells);
                const float normalWeight = smoothstep(0.48, 0.96, agreement);
                const float planeWeight = exp(-1.5 * planeResidualCells * planeResidualCells);
                const float confidenceWeight = mix(0.25, 1.0, neighbourConfidenceCoherence.x);
                const float weight = distanceWeight * normalWeight * planeWeight * confidenceWeight;
                if (weight <= 1e-5) { continue; }

                const float transverseCells = dot(neighbourDelta, transverseAxis) / resolution;
                const float sideWeight = weight * smoothstep(0.15, 1.25, abs(transverseCells));
                if (transverseCells < 0.0) {
                    negativeSupport += sideWeight;
                } else {
                    positiveSupport += sideWeight;
                }

                const float signedHeightCells = dot(neighbourDelta, best.normal) / resolution;
                const vec3 towardCentre = transverseCells < 0.0
                    ? transverseAxis
                    : -transverseAxis;
                const float inwardNormal = max(0.0, dot(neighbourNormal, towardCentre));
                const float grooveEvidence = clamp(
                    max(0.0, signedHeightCells) * 0.70 + inwardNormal * 0.30,
                    0.0,
                    1.0);
                concavityWeightSum += grooveEvidence * sideWeight;
                planarWeightSum += agreement * planeWeight * weight;
                neighbourWeightSum += weight;
            }
        }
    }

    const float transverseTotal = negativeSupport + positiveSupport;
    const float transverseBalance = transverseTotal > 1e-5
        ? (2.0 * min(negativeSupport, positiveSupport)) / transverseTotal
        : 0.0;
    const float transverseCoverage = clamp(transverseTotal / 1.35, 0.0, 1.0);
    best.transverseSupport = clamp(transverseBalance * transverseCoverage, 0.0, 1.0);
    if (neighbourWeightSum > 1e-5) {
        const float neighbourPlanarity = clamp(
            planarWeightSum / neighbourWeightSum,
            0.0,
            1.0);
        best.planarity = clamp(mix(best.planarity, neighbourPlanarity, 0.55), 0.0, 1.0);
    }
    best.concavity = transverseTotal > 1e-5
        ? clamp(concavityWeightSum / transverseTotal, 0.0, 1.0)
        : 0.0;
    const vec3 gravity = vec3(0.0, 0.0, -1.0);
    const vec3 projectedGravity = gravity - best.normal * dot(gravity, best.normal);
    best.gravityStrength = clamp(length(projectedGravity), 0.0, 1.0);
    best.projectedGravity = best.gravityStrength > 1e-6
        ? projectedGravity / best.gravityStrength
        : vec3(0.0);
    return best;
}

vec3 FlowSafeSegmentMix(vec3 a, vec3 b, float ta, float tb, float t) {
    const float denominator = tb - ta;
    // The recursive Barry-Goldman construction deliberately extrapolates A1
    // and A3 while t lies on the P1-P2 knot interval. Clamping this weight
    // collapses every Catmull-Rom span onto the straight P1-P2 chord.
    return abs(denominator) > 1e-6 ? mix(a, b, (t - ta) / denominator) : a;
}

vec3 FlowCentripetalCatmullRom(vec3 p0, vec3 p1, vec3 p2, vec3 p3, float u) {
    float t0 = 0.0;
    float t1 = t0 + sqrt(max(length(p1 - p0), 1e-6));
    float t2 = t1 + sqrt(max(length(p2 - p1), 1e-6));
    float t3 = t2 + sqrt(max(length(p3 - p2), 1e-6));
    const float t = mix(t1, t2, clamp(u, 0.0, 1.0));
    const vec3 a1 = FlowSafeSegmentMix(p0, p1, t0, t1, t);
    const vec3 a2 = FlowSafeSegmentMix(p1, p2, t1, t2, t);
    const vec3 a3 = FlowSafeSegmentMix(p2, p3, t2, t3, t);
    const vec3 b1 = FlowSafeSegmentMix(a1, a2, t0, t2, t);
    const vec3 b2 = FlowSafeSegmentMix(a2, a3, t1, t3, t);
    return FlowSafeSegmentMix(b1, b2, t1, t2, t);
}

uint FlowInputSegment(float distanceMeters, WaterFlowBranch branch) {
    const uint start = branch.inputRoute.x;
    const uint count = branch.inputRoute.y;
    for (uint localIndex = 0u; localIndex + 1u < count; ++localIndex) {
        if (flowInput.points[start + localIndex + 1u].positionDistance.w >= distanceMeters) {
            return start + localIndex;
        }
    }
    return count > 1u ? start + count - 2u : start;
}

float FlowManualSegmentParameter(float segmentDistance, vec4 arcDistances) {
    const float target = clamp(segmentDistance, 0.0, max(arcDistances.w, 0.0));
    float lowerDistance = 0.0;
    for (uint checkpoint = 0u; checkpoint < 4u; ++checkpoint) {
        const float upperDistance = arcDistances[checkpoint];
        if (target <= upperDistance || checkpoint == 3u) {
            const float fraction = upperDistance > lowerDistance + 1e-6
                ? clamp((target - lowerDistance) / (upperDistance - lowerDistance), 0.0, 1.0)
                : 0.0;
            return (float(checkpoint) + fraction) * 0.25;
        }
        lowerDistance = upperDistance;
    }
    return 1.0;
}

void FlowEvaluateAuthored(
    float distanceMeters,
    WaterFlowBranch branch,
    out vec3 position,
    out vec3 normal,
    out float confidence,
    out float laneSpanMeters) {
    const uint start = branch.inputRoute.x;
    const uint count = branch.inputRoute.y;
    const float routeLength = max(branch.metrics.x, 0.0);
    const float clampedDistance = clamp(distanceMeters, 0.0, routeLength);
    const uint segment = FlowInputSegment(clampedDistance, branch);
    const uint localSegment = segment - start;
    const WaterFlowInputPoint a = flowInput.points[segment];
    const WaterFlowInputPoint b = flowInput.points[min(segment + 1u, start + count - 1u)];
    const float segmentDistance = max(0.0, clampedDistance - a.positionDistance.w);
    const float denominator = max(b.positionDistance.w - a.positionDistance.w, 1e-6);
    const float local = branch.identity.w == 1u
        ? FlowManualSegmentParameter(segmentDistance, a.outgoingArcDistances)
        : clamp(segmentDistance / denominator, 0.0, 1.0);
    {
        const vec3 p1 = a.positionDistance.xyz;
        const vec3 p2 = b.positionDistance.xyz;
        // Match the CPU builder's extrapolated endpoint phantoms. Duplicating
        // endpoints changes the tangent and visibly pinches the first/last arc.
        const vec3 p0 = localSegment > 0u
            ? flowInput.points[segment - 1u].positionDistance.xyz
            : p1 + (p1 - p2);
        const vec3 p3 = localSegment + 2u < count
            ? flowInput.points[segment + 2u].positionDistance.xyz
            : p2 + (p2 - p1);
        // Sampled emitter anchors spline exactly like manual control points:
        // lerping cached anchors put a visible corner at every anchor, so
        // trails ran down chains of straight segments on curvy paths.
        position = FlowCentripetalCatmullRom(p0, p1, p2, p3, local);
    }
    normal = FlowSafeNormalize(
        mix(a.normalConfidence.xyz, b.normalConfidence.xyz, local),
        vec3(0.0, 0.0, 1.0));
    confidence = clamp(mix(a.normalConfidence.w, b.normalConfidence.w, local), 0.0, 1.0);
    const float globalLaneSpan = max(0.0, flow.lane0.x);
    const float aLaneSpan = a.laneWidth.x > 1.5
        ? globalLaneSpan * max(0.0, a.laneWidth.y)
        : (a.laneWidth.x > 0.5
               ? max(0.0, a.laneWidth.y)
               : globalLaneSpan);
    const float bLaneSpan = b.laneWidth.x > 1.5
        ? globalLaneSpan * max(0.0, b.laneWidth.y)
        : (b.laneWidth.x > 0.5
               ? max(0.0, b.laneWidth.y)
               : globalLaneSpan);
    // Exact at every authored node, C1 across node boundaries, and bounded
    // between its two widths (unlike an unconstrained scalar Catmull curve).
    const float widthAmount = smoothstep(0.0, 1.0, local);
    laneSpanMeters = mix(aLaneSpan, bLaneSpan, widthAmount);
}

float FlowEndpointFade(float distanceMeters, float routeLengthMeters, float routeSpacingMeters) {
    const float routeLength = max(routeLengthMeters, 1e-5);
    const float fadeDistance = min(routeLength * 0.25, max(0.02, routeSpacingMeters * 2.0));
    return smoothstep(0.0, fadeDistance, min(distanceMeters, routeLength - distanceMeters));
}
