#!/usr/bin/env python3
"""Deterministic fine-first WATER refinement primitives for Scene1 v12.

This module is deliberately independent of screenshots, production paths, PLY
publication, and the v11 pipeline.  It provides the geometry operations needed
by the focused v12 orchestrator while keeping their safety properties testable
on small synthetic clouds:

* stationary world-coordinate correlated fades (no crop-cell boundaries);
* robust upward local terrain fits which ignore stored normal orientation;
* separate terrain clearance and WATER--WATER exclusion distances;
* fail-closed, byte-reversible seeded component removal;
* moving circular density audits and deterministic local refills; and
* stable per-layer exact-row spatial downsampling.

Structured point records are never reconstructed.  Culling archives and
downsampled results are formed by indexing the original array, preserving every
field and its byte representation.  The helpers have no canonical install or
file-deletion capability.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntFlag
import itertools
import math
from typing import Iterable, Sequence

import numpy as np


DEFAULT_SEED = 0x5331563132574154
_UINT64_MASK = np.uint64(0xFFFFFFFFFFFFFFFF)
_HASH_SCALE_53 = 1.0 / float(1 << 53)


def _as_points(
    values: np.ndarray | Sequence[Sequence[float]],
    name: str,
    *,
    dimensions: int | None = None,
    allow_empty: bool = True,
) -> np.ndarray:
    points = np.asarray(values, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] < 1:
        raise ValueError(f"{name} must have shape (N, D)")
    if dimensions is not None:
        if points.shape[1] < dimensions:
            raise ValueError(f"{name} must have at least {dimensions} columns")
        points = points[:, :dimensions]
    if not allow_empty and not len(points):
        raise ValueError(f"{name} must not be empty")
    if not np.all(np.isfinite(points)):
        raise ValueError(f"{name} must be finite")
    return points


def _positive_finite(value: float, name: str) -> float:
    result = float(value)
    if not np.isfinite(result) or result <= 0.0:
        raise ValueError(f"{name} must be finite and positive")
    return result


def _splitmix64(values: np.ndarray | np.uint64) -> np.ndarray:
    """Vectorised SplitMix64 finaliser with intentional unsigned overflow."""

    value = np.asarray(values, dtype=np.uint64)
    with np.errstate(over="ignore"):
        value = value + np.uint64(0x9E3779B97F4A7C15)
        value = (value ^ (value >> np.uint64(30))) * np.uint64(
            0xBF58476D1CE4E5B9
        )
        value = (value ^ (value >> np.uint64(27))) * np.uint64(
            0x94D049BB133111EB
        )
    return value ^ (value >> np.uint64(31))


def _integer_pair_hash(
    x: np.ndarray,
    y: np.ndarray,
    *,
    seed: int,
) -> np.ndarray:
    x_u = np.asarray(x, dtype=np.int64).view(np.uint64)
    y_u = np.asarray(y, dtype=np.int64).view(np.uint64)
    with np.errstate(over="ignore"):
        combined = (
            x_u * np.uint64(0xD6E8FEB86659FD93)
            ^ y_u * np.uint64(0xA5A3564E27F886D9)
            ^ np.uint64(int(seed) & int(_UINT64_MASK))
        )
    return _splitmix64(combined)


def _hash_to_unit_interval(values: np.ndarray) -> np.ndarray:
    return ((np.asarray(values, np.uint64) >> np.uint64(11)).astype(np.float64)) * (
        _HASH_SCALE_53
    )


def _quintic_smoothstep(value: np.ndarray) -> np.ndarray:
    return value * value * value * (value * (value * 6.0 - 15.0) + 10.0)


def _rotated_value_noise(
    points_xy: np.ndarray,
    wavelength_m: float,
    *,
    seed: int,
    octave: int,
) -> np.ndarray:
    # Each octave gets a non-axis-aligned frame.  The field is evaluated in
    # absolute world coordinates, so crop boxes and processing chunks cannot
    # introduce phase resets or square boundary discontinuities.
    angle = 0.3819660112501051 + octave * 1.117902103
    cosine = math.cos(angle)
    sine = math.sin(angle)
    x = (cosine * points_xy[:, 0] - sine * points_xy[:, 1]) / wavelength_m
    y = (sine * points_xy[:, 0] + cosine * points_xy[:, 1]) / wavelength_m
    ix = np.floor(x).astype(np.int64)
    iy = np.floor(y).astype(np.int64)
    fx = _quintic_smoothstep(x - ix)
    fy = _quintic_smoothstep(y - iy)

    octave_seed = int(seed) ^ ((octave + 1) * 0x9E3779B9)

    def corner(dx: int, dy: int) -> np.ndarray:
        hashed = _integer_pair_hash(ix + dx, iy + dy, seed=octave_seed)
        return 2.0 * _hash_to_unit_interval(hashed) - 1.0

    n00 = corner(0, 0)
    n10 = corner(1, 0)
    n01 = corner(0, 1)
    n11 = corner(1, 1)
    lower = n00 + fx * (n10 - n00)
    upper = n01 + fx * (n11 - n01)
    return lower + fy * (upper - lower)


def multiscale_correlated_noise(
    points_xy: np.ndarray | Sequence[Sequence[float]],
    *,
    wavelengths_m: Sequence[float] = (0.08, 0.20, 0.50),
    weights: Sequence[float] = (0.50, 0.30, 0.20),
    seed: int = DEFAULT_SEED,
) -> np.ndarray:
    """Return stationary, smooth world-coordinate noise in ``[-1, 1]``.

    Values depend only on a point's coordinates and the supplied parameters.
    They are therefore exactly invariant to permutation, chunk boundaries, and
    the bounding box used to discover the points.  Rotating every octave avoids
    an axis-aligned lattice pattern while quintic interpolation keeps value and
    first derivative continuous at lattice boundaries.
    """

    points = _as_points(points_xy, "points_xy", dimensions=2)
    wavelengths = np.asarray(wavelengths_m, dtype=np.float64)
    octave_weights = np.asarray(weights, dtype=np.float64)
    if wavelengths.ndim != 1 or octave_weights.shape != wavelengths.shape:
        raise ValueError("wavelengths_m and weights must be equal-length vectors")
    if not len(wavelengths):
        raise ValueError("at least one wavelength is required")
    if not np.all(np.isfinite(wavelengths)) or np.any(wavelengths <= 0.0):
        raise ValueError("wavelengths_m must be finite and positive")
    if not np.all(np.isfinite(octave_weights)) or np.any(octave_weights < 0.0):
        raise ValueError("weights must be finite and non-negative")
    total_weight = float(np.sum(octave_weights))
    if total_weight <= 0.0:
        raise ValueError("at least one weight must be positive")

    result = np.zeros(len(points), dtype=np.float64)
    for octave, (wavelength, weight) in enumerate(
        zip(wavelengths, octave_weights, strict=True)
    ):
        if weight:
            result += weight * _rotated_value_noise(
                points,
                float(wavelength),
                seed=seed,
                octave=octave,
            )
    return np.clip(result / total_weight, -1.0, 1.0)


def _sigmoid(value: np.ndarray | float) -> np.ndarray:
    values = np.asarray(value, dtype=np.float64)
    result = np.empty_like(values)
    positive = values >= 0.0
    result[positive] = 1.0 / (1.0 + np.exp(-values[positive]))
    exponential = np.exp(values[~positive])
    result[~positive] = exponential / (1.0 + exponential)
    return result


def solve_logistic_bias(
    correlated_noise: np.ndarray | Sequence[float],
    target_mean: float,
    *,
    strength: float = 4.0,
    tolerance: float = 1.0e-13,
    iterations: int = 96,
) -> float:
    """Solve a logistic intercept whose mean probability is ``target_mean``.

    The returned scalar can be reused for every processing chunk.  Solving it
    once over a canonical calibration sample, rather than once per chunk, is
    what keeps later keep/reject decisions chunk invariant.
    """

    noise = np.asarray(correlated_noise, dtype=np.float64)
    if noise.ndim != 1 or not len(noise) or not np.all(np.isfinite(noise)):
        raise ValueError("correlated_noise must be a non-empty finite vector")
    target = float(target_mean)
    if not np.isfinite(target) or not 0.0 < target < 1.0:
        raise ValueError("target_mean must lie strictly between zero and one")
    amplitude = float(strength)
    if not np.isfinite(amplitude) or amplitude < 0.0:
        raise ValueError("strength must be finite and non-negative")
    if not isinstance(iterations, (int, np.integer)) or iterations <= 0:
        raise ValueError("iterations must be a positive integer")

    lower = -64.0
    upper = 64.0
    for _ in range(int(iterations)):
        midpoint = 0.5 * (lower + upper)
        mean = float(np.mean(_sigmoid(midpoint + amplitude * noise)))
        if mean < target:
            lower = midpoint
        else:
            upper = midpoint
        if abs(mean - target) <= tolerance:
            break
    return 0.5 * (lower + upper)


@dataclass(frozen=True)
class CorrelatedFadeResult:
    noise: np.ndarray
    probability: np.ndarray
    uniform_rank: np.ndarray
    keep_mask: np.ndarray
    bias: float
    target_probability_mean: float


def correlated_fade_selection(
    points_xy: np.ndarray | Sequence[Sequence[float]],
    *,
    bias: float | None = None,
    target_mean: float | None = None,
    strength: float = 4.0,
    wavelengths_m: Sequence[float] = (0.08, 0.20, 0.50),
    weights: Sequence[float] = (0.50, 0.30, 0.20),
    hash_resolution_m: float = 1.0e-6,
    seed: int = DEFAULT_SEED,
) -> CorrelatedFadeResult:
    """Make deterministic clustered retention decisions for world points.

    Supply exactly one of ``bias`` or ``target_mean``.  ``target_mean`` solves
    the bias on the points in this call.  For streamed production processing,
    first solve a bias on a fixed calibration sample and pass that same
    ``bias`` to every chunk.  The micro-rank is also coordinate-derived, so a
    point receives the same decision in any order or chunk.
    """

    points = _as_points(points_xy, "points_xy", dimensions=2)
    if (bias is None) == (target_mean is None):
        raise ValueError("provide exactly one of bias or target_mean")
    resolution = _positive_finite(hash_resolution_m, "hash_resolution_m")
    amplitude = float(strength)
    if not np.isfinite(amplitude) or amplitude < 0.0:
        raise ValueError("strength must be finite and non-negative")
    noise = multiscale_correlated_noise(
        points,
        wavelengths_m=wavelengths_m,
        weights=weights,
        seed=seed,
    )
    if bias is None:
        solved_bias = solve_logistic_bias(
            noise,
            float(target_mean),
            strength=amplitude,
        )
    else:
        solved_bias = float(bias)
        if not np.isfinite(solved_bias):
            raise ValueError("bias must be finite")
    probability = _sigmoid(solved_bias + amplitude * noise)
    quantised = np.rint(points / resolution).astype(np.int64)
    hashed = _integer_pair_hash(
        quantised[:, 0],
        quantised[:, 1],
        seed=int(seed) ^ 0xD1B54A32D192ED03,
    )
    uniform = _hash_to_unit_interval(hashed)
    keep = uniform < probability
    return CorrelatedFadeResult(
        noise=noise,
        probability=probability,
        uniform_rank=uniform,
        keep_mask=keep,
        bias=solved_bias,
        target_probability_mean=float(np.mean(probability)) if len(points) else math.nan,
    )


def _nearest_neighbours(
    query: np.ndarray,
    support: np.ndarray,
    count: int,
) -> tuple[np.ndarray, np.ndarray]:
    try:
        from scipy.spatial import cKDTree
    except ModuleNotFoundError:
        cKDTree = None
    if cKDTree is not None:
        distance, index = cKDTree(support).query(query, k=count, workers=-1)
        if count == 1:
            distance = np.asarray(distance)[:, None]
            index = np.asarray(index)[:, None]
        return np.asarray(distance, np.float64), np.asarray(index, np.int64)

    distance = np.empty((len(query), count), dtype=np.float64)
    index = np.empty((len(query), count), dtype=np.int64)
    chunk = max(1, int((64 * 1024 * 1024) // max(8 * len(support), 8)))
    for begin in range(0, len(query), chunk):
        end = min(begin + chunk, len(query))
        delta = query[begin:end, None, :] - support[None, :, :]
        squared = np.sum(np.square(delta), axis=2)
        selected = np.argsort(squared, axis=1, kind="stable")[:, :count]
        index[begin:end] = selected
        distance[begin:end] = np.sqrt(
            np.take_along_axis(squared, selected, axis=1)
        )
    return distance, index


class TerrainRejectReason(IntFlag):
    NONE = 0
    ABSOLUTE_CLEARANCE = 1 << 0
    INSUFFICIENT_SUPPORT = 1 << 1
    NON_PLANAR_SUPPORT = 1 << 2
    BELOW_TERRAIN = 1 << 3
    TOO_HIGH_ABOVE_TERRAIN = 1 << 4


@dataclass(frozen=True)
class TerrainGateResult:
    accepted_mask: np.ndarray
    accepted_indices: np.ndarray
    rejected_indices: np.ndarray
    reason_mask: np.ndarray
    nearest_distance_m: np.ndarray
    signed_height_m: np.ndarray
    fitted_normal: np.ndarray
    residual_rms_m: np.ndarray
    support_count: np.ndarray
    support_sector_count: np.ndarray


def _robust_upward_plane(
    candidate_xyz: np.ndarray,
    support_xyz: np.ndarray,
    *,
    iterations: int = 6,
) -> tuple[float, np.ndarray, float]:
    local_xy = support_xyz[:, :2] - candidate_xyz[None, :2]
    design = np.column_stack((local_xy, np.ones(len(local_xy), np.float64)))
    z = support_xyz[:, 2]
    coefficients, *_ = np.linalg.lstsq(design, z, rcond=None)
    weights = np.ones(len(support_xyz), dtype=np.float64)
    for _ in range(iterations):
        residual = z - design @ coefficients
        median = float(np.median(residual))
        mad = float(np.median(np.abs(residual - median)))
        scale = max(1.4826 * mad, 1.0e-7)
        normalised = np.abs(residual - median) / (1.345 * scale)
        weights = np.ones_like(normalised)
        outside = normalised > 1.0
        weights[outside] = 1.0 / normalised[outside]
        weighted_design = design * np.sqrt(weights)[:, None]
        weighted_z = z * np.sqrt(weights)
        updated, *_ = np.linalg.lstsq(weighted_design, weighted_z, rcond=None)
        if np.max(np.abs(updated - coefficients)) <= 1.0e-12:
            coefficients = updated
            break
        coefficients = updated
    residual = z - design @ coefficients
    weight_sum = max(float(np.sum(weights)), 1.0)
    rms = math.sqrt(float(np.sum(weights * np.square(residual))) / weight_sum)
    a, b, intercept = coefficients
    upward = np.array([-a, -b, 1.0], dtype=np.float64)
    upward /= np.linalg.norm(upward)
    signed = float(candidate_xyz[2] - intercept) * upward[2]
    return signed, upward, rms


def gate_candidates_against_terrain(
    candidate_xyz: np.ndarray | Sequence[Sequence[float]],
    terrain_xyz: np.ndarray | Sequence[Sequence[float]],
    *,
    terrain_normals: np.ndarray | Sequence[Sequence[float]] | None = None,
    absolute_clearance_m: float = 0.001,
    clearance_tolerance_m: float = 1.0e-9,
    neighbours: int = 20,
    support_radius_m: float = 0.008,
    minimum_support: int = 6,
    angular_sectors: int = 8,
    minimum_sectors: int = 3,
    maximum_residual_rms_m: float = 0.0015,
    below_tolerance_m: float = 0.0002,
    maximum_height_above_m: float | None = None,
) -> TerrainGateResult:
    """Gate candidates using distance plus a robust upward local surface.

    ``terrain_normals`` is accepted only so callers can pass complete terrain
    records without preprocessing.  Its shape is validated, but its values and
    signs are deliberately ignored.  The fitted plane is represented with a
    positive-Z normal, making inverted or zero stored normals harmless.

    An insufficient or incoherent neighbourhood fails closed.  A candidate is
    also rejected when any terrain point is closer than
    ``absolute_clearance_m`` in 3-D, independently of the later WATER spacing.
    """

    candidates = _as_points(candidate_xyz, "candidate_xyz", dimensions=3)
    terrain = _as_points(
        terrain_xyz, "terrain_xyz", dimensions=3, allow_empty=False
    )
    if terrain_normals is not None:
        normals = np.asarray(terrain_normals)
        if normals.shape != terrain.shape:
            raise ValueError("terrain_normals must match terrain_xyz shape")
    clearance = _positive_finite(absolute_clearance_m, "absolute_clearance_m")
    clearance_tolerance = float(clearance_tolerance_m)
    if not np.isfinite(clearance_tolerance) or clearance_tolerance < 0.0:
        raise ValueError("clearance_tolerance_m must be finite and non-negative")
    support_radius = _positive_finite(support_radius_m, "support_radius_m")
    residual_limit = _positive_finite(
        maximum_residual_rms_m, "maximum_residual_rms_m"
    )
    below_tolerance = float(below_tolerance_m)
    if not np.isfinite(below_tolerance) or below_tolerance < 0.0:
        raise ValueError("below_tolerance_m must be finite and non-negative")
    if not isinstance(neighbours, (int, np.integer)) or neighbours < 3:
        raise ValueError("neighbours must be an integer >= 3")
    if not isinstance(minimum_support, (int, np.integer)) or minimum_support < 3:
        raise ValueError("minimum_support must be an integer >= 3")
    if not isinstance(angular_sectors, (int, np.integer)) or angular_sectors < 3:
        raise ValueError("angular_sectors must be an integer >= 3")
    if not isinstance(minimum_sectors, (int, np.integer)) or not (
        1 <= minimum_sectors <= angular_sectors
    ):
        raise ValueError("minimum_sectors must lie in [1, angular_sectors]")
    if maximum_height_above_m is not None:
        maximum_height = _positive_finite(
            maximum_height_above_m, "maximum_height_above_m"
        )
    else:
        maximum_height = None

    count = min(int(neighbours), len(terrain))
    distance_3d, _ = _nearest_neighbours(candidates, terrain, 1)
    distance_xy, index_xy = _nearest_neighbours(
        candidates[:, :2], terrain[:, :2], count
    )
    nearest = distance_3d[:, 0]
    reasons = np.zeros(len(candidates), dtype=np.uint16)
    signed_height = np.full(len(candidates), np.nan, dtype=np.float64)
    fitted_normal = np.full((len(candidates), 3), np.nan, dtype=np.float64)
    residual_rms = np.full(len(candidates), np.nan, dtype=np.float64)
    support_count = np.zeros(len(candidates), dtype=np.int32)
    sector_count = np.zeros(len(candidates), dtype=np.int16)

    too_close = nearest < clearance - clearance_tolerance
    reasons[too_close] |= int(TerrainRejectReason.ABSOLUTE_CLEARANCE)
    for row, candidate in enumerate(candidates):
        within = distance_xy[row] <= support_radius
        support_index = index_xy[row, within]
        support = terrain[support_index]
        support_count[row] = len(support)
        if len(support):
            delta = support[:, :2] - candidate[None, :2]
            radial = np.linalg.norm(delta, axis=1)
            nonzero = radial > max(support_radius * 1.0e-9, 1.0e-12)
            if np.any(nonzero):
                angle = np.mod(np.arctan2(delta[nonzero, 1], delta[nonzero, 0]), 2.0 * math.pi)
                sector = np.floor(angle * angular_sectors / (2.0 * math.pi)).astype(int)
                sector_count[row] = len(np.unique(sector))
        if len(support) < minimum_support or sector_count[row] < minimum_sectors:
            reasons[row] |= int(TerrainRejectReason.INSUFFICIENT_SUPPORT)
            continue
        signed, upward, rms = _robust_upward_plane(candidate, support)
        signed_height[row] = signed
        fitted_normal[row] = upward
        residual_rms[row] = rms
        if rms > residual_limit:
            reasons[row] |= int(TerrainRejectReason.NON_PLANAR_SUPPORT)
        if signed < -below_tolerance:
            reasons[row] |= int(TerrainRejectReason.BELOW_TERRAIN)
        if maximum_height is not None and signed > maximum_height:
            reasons[row] |= int(TerrainRejectReason.TOO_HIGH_ABOVE_TERRAIN)

    accepted = reasons == 0
    return TerrainGateResult(
        accepted_mask=accepted,
        accepted_indices=np.flatnonzero(accepted),
        rejected_indices=np.flatnonzero(~accepted),
        reason_mask=reasons,
        nearest_distance_m=nearest,
        signed_height_m=signed_height,
        fitted_normal=fitted_normal,
        residual_rms_m=residual_rms,
        support_count=support_count,
        support_sector_count=sector_count,
    )


def _coordinate_hash(points: np.ndarray, *, seed: int) -> np.ndarray:
    normalised = np.asarray(points, np.float64).copy()
    normalised[normalised == 0.0] = 0.0  # canonicalise negative zero
    bits = np.ascontiguousarray(normalised).view(np.uint64).reshape(normalised.shape)
    result = np.full(len(points), np.uint64(int(seed) & int(_UINT64_MASK)))
    for dimension in range(points.shape[1]):
        with np.errstate(over="ignore"):
            result ^= _splitmix64(
                bits[:, dimension]
                + np.uint64((dimension + 1) * 0x9E3779B97F4A7C15 & int(_UINT64_MASK))
            )
    return _splitmix64(result)


def _stable_point_order(
    points: np.ndarray,
    *,
    seed: int,
    priority: np.ndarray | Sequence[float] | None = None,
    secondary_hash: np.ndarray | None = None,
) -> np.ndarray:
    coordinate_hash = _coordinate_hash(points, seed=seed)
    keys: list[np.ndarray] = [
        points[:, dimension]
        for dimension in reversed(range(points.shape[1]))
    ]
    if secondary_hash is not None:
        keys.append(np.asarray(secondary_hash, np.uint64))
    keys.append(coordinate_hash)
    if priority is not None:
        values = np.asarray(priority, np.float64)
        if values.shape != (len(points),) or not np.all(np.isfinite(values)):
            raise ValueError("priority must contain one finite value per point")
        keys.append(-values)
    return np.lexsort(tuple(keys))


def _cell_key(point: np.ndarray, spacing: float) -> tuple[int, ...]:
    return tuple(np.floor(point / spacing).astype(np.int64).tolist())


def _spacing_accept(
    point: np.ndarray,
    cells: dict[tuple[int, ...], list[np.ndarray]],
    spacing: float,
) -> bool:
    key = _cell_key(point, spacing)
    spacing_squared = spacing * spacing
    tolerance = max(spacing_squared * 1.0e-12, 1.0e-24)
    for offset in itertools.product((-1, 0, 1), repeat=len(point)):
        neighbour_key = tuple(key[d] + offset[d] for d in range(len(key)))
        for other in cells.get(neighbour_key, ()):
            if float(np.dot(point - other, point - other)) < spacing_squared - tolerance:
                return False
    return True


def _insert_cell(
    point: np.ndarray,
    cells: dict[tuple[int, ...], list[np.ndarray]],
    spacing: float,
) -> None:
    cells.setdefault(_cell_key(point, spacing), []).append(point)


@dataclass(frozen=True)
class WaterAdditionSelection:
    selected_indices: np.ndarray
    rejected_indices: np.ndarray
    terrain_gate: TerrainGateResult
    terrain_rejected_indices: np.ndarray
    water_spacing_rejected_indices: np.ndarray
    selection_order: np.ndarray
    terrain_clearance_m: float
    water_spacing_m: float


def select_water_additions(
    candidate_xyz: np.ndarray | Sequence[Sequence[float]],
    terrain_xyz: np.ndarray | Sequence[Sequence[float]],
    existing_water_xy: np.ndarray | Sequence[Sequence[float]],
    *,
    terrain_clearance_m: float = 0.001,
    water_spacing_m: float = 0.0019,
    priority: np.ndarray | Sequence[float] | None = None,
    seed: int = DEFAULT_SEED,
    **terrain_gate_kwargs: object,
) -> WaterAdditionSelection:
    """Gate terrain at one distance, then blue-noise WATER at another.

    The two distances never share a blocker list: a candidate 1.05 mm from a
    valid terrain surface may survive the terrain gate while still requiring
    1.8--2.0 mm clearance from existing and newly accepted WATER points.
    """

    candidates = _as_points(candidate_xyz, "candidate_xyz", dimensions=3)
    water = _as_points(existing_water_xy, "existing_water_xy", dimensions=2)
    clearance = _positive_finite(terrain_clearance_m, "terrain_clearance_m")
    spacing = _positive_finite(water_spacing_m, "water_spacing_m")
    gate = gate_candidates_against_terrain(
        candidates,
        terrain_xyz,
        absolute_clearance_m=clearance,
        **terrain_gate_kwargs,
    )
    accepted_by_terrain = gate.accepted_indices
    accepted_points = candidates[accepted_by_terrain, :2]
    local_priority = None
    if priority is not None:
        priority_values = np.asarray(priority, np.float64)
        if priority_values.shape != (len(candidates),) or not np.all(
            np.isfinite(priority_values)
        ):
            raise ValueError("priority must contain one finite value per candidate")
        local_priority = priority_values[accepted_by_terrain]
    order = _stable_point_order(
        accepted_points,
        seed=seed,
        priority=local_priority,
    )
    cells: dict[tuple[int, ...], list[np.ndarray]] = {}
    for blocker in water:
        _insert_cell(blocker, cells, spacing)
    selected_local: list[int] = []
    spacing_rejected_local: list[int] = []
    for local_index in order:
        point = accepted_points[local_index]
        if _spacing_accept(point, cells, spacing):
            selected_local.append(int(local_index))
            _insert_cell(point, cells, spacing)
        else:
            spacing_rejected_local.append(int(local_index))
    selected = accepted_by_terrain[np.asarray(selected_local, np.int64)]
    spacing_rejected = accepted_by_terrain[
        np.asarray(spacing_rejected_local, np.int64)
    ]
    selected_mask = np.zeros(len(candidates), dtype=bool)
    selected_mask[selected] = True
    return WaterAdditionSelection(
        selected_indices=np.sort(selected),
        rejected_indices=np.flatnonzero(~selected_mask),
        terrain_gate=gate,
        terrain_rejected_indices=gate.rejected_indices.copy(),
        water_spacing_rejected_indices=np.sort(spacing_rejected),
        selection_order=selected.copy(),
        terrain_clearance_m=clearance,
        water_spacing_m=spacing,
    )


def _radius_components(points: np.ndarray, radius: float) -> np.ndarray:
    count = len(points)
    parent = np.arange(count, dtype=np.int64)
    rank = np.zeros(count, dtype=np.int8)

    def find(index: int) -> int:
        root = index
        while parent[root] != root:
            root = int(parent[root])
        while parent[index] != index:
            following = int(parent[index])
            parent[index] = root
            index = following
        return root

    def union(left: int, right: int) -> None:
        left_root = find(left)
        right_root = find(right)
        if left_root == right_root:
            return
        if rank[left_root] < rank[right_root]:
            left_root, right_root = right_root, left_root
        parent[right_root] = left_root
        if rank[left_root] == rank[right_root]:
            rank[left_root] += 1

    try:
        from scipy.spatial import cKDTree
    except ModuleNotFoundError:
        cKDTree = None
    if cKDTree is not None:
        pairs = cKDTree(points).query_pairs(radius, output_type="ndarray")
        for left, right in pairs:
            union(int(left), int(right))
    else:
        radius_squared = radius * radius
        for left in range(count):
            delta = points[left + 1 :] - points[left]
            nearby = np.flatnonzero(
                np.sum(np.square(delta), axis=1) <= radius_squared
            )
            for offset in nearby:
                union(left, left + 1 + int(offset))
    roots = np.asarray([find(index) for index in range(count)], np.int64)
    unique = sorted(
        np.unique(roots),
        key=lambda root: tuple(np.min(points[roots == root], axis=0).tolist()),
    )
    root_to_label = {int(root): label for label, root in enumerate(unique)}
    return np.asarray([root_to_label[int(root)] for root in roots], np.int32)


@dataclass(frozen=True)
class SeededComponentCullPlan:
    cull_indices: np.ndarray
    keep_indices: np.ndarray
    archived_records: np.ndarray | None
    component_labels: np.ndarray
    selected_component_label: int | None
    original_count: int
    detached: bool
    reason: str


def plan_seeded_component_cull(
    points_xy: np.ndarray | Sequence[Sequence[float]],
    seed_xy: Sequence[float],
    *,
    connectivity_m: float,
    maximum_seed_distance_m: float,
    detachment_gap_m: float | None = None,
    protected_mask: np.ndarray | Sequence[bool] | None = None,
    maximum_component_fraction: float = 0.35,
    records: np.ndarray | None = None,
) -> SeededComponentCullPlan:
    """Plan removal of a small detached component nearest ``seed_xy``.

    The function refuses the largest component, any protected component, a
    seed without nearby support, and any component lacking the requested empty
    gap.  On refusal ``cull_indices`` and the archive are empty.  When records
    are supplied, archived rows are copied byte-for-byte for exact restoration.

    This helper is intended for a bounded review crop, not an uncropped
    multi-million-point connected-component pass.
    """

    points = _as_points(points_xy, "points_xy", dimensions=2, allow_empty=False)
    seed = np.asarray(seed_xy, dtype=np.float64)
    if seed.shape != (2,) or not np.all(np.isfinite(seed)):
        raise ValueError("seed_xy must contain two finite values")
    connectivity = _positive_finite(connectivity_m, "connectivity_m")
    maximum_seed_distance = _positive_finite(
        maximum_seed_distance_m, "maximum_seed_distance_m"
    )
    gap = connectivity if detachment_gap_m is None else _positive_finite(
        detachment_gap_m, "detachment_gap_m"
    )
    maximum_fraction = float(maximum_component_fraction)
    if not np.isfinite(maximum_fraction) or not 0.0 < maximum_fraction < 1.0:
        raise ValueError("maximum_component_fraction must lie in (0, 1)")
    if records is not None:
        record_array = np.asarray(records)
        if record_array.ndim != 1 or len(record_array) != len(points):
            raise ValueError("records must be a one-dimensional row per point")
        if record_array.dtype.hasobject:
            raise ValueError("records must have a fixed-width dtype")
    else:
        record_array = None
    if protected_mask is None:
        protected = np.zeros(len(points), dtype=bool)
    else:
        protected = np.asarray(protected_mask, dtype=bool)
        if protected.shape != (len(points),):
            raise ValueError("protected_mask must contain one value per point")

    labels = _radius_components(points, connectivity)
    empty = np.empty(0, dtype=np.int64)

    def refused(reason: str, label: int | None = None) -> SeededComponentCullPlan:
        return SeededComponentCullPlan(
            cull_indices=empty.copy(),
            keep_indices=np.arange(len(points), dtype=np.int64),
            archived_records=None,
            component_labels=labels,
            selected_component_label=label,
            original_count=len(points),
            detached=False,
            reason=reason,
        )

    distance = np.linalg.norm(points - seed[None, :], axis=1)
    nearest = int(np.argmin(distance))
    if distance[nearest] > maximum_seed_distance:
        return refused("no component lies within maximum_seed_distance_m")
    selected_label = int(labels[nearest])
    component = np.flatnonzero(labels == selected_label)
    counts = np.bincount(labels)
    largest = int(np.max(counts))
    if len(component) >= largest:
        return refused("seeded component is the largest component", selected_label)
    if len(component) / len(points) > maximum_fraction:
        return refused(
            "seeded component exceeds maximum_component_fraction", selected_label
        )
    if np.any(protected[component]):
        return refused("seeded component contains protected points", selected_label)
    keep = np.flatnonzero(labels != selected_label)
    if len(keep):
        nearest_to_rest, _ = _nearest_neighbours(points[component], points[keep], 1)
        if float(np.min(nearest_to_rest)) < gap:
            return refused("seeded component is not separated by detachment_gap_m", selected_label)
    archive = None if record_array is None else record_array[component].copy()
    return SeededComponentCullPlan(
        cull_indices=component,
        keep_indices=keep,
        archived_records=archive,
        component_labels=labels,
        selected_component_label=selected_label,
        original_count=len(points),
        detached=True,
        reason="detached seeded component accepted",
    )


def restore_seeded_component_cull(
    kept_records: np.ndarray,
    plan: SeededComponentCullPlan,
) -> np.ndarray:
    """Restore a successful cull archive to its exact original row order."""

    if not plan.detached or plan.archived_records is None:
        raise ValueError("plan does not contain an accepted record archive")
    kept = np.asarray(kept_records)
    archive = np.asarray(plan.archived_records)
    if kept.ndim != 1 or len(kept) != len(plan.keep_indices):
        raise ValueError("kept_records count does not match plan.keep_indices")
    if kept.dtype != archive.dtype:
        raise ValueError("kept_records dtype differs from archived_records")
    restored = np.empty(plan.original_count, dtype=kept.dtype)
    restored[plan.keep_indices] = kept
    restored[plan.cull_indices] = archive
    return restored


def _circle_counts(
    centres: np.ndarray,
    support: np.ndarray,
    radius: float,
) -> np.ndarray:
    if not len(support):
        return np.zeros(len(centres), dtype=np.int64)
    try:
        from scipy.spatial import cKDTree
    except ModuleNotFoundError:
        cKDTree = None
    if cKDTree is not None:
        return np.asarray(
            cKDTree(support).query_ball_point(centres, radius, return_length=True),
            dtype=np.int64,
        )
    radius_squared = radius * radius
    result = np.empty(len(centres), dtype=np.int64)
    for row, centre in enumerate(centres):
        result[row] = np.count_nonzero(
            np.sum(np.square(support - centre[None, :]), axis=1) <= radius_squared
        )
    return result


def _centres_covered_by_point(
    point: np.ndarray,
    centres: np.ndarray,
    radius: float,
) -> np.ndarray:
    return np.flatnonzero(
        np.sum(np.square(centres - point[None, :]), axis=1) <= radius * radius
    )


@dataclass(frozen=True)
class CircularDensityAudit:
    centres_xy: np.ndarray
    observed_count: np.ndarray
    target_count: np.ndarray
    density_ratio: np.ndarray
    deficit_count: np.ndarray
    dip_mask: np.ndarray
    radius_m: float
    minimum_ratio: float


def moving_circular_density_audit(
    centres_xy: np.ndarray | Sequence[Sequence[float]],
    support_xy: np.ndarray | Sequence[Sequence[float]],
    *,
    radius_m: float,
    target_density_per_m2: np.ndarray | Sequence[float] | float,
    minimum_ratio: float = 0.85,
) -> CircularDensityAudit:
    """Audit overlapping circular windows without raster or square cells."""

    centres = _as_points(centres_xy, "centres_xy", dimensions=2)
    support = _as_points(support_xy, "support_xy", dimensions=2)
    radius = _positive_finite(radius_m, "radius_m")
    target_density = np.asarray(target_density_per_m2, dtype=np.float64)
    if target_density.ndim == 0:
        target_density = np.full(len(centres), float(target_density), np.float64)
    if target_density.shape != (len(centres),):
        raise ValueError("target_density_per_m2 must be scalar or one per centre")
    if not np.all(np.isfinite(target_density)) or np.any(target_density < 0.0):
        raise ValueError("target_density_per_m2 must be finite and non-negative")
    ratio_limit = float(minimum_ratio)
    if not np.isfinite(ratio_limit) or not 0.0 < ratio_limit <= 1.0:
        raise ValueError("minimum_ratio must lie in (0, 1]")
    observed = _circle_counts(centres, support, radius)
    target = target_density * (math.pi * radius * radius)
    ratio = np.ones(len(centres), dtype=np.float64)
    positive = target > 0.0
    ratio[positive] = observed[positive] / target[positive]
    dip = positive & (ratio < ratio_limit)
    deficit = np.zeros(len(centres), dtype=np.int64)
    deficit[dip] = np.ceil(
        np.maximum(target[dip] - observed[dip], 0.0)
    ).astype(np.int64)
    return CircularDensityAudit(
        centres_xy=centres.copy(),
        observed_count=observed,
        target_count=target,
        density_ratio=ratio,
        deficit_count=deficit,
        dip_mask=dip,
        radius_m=radius,
        minimum_ratio=ratio_limit,
    )


@dataclass(frozen=True)
class CircularDensityRefillResult:
    selected_candidate_indices: np.ndarray
    rejected_candidate_indices: np.ndarray
    before: CircularDensityAudit
    after: CircularDensityAudit
    selection_order: np.ndarray
    required_minimum_count: np.ndarray
    remaining_deficit_count: np.ndarray
    candidate_count_per_centre: np.ndarray
    selected_count_per_centre: np.ndarray
    spacing_blocked_count_per_centre: np.ndarray
    upper_blocked_count_per_centre: np.ndarray
    active_centre_mask: np.ndarray


@dataclass(frozen=True)
class MeasuredContinuityTargets:
    """Per-window WATER requirements derived from measured areal support.

    Registered good-overlap windows measure WATER density per exact valid
    WATER-footprint area.
    Multiplying that density by independently measured *vacant* support area
    produces desired additions.  Immutable surface WATER and terrain are added
    only by the later attainable-count contract.
    """

    centres_xy: np.ndarray
    existing_water_count: np.ndarray
    terrain_count: np.ndarray
    shoreline_mask: np.ndarray
    raw_desired_addition_count: np.ndarray
    fillable_water_area_m2: np.ndarray
    reference_water_density_per_m2: np.ndarray
    reference_sample_count: np.ndarray
    reference_kind: tuple[str, ...]
    local_reference_water_count: np.ndarray
    local_reference_terrain_count: np.ndarray
    good_overlap_water_count: np.ndarray
    good_overlap_terrain_count: np.ndarray
    local_reference_water_area_m2: np.ndarray
    good_overlap_reference_water_area_m2: np.ndarray
    shoreline_terrain_count_threshold: float
    radius_m: float


@dataclass(frozen=True)
class AttainableAdditionDensityContract:
    """Integer WATER bounds with immutable source rows accounted once.

    ``raw_desired_addition_count`` is a measured demand over genuinely vacant
    support. It is not a total WATER count. The globally spaced reservoir is a
    necessary per-window lower-coverage check, not proof that one subset can
    satisfy every overlapping lower and upper interval. The final constrained
    solver selection supplies that certificate. Adding the immutable source
    count *after* ratio application prevents existing WATER from being
    discounted and then requested again.
    """

    immutable_water_count: np.ndarray
    raw_desired_addition_count: np.ndarray
    spacing_feasible_capacity_count: np.ndarray
    target_addition_count: np.ndarray
    addition_lower_count: np.ndarray
    addition_upper_count: np.ndarray
    target_water_count: np.ndarray
    water_lower_count: np.ndarray
    water_upper_count: np.ndarray
    capacity_sufficient_mask: np.ndarray
    active_centre_mask: np.ndarray
    minimum_ratio: float
    maximum_ratio: float


def attainable_addition_density_contract(
    immutable_water_count: np.ndarray | Sequence[int],
    raw_desired_addition_count: np.ndarray | Sequence[float],
    spacing_feasible_capacity_count: np.ndarray | Sequence[int],
    *,
    minimum_ratio: float = 0.85,
    maximum_ratio: float = 1.25,
    active_centre_mask: np.ndarray | Sequence[bool] | None = None,
) -> AttainableAdditionDensityContract:
    """Convert measured vacant-support demand into attainable count bounds.

    Ratios apply only to *new* additions.  Immutable WATER is added back once
    to the target and both bounds. A single globally spacing-valid reservoir
    provides a necessary lower-coverage condition; passing it is deliberately
    not called a joint interval-feasibility proof. Demand is never silently
    reduced to fit that greedy reservoir. Integer rounding can otherwise invert
    the interval for a sub-point target, so the upper count is raised to the
    lower count in that sole quantisation case.
    """

    immutable_raw = np.asarray(immutable_water_count, dtype=np.float64)
    desired = np.asarray(raw_desired_addition_count, dtype=np.float64)
    capacity_raw = np.asarray(
        spacing_feasible_capacity_count, dtype=np.float64
    )
    if immutable_raw.ndim != 1:
        raise ValueError("immutable_water_count must be one-dimensional")
    shape = immutable_raw.shape
    if desired.shape != shape or capacity_raw.shape != shape:
        raise ValueError("addition density arrays must have identical shapes")
    if active_centre_mask is None:
        active = np.ones(shape, dtype=bool)
    else:
        active = np.asarray(active_centre_mask, dtype=bool)
        if active.shape != shape:
            raise ValueError("active_centre_mask must match the count arrays")
    minimum = float(minimum_ratio)
    maximum = float(maximum_ratio)
    if not np.isfinite(minimum) or not 0.0 < minimum <= 1.0:
        raise ValueError("minimum_ratio must lie in (0, 1]")
    if not np.isfinite(maximum) or maximum < 1.0:
        raise ValueError("maximum_ratio must be finite and at least 1")
    if (
        not np.all(np.isfinite(immutable_raw))
        or np.any(immutable_raw < 0.0)
        or np.any(immutable_raw != np.floor(immutable_raw))
    ):
        raise ValueError("immutable_water_count must contain non-negative integers")
    if not np.all(np.isfinite(desired)) or np.any(desired < 0.0):
        raise ValueError(
            "raw_desired_addition_count must contain finite non-negative values"
        )
    if (
        not np.all(np.isfinite(capacity_raw))
        or np.any(capacity_raw < 0.0)
        or np.any(capacity_raw != np.floor(capacity_raw))
    ):
        raise ValueError(
            "spacing_feasible_capacity_count must contain non-negative integers"
        )

    immutable = immutable_raw.astype(np.int64)
    capacity = capacity_raw.astype(np.int64)
    target = desired.copy()
    target[~active] = 0.0
    lower = np.ceil(minimum * target).astype(np.int64)
    # Both bounds use ceiling on the discrete point-count lattice.  Flooring a
    # positive sub-point upper target can invert an otherwise valid one-point
    # interval (for example 0.85*0.49 and 1.25*0.49).
    upper = np.ceil(maximum * target).astype(np.int64)
    upper = np.maximum(upper, lower)
    lower[~active] = 0
    upper[~active] = 0
    if np.any(lower > upper):
        raise RuntimeError("addition density interval is inverted")

    return AttainableAdditionDensityContract(
        immutable_water_count=immutable,
        raw_desired_addition_count=desired.copy(),
        spacing_feasible_capacity_count=capacity,
        target_addition_count=target,
        addition_lower_count=lower,
        addition_upper_count=upper,
        target_water_count=immutable.astype(np.float64) + target,
        water_lower_count=immutable + lower,
        water_upper_count=immutable + upper,
        capacity_sufficient_mask=(~active) | (lower <= capacity),
        active_centre_mask=active.copy(),
        minimum_ratio=minimum,
        maximum_ratio=maximum,
    )


def measured_circular_continuity_targets(
    centres_xy: np.ndarray | Sequence[Sequence[float]],
    existing_water_xy: np.ndarray | Sequence[Sequence[float]],
    terrain_xy: np.ndarray | Sequence[Sequence[float]],
    local_reference_centres_xy: np.ndarray | Sequence[Sequence[float]],
    good_overlap_reference_centres_xy: np.ndarray | Sequence[Sequence[float]],
    *,
    radius_m: float,
    fillable_water_area_m2: np.ndarray | Sequence[float],
    reference_water_xy: (
        np.ndarray | Sequence[Sequence[float]] | None
    ) = None,
    local_reference_water_area_m2: (
        np.ndarray | Sequence[float] | None
    ) = None,
    good_overlap_reference_water_area_m2: (
        np.ndarray | Sequence[float] | None
    ) = None,
    reference_neighbours: int = 12,
    minimum_reference_windows: int = 6,
    shoreline_minimum_terrain_fraction: float = 0.10,
) -> MeasuredContinuityTargets:
    """Measure continuous vacant-support WATER demand without constants.

    The local reference centres are expected to form annuli around review
    dips.  The good-overlap centres sample the explicitly registered acceptable
    WATER/terrain interface.  Their WATER counts are normalised by exact valid
    footprint area, then scaled by each evaluated window's independently vacant
    WATER area.  References are selected by distance for every evaluated window
    so a broad site-wide median cannot flatten a real density trend.
    """

    centres = _as_points(centres_xy, "centres_xy", dimensions=2)
    water = _as_points(existing_water_xy, "existing_water_xy", dimensions=2)
    reference_water = (
        water
        if reference_water_xy is None
        else _as_points(reference_water_xy, "reference_water_xy", dimensions=2)
    )
    terrain = _as_points(terrain_xy, "terrain_xy", dimensions=2)
    local_centres = _as_points(
        local_reference_centres_xy,
        "local_reference_centres_xy",
        dimensions=2,
    )
    overlap_centres = _as_points(
        good_overlap_reference_centres_xy,
        "good_overlap_reference_centres_xy",
        dimensions=2,
    )
    radius = _positive_finite(radius_m, "radius_m")
    area = math.pi * radius * radius
    fillable_area = np.asarray(fillable_water_area_m2, dtype=np.float64)
    if fillable_area.shape != (len(centres),):
        raise ValueError("fillable_water_area_m2 must have one value per centre")
    if (
        not np.all(np.isfinite(fillable_area))
        or np.any(fillable_area < 0.0)
        or np.any(fillable_area > area)
    ):
        raise ValueError(
            "fillable_water_area_m2 must be finite and lie within the disk area"
        )
    neighbours = int(reference_neighbours)
    minimum = int(minimum_reference_windows)
    if neighbours < 1:
        raise ValueError("reference_neighbours must be positive")
    if minimum < 1 or minimum > neighbours:
        raise ValueError(
            "minimum_reference_windows must lie in [1, reference_neighbours]"
        )
    shoreline_fraction = float(shoreline_minimum_terrain_fraction)
    if not np.isfinite(shoreline_fraction) or not 0.0 < shoreline_fraction < 1.0:
        raise ValueError("shoreline_minimum_terrain_fraction must lie in (0, 1)")

    water_count = _circle_counts(centres, water, radius)
    terrain_count = _circle_counts(centres, terrain, radius)
    local_water = _circle_counts(local_centres, reference_water, radius)
    local_terrain = _circle_counts(local_centres, terrain, radius)
    overlap_water = _circle_counts(overlap_centres, reference_water, radius)
    overlap_terrain = _circle_counts(overlap_centres, terrain, radius)
    local_area = (
        np.full(len(local_centres), area, dtype=np.float64)
        if local_reference_water_area_m2 is None
        else np.asarray(local_reference_water_area_m2, dtype=np.float64)
    )
    overlap_area = (
        np.full(len(overlap_centres), area, dtype=np.float64)
        if good_overlap_reference_water_area_m2 is None
        else np.asarray(good_overlap_reference_water_area_m2, dtype=np.float64)
    )
    for label, values, expected in (
        ("local_reference_water_area_m2", local_area, len(local_centres)),
        (
            "good_overlap_reference_water_area_m2",
            overlap_area,
            len(overlap_centres),
        ),
    ):
        if values.shape != (expected,):
            raise ValueError(f"{label} must have one value per reference centre")
        if (
            not np.all(np.isfinite(values))
            or np.any(values < 0.0)
            or np.any(values > area)
        ):
            raise ValueError(f"{label} must lie within the reference disk area")

    desired_additions = np.zeros(len(centres), dtype=np.float64)
    reference_density = np.zeros(len(centres), dtype=np.float64)
    sample_count = np.zeros(len(centres), dtype=np.int32)
    reference_kind: list[str] = []
    overlap_combined = overlap_water + overlap_terrain
    valid_overlap = (
        (overlap_water > 0) & (overlap_terrain > 0) & (overlap_area > 0.0)
    )
    if np.count_nonzero(valid_overlap) >= minimum:
        shoreline_reference_count = float(np.median(overlap_combined[valid_overlap]))
    else:
        local_combined = local_water + local_terrain
        valid_local_shoreline = (local_water > 0) & (local_terrain > 0)
        if np.count_nonzero(valid_local_shoreline) < minimum:
            raise RuntimeError("insufficient measured shoreline density references")
        shoreline_reference_count = float(
            np.median(local_combined[valid_local_shoreline])
        )
    shoreline_terrain_threshold = shoreline_fraction * shoreline_reference_count
    shoreline = terrain_count >= shoreline_terrain_threshold
    local_water_density = np.divide(
        local_water,
        local_area,
        out=np.zeros(len(local_water), dtype=np.float64),
        where=local_area > 0.0,
    )
    overlap_water_density = np.divide(
        overlap_water,
        overlap_area,
        out=np.zeros(len(overlap_water), dtype=np.float64),
        where=overlap_area > 0.0,
    )

    def nearest_values(
        centre: np.ndarray,
        reference_centres: np.ndarray,
        values: np.ndarray,
        valid: np.ndarray,
        label: str,
    ) -> tuple[np.ndarray, str]:
        index = np.flatnonzero(valid)
        if len(index) < minimum:
            raise RuntimeError(
                f"only {len(index)} valid {label} density reference windows; "
                f"at least {minimum} are required"
            )
        distance_squared = np.sum(
            np.square(reference_centres[index] - centre[None, :]), axis=1
        )
        order = np.argsort(distance_squared, kind="stable")[:neighbours]
        selected = index[order]
        return values[selected].astype(np.float64), label

    local_combined = local_water + local_terrain
    for row, centre in enumerate(centres):
        try:
            values, label = nearest_values(
                centre,
                overlap_centres,
                overlap_water_density,
                valid_overlap,
                "good-overlap-water-area-normalized",
            )
        except RuntimeError:
            values, label = nearest_values(
                centre,
                local_centres,
                local_water_density,
                (local_water > 0) & (local_area > 0.0),
                "local-water-area-normalized",
            )
        reference_density[row] = float(np.median(values))
        desired_additions[row] = reference_density[row] * fillable_area[row]
        sample_count[row] = len(values)
        reference_kind.append(label)

    if np.any(reference_density <= 0.0):
        raise RuntimeError("measured areal reference density is empty")
    return MeasuredContinuityTargets(
        centres_xy=centres.copy(),
        existing_water_count=water_count,
        terrain_count=terrain_count,
        shoreline_mask=shoreline,
        raw_desired_addition_count=desired_additions,
        fillable_water_area_m2=fillable_area.copy(),
        reference_water_density_per_m2=reference_density,
        reference_sample_count=sample_count,
        reference_kind=tuple(reference_kind),
        local_reference_water_count=local_water,
        local_reference_terrain_count=local_terrain,
        good_overlap_water_count=overlap_water,
        good_overlap_terrain_count=overlap_terrain,
        local_reference_water_area_m2=local_area.copy(),
        good_overlap_reference_water_area_m2=overlap_area.copy(),
        shoreline_terrain_count_threshold=shoreline_terrain_threshold,
        radius_m=radius,
    )


def refill_circular_density_dips(
    centres_xy: np.ndarray | Sequence[Sequence[float]],
    existing_water_xy: np.ndarray | Sequence[Sequence[float]],
    candidate_xy: np.ndarray | Sequence[Sequence[float]],
    *,
    radius_m: float,
    target_density_per_m2: np.ndarray | Sequence[float] | float,
    water_spacing_m: float,
    minimum_ratio: float = 0.85,
    minimum_observed_count: np.ndarray | Sequence[int] | None = None,
    maximum_ratio: float | None = None,
    maximum_observed_count: np.ndarray | Sequence[int] | None = None,
    active_centre_mask: np.ndarray | Sequence[bool] | None = None,
    seed: int = DEFAULT_SEED,
) -> CircularDensityRefillResult:
    """Refill measured circular dips using deterministic blue-noise points.

    The lower acceptance bound, rather than the 100% reference target, is the
    stopping condition.  This matters for overlapping moving windows: spending
    points up to 100% in an early window can consume the spacing and upper-bound
    capacity needed to bring a neighbouring window to its required minimum.

    Deficient windows are processed from the scarcest candidate supply to the
    least scarce.  Within a window, candidates which also help other unresolved
    windows are preferred.  Both orderings use coordinate-derived hashes as
    deterministic tie breakers, so source row order cannot change the result.
    """

    centres = _as_points(centres_xy, "centres_xy", dimensions=2)
    existing = _as_points(existing_water_xy, "existing_water_xy", dimensions=2)
    candidates = _as_points(candidate_xy, "candidate_xy", dimensions=2)
    spacing = _positive_finite(water_spacing_m, "water_spacing_m")
    if active_centre_mask is None:
        active = np.ones(len(centres), dtype=bool)
    else:
        active = np.asarray(active_centre_mask, dtype=bool)
        if active.shape != (len(centres),):
            raise ValueError("active_centre_mask must have one value per centre")
    if maximum_ratio is None:
        ratio_ceiling = None
    else:
        ratio_ceiling = float(maximum_ratio)
        if not np.isfinite(ratio_ceiling) or ratio_ceiling < 1.0:
            raise ValueError("maximum_ratio must be finite and at least 1")
    if minimum_observed_count is None:
        explicit_minimum = None
    else:
        explicit_minimum = np.asarray(minimum_observed_count, dtype=np.float64)
        if explicit_minimum.shape != (len(centres),):
            raise ValueError("minimum_observed_count must have one value per centre")
        if (
            not np.all(np.isfinite(explicit_minimum))
            or np.any(explicit_minimum < 0.0)
            or np.any(explicit_minimum != np.floor(explicit_minimum))
        ):
            raise ValueError(
                "minimum_observed_count must contain non-negative integers"
            )
        explicit_minimum = explicit_minimum.astype(np.int64)
    if maximum_observed_count is None:
        explicit_ceiling = None
    else:
        explicit_ceiling = np.asarray(maximum_observed_count, dtype=np.float64)
        if explicit_ceiling.shape != (len(centres),):
            raise ValueError("maximum_observed_count must have one value per centre")
        if (
            not np.all(np.isfinite(explicit_ceiling))
            or np.any(explicit_ceiling < 0.0)
            or np.any(explicit_ceiling != np.floor(explicit_ceiling))
        ):
            raise ValueError("maximum_observed_count must contain non-negative integers")
        explicit_ceiling = explicit_ceiling.astype(np.int64)
    before = moving_circular_density_audit(
        centres,
        existing,
        radius_m=radius_m,
        target_density_per_m2=target_density_per_m2,
        minimum_ratio=minimum_ratio,
    )
    required_minimum = (
        np.ceil(float(minimum_ratio) * before.target_count).astype(np.int64)
        if explicit_minimum is None
        else explicit_minimum.copy()
    )
    required_minimum[~active] = 0
    remaining = np.maximum(
        required_minimum - before.observed_count,
        0,
    ).astype(np.int64)
    running_count = before.observed_count.astype(np.int64, copy=True)

    try:
        from scipy.spatial import cKDTree
    except ModuleNotFoundError:
        cKDTree = None
    if cKDTree is not None and len(candidates):
        centre_tree = cKDTree(centres)
        coverage = [
            np.asarray(value, dtype=np.int64)
            for value in centre_tree.query_ball_point(candidates, before.radius_m)
        ]
        candidate_tree = cKDTree(candidates)
        candidates_by_centre = [
            np.asarray(value, dtype=np.int64)
            for value in candidate_tree.query_ball_point(centres, before.radius_m)
        ]
    else:
        coverage = [
            _centres_covered_by_point(candidate, centres, before.radius_m)
            for candidate in candidates
        ]
        inverse: list[list[int]] = [[] for _ in range(len(centres))]
        for candidate_index, covered in enumerate(coverage):
            for centre_index in covered:
                inverse[int(centre_index)].append(candidate_index)
        candidates_by_centre = [
            np.asarray(value, dtype=np.int64) for value in inverse
        ]
    candidate_count = np.asarray(
        [len(value) for value in candidates_by_centre],
        dtype=np.int64,
    )

    # An explicit ceiling is a caller-computed count contract and is therefore
    # authoritative.  In particular, a shoreline caller may derive it from a
    # combined WATER+terrain target; applying ``maximum_ratio`` to the WATER
    # remainder a second time would incorrectly subtract a fraction of the
    # terrain contribution.  Ratio-derived ceilings retain the historical
    # grandfathering behaviour when no explicit ceiling was supplied.
    effective_ceiling: np.ndarray | None = None
    if explicit_ceiling is not None:
        if np.any(before.observed_count[active] > explicit_ceiling[active]):
            raise RuntimeError(
                "existing active-window WATER count exceeds the explicit maximum"
            )
        effective_ceiling = explicit_ceiling.copy()
    elif ratio_ceiling is not None:
        effective_ceiling = np.ceil(
            ratio_ceiling * before.target_count
        ).astype(np.int64)
        effective_ceiling = np.maximum(
            effective_ceiling,
            before.observed_count,
        )
    if (
        explicit_minimum is not None
        and effective_ceiling is not None
        and np.any(required_minimum[active] > effective_ceiling[active])
    ):
        raise RuntimeError(
            "explicit/derived minimum exceeds the active-window maximum"
        )

    cells: dict[tuple[int, ...], list[np.ndarray]] = {}
    for point in existing:
        _insert_cell(point, cells, spacing)
    selected: list[int] = []
    # 0=available/unseen, 1=selected, 2=permanently spacing-blocked,
    # 3=permanently upper-bound-blocked.  Both blockers remain monotonic as
    # points are added, so rejecting them once is safe.
    state = np.zeros(len(candidates), dtype=np.uint8)
    unprocessed = active.copy()
    while np.any(unprocessed & (remaining > 0)):
        unresolved = np.flatnonzero(unprocessed & (remaining > 0))
        supply_per_required = candidate_count[unresolved] / np.maximum(
            remaining[unresolved], 1
        )
        # lexsort's last key is primary: lowest supply/need first, then the
        # largest absolute deficit, then stable centre index.
        local_order = np.lexsort((
            unresolved,
            -remaining[unresolved],
            supply_per_required,
        ))
        centre_index = int(unresolved[int(local_order[0])])
        centre_candidates = candidates_by_centre[centre_index]
        available = centre_candidates[state[centre_candidates] == 0]
        if len(available):
            gain = np.zeros(len(available), dtype=np.float64)
            closes_satisfied_slack = np.zeros(len(available), dtype=np.int64)
            reciprocal_headroom_pressure = np.zeros(
                len(available), dtype=np.float64
            )
            for row, candidate_index in enumerate(available):
                covered = coverage[int(candidate_index)]
                needed = covered[remaining[covered] > 0]
                if len(needed):
                    scarcity = remaining[needed] / np.maximum(
                        candidate_count[needed], 1
                    )
                    gain[row] = float(len(needed) + np.sum(scarcity))
                if effective_ceiling is not None:
                    active_covered = covered[active[covered]]
                    satisfied = active_covered[remaining[active_covered] == 0]
                    if len(satisfied):
                        slack = (
                            effective_ceiling[satisfied]
                            - running_count[satisfied]
                        )
                        closes_satisfied_slack[row] = int(
                            np.count_nonzero(slack == 1)
                        )
                        reciprocal_headroom_pressure[row] = float(
                            np.sum(1.0 / np.maximum(slack, 1))
                        )
            points = candidates[available]
            coordinate_hash = _coordinate_hash(points, seed=seed)
            # Primary: help the most unresolved/scarce windows.  For equal
            # gain, avoid consuming the last unit of upper-bound headroom in
            # already-satisfied overlapping windows.  Coordinate-derived
            # hashes and coordinates make the result source-order invariant.
            order = np.lexsort(tuple(
                [
                    points[:, dimension]
                    for dimension in reversed(range(points.shape[1]))
                ]
                + [
                    coordinate_hash,
                    reciprocal_headroom_pressure,
                    closes_satisfied_slack,
                    -gain,
                ]
            ))
            for local_index in order:
                if remaining[centre_index] <= 0:
                    break
                index = int(available[int(local_index)])
                if state[index] != 0:
                    continue
                covered = coverage[index]
                active_covered = covered[active[covered]]
                if not np.any(remaining[active_covered] > 0):
                    continue
                if (
                    effective_ceiling is not None
                    and len(active_covered)
                    and np.any(
                        running_count[active_covered] + 1
                        > effective_ceiling[active_covered]
                    )
                ):
                    state[index] = 3
                    continue
                point = candidates[index]
                if not _spacing_accept(point, cells, spacing):
                    state[index] = 2
                    continue
                state[index] = 1
                selected.append(index)
                _insert_cell(point, cells, spacing)
                running_count[covered] += 1
                remaining[covered] = np.maximum(
                    required_minimum[covered] - running_count[covered],
                    0,
                )
        unprocessed[centre_index] = False

    selected_array = np.asarray(selected, dtype=np.int64)
    selected_mask = np.zeros(len(candidates), dtype=bool)
    selected_mask[selected_array] = True
    combined = (
        np.concatenate((existing, candidates[selected_array]), axis=0)
        if len(selected_array)
        else existing
    )
    after = moving_circular_density_audit(
        centres,
        combined,
        radius_m=radius_m,
        target_density_per_m2=target_density_per_m2,
        minimum_ratio=minimum_ratio,
    )
    spacing_blocked = np.flatnonzero(state == 2)
    upper_blocked = np.flatnonzero(state == 3)
    selected_count = _circle_counts(
        centres,
        candidates[selected_array],
        before.radius_m,
    )
    spacing_blocked_count = _circle_counts(
        centres,
        candidates[spacing_blocked],
        before.radius_m,
    )
    upper_blocked_count = _circle_counts(
        centres,
        candidates[upper_blocked],
        before.radius_m,
    )
    return CircularDensityRefillResult(
        selected_candidate_indices=np.sort(selected_array),
        rejected_candidate_indices=np.flatnonzero(~selected_mask),
        before=before,
        after=after,
        selection_order=selected_array,
        required_minimum_count=required_minimum,
        remaining_deficit_count=np.maximum(
            required_minimum - after.observed_count,
            0,
        ),
        candidate_count_per_centre=candidate_count,
        selected_count_per_centre=selected_count,
        spacing_blocked_count_per_centre=spacing_blocked_count,
        upper_blocked_count_per_centre=upper_blocked_count,
        active_centre_mask=active.copy(),
    )


def _row_hash(records: np.ndarray, *, seed: int) -> np.ndarray:
    contiguous = np.ascontiguousarray(records)
    raw = contiguous.view(np.uint8).reshape(len(contiguous), contiguous.dtype.itemsize)
    result = np.full(
        len(contiguous),
        np.uint64(0xCBF29CE484222325 ^ (int(seed) & int(_UINT64_MASK))),
    )
    with np.errstate(over="ignore"):
        for column in range(raw.shape[1]):
            result ^= raw[:, column].astype(np.uint64)
            result *= np.uint64(0x100000001B3)
    return _splitmix64(result)


def _resolve_rare_order_ties(
    order: np.ndarray,
    points: np.ndarray,
    coordinate_hash: np.ndarray,
    row_hash: np.ndarray,
    records: np.ndarray,
) -> np.ndarray:
    """Resolve 64-bit collisions by full row bytes without penalising normal rows."""

    result = order.copy()
    begin = 0
    while begin < len(result):
        first = result[begin]
        end = begin + 1
        while end < len(result):
            other = result[end]
            if coordinate_hash[other] != coordinate_hash[first]:
                break
            if row_hash[other] != row_hash[first]:
                break
            if not np.array_equal(points[other], points[first]):
                break
            end += 1
        if end - begin > 1:
            group = result[begin:end]
            result[begin:end] = np.asarray(
                sorted(group.tolist(), key=lambda index: records[index].tobytes()),
                dtype=np.int64,
            )
        begin = end
    return result


@dataclass(frozen=True)
class ExactRowDownsampleResult:
    selected_indices: np.ndarray
    rejected_indices: np.ndarray
    selected_records: np.ndarray
    selection_order: np.ndarray
    spacing_m: float
    coordinate_fields: tuple[str, ...]


def exact_row_spatial_downsample(
    records: np.ndarray,
    *,
    spacing_m: float,
    coordinate_fields: Sequence[str] = ("x", "y", "z"),
    seed: int = DEFAULT_SEED,
) -> ExactRowDownsampleResult:
    """Select a deterministic, exact-row, per-layer spatial subset.

    Selection is a stable world-coordinate priority Poisson pass.  It operates
    on exactly one structured layer and therefore never lets another role win
    a shared voxel.  The selected output is in canonical selection order, not
    source order, so permuting input rows or concatenating source chunks in a
    different order produces the same ordered record bytes.  No selected row is
    reconstructed or scalar-interpolated.
    """

    record_array = np.asarray(records)
    if record_array.ndim != 1 or record_array.dtype.names is None:
        raise ValueError("records must be a one-dimensional structured array")
    if record_array.dtype.hasobject:
        raise ValueError("records must use a fixed-width dtype")
    fields = tuple(str(name) for name in coordinate_fields)
    if not fields:
        raise ValueError("at least one coordinate field is required")
    missing = [name for name in fields if name not in record_array.dtype.names]
    if missing:
        raise ValueError(f"missing coordinate fields: {missing}")
    spacing = _positive_finite(spacing_m, "spacing_m")
    points = np.column_stack(
        [np.asarray(record_array[name], dtype=np.float64) for name in fields]
    )
    if not np.all(np.isfinite(points)):
        raise ValueError("coordinate fields must be finite")
    coordinate_hash = _coordinate_hash(points, seed=seed)
    row_hash = _row_hash(record_array, seed=seed ^ 0xA24BAED4963EE407)
    keys: list[np.ndarray] = [
        points[:, dimension] for dimension in reversed(range(points.shape[1]))
    ]
    keys.extend((row_hash, coordinate_hash))
    order = np.lexsort(tuple(keys))
    order = _resolve_rare_order_ties(
        order,
        points,
        coordinate_hash,
        row_hash,
        record_array,
    )
    cells: dict[tuple[int, ...], list[np.ndarray]] = {}
    selected: list[int] = []
    for index in order:
        point = points[int(index)]
        if _spacing_accept(point, cells, spacing):
            selected.append(int(index))
            _insert_cell(point, cells, spacing)
    selected_indices = np.asarray(selected, dtype=np.int64)
    selected_mask = np.zeros(len(record_array), dtype=bool)
    selected_mask[selected_indices] = True
    # Advanced indexing copies full fixed-width rows byte-for-byte, including
    # all unknown scalar fields and their NaN payloads.
    selected_records = record_array[selected_indices].copy()
    return ExactRowDownsampleResult(
        selected_indices=selected_indices,
        rejected_indices=np.flatnonzero(~selected_mask),
        selected_records=selected_records,
        selection_order=selected_indices.copy(),
        spacing_m=spacing,
        coordinate_fields=fields,
    )


def exact_row_spatial_downsample_chunks(
    record_chunks: Iterable[np.ndarray],
    *,
    spacing_m: float,
    coordinate_fields: Sequence[str] = ("x", "y", "z"),
    seed: int = DEFAULT_SEED,
) -> ExactRowDownsampleResult:
    """Reference chunk adapter with chunk-order-invariant output semantics.

    The adapter intentionally materialises its input.  Production PLY code may
    provide a native external sort, but must preserve this helper's stable key
    and exact-row contract.
    """

    chunks = [np.asarray(chunk) for chunk in record_chunks if len(chunk)]
    if not chunks:
        raise ValueError("record_chunks must contain at least one row")
    dtype = chunks[0].dtype
    if any(chunk.dtype != dtype for chunk in chunks):
        raise ValueError("all record chunks must share one dtype")
    combined = np.concatenate(chunks, axis=0)
    return exact_row_spatial_downsample(
        combined,
        spacing_m=spacing_m,
        coordinate_fields=coordinate_fields,
        seed=seed,
    )
