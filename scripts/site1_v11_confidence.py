#!/usr/bin/env python3
"""Pure geometry-confidence and density-sampling helpers for Scene1 v11.

This module deliberately contains no PLY I/O or pipeline orchestration.  It is
small enough to test with synthetic arrays and can therefore be shared by the
1 mm and 5 mm candidate builders without changing either cloud while a plan is
being assessed.

There are three independent pieces:

* a compact, continuous local-density interpolator with an optional bounded
  C1 outward taper;
* auditable hard confidence vetoes for proposed terrain geometry; and
* deterministic variable-radius blue-noise selection against measured and
  already-selected points.

Fine-scale residual energy (and analogous frequency/entropy descriptors) is
used only as a veto.  A high value is useful evidence of incoherent noise, but
a low value is not positive evidence of a real surface: it can equally mean a
genuinely flat surface, an over-smoothed reconstruction, or sparse sampling.
Consequently residual energy can reject a candidate but never raises its
confidence tier.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum, IntFlag
from typing import Sequence

import numpy as np


@dataclass(frozen=True)
class OutwardTaper:
    """A bounded, continuously differentiable density fade.

    ``distance`` is a caller-defined outward coordinate: radial distance,
    signed distance from a shoreline, or distance along a surveyed falloff
    direction all work.  The multiplier is one at and before ``start``,
    ``floor_ratio`` at and after ``end``, and a cubic smoothstep between them.
    """

    start: float
    end: float
    floor_ratio: float = 0.25

    def factors(self, distance: np.ndarray | Sequence[float]) -> np.ndarray:
        if not np.isfinite(self.start) or not np.isfinite(self.end):
            raise ValueError("taper bounds must be finite")
        if not self.end > self.start:
            raise ValueError("taper end must be greater than start")
        if not np.isfinite(self.floor_ratio) or not 0.0 <= self.floor_ratio <= 1.0:
            raise ValueError("taper floor_ratio must lie in [0, 1]")
        values = np.asarray(distance, dtype=np.float64)
        if not np.all(np.isfinite(values)):
            raise ValueError("outward distances must be finite")
        position = np.clip(
            (values - self.start) / (self.end - self.start), 0.0, 1.0
        )
        smooth = position * position * (3.0 - 2.0 * position)
        return np.clip(
            1.0 - (1.0 - self.floor_ratio) * smooth,
            self.floor_ratio,
            1.0,
        )


@dataclass(frozen=True)
class DensityTargetResult:
    """Local density estimates and the explicit taper applied to them."""

    target_density: np.ndarray
    local_density: np.ndarray
    taper_factor: np.ndarray
    nearest_support_distance: np.ndarray


def _as_point_matrix(values: np.ndarray | Sequence[Sequence[float]], name: str) -> np.ndarray:
    points = np.asarray(values, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] < 1:
        raise ValueError(f"{name} must have shape (N, D)")
    return points


def _ckdtree_type():
    """Return SciPy's cKDTree when available, otherwise a NumPy fallback marker."""

    try:
        from scipy.spatial import cKDTree
    except ModuleNotFoundError:
        return None
    return cKDTree


def _nearest_neighbours(
    query: np.ndarray,
    support: np.ndarray,
    count: int,
) -> tuple[np.ndarray, np.ndarray]:
    """Query nearest neighbours with a bounded-memory NumPy fallback.

    Scene builds normally provide SciPy.  The fallback keeps this pure helper
    testable and usable for small review crops without making SciPy an import-
    time dependency; it deliberately favours bounded memory over throughput.
    """

    tree_type = _ckdtree_type()
    if tree_type is not None:
        distances, indices = tree_type(support).query(query, k=count, workers=-1)
        if count == 1:
            distances = distances[:, None]
            indices = indices[:, None]
        return (
            np.asarray(distances, dtype=np.float64),
            np.asarray(indices, dtype=np.int64),
        )

    distances = np.empty((len(query), count), dtype=np.float64)
    indices = np.empty((len(query), count), dtype=np.int64)
    # Keep the temporary (query, support) distance matrix below roughly 64 MiB.
    chunk = max(1, int((64 * 1024 * 1024) // max(8 * len(support), 8)))
    for begin in range(0, len(query), chunk):
        end = min(begin + chunk, len(query))
        difference = query[begin:end, None, :] - support[None, :, :]
        squared = np.sum(np.square(difference), axis=2)
        # Stable sorting gives reproducible tie handling for symmetric support.
        selected = np.argsort(squared, axis=1, kind="stable")[:, :count]
        indices[begin:end] = selected
        distances[begin:end] = np.sqrt(
            np.take_along_axis(squared, selected, axis=1)
        )
    return distances, indices


def interpolate_target_density(
    query_points: np.ndarray | Sequence[Sequence[float]],
    support_points: np.ndarray | Sequence[Sequence[float]],
    support_density: np.ndarray | Sequence[float],
    *,
    neighbours: int = 12,
    minimum_bandwidth: float = 0.025,
    outward_distance: np.ndarray | Sequence[float] | None = None,
    taper: OutwardTaper | None = None,
    minimum_target: float = 0.0,
    maximum_target: float | None = None,
) -> DensityTargetResult:
    """Interpolate a smoothly varying local target density.

    An adaptive compact Wendland C2 kernel is used over the nearest support
    samples.  The furthest selected neighbour has zero weight at the support
    boundary, avoiding the abrupt value jumps of nearest-cell quota maps.
    Exact coincident support samples retain their mean observed value.

    The optional outward taper is applied *after* local interpolation.  It is
    bounded by :class:`OutwardTaper` and introduces no square planning-cell
    boundary.  Callers should omit it when the measured support densities
    already fully encode the desired acquisition falloff.
    """

    query = _as_point_matrix(query_points, "query_points")
    support = _as_point_matrix(support_points, "support_points")
    if query.shape[1] != support.shape[1]:
        raise ValueError("query_points and support_points dimensions differ")
    density = np.asarray(support_density, dtype=np.float64)
    if density.shape != (len(support),):
        raise ValueError("support_density must contain one value per support point")
    if len(support) == 0:
        raise ValueError("at least one support point is required")
    if not np.all(np.isfinite(query)):
        raise ValueError("query_points must be finite")
    valid_support = np.all(np.isfinite(support), axis=1) & np.isfinite(density)
    valid_support &= density >= 0.0
    support = support[valid_support]
    density = density[valid_support]
    if len(support) == 0:
        raise ValueError("no finite, non-negative density support remains")
    if not isinstance(neighbours, (int, np.integer)) or neighbours <= 0:
        raise ValueError("neighbours must be a positive integer")
    if not np.isfinite(minimum_bandwidth) or minimum_bandwidth <= 0.0:
        raise ValueError("minimum_bandwidth must be positive and finite")
    if not np.isfinite(minimum_target) or minimum_target < 0.0:
        raise ValueError("minimum_target must be finite and non-negative")
    if maximum_target is not None:
        if not np.isfinite(maximum_target) or maximum_target < minimum_target:
            raise ValueError("maximum_target must be finite and >= minimum_target")

    count = min(int(neighbours), len(support))
    distances, indices = _nearest_neighbours(query, support, count)

    local = np.empty(len(query), dtype=np.float64)
    exact_epsilon = max(np.finfo(np.float64).eps * 32.0, minimum_bandwidth * 1.0e-12)
    exact = distances <= exact_epsilon
    has_exact = np.any(exact, axis=1)
    if np.any(has_exact):
        exact_weights = exact[has_exact].astype(np.float64)
        local[has_exact] = np.sum(
            density[indices[has_exact]] * exact_weights, axis=1
        ) / np.sum(exact_weights, axis=1)

    remaining = ~has_exact
    if np.any(remaining):
        selected_distance = distances[remaining]
        # A tiny expansion keeps the kth neighbour just inside the compact
        # support while its weight tends to zero at a neighbour swap.
        bandwidth = np.maximum(
            minimum_bandwidth,
            selected_distance[:, -1] * (1.0 + 1.0e-9),
        )
        q = np.clip(selected_distance / bandwidth[:, None], 0.0, 1.0)
        weights = np.square(np.square(1.0 - q)) * (4.0 * q + 1.0)
        weight_sum = np.sum(weights, axis=1)
        no_weight = weight_sum <= np.finfo(np.float64).tiny
        if np.any(no_weight):
            weights[no_weight, 0] = 1.0
            weight_sum[no_weight] = 1.0
        local[remaining] = np.sum(
            density[indices[remaining]] * weights, axis=1
        ) / weight_sum

    local = np.maximum(local, minimum_target)
    if maximum_target is not None:
        local = np.minimum(local, maximum_target)

    taper_factor = np.ones(len(query), dtype=np.float64)
    if (outward_distance is None) != (taper is None):
        raise ValueError("outward_distance and taper must be provided together")
    if taper is not None:
        outward = np.asarray(outward_distance, dtype=np.float64)
        if outward.shape != (len(query),):
            raise ValueError("outward_distance must contain one value per query point")
        taper_factor = taper.factors(outward)

    target = local * taper_factor
    target = np.maximum(target, minimum_target)
    if maximum_target is not None:
        target = np.minimum(target, maximum_target)
    return DensityTargetResult(
        target_density=target,
        local_density=local,
        taper_factor=taper_factor,
        nearest_support_distance=distances[:, 0].copy(),
    )


class ConfidenceReason(IntFlag):
    """Hard-veto reasons recorded per proposed point or component."""

    NONE = 0
    INVALID_METRIC = 1 << 0
    INSUFFICIENT_DONOR_SECTORS = 1 << 1
    SURFACE_HEIGHT_DISAGREEMENT = 1 << 2
    LOW_NORMAL_COHERENCE = 1 << 3
    EXCESSIVE_VERTICAL_THICKNESS = 1 << 4
    VERTICAL_MULTIMODALITY = 1 << 5
    EXCESSIVE_RESIDUAL_ENERGY = 1 << 6


class ConfidenceTier(IntEnum):
    """Positive geometry tier after all hard vetoes pass."""

    REJECTED = 0
    CONSERVATIVE = 1
    SUPPORTED = 2
    STRONG = 3


@dataclass(frozen=True)
class ConfidenceThresholds:
    """Hard limits and stricter margins used to assign accepted tiers."""

    minimum_donor_sectors: int = 6
    strong_donor_sectors: int = 7
    maximum_surface_spread_m: float = 0.003
    strong_surface_spread_m: float = 0.002
    minimum_normal_coherence: float = 0.80
    strong_normal_coherence: float = 0.90
    maximum_vertical_thickness_m: float = 0.012
    strong_vertical_thickness_m: float = 0.006
    maximum_multimodality_score: float = 0.35
    strong_multimodality_score: float = 0.15
    maximum_residual_energy_ratio: float = 2.0
    supported_preferred_gates: int = 4

    def validate(self) -> None:
        if not 1 <= self.minimum_donor_sectors <= self.strong_donor_sectors <= 8:
            raise ValueError("donor-sector thresholds must satisfy 1 <= minimum <= strong <= 8")
        positive = (
            self.maximum_surface_spread_m,
            self.strong_surface_spread_m,
            self.minimum_normal_coherence,
            self.strong_normal_coherence,
            self.maximum_vertical_thickness_m,
            self.strong_vertical_thickness_m,
            self.maximum_residual_energy_ratio,
        )
        if not all(np.isfinite(value) and value >= 0.0 for value in positive):
            raise ValueError("confidence thresholds must be finite and non-negative")
        if self.strong_surface_spread_m > self.maximum_surface_spread_m:
            raise ValueError("strong surface spread must not exceed the hard maximum")
        if self.strong_normal_coherence < self.minimum_normal_coherence:
            raise ValueError("strong normal coherence must be at least the hard minimum")
        if not 0.0 <= self.minimum_normal_coherence <= 1.0:
            raise ValueError("minimum normal coherence must lie in [0, 1]")
        if not 0.0 <= self.strong_normal_coherence <= 1.0:
            raise ValueError("strong normal coherence must lie in [0, 1]")
        if self.strong_vertical_thickness_m > self.maximum_vertical_thickness_m:
            raise ValueError("strong vertical thickness must not exceed the hard maximum")
        if not 0.0 <= self.maximum_multimodality_score <= 1.0:
            raise ValueError("maximum multimodality score must lie in [0, 1]")
        if not 0.0 <= self.strong_multimodality_score <= self.maximum_multimodality_score:
            raise ValueError("strong multimodality score must lie below the hard maximum")
        if not 1 <= self.supported_preferred_gates <= 5:
            raise ValueError("supported_preferred_gates must lie in [1, 5]")


@dataclass(frozen=True)
class ConfidenceResult:
    """Vectorised confidence verdict and diagnostics."""

    reason_mask: np.ndarray
    tier: np.ndarray
    surface_spread_m: np.ndarray
    preferred_gate_count: np.ndarray

    @property
    def accepted(self) -> np.ndarray:
        return self.reason_mask == int(ConfidenceReason.NONE)


def evaluate_geometry_confidence(
    donor_sector_count: np.ndarray | Sequence[int],
    surface_heights_m: np.ndarray | Sequence[Sequence[float]],
    normal_coherence: np.ndarray | Sequence[float],
    vertical_thickness_m: np.ndarray | Sequence[float],
    multimodality_score: np.ndarray | Sequence[float],
    residual_energy_ratio: np.ndarray | Sequence[float],
    *,
    thresholds: ConfidenceThresholds = ConfidenceThresholds(),
) -> ConfidenceResult:
    """Apply independent hard vetoes and assign an accepted geometry tier.

    ``surface_heights_m`` contains two or more independent predictions, for
    example robust tangent-plane, screened-Poisson, and harmonic continuation
    heights.  Their per-row range is the agreement diagnostic.

    ``residual_energy_ratio`` compares fine-scale residual energy with nearby
    measured terrain.  It is intentionally absent from the preferred-gate
    count: frequency or entropy can identify too much incoherent energy, but a
    quiet signal cannot prove the proposed geometry is observed or correct.
    """

    thresholds.validate()
    sectors = np.asarray(donor_sector_count, dtype=np.float64)
    heights = np.asarray(surface_heights_m, dtype=np.float64)
    coherence = np.asarray(normal_coherence, dtype=np.float64)
    thickness = np.asarray(vertical_thickness_m, dtype=np.float64)
    multimodality = np.asarray(multimodality_score, dtype=np.float64)
    energy = np.asarray(residual_energy_ratio, dtype=np.float64)
    if sectors.ndim != 1:
        raise ValueError("donor_sector_count must be one-dimensional")
    count = len(sectors)
    if heights.ndim != 2 or heights.shape[0] != count or heights.shape[1] < 2:
        raise ValueError("surface_heights_m must have shape (N, M) with M >= 2")
    for name, values in (
        ("normal_coherence", coherence),
        ("vertical_thickness_m", thickness),
        ("multimodality_score", multimodality),
        ("residual_energy_ratio", energy),
    ):
        if values.shape != (count,):
            raise ValueError(f"{name} must contain one value per row")

    finite_heights = np.all(np.isfinite(heights), axis=1)
    surface_spread = np.full(count, np.nan, dtype=np.float64)
    if np.any(finite_heights):
        finite_rows = heights[finite_heights]
        surface_spread[finite_heights] = (
            np.max(finite_rows, axis=1) - np.min(finite_rows, axis=1)
        )

    valid = (
        np.isfinite(sectors)
        & finite_heights
        & np.isfinite(coherence)
        & np.isfinite(thickness)
        & np.isfinite(multimodality)
        & np.isfinite(energy)
        & (sectors >= 0.0)
        & (sectors <= 8.0)
        & (np.floor(sectors) == sectors)
        & (coherence >= 0.0)
        & (coherence <= 1.0)
        & (thickness >= 0.0)
        & (multimodality >= 0.0)
        & (multimodality <= 1.0)
        & (energy >= 0.0)
    )

    reasons = np.zeros(count, dtype=np.uint32)

    def veto(mask: np.ndarray, reason: ConfidenceReason) -> None:
        reasons[mask] |= np.uint32(int(reason))

    veto(~valid, ConfidenceReason.INVALID_METRIC)
    comparable = valid
    veto(
        comparable & (sectors < thresholds.minimum_donor_sectors),
        ConfidenceReason.INSUFFICIENT_DONOR_SECTORS,
    )
    veto(
        comparable & (surface_spread > thresholds.maximum_surface_spread_m),
        ConfidenceReason.SURFACE_HEIGHT_DISAGREEMENT,
    )
    veto(
        comparable & (coherence < thresholds.minimum_normal_coherence),
        ConfidenceReason.LOW_NORMAL_COHERENCE,
    )
    veto(
        comparable & (thickness > thresholds.maximum_vertical_thickness_m),
        ConfidenceReason.EXCESSIVE_VERTICAL_THICKNESS,
    )
    veto(
        comparable & (multimodality > thresholds.maximum_multimodality_score),
        ConfidenceReason.VERTICAL_MULTIMODALITY,
    )
    veto(
        comparable & (energy > thresholds.maximum_residual_energy_ratio),
        ConfidenceReason.EXCESSIVE_RESIDUAL_ENERGY,
    )

    preferred = np.column_stack(
        (
            sectors >= thresholds.strong_donor_sectors,
            surface_spread <= thresholds.strong_surface_spread_m,
            coherence >= thresholds.strong_normal_coherence,
            thickness <= thresholds.strong_vertical_thickness_m,
            multimodality <= thresholds.strong_multimodality_score,
        )
    )
    preferred_count = np.sum(preferred, axis=1).astype(np.uint8)
    tier = np.full(count, int(ConfidenceTier.REJECTED), dtype=np.uint8)
    accepted = reasons == 0
    tier[accepted] = int(ConfidenceTier.CONSERVATIVE)
    tier[accepted & (preferred_count >= thresholds.supported_preferred_gates)] = int(
        ConfidenceTier.SUPPORTED
    )
    tier[accepted & (preferred_count == preferred.shape[1])] = int(
        ConfidenceTier.STRONG
    )
    return ConfidenceResult(
        reason_mask=reasons,
        tier=tier,
        surface_spread_m=surface_spread,
        preferred_gate_count=preferred_count,
    )


def confidence_reason_names(mask: int | np.integer) -> tuple[str, ...]:
    """Decode a stored confidence bitmask into stable symbolic names."""

    value = int(mask)
    return tuple(
        reason.name
        for reason in ConfidenceReason
        if reason is not ConfidenceReason.NONE and value & int(reason)
    )


class BlueNoiseRejection(IntFlag):
    """Why a candidate was not included in a blue-noise sample."""

    NONE = 0
    INVALID_CANDIDATE = 1 << 0
    EXISTING_CLEARANCE = 1 << 1
    SELECTED_CLEARANCE = 1 << 2


@dataclass(frozen=True)
class BlueNoiseResult:
    """Selected original indices and a reason for every rejected candidate."""

    selected_indices: np.ndarray
    reason_mask: np.ndarray
    processing_order: np.ndarray

    @property
    def selected_mask(self) -> np.ndarray:
        mask = np.zeros(len(self.reason_mask), dtype=bool)
        mask[self.selected_indices] = True
        return mask


def _splitmix64(values: np.ndarray) -> np.ndarray:
    values = np.asarray(values, dtype=np.uint64)
    with np.errstate(over="ignore"):
        result = values + np.uint64(0x9E3779B97F4A7C15)
        result = (result ^ (result >> np.uint64(30))) * np.uint64(0xBF58476D1CE4E5B9)
        result = (result ^ (result >> np.uint64(27))) * np.uint64(0x94D049BB133111EB)
        return result ^ (result >> np.uint64(31))


def _point_hash(points: np.ndarray, seed: int) -> np.ndarray:
    canonical = np.asarray(points, dtype="<f8").copy()
    canonical[canonical == 0.0] = 0.0  # Canonicalise negative zero.
    bits = canonical.view(np.uint64).reshape(canonical.shape)
    hashes = np.full(len(points), np.uint64(seed & 0xFFFFFFFFFFFFFFFF), dtype=np.uint64)
    for axis in range(points.shape[1]):
        hashes = _splitmix64(
            hashes ^ _splitmix64(bits[:, axis] + np.uint64(axis + 1))
        )
    return hashes


def variable_radius_blue_noise(
    candidate_points: np.ndarray | Sequence[Sequence[float]],
    candidate_radius: np.ndarray | Sequence[float] | float,
    *,
    existing_points: np.ndarray | Sequence[Sequence[float]] | None = None,
    existing_radius: np.ndarray | Sequence[float] | float | None = None,
    priority: np.ndarray | Sequence[float] | None = None,
    seed: int = 0x5331563131434F44,
    rebuild_interval: int = 256,
    distance_tolerance: float = 1.0e-12,
) -> BlueNoiseResult:
    """Select a deterministic pointwise variable-radius blue-noise subset.

    A pair is compatible when its distance is at least the larger exclusion
    radius.  Measured/existing points are always considered before candidates.
    Candidate priority is descending; equal-priority candidates use a stable
    coordinate hash and never a planning-cell traversal.  Thus density can
    change continuously through the radius field without exposing square
    quota edges.

    The returned indices are in deterministic acceptance order and refer to
    the original candidate array.  Rejection bits make the selection suitable
    for reversible staging and diagnostics.
    """

    candidates = _as_point_matrix(candidate_points, "candidate_points")
    count, dimensions = candidates.shape
    radii = np.asarray(candidate_radius, dtype=np.float64)
    if radii.ndim == 0:
        radii = np.full(count, float(radii), dtype=np.float64)
    if radii.shape != (count,):
        raise ValueError("candidate_radius must be scalar or contain one value per candidate")
    if priority is None:
        priorities = np.zeros(count, dtype=np.float64)
    else:
        priorities = np.asarray(priority, dtype=np.float64)
        if priorities.shape != (count,):
            raise ValueError("priority must contain one value per candidate")
    if not isinstance(rebuild_interval, (int, np.integer)) or rebuild_interval <= 0:
        raise ValueError("rebuild_interval must be a positive integer")
    if not np.isfinite(distance_tolerance) or distance_tolerance < 0.0:
        raise ValueError("distance_tolerance must be finite and non-negative")

    reasons = np.zeros(count, dtype=np.uint32)
    valid = (
        np.all(np.isfinite(candidates), axis=1)
        & np.isfinite(radii)
        & (radii > 0.0)
        & np.isfinite(priorities)
    )
    reasons[~valid] |= np.uint32(int(BlueNoiseRejection.INVALID_CANDIDATE))

    if existing_points is None:
        existing = np.empty((0, dimensions), dtype=np.float64)
    else:
        existing = _as_point_matrix(existing_points, "existing_points")
        if existing.shape[1] != dimensions:
            raise ValueError("existing_points and candidate_points dimensions differ")
        if not np.all(np.isfinite(existing)):
            raise ValueError("existing_points must be finite")
    if existing_radius is None:
        existing_radii = np.zeros(len(existing), dtype=np.float64)
    else:
        existing_radii = np.asarray(existing_radius, dtype=np.float64)
        if existing_radii.ndim == 0:
            existing_radii = np.full(len(existing), float(existing_radii), dtype=np.float64)
        if existing_radii.shape != (len(existing),):
            raise ValueError("existing_radius must be scalar or contain one value per existing point")
        if not np.all(np.isfinite(existing_radii)) or np.any(existing_radii < 0.0):
            raise ValueError("existing_radius must be finite and non-negative")

    hashes = _point_hash(candidates, seed)
    original_index = np.arange(count, dtype=np.int64)
    order = np.lexsort((original_index, hashes, -priorities))
    order = order[valid[order]]

    tree_type = _ckdtree_type()
    existing_tree = tree_type(existing) if tree_type is not None and len(existing) else None
    maximum_existing_radius = float(np.max(existing_radii)) if len(existing) else 0.0

    selected_indices: list[int] = []
    selected_points: list[np.ndarray] = []
    selected_radii: list[float] = []
    selected_tree = None
    selected_tree_points = np.empty((0, dimensions), dtype=np.float64)
    selected_tree_radii = np.empty(0, dtype=np.float64)
    tree_count = 0
    maximum_selected_radius = 0.0

    def conflicts(
        point: np.ndarray,
        radius: float,
        points: np.ndarray,
        other_radii: np.ndarray,
    ) -> bool:
        if not len(points):
            return False
        squared = np.sum(np.square(points - point), axis=1)
        required = np.maximum(radius, other_radii)
        threshold = np.maximum(required - distance_tolerance, 0.0)
        return bool(np.any(squared < np.square(threshold)))

    for raw_index in order:
        index = int(raw_index)
        point = candidates[index]
        radius = float(radii[index])

        if existing_tree is not None:
            neighbours = existing_tree.query_ball_point(
                point, max(radius, maximum_existing_radius)
            )
            if neighbours and conflicts(
                point,
                radius,
                existing[np.asarray(neighbours, dtype=np.int64)],
                existing_radii[np.asarray(neighbours, dtype=np.int64)],
            ):
                reasons[index] |= np.uint32(int(BlueNoiseRejection.EXISTING_CLEARANCE))
                continue
        elif len(existing) and conflicts(point, radius, existing, existing_radii):
            reasons[index] |= np.uint32(int(BlueNoiseRejection.EXISTING_CLEARANCE))
            continue

        selected_conflict = False
        if selected_tree is not None and tree_count:
            neighbours = selected_tree.query_ball_point(
                point, max(radius, maximum_selected_radius)
            )
            if neighbours:
                neighbour_array = np.asarray(neighbours, dtype=np.int64)
                selected_conflict = conflicts(
                    point,
                    radius,
                    selected_tree_points[neighbour_array],
                    selected_tree_radii[neighbour_array],
                )
        if not selected_conflict and tree_count < len(selected_points):
            selected_conflict = conflicts(
                point,
                radius,
                np.asarray(selected_points[tree_count:]),
                np.asarray(selected_radii[tree_count:]),
            )
        if selected_conflict:
            reasons[index] |= np.uint32(int(BlueNoiseRejection.SELECTED_CLEARANCE))
            continue

        selected_indices.append(index)
        selected_points.append(point.copy())
        selected_radii.append(radius)
        maximum_selected_radius = max(maximum_selected_radius, radius)
        if tree_type is not None and len(selected_points) - tree_count >= rebuild_interval:
            selected_tree_points = np.asarray(selected_points).copy()
            selected_tree_radii = np.asarray(selected_radii).copy()
            selected_tree = tree_type(selected_tree_points)
            tree_count = len(selected_points)

    return BlueNoiseResult(
        selected_indices=np.asarray(selected_indices, dtype=np.int64),
        reason_mask=reasons,
        processing_order=np.asarray(order, dtype=np.int64),
    )


__all__ = [
    "BlueNoiseRejection",
    "BlueNoiseResult",
    "ConfidenceReason",
    "ConfidenceResult",
    "ConfidenceThresholds",
    "ConfidenceTier",
    "DensityTargetResult",
    "OutwardTaper",
    "confidence_reason_names",
    "evaluate_geometry_confidence",
    "interpolate_target_density",
    "variable_radius_blue_noise",
]
