#!/usr/bin/env python3
"""Candidate-only WATER density refinement helpers for Scene1 v11.

The functions in this module deliberately separate *where to review* from
*what to change*.  Screenshot polygons and boxes are masks for candidate
planning only.  Local measured SAND/ROCK/WATER spacing determines the target
density, and every keep/reject decision is pointwise; there are no planning
cell quotas that can expose square edges.

The PLY helpers are fixed-stride, binary-little-endian readers and streaming
candidate writers.  Selected records are copied without rewriting ``x/y/z``
or scalar fields, so accepted v10 WATER and oversample-reservoir records keep
their exact values.  Every canonical ``Site1-{role}-{1,2,5}mm.ply`` output name
and all source paths are refused.  This module has no install operation.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import os
from pathlib import Path
import re
import sys
from typing import Callable, Iterable, Iterator, Mapping, Sequence

import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from site1_v11_confidence import (  # noqa: E402
    BlueNoiseResult,
    OutwardTaper,
    variable_radius_blue_noise,
)


DEFAULT_SEED = 0x5331563131574154
CANONICAL_WATER_NAME = re.compile(r"^Site1-WATER-(?:1mm|2mm|5mm)\.ply$")
CANONICAL_SITE1_CLOUD_NAME = re.compile(
    r"^Site1-(?:SAND|ROCK|VEG|WATER)-(?:1mm|2mm|5mm)\.ply$"
)


def _as_xy(
    values: np.ndarray | Sequence[Sequence[float]],
    name: str,
    *,
    allow_empty: bool = True,
) -> np.ndarray:
    points = np.asarray(values, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] < 2:
        raise ValueError(f"{name} must have shape (N, D), D >= 2")
    points = points[:, :2]
    if not allow_empty and not len(points):
        raise ValueError(f"{name} must not be empty")
    if not np.all(np.isfinite(points)):
        raise ValueError(f"{name} must be finite")
    return points


def _as_bbox(values: Sequence[float]) -> tuple[float, float, float, float]:
    bbox = np.asarray(values, dtype=np.float64)
    if bbox.shape != (4,) or not np.all(np.isfinite(bbox)):
        raise ValueError("bbox must contain four finite values")
    xmin, xmax, ymin, ymax = (float(value) for value in bbox)
    if not xmin < xmax or not ymin < ymax:
        raise ValueError("bbox must satisfy xmin < xmax and ymin < ymax")
    return xmin, xmax, ymin, ymax


def expand_bbox(bbox: Sequence[float], margin_m: float) -> tuple[float, ...]:
    """Expand a world-space bbox without changing its coordinate convention."""

    xmin, xmax, ymin, ymax = _as_bbox(bbox)
    margin = float(margin_m)
    if not np.isfinite(margin) or margin < 0.0:
        raise ValueError("margin_m must be finite and non-negative")
    return xmin - margin, xmax + margin, ymin - margin, ymax + margin


def bbox_mask(
    points_xy: np.ndarray | Sequence[Sequence[float]],
    bbox: Sequence[float],
    *,
    inclusive: bool = True,
) -> np.ndarray:
    """Return a pointwise bbox mask; no cells or rasterisation are involved."""

    points = _as_xy(points_xy, "points_xy")
    xmin, xmax, ymin, ymax = _as_bbox(bbox)
    if inclusive:
        return (
            (points[:, 0] >= xmin)
            & (points[:, 0] <= xmax)
            & (points[:, 1] >= ymin)
            & (points[:, 1] <= ymax)
        )
    return (
        (points[:, 0] > xmin)
        & (points[:, 0] < xmax)
        & (points[:, 1] > ymin)
        & (points[:, 1] < ymax)
    )


def polygon_mask(
    points_xy: np.ndarray | Sequence[Sequence[float]],
    polygon_xy: np.ndarray | Sequence[Sequence[float]],
    *,
    include_boundary: bool = True,
    tolerance_m: float = 1.0e-10,
) -> np.ndarray:
    """Vectorised even/odd polygon mask with an explicit boundary policy.

    ``polygon_xy`` should describe a simple polygon.  The freehand cyan v11
    stroke is intentionally stored as evidence and should first be resolved
    into measured connected components rather than passed here verbatim.
    """

    points = _as_xy(points_xy, "points_xy")
    polygon = _as_xy(polygon_xy, "polygon_xy", allow_empty=False)
    if len(polygon) < 3:
        raise ValueError("polygon_xy must contain at least three vertices")
    tolerance = float(tolerance_m)
    if not np.isfinite(tolerance) or tolerance < 0.0:
        raise ValueError("tolerance_m must be finite and non-negative")

    x = points[:, 0]
    y = points[:, 1]
    inside = np.zeros(len(points), dtype=bool)
    boundary = np.zeros(len(points), dtype=bool)
    previous = polygon[-1]
    for current in polygon:
        x0, y0 = previous
        x1, y1 = current
        dx = x1 - x0
        dy = y1 - y0
        length = math.hypot(dx, dy)
        if length > 0.0:
            cross = (x - x0) * dy - (y - y0) * dx
            on_line = np.abs(cross) <= tolerance * length
            within = (
                (x >= min(x0, x1) - tolerance)
                & (x <= max(x0, x1) + tolerance)
                & (y >= min(y0, y1) - tolerance)
                & (y <= max(y0, y1) + tolerance)
            )
            boundary |= on_line & within

        crosses_y = (y0 > y) != (y1 > y)
        if y1 != y0:
            crossing_x = x0 + (y - y0) * dx / (y1 - y0)
            inside ^= crosses_y & (x < crossing_x)
        previous = current
    return inside | boundary if include_boundary else inside & ~boundary


@dataclass(frozen=True)
class LocalDensity2D:
    """Measured k-neighbour density and spacing at each query location."""

    density_per_m2: np.ndarray
    equivalent_spacing_m: np.ndarray
    nearest_spacing_m: np.ndarray
    support_radius_m: np.ndarray
    neighbour_count: np.ndarray
    nearest_support_distance_m: np.ndarray


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
    chunk_size = max(1, int((64 * 1024 * 1024) // max(8 * len(support), 8)))
    for begin in range(0, len(query), chunk_size):
        end = min(begin + chunk_size, len(query))
        delta = query[begin:end, None, :] - support[None, :, :]
        squared = np.sum(np.square(delta), axis=2)
        selected = np.argsort(squared, axis=1, kind="stable")[:, :count]
        index[begin:end] = selected
        distance[begin:end] = np.sqrt(
            np.take_along_axis(squared, selected, axis=1)
        )
    return distance, index


def measure_local_2d_density(
    query_xy: np.ndarray | Sequence[Sequence[float]],
    support_xy: np.ndarray | Sequence[Sequence[float]],
    *,
    neighbours: int = 12,
    zero_tolerance_m: float = 1.0e-12,
) -> LocalDensity2D:
    """Measure local 2-D density from positive-distance nearest neighbours.

    Exact coincidence is omitted, allowing ``query_xy is support_xy`` to
    measure a cloud without counting each point as its own neighbour.  The
    density estimate is ``k / (pi * r_k**2)`` and equivalent spacing is
    ``1 / sqrt(density)``.  Rows with no positive-distance support are NaN.
    """

    query = _as_xy(query_xy, "query_xy")
    support = _as_xy(support_xy, "support_xy", allow_empty=False)
    if not isinstance(neighbours, (int, np.integer)) or neighbours <= 0:
        raise ValueError("neighbours must be a positive integer")
    tolerance = float(zero_tolerance_m)
    if not np.isfinite(tolerance) or tolerance < 0.0:
        raise ValueError("zero_tolerance_m must be finite and non-negative")
    if not len(query):
        empty_float = np.empty(0, np.float64)
        return LocalDensity2D(
            empty_float.copy(), empty_float.copy(), empty_float.copy(),
            empty_float.copy(), np.empty(0, np.int32), empty_float.copy(),
        )

    requested = min(len(support), int(neighbours) + 1)
    distance, _ = _nearest_neighbours(query, support, requested)
    nearest_support = distance[:, 0].copy()
    positive = np.where(distance > tolerance, distance, np.inf)
    positive.sort(axis=1)
    selected = positive[:, : min(int(neighbours), positive.shape[1])]
    finite = np.isfinite(selected)
    count = np.sum(finite, axis=1).astype(np.int32)
    nearest = np.full(len(query), np.nan, np.float64)
    radius = np.full(len(query), np.nan, np.float64)
    has_neighbour = count > 0
    if np.any(has_neighbour):
        nearest[has_neighbour] = selected[has_neighbour, 0]
        row = np.flatnonzero(has_neighbour)
        radius[row] = selected[row, count[row] - 1]
    density = np.full(len(query), np.nan, np.float64)
    valid = has_neighbour & (radius > 0.0)
    density[valid] = count[valid] / (math.pi * np.square(radius[valid]))
    spacing = np.full(len(query), np.nan, np.float64)
    spacing[valid] = 1.0 / np.sqrt(density[valid])
    return LocalDensity2D(
        density_per_m2=density,
        equivalent_spacing_m=spacing,
        nearest_spacing_m=nearest,
        support_radius_m=radius,
        neighbour_count=count,
        nearest_support_distance_m=nearest_support,
    )


@dataclass(frozen=True)
class EastGuideProjection:
    projected_xy: np.ndarray
    segment_index: np.ndarray
    signed_distance_m: np.ndarray


def eastward_signed_distance(
    points_xy: np.ndarray | Sequence[Sequence[float]],
    guide_xy: np.ndarray | Sequence[Sequence[float]],
) -> EastGuideProjection:
    """Project points to a guide and sign Euclidean distance positive east."""

    points = _as_xy(points_xy, "points_xy")
    guide = _as_xy(guide_xy, "guide_xy", allow_empty=False)
    if len(guide) < 2:
        raise ValueError("guide_xy must contain at least two vertices")
    best_squared = np.full(len(points), np.inf, np.float64)
    projection = np.empty((len(points), 2), np.float64)
    segment_index = np.full(len(points), -1, np.int32)
    for index, (first, second) in enumerate(zip(guide[:-1], guide[1:])):
        vector = second - first
        length_squared = float(np.dot(vector, vector))
        if length_squared <= 0.0:
            continue
        position = np.clip(
            ((points - first) @ vector) / length_squared, 0.0, 1.0
        )
        local_projection = first + position[:, None] * vector
        delta = points - local_projection
        squared = np.sum(np.square(delta), axis=1)
        better = squared < best_squared
        best_squared[better] = squared[better]
        projection[better] = local_projection[better]
        segment_index[better] = index
    if np.any(segment_index < 0):
        raise ValueError("guide_xy contains no non-degenerate segment")
    distance = np.sqrt(best_squared)
    east = points[:, 0] >= projection[:, 0]
    signed = np.where(east, distance, -distance)
    signed[distance == 0.0] = 0.0
    return EastGuideProjection(projection, segment_index, signed)


@dataclass(frozen=True)
class EastTaperResult:
    signed_distance_m: np.ndarray
    factor: np.ndarray
    projected_xy: np.ndarray
    segment_index: np.ndarray


def c1_east_taper(
    points_xy: np.ndarray | Sequence[Sequence[float]],
    guide_xy: np.ndarray | Sequence[Sequence[float]],
    *,
    start_m: float = 0.0,
    end_m: float,
    floor_ratio: float = 0.25,
) -> EastTaperResult:
    """Return a bounded C1 density multiplier east of the guide."""

    projection = eastward_signed_distance(points_xy, guide_xy)
    taper = OutwardTaper(
        start=float(start_m), end=float(end_m), floor_ratio=float(floor_ratio)
    )
    factor = taper.factors(projection.signed_distance_m)
    return EastTaperResult(
        signed_distance_m=projection.signed_distance_m,
        factor=factor,
        projected_xy=projection.projected_xy,
        segment_index=projection.segment_index,
    )


def spacing_for_density_taper(
    base_spacing_m: np.ndarray | Sequence[float],
    density_factor: np.ndarray | Sequence[float],
    *,
    maximum_spacing_m: float | None = None,
) -> np.ndarray:
    """Convert a density multiplier to 2-D exclusion spacing.

    Density is inversely proportional to spacing squared, so a multiplier
    ``f`` maps to ``spacing / sqrt(f)``.
    """

    spacing = np.asarray(base_spacing_m, np.float64)
    factor = np.asarray(density_factor, np.float64)
    if spacing.shape != factor.shape:
        raise ValueError("base_spacing_m and density_factor shapes differ")
    if not np.all(np.isfinite(spacing)) or np.any(spacing <= 0.0):
        raise ValueError("base_spacing_m must be finite and positive")
    if not np.all(np.isfinite(factor)) or np.any(factor <= 0.0) or np.any(factor > 1.0):
        raise ValueError("density_factor must lie in (0, 1]")
    result = spacing / np.sqrt(factor)
    if maximum_spacing_m is not None:
        maximum = float(maximum_spacing_m)
        if not np.isfinite(maximum) or maximum <= 0.0:
            raise ValueError("maximum_spacing_m must be finite and positive")
        result = np.minimum(result, maximum)
    return result


@dataclass(frozen=True)
class PointwiseSelection:
    selected_indices: np.ndarray
    rejected_indices: np.ndarray
    selection_order: np.ndarray
    reason_mask: np.ndarray
    radius_m: np.ndarray


def deterministic_pointwise_thinning(
    points_xy: np.ndarray | Sequence[Sequence[float]],
    radius_m: np.ndarray | Sequence[float] | float,
    *,
    blocker_xy: np.ndarray | Sequence[Sequence[float]] | None = None,
    blocker_radius_m: np.ndarray | Sequence[float] | float | None = None,
    priority: np.ndarray | Sequence[float] | None = None,
    seed: int = DEFAULT_SEED,
    rebuild_interval: int = 256,
) -> PointwiseSelection:
    """Thin points using only pairwise variable-radius clearance."""

    points = _as_xy(points_xy, "points_xy")
    radius = np.asarray(radius_m, np.float64)
    if radius.ndim == 0:
        radius = np.full(len(points), float(radius), np.float64)
    if radius.shape != (len(points),):
        raise ValueError("radius_m must be scalar or contain one value per point")
    blockers = None if blocker_xy is None else _as_xy(blocker_xy, "blocker_xy")
    result: BlueNoiseResult = variable_radius_blue_noise(
        points,
        radius,
        existing_points=blockers,
        existing_radius=blocker_radius_m,
        priority=priority,
        seed=seed,
        rebuild_interval=rebuild_interval,
    )
    selected = np.sort(result.selected_indices)
    selected_mask = np.zeros(len(points), bool)
    selected_mask[selected] = True
    return PointwiseSelection(
        selected_indices=selected,
        rejected_indices=np.flatnonzero(~selected_mask),
        selection_order=result.selected_indices.copy(),
        reason_mask=result.reason_mask.copy(),
        radius_m=radius.copy(),
    )


def continuous_variable_radius_additions(
    candidate_xy: np.ndarray | Sequence[Sequence[float]],
    radius_m: np.ndarray | Sequence[float] | float,
    *,
    terrain_xy: np.ndarray | Sequence[Sequence[float]],
    water_xy: np.ndarray | Sequence[Sequence[float]],
    priority: np.ndarray | Sequence[float] | None = None,
    seed: int = DEFAULT_SEED,
    rebuild_interval: int = 256,
) -> PointwiseSelection:
    """Select additions against all existing SAND/ROCK and WATER support."""

    terrain = _as_xy(terrain_xy, "terrain_xy")
    water = _as_xy(water_xy, "water_xy")
    if len(terrain) and len(water):
        blockers = np.concatenate((terrain, water), axis=0)
    elif len(terrain):
        blockers = terrain
    else:
        blockers = water
    return deterministic_pointwise_thinning(
        candidate_xy,
        radius_m,
        blocker_xy=blockers,
        priority=priority,
        seed=seed,
        rebuild_interval=rebuild_interval,
    )


def _finite_local_spacing(
    query: np.ndarray,
    support: np.ndarray,
    *,
    neighbours: int,
    minimum_spacing_m: float,
    maximum_base_spacing_m: float,
) -> tuple[np.ndarray, LocalDensity2D]:
    measured = measure_local_2d_density(
        query, support, neighbours=neighbours
    )
    spacing = measured.equivalent_spacing_m.copy()
    fallback = measured.nearest_spacing_m
    missing = ~np.isfinite(spacing) | (spacing <= 0.0)
    spacing[missing] = fallback[missing]
    spacing[~np.isfinite(spacing) | (spacing <= 0.0)] = minimum_spacing_m
    spacing = np.clip(spacing, minimum_spacing_m, maximum_base_spacing_m)
    return spacing, measured


@dataclass(frozen=True)
class WaterDensityRefinementPlan:
    existing_selected_indices: np.ndarray
    existing_rejected_indices: np.ndarray
    reservoir_selected_indices: np.ndarray
    reservoir_rejected_indices: np.ndarray
    existing_reason_mask: np.ndarray
    reservoir_reason_mask: np.ndarray
    existing_radius_m: np.ndarray
    reservoir_radius_m: np.ndarray
    existing_taper_factor: np.ndarray
    reservoir_taper_factor: np.ndarray


def plan_water_density_refinement(
    existing_water_xy: np.ndarray | Sequence[Sequence[float]],
    reservoir_xy: np.ndarray | Sequence[Sequence[float]],
    terrain_xy: np.ndarray | Sequence[Sequence[float]],
    *,
    existing_refine_mask: np.ndarray | Sequence[bool] | None = None,
    reservoir_candidate_mask: np.ndarray | Sequence[bool] | None = None,
    guide_xy: np.ndarray | Sequence[Sequence[float]] | None = None,
    taper_start_m: float = 0.0,
    taper_end_m: float = 1.0,
    taper_floor_ratio: float = 0.25,
    neighbours: int = 12,
    minimum_spacing_m: float = 0.002,
    maximum_base_spacing_m: float = 0.005,
    maximum_tapered_spacing_m: float = 0.020,
    existing_priority: np.ndarray | Sequence[float] | None = None,
    reservoir_priority: np.ndarray | Sequence[float] | None = None,
    seed: int = DEFAULT_SEED,
) -> WaterDensityRefinementPlan:
    """Plan local thinning and additions without modifying record values.

    Existing points outside ``existing_refine_mask`` are fixed blockers and
    always survive.  Points accepted inside the mask are selected before the
    oversample reservoir.  Reservoir points are then tested against terrain,
    fixed WATER, and accepted refined WATER together.
    """

    existing = _as_xy(existing_water_xy, "existing_water_xy")
    reservoir = _as_xy(reservoir_xy, "reservoir_xy")
    terrain = _as_xy(terrain_xy, "terrain_xy")
    minimum = float(minimum_spacing_m)
    maximum_base = float(maximum_base_spacing_m)
    maximum_tapered = float(maximum_tapered_spacing_m)
    if not 0.0 < minimum <= maximum_base <= maximum_tapered:
        raise ValueError(
            "spacing limits must satisfy 0 < minimum <= maximum_base <= maximum_tapered"
        )

    if existing_refine_mask is None:
        refine = np.ones(len(existing), bool)
    else:
        refine = np.asarray(existing_refine_mask, bool)
        if refine.shape != (len(existing),):
            raise ValueError("existing_refine_mask has the wrong shape")
    if reservoir_candidate_mask is None:
        reservoir_candidate = np.ones(len(reservoir), bool)
    else:
        reservoir_candidate = np.asarray(reservoir_candidate_mask, bool)
        if reservoir_candidate.shape != (len(reservoir),):
            raise ValueError("reservoir_candidate_mask has the wrong shape")

    if len(existing) + len(terrain) == 0:
        raise ValueError("measured terrain or existing WATER support is required")
    support = (
        np.concatenate((terrain, existing), axis=0)
        if len(terrain) and len(existing)
        else (terrain if len(terrain) else existing)
    )
    fixed = existing[~refine]
    refine_index = np.flatnonzero(refine)
    refine_xy = existing[refine]
    base_existing, _ = _finite_local_spacing(
        refine_xy,
        support,
        neighbours=neighbours,
        minimum_spacing_m=minimum,
        maximum_base_spacing_m=maximum_base,
    )
    if guide_xy is None:
        existing_factor = np.ones(len(refine_xy), np.float64)
    else:
        existing_factor = c1_east_taper(
            refine_xy,
            guide_xy,
            start_m=taper_start_m,
            end_m=taper_end_m,
            floor_ratio=taper_floor_ratio,
        ).factor
    existing_radius_local = spacing_for_density_taper(
        base_existing,
        existing_factor,
        maximum_spacing_m=maximum_tapered,
    )
    blockers = (
        np.concatenate((terrain, fixed), axis=0)
        if len(terrain) and len(fixed)
        else (terrain if len(terrain) else fixed)
    )
    local_priority = None
    if existing_priority is not None:
        priority = np.asarray(existing_priority, np.float64)
        if priority.shape != (len(existing),):
            raise ValueError("existing_priority has the wrong shape")
        local_priority = priority[refine]
    thinned = deterministic_pointwise_thinning(
        refine_xy,
        existing_radius_local,
        blocker_xy=blockers,
        priority=local_priority,
        seed=seed,
    )
    kept_refine = refine_index[thinned.selected_indices]
    rejected_refine = refine_index[thinned.rejected_indices]
    fixed_index = np.flatnonzero(~refine)
    existing_selected = np.sort(np.concatenate((fixed_index, kept_refine)))
    accepted_water = existing[existing_selected]

    reservoir_index = np.flatnonzero(reservoir_candidate)
    reservoir_xy = reservoir[reservoir_candidate]
    base_reservoir, _ = _finite_local_spacing(
        reservoir_xy,
        support,
        neighbours=neighbours,
        minimum_spacing_m=minimum,
        maximum_base_spacing_m=maximum_base,
    )
    if guide_xy is None:
        reservoir_factor = np.ones(len(reservoir_xy), np.float64)
    else:
        reservoir_factor = c1_east_taper(
            reservoir_xy,
            guide_xy,
            start_m=taper_start_m,
            end_m=taper_end_m,
            floor_ratio=taper_floor_ratio,
        ).factor
    reservoir_radius_local = spacing_for_density_taper(
        base_reservoir,
        reservoir_factor,
        maximum_spacing_m=maximum_tapered,
    )
    local_reservoir_priority = None
    if reservoir_priority is not None:
        priority = np.asarray(reservoir_priority, np.float64)
        if priority.shape != (len(reservoir),):
            raise ValueError("reservoir_priority has the wrong shape")
        local_reservoir_priority = priority[reservoir_candidate]
    additions = continuous_variable_radius_additions(
        reservoir_xy,
        reservoir_radius_local,
        terrain_xy=terrain,
        water_xy=accepted_water,
        priority=local_reservoir_priority,
        seed=seed ^ 0xA5A5A5A5A5A5A5A5,
    )
    reservoir_selected = reservoir_index[additions.selected_indices]
    reservoir_rejected = reservoir_index[additions.rejected_indices]

    existing_reason = np.zeros(len(existing), np.uint32)
    existing_reason[refine_index] = thinned.reason_mask
    reservoir_reason = np.zeros(len(reservoir), np.uint32)
    reservoir_reason[reservoir_index] = additions.reason_mask
    existing_radius = np.full(len(existing), np.nan, np.float64)
    existing_radius[refine_index] = existing_radius_local
    reservoir_radius = np.full(len(reservoir), np.nan, np.float64)
    reservoir_radius[reservoir_index] = reservoir_radius_local
    existing_taper = np.ones(len(existing), np.float64)
    existing_taper[refine_index] = existing_factor
    reservoir_taper = np.ones(len(reservoir), np.float64)
    reservoir_taper[reservoir_index] = reservoir_factor
    return WaterDensityRefinementPlan(
        existing_selected_indices=existing_selected,
        existing_rejected_indices=np.sort(rejected_refine),
        reservoir_selected_indices=np.sort(reservoir_selected),
        reservoir_rejected_indices=np.sort(reservoir_rejected),
        existing_reason_mask=existing_reason,
        reservoir_reason_mask=reservoir_reason,
        existing_radius_m=existing_radius,
        reservoir_radius_m=reservoir_radius,
        existing_taper_factor=existing_taper,
        reservoir_taper_factor=reservoir_taper,
    )


def _polyline_normals(polyline: np.ndarray, side: str) -> np.ndarray:
    if len(polyline) < 2:
        raise ValueError("interface_xy must contain at least two points")
    tangent = np.empty_like(polyline)
    tangent[0] = polyline[1] - polyline[0]
    tangent[-1] = polyline[-1] - polyline[-2]
    if len(polyline) > 2:
        tangent[1:-1] = polyline[2:] - polyline[:-2]
    length = np.linalg.norm(tangent, axis=1)
    if np.any(length <= 0.0):
        raise ValueError("interface_xy contains an unresolved degenerate tangent")
    tangent /= length[:, None]
    normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))
    if side == "east":
        normal[normal[:, 0] < 0.0] *= -1.0
    elif side == "west":
        normal[normal[:, 0] > 0.0] *= -1.0
    elif side == "left":
        pass
    elif side == "right":
        normal *= -1.0
    else:
        raise ValueError("water_side must be east, west, left, or right")
    return normal


@dataclass(frozen=True)
class InterfaceDensityAudit:
    terrain_probe_xy: np.ndarray
    water_probe_xy: np.ndarray
    terrain_spacing_m: np.ndarray
    water_spacing_m: np.ndarray
    spacing_ratio: np.ndarray
    median_spacing_ratio: float
    p90_spacing_ratio: float
    maximum_nearest_support_distance_m: float
    passed: bool


def audit_interface_density(
    interface_xy: np.ndarray | Sequence[Sequence[float]],
    terrain_xy: np.ndarray | Sequence[Sequence[float]],
    water_xy: np.ndarray | Sequence[Sequence[float]],
    *,
    probe_offset_m: float = 0.025,
    water_side: str = "east",
    neighbours: int = 12,
    maximum_median_ratio: float = 1.25,
    maximum_p90_ratio: float = 1.50,
) -> InterfaceDensityAudit:
    """Audit combined point density immediately across a terrain/WATER edge."""

    interface = _as_xy(interface_xy, "interface_xy", allow_empty=False)
    terrain = _as_xy(terrain_xy, "terrain_xy")
    water = _as_xy(water_xy, "water_xy")
    if not len(terrain) or not len(water):
        raise ValueError("terrain_xy and water_xy must both contain support")
    offset = float(probe_offset_m)
    if not np.isfinite(offset) or offset <= 0.0:
        raise ValueError("probe_offset_m must be finite and positive")
    for name, value in (
        ("maximum_median_ratio", maximum_median_ratio),
        ("maximum_p90_ratio", maximum_p90_ratio),
    ):
        if not np.isfinite(value) or value < 1.0:
            raise ValueError(f"{name} must be finite and at least one")
    normal = _polyline_normals(interface, water_side)
    water_probe = interface + normal * offset
    terrain_probe = interface - normal * offset
    support = np.concatenate((terrain, water), axis=0)
    terrain_density = measure_local_2d_density(
        terrain_probe, support, neighbours=neighbours
    )
    water_density = measure_local_2d_density(
        water_probe, support, neighbours=neighbours
    )
    terrain_spacing = terrain_density.equivalent_spacing_m
    water_spacing = water_density.equivalent_spacing_m
    finite = (
        np.isfinite(terrain_spacing)
        & np.isfinite(water_spacing)
        & (terrain_spacing > 0.0)
        & (water_spacing > 0.0)
    )
    ratio = np.full(len(interface), np.inf, np.float64)
    ratio[finite] = np.maximum(
        terrain_spacing[finite] / water_spacing[finite],
        water_spacing[finite] / terrain_spacing[finite],
    )
    if np.any(finite):
        median = float(np.median(ratio[finite]))
        p90 = float(np.quantile(ratio[finite], 0.90))
    else:
        median = math.inf
        p90 = math.inf
    nearest = np.concatenate((
        terrain_density.nearest_support_distance_m,
        water_density.nearest_support_distance_m,
    ))
    max_nearest = float(np.max(nearest)) if len(nearest) else math.inf
    passed = bool(
        np.all(finite)
        and median <= maximum_median_ratio
        and p90 <= maximum_p90_ratio
    )
    return InterfaceDensityAudit(
        terrain_probe_xy=terrain_probe,
        water_probe_xy=water_probe,
        terrain_spacing_m=terrain_spacing,
        water_spacing_m=water_spacing,
        spacing_ratio=ratio,
        median_spacing_ratio=median,
        p90_spacing_ratio=p90,
        maximum_nearest_support_distance_m=max_nearest,
        passed=passed,
    )


@dataclass(frozen=True)
class PlyInfo:
    path: Path
    dtype: np.dtype
    count: int
    offset: int
    header: bytes
    size_bytes: int
    mtime_ns: int


_PLY_TYPES = {
    "float": "<f4",
    "float32": "<f4",
    "double": "<f8",
    "float64": "<f8",
    "uchar": "u1",
    "uint8": "u1",
    "char": "i1",
    "int8": "i1",
    "short": "<i2",
    "int16": "<i2",
    "ushort": "<u2",
    "uint16": "<u2",
    "int": "<i4",
    "int32": "<i4",
    "uint": "<u4",
    "uint32": "<u4",
}


def inspect_fixed_stride_ply(path: str | Path) -> PlyInfo:
    """Inspect a fixed-stride binary-little-endian vertex PLY."""

    source = Path(path)
    fields: list[tuple[str, str]] = []
    count: int | None = None
    current_element: str | None = None
    format_seen = False
    header = bytearray()
    with source.open("rb") as handle:
        first = handle.readline()
        if first.rstrip(b"\r\n") != b"ply":
            raise RuntimeError(f"not a PLY file: {source}")
        header.extend(first)
        while True:
            line = handle.readline()
            if not line:
                raise RuntimeError(f"no end_header in {source}")
            header.extend(line)
            text = line.decode("ascii", "strict").strip()
            parts = text.split()
            if parts[:1] == ["format"]:
                if parts[1:] != ["binary_little_endian", "1.0"]:
                    raise RuntimeError(
                        f"only binary_little_endian PLY 1.0 is supported: {source}"
                    )
                format_seen = True
            elif parts[:1] == ["element"]:
                if len(parts) != 3:
                    raise RuntimeError(f"malformed element line in {source}")
                current_element = parts[1]
                if current_element == "vertex":
                    count = int(parts[2])
            elif parts[:1] == ["property"] and current_element == "vertex":
                if len(parts) != 3 or parts[1] == "list":
                    raise RuntimeError(
                        f"list or malformed vertex property in {source}"
                    )
                if parts[1] not in _PLY_TYPES:
                    raise RuntimeError(
                        f"unsupported PLY property type {parts[1]} in {source}"
                    )
                fields.append((parts[2], _PLY_TYPES[parts[1]]))
            if text == "end_header":
                offset = handle.tell()
                break
    if not format_seen or count is None or count < 0 or not fields:
        raise RuntimeError(f"missing fixed-stride vertex schema in {source}")
    dtype = np.dtype(fields)
    stat = source.stat()
    expected = offset + count * dtype.itemsize
    if stat.st_size != expected:
        raise RuntimeError(
            f"PLY payload size mismatch in {source}: expected {expected}, found {stat.st_size}"
        )
    return PlyInfo(
        path=source,
        dtype=dtype,
        count=count,
        offset=offset,
        header=bytes(header),
        size_bytes=stat.st_size,
        mtime_ns=stat.st_mtime_ns,
    )


def iter_ply_chunks(
    path: str | Path,
    *,
    chunk_size: int = 1_000_000,
    info: PlyInfo | None = None,
) -> Iterator[tuple[int, np.ndarray]]:
    """Yield read-only structured PLY record spans and their source index."""

    if not isinstance(chunk_size, (int, np.integer)) or chunk_size <= 0:
        raise ValueError("chunk_size must be a positive integer")
    source_info = inspect_fixed_stride_ply(path) if info is None else info
    memory = np.memmap(
        source_info.path,
        dtype=source_info.dtype,
        mode="r",
        offset=source_info.offset,
        shape=(source_info.count,),
    )
    try:
        for begin in range(0, source_info.count, int(chunk_size)):
            yield begin, memory[begin : begin + int(chunk_size)]
    finally:
        del memory


@dataclass(frozen=True)
class PlyXYRecords:
    info: PlyInfo
    indices: np.ndarray
    xy: np.ndarray


def collect_ply_xy(
    path: str | Path,
    *,
    bbox: Sequence[float] | None = None,
    polygon_xy: np.ndarray | Sequence[Sequence[float]] | None = None,
    field_equals: Mapping[str, float | int] | None = None,
    record_filter: Callable[[np.ndarray], np.ndarray] | None = None,
    chunk_size: int = 1_000_000,
) -> PlyXYRecords:
    """Stream a PLY and materialise only selected source indices and XY."""

    info = inspect_fixed_stride_ply(path)
    if "x" not in info.dtype.names or "y" not in info.dtype.names:
        raise RuntimeError(f"PLY has no x/y fields: {info.path}")
    equality = dict(field_equals or {})
    missing = sorted(set(equality) - set(info.dtype.names))
    if missing:
        raise RuntimeError(f"PLY is missing filter fields {missing}: {info.path}")
    index_parts: list[np.ndarray] = []
    xy_parts: list[np.ndarray] = []
    for begin, chunk in iter_ply_chunks(
        info.path, chunk_size=chunk_size, info=info
    ):
        xy = np.column_stack((chunk["x"], chunk["y"])).astype(
            np.float64, copy=False
        )
        keep = np.ones(len(chunk), bool)
        if bbox is not None:
            keep &= bbox_mask(xy, bbox)
        if polygon_xy is not None:
            keep &= polygon_mask(xy, polygon_xy)
        for field, value in equality.items():
            keep &= chunk[field] == value
        if record_filter is not None:
            custom = np.asarray(record_filter(chunk), bool)
            if custom.shape != (len(chunk),):
                raise ValueError("record_filter returned the wrong shape")
            keep &= custom
        local = np.flatnonzero(keep)
        if len(local):
            index_parts.append(local.astype(np.int64) + begin)
            xy_parts.append(np.asarray(xy[local], np.float64))
    indices = (
        np.concatenate(index_parts)
        if index_parts
        else np.empty(0, np.int64)
    )
    xy = (
        np.concatenate(xy_parts, axis=0)
        if xy_parts
        else np.empty((0, 2), np.float64)
    )
    return PlyXYRecords(info=info, indices=indices, xy=xy)


def make_near_surface_record_filter(
    reference_height: Callable[[np.ndarray, np.ndarray], np.ndarray],
    *,
    below_m: float = 0.015,
    above_m: float = 0.025,
) -> Callable[[np.ndarray], np.ndarray]:
    """Build a streaming terrain filter around a WATER reference surface.

    Canonical SAND/ROCK can contain vertically unrelated reflection returns at
    the same XY as valid WATER.  Such points must not contribute density or
    block additions.  ``reference_height`` receives chunk ``x`` and ``y``
    arrays and returns the local WATER-reference ``z``.  The default collar
    matches the successful v10 near-sheet policy.
    """

    below = float(below_m)
    above = float(above_m)
    if not np.isfinite(below) or not np.isfinite(above) or below < 0.0 or above < 0.0:
        raise ValueError("below_m and above_m must be finite and non-negative")

    def record_filter(records: np.ndarray) -> np.ndarray:
        names = set(records.dtype.names or ())
        missing = sorted({"x", "y", "z"} - names)
        if missing:
            raise RuntimeError(f"terrain records are missing coordinate fields {missing}")
        x = np.asarray(records["x"], np.float64)
        y = np.asarray(records["y"], np.float64)
        z = np.asarray(records["z"], np.float64)
        reference = np.asarray(reference_height(x, y), np.float64)
        if reference.shape != (len(records),):
            raise ValueError("reference_height returned the wrong shape")
        delta = z - reference
        return (
            np.isfinite(x)
            & np.isfinite(y)
            & np.isfinite(z)
            & np.isfinite(reference)
            & (delta >= -below)
            & (delta <= above)
        )

    return record_filter


@dataclass(frozen=True)
class PlyRecordSelection:
    """Compact source selection: ``indices`` invert ``keep_by_default``."""

    info: PlyInfo
    keep_by_default: bool
    indices: np.ndarray
    label: str

    @property
    def selected_count(self) -> int:
        if self.keep_by_default:
            return self.info.count - len(self.indices)
        return len(self.indices)

    def mask_for_span(self, begin: int, length: int) -> np.ndarray:
        end = begin + length
        left = int(np.searchsorted(self.indices, begin, side="left"))
        right = int(np.searchsorted(self.indices, end, side="left"))
        local = self.indices[left:right] - begin
        mask = np.full(length, self.keep_by_default, bool)
        mask[local] = not self.keep_by_default
        return mask


def make_ply_record_selection(
    info_or_path: PlyInfo | str | Path,
    *,
    keep_by_default: bool,
    indices: np.ndarray | Sequence[int],
    label: str,
) -> PlyRecordSelection:
    info = (
        info_or_path
        if isinstance(info_or_path, PlyInfo)
        else inspect_fixed_stride_ply(info_or_path)
    )
    values = np.asarray(indices, np.int64)
    if values.ndim != 1:
        raise ValueError("indices must be one-dimensional")
    values = np.unique(values)
    if len(values) and (values[0] < 0 or values[-1] >= info.count):
        raise ValueError("selection index lies outside its source PLY")
    return PlyRecordSelection(
        info=info,
        keep_by_default=bool(keep_by_default),
        indices=values,
        label=str(label),
    )


def _write_ply_header(
    handle,
    dtype: np.dtype,
    count: int,
    comments: Iterable[str],
) -> None:
    typemap = {
        "f4": "float", "f8": "double", "u1": "uchar", "i1": "char",
        "i2": "short", "u2": "ushort", "i4": "int", "u4": "uint",
    }
    handle.write(b"ply\nformat binary_little_endian 1.0\n")
    for comment in comments:
        text = str(comment)
        if "\n" in text or "\r" in text:
            raise ValueError("PLY comments must be single-line")
        handle.write(f"comment {text}\n".encode("ascii"))
    handle.write(f"element vertex {count}\n".encode("ascii"))
    for name in dtype.names or ():
        code = dtype[name].str.lstrip("<>=|")
        if code not in typemap:
            raise RuntimeError(f"unsupported output dtype {dtype[name]} for {name}")
        handle.write(f"property {typemap[code]} {name}\n".encode("ascii"))
    handle.write(b"end_header\n")


def _same_path(first: Path, second: Path) -> bool:
    return first.expanduser().resolve(strict=False) == second.expanduser().resolve(
        strict=False
    )


def assert_candidate_output_path(
    output_path: str | Path,
    *,
    source_paths: Iterable[str | Path] = (),
    forbidden_paths: Iterable[str | Path] = (),
) -> Path:
    """Reject canonical names, source paths, and explicitly forbidden paths."""

    output = Path(output_path)
    if CANONICAL_SITE1_CLOUD_NAME.fullmatch(output.name):
        raise ValueError(f"refusing canonical Site1 cloud output path: {output}")
    for path in (*tuple(source_paths), *tuple(forbidden_paths)):
        if _same_path(output, Path(path)):
            raise ValueError(f"candidate output aliases protected path: {output}")
    return output


def write_candidate_ply(
    output_path: str | Path,
    selections: Sequence[PlyRecordSelection],
    *,
    comments: Iterable[str] = (),
    forbidden_paths: Iterable[str | Path] = (),
    required_scan_id: float | None = 999.0,
    overwrite: bool = False,
    chunk_size: int = 1_000_000,
) -> int:
    """Stream selected source records to a non-canonical candidate PLY.

    All sources must have the same schema.  Structured records are copied as
    complete values; this function never recomputes geometry or scalars.
    """

    if not selections:
        raise ValueError("at least one source selection is required")
    dtype = selections[0].info.dtype
    for selection in selections:
        if selection.info.dtype != dtype:
            raise RuntimeError(
                "source PLY schemas differ; refusing a lossy field projection"
            )
    required_coordinates = {"x", "y", "z"}
    missing_coordinates = sorted(required_coordinates - set(dtype.names or ()))
    if missing_coordinates:
        raise RuntimeError(
            f"WATER candidate schema is missing coordinates {missing_coordinates}"
        )
    if required_scan_id is not None and "scalar_ScanID" not in (dtype.names or ()):
        raise RuntimeError("WATER candidate schema is missing scalar_ScanID")
    output = assert_candidate_output_path(
        output_path,
        source_paths=[selection.info.path for selection in selections],
        forbidden_paths=forbidden_paths,
    )
    if output.exists() and not overwrite:
        raise FileExistsError(f"candidate already exists: {output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_name(f".{output.name}.{os.getpid()}.partial")
    if temporary.exists():
        temporary.unlink()
    expected = sum(selection.selected_count for selection in selections)
    written = 0
    try:
        with temporary.open("wb") as handle:
            _write_ply_header(
                handle,
                dtype,
                expected,
                [
                    "Scene1 v11 candidate-only WATER density refinement",
                    "Selected source records retain v10 xyz and scalar values",
                    *comments,
                ],
            )
            for selection in selections:
                before = selection.info.path.stat()
                if (
                    before.st_size != selection.info.size_bytes
                    or before.st_mtime_ns != selection.info.mtime_ns
                ):
                    raise RuntimeError(
                        f"source changed after planning: {selection.info.path}"
                    )
                for begin, chunk in iter_ply_chunks(
                    selection.info.path,
                    chunk_size=chunk_size,
                    info=selection.info,
                ):
                    keep = selection.mask_for_span(begin, len(chunk))
                    if np.any(keep):
                        selected = np.asarray(chunk[keep])
                        finite_xyz = (
                            np.isfinite(selected["x"])
                            & np.isfinite(selected["y"])
                            & np.isfinite(selected["z"])
                        )
                        if not np.all(finite_xyz):
                            raise RuntimeError(
                                f"selected WATER contains non-finite XYZ: {selection.info.path}"
                            )
                        if required_scan_id is not None and not np.all(
                            selected["scalar_ScanID"] == required_scan_id
                        ):
                            raise RuntimeError(
                                f"selected records do not all have ScanID={required_scan_id:g}: "
                                f"{selection.info.path}"
                            )
                        selected.tofile(handle)
                        written += int(np.count_nonzero(keep))
                after = selection.info.path.stat()
                if (
                    after.st_size != selection.info.size_bytes
                    or after.st_mtime_ns != selection.info.mtime_ns
                ):
                    raise RuntimeError(
                        f"source changed during candidate build: {selection.info.path}"
                    )
        if written != expected:
            raise RuntimeError(f"candidate wrote {written} records, expected {expected}")
        os.replace(temporary, output)
    except BaseException:
        if temporary.exists():
            temporary.unlink()
        raise
    return written


@dataclass(frozen=True)
class StreamingWaterPlan:
    existing_records: PlyXYRecords
    reservoir_records: PlyXYRecords
    terrain_record_count: int
    density_plan: WaterDensityRefinementPlan
    output_selections: tuple[PlyRecordSelection, PlyRecordSelection]


@dataclass(frozen=True)
class StreamingCandidateParts:
    """Schema-preserving existing and reservoir candidate artifacts."""

    existing_path: Path
    reservoir_path: Path
    existing_count: int
    reservoir_count: int


def plan_streaming_water_candidate(
    existing_water_path: str | Path,
    reservoir_path: str | Path,
    terrain_paths: Sequence[str | Path],
    *,
    review_bbox: Sequence[float],
    review_polygon_xy: np.ndarray | Sequence[Sequence[float]] | None = None,
    guide_xy: np.ndarray | Sequence[Sequence[float]] | None = None,
    support_margin_m: float = 0.25,
    terrain_record_filters: Sequence[
        Callable[[np.ndarray], np.ndarray] | None
    ] | None = None,
    terrain_support_is_prevalidated: bool = False,
    water_scan_id: float = 999.0,
    reservoir_scan_id_field: str | None = "scalar_ScanID",
    taper_start_m: float = 0.0,
    taper_end_m: float = 1.0,
    taper_floor_ratio: float = 0.25,
    neighbours: int = 12,
    minimum_spacing_m: float = 0.002,
    maximum_base_spacing_m: float = 0.005,
    maximum_tapered_spacing_m: float = 0.020,
    seed: int = DEFAULT_SEED,
    chunk_size: int = 1_000_000,
) -> StreamingWaterPlan:
    """Plan a local candidate from existing WATER and a v10 reservoir PLY.

    Only XY and source indices inside the review/support collar are retained
    in memory.  Existing records outside the review mask remain selected by
    default.  The full-schema v10 2 mm candidate is the preferred reservoir
    for a directly writable 5 mm candidate.  The prefix-schema combined v10
    oversample can also be planned, but must be emitted as a separate part and
    reanalysed rather than projected into the full schema.  When
    ``reservoir_scan_id_field`` is set, only ScanID 999 records are eligible.

    Raw canonical terrain is unsafe as XY support because reflections can be
    vertically unrelated to the WATER surface.  Supply one near-sheet
    ``terrain_record_filters`` entry per terrain source, normally made with
    :func:`make_near_surface_record_filter`.  Already filtered fixtures or
    dedicated near-sheet sources must opt in with
    ``terrain_support_is_prevalidated=True``.
    """

    review = _as_bbox(review_bbox)
    if support_margin_m < maximum_tapered_spacing_m:
        raise ValueError(
            "support_margin_m must be at least maximum_tapered_spacing_m"
        )
    if terrain_record_filters is None:
        if terrain_paths and not terrain_support_is_prevalidated:
            raise ValueError(
                "raw terrain support requires near-sheet record filters or "
                "terrain_support_is_prevalidated=True"
            )
        filters: list[Callable[[np.ndarray], np.ndarray] | None] = [
            None for _ in terrain_paths
        ]
    else:
        filters = list(terrain_record_filters)
        if len(filters) != len(terrain_paths):
            raise ValueError(
                "terrain_record_filters must contain one entry per terrain path"
            )
    collar = expand_bbox(review, support_margin_m)
    existing_records = collect_ply_xy(
        existing_water_path, bbox=collar, chunk_size=chunk_size
    )
    refine = bbox_mask(existing_records.xy, review)
    if review_polygon_xy is not None:
        refine &= polygon_mask(existing_records.xy, review_polygon_xy)

    equality = (
        {reservoir_scan_id_field: water_scan_id}
        if reservoir_scan_id_field is not None
        else None
    )
    reservoir_records = collect_ply_xy(
        reservoir_path,
        bbox=review,
        polygon_xy=review_polygon_xy,
        field_equals=equality,
        chunk_size=chunk_size,
    )
    terrain_parts = [
        collect_ply_xy(
            path,
            bbox=collar,
            record_filter=record_filter,
            chunk_size=chunk_size,
        )
        for path, record_filter in zip(terrain_paths, filters)
    ]
    terrain_xy = (
        np.concatenate([part.xy for part in terrain_parts], axis=0)
        if terrain_parts
        else np.empty((0, 2), np.float64)
    )
    density_plan = plan_water_density_refinement(
        existing_records.xy,
        reservoir_records.xy,
        terrain_xy,
        existing_refine_mask=refine,
        guide_xy=guide_xy,
        taper_start_m=taper_start_m,
        taper_end_m=taper_end_m,
        taper_floor_ratio=taper_floor_ratio,
        neighbours=neighbours,
        minimum_spacing_m=minimum_spacing_m,
        maximum_base_spacing_m=maximum_base_spacing_m,
        maximum_tapered_spacing_m=maximum_tapered_spacing_m,
        seed=seed,
    )
    rejected_existing_source = existing_records.indices[
        density_plan.existing_rejected_indices
    ]
    accepted_reservoir_source = reservoir_records.indices[
        density_plan.reservoir_selected_indices
    ]
    existing_selection = make_ply_record_selection(
        existing_records.info,
        keep_by_default=True,
        indices=rejected_existing_source,
        label="existing-v10-water-minus-pointwise-thinning",
    )
    reservoir_selection = make_ply_record_selection(
        reservoir_records.info,
        keep_by_default=False,
        indices=accepted_reservoir_source,
        label="accepted-v10-oversample-reservoir-additions",
    )
    return StreamingWaterPlan(
        existing_records=existing_records,
        reservoir_records=reservoir_records,
        terrain_record_count=sum(len(part.indices) for part in terrain_parts),
        density_plan=density_plan,
        output_selections=(existing_selection, reservoir_selection),
    )


def write_streaming_candidate_parts(
    plan: StreamingWaterPlan,
    existing_output_path: str | Path,
    reservoir_output_path: str | Path,
    *,
    comments: Iterable[str] = (),
    forbidden_paths: Iterable[str | Path] = (),
    overwrite: bool = False,
    chunk_size: int = 1_000_000,
) -> StreamingCandidateParts:
    """Write schema-preserving candidate parts from a streaming plan.

    The actual v10 oversample reservoir has a short pre-analysis schema while
    the accepted v10 candidate has the full analysed schema.  Silently
    projecting either source would discard or invent scalar fields.  This
    helper therefore emits two explicit candidate-only PLYs, each preserving
    its source schema and complete selected record values.  Callers may use
    :func:`write_candidate_ply` directly for a single combined candidate only
    when both selections already have identical schemas.
    """

    existing_output = assert_candidate_output_path(
        existing_output_path,
        source_paths=[selection.info.path for selection in plan.output_selections],
        forbidden_paths=forbidden_paths,
    )
    reservoir_output = assert_candidate_output_path(
        reservoir_output_path,
        source_paths=[selection.info.path for selection in plan.output_selections],
        forbidden_paths=forbidden_paths,
    )
    if _same_path(existing_output, reservoir_output):
        raise ValueError("existing and reservoir candidate outputs must differ")
    common_comments = [
        "Split candidate part; no schema projection or scalar recomputation",
        *comments,
    ]
    existing_count = write_candidate_ply(
        existing_output,
        [plan.output_selections[0]],
        comments=["Accepted existing v10 WATER records", *common_comments],
        forbidden_paths=forbidden_paths,
        overwrite=overwrite,
        chunk_size=chunk_size,
    )
    reservoir_count = write_candidate_ply(
        reservoir_output,
        [plan.output_selections[1]],
        comments=["Accepted v10 oversample reservoir records", *common_comments],
        forbidden_paths=forbidden_paths,
        overwrite=overwrite,
        chunk_size=chunk_size,
    )
    return StreamingCandidateParts(
        existing_path=existing_output,
        reservoir_path=reservoir_output,
        existing_count=existing_count,
        reservoir_count=reservoir_count,
    )


__all__ = [
    "DEFAULT_SEED",
    "EastGuideProjection",
    "EastTaperResult",
    "InterfaceDensityAudit",
    "LocalDensity2D",
    "PlyInfo",
    "PlyRecordSelection",
    "PlyXYRecords",
    "PointwiseSelection",
    "StreamingWaterPlan",
    "StreamingCandidateParts",
    "WaterDensityRefinementPlan",
    "assert_candidate_output_path",
    "audit_interface_density",
    "bbox_mask",
    "c1_east_taper",
    "collect_ply_xy",
    "continuous_variable_radius_additions",
    "deterministic_pointwise_thinning",
    "eastward_signed_distance",
    "expand_bbox",
    "inspect_fixed_stride_ply",
    "iter_ply_chunks",
    "make_ply_record_selection",
    "make_near_surface_record_filter",
    "measure_local_2d_density",
    "plan_streaming_water_candidate",
    "plan_water_density_refinement",
    "polygon_mask",
    "spacing_for_density_taper",
    "write_candidate_ply",
    "write_streaming_candidate_parts",
]
