#!/usr/bin/env python3
"""Seeded, support-derived WATER-hole reconstruction for Scene1 v11.

Screenshot marks are used only to select a nearby *measured empty component*.
The component itself is derived from WATER and terrain support, must be locally
enclosed, and must pass angular support checks.  This prevents review boxes
from becoming rectangular fill masks.

The module only plans XY additions.  A caller must independently validate the
surface height, construct full PLY records, and write a candidate artifact.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from enum import IntFlag
import math
from typing import Mapping, Sequence

import numpy as np

from site1_v11_confidence import variable_radius_blue_noise


class HoleReason(IntFlag):
    NONE = 0
    NO_NEARBY_EMPTY_CELL = 1 << 0
    COMPONENT_TOUCHES_REVIEW_BOUNDARY = 1 << 1
    COMPONENT_TOO_SMALL = 1 << 2
    COMPONENT_TOO_LARGE = 1 << 3
    INSUFFICIENT_SUPPORT_SECTORS = 1 << 4
    INSUFFICIENT_WATER_SECTORS = 1 << 5


@dataclass(frozen=True)
class SeededHolePolicy:
    diagnostic_cell_m: float
    water_support_radius_m: float
    terrain_support_radius_m: float
    seed_search_radius_m: float = 0.30
    minimum_area_m2: float = 0.0010
    maximum_area_m2: float = 0.75
    sector_radius_m: float = 0.45
    minimum_support_sectors: int = 6
    minimum_water_sectors: int = 3
    sectors: int = 8

    def validate(self) -> None:
        positive = (
            self.diagnostic_cell_m,
            self.water_support_radius_m,
            self.terrain_support_radius_m,
            self.seed_search_radius_m,
            self.minimum_area_m2,
            self.maximum_area_m2,
            self.sector_radius_m,
        )
        if not all(np.isfinite(value) and value > 0.0 for value in positive):
            raise ValueError("hole policy distances and areas must be positive")
        if self.minimum_area_m2 > self.maximum_area_m2:
            raise ValueError("minimum_area_m2 must not exceed maximum_area_m2")
        if not 4 <= self.sectors <= 32:
            raise ValueError("sectors must be in [4, 32]")
        if not 1 <= self.minimum_water_sectors <= self.minimum_support_sectors:
            raise ValueError("water sector requirement must be positive and bounded")
        if self.minimum_support_sectors > self.sectors:
            raise ValueError("minimum_support_sectors must not exceed sectors")


@dataclass(frozen=True)
class HoleGrid:
    bbox: tuple[float, float, float, float]
    cell_m: float
    x_centres: np.ndarray
    y_centres: np.ndarray
    water_distance_m: np.ndarray
    terrain_distance_m: np.ndarray
    empty: np.ndarray
    labels: np.ndarray


@dataclass(frozen=True)
class SeededHole:
    seed_id: str
    seed_xy: tuple[float, float]
    label: int
    accepted: bool
    reason_mask: int
    area_m2: float
    centroid_xy: tuple[float, float]
    bounds: tuple[float, float, float, float]
    seed_distance_m: float
    support_sectors: int
    water_sectors: int
    cell_indices: np.ndarray


@dataclass(frozen=True)
class HolePlan:
    grid: HoleGrid
    holes: tuple[SeededHole, ...]

    @property
    def accepted_labels(self) -> tuple[int, ...]:
        return tuple(sorted({hole.label for hole in self.holes if hole.accepted}))


@dataclass(frozen=True)
class HoleSamples:
    xy: np.ndarray
    source_candidate_count: int
    rejected_for_terrain_clearance: int
    rejected_for_blue_noise: int


def _xy(values, name: str, *, allow_empty: bool = True) -> np.ndarray:
    result = np.asarray(values, dtype=np.float64)
    if result.ndim != 2 or result.shape[1] < 2:
        raise ValueError(f"{name} must have shape (N, D), D >= 2")
    result = result[:, :2]
    if not allow_empty and not len(result):
        raise ValueError(f"{name} must not be empty")
    if not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be finite")
    return result


def _bbox(values: Sequence[float]) -> tuple[float, float, float, float]:
    result = np.asarray(values, dtype=np.float64)
    if result.shape != (4,) or not np.all(np.isfinite(result)):
        raise ValueError("bbox must contain four finite values")
    xmin, xmax, ymin, ymax = map(float, result)
    if not xmin < xmax or not ymin < ymax:
        raise ValueError("bbox must satisfy xmin < xmax and ymin < ymax")
    return xmin, xmax, ymin, ymax


def _nearest_distance(query: np.ndarray, support: np.ndarray) -> np.ndarray:
    if not len(support):
        return np.full(len(query), np.inf, dtype=np.float64)
    try:
        from scipy.spatial import cKDTree
    except ModuleNotFoundError:
        cKDTree = None
    if cKDTree is not None:
        distance, _ = cKDTree(support).query(query, k=1, workers=-1)
        return np.asarray(distance, dtype=np.float64)
    # Test-sized fallback; production uses the explicitly required SciPy
    # numeric environment.
    output = np.empty(len(query), dtype=np.float64)
    chunk = max(1, int((32 * 1024 * 1024) // max(16 * len(support), 16)))
    for begin in range(0, len(query), chunk):
        end = min(begin + chunk, len(query))
        delta = query[begin:end, None, :] - support[None, :, :]
        output[begin:end] = np.sqrt(np.min(np.sum(delta * delta, axis=2), axis=1))
    return output


def _label_four_connected(mask: np.ndarray) -> tuple[np.ndarray, int]:
    try:
        from scipy.ndimage import label
    except ModuleNotFoundError:
        label = None
    if label is not None:
        labels, count = label(
            mask,
            structure=np.array([[0, 1, 0], [1, 1, 1], [0, 1, 0]], dtype=np.uint8),
        )
        return labels.astype(np.int32, copy=False), int(count)
    labels = np.zeros(mask.shape, dtype=np.int32)
    height, width = mask.shape
    current = 0
    for row, col in zip(*np.nonzero(mask)):
        if labels[row, col]:
            continue
        current += 1
        labels[row, col] = current
        queue = deque([(int(row), int(col))])
        while queue:
            y, x = queue.popleft()
            for ny, nx in ((y - 1, x), (y + 1, x), (y, x - 1), (y, x + 1)):
                if (
                    0 <= ny < height
                    and 0 <= nx < width
                    and mask[ny, nx]
                    and labels[ny, nx] == 0
                ):
                    labels[ny, nx] = current
                    queue.append((ny, nx))
    return labels, current


def build_hole_grid(
    water_xy,
    terrain_xy,
    *,
    review_bbox: Sequence[float],
    policy: SeededHolePolicy,
) -> HoleGrid:
    """Rasterise exact nearest-support distances for component discovery."""

    policy.validate()
    water = _xy(water_xy, "water_xy")
    terrain = _xy(terrain_xy, "terrain_xy")
    xmin, xmax, ymin, ymax = _bbox(review_bbox)
    cell = float(policy.diagnostic_cell_m)
    columns = int(math.ceil((xmax - xmin) / cell))
    rows = int(math.ceil((ymax - ymin) / cell))
    x_centres = xmin + (np.arange(columns, dtype=np.float64) + 0.5) * cell
    y_centres = ymin + (np.arange(rows, dtype=np.float64) + 0.5) * cell
    xx, yy = np.meshgrid(x_centres, y_centres)
    query = np.column_stack((xx.ravel(), yy.ravel()))
    water_distance = _nearest_distance(query, water).reshape(rows, columns)
    terrain_distance = _nearest_distance(query, terrain).reshape(rows, columns)
    empty = (
        (water_distance > policy.water_support_radius_m)
        & (terrain_distance > policy.terrain_support_radius_m)
    )
    labels, _ = _label_four_connected(empty)
    return HoleGrid(
        bbox=(xmin, xmax, ymin, ymax),
        cell_m=cell,
        x_centres=x_centres,
        y_centres=y_centres,
        water_distance_m=water_distance,
        terrain_distance_m=terrain_distance,
        empty=empty,
        labels=labels,
    )


def _sector_count(origin: np.ndarray, support: np.ndarray, radius: float, sectors: int) -> int:
    if not len(support):
        return 0
    delta = support - origin
    distance = np.sqrt(np.sum(delta * delta, axis=1))
    keep = (distance > 0.0) & (distance <= radius)
    if not np.any(keep):
        return 0
    angle = np.mod(np.arctan2(delta[keep, 1], delta[keep, 0]), 2.0 * math.pi)
    bins = np.floor(angle * sectors / (2.0 * math.pi)).astype(np.int32)
    return int(len(np.unique(np.clip(bins, 0, sectors - 1))))


def detect_seeded_holes(
    water_xy,
    terrain_xy,
    *,
    review_bbox: Sequence[float],
    seeds: Mapping[str, Sequence[float]],
    policy: SeededHolePolicy,
) -> HolePlan:
    """Resolve each review seed to one nearby measured empty component."""

    water = _xy(water_xy, "water_xy")
    terrain = _xy(terrain_xy, "terrain_xy")
    grid = build_hole_grid(
        water, terrain, review_bbox=review_bbox, policy=policy
    )
    xx, yy = np.meshgrid(grid.x_centres, grid.y_centres)
    centres = np.column_stack((xx.ravel(), yy.ravel()))
    empty_index = np.flatnonzero(grid.empty.ravel())
    empty_centres = centres[empty_index]
    holes: list[SeededHole] = []
    cache: dict[int, tuple[np.ndarray, float, tuple[float, float], tuple[float, ...], bool]] = {}

    for seed_id, raw_seed in seeds.items():
        seed = np.asarray(raw_seed, dtype=np.float64)
        if seed.shape != (2,) or not np.all(np.isfinite(seed)):
            raise ValueError(f"seed {seed_id!r} must contain two finite values")
        reason = HoleReason.NONE
        label_id = 0
        seed_distance = math.inf
        if len(empty_centres):
            distance = np.sqrt(np.sum(np.square(empty_centres - seed), axis=1))
            selected = int(np.argmin(distance))
            seed_distance = float(distance[selected])
            if seed_distance <= policy.seed_search_radius_m:
                label_id = int(grid.labels.ravel()[empty_index[selected]])
        if label_id == 0:
            reason |= HoleReason.NO_NEARBY_EMPTY_CELL
            cells = np.empty(0, dtype=np.int64)
            area = 0.0
            centroid = (float(seed[0]), float(seed[1]))
            bounds = (float(seed[0]), float(seed[0]), float(seed[1]), float(seed[1]))
            touches = False
        else:
            if label_id not in cache:
                cells = np.flatnonzero(grid.labels.ravel() == label_id)
                rows, cols = np.unravel_index(cells, grid.labels.shape)
                component_xy = centres[cells]
                area = float(len(cells) * grid.cell_m * grid.cell_m)
                centroid = tuple(np.mean(component_xy, axis=0).tolist())
                bounds = (
                    float(np.min(component_xy[:, 0]) - 0.5 * grid.cell_m),
                    float(np.max(component_xy[:, 0]) + 0.5 * grid.cell_m),
                    float(np.min(component_xy[:, 1]) - 0.5 * grid.cell_m),
                    float(np.max(component_xy[:, 1]) + 0.5 * grid.cell_m),
                )
                touches = bool(
                    np.any(rows == 0)
                    or np.any(rows == grid.labels.shape[0] - 1)
                    or np.any(cols == 0)
                    or np.any(cols == grid.labels.shape[1] - 1)
                )
                cache[label_id] = (cells, area, centroid, bounds, touches)
            cells, area, centroid, bounds, touches = cache[label_id]
            if touches:
                reason |= HoleReason.COMPONENT_TOUCHES_REVIEW_BOUNDARY
            if area < policy.minimum_area_m2:
                reason |= HoleReason.COMPONENT_TOO_SMALL
            if area > policy.maximum_area_m2:
                reason |= HoleReason.COMPONENT_TOO_LARGE

        origin = np.asarray(centroid, dtype=np.float64)
        combined = (
            np.concatenate((water, terrain), axis=0)
            if len(water) and len(terrain)
            else (water if len(water) else terrain)
        )
        support_sectors = _sector_count(
            origin, combined, policy.sector_radius_m, policy.sectors
        )
        water_sectors = _sector_count(
            origin, water, policy.sector_radius_m, policy.sectors
        )
        if support_sectors < policy.minimum_support_sectors:
            reason |= HoleReason.INSUFFICIENT_SUPPORT_SECTORS
        if water_sectors < policy.minimum_water_sectors:
            reason |= HoleReason.INSUFFICIENT_WATER_SECTORS
        holes.append(SeededHole(
            seed_id=str(seed_id),
            seed_xy=(float(seed[0]), float(seed[1])),
            label=label_id,
            accepted=reason == HoleReason.NONE,
            reason_mask=int(reason),
            area_m2=float(area),
            centroid_xy=(float(centroid[0]), float(centroid[1])),
            bounds=tuple(float(value) for value in bounds),
            seed_distance_m=float(seed_distance),
            support_sectors=support_sectors,
            water_sectors=water_sectors,
            cell_indices=np.asarray(cells, dtype=np.int64).copy(),
        ))
    return HolePlan(grid=grid, holes=tuple(holes))


def _splitmix64(values: np.ndarray) -> np.ndarray:
    values = np.asarray(values, dtype=np.uint64)
    with np.errstate(over="ignore"):
        values = values + np.uint64(0x9E3779B97F4A7C15)
        values = (values ^ (values >> np.uint64(30))) * np.uint64(0xBF58476D1CE4E5B9)
        values = (values ^ (values >> np.uint64(27))) * np.uint64(0x94D049BB133111EB)
        return values ^ (values >> np.uint64(31))


def sample_accepted_holes(
    plan: HolePlan,
    water_xy,
    terrain_xy,
    *,
    spacing_m: float,
    terrain_clearance_m: float,
    seed: int = 0x5331563131484F4C,
) -> HoleSamples:
    """Create deterministic non-grid XY samples inside accepted components."""

    water = _xy(water_xy, "water_xy")
    terrain = _xy(terrain_xy, "terrain_xy")
    spacing = float(spacing_m)
    clearance = float(terrain_clearance_m)
    if not np.isfinite(spacing) or spacing <= 0.0:
        raise ValueError("spacing_m must be positive and finite")
    if not np.isfinite(clearance) or clearance < 0.0:
        raise ValueError("terrain_clearance_m must be finite and non-negative")
    accepted = set(plan.accepted_labels)
    if not accepted:
        return HoleSamples(np.empty((0, 2), np.float64), 0, 0, 0)

    xmin, xmax, ymin, ymax = plan.grid.bbox
    pitch = spacing / math.sqrt(2.0)
    columns = int(math.ceil((xmax - xmin) / pitch))
    rows = int(math.ceil((ymax - ymin) / pitch))
    # Generate exactly the same globally indexed jitter lattice as the former
    # full-review implementation, but only over each accepted component's
    # bounding rows/columns.  This avoids allocating ~12 million proposals for
    # the 2 mm production review merely to discard almost all of them.
    candidate_parts: list[np.ndarray] = []
    by_label = {
        item.label: item for item in plan.holes
        if item.accepted and item.label in accepted
    }
    for label in sorted(accepted):
        item = by_label[int(label)]
        bx0, bx1, by0, by1 = item.bounds
        col0 = max(0, int(math.floor((bx0 - xmin) / pitch)) - 1)
        col1 = min(columns, int(math.ceil((bx1 - xmin) / pitch)) + 1)
        row0 = max(0, int(math.floor((by0 - ymin) / pitch)) - 1)
        row1 = min(rows, int(math.ceil((by1 - ymin) / pitch)) + 1)
        local_columns = col1 - col0
        local_rows = row1 - row0
        proposal_count = local_columns * local_rows
        if proposal_count > 1_500_000:
            raise RuntimeError(
                f"accepted hole {label} requires {proposal_count:,} proposals"
            )
        col = np.tile(np.arange(col0, col1, dtype=np.int64), local_rows)
        row = np.repeat(np.arange(row0, row1, dtype=np.int64), local_columns)
        serial = row.astype(np.uint64) * np.uint64(columns) + col.astype(np.uint64)
        hx = _splitmix64(serial ^ np.uint64(seed & 0xFFFFFFFFFFFFFFFF))
        hy = _splitmix64(hx ^ np.uint64(0xD1B54A32D192ED03))
        jitter_x = (
            (hx >> np.uint64(11)).astype(np.float64) / float(1 << 53) - 0.5
        ) * pitch
        jitter_y = (
            (hy >> np.uint64(11)).astype(np.float64) / float(1 << 53) - 0.5
        ) * pitch
        x = xmin + (col.astype(np.float64) + 0.5) * pitch + jitter_x
        y = ymin + (row.astype(np.float64) + 0.5) * pitch + jitter_y
        inside_bbox = (x >= xmin) & (x < xmax) & (y >= ymin) & (y < ymax)
        x, y = x[inside_bbox], y[inside_bbox]
        grid_col = np.floor((x - xmin) / plan.grid.cell_m).astype(np.int64)
        grid_row = np.floor((y - ymin) / plan.grid.cell_m).astype(np.int64)
        valid_cell = (
            (grid_col >= 0)
            & (grid_col < len(plan.grid.x_centres))
            & (grid_row >= 0)
            & (grid_row < len(plan.grid.y_centres))
        )
        keep = np.zeros(len(x), bool)
        keep[valid_cell] = (
            plan.grid.labels[grid_row[valid_cell], grid_col[valid_cell]] == label
        )
        if np.any(keep):
            candidate_parts.append(np.column_stack((x[keep], y[keep])))
    candidates = (
        np.concatenate(candidate_parts)
        if candidate_parts else np.empty((0, 2), np.float64)
    )
    source_count = len(candidates)
    if len(candidates) and len(terrain):
        terrain_distance = _nearest_distance(candidates, terrain)
        clear = terrain_distance >= clearance
        terrain_rejected = int(np.count_nonzero(~clear))
        candidates = candidates[clear]
    else:
        terrain_rejected = 0
    if not len(candidates):
        return HoleSamples(np.empty((0, 2), np.float64), source_count, terrain_rejected, 0)
    selection = variable_radius_blue_noise(
        candidates,
        spacing,
        existing_points=water,
        existing_radius=spacing,
        seed=seed,
    )
    selected = candidates[selection.selected_indices]
    return HoleSamples(
        xy=selected,
        source_candidate_count=source_count,
        rejected_for_terrain_clearance=terrain_rejected,
        rejected_for_blue_noise=int(len(candidates) - len(selected)),
    )


__all__ = [
    "HoleGrid",
    "HolePlan",
    "HoleReason",
    "HoleSamples",
    "SeededHole",
    "SeededHolePolicy",
    "build_hole_grid",
    "detect_seeded_holes",
    "sample_accepted_holes",
]
