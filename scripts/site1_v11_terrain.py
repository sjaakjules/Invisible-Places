#!/usr/bin/env python3
"""Candidate-only terrain refinement helpers for Scene1/Fossils v11.

The annotations in ``site1_fossils_v11_review.json`` are deliberately treated
as *review locations*, never as fill masks.  This module detects point-density
deficits inside those locations, predicts each proposed height with independent
surface models, applies hard geometry/noise vetoes, and finally selects an
irregular variable-radius subset.  Geometry that is not supported remains
unchanged because every output is append-only and candidate-only.

Important invariants encoded here:

* accepted SAND/ROCK additions have ``ScanID=10``;
* attributes come only from same-role measured donors with ScanID 0--8;
* scanner-footprint and crack proposals require different confidence tiers;
* residual/frequency energy is veto-only and can never establish a surface;
* CleanMesh is run on a local collar plus additions, with reversible spans;
* canonical Scene1 cloud paths are refused by every writer in this module.

The numerical helpers have NumPy fallbacks so unit tests and small review crops
do not require SciPy.  If SciPy is available, cKDTree and Delaunay accelerate
the same decisions without changing the fail-closed policies.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, IntEnum, IntFlag
import hashlib
import itertools
import json
import math
import os
from pathlib import Path
import re
import sys
from typing import Iterable, Iterator, Mapping, Sequence

import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from site1_v11_confidence import (  # noqa: E402
    BlueNoiseRejection,
    BlueNoiseResult,
    ConfidenceReason,
    ConfidenceResult,
    ConfidenceThresholds,
    ConfidenceTier,
    evaluate_geometry_confidence,
    variable_radius_blue_noise,
)


MEASURED_SCAN_IDS = tuple(range(9))
ADDITION_SCAN_ID = 10
ROLE_TYPE_ID = {"ROCK": 0.0, "SAND": 1.0, "VEG": 2.0}
CANONICAL_TERRAIN_NAME = re.compile(
    r"^Site1-(?:SAND|ROCK|VEG)-(?:1mm|5mm)\.ply$"
)
DEFAULT_SEED = 0x5331563131544552


def _as_xy(
    values: np.ndarray | Sequence[Sequence[float]],
    name: str,
    *,
    allow_empty: bool = True,
) -> np.ndarray:
    result = np.asarray(values, dtype=np.float64)
    if result.ndim != 2 or result.shape[1] < 2:
        raise ValueError(f"{name} must have shape (N, D), D >= 2")
    result = result[:, :2]
    if not allow_empty and not len(result):
        raise ValueError(f"{name} must not be empty")
    if not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be finite")
    return result


def _as_xyz(
    values: np.ndarray | Sequence[Sequence[float]],
    name: str,
    *,
    allow_empty: bool = True,
) -> np.ndarray:
    result = np.asarray(values, dtype=np.float64)
    if result.ndim != 2 or result.shape[1] < 3:
        raise ValueError(f"{name} must have shape (N, D), D >= 3")
    result = result[:, :3]
    if not allow_empty and not len(result):
        raise ValueError(f"{name} must not be empty")
    if not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be finite")
    return result


def _bbox(values: Sequence[float]) -> tuple[float, float, float, float]:
    result = np.asarray(values, dtype=np.float64)
    if result.shape != (4,) or not np.all(np.isfinite(result)):
        raise ValueError("bbox must contain four finite values")
    xmin, xmax, ymin, ymax = (float(value) for value in result)
    if not xmin < xmax or not ymin < ymax:
        raise ValueError("bbox must satisfy xmin < xmax and ymin < ymax")
    return xmin, xmax, ymin, ymax


def _tree_type():
    try:
        from scipy.spatial import cKDTree
    except ModuleNotFoundError:
        return None
    return cKDTree


def _query_nearest(
    query: np.ndarray,
    support: np.ndarray,
    count: int,
) -> tuple[np.ndarray, np.ndarray]:
    count = min(int(count), len(support))
    if count <= 0:
        return (
            np.empty((len(query), 0), np.float64),
            np.empty((len(query), 0), np.int64),
        )
    tree_type = _tree_type()
    if tree_type is not None:
        distance, index = tree_type(support).query(query, k=count, workers=-1)
        if count == 1:
            distance = np.asarray(distance)[:, None]
            index = np.asarray(index)[:, None]
        return np.asarray(distance, np.float64), np.asarray(index, np.int64)

    distance = np.empty((len(query), count), np.float64)
    index = np.empty((len(query), count), np.int64)
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


def _neighbours_within(
    query: np.ndarray,
    support: np.ndarray,
    radius_m: float,
) -> list[np.ndarray]:
    tree_type = _tree_type()
    if tree_type is not None:
        tree = tree_type(support)
        return [
            np.asarray(value, dtype=np.int64)
            for value in tree.query_ball_point(query, radius_m, workers=-1)
        ]
    radius_squared = radius_m * radius_m
    return [
        np.flatnonzero(np.sum(np.square(support - point), axis=1) <= radius_squared)
        for point in query
    ]


# ---------------------------------------------------------------------------
# Review targets and density-deficit detection


class DeficitKind(str, Enum):
    MARKED = "marked"
    SCANNER = "scanner"
    CRACK = "crack"


@dataclass(frozen=True)
class TerrainReviewTarget:
    target_id: str
    kind: DeficitKind
    bbox: tuple[float, float, float, float]
    minimum_tier: ConfidenceTier
    centre_xy: tuple[float, float] | None = None
    search_radius_m: float | None = None


def terrain_targets_from_review_config(
    config_or_path: Mapping | str | Path,
) -> tuple[TerrainReviewTarget, ...]:
    """Extract only written terrain actions from the v11 review config.

    WATER-only interface and obstruction marks are intentionally omitted.  A
    screenshot mark becomes a terrain target only when its written action
    explicitly calls for ScanID 10 or SAND/ROCK terrain reconstruction.
    """

    if isinstance(config_or_path, Mapping):
        config = config_or_path
    else:
        with Path(config_or_path).open("r", encoding="utf-8") as handle:
            config = json.load(handle)
    marked = config.get("marked_locations")
    if not isinstance(marked, Mapping):
        raise ValueError("review config has no marked_locations mapping")

    targets: list[TerrainReviewTarget] = []
    for image_marks in marked.values():
        if not isinstance(image_marks, Sequence):
            continue
        for mark in image_marks:
            if not isinstance(mark, Mapping):
                continue
            action = str(mark.get("written_action", ""))
            action_lower = action.lower()
            terrain_action = (
                "scanid=10" in action_lower
                or "sand/rock interstitial" in action_lower
                or "scanid 10" in action_lower
                or "sand points only" in action_lower
            )
            if not terrain_action or "review_bbox" not in mark:
                continue
            evidence = str(mark.get("screenshot_evidence", "")).lower()
            if "scanner" in evidence:
                kind = DeficitKind.SCANNER
                tier = ConfidenceTier.SUPPORTED
            elif "crack" in evidence or "crevasse" in evidence:
                kind = DeficitKind.CRACK
                tier = ConfidenceTier.STRONG
            else:
                kind = DeficitKind.MARKED
                tier = ConfidenceTier.SUPPORTED
            centre = mark.get("world")
            centre_xy = None
            if isinstance(centre, Sequence) and len(centre) >= 2:
                centre_xy = (float(centre[0]), float(centre[1]))
            radius = mark.get("review_circle_radius_m")
            targets.append(
                TerrainReviewTarget(
                    target_id=str(mark.get("id", f"terrain_{len(targets)}")),
                    kind=kind,
                    bbox=_bbox(mark["review_bbox"]),
                    minimum_tier=tier,
                    centre_xy=centre_xy,
                    search_radius_m=float(radius) if radius is not None else None,
                )
            )
    return tuple(targets)


@dataclass(frozen=True)
class DensityDeficitComponent:
    component_id: int
    cell_count: int
    area_m2: float
    centre_xy: tuple[float, float]
    bbox: tuple[float, float, float, float]
    observed_points: int
    expected_points: float
    missing_points: float
    mean_deficit_fraction: float
    aspect_ratio: float


@dataclass(frozen=True)
class DensityDeficitResult:
    kind: DeficitKind
    bbox: tuple[float, float, float, float]
    cell_size_m: float
    x_centres: np.ndarray
    y_centres: np.ndarray
    observed_count: np.ndarray
    expected_count: np.ndarray
    deficit_fraction: np.ndarray
    candidate_mask: np.ndarray
    component_labels: np.ndarray
    components: tuple[DensityDeficitComponent, ...]

    @property
    def candidate_centres_xy(self) -> np.ndarray:
        iy, ix = np.nonzero(self.candidate_mask)
        if not len(ix):
            return np.empty((0, 2), np.float64)
        return np.column_stack((self.x_centres[ix], self.y_centres[iy]))


def _window_sum(values: np.ndarray, radius: int) -> tuple[np.ndarray, np.ndarray]:
    padded = np.pad(values, radius, mode="constant")
    valid = np.pad(np.ones(values.shape, np.float64), radius, mode="constant")

    def summed(array: np.ndarray) -> np.ndarray:
        integral = np.pad(array, ((1, 0), (1, 0)), mode="constant").cumsum(0).cumsum(1)
        width = 2 * radius + 1
        return (
            integral[width:, width:]
            - integral[:-width, width:]
            - integral[width:, :-width]
            + integral[:-width, :-width]
        )

    return summed(padded), summed(valid)


def _label_components(mask: np.ndarray) -> tuple[np.ndarray, int]:
    labels = np.zeros(mask.shape, np.int32)
    next_label = 0
    height, width = mask.shape
    for y, x in zip(*np.nonzero(mask), strict=True):
        if labels[y, x]:
            continue
        next_label += 1
        labels[y, x] = next_label
        stack = [(int(y), int(x))]
        while stack:
            cy, cx = stack.pop()
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    if not (dx or dy):
                        continue
                    ny, nx = cy + dy, cx + dx
                    if (
                        0 <= ny < height
                        and 0 <= nx < width
                        and mask[ny, nx]
                        and labels[ny, nx] == 0
                    ):
                        labels[ny, nx] = next_label
                        stack.append((ny, nx))
    return labels, next_label


def _component_aspect(x: np.ndarray, y: np.ndarray, cell_size: float) -> float:
    if len(x) < 2:
        return 1.0
    centred = np.column_stack((x - np.mean(x), y - np.mean(y)))
    covariance = centred.T @ centred / max(len(centred) - 1, 1)
    eigenvalues = np.linalg.eigvalsh(covariance)
    minimum = max(float(eigenvalues[0]), (cell_size * 0.25) ** 2)
    return math.sqrt(max(float(eigenvalues[-1]), minimum) / minimum)


def detect_density_deficits(
    measured_xy: np.ndarray | Sequence[Sequence[float]],
    *,
    bbox: Sequence[float],
    kind: DeficitKind | str = DeficitKind.MARKED,
    cell_size_m: float = 0.020,
    neighbourhood_radius_cells: int = 3,
    minimum_expected_points: float = 3.0,
    minimum_deficit_fraction: float = 0.45,
    minimum_component_cells: int = 3,
    minimum_aspect_ratio: float = 1.0,
    circular_search: tuple[Sequence[float], float] | None = None,
) -> DensityDeficitResult:
    """Detect connected local deficits without turning cells into fill quotas.

    Cells only identify geometry requiring review.  The final point set is
    sampled continuously by :func:`select_variable_radius_interstitials`.
    For scanner footprints, ``circular_search`` limits the search envelope but
    the detected connected boundary determines the actual component.
    """

    points = _as_xy(measured_xy, "measured_xy")
    bounds = _bbox(bbox)
    kind = DeficitKind(kind)
    if not np.isfinite(cell_size_m) or cell_size_m <= 0.0:
        raise ValueError("cell_size_m must be positive and finite")
    if (
        not isinstance(neighbourhood_radius_cells, (int, np.integer))
        or neighbourhood_radius_cells < 1
    ):
        raise ValueError("neighbourhood_radius_cells must be a positive integer")
    if not np.isfinite(minimum_expected_points) or minimum_expected_points < 0.0:
        raise ValueError("minimum_expected_points must be finite and non-negative")
    if not np.isfinite(minimum_deficit_fraction) or not 0.0 <= minimum_deficit_fraction <= 1.0:
        raise ValueError("minimum_deficit_fraction must lie in [0, 1]")
    if not isinstance(minimum_component_cells, (int, np.integer)) or minimum_component_cells < 1:
        raise ValueError("minimum_component_cells must be a positive integer")
    if not np.isfinite(minimum_aspect_ratio) or minimum_aspect_ratio < 1.0:
        raise ValueError("minimum_aspect_ratio must be finite and >= 1")

    xmin, xmax, ymin, ymax = bounds
    width = max(1, int(math.ceil((xmax - xmin) / cell_size_m)))
    height = max(1, int(math.ceil((ymax - ymin) / cell_size_m)))
    x_centres = xmin + (np.arange(width) + 0.5) * cell_size_m
    y_centres = ymin + (np.arange(height) + 0.5) * cell_size_m
    inside = (
        (points[:, 0] >= xmin)
        & (points[:, 0] < xmax)
        & (points[:, 1] >= ymin)
        & (points[:, 1] < ymax)
    )
    selected = points[inside]
    if len(selected):
        ix = np.floor((selected[:, 0] - xmin) / cell_size_m).astype(np.int64)
        iy = np.floor((selected[:, 1] - ymin) / cell_size_m).astype(np.int64)
        flat = np.clip(iy, 0, height - 1) * width + np.clip(ix, 0, width - 1)
        observed = np.bincount(flat, minlength=height * width).reshape(height, width)
    else:
        observed = np.zeros((height, width), np.int64)

    local_sum, local_cells = _window_sum(
        observed.astype(np.float64), int(neighbourhood_radius_cells)
    )
    denominator = np.maximum(local_cells - 1.0, 1.0)
    expected = np.maximum((local_sum - observed) / denominator, 0.0)

    valid_review = np.ones(observed.shape, bool)
    if circular_search is not None:
        centre, radius = circular_search
        centre = np.asarray(centre, np.float64)
        if centre.shape != (2,) or not np.all(np.isfinite(centre)):
            raise ValueError("circular_search centre must contain two finite values")
        if not np.isfinite(radius) or radius <= 0.0:
            raise ValueError("circular_search radius must be positive and finite")
        gx, gy = np.meshgrid(x_centres, y_centres)
        valid_review &= np.square(gx - centre[0]) + np.square(gy - centre[1]) <= radius * radius

    fraction = np.zeros(observed.shape, np.float64)
    has_expected = expected > 0.0
    fraction[has_expected] = np.clip(
        (expected[has_expected] - observed[has_expected]) / expected[has_expected],
        0.0,
        1.0,
    )
    raw = (
        valid_review
        & (expected >= minimum_expected_points)
        & (fraction >= minimum_deficit_fraction)
    )
    labels, count = _label_components(raw)
    keep = np.zeros_like(raw)
    components: list[DensityDeficitComponent] = []
    next_id = 0
    kept_labels = np.zeros_like(labels)
    for label in range(1, count + 1):
        component = labels == label
        iy, ix = np.nonzero(component)
        if len(ix) < minimum_component_cells:
            continue
        cx = x_centres[ix]
        cy = y_centres[iy]
        aspect = _component_aspect(cx, cy, cell_size_m)
        if aspect < minimum_aspect_ratio:
            continue
        if kind is DeficitKind.SCANNER and circular_search is not None:
            centre = np.asarray(circular_search[0], np.float64)
            nearest = np.min(np.hypot(cx - centre[0], cy - centre[1]))
            # A scanner component must approach its registered centre, but the
            # radius and boundary come from measured density, not the markup.
            if nearest > max(2.5 * cell_size_m, 0.20 * circular_search[1]):
                continue
        next_id += 1
        keep |= component
        kept_labels[component] = next_id
        observed_points = int(np.sum(observed[component]))
        expected_points = float(np.sum(expected[component]))
        components.append(
            DensityDeficitComponent(
                component_id=next_id,
                cell_count=len(ix),
                area_m2=len(ix) * cell_size_m * cell_size_m,
                centre_xy=(float(np.mean(cx)), float(np.mean(cy))),
                bbox=(
                    float(np.min(cx) - cell_size_m / 2),
                    float(np.max(cx) + cell_size_m / 2),
                    float(np.min(cy) - cell_size_m / 2),
                    float(np.max(cy) + cell_size_m / 2),
                ),
                observed_points=observed_points,
                expected_points=expected_points,
                missing_points=max(0.0, expected_points - observed_points),
                mean_deficit_fraction=float(np.mean(fraction[component])),
                aspect_ratio=aspect,
            )
        )
    return DensityDeficitResult(
        kind=kind,
        bbox=bounds,
        cell_size_m=float(cell_size_m),
        x_centres=x_centres,
        y_centres=y_centres,
        observed_count=observed,
        expected_count=expected,
        deficit_fraction=fraction,
        candidate_mask=keep,
        component_labels=kept_labels,
        components=tuple(components),
    )


def detect_marked_density_deficits(
    measured_xy: np.ndarray | Sequence[Sequence[float]],
    targets: Iterable[TerrainReviewTarget],
    **kwargs,
) -> dict[str, DensityDeficitResult]:
    """Detect actual deficits for marked (non-scanner/non-crack) targets."""

    return {
        target.target_id: detect_density_deficits(
            measured_xy, bbox=target.bbox, kind=DeficitKind.MARKED, **kwargs
        )
        for target in targets
        if target.kind is DeficitKind.MARKED
    }


def detect_scanner_footprint_deficit(
    measured_xy: np.ndarray | Sequence[Sequence[float]],
    target: TerrainReviewTarget,
    **kwargs,
) -> DensityDeficitResult:
    if target.kind is not DeficitKind.SCANNER:
        raise ValueError("target is not a scanner review target")
    if target.centre_xy is None or target.search_radius_m is None:
        raise ValueError("scanner target needs centre_xy and search_radius_m")
    return detect_density_deficits(
        measured_xy,
        bbox=target.bbox,
        kind=DeficitKind.SCANNER,
        circular_search=(target.centre_xy, target.search_radius_m),
        **kwargs,
    )


def detect_crack_density_deficit(
    measured_xy: np.ndarray | Sequence[Sequence[float]],
    target: TerrainReviewTarget,
    **kwargs,
) -> DensityDeficitResult:
    if target.kind is not DeficitKind.CRACK:
        raise ValueError("target is not a crack review target")
    defaults = {
        "minimum_deficit_fraction": 0.60,
        "minimum_component_cells": 3,
        "minimum_aspect_ratio": 1.8,
    }
    defaults.update(kwargs)
    return detect_density_deficits(
        measured_xy, bbox=target.bbox, kind=DeficitKind.CRACK, **defaults
    )


# ---------------------------------------------------------------------------
# Independent geometry evidence


@dataclass(frozen=True)
class SectorSupportResult:
    counts: np.ndarray
    occupied: np.ndarray
    occupied_sector_count: np.ndarray
    donor_count: np.ndarray
    nearest_distance_m: np.ndarray


def compute_eight_sector_support(
    query_xy: np.ndarray | Sequence[Sequence[float]],
    donor_xy: np.ndarray | Sequence[Sequence[float]],
    *,
    inner_radius_m: float = 0.0,
    outer_radius_m: float = 0.040,
    minimum_points_per_sector: int = 1,
) -> SectorSupportResult:
    """Count donors in eight fixed angular sectors around every query point."""

    query = _as_xy(query_xy, "query_xy")
    donors = _as_xy(donor_xy, "donor_xy", allow_empty=False)
    if not np.isfinite(inner_radius_m) or inner_radius_m < 0.0:
        raise ValueError("inner_radius_m must be finite and non-negative")
    if not np.isfinite(outer_radius_m) or outer_radius_m <= inner_radius_m:
        raise ValueError("outer_radius_m must exceed inner_radius_m")
    if (
        not isinstance(minimum_points_per_sector, (int, np.integer))
        or minimum_points_per_sector < 1
    ):
        raise ValueError("minimum_points_per_sector must be a positive integer")
    neighbourhoods = _neighbours_within(query, donors, outer_radius_m)
    counts = np.zeros((len(query), 8), np.int32)
    nearest = np.full(len(query), np.nan, np.float64)
    donor_count = np.zeros(len(query), np.int32)
    for row, indices in enumerate(neighbourhoods):
        if not len(indices):
            continue
        delta = donors[indices] - query[row]
        radius = np.linalg.norm(delta, axis=1)
        keep = radius >= inner_radius_m
        delta = delta[keep]
        radius = radius[keep]
        if not len(radius):
            continue
        angle = np.mod(np.arctan2(delta[:, 1], delta[:, 0]) + 2.0 * np.pi, 2.0 * np.pi)
        sector = np.minimum((angle / (np.pi / 4.0)).astype(np.int64), 7)
        counts[row] = np.bincount(sector, minlength=8)
        donor_count[row] = len(radius)
        nearest[row] = float(np.min(radius))
    occupied = counts >= int(minimum_points_per_sector)
    return SectorSupportResult(
        counts=counts,
        occupied=occupied,
        occupied_sector_count=np.sum(occupied, axis=1).astype(np.uint8),
        donor_count=donor_count,
        nearest_distance_m=nearest,
    )


def _surface_design(dx: np.ndarray, dy: np.ndarray, degree: int) -> np.ndarray:
    if degree == 1:
        return np.column_stack((np.ones(len(dx)), dx, dy))
    if degree == 2:
        return np.column_stack((np.ones(len(dx)), dx, dy, dx * dx, dx * dy, dy * dy))
    raise ValueError("degree must be 1 or 2")


def _fit_robust_local_surface(
    query_xy: np.ndarray,
    donor_xyz: np.ndarray,
    *,
    bandwidth_m: float,
    degree: int,
    iterations: int,
) -> tuple[np.ndarray, float, float] | None:
    delta = donor_xyz[:, :2] - query_xy
    radius = np.linalg.norm(delta, axis=1)
    design = _surface_design(delta[:, 0], delta[:, 1], degree)
    required = design.shape[1]
    if len(donor_xyz) < required:
        return None
    radial = np.exp(-4.0 * np.square(radius / max(bandwidth_m, 1.0e-12)))
    weights = radial.copy()
    coefficients = None
    for _ in range(max(1, int(iterations))):
        weighted = design * np.sqrt(weights)[:, None]
        target = donor_xyz[:, 2] * np.sqrt(weights)
        try:
            coefficients, _, rank, _ = np.linalg.lstsq(weighted, target, rcond=None)
        except np.linalg.LinAlgError:
            return None
        if rank < required:
            return None
        residual = donor_xyz[:, 2] - design @ coefficients
        centre = np.median(residual)
        mad = 1.4826 * np.median(np.abs(residual - centre))
        if mad <= 1.0e-12:
            break
        scaled = np.abs(residual - centre) / (1.5 * mad)
        robust = np.ones_like(scaled)
        outside = scaled > 1.0
        robust[outside] = 1.0 / scaled[outside]
        weights = radial * robust
    assert coefficients is not None
    residual = donor_xyz[:, 2] - design @ coefficients
    rms = math.sqrt(float(np.average(np.square(residual), weights=np.maximum(weights, 1e-15))))
    mad = 1.4826 * float(np.median(np.abs(residual - np.median(residual))))
    return coefficients, rms, mad


@dataclass(frozen=True)
class RobustSurfaceResult:
    height_m: np.ndarray
    normal: np.ndarray
    residual_rms_m: np.ndarray
    residual_mad_m: np.ndarray
    donor_count: np.ndarray
    coefficients: np.ndarray
    valid: np.ndarray


def predict_robust_quadratic_mls(
    query_xy: np.ndarray | Sequence[Sequence[float]],
    donor_xyz: np.ndarray | Sequence[Sequence[float]],
    *,
    bandwidth_m: float = 0.040,
    maximum_donors: int = 96,
    minimum_donors: int = 12,
    iterations: int = 4,
    degree: int = 2,
) -> RobustSurfaceResult:
    """Predict a robust local quadratic/MLS height and analytic normal."""

    query = _as_xy(query_xy, "query_xy")
    donors = _as_xyz(donor_xyz, "donor_xyz", allow_empty=False)
    if not np.isfinite(bandwidth_m) or bandwidth_m <= 0.0:
        raise ValueError("bandwidth_m must be positive and finite")
    if minimum_donors < 3 or maximum_donors < minimum_donors:
        raise ValueError("donor limits must satisfy 3 <= minimum <= maximum")
    parameter_count = 3 if degree == 1 else 6 if degree == 2 else 0
    if not parameter_count:
        raise ValueError("degree must be 1 or 2")
    minimum_donors = max(int(minimum_donors), parameter_count)
    neighbourhoods = _neighbours_within(query, donors[:, :2], bandwidth_m)
    width = parameter_count
    coefficients = np.full((len(query), width), np.nan, np.float64)
    height = np.full(len(query), np.nan, np.float64)
    normal = np.full((len(query), 3), np.nan, np.float64)
    rms = np.full(len(query), np.nan, np.float64)
    mad = np.full(len(query), np.nan, np.float64)
    count = np.zeros(len(query), np.int32)
    for row, indices in enumerate(neighbourhoods):
        if len(indices) > maximum_donors:
            distance = np.linalg.norm(donors[indices, :2] - query[row], axis=1)
            indices = indices[np.argsort(distance, kind="stable")[:maximum_donors]]
        count[row] = len(indices)
        if len(indices) < minimum_donors:
            continue
        fitted = _fit_robust_local_surface(
            query[row], donors[indices], bandwidth_m=bandwidth_m,
            degree=degree, iterations=iterations,
        )
        if fitted is None:
            continue
        coef, row_rms, row_mad = fitted
        coefficients[row] = coef
        height[row] = coef[0]
        vector = np.array([-coef[1], -coef[2], 1.0], np.float64)
        normal[row] = vector / np.linalg.norm(vector)
        rms[row], mad[row] = row_rms, row_mad
    valid = np.isfinite(height) & np.all(np.isfinite(normal), axis=1)
    return RobustSurfaceResult(height, normal, rms, mad, count, coefficients, valid)


@dataclass(frozen=True)
class SurfacePredictionResult:
    height_m: np.ndarray
    normal: np.ndarray
    donor_count: np.ndarray
    valid: np.ndarray
    method: str


def _triangle_barycentric(
    point: np.ndarray, triangle: np.ndarray
) -> np.ndarray | None:
    matrix = np.column_stack((triangle[0] - triangle[2], triangle[1] - triangle[2]))
    determinant = float(np.linalg.det(matrix))
    if abs(determinant) <= 1.0e-14:
        return None
    first = np.linalg.solve(matrix, point - triangle[2])
    return np.array([first[0], first[1], 1.0 - first.sum()], np.float64)


def _circumcircle(triangle: np.ndarray) -> tuple[np.ndarray, float] | None:
    a = 2.0 * (triangle[1:] - triangle[0])
    b = np.sum(np.square(triangle[1:]), axis=1) - np.sum(np.square(triangle[0]))
    try:
        centre = np.linalg.solve(a, b)
    except np.linalg.LinAlgError:
        return None
    return centre, float(np.linalg.norm(centre - triangle[0]))


def _numpy_linear_triangle(
    point: np.ndarray,
    donors: np.ndarray,
    maximum_donors: int,
) -> tuple[float, np.ndarray, int] | None:
    distance = np.linalg.norm(donors[:, :2] - point, axis=1)
    indices = np.argsort(distance, kind="stable")[: min(maximum_donors, len(donors))]
    local = donors[indices]
    best = None
    best_score = np.inf
    for triple in itertools.combinations(range(len(local)), 3):
        triangle = local[np.asarray(triple), :2]
        weights = _triangle_barycentric(point, triangle)
        if weights is None or np.min(weights) < -1.0e-10:
            continue
        circle = _circumcircle(triangle)
        if circle is None:
            continue
        centre, radius = circle
        other_distance = np.linalg.norm(local[:, :2] - centre, axis=1)
        triple_mask = np.ones(len(local), bool)
        triple_mask[list(triple)] = False
        delaunay = not np.any(other_distance[triple_mask] < radius - 1.0e-10)
        edge = max(
            np.linalg.norm(triangle[0] - triangle[1]),
            np.linalg.norm(triangle[1] - triangle[2]),
            np.linalg.norm(triangle[2] - triangle[0]),
        )
        score = radius + edge + (0.0 if delaunay else 1.0e6)
        if score >= best_score:
            continue
        selected = local[np.asarray(triple)]
        z = float(weights @ selected[:, 2])
        edge_a = selected[1] - selected[0]
        edge_b = selected[2] - selected[0]
        normal = np.cross(edge_a, edge_b)
        length = np.linalg.norm(normal)
        if length <= 1.0e-12:
            continue
        normal /= length
        if normal[2] < 0.0:
            normal *= -1.0
        best = (z, normal, len(local))
        best_score = score
    return best


def predict_delaunay_linear_surface(
    query_xy: np.ndarray | Sequence[Sequence[float]],
    donor_xyz: np.ndarray | Sequence[Sequence[float]],
    *,
    maximum_fallback_donors: int = 18,
) -> SurfacePredictionResult:
    """Piecewise-linear Delaunay prediction, with a local NumPy fallback."""

    query = _as_xy(query_xy, "query_xy")
    donors = _as_xyz(donor_xyz, "donor_xyz", allow_empty=False)
    # Collapse duplicate XY samples using their median height.  Vertical
    # multiplicity is assessed separately and must not make triangulation
    # order-dependent.
    order = np.lexsort((donors[:, 1], donors[:, 0]))
    sorted_xy = donors[order, :2]
    unique_xy, first, counts = np.unique(
        sorted_xy, axis=0, return_index=True, return_counts=True
    )
    unique_z = np.array(
        [
            np.median(donors[order[start : start + count], 2])
            for start, count in zip(first, counts, strict=True)
        ],
        np.float64,
    )
    unique = np.column_stack((unique_xy, unique_z))
    height = np.full(len(query), np.nan, np.float64)
    normal = np.full((len(query), 3), np.nan, np.float64)
    donor_count = np.zeros(len(query), np.int32)
    method = "numpy-local-empty-circumcircle"
    try:
        from scipy.spatial import Delaunay
    except ModuleNotFoundError:
        Delaunay = None
    if Delaunay is not None and len(unique) >= 3:
        try:
            triangulation = Delaunay(unique[:, :2])
            simplices = triangulation.find_simplex(query)
            for row, simplex in enumerate(simplices):
                if simplex < 0:
                    continue
                transform = triangulation.transform[simplex]
                first_two = transform[:2] @ (query[row] - transform[2])
                weights = np.r_[first_two, 1.0 - first_two.sum()]
                vertices = triangulation.simplices[simplex]
                selected = unique[vertices]
                height[row] = weights @ selected[:, 2]
                vector = np.cross(selected[1] - selected[0], selected[2] - selected[0])
                if vector[2] < 0.0:
                    vector *= -1.0
                normal[row] = vector / np.linalg.norm(vector)
                donor_count[row] = 3
            method = "scipy-delaunay-linear"
        except (ValueError, np.linalg.LinAlgError):
            pass
    else:
        for row, point in enumerate(query):
            fitted = _numpy_linear_triangle(point, unique, maximum_fallback_donors)
            if fitted is None:
                continue
            height[row], normal[row], donor_count[row] = fitted
    valid = np.isfinite(height) & np.all(np.isfinite(normal), axis=1)
    return SurfacePredictionResult(height, normal, donor_count, valid, method)


def predict_lower_boundary_surface(
    query_xy: np.ndarray | Sequence[Sequence[float]],
    donor_xyz: np.ndarray | Sequence[Sequence[float]],
    *,
    inner_radius_m: float = 0.010,
    outer_radius_m: float = 0.055,
    quantile: float = 0.25,
    minimum_sectors: int = 6,
) -> SurfacePredictionResult:
    """Fit a plane through per-sector lower-envelope boundary samples.

    This is intentionally independent of the MLS and triangulated surfaces.
    Reflections below a real surface can bias the envelope, which is useful:
    disagreement and multimodality then veto the proposal instead of quietly
    manufacturing a plausible surface.
    """

    query = _as_xy(query_xy, "query_xy")
    donors = _as_xyz(donor_xyz, "donor_xyz", allow_empty=False)
    if not 0.0 <= quantile <= 0.5:
        raise ValueError("quantile must lie in [0, 0.5]")
    if not 1 <= minimum_sectors <= 8:
        raise ValueError("minimum_sectors must lie in [1, 8]")
    if not 0.0 <= inner_radius_m < outer_radius_m:
        raise ValueError("radii must satisfy 0 <= inner < outer")
    neighbourhoods = _neighbours_within(query, donors[:, :2], outer_radius_m)
    height = np.full(len(query), np.nan, np.float64)
    normal = np.full((len(query), 3), np.nan, np.float64)
    donor_count = np.zeros(len(query), np.int32)
    for row, indices in enumerate(neighbourhoods):
        if not len(indices):
            continue
        selected = donors[indices]
        delta = selected[:, :2] - query[row]
        radius = np.linalg.norm(delta, axis=1)
        selected = selected[radius >= inner_radius_m]
        delta = delta[radius >= inner_radius_m]
        if not len(selected):
            continue
        angle = np.mod(np.arctan2(delta[:, 1], delta[:, 0]) + 2 * np.pi, 2 * np.pi)
        sector = np.minimum((angle / (np.pi / 4)).astype(np.int64), 7)
        representatives: list[np.ndarray] = []
        for sector_id in range(8):
            subset = selected[sector == sector_id]
            if not len(subset):
                continue
            target_z = float(np.quantile(subset[:, 2], quantile))
            representative = subset[np.argmin(np.abs(subset[:, 2] - target_z))]
            representatives.append(representative)
        if len(representatives) < minimum_sectors:
            continue
        boundary = np.asarray(representatives)
        design = _surface_design(
            boundary[:, 0] - query[row, 0],
            boundary[:, 1] - query[row, 1],
            1,
        )
        try:
            coefficients, _, rank, _ = np.linalg.lstsq(design, boundary[:, 2], rcond=None)
        except np.linalg.LinAlgError:
            continue
        if rank < 3:
            continue
        height[row] = coefficients[0]
        vector = np.array([-coefficients[1], -coefficients[2], 1.0])
        normal[row] = vector / np.linalg.norm(vector)
        donor_count[row] = len(boundary)
    valid = np.isfinite(height) & np.all(np.isfinite(normal), axis=1)
    return SurfacePredictionResult(
        height, normal, donor_count, valid, "sector-lower-boundary-plane"
    )


@dataclass(frozen=True)
class NormalCoherenceResult:
    coherence: np.ndarray
    donor_count: np.ndarray
    consensus_normal: np.ndarray


def compute_local_normal_coherence(
    query_xy: np.ndarray | Sequence[Sequence[float]],
    donor_xy: np.ndarray | Sequence[Sequence[float]],
    donor_normals: np.ndarray | Sequence[Sequence[float]],
    *,
    radius_m: float = 0.040,
    minimum_donors: int = 6,
) -> NormalCoherenceResult:
    query = _as_xy(query_xy, "query_xy")
    xy = _as_xy(donor_xy, "donor_xy", allow_empty=False)
    normals = np.asarray(donor_normals, np.float64)
    if normals.shape != (len(xy), 3):
        raise ValueError("donor_normals must have shape (N, 3)")
    finite = np.all(np.isfinite(normals), axis=1)
    length = np.linalg.norm(normals, axis=1)
    finite &= length > 1.0e-12
    normalized = np.zeros_like(normals)
    normalized[finite] = normals[finite] / length[finite, None]
    # Orient normals onto a common upper hemisphere before measuring their
    # directional concentration.  This handles equivalent flipped normals.
    flip = normalized[:, 2] < 0.0
    normalized[flip] *= -1.0
    neighbourhoods = _neighbours_within(query, xy, radius_m)
    coherence = np.full(len(query), np.nan, np.float64)
    consensus = np.full((len(query), 3), np.nan, np.float64)
    count = np.zeros(len(query), np.int32)
    for row, indices in enumerate(neighbourhoods):
        indices = indices[finite[indices]]
        count[row] = len(indices)
        if len(indices) < minimum_donors:
            continue
        covariance = normalized[indices].T @ normalized[indices]
        values, vectors = np.linalg.eigh(covariance)
        direction = vectors[:, np.argmax(values)]
        if direction[2] < 0.0:
            direction *= -1.0
        consensus[row] = direction
        coherence[row] = float(np.mean(np.abs(normalized[indices] @ direction)))
    return NormalCoherenceResult(coherence, count, consensus)


def _multimodality_score(residual: np.ndarray) -> float:
    residual = np.sort(np.asarray(residual, np.float64))
    if len(residual) < 8:
        return float("nan")
    minimum_side = max(2, int(math.ceil(0.20 * len(residual))))
    gaps = np.diff(residual)
    valid = np.arange(1, len(residual))
    valid = (valid >= minimum_side) & (valid <= len(residual) - minimum_side)
    if not np.any(valid):
        return 0.0
    internal = gaps[valid]
    largest = float(np.max(internal))
    typical = float(np.median(np.abs(gaps)))
    robust_range = float(np.quantile(residual, 0.90) - np.quantile(residual, 0.10))
    if robust_range <= 1.0e-12:
        return 0.0
    excess = max(0.0, largest - 3.0 * typical)
    split = int(np.flatnonzero(valid)[np.argmax(internal)] + 1)
    balance = 2.0 * min(split, len(residual) - split) / len(residual)
    return float(np.clip(balance * excess / robust_range * 2.0, 0.0, 1.0))


@dataclass(frozen=True)
class VerticalDistributionResult:
    thickness_m: np.ndarray
    multimodality_score: np.ndarray
    donor_count: np.ndarray


def compute_vertical_distribution_metrics(
    query_xy: np.ndarray | Sequence[Sequence[float]],
    donor_xyz: np.ndarray | Sequence[Sequence[float]],
    *,
    radius_m: float = 0.035,
    minimum_donors: int = 8,
) -> VerticalDistributionResult:
    """Measure detrended vertical thickness and separated residual modes."""

    query = _as_xy(query_xy, "query_xy")
    donors = _as_xyz(donor_xyz, "donor_xyz", allow_empty=False)
    neighbourhoods = _neighbours_within(query, donors[:, :2], radius_m)
    thickness = np.full(len(query), np.nan, np.float64)
    score = np.full(len(query), np.nan, np.float64)
    count = np.zeros(len(query), np.int32)
    for row, indices in enumerate(neighbourhoods):
        count[row] = len(indices)
        if len(indices) < minimum_donors:
            continue
        selected = donors[indices]
        fitted = _fit_robust_local_surface(
            query[row], selected, bandwidth_m=radius_m, degree=1, iterations=4
        )
        if fitted is None:
            continue
        coefficients = fitted[0]
        delta = selected[:, :2] - query[row]
        residual = selected[:, 2] - _surface_design(delta[:, 0], delta[:, 1], 1) @ coefficients
        thickness[row] = float(np.quantile(residual, 0.90) - np.quantile(residual, 0.10))
        score[row] = _multimodality_score(residual)
    return VerticalDistributionResult(thickness, score, count)


@dataclass(frozen=True)
class ResidualEnergyResult:
    energy_m2: np.ndarray
    reference_energy_m2: np.ndarray
    ratio: np.ndarray
    donor_count: np.ndarray


def residual_energy_ratio(
    energy_m2: np.ndarray | Sequence[float],
    reference_energy_m2: np.ndarray | Sequence[float] | float,
) -> np.ndarray:
    energy = np.asarray(energy_m2, np.float64)
    reference = np.asarray(reference_energy_m2, np.float64)
    if reference.ndim == 0:
        reference = np.full(energy.shape, float(reference), np.float64)
    if reference.shape != energy.shape:
        raise ValueError("reference_energy_m2 must be scalar or match energy_m2")
    result = np.full(energy.shape, np.nan, np.float64)
    valid = np.isfinite(energy) & np.isfinite(reference) & (energy >= 0.0) & (reference >= 0.0)
    positive = valid & (reference > 0.0)
    result[positive] = energy[positive] / reference[positive]
    result[valid & (reference == 0.0) & (energy == 0.0)] = 0.0
    result[valid & (reference == 0.0) & (energy > 0.0)] = np.inf
    return result


def compute_local_residual_energy(
    query_xy: np.ndarray | Sequence[Sequence[float]],
    donor_xyz: np.ndarray | Sequence[Sequence[float]],
    reference_energy_m2: np.ndarray | Sequence[float] | float,
    *,
    radius_m: float = 0.040,
    minimum_donors: int = 10,
) -> ResidualEnergyResult:
    """Compute high-frequency graph residual energy relative to measured terrain.

    A quiet residual does not increase confidence; callers pass only ``ratio``
    to the veto-only gate in :func:`evaluate_geometry_confidence`.
    """

    query = _as_xy(query_xy, "query_xy")
    donors = _as_xyz(donor_xyz, "donor_xyz", allow_empty=False)
    reference = np.asarray(reference_energy_m2, np.float64)
    if reference.ndim == 0:
        reference = np.full(len(query), float(reference), np.float64)
    if reference.shape != (len(query),):
        raise ValueError("reference_energy_m2 must be scalar or contain one value per query")
    neighbourhoods = _neighbours_within(query, donors[:, :2], radius_m)
    energy = np.full(len(query), np.nan, np.float64)
    count = np.zeros(len(query), np.int32)
    for row, indices in enumerate(neighbourhoods):
        count[row] = len(indices)
        if len(indices) < minimum_donors:
            continue
        selected = donors[indices]
        fitted = _fit_robust_local_surface(
            query[row], selected, bandwidth_m=radius_m, degree=2, iterations=4
        )
        if fitted is None:
            continue
        coefficients = fitted[0]
        delta = selected[:, :2] - query[row]
        residual = selected[:, 2] - _surface_design(delta[:, 0], delta[:, 1], 2) @ coefficients
        pairwise = np.sum(
            np.square(selected[:, None, :2] - selected[None, :, :2]), axis=2
        )
        np.fill_diagonal(pairwise, np.inf)
        nearest = np.argmin(pairwise, axis=1)
        differences = residual - residual[nearest]
        energy[row] = float(np.median(np.square(differences)))
    return ResidualEnergyResult(
        energy_m2=energy,
        reference_energy_m2=reference,
        ratio=residual_energy_ratio(energy, reference),
        donor_count=count,
    )


@dataclass(frozen=True)
class TerrainGeometryEvaluation:
    mls: RobustSurfaceResult
    linear: SurfacePredictionResult
    boundary: SurfacePredictionResult
    sectors: SectorSupportResult
    normals: NormalCoherenceResult
    vertical: VerticalDistributionResult
    residual_energy: ResidualEnergyResult
    confidence: ConfidenceResult

    @property
    def surface_heights_m(self) -> np.ndarray:
        return np.column_stack(
            (self.mls.height_m, self.linear.height_m, self.boundary.height_m)
        )


def evaluate_terrain_geometry(
    query_xy: np.ndarray | Sequence[Sequence[float]],
    donor_xyz: np.ndarray | Sequence[Sequence[float]],
    donor_normals: np.ndarray | Sequence[Sequence[float]],
    reference_energy_m2: np.ndarray | Sequence[float] | float,
    *,
    thresholds: ConfidenceThresholds = ConfidenceThresholds(),
    support_radius_m: float = 0.040,
) -> TerrainGeometryEvaluation:
    """Evaluate all independent evidence and the shared hard confidence gates."""

    query = _as_xy(query_xy, "query_xy")
    donors = _as_xyz(donor_xyz, "donor_xyz", allow_empty=False)
    mls = predict_robust_quadratic_mls(
        query, donors, bandwidth_m=support_radius_m
    )
    linear = predict_delaunay_linear_surface(query, donors)
    boundary = predict_lower_boundary_surface(
        query,
        donors,
        outer_radius_m=max(support_radius_m, 0.041),
        inner_radius_m=min(0.010, support_radius_m * 0.25),
    )
    sectors = compute_eight_sector_support(
        query, donors[:, :2], outer_radius_m=support_radius_m
    )
    normals = compute_local_normal_coherence(
        query, donors[:, :2], donor_normals, radius_m=support_radius_m
    )
    vertical = compute_vertical_distribution_metrics(
        query, donors, radius_m=min(support_radius_m, 0.035)
    )
    energy = compute_local_residual_energy(
        query, donors, reference_energy_m2, radius_m=support_radius_m
    )
    confidence = evaluate_geometry_confidence(
        sectors.occupied_sector_count,
        np.column_stack((mls.height_m, linear.height_m, boundary.height_m)),
        normals.coherence,
        vertical.thickness_m,
        vertical.multimodality_score,
        energy.ratio,
        thresholds=thresholds,
    )
    return TerrainGeometryEvaluation(
        mls, linear, boundary, sectors, normals, vertical, energy, confidence
    )


# ---------------------------------------------------------------------------
# Continuous interstitial selection and donor-safe additions


class TerrainDisposition(IntEnum):
    ACCEPTED = 1
    REVIEW = 2
    REJECTED = 3


class TerrainDecisionReason(IntFlag):
    NONE = 0
    HARD_GEOMETRY_VETO = 1 << 0
    BELOW_REQUIRED_TIER = 1 << 1
    EXISTING_CLEARANCE = 1 << 2
    SELECTED_CLEARANCE = 1 << 3
    INVALID_CANDIDATE = 1 << 4
    NO_SAME_ROLE_MEASURED_DONOR = 1 << 5


@dataclass(frozen=True)
class TerrainInterstitialSelection:
    selected_indices: np.ndarray
    review_indices: np.ndarray
    rejected_indices: np.ndarray
    disposition: np.ndarray
    reason_mask: np.ndarray
    blue_noise: BlueNoiseResult
    minimum_tier: ConfidenceTier


def select_variable_radius_interstitials(
    candidate_xyz: np.ndarray | Sequence[Sequence[float]],
    candidate_radius_m: np.ndarray | Sequence[float] | float,
    measured_xyz: np.ndarray | Sequence[Sequence[float]],
    confidence: ConfidenceResult,
    *,
    minimum_tier: ConfidenceTier = ConfidenceTier.SUPPORTED,
    priority: np.ndarray | Sequence[float] | None = None,
    seed: int = DEFAULT_SEED,
    dimensions: int = 3,
) -> TerrainInterstitialSelection:
    """Select pointwise interstitials only after geometry confidence passes."""

    candidates = _as_xyz(candidate_xyz, "candidate_xyz")
    measured = _as_xyz(measured_xyz, "measured_xyz")
    if dimensions not in (2, 3):
        raise ValueError("dimensions must be 2 or 3")
    if len(confidence.reason_mask) != len(candidates):
        raise ValueError("confidence row count differs from candidate_xyz")
    tier = ConfidenceTier(minimum_tier)
    radii = np.asarray(candidate_radius_m, np.float64)
    if radii.ndim == 0:
        radii = np.full(len(candidates), float(radii), np.float64)
    if radii.shape != (len(candidates),):
        raise ValueError("candidate_radius_m must be scalar or contain one value per candidate")
    priorities = (
        np.zeros(len(candidates), np.float64)
        if priority is None
        else np.asarray(priority, np.float64)
    )
    if priorities.shape != (len(candidates),):
        raise ValueError("priority must contain one value per candidate")

    hard_veto = confidence.reason_mask != int(ConfidenceReason.NONE)
    below_tier = ~hard_veto & (confidence.tier < int(tier))
    eligible = ~hard_veto & ~below_tier
    eligible_indices = np.flatnonzero(eligible)
    blue = variable_radius_blue_noise(
        candidates[eligible, :dimensions],
        radii[eligible],
        existing_points=measured[:, :dimensions],
        priority=priorities[eligible],
        seed=seed,
    )
    selected = eligible_indices[blue.selected_indices]
    disposition = np.full(len(candidates), int(TerrainDisposition.REJECTED), np.uint8)
    reasons = np.zeros(len(candidates), np.uint32)
    reasons[hard_veto] |= np.uint32(int(TerrainDecisionReason.HARD_GEOMETRY_VETO))
    disposition[below_tier] = int(TerrainDisposition.REVIEW)
    reasons[below_tier] |= np.uint32(int(TerrainDecisionReason.BELOW_REQUIRED_TIER))
    for local, original in enumerate(eligible_indices):
        mask = int(blue.reason_mask[local])
        if mask & int(BlueNoiseRejection.INVALID_CANDIDATE):
            reasons[original] |= np.uint32(int(TerrainDecisionReason.INVALID_CANDIDATE))
        if mask & int(BlueNoiseRejection.EXISTING_CLEARANCE):
            reasons[original] |= np.uint32(int(TerrainDecisionReason.EXISTING_CLEARANCE))
        if mask & int(BlueNoiseRejection.SELECTED_CLEARANCE):
            reasons[original] |= np.uint32(int(TerrainDecisionReason.SELECTED_CLEARANCE))
    disposition[selected] = int(TerrainDisposition.ACCEPTED)
    reasons[selected] = 0
    review = np.flatnonzero(disposition == int(TerrainDisposition.REVIEW))
    rejected = np.flatnonzero(disposition == int(TerrainDisposition.REJECTED))
    return TerrainInterstitialSelection(
        selected, review, rejected, disposition, reasons, blue, tier
    )


def _field_name(names: Sequence[str], aliases: Sequence[str]) -> str | None:
    exact = {name.lower(): name for name in names}
    for alias in aliases:
        if alias.lower() in exact:
            return exact[alias.lower()]
    normalized = {
        re.sub(r"[^a-z0-9]", "", name.lower()): name for name in names
    }
    for alias in aliases:
        key = re.sub(r"[^a-z0-9]", "", alias.lower())
        if key in normalized:
            return normalized[key]
    return None


def _scan_field(dtype: np.dtype) -> str:
    names = tuple(dtype.names or ())
    value = _field_name(names, ("scalar_ScanID", "ScanID", "scan_id"))
    if value is None:
        raise ValueError("record schema has no ScanID field")
    return value


def measured_scan_mask(records: np.ndarray) -> np.ndarray:
    scan = np.asarray(records[_scan_field(records.dtype)], np.float64)
    rounded = np.rint(scan)
    return (
        np.isfinite(scan)
        & (np.abs(scan - rounded) <= 1.0e-5)
        & (rounded >= MEASURED_SCAN_IDS[0])
        & (rounded <= MEASURED_SCAN_IDS[-1])
    )


@dataclass(frozen=True)
class DonorSampleResult:
    records: np.ndarray
    resolved_query_indices: np.ndarray
    unresolved_query_indices: np.ndarray
    primary_donor_indices: np.ndarray
    nearest_distance_m: np.ndarray
    contributing_donor_count: np.ndarray
    role: str


def _assign_blended_field(
    output: np.ndarray,
    name: str,
    donor_values: np.ndarray,
    weights: np.ndarray,
) -> None:
    dtype = output.dtype[name]
    if dtype.shape:
        raise ValueError(f"interpolated field {name} must be scalar")
    values = np.asarray(donor_values, np.float64)
    blended = np.sum(values * weights, axis=1) / np.maximum(np.sum(weights, axis=1), 1e-30)
    if dtype.kind in "ui":
        limits = np.iinfo(dtype)
        output[name] = np.rint(np.clip(blended, limits.min, limits.max)).astype(dtype)
    elif dtype.kind == "f":
        output[name] = blended.astype(dtype)
    else:
        raise ValueError(f"interpolated field {name} is not numeric")


def build_scanid10_additions(
    candidate_xyz: np.ndarray | Sequence[Sequence[float]],
    donor_records: np.ndarray,
    *,
    role: str,
    donor_role: str,
    maximum_donor_distance_m: float = 0.030,
    neighbours: int = 8,
    interpolated_fields: Sequence[str] | None = None,
    predicted_normals: np.ndarray | Sequence[Sequence[float]] | None = None,
) -> DonorSampleResult:
    """Copy/blend only same-role measured donors and force ScanID 10.

    Unresolved query rows are returned separately and produce no record.  The
    caller therefore leaves their source geometry unchanged.
    """

    candidates = _as_xyz(candidate_xyz, "candidate_xyz")
    role = str(role).upper()
    donor_role = str(donor_role).upper()
    if role not in ROLE_TYPE_ID or donor_role not in ROLE_TYPE_ID:
        raise ValueError("role and donor_role must be ROCK, SAND, or VEG")
    if role != donor_role:
        raise ValueError("cross-role donor sampling is forbidden")
    if donor_records.ndim != 1 or donor_records.dtype.names is None:
        raise ValueError("donor_records must be a one-dimensional structured array")
    names = tuple(donor_records.dtype.names)
    missing = sorted({"x", "y", "z"} - set(names))
    if missing:
        raise ValueError(f"donor_records are missing coordinates {missing}")
    if not np.isfinite(maximum_donor_distance_m) or maximum_donor_distance_m <= 0.0:
        raise ValueError("maximum_donor_distance_m must be positive and finite")
    if not isinstance(neighbours, (int, np.integer)) or neighbours <= 0:
        raise ValueError("neighbours must be a positive integer")
    donor_xyz = np.column_stack(
        (donor_records["x"], donor_records["y"], donor_records["z"])
    ).astype(np.float64)
    valid = measured_scan_mask(donor_records) & np.all(np.isfinite(donor_xyz), axis=1)
    valid_indices = np.flatnonzero(valid)
    if not len(valid_indices):
        return DonorSampleResult(
            np.empty(0, donor_records.dtype),
            np.empty(0, np.int64),
            np.arange(len(candidates), dtype=np.int64),
            np.empty(0, np.int64),
            np.empty(0, np.float64),
            np.empty(0, np.int32),
            role,
        )
    k = min(int(neighbours), len(valid_indices))
    distance, local_index = _query_nearest(candidates, donor_xyz[valid], k)
    within = np.isfinite(distance) & (distance <= maximum_donor_distance_m)
    resolved = np.any(within, axis=1)
    resolved_indices = np.flatnonzero(resolved)
    unresolved_indices = np.flatnonzero(~resolved)
    if not len(resolved_indices):
        return DonorSampleResult(
            np.empty(0, donor_records.dtype), resolved_indices, unresolved_indices,
            np.empty(0, np.int64), np.empty(0, np.float64),
            np.empty(0, np.int32), role,
        )
    safe_local = np.where(within[resolved], local_index[resolved], 0)
    donor_index = valid_indices[safe_local]
    row_distance = distance[resolved]
    primary_column = np.argmin(np.where(within[resolved], row_distance, np.inf), axis=1)
    primary = donor_index[np.arange(len(resolved_indices)), primary_column]
    output = np.asarray(donor_records[primary]).copy()
    output["x"] = candidates[resolved, 0].astype(output.dtype["x"])
    output["y"] = candidates[resolved, 1].astype(output.dtype["y"])
    output["z"] = candidates[resolved, 2].astype(output.dtype["z"])
    output_scan_field = _scan_field(output.dtype)
    output[output_scan_field] = np.asarray(
        ADDITION_SCAN_ID, dtype=output.dtype[output_scan_field]
    )

    default_fields = (
        "red", "green", "blue", "scalar_Intensity", "scalar_Composite"
    )
    fields = default_fields if interpolated_fields is None else tuple(interpolated_fields)
    forbidden = {"x", "y", "z", _scan_field(output.dtype), "nx", "ny", "nz"}
    unknown = sorted(set(fields) - set(names))
    if unknown and interpolated_fields is not None:
        raise ValueError(f"interpolated fields are absent: {unknown}")
    fields = tuple(name for name in fields if name in names and name not in forbidden)
    exact = within[resolved] & (row_distance <= 1.0e-12)
    weights = np.where(within[resolved], 1.0 / np.maximum(row_distance, 1.0e-6) ** 2, 0.0)
    has_exact = np.any(exact, axis=1)
    if np.any(has_exact):
        weights[has_exact] = exact[has_exact].astype(np.float64)
    for field in fields:
        _assign_blended_field(output, field, donor_records[field][donor_index], weights)

    if predicted_normals is not None:
        normals = np.asarray(predicted_normals, np.float64)
        if normals.shape != (len(candidates), 3):
            raise ValueError("predicted_normals must have shape (candidate count, 3)")
        if not {"nx", "ny", "nz"}.issubset(names):
            raise ValueError("predicted_normals supplied but schema has no nx/ny/nz")
        selected_normals = normals[resolved]
        finite_normal = np.all(np.isfinite(selected_normals), axis=1)
        length = np.linalg.norm(selected_normals, axis=1)
        finite_normal &= length > 1.0e-12
        if not np.all(finite_normal):
            raise ValueError("predicted_normals contain invalid resolved rows")
        selected_normals = selected_normals / length[:, None]
        output["nx"], output["ny"], output["nz"] = selected_normals.T

    contributing = np.sum(within[resolved], axis=1).astype(np.int32)
    nearest = np.min(np.where(within[resolved], row_distance, np.inf), axis=1)
    if not np.all(measured_scan_mask(donor_records[primary])):
        raise RuntimeError("internal error: a non-measured primary donor was selected")
    if not np.all(np.rint(output[_scan_field(output.dtype)]) == ADDITION_SCAN_ID):
        raise RuntimeError("internal error: addition ScanID assignment failed")
    return DonorSampleResult(
        output,
        resolved_indices,
        unresolved_indices,
        primary,
        nearest,
        contributing,
        role,
    )


@dataclass(frozen=True)
class TerrainCandidateProvenance:
    xyz: np.ndarray
    target_id: np.ndarray
    disposition: np.ndarray
    decision_reason_mask: np.ndarray
    confidence_reason_mask: np.ndarray
    confidence_tier: np.ndarray
    surface_spread_m: np.ndarray
    donor_index: np.ndarray
    donor_distance_m: np.ndarray

    def summary(self) -> dict[str, object]:
        disposition_counts = {
            value.name: int(np.count_nonzero(self.disposition == int(value)))
            for value in TerrainDisposition
        }
        decision_counts = {
            value.name: int(np.count_nonzero(self.decision_reason_mask & int(value)))
            for value in TerrainDecisionReason
            if value is not TerrainDecisionReason.NONE
        }
        return {
            "candidate_count": len(self.xyz),
            "disposition_counts": disposition_counts,
            "decision_reason_counts": decision_counts,
            "accepted_indices": np.flatnonzero(
                self.disposition == int(TerrainDisposition.ACCEPTED)
            ).tolist(),
            "canonical_writes": False,
        }


def build_candidate_provenance(
    candidate_xyz: np.ndarray | Sequence[Sequence[float]],
    target_id: Sequence[str] | str,
    confidence: ConfidenceResult,
    selection: TerrainInterstitialSelection,
    donor_result: DonorSampleResult,
) -> TerrainCandidateProvenance:
    """Combine geometry, spacing, and donor decisions into one point ledger."""

    xyz = _as_xyz(candidate_xyz, "candidate_xyz")
    count = len(xyz)
    if isinstance(target_id, str):
        target = np.full(count, target_id, dtype=f"U{max(1, len(target_id))}")
    else:
        values = [str(value) for value in target_id]
        if len(values) != count:
            raise ValueError("target_id must be scalar or contain one value per candidate")
        target = np.asarray(values, dtype=f"U{max(1, max(map(len, values), default=1))}")
    if len(selection.disposition) != count or len(confidence.reason_mask) != count:
        raise ValueError("selection/confidence count differs from candidate_xyz")
    disposition = np.asarray(selection.disposition, np.uint8).copy()
    reasons = np.asarray(selection.reason_mask, np.uint32).copy()
    selected = selection.selected_indices
    donor_index = np.full(count, -1, np.int64)
    donor_distance = np.full(count, np.nan, np.float64)
    if np.any(donor_result.resolved_query_indices >= len(selected)) or np.any(
        donor_result.unresolved_query_indices >= len(selected)
    ):
        raise ValueError("donor result does not index the selected candidate subset")
    resolved_original = selected[donor_result.resolved_query_indices]
    unresolved_original = selected[donor_result.unresolved_query_indices]
    disposition[unresolved_original] = int(TerrainDisposition.REVIEW)
    reasons[unresolved_original] |= np.uint32(
        int(TerrainDecisionReason.NO_SAME_ROLE_MEASURED_DONOR)
    )
    donor_index[resolved_original] = donor_result.primary_donor_indices
    donor_distance[resolved_original] = donor_result.nearest_distance_m
    disposition[resolved_original] = int(TerrainDisposition.ACCEPTED)
    reasons[resolved_original] = 0
    return TerrainCandidateProvenance(
        xyz=xyz,
        target_id=target,
        disposition=disposition,
        decision_reason_mask=reasons,
        confidence_reason_mask=np.asarray(confidence.reason_mask, np.uint32),
        confidence_tier=np.asarray(confidence.tier, np.uint8),
        surface_spread_m=np.asarray(confidence.surface_spread_m, np.float64),
        donor_index=donor_index,
        donor_distance_m=donor_distance,
    )


def write_candidate_provenance(
    output_prefix: str | Path,
    provenance: TerrainCandidateProvenance,
    *,
    overwrite: bool = False,
) -> tuple[Path, Path]:
    """Write lossless NPZ detail and a compact JSON summary."""

    prefix = assert_candidate_path(output_prefix)
    npz_path = prefix.with_suffix(".npz")
    json_path = prefix.with_suffix(".json")
    if not overwrite and (npz_path.exists() or json_path.exists()):
        raise FileExistsError(f"provenance output exists: {prefix}")
    prefix.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        npz_path,
        xyz=provenance.xyz,
        target_id=provenance.target_id,
        disposition=provenance.disposition,
        decision_reason_mask=provenance.decision_reason_mask,
        confidence_reason_mask=provenance.confidence_reason_mask,
        confidence_tier=provenance.confidence_tier,
        surface_spread_m=provenance.surface_spread_m,
        donor_index=provenance.donor_index,
        donor_distance_m=provenance.donor_distance_m,
    )
    json_path.write_text(
        json.dumps(provenance.summary(), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return npz_path, json_path


# ---------------------------------------------------------------------------
# Fixed-stride PLY and CleanMesh reduced-analysis adapters


_PLY_TYPES = {
    "float": "<f4", "float32": "<f4", "double": "<f8", "float64": "<f8",
    "uchar": "u1", "uint8": "u1", "char": "i1", "int8": "i1",
    "short": "<i2", "int16": "<i2", "ushort": "<u2", "uint16": "<u2",
    "int": "<i4", "int32": "<i4", "uint": "<u4", "uint32": "<u4",
}
_PLY_OUTPUT_TYPES = {
    "f4": "float", "f8": "double", "u1": "uchar", "i1": "char",
    "i2": "short", "u2": "ushort", "i4": "int", "u4": "uint",
}


@dataclass(frozen=True)
class PlyLayout:
    path: Path
    dtype: np.dtype
    vertex_count: int
    offset: int
    header: bytes
    size_bytes: int
    mtime_ns: int


def inspect_fixed_stride_ply(path: str | Path) -> PlyLayout:
    source = Path(path)
    fields: list[tuple[str, str]] = []
    count = None
    current_element = None
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
            parts = line.decode("ascii", "strict").strip().split()
            if parts[:1] == ["format"]:
                if parts[1:] != ["binary_little_endian", "1.0"]:
                    raise RuntimeError("only binary_little_endian PLY 1.0 is supported")
                format_seen = True
            elif parts[:1] == ["element"]:
                if len(parts) != 3:
                    raise RuntimeError(f"malformed element line in {source}")
                current_element = parts[1]
                if current_element == "vertex":
                    count = int(parts[2])
                elif int(parts[2]) != 0:
                    raise RuntimeError("non-empty non-vertex elements are unsupported")
            elif parts[:1] == ["property"] and current_element == "vertex":
                if len(parts) != 3 or parts[1] == "list" or parts[1] not in _PLY_TYPES:
                    raise RuntimeError(f"unsupported vertex property: {' '.join(parts)}")
                fields.append((parts[2], _PLY_TYPES[parts[1]]))
            if parts == ["end_header"]:
                offset = handle.tell()
                break
    if not format_seen or count is None or count < 0 or not fields:
        raise RuntimeError(f"missing fixed-stride vertex schema in {source}")
    dtype = np.dtype(fields, align=False)
    stat = source.stat()
    expected = offset + count * dtype.itemsize
    if stat.st_size != expected:
        raise RuntimeError(
            f"PLY payload size mismatch in {source}: expected {expected}, found {stat.st_size}"
        )
    return PlyLayout(source, dtype, count, offset, bytes(header), stat.st_size, stat.st_mtime_ns)


def iter_ply_chunks(
    layout_or_path: PlyLayout | str | Path,
    *,
    chunk_size: int = 1_000_000,
) -> Iterator[tuple[int, np.ndarray]]:
    if not isinstance(chunk_size, (int, np.integer)) or chunk_size <= 0:
        raise ValueError("chunk_size must be a positive integer")
    layout = (
        layout_or_path
        if isinstance(layout_or_path, PlyLayout)
        else inspect_fixed_stride_ply(layout_or_path)
    )
    memory = np.memmap(
        layout.path, dtype=layout.dtype, mode="r", offset=layout.offset,
        shape=(layout.vertex_count,),
    )
    try:
        for begin in range(0, layout.vertex_count, int(chunk_size)):
            yield begin, memory[begin : begin + int(chunk_size)]
    finally:
        del memory


def assert_candidate_path(
    path: str | Path,
    *,
    protected_paths: Iterable[str | Path] = (),
) -> Path:
    output = Path(path)
    if CANONICAL_TERRAIN_NAME.fullmatch(output.name):
        raise ValueError(f"refusing canonical terrain output path: {output}")
    resolved = output.expanduser().resolve(strict=False)
    for protected in protected_paths:
        if resolved == Path(protected).expanduser().resolve(strict=False):
            raise ValueError(f"candidate output aliases protected path: {output}")
    return output


def _write_header(handle, dtype: np.dtype, count: int, comments: Iterable[str]) -> None:
    handle.write(b"ply\nformat binary_little_endian 1.0\n")
    for comment in comments:
        value = str(comment)
        if "\n" in value or "\r" in value:
            raise ValueError("PLY comments must be single-line")
        handle.write(f"comment {value}\n".encode("ascii"))
    handle.write(f"element vertex {count}\n".encode("ascii"))
    for name in dtype.names or ():
        code = dtype[name].str.lstrip("<>=|")
        if code not in _PLY_OUTPUT_TYPES:
            raise RuntimeError(f"unsupported PLY dtype {dtype[name]} for {name}")
        handle.write(f"property {_PLY_OUTPUT_TYPES[code]} {name}\n".encode("ascii"))
    handle.write(b"end_header\n")


def reduced_analysis_dtype(full_dtype: np.dtype) -> np.dtype:
    names = tuple(full_dtype.names or ())
    scan = _scan_field(full_dtype)
    stop = names.index(scan) + 1
    fields = [(name, full_dtype.fields[name][0]) for name in names[:stop]]
    if "scalar_TypeID" not in names[:stop]:
        fields.append(("scalar_TypeID", "<f4"))
    return np.dtype(fields, align=False)


def _project_reduced(records: np.ndarray, dtype: np.dtype, role: str) -> np.ndarray:
    role = str(role).upper()
    if role not in ROLE_TYPE_ID:
        raise ValueError("role must be ROCK, SAND, or VEG")
    output = np.empty(len(records), dtype=dtype)
    for name in dtype.names or ():
        if name == "scalar_TypeID":
            output[name] = ROLE_TYPE_ID[role]
        elif name in (records.dtype.names or ()):
            output[name] = records[name]
        else:
            raise ValueError(f"source records are missing reduced field {name}")
    return output


@dataclass(frozen=True)
class LocalSourceSelection:
    layout: PlyLayout
    indices: np.ndarray
    role: str
    label: str


def make_local_source_selection(
    path_or_layout: str | Path | PlyLayout,
    indices: np.ndarray | Sequence[int],
    *,
    role: str,
    label: str,
) -> LocalSourceSelection:
    layout = (
        path_or_layout
        if isinstance(path_or_layout, PlyLayout)
        else inspect_fixed_stride_ply(path_or_layout)
    )
    selected = np.unique(np.asarray(indices, np.int64))
    if selected.ndim != 1:
        raise ValueError("indices must be one-dimensional")
    if len(selected) and (selected[0] < 0 or selected[-1] >= layout.vertex_count):
        raise ValueError("selection index lies outside source PLY")
    role = str(role).upper()
    if role not in ROLE_TYPE_ID:
        raise ValueError("role must be ROCK, SAND, or VEG")
    return LocalSourceSelection(layout, selected, role, str(label))


@dataclass(frozen=True)
class LocalAdditionBatch:
    records: np.ndarray
    role: str
    label: str

    def __post_init__(self) -> None:
        role = str(self.role).upper()
        if role not in ROLE_TYPE_ID:
            raise ValueError("role must be ROCK, SAND, or VEG")
        if self.records.ndim != 1 or self.records.dtype.names is None:
            raise ValueError("addition records must be a structured vector")
        if len(self.records):
            scan = np.asarray(self.records[_scan_field(self.records.dtype)], np.float64)
            if not np.all(np.isfinite(scan) & (np.rint(scan) == ADDITION_SCAN_ID)):
                raise ValueError("all local additions must have ScanID 10")


def _xyz_scan_fingerprint(records: np.ndarray) -> str:
    digest = hashlib.sha256()
    _update_xyz_scan_digest(digest, records)
    return digest.hexdigest()


def _update_xyz_scan_digest(digest, records: np.ndarray) -> None:
    """Extend a span identity digest without materialising the full span."""

    names = ("x", "y", "z", _scan_field(records.dtype))
    dtype = np.dtype(
        [(name, records.dtype.fields[name][0]) for name in names], align=False
    )
    identity = np.empty(len(records), dtype=dtype)
    for name in names:
        identity[name] = records[name]
    digest.update(identity.tobytes(order="C"))


def _xyz_scan_type_keys(records: np.ndarray) -> np.ndarray:
    """Return byte-comparable geometry/material keys for reordered output."""

    names = ("x", "y", "z", _scan_field(records.dtype), "scalar_TypeID")
    missing = [name for name in names if name not in (records.dtype.names or ())]
    if missing:
        raise RuntimeError(f"analysis records lack identity fields {missing}")
    dtype = np.dtype(
        [(name, records.dtype.fields[name][0]) for name in names], align=False
    )
    identity = np.empty(len(records), dtype=dtype)
    for name in names:
        identity[name] = records[name]
    return identity.view(np.dtype((np.void, dtype.itemsize))).reshape(-1)


@dataclass(frozen=True)
class LocalAnalysisSpan:
    label: str
    role: str
    start: int
    count: int
    is_addition: bool
    xyz_scan_sha256: str


@dataclass(frozen=True)
class LocalAnalysisManifest:
    input_path: Path
    manifest_path: Path
    vertex_count: int
    dtype: np.dtype
    spans: tuple[LocalAnalysisSpan, ...]
    source_paths: tuple[Path, ...]

    def as_json(self) -> dict[str, object]:
        return {
            "version": 1,
            "input_path": str(self.input_path),
            "vertex_count": self.vertex_count,
            "record_stride": self.dtype.itemsize,
            "properties": list(self.dtype.names or ()),
            "spans": [
                {
                    "label": span.label,
                    "role": span.role,
                    "start": span.start,
                    "count": span.count,
                    "is_addition": span.is_addition,
                    "xyz_scan_sha256": span.xyz_scan_sha256,
                }
                for span in self.spans
            ],
            "source_paths": [str(path) for path in self.source_paths],
            "canonical_writes": False,
        }


def write_local_analysis_input(
    output_path: str | Path,
    selections: Sequence[LocalSourceSelection],
    additions: Sequence[LocalAdditionBatch],
    *,
    overwrite: bool = False,
    manifest_path: str | Path | None = None,
    chunk_size: int = 1_000_000,
) -> LocalAnalysisManifest:
    """Write a local collar+addition input for CleanMesh reduced analysis."""

    if not selections:
        raise ValueError("at least one local source selection is required")
    if not isinstance(chunk_size, (int, np.integer)) or chunk_size <= 0:
        raise ValueError("chunk_size must be a positive integer")
    source_dtype = selections[0].layout.dtype
    for selection in selections:
        if selection.layout.dtype != source_dtype:
            raise ValueError("local source schemas differ")
    for batch in additions:
        if batch.records.dtype != source_dtype:
            raise ValueError("addition schema differs from local source schema")
    output = assert_candidate_path(
        output_path, protected_paths=[selection.layout.path for selection in selections]
    )
    manifest_output = (
        Path(manifest_path)
        if manifest_path is not None
        else output.with_suffix(output.suffix + ".manifest.json")
    )
    assert_candidate_path(
        manifest_output,
        protected_paths=[selection.layout.path for selection in selections],
    )
    if not overwrite and (output.exists() or manifest_output.exists()):
        raise FileExistsError(f"local analysis output exists: {output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    manifest_output.parent.mkdir(parents=True, exist_ok=True)
    reduced = reduced_analysis_dtype(source_dtype)
    total = sum(len(selection.indices) for selection in selections) + sum(
        len(batch.records) for batch in additions
    )
    temporary = output.with_name(f".{output.name}.{os.getpid()}.partial")
    spans: list[LocalAnalysisSpan] = []
    cursor = 0
    try:
        with temporary.open("wb") as handle:
            _write_header(
                handle,
                reduced,
                total,
                (
                    "Scene1 v11 candidate-only local terrain analysis",
                    "TypeID 0=ROCK, 1=SAND, 2=VEG",
                    "ScanID 10 identifies proposed terrain additions",
                ),
            )
            for selection in selections:
                before = selection.layout.path.stat()
                memory = np.memmap(
                    selection.layout.path,
                    dtype=selection.layout.dtype,
                    mode="r",
                    offset=selection.layout.offset,
                    shape=(selection.layout.vertex_count,),
                )
                span_digest = hashlib.sha256()
                for begin in range(0, len(selection.indices), int(chunk_size)):
                    index_chunk = selection.indices[begin : begin + int(chunk_size)]
                    selected = np.asarray(memory[index_chunk]).copy()
                    projected = _project_reduced(
                        selected, reduced, selection.role
                    )
                    projected.tofile(handle)
                    _update_xyz_scan_digest(span_digest, projected)
                del memory
                spans.append(
                    LocalAnalysisSpan(
                        selection.label,
                        selection.role,
                        cursor,
                        len(selection.indices),
                        False,
                        span_digest.hexdigest(),
                    )
                )
                cursor += len(selection.indices)
                after = selection.layout.path.stat()
                if (
                    before.st_size != selection.layout.size_bytes
                    or before.st_mtime_ns != selection.layout.mtime_ns
                    or after.st_size != selection.layout.size_bytes
                    or after.st_mtime_ns != selection.layout.mtime_ns
                ):
                    raise RuntimeError(
                        "local source changed during analysis input build: "
                        f"{selection.layout.path}"
                    )
            for batch in additions:
                projected = _project_reduced(batch.records, reduced, batch.role)
                projected.tofile(handle)
                spans.append(
                    LocalAnalysisSpan(
                        batch.label, str(batch.role).upper(), cursor, len(projected), True,
                        _xyz_scan_fingerprint(projected),
                    )
                )
                cursor += len(projected)
        if cursor != total:
            raise RuntimeError(f"local analysis wrote {cursor} records, expected {total}")
        os.replace(temporary, output)
        manifest = LocalAnalysisManifest(
            output,
            manifest_output,
            total,
            reduced,
            tuple(spans),
            tuple(selection.layout.path for selection in selections),
        )
        manifest_output.write_text(
            json.dumps(manifest.as_json(), indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        return manifest
    except BaseException:
        if temporary.exists():
            temporary.unlink()
        raise


@dataclass(frozen=True)
class AnalysedAdditionBatch:
    records: np.ndarray
    role: str
    label: str
    local_start: int


def extract_cleanmesh_analysed_additions(
    analysed_path: str | Path,
    manifest: LocalAnalysisManifest,
) -> tuple[AnalysedAdditionBatch, ...]:
    """Extract and identity-reorder ScanID 10 additions from tiled output."""

    layout = inspect_fixed_stride_ply(analysed_path)
    if layout.vertex_count != manifest.vertex_count:
        raise RuntimeError(
            f"CleanMesh output count changed: {layout.vertex_count} != {manifest.vertex_count}"
        )
    memory = np.memmap(
        layout.path, dtype=layout.dtype, mode="r", offset=layout.offset,
        shape=(layout.vertex_count,),
    )
    input_layout = inspect_fixed_stride_ply(manifest.input_path)
    if input_layout.vertex_count != manifest.vertex_count:
        raise RuntimeError("local analysis input count changed after manifest creation")
    input_memory = np.memmap(
        input_layout.path,
        dtype=input_layout.dtype,
        mode="r",
        offset=input_layout.offset,
        shape=(input_layout.vertex_count,),
    )
    result: list[AnalysedAdditionBatch] = []
    try:
        addition_spans = tuple(span for span in manifest.spans if span.is_addition)
        expected_parts = [
            np.asarray(
                input_memory[span.start : span.start + span.count]
            ).copy()
            for span in addition_spans
        ]
        expected = (
            np.concatenate(expected_parts)
            if expected_parts else np.empty(0, dtype=input_layout.dtype)
        )
        output_scan = np.asarray(memory[_scan_field(layout.dtype)], np.float64)
        output_addition_index = np.flatnonzero(
            np.isfinite(output_scan)
            & (np.rint(output_scan) == ADDITION_SCAN_ID)
        )
        analysed_additions = np.asarray(memory[output_addition_index]).copy()
        if len(analysed_additions) != len(expected):
            raise RuntimeError(
                "CleanMesh changed the ScanID 10 addition count: "
                f"{len(analysed_additions)} != {len(expected)}"
            )
        expected_keys = _xyz_scan_type_keys(expected)
        analysed_keys = _xyz_scan_type_keys(analysed_additions)
        expected_order = np.argsort(expected_keys, kind="stable")
        analysed_order = np.argsort(analysed_keys, kind="stable")
        sorted_expected = expected_keys[expected_order]
        if len(sorted_expected) > 1 and np.any(
            sorted_expected[1:] == sorted_expected[:-1]
        ):
            raise RuntimeError("local additions have ambiguous duplicate identities")
        if not np.array_equal(sorted_expected, analysed_keys[analysed_order]):
            raise RuntimeError("CleanMesh changed an addition XYZ/ScanID/TypeID identity")
        mapping = np.empty(len(expected_order), np.int64)
        mapping[expected_order] = analysed_order
        reordered = analysed_additions[mapping]
        cursor = 0
        for span in addition_spans:
            records = reordered[cursor : cursor + span.count].copy()
            cursor += span.count
            if _xyz_scan_fingerprint(records) != span.xyz_scan_sha256:
                raise RuntimeError(
                    "CleanMesh changed addition identity in "
                    f"{span.label}"
                )
            scan = np.asarray(records[_scan_field(records.dtype)], np.float64)
            if not np.all(np.isfinite(scan) & (np.rint(scan) == ADDITION_SCAN_ID)):
                raise RuntimeError(f"analysed addition span {span.label} lost ScanID 10")
            if "scalar_TypeID" in (records.dtype.names or ()):
                expected_type = ROLE_TYPE_ID[span.role]
                if not np.all(records["scalar_TypeID"] == expected_type):
                    raise RuntimeError(f"analysed addition span {span.label} changed TypeID")
            result.append(AnalysedAdditionBatch(records, span.role, span.label, span.start))
    finally:
        del input_memory
        del memory
    return tuple(result)


def project_analysed_additions(
    analysed_records: np.ndarray,
    target_dtype: np.dtype,
) -> np.ndarray:
    """Strip temporary TypeID and project analysed additions to canonical schema."""

    if analysed_records.ndim != 1 or analysed_records.dtype.names is None:
        raise ValueError("analysed_records must be a structured vector")
    missing = sorted(set(target_dtype.names or ()) - set(analysed_records.dtype.names or ()))
    if missing:
        raise ValueError(f"analysed records are missing target fields {missing}")
    output = np.empty(len(analysed_records), target_dtype)
    for name in target_dtype.names or ():
        output[name] = analysed_records[name]
    if len(output):
        scan = np.asarray(output[_scan_field(output.dtype)], np.float64)
        if not np.all(np.isfinite(scan) & (np.rint(scan) == ADDITION_SCAN_ID)):
            raise ValueError("projected analysed records are not all ScanID 10")
    return output


def _patch_header_count(header: bytes, count: int) -> bytes:
    pattern = re.compile(rb"(?m)^(element vertex )[0-9]+([ \t]*)(\r?)$")
    match = pattern.search(header)
    if match is None:
        raise RuntimeError("could not patch PLY vertex count")
    replacement = (
        match.group(1)
        + str(int(count)).encode("ascii")
        + match.group(2)
        + match.group(3)
    )
    return header[: match.start()] + replacement + header[match.end() :]


def write_append_only_candidate(
    source_path: str | Path,
    additions: np.ndarray,
    output_path: str | Path,
    *,
    overwrite: bool = False,
    chunk_size: int = 1_000_000,
) -> dict[str, object]:
    """Copy the source payload byte-for-byte and append accepted ScanID 10 rows."""

    layout = inspect_fixed_stride_ply(source_path)
    if additions.dtype != layout.dtype or additions.ndim != 1:
        raise ValueError("additions must be a structured vector matching the source schema")
    if len(additions):
        scan = np.asarray(additions[_scan_field(additions.dtype)], np.float64)
        if not np.all(np.isfinite(scan) & (np.rint(scan) == ADDITION_SCAN_ID)):
            raise ValueError("append-only additions must all have ScanID 10")
    output = assert_candidate_path(output_path, protected_paths=[layout.path])
    if output.exists() and not overwrite:
        raise FileExistsError(f"candidate output exists: {output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_name(f".{output.name}.{os.getpid()}.partial")
    source_digest = hashlib.sha256(layout.header)
    candidate_header = _patch_header_count(layout.header, layout.vertex_count + len(additions))
    candidate_digest = hashlib.sha256(candidate_header)
    try:
        with temporary.open("wb") as handle:
            handle.write(candidate_header)
            for _, chunk in iter_ply_chunks(layout, chunk_size=chunk_size):
                payload = np.asarray(chunk).tobytes(order="C")
                source_digest.update(payload)
                candidate_digest.update(payload)
                handle.write(payload)
            if len(additions):
                payload = additions.tobytes(order="C")
                candidate_digest.update(payload)
                handle.write(payload)
        current = layout.path.stat()
        if current.st_size != layout.size_bytes or current.st_mtime_ns != layout.mtime_ns:
            raise RuntimeError(f"source changed during append-only build: {layout.path}")
        os.replace(temporary, output)
    except BaseException:
        if temporary.exists():
            temporary.unlink()
        raise
    return {
        "source_path": str(layout.path),
        "source_sha256": source_digest.hexdigest(),
        "source_vertex_count": layout.vertex_count,
        "candidate_path": str(output),
        "candidate_sha256": candidate_digest.hexdigest(),
        "candidate_vertex_count": layout.vertex_count + len(additions),
        "addition_count": len(additions),
        "addition_scan_id": ADDITION_SCAN_ID,
        "base_payload_byte_identical": True,
        "canonical_writes": False,
    }


__all__ = [
    "ADDITION_SCAN_ID",
    "AnalysedAdditionBatch",
    "DeficitKind",
    "DensityDeficitComponent",
    "DensityDeficitResult",
    "DonorSampleResult",
    "LocalAdditionBatch",
    "LocalAnalysisManifest",
    "LocalAnalysisSpan",
    "LocalSourceSelection",
    "MEASURED_SCAN_IDS",
    "NormalCoherenceResult",
    "PlyLayout",
    "ResidualEnergyResult",
    "RobustSurfaceResult",
    "SectorSupportResult",
    "SurfacePredictionResult",
    "TerrainCandidateProvenance",
    "TerrainDecisionReason",
    "TerrainDisposition",
    "TerrainGeometryEvaluation",
    "TerrainInterstitialSelection",
    "TerrainReviewTarget",
    "VerticalDistributionResult",
    "assert_candidate_path",
    "build_candidate_provenance",
    "build_scanid10_additions",
    "compute_eight_sector_support",
    "compute_local_normal_coherence",
    "compute_local_residual_energy",
    "compute_vertical_distribution_metrics",
    "detect_crack_density_deficit",
    "detect_density_deficits",
    "detect_marked_density_deficits",
    "detect_scanner_footprint_deficit",
    "evaluate_terrain_geometry",
    "extract_cleanmesh_analysed_additions",
    "inspect_fixed_stride_ply",
    "iter_ply_chunks",
    "make_local_source_selection",
    "measured_scan_mask",
    "predict_delaunay_linear_surface",
    "predict_lower_boundary_surface",
    "predict_robust_quadratic_mls",
    "project_analysed_additions",
    "reduced_analysis_dtype",
    "residual_energy_ratio",
    "select_variable_radius_interstitials",
    "terrain_targets_from_review_config",
    "write_append_only_candidate",
    "write_candidate_provenance",
    "write_local_analysis_input",
]
