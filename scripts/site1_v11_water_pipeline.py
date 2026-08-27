#!/usr/bin/env python3
"""Bounded-memory, candidate-only WATER refinement for Scene1 v11.

This module performs the two conservative edits that can be made without
inventing a new WATER surface:

* thin *existing* v10 WATER point-by-point east of the registered red guide;
* recover exact v10 Poisson-selected WATER records that were removed only by
  the final all-terrain blocker, and only at reviewed terrain interfaces.

The current v10 records that survive thinning are copied byte-for-byte.  A
configured review rectangle is never sufficient to add a point: a recovery
must also be an exact member of ``pre_allterrain - post_allterrain``, lie in
the narrow relaxed terrain-clearance band, have nearby surviving WATER, and
clear every surviving WATER duplicate.  True-hole synthesis intentionally
does not live here.

All large PLYs are streamed or memory mapped.  Density is accumulated into a
small measurement raster, but record selection uses a continuous bilinear
retention field and an independent stable hash for each point; there are no
cell quotas and therefore no square selection boundaries.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import sys
from typing import Callable, Iterable, Iterator, Mapping, Sequence

import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import site1_v11_water_density as density  # noqa: E402
import site1_v11_confidence as confidence  # noqa: E402


WATER_SCAN_ID = 999.0
DEFAULT_INTERFACE_MARK_IDS = (
    "image_1_mark_1",
    "image_1_mark_4",
    "image_2_mark_5",
    "image_3_mark_1",
    "image_4_mark_1",
    "image_5_mark_1",
    "image_5_mark_2",
)


@dataclass(frozen=True)
class SourceFingerprint:
    path: str
    size_bytes: int
    mtime_ns: int
    sha256: str
    points: int
    record_stride: int


def sha256_path(path: str | Path, *, block_size: int = 16 * 1024 * 1024) -> str:
    """Hash a file without materialising it."""

    source = Path(path)
    digest = hashlib.sha256()
    with source.open("rb") as handle:
        while True:
            block = handle.read(block_size)
            if not block:
                break
            digest.update(block)
    return digest.hexdigest()


def fingerprint_ply(path: str | Path) -> SourceFingerprint:
    info = density.inspect_fixed_stride_ply(path)
    return SourceFingerprint(
        path=str(info.path.resolve()),
        size_bytes=info.size_bytes,
        mtime_ns=info.mtime_ns,
        sha256=sha256_path(info.path),
        points=info.count,
        record_stride=info.dtype.itemsize,
    )


def _assert_stat_unchanged(fingerprint: SourceFingerprint) -> None:
    stat = Path(fingerprint.path).stat()
    if stat.st_size != fingerprint.size_bytes or stat.st_mtime_ns != fingerprint.mtime_ns:
        raise RuntimeError(f"source changed after hashing: {fingerprint.path}")


@dataclass(frozen=True)
class ReviewRegion:
    id: str
    bbox: tuple[float, float, float, float]
    polygon_xy: tuple[tuple[float, float], ...] | None = None

    def mask(self, xy: np.ndarray) -> np.ndarray:
        selected = density.bbox_mask(xy, self.bbox)
        if self.polygon_xy is not None:
            selected &= density.polygon_mask(xy, self.polygon_xy)
        return selected


def load_interface_review_regions(
    config_path: str | Path,
    *,
    mark_ids: Sequence[str] = DEFAULT_INTERFACE_MARK_IDS,
) -> tuple[ReviewRegion, ...]:
    """Load only explicitly named interface regions from the v11 review file."""

    with Path(config_path).open("r", encoding="utf-8") as handle:
        config = json.load(handle)
    by_id: dict[str, Mapping[str, object]] = {}
    for marks in config.get("marked_locations", {}).values():
        for mark in marks:
            by_id[str(mark["id"])] = mark
    regions: list[ReviewRegion] = []
    for mark_id in mark_ids:
        if mark_id not in by_id:
            raise KeyError(f"review mark is missing from config: {mark_id}")
        mark = by_id[mark_id]
        bbox = tuple(float(value) for value in mark["review_bbox"])
        density._as_bbox(bbox)
        regions.append(ReviewRegion(str(mark_id), bbox))
    if len({region.id for region in regions}) != len(regions):
        raise ValueError("interface review mark ids must be unique")
    return tuple(regions)


def load_taper_guide(config_path: str | Path) -> np.ndarray:
    with Path(config_path).open("r", encoding="utf-8") as handle:
        config = json.load(handle)
    guide = np.asarray(
        config["plan_annotations"]["red_density_taper_guide"]["polyline"],
        dtype=np.float64,
    )
    if guide.ndim != 2 or guide.shape[1] != 2 or len(guide) < 2:
        raise ValueError("red density taper guide is invalid")
    return guide


def _union_bbox(regions: Sequence[ReviewRegion], margin_m: float = 0.0) -> tuple[float, ...]:
    if not regions:
        raise ValueError("at least one review region is required")
    margin = float(margin_m)
    if not np.isfinite(margin) or margin < 0.0:
        raise ValueError("margin_m must be finite and non-negative")
    return (
        min(region.bbox[0] for region in regions) - margin,
        max(region.bbox[1] for region in regions) + margin,
        min(region.bbox[2] for region in regions) - margin,
        max(region.bbox[3] for region in regions) + margin,
    )


def _any_region_mask(xy: np.ndarray, regions: Sequence[ReviewRegion]) -> np.ndarray:
    selected = np.zeros(len(xy), dtype=bool)
    for region in regions:
        selected |= region.mask(xy)
    return selected


def _smoothstep01(value: np.ndarray) -> np.ndarray:
    clipped = np.clip(value, 0.0, 1.0)
    return clipped * clipped * (3.0 - 2.0 * clipped)


def _convolve_axis(values: np.ndarray, kernel: np.ndarray, axis: int) -> np.ndarray:
    radius = len(kernel) // 2
    if radius == 0:
        return values.astype(np.float64, copy=True)
    pad = [(0, 0), (0, 0)]
    pad[axis] = (radius, radius)
    padded = np.pad(values, pad, mode="edge")
    result = np.zeros_like(values, dtype=np.float64)
    for offset, weight in enumerate(kernel):
        slices = [slice(None), slice(None)]
        slices[axis] = slice(offset, offset + values.shape[axis])
        result += float(weight) * padded[tuple(slices)]
    return result


def _gaussian_smooth(values: np.ndarray, sigma_cells: float) -> np.ndarray:
    sigma = float(sigma_cells)
    if not np.isfinite(sigma) or sigma < 0.0:
        raise ValueError("sigma_cells must be finite and non-negative")
    if sigma < 0.25:
        return values.astype(np.float64, copy=True)
    radius = max(1, int(math.ceil(3.0 * sigma)))
    coordinate = np.arange(-radius, radius + 1, dtype=np.float64)
    kernel = np.exp(-0.5 * np.square(coordinate / sigma))
    kernel /= np.sum(kernel)
    return _convolve_axis(_convolve_axis(values, kernel, 0), kernel, 1)


@dataclass(frozen=True)
class DensityRetentionGrid:
    bbox: tuple[float, float, float, float]
    cell_size_m: float
    retention: np.ndarray
    terrain_density_per_m2: np.ndarray
    water_density_per_m2: np.ndarray
    guide_xy: np.ndarray
    taper_start_m: float
    taper_end_m: float
    floor_ratio: float
    terrain_points_measured: int
    water_points_measured: int
    smoothing_bandwidth_m: float | None = None
    # Production grids retain the pre-transition terrain target so query()
    # can apply the analytic smoothstep at the exact point location.  None is
    # supported for small hand-built/test grids whose ``retention`` is final.
    target_ratio: np.ndarray | None = None

    def query(self, xy: np.ndarray | Sequence[Sequence[float]]) -> np.ndarray:
        """Bilinearly interpolate the continuous per-point keep probability."""

        points = density._as_xy(xy, "xy")
        xmin, xmax, ymin, ymax = self.bbox
        rows, columns = self.retention.shape
        inside = (
            (points[:, 0] >= xmin)
            & (points[:, 0] <= xmax)
            & (points[:, 1] >= ymin)
            & (points[:, 1] <= ymax)
        )
        result = np.ones(len(points), np.float64)
        if not np.any(inside):
            return result
        local = points[inside]
        gx = (local[:, 0] - xmin) / self.cell_size_m - 0.5
        gy = (local[:, 1] - ymin) / self.cell_size_m - 0.5
        x0 = np.floor(gx).astype(np.int64)
        y0 = np.floor(gy).astype(np.int64)
        tx = gx - x0
        ty = gy - y0
        x0 = np.clip(x0, 0, columns - 1)
        y0 = np.clip(y0, 0, rows - 1)
        x1 = np.clip(x0 + 1, 0, columns - 1)
        y1 = np.clip(y0 + 1, 0, rows - 1)
        field = self.retention if self.target_ratio is None else self.target_ratio
        value = (
            field[y0, x0] * (1.0 - tx) * (1.0 - ty)
            + field[y0, x1] * tx * (1.0 - ty)
            + field[y1, x0] * (1.0 - tx) * ty
            + field[y1, x1] * tx * ty
        )
        if self.target_ratio is not None:
            signed = density.eastward_signed_distance(
                local, self.guide_xy
            ).signed_distance_m
            transition = _smoothstep01(
                (signed - self.taper_start_m)
                / (self.taper_end_m - self.taper_start_m)
            )
            value = 1.0 - transition * (1.0 - value)
            value[signed <= self.taper_start_m] = 1.0
        result[inside] = np.clip(value, self.floor_ratio, 1.0)
        return result

    def summary(self) -> dict[str, object]:
        return {
            "bbox": list(self.bbox),
            "cell_size_m": self.cell_size_m,
            "shape": list(self.retention.shape),
            "taper_start_m": self.taper_start_m,
            "taper_end_m": self.taper_end_m,
            "floor_ratio": self.floor_ratio,
            "smoothing_bandwidth_m": self.smoothing_bandwidth_m,
            "terrain_points_measured": self.terrain_points_measured,
            "water_points_measured": self.water_points_measured,
            "retention_quantiles": {
                str(q): float(np.quantile(self.retention, q))
                for q in (0.0, 0.05, 0.25, 0.50, 0.75, 0.95, 1.0)
            },
            "terrain_density_quantiles_per_m2": {
                str(q): float(np.quantile(self.terrain_density_per_m2, q))
                for q in (0.05, 0.50, 0.95)
            },
            "water_density_quantiles_per_m2": {
                str(q): float(np.quantile(self.water_density_per_m2, q))
                for q in (0.05, 0.50, 0.95)
            },
        }


def _grid_shape(bbox: Sequence[float], cell_size_m: float) -> tuple[int, int]:
    xmin, xmax, ymin, ymax = density._as_bbox(bbox)
    cell = float(cell_size_m)
    if not np.isfinite(cell) or cell <= 0.0:
        raise ValueError("cell_size_m must be finite and positive")
    columns = max(1, int(math.ceil((xmax - xmin) / cell)))
    rows = max(1, int(math.ceil((ymax - ymin) / cell)))
    if rows * columns > 20_000_000:
        raise ValueError("measurement grid is unreasonably large")
    return rows, columns


def _accumulate_xy_counts(
    paths: Sequence[str | Path],
    bbox: Sequence[float],
    cell_size_m: float,
    *,
    record_filters: Sequence[Callable[[np.ndarray], np.ndarray] | None] | None = None,
    scan_id: float | None = None,
    chunk_size: int = 1_000_000,
) -> tuple[np.ndarray, int]:
    rows, columns = _grid_shape(bbox, cell_size_m)
    xmin, xmax, ymin, ymax = density._as_bbox(bbox)
    # Signed int64 interoperates with np.bincount across NumPy versions and
    # still has vastly more headroom than any practical per-cell count.
    counts = np.zeros((rows, columns), dtype=np.int64)
    filters = list(record_filters) if record_filters is not None else [None] * len(paths)
    if len(filters) != len(paths):
        raise ValueError("record_filters must match paths")
    measured = 0
    for source, record_filter in zip(paths, filters):
        info = density.inspect_fixed_stride_ply(source)
        required = {"x", "y"}
        if scan_id is not None:
            required.add("scalar_ScanID")
        missing = sorted(required - set(info.dtype.names or ()))
        if missing:
            raise RuntimeError(f"density source lacks fields {missing}: {info.path}")
        for _, records in density.iter_ply_chunks(info.path, info=info, chunk_size=chunk_size):
            x = np.asarray(records["x"], np.float64)
            y = np.asarray(records["y"], np.float64)
            keep = (
                np.isfinite(x)
                & np.isfinite(y)
                & (x >= xmin)
                & (x <= xmax)
                & (y >= ymin)
                & (y <= ymax)
            )
            if scan_id is not None:
                keep &= records["scalar_ScanID"] == scan_id
            if record_filter is not None:
                custom = np.asarray(record_filter(records), dtype=bool)
                if custom.shape != (len(records),):
                    raise ValueError("record filter returned the wrong shape")
                keep &= custom
            if not np.any(keep):
                continue
            ix = np.minimum(((x[keep] - xmin) / cell_size_m).astype(np.int64), columns - 1)
            iy = np.minimum(((y[keep] - ymin) / cell_size_m).astype(np.int64), rows - 1)
            flat = iy * columns + ix
            counts += np.bincount(flat, minlength=rows * columns).reshape(rows, columns)
            measured += int(np.count_nonzero(keep))
    return counts, measured


def build_measured_retention_grid(
    terrain_paths: Sequence[str | Path],
    current_water_path: str | Path,
    *,
    bbox: Sequence[float],
    guide_xy: np.ndarray | Sequence[Sequence[float]],
    cell_size_m: float = 0.05,
    smoothing_bandwidth_m: float = 0.20,
    taper_start_m: float = 0.0,
    taper_end_m: float = 1.50,
    floor_ratio: float = 0.08,
    terrain_record_filters: Sequence[Callable[[np.ndarray], np.ndarray] | None] | None = None,
    chunk_size: int = 1_000_000,
) -> DensityRetentionGrid:
    """Measure a continuous terrain-relative WATER retention field.

    The reference ratio is local smoothed SAND+ROCK point density divided by
    local smoothed v10 WATER density, bounded to ``[floor_ratio, 1]``.  It is
    introduced with a C1 smoothstep east of the guide and then constrained to
    be non-increasing along each eastward raster row.  The raster only
    measures the field; per-record hash decisions remain independent.
    """

    box = density._as_bbox(bbox)
    guide = density._as_xy(guide_xy, "guide_xy", allow_empty=False)
    start = float(taper_start_m)
    end = float(taper_end_m)
    floor = float(floor_ratio)
    if not 0.0 <= start < end:
        raise ValueError("taper distances must satisfy 0 <= start < end")
    if not 0.0 < floor <= 1.0:
        raise ValueError("floor_ratio must lie in (0, 1]")
    terrain_counts, terrain_total = _accumulate_xy_counts(
        terrain_paths,
        box,
        cell_size_m,
        record_filters=terrain_record_filters,
        chunk_size=chunk_size,
    )
    water_counts, water_total = _accumulate_xy_counts(
        [current_water_path],
        box,
        cell_size_m,
        scan_id=WATER_SCAN_ID,
        chunk_size=chunk_size,
    )
    if terrain_total == 0 or water_total == 0:
        raise RuntimeError("taper measurement requires both terrain and WATER support")
    sigma = float(smoothing_bandwidth_m) / float(cell_size_m)
    terrain_smooth = _gaussian_smooth(terrain_counts, sigma)
    water_smooth = _gaussian_smooth(water_counts, sigma)
    area = float(cell_size_m) ** 2
    terrain_density = terrain_smooth / area
    water_density = water_smooth / area
    target = np.divide(
        terrain_density,
        water_density,
        out=np.full_like(terrain_density, floor),
        where=water_density > 1.0e-12,
    )
    target = np.clip(target, floor, 1.0)

    rows, columns = target.shape
    xmin, _, ymin, _ = box
    x = xmin + (np.arange(columns, dtype=np.float64) + 0.5) * cell_size_m
    y = ymin + (np.arange(rows, dtype=np.float64) + 0.5) * cell_size_m
    xx, yy = np.meshgrid(x, y)
    grid_xy = np.column_stack((xx.ravel(), yy.ravel()))
    signed = density.eastward_signed_distance(grid_xy, guide).signed_distance_m.reshape(rows, columns)
    target[signed <= start] = 1.0
    # The registered guide is approximately north/south, so increasing X is
    # the requested outward direction.  This removes target-density rebounds
    # without imposing any point quota or hard line at a cell edge.
    for row in range(rows):
        east = np.flatnonzero(signed[row] > start)
        if len(east):
            begin = int(east[0])
            target[row, begin:] = np.minimum.accumulate(target[row, begin:])
    transition = _smoothstep01((signed - start) / (end - start))
    retention = 1.0 - transition * (1.0 - target)
    retention[signed <= start] = 1.0
    for row in range(rows):
        east = np.flatnonzero(signed[row] > start)
        if len(east):
            begin = int(east[0])
            retention[row, begin:] = np.minimum.accumulate(retention[row, begin:])
    retention = np.clip(retention, floor, 1.0)
    return DensityRetentionGrid(
        bbox=box,
        cell_size_m=float(cell_size_m),
        retention=retention,
        terrain_density_per_m2=terrain_density,
        water_density_per_m2=water_density,
        guide_xy=guide,
        taper_start_m=start,
        taper_end_m=end,
        floor_ratio=floor,
        terrain_points_measured=terrain_total,
        water_points_measured=water_total,
        smoothing_bandwidth_m=float(smoothing_bandwidth_m),
        target_ratio=target,
    )


def _mix64(value: np.ndarray) -> np.ndarray:
    value = value.astype(np.uint64, copy=False)
    value ^= value >> np.uint64(30)
    value *= np.uint64(0xBF58476D1CE4E5B9)
    value ^= value >> np.uint64(27)
    value *= np.uint64(0x94D049BB133111EB)
    value ^= value >> np.uint64(31)
    return value


def stable_point_uniform(records: np.ndarray, *, seed: int) -> np.ndarray:
    """Return a deterministic U[0,1) variate from exact point attributes."""

    names = set(records.dtype.names or ())
    missing = sorted({"x", "y", "z"} - names)
    if missing:
        raise RuntimeError(f"records are missing coordinate fields {missing}")
    x = np.asarray(records["x"], dtype="<f4").view("<u4").astype(np.uint64)
    y = np.asarray(records["y"], dtype="<f4").view("<u4").astype(np.uint64)
    z = np.asarray(records["z"], dtype="<f4").view("<u4").astype(np.uint64)
    seed64 = np.uint64(int(seed) & ((1 << 64) - 1))
    value = _mix64(x ^ seed64)
    value ^= _mix64(y ^ np.uint64(0x9E3779B97F4A7C15))
    value ^= _mix64(z ^ np.uint64(0xD1B54A32D192ED03))
    if "scalar_Intensity" in names:
        intensity = np.asarray(records["scalar_Intensity"], dtype="<f4").view("<u4").astype(np.uint64)
        value ^= _mix64(intensity ^ np.uint64(0xA24BAED4963EE407))
    mixed = _mix64(value)
    return ((mixed >> np.uint64(11)).astype(np.float64)) * (1.0 / (1 << 53))


def pointwise_keep_mask(
    records: np.ndarray,
    retention_grid: DensityRetentionGrid,
    *,
    seed: int,
) -> tuple[np.ndarray, np.ndarray]:
    xy = np.column_stack((records["x"], records["y"])).astype(np.float64, copy=False)
    probability = retention_grid.query(xy)
    keep = stable_point_uniform(records, seed=seed) < probability
    return keep, probability


def _records_equal(first: np.ndarray, second: np.ndarray) -> np.ndarray:
    if first.dtype != second.dtype or first.shape != second.shape:
        raise ValueError("record equality requires equal dtype and shape")
    item = first.dtype.itemsize
    return first.view(np.dtype((np.void, item))).reshape(-1) == second.view(
        np.dtype((np.void, item))
    ).reshape(-1)


@dataclass(frozen=True)
class RemovedSubsequence:
    pre_info: density.PlyInfo
    post_info: density.PlyInfo
    pre_indices: np.ndarray
    records: np.ndarray
    total_removed_records: int
    total_removed_water_records: int
    reviewed_removed_water_records: int
    per_region_reviewed: Mapping[str, int]


def collect_reviewed_final_blocker_rejections(
    pre_allterrain_path: str | Path,
    post_allterrain_path: str | Path,
    regions: Sequence[ReviewRegion],
    *,
    water_scan_id: float = WATER_SCAN_ID,
    search_window: int = 4096,
) -> RemovedSubsequence:
    """Collect exact WATER records deleted by a subsequence-only blocker pass.

    ``post_allterrain`` must be an exact, order-preserving subsequence of
    ``pre_allterrain``.  Any rewrite, reorder, or insertion is a hard failure.
    Only deleted WATER records inside one of ``regions`` are materialised.
    """

    if not regions:
        raise ValueError("at least one interface review region is required")
    if search_window <= 0:
        raise ValueError("search_window must be positive")
    pre_info = density.inspect_fixed_stride_ply(pre_allterrain_path)
    post_info = density.inspect_fixed_stride_ply(post_allterrain_path)
    if pre_info.dtype != post_info.dtype:
        raise RuntimeError("pre/post all-terrain schemas differ")
    if post_info.count > pre_info.count:
        raise RuntimeError("post all-terrain source cannot contain more records")
    if "scalar_ScanID" not in (pre_info.dtype.names or ()):
        raise RuntimeError("preselected source has no scalar_ScanID")
    pre = np.memmap(pre_info.path, dtype=pre_info.dtype, mode="r", offset=pre_info.offset, shape=(pre_info.count,))
    post = np.memmap(post_info.path, dtype=post_info.dtype, mode="r", offset=post_info.offset, shape=(post_info.count,))
    selected_indices: list[np.ndarray] = []
    selected_records: list[np.ndarray] = []
    per_region = {region.id: 0 for region in regions}
    total_removed = 0
    total_water = 0

    def inspect_removed(begin: int, end: int) -> None:
        nonlocal total_removed, total_water
        if end <= begin:
            return
        block = pre[begin:end]
        total_removed += len(block)
        is_water = block["scalar_ScanID"] == water_scan_id
        total_water += int(np.count_nonzero(is_water))
        if not np.any(is_water):
            return
        local_water = np.flatnonzero(is_water)
        xy = np.column_stack((block["x"][local_water], block["y"][local_water])).astype(np.float64, copy=False)
        reviewed = _any_region_mask(xy, regions)
        if not np.any(reviewed):
            return
        accepted_local = local_water[reviewed]
        accepted_xy = xy[reviewed]
        selected_indices.append(accepted_local.astype(np.int64) + begin)
        selected_records.append(np.asarray(block[accepted_local]).copy())
        for region in regions:
            per_region[region.id] += int(np.count_nonzero(region.mask(accepted_xy)))

    i = 0
    j = 0
    try:
        while j < post_info.count:
            # The blocker removes only about one point per several hundred in
            # the real v10 streams.  A bounded comparison window avoids
            # repeatedly comparing a million-record suffix after each sparse
            # deletion while retaining vectorised exact-byte checks.
            span = min(search_window, pre_info.count - i, post_info.count - j)
            if span <= 0:
                raise RuntimeError("post all-terrain PLY is not a subsequence of preselected PLY")
            equal = _records_equal(pre[i : i + span], post[j : j + span])
            if np.all(equal):
                i += span
                j += span
                continue
            prefix = int(np.flatnonzero(~equal)[0])
            i += prefix
            j += prefix
            # At a mismatch the next post record must occur later in pre.
            window = int(search_window)
            match: int | None = None
            while i + 1 < pre_info.count:
                end = min(pre_info.count, i + window)
                target = np.broadcast_to(post[j : j + 1], (end - i,))
                candidates = np.flatnonzero(_records_equal(pre[i:end], target))
                if len(candidates):
                    match = i + int(candidates[0])
                    break
                if end == pre_info.count:
                    break
                window = min(pre_info.count - i, window * 2)
            if match is None:
                raise RuntimeError(
                    "post all-terrain PLY is not an exact order-preserving subsequence"
                )
            inspect_removed(i, match)
            i = match
        inspect_removed(i, pre_info.count)
    finally:
        del pre, post
    expected_removed = pre_info.count - post_info.count
    if total_removed != expected_removed:
        raise RuntimeError(
            f"subsequence difference counted {total_removed}, expected {expected_removed}"
        )
    records = (
        np.concatenate(selected_records)
        if selected_records
        else np.empty(0, dtype=pre_info.dtype)
    )
    indices = np.concatenate(selected_indices) if selected_indices else np.empty(0, np.int64)
    return RemovedSubsequence(
        pre_info=pre_info,
        post_info=post_info,
        pre_indices=indices,
        records=records,
        total_removed_records=total_removed,
        total_removed_water_records=total_water,
        reviewed_removed_water_records=len(records),
        per_region_reviewed=per_region,
    )


def _collect_records_in_regions(
    path: str | Path,
    *,
    regions: Sequence[ReviewRegion],
    margin_m: float = 0.0,
    fields_only: Sequence[str] | None = None,
    scan_id: float | None = None,
    chunk_size: int = 1_000_000,
) -> tuple[density.PlyInfo, np.ndarray, np.ndarray]:
    if not regions:
        raise ValueError("at least one collection region is required")
    expanded = tuple(
        # Support collars deliberately expand beyond any evidence polygon;
        # the original polygon still gates the candidate itself.
        ReviewRegion(region.id, density.expand_bbox(region.bbox, margin_m), None)
        for region in regions
    )
    info = density.inspect_fixed_stride_ply(path)
    required = {"x", "y", "z"}
    if scan_id is not None:
        required.add("scalar_ScanID")
    missing = sorted(required - set(info.dtype.names or ()))
    if missing:
        raise RuntimeError(f"source lacks fields {missing}: {info.path}")
    if fields_only is not None:
        missing = sorted(set(fields_only) - set(info.dtype.names or ()))
        if missing:
            raise RuntimeError(f"source lacks requested fields {missing}: {info.path}")
    index_parts: list[np.ndarray] = []
    record_parts: list[np.ndarray] = []
    for begin, records in density.iter_ply_chunks(info.path, info=info, chunk_size=chunk_size):
        xy = np.column_stack((records["x"], records["y"])).astype(np.float64, copy=False)
        keep = _any_region_mask(xy, expanded)
        if scan_id is not None:
            keep &= records["scalar_ScanID"] == scan_id
        local = np.flatnonzero(keep)
        if not len(local):
            continue
        index_parts.append(local.astype(np.int64) + begin)
        if fields_only is None:
            record_parts.append(np.asarray(records[local]).copy())
        else:
            projected = np.empty(len(local), dtype=np.dtype([(name, info.dtype[name]) for name in fields_only]))
            for name in fields_only:
                projected[name] = records[name][local]
            record_parts.append(projected)
    indices = np.concatenate(index_parts) if index_parts else np.empty(0, np.int64)
    output_dtype = info.dtype if fields_only is None else np.dtype([(name, info.dtype[name]) for name in fields_only])
    records = np.concatenate(record_parts) if record_parts else np.empty(0, dtype=output_dtype)
    return info, indices, records


def _query_knn(support_xyz: np.ndarray, query_xyz: np.ndarray, count: int) -> tuple[np.ndarray, np.ndarray]:
    if len(support_xyz) == 0:
        return (
            np.full((len(query_xyz), count), np.inf, np.float64),
            np.full((len(query_xyz), count), -1, np.int64),
        )
    actual = min(int(count), len(support_xyz))
    try:
        from scipy.spatial import cKDTree
    except ModuleNotFoundError:
        cKDTree = None
    if cKDTree is not None:
        distance_value, index_value = cKDTree(support_xyz).query(query_xyz, k=actual, workers=-1)
        distance_value = np.asarray(distance_value, np.float64)
        index_value = np.asarray(index_value, np.int64)
        if actual == 1:
            distance_value = distance_value[:, None]
            index_value = index_value[:, None]
    else:
        distance_value = np.empty((len(query_xyz), actual), np.float64)
        index_value = np.empty((len(query_xyz), actual), np.int64)
        max_rows = max(1, int((64 * 1024 * 1024) // max(24 * len(support_xyz), 24)))
        for begin in range(0, len(query_xyz), max_rows):
            end = min(begin + max_rows, len(query_xyz))
            delta = query_xyz[begin:end, None, :] - support_xyz[None, :, :]
            squared = np.sum(np.square(delta), axis=2)
            order = np.argsort(squared, axis=1, kind="stable")[:, :actual]
            index_value[begin:end] = order
            distance_value[begin:end] = np.sqrt(np.take_along_axis(squared, order, axis=1))
    if actual == count:
        return distance_value, index_value
    distance_full = np.full((len(query_xyz), count), np.inf, np.float64)
    index_full = np.full((len(query_xyz), count), -1, np.int64)
    distance_full[:, :actual] = distance_value
    index_full[:, :actual] = index_value
    return distance_full, index_full


@dataclass(frozen=True)
class RecoveryCandidateAudit:
    pre_index: int
    region_id: str
    accepted: bool
    reason: str
    terrain_distance_m: float
    nearest_water_distance_m: float
    support_water_distance_m: float
    donor_current_index: int
    retention_probability: float
    retention_uniform: float


@dataclass(frozen=True)
class ValidatedRecovery:
    records: np.ndarray
    pre_indices: np.ndarray
    audits: tuple[RecoveryCandidateAudit, ...]
    accepted_per_region: Mapping[str, int]
    scalar_provenance: str


def validate_recovery_candidates(
    removed: RemovedSubsequence,
    current_water_path: str | Path,
    terrain_paths: Sequence[str | Path],
    regions: Sequence[ReviewRegion],
    *,
    nominal_spacing_m: float,
    relaxed_terrain_ratio: float = 0.65,
    duplicate_clearance_ratio: float = 0.90,
    maximum_bridge_m: float = 0.030,
    minimum_water_support: int = 3,
    retention_grid: DensityRetentionGrid | None = None,
    thinning_rejected_index_path: str | Path | None = None,
    thinning_seed: int = density.DEFAULT_SEED,
    chunk_size: int = 1_000_000,
) -> ValidatedRecovery:
    """Apply 3-D terrain, density, duplicate, and final-WATER support gates.

    When a thinning sidecar is supplied, support and scalar donors are drawn
    only from current WATER records that survive that exact v11 thinning
    decision.  Recovery candidates must independently pass the same
    pointwise retention field, preventing interface restoration from undoing
    the eastward density taper.
    """

    nominal = float(nominal_spacing_m)
    relaxed = nominal * float(relaxed_terrain_ratio)
    duplicate = nominal * float(duplicate_clearance_ratio)
    if not 0.0 < relaxed < nominal:
        raise ValueError("relaxed terrain clearance must lie inside the nominal blocker band")
    if not 0.0 < duplicate <= nominal:
        raise ValueError("duplicate clearance must lie in (0, nominal]")
    if maximum_bridge_m <= nominal or minimum_water_support <= 0:
        raise ValueError("invalid WATER support gate")
    current_info = density.inspect_fixed_stride_ply(current_water_path)
    output_dtype = current_info.dtype
    accepted_records: list[np.ndarray] = []
    accepted_indices: list[int] = []
    audits: list[RecoveryCandidateAudit] = []
    accepted_per_region = {region.id: 0 for region in regions}
    if not len(removed.records):
        return ValidatedRecovery(
            records=np.empty(0, dtype=output_dtype),
            pre_indices=np.empty(0, np.int64),
            audits=(),
            accepted_per_region=accepted_per_region,
            scalar_provenance=(
                "exact-pre-allterrain-record"
                if removed.pre_info.dtype == output_dtype
                else "exact-pre-allterrain-common-fields-plus-nearest-v10-water-analysis-fields"
            ),
        )

    # Scan each tens-of-millions source only once.  The point sets retained in
    # memory are the union of narrow, independently expanded interface ROIs,
    # not the much larger bounding rectangle spanning all review marks.
    _, water_source_indices, water_records = _collect_records_in_regions(
        current_water_path,
        regions=regions,
        margin_m=maximum_bridge_m + nominal,
        scan_id=WATER_SCAN_ID,
        chunk_size=chunk_size,
    )
    if thinning_rejected_index_path is not None and len(water_source_indices):
        rejected = _raw_rejected_indices(thinning_rejected_index_path)
        position = np.searchsorted(rejected, water_source_indices)
        rejected_here = np.zeros(len(water_source_indices), bool)
        valid_position = position < len(rejected)
        rejected_here[valid_position] = (
            rejected[position[valid_position]]
            == water_source_indices[valid_position]
        )
        water_source_indices = water_source_indices[~rejected_here]
        water_records = water_records[~rejected_here]
        del rejected
    terrain_records: list[np.ndarray] = []
    for path in terrain_paths:
        _, _, records = _collect_records_in_regions(
            path,
            regions=regions,
            margin_m=nominal,
            fields_only=("x", "y", "z"),
            chunk_size=chunk_size,
        )
        if len(records):
            terrain_records.append(records)
    water_xyz = np.column_stack(
        (water_records["x"], water_records["y"], water_records["z"])
    ).astype(np.float64, copy=False)
    terrain_joined = (
        np.concatenate(terrain_records)
        if terrain_records
        else np.empty(0, dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4")])
    )
    terrain_xyz = np.column_stack(
        (terrain_joined["x"], terrain_joined["y"], terrain_joined["z"])
    ).astype(np.float64, copy=False)
    candidates = removed.records
    candidate_xyz = np.column_stack(
        (candidates["x"], candidates["y"], candidates["z"])
    ).astype(np.float64, copy=False)
    candidate_xy = candidate_xyz[:, :2]
    if retention_grid is None:
        candidate_retention = np.ones(len(candidates), np.float64)
        candidate_uniform = np.zeros(len(candidates), np.float64)
        density_keep = np.ones(len(candidates), bool)
    else:
        candidate_retention = retention_grid.query(candidate_xy)
        candidate_uniform = stable_point_uniform(candidates, seed=thinning_seed)
        density_keep = candidate_uniform < candidate_retention
    region_ids = np.full(len(candidate_xy), "", dtype=object)
    for region in regions:
        unassigned = region_ids == ""
        region_ids[unassigned & region.mask(candidate_xy)] = region.id
    if np.any(region_ids == ""):
        raise RuntimeError("reviewed recovery candidate escaped every interface region")
    terrain_distance, _ = _query_knn(terrain_xyz, candidate_xyz, 1)
    water_distance, water_index = _query_knn(
        water_xyz, candidate_xyz, minimum_water_support
    )
    for local, source_record in enumerate(candidates):
        pre_index = int(removed.pre_indices[local])
        region_id = str(region_ids[local])
        terrain_nearest = float(terrain_distance[local, 0])
        water_nearest = float(water_distance[local, 0])
        support_distance = float(water_distance[local, minimum_water_support - 1])
        donor_local = int(water_index[local, 0])
        donor_source = int(water_source_indices[donor_local]) if donor_local >= 0 else -1
        reason = "accepted"
        accepted = True
        if not density_keep[local]:
            accepted, reason = False, "density-taper-rejected"
        elif not np.isfinite(terrain_nearest):
            accepted, reason = False, "no-terrain-support"
        elif terrain_nearest + 1.0e-9 < relaxed:
            accepted, reason = False, "terrain-clearance-too-small"
        elif terrain_nearest >= nominal + 1.0e-7:
            accepted, reason = False, "not-in-final-blocker-clearance-band"
        elif water_nearest + 1.0e-9 < duplicate:
            accepted, reason = False, "duplicate-clearance-too-small"
        elif not np.isfinite(support_distance) or support_distance > maximum_bridge_m:
            accepted, reason = False, "insufficient-surviving-water-support"
        elif donor_local < 0:
            accepted, reason = False, "missing-scalar-donor"
        if accepted:
            if removed.pre_info.dtype == output_dtype:
                output = np.asarray(source_record).copy()
            else:
                output = np.asarray(water_records[donor_local]).copy()
                for name in set(removed.pre_info.dtype.names or ()) & set(output_dtype.names or ()):
                    output[name] = source_record[name]
            accepted_records.append(output.reshape(1))
            accepted_indices.append(pre_index)
            accepted_per_region[region_id] += 1
        audits.append(
            RecoveryCandidateAudit(
                pre_index=pre_index,
                region_id=region_id,
                accepted=accepted,
                reason=reason,
                terrain_distance_m=terrain_nearest,
                nearest_water_distance_m=water_nearest,
                support_water_distance_m=support_distance,
                donor_current_index=donor_source,
                retention_probability=float(candidate_retention[local]),
                retention_uniform=float(candidate_uniform[local]),
            )
        )
    joined = np.concatenate(accepted_records) if accepted_records else np.empty(0, dtype=output_dtype)
    provenance = (
        "exact-pre-allterrain-record"
        if removed.pre_info.dtype == output_dtype
        else "exact-pre-allterrain-common-fields-plus-nearest-v10-water-analysis-fields"
    )
    return ValidatedRecovery(
        records=joined,
        pre_indices=np.asarray(accepted_indices, dtype=np.int64),
        audits=tuple(audits),
        accepted_per_region=accepted_per_region,
        scalar_provenance=provenance,
    )


@dataclass(frozen=True)
class ThinningAudit:
    source_points: int
    kept_points: int
    thinned_points: int
    probability_quantiles: Mapping[str, float]
    distance_bin_edges_m: tuple[float, ...]
    distance_bin_source_counts: tuple[int, ...]
    distance_bin_kept_counts: tuple[int, ...]
    rejected_index_path: str
    rejected_index_sha256: str


def _write_raw_index(handle, indices: np.ndarray) -> None:
    np.asarray(indices, dtype="<u8").tofile(handle)


def plan_pointwise_thinning(
    current_water_path: str | Path,
    retention_grid: DensityRetentionGrid,
    rejected_index_path: str | Path,
    *,
    seed: int,
    chunk_size: int = 1_000_000,
    overwrite: bool = False,
) -> ThinningAudit:
    """Stream deterministic thinning decisions to a compact raw-index audit."""

    output = density.assert_candidate_output_path(
        rejected_index_path,
        source_paths=[current_water_path],
    )
    source_info = density.inspect_fixed_stride_ply(current_water_path)
    if output.exists() and not overwrite:
        raise FileExistsError(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_name(f".{output.name}.{os.getpid()}.partial")
    temporary.unlink(missing_ok=True)
    source_count = kept_count = thinned_count = 0
    probability_samples: list[np.ndarray] = []
    # Finite sentinels keep the JSON audit strict (allow_nan=False).
    bin_edges = np.asarray([-1.0e30, 0.0, 0.25, 0.50, 1.0, 1.5, 2.0, 1.0e30], np.float64)
    bin_source = np.zeros(len(bin_edges) - 1, np.int64)
    bin_kept = np.zeros(len(bin_edges) - 1, np.int64)
    try:
        with temporary.open("wb") as index_handle:
            for begin, records in density.iter_ply_chunks(source_info.path, info=source_info, chunk_size=chunk_size):
                keep, probability = pointwise_keep_mask(records, retention_grid, seed=seed)
                local_rejected = np.flatnonzero(~keep)
                if len(local_rejected):
                    _write_raw_index(index_handle, local_rejected.astype(np.uint64) + begin)
                source_count += len(records)
                kept_count += int(np.count_nonzero(keep))
                thinned_count += len(local_rejected)
                # A deterministic stride sample is enough for distribution audit.
                probability_samples.append(probability[:: max(1, len(probability) // 4096)])
                xy = np.column_stack((records["x"], records["y"])).astype(np.float64, copy=False)
                xmin, xmax, ymin, ymax = retention_grid.bbox
                reviewed = (
                    (xy[:, 0] >= xmin)
                    & (xy[:, 0] <= xmax)
                    & (xy[:, 1] >= ymin)
                    & (xy[:, 1] <= ymax)
                )
                if np.any(reviewed):
                    signed = density.eastward_signed_distance(
                        xy[reviewed], retention_grid.guide_xy
                    ).signed_distance_m
                    bins = np.clip(
                        np.searchsorted(bin_edges, signed, side="right") - 1,
                        0,
                        len(bin_source) - 1,
                    )
                    bin_source += np.bincount(bins, minlength=len(bin_source))
                    bin_kept += np.bincount(bins[keep[reviewed]], minlength=len(bin_source))
        os.replace(temporary, output)
    except BaseException:
        temporary.unlink(missing_ok=True)
        raise
    sample = np.concatenate(probability_samples) if probability_samples else np.ones(1)
    return ThinningAudit(
        source_points=source_count,
        kept_points=kept_count,
        thinned_points=thinned_count,
        probability_quantiles={str(q): float(np.quantile(sample, q)) for q in (0.0, 0.05, 0.25, 0.5, 0.75, 0.95, 1.0)},
        distance_bin_edges_m=tuple(float(value) for value in bin_edges),
        distance_bin_source_counts=tuple(int(value) for value in bin_source),
        distance_bin_kept_counts=tuple(int(value) for value in bin_kept),
        rejected_index_path=str(output.resolve()),
        rejected_index_sha256=sha256_path(output),
    )


def _raw_rejected_indices(path: str | Path) -> np.ndarray:
    source = Path(path)
    if source.stat().st_size % 8:
        raise RuntimeError("raw rejected-index sidecar has invalid length")
    if source.stat().st_size == 0:
        return np.empty(0, dtype="<u8")
    return np.memmap(source, dtype="<u8", mode="r")


def _candidate_output_path(path: str | Path, protected: Iterable[str | Path]) -> Path:
    return density.assert_candidate_output_path(path, source_paths=protected)


def write_refined_candidate(
    current_water_path: str | Path,
    output_path: str | Path,
    retention_grid: DensityRetentionGrid,
    thinning: ThinningAudit,
    recovery: ValidatedRecovery,
    *,
    seed: int,
    comments: Iterable[str] = (),
    chunk_size: int = 1_000_000,
    overwrite: bool = False,
) -> int:
    """Write exact kept current records followed by validated recoveries."""

    current_info = density.inspect_fixed_stride_ply(current_water_path)
    if recovery.records.dtype != current_info.dtype:
        raise RuntimeError("validated recovery schema does not match current WATER")
    output = _candidate_output_path(output_path, [current_water_path, thinning.rejected_index_path])
    if output.exists() and not overwrite:
        raise FileExistsError(output)
    expected = thinning.kept_points + len(recovery.records)
    rejected = _raw_rejected_indices(thinning.rejected_index_path)
    if len(rejected) != thinning.thinned_points:
        raise RuntimeError("thinning index count disagrees with audit")
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_name(f".{output.name}.{os.getpid()}.partial")
    temporary.unlink(missing_ok=True)
    written = 0
    try:
        with temporary.open("wb") as handle:
            density._write_ply_header(
                handle,
                current_info.dtype,
                expected,
                [
                    "Scene1 v11 candidate-only WATER refinement",
                    "Surviving v10 records are byte-exact; no heights or scalars rewritten",
                    "Recoveries are exact pre-allterrain geometry with audited provenance",
                    *comments,
                ],
            )
            reject_cursor = 0
            for begin, records in density.iter_ply_chunks(current_info.path, info=current_info, chunk_size=chunk_size):
                end = begin + len(records)
                left = reject_cursor
                while left < len(rejected) and int(rejected[left]) < begin:
                    left += 1
                right = int(np.searchsorted(rejected, end, side="left"))
                local = np.asarray(rejected[left:right], np.int64) - begin
                keep = np.ones(len(records), dtype=bool)
                keep[local] = False
                # Re-evaluate to prove the raw sidecar and source still agree.
                recomputed, _ = pointwise_keep_mask(records, retention_grid, seed=seed)
                if not np.array_equal(keep, recomputed):
                    raise RuntimeError("source or thinning policy changed between plan and write")
                np.asarray(records[keep]).tofile(handle)
                written += int(np.count_nonzero(keep))
                reject_cursor = right
            if len(recovery.records):
                if not np.all(recovery.records["scalar_ScanID"] == WATER_SCAN_ID):
                    raise RuntimeError("recovery contains a non-WATER ScanID")
                recovery.records.tofile(handle)
                written += len(recovery.records)
        if written != expected:
            raise RuntimeError(f"candidate wrote {written}, expected {expected}")
        os.replace(temporary, output)
    except BaseException:
        temporary.unlink(missing_ok=True)
        raise
    finally:
        del rejected
    return written


def _atomic_json(path: Path, value: object, *, overwrite: bool) -> None:
    if path.exists() and not overwrite:
        raise FileExistsError(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.partial")
    temporary.unlink(missing_ok=True)
    try:
        with temporary.open("w", encoding="utf-8") as handle:
            json.dump(value, handle, indent=2, sort_keys=True, allow_nan=False)
            handle.write("\n")
        os.replace(temporary, path)
    except BaseException:
        temporary.unlink(missing_ok=True)
        raise


def _json_safe(value: object) -> object:
    """Replace non-finite audit metrics with null while preserving structure."""

    if isinstance(value, Mapping):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if isinstance(value, (float, np.floating)):
        number = float(value)
        return number if math.isfinite(number) else None
    if isinstance(value, (int, np.integer)):
        return int(value)
    return value


@dataclass(frozen=True)
class WaterPipelineResult:
    candidate_path: str
    candidate_points: int
    candidate_sha256: str
    audit_path: str
    recovery_audit_path: str
    rejected_index_path: str
    thinned_points: int
    recovered_points: int


def run_candidate_only_refinement(
    *,
    current_water_path: str | Path,
    pre_allterrain_path: str | Path,
    post_allterrain_path: str | Path,
    terrain_paths: Sequence[str | Path],
    config_path: str | Path,
    output_path: str | Path,
    audit_path: str | Path,
    recovery_audit_path: str | Path,
    rejected_index_path: str | Path,
    taper_bbox: Sequence[float],
    nominal_spacing_m: float,
    interface_mark_ids: Sequence[str] = DEFAULT_INTERFACE_MARK_IDS,
    cell_size_m: float = 0.05,
    smoothing_bandwidth_m: float = 0.20,
    taper_start_m: float = 0.0,
    taper_end_m: float = 1.50,
    floor_ratio: float = 0.08,
    relaxed_terrain_ratio: float = 0.65,
    duplicate_clearance_ratio: float = 0.90,
    maximum_bridge_m: float = 0.030,
    minimum_water_support: int = 3,
    seed: int = density.DEFAULT_SEED,
    chunk_size: int = 1_000_000,
    overwrite: bool = False,
) -> WaterPipelineResult:
    """Run one 2 mm or 5 mm candidate stage; never install a canonical file."""

    source_paths = [
        current_water_path,
        pre_allterrain_path,
        post_allterrain_path,
        *terrain_paths,
    ]
    config_source = Path(config_path)
    protected_paths = [*source_paths, config_source]
    output = _candidate_output_path(output_path, protected_paths)
    artifact_paths = [
        output,
        Path(audit_path),
        Path(recovery_audit_path),
        Path(rejected_index_path),
    ]
    resolved_artifacts = {
        path.expanduser().resolve(strict=False) for path in artifact_paths
    }
    if len(resolved_artifacts) != len(artifact_paths):
        raise ValueError("candidate and audit artifact paths must be distinct")
    for artifact in artifact_paths:
        density.assert_candidate_output_path(
            artifact,
            source_paths=protected_paths,
        )
        if artifact.exists() and not overwrite:
            raise FileExistsError(artifact)
    config_stat = config_source.stat()
    config_fingerprint = {
        "path": str(config_source.resolve()),
        "size_bytes": config_stat.st_size,
        "mtime_ns": config_stat.st_mtime_ns,
        "sha256": sha256_path(config_source),
    }
    implementation_fingerprints = {
        Path(__file__).name: sha256_path(__file__),
        Path(density.__file__).name: sha256_path(density.__file__),
        Path(confidence.__file__).name: sha256_path(confidence.__file__),
    }
    regions = load_interface_review_regions(config_source, mark_ids=interface_mark_ids)
    guide = load_taper_guide(config_source)
    source_fingerprints = {
        str(Path(path).resolve()): asdict(fingerprint_ply(path)) for path in source_paths
    }
    grid = build_measured_retention_grid(
        terrain_paths,
        current_water_path,
        bbox=taper_bbox,
        guide_xy=guide,
        cell_size_m=cell_size_m,
        smoothing_bandwidth_m=smoothing_bandwidth_m,
        taper_start_m=taper_start_m,
        taper_end_m=taper_end_m,
        floor_ratio=floor_ratio,
        chunk_size=chunk_size,
    )
    removed = collect_reviewed_final_blocker_rejections(
        pre_allterrain_path,
        post_allterrain_path,
        regions,
    )
    thinning = plan_pointwise_thinning(
        current_water_path,
        grid,
        rejected_index_path,
        seed=seed,
        chunk_size=chunk_size,
        overwrite=overwrite,
    )
    recovery = validate_recovery_candidates(
        removed,
        current_water_path,
        terrain_paths,
        regions,
        nominal_spacing_m=nominal_spacing_m,
        relaxed_terrain_ratio=relaxed_terrain_ratio,
        duplicate_clearance_ratio=duplicate_clearance_ratio,
        maximum_bridge_m=maximum_bridge_m,
        minimum_water_support=minimum_water_support,
        retention_grid=grid,
        thinning_rejected_index_path=rejected_index_path,
        thinning_seed=seed,
        chunk_size=chunk_size,
    )
    written = write_refined_candidate(
        current_water_path,
        output,
        grid,
        thinning,
        recovery,
        seed=seed,
        comments=[f"Nominal spacing {nominal_spacing_m:.6g} m"],
        chunk_size=chunk_size,
        overwrite=overwrite,
    )
    recovery_path = Path(recovery_audit_path)
    _atomic_json(
        recovery_path,
        {
            "schema_version": 1,
            "candidate_only": True,
            "eligibility": "exact pre_allterrain minus post_allterrain WATER subsequence",
            "review_regions_are_location_evidence_only": True,
            "support_is_post_thinning": True,
            "recovery_passes_pointwise_retention": True,
            "total_removed_records": removed.total_removed_records,
            "total_removed_water_records": removed.total_removed_water_records,
            "reviewed_removed_water_records": removed.reviewed_removed_water_records,
            "reviewed_per_region": dict(removed.per_region_reviewed),
            "accepted_per_region": dict(recovery.accepted_per_region),
            "accepted": len(recovery.records),
            "scalar_provenance": recovery.scalar_provenance,
            "records": _json_safe([asdict(item) for item in recovery.audits]),
        },
        overwrite=overwrite,
    )
    candidate_hash = sha256_path(output)
    for fingerprint in source_fingerprints.values():
        _assert_stat_unchanged(SourceFingerprint(**fingerprint))
    config_after = config_source.stat()
    if (
        config_after.st_size != config_fingerprint["size_bytes"]
        or config_after.st_mtime_ns != config_fingerprint["mtime_ns"]
        or sha256_path(config_source) != config_fingerprint["sha256"]
    ):
        raise RuntimeError("review config changed during WATER refinement")
    audit = {
        "schema_version": 1,
        "candidate_only": True,
        "canonical_install_performed": False,
        "candidate": {
            "path": str(output.resolve()),
            "points": written,
            "sha256": candidate_hash,
        },
        "sources": source_fingerprints,
        "config": config_fingerprint,
        "implementation": implementation_fingerprints,
        "parameters": {
            "taper_bbox": [float(value) for value in taper_bbox],
            "nominal_spacing_m": float(nominal_spacing_m),
            "interface_mark_ids": list(interface_mark_ids),
            "cell_size_m": float(cell_size_m),
            "smoothing_bandwidth_m": float(smoothing_bandwidth_m),
            "taper_start_m": float(taper_start_m),
            "taper_end_m": float(taper_end_m),
            "floor_ratio": float(floor_ratio),
            "relaxed_terrain_ratio": float(relaxed_terrain_ratio),
            "duplicate_clearance_ratio": float(duplicate_clearance_ratio),
            "maximum_bridge_m": float(maximum_bridge_m),
            "minimum_water_support": int(minimum_water_support),
            "seed": int(seed),
            "chunk_size": int(chunk_size),
        },
        "interface_mark_ids": list(interface_mark_ids),
        "annotations_are_not_fill_or_delete_masks": True,
        "density_reference": grid.summary(),
        "thinning": asdict(thinning),
        "recovery": {
            "accepted": len(recovery.records),
            "audit_path": str(recovery_path.resolve()),
            "audit_sha256": sha256_path(recovery_path),
            "relaxed_terrain_clearance_m": nominal_spacing_m * relaxed_terrain_ratio,
            "nominal_terrain_blocker_m": nominal_spacing_m,
            "duplicate_clearance_m": nominal_spacing_m * duplicate_clearance_ratio,
            "maximum_bridge_m": maximum_bridge_m,
            "minimum_water_support": minimum_water_support,
            "support_is_post_thinning": True,
            "recovery_passes_pointwise_retention": True,
        },
        "invariants": {
            "existing_survivors_byte_exact": True,
            "heights_or_scalars_of_existing_records_rewritten": False,
            "true_hole_synthesis_performed": False,
            "selection_uses_grid_cell_quota": False,
            "recovery_validated_against_final_surviving_water": True,
            "pointwise_hash_seed": int(seed),
        },
    }
    audit_output = Path(audit_path)
    _atomic_json(audit_output, audit, overwrite=overwrite)
    return WaterPipelineResult(
        candidate_path=str(output.resolve()),
        candidate_points=written,
        candidate_sha256=candidate_hash,
        audit_path=str(audit_output.resolve()),
        recovery_audit_path=str(recovery_path.resolve()),
        rejected_index_path=str(Path(rejected_index_path).resolve()),
        thinned_points=thinning.thinned_points,
        recovered_points=len(recovery.records),
    )


__all__ = [
    "DEFAULT_INTERFACE_MARK_IDS",
    "DensityRetentionGrid",
    "RecoveryCandidateAudit",
    "RemovedSubsequence",
    "ReviewRegion",
    "SourceFingerprint",
    "ThinningAudit",
    "ValidatedRecovery",
    "WaterPipelineResult",
    "build_measured_retention_grid",
    "collect_reviewed_final_blocker_rejections",
    "fingerprint_ply",
    "load_interface_review_regions",
    "load_taper_guide",
    "plan_pointwise_thinning",
    "pointwise_keep_mask",
    "run_candidate_only_refinement",
    "sha256_path",
    "stable_point_uniform",
    "validate_recovery_candidates",
    "write_refined_candidate",
]
