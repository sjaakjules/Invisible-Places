#!/usr/bin/env python3
"""Build reversible, candidate-only Site1 ROCK obstruction removals.

This module is intentionally narrower than the Scene1 rebuild orchestrator.  It
reads only the two configured obstruction review clips from immutable ROCK
clouds, derives three independent local terrain predictions, grows reviewed
3-D components from high residual seeds, requires bidirectional 1 mm/5 mm
agreement, validates terrain preservation, and writes separate candidate
bundles.  It has no API for installing or replacing canonical files.

The screenshot bboxes are locating evidence, never deletion masks.  A source
record is removed only when it belongs to a seed-connected elevated component,
does not touch the clip boundary, has an agreed local terrain surface, is not
ScanID 9, and has a corresponding independently elevated record at the other
resolution.  Exact removed records and original indices are archived by
``site1_v11_obstructions.write_candidate_archive`` and a byte-for-byte restore
hash is checked before the two-cloud run directory is published atomically.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass, replace
import hashlib
import json
import os
from pathlib import Path
import shutil
import tempfile
from typing import Iterable, Mapping, Sequence

import numpy as np

try:
    import site1_v11_obstructions as obstruction
except ModuleNotFoundError:  # pragma: no cover - package-style import fallback
    from scripts import site1_v11_obstructions as obstruction


_PLY_TYPES = {
    "char": "i1",
    "int8": "i1",
    "uchar": "u1",
    "uint8": "u1",
    "short": "<i2",
    "int16": "<i2",
    "ushort": "<u2",
    "uint16": "<u2",
    "int": "<i4",
    "int32": "<i4",
    "uint": "<u4",
    "uint32": "<u4",
    "int64": "<i8",
    "uint64": "<u8",
    "float": "<f4",
    "float32": "<f4",
    "double": "<f8",
    "float64": "<f8",
}


@dataclass(frozen=True)
class NamedPlyLayout:
    path: Path
    dtype: np.dtype
    vertex_count: int
    payload_offset: int
    header: bytes


@dataclass(frozen=True)
class ReviewTarget:
    target_id: str
    kind: str
    world_xy: tuple[float, float]
    bbox: tuple[float, float, float, float]
    evidence: str


@dataclass(frozen=True)
class ReviewClip:
    clip_id: str
    bbox: tuple[float, float, float, float]
    targets: tuple[ReviewTarget, ...]


@dataclass(frozen=True)
class ObstructionReviewSpec:
    config_path: Path
    config_sha256: str
    clips: tuple[ReviewClip, ...]


@dataclass(frozen=True)
class SurfaceParameters:
    anchor_cell_m: float = 0.025
    anchor_quantile: float = 0.15
    minimum_points_per_anchor: int = 3
    maximum_anchors: int = 30_000
    neighbour_count: int = 16
    query_chunk_points: int = 100_000
    huber_iterations: int = 8
    huber_scale_m: float = 0.012

    def __post_init__(self) -> None:
        if not np.isfinite(self.anchor_cell_m) or self.anchor_cell_m <= 0:
            raise ValueError("anchor_cell_m must be positive")
        if not 0.0 <= self.anchor_quantile <= 0.5:
            raise ValueError("anchor_quantile must lie in [0, 0.5]")
        for name in (
            "minimum_points_per_anchor",
            "maximum_anchors",
            "neighbour_count",
            "query_chunk_points",
            "huber_iterations",
        ):
            if int(getattr(self, name)) <= 0:
                raise ValueError(f"{name} must be positive")
        if not np.isfinite(self.huber_scale_m) or self.huber_scale_m <= 0:
            raise ValueError("huber_scale_m must be positive")


@dataclass(frozen=True)
class PipelineParameters:
    surface: SurfaceParameters = SurfaceParameters()
    thresholds: obstruction.ObstructionThresholds = obstruction.ObstructionThresholds()
    preservation: obstruction.TerrainPreservationThresholds = (
        obstruction.TerrainPreservationThresholds()
    )
    fine_voxel_m: float = 0.006
    coarse_voxel_m: float = 0.0125
    boundary_guard_m: float = 0.020
    core_inset_fraction: float = 0.12
    cross_scale_distance_m: float = 0.008
    preservation_cell_m: float = 0.025
    chunk_records: int = 1_000_000

    def __post_init__(self) -> None:
        for name in (
            "fine_voxel_m",
            "coarse_voxel_m",
            "boundary_guard_m",
            "cross_scale_distance_m",
            "preservation_cell_m",
        ):
            value = getattr(self, name)
            if not np.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be positive")
        if not 0.0 <= self.core_inset_fraction < 0.5:
            raise ValueError("core_inset_fraction must lie in [0, 0.5)")
        if int(self.chunk_records) <= 0:
            raise ValueError("chunk_records must be positive")


@dataclass(frozen=True)
class RoiCloud:
    source_path: Path
    source_sha256: str
    layout: NamedPlyLayout
    original_indices: np.ndarray
    xyz: np.ndarray
    scan_id: np.ndarray
    clip_index: np.ndarray
    core_mask: np.ndarray
    boundary_mask: np.ndarray


@dataclass(frozen=True)
class GroundAnchors:
    xy: np.ndarray
    z: np.ndarray
    support_count: np.ndarray


@dataclass(frozen=True)
class SurfacePrediction:
    model_names: tuple[str, str, str]
    heights_m: np.ndarray
    anchors: GroundAnchors


@dataclass(frozen=True)
class ConnectivityResult:
    core_connected: np.ndarray
    touches_boundary: np.ndarray
    component_label: np.ndarray
    component_count: int


@dataclass(frozen=True)
class ScaleAnalysis:
    name: str
    roi: RoiCloud
    prediction: SurfacePrediction
    candidate_mode: np.ndarray
    connectivity: ConnectivityResult
    classification: obstruction.ObstructionClassification

    @property
    def auto_xyz(self) -> np.ndarray:
        return self.roi.xyz[self.classification.auto_remove_mask]


@dataclass(frozen=True)
class PreservationAudit:
    passed: bool
    module_result: obstruction.TerrainPreservationResult
    outside_clip_removed_count: int
    scanid9_removed_count: int
    cross_scale_unmatched_count: int


@dataclass(frozen=True)
class RoundTripAudit:
    passed: bool
    reconstructed_sha256: str
    expected_source_sha256: str
    source_points: int
    candidate_points: int
    removed_points: int


@dataclass(frozen=True)
class ObstructionPipelineResult:
    output_dir: Path
    fine_bundle_dir: Path
    coarse_bundle_dir: Path
    manifest_path: Path
    fine_removed_count: int
    coarse_removed_count: int


def _sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def inspect_named_vertex_ply(path: str | Path) -> NamedPlyLayout:
    """Inspect the fixed-stride PLY and recover its named structured dtype."""

    basic = obstruction.inspect_binary_vertex_ply(path)
    fields: list[tuple[str, str]] = []
    current_element: str | None = None
    for line in basic.header.decode("ascii").splitlines():
        parts = line.split()
        if parts[:1] == ["element"]:
            current_element = parts[1]
        elif parts[:1] == ["property"] and current_element == "vertex":
            if len(parts) != 3 or parts[1] == "list" or parts[1] not in _PLY_TYPES:
                raise ValueError(f"unsupported vertex property in {path}: {line}")
            fields.append((parts[2], _PLY_TYPES[parts[1]]))
    dtype = np.dtype(fields)
    if dtype.itemsize != basic.record_stride:
        raise AssertionError("named PLY dtype disagrees with fixed record stride")
    required = {"x", "y", "z"}
    missing = sorted(required - set(dtype.names or ()))
    if missing:
        raise ValueError(f"{path}: missing coordinate fields {missing}")
    return NamedPlyLayout(
        path=basic.path,
        dtype=dtype,
        vertex_count=basic.vertex_count,
        payload_offset=basic.payload_offset,
        header=basic.header,
    )


def _as_bbox(values: Sequence[float], name: str) -> tuple[float, float, float, float]:
    array = np.asarray(values, dtype=np.float64)
    if array.shape != (4,) or not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must contain four finite values")
    xmin, xmax, ymin, ymax = map(float, array)
    if not xmin < xmax or not ymin < ymax:
        raise ValueError(f"{name} is not an ordered bbox")
    return xmin, xmax, ymin, ymax


def _bbox_contains(xy: np.ndarray, bbox: Sequence[float]) -> np.ndarray:
    xmin, xmax, ymin, ymax = bbox
    return (
        (xy[:, 0] >= xmin)
        & (xy[:, 0] <= xmax)
        & (xy[:, 1] >= ymin)
        & (xy[:, 1] <= ymax)
    )


def load_review_spec(config_path: str | Path) -> ObstructionReviewSpec:
    """Load only the written obstruction review contract from the v11 config."""

    path = Path(config_path).resolve(strict=True)
    raw = path.read_bytes()
    config = json.loads(raw)
    review = config.get("obstruction_review")
    if not isinstance(review, Mapping):
        raise ValueError("config has no obstruction_review object")
    marks = config.get("marked_locations", {}).get("image_1", [])
    by_id = {mark.get("id"): mark for mark in marks if isinstance(mark, Mapping)}
    target_ids = [
        "image_1_mark_5",
        "image_1_mark_6",
        "image_1_mark_7",
        "image_1_mark_8",
    ]
    targets: dict[str, ReviewTarget] = {}
    for target_id in target_ids:
        mark = by_id.get(target_id)
        if mark is None:
            raise ValueError(f"config is missing required obstruction mark {target_id}")
        world = np.asarray(mark.get("world"), dtype=np.float64)
        if world.shape != (2,) or not np.all(np.isfinite(world)):
            raise ValueError(f"{target_id} has no finite world coordinate")
        evidence = str(mark.get("screenshot_evidence", ""))
        kind = "person" if "person" in evidence.lower() else "bag"
        targets[target_id] = ReviewTarget(
            target_id=target_id,
            kind=kind,
            world_xy=(float(world[0]), float(world[1])),
            bbox=_as_bbox(mark.get("review_bbox"), f"{target_id}.review_bbox"),
            evidence=evidence,
        )
    southern = ReviewClip(
        clip_id="southern_people_and_bags",
        bbox=_as_bbox(review.get("southern_union_bbox"), "southern_union_bbox"),
        targets=tuple(targets[value] for value in target_ids[:3]),
    )
    northern = ReviewClip(
        clip_id="northern_two_people",
        bbox=_as_bbox(
            review.get("northern_people_group_bbox"),
            "northern_people_group_bbox",
        ),
        targets=(targets[target_ids[3]],),
    )
    for clip in (southern, northern):
        for target in clip.targets:
            x, y = target.world_xy
            if not (clip.bbox[0] <= x <= clip.bbox[1] and clip.bbox[2] <= y <= clip.bbox[3]):
                raise ValueError(f"{target.target_id} lies outside configured clip")
    return ObstructionReviewSpec(
        config_path=path,
        config_sha256=_sha256_bytes(raw),
        clips=(southern, northern),
    )


def _scan_field(dtype: np.dtype) -> str | None:
    names = set(dtype.names or ())
    for name in ("scalar_ScanID", "scan_id", "ScanID", "scanID"):
        if name in names:
            return name
    return None


def _core_mask_for_clip(
    xy: np.ndarray,
    clip: ReviewClip,
    inset_fraction: float,
) -> np.ndarray:
    result = np.zeros(len(xy), dtype=bool)
    for target in clip.targets:
        xmin, xmax, ymin, ymax = target.bbox
        dx = (xmax - xmin) * inset_fraction
        dy = (ymax - ymin) * inset_fraction
        inner = (xmin + dx, xmax - dx, ymin + dy, ymax - dy)
        result |= _bbox_contains(xy, inner)
    return result


def collect_configured_roi_points(
    source_path: str | Path,
    spec: ObstructionReviewSpec,
    *,
    source_sha256: str | None = None,
    core_inset_fraction: float = 0.12,
    boundary_guard_m: float = 0.020,
    chunk_records: int = 1_000_000,
) -> RoiCloud:
    """Materialise coordinates only from the configured local review clips."""

    if not 0 <= core_inset_fraction < 0.5:
        raise ValueError("core_inset_fraction must lie in [0, 0.5)")
    if boundary_guard_m <= 0 or chunk_records <= 0:
        raise ValueError("boundary guard and chunk_records must be positive")
    layout = inspect_named_vertex_ply(source_path)
    stat_before = layout.path.stat()
    identity_before = (
        stat_before.st_dev,
        stat_before.st_ino,
        stat_before.st_size,
        stat_before.st_mtime_ns,
    )
    observed_hash = obstruction.sha256_path(layout.path)
    if source_sha256 is not None and observed_hash != source_sha256.lower():
        raise RuntimeError("source hash changed before ROI extraction")
    scan_field = _scan_field(layout.dtype)
    if scan_field is None:
        raise RuntimeError(f"{layout.path}: no ScanID field; ScanID 9 cannot be protected")
    memory = np.memmap(
        layout.path,
        dtype=layout.dtype,
        mode="r",
        offset=layout.payload_offset,
        shape=(layout.vertex_count,),
    )
    index_parts: list[np.ndarray] = []
    xyz_parts: list[np.ndarray] = []
    scan_parts: list[np.ndarray] = []
    clip_parts: list[np.ndarray] = []
    core_parts: list[np.ndarray] = []
    boundary_parts: list[np.ndarray] = []
    try:
        for start in range(0, layout.vertex_count, int(chunk_records)):
            records = memory[start : start + int(chunk_records)]
            xy = np.column_stack((records["x"], records["y"])).astype(
                np.float64, copy=False
            )
            assigned = np.full(len(records), -1, dtype=np.int16)
            for clip_index, clip in enumerate(spec.clips):
                inside = _bbox_contains(xy, clip.bbox)
                assigned[(assigned < 0) & inside] = np.int16(clip_index)
            take = assigned >= 0
            if not np.any(take):
                continue
            local_xy = xy[take]
            local_clip = assigned[take]
            core = np.zeros(len(local_xy), dtype=bool)
            boundary = np.zeros(len(local_xy), dtype=bool)
            for clip_index, clip in enumerate(spec.clips):
                member = local_clip == clip_index
                if not np.any(member):
                    continue
                core[member] = _core_mask_for_clip(
                    local_xy[member], clip, core_inset_fraction
                )
                xmin, xmax, ymin, ymax = clip.bbox
                boundary[member] = (
                    (local_xy[member, 0] - xmin <= boundary_guard_m)
                    | (xmax - local_xy[member, 0] <= boundary_guard_m)
                    | (local_xy[member, 1] - ymin <= boundary_guard_m)
                    | (ymax - local_xy[member, 1] <= boundary_guard_m)
                )
            selected = records[take]
            xyz_parts.append(
                np.column_stack((selected["x"], selected["y"], selected["z"]))
                .astype(np.float64, copy=False)
                .copy()
            )
            index_parts.append(
                np.flatnonzero(take).astype(np.uint64) + np.uint64(start)
            )
            scan_parts.append(
                np.zeros(len(selected), dtype=np.float64)
                if scan_field is None
                else np.asarray(selected[scan_field], dtype=np.float64).copy()
            )
            clip_parts.append(local_clip.copy())
            core_parts.append(core)
            boundary_parts.append(boundary)
    finally:
        del memory
    stat_after = layout.path.stat()
    identity_after = (
        stat_after.st_dev,
        stat_after.st_ino,
        stat_after.st_size,
        stat_after.st_mtime_ns,
    )
    if identity_after != identity_before:
        raise RuntimeError("source identity changed during ROI extraction")
    if not index_parts:
        raise RuntimeError(f"no points from {layout.path} intersect configured clips")
    return RoiCloud(
        source_path=layout.path,
        source_sha256=observed_hash,
        layout=layout,
        original_indices=np.concatenate(index_parts),
        xyz=np.concatenate(xyz_parts),
        scan_id=np.concatenate(scan_parts),
        clip_index=np.concatenate(clip_parts),
        core_mask=np.concatenate(core_parts),
        boundary_mask=np.concatenate(boundary_parts),
    )


def build_lower_envelope_anchors(
    xyz: np.ndarray,
    donor_mask: np.ndarray | Sequence[bool],
    *,
    parameters: SurfaceParameters = SurfaceParameters(),
) -> GroundAnchors:
    """Build deterministic low-quantile XY anchors from non-core support."""

    points = np.asarray(xyz, dtype=np.float64)
    donors = np.asarray(donor_mask, dtype=bool)
    if points.ndim != 2 or points.shape[1] != 3 or donors.shape != (len(points),):
        raise ValueError("xyz must be Nx3 and donor_mask must have N entries")
    finite = donors & np.all(np.isfinite(points), axis=1)
    support = points[finite]
    if len(support) < max(12, parameters.minimum_points_per_anchor * 4):
        raise RuntimeError("insufficient finite non-core support for terrain models")
    origin = np.min(support[:, :2], axis=0)
    ij = np.floor((support[:, :2] - origin) / parameters.anchor_cell_m).astype(
        np.int64
    )
    width = int(np.max(ij[:, 0])) + 1
    cell_id = ij[:, 1] * width + ij[:, 0]
    order = np.argsort(cell_id, kind="stable")
    sorted_id = cell_id[order]
    starts = np.r_[0, np.flatnonzero(np.diff(sorted_id)) + 1]
    ends = np.r_[starts[1:], len(order)]
    xy_values: list[np.ndarray] = []
    z_values: list[float] = []
    counts: list[int] = []
    for begin, end in zip(starts, ends):
        count = int(end - begin)
        if count < parameters.minimum_points_per_anchor:
            continue
        cell = support[order[begin:end]]
        xy_values.append(np.median(cell[:, :2], axis=0))
        z_values.append(float(np.quantile(cell[:, 2], parameters.anchor_quantile)))
        counts.append(count)
    if len(xy_values) < 12:
        raise RuntimeError("fewer than 12 supported lower-envelope anchors")
    xy = np.asarray(xy_values, dtype=np.float64)
    z = np.asarray(z_values, dtype=np.float64)
    count_array = np.asarray(counts, dtype=np.int32)
    if len(xy) > parameters.maximum_anchors:
        take = np.linspace(
            0, len(xy) - 1, parameters.maximum_anchors, dtype=np.int64
        )
        xy, z, count_array = xy[take], z[take], count_array[take]
    return GroundAnchors(xy=xy, z=z, support_count=count_array)


def _quadratic_design(xy: np.ndarray, center: np.ndarray, scale: float) -> np.ndarray:
    local = (xy - center) / scale
    x, y = local[:, 0], local[:, 1]
    return np.column_stack((np.ones(len(x)), x, y, x * x, x * y, y * y))


def _robust_quadratic_prediction(
    anchors: GroundAnchors,
    query_xy: np.ndarray,
    parameters: SurfaceParameters,
) -> np.ndarray:
    center = np.median(anchors.xy, axis=0)
    spread = np.ptp(anchors.xy, axis=0)
    scale = max(float(np.max(spread)), parameters.anchor_cell_m)
    design = _quadratic_design(anchors.xy, center, scale)
    weights = np.sqrt(np.maximum(anchors.support_count, 1)).astype(np.float64)
    coefficients = np.linalg.lstsq(design * weights[:, None], anchors.z * weights, rcond=None)[0]
    for _ in range(parameters.huber_iterations):
        residual = anchors.z - design @ coefficients
        robust = np.minimum(1.0, parameters.huber_scale_m / np.maximum(np.abs(residual), 1.0e-12))
        total = weights * np.sqrt(robust)
        coefficients = np.linalg.lstsq(
            design * total[:, None], anchors.z * total, rcond=None
        )[0]
    return _quadratic_design(query_xy, center, scale) @ coefficients


def _nearest_anchor_indices(
    anchors_xy: np.ndarray,
    query_xy: np.ndarray,
    count: int,
) -> tuple[np.ndarray, np.ndarray]:
    k = min(int(count), len(anchors_xy))
    try:
        from scipy.spatial import cKDTree  # type: ignore

        distance, index = cKDTree(anchors_xy).query(query_xy, k=k, workers=-1)
        if k == 1:
            distance = distance[:, None]
            index = index[:, None]
        return np.asarray(distance), np.asarray(index, dtype=np.int64)
    except ImportError:
        # Tests and small review fixtures remain dependency-light.  Production
        # Site1 runs should use the repository's SciPy-enabled Python runtime.
        distance_parts: list[np.ndarray] = []
        index_parts: list[np.ndarray] = []
        for start in range(0, len(query_xy), 2_000):
            query = query_xy[start : start + 2_000]
            squared = np.sum(
                (query[:, None, :] - anchors_xy[None, :, :]) ** 2, axis=2
            )
            candidate = np.argpartition(squared, k - 1, axis=1)[:, :k]
            candidate_squared = np.take_along_axis(squared, candidate, axis=1)
            order = np.argsort(candidate_squared, axis=1)
            index = np.take_along_axis(candidate, order, axis=1)
            distance_parts.append(
                np.sqrt(np.take_along_axis(candidate_squared, order, axis=1))
            )
            index_parts.append(index)
        return np.concatenate(distance_parts), np.concatenate(index_parts)


def _idw_and_local_plane_predictions(
    anchors: GroundAnchors,
    query_xy: np.ndarray,
    parameters: SurfaceParameters,
) -> tuple[np.ndarray, np.ndarray]:
    idw_parts: list[np.ndarray] = []
    plane_parts: list[np.ndarray] = []
    for start in range(0, len(query_xy), parameters.query_chunk_points):
        query = query_xy[start : start + parameters.query_chunk_points]
        distance, index = _nearest_anchor_indices(
            anchors.xy, query, parameters.neighbour_count
        )
        neighbour_xy = anchors.xy[index]
        neighbour_z = anchors.z[index]
        weights = 1.0 / np.maximum(distance, parameters.anchor_cell_m * 0.20) ** 2
        idw = np.sum(weights * neighbour_z, axis=1) / np.sum(weights, axis=1)

        delta = neighbour_xy - query[:, None, :]
        design = np.concatenate(
            (np.ones((*delta.shape[:2], 1), dtype=np.float64), delta), axis=2
        )
        normal = np.einsum("nki,nk,nkj->nij", design, weights, design)
        rhs = np.einsum("nki,nk,nk->ni", design, weights, neighbour_z)
        normal[:, 0, 0] += 1.0e-10
        normal[:, 1, 1] += 1.0e-10
        normal[:, 2, 2] += 1.0e-10
        try:
            # The explicit trailing singleton keeps NumPy 2.x and 1.x batched
            # solve semantics identical (otherwise an (N, 3) RHS is treated
            # as a matrix rather than N vectors by newer releases).
            coefficients = np.linalg.solve(normal, rhs[..., None])[..., 0]
            plane = coefficients[:, 0]
        except np.linalg.LinAlgError:
            plane = idw.copy()
        invalid = ~np.isfinite(plane)
        plane[invalid] = idw[invalid]
        idw_parts.append(idw)
        plane_parts.append(plane)
    return np.concatenate(idw_parts), np.concatenate(plane_parts)


def fit_independent_surface_models(
    support_xyz: np.ndarray,
    donor_mask: np.ndarray | Sequence[bool],
    query_xy: np.ndarray,
    *,
    parameters: SurfaceParameters = SurfaceParameters(),
) -> SurfacePrediction:
    """Predict terrain with robust quadratic, local IDW, and local plane models."""

    query = np.asarray(query_xy, dtype=np.float64)
    if query.ndim != 2 or query.shape[1] != 2:
        raise ValueError("query_xy must have shape (N, 2)")
    anchors = build_lower_envelope_anchors(
        support_xyz, donor_mask, parameters=parameters
    )
    quadratic = _robust_quadratic_prediction(anchors, query, parameters)
    idw, plane = _idw_and_local_plane_predictions(anchors, query, parameters)
    return SurfacePrediction(
        model_names=("robust_quadratic", "lower_envelope_idw", "local_weighted_plane"),
        heights_m=np.column_stack((quadratic, idw, plane)),
        anchors=anchors,
    )


def _candidate_modes(
    point_z: np.ndarray,
    predictions: np.ndarray,
    thresholds: obstruction.ObstructionThresholds,
) -> np.ndarray:
    seed = obstruction.evaluate_surface_consensus(
        point_z,
        predictions,
        residual_threshold_m=thresholds.seed_height_m,
        minimum_models=thresholds.minimum_models,
        maximum_model_spread_m=thresholds.maximum_model_spread_m,
    )
    grow = obstruction.evaluate_surface_consensus(
        point_z,
        predictions,
        residual_threshold_m=thresholds.grow_height_m,
        minimum_models=thresholds.minimum_models,
        maximum_model_spread_m=thresholds.maximum_model_spread_m,
    )
    modes = np.full(len(point_z), np.uint8(obstruction.CandidateMode.NONE), dtype=np.uint8)
    modes[grow.above_threshold] = np.uint8(obstruction.CandidateMode.GROW)
    modes[seed.above_threshold] = np.uint8(obstruction.CandidateMode.SEED)
    return modes


def connect_candidate_components(
    xyz: np.ndarray,
    candidate_mode: np.ndarray | Sequence[int],
    core_seed_mask: np.ndarray | Sequence[bool],
    boundary_mask: np.ndarray | Sequence[bool],
    *,
    voxel_size_m: float,
) -> ConnectivityResult:
    """Grow candidate voxel components and retain only reviewed-core identities."""

    points = np.asarray(xyz, dtype=np.float64)
    modes = np.asarray(candidate_mode, dtype=np.uint8)
    core = np.asarray(core_seed_mask, dtype=bool)
    boundary = np.asarray(boundary_mask, dtype=bool)
    if modes.ndim == 0:
        modes = np.full(len(points), modes.item(), dtype=np.uint8)
    if core.ndim == 0:
        core = np.full(len(points), core.item(), dtype=bool)
    if boundary.ndim == 0:
        boundary = np.full(len(points), boundary.item(), dtype=bool)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("xyz must have shape (N, 3)")
    if modes.shape != (len(points),) or core.shape != modes.shape or boundary.shape != modes.shape:
        raise ValueError("candidate/core/boundary masks must match xyz")
    if not np.isfinite(voxel_size_m) or voxel_size_m <= 0:
        raise ValueError("voxel_size_m must be positive")
    active_index = np.flatnonzero(modes != int(obstruction.CandidateMode.NONE))
    labels = np.full(len(points), -1, dtype=np.int32)
    connected = np.zeros(len(points), dtype=bool)
    touches = np.zeros(len(points), dtype=bool)
    if not len(active_index):
        return ConnectivityResult(connected, touches, labels, 0)

    voxels = np.floor(points[active_index] / voxel_size_m).astype(np.int64)
    unique, inverse = np.unique(voxels, axis=0, return_inverse=True)
    voxel_lookup = {tuple(value): index for index, value in enumerate(unique)}
    adjacency: list[list[int]] = [[] for _ in range(len(unique))]
    offsets = [
        (dx, dy, dz)
        for dx in (-1, 0, 1)
        for dy in (-1, 0, 1)
        for dz in (-1, 0, 1)
        if (dx, dy, dz) != (0, 0, 0)
    ]
    for voxel_id, value in enumerate(unique):
        base = tuple(map(int, value))
        for offset in offsets:
            other = voxel_lookup.get(
                (base[0] + offset[0], base[1] + offset[1], base[2] + offset[2])
            )
            if other is not None and other > voxel_id:
                adjacency[voxel_id].append(other)
                adjacency[other].append(voxel_id)

    seed_voxel = np.zeros(len(unique), dtype=bool)
    point_seed = core[active_index] & (
        modes[active_index] == int(obstruction.CandidateMode.SEED)
    )
    np.logical_or.at(seed_voxel, inverse, point_seed)
    boundary_voxel = np.zeros(len(unique), dtype=bool)
    np.logical_or.at(boundary_voxel, inverse, boundary[active_index])
    voxel_component = np.full(len(unique), -1, dtype=np.int32)
    component_core: list[bool] = []
    component_boundary: list[bool] = []
    component = 0
    for root in range(len(unique)):
        if voxel_component[root] >= 0:
            continue
        stack = [root]
        voxel_component[root] = component
        has_core = False
        has_boundary = False
        while stack:
            current = stack.pop()
            has_core |= bool(seed_voxel[current])
            has_boundary |= bool(boundary_voxel[current])
            for neighbour in adjacency[current]:
                if voxel_component[neighbour] < 0:
                    voxel_component[neighbour] = component
                    stack.append(neighbour)
        component_core.append(has_core)
        component_boundary.append(has_boundary)
        component += 1
    point_component = voxel_component[inverse]
    labels[active_index] = point_component
    core_flags = np.asarray(component_core, dtype=bool)
    boundary_flags = np.asarray(component_boundary, dtype=bool)
    connected[active_index] = core_flags[point_component]
    touches[active_index] = boundary_flags[point_component]
    return ConnectivityResult(connected, touches, labels, component)


def _force_scanid9_protection(
    classification: obstruction.ObstructionClassification,
    scan_id: np.ndarray,
) -> obstruction.ObstructionClassification:
    scan9 = np.isfinite(scan_id) & np.isclose(scan_id, 9.0, atol=1.0e-4)
    disposition = classification.disposition.copy()
    reasons = classification.reason_mask.copy()
    disposition[scan9 & classification.auto_remove_mask] = np.uint8(
        obstruction.PointDisposition.REVIEW
    )
    reasons[scan9] |= np.uint32(obstruction.ObstructionReason.SCANID9_PROTECTED)
    return replace(classification, disposition=disposition, reason_mask=reasons)


def analyze_scale(
    name: str,
    roi: RoiCloud,
    *,
    voxel_size_m: float,
    parameters: PipelineParameters = PipelineParameters(),
) -> ScaleAnalysis:
    """Run independent local surface and component analysis for one resolution."""

    prediction_parts: list[tuple[np.ndarray, np.ndarray]] = []
    anchor_xy: list[np.ndarray] = []
    anchor_z: list[np.ndarray] = []
    anchor_counts: list[np.ndarray] = []
    for clip_index in np.unique(roi.clip_index):
        member = roi.clip_index == clip_index
        if not np.any(member):
            continue
        local = roi.xyz[member]
        donor = ~roi.core_mask[member]
        prediction = fit_independent_surface_models(
            local,
            donor,
            local[:, :2],
            parameters=parameters.surface,
        )
        prediction_parts.append((np.flatnonzero(member), prediction.heights_m))
        anchor_xy.append(prediction.anchors.xy)
        anchor_z.append(prediction.anchors.z)
        anchor_counts.append(prediction.anchors.support_count)
    heights = np.full((len(roi.xyz), 3), np.nan, dtype=np.float64)
    for index, values in prediction_parts:
        heights[index] = values
    combined_prediction = SurfacePrediction(
        model_names=("robust_quadratic", "lower_envelope_idw", "local_weighted_plane"),
        heights_m=heights,
        anchors=GroundAnchors(
            xy=np.concatenate(anchor_xy),
            z=np.concatenate(anchor_z),
            support_count=np.concatenate(anchor_counts),
        ),
    )
    modes = _candidate_modes(roi.xyz[:, 2], heights, parameters.thresholds)
    connectivity = connect_candidate_components(
        roi.xyz,
        modes,
        roi.core_mask,
        roi.boundary_mask,
        voxel_size_m=voxel_size_m,
    )
    classification = obstruction.classify_obstruction_points(
        roi.xyz[:, 2],
        heights,
        core_connected=connectivity.core_connected,
        candidate_mode=modes,
        touches_roi_boundary=connectivity.touches_boundary,
        collar_point=roi.boundary_mask,
        scan_id=roi.scan_id,
        thresholds=parameters.thresholds,
    )
    classification = _force_scanid9_protection(classification, roi.scan_id)
    return ScaleAnalysis(
        name=name,
        roi=roi,
        prediction=combined_prediction,
        candidate_mode=modes,
        connectivity=connectivity,
        classification=classification,
    )


def nearest_3d_distance(reference_xyz: np.ndarray, query_xyz: np.ndarray) -> np.ndarray:
    """Return exact nearest-reference distances, with a deterministic fallback."""

    reference = np.asarray(reference_xyz, dtype=np.float64)
    query = np.asarray(query_xyz, dtype=np.float64)
    if reference.ndim != 2 or reference.shape[1] != 3:
        raise ValueError("reference_xyz must have shape (N, 3)")
    if query.ndim != 2 or query.shape[1] != 3:
        raise ValueError("query_xyz must have shape (M, 3)")
    if not len(query):
        return np.empty(0, dtype=np.float64)
    if not len(reference):
        return np.full(len(query), np.inf, dtype=np.float64)
    try:
        from scipy.spatial import cKDTree  # type: ignore

        return np.asarray(cKDTree(reference).query(query, k=1, workers=-1)[0])
    except ImportError:
        output = np.empty(len(query), dtype=np.float64)
        for start in range(0, len(query), 2_000):
            squared = np.sum(
                (query[start : start + 2_000, None, :] - reference[None, :, :]) ** 2,
                axis=2,
            )
            output[start : start + len(squared)] = np.sqrt(np.min(squared, axis=1))
        return output


def _nearest_same_clip_distance(
    reference_xyz: np.ndarray,
    reference_clip: np.ndarray,
    query_xyz: np.ndarray,
    query_clip: np.ndarray,
) -> np.ndarray:
    """Nearest 3-D distance constrained to the same configured review clip."""

    reference_clip = np.asarray(reference_clip, dtype=np.int16)
    query_clip = np.asarray(query_clip, dtype=np.int16)
    if reference_clip.shape != (len(reference_xyz),):
        raise ValueError("reference clip IDs must match reference points")
    if query_clip.shape != (len(query_xyz),):
        raise ValueError("query clip IDs must match query points")
    result = np.full(len(query_xyz), np.inf, dtype=np.float64)
    for clip_id in np.unique(query_clip):
        query_member = query_clip == clip_id
        reference_member = reference_clip == clip_id
        result[query_member] = nearest_3d_distance(
            reference_xyz[reference_member], query_xyz[query_member]
        )
    return result


def _gate_classification(
    analysis: ScaleAnalysis,
    eligible_auto: np.ndarray,
    reason: obstruction.ObstructionReason,
) -> ScaleAnalysis:
    classification = analysis.classification
    auto = classification.auto_remove_mask
    if eligible_auto.shape != auto.shape:
        raise ValueError("cross-scale eligibility must match analysis")
    disposition = classification.disposition.copy()
    reasons = classification.reason_mask.copy()
    rejected = auto & ~eligible_auto
    disposition[rejected] = np.uint8(obstruction.PointDisposition.REVIEW)
    reasons[rejected] |= np.uint32(reason)
    return replace(
        analysis,
        classification=replace(
            classification,
            disposition=disposition,
            reason_mask=reasons,
        ),
    )


def enforce_bidirectional_cross_scale_agreement(
    fine: ScaleAnalysis,
    coarse: ScaleAnalysis,
    *,
    maximum_distance_m: float = 0.008,
) -> tuple[ScaleAnalysis, ScaleAnalysis]:
    """Require every automatic 1 mm and 5 mm removal to agree in 3-D."""

    if not np.isfinite(maximum_distance_m) or maximum_distance_m <= 0:
        raise ValueError("maximum_distance_m must be positive")
    fine_auto = fine.classification.auto_remove_mask
    coarse_auto = coarse.classification.auto_remove_mask
    fine_distance = _nearest_same_clip_distance(
        coarse.roi.xyz[coarse_auto],
        coarse.roi.clip_index[coarse_auto],
        fine.roi.xyz,
        fine.roi.clip_index,
    )
    fine_eligible = (~fine_auto) | (fine_distance <= maximum_distance_m)
    fine = _gate_classification(
        fine,
        fine_eligible,
        obstruction.ObstructionReason.FINE_SEED_MISSING_OR_TOO_FAR,
    )
    fine_auto_after_gate = fine.classification.auto_remove_mask
    coarse_distance = _nearest_same_clip_distance(
        fine.roi.xyz[fine_auto_after_gate],
        fine.roi.clip_index[fine_auto_after_gate],
        coarse.roi.xyz,
        coarse.roi.clip_index,
    )
    coarse_eligible = (~coarse_auto) | (coarse_distance <= maximum_distance_m)
    coarse = _gate_classification(
        coarse,
        coarse_eligible,
        obstruction.ObstructionReason.FINE_INDEPENDENT_RESIDUAL_FAILED,
    )
    # A coarse point removed above can invalidate fine points for which it was
    # the sole match.  One final fine gate makes the relation truly bilateral.
    coarse_auto_after_gate = coarse.classification.auto_remove_mask
    final_fine_distance = _nearest_same_clip_distance(
        coarse.roi.xyz[coarse_auto_after_gate],
        coarse.roi.clip_index[coarse_auto_after_gate],
        fine.roi.xyz,
        fine.roi.clip_index,
    )
    final_fine_eligible = (
        ~fine.classification.auto_remove_mask
        | (final_fine_distance <= maximum_distance_m)
    )
    fine = _gate_classification(
        fine,
        final_fine_eligible,
        obstruction.ObstructionReason.FINE_SEED_MISSING_OR_TOO_FAR,
    )
    return fine, coarse


def validate_scale_preservation(
    analysis: ScaleAnalysis,
    other_scale_auto_xyz: np.ndarray,
    *,
    parameters: PipelineParameters = PipelineParameters(),
) -> PreservationAudit:
    """Apply strict point, collar, cell, ScanID, and cross-scale guards."""

    classification = analysis.classification
    consensus = obstruction.evaluate_model_consensus(
        analysis.prediction.heights_m,
        minimum_models=parameters.thresholds.minimum_models,
        maximum_model_spread_m=parameters.thresholds.maximum_model_spread_m,
    )
    residual = analysis.roi.xyz[:, 2] - consensus.surface_height_m
    origin = np.min(analysis.roi.xyz[:, :2], axis=0)
    ij = np.floor(
        (analysis.roi.xyz[:, :2] - origin) / parameters.preservation_cell_m
    ).astype(np.int64)
    width = int(np.max(ij[:, 0])) + 1
    cell_id = ij[:, 1] * width + ij[:, 0]
    ground = consensus.has_consensus & np.isfinite(residual) & (
        residual <= parameters.preservation.ground_band_m
    )
    module_result = obstruction.validate_terrain_preservation(
        classification.disposition,
        residual,
        collar_mask=analysis.roi.boundary_mask,
        terrain_cell_ids=cell_id,
        well_supported_ground_mask=ground,
        thresholds=parameters.preservation,
    )
    removed = classification.auto_remove_mask
    outside = int(np.count_nonzero(removed & (analysis.roi.clip_index < 0)))
    scan9 = np.isfinite(analysis.roi.scan_id) & np.isclose(
        analysis.roi.scan_id, 9.0, atol=1.0e-4
    )
    scan9_removed = int(np.count_nonzero(removed & scan9))
    distances = nearest_3d_distance(
        np.asarray(other_scale_auto_xyz, dtype=np.float64),
        analysis.roi.xyz[removed],
    )
    unmatched = int(np.count_nonzero(distances > parameters.cross_scale_distance_m))
    passed = module_result.passed and not outside and not scan9_removed and not unmatched
    return PreservationAudit(
        passed=bool(passed),
        module_result=module_result,
        outside_clip_removed_count=outside,
        scanid9_removed_count=scan9_removed,
        cross_scale_unmatched_count=unmatched,
    )


def verify_round_trip_archive(
    bundle_dir: str | Path,
) -> RoundTripAudit:
    """Reconstruct the source byte stream from candidate+archive and hash it."""

    bundle = Path(bundle_dir)
    manifest = json.loads((bundle / "manifest.json").read_text())
    candidate = bundle / manifest["candidate"]["path"]
    removed = bundle / manifest["removed"]["records_path"]
    removed_indices = np.load(
        bundle / manifest["removed"]["indices_path"], allow_pickle=False
    )
    source_header = (bundle / manifest["source_header"]).read_bytes()
    candidate_layout = obstruction.inspect_binary_vertex_ply(candidate)
    removed_layout = obstruction.inspect_binary_vertex_ply(removed)
    source_count = int(manifest["source"]["points"])
    if candidate_layout.record_stride != removed_layout.record_stride:
        raise RuntimeError("candidate and removed archive schemas differ")
    if candidate_layout.vertex_count + removed_layout.vertex_count != source_count:
        raise RuntimeError("candidate plus archive count does not restore source count")
    if len(removed_indices) != removed_layout.vertex_count:
        raise RuntimeError("removed index count differs from removed record count")
    if len(removed_indices) and (
        np.any(removed_indices[1:] <= removed_indices[:-1])
        or int(removed_indices[-1]) >= source_count
    ):
        raise RuntimeError("removed indices are not strictly ordered source indices")

    stride = candidate_layout.record_stride
    candidate_records = np.memmap(
        candidate,
        dtype=np.dtype((np.void, stride)),
        mode="r",
        offset=candidate_layout.payload_offset,
        shape=(candidate_layout.vertex_count,),
    )
    removed_records = np.memmap(
        removed,
        dtype=np.dtype((np.void, stride)),
        mode="r",
        offset=removed_layout.payload_offset,
        shape=(removed_layout.vertex_count,),
    )
    digest = hashlib.sha256(source_header)
    candidate_cursor = 0
    removed_cursor = 0
    try:
        for start in range(0, source_count, 1_000_000):
            count = min(1_000_000, source_count - start)
            stop = start + count
            next_removed = int(np.searchsorted(removed_indices, stop, side="left"))
            local_removed = removed_indices[removed_cursor:next_removed].astype(np.int64) - start
            remove_mask = np.zeros(count, dtype=bool)
            remove_mask[local_removed] = True
            block = np.empty(count, dtype=np.dtype((np.void, stride)))
            kept_count = count - len(local_removed)
            block[~remove_mask] = candidate_records[
                candidate_cursor : candidate_cursor + kept_count
            ]
            block[remove_mask] = removed_records[removed_cursor:next_removed]
            digest.update(block.tobytes())
            candidate_cursor += kept_count
            removed_cursor = next_removed
    finally:
        del candidate_records
        del removed_records
    if candidate_cursor != candidate_layout.vertex_count or removed_cursor != len(removed_indices):
        raise RuntimeError("round-trip cursors did not consume every record")
    reconstructed = digest.hexdigest()
    expected = str(manifest["source"]["sha256"])
    return RoundTripAudit(
        passed=reconstructed == expected,
        reconstructed_sha256=reconstructed,
        expected_source_sha256=expected,
        source_points=source_count,
        candidate_points=candidate_layout.vertex_count,
        removed_points=removed_layout.vertex_count,
    )


def _audit_to_json(audit: PreservationAudit) -> dict[str, object]:
    result = audit.module_result
    return {
        "passed": audit.passed,
        "auto_removed_count": result.auto_removed_count,
        "invalid_auto_removed_count": result.invalid_auto_removed_count,
        "ground_band_removed_count": result.ground_band_removed_count,
        "collar_removed_count": result.collar_removed_count,
        "collar_point_count": result.collar_point_count,
        "collar_removed_fraction": result.collar_removed_fraction,
        "maximum_cell_ground_loss_fraction": result.maximum_cell_ground_loss_fraction,
        "failing_cell_ids": result.failing_cell_ids.tolist(),
        "outside_clip_removed_count": audit.outside_clip_removed_count,
        "scanid9_removed_count": audit.scanid9_removed_count,
        "cross_scale_unmatched_count": audit.cross_scale_unmatched_count,
    }


def build_obstruction_candidates(
    rock_1mm_path: str | Path,
    rock_5mm_path: str | Path,
    config_path: str | Path,
    output_dir: str | Path,
    *,
    parameters: PipelineParameters = PipelineParameters(),
) -> ObstructionPipelineResult:
    """Build and atomically publish two verified candidate-only ROCK bundles."""

    fine_source = Path(rock_1mm_path).resolve(strict=True)
    coarse_source = Path(rock_5mm_path).resolve(strict=True)
    destination = Path(output_dir)
    if destination.exists():
        raise FileExistsError(f"refusing to overwrite candidate run {destination}")
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.resolve(strict=False) in (fine_source, coarse_source):
        raise ValueError("output directory cannot be a source PLY")
    spec = load_review_spec(config_path)
    fine_roi = collect_configured_roi_points(
        fine_source,
        spec,
        core_inset_fraction=parameters.core_inset_fraction,
        boundary_guard_m=parameters.boundary_guard_m,
        chunk_records=parameters.chunk_records,
    )
    coarse_roi = collect_configured_roi_points(
        coarse_source,
        spec,
        core_inset_fraction=parameters.core_inset_fraction,
        boundary_guard_m=parameters.boundary_guard_m,
        chunk_records=parameters.chunk_records,
    )
    fine_hash = fine_roi.source_sha256
    coarse_hash = coarse_roi.source_sha256
    fine = analyze_scale(
        "ROCK-1mm", fine_roi, voxel_size_m=parameters.fine_voxel_m, parameters=parameters
    )
    coarse = analyze_scale(
        "ROCK-5mm", coarse_roi, voxel_size_m=parameters.coarse_voxel_m, parameters=parameters
    )
    fine, coarse = enforce_bidirectional_cross_scale_agreement(
        fine, coarse, maximum_distance_m=parameters.cross_scale_distance_m
    )
    fine_audit = validate_scale_preservation(
        fine, coarse.auto_xyz, parameters=parameters
    )
    coarse_audit = validate_scale_preservation(
        coarse, fine.auto_xyz, parameters=parameters
    )
    if not fine_audit.passed or not coarse_audit.passed:
        raise RuntimeError("terrain/cross-scale preservation validation failed closed")

    stage = Path(
        tempfile.mkdtemp(prefix=f".{destination.name}.staging-", dir=destination.parent)
    )
    try:
        fine_sparse = obstruction.SparsePointClassifications.from_classification(
            fine.roi.original_indices, fine.classification
        )
        coarse_sparse = obstruction.SparsePointClassifications.from_classification(
            coarse.roi.original_indices, coarse.classification
        )
        fine_archive = obstruction.write_candidate_archive(
            fine_source,
            stage / "rock-1mm",
            fine_sparse,
            expected_source_sha256=fine_hash,
            candidate_filename="Site1-ROCK-1mm.candidate.ply",
            chunk_records=parameters.chunk_records,
        )
        coarse_archive = obstruction.write_candidate_archive(
            coarse_source,
            stage / "rock-5mm",
            coarse_sparse,
            expected_source_sha256=coarse_hash,
            candidate_filename="Site1-ROCK-5mm.candidate.ply",
            chunk_records=parameters.chunk_records,
        )
        fine_round_trip = verify_round_trip_archive(fine_archive.bundle_dir)
        coarse_round_trip = verify_round_trip_archive(coarse_archive.bundle_dir)
        if not fine_round_trip.passed or not coarse_round_trip.passed:
            raise RuntimeError("candidate archive failed round-trip restoration")
        if obstruction.sha256_path(fine_source) != fine_hash:
            raise RuntimeError("ROCK-1mm source changed during candidate build")
        if obstruction.sha256_path(coarse_source) != coarse_hash:
            raise RuntimeError("ROCK-5mm source changed during candidate build")

        shutil.copy2(spec.config_path, stage / "review-config.json")
        manifest = {
            "schema": 1,
            "operation": "site1-v11-candidate-only-obstruction-pipeline",
            "canonical_writes": False,
            "config": {
                "source_path": str(spec.config_path),
                "sha256": spec.config_sha256,
                "archived_copy": "review-config.json",
            },
            "models": list(fine.prediction.model_names),
            "parameters": asdict(parameters),
            "implementation": {
                Path(__file__).name: obstruction.sha256_path(Path(__file__).resolve()),
                Path(obstruction.__file__).name: obstruction.sha256_path(
                    Path(obstruction.__file__).resolve()
                ),
            },
            "selection_contract": {
                "bbox_is_locating_evidence_only": True,
                "requires_seed_connected_3d_component": True,
                "minimum_agreeing_surface_models": parameters.thresholds.minimum_models,
                "maximum_model_spread_m": parameters.thresholds.maximum_model_spread_m,
                "seed_height_m": parameters.thresholds.seed_height_m,
                "grow_height_m": parameters.thresholds.grow_height_m,
                "ground_stop_height_m": parameters.thresholds.ground_stop_height_m,
                "cross_scale_distance_m": parameters.cross_scale_distance_m,
                "scanid9_absolute_protection": True,
            },
            "clips": [
                {
                    "id": clip.clip_id,
                    "bbox": list(clip.bbox),
                    "target_ids": [target.target_id for target in clip.targets],
                }
                for clip in spec.clips
            ],
            "fine": {
                "bundle": "rock-1mm",
                "source_sha256": fine_hash,
                "roi_points": len(fine.roi.xyz),
                "removed_points": fine_archive.removed_count,
                "surface_anchors": len(fine.prediction.anchors.xy),
                "preservation": _audit_to_json(fine_audit),
                "round_trip": fine_round_trip.__dict__,
            },
            "coarse": {
                "bundle": "rock-5mm",
                "source_sha256": coarse_hash,
                "roi_points": len(coarse.roi.xyz),
                "removed_points": coarse_archive.removed_count,
                "surface_anchors": len(coarse.prediction.anchors.xy),
                "preservation": _audit_to_json(coarse_audit),
                "round_trip": coarse_round_trip.__dict__,
            },
        }
        (stage / "manifest.json").write_text(
            json.dumps(manifest, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        if destination.exists():
            raise FileExistsError(
                f"candidate destination appeared during build: {destination}"
            )
        os.replace(stage, destination)
    except BaseException:
        if stage.exists():
            shutil.rmtree(stage)
        raise
    return ObstructionPipelineResult(
        output_dir=destination,
        fine_bundle_dir=destination / "rock-1mm",
        coarse_bundle_dir=destination / "rock-5mm",
        manifest_path=destination / "manifest.json",
        fine_removed_count=int(np.count_nonzero(fine.classification.auto_remove_mask)),
        coarse_removed_count=int(np.count_nonzero(coarse.classification.auto_remove_mask)),
    )


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Build reversible candidate-only Site1 ROCK obstruction removals"
    )
    parser.add_argument("--rock-1mm", required=True, type=Path)
    parser.add_argument("--rock-5mm", required=True, type=Path)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--chunk-records", type=int, default=1_000_000)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    parameters = replace(PipelineParameters(), chunk_records=args.chunk_records)
    result = build_obstruction_candidates(
        args.rock_1mm,
        args.rock_5mm,
        args.config,
        args.output,
        parameters=parameters,
    )
    print(
        json.dumps(
            {
                "output": str(result.output_dir),
                "fine_removed": result.fine_removed_count,
                "coarse_removed": result.coarse_removed_count,
                "canonical_writes": False,
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "ConnectivityResult",
    "GroundAnchors",
    "NamedPlyLayout",
    "ObstructionPipelineResult",
    "ObstructionReviewSpec",
    "PipelineParameters",
    "PreservationAudit",
    "ReviewClip",
    "ReviewTarget",
    "RoiCloud",
    "RoundTripAudit",
    "ScaleAnalysis",
    "SurfaceParameters",
    "SurfacePrediction",
    "analyze_scale",
    "build_lower_envelope_anchors",
    "build_obstruction_candidates",
    "collect_configured_roi_points",
    "connect_candidate_components",
    "enforce_bidirectional_cross_scale_agreement",
    "fit_independent_surface_models",
    "inspect_named_vertex_ply",
    "load_review_spec",
    "main",
    "nearest_3d_distance",
    "validate_scale_preservation",
    "verify_round_trip_archive",
]
