#!/usr/bin/env python3
"""Candidate-only Scene1 v11 terrain interstitial refinement pipeline.

The registered screenshot rectangles in ``site1_fossils_v11_review.json`` are
search windows, not fill masks.  The 1 mm terrain stage:

* measures SAND and ROCK independently and chooses the locally supported role;
* detects connected density deficits inside each registered search window;
* creates deterministic, irregular proposals only inside detected deficit cells;
* evaluates robust quadratic MLS, Delaunay-linear, and lower-boundary surfaces;
* applies the shared hard geometry/noise vetoes before blue-noise selection;
* requires SUPPORTED evidence for scanner/marked locations and STRONG evidence
  for crack locations;
* transfers colour/intensity/composition only from same-role, measured ScanID
  0--8 donors and stamps every accepted addition with ScanID 10;
* runs CleanMesh reduced analysis on local measured collars plus accepted
  additions, verifies every addition span, and appends the analysed additions.

The final 1 mm suffix and its hash-locked confidence/provenance archive are
then authoritative for 5 mm geometry.  Coarse additions are a deterministic,
maximal, vertically guarded exact-XYZ subset of those fine rows.  Their
properties come from same-role measured 5 mm donors and CleanMesh recomputes
their coarse scalar fields without changing geometry.  The pipeline writes
four separate candidate PLYs and detailed reversible cross-scale provenance.

Existing source records are never rewritten.  A caller may provide ROCK base
candidates produced by the obstruction pipeline; those exact records become
the immutable base of the resulting ROCK candidate.  This module deliberately
has no install operation, rejects canonical output names, hash-locks every
input, and refuses an already-existing destination directory.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass, field, replace
import hashlib
import io
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
from typing import Callable, Iterable, Mapping, Sequence
import zipfile

import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import site1_v11_terrain as terrain  # noqa: E402
import site1_v11_confidence as confidence  # noqa: E402
from site1_v11_confidence import (  # noqa: E402
    ConfidenceResult,
    ConfidenceThresholds,
    ConfidenceTier,
    evaluate_geometry_confidence,
)


DEFAULT_CONFIG = SCRIPT_DIR / "config" / "site1_fossils_v11_review.json"
DEFAULT_CLEANMESH = (
    SCRIPT_DIR.parent.parent / "CleanMesh" / "build-release"
    / "cleanmesh_reduced_analysis"
)
DEFAULT_SEED = 0x533156313154504C
PHYSICAL_METRICS = ("MeanCurvature", "CrossCurvature", "Recession", "Roughness")
DERIVED_SCALES = ("Fine", "Medium", "Broad")
COMBINED_WEIGHTS = np.asarray((0.45, 0.35, 0.20), np.float64)
LOCAL_VISIBILITY_FIELDS = (
    "scalar_A_R_Shelter_Lower",
    "scalar_A_R_RainExposure_Lower",
    "scalar_A_R_SVF_Lower",
)


def require_production_runtime() -> None:
    """Fail before a large run when indexed geometry support is unavailable."""

    try:
        from scipy.spatial import Delaunay, cKDTree  # noqa: F401
    except ImportError as error:
        raise RuntimeError(
            "Scene1 v11 terrain production requires SciPy cKDTree/Delaunay; "
            "run with /Users/juju/Documents/Repositories/CloudAlignment/"
            ".venv/bin/python"
        ) from error


def sha256_path(path: str | Path, *, block_size: int = 16 * 1024 * 1024) -> str:
    """Hash a file in bounded memory."""

    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        while True:
            block = handle.read(block_size)
            if not block:
                break
            digest.update(block)
    return digest.hexdigest()


@dataclass(frozen=True)
class SourceFingerprint:
    path: str
    size_bytes: int
    mtime_ns: int
    sha256: str
    points: int
    record_stride: int


def fingerprint_ply(path: str | Path) -> SourceFingerprint:
    layout = terrain.inspect_fixed_stride_ply(path)
    return SourceFingerprint(
        path=str(layout.path.resolve()),
        size_bytes=layout.size_bytes,
        mtime_ns=layout.mtime_ns,
        sha256=sha256_path(layout.path),
        points=layout.vertex_count,
        record_stride=layout.dtype.itemsize,
    )


def assert_fingerprint_unchanged(fingerprint: SourceFingerprint) -> None:
    path = Path(fingerprint.path)
    stat = path.stat()
    if stat.st_size != fingerprint.size_bytes or stat.st_mtime_ns != fingerprint.mtime_ns:
        raise RuntimeError(f"source stat changed during terrain build: {path}")
    if sha256_path(path) != fingerprint.sha256:
        raise RuntimeError(f"source hash changed during terrain build: {path}")


@dataclass(frozen=True)
class ResolutionParameters:
    label: str
    nominal_spacing_m: float
    deficit_cell_size_m: float = 0.025
    neighbourhood_radius_cells: int = 3
    minimum_expected_points: float = 3.0
    minimum_deficit_fraction: float = 0.40
    minimum_component_cells: int = 2
    support_radius_m: float = 0.050
    source_collar_m: float = 0.080
    cleanmesh_collar_m: float = 0.350
    property_donor_distance_m: float = 0.060
    proposal_oversampling: float = 1.25
    minimum_radius_ratio: float = 0.80
    maximum_radius_ratio: float = 4.0
    maximum_proposals_per_target: int = 300_000
    geometry_batch_points: int = 20_000
    reference_energy_samples: int = 2_048
    maximum_geometry_donors: int = 150_000
    maximum_energy_donors: int = 96
    cleanmesh_base_voxel_m: float = 0.003

    def __post_init__(self) -> None:
        if self.label not in {"1mm", "5mm"}:
            raise ValueError("resolution label must be 1mm or 5mm")
        positive = (
            self.nominal_spacing_m,
            self.deficit_cell_size_m,
            self.minimum_expected_points,
            self.support_radius_m,
            self.source_collar_m,
            self.cleanmesh_collar_m,
            self.property_donor_distance_m,
            self.proposal_oversampling,
            self.minimum_radius_ratio,
            self.maximum_radius_ratio,
            self.cleanmesh_base_voxel_m,
        )
        if not all(np.isfinite(value) and value > 0.0 for value in positive):
            raise ValueError("resolution distances/count thresholds must be positive")
        if not 0.0 < self.minimum_deficit_fraction <= 1.0:
            raise ValueError("minimum_deficit_fraction must lie in (0, 1]")
        if self.maximum_radius_ratio < self.minimum_radius_ratio:
            raise ValueError("maximum_radius_ratio must be >= minimum_radius_ratio")
        for name in (
            "neighbourhood_radius_cells",
            "minimum_component_cells",
            "maximum_proposals_per_target",
            "geometry_batch_points",
            "reference_energy_samples",
            "maximum_geometry_donors",
            "maximum_energy_donors",
        ):
            if int(getattr(self, name)) <= 0:
                raise ValueError(f"{name} must be positive")


@dataclass(frozen=True)
class PipelineParameters:
    fine: ResolutionParameters = field(
        default_factory=lambda: ResolutionParameters(
            label="1mm",
            nominal_spacing_m=0.0015,
            minimum_expected_points=8.0,
            minimum_deficit_fraction=0.38,
            support_radius_m=0.050,
            property_donor_distance_m=0.060,
        )
    )
    coarse: ResolutionParameters = field(
        default_factory=lambda: ResolutionParameters(
            label="5mm",
            nominal_spacing_m=0.005,
            minimum_expected_points=2.0,
            minimum_deficit_fraction=0.45,
            support_radius_m=0.060,
            property_donor_distance_m=0.075,
        )
    )
    confidence: ConfidenceThresholds = field(default_factory=ConfidenceThresholds)
    role_dominance_ratio: float = 1.12
    chunk_records: int = 1_000_000
    cleanmesh_tile_width_m: float = 4.0
    cleanmesh_chunk_points: int = 1_000_000
    cleanmesh_normalization_samples: int = 2_000_000
    global_normalization_samples: int = 2_000_000
    cross_scale_vertical_tolerance_m: float = 0.012
    cross_scale_distance_tolerance_m: float = 1.0e-9
    seed: int = DEFAULT_SEED

    def __post_init__(self) -> None:
        if self.fine.label != "1mm" or self.coarse.label != "5mm":
            raise ValueError("fine/coarse parameters must be labelled 1mm/5mm")
        self.confidence.validate()
        if not np.isfinite(self.role_dominance_ratio) or self.role_dominance_ratio < 1.0:
            raise ValueError("role_dominance_ratio must be finite and >= 1")
        for name in (
            "chunk_records",
            "cleanmesh_chunk_points",
            "cleanmesh_normalization_samples",
            "global_normalization_samples",
        ):
            if int(getattr(self, name)) <= 0:
                raise ValueError(f"{name} must be positive")
        if not np.isfinite(self.cleanmesh_tile_width_m) or self.cleanmesh_tile_width_m <= 0:
            raise ValueError("cleanmesh_tile_width_m must be positive")
        if (
            not np.isfinite(self.cross_scale_vertical_tolerance_m)
            or self.cross_scale_vertical_tolerance_m <= 0.0
        ):
            raise ValueError("cross_scale_vertical_tolerance_m must be positive")
        if (
            not np.isfinite(self.cross_scale_distance_tolerance_m)
            or self.cross_scale_distance_tolerance_m < 0.0
        ):
            raise ValueError("cross_scale_distance_tolerance_m must be non-negative")


@dataclass(frozen=True)
class LocalRoleCloud:
    role: str
    layout: terrain.PlyLayout
    original_indices: np.ndarray
    records: np.ndarray

    @property
    def measured_mask(self) -> np.ndarray:
        return terrain.measured_scan_mask(self.records)

    @property
    def measured_records(self) -> np.ndarray:
        return self.records[self.measured_mask]


@dataclass(frozen=True)
class RoleAssessment:
    role: str
    support_points: int
    occupied_fraction: float
    missing_points: float
    component_cells: int
    score: float
    deficit: terrain.DensityDeficitResult


@dataclass(frozen=True)
class RoleChoice:
    role: str | None
    reason: str
    assessments: tuple[RoleAssessment, ...]


@dataclass(frozen=True)
class DeficitProposals:
    xy: np.ndarray
    radius_m: np.ndarray
    priority: np.ndarray
    component_id: np.ndarray
    requested_before_cap: int
    proposal_cap_applied: bool


@dataclass(frozen=True)
class GeometryProposalEvaluation:
    candidate_xyz: np.ndarray
    predicted_normal: np.ndarray
    confidence: ConfidenceResult
    surface_heights_m: np.ndarray
    donor_sectors: np.ndarray
    normal_coherence: np.ndarray
    vertical_thickness_m: np.ndarray
    multimodality_score: np.ndarray
    residual_energy_ratio: np.ndarray


@dataclass(frozen=True)
class TargetBuildResult:
    target: terrain.TerrainReviewTarget
    role_choice: RoleChoice
    proposals: DeficitProposals
    accepted_records: np.ndarray | None
    accepted_role: str | None
    accepted_count: int
    audit: Mapping[str, object]
    accepted_radius_m: np.ndarray = field(
        default_factory=lambda: np.empty(0, np.float64)
    )
    accepted_priority: np.ndarray = field(
        default_factory=lambda: np.empty(0, np.float64)
    )
    accepted_confidence: ConfidenceResult = field(
        default_factory=lambda: ConfidenceResult(
            np.empty(0, np.uint32),
            np.empty(0, np.uint8),
            np.empty(0, np.float64),
            np.empty(0, np.uint8),
        )
    )
    accepted_candidate_index: np.ndarray = field(
        default_factory=lambda: np.empty(0, np.int64)
    )
    accepted_global_ledger_index: np.ndarray = field(
        default_factory=lambda: np.empty(0, np.int64)
    )
    accepted_primary_donor_index: np.ndarray = field(
        default_factory=lambda: np.empty(0, np.int64)
    )
    accepted_donor_distance_m: np.ndarray = field(
        default_factory=lambda: np.empty(0, np.float64)
    )
    accepted_donor_count: np.ndarray = field(
        default_factory=lambda: np.empty(0, np.int32)
    )


@dataclass(frozen=True)
class ResolutionBuildResult:
    label: str
    candidate_paths: Mapping[str, str]
    candidate_sha256: Mapping[str, str]
    addition_counts: Mapping[str, int]
    report_path: str
    report_sha256: str
    target_count: int
    addition_archive_paths: Mapping[str, str] = field(default_factory=dict)
    addition_archive_sha256: Mapping[str, str] = field(default_factory=dict)
    cross_scale_report_path: str | None = None
    cross_scale_report_sha256: str | None = None


@dataclass(frozen=True)
class TerrainPipelineResult:
    output_dir: Path
    manifest_path: Path
    resolutions: Mapping[str, ResolutionBuildResult]


CleanMeshRunner = Callable[
    [Path, Path, Path, Path, ResolutionParameters, PipelineParameters],
    Mapping[str, object],
]


def _expanded_bbox(
    bbox: Sequence[float], margin_m: float
) -> tuple[float, float, float, float]:
    xmin, xmax, ymin, ymax = (float(value) for value in bbox)
    return xmin - margin_m, xmax + margin_m, ymin - margin_m, ymax + margin_m


def _bbox_mask(xy: np.ndarray, bbox: Sequence[float]) -> np.ndarray:
    xmin, xmax, ymin, ymax = bbox
    return (
        (xy[:, 0] >= xmin)
        & (xy[:, 0] <= xmax)
        & (xy[:, 1] >= ymin)
        & (xy[:, 1] <= ymax)
    )


def _union_bbox_mask(
    xy: np.ndarray,
    bboxes: Iterable[Sequence[float]],
) -> np.ndarray:
    result = np.zeros(len(xy), dtype=bool)
    for bbox in bboxes:
        result |= _bbox_mask(xy, bbox)
    return result


def collect_local_role_cloud(
    path: str | Path,
    *,
    role: str,
    targets: Sequence[terrain.TerrainReviewTarget],
    collar_m: float,
    chunk_records: int = 1_000_000,
) -> LocalRoleCloud:
    """Collect only the union of target collars while preserving source indices."""

    role = str(role).upper()
    if role not in {"SAND", "ROCK"}:
        raise ValueError("terrain role must be SAND or ROCK")
    layout = terrain.inspect_fixed_stride_ply(path)
    names = set(layout.dtype.names or ())
    missing = sorted({"x", "y", "z", "nx", "ny", "nz"} - names)
    if missing:
        raise ValueError(f"{path}: missing terrain fields {missing}")
    bboxes = [_expanded_bbox(target.bbox, collar_m) for target in targets]
    index_pieces: list[np.ndarray] = []
    record_pieces: list[np.ndarray] = []
    for begin, chunk in terrain.iter_ply_chunks(layout, chunk_size=chunk_records):
        xy = np.column_stack((chunk["x"], chunk["y"])).astype(np.float64)
        selected = np.all(np.isfinite(xy), axis=1) & _union_bbox_mask(xy, bboxes)
        if np.any(selected):
            local = np.flatnonzero(selected)
            index_pieces.append(local.astype(np.int64) + int(begin))
            record_pieces.append(np.asarray(chunk[selected]).copy())
    indices = (
        np.concatenate(index_pieces)
        if index_pieces
        else np.empty(0, dtype=np.int64)
    )
    records = (
        np.concatenate(record_pieces)
        if record_pieces
        else np.empty(0, dtype=layout.dtype)
    )
    return LocalRoleCloud(role, layout, indices, records)


def _detect_target_deficit(
    measured_xy: np.ndarray,
    target: terrain.TerrainReviewTarget,
    parameters: ResolutionParameters,
) -> terrain.DensityDeficitResult:
    kwargs = {
        "cell_size_m": parameters.deficit_cell_size_m,
        "neighbourhood_radius_cells": parameters.neighbourhood_radius_cells,
        "minimum_expected_points": parameters.minimum_expected_points,
        "minimum_deficit_fraction": parameters.minimum_deficit_fraction,
        "minimum_component_cells": parameters.minimum_component_cells,
    }
    if target.kind is terrain.DeficitKind.SCANNER:
        return terrain.detect_scanner_footprint_deficit(
            measured_xy, target, **kwargs
        )
    if target.kind is terrain.DeficitKind.CRACK:
        return terrain.detect_crack_density_deficit(measured_xy, target, **kwargs)
    return terrain.detect_density_deficits(
        measured_xy,
        bbox=target.bbox,
        kind=terrain.DeficitKind.MARKED,
        **kwargs,
    )


def assess_target_role(
    cloud: LocalRoleCloud,
    target: terrain.TerrainReviewTarget,
    parameters: ResolutionParameters,
) -> RoleAssessment:
    measured = cloud.measured_records
    xy = (
        np.column_stack((measured["x"], measured["y"])).astype(np.float64)
        if len(measured)
        else np.empty((0, 2), dtype=np.float64)
    )
    deficit = _detect_target_deficit(xy, target, parameters)
    inside_count = int(np.count_nonzero(_bbox_mask(xy, target.bbox)))
    occupied_fraction = (
        float(np.count_nonzero(deficit.observed_count)) / deficit.observed_count.size
        if deficit.observed_count.size
        else 0.0
    )
    missing = float(sum(component.missing_points for component in deficit.components))
    component_cells = int(sum(component.cell_count for component in deficit.components))
    # A role is eligible only when it has measured local support *and* an
    # actual connected deficit.  The score favours boundary coverage and
    # missing support, not the raw screenshot rectangle area.
    score = 0.0
    if inside_count and component_cells:
        score = (
            math.log1p(inside_count)
            * max(occupied_fraction, 1.0 / deficit.observed_count.size)
            * math.log1p(max(missing, 1.0))
            * math.sqrt(component_cells)
        )
    return RoleAssessment(
        role=cloud.role,
        support_points=inside_count,
        occupied_fraction=occupied_fraction,
        missing_points=missing,
        component_cells=component_cells,
        score=score,
        deficit=deficit,
    )


def choose_supported_role(
    clouds: Mapping[str, LocalRoleCloud],
    target: terrain.TerrainReviewTarget,
    parameters: ResolutionParameters,
    *,
    dominance_ratio: float = 1.12,
) -> RoleChoice:
    """Choose SAND or ROCK from measured deficit-boundary support, fail closed."""

    assessments = tuple(
        assess_target_role(clouds[role], target, parameters)
        for role in ("SAND", "ROCK")
    )
    eligible = sorted(
        (item for item in assessments if item.score > 0.0),
        key=lambda item: (-item.score, item.role),
    )
    if not eligible:
        return RoleChoice(None, "no_connected_measured_density_deficit", assessments)
    if len(eligible) > 1 and eligible[0].score < eligible[1].score * dominance_ratio:
        return RoleChoice(None, "ambiguous_sand_rock_support", assessments)
    return RoleChoice(eligible[0].role, "dominant_measured_support", assessments)


def generate_irregular_deficit_proposals(
    deficit: terrain.DensityDeficitResult,
    *,
    nominal_spacing_m: float,
    oversampling: float = 1.25,
    minimum_radius_ratio: float = 0.80,
    maximum_radius_ratio: float = 4.0,
    maximum_proposals: int = 300_000,
    seed: int = DEFAULT_SEED,
) -> DeficitProposals:
    """Jitter proposals inside actual deficit cells, never over the whole bbox."""

    iy, ix = np.nonzero(deficit.candidate_mask)
    if not len(ix):
        return DeficitProposals(
            np.empty((0, 2), np.float64),
            np.empty(0, np.float64),
            np.empty(0, np.float64),
            np.empty(0, np.int32),
            0,
            False,
        )
    missing = np.maximum(
        deficit.expected_count[iy, ix] - deficit.observed_count[iy, ix], 0.0
    )
    count_per_cell = np.maximum(1, np.ceil(missing * oversampling).astype(np.int64))
    requested = int(np.sum(count_per_cell))
    cap_applied = requested > int(maximum_proposals)
    if cap_applied:
        scale = maximum_proposals / requested
        scaled = count_per_cell.astype(np.float64) * scale
        count_per_cell = np.floor(scaled).astype(np.int64)
        remainder = int(maximum_proposals - np.sum(count_per_cell))
        if remainder:
            fractional = scaled - count_per_cell
            order = np.lexsort((ix, iy, -fractional))
            count_per_cell[order[:remainder]] += 1
    active = count_per_cell > 0
    iy, ix, missing, count_per_cell = (
        values[active] for values in (iy, ix, missing, count_per_cell)
    )
    total = int(np.sum(count_per_cell))
    if not total:
        return DeficitProposals(
            np.empty((0, 2), np.float64),
            np.empty(0, np.float64),
            np.empty(0, np.float64),
            np.empty(0, np.int32),
            requested,
            cap_applied,
        )
    repeated_x = np.repeat(ix, count_per_cell)
    repeated_y = np.repeat(iy, count_per_cell)
    repeated_missing = np.repeat(missing, count_per_cell)
    expected = np.repeat(deficit.expected_count[iy, ix], count_per_cell)
    component = np.repeat(deficit.component_labels[iy, ix], count_per_cell)
    rng = np.random.default_rng(int(seed) & 0xFFFFFFFFFFFFFFFF)
    # Independent continuous jitter avoids a lattice signature.  Subsequent
    # variable-radius blue noise supplies the actual exclusion guarantee.
    jitter = rng.random((total, 2), dtype=np.float64)
    xmin, _, ymin, _ = deficit.bbox
    xy = np.column_stack(
        (
            xmin + (repeated_x + jitter[:, 0]) * deficit.cell_size_m,
            ymin + (repeated_y + jitter[:, 1]) * deficit.cell_size_m,
        )
    )
    local_spacing = np.sqrt(
        deficit.cell_size_m * deficit.cell_size_m / np.maximum(expected, 1.0)
    )
    radius = np.clip(
        local_spacing,
        nominal_spacing_m * minimum_radius_ratio,
        nominal_spacing_m * maximum_radius_ratio,
    )
    priority = repeated_missing / np.maximum(expected, 1.0)
    # A tiny deterministic non-cell term breaks exact priority ties without
    # changing the deficit-driven ordering.
    priority += rng.random(total) * 1.0e-9
    return DeficitProposals(
        xy=xy,
        radius_m=radius,
        priority=priority,
        component_id=component.astype(np.int32),
        requested_before_cap=requested,
        proposal_cap_applied=cap_applied,
    )


def _xyz(records: np.ndarray) -> np.ndarray:
    return np.column_stack((records["x"], records["y"], records["z"])).astype(
        np.float64
    )


def _normals(records: np.ndarray) -> np.ndarray:
    return np.column_stack((records["nx"], records["ny"], records["nz"])).astype(
        np.float64
    )


def select_bounded_geometry_donors(
    donor_records: np.ndarray,
    *,
    maximum_count: int,
) -> np.ndarray:
    """Bound triangulation cost while preserving spatial and vertical modes.

    At most the lowest, median, and highest record from each adaptive XY cell
    are retained before a spatially even final cap.  This keeps separated
    reflection/noise layers visible to thickness and multimodality vetoes;
    unlike a mean or lower-envelope decimation it cannot make noisy support
    look artificially clean.
    """

    measured = donor_records[terrain.measured_scan_mask(donor_records)]
    if len(measured) <= int(maximum_count):
        return measured
    if maximum_count < 12:
        raise ValueError("maximum_count must be at least 12")
    xyz = _xyz(measured)
    finite = np.all(np.isfinite(xyz), axis=1)
    measured = measured[finite]
    xyz = xyz[finite]
    if len(measured) <= int(maximum_count):
        return measured
    minimum = np.min(xyz[:, :2], axis=0)
    maximum = np.max(xyz[:, :2], axis=0)
    extent = np.maximum(maximum - minimum, 1.0e-12)
    target_cells = max(1, int(maximum_count) // 3)
    aspect = extent[0] / extent[1]
    columns = max(1, int(round(math.sqrt(target_cells * aspect))))
    rows = max(1, int(math.ceil(target_cells / columns)))
    ix = np.minimum(
        ((xyz[:, 0] - minimum[0]) / extent[0] * columns).astype(np.int64),
        columns - 1,
    )
    iy = np.minimum(
        ((xyz[:, 1] - minimum[1]) / extent[1] * rows).astype(np.int64),
        rows - 1,
    )
    cell = iy * columns + ix
    order = np.lexsort((np.arange(len(xyz), dtype=np.int64), xyz[:, 2], cell))
    ordered_cell = cell[order]
    starts = np.flatnonzero(np.r_[True, ordered_cell[1:] != ordered_cell[:-1]])
    ends = np.r_[starts[1:], len(order)]
    selected: list[int] = []
    for start, end in zip(starts, ends, strict=True):
        count = int(end - start)
        offsets = sorted({0, count // 2, count - 1})
        selected.extend(int(order[start + offset]) for offset in offsets)
    selected_array = np.asarray(selected, np.int64)
    if len(selected_array) > maximum_count:
        keep = np.linspace(
            0, len(selected_array) - 1, int(maximum_count), dtype=np.int64
        )
        selected_array = selected_array[keep]
    return measured[selected_array]


def compute_bounded_local_residual_energy(
    query_xy: np.ndarray,
    donor_xyz: np.ndarray,
    reference_energy_m2: np.ndarray | float,
    *,
    radius_m: float,
    maximum_donors: int,
    coefficients: np.ndarray | None = None,
) -> terrain.ResidualEnergyResult:
    """Compute the hard residual-energy veto with bounded local work.

    The underlying terrain implementation compares graph-neighbour residuals
    but forms an unbounded dense pairwise matrix in every neighbourhood.  A
    1 mm collar can contain more than a thousand points inside 5 cm, making
    that quadratic work unsuitable for production.  This equivalent path
    retains the nearest ``maximum_donors`` (the same bounded support used by
    MLS), then computes the identical detrended nearest-neighbour statistic.
    """

    query = np.asarray(query_xy, np.float64)
    donors = np.asarray(donor_xyz, np.float64)
    if query.ndim != 2 or query.shape[1] != 2:
        raise ValueError("query_xy must have shape (N, 2)")
    if donors.ndim != 2 or donors.shape[1] != 3 or not len(donors):
        raise ValueError("donor_xyz must have non-empty shape (N, 3)")
    if maximum_donors < 12:
        raise ValueError("maximum_donors must be at least 12")
    reference = np.asarray(reference_energy_m2, np.float64)
    if reference.ndim == 0:
        reference = np.full(len(query), float(reference), np.float64)
    if reference.shape != (len(query),):
        raise ValueError("reference_energy_m2 must be scalar or match query rows")
    if coefficients is None:
        fitted = terrain.predict_robust_quadratic_mls(
            query,
            donors,
            bandwidth_m=radius_m,
            maximum_donors=maximum_donors,
            minimum_donors=12,
        )
        fitted_coefficients = fitted.coefficients
    else:
        fitted_coefficients = np.asarray(coefficients, np.float64)
        if fitted_coefficients.shape != (len(query), 6):
            raise ValueError("coefficients must have shape (N, 6)")
    try:
        from scipy.spatial import cKDTree
    except ModuleNotFoundError:  # pragma: no cover - production has scipy
        neighbourhoods = [
            np.flatnonzero(
                np.sum(np.square(donors[:, :2] - point), axis=1)
                <= radius_m * radius_m
            )
            for point in query
        ]
    else:
        neighbourhoods = [
            np.asarray(value, np.int64)
            for value in cKDTree(donors[:, :2]).query_ball_point(
                query, radius_m, workers=-1
            )
        ]
    energy = np.full(len(query), np.nan, np.float64)
    counts = np.zeros(len(query), np.int32)
    for row, indices in enumerate(neighbourhoods):
        if len(indices) > maximum_donors:
            distance = np.linalg.norm(donors[indices, :2] - query[row], axis=1)
            indices = indices[
                np.argsort(distance, kind="stable")[:maximum_donors]
            ]
        counts[row] = len(indices)
        coefficients_row = fitted_coefficients[row]
        if len(indices) < 12 or not np.all(np.isfinite(coefficients_row)):
            continue
        selected = donors[indices]
        delta = selected[:, :2] - query[row]
        design = np.column_stack(
            (
                np.ones(len(selected)),
                delta[:, 0],
                delta[:, 1],
                np.square(delta[:, 0]),
                delta[:, 0] * delta[:, 1],
                np.square(delta[:, 1]),
            )
        )
        residual = selected[:, 2] - design @ coefficients_row
        pairwise = np.sum(
            np.square(selected[:, None, :2] - selected[None, :, :2]), axis=2
        )
        np.fill_diagonal(pairwise, np.inf)
        nearest = np.argmin(pairwise, axis=1)
        energy[row] = float(np.median(np.square(residual - residual[nearest])))
    return terrain.ResidualEnergyResult(
        energy_m2=energy,
        reference_energy_m2=reference,
        ratio=terrain.residual_energy_ratio(energy, reference),
        donor_count=counts,
    )


def estimate_local_reference_energy(
    donor_records: np.ndarray,
    *,
    nominal_spacing_m: float,
    support_radius_m: float,
    maximum_samples: int,
    maximum_energy_donors: int = 96,
) -> tuple[float, Mapping[str, object]]:
    """Estimate the measured residual-energy baseline; it remains veto-only."""

    measured = donor_records[terrain.measured_scan_mask(donor_records)]
    if not len(measured):
        raise ValueError("cannot estimate residual energy without measured donors")
    if len(measured) > maximum_samples:
        index = np.linspace(0, len(measured) - 1, maximum_samples, dtype=np.int64)
        sample = measured[index]
    else:
        sample = measured
    energy = compute_bounded_local_residual_energy(
        _xyz(sample)[:, :2],
        _xyz(measured),
        1.0,
        radius_m=support_radius_m,
        maximum_donors=maximum_energy_donors,
    ).energy_m2
    finite = energy[np.isfinite(energy) & (energy >= 0.0)]
    empirical = float(np.median(finite)) if len(finite) else 0.0
    numerical_floor = (nominal_spacing_m * 0.05) ** 2
    reference = max(empirical, numerical_floor)
    return reference, {
        "sample_count": int(len(sample)),
        "finite_sample_count": int(len(finite)),
        "empirical_median_m2": empirical,
        "numerical_floor_m2": numerical_floor,
        "reference_m2": reference,
        "positive_evidence": False,
        "use": "hard_veto_only",
        "maximum_local_donors": int(maximum_energy_donors),
    }


def _concatenate_confidence(parts: Sequence[ConfidenceResult]) -> ConfidenceResult:
    if not parts:
        return ConfidenceResult(
            np.empty(0, np.uint32),
            np.empty(0, np.uint8),
            np.empty(0, np.float64),
            np.empty(0, np.uint8),
        )
    return ConfidenceResult(
        reason_mask=np.concatenate([part.reason_mask for part in parts]),
        tier=np.concatenate([part.tier for part in parts]),
        surface_spread_m=np.concatenate([part.surface_spread_m for part in parts]),
        preferred_gate_count=np.concatenate(
            [part.preferred_gate_count for part in parts]
        ),
    )


def _take_confidence(
    confidence: ConfidenceResult,
    indices: np.ndarray,
) -> ConfidenceResult:
    selected = np.asarray(indices, np.int64)
    return ConfidenceResult(
        reason_mask=np.asarray(confidence.reason_mask[selected], np.uint32).copy(),
        tier=np.asarray(confidence.tier[selected], np.uint8).copy(),
        surface_spread_m=np.asarray(
            confidence.surface_spread_m[selected], np.float64
        ).copy(),
        preferred_gate_count=np.asarray(
            confidence.preferred_gate_count[selected], np.uint8
        ).copy(),
    )


def evaluate_proposals_batched(
    proposals: DeficitProposals,
    donor_records: np.ndarray,
    *,
    reference_energy_m2: float,
    support_radius_m: float,
    thresholds: ConfidenceThresholds,
    batch_points: int,
    maximum_energy_donors: int = 96,
) -> GeometryProposalEvaluation:
    """Evaluate all three surfaces and all hard gates in bounded batches."""

    if not len(proposals.xy):
        empty = np.empty(0, np.float64)
        return GeometryProposalEvaluation(
            np.empty((0, 3), np.float64),
            np.empty((0, 3), np.float64),
            _concatenate_confidence([]),
            np.empty((0, 3), np.float64),
            np.empty(0, np.uint8),
            empty,
            empty,
            empty,
            empty,
        )
    donors = donor_records[terrain.measured_scan_mask(donor_records)]
    if not len(donors):
        raise ValueError("geometry evaluation requires measured same-role donors")
    donor_xyz = _xyz(donors)
    donor_normals = _normals(donors)
    # Delaunay triangulation is the only global surface.  Build it once for all
    # queries; rebuilding a 100k-point triangulation for each batch would make
    # the production 1 mm stage needlessly expensive.
    linear_all = terrain.predict_delaunay_linear_surface(proposals.xy, donor_xyz)
    xyz_parts: list[np.ndarray] = []
    normal_parts: list[np.ndarray] = []
    surface_parts: list[np.ndarray] = []
    confidence_parts: list[ConfidenceResult] = []
    sector_parts: list[np.ndarray] = []
    coherence_parts: list[np.ndarray] = []
    thickness_parts: list[np.ndarray] = []
    mode_parts: list[np.ndarray] = []
    energy_parts: list[np.ndarray] = []
    for begin in range(0, len(proposals.xy), int(batch_points)):
        end = min(begin + int(batch_points), len(proposals.xy))
        query = proposals.xy[begin:end]
        mls = terrain.predict_robust_quadratic_mls(
            query, donor_xyz, bandwidth_m=support_radius_m
        )
        boundary = terrain.predict_lower_boundary_surface(
            query,
            donor_xyz,
            outer_radius_m=max(support_radius_m, 0.041),
            inner_radius_m=min(0.010, support_radius_m * 0.25),
        )
        sectors = terrain.compute_eight_sector_support(
            query, donor_xyz[:, :2], outer_radius_m=support_radius_m
        )
        normal_metrics = terrain.compute_local_normal_coherence(
            query, donor_xyz[:, :2], donor_normals, radius_m=support_radius_m
        )
        vertical = terrain.compute_vertical_distribution_metrics(
            query, donor_xyz, radius_m=min(support_radius_m, 0.035)
        )
        energy = compute_bounded_local_residual_energy(
            query,
            donor_xyz,
            reference_energy_m2,
            radius_m=support_radius_m,
            maximum_donors=maximum_energy_donors,
            coefficients=mls.coefficients,
        )
        linear_height = linear_all.height_m[begin:end]
        linear_normal = linear_all.normal[begin:end]
        surfaces = np.column_stack(
            (mls.height_m, linear_height, boundary.height_m)
        )
        confidence = evaluate_geometry_confidence(
            sectors.occupied_sector_count,
            surfaces,
            normal_metrics.coherence,
            vertical.thickness_m,
            vertical.multimodality_score,
            energy.ratio,
            thresholds=thresholds,
        )
        z = np.median(surfaces, axis=1)
        # The selector requires finite bookkeeping coordinates even for rows
        # already rejected by INVALID_METRIC.  A local measured median is used
        # only as a provenance placeholder; the hard confidence mask prevents
        # any such row from becoming an addition.
        invalid_z = ~np.isfinite(z)
        if np.any(invalid_z):
            z[invalid_z] = float(np.median(donor_xyz[:, 2]))
        normals = np.stack(
            (mls.normal, linear_normal, boundary.normal),
            axis=1,
        )
        flip = normals[:, :, 2] < 0.0
        normals[flip] *= -1.0
        predicted = np.mean(normals, axis=1)
        length = np.linalg.norm(predicted, axis=1)
        valid_normal = np.isfinite(length) & (length > 1.0e-12)
        predicted[valid_normal] /= length[valid_normal, None]
        predicted[~valid_normal] = np.nan
        xyz_parts.append(np.column_stack((query, z)))
        normal_parts.append(predicted)
        surface_parts.append(surfaces)
        confidence_parts.append(confidence)
        sector_parts.append(sectors.occupied_sector_count)
        coherence_parts.append(normal_metrics.coherence)
        thickness_parts.append(vertical.thickness_m)
        mode_parts.append(vertical.multimodality_score)
        energy_parts.append(energy.ratio)
    return GeometryProposalEvaluation(
        candidate_xyz=np.concatenate(xyz_parts),
        predicted_normal=np.concatenate(normal_parts),
        confidence=_concatenate_confidence(confidence_parts),
        surface_heights_m=np.concatenate(surface_parts),
        donor_sectors=np.concatenate(sector_parts),
        normal_coherence=np.concatenate(coherence_parts),
        vertical_thickness_m=np.concatenate(thickness_parts),
        multimodality_score=np.concatenate(mode_parts),
        residual_energy_ratio=np.concatenate(energy_parts),
    )


def _finite_quantiles(values: np.ndarray) -> Mapping[str, float | None]:
    finite = np.asarray(values, np.float64)
    finite = finite[np.isfinite(finite)]
    if not len(finite):
        return {str(q): None for q in (0.0, 0.05, 0.5, 0.95, 1.0)}
    return {
        str(q): float(np.quantile(finite, q))
        for q in (0.0, 0.05, 0.5, 0.95, 1.0)
    }


def _validate_combined_normalizations(
    values: Mapping[str, Mapping[str, float]],
) -> dict[str, dict[str, float]]:
    result: dict[str, dict[str, float]] = {}
    for metric in PHYSICAL_METRICS:
        if metric not in values:
            raise ValueError(f"combined normalization is missing {metric}")
        result[metric] = {}
        for scale in DERIVED_SCALES:
            value = float(values[metric][scale])
            if not np.isfinite(value) or value <= 0.0:
                raise ValueError(
                    f"combined normalization {metric}/{scale} must be positive"
                )
            result[metric][scale] = value
    return result


def load_combined_normalizations(
    manifest_path: str | Path,
) -> tuple[dict[str, dict[str, float]], Mapping[str, object]]:
    """Load the shared ROCK normalization from a hash-locked v10 manifest."""

    path = Path(manifest_path).resolve(strict=True)
    document = json.loads(path.read_text(encoding="utf-8"))
    values = document.get("rock_combined_normalizations", document)
    if not isinstance(values, Mapping):
        raise ValueError("normalization manifest has no normalization mapping")
    normalized = _validate_combined_normalizations(values)
    return normalized, {
        "method": "provided-manifest",
        "path": str(path),
        "sha256": sha256_path(path),
        "values": normalized,
    }


def infer_combined_normalizations(
    rock_1mm_path: str | Path,
    *,
    chunk_records: int,
    sample_limit: int,
) -> tuple[dict[str, dict[str, float]], Mapping[str, object]]:
    """Infer the same global ROCK p95 policy used by the Scene1 base clouds."""

    try:
        import site1_scalar_fill as scalar_fill
    except ModuleNotFoundError:  # pragma: no cover - package import fallback
        from scripts import site1_scalar_fill as scalar_fill
    options = scalar_fill.RepairOptions(
        chunk_size=int(chunk_records),
        derived_normalization_sample_limit=int(sample_limit),
    )
    values, report = scalar_fill.infer_derived_normalization(
        rock_1mm_path, "1mm", options
    )
    normalized = _validate_combined_normalizations(values)
    return normalized, {
        "method": "inferred-from-hash-locked-rock-1mm",
        "source": str(Path(rock_1mm_path).resolve()),
        "values": normalized,
        "inference": report,
    }


def postprocess_local_analysed_additions(
    records: np.ndarray,
    initial_records: np.ndarray,
    combined_normalizations: Mapping[str, Mapping[str, float]] | None,
) -> Mapping[str, object]:
    """Restore donor visibility fields and apply global combined normalization.

    CleanMesh recomputes local geometry components.  Visibility fields need a
    two-metre scene context, which a deliberately local collar cannot supply,
    so their same-role donor interpolation is restored.  Combined curvature,
    recession, and roughness are then rebuilt with the shared global ROCK p95
    scales instead of the local-collar histogram.
    """

    if records.dtype != initial_records.dtype or len(records) != len(initial_records):
        raise ValueError("analysed/initial addition rows or schemas differ")
    names = set(records.dtype.names or ())
    restored = []
    for field_name in LOCAL_VISIBILITY_FIELDS:
        if field_name in names:
            records[field_name] = initial_records[field_name]
            restored.append(field_name)
    rebuilt = []
    if combined_normalizations is not None:
        normalizations = _validate_combined_normalizations(combined_normalizations)
        for metric in PHYSICAL_METRICS:
            component_names = [
                f"scalar_A_R_{metric}_{scale}" for scale in DERIVED_SCALES
            ]
            combined_name = f"scalar_A_R_{metric}_Combined"
            if not set((*component_names, combined_name)).issubset(names):
                continue
            values = np.column_stack(
                [np.asarray(records[name], np.float64) for name in component_names]
            )
            scales = np.asarray(
                [normalizations[metric][scale] for scale in DERIVED_SCALES],
                np.float64,
            )
            finite = np.isfinite(values)
            normalized = values / scales[None, :]
            if metric == "Roughness":
                normalized = np.clip(normalized, 0.0, 1.0)
            else:
                normalized = np.clip(normalized, -1.0, 1.0)
            used = np.sum(finite * COMBINED_WEIGHTS[None, :], axis=1)
            weighted = np.sum(
                np.where(finite, normalized, 0.0) * COMBINED_WEIGHTS[None, :],
                axis=1,
            )
            combined = np.full(len(records), np.nan, np.float64)
            valid = used > 0.0
            combined[valid] = weighted[valid] / used[valid]
            records[combined_name] = combined.astype(records.dtype[combined_name])
            rebuilt.append(combined_name)
        relative_name = "scalar_A_R_RoughnessRelative_FineMedium"
        fine_name = "scalar_A_R_Roughness_Fine"
        medium_name = "scalar_A_R_Roughness_Medium"
        if {relative_name, fine_name, medium_name}.issubset(names):
            fine = np.asarray(records[fine_name], np.float64)
            medium = np.asarray(records[medium_name], np.float64)
            relative = np.full(len(records), np.nan, np.float64)
            valid = np.isfinite(fine) & np.isfinite(medium)
            relative[valid] = np.clip(
                fine[valid] / np.maximum(medium[valid], 1.0e-9), 0.0, 8.0
            )
            records[relative_name] = relative.astype(records.dtype[relative_name])
            rebuilt.append(relative_name)
    return {
        "restored_same_role_donor_visibility_fields": restored,
        "rebuilt_with_global_normalization": rebuilt,
        "field_quantiles_after": {
            name: _finite_quantiles(records[name])
            for name in (*restored, *rebuilt)
        },
        "local_histogram_normalization_retained": False
        if combined_normalizations is not None
        else True,
    }


def _component_json(component: terrain.DensityDeficitComponent) -> Mapping[str, object]:
    return asdict(component)


def _assessment_json(assessment: RoleAssessment) -> Mapping[str, object]:
    return {
        "role": assessment.role,
        "support_points": assessment.support_points,
        "occupied_fraction": assessment.occupied_fraction,
        "missing_points": assessment.missing_points,
        "component_cells": assessment.component_cells,
        "score": assessment.score,
        "components": [_component_json(value) for value in assessment.deficit.components],
    }


def _empty_proposals() -> DeficitProposals:
    return DeficitProposals(
        np.empty((0, 2), np.float64),
        np.empty(0, np.float64),
        np.empty(0, np.float64),
        np.empty(0, np.int32),
        0,
        False,
    )


def build_target_additions(
    target: terrain.TerrainReviewTarget,
    clouds: Mapping[str, LocalRoleCloud],
    output_dir: Path,
    parameters: ResolutionParameters,
    pipeline_parameters: PipelineParameters,
    *,
    seed: int,
) -> TargetBuildResult:
    """Build accepted initial records and lossless provenance for one target."""

    choice = choose_supported_role(
        clouds,
        target,
        parameters,
        dominance_ratio=pipeline_parameters.role_dominance_ratio,
    )
    base_audit: dict[str, object] = {
        "target_id": target.target_id,
        "kind": target.kind.value,
        "bbox": list(target.bbox),
        "bbox_is_search_window_only": True,
        "minimum_tier": target.minimum_tier.name,
        "role_choice": choice.role,
        "role_choice_reason": choice.reason,
        "role_assessments": [_assessment_json(value) for value in choice.assessments],
    }
    target_dir = output_dir / target.target_id

    def finish(
        proposals: DeficitProposals,
        records: np.ndarray | None,
        role: str | None,
        accepted_count: int,
        *,
        accepted_radius_m: np.ndarray | None = None,
        accepted_priority: np.ndarray | None = None,
        accepted_confidence: ConfidenceResult | None = None,
        accepted_candidate_index: np.ndarray | None = None,
        accepted_primary_donor_index: np.ndarray | None = None,
        accepted_donor_distance_m: np.ndarray | None = None,
        accepted_donor_count: np.ndarray | None = None,
    ) -> TargetBuildResult:
        target_dir.mkdir(parents=True, exist_ok=True)
        (target_dir / "audit.json").write_text(
            json.dumps(base_audit, indent=2, sort_keys=True, allow_nan=False) + "\n",
            encoding="utf-8",
        )
        return TargetBuildResult(
            target,
            choice,
            proposals,
            records,
            role,
            accepted_count,
            base_audit,
            (
                np.asarray(accepted_radius_m, np.float64).copy()
                if accepted_radius_m is not None
                else np.empty(0, np.float64)
            ),
            (
                np.asarray(accepted_priority, np.float64).copy()
                if accepted_priority is not None
                else np.empty(0, np.float64)
            ),
            (
                accepted_confidence
                if accepted_confidence is not None
                else _concatenate_confidence([])
            ),
            (
                np.asarray(accepted_candidate_index, np.int64).copy()
                if accepted_candidate_index is not None
                else np.empty(0, np.int64)
            ),
            np.empty(0, np.int64),
            (
                np.asarray(accepted_primary_donor_index, np.int64).copy()
                if accepted_primary_donor_index is not None
                else np.empty(0, np.int64)
            ),
            (
                np.asarray(accepted_donor_distance_m, np.float64).copy()
                if accepted_donor_distance_m is not None
                else np.empty(0, np.float64)
            ),
            (
                np.asarray(accepted_donor_count, np.int32).copy()
                if accepted_donor_count is not None
                else np.empty(0, np.int32)
            ),
        )

    if choice.role is None:
        base_audit.update({"proposals": 0, "accepted": 0, "status": "unchanged"})
        return finish(_empty_proposals(), None, None, 0)

    assessment = next(value for value in choice.assessments if value.role == choice.role)
    proposals = generate_irregular_deficit_proposals(
        assessment.deficit,
        nominal_spacing_m=parameters.nominal_spacing_m,
        oversampling=parameters.proposal_oversampling,
        minimum_radius_ratio=parameters.minimum_radius_ratio,
        maximum_radius_ratio=parameters.maximum_radius_ratio,
        maximum_proposals=parameters.maximum_proposals_per_target,
        seed=seed,
    )
    cloud = clouds[choice.role]
    measured = cloud.measured_records
    measured_xy = np.column_stack((measured["x"], measured["y"])).astype(np.float64)
    collar = _bbox_mask(measured_xy, _expanded_bbox(target.bbox, parameters.source_collar_m))
    donor_records = measured[collar]
    if not len(proposals.xy) or len(donor_records) < 12:
        base_audit.update(
            {
                "proposals": int(len(proposals.xy)),
                "accepted": 0,
                "status": "unchanged",
                "reason": "no_proposals_or_insufficient_measured_collar",
                "measured_collar_points": int(len(donor_records)),
            }
        )
        return finish(proposals, None, choice.role, 0)

    geometry_donors = select_bounded_geometry_donors(
        donor_records,
        maximum_count=parameters.maximum_geometry_donors,
    )
    reference_energy, energy_audit = estimate_local_reference_energy(
        geometry_donors,
        nominal_spacing_m=parameters.nominal_spacing_m,
        support_radius_m=parameters.support_radius_m,
        maximum_samples=parameters.reference_energy_samples,
        maximum_energy_donors=parameters.maximum_energy_donors,
    )
    evaluated = evaluate_proposals_batched(
        proposals,
        geometry_donors,
        reference_energy_m2=reference_energy,
        support_radius_m=parameters.support_radius_m,
        thresholds=pipeline_parameters.confidence,
        batch_points=parameters.geometry_batch_points,
        maximum_energy_donors=parameters.maximum_energy_donors,
    )
    selection = terrain.select_variable_radius_interstitials(
        evaluated.candidate_xyz,
        proposals.radius_m,
        _xyz(donor_records),
        evaluated.confidence,
        minimum_tier=target.minimum_tier,
        priority=proposals.priority,
        seed=seed ^ 0x424C55454E4F4953,
        dimensions=2,
    )
    selected = selection.selected_indices
    sampled = terrain.build_scanid10_additions(
        evaluated.candidate_xyz[selected],
        donor_records,
        role=choice.role,
        donor_role=choice.role,
        maximum_donor_distance_m=parameters.property_donor_distance_m,
        neighbours=8,
        predicted_normals=evaluated.predicted_normal[selected],
    )
    provenance = terrain.build_candidate_provenance(
        evaluated.candidate_xyz,
        target.target_id,
        evaluated.confidence,
        selection,
        sampled,
    )
    provenance_prefix = output_dir / target.target_id / "proposal-provenance"
    provenance_npz, provenance_json = terrain.write_candidate_provenance(
        provenance_prefix, provenance
    )
    accepted = sampled.records
    accepted_candidate_indices = selected[sampled.resolved_query_indices]
    tier_counts = {
        tier.name: int(np.count_nonzero(evaluated.confidence.tier == int(tier)))
        for tier in ConfidenceTier
    }
    base_audit.update(
        {
            "status": "accepted" if len(accepted) else "unchanged",
            "measured_collar_points": int(len(donor_records)),
            "proposal_generation": {
                "requested_before_cap": proposals.requested_before_cap,
                "generated": int(len(proposals.xy)),
                "cap_applied": proposals.proposal_cap_applied,
                "irregular_continuous_jitter": True,
                "whole_bbox_fill": False,
            },
            "geometry": {
                "models": [
                    "robust-quadratic-mls",
                    "delaunay-linear",
                    "sector-lower-boundary-plane",
                ],
                "all_three_required_finite": True,
                "hard_vetoes_required_clear": True,
                "raw_measured_collar_points": int(len(donor_records)),
                "bounded_geometry_donor_points": int(len(geometry_donors)),
                "vertical_modes_preserved_by_geometry_reduction": True,
                "tier_counts": tier_counts,
                "surface_spread_m": _finite_quantiles(
                    evaluated.confidence.surface_spread_m
                ),
                "normal_coherence": _finite_quantiles(
                    evaluated.normal_coherence
                ),
                "vertical_thickness_m": _finite_quantiles(
                    evaluated.vertical_thickness_m
                ),
                "multimodality_score": _finite_quantiles(
                    evaluated.multimodality_score
                ),
                "residual_energy_ratio": _finite_quantiles(
                    evaluated.residual_energy_ratio
                ),
                "reference_energy": energy_audit,
            },
            "spacing": {
                "blue_noise_selected_before_donor_gate": int(len(selected)),
                "accepted_after_same_role_donor_gate": int(len(accepted)),
                "radius_m": _finite_quantiles(proposals.radius_m),
                "dimensions": 2,
            },
            "properties": {
                "donor_role": choice.role,
                "donor_scan_ids": list(terrain.MEASURED_SCAN_IDS),
                "addition_scan_id": terrain.ADDITION_SCAN_ID,
                "resolved": int(len(sampled.resolved_query_indices)),
                "unresolved": int(len(sampled.unresolved_query_indices)),
                "nearest_donor_distance_m": _finite_quantiles(
                    sampled.nearest_distance_m
                ),
            },
            "provenance": {
                "npz": str(provenance_npz.relative_to(output_dir.parent)),
                "json": str(provenance_json.relative_to(output_dir.parent)),
                "npz_sha256": sha256_path(provenance_npz),
                "json_sha256": sha256_path(provenance_json),
            },
            "accepted": int(len(accepted)),
        }
    )
    return finish(
        proposals,
        accepted,
        choice.role,
        int(len(accepted)),
        accepted_radius_m=proposals.radius_m[accepted_candidate_indices],
        accepted_priority=proposals.priority[accepted_candidate_indices],
        accepted_confidence=_take_confidence(
            evaluated.confidence, accepted_candidate_indices
        ),
        accepted_candidate_index=accepted_candidate_indices,
        accepted_primary_donor_index=sampled.primary_donor_indices,
        accepted_donor_distance_m=sampled.nearest_distance_m,
        accepted_donor_count=sampled.contributing_donor_count,
    )


def run_cleanmesh_reduced_analysis(
    executable: Path,
    input_path: Path,
    output_path: Path,
    report_path: Path,
    resolution: ResolutionParameters,
    pipeline: PipelineParameters,
) -> Mapping[str, object]:
    """Run the production CleanMesh reduced analyser on the local input only."""

    executable = executable.resolve(strict=True)
    if not executable.is_file() or not os.access(executable, os.X_OK):
        raise ValueError(f"CleanMesh reduced analysis is not executable: {executable}")
    for artifact in (output_path, report_path):
        if artifact.exists():
            raise FileExistsError(f"refusing existing CleanMesh artifact: {artifact}")
    command = [
        str(executable),
        "--input", str(input_path),
        "--out", str(output_path),
        "--report", str(report_path),
        "--base-voxel", format(resolution.cleanmesh_base_voxel_m, ".12g"),
        "--tile-width", format(pipeline.cleanmesh_tile_width_m, ".12g"),
        "--chunk-points", str(pipeline.cleanmesh_chunk_points),
        "--normalization-samples", str(pipeline.cleanmesh_normalization_samples),
    ]
    completed = subprocess.run(command, check=False)
    if completed.returncode != 0:
        raise RuntimeError(
            f"CleanMesh reduced analysis failed with status {completed.returncode}"
        )
    if not output_path.is_file() or not report_path.is_file():
        raise RuntimeError("CleanMesh did not produce both output and report")
    terrain.inspect_fixed_stride_ply(output_path)
    return {
        "command": command,
        "returncode": completed.returncode,
        "output_sha256": sha256_path(output_path),
        "report_sha256": sha256_path(report_path),
    }


def _collar_source_selection(
    cloud: LocalRoleCloud,
    accepted_targets: Sequence[terrain.TerrainReviewTarget],
    margin_m: float,
) -> terrain.LocalSourceSelection | None:
    if not accepted_targets or not len(cloud.records):
        return None
    xy = np.column_stack((cloud.records["x"], cloud.records["y"])).astype(np.float64)
    bboxes = [_expanded_bbox(target.bbox, margin_m) for target in accepted_targets]
    selected = cloud.measured_mask & _union_bbox_mask(xy, bboxes)
    if not np.any(selected):
        return None
    return terrain.make_local_source_selection(
        cloud.layout,
        cloud.original_indices[selected],
        role=cloud.role,
        label=f"{cloud.role.lower()}-measured-local-collar",
    )


def _candidate_filename(role: str, label: str) -> str:
    return f"Site1-{role}-{label}.terrain-v11.candidate.ply"


def _write_resolution_report(path: Path, value: Mapping[str, object]) -> None:
    path.write_text(
        json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )


AUTHORITATIVE_ARCHIVE_KEYS = frozenset(
    {
        "records",
        "fine_index",
        "target_id",
        "target_candidate_index",
        "global_ledger_index",
        "radius_m",
        "priority",
        "confidence_reason_mask",
        "confidence_tier",
        "confidence_surface_spread_m",
        "confidence_preferred_gate_count",
        "target_donor_index",
        "target_donor_distance_m",
        "target_donor_count",
    }
)


def _write_deterministic_npz(path: Path, arrays: Mapping[str, np.ndarray]) -> None:
    """Write a pickle-free NPZ whose bytes do not depend on wall-clock time."""

    if path.exists():
        raise FileExistsError(f"refusing existing archive: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.partial")
    try:
        with zipfile.ZipFile(
            temporary,
            mode="w",
            compression=zipfile.ZIP_DEFLATED,
            compresslevel=6,
        ) as archive:
            for name in sorted(arrays):
                value = np.asarray(arrays[name])
                if value.dtype.hasobject:
                    raise ValueError(f"archive array {name} must not contain objects")
                payload = io.BytesIO()
                np.lib.format.write_array(payload, value, allow_pickle=False)
                entry = zipfile.ZipInfo(f"{name}.npy", date_time=(1980, 1, 1, 0, 0, 0))
                entry.compress_type = zipfile.ZIP_DEFLATED
                entry.external_attr = 0o600 << 16
                archive.writestr(entry, payload.getvalue(), compress_type=zipfile.ZIP_DEFLATED)
        os.replace(temporary, path)
    except BaseException:
        if temporary.exists():
            temporary.unlink()
        raise


def _record_payload_sha256(records: np.ndarray) -> str:
    digest = hashlib.sha256()
    digest.update(np.asarray(records).tobytes(order="C"))
    return digest.hexdigest()


def _verify_candidate_suffix(
    candidate_path: str | Path,
    *,
    source_count: int,
    expected_records: np.ndarray,
) -> None:
    """Prove an authoritative archive is the candidate's exact appended suffix."""

    layout = terrain.inspect_fixed_stride_ply(candidate_path)
    if layout.dtype != expected_records.dtype:
        raise RuntimeError("candidate/archive schemas differ")
    if layout.vertex_count != int(source_count) + len(expected_records):
        raise RuntimeError("candidate count does not equal source plus archive")
    memory = np.memmap(
        layout.path,
        dtype=layout.dtype,
        mode="r",
        offset=layout.offset,
        shape=(layout.vertex_count,),
    )
    try:
        suffix = np.asarray(memory[int(source_count) :])
        if suffix.tobytes(order="C") != expected_records.tobytes(order="C"):
            raise RuntimeError("candidate appended suffix differs from archive records")
    finally:
        del memory


def _fine_archive_arrays(
    role: str,
    records: np.ndarray,
    results: Sequence[TargetBuildResult],
) -> Mapping[str, np.ndarray]:
    """Return final-suffix-aligned fine provenance with no object arrays."""

    selected = [
        result
        for result in results
        if result.accepted_role == role and result.accepted_count > 0
    ]
    counts = [int(result.accepted_count) for result in selected]
    if sum(counts) != len(records):
        raise RuntimeError(f"{role}: final records/provenance counts differ")

    def concatenate(attribute: str, dtype: np.dtype) -> np.ndarray:
        values = [np.asarray(getattr(result, attribute), dtype=dtype) for result in selected]
        return np.concatenate(values) if values else np.empty(0, dtype=dtype)

    confidence_parts = [result.accepted_confidence for result in selected]
    combined_confidence = _concatenate_confidence(confidence_parts)
    maximum_target_length = max(
        (len(result.target.target_id) for result in selected), default=1
    )
    target_id = (
        np.concatenate(
            [
                np.full(
                    result.accepted_count,
                    result.target.target_id,
                    dtype=f"U{maximum_target_length}",
                )
                for result in selected
            ]
        )
        if selected
        else np.empty(0, dtype="U1")
    )
    arrays = {
        "records": np.asarray(records).copy(),
        "fine_index": np.arange(len(records), dtype=np.int64),
        "target_id": target_id,
        "target_candidate_index": concatenate(
            "accepted_candidate_index", np.int64
        ),
        "global_ledger_index": concatenate(
            "accepted_global_ledger_index", np.int64
        ),
        "radius_m": concatenate("accepted_radius_m", np.float64),
        "priority": concatenate("accepted_priority", np.float64),
        "confidence_reason_mask": np.asarray(
            combined_confidence.reason_mask, np.uint32
        ),
        "confidence_tier": np.asarray(combined_confidence.tier, np.uint8),
        "confidence_surface_spread_m": np.asarray(
            combined_confidence.surface_spread_m, np.float64
        ),
        "confidence_preferred_gate_count": np.asarray(
            combined_confidence.preferred_gate_count, np.uint8
        ),
        "target_donor_index": concatenate(
            "accepted_primary_donor_index", np.int64
        ),
        "target_donor_distance_m": concatenate(
            "accepted_donor_distance_m", np.float64
        ),
        "target_donor_count": concatenate("accepted_donor_count", np.int32),
    }
    if set(arrays) != AUTHORITATIVE_ARCHIVE_KEYS:
        raise RuntimeError("internal error: authoritative archive schema drift")
    for name, value in arrays.items():
        if len(value) != len(records):
            raise RuntimeError(f"{role}: archive field {name} is misaligned")
    if len(records):
        if len(np.unique(arrays["global_ledger_index"])) != len(records):
            raise RuntimeError(f"{role}: duplicate global arbitration provenance")
        if np.any(arrays["confidence_reason_mask"] != 0):
            raise RuntimeError(f"{role}: accepted fine archive contains a hard veto")
        if np.any(arrays["confidence_tier"] < int(ConfidenceTier.SUPPORTED)):
            raise RuntimeError(f"{role}: accepted fine archive is below SUPPORTED")
    return arrays


def _load_authoritative_archive(
    path: str | Path,
    *,
    expected_sha256: str,
    expected_count: int,
    expected_dtype: np.dtype,
) -> dict[str, np.ndarray]:
    """Load and strictly validate a fine archive before coarse derivation."""

    archive_path = Path(path)
    if sha256_path(archive_path) != expected_sha256:
        raise RuntimeError(f"authoritative fine archive hash changed: {archive_path}")
    with np.load(archive_path, allow_pickle=False) as loaded:
        if set(loaded.files) != AUTHORITATIVE_ARCHIVE_KEYS:
            raise RuntimeError("authoritative fine archive keys differ from schema")
        arrays = {name: np.asarray(loaded[name]).copy() for name in loaded.files}
    records = arrays["records"]
    if records.ndim != 1 or records.dtype != expected_dtype:
        raise RuntimeError("authoritative fine archive record schema differs")
    if len(records) != int(expected_count):
        raise RuntimeError("authoritative fine archive point count differs")
    for name, value in arrays.items():
        if value.ndim != 1 or len(value) != len(records):
            raise RuntimeError(f"authoritative fine archive field {name} is misaligned")
    if not np.array_equal(arrays["fine_index"], np.arange(len(records))):
        raise RuntimeError("authoritative fine archive index is not canonical")
    xyz = _xyz(records)
    if not np.all(np.isfinite(xyz)):
        raise RuntimeError("authoritative fine archive contains non-finite XYZ")
    if len(records) and not np.all(
        np.rint(records[terrain._scan_field(records.dtype)]) == terrain.ADDITION_SCAN_ID
    ):
        raise RuntimeError("authoritative fine archive contains non-ScanID10 rows")
    if np.any(arrays["confidence_reason_mask"] != 0) or np.any(
        arrays["confidence_tier"] < int(ConfidenceTier.SUPPORTED)
    ):
        raise RuntimeError("authoritative fine archive contains rejected confidence")
    if len(np.unique(arrays["global_ledger_index"])) != len(records):
        raise RuntimeError("authoritative fine archive has duplicate ledger mapping")
    return arrays


def _nearest_xy(
    query_xy: np.ndarray,
    support_xy: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Return exact nearest XY distance/index with a bounded-memory fallback."""

    query = np.asarray(query_xy, np.float64)
    support = np.asarray(support_xy, np.float64)
    if not len(query):
        return np.empty(0, np.float64), np.empty(0, np.int64)
    if not len(support):
        return (
            np.full(len(query), np.inf, np.float64),
            np.full(len(query), -1, np.int64),
        )
    try:
        from scipy.spatial import cKDTree
    except ModuleNotFoundError:  # pragma: no cover - production has scipy
        distance = np.full(len(query), np.inf, np.float64)
        index = np.full(len(query), -1, np.int64)
        for begin in range(0, len(query), 2_048):
            delta = query[begin : begin + 2_048, None, :] - support[None, :, :]
            squared = np.sum(np.square(delta), axis=2)
            local = np.argmin(squared, axis=1)
            rows = np.arange(len(local))
            distance[begin : begin + len(local)] = np.sqrt(squared[rows, local])
            index[begin : begin + len(local)] = local
        return distance, index
    distance, index = cKDTree(support).query(query, k=1, workers=-1)
    return np.asarray(distance, np.float64), np.asarray(index, np.int64)


def _final_spacing_audit(
    candidate_xyz: np.ndarray,
    candidate_radius_m: np.ndarray,
    measured_xyz: np.ndarray,
) -> Mapping[str, object]:
    """Independently audit final XY spacing to measured and added points."""

    xyz = np.asarray(candidate_xyz, np.float64)
    radii = np.asarray(candidate_radius_m, np.float64)
    measured = np.asarray(measured_xyz, np.float64)
    if not len(xyz):
        return {
            "selected_points": 0,
            "minimum_candidate_to_measured_distance_m": None,
            "minimum_candidate_to_measured_clearance_ratio": None,
            "minimum_candidate_pair_distance_m": None,
            "minimum_candidate_pair_clearance_ratio_within_max_radius": None,
            "measured_clearance_verified": True,
            "candidate_pair_clearance_verified": True,
        }
    measured_distance, _ = _nearest_xy(xyz[:, :2], measured[:, :2])
    measured_ratio = measured_distance / radii
    minimum_pair_distance = None
    minimum_pair_ratio = None
    pair_verified = True
    if len(xyz) > 1:
        try:
            from scipy.spatial import cKDTree
        except ModuleNotFoundError:  # pragma: no cover - production has scipy
            pair_distance = np.full(len(xyz), np.inf, np.float64)
            ratio_values: list[np.ndarray] = []
            for begin in range(0, len(xyz), 1_024):
                end = min(begin + 1_024, len(xyz))
                delta = xyz[begin:end, None, :2] - xyz[None, :, :2]
                distance = np.linalg.norm(delta, axis=2)
                rows = np.arange(begin, end)
                distance[np.arange(end - begin), rows] = np.inf
                pair_distance[begin:end] = np.min(distance, axis=1)
                required = np.maximum(radii[begin:end, None], radii[None, :])
                relevant = distance <= float(np.max(radii))
                if np.any(relevant):
                    ratio_values.append((distance / required)[relevant])
            minimum_pair_distance = float(np.min(pair_distance))
            if ratio_values:
                minimum_pair_ratio = float(np.min(np.concatenate(ratio_values)))
        else:
            tree = cKDTree(xyz[:, :2])
            nearest, _ = tree.query(xyz[:, :2], k=2, workers=-1)
            minimum_pair_distance = float(np.min(nearest[:, 1]))
            pairs = tree.query_pairs(
                float(np.max(radii)), output_type="ndarray"
            )
            if len(pairs):
                delta = xyz[pairs[:, 0], :2] - xyz[pairs[:, 1], :2]
                distance = np.linalg.norm(delta, axis=1)
                required = np.maximum(radii[pairs[:, 0]], radii[pairs[:, 1]])
                minimum_pair_ratio = float(np.min(distance / required))
        if minimum_pair_ratio is not None:
            pair_verified = minimum_pair_ratio >= 1.0 - 1.0e-10
    measured_verified = bool(np.all(measured_ratio >= 1.0 - 1.0e-10))
    return {
        "selected_points": int(len(xyz)),
        "radius_m": _finite_quantiles(radii),
        "minimum_candidate_to_measured_distance_m": float(
            np.min(measured_distance)
        ),
        "minimum_candidate_to_measured_clearance_ratio": float(
            np.min(measured_ratio)
        ),
        "minimum_candidate_pair_distance_m": minimum_pair_distance,
        "minimum_candidate_pair_clearance_ratio_within_max_radius": (
            minimum_pair_ratio
        ),
        "measured_clearance_verified": measured_verified,
        "candidate_pair_clearance_verified": bool(pair_verified),
    }


def arbitrate_resolution_additions(
    results: Sequence[TargetBuildResult],
    clouds: Mapping[str, LocalRoleCloud],
    output_dir: Path,
    resolution: ResolutionParameters,
    *,
    seed: int,
) -> tuple[tuple[TargetBuildResult, ...], Mapping[str, object]]:
    """Apply one deterministic variable-radius arbitration per terrain role.

    Target windows may overlap.  Their local selectors intentionally operate
    independently so each target keeps a complete evidence ledger; this pass
    is the authoritative final acceptance decision and prevents duplicate or
    over-dense ScanID10 rows across target boundaries.
    """

    updated = list(results)
    audit: dict[str, object] = {
        "strategy": "resolution-wide-per-role-variable-radius-blue-noise",
        "dimensions": 2,
        "confidence_first_priority": True,
        "roles": {},
    }
    for role_index, role in enumerate(("SAND", "ROCK")):
        result_indices = [
            index
            for index, result in enumerate(results)
            if result.accepted_role == role and result.accepted_count > 0
        ]
        if not result_indices:
            audit["roles"][role] = {
                "locally_accepted": 0,
                "globally_accepted": 0,
                "overlap_rejected": 0,
                "spacing": _final_spacing_audit(
                    np.empty((0, 3), np.float64),
                    np.empty(0, np.float64),
                    np.empty((0, 3), np.float64),
                ),
            }
            continue
        record_parts = []
        radius_parts = []
        priority_parts = []
        confidence_parts = []
        target_parts = []
        target_candidate_parts = []
        donor_index_parts = []
        donor_distance_parts = []
        donor_count_parts = []
        spans: list[tuple[int, int, int]] = []
        cursor = 0
        for result_index in result_indices:
            result = results[result_index]
            assert result.accepted_records is not None
            count = int(result.accepted_count)
            if (
                len(result.accepted_records) != count
                or len(result.accepted_radius_m) != count
                or len(result.accepted_priority) != count
                or len(result.accepted_confidence.tier) != count
                or len(result.accepted_candidate_index) != count
                or len(result.accepted_primary_donor_index) != count
                or len(result.accepted_donor_distance_m) != count
                or len(result.accepted_donor_count) != count
            ):
                raise RuntimeError("local target acceptance metadata is misaligned")
            record_parts.append(result.accepted_records)
            radius_parts.append(result.accepted_radius_m)
            priority_parts.append(result.accepted_priority)
            confidence_parts.append(result.accepted_confidence)
            target_parts.append(
                np.full(count, result.target.target_id, dtype=object)
            )
            target_candidate_parts.append(result.accepted_candidate_index)
            donor_index_parts.append(result.accepted_primary_donor_index)
            donor_distance_parts.append(result.accepted_donor_distance_m)
            donor_count_parts.append(result.accepted_donor_count)
            spans.append((result_index, cursor, cursor + count))
            cursor += count
        records = np.concatenate(record_parts)
        radii = np.concatenate(radius_parts)
        local_priority = np.concatenate(priority_parts)
        confidence = _concatenate_confidence(confidence_parts)
        target_ids = np.concatenate(target_parts).astype(str)
        target_candidate_index = np.concatenate(target_candidate_parts)
        target_donor_index = np.concatenate(donor_index_parts)
        target_donor_distance_m = np.concatenate(donor_distance_parts)
        target_donor_count = np.concatenate(donor_count_parts)
        measured_xyz = _xyz(clouds[role].measured_records)
        measured_xyz = measured_xyz[np.all(np.isfinite(measured_xyz), axis=1)]
        # Confidence is the primary order, then local deficit severity.  The
        # coordinate hash inside the selector provides the final stable tie.
        global_priority = confidence.tier.astype(np.float64) * 2.0 + local_priority
        selection = terrain.select_variable_radius_interstitials(
            _xyz(records),
            radii,
            measured_xyz,
            confidence,
            minimum_tier=ConfidenceTier.SUPPORTED,
            priority=global_priority,
            seed=seed ^ ((role_index + 1) * 0x474C4F42414C),
            dimensions=2,
        )
        selected_mask = np.zeros(len(records), dtype=bool)
        selected_mask[selection.selected_indices] = True
        final_records = records[selected_mask]
        final_radii = radii[selected_mask]
        spacing = _final_spacing_audit(
            _xyz(final_records), final_radii, measured_xyz
        )
        if not (
            spacing["measured_clearance_verified"]
            and spacing["candidate_pair_clearance_verified"]
        ):
            raise RuntimeError(f"{resolution.label}/{role}: final spacing audit failed")

        ledger_npz = output_dir / f"global-arbitration-{role.lower()}.npz"
        np.savez_compressed(
            ledger_npz,
            xyz=_xyz(records),
            radius_m=radii,
            local_priority=local_priority,
            global_priority=global_priority,
            target_candidate_index=target_candidate_index,
            target_donor_index=target_donor_index,
            target_donor_distance_m=target_donor_distance_m,
            target_donor_count=target_donor_count,
            confidence_reason_mask=confidence.reason_mask,
            confidence_tier=confidence.tier,
            confidence_surface_spread_m=confidence.surface_spread_m,
            confidence_preferred_gate_count=confidence.preferred_gate_count,
            target_id=target_ids,
            globally_selected=selected_mask,
            disposition=selection.disposition,
            decision_reason_mask=selection.reason_mask,
        )
        target_counts = {}
        for result_index, begin, end in spans:
            original = results[result_index]
            keep = selected_mask[begin:end]
            new_audit = dict(original.audit)
            new_audit["accepted_before_global_arbitration"] = int(end - begin)
            new_audit["accepted"] = int(np.count_nonzero(keep))
            new_audit["global_arbitration"] = {
                "role": role,
                "candidate_span": [int(begin), int(end)],
                "locally_accepted": int(end - begin),
                "globally_accepted": int(np.count_nonzero(keep)),
                "overlap_rejected": int(np.count_nonzero(~keep)),
                "ledger": ledger_npz.name,
            }
            filtered_confidence = _take_confidence(
                original.accepted_confidence, np.flatnonzero(keep)
            )
            updated[result_index] = replace(
                original,
                accepted_records=original.accepted_records[keep],
                accepted_count=int(np.count_nonzero(keep)),
                audit=new_audit,
                accepted_radius_m=original.accepted_radius_m[keep],
                accepted_priority=original.accepted_priority[keep],
                accepted_confidence=filtered_confidence,
                accepted_candidate_index=original.accepted_candidate_index[keep],
                accepted_global_ledger_index=np.arange(
                    begin, end, dtype=np.int64
                )[keep],
                accepted_primary_donor_index=(
                    original.accepted_primary_donor_index[keep]
                ),
                accepted_donor_distance_m=original.accepted_donor_distance_m[keep],
                accepted_donor_count=original.accepted_donor_count[keep],
            )
            target_counts[original.target.target_id] = {
                "locally_accepted": int(end - begin),
                "globally_accepted": int(np.count_nonzero(keep)),
            }
        role_audit = {
            "locally_accepted": int(len(records)),
            "globally_accepted": int(np.count_nonzero(selected_mask)),
            "overlap_rejected": int(np.count_nonzero(~selected_mask)),
            "minimum_tier": ConfidenceTier.SUPPORTED.name,
            "target_counts": target_counts,
            "spacing": spacing,
            "ledger_npz": ledger_npz.name,
            "ledger_npz_sha256": sha256_path(ledger_npz),
        }
        ledger_json = output_dir / f"global-arbitration-{role.lower()}.json"
        _write_resolution_report(ledger_json, role_audit)
        role_audit["ledger_json"] = ledger_json.name
        role_audit["ledger_json_sha256"] = sha256_path(ledger_json)
        audit["roles"][role] = role_audit
        for result_index, _, _ in spans:
            result = updated[result_index]
            refreshed_audit = dict(result.audit)
            refreshed_audit["global_arbitration"]["ledger_npz_sha256"] = (
                role_audit["ledger_npz_sha256"]
            )
            refreshed_audit["global_arbitration"]["ledger_json"] = (
                role_audit["ledger_json"]
            )
            refreshed_audit["global_arbitration"]["ledger_json_sha256"] = (
                role_audit["ledger_json_sha256"]
            )
            updated[result_index] = replace(result, audit=refreshed_audit)
            target_audit_path = (
                output_dir / "targets" / result.target.target_id / "audit.json"
            )
            _write_resolution_report(target_audit_path, refreshed_audit)
    return tuple(updated), audit


@dataclass(frozen=True)
class CrossScaleSubsetSelection:
    selected_fine_indices: np.ndarray
    represented_by_existing: np.ndarray
    existing_support_index: np.ndarray
    existing_xy_distance_m: np.ndarray
    existing_vertical_delta_m: np.ndarray
    coverage_source: np.ndarray
    coverage_index: np.ndarray
    coverage_xy_distance_m: np.ndarray
    coverage_vertical_delta_m: np.ndarray


def _nearest_surface_support_within(
    query_xyz: np.ndarray,
    support_xyz: np.ndarray,
    *,
    maximum_xy_distance_m: float,
    maximum_vertical_delta_m: float,
    tolerance_m: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Find nearest XY support only on a vertically compatible surface."""

    query = np.asarray(query_xyz, np.float64)
    support = np.asarray(support_xyz, np.float64)
    if query.ndim != 2 or query.shape[1] != 3:
        raise ValueError("query_xyz must have shape (N, 3)")
    if support.ndim != 2 or support.shape[1] != 3:
        raise ValueError("support_xyz must have shape (N, 3)")
    if not np.all(np.isfinite(query)) or not np.all(np.isfinite(support)):
        raise ValueError("surface support query must be finite")
    distance = np.full(len(query), np.inf, np.float64)
    index = np.full(len(query), -1, np.int64)
    vertical = np.full(len(query), np.inf, np.float64)
    if not len(query) or not len(support):
        return distance, index, vertical
    search_radius = float(maximum_xy_distance_m) + float(tolerance_m)
    try:
        from scipy.spatial import cKDTree
    except ModuleNotFoundError:  # pragma: no cover - production has scipy
        neighbourhoods = []
        for point in query:
            delta = support[:, :2] - point[:2]
            neighbourhoods.append(
                np.flatnonzero(np.sum(np.square(delta), axis=1) <= search_radius**2)
            )
    else:
        tree = cKDTree(support[:, :2])
        neighbourhoods = tree.query_ball_point(
            query[:, :2], search_radius, workers=-1
        )
    for row, raw_indices in enumerate(neighbourhoods):
        neighbours = np.asarray(raw_indices, np.int64)
        if not len(neighbours):
            continue
        dz = np.abs(support[neighbours, 2] - query[row, 2])
        compatible = dz <= maximum_vertical_delta_m + tolerance_m
        if not np.any(compatible):
            continue
        neighbours = neighbours[compatible]
        dz = dz[compatible]
        delta = support[neighbours, :2] - query[row, :2]
        xy_distance = np.linalg.norm(delta, axis=1)
        within = xy_distance <= maximum_xy_distance_m + tolerance_m
        if not np.any(within):
            continue
        neighbours = neighbours[within]
        dz = dz[within]
        xy_distance = xy_distance[within]
        order = np.lexsort((neighbours, dz, xy_distance))
        chosen = int(order[0])
        index[row] = int(neighbours[chosen])
        distance[row] = float(xy_distance[chosen])
        vertical[row] = float(dz[chosen])
    return distance, index, vertical


def _cross_scale_hash(points: np.ndarray, seed: int) -> np.ndarray:
    """Stable coordinate hash used only to break equal-priority ties."""

    canonical = np.asarray(points, dtype="<f8").copy()
    canonical[canonical == 0.0] = 0.0
    seed_bytes = int(seed & 0xFFFFFFFFFFFFFFFF).to_bytes(8, "little")
    return np.asarray(
        [
            int.from_bytes(
                hashlib.blake2b(
                    seed_bytes + point.tobytes(order="C"), digest_size=8
                ).digest(),
                "little",
            )
            for point in canonical
        ],
        np.uint64,
    )


def select_cross_scale_fine_subset(
    fine_xyz: np.ndarray,
    existing_xyz: np.ndarray,
    priority: np.ndarray,
    *,
    selection_eligible: np.ndarray | None = None,
    spacing_m: float,
    vertical_tolerance_m: float,
    distance_tolerance_m: float,
    seed: int,
) -> CrossScaleSubsetSelection:
    """Return a deterministic maximal surface-aware subset of fine geometry.

    Existing terrain and selected fine points only cover another fine point when
    both XY distance and vertical separation pass their explicit guards.  This
    prevents a floating/noisy return from suppressing the real surface below.
    """

    fine = np.asarray(fine_xyz, np.float64)
    existing = np.asarray(existing_xyz, np.float64)
    priorities = np.asarray(priority, np.float64)
    if fine.ndim != 2 or fine.shape[1] != 3:
        raise ValueError("fine_xyz must have shape (N, 3)")
    if existing.ndim != 2 or existing.shape[1] != 3:
        raise ValueError("existing_xyz must have shape (N, 3)")
    if priorities.shape != (len(fine),):
        raise ValueError("priority must contain one value per fine point")
    eligible = (
        np.ones(len(fine), dtype=bool)
        if selection_eligible is None
        else np.asarray(selection_eligible, dtype=bool)
    )
    if eligible.shape != (len(fine),):
        raise ValueError("selection_eligible must contain one value per fine point")
    if not np.all(np.isfinite(fine)) or not np.all(np.isfinite(existing)):
        raise ValueError("cross-scale geometry must be finite")
    if not np.all(np.isfinite(priorities)):
        raise ValueError("cross-scale priority must be finite")
    if spacing_m <= 0.0 or vertical_tolerance_m <= 0.0:
        raise ValueError("cross-scale spacing/tolerance must be positive")

    existing_distance, existing_index, existing_vertical = (
        _nearest_surface_support_within(
            fine,
            existing,
            maximum_xy_distance_m=spacing_m,
            maximum_vertical_delta_m=vertical_tolerance_m,
            tolerance_m=distance_tolerance_m,
        )
    )
    conflict_limit = max(float(spacing_m) - float(distance_tolerance_m), 0.0)
    represented = existing_index >= 0
    uncovered = np.flatnonzero(~represented & eligible)
    hashes = _cross_scale_hash(fine[uncovered], seed)
    order = uncovered[
        np.lexsort((uncovered, hashes, -priorities[uncovered]))
    ]
    selected: list[int] = []
    selected_array = np.empty((0, 3), np.float64)
    for raw_index in order:
        index = int(raw_index)
        if len(selected_array):
            delta = selected_array - fine[index]
            xy_distance = np.linalg.norm(delta[:, :2], axis=1)
            vertical_delta = np.abs(delta[:, 2])
            if np.any(
                (xy_distance < conflict_limit)
                & (
                    vertical_delta
                    <= float(vertical_tolerance_m) + float(distance_tolerance_m)
                )
            ):
                continue
        selected.append(index)
        selected_array = fine[np.asarray(selected, np.int64)]
    selected_indices = np.asarray(selected, np.int64)

    selected_distance, selected_witness, selected_vertical = (
        _nearest_surface_support_within(
            fine,
            fine[selected_indices],
            maximum_xy_distance_m=spacing_m,
            maximum_vertical_delta_m=vertical_tolerance_m,
            tolerance_m=distance_tolerance_m,
        )
    )
    covered_by_selected = selected_witness >= 0
    covered = represented | covered_by_selected
    if not np.all(covered):
        missing = np.flatnonzero(~covered)
        raise RuntimeError(
            "fine points cannot be covered by compatible existing terrain or "
            "a donor-eligible maximal coarse subset; first fine indices "
            f"{missing[:8].tolist()}"
        )
    if len(selected_indices) and np.any(
        existing_distance[selected_indices] < conflict_limit
    ):
        raise RuntimeError("cross-scale subset violates existing terrain clearance")

    coverage_source = np.zeros(len(fine), np.uint8)
    coverage_index = np.full(len(fine), -1, np.int64)
    coverage_distance = np.full(len(fine), np.inf, np.float64)
    coverage_vertical = np.full(len(fine), np.inf, np.float64)
    coverage_source[represented] = 1
    coverage_index[represented] = existing_index[represented]
    coverage_distance[represented] = existing_distance[represented]
    coverage_vertical[represented] = existing_vertical[represented]
    use_selected = ~represented & covered_by_selected
    coverage_source[use_selected] = 2
    coverage_index[use_selected] = selected_indices[selected_witness[use_selected]]
    coverage_distance[use_selected] = selected_distance[use_selected]
    coverage_vertical[use_selected] = selected_vertical[use_selected]
    if np.any(coverage_distance > spacing_m + distance_tolerance_m):
        raise RuntimeError("cross-scale coverage exceeds coarse spacing")
    if np.any(coverage_vertical > vertical_tolerance_m + distance_tolerance_m):
        raise RuntimeError("cross-scale coverage violates vertical support guard")
    return CrossScaleSubsetSelection(
        selected_indices,
        represented,
        existing_index,
        existing_distance,
        existing_vertical,
        coverage_source,
        coverage_index,
        coverage_distance,
        coverage_vertical,
    )


def _compatible_donor_neighbourhoods(
    query_xyz: np.ndarray,
    donor_xyz: np.ndarray,
    *,
    maximum_distance_m: float,
    maximum_vertical_delta_m: float,
    distance_tolerance_m: float,
    maximum_neighbours: int = 32,
) -> list[np.ndarray]:
    """Resolve same-surface donor neighbourhoods with deterministic tie order."""

    query = np.asarray(query_xyz, np.float64)
    donors = np.asarray(donor_xyz, np.float64)
    if not len(query):
        return []
    if not len(donors):
        return [np.empty(0, np.int64) for _ in query]
    radius = float(maximum_distance_m) + float(distance_tolerance_m)
    try:
        from scipy.spatial import cKDTree
    except ModuleNotFoundError:  # pragma: no cover - production has scipy
        raw = [
            np.flatnonzero(
                np.sum(np.square(donors[:, :2] - point[:2]), axis=1)
                <= radius**2
            )
            for point in query
        ]
    else:
        raw = cKDTree(donors[:, :2]).query_ball_point(
            query[:, :2], radius, workers=-1
        )
    result: list[np.ndarray] = []
    for point, raw_indices in zip(query, raw, strict=True):
        indices = np.asarray(raw_indices, np.int64)
        if not len(indices):
            result.append(indices)
            continue
        delta = donors[indices] - point
        distance = np.linalg.norm(delta, axis=1)
        compatible = (
            (np.abs(delta[:, 2]) <= maximum_vertical_delta_m + distance_tolerance_m)
            & (distance <= maximum_distance_m + distance_tolerance_m)
        )
        indices = indices[compatible]
        distance = distance[compatible]
        if len(indices):
            order = np.lexsort((indices, distance))[: int(maximum_neighbours)]
            indices = indices[order]
        result.append(indices)
    return result


def _cross_scale_initial_records(
    fine_records: np.ndarray,
    selected_fine_indices: np.ndarray,
    coarse_cloud: LocalRoleCloud,
    *,
    role: str,
    maximum_donor_distance_m: float,
    vertical_tolerance_m: float,
    distance_tolerance_m: float,
) -> tuple[np.ndarray, Mapping[str, np.ndarray]]:
    """Transfer properties from vertically compatible, same-role coarse donors."""

    selected = np.asarray(selected_fine_indices, np.int64)
    measured_mask = coarse_cloud.measured_mask
    donors = coarse_cloud.records[measured_mask]
    donor_source_index = coarse_cloud.original_indices[measured_mask]
    donor_xyz = _xyz(donors)
    finite = np.all(np.isfinite(donor_xyz), axis=1)
    donors = donors[finite]
    donor_xyz = donor_xyz[finite]
    donor_source_index = donor_source_index[finite]
    query = _xyz(fine_records[selected])
    neighbourhoods = _compatible_donor_neighbourhoods(
        query,
        donor_xyz,
        maximum_distance_m=maximum_donor_distance_m,
        maximum_vertical_delta_m=vertical_tolerance_m,
        distance_tolerance_m=distance_tolerance_m,
    )
    unresolved = [row for row, indices in enumerate(neighbourhoods) if not len(indices)]
    if unresolved:
        fine_indices = selected[np.asarray(unresolved, np.int64)]
        raise RuntimeError(
            f"5mm/{role}: {len(unresolved)} selected fine points have no "
            "vertically compatible same-role measured donor; first fine indices "
            f"{fine_indices[:8].tolist()}"
        )
    output_parts: list[np.ndarray] = []
    primary_source = np.empty(len(selected), np.int64)
    nearest_distance = np.empty(len(selected), np.float64)
    contributing_count = np.empty(len(selected), np.int32)
    names = set(fine_records.dtype.names or ())
    if {"nx", "ny", "nz"}.issubset(names):
        normals = np.column_stack(
            (
                fine_records["nx"][selected],
                fine_records["ny"][selected],
                fine_records["nz"][selected],
            )
        ).astype(np.float64)
    else:
        normals = None
    for row, donor_indices in enumerate(neighbourhoods):
        predicted = normals[row : row + 1] if normals is not None else None
        sampled = terrain.build_scanid10_additions(
            query[row : row + 1],
            donors[donor_indices],
            role=role,
            donor_role=role,
            maximum_donor_distance_m=maximum_donor_distance_m,
            neighbours=min(8, len(donor_indices)),
            predicted_normals=predicted,
        )
        if len(sampled.records) != 1 or len(sampled.unresolved_query_indices):
            raise RuntimeError(f"5mm/{role}: compatible donor transfer failed closed")
        local_primary = int(sampled.primary_donor_indices[0])
        primary_source[row] = int(donor_source_index[donor_indices[local_primary]])
        nearest_distance[row] = float(sampled.nearest_distance_m[0])
        contributing_count[row] = int(sampled.contributing_donor_count[0])
        output_parts.append(sampled.records)
    records = (
        np.concatenate(output_parts)
        if output_parts
        else np.empty(0, dtype=coarse_cloud.layout.dtype)
    )
    if len(records):
        fine_xyz = _xyz(fine_records[selected])
        if not np.array_equal(_xyz(records), fine_xyz):
            raise RuntimeError(f"5mm/{role}: donor transfer changed authoritative XYZ")
    return records, {
        "coarse_primary_donor_source_index": primary_source,
        "coarse_nearest_donor_distance_m": nearest_distance,
        "coarse_contributing_donor_count": contributing_count,
    }


CROSS_SCALE_ARCHIVE_KEYS = frozenset(
    {
        "records",
        "fine_selection_index",
        "fine_target_id",
        "fine_target_candidate_index",
        "fine_global_ledger_index",
        "fine_confidence_reason_mask",
        "fine_confidence_tier",
        "fine_confidence_surface_spread_m",
        "fine_confidence_preferred_gate_count",
        "coarse_primary_donor_source_index",
        "coarse_nearest_donor_distance_m",
        "coarse_contributing_donor_count",
    }
)


def _cross_scale_archive_arrays(
    records: np.ndarray,
    fine_archive: Mapping[str, np.ndarray],
    selected_fine_indices: np.ndarray,
    donor_metadata: Mapping[str, np.ndarray],
) -> Mapping[str, np.ndarray]:
    selected = np.asarray(selected_fine_indices, np.int64)
    arrays = {
        "records": np.asarray(records).copy(),
        "fine_selection_index": selected.copy(),
        "fine_target_id": fine_archive["target_id"][selected].copy(),
        "fine_target_candidate_index": fine_archive["target_candidate_index"][
            selected
        ].copy(),
        "fine_global_ledger_index": fine_archive["global_ledger_index"][
            selected
        ].copy(),
        "fine_confidence_reason_mask": fine_archive["confidence_reason_mask"][
            selected
        ].copy(),
        "fine_confidence_tier": fine_archive["confidence_tier"][selected].copy(),
        "fine_confidence_surface_spread_m": fine_archive[
            "confidence_surface_spread_m"
        ][selected].copy(),
        "fine_confidence_preferred_gate_count": fine_archive[
            "confidence_preferred_gate_count"
        ][selected].copy(),
        **{key: np.asarray(value).copy() for key, value in donor_metadata.items()},
    }
    if set(arrays) != CROSS_SCALE_ARCHIVE_KEYS:
        raise RuntimeError("internal error: cross-scale archive schema drift")
    for name, value in arrays.items():
        if value.ndim != 1 or len(value) != len(records):
            raise RuntimeError(f"cross-scale archive field {name} is misaligned")
    if len(np.unique(selected)) != len(selected):
        raise RuntimeError("cross-scale archive maps a fine point more than once")
    return arrays


def build_resolution_candidates(
    *,
    sand_path: str | Path,
    rock_base_path: str | Path,
    targets: Sequence[terrain.TerrainReviewTarget],
    output_dir: Path,
    cleanmesh_executable: Path,
    resolution: ResolutionParameters,
    pipeline: PipelineParameters,
    combined_normalizations: Mapping[str, Mapping[str, float]] | None = None,
    cleanmesh_runner: CleanMeshRunner = run_cleanmesh_reduced_analysis,
) -> ResolutionBuildResult:
    """Build SAND and ROCK candidates for one resolution in an empty directory."""

    if output_dir.exists():
        raise FileExistsError(f"refusing existing resolution output: {output_dir}")
    output_dir.mkdir(parents=True)
    sand = collect_local_role_cloud(
        sand_path,
        role="SAND",
        targets=targets,
        collar_m=max(resolution.source_collar_m, resolution.cleanmesh_collar_m),
        chunk_records=pipeline.chunk_records,
    )
    rock = collect_local_role_cloud(
        rock_base_path,
        role="ROCK",
        targets=targets,
        collar_m=max(resolution.source_collar_m, resolution.cleanmesh_collar_m),
        chunk_records=pipeline.chunk_records,
    )
    if sand.layout.dtype != rock.layout.dtype:
        raise ValueError(f"{resolution.label}: SAND and ROCK schemas differ")
    clouds = {"SAND": sand, "ROCK": rock}
    target_output = output_dir / "targets"
    results: list[TargetBuildResult] = []
    for index, target in enumerate(targets):
        results.append(
            build_target_additions(
                target,
                clouds,
                target_output,
                resolution,
                pipeline,
                seed=pipeline.seed ^ (index + 1) ^ (1 if resolution.label == "1mm" else 5),
            )
        )

    arbitrated, global_arbitration_audit = arbitrate_resolution_additions(
        results,
        clouds,
        output_dir,
        resolution,
        seed=pipeline.seed ^ (0x1 if resolution.label == "1mm" else 0x5),
    )
    results = list(arbitrated)

    accepted = [result for result in results if result.accepted_count > 0]
    additions_by_role: dict[str, list[np.ndarray]] = {"SAND": [], "ROCK": []}
    cleanmesh_audit: Mapping[str, object]
    if accepted:
        accepted_targets = [result.target for result in accepted]
        selections = [
            selection
            for selection in (
                _collar_source_selection(
                    sand, accepted_targets, resolution.cleanmesh_collar_m
                ),
                _collar_source_selection(
                    rock, accepted_targets, resolution.cleanmesh_collar_m
                ),
            )
            if selection is not None
        ]
        if not selections:
            raise RuntimeError("accepted additions have no measured CleanMesh collar")
        batches = [
            terrain.LocalAdditionBatch(
                result.accepted_records,
                result.accepted_role,
                result.target.target_id,
            )
            for result in accepted
            if result.accepted_records is not None and result.accepted_role is not None
        ]
        analysis_input = output_dir / "local-collars.analysis-input.ply"
        local_manifest = terrain.write_local_analysis_input(
            analysis_input,
            selections,
            batches,
            chunk_size=pipeline.chunk_records,
        )
        analysed_path = output_dir / "local-collars.analysis.ply"
        cleanmesh_report = output_dir / "local-collars.cleanmesh-report.json"
        runner_audit = cleanmesh_runner(
            cleanmesh_executable,
            analysis_input,
            analysed_path,
            cleanmesh_report,
            resolution,
            pipeline,
        )
        analysed_batches = terrain.extract_cleanmesh_analysed_additions(
            analysed_path, local_manifest
        )
        expected_labels = [batch.label for batch in batches]
        observed_labels = [batch.label for batch in analysed_batches]
        if observed_labels != expected_labels:
            raise RuntimeError("CleanMesh addition labels/order changed")
        scalar_postprocess = []
        for initial_batch, batch in zip(batches, analysed_batches, strict=True):
            if batch.role != initial_batch.role:
                raise RuntimeError("CleanMesh addition roles/order changed")
            projected = terrain.project_analysed_additions(
                batch.records, sand.layout.dtype
            )
            scalar_postprocess.append(
                {
                    "label": batch.label,
                    "role": batch.role,
                    "points": int(len(projected)),
                    **postprocess_local_analysed_additions(
                        projected,
                        initial_batch.records,
                        combined_normalizations,
                    ),
                }
            )
            additions_by_role[batch.role].append(projected)
        cleanmesh_audit = {
            "status": "completed",
            "local_input": analysis_input.name,
            "local_input_sha256": sha256_path(analysis_input),
            "local_manifest": local_manifest.manifest_path.name,
            "local_manifest_sha256": sha256_path(local_manifest.manifest_path),
            "analysed": analysed_path.name,
            "analysed_sha256": sha256_path(analysed_path),
            "report": cleanmesh_report.name,
            "report_sha256": sha256_path(cleanmesh_report),
            "runner": dict(runner_audit),
            "collar_points": int(sum(len(value.indices) for value in selections)),
            "addition_points": int(sum(len(batch.records) for batch in batches)),
            "full_cloud_analysis": False,
            "span_identity_verified": True,
            "scalar_postprocess": scalar_postprocess,
        }
    else:
        cleanmesh_audit = {
            "status": "skipped-no-accepted-additions",
            "full_cloud_analysis": False,
        }

    candidate_paths: dict[str, str] = {}
    candidate_hashes: dict[str, str] = {}
    addition_counts: dict[str, int] = {}
    append_audits: dict[str, object] = {}
    addition_archive_paths: dict[str, str] = {}
    addition_archive_hashes: dict[str, str] = {}
    addition_archive_audits: dict[str, object] = {}
    for role, cloud in (("SAND", sand), ("ROCK", rock)):
        additions = (
            np.concatenate(additions_by_role[role])
            if additions_by_role[role]
            else np.empty(0, dtype=cloud.layout.dtype)
        )
        candidate = output_dir / _candidate_filename(role, resolution.label)
        report = terrain.write_append_only_candidate(
            cloud.layout.path,
            additions,
            candidate,
            chunk_size=pipeline.chunk_records,
        )
        candidate_paths[role] = str(candidate)
        candidate_hashes[role] = sha256_path(candidate)
        addition_counts[role] = int(len(additions))
        append_audit = dict(report)
        append_audit["candidate_path"] = candidate.name
        append_audits[role] = append_audit
        archive_arrays = _fine_archive_arrays(role, additions, results)
        archive_path = output_dir / f"authoritative-additions-{role.lower()}.npz"
        _write_deterministic_npz(archive_path, archive_arrays)
        _verify_candidate_suffix(
            candidate,
            source_count=cloud.layout.vertex_count,
            expected_records=archive_arrays["records"],
        )
        archive_hash = sha256_path(archive_path)
        addition_archive_paths[role] = str(archive_path)
        addition_archive_hashes[role] = archive_hash
        selected_results = [
            result
            for result in results
            if result.accepted_role == role and result.accepted_count > 0
        ]
        role_arbitration = global_arbitration_audit["roles"][role]
        addition_archive_audits[role] = {
            "path": archive_path.name,
            "sha256": archive_hash,
            "keys": sorted(AUTHORITATIVE_ARCHIVE_KEYS),
            "points": int(len(additions)),
            "record_stride": int(additions.dtype.itemsize),
            "record_payload_sha256": _record_payload_sha256(additions),
            "candidate_suffix_byte_exact": True,
            "candidate_sha256": candidate_hashes[role],
            "global_arbitration_ledger": role_arbitration.get("ledger_npz"),
            "global_arbitration_ledger_sha256": role_arbitration.get(
                "ledger_npz_sha256"
            ),
            "target_provenance": {
                result.target.target_id: dict(result.audit["provenance"])
                for result in selected_results
            },
        }

    report_path = output_dir / "resolution-report.json"
    _write_resolution_report(
        report_path,
        {
            "schema_version": 1,
            "candidate_only": True,
            "canonical_writes": False,
            "resolution": asdict(resolution),
            "targets": [dict(result.audit) for result in results],
            "global_arbitration": global_arbitration_audit,
            "cleanmesh": cleanmesh_audit,
            "append_only": append_audits,
            "authoritative_addition_archives": addition_archive_audits,
            "addition_counts": addition_counts,
            "invariants": {
                "annotations_are_search_windows_only": True,
                "actual_connected_deficit_required": True,
                "same_role_measured_property_donors_only": True,
                "overlapping_targets_globally_arbitrated": True,
                "addition_scan_id": terrain.ADDITION_SCAN_ID,
                "crack_minimum_tier": ConfidenceTier.STRONG.name,
                "marked_scanner_minimum_tier": ConfidenceTier.SUPPORTED.name,
                "existing_base_payload_byte_exact": True,
                "authoritative_archive_equals_candidate_suffix": True,
            },
        },
    )
    return ResolutionBuildResult(
        label=resolution.label,
        candidate_paths=candidate_paths,
        candidate_sha256=candidate_hashes,
        addition_counts=addition_counts,
        report_path=str(report_path),
        report_sha256=sha256_path(report_path),
        target_count=len(results),
        addition_archive_paths=addition_archive_paths,
        addition_archive_sha256=addition_archive_hashes,
    )


def _verify_fine_artifact_closure(
    fine: ResolutionBuildResult,
) -> tuple[Mapping[str, object], Mapping[str, dict[str, np.ndarray]]]:
    """Verify the full fine report/archive/provenance closure before reuse."""

    report_path = Path(fine.report_path)
    if sha256_path(report_path) != fine.report_sha256:
        raise RuntimeError("fine resolution report hash changed before coarse build")
    report = json.loads(report_path.read_text(encoding="utf-8"))
    if report.get("candidate_only") is not True or report.get("canonical_writes") is not False:
        raise RuntimeError("fine resolution report is not candidate-only")
    if report.get("addition_counts") != dict(fine.addition_counts):
        raise RuntimeError("fine resolution report addition counts differ")
    archive_report = report.get("authoritative_addition_archives")
    if not isinstance(archive_report, Mapping):
        raise RuntimeError("fine resolution report has no authoritative archives")
    loaded_archives: dict[str, dict[str, np.ndarray]] = {}
    for role in ("SAND", "ROCK"):
        candidate_path = Path(fine.candidate_paths[role])
        if sha256_path(candidate_path) != fine.candidate_sha256[role]:
            raise RuntimeError(f"fine {role} candidate hash changed before coarse build")
        candidate_layout = terrain.inspect_fixed_stride_ply(candidate_path)
        archive_path = Path(fine.addition_archive_paths[role])
        expected_archive_hash = fine.addition_archive_sha256[role]
        role_report = archive_report.get(role)
        if not isinstance(role_report, Mapping):
            raise RuntimeError(f"fine {role} archive report is absent")
        if (
            role_report.get("sha256") != expected_archive_hash
            or Path(str(role_report.get("path", ""))).name != archive_path.name
        ):
            raise RuntimeError(f"fine {role} archive report differs from result")
        arrays = _load_authoritative_archive(
            archive_path,
            expected_sha256=expected_archive_hash,
            expected_count=fine.addition_counts[role],
            expected_dtype=candidate_layout.dtype,
        )
        append_report = report.get("append_only", {}).get(role)
        if not isinstance(append_report, Mapping):
            raise RuntimeError(f"fine {role} append-only report is absent")
        source_count = int(append_report.get("source_vertex_count", -1))
        _verify_candidate_suffix(
            candidate_path,
            source_count=source_count,
            expected_records=arrays["records"],
        )
        if role_report.get("record_payload_sha256") != _record_payload_sha256(
            arrays["records"]
        ):
            raise RuntimeError(f"fine {role} archive payload hash differs")
        arbitration = report.get("global_arbitration", {}).get("roles", {}).get(role)
        if not isinstance(arbitration, Mapping):
            raise RuntimeError(f"fine {role} global arbitration report is absent")
        if len(arrays["records"]):
            ledger_name = str(arbitration.get("ledger_npz", ""))
            if Path(ledger_name).name != ledger_name:
                raise RuntimeError("fine global arbitration ledger path is unsafe")
            ledger_path = report_path.parent / ledger_name
            if sha256_path(ledger_path) != arbitration.get("ledger_npz_sha256"):
                raise RuntimeError(f"fine {role} global arbitration ledger hash changed")
            with np.load(ledger_path, allow_pickle=False) as ledger:
                global_index = arrays["global_ledger_index"]
                if np.any(global_index < 0) or np.any(
                    global_index >= len(ledger["globally_selected"])
                ):
                    raise RuntimeError("fine archive global ledger index is invalid")
                if not np.all(ledger["globally_selected"][global_index]):
                    raise RuntimeError("fine archive maps to a rejected global ledger row")
                if not np.array_equal(
                    ledger["target_candidate_index"][global_index],
                    arrays["target_candidate_index"],
                ):
                    raise RuntimeError("fine archive target candidate mapping differs")
                if not np.array_equal(
                    ledger["xyz"][global_index], _xyz(arrays["records"])
                ):
                    raise RuntimeError("fine archive XYZ differs from global ledger")
                if not np.array_equal(
                    ledger["target_id"][global_index].astype(str),
                    arrays["target_id"].astype(str),
                ):
                    raise RuntimeError("fine archive target IDs differ from global ledger")
                if not np.array_equal(
                    ledger["radius_m"][global_index], arrays["radius_m"]
                ) or not np.array_equal(
                    ledger["local_priority"][global_index], arrays["priority"]
                ):
                    raise RuntimeError("fine archive selection metadata differs")
                if not np.array_equal(
                    ledger["confidence_reason_mask"][global_index],
                    arrays["confidence_reason_mask"],
                ):
                    raise RuntimeError("fine archive confidence provenance differs")
                for ledger_name, archive_name in (
                    ("confidence_tier", "confidence_tier"),
                    (
                        "confidence_surface_spread_m",
                        "confidence_surface_spread_m",
                    ),
                    (
                        "confidence_preferred_gate_count",
                        "confidence_preferred_gate_count",
                    ),
                    ("target_donor_index", "target_donor_index"),
                    ("target_donor_distance_m", "target_donor_distance_m"),
                    ("target_donor_count", "target_donor_count"),
                ):
                    if not np.array_equal(
                        ledger[ledger_name][global_index], arrays[archive_name]
                    ):
                        raise RuntimeError(
                            f"fine archive provenance differs for {archive_name}"
                        )
        target_provenance = role_report.get("target_provenance", {})
        if not isinstance(target_provenance, Mapping):
            raise RuntimeError("fine target provenance is not a mapping")
        for target_id in np.unique(arrays["target_id"]):
            entry = target_provenance.get(str(target_id))
            if not isinstance(entry, Mapping):
                raise RuntimeError(f"fine target provenance missing for {target_id}")
            for suffix in ("npz", "json"):
                relative = Path(str(entry.get(suffix, "")))
                resolved = (report_path.parent / relative).resolve(strict=True)
                if not resolved.is_relative_to(report_path.parent.resolve()):
                    raise RuntimeError("fine target provenance escaped resolution directory")
                if sha256_path(resolved) != entry.get(f"{suffix}_sha256"):
                    raise RuntimeError(
                        f"fine target provenance hash changed for {target_id}/{suffix}"
                    )
            target_npz = (
                report_path.parent / Path(str(entry["npz"]))
            ).resolve(strict=True)
            with np.load(target_npz, allow_pickle=False) as provenance:
                required = {
                    "xyz",
                    "target_id",
                    "disposition",
                    "decision_reason_mask",
                    "confidence_reason_mask",
                    "confidence_tier",
                    "surface_spread_m",
                    "donor_index",
                    "donor_distance_m",
                }
                if set(provenance.files) != required:
                    raise RuntimeError("fine target provenance keys differ from schema")
                rows = np.flatnonzero(arrays["target_id"].astype(str) == str(target_id))
                candidate_index = arrays["target_candidate_index"][rows]
                if np.any(candidate_index < 0) or np.any(
                    candidate_index >= len(provenance["xyz"])
                ):
                    raise RuntimeError("fine target candidate index is invalid")
                provenance_xyz = np.asarray(
                    provenance["xyz"][candidate_index], np.float64
                ).copy()
                for axis, field_name in enumerate(("x", "y", "z")):
                    provenance_xyz[:, axis] = provenance_xyz[:, axis].astype(
                        arrays["records"].dtype[field_name]
                    )
                if not np.array_equal(
                    provenance_xyz, _xyz(arrays["records"])[rows]
                ):
                    raise RuntimeError("fine target provenance XYZ differs")
                if not np.all(
                    provenance["disposition"][candidate_index]
                    == int(terrain.TerrainDisposition.ACCEPTED)
                ) or np.any(provenance["decision_reason_mask"][candidate_index] != 0):
                    raise RuntimeError("fine archive maps to rejected target provenance")
                if not np.array_equal(
                    provenance["confidence_reason_mask"][candidate_index],
                    arrays["confidence_reason_mask"][rows],
                ) or not np.array_equal(
                    provenance["confidence_tier"][candidate_index],
                    arrays["confidence_tier"][rows],
                ):
                    raise RuntimeError("fine target confidence provenance differs")
                if not np.array_equal(
                    provenance["surface_spread_m"][candidate_index],
                    arrays["confidence_surface_spread_m"][rows],
                ) or not np.array_equal(
                    provenance["donor_index"][candidate_index],
                    arrays["target_donor_index"][rows],
                ):
                    raise RuntimeError("fine target geometry/donor provenance differs")
                if not np.array_equal(
                    provenance["donor_distance_m"][candidate_index],
                    arrays["target_donor_distance_m"][rows],
                ):
                    raise RuntimeError("fine target donor distance provenance differs")
        loaded_archives[role] = arrays
    return report, loaded_archives


def _cross_scale_pair_audit(
    xyz: np.ndarray,
    *,
    spacing_m: float,
    vertical_tolerance_m: float,
    distance_tolerance_m: float,
) -> Mapping[str, object]:
    points = np.asarray(xyz, np.float64)
    if len(points) < 2:
        return {
            "minimum_vertical_compatible_pair_xy_distance_m": None,
            "verified": True,
        }
    minimum = np.inf
    for row in range(len(points) - 1):
        delta = points[row + 1 :] - points[row]
        compatible = (
            np.abs(delta[:, 2])
            <= vertical_tolerance_m + distance_tolerance_m
        )
        if np.any(compatible):
            minimum = min(
                minimum,
                float(np.min(np.linalg.norm(delta[compatible, :2], axis=1))),
            )
    verified = not np.isfinite(minimum) or minimum >= spacing_m - distance_tolerance_m
    return {
        "minimum_vertical_compatible_pair_xy_distance_m": (
            float(minimum) if np.isfinite(minimum) else None
        ),
        "verified": bool(verified),
    }


def build_cross_scale_coarse_candidates(
    *,
    sand_path: str | Path,
    rock_base_path: str | Path,
    fine: ResolutionBuildResult,
    targets: Sequence[terrain.TerrainReviewTarget],
    output_dir: Path,
    cleanmesh_executable: Path,
    resolution: ResolutionParameters,
    pipeline: PipelineParameters,
    combined_normalizations: Mapping[str, Mapping[str, float]] | None = None,
    cleanmesh_runner: CleanMeshRunner = run_cleanmesh_reduced_analysis,
) -> ResolutionBuildResult:
    """Derive every coarse addition as an exact XYZ subset of final fine rows."""

    if resolution.label != "5mm":
        raise ValueError("cross-scale coarse builder requires the 5mm resolution")
    if fine.label != "1mm":
        raise ValueError("cross-scale coarse builder requires a 1mm authority")
    if output_dir.exists():
        raise FileExistsError(f"refusing existing resolution output: {output_dir}")
    _, fine_archives = _verify_fine_artifact_closure(fine)
    output_dir.mkdir(parents=True)
    sand = collect_local_role_cloud(
        sand_path,
        role="SAND",
        targets=targets,
        collar_m=max(resolution.source_collar_m, resolution.cleanmesh_collar_m),
        chunk_records=pipeline.chunk_records,
    )
    rock = collect_local_role_cloud(
        rock_base_path,
        role="ROCK",
        targets=targets,
        collar_m=max(resolution.source_collar_m, resolution.cleanmesh_collar_m),
        chunk_records=pipeline.chunk_records,
    )
    if sand.layout.dtype != rock.layout.dtype:
        raise ValueError("5mm: SAND and ROCK schemas differ")
    clouds = {"SAND": sand, "ROCK": rock}
    target_by_id = {target.target_id: target for target in targets}
    selections_by_role: dict[str, CrossScaleSubsetSelection] = {}
    donor_eligible_by_role: dict[str, np.ndarray] = {}
    initial_by_role: dict[str, np.ndarray] = {}
    donor_metadata_by_role: dict[str, Mapping[str, np.ndarray]] = {}
    cross_role_audit: dict[str, object] = {}
    for role_index, role in enumerate(("SAND", "ROCK")):
        archive = fine_archives[role]
        fine_records = archive["records"]
        if fine_records.dtype != clouds[role].layout.dtype:
            raise RuntimeError(
                f"1mm/5mm {role} schemas differ; exact record projection is unsafe"
            )
        fine_xyz = _xyz(fine_records)
        measured_xyz = _xyz(clouds[role].measured_records)
        measured_xyz = measured_xyz[np.all(np.isfinite(measured_xyz), axis=1)]
        donor_neighbourhoods = _compatible_donor_neighbourhoods(
            fine_xyz,
            measured_xyz,
            maximum_distance_m=resolution.property_donor_distance_m,
            maximum_vertical_delta_m=pipeline.cross_scale_vertical_tolerance_m,
            distance_tolerance_m=pipeline.cross_scale_distance_tolerance_m,
        )
        donor_eligible = np.asarray(
            [bool(len(indices)) for indices in donor_neighbourhoods], dtype=bool
        )
        priority = (
            archive["confidence_tier"].astype(np.float64) * 2.0
            + archive["priority"].astype(np.float64)
        )
        subset = select_cross_scale_fine_subset(
            fine_xyz,
            measured_xyz,
            priority,
            selection_eligible=donor_eligible,
            spacing_m=resolution.nominal_spacing_m,
            vertical_tolerance_m=pipeline.cross_scale_vertical_tolerance_m,
            distance_tolerance_m=pipeline.cross_scale_distance_tolerance_m,
            seed=pipeline.seed ^ ((role_index + 1) * 0x43524F5353534341),
        )
        initial, donor_metadata = _cross_scale_initial_records(
            fine_records,
            subset.selected_fine_indices,
            clouds[role],
            role=role,
            maximum_donor_distance_m=resolution.property_donor_distance_m,
            vertical_tolerance_m=pipeline.cross_scale_vertical_tolerance_m,
            distance_tolerance_m=pipeline.cross_scale_distance_tolerance_m,
        )
        selections_by_role[role] = subset
        donor_eligible_by_role[role] = donor_eligible
        initial_by_role[role] = initial
        donor_metadata_by_role[role] = donor_metadata

    accepted_target_ids = {
        str(target_id)
        for role in ("SAND", "ROCK")
        for target_id in fine_archives[role]["target_id"][
            selections_by_role[role].selected_fine_indices
        ]
    }
    unknown_targets = sorted(accepted_target_ids - set(target_by_id))
    if unknown_targets:
        raise RuntimeError(
            f"cross-scale archive refers to unknown targets: {unknown_targets}"
        )
    accepted_targets = [
        target for target in targets if target.target_id in accepted_target_ids
    ]
    batches = [
        terrain.LocalAdditionBatch(
            initial_by_role[role], role, f"cross-scale-{role.lower()}-fine-subset"
        )
        for role in ("SAND", "ROCK")
        if len(initial_by_role[role])
    ]
    analysed_by_role: dict[str, np.ndarray] = {
        role: np.empty(0, dtype=clouds[role].layout.dtype)
        for role in ("SAND", "ROCK")
    }
    if batches:
        collar_selections = [
            selection
            for selection in (
                _collar_source_selection(
                    sand, accepted_targets, resolution.cleanmesh_collar_m
                ),
                _collar_source_selection(
                    rock, accepted_targets, resolution.cleanmesh_collar_m
                ),
            )
            if selection is not None
        ]
        if not collar_selections:
            raise RuntimeError("cross-scale additions have no measured CleanMesh collar")
        analysis_input = output_dir / "local-collars.analysis-input.ply"
        local_manifest = terrain.write_local_analysis_input(
            analysis_input,
            collar_selections,
            batches,
            chunk_size=pipeline.chunk_records,
        )
        analysed_path = output_dir / "local-collars.analysis.ply"
        cleanmesh_report = output_dir / "local-collars.cleanmesh-report.json"
        runner_audit = cleanmesh_runner(
            cleanmesh_executable,
            analysis_input,
            analysed_path,
            cleanmesh_report,
            resolution,
            pipeline,
        )
        analysed_batches = terrain.extract_cleanmesh_analysed_additions(
            analysed_path, local_manifest
        )
        if [batch.label for batch in analysed_batches] != [
            batch.label for batch in batches
        ]:
            raise RuntimeError("CleanMesh cross-scale addition labels/order changed")
        scalar_postprocess = []
        for initial_batch, analysed_batch in zip(
            batches, analysed_batches, strict=True
        ):
            if analysed_batch.role != initial_batch.role:
                raise RuntimeError("CleanMesh cross-scale addition roles changed")
            projected = terrain.project_analysed_additions(
                analysed_batch.records, clouds[initial_batch.role].layout.dtype
            )
            if not np.array_equal(_xyz(projected), _xyz(initial_batch.records)):
                raise RuntimeError("CleanMesh changed authoritative cross-scale XYZ")
            scalar_postprocess.append(
                {
                    "label": initial_batch.label,
                    "role": initial_batch.role,
                    "points": int(len(projected)),
                    **postprocess_local_analysed_additions(
                        projected,
                        initial_batch.records,
                        combined_normalizations,
                    ),
                }
            )
            analysed_by_role[initial_batch.role] = projected
        cleanmesh_audit: Mapping[str, object] = {
            "status": "completed",
            "local_input": analysis_input.name,
            "local_input_sha256": sha256_path(analysis_input),
            "local_manifest": local_manifest.manifest_path.name,
            "local_manifest_sha256": sha256_path(local_manifest.manifest_path),
            "analysed": analysed_path.name,
            "analysed_sha256": sha256_path(analysed_path),
            "report": cleanmesh_report.name,
            "report_sha256": sha256_path(cleanmesh_report),
            "runner": dict(runner_audit),
            "collar_points": int(
                sum(len(value.indices) for value in collar_selections)
            ),
            "addition_points": int(sum(len(batch.records) for batch in batches)),
            "full_cloud_analysis": False,
            "span_identity_verified": True,
            "exact_fine_xyz_preserved": True,
            "scalar_postprocess": scalar_postprocess,
        }
    else:
        cleanmesh_audit = {
            "status": "skipped-no-cross-scale-additions",
            "full_cloud_analysis": False,
            "exact_fine_xyz_preserved": True,
        }

    candidate_paths: dict[str, str] = {}
    candidate_hashes: dict[str, str] = {}
    addition_counts: dict[str, int] = {}
    append_audits: dict[str, object] = {}
    archive_paths: dict[str, str] = {}
    archive_hashes: dict[str, str] = {}
    for role in ("SAND", "ROCK"):
        cloud = clouds[role]
        records = analysed_by_role[role]
        selected_fine = selections_by_role[role].selected_fine_indices
        fine_records = fine_archives[role]["records"]
        if not np.array_equal(_xyz(records), _xyz(fine_records[selected_fine])):
            raise RuntimeError(f"5mm/{role}: final XYZ is not the selected fine subset")
        candidate = output_dir / _candidate_filename(role, resolution.label)
        append_report = terrain.write_append_only_candidate(
            cloud.layout.path,
            records,
            candidate,
            chunk_size=pipeline.chunk_records,
        )
        candidate_paths[role] = str(candidate)
        candidate_hashes[role] = sha256_path(candidate)
        addition_counts[role] = int(len(records))
        append_audit = dict(append_report)
        append_audit["candidate_path"] = candidate.name
        append_audits[role] = append_audit
        archive_arrays = _cross_scale_archive_arrays(
            records,
            fine_archives[role],
            selected_fine,
            donor_metadata_by_role[role],
        )
        archive_path = output_dir / f"cross-scale-additions-{role.lower()}.npz"
        _write_deterministic_npz(archive_path, archive_arrays)
        _verify_candidate_suffix(
            candidate,
            source_count=cloud.layout.vertex_count,
            expected_records=records,
        )
        archive_paths[role] = str(archive_path)
        archive_hashes[role] = sha256_path(archive_path)

        subset = selections_by_role[role]
        coverage_path = output_dir / f"cross-scale-coverage-{role.lower()}.npz"
        selected_mask = np.zeros(len(fine_archives[role]["records"]), bool)
        selected_mask[selected_fine] = True
        _write_deterministic_npz(
            coverage_path,
            {
                "fine_index": fine_archives[role]["fine_index"],
                "represented_by_existing": subset.represented_by_existing,
                "selected_for_coarse": selected_mask,
                "coverage_source": subset.coverage_source,
                "coverage_index": subset.coverage_index,
                "coverage_xy_distance_m": subset.coverage_xy_distance_m,
                "coverage_vertical_delta_m": subset.coverage_vertical_delta_m,
            },
        )
        pair_audit = _cross_scale_pair_audit(
            _xyz(records),
            spacing_m=resolution.nominal_spacing_m,
            vertical_tolerance_m=pipeline.cross_scale_vertical_tolerance_m,
            distance_tolerance_m=pipeline.cross_scale_distance_tolerance_m,
        )
        if not pair_audit["verified"]:
            raise RuntimeError(f"5mm/{role}: selected pair spacing audit failed")
        selected_existing_distance = subset.existing_xy_distance_m[selected_fine]
        finite_existing = selected_existing_distance[np.isfinite(selected_existing_distance)]
        selected_existing_minimum = (
            float(np.min(finite_existing)) if len(finite_existing) else None
        )
        if (
            selected_existing_minimum is not None
            and selected_existing_minimum
            < resolution.nominal_spacing_m
            - pipeline.cross_scale_distance_tolerance_m
        ):
            raise RuntimeError(f"5mm/{role}: terrain clearance audit failed")
        unique_targets, target_counts = np.unique(
            archive_arrays["fine_target_id"], return_counts=True
        )
        cross_role_audit[role] = {
            "fine_authority": {
                "candidate": os.path.relpath(
                    fine.candidate_paths[role], output_dir
                ),
                "candidate_sha256": fine.candidate_sha256[role],
                "archive": os.path.relpath(
                    fine.addition_archive_paths[role], output_dir
                ),
                "archive_sha256": fine.addition_archive_sha256[role],
                "points": int(len(fine_archives[role]["records"])),
            },
            "fine_points_already_represented_by_5mm_measured_terrain": int(
                np.count_nonzero(subset.represented_by_existing)
            ),
            "fine_points_selected_for_5mm": int(len(selected_fine)),
            "fine_points_with_vertical_same_role_donor": int(
                np.count_nonzero(donor_eligible_by_role[role])
            ),
            "fine_points_covered": int(len(subset.coverage_source)),
            "maximum_coverage_xy_distance_m": (
                float(np.max(subset.coverage_xy_distance_m))
                if len(subset.coverage_xy_distance_m)
                else None
            ),
            "maximum_coverage_vertical_delta_m": (
                float(np.max(subset.coverage_vertical_delta_m))
                if len(subset.coverage_vertical_delta_m)
                else None
            ),
            "minimum_selected_to_existing_xy_distance_m": (
                selected_existing_minimum
            ),
            "selected_to_existing_clearance_lower_bound_m": (
                resolution.nominal_spacing_m
            ),
            "pair_spacing": pair_audit,
            "target_counts": {
                str(target_id): int(count)
                for target_id, count in zip(unique_targets, target_counts, strict=True)
            },
            "coarse_archive": archive_path.name,
            "coarse_archive_sha256": archive_hashes[role],
            "coarse_archive_keys": sorted(CROSS_SCALE_ARCHIVE_KEYS),
            "coverage_ledger": coverage_path.name,
            "coverage_ledger_sha256": sha256_path(coverage_path),
            "exact_xyz_subset_verified": True,
            "coverage_verified": True,
            "terrain_clearance_verified": True,
            "vertical_support_guard_verified": True,
            "same_role_measured_donor_transfer_verified": True,
            "cleanmesh_geometry_identity_verified": True,
        }

    # Recheck the complete authority after every coarse artifact has been built.
    _verify_fine_artifact_closure(fine)
    cross_scale_report = {
        "schema_version": 1,
        "method": "deterministic-maximal-surface-aware-fine-xyz-subset-v1",
        "fine_resolution_report": os.path.relpath(fine.report_path, output_dir),
        "fine_resolution_report_sha256": fine.report_sha256,
        "coarse_spacing_m": resolution.nominal_spacing_m,
        "terrain_clearance_m": resolution.nominal_spacing_m,
        "vertical_support_tolerance_m": pipeline.cross_scale_vertical_tolerance_m,
        "distance_tolerance_m": pipeline.cross_scale_distance_tolerance_m,
        "selection_dimensions": "XY with explicit absolute-Z compatibility guard",
        "roles": cross_role_audit,
        "invariants": {
            "fine_final_additions_are_authoritative": True,
            "coarse_addition_xyz_is_exact_fine_subset": True,
            "coarse_independent_geometry_proposals": 0,
            "maximal_coverage_at_coarse_spacing": True,
            "floating_returns_do_not_count_as_surface_support": True,
            "same_role_scanid_0_to_8_property_donors_only": True,
            "cleanmesh_recomputed_coarse_scalars_without_xyz_changes": True,
            "candidate_only": True,
            "canonical_writes": False,
        },
    }
    cross_scale_path = output_dir / "cross-scale-report.json"
    _write_resolution_report(cross_scale_path, cross_scale_report)
    resolution_report = {
        "schema_version": 2,
        "candidate_only": True,
        "canonical_writes": False,
        "resolution": asdict(resolution),
        "construction": "exact-subset-of-final-1mm-authoritative-additions",
        "independent_target_proposals": 0,
        "fine_authority_report_sha256": fine.report_sha256,
        "cross_scale": {
            "report": cross_scale_path.name,
            "report_sha256": sha256_path(cross_scale_path),
        },
        "cleanmesh": cleanmesh_audit,
        "append_only": append_audits,
        "addition_counts": addition_counts,
        "invariants": cross_scale_report["invariants"],
    }
    report_path = output_dir / "resolution-report.json"
    _write_resolution_report(report_path, resolution_report)
    return ResolutionBuildResult(
        label=resolution.label,
        candidate_paths=candidate_paths,
        candidate_sha256=candidate_hashes,
        addition_counts=addition_counts,
        report_path=str(report_path),
        report_sha256=sha256_path(report_path),
        target_count=len(targets),
        addition_archive_paths=archive_paths,
        addition_archive_sha256=archive_hashes,
        cross_scale_report_path=str(cross_scale_path),
        cross_scale_report_sha256=sha256_path(cross_scale_path),
    )


def _result_with_final_paths(
    result: ResolutionBuildResult,
    stage: Path,
    destination: Path,
) -> ResolutionBuildResult:
    def translated(value: str) -> str:
        return str(destination / Path(value).relative_to(stage))

    return ResolutionBuildResult(
        label=result.label,
        candidate_paths={key: translated(value) for key, value in result.candidate_paths.items()},
        candidate_sha256=dict(result.candidate_sha256),
        addition_counts=dict(result.addition_counts),
        report_path=translated(result.report_path),
        report_sha256=result.report_sha256,
        target_count=result.target_count,
        addition_archive_paths={
            key: translated(value)
            for key, value in result.addition_archive_paths.items()
        },
        addition_archive_sha256=dict(result.addition_archive_sha256),
        cross_scale_report_path=(
            translated(result.cross_scale_report_path)
            if result.cross_scale_report_path is not None
            else None
        ),
        cross_scale_report_sha256=result.cross_scale_report_sha256,
    )


def build_terrain_candidates(
    *,
    sand_1mm_path: str | Path,
    rock_1mm_base_path: str | Path,
    sand_5mm_path: str | Path,
    rock_5mm_base_path: str | Path,
    config_path: str | Path,
    cleanmesh_executable: str | Path,
    output_dir: str | Path,
    normalization_manifest_path: str | Path | None = None,
    parameters: PipelineParameters = PipelineParameters(),
    cleanmesh_runner: CleanMeshRunner = run_cleanmesh_reduced_analysis,
    require_scipy: bool = True,
) -> TerrainPipelineResult:
    """Atomically publish a hash-locked, candidate-only four-cloud bundle."""

    destination = Path(output_dir).resolve(strict=False)
    if destination.exists():
        raise FileExistsError(f"refusing to overwrite terrain candidate run: {destination}")
    if require_scipy:
        require_production_runtime()
    destination.parent.mkdir(parents=True, exist_ok=True)
    paths = {
        "SAND-1mm": Path(sand_1mm_path).resolve(strict=True),
        "ROCK-1mm-base": Path(rock_1mm_base_path).resolve(strict=True),
        "SAND-5mm": Path(sand_5mm_path).resolve(strict=True),
        "ROCK-5mm-base": Path(rock_5mm_base_path).resolve(strict=True),
    }
    if len(set(paths.values())) != len(paths):
        raise ValueError("SAND/ROCK source paths must be distinct")
    config = Path(config_path).resolve(strict=True)
    executable = Path(cleanmesh_executable).resolve(strict=True)
    fingerprints = {name: fingerprint_ply(path) for name, path in paths.items()}
    config_hash = sha256_path(config)
    executable_hash = sha256_path(executable)
    implementation_paths = {
        Path(__file__).name: Path(__file__).resolve(),
        Path(terrain.__file__).name: Path(terrain.__file__).resolve(),
        Path(confidence.__file__).name: Path(confidence.__file__).resolve(),
    }
    implementation_hashes = {
        name: sha256_path(path) for name, path in implementation_paths.items()
    }
    normalization_manifest = (
        Path(normalization_manifest_path).resolve(strict=True)
        if normalization_manifest_path is not None
        else None
    )
    normalization_manifest_hash = (
        sha256_path(normalization_manifest)
        if normalization_manifest is not None
        else None
    )
    rock_1mm_dtype = terrain.inspect_fixed_stride_ply(paths["ROCK-1mm-base"]).dtype
    combined_fields = {
        f"scalar_A_R_{metric}_Combined" for metric in PHYSICAL_METRICS
    }
    has_combined_geometry = bool(
        combined_fields.intersection(rock_1mm_dtype.names or ())
    )
    if normalization_manifest is not None:
        combined_normalizations, normalization_audit = load_combined_normalizations(
            normalization_manifest
        )
    elif has_combined_geometry:
        combined_normalizations, normalization_audit = infer_combined_normalizations(
            paths["ROCK-1mm-base"],
            chunk_records=parameters.chunk_records,
            sample_limit=parameters.global_normalization_samples,
        )
    else:
        combined_normalizations = None
        normalization_audit = {
            "method": "not-applicable-no-standard-combined-geometry-fields",
            "values": None,
        }
    targets = terrain.terrain_targets_from_review_config(config)
    if not targets:
        raise ValueError("review config contains no terrain targets")
    stage = Path(
        tempfile.mkdtemp(prefix=f".{destination.name}.staging-", dir=destination.parent)
    )
    try:
        shutil.copy2(config, stage / "review-config.json")
        if normalization_manifest is not None:
            shutil.copy2(
                normalization_manifest, stage / "normalization-manifest.json"
            )
        fine = build_resolution_candidates(
            sand_path=paths["SAND-1mm"],
            rock_base_path=paths["ROCK-1mm-base"],
            targets=targets,
            output_dir=stage / "terrain-1mm",
            cleanmesh_executable=executable,
            resolution=parameters.fine,
            pipeline=parameters,
            combined_normalizations=combined_normalizations,
            cleanmesh_runner=cleanmesh_runner,
        )
        coarse = build_cross_scale_coarse_candidates(
            sand_path=paths["SAND-5mm"],
            rock_base_path=paths["ROCK-5mm-base"],
            fine=fine,
            targets=targets,
            output_dir=stage / "terrain-5mm",
            cleanmesh_executable=executable,
            resolution=parameters.coarse,
            pipeline=parameters,
            combined_normalizations=combined_normalizations,
            cleanmesh_runner=cleanmesh_runner,
        )
        final_fine = _result_with_final_paths(fine, stage, destination)
        final_coarse = _result_with_final_paths(coarse, stage, destination)
        if coarse.cross_scale_report_path is None:
            raise RuntimeError("5mm result has no cross-scale report")
        cross_scale_manifest = json.loads(
            Path(coarse.cross_scale_report_path).read_text(encoding="utf-8")
        )
        cross_scale_manifest["report"] = {
            "path": final_coarse.cross_scale_report_path,
            "sha256": final_coarse.cross_scale_report_sha256,
        }
        for fingerprint in fingerprints.values():
            assert_fingerprint_unchanged(fingerprint)
        if sha256_path(config) != config_hash:
            raise RuntimeError("review config changed during terrain build")
        if sha256_path(executable) != executable_hash:
            raise RuntimeError("CleanMesh executable changed during terrain build")
        for name, path in implementation_paths.items():
            if sha256_path(path) != implementation_hashes[name]:
                raise RuntimeError(
                    f"terrain implementation changed during build: {name}"
                )
        if (
            normalization_manifest is not None
            and sha256_path(normalization_manifest) != normalization_manifest_hash
        ):
            raise RuntimeError("normalization manifest changed during terrain build")
        normalization_manifest_audit = dict(normalization_audit)
        if normalization_manifest is not None:
            normalization_manifest_audit["archived_copy"] = (
                "normalization-manifest.json"
            )
        manifest = {
            "schema_version": 1,
            "operation": "site1-v11-candidate-only-terrain-interstitial-pipeline",
            "status": "built",
            "candidate_only": True,
            "canonical_install_performed": False,
            "config": {
                "path": str(config),
                "sha256": config_hash,
                "archived_copy": "review-config.json",
            },
            "cleanmesh": {
                "path": str(executable),
                "sha256": executable_hash,
                "scope": "local measured collars plus ScanID10 additions only",
            },
            "parameters": asdict(parameters),
            "implementation": implementation_hashes,
            "combined_geometry_normalization": normalization_manifest_audit,
            "sources": {
                name: asdict(fingerprint) for name, fingerprint in fingerprints.items()
            },
            "targets": [
                {
                    "id": target.target_id,
                    "kind": target.kind.value,
                    "bbox": list(target.bbox),
                    "minimum_tier": target.minimum_tier.name,
                }
                for target in targets
            ],
            "resolutions": {
                "1mm": asdict(final_fine),
                "5mm": asdict(final_coarse),
            },
            "cross_scale": cross_scale_manifest,
            "invariants": {
                "bbox_is_never_a_fill_mask": True,
                "connected_density_deficit_required": True,
                "all_three_surface_predictions_evaluated": True,
                "hard_geometry_vetoes_fail_closed": True,
                "overlapping_targets_globally_arbitrated_per_role": True,
                "same_role_scanid_0_to_8_donors_only": True,
                "all_additions_scanid": terrain.ADDITION_SCAN_ID,
                "existing_records_modified": 0,
                "caller_rock_base_preserved_byte_exact": True,
                "fine_additions_authoritative_for_coarse_geometry": True,
                "coarse_additions_exact_fine_xyz_subset": True,
                "coarse_maximal_surface_coverage_verified": True,
                "cross_scale_vertical_support_guard_verified": True,
                "canonical_writes": False,
            },
        }
        manifest_path = stage / "manifest.json"
        manifest_path.write_text(
            json.dumps(manifest, indent=2, sort_keys=True, allow_nan=False) + "\n",
            encoding="utf-8",
        )
        if destination.exists():
            raise FileExistsError(
                f"terrain destination appeared during build: {destination}"
            )
        os.replace(stage, destination)
    except BaseException:
        if stage.exists():
            shutil.rmtree(stage)
        raise
    return TerrainPipelineResult(
        output_dir=destination,
        manifest_path=destination / "manifest.json",
        resolutions={"1mm": final_fine, "5mm": final_coarse},
    )


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Build candidate-only Scene1 SAND/ROCK terrain interstitials"
    )
    parser.add_argument("--sand-1mm", required=True, type=Path)
    parser.add_argument("--rock-1mm-base", required=True, type=Path)
    parser.add_argument("--sand-5mm", required=True, type=Path)
    parser.add_argument("--rock-5mm-base", required=True, type=Path)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--cleanmesh", type=Path, default=DEFAULT_CLEANMESH)
    parser.add_argument(
        "--normalization-manifest",
        type=Path,
        help=(
            "Manifest containing rock_combined_normalizations; when omitted, "
            "infer them from the hash-locked 1 mm ROCK base"
        ),
    )
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--chunk-records", type=int, default=1_000_000)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    parameters = PipelineParameters(chunk_records=args.chunk_records)
    result = build_terrain_candidates(
        sand_1mm_path=args.sand_1mm,
        rock_1mm_base_path=args.rock_1mm_base,
        sand_5mm_path=args.sand_5mm,
        rock_5mm_base_path=args.rock_5mm_base,
        config_path=args.config,
        cleanmesh_executable=args.cleanmesh,
        output_dir=args.output,
        normalization_manifest_path=args.normalization_manifest,
        parameters=parameters,
    )
    print(
        json.dumps(
            {
                "output": str(result.output_dir),
                "additions": {
                    label: dict(value.addition_counts)
                    for label, value in result.resolutions.items()
                },
                "canonical_writes": False,
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "CleanMeshRunner",
    "CrossScaleSubsetSelection",
    "DeficitProposals",
    "GeometryProposalEvaluation",
    "LocalRoleCloud",
    "PipelineParameters",
    "ResolutionBuildResult",
    "ResolutionParameters",
    "RoleAssessment",
    "RoleChoice",
    "SourceFingerprint",
    "TargetBuildResult",
    "TerrainPipelineResult",
    "assess_target_role",
    "assert_fingerprint_unchanged",
    "arbitrate_resolution_additions",
    "build_resolution_candidates",
    "build_cross_scale_coarse_candidates",
    "build_target_additions",
    "build_terrain_candidates",
    "choose_supported_role",
    "collect_local_role_cloud",
    "compute_bounded_local_residual_energy",
    "estimate_local_reference_energy",
    "evaluate_proposals_batched",
    "fingerprint_ply",
    "generate_irregular_deficit_proposals",
    "infer_combined_normalizations",
    "load_combined_normalizations",
    "postprocess_local_analysed_additions",
    "require_production_runtime",
    "run_cleanmesh_reduced_analysis",
    "select_cross_scale_fine_subset",
    "select_bounded_geometry_donors",
    "sha256_path",
]
