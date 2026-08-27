#!/usr/bin/env python3
"""Candidate-only completion of the four reviewed southern WATER holes.

The registered marks select nearby empty connected components.  Geometry is
then derived from the current WATER/terrain support and the verified v10
surface reference.  Existing WATER records are copied byte-for-byte; only new
records inherit scalar attributes from a nearby v10 WATER donor.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import json
import math
import os
from pathlib import Path
from typing import Callable, Iterable, Mapping, Sequence

import numpy as np

import site1_v11_holes as holes
import site1_v11_water_density as density
import site1_v11_confidence as confidence


WATER_SCAN_ID = 999.0
POSITION_FIELDS = ("x", "y", "z")
NORMAL_FIELDS = ("nx", "ny", "nz")
DEFAULT_HOLE_MARK_IDS = (
    "image_2_mark_1",
    "image_2_mark_2",
    "image_2_mark_3",
    "image_2_mark_4",
)


@dataclass(frozen=True)
class LocalRecords:
    info: density.PlyInfo
    source_indices: np.ndarray
    records: np.ndarray


@dataclass(frozen=True)
class SourceFingerprint:
    path: str
    size_bytes: int
    mtime_ns: int
    sha256: str
    points: int
    record_stride: int


@dataclass(frozen=True)
class SurfaceAgreement:
    model_names: tuple[str, ...]
    sampled_count: int
    median_spread_m: float
    p95_spread_m: float
    maximum_spread_m: float
    accepted: bool


@dataclass(frozen=True)
class HolePipelineResult:
    candidate_path: str
    candidate_points: int
    candidate_sha256: str
    source_points: int
    addition_count: int
    accepted_hole_ids: tuple[str, ...]
    audit_path: str
    archive_path: str


def sha256_path(path: str | Path, block_size: int = 32 * 1024 * 1024) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        while block := handle.read(block_size):
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


def assert_fingerprint_unchanged(fingerprint: SourceFingerprint) -> None:
    path = Path(fingerprint.path)
    stat = path.stat()
    if (
        stat.st_size != fingerprint.size_bytes
        or stat.st_mtime_ns != fingerprint.mtime_ns
        or sha256_path(path) != fingerprint.sha256
    ):
        raise RuntimeError(f"source changed after hashing: {path}")


def collect_local_records(
    path: str | Path,
    bbox: Sequence[float],
    *,
    scan_id: float | None = None,
    chunk_size: int = 1_000_000,
) -> LocalRecords:
    info = density.inspect_fixed_stride_ply(path)
    required = {"x", "y", "z"}
    if scan_id is not None:
        required.add("scalar_ScanID")
    missing = sorted(required - set(info.dtype.names or ()))
    if missing:
        raise RuntimeError(f"source lacks required fields {missing}: {info.path}")
    index_parts: list[np.ndarray] = []
    record_parts: list[np.ndarray] = []
    for begin, records in density.iter_ply_chunks(
        info.path, info=info, chunk_size=chunk_size
    ):
        xy = np.column_stack((records["x"], records["y"])).astype(
            np.float64, copy=False
        )
        keep = density.bbox_mask(xy, bbox)
        if scan_id is not None:
            keep &= records["scalar_ScanID"] == scan_id
        selected = np.flatnonzero(keep)
        if len(selected):
            index_parts.append(selected.astype(np.int64) + begin)
            record_parts.append(np.asarray(records[selected]).copy())
    return LocalRecords(
        info=info,
        source_indices=(
            np.concatenate(index_parts) if index_parts else np.empty(0, np.int64)
        ),
        records=(
            np.concatenate(record_parts)
            if record_parts
            else np.empty(0, dtype=info.dtype)
        ),
    )


def load_hole_review(
    config_path: str | Path,
    *,
    mark_ids: Sequence[str] = DEFAULT_HOLE_MARK_IDS,
) -> tuple[tuple[float, float, float, float], dict[str, tuple[float, float]]]:
    with Path(config_path).open("r", encoding="utf-8") as handle:
        config = json.load(handle)
    bbox = tuple(
        float(value)
        for value in config["plan_annotations"]["cyan_southern_gap"]["review_bbox"]
    )
    by_id = {
        str(mark["id"]): mark
        for group in config["marked_locations"].values()
        for mark in group
    }
    seeds: dict[str, tuple[float, float]] = {}
    for mark_id in mark_ids:
        if mark_id not in by_id:
            raise KeyError(f"missing hole review mark {mark_id}")
        world = by_id[mark_id].get("world")
        if not isinstance(world, Sequence) or len(world) < 2:
            raise ValueError(f"hole review mark {mark_id} has no world coordinate")
        seeds[mark_id] = (float(world[0]), float(world[1]))
    return bbox, seeds


def _xyz(records: np.ndarray) -> np.ndarray:
    return np.column_stack((records["x"], records["y"], records["z"])).astype(
        np.float64, copy=False
    )


def _xy(records: np.ndarray) -> np.ndarray:
    return np.column_stack((records["x"], records["y"])).astype(
        np.float64, copy=False
    )


def _robust_polynomial_model(donor_xyz: np.ndarray, query_xy: np.ndarray) -> np.ndarray:
    centre = np.median(donor_xyz[:, :2], axis=0)
    scale = max(float(np.max(np.ptp(donor_xyz[:, :2], axis=0))), 0.10)
    dx = (donor_xyz[:, 0] - centre[0]) / scale
    dy = (donor_xyz[:, 1] - centre[1]) / scale
    design = np.column_stack((np.ones(len(dx)), dx, dy, dx * dx, dx * dy, dy * dy))
    weights = np.ones(len(dx), np.float64)
    coefficients = np.zeros(6, np.float64)
    # A ring of shoreline donors makes the constant and radial-quadratic
    # columns linearly dependent.  Penalising only the quadratic terms keeps
    # a genuinely planar pool planar while retaining gentle bowl/slope terms
    # when the observations support them.
    penalty = np.diag([0.0, 0.0, 0.0, 1.0e-3, 1.0e-3, 1.0e-3])
    for _ in range(8):
        normal = design.T @ (weights[:, None] * design) + penalty
        rhs = design.T @ (weights * donor_xyz[:, 2])
        coefficients = np.linalg.solve(normal, rhs)
        residual = donor_xyz[:, 2] - design @ coefficients
        sigma = max(1.4826 * float(np.median(np.abs(residual))), 0.001)
        ratio = np.abs(residual) / (2.5 * sigma)
        weights = np.ones_like(ratio)
        outside = ratio > 1.0
        weights[outside] = 1.0 / ratio[outside]
    qx = (query_xy[:, 0] - centre[0]) / scale
    qy = (query_xy[:, 1] - centre[1]) / scale
    query_design = np.column_stack(
        (np.ones(len(qx)), qx, qy, qx * qx, qx * qy, qy * qy)
    )
    return query_design @ coefficients


def _idw_model(donor_xyz: np.ndarray, query_xy: np.ndarray, neighbours: int = 16) -> np.ndarray:
    count = min(int(neighbours), len(donor_xyz))
    try:
        from scipy.spatial import cKDTree
    except ModuleNotFoundError:
        cKDTree = None
    if cKDTree is not None:
        distance, index = cKDTree(donor_xyz[:, :2]).query(
            query_xy, k=count, workers=-1
        )
        if count == 1:
            distance = np.asarray(distance)[:, None]
            index = np.asarray(index)[:, None]
    else:
        squared = np.sum(
            np.square(query_xy[:, None, :] - donor_xyz[None, :, :2]), axis=2
        )
        index = np.argsort(squared, axis=1, kind="stable")[:, :count]
        distance = np.sqrt(np.take_along_axis(squared, index, axis=1))
    exact = distance <= 1.0e-9
    weights = 1.0 / np.maximum(distance, 0.002) ** 2
    output = np.empty(len(query_xy), np.float64)
    for row, query in enumerate(query_xy):
        if np.any(exact[row]):
            output[row] = float(np.mean(donor_xyz[index[row, exact[row]], 2]))
            continue
        local = donor_xyz[index[row]]
        delta = local[:, :2] - query
        design = np.column_stack((np.ones(len(local)), delta))
        normal = design.T @ (weights[row, :, None] * design)
        # A tiny slope-only ridge makes a nearly collinear shoreline stable
        # without biasing the fitted elevation at the query location.
        normal += np.diag([0.0, 1.0e-9, 1.0e-9])
        rhs = design.T @ (weights[row] * local[:, 2])
        try:
            output[row] = float(np.linalg.solve(normal, rhs)[0])
        except np.linalg.LinAlgError:
            output[row] = float(
                np.sum(local[:, 2] * weights[row]) / np.sum(weights[row])
            )
    return output


def validate_surface_agreement(
    candidate_xy: np.ndarray,
    donor_xyz: np.ndarray,
    reference_height: Callable[[np.ndarray, np.ndarray], np.ndarray],
    *,
    sample_limit: int = 1024,
    maximum_p95_spread_m: float = 0.015,
    maximum_absolute_spread_m: float = 0.030,
) -> SurfaceAgreement:
    if not len(candidate_xy):
        return SurfaceAgreement(
            ("verified-v10", "robust-quadratic", "local-weighted-plane"), 0,
            math.nan, math.nan, math.nan, False,
        )
    if len(donor_xyz) < 24:
        return SurfaceAgreement(
            ("verified-v10", "robust-quadratic", "local-weighted-plane"), 0,
            math.nan, math.nan, math.nan, False,
        )
    if len(candidate_xy) > sample_limit:
        selected = np.linspace(0, len(candidate_xy) - 1, sample_limit).astype(np.int64)
        query = candidate_xy[selected]
    else:
        query = candidate_xy
    reference = np.asarray(reference_height(query[:, 0], query[:, 1]), np.float64)
    robust = _robust_polynomial_model(donor_xyz, query)
    local = _idw_model(donor_xyz, query)
    predictions = np.column_stack((reference, robust, local))
    spread = np.max(predictions, axis=1) - np.min(predictions, axis=1)
    finite = np.all(np.isfinite(predictions), axis=1)
    if not np.all(finite):
        return SurfaceAgreement(
            ("verified-v10", "robust-quadratic", "local-weighted-plane"), len(query),
            math.nan, math.nan, math.nan, False,
        )
    median = float(np.median(spread))
    p95 = float(np.quantile(spread, 0.95))
    maximum = float(np.max(spread))
    return SurfaceAgreement(
        ("verified-v10", "robust-quadratic", "local-weighted-plane"), len(query),
        median, p95, maximum,
        p95 <= maximum_p95_spread_m and maximum <= maximum_absolute_spread_m,
    )


def _hole_labels_at(plan: holes.HolePlan, xy: np.ndarray) -> np.ndarray:
    """Return diagnostic component labels for XY samples."""

    if not len(xy):
        return np.empty(0, np.int32)
    xmin, _, ymin, _ = plan.grid.bbox
    col = np.floor((xy[:, 0] - xmin) / plan.grid.cell_m).astype(np.int64)
    row = np.floor((xy[:, 1] - ymin) / plan.grid.cell_m).astype(np.int64)
    valid = (
        (row >= 0) & (row < plan.grid.labels.shape[0])
        & (col >= 0) & (col < plan.grid.labels.shape[1])
    )
    labels = np.zeros(len(xy), np.int32)
    labels[valid] = plan.grid.labels[row[valid], col[valid]]
    return labels


def validate_component_surfaces(
    plan: holes.HolePlan,
    candidate_xy: np.ndarray,
    donor_xyz: np.ndarray,
    reference_height: Callable[[np.ndarray, np.ndarray], np.ndarray],
    *,
    donor_margin_m: float = 0.30,
    maximum_reference_p95_error_m: float = 0.003,
    maximum_reference_error_m: float = 0.012,
) -> tuple[SurfaceAgreement, tuple[dict[str, object], ...]]:
    """Validate each disconnected pool against only its own shoreline.

    Different rock pools legitimately occupy different elevations.  A single
    polynomial over all four review marks can therefore reject correct local
    surfaces.  Independent models are retained as interior-extrapolation
    diagnostics per component.  The hard gate instead asks whether the exact
    archived v10 surface reproduces measured WATER along that component's
    shoreline; that is the same surface from which new geometry is sampled.
    """

    labels = _hole_labels_at(plan, candidate_xy)
    rows: list[dict[str, object]] = []
    results: list[SurfaceAgreement] = []
    by_label = {item.label: item for item in plan.holes}
    support_acceptance: list[bool] = []
    for label in plan.accepted_labels:
        query = candidate_xy[labels == label]
        item = by_label[int(label)]
        xmin, xmax, ymin, ymax = item.bounds
        local = (
            (donor_xyz[:, 0] >= xmin - donor_margin_m)
            & (donor_xyz[:, 0] <= xmax + donor_margin_m)
            & (donor_xyz[:, 1] >= ymin - donor_margin_m)
            & (donor_xyz[:, 1] <= ymax + donor_margin_m)
        )
        local_donors = donor_xyz[local]
        result = validate_surface_agreement(query, local_donors, reference_height)
        results.append(result)
        flat = np.asarray(item.cell_indices, np.int64)
        cell_row, cell_col = np.unravel_index(flat, plan.grid.labels.shape)
        component_xy = np.column_stack((
            plan.grid.x_centres[cell_col], plan.grid.y_centres[cell_row]
        ))
        if len(local_donors) and len(component_xy):
            try:
                from scipy.spatial import cKDTree
            except ModuleNotFoundError:
                squared = np.sum(
                    np.square(
                        local_donors[:, None, :2] - component_xy[None, :, :]
                    ),
                    axis=2,
                )
                component_distance = np.sqrt(np.min(squared, axis=1))
            else:
                component_distance = cKDTree(component_xy).query(
                    local_donors[:, :2], k=1, workers=-1
                )[0]
            support = component_distance <= max(0.075, 4.0 * plan.grid.cell_m)
        else:
            support = np.zeros(len(local_donors), bool)
        support_donors = local_donors[support]
        if len(support_donors):
            expected = np.asarray(
                reference_height(support_donors[:, 0], support_donors[:, 1]),
                np.float64,
            )
            residual = np.abs(support_donors[:, 2] - expected)
            finite = np.isfinite(residual)
            residual = residual[finite]
        else:
            residual = np.empty(0, np.float64)
        support_p95 = (
            float(np.quantile(residual, 0.95)) if len(residual) else math.nan
        )
        support_maximum = float(np.max(residual)) if len(residual) else math.nan
        support_ok = bool(
            len(residual) >= 24
            and support_p95 <= maximum_reference_p95_error_m
            and support_maximum <= maximum_reference_error_m
        )
        support_acceptance.append(support_ok)
        rows.append({
            "label": int(label),
            "bounds": list(item.bounds),
            "donor_count": int(np.count_nonzero(local)),
            "boundary_support_count": int(len(residual)),
            "reference_boundary_p95_error_m": support_p95,
            "reference_boundary_maximum_error_m": support_maximum,
            "reference_boundary_accepted": support_ok,
            # Polynomial and moving-plane predictions across the unmeasured
            # interior remain a useful uncertainty diagnostic, but cannot be
            # a hard truth test: extrapolation uncertainty grows with hole
            # size.  The hard geometric test is exact reproduction of the
            # measured shoreline by the archived v10 surface used for output.
            "interior_extrapolation_diagnostic": asdict(result),
        })
    if not results:
        aggregate = validate_surface_agreement(
            candidate_xy, donor_xyz, reference_height
        )
    else:
        aggregate = SurfaceAgreement(
            results[0].model_names,
            sum(item.sampled_count for item in results),
            max(item.median_spread_m for item in results),
            max(item.p95_spread_m for item in results),
            max(item.maximum_spread_m for item in results),
            all(support_acceptance),
        )
    return aggregate, tuple(rows)


def records_from_nearest_water(
    candidate_xy: np.ndarray,
    donor_records: np.ndarray,
    reference_surface: Callable[
        [np.ndarray, np.ndarray], tuple[np.ndarray, np.ndarray]
    ],
) -> tuple[np.ndarray, np.ndarray]:
    if not len(candidate_xy):
        return np.empty(0, donor_records.dtype), np.empty(0, np.int64)
    if not len(donor_records):
        raise ValueError("WATER additions require scalar donors")
    donor_xy = _xy(donor_records)
    try:
        from scipy.spatial import cKDTree
    except ModuleNotFoundError:
        cKDTree = None
    if cKDTree is not None:
        _, donor_index = cKDTree(donor_xy).query(candidate_xy, k=1, workers=-1)
    else:
        squared = np.sum(
            np.square(candidate_xy[:, None, :] - donor_xy[None, :, :]), axis=2
        )
        donor_index = np.argmin(squared, axis=1)
    donor_index = np.asarray(donor_index, np.int64)
    output = np.asarray(donor_records[donor_index]).copy()
    z, normal = reference_surface(candidate_xy[:, 0], candidate_xy[:, 1])
    z = np.asarray(z, np.float64)
    normal = np.asarray(normal, np.float64)
    if z.shape != (len(candidate_xy),) or normal.shape != (len(candidate_xy), 3):
        raise ValueError("reference_surface returned an invalid shape")
    if not np.all(np.isfinite(z)) or not np.all(np.isfinite(normal)):
        raise ValueError("reference_surface returned non-finite geometry")
    length = np.linalg.norm(normal, axis=1)
    if np.any(length <= 0.0):
        raise ValueError("reference_surface returned a zero normal")
    normal = normal / length[:, None]
    output["x"] = candidate_xy[:, 0].astype(output.dtype["x"])
    output["y"] = candidate_xy[:, 1].astype(output.dtype["y"])
    output["z"] = z.astype(output.dtype["z"])
    if {"nx", "ny", "nz"}.issubset(output.dtype.names or ()):
        output["nx"], output["ny"], output["nz"] = normal.T
    output["scalar_ScanID"] = np.asarray(
        WATER_SCAN_ID, dtype=output.dtype["scalar_ScanID"]
    )
    return output, donor_index


def copy_exact_fine_geometry(
    coarse_donor_records: np.ndarray,
    fine_records: np.ndarray,
    fine_selection_index: np.ndarray,
) -> tuple[np.ndarray, tuple[str, ...]]:
    """Copy selected fine geometry without changing coarse-donor attributes.

    Coarse WATER donors remain authoritative for every nongeometry field.  The
    selected fine archive is authoritative for XYZ and, when present, normals.
    Field dtypes must be identical so the cross-scale geometry can be proven
    byte-exact rather than merely numerically close after a dtype conversion.
    """

    output = np.asarray(coarse_donor_records).copy()
    fine = np.asarray(fine_records)
    selection = np.asarray(fine_selection_index)
    if output.ndim != 1 or fine.ndim != 1:
        raise ValueError("coarse and fine WATER records must be one-dimensional")
    if selection.ndim != 1 or selection.dtype.kind not in "iu":
        raise ValueError("fine_selection_index must be a one-dimensional integer array")
    if len(selection) != len(output):
        raise ValueError("fine_selection_index must align with coarse additions")
    if len(selection) and (
        int(np.min(selection)) < 0 or int(np.max(selection)) >= len(fine)
    ):
        raise IndexError("fine_selection_index lies outside fine records")

    coarse_names = set(output.dtype.names or ())
    fine_names = set(fine.dtype.names or ())
    if not set(POSITION_FIELDS).issubset(coarse_names & fine_names):
        raise RuntimeError("coarse and fine WATER records must both contain XYZ")
    coarse_has_normals = set(NORMAL_FIELDS).issubset(coarse_names)
    fine_has_normals = set(NORMAL_FIELDS).issubset(fine_names)
    if bool(coarse_names.intersection(NORMAL_FIELDS)) != coarse_has_normals:
        raise RuntimeError("coarse WATER schema contains an incomplete normal vector")
    if bool(fine_names.intersection(NORMAL_FIELDS)) != fine_has_normals:
        raise RuntimeError("fine WATER schema contains an incomplete normal vector")
    if coarse_has_normals != fine_has_normals:
        raise RuntimeError("coarse and fine WATER normal schemas differ")
    geometry_fields = POSITION_FIELDS + (NORMAL_FIELDS if coarse_has_normals else ())
    selected_fine = np.asarray(fine[selection])

    for name in geometry_fields:
        coarse_dtype = output.dtype.fields[name][0]
        fine_dtype = selected_fine.dtype.fields[name][0]
        if coarse_dtype != fine_dtype:
            raise RuntimeError(
                f"coarse/fine geometry dtype differs for {name}: "
                f"{coarse_dtype} != {fine_dtype}"
            )
    if len(selected_fine) and not np.all(np.isfinite(_xyz(selected_fine))):
        raise RuntimeError("selected fine WATER XYZ contains non-finite values")
    if coarse_has_normals and len(selected_fine):
        fine_normals = np.column_stack(
            tuple(selected_fine[name] for name in NORMAL_FIELDS)
        ).astype(np.float64, copy=False)
        if not np.all(np.isfinite(fine_normals)):
            raise RuntimeError("selected fine WATER normals contain non-finite values")

    nongeometry_fields = tuple(
        name for name in (output.dtype.names or ()) if name not in geometry_fields
    )
    donor_bytes = {
        name: np.ascontiguousarray(output[name]).tobytes()
        for name in nongeometry_fields
    }
    for name in geometry_fields:
        output[name] = selected_fine[name]
        if (
            np.ascontiguousarray(output[name]).tobytes()
            != np.ascontiguousarray(selected_fine[name]).tobytes()
        ):
            raise RuntimeError(f"coarse {name} is not byte-exact to selected fine rows")
    for name in nongeometry_fields:
        if np.ascontiguousarray(output[name]).tobytes() != donor_bytes[name]:
            raise RuntimeError(f"coarse donor field {name} changed during geometry copy")
    return output, geometry_fields


def _patched_header(header: bytes, count: int) -> bytes:
    import re

    pattern = re.compile(rb"(?m)^(element vertex )[0-9]+([ \t]*)(\r?)$")
    match = pattern.search(header)
    if match is None:
        raise RuntimeError("could not patch PLY vertex count")
    replacement = match.group(1) + str(int(count)).encode("ascii") + match.group(2) + match.group(3)
    return header[: match.start()] + replacement + header[match.end() :]


def append_candidate_records(
    source_path: str | Path,
    additions: np.ndarray,
    output_path: str | Path,
    *,
    overwrite: bool = False,
    chunk_size: int = 1_000_000,
) -> dict[str, object]:
    info = density.inspect_fixed_stride_ply(source_path)
    if additions.dtype != info.dtype or additions.ndim != 1:
        raise ValueError("additions must match the WATER source schema")
    output = density.assert_candidate_output_path(
        output_path, source_paths=[source_path]
    )
    if output.exists() and not overwrite:
        raise FileExistsError(output)
    temporary = output.with_name(f".{output.name}.{os.getpid()}.partial")
    temporary.unlink(missing_ok=True)
    source_before = info.path.stat()
    try:
        with info.path.open("rb") as source:
            header = source.read(info.offset)
        with temporary.open("wb") as handle:
            handle.write(_patched_header(header, info.count + len(additions)))
            for _, records in density.iter_ply_chunks(
                info.path, info=info, chunk_size=chunk_size
            ):
                np.asarray(records).tofile(handle)
            additions.tofile(handle)
        source_after = info.path.stat()
        if (
            source_before.st_size != source_after.st_size
            or source_before.st_mtime_ns != source_after.st_mtime_ns
        ):
            raise RuntimeError("source WATER changed during append")
        os.replace(temporary, output)
    except BaseException:
        temporary.unlink(missing_ok=True)
        raise
    return {
        "source_points": info.count,
        "addition_count": len(additions),
        "candidate_points": info.count + len(additions),
        "candidate_sha256": sha256_path(output),
        "existing_payload_byte_exact": True,
    }


def _json(path: Path, value: object, *, overwrite: bool) -> None:
    if path.exists() and not overwrite:
        raise FileExistsError(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.partial")
    temporary.write_text(
        json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, path)


def _declared_reference_paths(
    provenance: Mapping[str, object],
) -> tuple[Path, ...]:
    """Return declared provenance files for pre-write alias protection."""

    paths: list[Path] = []
    for key in ("surface_archive", "surface_config", "implementation"):
        value = provenance.get(key)
        if isinstance(value, Mapping) and "path" in value:
            paths.append(Path(str(value["path"])))
    return tuple(paths)


def _validated_artifact_paths(
    *,
    output_path: str | Path,
    audit_path: str | Path,
    archive_path: str | Path,
    protected_paths: Iterable[str | Path],
) -> tuple[Path, Path, Path]:
    """Reject artifact aliases, protected inputs, and all canonical clouds."""

    artifacts = (Path(output_path), Path(audit_path), Path(archive_path))
    resolved = tuple(
        path.expanduser().resolve(strict=False) for path in artifacts
    )
    if len(set(resolved)) != len(resolved):
        raise ValueError("candidate, audit, and archive paths must be distinct")
    protected = tuple(protected_paths)
    for artifact in artifacts:
        density.assert_candidate_output_path(
            artifact,
            source_paths=protected,
        )
    return artifacts


def _verify_reference_provenance(provenance: Mapping[str, object]) -> dict[str, object]:
    """Verify the versioned files that define the injected surface callable."""

    required = ("surface_archive", "surface_config", "implementation")
    output = dict(provenance)
    for key in required:
        value = provenance.get(key)
        if not isinstance(value, Mapping):
            raise ValueError(f"reference provenance lacks {key} fingerprint")
        try:
            path = Path(str(value["path"]))
            expected = str(value["sha256"])
        except KeyError as error:
            raise ValueError(f"reference provenance {key} is incomplete") from error
        if not path.is_file() or sha256_path(path) != expected:
            raise RuntimeError(f"reference provenance drift: {key}")
    if provenance.get("callable") != "rebuild_site1_fossils_v10.surface_values":
        raise ValueError("unexpected reference surface callable")
    noise_scale = float(provenance.get("noise_scale", math.nan))
    if not np.isfinite(noise_scale) or noise_scale <= 0.0:
        raise ValueError("reference provenance noise_scale must be positive")
    return output


def _resolve_recorded_path(value: str | Path, parent: Path) -> Path:
    path = Path(value)
    if not path.is_absolute():
        path = parent / path
    return path.resolve()


def _ordinary_file_fingerprint(path: str | Path) -> dict[str, object]:
    resolved = Path(path).resolve()
    stat = resolved.stat()
    return {
        "path": str(resolved),
        "size_bytes": stat.st_size,
        "mtime_ns": stat.st_mtime_ns,
        "sha256": sha256_path(resolved),
    }


def _fine_hole_geometry(
    fine_manifest_path: str | Path,
) -> tuple[
    dict[str, object], Path, np.ndarray, np.ndarray, np.ndarray, dict[str, object]
]:
    """Load a hash-closed fine hole archive and its exact stored geometry."""

    manifest_path = Path(fine_manifest_path).resolve()
    with manifest_path.open("r", encoding="utf-8") as handle:
        manifest = json.load(handle)
    if manifest.get("candidate_only") is not True:
        raise RuntimeError("fine hole manifest is not candidate-only")
    if manifest.get("canonical_install_performed") is not False:
        raise RuntimeError("fine hole manifest reports a canonical install")
    if manifest.get("existing_payload_byte_exact") is not True:
        raise RuntimeError("fine hole manifest lacks its prefix invariant")
    archive_path = _resolve_recorded_path(manifest["archive"], manifest_path.parent)
    if sha256_path(archive_path) != manifest["archive_sha256"]:
        raise RuntimeError("fine hole archive hash mismatch")
    candidate_path = _resolve_recorded_path(
        manifest["candidate"]["path"], manifest_path.parent
    )
    if sha256_path(candidate_path) != manifest["candidate"]["sha256"]:
        raise RuntimeError("fine hole candidate hash mismatch")
    with np.load(archive_path, allow_pickle=False) as archive:
        required = {"records", "candidate_xy", "candidate_label"}
        missing = sorted(required - set(archive.files))
        if missing:
            raise RuntimeError(f"fine hole archive lacks arrays {missing}")
        records = np.asarray(archive["records"]).copy()
        recorded_xy = np.asarray(archive["candidate_xy"], np.float64).copy()
        candidate_label = np.asarray(archive["candidate_label"], np.int32).copy()
    if records.ndim != 1 or recorded_xy.shape != (len(records), 2):
        raise RuntimeError("fine hole archive has inconsistent geometry shapes")
    if candidate_label.shape != (len(records),):
        raise RuntimeError("fine hole archive has inconsistent component labels")
    if int(manifest.get("addition_count", -1)) != len(records):
        raise RuntimeError("fine hole archive count differs from its manifest")
    if not {"x", "y", "z"}.issubset(records.dtype.names or ()):
        raise RuntimeError("fine hole archive lacks XYZ fields")
    record_xy = _xy(records)
    stored_as_records = np.column_stack(
        (
            recorded_xy[:, 0].astype(records.dtype["x"]),
            recorded_xy[:, 1].astype(records.dtype["y"]),
        )
    ).astype(np.float64)
    if not np.array_equal(record_xy, stored_as_records):
        raise RuntimeError("fine archive candidate_xy differs from stored record XY")
    accepted_labels = {
        int(item["label"])
        for item in manifest.get("holes", [])
        if item.get("accepted")
    }
    if not accepted_labels or not set(np.unique(candidate_label)).issubset(
        accepted_labels
    ):
        raise RuntimeError("fine archive contains an unaccepted component label")
    fingerprint = _ordinary_file_fingerprint(manifest_path)
    return (
        manifest, archive_path, records, record_xy, candidate_label, fingerprint
    )


def run_coarse_hole_completion_from_fine(
    *,
    source_water_path: str | Path,
    fine_manifest_path: str | Path,
    terrain_paths: Sequence[str | Path],
    config_path: str | Path,
    output_path: str | Path,
    audit_path: str | Path,
    archive_path: str | Path,
    spacing_m: float,
    reference_surface: Callable[
        [np.ndarray, np.ndarray], tuple[np.ndarray, np.ndarray]
    ],
    reference_provenance: Mapping[str, object],
    support_margin_m: float = 0.45,
    seed: int = 0x5331563131435353,
    overwrite: bool = False,
) -> HolePipelineResult:
    """Sample the coarse hole additions from the exact accepted fine geometry.

    The 5 mm stage is not allowed to rediscover a different occupancy topology.
    Its additions use a deterministic, unique subset of the fine archive.
    XYZ and normals are copied byte-exact from those fine rows after blue-noise
    selection against measured coarse WATER and vetoing by hash-locked coarse
    terrain support.  Nongeometry fields remain derived from coarse donors.
    """

    spacing = float(spacing_m)
    if not np.isfinite(spacing) or spacing <= 0.0:
        raise ValueError("spacing_m must be positive and finite")
    resolved_terrain_paths = tuple(Path(path).resolve() for path in terrain_paths)
    if len(set(resolved_terrain_paths)) != len(resolved_terrain_paths):
        raise ValueError("terrain_paths must not contain duplicate files")
    config_source = Path(config_path).resolve()
    initial_protected_paths = (
        Path(source_water_path),
        Path(fine_manifest_path),
        *resolved_terrain_paths,
        config_source,
        *_declared_reference_paths(reference_provenance),
    )
    output, audit, archive = _validated_artifact_paths(
        output_path=output_path,
        audit_path=audit_path,
        archive_path=archive_path,
        protected_paths=initial_protected_paths,
    )
    (
        fine,
        fine_archive_path,
        fine_records,
        fine_xy,
        fine_candidate_label,
        fine_manifest_fingerprint,
    ) = _fine_hole_geometry(fine_manifest_path)
    config_fingerprint = _ordinary_file_fingerprint(config_source)
    fine_config = fine.get("config", {})
    fine_config_path = _resolve_recorded_path(
        str(fine_config.get("path", "")), Path(fine_manifest_path).resolve().parent
    )
    if (
        fine_config_path != config_source
        or fine_config.get("sha256") != config_fingerprint["sha256"]
    ):
        raise RuntimeError("fine and coarse hole stages use different review configs")
    verified_reference = _verify_reference_provenance(reference_provenance)
    fine_reference = _verify_reference_provenance(fine["reference_provenance"])
    if fine_reference != verified_reference:
        raise RuntimeError("fine and coarse hole stages use different surfaces")

    fine_candidate_path = _resolve_recorded_path(
        fine["candidate"]["path"], Path(fine_manifest_path).resolve().parent
    )
    output, audit, archive = _validated_artifact_paths(
        output_path=output,
        audit_path=audit,
        archive_path=archive,
        protected_paths=(
            *initial_protected_paths,
            fine_archive_path,
            fine_candidate_path,
            *_declared_reference_paths(fine_reference),
        ),
    )

    source = fingerprint_ply(source_water_path)
    terrain_fingerprints = tuple(
        fingerprint_ply(path) for path in resolved_terrain_paths
    )
    review_bbox = tuple(float(value) for value in fine["review_bbox"])
    collar = density.expand_bbox(review_bbox, support_margin_m)
    water = collect_local_records(source.path, collar, scan_id=WATER_SCAN_ID)
    if not len(water.records):
        raise RuntimeError("no local coarse WATER support for reviewed holes")

    terrain_xy_parts: list[np.ndarray] = []
    for path in resolved_terrain_paths:
        part = collect_local_records(path, collar)
        if not len(part.records):
            continue
        xyz = _xyz(part.records)
        reference_z, _ = reference_surface(xyz[:, 0], xyz[:, 1])
        delta = xyz[:, 2] - np.asarray(reference_z, np.float64)
        near_surface = np.isfinite(delta) & (delta >= -0.015) & (delta <= 0.025)
        if np.any(near_surface):
            terrain_xy_parts.append(xyz[near_surface, :2].copy())
    terrain_xy = (
        np.concatenate(terrain_xy_parts)
        if terrain_xy_parts else np.empty((0, 2), np.float64)
    )
    if len(terrain_xy):
        terrain_distance = holes._nearest_distance(fine_xy, terrain_xy)
        terrain_clear = terrain_distance >= 0.50 * spacing
    else:
        terrain_distance = np.full(len(fine_xy), np.inf, np.float64)
        terrain_clear = np.ones(len(fine_xy), bool)
    eligible_index = np.flatnonzero(terrain_clear).astype(np.int64)
    selection = holes.variable_radius_blue_noise(
        fine_xy[eligible_index],
        spacing,
        existing_points=_xy(water.records),
        existing_radius=spacing,
        seed=seed,
    )
    fine_selection_index = eligible_index[selection.selected_indices]
    if len(np.unique(fine_selection_index)) != len(fine_selection_index):
        raise RuntimeError("coarse fine-selection mapping is not unique")
    selected_xy = fine_xy[fine_selection_index]
    additions, donor_index = records_from_nearest_water(
        selected_xy, water.records, reference_surface
    )
    additions, copied_geometry_fields = copy_exact_fine_geometry(
        additions, fine_records, fine_selection_index
    )
    selected_fine_records = np.asarray(fine_records[fine_selection_index])
    for name in copied_geometry_fields:
        if (
            np.ascontiguousarray(additions[name]).tobytes()
            != np.ascontiguousarray(selected_fine_records[name]).tobytes()
        ):
            raise RuntimeError(
                f"coarse stored {name} is not byte-exact to selected fine records"
            )

    combined_support = np.concatenate(
        (_xy(water.records), selected_xy, terrain_xy), axis=0
    )
    representation_distance = holes._nearest_distance(fine_xy, combined_support)
    maximum_representation_distance = float(np.max(representation_distance))
    if maximum_representation_distance > spacing + 1.0e-9:
        raise RuntimeError(
            "coarse WATER/terrain does not cover the accepted fine topology: "
            f"max distance {maximum_representation_distance:.6g} m"
        )

    coverage_rows: list[dict[str, object]] = []
    accepted_ids: list[str] = []
    for item in fine.get("holes", []):
        if not item.get("accepted"):
            continue
        accepted_ids.append(str(item["seed_id"]))
        xmin, xmax, ymin, ymax = (float(value) for value in item["bounds"])
        component_label = int(item["label"])
        fine_mask = fine_candidate_label == component_label
        coarse_mask = (
            fine_candidate_label[fine_selection_index] == component_label
        )
        if np.count_nonzero(fine_mask) and not np.count_nonzero(coarse_mask):
            raise RuntimeError(f"coarse subset misses accepted hole {item['seed_id']}")
        coverage_rows.append({
            "seed_id": str(item["seed_id"]),
            "component_label": component_label,
            "fine_count": int(np.count_nonzero(fine_mask)),
            "coarse_addition_count": int(np.count_nonzero(coarse_mask)),
            "bounds": [xmin, xmax, ymin, ymax],
        })

    write_report = append_candidate_records(
        source.path, additions, output, overwrite=overwrite
    )
    if archive.exists() and not overwrite:
        raise FileExistsError(archive)
    archive.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        archive,
        records=additions,
        donor_local_index=donor_index,
        candidate_xy=selected_xy,
        fine_selection_index=fine_selection_index,
        fine_component_label=fine_candidate_label[fine_selection_index],
    )

    assert_fingerprint_unchanged(source)
    for fingerprint in terrain_fingerprints:
        assert_fingerprint_unchanged(fingerprint)
    if _ordinary_file_fingerprint(config_source)["sha256"] != config_fingerprint["sha256"]:
        raise RuntimeError("review config changed during coarse hole completion")
    if _ordinary_file_fingerprint(fine_manifest_path) != fine_manifest_fingerprint:
        raise RuntimeError("fine manifest changed during coarse hole completion")
    if sha256_path(fine_archive_path) != fine["archive_sha256"]:
        raise RuntimeError("fine archive changed during coarse hole completion")
    _verify_reference_provenance(verified_reference)

    audit_value = {
        "schema_version": 1,
        "candidate_only": True,
        "canonical_install_performed": False,
        "annotations_are_seed_evidence_only": True,
        "config": config_fingerprint,
        "implementation": {
            Path(__file__).name: sha256_path(__file__),
            Path(holes.__file__).name: sha256_path(holes.__file__),
            Path(density.__file__).name: sha256_path(density.__file__),
            Path(confidence.__file__).name: sha256_path(confidence.__file__),
        },
        "reference_provenance": verified_reference,
        "source": asdict(source),
        "terrain_sources": [asdict(value) for value in terrain_fingerprints],
        "candidate": {
            "path": str(output.resolve()),
            "points": write_report["candidate_points"],
            "sha256": write_report["candidate_sha256"],
        },
        "review_bbox": list(review_bbox),
        "holes": fine.get("holes", []),
        "sampling": {
            "source_candidate_count": len(fine_xy),
            "rejected_for_terrain_clearance": int(np.count_nonzero(~terrain_clear)),
            "rejected_for_blue_noise": int(len(eligible_index) - len(selected_xy)),
            "xy": {
                "count": len(selected_xy),
                "minimum": np.min(selected_xy, axis=0).tolist() if len(selected_xy) else None,
                "maximum": np.max(selected_xy, axis=0).tolist() if len(selected_xy) else None,
            },
        },
        "surface_agreement": fine.get("surface_agreement"),
        "surface_agreement_components": fine.get("surface_agreement_components"),
        "addition_count": len(additions),
        "scalar_provenance": "nearest unchanged coarse v10 WATER donor for nongeometry fields; geometry-derived fields require enrichment",
        "geometry_provenance": "byte-exact XYZ and, when present, normal-vector subset of hash-verified fine v11 additions",
        "existing_payload_byte_exact": True,
        "archive": str(archive.resolve()),
        "archive_sha256": sha256_path(archive),
        "cross_scale": {
            "method": "deterministic-variable-radius-blue-noise-subset-v1",
            "fine_manifest": fine_manifest_fingerprint,
            "fine_archive": _ordinary_file_fingerprint(fine_archive_path),
            "fine_candidate_sha256": fine["candidate"]["sha256"],
            "fine_addition_count": len(fine_xy),
            "fine_selection_index_count": len(fine_selection_index),
            "fine_selection_index_unique": True,
            "coarse_xyz_exact_subset_of_fine_records_xyz": True,
            "coarse_normals_exact_subset_of_fine_records_normals": bool(
                set(NORMAL_FIELDS).issubset(copied_geometry_fields)
            ),
            "geometry_fields_copied_from_fine_records": list(
                copied_geometry_fields
            ),
            "nongeometry_fields_preserved_from_coarse_donors": True,
            "selection_seed": int(seed),
            "spacing_m": spacing,
            "maximum_fine_to_coarse_or_terrain_support_distance_m": maximum_representation_distance,
            "accepted_hole_coverage": coverage_rows,
        },
    }
    _json(audit, audit_value, overwrite=overwrite)
    return HolePipelineResult(
        candidate_path=str(output.resolve()),
        candidate_points=int(write_report["candidate_points"]),
        candidate_sha256=str(write_report["candidate_sha256"]),
        source_points=source.points,
        addition_count=len(additions),
        accepted_hole_ids=tuple(accepted_ids),
        audit_path=str(audit.resolve()),
        archive_path=str(archive.resolve()),
    )


def run_hole_completion(
    *,
    source_water_path: str | Path,
    terrain_paths: Sequence[str | Path],
    config_path: str | Path,
    output_path: str | Path,
    audit_path: str | Path,
    archive_path: str | Path,
    spacing_m: float,
    reference_surface: Callable[
        [np.ndarray, np.ndarray], tuple[np.ndarray, np.ndarray]
    ],
    reference_provenance: Mapping[str, object],
    mark_ids: Sequence[str] = DEFAULT_HOLE_MARK_IDS,
    diagnostic_cell_m: float | None = None,
    support_margin_m: float = 0.45,
    overwrite: bool = False,
) -> HolePipelineResult:
    """Complete only accepted seeded components and append full WATER rows."""

    resolved_terrain_paths = tuple(Path(path).resolve() for path in terrain_paths)
    if len(set(resolved_terrain_paths)) != len(resolved_terrain_paths):
        raise ValueError("terrain_paths must not contain duplicate files")
    config_source = Path(config_path).resolve()
    output, audit, archive = _validated_artifact_paths(
        output_path=output_path,
        audit_path=audit_path,
        archive_path=archive_path,
        protected_paths=(
            Path(source_water_path),
            *resolved_terrain_paths,
            config_source,
            *_declared_reference_paths(reference_provenance),
        ),
    )
    source_fingerprint = sha256_path(source_water_path)
    terrain_fingerprints = tuple(
        fingerprint_ply(path) for path in resolved_terrain_paths
    )
    config_fingerprint = {
        "path": str(config_source.resolve()),
        "sha256": sha256_path(config_source),
        "size_bytes": config_source.stat().st_size,
        "mtime_ns": config_source.stat().st_mtime_ns,
    }
    verified_reference = _verify_reference_provenance(reference_provenance)
    review_bbox, seeds = load_hole_review(config_path, mark_ids=mark_ids)
    collar = density.expand_bbox(review_bbox, support_margin_m)
    water = collect_local_records(
        source_water_path, collar, scan_id=WATER_SCAN_ID
    )
    if not len(water.records):
        raise RuntimeError("no local WATER support for reviewed holes")
    terrain_parts: list[np.ndarray] = []
    for path in resolved_terrain_paths:
        part = collect_local_records(path, collar)
        if not len(part.records):
            continue
        xyz = _xyz(part.records)
        reference_z, _ = reference_surface(xyz[:, 0], xyz[:, 1])
        delta = xyz[:, 2] - np.asarray(reference_z, np.float64)
        keep = np.isfinite(delta) & (delta >= -0.015) & (delta <= 0.025)
        terrain_parts.append(np.asarray(part.records[keep]).copy())
    terrain_records = (
        np.concatenate(terrain_parts)
        if terrain_parts
        else np.empty(0, dtype=water.records.dtype)
    )
    cell = (
        float(diagnostic_cell_m)
        if diagnostic_cell_m is not None
        else max(0.005, 2.5 * float(spacing_m))
    )
    policy = holes.SeededHolePolicy(
        diagnostic_cell_m=cell,
        water_support_radius_m=2.25 * float(spacing_m),
        terrain_support_radius_m=max(1.5 * float(spacing_m), 0.003),
        seed_search_radius_m=0.30,
        minimum_area_m2=0.0010,
        maximum_area_m2=0.75,
        sector_radius_m=0.45,
        minimum_support_sectors=6,
        minimum_water_sectors=3,
    )
    plan = holes.detect_seeded_holes(
        _xy(water.records),
        _xy(terrain_records) if len(terrain_records) else np.empty((0, 2)),
        review_bbox=review_bbox,
        seeds=seeds,
        policy=policy,
    )
    sampled = holes.sample_accepted_holes(
        plan,
        _xy(water.records),
        _xy(terrain_records) if len(terrain_records) else np.empty((0, 2)),
        spacing_m=float(spacing_m),
        terrain_clearance_m=0.50 * float(spacing_m),
    )
    agreement, component_agreements = validate_component_surfaces(
        plan, sampled.xy, _xyz(water.records),
        lambda x, y: reference_surface(x, y)[0],
    )
    if len(sampled.xy) and not agreement.accepted:
        raise RuntimeError(
            "reviewed WATER-hole surface models disagree: "
            f"p95={agreement.p95_spread_m:.6g} m, "
            f"max={agreement.maximum_spread_m:.6g} m"
        )
    additions, donor_index = records_from_nearest_water(
        sampled.xy, water.records, reference_surface
    )
    candidate_label = _hole_labels_at(plan, sampled.xy)
    if len(candidate_label) and not set(np.unique(candidate_label)).issubset(
        set(plan.accepted_labels)
    ):
        raise RuntimeError("sampled WATER additions contain an unaccepted component")
    write_report = append_candidate_records(
        source_water_path, additions, output, overwrite=overwrite
    )
    if archive.exists() and not overwrite:
        raise FileExistsError(archive)
    archive.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        archive,
        records=additions,
        donor_local_index=donor_index,
        candidate_xy=sampled.xy,
        candidate_label=candidate_label,
        accepted_labels=np.asarray(plan.accepted_labels, np.int32),
    )
    if sha256_path(source_water_path) != source_fingerprint:
        raise RuntimeError("source WATER changed during hole completion")
    for fingerprint in terrain_fingerprints:
        assert_fingerprint_unchanged(fingerprint)
    if (
        config_source.stat().st_size != config_fingerprint["size_bytes"]
        or config_source.stat().st_mtime_ns != config_fingerprint["mtime_ns"]
        or sha256_path(config_source) != config_fingerprint["sha256"]
    ):
        raise RuntimeError("review config changed during hole completion")
    _verify_reference_provenance(verified_reference)
    hole_rows = []
    for item in plan.holes:
        row = asdict(item)
        row["cell_indices"] = item.cell_indices.tolist()
        hole_rows.append(row)
    audit_value = {
        "schema_version": 1,
        "candidate_only": True,
        "canonical_install_performed": False,
        "annotations_are_seed_evidence_only": True,
        "config": config_fingerprint,
        "implementation": {
            Path(__file__).name: sha256_path(__file__),
            Path(holes.__file__).name: sha256_path(holes.__file__),
            Path(density.__file__).name: sha256_path(density.__file__),
            Path(confidence.__file__).name: sha256_path(confidence.__file__),
        },
        "reference_provenance": verified_reference,
        "source": {
            "path": str(Path(source_water_path).resolve()),
            "sha256": source_fingerprint,
            "points": water.info.count,
        },
        "terrain_sources": [
            asdict(fingerprint) for fingerprint in terrain_fingerprints
        ],
        "candidate": {
            "path": str(output.resolve()),
            "points": write_report["candidate_points"],
            "sha256": write_report["candidate_sha256"],
        },
        "review_bbox": list(review_bbox),
        "seeds": {key: list(value) for key, value in seeds.items()},
        "policy": asdict(policy),
        "holes": hole_rows,
        "sampling": asdict(sampled),
        "surface_agreement": asdict(agreement),
        "surface_agreement_components": component_agreements,
        "addition_count": len(additions),
        "component_membership": {
            "archive_key": "candidate_label",
            "all_additions_assigned_to_accepted_component": True,
            "accepted_labels": list(plan.accepted_labels),
        },
        "scalar_provenance": "nearest unchanged v10 WATER donor",
        "geometry_provenance": "hash-verified v10 height/noise/analytic-normal surface",
        "existing_payload_byte_exact": True,
        "archive": str(archive.resolve()),
        "archive_sha256": sha256_path(archive),
    }
    # JSON cannot serialise the XY vector in the dataclass directly.
    audit_value["sampling"]["xy"] = {
        "count": len(sampled.xy),
        "minimum": np.min(sampled.xy, axis=0).tolist() if len(sampled.xy) else None,
        "maximum": np.max(sampled.xy, axis=0).tolist() if len(sampled.xy) else None,
    }
    _json(audit, audit_value, overwrite=overwrite)
    accepted_ids = tuple(item.seed_id for item in plan.holes if item.accepted)
    return HolePipelineResult(
        candidate_path=str(output.resolve()),
        candidate_points=int(write_report["candidate_points"]),
        candidate_sha256=str(write_report["candidate_sha256"]),
        source_points=water.info.count,
        addition_count=len(additions),
        accepted_hole_ids=accepted_ids,
        audit_path=str(audit.resolve()),
        archive_path=str(archive.resolve()),
    )


__all__ = [
    "DEFAULT_HOLE_MARK_IDS",
    "HolePipelineResult",
    "LocalRecords",
    "SourceFingerprint",
    "SurfaceAgreement",
    "append_candidate_records",
    "assert_fingerprint_unchanged",
    "collect_local_records",
    "copy_exact_fine_geometry",
    "fingerprint_ply",
    "load_hole_review",
    "records_from_nearest_water",
    "run_coarse_hole_completion_from_fine",
    "run_hole_completion",
    "sha256_path",
    "validate_surface_agreement",
    "validate_component_surfaces",
]
