#!/usr/bin/env python3
"""Assemble, verify, install, or restore the six Scene1 v11 clouds safely.

The geometry builders deliberately stop at candidate files.  This release
orchestrator consumes their immutable JSON audits and makes the final name
changes only after independently checking every source/candidate hash, PLY
schema and point count, finite XYZ values, and each stage's byte-preservation
contract.

``build`` creates APFS-cloned (or copied) snapshots of all six canonical
sources and writes a hash-locked release manifest.  ``verify`` never changes
canonical data (it only refreshes a report inside the release bundle).
``install`` performs same-filesystem rename swaps under a durable intent
journal.  Any historic WATER ``-old01`` files are fingerprinted and left
untouched; the current six-cloud generation moves into the transaction's
recorded ``previous`` directory.  It does not create full replacement copies.
``restore`` reverses those renames while immutable APFS-cloned (or copied)
source snapshots remain as disaster-recovery evidence.  No action proceeds
while ``invisible_places`` is running.
"""

from __future__ import annotations

import argparse
from contextlib import contextmanager
from dataclasses import dataclass
import datetime as dt
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shutil
import stat
import subprocess
import tempfile
from typing import Mapping, Sequence

import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent
DEFAULT_DATA = ROOT / "Data" / "Scene1"
DEFAULT_RUN = (
    DEFAULT_DATA
    / "PatchRefinement"
    / "20260827-site1-v11-density-terrain-obstructions"
)
DEFAULT_RELEASE = DEFAULT_RUN / "release"
SCHEMA_VERSION = 1
OPERATION = "site1-v11-six-cloud-safe-release"
CHUNK_RECORDS = 250_000
CANONICAL_NAMES = (
    "Site1-SAND-1mm.ply",
    "Site1-ROCK-1mm.ply",
    "Site1-WATER-2mm.ply",
    "Site1-SAND-5mm.ply",
    "Site1-ROCK-5mm.ply",
    "Site1-WATER-5mm.ply",
)
EXPECTED_IMPLEMENTATIONS = {
    "obstruction": {
        "site1_v11_obstruction_pipeline.py",
        "site1_v11_obstructions.py",
    },
    "terrain": {
        "site1_v11_terrain_pipeline.py",
        "site1_v11_terrain.py",
        "site1_v11_confidence.py",
    },
    "water_base": {
        "site1_v11_water_pipeline.py",
        "site1_v11_water_density.py",
        "site1_v11_confidence.py",
    },
    "water_geometry": {
        "site1_v11_hole_pipeline.py",
        "site1_v11_holes.py",
        "site1_v11_water_density.py",
        "site1_v11_confidence.py",
    },
    "water_scalar": {
        "site1_v11_water_scalar_enrichment.py",
        "site1_v11_terrain.py",
        "site1_v11_terrain_pipeline.py",
        "site1_v11_hole_pipeline.py",
        "site1_v11_water_density.py",
        "site1_v11_confidence.py",
    },
}

OBSTRUCTION_PARAMETER_KEYS = {
    "surface",
    "thresholds",
    "preservation",
    "fine_voxel_m",
    "coarse_voxel_m",
    "boundary_guard_m",
    "core_inset_fraction",
    "cross_scale_distance_m",
    "preservation_cell_m",
    "chunk_records",
}
WATER_BASE_PARAMETER_KEYS = {
    "taper_bbox",
    "nominal_spacing_m",
    "interface_mark_ids",
    "cell_size_m",
    "smoothing_bandwidth_m",
    "taper_start_m",
    "taper_end_m",
    "floor_ratio",
    "relaxed_terrain_ratio",
    "duplicate_clearance_ratio",
    "maximum_bridge_m",
    "minimum_water_support",
    "seed",
    "chunk_size",
}
TERRAIN_PARAMETER_KEYS = {
    "fine",
    "coarse",
    "confidence",
    "role_dominance_ratio",
    "chunk_records",
    "cleanmesh_tile_width_m",
    "cleanmesh_chunk_points",
    "cleanmesh_normalization_samples",
    "global_normalization_samples",
    "cross_scale_vertical_tolerance_m",
    "cross_scale_distance_tolerance_m",
    "seed",
}
TERRAIN_RESOLUTION_PARAMETER_KEYS = {
    "label",
    "nominal_spacing_m",
    "deficit_cell_size_m",
    "neighbourhood_radius_cells",
    "minimum_expected_points",
    "minimum_deficit_fraction",
    "minimum_component_cells",
    "support_radius_m",
    "source_collar_m",
    "cleanmesh_collar_m",
    "property_donor_distance_m",
    "proposal_oversampling",
    "minimum_radius_ratio",
    "maximum_radius_ratio",
    "maximum_proposals_per_target",
    "geometry_batch_points",
    "reference_energy_samples",
    "maximum_geometry_donors",
    "maximum_energy_donors",
    "cleanmesh_base_voxel_m",
}
TERRAIN_CONFIDENCE_PARAMETER_KEYS = {
    "minimum_donor_sectors",
    "strong_donor_sectors",
    "maximum_surface_spread_m",
    "strong_surface_spread_m",
    "minimum_normal_coherence",
    "strong_normal_coherence",
    "maximum_vertical_thickness_m",
    "strong_vertical_thickness_m",
    "maximum_multimodality_score",
    "strong_multimodality_score",
    "maximum_residual_energy_ratio",
    "supported_preferred_gates",
}

TERRAIN_RESULT_KEYS = {
    "label",
    "candidate_paths",
    "candidate_sha256",
    "addition_counts",
    "report_path",
    "report_sha256",
    "target_count",
    "addition_archive_paths",
    "addition_archive_sha256",
    "cross_scale_report_path",
    "cross_scale_report_sha256",
}
TERRAIN_FINE_ARCHIVE_KEYS = {
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
TERRAIN_COARSE_ARCHIVE_KEYS = {
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
TERRAIN_COVERAGE_LEDGER_KEYS = {
    "fine_index",
    "represented_by_existing",
    "selected_for_coarse",
    "coverage_source",
    "coverage_index",
    "coverage_xy_distance_m",
    "coverage_vertical_delta_m",
}
TERRAIN_GLOBAL_LEDGER_KEYS = {
    "xyz",
    "radius_m",
    "local_priority",
    "global_priority",
    "target_candidate_index",
    "target_donor_index",
    "target_donor_distance_m",
    "target_donor_count",
    "confidence_reason_mask",
    "confidence_tier",
    "confidence_surface_spread_m",
    "confidence_preferred_gate_count",
    "target_id",
    "globally_selected",
    "disposition",
    "decision_reason_mask",
}
TERRAIN_TARGET_PROVENANCE_KEYS = {
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
    "float": "<f4",
    "float32": "<f4",
    "double": "<f8",
    "float64": "<f8",
}


@dataclass(frozen=True)
class PlyInfo:
    path: Path
    count: int
    offset: int
    dtype: np.dtype
    size_bytes: int

    @property
    def stride(self) -> int:
        return int(self.dtype.itemsize)

    @property
    def schema(self) -> tuple[tuple[str, str], ...]:
        return tuple(
            (name, self.dtype.fields[name][0].str)
            for name in self.dtype.names or ()
        )


def _now() -> str:
    return dt.datetime.now().isoformat(timespec="seconds")


def _entry_path(path: str | Path, label: str) -> Path:
    """Resolve only a path's parent and keep its final directory entry lexical."""

    value = Path(path).expanduser()
    if not value.is_absolute():
        value = Path.cwd() / value
    value = Path(os.path.abspath(value))
    try:
        lexical_parent_stat = os.lstat(value.parent)
    except OSError as error:
        raise RuntimeError(f"{label} parent is unavailable: {value.parent}") from error
    if not stat.S_ISDIR(lexical_parent_stat.st_mode):
        kind = (
            "symbolic link"
            if stat.S_ISLNK(lexical_parent_stat.st_mode)
            else "non-directory"
        )
        raise RuntimeError(f"{label} parent is a {kind}: {value.parent}")
    try:
        parent = value.parent.resolve(strict=True)
    except OSError as error:
        raise RuntimeError(f"{label} parent is unavailable: {value.parent}") from error
    try:
        parent_stat = os.lstat(parent)
    except OSError as error:
        raise RuntimeError(f"{label} parent is unavailable: {parent}") from error
    if not stat.S_ISDIR(parent_stat.st_mode):
        raise RuntimeError(f"{label} parent is not a directory: {parent}")
    return parent / value.name


def _lstat_regular(path: str | Path, label: str) -> tuple[Path, os.stat_result]:
    source = _entry_path(path, label)
    try:
        entry_stat = os.lstat(source)
    except FileNotFoundError:
        raise
    except OSError as error:
        raise RuntimeError(f"{label} cannot be inspected: {source}") from error
    if not stat.S_ISREG(entry_stat.st_mode):
        kind = "symbolic link" if stat.S_ISLNK(entry_stat.st_mode) else "special file"
        raise RuntimeError(f"{label} is a {kind}, not a regular file: {source}")
    return source, entry_stat


def _lstat_directory(path: str | Path, label: str) -> tuple[Path, os.stat_result]:
    source = _entry_path(path, label)
    try:
        entry_stat = os.lstat(source)
    except FileNotFoundError:
        raise
    except OSError as error:
        raise RuntimeError(f"{label} cannot be inspected: {source}") from error
    if not stat.S_ISDIR(entry_stat.st_mode):
        kind = "symbolic link" if stat.S_ISLNK(entry_stat.st_mode) else "non-directory"
        raise RuntimeError(f"{label} is a {kind}: {source}")
    return source, entry_stat


def _open_regular(path: str | Path, label: str) -> tuple[Path, os.stat_result, int]:
    source, before = _lstat_regular(path, label)
    flags = (
        os.O_RDONLY
        | getattr(os, "O_CLOEXEC", 0)
        | getattr(os, "O_NOFOLLOW", 0)
        | getattr(os, "O_NONBLOCK", 0)
    )
    try:
        descriptor = os.open(source, flags)
    except OSError as error:
        raise RuntimeError(f"{label} cannot be opened safely: {source}") from error
    after = os.fstat(descriptor)
    if (
        not stat.S_ISREG(after.st_mode)
        or (before.st_dev, before.st_ino) != (after.st_dev, after.st_ino)
    ):
        os.close(descriptor)
        raise RuntimeError(f"{label} changed while being opened: {source}")
    return source, after, descriptor


def sha256_path(path: str | Path, block_size: int = 32 * 1024 * 1024) -> str:
    digest = hashlib.sha256()
    source, opened, descriptor = _open_regular(path, "hash input")
    with os.fdopen(descriptor, "rb", closefd=True) as handle:
        while block := handle.read(block_size):
            digest.update(block)
    _, final = _lstat_regular(source, "hash input")
    if (opened.st_dev, opened.st_ino, opened.st_size, opened.st_mtime_ns) != (
        final.st_dev,
        final.st_ino,
        final.st_size,
        final.st_mtime_ns,
    ):
        raise RuntimeError(f"file changed while hashing: {source}")
    return digest.hexdigest()


def _stat_identity(path: Path) -> tuple[int, int, int, int]:
    source, entry_stat = _lstat_regular(path, "file identity")
    del source
    return (
        entry_stat.st_dev,
        entry_stat.st_ino,
        entry_stat.st_size,
        entry_stat.st_mtime_ns,
    )


def inspect_ply(path: str | Path) -> PlyInfo:
    """Inspect the fixed-stride binary little-endian PLYs used by Scene1."""

    source, opened, descriptor = _open_regular(path, "PLY input")
    header = bytearray()
    with os.fdopen(descriptor, "rb", closefd=True) as handle:
        for _ in range(200_000):
            line = handle.readline()
            if not line:
                raise ValueError(f"{source}: truncated PLY header")
            header.extend(line)
            if line.rstrip(b"\r\n") == b"end_header":
                break
        else:
            raise ValueError(f"{source}: unreasonably large PLY header")
    try:
        lines = bytes(header).decode("ascii").splitlines()
    except UnicodeDecodeError as error:
        raise ValueError(f"{source}: PLY header is not ASCII") from error
    if not lines or lines[0] != "ply":
        raise ValueError(f"{source}: not a PLY")
    if "format binary_little_endian 1.0" not in lines:
        raise ValueError(f"{source}: expected binary_little_endian 1.0")
    current: str | None = None
    count: int | None = None
    fields: list[tuple[str, str]] = []
    other_nonzero_elements: list[str] = []
    for line in lines:
        parts = line.split()
        if parts[:1] == ["element"]:
            if len(parts) != 3:
                raise ValueError(f"{source}: malformed element declaration")
            current = parts[1]
            element_count = int(parts[2])
            if current == "vertex":
                if count is not None:
                    raise ValueError(f"{source}: duplicate vertex element")
                count = element_count
            elif element_count:
                other_nonzero_elements.append(current)
        elif parts[:1] == ["property"] and current == "vertex":
            if len(parts) != 3 or parts[1] == "list":
                raise ValueError(f"{source}: variable-width vertex property")
            if parts[1] not in _PLY_TYPES:
                raise ValueError(f"{source}: unsupported PLY type {parts[1]}")
            fields.append((parts[2], _PLY_TYPES[parts[1]]))
    if count is None or count < 0 or not fields:
        raise ValueError(f"{source}: missing vertex schema/count")
    if other_nonzero_elements:
        raise ValueError(
            f"{source}: point-cloud release PLY has non-empty elements "
            + ", ".join(other_nonzero_elements)
        )
    dtype = np.dtype(fields)
    size = opened.st_size
    expected = len(header) + count * dtype.itemsize
    if size != expected:
        raise ValueError(
            f"{source}: byte size {size} != fixed payload size {expected}"
        )
    missing = sorted({"x", "y", "z"} - set(dtype.names or ()))
    if missing:
        raise ValueError(f"{source}: missing XYZ fields {missing}")
    return PlyInfo(source, count, len(header), dtype, size)


def file_fingerprint(path: str | Path, *, ply: bool | None = None) -> dict:
    source, entry_stat = _lstat_regular(path, "fingerprint input")
    before = _stat_identity(source)
    result: dict[str, object] = {
        "path": str(source),
        "bytes": entry_stat.st_size,
        "sha256": sha256_path(source),
        "device": entry_stat.st_dev,
        "inode": entry_stat.st_ino,
        "links": entry_stat.st_nlink,
        "mtime_ns": entry_stat.st_mtime_ns,
    }
    is_ply = source.suffix.lower() == ".ply" if ply is None else ply
    if is_ply:
        info = inspect_ply(source)
        result.update(
            {
                "points": info.count,
                "record_stride": info.stride,
                "schema": [list(item) for item in info.schema],
            }
        )
    if before != _stat_identity(source):
        raise RuntimeError(f"file changed while fingerprinting: {source}")
    return result


def _assert_fingerprint(
    path: str | Path,
    expected: Mapping,
    label: str,
    *,
    compare_identity: bool = True,
) -> dict:
    actual = file_fingerprint(path, ply="points" in expected)
    keys = ["bytes", "sha256", "points", "record_stride", "schema"]
    if compare_identity:
        keys.extend(("device", "inode", "links"))
    mismatched = [
        key for key in keys
        if key in expected and actual.get(key) != expected.get(key)
    ]
    if mismatched:
        raise RuntimeError(f"{label} fingerprint drift: {', '.join(mismatched)}")
    return actual


def _fsync_directory(path: Path) -> None:
    directory = Path(path).resolve(strict=True)
    descriptor = os.open(
        directory,
        os.O_RDONLY | getattr(os, "O_DIRECTORY", 0) | getattr(os, "O_CLOEXEC", 0),
    )
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _durable_replace(source: Path, destination: Path) -> None:
    """Rename and durably record both affected directory entries."""

    source_lexical = Path(os.path.abspath(Path(source).expanduser()))
    destination_lexical = Path(os.path.abspath(Path(destination).expanduser()))
    source_validated = _entry_path(source_lexical, "rename source")
    destination_validated = _entry_path(destination_lexical, "rename destination")
    source_parent = source_validated.parent
    destination_parent = destination_validated.parent
    os.replace(source_lexical, destination_lexical)
    _fsync_directory(destination_parent)
    if source_parent != destination_parent:
        _fsync_directory(source_parent)


def _atomic_json(path: Path, value: Mapping) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    destination = _entry_path(path, "JSON destination")
    destination_before: tuple[int, int] | None = None
    if os.path.lexists(destination):
        _, existing = _lstat_regular(destination, "JSON destination")
        destination_before = (existing.st_dev, existing.st_ino)
    payload = json.dumps(
        value, indent=2, sort_keys=True, allow_nan=False
    ) + "\n"
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{destination.name}.",
        suffix=".partial",
        dir=destination.parent,
        text=True,
    )
    temporary = Path(temporary_name)
    try:
        os.fchmod(descriptor, 0o600)
        handle = os.fdopen(descriptor, "w", encoding="utf-8", closefd=True)
        handle.write(payload)
        handle.flush()
        os.fsync(handle.fileno())
        handle.close()
        if destination_before is None:
            if os.path.lexists(destination):
                raise RuntimeError(
                    f"JSON destination appeared during write: {destination}"
                )
        else:
            _, current = _lstat_regular(destination, "JSON destination")
            if (current.st_dev, current.st_ino) != destination_before:
                raise RuntimeError(
                    f"JSON destination changed during write: {destination}"
                )
        _lstat_regular(temporary, "temporary JSON")
        _durable_replace(temporary, destination)
    except BaseException:
        try:
            os.close(descriptor)
        except OSError:
            pass
        if os.path.lexists(temporary):
            temporary.unlink()
        raise


def _load_json(path: str | Path, label: str) -> tuple[Path, dict]:
    source, opened, descriptor = _open_regular(path, label)
    try:
        with os.fdopen(descriptor, "r", encoding="utf-8", closefd=True) as handle:
            value = json.load(handle)
    except json.JSONDecodeError as error:
        raise RuntimeError(f"{label} is invalid JSON: {source}") from error
    _, final = _lstat_regular(source, label)
    if (opened.st_dev, opened.st_ino, opened.st_size, opened.st_mtime_ns) != (
        final.st_dev,
        final.st_ino,
        final.st_size,
        final.st_mtime_ns,
    ):
        raise RuntimeError(f"{label} changed while being read: {source}")
    if not isinstance(value, dict):
        raise RuntimeError(f"{label} must be a JSON object: {source}")
    return source, value


def _resolve(value: str | Path, parent: Path) -> Path:
    path = Path(value)
    if not path.is_absolute():
        path = parent / path
    source = _entry_path(path, "referenced artifact")
    entry_stat = os.lstat(source)
    if stat.S_ISREG(entry_stat.st_mode) or stat.S_ISDIR(entry_stat.st_mode):
        return source
    kind = "symbolic link" if stat.S_ISLNK(entry_stat.st_mode) else "special file"
    raise RuntimeError(f"referenced artifact is a {kind}: {source}")


def _require(value: bool, message: str) -> None:
    if not value:
        raise RuntimeError(message)


def _require_exact_keys(value: object, expected: set[str], label: str) -> Mapping:
    _require(isinstance(value, Mapping), f"{label} is missing")
    observed = {str(key) for key in value}
    missing = sorted(expected - observed)
    unexpected = sorted(observed - expected)
    detail = []
    if missing:
        detail.append("missing " + ", ".join(missing))
    if unexpected:
        detail.append("unexpected " + ", ".join(unexpected))
    _require(not detail, f"{label} keys differ: {'; '.join(detail)}")
    return value


def _finite_number(value: object, label: str) -> float:
    _require(not isinstance(value, bool), f"{label} is not numeric")
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise RuntimeError(f"{label} is not numeric") from error
    _require(np.isfinite(result), f"{label} is not finite")
    return result


def _verify_exact_implementation_hashes(
    value: Mapping,
    *,
    family: str,
    label: str,
    artifacts: dict[str, dict],
) -> dict[str, str]:
    expected = EXPECTED_IMPLEMENTATIONS[family]
    _require_exact_keys(value, expected, f"{label} implementation")
    return _verify_implementation_hashes(value, label=label, artifacts=artifacts)


def _verify_obstruction_parameters(value: object) -> Mapping:
    parameters = _require_exact_keys(
        value, OBSTRUCTION_PARAMETER_KEYS, "obstruction parameters"
    )
    nested_keys = {
        "surface": {
            "anchor_cell_m", "anchor_quantile", "minimum_points_per_anchor",
            "maximum_anchors", "neighbour_count", "query_chunk_points",
            "huber_iterations", "huber_scale_m",
        },
        "thresholds": {
            "minimum_models", "maximum_model_spread_m", "seed_height_m",
            "grow_height_m", "review_height_m", "ground_stop_height_m",
            "fine_seed_distance_m",
        },
        "preservation": {
            "ground_band_m", "maximum_collar_removed_fraction",
            "maximum_cell_ground_loss_fraction",
        },
    }
    for section, keys in nested_keys.items():
        nested = _require_exact_keys(
            parameters[section], keys, f"obstruction parameters.{section}"
        )
        for name, item in nested.items():
            _finite_number(item, f"obstruction parameters.{section}.{name}")
    for name in OBSTRUCTION_PARAMETER_KEYS - set(nested_keys):
        _finite_number(parameters[name], f"obstruction parameters.{name}")
    return parameters


def _verify_water_base_parameters(value: object, density_reference: object) -> Mapping:
    parameters = _require_exact_keys(
        value, WATER_BASE_PARAMETER_KEYS, "WATER base parameters"
    )
    bbox = parameters["taper_bbox"]
    _require(
        isinstance(bbox, list) and len(bbox) == 4,
        "WATER base taper_bbox must contain four values",
    )
    for index, item in enumerate(bbox):
        _finite_number(item, f"WATER base taper_bbox[{index}]")
    marks = parameters["interface_mark_ids"]
    _require(
        isinstance(marks, list)
        and bool(marks)
        and all(isinstance(item, str) and item for item in marks),
        "WATER base interface marks are invalid",
    )
    for name in WATER_BASE_PARAMETER_KEYS - {"taper_bbox", "interface_mark_ids"}:
        _finite_number(parameters[name], f"WATER base parameters.{name}")
    _require(isinstance(density_reference, Mapping), "WATER density reference is missing")
    for name in (
        "cell_size_m", "smoothing_bandwidth_m", "taper_start_m",
        "taper_end_m", "floor_ratio",
    ):
        _require(name in density_reference, f"WATER density reference lacks {name}")
        _require(
            _finite_number(density_reference[name], f"WATER density reference.{name}")
            == _finite_number(parameters[name], f"WATER base parameters.{name}"),
            f"WATER density reference disagrees with parameters: {name}",
        )
    return parameters


def _verify_terrain_parameters(value: object) -> Mapping:
    parameters = _require_exact_keys(
        value, TERRAIN_PARAMETER_KEYS, "terrain parameters"
    )
    for name, expected_label in (("fine", "1mm"), ("coarse", "5mm")):
        resolution = _require_exact_keys(
            parameters[name],
            TERRAIN_RESOLUTION_PARAMETER_KEYS,
            f"terrain parameters.{name}",
        )
        _require(
            resolution["label"] == expected_label,
            f"terrain {name} resolution label is wrong",
        )
        for key in TERRAIN_RESOLUTION_PARAMETER_KEYS - {"label"}:
            _finite_number(resolution[key], f"terrain parameters.{name}.{key}")
    confidence = _require_exact_keys(
        parameters["confidence"],
        TERRAIN_CONFIDENCE_PARAMETER_KEYS,
        "terrain parameters.confidence",
    )
    for name, item in confidence.items():
        _finite_number(item, f"terrain parameters.confidence.{name}")
    for name in TERRAIN_PARAMETER_KEYS - {"fine", "coarse", "confidence"}:
        _finite_number(parameters[name], f"terrain parameters.{name}")
    return parameters


def _verify_normalization_values(value: object, label: str) -> dict:
    metrics = {"MeanCurvature", "CrossCurvature", "Recession", "Roughness"}
    scales = {"Fine", "Medium", "Broad"}
    values = _require_exact_keys(value, metrics, f"{label} values")
    output = {}
    for metric in sorted(metrics):
        row = _require_exact_keys(values[metric], scales, f"{label} {metric}")
        output[metric] = {}
        for scale in sorted(scales):
            number = _finite_number(row[scale], f"{label} {metric}/{scale}")
            _require(number > 0.0, f"{label} {metric}/{scale} is not positive")
            output[metric][scale] = number
    return output


def _same_path(left: str | Path, right: str | Path) -> bool:
    return _entry_path(left, "left comparison path") == _entry_path(
        right, "right comparison path"
    )


def _finite_xyz(path: str | Path, *, chunk_records: int = CHUNK_RECORDS) -> int:
    info = inspect_ply(path)
    records = np.memmap(
        info.path,
        dtype=info.dtype,
        mode="r",
        offset=info.offset,
        shape=(info.count,),
    )
    invalid = 0
    try:
        for begin in range(0, info.count, chunk_records):
            part = records[begin : begin + chunk_records]
            invalid += int(np.count_nonzero(~(
                np.isfinite(part["x"])
                & np.isfinite(part["y"])
                & np.isfinite(part["z"])
            )))
    finally:
        del records
    return invalid


def _records_bytes(records: np.ndarray) -> memoryview:
    return memoryview(np.asarray(records).view(np.uint8).reshape(-1))


def verify_exact_append_prefix(
    source_path: str | Path,
    candidate_path: str | Path,
    source_count: int | None = None,
    *,
    chunk_records: int = CHUNK_RECORDS,
) -> dict:
    """Verify that a candidate starts with the exact source payload bytes."""

    source = inspect_ply(source_path)
    candidate = inspect_ply(candidate_path)
    count = source.count if source_count is None else int(source_count)
    _require(count == source.count, "append audit source count is inconsistent")
    _require(candidate.count >= count, "append candidate is shorter than its source")
    _require(source.schema == candidate.schema, "append source/candidate schemas differ")
    left = np.memmap(
        source.path, dtype=source.dtype, mode="r", offset=source.offset,
        shape=(source.count,),
    )
    right = np.memmap(
        candidate.path, dtype=candidate.dtype, mode="r", offset=candidate.offset,
        shape=(candidate.count,),
    )
    try:
        for begin in range(0, count, chunk_records):
            end = min(count, begin + chunk_records)
            if _records_bytes(left[begin:end]) != _records_bytes(right[begin:end]):
                raise RuntimeError(
                    f"append candidate prefix differs at records {begin}:{end}"
                )
    finally:
        del left, right
    return {"verified": True, "source_points": count, "addition_points": candidate.count - count}


def _payload_sha256(
    path: str | Path,
    *,
    start: int = 0,
    count: int | None = None,
) -> str:
    info = inspect_ply(path)
    length = info.count - start if count is None else int(count)
    _require(0 <= start <= info.count, f"payload hash start escapes {path}")
    _require(0 <= length <= info.count - start, f"payload hash length escapes {path}")
    digest = hashlib.sha256()
    remaining = length * info.stride
    with info.path.open("rb") as handle:
        handle.seek(info.offset + start * info.stride)
        while remaining:
            block = handle.read(min(32 * 1024 * 1024, remaining))
            if not block:
                raise RuntimeError(f"unexpected EOF while hashing PLY payload: {path}")
            digest.update(block)
            remaining -= len(block)
    return digest.hexdigest()


def _verify_npz_suffix_archive(
    candidate_path: str | Path,
    *,
    base_count: int,
    archive_path: str | Path,
) -> dict:
    info = inspect_ply(candidate_path)
    addition_count = info.count - int(base_count)
    _require(addition_count >= 0, "archive candidate is shorter than its base")
    with np.load(archive_path, allow_pickle=False) as loaded:
        _require("records" in loaded.files, "WATER hole archive lacks records")
        records = np.asarray(loaded["records"])
    _require(records.dtype == info.dtype, "WATER hole archive schema mismatch")
    _require(len(records) == addition_count, "WATER hole archive count mismatch")
    expected = hashlib.sha256(np.ascontiguousarray(records).tobytes()).hexdigest()
    observed = _payload_sha256(
        candidate_path, start=base_count, count=addition_count
    )
    _require(expected == observed, "WATER geometry suffix differs from its archive")
    return {
        "verified": True,
        "points": addition_count,
        "records_sha256": expected,
        "candidate_suffix_sha256": observed,
    }


def _accepted_holes(
    document: Mapping, label: str
) -> dict[str, tuple[int, tuple[float, float, float, float]]]:
    holes = document.get("holes")
    _require(isinstance(holes, list), f"{label} hole list is missing")
    accepted: dict[str, tuple[int, tuple[float, float, float, float]]] = {}
    accepted_labels: set[int] = set()
    for row in holes:
        _require(isinstance(row, Mapping), f"{label} hole row is invalid")
        if not row.get("accepted"):
            continue
        seed_id = str(row.get("seed_id", ""))
        component_value = row.get("label")
        _require(
            not isinstance(component_value, bool)
            and isinstance(component_value, (int, np.integer)),
            f"{label} accepted hole component label is invalid",
        )
        component_label = int(component_value)
        bounds_value = row.get("bounds")
        _require(
            isinstance(bounds_value, (list, tuple)) and len(bounds_value) == 4,
            f"{label} accepted hole is incomplete",
        )
        bounds = tuple(float(value) for value in bounds_value)
        _require(
            all(np.isfinite(bounds))
            and bounds[0] <= bounds[1]
            and bounds[2] <= bounds[3],
            f"{label} accepted hole bounds are invalid",
        )
        _require(bool(seed_id), f"{label} accepted hole ID is empty")
        _require(seed_id not in accepted, f"{label} accepted hole ID is duplicated")
        _require(
            component_label not in accepted_labels,
            f"{label} accepted hole component label is duplicated",
        )
        accepted_labels.add(component_label)
        accepted[seed_id] = (component_label, bounds)
    _require(bool(accepted), f"{label} contains no accepted holes")
    return accepted


def _record_xy_matches_archive(records: np.ndarray, candidate_xy: np.ndarray, label: str) -> None:
    _require(records.ndim == 1, f"{label} records are not a vector")
    _require(
        {"x", "y"}.issubset(records.dtype.names or ()),
        f"{label} records lack XY fields",
    )
    recorded = np.asarray(candidate_xy, np.float64)
    _require(
        recorded.shape == (len(records), 2) and np.all(np.isfinite(recorded)),
        f"{label} candidate_xy is invalid",
    )
    stored_as_records = np.column_stack(
        (
            recorded[:, 0].astype(records.dtype["x"]),
            recorded[:, 1].astype(records.dtype["y"]),
        )
    ).astype(np.float64)
    record_xy = np.column_stack((records["x"], records["y"])).astype(np.float64)
    _require(
        np.array_equal(record_xy, stored_as_records),
        f"{label} candidate_xy differs from record XY",
    )


def _geometry_component_labels(
    document: Mapping,
    *,
    labels: np.ndarray,
    accepted_holes: Mapping[str, tuple[int, tuple[float, float, float, float]]],
    label: str,
) -> np.ndarray:
    membership = document.get("component_membership")
    _require(isinstance(membership, Mapping), f"{label} component membership is missing")
    _require(
        membership.get("archive_key") == "candidate_label",
        f"{label} component membership archive key is unexpected",
    )
    _require(
        membership.get("all_additions_assigned_to_accepted_component") is True,
        f"{label} additions are not fully assigned to accepted components",
    )
    accepted_value = membership.get("accepted_labels")
    _require(isinstance(accepted_value, list), f"{label} accepted labels are missing")
    _require(
        all(
            not isinstance(value, bool) and isinstance(value, (int, np.integer))
            for value in accepted_value
        ),
        f"{label} accepted labels are invalid",
    )
    accepted_labels = [int(value) for value in accepted_value]
    _require(
        len(accepted_labels) == len(set(accepted_labels)) and bool(accepted_labels),
        f"{label} accepted labels are empty or duplicated",
    )
    hole_labels = {value[0] for value in accepted_holes.values()}
    _require(
        set(accepted_labels) == hole_labels,
        f"{label} component membership differs from accepted holes",
    )
    values = np.asarray(labels)
    _require(
        values.ndim == 1 and np.issubdtype(values.dtype, np.integer),
        f"{label} component-label array is invalid",
    )
    values = values.astype(np.int64, copy=False)
    _require(
        set(np.unique(values)) == hole_labels,
        f"{label} component-label array does not exactly cover accepted holes",
    )
    return values


def _array_sha256(values: np.ndarray) -> str:
    return hashlib.sha256(np.ascontiguousarray(values).tobytes()).hexdigest()


def _verify_component_membership_audit(
    value: object,
    *,
    archive_path: str | Path,
    required_arrays: set[str],
    labels: np.ndarray,
    accepted_labels: set[int],
    label: str,
) -> None:
    audit = value
    _require(isinstance(audit, Mapping), f"{label} component audit is missing")
    archive, _ = _lstat_regular(archive_path, f"{label} component archive")
    _verify_known_input_fingerprint(
        audit.get("archive", {}),
        actual_path=archive,
        actual=file_fingerprint(archive, ply=False),
        label=f"{label} component archive",
    )
    _require(
        set(audit.get("archive_arrays_verified", ())) == required_arrays,
        f"{label} component archive-array audit is incomplete",
    )
    _require(
        audit.get("candidate_xy_record_exact") is True,
        f"{label} component XY/record invariant is missing",
    )
    normalized = np.asarray(labels, np.int64)
    _require(
        int(audit.get("component_label_count", -1)) == len(normalized)
        and audit.get("component_label_sha256") == _array_sha256(normalized),
        f"{label} component-label hash/count mismatch",
    )
    declared_labels = audit.get("accepted_labels")
    _require(
        isinstance(declared_labels, list)
        and all(
            not isinstance(item, bool) and isinstance(item, (int, np.integer))
            for item in declared_labels
        )
        and len(declared_labels) == len(set(int(item) for item in declared_labels))
        and {int(item) for item in declared_labels} == accepted_labels,
        f"{label} accepted-component audit differs",
    )


def _ply_xy_within_bounds(
    paths: Sequence[str | Path],
    bounds: Sequence[tuple[float, float, float, float]],
    *,
    margin: float,
) -> np.ndarray:
    """Read only support XY capable of lying within ``margin`` of a hole."""

    chunks: list[np.ndarray] = []
    for path in paths:
        info = inspect_ply(path)
        records = np.memmap(
            info.path,
            dtype=info.dtype,
            mode="r",
            offset=info.offset,
            shape=(info.count,),
        )
        try:
            for begin in range(0, info.count, CHUNK_RECORDS):
                end = min(info.count, begin + CHUNK_RECORDS)
                x = np.asarray(records["x"][begin:end], np.float64)
                y = np.asarray(records["y"][begin:end], np.float64)
                keep = np.zeros(end - begin, dtype=bool)
                for xmin, xmax, ymin, ymax in bounds:
                    keep |= (
                        (x >= xmin - margin)
                        & (x <= xmax + margin)
                        & (y >= ymin - margin)
                        & (y <= ymax + margin)
                    )
                if np.any(keep):
                    chunks.append(np.column_stack((x[keep], y[keep])))
        finally:
            del records
    if not chunks:
        return np.empty((0, 2), dtype=np.float64)
    return np.concatenate(chunks, axis=0)


def _nearest_xy_distance(
    query: np.ndarray,
    support: np.ndarray,
    *,
    maximum_radius: float | None = None,
) -> np.ndarray:
    _require(query.ndim == 2 and query.shape[1] == 2, "nearest-distance query is invalid")
    _require(support.ndim == 2 and support.shape[1] == 2, "nearest-distance support is invalid")
    if not len(query):
        return np.empty(0, dtype=np.float64)
    if not len(support):
        return np.full(len(query), np.inf, dtype=np.float64)
    try:
        from scipy.spatial import cKDTree
    except ModuleNotFoundError:
        cKDTree = None
    if cKDTree is not None:
        distance, _ = cKDTree(support).query(query, k=1, workers=-1)
        return np.asarray(distance, np.float64)
    _require(
        maximum_radius is not None
        and np.isfinite(maximum_radius)
        and maximum_radius > 0.0,
        "a finite support radius is required when SciPy is unavailable",
    )
    radius = float(maximum_radius)
    # Exact fixed-radius nearest neighbours without SciPy.  Query points are
    # grouped by radius-sized cells; only the 3x3 neighbouring support cells
    # can contain a point within the radius.  The per-cell distance matrices
    # keep memory proportional to local density instead of query*support.
    origin = np.minimum(np.min(query, axis=0), np.min(support, axis=0))
    query_cell = np.floor((query - origin) / radius).astype(np.int64)
    support_cell = np.floor((support - origin) / radius).astype(np.int64)
    minimum_cell = np.minimum(
        np.min(query_cell, axis=0), np.min(support_cell, axis=0)
    )
    query_cell -= minimum_cell
    support_cell -= minimum_cell
    maximum_cell = np.maximum(
        np.max(query_cell, axis=0), np.max(support_cell, axis=0)
    )
    span_y = int(maximum_cell[1]) + 1
    _require(span_y > 0, "support-cell encoding is invalid")
    _require(
        int(maximum_cell[0]) <= (np.iinfo(np.int64).max // span_y) - 1,
        "support-cell encoding overflows int64",
    )
    support_key = support_cell[:, 0] * span_y + support_cell[:, 1]
    support_order = np.argsort(support_key, kind="stable")
    support_key = support_key[support_order]
    query_key = query_cell[:, 0] * span_y + query_cell[:, 1]
    query_order = np.argsort(query_key, kind="stable")
    ordered_query_key = query_key[query_order]
    boundaries = np.flatnonzero(
        np.r_[True, ordered_query_key[1:] != ordered_query_key[:-1], True]
    )
    output = np.full(len(query), np.inf, dtype=np.float64)
    offsets = tuple(
        (dx, dy) for dx in (-1, 0, 1) for dy in (-1, 0, 1)
    )
    radius_squared = radius * radius
    for group in range(len(boundaries) - 1):
        begin, end = int(boundaries[group]), int(boundaries[group + 1])
        query_indices = query_order[begin:end]
        cell_x, cell_y = query_cell[query_indices[0]]
        support_parts: list[np.ndarray] = []
        for dx, dy in offsets:
            neighbour_x = int(cell_x) + dx
            neighbour_y = int(cell_y) + dy
            if neighbour_x < 0 or neighbour_y < 0 or neighbour_y >= span_y:
                continue
            key = neighbour_x * span_y + neighbour_y
            left = int(np.searchsorted(support_key, key, side="left"))
            right = int(np.searchsorted(support_key, key, side="right"))
            if right > left:
                support_parts.append(support_order[left:right])
        if not support_parts:
            continue
        local_support = support[np.concatenate(support_parts)]
        query_chunk = max(1, 5_000_000 // max(len(local_support), 1))
        for local_begin in range(0, len(query_indices), query_chunk):
            local_indices = query_indices[
                local_begin : local_begin + query_chunk
            ]
            delta = (
                query[local_indices, None, :] - local_support[None, :, :]
            )
            squared = np.min(np.sum(delta * delta, axis=2), axis=1)
            within = squared <= radius_squared + 1.0e-18
            output[local_indices[within]] = np.sqrt(squared[within])
    return output


def _outward_stored_interval(
    lower: float,
    upper: float,
    dtype: np.dtype,
) -> tuple[float, float]:
    """Represent a JSON interval outward by exactly one stored-coordinate ULP."""

    dtype = np.dtype(dtype)
    _require(
        np.issubdtype(dtype, np.floating),
        "WATER archive coordinate field is not floating point",
    )
    lower_value = np.asarray(lower, dtype=dtype)
    upper_value = np.asarray(upper, dtype=dtype)
    return (
        float(
            np.nextafter(
                lower_value,
                np.asarray(-np.inf, dtype=dtype),
            )
        ),
        float(
            np.nextafter(
                upper_value,
                np.asarray(np.inf, dtype=dtype),
            )
        ),
    )


def _verify_coarse_fine_subset(
    coarse_manifest: Mapping,
    *,
    coarse_manifest_path: Path,
    coarse_archive_path: Path,
    fine_geometry: Mapping,
    support_paths: Sequence[str | Path],
) -> dict:
    """Prove coarse hole geometry is a deterministic exact fine subset."""

    cross = coarse_manifest.get("cross_scale")
    _require(isinstance(cross, Mapping), "5mm WATER geometry lacks cross-scale provenance")
    _require(
        cross.get("method") == "deterministic-variable-radius-blue-noise-subset-v1",
        "5mm WATER cross-scale method is unexpected",
    )
    _require(cross.get("fine_selection_index_unique") is True, "5mm fine-selection indices are not unique")
    _require(
        cross.get("coarse_xyz_exact_subset_of_fine_records_xyz") is True,
        "5mm WATER XYZ is not declared an exact fine-record subset",
    )
    _require(
        cross.get("coarse_normals_exact_subset_of_fine_records_normals") is True,
        "5mm WATER normals are not declared an exact fine-record subset",
    )
    _require(
        cross.get("nongeometry_fields_preserved_from_coarse_donors") is True,
        "5mm WATER nongeometry donor-field preservation is not attested",
    )
    geometry_fields = ["x", "y", "z", "nx", "ny", "nz"]
    _require(
        cross.get("geometry_fields_copied_from_fine_records") == geometry_fields,
        "5mm WATER fine-record geometry field declaration differs",
    )
    fine_manifest_path = Path(fine_geometry["final_manifest"])
    fine_manifest_fp = file_fingerprint(fine_manifest_path, ply=False)
    _verify_known_input_fingerprint(
        cross.get("fine_manifest", {}),
        actual_path=fine_manifest_path,
        actual=fine_manifest_fp,
        label="5mm cross-scale fine manifest",
    )
    fine_archive = fine_geometry["hole_archive"]
    fine_archive_path = Path(fine_archive["path"])
    _verify_known_input_fingerprint(
        cross.get("fine_archive", {}),
        actual_path=fine_archive_path,
        actual=fine_archive,
        label="5mm cross-scale fine archive",
    )
    _require(
        cross.get("fine_candidate_sha256")
        == fine_geometry["candidate"]["sha256"],
        "5mm cross-scale fine candidate hash mismatch",
    )
    _require(
        int(cross.get("fine_addition_count", -1))
        == int(fine_archive["points"]),
        "5mm cross-scale fine addition count mismatch",
    )

    fine_document = json.loads(fine_manifest_path.read_text(encoding="utf-8"))
    fine_holes = _accepted_holes(fine_document, "2mm WATER")
    coarse_holes = _accepted_holes(coarse_manifest, "5mm WATER")
    _require(coarse_holes == fine_holes, "5mm accepted hole IDs/footprints differ from 2mm")
    _require(
        coarse_manifest.get("review_bbox") == fine_document.get("review_bbox"),
        "5mm review footprint differs from 2mm",
    )

    with np.load(fine_archive_path, allow_pickle=False) as loaded:
        _require(
            {"records", "candidate_xy", "candidate_label"}.issubset(loaded.files),
            "2mm WATER archive lacks component-labelled geometry arrays",
        )
        fine_records = np.asarray(loaded["records"])
        fine_candidate_xy = np.asarray(loaded["candidate_xy"])
        fine_component_label = np.asarray(loaded["candidate_label"])
    with np.load(coarse_archive_path, allow_pickle=False) as loaded:
        _require(
            {
                "records",
                "candidate_xy",
                "fine_selection_index",
                "fine_component_label",
            }.issubset(loaded.files),
            "5mm WATER archive lacks exact fine-selection geometry arrays",
        )
        coarse_records = np.asarray(loaded["records"])
        coarse_candidate_xy = np.asarray(loaded["candidate_xy"])
        selection = np.asarray(loaded["fine_selection_index"])
        coarse_component_label = np.asarray(loaded["fine_component_label"])
    _record_xy_matches_archive(fine_records, fine_candidate_xy, "2mm WATER archive")
    _record_xy_matches_archive(coarse_records, coarse_candidate_xy, "5mm WATER archive")
    fine_component_label = _geometry_component_labels(
        fine_document,
        labels=fine_component_label,
        accepted_holes=fine_holes,
        label="2mm WATER geometry",
    )
    _require(selection.ndim == 1 and selection.dtype.kind in "iu", "5mm fine-selection index is invalid")
    _require(len(selection) == len(coarse_records), "5mm fine-selection count differs from additions")
    _require(
        coarse_component_label.ndim == 1
        and np.issubdtype(coarse_component_label.dtype, np.integer)
        and len(coarse_component_label) == len(coarse_records),
        "5mm fine-component labels are invalid",
    )
    coarse_component_label = coarse_component_label.astype(np.int64, copy=False)
    _require(
        len(selection) == int(cross.get("fine_selection_index_count", -1)),
        "5mm fine-selection count differs from manifest",
    )
    _require(
        not len(selection)
        or (int(selection.min()) >= 0 and int(selection.max()) < len(fine_records)),
        "5mm fine-selection index escapes fine additions",
    )
    _require(len(np.unique(selection)) == len(selection), "5mm fine-selection index is not unique")
    _require(
        set(geometry_fields).issubset(fine_records.dtype.names or ())
        and set(geometry_fields).issubset(coarse_records.dtype.names or ()),
        "WATER geometry archives lack XYZ/normal fields",
    )
    for coordinate in geometry_fields:
        _require(
            np.asarray(coarse_records[coordinate]).tobytes()
            == np.asarray(fine_records[coordinate][selection]).tobytes(),
            f"5mm {coordinate} is not an exact fine-record subset",
        )
    _require(
        np.array_equal(coarse_component_label, fine_component_label[selection]),
        "5mm component labels differ from the selected fine additions",
    )

    coverage = cross.get("accepted_hole_coverage")
    _require(isinstance(coverage, list), "5mm accepted-hole coverage is missing")
    _require(
        all(isinstance(row, Mapping) for row in coverage),
        "5mm accepted-hole coverage contains an invalid row",
    )
    coverage_by_id: dict[str, Mapping] = {}
    coverage_labels: set[int] = set()
    for row in coverage:
        seed_id = str(row.get("seed_id", ""))
        component_value = row.get("component_label")
        _require(seed_id and seed_id not in coverage_by_id, "5mm coverage hole ID is empty or duplicated")
        _require(
            not isinstance(component_value, bool)
            and isinstance(component_value, (int, np.integer)),
            f"5mm coverage component label is invalid for {seed_id}",
        )
        component_label = int(component_value)
        _require(
            component_label not in coverage_labels,
            "5mm coverage component label is duplicated",
        )
        coverage_labels.add(component_label)
        coverage_by_id[seed_id] = row
    _require(set(coverage_by_id) == set(fine_holes), "5mm accepted-hole coverage IDs differ from 2mm")
    fine_total = 0
    coarse_total = 0
    for seed_id, (component_label, bounds) in fine_holes.items():
        row = coverage_by_id[seed_id]
        _require(
            int(row.get("component_label", -1)) == component_label,
            f"5mm coverage component label differs for {seed_id}",
        )
        _require(tuple(float(value) for value in row.get("bounds", ())) == bounds, f"5mm coverage footprint differs for {seed_id}")
        fine_mask = fine_component_label == component_label
        coarse_mask = coarse_component_label == component_label
        fine_count = int(np.count_nonzero(fine_mask))
        coarse_count = int(np.count_nonzero(coarse_mask))
        xmin, xmax, ymin, ymax = bounds
        # The accepted footprint is recorded as JSON float64, while production
        # PLY/archive coordinates are float32.  Comparing those representations
        # directly can reject a coordinate that is exactly the stored form of a
        # decimal boundary (for example 772.93 -> 772.9299926757812).  Expand
        # each boundary by one ULP in the archive coordinate dtype: this admits
        # representation error only, while retaining the component-footprint
        # hard gate for geometrically out-of-bounds additions.
        x_dtype = fine_records.dtype.fields["x"][0]
        y_dtype = fine_records.dtype.fields["y"][0]
        xmin_stored, xmax_stored = _outward_stored_interval(
            xmin, xmax, x_dtype
        )
        ymin_stored, ymax_stored = _outward_stored_interval(
            ymin, ymax, y_dtype
        )
        fine_xy_for_hole = np.column_stack(
            (fine_records["x"][fine_mask], fine_records["y"][fine_mask])
        ).astype(np.float64)
        _require(
            np.all(
                (fine_xy_for_hole[:, 0] >= xmin_stored)
                & (fine_xy_for_hole[:, 0] <= xmax_stored)
                & (fine_xy_for_hole[:, 1] >= ymin_stored)
                & (fine_xy_for_hole[:, 1] <= ymax_stored)
            ),
            f"2mm component geometry escapes its accepted footprint for {seed_id}",
        )
        _require(
            int(row.get("fine_count", -1)) == fine_count,
            f"5mm coverage fine count is false for {seed_id}",
        )
        _require(
            int(row.get("coarse_addition_count", -1)) == coarse_count,
            f"5mm coverage coarse count is false for {seed_id}",
        )
        _require(fine_count > 0, f"2mm accepted hole {seed_id} has no additions")
        _require(coarse_count > 0, f"5mm subset misses accepted hole {seed_id}")
        _require(coarse_count <= fine_count, f"5mm subset exceeds fine count for {seed_id}")
        fine_total += fine_count
        coarse_total += coarse_count
    _require(fine_total == len(fine_records), "2mm component coverage does not partition additions")
    _require(coarse_total == len(coarse_records), "5mm component coverage does not partition additions")
    spacing = float(cross.get("spacing_m", np.nan))
    reported_maximum = float(
        cross.get("maximum_fine_to_coarse_or_terrain_support_distance_m", np.nan)
    )
    _require(
        np.isfinite(spacing)
        and spacing > 0.0
        and np.isfinite(reported_maximum)
        and reported_maximum >= 0.0,
        "5mm subset support-distance declaration is invalid",
    )
    fine_xy = np.column_stack((fine_records["x"], fine_records["y"])).astype(np.float64)
    support_xy = _ply_xy_within_bounds(
        support_paths,
        [value[1] for value in fine_holes.values()],
        margin=spacing,
    )
    coarse_xy = np.column_stack((coarse_records["x"], coarse_records["y"])).astype(np.float64)
    support_xy = np.concatenate((support_xy, coarse_xy), axis=0)
    distances = _nearest_xy_distance(
        fine_xy, support_xy, maximum_radius=spacing
    )
    maximum = float(np.max(distances)) if len(distances) else 0.0
    _require(
        np.isclose(maximum, reported_maximum, rtol=0.0, atol=1.0e-9),
        "5mm reported maximum support distance differs from independent geometry",
    )
    _require(
        maximum <= spacing + 1.0e-9,
        "5mm subset does not cover the accepted 2mm footprint",
    )
    return {
        "verified": True,
        "method": cross["method"],
        "fine_additions": len(fine_records),
        "coarse_additions": len(coarse_records),
        "accepted_hole_ids": sorted(fine_holes),
        "fine_manifest": str(fine_manifest_path),
        "fine_archive": str(fine_archive_path),
        "coarse_manifest": str(coarse_manifest_path),
        "maximum_support_distance_m": maximum,
        "maximum_support_distance_independently_recomputed": True,
        "component_label_coverage_independently_recomputed": True,
        "coarse_xyz_exact_fine_subset_independently_verified": True,
        "coarse_normals_exact_fine_subset_independently_verified": True,
    }


def _load_indices(path: str | Path, source_count: int) -> np.ndarray:
    source = Path(path)
    with source.open("rb") as handle:
        numpy_magic = handle.read(6) == b"\x93NUMPY"
    if numpy_magic:
        indices = np.load(source, mmap_mode="r", allow_pickle=False)
    else:
        if source.stat().st_size % np.dtype("<u8").itemsize:
            raise RuntimeError(
                f"raw removed-index sidecar is not uint64-aligned: {source}"
            )
        indices = (
            np.memmap(source, mode="r", dtype="<u8")
            if source.stat().st_size
            else np.empty(0, dtype="<u8")
        )
    if indices.ndim != 1 or indices.dtype.kind not in "iu":
        raise RuntimeError(f"removed-index sidecar is not an integer vector: {path}")
    if len(indices):
        if int(indices[0]) < 0 or int(indices[-1]) >= source_count:
            raise RuntimeError("removed indices escape the source")
        if np.any(indices[1:] <= indices[:-1]):
            raise RuntimeError("removed indices are not strictly increasing")
    return indices


def verify_exact_survivors(
    source_path: str | Path,
    candidate_path: str | Path,
    removed_indices_path: str | Path,
    survivor_count: int,
    *,
    removed_records_path: str | Path | None = None,
    chunk_records: int = CHUNK_RECORDS,
) -> dict:
    """Verify an exact ordered source partition without loading it in RAM."""

    source = inspect_ply(source_path)
    candidate = inspect_ply(candidate_path)
    indices = _load_indices(removed_indices_path, source.count)
    survivors = int(survivor_count)
    _require(source.count - len(indices) == survivors, "survivor count/index count mismatch")
    _require(candidate.count >= survivors, "candidate is shorter than survivor prefix")
    _require(source.schema == candidate.schema, "survivor source/candidate schemas differ")
    removed_info = inspect_ply(removed_records_path) if removed_records_path else None
    if removed_info is not None:
        _require(removed_info.schema == source.schema, "removed archive schema differs")
        _require(removed_info.count == len(indices), "removed archive count differs")
    source_records = np.memmap(
        source.path, dtype=source.dtype, mode="r", offset=source.offset,
        shape=(source.count,),
    )
    candidate_records = np.memmap(
        candidate.path, dtype=candidate.dtype, mode="r", offset=candidate.offset,
        shape=(candidate.count,),
    )
    removed_records = None
    if removed_info is not None:
        removed_records = np.memmap(
            removed_info.path, dtype=removed_info.dtype, mode="r",
            offset=removed_info.offset, shape=(removed_info.count,),
        )
    survivor_cursor = 0
    removed_cursor = 0
    try:
        for begin in range(0, source.count, chunk_records):
            end = min(source.count, begin + chunk_records)
            lo = int(np.searchsorted(indices, begin, side="left"))
            hi = int(np.searchsorted(indices, end, side="left"))
            local_removed = np.asarray(indices[lo:hi], np.int64) - begin
            keep = np.ones(end - begin, dtype=bool)
            keep[local_removed] = False
            kept = source_records[begin:end][keep]
            observed = candidate_records[
                survivor_cursor : survivor_cursor + len(kept)
            ]
            if _records_bytes(kept) != _records_bytes(observed):
                raise RuntimeError(
                    f"survivor payload differs in source block {begin}:{end}"
                )
            survivor_cursor += len(kept)
            if removed_records is not None and len(local_removed):
                removed = source_records[begin:end][~keep]
                observed_removed = removed_records[
                    removed_cursor : removed_cursor + len(removed)
                ]
                if _records_bytes(removed) != _records_bytes(observed_removed):
                    raise RuntimeError(
                        f"removed archive differs in source block {begin}:{end}"
                    )
                removed_cursor += len(removed)
    finally:
        del source_records, candidate_records
        if removed_records is not None:
            del removed_records
    _require(survivor_cursor == survivors, "survivor verification ended early")
    _require(removed_cursor in (0, len(indices)), "removed verification ended early")
    return {
        "verified": True,
        "source_points": source.count,
        "survivor_points": survivors,
        "removed_points": len(indices),
        "candidate_suffix_points": candidate.count - survivors,
        "removed_archive_verified": removed_info is not None,
    }


def _scan_appended_scan_id(
    candidate_path: Path, base_count: int, expected: float
) -> dict:
    info = inspect_ply(candidate_path)
    names = set(info.dtype.names or ())
    field = next(
        (name for name in ("scalar_ScanID", "ScanID", "scanID", "scan_id") if name in names),
        None,
    )
    _require(field is not None, f"{candidate_path}: no ScanID field")
    records = np.memmap(
        info.path, dtype=info.dtype, mode="r", offset=info.offset,
        shape=(info.count,),
    )
    invalid = 0
    try:
        for begin in range(base_count, info.count, CHUNK_RECORDS):
            values = np.asarray(records[field][begin : begin + CHUNK_RECORDS], np.float64)
            invalid += int(np.count_nonzero(~np.isfinite(values) | (np.rint(values) != expected)))
    finally:
        del records
    _require(invalid == 0, f"{candidate_path}: {invalid} appended records lack ScanID {expected:g}")
    return {"field": field, "expected": expected, "checked": info.count - base_count}


def app_running() -> bool:
    completed = subprocess.run(
        ["pgrep", "-f", "MacOS/invisible_places"],
        check=False,
        capture_output=True,
        text=True,
    )
    if completed.returncode == 0:
        return bool(completed.stdout.strip())
    if completed.returncode == 1:
        return False
    detail = completed.stderr.strip() or f"exit status {completed.returncode}"
    raise RuntimeError(
        "unable to determine whether invisible_places is running: " + detail
    )


def refuse_running_app() -> None:
    if app_running():
        raise RuntimeError("refusing: invisible_places is running")


@contextmanager
def release_lock(release_dir: Path):
    """Lock the shared release/transaction root, independent of ``run_dir``."""

    release_dir = Path(release_dir)
    release_dir.parent.mkdir(parents=True, exist_ok=True)
    if os.path.lexists(release_dir):
        _lstat_directory(release_dir, "release directory")
    parent = release_dir.parent.resolve(strict=True)
    lock_path = parent / f".{release_dir.name}.site1-v11-release.lock"
    flags = (
        os.O_RDWR
        | getattr(os, "O_CLOEXEC", 0)
        | getattr(os, "O_NOFOLLOW", 0)
        | getattr(os, "O_NONBLOCK", 0)
    )
    try:
        descriptor = os.open(lock_path, flags | os.O_CREAT | os.O_EXCL, 0o600)
    except FileExistsError:
        _, existing = _lstat_regular(lock_path, "release lock")
        _require(
            existing.st_nlink == 1,
            f"release lock has multiple hard links: {lock_path}",
        )
        try:
            descriptor = os.open(lock_path, flags)
        except OSError as error:
            raise RuntimeError(
                f"unable to open release lock safely: {lock_path}"
            ) from error
    except OSError as error:
        raise RuntimeError(f"unable to open release lock safely: {lock_path}") from error
    try:
        opened = os.fstat(descriptor)
        lexical = os.lstat(lock_path)
        _require(
            stat.S_ISREG(opened.st_mode)
            and stat.S_ISREG(lexical.st_mode)
            and (opened.st_dev, opened.st_ino) == (lexical.st_dev, lexical.st_ino)
            and opened.st_nlink == lexical.st_nlink == 1,
            f"release lock is not one stable regular file: {lock_path}",
        )
        handle = os.fdopen(descriptor, "r+", encoding="utf-8", closefd=True)
        try:
            fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            handle.close()
            raise RuntimeError("another Scene1 v11 release action holds the lock") from error
        current = os.lstat(lock_path)
        _require(
            stat.S_ISREG(current.st_mode)
            and (current.st_dev, current.st_ino) == (opened.st_dev, opened.st_ino)
            and current.st_nlink == os.fstat(handle.fileno()).st_nlink == 1,
            f"release lock directory entry changed while locking: {lock_path}",
        )
        try:
            yield
        finally:
            fcntl.flock(handle.fileno(), fcntl.LOCK_UN)
            handle.close()
    except BaseException:
        try:
            os.close(descriptor)
        except OSError:
            pass
        raise


def _clone_or_copy(source: Path, destination: Path) -> str:
    source, _ = _lstat_regular(source, "snapshot source")
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination = _entry_path(destination, "snapshot destination")
    if os.path.lexists(destination):
        raise FileExistsError(f"refusing to overwrite {destination}")
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{destination.name}.",
        suffix=".partial",
        dir=destination.parent,
    )
    os.close(descriptor)
    temporary = Path(temporary_name)
    completed = subprocess.run(
        ["cp", "-c", str(source), str(temporary)],
        check=False,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    method = "apfs-clone"
    if completed.returncode != 0:
        required = os.lstat(source).st_size + 1024 * 1024 * 1024
        available = shutil.disk_usage(destination.parent).free
        _require(
            available >= required,
            "APFS clone failed and there is insufficient space for a safe "
            f"snapshot copy of {source} (need {required} bytes including reserve; "
            f"have {available})",
        )
        shutil.copy2(source, temporary)
        method = "copy2"
    _, _, descriptor = _open_regular(temporary, "snapshot staging file")
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    if os.path.lexists(destination):
        temporary.unlink()
        raise RuntimeError(f"snapshot destination appeared during copy: {destination}")
    _durable_replace(temporary, destination)
    return method


def _artifact(path: Path, artifacts: dict[str, dict]) -> dict:
    key = str(_entry_path(path, "release artifact"))
    if key not in artifacts:
        artifacts[key] = file_fingerprint(path)
    return artifacts[key]


def _verify_input_fingerprint(
    value: Mapping,
    *,
    parent: Path,
    label: str,
    artifacts: dict[str, dict],
    path_key: str = "path",
) -> dict:
    """Verify a non-PLY input fingerprint recorded by a candidate stage."""

    _require(isinstance(value, Mapping), f"{label} fingerprint is not an object")
    _require(path_key in value and "sha256" in value, f"{label} fingerprint is incomplete")
    _require(
        isinstance(value[path_key], (str, os.PathLike))
        and isinstance(value["sha256"], str),
        f"{label} fingerprint has invalid path/hash values",
    )
    path = _resolve(value[path_key], parent)
    stat = path.stat()
    _require(sha256_path(path) == value["sha256"], f"{label} hash mismatch")
    if "size_bytes" in value:
        _require(stat.st_size == int(value["size_bytes"]), f"{label} size mismatch")
    if "mtime_ns" in value:
        _require(stat.st_mtime_ns == int(value["mtime_ns"]), f"{label} mtime mismatch")
    _artifact(path, artifacts)
    return {
        "path": str(path),
        "size_bytes": stat.st_size,
        "mtime_ns": stat.st_mtime_ns,
        "sha256": value["sha256"],
    }


def _verify_known_input_fingerprint(
    value: Mapping,
    *,
    actual_path: str | Path,
    actual: Mapping,
    label: str,
) -> dict:
    """Cross-check a stage input against an artifact already fully hashed."""

    _require(isinstance(value, Mapping), f"{label} fingerprint is not an object")
    _require(
        isinstance(value.get("path"), str)
        and isinstance(value.get("sha256"), str),
        f"{label} fingerprint is incomplete",
    )
    path, entry_stat = _lstat_regular(value["path"], label)
    expected_path, _ = _lstat_regular(actual_path, f"{label} expected input")
    _require(path == expected_path, f"{label} path mismatch")
    _require(value["sha256"] == actual["sha256"], f"{label} hash mismatch")
    file_stat = entry_stat
    if "size_bytes" in value:
        _require(int(value["size_bytes"]) == file_stat.st_size, f"{label} size mismatch")
    if "mtime_ns" in value:
        _require(int(value["mtime_ns"]) == file_stat.st_mtime_ns, f"{label} mtime mismatch")
    if "bytes" in actual:
        _require(int(actual["bytes"]) == file_stat.st_size, f"{label} verified size drift")
    if "points" in value:
        _require(int(value["points"]) == int(actual["points"]), f"{label} point count mismatch")
    if "record_stride" in value:
        _require(
            int(value["record_stride"]) == int(actual["record_stride"]),
            f"{label} record stride mismatch",
        )
    return {
        "path": str(path),
        "size_bytes": file_stat.st_size,
        "mtime_ns": file_stat.st_mtime_ns,
        "sha256": value["sha256"],
    }


def _verify_implementation_hashes(
    value: Mapping,
    *,
    label: str,
    artifacts: dict[str, dict],
) -> dict[str, str]:
    """Bind a stage to the exact checked-in Python modules that produced it."""

    _require(isinstance(value, Mapping) and bool(value), f"{label} implementation hashes are missing")
    verified: dict[str, str] = {}
    for filename, expected_hash in sorted(value.items()):
        name = str(filename)
        _require(Path(name).name == name, f"{label} implementation name is unsafe: {name}")
        path, _ = _lstat_regular(
            SCRIPT_DIR / name, f"{label} implementation {name}"
        )
        _require(path.parent == SCRIPT_DIR.resolve(), f"{label} implementation escaped scripts: {name}")
        _require(isinstance(expected_hash, str), f"{label} implementation hash is not text: {name}")
        _require(sha256_path(path) == expected_hash, f"{label} implementation hash mismatch: {name}")
        _artifact(path, artifacts)
        verified[name] = expected_hash
    return verified


def _verify_reference_provenance(
    value: Mapping,
    *,
    parent: Path,
    label: str,
    artifacts: dict[str, dict],
) -> dict:
    _require(isinstance(value, Mapping), f"{label} reference provenance is missing")
    result = dict(value)
    for key in ("surface_archive", "surface_config", "implementation"):
        result[key] = _verify_input_fingerprint(
            value.get(key, {}),
            parent=parent,
            label=f"{label} reference {key}",
            artifacts=artifacts,
        )
    _require(
        value.get("callable") == "rebuild_site1_fossils_v10.surface_values",
        f"{label} reference callable is unexpected",
    )
    try:
        noise_scale = float(value.get("noise_scale"))
    except (TypeError, ValueError) as error:
        raise RuntimeError(f"{label} reference noise scale is invalid") from error
    _require(np.isfinite(noise_scale) and noise_scale > 0.0, f"{label} reference noise scale is invalid")
    return result


def _canonical_source(data_dir: Path, name: str) -> Path:
    path, _ = _lstat_regular(data_dir / name, "canonical source")
    _require(path.parent == data_dir.resolve(), f"canonical path escaped data directory: {name}")
    return path


def _source_entry(path: Path, expected_hash: str, expected_points: int | None = None) -> dict:
    actual = file_fingerprint(path)
    _require(actual["sha256"] == expected_hash, f"source hash mismatch: {path}")
    if expected_points is not None:
        _require(actual["points"] == expected_points, f"source point count mismatch: {path}")
    return actual


def _candidate_entry(
    path: Path,
    expected_hash: str,
    expected_points: int,
    source: Mapping,
) -> dict:
    actual = file_fingerprint(path)
    _require(actual["sha256"] == expected_hash, f"candidate hash mismatch: {path}")
    _require(actual["points"] == int(expected_points), f"candidate point count mismatch: {path}")
    _require(actual["schema"] == source["schema"], f"candidate schema differs from canonical: {path}")
    invalid = _finite_xyz(path)
    _require(invalid == 0, f"candidate contains {invalid} non-finite XYZ records: {path}")
    actual["non_finite_xyz"] = invalid
    return actual


def _parse_obstructions(
    manifest_path: Path,
    data_dir: Path,
    artifacts: dict[str, dict],
) -> dict[str, dict]:
    top_path, top = _load_json(manifest_path, "obstruction manifest")
    _artifact(top_path, artifacts)
    _require(top.get("operation") == "site1-v11-candidate-only-obstruction-pipeline", "unexpected obstruction operation")
    _require(top.get("canonical_writes") is False, "obstruction stage reports canonical writes")
    parameters = _verify_obstruction_parameters(top.get("parameters"))
    implementation = _verify_exact_implementation_hashes(
        top.get("implementation", {}),
        family="obstruction",
        label="obstruction",
        artifacts=artifacts,
    )
    selection = top.get("selection_contract")
    _require(isinstance(selection, Mapping), "obstruction selection contract is missing")
    selection_links = {
        "minimum_agreeing_surface_models": parameters["thresholds"]["minimum_models"],
        "maximum_model_spread_m": parameters["thresholds"]["maximum_model_spread_m"],
        "seed_height_m": parameters["thresholds"]["seed_height_m"],
        "grow_height_m": parameters["thresholds"]["grow_height_m"],
        "ground_stop_height_m": parameters["thresholds"]["ground_stop_height_m"],
        "cross_scale_distance_m": parameters["cross_scale_distance_m"],
    }
    for name, expected in selection_links.items():
        _require(selection.get(name) == expected, f"obstruction selection contract differs: {name}")
    for name in (
        "bbox_is_locating_evidence_only",
        "requires_seed_connected_3d_component",
        "scanid9_absolute_protection",
    ):
        _require(selection.get(name) is True, f"obstruction selection contract failed: {name}")
    config_block = top.get("config")
    _require(isinstance(config_block, Mapping), "obstruction config fingerprint is missing")
    config = _verify_input_fingerprint(
        {
            "path": config_block.get("source_path"),
            "sha256": config_block.get("sha256"),
        },
        parent=top_path.parent,
        label="obstruction review config",
        artifacts=artifacts,
    )
    _require(
        isinstance(config_block.get("archived_copy"), str)
        and bool(config_block["archived_copy"]),
        "obstruction archived review config is missing",
    )
    archived_config = _resolve(config_block["archived_copy"], top_path.parent)
    _require(
        sha256_path(archived_config) == config["sha256"],
        "obstruction archived review config hash mismatch",
    )
    _artifact(archived_config, artifacts)
    result: dict[str, dict] = {}
    for label, section_name, canonical_name in (
        ("ROCK-1mm", "fine", "Site1-ROCK-1mm.ply"),
        ("ROCK-5mm", "coarse", "Site1-ROCK-5mm.ply"),
    ):
        section = top[section_name]
        _require(section["preservation"]["passed"] is True, f"{label} preservation failed")
        round_trip = section["round_trip"]
        _require(round_trip["passed"] is True, f"{label} round trip failed")
        _require(round_trip["reconstructed_sha256"] == section["source_sha256"], f"{label} reconstructed hash mismatch")
        bundle = _resolve(section["bundle"], top_path.parent)
        bundle_manifest_path, bundle_manifest = _load_json(bundle / "manifest.json", f"{label} bundle manifest")
        _artifact(bundle_manifest_path, artifacts)
        _require(bundle_manifest.get("canonical_writes") is False, f"{label} bundle reports canonical writes")
        source_path = _canonical_source(data_dir, canonical_name)
        source_block = bundle_manifest["source"]
        _require(_same_path(source_block["path"], source_path), f"{label} source path is not canonical")
        source = _source_entry(source_path, section["source_sha256"], source_block["points"])
        candidate_path = _resolve(bundle_manifest["candidate"]["path"], bundle)
        candidate = _candidate_entry(
            candidate_path,
            bundle_manifest["candidate"]["sha256"],
            bundle_manifest["candidate"]["points"],
            source,
        )
        removed_path = _resolve(bundle_manifest["removed"]["records_path"], bundle)
        indices_path = _resolve(bundle_manifest["removed"]["indices_path"], bundle)
        _require(bundle_manifest["removed"]["records_are_exact_source_bytes"] is True, f"{label} removed archive is not exact")
        _require(sha256_path(removed_path) == bundle_manifest["removed"]["records_sha256"], f"{label} removed archive hash mismatch")
        _artifact(removed_path, artifacts)
        _artifact(indices_path, artifacts)
        partition = verify_exact_survivors(
            source_path,
            candidate_path,
            indices_path,
            int(candidate["points"]),
            removed_records_path=removed_path,
        )
        result[label] = {
            "canonical": canonical_name,
            "source": source,
            "obstruction_candidate": candidate,
            "partition": partition,
            "bundle_manifest": str(bundle_manifest_path),
            "config": config,
            "parameters": parameters,
            "implementation": implementation,
        }
    return result


def _parse_water_geometry_chain(
    *,
    label: str,
    base_manifest_path: Path,
    geometry_manifest_path: Path,
    data_dir: Path,
    terrain: Mapping[str, Mapping],
    fine_geometry: Mapping | None,
    artifacts: dict[str, dict],
) -> dict:
    base_path, base = _load_json(base_manifest_path, f"{label} WATER base manifest")
    final_path, final = _load_json(geometry_manifest_path, f"{label} WATER geometry manifest")
    _artifact(base_path, artifacts)
    _artifact(final_path, artifacts)
    _require(base.get("candidate_only") is True, f"{label} WATER base is not candidate-only")
    _require(base.get("canonical_install_performed") is False, f"{label} WATER base reports install")
    invariants = base.get("invariants", {})
    _require(invariants.get("existing_survivors_byte_exact") is True, f"{label} WATER survivor invariant missing")
    _require(invariants.get("heights_or_scalars_of_existing_records_rewritten") is False, f"{label} WATER existing records were rewritten")
    _require(
        invariants.get("recovery_validated_against_final_surviving_water") is True,
        f"{label} WATER recovery was not validated post-thinning",
    )
    base_config = _verify_input_fingerprint(
        base.get("config", {}),
        parent=base_path.parent,
        label=f"{label} WATER base review config",
        artifacts=artifacts,
    )
    parameters = _verify_water_base_parameters(
        base.get("parameters"), base.get("density_reference")
    )
    _require(
        base.get("interface_mark_ids") == parameters["interface_mark_ids"],
        f"{label} WATER interface-mark parameters disagree",
    )
    base_implementation = _verify_exact_implementation_hashes(
        base.get("implementation", {}),
        family="water_base",
        label=f"{label} WATER base",
        artifacts=artifacts,
    )
    canonical_name = f"Site1-WATER-{label}.ply"
    canonical = _canonical_source(data_dir, canonical_name)
    source_values = base.get("sources")
    _require(
        isinstance(source_values, Mapping) and len(source_values) == 5,
        f"{label} WATER base must fingerprint exactly five source clouds",
    )
    verified_sources: dict[Path, dict] = {}
    for source_key, source_block in source_values.items():
        _require(isinstance(source_block, Mapping), f"{label} WATER source fingerprint is invalid")
        recorded_path, _ = _lstat_regular(
            str(source_block.get("path", "")),
            f"{label} WATER base source",
        )
        _require(
            _entry_path(str(source_key), f"{label} WATER source key")
            == recorded_path,
            f"{label} WATER source key/path mismatch",
        )
        _require(recorded_path not in verified_sources, f"{label} WATER source is duplicated")
        actual = file_fingerprint(recorded_path)
        _verify_known_input_fingerprint(
            source_block,
            actual_path=recorded_path,
            actual=actual,
            label=f"{label} WATER base source {recorded_path.name}",
        )
        _artifact(recorded_path, artifacts)
        verified_sources[recorded_path] = actual
    _require(canonical in verified_sources, f"{label} WATER base lacks canonical source")
    source = verified_sources[canonical]
    terrain_resolution = "1mm" if label == "2mm" else "5mm"
    expected_base_terrain = {
        _lstat_regular(
            terrain[f"{role}-{terrain_resolution}"]["candidate"]["path"],
            f"{label} WATER terrain candidate",
        )[0]
        for role in ("SAND", "ROCK")
    }
    _require(
        expected_base_terrain.issubset(verified_sources),
        f"{label} WATER base lacks the verified final terrain candidates",
    )
    comparison_directory, _ = _lstat_directory(
        data_dir
        / "PatchRefinement"
        / "20260826-water-v10-blue-noise",
        f"{label} WATER comparison directory",
    )
    expected_comparisons = {
        _lstat_regular(
            comparison_directory / f"combined-{label}-selected.ply",
            f"{label} WATER pre-terrain comparison",
        )[0],
        _lstat_regular(
            comparison_directory / f"combined-{label}-selected-allterrain.ply",
            f"{label} WATER post-terrain comparison",
        )[0],
    }
    comparison_sources = set(verified_sources) - expected_base_terrain - {canonical}
    _require(
        comparison_sources == expected_comparisons,
        f"{label} WATER base comparison-source set is not the approved v10 pair",
    )
    pre_comparison = comparison_directory / f"combined-{label}-selected.ply"
    post_comparison = (
        comparison_directory / f"combined-{label}-selected-allterrain.ply"
    )
    pre_info = inspect_ply(pre_comparison)
    post_info = inspect_ply(post_comparison)
    _require(
        pre_info.schema == post_info.schema,
        f"{label} WATER pre/post comparison schemas differ",
    )
    required_comparison_fields = {
        "x", "y", "z", "scalar_ScanID",
    }
    _require(
        required_comparison_fields.issubset(pre_info.dtype.names or ()),
        f"{label} WATER comparison sources lack required recovery fields",
    )
    _require(
        post_info.count <= pre_info.count,
        f"{label} WATER post-allterrain source exceeds its pre source",
    )
    for comparison in (pre_comparison, post_comparison):
        invalid = _finite_xyz(comparison)
        _require(
            invalid == 0,
            f"{label} WATER comparison source has {invalid} non-finite XYZ",
        )
    base_candidate_path = _resolve(base["candidate"]["path"], base_path.parent)
    base_candidate = _candidate_entry(
        base_candidate_path,
        base["candidate"]["sha256"],
        base["candidate"]["points"],
        source,
    )
    thinning = base["thinning"]
    rejected_path = _resolve(thinning["rejected_index_path"], base_path.parent)
    _require(sha256_path(rejected_path) == thinning["rejected_index_sha256"], f"{label} thinning index hash mismatch")
    _artifact(rejected_path, artifacts)
    recovery_block = base["recovery"]
    _require(
        recovery_block.get("support_is_post_thinning") is True,
        f"{label} recovery support is not post-thinning",
    )
    _require(
        recovery_block.get("recovery_passes_pointwise_retention") is True,
        f"{label} recovery did not pass pointwise retention",
    )
    recovery_links = {
        "relaxed_terrain_clearance_m": (
            float(parameters["nominal_spacing_m"])
            * float(parameters["relaxed_terrain_ratio"])
        ),
        "nominal_terrain_blocker_m": float(parameters["nominal_spacing_m"]),
        "duplicate_clearance_m": (
            float(parameters["nominal_spacing_m"])
            * float(parameters["duplicate_clearance_ratio"])
        ),
        "maximum_bridge_m": float(parameters["maximum_bridge_m"]),
        "minimum_water_support": float(parameters["minimum_water_support"]),
    }
    for name, expected in recovery_links.items():
        observed = _finite_number(recovery_block.get(name), f"{label} recovery {name}")
        _require(
            np.isclose(observed, expected, rtol=0.0, atol=1.0e-12),
            f"{label} WATER recovery parameters disagree: {name}",
        )
    _require(
        int(invariants.get("pointwise_hash_seed", -1)) == int(parameters["seed"]),
        f"{label} WATER thinning seed provenance disagrees",
    )
    recovery_path, recovery_audit = _load_json(
        _resolve(recovery_block["audit_path"], base_path.parent),
        f"{label} WATER recovery audit",
    )
    _require(sha256_path(recovery_path) == recovery_block["audit_sha256"], f"{label} recovery audit hash mismatch")
    _artifact(recovery_path, artifacts)
    _require(recovery_audit.get("candidate_only") is True, f"{label} recovery audit is not candidate-only")
    _require(
        recovery_audit.get("schema_version") == 1
        and recovery_audit.get("eligibility")
        == "exact pre_allterrain minus post_allterrain WATER subsequence",
        f"{label} WATER recovery eligibility provenance differs",
    )
    _require(recovery_audit.get("support_is_post_thinning") is True, f"{label} recovery audit lacks post-thinning support")
    _require(recovery_audit.get("recovery_passes_pointwise_retention") is True, f"{label} recovery audit lacks pointwise retention")
    _require(int(recovery_audit.get("accepted", -1)) == int(recovery_block["accepted"]), f"{label} recovery accepted counts disagree")
    kept = int(thinning["kept_points"])
    _require(int(thinning["source_points"]) == source["points"], f"{label} thinning source count mismatch")
    _require(kept + int(base["recovery"]["accepted"]) == base_candidate["points"], f"{label} WATER base count chain mismatch")
    survivor = verify_exact_survivors(canonical, base_candidate_path, rejected_path, kept)

    _require(final.get("candidate_only") is True, f"{label} WATER final is not candidate-only")
    _require(final.get("canonical_install_performed") is False, f"{label} WATER final reports install")
    _require(final.get("existing_payload_byte_exact") is True, f"{label} WATER final prefix invariant missing")
    final_config = _verify_input_fingerprint(
        final.get("config", {}),
        parent=final_path.parent,
        label=f"{label} WATER final review config",
        artifacts=artifacts,
    )
    _require(final_config["sha256"] == base_config["sha256"], f"{label} WATER stage config hashes disagree")
    _require(Path(final_config["path"]) == Path(base_config["path"]), f"{label} WATER stage config paths disagree")
    final_implementation = _verify_exact_implementation_hashes(
        final.get("implementation", {}),
        family="water_geometry",
        label=f"{label} WATER final",
        artifacts=artifacts,
    )
    reference_provenance = _verify_reference_provenance(
        final.get("reference_provenance", {}),
        parent=final_path.parent,
        label=f"{label} WATER final",
        artifacts=artifacts,
    )
    expected_terrain = {
        _lstat_regular(
            terrain[f"{role}-{terrain_resolution}"]["candidate"]["path"],
            f"{label} WATER expected terrain source",
        )[0]: terrain[f"{role}-{terrain_resolution}"]["candidate"]
        for role in ("SAND", "ROCK")
    }
    recorded_terrain = final.get("terrain_sources")
    _require(
        isinstance(recorded_terrain, list) and len(recorded_terrain) == 2,
        f"{label} WATER geometry lacks its two terrain-source fingerprints",
    )
    observed_terrain_paths: set[Path] = set()
    for index, recorded in enumerate(recorded_terrain):
        _require(isinstance(recorded, Mapping), f"{label} terrain source {index} is invalid")
        recorded_path, _ = _lstat_regular(
            str(recorded.get("path", "")),
            f"{label} WATER terrain source {index}",
        )
        _require(recorded_path in expected_terrain, f"{label} WATER used an unexpected terrain source")
        _verify_known_input_fingerprint(
            recorded,
            actual_path=recorded_path,
            actual=expected_terrain[recorded_path],
            label=f"{label} WATER terrain source {index}",
        )
        observed_terrain_paths.add(recorded_path)
    _require(
        observed_terrain_paths == set(expected_terrain),
        f"{label} WATER terrain source set is incomplete",
    )
    _require(final["source"]["sha256"] == base_candidate["sha256"], f"{label} WATER final source hash does not match base")
    _require(_same_path(final["source"]["path"], base_candidate_path), f"{label} WATER final source path does not match base")
    _require(int(final["source"]["points"]) == base_candidate["points"], f"{label} WATER final source count mismatch")
    final_candidate_path = _resolve(final["candidate"]["path"], final_path.parent)
    final_candidate = _candidate_entry(
        final_candidate_path,
        final["candidate"]["sha256"],
        final["candidate"]["points"],
        source,
    )
    _require(
        base_candidate["points"] + int(final["addition_count"]) == final_candidate["points"],
        f"{label} WATER final count chain mismatch",
    )
    prefix = verify_exact_append_prefix(base_candidate_path, final_candidate_path)
    archive_path = _resolve(final["archive"], final_path.parent)
    _require(sha256_path(archive_path) == final["archive_sha256"], f"{label} WATER hole archive hash mismatch")
    _artifact(archive_path, artifacts)
    archive = _verify_npz_suffix_archive(
        final_candidate_path,
        base_count=int(base_candidate["points"]),
        archive_path=archive_path,
    )
    _require(
        int(final.get("sampling", {}).get("xy", {}).get("count", -1))
        == int(final["addition_count"]),
        f"{label} WATER sampling/addition counts disagree",
    )
    cross_scale = None
    if label == "5mm":
        _require(fine_geometry is not None, "5mm WATER requires verified 2mm geometry")
        cross_scale = _verify_coarse_fine_subset(
            final,
            coarse_manifest_path=final_path,
            coarse_archive_path=archive_path,
            fine_geometry=fine_geometry,
            support_paths=(base_candidate_path, *expected_terrain),
        )
    return {
        "canonical": canonical_name,
        "source": source,
        "candidate": final_candidate,
        "water_base_candidate": base_candidate,
        "survivor": survivor,
        "final_prefix": prefix,
        "base_manifest": str(base_path),
        "final_manifest": str(final_path),
        "config": base_config,
        "implementations": {
            "base": base_implementation,
            "final": final_implementation,
        },
        "parameters": parameters,
        "base_sources": {
            str(path): value for path, value in sorted(
                verified_sources.items(), key=lambda item: str(item[0])
            )
        },
        "reference_provenance": reference_provenance,
        "hole_archive": {
            **file_fingerprint(archive_path, ply=False),
            **archive,
        },
        "cross_scale": cross_scale,
        "holes": final.get("holes"),
        "review_bbox": final.get("review_bbox"),
    }


_WATER_SCALAR_VISIBILITY_FIELDS = frozenset(
    {
        "scalar_A_R_Shelter_Lower",
        "scalar_A_R_RainExposure_Lower",
        "scalar_A_R_SVF_Lower",
    }
)


def _water_scalar_geometry_fields(dtype: np.dtype) -> tuple[str, ...]:
    return tuple(
        name
        for name in dtype.names or ()
        if name.startswith("scalar_A_R_")
        and name not in _WATER_SCALAR_VISIBILITY_FIELDS
    )


def _component_field_scalar_coverage(
    records: np.ndarray,
    component_labels: np.ndarray,
    fields: Sequence[str],
    *,
    label: str,
) -> dict[str, object]:
    """Independently audit every published A_R field in every component."""

    labels = np.asarray(component_labels)
    _require(
        labels.ndim == 1
        and len(labels) == len(records)
        and np.issubdtype(labels.dtype, np.integer),
        f"{label} component labels are not aligned integer labels",
    )
    required_fields = tuple(dict.fromkeys(str(name) for name in fields))
    available = set(records.dtype.names or ())
    _require(bool(required_fields), f"{label} geometry-field list is empty")
    _require(
        set(required_fields).issubset(available),
        f"{label} geometry fields are absent from the candidate",
    )
    rows: list[dict[str, object]] = []
    all_complete = True
    for raw_label in np.unique(labels.astype(np.int64, copy=False)):
        component_label = int(raw_label)
        member = labels == raw_label
        count = int(np.count_nonzero(member))
        field_rows: dict[str, object] = {}
        component_complete = True
        for name in required_fields:
            values = np.asarray(records[name][member], np.float64)
            finite = np.isfinite(values)
            finite_count = int(np.count_nonzero(finite))
            fraction = float(finite_count / count) if count else 1.0
            finite_values = values[finite]
            range_contract = "finite-physical-values-no-global-clamp"
            range_lower = range_upper = None
            range_passed = True
            if name.endswith("_Combined"):
                range_lower = 0.0 if "_Roughness_" in name else -1.0
                range_upper = 1.0
                range_contract = "global-normalized-combined-range"
            elif name == "scalar_A_R_RoughnessRelative_FineMedium":
                range_lower, range_upper = 0.0, 8.0
                range_contract = "derived-relative-roughness-range"
            if range_lower is not None and finite_count:
                range_passed = bool(
                    np.all(finite_values >= range_lower - 1.0e-6)
                    and np.all(finite_values <= range_upper + 1.0e-6)
                )
            accepted = bool(fraction + 1.0e-15 >= 1.0 and range_passed)
            component_complete &= accepted
            field_rows[name] = {
                "finite": finite_count,
                "total": count,
                "fraction": fraction,
                "minimum": float(np.min(finite_values)) if finite_count else None,
                "maximum": float(np.max(finite_values)) if finite_count else None,
                "range_contract": range_contract,
                "range_lower": range_lower,
                "range_upper": range_upper,
                "range_passed": range_passed,
                "accepted": accepted,
            }
        all_complete &= component_complete
        rows.append(
            {
                "component_label": component_label,
                "points": count,
                "all_required_fields_accepted": component_complete,
                "fields": field_rows,
            }
        )
    return {
        "method": "exact-per-component-per-geometry-field-finiteness-and-range-v1",
        "minimum_required_finite_fraction": 1.0,
        "required_fields": list(required_fields),
        "component_count": int(len(rows)),
        "components": rows,
        "all_components_all_required_fields_accepted": bool(all_complete),
    }


def _parse_water_enrichment(
    *,
    label: str,
    manifest_path: Path,
    geometry: Mapping,
    terrain: Mapping[str, Mapping],
    fine_enrichment: Mapping | None,
    artifacts: dict[str, dict],
) -> dict:
    path, manifest = _load_json(manifest_path, f"{label} WATER scalar manifest")
    _artifact(path, artifacts)
    _require(
        manifest.get("operation")
        == "site1-v11-candidate-only-water-addition-scalar-enrichment",
        f"{label} WATER scalar operation is unexpected",
    )
    _require(manifest.get("status") == "built", f"{label} WATER scalar stage is not built")
    _require(manifest.get("candidate_only") is True, f"{label} WATER scalar stage is not candidate-only")
    _require(
        manifest.get("canonical_install_performed") is False,
        f"{label} WATER scalar stage reports canonical writes",
    )
    _require(manifest.get("resolution_label") == label, f"{label} WATER scalar resolution mismatch")
    nominal_spacing = _finite_number(
        manifest.get("nominal_spacing_m"), f"{label} WATER scalar nominal spacing"
    )
    _require(nominal_spacing > 0.0, f"{label} WATER scalar nominal spacing is not positive")

    contract = manifest.get("geometry_contract")
    _require(isinstance(contract, Mapping), f"{label} WATER geometry contract is missing")
    base = geometry["water_base_candidate"]
    geometry_candidate = geometry["candidate"]
    geometry_manifest_path = Path(geometry["final_manifest"])
    archive = geometry["hole_archive"]
    contract_expected = {
        "base_points": base["points"],
        "candidate_points": geometry_candidate["points"],
        "addition_count": archive["points"],
        "base_sha256": base["sha256"],
        "candidate_sha256": geometry_candidate["sha256"],
        "manifest_sha256": sha256_path(geometry_manifest_path),
        "archive_sha256": archive["sha256"],
        "base_payload_sha256": _payload_sha256(base["path"]),
        "candidate_prefix_payload_sha256": _payload_sha256(
            geometry_candidate["path"], count=int(base["points"])
        ),
        "archive_records_sha256": archive["records_sha256"],
        "candidate_suffix_sha256": archive["candidate_suffix_sha256"],
    }
    for key, expected in contract_expected.items():
        observed = contract.get(key)
        if isinstance(expected, int):
            try:
                observed = int(observed)
            except (TypeError, ValueError):
                pass
        _require(observed == expected, f"{label} WATER geometry contract differs: {key}")
    archived_geometry = manifest.get("geometry_manifest")
    _require(isinstance(archived_geometry, Mapping), f"{label} archived geometry manifest is missing")
    _require(
        _lstat_regular(
            str(archived_geometry.get("path", "")),
            f"{label} WATER geometry manifest provenance",
        )[0]
        == _lstat_regular(
            geometry_manifest_path,
            f"{label} WATER expected geometry manifest",
        )[0]
        and archived_geometry.get("sha256") == contract_expected["manifest_sha256"],
        f"{label} archived geometry manifest provenance mismatch",
    )
    _require(
        isinstance(archived_geometry.get("archived_copy"), str)
        and bool(archived_geometry["archived_copy"]),
        f"{label} archived geometry manifest filename is missing",
    )
    archived_geometry_path = _resolve(
        archived_geometry["archived_copy"], path.parent
    )
    _require(
        sha256_path(archived_geometry_path)
        == archived_geometry.get("archived_copy_sha256")
        == contract_expected["manifest_sha256"],
        f"{label} archived geometry manifest copy differs",
    )
    _artifact(archived_geometry_path, artifacts)
    archived_candidate = archived_geometry.get("candidate")
    _require(
        isinstance(archived_candidate, Mapping)
        and _same_path(archived_candidate.get("path", ""), geometry_candidate["path"])
        and archived_candidate.get("sha256") == geometry_candidate["sha256"]
        and int(archived_candidate.get("points", -1))
        == int(geometry_candidate["points"]),
        f"{label} archived geometry candidate provenance mismatch",
    )
    contract_config = contract.get("config_fingerprint")
    _require(isinstance(contract_config, Mapping), f"{label} WATER geometry contract lacks config")
    _require(
        contract_config.get("path") == geometry["config"]["path"]
        and contract_config.get("sha256") == geometry["config"]["sha256"],
        f"{label} WATER scalar stage used different geometry config",
    )

    inputs = manifest.get("input_fingerprints")
    required_inputs = {
        "base_water",
        "geometry_candidate",
        "geometry_manifest",
        "geometry_archive",
        "sand",
        "rock",
        "cleanmesh",
        "normalization_manifest",
    }
    if label == "5mm":
        _require(fine_enrichment is not None, "5mm scalar stage requires verified 2mm enrichment")
        required_inputs.update(
            {
                "fine_enriched_candidate",
                "fine_enriched_manifest",
                "fine_geometry_manifest",
                "fine_geometry_archive",
            }
        )
    else:
        _require(fine_enrichment is None, "2mm scalar stage cannot depend on a fine enrichment")
    _require(
        isinstance(inputs, Mapping) and set(inputs) == required_inputs,
        f"{label} WATER scalar input set is incomplete",
    )
    geometry_manifest_actual = file_fingerprint(
        geometry_manifest_path, ply=False
    )
    known_inputs: dict[str, tuple[Path, Mapping]] = {
        "base_water": (Path(base["path"]), base),
        "geometry_candidate": (
            Path(geometry_candidate["path"]), geometry_candidate
        ),
        "geometry_manifest": (geometry_manifest_path, geometry_manifest_actual),
        "geometry_archive": (Path(archive["path"]), archive),
    }
    terrain_resolution = "1mm" if label == "2mm" else "5mm"
    for role, input_name in (("SAND", "sand"), ("ROCK", "rock")):
        candidate = terrain[f"{role}-{terrain_resolution}"]["candidate"]
        known_inputs[input_name] = (Path(candidate["path"]), candidate)
    if fine_enrichment is not None:
        fine_manifest_path = Path(fine_enrichment["enrichment_manifest"])
        known_inputs["fine_enriched_candidate"] = (
            Path(fine_enrichment["candidate"]["path"]),
            fine_enrichment["candidate"],
        )
        known_inputs["fine_enriched_manifest"] = (
            fine_manifest_path,
            file_fingerprint(fine_manifest_path, ply=False),
        )
        fine_geometry_manifest_path = Path(fine_enrichment["final_manifest"])
        known_inputs["fine_geometry_manifest"] = (
            fine_geometry_manifest_path,
            file_fingerprint(fine_geometry_manifest_path, ply=False),
        )
        fine_geometry_archive = fine_enrichment["hole_archive"]
        known_inputs["fine_geometry_archive"] = (
            Path(fine_geometry_archive["path"]),
            fine_geometry_archive,
        )
    verified_inputs = {}
    for name, (expected_path, expected) in known_inputs.items():
        verified_inputs[name] = _verify_known_input_fingerprint(
            inputs[name],
            actual_path=expected_path,
            actual=expected,
            label=f"{label} WATER scalar input {name}",
        )
    for name in ("cleanmesh", "normalization_manifest"):
        verified_inputs[name] = _verify_input_fingerprint(
            inputs[name],
            parent=path.parent,
            label=f"{label} WATER scalar input {name}",
            artifacts=artifacts,
        )
    implementation = _verify_exact_implementation_hashes(
        manifest.get("scalar_enrichment_implementation", {}),
        family="water_scalar",
        label=f"{label} WATER scalar",
        artifacts=artifacts,
    )
    normalization = manifest.get("combined_geometry_normalization")
    _require(isinstance(normalization, Mapping), f"{label} WATER normalization audit is missing")
    _require(
        normalization.get("method") == "provided-manifest",
        f"{label} WATER scalar normalization method is unexpected",
    )
    _require(
        _same_path(normalization.get("path", ""), verified_inputs["normalization_manifest"]["path"])
        and normalization.get("sha256") == verified_inputs["normalization_manifest"]["sha256"],
        f"{label} WATER normalization input provenance mismatch",
    )
    _verify_normalization_values(
        normalization.get("values"), f"{label} WATER normalization"
    )

    local = manifest.get("local_analysis")
    _require(isinstance(local, Mapping), f"{label} WATER local scalar analysis is missing")
    _require(local.get("water_type_id") == 1, f"{label} WATER local analysis TypeID mismatch")
    _require(local.get("temporary_addition_scan_id") == 10.0, f"{label} WATER temporary ScanID mismatch")
    _require(local.get("full_cloud_analysis") is False, f"{label} WATER unexpectedly used full-cloud analysis")
    _require(
        _finite_number(local.get("collar_m"), f"{label} WATER local collar") > 0.0,
        f"{label} WATER local collar is not positive",
    )
    _require(isinstance(local.get("collars"), Mapping), f"{label} WATER collar audit is missing")
    _require(
        local.get("tagged_identity_verified_after_tiled_output") is True,
        f"{label} WATER tagged identity was not verified",
    )
    _require(
        int(local.get("tagged_addition_count", -1)) == int(archive["points"]),
        f"{label} WATER tagged addition count mismatch",
    )
    output_policy = local.get("output_policy")
    _require(isinstance(output_policy, Mapping), f"{label} WATER scalar output policy is missing")
    _require(
        output_policy.get("accepted_for_output") is (label == "2mm"),
        f"{label} WATER scalar output policy is inconsistent",
    )
    for name, hash_name in (
        ("input", "input_sha256"),
        ("input_manifest", "input_manifest_sha256"),
        ("analysed", "analysed_sha256"),
        ("cleanmesh_report", "cleanmesh_report_sha256"),
    ):
        artifact_path = _resolve(local[name], path.parent)
        _require(
            sha256_path(artifact_path) == local[hash_name],
            f"{label} WATER local analysis artifact hash mismatch: {name}",
        )
        _artifact(artifact_path, artifacts)

    scalar = manifest.get("scalar_enrichment")
    _require(isinstance(scalar, Mapping), f"{label} WATER scalar audit is missing")
    parameters = manifest.get("parameters")
    semantic = parameters.get("semantic") if isinstance(parameters, Mapping) else None
    _require(
        isinstance(semantic, Mapping)
        and _finite_number(
            semantic.get("minimum_component_field_finite_fraction"),
            f"{label} WATER component scalar threshold",
        )
        == 1.0,
        f"{label} WATER component scalar threshold is not fail-closed",
    )
    _require(
        _finite_number(
            scalar.get("minimum_component_field_finite_fraction"),
            f"{label} WATER scalar-audit component threshold",
        )
        == 1.0,
        f"{label} WATER scalar audit permits incomplete component coverage",
    )
    _require(
        scalar.get("non_geometry_fields_archive_exact") is True
        and scalar.get("intensity_and_composition_archive_exact") is True,
        f"{label} WATER scalar stage changed donor/non-geometric values",
    )
    _require(float(scalar.get("restored_scan_id", np.nan)) == 999.0, f"{label} WATER scalar audit ScanID mismatch")
    changed = scalar.get("changed_points_by_field")
    _require(
        isinstance(changed, Mapping) and any(int(value) > 0 for value in changed.values()),
        f"{label} WATER scalar stage changed no geometry-derived values",
    )
    with np.load(archive["path"], allow_pickle=False) as loaded:
        archive_files = set(loaded.files)
        archived_records = np.asarray(loaded["records"])
        archive_candidate_xy = (
            np.asarray(loaded["candidate_xy"])
            if "candidate_xy" in archive_files
            else None
        )
        candidate_label = (
            np.asarray(loaded["candidate_label"])
            if "candidate_label" in archive_files
            else None
        )
        fine_selection_index = (
            np.asarray(loaded["fine_selection_index"])
            if "fine_selection_index" in archive_files
            else None
        )
        coarse_component_label = (
            np.asarray(loaded["fine_component_label"])
            if "fine_component_label" in archive_files
            else None
        )
    active_component_labels: np.ndarray | None = None
    if label == "2mm":
        _require(
            archive_candidate_xy is not None and candidate_label is not None,
            "2mm WATER scalar archive lacks component-labelled geometry",
        )
        _record_xy_matches_archive(
            archived_records, archive_candidate_xy, "2mm WATER scalar archive"
        )
        fine_holes = _accepted_holes(manifest, "2mm WATER scalar")
        candidate_label = _geometry_component_labels(
            manifest,
            labels=candidate_label,
            accepted_holes=fine_holes,
            label="2mm WATER scalar geometry",
        )
        _verify_component_membership_audit(
            scalar.get("geometry_component_membership"),
            archive_path=archive["path"],
            required_arrays={"records", "candidate_xy", "candidate_label"},
            labels=candidate_label,
            accepted_labels={item[0] for item in fine_holes.values()},
            label="2mm WATER scalar geometry",
        )
        active_component_labels = candidate_label
    if label == "5mm":
        _require(
            scalar.get("method") == "exact-fine-selection-index-geometry-transfer",
            "5mm WATER geometry scalars are not tied to fine-selection indices",
        )
        _require(
            scalar.get("source_is_locally_cleanmesh_enriched_fine_suffix") is True,
            "5mm WATER scalar source is not the enriched fine suffix",
        )
        _require(
            fine_enrichment is not None
            and fine_selection_index is not None
            and coarse_component_label is not None,
            "5mm WATER scalar archive lacks fine-selection component provenance",
        )
        fine_manifest_path = Path(fine_enrichment["enrichment_manifest"])
        _, fine_manifest = _load_json(
            fine_manifest_path, "2mm WATER scalar manifest for coarse transfer"
        )
        fine_holes = _accepted_holes(fine_manifest, "2mm WATER scalar")
        fine_archive = fine_enrichment["hole_archive"]
        with np.load(fine_archive["path"], allow_pickle=False) as loaded:
            _require(
                {"records", "candidate_xy", "candidate_label"}.issubset(loaded.files),
                "2mm WATER scalar archive lacks component-labelled geometry",
            )
            fine_archive_records = np.asarray(loaded["records"])
            fine_archive_xy = np.asarray(loaded["candidate_xy"])
            fine_component_label = np.asarray(loaded["candidate_label"])
        _record_xy_matches_archive(
            fine_archive_records, fine_archive_xy, "2mm WATER scalar archive"
        )
        fine_component_label = _geometry_component_labels(
            fine_manifest,
            labels=fine_component_label,
            accepted_holes=fine_holes,
            label="2mm WATER scalar geometry",
        )
        coarse_component_label = np.asarray(coarse_component_label, np.int64)
        _require(
            np.array_equal(
                coarse_component_label,
                fine_component_label[fine_selection_index],
            ),
            "5mm WATER scalar component labels differ from the fine selection",
        )
        component_audit = scalar.get("component_membership")
        _verify_component_membership_audit(
            component_audit,
            archive_path=fine_archive["path"],
            required_arrays={"records", "candidate_xy", "candidate_label"},
            labels=fine_component_label,
            accepted_labels={item[0] for item in fine_holes.values()},
            label="5mm WATER scalar fine geometry",
        )
        assert isinstance(component_audit, Mapping)
        _require(
            component_audit.get("coarse_archive_key") == "fine_component_label"
            and component_audit.get("coarse_component_label_sha256")
            == _array_sha256(coarse_component_label)
            and component_audit.get("coarse_labels_match_selected_fine_labels") is True,
            "5mm WATER scalar coarse-component audit differs",
        )
        cross_manifest = scalar.get("cross_scale_manifest")
        _require(
            cross_manifest == manifest.get("cross_scale"),
            "5mm WATER scalar cross-scale manifest was not preserved exactly",
        )
        cross_verification = scalar.get("cross_scale_verification")
        _require(
            isinstance(cross_verification, Mapping),
            "5mm WATER scalar cross-scale verification is missing",
        )
        expected_cross_keys = {
            "method",
            "fine_manifest",
            "fine_archive",
            "fine_candidate_sha256",
            "fine_addition_count",
            "fine_selection_index_count",
            "fine_selection_index_unique",
            "coarse_xyz_exact_subset_of_fine_records_xyz",
            "coarse_normals_exact_subset_of_fine_records_normals",
            "nongeometry_fields_preserved_from_coarse_donors",
            "geometry_fields_copied_from_fine_records",
            "selection_seed",
            "spacing_m",
            "maximum_fine_to_coarse_or_terrain_support_distance_m",
            "accepted_hole_coverage",
        }
        _require(
            set(cross_verification.get("required_keys_verified", ()))
            == expected_cross_keys,
            "5mm WATER scalar cross-scale key audit is incomplete",
        )
        for name in (
            "fine_geometry_manifest_sha256_verified",
            "fine_geometry_archive_sha256_verified",
            "fine_geometry_candidate_sha256_verified",
            "fine_coarse_normalization_manifest_sha256_verified",
            "fine_addition_count_verified",
            "fine_selection_index_count_verified",
            "fine_selection_index_unique_verified",
            "coarse_xyz_subset_attestation_verified",
            "coarse_normals_subset_attestation_verified",
            "coarse_donor_nongeometry_attestation_verified",
            "coarse_component_labels_match_selected_fine_labels",
        ):
            _require(
                cross_verification.get(name) is True,
                f"5mm WATER scalar cross-scale verification failed: {name}",
            )
        geometry_cross = geometry["cross_scale"]
        _require(
            cross_verification.get("accepted_hole_coverage")
            == manifest["cross_scale"]["accepted_hole_coverage"]
            and int(cross_verification.get("accepted_hole_coverage_rows_verified", -1))
            == len(manifest["cross_scale"]["accepted_hole_coverage"]),
            "5mm WATER scalar accepted-hole coverage audit differs",
        )
        _require(
            np.isclose(
                _finite_number(
                    cross_verification.get("maximum_support_distance_m"),
                    "5mm WATER scalar maximum support distance",
                ),
                float(geometry_cross["maximum_support_distance_m"]),
                rtol=0.0,
                atol=1.0e-9,
            ),
            "5mm WATER scalar maximum support distance differs from geometry",
        )
        _require(
            scalar.get("xyz_and_normals_byte_exact") is True,
            "5mm WATER scalar XYZ/normal exact-transfer invariant is missing",
        )
        active_component_labels = coarse_component_label
    invariants = manifest.get("invariants", {})
    required_invariants = {
        "geometry_candidate_verified_as_base_plus_archive": True,
        "existing_base_payload_byte_exact": True,
        "coordinates_and_normals_archive_exact": True,
        "colour_intensity_composition_archive_exact": True,
        "visibility_fields_archive_exact": True,
        "geometry_metrics_from_local_cleanmesh": True,
        "combined_metrics_use_v10_global_normalization": True,
        "undefined_geometry_fallback_component_strict": True,
        "undefined_geometry_fallback_no_extrapolation": True,
        "geometry_component_membership_verified": True,
        "component_field_scalar_coverage_complete": True,
        "component_field_scalar_ranges_verified": True,
        "final_addition_scan_id": 999.0,
        "canonical_writes": False,
    }
    for name, expected in required_invariants.items():
        _require(invariants.get(name) == expected, f"{label} WATER scalar invariant failed: {name}")
    _require(
        invariants.get("coarse_geometry_metrics_from_exact_fine_selection")
        is (label == "5mm"),
        f"{label} WATER coarse-transfer invariant is inconsistent",
    )
    _require(
        invariants.get("coarse_local_cleanmesh_is_diagnostic_only")
        is (label == "5mm"),
        f"{label} WATER diagnostic-only invariant is inconsistent",
    )

    candidate_path = _resolve(manifest["candidate"]["path"], path.parent)
    candidate = _candidate_entry(
        candidate_path,
        manifest["candidate"]["sha256"],
        manifest["candidate"]["points"],
        geometry["source"],
    )
    _require(
        int(candidate["points"]) == int(contract["candidate_points"]),
        f"{label} WATER enriched point count differs from geometry",
    )
    prefix = verify_exact_append_prefix(base["path"], candidate_path)
    prefix_hash = _payload_sha256(candidate_path, count=int(base["points"]))
    suffix_hash = _payload_sha256(
        candidate_path,
        start=int(base["points"]),
        count=int(archive["points"]),
    )
    _require(
        manifest["candidate"].get("base_payload_sha256") == prefix_hash,
        f"{label} WATER enriched prefix payload hash mismatch",
    )
    _require(
        manifest["candidate"].get("suffix_sha256") == suffix_hash,
        f"{label} WATER enriched suffix hash mismatch",
    )
    scan = _scan_appended_scan_id(candidate_path, int(base["points"]), 999.0)
    candidate_info = inspect_ply(candidate_path)
    candidate_memory = np.memmap(
        candidate_info.path,
        dtype=candidate_info.dtype,
        mode="r",
        offset=candidate_info.offset,
        shape=(candidate_info.count,),
    )
    enriched_records = np.asarray(candidate_memory[int(base["points"]):]).copy()
    del candidate_memory
    geometry_fields = tuple(str(name) for name in scalar.get("geometry_fields_replaced", ()))
    required_geometry_fields = _water_scalar_geometry_fields(candidate_info.dtype)
    _require(
        bool(required_geometry_fields)
        and set(geometry_fields) == set(required_geometry_fields),
        f"{label} WATER scalar geometry-field list is incomplete",
    )
    _require(
        set(geometry_fields) == set(changed),
        f"{label} WATER scalar changed-field audit is inconsistent",
    )
    for field in geometry_fields:
        _require(field in (candidate_info.dtype.names or ()), f"{label} WATER scalar field is absent: {field}")
    for field in candidate_info.dtype.names or ():
        if field in geometry_fields or field == scan["field"]:
            continue
        _require(
            np.asarray(enriched_records[field]).tobytes()
            == np.asarray(archived_records[field]).tobytes(),
            f"{label} WATER enrichment changed archived field {field}",
        )
    _require(
        active_component_labels is not None,
        f"{label} WATER component labels were not verified",
    )
    direct_component_coverage = _component_field_scalar_coverage(
        enriched_records,
        active_component_labels,
        required_geometry_fields,
        label=f"{label} WATER addition suffix",
    )
    _require(
        direct_component_coverage[
            "all_components_all_required_fields_accepted"
        ]
        is True,
        f"{label} WATER addition suffix has incomplete or out-of-range component scalars",
    )
    _require(
        scalar.get("component_field_finite_coverage")
        == direct_component_coverage,
        f"{label} WATER component scalar audit differs from the candidate",
    )
    if label == "5mm":
        assert fine_enrichment is not None and fine_selection_index is not None
        fine_candidate = fine_enrichment["candidate"]
        fine_info = inspect_ply(fine_candidate["path"])
        fine_base_count = int(fine_enrichment["water_base_candidate"]["points"])
        _require(
            len(fine_selection_index) == len(enriched_records),
            "5mm scalar fine-selection count mismatch",
        )
        fine_memory = np.memmap(
            fine_info.path,
            dtype=fine_info.dtype,
            mode="r",
            offset=fine_info.offset,
            shape=(fine_info.count,),
        )
        selected_fine = np.asarray(
            fine_memory[fine_base_count + fine_selection_index]
        ).copy()
        del fine_memory
        for field in ("x", "y", "z", "nx", "ny", "nz", *geometry_fields):
            _require(
                np.asarray(enriched_records[field]).tobytes()
                == np.asarray(selected_fine[field]).tobytes(),
                f"5mm enriched field is not the selected 2mm field: {field}",
            )
        declared_fine = scalar.get("fine_candidate", {})
        _require(
            declared_fine.get("sha256") == fine_candidate["sha256"]
            and int(declared_fine.get("points", -1))
            == int(fine_candidate["points"])
            and int(declared_fine.get("base_points", -1)) == fine_base_count,
            "5mm scalar fine candidate provenance mismatch",
        )
        declared_manifest = scalar.get("fine_manifest", {})
        fine_manifest_path = Path(fine_enrichment["enrichment_manifest"])
        _require(
            _lstat_regular(
                str(declared_manifest.get("path", "")),
                "5mm scalar declared fine manifest",
            )[0]
            == _lstat_regular(
                fine_manifest_path, "5mm scalar expected fine manifest"
            )[0]
            and declared_manifest.get("sha256") == sha256_path(fine_manifest_path),
            "5mm scalar fine manifest provenance mismatch",
        )
        selection_audit = scalar.get("fine_selection_index", {})
        selection_hash = hashlib.sha256(
            np.ascontiguousarray(fine_selection_index).tobytes()
        ).hexdigest()
        _require(
            selection_audit.get("archive_key") == "fine_selection_index"
            and selection_audit.get("unique") is True
            and int(selection_audit.get("count", -1)) == len(fine_selection_index)
            and selection_audit.get("sha256") == selection_hash,
            "5mm scalar fine-selection provenance mismatch",
        )
    result = dict(geometry)
    result.update(
        {
            "geometry_candidate": geometry_candidate,
            "candidate": candidate,
            "final_prefix": prefix,
            "enrichment_manifest": str(path),
            "enrichment_inputs": verified_inputs,
            "enrichment_implementation": implementation,
            "enriched_suffix_sha256": suffix_hash,
            "addition_scan_id": scan,
        }
    )
    return result


def _scan_field_name(dtype: np.dtype) -> str:
    names = tuple(dtype.names or ())
    exact = {name.lower(): name for name in names}
    for alias in ("scalar_scanid", "scanid", "scan_id"):
        if alias in exact:
            return exact[alias]
    normalized = {
        "".join(character for character in name.lower() if character.isalnum()): name
        for name in names
    }
    for alias in ("scalarscanid", "scanid"):
        if alias in normalized:
            return normalized[alias]
    raise RuntimeError("terrain record schema has no ScanID field")


def _xyz_records(records: np.ndarray) -> np.ndarray:
    _require(
        records.ndim == 1
        and records.dtype.names is not None
        and {"x", "y", "z"}.issubset(records.dtype.names),
        "terrain record array has no structured XYZ schema",
    )
    return np.column_stack(
        tuple(np.asarray(records[name], np.float64) for name in ("x", "y", "z"))
    )


def _load_exact_npz(
    path: Path,
    expected_keys: set[str],
    *,
    label: str,
    artifacts: dict[str, dict],
) -> dict[str, np.ndarray]:
    archive, _ = _lstat_regular(path, label)
    _artifact(archive, artifacts)
    with np.load(archive, allow_pickle=False) as loaded:
        _require_exact_keys(
            {name: True for name in loaded.files}, expected_keys, f"{label} arrays"
        )
        arrays = {name: np.asarray(loaded[name]).copy() for name in loaded.files}
    _require(
        all(not value.dtype.hasobject for value in arrays.values()),
        f"{label} contains an object array",
    )
    return arrays


def _relative_artifact(
    parent: Path,
    value: object,
    expected_hash: object,
    *,
    label: str,
    artifacts: dict[str, dict],
) -> Path:
    _require(isinstance(value, str) and bool(value), f"{label} path is missing")
    relative = Path(value)
    _require(not relative.is_absolute(), f"{label} path must be relative")
    _require(".." not in relative.parts, f"{label} path escapes its stage")
    path, _ = _lstat_regular(parent / relative, label)
    _require(
        path.is_relative_to(parent.resolve()), f"{label} path escapes its stage"
    )
    _require(
        isinstance(expected_hash, str) and sha256_path(path) == expected_hash,
        f"{label} hash mismatch",
    )
    _artifact(path, artifacts)
    return path


def _verify_suffix_records(
    candidate_path: Path,
    *,
    source_count: int,
    records: np.ndarray,
    label: str,
) -> str:
    info = inspect_ply(candidate_path)
    _require(records.ndim == 1 and records.dtype == info.dtype, f"{label} archive schema mismatch")
    _require(
        info.count == int(source_count) + len(records),
        f"{label} archive/candidate count mismatch",
    )
    candidate = np.memmap(
        info.path,
        dtype=info.dtype,
        mode="r",
        offset=info.offset,
        shape=(info.count,),
    )
    try:
        suffix = np.asarray(candidate[int(source_count) :])
        _require(
            suffix.tobytes(order="C") == records.tobytes(order="C"),
            f"{label} authoritative archive differs from candidate suffix",
        )
    finally:
        del candidate
    return hashlib.sha256(records.tobytes(order="C")).hexdigest()


def _verify_terrain_cleanmesh_artifacts(
    value: object,
    *,
    report_parent: Path,
    addition_count: int,
    cleanmesh: Mapping,
    coarse: bool,
    artifacts: dict[str, dict],
) -> None:
    _require(isinstance(value, Mapping), "terrain CleanMesh resolution audit is missing")
    status = value.get("status")
    _require(value.get("full_cloud_analysis") is False, "terrain CleanMesh used a full cloud")
    if addition_count == 0:
        expected = (
            "skipped-no-cross-scale-additions" if coarse
            else "skipped-no-accepted-additions"
        )
        _require(status == expected, "terrain empty CleanMesh audit status is wrong")
        if coarse:
            _require(
                value.get("exact_fine_xyz_preserved") is True,
                "terrain coarse empty XYZ-preservation audit is missing",
            )
        return
    _require(status == "completed", "terrain non-empty CleanMesh audit is incomplete")
    _require(value.get("span_identity_verified") is True, "terrain CleanMesh span identity is unverified")
    if coarse:
        _require(
            value.get("exact_fine_xyz_preserved") is True,
            "terrain coarse CleanMesh changed authoritative XYZ",
        )
    resolved: dict[str, Path] = {}
    for key, hash_key in (
        ("local_input", "local_input_sha256"),
        ("local_manifest", "local_manifest_sha256"),
        ("analysed", "analysed_sha256"),
        ("report", "report_sha256"),
    ):
        resolved[key] = _relative_artifact(
            report_parent,
            value.get(key),
            value.get(hash_key),
            label=f"terrain CleanMesh {key}",
            artifacts=artifacts,
        )
    _require(
        int(value.get("addition_points", -1)) == addition_count,
        "terrain CleanMesh addition count mismatch",
    )
    runner = value.get("runner")
    _require(isinstance(runner, Mapping), "terrain CleanMesh runner audit is missing")
    command = runner.get("command")
    _require(
        isinstance(command, list)
        and bool(command)
        and _same_path(command[0], cleanmesh["path"]),
        "terrain CleanMesh command does not identify the verified executable",
    )
    _require(int(runner.get("returncode", -1)) == 0, "terrain CleanMesh runner failed")
    _require(
        runner.get("output_sha256") == value.get("analysed_sha256")
        and runner.get("report_sha256") == value.get("report_sha256"),
        "terrain CleanMesh runner hashes disagree with its artifacts",
    )


def _verify_fine_terrain_archive(
    *,
    role: str,
    report: Mapping,
    report_path: Path,
    resolution_block: Mapping,
    candidate_path: Path,
    source_count: int,
    addition_count: int,
    artifacts: dict[str, dict],
) -> dict[str, object]:
    archive_paths = _require_exact_keys(
        resolution_block.get("addition_archive_paths"),
        {"SAND", "ROCK"},
        "terrain fine archive paths",
    )
    archive_hashes = _require_exact_keys(
        resolution_block.get("addition_archive_sha256"),
        {"SAND", "ROCK"},
        "terrain fine archive hashes",
    )
    archive_path = _resolve(archive_paths[role], report_path.parent)
    _require(
        sha256_path(archive_path) == archive_hashes[role],
        f"terrain fine {role} archive hash mismatch",
    )
    arrays = _load_exact_npz(
        archive_path,
        TERRAIN_FINE_ARCHIVE_KEYS,
        label=f"terrain fine {role} archive",
        artifacts=artifacts,
    )
    records = arrays["records"]
    for name, values in arrays.items():
        _require(
            values.ndim == 1 and len(values) == addition_count,
            f"terrain fine {role} archive field {name} is misaligned",
        )
    _require(
        np.array_equal(arrays["fine_index"], np.arange(addition_count)),
        f"terrain fine {role} archive index is not canonical",
    )
    _require(
        not addition_count or np.all(np.isfinite(_xyz_records(records))),
        f"terrain fine {role} archive contains non-finite XYZ",
    )
    scan_field = _scan_field_name(records.dtype)
    _require(
        not addition_count
        or np.all(np.rint(records[scan_field]) == 10.0),
        f"terrain fine {role} archive contains non-ScanID10 records",
    )
    _require(
        not addition_count
        or (
            np.all(arrays["confidence_reason_mask"] == 0)
            and np.all(arrays["confidence_tier"] >= 2)
        ),
        f"terrain fine {role} archive contains rejected confidence",
    )
    _require(
        len(np.unique(arrays["global_ledger_index"])) == addition_count,
        f"terrain fine {role} archive repeats global ledger indices",
    )
    payload_hash = _verify_suffix_records(
        candidate_path,
        source_count=source_count,
        records=records,
        label=f"terrain fine {role}",
    )
    archives = _require_exact_keys(
        report.get("authoritative_addition_archives"),
        {"SAND", "ROCK"},
        "terrain fine authoritative archives",
    )
    audit = archives[role]
    _require(isinstance(audit, Mapping), f"terrain fine {role} archive audit is missing")
    _require(
        Path(str(audit.get("path", ""))).name == archive_path.name
        and audit.get("sha256") == archive_hashes[role]
        and set(audit.get("keys", ())) == TERRAIN_FINE_ARCHIVE_KEYS
        and int(audit.get("points", -1)) == addition_count
        and int(audit.get("record_stride", -1)) == records.dtype.itemsize
        and audit.get("record_payload_sha256") == payload_hash
        and audit.get("candidate_suffix_byte_exact") is True
        and audit.get("candidate_sha256") == resolution_block["candidate_sha256"][role],
        f"terrain fine {role} archive audit is inconsistent",
    )

    arbitration = report.get("global_arbitration", {}).get("roles", {}).get(role)
    _require(isinstance(arbitration, Mapping), f"terrain fine {role} arbitration audit is missing")
    if addition_count:
        ledger_path = _relative_artifact(
            report_path.parent,
            arbitration.get("ledger_npz"),
            arbitration.get("ledger_npz_sha256"),
            label=f"terrain fine {role} global ledger",
            artifacts=artifacts,
        )
        _relative_artifact(
            report_path.parent,
            arbitration.get("ledger_json"),
            arbitration.get("ledger_json_sha256"),
            label=f"terrain fine {role} global ledger report",
            artifacts=artifacts,
        )
        _require(
            audit.get("global_arbitration_ledger") == ledger_path.name
            and audit.get("global_arbitration_ledger_sha256")
            == arbitration.get("ledger_npz_sha256"),
            f"terrain fine {role} archive/global-ledger provenance differs",
        )
        ledger = _load_exact_npz(
            ledger_path,
            TERRAIN_GLOBAL_LEDGER_KEYS,
            label=f"terrain fine {role} global ledger",
            artifacts=artifacts,
        )
        indices = np.asarray(arrays["global_ledger_index"], np.int64)
        ledger_count = len(ledger["globally_selected"])
        _require(
            np.all(indices >= 0) and np.all(indices < ledger_count),
            f"terrain fine {role} global ledger indices are invalid",
        )
        _require(
            all(values.ndim >= 1 and len(values) == ledger_count for values in ledger.values()),
            f"terrain fine {role} global ledger arrays are misaligned",
        )
        _require(
            np.all(ledger["globally_selected"][indices])
            and np.array_equal(ledger["xyz"][indices], _xyz_records(records))
            and np.array_equal(
                ledger["target_id"][indices].astype(str), arrays["target_id"].astype(str)
            )
            and np.array_equal(
                ledger["target_candidate_index"][indices], arrays["target_candidate_index"]
            )
            and np.array_equal(ledger["radius_m"][indices], arrays["radius_m"])
            and np.array_equal(ledger["local_priority"][indices], arrays["priority"]),
            f"terrain fine {role} global ledger geometry/selection differs",
        )
        for ledger_name, archive_name in (
            ("confidence_reason_mask", "confidence_reason_mask"),
            ("confidence_tier", "confidence_tier"),
            ("confidence_surface_spread_m", "confidence_surface_spread_m"),
            ("confidence_preferred_gate_count", "confidence_preferred_gate_count"),
            ("target_donor_index", "target_donor_index"),
            ("target_donor_distance_m", "target_donor_distance_m"),
            ("target_donor_count", "target_donor_count"),
        ):
            _require(
                np.array_equal(ledger[ledger_name][indices], arrays[archive_name]),
                f"terrain fine {role} global ledger {archive_name} differs",
            )

        target_provenance = audit.get("target_provenance")
        _require(isinstance(target_provenance, Mapping), f"terrain fine {role} target provenance is missing")
        expected_targets = {str(value) for value in np.unique(arrays["target_id"])}
        _require(
            set(map(str, target_provenance)) == expected_targets,
            f"terrain fine {role} target provenance set differs",
        )
        for target_id in sorted(expected_targets):
            entry = target_provenance[target_id]
            _require(isinstance(entry, Mapping), f"terrain target {target_id} provenance is invalid")
            provenance_npz = _relative_artifact(
                report_path.parent,
                entry.get("npz"),
                entry.get("npz_sha256"),
                label=f"terrain target {target_id} provenance",
                artifacts=artifacts,
            )
            _relative_artifact(
                report_path.parent,
                entry.get("json"),
                entry.get("json_sha256"),
                label=f"terrain target {target_id} provenance report",
                artifacts=artifacts,
            )
            provenance = _load_exact_npz(
                provenance_npz,
                TERRAIN_TARGET_PROVENANCE_KEYS,
                label=f"terrain target {target_id} provenance",
                artifacts=artifacts,
            )
            rows = np.flatnonzero(arrays["target_id"].astype(str) == target_id)
            candidate_index = np.asarray(arrays["target_candidate_index"][rows], np.int64)
            provenance_count = len(provenance["xyz"])
            _require(
                all(values.ndim >= 1 and len(values) == provenance_count for values in provenance.values())
                and np.all(candidate_index >= 0)
                and np.all(candidate_index < provenance_count),
                f"terrain target {target_id} provenance arrays/indices are invalid",
            )
            provenance_xyz = np.asarray(provenance["xyz"][candidate_index], np.float64).copy()
            for axis, field in enumerate(("x", "y", "z")):
                provenance_xyz[:, axis] = provenance_xyz[:, axis].astype(records.dtype[field])
            _require(
                np.array_equal(provenance_xyz, _xyz_records(records)[rows])
                and np.all(provenance["disposition"][candidate_index] == 1)
                and np.all(provenance["decision_reason_mask"][candidate_index] == 0)
                and np.array_equal(
                    provenance["confidence_reason_mask"][candidate_index],
                    arrays["confidence_reason_mask"][rows],
                )
                and np.array_equal(
                    provenance["confidence_tier"][candidate_index], arrays["confidence_tier"][rows]
                )
                and np.array_equal(
                    provenance["surface_spread_m"][candidate_index],
                    arrays["confidence_surface_spread_m"][rows],
                )
                and np.array_equal(
                    provenance["donor_index"][candidate_index], arrays["target_donor_index"][rows]
                )
                and np.array_equal(
                    provenance["donor_distance_m"][candidate_index],
                    arrays["target_donor_distance_m"][rows],
                ),
                f"terrain target {target_id} provenance differs from accepted records",
            )
    else:
        _require(
            not audit.get("target_provenance"),
            f"terrain fine {role} empty archive has target provenance",
        )
    return {"path": str(archive_path), "sha256": archive_hashes[role], "arrays": arrays}


def _terrain_targets(value: object) -> tuple[dict[str, object], ...]:
    _require(isinstance(value, list) and bool(value), "terrain target list is missing")
    result: list[dict[str, object]] = []
    seen: set[str] = set()
    for row in value:
        _require(isinstance(row, Mapping), "terrain target row is invalid")
        target_id = str(row.get("id", ""))
        _require(target_id and target_id not in seen, "terrain target IDs are empty or duplicated")
        seen.add(target_id)
        bbox = row.get("bbox")
        _require(
            isinstance(bbox, list)
            and len(bbox) == 4
            and all(np.isfinite(float(item)) for item in bbox),
            f"terrain target {target_id} bbox is invalid",
        )
        bounds = tuple(float(item) for item in bbox)
        _require(
            bounds[0] <= bounds[1] and bounds[2] <= bounds[3],
            f"terrain target {target_id} bbox is inverted",
        )
        _require(
            row.get("kind") in {"marked", "scanner", "crack"},
            f"terrain target {target_id} kind is invalid",
        )
        _require(
            row.get("minimum_tier") in {"SUPPORTED", "STRONG"},
            f"terrain target {target_id} minimum tier is invalid",
        )
        result.append(
            {
                "id": target_id,
                "kind": row["kind"],
                "bbox": bounds,
                "minimum_tier": row["minimum_tier"],
            }
        )
    return tuple(result)


def _collect_local_measured_xyz(
    path: Path,
    *,
    targets: Sequence[Mapping[str, object]],
    collar_m: float,
) -> np.ndarray:
    """Recreate the producer's source-order local measured terrain vector."""

    info = inspect_ply(path)
    scan_field = _scan_field_name(info.dtype)
    bounds = [
        (
            float(target["bbox"][0]) - collar_m,
            float(target["bbox"][1]) + collar_m,
            float(target["bbox"][2]) - collar_m,
            float(target["bbox"][3]) + collar_m,
        )
        for target in targets
    ]
    pieces: list[np.ndarray] = []
    records = np.memmap(
        info.path,
        dtype=info.dtype,
        mode="r",
        offset=info.offset,
        shape=(info.count,),
    )
    try:
        for begin in range(0, info.count, CHUNK_RECORDS):
            part = records[begin : begin + CHUNK_RECORDS]
            x = np.asarray(part["x"], np.float64)
            y = np.asarray(part["y"], np.float64)
            z = np.asarray(part["z"], np.float64)
            scan = np.asarray(part[scan_field], np.float64)
            rounded = np.rint(scan)
            keep = (
                np.isfinite(x)
                & np.isfinite(y)
                & np.isfinite(z)
                & np.isfinite(scan)
                & (np.abs(scan - rounded) <= 1.0e-5)
                & (rounded >= 0.0)
                & (rounded <= 8.0)
            )
            within = np.zeros(len(part), dtype=bool)
            for xmin, xmax, ymin, ymax in bounds:
                within |= (
                    (x >= xmin) & (x <= xmax) & (y >= ymin) & (y <= ymax)
                )
            keep &= within
            if np.any(keep):
                pieces.append(np.column_stack((x[keep], y[keep], z[keep])))
    finally:
        del records
    return np.concatenate(pieces) if pieces else np.empty((0, 3), np.float64)


def _nearest_surface_support(
    query_xyz: np.ndarray,
    support_xyz: np.ndarray,
    *,
    maximum_xy_m: float,
    maximum_vertical_m: float,
    tolerance_m: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    query = np.asarray(query_xyz, np.float64)
    support = np.asarray(support_xyz, np.float64)
    _require(query.ndim == 2 and query.shape[1] == 3, "terrain support query is invalid")
    _require(support.ndim == 2 and support.shape[1] == 3, "terrain support cloud is invalid")
    _require(np.all(np.isfinite(query)) and np.all(np.isfinite(support)), "terrain support geometry is non-finite")
    distances = np.full(len(query), np.inf, np.float64)
    indices = np.full(len(query), -1, np.int64)
    verticals = np.full(len(query), np.inf, np.float64)
    if not len(query) or not len(support):
        return distances, indices, verticals
    radius = maximum_xy_m + tolerance_m
    try:
        from scipy.spatial import cKDTree
    except ModuleNotFoundError:
        # Keep the release verifier usable with the system Python.  Quantise
        # support to radius-sized XY cells and retain only the 3x3 cells around
        # a query before the exact distance/Z test.  This is bounded by local
        # surface density, rather than by the full multi-million-point collar.
        cell_size = max(radius, np.finfo(np.float64).eps)
        query_cell = np.floor(query[:, :2] / cell_size).astype(np.int64)
        offsets = np.asarray(
            [(dx, dy) for dx in (-1, 0, 1) for dy in (-1, 0, 1)], np.int64
        )
        relevant_cell = (
            query_cell[:, None, :] + offsets[None, :, :]
        ).reshape(-1, 2)
        cell_dtype = np.dtype([("x", "<i8"), ("y", "<i8")])
        relevant_key = np.empty(len(relevant_cell), dtype=cell_dtype)
        relevant_key["x"], relevant_key["y"] = (
            relevant_cell[:, 0], relevant_cell[:, 1]
        )
        relevant_key = np.unique(relevant_key)
        support_cell = np.floor(support[:, :2] / cell_size).astype(np.int64)
        support_key = np.empty(len(support_cell), dtype=cell_dtype)
        support_key["x"], support_key["y"] = support_cell[:, 0], support_cell[:, 1]
        support = support[np.isin(support_key, relevant_key)]
        neighbourhoods = [
            np.flatnonzero(
                np.sum(np.square(support[:, :2] - point[:2]), axis=1)
                <= radius * radius
            )
            for point in query
        ]
    else:
        neighbourhoods = cKDTree(support[:, :2]).query_ball_point(
            query[:, :2], radius, workers=-1
        )
    for row, raw in enumerate(neighbourhoods):
        nearby = np.asarray(raw, np.int64)
        if not len(nearby):
            continue
        dz = np.abs(support[nearby, 2] - query[row, 2])
        nearby = nearby[dz <= maximum_vertical_m + tolerance_m]
        if not len(nearby):
            continue
        dz = np.abs(support[nearby, 2] - query[row, 2])
        xy = np.linalg.norm(support[nearby, :2] - query[row, :2], axis=1)
        within = xy <= radius
        nearby, dz, xy = nearby[within], dz[within], xy[within]
        if not len(nearby):
            continue
        order = np.lexsort((nearby, dz, xy))
        chosen = int(order[0])
        indices[row] = int(nearby[chosen])
        distances[row] = float(xy[chosen])
        verticals[row] = float(dz[chosen])
    return distances, indices, verticals


def _minimum_compatible_pair_distance(
    xyz: np.ndarray,
    *,
    spacing_m: float,
    vertical_m: float,
    tolerance_m: float,
) -> float | None:
    points = np.asarray(xyz, np.float64)
    if len(points) < 2:
        return None
    _require(spacing_m > 0.0, "terrain cross-scale spacing must be positive")
    vertical_limit = vertical_m + tolerance_m
    minimum = np.inf
    # This deliberately searches every vertically compatible pair, including
    # pairs farther apart than the requested spacing.  Restricting a tree
    # query to ``spacing + tolerance`` makes an otherwise valid reported
    # minimum disappear when it is (correctly) just above the threshold.  The
    # row-wise calculation is exact, deterministic, and bounded in memory;
    # the coarse addition cloud is the only input to this audit.
    for row in range(len(points) - 1):
        delta = points[row + 1 :] - points[row]
        compatible = np.abs(delta[:, 2]) <= vertical_limit
        if not np.any(compatible):
            continue
        xy = np.hypot(delta[compatible, 0], delta[compatible, 1])
        if len(xy):
            minimum = min(minimum, float(np.min(xy)))
    return None if not np.isfinite(minimum) else minimum


def _verify_terrain_cross_scale(
    *,
    manifest: Mapping,
    manifest_path: Path,
    parameters: Mapping,
    targets: Sequence[Mapping[str, object]],
    fine_block: Mapping,
    fine_report_path: Path,
    fine_archives: Mapping[str, Mapping[str, object]],
    coarse_block: Mapping,
    coarse_report: Mapping,
    coarse_report_path: Path,
    source_paths: Mapping[str, Path],
    candidate_paths: Mapping[str, Path],
    source_counts: Mapping[str, int],
    artifacts: dict[str, dict],
) -> dict[str, object]:
    cross_path = _resolve(coarse_block.get("cross_scale_report_path"), manifest_path.parent)
    _require(
        coarse_block.get("cross_scale_report_sha256") == sha256_path(cross_path),
        "terrain cross-scale report hash mismatch",
    )
    _artifact(cross_path, artifacts)
    _, cross = _load_json(cross_path, "terrain cross-scale report")
    cross_link = coarse_report.get("cross_scale")
    _require(isinstance(cross_link, Mapping), "terrain coarse report lacks cross-scale link")
    linked = _relative_artifact(
        coarse_report_path.parent,
        cross_link.get("report"),
        cross_link.get("report_sha256"),
        label="terrain coarse cross-scale report",
        artifacts=artifacts,
    )
    _require(linked == cross_path, "terrain coarse report links a different cross-scale report")
    top_cross = manifest.get("cross_scale")
    _require(isinstance(top_cross, Mapping), "terrain top-level cross-scale audit is missing")
    top_report = top_cross.get("report")
    _require(isinstance(top_report, Mapping), "terrain top-level cross-scale report fingerprint is missing")
    _require(
        _same_path(top_report.get("path", ""), cross_path)
        and top_report.get("sha256") == coarse_block.get("cross_scale_report_sha256"),
        "terrain top-level cross-scale report fingerprint differs",
    )
    inline = dict(top_cross)
    inline.pop("report", None)
    _require(inline == cross, "terrain inline cross-scale audit differs from its report")
    _require(cross.get("schema_version") == 1, "terrain cross-scale schema is unexpected")
    _require(
        cross.get("method") == "deterministic-maximal-surface-aware-fine-xyz-subset-v1",
        "terrain cross-scale method is unexpected",
    )
    _require(
        _same_path(
            _resolve(cross.get("fine_resolution_report"), cross_path.parent),
            fine_report_path,
        )
        and cross.get("fine_resolution_report_sha256") == fine_block.get("report_sha256"),
        "terrain cross-scale fine authority report differs",
    )
    spacing = _finite_number(cross.get("coarse_spacing_m"), "terrain coarse spacing")
    vertical = _finite_number(cross.get("vertical_support_tolerance_m"), "terrain vertical support tolerance")
    tolerance = _finite_number(cross.get("distance_tolerance_m"), "terrain distance tolerance")
    _require(
        spacing == float(parameters["coarse"]["nominal_spacing_m"])
        and float(cross.get("terrain_clearance_m")) == spacing
        and vertical == float(parameters["cross_scale_vertical_tolerance_m"])
        and tolerance == float(parameters["cross_scale_distance_tolerance_m"]),
        "terrain cross-scale parameters differ from the producer parameters",
    )
    _require(
        cross.get("selection_dimensions")
        == "XY with explicit absolute-Z compatibility guard",
        "terrain cross-scale selection dimensions are unexpected",
    )
    expected_invariants = {
        "fine_final_additions_are_authoritative": True,
        "coarse_addition_xyz_is_exact_fine_subset": True,
        "coarse_independent_geometry_proposals": 0,
        "maximal_coverage_at_coarse_spacing": True,
        "floating_returns_do_not_count_as_surface_support": True,
        "same_role_scanid_0_to_8_property_donors_only": True,
        "cleanmesh_recomputed_coarse_scalars_without_xyz_changes": True,
        "candidate_only": True,
        "canonical_writes": False,
    }
    _require(
        all(cross.get("invariants", {}).get(key) == expected for key, expected in expected_invariants.items()),
        "terrain cross-scale invariants are incomplete",
    )
    roles = _require_exact_keys(cross.get("roles"), {"SAND", "ROCK"}, "terrain cross-scale roles")
    result: dict[str, object] = {"report": str(cross_path), "roles": {}}
    collar = max(
        float(parameters["coarse"]["source_collar_m"]),
        float(parameters["coarse"]["cleanmesh_collar_m"]),
    )
    for role in ("SAND", "ROCK"):
        label = f"terrain cross-scale {role}"
        role_audit = roles[role]
        _require(isinstance(role_audit, Mapping), f"{label} audit is missing")
        fine_archive = fine_archives[role]
        fine_arrays = fine_archive["arrays"]
        fine_records = fine_arrays["records"]
        authority = role_audit.get("fine_authority")
        _require(isinstance(authority, Mapping), f"{label} fine authority is missing")
        _require(
            _same_path(_resolve(authority.get("candidate"), cross_path.parent), candidate_paths[f"{role}-1mm"])
            and authority.get("candidate_sha256") == fine_block["candidate_sha256"][role]
            and _same_path(_resolve(authority.get("archive"), cross_path.parent), fine_archive["path"])
            and authority.get("archive_sha256") == fine_archive["sha256"]
            and int(authority.get("points", -1)) == len(fine_records),
            f"{label} fine authority fingerprint differs",
        )
        archive_paths = _require_exact_keys(
            coarse_block.get("addition_archive_paths"), {"SAND", "ROCK"}, "terrain coarse archive paths"
        )
        archive_hashes = _require_exact_keys(
            coarse_block.get("addition_archive_sha256"), {"SAND", "ROCK"}, "terrain coarse archive hashes"
        )
        coarse_archive_path = _resolve(archive_paths[role], coarse_report_path.parent)
        _require(
            sha256_path(coarse_archive_path) == archive_hashes[role]
            and role_audit.get("coarse_archive_sha256") == archive_hashes[role]
            and Path(str(role_audit.get("coarse_archive", ""))).name == coarse_archive_path.name
            and set(role_audit.get("coarse_archive_keys", ())) == TERRAIN_COARSE_ARCHIVE_KEYS,
            f"{label} coarse archive fingerprint/schema differs",
        )
        coarse_arrays = _load_exact_npz(
            coarse_archive_path,
            TERRAIN_COARSE_ARCHIVE_KEYS,
            label=f"{label} archive",
            artifacts=artifacts,
        )
        coarse_count = int(coarse_block["addition_counts"][role])
        _require(
            all(value.ndim == 1 and len(value) == coarse_count for value in coarse_arrays.values()),
            f"{label} archive arrays are misaligned",
        )
        _verify_suffix_records(
            candidate_paths[f"{role}-5mm"],
            source_count=source_counts[f"{role}-5mm"],
            records=coarse_arrays["records"],
            label=label,
        )
        selection = np.asarray(coarse_arrays["fine_selection_index"], np.int64)
        _require(
            len(np.unique(selection)) == coarse_count
            and np.all(selection >= 0)
            and np.all(selection < len(fine_records)),
            f"{label} fine-selection indices are invalid",
        )
        coarse_xyz = _xyz_records(coarse_arrays["records"])
        fine_xyz = _xyz_records(fine_records)
        _require(
            coarse_arrays["records"].dtype == fine_records.dtype
            and np.array_equal(coarse_xyz, fine_xyz[selection]),
            f"{label} XYZ is not the exact selected fine subset",
        )
        for coarse_name, fine_name in (
            ("fine_target_id", "target_id"),
            ("fine_target_candidate_index", "target_candidate_index"),
            ("fine_global_ledger_index", "global_ledger_index"),
            ("fine_confidence_reason_mask", "confidence_reason_mask"),
            ("fine_confidence_tier", "confidence_tier"),
            ("fine_confidence_surface_spread_m", "confidence_surface_spread_m"),
            ("fine_confidence_preferred_gate_count", "confidence_preferred_gate_count"),
        ):
            _require(
                np.array_equal(coarse_arrays[coarse_name], fine_arrays[fine_name][selection]),
                f"{label} {coarse_name} differs from fine authority",
            )

        source_info = inspect_ply(source_paths[f"{role}-5mm"])
        donor_indices = np.asarray(coarse_arrays["coarse_primary_donor_source_index"], np.int64)
        _require(
            np.all(donor_indices >= 0) and np.all(donor_indices < source_info.count),
            f"{label} donor source indices are invalid",
        )
        source_memory = np.memmap(
            source_info.path,
            dtype=source_info.dtype,
            mode="r",
            offset=source_info.offset,
            shape=(source_info.count,),
        )
        try:
            donor_records = np.asarray(source_memory[donor_indices]).copy()
        finally:
            del source_memory
        if coarse_count:
            donor_scan = np.asarray(donor_records[_scan_field_name(donor_records.dtype)], np.float64)
            donor_distance = np.linalg.norm(_xyz_records(donor_records) - coarse_xyz, axis=1)
            _require(
                np.all(np.isfinite(donor_distance))
                and np.all(np.rint(donor_scan) >= 0)
                and np.all(np.rint(donor_scan) <= 8)
                and np.allclose(
                    donor_distance,
                    coarse_arrays["coarse_nearest_donor_distance_m"],
                    rtol=0.0,
                    atol=max(tolerance, 1.0e-12),
                )
                and np.all(
                    donor_distance
                    <= float(parameters["coarse"]["property_donor_distance_m"])
                    + tolerance
                )
                and np.all(coarse_arrays["coarse_contributing_donor_count"] > 0),
                f"{label} same-role measured donor transfer is invalid",
            )

        coverage_path = _relative_artifact(
            cross_path.parent,
            role_audit.get("coverage_ledger"),
            role_audit.get("coverage_ledger_sha256"),
            label=f"{label} coverage ledger",
            artifacts=artifacts,
        )
        coverage = _load_exact_npz(
            coverage_path,
            TERRAIN_COVERAGE_LEDGER_KEYS,
            label=f"{label} coverage ledger",
            artifacts=artifacts,
        )
        fine_count = len(fine_records)
        _require(
            all(value.ndim == 1 and len(value) == fine_count for value in coverage.values())
            and np.array_equal(coverage["fine_index"], fine_arrays["fine_index"]),
            f"{label} coverage ledger is misaligned",
        )
        represented = np.asarray(coverage["represented_by_existing"], bool)
        selected_mask = np.asarray(coverage["selected_for_coarse"], bool)
        expected_selected = np.zeros(fine_count, bool)
        expected_selected[selection] = True
        source_code = np.asarray(coverage["coverage_source"], np.uint8)
        witness = np.asarray(coverage["coverage_index"], np.int64)
        _require(
            np.array_equal(selected_mask, expected_selected)
            and np.array_equal(represented, source_code == 1)
            and np.all(np.isin(source_code, (1, 2))),
            f"{label} coverage source/selection masks differ",
        )
        measured_xyz = _collect_local_measured_xyz(
            source_paths[f"{role}-5mm"], targets=targets, collar_m=collar
        )
        existing_rows = source_code == 1
        selected_rows = source_code == 2
        _require(
            np.all(witness[existing_rows] >= 0)
            and np.all(witness[existing_rows] < len(measured_xyz))
            and np.all(witness[selected_rows] >= 0)
            and np.all(witness[selected_rows] < fine_count)
            and np.all(expected_selected[witness[selected_rows]]),
            f"{label} coverage witness indices are invalid",
        )
        support_xyz = np.empty((fine_count, 3), np.float64)
        support_xyz[existing_rows] = measured_xyz[witness[existing_rows]]
        support_xyz[selected_rows] = fine_xyz[witness[selected_rows]]
        recomputed_xy = np.linalg.norm(fine_xyz[:, :2] - support_xyz[:, :2], axis=1)
        recomputed_vertical = np.abs(fine_xyz[:, 2] - support_xyz[:, 2])
        _require(
            np.allclose(
                recomputed_xy,
                coverage["coverage_xy_distance_m"],
                rtol=0.0,
                atol=max(tolerance, 1.0e-12),
            )
            and np.allclose(
                recomputed_vertical,
                coverage["coverage_vertical_delta_m"],
                rtol=0.0,
                atol=max(tolerance, 1.0e-12),
            )
            and np.all(recomputed_xy <= spacing + tolerance)
            and np.all(recomputed_vertical <= vertical + tolerance),
            f"{label} coverage ledger differs from independent geometry",
        )
        conflict_limit = max(spacing - tolerance, 0.0)
        selected_existing, _, _ = _nearest_surface_support(
            coarse_xyz,
            measured_xyz,
            maximum_xy_m=spacing,
            maximum_vertical_m=vertical,
            tolerance_m=tolerance,
        )
        _require(
            not coarse_count or np.all(selected_existing >= conflict_limit),
            f"{label} selected additions violate measured terrain clearance",
        )
        minimum_pair = _minimum_compatible_pair_distance(
            coarse_xyz,
            spacing_m=spacing,
            vertical_m=vertical,
            tolerance_m=tolerance,
        )
        _require(
            minimum_pair is None or minimum_pair >= conflict_limit,
            f"{label} selected additions violate pair spacing",
        )
        report_pair = role_audit.get("pair_spacing")
        _require(
            isinstance(report_pair, Mapping)
            and report_pair.get("verified") is True
            and (
                (minimum_pair is None and report_pair.get("minimum_vertical_compatible_pair_xy_distance_m") is None)
                or (
                    minimum_pair is not None
                    and np.isclose(
                        minimum_pair,
                        float(report_pair.get("minimum_vertical_compatible_pair_xy_distance_m")),
                        rtol=0.0,
                        atol=max(tolerance, 1.0e-12),
                    )
                )
            ),
            f"{label} pair-spacing report differs from independent geometry",
        )
        maximum_xy = float(np.max(recomputed_xy)) if fine_count else None
        maximum_vertical = float(np.max(recomputed_vertical)) if fine_count else None
        finite_existing = selected_existing[np.isfinite(selected_existing)]
        minimum_existing = float(np.min(finite_existing)) if len(finite_existing) else None
        unique_targets, target_counts = np.unique(
            coarse_arrays["fine_target_id"].astype(str), return_counts=True
        )
        expected_target_counts = {
            str(target): int(count)
            for target, count in zip(unique_targets, target_counts, strict=True)
        }
        _require(
            int(role_audit.get("fine_points_already_represented_by_5mm_measured_terrain", -1))
            == int(np.count_nonzero(existing_rows))
            and int(role_audit.get("fine_points_selected_for_5mm", -1)) == coarse_count
            and int(role_audit.get("fine_points_covered", -1)) == fine_count
            and role_audit.get("target_counts") == expected_target_counts
            and (
                (maximum_xy is None and role_audit.get("maximum_coverage_xy_distance_m") is None)
                or np.isclose(maximum_xy, float(role_audit.get("maximum_coverage_xy_distance_m")), rtol=0.0, atol=max(tolerance, 1.0e-12))
            )
            and (
                (maximum_vertical is None and role_audit.get("maximum_coverage_vertical_delta_m") is None)
                or np.isclose(maximum_vertical, float(role_audit.get("maximum_coverage_vertical_delta_m")), rtol=0.0, atol=max(tolerance, 1.0e-12))
            )
            and (
                (minimum_existing is None and role_audit.get("minimum_selected_to_existing_xy_distance_m") is None)
                or np.isclose(minimum_existing, float(role_audit.get("minimum_selected_to_existing_xy_distance_m")), rtol=0.0, atol=max(tolerance, 1.0e-12))
            ),
            f"{label} count/distance summary differs from independent geometry",
        )
        for invariant in (
            "exact_xyz_subset_verified",
            "coverage_verified",
            "terrain_clearance_verified",
            "vertical_support_guard_verified",
            "same_role_measured_donor_transfer_verified",
            "cleanmesh_geometry_identity_verified",
        ):
            _require(role_audit.get(invariant) is True, f"{label} invariant {invariant} is missing")
        result["roles"][role] = {
            "fine_points": fine_count,
            "coarse_points": coarse_count,
            "maximum_coverage_xy_distance_m": maximum_xy,
            "maximum_coverage_vertical_delta_m": maximum_vertical,
            "archive": str(coarse_archive_path),
            "coverage_ledger": str(coverage_path),
        }
    return result


def _parse_terrain(
    manifest_path: Path,
    data_dir: Path,
    obstruction: Mapping[str, Mapping],
    artifacts: dict[str, dict],
) -> dict[str, dict]:
    manifest_path, manifest = _load_json(manifest_path, "terrain manifest")
    _artifact(manifest_path, artifacts)
    _require(manifest.get("schema_version") == 1, "unexpected terrain manifest schema")
    _require(manifest.get("operation") == "site1-v11-candidate-only-terrain-interstitial-pipeline", "unexpected terrain operation")
    _require(manifest.get("status") == "built", "terrain stage is not built")
    _require(manifest.get("candidate_only") is True, "terrain stage is not candidate-only")
    _require(manifest.get("canonical_install_performed") is False, "terrain stage reports install")
    invariants = manifest.get("invariants", {})
    expected_top_invariants = {
        "bbox_is_never_a_fill_mask": True,
        "connected_density_deficit_required": True,
        "all_three_surface_predictions_evaluated": True,
        "hard_geometry_vetoes_fail_closed": True,
        "overlapping_targets_globally_arbitrated_per_role": True,
        "same_role_scanid_0_to_8_donors_only": True,
        "all_additions_scanid": 10.0,
        "existing_records_modified": 0,
        "caller_rock_base_preserved_byte_exact": True,
        "fine_additions_authoritative_for_coarse_geometry": True,
        "coarse_additions_exact_fine_xyz_subset": True,
        "coarse_maximal_surface_coverage_verified": True,
        "cross_scale_vertical_support_guard_verified": True,
        "canonical_writes": False,
    }
    _require(
        all(invariants.get(key) == value for key, value in expected_top_invariants.items()),
        "terrain top-level invariants are incomplete",
    )
    config_block = manifest.get("config")
    _require(isinstance(config_block, Mapping), "terrain config fingerprint is missing")
    config = _verify_input_fingerprint(
        config_block,
        parent=manifest_path.parent,
        label="terrain review config",
        artifacts=artifacts,
    )
    _require(
        isinstance(config_block.get("archived_copy"), str)
        and bool(config_block["archived_copy"]),
        "terrain archived review config is missing",
    )
    archived_config = _resolve(config_block["archived_copy"], manifest_path.parent)
    _require(
        sha256_path(archived_config) == config["sha256"],
        "terrain archived review config hash mismatch",
    )
    _artifact(archived_config, artifacts)
    parameters = _verify_terrain_parameters(manifest.get("parameters"))
    implementation = _verify_exact_implementation_hashes(
        manifest.get("implementation", {}),
        family="terrain",
        label="terrain",
        artifacts=artifacts,
    )
    cleanmesh = _verify_input_fingerprint(
        manifest.get("cleanmesh", {}),
        parent=manifest_path.parent,
        label="terrain CleanMesh executable",
        artifacts=artifacts,
    )
    normalization = manifest.get("combined_geometry_normalization")
    _require(isinstance(normalization, Mapping), "terrain normalization audit is missing")
    method = normalization.get("method")
    _require(
        method in {"provided-manifest", "inferred-from-hash-locked-rock-1mm"},
        "terrain normalization method is unexpected",
    )
    normalization_values = _verify_normalization_values(
        normalization.get("values"), "terrain normalization"
    )
    normalization_input = None
    if method == "provided-manifest":
        normalization_input = _verify_input_fingerprint(
            normalization,
            parent=manifest_path.parent,
            label="terrain normalization manifest",
            artifacts=artifacts,
        )
        archived_name = normalization.get("archived_copy")
        _require(
            isinstance(archived_name, str) and bool(archived_name),
            "terrain archived normalization manifest is missing",
        )
        archived_normalization = _resolve(archived_name, manifest_path.parent)
        _require(
            sha256_path(archived_normalization) == normalization_input["sha256"],
            "terrain archived normalization manifest hash mismatch",
        )
        _artifact(archived_normalization, artifacts)
    else:
        _require(
            isinstance(normalization.get("source"), str)
            and bool(normalization["source"]),
            "terrain inferred normalization source is missing",
        )

    targets = _terrain_targets(manifest.get("targets"))
    sources = _require_exact_keys(
        manifest.get("sources"),
        {"SAND-1mm", "ROCK-1mm-base", "SAND-5mm", "ROCK-5mm-base"},
        "terrain source fingerprints",
    )
    source_paths: dict[str, Path] = {}
    source_entries: dict[str, dict] = {}
    release_sources: dict[str, dict] = {}
    for resolution in ("1mm", "5mm"):
        sand_label = f"SAND-{resolution}"
        sand_path = _canonical_source(data_dir, f"Site1-SAND-{resolution}.ply")
        sand_actual = file_fingerprint(sand_path)
        _verify_known_input_fingerprint(
            sources[sand_label],
            actual_path=sand_path,
            actual=sand_actual,
            label=f"terrain {sand_label} source",
        )
        _artifact(sand_path, artifacts)
        source_paths[sand_label] = sand_path
        source_entries[sand_label] = sand_actual
        release_sources[sand_label] = _source_entry(
            sand_path, sand_actual["sha256"], sand_actual["points"]
        )

        rock_label = f"ROCK-{resolution}"
        obstruction_entry = obstruction[rock_label]
        rock_path = Path(obstruction_entry["obstruction_candidate"]["path"])
        rock_actual = file_fingerprint(rock_path)
        _verify_known_input_fingerprint(
            sources[f"ROCK-{resolution}-base"],
            actual_path=rock_path,
            actual=rock_actual,
            label=f"terrain {rock_label} obstruction base",
        )
        _artifact(rock_path, artifacts)
        source_paths[rock_label] = rock_path
        source_entries[rock_label] = rock_actual
        release_sources[rock_label] = obstruction_entry["source"]

    resolution_blocks = _require_exact_keys(
        manifest.get("resolutions"), {"1mm", "5mm"}, "terrain resolutions"
    )
    result: dict[str, dict] = {}
    report_documents: dict[str, Mapping] = {}
    report_paths: dict[str, Path] = {}
    candidate_paths: dict[str, Path] = {}
    source_counts: dict[str, int] = {}
    fine_archives: dict[str, Mapping[str, object]] = {}
    for resolution in ("1mm", "5mm"):
        resolution_block = _require_exact_keys(
            resolution_blocks[resolution],
            TERRAIN_RESULT_KEYS,
            f"terrain {resolution} result",
        )
        _require(
            resolution_block["label"] == resolution,
            f"terrain {resolution} result label differs",
        )
        candidate_map = _require_exact_keys(
            resolution_block["candidate_paths"],
            {"SAND", "ROCK"},
            f"terrain {resolution} candidate paths",
        )
        candidate_hashes = _require_exact_keys(
            resolution_block["candidate_sha256"],
            {"SAND", "ROCK"},
            f"terrain {resolution} candidate hashes",
        )
        addition_counts = _require_exact_keys(
            resolution_block["addition_counts"],
            {"SAND", "ROCK"},
            f"terrain {resolution} addition counts",
        )
        _require(
            int(resolution_block["target_count"]) == len(targets),
            f"terrain {resolution} target count differs",
        )
        report_path = _resolve(resolution_block["report_path"], manifest_path.parent)
        report_path, report = _load_json(report_path, f"terrain {resolution} report")
        _require(
            sha256_path(report_path) == resolution_block["report_sha256"],
            f"terrain {resolution} report hash mismatch",
        )
        _artifact(report_path, artifacts)
        report_documents[resolution] = report
        report_paths[resolution] = report_path
        _require(report.get("candidate_only") is True, f"terrain {resolution} report is not candidate-only")
        _require(report.get("canonical_writes") is False, f"terrain {resolution} report claims canonical writes")
        _require(
            report.get("resolution") == parameters["fine" if resolution == "1mm" else "coarse"],
            f"terrain {resolution} report parameters differ",
        )
        _require(
            report.get("addition_counts") == {
                role: int(addition_counts[role]) for role in ("SAND", "ROCK")
            },
            f"terrain {resolution} report addition counts differ",
        )
        if resolution == "1mm":
            _require(report.get("schema_version") == 1, "terrain fine report schema is unexpected")
            fine_invariants = report.get("invariants", {})
            for key in (
                "annotations_are_search_windows_only",
                "actual_connected_deficit_required",
                "same_role_measured_property_donors_only",
                "overlapping_targets_globally_arbitrated",
                "existing_base_payload_byte_exact",
                "authoritative_archive_equals_candidate_suffix",
            ):
                _require(fine_invariants.get(key) is True, f"terrain fine invariant {key} is missing")
            _require(fine_invariants.get("addition_scan_id") == 10.0, "terrain fine ScanID invariant differs")
            report_targets = report.get("targets")
            _require(
                isinstance(report_targets, list) and len(report_targets) == len(targets),
                "terrain fine target audits are incomplete",
            )
            projected_targets = []
            for row in report_targets:
                _require(isinstance(row, Mapping), "terrain fine target audit row is invalid")
                projected_targets.append(
                    {
                        "id": str(row.get("target_id", "")),
                        "kind": row.get("kind"),
                        "bbox": tuple(float(item) for item in row.get("bbox", ())),
                        "minimum_tier": row.get("minimum_tier"),
                    }
                )
            _require(
                tuple(projected_targets) == tuple(targets),
                "terrain fine target audits differ from top-level targets",
            )
            _require(
                resolution_block["cross_scale_report_path"] is None
                and resolution_block["cross_scale_report_sha256"] is None,
                "terrain fine result unexpectedly contains a cross-scale report",
            )
        else:
            _require(report.get("schema_version") == 2, "terrain coarse report schema is unexpected")
            _require(
                report.get("construction") == "exact-subset-of-final-1mm-authoritative-additions"
                and report.get("independent_target_proposals") == 0
                and report.get("fine_authority_report_sha256")
                == resolution_blocks["1mm"]["report_sha256"],
                "terrain coarse authority construction differs",
            )
            for key, expected in {
                "fine_final_additions_are_authoritative": True,
                "coarse_addition_xyz_is_exact_fine_subset": True,
                "coarse_independent_geometry_proposals": 0,
                "maximal_coverage_at_coarse_spacing": True,
                "floating_returns_do_not_count_as_surface_support": True,
                "same_role_scanid_0_to_8_property_donors_only": True,
                "cleanmesh_recomputed_coarse_scalars_without_xyz_changes": True,
                "candidate_only": True,
                "canonical_writes": False,
            }.items():
                _require(report.get("invariants", {}).get(key) == expected, f"terrain coarse invariant {key} differs")

        _verify_terrain_cleanmesh_artifacts(
            report.get("cleanmesh"),
            report_parent=report_path.parent,
            addition_count=sum(int(addition_counts[role]) for role in ("SAND", "ROCK")),
            cleanmesh=cleanmesh,
            coarse=resolution == "5mm",
            artifacts=artifacts,
        )
        append_only = _require_exact_keys(
            report.get("append_only"), {"SAND", "ROCK"}, f"terrain {resolution} append audits"
        )
        for role in ("SAND", "ROCK"):
            label = f"{role}-{resolution}"
            canonical_name = f"Site1-{role}-{resolution}.ply"
            append = append_only[role]
            _require(isinstance(append, Mapping), f"terrain {label} append audit is missing")
            candidate_path = _resolve(candidate_map[role], manifest_path.parent)
            candidate_paths[label] = candidate_path
            _require(append["base_payload_byte_identical"] is True, f"{label} append invariant missing")
            _require(append["canonical_writes"] is False, f"{label} append stage reports canonical writes")
            _require(append["candidate_sha256"] == candidate_hashes[role], f"{label} candidate hashes disagree")
            prefix_source = source_paths[label]
            source = release_sources[label]
            _require(_same_path(append["source_path"], prefix_source), f"{label} append source path mismatch")
            _require(append["source_sha256"] == source_entries[label]["sha256"], f"{label} append source hash mismatch")
            candidate = _candidate_entry(
                candidate_path,
                candidate_hashes[role],
                append["candidate_vertex_count"],
                source,
            )
            base_count = inspect_ply(prefix_source).count
            source_counts[label] = base_count
            _require(base_count == int(append["source_vertex_count"]), f"{label} append source count mismatch")
            addition_count = int(addition_counts[role])
            _require(
                int(append["addition_count"]) == addition_count
                and base_count + addition_count == candidate["points"],
                f"{label} append count chain mismatch",
            )
            prefix = verify_exact_append_prefix(prefix_source, candidate_path)
            scan = _scan_appended_scan_id(candidate_path, base_count, 10.0)
            if resolution == "1mm":
                fine_archives[role] = _verify_fine_terrain_archive(
                    role=role,
                    report=report,
                    report_path=report_path,
                    resolution_block=resolution_block,
                    candidate_path=candidate_path,
                    source_count=base_count,
                    addition_count=addition_count,
                    artifacts=artifacts,
                )
            result[label] = {
                "canonical": canonical_name,
                "source": source,
                "candidate": candidate,
                "append_prefix": prefix,
                "addition_scan_id": scan,
                "resolution_report": str(report_path),
                "config": config,
                "parameters": parameters,
                "implementation": implementation,
                "cleanmesh": cleanmesh,
                "normalization": {
                    "method": method,
                    "values": normalization_values,
                    "input": normalization_input,
                },
            }
            if role == "ROCK":
                result[label]["partition"] = obstruction[label]["partition"]
                result[label]["obstruction_candidate"] = obstruction[label][
                    "obstruction_candidate"
                ]

    cross_scale = _verify_terrain_cross_scale(
        manifest=manifest,
        manifest_path=manifest_path,
        parameters=parameters,
        targets=targets,
        fine_block=resolution_blocks["1mm"],
        fine_report_path=report_paths["1mm"],
        fine_archives=fine_archives,
        coarse_block=resolution_blocks["5mm"],
        coarse_report=report_documents["5mm"],
        coarse_report_path=report_paths["5mm"],
        source_paths=source_paths,
        candidate_paths=candidate_paths,
        source_counts=source_counts,
        artifacts=artifacts,
    )
    for entry in result.values():
        entry["terrain_cross_scale"] = cross_scale
    return result


def _require_all_six(clouds: Mapping[str, Mapping]) -> None:
    expected = {
        name.removeprefix("Site1-").removesuffix(".ply") for name in CANONICAL_NAMES
    }
    observed = set(clouds)
    missing = sorted(expected - observed)
    unexpected = sorted(observed - expected)
    if missing or unexpected:
        detail = []
        if missing:
            detail.append("missing " + ", ".join(missing))
        if unexpected:
            detail.append("unexpected " + ", ".join(unexpected))
        raise RuntimeError("release cloud set is not exactly six: " + "; ".join(detail))


def _stage_paths(args) -> dict[str, Path]:
    run = args.run_dir
    return {
        "obstructions": args.obstruction_manifest or run / "obstructions" / "manifest.json",
        "water_base_2mm": args.water_2mm_base_manifest or run / "water-base-2mm" / "manifest.json",
        "water_base_5mm": args.water_5mm_base_manifest or run / "water-base-5mm" / "manifest.json",
        "water_geometry_2mm": args.water_2mm_geometry_manifest or run / "water-geometry-2mm" / "manifest.json",
        "water_geometry_5mm": args.water_5mm_geometry_manifest or run / "water-geometry-5mm" / "manifest.json",
        "water_final_2mm": args.water_2mm_final_manifest or run / "water-final-2mm" / "manifest.json",
        "water_final_5mm": args.water_5mm_final_manifest or run / "water-final-5mm" / "manifest.json",
        "terrain": args.terrain_manifest or run / "terrain" / "manifest.json",
    }


def _assemble_stage_chain(
    args,
) -> tuple[dict[str, dict], dict[str, dict], dict[str, Path], dict]:
    paths = {
        key: _lstat_regular(value, f"{key} stage manifest")[0]
        for key, value in _stage_paths(args).items()
    }
    artifacts: dict[str, dict] = {}
    obstruction = _parse_obstructions(paths["obstructions"], args.data_dir, artifacts)
    terrain = _parse_terrain(paths["terrain"], args.data_dir, obstruction, artifacts)
    water_geometry_2mm = _parse_water_geometry_chain(
        label="2mm",
        base_manifest_path=paths["water_base_2mm"],
        geometry_manifest_path=paths["water_geometry_2mm"],
        data_dir=args.data_dir,
        terrain=terrain,
        fine_geometry=None,
        artifacts=artifacts,
    )
    water_geometry_5mm = _parse_water_geometry_chain(
        label="5mm",
        base_manifest_path=paths["water_base_5mm"],
        geometry_manifest_path=paths["water_geometry_5mm"],
        data_dir=args.data_dir,
        terrain=terrain,
        fine_geometry=water_geometry_2mm,
        artifacts=artifacts,
    )
    water_2mm = _parse_water_enrichment(
        label="2mm",
        manifest_path=paths["water_final_2mm"],
        geometry=water_geometry_2mm,
        terrain=terrain,
        fine_enrichment=None,
        artifacts=artifacts,
    )
    water_5mm = _parse_water_enrichment(
        label="5mm",
        manifest_path=paths["water_final_5mm"],
        geometry=water_geometry_5mm,
        terrain=terrain,
        fine_enrichment=water_2mm,
        artifacts=artifacts,
    )
    clouds = {
        "SAND-1mm": terrain["SAND-1mm"],
        "ROCK-1mm": terrain["ROCK-1mm"],
        "WATER-2mm": water_2mm,
        "SAND-5mm": terrain["SAND-5mm"],
        "ROCK-5mm": terrain["ROCK-5mm"],
        "WATER-5mm": water_5mm,
    }
    _require_all_six(clouds)
    config_hashes = {cloud["config"]["sha256"] for cloud in clouds.values()}
    config_paths = {cloud["config"]["path"] for cloud in clouds.values()}
    _require(
        len(config_hashes) == 1,
        "candidate stages were not built from one review-config hash",
    )
    _require(
        len(config_paths) == 1,
        "candidate stages were not built from one review-config path",
    )
    review_config = {
        "path": next(iter(config_paths)),
        "sha256": next(iter(config_hashes)),
    }
    canonical_paths = {
        _resolve_parent_only(args.data_dir / name, "canonical release path")
        for name in CANONICAL_NAMES
    }
    candidate_paths = [
        _resolve_parent_only(
            cloud["candidate"]["path"], "candidate release path"
        )
        for cloud in clouds.values()
    ]
    aliases = sorted(str(path) for path in candidate_paths if path in canonical_paths)
    _require(not aliases, "candidate aliases a canonical cloud: " + ", ".join(aliases))
    _require(
        len(set(candidate_paths)) == len(candidate_paths),
        "the six release candidates are not distinct files",
    )
    return clouds, artifacts, paths, review_config


def _release_manifest_path(args) -> Path:
    return args.release_dir / "manifest.json"


def _build_locked(args) -> dict:
    refuse_running_app()
    args.release_dir.parent.mkdir(parents=True, exist_ok=True)
    destination = _resolve_parent_only(args.release_dir, "release destination")
    if os.path.lexists(destination):
        raise FileExistsError(f"refusing to overwrite release bundle: {destination}")
    clouds, artifacts, stage_paths, review_config = _assemble_stage_chain(args)
    stage = Path(tempfile.mkdtemp(prefix=f".{destination.name}.staging-", dir=destination.parent))
    try:
        snapshots: dict[str, dict] = {}
        for label, cloud in clouds.items():
            source = Path(cloud["source"]["path"])
            snapshot_final = destination / "source-snapshots" / cloud["canonical"]
            snapshot_stage = stage / "source-snapshots" / cloud["canonical"]
            method = _clone_or_copy(source, snapshot_stage)
            fingerprint = file_fingerprint(snapshot_stage)
            _assert_fingerprint(
                snapshot_stage,
                cloud["source"],
                f"{label} snapshot",
                compare_identity=False,
            )
            fingerprint["path"] = str(snapshot_final)
            fingerprint["method"] = method
            snapshots[label] = fingerprint
            cloud["snapshot"] = fingerprint
        mutable_paths = {
            _resolve_parent_only(
                args.data_dir / cloud["canonical"], "mutable canonical path"
            )
            for cloud in clouds.values()
        } | {
            _resolve_parent_only(
                cloud["candidate"]["path"], "mutable candidate path"
            )
            for cloud in clouds.values()
        }
        stable_artifacts = [
            value for value in artifacts.values()
            if _resolve_parent_only(
                value["path"], "stable artifact path"
            ) not in mutable_paths
        ]
        source_bytes = sum(int(cloud["source"]["bytes"]) for cloud in clouds.values())
        candidate_bytes = sum(int(cloud["candidate"]["bytes"]) for cloud in clouds.values())
        water_2mm_old01 = _water_old01_path(
            args.data_dir / clouds["WATER-2mm"]["canonical"]
        )
        water_5mm_old01 = _water_old01_path(
            args.data_dir / clouds["WATER-5mm"]["canonical"]
        )
        _require(
            not os.path.lexists(water_2mm_old01),
            "unexpected Site1-WATER-2mm-old01.ply; release refuses an "
            "ambiguous historic-backup generation",
        )
        _require(
            os.path.lexists(water_5mm_old01),
            "required historic Site1-WATER-5mm-old01.ply is missing",
        )
        protected_water_old01 = {
            "WATER-5mm": file_fingerprint(water_5mm_old01)
        }
        manifest = {
            "schema_version": SCHEMA_VERSION,
            "operation": OPERATION,
            "status": "built",
            "created": _now(),
            "canonical_install_performed": False,
            "data_dir": str(args.data_dir.resolve()),
            "run_dir": str(args.run_dir.resolve()),
            "release_dir": str(destination),
            "stage_manifests": {key: str(path) for key, path in stage_paths.items()},
            "stage_artifacts": stable_artifacts,
            "review_config": review_config,
            "clouds": clouds,
            "snapshots": snapshots,
            "protected_existing_water_old01": protected_water_old01,
            "transactions": [],
            "storage_plan": {
                "source_bytes": source_bytes,
                "candidate_bytes_already_present": candidate_bytes,
                "snapshot_logical_bytes": source_bytes,
                "snapshot_worst_case_new_bytes_if_clone_unavailable": source_bytes,
                "install_full_replacement_copy_bytes": 0,
                "install_method": "same-filesystem-rename",
                "current_generation_backup_location": (
                    "release/transactions/install-*/previous/"
                ),
                "protected_existing_water_old01": {
                    label: value["path"]
                    for label, value in protected_water_old01.items()
                },
            },
            "invariants": {
                "six_outputs_required": True,
                "candidate_xyz_finite": True,
                "source_snapshots_hash_locked": True,
                "water_existing_survivors_byte_exact": True,
                "water_hole_source_prefix_byte_exact": True,
                "obstruction_partition_round_trip_byte_exact": True,
                "terrain_base_prefix_byte_exact": True,
                "terrain_additions_scan_id": 10,
                "terrain_5mm_exact_subset_of_1mm_additions": True,
                "terrain_cross_scale_support_independently_verified": True,
                "cross_stage_review_config_hash_locked": True,
                "water_component_membership_independently_verified": True,
                "water_5mm_exact_subset_of_2mm_additions": True,
                "water_5mm_xyz_normals_exact_subset_of_2mm": True,
                "water_cross_scale_support_distance_independently_verified": True,
                "preexisting_water_old01_untouched": True,
            },
        }
        _atomic_json(stage / "manifest.json", manifest)
        refuse_running_app()
        if os.path.lexists(destination):
            raise FileExistsError(f"release destination appeared during build: {destination}")
        _durable_replace(stage, destination)
    except BaseException:
        if stage.exists():
            shutil.rmtree(stage)
        raise
    return verify(args, acquire_lock=False)


def build(args) -> dict:
    with release_lock(args.release_dir):
        return _build_locked(args)


def _read_release(args) -> tuple[Path, dict]:
    path, manifest = _load_json(_release_manifest_path(args), "release manifest")
    _require(manifest.get("schema_version") == SCHEMA_VERSION, "release schema version mismatch")
    _require(manifest.get("operation") == OPERATION, "unexpected release operation")
    _require(
        _lexical_absolute(manifest.get("data_dir", ""), "release data directory")
        == _lexical_absolute(args.data_dir, "requested data directory"),
        "release data directory mismatch",
    )
    _require(
        _lexical_absolute(manifest.get("run_dir", ""), "release run directory")
        == _lexical_absolute(args.run_dir, "requested run directory"),
        "release run directory mismatch",
    )
    _require(
        _lexical_absolute(manifest.get("release_dir", ""), "release directory")
        == _lexical_absolute(args.release_dir, "requested release directory"),
        "release directory mismatch",
    )
    _require_all_six(manifest.get("clouds", {}))
    allowed_status = {"built", "installed", "restored"}
    _require(manifest.get("status") in allowed_status, "invalid release status")
    expected_names = {
        name.removeprefix("Site1-").removesuffix(".ply"): name
        for name in CANONICAL_NAMES
    }
    candidates: list[Path] = []
    for label, cloud in manifest["clouds"].items():
        _require(cloud.get("canonical") == expected_names[label], f"{label} canonical name mismatch")
        canonical = _resolve_parent_only(
            args.data_dir / cloud["canonical"], f"{label} canonical path"
        )
        _require(
            canonical.parent == args.data_dir.resolve(),
            f"{label} canonical path escapes the data directory",
        )
        candidate = _resolve_parent_only(
            cloud["candidate"]["path"], f"{label} candidate path"
        )
        _require(candidate != canonical, f"{label} candidate aliases its canonical source")
        candidates.append(candidate)
        snapshot = _resolve_parent_only(
            cloud["snapshot"]["path"], f"{label} snapshot path"
        )
        expected_snapshot = _resolve_parent_only(
            args.release_dir / "source-snapshots" / cloud["canonical"],
            f"{label} expected snapshot path",
        )
        _require(snapshot == expected_snapshot, f"{label} snapshot path mismatch")
    _require(len(set(candidates)) == len(candidates), "release candidates are not unique")
    required_stage_keys = {
        "obstructions",
        "water_base_2mm",
        "water_base_5mm",
        "water_geometry_2mm",
        "water_geometry_5mm",
        "water_final_2mm",
        "water_final_5mm",
        "terrain",
    }
    _require(
        set(manifest.get("stage_manifests", {})) == required_stage_keys,
        "release does not reference the complete stage-manifest set",
    )
    required_invariants = {
        "six_outputs_required": True,
        "candidate_xyz_finite": True,
        "source_snapshots_hash_locked": True,
        "water_existing_survivors_byte_exact": True,
        "water_hole_source_prefix_byte_exact": True,
        "obstruction_partition_round_trip_byte_exact": True,
        "terrain_base_prefix_byte_exact": True,
        "terrain_additions_scan_id": 10,
        "terrain_5mm_exact_subset_of_1mm_additions": True,
        "terrain_cross_scale_support_independently_verified": True,
        "cross_stage_review_config_hash_locked": True,
        "water_component_membership_independently_verified": True,
        "water_5mm_exact_subset_of_2mm_additions": True,
        "water_5mm_xyz_normals_exact_subset_of_2mm": True,
        "water_cross_scale_support_distance_independently_verified": True,
        "preexisting_water_old01_untouched": True,
    }
    release_invariants = manifest.get("invariants", {})
    for name, expected in required_invariants.items():
        _require(
            release_invariants.get(name) == expected,
            f"release invariant is absent or false: {name}",
        )
    protected = manifest.get("protected_existing_water_old01")
    _require(
        isinstance(protected, Mapping)
        and set(protected) == {"WATER-5mm"},
        "release must protect exactly the historic WATER-5mm old01",
    )
    water_2mm_old01 = _water_old01_path(
        args.data_dir / manifest["clouds"]["WATER-2mm"]["canonical"]
    )
    _require(
        not os.path.lexists(water_2mm_old01),
        "unexpected Site1-WATER-2mm-old01.ply; release backup contract differs",
    )
    for label, fingerprint in protected.items():
        _require(
            isinstance(fingerprint, Mapping),
            f"{label} protected old01 fingerprint is invalid",
        )
        expected_old01 = _resolve_parent_only(_water_old01_path(
            args.data_dir / manifest["clouds"][label]["canonical"]
        ), f"{label} expected protected old01")
        _require(
            _resolve_parent_only(
                str(fingerprint.get("path", "")), f"{label} protected old01"
            )
            == expected_old01,
            f"{label} protected old01 path differs",
        )
    review_config = manifest.get("review_config", {})
    _require(
        isinstance(review_config, Mapping)
        and isinstance(review_config.get("path"), str)
        and isinstance(review_config.get("sha256"), str),
        "release review-config provenance is missing",
    )
    for label, cloud in manifest["clouds"].items():
        _require(
            cloud.get("config", {}).get("path") == review_config["path"]
            and cloud.get("config", {}).get("sha256") == review_config["sha256"],
            f"{label} review-config provenance differs from the release",
        )
        _require(
            cloud["candidate"].get("non_finite_xyz") == 0,
            f"{label} did not record a finite-XYZ candidate audit",
        )
        _require(
            cloud["snapshot"].get("method") in {"apfs-clone", "copy2"},
            f"{label} snapshot method is invalid",
        )
        if label.startswith("WATER-"):
            _require(cloud.get("survivor", {}).get("verified") is True, f"{label} survivor evidence is missing")
            _require(cloud.get("final_prefix", {}).get("verified") is True, f"{label} final-prefix evidence is missing")
        else:
            _require(cloud.get("append_prefix", {}).get("verified") is True, f"{label} append-prefix evidence is missing")
            scan = cloud.get("addition_scan_id", {})
            _require(scan.get("expected") == 10.0, f"{label} ScanID 10 evidence is missing")
            if label.startswith("ROCK-"):
                _require(cloud.get("partition", {}).get("verified") is True, f"{label} obstruction partition evidence is missing")
    return path, manifest


def _expected_canonical_fingerprint(manifest: Mapping, label: str) -> Mapping:
    cloud = manifest["clouds"][label]
    return cloud["candidate"] if manifest["status"] == "installed" else cloud["source"]


def verify(args, *, acquire_lock: bool = True) -> dict:
    if acquire_lock:
        with release_lock(args.release_dir):
            return verify(args, acquire_lock=False)
    refuse_running_app()
    # Bind the caller to this release before recovery is allowed to mutate a
    # journal or canonical.  Re-read afterwards because recovery may have
    # durably committed the manifest status.
    _read_release(args)
    recovered_transactions = _recover_incomplete_transactions(args)
    _, manifest = _read_release(args)
    failures: list[str] = []
    reports: dict[str, dict] = {}
    for artifact in manifest.get("stage_artifacts", []):
        try:
            _assert_fingerprint(artifact["path"], artifact, f"stage artifact {artifact['path']}")
        except Exception as error:  # aggregate all immutable audit drift
            failures.append(str(error))
    for label, fingerprint in manifest.get(
        "protected_existing_water_old01", {}
    ).items():
        try:
            _assert_fingerprint(
                fingerprint["path"], fingerprint, f"{label} protected old01"
            )
        except Exception as error:
            failures.append(str(error))
    if manifest["status"] == "installed":
        try:
            _validated_committed_install_previous(
                args,
                manifest,
                verify_files=True,
            )
        except Exception as error:
            failures.append(f"installed previous generation: {error}")
    for label, cloud in manifest["clouds"].items():
        report: dict[str, object] = {}
        try:
            source_snapshot = Path(cloud["snapshot"]["path"])
            _assert_fingerprint(
                source_snapshot,
                cloud["snapshot"],
                f"{label} source snapshot entry",
            )
            _assert_fingerprint(
                source_snapshot,
                cloud["source"],
                f"{label} source snapshot content",
                compare_identity=False,
            )
            canonical = args.data_dir / cloud["canonical"]
            candidate_location = (
                canonical
                if manifest["status"] == "installed"
                else Path(cloud["candidate"]["path"])
            )
            _assert_fingerprint(candidate_location, cloud["candidate"], f"{label} candidate")
            invalid = _finite_xyz(candidate_location)
            _require(invalid == 0, f"{label} candidate contains {invalid} non-finite XYZ")
            expected = _expected_canonical_fingerprint(manifest, label)
            _assert_fingerprint(canonical, expected, f"{label} canonical state")
            report.update(
                {
                    "source_points": cloud["source"]["points"],
                    "candidate_points": cloud["candidate"]["points"],
                    "canonical_expected": "candidate" if manifest["status"] == "installed" else "source",
                    "finite_xyz": True,
                }
            )
        except Exception as error:
            failures.append(f"{label}: {error}")
        reports[label] = report
    result = {
        "schema_version": 1,
        "created": _now(),
        "status": manifest["status"],
        "verified": not failures,
        "cloud_count": len(manifest["clouds"]),
        "reports": reports,
        "failures": failures,
        "recovered_transactions": recovered_transactions,
        "storage_plan": manifest.get("storage_plan"),
    }
    _atomic_json(args.release_dir / "verification-report.json", result)
    if failures:
        raise RuntimeError("Scene1 v11 release verification failed: " + "; ".join(failures))
    return result


def _verify_restore_recovery_state(args) -> tuple[Path, dict, dict[str, str]]:
    """Verify only immutable recovery evidence needed to reverse an install.

    Restore must remain available if producer scripts or review inputs have
    advanced since the release was built.  It therefore does not replay the
    build-provenance audit used by ``verify``/``install``.  It still fails
    closed on every canonical, source snapshot, committed previous backup,
    transaction journal, protected old01, and replacement destination.
    """

    refuse_running_app()
    # Recovery is write-capable.  Establish the release/data/run binding from
    # the immutable manifest before permitting it, then consume the manifest
    # again in case recovery completed a durable commit.
    _read_release(args)
    _recover_incomplete_transactions(args)
    manifest_path, manifest = _read_release(args)
    _require(
        manifest["status"] == "installed",
        "release is not currently installed",
    )
    for label, fingerprint in manifest["protected_existing_water_old01"].items():
        _assert_fingerprint(
            fingerprint["path"],
            fingerprint,
            f"{label} protected old01 for restore",
        )
    previous = _validated_committed_install_previous(
        args,
        manifest,
        verify_files=True,
    )
    for label, cloud in manifest["clouds"].items():
        snapshot = Path(cloud["snapshot"]["path"])
        _assert_fingerprint(
            snapshot,
            cloud["snapshot"],
            f"{label} restore source snapshot entry",
        )
        _assert_fingerprint(
            snapshot,
            cloud["source"],
            f"{label} restore source snapshot content",
            compare_identity=False,
        )
        canonical = args.data_dir / cloud["canonical"]
        _assert_fingerprint(
            canonical,
            cloud["candidate"],
            f"{label} installed canonical for restore",
        )
        invalid = _finite_xyz(canonical)
        _require(
            invalid == 0,
            f"{label} installed canonical contains {invalid} non-finite XYZ",
        )
        candidate_destination = Path(cloud["candidate"]["path"])
        _require(
            not os.path.lexists(candidate_destination),
            f"{label} restore candidate destination is occupied: "
            f"{candidate_destination}",
        )
    return manifest_path, manifest, previous


@dataclass
class SwapItem:
    label: str
    canonical: Path
    replacement: Path
    expected_current: Mapping
    expected_replacement: Mapping
    archive_destination: Path | None = None
    archived: Path | None = None


_TERMINAL_JOURNAL_STATES = {
    "committed",
    "committed-recovered",
    "rolled-back",
    "rolled-back-after-recovery",
}


def _write_journal(path: Path, journal: dict) -> None:
    journal["updated"] = _now()
    _atomic_json(path, journal)


def _journal_phase(
    path: Path,
    journal: dict,
    phase: str,
    *,
    label: str | None = None,
    detail: str | None = None,
) -> None:
    journal["state"] = phase
    if label is not None:
        for item in journal["items"]:
            if item["label"] == label:
                item["phase"] = phase
                break
    event: dict[str, object] = {"at": _now(), "phase": phase}
    if label is not None:
        event["label"] = label
    if detail is not None:
        event["detail"] = detail
    journal.setdefault("events", []).append(event)
    _write_journal(path, journal)


_RENAME_GUARD_ROLES = {
    "archive-current",
    "install-replacement",
    "return-replacement",
    "restore-previous",
}


def _entry_matches_without_link_count(
    path: Path, expected: Mapping
) -> dict | None:
    if not os.path.lexists(path):
        return None
    comparison = dict(expected)
    comparison.pop("links", None)
    try:
        return _assert_fingerprint(path, comparison, str(path))
    except Exception:
        return None


def _journal_item(journal: Mapping, label: str) -> dict:
    matches = [row for row in journal["items"] if row.get("label") == label]
    _require(len(matches) == 1, f"journal has no unique row for {label}")
    _require(isinstance(matches[0], dict), f"journal row for {label} is invalid")
    return matches[0]


def _clear_active_guard(
    journal_path: Path, journal: dict, row: dict
) -> None:
    row["active_guard"] = None
    _write_journal(journal_path, journal)


def _quarantine_entry(path: Path, guard_dir: Path, label: str) -> Path:
    """Move an unexpected raced entry aside without deleting its payload."""

    actual = file_fingerprint(path, ply=False)
    descriptor, quarantine_name = tempfile.mkstemp(
        prefix=f".{label}.", suffix=".conflict", dir=guard_dir
    )
    os.close(descriptor)
    quarantine = Path(quarantine_name)
    _durable_replace(path, quarantine)
    _assert_fingerprint(
        quarantine, actual, f"{label} quarantined rename-race entry"
    )
    return quarantine


def _guarded_durable_replace(
    source: Path,
    destination: Path,
    expected: Mapping,
    *,
    journal_path: Path,
    journal: dict,
    label: str,
    role: str,
) -> None:
    """Rename with a journalled inode-preserving hard-link recovery guard.

    A final pathname can be replaced after its last fingerprint check but
    before ``rename(2)`` resolves it.  The private guard keeps the validated
    inode reachable across that window.  A crash leaves enough journal state
    for startup recovery to remove the guard after identifying whether the
    source or destination owns the expected generation.
    """

    _require(role in _RENAME_GUARD_ROLES, f"invalid rename guard role: {role}")
    row = _journal_item(journal, label)
    _require(
        row.get("active_guard") is None,
        f"{label} already has an active rename guard",
    )
    expected_bound = dict(expected)
    _require(
        expected_bound.get("device") is not None
        and expected_bound.get("inode") is not None
        and expected_bound.get("links") == 1,
        f"{label} rename lacks a single-link inode contract",
    )
    _assert_fingerprint(source, expected_bound, f"{label} guarded rename source")
    _require(
        not os.path.lexists(destination),
        f"{label} guarded rename destination is occupied: {destination}",
    )

    guard_dir = journal_path.parent / "rename-guards"
    guard_dir.mkdir(exist_ok=True)
    guard_dir, _ = _lstat_directory(guard_dir, f"{label} rename guard directory")
    guard = guard_dir / f"{label}-{role}.guard"
    _require(not os.path.lexists(guard), f"{label} rename guard already exists")
    row["active_guard"] = {
        "path": str(guard),
        "source": str(source),
        "destination": str(destination),
        "role": role,
        "expected": expected_bound,
    }
    _write_journal(journal_path, journal)

    try:
        os.link(source, guard, follow_symlinks=False)
        _fsync_directory(guard_dir)
        guarded_expected = {**expected_bound, "links": 2}
        _assert_fingerprint(guard, guarded_expected, f"{label} rename guard")
        _assert_fingerprint(
            source, guarded_expected, f"{label} guarded immediate source"
        )
        _require(
            not os.path.lexists(destination),
            f"{label} guarded rename destination appeared",
        )
        refuse_running_app()
        _durable_replace(source, destination)
        try:
            _assert_fingerprint(
                destination,
                guarded_expected,
                f"{label} guarded rename destination",
            )
        except Exception as mismatch:
            # Preserve any raced entry rather than overwriting or deleting it,
            # then put the exact guarded inode back at the source pathname.
            if os.path.lexists(destination):
                _quarantine_entry(destination, guard_dir, f"{label}-{role}")
            if os.path.lexists(source):
                _quarantine_entry(source, guard_dir, f"{label}-{role}-source")
            _durable_replace(guard, source)
            _assert_fingerprint(
                source,
                {key: value for key, value in expected_bound.items() if key != "links"},
                f"{label} restored guarded source",
            )
            _clear_active_guard(journal_path, journal, row)
            raise RuntimeError(
                f"{label} rename source changed inside the rename window; "
                "the validated inode was restored and the raced entry quarantined"
            ) from mismatch

        guard.unlink()
        _fsync_directory(guard_dir)
        _clear_active_guard(journal_path, journal, row)
        _assert_fingerprint(
            destination, expected_bound, f"{label} completed guarded rename"
        )
    except BaseException:
        # If rename never happened, dropping the extra link restores the
        # source's single-link contract.  Ambiguous states deliberately keep
        # the guard and its journal evidence for recovery/manual inspection.
        source_actual = _entry_matches_without_link_count(source, expected_bound)
        destination_actual = _entry_matches_without_link_count(
            destination, expected_bound
        )
        if os.path.lexists(guard) and source_actual is not None and destination_actual is None:
            guard.unlink()
            _fsync_directory(guard_dir)
            _clear_active_guard(journal_path, journal, row)
        raise


def _reconcile_active_guards(journal_path: Path, journal: dict) -> None:
    """Finish cleanup of a guard left by a crash without moving cloud data."""

    changed = False
    for row in journal["items"]:
        active = row.get("active_guard")
        if active is None:
            continue
        guard = Path(active["path"])
        expected = active["expected"]
        if not os.path.lexists(guard):
            row["active_guard"] = None
            changed = True
            continue
        guard_actual = _entry_matches_without_link_count(guard, expected)
        _require(
            guard_actual is not None,
            f"{row['label']} active rename guard fingerprint differs",
        )
        source = Path(active["source"])
        destination = Path(active["destination"])
        source_actual = _entry_matches_without_link_count(source, expected)
        destination_actual = _entry_matches_without_link_count(
            destination, expected
        )
        _require(
            (source_actual is None) != (destination_actual is None),
            f"{row['label']} active rename guard has ambiguous source/destination state",
        )
        guard.unlink()
        _fsync_directory(guard.parent)
        survivor = source if source_actual is not None else destination
        _assert_fingerprint(
            survivor, expected, f"{row['label']} reconciled guarded rename"
        )
        row["active_guard"] = None
        changed = True
    if changed:
        _write_journal(journal_path, journal)


def _path_matches(path: Path, expected: Mapping) -> bool:
    if not os.path.lexists(path):
        return False
    try:
        _assert_fingerprint(path, expected, str(path))
    except Exception:
        return False
    return True


def _journal_canonicals_match(journal: Mapping, key: str) -> bool:
    return all(
        _path_matches(Path(item["canonical"]), item[key])
        for item in journal["items"]
    )


def _journal_archives_match(journal: Mapping) -> bool:
    return all(
        _path_matches(Path(item["archived"]), item["expected_current"])
        for item in journal["items"]
    )


def _assert_journal_archives(journal: Mapping, label: str) -> None:
    for item in journal["items"]:
        _assert_fingerprint(
            item["archived"],
            item["expected_current"],
            f"{item['label']} {label}",
        )


def _manifest_status(path: Path | None) -> str | None:
    if path is None or not os.path.lexists(path):
        return None
    try:
        _, manifest = _load_json(path, "transaction release manifest")
        return manifest.get("status")
    except (OSError, RuntimeError, AttributeError):
        return None


def _same_fingerprint_contract(left: Mapping, right: Mapping) -> bool:
    keys = (
        "bytes", "sha256", "points", "record_stride", "schema",
        "device", "inode", "links",
    )
    return all(left.get(key) == right.get(key) for key in keys if key in right)


def _lexical_absolute(path: str | Path, label: str) -> Path:
    value = Path(path)
    _require(value.is_absolute(), f"{label} must be absolute")
    return Path(os.path.abspath(value))


def _resolve_parent_only(path: str | Path, label: str) -> Path:
    """Resolve directory aliases without following the final path entry."""

    value = _lexical_absolute(path, label)
    return value.parent.resolve(strict=True) / value.name


def _validate_recovery_journal(
    args,
    journal_path: Path,
    journal: Mapping,
    release_manifest: Mapping,
) -> None:
    """Reject a stale/tampered journal before using any path it contains."""

    _require(journal.get("schema_version") == 2, f"invalid journal schema: {journal_path}")
    _require(
        journal.get("operation") == "site1-v11-durable-six-cloud-transaction",
        f"invalid journal operation: {journal_path}",
    )
    action = journal.get("action")
    source_status = journal.get("source_status")
    target_status = journal.get("target_status")
    if action == "install":
        _require(source_status in {"built", "restored"}, "invalid install journal source status")
        _require(target_status == "installed", "invalid install journal target status")
    elif action == "restore":
        _require(source_status == "installed", "invalid restore journal source status")
        _require(target_status == "restored", "invalid restore journal target status")
    else:
        raise RuntimeError(f"invalid journal action: {action!r}")

    _require(release_manifest.get("operation") == OPERATION, "release manifest operation mismatch during recovery")
    clouds = release_manifest.get("clouds", {})
    _require_all_six(clouds)
    rows = journal.get("items")
    _require(isinstance(rows, list) and len(rows) == 6, "recovery journal does not contain six items")
    _require({row.get("label") for row in rows} == set(clouds), "recovery journal cloud set mismatch")

    data_dir = args.data_dir.resolve()
    transactions_root, _ = _lstat_directory(
        args.release_dir / "transactions", "release transactions directory"
    )
    lexical_journal = _lexical_absolute(journal_path, "recovery journal path")
    resolved_journal = lexical_journal.resolve(strict=True)
    _require(
        not lexical_journal.is_symlink()
        and not lexical_journal.parent.is_symlink()
        and resolved_journal.parent.parent == transactions_root,
        f"recovery journal escapes or traverses a symlink: {journal_path}",
    )
    transaction_dir = resolved_journal.parent
    _require(
        transaction_dir.name.startswith(f"{action}-"),
        f"recovery journal directory does not match its action: {journal_path}",
    )
    previous_dir = transaction_dir / "previous"
    _require(
        previous_dir.resolve(strict=True) == previous_dir,
        f"recovery previous directory traverses a symlink: {previous_dir}",
    )
    latest_install_previous: Mapping | None = None
    if action == "restore":
        latest_install_previous = _validated_committed_install_previous(
            args,
            release_manifest,
            verify_files=False,
        )
    for row in rows:
        label = row["label"]
        cloud = clouds[label]
        canonical = _resolve_parent_only(
            data_dir / cloud["canonical"], f"{label} recovery canonical"
        )
        _require(
            _resolve_parent_only(
                row["canonical"], f"{label} journal canonical path"
            ) == canonical,
            f"{label} journal canonical path mismatch",
        )
        archived_lexical = _lexical_absolute(
            row["archived"], f"{label} journal archive path"
        )
        archived = _resolve_parent_only(
            archived_lexical, f"{label} journal archive path"
        )
        if action == "install":
            expected_current = cloud["source"]
            expected_replacement = cloud["candidate"]
            replacement = Path(cloud["candidate"]["path"])
            expected_archive = previous_dir / canonical.name
        else:
            expected_current = cloud["candidate"]
            expected_replacement = cloud["source"]
            assert latest_install_previous is not None
            _require(
                isinstance(latest_install_previous.get(label), str),
                f"{label} committed install backup path is missing",
            )
            replacement = Path(latest_install_previous[label])
            expected_archive = Path(cloud["candidate"]["path"])
        _require(
            archived == expected_archive and not archived_lexical.is_symlink(),
            f"{label} journal archive path mismatch",
        )
        _require(
            _resolve_parent_only(
                row["replacement"], f"{label} journal replacement path"
            )
            == _resolve_parent_only(
                replacement, f"{label} expected replacement path"
            ),
            f"{label} journal replacement path mismatch",
        )
        _require(
            isinstance(row.get("expected_current"), Mapping)
            and _same_fingerprint_contract(row["expected_current"], expected_current),
            f"{label} journal current fingerprint mismatch",
        )
        _require(
            isinstance(row.get("expected_replacement"), Mapping)
            and _same_fingerprint_contract(
                row["expected_replacement"], expected_replacement
            ),
            f"{label} journal replacement fingerprint mismatch",
        )
        if journal.get("state") not in _TERMINAL_JOURNAL_STATES:
            _require(
                row["expected_current"].get("links") == 1
                and row["expected_replacement"].get("links") == 1,
                f"{label} incomplete journal lacks a single-link contract",
            )
        active = row.get("active_guard")
        if active is not None:
            active = _require_exact_keys(
                active,
                {"path", "source", "destination", "role", "expected"},
                f"{label} active rename guard",
            )
            role = active["role"]
            _require(
                role in _RENAME_GUARD_ROLES,
                f"{label} active rename guard role is invalid",
            )
            if role == "archive-current":
                guard_source, guard_destination = canonical, archived
                guard_expected = row["expected_current"]
            elif role == "install-replacement":
                guard_source, guard_destination = replacement, canonical
                guard_expected = row["expected_replacement"]
            elif role == "return-replacement":
                guard_source, guard_destination = canonical, replacement
                guard_expected = row["expected_replacement"]
            else:
                guard_source, guard_destination = archived, canonical
                guard_expected = row["expected_current"]
            expected_guard = (
                transaction_dir
                / "rename-guards"
                / f"{label}-{role}.guard"
            )
            _require(
                _resolve_parent_only(active["path"], f"{label} guard path")
                == _resolve_parent_only(
                    expected_guard, f"{label} expected guard path"
                )
                and _resolve_parent_only(
                    active["source"], f"{label} guard source"
                )
                == _resolve_parent_only(
                    guard_source, f"{label} expected guard source"
                )
                and _resolve_parent_only(
                    active["destination"], f"{label} guard destination"
                )
                == _resolve_parent_only(
                    guard_destination,
                    f"{label} expected guard destination",
                )
                and active["expected"] == guard_expected,
                f"{label} active rename guard contract differs",
            )


def _validated_committed_install_previous(
    args,
    manifest: Mapping,
    *,
    verify_files: bool,
) -> dict[str, str]:
    """Return only the exact, journal-backed previous generation.

    Restore is destructive to its replacement paths, so no path from the
    mutable release manifest is used until it is proven to be the canonical
    ``previous/<canonical>`` entry of the latest committed install journal.
    """

    installs = [
        row for row in manifest.get("transactions", [])
        if isinstance(row, Mapping) and row.get("action") == "install"
    ]
    _require(bool(installs), "release has no committed install backup paths")
    transaction = installs[-1]
    _require(
        isinstance(transaction.get("directory"), str)
        and isinstance(transaction.get("journal"), str),
        "install transaction directory/journal provenance is missing",
    )
    transactions_root, _ = _lstat_directory(
        args.release_dir / "transactions", "release transactions directory"
    )
    transaction_lexical = _lexical_absolute(
        transaction["directory"], "install transaction directory"
    )
    transaction_dir = transaction_lexical.resolve(strict=True)
    _require(
        transaction_dir.parent == transactions_root
        and transaction_dir.name.startswith("install-")
        and not transaction_lexical.is_symlink(),
        "install transaction directory escapes the release transaction root",
    )
    journal_lexical = _lexical_absolute(
        transaction["journal"], "install transaction journal"
    )
    journal_path = journal_lexical.resolve(strict=True)
    expected_journal_path = (transaction_dir / "journal.json").resolve(strict=True)
    _require(
        journal_path == expected_journal_path
        and not journal_lexical.is_symlink(),
        "install transaction journal path differs from its directory",
    )
    previous = transaction.get("previous")
    _require(
        isinstance(previous, Mapping)
        and set(previous) == set(manifest["clouds"])
        and all(isinstance(value, str) for value in previous.values()),
        "install transaction backup map is incomplete",
    )
    expected_previous: dict[str, str] = {}
    previous_dir = transaction_dir / "previous"
    _require(
        previous_dir.resolve(strict=True) == previous_dir,
        "install transaction previous directory traverses a symlink",
    )
    for label, cloud in manifest["clouds"].items():
        expected_path = previous_dir / cloud["canonical"]
        actual_lexical = _lexical_absolute(
            previous[label], f"{label} install backup path"
        )
        actual_path = _resolve_parent_only(
            actual_lexical, f"{label} install backup path"
        )
        _require(
            actual_path == expected_path
            and not actual_lexical.is_symlink()
            and not expected_path.is_symlink(),
            f"{label} install backup path is not the committed previous entry",
        )
        expected_previous[label] = str(expected_path)

    _, journal = _load_json(journal_path, "committed install journal")
    _require(
        journal.get("state") in {"committed", "committed-recovered"},
        "install transaction journal is not committed",
    )
    _validate_recovery_journal(args, journal_path, journal, manifest)
    if verify_files:
        for label, path in expected_previous.items():
            _assert_fingerprint(
                path,
                manifest["clouds"][label]["source"],
                f"{label} committed previous backup",
            )
    return expected_previous


def _rollback_transaction_journal(
    journal_path: Path,
    journal: dict,
    *,
    recovered: bool,
    reason: str,
) -> None:
    """Infer rename progress, then durably restore the old generation.

    Every item's finite state is validated before the first rename.  This is
    important for a multi-cloud generation: discovering an unknown file late
    in the reversed journal must not leave earlier clouds already rolled back
    while later clouds still contain the replacement generation.
    """

    transaction_dir = journal_path.parent
    _reconcile_active_guards(journal_path, journal)
    _journal_phase(
        journal_path,
        journal,
        "rollback-started",
        detail=reason,
    )

    rollback_plan: list[dict[str, object]] = []
    for item in reversed(journal["items"]):
        label = str(item["label"])
        canonical = Path(item["canonical"])
        replacement = Path(item["replacement"])
        archived = Path(item["archived"])
        expected_current = item["expected_current"]
        expected_replacement = item["expected_replacement"]

        canonical_state = "absent"
        if os.path.lexists(canonical):
            if _path_matches(canonical, expected_replacement):
                canonical_state = "replacement"
                _require(
                    not os.path.lexists(replacement),
                    f"{label}: replacement path is occupied during rollback",
                )
            elif _path_matches(canonical, expected_current):
                canonical_state = "current"
            else:
                raise RuntimeError(
                    f"{label}: canonical matches neither journal generation; "
                    "automatic recovery refuses to overwrite it"
                )

        if canonical_state in {"absent", "replacement"}:
            if not _path_matches(archived, expected_current):
                raise RuntimeError(
                    f"{label}: old canonical is absent from both canonical and archive"
                )

        if canonical_state in {"absent", "current"}:
            if not _path_matches(replacement, expected_replacement):
                raise RuntimeError(
                    f"{label}: replacement generation is absent or does not match "
                    "the journal"
                )

        rollback_plan.append(
            {
                "label": label,
                "canonical": canonical,
                "replacement": replacement,
                "archived": archived,
                "expected_current": expected_current,
                "expected_replacement": expected_replacement,
                "return_replacement": canonical_state == "replacement",
                "restore_previous": canonical_state in {"absent", "replacement"},
            }
        )

    for row in rollback_plan:
        label = str(row["label"])
        canonical = Path(row["canonical"])
        replacement = Path(row["replacement"])
        archived = Path(row["archived"])
        expected_current = row["expected_current"]
        expected_replacement = row["expected_replacement"]

        if row["return_replacement"]:
            replacement.parent.mkdir(parents=True, exist_ok=True)
            _lstat_directory(replacement.parent, f"{label} replacement parent")
            _require(
                not os.path.lexists(replacement),
                f"{label}: replacement path became occupied during rollback",
            )
            _assert_fingerprint(
                canonical,
                expected_replacement,
                f"{label} rollback replacement source",
            )
            _journal_phase(
                journal_path, journal, "before-return-replacement",
                label=label, detail=f"{canonical} -> {replacement}",
            )
            _guarded_durable_replace(
                canonical,
                replacement,
                expected_replacement,
                journal_path=journal_path,
                journal=journal,
                label=label,
                role="return-replacement",
            )
            _journal_phase(
                journal_path, journal, "returned-replacement",
                label=label,
            )

        if row["restore_previous"]:
            _require(
                not os.path.lexists(canonical),
                f"{label}: canonical path became occupied before rollback restore",
            )
            _assert_fingerprint(
                archived,
                expected_current,
                f"{label} rollback previous source",
            )
            _journal_phase(
                journal_path, journal, "before-restore-previous",
                label=label, detail=f"{archived} -> {canonical}",
            )
            _guarded_durable_replace(
                archived,
                canonical,
                expected_current,
                journal_path=journal_path,
                journal=journal,
                label=label,
                role="restore-previous",
            )
            _journal_phase(
                journal_path, journal, "restored-previous",
                label=label,
            )

        _assert_fingerprint(
            canonical, expected_current, f"{label} recovered canonical"
        )
        _assert_fingerprint(
            replacement, expected_replacement, f"{label} returned replacement"
        )

    final_state = (
        "rolled-back-after-recovery" if recovered else "rolled-back"
    )
    _journal_phase(
        journal_path, journal, final_state, detail=reason
    )


def _recover_incomplete_transactions(args) -> list[dict]:
    """Recover crash-interrupted swaps before trusting canonical state."""

    root = args.release_dir / "transactions"
    if not os.path.lexists(root):
        return []
    root, _ = _lstat_directory(root, "release transactions directory")
    manifest_path = args.release_dir / "manifest.json"
    _, release_manifest = _load_json(manifest_path, "release manifest during recovery")
    recovered: list[dict] = []
    for journal_path in sorted(root.glob("*/journal.json")):
        _, journal = _load_json(journal_path, "transaction journal")
        state = journal.get("state")
        if state in _TERMINAL_JOURNAL_STATES:
            continue
        _validate_recovery_journal(args, journal_path, journal, release_manifest)
        _reconcile_active_guards(journal_path, journal)
        source_status = journal.get("source_status")
        target_status = journal.get("target_status")
        observed_status = _manifest_status(manifest_path)
        if (
            observed_status == target_status
            and _journal_canonicals_match(journal, "expected_replacement")
            and _journal_archives_match(journal)
        ):
            _journal_phase(
                journal_path,
                journal,
                "committed-recovered",
                detail="manifest and all canonical replacements were durable",
            )
            recovered.append(
                {
                    "journal": str(journal_path),
                    "outcome": "committed-recovered",
                }
            )
            continue
        if observed_status != source_status:
            raise RuntimeError(
                f"incomplete transaction {journal_path.parent.name} has "
                f"manifest status {observed_status!r}, expected source status "
                f"{source_status!r}; refusing ambiguous automatic recovery"
            )
        _rollback_transaction_journal(
            journal_path,
            journal,
            recovered=True,
            reason="startup recovery of incomplete transaction",
        )
        recovered.append(
            {
                "journal": str(journal_path),
                "outcome": "rolled-back-after-recovery",
            }
        )
    return recovered


def _transactional_replace(
    items: Sequence[SwapItem],
    *,
    transaction_dir: Path,
    manifest_commit,
    action: str = "test",
    source_status: str = "built",
    target_status: str = "installed",
    manifest_path: Path | None = None,
) -> dict:
    """Durably journal and rename-swap every replacement as one generation.

    The app process is probed after preflight and immediately before both
    canonical-directory renames for every cloud.  A process-name probe cannot
    eliminate the final scheduling race without a lock shared by the app, so
    the residual window is deliberately kept to the single rename call.
    """

    transaction_dir.parent.mkdir(parents=True, exist_ok=True)
    _lstat_directory(transaction_dir.parent, "transaction root")
    transaction_dir = _resolve_parent_only(
        transaction_dir, "transaction directory"
    )
    if os.path.lexists(transaction_dir):
        raise FileExistsError(f"transaction directory exists: {transaction_dir}")
    transaction_dir.mkdir()
    _lstat_directory(transaction_dir, "transaction directory")
    _fsync_directory(transaction_dir.parent)
    previous = transaction_dir / "previous"
    previous.mkdir()
    _lstat_directory(previous, "transaction previous directory")
    _fsync_directory(transaction_dir)
    for item in items:
        item.archived = (
            item.archive_destination
            if item.archive_destination is not None
            else previous / item.canonical.name
        )
        item.archived.parent.mkdir(parents=True, exist_ok=True)
        _lstat_directory(item.archived.parent, f"{item.label} archive parent")
    canonical_paths = {
        _resolve_parent_only(item.canonical, f"{item.label} canonical path")
        for item in items
    }
    replacement_paths = {
        _resolve_parent_only(item.replacement, f"{item.label} replacement path")
        for item in items
    }
    archive_paths = {
        _resolve_parent_only(item.archived, f"{item.label} archive path")
        for item in items if item.archived is not None
    }
    _require(len(canonical_paths) == len(items), "transaction canonical paths are duplicated")
    _require(len(replacement_paths) == len(items), "transaction replacement paths are duplicated")
    _require(len(archive_paths) == len(items), "transaction archive paths are duplicated")
    _require(not canonical_paths.intersection(replacement_paths), "transaction replacement aliases a canonical")
    _require(not canonical_paths.intersection(archive_paths), "transaction archive aliases a canonical")
    _require(not replacement_paths.intersection(archive_paths), "transaction archive aliases a replacement")
    entry_identities: set[tuple[int, int]] = set()
    for item in items:
        assert item.archived is not None
        current_actual = _assert_fingerprint(
            item.canonical, item.expected_current, f"{item.label} current canonical"
        )
        replacement_actual = _assert_fingerprint(
            item.replacement,
            item.expected_replacement,
            f"{item.label} replacement",
        )
        # Older release manifests did not persist entry identity.  Preserve
        # compatibility with their content contract, but bind every new
        # transaction and recovery journal to the exact inodes observed in
        # this locked preflight.
        item.expected_current = {
            **dict(item.expected_current),
            "device": int(current_actual["device"]),
            "inode": int(current_actual["inode"]),
            "links": int(current_actual["links"]),
        }
        item.expected_replacement = {
            **dict(item.expected_replacement),
            "device": int(replacement_actual["device"]),
            "inode": int(replacement_actual["inode"]),
            "links": int(replacement_actual["links"]),
        }
        _require(
            current_actual["links"] == replacement_actual["links"] == 1,
            f"{item.label} transaction entry has external hard links",
        )
        _require(
            not os.path.lexists(item.archived),
            f"{item.label} archive destination already exists: {item.archived}",
        )
        for kind, actual in (
            ("canonical", current_actual),
            ("replacement", replacement_actual),
        ):
            identity = (int(actual["device"]), int(actual["inode"]))
            _require(
                identity not in entry_identities,
                f"{item.label} {kind} hard-links another transaction entry",
            )
            entry_identities.add(identity)
        _, canonical_parent_stat = _lstat_directory(
            item.canonical.parent, f"{item.label} canonical parent"
        )
        _, archive_parent_stat = _lstat_directory(
            item.archived.parent, f"{item.label} archive parent"
        )
        devices = {
            canonical_parent_stat.st_dev,
            int(replacement_actual["device"]),
            archive_parent_stat.st_dev,
        }
        _require(
            len(devices) == 1,
            f"{item.label} transaction paths are not on one filesystem",
        )
    journal_path = transaction_dir / "journal.json"
    journal = {
        "schema_version": 2,
        "operation": "site1-v11-durable-six-cloud-transaction",
        "action": action,
        "source_status": source_status,
        "target_status": target_status,
        "state": "intent",
        "created": _now(),
        "items": [
            {
                "label": item.label,
                "canonical": str(item.canonical),
                "replacement": str(item.replacement),
                "archived": str(item.archived),
                "expected_current": dict(item.expected_current),
                "expected_replacement": dict(item.expected_replacement),
                "active_guard": None,
                "phase": "intent",
            }
            for item in items
        ],
        "events": [{"at": _now(), "phase": "intent"}],
    }
    _write_journal(journal_path, journal)
    methods = {item.label: "same-filesystem-rename" for item in items}
    try:
        refuse_running_app()
        for item in items:
            refuse_running_app()
            _assert_fingerprint(item.canonical, item.expected_current, f"{item.label} pre-swap canonical")
            _assert_fingerprint(item.replacement, item.expected_replacement, f"{item.label} pre-swap replacement")
            assert item.archived is not None
            _require(
                not os.path.lexists(item.archived),
                f"{item.label} archive destination appeared before swap",
            )
            _journal_phase(
                journal_path, journal, "before-archive-current",
                label=item.label,
            )
            refuse_running_app()
            _assert_fingerprint(
                item.canonical,
                item.expected_current,
                f"{item.label} immediate pre-archive canonical",
            )
            _require(
                not os.path.lexists(item.archived),
                f"{item.label} archive destination appeared before rename",
            )
            _guarded_durable_replace(
                item.canonical,
                item.archived,
                item.expected_current,
                journal_path=journal_path,
                journal=journal,
                label=item.label,
                role="archive-current",
            )
            _assert_fingerprint(
                item.archived,
                item.expected_current,
                f"{item.label} archived canonical",
            )
            _journal_phase(
                journal_path, journal, "archived-current", label=item.label
            )
            _journal_phase(
                journal_path, journal, "before-install-replacement",
                label=item.label,
            )
            refuse_running_app()
            _require(
                not os.path.lexists(item.canonical),
                f"{item.label} canonical path appeared before replacement rename",
            )
            _assert_fingerprint(
                item.replacement,
                item.expected_replacement,
                f"{item.label} immediate pre-install replacement",
            )
            _guarded_durable_replace(
                item.replacement,
                item.canonical,
                item.expected_replacement,
                journal_path=journal_path,
                journal=journal,
                label=item.label,
                role="install-replacement",
            )
            _journal_phase(
                journal_path, journal, "installed-replacement",
                label=item.label,
            )
        _journal_phase(journal_path, journal, "verifying-installed-generation")
        for item in items:
            _assert_fingerprint(item.canonical, item.expected_replacement, f"{item.label} installed canonical")
        _assert_journal_archives(journal, "archived previous generation")
        refuse_running_app()
        transaction = {
            "created": _now(),
            "directory": str(transaction_dir),
            "journal": str(journal_path),
            "replacement_methods": methods,
            "previous": {
                item.label: str(item.archived) for item in items
            },
        }
        _journal_phase(journal_path, journal, "before-manifest-commit")
        manifest_commit(transaction)
        _journal_phase(journal_path, journal, "committed")
        return transaction
    except BaseException as original_error:
        if (
            _manifest_status(manifest_path) == target_status
            and _journal_canonicals_match(journal, "expected_replacement")
            and _journal_archives_match(journal)
        ):
            try:
                _journal_phase(
                    journal_path,
                    journal,
                    "committed-recovered",
                    detail=(
                        "manifest commit was durable although transaction "
                        f"finalisation raised: {original_error}"
                    ),
                )
            except Exception:
                pass
            return {
                "created": _now(),
                "directory": str(transaction_dir),
                "journal": str(journal_path),
                "replacement_methods": methods,
                "previous": {
                    item.label: str(item.archived) for item in items
                },
                "commit_recovered_after_error": str(original_error),
            }
        try:
            _rollback_transaction_journal(
                journal_path,
                journal,
                recovered=False,
                reason=f"{type(original_error).__name__}: {original_error}",
            )
        except Exception as rollback_error:
            raise RuntimeError(
                f"release transaction failed ({original_error}); durable rollback "
                f"also failed ({rollback_error}). Recovery journal: {journal_path}"
            ) from rollback_error
        raise


def _new_transaction_dir(args, action: str) -> Path:
    stamp = dt.datetime.now().strftime("%Y%m%dT%H%M%S.%f")
    return args.release_dir / "transactions" / f"{action}-{stamp}-{os.getpid()}"


def _commit_status(
    manifest_path: Path,
    manifest: dict,
    *,
    status: str,
    action: str,
    transaction: Mapping,
) -> None:
    updated = json.loads(json.dumps(manifest))
    updated["status"] = status
    updated["canonical_install_performed"] = status == "installed"
    updated[f"{action}_at"] = _now()
    history = list(updated.get("transactions", []))
    history.append({"action": action, **dict(transaction)})
    updated["transactions"] = history
    _atomic_json(manifest_path, updated)


def _water_old01_path(canonical: Path) -> Path:
    return canonical.with_name(f"{canonical.stem}-old01{canonical.suffix}")


def install(args) -> dict:
    with release_lock(args.release_dir):
        refuse_running_app()
        verify(args, acquire_lock=False)
        manifest_path, manifest = _read_release(args)
        _require(manifest["status"] != "installed", "release is already installed")
        items = []
        for label, cloud in manifest["clouds"].items():
            canonical = args.data_dir / cloud["canonical"]
            items.append(
                SwapItem(
                    label=label,
                    canonical=canonical,
                    replacement=Path(cloud["candidate"]["path"]),
                    expected_current=cloud["source"],
                    expected_replacement=cloud["candidate"],
                )
            )
        transaction_dir = _new_transaction_dir(args, "install")

        def commit(transaction):
            _commit_status(
                manifest_path, manifest, status="installed", action="install",
                transaction=transaction,
            )

        transaction = _transactional_replace(
            items,
            transaction_dir=transaction_dir,
            manifest_commit=commit,
            action="install",
            source_status=manifest["status"],
            target_status="installed",
            manifest_path=manifest_path,
        )
        return {"installed": True, "transaction": transaction}


def restore(args) -> dict:
    with release_lock(args.release_dir):
        refuse_running_app()
        manifest_path, manifest, previous = _verify_restore_recovery_state(args)
        items = []
        for label, cloud in manifest["clouds"].items():
            items.append(
                SwapItem(
                    label=label,
                    canonical=args.data_dir / cloud["canonical"],
                    replacement=Path(previous[label]),
                    expected_current=cloud["candidate"],
                    expected_replacement=cloud["source"],
                    archive_destination=Path(cloud["candidate"]["path"]),
                )
            )
        transaction_dir = _new_transaction_dir(args, "restore")

        def commit(transaction):
            _commit_status(
                manifest_path, manifest, status="restored", action="restore",
                transaction=transaction,
            )

        transaction = _transactional_replace(
            items,
            transaction_dir=transaction_dir,
            manifest_commit=commit,
            action="restore",
            source_status="installed",
            target_status="restored",
            manifest_path=manifest_path,
        )
        return {"restored": True, "transaction": transaction}


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    result.add_argument("stage", choices=("build", "verify", "install", "restore"))
    result.add_argument("--data-dir", type=Path, default=DEFAULT_DATA)
    result.add_argument("--run-dir", type=Path, default=DEFAULT_RUN)
    result.add_argument("--release-dir", type=Path, default=None)
    result.add_argument("--obstruction-manifest", type=Path)
    result.add_argument("--water-2mm-base-manifest", type=Path)
    result.add_argument("--water-5mm-base-manifest", type=Path)
    result.add_argument("--water-2mm-geometry-manifest", type=Path)
    result.add_argument("--water-5mm-geometry-manifest", type=Path)
    result.add_argument("--water-2mm-final-manifest", type=Path)
    result.add_argument("--water-5mm-final-manifest", type=Path)
    result.add_argument("--terrain-manifest", type=Path)
    return result


def main(argv: Sequence[str] | None = None) -> int:
    args = parser().parse_args(argv)
    args.data_dir = args.data_dir.resolve()
    args.run_dir = args.run_dir.resolve()
    args.release_dir = (
        args.release_dir.resolve()
        if args.release_dir is not None
        else args.run_dir / "release"
    )
    if args.stage == "build":
        result = build(args)
    elif args.stage == "verify":
        result = verify(args)
    elif args.stage == "install":
        result = install(args)
    else:
        result = restore(args)
    print(json.dumps(result, indent=2, sort_keys=True), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
