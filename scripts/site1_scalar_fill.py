#!/usr/bin/env python3
"""Conservatively fill undefined Scene1 geometry scalar fields.

The Scene1 PLY files are fixed-stride binary-little-endian record arrays.  This
module clones (or copies) a source cloud to a candidate path and changes only
non-finite geometry scalar values in that candidate.  Existing finite values,
coordinates, colours, normals, Intensity, Composite, and ScanID are preserved
bit-for-bit.

Undefined component fields are transferred from complete, same-role 3-D
neighbours.  Fine, Medium, Broad, and lower-context fields keep separate
candidate masks, while one complete joint donor bundle lets their union share
a single spatial query without contaminating finite values.  Directional
fields are first regenerated directly from each point's finite normal.  An
optional, explicitly configured group-specific XY IDW pass can then repair
the still-undefined rows at larger distances without letting a different
scalar group restrict the donor set.  An optional repaired 5 mm cloud can
contribute same-role donors while repairing a 1 mm cloud.

Combined and Fine/Medium-relative fields are never donor-transferred.  Only
their non-finite values are regenerated after component repair.  Combined
fields follow CleanMesh reduced-analysis semantics: each finite scale is
divided by a robust source-derived absolute p95, clipped signed or positive,
then blended with 0.45/0.35/0.20 weights renormalized over the finite scales.
Existing finite derived values remain byte-identical.

The donor catalogue is deterministically subsampled to cap memory.  Candidate
records are queried and patched in chunks, and macOS APFS clone-on-write is
used when available.  The final verifier checks every property: unrelated
properties and finite source scalar values must be byte-identical, while each
changed value must replace a non-finite source value and satisfy hard bounds.
No canonical Data file is selected by default; both source and output paths
must be supplied explicitly.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import shutil
import subprocess
import sys
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Mapping, Sequence

import numpy as np

try:
    import rebuild_site1_fossils_water as v6
except ModuleNotFoundError:  # Imported by path from outside scripts/.
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import rebuild_site1_fossils_water as v6


ROLES = ("SAND", "ROCK", "VEG")
REPAIR_ROLES = (*ROLES, "WATER")
SPACINGS = ("1mm", "2mm", "5mm")
CANONICAL_TERRAIN_SPACINGS = ("1mm", "5mm")
XYZ_FIELDS = ("x", "y", "z")
DISTANCE_BUCKETS_M = (0.025, 0.050, 0.100, 0.200, 0.400)
DEFAULT_DATA_DIR = Path(__file__).resolve().parents[1] / "Data/Scene1"
DERIVED_COMBINATION_ALGORITHM = "cleanmesh-reduced-p95-weighted-v1"
DERIVED_SCALES = ("Fine", "Medium", "Broad")
DERIVED_WEIGHTS = (0.45, 0.35, 0.20)
DERIVED_NORMALIZATION_PERCENTILE = 0.95
DERIVED_NORMALIZATION_SAMPLE_LIMIT = 2_000_000
NOMINAL_SPACING_M = {"1mm": 0.001, "2mm": 0.002, "5mm": 0.005}

METRICS = ("MeanCurvature", "CrossCurvature", "Recession", "Roughness")
COMPONENT_GROUPS: Mapping[str, tuple[str, ...]] = {
    scale: tuple(f"scalar_A_R_{metric}_{scale}" for metric in METRICS)
    for scale in ("Fine", "Medium", "Broad")
}
COMPONENT_GROUPS = {
    **COMPONENT_GROUPS,
    "LowerContext": (
        "scalar_A_R_Shelter_Lower",
        "scalar_A_R_RainExposure_Lower",
        "scalar_A_R_SVF_Lower",
    ),
    "Directional": (
        "scalar_A_R_Downhill_X",
        "scalar_A_R_Downhill_Y",
        "scalar_A_R_Downhill_Z",
        "scalar_A_R_DownhillMagnitude",
        "scalar_A_R_Horizontalness",
        "scalar_A_R_Slope_deg",
    ),
}

DERIVED_FIELDS = (
    "scalar_A_R_MeanCurvature_Combined",
    "scalar_A_R_CrossCurvature_Combined",
    "scalar_A_R_Recession_Combined",
    "scalar_A_R_Roughness_Combined",
    "scalar_A_R_RoughnessRelative_FineMedium",
)

REPAIRABLE_FIELDS = tuple(
    dict.fromkeys(
        field
        for fields in COMPONENT_GROUPS.values()
        for field in fields
    )
) + DERIVED_FIELDS

# These are validity guards, not a normalization range.  Local IDW remains
# range-preserving; clipping can only absorb floating-point round-off at a
# guard boundary.  The component guards are intentionally wider than the
# measured Scene1 distributions.
FIELD_BOUNDS: Mapping[str, tuple[float, float]] = {
    **{
        f"scalar_A_R_{metric}_{scale}": (-5_000.0, 5_000.0)
        for metric in ("MeanCurvature", "CrossCurvature")
        for scale in ("Fine", "Medium", "Broad")
    },
    **{
        f"scalar_A_R_Recession_{scale}": (-2.0, 2.0)
        for scale in ("Fine", "Medium", "Broad")
    },
    **{
        f"scalar_A_R_Roughness_{scale}": (0.0, 2.0)
        for scale in ("Fine", "Medium", "Broad")
    },
    "scalar_A_R_MeanCurvature_Combined": (-1.0, 1.0),
    "scalar_A_R_CrossCurvature_Combined": (-1.0, 1.0),
    "scalar_A_R_Recession_Combined": (-1.0, 1.0),
    "scalar_A_R_Roughness_Combined": (0.0, 1.0),
    "scalar_A_R_RoughnessRelative_FineMedium": (0.0, 8.0),
    "scalar_A_R_Shelter_Lower": (0.0, 1.0),
    "scalar_A_R_RainExposure_Lower": (0.0, 1.0),
    "scalar_A_R_SVF_Lower": (0.0, 1.0),
    "scalar_A_R_Downhill_X": (-1.0, 1.0),
    "scalar_A_R_Downhill_Y": (-1.0, 1.0),
    "scalar_A_R_Downhill_Z": (-1.0, 1.0),
    "scalar_A_R_DownhillMagnitude": (0.0, 100.0),
    "scalar_A_R_Horizontalness": (-1.0, 1.0),
    "scalar_A_R_Slope_deg": (0.0, 90.0),
}


@dataclass(frozen=True)
class PlyInfo:
    path: Path
    dtype: np.dtype
    count: int
    offset: int
    header: bytes
    file_size: int


@dataclass(frozen=True)
class RepairOptions:
    """Resource and safety controls for :func:`repair_scalar_file`."""

    chunk_size: int = 300_000
    max_donors_per_group: int = 1_500_000
    donor_query_k: int = 8
    fallback_share: float = 0.35
    distance_buckets_m: tuple[float, ...] = DISTANCE_BUCKETS_M
    xy_fallback_buckets_m: tuple[float, ...] = ()
    workers: int = -1
    prefer_clone: bool = True
    overwrite: bool = False
    verify: bool = True
    derived_weights: tuple[float, float, float] = DERIVED_WEIGHTS
    derived_normalization_percentile: float = DERIVED_NORMALIZATION_PERCENTILE
    derived_normalization_sample_limit: int = DERIVED_NORMALIZATION_SAMPLE_LIMIT
    derived_normalization: Mapping[str, Mapping[str, float]] | None = None

    def validate(self) -> None:
        if self.chunk_size <= 0:
            raise ValueError("chunk_size must be positive")
        if self.max_donors_per_group <= 0:
            raise ValueError("max_donors_per_group must be positive")
        if self.donor_query_k <= 0:
            raise ValueError("donor_query_k must be positive")
        if not 0.0 <= self.fallback_share <= 1.0:
            raise ValueError("fallback_share must be in [0, 1]")
        buckets = np.asarray(self.distance_buckets_m, dtype=np.float64)
        if len(buckets) == 0 or np.any(~np.isfinite(buckets)):
            raise ValueError("distance buckets must be finite and non-empty")
        if np.any(buckets <= 0.0) or np.any(np.diff(buckets) <= 0.0):
            raise ValueError("distance buckets must be positive and increasing")
        xy_buckets = np.asarray(self.xy_fallback_buckets_m, dtype=np.float64)
        if len(xy_buckets):
            if np.any(~np.isfinite(xy_buckets)) or np.any(xy_buckets <= 0.0):
                raise ValueError("XY fallback buckets must be finite and positive")
            if np.any(np.diff(xy_buckets) <= 0.0):
                raise ValueError("XY fallback buckets must be increasing")
        weights = np.asarray(self.derived_weights, dtype=np.float64)
        if weights.shape != (3,) or np.any(~np.isfinite(weights)):
            raise ValueError("derived weights must contain three finite values")
        if np.any(weights < 0.0) or float(np.sum(weights)) <= 0.0:
            raise ValueError("derived weights must be nonnegative with positive sum")
        if not 0.0 < self.derived_normalization_percentile <= 1.0:
            raise ValueError("derived normalization percentile must be in (0, 1]")
        if self.derived_normalization_sample_limit <= 0:
            raise ValueError("derived normalization sample limit must be positive")
        if self.derived_normalization is not None:
            _validated_derived_normalization(
                self.derived_normalization,
                nominal_spacing=0.001,
            )


@dataclass
class DonorTable:
    tree: object
    coordinates: np.ndarray
    values: dict[str, np.ndarray]
    origins: np.ndarray
    count_by_origin: dict[str, int]
    sampling: list[dict]


def inspect_fixed_stride_ply(path: Path | str) -> PlyInfo:
    """Return and validate the fixed-record layout used by Scene1 clouds."""

    path = Path(path)
    dtype, count, offset, header = v6.read_ply_header(path)
    if b"format binary_little_endian 1.0" not in header:
        raise RuntimeError(f"{path} is not binary_little_endian PLY")
    if any(dtype[name].hasobject or dtype[name].subdtype for name in dtype.names):
        raise RuntimeError(f"{path} is not a fixed-scalar record PLY")
    file_size = path.stat().st_size
    expected_size = offset + count * dtype.itemsize
    if file_size != expected_size:
        raise RuntimeError(
            f"{path} is not fixed-stride: expected {expected_size} bytes, "
            f"found {file_size}"
        )
    return PlyInfo(path, dtype, count, offset, header, file_size)


def _records(info: PlyInfo, mode: str = "r") -> np.memmap:
    return np.memmap(
        info.path,
        dtype=info.dtype,
        mode=mode,
        offset=info.offset,
        shape=(info.count,),
    )


def _slices(count: int, chunk_size: int) -> Iterable[slice]:
    for start in range(0, count, chunk_size):
        yield slice(start, min(start + chunk_size, count))


def _present_groups(dtype: np.dtype) -> dict[str, tuple[str, ...]]:
    names = set(dtype.names)
    groups: dict[str, tuple[str, ...]] = {}
    for group, fields in COMPONENT_GROUPS.items():
        present = tuple(field for field in fields if field in names)
        if present and len(present) != len(fields):
            missing = sorted(set(fields) - names)
            raise RuntimeError(f"partial {group} scalar group; missing {missing}")
        if present:
            groups[group] = present
    return groups


def _validate_schema(info: PlyInfo) -> dict[str, tuple[str, ...]]:
    missing_xyz = sorted(set(XYZ_FIELDS) - set(info.dtype.names))
    if missing_xyz:
        raise RuntimeError(f"{info.path} is missing coordinates {missing_xyz}")
    groups = _present_groups(info.dtype)
    for name in (*XYZ_FIELDS, *(f for fields in groups.values() for f in fields)):
        if info.dtype[name].kind != "f":
            raise RuntimeError(f"{info.path}: {name} must be floating point")
    names = set(info.dtype.names)
    for field in DERIVED_FIELDS:
        if field in names and info.dtype[field].kind != "f":
            raise RuntimeError(f"{info.path}: {field} must be floating point")
    return groups


def audit_nonfinite(
    info: PlyInfo,
    groups: Mapping[str, Sequence[str]],
    chunk_size: int,
) -> dict:
    """Count every repair mask in one sequential read of the cloud."""

    group_report = {
        group: {
            "fields": list(fields),
            "candidate_rows": 0,
            "invalid_by_field": {field: 0 for field in fields},
        }
        for group, fields in groups.items()
    }
    derived_fields = [field for field in DERIVED_FIELDS if field in info.dtype.names]
    derived_report = {field: 0 for field in derived_fields}
    records = _records(info)
    for span in _slices(info.count, chunk_size):
        chunk = records[span]
        for group, fields in groups.items():
            group_invalid = np.zeros(len(chunk), dtype=bool)
            for field in fields:
                invalid = ~np.isfinite(chunk[field])
                group_report[group]["invalid_by_field"][field] += int(
                    np.count_nonzero(invalid)
                )
                group_invalid |= invalid
            group_report[group]["candidate_rows"] += int(
                np.count_nonzero(group_invalid)
            )
        for field in derived_fields:
            derived_report[field] += int(np.count_nonzero(~np.isfinite(chunk[field])))
    del records
    return {"groups": group_report, "derived": derived_report}


def _repair_directional_from_normals(
    candidate: PlyInfo,
    audit: Mapping,
    options: RepairOptions,
) -> dict:
    """Fill only invalid directional fields from each point's own normal."""

    fields = COMPONENT_GROUPS["Directional"]
    report = {
        "candidate_rows": int(audit["candidate_rows"]),
        "filled_rows": 0,
        "filled_by_field": {field: 0 for field in fields},
        "remaining_nonfinite_by_field": dict(audit["invalid_by_field"]),
        "remaining_candidate_rows": int(audit["candidate_rows"]),
        "nonfinite_normal_rows": 0,
        "zero_length_normal_rows": 0,
        "flipped_upward_rows": 0,
    }
    required = {*fields, "nx", "ny", "nz"}
    if not required.issubset(candidate.dtype.names) or not audit["candidate_rows"]:
        report["skipped"] = "directional fields or normals unavailable"
        return report
    records = _records(candidate, mode="r+")
    for span in _slices(candidate.count, options.chunk_size):
        chunk = records[span]
        invalid_by_field = {
            field: ~np.isfinite(chunk[field])
            for field in fields
        }
        candidate_mask = np.zeros(len(chunk), dtype=bool)
        for invalid in invalid_by_field.values():
            candidate_mask |= invalid
        local = np.flatnonzero(candidate_mask)
        if not len(local):
            continue
        normals = np.column_stack(
            [chunk[field][local] for field in ("nx", "ny", "nz")]
        ).astype(np.float64, copy=False)
        finite = np.all(np.isfinite(normals), axis=1)
        length = np.linalg.norm(np.where(finite[:, None], normals, 0.0), axis=1)
        usable = finite & (length > 1e-8)
        report["nonfinite_normal_rows"] += int(np.count_nonzero(~finite))
        report["zero_length_normal_rows"] += int(
            np.count_nonzero(finite & (length <= 1e-8))
        )
        if not np.any(usable):
            continue
        normal = normals[usable] / length[usable, None]
        downward = normal[:, 2] < 0.0
        normal[downward] *= -1.0
        report["flipped_upward_rows"] += int(np.count_nonzero(downward))
        horizontal = np.linalg.norm(normal[:, :2], axis=1)
        nz = np.clip(normal[:, 2], 1e-5, 1.0)
        downhill_x = np.zeros(len(normal), dtype=np.float64)
        downhill_y = np.zeros(len(normal), dtype=np.float64)
        active = horizontal > 1e-7
        downhill_x[active] = normal[active, 0] / horizontal[active]
        downhill_y[active] = normal[active, 1] / horizontal[active]
        derived = {
            "scalar_A_R_Downhill_X": downhill_x,
            "scalar_A_R_Downhill_Y": downhill_y,
            "scalar_A_R_Downhill_Z": np.zeros(len(normal), dtype=np.float64),
            "scalar_A_R_DownhillMagnitude": horizontal / nz,
            "scalar_A_R_Horizontalness": nz,
            "scalar_A_R_Slope_deg": np.degrees(np.arccos(nz)),
        }
        usable_rows = local[usable]
        for field, values in derived.items():
            field_invalid = invalid_by_field[field][usable_rows]
            if not np.any(field_invalid):
                continue
            low, high = FIELD_BOUNDS[field]
            chunk[field][usable_rows[field_invalid]] = np.clip(
                values[field_invalid], low, high
            )
            report["filled_by_field"][field] += int(np.count_nonzero(field_invalid))
        report["filled_rows"] += len(usable_rows)
    records.flush()
    del records
    report["remaining_nonfinite_by_field"] = {
        field: int(audit["invalid_by_field"][field])
        - report["filled_by_field"][field]
        for field in fields
    }
    report["remaining_candidate_rows"] = (
        int(audit["candidate_rows"]) - report["filled_rows"]
    )
    return report


def _canonical_role_in_name(path: Path) -> str | None:
    match = re.search(
        r"(?:^|-)" + "(" + "|".join(REPAIR_ROLES) + r")(?:-|\.)",
        path.name,
    )
    return match.group(1) if match else None


def _validate_role(path: Path, expected_role: str) -> None:
    labelled_role = _canonical_role_in_name(path)
    if labelled_role is not None and labelled_role != expected_role:
        raise RuntimeError(
            f"same-role transfer required: {path.name} is {labelled_role}, "
            f"expected {expected_role}"
        )


def is_canonical_output(path: Path | str) -> bool:
    """True only for one of the six canonical Scene1 terrain paths."""

    resolved = Path(path).resolve()
    return any(
        resolved == (DEFAULT_DATA_DIR / f"Site1-{role}-{spacing}.ply").resolve()
        for role in ROLES
        for spacing in CANONICAL_TERRAIN_SPACINGS
    )


def _values_in_bounds(chunk: np.ndarray, fields: Sequence[str]) -> np.ndarray:
    valid = np.ones(len(chunk), dtype=bool)
    for field in fields:
        values = chunk[field]
        low, high = FIELD_BOUNDS[field]
        valid &= np.isfinite(values) & (values >= low) & (values <= high)
    return valid


def _valid_donor_mask(chunk: np.ndarray, fields: Sequence[str]) -> np.ndarray:
    valid = np.ones(len(chunk), dtype=bool)
    for coordinate in XYZ_FIELDS:
        valid &= np.isfinite(chunk[coordinate])
    return valid & _values_in_bounds(chunk, fields)


def _count_valid_donors(
    info: PlyInfo, fields: Sequence[str], chunk_size: int
) -> int:
    records = _records(info)
    count = 0
    for span in _slices(info.count, chunk_size):
        count += int(np.count_nonzero(_valid_donor_mask(records[span], fields)))
    del records
    return count


def _allocate_donor_quotas(
    primary_count: int, fallback_count: int, maximum: int, fallback_share: float
) -> tuple[int, int]:
    if fallback_count <= 0:
        return min(primary_count, maximum), 0
    fallback_quota = min(fallback_count, int(round(maximum * fallback_share)))
    primary_quota = min(primary_count, maximum - fallback_quota)
    remaining = maximum - primary_quota - fallback_quota
    if remaining:
        extra_primary = min(remaining, primary_count - primary_quota)
        primary_quota += extra_primary
        remaining -= extra_primary
    if remaining:
        fallback_quota += min(remaining, fallback_count - fallback_quota)
    return primary_quota, fallback_quota


def _sample_donors(
    info: PlyInfo,
    fields: Sequence[str],
    valid_count: int,
    quota: int,
    chunk_size: int,
) -> tuple[np.ndarray, dict[str, np.ndarray], dict]:
    if valid_count <= 0 or quota <= 0:
        return (
            np.empty((0, 3), dtype=np.float64),
            {field: np.empty(0, dtype=np.float32) for field in fields},
            {"path": str(info.path), "valid_donors": valid_count, "sampled": 0, "stride": None},
        )
    stride = max(1, math.ceil(valid_count / quota))
    capacity = (valid_count + stride - 1) // stride
    coordinates = np.empty((capacity, 3), dtype=np.float64)
    values = {field: np.empty(capacity, dtype=np.float32) for field in fields}
    records = _records(info)
    valid_seen = 0
    written = 0
    for span in _slices(info.count, chunk_size):
        chunk = records[span]
        donor_mask = _valid_donor_mask(chunk, fields)
        local = np.flatnonzero(donor_mask)
        if len(local) == 0:
            continue
        sequence = valid_seen + np.arange(len(local), dtype=np.int64)
        selected = local[(sequence % stride) == 0]
        valid_seen += len(local)
        if len(selected) == 0:
            continue
        end = written + len(selected)
        coordinates[written:end, 0] = chunk["x"][selected]
        coordinates[written:end, 1] = chunk["y"][selected]
        coordinates[written:end, 2] = chunk["z"][selected]
        for field in fields:
            values[field][written:end] = chunk[field][selected]
        written = end
    del records
    coordinates = coordinates[:written]
    values = {field: array[:written] for field, array in values.items()}
    return coordinates, values, {
        "path": str(info.path),
        "valid_donors": valid_count,
        "sampled": written,
        "stride": stride,
    }


def _build_donor_table(
    source: PlyInfo,
    fallback: PlyInfo | None,
    fields: Sequence[str],
    options: RepairOptions,
    *,
    dimensions: int = 3,
) -> DonorTable | None:
    from scipy.spatial import cKDTree

    primary_count = _count_valid_donors(source, fields, options.chunk_size)
    fallback_count = (
        _count_valid_donors(fallback, fields, options.chunk_size)
        if fallback is not None
        else 0
    )
    primary_quota, fallback_quota = _allocate_donor_quotas(
        primary_count,
        fallback_count,
        options.max_donors_per_group,
        options.fallback_share,
    )
    primary_xyz, primary_values, primary_report = _sample_donors(
        source,
        fields,
        primary_count,
        primary_quota,
        options.chunk_size,
    )
    tables = [("source", primary_xyz, primary_values, primary_report)]
    if fallback is not None:
        fallback_xyz, fallback_values, fallback_report = _sample_donors(
            fallback,
            fields,
            fallback_count,
            fallback_quota,
            options.chunk_size,
        )
        tables.append(("fallback", fallback_xyz, fallback_values, fallback_report))
    total = sum(len(table[1]) for table in tables)
    if total == 0:
        return None
    coordinates = np.concatenate([table[1] for table in tables], axis=0)
    values = {
        field: np.concatenate([table[2][field] for table in tables], axis=0)
        for field in fields
    }
    origins = np.concatenate(
        [np.full(len(table[1]), index, dtype=np.uint8) for index, table in enumerate(tables)]
    )
    count_by_origin = {
        table[0]: int(len(table[1]))
        for table in tables
    }
    if dimensions not in (2, 3):
        raise ValueError("donor tree dimensions must be 2 or 3")
    tree_coordinates = (
        coordinates
        if dimensions == 3
        else np.ascontiguousarray(coordinates[:, :2])
    )
    tree = cKDTree(
        tree_coordinates,
        compact_nodes=True,
        balanced_tree=True,
        copy_data=False,
    )
    return DonorTable(
        tree=tree,
        coordinates=coordinates,
        values=values,
        origins=origins,
        count_by_origin=count_by_origin,
        sampling=[table[3] for table in tables],
    )


def _bucket_labels(buckets: Sequence[float]) -> list[str]:
    return [f"le_{int(round(distance * 1000))}mm" for distance in buckets]


def _empty_group_report(
    fields: Sequence[str],
    audit: Mapping,
    donor_table: DonorTable | None,
    labels: Sequence[str],
) -> dict:
    invalid = dict(audit["invalid_by_field"])
    return {
        "fields": list(fields),
        "candidate_rows": int(audit["candidate_rows"]),
        "invalid_before_by_field": invalid,
        "filled_by_field": {field: 0 for field in fields},
        "remaining_nonfinite_by_field": dict(invalid),
        "filled_rows_by_bucket": {label: 0 for label in labels},
        "remaining_candidate_rows": int(audit["candidate_rows"]),
        "rows_without_finite_coordinates": 0,
        "rows_beyond_max_bucket_or_without_donor": 0,
        "nearest_origin_for_filled_rows": {"source": 0, "fallback": 0},
        "donors": (
            donor_table.count_by_origin
            if donor_table is not None
            else {"source": 0, "fallback": 0}
        ),
        "sampling": donor_table.sampling if donor_table is not None else [],
    }


def _fill_groups_joint(
    candidate: PlyInfo,
    groups: Mapping[str, Sequence[str]],
    audits: Mapping[str, Mapping],
    donor_table: DonorTable | None,
    options: RepairOptions,
    spacing: str,
) -> dict[str, dict]:
    """Use one complete donor bundle and one query for all active masks."""

    buckets = np.asarray(options.distance_buckets_m, dtype=np.float64)
    labels = _bucket_labels(buckets)
    reports = {
        group: _empty_group_report(fields, audits[group], donor_table, labels)
        for group, fields in groups.items()
    }
    if donor_table is None or not groups:
        if donor_table is None:
            for report in reports.values():
                report["rows_beyond_max_bucket_or_without_donor"] = report[
                    "candidate_rows"
                ]
        return reports
    records = _records(candidate, mode="r+")
    donor_count = len(donor_table.origins)
    query_k = min(options.donor_query_k, donor_count)
    idw_floor = 0.0005 if spacing == "1mm" else 0.002
    for span in _slices(candidate.count, options.chunk_size):
        chunk = records[span]
        invalid_by_group: dict[str, dict[str, np.ndarray]] = {}
        mask_by_group: dict[str, np.ndarray] = {}
        union_mask = np.zeros(len(chunk), dtype=bool)
        for group, fields in groups.items():
            invalid_by_field = {
                field: ~np.isfinite(chunk[field])
                for field in fields
            }
            group_mask = np.zeros(len(chunk), dtype=bool)
            for invalid in invalid_by_field.values():
                group_mask |= invalid
            invalid_by_group[group] = invalid_by_field
            mask_by_group[group] = group_mask
            union_mask |= group_mask
        for coordinate in XYZ_FIELDS:
            union_mask &= np.isfinite(chunk[coordinate])
        local = np.flatnonzero(union_mask)
        for group in groups:
            reports[group]["rows_without_finite_coordinates"] += int(
                np.count_nonzero(mask_by_group[group] & ~union_mask)
            )
        if len(local) == 0:
            continue
        query_points = np.column_stack(
            [chunk[coordinate][local] for coordinate in XYZ_FIELDS]
        ).astype(np.float64, copy=False)
        distances, donor_indices = donor_table.tree.query(
            query_points,
            k=query_k,
            workers=options.workers,
        )
        if query_k == 1:
            distances = distances[:, None]
            donor_indices = donor_indices[:, None]
        nearest = distances[:, 0]
        bucket_index = np.searchsorted(buckets, nearest, side="left")
        eligible = bucket_index < len(buckets)
        safe_bucket_index = np.minimum(bucket_index, len(buckets) - 1)
        ceiling = buckets[safe_bucket_index]
        neighbour_mask = (
            np.isfinite(distances)
            & (donor_indices >= 0)
            & (donor_indices < donor_count)
            & (distances <= ceiling[:, None])
        )
        safe_indices = np.where(neighbour_mask, donor_indices, 0)
        weights = np.where(
            neighbour_mask,
            1.0 / np.maximum(distances, idw_floor) ** 2,
            0.0,
        )
        weight_sum = np.sum(weights, axis=1)
        eligible &= weight_sum > 0.0
        if not np.any(eligible):
            for group in groups:
                reports[group]["rows_beyond_max_bucket_or_without_donor"] += int(
                    np.count_nonzero(mask_by_group[group][local])
                )
            continue
        for group, fields in groups.items():
            group_local_mask = mask_by_group[group][local]
            reports[group]["rows_beyond_max_bucket_or_without_donor"] += int(
                np.count_nonzero(group_local_mask & ~eligible)
            )
            group_query_mask = group_local_mask & eligible
            if not np.any(group_query_mask):
                continue
            query_rows = np.flatnonzero(group_query_mask)
            target_local = local[query_rows]
            for field in fields:
                field_invalid = invalid_by_group[group][field][target_local]
                if not np.any(field_invalid):
                    continue
                donor_values = donor_table.values[field][safe_indices]
                blended = np.sum(donor_values * weights, axis=1) / np.maximum(
                    weight_sum, 1e-300
                )
                low, high = FIELD_BOUNDS[field]
                blended = np.clip(blended, low, high)
                chunk[field][target_local[field_invalid]] = blended[query_rows][
                    field_invalid
                ]
                reports[group]["filled_by_field"][field] += int(
                    np.count_nonzero(field_invalid)
                )
            group_buckets = bucket_index[group_query_mask]
            for index, label in enumerate(labels):
                reports[group]["filled_rows_by_bucket"][label] += int(
                    np.count_nonzero(group_buckets == index)
                )
            nearest_origins = donor_table.origins[
                donor_indices[group_query_mask, 0]
            ]
            reports[group]["nearest_origin_for_filled_rows"]["source"] += int(
                np.count_nonzero(nearest_origins == 0)
            )
            reports[group]["nearest_origin_for_filled_rows"]["fallback"] += int(
                np.count_nonzero(nearest_origins == 1)
            )
    records.flush()
    del records
    for report in reports.values():
        filled_rows = sum(report["filled_rows_by_bucket"].values())
        report["remaining_candidate_rows"] = report["candidate_rows"] - filled_rows
        report["remaining_nonfinite_by_field"] = {
            field: report["invalid_before_by_field"][field]
            - report["filled_by_field"][field]
            for field in report["fields"]
        }
    return reports


def _sample_quantiles(values: list[np.ndarray]) -> dict[str, float] | None:
    if not values:
        return None
    combined = np.concatenate(values)
    combined = combined[np.isfinite(combined)]
    if not len(combined):
        return None
    return {
        "min": float(np.min(combined)),
        "q50": float(np.quantile(combined, 0.50)),
        "q90": float(np.quantile(combined, 0.90)),
        "q99": float(np.quantile(combined, 0.99)),
        "max": float(np.max(combined)),
    }


def _fill_group_xy(
    candidate: PlyInfo,
    fields: Sequence[str],
    audit: Mapping,
    donor_table: DonorTable | None,
    options: RepairOptions,
    spacing: str,
) -> dict:
    """Fill one group's remaining gaps from its own same-role XY donors."""

    buckets = np.asarray(options.xy_fallback_buckets_m, dtype=np.float64)
    labels = _bucket_labels(buckets)
    report = _empty_group_report(fields, audit, donor_table, labels)
    report["distance_mode"] = "same-role-xy-idw"
    report["nearest_xy_distance_sample_quantiles_m"] = None
    report["nearest_abs_z_separation_sample_quantiles_m"] = None
    report["distance_sample_count"] = 0
    if not report["candidate_rows"]:
        report["skipped"] = "no_remaining_nonfinite_values"
        return report
    if not len(buckets):
        report["skipped"] = "xy_fallback_disabled"
        return report
    if donor_table is None:
        report["rows_beyond_max_bucket_or_without_donor"] = report[
            "candidate_rows"
        ]
        report["skipped"] = "no_group_complete_donors"
        return report

    donor_count = len(donor_table.origins)
    query_k = min(options.donor_query_k, donor_count)
    idw_floor = 0.0005 if spacing == "1mm" else 0.002
    sample_stride = max(1, math.ceil(max(1, report["candidate_rows"]) / 50_000))
    sample_seen = 0
    distance_samples: list[np.ndarray] = []
    z_samples: list[np.ndarray] = []
    records = _records(candidate, mode="r+")
    for span in _slices(candidate.count, options.chunk_size):
        chunk = records[span]
        invalid_by_field = {
            field: ~np.isfinite(chunk[field])
            for field in fields
        }
        candidate_mask = np.zeros(len(chunk), dtype=bool)
        for invalid in invalid_by_field.values():
            candidate_mask |= invalid
        finite_xy = np.isfinite(chunk["x"]) & np.isfinite(chunk["y"])
        report["rows_without_finite_coordinates"] += int(
            np.count_nonzero(candidate_mask & ~finite_xy)
        )
        local = np.flatnonzero(candidate_mask & finite_xy)
        if not len(local):
            continue
        query_points = np.column_stack(
            [chunk[coordinate][local] for coordinate in ("x", "y")]
        ).astype(np.float64, copy=False)
        distances, donor_indices = donor_table.tree.query(
            query_points,
            k=query_k,
            workers=options.workers,
        )
        if query_k == 1:
            distances = distances[:, None]
            donor_indices = donor_indices[:, None]
        nearest = distances[:, 0]
        bucket_index = np.searchsorted(buckets, nearest, side="left")
        eligible = bucket_index < len(buckets)
        safe_bucket_index = np.minimum(bucket_index, len(buckets) - 1)
        ceiling = buckets[safe_bucket_index]
        neighbour_mask = (
            np.isfinite(distances)
            & (donor_indices >= 0)
            & (donor_indices < donor_count)
            & (distances <= ceiling[:, None])
        )
        safe_indices = np.where(neighbour_mask, donor_indices, 0)
        weights = np.where(
            neighbour_mask,
            1.0 / np.maximum(distances, idw_floor) ** 2,
            0.0,
        )
        weight_sum = np.sum(weights, axis=1)
        eligible &= weight_sum > 0.0
        report["rows_beyond_max_bucket_or_without_donor"] += int(
            np.count_nonzero(~eligible)
        )
        if np.any(eligible):
            eligible_rows = np.flatnonzero(eligible)
            target_local = local[eligible_rows]
            for field in fields:
                field_invalid = invalid_by_field[field][target_local]
                if not np.any(field_invalid):
                    continue
                donor_values = donor_table.values[field][safe_indices]
                blended = np.sum(donor_values * weights, axis=1) / np.maximum(
                    weight_sum, 1e-300
                )
                low, high = FIELD_BOUNDS[field]
                blended = np.clip(blended, low, high)
                chunk[field][target_local[field_invalid]] = blended[eligible_rows][
                    field_invalid
                ]
                report["filled_by_field"][field] += int(
                    np.count_nonzero(field_invalid)
                )
            for index, label in enumerate(labels):
                report["filled_rows_by_bucket"][label] += int(
                    np.count_nonzero(bucket_index[eligible] == index)
                )
            nearest_origins = donor_table.origins[
                donor_indices[eligible, 0]
            ]
            report["nearest_origin_for_filled_rows"]["source"] += int(
                np.count_nonzero(nearest_origins == 0)
            )
            report["nearest_origin_for_filled_rows"]["fallback"] += int(
                np.count_nonzero(nearest_origins == 1)
            )

        sequence = sample_seen + np.arange(len(local), dtype=np.int64)
        sampled = (sequence % sample_stride) == 0
        sample_seen += len(local)
        sampled &= np.isfinite(nearest)
        if np.any(sampled):
            distance_samples.append(nearest[sampled])
            nearest_donor = donor_indices[sampled, 0]
            target_z = chunk["z"][local[sampled]].astype(np.float64)
            donor_z = donor_table.coordinates[nearest_donor, 2]
            finite_z = np.isfinite(target_z) & np.isfinite(donor_z)
            if np.any(finite_z):
                z_samples.append(np.abs(target_z[finite_z] - donor_z[finite_z]))
    records.flush()
    del records
    filled_rows = sum(report["filled_rows_by_bucket"].values())
    report["remaining_candidate_rows"] = report["candidate_rows"] - filled_rows
    report["remaining_nonfinite_by_field"] = {
        field: report["invalid_before_by_field"][field]
        - report["filled_by_field"][field]
        for field in fields
    }
    report["nearest_xy_distance_sample_quantiles_m"] = _sample_quantiles(
        distance_samples
    )
    report["nearest_abs_z_separation_sample_quantiles_m"] = _sample_quantiles(
        z_samples
    )
    report["distance_sample_count"] = int(
        sum(len(values) for values in distance_samples)
    )
    return report


def _remaining_audit(report: Mapping) -> dict:
    return {
        "candidate_rows": int(report["remaining_candidate_rows"]),
        "invalid_by_field": dict(report["remaining_nonfinite_by_field"]),
    }


def _merge_xy_report(group_report: dict, xy_report: Mapping) -> None:
    group_report["xy_fallback"] = dict(xy_report)
    for field, count in xy_report["filled_by_field"].items():
        group_report["filled_by_field"][field] += int(count)
    group_report["remaining_nonfinite_by_field"] = dict(
        xy_report["remaining_nonfinite_by_field"]
    )
    group_report["remaining_candidate_rows"] = int(
        xy_report["remaining_candidate_rows"]
    )


def _normalization_floor(metric: str, nominal_spacing: float) -> float:
    return 1.0e-6 if metric in ("MeanCurvature", "CrossCurvature") else nominal_spacing


def _validated_derived_normalization(
    normalization: Mapping[str, Mapping[str, float]],
    *,
    nominal_spacing: float,
) -> dict[str, dict[str, float]]:
    expected_metrics = set(METRICS)
    if set(normalization) != expected_metrics:
        raise ValueError(
            "derived normalization must contain exactly "
            + ", ".join(METRICS)
        )
    result: dict[str, dict[str, float]] = {}
    for metric in METRICS:
        supplied = normalization[metric]
        if set(supplied) != set(DERIVED_SCALES):
            raise ValueError(
                f"{metric} normalization must contain exactly "
                + ", ".join(DERIVED_SCALES)
            )
        floor = _normalization_floor(metric, nominal_spacing)
        result[metric] = {}
        for scale in DERIVED_SCALES:
            value = float(supplied[scale])
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError("derived normalization values must be finite and positive")
            result[metric][scale] = max(value, floor)
    return result


def derived_combination_policy(options: RepairOptions, spacing: str) -> dict:
    if spacing not in NOMINAL_SPACING_M:
        raise ValueError(f"spacing must be one of {SPACINGS}")
    nominal_spacing = NOMINAL_SPACING_M[spacing]
    provided = (
        _validated_derived_normalization(
            options.derived_normalization,
            nominal_spacing=nominal_spacing,
        )
        if options.derived_normalization is not None
        else None
    )
    return {
        "algorithm": DERIVED_COMBINATION_ALGORITHM,
        "weights": {
            scale: float(weight)
            for scale, weight in zip(DERIVED_SCALES, options.derived_weights)
        },
        "normalization": {
            "mode": "provided" if provided is not None else "finite_source_abs_p95",
            "percentile": float(options.derived_normalization_percentile),
            "sample_limit_per_field": int(
                options.derived_normalization_sample_limit
            ),
            "nominal_spacing_m": nominal_spacing,
            "provided_values": provided,
        },
        "signed_clip": [-1.0, 1.0],
        "positive_clip": [0.0, 1.0],
        "finite_scale_weights_renormalized": True,
        "roughness_relative": "fine / max(medium, 1e-9), output guard [0,8]",
    }


def _component_finite_counts(
    source: PlyInfo,
    chunk_size: int,
) -> dict[str, int]:
    fields = [
        f"scalar_A_R_{metric}_{scale}"
        for metric in METRICS
        for scale in DERIVED_SCALES
    ]
    present = [field for field in fields if field in source.dtype.names]
    counts = {field: 0 for field in present}
    records = _records(source)
    for span in _slices(source.count, chunk_size):
        chunk = records[span]
        for field in present:
            counts[field] += int(np.count_nonzero(np.isfinite(chunk[field])))
    del records
    return counts


def _systematic_abs_samples(
    source: PlyInfo,
    finite_counts: Mapping[str, int],
    *,
    sample_limit: int,
    chunk_size: int,
) -> dict[str, np.ndarray]:
    plans: dict[str, dict] = {}
    for field, finite_count_value in finite_counts.items():
        finite_count = int(finite_count_value)
        sample_count = min(finite_count, sample_limit)
        if sample_count == 0:
            ranks = np.empty(0, dtype=np.uint64)
        elif sample_count == 1:
            ranks = np.zeros(1, dtype=np.uint64)
        else:
            indices = np.arange(sample_count, dtype=np.uint64)
            ranks = (
                indices * np.uint64(finite_count - 1)
            ) // np.uint64(sample_count - 1)
        plans[field] = {
            "ranks": ranks,
            "values": np.empty(sample_count, dtype=np.float32),
            "seen": 0,
        }
    records = _records(source)
    for span in _slices(source.count, chunk_size):
        chunk = records[span]
        for field, plan in plans.items():
            raw = chunk[field]
            finite = raw[np.isfinite(raw)]
            lower = int(plan["seen"])
            upper = lower + len(finite)
            ranks = plan["ranks"]
            left = int(np.searchsorted(ranks, lower, side="left"))
            right = int(np.searchsorted(ranks, upper, side="left"))
            if right > left:
                local = (ranks[left:right] - np.uint64(lower)).astype(np.intp)
                plan["values"][left:right] = np.abs(finite[local])
            plan["seen"] = upper
    del records
    for field, plan in plans.items():
        if int(plan["seen"]) != int(finite_counts[field]):
            raise RuntimeError(f"finite source count changed while sampling {field}")
    return {field: plan["values"] for field, plan in plans.items()}


def _nearest_lower_percentile(
    values: np.ndarray,
    fraction: float,
    fallback: float,
) -> float:
    if not len(values):
        return fallback
    index = min(len(values) - 1, int(math.floor(fraction * (len(values) - 1))))
    value = float(np.partition(values, index)[index])
    return max(value, fallback)


def infer_derived_normalization(
    source: PlyInfo | Path | str,
    spacing: str,
    options: RepairOptions,
    *,
    finite_counts: Mapping[str, int] | None = None,
) -> tuple[dict[str, dict[str, float]], dict]:
    """Infer CleanMesh-style per-field p95 scales from finite source rows."""

    if spacing not in NOMINAL_SPACING_M:
        raise ValueError(f"spacing must be one of {SPACINGS}")
    source_info = (
        source if isinstance(source, PlyInfo) else inspect_fixed_stride_ply(source)
    )
    nominal_spacing = NOMINAL_SPACING_M[spacing]
    if options.derived_normalization is not None:
        normalization = _validated_derived_normalization(
            options.derived_normalization,
            nominal_spacing=nominal_spacing,
        )
        return normalization, {
            "method": "provided",
            "values": normalization,
            "finite_source_count_by_field": None,
            "sampled_count_by_field": None,
        }
    counts = (
        dict(finite_counts)
        if finite_counts is not None
        else _component_finite_counts(source_info, options.chunk_size)
    )
    fields = [
        f"scalar_A_R_{metric}_{scale}"
        for metric in METRICS
        for scale in DERIVED_SCALES
    ]
    missing = sorted(set(fields) - set(counts))
    if missing:
        raise RuntimeError(f"normalization source is missing component fields {missing}")
    samples = _systematic_abs_samples(
        source_info,
        {field: counts[field] for field in fields},
        sample_limit=options.derived_normalization_sample_limit,
        chunk_size=options.chunk_size,
    )
    normalization: dict[str, dict[str, float]] = {}
    for metric in METRICS:
        fallback = _normalization_floor(metric, nominal_spacing)
        normalization[metric] = {
            scale: _nearest_lower_percentile(
                samples[f"scalar_A_R_{metric}_{scale}"],
                options.derived_normalization_percentile,
                fallback,
            )
            for scale in DERIVED_SCALES
        }
    return normalization, {
        "method": "deterministic_systematic_finite_source_abs_percentile",
        "percentile": float(options.derived_normalization_percentile),
        "sample_limit_per_field": int(options.derived_normalization_sample_limit),
        "values": normalization,
        "finite_source_count_by_field": {
            field: int(counts[field]) for field in fields
        },
        "sampled_count_by_field": {
            field: int(len(samples[field])) for field in fields
        },
    }


def _derived_value(
    name: str,
    chunk: np.ndarray,
    normalization: Mapping[str, Mapping[str, float]] | None,
    weights: Sequence[float],
) -> tuple[np.ndarray, np.ndarray]:
    if name == "scalar_A_R_RoughnessRelative_FineMedium":
        fine = chunk["scalar_A_R_Roughness_Fine"].astype(np.float64)
        medium = chunk["scalar_A_R_Roughness_Medium"].astype(np.float64)
        valid = np.isfinite(fine) & np.isfinite(medium)
        value = fine / np.maximum(medium, 1.0e-9)
        return np.clip(value, 0.0, 8.0), valid
    metric = name.removeprefix("scalar_A_R_").removesuffix("_Combined")
    if metric not in METRICS or normalization is None:
        raise KeyError(name)
    values = np.column_stack(
        [
            chunk[f"scalar_A_R_{metric}_{scale}"].astype(np.float64)
            for scale in DERIVED_SCALES
        ]
    )
    finite = np.isfinite(values)
    scales = np.asarray(
        [normalization[metric][scale] for scale in DERIVED_SCALES],
        dtype=np.float64,
    )
    lower = 0.0 if metric == "Roughness" else -1.0
    normalized = np.clip(values / scales[None, :], lower, 1.0)
    scale_weights = np.asarray(weights, dtype=np.float64)
    used_weights = np.sum(finite * scale_weights[None, :], axis=1)
    weighted = np.sum(
        np.where(finite, normalized, 0.0) * scale_weights[None, :],
        axis=1,
    )
    valid = used_weights > 0.0
    result = np.full(len(chunk), np.nan, dtype=np.float64)
    result[valid] = weighted[valid] / used_weights[valid]
    return result, valid


def _derived_dependencies_present(dtype: np.dtype, name: str) -> bool:
    names = set(dtype.names)
    if "RoughnessRelative" in name:
        dependencies = (
            "scalar_A_R_Roughness_Fine",
            "scalar_A_R_Roughness_Medium",
        )
    else:
        metric = name.removeprefix("scalar_A_R_").removesuffix("_Combined")
        dependencies = tuple(
            f"scalar_A_R_{metric}_{scale}" for scale in ("Fine", "Medium", "Broad")
        )
    return all(field in names for field in dependencies)


def _recompute_invalid_derived(
    source: PlyInfo,
    candidate: PlyInfo,
    spacing: str,
    options: RepairOptions,
    invalid_audit: Mapping[str, int],
    component_audit: Mapping[str, Mapping],
) -> dict:
    records = _records(candidate, mode="r+")
    all_present = [
        field
        for field in DERIVED_FIELDS
        if field in candidate.dtype.names and _derived_dependencies_present(candidate.dtype, field)
    ]
    present = [field for field in all_present if invalid_audit.get(field, 0) > 0]
    combination_fields = [
        field for field in present if field.endswith("_Combined")
    ]
    normalization = None
    normalization_report = {"method": "not_needed", "values": None}
    if combination_fields:
        finite_counts = {
            field: source.count
            - int(component_audit[scale]["invalid_by_field"][field])
            for scale in DERIVED_SCALES
            for field in COMPONENT_GROUPS[scale]
        }
        normalization, normalization_report = infer_derived_normalization(
            source,
            spacing,
            options,
            finite_counts=finite_counts,
        )
    report = {
        "combination": {
            **derived_combination_policy(options, spacing),
            "inference": normalization_report,
        },
        "invalid_before_by_field": {
            field: int(invalid_audit.get(field, 0)) for field in all_present
        },
        "recomputed_by_field": {field: 0 for field in all_present},
        "remaining_nonfinite_by_field": {
            field: int(invalid_audit.get(field, 0)) for field in all_present
        },
    }
    if not present:
        del records
        return report
    for span in _slices(candidate.count, options.chunk_size):
        chunk = records[span]
        for field in present:
            invalid = ~np.isfinite(chunk[field])
            if not np.any(invalid):
                continue
            values, dependencies_valid = _derived_value(
                field,
                chunk,
                normalization,
                options.derived_weights,
            )
            target = invalid & dependencies_valid & np.isfinite(values)
            if not np.any(target):
                continue
            low, high = FIELD_BOUNDS[field]
            chunk[field][target] = np.clip(values[target], low, high)
            report["recomputed_by_field"][field] += int(np.count_nonzero(target))
    records.flush()
    del records
    report["remaining_nonfinite_by_field"] = {
        field: report["invalid_before_by_field"][field]
        - report["recomputed_by_field"][field]
        for field in all_present
    }
    return report


def _raw_equal(left: np.ndarray, right: np.ndarray) -> bool:
    left_bytes = np.ascontiguousarray(left).view(np.uint8)
    right_bytes = np.ascontiguousarray(right).view(np.uint8)
    return bool(np.array_equal(left_bytes, right_bytes))


def verify_repair(
    source_path: Path | str,
    candidate_path: Path | str,
    chunk_size: int = 300_000,
) -> dict:
    """Fully verify layout, byte preservation, changed-value scope and bounds."""

    source = inspect_fixed_stride_ply(source_path)
    candidate = inspect_fixed_stride_ply(candidate_path)
    layout_identical = (
        source.dtype == candidate.dtype
        and source.count == candidate.count
        and source.offset == candidate.offset
        and source.header == candidate.header
        and source.file_size == candidate.file_size
    )
    if not layout_identical:
        return {"verified": False, "layout_identical": False}
    repairable = set(REPAIRABLE_FIELDS) & set(source.dtype.names)
    source_records = _records(source)
    candidate_records = _records(candidate)
    finite_source_preserved = True
    unrelated_fields_preserved = True
    unfilled_invalid_preserved = True
    repaired_within_bounds = True
    changed_by_field = {field: 0 for field in repairable}
    remaining_nonfinite_by_field = {field: 0 for field in repairable}
    for span in _slices(source.count, chunk_size):
        before = source_records[span]
        after = candidate_records[span]
        for field in source.dtype.names:
            if field not in repairable:
                unrelated_fields_preserved &= _raw_equal(before[field], after[field])
                continue
            finite_before = np.isfinite(before[field])
            finite_after = np.isfinite(after[field])
            if np.any(finite_before):
                finite_source_preserved &= _raw_equal(
                    before[field][finite_before], after[field][finite_before]
                )
            source_invalid = ~finite_before
            still_invalid = source_invalid & ~finite_after
            if np.any(still_invalid):
                unfilled_invalid_preserved &= _raw_equal(
                    before[field][still_invalid], after[field][still_invalid]
                )
            repaired = source_invalid & finite_after
            changed_by_field[field] += int(np.count_nonzero(repaired))
            remaining_nonfinite_by_field[field] += int(np.count_nonzero(~finite_after))
            if np.any(repaired):
                low, high = FIELD_BOUNDS[field]
                repaired_values = after[field][repaired]
                repaired_within_bounds &= bool(
                    np.all((repaired_values >= low) & (repaired_values <= high))
                )
    del source_records, candidate_records
    verified = all(
        (
            layout_identical,
            finite_source_preserved,
            unrelated_fields_preserved,
            unfilled_invalid_preserved,
            repaired_within_bounds,
        )
    )
    return {
        "verified": verified,
        "layout_identical": layout_identical,
        "finite_source_values_byte_identical": finite_source_preserved,
        "unrelated_fields_byte_identical": unrelated_fields_preserved,
        "unfilled_invalid_payloads_byte_identical": unfilled_invalid_preserved,
        "repaired_values_within_hard_bounds": repaired_within_bounds,
        "changed_by_field": dict(sorted(changed_by_field.items())),
        "remaining_nonfinite_by_field": dict(sorted(remaining_nonfinite_by_field.items())),
    }


def _clone_or_copy(source: Path, destination: Path, prefer_clone: bool) -> str:
    if prefer_clone and sys.platform == "darwin":
        completed = subprocess.run(
            ["cp", "-c", str(source), str(destination)],
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        if completed.returncode == 0:
            return "apfs-clone"
    shutil.copyfile(source, destination)
    return "full-copy"


def repair_scalar_file(
    source_path: Path | str,
    output_path: Path | str,
    *,
    role: str,
    spacing: str,
    fallback_5mm_path: Path | str | None = None,
    report_path: Path | str | None = None,
    options: RepairOptions | None = None,
) -> dict:
    """Build and verify a repaired candidate without modifying ``source_path``.

    ``fallback_5mm_path`` is intended for repaired 5 mm output of the same
    role when constructing a 1 mm candidate.  Cross-role fallback is rejected.
    """

    options = options or RepairOptions()
    options.validate()
    role = role.upper()
    if role not in REPAIR_ROLES:
        raise ValueError(f"role must be one of {REPAIR_ROLES}")
    if spacing not in SPACINGS:
        raise ValueError(f"spacing must be one of {SPACINGS}")
    source_path = Path(source_path).resolve()
    output_path = Path(output_path).resolve()
    if source_path == output_path:
        raise ValueError("output must not be the source/canonical path")
    if is_canonical_output(output_path):
        raise ValueError("output must be a staged candidate, not a canonical cloud name")
    _validate_role(source_path, role)
    source = inspect_fixed_stride_ply(source_path)
    groups = _validate_schema(source)
    fallback = None
    if fallback_5mm_path is not None:
        if spacing != "1mm":
            raise ValueError("5 mm fallback is only accepted for a 1 mm repair")
        fallback_path = Path(fallback_5mm_path).resolve()
        _validate_role(fallback_path, role)
        fallback = inspect_fixed_stride_ply(fallback_path)
        fallback_groups = _validate_schema(fallback)
        missing_groups = sorted(set(groups) - set(fallback_groups))
        if missing_groups:
            raise RuntimeError(f"fallback is missing scalar groups {missing_groups}")
    audit = audit_nonfinite(source, groups, options.chunk_size)
    if output_path.exists() and not options.overwrite:
        raise FileExistsError(f"candidate already exists: {output_path}")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    temporary = output_path.with_name(
        f".{output_path.name}.{os.getpid()}.{uuid.uuid4().hex}.partial"
    )
    source_stat_before = source_path.stat()
    try:
        copy_method = _clone_or_copy(source_path, temporary, options.prefer_clone)
        temporary_info = inspect_fixed_stride_ply(temporary)
        directional_direct = None
        if (
            "Directional" in groups
            and audit["groups"]["Directional"]["candidate_rows"] > 0
        ):
            directional_direct = _repair_directional_from_normals(
                temporary_info,
                audit["groups"]["Directional"],
                options,
            )

        # Directional fields use each point's own normal first and never make
        # the expensive joint 3-D donor catalogue more restrictive.
        active_3d_groups = {
            group: fields
            for group, fields in groups.items()
            if group != "Directional"
            and audit["groups"][group]["candidate_rows"] > 0
        }
        joint_fields = tuple(
            dict.fromkeys(
                field
                for fields in active_3d_groups.values()
                for field in fields
            )
        )
        donor_table = (
            _build_donor_table(source, fallback, joint_fields, options)
            if joint_fields
            else None
        )
        group_reports = _fill_groups_joint(
            temporary_info,
            active_3d_groups,
            audit["groups"],
            donor_table,
            options,
            spacing,
        )
        del donor_table
        labels = _bucket_labels(options.distance_buckets_m)
        for group, fields in groups.items():
            if group in group_reports:
                continue
            report = _empty_group_report(
                fields,
                audit["groups"][group],
                None,
                labels,
            )
            if group == "Directional" and directional_direct is not None:
                report["filled_by_field"] = dict(
                    directional_direct["filled_by_field"]
                )
                report["remaining_nonfinite_by_field"] = dict(
                    directional_direct["remaining_nonfinite_by_field"]
                )
                report["remaining_candidate_rows"] = int(
                    directional_direct["remaining_candidate_rows"]
                )
                report["direct_from_normals"] = directional_direct
                report["joint_three_dimensional_transfer_skipped"] = (
                    "directional fields use a separate post-normal 3-D donor pass"
                )
            else:
                report["skipped"] = "no_nonfinite_values"
            group_reports[group] = report

        # Rows whose normals are absent or zero cannot be derived directly.
        # Preserve the former local 3-D behavior for only those remaining
        # Directional rows, using a group-specific donor catalogue rather
        # than reintroducing Directional into the joint component bundle.
        if "Directional" in groups:
            directional_report = group_reports["Directional"]
            directional_remaining = _remaining_audit(directional_report)
            if directional_remaining["candidate_rows"]:
                fields = groups["Directional"]
                directional_donors = _build_donor_table(
                    source,
                    fallback,
                    fields,
                    options,
                )
                post_normal_3d = _fill_groups_joint(
                    temporary_info,
                    {"Directional": fields},
                    {"Directional": directional_remaining},
                    directional_donors,
                    options,
                    spacing,
                )["Directional"]
                directional_report["group_specific_3d_after_normals"] = (
                    post_normal_3d
                )
                for field, count in post_normal_3d["filled_by_field"].items():
                    directional_report["filled_by_field"][field] += int(count)
                directional_report["filled_rows_by_bucket"] = dict(
                    post_normal_3d["filled_rows_by_bucket"]
                )
                directional_report["remaining_nonfinite_by_field"] = dict(
                    post_normal_3d["remaining_nonfinite_by_field"]
                )
                directional_report["remaining_candidate_rows"] = int(
                    post_normal_3d["remaining_candidate_rows"]
                )
                for key in (
                    "rows_without_finite_coordinates",
                    "rows_beyond_max_bucket_or_without_donor",
                    "nearest_origin_for_filled_rows",
                    "donors",
                    "sampling",
                ):
                    directional_report[key] = post_normal_3d[key]
                del directional_donors

        # Remaining gaps get a same-role, group-specific XY catalogue.  Each
        # donor therefore needs only this group's values, not every active
        # group's bundle, which both increases locality and avoids coupling
        # unrelated undefined masks.
        for group, fields in groups.items():
            group_report = group_reports[group]
            remaining = _remaining_audit(group_report)
            if not remaining["candidate_rows"]:
                continue
            xy_donors = (
                _build_donor_table(
                    source,
                    fallback,
                    fields,
                    options,
                    dimensions=2,
                )
                if options.xy_fallback_buckets_m
                else None
            )
            xy_report = _fill_group_xy(
                temporary_info,
                fields,
                remaining,
                xy_donors,
                options,
                spacing,
            )
            _merge_xy_report(group_report, xy_report)
            del xy_donors
        derived_report = _recompute_invalid_derived(
            source,
            temporary_info,
            spacing,
            options,
            audit["derived"],
            audit["groups"],
        )
        verification = (
            verify_repair(source_path, temporary, options.chunk_size)
            if options.verify
            else {"verified": None, "skipped": True}
        )
        if options.verify and not verification["verified"]:
            raise RuntimeError("scalar candidate failed full byte-preservation verification")
        source_stat_after = source_path.stat()
        source_snapshot_unchanged = (
            source_stat_before.st_size == source_stat_after.st_size
            and source_stat_before.st_mtime_ns == source_stat_after.st_mtime_ns
        )
        if not source_snapshot_unchanged:
            raise RuntimeError("source changed while scalar repair was running")
        report = {
            "schema_version": 3,
            "source": str(source_path),
            "output": str(output_path),
            "role": role,
            "spacing": spacing,
            "fallback_5mm": str(fallback.path) if fallback is not None else None,
            "point_count": source.count,
            "record_stride_bytes": source.dtype.itemsize,
            "copy_method": copy_method,
            "distance_buckets_mm": [distance * 1000.0 for distance in options.distance_buckets_m],
            "xy_fallback_buckets_mm": [
                distance * 1000.0
                for distance in options.xy_fallback_buckets_m
            ],
            "idw_power": 2.0,
            "resource_options": {
                "chunk_size": options.chunk_size,
                "max_donors": options.max_donors_per_group,
                "query_k": options.donor_query_k,
                "fallback_share": options.fallback_share,
                "workers": options.workers,
            },
            "hard_bounds": {
                field: list(FIELD_BOUNDS[field])
                for field in REPAIRABLE_FIELDS
                if field in source.dtype.names
            },
            "joint_complete_donor_bundle": list(joint_fields),
            "groups": group_reports,
            "derived": derived_report,
            "source_snapshot_unchanged": source_snapshot_unchanged,
            "verification": verification,
        }
        os.replace(temporary, output_path)
        if report_path is not None:
            report_path = Path(report_path)
            report_path.parent.mkdir(parents=True, exist_ok=True)
            report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
        return report
    finally:
        if temporary.exists():
            temporary.unlink()


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--role", required=True, choices=REPAIR_ROLES)
    parser.add_argument("--spacing", required=True, choices=SPACINGS)
    parser.add_argument("--fallback-5mm", type=Path)
    parser.add_argument("--report", type=Path)
    parser.add_argument("--chunk-size", type=int, default=RepairOptions.chunk_size)
    parser.add_argument(
        "--max-donors", type=int, default=RepairOptions.max_donors_per_group
    )
    parser.add_argument("--query-k", type=int, default=RepairOptions.donor_query_k)
    parser.add_argument("--workers", type=int, default=RepairOptions.workers)
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--no-clone", action="store_true")
    parser.add_argument("--no-verify", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    options = RepairOptions(
        chunk_size=args.chunk_size,
        max_donors_per_group=args.max_donors,
        donor_query_k=args.query_k,
        workers=args.workers,
        overwrite=args.overwrite,
        prefer_clone=not args.no_clone,
        verify=not args.no_verify,
    )
    report = repair_scalar_file(
        args.source,
        args.output,
        role=args.role,
        spacing=args.spacing,
        fallback_5mm_path=args.fallback_5mm,
        report_path=args.report,
        options=options,
    )
    summary = {
        "output": report["output"],
        "point_count": report["point_count"],
        "verified": report["verification"].get("verified"),
        "remaining_nonfinite": {
            field: count
            for field, count in report["verification"].get(
                "remaining_nonfinite_by_field", {}
            ).items()
            if count
        },
    }
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
