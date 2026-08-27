#!/usr/bin/env python3
"""Candidate-only CleanMesh enrichment for appended Scene1 WATER geometry.

The southern hole builder intentionally creates complete 147-byte WATER rows
by copying a nearby WATER donor and replacing its geometry.  That is the safe
way to construct a reversible geometry candidate, but the donor's ``A_R``
geometry fields describe the donor location.  This stage replaces only those
geometry-derived scalar fields with a local CleanMesh analysis.

The contract is deliberately strict:

* the geometry candidate must be exactly ``base payload + archived records``;
* only a measured SAND/ROCK and existing WATER collar is analysed;
* WATER is presented to CleanMesh as TypeID 1, while the appended rows are
  temporarily tagged ScanID 10 so they can be recovered after tiled output;
* coordinates, colour, normals, intensity, composition, visibility and every
  other non-geometric field are restored byte-for-byte from the archive;
* combined geometry fields use the hash-locked v10 global normalisations;
* the final suffix is restored to WATER ScanID 999;
* 5 mm rows take geometry fields from their exact enriched 2 mm source rows;
  the under-supported direct 5 mm CleanMesh result is retained as a diagnostic;
* the existing base payload is copied byte-for-byte and no canonical path is
  ever written.

The API accepts arbitrary base/candidate/manifest/archive paths.  In
particular, it does not depend on a specific coarse down-sampling count or
hash.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys
import tempfile
from typing import Callable, Iterable, Mapping, Sequence

import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import site1_v11_hole_pipeline as hole_pipeline  # noqa: E402
import site1_v11_terrain as terrain  # noqa: E402
import site1_v11_terrain_pipeline as terrain_pipeline  # noqa: E402
import site1_v11_water_density as density  # noqa: E402


WATER_SCAN_ID = 999.0
TEMPORARY_ADDITION_SCAN_ID = terrain.ADDITION_SCAN_ID
WATER_ANALYSIS_ROLE = "SAND"  # CleanMesh TypeID 1, as requested.
VISIBILITY_FIELDS = frozenset(terrain_pipeline.LOCAL_VISIBILITY_FIELDS)
PHYSICAL_METRICS = terrain_pipeline.PHYSICAL_METRICS
DERIVED_SCALES = terrain_pipeline.DERIVED_SCALES
MINIMUM_COMBINED_FINITE_FRACTION = 1.0
MINIMUM_COMPONENT_FIELD_FINITE_FRACTION = 1.0
SCALAR_FALLBACK_RADII_M = (0.02, 0.04, 0.08, 0.12, 0.24, 0.48, 0.96, 8.0)
SCALAR_FALLBACK_COMPONENT_DIAMETER_BOUND_M = 8.0
SCALAR_FALLBACK_MAX_NEIGHBOURS = 16
SCALAR_FALLBACK_MIN_NEIGHBOURS = 3
SCALAR_FALLBACK_TINY_COMPONENT_MAX_POINTS = 64


@dataclass(frozen=True)
class FileFingerprint:
    path: str
    size_bytes: int
    mtime_ns: int
    sha256: str


@dataclass(frozen=True)
class GeometryContract:
    source_points: int
    base_points: int
    removed_base_count: int
    candidate_points: int
    addition_count: int
    base_sha256: str
    candidate_sha256: str
    manifest_sha256: str
    archive_sha256: str
    base_payload_sha256: str
    candidate_prefix_payload_sha256: str
    archive_records_sha256: str
    candidate_suffix_sha256: str
    config_fingerprint: Mapping[str, object] | None


@dataclass(frozen=True)
class EnrichmentResult:
    output_dir: Path
    candidate_path: Path
    manifest_path: Path
    candidate_points: int
    addition_count: int
    candidate_sha256: str


CleanMeshRunner = Callable[
    [
        Path,
        Path,
        Path,
        Path,
        terrain_pipeline.ResolutionParameters,
        terrain_pipeline.PipelineParameters,
    ],
    Mapping[str, object],
]


def sha256_path(path: str | Path, *, block_size: int = 32 * 1024 * 1024) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        while block := handle.read(block_size):
            digest.update(block)
    return digest.hexdigest()


def fingerprint_file(path: str | Path) -> FileFingerprint:
    source = Path(path).resolve(strict=True)
    stat = source.stat()
    return FileFingerprint(
        path=str(source),
        size_bytes=stat.st_size,
        mtime_ns=stat.st_mtime_ns,
        sha256=sha256_path(source),
    )


def assert_fingerprint_unchanged(fingerprint: FileFingerprint) -> None:
    path = Path(fingerprint.path)
    stat = path.stat()
    if stat.st_size != fingerprint.size_bytes or stat.st_mtime_ns != fingerprint.mtime_ns:
        raise RuntimeError(f"hash-locked input stat changed: {path}")
    if sha256_path(path) != fingerprint.sha256:
        raise RuntimeError(f"hash-locked input content changed: {path}")


def _records_sha256(records: np.ndarray) -> str:
    digest = hashlib.sha256()
    digest.update(np.asarray(records).tobytes(order="C"))
    return digest.hexdigest()


def _payload_sha256(
    layout: terrain.PlyLayout,
    *,
    start: int = 0,
    count: int | None = None,
    chunk_records: int = 1_000_000,
) -> str:
    if start < 0 or start > layout.vertex_count:
        raise ValueError("payload hash start lies outside PLY")
    available = layout.vertex_count - start
    length = available if count is None else int(count)
    if length < 0 or length > available:
        raise ValueError("payload hash count lies outside PLY")
    digest = hashlib.sha256()
    with layout.path.open("rb") as handle:
        handle.seek(layout.offset + start * layout.dtype.itemsize)
        remaining = length * layout.dtype.itemsize
        block_bytes = max(layout.dtype.itemsize, chunk_records * layout.dtype.itemsize)
        while remaining:
            block = handle.read(min(remaining, block_bytes))
            if not block:
                raise RuntimeError(f"unexpected EOF while hashing {layout.path}")
            digest.update(block)
            remaining -= len(block)
    return digest.hexdigest()


def _manifest_entry(document: Mapping[str, object], key: str) -> Mapping[str, object]:
    value = document.get(key)
    if not isinstance(value, Mapping):
        raise ValueError(f"geometry manifest lacks mapping {key!r}")
    return value


def _verify_declared_file(
    declared: Mapping[str, object],
    actual_path: Path,
    actual_hash: str,
    *,
    label: str,
) -> None:
    expected_hash = declared.get("sha256")
    if not isinstance(expected_hash, str) or expected_hash != actual_hash:
        raise RuntimeError(f"geometry manifest {label} hash does not match supplied file")
    declared_path = declared.get("path")
    if declared_path is not None:
        # Paths are provenance, not identity: a verified artifact may be moved.
        Path(str(declared_path))
    points = declared.get("points")
    if points is not None and int(points) < 0:
        raise ValueError(f"geometry manifest {label} point count is invalid")


def _verify_referenced_config(
    document: Mapping[str, object],
) -> Mapping[str, object] | None:
    value = document.get("config")
    if value is None:
        return None
    if not isinstance(value, Mapping):
        raise ValueError("geometry manifest config must be a mapping")
    path_value = value.get("path")
    expected_hash = value.get("sha256")
    if not isinstance(path_value, str) or not isinstance(expected_hash, str):
        raise ValueError("geometry manifest config fingerprint is incomplete")
    path = Path(path_value).resolve(strict=True)
    actual_hash = sha256_path(path)
    if actual_hash != expected_hash:
        raise RuntimeError("geometry manifest review config has drifted")
    stat = path.stat()
    if "size_bytes" in value and int(value["size_bytes"]) != stat.st_size:
        raise RuntimeError("geometry manifest review config size has drifted")
    if "mtime_ns" in value and int(value["mtime_ns"]) != stat.st_mtime_ns:
        raise RuntimeError("geometry manifest review config mtime has drifted")
    return {
        "path": str(path),
        "sha256": actual_hash,
        "size_bytes": stat.st_size,
        "mtime_ns": stat.st_mtime_ns,
    }


def _strict_archive_fingerprint(
    block: object,
    *,
    label: str,
) -> Path:
    if not isinstance(block, Mapping):
        raise ValueError(f"geometry {label} fingerprint is missing")
    path_value = block.get("path")
    if not isinstance(path_value, str):
        raise ValueError(f"geometry {label} path is missing")
    path = Path(path_value).resolve(strict=True)
    stat = path.stat()
    if int(block.get("size_bytes", -1)) != stat.st_size:
        raise RuntimeError(f"geometry {label} byte size has drifted")
    if int(block.get("mtime_ns", -1)) != stat.st_mtime_ns:
        raise RuntimeError(f"geometry {label} mtime has drifted")
    if block.get("sha256") != sha256_path(path):
        raise RuntimeError(f"geometry {label} hash has drifted")
    return path


def _verify_reversible_base_cull(
    document: Mapping[str, object],
    base_layout: terrain.PlyLayout,
    candidate_layout: terrain.PlyLayout,
    *,
    chunk_records: int,
) -> tuple[int, str]:
    """Verify and replay an optional exact-row base cull as a subsequence."""

    raw = document.get("far_lobe_cull")
    if raw is None:
        prefix_hash = _payload_sha256(
            candidate_layout,
            count=base_layout.vertex_count,
            chunk_records=chunk_records,
        )
        base_hash = _payload_sha256(base_layout, chunk_records=chunk_records)
        if prefix_hash != base_hash:
            raise RuntimeError("geometry candidate base payload is not byte-exact")
        return 0, prefix_hash
    if not isinstance(raw, Mapping) or raw.get("reversible") is not True:
        raise ValueError("geometry far-lobe decision lacks reversible provenance")
    performed = raw.get("performed") is True
    removed = int(raw.get("removed_count", -1))
    if not performed:
        if removed != 0 or raw.get("measured_no_eligible_component") is not True:
            raise RuntimeError(
                "unperformed far-lobe decision lacks measured no-component proof"
            )
        prefix_hash = _payload_sha256(
            candidate_layout,
            count=base_layout.vertex_count,
            chunk_records=chunk_records,
        )
        base_hash = _payload_sha256(base_layout, chunk_records=chunk_records)
        if prefix_hash != base_hash:
            raise RuntimeError("geometry candidate base payload is not byte-exact")
        return 0, prefix_hash
    if removed <= 0 or removed >= base_layout.vertex_count:
        raise RuntimeError("performed far-lobe cull has an invalid removed_count")
    archive = raw.get("archive")
    if not isinstance(archive, Mapping):
        raise ValueError("performed far-lobe cull lacks its exact archive")
    if archive.get("exact_source_rows_archived") is not True or archive.get(
        "exact_source_indices_archived"
    ) is not True:
        raise RuntimeError("far-lobe archive is not attested exact and reversible")
    if int(archive.get("removed_count", -1)) != removed:
        raise RuntimeError("far-lobe archive count disagrees with manifest")
    if int(archive.get("record_stride_bytes", -1)) != base_layout.dtype.itemsize:
        raise RuntimeError("far-lobe archive stride differs from WATER schema")
    if archive.get("source_index_dtype") != "<i8":
        raise RuntimeError("far-lobe source-index archive is not little-endian int64")
    record_path = _strict_archive_fingerprint(
        archive.get("records"), label="far-lobe record archive"
    )
    index_path = _strict_archive_fingerprint(
        archive.get("source_indices"), label="far-lobe index archive"
    )
    if record_path.stat().st_size != removed * base_layout.dtype.itemsize:
        raise RuntimeError("far-lobe record archive length is inconsistent")
    if index_path.stat().st_size != removed * np.dtype("<i8").itemsize:
        raise RuntimeError("far-lobe index archive length is inconsistent")
    indices = np.memmap(index_path, dtype="<i8", mode="r", shape=(removed,))
    archived = np.memmap(
        record_path, dtype=base_layout.dtype, mode="r", shape=(removed,)
    )
    if int(indices[0]) < 0 or int(indices[-1]) >= base_layout.vertex_count:
        raise RuntimeError("far-lobe source indices lie outside the base WATER")
    if np.any(indices[1:] <= indices[:-1]):
        raise RuntimeError("far-lobe source indices are not strictly increasing")
    candidate_prefix_count = base_layout.vertex_count - removed
    candidate_memory = np.memmap(
        candidate_layout.path,
        dtype=candidate_layout.dtype,
        mode="r",
        offset=candidate_layout.offset,
        shape=(candidate_layout.vertex_count,),
    )
    removed_cursor = 0
    surviving_cursor = 0
    try:
        for begin, records in terrain.iter_ply_chunks(
            base_layout, chunk_size=chunk_records
        ):
            end = begin + len(records)
            left = int(np.searchsorted(indices, begin, side="left"))
            right = int(np.searchsorted(indices, end, side="left"))
            local_removed = np.asarray(indices[left:right], np.int64) - begin
            if len(local_removed):
                observed_archive = np.asarray(archived[left:right])
                expected_archive = np.asarray(records[local_removed])
                if observed_archive.tobytes() != expected_archive.tobytes():
                    raise RuntimeError(
                        "far-lobe archived rows differ from their source indices"
                    )
            keep = np.ones(len(records), dtype=bool)
            keep[local_removed] = False
            expected_surviving = np.asarray(records[keep])
            observed_surviving = np.asarray(candidate_memory[
                surviving_cursor : surviving_cursor + len(expected_surviving)
            ])
            if observed_surviving.tobytes() != expected_surviving.tobytes():
                raise RuntimeError(
                    "geometry candidate is not the exact ordered surviving base"
                )
            removed_cursor = right
            surviving_cursor += len(expected_surviving)
        if removed_cursor != removed or surviving_cursor != candidate_prefix_count:
            raise RuntimeError("far-lobe replay did not consume the declared rows")
    finally:
        del candidate_memory
        del archived
        del indices
    prefix_hash = _payload_sha256(
        candidate_layout,
        count=candidate_prefix_count,
        chunk_records=chunk_records,
    )
    return removed, prefix_hash


def verify_reversible_base_cull_prefix(
    base_water_path: str | Path,
    candidate_path: str | Path,
    geometry_manifest: Mapping[str, object],
    *,
    chunk_records: int = 1_000_000,
) -> Mapping[str, object]:
    """Public, bounded proof of the surviving base prefix after a cull."""

    base_layout = terrain.inspect_fixed_stride_ply(base_water_path)
    candidate_layout = terrain.inspect_fixed_stride_ply(candidate_path)
    if base_layout.dtype != candidate_layout.dtype:
        raise ValueError("base WATER and candidate schemas differ")
    removed, prefix_hash = _verify_reversible_base_cull(
        geometry_manifest,
        base_layout,
        candidate_layout,
        chunk_records=chunk_records,
    )
    surviving = base_layout.vertex_count - removed
    raw = geometry_manifest.get("far_lobe_cull")
    if not isinstance(raw, Mapping):
        raise ValueError("geometry far-lobe decision is missing")
    if (
        int(raw.get("source_count_before", -1)) != base_layout.vertex_count
        or int(raw.get("surviving_source_count", -1)) != surviving
        or int(raw.get("removed_count", -1)) != removed
        or raw.get("surviving_source_payload_byte_exact") is not True
        or raw.get("surviving_source_row_order_preserved") is not True
    ):
        raise RuntimeError("geometry far-lobe surviving-prefix contract differs")
    return {
        "source_points": int(base_layout.vertex_count),
        "removed_base_count": int(removed),
        "surviving_base_points": int(surviving),
        "surviving_prefix_payload_sha256": prefix_hash,
        "surviving_source_payload_byte_exact": True,
        "surviving_source_row_order_preserved": True,
    }


def verify_append_only_geometry(
    *,
    base_water_path: str | Path,
    geometry_candidate_path: str | Path,
    geometry_manifest_path: str | Path,
    geometry_archive_path: str | Path,
    chunk_records: int = 1_000_000,
) -> tuple[GeometryContract, np.ndarray]:
    """Prove that the geometry input is exactly base + archived suffix."""

    base = Path(base_water_path).resolve(strict=True)
    candidate = Path(geometry_candidate_path).resolve(strict=True)
    manifest = Path(geometry_manifest_path).resolve(strict=True)
    archive = Path(geometry_archive_path).resolve(strict=True)
    base_layout = terrain.inspect_fixed_stride_ply(base)
    candidate_layout = terrain.inspect_fixed_stride_ply(candidate)
    if base_layout.dtype != candidate_layout.dtype:
        raise ValueError("base WATER and geometry candidate schemas differ")
    base_hash = sha256_path(base)
    candidate_hash = sha256_path(candidate)
    manifest_hash = sha256_path(manifest)
    archive_hash = sha256_path(archive)
    document = json.loads(manifest.read_text(encoding="utf-8"))
    if not isinstance(document, Mapping):
        raise ValueError("geometry manifest root must be an object")
    source_declared = _manifest_entry(document, "source")
    candidate_declared = _manifest_entry(document, "candidate")
    _verify_declared_file(source_declared, base, base_hash, label="source")
    _verify_declared_file(candidate_declared, candidate, candidate_hash, label="candidate")
    if "points" in source_declared and int(source_declared["points"]) != base_layout.vertex_count:
        raise RuntimeError("geometry manifest source point count disagrees with base")
    if (
        "points" in candidate_declared
        and int(candidate_declared["points"]) != candidate_layout.vertex_count
    ):
        raise RuntimeError("geometry manifest candidate point count disagrees with candidate")
    expected_archive_hash = document.get("archive_sha256")
    if not isinstance(expected_archive_hash, str) or expected_archive_hash != archive_hash:
        raise RuntimeError("geometry manifest archive hash does not match supplied archive")

    with np.load(archive, allow_pickle=False) as loaded:
        if "records" not in loaded.files:
            raise ValueError("geometry archive has no records array")
        archived = np.asarray(loaded["records"]).copy()
    if archived.ndim != 1 or archived.dtype != base_layout.dtype:
        raise ValueError("geometry archive records do not match WATER schema")
    removed_base_count, prefix_hash = _verify_reversible_base_cull(
        document,
        base_layout,
        candidate_layout,
        chunk_records=chunk_records,
    )
    surviving_base_points = base_layout.vertex_count - removed_base_count
    if candidate_layout.vertex_count < surviving_base_points:
        raise ValueError("geometry candidate is shorter than its surviving WATER base")
    addition_count = candidate_layout.vertex_count - surviving_base_points
    if len(archived) != addition_count:
        raise RuntimeError("geometry archive count does not match candidate suffix")
    if "addition_count" in document and int(document["addition_count"]) != addition_count:
        raise RuntimeError("geometry manifest addition_count disagrees with suffix")
    scan_name = "scalar_ScanID"
    if scan_name not in (archived.dtype.names or ()):
        raise ValueError("WATER schema has no scalar_ScanID")
    if len(archived) and not np.all(
        np.isfinite(archived[scan_name])
        & (archived[scan_name] == WATER_SCAN_ID)
    ):
        raise RuntimeError("geometry archive contains a non-WATER ScanID")

    base_payload_hash = prefix_hash
    archive_records_hash = _records_sha256(archived)
    suffix_hash = _payload_sha256(
        candidate_layout,
        start=surviving_base_points,
        count=addition_count,
        chunk_records=chunk_records,
    )
    if suffix_hash != archive_records_hash:
        raise RuntimeError("geometry candidate suffix differs from archived records")
    config_fingerprint = _verify_referenced_config(document)
    contract = GeometryContract(
        source_points=base_layout.vertex_count,
        base_points=surviving_base_points,
        removed_base_count=removed_base_count,
        candidate_points=candidate_layout.vertex_count,
        addition_count=addition_count,
        base_sha256=base_hash,
        candidate_sha256=candidate_hash,
        manifest_sha256=manifest_hash,
        archive_sha256=archive_hash,
        base_payload_sha256=base_payload_hash,
        candidate_prefix_payload_sha256=prefix_hash,
        archive_records_sha256=archive_records_hash,
        candidate_suffix_sha256=suffix_hash,
        config_fingerprint=config_fingerprint,
    )
    return contract, archived


def _write_enriched_geometry_candidate(
    geometry_candidate_path: str | Path,
    enriched_additions: np.ndarray,
    output_path: str | Path,
    contract: GeometryContract,
    *,
    chunk_records: int,
) -> dict[str, object]:
    """Preserve the verified surviving prefix and replace only its suffix."""

    geometry = terrain.inspect_fixed_stride_ply(geometry_candidate_path)
    additions = np.asarray(enriched_additions)
    if additions.ndim != 1 or additions.dtype != geometry.dtype:
        raise ValueError("enriched additions must match geometry WATER schema")
    if len(additions) != contract.addition_count:
        raise ValueError("enriched addition count differs from geometry contract")
    if geometry.vertex_count != contract.candidate_points:
        raise RuntimeError("geometry candidate count drifted before enrichment write")
    output = density.assert_candidate_output_path(
        output_path, source_paths=[geometry.path]
    )
    if output.exists():
        raise FileExistsError(output)
    temporary = output.with_name(f".{output.name}.{os.getpid()}.partial")
    temporary.unlink(missing_ok=True)
    before = geometry.path.stat()
    remaining = contract.base_points * geometry.dtype.itemsize
    block_bytes = max(geometry.dtype.itemsize, chunk_records * geometry.dtype.itemsize)
    try:
        with geometry.path.open("rb") as source, temporary.open("wb") as target:
            target.write(source.read(geometry.offset))
            while remaining:
                block = source.read(min(remaining, block_bytes))
                if not block:
                    raise RuntimeError("unexpected EOF in geometry surviving prefix")
                target.write(block)
                remaining -= len(block)
            additions.tofile(target)
        after = geometry.path.stat()
        if (before.st_size, before.st_mtime_ns) != (after.st_size, after.st_mtime_ns):
            raise RuntimeError("geometry candidate changed during enrichment write")
        os.replace(temporary, output)
    except BaseException:
        temporary.unlink(missing_ok=True)
        raise
    result = terrain.inspect_fixed_stride_ply(output)
    if result.vertex_count != contract.candidate_points:
        raise RuntimeError("enriched geometry candidate point count changed")
    return {
        "source_points": contract.source_points,
        "removed_base_count": contract.removed_base_count,
        "surviving_source_points": contract.base_points,
        "addition_count": contract.addition_count,
        "candidate_points": result.vertex_count,
        "candidate_sha256": sha256_path(output),
        "surviving_source_payload_byte_exact": True,
        "surviving_source_row_order_preserved": True,
    }


def _addition_tree(additions: np.ndarray):
    xy = np.column_stack((additions["x"], additions["y"])).astype(np.float64)
    if not len(xy) or not np.all(np.isfinite(xy)):
        raise ValueError("addition geometry must contain finite XY coordinates")
    try:
        from scipy.spatial import cKDTree
    except ModuleNotFoundError:  # pragma: no cover - production runtime has SciPy
        return xy
    return cKDTree(xy)


def _nearest_addition_distance(tree, query_xy: np.ndarray) -> np.ndarray:
    if isinstance(tree, np.ndarray):  # bounded test fallback
        result = np.full(len(query_xy), np.inf, np.float64)
        for begin in range(0, len(query_xy), 2_048):
            delta = query_xy[begin : begin + 2_048, None, :] - tree[None, :, :]
            result[begin : begin + 2_048] = np.sqrt(
                np.min(np.sum(np.square(delta), axis=2), axis=1)
            )
        return result
    distance, _ = tree.query(query_xy, k=1, workers=-1)
    return np.asarray(distance, np.float64)


def collect_collar_indices(
    path_or_layout: str | Path | terrain.PlyLayout,
    additions: np.ndarray,
    *,
    collar_m: float,
    measured_only: bool,
    required_scan_id: float | None = None,
    chunk_records: int = 1_000_000,
) -> tuple[terrain.PlyLayout, np.ndarray, Mapping[str, object]]:
    """Select only records within an XY collar of an appended point."""

    if not np.isfinite(collar_m) or collar_m <= 0.0:
        raise ValueError("collar_m must be positive and finite")
    layout = (
        path_or_layout
        if isinstance(path_or_layout, terrain.PlyLayout)
        else terrain.inspect_fixed_stride_ply(path_or_layout)
    )
    if layout.dtype != additions.dtype:
        raise ValueError("collar source and WATER additions have different schemas")
    tree = _addition_tree(additions)
    addition_xy = np.column_stack((additions["x"], additions["y"])).astype(np.float64)
    bounds = (
        float(np.min(addition_xy[:, 0]) - collar_m),
        float(np.max(addition_xy[:, 0]) + collar_m),
        float(np.min(addition_xy[:, 1]) - collar_m),
        float(np.max(addition_xy[:, 1]) + collar_m),
    )
    selected_parts: list[np.ndarray] = []
    candidate_points = 0
    maximum_distance = 0.0
    for begin, records in terrain.iter_ply_chunks(layout, chunk_size=chunk_records):
        xy = np.column_stack((records["x"], records["y"])).astype(
            np.float64, copy=False
        )
        keep = (
            (xy[:, 0] >= bounds[0])
            & (xy[:, 0] <= bounds[1])
            & (xy[:, 1] >= bounds[2])
            & (xy[:, 1] <= bounds[3])
        )
        if measured_only:
            keep &= terrain.measured_scan_mask(records)
        if required_scan_id is not None:
            keep &= np.isfinite(records["scalar_ScanID"])
            keep &= records["scalar_ScanID"] == required_scan_id
        local = np.flatnonzero(keep)
        candidate_points += int(len(local))
        if not len(local):
            continue
        distance = _nearest_addition_distance(tree, xy[local])
        within = distance <= collar_m + 1.0e-12
        accepted = local[within]
        if len(accepted):
            selected_parts.append(accepted.astype(np.int64) + begin)
            maximum_distance = max(maximum_distance, float(np.max(distance[within])))
    selected = (
        np.concatenate(selected_parts)
        if selected_parts
        else np.empty(0, np.int64)
    )
    audit = {
        "source": str(layout.path.resolve()),
        "bbox_prefilter_points": candidate_points,
        "selected_points": int(len(selected)),
        "collar_m": float(collar_m),
        "maximum_selected_distance_to_addition_m": (
            maximum_distance if len(selected) else None
        ),
        "measured_only": bool(measured_only),
        "required_scan_id": required_scan_id,
        "selection_is_local_collar_only": True,
    }
    return layout, selected, audit


def collect_tagged_analysed_additions(
    analysed_path: str | Path,
    initial_scan10: np.ndarray,
    *,
    chunk_records: int = 1_000_000,
) -> np.ndarray:
    """Collect tiled ScanID10 output and restore the archive's point order."""

    layout = terrain.inspect_fixed_stride_ply(analysed_path)
    names = set(layout.dtype.names or ())
    required = {"x", "y", "z", "scalar_ScanID", "scalar_TypeID"}
    missing = sorted(required - names)
    if missing:
        raise RuntimeError(f"CleanMesh output lacks fields {missing}")
    parts: list[np.ndarray] = []
    for _, records in terrain.iter_ply_chunks(layout, chunk_size=chunk_records):
        scan = np.asarray(records["scalar_ScanID"], np.float64)
        keep = np.isfinite(scan) & (
            scan == TEMPORARY_ADDITION_SCAN_ID
        )
        if np.any(keep):
            parts.append(np.asarray(records[keep]).copy())
    observed = (
        np.concatenate(parts)
        if parts
        else np.empty(0, dtype=layout.dtype)
    )
    if len(observed) != len(initial_scan10):
        raise RuntimeError(
            "CleanMesh ScanID10 extraction count changed: "
            f"{len(observed)} != {len(initial_scan10)}"
        )
    if len(observed) and not np.all(observed["scalar_TypeID"] == 1.0):
        raise RuntimeError("CleanMesh changed WATER temporary TypeID 1")
    if not len(observed):
        return observed

    expected_order = np.lexsort(
        (
            initial_scan10["z"],
            initial_scan10["y"],
            initial_scan10["x"],
        )
    )
    observed_order = np.lexsort(
        (observed["z"], observed["y"], observed["x"])
    )
    for name in ("x", "y", "z"):
        expected_bytes = np.asarray(initial_scan10[name][expected_order]).tobytes()
        observed_bytes = np.asarray(observed[name][observed_order]).tobytes()
        if expected_bytes != observed_bytes:
            raise RuntimeError(
                f"CleanMesh changed/replaced appended WATER identity field {name}"
            )
    reordered = np.empty_like(observed)
    reordered[expected_order] = observed[observed_order]
    return reordered


def _geometry_fields(dtype: np.dtype) -> tuple[str, ...]:
    return tuple(
        name
        for name in dtype.names or ()
        if name.startswith("scalar_A_R_") and name not in VISIBILITY_FIELDS
    )


def _changed_count(first: np.ndarray, second: np.ndarray, name: str) -> int:
    left = np.asarray(first[name])
    right = np.asarray(second[name])
    if np.issubdtype(left.dtype, np.floating):
        same = (left == right) | (np.isnan(left) & np.isnan(right))
    else:
        same = left == right
    return int(np.count_nonzero(~same))


def _bounded_knn_query(
    support_xyz: np.ndarray,
    query_xyz: np.ndarray,
    *,
    neighbours: int,
    maximum_distance_m: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Query bounded 3D neighbours with a small-test NumPy fallback."""

    support = np.asarray(support_xyz, np.float64)
    query = np.asarray(query_xyz, np.float64)
    count = min(int(neighbours), len(support))
    if count <= 0:
        return (
            np.empty((len(query), 0), np.float64),
            np.empty((len(query), 0), np.int64),
        )
    try:
        from scipy.spatial import cKDTree
    except ModuleNotFoundError:  # pragma: no cover - production runtime has SciPy
        distance = np.full((len(query), count), np.inf, np.float64)
        index = np.full((len(query), count), len(support), np.int64)
        for begin in range(0, len(query), 256):
            local_query = query[begin : begin + 256]
            squared = np.sum(
                np.square(local_query[:, None, :] - support[None, :, :]), axis=2
            )
            if count == len(support):
                local_index = np.broadcast_to(
                    np.arange(len(support), dtype=np.int64), squared.shape
                ).copy()
            else:
                local_index = np.argpartition(squared, count - 1, axis=1)[:, :count]
            local_squared = np.take_along_axis(squared, local_index, axis=1)
            # A stable donor-index tie break makes the reference path deterministic.
            order = np.lexsort((local_index, local_squared), axis=1)
            local_index = np.take_along_axis(local_index, order, axis=1)
            local_squared = np.take_along_axis(local_squared, order, axis=1)
            local_distance = np.sqrt(local_squared)
            outside = local_distance > maximum_distance_m + 1.0e-12
            local_distance[outside] = np.inf
            local_index[outside] = len(support)
            distance[begin : begin + len(local_query)] = local_distance
            index[begin : begin + len(local_query)] = local_index
        return distance, index
    distance, index = cKDTree(support).query(
        query,
        k=count,
        distance_upper_bound=maximum_distance_m + 1.0e-12,
        # One worker plus the explicit distance/index sort below keeps donor
        # selection independent of thread scheduling in provenance reruns.
        workers=1,
    )
    distance = np.asarray(distance, np.float64)
    index = np.asarray(index, np.int64)
    if count == 1:
        distance = distance[:, None]
        index = index[:, None]
    # cKDTree orders by distance, but explicitly stabilize equal-distance donors.
    order = np.lexsort((index, distance), axis=1)
    distance = np.take_along_axis(distance, order, axis=1)
    index = np.take_along_axis(index, order, axis=1)
    return distance, index


def _finite_distance_summary(values: Sequence[np.ndarray]) -> Mapping[str, object]:
    finite_parts = [
        np.asarray(value, np.float64)[np.isfinite(value)]
        for value in values
        if len(value)
    ]
    if not finite_parts or not any(len(value) for value in finite_parts):
        return {
            "count": 0,
            "minimum": None,
            "p50": None,
            "p95": None,
            "maximum": None,
        }
    merged = np.concatenate([value for value in finite_parts if len(value)])
    quantiles = np.quantile(merged, (0.0, 0.5, 0.95, 1.0))
    return {
        "count": int(len(merged)),
        "minimum": float(quantiles[0]),
        "p50": float(quantiles[1]),
        "p95": float(quantiles[2]),
        "maximum": float(quantiles[3]),
    }


def _fill_undefined_geometry_from_component_neighbours(
    records: np.ndarray,
    component_labels: np.ndarray,
    *,
    fields: Iterable[str],
    radii_m: Sequence[float] = SCALAR_FALLBACK_RADII_M,
    maximum_neighbours: int = SCALAR_FALLBACK_MAX_NEIGHBOURS,
    minimum_neighbours: int = SCALAR_FALLBACK_MIN_NEIGHBOURS,
    tiny_component_max_points: int = SCALAR_FALLBACK_TINY_COMPONENT_MAX_POINTS,
) -> Mapping[str, object]:
    """Fill only undefined geometry from finite, same-component analysed rows.

    Each value is an inverse-distance-squared convex interpolation of at most
    ``maximum_neighbours`` originally finite CleanMesh rows.  The smallest
    radius with enough support is used, values are clamped to the local donor
    range, and imputed values never become donors for later rows or fields.
    """

    labels = np.asarray(component_labels)
    if labels.ndim != 1 or len(labels) != len(records):
        raise ValueError("component labels are not aligned to analysed additions")
    if not np.issubdtype(labels.dtype, np.integer):
        raise ValueError("component labels must have an integer dtype")
    radii = tuple(float(value) for value in radii_m)
    if (
        not radii
        or any(not np.isfinite(value) or value <= 0.0 for value in radii)
        or any(right <= left for left, right in zip(radii, radii[1:]))
    ):
        raise ValueError("fallback radii must be finite, positive, and increasing")
    if maximum_neighbours <= 0 or minimum_neighbours <= 0:
        raise ValueError("fallback neighbour counts must be positive")
    if tiny_component_max_points <= 0:
        raise ValueError("tiny_component_max_points must be positive")
    names = set(records.dtype.names or ())
    selected_fields = tuple(dict.fromkeys(str(name) for name in fields))
    missing_fields = sorted(set(selected_fields) - names)
    if missing_fields:
        raise ValueError(f"fallback fields are absent from records: {missing_fields}")
    xyz = np.column_stack((records["x"], records["y"], records["z"])).astype(
        np.float64
    )
    if not np.all(np.isfinite(xyz)):
        raise ValueError("fallback addition coordinates must be finite")
    integer_labels = labels.astype(np.int64, copy=False)
    unique_labels = np.unique(integer_labels)
    field_audit: dict[str, object] = {}
    total_filled = 0
    total_unresolved = 0

    for name in selected_fields:
        values = np.asarray(records[name])
        if not np.issubdtype(values.dtype, np.floating):
            raise ValueError(f"geometry fallback field is not floating point: {name}")
        finite_before = np.isfinite(values)
        finite_indices = np.flatnonzero(finite_before)
        finite_bytes = np.asarray(values[finite_indices]).tobytes()
        missing_before = ~finite_before
        fill_by_radius = {format(radius, ".12g"): 0 for radius in radii}
        fill_by_component: dict[str, int] = {}
        donor_count_by_component: dict[str, int] = {}
        fill_by_radius_by_component: dict[str, dict[str, int]] = {}
        relaxed_components: set[int] = set()
        used_distances: list[np.ndarray] = []
        used_distances_by_component: dict[str, list[np.ndarray]] = {}
        used_local_min_by_component: dict[str, list[np.ndarray]] = {}
        used_local_max_by_component: dict[str, list[np.ndarray]] = {}
        original_donor_range_by_component: dict[str, object] = {}
        filled_count = 0
        range_violations = 0

        for label in unique_labels:
            component = integer_labels == label
            label_key = str(int(label))
            target_index = np.flatnonzero(component & missing_before)
            donor_index = np.flatnonzero(component & finite_before)
            donor_count_by_component[label_key] = int(len(donor_index))
            donor_values_for_range = np.asarray(values[donor_index], np.float64)
            original_donor_range_by_component[label_key] = {
                "minimum": (
                    float(np.min(donor_values_for_range))
                    if len(donor_values_for_range)
                    else None
                ),
                "maximum": (
                    float(np.max(donor_values_for_range))
                    if len(donor_values_for_range)
                    else None
                ),
            }
            used_distances_by_component[label_key] = []
            used_local_min_by_component[label_key] = []
            used_local_max_by_component[label_key] = []
            fill_by_radius_by_component[label_key] = {
                format(radius, ".12g"): 0 for radius in radii
            }
            if not len(target_index):
                fill_by_component[label_key] = 0
                continue
            if not len(donor_index):
                fill_by_component[label_key] = 0
                continue
            component_count = int(np.count_nonzero(component))
            required = int(minimum_neighbours)
            if component_count <= tiny_component_max_points:
                required = min(required, len(donor_index))
                if required < minimum_neighbours:
                    relaxed_components.add(int(label))
            if len(donor_index) < required:
                fill_by_component[label_key] = 0
                continue
            distance, local_index = _bounded_knn_query(
                xyz[donor_index],
                xyz[target_index],
                neighbours=maximum_neighbours,
                maximum_distance_m=radii[-1],
            )
            donor_values = np.asarray(values[donor_index], np.float64)
            counts = np.column_stack(
                [
                    np.count_nonzero(distance <= radius + 1.0e-12, axis=1)
                    for radius in radii
                ]
            )
            supported = counts >= required
            fillable = np.any(supported, axis=1)
            tier = np.argmax(supported, axis=1)
            chosen_radius = np.asarray(radii, np.float64)[tier]
            selected = fillable[:, None] & (
                distance <= chosen_radius[:, None] + 1.0e-12
            )
            safe_index = np.minimum(local_index, len(donor_index) - 1)
            local_values = donor_values[safe_index]
            exact = selected & (distance <= 1.0e-12)
            exact_rows = np.any(exact, axis=1)
            weight = np.where(
                selected,
                1.0 / np.maximum(np.square(distance), 1.0e-18),
                0.0,
            )
            weight[exact_rows] = exact[exact_rows].astype(np.float64)
            weight_sum = np.sum(weight, axis=1)
            fillable &= np.isfinite(weight_sum) & (weight_sum > 0.0)
            estimate = np.full(len(target_index), np.nan, np.float64)
            estimate[fillable] = (
                np.sum(weight[fillable] * local_values[fillable], axis=1)
                / weight_sum[fillable]
            )
            local_min = np.min(np.where(selected, local_values, np.inf), axis=1)
            local_max = np.max(np.where(selected, local_values, -np.inf), axis=1)
            estimate = np.minimum(np.maximum(estimate, local_min), local_max)
            cast = estimate.astype(values.dtype).astype(np.float64)
            cast = np.minimum(np.maximum(cast, local_min), local_max)
            fillable &= np.isfinite(cast)
            in_range = (cast >= local_min) & (cast <= local_max)
            range_violations += int(np.count_nonzero(fillable & ~in_range))
            fillable &= in_range
            output_index = target_index[fillable]
            records[name][output_index] = cast[fillable].astype(values.dtype)
            component_filled = int(np.count_nonzero(fillable))
            if component_filled:
                tier_count = np.bincount(tier[fillable], minlength=len(radii))
                for radius_index, radius in enumerate(radii):
                    key = format(radius, ".12g")
                    count_at_radius = int(tier_count[radius_index])
                    fill_by_radius[key] += count_at_radius
                    fill_by_radius_by_component[label_key][key] += count_at_radius
                selected_distances = distance[selected & fillable[:, None]]
                used_distances.append(selected_distances)
                used_distances_by_component[label_key].append(selected_distances)
                used_local_min_by_component[label_key].append(local_min[fillable])
                used_local_max_by_component[label_key].append(local_max[fillable])
            filled_count += component_filled
            fill_by_component[label_key] = component_filled

        if np.asarray(records[name][finite_indices]).tobytes() != finite_bytes:
            raise RuntimeError(f"fallback changed a finite CleanMesh value in {name}")
        finite_after = np.isfinite(records[name])
        unresolved = int(np.count_nonzero(~finite_after))
        unresolved_by_component = {
            str(int(label)): int(
                np.count_nonzero((integer_labels == label) & ~finite_after)
            )
            for label in unique_labels
        }
        if range_violations:
            raise RuntimeError(f"fallback extrapolated outside local donor range in {name}")
        total_filled += filled_count
        total_unresolved += unresolved
        field_audit[name] = {
            "finite_before": int(np.count_nonzero(finite_before)),
            "undefined_before": int(np.count_nonzero(missing_before)),
            "filled": int(filled_count),
            "unresolved": unresolved,
            "finite_after": int(np.count_nonzero(finite_after)),
            "filled_by_radius_m": fill_by_radius,
            "filled_by_component_label": fill_by_component,
            "filled_by_radius_m_by_component_label": fill_by_radius_by_component,
            "original_finite_donor_count_by_component_label": (
                donor_count_by_component
            ),
            "original_finite_donor_range_by_component_label": (
                original_donor_range_by_component
            ),
            "unresolved_by_component_label": unresolved_by_component,
            "tiny_component_minimum_relaxed_labels": sorted(relaxed_components),
            "contributing_donor_distance_m": _finite_distance_summary(used_distances),
            "contributing_donor_distance_m_by_component_label": {
                key: _finite_distance_summary(value)
                for key, value in used_distances_by_component.items()
            },
            "selected_local_donor_minimum_by_component_label": {
                key: _finite_distance_summary(value)
                for key, value in used_local_min_by_component.items()
            },
            "selected_local_donor_maximum_by_component_label": {
                key: _finite_distance_summary(value)
                for key, value in used_local_max_by_component.items()
            },
            "original_finite_values_byte_exact": True,
            "local_donor_range_bounded": True,
        }

    return {
        "method": "component-strict-multiscale-3d-idw",
        "source": "originally-finite-local-cleanmesh-analysed-additions-only",
        "coordinate_space": "XYZ-metres",
        "radii_m": list(radii),
        "adaptive_larger_radius_tiers": bool(len(radii) > 4),
        "maximum_radius_m": float(radii[-1]),
        "maximum_radius_is_registered_component_diameter_bound": bool(
            np.isclose(
                radii[-1],
                SCALAR_FALLBACK_COMPONENT_DIAMETER_BOUND_M,
                rtol=0.0,
                atol=1.0e-12,
            )
        ),
        "maximum_neighbours": int(maximum_neighbours),
        "minimum_neighbours": int(minimum_neighbours),
        "tiny_component_max_points": int(tiny_component_max_points),
        "tiny_components_may_use_all_available_finite_support": True,
        "cross_component_borrowing": False,
        "imputed_values_may_become_donors": False,
        "only_original_finite_values_may_be_donors": True,
        "multi_hop_propagation": False,
        "interpolation_is_convex": True,
        "local_donor_range_bounded": True,
        "component_labels": [int(value) for value in unique_labels],
        "fields": field_audit,
        "total_filled_field_values": int(total_filled),
        "total_unresolved_field_values": int(total_unresolved),
    }


def merge_analysed_geometry(
    analysed: np.ndarray,
    initial_scan10: np.ndarray,
    combined_normalizations: Mapping[str, Mapping[str, float]],
    *,
    component_labels: np.ndarray | None = None,
) -> tuple[np.ndarray, Mapping[str, object]]:
    """Keep only CleanMesh geometry A_R fields and restore WATER identity."""

    if len(analysed) != len(initial_scan10):
        raise ValueError("analysed and initial addition counts differ")
    target_dtype = initial_scan10.dtype
    analysed_names = set(analysed.dtype.names or ())
    geometry_fields = _geometry_fields(target_dtype)
    missing = sorted(set(geometry_fields) - analysed_names)
    if missing:
        raise RuntimeError(f"CleanMesh output lacks geometry fields {missing}")
    output = np.asarray(initial_scan10).copy()
    for name in geometry_fields:
        output[name] = analysed[name]
    initial_postprocess = terrain_pipeline.postprocess_local_analysed_additions(
        output, initial_scan10, combined_normalizations
    )
    active_component_labels = (
        np.asarray(component_labels)
        if component_labels is not None
        else np.zeros(len(output), np.int32)
    )
    component_coverage_before = _component_field_finite_coverage(
        output, active_component_labels, geometry_fields
    )
    combined_fields = [
        f"scalar_A_R_{metric}_Combined"
        for metric in PHYSICAL_METRICS
        if f"scalar_A_R_{metric}_Combined" in geometry_fields
    ]
    combined_coverage_before = _finite_coverage(output, combined_fields)
    dependent_fields = set(combined_fields)
    relative_name = "scalar_A_R_RoughnessRelative_FineMedium"
    if relative_name in geometry_fields:
        dependent_fields.add(relative_name)
    fallback_fields = tuple(
        name for name in geometry_fields if name not in dependent_fields
    )
    has_undefined_fallback_fields = any(
        np.any(~np.isfinite(output[name])) for name in fallback_fields
    )
    if has_undefined_fallback_fields and component_labels is None:
        raise RuntimeError(
            "undefined fine CleanMesh geometry requires verified component labels"
        )
    fallback_audit = _fill_undefined_geometry_from_component_neighbours(
        output,
        (
            active_component_labels
        ),
        fields=fallback_fields,
    )
    # Combined and relative roughness remain globally normalized derived
    # quantities.  They are rebuilt after filling their physical components;
    # no direct Combined-field borrowing or local histogram normalization occurs.
    final_postprocess = terrain_pipeline.postprocess_local_analysed_additions(
        output, initial_scan10, combined_normalizations
    )
    # The initial archive is authoritative for every field except the
    # geometry A_R family.  This explicitly covers intensity/composition,
    # analytic normals and local visibility.
    for name in target_dtype.names or ():
        if name not in geometry_fields:
            output[name] = initial_scan10[name]
    output["scalar_ScanID"] = np.asarray(
        WATER_SCAN_ID, dtype=output.dtype["scalar_ScanID"]
    )

    required_components = {
        f"scalar_A_R_{metric}_{scale}"
        for metric in PHYSICAL_METRICS
        for scale in DERIVED_SCALES
        if f"scalar_A_R_{metric}_{scale}" in (target_dtype.names or ())
    }
    if required_components and not any(
        np.any(np.isfinite(output[name])) for name in required_components
    ):
        raise RuntimeError("CleanMesh produced no finite physical geometry metrics")
    changed = {
        name: _changed_count(output, initial_scan10, name)
        for name in geometry_fields
    }
    if len(output) and geometry_fields and not any(changed.values()):
        raise RuntimeError("CleanMesh enrichment left every geometry A_R value unchanged")
    combined_coverage = _finite_coverage(output, combined_fields)
    component_coverage = _require_component_field_coverage(
        output,
        active_component_labels,
        geometry_fields,
        context="fine CleanMesh enrichment",
    )
    if combined_fields and any(
        float(combined_coverage[name]["fraction"])
        < MINIMUM_COMBINED_FINITE_FRACTION
        for name in combined_fields
    ):
        raise RuntimeError(
            "fine CleanMesh enrichment has insufficient finite combined-field coverage"
        )

    if "scalar_Intensity" in (target_dtype.names or ()):
        intensity = np.asarray(output["scalar_Intensity"], np.float64)
        if np.any(~np.isfinite(intensity) | (intensity < 0.0)):
            raise RuntimeError("restored WATER intensity contains invalid values")
    if "scalar_Composite" in (target_dtype.names or ()):
        composite = np.asarray(output["scalar_Composite"], np.float64)
        if np.any(~np.isfinite(composite) | (composite < 0.0) | (composite > 255.0)):
            raise RuntimeError("restored WATER composition contains invalid values")
    non_geometry_exact = all(
        np.asarray(output[name]).tobytes() == np.asarray(initial_scan10[name]).tobytes()
        for name in target_dtype.names or ()
        if name not in geometry_fields and name != "scalar_ScanID"
    )
    if not non_geometry_exact:
        raise RuntimeError("a non-geometric archived WATER field changed")
    return output, {
        "geometry_fields_replaced": list(geometry_fields),
        "changed_points_by_field": changed,
        "temporary_scan_id": TEMPORARY_ADDITION_SCAN_ID,
        "restored_scan_id": WATER_SCAN_ID,
        "non_geometry_fields_archive_exact": True,
        "intensity_and_composition_archive_exact": True,
        "combined_field_finite_coverage": combined_coverage,
        "combined_field_finite_coverage_before_fallback": combined_coverage_before,
        "minimum_combined_finite_fraction": MINIMUM_COMBINED_FINITE_FRACTION,
        "component_field_finite_coverage": component_coverage,
        "component_field_finite_coverage_before_fallback": (
            component_coverage_before
        ),
        "minimum_component_field_finite_fraction": (
            MINIMUM_COMPONENT_FIELD_FINITE_FRACTION
        ),
        "undefined_geometry_fallback": {
            **dict(fallback_audit),
            "interpolated_fields": list(fallback_fields),
            "dependent_fields_rebuilt_not_interpolated": sorted(dependent_fields),
        },
        "postprocess": {
            "before_fallback": dict(initial_postprocess),
            "after_fallback": dict(final_postprocess),
        },
    }


def _finite_coverage(records: np.ndarray, fields: Iterable[str]) -> dict[str, object]:
    result: dict[str, object] = {}
    names = set(records.dtype.names or ())
    for name in fields:
        if name not in names:
            result[name] = {"present": False, "finite": 0, "fraction": 0.0}
            continue
        finite = int(np.count_nonzero(np.isfinite(records[name])))
        result[name] = {
            "present": True,
            "finite": finite,
            "total": int(len(records)),
            "fraction": float(finite / len(records)) if len(records) else 1.0,
        }
    return result


def _component_field_finite_coverage(
    records: np.ndarray,
    component_labels: np.ndarray,
    fields: Iterable[str],
) -> Mapping[str, object]:
    """Audit every geometry field independently within every component."""

    labels = np.asarray(component_labels)
    if labels.ndim != 1 or len(labels) != len(records):
        raise ValueError("component labels are not aligned to scalar records")
    if not np.issubdtype(labels.dtype, np.integer):
        raise ValueError("component labels must have an integer dtype")
    selected_fields = tuple(dict.fromkeys(str(name) for name in fields))
    names = set(records.dtype.names or ())
    missing = sorted(set(selected_fields) - names)
    if missing:
        raise ValueError(f"component scalar audit fields are absent: {missing}")
    rows: list[dict[str, object]] = []
    all_complete = True
    for raw_label in np.unique(labels.astype(np.int64, copy=False)):
        label = int(raw_label)
        member = labels == raw_label
        count = int(np.count_nonzero(member))
        field_rows: dict[str, object] = {}
        component_complete = True
        for name in selected_fields:
            values = np.asarray(records[name][member], np.float64)
            finite = np.isfinite(values)
            finite_count = int(np.count_nonzero(finite))
            fraction = float(finite_count / count) if count else 1.0
            finite_values = values[finite]
            range_contract = "finite-physical-values-no-global-clamp"
            range_passed = True
            lower = upper = None
            if name.endswith("_Combined"):
                lower = 0.0 if "_Roughness_" in name else -1.0
                upper = 1.0
                range_contract = "global-normalized-combined-range"
            elif name == "scalar_A_R_RoughnessRelative_FineMedium":
                lower, upper = 0.0, 8.0
                range_contract = "derived-relative-roughness-range"
            if lower is not None and finite_count:
                range_passed = bool(
                    np.all(finite_values >= lower - 1.0e-6)
                    and np.all(finite_values <= upper + 1.0e-6)
                )
            complete = bool(
                fraction + 1.0e-15
                >= MINIMUM_COMPONENT_FIELD_FINITE_FRACTION
                and range_passed
            )
            component_complete &= complete
            field_rows[name] = {
                "finite": finite_count,
                "total": count,
                "fraction": fraction,
                "minimum": float(np.min(finite_values)) if finite_count else None,
                "maximum": float(np.max(finite_values)) if finite_count else None,
                "range_contract": range_contract,
                "range_lower": lower,
                "range_upper": upper,
                "range_passed": range_passed,
                "accepted": complete,
            }
        all_complete &= component_complete
        rows.append(
            {
                "component_label": label,
                "points": count,
                "all_required_fields_accepted": component_complete,
                "fields": field_rows,
            }
        )
    return {
        "method": "exact-per-component-per-geometry-field-finiteness-and-range-v1",
        "minimum_required_finite_fraction": (
            MINIMUM_COMPONENT_FIELD_FINITE_FRACTION
        ),
        "required_fields": list(selected_fields),
        "component_count": int(len(rows)),
        "components": rows,
        "all_components_all_required_fields_accepted": bool(all_complete),
    }


def _require_component_field_coverage(
    records: np.ndarray,
    component_labels: np.ndarray,
    fields: Iterable[str],
    *,
    context: str,
) -> Mapping[str, object]:
    audit = _component_field_finite_coverage(records, component_labels, fields)
    if not audit["all_components_all_required_fields_accepted"]:
        failures = []
        for component in audit["components"]:
            for name, field in component["fields"].items():
                if not field["accepted"]:
                    failures.append(
                        f"label {component['component_label']} {name} "
                        f"finite={field['finite']}/{field['total']} "
                        f"range_passed={field['range_passed']}"
                    )
        raise RuntimeError(
            f"{context} has insufficient finite per-component scalar coverage: "
            + "; ".join(failures[:12])
        )
    return audit


def verify_candidate_component_scalar_coverage(
    candidate_path: str | Path,
    *,
    base_points: int,
    component_labels: np.ndarray,
    context: str = "candidate addition suffix",
) -> Mapping[str, object]:
    """Directly prove every appended component has complete A_R scalars.

    This is intentionally a file-level publication check rather than a trust
    in the enrichment manifest.  The labels come from the hash-locked geometry
    archive and are aligned one-for-one with the appended records.
    """

    layout = terrain.inspect_fixed_stride_ply(candidate_path)
    prefix = int(base_points)
    if prefix < 0 or prefix > layout.vertex_count:
        raise RuntimeError("candidate scalar-audit base point count is invalid")
    labels = np.asarray(component_labels)
    addition_count = layout.vertex_count - prefix
    if labels.ndim != 1 or len(labels) != addition_count:
        raise RuntimeError(
            "candidate scalar-audit labels are not aligned to the addition suffix"
        )
    if not np.issubdtype(labels.dtype, np.integer):
        raise ValueError("candidate scalar-audit labels must be integers")
    fields = _geometry_fields(layout.dtype)
    if not fields:
        raise RuntimeError("candidate has no required A_R geometry fields")
    memory = np.memmap(
        layout.path,
        dtype=layout.dtype,
        mode="r",
        offset=layout.offset,
        shape=(layout.vertex_count,),
    )
    try:
        suffix = memory[prefix:]
        coverage = _require_component_field_coverage(
            suffix,
            labels,
            fields,
            context=context,
        )
    finally:
        del memory
    return {
        "method": "direct-ply-addition-suffix-per-component-scalar-audit-v1",
        "candidate_path": str(layout.path),
        "candidate_points": int(layout.vertex_count),
        "base_points": prefix,
        "addition_count": int(addition_count),
        "component_label_sha256": _records_sha256(
            labels.astype(np.int64, copy=False)
        ),
        "geometry_fields": list(fields),
        "coverage": coverage,
        "all_required_fields_accepted": True,
    }


def verify_coarse_exact_subset_component_scalar_coverage(
    fine_candidate_path: str | Path,
    coarse_candidate_path: str | Path,
    *,
    fine_base_points: int,
    fine_component_labels: np.ndarray,
    chunk_records: int = 250_000,
) -> Mapping[str, object]:
    """Audit the active 5 mm additions selected exactly from the fine cloud.

    Full record bytes identify which coarse rows came from the enriched fine
    suffix.  This avoids geometric nearest-neighbour ambiguity and proves that
    every fine component remains represented at 5 mm.  Only the matched active
    addition rows are audited; pre-existing WATER rows are outside this scalar
    enrichment contract.
    """

    if chunk_records <= 0:
        raise ValueError("chunk_records must be positive")
    fine_layout = terrain.inspect_fixed_stride_ply(fine_candidate_path)
    coarse_layout = terrain.inspect_fixed_stride_ply(coarse_candidate_path)
    if fine_layout.dtype != coarse_layout.dtype:
        raise RuntimeError("fine/coarse scalar-audit schemas differ")
    prefix = int(fine_base_points)
    if prefix < 0 or prefix > fine_layout.vertex_count:
        raise RuntimeError("fine scalar-audit base point count is invalid")
    labels = np.asarray(fine_component_labels)
    fine_addition_count = fine_layout.vertex_count - prefix
    if labels.ndim != 1 or len(labels) != fine_addition_count:
        raise RuntimeError(
            "fine scalar-audit labels are not aligned to the addition suffix"
        )
    if not np.issubdtype(labels.dtype, np.integer):
        raise ValueError("fine scalar-audit labels must be integers")
    fields = _geometry_fields(fine_layout.dtype)
    if not fields:
        raise RuntimeError("fine/coarse candidates have no required A_R geometry fields")

    fine_memory = np.memmap(
        fine_layout.path,
        dtype=fine_layout.dtype,
        mode="r",
        offset=fine_layout.offset,
        shape=(fine_layout.vertex_count,),
    )
    coarse_memory = np.memmap(
        coarse_layout.path,
        dtype=coarse_layout.dtype,
        mode="r",
        offset=coarse_layout.offset,
        shape=(coarse_layout.vertex_count,),
    )
    key_dtype = np.dtype((np.void, fine_layout.dtype.itemsize))
    try:
        fine_suffix = fine_memory[prefix:]
        fine_keys = np.asarray(fine_suffix).view(key_dtype).reshape(-1)
        if len(np.unique(fine_keys)) != len(fine_keys):
            raise RuntimeError(
                "fine addition suffix has duplicate full records; coarse membership is ambiguous"
            )
        order = np.argsort(fine_keys, kind="stable")
        sorted_keys = fine_keys[order]
        matched_record_parts: list[np.ndarray] = []
        matched_label_parts: list[np.ndarray] = []
        matched_index_parts: list[np.ndarray] = []
        for start in range(0, coarse_layout.vertex_count, int(chunk_records)):
            stop = min(start + int(chunk_records), coarse_layout.vertex_count)
            block = coarse_memory[start:stop]
            keys = np.asarray(block).view(key_dtype).reshape(-1)
            positions = np.searchsorted(sorted_keys, keys)
            inside = positions < len(sorted_keys)
            matched = np.zeros(len(keys), dtype=bool)
            if np.any(inside):
                inside_indices = np.flatnonzero(inside)
                matched[inside_indices] = (
                    sorted_keys[positions[inside_indices]] == keys[inside_indices]
                )
            if not np.any(matched):
                continue
            fine_indices = order[positions[matched]]
            matched_record_parts.append(np.asarray(block[matched]).copy())
            matched_label_parts.append(
                labels[fine_indices].astype(np.int64, copy=True)
            )
            matched_index_parts.append(fine_indices.astype(np.int64, copy=True))
        if not matched_record_parts:
            raise RuntimeError("5mm candidate contains no exact fine addition records")
        matched_records = np.concatenate(matched_record_parts)
        matched_labels = np.concatenate(matched_label_parts)
        matched_indices = np.concatenate(matched_index_parts)
        if len(np.unique(matched_indices)) != len(matched_indices):
            raise RuntimeError("5mm candidate repeats an exact fine addition record")
        fine_labels_present = set(
            int(value) for value in np.unique(labels.astype(np.int64, copy=False))
        )
        coarse_labels_present = set(int(value) for value in np.unique(matched_labels))
        if coarse_labels_present != fine_labels_present:
            missing = sorted(fine_labels_present - coarse_labels_present)
            raise RuntimeError(
                "5mm candidate omits fine scalar component labels: " + str(missing)
            )
        coverage = _require_component_field_coverage(
            matched_records,
            matched_labels,
            fields,
            context="active 5mm exact fine-subset additions",
        )
    finally:
        del fine_memory
        del coarse_memory
    return {
        "method": "exact-full-record-fine-suffix-membership-scalar-audit-v1",
        "fine_candidate_path": str(fine_layout.path),
        "coarse_candidate_path": str(coarse_layout.path),
        "fine_candidate_points": int(fine_layout.vertex_count),
        "coarse_candidate_points": int(coarse_layout.vertex_count),
        "fine_base_points": prefix,
        "fine_addition_count": int(fine_addition_count),
        "matched_coarse_addition_count": int(len(matched_records)),
        "fine_component_labels": sorted(fine_labels_present),
        "coarse_component_labels": sorted(coarse_labels_present),
        "component_label_sha256": _records_sha256(
            labels.astype(np.int64, copy=False)
        ),
        "geometry_fields": list(fields),
        "coverage": coverage,
        "full_record_membership_exact": True,
        "every_fine_component_represented": True,
        "all_required_fields_accepted": True,
    }


def _load_cross_scale_mapping(
    archive_path: Path,
    *,
    expected_count: int,
) -> tuple[np.ndarray, np.ndarray]:
    with np.load(archive_path, allow_pickle=False) as loaded:
        required = {"fine_selection_index", "fine_component_label"}
        missing = sorted(required - set(loaded.files))
        if missing:
            raise ValueError(
                f"5mm cross-scale geometry archive lacks arrays {missing}"
            )
        value = np.asarray(loaded["fine_selection_index"])
        component_value = np.asarray(loaded["fine_component_label"])
    if value.ndim != 1 or len(value) != expected_count:
        raise RuntimeError("fine_selection_index is not aligned to coarse additions")
    if component_value.ndim != 1 or len(component_value) != expected_count:
        raise RuntimeError("fine_component_label is not aligned to coarse additions")
    if not np.issubdtype(value.dtype, np.integer):
        raise ValueError("fine_selection_index must have an integer dtype")
    if not np.issubdtype(component_value.dtype, np.integer):
        raise ValueError("fine_component_label must have an integer dtype")
    index = value.astype(np.int64, copy=False)
    component_label = component_value.astype(np.int64, copy=False)
    if len(np.unique(index)) != len(index):
        raise RuntimeError("fine_selection_index is not unique")
    return index, component_label


def _load_geometry_component_membership(
    document: Mapping[str, object],
    archive_path: str | Path,
    *,
    expected_count: int,
) -> tuple[np.ndarray, Mapping[str, object]]:
    """Load a hole archive and prove every addition's component identity."""

    membership = document.get("component_membership")
    if not isinstance(membership, Mapping):
        raise ValueError("geometry manifest lacks component_membership proof")
    if membership.get("archive_key") != "candidate_label":
        raise ValueError("geometry component_membership uses an unexpected archive key")
    if membership.get("all_additions_assigned_to_accepted_component") is not True:
        raise RuntimeError("geometry component_membership is not fully accepted")
    accepted_value = membership.get("accepted_labels")
    if not isinstance(accepted_value, list):
        raise ValueError("geometry component_membership accepted_labels must be a list")
    if any(isinstance(value, bool) or not isinstance(value, int) for value in accepted_value):
        raise ValueError("geometry component_membership labels must be integers")
    accepted_labels = tuple(int(value) for value in accepted_value)
    if len(set(accepted_labels)) != len(accepted_labels) or not accepted_labels:
        raise ValueError("geometry component_membership labels are empty or duplicated")
    holes_value = document.get("holes")
    if not isinstance(holes_value, list):
        raise ValueError("geometry manifest lacks hole records")
    accepted_hole_labels: set[int] = set()
    for item in holes_value:
        if not isinstance(item, Mapping) or not item.get("accepted"):
            continue
        label_value = item.get("label")
        if isinstance(label_value, bool) or not isinstance(label_value, int):
            raise ValueError("accepted geometry hole has an invalid component label")
        component_label = int(label_value)
        if component_label in accepted_hole_labels:
            raise ValueError("accepted geometry hole component label is duplicated")
        accepted_hole_labels.add(component_label)
    if set(accepted_labels) != accepted_hole_labels:
        raise RuntimeError(
            "geometry component_membership labels differ from accepted holes"
        )

    archive = Path(archive_path).resolve(strict=True)
    with np.load(archive, allow_pickle=False) as loaded:
        required = {"records", "candidate_xy", "candidate_label"}
        missing = sorted(required - set(loaded.files))
        if missing:
            raise ValueError(f"geometry archive lacks arrays {missing}")
        records = np.asarray(loaded["records"]).copy()
        candidate_xy = np.asarray(loaded["candidate_xy"], np.float64).copy()
        value = np.asarray(loaded["candidate_label"])
    if records.ndim != 1 or len(records) != expected_count:
        raise RuntimeError("geometry archive records have the wrong count")
    if candidate_xy.shape != (expected_count, 2):
        raise RuntimeError("geometry archive candidate_xy has the wrong shape")
    if value.ndim != 1 or len(value) != expected_count:
        raise RuntimeError("candidate_label is not aligned to additions")
    if not np.issubdtype(value.dtype, np.integer):
        raise ValueError("candidate_label must have an integer dtype")
    if not {"x", "y"}.issubset(records.dtype.names or ()):
        raise ValueError("geometry archive records lack XY fields")
    stored_xy = np.column_stack(
        (
            candidate_xy[:, 0].astype(records.dtype["x"]),
            candidate_xy[:, 1].astype(records.dtype["y"]),
        )
    ).astype(np.float64)
    record_xy = np.column_stack((records["x"], records["y"])).astype(np.float64)
    if not np.array_equal(record_xy, stored_xy):
        raise RuntimeError("geometry archive candidate_xy differs from record XY")
    component_label = value.astype(np.int64, copy=False)
    if set(np.unique(component_label)) != set(accepted_labels):
        raise RuntimeError(
            "geometry archive component labels do not cover accepted holes exactly"
        )
    return component_label, {
        "archive": asdict(fingerprint_file(archive)),
        "archive_arrays_verified": sorted(required),
        "candidate_xy_record_exact": True,
        "component_label_count": int(len(component_label)),
        "component_label_sha256": _records_sha256(component_label),
        "accepted_labels": list(accepted_labels),
    }


def _load_fine_component_membership(
    fine_document: Mapping[str, object],
    *,
    expected_count: int,
) -> tuple[np.ndarray, Mapping[str, object]]:
    """Load the fine archive through its exact enrichment fingerprint."""

    fine_inputs = fine_document.get("input_fingerprints")
    if not isinstance(fine_inputs, Mapping):
        raise ValueError("fine enrichment lacks input_fingerprints")
    archive_ref = fine_inputs.get("geometry_archive")
    if not isinstance(archive_ref, Mapping):
        raise ValueError("fine enrichment lacks its geometry_archive fingerprint")
    path_value = archive_ref.get("path")
    if not isinstance(path_value, str):
        raise ValueError("fine geometry_archive fingerprint lacks a path")
    archive_path = Path(path_value).resolve(strict=True)
    actual = asdict(fingerprint_file(archive_path))
    for name in ("path", "size_bytes", "mtime_ns", "sha256"):
        if name not in archive_ref or archive_ref[name] != actual[name]:
            raise RuntimeError(f"fine geometry archive {name} fingerprint mismatch")
    component_label, audit = _load_geometry_component_membership(
        fine_document,
        archive_path,
        expected_count=expected_count,
    )
    return component_label, {**dict(audit), "archive": actual}


def _verify_cross_scale_manifest(
    cross_scale: Mapping[str, object],
    fine_document: Mapping[str, object],
    fine_contract: Mapping[str, object],
    selection_index: np.ndarray,
    fine_component_label: np.ndarray,
    coarse_component_label: np.ndarray,
    normalization_manifest_sha256: str,
) -> Mapping[str, object]:
    """Verify the coarse manifest's fine-artifact and selection assertions."""

    required = {
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
    missing = sorted(required - set(cross_scale))
    if missing:
        raise ValueError(f"5mm cross_scale proof lacks keys {missing}")
    if cross_scale["method"] != "deterministic-variable-radius-blue-noise-subset-v1":
        raise ValueError("5mm cross_scale method is not the approved producer")
    fine_geometry_manifest = fine_document.get("geometry_manifest")
    if not isinstance(fine_geometry_manifest, Mapping):
        raise ValueError("fine enrichment lacks upstream geometry_manifest proof")
    fine_inputs = fine_document.get("input_fingerprints")
    if not isinstance(fine_inputs, Mapping):
        raise ValueError("fine enrichment lacks hash-locked input fingerprints")
    fine_geometry_input = fine_inputs.get("geometry_manifest")
    fine_archive_input = fine_inputs.get("geometry_archive")
    if not isinstance(fine_geometry_input, Mapping) or not isinstance(
        fine_archive_input, Mapping
    ):
        raise ValueError("fine enrichment lacks geometry manifest/archive fingerprints")
    fine_normalization_input = fine_inputs.get("normalization_manifest")
    if not isinstance(fine_normalization_input, Mapping) or not isinstance(
        fine_normalization_input.get("sha256"), str
    ):
        raise ValueError("fine enrichment lacks normalization manifest fingerprint")
    if fine_normalization_input["sha256"] != normalization_manifest_sha256:
        raise RuntimeError("fine/coarse combined normalizations differ")
    manifest_ref = cross_scale["fine_manifest"]
    archive_ref = cross_scale["fine_archive"]
    if not isinstance(manifest_ref, Mapping) or not isinstance(archive_ref, Mapping):
        raise ValueError("cross_scale fine artifact fingerprints must be mappings")
    fingerprint_keys = ("path", "size_bytes", "mtime_ns", "sha256")
    for label, reference, enrichment_input in (
        ("manifest", manifest_ref, fine_geometry_input),
        ("archive", archive_ref, fine_archive_input),
    ):
        missing_fingerprint = [
            name
            for name in fingerprint_keys
            if name not in reference or name not in enrichment_input
        ]
        if missing_fingerprint:
            raise ValueError(
                f"fine geometry {label} fingerprint lacks keys "
                f"{missing_fingerprint}"
            )
        if any(reference[name] != enrichment_input[name] for name in fingerprint_keys):
            raise RuntimeError(
                f"cross_scale fine geometry {label} fingerprint mismatch"
            )
    if manifest_ref.get("sha256") != fine_geometry_manifest.get("sha256"):
        raise RuntimeError("cross_scale fine geometry manifest hash mismatch")
    if archive_ref.get("sha256") != fine_document.get("archive_sha256"):
        raise RuntimeError("cross_scale fine geometry archive hash mismatch")
    if cross_scale["fine_candidate_sha256"] != fine_contract.get("candidate_sha256"):
        raise RuntimeError("cross_scale fine geometry candidate hash mismatch")
    fine_addition_count = int(fine_contract["addition_count"])
    if int(cross_scale["fine_addition_count"]) != fine_addition_count:
        raise RuntimeError("cross_scale fine addition count mismatch")
    if int(cross_scale["fine_selection_index_count"]) != len(selection_index):
        raise RuntimeError("cross_scale selection count mismatch")
    if cross_scale["fine_selection_index_unique"] is not True:
        raise RuntimeError("cross_scale manifest does not attest unique selection")
    if cross_scale["coarse_xyz_exact_subset_of_fine_records_xyz"] is not True:
        raise RuntimeError("cross_scale manifest does not attest exact fine XYZ subset")
    if cross_scale["coarse_normals_exact_subset_of_fine_records_normals"] is not True:
        raise RuntimeError(
            "cross_scale manifest does not attest exact selected fine normals"
        )
    if cross_scale["nongeometry_fields_preserved_from_coarse_donors"] is not True:
        raise RuntimeError(
            "cross_scale manifest does not attest coarse donor nongeometry"
        )
    copied_geometry = cross_scale["geometry_fields_copied_from_fine_records"]
    if copied_geometry != ["x", "y", "z", "nx", "ny", "nz"]:
        raise RuntimeError(
            "cross_scale manifest has an unexpected fine geometry field set"
        )
    if len(selection_index) and (
        int(np.min(selection_index)) < 0
        or int(np.max(selection_index)) >= fine_addition_count
    ):
        raise RuntimeError("fine_selection_index lies outside fine addition suffix")
    selection_seed = cross_scale["selection_seed"]
    if isinstance(selection_seed, bool) or not isinstance(selection_seed, int):
        raise ValueError("cross_scale selection_seed must be an integer")
    spacing = float(cross_scale["spacing_m"])
    maximum_support_distance = float(
        cross_scale["maximum_fine_to_coarse_or_terrain_support_distance_m"]
    )
    if not np.isfinite(spacing) or spacing <= 0.0:
        raise ValueError("cross_scale spacing_m must be positive")
    if not np.isfinite(maximum_support_distance) or maximum_support_distance < 0.0:
        raise ValueError("cross_scale maximum support distance is invalid")
    if maximum_support_distance > spacing + 1.0e-9:
        raise RuntimeError(
            "cross_scale fine topology is not covered within coarse spacing"
        )
    coverage = cross_scale["accepted_hole_coverage"]
    if not isinstance(coverage, list):
        raise ValueError("cross_scale accepted_hole_coverage must be a list")
    fine_holes = fine_document.get("holes")
    if not isinstance(fine_holes, list):
        raise ValueError("fine enrichment lacks the geometry hole records")
    accepted_holes: dict[
        str, tuple[int, tuple[float, float, float, float]]
    ] = {}
    accepted_component_labels: set[int] = set()
    for item in fine_holes:
        if not isinstance(item, Mapping) or not item.get("accepted"):
            continue
        seed_id = str(item.get("seed_id", ""))
        label_value = item.get("label")
        bounds_value = item.get("bounds")
        if (
            not seed_id
            or isinstance(label_value, bool)
            or not isinstance(label_value, int)
            or not isinstance(bounds_value, list)
            or len(bounds_value) != 4
        ):
            raise ValueError("accepted fine hole has invalid seed_id/label/bounds")
        component_label = int(label_value)
        bounds = tuple(float(value) for value in bounds_value)
        if not all(np.isfinite(bounds)) or bounds[0] > bounds[1] or bounds[2] > bounds[3]:
            raise ValueError("accepted fine hole has invalid bounds")
        if seed_id in accepted_holes or component_label in accepted_component_labels:
            raise ValueError("accepted fine hole seed_id/label is not unique")
        accepted_component_labels.add(component_label)
        accepted_holes[seed_id] = (component_label, bounds)
    if len(fine_component_label) != fine_addition_count:
        raise RuntimeError("fine component labels differ from fine addition count")
    if len(coarse_component_label) != len(selection_index):
        raise RuntimeError("coarse component labels differ from selection count")
    if not np.array_equal(
        coarse_component_label,
        fine_component_label[selection_index],
    ):
        raise RuntimeError(
            "coarse fine_component_label differs from selected fine labels"
        )
    if set(np.unique(fine_component_label)) != accepted_component_labels:
        raise RuntimeError(
            "fine component labels do not cover exactly the accepted holes"
        )
    membership = fine_document.get("component_membership")
    if not isinstance(membership, Mapping):
        raise ValueError("fine enrichment lacks component_membership proof")
    membership_labels = membership.get("accepted_labels")
    if not isinstance(membership_labels, list):
        raise ValueError("fine component_membership accepted_labels must be a list")
    if set(int(value) for value in membership_labels) != accepted_component_labels:
        raise RuntimeError(
            "fine component_membership labels differ from accepted holes"
        )
    coverage_rows: list[dict[str, object]] = []
    seen_ids: set[str] = set()
    for item in coverage:
        if not isinstance(item, Mapping):
            raise ValueError("accepted_hole_coverage row must be an object")
        row_required = {
            "seed_id",
            "component_label",
            "fine_count",
            "coarse_addition_count",
            "bounds",
        }
        row_missing = sorted(row_required - set(item))
        if row_missing:
            raise ValueError(
                f"accepted_hole_coverage row lacks keys {row_missing}"
            )
        seed_id = str(item["seed_id"])
        if not seed_id or seed_id in seen_ids:
            raise ValueError("accepted_hole_coverage seed_id is empty or duplicated")
        if seed_id not in accepted_holes:
            raise RuntimeError("accepted_hole_coverage contains an unaccepted hole")
        seen_ids.add(seed_id)
        component_value = item["component_label"]
        if isinstance(component_value, bool) or not isinstance(component_value, int):
            raise ValueError("accepted_hole_coverage component_label must be an integer")
        component_label = int(component_value)
        accepted_component_label, accepted_bounds = accepted_holes[seed_id]
        if component_label != accepted_component_label:
            raise RuntimeError(
                "accepted_hole_coverage component label differs from fine hole"
            )
        bounds_value = item["bounds"]
        if not isinstance(bounds_value, list) or len(bounds_value) != 4:
            raise ValueError("accepted_hole_coverage bounds must have four values")
        bounds = tuple(float(value) for value in bounds_value)
        if not all(np.isfinite(bounds)) or bounds[0] > bounds[1] or bounds[2] > bounds[3]:
            raise ValueError("accepted_hole_coverage bounds are invalid")
        if bounds != accepted_bounds:
            raise RuntimeError("accepted_hole_coverage bounds differ from fine hole")
        fine_count = item["fine_count"]
        coarse_count = item["coarse_addition_count"]
        if (
            isinstance(fine_count, bool)
            or not isinstance(fine_count, int)
            or isinstance(coarse_count, bool)
            or not isinstance(coarse_count, int)
        ):
            raise ValueError("accepted_hole_coverage counts must be integers")
        actual_fine_count = int(np.count_nonzero(fine_component_label == component_label))
        actual_coarse_count = int(
            np.count_nonzero(coarse_component_label == component_label)
        )
        if fine_count != actual_fine_count or coarse_count != actual_coarse_count:
            raise RuntimeError("accepted_hole_coverage counts differ from archives")
        if not (0 < coarse_count <= fine_count <= fine_addition_count):
            raise RuntimeError("accepted_hole_coverage counts are inconsistent")
        if coarse_count > len(selection_index):
            raise RuntimeError("accepted_hole_coverage exceeds coarse addition count")
        coverage_rows.append(
            {
                "seed_id": seed_id,
                "component_label": component_label,
                "fine_count": fine_count,
                "coarse_addition_count": coarse_count,
                "bounds": list(bounds),
            }
        )
    if seen_ids != set(accepted_holes):
        raise RuntimeError("accepted_hole_coverage does not cover every accepted hole")
    return {
        "required_keys_verified": sorted(required),
        "fine_geometry_manifest_sha256_verified": True,
        "fine_geometry_archive_sha256_verified": True,
        "fine_geometry_candidate_sha256_verified": True,
        "fine_coarse_normalization_manifest_sha256_verified": True,
        "fine_addition_count_verified": True,
        "fine_selection_index_count_verified": True,
        "fine_selection_index_unique_verified": True,
        "coarse_xyz_subset_attestation_verified": True,
        "coarse_normals_subset_attestation_verified": True,
        "coarse_donor_nongeometry_attestation_verified": True,
        "coarse_component_labels_match_selected_fine_labels": True,
        "spacing_m": spacing,
        "maximum_support_distance_m": maximum_support_distance,
        "accepted_hole_coverage_rows_verified": len(coverage_rows),
        "accepted_hole_coverage": coverage_rows,
    }


def transfer_geometry_from_fine_suffix(
    *,
    coarse_initial_scan10: np.ndarray,
    coarse_geometry_archive_path: str | Path,
    coarse_geometry_manifest: Mapping[str, object],
    fine_enriched_candidate_path: str | Path,
    fine_enriched_manifest_path: str | Path,
    normalization_manifest_sha256: str,
) -> tuple[np.ndarray, Mapping[str, object]]:
    """Map 5mm geometry fields from the exact selected fine suffix rows."""

    cross_scale = coarse_geometry_manifest.get("cross_scale")
    if not isinstance(cross_scale, Mapping):
        raise ValueError("5mm geometry manifest has no cross_scale proof")
    fine_path = Path(fine_enriched_candidate_path).resolve(strict=True)
    fine_manifest_path = Path(fine_enriched_manifest_path).resolve(strict=True)
    fine_manifest_hash = sha256_path(fine_manifest_path)
    fine_document = json.loads(fine_manifest_path.read_text(encoding="utf-8"))
    if not isinstance(fine_document, Mapping):
        raise ValueError("fine enrichment manifest root must be an object")
    if (
        fine_document.get("operation")
        != "site1-v11-candidate-only-water-addition-scalar-enrichment"
    ):
        raise ValueError("fine manifest is not a WATER scalar-enrichment result")
    if str(fine_document.get("resolution_label")) not in {"1mm", "2mm"}:
        raise ValueError("fine enrichment manifest has an unexpected resolution")
    fine_invariants = fine_document.get("invariants")
    if not isinstance(fine_invariants, Mapping) or not bool(
        fine_invariants.get("geometry_metrics_from_local_cleanmesh")
    ):
        raise RuntimeError("fine manifest does not attest local CleanMesh geometry")
    fine_candidate = _manifest_entry(fine_document, "candidate")
    fine_candidate_hash = sha256_path(fine_path)
    _verify_declared_file(
        fine_candidate, fine_path, fine_candidate_hash, label="fine candidate"
    )
    fine_contract = _manifest_entry(fine_document, "geometry_contract")
    try:
        fine_base_points = int(fine_contract["base_points"])
        fine_addition_count = int(fine_contract["addition_count"])
        declared_fine_candidate_points = int(fine_contract["candidate_points"])
    except KeyError as error:
        raise ValueError("fine geometry_contract is incomplete") from error
    fine_layout = terrain.inspect_fixed_stride_ply(fine_path)
    if fine_layout.dtype != coarse_initial_scan10.dtype:
        raise ValueError("fine and coarse WATER schemas differ")
    if fine_layout.vertex_count != declared_fine_candidate_points:
        raise RuntimeError("fine candidate count disagrees with geometry_contract")
    if fine_base_points + fine_addition_count != fine_layout.vertex_count:
        raise RuntimeError("fine geometry_contract suffix bounds are inconsistent")
    declared_prefix_hash = fine_contract.get("base_payload_sha256")
    if not isinstance(declared_prefix_hash, str):
        raise ValueError("fine geometry_contract lacks base_payload_sha256")
    actual_prefix_hash = _payload_sha256(
        fine_layout, count=fine_base_points
    )
    if actual_prefix_hash != declared_prefix_hash:
        raise RuntimeError("fine candidate base prefix disagrees with geometry_contract")
    index, coarse_component_label = _load_cross_scale_mapping(
        Path(coarse_geometry_archive_path),
        expected_count=len(coarse_initial_scan10),
    )
    fine_component_label, component_membership_audit = (
        _load_fine_component_membership(
            fine_document,
            expected_count=fine_addition_count,
        )
    )
    cross_scale_verification = _verify_cross_scale_manifest(
        cross_scale,
        fine_document,
        fine_contract,
        index,
        fine_component_label,
        coarse_component_label,
        normalization_manifest_sha256,
    )
    if len(index) and (
        int(np.min(index)) < 0 or int(np.max(index)) >= fine_addition_count
    ):
        raise RuntimeError("fine_selection_index lies outside fine addition suffix")
    fine_memory = np.memmap(
        fine_layout.path,
        dtype=fine_layout.dtype,
        mode="r",
        offset=fine_layout.offset,
        shape=(fine_layout.vertex_count,),
    )
    selected = np.asarray(fine_memory[fine_base_points + index]).copy()
    del fine_memory
    for name in ("x", "y", "z", "nx", "ny", "nz"):
        if (
            np.asarray(selected[name]).tobytes()
            != np.asarray(coarse_initial_scan10[name]).tobytes()
        ):
            raise RuntimeError(
                f"coarse/fine cross-scale identity mismatch in {name}"
            )
    if len(selected) and not np.all(
        selected["scalar_ScanID"] == WATER_SCAN_ID
    ):
        raise RuntimeError("selected fine enrichment suffix is not WATER ScanID999")

    geometry_fields = _geometry_fields(coarse_initial_scan10.dtype)
    output = np.asarray(coarse_initial_scan10).copy()
    for name in geometry_fields:
        output[name] = selected[name]
    output["scalar_ScanID"] = np.asarray(
        WATER_SCAN_ID, dtype=output.dtype["scalar_ScanID"]
    )
    changed = {
        name: _changed_count(output, coarse_initial_scan10, name)
        for name in geometry_fields
    }
    if len(output) and geometry_fields and not any(changed.values()):
        raise RuntimeError("fine transfer left every coarse geometry value unchanged")
    for name in coarse_initial_scan10.dtype.names or ():
        if name in geometry_fields or name == "scalar_ScanID":
            continue
        if (
            np.asarray(output[name]).tobytes()
            != np.asarray(coarse_initial_scan10[name]).tobytes()
        ):
            raise RuntimeError(f"fine transfer changed coarse nongeometry field {name}")
    combined_fields = [
        f"scalar_A_R_{metric}_Combined"
        for metric in PHYSICAL_METRICS
        if f"scalar_A_R_{metric}_Combined" in geometry_fields
    ]
    combined_coverage = _finite_coverage(output, combined_fields)
    component_coverage = _require_component_field_coverage(
        output,
        coarse_component_label,
        geometry_fields,
        context="deterministic 5mm fine-subset transfer",
    )
    if combined_fields and any(
        float(combined_coverage[name]["fraction"])
        < MINIMUM_COMBINED_FINITE_FRACTION
        for name in combined_fields
    ):
        raise RuntimeError(
            "fine enrichment has insufficient finite combined-field coverage "
            "for deterministic 5mm transfer"
        )
    return output, {
        "method": "exact-fine-selection-index-geometry-transfer",
        "source_is_locally_cleanmesh_enriched_fine_suffix": True,
        "fine_candidate": {
            "path": str(fine_path),
            "sha256": fine_candidate_hash,
            "points": fine_layout.vertex_count,
            "base_points": fine_base_points,
            "addition_count": fine_addition_count,
            "base_payload_sha256": actual_prefix_hash,
        },
        "fine_manifest": {
            "path": str(fine_manifest_path),
            "sha256": fine_manifest_hash,
        },
        "fine_selection_index": {
            "archive_key": "fine_selection_index",
            "count": int(len(index)),
            "unique": True,
            "minimum": int(np.min(index)) if len(index) else None,
            "maximum": int(np.max(index)) if len(index) else None,
            "sha256": _records_sha256(index),
        },
        "component_membership": {
            **dict(component_membership_audit),
            "coarse_archive_key": "fine_component_label",
            "coarse_component_label_sha256": _records_sha256(
                coarse_component_label
            ),
            "coarse_labels_match_selected_fine_labels": True,
        },
        "xyz_and_normals_byte_exact": True,
        "geometry_fields_replaced": list(geometry_fields),
        "changed_points_by_field": changed,
        "combined_field_finite_coverage": combined_coverage,
        "minimum_combined_finite_fraction": MINIMUM_COMBINED_FINITE_FRACTION,
        "component_field_finite_coverage": component_coverage,
        "minimum_component_field_finite_fraction": (
            MINIMUM_COMPONENT_FIELD_FINITE_FRACTION
        ),
        "non_geometry_fields_archive_exact": True,
        "intensity_and_composition_archive_exact": True,
        "restored_scan_id": WATER_SCAN_ID,
        "cross_scale_manifest": dict(cross_scale),
        "cross_scale_verification": dict(cross_scale_verification),
    }


def _resolution_parameters(
    resolution_label: str,
    nominal_spacing_m: float,
    collar_m: float,
) -> terrain_pipeline.ResolutionParameters:
    cleanmesh_label = "1mm" if resolution_label in {"1mm", "2mm"} else "5mm"
    if cleanmesh_label == "5mm" and resolution_label != "5mm":
        raise ValueError("resolution_label must be 1mm, 2mm, or 5mm")
    return terrain_pipeline.ResolutionParameters(
        label=cleanmesh_label,
        nominal_spacing_m=float(nominal_spacing_m),
        cleanmesh_collar_m=float(collar_m),
    )


def _json_write(path: Path, value: object) -> None:
    path.write_text(
        json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )


def _fingerprint_mapping(paths: Mapping[str, Path]) -> dict[str, FileFingerprint]:
    return {name: fingerprint_file(path) for name, path in paths.items()}


def enrich_water_addition_scalars(
    *,
    base_water_path: str | Path,
    geometry_candidate_path: str | Path,
    geometry_manifest_path: str | Path,
    geometry_archive_path: str | Path,
    sand_path: str | Path,
    rock_path: str | Path,
    cleanmesh_executable: str | Path,
    normalization_manifest_path: str | Path,
    output_dir: str | Path,
    resolution_label: str,
    nominal_spacing_m: float,
    fine_enriched_candidate_path: str | Path | None = None,
    fine_enriched_manifest_path: str | Path | None = None,
    collar_m: float = 0.46,
    chunk_records: int = 1_000_000,
    cleanmesh_runner: CleanMeshRunner = terrain_pipeline.run_cleanmesh_reduced_analysis,
    require_scipy: bool = True,
) -> EnrichmentResult:
    """Build an atomic, candidate-only WATER scalar-enrichment directory."""

    destination = Path(output_dir).resolve(strict=False)
    if destination.exists():
        raise FileExistsError(f"refusing existing enrichment run: {destination}")
    if require_scipy:
        terrain_pipeline.require_production_runtime()
    if chunk_records <= 0:
        raise ValueError("chunk_records must be positive")
    destination.parent.mkdir(parents=True, exist_ok=True)
    inputs = {
        "base_water": Path(base_water_path).resolve(strict=True),
        "geometry_candidate": Path(geometry_candidate_path).resolve(strict=True),
        "geometry_manifest": Path(geometry_manifest_path).resolve(strict=True),
        "geometry_archive": Path(geometry_archive_path).resolve(strict=True),
        "sand": Path(sand_path).resolve(strict=True),
        "rock": Path(rock_path).resolve(strict=True),
        "cleanmesh": Path(cleanmesh_executable).resolve(strict=True),
        "normalization_manifest": Path(normalization_manifest_path).resolve(strict=True),
    }
    cross_scale_mode = resolution_label == "5mm"
    if cross_scale_mode:
        if fine_enriched_candidate_path is None or fine_enriched_manifest_path is None:
            raise ValueError(
                "5mm enrichment requires the completed fine candidate and manifest"
            )
        inputs["fine_enriched_candidate"] = Path(
            fine_enriched_candidate_path
        ).resolve(strict=True)
        inputs["fine_enriched_manifest"] = Path(
            fine_enriched_manifest_path
        ).resolve(strict=True)
        fine_preflight = json.loads(
            inputs["fine_enriched_manifest"].read_text(encoding="utf-8")
        )
        if not isinstance(fine_preflight, Mapping):
            raise ValueError("fine enrichment manifest root must be an object")
        fine_preflight_inputs = fine_preflight.get("input_fingerprints")
        if not isinstance(fine_preflight_inputs, Mapping):
            raise ValueError("fine enrichment manifest lacks input_fingerprints")
        for input_key, destination_key in (
            ("geometry_manifest", "fine_geometry_manifest"),
            ("geometry_archive", "fine_geometry_archive"),
        ):
            reference = fine_preflight_inputs.get(input_key)
            if not isinstance(reference, Mapping) or not isinstance(
                reference.get("path"), str
            ):
                raise ValueError(
                    f"fine enrichment manifest lacks {input_key} fingerprint path"
                )
            referenced_path = Path(str(reference["path"]))
            if not referenced_path.is_absolute():
                referenced_path = (
                    inputs["fine_enriched_manifest"].parent / referenced_path
                )
            inputs[destination_key] = referenced_path.resolve(strict=True)
    elif fine_enriched_candidate_path is not None or fine_enriched_manifest_path is not None:
        raise ValueError("fine enrichment inputs are only valid for 5mm transfer")
    if len(set(inputs.values())) != len(inputs):
        raise ValueError("enrichment input paths must be distinct")
    candidate_name = f"Site1-WATER-{resolution_label}.candidate.ply"
    density.assert_candidate_output_path(
        destination / candidate_name,
        source_paths=inputs.values(),
    )
    contract, archived = verify_append_only_geometry(
        base_water_path=inputs["base_water"],
        geometry_candidate_path=inputs["geometry_candidate"],
        geometry_manifest_path=inputs["geometry_manifest"],
        geometry_archive_path=inputs["geometry_archive"],
        chunk_records=chunk_records,
    )
    geometry_document = json.loads(
        inputs["geometry_manifest"].read_text(encoding="utf-8")
    )
    if not isinstance(geometry_document, Mapping):
        raise ValueError("geometry manifest root must be an object")
    if not contract.addition_count:
        raise ValueError("geometry candidate has no appended WATER rows to enrich")
    geometry_component_labels: np.ndarray | None = None
    geometry_component_membership_audit: Mapping[str, object] | None = None
    if not cross_scale_mode:
        geometry_component_labels, geometry_component_membership_audit = (
            _load_geometry_component_membership(
                geometry_document,
                inputs["geometry_archive"],
                expected_count=contract.addition_count,
            )
        )
    fingerprints = _fingerprint_mapping(inputs)
    implementation_paths = {
        Path(__file__).name: Path(__file__).resolve(),
        Path(terrain.__file__).name: Path(terrain.__file__).resolve(),
        Path(terrain_pipeline.__file__).name: Path(terrain_pipeline.__file__).resolve(),
        Path(hole_pipeline.__file__).name: Path(hole_pipeline.__file__).resolve(),
        Path(density.__file__).name: Path(density.__file__).resolve(),
        Path(terrain_pipeline.confidence.__file__).name: Path(
            terrain_pipeline.confidence.__file__
        ).resolve(),
    }
    implementation_hashes = {
        name: sha256_path(path) for name, path in implementation_paths.items()
    }
    normalizations, normalization_audit = (
        terrain_pipeline.load_combined_normalizations(
            inputs["normalization_manifest"]
        )
    )
    base_layout = terrain.inspect_fixed_stride_ply(inputs["base_water"])
    for role_path in (inputs["sand"], inputs["rock"]):
        if terrain.inspect_fixed_stride_ply(role_path).dtype != base_layout.dtype:
            raise ValueError("WATER/SAND/ROCK schemas differ")

    initial_scan10 = np.asarray(archived).copy()
    initial_scan10["scalar_ScanID"] = np.asarray(
        TEMPORARY_ADDITION_SCAN_ID,
        dtype=initial_scan10.dtype["scalar_ScanID"],
    )
    stage = Path(
        tempfile.mkdtemp(prefix=f".{destination.name}.staging-", dir=destination.parent)
    )
    try:
        shutil.copy2(inputs["geometry_manifest"], stage / "geometry-manifest.json")
        collar_specs = (
            ("water", inputs["base_water"], False, WATER_SCAN_ID, WATER_ANALYSIS_ROLE),
            ("sand", inputs["sand"], True, None, "SAND"),
            ("rock", inputs["rock"], True, None, "ROCK"),
        )
        selections = []
        collar_audit = {}
        for label, source, measured_only, required_scan, role in collar_specs:
            layout, indices, audit = collect_collar_indices(
                source,
                initial_scan10,
                collar_m=collar_m,
                measured_only=measured_only,
                required_scan_id=required_scan,
                chunk_records=chunk_records,
            )
            collar_audit[label] = audit
            if len(indices):
                selections.append(
                    terrain.make_local_source_selection(
                        layout,
                        indices,
                        role=role,
                        label=f"{label}-local-collar",
                    )
                )
        if not selections:
            raise RuntimeError("appended WATER geometry has no local analysis collar")
        analysis_input = stage / "water-additions.analysis-input.ply"
        local_manifest = terrain.write_local_analysis_input(
            analysis_input,
            selections,
            [
                terrain.LocalAdditionBatch(
                    initial_scan10, WATER_ANALYSIS_ROLE, "water-appended-geometry"
                )
            ],
            chunk_size=chunk_records,
        )
        analysed_path = stage / "water-additions.analysis.ply"
        cleanmesh_report = stage / "water-additions.cleanmesh-report.json"
        resolution = _resolution_parameters(
            resolution_label, nominal_spacing_m, collar_m
        )
        pipeline_parameters = terrain_pipeline.PipelineParameters(
            chunk_records=chunk_records
        )
        runner_audit = cleanmesh_runner(
            inputs["cleanmesh"],
            analysis_input,
            analysed_path,
            cleanmesh_report,
            resolution,
            pipeline_parameters,
        )
        analysed = collect_tagged_analysed_additions(
            analysed_path,
            initial_scan10,
            chunk_records=chunk_records,
        )
        diagnostic_fields = _geometry_fields(initial_scan10.dtype)
        cleanmesh_finite_coverage = _finite_coverage(analysed, diagnostic_fields)
        if cross_scale_mode:
            enriched, scalar_audit = transfer_geometry_from_fine_suffix(
                coarse_initial_scan10=initial_scan10,
                coarse_geometry_archive_path=inputs["geometry_archive"],
                coarse_geometry_manifest=geometry_document,
                fine_enriched_candidate_path=inputs["fine_enriched_candidate"],
                fine_enriched_manifest_path=inputs["fine_enriched_manifest"],
                normalization_manifest_sha256=fingerprints[
                    "normalization_manifest"
                ].sha256,
            )
            cleanmesh_output_policy = {
                "accepted_for_output": False,
                "purpose": "diagnostic-only-coarse-local-analysis",
                "reason": (
                    "5mm spacing under-supports CleanMesh's 50-point/20mm "
                    "fine fit; exact fine_selection_index transfer preserves "
                    "the validated 2mm local analysis"
                ),
                "finite_coverage_by_geometry_field": cleanmesh_finite_coverage,
            }
        else:
            enriched, scalar_audit = merge_analysed_geometry(
                analysed,
                initial_scan10,
                normalizations,
                component_labels=geometry_component_labels,
            )
            if geometry_component_membership_audit is None:
                raise RuntimeError("fine geometry component membership was not verified")
            scalar_audit = {
                **dict(scalar_audit),
                "geometry_component_membership": dict(
                    geometry_component_membership_audit
                ),
            }
            cleanmesh_output_policy = {
                "accepted_for_output": True,
                "purpose": "authoritative-fine-local-analysis",
                "finite_coverage_by_geometry_field": cleanmesh_finite_coverage,
            }
        candidate_stage = stage / candidate_name
        append_audit = _write_enriched_geometry_candidate(
            inputs["geometry_candidate"],
            enriched,
            candidate_stage,
            contract,
            chunk_records=chunk_records,
        )
        output_layout = terrain.inspect_fixed_stride_ply(candidate_stage)
        output_prefix_hash = _payload_sha256(
            output_layout,
            count=contract.base_points,
            chunk_records=chunk_records,
        )
        output_suffix_hash = _payload_sha256(
            output_layout,
            start=contract.base_points,
            count=contract.addition_count,
            chunk_records=chunk_records,
        )
        if output_prefix_hash != contract.base_payload_sha256:
            raise RuntimeError("enriched candidate changed existing WATER payload")
        if output_suffix_hash != _records_sha256(enriched):
            raise RuntimeError("enriched candidate suffix differs from enriched records")
        if output_layout.vertex_count != contract.candidate_points:
            raise RuntimeError("enriched candidate point count changed")
        output_memory = np.memmap(
            output_layout.path,
            dtype=output_layout.dtype,
            mode="r",
            offset=output_layout.offset,
            shape=(output_layout.vertex_count,),
        )
        output_suffix = np.asarray(output_memory[contract.base_points:]).copy()
        del output_memory
        if not np.all(output_suffix["scalar_ScanID"] == WATER_SCAN_ID):
            raise RuntimeError("enriched candidate suffix did not restore WATER ScanID999")
        non_geometry = [
            name
            for name in archived.dtype.names or ()
            if name not in _geometry_fields(archived.dtype) and name != "scalar_ScanID"
        ]
        for name in non_geometry:
            if np.asarray(output_suffix[name]).tobytes() != np.asarray(archived[name]).tobytes():
                raise RuntimeError(f"enriched candidate changed archived field {name}")

        for fingerprint in fingerprints.values():
            assert_fingerprint_unchanged(fingerprint)
        for name, path in implementation_paths.items():
            if sha256_path(path) != implementation_hashes[name]:
                raise RuntimeError(f"implementation changed during build: {path}")
        if contract.config_fingerprint is not None:
            config_path = Path(str(contract.config_fingerprint["path"]))
            if sha256_path(config_path) != contract.config_fingerprint["sha256"]:
                raise RuntimeError("geometry review config changed during enrichment")

        final_candidate = destination / candidate_name
        # Start from the exact upstream document so geometry evidence remains
        # available at its established top-level keys.  In particular this
        # preserves terrain_sources, accepted holes, archive mapping and the
        # fine-to-coarse cross_scale proof.  The original document is also
        # copied byte-for-byte beside this derived manifest.
        manifest = dict(geometry_document)
        upstream_candidate = geometry_document.get("candidate")
        upstream_operation = geometry_document.get("operation")
        upstream_invariants = geometry_document.get("invariants")
        manifest.update({
            "schema_version": max(int(geometry_document.get("schema_version", 1)), 1),
            "operation": "site1-v11-candidate-only-water-addition-scalar-enrichment",
            "status": "built",
            "candidate_only": True,
            "canonical_install_performed": False,
            "resolution_label": resolution_label,
            "nominal_spacing_m": float(nominal_spacing_m),
            "geometry_contract": asdict(contract),
            "geometry_manifest": {
                "path": str(inputs["geometry_manifest"]),
                "sha256": contract.manifest_sha256,
                "archived_copy": "geometry-manifest.json",
                "archived_copy_sha256": sha256_path(stage / "geometry-manifest.json"),
                "operation": upstream_operation,
                "candidate": upstream_candidate,
            },
            "input_fingerprints": {
                name: asdict(value) for name, value in fingerprints.items()
            },
            "scalar_enrichment_implementation": implementation_hashes,
            "parameters": {
                "semantic": {
                    "resolution_label": resolution_label,
                    "nominal_spacing_m": float(nominal_spacing_m),
                    "water_analysis_type_id": 1,
                    "temporary_addition_scan_id": TEMPORARY_ADDITION_SCAN_ID,
                    "final_water_scan_id": WATER_SCAN_ID,
                    "local_collar_m": float(collar_m),
                    "minimum_combined_finite_fraction": (
                        MINIMUM_COMBINED_FINITE_FRACTION
                    ),
                    "minimum_component_field_finite_fraction": (
                        MINIMUM_COMPONENT_FIELD_FINITE_FRACTION
                    ),
                    "undefined_geometry_fallback": {
                        "method": "component-strict-multiscale-3d-idw",
                        "radii_m": list(SCALAR_FALLBACK_RADII_M),
                        "component_diameter_bound_m": (
                            SCALAR_FALLBACK_COMPONENT_DIAMETER_BOUND_M
                        ),
                        "maximum_neighbours": SCALAR_FALLBACK_MAX_NEIGHBOURS,
                        "minimum_neighbours": SCALAR_FALLBACK_MIN_NEIGHBOURS,
                        "tiny_component_max_points": (
                            SCALAR_FALLBACK_TINY_COMPONENT_MAX_POINTS
                        ),
                        "cross_component_borrowing": False,
                        "imputed_values_may_become_donors": False,
                        "only_original_finite_values_may_be_donors": True,
                        "multi_hop_propagation": False,
                        "local_donor_range_bounded": True,
                    },
                    "coarse_geometry_source": (
                        "exact-fine-selection-index"
                        if cross_scale_mode
                        else "local-cleanmesh"
                    ),
                },
                "cleanmesh_invocation": {
                    "resolution_object": asdict(resolution),
                    "pipeline_object": asdict(pipeline_parameters),
                    "fields_read_by_production_runner": [
                        "resolution.cleanmesh_base_voxel_m",
                        "pipeline.cleanmesh_tile_width_m",
                        "pipeline.cleanmesh_chunk_points",
                        "pipeline.cleanmesh_normalization_samples",
                    ],
                },
                "execution_only": {
                    "outer_io_chunk_records": int(chunk_records),
                    "classification": (
                        "streaming, hashing and append batching only; it does not "
                        "change collar membership or scalar arithmetic"
                    ),
                },
            },
            "combined_geometry_normalization": dict(normalization_audit),
            "local_analysis": {
                "water_type_id": 1,
                "temporary_addition_scan_id": TEMPORARY_ADDITION_SCAN_ID,
                "collar_m": float(collar_m),
                "collars": collar_audit,
                "input": analysis_input.name,
                "input_sha256": sha256_path(analysis_input),
                "input_manifest": local_manifest.manifest_path.name,
                "input_manifest_sha256": sha256_path(local_manifest.manifest_path),
                "analysed": analysed_path.name,
                "analysed_sha256": sha256_path(analysed_path),
                "cleanmesh_report": cleanmesh_report.name,
                "cleanmesh_report_sha256": sha256_path(cleanmesh_report),
                "runner": dict(runner_audit),
                "full_cloud_analysis": False,
                "tagged_addition_count": int(len(analysed)),
                "tagged_identity_verified_after_tiled_output": True,
                "output_policy": cleanmesh_output_policy,
            },
            "scalar_enrichment": dict(scalar_audit),
            "candidate": {
                "path": str(final_candidate),
                "points": output_layout.vertex_count,
                "sha256": append_audit["candidate_sha256"],
                "base_payload_sha256": output_prefix_hash,
                "suffix_sha256": output_suffix_hash,
            },
            "invariants": {
                **(
                    dict(upstream_invariants)
                    if isinstance(upstream_invariants, Mapping)
                    else {}
                ),
                "geometry_candidate_verified_as_base_plus_archive": True,
                "existing_base_payload_byte_exact": True,
                "coordinates_and_normals_archive_exact": True,
                "colour_intensity_composition_archive_exact": True,
                "visibility_fields_archive_exact": True,
                "geometry_metrics_from_local_cleanmesh": True,
                "coarse_geometry_metrics_from_exact_fine_selection": (
                    bool(cross_scale_mode)
                ),
                "coarse_local_cleanmesh_is_diagnostic_only": (
                    bool(cross_scale_mode)
                ),
                "combined_metrics_use_v10_global_normalization": True,
                "undefined_geometry_fallback_component_strict": True,
                "undefined_geometry_fallback_no_extrapolation": True,
                "geometry_component_membership_verified": True,
                "component_field_scalar_coverage_complete": True,
                "component_field_scalar_ranges_verified": True,
                "final_addition_scan_id": WATER_SCAN_ID,
                "canonical_writes": False,
            },
        })
        manifest_stage = stage / "manifest.json"
        _json_write(manifest_stage, manifest)
        if destination.exists():
            raise FileExistsError(f"enrichment destination appeared: {destination}")
        os.replace(stage, destination)
    except BaseException:
        if stage.exists():
            shutil.rmtree(stage)
        raise
    return EnrichmentResult(
        output_dir=destination,
        candidate_path=destination / candidate_name,
        manifest_path=destination / "manifest.json",
        candidate_points=contract.candidate_points,
        addition_count=contract.addition_count,
        candidate_sha256=str(append_audit["candidate_sha256"]),
    )


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Recompute local CleanMesh A_R fields for appended WATER geometry"
    )
    parser.add_argument("--base-water", required=True, type=Path)
    parser.add_argument("--geometry-candidate", required=True, type=Path)
    parser.add_argument("--geometry-manifest", required=True, type=Path)
    parser.add_argument("--geometry-archive", required=True, type=Path)
    parser.add_argument("--sand", required=True, type=Path)
    parser.add_argument("--rock", required=True, type=Path)
    parser.add_argument("--cleanmesh", required=True, type=Path)
    parser.add_argument("--normalization-manifest", required=True, type=Path)
    parser.add_argument(
        "--fine-enriched-candidate",
        type=Path,
        help="required for 5mm: completed 2mm enriched WATER candidate",
    )
    parser.add_argument(
        "--fine-enriched-manifest",
        type=Path,
        help="required for 5mm: manifest for the completed 2mm enrichment",
    )
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--resolution", required=True, choices=("1mm", "2mm", "5mm"))
    parser.add_argument("--spacing", required=True, type=float)
    parser.add_argument("--collar", type=float, default=0.46)
    parser.add_argument("--chunk-records", type=int, default=1_000_000)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    result = enrich_water_addition_scalars(
        base_water_path=args.base_water,
        geometry_candidate_path=args.geometry_candidate,
        geometry_manifest_path=args.geometry_manifest,
        geometry_archive_path=args.geometry_archive,
        sand_path=args.sand,
        rock_path=args.rock,
        cleanmesh_executable=args.cleanmesh,
        normalization_manifest_path=args.normalization_manifest,
        output_dir=args.output,
        resolution_label=args.resolution,
        nominal_spacing_m=args.spacing,
        fine_enriched_candidate_path=args.fine_enriched_candidate,
        fine_enriched_manifest_path=args.fine_enriched_manifest,
        collar_m=args.collar,
        chunk_records=args.chunk_records,
    )
    print(json.dumps(asdict(result), indent=2, default=str))
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())


__all__ = [
    "EnrichmentResult",
    "FileFingerprint",
    "GeometryContract",
    "collect_collar_indices",
    "collect_tagged_analysed_additions",
    "enrich_water_addition_scalars",
    "merge_analysed_geometry",
    "transfer_geometry_from_fine_suffix",
    "verify_append_only_geometry",
]
