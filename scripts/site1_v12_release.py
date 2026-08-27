#!/usr/bin/env python3
"""Safely publish or restore the two Scene1 v12 WATER clouds.

This module is deliberately narrower than the v11 six-cloud release tool.  It
can mutate only ``Site1-WATER-2mm.ply`` and ``Site1-WATER-5mm.ply``.  Before a
release is built it independently verifies the native CleanMesh downsample
report and proves, record for record, that the 5 mm candidate is an ordered
subsequence of the 2 mm candidate.  PLY comments and other header text may
differ; payload records may not.

Publication is also contingent on the fixed v12 geometry, fine scalar,
post-build terrain/WATER interface-audit, and downsample-stage manifests.  The
release snapshots their compact, hash-locked provenance so verification still
works after installation moves the candidates and approved cleanup retires
large intermediate geometry/analysis clouds.

``build`` fingerprints all inputs and creates immutable clone-or-copy rollback
snapshots.  ``install`` and ``restore`` use durable rename journals.  Restore
material is cloned from the release snapshots rather than trusted from a
mutable transaction manifest.  Startup recovery either proves a transaction
committed or returns every path to its pre-transaction generation; ambiguous
states fail closed.  The historic ``Site1-WATER-5mm-old01.ply`` is only
fingerprinted and is never a transaction target.
"""

from __future__ import annotations

import argparse
from contextlib import contextmanager
from dataclasses import dataclass
import datetime as dt
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import stat
import subprocess
import sys
import tempfile
from typing import Callable, Mapping, Sequence
import uuid

import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import site1_v11_water_scalar_enrichment as scalar_enrichment  # noqa: E402
import site1_v12_interface_audit as interface_audit  # noqa: E402

ROOT = SCRIPT_DIR.parent
DEFAULT_DATA = ROOT / "Data" / "Scene1"
DEFAULT_RUN = (
    DEFAULT_DATA
    / "PatchRefinement"
    / "20260827-site1-v12-water-interface"
)
DEFAULT_RELEASE = DEFAULT_RUN / "release"
DEFAULT_FINE_CANDIDATE = (
    DEFAULT_RUN / "water-final-2mm" / "Site1-WATER-2mm.candidate.ply"
)
DEFAULT_COARSE_CANDIDATE = (
    DEFAULT_RUN / "water-final-5mm" / "Site1-WATER-5mm.candidate.ply"
)
DEFAULT_DOWNSAMPLE_REPORT = (
    DEFAULT_RUN / "water-final-5mm" / "downsample-report.json"
)
DEFAULT_FINE_MANIFEST = DEFAULT_RUN / "water-final-2mm" / "manifest.json"
DEFAULT_GEOMETRY_MANIFEST = DEFAULT_RUN / "water-geometry-2mm" / "manifest.json"
DEFAULT_GEOMETRY_ARCHIVE = DEFAULT_RUN / "water-geometry-2mm" / "additions.npz"
DEFAULT_INTERFACE_AUDIT_MANIFEST = DEFAULT_RUN / "interface-audit" / "manifest.json"
DEFAULT_DOWNSAMPLE_MANIFEST = DEFAULT_RUN / "water-final-5mm" / "stage-manifest.json"
DEFAULT_REVIEW_CONFIG = SCRIPT_DIR / "config" / "site1_fossils_v12_review.json"
DEFAULT_V11_RUN = (
    DEFAULT_DATA
    / "PatchRefinement"
    / "20260827-site1-v11-density-terrain-obstructions"
)
DEFAULT_NORMALIZATION_MANIFEST = (
    DEFAULT_V11_RUN / "terrain" / "normalization-manifest.json"
)
DEFAULT_CLEANMESH_EXECUTABLE = Path(
    "/Users/juju/Documents/Repositories/CleanMesh/build-release/"
    "cleanmesh_reduced_analysis"
)
DEFAULT_DOWNSAMPLE_EXECUTABLE = Path(
    "/Users/juju/Documents/Repositories/CleanMesh/build-release/"
    "cleanmesh_spatial_downsample"
)

SCHEMA_VERSION = 3
JOURNAL_SCHEMA_VERSION = 1
OPERATION = "site1-v12-water-only-safe-release"
JOURNAL_OPERATION = "site1-v12-durable-two-water-transaction"
MINIMUM_COARSE_SPACING_M = 0.005
CHUNK_RECORDS = 250_000

CANONICAL_BY_LABEL = {
    "WATER-2mm": "Site1-WATER-2mm.ply",
    "WATER-5mm": "Site1-WATER-5mm.ply",
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


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise RuntimeError(message)


def _now() -> str:
    return dt.datetime.now().isoformat(timespec="seconds")


def _lexical_absolute(path: str | Path, label: str) -> Path:
    value = Path(path)
    if not value.is_absolute():
        value = Path.cwd() / value
    result = Path(os.path.abspath(os.fspath(value)))
    _require(result.is_absolute(), f"{label} must be absolute")
    return result


def _strict_existing_directory(path: str | Path, label: str) -> Path:
    lexical = _lexical_absolute(path, label)
    _require(not lexical.is_symlink(), f"{label} may not be a symlink: {lexical}")
    resolved = lexical.resolve(strict=True)
    _require(resolved == lexical, f"{label} traverses a path alias: {lexical}")
    _require(resolved.is_dir(), f"{label} is not a directory: {resolved}")
    return resolved


def _strict_existing_file(path: str | Path, label: str) -> Path:
    lexical = _lexical_absolute(path, label)
    _require(not lexical.is_symlink(), f"{label} may not be a symlink: {lexical}")
    resolved = lexical.resolve(strict=True)
    _require(resolved == lexical, f"{label} traverses a path alias: {lexical}")
    mode = resolved.stat().st_mode
    _require(stat.S_ISREG(mode), f"{label} is not a regular file: {resolved}")
    return resolved


def _strict_absent_path(path: str | Path, label: str) -> Path:
    lexical = _lexical_absolute(path, label)
    parent = _strict_existing_directory(lexical.parent, f"{label} parent")
    result = parent / lexical.name
    if result.exists() or result.is_symlink():
        raise FileExistsError(f"{label} already exists: {result}")
    return result


def _parent_resolved_path(path: str | Path, label: str) -> Path:
    lexical = _lexical_absolute(path, label)
    parent = _strict_existing_directory(lexical.parent, f"{label} parent")
    result = parent / lexical.name
    _require(not result.is_symlink(), f"{label} may not be a symlink: {result}")
    return result


def _require_beneath(path: Path, root: Path, label: str) -> None:
    try:
        path.relative_to(root)
    except ValueError as error:
        raise RuntimeError(f"{label} escapes {root}: {path}") from error


def _inode(path: Path) -> tuple[int, int]:
    value = path.stat()
    return int(value.st_dev), int(value.st_ino)


def _require_distinct_existing(paths: Mapping[str, Path]) -> None:
    lexical: dict[Path, str] = {}
    inodes: dict[tuple[int, int], str] = {}
    for label, path in paths.items():
        if path in lexical:
            raise RuntimeError(f"{label} aliases {lexical[path]}: {path}")
        lexical[path] = label
        identity = _inode(path)
        if identity in inodes:
            raise RuntimeError(
                f"{label} hard-link aliases {inodes[identity]}: {path}"
            )
        inodes[identity] = label


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


def inspect_ply(path: str | Path) -> PlyInfo:
    """Inspect a fixed-stride binary-little-endian point-cloud PLY."""

    source = Path(path).resolve(strict=True)
    _require(source.is_file(), f"PLY is not a regular file: {source}")
    header = bytearray()
    with source.open("rb") as handle:
        for _ in range(200_000):
            line = handle.readline()
            if not line:
                raise ValueError(f"{source}: truncated PLY header")
            header.extend(line)
            if len(header) > 16 * 1024 * 1024:
                raise ValueError(f"{source}: unreasonably large PLY header")
            if line.rstrip(b"\r\n") == b"end_header":
                break
        else:
            raise ValueError(f"{source}: unterminated PLY header")
        offset = handle.tell()
    try:
        lines = bytes(header).decode("ascii").splitlines()
    except UnicodeDecodeError as error:
        raise ValueError(f"{source}: PLY header is not ASCII") from error
    if not lines or lines[0] != "ply":
        raise ValueError(f"{source}: not a PLY")
    if "format binary_little_endian 1.0" not in lines:
        raise ValueError(f"{source}: expected binary_little_endian 1.0")

    current: str | None = None
    vertex_count: int | None = None
    fields: list[tuple[str, str]] = []
    other_nonzero: list[str] = []
    for line in lines:
        parts = line.split()
        if parts[:1] == ["element"]:
            if len(parts) != 3:
                raise ValueError(f"{source}: malformed element declaration")
            current = parts[1]
            try:
                count = int(parts[2])
            except ValueError as error:
                raise ValueError(f"{source}: invalid element count") from error
            if count < 0:
                raise ValueError(f"{source}: negative element count")
            if current == "vertex":
                if vertex_count is not None:
                    raise ValueError(f"{source}: duplicate vertex element")
                vertex_count = count
            elif count:
                other_nonzero.append(current)
        elif parts[:1] == ["property"] and current == "vertex":
            if len(parts) != 3 or parts[1] == "list":
                raise ValueError(f"{source}: variable-width vertex property")
            if parts[1] not in _PLY_TYPES:
                raise ValueError(f"{source}: unsupported PLY type {parts[1]}")
            fields.append((parts[2], _PLY_TYPES[parts[1]]))

    if vertex_count is None or not fields:
        raise ValueError(f"{source}: missing vertex schema/count")
    if other_nonzero:
        raise ValueError(
            f"{source}: non-point payload elements are not supported: "
            + ", ".join(other_nonzero)
        )
    try:
        dtype = np.dtype(fields, align=False)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{source}: invalid or duplicate vertex properties") from error
    missing = sorted({"x", "y", "z"} - set(dtype.names or ()))
    if missing:
        raise ValueError(f"{source}: missing XYZ properties {missing}")
    size = source.stat().st_size
    expected_size = offset + vertex_count * dtype.itemsize
    if size != expected_size:
        raise ValueError(
            f"{source}: byte size {size} != fixed payload size {expected_size}"
        )
    return PlyInfo(source, vertex_count, offset, dtype, size)


def sha256_path(path: str | Path, block_size: int = 32 * 1024 * 1024) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        while block := handle.read(block_size):
            digest.update(block)
    return digest.hexdigest()


def _stat_identity(path: Path) -> tuple[int, int, int, int]:
    value = path.stat()
    return value.st_dev, value.st_ino, value.st_size, value.st_mtime_ns


def file_fingerprint(path: str | Path, *, ply: bool | None = None) -> dict:
    source = Path(path).resolve(strict=True)
    before = _stat_identity(source)
    result: dict[str, object] = {
        "path": str(source),
        "bytes": int(source.stat().st_size),
        "sha256": sha256_path(source),
    }
    is_ply = source.suffix.lower() == ".ply" if ply is None else bool(ply)
    if is_ply:
        info = inspect_ply(source)
        result.update(
            points=int(info.count),
            record_stride=int(info.stride),
            schema=[list(item) for item in info.schema],
        )
    if before != _stat_identity(source):
        raise RuntimeError(f"file changed while fingerprinting: {source}")
    return result


_GENERIC_FINGERPRINT_KEYS = ("bytes", "sha256")
_PLY_FINGERPRINT_KEYS = ("points", "record_stride", "schema")
_FINGERPRINT_KEYS = _GENERIC_FINGERPRINT_KEYS + _PLY_FINGERPRINT_KEYS


def _fingerprint_keys(value: Mapping, *, ply: bool | None = None) -> tuple[str, ...]:
    """Return the complete key contract for a release fingerprint.

    A partial mapping must never turn comparison into a vacuous success.  PLY
    fingerprints are all-or-nothing: the point count, fixed record stride and
    schema travel together with the content hash.
    """

    _require(isinstance(value, Mapping), "fingerprint must be a mapping")
    has_ply_key = any(key in value for key in _PLY_FINGERPRINT_KEYS)
    use_ply = has_ply_key if ply is None else bool(ply)
    keys = _FINGERPRINT_KEYS if use_ply else _GENERIC_FINGERPRINT_KEYS
    missing = [key for key in keys if key not in value]
    _require(not missing, "fingerprint missing required keys: " + ", ".join(missing))
    if not use_ply:
        partial_ply = [key for key in _PLY_FINGERPRINT_KEYS if key in value]
        _require(not partial_ply, "generic fingerprint contains partial PLY fields")
    return keys


def _same_fingerprint(left: Mapping, right: Mapping) -> bool:
    try:
        use_ply = any(
            key in value
            for value in (left, right)
            for key in _PLY_FINGERPRINT_KEYS
        )
        keys = _fingerprint_keys(left, ply=use_ply)
        _fingerprint_keys(right, ply=use_ply)
    except (RuntimeError, TypeError):
        return False
    return all(left.get(key) == right.get(key) for key in keys)


def _same_content(left: Mapping, right: Mapping) -> bool:
    """Compare only the mandatory byte/hash identity across fingerprint formats."""

    if not isinstance(left, Mapping) or not isinstance(right, Mapping):
        return False
    if any(key not in left or key not in right for key in _GENERIC_FINGERPRINT_KEYS):
        return False
    return (
        left.get("bytes") == right.get("bytes")
        and left.get("sha256") == right.get("sha256")
    )


def _assert_fingerprint(path: str | Path, expected: Mapping, label: str) -> dict:
    source = _strict_existing_file(path, label)
    use_ply = source.suffix.lower() == ".ply"
    keys = _fingerprint_keys(expected, ply=use_ply)
    actual = file_fingerprint(source, ply=use_ply)
    mismatched = [
        key for key in keys if actual.get(key) != expected.get(key)
    ]
    if mismatched:
        raise RuntimeError(f"{label} fingerprint drift: {', '.join(mismatched)}")
    return actual


def _json_pairs(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON key {key!r}")
        result[key] = value
    return result


def _load_json(path: str | Path, label: str) -> tuple[Path, dict]:
    source = _strict_existing_file(path, label)
    try:
        value = json.loads(
            source.read_text(encoding="utf-8"), object_pairs_hook=_json_pairs
        )
    except (json.JSONDecodeError, UnicodeDecodeError, ValueError) as error:
        raise RuntimeError(f"{label} is invalid JSON: {source}: {error}") from error
    _require(isinstance(value, dict), f"{label} must be a JSON object: {source}")
    return source, value


def _fsync_directory(path: Path) -> None:
    descriptor = os.open(path, os.O_RDONLY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _durable_replace(source: Path, destination: Path) -> None:
    source_parent = source.parent.resolve(strict=True)
    destination_parent = destination.parent.resolve(strict=True)
    os.replace(source, destination)
    _fsync_directory(destination_parent)
    if source_parent != destination_parent:
        _fsync_directory(source_parent)


def _atomic_json(path: Path, value: Mapping, *, overwrite: bool = False) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    _strict_existing_directory(path.parent, f"JSON output parent for {path.name}")
    if not overwrite and (path.exists() or path.is_symlink()):
        raise FileExistsError(f"refusing to overwrite JSON output: {path}")
    if path.is_symlink():
        raise RuntimeError(f"refusing symlink JSON output: {path}")
    temporary = path.with_name(f".{path.name}.{os.getpid()}.{uuid.uuid4().hex}.partial")
    _require(not temporary.exists(), f"temporary JSON path exists: {temporary}")
    payload = json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n"
    with temporary.open("x", encoding="utf-8") as handle:
        handle.write(payload)
        handle.flush()
        os.fsync(handle.fileno())
    _durable_replace(temporary, path)


def _record_keys(records: np.ndarray, stride: int) -> np.ndarray:
    """Return a compact exact-byte prefix key used only as a search filter."""

    width = min(16, stride)
    raw = records.view(np.uint8).reshape(len(records), stride)
    return np.ascontiguousarray(raw[:, :width]).view(np.dtype((np.void, width))).reshape(-1)


def verify_ordered_record_subsequence(
    fine_path: str | Path,
    coarse_path: str | Path,
    *,
    chunk_records: int = CHUNK_RECORDS,
) -> dict:
    """Prove that every coarse payload record occurs in fine payload order.

    Input is read in bounded blocks.  A compact record prefix is used to avoid
    Python work for non-matches, but every accepted match compares the complete
    fixed-stride record.  Prefix collisions therefore cannot produce a false
    positive.
    """

    _require(chunk_records > 0, "chunk_records must be positive")
    fine = inspect_ply(fine_path)
    coarse = inspect_ply(coarse_path)
    _require(fine.schema == coarse.schema, "fine/coarse PLY schemas differ")
    _require(fine.stride == coarse.stride, "fine/coarse record strides differ")
    _require(coarse.count <= fine.count, "coarse candidate has more records than fine")
    void_dtype = np.dtype((np.void, fine.stride))

    matched = 0
    source_records_read = 0
    last_source_index = -1
    fine_buffer = np.empty(0, dtype=void_dtype)
    fine_position = 0

    with fine.path.open("rb") as fine_handle, coarse.path.open("rb") as coarse_handle:
        fine_handle.seek(fine.offset)
        coarse_handle.seek(coarse.offset)
        while matched < coarse.count:
            requested = min(chunk_records, coarse.count - matched)
            payload = coarse_handle.read(requested * fine.stride)
            if len(payload) != requested * fine.stride:
                raise RuntimeError("coarse payload ended before its declared count")
            coarse_records = np.frombuffer(payload, dtype=void_dtype, count=requested)
            coarse_position = 0

            while coarse_position < requested:
                if fine_position >= len(fine_buffer):
                    remaining = fine.count - source_records_read
                    if remaining <= 0:
                        raise RuntimeError(
                            "5mm payload is not an ordered full-record subsequence; "
                            f"first unmatched coarse record is {matched + coarse_position}"
                        )
                    count = min(chunk_records, remaining)
                    fine_payload = fine_handle.read(count * fine.stride)
                    if len(fine_payload) != count * fine.stride:
                        raise RuntimeError("fine payload ended before its declared count")
                    fine_buffer = np.frombuffer(
                        fine_payload, dtype=void_dtype, count=count
                    )
                    fine_position = 0
                    fine_block_start = source_records_read
                    source_records_read += count
                else:
                    fine_block_start = source_records_read - len(fine_buffer)

                available_fine = fine_buffer[fine_position:]
                wanted = coarse_records[coarse_position:]
                fine_keys = _record_keys(available_fine, fine.stride)
                wanted_keys = _record_keys(wanted, fine.stride)
                candidate_offsets = np.flatnonzero(
                    np.isin(fine_keys, wanted_keys, assume_unique=False)
                )
                if not len(candidate_offsets):
                    fine_position = len(fine_buffer)
                    continue

                start_fine_position = fine_position
                completed_block = False
                for relative in candidate_offsets:
                    absolute = start_fine_position + int(relative)
                    if fine_buffer[absolute] == coarse_records[coarse_position]:
                        last_source_index = fine_block_start + absolute
                        coarse_position += 1
                        fine_position = absolute + 1
                        if coarse_position == requested:
                            completed_block = True
                            break
                if completed_block:
                    break
                # All candidate keys in this fine tail have been examined.  A
                # record skipped before the next required record cannot be used
                # by a later ordered match.
                fine_position = len(fine_buffer)

            matched += requested

        if coarse_handle.read(1):
            raise RuntimeError("coarse payload contains undeclared trailing bytes")

    _require(matched == coarse.count, "coarse subsequence verification was incomplete")
    return {
        "verified": True,
        "relation": "byte-exact ordered full-record subsequence",
        "fine_points": int(fine.count),
        "coarse_points": int(coarse.count),
        "matched_points": int(matched),
        "record_stride": int(fine.stride),
        "last_matched_fine_index": int(last_source_index),
        "header_bytes_may_differ": True,
    }


def _finite_number(value: object, label: str) -> float:
    _require(not isinstance(value, bool), f"{label} is not numeric")
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise RuntimeError(f"{label} is not numeric") from error
    _require(math.isfinite(result), f"{label} is not finite")
    return result


def _integer(value: object, label: str) -> int:
    _require(not isinstance(value, bool), f"{label} is not an integer")
    try:
        result = int(value)
    except (TypeError, ValueError) as error:
        raise RuntimeError(f"{label} is not an integer") from error
    _require(result == value, f"{label} is not an exact integer")
    return result


def verify_cleanmesh_downsample_report(
    report_path: str | Path,
    fine_path: str | Path,
    coarse_path: str | Path,
    *,
    required_spacing_m: float = MINIMUM_COARSE_SPACING_M,
) -> dict:
    """Verify the native ``cleanmesh_spatial_downsample`` JSON report."""

    report_source, report = _load_json(report_path, "CleanMesh downsample report")
    fine_source = _strict_existing_file(fine_path, "fine WATER candidate")
    coarse_source = _strict_existing_file(coarse_path, "coarse WATER candidate")
    fine = inspect_ply(fine_source)
    coarse = inspect_ply(coarse_source)

    _require(
        report.get("method") == "greedy_spatial_minimum_distance",
        "unexpected CleanMesh downsample method",
    )
    spacing = _finite_number(report.get("minimum_spacing_m"), "minimum_spacing_m")
    _require(
        math.isclose(spacing, required_spacing_m, rel_tol=0.0, abs_tol=1.0e-12),
        f"CleanMesh downsample spacing {spacing} != {required_spacing_m}",
    )
    source_points = _integer(report.get("source_points"), "source_points")
    output_points = _integer(report.get("output_points"), "output_points")
    stride = _integer(report.get("record_stride"), "record_stride")
    _require(source_points == fine.count, "downsample source count differs from fine PLY")
    _require(output_points == coarse.count, "downsample output count differs from coarse PLY")
    _require(stride == fine.stride == coarse.stride, "downsample record stride differs")
    _require(
        _integer(report.get("non_finite_positions"), "non_finite_positions") == 0,
        "CleanMesh downsample encountered non-finite positions",
    )

    report_input = _strict_existing_file(report.get("input", ""), "downsample report input")
    report_output = _strict_existing_file(report.get("output", ""), "downsample report output")
    _require(report_input == fine_source, "downsample report input is not the fine candidate")
    _require(report_output == coarse_source, "downsample report output is not the coarse candidate")
    return {
        "verified": True,
        "report": file_fingerprint(report_source, ply=False),
        "method": report["method"],
        "minimum_spacing_m": spacing,
        "source_points": source_points,
        "output_points": output_points,
        "record_stride": stride,
        "non_finite_positions": 0,
        "input": str(fine_source),
        "output": str(coarse_source),
    }


@dataclass(frozen=True)
class ReleasePaths:
    data_dir: Path
    run_dir: Path
    release_dir: Path
    candidate_2mm: Path
    candidate_5mm: Path
    downsample_report: Path
    fine_manifest: Path
    geometry_manifest: Path
    geometry_archive: Path
    interface_audit_manifest: Path
    downsample_manifest: Path
    review_config: Path
    normalization_manifest: Path
    cleanmesh_executable: Path
    downsample_executable: Path


def _coerce_paths(args) -> ReleasePaths:
    data_dir = _lexical_absolute(args.data_dir, "data directory")
    run_dir = _lexical_absolute(args.run_dir, "run directory")
    release_value = getattr(args, "release_dir", None)
    release_dir = _lexical_absolute(
        release_value if release_value is not None else run_dir / "release",
        "release directory",
    )
    fine_value = getattr(args, "candidate_2mm", None)
    coarse_value = getattr(args, "candidate_5mm", None)
    report_value = getattr(args, "downsample_report", None)
    fine_manifest_value = getattr(args, "fine_manifest", None)
    geometry_manifest_value = getattr(args, "geometry_manifest", None)
    geometry_archive_value = getattr(args, "geometry_archive", None)
    interface_audit_manifest_value = getattr(args, "interface_audit_manifest", None)
    downsample_manifest_value = getattr(args, "downsample_manifest", None)
    candidate_2mm = _lexical_absolute(
        fine_value
        if fine_value is not None
        else run_dir / "water-final-2mm" / "Site1-WATER-2mm.candidate.ply",
        "fine candidate",
    )
    candidate_5mm = _lexical_absolute(
        coarse_value
        if coarse_value is not None
        else run_dir / "water-final-5mm" / "Site1-WATER-5mm.candidate.ply",
        "coarse candidate",
    )
    downsample_report = _lexical_absolute(
        report_value
        if report_value is not None
        else run_dir / "water-final-5mm" / "downsample-report.json",
        "downsample report",
    )
    fine_manifest = _lexical_absolute(
        fine_manifest_value
        if fine_manifest_value is not None
        else run_dir / "water-final-2mm" / "manifest.json",
        "fine scalar manifest",
    )
    geometry_manifest = _lexical_absolute(
        geometry_manifest_value
        if geometry_manifest_value is not None
        else run_dir / "water-geometry-2mm" / "manifest.json",
        "geometry manifest",
    )
    geometry_archive = _lexical_absolute(
        geometry_archive_value
        if geometry_archive_value is not None
        else run_dir / "water-geometry-2mm" / "additions.npz",
        "geometry archive",
    )
    interface_audit_manifest = _lexical_absolute(
        interface_audit_manifest_value
        if interface_audit_manifest_value is not None
        else run_dir / "interface-audit" / "manifest.json",
        "interface audit manifest",
    )
    downsample_manifest = _lexical_absolute(
        downsample_manifest_value
        if downsample_manifest_value is not None
        else run_dir / "water-final-5mm" / "stage-manifest.json",
        "downsample stage manifest",
    )
    review_config = _lexical_absolute(
        getattr(args, "review_config", DEFAULT_REVIEW_CONFIG),
        "v12 review config",
    )
    normalization_manifest = _lexical_absolute(
        getattr(args, "normalization_manifest", DEFAULT_NORMALIZATION_MANIFEST),
        "normalization manifest",
    )
    cleanmesh_executable = _lexical_absolute(
        getattr(args, "cleanmesh", DEFAULT_CLEANMESH_EXECUTABLE),
        "CleanMesh reduced-analysis executable",
    )
    downsample_executable = _lexical_absolute(
        getattr(args, "downsample", DEFAULT_DOWNSAMPLE_EXECUTABLE),
        "CleanMesh downsample executable",
    )
    _require(release_dir == run_dir / "release", "release directory must be <run>/release")
    fixed_run_paths = {
        "fine candidate": (
            candidate_2mm,
            run_dir / "water-final-2mm" / "Site1-WATER-2mm.candidate.ply",
        ),
        "coarse candidate": (
            candidate_5mm,
            run_dir / "water-final-5mm" / "Site1-WATER-5mm.candidate.ply",
        ),
        "downsample report": (
            downsample_report,
            run_dir / "water-final-5mm" / "downsample-report.json",
        ),
        "fine scalar manifest": (
            fine_manifest,
            run_dir / "water-final-2mm" / "manifest.json",
        ),
        "geometry manifest": (
            geometry_manifest,
            run_dir / "water-geometry-2mm" / "manifest.json",
        ),
        "geometry archive": (
            geometry_archive,
            run_dir / "water-geometry-2mm" / "additions.npz",
        ),
        "interface audit manifest": (
            interface_audit_manifest,
            run_dir / "interface-audit" / "manifest.json",
        ),
        "downsample stage manifest": (
            downsample_manifest,
            run_dir / "water-final-5mm" / "stage-manifest.json",
        ),
    }
    for label, (path, expected) in fixed_run_paths.items():
        _require(path == expected, f"{label} must use the fixed v12 stage path")
        _require_beneath(path, run_dir, label)
        _require(not path.is_relative_to(release_dir), f"{label} may not be inside release")
    _require(
        review_config == DEFAULT_REVIEW_CONFIG,
        "v12 review config must use the fixed repository path",
    )
    return ReleasePaths(
        data_dir, run_dir, release_dir, candidate_2mm, candidate_5mm,
        downsample_report, fine_manifest, geometry_manifest, geometry_archive,
        interface_audit_manifest, downsample_manifest, review_config, normalization_manifest,
        cleanmesh_executable, downsample_executable,
    )


def _canonical(paths: ReleasePaths, label: str) -> Path:
    return paths.data_dir / CANONICAL_BY_LABEL[label]


def _old01(paths: ReleasePaths) -> Path:
    return paths.data_dir / "Site1-WATER-5mm-old01.ply"


def _validate_build_paths(paths: ReleasePaths) -> dict[str, Path]:
    data = _strict_existing_directory(paths.data_dir, "data directory")
    run = _strict_existing_directory(paths.run_dir, "run directory")
    _require(paths.release_dir.parent == run, "release parent differs from run directory")
    _strict_absent_path(paths.release_dir, "release directory")
    existing = {
        "WATER-2mm canonical": _strict_existing_file(_canonical(paths, "WATER-2mm"), "WATER-2mm canonical"),
        "WATER-5mm canonical": _strict_existing_file(_canonical(paths, "WATER-5mm"), "WATER-5mm canonical"),
        "WATER-2mm candidate": _strict_existing_file(paths.candidate_2mm, "WATER-2mm candidate"),
        "WATER-5mm candidate": _strict_existing_file(paths.candidate_5mm, "WATER-5mm candidate"),
        "CleanMesh report": _strict_existing_file(paths.downsample_report, "CleanMesh report"),
        "fine scalar manifest": _strict_existing_file(paths.fine_manifest, "fine scalar manifest"),
        "geometry manifest": _strict_existing_file(paths.geometry_manifest, "geometry manifest"),
        "geometry archive": _strict_existing_file(paths.geometry_archive, "geometry archive"),
        "interface audit manifest": _strict_existing_file(
            paths.interface_audit_manifest, "interface audit manifest"
        ),
        "downsample stage manifest": _strict_existing_file(paths.downsample_manifest, "downsample stage manifest"),
        "fine geometry manifest copy": _strict_existing_file(
            paths.fine_manifest.parent / "geometry-manifest.json",
            "fine geometry manifest copy",
        ),
        "v12 review config": _strict_existing_file(paths.review_config, "v12 review config"),
        "normalization manifest": _strict_existing_file(paths.normalization_manifest, "normalization manifest"),
        "CleanMesh reduced-analysis executable": _strict_existing_file(paths.cleanmesh_executable, "CleanMesh reduced-analysis executable"),
        "CleanMesh downsample executable": _strict_existing_file(paths.downsample_executable, "CleanMesh downsample executable"),
        "SAND-1mm canonical": _strict_existing_file(paths.data_dir / "Site1-SAND-1mm.ply", "SAND-1mm canonical"),
        "ROCK-1mm canonical": _strict_existing_file(paths.data_dir / "Site1-ROCK-1mm.ply", "ROCK-1mm canonical"),
        "WATER-5mm old01": _strict_existing_file(_old01(paths), "WATER-5mm old01"),
    }
    _require_distinct_existing(existing)
    unexpected_old = data / "Site1-WATER-2mm-old01.ply"
    _require(
        not unexpected_old.exists() and not unexpected_old.is_symlink(),
        "unexpected Site1-WATER-2mm-old01.ply makes rollback naming ambiguous",
    )
    return existing


_GEOMETRY_IMPLEMENTATIONS = (
    "site1_v12_water_pipeline.py",
    "site1_v12_water_refinement.py",
    "site1_v11_confidence.py",
    "rebuild_site1_fossils_v10.py",
)
_FINE_IMPLEMENTATIONS = (
    "site1_v11_water_scalar_enrichment.py",
    "site1_v11_terrain.py",
    "site1_v11_terrain_pipeline.py",
    "site1_v11_hole_pipeline.py",
    "site1_v11_water_density.py",
    "site1_v11_confidence.py",
)
_FINE_INPUT_NAMES = {
    "base_water",
    "geometry_candidate",
    "geometry_manifest",
    "geometry_archive",
    "sand",
    "rock",
    "cleanmesh",
    "normalization_manifest",
}
_INTERFACE_AUDIT_INPUT_NAMES = {
    "base_water",
    "final_water",
    "fine_manifest",
    "geometry_manifest",
    "geometry_archive",
    "sand_1mm",
    "rock_1mm",
    "review_config",
}
_INTERFACE_AUDIT_IMPLEMENTATIONS = (
    "site1_v12_interface_audit.py",
    "site1_v11_water_density.py",
    "site1_v11_water_scalar_enrichment.py",
    "site1_v12_water_pipeline.py",
    "rebuild_site1_fossils_v10.py",
    "rebuild_site1_fossils_v9.py",
    "rebuild_site1_fossils_water.py",
    "site1_v11_confidence.py",
    "site1_v12_water_refinement.py",
    "site1_v11_terrain.py",
)
_INTERFACE_AUDIT_CHECKS = {
    "append_contract",
    "final_additions_are_exact_vacant_safe_reservoir_rows",
    "final_addition_stored_coordinate_geometry_passed",
    "terrain_edge_eligibility_is_candidate_independent",
    "terrain_edge_meaningful_configured_support_continuity_passed",
    "measured_density_lower_and_upper_bounds_passed",
}
_PROVENANCE_SNAPSHOT_NAMES = {
    "fine_manifest": "fine-scalar-manifest.json",
    "geometry_manifest": "geometry-manifest.json",
    "geometry_archive": "geometry-additions.npz",
    "fine_geometry_manifest_copy": "fine-geometry-manifest-copy.json",
    "interface_audit_manifest": "interface-audit-manifest.json",
    "downsample_manifest": "downsample-stage-manifest.json",
    "downsample_report": "downsample-report.json",
    "review_config": "site1-fossils-v12-review.json",
    "normalization_manifest": "normalization-manifest.json",
}
_FAR_LOBE_PROVENANCE_SNAPSHOT_NAMES = {
    "far_lobe_records": "far-lobe-removed-records.bin",
    "far_lobe_source_indices": "far-lobe-source-indices.i64",
}


def _payload_sha256(
    info: PlyInfo,
    *,
    start: int = 0,
    count: int | None = None,
    chunk_records: int = CHUNK_RECORDS,
) -> str:
    _require(0 <= start <= info.count, "PLY payload hash start is outside the cloud")
    length = info.count - start if count is None else int(count)
    _require(0 <= length <= info.count - start, "PLY payload hash length is outside the cloud")
    digest = hashlib.sha256()
    remaining = length * info.stride
    with info.path.open("rb") as handle:
        handle.seek(info.offset + start * info.stride)
        while remaining:
            block = handle.read(min(remaining, max(1, chunk_records) * info.stride))
            _require(bool(block), f"unexpected EOF while hashing PLY payload: {info.path}")
            digest.update(block)
            remaining -= len(block)
    return digest.hexdigest()


def _upstream_file_block(
    block: Mapping,
    expected_path: Path,
    label: str,
) -> dict:
    _require(isinstance(block, Mapping), f"{label} fingerprint is not a mapping")
    required = {"path", "size_bytes", "mtime_ns", "sha256"}
    missing = sorted(required - set(block))
    _require(not missing, f"{label} fingerprint is incomplete: {', '.join(missing)}")
    source = _strict_existing_file(expected_path, label)
    declared = _lexical_absolute(str(block["path"]), f"{label} declared path")
    _require(declared == source, f"{label} path differs from the fixed provenance path")
    value = source.stat()
    _require(int(block["size_bytes"]) == value.st_size, f"{label} byte size drift")
    _require(int(block["mtime_ns"]) == value.st_mtime_ns, f"{label} mtime drift")
    digest = sha256_path(source)
    _require(block["sha256"] == digest, f"{label} hash drift")
    return {
        "path": str(source),
        "bytes": int(value.st_size),
        "sha256": digest,
    }


def _assert_implementation_contract(
    declared: object,
    names: Sequence[str],
    label: str,
) -> dict[str, str]:
    _require(isinstance(declared, Mapping), f"{label} implementation hashes are missing")
    _require(set(declared) == set(names), f"{label} implementation hash set differs")
    result: dict[str, str] = {}
    for name in names:
        source = _strict_existing_file(SCRIPT_DIR / name, f"{label} implementation {name}")
        digest = sha256_path(source)
        _require(declared.get(name) == digest, f"{label} implementation drift: {name}")
        result[name] = digest
    return result


def _assert_stage_fingerprint(
    block: object,
    expected_path: Path,
    label: str,
    *,
    ply: bool,
) -> dict:
    _require(isinstance(block, Mapping), f"{label} fingerprint is missing")
    source = _strict_existing_file(expected_path, label)
    _require(block.get("path") == str(source), f"{label} path differs")
    _fingerprint_keys(block, ply=ply)
    actual = file_fingerprint(source, ply=ply)
    _require(_same_fingerprint(actual, block), f"{label} fingerprint drift")
    return actual


def _verify_far_lobe_provenance(
    value: object,
    *,
    base_info: PlyInfo,
) -> dict:
    _require(isinstance(value, Mapping), "geometry far-lobe decision is missing")
    required = {
        "performed",
        "reversible",
        "measured_no_eligible_component",
        "reason",
        "seed_xy",
        "maximum_seed_distance_m",
        "grid_pitch_m",
        "bridge_radius_m",
        "bridge_iterations",
        "detachment_gap_m",
        "maximum_component_fraction",
        "occupied_cell_count",
        "selected_occupied_cell_count",
        "largest_occupied_cell_count",
        "seed_distance_m",
        "component_fraction",
        "minimum_cell_center_separation_m",
        "minimum_point_separation_lower_bound_m",
        "removed_count",
        "source_count_before",
        "surviving_source_count",
        "surviving_source_payload_byte_exact",
        "surviving_source_row_order_preserved",
        "archive",
    }
    _require(set(value) == required, "geometry far-lobe decision key set differs")
    _require(value.get("reversible") is True, "geometry far-lobe decision is not reversible")
    performed = value.get("performed")
    no_component = value.get("measured_no_eligible_component")
    _require(
        isinstance(performed, bool) and isinstance(no_component, bool),
        "geometry far-lobe decision booleans are invalid",
    )
    _require(performed is (not no_component), "geometry far-lobe decision is ambiguous")
    _require(
        isinstance(value.get("reason"), str) and bool(value["reason"].strip()),
        "geometry far-lobe decision reason is missing",
    )
    removed = _integer(value.get("removed_count"), "far-lobe removed count")
    source_count = _integer(value.get("source_count_before"), "far-lobe source count")
    surviving = _integer(
        value.get("surviving_source_count"), "far-lobe surviving source count"
    )
    _require(
        source_count == base_info.count
        and surviving == source_count - removed
        and value.get("surviving_source_payload_byte_exact") is True
        and value.get("surviving_source_row_order_preserved") is True,
        "geometry far-lobe source-count/payload contract differs",
    )
    seed = value.get("seed_xy")
    _require(
        isinstance(seed, list)
        and len(seed) == 2
        and all(math.isfinite(float(item)) for item in seed),
        "geometry far-lobe seed is invalid",
    )
    for name in (
        "maximum_seed_distance_m",
        "grid_pitch_m",
        "bridge_radius_m",
        "detachment_gap_m",
        "maximum_component_fraction",
    ):
        _require(
            _finite_number(value.get(name), f"far-lobe {name}") > 0.0,
            f"geometry far-lobe {name} is not positive",
        )
    for name in (
        "bridge_iterations",
        "occupied_cell_count",
        "selected_occupied_cell_count",
        "largest_occupied_cell_count",
    ):
        _require(
            _integer(value.get(name), f"far-lobe {name}") >= 0,
            f"geometry far-lobe {name} is negative",
        )
    if not performed:
        _require(
            removed == 0
            and value.get("archive") is None
            and _integer(
                value.get("selected_occupied_cell_count"),
                "far-lobe selected cells",
            )
            == 0
            and value.get("seed_distance_m") is None
            and value.get("component_fraction") is None
            and value.get("minimum_cell_center_separation_m") is None
            and value.get("minimum_point_separation_lower_bound_m") is None,
            "unperformed far-lobe decision lacks a measured no-component proof",
        )
        return {
            "performed": False,
            "reversible": True,
            "measured_no_eligible_component": True,
            "removed_count": 0,
            "source_count_before": source_count,
            "surviving_source_count": surviving,
            "record_archive": None,
            "source_index_archive": None,
            "exact_source_rows_archived": False,
        }

    _require(0 < removed < source_count, "performed far-lobe count is invalid")
    selected_cells = _integer(
        value.get("selected_occupied_cell_count"), "far-lobe selected cells"
    )
    largest_cells = _integer(
        value.get("largest_occupied_cell_count"), "far-lobe largest cells"
    )
    occupied_cells = _integer(
        value.get("occupied_cell_count"), "far-lobe occupied cells"
    )
    fraction = _finite_number(value.get("component_fraction"), "far-lobe fraction")
    maximum_fraction = _finite_number(
        value.get("maximum_component_fraction"), "far-lobe maximum fraction"
    )
    _require(
        0 < selected_cells < occupied_cells
        and 0 < largest_cells <= occupied_cells
        and selected_cells != largest_cells
        and 0.0 < fraction <= maximum_fraction,
        "performed far-lobe component measurement is not eligible",
    )
    _require(
        math.isclose(
            fraction,
            selected_cells / occupied_cells,
            rel_tol=1.0e-9,
            abs_tol=1.0e-12,
        ),
        "performed far-lobe component fraction disagrees with cell counts",
    )
    _require(
        _finite_number(value.get("seed_distance_m"), "far-lobe seed distance")
        <= _finite_number(
            value.get("maximum_seed_distance_m"),
            "far-lobe maximum seed distance",
        ),
        "performed far-lobe component is too far from its seed",
    )
    separation = _finite_number(
        value.get("minimum_point_separation_lower_bound_m"),
        "far-lobe point separation lower bound",
    )
    cell_separation = _finite_number(
        value.get("minimum_cell_center_separation_m"),
        "far-lobe cell-center separation",
    )
    _require(
        cell_separation > 0.0
        and separation > 0.0
        and separation
        >= _finite_number(value.get("detachment_gap_m"), "far-lobe detachment gap"),
        "performed far-lobe component lacks measured detachment",
    )
    archive = value.get("archive")
    _require(isinstance(archive, Mapping), "performed far-lobe archive is missing")
    archive_required = {
        "removed_count",
        "record_stride_bytes",
        "record_archive_format",
        "record_dtype_descr",
        "records",
        "source_indices",
        "source_index_dtype",
        "exact_source_rows_archived",
        "exact_source_indices_archived",
    }
    _require(set(archive) == archive_required, "far-lobe archive key set differs")
    _require(
        _integer(archive.get("removed_count"), "far-lobe archive removed count")
        == removed
        and _integer(archive.get("record_stride_bytes"), "far-lobe archive stride")
        == base_info.stride
        and archive.get("record_archive_format")
        == "raw-fixed-stride-source-dtype"
        and archive.get("record_dtype_descr")
        == [list(item) for item in base_info.dtype.descr]
        and archive.get("source_index_dtype") == "<i8"
        and archive.get("exact_source_rows_archived") is True
        and archive.get("exact_source_indices_archived") is True,
        "far-lobe archive schema/attestation differs",
    )
    records_block = archive.get("records")
    indices_block = archive.get("source_indices")
    _require(isinstance(records_block, Mapping), "far-lobe record fingerprint is missing")
    _require(isinstance(indices_block, Mapping), "far-lobe index fingerprint is missing")
    _require(
        set(records_block) == {"path", "size_bytes", "mtime_ns", "sha256"},
        "far-lobe record fingerprint key set differs",
    )
    _require(
        set(indices_block)
        == {"path", "size_bytes", "mtime_ns", "sha256", "points"}
        and _integer(indices_block.get("points"), "far-lobe index points") == removed,
        "far-lobe index fingerprint key set/count differs",
    )
    record_audit = _upstream_file_block(
        records_block, Path(str(records_block["path"])), "far-lobe record archive"
    )
    index_audit = _upstream_file_block(
        indices_block, Path(str(indices_block["path"])), "far-lobe index archive"
    )
    _require(
        record_audit["bytes"] == removed * base_info.stride
        and index_audit["bytes"] == removed * np.dtype("<i8").itemsize,
        "far-lobe archive byte lengths differ",
    )
    return {
        "performed": True,
        "reversible": True,
        "measured_no_eligible_component": False,
        "removed_count": removed,
        "source_count_before": source_count,
        "surviving_source_count": surviving,
        "minimum_point_separation_lower_bound_m": separation,
        "detachment_gap_m": float(value["detachment_gap_m"]),
        "component_fraction": fraction,
        "maximum_component_fraction": maximum_fraction,
        "record_archive": record_audit,
        "source_index_archive": index_audit,
        "exact_source_rows_archived": True,
        "exact_source_indices_archived": True,
    }


def _verify_geometry_provenance(
    paths: ReleasePaths,
    source_fine: Mapping,
) -> dict:
    manifest_source, document = _load_json(paths.geometry_manifest, "v12 geometry manifest")
    _require(
        document.get("algorithm")
        == "site1-v12-fine-first-supported-water-interface-v1",
        "unexpected v12 geometry algorithm",
    )
    _require(document.get("candidate_only") is True, "geometry stage is not candidate-only")
    _require(
        document.get("canonical_install_performed") is False,
        "geometry stage claims a canonical write",
    )
    _require(
        document.get("existing_payload_byte_exact") is True,
        "geometry stage does not attest a byte-exact base payload",
    )
    parameters = document.get("parameters")
    _require(isinstance(parameters, Mapping), "geometry parameters are missing")
    _require(parameters.get("fine_first") is True, "geometry stage is not fine-first")
    _require(
        parameters.get("coarse_recomputation_allowed") is False,
        "geometry stage permits coarse recomputation",
    )
    base_info = inspect_ply(_canonical(paths, "WATER-2mm"))
    geometry_info = inspect_ply(paths.run_dir / "water-geometry-2mm" / "Site1-WATER-2mm.geometry-v12.candidate.ply")
    _require(base_info.dtype == geometry_info.dtype, "geometry candidate schema differs from its base")
    far_lobe = _verify_far_lobe_provenance(
        document.get("far_lobe_cull"), base_info=base_info
    )
    scalar_contract, scalar_archived = (
        scalar_enrichment.verify_append_only_geometry(
            base_water_path=base_info.path,
            geometry_candidate_path=geometry_info.path,
            geometry_manifest_path=paths.geometry_manifest,
            geometry_archive_path=paths.geometry_archive,
            chunk_records=CHUNK_RECORDS,
        )
    )
    source_block = document.get("source")
    candidate_block = document.get("candidate")
    source_audit = _upstream_file_block(source_block, base_info.path, "geometry source WATER")
    candidate_audit = _upstream_file_block(candidate_block, geometry_info.path, "geometry candidate")
    _require(int(source_block.get("points", -1)) == base_info.count, "geometry source point count drift")
    _require(int(candidate_block.get("points", -1)) == geometry_info.count, "geometry candidate point count drift")
    _require(
        source_audit["sha256"] == source_fine.get("sha256")
        and source_audit["bytes"] == source_fine.get("bytes"),
        "geometry source is not the current canonical fine WATER generation",
    )
    _require(
        document.get("archive") == str(paths.geometry_archive),
        "geometry archive path differs from the fixed stage path",
    )
    archive_fp = file_fingerprint(paths.geometry_archive, ply=False)
    _require(
        document.get("archive_sha256") == archive_fp["sha256"],
        "geometry archive hash drift",
    )
    try:
        with np.load(paths.geometry_archive, allow_pickle=False) as loaded:
            _require("records" in loaded.files, "geometry archive has no records array")
            records = np.asarray(loaded["records"]).copy()
    except (OSError, ValueError) as error:
        raise RuntimeError(f"invalid geometry archive: {paths.geometry_archive}: {error}") from error
    _require(records.ndim == 1, "geometry archive records must be one-dimensional")
    _require(records.dtype == base_info.dtype, "geometry archive schema differs from WATER")
    _require(
        "scalar_ScanID" in (records.dtype.names or ())
        and np.all(np.isfinite(records["scalar_ScanID"]))
        and np.all(records["scalar_ScanID"] == 999.0),
        "geometry archive contains a non-WATER ScanID",
    )
    addition_count = int(scalar_contract.addition_count)
    surviving_source_points = int(scalar_contract.base_points)
    removed_base_count = int(scalar_contract.removed_base_count)
    _require(
        scalar_contract.source_points == base_info.count
        and surviving_source_points == far_lobe["surviving_source_count"]
        and removed_base_count == far_lobe["removed_count"]
        and geometry_info.count == surviving_source_points + addition_count,
        "geometry candidate surviving-base/addition count contract differs",
    )
    _require(len(records) == addition_count, "geometry archive count differs from candidate suffix")
    _require(
        np.asarray(records).tobytes() == np.asarray(scalar_archived).tobytes(),
        "independent geometry archive reload differs",
    )
    _require(int(document.get("addition_count", -1)) == addition_count, "geometry addition count drift")
    _require(
        scalar_contract.candidate_prefix_payload_sha256
        == _payload_sha256(geometry_info, count=surviving_source_points),
        "geometry candidate surviving base payload is not byte-exact",
    )
    archive_records_sha = hashlib.sha256(records.tobytes(order="C")).hexdigest()
    _require(
        _payload_sha256(
            geometry_info,
            start=surviving_source_points,
            count=addition_count,
        )
        == archive_records_sha,
        "geometry candidate suffix differs from the archived records",
    )
    config_audit = _upstream_file_block(
        document.get("config"), paths.review_config, "geometry review config"
    )
    implementations = _assert_implementation_contract(
        document.get("implementation_sha256"),
        _GEOMETRY_IMPLEMENTATIONS,
        "geometry",
    )
    return {
        "manifest": file_fingerprint(manifest_source, ply=False),
        "algorithm": document["algorithm"],
        "source": {
            **file_fingerprint(base_info.path),
            "size_bytes": int(source_block["size_bytes"]),
            "mtime_ns": int(source_block["mtime_ns"]),
        },
        "candidate": {
            **file_fingerprint(geometry_info.path),
            "size_bytes": int(candidate_block["size_bytes"]),
            "mtime_ns": int(candidate_block["mtime_ns"]),
        },
        "archive": archive_fp,
        "archive_records_sha256": archive_records_sha,
        "source_points": int(base_info.count),
        "surviving_source_points": surviving_source_points,
        "removed_base_count": removed_base_count,
        "addition_count": addition_count,
        "far_lobe_cull": far_lobe,
        "config": config_audit,
        "implementation_sha256": implementations,
        "fine_first": True,
        "coarse_recomputation_allowed": False,
        "base_payload_byte_exact": True,
        "surviving_source_payload_byte_exact": True,
    }


def _verify_fine_provenance(
    paths: ReleasePaths,
    source_fine: Mapping,
    candidate_fine: Mapping,
    geometry: Mapping,
) -> dict:
    manifest_source, document = _load_json(paths.fine_manifest, "fine scalar-enrichment manifest")
    _require(
        document.get("operation")
        == "site1-v11-candidate-only-water-addition-scalar-enrichment",
        "fine result is not the fixed scalar-enrichment operation",
    )
    _require(document.get("status") == "built", "fine scalar manifest is not built")
    _require(document.get("candidate_only") is True, "fine scalar stage is not candidate-only")
    _require(
        document.get("canonical_install_performed") is False,
        "fine scalar stage claims a canonical write",
    )
    _require(document.get("resolution_label") == "2mm", "fine scalar resolution is not 2mm")
    candidate = document.get("candidate")
    _require(isinstance(candidate, Mapping), "fine scalar candidate fingerprint is missing")
    _require(candidate.get("path") == str(paths.candidate_2mm), "fine scalar candidate path differs")
    _require(candidate.get("sha256") == candidate_fine.get("sha256"), "fine scalar candidate hash drift")
    _require(int(candidate.get("points", -1)) == candidate_fine.get("points"), "fine scalar candidate point count drift")

    fine_info = inspect_ply(paths.candidate_2mm)
    base_info = inspect_ply(_canonical(paths, "WATER-2mm"))
    _require(fine_info.dtype == base_info.dtype, "fine scalar candidate schema differs from its base")
    surviving_base_points = int(geometry["surviving_source_points"])
    removed_base_count = int(geometry["removed_base_count"])
    _require(
        base_info.count == surviving_base_points + removed_base_count
        and fine_info.count
        == surviving_base_points + int(geometry["addition_count"]),
        "fine scalar candidate surviving-base/addition bounds differ",
    )
    geometry_info = inspect_ply(geometry["candidate"]["path"])
    base_payload_sha = _payload_sha256(
        geometry_info, count=surviving_base_points
    )
    fine_prefix_sha = _payload_sha256(
        fine_info, count=surviving_base_points
    )
    fine_suffix_sha = _payload_sha256(
        fine_info,
        start=surviving_base_points,
        count=fine_info.count - surviving_base_points,
    )
    _require(
        fine_prefix_sha == base_payload_sha,
        "fine scalar candidate changed the surviving WATER payload",
    )
    _require(candidate.get("base_payload_sha256") == fine_prefix_sha, "fine candidate base payload attestation differs")
    _require(candidate.get("suffix_sha256") == fine_suffix_sha, "fine candidate suffix attestation differs")
    try:
        with np.load(paths.geometry_archive, allow_pickle=False) as loaded:
            archived = np.asarray(loaded["records"]).copy()
            _require(
                "candidate_label" in loaded.files,
                "geometry archive lacks candidate_label for scalar verification",
            )
            component_labels = np.asarray(loaded["candidate_label"]).copy()
    except (KeyError, OSError, ValueError) as error:
        raise RuntimeError("unable to reload verified geometry archive") from error
    fine_memory = np.memmap(
        fine_info.path,
        dtype=fine_info.dtype,
        mode="r",
        offset=fine_info.offset,
        shape=(fine_info.count,),
    )
    fine_suffix = np.asarray(fine_memory[surviving_base_points:]).copy()
    del fine_memory
    _require(len(fine_suffix) == len(archived), "fine scalar suffix count differs from geometry archive")
    _require(
        component_labels.ndim == 1
        and len(component_labels) == len(archived)
        and np.issubdtype(component_labels.dtype, np.integer),
        "geometry component labels are not aligned integer labels",
    )
    visibility_fields = {
        "scalar_A_R_Shelter_Lower",
        "scalar_A_R_RainExposure_Lower",
        "scalar_A_R_SVF_Lower",
    }
    geometry_fields = {
        name
        for name in archived.dtype.names or ()
        if name.startswith("scalar_A_R_") and name not in visibility_fields
    }
    stable_fields = [
        name
        for name in archived.dtype.names or ()
        if name not in geometry_fields and name != "scalar_ScanID"
    ]
    for name in stable_fields:
        _require(
            np.asarray(fine_suffix[name]).tobytes()
            == np.asarray(archived[name]).tobytes(),
            f"fine scalar enrichment changed archived stable field: {name}",
        )
    _require(
        np.all(np.isfinite(fine_suffix["scalar_ScanID"]))
        and np.all(fine_suffix["scalar_ScanID"] == 999.0),
        "fine scalar suffix did not restore WATER ScanID 999",
    )

    invariants = document.get("invariants")
    required_invariants = (
        "geometry_candidate_verified_as_base_plus_archive",
        "existing_base_payload_byte_exact",
        "coordinates_and_normals_archive_exact",
        "colour_intensity_composition_archive_exact",
        "geometry_metrics_from_local_cleanmesh",
        "combined_metrics_use_v10_global_normalization",
        "geometry_component_membership_verified",
        "component_field_scalar_coverage_complete",
        "component_field_scalar_ranges_verified",
    )
    _require(isinstance(invariants, Mapping), "fine scalar invariants are missing")
    _require(
        all(invariants.get(name) is True for name in required_invariants),
        "fine scalar invariant set is incomplete",
    )
    _require(
        invariants.get("coarse_geometry_metrics_from_exact_fine_selection") is False,
        "fine scalar stage unexpectedly used coarse scalar transfer",
    )
    _require(invariants.get("canonical_writes") is False, "fine scalar stage claims canonical writes")
    parameters = document.get("parameters")
    semantic = parameters.get("semantic") if isinstance(parameters, Mapping) else None
    _require(isinstance(semantic, Mapping), "fine scalar semantic parameters are missing")
    _require(
        math.isclose(float(semantic.get("nominal_spacing_m", -1.0)), 0.002, rel_tol=0.0, abs_tol=1.0e-12),
        "fine scalar spacing is not 0.002 m",
    )
    _require(
        semantic.get("coarse_geometry_source") == "local-cleanmesh",
        "fine scalar coarse-source contract is not local-cleanmesh",
    )
    _require(
        math.isclose(
            float(semantic.get("minimum_combined_finite_fraction", -1.0)),
            1.0,
            rel_tol=0.0,
            abs_tol=0.0,
        ),
        "fine scalar combined coverage threshold is not fail-closed",
    )
    _require(
        math.isclose(
            float(
                semantic.get("minimum_component_field_finite_fraction", -1.0)
            ),
            1.0,
            rel_tol=0.0,
            abs_tol=0.0,
        ),
        "fine scalar component coverage threshold is not fail-closed",
    )

    scalar_block = document.get("scalar_enrichment")
    _require(isinstance(scalar_block, Mapping), "fine scalar-enrichment audit is missing")
    declared_component_coverage = scalar_block.get(
        "component_field_finite_coverage"
    )
    _require(
        isinstance(declared_component_coverage, Mapping),
        "fine per-component scalar coverage audit is missing",
    )
    _require(
        float(
            scalar_block.get("minimum_component_field_finite_fraction", -1.0)
        )
        == 1.0,
        "fine scalar audit permits incomplete component coverage",
    )
    direct_scalar_audit = dict(
        scalar_enrichment.verify_candidate_component_scalar_coverage(
            paths.candidate_2mm,
            base_points=surviving_base_points,
            component_labels=component_labels,
            context="release fine WATER addition suffix",
        )
    )
    _require(
        declared_component_coverage == direct_scalar_audit["coverage"],
        "fine manifest per-component scalar audit differs from direct candidate audit",
    )
    direct_scalar_audit.update(
        candidate_sha256=candidate_fine["sha256"],
        geometry_archive_sha256=geometry["archive"]["sha256"],
    )

    expected_inputs = {
        "base_water": _canonical(paths, "WATER-2mm"),
        "geometry_candidate": paths.run_dir / "water-geometry-2mm" / "Site1-WATER-2mm.geometry-v12.candidate.ply",
        "geometry_manifest": paths.geometry_manifest,
        "geometry_archive": paths.geometry_archive,
        "sand": paths.data_dir / "Site1-SAND-1mm.ply",
        "rock": paths.data_dir / "Site1-ROCK-1mm.ply",
        "cleanmesh": paths.cleanmesh_executable,
        "normalization_manifest": paths.normalization_manifest,
    }
    blocks = document.get("input_fingerprints")
    _require(isinstance(blocks, Mapping), "fine scalar input fingerprints are missing")
    _require(set(blocks) == _FINE_INPUT_NAMES, "fine scalar input fingerprint set differs")
    input_audits = {
        name: _upstream_file_block(blocks[name], path, f"fine scalar input {name}")
        for name, path in expected_inputs.items()
    }
    _require(
        input_audits["base_water"]["sha256"] == source_fine.get("sha256"),
        "fine scalar base input is not the current canonical WATER source",
    )
    _require(
        input_audits["geometry_manifest"]["sha256"]
        == geometry["manifest"]["sha256"],
        "fine scalar stage is not bound to the verified geometry manifest",
    )
    _require(
        input_audits["geometry_archive"]["sha256"] == geometry["archive"]["sha256"],
        "fine scalar stage is not bound to the verified geometry archive",
    )
    _require(
        input_audits["geometry_candidate"]["sha256"] == geometry["candidate"]["sha256"],
        "fine scalar stage is not bound to the verified geometry candidate",
    )
    geometry_contract = document.get("geometry_contract")
    _require(isinstance(geometry_contract, Mapping), "fine scalar geometry contract is missing")
    contract_expected = {
        "source_points": base_info.count,
        "base_points": surviving_base_points,
        "removed_base_count": removed_base_count,
        "candidate_points": geometry["candidate"]["points"],
        "addition_count": geometry["addition_count"],
        "base_sha256": source_fine["sha256"],
        "candidate_sha256": geometry["candidate"]["sha256"],
        "manifest_sha256": geometry["manifest"]["sha256"],
        "archive_sha256": geometry["archive"]["sha256"],
        "base_payload_sha256": base_payload_sha,
        "candidate_prefix_payload_sha256": base_payload_sha,
        "archive_records_sha256": geometry["archive_records_sha256"],
    }
    for key, expected in contract_expected.items():
        _require(geometry_contract.get(key) == expected, f"fine scalar geometry contract drift: {key}")
    _require(
        geometry_contract.get("candidate_suffix_sha256")
        == geometry["archive_records_sha256"],
        "fine scalar geometry suffix contract drift",
    )
    _require(
        document.get("algorithm") == geometry.get("algorithm"),
        "fine scalar manifest lost the v12 geometry algorithm provenance",
    )
    for key in ("sha256", "points", "size_bytes", "mtime_ns", "path"):
        _require(
            isinstance(document.get("source"), Mapping)
            and document["source"].get(key)
            == (
                source_fine.get("bytes")
                if key == "size_bytes"
                else geometry["source"].get(key)
            ),
            f"fine scalar copied geometry source drift: {key}",
        )
    _require(
        document.get("archive_sha256") == geometry["archive"]["sha256"],
        "fine scalar copied geometry archive hash drift",
    )
    _require(
        isinstance(document.get("config"), Mapping)
        and document["config"].get("path") == geometry["config"]["path"]
        and document["config"].get("sha256") == geometry["config"]["sha256"],
        "fine scalar copied geometry config provenance drift",
    )
    _require(
        document.get("implementation_sha256")
        == geometry.get("implementation_sha256"),
        "fine scalar copied geometry implementation provenance drift",
    )

    geometry_reference = document.get("geometry_manifest")
    _require(isinstance(geometry_reference, Mapping), "fine scalar geometry-manifest reference is missing")
    _require(geometry_reference.get("path") == str(paths.geometry_manifest), "fine scalar geometry-manifest path differs")
    _require(geometry_reference.get("sha256") == geometry["manifest"]["sha256"], "fine scalar geometry-manifest hash differs")
    archived_copy = paths.fine_manifest.parent / "geometry-manifest.json"
    archived_copy_fp = file_fingerprint(archived_copy, ply=False)
    _require(geometry_reference.get("archived_copy") == archived_copy.name, "unexpected fine geometry-manifest copy name")
    _require(geometry_reference.get("archived_copy_sha256") == archived_copy_fp["sha256"], "fine geometry-manifest copy hash drift")
    _require(archived_copy_fp["sha256"] == geometry["manifest"]["sha256"], "fine geometry-manifest copy is not byte-exact")
    implementations = _assert_implementation_contract(
        document.get("scalar_enrichment_implementation"),
        _FINE_IMPLEMENTATIONS,
        "fine scalar enrichment",
    )
    return {
        "manifest": file_fingerprint(manifest_source, ply=False),
        "candidate": dict(candidate_fine),
        "source": dict(source_fine),
        "geometry_manifest_copy": archived_copy_fp,
        "input_fingerprints": input_audits,
        "implementation_sha256": implementations,
        "fine_spacing_m": 0.002,
        "fine_first": True,
        "coarse_scalar_recalculation": False,
        "base_payload_byte_exact": True,
        "component_field_scalar_coverage": direct_scalar_audit,
    }


def _canonical_json_hash_without_key(document: Mapping, excluded: str) -> str:
    payload = json.dumps(
        {key: value for key, value in document.items() if key != excluded},
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _verify_interface_audit_fingerprint(
    block: object,
    expected_path: Path,
    label: str,
    *,
    ply: bool,
) -> dict:
    """Verify the interface-audit module's stricter path/stat fingerprint."""

    _require(isinstance(block, Mapping), f"{label} fingerprint is missing")
    required = {"path", "size_bytes", "mtime_ns", "sha256"}
    if ply:
        required.update(("points", "record_stride", "schema"))
    _require(
        set(block) == required,
        f"{label} fingerprint key set differs",
    )
    source = _strict_existing_file(expected_path, label)
    _require(block.get("path") == str(source), f"{label} path differs")
    before = _stat_identity(source)
    _require(int(block["size_bytes"]) == before[2], f"{label} byte size drift")
    _require(int(block["mtime_ns"]) == before[3], f"{label} mtime drift")
    _require(block.get("sha256") == sha256_path(source), f"{label} hash drift")
    actual = file_fingerprint(source, ply=ply)
    if ply:
        _require(block.get("points") == actual["points"], f"{label} point count drift")
        _require(
            block.get("record_stride") == actual["record_stride"],
            f"{label} record stride drift",
        )
        _require(block.get("schema") == actual["schema"], f"{label} schema drift")
    _require(before == _stat_identity(source), f"{label} changed while verifying")
    return actual


def _verify_interface_audit_provenance(
    paths: ReleasePaths,
    source_fine: Mapping,
    candidate_fine: Mapping,
    geometry: Mapping,
    fine: Mapping,
    *,
    recomputing_verifier: Callable[..., Mapping] | None = None,
) -> dict:
    """Require a passed audit whose gates independently recompute.

    The compact checks below bind the audit into release provenance, but its
    unkeyed JSON self-lock is not an authenticity proof.  The default path
    therefore invokes the audit module's full deterministic verifier first.
    ``recomputing_verifier`` is a keyword-only seam for compact unit fixtures;
    neither the CLI nor the default :func:`build` path supplies an override.
    """

    verifier = (
        interface_audit.verify_interface_audit
        if recomputing_verifier is None
        else recomputing_verifier
    )
    independently_verified = verifier(
        manifest_path=paths.interface_audit_manifest,
        base_water_path=_canonical(paths, "WATER-2mm"),
        final_water_path=paths.candidate_2mm,
        fine_manifest_path=paths.fine_manifest,
        geometry_manifest_path=paths.geometry_manifest,
        geometry_archive_path=paths.geometry_archive,
        sand_1mm_path=paths.data_dir / "Site1-SAND-1mm.ply",
        rock_1mm_path=paths.data_dir / "Site1-ROCK-1mm.ply",
        review_config_path=paths.review_config,
    )
    _require(
        isinstance(independently_verified, Mapping)
        and independently_verified.get("verified") is True
        and independently_verified.get("status") == "passed",
        "independent interface audit gate recomputation did not pass",
    )

    manifest_source, document = _load_json(
        paths.interface_audit_manifest, "v12 interface audit manifest"
    )
    manifest_sha256 = sha256_path(manifest_source)
    _require(
        independently_verified.get("manifest") == str(manifest_source)
        and independently_verified.get("manifest_sha256") == manifest_sha256,
        "independent interface audit verification is not bound to this manifest",
    )
    _require(document.get("schema_version") == 1, "interface audit schema mismatch")
    _require(
        document.get("operation")
        == "site1-v12-post-build-terrain-water-interface-audit",
        "unexpected interface audit operation",
    )
    _require(document.get("status") == "passed", "interface audit did not pass")
    _require(document.get("candidate_only") is True, "interface audit is not candidate-only")
    _require(document.get("canonical_writes") is False, "interface audit claims canonical writes")
    _require(
        document.get("annotations_are_search_neighbourhoods_not_masks") is True,
        "interface audit treats review annotations as fill masks",
    )
    lock = document.get("manifest_lock")
    _require(isinstance(lock, Mapping), "interface audit manifest lock is missing")
    _require(
        set(lock) == {"method", "sha256"}
        and lock.get("method")
        == "sha256-canonical-json-excluding-manifest_lock",
        "unexpected interface audit manifest lock",
    )
    lock_sha = _canonical_json_hash_without_key(document, "manifest_lock")
    _require(lock.get("sha256") == lock_sha, "interface audit manifest lock mismatch")

    terrain = document.get("terrain_resolution")
    _require(isinstance(terrain, Mapping), "interface audit terrain resolution is missing")
    _require(
        terrain.get("selected") == "canonical-1mm-SAND-plus-ROCK"
        and math.isclose(float(terrain.get("spacing_m", -1.0)), 0.001, rel_tol=0.0, abs_tol=1.0e-12)
        and terrain.get("coarse_5mm_used") is False,
        "interface audit did not use canonical 1mm SAND plus ROCK",
    )

    input_paths = {
        "base_water": _canonical(paths, "WATER-2mm"),
        "final_water": paths.candidate_2mm,
        "fine_manifest": paths.fine_manifest,
        "geometry_manifest": paths.geometry_manifest,
        "geometry_archive": paths.geometry_archive,
        "sand_1mm": paths.data_dir / "Site1-SAND-1mm.ply",
        "rock_1mm": paths.data_dir / "Site1-ROCK-1mm.ply",
        "review_config": paths.review_config,
    }
    inputs = document.get("inputs")
    _require(
        isinstance(inputs, Mapping) and set(inputs) == _INTERFACE_AUDIT_INPUT_NAMES,
        "interface audit input set differs",
    )
    input_audits = {
        name: _verify_interface_audit_fingerprint(
            inputs[name], path, f"interface audit input {name}",
            ply=name in {"base_water", "final_water", "sand_1mm", "rock_1mm"},
        )
        for name, path in input_paths.items()
    }
    _require(
        _same_fingerprint(input_audits["base_water"], source_fine),
        "interface audit base WATER differs from the release source",
    )
    _require(
        _same_fingerprint(input_audits["final_water"], candidate_fine),
        "interface audit final WATER differs from the fine candidate",
    )
    _require(
        _same_fingerprint(input_audits["fine_manifest"], fine.get("manifest", {})),
        "interface audit fine manifest differs from verified scalar provenance",
    )
    _require(
        _same_fingerprint(
            input_audits["geometry_manifest"], geometry.get("manifest", {})
        ),
        "interface audit geometry manifest differs from verified geometry provenance",
    )
    _require(
        _same_fingerprint(input_audits["geometry_archive"], geometry.get("archive", {})),
        "interface audit geometry archive differs from verified geometry provenance",
    )
    _require(
        _same_content(
            input_audits["sand_1mm"], fine.get("input_fingerprints", {}).get("sand", {})
        ),
        "interface audit SAND input differs from fine scalar provenance",
    )
    _require(
        _same_content(
            input_audits["rock_1mm"], fine.get("input_fingerprints", {}).get("rock", {})
        ),
        "interface audit ROCK input differs from fine scalar provenance",
    )
    _require(
        _same_fingerprint(input_audits["review_config"], geometry.get("config", {})),
        "interface audit config differs from verified geometry provenance",
    )

    implementation_paths = {
        name: SCRIPT_DIR / name for name in _INTERFACE_AUDIT_IMPLEMENTATIONS
    }
    implementations = document.get("implementations")
    _require(
        isinstance(implementations, Mapping)
        and set(implementations) == set(implementation_paths),
        "interface audit implementation set differs",
    )
    implementation_audits = {
        name: _verify_interface_audit_fingerprint(
            implementations[name], path, f"interface audit implementation {name}",
            ply=False,
        )
        for name, path in implementation_paths.items()
    }

    append = document.get("append_contract")
    _require(isinstance(append, Mapping), "interface audit append contract is missing")
    base_info = inspect_ply(input_paths["base_water"])
    final_info = inspect_ply(input_paths["final_water"])
    surviving_base_points = int(geometry["surviving_source_points"])
    removed_base_points = int(geometry["removed_base_count"])
    expected_append = {
        "source_base_points": base_info.count,
        "removed_base_points": removed_base_points,
        "base_points": surviving_base_points,
        "addition_count": geometry.get("addition_count"),
        "final_points": final_info.count,
        "base_payload_sha256": _payload_sha256(base_info),
        "final_prefix_payload_sha256": _payload_sha256(
            final_info, count=surviving_base_points
        ),
        "base_payload_byte_exact": True,
        "surviving_base_payload_byte_exact": True,
        "surviving_base_row_order_preserved": True,
        "suffix_xyz_archive_exact": True,
    }
    for key, expected in expected_append.items():
        _require(append.get(key) == expected, f"interface audit append contract drift: {key}")
    labels = append.get("component_labels_present")
    _require(
        isinstance(labels, list)
        and labels
        and all(isinstance(value, int) and not isinstance(value, bool) for value in labels),
        "interface audit component-label provenance is missing",
    )

    acceptance = document.get("acceptance")
    _require(isinstance(acceptance, Mapping), "interface audit acceptance is missing")
    checks = acceptance.get("checks")
    _require(
        isinstance(checks, Mapping)
        and set(checks) == _INTERFACE_AUDIT_CHECKS
        and all(value is True for value in checks.values()),
        "interface audit acceptance checks are incomplete",
    )
    _require(acceptance.get("passed") is True, "interface audit acceptance did not pass")
    _require(
        acceptance.get("water_only_center_count_is_acceptance_criterion") is True,
        "interface audit does not directly gate required WATER support",
    )
    metrics = document.get("metrics")
    density_gate = (
        metrics.get("density_continuity_lower_and_upper_gate")
        if isinstance(metrics, Mapping)
        else None
    )
    moving = metrics.get("moving_circle_aggregate") if isinstance(metrics, Mapping) else None
    _require(isinstance(density_gate, Mapping), "interface audit density gate is missing")
    _require(
        math.isclose(float(density_gate.get("circle_radius_m", -1.0)), 0.08, rel_tol=0.0, abs_tol=1.0e-12)
        and math.isclose(float(density_gate.get("step_m", -1.0)), 0.08, rel_tol=0.0, abs_tol=1.0e-12)
        and math.isclose(float(density_gate.get("minimum_ratio", -1.0)), 0.85, rel_tol=0.0, abs_tol=1.0e-12)
        and math.isclose(float(density_gate.get("maximum_ratio", -1.0)), 1.25, rel_tol=0.0, abs_tol=1.0e-12)
        and density_gate.get("post_build_lower_and_upper_bounds_passed") is True
        and density_gate.get("water_only_center_count_is_acceptance_criterion") is True,
        "interface audit measured density gate contract differs",
    )
    _require(
        isinstance(moving, Mapping)
        and moving.get("water_only_center_count_is_acceptance_criterion") is False,
        "interface audit moving-circle terrain/WATER contract differs",
    )
    return {
        "manifest": file_fingerprint(manifest_source, ply=False),
        "operation": document["operation"],
        "status": "passed",
        "candidate_only": True,
        "canonical_writes": False,
        "manifest_lock_sha256": lock_sha,
        "independent_gate_recomputation": {
            "verified": True,
            "status": "passed",
            "manifest_sha256": manifest_sha256,
        },
        "terrain_resolution_m": 0.001,
        "input_fingerprints": input_audits,
        "implementation_fingerprints": implementation_audits,
        "append_contract": dict(append),
        "acceptance_checks": dict(checks),
        "measured_density_gate": {
            "circle_radius_m": 0.08,
            "step_m": 0.08,
            "minimum_ratio": 0.85,
            "maximum_ratio": 1.25,
            "post_build_lower_and_upper_bounds_passed": True,
            "water_only_center_count_is_acceptance_criterion": True,
        },
        "water_only_center_count_is_acceptance_criterion": True,
    }


def _verify_downsample_provenance(
    paths: ReleasePaths,
    candidate_fine: Mapping,
    candidate_coarse: Mapping,
    downsample: Mapping,
    relation: Mapping,
) -> dict:
    manifest_source, document = _load_json(paths.downsample_manifest, "v12 downsample stage manifest")
    _require(
        document.get("operation") == "site1-v12-native-fine-to-coarse-downsample",
        "unexpected downsample stage operation",
    )
    _require(document.get("candidate_only") is True, "downsample stage is not candidate-only")
    _require(document.get("fine_first") is True, "downsample stage is not fine-first")
    _require(
        document.get("coarse_scalar_recalculation_performed") is False,
        "downsample stage declares coarse scalar recalculation",
    )
    _require(
        math.isclose(float(document.get("minimum_spacing_m", -1.0)), 0.005, rel_tol=0.0, abs_tol=1.0e-12),
        "downsample stage spacing is not exactly 0.005 m",
    )
    executable = _assert_stage_fingerprint(
        document.get("executable"), paths.downsample_executable,
        "downsample executable", ply=False,
    )
    fine = _assert_stage_fingerprint(
        document.get("fine_candidate"), paths.candidate_2mm,
        "downsample fine candidate", ply=True,
    )
    coarse = _assert_stage_fingerprint(
        document.get("coarse_candidate"), paths.candidate_5mm,
        "downsample coarse candidate", ply=True,
    )
    report = _assert_stage_fingerprint(
        document.get("native_report"), paths.downsample_report,
        "downsample native report", ply=False,
    )
    _require(_same_fingerprint(fine, candidate_fine), "downsample stage fine candidate differs from scalar output")
    _require(_same_fingerprint(coarse, candidate_coarse), "downsample stage coarse candidate differs from release candidate")
    _require(_same_fingerprint(report, downsample["report"]), "downsample stage report differs from verified native report")
    command = document.get("command")
    _require(isinstance(command, list) and len(command) == 11, "downsample command is not the fixed native invocation")
    expected_prefix = [
        str(paths.downsample_executable), "--input", str(paths.candidate_2mm),
        "--output", str(paths.candidate_5mm), "--spacing", "0.005",
        "--report", str(paths.downsample_report), "--chunk-points",
    ]
    _require(command[:10] == expected_prefix, "downsample command path/spacing contract differs")
    try:
        chunk_points = int(command[10])
    except (TypeError, ValueError) as error:
        raise RuntimeError("downsample chunk-points value is invalid") from error
    _require(chunk_points > 0 and str(chunk_points) == str(command[10]), "downsample chunk-points value is invalid")
    verification = document.get("verification")
    _require(isinstance(verification, Mapping), "downsample verification block is missing")
    native = verification.get("native_report")
    subset = verification.get("exact_ordered_subsequence")
    _require(
        isinstance(native, Mapping)
        and native.get("verified") is True
        and native.get("minimum_spacing_m") == 0.005
        and native.get("source_points") == candidate_fine.get("points")
        and native.get("output_points") == candidate_coarse.get("points"),
        "downsample stage native verification contract differs",
    )
    _require(
        isinstance(subset, Mapping)
        and subset.get("verified") is True
        and subset.get("matched_points") == candidate_coarse.get("points")
        and subset.get("fine_points") == candidate_fine.get("points")
        and subset.get("relation") == "byte-exact ordered full-record subsequence",
        "downsample stage ordered-subsequence contract differs",
    )
    native_report_fp = native.get("report")
    _require(
        isinstance(native_report_fp, Mapping)
        and native_report_fp.get("sha256") == downsample["report"]["sha256"],
        "downsample stage verification report hash differs",
    )
    for key in (
        "relation", "fine_points", "coarse_points", "matched_points",
        "record_stride", "last_matched_fine_index", "header_bytes_may_differ",
    ):
        _require(
            subset.get(key) == relation.get(key),
            f"downsample stage subsequence proof differs: {key}",
        )
    return {
        "manifest": file_fingerprint(manifest_source, ply=False),
        "executable": executable,
        "fine_candidate": fine,
        "coarse_candidate": coarse,
        "native_report": report,
        "chunk_points": chunk_points,
        "minimum_spacing_m": 0.005,
        "fine_first": True,
        "coarse_scalar_recalculation": False,
    }


def _clone_or_copy(source: Path, destination: Path) -> str:
    destination = _strict_absent_path(destination, "snapshot destination")
    temporary = destination.with_name(
        f".{destination.name}.{os.getpid()}.{uuid.uuid4().hex}.partial"
    )
    completed = subprocess.run(
        ["cp", "-c", str(source), str(temporary)],
        check=False,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    method = "apfs-clone"
    if completed.returncode != 0:
        # ``cp -c`` can leave a partial destination on some non-APFS
        # filesystems.  It is our UUID-scoped staging file, but it must not be
        # silently reused by the fallback copy.
        if temporary.exists() or temporary.is_symlink():
            _require(
                temporary.is_file() and not temporary.is_symlink(),
                f"clone left an unsafe staging entry: {temporary}",
            )
            temporary.unlink()
        required = source.stat().st_size + 64 * 1024 * 1024
        available = shutil.disk_usage(destination.parent).free
        _require(
            available >= required,
            "APFS clone failed and free space is insufficient for a safe "
            f"snapshot copy of {source} (need {required}, have {available})",
        )
        with source.open("rb") as source_handle, temporary.open("xb") as output:
            shutil.copyfileobj(source_handle, output, length=32 * 1024 * 1024)
        shutil.copystat(source, temporary)
        method = "copy2"
    with temporary.open("rb") as handle:
        os.fsync(handle.fileno())
    _durable_replace(temporary, destination)
    return method


@contextmanager
def release_lock(run_dir: Path):
    run = _strict_existing_directory(run_dir, "run directory")
    lock_path = run / ".site1-v12-release.lock"
    _require(not lock_path.is_symlink(), f"release lock may not be a symlink: {lock_path}")
    if lock_path.exists():
        lock_stat = lock_path.stat()
        _require(
            stat.S_ISREG(lock_stat.st_mode) and lock_stat.st_nlink == 1,
            f"release lock is not a private regular file: {lock_path}",
        )
    with lock_path.open("a+", encoding="utf-8") as handle:
        try:
            fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise RuntimeError("another Scene1 v12 release action holds the lock") from error
        handle.seek(0)
        handle.truncate()
        handle.write(f"pid={os.getpid()} created={_now()}\n")
        handle.flush()
        os.fsync(handle.fileno())
        try:
            yield
        finally:
            fcntl.flock(handle.fileno(), fcntl.LOCK_UN)


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
    raise RuntimeError("unable to determine whether invisible_places is running: " + detail)


def refuse_running_app() -> None:
    if app_running():
        raise RuntimeError("refusing WATER publication: invisible_places is running")


def _build_locked(
    paths: ReleasePaths,
    *,
    interface_audit_verifier: Callable[..., Mapping] | None = None,
) -> dict:
    files = _validate_build_paths(paths)
    fine_info = inspect_ply(files["WATER-2mm candidate"])
    coarse_info = inspect_ply(files["WATER-5mm candidate"])
    _require(
        fine_info.schema == inspect_ply(files["WATER-2mm canonical"]).schema,
        "fine candidate schema differs from current fine canonical",
    )
    _require(
        coarse_info.schema == inspect_ply(files["WATER-5mm canonical"]).schema,
        "coarse candidate schema differs from current coarse canonical",
    )
    # Fingerprint before performing the streaming proofs, then assert the same
    # generation again after every upstream contract has been inspected.  This
    # closes the candidate/report TOCTOU window during release construction.
    source_fingerprints = {
        "WATER-2mm": file_fingerprint(files["WATER-2mm canonical"]),
        "WATER-5mm": file_fingerprint(files["WATER-5mm canonical"]),
    }
    candidate_fingerprints = {
        "WATER-2mm": file_fingerprint(files["WATER-2mm candidate"]),
        "WATER-5mm": file_fingerprint(files["WATER-5mm candidate"]),
    }
    report_fingerprint = file_fingerprint(files["CleanMesh report"], ply=False)
    geometry = _verify_geometry_provenance(
        paths, source_fingerprints["WATER-2mm"]
    )
    fine_provenance = _verify_fine_provenance(
        paths,
        source_fingerprints["WATER-2mm"],
        candidate_fingerprints["WATER-2mm"],
        geometry,
    )
    interface_audit = _verify_interface_audit_provenance(
        paths,
        source_fingerprints["WATER-2mm"],
        candidate_fingerprints["WATER-2mm"],
        geometry,
        fine_provenance,
        recomputing_verifier=interface_audit_verifier,
    )
    relation = verify_ordered_record_subsequence(
        files["WATER-2mm candidate"], files["WATER-5mm candidate"]
    )
    try:
        with np.load(paths.geometry_archive, allow_pickle=False) as loaded:
            _require(
                "candidate_label" in loaded.files,
                "geometry archive lacks candidate_label for coarse scalar verification",
            )
            component_labels = np.asarray(loaded["candidate_label"]).copy()
    except (OSError, ValueError) as error:
        raise RuntimeError(
            "unable to load geometry labels for coarse scalar verification"
        ) from error
    coarse_scalar_coverage = dict(
        scalar_enrichment.verify_coarse_exact_subset_component_scalar_coverage(
            files["WATER-2mm candidate"],
            files["WATER-5mm candidate"],
            fine_base_points=int(
                fine_provenance["component_field_scalar_coverage"]["base_points"]
            ),
            fine_component_labels=component_labels,
            chunk_records=CHUNK_RECORDS,
        )
    )
    coarse_scalar_coverage.update(
        fine_candidate_sha256=candidate_fingerprints["WATER-2mm"]["sha256"],
        coarse_candidate_sha256=candidate_fingerprints["WATER-5mm"]["sha256"],
        geometry_archive_sha256=geometry["archive"]["sha256"],
    )
    downsample = verify_cleanmesh_downsample_report(
        files["CleanMesh report"],
        files["WATER-2mm candidate"],
        files["WATER-5mm candidate"],
    )
    downsample_provenance = _verify_downsample_provenance(
        paths,
        candidate_fingerprints["WATER-2mm"],
        candidate_fingerprints["WATER-5mm"],
        downsample,
        relation,
    )
    _assert_fingerprint(
        files["WATER-2mm canonical"], source_fingerprints["WATER-2mm"],
        "WATER-2mm canonical after provenance verification",
    )
    _assert_fingerprint(
        files["WATER-5mm canonical"], source_fingerprints["WATER-5mm"],
        "WATER-5mm canonical after provenance verification",
    )
    _assert_fingerprint(
        files["WATER-2mm candidate"], candidate_fingerprints["WATER-2mm"],
        "WATER-2mm candidate after provenance verification",
    )
    _assert_fingerprint(
        files["WATER-5mm candidate"], candidate_fingerprints["WATER-5mm"],
        "WATER-5mm candidate after provenance verification",
    )
    _assert_fingerprint(
        files["CleanMesh report"], report_fingerprint,
        "CleanMesh report after provenance verification",
    )
    _assert_fingerprint(
        files["interface audit manifest"], interface_audit["manifest"],
        "interface audit manifest after provenance verification",
    )
    for name in ("sand_1mm", "rock_1mm"):
        _assert_fingerprint(
            interface_audit["input_fingerprints"][name]["path"],
            interface_audit["input_fingerprints"][name],
            f"interface audit {name} after provenance verification",
        )
    old01 = file_fingerprint(files["WATER-5mm old01"])

    stage = Path(
        tempfile.mkdtemp(
            prefix=f".{paths.release_dir.name}.staging-",
            dir=paths.release_dir.parent,
        )
    )
    snapshots: dict[str, dict] = {}
    provenance_snapshots: dict[str, dict] = {}
    try:
        for label, canonical_name in CANONICAL_BY_LABEL.items():
            destination_stage = stage / "source-snapshots" / canonical_name
            destination_stage.parent.mkdir(parents=True, exist_ok=True)
            method = _clone_or_copy(
                files[f"{label} canonical"], destination_stage
            )
            snapshot = _assert_fingerprint(
                destination_stage, source_fingerprints[label], f"{label} snapshot"
            )
            snapshot["path"] = str(
                paths.release_dir / "source-snapshots" / canonical_name
            )
            snapshot["method"] = method
            snapshots[label] = snapshot

        provenance_sources = {
            "fine_manifest": fine_provenance["manifest"],
            "geometry_manifest": geometry["manifest"],
            "geometry_archive": geometry["archive"],
            "fine_geometry_manifest_copy": fine_provenance["geometry_manifest_copy"],
            "interface_audit_manifest": interface_audit["manifest"],
            "downsample_manifest": downsample_provenance["manifest"],
            "downsample_report": downsample_provenance["native_report"],
            "review_config": geometry["config"],
            "normalization_manifest": fine_provenance["input_fingerprints"]["normalization_manifest"],
        }
        provenance_paths = {
            "fine_manifest": paths.fine_manifest,
            "geometry_manifest": paths.geometry_manifest,
            "geometry_archive": paths.geometry_archive,
            "fine_geometry_manifest_copy": paths.fine_manifest.parent / "geometry-manifest.json",
            "interface_audit_manifest": paths.interface_audit_manifest,
            "downsample_manifest": paths.downsample_manifest,
            "downsample_report": paths.downsample_report,
            "review_config": paths.review_config,
            "normalization_manifest": paths.normalization_manifest,
        }
        provenance_snapshot_names = dict(_PROVENANCE_SNAPSHOT_NAMES)
        far_lobe_release = geometry["far_lobe_cull"]
        if far_lobe_release["performed"]:
            provenance_snapshot_names.update(_FAR_LOBE_PROVENANCE_SNAPSHOT_NAMES)
            provenance_sources.update(
                far_lobe_records=far_lobe_release["record_archive"],
                far_lobe_source_indices=far_lobe_release["source_index_archive"],
            )
            provenance_paths.update(
                far_lobe_records=Path(far_lobe_release["record_archive"]["path"]),
                far_lobe_source_indices=Path(
                    far_lobe_release["source_index_archive"]["path"]
                ),
            )
        for name, filename in provenance_snapshot_names.items():
            destination_stage = stage / "provenance" / filename
            destination_stage.parent.mkdir(parents=True, exist_ok=True)
            method = _clone_or_copy(provenance_paths[name], destination_stage)
            snapshot = _assert_fingerprint(
                destination_stage,
                provenance_sources[name],
                f"{name} provenance snapshot",
            )
            snapshot["path"] = str(paths.release_dir / "provenance" / filename)
            snapshot["method"] = method
            provenance_snapshots[name] = {
                "source": provenance_sources[name],
                "snapshot": snapshot,
            }

        clouds = {}
        for label, canonical_name in CANONICAL_BY_LABEL.items():
            candidate_path = (
                paths.candidate_2mm if label == "WATER-2mm" else paths.candidate_5mm
            )
            clouds[label] = {
                "canonical": canonical_name,
                "source": source_fingerprints[label],
                "candidate": candidate_fingerprints[label],
                "snapshot": snapshots[label],
                "candidate_path": str(candidate_path),
            }
        manifest = {
            "schema_version": SCHEMA_VERSION,
            "operation": OPERATION,
            "status": "built",
            "created": _now(),
            "data_dir": str(paths.data_dir),
            "run_dir": str(paths.run_dir),
            "release_dir": str(paths.release_dir),
            "clouds": clouds,
            "cross_scale_verification": relation,
            "downsample_verification": downsample,
            "upstream_provenance": {
                "verified": True,
                "fine_first": True,
                "coarse_scalar_recalculation": False,
                "source_fine": source_fingerprints["WATER-2mm"],
                "candidate_fine": candidate_fingerprints["WATER-2mm"],
                "candidate_coarse": candidate_fingerprints["WATER-5mm"],
                "geometry": geometry,
                "fine_scalar_enrichment": fine_provenance,
                "coarse_component_field_scalar_coverage": coarse_scalar_coverage,
                "interface_audit": interface_audit,
                "downsample_stage": downsample_provenance,
                "artifacts": provenance_snapshots,
            },
            "protected_existing_water_old01": old01,
            "transactions": [],
            "invariants": {
                "exactly_two_water_canonicals": True,
                "coarse_is_ordered_full_record_subsequence": True,
                "cleanmesh_native_report_hash_locked": True,
                "upstream_v12_provenance_hash_locked": True,
                "fine_candidate_bound_to_scalar_manifest": True,
                "fine_and_coarse_component_scalar_coverage_directly_verified": True,
                "geometry_base_archive_config_implementation_bound": True,
                "passed_interface_audit_hash_locked": True,
                "passed_interface_audit_independently_recomputed": True,
                "downsample_stage_binds_fine_coarse_report_executable": True,
                "coarse_scalar_recalculation_forbidden": True,
                "compact_provenance_snapshots_hash_locked": True,
                "source_snapshots_hash_locked": True,
                "restore_uses_source_snapshots": True,
                "preexisting_water_old01_untouched": True,
            },
        }
        _assert_fingerprint(
            files["WATER-5mm old01"], old01, "protected WATER-5mm old01"
        )
        _atomic_json(stage / "manifest.json", manifest)
        _strict_absent_path(paths.release_dir, "release directory")
        _durable_replace(stage, paths.release_dir)
    except BaseException:
        if stage.exists() and stage.parent == paths.release_dir.parent:
            shutil.rmtree(stage)
        raise
    return {
        "built": True,
        "release_dir": str(paths.release_dir),
        "manifest": str(paths.release_dir / "manifest.json"),
        "cross_scale_verification": relation,
        "downsample_verification": downsample,
    }


def build(
    args,
    *,
    interface_audit_verifier: Callable[..., Mapping] | None = None,
) -> dict:
    paths = _coerce_paths(args)
    with release_lock(paths.run_dir):
        return _build_locked(
            paths,
            interface_audit_verifier=interface_audit_verifier,
        )


def _manifest_path(paths: ReleasePaths) -> Path:
    return paths.release_dir / "manifest.json"


def _required_geometry_fields_from_fingerprint(candidate: Mapping) -> list[str]:
    schema = candidate.get("schema")
    _require(isinstance(schema, list), "candidate scalar schema is missing")
    visibility = {
        "scalar_A_R_Shelter_Lower",
        "scalar_A_R_RainExposure_Lower",
        "scalar_A_R_SVF_Lower",
    }
    fields: list[str] = []
    for item in schema:
        _require(
            isinstance(item, list)
            and len(item) == 2
            and isinstance(item[0], str),
            "candidate scalar schema row is invalid",
        )
        name = item[0]
        if name.startswith("scalar_A_R_") and name not in visibility:
            fields.append(name)
    _require(fields, "candidate has no required A_R geometry fields")
    return fields


def _validate_component_field_coverage(
    value: object,
    *,
    required_fields: Sequence[str],
    label: str,
) -> None:
    _require(isinstance(value, Mapping), f"{label} coverage is missing")
    _require(
        value.get("method")
        == "exact-per-component-per-geometry-field-finiteness-and-range-v1",
        f"{label} coverage method differs",
    )
    _require(
        _finite_number(
            value.get("minimum_required_finite_fraction"),
            f"{label} minimum finite fraction",
        )
        == 1.0,
        f"{label} coverage threshold is not fail-closed",
    )
    _require(
        value.get("required_fields") == list(required_fields),
        f"{label} required scalar fields differ",
    )
    components = value.get("components")
    _require(isinstance(components, list) and components, f"{label} components are missing")
    _require(
        _integer(value.get("component_count"), f"{label} component count")
        == len(components),
        f"{label} component count differs",
    )
    _require(
        value.get("all_components_all_required_fields_accepted") is True,
        f"{label} aggregate acceptance is false",
    )
    seen: set[int] = set()
    for component in components:
        _require(isinstance(component, Mapping), f"{label} component row is invalid")
        _require(
            set(component)
            == {
                "component_label",
                "points",
                "all_required_fields_accepted",
                "fields",
            },
            f"{label} component row key set differs",
        )
        component_label = _integer(
            component.get("component_label"), f"{label} component label"
        )
        _require(component_label not in seen, f"{label} component label is duplicated")
        seen.add(component_label)
        points = _integer(component.get("points"), f"{label} component points")
        _require(points > 0, f"{label} component is empty")
        _require(
            component.get("all_required_fields_accepted") is True,
            f"{label} component acceptance is false",
        )
        fields = component.get("fields")
        _require(
            isinstance(fields, Mapping) and set(fields) == set(required_fields),
            f"{label} component field set differs",
        )
        for name in required_fields:
            row = fields[name]
            _require(isinstance(row, Mapping), f"{label} field row is invalid: {name}")
            _require(
                set(row)
                == {
                    "finite",
                    "total",
                    "fraction",
                    "minimum",
                    "maximum",
                    "range_contract",
                    "range_lower",
                    "range_upper",
                    "range_passed",
                    "accepted",
                },
                f"{label} field row key set differs: {name}",
            )
            finite = _integer(row.get("finite"), f"{label} {name} finite count")
            total = _integer(row.get("total"), f"{label} {name} total count")
            fraction = _finite_number(row.get("fraction"), f"{label} {name} fraction")
            _require(
                finite == total == points and fraction == 1.0,
                f"{label} {name} is not 100% finite",
            )
            minimum = _finite_number(row.get("minimum"), f"{label} {name} minimum")
            maximum = _finite_number(row.get("maximum"), f"{label} {name} maximum")
            _require(minimum <= maximum, f"{label} {name} range is inverted")
            expected_lower = expected_upper = None
            expected_contract = "finite-physical-values-no-global-clamp"
            if name.endswith("_Combined"):
                expected_lower = 0.0 if "_Roughness_" in name else -1.0
                expected_upper = 1.0
                expected_contract = "global-normalized-combined-range"
            elif name == "scalar_A_R_RoughnessRelative_FineMedium":
                expected_lower, expected_upper = 0.0, 8.0
                expected_contract = "derived-relative-roughness-range"
            _require(
                row.get("range_contract") == expected_contract
                and row.get("range_lower") == expected_lower
                and row.get("range_upper") == expected_upper,
                f"{label} {name} range contract differs",
            )
            if expected_lower is not None:
                _require(
                    minimum >= expected_lower - 1.0e-6
                    and maximum <= expected_upper + 1.0e-6,
                    f"{label} {name} lies outside its physical range",
                )
            _require(
                row.get("range_passed") is True and row.get("accepted") is True,
                f"{label} {name} was not accepted",
            )


def _validate_direct_scalar_audits(
    fine: Mapping,
    coarse: object,
    *,
    candidate_fine: Mapping,
    candidate_coarse: Mapping,
    geometry_archive: Mapping,
) -> None:
    required_fields = _required_geometry_fields_from_fingerprint(candidate_fine)
    fine_audit = fine.get("component_field_scalar_coverage")
    _require(isinstance(fine_audit, Mapping), "direct fine scalar audit is missing")
    _require(
        fine_audit.get("method")
        == "direct-ply-addition-suffix-per-component-scalar-audit-v1"
        and fine_audit.get("candidate_sha256") == candidate_fine.get("sha256")
        and fine_audit.get("geometry_archive_sha256")
        == geometry_archive.get("sha256")
        and fine_audit.get("candidate_points") == candidate_fine.get("points")
        and fine_audit.get("geometry_fields") == required_fields
        and fine_audit.get("all_required_fields_accepted") is True,
        "direct fine scalar audit contract differs",
    )
    fine_base_points = _integer(
        fine_audit.get("base_points"), "direct fine scalar base points"
    )
    fine_additions = _integer(
        fine_audit.get("addition_count"), "direct fine scalar addition count"
    )
    _require(
        fine_base_points >= 0
        and fine_additions > 0
        and fine_base_points + fine_additions == candidate_fine.get("points"),
        "direct fine scalar suffix bounds differ",
    )
    label_sha = fine_audit.get("component_label_sha256")
    _require(
        isinstance(label_sha, str) and len(label_sha) == 64,
        "direct fine scalar component-label hash is invalid",
    )
    _validate_component_field_coverage(
        fine_audit.get("coverage"),
        required_fields=required_fields,
        label="direct fine scalar",
    )

    _require(isinstance(coarse, Mapping), "direct coarse scalar audit is missing")
    _require(
        coarse.get("method")
        == "exact-full-record-fine-suffix-membership-scalar-audit-v1"
        and coarse.get("fine_candidate_sha256") == candidate_fine.get("sha256")
        and coarse.get("coarse_candidate_sha256") == candidate_coarse.get("sha256")
        and coarse.get("geometry_archive_sha256") == geometry_archive.get("sha256")
        and coarse.get("fine_candidate_points") == candidate_fine.get("points")
        and coarse.get("coarse_candidate_points") == candidate_coarse.get("points")
        and coarse.get("fine_base_points") == fine_base_points
        and coarse.get("fine_addition_count") == fine_additions
        and coarse.get("component_label_sha256") == label_sha
        and coarse.get("geometry_fields") == required_fields
        and coarse.get("full_record_membership_exact") is True
        and coarse.get("every_fine_component_represented") is True
        and coarse.get("all_required_fields_accepted") is True,
        "direct coarse scalar audit contract differs",
    )
    matched = _integer(
        coarse.get("matched_coarse_addition_count"),
        "direct coarse scalar matched addition count",
    )
    _require(
        0 < matched <= candidate_coarse.get("points"),
        "direct coarse scalar matched count differs",
    )
    _require(
        coarse.get("fine_component_labels") == coarse.get("coarse_component_labels")
        and isinstance(coarse.get("fine_component_labels"), list)
        and coarse.get("fine_component_labels"),
        "direct coarse scalar component representation differs",
    )
    _validate_component_field_coverage(
        coarse.get("coverage"),
        required_fields=required_fields,
        label="direct coarse scalar",
    )


def _validate_release_provenance(paths: ReleasePaths, manifest: Mapping) -> None:
    provenance = manifest.get("upstream_provenance")
    _require(isinstance(provenance, Mapping), "upstream v12 provenance is missing")
    _require(provenance.get("verified") is True, "upstream v12 provenance is not verified")
    _require(provenance.get("fine_first") is True, "release provenance is not fine-first")
    _require(
        provenance.get("coarse_scalar_recalculation") is False,
        "release provenance permits coarse scalar recalculation",
    )
    clouds = manifest["clouds"]
    source_fine = provenance.get("source_fine")
    candidate_fine = provenance.get("candidate_fine")
    candidate_coarse = provenance.get("candidate_coarse")
    for block, label in (
        (source_fine, "provenance source fine"),
        (candidate_fine, "provenance candidate fine"),
        (candidate_coarse, "provenance candidate coarse"),
    ):
        _require(isinstance(block, Mapping), f"{label} fingerprint is missing")
        _fingerprint_keys(block, ply=True)
    _require(
        _same_fingerprint(source_fine, clouds["WATER-2mm"]["source"]),
        "provenance source fine differs from release source",
    )
    _require(
        _same_fingerprint(candidate_fine, clouds["WATER-2mm"]["candidate"]),
        "provenance fine candidate differs from release candidate",
    )
    _require(
        _same_fingerprint(candidate_coarse, clouds["WATER-5mm"]["candidate"]),
        "provenance coarse candidate differs from release candidate",
    )

    geometry = provenance.get("geometry")
    fine = provenance.get("fine_scalar_enrichment")
    coarse_scalar = provenance.get("coarse_component_field_scalar_coverage")
    interface_audit = provenance.get("interface_audit")
    downsample = provenance.get("downsample_stage")
    _require(isinstance(geometry, Mapping), "verified geometry provenance is missing")
    _require(isinstance(fine, Mapping), "verified fine scalar provenance is missing")
    _require(
        isinstance(coarse_scalar, Mapping),
        "verified coarse scalar provenance is missing",
    )
    _require(
        isinstance(interface_audit, Mapping),
        "verified interface-audit provenance is missing",
    )
    _require(isinstance(downsample, Mapping), "verified downsample provenance is missing")
    _require(
        geometry.get("algorithm")
        == "site1-v12-fine-first-supported-water-interface-v1"
        and geometry.get("fine_first") is True
        and geometry.get("coarse_recomputation_allowed") is False
        and geometry.get("base_payload_byte_exact") is True,
        "geometry provenance contract is incomplete",
    )
    _require(
        _same_fingerprint(geometry.get("source", {}), source_fine),
        "geometry provenance source differs from canonical fine source",
    )
    geometry_candidate = geometry.get("candidate")
    geometry_manifest = geometry.get("manifest")
    geometry_archive = geometry.get("archive")
    geometry_config = geometry.get("config")
    _require(isinstance(geometry_candidate, Mapping), "geometry candidate provenance is missing")
    _require(isinstance(geometry_manifest, Mapping), "geometry manifest provenance is missing")
    _require(isinstance(geometry_archive, Mapping), "geometry archive provenance is missing")
    _require(isinstance(geometry_config, Mapping), "geometry config provenance is missing")
    _fingerprint_keys(geometry_candidate, ply=True)
    for block in (geometry_manifest, geometry_archive, geometry_config):
        _fingerprint_keys(block, ply=False)
    _require(
        geometry_candidate.get("path")
        == str(paths.run_dir / "water-geometry-2mm" / "Site1-WATER-2mm.geometry-v12.candidate.ply"),
        "geometry candidate provenance path differs",
    )
    _require(geometry_manifest.get("path") == str(paths.geometry_manifest), "geometry manifest provenance path differs")
    _require(geometry_archive.get("path") == str(paths.geometry_archive), "geometry archive provenance path differs")
    _require(geometry_config.get("path") == str(paths.review_config), "geometry config provenance path differs")
    _require(
        isinstance(geometry.get("archive_records_sha256"), str)
        and len(geometry["archive_records_sha256"]) == 64
        and int(geometry.get("addition_count", -1))
        == int(geometry_candidate.get("points", -2))
        - int(geometry.get("surviving_source_points", -3))
        and int(geometry.get("source_points", -1))
        == int(source_fine.get("points", -2))
        and int(geometry.get("source_points", -1))
        == int(geometry.get("surviving_source_points", -2))
        + int(geometry.get("removed_base_count", -3)),
        "geometry archive/count provenance contract differs",
    )
    far_lobe = geometry.get("far_lobe_cull")
    _require(isinstance(far_lobe, Mapping), "geometry far-lobe provenance is missing")
    _require(
        far_lobe.get("reversible") is True
        and far_lobe.get("source_count_before") == geometry.get("source_points")
        and far_lobe.get("surviving_source_count")
        == geometry.get("surviving_source_points")
        and far_lobe.get("removed_count") == geometry.get("removed_base_count"),
        "geometry compact far-lobe provenance differs",
    )
    if far_lobe.get("performed") is True:
        _require(
            far_lobe.get("measured_no_eligible_component") is False
            and far_lobe.get("exact_source_rows_archived") is True
            and far_lobe.get("exact_source_indices_archived") is True,
            "performed compact far-lobe provenance is incomplete",
        )
        for key in ("record_archive", "source_index_archive"):
            block = far_lobe.get(key)
            _require(isinstance(block, Mapping), f"compact far-lobe {key} is missing")
            _fingerprint_keys(block, ply=False)
    else:
        _require(
            far_lobe.get("performed") is False
            and far_lobe.get("measured_no_eligible_component") is True
            and far_lobe.get("removed_count") == 0
            and far_lobe.get("record_archive") is None
            and far_lobe.get("source_index_archive") is None,
            "unperformed compact far-lobe provenance is incomplete",
        )
    _require(
        fine.get("fine_first") is True
        and fine.get("coarse_scalar_recalculation") is False
        and fine.get("base_payload_byte_exact") is True
        and fine.get("fine_spacing_m") == 0.002,
        "fine scalar provenance contract is incomplete",
    )
    _require(
        _same_fingerprint(fine.get("candidate", {}), candidate_fine),
        "fine scalar manifest is not bound to the release fine candidate",
    )
    _require(
        _same_fingerprint(fine.get("source", {}), source_fine),
        "fine scalar manifest is not bound to the release source",
    )
    fine_manifest = fine.get("manifest")
    fine_copy = fine.get("geometry_manifest_copy")
    fine_inputs = fine.get("input_fingerprints")
    _require(isinstance(fine_manifest, Mapping), "fine scalar manifest fingerprint is missing")
    _require(isinstance(fine_copy, Mapping), "fine geometry-manifest copy fingerprint is missing")
    _require(
        isinstance(fine_inputs, Mapping) and set(fine_inputs) == _FINE_INPUT_NAMES,
        "fine scalar input provenance set differs",
    )
    _fingerprint_keys(fine_manifest, ply=False)
    _fingerprint_keys(fine_copy, ply=False)
    _require(fine_manifest.get("path") == str(paths.fine_manifest), "fine scalar manifest provenance path differs")
    _require(
        fine_copy.get("path") == str(paths.fine_manifest.parent / "geometry-manifest.json"),
        "fine geometry-manifest copy provenance path differs",
    )
    _require(
        _same_content(fine_copy, geometry_manifest),
        "fine geometry-manifest copy differs from verified geometry manifest",
    )
    expected_fine_input_paths = {
        "base_water": _canonical(paths, "WATER-2mm"),
        "geometry_candidate": paths.run_dir / "water-geometry-2mm" / "Site1-WATER-2mm.geometry-v12.candidate.ply",
        "geometry_manifest": paths.geometry_manifest,
        "geometry_archive": paths.geometry_archive,
        "sand": paths.data_dir / "Site1-SAND-1mm.ply",
        "rock": paths.data_dir / "Site1-ROCK-1mm.ply",
        "cleanmesh": paths.cleanmesh_executable,
        "normalization_manifest": paths.normalization_manifest,
    }
    for name, expected_path in expected_fine_input_paths.items():
        block = fine_inputs[name]
        _require(isinstance(block, Mapping), f"fine scalar input provenance is invalid: {name}")
        _fingerprint_keys(block, ply=False)
        _require(block.get("path") == str(expected_path), f"fine scalar input provenance path differs: {name}")
    _require(_same_content(fine_inputs["base_water"], source_fine), "fine scalar base provenance differs")
    _require(_same_content(fine_inputs["geometry_candidate"], geometry_candidate), "fine scalar geometry candidate provenance differs")
    _require(_same_content(fine_inputs["geometry_manifest"], geometry_manifest), "fine scalar geometry manifest provenance differs")
    _require(_same_content(fine_inputs["geometry_archive"], geometry_archive), "fine scalar geometry archive provenance differs")
    _validate_direct_scalar_audits(
        fine,
        coarse_scalar,
        candidate_fine=candidate_fine,
        candidate_coarse=candidate_coarse,
        geometry_archive=geometry_archive,
    )

    audit_manifest = interface_audit.get("manifest")
    audit_inputs = interface_audit.get("input_fingerprints")
    audit_implementations = interface_audit.get("implementation_fingerprints")
    audit_append = interface_audit.get("append_contract")
    audit_checks = interface_audit.get("acceptance_checks")
    audit_density = interface_audit.get("measured_density_gate")
    audit_recomputation = interface_audit.get("independent_gate_recomputation")
    _require(
        interface_audit.get("operation")
        == "site1-v12-post-build-terrain-water-interface-audit"
        and interface_audit.get("status") == "passed"
        and interface_audit.get("candidate_only") is True
        and interface_audit.get("canonical_writes") is False
        and interface_audit.get("terrain_resolution_m") == 0.001
        and interface_audit.get("water_only_center_count_is_acceptance_criterion") is True,
        "interface-audit provenance contract is incomplete",
    )
    _require(
        isinstance(interface_audit.get("manifest_lock_sha256"), str)
        and len(interface_audit["manifest_lock_sha256"]) == 64,
        "interface-audit self-lock provenance is invalid",
    )
    _require(isinstance(audit_manifest, Mapping), "interface-audit manifest fingerprint is missing")
    _fingerprint_keys(audit_manifest, ply=False)
    _require(
        isinstance(audit_recomputation, Mapping)
        and set(audit_recomputation)
        == {"verified", "status", "manifest_sha256"}
        and audit_recomputation.get("verified") is True
        and audit_recomputation.get("status") == "passed"
        and audit_recomputation.get("manifest_sha256")
        == audit_manifest.get("sha256"),
        "interface-audit independent recomputation provenance is invalid",
    )
    _require(
        audit_manifest.get("path") == str(paths.interface_audit_manifest),
        "interface-audit manifest provenance path differs",
    )
    _require(
        isinstance(audit_inputs, Mapping)
        and set(audit_inputs) == _INTERFACE_AUDIT_INPUT_NAMES,
        "interface-audit input provenance set differs",
    )
    expected_audit_paths = {
        "base_water": _canonical(paths, "WATER-2mm"),
        "final_water": paths.candidate_2mm,
        "fine_manifest": paths.fine_manifest,
        "geometry_manifest": paths.geometry_manifest,
        "geometry_archive": paths.geometry_archive,
        "sand_1mm": paths.data_dir / "Site1-SAND-1mm.ply",
        "rock_1mm": paths.data_dir / "Site1-ROCK-1mm.ply",
        "review_config": paths.review_config,
    }
    for name, expected_path in expected_audit_paths.items():
        block = audit_inputs[name]
        _require(isinstance(block, Mapping), f"interface-audit input provenance is invalid: {name}")
        _fingerprint_keys(
            block,
            ply=name in {"base_water", "final_water", "sand_1mm", "rock_1mm"},
        )
        _require(block.get("path") == str(expected_path), f"interface-audit input path differs: {name}")
    _require(_same_fingerprint(audit_inputs["base_water"], source_fine), "interface-audit base WATER provenance differs")
    _require(_same_fingerprint(audit_inputs["final_water"], candidate_fine), "interface-audit final WATER provenance differs")
    _require(_same_fingerprint(audit_inputs["fine_manifest"], fine_manifest), "interface-audit fine manifest provenance differs")
    _require(_same_fingerprint(audit_inputs["geometry_manifest"], geometry_manifest), "interface-audit geometry manifest provenance differs")
    _require(_same_fingerprint(audit_inputs["geometry_archive"], geometry_archive), "interface-audit geometry archive provenance differs")
    _require(_same_content(audit_inputs["sand_1mm"], fine_inputs["sand"]), "interface-audit SAND provenance differs")
    _require(_same_content(audit_inputs["rock_1mm"], fine_inputs["rock"]), "interface-audit ROCK provenance differs")
    _require(_same_fingerprint(audit_inputs["review_config"], geometry_config), "interface-audit review config provenance differs")
    _require(
        isinstance(audit_implementations, Mapping)
        and set(audit_implementations) == set(_INTERFACE_AUDIT_IMPLEMENTATIONS),
        "interface-audit implementation provenance set differs",
    )
    for name in _INTERFACE_AUDIT_IMPLEMENTATIONS:
        block = audit_implementations[name]
        _require(isinstance(block, Mapping), f"interface-audit implementation fingerprint is invalid: {name}")
        _fingerprint_keys(block, ply=False)
        _require(block.get("path") == str(SCRIPT_DIR / name), f"interface-audit implementation path differs: {name}")
    _require(
        isinstance(audit_append, Mapping)
        and audit_append.get("source_base_points") == source_fine.get("points")
        and audit_append.get("base_points")
        == geometry.get("surviving_source_points")
        and audit_append.get("removed_base_points")
        == geometry.get("removed_base_count")
        and audit_append.get("final_points") == candidate_fine.get("points")
        and audit_append.get("addition_count") == geometry.get("addition_count")
        and audit_append.get("base_payload_byte_exact") is True
        and audit_append.get("surviving_base_payload_byte_exact") is True
        and audit_append.get("surviving_base_row_order_preserved") is True
        and audit_append.get("suffix_xyz_archive_exact") is True,
        "interface-audit append provenance differs",
    )
    _require(
        isinstance(audit_checks, Mapping)
        and set(audit_checks) == _INTERFACE_AUDIT_CHECKS
        and all(value is True for value in audit_checks.values()),
        "interface-audit acceptance provenance differs",
    )
    _require(
        isinstance(audit_density, Mapping)
        and audit_density.get("circle_radius_m") == 0.08
        and audit_density.get("step_m") == 0.08
        and audit_density.get("minimum_ratio") == 0.85
        and audit_density.get("maximum_ratio") == 1.25
        and audit_density.get("post_build_lower_and_upper_bounds_passed") is True
        and audit_density.get("water_only_center_count_is_acceptance_criterion") is True,
        "interface-audit measured density provenance differs",
    )
    _require(
        downsample.get("fine_first") is True
        and downsample.get("coarse_scalar_recalculation") is False
        and downsample.get("minimum_spacing_m") == 0.005,
        "downsample stage provenance contract is incomplete",
    )
    _require(
        _same_fingerprint(downsample.get("fine_candidate", {}), candidate_fine)
        and _same_fingerprint(downsample.get("coarse_candidate", {}), candidate_coarse),
        "downsample stage is not bound to both release candidates",
    )
    _require(
        isinstance(downsample.get("executable"), Mapping)
        and downsample["executable"].get("path") == str(paths.downsample_executable),
        "downsample executable provenance path differs",
    )
    _fingerprint_keys(downsample["executable"], ply=False)
    downsample_manifest = downsample.get("manifest")
    downsample_report = downsample.get("native_report")
    _require(isinstance(downsample_manifest, Mapping), "downsample stage manifest fingerprint is missing")
    _require(isinstance(downsample_report, Mapping), "downsample stage report fingerprint is missing")
    _fingerprint_keys(downsample_manifest, ply=False)
    _fingerprint_keys(downsample_report, ply=False)
    _require(downsample_manifest.get("path") == str(paths.downsample_manifest), "downsample stage manifest provenance path differs")
    _require(downsample_report.get("path") == str(paths.downsample_report), "downsample report provenance path differs")
    _require(
        _same_content(downsample_report, manifest["downsample_verification"]["report"]),
        "downsample stage report provenance differs from native verification",
    )

    implementation_sets = (
        (geometry.get("implementation_sha256"), _GEOMETRY_IMPLEMENTATIONS, "geometry"),
        (fine.get("implementation_sha256"), _FINE_IMPLEMENTATIONS, "fine scalar"),
    )
    for declared, names, label in implementation_sets:
        _require(
            isinstance(declared, Mapping) and set(declared) == set(names),
            f"{label} implementation provenance set differs",
        )
        _require(
            all(
                isinstance(declared[name], str)
                and len(declared[name]) == 64
                and all(character in "0123456789abcdef" for character in declared[name])
                for name in names
            ),
            f"{label} implementation provenance contains an invalid hash",
        )

    artifacts = provenance.get("artifacts")
    provenance_snapshot_names = dict(_PROVENANCE_SNAPSHOT_NAMES)
    if far_lobe.get("performed") is True:
        provenance_snapshot_names.update(_FAR_LOBE_PROVENANCE_SNAPSHOT_NAMES)
    _require(
        isinstance(artifacts, Mapping)
        and set(artifacts) == set(provenance_snapshot_names),
        "compact provenance artifact set differs",
    )
    source_paths = {
        "fine_manifest": paths.fine_manifest,
        "geometry_manifest": paths.geometry_manifest,
        "geometry_archive": paths.geometry_archive,
        "fine_geometry_manifest_copy": paths.fine_manifest.parent / "geometry-manifest.json",
        "interface_audit_manifest": paths.interface_audit_manifest,
        "downsample_manifest": paths.downsample_manifest,
        "downsample_report": paths.downsample_report,
        "review_config": paths.review_config,
        "normalization_manifest": paths.normalization_manifest,
    }
    contract_sources = {
        "fine_manifest": fine.get("manifest"),
        "geometry_manifest": geometry.get("manifest"),
        "geometry_archive": geometry.get("archive"),
        "fine_geometry_manifest_copy": fine.get("geometry_manifest_copy"),
        "interface_audit_manifest": interface_audit.get("manifest"),
        "downsample_manifest": downsample.get("manifest"),
        "downsample_report": downsample.get("native_report"),
        "review_config": geometry.get("config"),
        "normalization_manifest": (
            fine.get("input_fingerprints", {}).get("normalization_manifest")
            if isinstance(fine.get("input_fingerprints"), Mapping)
            else None
        ),
    }
    if far_lobe.get("performed") is True:
        source_paths.update(
            far_lobe_records=Path(far_lobe["record_archive"]["path"]),
            far_lobe_source_indices=Path(
                far_lobe["source_index_archive"]["path"]
            ),
        )
        contract_sources.update(
            far_lobe_records=far_lobe["record_archive"],
            far_lobe_source_indices=far_lobe["source_index_archive"],
        )
    for name, filename in provenance_snapshot_names.items():
        row = artifacts[name]
        _require(isinstance(row, Mapping), f"provenance artifact row is invalid: {name}")
        source = row.get("source")
        snapshot = row.get("snapshot")
        _require(
            isinstance(source, Mapping) and isinstance(snapshot, Mapping),
            f"provenance fingerprints are missing: {name}",
        )
        _fingerprint_keys(source, ply=False)
        _fingerprint_keys(snapshot, ply=False)
        _require(source.get("path") == str(source_paths[name]), f"provenance source path differs: {name}")
        expected_snapshot = paths.release_dir / "provenance" / filename
        _require(snapshot.get("path") == str(expected_snapshot), f"provenance snapshot path differs: {name}")
        _require(_same_fingerprint(source, snapshot), f"provenance source/snapshot differ: {name}")
        _require(
            isinstance(contract_sources[name], Mapping)
            and _same_fingerprint(source, contract_sources[name]),
            f"provenance artifact is not bound to its stage contract: {name}",
        )


def _read_release(paths: ReleasePaths) -> tuple[Path, dict]:
    release = _strict_existing_directory(paths.release_dir, "release directory")
    _require(release == paths.run_dir / "release", "release directory is not <run>/release")
    manifest_path, manifest = _load_json(release / "manifest.json", "v12 release manifest")
    _require(manifest.get("schema_version") == SCHEMA_VERSION, "release schema mismatch")
    _require(manifest.get("operation") == OPERATION, "release operation mismatch")
    _require(manifest.get("status") in {"built", "installed", "restored"}, "invalid release status")
    _require(manifest.get("data_dir") == str(paths.data_dir), "release data directory mismatch")
    _require(manifest.get("run_dir") == str(paths.run_dir), "release run directory mismatch")
    _require(manifest.get("release_dir") == str(paths.release_dir), "release path mismatch")
    clouds = manifest.get("clouds")
    _require(isinstance(clouds, Mapping) and set(clouds) == set(CANONICAL_BY_LABEL), "release must contain exactly two WATER clouds")

    for label, canonical_name in CANONICAL_BY_LABEL.items():
        cloud = clouds[label]
        _require(isinstance(cloud, Mapping), f"{label} manifest entry is invalid")
        _require(
            isinstance(cloud.get("source"), Mapping)
            and isinstance(cloud.get("candidate"), Mapping)
            and isinstance(cloud.get("snapshot"), Mapping),
            f"{label} fingerprint blocks are invalid",
        )
        _fingerprint_keys(cloud["source"], ply=True)
        _fingerprint_keys(cloud["candidate"], ply=True)
        _fingerprint_keys(cloud["snapshot"], ply=True)
        _require(cloud.get("canonical") == canonical_name, f"{label} canonical name mismatch")
        canonical = _parent_resolved_path(_canonical(paths, label), f"{label} canonical")
        _require(canonical == paths.data_dir / canonical_name, f"{label} canonical escapes data directory")
        expected_candidate = paths.candidate_2mm if label == "WATER-2mm" else paths.candidate_5mm
        candidate = _parent_resolved_path(cloud.get("candidate_path", ""), f"{label} candidate")
        _require(candidate == expected_candidate, f"{label} candidate path mismatch")
        _require(candidate != canonical, f"{label} candidate aliases canonical")
        _require_beneath(candidate, paths.run_dir, f"{label} candidate")
        _require(not candidate.is_relative_to(paths.release_dir), f"{label} candidate is inside release")
        snapshot = _parent_resolved_path(cloud["snapshot"].get("path", ""), f"{label} snapshot")
        expected_snapshot = paths.release_dir / "source-snapshots" / canonical_name
        _require(snapshot == expected_snapshot, f"{label} snapshot path mismatch")
        _require(cloud["source"].get("path") == str(canonical), f"{label} source path mismatch")
        _require(cloud["candidate"].get("path") == str(candidate), f"{label} candidate fingerprint path mismatch")
        _require(
            cloud["source"].get("schema") == cloud["candidate"].get("schema")
            and cloud["source"].get("record_stride")
            == cloud["candidate"].get("record_stride"),
            f"{label} source/candidate schema contract differs",
        )

    candidate_paths = [paths.candidate_2mm, paths.candidate_5mm]
    _require(len(set(candidate_paths)) == 2, "candidate paths alias each other")
    report_block = manifest.get("downsample_verification")
    _require(isinstance(report_block, Mapping), "downsample verification is missing")
    report_fp = report_block.get("report")
    _require(isinstance(report_fp, Mapping), "downsample report fingerprint is missing")
    _fingerprint_keys(report_fp, ply=False)
    _require(report_fp.get("path") == str(paths.downsample_report), "downsample report path mismatch")
    relation = manifest.get("cross_scale_verification")
    _require(
        isinstance(relation, Mapping)
        and relation.get("verified") is True
        and relation.get("relation") == "byte-exact ordered full-record subsequence",
        "cross-scale full-record proof is missing",
    )
    fine_fp = clouds["WATER-2mm"]["candidate"]
    coarse_fp = clouds["WATER-5mm"]["candidate"]
    _require(
        fine_fp.get("schema") == coarse_fp.get("schema")
        and fine_fp.get("record_stride") == coarse_fp.get("record_stride"),
        "fine/coarse candidate schema contract differs",
    )
    _require(
        relation.get("fine_points") == fine_fp.get("points")
        and relation.get("coarse_points") == coarse_fp.get("points")
        and relation.get("matched_points") == coarse_fp.get("points")
        and relation.get("record_stride") == fine_fp.get("record_stride"),
        "cross-scale proof counts/stride differ from candidate fingerprints",
    )
    _require(
        report_block.get("verified") is True
        and report_block.get("method") == "greedy_spatial_minimum_distance"
        and math.isclose(
            _finite_number(
                report_block.get("minimum_spacing_m"),
                "manifest downsample minimum spacing",
            ),
            MINIMUM_COARSE_SPACING_M,
            rel_tol=0.0,
            abs_tol=1.0e-12,
        )
        and report_block.get("source_points") == fine_fp.get("points")
        and report_block.get("output_points") == coarse_fp.get("points")
        and report_block.get("record_stride") == fine_fp.get("record_stride")
        and report_block.get("non_finite_positions") == 0
        and report_block.get("input") == str(paths.candidate_2mm)
        and report_block.get("output") == str(paths.candidate_5mm),
        "CleanMesh report contract differs from candidate fingerprints",
    )
    expected_invariants = {
        "exactly_two_water_canonicals",
        "coarse_is_ordered_full_record_subsequence",
        "cleanmesh_native_report_hash_locked",
        "upstream_v12_provenance_hash_locked",
        "fine_candidate_bound_to_scalar_manifest",
        "fine_and_coarse_component_scalar_coverage_directly_verified",
        "geometry_base_archive_config_implementation_bound",
        "passed_interface_audit_hash_locked",
        "passed_interface_audit_independently_recomputed",
        "downsample_stage_binds_fine_coarse_report_executable",
        "coarse_scalar_recalculation_forbidden",
        "compact_provenance_snapshots_hash_locked",
        "source_snapshots_hash_locked",
        "restore_uses_source_snapshots",
        "preexisting_water_old01_untouched",
    }
    invariants = manifest.get("invariants")
    _require(
        isinstance(invariants, Mapping)
        and all(invariants.get(name) is True for name in expected_invariants),
        "release invariant set is incomplete",
    )
    protected = manifest.get("protected_existing_water_old01")
    _require(isinstance(protected, Mapping), "protected old01 fingerprint missing")
    _fingerprint_keys(protected, ply=True)
    _require(protected.get("path") == str(_old01(paths)), "protected old01 path mismatch")
    _validate_release_provenance(paths, manifest)
    return manifest_path, manifest


def _assert_old01(paths: ReleasePaths, manifest: Mapping) -> None:
    unexpected = paths.data_dir / "Site1-WATER-2mm-old01.ply"
    _require(
        not unexpected.exists() and not unexpected.is_symlink(),
        "unexpected Site1-WATER-2mm-old01.ply makes rollback ambiguous",
    )
    _assert_fingerprint(
        _old01(paths),
        manifest["protected_existing_water_old01"],
        "protected Site1-WATER-5mm-old01.ply",
    )


def _verify_release_files(paths: ReleasePaths, manifest: Mapping) -> dict:
    _assert_old01(paths, manifest)
    _assert_fingerprint(
        paths.downsample_report,
        manifest["downsample_verification"]["report"],
        "CleanMesh downsample report",
    )
    for label, cloud in manifest["clouds"].items():
        _assert_fingerprint(cloud["snapshot"]["path"], cloud["source"], f"{label} snapshot")
    provenance_artifacts = manifest["upstream_provenance"]["artifacts"]
    for name, row in provenance_artifacts.items():
        _assert_fingerprint(
            row["snapshot"]["path"], row["source"],
            f"{name} compact provenance snapshot",
        )

    status = manifest["status"]
    fine_location: Path
    coarse_location: Path
    for label, cloud in manifest["clouds"].items():
        canonical = _canonical(paths, label)
        candidate = Path(cloud["candidate_path"])
        if status == "installed":
            _assert_fingerprint(canonical, cloud["candidate"], f"{label} installed canonical")
            _require(not candidate.exists() and not candidate.is_symlink(), f"{label} candidate path unexpectedly occupied while installed")
        else:
            _assert_fingerprint(canonical, cloud["source"], f"{label} source canonical")
            _assert_fingerprint(candidate, cloud["candidate"], f"{label} candidate")

    distinct = {
        "CleanMesh report": _strict_existing_file(
            paths.downsample_report, "CleanMesh report"
        ),
        "protected WATER old01": _strict_existing_file(
            _old01(paths), "protected WATER old01"
        ),
    }
    for label, cloud in manifest["clouds"].items():
        distinct[f"{label} canonical"] = _strict_existing_file(
            _canonical(paths, label), f"{label} canonical"
        )
        distinct[f"{label} snapshot"] = _strict_existing_file(
            cloud["snapshot"]["path"], f"{label} snapshot"
        )
        if status != "installed":
            distinct[f"{label} candidate"] = _strict_existing_file(
                cloud["candidate_path"], f"{label} candidate"
            )
    for name, row in provenance_artifacts.items():
        distinct[f"provenance snapshot {name}"] = _strict_existing_file(
            row["snapshot"]["path"], f"provenance snapshot {name}"
        )
    _require_distinct_existing(distinct)

    if status == "installed":
        fine_location = _canonical(paths, "WATER-2mm")
        coarse_location = _canonical(paths, "WATER-5mm")
    else:
        fine_location = paths.candidate_2mm
        coarse_location = paths.candidate_5mm
    # Hash equality to the audited candidate fingerprints makes the original
    # streaming proof applicable at whichever publication paths are active.
    fine_actual = file_fingerprint(fine_location)
    coarse_actual = file_fingerprint(coarse_location)
    _require(_same_fingerprint(fine_actual, manifest["clouds"]["WATER-2mm"]["candidate"]), "active fine generation differs from audited candidate")
    _require(_same_fingerprint(coarse_actual, manifest["clouds"]["WATER-5mm"]["candidate"]), "active coarse generation differs from audited candidate")
    stored_fine_audit = manifest["upstream_provenance"]["fine_scalar_enrichment"][
        "component_field_scalar_coverage"
    ]
    stored_coarse_audit = manifest["upstream_provenance"][
        "coarse_component_field_scalar_coverage"
    ]
    archive_snapshot = Path(
        provenance_artifacts["geometry_archive"]["snapshot"]["path"]
    )
    try:
        with np.load(archive_snapshot, allow_pickle=False) as loaded:
            _require(
                "candidate_label" in loaded.files,
                "release geometry archive snapshot lacks candidate_label",
            )
            component_labels = np.asarray(loaded["candidate_label"]).copy()
    except (OSError, ValueError) as error:
        raise RuntimeError(
            "unable to load release geometry labels for active scalar verification"
        ) from error
    direct_fine_audit = scalar_enrichment.verify_candidate_component_scalar_coverage(
        fine_location,
        base_points=int(stored_fine_audit["base_points"]),
        component_labels=component_labels,
        context="active release 2mm WATER addition suffix",
    )
    fine_comparison_keys = (
        "method",
        "candidate_points",
        "base_points",
        "addition_count",
        "component_label_sha256",
        "geometry_fields",
        "coverage",
        "all_required_fields_accepted",
    )
    _require(
        all(
            direct_fine_audit.get(key) == stored_fine_audit.get(key)
            for key in fine_comparison_keys
        ),
        "active fine component scalar audit differs from release provenance",
    )
    direct_coarse_audit = (
        scalar_enrichment.verify_coarse_exact_subset_component_scalar_coverage(
            fine_location,
            coarse_location,
            fine_base_points=int(stored_fine_audit["base_points"]),
            fine_component_labels=component_labels,
            chunk_records=CHUNK_RECORDS,
        )
    )
    coarse_comparison_keys = (
        "method",
        "fine_candidate_points",
        "coarse_candidate_points",
        "fine_base_points",
        "fine_addition_count",
        "matched_coarse_addition_count",
        "fine_component_labels",
        "coarse_component_labels",
        "component_label_sha256",
        "geometry_fields",
        "coverage",
        "full_record_membership_exact",
        "every_fine_component_represented",
        "all_required_fields_accepted",
    )
    _require(
        all(
            direct_coarse_audit.get(key) == stored_coarse_audit.get(key)
            for key in coarse_comparison_keys
        ),
        "active coarse component scalar audit differs from release provenance",
    )
    return {
        "verified": True,
        "status": status,
        "old01_untouched": True,
        "interface_audit_provenance_verified": True,
        "active_fine": str(fine_location),
        "active_coarse": str(coarse_location),
        "fine_component_scalar_coverage_verified": True,
        "coarse_component_scalar_coverage_verified": True,
    }


_TERMINAL_JOURNAL_STATES = {
    "committed",
    "committed-recovered",
    "rolled-back",
    "rolled-back-after-recovery",
}


def _write_journal(path: Path, journal: dict, *, overwrite: bool = False) -> None:
    journal["updated"] = _now()
    _atomic_json(path, journal, overwrite=overwrite)


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
    _write_journal(path, journal, overwrite=True)


def _path_matches(path: Path, expected: Mapping) -> bool:
    if not path.exists() or path.is_symlink():
        return False
    try:
        _assert_fingerprint(path, expected, str(path))
    except Exception:
        return False
    return True


@dataclass
class SwapItem:
    label: str
    canonical: Path
    replacement: Path
    archive: Path
    expected_current: Mapping
    expected_replacement: Mapping


def _journal_items(
    paths: ReleasePaths,
    manifest: Mapping,
    transaction_dir: Path,
    action: str,
) -> list[SwapItem]:
    result = []
    for label, cloud in manifest["clouds"].items():
        canonical = _canonical(paths, label)
        if action == "install":
            replacement = Path(cloud["candidate_path"])
            archive = transaction_dir / "previous" / cloud["canonical"]
            current_fp = cloud["source"]
            replacement_fp = cloud["candidate"]
        else:
            replacement = transaction_dir / "restore-replacements" / cloud["canonical"]
            archive = Path(cloud["candidate_path"])
            current_fp = cloud["candidate"]
            replacement_fp = cloud["source"]
        result.append(
            SwapItem(label, canonical, replacement, archive, current_fp, replacement_fp)
        )
    return result


def _validate_transaction_paths(items: Sequence[SwapItem]) -> None:
    groups = {
        "canonical": [item.canonical for item in items],
        "replacement": [item.replacement for item in items],
        "archive": [item.archive for item in items],
    }
    for name, values in groups.items():
        _require(len(set(values)) == len(values), f"transaction {name} paths are duplicated")
    all_paths = groups["canonical"] + groups["replacement"] + groups["archive"]
    _require(len(set(all_paths)) == len(all_paths), "transaction paths alias across roles")
    existing = {}
    for item in items:
        existing[f"{item.label} canonical"] = _strict_existing_file(item.canonical, f"{item.label} canonical")
        existing[f"{item.label} replacement"] = _strict_existing_file(item.replacement, f"{item.label} replacement")
        archive = _parent_resolved_path(item.archive, f"{item.label} archive")
        _require(not archive.exists(), f"refusing to overwrite transaction archive: {archive}")
    _require_distinct_existing(existing)
    devices = {
        path.stat().st_dev for path in existing.values()
    } | {
        item.archive.parent.stat().st_dev for item in items
    }
    _require(len(devices) == 1, "transaction paths are not on one filesystem")


def _new_transaction_dir(paths: ReleasePaths, action: str) -> Path:
    root = paths.release_dir / "transactions"
    root.mkdir(parents=True, exist_ok=True)
    _strict_existing_directory(root, "transaction root")
    stamp = dt.datetime.now().strftime("%Y%m%dT%H%M%S.%f")
    return root / f"{action}-{stamp}-{os.getpid()}-{uuid.uuid4().hex[:8]}"


def _manifest_status(path: Path) -> str | None:
    if not path.exists() or path.is_symlink():
        return None
    try:
        return json.loads(path.read_text(encoding="utf-8")).get("status")
    except (OSError, json.JSONDecodeError, AttributeError):
        return None


def _commit_status(
    manifest_path: Path,
    manifest: Mapping,
    *,
    status: str,
    action: str,
    transaction: Mapping,
) -> None:
    updated = json.loads(json.dumps(manifest))
    updated["status"] = status
    updated[f"{action}_at"] = _now()
    history = list(updated.get("transactions", []))
    history.append({"action": action, **dict(transaction)})
    updated["transactions"] = history
    _atomic_json(manifest_path, updated, overwrite=True)


def _rollback_journal(
    journal_path: Path,
    journal: dict,
    *,
    recovered: bool,
    reason: str,
) -> None:
    refuse_running_app()
    _journal_phase(journal_path, journal, "rollback-started", detail=reason)
    # First prove every location belongs to one of the finite, recoverable
    # states.  No rename occurs until the complete transaction is unambiguous.
    states = []
    for row in journal["items"]:
        canonical = Path(row["canonical"])
        replacement = Path(row["replacement"])
        archive = Path(row["archive"])
        current = row["expected_current"]
        new = row["expected_replacement"]
        canonical_current = _path_matches(canonical, current)
        canonical_new = _path_matches(canonical, new)
        replacement_new = _path_matches(replacement, new)
        archive_current = _path_matches(archive, current)
        occupied_wrong = (
            (canonical.exists() and not canonical_current and not canonical_new)
            or (replacement.exists() and not replacement_new)
            or (archive.exists() and not archive_current)
        )
        _require(not occupied_wrong, f"{row['label']}: transaction location has unknown content")
        if canonical_current and replacement_new and not archive.exists():
            state_name = "untouched"
        elif not canonical.exists() and replacement_new and archive_current:
            state_name = "archived"
        elif canonical_new and not replacement.exists() and archive_current:
            state_name = "installed"
        else:
            raise RuntimeError(f"{row['label']}: ambiguous transaction state")
        states.append((row, state_name))

    for row, state_name in reversed(states):
        canonical = Path(row["canonical"])
        replacement = Path(row["replacement"])
        archive = Path(row["archive"])
        if state_name == "installed":
            refuse_running_app()
            _journal_phase(journal_path, journal, "before-return-replacement", label=row["label"])
            _durable_replace(canonical, replacement)
        if state_name in {"installed", "archived"}:
            refuse_running_app()
            _journal_phase(journal_path, journal, "before-restore-current", label=row["label"])
            _durable_replace(archive, canonical)
        _assert_fingerprint(canonical, row["expected_current"], f"{row['label']} rolled-back canonical")
        _assert_fingerprint(replacement, row["expected_replacement"], f"{row['label']} returned replacement")

    final = "rolled-back-after-recovery" if recovered else "rolled-back"
    _journal_phase(journal_path, journal, final, detail=reason)


def _validate_recovery_journal(
    paths: ReleasePaths,
    manifest: Mapping,
    journal_path: Path,
    journal: Mapping,
) -> None:
    _require(journal.get("schema_version") == JOURNAL_SCHEMA_VERSION, "invalid transaction journal schema")
    _require(journal.get("operation") == JOURNAL_OPERATION, "invalid transaction journal operation")
    action = journal.get("action")
    _require(action in {"install", "restore"}, "invalid transaction action")
    expected_source = {"install": {"built", "restored"}, "restore": {"installed"}}[action]
    expected_target = {"install": "installed", "restore": "restored"}[action]
    _require(journal.get("source_status") in expected_source, "invalid journal source status")
    _require(journal.get("target_status") == expected_target, "invalid journal target status")

    root = _strict_existing_directory(paths.release_dir / "transactions", "transaction root")
    resolved_journal = _strict_existing_file(journal_path, "transaction journal")
    transaction_dir = resolved_journal.parent
    _require(transaction_dir.parent == root, "transaction journal escapes transaction root")
    _require(transaction_dir.name.startswith(f"{action}-"), "transaction directory/action mismatch")
    _require(resolved_journal == transaction_dir / "journal.json", "unexpected transaction journal name")
    rows = journal.get("items")
    _require(isinstance(rows, list) and len(rows) == 2, "transaction journal must contain two items")
    _require({row.get("label") for row in rows} == set(CANONICAL_BY_LABEL), "journal WATER set mismatch")
    expected_items = {item.label: item for item in _journal_items(paths, manifest, transaction_dir, action)}
    for row in rows:
        item = expected_items[row["label"]]
        _require(row.get("canonical") == str(item.canonical), f"{item.label} journal canonical mismatch")
        _require(row.get("replacement") == str(item.replacement), f"{item.label} journal replacement mismatch")
        _require(row.get("archive") == str(item.archive), f"{item.label} journal archive mismatch")
        _require(isinstance(row.get("expected_current"), Mapping) and _same_fingerprint(row["expected_current"], item.expected_current), f"{item.label} journal current fingerprint mismatch")
        _require(isinstance(row.get("expected_replacement"), Mapping) and _same_fingerprint(row["expected_replacement"], item.expected_replacement), f"{item.label} journal replacement fingerprint mismatch")


def _committed_layout(journal: Mapping) -> bool:
    return all(
        _path_matches(Path(row["canonical"]), row["expected_replacement"])
        and not Path(row["replacement"]).exists()
        and _path_matches(Path(row["archive"]), row["expected_current"])
        for row in journal["items"]
    )


def recover_incomplete_transactions(paths: ReleasePaths) -> list[dict]:
    root = paths.release_dir / "transactions"
    if not root.exists():
        return []
    _strict_existing_directory(root, "transaction root")
    _, manifest = _read_release(paths)
    _assert_old01(paths, manifest)
    recovered = []
    for journal_path in sorted(root.glob("*/journal.json")):
        _, journal = _load_json(journal_path, "transaction journal")
        if journal.get("state") in _TERMINAL_JOURNAL_STATES:
            continue
        _validate_recovery_journal(paths, manifest, journal_path, journal)
        observed = _manifest_status(_manifest_path(paths))
        target = journal["target_status"]
        source = journal["source_status"]
        if observed == target and _committed_layout(journal):
            _journal_phase(journal_path, journal, "committed-recovered", detail="manifest and both WATER swaps were durable")
            recovered.append({"journal": str(journal_path), "outcome": "committed-recovered"})
        elif observed == source:
            _rollback_journal(
                journal_path,
                journal,
                recovered=True,
                reason="startup recovery of incomplete WATER transaction",
            )
            recovered.append({"journal": str(journal_path), "outcome": "rolled-back-after-recovery"})
        else:
            raise RuntimeError(
                f"ambiguous incomplete transaction {journal_path.parent.name}: "
                f"manifest status {observed!r}, expected {source!r} or {target!r}"
            )
    return recovered


def _transactional_replace(
    paths: ReleasePaths,
    manifest_path: Path,
    manifest: Mapping,
    items: Sequence[SwapItem],
    *,
    transaction_dir: Path,
    action: str,
    source_status: str,
    target_status: str,
    prepared: bool = False,
    protected_check: Callable[[], None] | None = None,
) -> dict:
    if prepared:
        transaction_dir = _strict_existing_directory(transaction_dir, "prepared transaction directory")
    else:
        transaction_dir.mkdir(parents=False)
        _fsync_directory(transaction_dir.parent)
    previous = transaction_dir / "previous"
    previous.mkdir(exist_ok=False)
    _fsync_directory(transaction_dir)
    for item in items:
        item.archive.parent.mkdir(parents=True, exist_ok=True)
    _validate_transaction_paths(items)
    for item in items:
        _assert_fingerprint(item.canonical, item.expected_current, f"{item.label} current canonical")
        _assert_fingerprint(item.replacement, item.expected_replacement, f"{item.label} replacement")

    journal_path = transaction_dir / "journal.json"
    journal = {
        "schema_version": JOURNAL_SCHEMA_VERSION,
        "operation": JOURNAL_OPERATION,
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
                "archive": str(item.archive),
                "expected_current": dict(item.expected_current),
                "expected_replacement": dict(item.expected_replacement),
                "phase": "intent",
            }
            for item in items
        ],
        "events": [{"at": _now(), "phase": "intent"}],
    }
    _write_journal(journal_path, journal)
    try:
        refuse_running_app()
        for item in items:
            refuse_running_app()
            if protected_check is not None:
                protected_check()
            _assert_fingerprint(item.canonical, item.expected_current, f"{item.label} pre-swap canonical")
            _assert_fingerprint(item.replacement, item.expected_replacement, f"{item.label} pre-swap replacement")
            _journal_phase(journal_path, journal, "before-archive-current", label=item.label)
            _durable_replace(item.canonical, item.archive)
            _journal_phase(journal_path, journal, "archived-current", label=item.label)
            if protected_check is not None:
                protected_check()
            _journal_phase(journal_path, journal, "before-install-replacement", label=item.label)
            _durable_replace(item.replacement, item.canonical)
            _journal_phase(journal_path, journal, "installed-replacement", label=item.label)

        for item in items:
            _assert_fingerprint(item.canonical, item.expected_replacement, f"{item.label} installed canonical")
            _assert_fingerprint(item.archive, item.expected_current, f"{item.label} archived generation")
        if protected_check is not None:
            protected_check()
        transaction = {
            "created": _now(),
            "directory": str(transaction_dir),
            "journal": str(journal_path),
            "restore_source": "release source snapshots" if action == "restore" else None,
            "archives": {item.label: str(item.archive) for item in items},
        }
        _journal_phase(journal_path, journal, "before-manifest-commit")
        _commit_status(
            manifest_path,
            manifest,
            status=target_status,
            action=action,
            transaction=transaction,
        )
        _journal_phase(journal_path, journal, "committed")
        return transaction
    except BaseException as original_error:
        if _manifest_status(manifest_path) == target_status and _committed_layout(journal):
            try:
                _journal_phase(journal_path, journal, "committed-recovered", detail=f"manifest commit durable; finalisation raised {original_error}")
            except Exception:
                pass
            return {
                "directory": str(transaction_dir),
                "journal": str(journal_path),
                "commit_recovered_after_error": str(original_error),
            }
        try:
            _rollback_journal(
                journal_path,
                journal,
                recovered=False,
                reason=f"{type(original_error).__name__}: {original_error}",
            )
        except Exception as rollback_error:
            raise RuntimeError(
                f"WATER transaction failed ({original_error}); rollback also "
                f"failed ({rollback_error}). Journal: {journal_path}"
            ) from rollback_error
        raise


def verify(args, *, acquire_lock: bool = True) -> dict:
    paths = _coerce_paths(args)
    if acquire_lock:
        with release_lock(paths.run_dir):
            recovered = recover_incomplete_transactions(paths)
            _, manifest = _read_release(paths)
            result = _verify_release_files(paths, manifest)
            result["recovered_transactions"] = recovered
            return result
    recovered = recover_incomplete_transactions(paths)
    _, manifest = _read_release(paths)
    result = _verify_release_files(paths, manifest)
    result["recovered_transactions"] = recovered
    return result


def install(args) -> dict:
    paths = _coerce_paths(args)
    with release_lock(paths.run_dir):
        refuse_running_app()
        recover_incomplete_transactions(paths)
        manifest_path, manifest = _read_release(paths)
        _require(manifest["status"] in {"built", "restored"}, "release is already installed or has invalid status")
        _verify_release_files(paths, manifest)
        transaction_dir = _new_transaction_dir(paths, "install")
        transaction = _transactional_replace(
            paths,
            manifest_path,
            manifest,
            _journal_items(paths, manifest, transaction_dir, "install"),
            transaction_dir=transaction_dir,
            action="install",
            source_status=manifest["status"],
            target_status="installed",
            protected_check=lambda: _assert_old01(paths, manifest),
        )
        _assert_old01(paths, manifest)
        return {"installed": True, "transaction": transaction}


def restore(args) -> dict:
    paths = _coerce_paths(args)
    with release_lock(paths.run_dir):
        refuse_running_app()
        recover_incomplete_transactions(paths)
        manifest_path, manifest = _read_release(paths)
        _require(manifest["status"] == "installed", "release is not installed")
        _verify_release_files(paths, manifest)
        transaction_dir = _new_transaction_dir(paths, "restore")
        transaction_dir.mkdir(parents=False)
        try:
            restore_dir = transaction_dir / "restore-replacements"
            restore_dir.mkdir()
            methods = {}
            for label, cloud in manifest["clouds"].items():
                destination = restore_dir / cloud["canonical"]
                methods[label] = _clone_or_copy(Path(cloud["snapshot"]["path"]), destination)
                _assert_fingerprint(destination, cloud["source"], f"{label} snapshot restore replacement")
            transaction = _transactional_replace(
                paths,
                manifest_path,
                manifest,
                _journal_items(paths, manifest, transaction_dir, "restore"),
                transaction_dir=transaction_dir,
                action="restore",
                source_status="installed",
                target_status="restored",
                prepared=True,
                protected_check=lambda: _assert_old01(paths, manifest),
            )
        except BaseException:
            # Before the durable journal exists this directory contains only
            # private clone material.  Do not leave a multi-gigabyte orphan if
            # preparing the second WATER replacement fails.  Once journalled,
            # recovery owns the directory and it must remain intact.
            journal = transaction_dir / "journal.json"
            if transaction_dir.exists() and not journal.exists():
                shutil.rmtree(transaction_dir)
                _fsync_directory(transaction_dir.parent)
            raise
        transaction["snapshot_materialisation"] = methods
        _assert_old01(paths, manifest)
        return {"restored": True, "transaction": transaction}


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    result.add_argument("stage", choices=("build", "verify", "install", "restore"))
    result.add_argument("--data-dir", type=Path, default=DEFAULT_DATA)
    result.add_argument("--run-dir", type=Path, default=DEFAULT_RUN)
    result.add_argument("--release-dir", type=Path)
    result.add_argument("--candidate-2mm", type=Path)
    result.add_argument("--candidate-5mm", type=Path)
    result.add_argument("--downsample-report", type=Path)
    result.add_argument("--fine-manifest", type=Path)
    result.add_argument("--geometry-manifest", type=Path)
    result.add_argument("--geometry-archive", type=Path)
    result.add_argument("--interface-audit-manifest", type=Path)
    result.add_argument("--downsample-manifest", type=Path)
    result.add_argument("--review-config", type=Path, default=DEFAULT_REVIEW_CONFIG)
    result.add_argument(
        "--normalization-manifest", type=Path,
        default=DEFAULT_NORMALIZATION_MANIFEST,
    )
    result.add_argument(
        "--cleanmesh", type=Path, default=DEFAULT_CLEANMESH_EXECUTABLE,
    )
    result.add_argument(
        "--downsample", type=Path, default=DEFAULT_DOWNSAMPLE_EXECUTABLE,
    )
    return result


def main(argv: Sequence[str] | None = None) -> int:
    args = parser().parse_args(argv)
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
