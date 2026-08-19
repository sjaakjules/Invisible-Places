#!/usr/bin/env python3
"""Build a deterministic, locally cached Scene3 display-density bundle.

The canonical 1 mm PLY files are opened read-only.  The builder derives all
three role files in one private staging directory and publishes neither the
bundle nor its active pointer until every role has passed count, schema, size,
and full-file hash validation.

The sampling is density preserving rather than one-point-per-voxel.  A 5 mm
cell containing ``n`` parents receives approximately ``n * K / N`` output
strata, where ``K`` is the requested role budget and ``N`` is its source point
count.  Seeded systematic rounding makes the total exactly ``K`` and avoids an
occupancy-class cutoff.  Its per-cell rounding error is less than one output
point; stable hashes define traversal without relying on dictionary iteration.
Each non-empty stratum keeps one real parent position and prefilters the
remaining attributes.

Peak RAM is bounded by one hash shard rather than the complete cloud.  The
tradeoff is temporary disk approximately equal to the largest source role:

1. stream and hash the source into deterministic spatial shards;
2. sort each shard by cell and point hash while collecting cell populations;
3. apportion exact quotas and aggregate each sorted shard into the output;
4. independently re-read and validate the output before publishing.

RGB defaults to a numeric byte-domain mean because that is how the current
1 mm renderer interprets PLY RGB.  ``--rgb-filter srgb-linear-light`` exists
only for an explicitly recorded comparison; changing the final 1 mm colour
decoding is deliberately outside this tool's scope.
"""

from __future__ import annotations

import argparse
import dataclasses
import datetime as dt
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import sys
import uuid
from typing import BinaryIO, Sequence

try:
    import numpy as np
except ImportError as exc:  # pragma: no cover - exercised by the CLI host.
    raise SystemExit(
        "This cache builder requires NumPy. Install the repository's Python "
        "dependencies before rebuilding the display-density cache."
    ) from exc


SCHEMA_VERSION = 1
ALGORITHM_ID = "scene-display-density-stratified-prefilter-v1"
ALGORITHM_VERSION = 1
MANIFEST_NAME = "display-density-manifest.json"
ACTIVE_POINTER_NAME = "active-bundle.json"
DEFAULT_SEED = 0x4950565F53433331  # "IPV_SC31"
FREE_SPACE_MARGIN_BYTES = 2 * 1024 * 1024 * 1024
DEFAULT_TARGETS = {
    "ROCK": 2_317_741,
    "SAND": 8_586_125,
    "VEG": 3_732_504,
}
ROLE_ORDER = ("ROCK", "SAND", "VEG")
RGB_FILTER_RENDERER_BYTE = "renderer-byte-mean"
RGB_FILTER_SRGB_LINEAR = "srgb-linear-light"

_PLY_NUMPY_TYPES = {
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


@dataclasses.dataclass(frozen=True)
class PlyProperty:
    type_name: str
    name: str


@dataclasses.dataclass(frozen=True)
class PlyDescription:
    path: Path
    raw_header: bytes
    header_lines: tuple[str, ...]
    data_offset: int
    vertex_count: int
    properties: tuple[PlyProperty, ...]
    dtype: np.dtype
    schema_sha256: str


@dataclasses.dataclass(frozen=True)
class SourceProof:
    path: str
    size_bytes: int
    mtime_ns: int
    sha256: str
    vertex_count: int
    schema_sha256: str


@dataclasses.dataclass(frozen=True)
class BuildConfig:
    source_root: Path
    cache_root: Path
    targets: dict[str, int]
    voxel_size_m: float = 0.005
    shard_count: int = 64
    seed: int = DEFAULT_SEED
    rgb_filter: str = RGB_FILTER_RENDERER_BYTE
    chunk_records: int = 500_000
    keep_staging_on_error: bool = False


def _canonical_json(value: object) -> bytes:
    return json.dumps(
        value,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")


def _sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def _sha256_file(path: Path, chunk_bytes: int = 8 * 1024 * 1024) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(chunk_bytes):
            digest.update(chunk)
    return digest.hexdigest()


def _schema_payload(
    format_name: str,
    properties: Sequence[PlyProperty],
) -> dict[str, object]:
    return {
        "format": format_name,
        "vertex_properties": [
            {"type": prop.type_name, "name": prop.name} for prop in properties
        ],
    }


def read_ply_description(path: Path) -> PlyDescription:
    """Parse the fixed-width binary-LE vertex schema needed by the builder."""

    raw_lines: list[bytes] = []
    decoded_lines: list[str] = []
    with path.open("rb") as stream:
        for _ in range(100_000):
            raw = stream.readline()
            if not raw:
                raise ValueError(f"{path}: EOF before end_header")
            raw_lines.append(raw)
            try:
                line = raw.rstrip(b"\r\n").decode("ascii")
            except UnicodeDecodeError as exc:
                raise ValueError(f"{path}: PLY header is not ASCII") from exc
            decoded_lines.append(line)
            if line == "end_header":
                break
        else:
            raise ValueError(f"{path}: PLY header is unreasonably long")

    if not decoded_lines or decoded_lines[0] != "ply":
        raise ValueError(f"{path}: missing PLY magic")

    format_name = ""
    current_element: str | None = None
    vertex_count: int | None = None
    properties: list[PlyProperty] = []
    unsupported_elements: list[str] = []
    for line in decoded_lines[1:]:
        tokens = line.split()
        if not tokens:
            continue
        if tokens[0] == "format":
            if len(tokens) < 3:
                raise ValueError(f"{path}: malformed format line")
            format_name = tokens[1]
        elif tokens[0] == "element":
            if len(tokens) != 3:
                raise ValueError(f"{path}: malformed element line: {line}")
            current_element = tokens[1]
            count = int(tokens[2])
            if current_element == "vertex":
                if vertex_count is not None:
                    raise ValueError(f"{path}: duplicate vertex element")
                vertex_count = count
            elif count != 0:
                unsupported_elements.append(current_element)
        elif tokens[0] == "property" and current_element == "vertex":
            if len(tokens) >= 2 and tokens[1] == "list":
                raise ValueError(f"{path}: list-valued vertex properties are unsupported")
            if len(tokens) != 3:
                raise ValueError(f"{path}: malformed vertex property: {line}")
            if tokens[1] not in _PLY_NUMPY_TYPES:
                raise ValueError(f"{path}: unsupported PLY type {tokens[1]!r}")
            properties.append(PlyProperty(tokens[1], tokens[2]))

    if format_name != "binary_little_endian":
        raise ValueError(
            f"{path}: expected binary_little_endian, found {format_name or 'none'}"
        )
    if vertex_count is None or vertex_count <= 0:
        raise ValueError(f"{path}: missing or empty vertex element")
    if unsupported_elements:
        raise ValueError(
            f"{path}: non-empty non-vertex elements are unsupported: "
            + ", ".join(unsupported_elements)
        )
    names = [prop.name for prop in properties]
    if len(set(names)) != len(names):
        raise ValueError(f"{path}: duplicate vertex property names")
    for required in ("x", "y", "z"):
        if required not in names:
            raise ValueError(f"{path}: missing required property {required!r}")

    dtype = np.dtype(
        [(prop.name, _PLY_NUMPY_TYPES[prop.type_name]) for prop in properties],
        align=False,
    )
    raw_header = b"".join(raw_lines)
    schema_sha256 = _sha256_bytes(
        _canonical_json(_schema_payload(format_name, properties))
    )
    return PlyDescription(
        path=path,
        raw_header=raw_header,
        header_lines=tuple(decoded_lines),
        data_offset=len(raw_header),
        vertex_count=vertex_count,
        properties=tuple(properties),
        dtype=dtype,
        schema_sha256=schema_sha256,
    )


def _output_header(
    description: PlyDescription,
    output_count: int,
    voxel_size_m: float,
    seed: int,
    rgb_filter: str,
) -> bytes:
    lines: list[str] = []
    replaced_vertex = False
    for line in description.header_lines:
        if line.startswith("element vertex "):
            lines.append(f"element vertex {output_count}")
            replaced_vertex = True
            continue
        if line == "end_header":
            lines.extend(
                (
                    f"comment Invisible Places algorithm {ALGORITHM_ID}",
                    f"comment Invisible Places voxel_size_m {voxel_size_m:.9f}",
                    f"comment Invisible Places seed {seed:016x}",
                    f"comment Invisible Places rgb_filter {rgb_filter}",
                )
            )
        lines.append(line)
    if not replaced_vertex:
        raise ValueError(f"{description.path}: vertex element disappeared")
    return ("\n".join(lines) + "\n").encode("ascii")


def _mix64(value: np.ndarray | np.uint64 | int) -> np.ndarray:
    """Vectorised SplitMix64 finalizer with intentional uint64 wraparound."""

    result = np.asarray(value, dtype=np.uint64)
    with np.errstate(over="ignore"):
        result = (result ^ (result >> np.uint64(30))) * np.uint64(
            0xBF58476D1CE4E5B9
        )
        result = (result ^ (result >> np.uint64(27))) * np.uint64(
            0x94D049BB133111EB
        )
    return result ^ (result >> np.uint64(31))


def _cell_coordinates(
    records: np.ndarray,
    voxel_size_m: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    # A fixed half-cell offset prevents boundaries from coinciding with common
    # whole-world-unit coordinates and remains identical for every role.
    grid_offset = voxel_size_m * 0.5
    coordinates: list[np.ndarray] = []
    for name in ("x", "y", "z"):
        values = records[name].astype(np.float64, copy=False)
        if not np.all(np.isfinite(values)):
            raise ValueError(f"non-finite {name} coordinate in source PLY")
        coordinates.append(
            np.floor((values - grid_offset) / voxel_size_m).astype(np.int64)
        )
    return coordinates[0], coordinates[1], coordinates[2]


def _cell_priority(
    cell_x: np.ndarray,
    cell_y: np.ndarray,
    cell_z: np.ndarray,
    seed: int,
) -> np.ndarray:
    ux = cell_x.astype(np.uint64, copy=False)
    uy = cell_y.astype(np.uint64, copy=False)
    uz = cell_z.astype(np.uint64, copy=False)
    value = _mix64(ux ^ np.uint64(seed))
    value ^= _mix64(uy ^ np.uint64(seed ^ 0x9E3779B97F4A7C15))
    value ^= _mix64(uz ^ np.uint64(seed ^ 0xD1B54A32D192ED03))
    return _mix64(value)


def _point_priority(records: np.ndarray, seed: int) -> np.ndarray:
    """Hash real positions (and RGB when present) to order parents stably."""

    # Micrometre quantisation is much finer than these canonical 1 mm sources
    # while staying independent of the source's float32/float64 declaration.
    x = np.rint(records["x"].astype(np.float64) * 1_000_000.0).astype(np.int64)
    y = np.rint(records["y"].astype(np.float64) * 1_000_000.0).astype(np.int64)
    z = np.rint(records["z"].astype(np.float64) * 1_000_000.0).astype(np.int64)
    result = _cell_priority(x, y, z, seed ^ 0xA0761D6478BD642F)
    names = records.dtype.names or ()
    if all(name in names for name in ("red", "green", "blue")):
        packed_rgb = (
            records["red"].astype(np.uint64)
            | (records["green"].astype(np.uint64) << np.uint64(8))
            | (records["blue"].astype(np.uint64) << np.uint64(16))
        )
        result ^= _mix64(packed_rgb ^ np.uint64(seed))
    return _mix64(result)


def _group_starts_from_cells(
    cell_x: np.ndarray,
    cell_y: np.ndarray,
    cell_z: np.ndarray,
) -> np.ndarray:
    if cell_x.size == 0:
        return np.empty(0, dtype=np.int64)
    first = np.empty(cell_x.size, dtype=bool)
    first[0] = True
    first[1:] = (
        (cell_x[1:] != cell_x[:-1])
        | (cell_y[1:] != cell_y[:-1])
        | (cell_z[1:] != cell_z[:-1])
    )
    return np.flatnonzero(first).astype(np.int64, copy=False)


def _counts_from_starts(starts: np.ndarray, total: int) -> np.ndarray:
    return np.diff(
        np.concatenate((starts, np.asarray([total], dtype=np.int64)))
    ).astype(np.uint64, copy=False)


def _write_shards(
    description: PlyDescription,
    work_root: Path,
    config: BuildConfig,
    role_seed: int,
) -> tuple[list[Path], SourceProof]:
    work_root.mkdir(parents=True, exist_ok=False)
    shard_paths = [work_root / f"shard-{index:04d}.bin" for index in range(config.shard_count)]
    streams: list[BinaryIO] = [path.open("wb") for path in shard_paths]
    source_path = description.path.resolve(strict=True)
    before = source_path.stat()
    expected_size = description.data_offset + description.vertex_count * description.dtype.itemsize
    if before.st_size != expected_size:
        for stream in streams:
            stream.close()
        raise ValueError(
            f"{source_path}: size {before.st_size} does not equal header + "
            f"{description.vertex_count} fixed-width records ({expected_size})"
        )

    digest = hashlib.sha256()
    digest.update(description.raw_header)
    completed = 0
    try:
        with source_path.open("rb") as source:
            source.seek(description.data_offset)
            while completed < description.vertex_count:
                take = min(config.chunk_records, description.vertex_count - completed)
                raw = source.read(take * description.dtype.itemsize)
                if len(raw) != take * description.dtype.itemsize:
                    raise ValueError(f"{source_path}: truncated vertex data")
                digest.update(raw)
                records = np.frombuffer(raw, dtype=description.dtype, count=take)
                cell_x, cell_y, cell_z = _cell_coordinates(
                    records, config.voxel_size_m
                )
                priorities = _cell_priority(cell_x, cell_y, cell_z, role_seed)
                shard_ids = np.remainder(priorities, config.shard_count).astype(
                    np.int64, copy=False
                )
                order = np.argsort(shard_ids, kind="stable")
                sorted_ids = shard_ids[order]
                boundaries = np.flatnonzero(sorted_ids[1:] != sorted_ids[:-1]) + 1
                run_starts = np.concatenate((np.asarray([0]), boundaries))
                run_ends = np.concatenate((boundaries, np.asarray([take])))
                for run_start, run_end in zip(run_starts, run_ends, strict=True):
                    shard_id = int(sorted_ids[run_start])
                    streams[shard_id].write(records[order[run_start:run_end]].tobytes())
                completed += take
                print(
                    f"[{description.path.name}] sharded {completed:,}/"
                    f"{description.vertex_count:,}",
                    flush=True,
                )
            if source.read(1):
                raise ValueError(f"{source_path}: trailing data after vertex records")
    finally:
        for stream in streams:
            stream.flush()
            os.fsync(stream.fileno())
            stream.close()

    after = source_path.stat()
    if (before.st_size, before.st_mtime_ns) != (after.st_size, after.st_mtime_ns):
        raise RuntimeError(
            f"{source_path}: canonical source changed while it was read; cache not published"
        )
    return shard_paths, SourceProof(
        path=str(source_path),
        size_bytes=before.st_size,
        mtime_ns=before.st_mtime_ns,
        sha256=digest.hexdigest(),
        vertex_count=description.vertex_count,
        schema_sha256=description.schema_sha256,
    )


def _sort_shards_and_collect_cells(
    shard_paths: Sequence[Path],
    description: PlyDescription,
    config: BuildConfig,
    role_seed: int,
) -> tuple[np.ndarray, np.ndarray, list[int]]:
    count_parts: list[np.ndarray] = []
    priority_parts: list[np.ndarray] = []
    cell_part_sizes: list[int] = []
    for shard_index, path in enumerate(shard_paths):
        byte_count = path.stat().st_size
        if byte_count % description.dtype.itemsize != 0:
            raise RuntimeError(f"{path}: incomplete temporary record")
        record_count = byte_count // description.dtype.itemsize
        if record_count == 0:
            cell_part_sizes.append(0)
            continue
        records = np.memmap(path, dtype=description.dtype, mode="r", shape=(record_count,))
        cell_x, cell_y, cell_z = _cell_coordinates(records, config.voxel_size_m)
        point_priority = _point_priority(records, role_seed)
        # lexsort's last key is primary: cell XYZ, then stable point identity.
        order = np.lexsort((point_priority, cell_z, cell_y, cell_x))
        sorted_path = path.with_suffix(".sorted")
        with sorted_path.open("wb") as output:
            for start in range(0, record_count, config.chunk_records):
                end = min(record_count, start + config.chunk_records)
                output.write(records[order[start:end]].tobytes())
            output.flush()
            os.fsync(output.fileno())
        del records
        os.replace(sorted_path, path)

        sorted_records = np.memmap(
            path, dtype=description.dtype, mode="r", shape=(record_count,)
        )
        cell_x, cell_y, cell_z = _cell_coordinates(
            sorted_records, config.voxel_size_m
        )
        starts = _group_starts_from_cells(cell_x, cell_y, cell_z)
        counts = _counts_from_starts(starts, record_count)
        priorities = _cell_priority(
            cell_x[starts], cell_y[starts], cell_z[starts], role_seed
        )
        count_parts.append(np.asarray(counts))
        priority_parts.append(np.asarray(priorities))
        cell_part_sizes.append(int(starts.size))
        del sorted_records
        print(
            f"[{description.path.name}] prepared shard {shard_index + 1}/"
            f"{len(shard_paths)} ({record_count:,} parents, {starts.size:,} cells)",
            flush=True,
        )

    if not count_parts:
        raise RuntimeError(f"{description.path}: no populated spatial cells")
    return (
        np.concatenate(count_parts),
        np.concatenate(priority_parts),
        cell_part_sizes,
    )


def _make_systematic_extras(
    cell_counts: np.ndarray,
    cell_priorities: np.ndarray,
    source_count: int,
    target_count: int,
    seed: int,
) -> tuple[np.ndarray, int]:
    """Round fractional quotas exactly without an occupancy-class cutoff.

    A seed-derived phase advances through fractional remainders in stable
    cell-hash order.  An extra output is granted whenever the accumulated
    integer remainder wraps past ``source_count``.  Across possible phases a
    cell's extra probability is exactly its fractional quota; the committed
    phase remains byte-for-byte deterministic.
    """

    products = cell_counts.astype(np.uint64) * np.uint64(target_count)
    remainders = products % np.uint64(source_count)
    remainder_total = int(np.sum(remainders, dtype=np.uint64))
    if remainder_total % source_count != 0:
        raise RuntimeError("fractional quota remainders do not sum to an integer")
    extras_required = remainder_total // source_count
    order = np.argsort(cell_priorities, kind="stable")
    ordered_remainders = remainders[order]
    phase_value = _mix64(np.uint64(seed ^ 0xE7037ED1A0B428DB))
    phase = np.uint64(int(phase_value.item()) % source_count)
    cumulative = np.cumsum(ordered_remainders, dtype=np.uint64)
    previous = cumulative - ordered_remainders
    ordered_extras = (
        (phase + cumulative) // np.uint64(source_count)
        > (phase + previous) // np.uint64(source_count)
    )
    if int(np.count_nonzero(ordered_extras)) != extras_required:
        raise RuntimeError("systematic quota rounding did not produce the exact total")
    extras = np.zeros(cell_counts.size, dtype=bool)
    extras[order] = ordered_extras
    return extras, extras_required


def _finite_group_mean(values: np.ndarray, starts: np.ndarray) -> np.ndarray:
    as_float = values.astype(np.float64)
    finite = np.isfinite(as_float)
    sums = np.add.reduceat(np.where(finite, as_float, 0.0), starts)
    counts = np.add.reduceat(finite.astype(np.uint64), starts)
    with np.errstate(invalid="ignore", divide="ignore"):
        means = sums / counts
    empty = counts == 0
    if np.any(empty):
        means[empty] = as_float[starts[empty]]
    return means


def _categorical_mode(
    values: np.ndarray,
    starts: np.ndarray,
) -> np.ndarray:
    counts = _counts_from_starts(starts, values.size).astype(np.int64)
    group_ids = np.repeat(np.arange(starts.size, dtype=np.int64), counts)
    order = np.lexsort((values, group_ids))
    sorted_groups = group_ids[order]
    sorted_values = values[order]
    new_run = np.empty(values.size, dtype=bool)
    new_run[0] = True
    if np.issubdtype(values.dtype, np.floating):
        value_changed = ~(
            (sorted_values[1:] == sorted_values[:-1])
            | (np.isnan(sorted_values[1:]) & np.isnan(sorted_values[:-1]))
        )
    else:
        value_changed = sorted_values[1:] != sorted_values[:-1]
    new_run[1:] = (
        (sorted_groups[1:] != sorted_groups[:-1]) | value_changed
    )
    run_starts = np.flatnonzero(new_run)
    run_counts = _counts_from_starts(run_starts, values.size).astype(np.int64)
    run_groups = sorted_groups[run_starts]
    run_values = sorted_values[run_starts]
    maximum_counts = np.zeros(starts.size, dtype=np.int64)
    np.maximum.at(maximum_counts, run_groups, run_counts)
    candidates = run_counts == maximum_counts[run_groups]
    # Runs are ordered by group then value, so the first maximum is the
    # deterministic lowest-value tie break.
    candidate_groups = run_groups[candidates]
    candidate_values = run_values[candidates]
    first_for_group = np.empty(candidate_groups.size, dtype=bool)
    first_for_group[0] = True
    first_for_group[1:] = candidate_groups[1:] != candidate_groups[:-1]
    result = np.empty(starts.size, dtype=values.dtype)
    result[candidate_groups[first_for_group]] = candidate_values[first_for_group]
    return result


def _srgb_to_linear(values: np.ndarray) -> np.ndarray:
    return np.where(
        values <= 0.04045,
        values / 12.92,
        ((values + 0.055) / 1.055) ** 2.4,
    )


def _linear_to_srgb(values: np.ndarray) -> np.ndarray:
    return np.where(
        values <= 0.0031308,
        values * 12.92,
        1.055 * np.maximum(values, 0.0) ** (1.0 / 2.4) - 0.055,
    )


def _aggregate_sorted_groups(
    records: np.ndarray,
    starts: np.ndarray,
    rgb_filter: str,
) -> np.ndarray:
    output = np.empty(starts.size, dtype=records.dtype)
    names = records.dtype.names or ()
    handled: set[str] = set()

    # A sampled child position is always a real canonical parent position.
    for name in ("x", "y", "z"):
        output[name] = records[name][starts]
        handled.add(name)

    rgb_names = ("red", "green", "blue")
    if all(name in names for name in rgb_names):
        for name in rgb_names:
            values = records[name].astype(np.float64) / 255.0
            if rgb_filter == RGB_FILTER_SRGB_LINEAR:
                values = _srgb_to_linear(values)
            means = _finite_group_mean(values, starts)
            if rgb_filter == RGB_FILTER_SRGB_LINEAR:
                means = _linear_to_srgb(means)
            output[name] = np.rint(np.clip(means, 0.0, 1.0) * 255.0).astype(
                records.dtype[name]
            )
            handled.add(name)

    normal_names = ("nx", "ny", "nz")
    if all(name in names for name in normal_names):
        normal = np.column_stack(
            [records[name].astype(np.float64) for name in normal_names]
        )
        normal_length = np.linalg.norm(normal, axis=1)
        valid = np.all(np.isfinite(normal), axis=1) & (normal_length > 1.0e-12)
        unit_normal = np.zeros_like(normal)
        unit_normal[valid] = normal[valid] / normal_length[valid, None]
        all_indices = np.arange(records.size, dtype=np.int64)
        first_valid = np.minimum.reduceat(
            np.where(valid, all_indices, records.size), starts
        )
        has_valid = first_valid < records.size
        safe_first = np.where(has_valid, first_valid, starts)
        reference = unit_normal[safe_first]
        group_counts = _counts_from_starts(starts, records.size).astype(np.int64)
        reference_per_record = np.repeat(reference, group_counts, axis=0)
        dot = np.sum(unit_normal * reference_per_record, axis=1)
        # Opposite normals are treated as a convention mismatch and flipped;
        # normals more than 60 degrees from the representative are excluded so
        # a 5 mm cell cannot smooth across a sharp ledge onto another face.
        compatible = valid & (np.abs(dot) >= 0.5)
        aligned = np.where((dot < 0.0)[:, None], -unit_normal, unit_normal)
        aligned[~compatible] = 0.0
        summed = np.column_stack(
            [np.add.reduceat(aligned[:, axis], starts) for axis in range(3)]
        )
        length = np.linalg.norm(summed, axis=1)
        good = has_valid & (length > 1.0e-12)
        normalized = np.zeros_like(summed)
        normalized[good] = summed[good] / length[good, None]
        fallback_length = np.linalg.norm(reference, axis=1)
        fallback = fallback_length > 1.0e-12
        bad_with_fallback = ~good & fallback
        normalized[bad_with_fallback] = (
            reference[bad_with_fallback]
            / fallback_length[bad_with_fallback, None]
        )
        for axis, name in enumerate(normal_names):
            output[name] = normalized[:, axis].astype(records.dtype[name])
            handled.add(name)

    for name in names:
        if name in handled:
            continue
        normalized_name = name.lower().replace("_", "")
        if normalized_name in ("scanid", "scalarscanid"):
            output[name] = _categorical_mode(records[name], starts)
            continue
        means = _finite_group_mean(records[name], starts)
        field_dtype = records.dtype[name]
        if np.issubdtype(field_dtype, np.integer):
            limits = np.iinfo(field_dtype)
            means = np.rint(np.clip(means, limits.min, limits.max))
        output[name] = means.astype(field_dtype)
    return output


def _aggregate_shard(
    path: Path,
    description: PlyDescription,
    config: BuildConfig,
    target_count: int,
    systematic_extras: np.ndarray,
) -> np.ndarray:
    record_count = path.stat().st_size // description.dtype.itemsize
    if record_count == 0:
        return np.empty(0, dtype=description.dtype)
    records = np.memmap(path, dtype=description.dtype, mode="r", shape=(record_count,))
    cell_x, cell_y, cell_z = _cell_coordinates(records, config.voxel_size_m)
    cell_starts = _group_starts_from_cells(cell_x, cell_y, cell_z)
    cell_counts = _counts_from_starts(cell_starts, record_count)
    if systematic_extras.size != cell_starts.size:
        raise RuntimeError(f"{path}: systematic quota slice has the wrong size")
    products = cell_counts.astype(np.uint64) * np.uint64(target_count)
    quotas = (products // np.uint64(description.vertex_count)).astype(np.int64)
    quotas += systematic_extras.astype(np.int64)
    if np.any(quotas < 0) or np.any(quotas > cell_counts.astype(np.int64)):
        raise RuntimeError("cell quota exceeded its parent population")
    counts_i64 = cell_counts.astype(np.int64)
    cell_for_record = np.repeat(
        np.arange(cell_starts.size, dtype=np.int64), counts_i64
    )
    keep = quotas[cell_for_record] > 0
    if not np.any(keep):
        return np.empty(0, dtype=description.dtype)
    within_cell_rank = np.arange(record_count, dtype=np.int64) - cell_starts[
        cell_for_record
    ]
    lane = np.zeros(record_count, dtype=np.int64)
    lane[keep] = (
        within_cell_rank[keep] * quotas[cell_for_record[keep]]
    ) // counts_i64[cell_for_record[keep]]
    kept_cells = cell_for_record[keep]
    kept_lanes = lane[keep]
    new_group = np.empty(kept_cells.size, dtype=bool)
    new_group[0] = True
    new_group[1:] = (
        (kept_cells[1:] != kept_cells[:-1])
        | (kept_lanes[1:] != kept_lanes[:-1])
    )
    output_starts = np.flatnonzero(new_group).astype(np.int64, copy=False)
    kept_records = np.asarray(records[keep])
    output = _aggregate_sorted_groups(
        kept_records,
        output_starts,
        config.rgb_filter,
    )
    expected = int(np.sum(quotas, dtype=np.int64))
    if output.size != expected:
        raise RuntimeError(
            f"{path}: aggregated {output.size} strata, expected {expected}"
        )
    return output


def _validate_output(
    path: Path,
    expected_count: int,
    expected_schema_sha256: str,
    streamed_sha256: str,
) -> tuple[PlyDescription, os.stat_result, str]:
    description = read_ply_description(path)
    if description.vertex_count != expected_count:
        raise RuntimeError(
            f"{path}: output count {description.vertex_count} != {expected_count}"
        )
    if description.schema_sha256 != expected_schema_sha256:
        raise RuntimeError(f"{path}: output property schema changed")
    stat = path.stat()
    expected_size = description.data_offset + expected_count * description.dtype.itemsize
    if stat.st_size != expected_size:
        raise RuntimeError(
            f"{path}: output size {stat.st_size} != expected {expected_size}"
        )
    independent_sha256 = _sha256_file(path)
    if independent_sha256 != streamed_sha256:
        raise RuntimeError(f"{path}: streamed and independent SHA-256 differ")
    return description, stat, independent_sha256


def _build_role(
    role: str,
    source_path: Path,
    output_path: Path,
    work_root: Path,
    config: BuildConfig,
) -> dict[str, object]:
    description = read_ply_description(source_path)
    target_count = config.targets[role]
    if target_count <= 0 or target_count > description.vertex_count:
        raise ValueError(
            f"{role}: target {target_count} must be in 1..{description.vertex_count}"
        )
    role_seed = config.seed ^ int.from_bytes(role.encode("ascii"), "little")
    shard_paths, source_proof = _write_shards(
        description, work_root, config, role_seed
    )
    cell_counts, cell_priorities, cell_part_sizes = _sort_shards_and_collect_cells(
        shard_paths, description, config, role_seed
    )
    if int(np.sum(cell_counts, dtype=np.uint64)) != description.vertex_count:
        raise RuntimeError(f"{role}: sharded cell populations lost source parents")
    base_total = int(
        np.sum(
            (cell_counts.astype(np.uint64) * np.uint64(target_count))
            // np.uint64(description.vertex_count),
            dtype=np.uint64,
        )
    )
    systematic_extras, extras_required = _make_systematic_extras(
        cell_counts,
        cell_priorities,
        description.vertex_count,
        target_count,
        role_seed,
    )
    del cell_counts, cell_priorities
    print(
        f"[{source_path.name}] apportionment base={base_total:,}, "
        f"extras={extras_required:,}, target={target_count:,}",
        flush=True,
    )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    header = _output_header(
        description,
        target_count,
        config.voxel_size_m,
        config.seed,
        config.rgb_filter,
    )
    digest = hashlib.sha256()
    digest.update(header)
    emitted = 0
    cell_offset = 0
    with output_path.open("xb") as output:
        output.write(header)
        for shard_index, shard_path in enumerate(shard_paths):
            cell_count = cell_part_sizes[shard_index]
            aggregated = _aggregate_shard(
                shard_path,
                description,
                config,
                target_count,
                systematic_extras[cell_offset : cell_offset + cell_count],
            )
            cell_offset += cell_count
            raw = aggregated.tobytes()
            output.write(raw)
            digest.update(raw)
            emitted += aggregated.size
            print(
                f"[{source_path.name}] aggregated shard {shard_index + 1}/"
                f"{len(shard_paths)}; emitted {emitted:,}/{target_count:,}",
                flush=True,
            )
        output.flush()
        os.fsync(output.fileno())
    if cell_offset != systematic_extras.size:
        raise RuntimeError(f"{role}: did not consume every systematic quota")
    if emitted != target_count:
        raise RuntimeError(f"{role}: emitted {emitted}, expected {target_count}")

    output_description, output_stat, output_sha256 = _validate_output(
        output_path,
        target_count,
        description.schema_sha256,
        digest.hexdigest(),
    )
    shutil.rmtree(work_root)
    return {
        "role": role,
        "source": dataclasses.asdict(source_proof),
        "requested_point_count": target_count,
        "output": {
            "file": output_path.as_posix(),  # made bundle-relative below
            "size_bytes": output_stat.st_size,
            "mtime_ns": output_stat.st_mtime_ns,
            "sha256": output_sha256,
            "vertex_count": output_description.vertex_count,
            "schema_sha256": output_description.schema_sha256,
        },
        "population": {
            "source_points": description.vertex_count,
            "output_points": target_count,
            "maximum_cell_rounding_error_points": 1,
        },
    }


def _fsync_directory(path: Path) -> None:
    try:
        descriptor = os.open(path, os.O_RDONLY)
    except OSError:
        return
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _write_json_atomic(path: Path, value: object) -> None:
    temporary = path.with_name(f".{path.name}.tmp-{uuid.uuid4().hex}")
    payload = json.dumps(value, indent=2, sort_keys=True) + "\n"
    with temporary.open("x", encoding="utf-8") as stream:
        stream.write(payload)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)
    _fsync_directory(path.parent)


def _bundle_fingerprint_payload(manifest: dict[str, object]) -> dict[str, object]:
    roles = manifest["roles"]
    assert isinstance(roles, list)
    return {
        "schema_version": manifest["schema_version"],
        "algorithm": manifest["algorithm"],
        "scene": manifest["scene"],
        "roles": [
            {
                "role": role["role"],
                "requested_point_count": role["requested_point_count"],
                "source_sha256": role["source"]["sha256"],
                "source_schema_sha256": role["source"]["schema_sha256"],
                "output_sha256": role["output"]["sha256"],
                "output_schema_sha256": role["output"]["schema_sha256"],
            }
            for role in roles
        ],
    }


def _path_is_within(path: Path, directory: Path) -> bool:
    return path == directory or path.is_relative_to(directory)


def _preflight_build(config: BuildConfig) -> tuple[Path, Path]:
    source_root = config.source_root.resolve(strict=True)
    cache_root = config.cache_root.resolve(strict=False)
    if _path_is_within(cache_root, source_root) or _path_is_within(
        source_root, cache_root
    ):
        raise ValueError(
            "cache root and canonical source root must be disjoint: "
            f"{cache_root} vs {source_root}"
        )

    descriptions: dict[str, PlyDescription] = {}
    for role in ROLE_ORDER:
        source_path = source_root / f"Site3-{role}-1mm.ply"
        if not source_path.is_file():
            raise FileNotFoundError(f"missing canonical role source: {source_path}")
        canonical_source = source_path.resolve(strict=True)
        if _path_is_within(cache_root, canonical_source.parent) or _path_is_within(
            canonical_source, cache_root
        ):
            raise ValueError(
                "cache root may not contain or sit beside its canonical source "
                f"through a symlink alias: {cache_root} vs {canonical_source}"
            )
        description = read_ply_description(source_path)
        target = config.targets[role]
        if target <= 0 or target > description.vertex_count:
            raise ValueError(
                f"{role}: target {target} must be in 1..{description.vertex_count}"
            )
        descriptions[role] = description

    cache_root.mkdir(parents=True, exist_ok=True)
    source_sizes = [
        descriptions[role].path.resolve(strict=True).stat().st_size
        for role in ROLE_ORDER
    ]
    output_bytes = sum(
        config.targets[role] * descriptions[role].dtype.itemsize
        + len(
            _output_header(
                descriptions[role],
                config.targets[role],
                config.voxel_size_m,
                config.seed,
                config.rgb_filter,
            )
        )
        for role in ROLE_ORDER
    )
    largest_source = max(source_sizes)
    # Each role owns one full set of raw shards. Sorting briefly needs one
    # additional shard copy; allow two average-shard sizes for skew, all final
    # outputs, a 10% filesystem margin, and a fixed 2 GiB safety reserve.
    largest_shard_allowance = (
        2 * (largest_source + config.shard_count - 1) // config.shard_count
    )
    working_bytes = largest_source + largest_shard_allowance + output_bytes
    required_free = int(math.ceil(working_bytes * 1.10)) + FREE_SPACE_MARGIN_BYTES
    available = shutil.disk_usage(cache_root).free
    if available < required_free:
        raise RuntimeError(
            "insufficient free space for transactional density-cache build: "
            f"need at least {required_free:,} bytes, found {available:,} at "
            f"{cache_root}"
        )
    return source_root, cache_root


def _assert_sources_unchanged(role_records: Sequence[dict[str, object]]) -> None:
    for role_record in role_records:
        source = role_record["source"]
        assert isinstance(source, dict)
        path = Path(str(source["path"]))
        stat = path.stat()
        if (stat.st_size, stat.st_mtime_ns) != (
            int(source["size_bytes"]),
            int(source["mtime_ns"]),
        ):
            raise RuntimeError(
                f"{path}: canonical source changed after its role build; "
                "prior active cache remains unchanged"
            )


def _validated_relative_path(root: Path, relative_text: object) -> Path:
    relative = Path(str(relative_text))
    if relative.is_absolute() or ".." in relative.parts:
        raise RuntimeError(f"unsafe bundle-relative path: {relative}")
    candidate = root / relative
    if candidate.is_symlink() or not candidate.is_file():
        raise RuntimeError(f"bundle output is not a regular local file: {relative}")
    resolved = candidate.resolve(strict=True)
    if not _path_is_within(resolved, root.resolve(strict=True)):
        raise RuntimeError(f"bundle output escapes fingerprint directory: {relative}")
    return resolved


def _validate_existing_bundle(
    final_root: Path,
    expected_manifest: dict[str, object],
    fingerprint: str,
) -> Path:
    manifest_path = final_root / MANIFEST_NAME
    if (
        final_root.is_symlink()
        or not final_root.is_dir()
        or manifest_path.is_symlink()
        or not manifest_path.is_file()
    ):
        raise RuntimeError(f"existing fingerprint directory is incomplete: {final_root}")
    try:
        existing = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"existing bundle manifest is unreadable: {manifest_path}") from exc
    if not isinstance(existing, dict) or existing.get("bundle_fingerprint") != fingerprint:
        raise RuntimeError(f"existing bundle fingerprint mismatch: {final_root}")
    if existing.get("complete") is not True:
        raise RuntimeError(f"existing bundle is not marked complete: {final_root}")
    recomputed = _sha256_bytes(
        _canonical_json(_bundle_fingerprint_payload(existing))
    )
    if recomputed != fingerprint:
        raise RuntimeError(f"existing manifest identity does not match its directory: {final_root}")
    if _bundle_fingerprint_payload(existing) != _bundle_fingerprint_payload(
        expected_manifest
    ):
        raise RuntimeError(f"existing bundle does not match the newly built proof: {final_root}")

    existing_roles = existing.get("roles")
    expected_roles = expected_manifest.get("roles")
    if not isinstance(existing_roles, list) or not isinstance(expected_roles, list):
        raise RuntimeError(f"existing bundle roles are malformed: {final_root}")
    if [role.get("role") for role in existing_roles] != list(ROLE_ORDER):
        raise RuntimeError(f"existing bundle roles are incomplete or out of order: {final_root}")
    for existing_role, expected_role in zip(existing_roles, expected_roles, strict=True):
        if existing_role.get("source") != expected_role.get("source"):
            raise RuntimeError(
                f"existing {existing_role.get('role')} source proof differs from current build"
            )
        output = existing_role.get("output")
        expected_output = expected_role.get("output")
        if not isinstance(output, dict) or not isinstance(expected_output, dict):
            raise RuntimeError("existing output proof is malformed")
        if output.get("file") != expected_output.get("file"):
            raise RuntimeError("existing output path differs from current build")
        output_path = _validated_relative_path(final_root, output["file"])
        stat = output_path.stat()
        if (stat.st_size, stat.st_mtime_ns) != (
            int(output.get("size_bytes", -1)),
            int(output.get("mtime_ns", -1)),
        ):
            raise RuntimeError(f"existing cached output stat changed: {output_path}")
        description = read_ply_description(output_path)
        if description.vertex_count != int(output.get("vertex_count", -1)):
            raise RuntimeError(f"existing cached output count changed: {output_path}")
        if description.schema_sha256 != output.get("schema_sha256"):
            raise RuntimeError(f"existing cached output schema changed: {output_path}")
        if _sha256_file(output_path) != output.get("sha256"):
            raise RuntimeError(f"existing cached output hash changed: {output_path}")
        if output.get("sha256") != expected_output.get("sha256"):
            raise RuntimeError(
                "existing cached output differs from deterministic rebuild: "
                f"{output_path}"
            )
    return manifest_path


def build_cache(config: BuildConfig) -> Path:
    """Build and atomically activate a complete local three-role bundle."""

    if config.shard_count <= 0:
        raise ValueError("shard_count must be positive")
    if config.shard_count > 256:
        raise ValueError("shard_count must not exceed 256 open temporary files")
    if config.chunk_records <= 0:
        raise ValueError("chunk_records must be positive")
    if config.seed < 0 or config.seed > 0xFFFFFFFFFFFFFFFF:
        raise ValueError("seed must fit an unsigned 64-bit integer")
    if not math.isfinite(config.voxel_size_m) or config.voxel_size_m <= 0.0:
        raise ValueError("voxel_size_m must be finite and positive")
    if config.rgb_filter not in (
        RGB_FILTER_RENDERER_BYTE,
        RGB_FILTER_SRGB_LINEAR,
    ):
        raise ValueError(f"unsupported RGB filter {config.rgb_filter!r}")
    if set(config.targets) != set(ROLE_ORDER):
        raise ValueError(f"targets must contain exactly {', '.join(ROLE_ORDER)}")

    source_root, cache_root = _preflight_build(config)
    staging_root = cache_root / f".staging-{uuid.uuid4().hex}"
    staging_root.mkdir(exist_ok=False)
    try:
        role_records: list[dict[str, object]] = []
        for role in ROLE_ORDER:
            source_path = source_root / f"Site3-{role}-1mm.ply"
            if not source_path.is_file():
                raise FileNotFoundError(f"missing canonical role source: {source_path}")
            relative_output = Path("Scene3") / f"Site3-{role}-5mm.ply"
            record = _build_role(
                role,
                source_path,
                staging_root / relative_output,
                staging_root / ".work" / role,
                config,
            )
            output = record["output"]
            assert isinstance(output, dict)
            output["file"] = relative_output.as_posix()
            role_records.append(record)

        manifest: dict[str, object] = {
            "schema_version": SCHEMA_VERSION,
            "algorithm": {
                "id": ALGORITHM_ID,
                "version": ALGORITHM_VERSION,
                "seed_hex": f"{config.seed:016x}",
                "voxel_size_m": config.voxel_size_m,
                "rgb_filter": config.rgb_filter,
                "apportionment": "seeded-systematic-parent-population",
                "position_policy": "real-parent-stable-hash",
                "cell_grid_offset": "half-voxel-xyz",
                "normal_filter": (
                    "hemisphere-aligned-normalized-mean-cosine-gate-0.5"
                ),
                "categorical_filter": "scanid-mode-low-value-tie",
                "continuous_filter": "finite-arithmetic-mean",
            },
            "scene": "Scene3",
            "complete": True,
            "created_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
            "roles": role_records,
        }
        fingerprint = _sha256_bytes(
            _canonical_json(_bundle_fingerprint_payload(manifest))
        )
        manifest["bundle_fingerprint"] = fingerprint
        _write_json_atomic(staging_root / MANIFEST_NAME, manifest)
        _fsync_directory(staging_root)

        final_root = cache_root / fingerprint
        if final_root.exists():
            active_manifest_path = _validate_existing_bundle(
                final_root, manifest, fingerprint
            )
            shutil.rmtree(staging_root)
        else:
            os.replace(staging_root, final_root)
            _fsync_directory(cache_root)
            active_manifest_path = final_root / MANIFEST_NAME

        # This small pointer is the only activation mutation.  It occurs after
        # the finalized three-role directory and manifest are durable.
        _assert_sources_unchanged(role_records)
        manifest_sha256 = _sha256_file(active_manifest_path)
        _write_json_atomic(
            cache_root / ACTIVE_POINTER_NAME,
            {
                "schema_version": SCHEMA_VERSION,
                "bundle_fingerprint": fingerprint,
                "manifest_sha256": manifest_sha256,
            },
        )
        return final_root
    except Exception:
        if staging_root.exists() and not config.keep_staging_on_error:
            shutil.rmtree(staging_root)
        raise


def _parse_target(value: str) -> tuple[str, int]:
    try:
        role, count_text = value.split("=", 1)
        role = role.strip().upper()
        count = int(count_text)
    except (ValueError, TypeError) as exc:
        raise argparse.ArgumentTypeError("target must be ROLE=COUNT") from exc
    if role not in ROLE_ORDER or count <= 0:
        raise argparse.ArgumentTypeError(
            f"target role must be one of {', '.join(ROLE_ORDER)} and count positive"
        )
    return role, count


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--source-root",
        type=Path,
        default=Path("Data/Scene3"),
        help="directory containing the canonical Site3-ROLE-1mm.ply files",
    )
    parser.add_argument(
        "--cache-root",
        type=Path,
        default=Path("Saved/.invisible_places/cache/display_density/Scene3"),
        help="scene-specific local cache root",
    )
    parser.add_argument(
        "--target",
        action="append",
        type=_parse_target,
        metavar="ROLE=COUNT",
        help="override a role point budget (repeat for multiple roles)",
    )
    parser.add_argument("--voxel-size-mm", type=float, default=5.0)
    parser.add_argument("--shards", type=int, default=64)
    parser.add_argument("--chunk-records", type=int, default=500_000)
    parser.add_argument(
        "--seed",
        type=lambda text: int(text, 0),
        default=DEFAULT_SEED,
        help="stable integer seed (decimal or 0x-prefixed)",
    )
    parser.add_argument(
        "--rgb-filter",
        choices=(RGB_FILTER_RENDERER_BYTE, RGB_FILTER_SRGB_LINEAR),
        default=RGB_FILTER_RENDERER_BYTE,
        help="renderer-byte-mean matches the unchanged 1 mm renderer",
    )
    parser.add_argument("--keep-staging-on-error", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    arguments = _parser().parse_args(argv)
    targets = dict(DEFAULT_TARGETS)
    if arguments.target:
        targets.update(arguments.target)
    config = BuildConfig(
        source_root=arguments.source_root,
        cache_root=arguments.cache_root,
        targets=targets,
        voxel_size_m=arguments.voxel_size_mm / 1000.0,
        shard_count=arguments.shards,
        seed=arguments.seed,
        rgb_filter=arguments.rgb_filter,
        chunk_records=arguments.chunk_records,
        keep_staging_on_error=arguments.keep_staging_on_error,
    )
    bundle = build_cache(config)
    print(f"Activated local display-density bundle: {bundle}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
