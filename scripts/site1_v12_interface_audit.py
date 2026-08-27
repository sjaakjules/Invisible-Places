#!/usr/bin/env python3
"""Hash-locked post-build audit for the Scene1 v12 WATER/terrain interface.

The hand-drawn review marks are search neighbourhoods, never fill masks.  This
module evaluates the completed fine WATER candidate against the canonical 1 mm
SAND and ROCK clouds without changing any cloud.  It proves the append-only
geometry contract, measures moving-circle combined support, measures WATER
continuity only where terrain is absent, and samples terrain-edge distances in
a deterministic bounded fashion.  A self-hashed JSON manifest is the only
output.
"""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import math
import os
from pathlib import Path
import sys
from typing import Mapping, Sequence

import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import site1_v11_water_density as density  # noqa: E402
import site1_v11_water_scalar_enrichment as scalar_enrichment  # noqa: E402
import site1_v12_water_pipeline as water_pipeline  # noqa: E402


OPERATION = "site1-v12-post-build-terrain-water-interface-audit"
EVALUATION_RADIUS_M = 0.08
SUPPORT_RADIUS_M = 0.16
GRID_STEP_M = 0.08
EDGE_CONTINUITY_M = 0.04
EDGE_SEARCH_M = 0.12
EDGE_CENSOR_M = 0.25
EDGE_SAMPLE_LIMIT = 250_000
EDGE_UNRESOLVED_FRACTION_LIMIT = 0.10
FADE_EXTREME_FRACTION_LIMIT = 0.10
MEASURED_DENSITY_KINDS = frozenset(("interface", "hole", "dip"))


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise RuntimeError(message)


def _sha256(path: str | Path, block_size: int = 32 * 1024 * 1024) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        while block := handle.read(block_size):
            digest.update(block)
    return digest.hexdigest()


def _strict_file(path: str | Path, label: str) -> Path:
    lexical = Path(os.path.abspath(os.fspath(Path(path))))
    _require(not lexical.is_symlink(), f"{label} may not be a symlink: {lexical}")
    resolved = lexical.resolve(strict=True)
    _require(resolved == lexical, f"{label} traverses a path alias: {lexical}")
    _require(resolved.is_file(), f"{label} is not a regular file: {resolved}")
    return resolved


def _fingerprint(path: str | Path, *, ply: bool = False) -> dict:
    source = _strict_file(path, "audit input")
    before = source.stat()
    result = {
        "path": str(source),
        "size_bytes": int(before.st_size),
        "mtime_ns": int(before.st_mtime_ns),
        "sha256": _sha256(source),
    }
    if ply:
        info = density.inspect_fixed_stride_ply(source)
        result.update(
            points=int(info.count),
            record_stride=int(info.dtype.itemsize),
            schema=[[name, info.dtype.fields[name][0].str] for name in info.dtype.names or ()],
        )
    after = source.stat()
    _require(
        (before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns)
        == (after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns),
        f"audit input changed while hashing: {source}",
    )
    return result


def _assert_fingerprint(expected: Mapping, path: str | Path, label: str) -> dict:
    required = {"path", "size_bytes", "mtime_ns", "sha256"}
    if str(path).lower().endswith(".ply") or "points" in expected:
        required.update(("points", "record_stride", "schema"))
    missing = sorted(required - set(expected))
    _require(not missing, f"{label} fingerprint is incomplete: {', '.join(missing)}")
    _require(set(expected) == required, f"{label} fingerprint key set differs")
    actual = _fingerprint(path, ply="points" in expected)
    _require(str(expected.get("path", "")) == actual["path"], f"{label} path drift")
    for key in ("size_bytes", "mtime_ns", "sha256", "points", "record_stride", "schema"):
        if key in expected:
            _require(expected.get(key) == actual.get(key), f"{label} {key} drift")
    return actual


def _load_json(path: str | Path, label: str) -> tuple[Path, dict]:
    source = _strict_file(path, label)
    try:
        value = json.loads(source.read_text(encoding="utf-8"))
    except json.JSONDecodeError as error:
        raise RuntimeError(f"{label} is invalid JSON: {source}") from error
    _require(isinstance(value, dict), f"{label} must contain an object")
    return source, value


def _canonical_document_hash(document: Mapping) -> str:
    unlocked = {key: value for key, value in document.items() if key != "manifest_lock"}
    payload = json.dumps(
        unlocked, sort_keys=True, separators=(",", ":"), allow_nan=False
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _atomic_json(path: str | Path, document: Mapping) -> Path:
    destination = Path(os.path.abspath(os.fspath(Path(path))))
    _require(destination.parent.exists(), f"audit output parent is missing: {destination.parent}")
    _require(not destination.exists() and not destination.is_symlink(), f"audit output exists: {destination}")
    temporary = destination.with_name(f".{destination.name}.{os.getpid()}.partial")
    _require(not temporary.exists() and not temporary.is_symlink(), f"stale audit temporary file: {temporary}")
    payload = json.dumps(document, indent=2, sort_keys=True, allow_nan=False) + "\n"
    with temporary.open("x", encoding="utf-8") as handle:
        handle.write(payload)
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(temporary, destination)
    descriptor = os.open(destination.parent, os.O_RDONLY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    return destination


def _payload_sha256(
    info: density.PlyInfo, *, start: int = 0, count: int | None = None
) -> str:
    active_count = info.count - start if count is None else int(count)
    _require(start >= 0 and active_count >= 0 and start + active_count <= info.count, "invalid PLY payload span")
    digest = hashlib.sha256()
    remaining = active_count * info.dtype.itemsize
    with info.path.open("rb") as handle:
        handle.seek(info.offset + start * info.dtype.itemsize)
        while remaining:
            block = handle.read(min(32 * 1024 * 1024, remaining))
            _require(bool(block), f"short PLY payload read: {info.path}")
            digest.update(block)
            remaining -= len(block)
    return digest.hexdigest()


def _append_contract(
    base_water_path: Path,
    final_water_path: Path,
    geometry_archive_path: Path,
    geometry_manifest: Mapping[str, object],
    *,
    chunk_records: int,
) -> tuple[dict, np.ndarray, np.ndarray]:
    base = density.inspect_fixed_stride_ply(base_water_path)
    final = density.inspect_fixed_stride_ply(final_water_path)
    _require(base.dtype == final.dtype, "final WATER PLY schema differs from base WATER")
    with np.load(geometry_archive_path, allow_pickle=False) as archive:
        _require("records" in archive.files, "geometry archive lacks records")
        _require("candidate_label" in archive.files, "geometry archive lacks candidate_label")
        records = np.asarray(archive["records"]).copy()
        labels = np.asarray(archive["candidate_label"], np.int32).copy()
    _require(records.dtype == final.dtype, "geometry archive schema differs from final WATER")
    _require(len(records) == len(labels), "geometry archive label count differs from records")
    prefix_proof = scalar_enrichment.verify_reversible_base_cull_prefix(
        base_water_path,
        final_water_path,
        geometry_manifest,
        chunk_records=chunk_records,
    )
    surviving_base_points = int(prefix_proof["surviving_base_points"])
    removed_base_points = int(prefix_proof["removed_base_count"])
    _require(
        final.count == surviving_base_points + len(records),
        "final WATER count is not surviving base plus archive",
    )
    base_payload = _payload_sha256(base)
    prefix_payload = _payload_sha256(final, count=surviving_base_points)
    _require(
        prefix_payload == prefix_proof["surviving_prefix_payload_sha256"],
        "final WATER surviving base prefix differs from the reversible cull proof",
    )
    memory = np.memmap(
        final.path,
        dtype=final.dtype,
        mode="r",
        offset=final.offset,
        shape=(final.count,),
    )
    try:
        suffix = memory[surviving_base_points:]
        for name in ("x", "y", "z"):
            _require(name in final.dtype.names, f"final WATER lacks {name}")
            _require(
                np.asarray(suffix[name]).tobytes() == np.asarray(records[name]).tobytes(),
                f"final WATER suffix {name} differs from geometry archive",
            )
    finally:
        del memory
    additions_xy = np.column_stack((records["x"], records["y"])).astype(np.float64)
    return (
        {
            "source_base_points": int(base.count),
            "removed_base_points": removed_base_points,
            "base_points": surviving_base_points,
            "addition_count": int(len(records)),
            "final_points": int(final.count),
            "base_payload_sha256": base_payload,
            "final_prefix_payload_sha256": prefix_payload,
            "base_payload_byte_exact": True,
            "surviving_base_payload_byte_exact": True,
            "surviving_base_row_order_preserved": True,
            "suffix_xyz_archive_exact": True,
            "component_labels_present": sorted(int(item) for item in np.unique(labels)),
        },
        additions_xy,
        labels,
    )


def _circle_union_mask(xy: np.ndarray, specs: Sequence[water_pipeline.CircleSpec]) -> np.ndarray:
    keep = np.zeros(len(xy), dtype=bool)
    for spec in specs:
        dx = xy[:, 0] - spec.center_xy[0]
        dy = xy[:, 1] - spec.center_xy[1]
        keep |= dx * dx + dy * dy <= spec.radius_m * spec.radius_m
    return keep


def _expanded_specs(
    specs: Sequence[water_pipeline.CircleSpec], margin_m: float
) -> tuple[water_pipeline.CircleSpec, ...]:
    return tuple(
        water_pipeline.CircleSpec(
            identifier=spec.identifier,
            center_xy=spec.center_xy,
            radius_m=spec.radius_m + margin_m,
            kind=spec.kind,
            oversample_pitch_m=spec.oversample_pitch_m,
            maximum_water_support_distance_m=spec.maximum_water_support_distance_m,
            priority=spec.priority,
            label=spec.label,
        )
        for spec in specs
    )


def _bbox(specs: Sequence[water_pipeline.CircleSpec]) -> tuple[float, float, float, float]:
    return (
        min(spec.center_xy[0] - spec.radius_m for spec in specs),
        max(spec.center_xy[0] + spec.radius_m for spec in specs),
        min(spec.center_xy[1] - spec.radius_m for spec in specs),
        max(spec.center_xy[1] + spec.radius_m for spec in specs),
    )


def _collect_water_xy(
    path: Path,
    specs: Sequence[water_pipeline.CircleSpec],
    *,
    chunk_records: int,
    excluded_source_indices: np.ndarray | None = None,
) -> np.ndarray:
    info = density.inspect_fixed_stride_ply(path)
    bounds = _bbox(specs)
    parts: list[np.ndarray] = []
    excluded = np.asarray(
        np.empty(0, np.int64)
        if excluded_source_indices is None
        else excluded_source_indices,
        np.int64,
    )
    _require(
        not len(excluded) or np.all(excluded[1:] > excluded[:-1]),
        "excluded source indices must be strictly increasing",
    )
    _require(
        not len(excluded)
        or (int(excluded[0]) >= 0 and int(excluded[-1]) < info.count),
        "excluded source indices lie outside WATER",
    )
    for begin, records in density.iter_ply_chunks(
        path, info=info, chunk_size=chunk_records
    ):
        x = records["x"].astype(np.float64, copy=False)
        y = records["y"].astype(np.float64, copy=False)
        pre = (x >= bounds[0]) & (x <= bounds[1]) & (y >= bounds[2]) & (y <= bounds[3])
        if not np.any(pre):
            continue
        local = np.flatnonzero(pre)
        if len(excluded):
            source_index = local.astype(np.int64) + int(begin)
            position = np.searchsorted(excluded, source_index, side="left")
            clipped = np.minimum(position, len(excluded) - 1)
            removed = (position < len(excluded)) & (
                excluded[clipped] == source_index
            )
            local = local[~removed]
            if not len(local):
                continue
        xy = np.column_stack((x[local], y[local]))
        inside = _circle_union_mask(xy, specs)
        if np.any(inside):
            parts.append(xy[inside].copy())
    return np.concatenate(parts) if parts else np.empty((0, 2), np.float64)


def _declared_removed_source_indices(
    geometry_manifest: Mapping[str, object], *, source_count: int
) -> np.ndarray:
    """Load the exact removed-row set already proven by `_append_contract`."""

    raw = geometry_manifest.get("far_lobe_cull")
    _require(isinstance(raw, Mapping), "geometry far-lobe decision is missing")
    if raw.get("performed") is not True:
        _require(
            int(raw.get("removed_count", -1)) == 0,
            "unperformed far-lobe decision declares removed rows",
        )
        return np.empty(0, np.int64)
    archive = raw.get("archive")
    _require(isinstance(archive, Mapping), "performed far-lobe cull lacks archive")
    block = archive.get("source_indices")
    _require(isinstance(block, Mapping), "far-lobe index fingerprint is missing")
    path = Path(str(block.get("path", ""))).resolve(strict=True)
    removed_count = int(raw.get("removed_count", -1))
    _require(removed_count > 0, "performed far-lobe cull removes no rows")
    indices = np.fromfile(path, dtype="<i8")
    _require(
        len(indices) == removed_count,
        "far-lobe index archive count differs from manifest",
    )
    _require(
        int(indices[0]) >= 0
        and int(indices[-1]) < source_count
        and np.all(indices[1:] > indices[:-1]),
        "far-lobe source indices are invalid",
    )
    return np.asarray(indices, np.int64)


def _grid_for_spec(
    spec: water_pipeline.CircleSpec, *, step_m: float = GRID_STEP_M
) -> tuple[np.ndarray, np.ndarray]:
    count = int(math.ceil(spec.radius_m / step_m))
    offsets = np.arange(-count, count + 1, dtype=np.int32)
    iy, ix = np.meshgrid(offsets, offsets, indexing="ij")
    dx = ix.ravel().astype(np.float64) * step_m
    dy = iy.ravel().astype(np.float64) * step_m
    keep = dx * dx + dy * dy <= spec.radius_m * spec.radius_m + 1e-12
    xy = np.column_stack((spec.center_xy[0] + dx[keep], spec.center_xy[1] + dy[keep]))
    keys = np.column_stack((ix.ravel()[keep], iy.ravel()[keep])).astype(np.int32)
    return xy, keys


def _moving_centres(specs: Sequence[water_pipeline.CircleSpec]) -> tuple[np.ndarray, list[slice], list[np.ndarray]]:
    parts: list[np.ndarray] = []
    slices: list[slice] = []
    keys: list[np.ndarray] = []
    begin = 0
    for spec in specs:
        xy, grid_keys = _grid_for_spec(spec)
        parts.append(xy)
        keys.append(grid_keys)
        slices.append(slice(begin, begin + len(xy)))
        begin += len(xy)
    return (np.concatenate(parts) if parts else np.empty((0, 2))), slices, keys


class _NumpySpatialIndex:
    """Small-fixture fallback when SciPy is unavailable.

    Production Scene1 runs use scipy.cKDTree.  Keeping this bounded NumPy
    implementation makes the audit and its contract tests independent of an
    optional wheel without silently changing any arithmetic.
    """

    def __init__(self, support: np.ndarray):
        self.support = np.asarray(support, np.float64)

    def query_ball_point(self, query, radius, *, return_length=True, workers=-1):
        _require(return_length is True, "NumPy spatial fallback only returns counts")
        points = np.asarray(query, np.float64)
        output = np.empty(len(points), np.int64)
        bytes_per_query = max(16 * len(self.support), 16)
        chunk = max(1, (32 * 1024 * 1024) // bytes_per_query)
        radius_squared = float(radius) ** 2
        for begin in range(0, len(points), chunk):
            delta = points[begin : begin + chunk, None, :] - self.support[None, :, :]
            output[begin : begin + chunk] = np.count_nonzero(
                np.sum(delta * delta, axis=2) <= radius_squared, axis=1
            )
        return output

    def query(self, query, *, k=1, workers=-1):
        _require(k == 1, "NumPy spatial fallback supports nearest-one queries only")
        points = np.asarray(query, np.float64)
        distance = np.empty(len(points), np.float64)
        index = np.empty(len(points), np.int64)
        bytes_per_query = max(16 * len(self.support), 16)
        chunk = max(1, (32 * 1024 * 1024) // bytes_per_query)
        for begin in range(0, len(points), chunk):
            delta = points[begin : begin + chunk, None, :] - self.support[None, :, :]
            squared = np.sum(delta * delta, axis=2)
            selected = np.argmin(squared, axis=1)
            index[begin : begin + chunk] = selected
            distance[begin : begin + chunk] = np.sqrt(
                squared[np.arange(len(selected)), selected]
            )
        return distance, index


def _spatial_index(points: np.ndarray):
    support = np.asarray(points, np.float64)
    _require(len(support) > 0, "spatial index support is empty")
    try:
        from scipy.spatial import cKDTree
    except ModuleNotFoundError:
        _require(
            len(support) <= 10_000,
            "SciPy cKDTree is required for a production-size interface audit; "
            "the NumPy fallback is limited to 10,000 support points",
        )
        return _NumpySpatialIndex(support)
    return cKDTree(support)


def _tree_counts(tree, centres: np.ndarray, radius_m: float) -> np.ndarray:
    if len(centres) == 0:
        return np.empty(0, np.int64)
    if tree is None:
        return np.zeros(len(centres), np.int64)
    return np.asarray(
        tree.query_ball_point(centres, radius_m, return_length=True, workers=-1),
        np.int64,
    )


def _quantiles(values: np.ndarray) -> dict:
    finite = np.asarray(values, np.float64)
    finite = finite[np.isfinite(finite)]
    if len(finite) == 0:
        return {"count": 0, "p50": None, "p90": None, "p99": None, "maximum": None}
    return {
        "count": int(len(finite)),
        "p50": float(np.quantile(finite, 0.50)),
        "p90": float(np.quantile(finite, 0.90)),
        "p99": float(np.quantile(finite, 0.99)),
        "maximum": float(np.max(finite)),
    }


def _count_summary(values: np.ndarray) -> dict:
    source = np.asarray(values)
    result = _quantiles(source.astype(np.float64, copy=False))
    total = np.sum(source)
    result["total"] = (
        int(total) if np.issubdtype(source.dtype, np.integer) else float(total)
    )
    return result


def _splitmix64(values: np.ndarray) -> np.ndarray:
    result = np.asarray(values, np.uint64)
    with np.errstate(over="ignore"):
        result = result + np.uint64(0x9E3779B97F4A7C15)
        result = (result ^ (result >> np.uint64(30))) * np.uint64(0xBF58476D1CE4E5B9)
        result = (result ^ (result >> np.uint64(27))) * np.uint64(0x94D049BB133111EB)
    return result ^ (result >> np.uint64(31))


class _BottomK:
    def __init__(self, limit: int):
        self.limit = int(limit)
        self.priority = np.empty(0, np.uint64)
        self.before = np.empty(0, np.float64)
        self.after = np.empty(0, np.float64)
        self.before_ratio = np.empty(0, np.float64)
        self.after_ratio = np.empty(0, np.float64)

    def add(
        self,
        priority: np.ndarray,
        before: np.ndarray,
        after: np.ndarray,
        before_ratio: np.ndarray,
        after_ratio: np.ndarray,
    ) -> None:
        if not len(priority):
            return
        all_priority = np.concatenate((self.priority, np.asarray(priority, np.uint64)))
        all_before = np.concatenate((self.before, np.asarray(before, np.float64)))
        all_after = np.concatenate((self.after, np.asarray(after, np.float64)))
        all_before_ratio = np.concatenate(
            (self.before_ratio, np.asarray(before_ratio, np.float64))
        )
        all_after_ratio = np.concatenate(
            (self.after_ratio, np.asarray(after_ratio, np.float64))
        )
        if len(all_priority) > self.limit:
            selected = np.argpartition(all_priority, self.limit - 1)[: self.limit]
            order = np.argsort(all_priority[selected], kind="stable")
            selected = selected[order]
            all_priority = all_priority[selected]
            all_before = all_before[selected]
            all_after = all_after[selected]
            all_before_ratio = all_before_ratio[selected]
            all_after_ratio = all_after_ratio[selected]
        self.priority = all_priority
        self.before = all_before
        self.after = all_after
        self.before_ratio = all_before_ratio
        self.after_ratio = all_after_ratio


def _stream_terrain_counts(
    terrain_paths: Sequence[Path],
    specs: Sequence[water_pipeline.CircleSpec],
    centres: np.ndarray,
    *,
    chunk_records: int,
) -> tuple[np.ndarray, np.ndarray]:
    expanded = _expanded_specs(specs, max(SUPPORT_RADIUS_M, EDGE_CENSOR_M))
    bounds = _bbox(expanded)
    inner_counts = np.zeros(len(centres), np.int64)
    outer_counts = np.zeros(len(centres), np.int64)
    for path in terrain_paths:
        info = density.inspect_fixed_stride_ply(path)
        for _, records in density.iter_ply_chunks(
            path, info=info, chunk_size=chunk_records
        ):
            x = records["x"].astype(np.float64, copy=False)
            y = records["y"].astype(np.float64, copy=False)
            pre = (x >= bounds[0]) & (x <= bounds[1]) & (y >= bounds[2]) & (y <= bounds[3])
            if not np.any(pre):
                continue
            local_index = np.flatnonzero(pre)
            xy = np.column_stack((x[local_index], y[local_index]))
            expanded_keep = _circle_union_mask(xy, expanded)
            if not np.any(expanded_keep):
                continue
            local_index = local_index[expanded_keep]
            xy = xy[expanded_keep]
            chunk_tree = _spatial_index(xy)
            inner_counts += _tree_counts(chunk_tree, centres, EVALUATION_RADIUS_M)
            outer_counts += _tree_counts(chunk_tree, centres, SUPPORT_RADIUS_M)
    return inner_counts, outer_counts


def _edge_seed_centres(
    contract: Mapping,
    terrain_inner: np.ndarray,
    terrain_outer: np.ndarray,
) -> dict[str, np.ndarray]:
    """Select immutable canonical-terrain boundary centres per review spec.

    The upstream geometry contract derives this mask only from canonical
    terrain: terrain-free inner support next to terrain outer support, plus
    occupied centres adjacent to those terrain-free centres.  Final WATER is
    deliberately absent, so an entirely missing interface remains visible.
    """

    spec_id = np.asarray(contract["centre_spec_id"], object)
    spec_kind = np.asarray(contract["centre_spec_kind"], object)
    required = np.asarray(contract["required_mask"], bool)
    target_kind = np.asarray(
        [str(value) in MEASURED_DENSITY_KINDS for value in spec_kind], bool
    )
    _require(
        np.array_equal(np.asarray(terrain_inner, np.int64), contract["terrain_count"]),
        "canonical terrain inner recount differs from edge contract",
    )
    _require(
        np.array_equal(
            np.asarray(terrain_outer, np.int64), contract["terrain_outer_count"]
        ),
        "canonical terrain outer recount differs from edge contract",
    )
    boundary = (
        required
        & target_kind
        & np.asarray(contract["terrain_boundary_mask"], bool)
    )
    centres = np.asarray(contract["centres"], np.float64)
    result: dict[str, np.ndarray] = {}
    for identifier in sorted(set(str(value) for value in spec_id[target_kind])):
        member = boundary & (spec_id == identifier)
        result[identifier] = centres[member]
    return result


def _stream_terrain_edges(
    terrain_paths: Sequence[Path],
    specs: Sequence[water_pipeline.CircleSpec],
    boundary_centres: Mapping[str, np.ndarray],
    base_tree,
    final_tree,
    *,
    chunk_records: int,
    edge_sample_limit: int,
) -> dict:
    """Measure WATER proximity from candidate-independent terrain evidence."""

    edge_specs = tuple(spec for spec in specs if spec.kind in MEASURED_DENSITY_KINDS)
    expanded = _expanded_specs(edge_specs, EDGE_SEARCH_M)
    bounds = _bbox(expanded)
    boundary_trees = {
        spec.identifier: (
            _spatial_index(np.asarray(boundary_centres[spec.identifier], np.float64))
            if len(boundary_centres.get(spec.identifier, ()))
            else None
        )
        for spec in edge_specs
    }
    per_spec_limit = max(1, int(math.ceil(edge_sample_limit / len(edge_specs))))
    per_spec = {
        spec.identifier: {
            "spec": spec,
            "eligible": 0,
            "baseline_adjacent": 0,
            "boundary_adjacent": 0,
            "unresolved_before": 0,
            "unresolved_after": 0,
            "sampler": _BottomK(per_spec_limit),
        }
        for spec in edge_specs
    }
    eligible_count = 0
    baseline_adjacent_count = 0
    boundary_adjacent_count = 0
    unresolved_before = 0
    unresolved_after = 0
    sampler = _BottomK(edge_sample_limit)
    for source_ordinal, path in enumerate(terrain_paths):
        info = density.inspect_fixed_stride_ply(path)
        for begin, records in density.iter_ply_chunks(
            path, info=info, chunk_size=chunk_records
        ):
            x = records["x"].astype(np.float64, copy=False)
            y = records["y"].astype(np.float64, copy=False)
            pre = (
                (x >= bounds[0])
                & (x <= bounds[1])
                & (y >= bounds[2])
                & (y <= bounds[3])
            )
            if not np.any(pre):
                continue
            local_index = np.flatnonzero(pre)
            xy = np.column_stack((x[local_index], y[local_index]))
            inner_keep = _circle_union_mask(xy, edge_specs)
            if not np.any(inner_keep):
                continue
            edge_xy = xy[inner_keep]
            edge_indices = local_index[inner_keep].astype(np.uint64) + np.uint64(begin)
            before_distance = (
                np.asarray(base_tree.query(edge_xy, k=1, workers=-1)[0], np.float64)
                if base_tree is not None
                else np.full(len(edge_xy), np.inf)
            )
            after_distance = (
                np.asarray(final_tree.query(edge_xy, k=1, workers=-1)[0], np.float64)
                if final_tree is not None
                else np.full(len(edge_xy), np.inf)
            )
            eligible_any = np.zeros(len(edge_xy), bool)
            baseline_any = np.zeros(len(edge_xy), bool)
            boundary_any = np.zeros(len(edge_xy), bool)
            support_threshold = np.full(len(edge_xy), np.inf, np.float64)
            salt = np.uint64(
                ((source_ordinal + 1) * 0xD1B54A32D192ED03)
                & 0xFFFFFFFFFFFFFFFF
            )
            priority = _splitmix64(edge_indices ^ salt)

            for spec in edge_specs:
                dx = edge_xy[:, 0] - spec.center_xy[0]
                dy = edge_xy[:, 1] - spec.center_xy[1]
                member = dx * dx + dy * dy <= spec.radius_m * spec.radius_m
                if not np.any(member):
                    continue
                threshold = float(spec.maximum_water_support_distance_m)
                baseline_adjacent = before_distance <= threshold
                boundary_tree = boundary_trees[spec.identifier]
                if boundary_tree is None:
                    boundary_adjacent = np.zeros(len(edge_xy), bool)
                else:
                    boundary_distance = np.asarray(
                        boundary_tree.query(edge_xy, k=1, workers=-1)[0], np.float64
                    )
                    boundary_adjacent = boundary_distance <= threshold
                eligible = member & (baseline_adjacent | boundary_adjacent)
                if not np.any(eligible):
                    continue
                eligible_any |= eligible
                boundary_any |= member & boundary_adjacent
                support_threshold[eligible] = np.minimum(
                    support_threshold[eligible],
                    float(spec.maximum_water_support_distance_m),
                )
                item = per_spec[spec.identifier]
                before = before_distance[eligible]
                after = after_distance[eligible]
                item["eligible"] += int(np.count_nonzero(eligible))
                item["baseline_adjacent"] += int(
                    np.count_nonzero(eligible & baseline_adjacent)
                )
                item["boundary_adjacent"] += int(
                    np.count_nonzero(eligible & boundary_adjacent)
                )
                baseline_any |= member & baseline_adjacent
                item["unresolved_before"] += int(np.count_nonzero(before > threshold))
                item["unresolved_after"] += int(np.count_nonzero(after > threshold))
                item["sampler"].add(
                    priority[eligible],
                    np.minimum(before, EDGE_CENSOR_M),
                    np.minimum(after, EDGE_CENSOR_M),
                    np.minimum(before, EDGE_CENSOR_M) / threshold,
                    np.minimum(after, EDGE_CENSOR_M) / threshold,
                )

            if not np.any(eligible_any):
                continue
            before = before_distance[eligible_any]
            after = after_distance[eligible_any]
            threshold = support_threshold[eligible_any]
            _require(np.all(np.isfinite(threshold) & (threshold > 0.0)), "edge support threshold is invalid")
            eligible_count += int(np.count_nonzero(eligible_any))
            baseline_adjacent_count += int(
                np.count_nonzero(eligible_any & baseline_any)
            )
            boundary_adjacent_count += int(
                np.count_nonzero(eligible_any & boundary_any)
            )
            unresolved_before += int(np.count_nonzero(before > threshold))
            unresolved_after += int(np.count_nonzero(after > threshold))
            sampler.add(
                priority[eligible_any],
                np.minimum(before, EDGE_CENSOR_M),
                np.minimum(after, EDGE_CENSOR_M),
                np.minimum(before, EDGE_CENSOR_M) / threshold,
                np.minimum(after, EDGE_CENSOR_M) / threshold,
            )

    def summarize(item: Mapping) -> dict:
        active_sampler = item["sampler"]
        eligible = int(item["eligible"])
        unresolved_after_count = int(item["unresolved_after"])
        return {
            "kind": item["spec"].kind,
            "configured_support_m": float(
                item["spec"].maximum_water_support_distance_m
            ),
            "candidate_independent_boundary_centres": int(
                len(boundary_centres.get(item["spec"].identifier, ()))
            ),
            "eligible_edge_points": eligible,
            "baseline_adjacent_points": int(item["baseline_adjacent"]),
            "terrain_boundary_adjacent_points": int(item["boundary_adjacent"]),
            "nearest_water_distance_before_m": _quantiles(active_sampler.before),
            "nearest_water_distance_after_m": _quantiles(active_sampler.after),
            "distance_to_configured_support_ratio_before": _quantiles(
                active_sampler.before_ratio
            ),
            "distance_to_configured_support_ratio_after": _quantiles(
                active_sampler.after_ratio
            ),
            "unresolved_before": int(item["unresolved_before"]),
            "unresolved_after": unresolved_after_count,
            "unresolved_after_fraction": (
                float(unresolved_after_count / eligible) if eligible else None
            ),
        }

    by_spec = {
        identifier: summarize(item) for identifier, item in per_spec.items()
    }
    improvement = sampler.before - sampler.after
    unresolved_fraction = (
        float(unresolved_after / eligible_count) if eligible_count else None
    )
    after_ratio = _quantiles(sampler.after_ratio)
    return {
        "terrain_sources": [path.name for path in terrain_paths],
        "eligibility": {
            "method": "immutable-base-WATER-or-canonical-terrain-free-boundary-centres",
            "final_water_used_for_eligibility": False,
            "annotations_are_bounded_search_neighbourhoods_not_masks": True,
            "eligibility_distance_source": "per-search configured maximum_water_support_distance_m",
            "candidate_independent_boundary_centres": int(
                sum(len(value) for value in boundary_centres.values())
            ),
            "baseline_adjacent_points": int(baseline_adjacent_count),
            "terrain_boundary_adjacent_points": int(boundary_adjacent_count),
        },
        "eligible_edge_points": int(eligible_count),
        "sample": {
            "method": "deterministic-bottom-k-splitmix64-by-source-row",
            "limit": int(edge_sample_limit),
            "count": int(len(sampler.before)),
            "distance_is_right_censored_at_m": EDGE_CENSOR_M,
        },
        "nearest_water_distance_before_m": _quantiles(sampler.before),
        "nearest_water_distance_after_m": _quantiles(sampler.after),
        "distance_improvement_m": _quantiles(improvement),
        "distance_to_configured_support_ratio_before": _quantiles(
            sampler.before_ratio
        ),
        "distance_to_configured_support_ratio_after": after_ratio,
        "unresolved_fraction_limit": EDGE_UNRESOLVED_FRACTION_LIMIT,
        "unresolved_before": int(unresolved_before),
        "unresolved_after": int(unresolved_after),
        "unresolved_after_fraction": unresolved_fraction,
        "per_search_neighbourhood": by_spec,
        "meaningful_edge_continuity_passed": bool(
            eligible_count > 0
            and after_ratio["p90"] is not None
            and after_ratio["p90"] <= 1.0 + 1e-12
            and unresolved_fraction is not None
            and unresolved_fraction <= EDGE_UNRESOLVED_FRACTION_LIMIT + 1e-12
            and all(
                item["eligible_edge_points"] > 0
                and item["distance_to_configured_support_ratio_after"]["p90"]
                is not None
                and item["distance_to_configured_support_ratio_after"]["p90"]
                <= 1.0 + 1e-12
                and item["unresolved_after_fraction"] is not None
                and item["unresolved_after_fraction"]
                <= EDGE_UNRESOLVED_FRACTION_LIMIT + 1e-12
                for item in by_spec.values()
            )
        ),
    }


def _region_metrics(
    specs: Sequence[water_pipeline.CircleSpec],
    slices: Sequence[slice],
    base_xy: np.ndarray,
    additions_xy: np.ndarray,
    labels: np.ndarray,
    base_inner: np.ndarray,
    final_inner: np.ndarray,
    terrain_inner: np.ndarray,
    base_outer: np.ndarray,
    final_outer: np.ndarray,
    terrain_outer: np.ndarray,
) -> tuple[list[dict], dict]:
    regions: list[dict] = []
    all_active = (base_outer + terrain_outer > 0) | (final_outer + terrain_outer > 0)
    all_terrain_free = all_active & (terrain_inner == 0)
    for spec, span in zip(specs, slices):
        dx_base = base_xy[:, 0] - spec.center_xy[0]
        dy_base = base_xy[:, 1] - spec.center_xy[1]
        dx_add = additions_xy[:, 0] - spec.center_xy[0]
        dy_add = additions_xy[:, 1] - spec.center_xy[1]
        base_points = int(np.count_nonzero(dx_base * dx_base + dy_base * dy_base <= spec.radius_m ** 2))
        addition_points = int(np.count_nonzero(dx_add * dx_add + dy_add * dy_add <= spec.radius_m ** 2))
        active = all_active[span]
        terrain_occupied = active & (terrain_inner[span] > 0)
        terrain_free = active & (terrain_inner[span] == 0)
        combined_before = base_inner[span] + terrain_inner[span]
        combined_after = final_inner[span] + terrain_inner[span]
        regions.append({
            "id": spec.identifier,
            "kind": spec.kind,
            "component_label": int(spec.label),
            "search_center_xy": list(spec.center_xy),
            "search_radius_m": float(spec.radius_m),
            "annotations_are_search_neighbourhoods_not_fill_masks": True,
            "water_points": {
                "before": base_points,
                "additions": addition_points,
                "after": base_points + addition_points,
                "registered_component_additions": int(np.count_nonzero(labels == spec.label)),
            },
            "moving_circle": {
                "radius_m": EVALUATION_RADIUS_M,
                "support_activation_radius_m": SUPPORT_RADIUS_M,
                "centres": int(span.stop - span.start),
                "support_active_centres": int(np.count_nonzero(active)),
                "terrain_occupied_centres": int(np.count_nonzero(terrain_occupied)),
                "combined_count_before": _count_summary(combined_before[active]),
                "combined_count_after": _count_summary(combined_after[active]),
                "combined_unresolved_before": int(np.count_nonzero(active & (combined_before == 0))),
                "combined_unresolved_after": int(np.count_nonzero(active & (combined_after == 0))),
                "terrain_free_centres": int(np.count_nonzero(terrain_free)),
                "terrain_free_water_unresolved_before": int(np.count_nonzero(terrain_free & (base_inner[span] == 0))),
                "terrain_free_water_unresolved_after": int(np.count_nonzero(terrain_free & (final_inner[span] == 0))),
            },
        })
    combined_before = base_inner + terrain_inner
    combined_after = final_inner + terrain_inner
    aggregate = {
        "moving_circle_centres": int(len(combined_before)),
        "support_active_centres": int(np.count_nonzero(all_active)),
        "terrain_occupied_centres": int(np.count_nonzero(all_active & (terrain_inner > 0))),
        "combined_count_before": _count_summary(combined_before[all_active]),
        "combined_count_after": _count_summary(combined_after[all_active]),
        "combined_unresolved_before": int(np.count_nonzero(all_active & (combined_before == 0))),
        "combined_unresolved_after": int(np.count_nonzero(all_active & (combined_after == 0))),
        "terrain_free_centres": int(np.count_nonzero(all_terrain_free)),
        "terrain_free_water_unresolved_before": int(np.count_nonzero(all_terrain_free & (base_inner == 0))),
        "terrain_free_water_unresolved_after": int(np.count_nonzero(all_terrain_free & (final_inner == 0))),
        "water_only_center_count_is_acceptance_criterion": False,
        "reason": "Terrain-occupied centres are accepted through combined SAND/ROCK plus WATER support; WATER-only coverage is evaluated only at terrain-free support-active centres.",
    }
    return regions, aggregate


def _fade_audit(specs, additions_tree, final_tree) -> dict:
    fade_specs = [spec for spec in specs if spec.kind == "fade"]
    if not fade_specs:
        return {"feasible": False, "reason": "no registered fade neighbourhood"}
    spec = fade_specs[0]
    centres, keys = _grid_for_spec(spec)
    additions = _tree_counts(additions_tree, centres, EVALUATION_RADIUS_M)
    final = _tree_counts(final_tree, centres, EVALUATION_RADIUS_M)
    active = final > 0
    positive = additions[additions > 0].astype(np.float64)
    key_to_index = {tuple(key): index for index, key in enumerate(keys)}
    differences: list[float] = []
    for index, (ix, iy) in enumerate(keys):
        for neighbour in ((int(ix) + 1, int(iy)), (int(ix), int(iy) + 1)):
            other = key_to_index.get(neighbour)
            if other is not None and active[index] and active[other]:
                differences.append(abs(float(additions[index]) - float(additions[other])))
    diff = np.asarray(differences, np.float64)
    if len(diff):
        median = float(np.median(diff))
        mad = float(np.median(np.abs(diff - median)))
        threshold = max(1.0, median + 6.0 * mad)
        extreme_fraction = float(np.mean(diff > threshold))
    else:
        median = mad = threshold = 0.0
        extreme_fraction = 0.0
    distance = np.linalg.norm(centres - np.asarray(spec.center_xy), axis=1)
    bands = []
    for low, high in zip(np.linspace(0.0, spec.radius_m, 5)[:-1], np.linspace(0.0, spec.radius_m, 5)[1:]):
        member = (distance >= low) & (distance <= high if high == spec.radius_m else distance < high)
        bands.append({
            "inner_radius_m": float(low),
            "outer_radius_m": float(high),
            "centres": int(np.count_nonzero(member)),
            "addition_count": _count_summary(additions[member]),
        })
    feasible = bool(np.count_nonzero(active) >= 12 and len(diff) >= 12)
    return {
        "feasible": feasible,
        "search_neighbourhood": spec.identifier,
        "moving_circle_radius_m": EVALUATION_RADIUS_M,
        "active_centres": int(np.count_nonzero(active)),
        "addition_count": _count_summary(additions[active]),
        "positive_addition_count_coefficient_of_variation": (
            float(np.std(positive) / np.mean(positive)) if len(positive) and np.mean(positive) > 0 else None
        ),
        "distinct_positive_addition_counts": int(len(np.unique(positive))),
        "radial_bands": bands,
        "axis_aligned_adjacent_difference": {
            **_quantiles(diff),
            "median": median,
            "mad": mad,
            "extreme_threshold": threshold,
            "extreme_fraction": extreme_fraction,
        },
        "no_repeated_square_discontinuity_evidence": (
            bool(extreme_fraction <= FADE_EXTREME_FRACTION_LIMIT) if feasible else None
        ),
        "advisory_not_release_gating": True,
    }


def _verified_reference_surface(fine_manifest: Mapping):
    """Load the exact immutable surface that declared density eligibility."""

    block = fine_manifest.get("reference_surface")
    _require(
        isinstance(block, Mapping),
        "geometry manifest lacks reference-surface provenance",
    )
    _require(
        set(block) == {"v9_run_path", "surface", "v10_config"},
        "geometry reference-surface provenance key set differs",
    )
    surface_block = block.get("surface")
    config_block = block.get("v10_config")
    _require(
        isinstance(surface_block, Mapping)
        and isinstance(config_block, Mapping),
        "geometry reference-surface fingerprints are invalid",
    )
    surface_path = _strict_file(
        str(surface_block.get("path", "")), "reference surface-v9"
    )
    config_path = _strict_file(
        str(config_block.get("path", "")), "reference v10 config"
    )
    _assert_fingerprint(surface_block, surface_path, "reference surface-v9")
    _assert_fingerprint(config_block, config_path, "reference v10 config")
    run_path = Path(str(block.get("v9_run_path", ""))).resolve(strict=True)
    _require(run_path.is_dir(), "reference v9 run path is not a directory")
    _require(
        surface_path == run_path / "surface-v9.npz",
        "reference surface-v9 is not inside the declared v9 run",
    )
    return water_pipeline._load_surface(run_path, config_path)


def _declared_density_contract_v2_legacy(
    fine_manifest: Mapping,
    config_document: Mapping,
    specs: Sequence[water_pipeline.CircleSpec],
    geometry_archive_path: str | Path | None = None,
) -> dict:
    value = fine_manifest.get("density_audit")
    _require(isinstance(value, Mapping), "fine manifest lacks measured density audit")
    required = {
        "method",
        "circle_radius_m",
        "step_m",
        "centres_xy",
        "centre_spec_id",
        "centre_spec_kind",
        "centre_spec_label",
        "required_mask",
        "density_eligibility_rule",
        "required_count_by_spec",
        "reference_surface_active_mask",
        "source_water_active_mask",
        "source_support_active_mask",
        "fillable_support_active_mask",
        "support_sampling_pitch_m",
        "support_sample_cell_area_m2",
        "footprint_full_disk_sample_count",
        "valid_footprint_sample_count",
        "valid_footprint_area_m2",
        "fillable_support_sample_count",
        "fillable_support_area_m2",
        "fillable_support_archive_key",
        "fillable_support_pitch_archive_key",
        "reference_combined_density_per_m2",
        "spacing_feasible_capacity_count",
        "spacing_capacity_selection_count",
        "spacing_capacity_seed",
        "spacing_capacity_uses_complete_safe_reservoir",
        "terrain_outer_count",
        "terrain_boundary_mask",
        "terrain_boundary_outer_radius_m",
        "repair_reservoir_count",
        "repair_reservoir_selected_count",
        "initial_lower_violations",
        "upper_violations_after",
        "target_water_count",
        "target_combined_count",
        "target_water_density_per_m2",
        "target_combined_density_per_m2",
        "terrain_count",
        "shoreline_mask",
        "shoreline_terrain_count_threshold",
        "reference_kind",
        "reference_sample_count",
        "minimum_ratio",
        "maximum_ratio",
        "water_lower_count",
        "water_nominal_upper_count",
        "water_upper_count",
        "water_before_count",
        "water_after_count",
        "immutable_source_upper_grandfather_mask",
        "immutable_source_upper_grandfather_count",
        "combined_lower_count",
        "combined_nominal_upper_count",
        "combined_upper_count",
        "combined_before_count",
        "combined_after_count",
        "combined_before_ratio",
        "combined_after_ratio",
        "unresolved_lower_after",
        "new_upper_violations",
        "lower_and_allowed_upper_bounds_enforced",
        "strict_nominal_upper_ratio_enforced_for_non_grandfathered",
        "grandfathered_windows_cannot_increase",
        "water_only_center_count_is_acceptance_criterion",
        "uses_overlapping_circular_windows",
    }
    missing = sorted(required - set(value))
    _require(not missing, f"fine density audit is incomplete: {', '.join(missing)}")
    _require(
        value.get("method")
        == "measured-good-overlap-density-times-fillable-support-area-v2",
        "fine density audit has an unexpected method",
    )
    _require(value.get("lower_and_allowed_upper_bounds_enforced") is True, "fine density audit did not enforce lower/allowed-upper bounds")
    _require(value.get("strict_nominal_upper_ratio_enforced_for_non_grandfathered") is True, "fine density audit did not enforce nominal upper bounds")
    _require(value.get("grandfathered_windows_cannot_increase") is True, "fine density audit did not freeze grandfathered source counts")
    _require(value.get("water_only_center_count_is_acceptance_criterion") is True, "fine density audit does not directly gate WATER support")
    _require(value.get("uses_overlapping_circular_windows") is True, "fine density audit does not use overlapping circular windows")
    parameters = config_document.get("parameters")
    _require(isinstance(parameters, Mapping), "review config lacks parameters")
    radius = float(value["circle_radius_m"])
    step = float(value["step_m"])
    minimum = float(value["minimum_ratio"])
    maximum = float(value["maximum_ratio"])
    _require(abs(radius - float(parameters["density_audit_radius_m"])) <= 1e-12, "density audit radius differs from review config")
    _require(abs(step - float(parameters["density_audit_step_m"])) <= 1e-12, "density audit step differs from review config")
    _require(abs(minimum - float(parameters["density_minimum_ratio"])) <= 1e-12, "density lower ratio differs from review config")
    _require(abs(maximum - float(parameters["density_maximum_ratio"])) <= 1e-12, "density upper ratio differs from review config")
    _require(abs(radius - EVALUATION_RADIUS_M) <= 1e-12, "post-build audit radius differs from its fixed 0.08 m contract")
    centres = np.asarray(value["centres_xy"], np.float64)
    _require(centres.ndim == 2 and centres.shape[1] == 2 and len(centres), "fine density audit centers are invalid")
    count = len(centres)
    def declared_bool_mask(name: str) -> np.ndarray:
        raw = value[name]
        _require(
            isinstance(raw, list)
            and len(raw) == count
            and all(isinstance(item, bool) for item in raw),
            f"fine density audit {name} must be an exact boolean mask",
        )
        return np.asarray(raw, bool)

    arrays = {}
    integer_names = (
        "terrain_count",
        "reference_sample_count",
        "valid_footprint_sample_count",
        "fillable_support_sample_count",
        "spacing_feasible_capacity_count",
        "water_lower_count",
        "water_nominal_upper_count",
        "water_upper_count",
        "water_before_count",
        "water_after_count",
        "combined_lower_count",
        "combined_nominal_upper_count",
        "combined_upper_count",
        "combined_before_count",
        "combined_after_count",
    )
    float_names = (
        "target_water_count",
        "target_combined_count",
        "target_water_density_per_m2",
        "target_combined_density_per_m2",
        "valid_footprint_area_m2",
        "fillable_support_area_m2",
        "reference_combined_density_per_m2",
        "combined_before_ratio",
        "combined_after_ratio",
    )
    for name in integer_names:
        array = np.asarray(value[name], np.int64)
        _require(array.shape == (count,), f"fine density audit {name} shape differs")
        _require(np.all(array >= 0), f"fine density audit {name} contains a negative value")
        arrays[name] = array
    for name in float_names:
        array = np.asarray(value[name], np.float64)
        _require(array.shape == (count,), f"fine density audit {name} shape differs")
        _require(np.all(np.isfinite(array)) and np.all(array >= 0.0), f"fine density audit {name} is invalid")
        arrays[name] = array
    shoreline = declared_bool_mask("shoreline_mask")
    required_mask = declared_bool_mask("required_mask")
    _require(
        required_mask.shape == (count,) and np.any(required_mask),
        "fine density audit has no release-gated centre",
    )
    expected_rule = (
        "all=exact_exclusion_aware_reference_footprint_intersection"
        "&safe_preselection_fillable_support"
    )
    _require(
        value["density_eligibility_rule"] == expected_rule,
        "fine density eligibility rule differs",
    )
    reference_surface_active = declared_bool_mask(
        "reference_surface_active_mask"
    )
    source_water_active = declared_bool_mask("source_water_active_mask")
    surface = _verified_reference_surface(fine_manifest)
    independent_surface_active = water_pipeline._surface_intersects_audit_disks(
        surface,
        centres,
        radius_m=radius,
    )
    _require(
        np.array_equal(reference_surface_active, independent_surface_active),
        "fine density reference-surface mask differs from independent recount",
    )
    source_support_active = declared_bool_mask("source_support_active_mask")
    fillable_support_active = declared_bool_mask(
        "fillable_support_active_mask"
    )
    grandfathered = declared_bool_mask(
        "immutable_source_upper_grandfather_mask"
    )
    support_pitch = float(value["support_sampling_pitch_m"])
    support_cell_area = float(value["support_sample_cell_area_m2"])
    _require(
        np.isfinite(support_pitch)
        and support_pitch > 0.0
        and abs(support_cell_area - support_pitch * support_pitch) <= 1e-15,
        "fine density support sampling geometry is invalid",
    )
    independent_footprint = water_pipeline._surface_audit_disk_support(
        surface,
        centres,
        radius_m=radius,
        sample_pitch_m=support_pitch,
    )
    _require(
        int(value["footprint_full_disk_sample_count"])
        == independent_footprint.full_disk_sample_count,
        "fine density full-disk footprint sample count differs",
    )
    _require(
        np.array_equal(
            arrays["valid_footprint_sample_count"],
            independent_footprint.valid_footprint_sample_count,
        )
        and np.allclose(
            arrays["valid_footprint_area_m2"],
            independent_footprint.valid_footprint_area_m2,
            rtol=0.0,
            atol=1e-15,
        ),
        "fine density exclusion-aware footprint area differs from independent recount",
    )
    _require(
        np.array_equal(reference_surface_active, independent_footprint.active_mask),
        "fine density reference-surface mask differs from exact area recount",
    )
    archive_key = value["fillable_support_archive_key"]
    pitch_key = value["fillable_support_pitch_archive_key"]
    _require(
        archive_key == "fillable_support_cell_keys"
        and pitch_key == "fillable_support_pitch_m",
        "fine density fillable-support archive keys differ",
    )
    archive_value = geometry_archive_path or fine_manifest.get("archive")
    _require(archive_value is not None, "fine density support archive path is missing")
    support_archive = _strict_file(archive_value, "geometry fillable-support archive")
    declared_archive_hash = fine_manifest.get("archive_sha256")
    if declared_archive_hash is not None:
        _require(
            declared_archive_hash == _sha256(support_archive),
            "geometry fillable-support archive hash drift",
        )
    with np.load(support_archive, allow_pickle=False) as loaded:
        _require(
            archive_key in loaded.files and pitch_key in loaded.files,
            "geometry archive lacks fillable-support evidence",
        )
        support_keys = np.asarray(loaded[archive_key], np.int64).copy()
        archived_pitch = float(np.asarray(loaded[pitch_key]).reshape(()))
    _require(
        support_keys.ndim == 2
        and support_keys.shape[1] == 2
        and len(support_keys)
        and np.array_equal(support_keys, np.unique(support_keys, axis=0)),
        "geometry fillable-support cells are invalid or duplicated",
    )
    _require(
        abs(archived_pitch - support_pitch) <= 1e-15,
        "geometry fillable-support pitch differs from manifest",
    )
    support_centres = (support_keys.astype(np.float64) + 0.5) * support_pitch
    independent_fillable_count = water_pipeline._circle_point_counts(
        centres, support_centres, radius_m=radius
    )
    independent_fillable_area = np.minimum(
        independent_fillable_count.astype(np.float64) * support_cell_area,
        independent_footprint.valid_footprint_area_m2,
    )
    _require(
        np.array_equal(
            arrays["fillable_support_sample_count"],
            independent_fillable_count,
        )
        and np.allclose(
            arrays["fillable_support_area_m2"],
            independent_fillable_area,
            rtol=0.0,
            atol=1e-15,
        ),
        "fine density fillable-support area differs from archived-cell recount",
    )
    _require(
        np.array_equal(fillable_support_active, independent_fillable_count > 0),
        "fine density fillable-support mask differs from archived-cell recount",
    )
    terrain_boundary = declared_bool_mask("terrain_boundary_mask")
    spec_id = list(value["centre_spec_id"])
    spec_kind = list(value["centre_spec_kind"])
    spec_label = np.asarray(value["centre_spec_label"], np.int64)
    _require(
        len(spec_id) == count
        and all(isinstance(item, str) and item for item in spec_id),
        "fine density spec ids are invalid",
    )
    _require(
        len(spec_kind) == count
        and all(
            isinstance(item, str) and item in MEASURED_DENSITY_KINDS
            for item in spec_kind
        ),
        "fine density spec kinds are invalid",
    )
    _require(spec_label.shape == (count,), "fine density spec labels shape differs")
    configured = {
        spec.identifier: spec
        for spec in specs
        if spec.kind in MEASURED_DENSITY_KINDS
    }
    _require(configured, "review config has no release-gated density specs")
    _require(
        set(spec_id) == set(configured),
        "fine density audit does not cover every configured interface/hole/dip search",
    )
    for index, identifier in enumerate(spec_id):
        spec = configured.get(identifier)
        _require(spec is not None, f"fine density audit has unknown spec {identifier}")
        _require(spec_kind[index] == spec.kind, f"fine density kind differs for {identifier}")
        _require(spec_label[index] == spec.label, f"fine density label differs for {identifier}")
        delta = centres[index] - np.asarray(spec.center_xy, np.float64)
        _require(
            float(delta @ delta) <= spec.radius_m * spec.radius_m + 1e-10,
            f"fine density centre lies outside search neighbourhood {identifier}",
        )
    eligibility_contract = water_pipeline.DensityAuditCentres(
        centres_xy=centres,
        spec_id=tuple(spec_id),
        spec_kind=tuple(spec_kind),
        spec_label=spec_label.astype(np.int32),
    )
    expected_required = water_pipeline._density_required_mask(
        eligibility_contract,
        independent_footprint.active_mask,
        independent_fillable_count > 0,
    )
    _require(
        np.array_equal(required_mask, expected_required),
        "fine density required mask differs from exact support recount",
    )
    for identifier in configured:
        member = np.asarray([item == identifier for item in spec_id], bool)
        _require(
            np.any(member & required_mask),
            f"fine density search {identifier} has no release-gated centre",
        )
    declared_required_by_spec = value["required_count_by_spec"]
    _require(
        isinstance(declared_required_by_spec, Mapping)
        and set(declared_required_by_spec) == set(configured)
        and all(
            isinstance(item, int)
            and not isinstance(item, bool)
            and item > 0
            for item in declared_required_by_spec.values()
        ),
        "fine density required-count-by-spec is invalid",
    )
    required_by_spec = {
        identifier: int(np.count_nonzero(
            required_mask
            & np.asarray([item == identifier for item in spec_id], bool)
        ))
        for identifier in configured
    }
    _require(
        dict(declared_required_by_spec) == required_by_spec,
        "fine density required-count-by-spec differs from required mask",
    )
    terrain_outer_radius = float(value["terrain_boundary_outer_radius_m"])
    _require(
        abs(terrain_outer_radius - (radius + step)) <= 1e-12,
        "fine density terrain-boundary radius differs from audit radius plus step",
    )
    for name in (
        "terrain_outer_count",
        "repair_reservoir_count",
        "repair_reservoir_selected_count",
    ):
        array = np.asarray(value[name], np.int64)
        _require(array.shape == (count,), f"fine density audit {name} shape differs")
        _require(np.all(array >= 0), f"fine density audit {name} contains a negative value")
        arrays[name] = array
    _require(
        np.all(
            arrays["repair_reservoir_selected_count"]
            <= arrays["repair_reservoir_count"]
        ),
        "fine density selected repair count exceeds its bounded reservoir",
    )
    scalar_counts = {}
    for name in (
        "initial_lower_violations",
        "unresolved_lower_after",
        "upper_violations_after",
    ):
        raw = value[name]
        _require(
            isinstance(raw, int) and not isinstance(raw, bool) and raw >= 0,
            f"fine density audit {name} is invalid",
        )
        scalar_counts[name] = int(raw)
    _require(
        scalar_counts["unresolved_lower_after"] == 0,
        "fine density audit declares unresolved lower-bound centres",
    )
    _require(
        scalar_counts["upper_violations_after"] == 0,
        "fine density audit declares upper-bound centres",
    )
    references = list(value["reference_kind"])
    _require(len(references) == count and all(isinstance(item, str) for item in references), "fine density reference kinds are invalid")
    shoreline_threshold = float(value["shoreline_terrain_count_threshold"])
    _require(np.isfinite(shoreline_threshold) and shoreline_threshold >= 0.0, "fine density shoreline threshold is invalid")
    area = math.pi * radius * radius
    _require(
        np.allclose(
            arrays["target_combined_density_per_m2"],
            arrays["target_combined_count"] / area,
            rtol=1e-9,
            atol=1e-9,
        ),
        "fine density combined target count/density differs",
    )
    _require(
        np.allclose(
            arrays["target_water_density_per_m2"],
            arrays["target_water_count"] / area,
            rtol=1e-9,
            atol=1e-9,
        ),
        "fine density WATER target count/density differs",
    )
    _require(
        np.allclose(
            arrays["target_water_count"],
            arrays["reference_combined_density_per_m2"]
            * arrays["fillable_support_area_m2"],
            rtol=1e-9,
            atol=1e-9,
        ),
        "fine density WATER target differs from support-area reference density",
    )
    _require(
        np.allclose(
            arrays["target_combined_count"],
            arrays["terrain_count"] + arrays["target_water_count"],
            rtol=1e-9,
            atol=1e-9,
        ),
        "fine density combined target is not fixed terrain plus WATER target",
    )
    expected_water_lower = np.ceil(
        minimum * arrays["target_water_count"]
    ).astype(np.int64)
    expected_water_lower[~required_mask] = 0
    expected_nominal_upper = np.floor(
        maximum * arrays["target_water_count"]
    ).astype(np.int64)
    expected_grandfather = required_mask & (
        arrays["water_before_count"] > expected_nominal_upper
    )
    expected_allowed_upper = np.maximum(
        expected_nominal_upper, arrays["water_before_count"]
    )
    _require(
        np.array_equal(arrays["water_lower_count"], expected_water_lower)
        and np.array_equal(
            arrays["water_nominal_upper_count"], expected_nominal_upper
        )
        and np.array_equal(arrays["water_upper_count"], expected_allowed_upper),
        "fine density direct WATER lower/upper bounds differ",
    )
    _require(
        np.array_equal(grandfathered, expected_grandfather)
        and int(value["immutable_source_upper_grandfather_count"])
        == int(np.count_nonzero(expected_grandfather)),
        "fine density immutable-source upper exception differs",
    )
    _require(
        np.array_equal(
            arrays["combined_lower_count"],
            arrays["terrain_count"] + expected_water_lower,
        )
        and np.array_equal(
            arrays["combined_nominal_upper_count"],
            arrays["terrain_count"] + expected_nominal_upper,
        )
        and np.array_equal(
            arrays["combined_upper_count"],
            arrays["terrain_count"] + expected_allowed_upper,
        ),
        "fine density combined upper-bound arithmetic differs",
    )
    _require(
        np.all(
            arrays["water_lower_count"][required_mask]
            <= (
                arrays["water_before_count"]
                + arrays["spacing_feasible_capacity_count"]
            )[required_mask]
        ),
        "fine density lower bound exceeds declared deterministic spacing capacity",
    )
    for name in ("spacing_capacity_selection_count", "spacing_capacity_seed"):
        _require(
            isinstance(value[name], int) and not isinstance(value[name], bool),
            f"fine density {name} is invalid",
        )
    _require(
        value["spacing_capacity_uses_complete_safe_reservoir"] is True,
        "fine density spacing capacity was not measured from the safe reservoir",
    )
    return {
        "centres": centres,
        "radius_m": radius,
        "step_m": step,
        "minimum_ratio": minimum,
        "maximum_ratio": maximum,
        "shoreline_mask": shoreline,
        "required_mask": required_mask,
        "density_eligibility_rule": expected_rule,
        "required_count_by_spec": required_by_spec,
        "reference_surface_active_mask": reference_surface_active,
        "source_water_active_mask": source_water_active,
        "source_support_active_mask": source_support_active,
        "fillable_support_active_mask": fillable_support_active,
        "immutable_source_upper_grandfather_mask": grandfathered,
        "support_sampling_pitch_m": support_pitch,
        "support_sample_cell_area_m2": support_cell_area,
        "terrain_boundary_mask": terrain_boundary,
        "terrain_boundary_outer_radius_m": terrain_outer_radius,
        "centre_spec_id": spec_id,
        "centre_spec_kind": spec_kind,
        "centre_spec_label": spec_label,
        **scalar_counts,
        "shoreline_terrain_count_threshold": shoreline_threshold,
        "reference_kind": references,
        **arrays,
    }


def _declared_density_contract(
    fine_manifest: Mapping,
    config_document: Mapping,
    specs: Sequence[water_pipeline.CircleSpec],
    geometry_archive_path: str | Path | None = None,
) -> dict:
    """Reconstruct the v3 vacant-support addition contract.

    This deliberately does not trust a declared total-density target.  It
    re-counts raw and immutable-WATER-clear support cells, re-counts the one
    archived globally spaced capacity witness, reconstructs WATER-only
    reference density, and then rebuilds the discrete addition interval.
    """

    value = fine_manifest.get("density_audit")
    _require(isinstance(value, Mapping), "fine manifest lacks measured density audit")
    required = {
        "method", "circle_radius_m", "step_m", "centres_xy",
        "centre_spec_id", "centre_spec_kind", "centre_spec_label",
        "required_mask", "density_eligibility_rule", "required_count_by_spec",
        "reference_surface_active_mask", "source_water_active_mask",
        "source_support_active_mask", "vacant_support_active_mask",
        "support_sampling_pitch_m", "support_sample_cell_area_m2",
        "footprint_full_disk_sample_count", "valid_footprint_sample_count",
        "valid_footprint_area_m2", "raw_support_sample_count",
        "raw_support_area_m2", "vacant_support_sample_count",
        "vacant_support_area_m2", "raw_support_cell_count",
        "vacant_support_cell_count", "raw_support_archive_key",
        "raw_support_representative_archive_key", "vacant_support_archive_key",
        "vacant_support_representative_archive_key", "support_pitch_archive_key",
        "immutable_water_spacing_m", "immutable_water_blocker_count",
        "all_surviving_immutable_water_rows_block_placement",
        "reference_water_density_per_m2", "local_reference_centres_xy",
        "local_reference_water_count", "local_reference_terrain_count",
        "local_reference_water_area_m2", "good_overlap_reference_centres_xy",
        "good_overlap_reference_water_count",
        "good_overlap_reference_terrain_count",
        "good_overlap_reference_water_area_m2", "reference_kind",
        "reference_sample_count", "spacing_feasible_capacity_count",
        "spacing_capacity_selection_count", "spacing_capacity_seed",
        "spacing_capacity_archive_key",
        "spacing_capacity_uses_complete_safe_reservoir",
        "spacing_capacity_is_constructive_witness_not_maximum",
        "spacing_capacity_in_joint_candidate_pool", "raw_desired_addition_count",
        "target_addition_count", "addition_lower_count", "addition_upper_count",
        "capacity_sufficient_mask", "addition_bounds_rounding",
        "immutable_source_water_count", "target_water_count",
        "target_combined_count", "target_water_density_per_m2",
        "target_combined_density_per_m2", "terrain_count", "terrain_outer_count",
        "terrain_boundary_mask", "terrain_boundary_outer_radius_m",
        "shoreline_mask", "shoreline_terrain_count_threshold", "minimum_ratio",
        "maximum_ratio", "repair_reservoir_count",
        "repair_reservoir_selected_count", "water_lower_count",
        "water_nominal_upper_count", "water_upper_count", "water_before_count",
        "water_after_count", "combined_lower_count",
        "combined_nominal_upper_count", "combined_upper_count",
        "combined_before_count", "combined_after_count", "combined_before_ratio",
        "combined_after_ratio", "initial_lower_violations",
        "unresolved_lower_after", "upper_violations_after",
        "new_upper_violations", "lower_and_allowed_upper_bounds_enforced",
        "water_only_center_count_is_acceptance_criterion",
        "uses_overlapping_circular_windows",
    }
    missing = sorted(required - set(value))
    _require(not missing, f"fine density audit is incomplete: {', '.join(missing)}")
    _require(
        value["method"] == "measured-water-density-times-vacant-support-area-v3",
        "fine density audit has an unexpected method",
    )
    _require(
        value["density_eligibility_rule"]
        == "all=exact_exclusion_aware_reference_footprint_intersection"
        "&genuinely_vacant_safe_support_after_immutable_water_clearance",
        "fine density eligibility rule differs",
    )
    for name in (
        "all_surviving_immutable_water_rows_block_placement",
        "spacing_capacity_uses_complete_safe_reservoir",
        "spacing_capacity_is_constructive_witness_not_maximum",
        "spacing_capacity_in_joint_candidate_pool",
        "lower_and_allowed_upper_bounds_enforced",
        "water_only_center_count_is_acceptance_criterion",
        "uses_overlapping_circular_windows",
    ):
        _require(value[name] is True, f"fine density contract flag {name} is false")
    _require(
        value["addition_bounds_rounding"]
        == "ceil-both-on-discrete-point-count-lattice",
        "fine density addition rounding convention differs",
    )

    parameters = config_document.get("parameters")
    _require(isinstance(parameters, Mapping), "review config lacks parameters")
    radius = float(value["circle_radius_m"])
    step = float(value["step_m"])
    minimum = float(value["minimum_ratio"])
    maximum = float(value["maximum_ratio"])
    spacing = float(value["immutable_water_spacing_m"])
    _require(abs(radius - float(parameters["density_audit_radius_m"])) <= 1e-12, "density audit radius differs from review config")
    _require(abs(step - float(parameters["density_audit_step_m"])) <= 1e-12, "density audit step differs from review config")
    _require(abs(minimum - float(parameters["density_minimum_ratio"])) <= 1e-12, "density lower ratio differs from review config")
    _require(abs(maximum - float(parameters["density_maximum_ratio"])) <= 1e-12, "density upper ratio differs from review config")
    _require(abs(spacing - float(parameters["fine_water_selection_radius_m"])) <= 1e-12, "immutable-WATER spacing differs from review config")
    _require(abs(radius - EVALUATION_RADIUS_M) <= 1e-12, "post-build audit radius differs from fixed contract")

    centres = np.asarray(value["centres_xy"], np.float64)
    _require(centres.ndim == 2 and centres.shape[1] == 2 and len(centres), "fine density audit centres are invalid")
    count = len(centres)

    def bool_mask(name: str) -> np.ndarray:
        raw = value[name]
        _require(isinstance(raw, list) and len(raw) == count and all(isinstance(item, bool) for item in raw), f"fine density audit {name} must be an exact boolean mask")
        return np.asarray(raw, bool)

    def integer_array(name: str, expected: int = count) -> np.ndarray:
        raw = np.asarray(value[name])
        _require(raw.shape == (expected,), f"fine density audit {name} shape differs")
        cast = raw.astype(np.int64)
        _require(np.all(np.isfinite(raw.astype(np.float64))) and np.array_equal(raw, cast) and np.all(cast >= 0), f"fine density audit {name} is not non-negative integer data")
        return cast

    def float_array(name: str, expected: int = count) -> np.ndarray:
        result = np.asarray(value[name], np.float64)
        _require(result.shape == (expected,), f"fine density audit {name} shape differs")
        _require(np.all(np.isfinite(result)) and np.all(result >= 0.0), f"fine density audit {name} is invalid")
        return result

    spec_id = list(value["centre_spec_id"])
    spec_kind = list(value["centre_spec_kind"])
    spec_label = np.asarray(value["centre_spec_label"], np.int64)
    _require(len(spec_id) == count and all(isinstance(item, str) and item for item in spec_id), "fine density spec ids are invalid")
    _require(len(spec_kind) == count and all(item in MEASURED_DENSITY_KINDS for item in spec_kind), "fine density spec kinds are invalid")
    _require(spec_label.shape == (count,), "fine density spec labels shape differs")
    configured = {spec.identifier: spec for spec in specs if spec.kind in MEASURED_DENSITY_KINDS}
    _require(configured and set(spec_id) == set(configured), "fine density audit does not cover every configured search")
    for row, identifier in enumerate(spec_id):
        spec = configured.get(identifier)
        _require(spec is not None and spec_kind[row] == spec.kind and spec_label[row] == spec.label, f"fine density metadata differs for {identifier}")
        delta = centres[row] - np.asarray(spec.center_xy, np.float64)
        _require(float(delta @ delta) <= spec.radius_m**2 + 1e-10, f"fine density centre lies outside {identifier}")

    required_mask = bool_mask("required_mask")
    reference_surface_active = bool_mask("reference_surface_active_mask")
    source_water_active = bool_mask("source_water_active_mask")
    source_support_active = bool_mask("source_support_active_mask")
    vacant_support_active = bool_mask("vacant_support_active_mask")
    shoreline = bool_mask("shoreline_mask")
    terrain_boundary = bool_mask("terrain_boundary_mask")
    capacity_sufficient = bool_mask("capacity_sufficient_mask")
    surface = _verified_reference_surface(fine_manifest)
    support_pitch = float(value["support_sampling_pitch_m"])
    support_cell_area = float(value["support_sample_cell_area_m2"])
    _require(np.isfinite(support_pitch) and support_pitch > 0.0 and abs(support_cell_area - support_pitch**2) <= 1e-15, "fine density support sampling geometry is invalid")
    footprint = water_pipeline._surface_audit_disk_support(surface, centres, radius_m=radius, sample_pitch_m=support_pitch)
    _require(int(value["footprint_full_disk_sample_count"]) == footprint.full_disk_sample_count, "fine density full-disk footprint count differs")
    valid_footprint_count = integer_array("valid_footprint_sample_count")
    valid_footprint_area = float_array("valid_footprint_area_m2")
    _require(np.array_equal(valid_footprint_count, footprint.valid_footprint_sample_count) and np.allclose(valid_footprint_area, footprint.valid_footprint_area_m2, rtol=0.0, atol=1e-15), "fine density footprint area differs from independent recount")
    _require(np.array_equal(reference_surface_active, footprint.active_mask), "fine density footprint-active mask differs")

    expected_archive_keys = {
        "raw_support_archive_key": "raw_support_cell_keys",
        "raw_support_representative_archive_key": "raw_support_representative_xy",
        "vacant_support_archive_key": "vacant_support_cell_keys",
        "vacant_support_representative_archive_key": "vacant_support_representative_xy",
        "support_pitch_archive_key": "support_pitch_m",
        "spacing_capacity_archive_key": "spacing_capacity_xy",
    }
    for manifest_key, archive_key in expected_archive_keys.items():
        _require(value[manifest_key] == archive_key, f"fine density archive key {manifest_key} differs")
    archive_value = geometry_archive_path or fine_manifest.get("archive")
    _require(archive_value is not None, "fine density evidence archive path is missing")
    archive = _strict_file(archive_value, "geometry density-evidence archive")
    declared_hash = fine_manifest.get("archive_sha256")
    if declared_hash is not None:
        _require(declared_hash == _sha256(archive), "geometry density-evidence archive hash drift")
    with np.load(archive, allow_pickle=False) as loaded:
        _require(set(expected_archive_keys.values()) <= set(loaded.files), "geometry archive lacks v3 density evidence")
        raw_keys = np.asarray(loaded["raw_support_cell_keys"], np.int64).copy()
        raw_representatives = np.asarray(loaded["raw_support_representative_xy"], np.float64).copy()
        vacant_keys = np.asarray(loaded["vacant_support_cell_keys"], np.int64).copy()
        vacant_representatives = np.asarray(loaded["vacant_support_representative_xy"], np.float64).copy()
        archived_pitch = float(np.asarray(loaded["support_pitch_m"]).reshape(()))
        capacity_xy = np.asarray(loaded["spacing_capacity_xy"], np.float64).copy()
    _require(abs(archived_pitch - support_pitch) <= 1e-15, "geometry support pitch differs from manifest")

    def validate_cells(label: str, keys: np.ndarray, representatives: np.ndarray) -> None:
        _require(keys.ndim == 2 and keys.shape[1] == 2 and len(keys), f"{label} support keys are invalid")
        _require(representatives.shape == keys.shape and np.all(np.isfinite(representatives)), f"{label} support representatives are invalid")
        _require(len(np.unique(keys, axis=0)) == len(keys), f"{label} support keys are duplicated")
        _require(np.array_equal(np.floor(representatives / support_pitch).astype(np.int64), keys), f"{label} support representatives do not reconstruct their keys")

    validate_cells("raw", raw_keys, raw_representatives)
    validate_cells("vacant", vacant_keys, vacant_representatives)
    raw_view = np.ascontiguousarray(raw_keys).view(np.dtype((np.void, raw_keys.dtype.itemsize * 2))).ravel()
    vacant_view = np.ascontiguousarray(vacant_keys).view(np.dtype((np.void, vacant_keys.dtype.itemsize * 2))).ravel()
    _require(np.all(np.isin(vacant_view, raw_view)), "vacant support is not a subset of raw support")
    _require(int(value["raw_support_cell_count"]) == len(raw_keys) and int(value["vacant_support_cell_count"]) == len(vacant_keys), "global support-cell counts differ from archive")
    raw_centres = (raw_keys.astype(np.float64) + 0.5) * support_pitch
    vacant_centres = (vacant_keys.astype(np.float64) + 0.5) * support_pitch
    independent_raw_count = water_pipeline._circle_point_counts(centres, raw_centres, radius_m=radius)
    independent_vacant_count = water_pipeline._circle_point_counts(centres, vacant_centres, radius_m=radius)
    independent_raw_area = np.minimum(independent_raw_count * support_cell_area, footprint.valid_footprint_area_m2)
    independent_vacant_area = np.minimum(independent_vacant_count * support_cell_area, footprint.valid_footprint_area_m2)
    raw_support_count = integer_array("raw_support_sample_count")
    vacant_support_count = integer_array("vacant_support_sample_count")
    raw_support_area = float_array("raw_support_area_m2")
    vacant_support_area = float_array("vacant_support_area_m2")
    _require(np.array_equal(raw_support_count, independent_raw_count) and np.allclose(raw_support_area, independent_raw_area, rtol=0.0, atol=1e-15), "raw support differs from archived-cell recount")
    _require(np.array_equal(vacant_support_count, independent_vacant_count) and np.allclose(vacant_support_area, independent_vacant_area, rtol=0.0, atol=1e-15), "vacant support differs from archived-cell recount")
    _require(np.all(vacant_support_count <= raw_support_count) and np.all(vacant_support_area <= raw_support_area + 1e-15), "vacant support exceeds raw support")
    _require(np.array_equal(vacant_support_active, independent_vacant_count > 0), "vacant-support active mask differs from recount")

    _require(capacity_xy.ndim == 2 and capacity_xy.shape[1] == 2 and np.all(np.isfinite(capacity_xy)), "spacing-capacity witness is invalid")
    _require(int(value["spacing_capacity_selection_count"]) == len(capacity_xy), "spacing-capacity global count differs")
    capacity_cell_keys = np.floor(capacity_xy / support_pitch).astype(np.int64)
    capacity_view = np.ascontiguousarray(capacity_cell_keys).view(np.dtype((np.void, capacity_cell_keys.dtype.itemsize * 2))).ravel()
    _require(np.all(np.isin(capacity_view, vacant_view)), "spacing-capacity witness leaves vacant support")
    capacity_count = water_pipeline._circle_point_counts(centres, capacity_xy, radius_m=radius)
    declared_capacity_count = integer_array("spacing_feasible_capacity_count")
    _require(np.array_equal(capacity_count, declared_capacity_count), "spacing-capacity window counts differ from archived witness")
    if len(capacity_xy) > 1:
        try:
            from scipy.spatial import cKDTree
        except ModuleNotFoundError:
            _require(
                len(capacity_xy) <= 10_000,
                "SciPy cKDTree is required to verify pair spacing for a "
                "production-size capacity witness; the bounded NumPy fallback "
                "is limited to 10,000 points",
            )
            delta = capacity_xy[:, None, :] - capacity_xy[None, :, :]
            squared = np.sum(delta * delta, axis=2)
            np.fill_diagonal(squared, np.inf)
            minimum_capacity_spacing = float(np.sqrt(np.min(squared)))
        else:
            nearest, _ = cKDTree(capacity_xy).query(capacity_xy, k=2, workers=-1)
            minimum_capacity_spacing = float(np.min(nearest[:, 1]))
        _require(minimum_capacity_spacing >= spacing - 1e-12, "spacing-capacity witness violates pair spacing")

    eligibility = water_pipeline.DensityAuditCentres(centres, tuple(spec_id), tuple(spec_kind), spec_label.astype(np.int32))
    expected_required = water_pipeline._density_required_mask(eligibility, footprint.active_mask, independent_vacant_count > 0)
    _require(np.array_equal(required_mask, expected_required), "required mask differs from exact vacant-support eligibility")
    declared_required = value["required_count_by_spec"]
    expected_required_by_spec = {identifier: int(np.count_nonzero(required_mask & (np.asarray(spec_id, object) == identifier))) for identifier in configured}
    _require(isinstance(declared_required, Mapping) and dict(declared_required) == expected_required_by_spec and all(item > 0 for item in expected_required_by_spec.values()), "required-count-by-spec differs")

    settings = water_pipeline._density_continuity_settings(parameters)
    good_overlap = water_pipeline._good_overlap_spec(config_document)
    expected_local_centres, expected_overlap_centres = water_pipeline._density_reference_centres(specs, good_overlap, settings)
    local_centres = np.asarray(value["local_reference_centres_xy"], np.float64)
    overlap_centres = np.asarray(value["good_overlap_reference_centres_xy"], np.float64)
    _require(np.array_equal(local_centres, expected_local_centres) and np.array_equal(overlap_centres, expected_overlap_centres), "density reference centres differ from review config")
    local_count = len(local_centres)
    overlap_count = len(overlap_centres)
    local_water_count = integer_array("local_reference_water_count", local_count)
    local_terrain_count = integer_array("local_reference_terrain_count", local_count)
    overlap_water_count = integer_array("good_overlap_reference_water_count", overlap_count)
    overlap_terrain_count = integer_array("good_overlap_reference_terrain_count", overlap_count)
    local_area = float_array("local_reference_water_area_m2", local_count)
    overlap_area = float_array("good_overlap_reference_water_area_m2", overlap_count)
    independent_local_support = water_pipeline._surface_audit_disk_support(surface, local_centres, radius_m=radius, sample_pitch_m=support_pitch)
    independent_overlap_support = water_pipeline._surface_audit_disk_support(surface, overlap_centres, radius_m=radius, sample_pitch_m=support_pitch)
    _require(np.allclose(local_area, independent_local_support.valid_footprint_area_m2, rtol=0.0, atol=1e-15) and np.allclose(overlap_area, independent_overlap_support.valid_footprint_area_m2, rtol=0.0, atol=1e-15), "density reference footprint areas differ from independent recount")
    local_density = np.divide(local_water_count, local_area, out=np.zeros(local_count, np.float64), where=local_area > 0.0)
    overlap_density = np.divide(overlap_water_count, overlap_area, out=np.zeros(overlap_count, np.float64), where=overlap_area > 0.0)
    valid_overlap = (overlap_water_count > 0) & (overlap_terrain_count > 0) & (overlap_area > 0.0)
    valid_local = (local_water_count > 0) & (local_area > 0.0)
    reference_density = np.zeros(count, np.float64)
    reference_sample_count = np.zeros(count, np.int64)
    reference_kind: list[str] = []
    for row, centre in enumerate(centres):
        if np.count_nonzero(valid_overlap) >= settings.minimum_reference_windows:
            ref_centres, ref_values, valid, label = overlap_centres, overlap_density, valid_overlap, "good-overlap-water-area-normalized"
        else:
            ref_centres, ref_values, valid, label = local_centres, local_density, valid_local, "local-water-area-normalized"
        index = np.flatnonzero(valid)
        _require(len(index) >= settings.minimum_reference_windows, "insufficient WATER-only density reference windows")
        distance_squared = np.sum((ref_centres[index] - centre[None, :]) ** 2, axis=1)
        selected = index[np.argsort(distance_squared, kind="stable")[: settings.reference_neighbours]]
        reference_density[row] = float(np.median(ref_values[selected]))
        reference_sample_count[row] = len(selected)
        reference_kind.append(label)
    declared_reference_density = float_array("reference_water_density_per_m2")
    _require(np.allclose(declared_reference_density, reference_density, rtol=1e-12, atol=1e-9), "WATER-only reference density differs from reconstruction")
    _require(np.array_equal(integer_array("reference_sample_count"), reference_sample_count) and list(value["reference_kind"]) == reference_kind, "density reference selection differs")

    raw_desired = float_array("raw_desired_addition_count")
    _require(np.allclose(raw_desired, reference_density * independent_vacant_area, rtol=1e-12, atol=1e-12), "raw addition demand differs from WATER density times vacant area")
    immutable = integer_array("immutable_source_water_count")
    water_before = integer_array("water_before_count")
    _require(np.array_equal(immutable, water_before), "immutable WATER baseline differs from before count")
    rebuilt = water_pipeline.refinement.attainable_addition_density_contract(immutable, raw_desired, capacity_count, minimum_ratio=minimum, maximum_ratio=maximum, active_centre_mask=required_mask)
    integer_expected = {
        "addition_lower_count": rebuilt.addition_lower_count,
        "addition_upper_count": rebuilt.addition_upper_count,
        "water_lower_count": rebuilt.water_lower_count,
        "water_nominal_upper_count": rebuilt.water_upper_count,
        "water_upper_count": rebuilt.water_upper_count,
    }
    arrays: dict[str, np.ndarray] = {
        "valid_footprint_sample_count": valid_footprint_count,
        "valid_footprint_area_m2": valid_footprint_area,
        "raw_support_sample_count": raw_support_count,
        "raw_support_area_m2": raw_support_area,
        "vacant_support_sample_count": vacant_support_count,
        "vacant_support_area_m2": vacant_support_area,
        "spacing_feasible_capacity_count": capacity_count,
        "reference_water_density_per_m2": reference_density,
        "reference_sample_count": reference_sample_count,
        "raw_desired_addition_count": raw_desired,
        "immutable_source_water_count": immutable,
        "water_before_count": water_before,
    }
    for name, expected in integer_expected.items():
        actual = integer_array(name)
        _require(np.array_equal(actual, expected), f"fine density {name} differs from rebuilt addition contract")
        arrays[name] = actual
    target_addition = float_array("target_addition_count")
    _require(np.allclose(target_addition, rebuilt.target_addition_count, rtol=0.0, atol=1e-12), "addition target was capped or changed")
    _require(np.array_equal(capacity_sufficient, rebuilt.capacity_sufficient_mask) and np.all(capacity_sufficient[required_mask]), "constructive capacity does not prove every active lower bound")
    arrays["target_addition_count"] = target_addition
    arrays["addition_lower_count"] = rebuilt.addition_lower_count
    arrays["addition_upper_count"] = rebuilt.addition_upper_count

    terrain_count = integer_array("terrain_count")
    terrain_outer_count = integer_array("terrain_outer_count")
    target_water = float_array("target_water_count")
    target_combined = float_array("target_combined_count")
    disk_area = math.pi * radius**2
    _require(np.allclose(target_water, immutable + target_addition, rtol=0.0, atol=1e-12), "immutable WATER was not added exactly once")
    _require(np.allclose(target_combined, terrain_count + target_water, rtol=0.0, atol=1e-12), "combined target arithmetic differs")
    target_water_density = float_array("target_water_density_per_m2")
    target_combined_density = float_array("target_combined_density_per_m2")
    _require(np.allclose(target_water_density, target_water / disk_area, rtol=1e-12, atol=1e-9) and np.allclose(target_combined_density, target_combined / disk_area, rtol=1e-12, atol=1e-9), "target count/density conversion differs")
    combined_lower = integer_array("combined_lower_count")
    combined_nominal_upper = integer_array("combined_nominal_upper_count")
    combined_upper = integer_array("combined_upper_count")
    _require(np.array_equal(combined_lower, terrain_count + rebuilt.water_lower_count) and np.array_equal(combined_nominal_upper, terrain_count + rebuilt.water_upper_count) and np.array_equal(combined_upper, terrain_count + rebuilt.water_upper_count), "combined addition-bound arithmetic differs")
    arrays.update(target_water_count=target_water, target_combined_count=target_combined, target_water_density_per_m2=target_water_density, target_combined_density_per_m2=target_combined_density, terrain_count=terrain_count, terrain_outer_count=terrain_outer_count, combined_lower_count=combined_lower, combined_nominal_upper_count=combined_nominal_upper, combined_upper_count=combined_upper)
    for name in ("water_after_count", "combined_before_count", "combined_after_count", "repair_reservoir_count", "repair_reservoir_selected_count"):
        arrays[name] = integer_array(name)
    _require(np.all(arrays["repair_reservoir_selected_count"] <= arrays["repair_reservoir_count"]), "selected repair count exceeds reservoir")
    for name in ("combined_before_ratio", "combined_after_ratio"):
        arrays[name] = float_array(name)
    for name in ("initial_lower_violations", "unresolved_lower_after", "upper_violations_after", "new_upper_violations"):
        raw = value[name]
        _require(isinstance(raw, int) and not isinstance(raw, bool) and raw >= 0, f"fine density {name} is invalid")
    _require(int(value["unresolved_lower_after"]) == 0 and int(value["upper_violations_after"]) == 0 and int(value["new_upper_violations"]) == 0, "fine density manifest declares a final bound violation")
    terrain_outer_radius = float(value["terrain_boundary_outer_radius_m"])
    _require(abs(terrain_outer_radius - (radius + step)) <= 1e-12, "terrain-boundary radius differs")
    shoreline_threshold = float(value["shoreline_terrain_count_threshold"])
    _require(np.isfinite(shoreline_threshold) and shoreline_threshold >= 0.0, "shoreline threshold is invalid")
    blocker_count = value["immutable_water_blocker_count"]
    _require(isinstance(blocker_count, int) and not isinstance(blocker_count, bool) and blocker_count > 0, "immutable blocker count is invalid")
    capacity_seed = value["spacing_capacity_seed"]
    _require(isinstance(capacity_seed, int) and not isinstance(capacity_seed, bool), "spacing-capacity seed is invalid")
    return {
        "centres": centres, "radius_m": radius, "step_m": step,
        "minimum_ratio": minimum, "maximum_ratio": maximum,
        "immutable_water_spacing_m": spacing,
        "shoreline_mask": shoreline, "required_mask": required_mask,
        "density_eligibility_rule": value["density_eligibility_rule"],
        "required_count_by_spec": expected_required_by_spec,
        "reference_surface_active_mask": reference_surface_active,
        "source_water_active_mask": source_water_active,
        "source_support_active_mask": source_support_active,
        "fillable_support_active_mask": vacant_support_active,
        "vacant_support_active_mask": vacant_support_active,
        "capacity_sufficient_mask": capacity_sufficient,
        "support_sampling_pitch_m": support_pitch,
        "support_sample_cell_area_m2": support_cell_area,
        "terrain_boundary_mask": terrain_boundary,
        "terrain_boundary_outer_radius_m": terrain_outer_radius,
        "centre_spec_id": spec_id, "centre_spec_kind": spec_kind,
        "centre_spec_label": spec_label,
        "shoreline_terrain_count_threshold": shoreline_threshold,
        "reference_kind": reference_kind,
        "surface": surface,
        "raw_support_representative_xy": raw_representatives,
        "vacant_support_representative_xy": vacant_representatives,
        "spacing_capacity_xy": capacity_xy,
        "local_reference_centres_xy": local_centres,
        "good_overlap_reference_centres_xy": overlap_centres,
        "local_reference_water_count": local_water_count,
        "local_reference_terrain_count": local_terrain_count,
        "good_overlap_reference_water_count": overlap_water_count,
        "good_overlap_reference_terrain_count": overlap_terrain_count,
        "local_reference_water_area_m2": local_area,
        "good_overlap_reference_water_area_m2": overlap_area,
        "immutable_water_blocker_count": int(blocker_count),
        "initial_lower_violations": int(value["initial_lower_violations"]),
        "unresolved_lower_after": int(value["unresolved_lower_after"]),
        "upper_violations_after": int(value["upper_violations_after"]),
        "new_upper_violations": int(value["new_upper_violations"]),
        "immutable_source_upper_grandfather_mask": np.zeros(count, bool),
        **arrays,
    }


def _independent_density_gate(
    contract: Mapping,
    base_water_count: np.ndarray,
    final_water_count: np.ndarray,
    terrain_count: np.ndarray,
    terrain_outer_count: np.ndarray,
    *,
    base_water_xy: np.ndarray | None = None,
    local_reference_water_count: np.ndarray | None = None,
    local_reference_terrain_count: np.ndarray | None = None,
    good_overlap_reference_water_count: np.ndarray | None = None,
    good_overlap_reference_terrain_count: np.ndarray | None = None,
) -> dict:
    target = np.asarray(contract["target_combined_count"], np.float64)
    active = np.asarray(contract["required_mask"], bool)
    _require(np.any(active), "fine density contract has no active centre")
    _require(
        np.all(target[active] > 0),
        "fine density contract contains a zero active combined target",
    )
    _require(
        np.all(np.asarray(contract["target_water_count"])[active] > 0),
        "fine density contract contains a zero active WATER target",
    )
    base_water = np.asarray(base_water_count, np.int64)
    final_water = np.asarray(final_water_count, np.int64)
    terrain = np.asarray(terrain_count, np.int64)
    terrain_outer = np.asarray(terrain_outer_count, np.int64)
    _require(
        all(value.shape == target.shape for value in (
            base_water, final_water, terrain, terrain_outer
        )),
        "independent density recount shape differs from contract",
    )
    if base_water_xy is not None:
        blockers = np.asarray(base_water_xy, np.float64)
        _require(
            blockers.ndim == 2 and blockers.shape[1] == 2 and len(blockers),
            "independent immutable-WATER blocker cloud is invalid",
        )
        _require(
            len(blockers) == int(contract["immutable_water_blocker_count"]),
            "immutable-WATER blocker count differs from independent source collection",
        )
        spacing = float(contract["immutable_water_spacing_m"])
        vacant_representatives = np.asarray(
            contract["vacant_support_representative_xy"], np.float64
        )
        capacity_xy = np.asarray(contract["spacing_capacity_xy"], np.float64)
        vacant_distance = water_pipeline._nearest_distance(
            vacant_representatives, blockers
        )
        capacity_distance = water_pipeline._nearest_distance(capacity_xy, blockers)
        _require(
            np.all(vacant_distance >= spacing - 1e-12),
            "archived vacant support contains an immutable-WATER-blocked proposal",
        )
        _require(
            np.all(capacity_distance >= spacing - 1e-12),
            "archived capacity witness violates immutable-WATER spacing",
        )
    for name, actual in (
        ("local_reference_water_count", local_reference_water_count),
        ("local_reference_terrain_count", local_reference_terrain_count),
        ("good_overlap_reference_water_count", good_overlap_reference_water_count),
        ("good_overlap_reference_terrain_count", good_overlap_reference_terrain_count),
    ):
        if actual is not None:
            _require(
                np.array_equal(np.asarray(actual, np.int64), contract[name]),
                f"fine density {name} differs from independent source recount",
            )
    expected_source_water = base_water > 0
    expected_boundary = water_pipeline._terrain_boundary_centres(
        np.asarray(contract["centres"], np.float64),
        terrain,
        terrain_outer,
        step_m=float(contract["step_m"]),
    )
    expected_source_support = (
        np.asarray(contract["reference_surface_active_mask"], bool)
        | expected_source_water
        | (terrain_outer > 0)
    )
    eligibility_contract = water_pipeline.DensityAuditCentres(
        centres_xy=np.asarray(contract["centres"], np.float64),
        spec_id=tuple(contract["centre_spec_id"]),
        spec_kind=tuple(contract["centre_spec_kind"]),
        spec_label=np.asarray(contract["centre_spec_label"], np.int32),
    )
    expected_required = water_pipeline._density_required_mask(
        eligibility_contract,
        np.asarray(contract["reference_surface_active_mask"], bool),
        np.asarray(contract["fillable_support_active_mask"], bool),
    )
    _require(
        np.array_equal(
            np.asarray(contract["source_water_active_mask"], bool),
            expected_source_water,
        ),
        "fine density source-WATER mask differs from independent base recount",
    )
    _require(
        np.array_equal(
            np.asarray(contract["terrain_boundary_mask"], bool),
            expected_boundary,
        ),
        "fine density terrain-boundary mask differs from independent recount",
    )
    _require(
        np.array_equal(
            np.asarray(contract["source_support_active_mask"], bool),
            expected_source_support,
        ),
        "fine density source-support mask differs from independent recount",
    )
    _require(
        np.array_equal(active, expected_required),
        "fine density required mask differs from independent eligibility recount",
    )
    before = base_water + terrain
    after = final_water + terrain
    before_ratio = np.full(len(target), np.nan, np.float64)
    after_ratio = np.full(len(target), np.nan, np.float64)
    before_ratio[active] = before[active] / target[active]
    after_ratio[active] = after[active] / target[active]
    target_water = np.asarray(contract["target_water_count"], np.float64)
    water_before_ratio = np.full(len(target), np.nan, np.float64)
    water_after_ratio = np.full(len(target), np.nan, np.float64)
    water_before_ratio[active] = base_water[active] / target_water[active]
    water_after_ratio[active] = final_water[active] / target_water[active]
    water_lower = np.asarray(contract["water_lower_count"], np.int64)
    nominal_upper = np.asarray(contract["water_nominal_upper_count"], np.int64)
    allowed_upper = np.asarray(contract["water_upper_count"], np.int64)
    grandfathered = np.zeros(len(target), bool)
    lower_violation = active & (final_water < water_lower)
    strict_upper_violation = active & (final_water > nominal_upper)
    grandfather_increase = np.zeros(len(target), bool)
    new_upper = active & (final_water > allowed_upper)
    added = final_water - base_water
    target_addition = np.asarray(contract["target_addition_count"], np.float64)
    addition_ratio = np.full(len(target), np.nan, np.float64)
    positive_target = active & (target_addition > 0.0)
    addition_ratio[positive_target] = (
        added[positive_target] / target_addition[positive_target]
    )
    terrain_matches = np.array_equal(terrain_count, contract["terrain_count"])
    before_matches = np.array_equal(before, contract["combined_before_count"])
    water_before_matches = np.array_equal(
        base_water, contract["water_before_count"]
    )
    water_after_matches = np.array_equal(
        final_water, contract["water_after_count"]
    )
    by_spec: dict[str, dict] = {}
    spec_id = np.asarray(contract["centre_spec_id"], object)
    spec_kind = np.asarray(contract["centre_spec_kind"], object)
    for identifier in sorted(set(str(value) for value in spec_id)):
        member = active & (spec_id == identifier)
        by_spec[identifier] = {
            "kind": str(spec_kind[np.flatnonzero(spec_id == identifier)[0]]),
            "declared_centres": int(np.count_nonzero(spec_id == identifier)),
            "support_active_centres": int(np.count_nonzero(member)),
            "combined_after_ratio": _quantiles(after_ratio[member]),
            "water_after_ratio": _quantiles(water_after_ratio[member]),
            "unresolved_lower_after": int(np.count_nonzero(lower_violation & member)),
            "new_upper_violations": int(np.count_nonzero(new_upper & member)),
            "grandfathered_upper_centres": int(
                np.count_nonzero(grandfathered & member)
            ),
        }
    return {
        "method": "independent-post-build-water-recount-against-vacant-addition-targets-v3",
        "circle_radius_m": float(contract["radius_m"]),
        "step_m": float(contract["step_m"]),
        "centre_count": int(len(before)),
        "support_active_centres": int(np.count_nonzero(active)),
        "required_count_by_spec": dict(contract["required_count_by_spec"]),
        "candidate_independent_eligibility_recount_matches": True,
        "reference_surface_mask_independently_recounted": True,
        "raw_and_vacant_support_cells_independently_recounted": True,
        "vacant_support_immutable_spacing_independently_recounted": (
            base_water_xy is not None
        ),
        "constructive_capacity_witness_independently_recounted": True,
        "reference_water_and_terrain_counts_independently_recounted": all(
            item is not None
            for item in (
                local_reference_water_count,
                local_reference_terrain_count,
                good_overlap_reference_water_count,
                good_overlap_reference_terrain_count,
            )
        ),
        "source_water_mask_independently_recounted": True,
        "terrain_boundary_mask_independently_recounted": True,
        "minimum_ratio": float(contract["minimum_ratio"]),
        "maximum_ratio": float(contract["maximum_ratio"]),
        "shoreline_centres": int(np.count_nonzero(contract["shoreline_mask"])),
        "shoreline_terrain_count_threshold": float(
            contract["shoreline_terrain_count_threshold"]
        ),
        "reference_kind_counts": {
            name: int(contract["reference_kind"].count(name))
            for name in sorted(set(contract["reference_kind"]))
        },
        "target_combined_count": _count_summary(contract["target_combined_count"][active]),
        "target_water_count": _count_summary(target_water[active]),
        "water_before_count": _count_summary(base_water[active]),
        "water_after_count": _count_summary(final_water[active]),
        "addition_target_count": _count_summary(target_addition[active]),
        "addition_after_count": _count_summary(added[active]),
        "addition_after_ratio": _quantiles(addition_ratio[active]),
        "water_before_ratio": _quantiles(water_before_ratio[active]),
        "water_after_ratio": _quantiles(water_after_ratio[active]),
        "combined_before_count": _count_summary(before[active]),
        "combined_after_count": _count_summary(after[active]),
        "combined_before_ratio": _quantiles(before_ratio[active]),
        "combined_after_ratio": _quantiles(after_ratio[active]),
        "unresolved_lower_after": int(np.count_nonzero(lower_violation)),
        "new_upper_violations": int(np.count_nonzero(new_upper)),
        "strict_nominal_upper_violations": int(
            np.count_nonzero(strict_upper_violation)
        ),
        "immutable_source_upper_grandfathered_centres": int(
            np.count_nonzero(grandfathered)
        ),
        "grandfathered_window_increases": int(
            np.count_nonzero(grandfather_increase)
        ),
        "declared_terrain_count_matches_independent_1mm_recount": bool(terrain_matches),
        "declared_before_count_matches_independent_recount": bool(before_matches),
        "declared_water_before_count_matches_independent_recount": bool(
            water_before_matches
        ),
        "declared_water_after_count_matches_independent_recount": bool(
            water_after_matches
        ),
        "declared_geometry_after_count_difference": _count_summary(
            after - np.asarray(contract["combined_after_count"], np.int64)
        ),
        "per_search_neighbourhood": by_spec,
        "post_build_lower_and_upper_bounds_passed": bool(
            terrain_matches
            and before_matches
            and water_before_matches
            and water_after_matches
            and not np.any(lower_violation)
            and not np.any(new_upper)
            and not np.any(strict_upper_violation)
            and not np.any(grandfather_increase)
        ),
        "water_only_center_count_is_acceptance_criterion": True,
    }


def _verify_upstream_provenance(
    fine_manifest: Mapping,
    *,
    base_water: Path,
    final_water: Path,
    geometry_manifest_path: Path,
    geometry_manifest_document: Mapping,
    archive: Path,
    sand: Path,
    rock: Path,
    config: Path,
) -> None:
    _require(
        fine_manifest.get("operation") == "site1-v11-candidate-only-water-addition-scalar-enrichment",
        "fine manifest has the wrong operation",
    )
    candidate = fine_manifest.get("candidate")
    _require(isinstance(candidate, Mapping), "fine manifest lacks candidate fingerprint")
    _require(str(Path(str(candidate.get("path", ""))).resolve(strict=False)) == str(final_water), "fine manifest candidate path differs")
    _require(candidate.get("sha256") == _sha256(final_water), "fine manifest candidate hash drift")
    _require(int(candidate.get("points", -1)) == density.inspect_fixed_stride_ply(final_water).count, "fine manifest candidate count drift")
    inputs = fine_manifest.get("input_fingerprints")
    _require(isinstance(inputs, Mapping), "fine manifest lacks input fingerprints")
    expected = {
        "base_water": base_water,
        "geometry_manifest": geometry_manifest_path,
        "geometry_archive": archive,
        "sand": sand,
        "rock": rock,
    }
    for name, path in expected.items():
        block = inputs.get(name)
        _require(isinstance(block, Mapping), f"fine manifest lacks {name} fingerprint")
        required = {"path", "size_bytes", "mtime_ns", "sha256"}
        missing = sorted(required - set(block))
        _require(not missing, f"fine manifest {name} fingerprint is incomplete: {', '.join(missing)}")
        _require(
            Path(str(block["path"])).resolve(strict=False) == path,
            f"fine manifest {name} path drift",
        )
        _require(block.get("sha256") == _sha256(path), f"fine manifest {name} hash drift")
        _require(int(block["size_bytes"]) == path.stat().st_size, f"fine manifest {name} size drift")
        _require(int(block["mtime_ns"]) == path.stat().st_mtime_ns, f"fine manifest {name} mtime drift")
    nested_geometry = fine_manifest.get("geometry_manifest")
    _require(isinstance(nested_geometry, Mapping), "fine manifest lacks nested geometry-manifest provenance")
    geometry_hash = _sha256(geometry_manifest_path)
    _require(
        Path(str(nested_geometry.get("path", ""))).resolve(strict=False)
        == geometry_manifest_path,
        "fine nested geometry-manifest path drift",
    )
    _require(nested_geometry.get("sha256") == geometry_hash, "fine nested geometry-manifest hash drift")
    archived_name = nested_geometry.get("archived_copy")
    archived_hash = nested_geometry.get("archived_copy_sha256")
    _require(isinstance(archived_name, str) and Path(archived_name).name == archived_name, "fine nested geometry-manifest copy name is invalid")
    _require(isinstance(archived_hash, str), "fine nested geometry-manifest copy hash is missing")
    archived_copy = _strict_file(
        Path(str(fine_manifest["candidate"]["path"])).parent / archived_name,
        "fine archived geometry-manifest copy",
    )
    _require(_sha256(archived_copy) == archived_hash == geometry_hash, "fine archived geometry-manifest copy differs from source")
    _require(
        fine_manifest.get("density_audit") == geometry_manifest_document.get("density_audit"),
        "fine manifest density audit differs from the source geometry manifest",
    )
    declared_config = fine_manifest.get("config")
    _require(isinstance(declared_config, Mapping), "fine manifest lacks inherited review config fingerprint")
    _require(declared_config.get("sha256") == _sha256(config), "fine manifest review config hash drift")


def build_interface_audit(
    *,
    base_water_path: str | Path,
    final_water_path: str | Path,
    fine_manifest_path: str | Path,
    geometry_manifest_path: str | Path,
    geometry_archive_path: str | Path,
    sand_1mm_path: str | Path,
    rock_1mm_path: str | Path,
    review_config_path: str | Path,
    output_path: str | Path,
    chunk_records: int = 1_000_000,
    edge_sample_limit: int = EDGE_SAMPLE_LIMIT,
) -> dict:
    """Build one immutable, self-hashed, candidate-only interface audit."""

    _require(chunk_records > 0, "chunk_records must be positive")
    _require(edge_sample_limit > 0, "edge_sample_limit must be positive")
    base_water = _strict_file(base_water_path, "base WATER")
    final_water = _strict_file(final_water_path, "final WATER")
    fine_manifest_source, fine_manifest = _load_json(fine_manifest_path, "fine manifest")
    geometry_manifest_source, geometry_manifest = _load_json(
        geometry_manifest_path, "geometry manifest"
    )
    archive = _strict_file(geometry_archive_path, "geometry archive")
    sand = _strict_file(sand_1mm_path, "canonical 1mm SAND")
    rock = _strict_file(rock_1mm_path, "canonical 1mm ROCK")
    config, config_document = _load_json(review_config_path, "v12 review config")
    _verify_upstream_provenance(
        fine_manifest,
        base_water=base_water,
        final_water=final_water,
        geometry_manifest_path=geometry_manifest_source,
        geometry_manifest_document=geometry_manifest,
        archive=archive,
        sand=sand,
        rock=rock,
        config=config,
    )
    append, additions_xy, labels = _append_contract(
        base_water,
        final_water,
        archive,
        geometry_manifest,
        chunk_records=chunk_records,
    )
    specs = water_pipeline.load_circle_specs(config)
    density_contract = _declared_density_contract(
        geometry_manifest, config_document, specs, archive
    )
    parameters = config_document.get("parameters")
    _require(isinstance(parameters, Mapping), "review config lacks parameters")
    density_settings = water_pipeline._density_continuity_settings(parameters)
    good_overlap = water_pipeline._good_overlap_spec(config_document)
    density_specs = tuple(
        item for item in specs if item.kind in MEASURED_DENSITY_KINDS
    )
    # Match the geometry builder's immutable-WATER collection exactly.  This
    # lets the independent audit prove that every archived vacant-support and
    # capacity-witness row clears every blocker that influenced production.
    density_collection_specs = (
        *_expanded_specs(
            density_specs,
            density_settings.local_reference_outer_margin_m
            + density_settings.audit_radius_m
            + density_settings.support_margin_m,
        ),
        *_expanded_specs((good_overlap,), density_settings.audit_radius_m),
    )
    expanded = (
        *_expanded_specs(specs, max(SUPPORT_RADIUS_M, EDGE_CENSOR_M)),
        *density_collection_specs,
    )
    removed_source_indices = _declared_removed_source_indices(
        geometry_manifest,
        source_count=density.inspect_fixed_stride_ply(base_water).count,
    )
    base_xy = _collect_water_xy(
        base_water,
        expanded,
        chunk_records=chunk_records,
        excluded_source_indices=removed_source_indices,
    )
    additions_local = additions_xy[_circle_union_mask(additions_xy, expanded)]
    final_xy = _collect_water_xy(
        final_water, expanded, chunk_records=chunk_records
    )
    base_tree = _spatial_index(base_xy) if len(base_xy) else None
    reference_base_xy = base_xy[
        water_pipeline._surface_contains(density_contract["surface"], base_xy)
    ]
    reference_base_tree = (
        _spatial_index(reference_base_xy) if len(reference_base_xy) else None
    )
    additions_tree = _spatial_index(additions_local) if len(additions_local) else None
    final_tree = _spatial_index(final_xy) if len(final_xy) else None
    centres, slices, _ = _moving_centres(specs)
    local_reference_centres = np.asarray(
        density_contract["local_reference_centres_xy"], np.float64
    )
    overlap_reference_centres = np.asarray(
        density_contract["good_overlap_reference_centres_xy"], np.float64
    )
    audit_centres = np.concatenate((
        centres,
        density_contract["centres"],
        local_reference_centres,
        overlap_reference_centres,
    ), axis=0)
    base_inner_all = _tree_counts(base_tree, audit_centres, EVALUATION_RADIUS_M)
    final_inner_all = _tree_counts(final_tree, audit_centres, EVALUATION_RADIUS_M)
    base_outer_all = _tree_counts(base_tree, audit_centres, SUPPORT_RADIUS_M)
    final_outer_all = _tree_counts(final_tree, audit_centres, SUPPORT_RADIUS_M)
    terrain_inner_all, terrain_outer_all = _stream_terrain_counts(
        (sand, rock),
        specs,
        audit_centres,
        chunk_records=chunk_records,
    )
    review_count = len(centres)
    contract_count = len(density_contract["centres"])
    local_reference_count = len(local_reference_centres)
    contract_begin = review_count
    contract_end = contract_begin + contract_count
    local_reference_end = contract_end + local_reference_count
    base_inner = base_inner_all[:review_count]
    final_inner = final_inner_all[:review_count]
    base_outer = base_outer_all[:review_count]
    final_outer = final_outer_all[:review_count]
    terrain_inner = terrain_inner_all[:review_count]
    terrain_outer = terrain_outer_all[:review_count]
    contract_terrain_inner = terrain_inner_all[contract_begin:contract_end]
    contract_terrain_outer = terrain_outer_all[contract_begin:contract_end]
    independent_reference_water_count = _tree_counts(
        reference_base_tree,
        np.concatenate((local_reference_centres, overlap_reference_centres)),
        EVALUATION_RADIUS_M,
    )
    independent_local_water_count = independent_reference_water_count[
        :local_reference_count
    ]
    independent_overlap_water_count = independent_reference_water_count[
        local_reference_count:
    ]
    independent_local_terrain_count = terrain_inner_all[
        contract_end:local_reference_end
    ]
    independent_overlap_terrain_count = terrain_inner_all[local_reference_end:]
    boundary_centres = _edge_seed_centres(
        density_contract,
        contract_terrain_inner,
        contract_terrain_outer,
    )
    terrain = _stream_terrain_edges(
        (sand, rock),
        specs,
        boundary_centres,
        base_tree,
        final_tree,
        chunk_records=chunk_records,
        edge_sample_limit=edge_sample_limit,
    )
    density_gate = _independent_density_gate(
        density_contract,
        base_inner_all[contract_begin:contract_end],
        final_inner_all[contract_begin:contract_end],
        contract_terrain_inner,
        contract_terrain_outer,
        base_water_xy=base_xy,
        local_reference_water_count=independent_local_water_count,
        local_reference_terrain_count=independent_local_terrain_count,
        good_overlap_reference_water_count=independent_overlap_water_count,
        good_overlap_reference_terrain_count=independent_overlap_terrain_count,
    )
    regions, moving = _region_metrics(
        specs,
        slices,
        base_xy,
        additions_xy,
        labels,
        base_inner,
        final_inner,
        terrain_inner,
        base_outer,
        final_outer,
        terrain_outer,
    )
    fade = _fade_audit(specs, additions_tree, final_tree)
    checks = {
        "append_contract": True,
        "terrain_edge_eligibility_is_candidate_independent": (
            terrain["eligibility"]["final_water_used_for_eligibility"] is False
            and terrain["eligible_edge_points"] > 0
        ),
        "terrain_edge_meaningful_configured_support_continuity_passed": terrain[
            "meaningful_edge_continuity_passed"
        ],
        "measured_density_lower_and_upper_bounds_passed": density_gate["post_build_lower_and_upper_bounds_passed"],
    }
    passed = bool(all(checks.values()))
    inputs = {
        "base_water": _fingerprint(base_water, ply=True),
        "final_water": _fingerprint(final_water, ply=True),
        "fine_manifest": _fingerprint(fine_manifest_source),
        "geometry_manifest": _fingerprint(geometry_manifest_source),
        "geometry_archive": _fingerprint(archive),
        "sand_1mm": _fingerprint(sand, ply=True),
        "rock_1mm": _fingerprint(rock, ply=True),
        "review_config": _fingerprint(config),
    }
    implementations = {
        Path(__file__).name: _fingerprint(Path(__file__)),
        Path(density.__file__).name: _fingerprint(Path(density.__file__)),
        Path(scalar_enrichment.__file__).name: _fingerprint(
            Path(scalar_enrichment.__file__)
        ),
        Path(water_pipeline.__file__).name: _fingerprint(Path(water_pipeline.__file__)),
    }
    document = {
        "schema_version": 1,
        "operation": OPERATION,
        "created": dt.datetime.now().isoformat(timespec="seconds"),
        "status": "passed" if passed else "failed",
        "candidate_only": True,
        "canonical_writes": False,
        "annotations_are_search_neighbourhoods_not_masks": True,
        "terrain_resolution": {
            "selected": "canonical-1mm-SAND-plus-ROCK",
            "spacing_m": 0.001,
            "coarse_5mm_used": False,
            "reason": "The fine WATER/terrain edge contract requires native 1 mm occupancy and nearest-distance evidence; 5 mm terrain would erase the interface gaps being audited.",
        },
        "inputs": inputs,
        "implementations": implementations,
        "append_contract": append,
        "metrics": {
            "regions": regions,
            "moving_circle_aggregate": moving,
            "terrain_edge_nearest_water": terrain,
            "density_continuity_lower_and_upper_gate": density_gate,
            "density_dip_maximum_ratios": {
                "covered_by": "density_continuity_lower_and_upper_gate",
                "maximum_ratio": density_gate["maximum_ratio"],
                "new_upper_violations": density_gate["new_upper_violations"],
            },
            "clustered_fade_spatial_variation": fade,
        },
        "acceptance": {
            "checks": checks,
            "passed": passed,
            "water_only_center_count_is_acceptance_criterion": True,
            "legacy_zero_or_nonzero_window_checks_are_diagnostic_only": True,
            "clustered_fade_square_discontinuity_is_advisory": True,
        },
    }
    document["manifest_lock"] = {
        "method": "sha256-canonical-json-excluding-manifest_lock",
        "sha256": _canonical_document_hash(document),
    }
    destination = _atomic_json(output_path, document)
    return {
        "built": True,
        "verified": passed,
        "status": document["status"],
        "manifest": str(destination),
        "manifest_sha256": _sha256(destination),
        "acceptance": document["acceptance"],
    }


def verify_interface_audit(
    *,
    manifest_path: str | Path,
    base_water_path: str | Path,
    final_water_path: str | Path,
    fine_manifest_path: str | Path,
    geometry_manifest_path: str | Path,
    geometry_archive_path: str | Path,
    sand_1mm_path: str | Path,
    rock_1mm_path: str | Path,
    review_config_path: str | Path,
) -> dict:
    """Verify the self-lock, every input hash, and the passed audit state."""

    source, document = _load_json(manifest_path, "v12 interface audit manifest")
    _require(document.get("operation") == OPERATION, "unexpected interface audit operation")
    lock = document.get("manifest_lock")
    _require(isinstance(lock, Mapping), "interface audit lacks manifest lock")
    _require(lock.get("method") == "sha256-canonical-json-excluding-manifest_lock", "unexpected interface audit lock method")
    _require(lock.get("sha256") == _canonical_document_hash(document), "interface audit manifest lock mismatch")
    inputs = document.get("inputs")
    _require(isinstance(inputs, Mapping), "interface audit lacks inputs")
    expected = {
        "base_water": base_water_path,
        "final_water": final_water_path,
        "fine_manifest": fine_manifest_path,
        "geometry_manifest": geometry_manifest_path,
        "geometry_archive": geometry_archive_path,
        "sand_1mm": sand_1mm_path,
        "rock_1mm": rock_1mm_path,
        "review_config": review_config_path,
    }
    _require(set(inputs) == set(expected), "interface audit input set differs")
    for name, path in expected.items():
        block = inputs[name]
        _require(isinstance(block, Mapping), f"interface audit input fingerprint is invalid: {name}")
        _assert_fingerprint(block, path, f"interface audit input {name}")
    implementations = document.get("implementations")
    implementation_paths = {
        Path(__file__).name: Path(__file__),
        Path(density.__file__).name: Path(density.__file__),
        Path(scalar_enrichment.__file__).name: Path(scalar_enrichment.__file__),
        Path(water_pipeline.__file__).name: Path(water_pipeline.__file__),
    }
    _require(isinstance(implementations, Mapping) and set(implementations) == set(implementation_paths), "interface audit implementation set differs")
    for name, path in implementation_paths.items():
        block = implementations[name]
        _require(isinstance(block, Mapping), f"invalid implementation fingerprint: {name}")
        _assert_fingerprint(block, path, f"interface audit implementation {name}")
    acceptance = document.get("acceptance")
    _require(isinstance(acceptance, Mapping), "interface audit lacks acceptance")
    _require(document.get("status") == "passed" and acceptance.get("passed") is True, "interface audit did not pass")
    checks = acceptance.get("checks")
    _require(isinstance(checks, Mapping) and checks and all(value is True for value in checks.values()), "interface audit acceptance checks are incomplete")
    _require(
        acceptance.get("water_only_center_count_is_acceptance_criterion") is True,
        "interface audit does not directly gate required WATER support",
    )
    return {
        "verified": True,
        "status": "passed",
        "manifest": str(source),
        "manifest_sha256": _sha256(source),
        "acceptance": dict(acceptance),
    }


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    result.add_argument("action", choices=("build", "verify"))
    result.add_argument("--base-water", required=True, type=Path)
    result.add_argument("--final-water", required=True, type=Path)
    result.add_argument("--fine-manifest", required=True, type=Path)
    result.add_argument("--geometry-manifest", required=True, type=Path)
    result.add_argument("--geometry-archive", required=True, type=Path)
    result.add_argument("--sand-1mm", required=True, type=Path)
    result.add_argument("--rock-1mm", required=True, type=Path)
    result.add_argument("--review-config", required=True, type=Path)
    result.add_argument("--output", required=True, type=Path)
    result.add_argument("--chunk-records", type=int, default=1_000_000)
    result.add_argument("--edge-sample-limit", type=int, default=EDGE_SAMPLE_LIMIT)
    return result


def main(argv: Sequence[str] | None = None) -> int:
    args = parser().parse_args(argv)
    common = dict(
        base_water_path=args.base_water,
        final_water_path=args.final_water,
        fine_manifest_path=args.fine_manifest,
        geometry_manifest_path=args.geometry_manifest,
        geometry_archive_path=args.geometry_archive,
        sand_1mm_path=args.sand_1mm,
        rock_1mm_path=args.rock_1mm,
        review_config_path=args.review_config,
    )
    if args.action == "build":
        result = build_interface_audit(
            **common,
            output_path=args.output,
            chunk_records=args.chunk_records,
            edge_sample_limit=args.edge_sample_limit,
        )
    else:
        result = verify_interface_audit(manifest_path=args.output, **common)
    print(json.dumps(result, indent=2, sort_keys=True), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
