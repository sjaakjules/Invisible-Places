#!/usr/bin/env python3
"""Direct scalar-coverage audit for the Scene1 v12 coarse WATER cloud.

The fine scalar-enrichment implementation is already hash-locked by the fine
and interface stages, so this v12-only module owns the release-specific coarse
audit.  Exact fine-addition survivors remain the primary evidence.  A
component with no exact survivor may instead be represented only when every
fine row in that component has a deterministic coarse-record proxy strictly
within 5 mm in full 3D and the unique proxy records pass the same scalar-field
coverage and range checks.
"""

from __future__ import annotations

from pathlib import Path
from typing import Mapping

import numpy as np

import site1_v11_terrain as terrain
import site1_v11_water_scalar_enrichment as scalar_enrichment


IMPLEMENTATION_PATH = Path(__file__).resolve()
_NUMPY_WORKING_BUDGET_BYTES = 64 * 1024 * 1024


def _implementation_fingerprint() -> Mapping[str, str]:
    return {
        "path": str(IMPLEMENTATION_PATH),
        "sha256": scalar_enrichment.sha256_path(IMPLEMENTATION_PATH),
    }


def _nearest_coarse_record_proxies(
    coarse_records: np.ndarray,
    query_records: np.ndarray,
    *,
    maximum_distance_m: float,
    chunk_records: int,
) -> tuple[np.ndarray, np.ndarray]:
    """Return deterministic nearest coarse rows for fine component rows.

    Distances use the full XYZ geometry used by CleanMesh's spatial
    downsampler.  The nearest record is selected by exact squared distance,
    then by the smallest coarse source index.  Coarse chunks only bound
    memory; they cannot affect the result.
    """

    limit = float(maximum_distance_m)
    if not np.isfinite(limit) or limit <= 0.0:
        raise ValueError("maximum proxy distance must be positive and finite")
    if chunk_records <= 0:
        raise ValueError("chunk_records must be positive")
    query_xyz = np.column_stack(
        (query_records["x"], query_records["y"], query_records["z"])
    ).astype(np.float64, copy=False)
    if query_xyz.shape != (len(query_records), 3) or not np.all(
        np.isfinite(query_xyz)
    ):
        raise RuntimeError("fine component proxy queries contain non-finite XYZ")
    if not len(query_xyz):
        return np.empty(0, np.int64), np.empty(0, np.float64)

    best_squared = np.full(len(query_xyz), np.inf, np.float64)
    best_index = np.full(len(query_xyz), -1, np.int64)
    try:
        from scipy.spatial import cKDTree
    except ModuleNotFoundError:
        cKDTree = None

    # NumPy is the production fallback.  Cap a coarse block so even one query
    # fits the same approximate 64 MiB delta/square/reduction working budget.
    # Query batches below are then sized from the actual coarse-block length.
    float_bytes = np.dtype(np.float64).itemsize
    working_bytes_per_pair = 2 * 3 * float_bytes + float_bytes
    effective_chunk_records = int(chunk_records)
    if cKDTree is None:
        effective_chunk_records = min(
            effective_chunk_records,
            max(1, _NUMPY_WORKING_BUDGET_BYTES // working_bytes_per_pair),
        )

    for start in range(0, len(coarse_records), effective_chunk_records):
        stop = min(start + effective_chunk_records, len(coarse_records))
        block = coarse_records[start:stop]
        block_xyz = np.column_stack(
            (block["x"], block["y"], block["z"])
        ).astype(np.float64, copy=False)
        if not np.all(np.isfinite(block_xyz)):
            raise RuntimeError("5mm candidate contains non-finite XYZ")
        if not len(block_xyz):
            continue

        if cKDTree is None:
            local_squared = np.full(len(query_xyz), np.inf, np.float64)
            local_index = np.full(len(query_xyz), -1, np.int64)
            working_bytes_per_query = len(block_xyz) * working_bytes_per_pair
            query_batch = max(
                1,
                min(
                    len(query_xyz),
                    _NUMPY_WORKING_BUDGET_BYTES // working_bytes_per_query,
                ),
            )
            for query_start in range(0, len(query_xyz), query_batch):
                query_stop = min(query_start + query_batch, len(query_xyz))
                delta = (
                    query_xyz[query_start:query_stop, None, :]
                    - block_xyz[None, :, :]
                )
                squared = np.sum(np.square(delta), axis=2)
                selected = np.argmin(squared, axis=1)
                rows = np.arange(query_stop - query_start)
                local_squared[query_start:query_stop] = squared[rows, selected]
                local_index[query_start:query_stop] = selected
        else:
            tree = cKDTree(block_xyz)
            neighbours = min(2, len(block_xyz))
            _, queried_index = tree.query(
                query_xyz,
                k=neighbours,
                workers=1,
            )
            queried_index = np.asarray(queried_index, np.int64)
            if neighbours == 1:
                queried_index = queried_index[:, None]
            first_index = queried_index[:, 0]
            first_delta = query_xyz - block_xyz[first_index]
            local_squared = np.sum(np.square(first_delta), axis=1)
            local_index = first_index.copy()

            # cKDTree does not promise an index tie-break.  Detect exact
            # nearest-distance ties with k=2, then enumerate just those rare
            # balls and choose the smallest exact-distance source index.
            if neighbours == 2:
                second_index = queried_index[:, 1]
                second_delta = query_xyz - block_xyz[second_index]
                second_squared = np.sum(np.square(second_delta), axis=1)
                tied_rows = np.flatnonzero(second_squared == local_squared)
                for row in tied_rows:
                    radius = np.nextafter(
                        float(np.sqrt(local_squared[row])), np.inf
                    )
                    members = np.asarray(
                        tree.query_ball_point(query_xyz[row], radius),
                        np.int64,
                    )
                    member_delta = block_xyz[members] - query_xyz[row]
                    member_squared = np.sum(np.square(member_delta), axis=1)
                    exact_minimum = float(np.min(member_squared))
                    exact_members = members[member_squared == exact_minimum]
                    local_squared[row] = exact_minimum
                    local_index[row] = int(np.min(exact_members))

        global_index = local_index + int(start)
        better = local_squared < best_squared
        tied_better_index = (
            (local_squared == best_squared)
            & ((best_index < 0) | (global_index < best_index))
        )
        update = better | tied_better_index
        best_squared[update] = local_squared[update]
        best_index[update] = global_index[update]

    limit_squared = limit * limit
    uncovered = (best_index < 0) | ~np.isfinite(best_squared) | (
        best_squared >= limit_squared
    )
    if np.any(uncovered):
        raise RuntimeError(
            "5mm candidate lacks a strictly <"
            f"{limit:.12g}m full-3D coarse proxy for "
            f"{int(np.count_nonzero(uncovered))} omitted-component fine rows"
        )
    return best_index, np.sqrt(best_squared)


def verify_coarse_exact_subset_component_scalar_coverage(
    fine_candidate_path: str | Path,
    coarse_candidate_path: str | Path,
    *,
    fine_base_points: int,
    fine_component_labels: np.ndarray,
    chunk_records: int = 250_000,
    maximum_proxy_distance_m: float = 0.005,
) -> Mapping[str, object]:
    """Audit exact 5 mm additions and covered fine-component proxies."""

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
    fields = scalar_enrichment._geometry_fields(fine_layout.dtype)
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
        matched_records = (
            np.concatenate(matched_record_parts)
            if matched_record_parts
            else np.empty(0, fine_layout.dtype)
        )
        matched_labels = (
            np.concatenate(matched_label_parts)
            if matched_label_parts
            else np.empty(0, np.int64)
        )
        matched_indices = (
            np.concatenate(matched_index_parts)
            if matched_index_parts
            else np.empty(0, np.int64)
        )
        if len(np.unique(matched_indices)) != len(matched_indices):
            raise RuntimeError("5mm candidate repeats an exact fine addition record")
        fine_labels_present = set(
            int(value) for value in np.unique(labels.astype(np.int64, copy=False))
        )
        exact_labels_present = set(int(value) for value in np.unique(matched_labels))
        missing_labels = sorted(fine_labels_present - exact_labels_present)
        proxy_record_parts: list[np.ndarray] = []
        proxy_label_parts: list[np.ndarray] = []
        proxy_fine_indices = np.empty(0, np.int64)
        proxy_coarse_indices = np.empty(0, np.int64)
        proxy_distances = np.empty(0, np.float64)
        proxy_rows_by_label: dict[int, int] = {}
        proxy_maximum_by_label: dict[int, float] = {}
        if missing_labels:
            missing_mask = np.isin(labels, np.asarray(missing_labels, np.int64))
            proxy_fine_indices = np.flatnonzero(missing_mask).astype(np.int64)
            proxy_queries = np.asarray(fine_suffix[missing_mask])
            try:
                proxy_coarse_indices, proxy_distances = (
                    _nearest_coarse_record_proxies(
                        coarse_memory,
                        proxy_queries,
                        maximum_distance_m=maximum_proxy_distance_m,
                        chunk_records=chunk_records,
                    )
                )
            except RuntimeError as error:
                raise RuntimeError(
                    "5mm candidate omits exact fine component labels "
                    f"{missing_labels} without complete spatial proxy coverage: "
                    f"{error}"
                ) from error
            query_labels = labels[proxy_fine_indices].astype(np.int64, copy=False)
            for label in missing_labels:
                local = query_labels == label
                unique_proxy_indices = np.unique(proxy_coarse_indices[local])
                proxy_records = np.asarray(
                    coarse_memory[unique_proxy_indices]
                ).copy()
                proxy_record_parts.append(proxy_records)
                proxy_label_parts.append(
                    np.full(len(proxy_records), label, np.int64)
                )
                proxy_rows_by_label[label] = int(len(unique_proxy_indices))
                proxy_maximum_by_label[label] = float(
                    np.max(proxy_distances[local])
                )

        coverage_records = [matched_records, *proxy_record_parts]
        coverage_labels = [matched_labels, *proxy_label_parts]
        coverage = scalar_enrichment._require_component_field_coverage(
            np.concatenate(coverage_records),
            np.concatenate(coverage_labels),
            fields,
            context="active 5mm exact/proxy fine-component representation",
        )

        fine_counts = {
            int(label): int(count)
            for label, count in zip(*np.unique(labels, return_counts=True))
        }
        exact_counts = {
            int(label): int(count)
            for label, count in zip(*np.unique(matched_labels, return_counts=True))
        }
        component_representation = []
        for label in sorted(fine_labels_present):
            exact_count = exact_counts.get(label, 0)
            component_representation.append(
                {
                    "component_label": label,
                    "fine_rows": fine_counts[label],
                    "representation": (
                        "exact-fine-addition-records"
                        if exact_count
                        else "strict-full-3d-coarse-proxies"
                    ),
                    "exact_coarse_addition_rows": exact_count,
                    "proxy_fine_rows": (
                        fine_counts[label] if label in missing_labels else 0
                    ),
                    "unique_proxy_coarse_rows": proxy_rows_by_label.get(label, 0),
                    "maximum_proxy_distance_m": proxy_maximum_by_label.get(label),
                }
            )
        proxy_assignment_dtype = np.dtype(
            [
                ("fine_index", "<i8"),
                ("component_label", "<i8"),
                ("coarse_index", "<i8"),
                ("distance_m", "<f8"),
            ]
        )
        proxy_assignment = np.empty(len(proxy_fine_indices), proxy_assignment_dtype)
        proxy_assignment["fine_index"] = proxy_fine_indices
        proxy_assignment["component_label"] = labels[proxy_fine_indices]
        proxy_assignment["coarse_index"] = proxy_coarse_indices
        proxy_assignment["distance_m"] = proxy_distances
    finally:
        del fine_memory
        del coarse_memory
    return {
        "method": "exact-addition-or-strict-3d-coarse-proxy-scalar-audit-v2",
        "implementation": dict(_implementation_fingerprint()),
        "fine_candidate_path": str(fine_layout.path),
        "coarse_candidate_path": str(coarse_layout.path),
        "fine_candidate_points": int(fine_layout.vertex_count),
        "coarse_candidate_points": int(coarse_layout.vertex_count),
        "fine_base_points": prefix,
        "fine_addition_count": int(fine_addition_count),
        "matched_coarse_addition_count": int(len(matched_records)),
        "fine_component_labels": sorted(fine_labels_present),
        "coarse_component_labels": sorted(fine_labels_present),
        "exact_component_labels": sorted(exact_labels_present),
        "proxy_component_labels": missing_labels,
        "exact_component_count": int(len(exact_labels_present)),
        "proxy_component_count": int(len(missing_labels)),
        "proxy_fine_row_count": int(len(proxy_fine_indices)),
        "unique_proxy_record_count": int(sum(proxy_rows_by_label.values())),
        "maximum_proxy_distance_m": (
            float(np.max(proxy_distances)) if len(proxy_distances) else None
        ),
        "proxy_distance_limit_m": float(maximum_proxy_distance_m),
        "proxy_distance_contract": "strictly-less-than-full-3d",
        "deterministic_nearest_proxy_tie_break": (
            "minimum-exact-squared-distance-then-smallest-coarse-index"
        ),
        "proxy_fine_index_space": "zero-based-fine-addition-suffix-local",
        "proxy_assignment_sha256": scalar_enrichment._records_sha256(
            proxy_assignment
        ),
        "component_representation": component_representation,
        "component_label_sha256": scalar_enrichment._records_sha256(
            labels.astype(np.int64, copy=False)
        ),
        "geometry_fields": list(fields),
        "coverage": coverage,
        "full_record_membership_exact": True,
        "proxy_records_are_exact_coarse_records": True,
        "every_omitted_component_fine_row_has_strict_proxy": True,
        "every_fine_component_represented": True,
        "all_required_fields_accepted": True,
    }
