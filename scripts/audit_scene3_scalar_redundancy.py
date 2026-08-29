#!/usr/bin/env python3
"""Audit authored usage and sampled redundancy of Scene3 scalar fields."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np

from prune_ply_scalar_fields import inspect_ply


TARGETS = {
    "Interest",
    "Ranges",
    "A_R_Shelter_Lower",
    "A_R_RainExposure_Lower",
    "A_R_SVF_Lower",
    "A_R_Downhill_X",
    "A_R_Downhill_Y",
    "A_R_Downhill_Z",
    "A_R_DownhillMagnitude",
    "A_R_Horizontalness",
    "A_R_Slope_deg",
}
MEMORY_PATH_PARTS = {
    "field_bounds_memory",
    "field_visual_memory",
    "timing_scalar_bounds_stores",
}


def _walk(value: Any, path: tuple[str, ...] = ()):
    if isinstance(value, dict):
        for key, child in value.items():
            yield from _walk(child, path + (str(key),))
    elif isinstance(value, list):
        for index, child in enumerate(value):
            yield from _walk(child, path + (str(index),))
    elif isinstance(value, str):
        yield path, value


def audit_usage(project: Path, animations: Path) -> dict[str, Any]:
    active = {field: [] for field in TARGETS}
    remembered = {field: [] for field in TARGETS}
    document = json.loads(project.read_text())
    for path, value in _walk(document):
        if value not in TARGETS:
            continue
        destination = (
            remembered
            if MEMORY_PATH_PARTS.intersection(path)
            else active
        )
        destination[value].append(".".join(path))
    animation_hits = {field: [] for field in TARGETS}
    for animation in sorted(animations.glob("*.ipanim.json")):
        try:
            content = json.loads(animation.read_text())
        except (OSError, json.JSONDecodeError):
            continue
        strings = {value for _, value in _walk(content)}
        for field in TARGETS.intersection(strings):
            animation_hits[field].append(animation.name)
    return {
        field: {
            "active_project_reference_count": len(active[field]),
            "remembered_ui_reference_count": len(remembered[field]),
            "animation_files": animation_hits[field],
            "active_paths": active[field],
        }
        for field in sorted(TARGETS)
    }


def _finite_pair(left: np.ndarray, right: np.ndarray):
    valid = np.isfinite(left) & np.isfinite(right)
    return left[valid].astype(np.float64), right[valid].astype(np.float64)


def _comparison(left: np.ndarray, right: np.ndarray) -> dict[str, Any]:
    left, right = _finite_pair(left, right)
    if not len(left):
        return {"count": 0}
    difference = np.abs(left - right)
    correlation = (
        float(np.corrcoef(left, right)[0, 1])
        if len(left) > 1 and np.std(left) > 0 and np.std(right) > 0
        else None
    )
    return {
        "count": int(len(left)),
        "correlation": correlation,
        "absolute_error_median": float(np.median(difference)),
        "absolute_error_p99": float(np.quantile(difference, 0.99)),
        "absolute_error_maximum": float(np.max(difference)),
    }


def audit_source(path: Path, maximum_samples: int) -> dict[str, Any]:
    layout = inspect_ply(path)
    data = np.memmap(
        path,
        mode="r",
        dtype=layout.dtype,
        offset=len(layout.header_bytes),
        shape=(layout.vertex_count,),
    )
    sample_count = min(maximum_samples, layout.vertex_count)
    # A handful of contiguous windows samples the whole source without
    # turning a multi-GiB cloud into hundreds of thousands of random reads.
    window_count = min(8, max(1, sample_count))
    base_window_size = sample_count // window_count
    extra = sample_count % window_count
    chunks: list[np.ndarray] = []
    for window in range(window_count):
        count = base_window_size + (1 if window < extra else 0)
        maximum_start = max(0, layout.vertex_count - count)
        start = (
            0
            if window_count == 1
            else (maximum_start * window) // (window_count - 1)
        )
        chunks.append(np.asarray(data[start : start + count]).copy())
    names = set(layout.dtype.names or ())
    sampled: dict[str, np.ndarray] = {}
    for field in TARGETS:
        disk_name = "scalar_" + field
        if disk_name in names:
            sampled[field] = np.concatenate(
                [chunk[disk_name] for chunk in chunks]
            )
    result: dict[str, Any] = {
        "path": str(path),
        "point_count": layout.vertex_count,
        "record_size": layout.dtype.itemsize,
        "sample_count": sample_count,
        "present_target_fields": sorted(sampled),
        "comparisons": {},
        "sampled_ranges": {
            field: {
                "minimum": float(np.nanmin(values)),
                "maximum": float(np.nanmax(values)),
            }
            for field, values in sampled.items()
        },
    }
    comparisons = result["comparisons"]
    if "A_R_Shelter_Lower" in sampled and "A_R_SVF_Lower" in sampled:
        comparisons["shelter_vs_svf"] = _comparison(
            sampled["A_R_Shelter_Lower"], sampled["A_R_SVF_Lower"]
        )
        comparisons["shelter_vs_one_minus_svf"] = _comparison(
            sampled["A_R_Shelter_Lower"],
            1.0 - sampled["A_R_SVF_Lower"],
        )
    if "A_R_Shelter_Lower" in sampled and "A_R_RainExposure_Lower" in sampled:
        comparisons["shelter_vs_rain_exposure"] = _comparison(
            sampled["A_R_Shelter_Lower"],
            sampled["A_R_RainExposure_Lower"],
        )
        comparisons["shelter_vs_one_minus_rain_exposure"] = _comparison(
            sampled["A_R_Shelter_Lower"],
            1.0 - sampled["A_R_RainExposure_Lower"],
        )

    if "A_R_Horizontalness" in sampled and "A_R_Slope_deg" in sampled:
        horizontalness = sampled["A_R_Horizontalness"]
        comparisons["slope_vs_acos_horizontalness"] = _comparison(
            sampled["A_R_Slope_deg"],
            np.degrees(np.arccos(np.clip(horizontalness, -1.0, 1.0))),
        )
        comparisons["slope_vs_acos_abs_horizontalness"] = _comparison(
            sampled["A_R_Slope_deg"],
            np.degrees(np.arccos(np.clip(np.abs(horizontalness), 0.0, 1.0))),
        )
    if "A_R_DownhillMagnitude" in sampled and "A_R_Slope_deg" in sampled:
        slope_radians = np.radians(sampled["A_R_Slope_deg"])
        comparisons["downhill_magnitude_vs_tan_slope"] = _comparison(
            sampled["A_R_DownhillMagnitude"], np.tan(slope_radians)
        )
        comparisons["slope_vs_atan_downhill_magnitude"] = _comparison(
            sampled["A_R_Slope_deg"],
            np.degrees(np.arctan(sampled["A_R_DownhillMagnitude"])),
        )
        comparisons["downhill_magnitude_vs_sin_slope"] = _comparison(
            sampled["A_R_DownhillMagnitude"], np.sin(slope_radians)
        )
    downhill_components = {
        field: sampled[field]
        for field in ("A_R_Downhill_X", "A_R_Downhill_Y", "A_R_Downhill_Z")
        if field in sampled
    }
    if len(downhill_components) == 3 and "A_R_DownhillMagnitude" in sampled:
        vector_magnitude = np.sqrt(
            sum(np.square(values) for values in downhill_components.values())
        )
        comparisons["downhill_magnitude_vs_xyz_length"] = _comparison(
            sampled["A_R_DownhillMagnitude"], vector_magnitude
        )
        horizontal_length = np.sqrt(
            np.square(downhill_components["A_R_Downhill_X"])
            + np.square(downhill_components["A_R_Downhill_Y"])
        )
        comparisons["downhill_magnitude_vs_xy_length"] = _comparison(
            sampled["A_R_DownhillMagnitude"], horizontal_length
        )

    if {"nx", "ny", "nz"}.issubset(names):
        normals = np.column_stack(
            [
                np.concatenate([chunk[axis] for chunk in chunks]).astype(
                    np.float64
                )
                for axis in ("nx", "ny", "nz")
            ]
        )
        lengths = np.linalg.norm(normals, axis=1)
        valid = np.isfinite(normals).all(axis=1) & (lengths > 1.0e-8)
        normal = np.zeros_like(normals)
        normal[valid] = normals[valid] / lengths[valid, None]
        normal[normal[:, 2] < 0.0] *= -1.0
        horizontal = np.linalg.norm(normal[:, :2], axis=1)
        nz = np.clip(normal[:, 2], 1.0e-5, 1.0)
        direction_x = np.divide(
            normal[:, 0], horizontal, out=np.zeros(sample_count), where=horizontal > 1.0e-7
        )
        direction_y = np.divide(
            normal[:, 1], horizontal, out=np.zeros(sample_count), where=horizontal > 1.0e-7
        )
        derived = {
            "A_R_Downhill_X": direction_x,
            "A_R_Downhill_Y": direction_y,
            "A_R_Downhill_Z": np.zeros(sample_count),
            "A_R_DownhillMagnitude": horizontal / nz,
            "A_R_Horizontalness": nz,
            "A_R_Slope_deg": np.degrees(np.arccos(nz)),
        }
        for field, expected in derived.items():
            if field in sampled:
                comparisons[field + "_vs_normal_derivation"] = _comparison(
                    sampled[field][valid], expected[valid]
                )
    del data
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--project", type=Path, required=True)
    parser.add_argument("--animations", type=Path, required=True)
    parser.add_argument("--source", type=Path, action="append", required=True)
    parser.add_argument("--maximum-samples", type=int, default=250_000)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = {
        "schema_version": 1,
        "project": str(args.project),
        "usage": audit_usage(args.project, args.animations),
        "sources": [audit_source(path, args.maximum_samples) for path in args.source],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
