#!/usr/bin/env python3
"""Rebuild Scene1/Fossils WATER and route terrain support into SAND/ROCK.

Version 7 treats the cleaned SAND/ROCK clouds as immutable measured prefixes.
It appends conservative ScanID 9 density support to their 1 mm and 5 mm
variants, while rebuilding WATER from cavity evidence only.  The build is
staged, audited, hash-locked, and reversible before anything is installed.
"""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import io
import json
import math
import os
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator

import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))
import rebuild_site1_fossils_water as v6  # noqa: E402


ROOT = SCRIPT_DIR.parent
DEFAULT_DATA = ROOT / "Data/Scene1"
DEFAULT_CONFIG = SCRIPT_DIR / "config/site1_fossils_v7_regions.json"
DEFAULT_RUN = DEFAULT_DATA / "PatchRefinement/20260826-fossils-v7-mixed"
LEGACY_CONFIG = SCRIPT_DIR / "config/site1_fossils_reconstruction_regions.json"

CHUNK = 2_000_000
TERRAIN_SCAN_ID = 9.0
WATER_SCAN_ID = 999.0
RIPPLE_RMS_M = v6.RIPPLE_RMS_M

GEOMETRY_FIELDS = (
    "scalar_A_R_MeanCurvature_Fine",
    "scalar_A_R_MeanCurvature_Medium",
    "scalar_A_R_MeanCurvature_Broad",
    "scalar_A_R_MeanCurvature_Combined",
    "scalar_A_R_CrossCurvature_Fine",
    "scalar_A_R_CrossCurvature_Medium",
    "scalar_A_R_CrossCurvature_Broad",
    "scalar_A_R_CrossCurvature_Combined",
    "scalar_A_R_Recession_Fine",
    "scalar_A_R_Recession_Medium",
    "scalar_A_R_Recession_Broad",
    "scalar_A_R_Recession_Combined",
    "scalar_A_R_Roughness_Fine",
    "scalar_A_R_Roughness_Medium",
    "scalar_A_R_Roughness_Broad",
    "scalar_A_R_Roughness_Combined",
    "scalar_A_R_RoughnessRelative_FineMedium",
)
ENVIRONMENT_FIELDS = (
    "scalar_A_R_Shelter_Lower",
    "scalar_A_R_RainExposure_Lower",
    "scalar_A_R_SVF_Lower",
)


def cloud_path(data_dir: Path, role: str, spacing: str) -> Path:
    return data_dir / f"Site1-{role}-{spacing}.ply"


def patch_header_count(header: bytes, count: int) -> bytes:
    pattern = re.compile(rb"(?m)^(element vertex )(\d+)([ \t]*)(\r?)$")
    match = pattern.search(header)
    if match is None:
        raise RuntimeError("could not patch PLY vertex count")
    digits = str(int(count)).encode("ascii")
    width = len(match.group(2)) + len(match.group(3))
    if len(digits) > width:
        raise RuntimeError(f"PLY count {count} exceeds the reserved header width")
    replacement = match.group(1) + digits + b" " * (width - len(digits)) + match.group(4)
    result = header[:match.start()] + replacement + header[match.end():]
    if len(result) != len(header):
        raise RuntimeError("PLY count patch changed header length")
    return result


def payload_sha256(path: Path) -> str:
    _, count, offset, _ = v6.read_ply_header(path)
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        stream.seek(offset)
        remaining = path.stat().st_size - offset
        while remaining:
            block = stream.read(min(32 * 1024 * 1024, remaining))
            if not block:
                break
            digest.update(block)
            remaining -= len(block)
    if remaining or count < 0:
        raise RuntimeError(f"truncated PLY payload in {path}")
    return digest.hexdigest()


def source_fingerprint(path: Path) -> dict:
    dtype, count, offset, _ = v6.read_ply_header(path)
    stat = path.stat()
    expected = offset + count * dtype.itemsize
    if stat.st_size != expected:
        raise RuntimeError(
            f"{path} size {stat.st_size:,} does not match header/schema {expected:,}")
    return {
        "path": str(path.resolve()),
        "points": int(count),
        "bytes": int(stat.st_size),
        "mtime_ns": int(stat.st_mtime_ns),
        "itemsize": int(dtype.itemsize),
    }


@dataclass
class AnnotationMasks:
    cyan: np.ndarray
    pink: np.ndarray
    red: np.ndarray
    yellow: np.ndarray
    legacy_terrain: np.ndarray


@dataclass
class Classification:
    water_cells: np.ndarray
    water_new_cells: np.ndarray
    terrain_cells: np.ndarray
    groove_cells: np.ndarray
    sand_cells: np.ndarray
    rock_cells: np.ndarray
    terrain_quota_5mm: np.ndarray
    terrain_quota_1mm: np.ndarray
    water_quota_5mm: np.ndarray
    report: dict


def load_config(path: Path):
    raw = json.loads(path.read_text())
    grid = v6.GridSpec(**raw["support_grid"])
    return raw, grid


def build_annotation_masks(config: dict, grid: v6.GridSpec) -> AnnotationMasks:
    annotations = config["annotations"]
    legacy = json.loads(LEGACY_CONFIG.read_text())
    legacy_mask = np.zeros(grid.shape, bool)
    for region in legacy["terrain_regions"]:
        legacy_mask |= v6.rasterise_polygons(region["polygons"], grid)
    return AnnotationMasks(
        cyan=v6.rasterise_polygons(annotations["cyan_mixed_review"], grid),
        pink=v6.rasterise_polygons(annotations["pink_water_exclusion"], grid),
        red=v6.rasterise_polygons(annotations["red_water_connections"], grid),
        yellow=v6.rasterise_polygons(annotations["yellow_terrain_review"], grid),
        legacy_terrain=legacy_mask,
    )


def _cache_matches(saved, grid: v6.GridSpec, fingerprints: list[dict]) -> bool:
    if "meta_json" not in saved:
        return False
    metadata = json.loads(str(saved["meta_json"].item()))
    expected_grid = [grid.x0, grid.y0, grid.x1, grid.y1, grid.cell]
    return metadata.get("grid") == expected_grid and metadata.get("sources") == fingerprints


def build_5mm_maps(
    data_dir: Path,
    source_water: Path,
    grid: v6.GridSpec,
    cache_path: Path,
    reuse: bool,
) -> v6.SupportMaps:
    sources = [
        source_fingerprint(cloud_path(data_dir, "SAND", "5mm")),
        source_fingerprint(cloud_path(data_dir, "ROCK", "5mm")),
        source_fingerprint(source_water),
    ]
    if reuse and cache_path.exists():
        with np.load(cache_path, allow_pickle=False) as saved:
            if _cache_matches(saved, grid, sources):
                print(f"[maps] reuse {cache_path}", flush=True)
                return v6.SupportMaps(
                    grid, *(saved[name].copy() for name in
                            ("sand", "rock", "water", "sand_z", "rock_z", "water_z")))

    shape = grid.shape
    counts = {name: np.zeros(shape, np.uint16) for name in ("sand", "rock", "water")}
    zsum = {name: np.zeros(shape, np.float64) for name in counts}
    zcount = {name: np.zeros(shape, np.uint16) for name in counts}
    for name, path, horizontal in (
        ("sand", cloud_path(data_dir, "SAND", "5mm"), True),
        ("rock", cloud_path(data_dir, "ROCK", "5mm"), True),
        ("water", source_water, False),
    ):
        count = v6._stream_grid_accumulate(
            path, grid, counts[name], zsum[name], zcount[name], horizontal)
        print(f"[maps] 5 mm {name}: {count:,} points", flush=True)
    means = {
        name: np.divide(
            zsum[name], zcount[name], out=np.full(shape, np.nan, np.float32),
            where=zcount[name] > 0).astype(np.float32)
        for name in counts
    }
    metadata = {"grid": [grid.x0, grid.y0, grid.x1, grid.y1, grid.cell],
                "sources": sources}
    np.savez_compressed(
        cache_path,
        sand=counts["sand"], rock=counts["rock"], water=counts["water"],
        sand_z=means["sand"], rock_z=means["rock"], water_z=means["water"],
        meta_json=np.array(json.dumps(metadata)),
    )
    return v6.SupportMaps(
        grid, counts["sand"], counts["rock"], counts["water"],
        means["sand"], means["rock"], means["water"])


def build_1mm_count_maps(
    data_dir: Path,
    grid: v6.GridSpec,
    cache_path: Path,
    reuse: bool,
) -> tuple[np.ndarray, np.ndarray]:
    sources = [
        source_fingerprint(cloud_path(data_dir, "SAND", "1mm")),
        source_fingerprint(cloud_path(data_dir, "ROCK", "1mm")),
    ]
    if reuse and cache_path.exists():
        with np.load(cache_path, allow_pickle=False) as saved:
            if _cache_matches(saved, grid, sources):
                print(f"[maps] reuse {cache_path}", flush=True)
                return saved["sand"].copy(), saved["rock"].copy()
    result = {}
    for role in ("SAND", "ROCK"):
        count_map = np.zeros(grid.shape, np.uint16)
        count = v6._stream_grid_accumulate(
            cloud_path(data_dir, role, "1mm"), grid, count_map)
        result[role.lower()] = count_map
        print(f"[maps] 1 mm {role.lower()}: {count:,} points", flush=True)
    metadata = {"grid": [grid.x0, grid.y0, grid.x1, grid.y1, grid.cell],
                "sources": sources}
    np.savez_compressed(
        cache_path, sand=result["sand"], rock=result["rock"],
        meta_json=np.array(json.dumps(metadata)))
    return result["sand"], result["rock"]


def combine_base_z(maps: v6.SupportMaps) -> np.ndarray:
    sand = maps.sand.astype(np.float64)
    rock = maps.rock.astype(np.float64)
    sand_valid = np.isfinite(maps.sand_z)
    rock_valid = np.isfinite(maps.rock_z)
    weight = sand * sand_valid + rock * rock_valid
    numerator = (np.nan_to_num(maps.sand_z) * sand +
                 np.nan_to_num(maps.rock_z) * rock)
    return np.divide(
        numerator, weight, out=np.full(maps.grid.shape, np.nan, np.float64),
        where=weight > 0).astype(np.float32)


def local_positive_mean(values: np.ndarray, sigma: float = 5.0) -> np.ndarray:
    from scipy import ndimage
    positive = values > 0
    numerator = ndimage.gaussian_filter(values.astype(np.float32), sigma)
    denominator = ndimage.gaussian_filter(positive.astype(np.float32), sigma)
    return np.divide(
        numerator, denominator, out=np.zeros_like(numerator), where=denominator > 1e-5)


def classify_cells(
    config: dict,
    maps: v6.SupportMaps,
    sand_1mm: np.ndarray,
    rock_1mm: np.ndarray,
    masks: AnnotationMasks,
) -> Classification:
    from scipy import ndimage

    options = config["classification"]
    sand = maps.sand.astype(np.int32)
    rock = maps.rock.astype(np.int32)
    base = sand + rock
    prior_water = maps.water > 0
    base_z = combine_base_z(maps)
    difference = maps.water_z - base_z
    reflected_below = (
        np.isfinite(difference) &
        (difference > float(options["reflection_clearance_m"])))

    # In the mixed annotations, a cavity must either have no measured terrain
    # or sit materially above a lower return.  One low-count cell is allowed
    # to keep shoreline continuity, but a WATER point coincident with measured
    # terrain is reclassified as density support.
    cavity_allowed = (base <= 1) | reflected_below
    cyan_water = masks.cyan & prior_water & cavity_allowed
    cyan_water = (
        ndimage.binary_closing(cyan_water, structure=np.ones((3, 3), bool)) &
        masks.cyan & cavity_allowed)

    # Red is an explicit connection request, but it does not trump convincing
    # terrain.  It fills the missing/low-return part of each outline and leaves
    # its dense, height-agreeing portion to the terrain classifier.
    red_water = masks.red & cavity_allowed
    red_water |= (
        ndimage.binary_closing(
            (prior_water | red_water) & masks.red,
            structure=np.ones((3, 3), bool)) & masks.red & cavity_allowed)

    mixed = masks.cyan | masks.red | masks.yellow
    water_cells = (
        (prior_water & ~mixed & ~masks.pink) |
        cyan_water | red_water)
    water_new_cells = water_cells & ~prior_water

    terrain_agrees = (
        np.isfinite(difference) &
        (np.abs(difference) <= float(options["terrain_height_agreement_m"])))
    terrain_seed = (
        prior_water & mixed & ~water_cells &
        (((base > 0) & terrain_agrees) | masks.yellow))

    distance_to_base = (
        ndimage.distance_transform_edt(base == 0).astype(np.float32) * maps.grid.cell)
    support = ((base > 0) |
               (distance_to_base <= float(options["terrain_support_distance_m"])))
    terrain_cells = (
        ndimage.binary_dilation(
            terrain_seed | masks.legacy_terrain, iterations=1) &
        support & mixed & ~water_cells & ~masks.pink)

    # Thin empty runs between well-supported neighbours model the marked rock
    # grooves.  They receive only a small fraction of the normal quota.
    neighbourhood = ndimage.uniform_filter(
        (base > 0).astype(np.float32), size=3, mode="constant")
    groove_cells = (
        masks.cyan & (base == 0) &
        (distance_to_base <= float(options["terrain_support_distance_m"])) &
        (neighbourhood >= 0.35) & ~water_cells)
    terrain_cells |= groove_cells

    # Route by measured material.  The yellow annotation is unambiguously
    # ROCK in the cleaned clouds and therefore overrides the nearest-role tie.
    sand_distance = ndimage.distance_transform_edt(sand == 0)
    rock_distance = ndimage.distance_transform_edt(rock == 0)
    sand_cells = terrain_cells & (
        (sand > rock) | ((sand == rock) & (sand_distance <= rock_distance)))
    sand_cells[masks.yellow] = False
    rock_cells = terrain_cells & ~sand_cells

    target_5mm = np.clip(
        np.rint(local_positive_mean(base)),
        int(options["terrain_5mm_target_min"]),
        int(options["terrain_5mm_target_max"])).astype(np.int16)
    quota_5mm = np.maximum(target_5mm.astype(np.int32) - base, 0)
    quota_5mm[groove_cells] = np.ceil(
        quota_5mm[groove_cells] * float(options["linear_groove_5mm_fraction"])
    ).astype(np.int32)
    quota_5mm[~terrain_cells] = 0
    quota_5mm = np.clip(quota_5mm, 0, 25).astype(np.int16)

    base_1mm = sand_1mm.astype(np.int32) + rock_1mm.astype(np.int32)
    target_1mm = np.clip(
        np.rint(local_positive_mean(base_1mm)),
        int(options["terrain_1mm_target_min"]),
        int(options["terrain_1mm_target_max"])).astype(np.int16)
    quota_1mm = np.maximum(target_1mm.astype(np.int32) - base_1mm, 0)
    quota_1mm[groove_cells] = np.ceil(
        quota_1mm[groove_cells] * float(options["linear_groove_1mm_fraction"])
    ).astype(np.int32)
    quota_1mm[~terrain_cells] = 0
    quota_1mm = np.clip(quota_1mm, 0, 625).astype(np.int16)

    # Existing coarse shoreline cells retain their irregular partial density.
    # Interior cells and newly requested red/cyan connection cells are brought
    # to the established 25-point/25-mm-cell WATER density.
    interior = ndimage.binary_erosion(
        water_cells, structure=np.ones((3, 3), bool), border_value=0)
    target_water = maps.water.astype(np.int16).copy()
    target = int(options["water_target_points_per_25mm_cell"])
    target_water[interior] = target
    new_boundary = water_new_cells & ~interior
    target_water[new_boundary] = max(1, target // 2)
    target_water[water_new_cells & interior] = target
    water_quota = np.maximum(
        target_water.astype(np.int32) - maps.water.astype(np.int32), 0)
    water_quota[~water_cells] = 0
    water_quota = np.clip(water_quota, 0, 25).astype(np.int16)

    report = {
        "water": {
            "source_cells": int(prior_water.sum()),
            "output_cells": int(water_cells.sum()),
            "new_connection_cells": int(water_new_cells.sum()),
            "kept_source_points": int(maps.water[water_cells].sum()),
            "removed_source_points": int(maps.water.sum() - maps.water[water_cells].sum()),
            "planned_additions": int(water_quota.sum()),
            "pink_removed_points": int(maps.water[masks.pink].sum()),
            "yellow_removed_points": int(maps.water[masks.yellow & ~red_water].sum()),
        },
        "terrain": {
            "area_m2": float(terrain_cells.sum() * maps.grid.cell ** 2),
            "groove_area_m2": float(groove_cells.sum() * maps.grid.cell ** 2),
            "misclassified_water_points": int(maps.water[terrain_cells].sum()),
            "planned_5mm": int(quota_5mm.sum()),
            "planned_1mm": int(quota_1mm.sum()),
            "sand_cells": int(sand_cells.sum()),
            "rock_cells": int(rock_cells.sum()),
            "sand_planned_5mm": int(quota_5mm[sand_cells].sum()),
            "rock_planned_5mm": int(quota_5mm[rock_cells].sum()),
            "sand_planned_1mm": int(quota_1mm[sand_cells].sum()),
            "rock_planned_1mm": int(quota_1mm[rock_cells].sum()),
        },
    }
    return Classification(
        water_cells=water_cells,
        water_new_cells=water_new_cells,
        terrain_cells=terrain_cells,
        groove_cells=groove_cells,
        sand_cells=sand_cells,
        rock_cells=rock_cells,
        terrain_quota_5mm=quota_5mm,
        terrain_quota_1mm=quota_1mm,
        water_quota_5mm=water_quota,
        report=report,
    )


def write_plan_reviews(
    run_dir: Path,
    maps: v6.SupportMaps,
    masks: AnnotationMasks,
    classification: Classification,
) -> None:
    from PIL import Image

    base = np.full((*maps.grid.shape, 3), 255, np.uint8)
    base[maps.sand > 0] = (177, 132, 82)
    base[maps.rock > 0] = (104, 111, 119)

    def blend(image, mask, colour, alpha):
        image[mask] = np.clip(
            image[mask].astype(np.float32) * (1.0 - alpha) +
            np.asarray(colour, np.float32) * alpha, 0, 255).astype(np.uint8)

    review = base.copy()
    blend(review, classification.water_cells, (0, 120, 255), 0.58)
    blend(review, classification.terrain_quota_5mm > 0, (255, 145, 0), 0.72)
    blend(review, classification.groove_cells, (255, 255, 0), 0.92)
    blend(review, masks.pink & (maps.water > 0), (255, 0, 255), 0.75)
    blend(review, classification.water_new_cells, (255, 0, 0), 0.88)
    Image.fromarray(review).save(run_dir / "review-classification.png")

    density = base.copy()
    q5 = classification.terrain_quota_5mm.astype(np.float32)
    terrain_scale = np.clip(q5 / 20.0, 0.0, 1.0)
    active = q5 > 0
    density[active, 0] = 255
    density[active, 1] = (230 - 180 * terrain_scale[active]).astype(np.uint8)
    density[active, 2] = 0
    water_active = classification.water_quota_5mm > 0
    density[water_active] = (0, 165, 255)
    Image.fromarray(density).save(run_dir / "review-planned-density.png")


def prepare_plan(args):
    config, grid = load_config(args.config)
    source_water = cloud_path(args.data_dir, "WATER", "5mm")
    args.run_dir.mkdir(parents=True, exist_ok=True)
    (args.data_dir / "PatchRefinement/.invisible_places-ignore").touch()
    maps = build_5mm_maps(
        args.data_dir, source_water, grid,
        args.run_dir / "support-5mm-25mm.npz", args.reuse_maps)
    sand_1mm, rock_1mm = build_1mm_count_maps(
        args.data_dir, grid, args.run_dir / "support-1mm-counts-25mm.npz",
        args.reuse_maps)
    masks = build_annotation_masks(config, grid)
    classification = classify_cells(config, maps, sand_1mm, rock_1mm, masks)
    np.savez_compressed(
        args.run_dir / "classification.npz",
        water_cells=classification.water_cells,
        water_new_cells=classification.water_new_cells,
        terrain_cells=classification.terrain_cells,
        groove_cells=classification.groove_cells,
        sand_cells=classification.sand_cells,
        rock_cells=classification.rock_cells,
        terrain_quota_5mm=classification.terrain_quota_5mm,
        terrain_quota_1mm=classification.terrain_quota_1mm,
        water_quota_5mm=classification.water_quota_5mm,
    )
    (args.run_dir / "classification-report.json").write_text(
        json.dumps(classification.report, indent=2))
    write_plan_reviews(args.run_dir, maps, masks, classification)
    print(json.dumps(classification.report, indent=2), flush=True)
    return config, grid, maps, sand_1mm, rock_1mm, masks, classification


class PlySubsetWriter:
    """Stream a schema-compatible PLY subset without retaining it in memory."""

    def __init__(self, path: Path, dtype: np.dtype, comments: list[str]):
        self.path = path
        self.temp = path.with_suffix(path.suffix + ".tmp")
        self.dtype = dtype
        self.count = 0
        self.stream = self.temp.open("wb+")
        buffer = io.BytesIO()
        v6.write_ply_header(buffer, dtype, 0, comments)
        header = buffer.getvalue().replace(
            b"element vertex 0\n", b"element vertex 0                   \n", 1)
        self.stream.write(header)
        self.header_size = self.stream.tell()

    def write(self, records: np.ndarray) -> None:
        if records.dtype != self.dtype:
            records = records.astype(self.dtype, copy=False)
        self.stream.write(records.tobytes(order="C"))
        self.count += len(records)

    def close(self) -> int:
        self.stream.flush()
        os.fsync(self.stream.fileno())
        self.stream.seek(0)
        header = self.stream.read(self.header_size)
        replacement = patch_header_count(header, self.count)
        self.stream.seek(0)
        self.stream.write(replacement)
        self.stream.flush()
        os.fsync(self.stream.fileno())
        self.stream.close()
        self.temp.replace(self.path)
        return self.count

    def abort(self) -> None:
        try:
            self.stream.close()
        finally:
            self.temp.unlink(missing_ok=True)


def build_append_candidate(
    source: Path,
    additions: Path,
    candidate: Path,
    overwrite: bool,
) -> dict:
    if candidate.exists() and not overwrite:
        raise RuntimeError(f"{candidate} exists; pass --overwrite")
    source_dtype, source_count, source_offset, source_header = v6.read_ply_header(source)
    addition_dtype, addition_count, addition_offset, _ = v6.read_ply_header(additions)
    if source_dtype != addition_dtype:
        raise RuntimeError(f"schema mismatch between {source} and {additions}")
    candidate_header = patch_header_count(source_header, source_count + addition_count)
    temp = candidate.with_suffix(candidate.suffix + ".tmp")
    digest = hashlib.sha256()
    with temp.open("wb") as output:
        output.write(candidate_header)
        digest.update(candidate_header)
        for path, offset in ((source, source_offset), (additions, addition_offset)):
            with path.open("rb") as stream:
                stream.seek(offset)
                while block := stream.read(32 * 1024 * 1024):
                    output.write(block)
                    digest.update(block)
        output.flush()
        os.fsync(output.fileno())
    temp.replace(candidate)
    return {
        "source": str(source.resolve()),
        "source_points": int(source_count),
        "source_sha256": v6.sha256_path(source),
        "source_payload_sha256": payload_sha256(source),
        "source_header_hex": source_header.hex(),
        "additions": str(additions.resolve()),
        "addition_points": int(addition_count),
        "candidate": str(candidate.resolve()),
        "candidate_points": int(source_count + addition_count),
        "candidate_sha256": digest.hexdigest(),
        "record_stride": int(source_dtype.itemsize),
    }


def iter_candidate_xy(
    quota: np.ndarray,
    role_mask: np.ndarray,
    grid: v6.GridSpec,
    spacing_m: float,
    seed: int,
    occupied_keys: np.ndarray | None = None,
    max_batch_points: int = 300_000,
) -> Iterator[tuple[np.ndarray, np.ndarray, np.ndarray]]:
    """Yield deterministic, unoccupied sub-cell candidates and their grid cells."""
    subdivisions = int(round(grid.cell / spacing_m))
    if not np.isclose(subdivisions * spacing_m, grid.cell, atol=1e-8):
        raise ValueError("candidate spacing must divide the 25 mm planning cell")
    capacity = subdivisions * subdivisions
    cells_y, cells_x = np.nonzero(role_mask & (quota > 0))
    requested = quota[cells_y, cells_x].astype(np.int32)
    rng = np.random.default_rng(seed)
    permutation = rng.permutation(capacity).astype(np.int32)
    fine_nx = int(round((grid.x1 - grid.x0) / spacing_m))

    begin = 0
    while begin < len(cells_x):
        end = begin
        points = 0
        while end < len(cells_x) and (
                end == begin or points + int(requested[end]) <= max_batch_points):
            points += int(requested[end])
            end += 1
        bx = cells_x[begin:end]
        by = cells_y[begin:end]
        bq = requested[begin:end]
        out_x: list[np.ndarray] = []
        out_y: list[np.ndarray] = []
        out_cell: list[np.ndarray] = []
        for cx, cy, count in zip(bx, by, bq, strict=True):
            shift = int((cx * 1103515245 + cy * 12345 + seed * 2654435761) % capacity)
            order = permutation[(np.arange(capacity, dtype=np.int32) + shift) % capacity]
            sx = order % subdivisions
            sy = order // subdivisions
            if occupied_keys is not None and len(occupied_keys):
                global_x = int(cx) * subdivisions + sx
                global_y = int(cy) * subdivisions + sy
                keys = global_y.astype(np.int64) * fine_nx + global_x.astype(np.int64)
                positions = np.searchsorted(occupied_keys, keys)
                free = positions >= len(occupied_keys)
                inside = ~free
                if inside.any():
                    free[inside] = occupied_keys[positions[inside]] != keys[inside]
                sx, sy = sx[free], sy[free]
            take = min(int(count), len(sx))
            if take == 0:
                continue
            sx, sy = sx[:take], sy[:take]
            # Jitter is contained well inside its fine cell.  It breaks a
            # machine-perfect lattice without weakening the spacing contract.
            local_seed = seed ^ (int(cx) * 73856093) ^ (int(cy) * 19349663)
            local_rng = np.random.default_rng(local_seed & 0xFFFFFFFF)
            jitter = local_rng.uniform(
                -0.08 * spacing_m, 0.08 * spacing_m, (take, 2)).astype(np.float32)
            x = (grid.x0 + int(cx) * grid.cell +
                 (sx.astype(np.float32) + 0.5) * spacing_m + jitter[:, 0])
            y = (grid.y0 + int(cy) * grid.cell +
                 (sy.astype(np.float32) + 0.5) * spacing_m + jitter[:, 1])
            out_x.append(x.astype(np.float32))
            out_y.append(y.astype(np.float32))
            out_cell.append(np.full(take, int(cy) * grid.nx + int(cx), np.int64))
        if out_x:
            yield np.concatenate(out_x), np.concatenate(out_y), np.concatenate(out_cell)
        begin = end


def fine_occupancy_keys(
    path: Path,
    planning_mask: np.ndarray,
    grid: v6.GridSpec,
    spacing_m: float,
) -> np.ndarray:
    cloud, _, _ = v6.memmap_cloud(path)
    fine_nx = int(round((grid.x1 - grid.x0) / spacing_m))
    fine_ny = int(round((grid.y1 - grid.y0) / spacing_m))
    pieces: list[np.ndarray] = []
    for start in range(0, len(cloud), CHUNK):
        chunk = cloud[start:start + CHUNK]
        gx, gy = grid.indices(chunk["x"], chunk["y"])
        inside = (gx >= 0) & (gx < grid.nx) & (gy >= 0) & (gy < grid.ny)
        rows = np.nonzero(inside)[0]
        selected = rows[planning_mask[gy[inside], gx[inside]]]
        if not len(selected):
            continue
        ix = np.floor((chunk["x"][selected] - grid.x0) / spacing_m).astype(np.int64)
        iy = np.floor((chunk["y"][selected] - grid.y0) / spacing_m).astype(np.int64)
        valid = (ix >= 0) & (ix < fine_nx) & (iy >= 0) & (iy < fine_ny)
        pieces.append(np.unique(iy[valid] * fine_nx + ix[valid]))
    if not pieces:
        return np.zeros(0, np.int64)
    keys = np.unique(np.concatenate(pieces))
    print(f"[occupancy] {path.name}: {len(keys):,} occupied {spacing_m*1000:g} mm cells",
          flush=True)
    return keys


@dataclass
class GeometryDonors:
    xyz: np.ndarray
    normals: np.ndarray
    rgb: np.ndarray
    tree_xy: object


@dataclass
class ScalarDonors:
    records: np.ndarray
    xyz: np.ndarray
    tree_xyz: object
    tree_xy: object


def _chunk_best_per_key(
    keys: np.ndarray,
    scores: np.ndarray,
    source_indices: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    order = np.lexsort((scores, keys))
    ordered_keys = keys[order]
    last = np.r_[np.flatnonzero(ordered_keys[:-1] != ordered_keys[1:]), len(order) - 1]
    selected = order[last]
    return keys[selected], scores[selected], source_indices[selected]


def collect_geometry_donors(
    path: Path,
    support_mask: np.ndarray,
    grid: v6.GridSpec,
) -> GeometryDonors:
    from scipy.spatial import cKDTree

    cloud, _, _ = v6.memmap_cloud(path)
    xyz_parts: list[np.ndarray] = []
    normal_parts: list[np.ndarray] = []
    rgb_parts: list[np.ndarray] = []
    for start in range(0, len(cloud), CHUNK):
        chunk = cloud[start:start + CHUNK]
        gx, gy = grid.indices(chunk["x"], chunk["y"])
        inside = (gx >= 0) & (gx < grid.nx) & (gy >= 0) & (gy < grid.ny)
        rows = np.nonzero(inside)[0]
        selected = rows[support_mask[gy[inside], gx[inside]]]
        if not len(selected):
            continue
        xyz = np.column_stack((chunk["x"][selected], chunk["y"][selected],
                               chunk["z"][selected])).astype(np.float32)
        normals = np.column_stack((chunk["nx"][selected], chunk["ny"][selected],
                                   chunk["nz"][selected])).astype(np.float32)
        length = np.linalg.norm(normals, axis=1)
        valid = (np.all(np.isfinite(xyz), axis=1) & np.all(np.isfinite(normals), axis=1) &
                 (length > 0.5))
        xyz, normals, length = xyz[valid], normals[valid], length[valid]
        normals /= length[:, None]
        normals[normals[:, 2] < 0] *= -1
        # Very steep/vertical returns do not define a stable XY height sheet.
        stable = normals[:, 2] >= 0.28
        xyz, normals = xyz[stable], normals[stable]
        colours = np.column_stack((chunk["red"][selected][valid][stable],
                                   chunk["green"][selected][valid][stable],
                                   chunk["blue"][selected][valid][stable])).astype(np.uint8)
        xyz_parts.append(xyz)
        normal_parts.append(normals)
        rgb_parts.append(colours)
    if not xyz_parts:
        raise RuntimeError(f"no stable geometry donors found in {path}")
    xyz = np.concatenate(xyz_parts)
    normals = np.concatenate(normal_parts)
    rgb = np.concatenate(rgb_parts)
    print(f"[donors] {path.name}: {len(xyz):,} stable 5 mm geometry records", flush=True)
    return GeometryDonors(xyz, normals, rgb, cKDTree(xyz[:, :2]))


def collect_scalar_donors(
    path: Path,
    support_mask: np.ndarray,
    grid: v6.GridSpec,
) -> ScalarDonors:
    """Select one complete, horizontal-preferred 1 mm record per 25 mm cell."""
    from scipy.spatial import cKDTree

    cloud, dtype, _ = v6.memmap_cloud(path)
    cell_count = grid.nx * grid.ny
    best_score = np.full(cell_count, -np.inf, np.float32)
    best_index = np.full(cell_count, -1, np.int64)
    float_fields = [name for name in dtype.names if dtype[name].kind == "f"]
    for start in range(0, len(cloud), CHUNK):
        chunk = cloud[start:start + CHUNK]
        gx, gy = grid.indices(chunk["x"], chunk["y"])
        inside = (gx >= 0) & (gx < grid.nx) & (gy >= 0) & (gy < grid.ny)
        rows = np.nonzero(inside)[0]
        rows = rows[support_mask[gy[inside], gx[inside]]]
        if not len(rows):
            continue
        complete = np.ones(len(rows), bool)
        for name in float_fields:
            complete &= np.isfinite(chunk[name][rows])
        complete &= chunk["scalar_Intensity"][rows] >= 5000.0
        complete &= chunk["scalar_Composite"][rows] >= 50.0
        rows = rows[complete]
        if not len(rows):
            continue
        keys = gy[rows].astype(np.int64) * grid.nx + gx[rows].astype(np.int64)
        normals = np.column_stack((chunk["nx"][rows], chunk["ny"][rows],
                                   chunk["nz"][rows])).astype(np.float32)
        length = np.maximum(np.linalg.norm(normals, axis=1), 1e-6)
        scores = (np.abs(normals[:, 2]) / length +
                  1e-6 * chunk["z"][rows].astype(np.float32))
        key, score, indices = _chunk_best_per_key(
            keys, scores, start + rows.astype(np.int64))
        improve = score > best_score[key]
        best_score[key[improve]] = score[improve]
        best_index[key[improve]] = indices[improve]
    indices = best_index[best_index >= 0]
    del best_score, best_index
    if not len(indices):
        raise RuntimeError(f"no complete scalar donors found in {path}")
    records = np.asarray(cloud[indices]).copy()
    xyz = np.column_stack((records["x"], records["y"], records["z"])).astype(np.float32)
    print(f"[donors] {path.name}: {len(records):,} complete 25 mm scalar records", flush=True)
    return ScalarDonors(records, xyz, cKDTree(xyz), cKDTree(xyz[:, :2]))


def collect_global_water_donors(
    data_dir: Path,
    grid: v6.GridSpec,
) -> ScalarDonors:
    """Choose one valid cleaned-terrain record per occupied 25 mm cell."""
    from scipy.spatial import cKDTree

    paths = [cloud_path(data_dir, "SAND", "5mm"),
             cloud_path(data_dir, "ROCK", "5mm")]
    dtype, _, _, _ = v6.read_ply_header(paths[0])
    cell_count = grid.nx * grid.ny
    best_score = np.full(cell_count, -np.inf, np.float32)
    best_index = np.full(cell_count, -1, np.int64)
    best_role = np.full(cell_count, -1, np.int8)
    for role_index, path in enumerate(paths):
        cloud, role_dtype, _ = v6.memmap_cloud(path)
        if role_dtype != dtype:
            raise RuntimeError("SAND/ROCK WATER donor schemas differ")
        for start in range(0, len(cloud), CHUNK):
            chunk = cloud[start:start + CHUNK]
            gx, gy = grid.indices(chunk["x"], chunk["y"])
            inside = (gx >= 0) & (gx < grid.nx) & (gy >= 0) & (gy < grid.ny)
            rows = np.nonzero(inside)[0]
            if not len(rows):
                continue
            valid = (
                np.isfinite(chunk["scalar_Intensity"][rows]) &
                (chunk["scalar_Intensity"][rows] >= 5000.0) &
                np.isfinite(chunk["scalar_Composite"][rows]) &
                (chunk["scalar_Composite"][rows] >= 50.0))
            for name in ENVIRONMENT_FIELDS:
                valid &= np.isfinite(chunk[name][rows])
            rows = rows[valid]
            if not len(rows):
                continue
            keys = gy[rows].astype(np.int64) * grid.nx + gx[rows].astype(np.int64)
            normal_length = np.sqrt(
                chunk["nx"][rows] ** 2 + chunk["ny"][rows] ** 2 +
                chunk["nz"][rows] ** 2)
            scores = (np.abs(chunk["nz"][rows]) / np.maximum(normal_length, 1e-6) +
                      1e-6 * chunk["z"][rows])
            key, score, indices = _chunk_best_per_key(
                keys, scores.astype(np.float32), start + rows.astype(np.int64))
            improve = score > best_score[key]
            best_score[key[improve]] = score[improve]
            best_index[key[improve]] = indices[improve]
            best_role[key[improve]] = role_index
    valid_cells = best_index >= 0
    cells = np.nonzero(valid_cells)[0]
    records = np.empty(len(cells), dtype=dtype)
    for role_index, path in enumerate(paths):
        selected = np.nonzero(best_role[cells] == role_index)[0]
        if not len(selected):
            continue
        cloud, _, _ = v6.memmap_cloud(path)
        records[selected] = cloud[best_index[cells[selected]]]
    del best_score, best_index, best_role
    xyz = np.column_stack((records["x"], records["y"], records["z"])).astype(np.float32)
    print(f"[donors] global WATER scalar source: {len(records):,} terrain cells", flush=True)
    return ScalarDonors(records, xyz, cKDTree(xyz), cKDTree(xyz[:, :2]))


def reconstruct_supported_geometry(
    x: np.ndarray,
    y: np.ndarray,
    donors: GeometryDonors,
    role: str,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    neighbours = min(12, len(donors.xyz))
    distance, index = donors.tree_xy.query(
        np.column_stack((x, y)), k=neighbours, workers=-1)
    if neighbours == 1:
        distance, index = distance[:, None], index[:, None]
    xyz = donors.xyz[index]
    normals = donors.normals[index]
    dx = x[:, None] - xyz[:, :, 0]
    dy = y[:, None] - xyz[:, :, 1]
    predicted = xyz[:, :, 2] - (
        normals[:, :, 0] * dx + normals[:, :, 1] * dy
    ) / np.maximum(normals[:, :, 2], 0.20)
    radius = 0.045 if role == "SAND" else 0.055
    valid = distance <= radius
    weights = np.where(valid, 1.0 / np.square(distance + 0.0015), 0.0)
    weight_sum = weights.sum(axis=1)
    z = np.divide(
        np.sum(predicted * weights, axis=1), weight_sum,
        out=np.zeros(len(x), np.float64), where=weight_sum > 0)
    variance = np.divide(
        np.sum(np.square(predicted - z[:, None]) * weights, axis=1), weight_sum,
        out=np.full(len(x), np.inf, np.float64), where=weight_sum > 0)
    roughness = np.sqrt(np.maximum(variance, 0.0)).astype(np.float32)
    normal = np.sum(normals * weights[:, :, None], axis=1)
    normal_length = np.linalg.norm(normal, axis=1)
    normal /= np.maximum(normal_length[:, None], 1e-8)
    agreement = np.divide(
        np.sum(np.abs(np.sum(normals * normal[:, None, :], axis=2)) * weights, axis=1),
        weight_sum, out=np.zeros(len(x), np.float64), where=weight_sum > 0)
    roughness_limit = 0.008 if role == "SAND" else 0.012
    accept = (
        (weight_sum > 0) & (distance[:, 0] <= radius * 0.72) &
        np.isfinite(z) & (roughness <= roughness_limit) &
        (normal[:, 2] >= 0.32) & (agreement >= 0.70))
    output_xyz = np.column_stack((x, y, z.astype(np.float32))).astype(np.float32)
    return output_xyz, normal.astype(np.float32), roughness, accept


def _idw_weights(distance: np.ndarray, floor: float = 0.003) -> np.ndarray:
    weights = 1.0 / np.square(np.maximum(distance.astype(np.float64), floor))
    weights /= np.maximum(weights.sum(axis=1, keepdims=True), 1e-12)
    return weights


def make_terrain_records(
    xyz: np.ndarray,
    normals: np.ndarray,
    geometry_roughness: np.ndarray,
    donors: ScalarDonors,
    dtype: np.dtype,
) -> tuple[np.ndarray, np.ndarray, dict]:
    neighbours = min(8, len(donors.records))
    distance, index = donors.tree_xyz.query(xyz, k=neighbours, workers=-1)
    if neighbours == 1:
        distance, index = distance[:, None], index[:, None]
    keep = np.isfinite(distance[:, 0]) & (distance[:, 0] <= 0.25)
    if not keep.any():
        return np.empty(0, dtype=dtype), keep, {
            "nearest_p50_m": None, "nearest_p95_m": None, "nearest_max_m": None}
    xyz = xyz[keep]
    normals = normals[keep]
    geometry_roughness = geometry_roughness[keep]
    distance = distance[keep]
    index = index[keep]
    weights = _idw_weights(distance)
    records = donors.records[index[:, 0]].copy().astype(dtype, copy=False)

    blend_fields = (
        "scalar_Intensity", "scalar_Composite", *GEOMETRY_FIELDS,
        *ENVIRONMENT_FIELDS)
    for name in blend_fields:
        values = donors.records[name][index].astype(np.float64)
        records[name] = np.sum(values * weights, axis=1)
    records["scalar_Intensity"] = np.maximum(records["scalar_Intensity"], 5000.0)
    records["scalar_Composite"] = np.clip(records["scalar_Composite"], 50.0, 255.0)

    neighbour_rgb = np.stack((donors.records["red"][index],
                              donors.records["green"][index],
                              donors.records["blue"][index]), axis=2)
    lab = v6._srgb_to_oklab(neighbour_rgb.reshape(-1, 3)).reshape(
        len(records), neighbours, 3)
    rgb = v6._oklab_to_srgb(np.sum(lab * weights[:, :, None], axis=1))
    records["red"], records["green"], records["blue"] = rgb.T

    records["x"], records["y"], records["z"] = xyz.T
    records["nx"], records["ny"], records["nz"] = normals.T
    records["scalar_ScanID"] = TERRAIN_SCAN_ID
    horizontal = np.linalg.norm(normals[:, :2], axis=1)
    nz = np.clip(normals[:, 2], 1e-5, 1.0)
    records["scalar_A_R_Downhill_X"] = 0.0
    records["scalar_A_R_Downhill_Y"] = 0.0
    active = horizontal > 1e-7
    records["scalar_A_R_Downhill_X"][active] = normals[active, 0] / horizontal[active]
    records["scalar_A_R_Downhill_Y"][active] = normals[active, 1] / horizontal[active]
    records["scalar_A_R_Downhill_Z"] = 0.0
    records["scalar_A_R_DownhillMagnitude"] = horizontal / nz
    records["scalar_A_R_Horizontalness"] = nz
    records["scalar_A_R_Slope_deg"] = np.degrees(np.arccos(nz))

    # The nearest measured bundle supplies the multi-scale context.  The fine
    # roughness cannot be smoother than the actual local plane disagreement.
    if "scalar_A_R_Roughness_Fine" in dtype.names:
        records["scalar_A_R_Roughness_Fine"] = np.maximum(
            records["scalar_A_R_Roughness_Fine"], geometry_roughness)
    fine = records["scalar_A_R_Roughness_Fine"].astype(np.float32)
    medium = records["scalar_A_R_Roughness_Medium"].astype(np.float32)
    broad = records["scalar_A_R_Roughness_Broad"].astype(np.float32)
    records["scalar_A_R_Roughness_Combined"] = np.clip(
        (fine / 0.0052 + medium / 0.0134 + broad / 0.0439) / 3.0, 0.0, 1.0)
    records["scalar_A_R_RoughnessRelative_FineMedium"] = np.clip(
        fine / np.maximum(medium, 1e-5), 0.0, 8.0)
    report = {
        "nearest_p50_m": float(np.quantile(distance[:, 0], 0.50)),
        "nearest_p95_m": float(np.quantile(distance[:, 0], 0.95)),
        "nearest_max_m": float(np.max(distance[:, 0])),
    }
    return records, keep, report


def build_role_additions(
    args,
    role: str,
    grid: v6.GridSpec,
    classification: Classification,
) -> tuple[dict, dict[str, np.ndarray]]:
    from scipy import ndimage

    role_cells = classification.sand_cells if role == "SAND" else classification.rock_cells
    support_mask = ndimage.binary_dilation(role_cells, iterations=1)
    geometry = collect_geometry_donors(
        cloud_path(args.data_dir, role, "5mm"), support_mask, grid)
    scalar = collect_scalar_donors(
        cloud_path(args.data_dir, role, "1mm"), support_mask, grid)

    role_report = {}
    accepted_maps: dict[str, np.ndarray] = {}
    for spacing_name, spacing_m, quota, seed in (
        ("5mm", 0.005, classification.terrain_quota_5mm, 9105),
        ("1mm", 0.001, classification.terrain_quota_1mm, 9101),
    ):
        source = cloud_path(args.data_dir, role, spacing_name)
        dtype, _, _, _ = v6.read_ply_header(source)
        occupied = fine_occupancy_keys(source, role_cells, grid, spacing_m)
        addition_path = args.run_dir / f"Site1-{role}-{spacing_name}-additions-scanid9.ply"
        if addition_path.exists() and not args.overwrite:
            raise RuntimeError(f"{addition_path} exists; pass --overwrite")
        writer = PlySubsetWriter(addition_path, dtype, [
            "Scene1/Fossils v7 conservative terrain density support",
            f"Role={role}; spacing={spacing_name}; ScanID=9",
            "Geometry is a support-gated local plane reconstruction from cleaned 5 mm points",
            "Intensity, Composite, RGB, and scalar context come from complete neighbouring 1 mm records",
        ])
        requested = int(quota[role_cells].sum())
        generated = geometry_accepted = scalar_accepted = 0
        roughness_samples: list[np.ndarray] = []
        nearest_reports: list[dict] = []
        accepted_map = np.zeros(grid.shape, np.int32)
        try:
            for x, y, cell_index in iter_candidate_xy(
                    quota, role_cells, grid, spacing_m, seed, occupied):
                generated += len(x)
                xyz, normals, roughness, geometry_keep = reconstruct_supported_geometry(
                    x, y, geometry, role)
                xyz = xyz[geometry_keep]
                normals = normals[geometry_keep]
                roughness = roughness[geometry_keep]
                cells = cell_index[geometry_keep]
                geometry_accepted += len(xyz)
                records, scalar_keep, scalar_report = make_terrain_records(
                    xyz, normals, roughness, scalar, dtype)
                cells = cells[scalar_keep]
                scalar_accepted += len(records)
                writer.write(records)
                if len(records):
                    np.add.at(accepted_map.ravel(), cells, 1)
                    roughness_samples.append(roughness[scalar_keep][::101])
                    nearest_reports.append(scalar_report)
                if generated == len(x) or generated % 1_000_000 < len(x):
                    print(
                        f"[terrain] {role} {spacing_name}: generated {generated:,}/"
                        f"{requested:,}; accepted {scalar_accepted:,}", flush=True)
            written = writer.close()
        except Exception:
            writer.abort()
            raise
        if written != scalar_accepted:
            raise RuntimeError("terrain addition writer count drift")
        rough = np.concatenate(roughness_samples) if roughness_samples else np.zeros(0)
        role_report[spacing_name] = {
            "requested": requested,
            "free_subcell_candidates": int(generated),
            "geometry_accepted": int(geometry_accepted),
            "accepted": int(scalar_accepted),
            "acceptance_fraction": float(scalar_accepted / max(generated, 1)),
            "addition_path": str(addition_path.resolve()),
            "addition_sha256": v6.sha256_path(addition_path),
            "roughness_geometry_q": (
                np.quantile(rough, [0.05, 0.5, 0.95]).tolist() if len(rough) else None),
            "scalar_nearest_p95_m": (
                max(r["nearest_p95_m"] for r in nearest_reports)
                if nearest_reports else None),
        }
        accepted_maps[spacing_name] = accepted_map
        print(f"[terrain] {role} {spacing_name}: wrote {written:,} additions", flush=True)
        candidate = args.run_dir / f"Site1-{role}-{spacing_name}.candidate.ply"
        role_report[spacing_name]["candidate"] = build_append_candidate(
            source, addition_path, candidate, args.overwrite)
    return role_report, accepted_maps


def robust_plane_fit(x: np.ndarray, y: np.ndarray, z: np.ndarray) -> tuple[np.ndarray, dict]:
    x = np.asarray(x, np.float64)
    y = np.asarray(y, np.float64)
    z = np.asarray(z, np.float64)
    x0, y0 = float(np.median(x)), float(np.median(y))
    design = np.column_stack((x - x0, y - y0, np.ones(len(x))))
    weights = np.ones(len(x), np.float64)
    coefficient = np.linalg.lstsq(design, z, rcond=None)[0]
    for _ in range(5):
        residual = z - design @ coefficient
        scale = max(1.4826 * float(np.median(np.abs(residual))), 0.002)
        u = np.abs(residual) / (2.5 * scale)
        weights = np.where(u <= 1.0, 1.0, 1.0 / np.maximum(u, 1e-8))
        weighted = design * np.sqrt(weights[:, None])
        coefficient = np.linalg.lstsq(weighted, z * np.sqrt(weights), rcond=None)[0]
    slope = float(np.hypot(coefficient[0], coefficient[1]))
    if slope > 0.03:
        coefficient[:2] *= 0.03 / slope
    model = np.array([coefficient[0], coefficient[1],
                      coefficient[2] - coefficient[0] * x0 - coefficient[1] * y0])
    residual = z - (model[0] * x + model[1] * y + model[2])
    model[2] += float(np.quantile(residual, 0.30)) - 0.002
    return model, {
        "slope": float(np.hypot(model[0], model[1])),
        "anchor_residual_mad_m": float(1.4826 * np.median(np.abs(residual))),
    }


def build_water_base_surface(
    maps: v6.SupportMaps,
    water_cells: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, dict]:
    from scipy import ndimage
    from scipy.spatial import cKDTree

    grid = maps.grid
    labels, component_count = ndimage.label(water_cells, np.ones((3, 3), bool))
    edge_distance = ndimage.distance_transform_edt(water_cells).astype(np.float32) * grid.cell
    base_z = combine_base_z(maps)
    base_valid = np.isfinite(base_z)
    distance, nearest = ndimage.distance_transform_edt(
        ~water_cells, return_indices=True)
    nearest_label = labels[tuple(nearest)]
    anchor_mask = (
        base_valid & ~water_cells & (nearest_label > 0) &
        (distance * grid.cell <= 0.45))

    water_flat = np.flatnonzero(water_cells)
    water_labels = labels.ravel()[water_flat]
    water_order = np.argsort(water_labels, kind="stable")
    water_flat = water_flat[water_order]
    water_labels = water_labels[water_order]
    water_starts = np.r_[0, np.flatnonzero(np.diff(water_labels)) + 1]
    water_stops = np.r_[water_starts[1:], len(water_labels)]

    anchor_flat = np.flatnonzero(anchor_mask)
    anchor_labels = nearest_label.ravel()[anchor_flat]
    anchor_order = np.argsort(anchor_labels, kind="stable")
    anchor_flat = anchor_flat[anchor_order]
    anchor_labels = anchor_labels[anchor_order]
    anchor_starts = np.r_[0, np.flatnonzero(np.diff(anchor_labels)) + 1]
    anchor_stops = np.r_[anchor_starts[1:], len(anchor_labels)]
    anchor_ranges = {
        int(anchor_labels[start]): (int(start), int(stop))
        for start, stop in zip(anchor_starts, anchor_stops, strict=True)
    }

    result = np.full(grid.shape, np.nan, np.float32)
    rows_report: list[dict] = []
    xs_all = grid.x0 + (np.arange(grid.nx, dtype=np.float64) + 0.5) * grid.cell
    ys_all = grid.y0 + (np.arange(grid.ny, dtype=np.float64) + 0.5) * grid.cell

    for start, stop in zip(water_starts, water_stops, strict=True):
        label = int(water_labels[start])
        flat = water_flat[start:stop]
        cy, cx = np.divmod(flat, grid.nx)
        x, y = xs_all[cx], ys_all[cy]
        prior = maps.water_z.ravel()[flat]
        prior = prior[np.isfinite(prior)]
        prior_median = float(np.median(prior)) if len(prior) else math.nan
        anchor_range = anchor_ranges.get(label)
        if anchor_range is None:
            anchors = np.zeros(0, np.int64)
        else:
            anchors = anchor_flat[slice(*anchor_range)]
        ay, ax = np.divmod(anchors, grid.nx)
        az = base_z.ravel()[anchors].astype(np.float64)
        anchor_x, anchor_y = xs_all[ax], ys_all[ay]
        if np.isfinite(prior_median) and len(az):
            plausible = ((az >= prior_median - 0.30) &
                         (az <= prior_median + 0.10))
            if plausible.sum() >= 8:
                az, anchor_x, anchor_y = az[plausible], anchor_x[plausible], anchor_y[plausible]
        if len(az) >= 8:
            upper = np.quantile(az, 0.80)
            plausible = az <= upper
            az, anchor_x, anchor_y = az[plausible], anchor_x[plausible], anchor_y[plausible]

        if len(az) >= 6:
            if len(az) > 20_000:
                stride = int(math.ceil(len(az) / 20_000))
                fit_x, fit_y, fit_z = anchor_x[::stride], anchor_y[::stride], az[::stride]
            else:
                fit_x, fit_y, fit_z = anchor_x, anchor_y, az
            plane, fit_report = robust_plane_fit(fit_x, fit_y, fit_z)
            surface = plane[0] * x + plane[1] * y + plane[2]
            plane_anchor = plane[0] * anchor_x + plane[1] * anchor_y + plane[2]
            residual = np.clip(az - plane_anchor, -0.05, 0.05)
            # Broad inverse-distance residual interpolation gives the large
            # pools real, continuous variation without following every noisy
            # return or creating a Poisson bowl.
            if len(flat) >= 100 and len(residual) >= 12:
                if len(residual) > 6_000:
                    stride = int(math.ceil(len(residual) / 6_000))
                    rx, ry, residual = (
                        anchor_x[::stride], anchor_y[::stride], residual[::stride])
                else:
                    rx, ry = anchor_x, anchor_y
                tree = cKDTree(np.column_stack((rx, ry)))
                k = min(8, len(residual))
                d, ix = tree.query(np.column_stack((x, y)), k=k, workers=-1)
                if k == 1:
                    d, ix = d[:, None], ix[:, None]
                weights = np.exp(-np.square(d / 0.65))
                weights[d > 1.75] = 0.0
                correction = np.divide(
                    np.sum(residual[ix] * weights, axis=1), weights.sum(axis=1),
                    out=np.zeros(len(x), np.float64), where=weights.sum(axis=1) > 1e-8)
                surface += np.clip(correction, -0.025, 0.025)
            # Keep a shoreline fit from drifting implausibly far from the
            # existing local vertical band while still replacing its steps.
            if np.isfinite(prior_median):
                median_surface = float(np.median(surface))
                if median_surface > prior_median + 0.05:
                    surface -= median_surface - (prior_median + 0.05)
                elif median_surface < prior_median - 0.10:
                    surface += (prior_median - 0.10) - median_surface
        elif np.isfinite(prior_median):
            surface = np.full(len(flat), prior_median, np.float64)
            fit_report = {"slope": 0.0, "anchor_residual_mad_m": None}
        elif len(az):
            surface = np.full(len(flat), float(np.quantile(az, 0.30)) - 0.002)
            fit_report = {"slope": 0.0, "anchor_residual_mad_m": None}
        else:
            surface = np.full(len(flat), 2.2, np.float64)
            fit_report = {"slope": 0.0, "anchor_residual_mad_m": None}

        # A three-millimetre interior bias suggests a shallow retained cavity
        # without imposing the conspicuous bowl that the user rejected.
        depth = np.clip(edge_distance.ravel()[flat] / 0.60, 0.0, 1.0)
        depth = depth * depth * (3.0 - 2.0 * depth)
        surface -= 0.003 * depth
        result.ravel()[flat] = surface.astype(np.float32)
        if len(flat) >= 100 or len(rows_report) < 30:
            rows_report.append({
                "id": label,
                "cells": int(len(flat)),
                "area_m2": float(len(flat) * grid.cell ** 2),
                "anchor_cells": int(len(az)),
                "old_water_median_z": prior_median if np.isfinite(prior_median) else None,
                "new_z_min": float(np.min(surface)),
                "new_z_median": float(np.median(surface)),
                "new_z_max": float(np.max(surface)),
                **fit_report,
            })
    rows_report.sort(key=lambda row: row["cells"], reverse=True)
    report = {
        "components": int(component_count),
        "water_area_m2": float(water_cells.sum() * grid.cell ** 2),
        "largest_components": rows_report[:200],
    }
    return result, edge_distance, report


def build_water_surface_model(
    grid: v6.GridSpec,
    water_cells: np.ndarray,
    base_surface: np.ndarray,
    edge_distance: np.ndarray,
) -> tuple[v6.SurfaceModel, dict]:
    ripple, ripple_report = v6.build_ripple_grid(grid, water_cells, edge_distance)
    surface = base_surface + ripple
    filled, _ = v6.fill_nearest(surface, water_cells)
    dzdy, dzdx = v6.masked_gradient(surface, water_cells, grid.cell)
    static = {}
    for label, sigma in {"Fine": 0.75, "Medium": 2.0, "Broad": 6.0}.items():
        smooth = v6.normalised_gaussian(surface, water_cells, sigma)
        residual = np.where(water_cells, surface - smooth, 0.0)
        roughness = np.sqrt(np.maximum(
            v6.normalised_gaussian(
                residual * residual, water_cells, max(0.75, sigma)), 0.0))
        gy, gx = v6.masked_gradient(smooth, water_cells, grid.cell)
        dyy, _ = v6.masked_gradient(gy, water_cells, grid.cell)
        dxy, dxx = v6.masked_gradient(gx, water_cells, grid.cell)
        static[f"scalar_A_R_MeanCurvature_{label}"] = (
            0.5 * (dxx + dyy)).astype(np.float16)
        static[f"scalar_A_R_CrossCurvature_{label}"] = dxy.astype(np.float16)
        static[f"scalar_A_R_Recession_{label}"] = residual.astype(np.float16)
        static[f"scalar_A_R_Roughness_{label}"] = roughness.astype(np.float16)
    model = v6.SurfaceModel(
        grid, water_cells, filled.astype(np.float32), dzdx.astype(np.float32),
        dzdy.astype(np.float32), static, ripple)
    return model, ripple_report


def apply_water_scalar_donors(
    records: np.ndarray,
    donors: ScalarDonors,
    preserve_rgb: bool,
) -> dict:
    query = np.column_stack((records["x"], records["y"]))
    neighbours = min(8, len(donors.records))
    distance, index = donors.tree_xy.query(query, k=neighbours, workers=-1)
    if neighbours == 1:
        distance, index = distance[:, None], index[:, None]
    weights = _idw_weights(distance, floor=0.015)
    for name in ("scalar_Intensity", "scalar_Composite", *ENVIRONMENT_FIELDS):
        values = donors.records[name][index].astype(np.float64)
        records[name] = np.sum(values * weights, axis=1)
    records["scalar_Intensity"] = np.maximum(records["scalar_Intensity"], 5000.0)
    records["scalar_Composite"] = np.clip(records["scalar_Composite"], 50.0, 255.0)
    if not preserve_rgb:
        neighbour_rgb = np.stack((donors.records["red"][index],
                                  donors.records["green"][index],
                                  donors.records["blue"][index]), axis=2)
        lab = v6._srgb_to_oklab(neighbour_rgb.reshape(-1, 3)).reshape(
            len(records), neighbours, 3)
        rgb = v6._oklab_to_srgb(np.sum(lab * weights[:, :, None], axis=1))
        records["red"], records["green"], records["blue"] = rgb.T
    return {
        "nearest_p50_m": float(np.quantile(distance[:, 0], 0.50)),
        "nearest_p95_m": float(np.quantile(distance[:, 0], 0.95)),
        "nearest_max_m": float(np.max(distance[:, 0])),
    }


def build_water_candidate(
    args,
    grid: v6.GridSpec,
    maps: v6.SupportMaps,
    classification: Classification,
) -> tuple[dict, v6.SurfaceModel]:
    source = cloud_path(args.data_dir, "WATER", "5mm")
    source_cloud, dtype, _ = v6.memmap_cloud(source)
    base_surface, edge_distance, surface_report = build_water_base_surface(
        maps, classification.water_cells)
    model, ripple_report = build_water_surface_model(
        grid, classification.water_cells, base_surface, edge_distance)
    donors = collect_global_water_donors(args.data_dir, grid)
    occupied = fine_occupancy_keys(source, classification.water_cells, grid, 0.005)

    addition_path = args.run_dir / "Site1-WATER-5mm-additions.ply"
    if addition_path.exists() and not args.overwrite:
        raise RuntimeError(f"{addition_path} exists; pass --overwrite")
    addition_writer = PlySubsetWriter(addition_path, dtype, [
        "Scene1/Fossils v7 cavity-connection WATER additions",
        "25 mm cell deficits use unoccupied 5 mm sub-cells; ScanID=999",
        "Height follows a robust local shoreline plane plus broad residual and 1.15 mm ripple",
        "Intensity and Composite come only from positive cleaned terrain neighbours",
    ])
    planned = int(classification.water_quota_5mm.sum())
    generated = 0
    donor_reports = []
    try:
        for x, y, _ in iter_candidate_xy(
                classification.water_quota_5mm, classification.water_cells,
                grid, 0.005, 9955, occupied):
            generated += len(x)
            query = np.column_stack((x, y))
            _, nearest = donors.tree_xy.query(query, k=1, workers=-1)
            records = donors.records[nearest].copy().astype(dtype, copy=False)
            records["x"], records["y"] = x, y
            v6.apply_water_surface(records, model)
            donor_reports.append(apply_water_scalar_donors(records, donors, preserve_rgb=False))
            addition_writer.write(records)
            if generated == len(x) or generated % 1_000_000 < len(x):
                print(f"[water] additions {generated:,}/{planned:,}", flush=True)
        addition_count = addition_writer.close()
    except Exception:
        addition_writer.abort()
        raise

    candidate = args.run_dir / "Site1-WATER-5mm.candidate.ply"
    if candidate.exists() and not args.overwrite:
        raise RuntimeError(f"{candidate} exists; pass --overwrite")
    final_writer = PlySubsetWriter(candidate, dtype, [
        f"Scene1/Fossils water reconstruction v7 generated {dt.datetime.now().isoformat(timespec='seconds')}",
        "Pink annotations removed; red voids connected; cyan terrain-density points excluded",
        "Water height is a continuous terrain-following shoreline fit, not a flat cloth/Poisson sheet",
        "Four deterministic Perlin octaves add approximately 1.15 mm RMS micro-ripple",
        "Intensity and Composite are positive neighbour-derived values; all geometry fields recomputed",
        "Generated WATER records use ScanID=999",
    ])
    kept = 0
    retained_donor_reports = []
    try:
        for start in range(0, len(source_cloud), CHUNK):
            chunk = np.asarray(source_cloud[start:start + CHUNK]).copy()
            gx, gy = grid.indices(chunk["x"], chunk["y"])
            inside = (gx >= 0) & (gx < grid.nx) & (gy >= 0) & (gy < grid.ny)
            keep = np.zeros(len(chunk), bool)
            rows = np.nonzero(inside)[0]
            keep[rows] = classification.water_cells[gy[inside], gx[inside]]
            records = chunk[keep]
            if len(records):
                v6.apply_water_surface(records, model)
                retained_donor_reports.append(
                    apply_water_scalar_donors(records, donors, preserve_rgb=True))
                final_writer.write(records)
                kept += len(records)
            if start == 0 or start + CHUNK >= len(source_cloud):
                print(f"[water] retained {kept:,}", flush=True)
        additions, _, _ = v6.memmap_cloud(addition_path)
        for start in range(0, len(additions), CHUNK):
            final_writer.write(np.asarray(additions[start:start + CHUNK]))
        candidate_count = final_writer.close()
    except Exception:
        final_writer.abort()
        raise
    expected = kept + addition_count
    if candidate_count != expected:
        raise RuntimeError(f"WATER candidate count {candidate_count} != {expected}")
    report = {
        "source": str(source.resolve()),
        "source_points": int(len(source_cloud)),
        "source_sha256": v6.sha256_path(source),
        "kept_points": int(kept),
        "planned_additions": planned,
        "addition_points": int(addition_count),
        "candidate": str(candidate.resolve()),
        "candidate_points": int(candidate_count),
        "candidate_sha256": v6.sha256_path(candidate),
        "surface": surface_report,
        "ripple": ripple_report,
        "scalar_nearest_p95_m": max(
            [row["nearest_p95_m"] for row in donor_reports + retained_donor_reports],
            default=None),
    }
    return report, model


def write_surface_reviews(
    run_dir: Path,
    maps: v6.SupportMaps,
    classification: Classification,
    model: v6.SurfaceModel,
    accepted_maps: dict[str, dict[str, np.ndarray]],
) -> None:
    from PIL import Image

    valid = classification.water_cells
    z = model.z
    values = z[valid]
    lo, hi = np.quantile(values, [0.01, 0.99])
    scale = np.clip((z - lo) / max(float(hi - lo), 1e-6), 0.0, 1.0)
    height = np.full((*maps.grid.shape, 3), 255, np.uint8)
    # Compact viridis-like ramp sufficient for a diagnostic review.
    stops = np.array([[68, 1, 84], [59, 82, 139], [33, 145, 140],
                      [94, 201, 98], [253, 231, 37]], np.float32)
    position = scale * (len(stops) - 1)
    index = np.minimum(np.floor(position).astype(np.int32), len(stops) - 2)
    fraction = position - index
    colour = stops[index] * (1.0 - fraction[..., None]) + stops[index + 1] * fraction[..., None]
    height[valid] = colour[valid].astype(np.uint8)
    Image.fromarray(height).save(run_dir / "review-water-height.png")

    delta = np.full((*maps.grid.shape, 3), 255, np.uint8)
    comparable = valid & np.isfinite(maps.water_z)
    change = maps.water_z - z
    amount = np.clip(np.abs(change) / 0.15, 0.0, 1.0)
    lower = comparable & (change > 0)
    raise_ = comparable & (change <= 0)
    delta[lower, 0] = (240 - 180 * amount[lower]).astype(np.uint8)
    delta[lower, 1] = (240 - 120 * amount[lower]).astype(np.uint8)
    delta[lower, 2] = 255
    delta[raise_, 0] = 255
    delta[raise_, 1] = (230 - 180 * amount[raise_]).astype(np.uint8)
    delta[raise_, 2] = (40 + 80 * amount[raise_]).astype(np.uint8)
    Image.fromarray(delta).save(run_dir / "review-water-height-change.png")

    density = np.zeros((*maps.grid.shape, 3), np.uint8)
    sand_5 = accepted_maps["SAND"]["5mm"]
    rock_5 = accepted_maps["ROCK"]["5mm"]
    density[sand_5 > 0] = (222, 151, 61)
    density[rock_5 > 0] = (120, 156, 210)
    density[classification.groove_cells & ((sand_5 + rock_5) > 0)] = (255, 240, 0)
    Image.fromarray(density).save(run_dir / "review-accepted-terrain-additions.png")

    ripple = model.ripple
    ripple_scale = np.clip(0.5 + ripple / 0.006, 0.0, 1.0)
    ripple_image = np.zeros((*maps.grid.shape, 3), np.uint8)
    ripple_image[..., 0] = (255 * ripple_scale).astype(np.uint8)
    ripple_image[..., 1] = (200 * (1 - np.abs(ripple_scale - 0.5) * 2)).astype(np.uint8)
    ripple_image[..., 2] = (255 * (1 - ripple_scale)).astype(np.uint8)
    ripple_image[~valid] = 0
    Image.fromarray(ripple_image).save(run_dir / "review-ripple-exaggerated.png")


def build(args):
    (config, grid, maps, sand_1mm, rock_1mm, masks,
     classification) = prepare_plan(args)
    terrain_report = {}
    accepted_maps = {}
    for role in ("SAND", "ROCK"):
        report, maps_by_spacing = build_role_additions(
            args, role, grid, classification)
        terrain_report[role] = report
        accepted_maps[role] = maps_by_spacing
    np.savez_compressed(
        args.run_dir / "accepted-terrain-counts-25mm.npz",
        sand_5mm=accepted_maps["SAND"]["5mm"],
        sand_1mm=accepted_maps["SAND"]["1mm"],
        rock_5mm=accepted_maps["ROCK"]["5mm"],
        rock_1mm=accepted_maps["ROCK"]["1mm"],
    )
    water_report, model = build_water_candidate(
        args, grid, maps, classification)
    write_surface_reviews(args.run_dir, maps, classification, model, accepted_maps)

    sources = {
        f"{role}-{spacing}": source_fingerprint(cloud_path(args.data_dir, role, spacing))
        for role in ("SAND", "ROCK") for spacing in ("1mm", "5mm")
    }
    sources["WATER-5mm"] = source_fingerprint(
        cloud_path(args.data_dir, "WATER", "5mm"))
    manifest = {
        "version": 7,
        "created": dt.datetime.now().isoformat(timespec="seconds"),
        "config": str(args.config.resolve()),
        "data_dir": str(args.data_dir.resolve()),
        "run_dir": str(args.run_dir.resolve()),
        "source_fingerprints": sources,
        "classification": classification.report,
        "terrain": terrain_report,
        "water": water_report,
        "installed": False,
    }
    (args.run_dir / "manifest.json").write_text(json.dumps(manifest, indent=2))
    print(f"[build] staged all five candidates in {args.run_dir}", flush=True)
    return manifest


def hash_candidate_prefix(path: Path, source_count: int, stride: int) -> str:
    _, _, offset, _ = v6.read_ply_header(path)
    remaining = int(source_count) * int(stride)
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        stream.seek(offset)
        while remaining:
            block = stream.read(min(32 * 1024 * 1024, remaining))
            if not block:
                raise RuntimeError(f"truncated candidate prefix in {path}")
            digest.update(block)
            remaining -= len(block)
    return digest.hexdigest()


def scan_generated_fields(
    path: Path,
    expected_scan_id: float,
    start_index: int = 0,
) -> dict:
    cloud, dtype, _ = v6.memmap_cloud(path)
    float_fields = [name for name in dtype.names if dtype[name].kind == "f"]
    nonfinite = {name: 0 for name in float_fields}
    wrong_scan = 0
    intensity_min = math.inf
    intensity_max = -math.inf
    composite_min = math.inf
    composite_max = -math.inf
    for start in range(start_index, len(cloud), CHUNK):
        chunk = cloud[start:min(start + CHUNK, len(cloud))]
        for name in float_fields:
            nonfinite[name] += int((~np.isfinite(chunk[name])).sum())
        wrong_scan += int((chunk["scalar_ScanID"] != expected_scan_id).sum())
        intensity_min = min(intensity_min, float(np.min(chunk["scalar_Intensity"])))
        intensity_max = max(intensity_max, float(np.max(chunk["scalar_Intensity"])))
        composite_min = min(composite_min, float(np.min(chunk["scalar_Composite"])))
        composite_max = max(composite_max, float(np.max(chunk["scalar_Composite"])))
    return {
        "points_scanned": int(len(cloud) - start_index),
        "wrong_scan_id": wrong_scan,
        "nonfinite": {name: value for name, value in nonfinite.items() if value},
        "intensity_min": intensity_min,
        "intensity_max": intensity_max,
        "composite_min": composite_min,
        "composite_max": composite_max,
    }


def rasterise_candidate(path: Path, grid: v6.GridSpec) -> tuple[np.ndarray, np.ndarray]:
    cloud, _, _ = v6.memmap_cloud(path)
    count = np.zeros(grid.shape, np.uint16)
    zsum = np.zeros(grid.shape, np.float64)
    zcount = np.zeros(grid.shape, np.uint16)
    v6._stream_grid_accumulate(path, grid, count, zsum, zcount, False)
    mean = np.divide(
        zsum, zcount, out=np.full(grid.shape, np.nan, np.float32),
        where=zcount > 0).astype(np.float32)
    if int(count.sum()) != len(cloud):
        raise RuntimeError(f"{path} contains points outside the verification grid")
    return count, mean


def _candidate_path(manifest: dict, key: str, entry: dict) -> Path:
    if manifest.get("installed"):
        return Path(manifest["installed_paths"][key]["canonical"])
    return Path(entry["candidate"])


def verify(args) -> dict:
    from scipy import ndimage

    manifest_path = args.run_dir / "manifest.json"
    manifest = json.loads(manifest_path.read_text())
    config, grid = load_config(args.config)
    masks = build_annotation_masks(config, grid)
    failures: list[str] = []
    terrain_results = {}

    for role in ("SAND", "ROCK"):
        terrain_results[role] = {}
        for spacing in ("1mm", "5mm"):
            key = f"{role}-{spacing}"
            entry = manifest["terrain"][role][spacing]["candidate"]
            path = _candidate_path(manifest, key, entry)
            dtype, count, _, _ = v6.read_ply_header(path)
            if count != int(entry["candidate_points"]):
                failures.append(f"{key}: count {count} != {entry['candidate_points']}")
            digest = v6.sha256_path(path)
            if digest != entry["candidate_sha256"]:
                failures.append(f"{key}: candidate SHA-256 mismatch")
            prefix = hash_candidate_prefix(path, entry["source_points"], entry["record_stride"])
            if prefix != entry["source_payload_sha256"]:
                failures.append(f"{key}: measured source payload prefix changed")
            fields = scan_generated_fields(path, TERRAIN_SCAN_ID, entry["source_points"])
            if fields["wrong_scan_id"]:
                failures.append(f"{key}: {fields['wrong_scan_id']} additions are not ScanID 9")
            if fields["nonfinite"]:
                failures.append(f"{key}: non-finite generated fields {fields['nonfinite']}")
            if fields["intensity_min"] < 5000.0:
                failures.append(f"{key}: generated Intensity below 5000")
            if fields["composite_min"] < 50.0:
                failures.append(f"{key}: generated Composite below 50")
            terrain_results[role][spacing] = {
                "path": str(path), "sha256": digest, "points": int(count),
                "prefix_payload_sha256": prefix, "generated_fields": fields,
            }

    water_entry = manifest["water"]
    water_path = _candidate_path(manifest, "WATER-5mm", water_entry)
    water_dtype, water_count, _, _ = v6.read_ply_header(water_path)
    if water_count != int(water_entry["candidate_points"]):
        failures.append(
            f"WATER-5mm: count {water_count} != {water_entry['candidate_points']}")
    water_digest = v6.sha256_path(water_path)
    if water_digest != water_entry["candidate_sha256"]:
        failures.append("WATER-5mm: candidate SHA-256 mismatch")
    water_fields = scan_generated_fields(water_path, WATER_SCAN_ID)
    if water_fields["wrong_scan_id"]:
        failures.append(
            f"WATER-5mm: {water_fields['wrong_scan_id']} points are not ScanID 999")
    if water_fields["nonfinite"]:
        failures.append(f"WATER-5mm: non-finite fields {water_fields['nonfinite']}")
    if water_fields["intensity_min"] < 5000.0:
        failures.append("WATER-5mm: Intensity below 5000")
    if water_fields["composite_min"] < 50.0:
        failures.append("WATER-5mm: Composite below 50")

    water_cells = np.load(args.run_dir / "classification.npz")["water_cells"]
    count_map, z_map = rasterise_candidate(water_path, grid)
    forbidden = masks.pink | (masks.yellow & ~masks.red)
    forbidden_points = int(count_map[forbidden].sum())
    outside_points = int(count_map[~water_cells].sum())
    if forbidden_points:
        failures.append(f"WATER-5mm: {forbidden_points} points remain in pink/yellow exclusions")
    if outside_points:
        failures.append(f"WATER-5mm: {outside_points} points fall outside classified cavities")

    interior = ndimage.binary_erosion(
        water_cells, structure=np.ones((3, 3), bool), border_value=0)
    interior_density = count_map[interior]
    underdense_fraction = float(np.mean(interior_density < 20)) if len(interior_density) else 1.0
    if underdense_fraction > 0.02:
        failures.append(
            f"WATER-5mm: {100*underdense_fraction:.2f}% of interior cells have <20 points")

    horizontal = water_cells[:, :-1] & water_cells[:, 1:]
    vertical = water_cells[:-1, :] & water_cells[1:, :]
    steps = np.concatenate((
        np.abs(z_map[:, :-1][horizontal] - z_map[:, 1:][horizontal]),
        np.abs(z_map[:-1, :][vertical] - z_map[1:, :][vertical]),
    ))
    steps = steps[np.isfinite(steps)]
    step_q = np.quantile(steps, [0.50, 0.95, 0.99, 0.999]).tolist() if len(steps) else None
    if len(steps) and float(np.quantile(steps, 0.99)) > 0.06:
        failures.append("WATER-5mm: 99th-percentile adjacent height step exceeds 60 mm")

    labels, component_count = ndimage.label(water_cells, np.ones((3, 3), bool))
    sizes = np.bincount(labels.ravel())[1:]
    largest = np.argsort(sizes)[-10:][::-1] + 1
    component_ranges = []
    for label in largest:
        values = z_map[labels == label]
        values = values[np.isfinite(values)]
        if not len(values):
            continue
        component_ranges.append({
            "id": int(label), "cells": int(sizes[label - 1]),
            "z_q05": float(np.quantile(values, 0.05)),
            "z_q50": float(np.quantile(values, 0.50)),
            "z_q95": float(np.quantile(values, 0.95)),
            "q95_minus_q05_m": float(np.quantile(values, 0.95) - np.quantile(values, 0.05)),
        })
    if component_ranges and max(row["q95_minus_q05_m"] for row in component_ranges[:2]) < 0.01:
        failures.append("WATER-5mm: both largest regions remain effectively flat")

    accepted = np.load(args.run_dir / "accepted-terrain-counts-25mm.npz")
    density_report = {}
    base_5 = np.load(args.run_dir / "support-5mm-25mm.npz")
    base_1 = np.load(args.run_dir / "support-1mm-counts-25mm.npz")
    for spacing, base_source in (("5mm", base_5), ("1mm", base_1)):
        addition = accepted[f"sand_{spacing}"] + accepted[f"rock_{spacing}"]
        before = base_source["sand"].astype(np.int32) + base_source["rock"].astype(np.int32)
        active = addition > 0
        density_report[spacing] = {
            "accepted_additions": int(addition.sum()),
            "active_cells": int(active.sum()),
            "before_q": np.quantile(before[active], [0.05, 0.5, 0.95]).tolist()
            if active.any() else None,
            "after_q": np.quantile((before + addition)[active], [0.05, 0.5, 0.95]).tolist()
            if active.any() else None,
            "groove_additions": int(addition[np.load(args.run_dir / "classification.npz")["groove_cells"]].sum()),
        }
        if int(addition.sum()) == 0:
            failures.append(f"terrain {spacing}: no additions accepted")

    report = {
        "verified": not failures,
        "verified_at": dt.datetime.now().isoformat(timespec="seconds"),
        "installed": bool(manifest.get("installed")),
        "terrain": terrain_results,
        "water": {
            "path": str(water_path), "sha256": water_digest,
            "points": int(water_count), "fields": water_fields,
            "forbidden_points": forbidden_points,
            "outside_classification_points": outside_points,
            "interior_density_q": np.quantile(
                interior_density, [0.01, 0.05, 0.5, 0.95, 0.99]).tolist()
            if len(interior_density) else None,
            "interior_underdense_fraction": underdense_fraction,
            "adjacent_height_step_q_m": step_q,
            "component_count": int(component_count),
            "largest_component_height_ranges": component_ranges,
        },
        "density": density_report,
        "failures": failures,
    }
    (args.run_dir / "verification-report.json").write_text(json.dumps(report, indent=2))
    if failures:
        raise RuntimeError("verification failed: " + "; ".join(failures))
    print(f"[verify] pass: WATER {water_count:,} points; all terrain prefixes byte-identical",
          flush=True)
    return report


def _installation_operations(args, manifest: dict) -> list[tuple[str, Path, Path, Path, str]]:
    backup_dir = args.run_dir / "source-backups"
    operations = []
    for role in ("SAND", "ROCK"):
        for spacing in ("1mm", "5mm"):
            key = f"{role}-{spacing}"
            entry = manifest["terrain"][role][spacing]["candidate"]
            canonical = cloud_path(args.data_dir, role, spacing)
            candidate = Path(entry["candidate"])
            backup = backup_dir / canonical.name
            operations.append((key, canonical, candidate, backup, entry["source_sha256"]))
    water = manifest["water"]
    canonical = cloud_path(args.data_dir, "WATER", "5mm")
    operations.append((
        "WATER-5mm", canonical, Path(water["candidate"]),
        args.data_dir / "Site1-WATER-5mm-old02.ply", water["source_sha256"]))
    return operations


def install(args) -> None:
    manifest_path = args.run_dir / "manifest.json"
    manifest = json.loads(manifest_path.read_text())
    if manifest.get("installed"):
        raise RuntimeError("v7 is already installed")
    verify(args)
    operations = _installation_operations(args, manifest)
    for key, canonical, candidate, backup, source_hash in operations:
        if not candidate.exists():
            raise RuntimeError(f"missing staged candidate {candidate}")
        if backup.exists():
            raise RuntimeError(f"refusing to overwrite backup {backup}")
        if v6.sha256_path(canonical) != source_hash:
            raise RuntimeError(f"{key}: canonical source changed after build")
    completed = []
    try:
        for key, canonical, candidate, backup, _ in operations:
            backup.parent.mkdir(parents=True, exist_ok=True)
            canonical.replace(backup)
            try:
                candidate.replace(canonical)
            except Exception:
                backup.replace(canonical)
                raise
            completed.append((key, canonical, candidate, backup))
            print(f"[install] {key}", flush=True)
    except Exception:
        for _, canonical, candidate, backup in reversed(completed):
            if canonical.exists() and not candidate.exists():
                canonical.replace(candidate)
            if backup.exists():
                backup.replace(canonical)
        raise
    manifest["installed"] = True
    manifest["installed_at"] = dt.datetime.now().isoformat(timespec="seconds")
    manifest["installed_paths"] = {
        key: {"canonical": str(canonical.resolve()), "backup": str(backup.resolve()),
              "staged_candidate": str(candidate.resolve())}
        for key, canonical, candidate, backup, _ in operations
    }
    manifest_path.write_text(json.dumps(manifest, indent=2))
    verify(args)
    print("[install] v7 canonical clouds installed; current WATER preserved as old02", flush=True)


def restore(args) -> None:
    manifest_path = args.run_dir / "manifest.json"
    manifest = json.loads(manifest_path.read_text())
    if not manifest.get("installed"):
        raise RuntimeError("v7 is not installed")
    operations = _installation_operations(args, manifest)
    completed = []
    try:
        for key, canonical, candidate, backup, source_hash in reversed(operations):
            if candidate.exists():
                raise RuntimeError(f"refusing to overwrite restored candidate {candidate}")
            if not backup.exists():
                raise RuntimeError(f"missing source backup {backup}")
            expected_candidate = (
                manifest["water"]["candidate_sha256"] if key == "WATER-5mm"
                else manifest["terrain"][key.split("-")[0]][key.split("-")[1]]["candidate"]["candidate_sha256"])
            if v6.sha256_path(canonical) != expected_candidate:
                raise RuntimeError(f"{key}: installed canonical changed; refusing restore")
            canonical.replace(candidate)
            backup.replace(canonical)
            if v6.sha256_path(canonical) != source_hash:
                raise RuntimeError(f"{key}: restored source hash mismatch")
            completed.append((key, canonical, candidate, backup))
            print(f"[restore] {key}", flush=True)
    except Exception:
        # Best-effort return of already restored files to their installed state.
        for _, canonical, candidate, backup in completed:
            if canonical.exists() and not backup.exists():
                canonical.replace(backup)
            if candidate.exists():
                candidate.replace(canonical)
        raise
    manifest["installed"] = False
    manifest["restored_at"] = dt.datetime.now().isoformat(timespec="seconds")
    manifest.pop("installed_paths", None)
    manifest_path.write_text(json.dumps(manifest, indent=2))
    print("[restore] byte-exact cleaned sources restored; v7 candidates restaged", flush=True)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("stage", choices=("plan", "build", "verify", "install", "restore"))
    parser.add_argument("--data-dir", type=Path, default=DEFAULT_DATA)
    parser.add_argument("--run-dir", type=Path, default=DEFAULT_RUN)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--reuse-maps", action="store_true")
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()
    if args.stage == "plan":
        prepare_plan(args)
    elif args.stage == "build":
        build(args)
    elif args.stage == "verify":
        verify(args)
    elif args.stage == "install":
        install(args)
    else:
        restore(args)


if __name__ == "__main__":
    main()
