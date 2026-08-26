#!/usr/bin/env python3
"""Rebuild the Scene1/Fossils water sheet and annotated terrain patches.

This is the conservative successor to ``site1_clean_and_cloth_water.py``.
It deliberately separates three jobs that the earlier cloth surface mixed:

* the user-drawn pink boundary removes generated WATER only (authored scene
  points are never changed);
* each connected pool is levelled from its own dense, near-horizontal rim
  measurements instead of following the terrain/Poisson mesh;
* the red, blue, and yellow annotations are density patches sampled from the
  screened-Poisson *terrain* mesh, accepted only when nearby measured points
  support the mesh geometry.

The water surface receives deterministic, multi-octave gradient noise with a
measured-flat-surface RMS of about 1.15 mm.  Normals and the geometry-derived
curvature, recession, roughness, downhill, horizontalness, and slope scalar
fields are regenerated from that surface.

The build is staged under a PatchRefinement run directory.  ``install`` moves
the current canonical cloud to ``Site1-WATER-5mm-old01.ply`` and atomically
puts the verified candidate at ``Site1-WATER-5mm.ply``.  ``restore`` reverses
that operation.

Requires numpy, scipy, Pillow, and open3d.  The local CloudAlignment venv has
the complete runtime on the machine used for the 2026-08-26 build.
"""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = ROOT / "scripts/config/site1_fossils_reconstruction_regions.json"
DEFAULT_DATA = ROOT / "Data/Scene1"
DEFAULT_RUN = DEFAULT_DATA / "PatchRefinement/20260826-water-v6-annotated"

WATER_SCAN_ID = 999.0
CHUNK = 2_000_000
CORE_OPEN_M = 0.15
RIM_M = 0.15
MIN_ANCHOR_POINTS_PER_CELL = 5
RIM_QUANTILE = 0.20
OLD_WATER_LEVEL_QUANTILE = 0.35
RIPPLE_RMS_M = 0.00115
RIPPLE_WAVELENGTHS_M = (0.075, 0.137, 0.263, 0.521)
RIPPLE_WEIGHTS = (0.48, 0.72, 0.62, 0.38)
TERRAIN_CLEARANCE_M = 0.00325
SCALAR_DONOR_CELL_M = 0.01

STATIC_FIELD_NAMES = (
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


def read_ply_header(path: Path):
    fields = []
    with open(path, "rb") as handle:
        header = b""
        while not header.endswith(b"end_header\n"):
            line = handle.readline()
            if not line:
                raise RuntimeError(f"no end_header in {path}")
            header += line
        offset = handle.tell()
    typemap = {
        "float": "<f4", "double": "<f8", "uchar": "u1", "char": "i1",
        "int": "<i4", "uint": "<u4", "short": "<i2", "ushort": "<u2",
    }
    count = None
    for line in header.decode("ascii", "replace").splitlines():
        parts = line.split()
        if not parts:
            continue
        if parts[:2] == ["element", "vertex"]:
            count = int(parts[2])
        elif parts[0] == "property" and parts[1] != "list":
            fields.append((parts[2], typemap[parts[1]]))
    if count is None or not fields:
        raise RuntimeError(f"unsupported PLY header in {path}")
    return np.dtype(fields), count, offset, header


def memmap_cloud(path: Path):
    dtype, count, offset, header = read_ply_header(path)
    return np.memmap(path, dtype=dtype, mode="r", offset=offset, shape=(count,)), dtype, header


def sha256_path(path: Path, block_size: int = 32 * 1024 * 1024) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        while block := handle.read(block_size):
            digest.update(block)
    return digest.hexdigest()


def write_ply_header(handle, dtype: np.dtype, count: int, comments: list[str]):
    typemap = {"f4": "float", "f8": "double", "u1": "uchar", "i1": "char",
               "i2": "short", "u2": "ushort", "i4": "int", "u4": "uint"}
    lines = ["ply", "format binary_little_endian 1.0"]
    lines.extend(f"comment {line}" for line in comments)
    lines.append(f"element vertex {count}")
    for name in dtype.names:
        code = dtype[name].str.lstrip("<>|=")
        lines.append(f"property {typemap[code]} {name}")
    lines.append("end_header")
    handle.write(("\n".join(lines) + "\n").encode("ascii"))


def cloud_path(data_dir: Path, role: str, spacing: str = "5mm") -> Path:
    return data_dir / f"Site1-{role}-{spacing}.ply"


@dataclass(frozen=True)
class GridSpec:
    x0: float
    y0: float
    x1: float
    y1: float
    cell: float

    @property
    def nx(self):
        return int(round((self.x1 - self.x0) / self.cell))

    @property
    def ny(self):
        return int(round((self.y1 - self.y0) / self.cell))

    @property
    def shape(self):
        return self.ny, self.nx

    def indices(self, x, y):
        return ((x - self.x0) / self.cell).astype(np.int64), \
               ((y - self.y0) / self.cell).astype(np.int64)


@dataclass
class SupportMaps:
    grid: GridSpec
    sand: np.ndarray
    rock: np.ndarray
    water: np.ndarray
    sand_z: np.ndarray
    rock_z: np.ndarray
    water_z: np.ndarray


def load_config(path: Path):
    config = json.loads(path.read_text())
    grid = GridSpec(**config["support_grid"])
    return config, grid


def rasterise_polygons(polygons, grid: GridSpec):
    from PIL import Image, ImageDraw
    image = Image.new("1", (grid.nx, grid.ny), 0)
    draw = ImageDraw.Draw(image)
    for polygon in polygons:
        points = [((x - grid.x0) / grid.cell, (y - grid.y0) / grid.cell)
                  for x, y in polygon]
        draw.polygon(points, fill=1)
    return np.asarray(image, dtype=bool).copy()


def build_annotation_masks(config, grid: GridSpec):
    boundary = config["water_exclusion_boundary"]
    outside = boundary + [
        [grid.x0 - 1.0, grid.y0 - 1.0],
        [grid.x1 + 1.0, grid.y0 - 1.0],
        [grid.x1 + 1.0, grid.y1 + 1.0],
        [boundary[0][0], grid.y1 + 1.0],
    ]
    exclusion = rasterise_polygons([outside], grid)
    terrain = {}
    combined = np.zeros(grid.shape, bool)
    for region in config["terrain_regions"]:
        mask = rasterise_polygons(region["polygons"], grid)
        terrain[region["id"]] = mask
        combined |= mask
    return exclusion, terrain, combined


def _stream_grid_accumulate(path: Path, grid: GridSpec, count, zsum=None,
                            zcount=None, horizontal_only=False):
    cloud, _, _ = memmap_cloud(path)
    flat_count = count.ravel()
    flat_zsum = None if zsum is None else zsum.ravel()
    flat_zcount = None if zcount is None else zcount.ravel()
    for start in range(0, len(cloud), CHUNK):
        chunk = cloud[start:start + CHUNK]
        gx, gy = grid.indices(chunk["x"], chunk["y"])
        inside = (gx >= 0) & (gx < grid.nx) & (gy >= 0) & (gy < grid.ny)
        flat = gy[inside] * grid.nx + gx[inside]
        np.add.at(flat_count, flat, 1)
        if flat_zsum is None:
            continue
        values = chunk["z"][inside].astype(np.float64)
        if horizontal_only:
            nx = chunk["nx"][inside].astype(np.float32)
            ny = chunk["ny"][inside].astype(np.float32)
            nz = chunk["nz"][inside].astype(np.float32)
            length = np.sqrt(nx * nx + ny * ny + nz * nz)
            horizontal = np.abs(nz) / np.maximum(length, 1e-6) >= 0.70
            flat, values = flat[horizontal], values[horizontal]
        np.add.at(flat_zsum, flat, values)
        np.add.at(flat_zcount, flat, 1)
    return len(cloud)


def build_support_maps(data_dir: Path, source_water: Path, grid: GridSpec,
                       cache_path: Path, reuse: bool):
    if reuse and cache_path.exists():
        saved = np.load(cache_path)
        meta = saved["meta"]
        expected = np.array([grid.x0, grid.y0, grid.x1, grid.y1, grid.cell])
        if np.allclose(meta, expected):
            print(f"[maps] reuse {cache_path}", flush=True)
            return SupportMaps(grid, *(saved[name] for name in
                ("sand", "rock", "water", "sand_z", "rock_z", "water_z")))

    shape = grid.shape
    counts = {name: np.zeros(shape, np.uint16) for name in ("sand", "rock", "water")}
    zsum = {name: np.zeros(shape, np.float64) for name in ("sand", "rock", "water")}
    zcount = {name: np.zeros(shape, np.uint16) for name in ("sand", "rock", "water")}
    for name, path, horizontal in (
        ("sand", cloud_path(data_dir, "SAND"), True),
        ("rock", cloud_path(data_dir, "ROCK"), True),
        ("water", source_water, False),
    ):
        count = _stream_grid_accumulate(path, grid, counts[name], zsum[name],
                                        zcount[name], horizontal)
        print(f"[maps] {name}: {count:,} points", flush=True)
    means = {
        name: np.divide(zsum[name], zcount[name],
                        out=np.full(shape, np.nan, np.float32),
                        where=zcount[name] > 0).astype(np.float32)
        for name in zsum
    }
    cache_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        cache_path,
        sand=counts["sand"], rock=counts["rock"], water=counts["water"],
        sand_z=means["sand"], rock_z=means["rock"], water_z=means["water"],
        meta=np.array([grid.x0, grid.y0, grid.x1, grid.y1, grid.cell]),
    )
    print(f"[maps] cached {cache_path}", flush=True)
    return SupportMaps(grid, counts["sand"], counts["rock"], counts["water"],
                       means["sand"], means["rock"], means["water"])


def grouped_quantiles(labels, values, quantiles, size):
    result = np.full((size, len(quantiles)), np.nan, np.float32)
    counts = np.zeros(size, np.int64)
    good = (labels > 0) & np.isfinite(values)
    labels = labels[good].astype(np.int64)
    values = values[good]
    if not len(labels):
        return result, counts
    order = np.argsort(labels, kind="stable")
    labels, values = labels[order], values[order]
    starts = np.r_[0, np.flatnonzero(np.diff(labels)) + 1]
    stops = np.r_[starts[1:], len(labels)]
    for begin, end in zip(starts, stops):
        label = labels[begin]
        counts[label] = end - begin
        result[label] = np.quantile(values[begin:end], quantiles)
    return result, counts


def fill_nearest(values, valid):
    from scipy import ndimage
    if not valid.any():
        raise RuntimeError("cannot nearest-fill an empty surface")
    indices = ndimage.distance_transform_edt(
        ~valid, return_distances=False, return_indices=True)
    return values[tuple(indices)], indices


def normalised_gaussian(values, valid, sigma):
    from scipy import ndimage
    numerator = ndimage.gaussian_filter(
        np.where(valid, values, 0.0).astype(np.float32), sigma)
    weight = ndimage.gaussian_filter(valid.astype(np.float32), sigma)
    return np.divide(numerator, weight, out=np.zeros_like(numerator), where=weight > 1e-5)


def masked_gradient(values, valid, spacing):
    """Differentiate within a surface without looking across holes or basins.

    ``numpy.gradient`` on a nearest-filled footprint sees the water level on
    the opposite bank of a deliberately cut basin separator.  That produces
    very steep normals even though neither pool contains a ramp.  Centred
    differences are used where both neighbours belong to the footprint and a
    one-sided difference is used at its edge.
    """
    gx = np.zeros_like(values, dtype=np.float32)
    gy = np.zeros_like(values, dtype=np.float32)

    left = np.zeros_like(valid)
    right = np.zeros_like(valid)
    left[:, 1:] = valid[:, :-1]
    right[:, :-1] = valid[:, 1:]
    both = valid & left & right
    rows, cols = np.nonzero(both)
    gx[rows, cols] = (values[rows, cols + 1] - values[rows, cols - 1]) / (2 * spacing)
    forward = valid & ~left & right
    rows, cols = np.nonzero(forward)
    gx[rows, cols] = (values[rows, cols + 1] - values[rows, cols]) / spacing
    backward = valid & left & ~right
    rows, cols = np.nonzero(backward)
    gx[rows, cols] = (values[rows, cols] - values[rows, cols - 1]) / spacing

    down = np.zeros_like(valid)
    up = np.zeros_like(valid)
    down[1:, :] = valid[:-1, :]
    up[:-1, :] = valid[1:, :]
    both = valid & down & up
    rows, cols = np.nonzero(both)
    gy[rows, cols] = (values[rows + 1, cols] - values[rows - 1, cols]) / (2 * spacing)
    forward = valid & ~down & up
    rows, cols = np.nonzero(forward)
    gy[rows, cols] = (values[rows + 1, cols] - values[rows, cols]) / spacing
    backward = valid & down & ~up
    rows, cols = np.nonzero(backward)
    gy[rows, cols] = (values[rows, cols] - values[rows - 1, cols]) / spacing
    return gy, gx


def build_pool_levels(maps: SupportMaps, exclusion, terrain_mask):
    from scipy import ndimage
    grid = maps.grid
    occ = (maps.water > 0) & ~exclusion & ~terrain_mask
    structure = np.ones((3, 3), bool)
    original, original_count = ndimage.label(occ, structure)

    radius = max(1, int(round(CORE_OPEN_M / grid.cell)))
    yy, xx = np.mgrid[-radius:radius + 1, -radius:radius + 1]
    disk = xx * xx + yy * yy <= radius * radius
    core = ndimage.binary_opening(occ, structure=disk)
    core_labels, core_count = ndimage.label(core, structure)
    if core_count == 0:
        raise RuntimeError("water footprint has no pool cores")

    nearest_core_indices = ndimage.distance_transform_edt(
        core_labels == 0, return_distances=False, return_indices=True)
    nearest_core = core_labels[tuple(nearest_core_indices)]
    core_original = np.zeros(core_count + 1, np.int32)
    core_pixels = core_labels > 0
    np.maximum.at(core_original, core_labels[core_pixels], original[core_pixels])

    rim_cells = max(1, int(round(RIM_M / grid.cell)))
    rim = ndimage.binary_dilation(occ, iterations=rim_cells) & ~occ
    sand_anchor = (rim & (maps.sand >= MIN_ANCHOR_POINTS_PER_CELL) &
                   np.isfinite(maps.sand_z))
    rock_anchor = (rim & ~sand_anchor &
                   (maps.rock >= MIN_ANCHOR_POINTS_PER_CELL) &
                   np.isfinite(maps.rock_z))
    sand_q, sand_n = grouped_quantiles(
        nearest_core[sand_anchor], maps.sand_z[sand_anchor],
        (RIM_QUANTILE,), core_count + 1)
    rock_q, rock_n = grouped_quantiles(
        nearest_core[rock_anchor], maps.rock_z[rock_anchor],
        (RIM_QUANTILE,), core_count + 1)
    water_q, water_n = grouped_quantiles(
        nearest_core[occ], maps.water_z[occ],
        (0.10, OLD_WATER_LEVEL_QUANTILE, 0.50, 0.90), core_count + 1)

    levels = np.full(core_count + 1, np.nan, np.float32)
    for label in range(1, core_count + 1):
        anchor = sand_q[label, 0] if sand_n[label] >= 12 else rock_q[label, 0]
        prior = water_q[label, 1]
        if np.isfinite(anchor) and np.isfinite(prior):
            levels[label] = min(anchor, prior)
        elif np.isfinite(prior):
            levels[label] = prior
        elif np.isfinite(anchor):
            levels[label] = anchor

    # Opening separates pools only at narrow necks.  Adjacent cores whose
    # independently measured levels agree within 3 cm are the same sheet and
    # receive one weighted level.  A larger disagreement is evidence that a
    # stochastic low-density bridge joined separate pools; those neck cells
    # are removed instead of manufacturing a steep ramp between water levels.
    pair_parts = []
    for a, b, valid_pair in (
        (nearest_core[:, :-1], nearest_core[:, 1:], occ[:, :-1] & occ[:, 1:]),
        (nearest_core[:-1, :], nearest_core[1:, :], occ[:-1, :] & occ[1:, :]),
    ):
        different = valid_pair & (a != b) & (a > 0) & (b > 0)
        if different.any():
            pair_parts.append(np.column_stack((a[different], b[different])))
    if pair_parts:
        pairs = np.concatenate(pair_parts).astype(np.int32)
        pairs.sort(axis=1)
        pairs = np.unique(pairs, axis=0)
    else:
        pairs = np.zeros((0, 2), np.int32)

    parent = np.arange(core_count + 1, dtype=np.int32)

    def find(label):
        while parent[label] != label:
            parent[label] = parent[parent[label]]
            label = parent[label]
        return label

    def union(a, b):
        a, b = find(int(a)), find(int(b))
        if a != b:
            parent[b] = a

    for a, b in pairs:
        if (np.isfinite(levels[a]) and np.isfinite(levels[b]) and
                abs(float(levels[a] - levels[b])) <= 0.03):
            union(a, b)
    roots = np.array([find(label) for label in range(core_count + 1)], np.int32)
    for root in np.unique(roots[1:]):
        members = np.nonzero(roots == root)[0]
        valid_members = members[np.isfinite(levels[members])]
        if not len(valid_members):
            continue
        weights = np.maximum(water_n[valid_members], 1)
        merged = float(np.average(levels[valid_members], weights=weights))
        levels[members] = merged

    cell_level = levels[nearest_core]
    separator = np.zeros_like(occ)
    horizontal_cut = (occ[:, :-1] & occ[:, 1:] &
                      (np.abs(cell_level[:, :-1] - cell_level[:, 1:]) > 0.03))
    vertical_cut = (occ[:-1, :] & occ[1:, :] &
                    (np.abs(cell_level[:-1, :] - cell_level[1:, :]) > 0.03))
    separator[:, :-1] |= horizontal_cut
    separator[:, 1:] |= horizontal_cut
    separator[:-1, :] |= vertical_cut
    separator[1:, :] |= vertical_cut
    separator &= occ & ~core
    separator = ndimage.binary_dilation(separator, iterations=1) & occ & ~core
    occ &= ~separator

    # Thin fragments that have no eroded core retain a per-fragment robust
    # level instead of borrowing a nearby but topologically separate pool.
    original_q, _ = grouped_quantiles(
        original[occ], maps.water_z[occ],
        (OLD_WATER_LEVEL_QUANTILE,), original_count + 1)
    initial = levels[nearest_core]
    same_component = core_original[nearest_core] == original
    fallback = original_q[original, 0]
    initial[occ & ~same_component] = fallback[occ & ~same_component]
    valid = occ & np.isfinite(initial)
    if np.any(occ & ~valid):
        nearest, _ = fill_nearest(initial, valid)
        initial[occ & ~valid] = nearest[occ & ~valid]

    # The first cut is based on core levels.  A few tiny no-core fragments can
    # acquire their robust fallback level only afterwards.  Cut any remaining
    # >3 cm cardinal jump now as well; retaining it would encode a near-
    # vertical water wall in the normals.  This final cut is deliberately
    # small and applies to both sides of the discontinuity.
    post_separator = np.zeros_like(occ)
    horizontal_cut = (occ[:, :-1] & occ[:, 1:] &
                      (np.abs(initial[:, :-1] - initial[:, 1:]) > 0.03))
    vertical_cut = (occ[:-1, :] & occ[1:, :] &
                    (np.abs(initial[:-1, :] - initial[1:, :]) > 0.03))
    post_separator[:, :-1] |= horizontal_cut
    post_separator[:, 1:] |= horizontal_cut
    post_separator[:-1, :] |= vertical_cut
    post_separator[1:, :] |= vertical_cut
    post_separator = ndimage.binary_dilation(post_separator, iterations=1) & occ
    separator |= post_separator
    occ &= ~post_separator
    separator_points = int(maps.water[separator].sum())

    # A connected body of still water has one level.  Consolidating every
    # final component after the cuts removes the last sub-centimetre core-
    # partition steps without averaging across genuinely separate basins.
    final_components, final_component_count = ndimage.label(occ, structure)
    labels = final_components[occ]
    weights = maps.water[occ].astype(np.float64)
    weighted_sum = np.bincount(
        labels, weights=initial[occ].astype(np.float64) * weights,
        minlength=final_component_count + 1)
    weight_sum = np.bincount(
        labels, weights=weights, minlength=final_component_count + 1)
    component_levels = np.divide(
        weighted_sum, weight_sum, out=np.zeros_like(weighted_sum),
        where=weight_sum > 0)
    initial[occ] = component_levels[labels].astype(np.float32)

    # Keep the accepted sheet exactly level.  Its elevation already comes
    # from dense shoreline cells; interpolating individual rim samples back
    # into the footprint creates the bowl/hover artefact this rebuild avoids.
    # The independently derived ripple is the only local height variation.
    smoothed = initial
    edge_distance = ndimage.distance_transform_edt(occ).astype(np.float32) * grid.cell

    core_rows = []
    for label in range(1, core_count + 1):
        if not np.isfinite(levels[label]):
            continue
        core_rows.append({
            "id": label,
            "water_cells": int(water_n[label]),
            "sand_anchor_cells": int(sand_n[label]),
            "rock_anchor_cells": int(rock_n[label]),
            "old_z_q10": float(water_q[label, 0]),
            "old_z_q35": float(water_q[label, 1]),
            "old_z_q50": float(water_q[label, 2]),
            "old_z_q90": float(water_q[label, 3]),
            "new_level": float(levels[label]),
        })
    report = {
        "source_water_points": int(maps.water.sum()),
        "kept_water_points": int(maps.water[occ].sum()),
        "excluded_water_points": int(maps.water[exclusion].sum()),
        "terrain_region_water_points_replaced": int(maps.water[terrain_mask].sum()),
        "basin_separator_points_removed": separator_points,
        "water_area_m2": float(occ.sum() * grid.cell * grid.cell),
        "original_components": int(original_count),
        "final_level_components": int(final_component_count),
        "pool_cores": int(core_count),
        "pool_levels": core_rows,
    }
    return occ, smoothed.astype(np.float32), edge_distance, report


def _hash_angle(ix, iy, seed):
    n = (ix.astype(np.int64) * 374761393 + iy.astype(np.int64) * 668265263 +
         np.int64(seed) * 2246822519)
    n = (n ^ (n >> 13)) * np.int64(1274126177)
    n ^= n >> 16
    return (n & np.int64(0xFFFFFFFF)).astype(np.float64) * (2.0 * np.pi / 2**32)


def perlin2(x, y, wavelength, seed, angle):
    ca, sa = math.cos(angle), math.sin(angle)
    u = (ca * x - sa * y) / wavelength + seed * 0.173
    v = (sa * x + ca * y) / wavelength - seed * 0.117
    ix = np.floor(u).astype(np.int64)
    iy = np.floor(v).astype(np.int64)
    fx = u - ix
    fy = v - iy
    fade_x = fx**3 * (fx * (fx * 6.0 - 15.0) + 10.0)
    fade_y = fy**3 * (fy * (fy * 6.0 - 15.0) + 10.0)

    def dot(dx, dy):
        theta = _hash_angle(ix + dx, iy + dy, seed)
        return np.cos(theta) * (fx - dx) + np.sin(theta) * (fy - dy)

    n00, n10, n01, n11 = dot(0, 0), dot(1, 0), dot(0, 1), dot(1, 1)
    nx0 = n00 + fade_x * (n10 - n00)
    nx1 = n01 + fade_x * (n11 - n01)
    return nx0 + fade_y * (nx1 - nx0)


def build_ripple_grid(grid: GridSpec, occ, edge_distance):
    noise = np.empty(grid.shape, np.float32)
    xs = grid.x0 + (np.arange(grid.nx, dtype=np.float64) + 0.5) * grid.cell
    angles = (0.17, 1.11, 2.37, 0.73)
    for y0 in range(0, grid.ny, 128):
        y1 = min(y0 + 128, grid.ny)
        ys = grid.y0 + (np.arange(y0, y1, dtype=np.float64) + 0.5) * grid.cell
        xx, yy = np.meshgrid(xs, ys)
        value = np.zeros_like(xx)
        for octave, (wave, weight, angle) in enumerate(zip(
                RIPPLE_WAVELENGTHS_M, RIPPLE_WEIGHTS, angles), 1):
            value += weight * perlin2(xx, yy, wave, 41 + 29 * octave, angle)
        noise[y0:y1] = value.astype(np.float32)
    fade = np.clip(edge_distance / 0.05, 0.0, 1.0)
    sample = noise[occ] * fade[occ]
    measured = float(np.sqrt(np.mean(sample * sample)))
    if measured <= 1e-8:
        raise RuntimeError("degenerate ripple noise")
    noise *= RIPPLE_RMS_M / measured
    noise *= fade
    noise[~occ] = 0.0
    return noise, {
        "rms_m": float(np.sqrt(np.mean(noise[occ] ** 2))),
        "p99_abs_m": float(np.quantile(np.abs(noise[occ]), 0.99)),
        "max_abs_m": float(np.max(np.abs(noise[occ]))),
        "wavelengths_m": list(RIPPLE_WAVELENGTHS_M),
    }


@dataclass
class SurfaceModel:
    grid: GridSpec
    occ: np.ndarray
    z: np.ndarray
    dzdx: np.ndarray
    dzdy: np.ndarray
    static: dict[str, np.ndarray]
    ripple: np.ndarray


def build_surface_model(grid, occ, base_level, edge_distance):
    ripple, ripple_report = build_ripple_grid(grid, occ, edge_distance)
    surface = base_level + ripple
    filled, nearest_indices = fill_nearest(surface, occ)
    del nearest_indices
    dzdy, dzdx = masked_gradient(surface, occ, grid.cell)
    static = {}
    scales = {
        "Fine": 0.75,
        "Medium": 2.0,
        "Broad": 6.0,
    }
    for label, sigma in scales.items():
        smooth = normalised_gaussian(ripple, occ, sigma)
        residual = np.where(occ, ripple - smooth, 0.0)
        rough = np.sqrt(np.maximum(
            normalised_gaussian(residual * residual, occ, max(0.75, sigma)), 0.0))
        gy, gx = masked_gradient(smooth, occ, grid.cell)
        dyy, _ = masked_gradient(gy, occ, grid.cell)
        dxy, dxx = masked_gradient(gx, occ, grid.cell)
        static[f"scalar_A_R_MeanCurvature_{label}"] = (
            0.5 * (dxx + dyy)).astype(np.float16)
        static[f"scalar_A_R_CrossCurvature_{label}"] = dxy.astype(np.float16)
        static[f"scalar_A_R_Recession_{label}"] = residual.astype(np.float16)
        static[f"scalar_A_R_Roughness_{label}"] = rough.astype(np.float16)
    return SurfaceModel(grid, occ, filled.astype(np.float32),
                        dzdx.astype(np.float32), dzdy.astype(np.float32),
                        static, ripple), ripple_report


def sample_grid(values, x, y, grid: GridSpec, order=1):
    u = ((np.asarray(x, dtype=np.float32) - grid.x0) / grid.cell - 0.5)
    v = ((np.asarray(y, dtype=np.float32) - grid.y0) / grid.cell - 0.5)
    if order != 1:
        from scipy.ndimage import map_coordinates
        source = values.astype(np.float32) if values.dtype == np.float16 else values
        return map_coordinates(source, [v, u], order=order, mode="nearest")

    # scipy.ndimage.map_coordinates does not accept float16 on this runtime.
    # Direct bilinear sampling preserves the compact static-field grids and
    # avoids recasting every 12-million-cell field for each output chunk.
    u = np.clip(u, 0.0, values.shape[1] - 1.0)
    v = np.clip(v, 0.0, values.shape[0] - 1.0)
    x0 = np.floor(u).astype(np.int32)
    y0 = np.floor(v).astype(np.int32)
    x1 = np.minimum(x0 + 1, values.shape[1] - 1)
    y1 = np.minimum(y0 + 1, values.shape[0] - 1)
    fx = u - x0
    fy = v - y0
    result = values[y0, x0].astype(np.float32)
    result *= (1.0 - fx) * (1.0 - fy)
    result += values[y0, x1].astype(np.float32) * fx * (1.0 - fy)
    result += values[y1, x0].astype(np.float32) * (1.0 - fx) * fy
    result += values[y1, x1].astype(np.float32) * fx * fy
    return result


def apply_water_surface(record, model: SurfaceModel):
    x = record["x"].astype(np.float32)
    y = record["y"].astype(np.float32)
    z = sample_grid(model.z, x, y, model.grid).astype(np.float32)
    gx = sample_grid(model.dzdx, x, y, model.grid).astype(np.float32)
    gy = sample_grid(model.dzdy, x, y, model.grid).astype(np.float32)
    length = np.sqrt(1.0 + gx * gx + gy * gy)
    nx, ny, nz = -gx / length, -gy / length, 1.0 / length
    record["z"] = z
    record["nx"], record["ny"], record["nz"] = nx, ny, nz
    magnitude = np.sqrt(gx * gx + gy * gy)
    slope = np.degrees(np.arctan(magnitude)).astype(np.float32)
    horizontal_length = np.sqrt(nx * nx + ny * ny)
    horizontal = horizontal_length > 1e-8
    record["scalar_A_R_Downhill_X"] = 0
    record["scalar_A_R_Downhill_Y"] = 0
    record["scalar_A_R_Downhill_Z"] = 0
    record["scalar_A_R_Downhill_X"][horizontal] = (
        nx[horizontal] / horizontal_length[horizontal])
    record["scalar_A_R_Downhill_Y"][horizontal] = (
        ny[horizontal] / horizontal_length[horizontal])
    record["scalar_A_R_DownhillMagnitude"] = magnitude
    record["scalar_A_R_Horizontalness"] = nz
    record["scalar_A_R_Slope_deg"] = slope
    record["scalar_ScanID"] = WATER_SCAN_ID

    sampled = {}
    for name, values in model.static.items():
        sampled[name] = sample_grid(values, x, y, model.grid).astype(np.float32)
        if name in record.dtype.names:
            record[name] = sampled[name]
    fine_mean = sampled["scalar_A_R_MeanCurvature_Fine"]
    medium_mean = sampled["scalar_A_R_MeanCurvature_Medium"]
    broad_mean = sampled["scalar_A_R_MeanCurvature_Broad"]
    fine_cross = sampled["scalar_A_R_CrossCurvature_Fine"]
    medium_cross = sampled["scalar_A_R_CrossCurvature_Medium"]
    broad_cross = sampled["scalar_A_R_CrossCurvature_Broad"]
    fine_rec = sampled["scalar_A_R_Recession_Fine"]
    medium_rec = sampled["scalar_A_R_Recession_Medium"]
    broad_rec = sampled["scalar_A_R_Recession_Broad"]
    fine_rough = sampled["scalar_A_R_Roughness_Fine"]
    medium_rough = sampled["scalar_A_R_Roughness_Medium"]
    broad_rough = sampled["scalar_A_R_Roughness_Broad"]
    record["scalar_A_R_MeanCurvature_Combined"] = np.tanh(
        0.05 * fine_mean + 0.15 * medium_mean + 0.50 * broad_mean)
    record["scalar_A_R_CrossCurvature_Combined"] = np.tanh(
        0.05 * fine_cross + 0.15 * medium_cross + 0.50 * broad_cross)
    record["scalar_A_R_Recession_Combined"] = np.tanh(
        (fine_rec + medium_rec + broad_rec) / 0.01)
    record["scalar_A_R_Roughness_Combined"] = np.clip(
        (fine_rough / 0.0052 + medium_rough / 0.0134 + broad_rough / 0.0439) / 3.0,
        0.0, 1.0)
    record["scalar_A_R_RoughnessRelative_FineMedium"] = np.clip(
        fine_rough / np.maximum(medium_rough, 1e-5), 0.0, 8.0)


def _srgb_to_oklab(rgb):
    c = rgb.astype(np.float64) / 255.0
    linear = np.where(c <= 0.04045, c / 12.92, ((c + 0.055) / 1.055) ** 2.4)
    # Keep the LMS channels explicit: single-letter ``l`` is easily confused
    # with one and made this perceptual donor blend difficult to review.
    long = 0.4122214708 * linear[:, 0] + 0.5363325363 * linear[:, 1] + 0.0514459929 * linear[:, 2]
    medium = 0.2119034982 * linear[:, 0] + 0.6806995451 * linear[:, 1] + 0.1073969566 * linear[:, 2]
    short = 0.0883024619 * linear[:, 0] + 0.2817188376 * linear[:, 1] + 0.6299787005 * linear[:, 2]
    long, medium, short = np.cbrt(long), np.cbrt(medium), np.cbrt(short)
    return np.column_stack((
        0.2104542553 * long + 0.7936177850 * medium - 0.0040720468 * short,
        1.9779984951 * long - 2.4285922050 * medium + 0.4505937099 * short,
        0.0259040371 * long + 0.7827717662 * medium - 0.8086757660 * short,
    ))


def _oklab_to_srgb(lab):
    long = lab[:, 0] + 0.3963377774 * lab[:, 1] + 0.2158037573 * lab[:, 2]
    medium = lab[:, 0] - 0.1055613458 * lab[:, 1] - 0.0638541728 * lab[:, 2]
    short = lab[:, 0] - 0.0894841775 * lab[:, 1] - 1.2914855480 * lab[:, 2]
    long, medium, short = long**3, medium**3, short**3
    linear = np.column_stack((
        4.0767416621 * long - 3.3077115913 * medium + 0.2309699292 * short,
        -1.2684380046 * long + 2.6097574011 * medium - 0.3413193965 * short,
        -0.0041960863 * long - 0.7034186147 * medium + 1.7076147010 * short,
    ))
    linear = np.clip(linear, 0.0, 1.0)
    c = np.where(linear <= 0.0031308, 12.92 * linear,
                 1.055 * linear ** (1 / 2.4) - 0.055)
    return np.clip(c * 255.0 + 0.5, 0, 255).astype(np.uint8)


def inverse_distance_squared_weights(distances, floor_m=0.002):
    weights = 1.0 / np.maximum(
        np.asarray(distances, dtype=np.float32), floor_m) ** 2
    weights /= np.maximum(weights.sum(axis=1, keepdims=True), 1e-12)
    return weights


def raycast_top(scene, x, y):
    import open3d as o3d
    rays = np.zeros((len(x), 6), np.float32)
    rays[:, 0], rays[:, 1], rays[:, 2], rays[:, 5] = x, y, 30.0, -1.0
    answer = scene.cast_rays(o3d.core.Tensor(rays))
    hit = answer["t_hit"].numpy()
    z = 30.0 - hit
    normal = answer["primitive_normals"].numpy()
    return z.astype(np.float32), normal.astype(np.float32), np.isfinite(hit)


def collect_donors(data_dir, roles, bbox, spacing="5mm", finite_fields=()):
    parts = []
    x0, y0, x1, y1 = bbox
    for role in roles:
        cloud, _, _ = memmap_cloud(cloud_path(data_dir, role, spacing))
        for start in range(0, len(cloud), CHUNK):
            chunk = cloud[start:start + CHUNK]
            keep = ((chunk["x"] >= x0) & (chunk["x"] <= x1) &
                    (chunk["y"] >= y0) & (chunk["y"] <= y1))
            for name in finite_fields:
                keep &= np.isfinite(chunk[name])
            if keep.any():
                parts.append(np.asarray(chunk[keep]).copy())
    if not parts:
        raise RuntimeError(f"no measured donors in {bbox}")
    return np.concatenate(parts)


def build_scalar_donor_grids(config, data_dir, cache_path, reuse=False):
    """Keep the top complete 1 mm record in each local 10 mm XY cell.

    The source SAND/ROCK clouds are tens of gigabytes.  Streaming each role
    once preserves a dense local scalar source without retaining tens of
    millions of redundant records in memory.  Top-of-column selection matches
    the top-down terrain-mesh ray used to position patch candidates.
    """
    if reuse and cache_path.exists():
        with np.load(cache_path, allow_pickle=False) as saved:
            if ("cell_m" in saved and
                    np.isclose(float(saved["cell_m"][0]), SCALAR_DONOR_CELL_M)):
                print(f"[scalar] reuse {cache_path}", flush=True)
                return {region["id"]: saved[region["id"]].copy()
                        for region in config["terrain_regions"]}

    dtype, _, _, _ = read_ply_header(cloud_path(data_dir, "SAND", "1mm"))
    accumulators = {}
    for region in config["terrain_regions"]:
        polygon = np.concatenate([np.asarray(p) for p in region["polygons"]])
        margin = float(region["support_tangent_m"]) + 0.08
        bbox = (float(polygon[:, 0].min() - margin),
                float(polygon[:, 1].min() - margin),
                float(polygon[:, 0].max() + margin),
                float(polygon[:, 1].max() + margin))
        nx = int(math.ceil((bbox[2] - bbox[0]) / SCALAR_DONOR_CELL_M))
        ny = int(math.ceil((bbox[3] - bbox[1]) / SCALAR_DONOR_CELL_M))
        accumulators[region["id"]] = {
            "roles": set(region["roles"]), "bbox": bbox, "nx": nx, "ny": ny,
            "best_z": np.full(nx * ny, -np.inf, np.float32),
            "records": np.empty(nx * ny, dtype=dtype),
        }

    roles = sorted({role for region in config["terrain_regions"]
                    for role in region["roles"]})
    for role in roles:
        cloud, source_dtype, _ = memmap_cloud(cloud_path(data_dir, role, "1mm"))
        if source_dtype != dtype:
            raise RuntimeError(f"1 mm scalar schema mismatch for {role}")
        targets = [acc for acc in accumulators.values() if role in acc["roles"]]
        print(f"[scalar] scan {role} 1mm: {len(cloud):,} records", flush=True)
        for start in range(0, len(cloud), CHUNK):
            chunk = cloud[start:start + CHUNK]
            complete = np.ones(len(chunk), bool)
            for name in STATIC_FIELD_NAMES:
                complete &= np.isfinite(chunk[name])
            if not complete.any():
                continue
            for acc in targets:
                x0, y0, x1, y1 = acc["bbox"]
                keep = (complete & (chunk["x"] >= x0) & (chunk["x"] <= x1) &
                        (chunk["y"] >= y0) & (chunk["y"] <= y1))
                indices = np.nonzero(keep)[0]
                if not len(indices):
                    continue
                ix = np.floor((chunk["x"][indices] - x0) /
                              SCALAR_DONOR_CELL_M).astype(np.int32)
                iy = np.floor((chunk["y"][indices] - y0) /
                              SCALAR_DONOR_CELL_M).astype(np.int32)
                ix = np.clip(ix, 0, acc["nx"] - 1)
                iy = np.clip(iy, 0, acc["ny"] - 1)
                keys = iy.astype(np.int64) * acc["nx"] + ix
                z = chunk["z"][indices]
                order = np.lexsort((z, keys))
                sorted_keys = keys[order]
                last = np.r_[np.flatnonzero(sorted_keys[:-1] != sorted_keys[1:]),
                             len(sorted_keys) - 1]
                chosen = indices[order[last]]
                chosen_keys = keys[order[last]]
                improve = chunk["z"][chosen] > acc["best_z"][chosen_keys]
                chosen, chosen_keys = chosen[improve], chosen_keys[improve]
                acc["best_z"][chosen_keys] = chunk["z"][chosen]
                acc["records"][chosen_keys] = chunk[chosen]

    result = {}
    for region in config["terrain_regions"]:
        acc = accumulators[region["id"]]
        result[region["id"]] = acc["records"][np.isfinite(acc["best_z"])].copy()
        print(f"[scalar] {region['id']}: {len(result[region['id']]):,} "
              "complete top-surface donors", flush=True)
    cache_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez(cache_path, cell_m=np.array([SCALAR_DONOR_CELL_M]), **result)
    print(f"[scalar] cached {cache_path}", flush=True)
    return result


def terrain_candidate_xy(mask, base_count, target, grid: GridSpec, seed):
    from scipy import ndimage
    distance = ndimage.distance_transform_edt(mask).astype(np.float32) * grid.cell
    confidence = np.clip(distance / 0.10, 0.0, 1.0)
    confidence = confidence * confidence * (3.0 - 2.0 * confidence)
    deficit = np.maximum(target - base_count.astype(np.int32), 0)
    availability = np.maximum(25 - np.minimum(base_count, 24), 1)
    quota = np.ceil(deficit * 25.0 / availability * confidence).astype(np.int16)
    quota = np.clip(quota, 0, 25)
    cy, cx = np.nonzero(quota > 0)
    if not len(cx):
        return np.zeros(0, np.float32), np.zeros(0, np.float32), quota
    rng = np.random.default_rng(seed)
    offsets = np.array([(x, y) for y in range(5) for x in range(5)], np.float32)
    offsets = offsets[rng.permutation(25)]
    selection = np.arange(25)[None, :] < quota[cy, cx, None]
    rows, order = np.nonzero(selection)
    sx, sy = cx[rows], cy[rows]
    chosen = offsets[order]
    x = grid.x0 + (sx + (chosen[:, 0] + 0.5) / 5.0) * grid.cell
    y = grid.y0 + (sy + (chosen[:, 1] + 0.5) / 5.0) * grid.cell
    jitter = rng.uniform(-0.00035, 0.00035, (len(x), 2)).astype(np.float32)
    return (x.astype(np.float32) + jitter[:, 0],
            y.astype(np.float32) + jitter[:, 1], quota)


def build_terrain_additions(config, maps, terrain_masks, dtype, data_dir, mesh_path,
                            scalar_donors):
    import open3d as o3d
    from scipy.spatial import cKDTree
    mesh = o3d.io.read_triangle_mesh(str(mesh_path))
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
    del mesh
    additions = []
    reports = []
    for region_index, region in enumerate(config["terrain_regions"]):
        mask = terrain_masks[region["id"]]
        roles = [role.lower() for role in region["roles"]]
        base_count = sum(getattr(maps, role).astype(np.int32) for role in roles)
        from scipy import ndimage
        ring = ndimage.binary_dilation(mask, iterations=12) & ~mask
        ring_values = base_count[ring]
        positive = ring_values[ring_values > 0]
        auto = max(float(np.median(positive)) if len(positive) else 0.0,
                   float(np.mean(ring_values)) if len(ring_values) else 0.0)
        target = int(np.clip(round(auto), region["target_min"], region["target_max"]))
        px, py, quota = terrain_candidate_xy(mask, base_count, target, maps.grid,
                                              1901 + 101 * region_index)
        mesh_z, mesh_n, hit = raycast_top(scene, px, py)
        px, py, mesh_z, mesh_n = px[hit], py[hit], mesh_z[hit], mesh_n[hit]
        length = np.linalg.norm(mesh_n, axis=1)
        mesh_n /= np.maximum(length[:, None], 1e-6)
        flip = mesh_n[:, 2] < 0
        mesh_n[flip] *= -1

        polygons = np.concatenate([np.asarray(p) for p in region["polygons"]])
        margin = float(region["support_tangent_m"]) + 0.08
        bbox = (float(polygons[:, 0].min() - margin),
                float(polygons[:, 1].min() - margin),
                float(polygons[:, 0].max() + margin),
                float(polygons[:, 1].max() + margin))
        donors = collect_donors(data_dir, region["roles"], bbox)
        donor_xyz = np.column_stack((donors["x"], donors["y"], donors["z"])).astype(np.float32)
        donor_mesh_z = np.empty(len(donors), np.float32)
        donor_mesh_n = np.empty((len(donors), 3), np.float32)
        donor_hit = np.zeros(len(donors), bool)
        for start in range(0, len(donors), 1_000_000):
            stop = min(start + 1_000_000, len(donors))
            z, n, ok = raycast_top(scene, donors["x"][start:stop], donors["y"][start:stop])
            donor_mesh_z[start:stop], donor_mesh_n[start:stop], donor_hit[start:stop] = z, n, ok
        donor_n = np.column_stack((donors["nx"], donors["ny"], donors["nz"])).astype(np.float32)
        donor_n /= np.maximum(np.linalg.norm(donor_n, axis=1, keepdims=True), 1e-6)
        donor_mesh_n /= np.maximum(np.linalg.norm(donor_mesh_n, axis=1, keepdims=True), 1e-6)
        donor_agree = np.abs(np.sum(donor_n * donor_mesh_n, axis=1))
        mesh_supported = (
            donor_hit & (np.abs(donor_xyz[:, 2] - donor_mesh_z) <= 0.04) &
            (donor_agree >= region["normal_dot_min"]))
        scalar_records = scalar_donors[region["id"]]
        if mesh_supported.sum() < 100 or len(scalar_records) < 100:
            raise RuntimeError(
                f"{region['id']}: too few geometry or complete-scalar donors")
        geometry_records = donors[mesh_supported]
        geometry_xyz = donor_xyz[mesh_supported]
        geometry_n = donor_n[mesh_supported]
        scalar_xyz = np.column_stack((scalar_records["x"], scalar_records["y"],
                                      scalar_records["z"])).astype(np.float32)
        all_tree = cKDTree(donor_xyz)
        tree = cKDTree(geometry_xyz)
        scalar_tree = cKDTree(scalar_xyz)
        candidate_xyz = np.column_stack((px, py, mesh_z)).astype(np.float32)
        clearance, _ = all_tree.query(candidate_xyz, k=1, workers=-1)
        neighbour_dist, neighbour_index = tree.query(candidate_xyz, k=8, workers=-1)
        nearest = neighbour_index[:, 0]
        vector = candidate_xyz - geometry_xyz[nearest]
        plane = np.abs(np.sum(vector * geometry_n[nearest], axis=1))
        distance = neighbour_dist[:, 0]
        tangent = np.sqrt(np.maximum(distance * distance - plane * plane, 0.0))
        normal_dot = np.abs(np.sum(mesh_n * geometry_n[nearest], axis=1))
        accept = ((clearance >= TERRAIN_CLEARANCE_M) &
                  (plane <= region["support_plane_m"]) &
                  (tangent <= region["support_tangent_m"]) &
                  (normal_dot >= region["normal_dot_min"]))
        candidate_xyz, mesh_n = candidate_xyz[accept], mesh_n[accept]
        neighbour_dist, neighbour_index = neighbour_dist[accept], neighbour_index[accept]
        scalar_distance, scalar_index = scalar_tree.query(
            candidate_xyz, k=8, workers=-1)
        scalar_weights = inverse_distance_squared_weights(scalar_distance)
        nearest_geometry = neighbour_index[:, 0]
        record = geometry_records[nearest_geometry].copy().astype(dtype, copy=False)
        for name in STATIC_FIELD_NAMES:
            record[name] = np.sum(
                scalar_records[name][scalar_index] * scalar_weights, axis=1)
        record["x"], record["y"], record["z"] = candidate_xyz.T
        record["nx"], record["ny"], record["nz"] = mesh_n.T
        record["scalar_ScanID"] = WATER_SCAN_ID
        horizontal = np.linalg.norm(mesh_n[:, :2], axis=1)
        nz = np.clip(mesh_n[:, 2], -1.0, 1.0)
        record["scalar_A_R_Downhill_X"] = 0
        record["scalar_A_R_Downhill_Y"] = 0
        good = horizontal > 1e-6
        record["scalar_A_R_Downhill_X"][good] = mesh_n[good, 0] / horizontal[good]
        record["scalar_A_R_Downhill_Y"][good] = mesh_n[good, 1] / horizontal[good]
        record["scalar_A_R_Downhill_Z"] = 0
        record["scalar_A_R_DownhillMagnitude"] = horizontal / np.maximum(nz, 1e-5)
        record["scalar_A_R_Horizontalness"] = nz
        record["scalar_A_R_Slope_deg"] = np.degrees(np.arccos(nz))

        # Natural colour is blended from measured neighbours in perceptual
        # Oklab; the complete scalar bundle still comes from one real donor.
        valid_neighbour = np.isfinite(neighbour_dist)
        weights = np.where(valid_neighbour, 1.0 / np.maximum(neighbour_dist, 0.002) ** 2, 0.0)
        weights /= np.maximum(weights.sum(axis=1, keepdims=True), 1e-12)
        donor_rgb = np.stack((geometry_records["red"], geometry_records["green"],
                              geometry_records["blue"]), axis=1)
        lab = _srgb_to_oklab(donor_rgb)
        blended = np.sum(lab[neighbour_index] * weights[:, :, None], axis=1)
        rgb = _oklab_to_srgb(blended)
        record["red"], record["green"], record["blue"] = rgb.T
        additions.append(record)

        accepted_cells = np.zeros(maps.grid.shape, np.int32)
        gx, gy = maps.grid.indices(candidate_xyz[:, 0], candidate_xyz[:, 1])
        np.add.at(accepted_cells, (gy, gx), 1)
        before = base_count[mask]
        after = (base_count + accepted_cells)[mask]
        reports.append({
            "id": region["id"],
            "roles": region["roles"],
            "area_m2": float(mask.sum() * maps.grid.cell ** 2),
            "target_points_per_25mm_cell": target,
            "candidate_points": int(len(px)),
            "accepted_points": int(len(record)),
            "acceptance_fraction": float(len(record) / max(len(px), 1)),
            "mesh_supported_donors": int(mesh_supported.sum()),
            "complete_static_1mm_top_donors": int(len(scalar_records)),
            "scalar_transfer_method": "8-neighbour inverse-distance-squared",
            "scalar_transfer_nearest_p50_m": float(np.quantile(scalar_distance[:, 0], 0.50)),
            "scalar_transfer_nearest_p95_m": float(np.quantile(scalar_distance[:, 0], 0.95)),
            "scalar_transfer_nearest_max_m": float(np.max(scalar_distance[:, 0])),
            "density_before_mean": float(before.mean()),
            "density_after_mean": float(after.mean()),
            "empty_cells_before": int((before == 0).sum()),
            "empty_cells_after": int((after == 0).sum()),
            "support_plane_p95_m": float(np.quantile(plane[accept], 0.95)) if accept.any() else None,
            "support_tangent_p95_m": float(np.quantile(tangent[accept], 0.95)) if accept.any() else None,
        })
        print(f"[terrain] {region['id']}: {len(record):,}/{len(px):,} accepted; "
              f"cell density {before.mean():.1f} -> {after.mean():.1f} (target {target})",
              flush=True)
    return additions, reports


def write_review_images(run_dir: Path, maps: SupportMaps, exclusion, terrain_masks,
                        model: SurfaceModel, level_report):
    from PIL import Image
    base = np.zeros((*maps.grid.shape, 3), np.uint8)
    base[maps.sand > 0] = (174, 128, 76)
    base[maps.rock > 0] = (92, 119, 148)
    base[model.occ] = (50, 145, 220)
    base[exclusion & ((maps.sand + maps.rock) > 0)] = (125, 0, 140)
    colours = ((255, 70, 20), (55, 120, 255), (255, 225, 0))
    for colour, mask in zip(colours, terrain_masks.values()):
        base[mask] = colour
    Image.fromarray(np.fliplr(np.rot90(base, 1))).save(run_dir / "review-regions.png")

    correction = maps.water_z - model.z
    valid = model.occ & np.isfinite(maps.water_z)
    image = np.zeros((*maps.grid.shape, 3), np.uint8)
    lower = np.clip(correction / 0.25, 0.0, 1.0)
    raise_ = np.clip(-correction / 0.10, 0.0, 1.0)
    image[valid, 0] = (235 * (1 - lower[valid]) + 30 * lower[valid]).astype(np.uint8)
    image[valid, 1] = (235 * (1 - lower[valid]) + 80 * lower[valid]).astype(np.uint8)
    image[valid, 2] = 245
    image[valid & (correction < 0)] = np.column_stack((
        np.full((valid & (correction < 0)).sum(), 255, np.uint8),
        (210 - 120 * raise_[valid & (correction < 0)]).astype(np.uint8),
        np.full((valid & (correction < 0)).sum(), 20, np.uint8)))
    Image.fromarray(np.fliplr(np.rot90(image, 1))).save(run_dir / "review-water-level-change.png")

    ripple = model.ripple
    scale = np.clip(0.5 + ripple / 0.006, 0.0, 1.0)
    ripple_rgb = np.zeros((*maps.grid.shape, 3), np.uint8)
    ripple_rgb[..., 0] = (255 * scale).astype(np.uint8)
    ripple_rgb[..., 1] = (200 * (1 - np.abs(scale - 0.5) * 2)).astype(np.uint8)
    ripple_rgb[..., 2] = (255 * (1 - scale)).astype(np.uint8)
    ripple_rgb[~model.occ] = 0
    Image.fromarray(np.fliplr(np.rot90(ripple_rgb, 1))).save(run_dir / "review-ripple-exaggerated.png")


def choose_source_water(data_dir: Path, explicit: Path | None):
    if explicit is not None:
        return explicit
    old = data_dir / "Site1-WATER-5mm-old01.ply"
    canonical = data_dir / "Site1-WATER-5mm.ply"
    return old if old.exists() else canonical


def build(args):
    config, grid = load_config(args.config)
    source = choose_source_water(args.data_dir, args.source_water)
    if not source.exists():
        raise SystemExit(f"missing source WATER cloud: {source}")
    args.run_dir.mkdir(parents=True, exist_ok=True)
    (args.data_dir / "PatchRefinement/.invisible_places-ignore").touch()
    candidate = args.run_dir / "Site1-WATER-5mm.candidate.ply"
    if candidate.exists() and not args.overwrite:
        raise SystemExit(f"{candidate} exists; pass --overwrite")

    source_mm, source_dtype, _ = memmap_cloud(source)
    sand_dtype, _, _, _ = read_ply_header(cloud_path(args.data_dir, "SAND"))
    if source_dtype != sand_dtype:
        raise RuntimeError("source WATER schema no longer matches Site1 SAND schema")
    maps = build_support_maps(args.data_dir, source, grid,
                              args.run_dir / "support-maps.npz", args.reuse_maps)
    exclusion, terrain_masks, terrain_combined = build_annotation_masks(config, grid)
    occ, base_level, edge_distance, level_report = build_pool_levels(
        maps, exclusion, terrain_combined)
    model, ripple_report = build_surface_model(grid, occ, base_level, edge_distance)
    scalar_donors = build_scalar_donor_grids(
        config, args.data_dir, args.run_dir / "scalar-donors-1mm-top-10mm.npz",
        args.reuse_maps)
    additions, terrain_report = build_terrain_additions(
        config, maps, terrain_masks, source_dtype, args.data_dir,
        args.data_dir / "Site1-MESH.ply", scalar_donors)
    terrain_count = sum(len(part) for part in additions)
    kept_count = int(maps.water[occ].sum())
    output_count = kept_count + terrain_count
    stamp = dt.datetime.now().isoformat(timespec="seconds")
    comments = [
        f"Site1 Fossils water/terrain reconstruction v6 generated {stamp}",
        "Pink annotation is a hard generated-water exclusion; authored clouds unchanged",
        "Water pools use per-component dense horizontal rim levels, not Poisson/cloth z",
        "Each accepted pool sheet remains level at its dense shoreline-derived elevation",
        "Four rotated gradient-noise octaves add approximately 1.15 mm RMS micro-ripples",
        "Normals and curvature/recession/roughness/downhill/slope fields follow that surface",
        "Red/blue/yellow terrain deficits use mesh candidates gated by measured support",
        "All generated records retain ScanID=999 for the established WATER selection contract",
        "Generated by scripts/rebuild_site1_fossils_water.py",
    ]
    temp = candidate.with_suffix(".ply.tmp")
    with open(temp, "wb") as handle:
        write_ply_header(handle, source_dtype, output_count, comments)
        written = 0
        for start in range(0, len(source_mm), CHUNK):
            chunk = np.asarray(source_mm[start:start + CHUNK]).copy()
            gx, gy = grid.indices(chunk["x"], chunk["y"])
            inside = (gx >= 0) & (gx < grid.nx) & (gy >= 0) & (gy < grid.ny)
            keep = np.zeros(len(chunk), bool)
            rows = np.nonzero(inside)[0]
            keep[rows] = occ[gy[inside], gx[inside]]
            chunk = chunk[keep]
            apply_water_surface(chunk, model)
            chunk.tofile(handle)
            written += len(chunk)
            print(f"[write] water {written:,}/{kept_count:,}", flush=True)
        for records in additions:
            records.tofile(handle)
            written += len(records)
    if written != output_count:
        temp.unlink(missing_ok=True)
        raise RuntimeError(f"write count mismatch: {written} != {output_count}")
    temp.replace(candidate)

    source_hash = sha256_path(source)
    candidate_hash = sha256_path(candidate)
    manifest = {
        "version": 6,
        "created": stamp,
        "source": str(source),
        "source_sha256": source_hash,
        "candidate": str(candidate),
        "candidate_sha256": candidate_hash,
        "source_points": int(len(source_mm)),
        "candidate_points": int(output_count),
        "kept_water_points": kept_count,
        "terrain_patch_points": terrain_count,
        "grid": vars(grid),
        "level_report": level_report,
        "ripple_report": ripple_report,
        "terrain_report": terrain_report,
        "installed": False,
    }
    (args.run_dir / "manifest.json").write_text(json.dumps(manifest, indent=2))
    write_review_images(args.run_dir, maps, exclusion, terrain_masks, model, level_report)
    print(f"[build] wrote {candidate} ({output_count:,} points, "
          f"{candidate.stat().st_size / 1e9:.2f} GB)", flush=True)
    return candidate


def verify(args, path: Path | None = None):
    config, grid = load_config(args.config)
    manifest_path = args.run_dir / "manifest.json"
    manifest = json.loads(manifest_path.read_text())
    if path is None:
        candidate = Path(manifest["candidate"])
        path = candidate if candidate.exists() else args.data_dir / "Site1-WATER-5mm.ply"
    cloud, dtype, _ = memmap_cloud(path)
    expected_dtype, _, _, _ = read_ply_header(cloud_path(args.data_dir, "SAND"))
    failures = []
    if dtype != expected_dtype:
        failures.append("schema mismatch")
    if len(cloud) != manifest["candidate_points"]:
        failures.append(f"count {len(cloud)} != {manifest['candidate_points']}")
    exclusion, terrain_masks, terrain_combined = build_annotation_masks(config, grid)
    excluded = terrain_counts = nonfinite = wrong_scan = 0
    float_fields = [name for name in dtype.names if dtype[name].kind == "f"]
    nonfinite_fields = {name: 0 for name in float_fields}
    zmin, zmax = np.inf, -np.inf
    rough_values = []
    for start in range(0, len(cloud), CHUNK):
        chunk = cloud[start:start + CHUNK]
        gx, gy = grid.indices(chunk["x"], chunk["y"])
        inside = (gx >= 0) & (gx < grid.nx) & (gy >= 0) & (gy < grid.ny)
        excluded += int(exclusion[gy[inside], gx[inside]].sum())
        terrain_counts += int(terrain_combined[gy[inside], gx[inside]].sum())
        nonfinite += int((~np.isfinite(chunk["x"]) | ~np.isfinite(chunk["y"]) |
                          ~np.isfinite(chunk["z"])).sum())
        for name in float_fields:
            nonfinite_fields[name] += int((~np.isfinite(chunk[name])).sum())
        wrong_scan += int((chunk["scalar_ScanID"] != WATER_SCAN_ID).sum())
        zmin = min(zmin, float(chunk["z"].min()))
        zmax = max(zmax, float(chunk["z"].max()))
        rough_values.append(np.asarray(chunk["scalar_A_R_Roughness_Fine"][::101], np.float32))
    rough = np.concatenate(rough_values)
    finite_rough = rough[np.isfinite(rough)]
    if excluded:
        failures.append(f"{excluded} points remain beyond pink boundary")
    if nonfinite:
        failures.append(f"{nonfinite} non-finite points")
    nonfinite_fields = {name: count for name, count in nonfinite_fields.items() if count}
    if nonfinite_fields:
        failures.append("non-finite float fields: " + ", ".join(
            f"{name}={count}" for name, count in nonfinite_fields.items()))
    if wrong_scan:
        failures.append(f"{wrong_scan} records do not have ScanID 999")
    if not np.any(rough > 0):
        failures.append("roughness field is identically zero")
    digest = sha256_path(path)
    if path.name.endswith("candidate.ply") and digest != manifest["candidate_sha256"]:
        failures.append("candidate sha256 mismatch")
    report = {
        "verified": not failures,
        "path": str(path),
        "sha256": digest,
        "points": int(len(cloud)),
        "pink_exclusion_points": excluded,
        "terrain_region_points": terrain_counts,
        "nonfinite_points": nonfinite,
        "nonfinite_field_counts": nonfinite_fields,
        "wrong_scan_id_points": wrong_scan,
        "z_min": zmin,
        "z_max": zmax,
        "roughness_fine_q": (
            np.quantile(finite_rough, [0.05, 0.5, 0.95]).tolist()
            if len(finite_rough) else None),
        "failures": failures,
    }
    (args.run_dir / "verification-report.json").write_text(json.dumps(report, indent=2))
    if failures:
        raise RuntimeError("verification failed: " + "; ".join(failures))
    print(f"[verify] pass: {len(cloud):,} points, z {zmin:.3f}..{zmax:.3f}, "
          f"terrain-region points {terrain_counts:,}", flush=True)
    return report


def install(args):
    manifest_path = args.run_dir / "manifest.json"
    manifest = json.loads(manifest_path.read_text())
    candidate = Path(manifest["candidate"])
    canonical = args.data_dir / "Site1-WATER-5mm.ply"
    old = args.data_dir / "Site1-WATER-5mm-old01.ply"
    verify(args, candidate)
    if old.exists():
        raise SystemExit(f"refusing to overwrite preserved source {old}")
    if sha256_path(canonical) != manifest["source_sha256"]:
        raise RuntimeError("canonical WATER changed after the candidate build")
    canonical.replace(old)
    try:
        candidate.replace(canonical)
    except Exception:
        old.replace(canonical)
        raise
    manifest["installed"] = True
    manifest["installed_at"] = dt.datetime.now().isoformat(timespec="seconds")
    manifest["canonical"] = str(canonical)
    manifest["old01"] = str(old)
    manifest_path.write_text(json.dumps(manifest, indent=2))
    verify(args, canonical)
    print(f"[install] {old.name} preserves the {manifest['source_points']:,}-point source", flush=True)
    print(f"[install] {canonical.name} is the verified v6 reconstruction", flush=True)


def restore(args):
    manifest_path = args.run_dir / "manifest.json"
    manifest = json.loads(manifest_path.read_text())
    canonical = args.data_dir / "Site1-WATER-5mm.ply"
    old = args.data_dir / "Site1-WATER-5mm-old01.ply"
    candidate = Path(manifest["candidate"])
    if not old.exists():
        raise SystemExit(f"no preserved source at {old}")
    if candidate.exists():
        raise SystemExit(f"refusing to overwrite staged candidate {candidate}")
    canonical.replace(candidate)
    old.replace(canonical)
    if sha256_path(canonical) != manifest["source_sha256"]:
        raise RuntimeError("restored source hash mismatch")
    manifest["installed"] = False
    manifest["restored_at"] = dt.datetime.now().isoformat(timespec="seconds")
    manifest_path.write_text(json.dumps(manifest, indent=2))
    print("[restore] byte-exact old01 source restored", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("stage", choices=("build", "verify", "install", "restore"))
    parser.add_argument("--data-dir", type=Path, default=DEFAULT_DATA)
    parser.add_argument("--run-dir", type=Path, default=DEFAULT_RUN)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--source-water", type=Path)
    parser.add_argument("--reuse-maps", action="store_true")
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()
    if args.stage == "build":
        build(args)
    elif args.stage == "verify":
        verify(args)
    elif args.stage == "install":
        install(args)
    else:
        restore(args)


if __name__ == "__main__":
    main()
