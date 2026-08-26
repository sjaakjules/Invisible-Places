#!/usr/bin/env python3
"""Stage the Scene1/Fossils v9 WATER continuity and scalar repair.

v9 is deliberately evidence-led.  It removes the two registered off-site
components, carves water sheets that hover over densely measured original
terrain, and solves every remaining connected pool as a single regularised
surface.  Missing terrain is never considered a dam: only a continuous run
of measured, exposed terrain can divide the solve.  Sparse compatible terrain
may anchor a 2.5 mm film; terrain far below the water is ignored so reflected
returns cannot pull the sheet down.

The output is a uniform 5 mm-class layer.  Shore cells complement compatible
terrain but retain a 20/25 point floor, and no seam/edge probability suppresses
the water.  Geometry fields use the masked derivatives and combined formulas
from the v6 reconstruction.  Donor fields are deterministic bounded local
means without variance noise.

Stages are build, verify, install and restore.  ``build`` and ``verify`` only
write below PatchRefinement.  ``install`` preserves the current v8 canonical
cloud byte-for-byte in the v9 run directory; ``restore`` reverses that move.
"""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import math
import shutil
import sys
from pathlib import Path

import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))
import rebuild_site1_fossils_water as v6  # noqa: E402
import rebuild_site1_fossils_v8 as v8  # noqa: E402


ROOT = SCRIPT_DIR.parent
DEFAULT_DATA = ROOT / "Data/Scene1"
DEFAULT_RUN = DEFAULT_DATA / "PatchRefinement/20260826-water-v9-connected"

CELL = 0.025
DONOR_CELL = 0.05
OUTPUT_PITCH = 0.005
CHUNK = 2_000_000
WATER_SCAN_ID = 999.0

# Strict registered XY gates from the 13:24 review.  Inclusive limits are
# intentional; point generation and verification use this same predicate.
EXCLUSION_BOXES = (
    (756.675, 759.500, 820.975, 824.575, "offsite_semicircle"),
    (759.400, 760.275, 821.125, 824.775, "offsite_thin_component"),
)
FOCUS_BOX = (772.20, 772.42, 826.80, 827.05)

DENSE_ORIGINAL_COUNT = 8
EXPOSED_MIN_CLEARANCE = 0.006
DEEP_RETURN_CLEARANCE = 0.150
FILM_LIFT = 0.0025
MIN_EXPOSED_COMPONENT_CELLS = 12
FOOTPRINT_CLOSE_CELLS = 3
PLANE_WEIGHT = 0.035
WATER_WEIGHT = 0.015
TERRAIN_ANCHOR_WEIGHT = 4.0
LAPLACIAN_WEIGHT = 1.0
WATER_ANCHOR_CLAMP = 0.025
MAX_OPEN_NEIGHBOUR_STEP = 0.012
MAX_POST_SOLVE_PASSES = 6

TARGET_POINTS_PER_CELL = 25
MIN_SHORE_POINTS_PER_CELL = 20

FIELD_BOUNDS = {
    "red": (0.0, 255.0),
    "green": (0.0, 255.0),
    "blue": (0.0, 255.0),
    "scalar_Intensity": (5000.0, 5_302_784.0),
    "scalar_Composite": (50.0, 255.0),
    "scalar_A_R_Shelter_Lower": (0.0, 1.0),
    "scalar_A_R_RainExposure_Lower": (0.0, 1.0),
    "scalar_A_R_SVF_Lower": (0.0, 1.0),
}

COMBINED_BOUNDS = {
    "scalar_A_R_MeanCurvature_Combined": (-1.0, 1.0),
    "scalar_A_R_CrossCurvature_Combined": (-1.0, 1.0),
    "scalar_A_R_Recession_Combined": (-1.0, 1.0),
    "scalar_A_R_Roughness_Combined": (0.0, 1.0),
}


def log_line(prefix, message, handle=None):
    line = f"[{prefix}] {message}"
    print(line, flush=True)
    if handle is not None:
        handle.write(line + "\n")
        handle.flush()


def sha256_path(path: Path, block_size=32 * 1024 * 1024) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        while block := handle.read(block_size):
            digest.update(block)
    return digest.hexdigest()


def source_fingerprint(path: Path) -> dict:
    stat = path.stat()
    mm = v6.memmap_cloud(path)
    mm = mm[0] if isinstance(mm, tuple) else mm
    return {
        "path": str(path), "points": int(len(mm)), "bytes": stat.st_size,
        "mtime_ns": stat.st_mtime_ns,
    }


def app_running() -> bool:
    return v8.app_running()


def points_in_exclusion(x, y) -> np.ndarray:
    x = np.asarray(x)
    y = np.asarray(y)
    excluded = np.zeros(np.broadcast_shapes(x.shape, y.shape), dtype=bool)
    for x0, x1, y0, y1, _ in EXCLUSION_BOXES:
        excluded |= (x >= x0) & (x <= x1) & (y >= y0) & (y <= y1)
    return excluded


def bbox_mask(grid: v6.GridSpec, boxes=EXCLUSION_BOXES) -> np.ndarray:
    xs = grid.x0 + (np.arange(grid.nx) + 0.5) * grid.cell
    ys = grid.y0 + (np.arange(grid.ny) + 0.5) * grid.cell
    xx, yy = np.meshgrid(xs, ys)
    mask = np.zeros(grid.shape, bool)
    for x0, x1, y0, y1, *_ in boxes:
        mask |= (xx >= x0) & (xx <= x1) & (yy >= y0) & (yy <= y1)
    return mask


def aligned_water_grid(data_dir: Path, cell=CELL) -> v6.GridSpec:
    water = v6.memmap_cloud(data_dir / "Site1-WATER-5mm.ply")
    water = water[0] if isinstance(water, tuple) else water
    xmin = float(np.min(water["x"]))
    xmax = float(np.max(water["x"]))
    ymin = float(np.min(water["y"]))
    ymax = float(np.max(water["y"]))
    x0 = math.floor((xmin - cell) / cell) * cell
    y0 = math.floor((ymin - cell) / cell) * cell
    x1 = math.ceil((xmax + cell) / cell) * cell
    y1 = math.ceil((ymax + cell) / cell) * cell
    return v6.GridSpec(x0, y0, x1, y1, cell)


def _accumulate_z(path: Path, grid: v6.GridSpec, count, zsum,
                  zsum2=None, original_only=False, role_count=None):
    mm = v6.memmap_cloud(path)
    mm = mm[0] if isinstance(mm, tuple) else mm
    fc = count.ravel()
    fs = zsum.ravel()
    fs2 = None if zsum2 is None else zsum2.ravel()
    frc = None if role_count is None else role_count.ravel()
    for start in range(0, len(mm), CHUNK):
        chunk = mm[start:start + CHUNK]
        keep = ((chunk["x"] >= grid.x0) & (chunk["x"] < grid.x1) &
                (chunk["y"] >= grid.y0) & (chunk["y"] < grid.y1))
        if original_only:
            keep &= chunk["scalar_ScanID"] != 9.0
        if not keep.any():
            continue
        indices = np.nonzero(keep)[0]
        gx, gy = grid.indices(chunk["x"][indices], chunk["y"][indices])
        valid = (gx >= 0) & (gx < grid.nx) & (gy >= 0) & (gy < grid.ny)
        keys = gy[valid].astype(np.int64) * grid.nx + gx[valid]
        z = chunk["z"][indices[valid]].astype(np.float64)
        np.add.at(fc, keys, 1)
        np.add.at(fs, keys, z)
        if fs2 is not None:
            np.add.at(fs2, keys, z * z)
        if frc is not None:
            np.add.at(frc, keys, 1)


def stream_evidence(data_dir: Path, grid: v6.GridSpec, log) -> dict:
    """Accumulate v8 water and *original measured* terrain at 25 mm.

    ScanID-9 terrain is deliberately not allowed to create a dam, carve a
    sheet, or anchor a level.  This prevents generated additions from proving
    their own reconstruction.
    """
    shape = grid.shape
    wc = np.zeros(shape, np.int32)
    ws = np.zeros(shape, np.float64)
    _accumulate_z(data_dir / "Site1-WATER-5mm.ply", grid, wc, ws)
    tc = np.zeros(shape, np.int32)
    ts = np.zeros(shape, np.float64)
    ts2 = np.zeros(shape, np.float64)
    sandc = np.zeros(shape, np.int32)
    _accumulate_z(data_dir / "Site1-SAND-5mm.ply", grid, tc, ts, ts2,
                  original_only=True, role_count=sandc)
    _accumulate_z(data_dir / "Site1-ROCK-5mm.ply", grid, tc, ts, ts2,
                  original_only=True)
    log(f"evidence grid {grid.ny}x{grid.nx}: water {int(wc.sum()):,}, "
        f"original terrain {int(tc.sum()):,}")
    return {"water_count": wc, "water_sum": ws, "terrain_count": tc,
            "terrain_sum": ts, "terrain_sum2": ts2,
            "original_sand_count": sandc}


def finite_mean(total, count):
    return np.divide(total, count, out=np.full(total.shape, np.nan),
                     where=count > 0)


def robust_terrain_upper(evidence: dict) -> np.ndarray:
    count = evidence["terrain_count"]
    mean = finite_mean(evidence["terrain_sum"], count)
    second = finite_mean(evidence["terrain_sum2"], count)
    variance = np.maximum(second - np.nan_to_num(mean) ** 2, 0.0)
    sigma = np.sqrt(variance)
    # One high scanner return must not become a dam.  The capped half-sigma
    # is a robust upper proxy while retaining the top of a compact surface.
    return mean + np.minimum(0.5 * sigma, 0.020)


def keep_large_components(mask: np.ndarray, minimum: int) -> np.ndarray:
    from scipy import ndimage
    labels, count = ndimage.label(mask, structure=np.ones((3, 3), bool))
    if count == 0:
        return np.zeros_like(mask)
    sizes = np.bincount(labels.ravel())
    keep = sizes >= minimum
    keep[0] = False
    return keep[labels]


def high_side_of_water_steps(water_z: np.ndarray, valid: np.ndarray,
                             threshold=0.020, terrain_upper=None,
                             dense_terrain=None) -> np.ndarray:
    """Cells on the high side of a measured adjacent-water discontinuity."""
    seed = np.zeros(valid.shape, bool)
    for axis in (0, 1):
        z = water_z if axis == 0 else water_z.T
        v = valid if axis == 0 else valid.T
        pair = v[1:] & v[:-1] & np.isfinite(z[1:]) & np.isfinite(z[:-1])
        if terrain_upper is not None and dense_terrain is not None:
            upper = terrain_upper if axis == 0 else terrain_upper.T
            dense = dense_terrain if axis == 0 else dense_terrain.T
            # A step is hydraulically permitted only where dense measured
            # terrain on both sides is above both adjacent water levels.
            ridge = (dense[1:] & dense[:-1] &
                     (np.minimum(upper[1:], upper[:-1]) >
                      np.maximum(z[1:], z[:-1]) + 0.004))
            pair &= ~ridge
        high_next = pair & (z[1:] > z[:-1] + threshold)
        high_prev = pair & (z[:-1] > z[1:] + threshold)
        marked = np.zeros_like(v)
        marked[1:] |= high_next
        marked[:-1] |= high_prev
        seed |= marked if axis == 0 else marked.T
    return seed


def propagate_floating_terraces(step_high: np.ndarray, footprint: np.ndarray,
                                water_z: np.ndarray, floating_support: np.ndarray,
                                dense: np.ndarray) -> np.ndarray:
    """Grow proven high-side seeds across a coherent, terrain-backed sheet."""
    from scipy import ndimage
    if not step_high.any():
        return np.zeros_like(footprint)
    known = footprint & np.isfinite(water_z)
    filled = v6.fill_nearest(np.nan_to_num(water_z), known)[0]
    seeds, nseed = ndimage.label(step_high, structure=np.ones((3, 3), bool))
    result = np.zeros_like(footprint)
    for label in range(1, nseed + 1):
        seed = seeds == label
        levels = water_z[seed & np.isfinite(water_z)]
        if not len(levels):
            continue
        lower = float(np.quantile(levels, 0.20)) - 0.035
        candidate = footprint & (filled >= lower)
        region = ndimage.binary_propagation(
            seed, structure=np.ones((3, 3), bool), mask=candidate)
        cells = int(region.sum())
        if cells < MIN_EXPOSED_COMPONENT_CELLS:
            continue
        support_fraction = float((region & floating_support).sum() / cells)
        dense_fraction = float((region & dense).sum() / cells)
        if support_fraction >= 0.50 and dense_fraction >= 0.30:
            result |= region
    return result


def classify_terrain(water_reference: np.ndarray, footprint: np.ndarray,
                     evidence: dict, water_z: np.ndarray | None = None) -> dict:
    """Classify measured terrain as exposed, film anchor, or irrelevant.

    Dense original SAND below a visibly floating water sheet is exposed and
    carved (the marked comp1073 case).  A compact compatible surface within
    6 mm receives a film anchor.  Returns more than 150 mm below water are
    ignored and therefore can never drag the solve downward.
    """
    from scipy import ndimage
    count = evidence["terrain_count"]
    upper = robust_terrain_upper(evidence)
    dense = count >= DENSE_ORIGINAL_COUNT
    clearance = water_reference - upper
    above_water = dense & (upper > water_reference + 0.004)
    floating_dense = (footprint & dense &
                      (clearance >= EXPOSED_MIN_CLEARANCE) &
                      (clearance <= DEEP_RETURN_CLEARANCE))
    if water_z is None:
        step_high = np.zeros(footprint.shape, bool)
    else:
        step_high = high_side_of_water_steps(
            water_z, footprint, terrain_upper=upper, dense_terrain=dense)
    # Only propagate from the high side of a real water discontinuity.  Dense
    # shallow pools without a step remain valid water instead of being carved
    # merely because their bottom is measured.
    # Once a high-side seed is proven by dense original terrain, propagate
    # across its same measured sheet through sparsely sampled 25 mm cells.
    # A one-cell close bridges sampling gaps but cannot seed a carve itself.
    floating_support = (footprint & (count > 0) & np.isfinite(upper) &
                        (clearance >= EXPOSED_MIN_CLEARANCE) &
                        (clearance <= DEEP_RETURN_CLEARANCE))
    floating_support |= ndimage.binary_closing(
        floating_support, structure=v8.disk(1)) & footprint
    floating_over_terrain = propagate_floating_terraces(
        step_high & floating_dense, footprint, water_z, floating_support, dense)
    seed = footprint & np.isfinite(upper) & (above_water |
                                             floating_over_terrain)
    # Closing joins one-cell sampling gaps; the area gate prevents a lone
    # noisy upper return from becoming either a carve or a hydraulic ridge.
    joined = seed | ndimage.binary_closing(seed, structure=v8.disk(1))
    exposed = keep_large_components(joined, MIN_EXPOSED_COMPONENT_CELLS)
    exposed &= footprint & (np.isfinite(upper) | floating_over_terrain)

    moderate = count >= 4
    compatible = (footprint & ~exposed & moderate & np.isfinite(upper) &
                  (clearance >= -0.005) &
                  (clearance <= DEEP_RETURN_CLEARANCE))
    # Dense surfaces with >6 mm clearance have already been carved.  Sparse
    # surfaces can anchor only if they are close enough to be a plausible rim.
    compatible &= ((clearance <= EXPOSED_MIN_CLEARANCE) |
                   (count < DENSE_ORIGINAL_COUNT))
    film = np.where(compatible, upper + FILM_LIFT, np.nan)
    return {"terrain_upper": upper, "clearance": clearance,
            "dense_original": dense, "exposed": exposed,
        "film_compatible": compatible, "film_target": film,
        "step_high_seed": step_high,
        "floating_exposed": floating_over_terrain}


def robust_plane(x, y, z) -> np.ndarray:
    x = np.asarray(x, np.float64)
    y = np.asarray(y, np.float64)
    z = np.asarray(z, np.float64)
    if len(z) < 3:
        return np.array([0.0, 0.0, float(np.nanmedian(z)) if len(z) else 0.0])
    xc = float(np.mean(x))
    yc = float(np.mean(y))
    design = np.column_stack([x - xc, y - yc, np.ones(len(x))])
    keep = np.isfinite(z)
    coeff = np.linalg.lstsq(design[keep], z[keep], rcond=None)[0]
    for _ in range(5):
        residual = z - design @ coeff
        med = np.median(residual[keep])
        mad = 1.4826 * np.median(np.abs(residual[keep] - med)) + 1e-5
        new_keep = np.isfinite(z) & (np.abs(residual - med) < 3.5 * mad)
        if new_keep.sum() < 3 or np.array_equal(new_keep, keep):
            break
        keep = new_keep
        coeff = np.linalg.lstsq(design[keep], z[keep], rcond=None)[0]
    slope = float(np.hypot(coeff[0], coeff[1]))
    if slope > 0.05:
        coeff[:2] *= 0.05 / slope
    # Convert the centred intercept to world-coordinate form.
    return np.array([coeff[0], coeff[1],
                     coeff[2] - coeff[0] * xc - coeff[1] * yc])


def component_plane_targets(open_mask: np.ndarray, water_z: np.ndarray,
                            film_target: np.ndarray,
                            grid: v6.GridSpec) -> tuple[np.ndarray, np.ndarray,
                                                        list[dict]]:
    """Robust broad plane for each hydraulically connected open component."""
    from scipy import ndimage
    labels, nlabels = ndimage.label(open_mask, structure=np.ones((3, 3), bool))
    target = np.full(open_mask.shape, np.nan, np.float32)
    rows_report = []
    objects = ndimage.find_objects(labels)
    for label, obj in enumerate(objects, 1):
        if obj is None:
            continue
        local_rows, local_cols = np.nonzero(labels[obj] == label)
        rows = local_rows + obj[0].start
        cols = local_cols + obj[1].start
        if not len(rows):
            continue
        measured = water_z[rows, cols]
        finite = np.isfinite(measured)
        if finite.any():
            q65 = np.nanquantile(measured[finite], 0.65)
            use = finite & (measured <= q65)
        else:
            use = finite
        xs = grid.x0 + (cols[use] + 0.5) * grid.cell
        ys = grid.y0 + (rows[use] + 0.5) * grid.cell
        zs = measured[use]
        film = np.isfinite(film_target[rows, cols])
        if film.any():
            xs = np.r_[xs, grid.x0 + (cols[film] + 0.5) * grid.cell]
            ys = np.r_[ys, grid.y0 + (rows[film] + 0.5) * grid.cell]
            zs = np.r_[zs, film_target[rows[film], cols[film]]]
        if len(zs) == 0:
            continue
        if len(zs) > 60_000:
            # Deterministic spatial subsampling bounds the robust fit memory;
            # the solved graph still contains every cell.
            take = np.linspace(0, len(zs) - 1, 60_000, dtype=np.int64)
            xs, ys, zs = xs[take], ys[take], zs[take]
        coeff = robust_plane(xs, ys, zs)
        all_x = grid.x0 + (cols + 0.5) * grid.cell
        all_y = grid.y0 + (rows + 0.5) * grid.cell
        target[rows, cols] = (coeff[0] * all_x + coeff[1] * all_y +
                              coeff[2]).astype(np.float32)
        rows_report.append({"label": label, "cells": int(len(rows)),
                            "slope": float(np.hypot(*coeff[:2])),
                            "level_median": float(np.median(target[rows, cols]))})
    return target, np.asarray(labels, np.int32), rows_report


def solve_connected_surface(open_mask: np.ndarray, water_z: np.ndarray,
                            film_target: np.ndarray,
                            grid: v6.GridSpec) -> tuple[np.ndarray, dict]:
    """Solve a weighted graph surface; unknown terrain creates no edge cut."""
    from scipy import sparse
    from scipy.sparse.linalg import LinearOperator, cg, minres

    plane, labels, component_rows = component_plane_targets(
        open_mask, water_z, film_target, grid)
    rows, cols = np.nonzero(open_mask)
    n = len(rows)
    if n == 0:
        raise RuntimeError("v9 footprint is empty")
    node = np.full(open_mask.shape, -1, np.int32)
    node[rows, cols] = np.arange(n, dtype=np.int32)

    plane_v = plane[rows, cols].astype(np.float64)
    raw_v = water_z[rows, cols].astype(np.float64)
    film_v = film_target[rows, cols].astype(np.float64)
    w = np.full(n, PLANE_WEIGHT, np.float64)
    b = PLANE_WEIGHT * plane_v
    have_water = np.isfinite(raw_v)
    clipped = np.clip(raw_v, plane_v - WATER_ANCHOR_CLAMP,
                      plane_v + WATER_ANCHOR_CLAMP)
    w[have_water] += WATER_WEIGHT
    b[have_water] += WATER_WEIGHT * clipped[have_water]
    have_film = np.isfinite(film_v)
    w[have_film] += TERRAIN_ANCHOR_WEIGHT
    b[have_film] += TERRAIN_ANCHOR_WEIGHT * film_v[have_film]

    edge_a = []
    edge_b = []
    right = open_mask[:, :-1] & open_mask[:, 1:]
    rr, rc = np.nonzero(right)
    edge_a.append(node[rr, rc])
    edge_b.append(node[rr, rc + 1])
    up = open_mask[:-1, :] & open_mask[1:, :]
    ur, uc = np.nonzero(up)
    edge_a.append(node[ur, uc])
    edge_b.append(node[ur + 1, uc])
    ea = np.concatenate(edge_a)
    eb = np.concatenate(edge_b)
    degree = np.bincount(np.r_[ea, eb], minlength=n).astype(np.float64)
    diag = w + LAPLACIAN_WEIGHT * degree
    matrix = sparse.coo_matrix((
        np.r_[diag,
              np.full(len(ea), -LAPLACIAN_WEIGHT),
              np.full(len(eb), -LAPLACIAN_WEIGHT)],
        (np.r_[np.arange(n), ea, eb], np.r_[np.arange(n), eb, ea])),
        shape=(n, n)).tocsr()
    initial = np.where(have_film, film_v, plane_v)
    if not (np.isfinite(matrix.data).all() and np.isfinite(b).all() and
            np.isfinite(initial).all()):
        raise RuntimeError("connected surface system contains non-finite data")

    # The global matrix is block diagonal (one block per open component) and
    # positive definite because every cell carries PLANE_WEIGHT.  A Jacobi
    # preconditioner removes the degree/anchor scale disparity that made the
    # first real 3291x2296 solve hit the unpreconditioned 700-iteration cap.
    inv_diag = 1.0 / np.maximum(diag, 1e-12)
    preconditioner = LinearOperator(
        matrix.shape, matvec=lambda value: inv_diag * value,
        rmatvec=lambda value: inv_diag * value, dtype=np.float64)
    iterations = 0

    def count_iteration(_):
        nonlocal iterations
        iterations += 1

    rhs_norm = max(float(np.linalg.norm(b)), 1e-12)
    solved, info = cg(
        matrix, b, x0=initial, M=preconditioner, rtol=1e-5, atol=1e-8,
        maxiter=1800, callback=count_iteration)
    relative_residual = float(np.linalg.norm(matrix @ solved - b) / rhs_norm) \
        if np.isfinite(solved).all() else math.inf
    method = "cg-jacobi"
    fallback_info = None
    # MINRES is a conservative symmetric fallback.  It starts from the CG
    # iterate and uses the same positive Jacobi preconditioner; acceptance is
    # based on the measured residual rather than the library status alone.
    if info != 0 or relative_residual > 5e-5:
        minres_iterations = 0

        def count_minres(_):
            nonlocal minres_iterations
            minres_iterations += 1

        solved2, fallback_info = minres(
            matrix, b, x0=np.where(np.isfinite(solved), solved, initial),
            M=preconditioner, rtol=1e-5, maxiter=3000,
            callback=count_minres, check=False)
        residual2 = float(np.linalg.norm(matrix @ solved2 - b) / rhs_norm) \
            if np.isfinite(solved2).all() else math.inf
        if residual2 < relative_residual:
            solved = solved2
            relative_residual = residual2
            method = "minres-jacobi"
            iterations += minres_iterations
    if not np.isfinite(solved).all() or relative_residual > 5e-5:
        raise RuntimeError(
            "connected surface solve did not converge: "
            f"cg_info={info}, fallback_info={fallback_info}, "
            f"relative_residual={relative_residual:.3e}")
    surface = np.full(open_mask.shape, np.nan, np.float32)
    surface[rows, cols] = solved.astype(np.float32)
    report = {"components": int(labels.max()), "open_cells": int(n),
              "film_anchor_cells": int(have_film.sum()),
              "linear_solver": {"method": method,
                                "iterations": int(iterations),
                                "cg_info": int(info),
                                "fallback_info": (None if fallback_info is None
                                                  else int(fallback_info)),
                                "relative_residual": relative_residual},
              "components_detail": component_rows}
    return surface, report


def prune_unanchored_components(open_mask: np.ndarray, water_z: np.ndarray,
                                film_target: np.ndarray) -> tuple[np.ndarray, dict]:
    """Remove closure-only fragments split off by exposed-terrain carving."""
    from scipy import ndimage
    labels, count = ndimage.label(open_mask, structure=np.ones((3, 3), bool))
    evidence = open_mask & (np.isfinite(water_z) | np.isfinite(film_target))
    has_evidence = np.bincount(labels[evidence].ravel(), minlength=count + 1) > 0
    has_evidence[0] = False
    kept = open_mask & has_evidence[labels]
    dropped = open_mask & ~kept
    dropped_labels = np.unique(labels[dropped])
    dropped_labels = dropped_labels[dropped_labels > 0]
    return kept, {"cells": int(dropped.sum()),
                  "components": int(len(dropped_labels))}


def max_open_neighbour_step(surface: np.ndarray,
                            open_mask: np.ndarray) -> float:
    maximum = 0.0
    for axis in (0, 1):
        values = surface if axis == 0 else surface.T
        mask = open_mask if axis == 0 else open_mask.T
        pair = mask[:, 1:] & mask[:, :-1]
        if pair.any():
            maximum = max(maximum, float(np.max(
                np.abs(values[:, 1:] - values[:, :-1])[pair])))
    return maximum


def project_open_surface_gradient(surface: np.ndarray, open_mask: np.ndarray,
                                  limit=MAX_OPEN_NEIGHBOUR_STEP,
                                  max_sweeps=240) -> tuple[np.ndarray, dict]:
    """Project direct open-water neighbours onto a bounded height gradient.

    Exposed terrain has already been removed from ``open_mask``.  Therefore
    every remaining direct edge is hydraulically open and cannot retain a
    discontinuity.  Red/black edge passes preserve each pair's mean while
    removing only the excess above ``limit``.
    """
    work = surface.astype(np.float64, copy=True)
    before = max_open_neighbour_step(work, open_mask)
    sweeps = 0
    for sweeps in range(1, max_sweeps + 1):
        changed = False
        for axis in (0, 1):
            values = work if axis == 0 else work.T
            mask = open_mask if axis == 0 else open_mask.T
            for parity in (0, 1):
                left = values[:, parity:-1:2]
                right = values[:, parity + 1::2]
                valid = (mask[:, parity:-1:2] &
                         mask[:, parity + 1::2])
                difference = right - left
                excess = np.maximum(np.abs(difference) - limit, 0.0)
                active = valid & (excess > 1e-7)
                if not active.any():
                    continue
                delta = 0.5 * np.sign(difference[active]) * excess[active]
                left[active] += delta
                right[active] -= delta
                changed = True
        if not changed or max_open_neighbour_step(work, open_mask) <= limit + 1e-6:
            break
    after = max_open_neighbour_step(work, open_mask)
    work[~open_mask] = np.nan
    return work.astype(np.float32), {
        "limit_mm": float(limit * 1000), "sweeps": int(sweeps),
        "before_max_mm": float(before * 1000),
        "after_max_mm": float(after * 1000),
    }


def build_surface(evidence: dict, grid: v6.GridSpec, log):
    from scipy import ndimage
    wc = evidence["water_count"]
    raw = wc > 0
    water_z = finite_mean(evidence["water_sum"], wc)
    exclusion = bbox_mask(grid)
    # Repair suppression trenches up to 75 mm.  There is no post-solve
    # adoption of earlier water cells or external generations: this closure
    # is derived solely from the installed v8 footprint and remains inside
    # the exact v9 exclusion gate.
    closed = ndimage.binary_closing(
        raw, structure=v8.disk(FOOTPRINT_CLOSE_CELLS))
    footprint = (raw | closed) & ~exclusion
    water_ref = v6.normalised_gaussian(
        np.nan_to_num(water_z), raw, sigma=8.0)
    missing = ~np.isfinite(water_z)
    water_ref[missing] = v6.fill_nearest(water_ref, np.isfinite(water_ref))[0][missing]
    terrain = classify_terrain(water_ref, footprint, evidence, water_z)
    cumulative_exposed = terrain["exposed"].copy()
    reference = water_ref
    pass_reports = []
    total_pruned = {"cells": 0, "components": 0}
    surface = None
    open_mask = None
    solve_report = None
    projection_report = None
    for pass_index in range(MAX_POST_SOLVE_PASSES):
        remaining = footprint & ~cumulative_exposed
        if pass_index:
            terrain = classify_terrain(reference, remaining, evidence, reference)
            cumulative_exposed |= terrain["exposed"]
            remaining &= ~terrain["exposed"]
        open_mask, pruned = prune_unanchored_components(
            remaining, water_z, terrain["film_target"])
        total_pruned["cells"] += pruned["cells"]
        total_pruned["components"] += pruned["components"]
        surface, solve_report = solve_connected_surface(
            open_mask, water_z, terrain["film_target"], grid)

        # Re-evaluate against the solved height.  This catches water pulled
        # below measured terrain and newly revealed high terraces; both were
        # impossible to identify reliably from the pre-solve v8 surface.
        post = classify_terrain(surface, open_mask, evidence, surface)
        new_exposed = post["exposed"] & ~cumulative_exposed
        pass_row = {
            "pass": pass_index + 1,
            "open_cells": int(open_mask.sum()),
            "new_postsolve_exposed_cells": int(new_exposed.sum()),
            "linear_solver": solve_report["linear_solver"],
        }
        if new_exposed.any():
            cumulative_exposed |= new_exposed
            reference = surface
            pass_reports.append(pass_row)
            log(f"surface pass {pass_index + 1}: post-solve carve "
                f"{int(new_exposed.sum()):,} cells")
            continue

        projected, projection_report = project_open_surface_gradient(
            surface, open_mask)
        # Projection can lower a cell onto terrain; run one more evidence gate
        # before accepting it as the final water sheet.
        post_project = classify_terrain(
            projected, open_mask, evidence, projected)
        projected_exposed = post_project["exposed"] & ~cumulative_exposed
        pass_row["projection"] = projection_report
        pass_row["new_postprojection_exposed_cells"] = int(
            projected_exposed.sum())
        pass_reports.append(pass_row)
        if projected_exposed.any():
            cumulative_exposed |= projected_exposed
            reference = projected
            log(f"surface pass {pass_index + 1}: post-projection carve "
                f"{int(projected_exposed.sum()):,} cells")
            continue
        surface = projected
        break
    else:
        raise RuntimeError("post-solve terrain classification did not stabilise")

    terrain["exposed"] = cumulative_exposed
    terrain["film_compatible"] &= open_mask
    terrain["film_target"][~open_mask] = np.nan
    edge_distance = ndimage.distance_transform_edt(open_mask).astype(np.float32) * grid.cell
    model, ripple_report = v6.build_surface_model(
        grid, open_mask, surface, edge_distance)
    report = {
        "source_cells": int(raw.sum()),
        "closed_gap_cells": int((footprint & ~raw).sum()),
        "excluded_source_cells": int((raw & exclusion).sum()),
        "exposed_cells_carved": int(terrain["exposed"].sum()),
        "dense_original_cells": int(terrain["dense_original"].sum()),
        "deep_returns_ignored": int((terrain["clearance"] >
                                     DEEP_RETURN_CLEARANCE).sum()),
        "unanchored_fragments_pruned": total_pruned,
        "solve_passes": pass_reports,
        "gradient_projection": projection_report,
        "solve": solve_report, "ripple": ripple_report,
    }
    log(f"surface: open {int(open_mask.sum()):,}, exposed carve "
        f"{int(terrain['exposed'].sum()):,}, components "
        f"{solve_report['components']:,}")
    return model, terrain, exclusion, report


def terrain_near_surface_count(data_dir: Path, model: v6.SurfaceModel,
                               log) -> np.ndarray:
    """Measured terrain within 12 mm of the solved sheet; deep returns ignored."""
    from scipy.ndimage import map_coordinates
    grid = model.grid
    count = np.zeros(grid.shape, np.int16)
    flat = count.ravel()
    for role in ("SAND", "ROCK", "VEG"):
        mm = v6.memmap_cloud(data_dir / f"Site1-{role}-5mm.ply")
        mm = mm[0] if isinstance(mm, tuple) else mm
        for start in range(0, len(mm), CHUNK):
            c = mm[start:start + CHUNK]
            keep = ((c["x"] >= grid.x0) & (c["x"] < grid.x1) &
                    (c["y"] >= grid.y0) & (c["y"] < grid.y1) &
                    (c["scalar_ScanID"] != 9.0))
            if not keep.any():
                continue
            idx = np.nonzero(keep)[0]
            x = c["x"][idx]
            y = c["y"][idx]
            u = (x - grid.x0) / grid.cell - 0.5
            v = (y - grid.y0) / grid.cell - 0.5
            sz = map_coordinates(model.z, [v, u], order=1, mode="nearest")
            wet = map_coordinates(model.occ.astype(np.uint8), [v, u],
                                   order=0, mode="constant") > 0
            near = wet & (np.abs(c["z"][idx] - sz) <= 0.012)
            gx, gy = grid.indices(x[near], y[near])
            ok = (gx >= 0) & (gx < grid.nx) & (gy >= 0) & (gy < grid.ny)
            keys = gy[ok].astype(np.int64) * grid.nx + gx[ok]
            np.add.at(flat, keys, 1)
    log(f"near-surface original terrain points: {int(count.sum()):,}")
    return count


# A spatially spread 5x5 order.  Prefixes remain distributed rather than
# forming a block when shore cells receive the 20-point complementary floor.
SUBCELL_ORDER = np.array([
    0, 12, 24, 4, 20, 10, 14, 2, 22, 6, 18, 8, 16,
    1, 23, 5, 19, 11, 13, 3, 21, 7, 17, 9, 15,
], np.int16)


def shoreline_quota(open_mask: np.ndarray,
                    terrain_near: np.ndarray) -> np.ndarray:
    quota = np.zeros(open_mask.shape, np.int16)
    complement = TARGET_POINTS_PER_CELL - np.minimum(
        terrain_near.astype(np.int32), TARGET_POINTS_PER_CELL)
    quota[open_mask] = np.clip(complement[open_mask],
                               MIN_SHORE_POINTS_PER_CELL,
                               TARGET_POINTS_PER_CELL)
    return quota


def generate_points(model: v6.SurfaceModel, quota: np.ndarray,
                    exclusion: np.ndarray, log) -> np.ndarray:
    """Generate a 5 mm lattice with no edge/seam acceptance suppression."""
    grid = model.grid
    rows, cols = np.nonzero(quota > 0)
    total = int(quota[rows, cols].sum())
    xyz = np.empty((total, 3), np.float32)
    cursor = 0
    for start in range(0, len(rows), 100_000):
        rr = rows[start:start + 100_000]
        cc = cols[start:start + 100_000]
        qq = quota[rr, cc].astype(np.int32)
        shift = (v8.hash01(cc.astype(np.int64), rr.astype(np.int64), 313) *
                 25).astype(np.int32)
        rank = (np.arange(25)[None, :] + shift[:, None]) % 25
        take = rank < qq[:, None]
        cell_row, order = np.nonzero(take)
        chosen = SUBCELL_ORDER[order]
        x = grid.x0 + (cc[cell_row] + (chosen % 5 + 0.5) / 5.0) * grid.cell
        y = grid.y0 + (rr[cell_row] + (chosen // 5 + 0.5) / 5.0) * grid.cell
        # Bounded sub-millimetre XY jitter breaks a visible lattice without
        # crossing the 25 mm evidence cell or the exact exclusion gate.
        global_x = cc[cell_row].astype(np.int64) * 5 + chosen % 5
        global_y = rr[cell_row].astype(np.int64) * 5 + chosen // 5
        x += (v8.hash01(global_x, global_y, 331) - 0.5) * 0.0005
        y += (v8.hash01(global_x, global_y, 337) - 0.5) * 0.0005
        z = v6.sample_grid(model.z, x, y, grid)
        count = len(x)
        xyz[cursor:cursor + count, 0] = x
        xyz[cursor:cursor + count, 1] = y
        xyz[cursor:cursor + count, 2] = z
        cursor += count
    xyz = xyz[:cursor]
    gx, gy = grid.indices(xyz[:, 0], xyz[:, 1])
    inside = ((gx >= 0) & (gx < grid.nx) & (gy >= 0) & (gy < grid.ny))
    valid = inside.copy()
    valid[inside] &= model.occ[gy[inside], gx[inside]]
    valid &= ~points_in_exclusion(xyz[:, 0], xyz[:, 1])
    if not valid.all():
        xyz = xyz[valid]
    log(f"generated {len(xyz):,} WATER 5mm points; exact-gate drops "
        f"{int((~valid).sum()):,}")
    return xyz


def _donor_grid(grid: v6.GridSpec) -> v6.GridSpec:
    x0 = math.floor(grid.x0 / DONOR_CELL) * DONOR_CELL
    y0 = math.floor(grid.y0 / DONOR_CELL) * DONOR_CELL
    x1 = math.ceil(grid.x1 / DONOR_CELL) * DONOR_CELL
    y1 = math.ceil(grid.y1 / DONOR_CELL) * DONOR_CELL
    return v6.GridSpec(x0, y0, x1, y1, DONOR_CELL)


def diffuse_donor(mean: np.ndarray, valid: np.ndarray) -> np.ndarray:
    """Deterministic masked mean fill; deliberately adds no variance noise."""
    smooth = v6.normalised_gaussian(np.nan_to_num(mean), valid, 0.8)
    have = v6.normalised_gaussian(valid.astype(np.float32), valid, 0.8) > 0
    smooth_valid = valid | have
    if not smooth_valid.all():
        smooth = v6.fill_nearest(smooth, smooth_valid)[0]
    # Exact nearest fill catches remote cells outside the donor footprint.
    return v6.fill_nearest(smooth, np.isfinite(smooth) & smooth_valid)[0]


def build_donor_bundle(data_dir: Path, surface_grid: v6.GridSpec, log,
                       donor_paths=None) -> dict:
    """Public scalar-fill API: bounded local donor means, no random wobble."""
    grid = _donor_grid(surface_grid)
    names = tuple(FIELD_BOUNDS)
    sums = {name: np.zeros(grid.shape, np.float64) for name in names}
    counts = {name: np.zeros(grid.shape, np.int32) for name in names}
    if donor_paths is None:
        donor_paths = [data_dir / "Site1-WATER-5mm.ply",
                       data_dir / "Site1-SAND-5mm.ply",
                       data_dir / "Site1-ROCK-5mm.ply",
                       data_dir / "Site1-VEG-5mm.ply"]
    for path in donor_paths:
        mm = v6.memmap_cloud(Path(path))
        mm = mm[0] if isinstance(mm, tuple) else mm
        for start in range(0, len(mm), CHUNK):
            c = mm[start:start + CHUNK]
            spatial = ((c["x"] >= grid.x0) & (c["x"] < grid.x1) &
                       (c["y"] >= grid.y0) & (c["y"] < grid.y1))
            if not spatial.any():
                continue
            idx = np.nonzero(spatial)[0]
            gx, gy = grid.indices(c["x"][idx], c["y"][idx])
            key = gy.astype(np.int64) * grid.nx + gx
            for name in names:
                lo, hi = FIELD_BOUNDS[name]
                value = np.asarray(c[name][idx], np.float64)
                good = np.isfinite(value) & (value >= lo) & (value <= hi)
                if not good.any():
                    continue
                aggregate = np.log(value[good]) if name == "scalar_Intensity" \
                    else value[good]
                np.add.at(sums[name].ravel(), key[good], aggregate)
                np.add.at(counts[name].ravel(), key[good], 1)
    fields = {}
    for name in names:
        mean = finite_mean(sums[name], counts[name])
        if name == "scalar_Intensity":
            mean = np.exp(mean)
        fields[name] = diffuse_donor(mean, counts[name] > 0).astype(np.float32)
        lo, hi = FIELD_BOUNDS[name]
        fields[name] = np.clip(fields[name], lo, hi)
        log(f"donor {name}: {int((counts[name] > 0).sum()):,} cells")
    return {"grid": grid, "fields": fields}


def apply_donor_bundle(record: np.ndarray, bundle: dict) -> None:
    """Public scalar-fill API, separated for the parent all-cloud pass."""
    grid = bundle["grid"]
    for name, values in bundle["fields"].items():
        sampled = v6.sample_grid(values, record["x"], record["y"], grid)
        lo, hi = FIELD_BOUNDS[name]
        sampled = np.clip(sampled, lo, hi)
        if record.dtype[name].kind == "u":
            record[name] = np.rint(sampled).astype(record.dtype[name])
        else:
            record[name] = sampled.astype(record.dtype[name])


def build_attributes(data_dir: Path, xyz: np.ndarray, model: v6.SurfaceModel,
                     log, scalar_fill=None) -> np.ndarray:
    source = v6.memmap_cloud(data_dir / "Site1-SAND-5mm.ply")
    source = source[0] if isinstance(source, tuple) else source
    record = np.zeros(len(xyz), source.dtype)
    record["x"], record["y"], record["z"] = xyz.T
    # This call is the v6 masked-gradient implementation and applies its tanh
    # curvature/recession plus normalised roughness combined formulas.
    v6.apply_water_surface(record, model)
    if scalar_fill is None:
        bundle = build_donor_bundle(data_dir, model.grid, log)
        apply_donor_bundle(record, bundle)
    else:
        scalar_fill(record, data_dir, model.grid, log)
    record["scalar_ScanID"] = WATER_SCAN_ID
    return record


def write_ply(path: Path, record: np.ndarray, comments: list[str]):
    v8.write_ply(path, record, comments)


def density_band_metrics(count: np.ndarray, wet: np.ndarray,
                         cell=CELL) -> dict:
    from scipy import ndimage
    distance = ndimage.distance_transform_edt(wet) * cell
    bands = ((0.0, 0.025), (0.025, 0.050), (0.050, 0.100),
             (0.100, 0.200), (0.200, np.inf))
    report = {}
    for lo, hi in bands:
        mask = wet & (distance > lo) & (distance <= hi)
        values = count[mask]
        key = f"{int(lo * 1000)}-{('inf' if not np.isfinite(hi) else int(hi * 1000))}mm"
        report[key] = {
            "cells": int(len(values)),
            "q05": float(np.quantile(values, 0.05)) if len(values) else None,
            "median": float(np.median(values)) if len(values) else None,
            "q95": float(np.quantile(values, 0.95)) if len(values) else None,
        }
    return report


def neighbour_step_metrics(zgrid: np.ndarray, wet: np.ndarray,
                           grid: v6.GridSpec, focus=None) -> dict:
    values = []
    transitions = []
    for axis in (0, 1):
        z = zgrid if axis == 0 else zgrid.T
        w = wet if axis == 0 else wet.T
        both = w[1:] & w[:-1]
        dz = np.abs(z[1:] - z[:-1])
        rr, cc = np.nonzero(both)
        vals = dz[both]
        values.append(vals)
        if len(vals):
            take = np.argpartition(vals, max(0, len(vals) - min(12, len(vals))))[-12:]
            for idx in take:
                r = int(rr[idx])
                c = int(cc[idx])
                if axis == 0:
                    x = grid.x0 + (c + 0.5) * grid.cell
                    y = grid.y0 + (r + 1.0) * grid.cell
                else:
                    x = grid.x0 + (r + 1.0) * grid.cell
                    y = grid.y0 + (c + 0.5) * grid.cell
                transitions.append((float(vals[idx]), x, y))
    joined = np.concatenate(values) if values else np.zeros(0)
    result = {
        "pairs": int(len(joined)),
        "q50_mm": float(np.quantile(joined, 0.50) * 1000) if len(joined) else 0,
        "q95_mm": float(np.quantile(joined, 0.95) * 1000) if len(joined) else 0,
        "q99_mm": float(np.quantile(joined, 0.99) * 1000) if len(joined) else 0,
        "q999_mm": float(np.quantile(joined, 0.999) * 1000) if len(joined) else 0,
        "max_mm": float(np.max(joined) * 1000) if len(joined) else 0,
        "gt10mm": int((joined > 0.010).sum()),
        "worst": [{"dz_mm": d * 1000, "x": x, "y": y}
                  for d, x, y in sorted(transitions, reverse=True)[:10]],
    }
    if focus is not None:
        x0, x1, y0, y1 = focus
        gx0 = max(0, int((x0 - grid.x0) / grid.cell))
        gx1 = min(grid.nx, int(math.ceil((x1 - grid.x0) / grid.cell)))
        gy0 = max(0, int((y0 - grid.y0) / grid.cell))
        gy1 = min(grid.ny, int(math.ceil((y1 - grid.y0) / grid.cell)))
        sub = zgrid[gy0:gy1, gx0:gx1]
        sm = wet[gy0:gy1, gx0:gx1]
        result["focus"] = neighbour_step_metrics(
            sub, sm,
            v6.GridSpec(grid.x0 + gx0 * grid.cell,
                        grid.y0 + gy0 * grid.cell,
                        grid.x0 + gx1 * grid.cell,
                        grid.y0 + gy1 * grid.cell, grid.cell))
    return result


def write_reviews(run: Path, evidence: dict, model: v6.SurfaceModel,
                  terrain: dict, quota: np.ndarray):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    extent = [model.grid.x0, model.grid.x1, model.grid.y0, model.grid.y1]
    fig, axes = plt.subplots(1, 3, figsize=(18, 10), dpi=120)
    axes[0].imshow(np.where(model.occ, model.z, np.nan), origin="lower",
                   extent=extent, cmap="viridis")
    axes[0].set_title("v9 connected water height")
    classmap = np.zeros(model.grid.shape, np.uint8)
    classmap[model.occ] = 1
    classmap[terrain["film_compatible"]] = 2
    classmap[terrain["exposed"]] = 3
    axes[1].imshow(classmap, origin="lower", extent=extent,
                   cmap="viridis", vmin=0, vmax=3)
    axes[1].set_title("0 dry / 1 water / 2 film / 3 exposed carve")
    axes[2].imshow(np.where(model.occ, quota, np.nan), origin="lower",
                   extent=extent, cmap="magma", vmin=0, vmax=25)
    axes[2].set_title("5 mm quota (20-25 through shore)")
    for ax in axes:
        ax.set_xlim(FOCUS_BOX[0] - 1.2, FOCUS_BOX[1] + 1.2)
        ax.set_ylim(FOCUS_BOX[2] - 1.2, FOCUS_BOX[3] + 1.2)
    fig.tight_layout()
    fig.savefig(run / "review-focus-connected-solve.png", bbox_inches="tight")
    plt.close(fig)


def build(args):
    run = args.run_dir
    run.mkdir(parents=True, exist_ok=True)
    (args.data_dir / "PatchRefinement/.invisible_places-ignore").touch()
    source = args.data_dir / "Site1-WATER-5mm.ply"
    with open(run / "build.log", "a") as logfile:
        def log(message):
            log_line("build", message, logfile)
        grid = aligned_water_grid(args.data_dir)
        evidence = stream_evidence(args.data_dir, grid, log)
        model, terrain, exclusion, surface_report = build_surface(
            evidence, grid, log)
        near = terrain_near_surface_count(args.data_dir, model, log)
        quota = shoreline_quota(model.occ, near)
        xyz = generate_points(model, quota, exclusion, log)
        record = build_attributes(args.data_dir, xyz, model, log)
        candidate = run / "Site1-WATER-5mm.candidate.ply"
        comments = [
            f"Site1 WATER v9 connected reconstruction {dt.date.today().isoformat()}",
            "Measured exposed terrain carved; compatible terrain film +2.5 mm.",
            "Unknown terrain is not a dam; connected graph surface solve.",
            "Uniform complementary 5 mm sampling; no seam suppression.",
            "Bounded deterministic donor means; v6 masked geometry formulas.",
            f"ScanID={WATER_SCAN_ID:.0f}",
            "Generated by scripts/rebuild_site1_fossils_v9.py",
        ]
        write_ply(candidate, record, comments)
        np.savez_compressed(
            run / "surface-v9.npz", base_surface=np.where(model.occ, model.z - model.ripple, np.nan),
            surface=model.z, wet=model.occ, exposed=terrain["exposed"],
            film=terrain["film_compatible"], terrain_upper=terrain["terrain_upper"],
            terrain_count=evidence["terrain_count"], quota=quota,
            exclusion=exclusion, meta=np.array([grid.x0, grid.y0, grid.x1,
                                                grid.y1, grid.cell]))
        write_reviews(run, evidence, model, terrain, quota)
        manifest = {
            "version": 9,
            "created": dt.datetime.now().isoformat(timespec="seconds"),
            "data_dir": str(args.data_dir), "run_dir": str(run),
            "candidate": str(candidate), "candidate_points": int(len(record)),
            "candidate_sha256": sha256_path(candidate),
            "source_water_sha256": sha256_path(source),
            "source_water_fingerprint": source_fingerprint(source),
            "terrain_sources": {
                role: source_fingerprint(args.data_dir / f"Site1-{role}-5mm.ply")
                for role in ("SAND", "ROCK", "VEG")},
            "surface_report": surface_report,
            "installed": False,
        }
        (run / "manifest.json").write_text(json.dumps(manifest, indent=2))
        log(f"candidate {len(record):,} points, "
            f"{candidate.stat().st_size / 1e9:.2f} GB")


def _grid_from_archive(saved) -> v6.GridSpec:
    x0, y0, x1, y1, cell = saved["meta"]
    return v6.GridSpec(float(x0), float(y0), float(x1), float(y1), float(cell))


def verify(args, candidate: Path | None = None) -> dict:
    run = args.run_dir
    manifest = json.loads((run / "manifest.json").read_text())
    candidate = candidate or Path(manifest["candidate"])
    saved = np.load(run / "surface-v9.npz")
    grid = _grid_from_archive(saved)
    wet = saved["wet"].astype(bool)
    terrain_upper = saved["terrain_upper"]
    terrain_count = saved["terrain_count"]
    mm = v6.memmap_cloud(candidate)
    mm = mm[0] if isinstance(mm, tuple) else mm
    failures = []
    finite_bad = {name: 0 for name in mm.dtype.names
                  if mm.dtype[name].kind == "f"}
    bound_bad = {name: 0 for name in {**FIELD_BOUNDS, **COMBINED_BOUNDS}}
    wrong_scan = outside = excluded = 0
    count = np.zeros(grid.shape, np.int32)
    zsum = np.zeros(grid.shape, np.float64)
    for start in range(0, len(mm), CHUNK):
        c = mm[start:start + CHUNK]
        wrong_scan += int((c["scalar_ScanID"] != WATER_SCAN_ID).sum())
        for name in finite_bad:
            finite_bad[name] += int((~np.isfinite(c[name])).sum())
        for name, (lo, hi) in {**FIELD_BOUNDS, **COMBINED_BOUNDS}.items():
            val = np.asarray(c[name], np.float64)
            bound_bad[name] += int(((val < lo) | (val > hi) |
                                    ~np.isfinite(val)).sum())
        excluded += int(points_in_exclusion(c["x"], c["y"]).sum())
        gx, gy = grid.indices(c["x"], c["y"])
        inside = ((gx >= 0) & (gx < grid.nx) &
                  (gy >= 0) & (gy < grid.ny))
        accepted = np.zeros(len(c), bool)
        accepted[inside] = wet[gy[inside], gx[inside]]
        outside += int((~accepted).sum())
        keys = gy[accepted].astype(np.int64) * grid.nx + gx[accepted]
        np.add.at(count.ravel(), keys, 1)
        np.add.at(zsum.ravel(), keys, c["z"][accepted].astype(np.float64))
    zgrid = finite_mean(zsum, count)
    occupied = count > 0
    step = neighbour_step_metrics(zgrid, occupied, grid, FOCUS_BOX)
    density = density_band_metrics(count, wet)
    clearance = zgrid - terrain_upper
    compatible = wet & (terrain_count >= 4) & np.isfinite(clearance) & \
        (clearance >= -0.02) & (clearance <= DEEP_RETURN_CLEARANCE)
    cv = clearance[compatible]
    floating = {
        "cells": int(len(cv)),
        "q05_mm": float(np.quantile(cv, 0.05) * 1000) if len(cv) else None,
        "median_mm": float(np.median(cv) * 1000) if len(cv) else None,
        "q95_mm": float(np.quantile(cv, 0.95) * 1000) if len(cv) else None,
        "gt20mm": int((cv > 0.020).sum()),
    }
    if len(mm) != manifest["candidate_points"]:
        failures.append("candidate point count mismatch")
    if wrong_scan:
        failures.append(f"wrong ScanID: {wrong_scan}")
    if any(finite_bad.values()):
        failures.append("non-finite scalar or coordinate fields")
    if any(bound_bad.values()):
        failures.append("one or more bounded scalar fields are out of range")
    if excluded:
        failures.append(f"strict exclusion contains {excluded} points")
    if outside:
        failures.append(f"outside footprint contains {outside} points")
    shore = density["0-25mm"]
    if shore["q05"] is not None and shore["q05"] < MIN_SHORE_POINTS_PER_CELL:
        failures.append("shoreline density falls below 20/25 points")
    if step["q99_mm"] > 10.0 or step["max_mm"] > 40.0:
        failures.append("connected water retains a material open transition")
    checks = {
        "points": int(len(mm)), "wrong_scan": wrong_scan,
        "finite_bad": finite_bad, "scalar_bound_bad": bound_bad,
        "excluded_points": excluded, "outside_points": outside,
        "wet_cells_expected": int(wet.sum()),
        "wet_cells_occupied": int(occupied.sum()),
        "density_signed_distance_bands": density,
        "local_steps": step, "floating_clearance": floating,
    }
    report = {"created": dt.datetime.now().isoformat(timespec="seconds"),
              "candidate": str(candidate), "checks": checks,
              "failures": failures, "verified": not failures}
    (run / "verification-report.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(checks, indent=1), flush=True)
    print("FAILURES:", failures if failures else "none", flush=True)
    return report


def install(args):
    run = args.run_dir
    manifest_path = run / "manifest.json"
    manifest = json.loads(manifest_path.read_text())
    if app_running():
        raise SystemExit("refusing: invisible_places is running")
    report = verify(args)
    if report["failures"]:
        raise SystemExit(f"verification failed: {report['failures']}")
    canonical = args.data_dir / "Site1-WATER-5mm.ply"
    if sha256_path(canonical) != manifest["source_water_sha256"]:
        raise RuntimeError("canonical WATER changed since the v9 build")
    preserve = run / "Site1-WATER-5mm.v8-installed.ply"
    if preserve.exists():
        raise RuntimeError(f"refusing to overwrite v8 backup {preserve}")
    shutil.move(str(canonical), str(preserve))
    try:
        shutil.copy2(manifest["candidate"], canonical)
        if sha256_path(canonical) != manifest["candidate_sha256"]:
            raise RuntimeError("installed v9 hash mismatch")
    except Exception:
        if canonical.exists():
            canonical.unlink()
        shutil.move(str(preserve), str(canonical))
        raise
    manifest["installed"] = True
    manifest["installed_at"] = dt.datetime.now().isoformat(timespec="seconds")
    manifest["v8_backup"] = str(preserve)
    manifest_path.write_text(json.dumps(manifest, indent=2))
    log_line("install", "v9 WATER installed; byte-exact v8 retained in run")


def restore(args):
    run = args.run_dir
    manifest_path = run / "manifest.json"
    manifest = json.loads(manifest_path.read_text())
    if app_running():
        raise SystemExit("refusing: invisible_places is running")
    canonical = args.data_dir / "Site1-WATER-5mm.ply"
    preserve = Path(manifest.get(
        "v8_backup", run / "Site1-WATER-5mm.v8-installed.ply"))
    if not preserve.exists():
        raise RuntimeError("v8 backup is absent; nothing safe to restore")
    if canonical.exists() and sha256_path(canonical) != manifest["candidate_sha256"]:
        raise RuntimeError("canonical WATER changed after v9 install; refusing restore")
    if canonical.exists():
        canonical.unlink()
    shutil.move(str(preserve), str(canonical))
    if sha256_path(canonical) != manifest["source_water_sha256"]:
        raise RuntimeError("restored v8 hash mismatch")
    manifest["installed"] = False
    manifest["restored_at"] = dt.datetime.now().isoformat(timespec="seconds")
    manifest_path.write_text(json.dumps(manifest, indent=2))
    log_line("restore", "byte-exact v8 WATER restored")


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("stage", choices=("build", "verify", "install", "restore"))
    parser.add_argument("--data-dir", type=Path, default=DEFAULT_DATA)
    parser.add_argument("--run-dir", type=Path, default=DEFAULT_RUN)
    args = parser.parse_args()
    {"build": build, "verify": verify, "install": install,
     "restore": restore}[args.stage](args)


if __name__ == "__main__":
    main()
