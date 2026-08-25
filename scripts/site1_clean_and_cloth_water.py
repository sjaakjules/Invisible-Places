#!/usr/bin/env python3
"""Site1 reflection-noise cleanup and cloth-guided water fill.

Inputs authored by hand in CloudCompare (2026-08-25):
  Data/Scene1/Site1-ToMesh.ply     manually selected SAND+ROCK subset of the
                                   central flat (verbatim 5 mm records; legs
                                   and equipment already deselected), with
                                   scalar_Surface_density_(r=0.015) and
                                   scalar_C2M_signed_distances
  Data/Scene1/Site1-ClothMesh.ply  CSF cloth ground estimate, 0.1 m grid,
                                   no post processing (open gaps where the
                                   cloud has none)

What this pipeline does:

classify  v4 (2026-08-26): cluster-first over the WHOLE site (v3 only
          examined the ToMesh hull). Candidates are points farther than
          25 mm from the hand-made ToMesh selection (inside the eroded safe
          hull) plus points more than 6 cm above an opened+smoothed ground
          sheet, clustered at 3 cm. Per-cluster verdicts: big attached
          structures keep; sparse clusters (kNN density < 6 k/m^2) lose
          their hovering members (>= 22 cm over the 12.5 cm column minimum,
          vetoed on near-vertical faces); fully detached clusters and
          intensity-extreme clusters with transience evidence (air gap
          under, single-scan capture, self-shadowed ground) are removed as
          objects; bright/dark/manual sub-objects are extracted out of kept
          clusters (2 cm linking, single-scan or hovering, free-standing by
          lateral perimeter touch); removals complete through
          intensity-matched neighbours. The v3 below-cloth mirror rule is
          unchanged. Reasons: 1 below, 2 sparse, 3 floating, 4 object,
          5 seed, 6 sub-object, 7 completion, 8 strict-sparse.

apply     Rewrites the four canonical clouds atomically (temp + rename)
          minus the flagged records, and stores the minimum for a byte-exact
          undo in Data/Scene1/PatchRefinement/<run>/: the removed records as
          a viewable PLY in the original schema, their original indices, the
          original header bytes, sha256 before/after, and the parameters.
          The parent folder carries .invisible_places-ignore so discovery
          never lists rollback data.

restore   Reinserts the removed records at their recorded indices and
          verifies the pre-cleanup sha256, then swaps the file back.

cloth     Harmonic (Laplace) fill of the cloth grid's open gaps across the
          region hull, bilinear upsample to 0.05 m, light smoothing of the
          filled areas only -> Data/Scene1/Site1-ClothMesh-Refined.ply.

water     Replaces Site1-WATER-5mm.ply with a density-continuous fill: a
          jittered 5 mm candidate grid is accepted with probability
          1 - D/Dtarget, where D is the local 2D density of the cleaned
          SAND+ROCK+VEG 5 mm clouds (Gaussian kernel comparable to the
          15 mm CloudCompare neighbourhood) and Dtarget is the 25th
          percentile of well-covered density. Fill height rides the refined
          cloth inside the region (blending to the measured local surface
          where real points exist) and the v1 winsorized-rim harmonic sheet
          outside it (bay inlet etc., z <= 2.6 m there). Attributes: IDW
          colour/intensity/Composite/Shelter/RainExposure/SVF from cleaned
          SAND+ROCK, normals and Slope/Horizontalness from the fill surface
          gradient, curvature/roughness zeroed, ScanID=999.

Review images (seam close-ups, density continuity, removal maps) are
produced by the session QA scripts alongside this tool.

Requires numpy, scipy, open3d (same venv as build_site1_ground_mesh_and_water).
"""

from __future__ import annotations

import argparse
import datetime as _dt
import hashlib
import json
import re
import sys
from pathlib import Path

import numpy as np

# ---- geometry constants -------------------------------------------------
REGION_CELL = 0.1
COLUMN_CELL = 0.025
SURFACE_BAND = 0.03
BELOW_DZ = -0.03
ABOVE_DZ = 0.05
COVER_RADIUS_CELLS = 3          # 3 * 2.5 cm = 7.5 cm XY
SUPPORT_GAP = 0.06
MANUAL_DISTANCE = 0.025
HULL_CLOSE_CELLS = 4            # 0.4 m closing for the region hull
SAFE_ERODE_CELLS = 5            # 0.5 m erosion for the manual test
DENSITY_CELL = 0.01
DENSITY_SIGMA_M = 0.006         # ~15 mm neighbourhood equivalent
WELL_COVERED_MIN = 10_000.0
TARGET_PERCENTILE = 25.0
FILL_TRIGGER = 0.85             # fill only where density < 85% of target
KEEP_CLEAR = 0.004
OUTSIDE_Z_CAP = 2.6
WATER_SCAN_ID = 999.0
RUN_NAME = "20260826-noise-cleanup-v4"

# ---- v4 classifier constants (calibrated on named exemplars: user-marked
# ---- legs/bags/poles, the two trail areas, and every dense patch v3 lost;
# ---- the dry-run harness lives in the session notes) ---------------------
ENV_CELL = 0.10                 # ground-sheet cell
OPEN_RADIUS_CELLS = 2           # 0.2 m grey opening: sheet hugs the low ground
SHEET_SIGMA = 1.2
ISLAND_MAX_CELLS = 60           # small sparse data-islands cannot support
ISLAND_SPARSE = 6_000.0         # ...the sheet beneath themselves
H_CAND = 0.06                   # candidate height over the sheet
CLUSTER_VOX = 0.03
SPARSE_DENSITY_MAX = 6_000.0    # cluster-median kNN density below = sparse
PP_SPARSE_DENSITY = 4_300.0     # strict per-point sparse inside kept clusters
PP_HOVER3 = 0.25
HOVER3_REMOVE = 0.22            # hover over the 12.5 cm column minimum
HOVER3_COMPLETE = 0.15
SPARSE_SHEET_H = 0.35
VOID_SHEET_H = 0.25             # lower bar on env-voided island cells
WALL_RELIEF_MAX = 0.5           # hover rules stand down on near-vertical faces
BIG_KEEP_N = 20_000
BIG_KEEP_DENSITY = 8_000.0
FLOAT_CONTACT_R = 0.045
FLOAT_MIN_N = 60
OBJ_I_MED_HI = 590_000.0        # intensity gates for transient objects
OBJ_I_P90_HI = 950_000.0
OBJ_I_MED_LO = 150_000.0
OBJ_MANUAL_FRAC = 0.70          # the user's hand is evidence on the flat
FLAT_MAJOR = 0.5
EV_GAP_FRAC = 0.35              # air-gap under, single-scan, self-shadow
EV_GAP = 0.08
EV_SINGLE_SCAN = 0.95
EV_SELF_SHADOW = 0.45
EV_SHADOW_H = 0.12
MIN_OBJ_N = 60
SUB_I_HI = 520_000.0            # sub-object extraction out of kept clusters
SUB_VOX = 0.02
SUB_MAX_N = 30_000
SUB_MAX_PARENT_FRAC = 0.5
SUB_HOVER3 = 0.30
SUB_LOCAL_SCANS = 1.3           # transient volumes are one scan's private space
SUB_FREE_TOUCH = 0.05
SUB_FREE_H3 = 0.5
SUB_PTOUCH_BRIGHT = 0.35
SUB_PTOUCH_MANUAL = 0.25
COMPLETE_R = 0.04
COMPLETE_BRIGHT = 450_000.0
COMPLETE_DARK = 220_000.0

# Conservative recovery for gaps punched into the water fill by the coarse
# steep-cloth veto.  Recovery is inferred from the SAND layer in XY, but its
# height must also agree in 3D and a high ROCK neighbourhood vetoes it.
WATER_SUPPORT_CELL = 0.025
WATER_SUPPORT_Z_CAP = 2.65
WATER_XY_SEED_CLOSE_M = 0.05
WATER_XY_CLOSE_M = 0.40
WATER_Z_COHERENCE_SIGMA_M = 0.20
WATER_Z_STD_MAX = 0.10
WATER_ROCK_CLEARANCE = 0.12
WATER_ROCK_LOCAL_MIN = 0.15
WATER_ROCK_DOMINANCE = 1.00
WATER_RECOVERY_EDGE_CLEAR_M = 0.075
WATER_RECOVERY_MAX_COMPONENT_M2 = 5.0
WATER_RECOVERY_MIN_CONFIDENCE = 0.10

# v2 classifier guards (after the v1 review): ghost tests only where the
# cloth is near-flat (rock intersections broke the band logic and lost real
# squares), the below-cover must be a near-horizontal surface, and a
# candidate must be locally sparse (ripple-trough floors are dense ~28k/m^2,
# reflection ghosts ~6k). 1 mm ghosts inherit from removed 5 mm ghosts so
# both densities always agree.
SLOPE_MAX_DEG = 15.0
HORIZONTAL_NZ = 0.6
BELOW_DENSITY_MAX = 12_000.0
ABOVE_DENSITY_MAX = 15_000.0
DENSITY_NEIGHBOURS = 13
INHERIT_RADIUS = 0.012
SEED_CLUSTERS = [               # user-marked limbs/equipment on the ledge
    (762.3, 820.6, 2.9),
    (762.4, 829.1, 2.8),
]
SEED_BOX = 1.25                 # half-width of the seed neighbourhood
SEED_MIN_COMPONENT = 300        # 5 mm points; smaller far-specks are left
SEED_VOXEL = 0.03               # connectivity voxel for far-component labelling
SEED_RADIUS = 0.9               # a cluster must come within this XY radius of the seed
SEED_Z_BELOW = 0.4              # accepted cluster z window around the marked point
SEED_Z_ABOVE = 0.9
STEEP_DILATE_CELLS = 4          # ghost tests keep 0.4 m clear of steep cloth: a shelf
                                # crest is locally flat while its face is steep, and v2
                                # removed column-quantised squares exactly there

CLEAN_TARGETS = [
    ("SAND", "5mm"), ("ROCK", "5mm"), ("SAND", "1mm"), ("ROCK", "1mm"),
]
REASON_NAMES = {1: "below-ghost", 2: "sparse-noise", 3: "floating", 4: "object",
                5: "seeded-cluster", 6: "sub-object", 7: "completion",
                8: "strict-sparse"}


def read_header(path):
    fields = []
    with open(path, "rb") as handle:
        header = b""
        while not header.endswith(b"end_header\n"):
            line = handle.readline()
            if not line:
                raise RuntimeError(f"no end_header in {path}")
            header += line
        offset = handle.tell()
    count = None
    typemap = {"float": "<f4", "double": "<f8", "uchar": "u1", "int": "<i4",
               "uint": "<u4", "short": "<i2", "ushort": "<u2", "char": "i1"}
    for line in header.decode("ascii", "replace").splitlines():
        parts = line.split()
        if not parts:
            continue
        if parts[0] == "element" and parts[1] == "vertex":
            count = int(parts[2])
        elif parts[0] == "property" and parts[1] != "list":
            fields.append((parts[2], typemap[parts[1]]))
    return np.dtype(fields), count, offset, header


def memmap_cloud(path):
    dtype, count, offset, header = read_header(path)
    return np.memmap(path, dtype=dtype, mode="r", offset=offset, shape=(count,)), dtype, header


def sha256_path(path: Path, chunk: int = 32 * 1024 * 1024) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        while True:
            block = handle.read(chunk)
            if not block:
                break
            digest.update(block)
    return digest.hexdigest()


def cloud_path(data_dir: Path, role: str, spacing: str) -> Path:
    return data_dir / f"Site1-{role}-{spacing}.ply"


# ---- shared spatial context --------------------------------------------

class Context:
    """Region masks, filled cloth grid, and the 5 mm surface-band columns."""

    def __init__(self, data_dir: Path, work: Path):
        from scipy import ndimage
        self.work = work
        tomesh, _, _ = memmap_cloud(data_dir / "Site1-ToMesh.ply")
        self.tomesh_xyz = np.stack(
            [tomesh["x"], tomesh["y"], tomesh["z"]], 1).astype(np.float32)
        margin = 1.0
        self.x0 = float(self.tomesh_xyz[:, 0].min() - margin)
        self.y0 = float(self.tomesh_xyz[:, 1].min() - margin)
        self.nx = int((self.tomesh_xyz[:, 0].max() + margin - self.x0) / REGION_CELL) + 1
        self.ny = int((self.tomesh_xyz[:, 1].max() + margin - self.y0) / REGION_CELL) + 1

        gx = ((self.tomesh_xyz[:, 0] - self.x0) / REGION_CELL).astype(np.int32)
        gy = ((self.tomesh_xyz[:, 1] - self.y0) / REGION_CELL).astype(np.int32)
        occupancy = np.zeros((self.ny, self.nx), bool)
        occupancy[gy, gx] = True

        def disk(radius):
            span = np.mgrid[-radius:radius + 1, -radius:radius + 1]
            return np.hypot(span[0], span[1]) <= radius

        self.hull = ndimage.binary_fill_holes(
            ndimage.binary_closing(occupancy, structure=disk(HULL_CLOSE_CELLS)))
        self.safe = ndimage.binary_erosion(self.hull, structure=disk(SAFE_ERODE_CELLS))

        cloth, _, _ = memmap_cloud(data_dir / "Site1-ClothMesh.ply")
        cx = ((cloth["x"] - self.x0) / REGION_CELL + 0.5).astype(np.int32)
        cy = ((cloth["y"] - self.y0) / REGION_CELL + 0.5).astype(np.int32)
        inside = (cx >= 0) & (cx < self.nx) & (cy >= 0) & (cy < self.ny)
        self.cloth_raw = np.full((self.ny, self.nx), np.nan, np.float32)
        self.cloth_raw[cy[inside], cx[inside]] = cloth["z"][inside]
        self.cloth_defined = np.isfinite(self.cloth_raw)
        self.cloth_near = ndimage.binary_dilation(self.cloth_defined, iterations=2)
        self.cloth_filled = self._harmonic_fill(self.cloth_raw, self.hull)
        gradient_y, gradient_x = np.gradient(self.cloth_filled, REGION_CELL)
        self.cloth_slope_deg = np.degrees(
            np.arctan(np.hypot(gradient_x, gradient_y))).astype(np.float32)
        steep = self.cloth_slope_deg >= SLOPE_MAX_DEG
        self.steep_near = ndimage.binary_dilation(
            steep, structure=disk(STEEP_DILATE_CELLS))
        np.savez_compressed(
            work / "context.npz",
            hull=self.hull, safe=self.safe, cloth_filled=self.cloth_filled,
            cloth_defined=self.cloth_defined, cloth_slope=self.cloth_slope_deg,
            meta=np.array([self.x0, self.y0, REGION_CELL, self.nx, self.ny]))

        self._tree = None
        self._columns = None

    def _harmonic_fill(self, grid, domain):
        from scipy import ndimage
        from scipy.sparse import coo_matrix, identity
        from scipy.sparse.linalg import spsolve
        known = np.isfinite(grid)
        unknown = domain & ~known
        index = np.full(grid.shape, -1, np.int64)
        index[unknown] = np.arange(unknown.sum())
        uy, ux = np.nonzero(unknown)
        rows, cols, vals = [], [], []
        rhs = np.zeros(unknown.sum())
        for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            ny_, nx_ = uy + dy, ux + dx
            inside = (ny_ >= 0) & (ny_ < grid.shape[0]) & (nx_ >= 0) & (nx_ < grid.shape[1])
            this = index[uy[inside], ux[inside]]
            neighbour = index[ny_[inside], nx_[inside]]
            fixed = known[ny_[inside], nx_[inside]]
            connected = (neighbour >= 0) | fixed
            rows.append(this[connected]); cols.append(this[connected])
            vals.append(np.ones(connected.sum()))
            link = neighbour >= 0
            rows.append(this[link]); cols.append(neighbour[link])
            vals.append(-np.ones(link.sum()))
            np.add.at(rhs, this[fixed], np.nan_to_num(grid[ny_[inside][fixed], nx_[inside][fixed]]))
        matrix = coo_matrix(
            (np.concatenate(vals), (np.concatenate(rows), np.concatenate(cols))),
            shape=(len(rhs), len(rhs))).tocsr()
        matrix = matrix + 1e-6 * identity(len(rhs), format="csr")
        rhs = rhs + 1e-6 * np.nanmedian(grid)
        solved = spsolve(matrix, rhs)
        filled = grid.copy()
        filled[uy, ux] = solved.astype(np.float32)
        # gentle smoothing of the filled cells only
        blurred = ndimage.gaussian_filter(np.nan_to_num(filled, nan=np.nanmedian(grid)), 1.0)
        filled[unknown] = blurred[unknown]
        return filled

    def tomesh_tree(self):
        if self._tree is None:
            from scipy.spatial import cKDTree
            self._tree = cKDTree(self.tomesh_xyz)
        return self._tree

    # -- column grids (2.5 cm) built from the 5 mm SAND+ROCK clouds -------
    #
    # band       a surface-band point (|z - cloth| <= 3 cm) exists within
    #            7.5 cm XY of the column
    # band_top   max z of those surface-band points (same neighbourhood)
    # conn_top   the highest z reachable from the surface band by climbing
    #            the column's occupied 3 cm z-bins without ever crossing an
    #            empty run of two or more bins (>= 6 cm of clear air).
    #            A grounded mound is fully connected (conn_top = its crest);
    #            a hovering reflection slab is not (conn_top stays at the
    #            surface), which is the floater discriminator.
    Z_BIN = 0.03
    Z_MIN = 1.4
    Z_BINS = 80  # covers 1.4 .. 3.8 m

    def columns(self, data_dir: Path):
        if self._columns is not None:
            return self._columns
        cache = self.work / "columns.npz"
        if cache.exists():
            data = np.load(cache)
            self._columns = (data["band"], data["band_top"], data["conn_top"])
            return self._columns
        from scipy import ndimage
        cnx = int(self.nx * REGION_CELL / COLUMN_CELL) + 1
        cny = int(self.ny * REGION_CELL / COLUMN_CELL) + 1
        band = np.zeros((cny, cnx), bool)
        band_top = np.full((cny, cnx), -np.inf, np.float32)
        occupancy = np.zeros((cny * cnx, self.Z_BINS), bool)
        for role in ("SAND", "ROCK"):
            cloud, _, _ = memmap_cloud(cloud_path(data_dir, role, "5mm"))
            for start in range(0, len(cloud), 8_000_000):
                chunk = cloud[start:start + 8_000_000]
                cx = ((chunk["x"] - self.x0) / COLUMN_CELL).astype(np.int64)
                cy = ((chunk["y"] - self.y0) / COLUMN_CELL).astype(np.int64)
                inside = (cx >= 0) & (cx < cnx) & (cy >= 0) & (cy < cny)
                if not inside.any():
                    continue
                z = chunk["z"][inside].astype(np.float32)
                flat = cy[inside] * cnx + cx[inside]
                zbin = np.clip(((z - self.Z_MIN) / self.Z_BIN).astype(np.int64),
                               0, self.Z_BINS - 1)
                occupancy[flat, zbin] = True
                gx = np.clip((chunk["x"][inside] - self.x0) / REGION_CELL,
                             0, self.nx - 1).astype(np.int64)
                gy = np.clip((chunk["y"][inside] - self.y0) / REGION_CELL,
                             0, self.ny - 1).astype(np.int64)
                dz = z - self.cloth_filled[gy, gx]
                normals = np.stack([chunk["nx"][inside], chunk["ny"][inside],
                                    chunk["nz"][inside]], 1).astype(np.float32)
                length = np.linalg.norm(normals, axis=1)
                horizontal = (length > 0.5) & (
                    np.abs(normals[:, 2]) / np.maximum(length, 1e-6) >= HORIZONTAL_NZ)
                in_band = (np.abs(dz) <= SURFACE_BAND) & horizontal
                np.maximum.at(band_top.ravel(), flat[in_band], z[in_band])
                band.ravel()[flat[in_band]] = True
        size = 2 * COVER_RADIUS_CELLS + 1
        band = ndimage.maximum_filter(band.astype(np.uint8), size=size).astype(bool)
        band_top = ndimage.maximum_filter(band_top, size=size)

        # climb the columns: start at the surface-band bin, tolerate a single
        # empty bin, stop at the first >= 2-bin air gap
        start_bin = np.clip(((band_top.ravel() - self.Z_MIN) / self.Z_BIN)
                            .astype(np.int64), 0, self.Z_BINS - 1)
        start_bin[~band.ravel()] = -1
        conn_bin = start_bin.copy()
        empty_run = np.zeros(cny * cnx, np.int8)
        active = start_bin >= 0
        for b in range(self.Z_BINS):
            above_start = active & (b > start_bin)
            occ = occupancy[:, b]
            empty_run[above_start & occ] = 0
            conn_bin[above_start & occ] = b
            empty_run[above_start & ~occ] += 1
            active &= ~(above_start & ~occ & (empty_run >= 2))
        conn_top = np.where(
            start_bin >= 0,
            self.Z_MIN + (conn_bin + 1) * self.Z_BIN,
            -np.inf).astype(np.float32).reshape(cny, cnx)
        conn_top = ndimage.maximum_filter(conn_top, size=size)
        np.savez_compressed(cache, band=band, band_top=band_top, conn_top=conn_top)
        self._columns = (band, band_top, conn_top)
        return self._columns


# ---- classify (v4) -------------------------------------------------------

def _cluster_labels(xyz, vox):
    """Sparse 26-connectivity voxel clustering; per-point labels."""
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import connected_components
    v = np.floor(xyz / vox).astype(np.int64)
    v -= v.min(axis=0)
    span = v.max(axis=0) + 2
    pack = (v[:, 0] * span[1] + v[:, 1]) * span[2] + v[:, 2]
    keys, inverse = np.unique(pack, return_inverse=True)
    rows, cols = [], []
    for dx in (0, 1):
        for dy in (-1, 0, 1):
            for dz in (-1, 0, 1):
                if (dx, dy, dz) <= (0, 0, 0):
                    continue
                off = dx * span[1] * span[2] + dy * span[2] + dz
                pos = np.clip(np.searchsorted(keys, keys + off), 0, len(keys) - 1)
                hit = keys[pos] == keys + off
                rows.append(np.nonzero(hit)[0])
                cols.append(pos[hit])
    graph = coo_matrix(
        (np.ones(sum(len(r) for r in rows), np.uint8),
         (np.concatenate(rows), np.concatenate(cols))),
        shape=(len(keys), len(keys)))
    _, voxel_label = connected_components(graph, directed=False)
    return voxel_label[inverse]


def classify_verdicts(metrics):
    """Pure cluster-verdict table over per-cluster metric arrays.

    metrics: dict with n, dens_med, i_med, i_p90, share, f_under, f_gap8,
    man_f, flat_f, h_med, contact, seedhit (equal-length numpy arrays).
    Returns uint8 verdicts: 0 keep, 2 sparse, 3 floating, 4 object, 5 seed.
    """
    n = metrics["n"]
    big = (n >= BIG_KEEP_N) & (metrics["dens_med"] >= BIG_KEEP_DENSITY)
    sparse = ~big & (metrics["dens_med"] < SPARSE_DENSITY_MAX)
    floating = (~big & ~sparse & (n >= FLOAT_MIN_N) &
                (metrics["contact"] < np.minimum(25, np.maximum(1, n // 100))))
    flat_major = metrics["flat_f"] >= FLAT_MAJOR
    gate = ((metrics["i_med"] >= OBJ_I_MED_HI) |
            (metrics["i_p90"] >= OBJ_I_P90_HI) |
            (metrics["i_med"] <= OBJ_I_MED_LO))
    evidence = ((metrics["f_gap8"] >= EV_GAP_FRAC) |
                (metrics["share"] >= EV_SINGLE_SCAN) |
                ((metrics["f_under"] <= EV_SELF_SHADOW) &
                 (metrics["h_med"] >= EV_SHADOW_H) & flat_major))
    obj = (~big & ~sparse & ~floating & (n >= MIN_OBJ_N) &
           ((gate & evidence) |
            ((metrics["man_f"] >= OBJ_MANUAL_FRAC) & flat_major)))
    verdict = np.zeros(len(n), np.uint8)
    verdict[floating] = 3
    verdict[obj] = 4
    verdict[(metrics["seedhit"] == 1) & ~big & (verdict == 0)] = 5
    verdict[sparse] = 2
    return verdict


def stage_classify(context: Context, data_dir: Path, work: Path):
    from scipy import ndimage
    from scipy.spatial import cKDTree

    def log(message):
        print(f"[classify] {message}", flush=True)

    # ---- combined 5 mm SAND+ROCK arrays (original clouds on disk) --------
    role_span = {}
    xs, ys, zs, is_, ss = [], [], [], [], []
    cursor = 0
    for role in ("SAND", "ROCK"):
        cloud, _, _ = memmap_cloud(cloud_path(data_dir, role, "5mm"))
        for a in range(0, len(cloud), 8_000_000):
            k = cloud[a:a + 8_000_000]
            xs.append(k["x"].astype(np.float32))
            ys.append(k["y"].astype(np.float32))
            zs.append(k["z"].astype(np.float32))
            is_.append(k["scalar_Intensity"].astype(np.float32))
            ss.append(k["scalar_ScanID"].astype(np.float32))
        role_span[role] = (cursor, cursor + len(cloud))
        cursor += len(cloud)
    X = np.concatenate(xs); Y = np.concatenate(ys); Z = np.concatenate(zs)
    I = np.concatenate(is_); S = np.concatenate(ss)
    del xs, ys, zs, is_, ss
    log(f"combined 5 mm cloud: {len(X):,} points")

    # ---- global kNN surface density (13 neighbours, disk-equivalent) -----
    tree = cKDTree(np.stack([X, Y, Z], 1))
    dens = np.empty(len(X), np.float32)
    for a in range(0, len(X), 4_000_000):
        d, _ = tree.query(
            np.stack([X[a:a + 4_000_000], Y[a:a + 4_000_000],
                      Z[a:a + 4_000_000]], 1), k=14, workers=-1)
        dens[a:a + 4_000_000] = 13.0 / (np.pi * np.maximum(d[:, 13], 1e-4) ** 2)
    log("global density done")

    # ---- v3 below-cloth mirror rule (unchanged; combined-tree density) ---
    band, band_top, _ = context.columns(data_dir)
    cny, cnx = band.shape
    gx = np.clip(((X - context.x0) / REGION_CELL).astype(np.int64), 0, context.nx - 1)
    gy = np.clip(((Y - context.y0) / REGION_CELL).astype(np.int64), 0, context.ny - 1)
    inside = ((X >= context.x0) & (X < context.x0 + context.nx * REGION_CELL) &
              (Y >= context.y0) & (Y < context.y0 + context.ny * REGION_CELL))
    in_safe = inside & context.safe[gy, gx]
    near_cloth = inside & context.cloth_near[gy, gx]
    point_flat = inside & ~context.steep_near[gy, gx]
    dz = Z - context.cloth_filled[gy, gx]
    ccx = np.clip(((X - context.x0) / COLUMN_CELL).astype(np.int64), 0, cnx - 1)
    ccy = np.clip(((Y - context.y0) / COLUMN_CELL).astype(np.int64), 0, cny - 1)
    below = (near_cloth & ~context.steep_near[gy, gx] & (dz < BELOW_DZ) &
             band[ccy, ccx] & (band_top[ccy, ccx] > Z + 0.04) &
             (dens < BELOW_DENSITY_MAX))
    log(f"below-cloth mirrors: {below.sum():,}")

    # ---- ground sheet: cell minima, island voiding, opening, smoothing ---
    ex0, ey0 = X.min() - 0.5, Y.min() - 0.5
    enx = int((X.max() + 0.5 - ex0) / ENV_CELL) + 1
    eny = int((Y.max() + 0.5 - ey0) / ENV_CELL) + 1
    egx = ((X - ex0) / ENV_CELL).astype(np.int64)
    egy = ((Y - ey0) / ENV_CELL).astype(np.int64)
    eflat = egy * enx + egx
    env = np.full(eny * enx, np.inf, np.float32)
    np.minimum.at(env, eflat[~below], Z[~below])
    dens_sum = np.zeros(eny * enx, np.float64)
    dens_cnt = np.zeros(eny * enx, np.int64)
    np.add.at(dens_sum, eflat[~below], dens[~below])
    np.add.at(dens_cnt, eflat[~below], 1)
    cell_dens = np.divide(dens_sum, dens_cnt, out=np.zeros(eny * enx),
                          where=dens_cnt > 0).reshape(eny, enx)
    have = np.isfinite(env).reshape(eny, enx)
    env = env.reshape(eny, enx)
    env[~have] = np.nan
    point_voided = np.zeros(len(X), bool)
    labels, ncomp = ndimage.label(have, structure=np.ones((3, 3), bool))
    if ncomp:
        sizes = np.bincount(labels.ravel())
        comp_dens = np.bincount(labels.ravel(),
                                weights=np.nan_to_num(cell_dens).ravel())
        comp_mean = np.divide(comp_dens, np.maximum(sizes, 1))
        void = (sizes <= ISLAND_MAX_CELLS) & (comp_mean < ISLAND_SPARSE)
        void[0] = False
        voided = void[labels]
        env[voided] = np.nan
        have &= ~voided
        point_voided = voided[egy, egx]
        log(f"env islands voided: {int(void.sum()):,} components")
    disk_span = np.mgrid[-OPEN_RADIUS_CELLS:OPEN_RADIUS_CELLS + 1,
                         -OPEN_RADIUS_CELLS:OPEN_RADIUS_CELLS + 1]
    disk = np.hypot(disk_span[0], disk_span[1]) <= OPEN_RADIUS_CELLS
    sheet = ndimage.grey_dilation(
        ndimage.grey_erosion(fill_nan_nearest(env.copy()), footprint=disk),
        footprint=disk)
    sheet = ndimage.gaussian_filter(sheet, SHEET_SIGMA)
    H = Z - sheet[egy, egx]

    # ---- 2.5 cm column minima, hover, and wall relief --------------------
    ncx2 = int((X.max() + 0.5 - ex0) / COLUMN_CELL) + 1
    ncy2 = int((Y.max() + 0.5 - ey0) / COLUMN_CELL) + 1
    c2x = ((X - ex0) / COLUMN_CELL).astype(np.int64)
    c2y = ((Y - ey0) / COLUMN_CELL).astype(np.int64)
    c2flat = c2y * ncx2 + c2x
    colbase = np.full(ncy2 * ncx2, np.inf, np.float32)
    np.minimum.at(colbase, c2flat[~below], Z[~below])
    colbase = colbase.reshape(ncy2, ncx2)
    lo5 = ndimage.minimum_filter(
        np.where(np.isfinite(colbase), colbase, np.inf), size=5)
    hi5 = ndimage.maximum_filter(
        np.where(np.isfinite(colbase), colbase, -np.inf), size=5)
    RELIEF = (hi5 - lo5)[c2y, c2x]
    colmin = ndimage.minimum_filter(colbase, size=5)
    HOVER3 = Z - colmin[c2y, c2x]
    log("sheet + column minima done")

    # ---- candidates: far from the hand-made mesh, or above the sheet -----
    tomesh = context.tomesh_tree()
    manual = np.zeros(len(X), bool)
    sidx = np.nonzero(in_safe & ~below)[0]
    for a in range(0, len(sidx), 6_000_000):
        q = sidx[a:a + 6_000_000]
        d, _ = tomesh.query(np.stack([X[q], Y[q], Z[q]], 1), k=1, workers=-1,
                            distance_upper_bound=MANUAL_DISTANCE)
        manual[q[~np.isfinite(d)]] = True
    cand = ~below & (manual | (np.isfinite(H) & (H > H_CAND)))
    cidx = np.nonzero(cand)[0]
    label = _cluster_labels(np.stack([X[cidx], Y[cidx], Z[cidx]], 1), CLUSTER_VOX)
    n_clusters = int(label.max()) + 1 if len(cidx) else 0
    log(f"candidates {len(cidx):,} in {n_clusters:,} clusters")

    # ---- under metrics against non-candidates (site-wide columns) --------
    colkey = c2x * np.int64(200_003) + c2y
    nc_idx = np.nonzero(~cand & ~below)[0]
    order = np.lexsort((Z[nc_idx], colkey[nc_idx]))
    nc_key = colkey[nc_idx][order]
    nc_z = Z[nc_idx][order]

    def under_of(points_idx):
        pk = colkey[points_idx]
        pz = Z[points_idx] - 0.01
        a = np.searchsorted(nc_key, pk, "left")
        b = np.searchsorted(nc_key, pk, "right")
        zi = np.fromiter(
            (np.searchsorted(nc_z[lo:hi], q) for lo, hi, q in zip(a, b, pz)),
            np.int64, len(pk)) - 1
        out = np.full(len(pk), -np.inf, np.float32)
        ok = (b > a) & (zi >= 0)
        out[ok] = nc_z[(a + np.maximum(zi, 0))[ok]]
        return out

    ck = colkey[cidx]
    pair = ck * np.int64(1 << 21) + label
    o2 = np.lexsort((Z[cidx], pair))
    _, ufirst = np.unique(pair[o2], return_index=True)
    rep = o2[ufirst]
    rep_lab = label[rep]
    rep_under = under_of(cidx[rep])
    rep_gap = Z[cidx[rep]] - rep_under
    cnt_cols = np.bincount(rep_lab, minlength=n_clusters)
    f_under = np.bincount(rep_lab[rep_under > -np.inf],
                          minlength=n_clusters) / np.maximum(cnt_cols, 1)
    f_gap8 = np.bincount(rep_lab[(rep_under > -np.inf) & (rep_gap >= EV_GAP)],
                         minlength=n_clusters) / np.maximum(cnt_cols, 1)
    nc_tree = cKDTree(np.stack([X[nc_idx], Y[nc_idx], Z[nc_idx]], 1))
    contact = np.zeros(len(cidx), bool)
    for a in range(0, len(cidx), 4_000_000):
        q = cidx[a:a + 4_000_000]
        d, _ = nc_tree.query(np.stack([X[q], Y[q], Z[q]], 1), k=1, workers=-1,
                             distance_upper_bound=FLOAT_CONTACT_R)
        contact[a:a + len(q)] = np.isfinite(d)
    cnt_contact = np.bincount(label[contact], minlength=n_clusters)
    log("under metrics + contact done")

    # ---- per-cluster aggregates -----------------------------------------
    CI = I[cidx]; CD = dens[cidx]; CS = S[cidx]; CM = manual[cidx]
    CX = X[cidx]; CY = Y[cidx]; CZ = Z[cidx]
    CH = H[cidx]; CH3 = HOVER3[cidx]; CF = point_flat[cidx]
    sort_order = np.argsort(label, kind="stable")
    lab_s = label[sort_order]
    starts = np.concatenate([[0], np.nonzero(np.diff(lab_s))[0] + 1])
    ends = np.concatenate([starts[1:], [len(lab_s)]])
    m = {name: np.zeros(n_clusters, np.float32)
         for name in ("i_med", "i_p90", "dens_med", "share", "man_f",
                      "flat_f", "h_med")}
    seedhit = np.zeros(n_clusters, np.int8)
    for st, en in zip(starts, ends):
        li = lab_s[st]
        idx = sort_order[st:en]
        m["i_med"][li] = np.median(CI[idx])
        m["i_p90"][li] = np.percentile(CI[idx], 90)
        m["dens_med"][li] = np.median(CD[idx])
        m["h_med"][li] = np.nanmedian(CH[idx])
        _, cc = np.unique(CS[idx], return_counts=True)
        m["share"][li] = cc.max() / (en - st)
        m["man_f"][li] = CM[idx].mean()
        m["flat_f"][li] = CF[idx].mean()
        for sx, sy, sz in SEED_CLUSTERS:
            if (np.hypot(CX[idx] - sx, CY[idx] - sy).min() <= SEED_RADIUS and
                    CZ[idx].min() >= sz - SEED_Z_BELOW and
                    CZ[idx].max() <= sz + SEED_Z_ABOVE and
                    (en - st) >= SEED_MIN_COMPONENT):
                seedhit[li] = 1
    n_of = np.bincount(label, minlength=n_clusters)
    verdict = classify_verdicts({
        "n": n_of, "dens_med": m["dens_med"], "i_med": m["i_med"],
        "i_p90": m["i_p90"], "share": m["share"], "f_under": f_under,
        "f_gap8": f_gap8, "man_f": m["man_f"], "flat_f": m["flat_f"],
        "h_med": m["h_med"], "contact": cnt_contact, "seedhit": seedhit})
    sparse_isolated = ((verdict == 2) &
                       (cnt_contact < np.maximum(2, n_of // 200)))
    vp = verdict[label]

    # ---- per-point removals with reasons ---------------------------------
    reasons = np.zeros(len(X), np.uint8)
    reasons[below] = 1
    remove_pt = np.isin(vp, (3, 4, 5))
    point_reason = np.where(remove_pt, vp, 0).astype(np.uint8)
    iso = sparse_isolated[label] & ~remove_pt
    remove_pt |= iso
    point_reason[iso] = 2
    sp_pts = np.nonzero((vp == 2) & ~remove_pt)[0]
    if len(sp_pts):
        gentle = RELIEF[cidx[sp_pts]] <= WALL_RELIEF_MAX
        airborne = gentle & (
            (CH3[sp_pts] >= HOVER3_REMOVE) |
            (np.nan_to_num(CH[sp_pts], nan=0.0) >= SPARSE_SHEET_H) |
            (point_voided[cidx[sp_pts]] &
             (np.nan_to_num(CH[sp_pts], nan=1.0) >= VOID_SHEET_H)))
        remove_pt[sp_pts[airborne]] = True
        point_reason[sp_pts[airborne]] = 2
    keep_pts = np.nonzero((vp == 0) & (CD < PP_SPARSE_DENSITY) & ~remove_pt)[0]
    if len(keep_pts):
        airborne = ((CH3[keep_pts] >= PP_HOVER3) &
                    (RELIEF[cidx[keep_pts]] <= WALL_RELIEF_MAX))
        remove_pt[keep_pts[airborne]] = True
        point_reason[keep_pts[airborne]] = 8
    log(f"cluster + sparse removals: {int(remove_pt.sum()):,}")

    # ---- sub-object extraction out of kept clusters ----------------------
    kept_candidate = np.nonzero((vp == 0) & ~remove_pt)[0]

    def run_sub_extraction(sel_idx, need_man):
        removed = np.zeros(len(cidx), bool)
        if not len(sel_idx):
            return removed
        slab = _cluster_labels(
            np.stack([CX[sel_idx], CY[sel_idx], CZ[sel_idx]], 1), SUB_VOX)
        n_sub = int(slab.max()) + 1
        sn = np.bincount(slab, minlength=n_sub)
        in_sel = np.zeros(len(cidx), bool)
        in_sel[sel_idx] = True
        outside = kept_candidate[~in_sel[kept_candidate]]
        touch = np.zeros(len(sel_idx), bool)
        if len(outside):
            ot = cKDTree(np.stack([CX[outside], CY[outside], CZ[outside]], 1))
            d, j = ot.query(
                np.stack([CX[sel_idx], CY[sel_idx], CZ[sel_idx]], 1),
                k=4, workers=-1, distance_upper_bound=0.035)
            oz = CZ[outside]
            for col in range(d.shape[1]):
                ok = np.isfinite(d[:, col])
                jj = np.clip(j[ok, col], 0, len(oz) - 1)
                lateral = oz[jj] >= CZ[sel_idx[ok]] - 0.03
                touch[np.nonzero(ok)[0][lateral]] = True
        s_touch = np.bincount(slab[touch], minlength=n_sub) / np.maximum(sn, 1)
        dloc, jloc = tree.query(
            np.stack([X[cidx[sel_idx]], Y[cidx[sel_idx]], Z[cidx[sel_idx]]], 1),
            k=8, workers=-1)
        neigh = S[jloc]
        local_scans = np.ones(len(sel_idx), np.float32)
        for col in range(1, neigh.shape[1]):
            near = dloc[:, col] <= 0.025
            fresh = near & (np.abs(neigh[:, col] - neigh[:, 0]) > 0.5)
            for prev in range(1, col):
                fresh &= ~(near & (np.abs(neigh[:, col] - neigh[:, prev]) < 0.5))
            local_scans += fresh.astype(np.float32)
        s_share = np.zeros(n_sub, np.float32)
        s_h3 = np.zeros(n_sub, np.float32)
        s_mix = np.zeros(n_sub, np.float32)
        s_man = np.zeros(n_sub, np.float32)
        s_pf = np.zeros(n_sub, np.float32)
        parent_n = n_of[label]
        so2 = np.argsort(slab, kind="stable")
        sl_s = slab[so2]
        st2 = np.concatenate([[0], np.nonzero(np.diff(sl_s))[0] + 1])
        en2 = np.concatenate([st2[1:], [len(sl_s)]])
        for st, en in zip(st2, en2):
            li = sl_s[st]
            idx = sel_idx[so2[st:en]]
            _, cc = np.unique(CS[idx], return_counts=True)
            s_share[li] = cc.max() / (en - st)
            s_h3[li] = np.median(CH3[idx])
            s_mix[li] = np.median(local_scans[so2[st:en]])
            s_man[li] = CM[idx].mean()
            s_pf[li] = (en - st) / parent_n[idx[0]]
        base = (sn >= MIN_OBJ_N) & (sn <= SUB_MAX_N) & (s_pf <= SUB_MAX_PARENT_FRAC)
        if need_man:
            s_obj = base & (s_man >= OBJ_MANUAL_FRAC) & (s_touch <= SUB_PTOUCH_MANUAL)
        else:
            s_obj = base & (
                ((s_touch <= SUB_PTOUCH_BRIGHT) &
                 ((s_share >= EV_SINGLE_SCAN) |
                  ((s_h3 >= SUB_HOVER3) & (s_mix <= SUB_LOCAL_SCANS)))) |
                ((s_touch <= SUB_FREE_TOUCH) & (s_h3 >= SUB_FREE_H3)))
        removed[sel_idx[s_obj[slab]]] = True
        return removed

    kept_mask = (vp == 0) & ~remove_pt
    sub_rm = (run_sub_extraction(
                  np.nonzero(kept_mask & ((CI >= SUB_I_HI) |
                                          (CI <= OBJ_I_MED_LO)))[0], False) |
              run_sub_extraction(np.nonzero(kept_mask & CM)[0], True))
    sub_rm &= ~remove_pt
    remove_pt |= sub_rm
    point_reason[sub_rm] = 6
    log(f"sub-objects: {int(sub_rm.sum()):,}")

    # ---- completions ------------------------------------------------------
    rm_all = below.copy()
    reasons[cidx[remove_pt]] = point_reason[remove_pt]
    rm_all[cidx[remove_pt]] = True
    obj_global = cidx[remove_pt & (point_reason != 2) & (point_reason != 8)]
    for _ in range(2):
        if not len(obj_global):
            break
        ot = cKDTree(np.stack([X[obj_global], Y[obj_global], Z[obj_global]], 1))
        oI = I[obj_global]
        newly = []
        left = np.nonzero(~rm_all)[0]
        for a in range(0, len(left), 6_000_000):
            q = left[a:a + 6_000_000]
            d, j = ot.query(np.stack([X[q], Y[q], Z[q]], 1), k=1, workers=-1,
                            distance_upper_bound=COMPLETE_R)
            hit = np.isfinite(d)
            src_bright = oI[np.clip(j[hit], 0, len(oI) - 1)] >= OBJ_I_MED_HI
            okj = np.where(src_bright, I[q[hit]] >= COMPLETE_BRIGHT,
                           I[q[hit]] <= COMPLETE_DARK)
            newly.append(q[hit][okj])
        newly = np.concatenate(newly) if newly else np.array([], np.int64)
        if not len(newly):
            break
        rm_all[newly] = True
        reasons[newly] = 7
        obj_global = newly
    sparse_global = cidx[remove_pt & (point_reason == 2)]
    if len(sparse_global):
        stree = cKDTree(np.stack([X[sparse_global], Y[sparse_global],
                                  Z[sparse_global]], 1))
        left = np.nonzero(~rm_all & (dens < SPARSE_DENSITY_MAX) &
                          (HOVER3 >= HOVER3_COMPLETE) &
                          (RELIEF <= WALL_RELIEF_MAX))[0]
        for a in range(0, len(left), 6_000_000):
            q = left[a:a + 6_000_000]
            d, _ = stree.query(np.stack([X[q], Y[q], Z[q]], 1), k=1, workers=-1,
                               distance_upper_bound=0.025)
            hit = np.isfinite(d)
            rm_all[q[hit]] = True
            reasons[q[hit]] = 7
    reasons[~rm_all] = 0
    log(f"total 5 mm removal: {int(rm_all.sum()):,} "
        f"({rm_all.sum() / len(X) * 100:.2f}%)")

    # ---- split back per role, save, summarize ----------------------------
    summary = {}
    removed_xyz = []
    for role in ("SAND", "ROCK"):
        lo, hi = role_span[role]
        role_reasons = reasons[lo:hi]
        np.save(work / f"reasons-{role}-5mm.npy", role_reasons)
        counts = {name: int((role_reasons == code).sum())
                  for code, name in REASON_NAMES.items()}
        counts["total_removed"] = int((role_reasons > 0).sum())
        counts["total_points"] = int(hi - lo)
        summary[f"{role}-5mm"] = counts
        print(f"[classify] {role}-5mm: {counts}", flush=True)
        removed_xyz.append(np.stack(
            [X[lo:hi][role_reasons > 0], Y[lo:hi][role_reasons > 0],
             Z[lo:hi][role_reasons > 0]], 1))
    inherit_reason = np.concatenate(
        [reasons[np.arange(*role_span[role])][
             reasons[np.arange(*role_span[role])] > 0]
         for role in ("SAND", "ROCK")])
    inherit_tree = cKDTree(np.vstack(removed_xyz))

    # ---- 1 mm: pure inheritance (the 5 mm cloud is a greedy min-spacing
    # ---- subset of the 1 mm cloud, so every 1 mm member of a removed
    # ---- structure has a removed 5 mm point within the inherit radius) ---
    for role in ("SAND", "ROCK"):
        key = f"{role}-1mm"
        cloud, _, _ = memmap_cloud(cloud_path(data_dir, role, "1mm"))
        fine_reasons = np.zeros(len(cloud), np.uint8)
        for a in range(0, len(cloud), 8_000_000):
            k = cloud[a:a + 8_000_000]
            pts = np.stack([k["x"].astype(np.float32),
                            k["y"].astype(np.float32),
                            k["z"].astype(np.float32)], 1)
            distance, neighbour = inherit_tree.query(
                pts, k=1, workers=-1, distance_upper_bound=INHERIT_RADIUS)
            hit = np.isfinite(distance)
            fine_reasons[a + np.nonzero(hit)[0]] = inherit_reason[neighbour[hit]]
        np.save(work / f"reasons-{key}.npy", fine_reasons)
        counts = {name: int((fine_reasons == code).sum())
                  for code, name in REASON_NAMES.items()}
        counts["total_removed"] = int((fine_reasons > 0).sum())
        counts["total_points"] = int(len(fine_reasons))
        summary[key] = counts
        print(f"[classify] {key}: {counts}", flush=True)
    (work / "classify-summary.json").write_text(json.dumps(summary, indent=2))
    return summary


# ---- apply / restore ----------------------------------------------------

def patched_header(header: bytes, new_count: int) -> bytes:
    text = header.decode("ascii")
    match = re.search(r"(element vertex )(\d+)( *)", text)
    field = match.group(2) + match.group(3)
    replacement = str(new_count).ljust(len(field))
    if len(replacement) > len(field):
        raise RuntimeError("new vertex count does not fit the header field")
    patched = text[:match.start(2)] + replacement + text[match.end(3):]
    return patched.encode("ascii")


def stage_apply(data_dir: Path, work: Path, run_name: str = RUN_NAME):
    run_dir = data_dir / "PatchRefinement" / run_name
    run_dir.mkdir(parents=True, exist_ok=True)
    (data_dir / "PatchRefinement" / ".invisible_places-ignore").touch()
    manifest = {"run": run_name, "created": _dt.datetime.now().isoformat(),
                "parameters": {
                    "manual_distance": MANUAL_DISTANCE, "below_dz": BELOW_DZ,
                    "sparse_density_max": SPARSE_DENSITY_MAX,
                    "hover3_remove": HOVER3_REMOVE,
                    "wall_relief_max": WALL_RELIEF_MAX,
                    "obj_i_med_hi": OBJ_I_MED_HI, "obj_i_med_lo": OBJ_I_MED_LO,
                    "sub_i_hi": SUB_I_HI, "complete_r": COMPLETE_R},
                "files": {}}
    for role, spacing in CLEAN_TARGETS:
        key = f"{role}-{spacing}"
        path = cloud_path(data_dir, role, spacing)
        reasons = np.load(work / f"reasons-{key}.npy")
        removed_index = np.nonzero(reasons > 0)[0]
        if len(removed_index) == 0:
            print(f"[apply] {key}: nothing to remove")
            continue
        cloud, dtype, header = memmap_cloud(path)
        if len(reasons) != len(cloud):
            raise RuntimeError(f"{key}: classification is stale (count mismatch)")
        sha_before = sha256_path(path)

        removed_records = cloud[removed_index]
        rollback_ply = run_dir / f"Site1-{role}-{spacing}.removed.ply"
        with open(rollback_ply, "wb") as handle:
            handle.write(patched_header(header, len(removed_records)))
            removed_records.tofile(handle)
        np.save(run_dir / f"Site1-{role}-{spacing}.removed_indices.npy", removed_index)
        np.save(run_dir / f"Site1-{role}-{spacing}.removed_reasons.npy",
                reasons[removed_index])
        (run_dir / f"Site1-{role}-{spacing}.header.bin").write_bytes(header)

        keep_count = len(cloud) - len(removed_index)
        temp = path.with_suffix(".ply.tmp")
        with open(temp, "wb") as handle:
            handle.write(patched_header(header, keep_count))
            for start in range(0, len(cloud), 8_000_000):
                chunk = cloud[start:start + 8_000_000]
                keep = reasons[start:start + len(chunk)] == 0
                chunk[keep].tofile(handle)
        del cloud
        temp.replace(path)
        sha_after = sha256_path(path)
        manifest["files"][key] = {
            "source": str(path), "sha256_before": sha_before,
            "sha256_after": sha_after, "removed": int(len(removed_index)),
            "kept": int(keep_count),
            "reasons": {name: int((reasons[removed_index] == code).sum())
                        for code, name in REASON_NAMES.items()},
        }
        print(f"[apply] {key}: removed {len(removed_index):,} of {len(reasons):,} "
              f"({len(removed_index)/len(reasons)*100:.2f}%) -> rollback {rollback_ply.name}")
    (run_dir / "manifest.json").write_text(json.dumps(manifest, indent=2))
    print(f"[apply] manifest -> {run_dir/'manifest.json'}")


def stage_restore(data_dir: Path, run_name: str = RUN_NAME):
    run_dir = data_dir / "PatchRefinement" / run_name
    manifest = json.loads((run_dir / "manifest.json").read_text())
    for key, entry in manifest["files"].items():
        role, spacing = key.split("-")
        path = cloud_path(data_dir, role, spacing)
        removed_index = np.load(run_dir / f"Site1-{role}-{spacing}.removed_indices.npy")
        header = (run_dir / f"Site1-{role}-{spacing}.header.bin").read_bytes()
        removed, dtype, _ = memmap_cloud(run_dir / f"Site1-{role}-{spacing}.removed.ply")
        current, _, _ = memmap_cloud(path)
        total = len(current) + len(removed)
        merged_temp = path.with_suffix(".ply.restore_tmp")
        # streamed merge over the original index space: for output range
        # [a, b) the removed rows are a searchsorted slice and the kept rows
        # are the contiguous complement in the current (cleaned) file.
        with open(merged_temp, "wb") as handle:
            handle.write(header)
            chunk_rows = 8_000_000
            for a in range(0, total, chunk_rows):
                b = min(a + chunk_rows, total)
                r_lo = np.searchsorted(removed_index, a, side="left")
                r_hi = np.searchsorted(removed_index, b, side="left")
                output = np.empty(b - a, dtype)
                mask_removed = np.zeros(b - a, bool)
                mask_removed[removed_index[r_lo:r_hi] - a] = True
                output[mask_removed] = removed[r_lo:r_hi]
                k_lo = a - r_lo
                k_hi = b - r_hi
                output[~mask_removed] = current[k_lo:k_hi]
                output.tofile(handle)
        del current, removed
        digest = sha256_path(merged_temp)
        if digest != entry["sha256_before"]:
            raise RuntimeError(f"{key}: restore verification failed ({digest})")
        merged_temp.replace(path)
        print(f"[restore] {key}: byte-exact restore verified ({digest[:12]}...)")


# ---- cloth --------------------------------------------------------------

def stage_cloth(context: Context, data_dir: Path):
    from scipy import ndimage
    out = data_dir / "Site1-ClothMesh-Refined.ply"
    fine_cell = 0.05
    scale = int(REGION_CELL / fine_cell)
    filled = fill_nan_nearest(context.cloth_filled.copy())
    fine = np.kron(filled, np.ones((scale, scale), np.float32))
    fine = ndimage.gaussian_filter(fine, 1.0)
    hull_fine = np.kron(context.hull, np.ones((scale, scale), bool))
    ny, nx = fine.shape
    node_index = np.full((ny, nx), -1, np.int64)
    valid = hull_fine
    node_index[valid] = np.arange(valid.sum())
    vy, vx = np.nonzero(valid)
    vertices = np.zeros(valid.sum(), np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                                               ("nx", "<f4"), ("ny", "<f4"), ("nz", "<f4")]))
    vertices["x"] = context.x0 + (vx + 0.5) * fine_cell
    vertices["y"] = context.y0 + (vy + 0.5) * fine_cell
    vertices["z"] = fine[vy, vx]
    gy, gx = np.gradient(fine, fine_cell)
    normals = np.stack([-gx[vy, vx], -gy[vy, vx], np.ones(valid.sum(), np.float32)], 1)
    normals /= np.linalg.norm(normals, axis=1, keepdims=True)
    vertices["nx"], vertices["ny"], vertices["nz"] = normals.T

    quads_y, quads_x = np.nonzero(valid[:-1, :-1] & valid[1:, :-1] & valid[:-1, 1:] & valid[1:, 1:])
    a = node_index[quads_y, quads_x]
    b = node_index[quads_y, quads_x + 1]
    c = node_index[quads_y + 1, quads_x + 1]
    d = node_index[quads_y + 1, quads_x]
    faces = np.empty((2 * len(a), 3), np.int32)
    faces[0::2] = np.stack([a, b, c], 1)
    faces[1::2] = np.stack([a, c, d], 1)
    header = ["ply", "format binary_little_endian 1.0",
              f"comment Refined CSF cloth: harmonic gap fill + 0.05 m resample ({_dt.date.today().isoformat()})",
              "comment Generated by scripts/site1_clean_and_cloth_water.py",
              f"element vertex {len(vertices)}",
              "property float x", "property float y", "property float z",
              "property float nx", "property float ny", "property float nz",
              f"element face {len(faces)}",
              "property list uchar int vertex_indices", "end_header"]
    face_records = np.empty(len(faces), np.dtype([("n", "u1"), ("v", "<i4", 3)]))
    face_records["n"] = 3
    face_records["v"] = faces
    with open(out, "wb") as handle:
        handle.write(("\n".join(header) + "\n").encode("ascii"))
        vertices.tofile(handle)
        face_records.tofile(handle)
    print(f"[cloth] wrote {out} ({len(vertices):,} verts, {len(faces):,} faces)")
    return fine, fine_cell


# ---- water --------------------------------------------------------------

def fill_nan_nearest(grid):
    """Replace NaNs with the nearest finite value (spline prefilters are
    global, so a single NaN would otherwise poison bicubic sampling)."""
    from scipy.ndimage import distance_transform_edt
    mask = ~np.isfinite(grid)
    if not mask.any():
        return grid
    index = distance_transform_edt(mask, return_distances=False, return_indices=True)
    return grid[tuple(index)]


def sample_grid(grid, px, py, gx0, gy0, cell, order=1):
    """Sample a cell-centred grid at world coordinates (chunk-friendly)."""
    from scipy.ndimage import map_coordinates
    u = (px - gx0) / cell - 0.5
    v = (py - gy0) / cell - 0.5
    return map_coordinates(grid, [v, u], order=order, mode="nearest").astype(np.float32)


def _disk_close(mask, radius_cells):
    """Binary closing with a Euclidean disk and an explicit empty border."""
    from scipy import ndimage
    radius_cells = int(radius_cells)
    if radius_cells <= 0:
        return mask.copy()
    padded = np.pad(mask, radius_cells, mode="constant", constant_values=False)
    dilated = ndimage.distance_transform_edt(~padded) <= radius_cells
    closed = ndimage.distance_transform_edt(dilated) > radius_cells
    return closed[radius_cells:-radius_cells, radius_cells:-radius_cells]


def _normalised_gaussian(values, known, sigma):
    """Gaussian mean and support weight without allowing NaNs to spread."""
    from scipy import ndimage
    numerator = ndimage.gaussian_filter(
        np.where(known, values, 0.0).astype(np.float32), sigma)
    weight = ndimage.gaussian_filter(known.astype(np.float32), sigma)
    mean = np.divide(numerator, weight, out=np.full_like(numerator, np.nan),
                     where=weight > 1e-4)
    return mean, weight


def build_water_xy_recovery(sand_count, sand_zsum, rock_count, rock_zsum,
                            in_hull, base_allowed, cell=WATER_SUPPORT_CELL):
    """Recover small steep-mask holes only where original SAND is continuous.

    The XY closing identifies gaps surrounded by the SAND layer.  A sand-only
    height sheet and local height variance provide the 3D-connectedness test;
    high, locally dominant ROCK and large/open regions remain excluded.  The
    returned confidence is softly feathered to avoid a new cell-shaped edge.
    """
    from scipy import ndimage

    shape = in_hull.shape
    arrays = (sand_count, sand_zsum, rock_count, rock_zsum, base_allowed)
    if any(array.shape != shape for array in arrays):
        raise ValueError("water recovery grids must have matching shapes")

    sand_z = np.divide(
        sand_zsum, sand_count,
        out=np.full(shape, np.nan, np.float32), where=sand_count > 0)
    rock_z = np.divide(
        rock_zsum, rock_count,
        out=np.full(shape, np.nan, np.float32), where=rock_count > 0)
    sand_anchor = (in_hull & (sand_count > 0) & np.isfinite(sand_z) &
                   (sand_z <= WATER_SUPPORT_Z_CAP))
    empty_confidence = np.zeros(shape, np.float32)
    if not sand_anchor.any():
        stats = {"sand_anchor_cells": 0, "xy_continuity_cells": 0,
                 "recovered_cells": 0, "recovered_area_m2": 0.0,
                 "rock_veto_cells": 0, "z_veto_cells": 0,
                 "large_component_veto_cells": 0}
        return empty_confidence, np.full(shape, np.nan, np.float32), stats

    seed_radius = max(1, int(round(WATER_XY_SEED_CLOSE_M / cell)))
    close_radius = max(seed_radius, int(round(WATER_XY_CLOSE_M / cell)))
    sand_surface = _disk_close(sand_anchor, seed_radius)
    xy_continuity = _disk_close(sand_surface, close_radius)

    sigma = max(1.0, WATER_Z_COHERENCE_SIGMA_M / cell)
    sand_sheet, sand_weight = _normalised_gaussian(sand_z, sand_anchor, sigma)
    sand_z2, _ = _normalised_gaussian(sand_z * sand_z, sand_anchor, sigma)
    sand_z_std = np.sqrt(np.maximum(sand_z2 - sand_sheet * sand_sheet, 0.0))

    sand_local = ndimage.gaussian_filter(sand_anchor.astype(np.float32), sigma)
    high_rock = ((rock_count > 0) & np.isfinite(rock_z) &
                 (rock_z > sand_sheet + WATER_ROCK_CLEARANCE))
    rock_local = ndimage.gaussian_filter(high_rock.astype(np.float32), sigma)
    rock_dominant = (high_rock |
                     ((rock_local > WATER_ROCK_LOCAL_MIN) &
                      (rock_local > WATER_ROCK_DOMINANCE * sand_local)))

    edge_distance = ndimage.distance_transform_edt(in_hull) * cell
    candidate = (in_hull & ~base_allowed & xy_continuity &
                 (edge_distance >= WATER_RECOVERY_EDGE_CLEAR_M) &
                 (sand_weight > 0.01) & (sand_sheet <= WATER_SUPPORT_Z_CAP) &
                 (sand_z_std <= WATER_Z_STD_MAX) & ~rock_dominant)

    labels, component_count = ndimage.label(
        candidate, structure=np.ones((3, 3), bool))
    sizes = np.bincount(labels.ravel(), minlength=component_count + 1)
    max_cells = int(WATER_RECOVERY_MAX_COMPONENT_M2 / (cell * cell))
    too_large = candidate & (sizes[labels] > max_cells)
    recovered = candidate & ~too_large

    # A one-cell halo and soft confidence make the recovery edge sub-cell and
    # irregular after jittering, instead of exposing another square mask.
    confidence = ndimage.gaussian_filter(recovered.astype(np.float32), 1.0)
    confidence = np.clip(confidence / 0.5, 0.0, 1.0)
    halo = ndimage.binary_dilation(recovered, iterations=1)
    confidence[~(halo & in_hull)] = 0.0

    stats = {
        "sand_anchor_cells": int(sand_anchor.sum()),
        "xy_continuity_cells": int((xy_continuity & in_hull).sum()),
        "recovered_cells": int(recovered.sum()),
        "recovered_area_m2": float(recovered.sum() * cell * cell),
        "rock_veto_cells": int((xy_continuity & in_hull & rock_dominant).sum()),
        "z_veto_cells": int((xy_continuity & in_hull &
                             (sand_z_std > WATER_Z_STD_MAX)).sum()),
        "large_component_veto_cells": int(too_large.sum()),
    }
    return confidence.astype(np.float32), sand_sheet.astype(np.float32), stats


def stage_water(context: Context, data_dir: Path, work: Path):
    import open3d as o3d
    from scipy import ndimage
    from scipy.spatial import cKDTree

    stamp = _dt.date.today().isoformat()
    x0 = context.x0 - 40.0
    y0 = context.y0 - 45.0
    nx = int(120.0 / DENSITY_CELL)
    ny = int(165.0 / DENSITY_CELL)
    ATTR_CELL = 0.025
    anx = int(120.0 / ATTR_CELL)
    any_ = int(165.0 / ATTR_CELL)
    idw_fields = ["red", "green", "blue", "scalar_Intensity", "scalar_Composite",
                  "scalar_A_R_Shelter_Lower", "scalar_A_R_RainExposure_Lower",
                  "scalar_A_R_SVF_Lower"]
    support_nx = int(context.nx * REGION_CELL / WATER_SUPPORT_CELL) + 1
    support_ny = int(context.ny * REGION_CELL / WATER_SUPPORT_CELL) + 1
    sand_support_count = np.zeros((support_ny, support_nx), np.uint32)
    rock_support_count = np.zeros((support_ny, support_nx), np.uint32)
    sand_support_zsum = np.zeros((support_ny, support_nx), np.float64)
    rock_support_zsum = np.zeros((support_ny, support_nx), np.float64)

    # ---- one streaming pass: 1 cm coverage counts + measured heights,
    # ---- 2.5 cm SAND/ROCK continuity and attribute grids, plus keep-clear XY
    counts = np.zeros((ny, nx), np.float32)
    zsum = np.zeros(ny * nx, np.float64)
    zcnt = np.zeros(ny * nx, np.int64)
    attr_count = np.zeros(any_ * anx, np.float32)
    attr_sum = {name: np.zeros(any_ * anx, np.float32) for name in idw_fields}
    attr_sq = {name: np.zeros(any_ * anx, np.float32) for name in idw_fields}
    clear_xy = []
    for role in ("SAND", "ROCK", "VEG"):
        cloud, _, _ = memmap_cloud(cloud_path(data_dir, role, "5mm"))
        for start in range(0, len(cloud), 8_000_000):
            chunk = cloud[start:start + 8_000_000]
            gx = ((chunk["x"] - x0) / DENSITY_CELL).astype(np.int64)
            gy = ((chunk["y"] - y0) / DENSITY_CELL).astype(np.int64)
            inside = (gx >= 0) & (gx < nx) & (gy >= 0) & (gy < ny)
            flat = gy[inside] * nx + gx[inside]
            np.add.at(counts.ravel(), flat, 1.0)
            if role in ("SAND", "ROCK"):
                sx = ((chunk["x"] - context.x0) / WATER_SUPPORT_CELL).astype(np.int64)
                sy = ((chunk["y"] - context.y0) / WATER_SUPPORT_CELL).astype(np.int64)
                support_inside = ((sx >= 0) & (sx < support_nx) &
                                  (sy >= 0) & (sy < support_ny))
                support_flat = sy[support_inside] * support_nx + sx[support_inside]
                support_z = chunk["z"][support_inside].astype(np.float64)
                if role == "SAND":
                    np.add.at(sand_support_count.ravel(), support_flat, 1)
                    np.add.at(sand_support_zsum.ravel(), support_flat, support_z)
                else:
                    np.add.at(rock_support_count.ravel(), support_flat, 1)
                    np.add.at(rock_support_zsum.ravel(), support_flat, support_z)
            if role == "VEG":
                continue
            np.add.at(zsum, flat, chunk["z"][inside].astype(np.float64))
            np.add.at(zcnt, flat, 1)
            ax = ((chunk["x"] - x0) / ATTR_CELL).astype(np.int64)
            ay = ((chunk["y"] - y0) / ATTR_CELL).astype(np.int64)
            ainside = (ax >= 0) & (ax < anx) & (ay >= 0) & (ay < any_)
            aflat = ay[ainside] * anx + ax[ainside]
            np.add.at(attr_count, aflat, 1.0)
            for name in idw_fields:
                values = np.asarray(chunk[name], dtype=np.float32)[ainside]
                np.add.at(attr_sum[name], aflat, values)
                np.add.at(attr_sq[name], aflat, values * values)
            clear_xy.append(np.stack([chunk["x"][inside], chunk["y"][inside]], 1)
                            .astype(np.float32))
    sigma_cells = DENSITY_SIGMA_M / DENSITY_CELL
    density = ndimage.gaussian_filter(counts, sigma_cells) / (DENSITY_CELL ** 2)
    covered = density[density > WELL_COVERED_MIN]
    target = float(np.percentile(covered, TARGET_PERCENTILE))
    print(f"[water] density target = {target:,.0f} pts/m^2 "
          f"(median covered {np.median(covered):,.0f})", flush=True)
    zmean = np.divide(zsum, zcnt, out=np.full(ny * nx, np.nan), where=zcnt > 0)
    zmean = zmean.reshape(ny, nx).astype(np.float32)
    zmean_s = ndimage.gaussian_filter(np.nan_to_num(zmean, nan=0.0), 2.0)
    zmean_w = ndimage.gaussian_filter(np.isfinite(zmean).astype(np.float32), 2.0)
    del zsum, zcnt, zmean

    # ---- role-aware XY/3D recovery at 2.5 cm.  This is deliberately based
    # ---- on original SAND only; ROCK can veto the sheet but never define it.
    support_x = context.x0 + (np.arange(support_nx) + 0.5) * WATER_SUPPORT_CELL
    support_y = context.y0 + (np.arange(support_ny) + 0.5) * WATER_SUPPORT_CELL
    context_x = np.clip(((support_x - context.x0) / REGION_CELL).astype(np.int64),
                        0, context.nx - 1)
    context_y = np.clip(((support_y - context.y0) / REGION_CELL).astype(np.int64),
                        0, context.ny - 1)
    support_hull = context.hull[context_y[:, None], context_x[None, :]]
    support_base_allowed = (
        support_hull & ~context.steep_near[context_y[:, None], context_x[None, :]] &
        (context.cloth_filled[context_y[:, None], context_x[None, :]] <= OUTSIDE_Z_CAP))
    recovery_confidence, sand_recovery_sheet, recovery_stats = build_water_xy_recovery(
        sand_support_count, sand_support_zsum, rock_support_count, rock_support_zsum,
        support_hull, support_base_allowed, WATER_SUPPORT_CELL)
    print("[water] sand XY/3D recovery: "
          f"{recovery_stats['recovered_cells']:,} support cells "
          f"({recovery_stats['recovered_area_m2']:.1f} m^2); "
          f"rock veto {recovery_stats['rock_veto_cells']:,}, "
          f"height veto {recovery_stats['z_veto_cells']:,}, "
          f"large-patch veto {recovery_stats['large_component_veto_cells']:,}",
          flush=True)
    del sand_support_count, sand_support_zsum, rock_support_count, rock_support_zsum

    # ---- region routing masks (cell level)
    hull_gx = ((np.arange(nx) * DENSITY_CELL + x0 - context.x0) / REGION_CELL).astype(np.int64)
    hull_gy = ((np.arange(ny) * DENSITY_CELL + y0 - context.y0) / REGION_CELL).astype(np.int64)
    okx = (hull_gx >= 0) & (hull_gx < context.nx)
    oky = (hull_gy >= 0) & (hull_gy < context.ny)
    in_hull = np.zeros((ny, nx), bool)
    sub = context.hull[np.clip(hull_gy, 0, context.ny - 1)[:, None],
                       np.clip(hull_gx, 0, context.nx - 1)[None, :]]
    in_hull[np.ix_(oky, okx)] = sub[np.ix_(oky, okx)]
    hull_mix = ndimage.gaussian_filter(in_hull.astype(np.float32), 50.0)
    # The fill stays roughly at water level: no candidates on or near steep
    # cloth (the ledge faces produced draped drip-cones) and none above the
    # cap (no water high on the rock tops).
    flat_sub = (~context.steep_near)[np.clip(hull_gy, 0, context.ny - 1)[:, None],
                                     np.clip(hull_gx, 0, context.nx - 1)[None, :]]
    cloth_sub = context.cloth_filled[np.clip(hull_gy, 0, context.ny - 1)[:, None],
                                     np.clip(hull_gx, 0, context.nx - 1)[None, :]]
    cloth_capped = np.nan_to_num(cloth_sub, nan=OUTSIDE_Z_CAP + 1.0) <= OUTSIDE_Z_CAP
    level_ok = np.zeros((ny, nx), bool)
    level_ok[np.ix_(oky, okx)] = (flat_sub & cloth_capped)[np.ix_(oky, okx)]

    # Add only the support-grid recovery.  Sampling and assignment are limited
    # to the small ToMesh rectangle so the 120 x 165 m density grid does not
    # need another full-size float allocation.
    recover_gx = ((np.arange(nx) * DENSITY_CELL + x0 - context.x0) /
                  WATER_SUPPORT_CELL).astype(np.int64)
    recover_gy = ((np.arange(ny) * DENSITY_CELL + y0 - context.y0) /
                  WATER_SUPPORT_CELL).astype(np.int64)
    recover_okx = (recover_gx >= 0) & (recover_gx < support_nx)
    recover_oky = (recover_gy >= 0) & (recover_gy < support_ny)
    recover_x = np.nonzero(recover_okx)[0]
    recover_y = np.nonzero(recover_oky)[0]
    if len(recover_x) and len(recover_y):
        block = level_ok[np.ix_(recover_y, recover_x)]
        sampled_recovery = recovery_confidence[
            recover_gy[recover_y, None], recover_gx[None, recover_x]]
        block |= sampled_recovery >= WATER_RECOVERY_MIN_CONFIDENCE
        level_ok[np.ix_(recover_y, recover_x)] = block

    # ---- outside surface: mesh raycast bounds the extent, a coarse (5 cm)
    # ---- harmonic sheet from measured rims provides the height
    mesh = o3d.io.read_triangle_mesh(str(data_dir / "Site1-MESH.ply"))
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
    deficit = (density < target * FILL_TRIGGER) & ~in_hull
    coarse_factor = 10
    fny, fnx = ny // coarse_factor, nx // coarse_factor
    occupied_coarse = counts[:fny * coarse_factor, :fnx * coarse_factor].reshape(
        fny, coarse_factor, fnx, coarse_factor).sum(axis=(1, 3)) > 0
    span = np.mgrid[-20:21, -20:21]
    disk2m = np.hypot(span[0], span[1]) <= 20
    footprint_coarse = ndimage.binary_fill_holes(
        ndimage.binary_closing(occupied_coarse, structure=disk2m))
    footprint_coarse &= ndimage.binary_dilation(occupied_coarse, structure=disk2m)
    footprint = np.zeros((ny, nx), bool)
    footprint[:fny * coarse_factor, :fnx * coarse_factor] = np.kron(
        footprint_coarse, np.ones((coarse_factor, coarse_factor), bool))
    outside_mask = deficit & footprint
    oy, ox = np.nonzero(outside_mask)
    allowed = np.zeros((ny, nx), bool)
    solved = np.full((ny // 5, nx // 5), np.nan, np.float32)
    if len(ox):
        rays = np.zeros((len(ox), 6), np.float32)
        rays[:, 0] = x0 + (ox + 0.5) * DENSITY_CELL
        rays[:, 1] = y0 + (oy + 0.5) * DENSITY_CELL
        rays[:, 2] = 60.0; rays[:, 5] = -1.0
        hit = scene.cast_rays(o3d.core.Tensor(rays))["t_hit"].numpy()
        zray = 60.0 - hit
        rayok = np.isfinite(hit) & (zray <= OUTSIDE_Z_CAP)
        allowed[oy[rayok], ox[rayok]] = True
        factor = 5
        cny, cnx = ny // factor, nx // factor
        coarse_mask = allowed[:cny * factor, :cnx * factor].reshape(
            cny, factor, cnx, factor).any(axis=(1, 3))
        zdata_grid = np.divide(zmean_s, zmean_w,
                               out=np.full_like(zmean_s, np.nan),
                               where=zmean_w > 0.05)
        zdata_coarse = np.nanmean(
            zdata_grid[:cny * factor, :cnx * factor].reshape(
                cny, factor, cnx, factor), axis=(1, 3))
        rim = ndimage.binary_dilation(coarse_mask, iterations=2) & ~coarse_mask
        seed = np.where(rim & np.isfinite(zdata_coarse), zdata_coarse, np.nan)
        solved = context._harmonic_fill(seed, coarse_mask | rim)
        solved = np.minimum(solved, OUTSIDE_Z_CAP).astype(np.float32)
        del zdata_grid

    # ---- candidates from the density deficit
    rng = np.random.default_rng(41)
    fill_region = (density < target * FILL_TRIGGER) & ((in_hull & level_ok) | allowed)
    fy, fx = np.nonzero(fill_region)
    print(f"[water] fill region: {len(fx):,} cells ({len(fx)*DENSITY_CELL**2:.0f} m^2)",
          flush=True)
    per_cell = 4
    offsets = np.array([[0.25, 0.25], [0.75, 0.25], [0.25, 0.75], [0.75, 0.75]], np.float32)
    px = (x0 + (fx[:, None] + offsets[None, :, 0]) * DENSITY_CELL).ravel()
    py = (y0 + (fy[:, None] + offsets[None, :, 1]) * DENSITY_CELL).ravel()
    px += rng.uniform(-0.0015, 0.0015, len(px)).astype(np.float32)
    py += rng.uniform(-0.0015, 0.0015, len(py)).astype(np.float32)
    cell_density = np.repeat(density[fy, fx], per_cell)
    accept = rng.random(len(px)) < np.clip(1.0 - cell_density / target, 0.0, 1.0)
    px, py = px[accept].astype(np.float32), py[accept].astype(np.float32)
    print(f"[water] accepted {len(px):,} candidates from the density deficit", flush=True)

    gxc = np.clip(((px - x0) / DENSITY_CELL).astype(np.int64), 0, nx - 1)
    gyc = np.clip(((py - y0) / DENSITY_CELL).astype(np.int64), 0, ny - 1)
    near_data = density[gyc, gxc] > target * 0.02
    if near_data.any():
        clear_tree = cKDTree(np.vstack(clear_xy))
        distance, _ = clear_tree.query(np.stack([px[near_data], py[near_data]], 1),
                                       k=1, workers=-1, distance_upper_bound=KEEP_CLEAR)
        collide = np.zeros(len(px), bool)
        collide[np.nonzero(near_data)[0][np.isfinite(distance)]] = True
        px, py = px[~collide], py[~collide]
        gxc, gyc = gxc[~collide], gyc[~collide]
        print(f"[water] keep-clear dropped {collide.sum():,}", flush=True)
    del clear_xy

    # ---- continuous fill surface: bicubic cloth inside the hull, bicubic
    # ---- harmonic sheet outside, hull-blended; measured surface wins where
    # ---- real points exist. No cell-quantised lookups anywhere.
    cloth_sample = fill_nan_nearest(context.cloth_filled.copy())

    if np.isfinite(sand_recovery_sheet).any():
        sand_recovery_sample = fill_nan_nearest(sand_recovery_sheet.copy())
    else:
        sand_recovery_sample = np.full_like(
            sand_recovery_sheet, np.nanmedian(context.cloth_filled))

    def surface_z(qx, qy):
        z_cloth = sample_grid(cloth_sample, qx, qy,
                              context.x0, context.y0, REGION_CELL, order=3)
        z_out = sample_grid(solved, qx, qy, x0, y0, DENSITY_CELL * 5, order=1)
        z_out = np.where(np.isfinite(z_out), z_out, z_cloth)
        mix = sample_grid(hull_mix, qx, qy, x0, y0, DENSITY_CELL, order=1)
        z_ref = mix * z_cloth + (1.0 - mix) * z_out
        num = sample_grid(zmean_s, qx, qy, x0, y0, DENSITY_CELL, order=1)
        den = sample_grid(zmean_w, qx, qy, x0, y0, DENSITY_CELL, order=1)
        has_data = den > 0.05
        z_dat = np.where(has_data, num / np.maximum(den, 1e-6), np.nan)
        local_density = sample_grid(density, qx, qy, x0, y0, DENSITY_CELL, order=1)
        weight = np.clip(local_density / (0.5 * target), 0.0, 1.0)
        weight[~has_data] = 0.0
        blended = (1.0 - weight) * z_ref + weight * np.nan_to_num(z_dat, nan=0.0)
        low = np.where(has_data, np.minimum(z_ref, np.nan_to_num(z_dat, nan=np.inf)),
                       z_ref) - 0.08
        high = np.where(has_data, np.maximum(z_ref, np.nan_to_num(z_dat, nan=-np.inf)),
                        z_ref) + 0.08
        bounded = np.clip(blended, low, high).astype(np.float32)
        recovery = sample_grid(
            recovery_confidence, qx, qy, context.x0, context.y0,
            WATER_SUPPORT_CELL, order=1)
        recovery = np.clip(recovery, 0.0, 1.0)
        z_sand = sample_grid(
            sand_recovery_sample, qx, qy, context.x0, context.y0,
            WATER_SUPPORT_CELL, order=1)
        surface = ((1.0 - recovery) * bounded + recovery * z_sand).astype(np.float32)
        # Last-resort guard against sparse high ROCK leaking through z_dat.
        return np.minimum(surface, WATER_SUPPORT_Z_CAP)

    pz = np.empty(len(px), np.float32)
    normal = np.empty((len(px), 3), np.float32)
    delta = 0.02
    for start in range(0, len(px), 4_000_000):
        stop = min(start + 4_000_000, len(px))
        qx, qy = px[start:stop], py[start:stop]
        centre = surface_z(qx, qy)
        dzdx = (surface_z(qx + delta, qy) - surface_z(qx - delta, qy)) / (2 * delta)
        dzdy = (surface_z(qx, qy + delta) - surface_z(qx, qy - delta)) / (2 * delta)
        vector = np.stack([-dzdx, -dzdy, np.ones(stop - start, np.float32)], 1)
        vector /= np.linalg.norm(vector, axis=1, keepdims=True)
        pz[start:stop] = centre
        normal[start:stop] = vector
    pz += rng.normal(0.0, 0.0005, len(pz)).astype(np.float32)
    np.minimum(pz, WATER_SUPPORT_Z_CAP, out=pz)

    # ---- attributes: cell means diffused outward (multi-scale masked
    # ---- Gaussian) and sampled bilinearly, plus correlated noise whose
    # ---- amplitude follows the local measured variance
    def diffuse(mean_grid):
        filled = mean_grid.copy()
        for sigma in (2.0, 4.0, 8.0, 16.0, 32.0, 64.0, 128.0):
            known = np.isfinite(filled).astype(np.float32)
            blurred = ndimage.gaussian_filter(np.nan_to_num(filled, nan=0.0), sigma)
            weightg = ndimage.gaussian_filter(known, sigma)
            estimate = np.divide(blurred, weightg,
                                 out=np.zeros_like(blurred), where=weightg > 1e-4)
            need = ~np.isfinite(filled) & (weightg > 1e-4)
            filled[need] = estimate[need]
        remaining = ~np.isfinite(filled)
        if remaining.any():
            filled[remaining] = np.nanmedian(mean_grid)
        return ndimage.gaussian_filter(filled, 1.0)

    have_attr = attr_count > 0
    noise_a = ndimage.gaussian_filter(
        rng.standard_normal((any_, anx)).astype(np.float32), 4.0)
    noise_a /= max(noise_a.std(), 1e-6)
    noise_b = ndimage.gaussian_filter(
        rng.standard_normal((any_, anx)).astype(np.float32), 4.0)
    noise_b /= max(noise_b.std(), 1e-6)
    noise_amplitude = 0.7

    dtype, _, _, _ = read_header(cloud_path(data_dir, "SAND", "5mm"))
    record = np.zeros(len(px), dtype)
    record["x"], record["y"], record["z"] = px, py, pz
    record["nx"], record["ny"], record["nz"] = normal.T
    slope = np.degrees(np.arccos(np.clip(normal[:, 2], -1.0, 1.0))).astype(np.float32)
    record["scalar_ScanID"] = WATER_SCAN_ID
    record["scalar_A_R_Horizontalness"] = np.cos(np.radians(slope))
    record["scalar_A_R_Slope_deg"] = slope
    horizontal = np.linalg.norm(normal[:, :2], axis=1)
    safe = horizontal > 1e-5
    record["scalar_A_R_Downhill_X"][safe] = normal[safe, 0] / horizontal[safe]
    record["scalar_A_R_Downhill_Y"][safe] = normal[safe, 1] / horizontal[safe]
    record["scalar_A_R_DownhillMagnitude"] = np.tan(np.radians(slope))

    for name in idw_fields:
        mean_grid = np.divide(attr_sum[name], attr_count,
                              out=np.full(any_ * anx, np.nan, np.float32),
                              where=have_attr).reshape(any_, anx)
        var_grid = np.divide(attr_sq[name], attr_count,
                             out=np.full(any_ * anx, np.nan, np.float32),
                             where=have_attr).reshape(any_, anx)
        var_grid = np.clip(var_grid - np.nan_to_num(mean_grid, nan=0.0) ** 2, 0.0, None)
        mean_field = diffuse(mean_grid)
        sigma_field = np.sqrt(diffuse(var_grid))
        noise_field = noise_a if name in ("red", "green", "blue") else noise_b
        for start in range(0, len(px), 8_000_000):
            stop = min(start + 8_000_000, len(px))
            value = sample_grid(mean_field, px[start:stop], py[start:stop],
                                x0, y0, ATTR_CELL, order=1)
            spread = sample_grid(sigma_field, px[start:stop], py[start:stop],
                                 x0, y0, ATTR_CELL, order=1)
            wobble = sample_grid(noise_field, px[start:stop], py[start:stop],
                                 x0, y0, ATTR_CELL, order=1)
            value = value + noise_amplitude * spread * wobble
            if record.dtype[name].kind == "u":
                record[name][start:stop] = np.clip(value + 0.5, 0, 255).astype(np.uint8)
            else:
                record[name][start:stop] = value.astype(record.dtype[name].base)

    out = data_dir / "Site1-WATER-5mm.ply"
    header = ["ply", "format binary_little_endian 1.0",
              f"comment Site1 water gap fill v5 generated {stamp} (sand-continuous, rock-vetoed)",
              "comment Fill accepted per point with probability 1 - density/target so the",
              "comment combined cloud density stays seam-free against the real points",
              "comment Heights sample the refined cloth bicubically (no grid tiling);",
              "comment steep-mask holes recover only where SAND is XY-continuous and",
              "comment locally height-coherent; high/dominant ROCK and large patches veto fill",
              "comment Recovery heights use a sand-only sheet, preventing cloth drip-cones;",
              "comment attributes use diffusion-blended means plus variance-scaled noise",
              f"comment Final fill surface is capped at z <= {WATER_SUPPORT_Z_CAP} m",
              f"comment ScanID={WATER_SCAN_ID:.0f}; outside sheet capped at z <= {OUTSIDE_Z_CAP} m",
              "comment Generated by scripts/site1_clean_and_cloth_water.py",
              f"element vertex {len(record)}"]
    typemap = {"f4": "float", "f8": "double", "u1": "uchar"}
    for name in record.dtype.names:
        header.append(f"property {typemap[record.dtype[name].str.lstrip('<>|=')]} {name}")
    header.append("end_header")
    temp = out.with_suffix(".ply.tmp")
    with open(temp, "wb") as handle:
        handle.write(("\n".join(header) + "\n").encode("ascii"))
        record.tofile(handle)
    temp.replace(out)
    print(f"[water] wrote {out} ({len(record):,} pts, {out.stat().st_size/1e9:.2f} GB)",
          flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("stage", choices=["classify", "apply", "cloth", "water",
                                          "restore", "all"])
    parser.add_argument("--data-dir", type=Path, default=Path("Data/Scene1"))
    parser.add_argument("--work-dir", type=Path, required=True)
    parser.add_argument("--run-name", default=RUN_NAME,
                        help="PatchRefinement run to apply into or restore from")
    args = parser.parse_args()
    args.work_dir.mkdir(parents=True, exist_ok=True)
    if args.stage == "restore":
        stage_restore(args.data_dir, args.run_name)
        return
    context = Context(args.data_dir, args.work_dir)
    if args.stage in ("classify", "all"):
        stage_classify(context, args.data_dir, args.work_dir)
    if args.stage in ("apply", "all"):
        stage_apply(args.data_dir, args.work_dir, args.run_name)
    if args.stage in ("cloth", "all"):
        stage_cloth(context, args.data_dir)
    if args.stage in ("water", "all"):
        stage_water(context, args.data_dir, args.work_dir)


if __name__ == "__main__":
    main()
