#!/usr/bin/env python3
"""Site1 reflection-noise cleanup and cloth-guided water fill (v2).

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

classify  Region = the ToMesh footprint hull (the central flat; everything
          outside it is untouched). Three per-point noise tests, identical
          for the 5 mm and 1 mm SAND/ROCK clouds:
            manual   farther than 25 mm from any ToMesh point inside the
                     eroded-safe region (recovers the by-hand deselection --
                     membership is verbatim, 99.9% of kept points sit at 0)
            below    more than 3 cm under the cloth AND covered by a real
                     surface above (a surface-band point within 7.5 cm XY);
                     kills the mirrored under-water ghosts while keeping
                     channel floors the stiff cloth bridges (nothing above
                     those)
            above    more than 5 cm over the cloth AND floating: no
                     surface-band support within 7.5 cm XY and no kept point
                     within 6 cm below it in its own column; kills hovering
                     rock "shadow" slabs while keeping grounded sand mounds
          The surface band (|z - cloth| <= 3 cm) is built once from the 5 mm
          SAND+ROCK clouds on a 2.5 cm column grid, so both densities are
          classified against the same physical surface.

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
RUN_NAME = "20260825-noise-cleanup-v1"

CLEAN_TARGETS = [
    ("SAND", "5mm"), ("ROCK", "5mm"), ("SAND", "1mm"), ("ROCK", "1mm"),
]
REASON_NAMES = {1: "manual", 2: "below-ghost", 3: "above-ghost"}


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
        np.savez_compressed(
            work / "context.npz",
            hull=self.hull, safe=self.safe, cloth_filled=self.cloth_filled,
            cloth_defined=self.cloth_defined,
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
                in_band = np.abs(dz) <= SURFACE_BAND
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


# ---- classify -----------------------------------------------------------

def classify_file(context: Context, data_dir: Path, role: str, spacing: str):
    band, band_top, conn_top = context.columns(data_dir)
    cny, cnx = band.shape
    tree = context.tomesh_tree()
    cloud, _, _ = memmap_cloud(cloud_path(data_dir, role, spacing))
    reasons = np.zeros(len(cloud), np.uint8)
    for start in range(0, len(cloud), 8_000_000):
        chunk = cloud[start:start + 8_000_000]
        x = chunk["x"].astype(np.float32); y = chunk["y"].astype(np.float32)
        z = chunk["z"].astype(np.float32)
        gx = ((x - context.x0) / REGION_CELL).astype(np.int64)
        gy = ((y - context.y0) / REGION_CELL).astype(np.int64)
        inside = (gx >= 0) & (gx < context.nx) & (gy >= 0) & (gy < context.ny)
        gxc = np.clip(gx, 0, context.nx - 1); gyc = np.clip(gy, 0, context.ny - 1)
        in_safe = inside & context.safe[gyc, gxc]
        near_cloth = inside & context.cloth_near[gyc, gxc]
        chunk_reason = np.zeros(len(chunk), np.uint8)

        if in_safe.any():
            pts = np.stack([x[in_safe], y[in_safe], z[in_safe]], 1)
            distance, _ = tree.query(pts, k=1, workers=-1)
            manual = np.zeros(len(chunk), bool)
            manual[np.nonzero(in_safe)[0][distance > MANUAL_DISTANCE]] = True
            chunk_reason[manual] = 1

        ghosts_domain = near_cloth & (chunk_reason == 0)
        if ghosts_domain.any():
            dz = z - context.cloth_filled[gyc, gxc]
            cx = np.clip(((x - context.x0) / COLUMN_CELL).astype(np.int64), 0, cnx - 1)
            cy = np.clip(((y - context.y0) / COLUMN_CELL).astype(np.int64), 0, cny - 1)
            has_band = band[cy, cx]
            top_band = band_top[cy, cx]
            # below-ghost: under the cloth AND a real surface continues above
            # it (double layer). A channel floor the cloth bridges has no
            # surface band above and is kept.
            below = ghosts_domain & (dz < BELOW_DZ) & has_band & (top_band > z + 0.04)
            chunk_reason[below] = 2
            # above-ghost: over the cloth AND not reachable from the surface
            # band by a connected column (>= 6 cm air gap below it).
            floating = ghosts_domain & (dz > ABOVE_DZ) & has_band & (
                z > conn_top[cy, cx] + 0.03)
            chunk_reason[floating] = 3
        reasons[start:start + len(chunk)] = chunk_reason
    return reasons


def stage_classify(context: Context, data_dir: Path, work: Path):
    summary = {}
    for role, spacing in CLEAN_TARGETS:
        key = f"{role}-{spacing}"
        out = work / f"reasons-{key}.npy"
        if out.exists():
            reasons = np.load(out)
        else:
            reasons = classify_file(context, data_dir, role, spacing)
            np.save(out, reasons)
        counts = {name: int((reasons == code).sum()) for code, name in REASON_NAMES.items()}
        counts["total_removed"] = int((reasons > 0).sum())
        counts["total_points"] = int(len(reasons))
        summary[key] = counts
        print(f"[classify] {key}: {counts}")
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


def stage_apply(data_dir: Path, work: Path):
    run_dir = data_dir / "PatchRefinement" / RUN_NAME
    run_dir.mkdir(parents=True, exist_ok=True)
    (data_dir / "PatchRefinement" / ".invisible_places-ignore").touch()
    manifest = {"run": RUN_NAME, "created": _dt.datetime.now().isoformat(),
                "parameters": {
                    "manual_distance": MANUAL_DISTANCE, "surface_band": SURFACE_BAND,
                    "below_dz": BELOW_DZ, "above_dz": ABOVE_DZ,
                    "support_gap": SUPPORT_GAP, "cover_radius_cells": COVER_RADIUS_CELLS},
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


def stage_restore(data_dir: Path):
    run_dir = data_dir / "PatchRefinement" / RUN_NAME
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
    filled = context.cloth_filled
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

def stage_water(context: Context, data_dir: Path, work: Path):
    import open3d as o3d
    from scipy import ndimage
    from scipy.spatial import cKDTree

    stamp = _dt.date.today().isoformat()
    x0 = context.x0 - 40.0
    y0 = context.y0 - 45.0
    nx = int(120.0 / DENSITY_CELL)
    ny = int(165.0 / DENSITY_CELL)

    # -- coverage density + measured surface height from the cleaned clouds
    counts = np.zeros((ny, nx), np.float32)
    ground_counts = np.zeros(ny * nx, np.float64)
    zsum = np.zeros(ny * nx, np.float64)
    zcnt = np.zeros(ny * nx, np.int64)
    attr_sources = []
    for role in ("SAND", "ROCK", "VEG"):
        cloud, _, _ = memmap_cloud(cloud_path(data_dir, role, "5mm"))
        for start in range(0, len(cloud), 8_000_000):
            chunk = cloud[start:start + 8_000_000]
            gx = ((chunk["x"] - x0) / DENSITY_CELL).astype(np.int64)
            gy = ((chunk["y"] - y0) / DENSITY_CELL).astype(np.int64)
            inside = (gx >= 0) & (gx < nx) & (gy >= 0) & (gy < ny)
            flat = gy[inside] * nx + gx[inside]
            np.add.at(counts.ravel(), flat, 1.0)
            if role != "VEG":
                np.add.at(zsum, flat, chunk["z"][inside].astype(np.float64))
                np.add.at(zcnt, flat, 1)
        if role != "VEG":
            stride = max(1, len(cloud) // 12_000_000)
            attr_sources.append(cloud[::stride])
    sigma_cells = DENSITY_SIGMA_M / DENSITY_CELL
    density = ndimage.gaussian_filter(counts, sigma_cells) / (DENSITY_CELL ** 2)
    covered = density[density > WELL_COVERED_MIN]
    target = float(np.percentile(covered, TARGET_PERCENTILE))
    print(f"[water] density target = {target:,.0f} pts/m^2 "
          f"(median covered {np.median(covered):,.0f})")
    zmean = np.divide(zsum, zcnt, out=np.full(ny * nx, np.nan), where=zcnt > 0)
    zmean = zmean.reshape(ny, nx).astype(np.float32)
    zmean_s = ndimage.gaussian_filter(np.nan_to_num(zmean, nan=0.0), 2.0)
    zmean_w = ndimage.gaussian_filter(np.isfinite(zmean).astype(np.float32), 2.0)
    zdata = np.divide(zmean_s, zmean_w, out=np.full_like(zmean_s, np.nan), where=zmean_w > 0.05)

    # -- reference surfaces: refined cloth inside the hull, v1 harmonic outside
    region_scale = REGION_CELL / DENSITY_CELL
    def region_lookup(mask):
        gx = ((np.arange(nx) * DENSITY_CELL + x0 - context.x0) / REGION_CELL).astype(np.int64)
        gy = ((np.arange(ny) * DENSITY_CELL + y0 - context.y0) / REGION_CELL).astype(np.int64)
        ok_x = (gx >= 0) & (gx < context.nx)
        ok_y = (gy >= 0) & (gy < context.ny)
        out = np.zeros((ny, nx), bool)
        sub = mask[np.clip(gy, 0, context.ny - 1)[:, None],
                   np.clip(gx, 0, context.nx - 1)[None, :]]
        out[np.ix_(ok_y, ok_x)] = sub[np.ix_(ok_y, ok_x)]
        return out

    in_hull = region_lookup(context.hull)
    cloth_z = np.full((ny, nx), np.nan, np.float32)
    gx = ((np.arange(nx) * DENSITY_CELL + x0 - context.x0) / REGION_CELL).astype(np.int64)
    gy = ((np.arange(ny) * DENSITY_CELL + y0 - context.y0) / REGION_CELL).astype(np.int64)
    okx = (gx >= 0) & (gx < context.nx); oky = (gy >= 0) & (gy < context.ny)
    cloth_z[np.ix_(oky, okx)] = context.cloth_filled[
        np.clip(gy, 0, context.ny - 1)[oky][:, None],
        np.clip(gx, 0, context.nx - 1)[okx][None, :]]

    # outside surface: the Poisson mesh limits the extent (raycast hit +
    # z-cap keeps rock occlusion shadows dry there), but the height is a
    # harmonic sheet solved at 5 cm from the measured surrounding surface --
    # the raw mesh wobbles in unsupported spans (the v1 lesson).
    mesh = o3d.io.read_triangle_mesh(str(data_dir / "Site1-MESH.ply"))
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
    deficit = (density < target * FILL_TRIGGER) & ~in_hull
    # footprint at 10 cm with a 2 m closing (as in v1): the open bay sits
    # tens of metres from the nearest return and only a coarse closing over
    # the sparse surrounding speckle encloses it
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
    outside_z = np.full((ny, nx), np.nan, np.float32)
    if len(ox):
        rays = np.zeros((len(ox), 6), np.float32)
        rays[:, 0] = x0 + (ox + 0.5) * DENSITY_CELL
        rays[:, 1] = y0 + (oy + 0.5) * DENSITY_CELL
        rays[:, 2] = 60.0; rays[:, 5] = -1.0
        hit = scene.cast_rays(o3d.core.Tensor(rays))["t_hit"].numpy()
        zray = 60.0 - hit
        rayok = np.isfinite(hit) & (zray <= OUTSIDE_Z_CAP)
        allowed = np.zeros((ny, nx), bool)
        allowed[oy[rayok], ox[rayok]] = True
        # coarse (5 cm) harmonic solve with measured-surface rims
        factor = 5
        cny, cnx = ny // factor, nx // factor
        coarse_mask = allowed[:cny * factor, :cnx * factor].reshape(
            cny, factor, cnx, factor).any(axis=(1, 3))
        zdata_coarse = np.nanmean(
            np.where(np.isfinite(zdata[:cny * factor, :cnx * factor]),
                     zdata[:cny * factor, :cnx * factor], np.nan).reshape(
                cny, factor, cnx, factor), axis=(1, 3))
        rim = ndimage.binary_dilation(coarse_mask, iterations=2) & ~coarse_mask
        seed = np.where(rim & np.isfinite(zdata_coarse), zdata_coarse, np.nan)
        solved = context._harmonic_fill(seed, coarse_mask | rim)
        fine = np.kron(solved, np.ones((factor, factor), np.float32))
        fine = ndimage.gaussian_filter(fine, 4.0)
        outside_z[:cny * factor, :cnx * factor] = fine
        outside_z = np.where(allowed, np.minimum(outside_z, OUTSIDE_Z_CAP), np.nan)

    reference = np.where(in_hull, cloth_z, outside_z)

    # -- candidates: jittered 5 mm grid where the deficit says so
    rng = np.random.default_rng(41)
    fill_region = np.isfinite(reference) & (density < target * FILL_TRIGGER) & (
        in_hull | outside_mask)
    fy, fx = np.nonzero(fill_region)
    print(f"[water] fill region: {len(fx):,} cells ({len(fx)*DENSITY_CELL**2:.0f} m^2)")
    per_cell = 4  # 1 cm cell -> 4 candidates = 5 mm pitch
    offsets = np.array([[0.25, 0.25], [0.75, 0.25], [0.25, 0.75], [0.75, 0.75]], np.float32)
    px = (x0 + (fx[:, None] + offsets[None, :, 0]) * DENSITY_CELL).ravel()
    py = (y0 + (fy[:, None] + offsets[None, :, 1]) * DENSITY_CELL).ravel()
    px += rng.uniform(-0.0015, 0.0015, len(px)).astype(np.float32)
    py += rng.uniform(-0.0015, 0.0015, len(py)).astype(np.float32)
    cell_density = np.repeat(density[fy, fx], per_cell)
    accept = rng.random(len(px)) < np.clip(1.0 - cell_density / target, 0.0, 1.0)
    px, py = px[accept].astype(np.float32), py[accept].astype(np.float32)
    print(f"[water] accepted {len(px):,} candidates from the density deficit")

    # keep-clear against real points where any coverage exists
    gxc = np.clip(((px - x0) / DENSITY_CELL).astype(np.int64), 0, nx - 1)
    gyc = np.clip(((py - y0) / DENSITY_CELL).astype(np.int64), 0, ny - 1)
    near_data = density[gyc, gxc] > target * 0.02
    if near_data.any():
        neighbour_xy = []
        for source in attr_sources:
            sx = ((source["x"] - x0) / DENSITY_CELL).astype(np.int64)
            sy = ((source["y"] - y0) / DENSITY_CELL).astype(np.int64)
            ok = (sx >= 0) & (sx < nx) & (sy >= 0) & (sy < ny)
            ok[ok] = density[sy[ok], sx[ok]] > target * 0.02
            neighbour_xy.append(np.stack([source["x"][ok], source["y"][ok]], 1).astype(np.float32))
        clear_tree = cKDTree(np.vstack(neighbour_xy))
        distance, _ = clear_tree.query(np.stack([px[near_data], py[near_data]], 1),
                                       k=1, workers=-1, distance_upper_bound=KEEP_CLEAR)
        collide = np.zeros(len(px), bool)
        collide[np.nonzero(near_data)[0][np.isfinite(distance)]] = True
        px, py = px[~collide], py[~collide]
        gxc, gyc = gxc[~collide], gyc[~collide]
        print(f"[water] keep-clear dropped {collide.sum():,}")

    # Trust the measured local surface wherever real points exist (full trust
    # from half the target density up), and only let the reference sheet rule
    # where the ground is truly empty. The clamp brackets the blend partners,
    # not the reference alone -- otherwise fill on sparse sloped sand hovers
    # above the real slope (the bay's west edge in v2.0).
    z_ref = reference[gyc, gxc]
    z_dat = zdata[gyc, gxc]
    has_data = np.isfinite(z_dat)
    weight = np.clip(density[gyc, gxc] / (0.5 * target), 0.0, 1.0)
    weight[~has_data] = 0.0
    pz = ((1.0 - weight) * z_ref + weight * np.nan_to_num(z_dat, nan=0.0)).astype(np.float32)
    low = np.where(has_data, np.minimum(z_ref, np.nan_to_num(z_dat, nan=np.inf)), z_ref) - 0.08
    high = np.where(has_data, np.maximum(z_ref, np.nan_to_num(z_dat, nan=-np.inf)), z_ref) + 0.08
    pz = np.clip(pz, low, high)
    pz += rng.normal(0.0, 0.0005, len(pz)).astype(np.float32)

    # normals + slope from the blended reference surface
    blend_surface = np.where(np.isfinite(zdata) & (density > target),
                             zdata, reference)
    blend_surface = ndimage.gaussian_filter(
        np.nan_to_num(blend_surface, nan=np.nanmedian(reference)), 3.0)
    gsy, gsx = np.gradient(blend_surface, DENSITY_CELL)
    nvec = np.stack([-gsx[gyc, gxc], -gsy[gyc, gxc], np.ones(len(px), np.float32)], 1)
    nvec /= np.linalg.norm(nvec, axis=1, keepdims=True)
    slope = np.degrees(np.arccos(np.clip(nvec[:, 2], -1.0, 1.0))).astype(np.float32)

    # attributes via 2D IDW from cleaned SAND+ROCK
    dtype, _, _, _ = read_header(cloud_path(data_dir, "SAND", "5mm"))
    record = np.zeros(len(px), dtype)
    record["x"], record["y"], record["z"] = px, py, pz
    record["nx"], record["ny"], record["nz"] = nvec.T
    record["scalar_ScanID"] = WATER_SCAN_ID
    record["scalar_A_R_Horizontalness"] = np.cos(np.radians(slope))
    record["scalar_A_R_Slope_deg"] = slope
    downhill = np.stack([gsx[gyc, gxc], gsy[gyc, gxc], np.zeros(len(px), np.float32)], 1)
    magnitude = np.linalg.norm(downhill[:, :2], axis=1)
    safe = magnitude > 1e-5
    record["scalar_A_R_Downhill_X"][safe] = (-downhill[safe, 0] / magnitude[safe])
    record["scalar_A_R_Downhill_Y"][safe] = (-downhill[safe, 1] / magnitude[safe])
    record["scalar_A_R_DownhillMagnitude"] = np.tan(np.radians(slope))

    idw_fields = ["red", "green", "blue", "scalar_Intensity", "scalar_Composite",
                  "scalar_A_R_Shelter_Lower", "scalar_A_R_RainExposure_Lower",
                  "scalar_A_R_SVF_Lower"]
    source_xy = np.vstack([
        np.stack([s["x"], s["y"]], 1).astype(np.float32) for s in attr_sources])
    stacked = {name: np.concatenate([np.asarray(s[name], dtype=np.float64)
                                     for s in attr_sources])
               for name in idw_fields}
    attr_tree = cKDTree(source_xy)
    for start in range(0, len(px), 4_000_000):
        stop = min(start + 4_000_000, len(px))
        distance, neighbour = attr_tree.query(
            np.stack([px[start:stop], py[start:stop]], 1), k=6, workers=-1)
        idw = 1.0 / np.maximum(distance, 0.01) ** 2
        idw /= idw.sum(axis=1, keepdims=True)
        for name in idw_fields:
            value = (stacked[name][neighbour] * idw).sum(axis=1)
            if record.dtype[name].kind == "u":
                record[name][start:stop] = np.clip(value + 0.5, 0, 255).astype(np.uint8)
            else:
                record[name][start:stop] = value.astype(record.dtype[name].base)

    out = data_dir / "Site1-WATER-5mm.ply"
    header = ["ply", "format binary_little_endian 1.0",
              f"comment Site1 water gap fill v2 generated {stamp} (cloth-guided, density-continuous)",
              "comment Fill accepted per point with probability 1 - density/target so the",
              "comment combined cloud density stays seam-free against the real points",
              f"comment ScanID={WATER_SCAN_ID:.0f}; heights ride the refined CSF cloth inside the",
              f"comment central flat and the harmonic sheet outside (z <= {OUTSIDE_Z_CAP} m there)",
              "comment Generated by scripts/site1_clean_and_cloth_water.py",
              f"element vertex {len(record)}"]
    typemap = {"f4": "float", "f8": "double", "u1": "uchar"}
    for name in record.dtype.names:
        header.append(f"property {typemap[record.dtype[name].str.lstrip('<>|=')]} {name}")
    header.append("end_header")
    with open(out, "wb") as handle:
        handle.write(("\n".join(header) + "\n").encode("ascii"))
        record.tofile(handle)
    print(f"[water] wrote {out} ({len(record):,} pts, {out.stat().st_size/1e9:.2f} GB)")


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("stage", choices=["classify", "apply", "cloth", "water",
                                          "restore", "all"])
    parser.add_argument("--data-dir", type=Path, default=Path("Data/Scene1"))
    parser.add_argument("--work-dir", type=Path, required=True)
    args = parser.parse_args()
    args.work_dir.mkdir(parents=True, exist_ok=True)
    if args.stage == "restore":
        stage_restore(args.data_dir)
        return
    context = Context(args.data_dir, args.work_dir)
    if args.stage in ("classify", "all"):
        stage_classify(context, args.data_dir, args.work_dir)
    if args.stage in ("apply", "all"):
        stage_apply(args.data_dir, args.work_dir)
    if args.stage in ("cloth", "all"):
        stage_cloth(context, args.data_dir)
    if args.stage in ("water", "all"):
        stage_water(context, args.data_dir, args.work_dir)


if __name__ == "__main__":
    main()
