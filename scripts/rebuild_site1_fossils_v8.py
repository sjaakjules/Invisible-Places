#!/usr/bin/env python3
"""Rebuild Scene1/Fossils WATER as a hydraulically levelled, density-blended
sheet (v8), and tidy the v7 terrain additions.

Where the earlier generations went wrong and what v8 does instead:

* v6 levelled per fragment and cut axis-aligned separator trenches; v7 kept
  a 25 mm cell-boolean footprint (a couple of SAND points veto a whole
  cell), fitted one plane per connected fragment (5,840 fragments -> 55 mm
  median terraces between neighbours) and filled quota lattices with
  +-0.4 mm jitter (rectilinear edges).  Neither generation could create
  water outside the inherited footprint or the drawn annotations, so the
  scanner-starburst pool and other evidence-obvious cavities stayed dry.

* v8 builds the footprint from evidence (previous water of BOTH prior
  generations, minus the user's latest pink boundary, plus density-deficit
  cavities at plausible water level, guarded by every pink generation),
  closes it into physical pool bodies, and runs a hydraulic fixed point:
  dams (terrain above the local level, and unscanned strips) split zones,
  open water equalises, every zone caps at the spill height of its rim,
  water below a well-measured floor dries into a waterline, sparse floors
  keep a thin film so no black holes reopen.  Points are then generated at
  fine pitch with position-hash determinism, carved with a feathered
  standoff around emergent terrain only (submerged sand never vetoes), and
  accepted against the measured neighbourhood density so seams blend.  The
  5 mm layer is a hash-derived subset of the fine layer, so the two are
  co-registered exactly like the SAND/ROCK 1 mm / 5 mm bundles.

Stages: build -> verify -> install (reversible via restore).  install also
micro-culls the detached tufts of the v7 ScanID-9 terrain additions (their
append-only layout keeps the truncation rollback valid) and relocates the
old water backups out of the discovery path (they match the app's water
token and were triple-stacking in the live view).

Requires numpy and scipy.  Uses rebuild_site1_fossils_water (v6) for the
streaming grid accumulator, ripple grid, and record helpers.
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

ROOT = SCRIPT_DIR.parent
DEFAULT_DATA = ROOT / "Data/Scene1"
DEFAULT_RUN = DEFAULT_DATA / "PatchRefinement/20260826-water-v8-hydraulic"
V7_RUN = DEFAULT_DATA / "PatchRefinement/20260826-fossils-v7-mixed"
CONFIG_V7 = SCRIPT_DIR / "config/site1_fossils_v7_regions.json"
CONFIG_LEGACY = SCRIPT_DIR / "config/site1_fossils_reconstruction_regions.json"

WATER_SCAN_ID = 999.0
TERRAIN_SCAN_ID = 9.0
CHUNK = 2_000_000

# ---- geometry (metres everywhere; review H3) -----------------------------
CELL = 0.05                     # leveller grid
DENS_CELL = 0.01                # density field
HULL_CLOSE_M = 2.0
FOOT_CLOSE_M = 0.25
CAVITY_OPEN_M = 0.10
CAVITY_MAX_Z = 2.6
CAVITY_DENSITY_FRAC = 0.40
BODY_MIN_M2 = 0.02
ANCHOR_FLAT_SPREAD = 0.06
ANCHOR_RING_M = 0.30
LEVEL_SIGMA_ANCHOR_M = 1.50
LEVEL_SIGMA_OLD_M = 1.00
EV_SMOOTH_M = 0.40
ZONE_DAM_CLEAR = 0.01
SPILL_RIM_Q = 0.15
MERGE_PULL_CAP = 0.10
RESIDUAL_SIGMA_M = 0.60
RESIDUAL_CLAMP = 0.02
POOL_SMOOTH_M = 0.15
POOL_SMOOTH_CLAMP = 0.015
FINAL_SMOOTH_M = 0.075
FINAL_SMOOTH_CLAMP = 0.004
ADOPT_NEAR_M = 0.5
ADOPT_RIM_RING_M = 0.75
FILM_MIN_COUNT = 5
FILM_LIFT = 0.003
FILM_MAX_CLIMB = 0.25
FILM_FLOOR_SMOOTH_M = 0.15
STEP_OPEN = 0.010
STEP_SEAM = 0.03
SPILL_SWEEPS = 40

# ---- sampling ------------------------------------------------------------
PITCH_FINE = 0.0022             # ~207k/m^2 candidate lattice
TILE_M = 8.0
STAND_IN, STAND_OUT = 0.018, 0.040
COLLAR_LO, COLLAR_HI = -0.005, 0.5
DENS_SMOOTH_M = 0.025
TARGET_SMOOTH_M = 0.15
SEAM_BAND_M = 0.30              # complement-of-terrain applies here only
INTERIOR_T_FINE = 190_000.0     # measured 1 mm-class seam median
INTERIOR_T_5MM = 34_000.0       # between seam median 29.6k and v7's 40k
T_FINE_CLAMP = (76_000.0, 340_000.0)
T_5MM_CLAMP = (22_000.0, 40_000.0)
EDGE_FEATHER_LO, EDGE_FEATHER_HI = 0.35, 0.75
SEAM_FEATHER_M = 0.10
PUDDLE_MIN_M2 = 0.02
Z_NOISE = 0.0008

# ---- terrain micro-cull (v7 ScanID 9 additions) --------------------------
CULL_DZ = 0.02
CULL_NN = 0.020
RIM_SKIRT_CENTRE = (764.53, 812.09, 3.28)
RIM_SKIRT_R = 0.6
RIM_SKIRT_NN = 0.015

GEOMETRY_SIGMAS = {"Fine": 0.75, "Medium": 2.0, "Broad": 6.0}


def hash01(ix: np.ndarray, iy: np.ndarray, salt: int) -> np.ndarray:
    """Deterministic position-hash uniform in [0,1) (review H4: tiling- and
    data-independent randomness, reproducible per lattice cell)."""
    h = (ix.astype(np.uint64) * np.uint64(0x9E3779B97F4A7C15) ^
         iy.astype(np.uint64) * np.uint64(0xC2B2AE3D27D4EB4F) ^
         np.uint64(salt) * np.uint64(0xD6E8FEB86659FD93))
    h ^= h >> np.uint64(33)
    h *= np.uint64(0xFF51AFD7ED558CCD)
    h ^= h >> np.uint64(33)
    return (h >> np.uint64(11)).astype(np.float64) / float(1 << 53)


def disk(radius_cells: int) -> np.ndarray:
    span = np.mgrid[-radius_cells:radius_cells + 1, -radius_cells:radius_cells + 1]
    return np.hypot(span[0], span[1]) <= radius_cells


def cells(metres: float, cell: float = CELL) -> int:
    return max(1, int(round(metres / cell)))


def fill_nan_nearest(grid: np.ndarray) -> np.ndarray:
    from scipy import ndimage
    mask = ~np.isfinite(grid)
    if not mask.any():
        return grid
    idx = ndimage.distance_transform_edt(mask, return_distances=False,
                                         return_indices=True)
    return grid[tuple(idx)]


def rasterize_polys(polys, ny, nx, x0, y0, cell) -> np.ndarray:
    from matplotlib.path import Path as MplPath
    yy, xx = np.mgrid[0:ny, 0:nx]
    pts = np.column_stack([(xx.ravel() + 0.5) * cell + x0,
                           (yy.ravel() + 0.5) * cell + y0])
    mask = np.zeros(ny * nx, bool)
    for poly in polys:
        arr = np.asarray(poly, np.float64)
        if arr.ndim == 2 and len(arr) >= 3:
            mask |= MplPath(arr).contains_points(pts)
    return mask.reshape(ny, nx)


def load_pink_masks(ny, nx, x0, y0):
    cfg7 = json.loads(CONFIG_V7.read_text())
    pink7 = rasterize_polys(cfg7["annotations"]["pink_water_exclusion"],
                            ny, nx, x0, y0, CELL)
    legacy = []
    if CONFIG_LEGACY.exists():
        raw = json.loads(CONFIG_LEGACY.read_text()).get(
            "water_exclusion_boundary", [])
        if raw and isinstance(raw[0], (list, tuple)) and raw[0] and \
                isinstance(raw[0][0], (int, float)):
            raw = [raw]        # a single polygon arrives as bare pairs
        legacy = raw
    pink_legacy = rasterize_polys(legacy, ny, nx, x0, y0, CELL) if legacy \
        else np.zeros((ny, nx), bool)
    return pink7, pink7 | pink_legacy


# ==========================================================================
# grids
# ==========================================================================

def stream_grids(data_dir: Path, log):
    """5 cm terrain/prior-water grids ([y, x] layout) + terrain extent."""
    paths = {
        "sand": data_dir / "Site1-SAND-5mm.ply",
        "rock": data_dir / "Site1-ROCK-5mm.ply",
        "water": data_dir / "Site1-WATER-5mm.ply",          # v7 installed
        "waterold": data_dir / "Site1-WATER-5mm-old02.ply",  # v6b
    }
    mins = np.array([np.inf, np.inf])
    maxs = np.array([-np.inf, -np.inf])
    for key in ("sand", "rock"):
        mm = v6.memmap_cloud(paths[key])[0] if isinstance(
            v6.memmap_cloud(paths[key]), tuple) else v6.memmap_cloud(paths[key])
        for i in range(0, len(mm), CHUNK * 4):
            c = mm[i:i + CHUNK * 4]
            mins = np.minimum(mins, [c["x"].min(), c["y"].min()])
            maxs = np.maximum(maxs, [c["x"].max(), c["y"].max()])
    x0 = math.floor((mins[0] - 0.5) / CELL) * CELL
    y0 = math.floor((mins[1] - 0.5) / CELL) * CELL
    nx = int(math.ceil((maxs[0] + 0.5 - x0) / CELL)) + 1
    ny = int(math.ceil((maxs[1] + 0.5 - y0) / CELL)) + 1
    log(f"grid {ny} x {nx} @ {CELL} m, origin ({x0:.2f}, {y0:.2f})")

    out = {"x0": x0, "y0": y0, "nx": nx, "ny": ny}
    for key, path in paths.items():
        cnt = np.zeros(ny * nx, np.int64)
        zsum = np.zeros(ny * nx, np.float64)
        zmin = np.full(ny * nx, np.inf, np.float32)
        zmax = np.full(ny * nx, -np.inf, np.float32)
        mm = v6.memmap_cloud(path)
        mm = mm[0] if isinstance(mm, tuple) else mm
        for i in range(0, len(mm), CHUNK * 4):
            c = mm[i:i + CHUNK * 4]
            gx = ((c["x"] - x0) / CELL).astype(np.int64)
            gy = ((c["y"] - y0) / CELL).astype(np.int64)
            ok = (gx >= 0) & (gx < nx) & (gy >= 0) & (gy < ny)
            k = gy[ok] * nx + gx[ok]
            z = c["z"][ok].astype(np.float64)
            cnt += np.bincount(k, minlength=ny * nx)
            zsum += np.bincount(k, weights=z, minlength=ny * nx)
            np.minimum.at(zmin, k, z.astype(np.float32))
            np.maximum.at(zmax, k, z.astype(np.float32))
        out[key + "_cnt"] = cnt.reshape(ny, nx)
        out[key + "_zsum"] = zsum.reshape(ny, nx)
        out[key + "_zmin"] = zmin.reshape(ny, nx)
        out[key + "_zmax"] = zmax.reshape(ny, nx)
        log(f"  {key}: {int(cnt.sum()):,} points, "
            f"{int((cnt > 0).sum()):,} occupied cells")
    return out


# ==========================================================================
# hydraulic leveller (validated as a 5 cm prototype; review fixes H1, M2,
# M5, M6, L1, L7, L9 folded in)
# ==========================================================================

def build_water_surface(G: dict, log):
    from scipy import ndimage
    ny, nx = G["ny"], G["nx"]
    x0, y0 = G["x0"], G["y0"]

    terr_cnt = G["sand_cnt"] + G["rock_cnt"]
    terr_zsum = G["sand_zsum"] + G["rock_zsum"]
    terr_zmean = np.divide(terr_zsum, terr_cnt,
                           out=np.full((ny, nx), np.nan), where=terr_cnt > 0)
    terr_zmin = np.where(G["sand_cnt"] > 0, G["sand_zmin"], np.inf)
    terr_zmin = np.minimum(terr_zmin,
                           np.where(G["rock_cnt"] > 0, G["rock_zmin"], np.inf))
    terr_zmax = np.where(G["sand_cnt"] > 0, G["sand_zmax"], -np.inf)
    terr_zmax = np.maximum(terr_zmax,
                           np.where(G["rock_cnt"] > 0, G["rock_zmax"], -np.inf))
    terr_top = np.where(np.isfinite(terr_zmax) & (terr_cnt > 0),
                        terr_zmax, -np.inf)
    wat7 = G["water_cnt"] > 0
    watO = G["waterold_cnt"] > 0
    wz7 = np.divide(G["water_zsum"], G["water_cnt"],
                    out=np.full((ny, nx), np.nan), where=G["water_cnt"] > 0)
    wzO = np.divide(G["waterold_zsum"], G["waterold_cnt"],
                    out=np.full((ny, nx), np.nan), where=G["waterold_cnt"] > 0)

    pink7, pink_all = load_pink_masks(ny, nx, x0, y0)
    log(f"pink: latest {int(pink7.sum()):,} cells, "
        f"cavity-guard {int(pink_all.sum()):,}")

    # footprint: previous water (minus the latest pink) plus evidence cavities
    med_terr = np.median(terr_cnt[terr_cnt > 0])
    hull = ndimage.binary_fill_holes(
        ndimage.binary_closing(terr_cnt > 0, structure=disk(cells(HULL_CLOSE_M))))
    ctx_med = ndimage.median_filter(
        np.nan_to_num(terr_zmean, nan=99.0), size=2 * cells(0.5) + 1)
    cavity = (hull & ~wat7 & ~watO &
              (terr_cnt < CAVITY_DENSITY_FRAC * med_terr) &
              (ctx_med <= CAVITY_MAX_Z))
    cavity = ndimage.binary_opening(cavity & ~pink_all,
                                    structure=disk(cells(CAVITY_OPEN_M)))
    foot = (wat7 | watO | cavity) & ~pink7
    foot_closed = ndimage.binary_closing(
        foot, structure=disk(cells(FOOT_CLOSE_M))) & ~pink7 & hull
    labels, _ = ndimage.label(foot_closed, structure=np.ones((3, 3), bool))
    sizes = np.bincount(labels.ravel())
    keep_body = sizes >= max(1, int(BODY_MIN_M2 / (CELL * CELL)))
    keep_body[0] = False
    log(f"footprint {int(foot.sum()):,} -> closed {int(foot_closed.sum()):,}; "
        f"bodies {int(keep_body.sum())}")

    # level evidence: waterline terrain anchors + old02 pool medians
    near_flat = (terr_cnt >= 3) & ((terr_zmax - terr_zmin) < ANCHOR_FLAT_SPREAD)
    body_dil = ndimage.binary_dilation(foot_closed,
                                       structure=disk(cells(ANCHOR_RING_M)))
    anchors = near_flat & body_dil & ~foot_closed
    anchor_z = np.where(anchors, terr_zmean, np.nan)
    old_ev = np.where(watO & foot_closed, wzO, np.nan)

    def local_quantile(valgrid, mask, sigma_cells, q):
        vals = valgrid[mask]
        if not len(vals):
            return np.full_like(valgrid, np.nan, dtype=np.float64)
        qs = np.nanquantile(vals, [0.02, 0.98])
        bins = np.linspace(qs[0], qs[1], 56)
        w = ndimage.gaussian_filter(mask.astype(np.float32), sigma_cells)
        below = None
        idx = np.full(valgrid.shape, len(bins) - 1, np.int32)
        found = np.zeros(valgrid.shape, bool)
        for i, b in enumerate(bins):
            ind = (mask & (valgrid <= b)).astype(np.float32)
            frac = np.divide(ndimage.gaussian_filter(ind, sigma_cells),
                             np.maximum(w, 1e-9))
            hit = ~found & (frac >= q)
            idx[hit] = i
            found |= hit
        out = bins[idx]
        out[w < 1e-4] = np.nan
        return out

    L_anchor = local_quantile(np.nan_to_num(anchor_z, nan=0.0), anchors,
                              cells(LEVEL_SIGMA_ANCHOR_M), 0.30)
    L_old = local_quantile(np.nan_to_num(old_ev, nan=0.0),
                           np.isfinite(old_ev), cells(LEVEL_SIGMA_OLD_M), 0.50)
    w_anchor = ndimage.gaussian_filter(anchors.astype(np.float32),
                                       cells(LEVEL_SIGMA_ANCHOR_M))
    w_old = ndimage.gaussian_filter(
        np.isfinite(old_ev).astype(np.float32), cells(LEVEL_SIGMA_OLD_M)) * 3.0
    w_anchor = np.where(np.isfinite(L_anchor), w_anchor, 0.0)
    w_old = np.where(np.isfinite(L_old), w_old, 0.0)
    with np.errstate(invalid="ignore"):
        L_raw = np.where(
            (w_anchor + w_old) > 1e-4,
            (np.nan_to_num(L_anchor) * w_anchor + np.nan_to_num(L_old) * w_old)
            / np.maximum(w_anchor + w_old, 1e-9), np.nan)
    log("level evidence built")

    # hydraulic fixed point per body
    surface = np.full((ny, nx), np.nan, np.float32)
    zone_id = np.zeros((ny, nx), np.int32)
    zid = 0
    for b in np.nonzero(keep_body)[0]:
        body = labels == b
        Lb = np.where(body, L_raw, np.nan)
        if not np.isfinite(Lb).any():
            fallback = wzO[body]
            surface[body] = (np.nanmedian(fallback)
                             if np.isfinite(fallback).any() else 2.2)
            continue
        sig = cells(EV_SMOOTH_M)
        Ls = ndimage.gaussian_filter(np.nan_to_num(Lb, nan=0.0), sig)
        Lw = ndimage.gaussian_filter(
            (body & np.isfinite(Lb)).astype(np.float32), sig)
        Ls = np.where(body, Ls / np.maximum(Lw, 1e-9), np.nan)
        level = Ls.copy()
        zones = np.full((ny, nx), -1, np.int64)
        for _ in range(4):
            damcell = body & (terr_top > level + ZONE_DAM_CLEAR)
            open_water = body & ~damcell
            lab2, nz = ndimage.label(open_water, structure=np.ones((3, 3), bool))
            if nz == 0:
                break
            raw_ev = np.where(open_water & np.isfinite(wzO), wzO, np.nan)
            ev = np.where(np.isfinite(raw_ev), raw_ev,
                          np.where(open_water & np.isfinite(Ls), Ls, np.nan))
            idx = lab2[open_water]
            evv = ev[open_water]
            order = np.argsort(idx, kind="stable")
            idx_s, evv_s = idx[order], evv[order]
            starts = np.r_[0, np.flatnonzero(np.diff(idx_s)) + 1]
            stops = np.r_[starts[1:], len(idx_s)]
            lvl_of = np.full(nz + 1, np.nan)
            for a, o in zip(starts, stops):
                seg = evv_s[a:o]
                fin = seg[np.isfinite(seg)]
                if len(fin):
                    lvl_of[int(idx_s[a])] = float(np.median(fin))
            # spill cap: q15 of the emergent rim attributed by nearest zone
            rim_cells = ((body & ~open_water) |
                         (ndimage.binary_dilation(body, structure=disk(2)) & ~body))
            rim_cells &= np.isfinite(terr_zmax) & (terr_cnt > 0)
            if rim_cells.any() and open_water.any():
                _, zn = ndimage.distance_transform_edt(
                    ~open_water, return_indices=True)
                rim_zone = lab2[tuple(zn)][rim_cells]
                rim_h = terr_zmax[rim_cells]
                order2 = np.argsort(rim_zone, kind="stable")
                rz, rh = rim_zone[order2], rim_h[order2]
                st3 = np.r_[0, np.flatnonzero(np.diff(rz)) + 1]
                en3 = np.r_[st3[1:], len(rz)]
                for a3, b3 in zip(st3, en3):
                    zid3 = int(rz[a3])
                    if zid3 <= 0 or not np.isfinite(lvl_of[zid3]):
                        continue
                    seg = rh[a3:b3]
                    seg = seg[np.isfinite(seg)]
                    if len(seg) >= 6:
                        spill = float(np.quantile(seg, SPILL_RIM_Q)) + 0.01
                        if lvl_of[zid3] > spill:
                            lvl_of[zid3] = spill
            assigned = lvl_of[lab2]
            have = open_water & np.isfinite(assigned)
            if have.any():
                _, near = ndimage.distance_transform_edt(
                    ~have, return_indices=True)
                filled = assigned[tuple(near)]
                newlevel = np.where(body, filled, np.nan).astype(np.float32)
            else:
                newlevel = level
            if np.allclose(np.nan_to_num(newlevel[body]),
                           np.nan_to_num(level[body]), atol=0.002):
                level = newlevel
                zones = np.where(open_water, lab2, -1)
                break
            level = newlevel
            zones = np.where(open_water, lab2, -1)
        surface[body] = level[body]
        zvals = zones[body]
        uniq = np.unique(zvals[zvals >= 0])
        if len(zvals):
            remap = np.zeros(int(zvals.max()) + 2, np.int32)
            for i, u in enumerate(uniq):
                remap[u] = zid + i + 1
            inbody = body & (zones >= 0)
            zone_id[inbody] = remap[zones[inbody]]
        zid += len(uniq)
    log(f"bodies levelled; zones {zid}")

    # clamped residual detail from evidence
    resid_src = np.clip(np.where(np.isfinite(old_ev), old_ev - surface, np.nan),
                        -0.05, 0.05)
    rnum = ndimage.gaussian_filter(np.nan_to_num(resid_src, nan=0.0),
                                   cells(RESIDUAL_SIGMA_M))
    rden = ndimage.gaussian_filter(
        np.isfinite(resid_src).astype(np.float32), cells(RESIDUAL_SIGMA_M))
    resid = np.where(rden > 1e-4, rnum / np.maximum(rden, 1e-9), 0.0)
    surface = surface + np.clip(resid, -RESIDUAL_CLAMP,
                                RESIDUAL_CLAMP).astype(np.float32)
    surface[~foot_closed] = np.nan

    def spill_pull(surface, band=None, thresh=STEP_OPEN):
        """Pull-to-lower across open pairs (min-composed; review H1)."""
        unknown = terr_cnt == 0
        any_change = False
        for _ in range(SPILL_SWEEPS):
            changed = False
            for axis in (0, 1):
                aS = surface if axis == 0 else surface.T
                aT = terr_top if axis == 0 else terr_top.T
                aU = unknown if axis == 0 else unknown.T
                both = np.isfinite(aS[1:]) & np.isfinite(aS[:-1])
                if band is not None:
                    aB = band if axis == 0 else band.T
                    both &= aB[1:] | aB[:-1]
                dz = np.abs(aS[1:] - aS[:-1])
                dam = (np.maximum(np.nan_to_num(aT[1:], neginf=-9),
                                  np.nan_to_num(aT[:-1], neginf=-9))
                       > np.minimum(aS[1:], aS[:-1]) + ZONE_DAM_CLEAR)
                dam |= aU[1:] & aU[:-1]
                bad = both & ~dam & (dz > thresh)
                if not bad.any():
                    continue
                lo = np.minimum(aS[1:], aS[:-1])
                a1 = aS[1:].copy(); a0 = aS[:-1].copy()
                a1[bad] = lo[bad]; a0[bad] = lo[bad]
                before = np.nansum(aS)
                aS[1:] = np.minimum(aS[1:], a1)
                aS[:-1] = np.minimum(aS[:-1], a0)
                if np.nansum(aS) != before:
                    changed = True
                surface = aS.T if axis == 1 else aS
            # dry well-measured floors; film over sparse floors
            floor = np.where(terr_cnt > 0, terr_zmin, -np.inf)
            below_floor = np.isfinite(surface) & (surface < floor - 0.005)
            drowned = below_floor & (terr_cnt >= FILM_MIN_COUNT)
            film = below_floor & (terr_cnt < FILM_MIN_COUNT)
            if drowned.any():
                surface[drowned] = np.nan
                changed = True
            if film.any():
                fsm = ndimage.gaussian_filter(
                    np.where(terr_cnt > 0,
                             np.nan_to_num(terr_zmin, posinf=0.0), 0.0),
                    cells(FILM_FLOOR_SMOOTH_M))
                fsw = ndimage.gaussian_filter(
                    (terr_cnt > 0).astype(np.float32),
                    cells(FILM_FLOOR_SMOOTH_M))
                floor_smooth = np.where(fsw > 1e-4,
                                        fsm / np.maximum(fsw, 1e-9), np.nan)
                target_film = floor_smooth + FILM_LIFT
                too_high = film & np.isfinite(floor_smooth) & \
                    (target_film > surface + FILM_MAX_CLIMB)
                surface[too_high] = np.nan
                lift = film & np.isfinite(floor_smooth) & ~too_high
                surface[lift] = np.maximum(surface[lift], target_film[lift])
                changed = True
            any_change |= changed
            if not changed:
                break
        return surface, any_change

    # whole-zone no-dam merge (union-find, spill physics, pull cap M2)
    parent = np.arange(zid + 1)

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    zone_level = np.full(zid + 1, np.nan)
    zm = zone_id > 0
    flatz, flats = zone_id[zm], surface[zm]
    order = np.argsort(flatz, kind="stable")
    fz, fs = flatz[order], flats[order]
    starts = np.r_[0, np.flatnonzero(np.diff(fz)) + 1]
    stops = np.r_[starts[1:], len(fz)]
    for a, b in zip(starts, stops):
        fin = fs[a:b][np.isfinite(fs[a:b])]
        if len(fin):
            zone_level[fz[a]] = float(np.median(fin))
    unknown = terr_cnt == 0
    for _ in range(4):
        merged = False
        for axis in (0, 1):
            aS = surface if axis == 0 else surface.T
            aZ = zone_id if axis == 0 else zone_id.T
            aT = terr_top if axis == 0 else terr_top.T
            aU = unknown if axis == 0 else unknown.T
            both = (np.isfinite(aS[1:]) & np.isfinite(aS[:-1]) &
                    (aZ[1:] > 0) & (aZ[:-1] > 0))
            dz = np.abs(aS[1:] - aS[:-1])
            dam = (np.maximum(np.nan_to_num(aT[1:], neginf=-9),
                              np.nan_to_num(aT[:-1], neginf=-9))
                   > np.minimum(aS[1:], aS[:-1]) + ZONE_DAM_CLEAR)
            dam |= aU[1:] & aU[:-1]
            bad = both & ~dam & (dz > 0.012)
            for u, v in zip(aZ[1:][bad].ravel(), aZ[:-1][bad].ravel()):
                ru, rv = find(u), find(v)
                if ru != rv:
                    parent[max(ru, rv)] = min(ru, rv)
                    merged = True
        if not merged:
            break
        roots = np.array([find(z) for z in range(zid + 1)])
        group_min = np.full(zid + 1, np.inf)
        for z in range(1, zid + 1):
            if np.isfinite(zone_level[z]):
                group_min[roots[z]] = min(group_min[roots[z]], zone_level[z])
        pulled = np.maximum(group_min[roots],
                            np.nan_to_num(zone_level, nan=np.inf) - MERGE_PULL_CAP)
        newlvl = np.where(np.isfinite(zone_level), pulled, np.nan)
        upd = zm & (np.abs(np.nan_to_num(newlvl - zone_level, nan=0.0))
                    > 1e-6)[zone_id]
        zone_level = newlvl
        surface[upd] = zone_level[zone_id[upd]]

    # gentle within-pool smoothing (clamped)
    sm = ndimage.gaussian_filter(np.nan_to_num(surface, nan=0.0),
                                 cells(POOL_SMOOTH_M))
    smw = ndimage.gaussian_filter(np.isfinite(surface).astype(np.float32),
                                  cells(POOL_SMOOTH_M))
    smoothed = np.where(smw > 1e-4, sm / np.maximum(smw, 1e-9), np.nan)
    fin = np.isfinite(surface)
    surface[fin] = np.clip(smoothed[fin], surface[fin] - POOL_SMOOTH_CLAMP,
                           surface[fin] + POOL_SMOOTH_CLAMP)

    # wet cells + adoption of previously wet cells the bodies missed
    bridge = foot_closed & ~foot
    submerged = bridge & (~np.isfinite(terr_zmean) |
                          (np.nan_to_num(terr_top, neginf=-9) < surface - 0.005))
    wet_mask = (foot & foot_closed) | submerged
    surface = np.where(wet_mask, surface, np.nan)
    prev_left = (wat7 | watO) & ~pink7 & ~np.isfinite(surface)
    adopted = np.zeros((ny, nx), bool)
    if prev_left.any():
        have = np.isfinite(surface)
        dist_map, near = ndimage.distance_transform_edt(~have,
                                                        return_indices=True)
        near_surface = surface[tuple(near)]
        adopt = prev_left & (dist_map * CELL <= ADOPT_NEAR_M)
        surface[adopt] = near_surface[adopt]
        oz = np.where(np.isfinite(wzO), wzO, wz7)
        faraway = prev_left & ~adopt & np.isfinite(oz)
        surface[faraway] = oz[faraway]
        adopted = adopt | faraway
        # spill-cap each adopted patch against its surrounding rim
        lab3, ncl = ndimage.label(adopted, structure=np.ones((3, 3), bool))
        for ci in range(1, ncl + 1):
            cm = lab3 == ci
            ring = ndimage.binary_dilation(
                cm, structure=disk(cells(ADOPT_RIM_RING_M))) & ~cm
            rim = terr_zmax[ring & (terr_cnt > 0)]
            rim = rim[np.isfinite(rim)]
            if len(rim) >= 8:
                cap = float(np.quantile(rim, 0.70)) + 0.01
                over = cm & (surface > cap)
                surface[over] = cap
        band = ndimage.binary_dilation(adopted, structure=disk(3))
        surface, _ = spill_pull(surface, band=band)
        log(f"adopted prev cells: {int(adopted.sum()):,} (spill-capped)")

    # final global spill enforcement, then a tightly clamped smooth (L9)
    surface, _ = spill_pull(surface)
    smf = ndimage.gaussian_filter(np.nan_to_num(surface, nan=0.0),
                                  cells(FINAL_SMOOTH_M))
    smfw = ndimage.gaussian_filter(np.isfinite(surface).astype(np.float32),
                                   cells(FINAL_SMOOTH_M))
    smf2 = np.where(smfw > 1e-4, smf / np.maximum(smfw, 1e-9), np.nan)
    fin = np.isfinite(surface)
    surface[fin] = np.clip(smf2[fin], surface[fin] - FINAL_SMOOTH_CLAMP,
                           surface[fin] + FINAL_SMOOTH_CLAMP)
    surface, _ = spill_pull(surface)

    # seam mask: unknown-strip crossings and residual open steps feather out
    wet = np.isfinite(surface)
    seam = np.zeros((ny, nx), bool)
    for axis in (0, 1):
        aS = surface if axis == 0 else surface.T
        aT = terr_top if axis == 0 else terr_top.T
        aU = unknown if axis == 0 else unknown.T
        aW = wet if axis == 0 else wet.T
        both = aW[1:] & aW[:-1]
        dz = np.abs(aS[1:] - aS[:-1])
        dam = (np.maximum(np.nan_to_num(aT[1:], neginf=-9),
                          np.nan_to_num(aT[:-1], neginf=-9))
               > np.minimum(aS[1:], aS[:-1]) + ZONE_DAM_CLEAR)
        cross = both & (dz > STEP_SEAM) & ((aU[1:] & aU[:-1]) | ~dam)
        sm_ = np.zeros_like(aU)
        sm_[1:] |= cross
        sm_[:-1] |= cross
        seam |= sm_ if axis == 0 else sm_.T
    # metrics
    dzx = np.abs(np.diff(surface, axis=1))
    dzy = np.abs(np.diff(surface, axis=0))
    bx = wet[:, 1:] & wet[:, :-1]
    by = wet[1:] & wet[:-1]
    unknown_px = unknown[:, 1:] & unknown[:, :-1]
    unknown_py = unknown[1:] & unknown[:-1]
    damx = (np.maximum(np.nan_to_num(terr_top, neginf=-9)[:, 1:],
                       np.nan_to_num(terr_top, neginf=-9)[:, :-1])
            > np.minimum(surface[:, 1:], surface[:, :-1])) | unknown_px
    damy = (np.maximum(np.nan_to_num(terr_top, neginf=-9)[1:],
                       np.nan_to_num(terr_top, neginf=-9)[:-1])
            > np.minimum(surface[1:], surface[:-1])) | unknown_py
    open_steps = np.concatenate([dzx[bx & ~damx], dzy[by & ~damy]])
    report = {
        "wet_cells": int(wet.sum()),
        "wet_area_m2": float(wet.sum() * CELL * CELL),
        "zones": int(zid),
        "seam_cells": int(seam.sum()),
        "open_step_p99_mm": float(np.percentile(open_steps, 99) * 1000)
        if len(open_steps) else 0.0,
        "open_step_max_mm": float(open_steps.max() * 1000)
        if len(open_steps) else 0.0,
        "open_steps_gt_3cm": int((open_steps > 0.03).sum()),
        "prev_cells_lost": int(((wat7 | watO) & ~pink7 & ~wet).sum()),
    }
    log(f"surface: {report}")
    aux = {
        "terr_cnt": terr_cnt, "terr_zmin": terr_zmin, "terr_zmax": terr_zmax,
        "wat7": wat7, "watO": watO, "wz7": wz7, "wzO": wzO,
        "pink7": pink7,
    }
    return surface, seam, report, aux


# ==========================================================================
# point generation (tiled, position-hash deterministic; reviews H2, H4,
# M1, M3, M4, L4, L5 folded in)
# ==========================================================================

def generate_points(data_dir: Path, run_dir: Path, surface, seam, aux, log):
    from scipy import ndimage
    from scipy.spatial import cKDTree
    from scipy.ndimage import map_coordinates

    ny, nx = surface.shape
    x0 = aux["x0"]; y0 = aux["y0"]
    wet = np.isfinite(surface)

    # per-zone-safe sampling surface: fill + smooth, but clamp every sample
    # to the candidate cell's own level +-2 cm so a neighbouring pool can
    # never tilt this one across a thin dam (review M1)
    surf_fill = fill_nan_nearest(surface.copy())
    surf_s = ndimage.gaussian_filter(surf_fill, 1.0)
    wet_f = ndimage.uniform_filter(wet.astype(np.float32), size=3)
    seam_soft = ndimage.gaussian_filter(
        seam.astype(np.float32), cells(SEAM_FEATHER_M)) * (2.0 * math.pi *
        cells(SEAM_FEATHER_M) ** 2)
    seam_soft = np.clip(seam_soft, 0.0, 1.0)

    # ripple grid at the leveller cell (same recipe/energy as v6)
    edge_distance = ndimage.distance_transform_edt(wet).astype(np.float32) * CELL
    grid_spec = v6.GridSpec(x0, y0, x0 + nx * CELL, y0 + ny * CELL, CELL)
    ripple, _ = v6.build_ripple_grid(grid_spec, wet, edge_distance)

    # 1 cm terrain density of the WATERLINE COLLAR only (review H2): points
    # within [surface-0.10, surface+0.5]; deep submerged floors do not thin
    # the sheet and emergent rock is handled by the carve instead.
    dnx = int(round(nx * CELL / DENS_CELL))
    dny = int(round(ny * CELL / DENS_CELL))
    collar_cnt = np.zeros(dny * dnx, np.float32)
    emergent_parts = []
    for role in ("SAND", "ROCK"):
        mm = v6.memmap_cloud(data_dir / f"Site1-{role}-5mm.ply")
        mm = mm[0] if isinstance(mm, tuple) else mm
        for i in range(0, len(mm), CHUNK * 4):
            c = mm[i:i + CHUNK * 4]
            px = c["x"].astype(np.float64); py = c["y"].astype(np.float64)
            pz = c["z"].astype(np.float32)
            u = (px - x0) / CELL - 0.5
            v = (py - y0) / CELL - 0.5
            sz = map_coordinates(surf_s, [v, u], order=1,
                                 mode="nearest").astype(np.float32)
            wetness = map_coordinates(wet.astype(np.float32), [v, u],
                                      order=1, mode="constant")
            near_water = wetness > 0.05
            collar = near_water & (pz > sz - 0.10) & (pz < sz + COLLAR_HI)
            gx = ((px[collar] - x0) / DENS_CELL).astype(np.int64)
            gy = ((py[collar] - y0) / DENS_CELL).astype(np.int64)
            ok = (gx >= 0) & (gx < dnx) & (gy >= 0) & (gy < dny)
            np.add.at(collar_cnt, gy[ok] * dnx + gx[ok], 1.0)
            emer = near_water & (pz > sz + COLLAR_LO) & (pz < sz + COLLAR_HI)
            if emer.any():
                emergent_parts.append(np.stack(
                    [px[emer].astype(np.float32), py[emer].astype(np.float32),
                     pz[emer]], 1))
    collar_cnt = collar_cnt.reshape(dny, dnx)
    dens = ndimage.gaussian_filter(collar_cnt, DENS_SMOOTH_M / DENS_CELL) \
        / (DENS_CELL * DENS_CELL)
    ET = np.concatenate(emergent_parts) if emergent_parts else \
        np.zeros((0, 3), np.float32)
    log(f"collar terrain: emergent pts {len(ET):,}")
    emer_tree = cKDTree(ET[:, :2]) if len(ET) else None

    # seam-band mask at density resolution: complement applies near shore
    emer_occ = np.zeros((dny, dnx), bool)
    if len(ET):
        egx = ((ET[:, 0] - x0) / DENS_CELL).astype(np.int64)
        egy = ((ET[:, 1] - y0) / DENS_CELL).astype(np.int64)
        ok = (egx >= 0) & (egx < dnx) & (egy >= 0) & (egy < dny)
        emer_occ[egy[ok], egx[ok]] = True
    shore_dist = ndimage.distance_transform_edt(~emer_occ) * DENS_CELL
    seam_w = np.clip(1.0 - shore_dist / SEAM_BAND_M, 0.0, 1.0)

    # targets: smoothed collar density in the seam band, interior constants
    target_local = np.clip(
        ndimage.gaussian_filter(collar_cnt, TARGET_SMOOTH_M / DENS_CELL)
        / (DENS_CELL * DENS_CELL) * 6.45,       # measured 1mm/5mm seam ratio
        *T_FINE_CLAMP)
    T_fine = seam_w * target_local + (1.0 - seam_w) * INTERIOR_T_FINE
    # deficit uses the collar density only inside the seam band
    D_eff = dens * seam_w

    salt_jx, salt_jy, salt_acc, salt_thin, salt_z = 11, 23, 37, 53, 71
    lat_nx = int(math.ceil(nx * CELL / PITCH_FINE))
    records = []
    accepted_total = 0
    tiles_y = int(math.ceil(ny * CELL / TILE_M))
    tiles_x = int(math.ceil(nx * CELL / TILE_M))
    out_xyz = []
    out_extra = []
    for ty in range(tiles_y):
        for tx in range(tiles_x):
            wx0 = x0 + tx * TILE_M; wy0 = y0 + ty * TILE_M
            cy0 = int(ty * TILE_M / CELL); cx0 = int(tx * TILE_M / CELL)
            cy1 = min(ny, cy0 + cells(TILE_M)); cx1 = min(nx, cx0 + cells(TILE_M))
            if not wet[cy0:cy1, cx0:cx1].any():
                continue
            ix0 = int(round((wx0 - x0) / PITCH_FINE))
            iy0 = int(round((wy0 - y0) / PITCH_FINE))
            n_i = int(TILE_M / PITCH_FINE) + 1
            ii, jj = np.meshgrid(np.arange(ix0, ix0 + n_i, dtype=np.int64),
                                 np.arange(iy0, iy0 + n_i, dtype=np.int64))
            ii = ii.ravel(); jj = jj.ravel()
            qx = x0 + (ii.astype(np.float64) + 0.5) * PITCH_FINE + \
                (hash01(ii, jj, salt_jx) - 0.5) * PITCH_FINE
            qy = y0 + (jj.astype(np.float64) + 0.5) * PITCH_FINE + \
                (hash01(ii, jj, salt_jy) - 0.5) * PITCH_FINE
            u = (qx - x0) / CELL - 0.5
            v = (qy - y0) / CELL - 0.5
            wetness = map_coordinates(wet_f, [v, u], order=1, mode="constant")
            keep = wetness > 0.05
            if not keep.any():
                continue
            ii, jj, qx, qy, u, v = ii[keep], jj[keep], qx[keep], qy[keep], \
                u[keep], v[keep]
            wetness = wetness[keep]
            own = map_coordinates(surface if False else surf_fill,
                                  [np.round(v), np.round(u)], order=0,
                                  mode="nearest")
            sz = map_coordinates(surf_s, [v, u], order=1, mode="nearest")
            sz = np.clip(sz, own - 0.02, own + 0.02)      # review M1
            rz = map_coordinates(ripple, [v, u], order=1, mode="nearest")
            sz = (sz + rz).astype(np.float64)
            # feathered standoff around emergent terrain (folded into one
            # acceptance probability; review L5)
            if emer_tree is not None:
                d2, _ = emer_tree.query(np.stack([qx, qy], 1), k=1,
                                        workers=-1,
                                        distance_upper_bound=STAND_OUT)
                carve_p = np.clip((d2 - STAND_IN) / (STAND_OUT - STAND_IN),
                                  0.0, 1.0)
                carve_p[~np.isfinite(d2)] = 1.0
            else:
                carve_p = np.ones(len(qx))
            dgx = np.clip(((qx - x0) / DENS_CELL).astype(np.int64), 0, dnx - 1)
            dgy = np.clip(((qy - y0) / DENS_CELL).astype(np.int64), 0, dny - 1)
            Tf = T_fine[dgy, dgx]
            deficit = np.clip(Tf - D_eff[dgy, dgx], 0.0, None)
            p = np.clip(deficit * PITCH_FINE ** 2, 0.0, 1.0)
            p *= carve_p
            # organic macro edges + seam feather
            edge = np.clip((wetness - EDGE_FEATHER_LO) /
                           (EDGE_FEATHER_HI - EDGE_FEATHER_LO), 0.0, 1.0)
            p *= edge * edge * (3.0 - 2.0 * edge)
            sgy = np.clip((v + 0.5).astype(np.int64), 0, ny - 1)
            sgx = np.clip((u + 0.5).astype(np.int64), 0, nx - 1)
            p *= 1.0 - seam_soft[sgy, sgx]
            acc = hash01(ii, jj, salt_acc) < p
            if not acc.any():
                continue
            ii, jj = ii[acc], jj[acc]
            qx, qy, sz = qx[acc], qy[acc], sz[acc]
            Tf = Tf[acc]
            dgx, dgy = dgx[acc], dgy[acc]
            zj = (hash01(ii, jj, salt_z) - 0.5) * 2.0 * Z_NOISE
            sz = sz + zj
            # 5 mm-layer membership: hash-derived subset with the 5 mm target
            T5 = np.clip(seam_w[dgy, dgx] *
                         np.clip(target_local[dgy, dgx] / 6.45,
                                 *T_5MM_CLAMP) +
                         (1.0 - seam_w[dgy, dgx]) * INTERIOR_T_5MM,
                         *T_5MM_CLAMP)
            ratio = np.clip(T5 / np.maximum(Tf, 1.0), 0.0, 1.0)
            in5 = hash01(ii, jj, salt_thin) < ratio
            out_xyz.append(np.stack([qx.astype(np.float32),
                                     qy.astype(np.float32),
                                     sz.astype(np.float32)], 1))
            out_extra.append(in5)
            accepted_total += int(acc.sum())
    XYZ = np.concatenate(out_xyz) if out_xyz else np.zeros((0, 3), np.float32)
    IN5 = np.concatenate(out_extra) if out_extra else np.zeros(0, bool)
    log(f"accepted fine points: {len(XYZ):,}; 5 mm subset: {int(IN5.sum()):,}")

    # puddle filter: drop tiny isolated splashes (review w1)
    occ_nx = int(round(nx * CELL / 0.05)); occ_ny = int(round(ny * CELL / 0.05))
    gx = np.clip(((XYZ[:, 0] - x0) / 0.05).astype(np.int64), 0, occ_nx - 1)
    gy = np.clip(((XYZ[:, 1] - y0) / 0.05).astype(np.int64), 0, occ_ny - 1)
    occ = np.zeros((occ_ny, occ_nx), bool)
    occ[gy, gx] = True
    lab, ncomp = ndimage.label(occ, structure=np.ones((3, 3), bool))
    sizes = np.bincount(lab.ravel())
    tiny = sizes * 0.05 * 0.05 < PUDDLE_MIN_M2
    tiny[0] = False
    drop = tiny[lab[gy, gx]]
    if drop.any():
        XYZ, IN5 = XYZ[~drop], IN5[~drop]
        log(f"puddle filter dropped {int(drop.sum()):,} points "
            f"({int(tiny.sum())} splashes)")
    return XYZ, IN5, surf_s


# ==========================================================================
# attributes
# ==========================================================================

DONOR_FIELDS = ("red", "green", "blue", "scalar_Intensity", "scalar_Composite",
                "scalar_A_R_Shelter_Lower", "scalar_A_R_RainExposure_Lower",
                "scalar_A_R_SVF_Lower")


def build_attributes(data_dir: Path, XYZ, IN5, surface, aux, log):
    """Donor colour/intensity/environment fields (prior water weighted over
    terrain), diffusion-filled with variance noise; geometry statics from the
    surface model."""
    from scipy import ndimage
    from scipy.ndimage import map_coordinates

    ny, nx = surface.shape
    x0, y0 = aux["x0"], aux["y0"]
    AC = 0.025
    anx = int(round(nx * CELL / AC)); any_ = int(round(ny * CELL / AC))
    asum = {f: np.zeros(any_ * anx, np.float64) for f in DONOR_FIELDS}
    asq = {f: np.zeros(any_ * anx, np.float64) for f in DONOR_FIELDS}
    acnt = np.zeros(any_ * anx, np.float64)
    for path, weight in ((data_dir / "Site1-WATER-5mm.ply", 3.0),
                        (data_dir / "Site1-SAND-5mm.ply", 1.0),
                        (data_dir / "Site1-ROCK-5mm.ply", 1.0)):
        mm = v6.memmap_cloud(path)
        mm = mm[0] if isinstance(mm, tuple) else mm
        for i in range(0, len(mm), CHUNK * 4):
            c = mm[i:i + CHUNK * 4]
            gx = ((c["x"] - x0) / AC).astype(np.int64)
            gy = ((c["y"] - y0) / AC).astype(np.int64)
            ok = (gx >= 0) & (gx < anx) & (gy >= 0) & (gy < any_)
            k = gy[ok] * anx + gx[ok]
            np.add.at(acnt, k, weight)
            for f in DONOR_FIELDS:
                vals = np.asarray(c[f], np.float64)[ok]
                np.add.at(asum[f], k, vals * weight)
                np.add.at(asq[f], k, vals * vals * weight)
    have = acnt > 0
    log(f"donor cells: {int(have.sum()):,}")

    def diffuse(mean_grid):
        filled = mean_grid.copy()
        for sigma in (2.0, 4.0, 8.0, 16.0, 32.0, 64.0, 128.0):
            known = np.isfinite(filled).astype(np.float32)
            blurred = ndimage.gaussian_filter(
                np.nan_to_num(filled, nan=0.0), sigma)
            weight = ndimage.gaussian_filter(known, sigma)
            estimate = np.divide(blurred, weight,
                                 out=np.zeros_like(blurred),
                                 where=weight > 1e-4)
            need = ~np.isfinite(filled) & (weight > 1e-4)
            filled[need] = estimate[need]
        rem = ~np.isfinite(filled)
        if rem.any():
            filled[rem] = np.nanmedian(mean_grid)
        return ndimage.gaussian_filter(filled, 1.0)

    # deterministic correlated noise fields
    ii, jj = np.meshgrid(np.arange(anx, dtype=np.int64),
                         np.arange(any_, dtype=np.int64))
    noise_a = ndimage.gaussian_filter(
        (hash01(ii.ravel(), jj.ravel(), 101).reshape(any_, anx) - 0.5)
        .astype(np.float32), 4.0)
    noise_a /= max(noise_a.std(), 1e-6)
    noise_b = ndimage.gaussian_filter(
        (hash01(ii.ravel(), jj.ravel(), 202).reshape(any_, anx) - 0.5)
        .astype(np.float32), 4.0)
    noise_b /= max(noise_b.std(), 1e-6)

    # surface-derived geometry statics at the leveller cell
    surf_fill = fill_nan_nearest(surface.copy())
    wetm = np.isfinite(surface)
    gy_, gx_ = np.gradient(ndimage.gaussian_filter(surf_fill, 1.0), CELL)
    statics = {}
    for label, sigma in GEOMETRY_SIGMAS.items():
        smooth = ndimage.gaussian_filter(surf_fill, sigma)
        residual = np.where(wetm, surf_fill - smooth, 0.0)
        rough = np.sqrt(np.maximum(ndimage.gaussian_filter(
            residual * residual, max(0.75, sigma)), 0.0))
        sgy, sgx = np.gradient(smooth, CELL)
        dyy, _ = np.gradient(sgy, CELL)
        dxy, dxx = np.gradient(sgx, CELL)
        statics[f"scalar_A_R_MeanCurvature_{label}"] = 0.5 * (dxx + dyy)
        statics[f"scalar_A_R_CrossCurvature_{label}"] = dxy
        statics[f"scalar_A_R_Recession_{label}"] = residual
        statics[f"scalar_A_R_Roughness_{label}"] = rough
    combined = {}
    for base in ("MeanCurvature", "CrossCurvature", "Recession", "Roughness"):
        combined[f"scalar_A_R_{base}_Combined"] = (
            0.5 * statics[f"scalar_A_R_{base}_Fine"] +
            0.35 * statics[f"scalar_A_R_{base}_Medium"] +
            0.15 * statics[f"scalar_A_R_{base}_Broad"])
    statics.update(combined)
    statics["scalar_A_R_RoughnessRelative_FineMedium"] = np.divide(
        statics["scalar_A_R_Roughness_Fine"],
        np.maximum(statics["scalar_A_R_Roughness_Medium"], 1e-6))

    dtype = None
    mm = v6.memmap_cloud(data_dir / "Site1-SAND-5mm.ply")
    mm = mm[0] if isinstance(mm, tuple) else mm
    dtype = mm.dtype
    record = np.zeros(len(XYZ), dtype)
    record["x"], record["y"], record["z"] = XYZ.T
    record["scalar_ScanID"] = WATER_SCAN_ID

    u = (XYZ[:, 0].astype(np.float64) - x0) / CELL - 0.5
    v = (XYZ[:, 1].astype(np.float64) - y0) / CELL - 0.5
    dzdx = map_coordinates(gx_, [v, u], order=1, mode="nearest")
    dzdy = map_coordinates(gy_, [v, u], order=1, mode="nearest")
    normal = np.stack([-dzdx, -dzdy, np.ones(len(XYZ))], 1)
    normal /= np.linalg.norm(normal, axis=1, keepdims=True)
    record["nx"], record["ny"], record["nz"] = normal.T.astype(np.float32)
    slope = np.degrees(np.arccos(np.clip(normal[:, 2], -1.0, 1.0)))
    record["scalar_A_R_Slope_deg"] = slope.astype(np.float32)
    record["scalar_A_R_Horizontalness"] = np.cos(
        np.radians(slope)).astype(np.float32)
    horiz = np.linalg.norm(normal[:, :2], axis=1)
    safe = horiz > 1e-5
    record["scalar_A_R_Downhill_X"][safe] = (
        normal[safe, 0] / horiz[safe]).astype(np.float32)
    record["scalar_A_R_Downhill_Y"][safe] = (
        normal[safe, 1] / horiz[safe]).astype(np.float32)
    record["scalar_A_R_DownhillMagnitude"] = np.tan(
        np.radians(slope)).astype(np.float32)
    for name, grid in statics.items():
        record[name] = map_coordinates(
            grid, [v, u], order=1, mode="nearest").astype(np.float32)

    au = (XYZ[:, 0].astype(np.float64) - x0) / AC - 0.5
    av = (XYZ[:, 1].astype(np.float64) - y0) / AC - 0.5
    for f in DONOR_FIELDS:
        mean_grid = np.divide(asum[f], acnt,
                              out=np.full(any_ * anx, np.nan),
                              where=have).reshape(any_, anx)
        var_grid = np.divide(asq[f], acnt,
                             out=np.full(any_ * anx, np.nan),
                             where=have).reshape(any_, anx)
        var_grid = np.clip(var_grid - np.nan_to_num(mean_grid, nan=0.0) ** 2,
                           0.0, None)
        mean_f = diffuse(mean_grid)
        sigma_f = np.sqrt(diffuse(var_grid))
        nf = noise_a if f in ("red", "green", "blue") else noise_b
        val = map_coordinates(mean_f, [av, au], order=1, mode="nearest")
        spread = map_coordinates(sigma_f, [av, au], order=1, mode="nearest")
        wob = map_coordinates(nf, [av, au], order=1, mode="nearest")
        val = val + 0.7 * spread * wob
        if record.dtype[f].kind == "u":
            record[f] = np.clip(val + 0.5, 0, 255).astype(np.uint8)
        else:
            record[f] = val.astype(record.dtype[f].base)
    log("attributes built")
    return record


def write_ply(path: Path, record: np.ndarray, comments: list[str]):
    typemap = {"f4": "float", "f8": "double", "u1": "uchar"}
    header = ["ply", "format binary_little_endian 1.0"]
    header += [f"comment {c}" for c in comments]
    header.append(f"element vertex {len(record)}")
    for name in record.dtype.names:
        header.append(
            f"property {typemap[record.dtype[name].str.lstrip('<>|=')]} {name}")
    header.append("end_header")
    tmp = path.with_suffix(path.suffix + ".tmp")
    with open(tmp, "wb") as handle:
        handle.write(("\n".join(header) + "\n").encode("ascii"))
        record.tofile(handle)
    tmp.replace(path)


def sha256_path(path: Path, chunk: int = 32 * 1024 * 1024) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        while True:
            block = handle.read(chunk)
            if not block:
                break
            digest.update(block)
    return digest.hexdigest()


# ==========================================================================
# stages
# ==========================================================================

def app_running() -> bool:
    import subprocess
    result = subprocess.run(["pgrep", "-f", "MacOS/invisible_places"],
                            capture_output=True, text=True)
    return bool(result.stdout.strip())


def source_fingerprint(path: Path) -> dict:
    stat = path.stat()
    return {"path": str(path), "bytes": stat.st_size,
            "mtime_ns": stat.st_mtime_ns}


def build(args):
    from scipy import ndimage
    run = args.run_dir
    run.mkdir(parents=True, exist_ok=True)
    (args.data_dir / "PatchRefinement/.invisible_places-ignore").touch()
    logf = open(run / "build.log", "a")

    def log(msg):
        line = f"[build] {msg}"
        print(line, flush=True)
        logf.write(line + "\n")
        logf.flush()

    G = stream_grids(args.data_dir, log)
    surface, seam, report, aux = build_water_surface(G, log)
    aux["x0"], aux["y0"] = G["x0"], G["y0"]
    np.savez_compressed(run / "leveller.npz", surface=surface, seam=seam,
                        meta=np.array([G["x0"], G["y0"], CELL]))
    XYZ, IN5, surf_s = generate_points(args.data_dir, run, surface, seam,
                                       aux, log)
    record = build_attributes(args.data_dir, XYZ, IN5, surface, aux, log)
    stamp = dt.date.today().isoformat()
    comments = [
        f"Site1 water v8 hydraulic reconstruction generated {stamp}",
        "Pool-physics levels (dam-split zones, spill caps, floor drying),",
        "point-level feathered carving around emergent terrain only, and",
        "position-hash density acceptance matched to the measured",
        "neighbourhood; the 5 mm layer is a hash-derived subset of the",
        "fine layer so the two stay co-registered.",
        f"ScanID={WATER_SCAN_ID:.0f}",
        "Generated by scripts/rebuild_site1_fossils_v8.py",
    ]
    fine_path = run / "Site1-WATER-1mm.candidate.ply"
    coarse_path = run / "Site1-WATER-5mm.candidate.ply"
    write_ply(fine_path, record, comments + [
        "fine layer: seam density matched to the 1 mm-class clouds"])
    write_ply(coarse_path, record[IN5], comments + [
        "5 mm layer: hash-derived subset of the fine layer"])
    log(f"candidates written: fine {len(record):,} "
        f"({fine_path.stat().st_size / 1e9:.2f} GB), "
        f"5mm {int(IN5.sum()):,} "
        f"({coarse_path.stat().st_size / 1e9:.2f} GB)")

    # review renders
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    x0, y0 = G["x0"], G["y0"]
    fig, ax = plt.subplots(figsize=(11, 14), dpi=110)
    terr = (G["sand_cnt"] + G["rock_cnt"]).astype(np.float32)
    ax.imshow(np.log1p(terr), origin="lower", cmap="Greys",
              extent=[x0, x0 + G["nx"] * CELL, y0, y0 + G["ny"] * CELL])
    sub = record[IN5]
    ax.plot(sub["x"][::7], sub["y"][::7], ".", ms=0.25, color="crimson",
            rasterized=True)
    ax.set_title("v8 water (5 mm layer, 1/7 sample) over terrain")
    fig.savefig(run / "review-water-topdown.png", bbox_inches="tight")
    plt.close(fig)
    fig, axes = plt.subplots(3, 1, figsize=(18, 13), dpi=110)
    slabs = [(841.45, 841.70, "y", "step band y=841.5"),
             (807.90, 808.15, "y", "starburst y=808.0"),
             (762.70, 762.95, "x", "terraces x=762.8")]
    for ax2, (lo, hi, axis, title) in zip(axes, slabs):
        if axis == "y":
            m = (sub["y"] > lo) & (sub["y"] < hi)
            h = sub["x"][m]
        else:
            m = (sub["x"] > lo) & (sub["x"] < hi)
            h = sub["y"][m]
        ax2.plot(h, sub["z"][m], ".", ms=1.5, color="crimson", rasterized=True)
        ax2.set_title(title)
    fig.tight_layout()
    fig.savefig(run / "review-sections.png", bbox_inches="tight")
    plt.close(fig)

    manifest = {
        "version": 8,
        "created": dt.datetime.now().isoformat(timespec="seconds"),
        "data_dir": str(args.data_dir),
        "run_dir": str(run),
        "surface_report": report,
        "fine_points": int(len(record)),
        "coarse_points": int(IN5.sum()),
        "fine_candidate": str(fine_path),
        "coarse_candidate": str(coarse_path),
        "fine_sha256": sha256_path(fine_path),
        "coarse_sha256": sha256_path(coarse_path),
        "source_water_sha256": sha256_path(
            args.data_dir / "Site1-WATER-5mm.ply"),
        "sources": {name: source_fingerprint(args.data_dir / name)
                    for name in ("Site1-SAND-5mm.ply", "Site1-ROCK-5mm.ply",
                                 "Site1-WATER-5mm.ply",
                                 "Site1-WATER-5mm-old02.ply")},
        "installed": False,
    }
    (run / "manifest.json").write_text(json.dumps(manifest, indent=2))
    log("manifest written")


def verify(args) -> dict:
    from scipy import ndimage
    from scipy.spatial import cKDTree
    run = args.run_dir
    manifest = json.loads((run / "manifest.json").read_text())
    failures = []
    checks = {}
    lev = np.load(run / "leveller.npz")
    surface = lev["surface"]
    x0, y0, cell = lev["meta"]
    coarse = v6.memmap_cloud(Path(manifest["coarse_candidate"]))
    coarse = coarse[0] if isinstance(coarse, tuple) else coarse
    fine = v6.memmap_cloud(Path(manifest["fine_candidate"]))
    fine = fine[0] if isinstance(fine, tuple) else fine
    checks["counts"] = {"fine": len(fine), "coarse": len(coarse)}
    if len(fine) != manifest["fine_points"]:
        failures.append("fine count mismatch")
    if len(coarse) != manifest["coarse_points"]:
        failures.append("coarse count mismatch")
    for name, mm in (("fine", fine), ("coarse", coarse)):
        scan = np.asarray(mm["scalar_ScanID"][::997])
        if not np.all(scan == WATER_SCAN_ID):
            failures.append(f"{name}: ScanID not uniform")
        for f in mm.dtype.names:
            if mm.dtype[f].kind == "f":
                sample = np.asarray(mm[f][::9973], np.float64)
                if not np.isfinite(sample).all():
                    failures.append(f"{name}: non-finite {f}")
                    break
    # open-step check on the rasterized coarse candidate
    gx = ((coarse["x"] - x0) / cell).astype(np.int64)
    gy = ((coarse["y"] - y0) / cell).astype(np.int64)
    ny, nx = surface.shape
    ok = (gx >= 0) & (gx < nx) & (gy >= 0) & (gy < ny)
    zsum = np.zeros(ny * nx); cnt = np.zeros(ny * nx)
    np.add.at(zsum, gy[ok] * nx + gx[ok], coarse["z"][ok].astype(np.float64))
    np.add.at(cnt, gy[ok] * nx + gx[ok], 1)
    wz = np.divide(zsum, cnt, out=np.full(ny * nx, np.nan),
                   where=cnt > 0).reshape(ny, nx)
    wet = np.isfinite(wz)
    # a light terrain-top grid to exempt dam-adjacent pairs (steps at rock
    # sills are pool physics, not defects)
    tmax = np.full(ny * nx, -np.inf, np.float32)
    emer_dil = np.zeros((ny, nx), bool)
    for role in ("SAND", "ROCK"):
        mm2 = v6.memmap_cloud(args.data_dir / f"Site1-{role}-5mm.ply")
        mm2 = mm2[0] if isinstance(mm2, tuple) else mm2
        for i in range(0, len(mm2), CHUNK * 4):
            c = mm2[i:i + CHUNK * 4]
            g2x = ((c["x"] - x0) / cell).astype(np.int64)
            g2y = ((c["y"] - y0) / cell).astype(np.int64)
            ok2 = (g2x >= 0) & (g2x < nx) & (g2y >= 0) & (g2y < ny)
            np.maximum.at(tmax, g2y[ok2] * nx + g2x[ok2],
                          c["z"][ok2].astype(np.float32))
    tmax = tmax.reshape(ny, nx)
    open_steps = []
    dam_steps = 0
    for axis in (0, 1):
        a = wz if axis == 0 else wz.T
        w = wet if axis == 0 else wet.T
        t = tmax if axis == 0 else tmax.T
        both = w[1:] & w[:-1]
        dz = np.abs(a[1:] - a[:-1])
        dam = (np.maximum(t[1:], t[:-1])
               > np.minimum(a[1:], a[:-1]) - 0.005)
        open_steps.append(dz[both & ~dam])
        dam_steps += int((dz[both & dam] > 0.05).sum())
    open_steps = np.concatenate(open_steps)
    checks["neighbour_dz_mm"] = {
        "open_p95": float(np.percentile(open_steps, 95) * 1000)
        if len(open_steps) else 0.0,
        "open_p99": float(np.percentile(open_steps, 99) * 1000)
        if len(open_steps) else 0.0,
        "open_gt_5cm": int((open_steps > 0.05).sum()),
        "dam_gt_5cm_pairs": dam_steps,
    }
    if checks["neighbour_dz_mm"]["open_gt_5cm"] > 0.001 * wet.sum():
        failures.append("too many open >5 cm neighbour steps")
    # coverage: candidate covers the surface minus the standoff halo around
    # emergent terrain, the seam feather, and the boundary feather ring
    from scipy import ndimage as ndi
    seam_m = lev["seam"]
    lev_wet = np.isfinite(surface)
    halo = ndi.binary_dilation(
        tmax > np.nan_to_num(np.where(lev_wet, surface, np.inf),
                             posinf=np.inf) - 0.01,
        structure=disk(2))
    edge_ring = lev_wet & ~ndi.binary_erosion(lev_wet, structure=disk(2))
    expect = lev_wet & ~halo & ~ndi.binary_dilation(seam_m, structure=disk(3)) \
        & ~edge_ring
    uncovered = expect & ~wet
    checks["wet_cells"] = {"leveller": int(lev_wet.sum()),
                          "expected": int(expect.sum()),
                          "candidate": int(wet.sum()),
                          "uncovered": int(uncovered.sum())}
    if uncovered.sum() > 0.03 * max(expect.sum(), 1):
        failures.append("candidate leaves expected open water dry")
    # the user's three circled regions must hold water
    for name, (rx, ry) in (("circle1", (765.0, 826.0)),
                           ("circle2", (772.4, 810.0)),
                           ("starburst", (777.4, 808.3))):
        gx0 = int((rx - 0.75 - x0) / cell); gx1 = int((rx + 0.75 - x0) / cell)
        gy0 = int((ry - 0.75 - y0) / cell); gy1 = int((ry + 0.75 - y0) / cell)
        wcount = int(wet[gy0:gy1, gx0:gx1].sum())
        checks[f"region_{name}_wet_cells"] = wcount
        if wcount < 60:
            failures.append(f"region {name} has almost no water")
    report = {"created": dt.datetime.now().isoformat(timespec="seconds"),
              "checks": checks, "failures": failures}
    (run / "verification-report.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(report["checks"], indent=1))
    print("FAILURES:", failures if failures else "none")
    return report


def _terrain_micro_cull(args, run: Path, manifest: dict, log):
    """Drop detached tufts from the v7 ScanID-9 tails (append-only layout;
    removal keeps the truncate-to-N_orig rollback valid)."""
    from scipy.spatial import cKDTree
    v7m = json.loads((V7_RUN / "manifest.json").read_text())
    culls = {}
    for role in ("SAND", "ROCK"):
        for spacing in ("5mm", "1mm"):
            key = f"{role}-{spacing}"
            path = args.data_dir / f"Site1-{role}-{spacing}.ply"
            mm = v6.memmap_cloud(path)
            mm, header = (mm[0], mm[2]) if isinstance(mm, tuple) else (mm, None)
            n_orig = v7m["source_fingerprints"][key]["points"]
            tail = np.array(mm[n_orig:])
            if not len(tail):
                continue
            stride = 3 if spacing == "1mm" else 1
            prefix = mm[:n_orig:stride]
            tree = cKDTree(np.stack([prefix["x"], prefix["y"], prefix["z"]],
                                    1).astype(np.float32))
            txyz = np.stack([tail["x"], tail["y"], tail["z"]], 1)
            d, _ = tree.query(txyz, k=1, workers=-1)
            near_rim = (np.linalg.norm(
                txyz - np.asarray(RIM_SKIRT_CENTRE, np.float32), axis=1)
                <= RIM_SKIRT_R)
            cull = (d > CULL_NN) | (near_rim & (d > RIM_SKIRT_NN))
            if not cull.any():
                culls[key] = 0
                continue
            removed = tail[cull]
            write_ply(run / f"culled-scanid9-{key}.ply", removed,
                      [f"v8 micro-cull of detached v7 ScanID9 additions {key}"])
            kept_tail = tail[~cull]
            with open(path, "r+b") as handle:
                # header size = file size - record bytes
                total = path.stat().st_size
                header_len = total - len(mm) * mm.dtype.itemsize
                new_count = n_orig + len(kept_tail)
                handle.seek(header_len + n_orig * mm.dtype.itemsize)
                kept_tail.tofile(handle)
                handle.truncate(header_len + new_count * mm.dtype.itemsize)
                handle.seek(0)
                head = handle.read(header_len)
                import re as _re
                m = _re.search(rb"(element vertex )(\d+)([ \t]*)", head)
                field = m.group(2) + m.group(3)
                repl = str(new_count).encode().ljust(len(field))
                if len(repl) > len(field):
                    raise RuntimeError("count does not fit header")
                handle.seek(m.start(2))
                handle.write(repl)
            culls[key] = int(cull.sum())
            log(f"micro-cull {key}: removed {int(cull.sum()):,} of "
                f"{len(tail):,} additions")
    return culls


def install(args):
    run = args.run_dir
    manifest = json.loads((run / "manifest.json").read_text())
    if app_running():
        raise SystemExit("refusing: invisible_places is running")
    report = verify(args)
    if report["failures"]:
        raise SystemExit(f"verification failed: {report['failures']}")
    canonical = args.data_dir / "Site1-WATER-5mm.ply"
    if sha256_path(canonical) != manifest["source_water_sha256"]:
        raise RuntimeError("canonical WATER changed since build")

    def log(msg):
        print(f"[install] {msg}", flush=True)

    preserve = run / "Site1-WATER-5mm.v7-installed.ply"
    if preserve.exists():
        raise SystemExit(f"refusing to overwrite {preserve}")
    shutil.move(str(canonical), str(preserve))
    try:
        shutil.copy2(manifest["coarse_candidate"], str(canonical))
    except Exception:
        shutil.move(str(preserve), str(canonical))
        raise
    log(f"{canonical.name} is now the v8 5 mm sheet; the v7 install is "
        f"preserved in the run directory")

    manifest["terrain_cull"] = _terrain_micro_cull(args, run, manifest, log)

    archive = run / "water-archive"
    archive.mkdir(exist_ok=True)
    moved = []
    for name in ("Site1-WATER-5mm-old01.ply", "Site1-WATER-5mm-old02.ply"):
        src = args.data_dir / name
        if src.exists():
            shutil.move(str(src), str(archive / name))
            moved.append(name)
    (archive / "README.txt").write_text(
        "These earlier water generations match the app's water-token\n"
        "detection and were auto-loading as extra stacked sheets. They are\n"
        "byte-identical moves; restore returns them to Data/Scene1. The v6\n"
        "and v7 run manifests reference them at their old paths - move them\n"
        "back before running those scripts' restore stages.\n")
    log(f"archived stacked water backups: {moved}")

    manifest["installed"] = True
    manifest["installed_at"] = dt.datetime.now().isoformat(timespec="seconds")
    manifest["archived_old_water"] = moved
    (run / "manifest.json").write_text(json.dumps(manifest, indent=2))
    log("install complete")


def restore(args):
    run = args.run_dir
    manifest = json.loads((run / "manifest.json").read_text())
    if app_running():
        raise SystemExit("refusing: invisible_places is running")
    canonical = args.data_dir / "Site1-WATER-5mm.ply"
    preserve = run / "Site1-WATER-5mm.v7-installed.ply"

    def log(msg):
        print(f"[restore] {msg}", flush=True)

    # terrain tails: re-append the culled ScanID9 records
    for key, culled in manifest.get("terrain_cull", {}).items():
        if not culled:
            continue
        role, spacing = key.split("-")
        path = args.data_dir / f"Site1-{role}-{spacing}.ply"
        removed = v6.memmap_cloud(run / f"culled-scanid9-{key}.ply")
        removed = removed[0] if isinstance(removed, tuple) else removed
        mm = v6.memmap_cloud(path)
        mm = mm[0] if isinstance(mm, tuple) else mm
        total = path.stat().st_size
        header_len = total - len(mm) * mm.dtype.itemsize
        new_count = len(mm) + len(removed)
        with open(path, "r+b") as handle:
            handle.seek(0, 2)
            np.array(removed).tofile(handle)
            handle.seek(0)
            head = handle.read(header_len)
            import re as _re
            m = _re.search(rb"(element vertex )(\d+)([ \t]*)", head)
            field = m.group(2) + m.group(3)
            repl = str(new_count).encode().ljust(len(field))
            handle.seek(m.start(2))
            handle.write(repl)
        log(f"terrain {key}: re-appended {len(removed):,} culled additions")

    archive = run / "water-archive"
    for name in manifest.get("archived_old_water", []):
        src = archive / name
        if src.exists():
            shutil.move(str(src), str(args.data_dir / name))
            log(f"returned {name}")
    if preserve.exists():
        if canonical.exists():
            canonical.unlink()
        shutil.move(str(preserve), str(canonical))
        log("v7 canonical restored")
    manifest["installed"] = False
    manifest["restored_at"] = dt.datetime.now().isoformat(timespec="seconds")
    (run / "manifest.json").write_text(json.dumps(manifest, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument("stage", choices=("build", "verify", "install",
                                          "restore"))
    parser.add_argument("--data-dir", type=Path, default=DEFAULT_DATA)
    parser.add_argument("--run-dir", type=Path, default=DEFAULT_RUN)
    args = parser.parse_args()
    {"build": build, "verify": verify, "install": install,
     "restore": restore}[args.stage](args)


if __name__ == "__main__":
    main()
