#!/usr/bin/env python3
"""Build the rough Site1 ground mesh and the WATER gap-fill cloud.

Site1 (Data/Scene1) was scanned at low tide over a rocky tidal flat, so the
SAND/ROCK/VEG clouds carry large no-return holes wherever standing water
absorbed or mirrored the laser. This pipeline reconstructs a rough ground
surface across those holes and produces three derived files next to the
canonical clouds:

  Site1-MESH.ply            screened-Poisson triangle mesh (rough, ~4 cm)
  Site1-MESHSampled-5mm.ply mesh-sampled Ground cloud; its presence is what
                            un-greys the Water tab for Scene1
                            (FindSampledGroundSurfaceInDirectory needs the
                            "meshsampled" stem, the 5mm token, zero faces,
                            normals, and Dip/Dip-direction scalars)
  Site1-WATER-5mm.ply       5 mm gap-fill points where the water stood; a
                            standalone role-less layer (the WATER token is
                            deliberately not a scene role so bundles stay
                            untouched), schema-identical to the source
                            clouds, ScanID=999 for scalar selection

Stages (resumable via --work-dir; each stage reuses prior artifacts):
  holemap  5 cm occupancy/hole grids from the three 5 mm clouds
  orient   SAND+ROCK extraction, MST normal-sign repair, water-plane seeds
  mesh     screened Poisson (depth 12) + density/region/component trims
  outputs  the three PLYs above (refuses to overwrite without --overwrite)

Notes recorded from the 2026-08-25 build:
  - CleanMesh normals are unit but sign-inconsistent (~10% flipped in
    patches); MST propagation on a stride-12 subsample + nearest-neighbour
    sign transfer + a 1 m-cell majority fixup repairs them.
  - Poisson depth 13 reproducibly dies in PoissonRecon's iso-surfacer
    ("Failed to close loop"); depth 12 is stable and adequate for a rough
    mesh. Perfectly coplanar water-plane seeds trigger the same crash, so
    seeds are jittered (+-4 cm XY, +-4 mm Z).
  - Water fill is capped at z <= 2.6 m so occlusion pockets high on the rock
    ledge stay dry, and interior fill z is smoothed (boundary-anchored) so
    the water reads calm across Poisson wobble in unsupported spans.

Requires: numpy, scipy, open3d (a venv with these; see --python-hint).
"""

from __future__ import annotations

import argparse
import datetime as _dt
import os
import sys
import time
from pathlib import Path

import numpy as np

CELL = 0.05
GRID_X0, GRID_Y0 = 732.0, 758.5
GRID_NX, GRID_NY = int(110.0 / CELL), int(162.0 / CELL)
POISSON_DEPTH_DEFAULT = 12
COMPONENT_FLOOR_M2 = 200.0
WATER_Z_CAP = 2.6
WATER_SCAN_ID = 999.0
SEED_COMPONENT_MIN_M2 = 50.0
MESH_SAMPLED_POINTS = 10_000_000


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
    return np.dtype(fields), count, offset


def memmap_cloud(path):
    dtype, count, offset = read_header(path)
    return np.memmap(path, dtype=dtype, mode="r", offset=offset, shape=(count,)), dtype


def source_paths(data_dir: Path):
    return {role: data_dir / f"Site1-{role}-5mm.ply" for role in ("SAND", "ROCK", "VEG")}


def stage_holemap(data_dir: Path, work: Path):
    out = work / "holemap.npz"
    if out.exists():
        print(f"[holemap] reuse {out}")
        return
    from scipy import ndimage

    zsum = np.zeros(GRID_NX * GRID_NY, np.float64)
    zcnt = np.zeros(GRID_NX * GRID_NY, np.int64)
    grids = {}
    for role, path in source_paths(data_dir).items():
        mm, _ = memmap_cloud(path)
        occ = np.zeros(GRID_NX * GRID_NY, np.int32)
        for i in range(0, len(mm), 8_000_000):
            c = mm[i:i + 8_000_000]
            ix = ((c["x"] - GRID_X0) / CELL).astype(np.int32)
            iy = ((c["y"] - GRID_Y0) / CELL).astype(np.int32)
            ok = (ix >= 0) & (ix < GRID_NX) & (iy >= 0) & (iy < GRID_NY)
            idx = iy[ok].astype(np.int64) * GRID_NX + ix[ok]
            np.add.at(occ, idx, 1)
            if role != "VEG":
                np.add.at(zsum, idx, c["z"][ok].astype(np.float64))
                np.add.at(zcnt, idx, 1)
        grids[role] = occ.reshape(GRID_NY, GRID_NX)
        print(f"[holemap] {role}: {len(mm):,} pts")
    anyocc = (grids["SAND"] + grids["ROCK"] + grids["VEG"]) > 0
    radius = int(2.0 / CELL)
    yy, xx = np.mgrid[-radius:radius + 1, -radius:radius + 1]
    disk = np.hypot(xx, yy) <= radius
    footprint = ndimage.binary_fill_holes(ndimage.binary_closing(anyocc, structure=disk))
    footprint &= ndimage.binary_dilation(anyocc, structure=disk, iterations=1)
    holes = footprint & ~anyocc
    labels, count = ndimage.label(holes)
    sizes = ndimage.sum_labels(np.ones_like(labels), labels, index=np.arange(1, count + 1))
    keep = np.zeros(count + 1, bool)
    keep[1:] = sizes >= (0.25 / CELL ** 2)
    zmean = np.divide(zsum, zcnt, out=np.full(GRID_NX * GRID_NY, np.nan), where=zcnt > 0)
    np.savez_compressed(
        out,
        occ_rock=grids["ROCK"], occ_sand=grids["SAND"], occ_veg=grids["VEG"],
        holes=keep[labels], footprint=footprint,
        zmean=zmean.reshape(GRID_NY, GRID_NX).astype(np.float32),
        meta=np.array([GRID_X0, GRID_Y0, CELL, GRID_NX, GRID_NY]))
    print(f"[holemap] holes >=0.25 m2: {keep[labels].sum() * CELL * CELL:.0f} m2 -> {out}")


def stage_orient(data_dir: Path, work: Path):
    out = work / "poisson_input2.npz"
    if out.exists():
        print(f"[orient] reuse {out}")
        return
    import open3d as o3d
    from scipy import ndimage
    from scipy.spatial import cKDTree

    xyz_parts, n_parts = [], []
    for role in ("SAND", "ROCK"):
        mm, _ = memmap_cloud(source_paths(data_dir)[role])
        for i in range(0, len(mm), 8_000_000):
            c = mm[i:i + 8_000_000]
            normals = np.stack([c["nx"], c["ny"], c["nz"]], 1).astype(np.float32)
            length = np.linalg.norm(normals, axis=1)
            ok = length > 0.5
            xyz_parts.append(np.stack([c["x"], c["y"], c["z"]], 1).astype(np.float32)[ok])
            n_parts.append(normals[ok] / length[ok, None])
    xyz = np.concatenate(xyz_parts)
    normals = np.concatenate(n_parts)
    del xyz_parts, n_parts
    print(f"[orient] {len(xyz):,} SAND+ROCK pts with unit normals")

    stride = 12
    sub = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz[::stride].astype(np.float64)))
    sub.normals = o3d.utility.Vector3dVector(normals[::stride].astype(np.float64))
    started = time.time()
    sub.orient_normals_consistent_tangent_plane(15)
    print(f"[orient] MST on {len(sub.points):,} pts: {time.time() - started:.0f}s")
    sub_normals = np.asarray(sub.normals, dtype=np.float32)
    horizontal = np.abs(sub_normals[:, 2]) > 0.5
    if np.median(sub_normals[horizontal, 2]) < 0:
        sub_normals = -sub_normals
    tree = cKDTree(xyz[::stride])
    for i in range(0, len(xyz), 4_000_000):
        _, nearest = tree.query(xyz[i:i + 4_000_000], k=1, workers=-1)
        flip = (normals[i:i + 4_000_000] * sub_normals[nearest]).sum(1) < 0
        normals[i:i + 4_000_000][flip] = -normals[i:i + 4_000_000][flip]
    # 1 m-cell majority fixup for inverted islands the MST graph missed.
    ix = ((xyz[:, 0] - GRID_X0) / 1.0).astype(np.int32)
    iy = ((xyz[:, 1] - GRID_Y0) / 1.0).astype(np.int32)
    key = iy.astype(np.int64) * (ix.max() + 1) + ix
    horizontal = np.abs(normals[:, 2]) > 0.5
    sums = np.bincount(key[horizontal], weights=normals[horizontal, 2])
    counts = np.bincount(key[horizontal])
    bad = np.nonzero((counts > 20) & (sums < -0.3 * counts))[0]
    if len(bad):
        mask = np.isin(key, bad)
        normals[mask] = -normals[mask]
        print(f"[orient] majority fixup flipped {mask.sum():,} pts in {len(bad)} cells")

    hm = np.load(work / "holemap.npz")
    holes, zmean = hm["holes"], hm["zmean"]
    ground = (hm["occ_rock"] + hm["occ_sand"]) > 0
    labels, count = ndimage.label(holes)
    sizes = ndimage.sum_labels(np.ones_like(labels), labels, np.arange(1, count + 1)) * CELL * CELL
    ground_near = ndimage.binary_dilation(ground, iterations=3)
    seed_xy, seed_z = [], []
    for comp in np.arange(1, count + 1)[sizes >= SEED_COMPONENT_MIN_M2]:
        component = labels == comp
        ring = ndimage.binary_dilation(component, iterations=4) & ground_near & np.isfinite(zmean)
        level = np.percentile(zmean[ring], 15) if ring.any() else np.nanpercentile(zmean, 10)
        core = ndimage.binary_erosion(component, iterations=int(1.0 / CELL))
        cy, cx = np.nonzero(core)
        pick = (cx % 3 == 0) & (cy % 3 == 0)
        seed_xy.append(np.stack([GRID_X0 + (cx[pick] + .5) * CELL,
                                 GRID_Y0 + (cy[pick] + .5) * CELL], 1))
        seed_z.append(np.full(pick.sum(), level, np.float32))
        print(f"[orient] seed comp {int(comp)}: {sizes[comp - 1]:.0f} m2 at z={level:.3f}")
    if seed_xy:
        seeds = np.column_stack([np.concatenate(seed_xy), np.concatenate(seed_z)]).astype(np.float32)
    else:
        seeds = np.zeros((0, 3), np.float32)
    seed_normals = np.zeros_like(seeds)
    if len(seeds):
        seed_normals[:, 2] = 1.0
    np.savez(out, xyz=xyz, n=normals, seeds=seeds, seed_n=seed_normals)
    print(f"[orient] saved {out}")


def stage_mesh(work: Path, depth: int):
    out = work / f"site1_mesh_d{depth}.ply"
    if out.exists():
        print(f"[mesh] reuse {out}")
        return out
    import open3d as o3d
    from scipy import ndimage

    data = np.load(work / "poisson_input2.npz")
    rng = np.random.default_rng(11)
    seeds = data["seeds"].astype(np.float64)
    if len(seeds):
        seeds[:, :2] += rng.uniform(-0.04, 0.04, (len(seeds), 2))
        seeds[:, 2] += rng.uniform(-0.004, 0.004, len(seeds))
    xyz = np.vstack([data["xyz"].astype(np.float64), seeds])
    normals = np.vstack([data["n"], data["seed_n"]]).astype(np.float64)
    finite = np.isfinite(xyz).all(1) & np.isfinite(normals).all(1)
    xyz, normals = xyz[finite], normals[finite]
    cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz))
    cloud.normals = o3d.utility.Vector3dVector(normals)
    started = time.time()
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        cloud, depth=depth, scale=1.05, linear_fit=False)
    print(f"[mesh] poisson d{depth}: {time.time() - started:.0f}s, "
          f"{len(mesh.vertices):,} verts / {len(mesh.triangles):,} tris")
    mesh.remove_vertices_by_mask(np.asarray(densities) < np.quantile(np.asarray(densities), 0.015))

    hm = np.load(work / "holemap.npz")
    ground = (hm["occ_rock"] + hm["occ_sand"]) > 0
    keep_region = ndimage.binary_dilation(ground, iterations=int(0.6 / CELL)) | hm["holes"]
    verts = np.asarray(mesh.vertices)
    tris = np.asarray(mesh.triangles)
    centers = verts[tris].mean(axis=1)
    cx = ((centers[:, 0] - GRID_X0) / CELL).astype(np.int64)
    cy = ((centers[:, 1] - GRID_Y0) / CELL).astype(np.int64)
    inside = (cx >= 0) & (cx < GRID_NX) & (cy >= 0) & (cy < GRID_NY)
    keep = np.zeros(len(tris), bool)
    keep[inside] = keep_region[cy[inside], cx[inside]]
    mesh.remove_triangles_by_mask(~keep)
    mesh.remove_unreferenced_vertices()
    mesh.remove_degenerate_triangles()
    o3d.io.write_triangle_mesh(str(out), mesh)
    print(f"[mesh] trimmed to {len(mesh.vertices):,} verts -> {out}")
    return out


def write_ply(path: Path, array: np.ndarray, comments):
    typemap = {"f4": "float", "f8": "double", "u1": "uchar"}
    lines = ["ply", "format binary_little_endian 1.0"]
    lines += [f"comment {c}" for c in comments]
    lines.append(f"element vertex {len(array)}")
    for name in array.dtype.names:
        code = array.dtype[name].str.lstrip("<>|=")
        lines.append(f"property {typemap[code]} {name}")
    lines.append("end_header")
    with open(path, "wb") as handle:
        handle.write(("\n".join(lines) + "\n").encode("ascii"))
        array.tofile(handle)


def stage_outputs(data_dir: Path, work: Path, depth: int, overwrite: bool):
    import open3d as o3d
    from scipy import ndimage
    from scipy.spatial import cKDTree

    stamp = _dt.date.today().isoformat()
    mesh_path = data_dir / "Site1-MESH.ply"
    sampled_path = data_dir / "Site1-MESHSampled-5mm.ply"
    water_path = data_dir / "Site1-WATER-5mm.ply"
    for target in (mesh_path, sampled_path, water_path):
        if target.exists() and not overwrite:
            raise SystemExit(f"{target} exists; pass --overwrite to replace it")

    mesh = o3d.io.read_triangle_mesh(str(work / f"site1_mesh_d{depth}.ply"))
    labels, count, areas = mesh.cluster_connected_triangles()
    labels = np.asarray(labels)
    small = np.nonzero(np.asarray(areas) < COMPONENT_FLOOR_M2)[0]
    if len(small):
        mesh.remove_triangles_by_mask(np.isin(labels, small))
        mesh.remove_unreferenced_vertices()
    mesh.compute_vertex_normals()
    vertex_normals = np.asarray(mesh.vertex_normals)
    flip = vertex_normals[:, 2] < 0
    vertex_normals[flip] = -vertex_normals[flip]
    mesh.vertex_normals = o3d.utility.Vector3dVector(vertex_normals)
    print(f"[outputs] mesh comps >= {COMPONENT_FLOOR_M2:.0f} m2: "
          f"{len(mesh.vertices):,} verts, area {mesh.get_surface_area():.0f} m2")

    # colours: nearest SAND/ROCK source point
    sand, _ = memmap_cloud(source_paths(data_dir)["SAND"])
    rock, _ = memmap_cloud(source_paths(data_dir)["ROCK"])
    stride = 4
    src_xyz = np.vstack([
        np.stack([sand["x"][::stride], sand["y"][::stride], sand["z"][::stride]], 1),
        np.stack([rock["x"][::stride], rock["y"][::stride], rock["z"][::stride]], 1),
    ]).astype(np.float32)
    src_rgb = np.vstack([
        np.stack([sand["red"][::stride], sand["green"][::stride], sand["blue"][::stride]], 1),
        np.stack([rock["red"][::stride], rock["green"][::stride], rock["blue"][::stride]], 1),
    ])
    tree3 = cKDTree(src_xyz)
    verts = np.asarray(mesh.vertices).astype(np.float32)
    _, nearest = tree3.query(verts, k=1, workers=-1)
    mesh.vertex_colors = o3d.utility.Vector3dVector(src_rgb[nearest].astype(np.float64) / 255.0)
    o3d.io.write_triangle_mesh(str(mesh_path), mesh)
    print(f"[outputs] wrote {mesh_path} ({mesh_path.stat().st_size / 1e6:.0f} MB)")

    sampled = mesh.sample_points_uniformly(number_of_points=MESH_SAMPLED_POINTS)
    pts = np.asarray(sampled.points, dtype=np.float32)
    nrm = np.asarray(sampled.normals, dtype=np.float32)
    flip = nrm[:, 2] < 0
    nrm[flip] = -nrm[flip]
    rgb = (np.asarray(sampled.colors) * 255.0 + 0.5).astype(np.uint8)
    dip = np.degrees(np.arccos(np.clip(nrm[:, 2], -1.0, 1.0))).astype(np.float32)
    dip_dir = (np.degrees(np.arctan2(nrm[:, 0], nrm[:, 1])) % 360.0).astype(np.float32)
    sampled_dtype = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                              ("red", "u1"), ("green", "u1"), ("blue", "u1"),
                              ("nx", "<f4"), ("ny", "<f4"), ("nz", "<f4"),
                              ("scalar_Dip_(degrees)", "<f4"),
                              ("scalar_Dip_direction_(degrees)", "<f4")])
    record = np.empty(len(pts), sampled_dtype)
    record["x"], record["y"], record["z"] = pts.T
    record["red"], record["green"], record["blue"] = rgb.T
    record["nx"], record["ny"], record["nz"] = nrm.T
    record["scalar_Dip_(degrees)"] = dip
    record["scalar_Dip_direction_(degrees)"] = dip_dir
    write_ply(sampled_path, record, [
        f"Site1 ground cloud sampled from the {stamp} screened-Poisson mesh",
        "Generated by scripts/build_site1_ground_mesh_and_water.py",
    ])
    print(f"[outputs] wrote {sampled_path} ({sampled_path.stat().st_size / 1e6:.0f} MB)")

    # ---- WATER gap fill ----
    hm = np.load(work / "holemap.npz")
    holes = hm["holes"]
    anyocc = (hm["occ_rock"] + hm["occ_sand"] + hm["occ_veg"]) > 0
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
    hole_iy, hole_ix = np.nonzero(holes)
    rays = np.zeros((len(hole_ix), 6), np.float32)
    rays[:, 0] = GRID_X0 + (hole_ix + .5) * CELL
    rays[:, 1] = GRID_Y0 + (hole_iy + .5) * CELL
    rays[:, 2] = 60.0
    rays[:, 5] = -1.0
    hit_t = scene.cast_rays(o3d.core.Tensor(rays))["t_hit"].numpy()
    zhit = 60.0 - hit_t
    water_sel = np.isfinite(hit_t) & (zhit <= WATER_Z_CAP)
    zgrid = np.full((GRID_NY, GRID_NX), np.nan, np.float32)
    zgrid[hole_iy[water_sel], hole_ix[water_sel]] = zhit[water_sel]
    water_mask = np.zeros((GRID_NY, GRID_NX), bool)
    water_mask[hole_iy[water_sel], hole_ix[water_sel]] = True
    print(f"[outputs] water cells: {water_mask.sum():,} ({water_mask.sum() * CELL * CELL:.0f} m2)")

    # boundary ring z from the mesh, then boundary-anchored smoothing
    ring = ndimage.binary_dilation(water_mask, iterations=3) & ~water_mask
    ring_iy, ring_ix = np.nonzero(ring)
    rays = np.zeros((len(ring_ix), 6), np.float32)
    rays[:, 0] = GRID_X0 + (ring_ix + .5) * CELL
    rays[:, 1] = GRID_Y0 + (ring_iy + .5) * CELL
    rays[:, 2] = 60.0
    rays[:, 5] = -1.0
    ring_t = scene.cast_rays(o3d.core.Tensor(rays))["t_hit"].numpy()
    ring_ok = np.isfinite(ring_t)
    ring_z = np.full((GRID_NY, GRID_NX), np.nan, np.float32)
    ring_z[ring_iy[ring_ok], ring_ix[ring_ok]] = 60.0 - ring_t[ring_ok]
    # Where a rim cell holds real SAND/ROCK points, the measured mean height
    # is the truth; the Poisson mesh wobbles on sparse grazing returns (the
    # scanner starburst zones), and a noisy Dirichlet rim would ripple the
    # whole sheet. The raycast height remains only for data-free rims such as
    # the seaward mesh edge.
    measured = hm["zmean"][ring_iy, ring_ix]
    use_measured = np.isfinite(measured)
    ring_z[ring_iy[use_measured], ring_ix[use_measured]] = measured[use_measured]

    # Harmonic (Laplace) interior: standing water is calm, so interior z is
    # solved from the rim heights alone. A harmonic surface has no interior
    # extrema, which removes the Poisson wobble that a local smoothing of the
    # mesh keeps (chop in streak gaps, humps mid-bay). Rim cells act as
    # Dirichlet boundaries; footprint edges without mesh support fall back to
    # natural (zero-flux) boundaries.
    from scipy.sparse import coo_matrix
    from scipy.sparse.linalg import spsolve

    unknown_index = np.full((GRID_NY, GRID_NX), -1, np.int64)
    unknown_index[water_mask] = np.arange(water_mask.sum())
    dirichlet = ~water_mask & np.isfinite(ring_z)
    wy_all, wx_all = np.nonzero(water_mask)

    # Rim heights are winsorized per connected water component before they act
    # as Dirichlet boundaries: a small data island poking through a sheet must
    # not tent the water surface upward, and a steep Poisson wall in a narrow
    # streak gap must not drag it down. Water laps such rims instead.
    comp_labels, _ = ndimage.label(water_mask)
    comp_of_unknown = comp_labels[wy_all, wx_all]
    rim_comp, rim_value = [], []
    for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        ny_, nx_ = wy_all + dy, wx_all + dx
        inside = (ny_ >= 0) & (ny_ < GRID_NY) & (nx_ >= 0) & (nx_ < GRID_NX)
        fixed = dirichlet[ny_[inside], nx_[inside]]
        rim_comp.append(comp_of_unknown[inside][fixed])
        rim_value.append(ring_z[ny_[inside][fixed], nx_[inside][fixed]])
    rim_comp = np.concatenate(rim_comp); rim_value = np.concatenate(rim_value)
    caps = {}
    for comp in np.unique(rim_comp):
        values = rim_value[rim_comp == comp]
        caps[comp] = (np.percentile(values, 60.0), np.percentile(values, 5.0))

    rows, cols, vals = [], [], []
    rhs = np.zeros(water_mask.sum())
    for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        ny_, nx_ = wy_all + dy, wx_all + dx
        inside = (ny_ >= 0) & (ny_ < GRID_NY) & (nx_ >= 0) & (nx_ < GRID_NX)
        this = unknown_index[wy_all[inside], wx_all[inside]]
        neigh_unknown = unknown_index[ny_[inside], nx_[inside]]
        neigh_dirichlet = dirichlet[ny_[inside], nx_[inside]]
        connected = (neigh_unknown >= 0) | neigh_dirichlet
        rows.append(this[connected]); cols.append(this[connected])
        vals.append(np.ones(connected.sum()))
        interior_link = neigh_unknown >= 0
        rows.append(this[interior_link]); cols.append(neigh_unknown[interior_link])
        vals.append(-np.ones(interior_link.sum()))
        fixed = neigh_dirichlet & ~interior_link
        fixed_values = ring_z[ny_[inside][fixed], nx_[inside][fixed]].astype(np.float64)
        fixed_comps = comp_of_unknown[inside][fixed]
        high = np.array([caps[c][0] for c in fixed_comps])
        low = np.array([caps[c][1] for c in fixed_comps]) - 0.10
        np.add.at(rhs, this[fixed], np.clip(fixed_values, low, high))
    from scipy.sparse import identity
    laplacian = coo_matrix(
        (np.concatenate(vals), (np.concatenate(rows), np.concatenate(cols))),
        shape=(len(rhs), len(rhs))).tocsr()
    # weak pull toward the mesh height removes singular all-Neumann components
    epsilon = 1e-6
    laplacian = laplacian + epsilon * identity(len(rhs), format="csr")
    rhs = rhs + epsilon * np.nan_to_num(zgrid[wy_all, wx_all], nan=WATER_Z_CAP)
    solved = spsolve(laplacian, rhs)
    harmonic = np.full((GRID_NY, GRID_NX), np.nan, np.float32)
    harmonic[wy_all, wx_all] = solved.astype(np.float32)
    interior = ndimage.distance_transform_edt(water_mask, sampling=CELL)
    alpha = np.clip(interior / 0.15, 0.0, 1.0)
    blend = (1.0 - alpha) * np.nan_to_num(zgrid) + alpha * harmonic
    # The blend seats water into the real beach edges, but may only deviate a
    # few centimetres from the calm harmonic sheet; this removes the mesh chop
    # in narrow streak gaps and any tenting over data islands.
    zfinal = np.where(water_mask,
                      np.clip(blend, harmonic - 0.06, harmonic + 0.06),
                      np.nan)

    rng = np.random.default_rng(23)
    wy, wx = np.nonzero(water_mask)
    per_cell = 10
    offsets = (np.arange(per_cell) + 0.5) / per_cell
    ox, oy = np.meshgrid(offsets, offsets)
    ox, oy = ox.ravel(), oy.ravel()
    px = (GRID_X0 + (wx[:, None] + ox[None, :]) * CELL).ravel().astype(np.float32)
    py = (GRID_Y0 + (wy[:, None] + oy[None, :]) * CELL).ravel().astype(np.float32)
    px += rng.uniform(-0.0015, 0.0015, len(px)).astype(np.float32)
    py += rng.uniform(-0.0015, 0.0015, len(py)).astype(np.float32)
    cell_alpha = np.repeat(alpha[wy, wx], per_cell * per_cell)
    gx = np.clip(((px - GRID_X0) / CELL).astype(np.int64), 0, GRID_NX - 1)
    gy = np.clip(((py - GRID_Y0) / CELL).astype(np.int64), 0, GRID_NY - 1)
    pz = zfinal[gy, gx].astype(np.float32)
    keep = np.isfinite(pz)
    px, py, pz, cell_alpha = px[keep], py[keep], pz[keep], cell_alpha[keep]
    pz += rng.normal(0.0, 0.0005, len(pz)).astype(np.float32)

    # drop generated points closer than 4 mm (XY) to a real point, near edges only
    near_edge = cell_alpha < 1.0
    if near_edge.any():
        edge_cells = ndimage.binary_dilation(water_mask, iterations=1) & anyocc
        keep_xy = []
        for role, path in source_paths(data_dir).items():
            mm, _ = memmap_cloud(path)
            for i in range(0, len(mm), 8_000_000):
                c = mm[i:i + 8_000_000]
                sx = ((c["x"] - GRID_X0) / CELL).astype(np.int64)
                sy = ((c["y"] - GRID_Y0) / CELL).astype(np.int64)
                ok = (sx >= 0) & (sx < GRID_NX) & (sy >= 0) & (sy < GRID_NY)
                ok[ok] = edge_cells[sy[ok], sx[ok]]
                if ok.any():
                    keep_xy.append(np.stack([c["x"][ok], c["y"][ok]], 1).astype(np.float32))
        if keep_xy:
            edge_tree = cKDTree(np.vstack(keep_xy))
            dist, _ = edge_tree.query(np.stack([px[near_edge], py[near_edge]], 1),
                                      k=1, workers=-1, distance_upper_bound=0.004)
            collide = np.zeros(len(px), bool)
            collide[np.nonzero(near_edge)[0][np.isfinite(dist)]] = True
            px, py, pz = px[~collide], py[~collide], pz[~collide]
            print(f"[outputs] edge rejection dropped {collide.sum():,} pts")

    print(f"[outputs] water points: {len(px):,}")

    # attributes by 2D IDW from SAND+ROCK
    src_xy = src_xyz[:, :2]
    tree2 = cKDTree(src_xy)
    source_fields = ["red", "green", "blue", "scalar_Intensity", "scalar_Composite",
                     "scalar_A_R_Shelter_Lower", "scalar_A_R_RainExposure_Lower",
                     "scalar_A_R_SVF_Lower"]
    stacked = {name: np.concatenate([sand[name][::stride], rock[name][::stride]]).astype(np.float64)
               for name in source_fields}
    water_dtype, _, _ = read_header(source_paths(data_dir)["SAND"])
    record = np.zeros(len(px), water_dtype)
    record["x"], record["y"], record["z"] = px, py, pz
    record["nz"] = 1.0
    record["scalar_ScanID"] = WATER_SCAN_ID
    record["scalar_A_R_Horizontalness"] = 1.0
    for start in range(0, len(px), 4_000_000):
        stop = min(start + 4_000_000, len(px))
        dist, nearest = tree2.query(np.stack([px[start:stop], py[start:stop]], 1),
                                    k=6, workers=-1)
        wgt = 1.0 / np.maximum(dist, 0.01) ** 2
        wgt /= wgt.sum(axis=1, keepdims=True)
        for name in source_fields:
            value = (stacked[name][nearest] * wgt).sum(axis=1)
            if record.dtype[name].kind == "u":
                record[name][start:stop] = np.clip(value + 0.5, 0, 255).astype(np.uint8)
            else:
                record[name][start:stop] = value.astype(record.dtype[name].base)
    write_ply(water_path, record, [
        f"Site1 water gap fill generated {stamp} from the screened-Poisson ground mesh",
        "Points fill laser no-return holes (standing water at scan time)",
        f"5 mm jittered grid, z capped at {WATER_Z_CAP} m, ScanID={WATER_SCAN_ID:.0f}",
        "Generated by scripts/build_site1_ground_mesh_and_water.py",
    ])
    print(f"[outputs] wrote {water_path} ({water_path.stat().st_size / 1e9:.2f} GB)")


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("stage", choices=["holemap", "orient", "mesh", "outputs", "all"])
    parser.add_argument("--data-dir", type=Path, default=Path("Data/Scene1"))
    parser.add_argument("--work-dir", type=Path, required=True,
                        help="directory for resumable intermediates (holemap.npz, "
                             "poisson_input2.npz, site1_mesh_d<depth>.ply)")
    parser.add_argument("--depth", type=int, default=POISSON_DEPTH_DEFAULT)
    parser.add_argument("--overwrite", action="store_true",
                        help="allow the outputs stage to replace existing Data files")
    args = parser.parse_args()
    args.work_dir.mkdir(parents=True, exist_ok=True)
    if args.stage in ("holemap", "all"):
        stage_holemap(args.data_dir, args.work_dir)
    if args.stage in ("orient", "all"):
        stage_orient(args.data_dir, args.work_dir)
    if args.stage in ("mesh", "all"):
        stage_mesh(args.work_dir, args.depth)
    if args.stage in ("outputs", "all"):
        stage_outputs(args.data_dir, args.work_dir, args.depth, args.overwrite)


if __name__ == "__main__":
    main()
