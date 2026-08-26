#!/usr/bin/env python3
"""Build blue-noise Scene1/Fossils WATER clouds from the verified v9 sheet.

v10 deliberately keeps the successful v9 hydraulic height solution and
changes only the sub-cell footprint, sampling, and micro-texture.  A dense
oversample is greedily Poisson-thinned together with the measured terrain,
with terrain selected first.  The resulting WATER points therefore fill
individual gaps rather than fixed 25 mm quota squares.  The dense result is
analysed at approximately 2 mm spacing; the 5 mm result is a second greedy
selection against the authored 5 mm SAND/ROCK clouds and retains the dense
analysis values.

The dense geometry is analysed by CleanMesh's real reduced-analysis engine.
Undefined edge values are repaired within WATER, environmental fields are
sampled from the surrounding scene, and the four combined fields use the
same ROCK-derived normalization as the terrain rather than a WATER-only
histogram.  Build and verify are staged below PatchRefinement.  Install only
replaces the canonical 5 mm WATER cloud and preserves the previous file as
``Site1-WATER-5mm-old01.ply``; the dense candidate remains an explicit review
asset unless ``--install-dense`` is passed.
"""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import math
import os
import shutil
import subprocess
import sys
from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Mapping

import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))
import rebuild_site1_fossils_water as v6  # noqa: E402
import rebuild_site1_fossils_v8 as v8  # noqa: E402
import rebuild_site1_fossils_v9 as v9  # noqa: E402
import site1_scalar_fill as scalar_fill  # noqa: E402


ROOT = SCRIPT_DIR.parent
DEFAULT_DATA = ROOT / "Data/Scene1"
DEFAULT_V9_RUN = DEFAULT_DATA / "PatchRefinement/20260826-water-v9-connected"
DEFAULT_RUN = DEFAULT_DATA / "PatchRefinement/20260826-water-v10-blue-noise"
DEFAULT_CONFIG = SCRIPT_DIR / "config/site1_fossils_v10_review.json"
DEFAULT_CLEANMESH = Path(
    "/Users/juju/Documents/Repositories/CleanMesh/build-release/"
    "cleanmesh_reduced_analysis"
)
DEFAULT_DOWNSAMPLE = Path(
    "/Users/juju/Documents/Repositories/CleanMesh/build-release/"
    "cleanmesh_spatial_downsample"
)

WATER_SCAN_ID = 999.0
BASE_PRIORITY_SCAN_ID = -987654.0
CHUNK = 2_000_000
DENSE_SPACING = 0.002
COARSE_SPACING = 0.005
OVERSAMPLE_PITCH = 0.00135
GENERATION_TILE_M = 0.40
BASE_Z_BELOW_M = 0.015
BASE_Z_ABOVE_M = 0.025
TERRAIN_ANALYSIS_COLLAR_M = 0.35
FOOTPRINT_SMOOTH_SIGMA_CELLS = 0.85
COMBINED_WEIGHTS = np.array([0.45, 0.35, 0.20], np.float64)
BUILD_STATE_SCHEMA = 1
ALGORITHM_ID = "site1-water-v10-blue-noise-cleanmesh-v2"

# The calibrated texture uses one continuous rotated gradient-noise field per
# coherent octave and a decorrelated 18-wave Gabor micrograin band.  Values
# below are RMS per band, not peaks.  The fixed native RMS converts classic
# 2-D gradient noise to those physical amplitudes without normalising each
# pool independently (which would make seams and change the surface on edits).
NOISE_OCTAVE_WAVELENGTHS_M = np.array(
    [0.018, 0.030, 0.050, 0.084, 0.142, 0.240, 0.406], np.float64
)
NOISE_OCTAVE_RMS_M = np.array(
    [0.00040, 0.00065, 0.00090, 0.00100,
     0.00090, 0.00065, 0.00035], np.float64
)
NOISE_OCTAVE_SEEDS = np.array(
    [71, 137, 211, 293, 379, 463, 557], np.int64
)
NOISE_OCTAVE_ANGLES_RAD = np.array(
    [0.17, 1.11, 2.37, 0.73, 1.91, 2.79, 0.41], np.float64
)
NOISE_PERLIN_NATIVE_RMS = 0.21575
NOISE_MICROGRAIN_RMS_M = 0.00060
NOISE_MICROGRAIN_COMPONENTS = 18
NOISE_MICROGRAIN_SEED = 98473
NOISE_SOFT_CLIP_START_M = 0.0070
NOISE_SOFT_CLIP_LIMIT_M = 0.0089
SHORE_NOISE_FLOOR = 0.45
SHORE_NOISE_RAMP_M = 0.0625
NOISE_UNCLIPPED_RMS_M = float(np.sqrt(
    np.sum(NOISE_OCTAVE_RMS_M**2) + NOISE_MICROGRAIN_RMS_M**2
))


def _micrograin_spectrum() -> tuple[np.ndarray, ...]:
    rng = np.random.default_rng(NOISE_MICROGRAIN_SEED)
    count = NOISE_MICROGRAIN_COMPONENTS
    wavelengths = np.exp(rng.uniform(
        math.log(0.016), math.log(0.026), count
    ))
    amplitudes = np.full(
        count, NOISE_MICROGRAIN_RMS_M * math.sqrt(2.0 / count)
    )
    angles = rng.uniform(0.0, math.pi, count)
    phases = rng.uniform(0.0, 2.0 * math.pi, count)
    return tuple(np.asarray(values, np.float64) for values in (
        wavelengths, amplitudes, angles, phases
    ))


(
    MICROGRAIN_WAVELENGTHS_M,
    MICROGRAIN_AMPLITUDES_M,
    MICROGRAIN_ANGLES_RAD,
    MICROGRAIN_PHASE_RAD,
) = _micrograin_spectrum()

PHYSICAL_METRICS = (
    "MeanCurvature", "CrossCurvature", "Recession", "Roughness"
)
SCALES = ("Fine", "Medium", "Broad")
ENV_FIELDS = (
    "scalar_A_R_Shelter_Lower",
    "scalar_A_R_RainExposure_Lower",
    "scalar_A_R_SVF_Lower",
)


def sha256_path(path: Path, block_size: int = 32 * 1024 * 1024) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        while block := handle.read(block_size):
            digest.update(block)
    return digest.hexdigest()


def _stat_identity(stat: os.stat_result) -> tuple[int, int, int, int]:
    return (stat.st_dev, stat.st_ino, stat.st_size, stat.st_mtime_ns)


def source_fingerprint(path: Path) -> dict:
    before = path.stat()
    info = scalar_fill.inspect_fixed_stride_ply(path)
    digest = sha256_path(path)
    after = path.stat()
    if _stat_identity(before) != _stat_identity(after):
        raise RuntimeError(f"source changed while hashing: {path}")
    result = {
        "path": str(path),
        "points": info.count,
        "bytes": after.st_size,
        "mtime_ns": after.st_mtime_ns,
        "sha256": digest,
        "record_stride_bytes": info.dtype.itemsize,
    }
    return result


def file_fingerprint(path: Path) -> dict:
    before = path.stat()
    digest = sha256_path(path)
    result = {
        "path": str(path),
        "bytes": before.st_size,
        "sha256": digest,
    }
    if path.suffix.lower() == ".ply":
        info = scalar_fill.inspect_fixed_stride_ply(path)
        result["points"] = info.count
        result["record_stride_bytes"] = info.dtype.itemsize
    after = path.stat()
    if _stat_identity(before) != _stat_identity(after):
        raise RuntimeError(f"file changed while fingerprinting: {path}")
    return result


def _json_sha256(value: Mapping) -> str:
    payload = json.dumps(
        value, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _atomic_write_json(path: Path, value: Mapping) -> None:
    temporary = path.with_name(
        f".{path.name}.{os.getpid()}.partial"
    )
    temporary.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n")
    os.replace(temporary, path)


def build_inputs(args, v9_candidate: Path) -> dict:
    """Hash every immutable input before any expensive stage is trusted."""

    terrain = {
        f"{role}-{spacing}": source_fingerprint(
            args.data_dir / f"Site1-{role}-{spacing}.ply"
        )
        for role in ("SAND", "ROCK")
        for spacing in ("1mm", "5mm")
    }
    config_fingerprint = (
        file_fingerprint(args.config)
        if args.config.exists()
        else {"path": str(args.config), "exists": False, "sha256": None}
    )
    import scipy

    details = {
        "algorithm": ALGORITHM_ID,
        "v9_candidate": source_fingerprint(v9_candidate),
        "v9_surface": file_fingerprint(args.v9_run / "surface-v9.npz"),
        "source_water": source_fingerprint(
            args.data_dir / "Site1-WATER-5mm.ply"
        ),
        "terrain_sources": terrain,
        "config": config_fingerprint,
        "parameters": {
            "dense_spacing_m": DENSE_SPACING,
            "coarse_spacing_m": COARSE_SPACING,
            "oversample_pitch_m": OVERSAMPLE_PITCH,
            "noise_scale": float(args.noise_scale),
            "noise_octave_wavelengths_m": (
                NOISE_OCTAVE_WAVELENGTHS_M.tolist()
            ),
            "noise_octave_rms_m": NOISE_OCTAVE_RMS_M.tolist(),
            "noise_octave_seeds": NOISE_OCTAVE_SEEDS.tolist(),
            "noise_octave_angles_rad": NOISE_OCTAVE_ANGLES_RAD.tolist(),
            "noise_perlin_native_rms": NOISE_PERLIN_NATIVE_RMS,
            "micrograin_wavelengths_m": (
                MICROGRAIN_WAVELENGTHS_M.tolist()
            ),
            "micrograin_peak_amplitudes_m": (
                MICROGRAIN_AMPLITUDES_M.tolist()
            ),
            "micrograin_angles_rad": MICROGRAIN_ANGLES_RAD.tolist(),
            "micrograin_phases_rad": MICROGRAIN_PHASE_RAD.tolist(),
            "noise_soft_clip_start_m": NOISE_SOFT_CLIP_START_M,
            "noise_soft_clip_limit_m": NOISE_SOFT_CLIP_LIMIT_M,
            "shore_noise_floor": SHORE_NOISE_FLOOR,
            "shore_noise_ramp_m": SHORE_NOISE_RAMP_M,
        },
        "executables": {
            "downsample": file_fingerprint(args.downsample),
            "reduced_analysis": file_fingerprint(args.cleanmesh),
        },
        "implementation": {
            "v10": file_fingerprint(Path(__file__).resolve()),
            "v6": file_fingerprint(Path(v6.__file__).resolve()),
            "v8": file_fingerprint(Path(v8.__file__).resolve()),
            "v9": file_fingerprint(Path(v9.__file__).resolve()),
            "scalar_fill": file_fingerprint(
                Path(scalar_fill.__file__).resolve()
            ),
        },
        "runtime": {
            "python": sys.version,
            "numpy": np.__version__,
            "scipy": scipy.__version__,
        },
    }
    return details


@contextmanager
def build_lock(run_dir: Path):
    """Kernel-held lock: stale files are harmless after a crashed process."""

    import fcntl

    run_dir.mkdir(parents=True, exist_ok=True)
    path = run_dir / ".v10-build.lock"
    with open(path, "a+") as handle:
        try:
            fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            handle.seek(0)
            raise RuntimeError(
                "another v10 build/verify/install operation holds the lock: "
                + handle.read().strip()
            ) from error
        handle.seek(0)
        handle.truncate()
        handle.write(
            f"pid={os.getpid()} created="
            f"{dt.datetime.now().isoformat(timespec='seconds')}\n"
        )
        handle.flush()
        try:
            yield
        finally:
            fcntl.flock(handle.fileno(), fcntl.LOCK_UN)


def _clone_or_copy_file(source: Path, destination: Path, overwrite: bool) -> str:
    if destination.exists():
        if not overwrite:
            raise FileExistsError(f"snapshot already exists: {destination}")
        destination.unlink()
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_name(
        f".{destination.name}.{os.getpid()}.partial"
    )
    temporary.unlink(missing_ok=True)
    method = "copy"
    completed = subprocess.run(
        ["cp", "-c", str(source), str(temporary)],
        check=False,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    if completed.returncode == 0:
        method = "apfs-clone"
    else:
        shutil.copy2(source, temporary)
    os.replace(temporary, destination)
    return method


def _assert_fingerprint_content(
    actual: Mapping,
    expected: Mapping,
    label: str,
) -> None:
    keys = ["bytes", "sha256"]
    if "points" in expected:
        keys.extend(("points", "record_stride_bytes"))
    mismatched = [
        key for key in keys if actual.get(key) != expected.get(key)
    ]
    if mismatched:
        raise RuntimeError(
            f"{label} differs from the hash-locked input in: "
            + ", ".join(mismatched)
        )


def create_source_snapshots(
    args,
    v9_candidate: Path,
    inputs: Mapping,
    overwrite: bool,
) -> dict:
    root = args.run_dir / "source-snapshots"
    data_dir = root / "DataScene1"
    v9_run = root / "v9"
    methods = {}
    fingerprints = {}
    for role in ("SAND", "ROCK"):
        for spacing in ("1mm", "5mm"):
            name = f"Site1-{role}-{spacing}.ply"
            destination = data_dir / name
            methods[name] = _clone_or_copy_file(
                args.data_dir / name, destination, overwrite
            )
            fingerprints[name] = file_fingerprint(destination)
            _assert_fingerprint_content(
                fingerprints[name],
                inputs["terrain_sources"][f"{role}-{spacing}"],
                f"snapshot {name}",
            )
    water_name = "Site1-WATER-5mm.ply"
    water_destination = data_dir / water_name
    methods[water_name] = _clone_or_copy_file(
        args.data_dir / water_name, water_destination, overwrite
    )
    fingerprints[water_name] = file_fingerprint(water_destination)
    _assert_fingerprint_content(
        fingerprints[water_name], inputs["source_water"],
        f"snapshot {water_name}",
    )
    surface_destination = v9_run / "surface-v9.npz"
    methods["surface-v9.npz"] = _clone_or_copy_file(
        args.v9_run / "surface-v9.npz",
        surface_destination,
        overwrite,
    )
    fingerprints["surface-v9.npz"] = file_fingerprint(surface_destination)
    _assert_fingerprint_content(
        fingerprints["surface-v9.npz"], inputs["v9_surface"],
        "snapshot surface-v9.npz",
    )
    candidate_destination = v9_run / v9_candidate.name
    methods["v9-candidate"] = _clone_or_copy_file(
        v9_candidate, candidate_destination, overwrite
    )
    fingerprints["v9-candidate"] = file_fingerprint(candidate_destination)
    _assert_fingerprint_content(
        fingerprints["v9-candidate"], inputs["v9_candidate"],
        "snapshot v9 candidate",
    )
    config_path = root / "site1_fossils_v10_review.json"
    if args.config.exists():
        methods["config"] = _clone_or_copy_file(
            args.config, config_path, overwrite
        )
        fingerprints["config"] = file_fingerprint(config_path)
        _assert_fingerprint_content(
            fingerprints["config"], inputs["config"], "snapshot config"
        )
    else:
        if config_path.exists() and not overwrite:
            raise FileExistsError(f"snapshot already exists: {config_path}")
        _atomic_write_json(config_path, load_config(args.config))
        methods["config"] = "generated-default"
        fingerprints["config"] = file_fingerprint(config_path)
    return {
        "root": str(root),
        "data_dir": str(data_dir),
        "v9_run": str(v9_run),
        "v9_candidate": str(v9_run / v9_candidate.name),
        "config": str(config_path),
        "methods": methods,
        "fingerprints": fingerprints,
    }


def initialise_build_state(args, inputs: Mapping) -> tuple[Path, dict]:
    path = args.run_dir / "build-state.json"
    signature = _json_sha256(inputs)
    if path.exists() and args.resume:
        state = json.loads(path.read_text())
        if state.get("schema_version") != BUILD_STATE_SCHEMA:
            raise RuntimeError("v10 build-state schema mismatch")
        if state.get("input_signature") != signature:
            raise RuntimeError(
                "v10 inputs changed; refusing to reuse prior build stages"
            )
        return path, state
    if path.exists() and not args.overwrite:
        raise FileExistsError(
            f"{path} exists; use --resume or explicitly pass --overwrite"
        )
    state = {
        "schema_version": BUILD_STATE_SCHEMA,
        "input_signature": signature,
        "inputs": inputs,
        "stages": {},
    }
    _atomic_write_json(path, state)
    return path, state


def reusable_stage(
    state: Mapping,
    name: str,
    expected_paths: Iterable[Path],
) -> bool:
    stage = state.get("stages", {}).get(name)
    if not stage:
        return False
    recorded = {item["path"]: item for item in stage.get("outputs", [])}
    for path in expected_paths:
        expected = recorded.get(str(path))
        if expected is None or not path.exists():
            return False
        stat = path.stat()
        if stat.st_size != expected.get("bytes"):
            return False
        if sha256_path(path) != expected.get("sha256"):
            return False
        if path.suffix.lower() == ".ply":
            info = scalar_fill.inspect_fixed_stride_ply(path)
            if info.count != expected.get("points"):
                return False
            if info.dtype.itemsize != expected.get("record_stride_bytes"):
                return False
    return True


def record_stage(
    state_path: Path,
    state: dict,
    name: str,
    output_paths: Iterable[Path],
    detail: Mapping | None = None,
) -> None:
    state["stages"][name] = {
        "completed": dt.datetime.now().isoformat(timespec="seconds"),
        "outputs": [file_fingerprint(path) for path in output_paths],
        "detail": dict(detail or {}),
    }
    _atomic_write_json(state_path, state)


def load_config(path: Path) -> dict:
    if not path.exists():
        return {"version": 1, "add_water_polygons": [],
                "remove_water_polygons": [], "marked_locations": {}}
    return json.loads(path.read_text())


@dataclass
class SurfaceReference:
    grid: v6.GridSpec
    wet: np.ndarray
    signed_cells: np.ndarray
    z: np.ndarray
    dzdx: np.ndarray
    dzdy: np.ndarray
    signed_dx: np.ndarray
    signed_dy: np.ndarray
    noise_mean: np.ndarray
    review_additions: np.ndarray
    exclusion: np.ndarray


def _polygon_mask(polygons, grid: v6.GridSpec) -> np.ndarray:
    if not polygons:
        return np.zeros(grid.shape, bool)
    return v6.rasterise_polygons(polygons, grid)


def _harmonic_review_extension(
    values: np.ndarray,
    fixed: np.ndarray,
    additions: np.ndarray,
) -> np.ndarray:
    """Smoothly extend fixed sheet heights through reviewed added cells."""

    from scipy.sparse import coo_matrix
    from scipy.sparse.linalg import spsolve

    additions = additions.copy()
    fixed = fixed.copy()
    values = values.copy()
    rows, cols = np.nonzero(additions)
    if not len(rows):
        return values
    # Every free component needs a Dirichlet neighbour. Without this check a
    # disconnected reviewed island has a singular Laplacian and spsolve can
    # return an arbitrary finite-looking level.
    from scipy import ndimage

    connectivity = np.array(
        [[0, 1, 0], [1, 1, 1], [0, 1, 0]], np.uint8
    )
    labels, component_count = ndimage.label(
        additions, structure=connectivity
    )
    fixed_neighbour = ndimage.binary_dilation(
        fixed, structure=connectivity
    )
    anchored = np.unique(labels[additions & fixed_neighbour])
    anchored = anchored[anchored != 0]
    missing = sorted(set(range(1, component_count + 1)) - set(anchored))
    if missing:
        if not np.any(fixed):
            raise RuntimeError(
                "cannot anchor reviewed WATER because no fixed WATER exists"
            )
        distance, nearest = ndimage.distance_transform_edt(
            ~fixed, return_indices=True
        )
        for component in missing:
            component_rows, component_cols = np.nonzero(labels == component)
            nearest_member = int(np.argmin(
                distance[component_rows, component_cols]
            ))
            row = int(component_rows[nearest_member])
            col = int(component_cols[nearest_member])
            source_row = int(nearest[0, row, col])
            source_col = int(nearest[1, row, col])
            values[row, col] = values[source_row, source_col]
            fixed[row, col] = True
            additions[row, col] = False
        rows, cols = np.nonzero(additions)
        if not len(rows):
            return values
    index = np.full(additions.shape, -1, np.int32)
    index[rows, cols] = np.arange(len(rows), dtype=np.int32)
    domain = fixed | additions
    matrix_row = []
    matrix_col = []
    matrix_value = []
    rhs = np.zeros(len(rows), np.float64)
    for equation, (row, col) in enumerate(zip(rows, cols)):
        degree = 0
        for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            rr, cc = row + dr, col + dc
            if not (0 <= rr < domain.shape[0] and 0 <= cc < domain.shape[1]):
                continue
            if not domain[rr, cc]:
                continue
            degree += 1
            neighbour = index[rr, cc]
            if neighbour >= 0:
                matrix_row.append(equation)
                matrix_col.append(int(neighbour))
                matrix_value.append(-1.0)
            else:
                rhs[equation] += float(values[rr, cc])
        if degree == 0:
            degree = 1
            rhs[equation] = float(values[row, col])
        matrix_row.append(equation)
        matrix_col.append(equation)
        matrix_value.append(float(degree))
    matrix = coo_matrix(
        (matrix_value, (matrix_row, matrix_col)),
        shape=(len(rows), len(rows)),
    ).tocsr()
    solved = np.asarray(spsolve(matrix, rhs), np.float64)
    finite = np.isfinite(solved)
    result = values.copy()
    result[rows[finite], cols[finite]] = solved[finite].astype(result.dtype)
    return result


def load_surface_reference(v9_run: Path, config: dict) -> SurfaceReference:
    from scipy import ndimage

    saved = np.load(v9_run / "surface-v9.npz")
    x0, y0, x1, y1, cell = map(float, saved["meta"])
    grid = v6.GridSpec(x0, y0, x1, y1, cell)
    original_wet = saved["wet"].astype(bool)
    wet = original_wet.copy()
    wet |= _polygon_mask(config.get("add_water_polygons", []), grid)
    terrain_count = saved["terrain_count"]
    blocked_measured_additions = np.zeros(grid.shape, bool)
    distance_to_wet_m = (
        ndimage.distance_transform_edt(~original_wet) * grid.cell
    )
    ys = grid.y0 + (np.arange(grid.ny) + 0.5) * grid.cell
    for region in config.get("add_water_regions", []):
        region_mask = _polygon_mask([region["polygon"]], grid)
        maximum = region.get("max_terrain_count")
        if maximum is not None:
            blocked_measured_additions |= (
                region_mask & ~original_wet &
                (terrain_count > int(maximum))
            )
            region_mask &= terrain_count <= int(maximum)
        maximum_distance = region.get("max_distance_to_existing_wet_m")
        if maximum_distance is not None:
            region_mask &= distance_to_wet_m <= float(maximum_distance)
        if region.get("y_min") is not None:
            region_mask &= ys[:, None] >= float(region["y_min"])
        if region.get("y_max") is not None:
            region_mask &= ys[:, None] <= float(region["y_max"])
        wet |= region_mask
    wet &= ~_polygon_mask(config.get("remove_water_polygons", []), grid)
    exclusion = saved["exclusion"].astype(bool).copy()
    exclusion |= blocked_measured_additions
    exclusion |= _polygon_mask(config.get("remove_water_polygons", []), grid)
    wet &= ~exclusion

    # v9's archived surface is finite over the full grid (nearest-filled
    # outside the original mask), so small reviewed additions inherit the
    # adjacent connected level without rerunning the hydraulic solve.
    # Replace v9's old coarse-grid ripple while preserving its successful
    # hydraulic/base solve.  Nearest extension supplies stable derivatives at
    # the shoreline and seed values for the reviewed harmonic additions.
    base = saved["base_surface"].astype(np.float32)
    finite_base = np.isfinite(base)
    if not np.any(finite_base):
        raise RuntimeError("v9 base surface contains no finite values")
    if np.all(finite_base):
        z = base.copy()
    else:
        _, nearest = ndimage.distance_transform_edt(
            ~finite_base, return_indices=True
        )
        z = base[tuple(nearest)].astype(np.float32)
    reviewed_topology = wet.copy()
    z = _harmonic_review_extension(
        z,
        original_wet & reviewed_topology,
        reviewed_topology & ~original_wet,
    )
    inside = ndimage.distance_transform_edt(wet).astype(np.float32)
    outside = ndimage.distance_transform_edt(~wet).astype(np.float32)
    signed = inside - outside
    # Smooth only the sub-cell boundary coordinate, not the water elevations.
    # This removes the visible 25 mm staircase inherited from v9's quota grid
    # while keeping the same topology and explicit strict exclusions.
    signed = ndimage.gaussian_filter(
        signed, FOOTPRINT_SMOOTH_SIGMA_CELLS, mode="nearest"
    )
    # Preserve every reviewed cell's inside/outside classification.  Only the
    # interpolated sub-cell crossing moves; narrow pools cannot vanish and the
    # smoothed field cannot leak into a measured island.
    signed[reviewed_topology] = np.maximum(
        signed[reviewed_topology], 0.05
    )
    signed[~reviewed_topology] = np.minimum(
        signed[~reviewed_topology], -0.05
    )
    wet = reviewed_topology
    dzdy, dzdx = np.gradient(z.astype(np.float64), grid.cell)
    signed_dy, signed_dx = np.gradient(
        signed.astype(np.float64) * grid.cell, grid.cell
    )
    labels, component_count = ndimage.label(
        wet, structure=np.ones((3, 3), bool)
    )
    noise_mean = np.zeros(grid.shape, np.float32)
    if component_count:
        row, col = np.nonzero(wet)
        x = grid.x0 + (col + 0.5) * grid.cell
        y = grid.y0 + (row + 0.5) * grid.cell
        raw_noise = _unbounded_micro_texture(x, y)[0]
        label = labels[row, col]
        totals = np.bincount(
            label, weights=raw_noise, minlength=component_count + 1
        )
        counts = np.bincount(label, minlength=component_count + 1)
        means = np.divide(
            totals, counts, out=np.zeros_like(totals), where=counts > 0
        )
        noise_mean[wet] = means[labels[wet]].astype(np.float32)
    return SurfaceReference(
        grid, wet, signed.astype(np.float32), z,
        dzdx.astype(np.float32), dzdy.astype(np.float32),
        signed_dx.astype(np.float32), signed_dy.astype(np.float32),
        noise_mean, reviewed_topology & ~original_wet, exclusion,
    )


def _sample(values: np.ndarray, x, y, surface: SurfaceReference) -> np.ndarray:
    return v6.sample_grid(values, x, y, surface.grid)


def footprint_contains(surface: SurfaceReference, x, y) -> np.ndarray:
    x = np.asarray(x)
    y = np.asarray(y)
    score = _sample(surface.signed_cells, x, y, surface)
    inside = score > 0.0
    gx = np.floor((x - surface.grid.x0) / surface.grid.cell).astype(np.int64)
    gy = np.floor((y - surface.grid.y0) / surface.grid.cell).astype(np.int64)
    valid = (
        (x >= surface.grid.x0) & (x < surface.grid.x1) &
        (y >= surface.grid.y0) & (y < surface.grid.y1) &
        (gx >= 0) & (gx < surface.grid.nx) &
        (gy >= 0) & (gy < surface.grid.ny)
    )
    result = inside & valid
    rows = np.flatnonzero(valid)
    result[rows] &= ~surface.exclusion[gy[rows], gx[rows]]
    result &= ~v9.points_in_exclusion(x, y)
    return result


def _gradient_noise2(x, y, wavelength: float, seed: int, angle: float):
    """Classic continuous 2-D gradient noise plus analytic world gradients."""

    ca, sa = math.cos(angle), math.sin(angle)
    u = (ca * x - sa * y) / wavelength + seed * 0.173
    v = (sa * x + ca * y) / wavelength - seed * 0.117
    ix = np.floor(u).astype(np.int64)
    iy = np.floor(v).astype(np.int64)
    fx = u - ix
    fy = v - iy
    fade_x = fx**3 * (fx * (fx * 6.0 - 15.0) + 10.0)
    fade_y = fy**3 * (fy * (fy * 6.0 - 15.0) + 10.0)
    fade_dx = 30.0 * fx**2 * (fx - 1.0) ** 2
    fade_dy = 30.0 * fy**2 * (fy - 1.0) ** 2

    def corner(offset_x: int, offset_y: int):
        theta = v6._hash_angle(ix + offset_x, iy + offset_y, seed)
        gradient_x = np.cos(theta)
        gradient_y = np.sin(theta)
        value = (
            gradient_x * (fx - offset_x)
            + gradient_y * (fy - offset_y)
        )
        return value, gradient_x, gradient_y

    n00, gx00, gy00 = corner(0, 0)
    n10, gx10, gy10 = corner(1, 0)
    n01, gx01, gy01 = corner(0, 1)
    n11, gx11, gy11 = corner(1, 1)
    nx0 = n00 + fade_x * (n10 - n00)
    nx1 = n01 + fade_x * (n11 - n01)
    value = nx0 + fade_y * (nx1 - nx0)

    nx0_du = (
        gx00 + fade_x * (gx10 - gx00) + fade_dx * (n10 - n00)
    )
    nx1_du = (
        gx01 + fade_x * (gx11 - gx01) + fade_dx * (n11 - n01)
    )
    derivative_u = nx0_du + fade_y * (nx1_du - nx0_du)
    nx0_dv = gy00 + fade_x * (gy10 - gy00)
    nx1_dv = gy01 + fade_x * (gy11 - gy01)
    derivative_v = (
        nx0_dv + fade_y * (nx1_dv - nx0_dv)
        + fade_dy * (nx1 - nx0)
    )
    derivative_x = (ca * derivative_u + sa * derivative_v) / wavelength
    derivative_y = (-sa * derivative_u + ca * derivative_v) / wavelength
    return value, derivative_x, derivative_y


def _soft_limit_noise(height):
    """C1-compress only rare extremes while preserving the calibrated RMS."""

    magnitude = np.abs(height)
    span = NOISE_SOFT_CLIP_LIMIT_M - NOISE_SOFT_CLIP_START_M
    over = np.maximum(magnitude - NOISE_SOFT_CLIP_START_M, 0.0)
    tangent = np.tanh(over / span)
    limited = np.sign(height) * (
        NOISE_SOFT_CLIP_START_M + span * tangent
    )
    active = magnitude > NOISE_SOFT_CLIP_START_M
    result = np.where(active, limited, height)
    derivative = np.where(active, 1.0 - tangent * tangent, 1.0)
    return result, derivative


def _unbounded_micro_texture(x, y, scale: float = 1.0):
    """Return the calibrated raw spectrum and analytic XY gradients."""

    x, y = np.broadcast_arrays(
        np.asarray(x, np.float64), np.asarray(y, np.float64)
    )
    height = np.zeros(x.shape, np.float64)
    dx = np.zeros(x.shape, np.float64)
    dy = np.zeros(x.shape, np.float64)
    for wavelength, rms, seed, angle in zip(
            NOISE_OCTAVE_WAVELENGTHS_M, NOISE_OCTAVE_RMS_M,
            NOISE_OCTAVE_SEEDS, NOISE_OCTAVE_ANGLES_RAD):
        value, value_dx, value_dy = _gradient_noise2(
            x, y, float(wavelength), int(seed), float(angle)
        )
        amplitude = scale * float(rms) / NOISE_PERLIN_NATIVE_RMS
        height += amplitude * value
        dx += amplitude * value_dx
        dy += amplitude * value_dy
    for wavelength, amplitude, angle, phase in zip(
            MICROGRAIN_WAVELENGTHS_M, MICROGRAIN_AMPLITUDES_M,
            MICROGRAIN_ANGLES_RAD, MICROGRAIN_PHASE_RAD):
        ca, sa = math.cos(float(angle)), math.sin(float(angle))
        wave_number = 2.0 * math.pi / float(wavelength)
        argument = wave_number * (ca * x + sa * y) + float(phase)
        sine = np.sin(argument)
        cosine = np.cos(argument)
        peak = scale * float(amplitude)
        height += peak * sine
        dx += peak * wave_number * ca * cosine
        dy += peak * wave_number * sa * cosine
    return height, dx, dy


def micro_texture(x, y, scale: float = 1.0):
    """Return bounded continuous height and analytic world-XY gradients."""

    height, dx, dy = _unbounded_micro_texture(x, y, scale)
    height, derivative = _soft_limit_noise(height)
    return (
        height.astype(np.float32),
        (dx * derivative).astype(np.float32),
        (dy * derivative).astype(np.float32),
    )


def surface_values(surface: SurfaceReference, x, y, noise_scale: float = 1.0):
    z = _sample(surface.z, x, y, surface)
    gx = _sample(surface.dzdx, x, y, surface)
    gy = _sample(surface.dzdy, x, y, surface)
    raw_ripple, raw_rx, raw_ry = _unbounded_micro_texture(
        x, y, noise_scale
    )
    component_mean = (
        _sample(surface.noise_mean, x, y, surface) * noise_scale
    )
    ripple, limit_derivative = _soft_limit_noise(
        raw_ripple - component_mean
    )
    raw_rx *= limit_derivative
    raw_ry *= limit_derivative
    distance = np.maximum(
        _sample(surface.signed_cells, x, y, surface) * surface.grid.cell,
        0.0,
    )
    t = np.clip(distance / SHORE_NOISE_RAMP_M, 0.0, 1.0)
    smooth = t * t * (3.0 - 2.0 * t)
    fade = SHORE_NOISE_FLOOR + (1.0 - SHORE_NOISE_FLOOR) * smooth
    derivative = np.where(
        (t > 0.0) & (t < 1.0),
        (1.0 - SHORE_NOISE_FLOOR) * 6.0 * t * (1.0 - t)
        / SHORE_NOISE_RAMP_M,
        0.0,
    )
    fade_dx = derivative * _sample(surface.signed_dx, x, y, surface)
    fade_dy = derivative * _sample(surface.signed_dy, x, y, surface)
    z = z + fade * ripple
    gx = gx + fade * raw_rx + ripple * fade_dx
    gy = gy + fade * raw_ry + ripple * fade_dy
    length = np.sqrt(1.0 + gx * gx + gy * gy)
    normals = np.column_stack((-gx / length, -gy / length, 1.0 / length))
    return z.astype(np.float32), normals.astype(np.float32)


def prefix_dtype(full_dtype: np.dtype) -> np.dtype:
    names = list(full_dtype.names)
    stop = names.index("scalar_ScanID") + 1
    fields = [(name, full_dtype.fields[name][0]) for name in names[:stop]]
    # CleanMesh reduced analysis needs a material identity when the larger
    # Group-A schema is absent.  This temporary field is stripped after the
    # combined terrain+WATER analysis.
    fields.append(("scalar_TypeID", "<f4"))
    return np.dtype(fields)


def write_counted_header(handle, dtype: np.dtype, comments: Iterable[str]):
    typemap = {"f4": "float", "f8": "double", "u1": "uchar",
               "i1": "char", "i2": "short", "u2": "ushort",
               "i4": "int", "u4": "uint"}
    handle.write(b"ply\nformat binary_little_endian 1.0\n")
    for comment in comments:
        handle.write(f"comment {comment}\n".encode("ascii"))
    handle.write(b"element vertex ")
    count_offset = handle.tell()
    handle.write(b"00000000000000000000\n")
    for name in dtype.names:
        code = dtype[name].str.lstrip("<>|=")
        handle.write(f"property {typemap[code]} {name}\n".encode("ascii"))
    handle.write(b"end_header\n")
    return count_offset


def patch_count(handle, count_offset: int, count: int) -> None:
    if count < 0 or count >= 10**20:
        raise ValueError("PLY count does not fit fixed header slot")
    current = handle.tell()
    handle.seek(count_offset)
    handle.write(f"{count:020d}".encode("ascii"))
    handle.seek(current)


def _iter_chunks(mm, chunk_size=CHUNK):
    for start in range(0, len(mm), chunk_size):
        yield mm[start:start + chunk_size]


def _base_near_surface(
    chunk,
    surface: SurfaceReference,
    collar_m: float = TERRAIN_ANALYSIS_COLLAR_M,
) -> np.ndarray:
    x = np.asarray(chunk["x"])
    y = np.asarray(chunk["y"])
    in_bounds = (
        (x >= surface.grid.x0) & (x < surface.grid.x1) &
        (y >= surface.grid.y0) & (y < surface.grid.y1)
    )
    signed_m = _sample(surface.signed_cells, x, y, surface) * surface.grid.cell
    inside = in_bounds & (signed_m >= -float(collar_m))
    inside &= ~v9.points_in_exclusion(x, y)
    local = np.flatnonzero(inside)
    keep = np.zeros(len(chunk), bool)
    if not len(local):
        return keep
    zref = _sample(surface.z, chunk["x"][local], chunk["y"][local], surface)
    dz = chunk["z"][local].astype(np.float32) - zref
    keep[local] = (np.isfinite(dz) & (dz >= -BASE_Z_BELOW_M) &
                   (dz <= BASE_Z_ABOVE_M))
    return keep


def write_priority_base(
    handle,
    data_dir: Path,
    spacing: str,
    surface: SurfaceReference,
    output_dtype: np.dtype,
    *,
    full_records: bool,
) -> int:
    written = 0
    for role in ("SAND", "ROCK"):
        mm, dtype, *_ = v6.memmap_cloud(data_dir / f"Site1-{role}-{spacing}.ply")
        if full_records and dtype != output_dtype:
            raise RuntimeError(f"{role}-{spacing} schema mismatch")
        for chunk in _iter_chunks(mm):
            keep = _base_near_surface(chunk, surface)
            if not np.any(keep):
                continue
            source = np.asarray(chunk[keep])
            if full_records:
                record = source.copy()
            else:
                record = np.empty(len(source), output_dtype)
                for name in output_dtype.names:
                    if name == "scalar_TypeID":
                        record[name] = 1.0 if role == "SAND" else 0.0
                    else:
                        record[name] = source[name]
            if "scalar_TypeID" in output_dtype.names:
                record["scalar_TypeID"] = 1.0 if role == "SAND" else 0.0
            record["scalar_ScanID"] = BASE_PRIORITY_SCAN_ID
            record.tofile(handle)
            written += len(record)
    return written


def _tile_ranges(surface: SurfaceReference, pitch: float, tile_m: float):
    grid = surface.grid
    ntx = int(math.ceil((grid.x1 - grid.x0) / tile_m))
    nty = int(math.ceil((grid.y1 - grid.y0) / tile_m))
    tiles = np.arange(ntx * nty, dtype=np.int64)
    priority = v8.hash01(tiles % ntx, tiles // ntx, 503)
    for tile in tiles[np.argsort(priority, kind="stable")]:
        tx, ty = int(tile % ntx), int(tile // ntx)
        xlo = grid.x0 + tx * tile_m
        xhi = min(grid.x1, xlo + tile_m)
        ylo = grid.y0 + ty * tile_m
        yhi = min(grid.y1, ylo + tile_m)
        ix0 = max(0, int(math.ceil((xlo - grid.x0) / pitch - 0.5)))
        ix1 = min(int(math.ceil((grid.x1 - grid.x0) / pitch)),
                  int(math.ceil((xhi - grid.x0) / pitch - 0.5)))
        iy0 = max(0, int(math.ceil((ylo - grid.y0) / pitch - 0.5)))
        iy1 = min(int(math.ceil((grid.y1 - grid.y0) / pitch)),
                  int(math.ceil((yhi - grid.y0) / pitch - 0.5)))
        if ix1 > ix0 and iy1 > iy0:
            yield ix0, ix1, iy0, iy1


def _apply_donor_prefix(record: np.ndarray, bundle: dict) -> None:
    grid = bundle["grid"]
    for name in ("red", "green", "blue", "scalar_Intensity", "scalar_Composite"):
        values = bundle["fields"][name]
        sampled = v6.sample_grid(values, record["x"], record["y"], grid)
        lo, hi = v9.FIELD_BOUNDS[name]
        sampled = np.clip(sampled, lo, hi)
        if record.dtype[name].kind == "u":
            record[name] = np.rint(sampled).astype(record.dtype[name])
        else:
            record[name] = sampled.astype(record.dtype[name])


def write_oversampled_water(
    handle,
    surface: SurfaceReference,
    dtype: np.dtype,
    donor_bundle: dict,
    noise_scale: float,
    log,
) -> int:
    total = 0
    pitch = OVERSAMPLE_PITCH
    for tile_index, (ix0, ix1, iy0, iy1) in enumerate(
            _tile_ranges(surface, pitch, GENERATION_TILE_M), 1):
        ix, iy = np.meshgrid(
            np.arange(ix0, ix1, dtype=np.int64),
            np.arange(iy0, iy1, dtype=np.int64),
        )
        ix = ix.ravel()
        iy = iy.ravel()
        jx = v8.hash01(ix, iy, 541) - 0.5
        jy = v8.hash01(ix, iy, 547) - 0.5
        x = np.asarray(
            surface.grid.x0 + (ix + 0.5 + 0.90 * jx) * pitch,
            np.float32,
        )
        y = np.asarray(
            surface.grid.y0 + (iy + 0.5 + 0.90 * jy) * pitch,
            np.float32,
        )
        keep = footprint_contains(surface, x, y)
        if not np.any(keep):
            continue
        ix, iy, x, y = ix[keep], iy[keep], x[keep], y[keep]
        order = np.argsort(v8.hash01(ix, iy, 557), kind="stable")
        x = x[order]
        y = y[order]
        z, normal = surface_values(surface, x, y, noise_scale)
        record = np.zeros(len(x), dtype)
        record["x"], record["y"], record["z"] = x, y, z
        record["nx"], record["ny"], record["nz"] = normal.T
        _apply_donor_prefix(record, donor_bundle)
        record["scalar_ScanID"] = WATER_SCAN_ID
        record["scalar_TypeID"] = 1.0
        record.tofile(handle)
        total += len(record)
        if tile_index % 250 == 0:
            log(f"oversample tiles {tile_index:,}; WATER {total:,}")
    return total


def build_dense_downsample_input(
    path: Path,
    data_dir: Path,
    surface: SurfaceReference,
    donor_bundle: dict,
    noise_scale: float,
    overwrite: bool,
    log,
) -> dict:
    if path.exists() and not overwrite:
        raise FileExistsError(f"{path} exists; pass --overwrite")
    full_dtype, *_ = v6.read_ply_header(data_dir / "Site1-SAND-5mm.ply")
    dtype = prefix_dtype(full_dtype)
    temporary = path.with_suffix(path.suffix + ".partial")
    temporary.unlink(missing_ok=True)
    with open(temporary, "w+b") as handle:
        count_offset = write_counted_header(handle, dtype, [
            "Site1 WATER v10 dense Poisson input",
            "Measured SAND/ROCK use a temporary priority ScanID; canonical files unchanged",
            "Generated WATER is an irregular 1.35 mm oversample of the verified v9 sheet",
        ])
        base_count = write_priority_base(
            handle, data_dir, "1mm", surface, dtype, full_records=False)
        log(f"dense priority terrain {base_count:,}")
        water_count = write_oversampled_water(
            handle, surface, dtype, donor_bundle, noise_scale, log)
        patch_count(handle, count_offset, base_count + water_count)
    os.replace(temporary, path)
    return {"base_points": base_count, "water_oversample_points": water_count,
            "total_points": base_count + water_count}


def run_downsample(
    executable: Path,
    source: Path,
    output: Path,
    spacing: float,
    report: Path,
    overwrite: bool,
    log,
) -> None:
    if not executable.exists():
        raise FileNotFoundError(f"missing CleanMesh downsampler: {executable}")
    command = [
        str(executable), "--input", str(source), "--output", str(output),
        "--spacing", f"{spacing:.9g}", "--report", str(report),
        "--priority-scan-id", str(int(BASE_PRIORITY_SCAN_ID)),
        "--chunk-points", "1000000",
    ]
    if overwrite:
        command.append("--force")
    log("run: " + " ".join(command))
    subprocess.run(command, check=True)


def extract_scan_id(
    source: Path,
    output: Path,
    scan_id: float,
    comments: list[str],
    overwrite: bool,
) -> int:
    if output.exists() and not overwrite:
        raise FileExistsError(f"{output} exists; pass --overwrite")
    mm, dtype, *_ = v6.memmap_cloud(source)
    count = 0
    for chunk in _iter_chunks(mm):
        count += int(np.count_nonzero(chunk["scalar_ScanID"] == scan_id))
    temporary = output.with_suffix(output.suffix + ".partial")
    temporary.unlink(missing_ok=True)
    with open(temporary, "wb") as handle:
        v6.write_ply_header(handle, dtype, count, comments)
        written = 0
        for chunk in _iter_chunks(mm):
            keep = chunk["scalar_ScanID"] == scan_id
            if np.any(keep):
                np.asarray(chunk[keep]).tofile(handle)
                written += int(np.count_nonzero(keep))
    if written != count:
        temporary.unlink(missing_ok=True)
        raise RuntimeError(f"scan extraction wrote {written}, expected {count}")
    os.replace(temporary, output)
    return count


def strip_to_expected_schema(
    source: Path,
    output: Path,
    expected_schema_path: Path,
    comments: list[str],
    overwrite: bool,
) -> int:
    """Copy named canonical fields while dropping temporary analysis fields."""

    if output.exists() and not overwrite:
        raise FileExistsError(f"{output} exists; pass --overwrite")
    source_mm, source_dtype, *_ = v6.memmap_cloud(source)
    expected_dtype, *_ = v6.read_ply_header(expected_schema_path)
    missing = sorted(set(expected_dtype.names) - set(source_dtype.names))
    if missing:
        raise RuntimeError(f"analysed WATER is missing canonical fields {missing}")
    temporary = output.with_suffix(output.suffix + ".partial")
    temporary.unlink(missing_ok=True)
    with open(temporary, "wb") as handle:
        v6.write_ply_header(handle, expected_dtype, len(source_mm), comments)
        for chunk in _iter_chunks(source_mm):
            record = np.empty(len(chunk), expected_dtype)
            for name in expected_dtype.names:
                record[name] = chunk[name]
            record.tofile(handle)
    os.replace(temporary, output)
    return int(len(source_mm))


def build_base_blocker_tree(
    data_dir: Path,
    spacing_label: str,
    surface: SurfaceReference,
    collar_m: float,
    log,
):
    """Index every relevant terrain point, including ones priority-thinned."""

    from scipy.spatial import cKDTree

    coordinates = []
    for role in ("SAND", "ROCK"):
        mm, *_ = v6.memmap_cloud(
            data_dir / f"Site1-{role}-{spacing_label}.ply"
        )
        role_count = 0
        for chunk in _iter_chunks(mm):
            keep = _base_near_surface(
                chunk, surface, collar_m=collar_m
            )
            if not np.any(keep):
                continue
            xyz = np.column_stack((
                chunk["x"][keep], chunk["y"][keep], chunk["z"][keep]
            )).astype(np.float32)
            coordinates.append(xyz)
            role_count += len(xyz)
        log(f"{spacing_label} blocker terrain {role}: {role_count:,}")
    if not coordinates:
        raise RuntimeError(f"no {spacing_label} terrain blocker points")
    joined = np.concatenate(coordinates, axis=0)
    log(f"{spacing_label} all-terrain blocker index: {len(joined):,}")
    return cKDTree(joined, compact_nodes=True, balanced_tree=True), joined


def filter_water_against_all_base(
    source: Path,
    output: Path,
    data_dir: Path,
    spacing_label: str,
    spacing_m: float,
    surface: SurfaceReference,
    overwrite: bool,
    log,
) -> dict:
    """Remove WATER closer than spacing to *any* canonical terrain point."""

    if output.exists() and not overwrite:
        raise FileExistsError(f"{output} exists; pass --overwrite")
    tree, blocker_xyz = build_base_blocker_tree(
        data_dir, spacing_label, surface, collar_m=spacing_m, log=log
    )
    mm, dtype, *_ = v6.memmap_cloud(source)
    temporary = output.with_suffix(output.suffix + ".partial")
    temporary.unlink(missing_ok=True)
    source_water = kept_water = removed_water = kept_total = 0
    minimum_observed = math.inf
    with open(temporary, "w+b") as handle:
        count_offset = write_counted_header(handle, dtype, [
            "Site1 v10 exact terrain-blocker pass",
            f"Every WATER point is at least {spacing_m:.6g} m from all nearby canonical SAND/ROCK",
        ])
        for chunk in _iter_chunks(mm, 500_000):
            is_water = chunk["scalar_ScanID"] == WATER_SCAN_ID
            keep = np.ones(len(chunk), bool)
            local = np.flatnonzero(is_water)
            source_water += len(local)
            if len(local):
                query = np.column_stack((
                    chunk["x"][local], chunk["y"][local], chunk["z"][local]
                ))
                distance, _ = tree.query(
                    query, k=1, distance_upper_bound=spacing_m, workers=-1
                )
                finite = np.isfinite(distance)
                if np.any(finite):
                    minimum_observed = min(
                        minimum_observed, float(np.min(distance[finite]))
                    )
                collision = finite & (distance < spacing_m)
                keep[local[collision]] = False
                removed_water += int(np.count_nonzero(collision))
                kept_water += int(len(local) - np.count_nonzero(collision))
            np.asarray(chunk[keep]).tofile(handle)
            kept_total += int(np.count_nonzero(keep))
        patch_count(handle, count_offset, kept_total)
    os.replace(temporary, output)
    del tree, blocker_xyz
    return {
        "spacing_label": spacing_label,
        "minimum_spacing_m": spacing_m,
        "source_water_points": source_water,
        "kept_water_points": kept_water,
        "removed_water_points": removed_water,
        "output_total_points": kept_total,
        "minimum_removed_distance_m": (
            minimum_observed if np.isfinite(minimum_observed) else None
        ),
    }


def run_reduced_analysis(
    executable: Path,
    source: Path,
    output: Path,
    report: Path,
    overwrite: bool,
    log,
) -> None:
    if not executable.exists():
        raise FileNotFoundError(f"missing CleanMesh reduced analysis: {executable}")
    command = [
        str(executable), "--input", str(source), "--out", str(output),
        "--report", str(report), "--base-voxel", "0.003",
        "--tile-width", "4.0",
        "--chunk-points", "1000000", "--normalization-samples", "2000000",
    ]
    if overwrite:
        command.append("--force")
    log("run: " + " ".join(command))
    subprocess.run(command, check=True)


def rock_normalizations(data_dir: Path) -> tuple[dict, dict]:
    """Infer the exact shared ROCK p95 policy used by scalar repair."""

    options = scalar_fill.RepairOptions(
        chunk_size=500_000,
        derived_normalization_sample_limit=2_000_000,
    )
    return scalar_fill.infer_derived_normalization(
        data_dir / "Site1-ROCK-1mm.ply", "1mm", options
    )


def _combined(metric: str, chunk: np.ndarray, scales) -> np.ndarray:
    values = np.column_stack([
        np.asarray(chunk[f"scalar_A_R_{metric}_{scale}"], np.float64)
        for scale in SCALES
    ])
    if isinstance(scales, Mapping):
        scales = [scales[scale] for scale in SCALES]
    finite = np.isfinite(values)
    normalized = values / np.asarray(scales, np.float64)
    if metric == "Roughness":
        normalized = np.clip(normalized, 0.0, 1.0)
    else:
        normalized = np.clip(normalized, -1.0, 1.0)
    used = np.sum(finite * COMBINED_WEIGHTS[None, :], axis=1)
    weighted = np.sum(
        np.where(finite, normalized, 0.0) * COMBINED_WEIGHTS[None, :],
        axis=1,
    )
    result = np.full(len(chunk), np.nan, np.float64)
    valid = used > 0.0
    result[valid] = weighted[valid] / used[valid]
    return result


def postprocess_water(
    path: Path,
    donor_bundle: dict,
    normalizations: dict,
) -> dict:
    info = scalar_fill.inspect_fixed_stride_ply(path)
    mm = np.memmap(path, dtype=info.dtype, mode="r+", offset=info.offset,
                   shape=(info.count,))
    ranges = {}
    for span_start in range(0, len(mm), CHUNK):
        chunk = mm[span_start:span_start + CHUNK]
        for name in ENV_FIELDS:
            chunk[name] = v6.sample_grid(
                donor_bundle["fields"][name], chunk["x"], chunk["y"],
                donor_bundle["grid"],
            ).astype(np.float32)
        for metric in PHYSICAL_METRICS:
            name = f"scalar_A_R_{metric}_Combined"
            chunk[name] = _combined(metric, chunk, normalizations[metric]).astype(
                np.float32
            )
        fine = np.asarray(chunk["scalar_A_R_Roughness_Fine"], np.float64)
        medium = np.asarray(chunk["scalar_A_R_Roughness_Medium"], np.float64)
        chunk["scalar_A_R_RoughnessRelative_FineMedium"] = np.clip(
            fine / np.maximum(medium, 1e-9), 0.0, 8.0
        ).astype(np.float32)
    mm.flush()
    for metric in PHYSICAL_METRICS:
        name = f"scalar_A_R_{metric}_Combined"
        values = np.asarray(mm[name][::max(1, len(mm) // 500_000)], np.float64)
        values = values[np.isfinite(values)]
        ranges[name] = {
            "q01": float(np.quantile(values, 0.01)),
            "q50": float(np.quantile(values, 0.50)),
            "q99": float(np.quantile(values, 0.99)),
        }
    del mm
    return ranges


def verify_final_water_postprocess(
    analysed: Path,
    candidate: Path,
) -> dict:
    """Verify scalar repair plus the explicitly allowed final field rewrite."""

    source_info = scalar_fill.inspect_fixed_stride_ply(analysed)
    candidate_info = scalar_fill.inspect_fixed_stride_ply(candidate)
    failures = []
    if source_info.dtype != candidate_info.dtype:
        failures.append("schema changed")
    if source_info.count != candidate_info.count:
        failures.append("point count changed")
    if failures:
        return {"verified": False, "failures": failures}
    source = np.memmap(
        analysed, dtype=source_info.dtype, mode="r",
        offset=source_info.offset, shape=(source_info.count,),
    )
    result = np.memmap(
        candidate, dtype=candidate_info.dtype, mode="r",
        offset=candidate_info.offset, shape=(candidate_info.count,),
    )
    rewrite_all = set(ENV_FIELDS) | set(scalar_fill.DERIVED_FIELDS)
    repairable = set(scalar_fill.REPAIRABLE_FIELDS)
    unrelated = set(source_info.dtype.names) - repairable
    unrelated_changed = {field: 0 for field in unrelated}
    finite_component_changed = {
        field: 0 for field in repairable - rewrite_all
    }
    nonfinite = {
        field: 0 for field in source_info.dtype.names
        if source_info.dtype[field].kind == "f"
    }
    out_of_bounds = {
        field: 0 for field in scalar_fill.FIELD_BOUNDS
        if field in source_info.dtype.names
    }
    for span_start in range(0, source_info.count, CHUNK):
        span = slice(span_start, min(span_start + CHUNK, source_info.count))
        before = source[span]
        after = result[span]
        for field in unrelated:
            left = np.ascontiguousarray(before[field]).view(np.uint8)
            right = np.ascontiguousarray(after[field]).view(np.uint8)
            unrelated_changed[field] += int(np.count_nonzero(left != right))
        for field in finite_component_changed:
            finite = np.isfinite(before[field])
            if not np.any(finite):
                continue
            itemsize = source_info.dtype[field].itemsize
            unsigned = np.dtype(f"u{itemsize}")
            left = np.ascontiguousarray(before[field][finite]).view(unsigned)
            right = np.ascontiguousarray(after[field][finite]).view(unsigned)
            finite_component_changed[field] += int(np.count_nonzero(left != right))
        for field in nonfinite:
            nonfinite[field] += int(np.count_nonzero(~np.isfinite(after[field])))
        for field in out_of_bounds:
            low, high = scalar_fill.FIELD_BOUNDS[field]
            values = after[field]
            out_of_bounds[field] += int(np.count_nonzero(
                ~np.isfinite(values) | (values < low) | (values > high)
            ))
    del source, result
    changed_unrelated = {
        field: count for field, count in unrelated_changed.items() if count
    }
    changed_finite = {
        field: count for field, count in finite_component_changed.items() if count
    }
    remaining_nonfinite = {
        field: count for field, count in nonfinite.items() if count
    }
    invalid_bounds = {
        field: count for field, count in out_of_bounds.items() if count
    }
    if changed_unrelated:
        failures.append("unrelated field bytes changed")
    if changed_finite:
        failures.append("finite physical/directional component bytes changed")
    if remaining_nonfinite:
        failures.append("non-finite values remain")
    if invalid_bounds:
        failures.append("final values violate hard bounds")
    return {
        "verified": not failures,
        "allowed_full_rewrites": sorted(rewrite_all),
        "unrelated_changed_bytes": changed_unrelated,
        "finite_component_changed_values": changed_finite,
        "remaining_nonfinite": remaining_nonfinite,
        "out_of_bounds": invalid_bounds,
        "failures": failures,
    }


def repair_water_scalars(
    analysed: Path,
    candidate: Path,
    report: Path,
    overwrite: bool,
    normalizations: Mapping[str, Mapping[str, float]],
) -> dict:
    options = scalar_fill.RepairOptions(
        chunk_size=500_000,
        max_donors_per_group=1_500_000,
        donor_query_k=8,
        distance_buckets_m=(0.025, 0.050, 0.100, 0.200, 0.400,
                            0.800, 1.600),
        xy_fallback_buckets_m=(0.025, 0.050, 0.100, 0.200, 0.400,
                               0.800, 1.600, 3.200, 6.400, 12.800,
                               25.600, 51.200, 102.400, 204.800),
        overwrite=overwrite,
        verify=True,
        derived_normalization=normalizations,
    )
    return scalar_fill.repair_scalar_file(
        analysed, candidate, role="WATER", spacing="2mm",
        report_path=report, options=options,
    )


def build_coarse_input(
    path: Path,
    data_dir: Path,
    dense_candidate: Path,
    surface: SurfaceReference,
    overwrite: bool,
) -> dict:
    dense, dtype, *_ = v6.memmap_cloud(dense_candidate)
    temporary = path.with_suffix(path.suffix + ".partial")
    if path.exists() and not overwrite:
        raise FileExistsError(f"{path} exists; pass --overwrite")
    temporary.unlink(missing_ok=True)
    with open(temporary, "w+b") as handle:
        count_offset = write_counted_header(handle, dtype, [
            "Site1 WATER v10 coarse Poisson input",
            "Measured 5 mm SAND/ROCK use a temporary priority ScanID",
            "Dense v10 WATER is selected only after the measured terrain pass",
        ])
        base_count = write_priority_base(
            handle, data_dir, "5mm", surface, dtype, full_records=True)
        for chunk in _iter_chunks(dense):
            np.asarray(chunk).tofile(handle)
        patch_count(handle, count_offset, base_count + len(dense))
    os.replace(temporary, path)
    return {"base_points": base_count, "dense_water_points": int(len(dense)),
            "total_points": base_count + int(len(dense))}


def _sample_field_quantiles(mm, field: str, maximum=500_000) -> dict:
    stride = max(1, math.ceil(len(mm) / maximum))
    values = np.asarray(mm[field][::stride], np.float64)
    values = values[np.isfinite(values)]
    if not len(values):
        return {"count": 0}
    q = np.quantile(values, [0.01, 0.05, 0.25, 0.50, 0.75, 0.95, 0.99])
    return {"count": int(len(values)), **{
        key: float(value) for key, value in zip(
            ("q01", "q05", "q25", "q50", "q75", "q95", "q99"), q
        )
    }}


def _quantiles(values: np.ndarray) -> dict:
    """Compact, JSON-safe distribution summary for sampled values."""

    finite = np.asarray(values, np.float64)
    finite = finite[np.isfinite(finite)]
    if not len(finite):
        return {"count": 0}
    probabilities = (
        0.0, 0.01, 0.05, 0.25, 0.50, 0.75, 0.95, 0.99, 1.0
    )
    names = (
        "min", "q01", "q05", "q25", "q50",
        "q75", "q95", "q99", "max",
    )
    return {
        "count": int(len(finite)),
        **{
            name: float(value)
            for name, value in zip(names, np.quantile(finite, probabilities))
        },
    }


def audit_candidate(
    path: Path,
    data_dir: Path,
    surface: SurfaceReference,
    spacing: float,
    config: Mapping,
    noise_scale: float = 1.0,
) -> dict:
    mm, dtype, *_ = v6.memmap_cloud(path)
    expected, *_ = v6.read_ply_header(data_dir / "Site1-SAND-5mm.ply")
    finite_bad = {name: 0 for name in dtype.names if dtype[name].kind == "f"}
    bounds = {
        **{
            name: value for name, value in v9.FIELD_BOUNDS.items()
            if name in dtype.names
        },
        **{
            name: value for name, value in scalar_fill.FIELD_BOUNDS.items()
            if name in dtype.names
        },
    }
    out_of_bounds = {name: 0 for name in bounds}
    wrong_scan = outside = excluded = cross_collisions = 0
    cell_count = np.zeros(surface.grid.shape, np.int32)
    spacing_label = "1mm" if spacing == DENSE_SPACING else "5mm"
    blocker_tree, blocker_xyz = build_base_blocker_tree(
        data_dir, spacing_label, surface, collar_m=spacing, log=lambda _: None
    )
    base_count = np.zeros(surface.grid.shape, np.int32)
    base_inside = footprint_contains(
        surface, blocker_xyz[:, 0], blocker_xyz[:, 1]
    )
    bx = np.floor(
        (blocker_xyz[base_inside, 0] - surface.grid.x0) / surface.grid.cell
    ).astype(np.int64)
    by = np.floor(
        (blocker_xyz[base_inside, 1] - surface.grid.y0) / surface.grid.cell
    ).astype(np.int64)
    np.add.at(
        base_count.ravel(), by * surface.grid.nx + bx, 1
    )
    sample_step = max(1, math.ceil(len(mm) / 1_000_000))
    sampled_z_error = []
    sampled_ripple = []
    sampled_base_distance = []
    for start in range(0, len(mm), CHUNK):
        chunk = mm[start:start + CHUNK]
        for name in finite_bad:
            finite_bad[name] += int(np.count_nonzero(~np.isfinite(chunk[name])))
        for name, (low, high) in bounds.items():
            values = chunk[name]
            out_of_bounds[name] += int(np.count_nonzero(
                ~np.isfinite(values) | (values < low) | (values > high)
            ))
        wrong_scan += int(np.count_nonzero(chunk["scalar_ScanID"] != WATER_SCAN_ID))
        accepted = footprint_contains(surface, chunk["x"], chunk["y"])
        outside += int(np.count_nonzero(~accepted))
        excluded += int(np.count_nonzero(v9.points_in_exclusion(
            chunk["x"], chunk["y"])))
        gx = np.floor(
            (chunk["x"][accepted] - surface.grid.x0) / surface.grid.cell
        ).astype(np.int64)
        gy = np.floor(
            (chunk["y"][accepted] - surface.grid.y0) / surface.grid.cell
        ).astype(np.int64)
        key = gy.astype(np.int64) * surface.grid.nx + gx
        np.add.at(cell_count.ravel(), key, 1)
        xyz = np.column_stack((chunk["x"], chunk["y"], chunk["z"]))
        distance, _ = blocker_tree.query(
            xyz, k=1, distance_upper_bound=spacing, workers=-1
        )
        cross_collisions += int(np.count_nonzero(
            np.isfinite(distance) & (distance < spacing - 1.0e-7)
        ))
        global_index = np.arange(start, start + len(chunk), dtype=np.int64)
        sample = np.flatnonzero((global_index % sample_step) == 0)
        if len(sample):
            sx = chunk["x"][sample]
            sy = chunk["y"][sample]
            expected_z = surface_values(
                surface, sx, sy, noise_scale=noise_scale
            )[0]
            sampled_z_error.append(
                np.abs(chunk["z"][sample].astype(np.float64) - expected_z)
            )
            base_z = _sample(surface.z, sx, sy, surface)
            sampled_ripple.append(
                expected_z.astype(np.float64) - base_z.astype(np.float64)
            )
            nearest, _ = blocker_tree.query(xyz[sample], k=1, workers=-1)
            sampled_base_distance.append(nearest)
    del blocker_tree, blocker_xyz
    wet_values = cell_count[surface.wet]
    occupied = wet_values > 0
    combined = cell_count + base_count
    combined_wet = combined[surface.wet]
    combined_occupied = combined_wet > 0
    fields = {}
    for metric in PHYSICAL_METRICS:
        for suffix in (*SCALES, "Combined"):
            name = f"scalar_A_R_{metric}_{suffix}"
            fields[name] = _sample_field_quantiles(mm, name)
    z_error = np.concatenate(sampled_z_error) if sampled_z_error else np.zeros(0)
    ripple = np.concatenate(sampled_ripple) if sampled_ripple else np.zeros(0)
    base_distance = (
        np.concatenate(sampled_base_distance)
        if sampled_base_distance else np.zeros(0)
    )
    addition_metrics = {}
    for region in config.get("add_water_regions", []):
        mask = _polygon_mask([region["polygon"]], surface.grid)
        mask &= surface.review_additions
        values = combined[mask]
        addition_metrics[region["id"]] = {
            "cells": int(len(values)),
            "occupied_fraction": (
                float(np.mean(values > 0)) if len(values) else None
            ),
            "combined_count_q05": (
                float(np.quantile(values, 0.05)) if len(values) else None
            ),
            "combined_count_median": (
                float(np.median(values)) if len(values) else None
            ),
        }
    marked_density = {}
    for name, marked in config.get("marked_locations", {}).items():
        if not isinstance(marked, Mapping) or "review_bbox" not in marked:
            continue
        x0, x1, y0, y1 = marked["review_bbox"]
        col0 = max(0, int(math.floor((x0 - surface.grid.x0) / surface.grid.cell)))
        col1 = min(surface.grid.nx, int(math.ceil((x1 - surface.grid.x0) / surface.grid.cell)))
        row0 = max(0, int(math.floor((y0 - surface.grid.y0) / surface.grid.cell)))
        row1 = min(surface.grid.ny, int(math.ceil((y1 - surface.grid.y0) / surface.grid.cell)))
        values = combined[row0:row1, col0:col1].ravel()
        marked_density[name] = (
            {
                "cells": int(len(values)),
                "combined_q05": float(np.quantile(values, 0.05)),
                "combined_median": float(np.median(values)),
                "combined_q95": float(np.quantile(values, 0.95)),
            }
            if len(values) else {"cells": 0}
        )
    failures = []
    if dtype != expected:
        failures.append("schema mismatch")
    if any(finite_bad.values()):
        failures.append("non-finite fields remain")
    if any(out_of_bounds.values()):
        failures.append("scalar fields violate hard bounds")
    if wrong_scan:
        failures.append(f"wrong ScanID on {wrong_scan} points")
    if outside:
        failures.append(f"{outside} points outside continuous footprint")
    if excluded:
        failures.append(f"{excluded} points inside strict v9 exclusions")
    if cross_collisions:
        failures.append(
            f"{cross_collisions} WATER points overlap all-terrain blockers"
        )
    acceptance = config.get("acceptance", {})
    minimum_occupancy = float(
        acceptance.get("minimum_combined_wet_occupancy_fraction", 0.99)
    )
    combined_occupancy = float(np.mean(combined_occupied))
    if combined_occupancy < minimum_occupancy:
        failures.append(
            f"combined wet-cell occupancy {combined_occupancy:.6f} < "
            f"{minimum_occupancy:.6f}"
        )
    minimum_addition_occupancy = float(
        acceptance.get("minimum_review_addition_occupancy_fraction", 0.98)
    )
    for region, metric in addition_metrics.items():
        value = metric["occupied_fraction"]
        if value is not None and value < minimum_addition_occupancy:
            failures.append(
                f"review addition {region} occupancy {value:.6f} < "
                f"{minimum_addition_occupancy:.6f}"
            )
    maximum_z_error = float(
        acceptance.get("maximum_surface_reproduction_error_m", 0.00025)
    )
    if len(z_error) and float(np.max(z_error)) > maximum_z_error:
        failures.append("stored height does not reproduce the reviewed surface")
    noise_p99_limit = float(
        acceptance.get("maximum_noise_abs_p99_m", 0.0055)
    )
    noise_max_limit = float(
        acceptance.get("maximum_noise_abs_max_m", 0.009)
    )
    if len(ripple):
        if float(np.quantile(np.abs(ripple), 0.99)) > noise_p99_limit:
            failures.append("noise p99 displacement exceeds calibrated limit")
        if float(np.max(np.abs(ripple))) > noise_max_limit:
            failures.append("noise maximum displacement exceeds calibrated limit")
    roughness_limits = acceptance.get("fine_roughness_median_m")
    roughness_median = fields["scalar_A_R_Roughness_Fine"].get("q50")
    if roughness_limits and roughness_median is not None:
        if not float(roughness_limits[0]) <= roughness_median <= float(roughness_limits[1]):
            failures.append("fine roughness median misses calibrated flat-SAND band")
    for field, quantile_limits in acceptance.get(
        "scalar_quantile_bands", {}
    ).items():
        if field not in fields:
            failures.append(f"unknown scalar acceptance field {field}")
            continue
        for quantile, limits in quantile_limits.items():
            value = fields[field].get(quantile)
            if value is None:
                failures.append(
                    f"{field} has no finite {quantile} for acceptance"
                )
            elif not float(limits[0]) <= value <= float(limits[1]):
                failures.append(
                    f"{field} {quantile}={value:.6g} outside "
                    f"[{float(limits[0]):.6g}, {float(limits[1]):.6g}]"
                )
    return {
        "path": str(path), "points": int(len(mm)), "spacing_m": spacing,
        "sha256": sha256_path(path), "schema_matches": dtype == expected,
        "wrong_scan": wrong_scan, "outside": outside, "excluded": excluded,
        "finite_bad": finite_bad,
        "out_of_bounds": out_of_bounds,
        "cross_cloud_collisions": cross_collisions,
        "sampled_nearest_base_distance_m": (
            _quantiles(base_distance) if len(base_distance) else None
        ),
        "sampled_surface_error_m": (
            _quantiles(z_error) if len(z_error) else None
        ),
        "sampled_noise_displacement_m": (
            _quantiles(ripple) if len(ripple) else None
        ),
        "wet_25mm_cells": int(len(wet_values)),
        "occupied_25mm_cells": int(np.count_nonzero(occupied)),
        "empty_25mm_cells": int(np.count_nonzero(~occupied)),
        "combined_occupied_25mm_cells": int(np.count_nonzero(combined_occupied)),
        "combined_empty_25mm_cells": int(np.count_nonzero(~combined_occupied)),
        "combined_wet_occupancy_fraction": combined_occupancy,
        "points_per_25mm_cell": {
            key: float(value) for key, value in zip(
                ("q01", "q05", "q25", "q50", "q75", "q95", "q99"),
                np.quantile(wet_values, [.01, .05, .25, .5, .75, .95, .99]),
            )
        },
        "combined_points_per_25mm_cell": {
            key: float(value) for key, value in zip(
                ("q01", "q05", "q25", "q50", "q75", "q95", "q99"),
                np.quantile(combined_wet, [.01, .05, .25, .5, .75, .95, .99]),
            )
        },
        "review_additions": addition_metrics,
        "marked_density": marked_density,
        "scalar_quantiles": fields,
        "failures": failures,
    }


def _build_locked(args) -> None:
    run = args.run_dir
    run.mkdir(parents=True, exist_ok=True)
    (args.data_dir / "PatchRefinement/.invisible_places-ignore").touch()
    log_path = run / "build.log"
    with open(log_path, "a") as log_handle:
        def log(message):
            line = f"[v10] {message}"
            print(line, flush=True)
            log_handle.write(line + "\n")
            log_handle.flush()

        v9_manifest = json.loads((args.v9_run / "manifest.json").read_text())
        v9_verify = json.loads(
            (args.v9_run / "verification-report.json").read_text())
        if not v9_verify.get("verified"):
            raise RuntimeError("v9 height reference is not verified")
        v9_candidate = Path(v9_manifest["candidate"])
        if sha256_path(v9_candidate) != v9_manifest["candidate_sha256"]:
            raise RuntimeError("v9 candidate hash drift")

        inputs = build_inputs(args, v9_candidate)
        state_path, state = initialise_build_state(args, inputs)
        force_stage = bool(args.overwrite or args.resume)

        snapshot_root = run / "source-snapshots"
        expected_snapshots = [
            snapshot_root / "DataScene1" / f"Site1-{role}-{spacing}.ply"
            for role in ("SAND", "ROCK")
            for spacing in ("1mm", "5mm")
        ] + [
            snapshot_root / "DataScene1/Site1-WATER-5mm.ply",
            snapshot_root / "v9/surface-v9.npz",
            snapshot_root / "v9" / v9_candidate.name,
            snapshot_root / "site1_fossils_v10_review.json",
        ]
        if args.resume and reusable_stage(
                state, "source_snapshots", expected_snapshots):
            snapshots = state["stages"]["source_snapshots"]["detail"]
            log("resume: verified immutable APFS source snapshots")
        else:
            snapshots = create_source_snapshots(
                args, v9_candidate, inputs, force_stage
            )
            record_stage(
                state_path, state, "source_snapshots", expected_snapshots,
                snapshots,
            )
        working_data = Path(snapshots["data_dir"])
        working_v9_run = Path(snapshots["v9_run"])
        working_config = Path(snapshots["config"])
        config = load_config(working_config)
        surface = load_surface_reference(working_v9_run, config)

        # The donor grids are small compared with the staged clouds and are
        # deterministic from the hash-locked sources.  Rebuilding them keeps
        # resume state compact and avoids serialising lossy grid caches.
        donor_paths = [
            working_data / "Site1-SAND-5mm.ply",
            working_data / "Site1-ROCK-5mm.ply",
        ]
        donor_bundle = v9.build_donor_bundle(
            working_data, surface.grid, log, donor_paths=donor_paths
        )
        dense_input = run / "combined-2mm-downsample-input.ply"
        if args.resume and reusable_stage(state, "dense_input", [dense_input]):
            dense_input_report = state["stages"]["dense_input"]["detail"]
            log("resume: verified dense downsample input")
        else:
            dense_input_report = build_dense_downsample_input(
                dense_input, working_data, surface, donor_bundle,
                args.noise_scale, force_stage, log)
            record_stage(
                state_path, state, "dense_input", [dense_input],
                dense_input_report,
            )
        dense_selected = run / "combined-2mm-selected.ply"
        dense_downsample_report = run / "downsample-2mm.json"
        if args.resume and reusable_stage(
                state, "dense_downsample",
                [dense_selected, dense_downsample_report]):
            log("resume: verified dense Poisson selection")
        else:
            run_downsample(
                args.downsample, dense_input, dense_selected, DENSE_SPACING,
                dense_downsample_report, force_stage, log)
            record_stage(
                state_path, state, "dense_downsample",
                [dense_selected, dense_downsample_report],
            )
        dense_blocked = run / "combined-2mm-selected-allterrain.ply"
        if args.resume and reusable_stage(
                state, "dense_allterrain_block", [dense_blocked]):
            dense_block_report = state["stages"][
                "dense_allterrain_block"
            ]["detail"]
            log("resume: verified exact dense all-terrain blocker pass")
        else:
            dense_block_report = filter_water_against_all_base(
                dense_selected, dense_blocked, working_data, "1mm",
                DENSE_SPACING, surface, force_stage, log,
            )
            record_stage(
                state_path, state, "dense_allterrain_block", [dense_blocked],
                dense_block_report,
            )

        combined_analysed = run / "combined-2mm.analysis.ply"
        analysis_report = run / "cleanmesh-2mm-analysis.json"
        if args.resume and reusable_stage(
                state, "dense_analysis",
                [combined_analysed, analysis_report]):
            log("resume: verified combined terrain+WATER CleanMesh analysis")
        else:
            run_reduced_analysis(
                args.cleanmesh, dense_blocked, combined_analysed,
                analysis_report,
                force_stage, log)
            record_stage(
                state_path, state, "dense_analysis",
                [combined_analysed, analysis_report]
            )

        typed_water = run / "Site1-WATER-2mm.analysis-with-typeid.ply"
        if args.resume and reusable_stage(
                state, "dense_extract_typed", [typed_water]):
            dense_count = scalar_fill.inspect_fixed_stride_ply(
                typed_water
            ).count
            log("resume: verified analysed WATER extraction")
        else:
            dense_count = extract_scan_id(
                combined_analysed, typed_water, WATER_SCAN_ID,
                ["Site1 WATER v10 analysed with a measured terrain collar",
                 "Temporary scalar_TypeID is removed in the next staged pass"],
                force_stage,
            )
            record_stage(
                state_path, state, "dense_extract_typed", [typed_water],
                {"water_points": dense_count},
            )

        analysed = run / "Site1-WATER-2mm.analysis.ply"
        if args.resume and reusable_stage(
                state, "dense_strip_typeid", [analysed]):
            dense_count = scalar_fill.inspect_fixed_stride_ply(analysed).count
            log("resume: verified canonical-schema analysed WATER")
        else:
            dense_count = strip_to_expected_schema(
                typed_water, analysed,
                working_data / "Site1-SAND-5mm.ply",
                ["Site1 WATER v10 approximately 2 mm analysed geometry",
                 "Measured terrain collar supplied continuous shoreline neighbourhoods",
                 "Temporary material TypeID removed; canonical Scene1 schema restored"],
                force_stage,
            )
            record_stage(
                state_path, state, "dense_strip_typeid", [analysed],
                {"water_points": dense_count},
            )
        log(f"dense WATER analysed geometry {dense_count:,}")
        dense_candidate = run / "Site1-WATER-2mm.candidate.ply"
        scalar_report = run / "Site1-WATER-2mm.scalar-repair.json"
        final_scalar_report = run / "Site1-WATER-2mm.final-scalar-verification.json"
        normalizations, normalization_report = rock_normalizations(working_data)
        if args.resume and reusable_stage(
                state, "dense_scalar_repair",
                [dense_candidate, scalar_report, final_scalar_report]):
            combined_ranges = state["stages"][
                "dense_scalar_repair"
            ]["detail"]["combined_ranges"]
            log("resume: verified dense scalar repair and ROCK normalization")
        else:
            repair_water_scalars(
                analysed, dense_candidate, scalar_report, force_stage,
                normalizations,
            )
            combined_ranges = postprocess_water(
                dense_candidate, donor_bundle, normalizations)
            final_scalar_verification = verify_final_water_postprocess(
                analysed, dense_candidate
            )
            _atomic_write_json(
                final_scalar_report, final_scalar_verification
            )
            if not final_scalar_verification["verified"]:
                raise RuntimeError(
                    "final WATER scalar verification failed: "
                    + "; ".join(final_scalar_verification["failures"])
                )
            record_stage(
                state_path, state, "dense_scalar_repair",
                [dense_candidate, scalar_report, final_scalar_report],
                {
                    "combined_ranges": combined_ranges,
                    "rock_normalizations": normalizations,
                    "normalization_inference": normalization_report,
                },
            )
        log(f"dense postprocess complete: {combined_ranges}")

        coarse_input = run / "combined-5mm-downsample-input.ply"
        if args.resume and reusable_stage(
                state, "coarse_input", [coarse_input]):
            coarse_input_report = state["stages"]["coarse_input"]["detail"]
            log("resume: verified coarse downsample input")
        else:
            coarse_input_report = build_coarse_input(
                coarse_input, working_data, dense_candidate, surface,
                force_stage)
            record_stage(
                state_path, state, "coarse_input", [coarse_input],
                coarse_input_report,
            )
        coarse_selected = run / "combined-5mm-selected.ply"
        coarse_downsample_report = run / "downsample-5mm.json"
        if args.resume and reusable_stage(
                state, "coarse_downsample",
                [coarse_selected, coarse_downsample_report]):
            log("resume: verified coarse Poisson selection")
        else:
            run_downsample(
                args.downsample, coarse_input, coarse_selected, COARSE_SPACING,
                coarse_downsample_report, force_stage, log)
            record_stage(
                state_path, state, "coarse_downsample",
                [coarse_selected, coarse_downsample_report],
            )
        coarse_blocked = run / "combined-5mm-selected-allterrain.ply"
        if args.resume and reusable_stage(
                state, "coarse_allterrain_block", [coarse_blocked]):
            coarse_block_report = state["stages"][
                "coarse_allterrain_block"
            ]["detail"]
            log("resume: verified exact coarse all-terrain blocker pass")
        else:
            coarse_block_report = filter_water_against_all_base(
                coarse_selected, coarse_blocked, working_data, "5mm",
                COARSE_SPACING, surface, force_stage, log,
            )
            record_stage(
                state_path, state, "coarse_allterrain_block",
                [coarse_blocked], coarse_block_report,
            )
        coarse_candidate = run / "Site1-WATER-5mm.candidate.ply"
        if args.resume and reusable_stage(
                state, "coarse_extract", [coarse_candidate]):
            coarse_count = scalar_fill.inspect_fixed_stride_ply(
                coarse_candidate
            ).count
            log("resume: verified coarse WATER extraction")
        else:
            coarse_count = extract_scan_id(
                coarse_blocked, coarse_candidate, WATER_SCAN_ID,
                ["Site1 WATER v10 5 mm Poisson-selected candidate",
                 "Measured 5 mm SAND/ROCK were selected first during thinning",
                 "Scalar values retain the dense CleanMesh analysis"],
                force_stage,
            )
            record_stage(
                state_path, state, "coarse_extract", [coarse_candidate],
                {"water_points": coarse_count},
            )
        log(f"coarse WATER candidate {coarse_count:,}")

        manifest = {
            "version": 10,
            "created": dt.datetime.now().isoformat(timespec="seconds"),
            "data_dir": str(args.data_dir), "run_dir": str(run),
            "config": str(args.config),
            "source_snapshots": snapshots,
            "donor_roles": ["SAND-5mm", "ROCK-5mm"],
            "input_signature": state["input_signature"],
            "v9_height_reference": inputs["v9_candidate"],
            "source_water": inputs["source_water"],
            "terrain_sources": inputs["terrain_sources"],
            "dense_candidate": source_fingerprint(dense_candidate),
            "coarse_candidate": source_fingerprint(coarse_candidate),
            "dense_input": dense_input_report,
            "coarse_input": coarse_input_report,
            "downsample_reports": {
                "2mm": file_fingerprint(dense_downsample_report),
                "5mm": file_fingerprint(coarse_downsample_report),
            },
            "all_terrain_blocker_reports": {
                "2mm": dense_block_report,
                "5mm": coarse_block_report,
            },
            "analysis_report": file_fingerprint(analysis_report),
            "scalar_repair_report": file_fingerprint(scalar_report),
            "final_scalar_verification": file_fingerprint(
                final_scalar_report
            ),
            "rock_combined_normalizations": normalizations,
            "rock_normalization_inference": normalization_report,
            "combined_quantiles": combined_ranges,
            "noise": {
                "scale": args.noise_scale,
                "octave_wavelengths_m": (
                    NOISE_OCTAVE_WAVELENGTHS_M.tolist()
                ),
                "octave_rms_m": (
                    NOISE_OCTAVE_RMS_M * args.noise_scale
                ).tolist(),
                "octave_seeds": NOISE_OCTAVE_SEEDS.tolist(),
                "octave_angles_rad": NOISE_OCTAVE_ANGLES_RAD.tolist(),
                "micrograin_wavelengths_m": (
                    MICROGRAIN_WAVELENGTHS_M.tolist()
                ),
                "micrograin_peak_amplitudes_m": (
                    MICROGRAIN_AMPLITUDES_M * args.noise_scale
                ).tolist(),
                "soft_clip_start_m": NOISE_SOFT_CLIP_START_M,
                "soft_clip_limit_m": NOISE_SOFT_CLIP_LIMIT_M,
                "unclipped_rms_m": (
                    NOISE_UNCLIPPED_RMS_M * args.noise_scale
                ),
            },
            "installed": False,
        }
        _atomic_write_json(run / "manifest.json", manifest)
        log("manifest written")

    verify(args, acquire_lock=False)


def build(args) -> None:
    require_runtime_dependencies()
    free = shutil.disk_usage(args.run_dir.parent).free
    minimum_free = 120 * 1024**3
    if free < minimum_free:
        raise RuntimeError(
            f"v10 requires at least 120 GiB free; found {free / 1024**3:.1f} GiB"
        )
    with build_lock(args.run_dir):
        _build_locked(args)


def require_runtime_dependencies() -> None:
    """Fail before hashing/snapshotting if the numeric runtime is incomplete."""

    try:
        import scipy  # noqa: F401
        from scipy.spatial import cKDTree  # noqa: F401
        from scipy.sparse import coo_matrix  # noqa: F401
    except ImportError as error:
        raise RuntimeError(
            "Scene1 v10 requires a Python runtime containing SciPy; "
            "run it with the project numeric environment before starting "
            "the large staged build"
        ) from error


def _verify_fingerprinted_json(
    entry: Mapping,
    label: str,
    failures: list[str],
) -> dict | None:
    """Verify a small manifested JSON artifact before trusting its contents."""

    try:
        path = Path(entry["path"])
    except (KeyError, TypeError):
        failures.append(f"{label}: malformed manifest fingerprint")
        return None
    if not path.exists():
        failures.append(f"{label}: missing {path}")
        return None
    if path.stat().st_size != entry.get("bytes"):
        failures.append(f"{label}: byte-size drift")
        return None
    if sha256_path(path) != entry.get("sha256"):
        failures.append(f"{label}: hash drift")
        return None
    try:
        return json.loads(path.read_text())
    except (OSError, json.JSONDecodeError) as error:
        failures.append(f"{label}: invalid JSON ({error})")
        return None


def verify_pipeline_provenance(
    run: Path,
    manifest: Mapping,
    config: Mapping,
) -> dict:
    """Validate the immutable build chain and CleanMesh method reports."""

    failures = []
    state_path = run / "build-state.json"
    if not state_path.exists():
        return {"verified": False, "failures": ["missing build-state.json"]}
    try:
        state = json.loads(state_path.read_text())
    except json.JSONDecodeError as error:
        return {
            "verified": False,
            "failures": [f"invalid build-state.json ({error})"],
        }
    if state.get("schema_version") != BUILD_STATE_SCHEMA:
        failures.append("build-state schema mismatch")
    if state.get("input_signature") != manifest.get("input_signature"):
        failures.append("build-state/manifest input signature mismatch")
    if _json_sha256(state.get("inputs", {})) != state.get("input_signature"):
        failures.append("build-state input payload does not match its signature")

    snapshot_stage = state.get("stages", {}).get("source_snapshots", {})
    snapshot_outputs = [
        item
        for item in snapshot_stage.get("outputs", [])
        if isinstance(item, Mapping) and item.get("path")
    ]
    snapshot_paths = [Path(item["path"]) for item in snapshot_outputs]
    snapshots = manifest.get("source_snapshots", {})
    snapshot_data = Path(snapshots.get("data_dir", ""))
    snapshot_v9 = Path(snapshots.get("v9_run", ""))
    snapshot_config = Path(snapshots.get("config", ""))
    original_inputs = state.get("inputs", {})
    expected_snapshot_inputs = {
        snapshot_data / f"Site1-{role}-{spacing}.ply":
            original_inputs.get("terrain_sources", {}).get(
                f"{role}-{spacing}", {}
            )
        for role in ("SAND", "ROCK")
        for spacing in ("1mm", "5mm")
    }
    expected_snapshot_inputs[
        snapshot_data / "Site1-WATER-5mm.ply"
    ] = original_inputs.get("source_water", {})
    expected_snapshot_inputs[
        snapshot_v9 / "surface-v9.npz"
    ] = original_inputs.get("v9_surface", {})
    expected_snapshot_inputs[
        snapshot_v9 / Path(
            original_inputs.get("v9_candidate", {}).get("path", "missing")
        ).name
    ] = original_inputs.get("v9_candidate", {})
    expected_snapshot_inputs[snapshot_config] = original_inputs.get(
        "config", {}
    )
    if set(snapshot_paths) != set(expected_snapshot_inputs):
        failures.append("source snapshot set is incomplete or unexpected")
    if not snapshot_paths or not reusable_stage(
        state, "source_snapshots", snapshot_paths
    ):
        failures.append("immutable source snapshots failed hash verification")
    output_by_path = {
        Path(item["path"]): item for item in snapshot_outputs
    }
    for path, expected in expected_snapshot_inputs.items():
        actual = output_by_path.get(path)
        if actual is None:
            continue
        if expected.get("exists") is False:
            continue
        try:
            _assert_fingerprint_content(
                actual, expected, f"source snapshot {path.name}"
            )
        except RuntimeError as error:
            failures.append(str(error))

    acceptance = config.get("acceptance", {})
    spacing_by_label = {
        "2mm": float(
            acceptance.get("minimum_dense_spacing_m", DENSE_SPACING)
        ),
        "5mm": float(
            acceptance.get("minimum_coarse_spacing_m", COARSE_SPACING)
        ),
    }
    downsample = {}
    for label, required_spacing in spacing_by_label.items():
        report = _verify_fingerprinted_json(
            manifest.get("downsample_reports", {}).get(label, {}),
            f"{label} downsample report",
            failures,
        )
        downsample[label] = report
        if report is None:
            continue
        if report.get("method") != "greedy_spatial_minimum_distance":
            failures.append(f"{label}: unexpected downsample method")
        if not math.isclose(
            float(report.get("minimum_spacing_m", -1.0)),
            required_spacing,
            rel_tol=0.0,
            abs_tol=1.0e-12,
        ):
            failures.append(f"{label}: downsample spacing mismatch")
        if report.get("priority_scan_id") != int(BASE_PRIORITY_SCAN_ID):
            failures.append(f"{label}: terrain priority ScanID mismatch")
        if float(report.get("priority_minimum_spacing_m", -1.0)) + 1.0e-12 < required_spacing:
            failures.append(f"{label}: priority spacing is below target")
        if int(report.get("non_finite_positions", -1)) != 0:
            failures.append(f"{label}: downsample saw non-finite positions")
        if int(report.get("output_points", 0)) <= 0:
            failures.append(f"{label}: downsample output is empty")

        blocker = manifest.get("all_terrain_blocker_reports", {}).get(
            label, {}
        )
        candidate = manifest.get(
            "dense_candidate" if label == "2mm" else "coarse_candidate",
            {},
        )
        if int(blocker.get("kept_water_points", -1)) != int(
            candidate.get("points", -2)
        ):
            failures.append(f"{label}: blocker/candidate point count mismatch")
        if not math.isclose(
            float(blocker.get("minimum_spacing_m", -1.0)),
            required_spacing,
            rel_tol=0.0,
            abs_tol=1.0e-12,
        ):
            failures.append(f"{label}: all-terrain blocker spacing mismatch")
        selected_water = int(report.get("output_points", 0)) - int(
            report.get("priority_output_points", 0)
        )
        if int(blocker.get("source_water_points", -1)) != selected_water:
            failures.append(
                f"{label}: downsample/blocker source count mismatch"
            )

    analysis = _verify_fingerprinted_json(
        manifest.get("analysis_report", {}),
        "CleanMesh analysis report",
        failures,
    )
    if analysis is not None:
        expected = manifest.get("all_terrain_blocker_reports", {}).get(
            "2mm", {}
        ).get("output_total_points")
        if int(analysis.get("input_points", -1)) != int(expected or -2):
            failures.append("CleanMesh analysis input count mismatch")
        if int(analysis.get("base_voxels", 0)) <= 0:
            failures.append("CleanMesh analysis has no base voxels")
        valid_proxy = analysis.get("valid_proxy_voxels")
        if isinstance(valid_proxy, Mapping):
            for scale in ("fine", "medium"):
                if int(valid_proxy.get(scale, 0)) <= 0:
                    failures.append(
                        f"CleanMesh analysis has no valid {scale} proxies"
                    )
        elif int(analysis.get("valid_analysis_voxels", 0)) <= 0:
            failures.append(
                "CleanMesh analysis has no recognised valid-voxel statistic"
            )
        fine_voxel = analysis.get("scales", {}).get("fine", {}).get(
            "voxel_m"
        )
        if fine_voxel is not None and not math.isclose(
            float(fine_voxel), 0.003, rel_tol=0.0, abs_tol=1.0e-12
        ):
            failures.append("CleanMesh fine base voxel is not 3 mm")
        fields = analysis.get("fields", {})
        if fields:
            for metric in PHYSICAL_METRICS:
                for scale in SCALES:
                    name = f"A_R_{metric}_{scale}"
                    if int(fields.get(name, {}).get("finite_count", 0)) <= 0:
                        failures.append(
                            f"CleanMesh field {name} has no finite values"
                        )

    scalar = _verify_fingerprinted_json(
        manifest.get("scalar_repair_report", {}),
        "WATER scalar-repair report",
        failures,
    )
    if scalar is not None and not scalar.get("verification", {}).get(
        "verified", False
    ):
        failures.append("WATER scalar repair did not verify")
    final_scalar = _verify_fingerprinted_json(
        manifest.get("final_scalar_verification", {}),
        "final WATER scalar report",
        failures,
    )
    if final_scalar is not None and not final_scalar.get("verified", False):
        failures.append("final WATER scalar postprocess did not verify")

    return {
        "verified": not failures,
        "source_snapshot_files": len(snapshot_paths),
        "downsample": downsample,
        "analysis": analysis,
        "scalar_repair_verified": (
            scalar.get("verification", {}).get("verified")
            if scalar is not None else None
        ),
        "final_scalar_verified": (
            final_scalar.get("verified")
            if final_scalar is not None else None
        ),
        "failures": failures,
    }


def verify(args, acquire_lock: bool = True) -> dict:
    require_runtime_dependencies()
    if acquire_lock:
        with build_lock(args.run_dir):
            return verify(args, acquire_lock=False)
    run = args.run_dir
    manifest = json.loads((run / "manifest.json").read_text())
    snapshots = manifest["source_snapshots"]
    snapshot_data = Path(snapshots["data_dir"])
    snapshot_v9_run = Path(snapshots["v9_run"])
    snapshot_config = Path(snapshots["config"])
    config = load_config(snapshot_config)
    surface = load_surface_reference(snapshot_v9_run, config)
    dense = Path(manifest["dense_candidate"]["path"])
    coarse = Path(manifest["coarse_candidate"]["path"])
    reports = {
        "2mm": audit_candidate(
            dense, snapshot_data, surface, DENSE_SPACING, config,
            noise_scale=float(manifest["noise"]["scale"]),
        ),
        "5mm": audit_candidate(
            coarse, snapshot_data, surface, COARSE_SPACING, config,
            noise_scale=float(manifest["noise"]["scale"]),
        ),
    }
    provenance = verify_pipeline_provenance(run, manifest, config)
    failures = list(provenance["failures"])
    for label, report in reports.items():
        failures.extend(f"{label}: {failure}" for failure in report["failures"])
        expected = manifest[
            "dense_candidate" if label == "2mm" else "coarse_candidate"
        ]
        if report["sha256"] != expected["sha256"]:
            failures.append(f"{label}: candidate hash drift")
    result = {
        "created": dt.datetime.now().isoformat(timespec="seconds"),
        "verified": not failures,
        "pipeline_provenance": provenance,
        "reports": reports,
        "failures": failures,
    }
    _atomic_write_json(run / "verification-report.json", result)
    print(json.dumps(result, indent=1), flush=True)
    if failures:
        raise RuntimeError("v10 verification failed: " + "; ".join(failures))
    return result


def _install_locked(args) -> None:
    if v9.app_running():
        raise SystemExit("refusing: invisible_places is running")
    report = verify(args, acquire_lock=False)
    if not report["verified"]:
        raise RuntimeError("verification did not pass")
    manifest_path = args.run_dir / "manifest.json"
    manifest = json.loads(manifest_path.read_text())
    current_v9_manifest = json.loads((args.v9_run / "manifest.json").read_text())
    current_inputs = build_inputs(
        args, Path(current_v9_manifest["candidate"])
    )
    if _json_sha256(current_inputs) != manifest["input_signature"]:
        raise RuntimeError(
            "canonical/config/executable inputs changed after v10 build"
        )
    canonical = args.data_dir / "Site1-WATER-5mm.ply"
    old = args.data_dir / "Site1-WATER-5mm-old01.ply"
    if old.exists():
        raise SystemExit(f"refusing to overwrite preserved source {old}")
    if sha256_path(canonical) != manifest["source_water"]["sha256"]:
        raise RuntimeError("canonical WATER changed after v10 build")
    candidate = Path(manifest["coarse_candidate"]["path"])
    staged_copy = canonical.with_name(
        f".{canonical.name}.v10-install.partial"
    )
    _clone_or_copy_file(candidate, staged_copy, overwrite=True)
    if sha256_path(staged_copy) != manifest["coarse_candidate"]["sha256"]:
        staged_copy.unlink(missing_ok=True)
        raise RuntimeError("staged 5 mm WATER install hash mismatch")
    dense_canonical = None
    dense_staged = None
    if args.install_dense:
        dense_canonical = args.data_dir / "Site1-WATER-2mm.ply"
        if dense_canonical.exists():
            staged_copy.unlink(missing_ok=True)
            raise RuntimeError(f"refusing to overwrite {dense_canonical}")
        dense_staged = dense_canonical.with_name(
            f".{dense_canonical.name}.v10-install.partial"
        )
        _clone_or_copy_file(
            Path(manifest["dense_candidate"]["path"]),
            dense_staged,
            overwrite=True,
        )
        if sha256_path(dense_staged) != manifest["dense_candidate"]["sha256"]:
            staged_copy.unlink(missing_ok=True)
            dense_staged.unlink(missing_ok=True)
            raise RuntimeError("staged 2 mm WATER install hash mismatch")
    # Candidate clone/hash can be slow on a busy volume. Recheck the exact
    # canonical preconditions immediately before the first irreversible name
    # change, while still holding the process lock.
    if old.exists():
        staged_copy.unlink(missing_ok=True)
        if dense_staged is not None:
            dense_staged.unlink(missing_ok=True)
        raise RuntimeError(f"preserved source appeared during install: {old}")
    if not canonical.exists() or sha256_path(canonical) != manifest[
        "source_water"
    ]["sha256"]:
        staged_copy.unlink(missing_ok=True)
        if dense_staged is not None:
            dense_staged.unlink(missing_ok=True)
        raise RuntimeError("canonical WATER changed while staging install")
    if dense_canonical is not None and dense_canonical.exists():
        staged_copy.unlink(missing_ok=True)
        dense_staged.unlink(missing_ok=True)
        raise RuntimeError(
            f"dense canonical appeared while staging install: {dense_canonical}"
        )
    if v9.app_running():
        staged_copy.unlink(missing_ok=True)
        if dense_staged is not None:
            dense_staged.unlink(missing_ok=True)
        raise RuntimeError(
            "invisible_places started while staging; install aborted"
        )
    canonical.replace(old)
    try:
        staged_copy.replace(canonical)
        if sha256_path(canonical) != manifest["coarse_candidate"]["sha256"]:
            raise RuntimeError("installed 5 mm WATER hash mismatch")
        if dense_canonical is not None:
            dense_staged.replace(dense_canonical)
            if sha256_path(dense_canonical) != manifest["dense_candidate"]["sha256"]:
                raise RuntimeError("installed 2 mm WATER hash mismatch")
    except Exception:
        canonical.unlink(missing_ok=True)
        old.replace(canonical)
        if dense_canonical is not None:
            dense_canonical.unlink(missing_ok=True)
        staged_copy.unlink(missing_ok=True)
        if dense_staged is not None:
            dense_staged.unlink(missing_ok=True)
        raise
    manifest["installed"] = True
    manifest["installed_at"] = dt.datetime.now().isoformat(timespec="seconds")
    manifest["canonical_5mm"] = str(canonical)
    manifest["old01"] = str(old)
    manifest["canonical_2mm"] = (
        str(dense_canonical) if dense_canonical is not None else None
    )
    _atomic_write_json(manifest_path, manifest)


def install(args) -> None:
    with build_lock(args.run_dir):
        _install_locked(args)


def _restore_locked(args) -> None:
    if v9.app_running():
        raise SystemExit("refusing: invisible_places is running")
    manifest_path = args.run_dir / "manifest.json"
    manifest = json.loads(manifest_path.read_text())
    canonical = args.data_dir / "Site1-WATER-5mm.ply"
    old = args.data_dir / "Site1-WATER-5mm-old01.ply"
    if not old.exists():
        raise SystemExit(f"no preserved source at {old}")
    staged_installed = args.run_dir / "Site1-WATER-5mm.installed.ply"
    if staged_installed.exists():
        raise SystemExit(f"refusing to overwrite {staged_installed}")
    if not canonical.exists():
        raise RuntimeError(f"installed canonical is missing: {canonical}")
    if sha256_path(old) != manifest["source_water"]["sha256"]:
        raise RuntimeError("preserved old01 WATER hash mismatch")
    if sha256_path(canonical) != manifest["coarse_candidate"]["sha256"]:
        raise RuntimeError("canonical WATER is not the manifested v10 file")
    dense = args.data_dir / "Site1-WATER-2mm.ply"
    dense_archive = args.run_dir / "Site1-WATER-2mm.installed.ply"
    move_dense = dense.exists() and manifest.get("canonical_2mm") == str(dense)
    if move_dense:
        if dense_archive.exists():
            raise RuntimeError(f"refusing to overwrite {dense_archive}")
        if sha256_path(dense) != manifest["dense_candidate"]["sha256"]:
            raise RuntimeError("canonical 2 mm WATER hash mismatch")
    if v9.app_running():
        raise RuntimeError(
            "invisible_places started while checking restore; restore aborted"
        )
    canonical.replace(staged_installed)
    old_moved = dense_moved = False
    try:
        old.replace(canonical)
        old_moved = True
        if move_dense:
            dense.replace(dense_archive)
            dense_moved = True
        if sha256_path(canonical) != manifest["source_water"]["sha256"]:
            raise RuntimeError("restored WATER source hash mismatch")
    except Exception:
        if dense_moved:
            dense_archive.replace(dense)
        if old_moved:
            canonical.replace(old)
        staged_installed.replace(canonical)
        raise
    manifest["installed"] = False
    manifest["restored_at"] = dt.datetime.now().isoformat(timespec="seconds")
    _atomic_write_json(manifest_path, manifest)


def restore(args) -> None:
    with build_lock(args.run_dir):
        _restore_locked(args)


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    result.add_argument("stage", choices=("build", "verify", "install", "restore"))
    result.add_argument("--data-dir", type=Path, default=DEFAULT_DATA)
    result.add_argument("--v9-run", type=Path, default=DEFAULT_V9_RUN)
    result.add_argument("--run-dir", type=Path, default=DEFAULT_RUN)
    result.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    result.add_argument("--cleanmesh", type=Path, default=DEFAULT_CLEANMESH)
    result.add_argument("--downsample", type=Path, default=DEFAULT_DOWNSAMPLE)
    result.add_argument("--noise-scale", type=float, default=1.0)
    result.add_argument("--overwrite", action="store_true")
    result.add_argument(
        "--resume", action="store_true",
        help="reuse only hash-verified stages from a matching v10 build",
    )
    result.add_argument("--install-dense", action="store_true")
    return result


def main(argv=None) -> int:
    args = parser().parse_args(argv)
    if not np.isfinite(args.noise_scale) or args.noise_scale <= 0:
        raise SystemExit("--noise-scale must be finite and positive")
    if args.overwrite and args.resume:
        raise SystemExit("choose either --overwrite or --resume, not both")
    if args.stage == "build":
        build(args)
    elif args.stage == "verify":
        verify(args)
    elif args.stage == "install":
        install(args)
    else:
        restore(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
