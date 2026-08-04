#!/usr/bin/env python3
"""Refine visible Scene3 terrestrial-LiDAR scanner and scan-edge patches.

The tool is deliberately conservative and reversible:

* only ScanID 10/11 records inside the three CleanMesh component footprints
  may be changed;
* those records retain their geometry and become ScanID 12;
* measured density additions are appended as ScanID 13;
* the Patch 04 ScanID 9 density edge is softened only with appended points;
* complete original records and vertex indices are retained for byte-exact
  restoration without keeping four full duplicate clouds.

Run ``build`` and ``verify`` before rendering the generated validation project.
``install`` performs a transactional canonical-file swap and invokes the GUI
smoke before discarding its temporary full-file rollback copies.
"""

from __future__ import annotations

import argparse
import collections
import dataclasses
import datetime as dt
import hashlib
import json
import math
import os
import re
import shutil
import struct
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Iterable, Iterator, Sequence

import numpy as np

try:
    import cv2
except ImportError:  # Unit/rollback-only environments do not need image ops.
    cv2 = None  # type: ignore[assignment]


PLY_TYPES: dict[str, str] = {
    "char": "i1", "int8": "i1", "uchar": "u1", "uint8": "u1",
    "short": "<i2", "int16": "<i2", "ushort": "<u2", "uint16": "<u2",
    "int": "<i4", "int32": "<i4", "uint": "<u4", "uint32": "<u4",
    "float": "<f4", "float32": "<f4", "double": "<f8", "float64": "<f8",
}

BASE_ALIASES: dict[str, tuple[str, ...]] = {
    "x": ("x",), "y": ("y",), "z": ("z",),
    "red": ("red", "r"), "green": ("green", "g"), "blue": ("blue", "b"),
    "nx": ("nx", "normal_x"), "ny": ("ny", "normal_y"), "nz": ("nz", "normal_z"),
    "intensity": ("scalar_Intensity", "Intensity"),
    "ranges": ("scalar_Ranges", "Ranges"),
    "composite": ("scalar_Composite", "Composite"),
    "scan_id": ("scalar_ScanID", "ScanID", "scan_id"),
    "roughness": ("scalar_Roughness", "Roughness"),
    "interest": ("scalar_Interest", "Interest"),
}

EDITABLE_FIELDS = ("red", "green", "blue", "intensity", "composite", "scan_id", "roughness")
SMOOTH_SCALARS = ("intensity", "composite", "roughness")
SOURCE_SCAN_IDS = (10, 11)
REPLACEMENT_SCAN_ID = 12
ADDITION_SCAN_ID = 13
COMPONENT_CELL_METRES = 0.020
SUPPORT_Z_MARGIN_METRES = 0.030
REFERENCE_MARGIN_METRES = 0.100
MAX_PAIR_DISTANCE_METRES = 0.080
PAIR_Z_TOLERANCE_METRES = 0.030
PAIR_NORMAL_DOT_MIN = 0.50
LOCAL_SMOOTHING_METRES = 0.100
NEAREST_AR_MAX_METRES = 0.005
SPACINGS_MM = (1, 2, 3, 5)
# CleanMesh uses a 0.65 random-packing spacing when it samples a reconstructed
# surface.  It provides enough candidate headroom to avoid imprinting voxel
# shells while the later density quota controls the rendered coverage.
RANDOM_PACKING_SPACING_FACTOR = 0.65
SOURCE_CLEARANCE_FACTOR = 0.65
# Patch 02 and Patch 03 already blend well with a small coverage reserve under
# Projector-01. Patch 01 is different: its host scan is also sparse across the
# scanner footprint, so the per-scan cap hides a large total-density deficit.
# Match the observed total density there and let the fixed 2 mm/1 mm camera
# render be the final gate.
DENSITY_DEFICIT_FILL_FRACTION = 0.80
PATCH_01_DENSITY_DEFICIT_FILL_FRACTION = 1.00
PATCH_04_SOURCE_SCAN_ID = 9
PATCH_04_TARGET_XY = np.array([307.0819396972656, 104.03511047363281])
PATCH_04_BLEND_WIDTH_METRES = 0.220
PATCH_04_JAGGED_MIN_WIDTH_METRES = 0.080
PATCH_04_JAGGED_BASE_WIDTH_METRES = 0.200
PATCH_04_JAGGED_MAX_WIDTH_METRES = 0.440
PATCH_04_JAGGED_CORE_WIDTH_METRES = 0.030
PATCH_04_JAGGED_PLANARITY_METRES = 0.040
PATCH_04_JAGGED_NOISE_SEED = 9041309
PATCH_04_EDGE_FIT_RADIUS_METRES = 0.450
PATCH_04_EDGE_END_MARGIN_METRES = 0.030
PATCH_04_GEOMETRY_MAX_DISTANCE_METRES = 0.002
PATCH_04_GEOMETRY_NORMAL_DOT_MIN = 0.75
PATCH_04_RGB_NEIGHBOURS = 8
PATCH_04_RGB_RADIUS_METRES = 0.015
DISCOVERY_IGNORE_MARKER = ".invisible_places-ignore"


def _normalised_name(name: str) -> str:
    return re.sub(r"[^a-z0-9]", "", name.lower())


def _field_by_alias(names: Sequence[str], aliases: Iterable[str], *, suffix: str | None = None) -> str | None:
    wanted = {_normalised_name(value) for value in aliases}
    suffix_norm = _normalised_name(suffix) if suffix else None
    for name in names:
        normalised = _normalised_name(name)
        if normalised in wanted or (suffix_norm and normalised.endswith(suffix_norm)):
            return name
    return None


@dataclasses.dataclass(frozen=True)
class PlyInfo:
    path: Path
    header: bytes
    vertex_count: int
    dtype: np.dtype
    property_names: tuple[str, ...]
    fields: dict[str, str]

    @property
    def header_size(self) -> int:
        return len(self.header)

    @property
    def stride(self) -> int:
        return int(self.dtype.itemsize)

    @property
    def ar_fields(self) -> tuple[str, ...]:
        return tuple(name for name in self.property_names if name.startswith("scalar_A_R_"))


def read_ply_info(
    path: Path,
    required_fields: Sequence[str] | None = None,
) -> PlyInfo:
    path = path.expanduser().resolve()
    header_parts: list[bytes] = []
    with path.open("rb") as stream:
        first = stream.readline()
        if first.rstrip(b"\r\n") != b"ply":
            raise ValueError(f"{path} is not a PLY file")
        header_parts.append(first)
        while True:
            line = stream.readline()
            if not line:
                raise ValueError(f"{path} has no end_header line")
            header_parts.append(line)
            if line.rstrip(b"\r\n") == b"end_header":
                break
    header = b"".join(header_parts)
    format_name: str | None = None
    current_element: str | None = None
    elements: list[tuple[str, int]] = []
    properties: list[tuple[str, str]] = []
    for line in header.decode("ascii").splitlines():
        parts = line.split()
        if not parts:
            continue
        if parts[0] == "format" and len(parts) >= 2:
            format_name = parts[1]
        elif parts[0] == "element" and len(parts) == 3:
            current_element = parts[1]
            elements.append((current_element, int(parts[2])))
        elif parts[0] == "property" and current_element == "vertex":
            if len(parts) >= 2 and parts[1] == "list":
                raise ValueError("Variable-length vertex properties are unsupported")
            if len(parts) != 3 or parts[1] not in PLY_TYPES:
                raise ValueError(f"Unsupported vertex property: {line}")
            properties.append((parts[2], PLY_TYPES[parts[1]]))
    if format_name != "binary_little_endian":
        raise ValueError(f"{path} is not binary_little_endian")
    vertices = [count for name, count in elements if name == "vertex"]
    if len(vertices) != 1:
        raise ValueError(f"{path} must contain exactly one vertex element")
    non_vertex = [(name, count) for name, count in elements if name != "vertex" and count]
    if non_vertex:
        raise ValueError(f"{path} has unsupported non-vertex elements: {non_vertex}")
    dtype = np.dtype(properties, align=False)
    vertex_count = vertices[0]
    expected_size = len(header) + vertex_count * dtype.itemsize
    actual_size = path.stat().st_size
    if actual_size != expected_size:
        raise ValueError(f"{path} size mismatch: expected {expected_size:,}, found {actual_size:,}")
    names = tuple(name for name, _ in properties)
    fields: dict[str, str] = {}
    for canonical, aliases in BASE_ALIASES.items():
        value = _field_by_alias(names, aliases, suffix="scanid" if canonical == "scan_id" else None)
        if value is not None:
            fields[canonical] = value
    required = tuple(BASE_ALIASES) if required_fields is None else tuple(required_fields)
    unknown_required = sorted(set(required) - set(BASE_ALIASES))
    if unknown_required:
        raise ValueError(f"Unknown required PLY aliases: {', '.join(unknown_required)}")
    missing = [name for name in required if name not in fields]
    if missing:
        raise ValueError(f"{path} is missing required properties: {', '.join(missing)}")
    for channel in ("red", "green", "blue"):
        if channel not in fields:
            continue
        if dtype.fields[fields[channel]][0] != np.dtype("u1"):
            raise ValueError(f"{path}: {fields[channel]} must be uchar")
    return PlyInfo(path, header, vertex_count, dtype, names, fields)


def records(info: PlyInfo, mode: str = "r") -> np.memmap:
    return np.memmap(info.path, dtype=info.dtype, mode=mode, offset=info.header_size, shape=(info.vertex_count,))


def patch_header_count(header: bytes, count: int) -> bytes:
    pattern = re.compile(rb"(?m)^(element vertex )(\d+)([ \t]*)(\r?)$")
    match = pattern.search(header)
    if match is None:
        raise ValueError("Could not patch the PLY vertex count")
    digits = str(count).encode("ascii")
    available_width = len(match.group(2)) + len(match.group(3))
    padding = b" " * max(0, available_width - len(digits))
    replacement = match.group(1) + digits + padding + match.group(4)
    return header[:match.start()] + replacement + header[match.end():]


def sha256_path(path: Path, chunk_bytes: int = 16 * 1024 * 1024) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(chunk_bytes):
            digest.update(chunk)
    return digest.hexdigest()


def _scan_ids(chunk: np.ndarray, info: PlyInfo) -> tuple[np.ndarray, np.ndarray]:
    raw = np.asarray(chunk[info.fields["scan_id"]], dtype=np.float64)
    valid = np.isfinite(raw)
    rounded = np.full(raw.shape, -32768, dtype=np.int16)
    rounded[valid] = np.rint(raw[valid]).astype(np.int16)
    return rounded, valid


def _pack_cells(ix: np.ndarray, iy: np.ndarray) -> np.ndarray:
    x64 = np.asarray(ix, dtype=np.int64)
    y64 = np.asarray(iy, dtype=np.int64)
    return (x64 << np.int64(32)) | (y64 & np.int64(0xFFFFFFFF))


def _pack_cell_scalar(ix: int, iy: int) -> int:
    return (int(ix) << 32) | (int(iy) & 0xFFFFFFFF)


def _unpack_cells(keys: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    keys = np.asarray(keys, dtype=np.int64)
    return keys >> np.int64(32), (keys.astype(np.uint64) & np.uint64(0xFFFFFFFF)).astype(np.uint32).view(np.int32).astype(np.int64)


def _pack_voxels(x: np.ndarray, y: np.ndarray, z: np.ndarray, spacing: float) -> np.ndarray:
    quantised = [np.floor(np.asarray(axis, dtype=np.float64) / spacing).astype(np.int64) for axis in (x, y, z)]
    bias = np.int64(1 << 20)
    mask = np.int64((1 << 21) - 1)
    for values in quantised:
        shifted = values + bias
        if np.any(shifted < 0) or np.any(shifted > mask):
            raise ValueError("Point coordinate is outside the reversible 21-bit voxel-key range")
    return ((quantised[0] + bias) << np.int64(42)) | ((quantised[1] + bias) << np.int64(21)) | (quantised[2] + bias)


def _splitmix64(values: np.ndarray) -> np.ndarray:
    x = np.asarray(values, dtype=np.uint64).copy()
    x += np.uint64(0x9E3779B97F4A7C15)
    x = (x ^ (x >> np.uint64(30))) * np.uint64(0xBF58476D1CE4E5B9)
    x = (x ^ (x >> np.uint64(27))) * np.uint64(0x94D049BB133111EB)
    return x ^ (x >> np.uint64(31))


def _ellipse_kernel(radius: int) -> np.ndarray:
    if cv2 is None:
        raise RuntimeError("OpenCV (cv2) is required to build scanner-patch candidates")
    radius = max(1, int(radius))
    return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * radius + 1, 2 * radius + 1))


@dataclasses.dataclass
class ComponentSupport:
    camera_name: str
    cleanmesh_index: int
    host_scan_id: int
    bounds_min: np.ndarray
    bounds_max: np.ndarray
    dense_records: np.ndarray
    cell_keys: np.ndarray
    cell_z_min: np.ndarray
    cell_z_max: np.ndarray
    origin_ix: int
    origin_iy: int
    mask: np.ndarray
    boundary: np.ndarray
    outside_ring: np.ndarray

    @property
    def name(self) -> str:
        return f"Patch{self.cleanmesh_index}"

    def grid_coordinates(self, x: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        ix = np.floor(np.asarray(x, dtype=np.float64) / COMPONENT_CELL_METRES).astype(np.int64)
        iy = np.floor(np.asarray(y, dtype=np.float64) / COMPONENT_CELL_METRES).astype(np.int64)
        gx, gy = ix - self.origin_ix, iy - self.origin_iy
        within = (gx >= 0) & (gx < self.mask.shape[1]) & (gy >= 0) & (gy < self.mask.shape[0])
        return gx, gy, within

    def contains(self, x: np.ndarray, y: np.ndarray, z: np.ndarray) -> np.ndarray:
        ix = np.floor(np.asarray(x, dtype=np.float64) / COMPONENT_CELL_METRES).astype(np.int64)
        iy = np.floor(np.asarray(y, dtype=np.float64) / COMPONENT_CELL_METRES).astype(np.int64)
        keys = _pack_cells(ix, iy)
        positions = np.searchsorted(self.cell_keys, keys)
        present = positions < len(self.cell_keys)
        safe = np.minimum(positions, max(len(self.cell_keys) - 1, 0))
        if len(self.cell_keys):
            present &= self.cell_keys[safe] == keys
        else:
            present[:] = False
        result = present.copy()
        if np.any(present):
            zp = np.asarray(z, dtype=np.float64)[present]
            indices = safe[present]
            result[present] &= (
                (zp >= self.cell_z_min[indices] - SUPPORT_Z_MARGIN_METRES)
                & (zp <= self.cell_z_max[indices] + SUPPORT_Z_MARGIN_METRES)
            )
        return result


@dataclasses.dataclass
class CellStats:
    ix: np.ndarray
    iy: np.ndarray
    xyz: np.ndarray
    normals: np.ndarray
    values: np.ndarray
    counts: np.ndarray


@dataclasses.dataclass
class CorrectionModel:
    component_index: int
    source_scan_id: int
    host_scan_id: int
    gain: np.ndarray
    offset: np.ndarray
    residual_field: np.ndarray
    clip_low: np.ndarray
    clip_high: np.ndarray
    pair_source: np.ndarray
    pair_target: np.ndarray
    metrics: dict[str, Any]


@dataclasses.dataclass
class LinearEdgeSupport:
    component: ComponentSupport
    source_scan_id: int
    edge_origin_xy: np.ndarray
    edge_direction_xy: np.ndarray
    outside_normal_xy: np.ndarray
    signed_distance: np.ndarray
    along_distance: np.ndarray
    high_density_mask: np.ndarray
    transition_mask: np.ndarray
    along_min: float
    along_max: float
    blend_width_metres: float
    fit_rms_metres: float
    edge_profile: str = "linear"
    fade_width_map: np.ndarray | None = None
    surface_planarity_map: np.ndarray | None = None
    core_width_metres: float = 0.0
    noise_seed: int = 0


def geometry_bounds(info: PlyInfo, chunk_records: int) -> tuple[np.ndarray, np.ndarray]:
    source = records(info)
    lower = np.full(3, np.inf, dtype=np.float64)
    upper = np.full(3, -np.inf, dtype=np.float64)
    for start in range(0, info.vertex_count, chunk_records):
        end = min(start + chunk_records, info.vertex_count)
        chunk = source[start:end]
        xyz = np.column_stack([
            chunk[info.fields[channel]]
            for channel in ("x", "y", "z")
        ]).astype(np.float64, copy=False)
        finite = np.all(np.isfinite(xyz), axis=1)
        if np.any(finite):
            lower = np.minimum(lower, np.min(xyz[finite], axis=0))
            upper = np.maximum(upper, np.max(xyz[finite], axis=0))
    del source
    if not np.all(np.isfinite(lower)) or not np.all(np.isfinite(upper)):
        raise ValueError(f"{info.path} has no finite geometry")
    return lower, upper


def _grid_counts_for_scan(
    points: np.ndarray,
    info: PlyInfo,
    component: ComponentSupport,
    scan_id: int,
) -> tuple[np.ndarray, np.ndarray]:
    shape = component.mask.shape
    total = np.zeros(shape, dtype=np.int64)
    selected_scan = np.zeros(shape, dtype=np.int64)
    if not len(points):
        return total, selected_scan
    x = np.asarray(points[info.fields["x"]], dtype=np.float64)
    y = np.asarray(points[info.fields["y"]], dtype=np.float64)
    gx, gy, within = component.grid_coordinates(x, y)
    scan_ids, valid_scan = _scan_ids(points, info)
    if np.any(within):
        np.add.at(total, (gy[within], gx[within]), 1)
    wanted = within & valid_scan & (scan_ids == scan_id)
    if np.any(wanted):
        np.add.at(selected_scan, (gy[wanted], gx[wanted]), 1)
    return total, selected_scan


def _target_edge_component(mask: np.ndarray, origin_ix: int, origin_iy: int) -> np.ndarray:
    count, labels = cv2.connectedComponents(mask.astype(np.uint8), connectivity=8)
    if count <= 1:
        raise ValueError("Patch 04 ScanID 9 footprint has no connected component")
    y, x = np.nonzero(mask)
    xy = np.column_stack((
        (x + origin_ix + 0.5) * COMPONENT_CELL_METRES,
        (y + origin_iy + 0.5) * COMPONENT_CELL_METRES,
    ))
    nearest = int(np.argmin(np.sum((xy - PATCH_04_TARGET_XY[None, :]) ** 2, axis=1)))
    label = int(labels[y[nearest], x[nearest]])
    return labels == label


def build_patch04_edge_support(
    local_1mm: np.ndarray,
    info_1mm: PlyInfo,
    mesh_bounds_min: np.ndarray,
    mesh_bounds_max: np.ndarray,
    blend_width_metres: float,
) -> LinearEdgeSupport:
    if cv2 is None:
        raise RuntimeError("OpenCV (cv2) is required for the Patch 04 edge model")
    if blend_width_metres <= COMPONENT_CELL_METRES:
        raise ValueError("Patch 04 blend width must exceed one planning cell")
    origin_ix = int(math.floor(mesh_bounds_min[0] / COMPONENT_CELL_METRES))
    origin_iy = int(math.floor(mesh_bounds_min[1] / COMPONENT_CELL_METRES))
    maximum_ix = int(math.floor(mesh_bounds_max[0] / COMPONENT_CELL_METRES))
    maximum_iy = int(math.floor(mesh_bounds_max[1] / COMPONENT_CELL_METRES))
    width = maximum_ix - origin_ix + 1
    height = maximum_iy - origin_iy + 1
    if width <= 0 or height <= 0:
        raise ValueError("Patch 04 mesh bounds do not form a usable grid")
    placeholder = ComponentSupport(
        camera_name="Patch 04",
        cleanmesh_index=4,
        host_scan_id=PATCH_04_SOURCE_SCAN_ID,
        bounds_min=np.asarray(mesh_bounds_min, dtype=np.float64),
        bounds_max=np.asarray(mesh_bounds_max, dtype=np.float64),
        dense_records=np.empty(0, dtype=np.dtype([])),
        cell_keys=np.empty(0, dtype=np.int64),
        cell_z_min=np.empty(0, dtype=np.float64),
        cell_z_max=np.empty(0, dtype=np.float64),
        origin_ix=origin_ix,
        origin_iy=origin_iy,
        mask=np.zeros((height, width), dtype=bool),
        boundary=np.zeros((height, width), dtype=bool),
        outside_ring=np.zeros((height, width), dtype=bool),
    )
    total, id9 = _grid_counts_for_scan(
        local_1mm,
        info_1mm,
        placeholder,
        PATCH_04_SOURCE_SCAN_ID,
    )
    smoothed_id9 = cv2.GaussianBlur(id9.astype(np.float32), (0, 0), 1.0)
    high_density = smoothed_id9 >= 3.0
    high_density = cv2.morphologyEx(
        high_density.astype(np.uint8),
        cv2.MORPH_CLOSE,
        _ellipse_kernel(2),
    ) > 0
    high_density = cv2.morphologyEx(
        high_density.astype(np.uint8),
        cv2.MORPH_OPEN,
        _ellipse_kernel(1),
    ) > 0
    high_density = _target_edge_component(high_density, origin_ix, origin_iy)
    eroded = cv2.erode(high_density.astype(np.uint8), np.ones((3, 3), np.uint8)) > 0
    boundary = high_density & ~eroded
    by, bx = np.nonzero(boundary)
    boundary_xy = np.column_stack((
        (bx + origin_ix + 0.5) * COMPONENT_CELL_METRES,
        (by + origin_iy + 0.5) * COMPONENT_CELL_METRES,
    ))
    fit_points = boundary_xy[
        np.linalg.norm(boundary_xy - PATCH_04_TARGET_XY[None, :], axis=1)
        <= PATCH_04_EDGE_FIT_RADIUS_METRES
    ]
    if len(fit_points) < 12:
        raise ValueError("Patch 04 has too few ScanID 9 boundary cells near its saved camera target")
    fit_centre = np.mean(fit_points, axis=0)
    _, _, vh = np.linalg.svd(fit_points - fit_centre, full_matrices=False)
    direction = np.asarray(vh[0], dtype=np.float64)
    direction /= np.linalg.norm(direction)
    normal = np.array([-direction[1], direction[0]], dtype=np.float64)
    edge_origin = fit_centre + direction * float(np.dot(PATCH_04_TARGET_XY - fit_centre, direction))

    grid_x, grid_y = np.meshgrid(
        (np.arange(width) + origin_ix + 0.5) * COMPONENT_CELL_METRES,
        (np.arange(height) + origin_iy + 0.5) * COMPONENT_CELL_METRES,
    )
    grid_xy = np.column_stack((grid_x.ravel(), grid_y.ravel()))
    signed = ((grid_xy - edge_origin[None, :]) @ normal).reshape(height, width)
    along = ((grid_xy - edge_origin[None, :]) @ direction).reshape(height, width)
    high_near_line = high_density & (np.abs(signed) <= blend_width_metres)
    if not np.any(high_near_line):
        raise ValueError("Patch 04 fitted edge does not touch the ScanID 9 footprint")
    if float(np.median(signed[high_near_line])) > 0.0:
        normal *= -1.0
        signed *= -1.0

    corners = np.array([
        [mesh_bounds_min[0], mesh_bounds_min[1]],
        [mesh_bounds_min[0], mesh_bounds_max[1]],
        [mesh_bounds_max[0], mesh_bounds_min[1]],
        [mesh_bounds_max[0], mesh_bounds_max[1]],
    ])
    projected_corners = (corners - edge_origin[None, :]) @ direction
    along_min = float(np.min(projected_corners) + PATCH_04_EDGE_END_MARGIN_METRES)
    along_max = float(np.max(projected_corners) - PATCH_04_EDGE_END_MARGIN_METRES)
    transition = (
        ~high_density
        & (signed >= 0.0)
        & (signed <= blend_width_metres)
        & (along >= along_min)
        & (along <= along_max)
        & (total > 0)
    )
    if np.count_nonzero(transition) < 25:
        raise ValueError("Patch 04 transition belt contains too few occupied cells")
    ambient_ring = (
        ~high_density
        & (signed >= blend_width_metres * 0.70)
        & (signed <= blend_width_metres)
        & (along >= along_min)
        & (along <= along_max)
        & (total > 0)
    )
    fit_rms = float(np.sqrt(np.mean(((fit_points - edge_origin[None, :]) @ normal) ** 2)))
    placeholder.mask = transition
    placeholder.boundary = boundary
    placeholder.outside_ring = ambient_ring
    return LinearEdgeSupport(
        component=placeholder,
        source_scan_id=PATCH_04_SOURCE_SCAN_ID,
        edge_origin_xy=edge_origin,
        edge_direction_xy=direction,
        outside_normal_xy=normal,
        signed_distance=signed,
        along_distance=along,
        high_density_mask=high_density,
        transition_mask=transition,
        along_min=along_min,
        along_max=along_max,
        blend_width_metres=blend_width_metres,
        fit_rms_metres=fit_rms,
    )


def _hashed_noise_values(indices: np.ndarray, seed: int) -> np.ndarray:
    """Return deterministic [-1, 1] values for signed integer lattice points."""
    values = np.asarray(indices, dtype=np.int64).astype(np.uint64, copy=False)
    values ^= np.uint64(seed & 0xFFFFFFFFFFFFFFFF)
    values += np.uint64(0x9E3779B97F4A7C15)
    values = (values ^ (values >> np.uint64(30))) * np.uint64(0xBF58476D1CE4E5B9)
    values = (values ^ (values >> np.uint64(27))) * np.uint64(0x94D049BB133111EB)
    values ^= values >> np.uint64(31)
    unit = (values >> np.uint64(11)).astype(np.float64) * (1.0 / float(1 << 53))
    return unit * 2.0 - 1.0


def _smooth_value_noise_1d(
    coordinate: np.ndarray,
    wavelength_metres: float,
    seed: int,
) -> np.ndarray:
    if wavelength_metres <= 0.0:
        raise ValueError("Noise wavelength must be positive")
    scaled = np.asarray(coordinate, dtype=np.float64) / wavelength_metres
    lattice = np.floor(scaled).astype(np.int64)
    fraction = scaled - lattice
    blend = fraction * fraction * (3.0 - 2.0 * fraction)
    left = _hashed_noise_values(lattice, seed)
    right = _hashed_noise_values(lattice + 1, seed)
    return left * (1.0 - blend) + right * blend


def _grid_surface_planarity(
    points: np.ndarray,
    info: PlyInfo,
    component: ComponentSupport,
) -> tuple[np.ndarray, dict[str, float]]:
    """Measure connected local normal agreement without depending on normal sign."""
    shape = component.mask.shape
    count = np.zeros(shape, dtype=np.float64)
    tensor = np.zeros((*shape, 6), dtype=np.float64)
    if not len(points):
        return np.full(shape, 0.5, dtype=np.float64), {
            "raw_p20": 0.0,
            "raw_p80": 0.0,
        }
    x = np.asarray(points[info.fields["x"]], dtype=np.float64)
    y = np.asarray(points[info.fields["y"]], dtype=np.float64)
    normals = np.column_stack([
        points[info.fields[channel]]
        for channel in ("nx", "ny", "nz")
    ]).astype(np.float64, copy=False)
    lengths = np.linalg.norm(normals, axis=1)
    gx, gy, within = component.grid_coordinates(x, y)
    valid = within & np.all(np.isfinite(normals), axis=1) & (lengths > 1.0e-6)
    if not np.any(valid):
        return np.full(shape, 0.5, dtype=np.float64), {
            "raw_p20": 0.0,
            "raw_p80": 0.0,
        }
    normals = normals[valid] / lengths[valid, None]
    gx = gx[valid]
    gy = gy[valid]
    np.add.at(count, (gy, gx), 1.0)
    products = np.column_stack((
        normals[:, 0] * normals[:, 0],
        normals[:, 0] * normals[:, 1],
        normals[:, 0] * normals[:, 2],
        normals[:, 1] * normals[:, 1],
        normals[:, 1] * normals[:, 2],
        normals[:, 2] * normals[:, 2],
    ))
    for index in range(products.shape[1]):
        np.add.at(tensor[..., index], (gy, gx), products[:, index])
    sigma_cells = 1.5
    blurred_count = cv2.GaussianBlur(count, (0, 0), sigma_cells)
    blurred = np.stack([
        cv2.GaussianBlur(tensor[..., index], (0, 0), sigma_cells)
        for index in range(tensor.shape[-1])
    ], axis=-1)
    matrices = np.zeros((*shape, 3, 3), dtype=np.float64)
    matrices[..., 0, 0] = blurred[..., 0]
    matrices[..., 0, 1] = matrices[..., 1, 0] = blurred[..., 1]
    matrices[..., 0, 2] = matrices[..., 2, 0] = blurred[..., 2]
    matrices[..., 1, 1] = blurred[..., 3]
    matrices[..., 1, 2] = matrices[..., 2, 1] = blurred[..., 4]
    matrices[..., 2, 2] = blurred[..., 5]
    eigenvalues = np.linalg.eigvalsh(matrices)
    raw = np.divide(
        eigenvalues[..., 2],
        np.maximum(blurred_count, 1.0e-12),
        out=np.zeros(shape, dtype=np.float64),
        where=blurred_count > 1.0e-8,
    )
    reference = component.mask & (blurred_count > 1.0e-4)
    values = raw[reference]
    if len(values):
        low, high = np.percentile(values, [20.0, 80.0])
    else:
        low, high = 0.0, 1.0
    span = max(float(high - low), 1.0e-6)
    scaled = np.clip((raw - low) / span, 0.0, 1.0)
    scaled = scaled * scaled * (3.0 - 2.0 * scaled)
    observed = (blurred_count > 1.0e-4).astype(np.float64)
    numerator = cv2.GaussianBlur(scaled * observed, (0, 0), 1.25)
    denominator = cv2.GaussianBlur(observed, (0, 0), 1.25)
    planarity = np.divide(
        numerator,
        np.maximum(denominator, 1.0e-8),
        out=np.full(shape, 0.5, dtype=np.float64),
        where=denominator > 1.0e-8,
    )
    return np.clip(planarity, 0.0, 1.0), {
        "raw_p20": float(low),
        "raw_p80": float(high),
    }


def configure_patch04_jagged_fade(
    support: LinearEdgeSupport,
    local_1mm: np.ndarray,
    info_1mm: PlyInfo,
    *,
    minimum_width_metres: float,
    base_width_metres: float,
    maximum_width_metres: float,
    core_width_metres: float,
    noise_seed: int,
) -> dict[str, Any]:
    if not (
        0.0 <= core_width_metres < minimum_width_metres
        <= base_width_metres < maximum_width_metres
        <= support.blend_width_metres + 1.0e-9
    ):
        raise ValueError(
            "Jagged Patch 04 widths must satisfy "
            "0 <= core < minimum <= base < maximum <= support width"
        )
    along = support.along_distance
    broad = _smooth_value_noise_1d(along, 0.78, noise_seed + 11)
    medium = _smooth_value_noise_1d(along, 0.34, noise_seed + 37)
    fine = _smooth_value_noise_1d(along, 0.15, noise_seed + 71)
    zones = 0.60 * broad + 0.28 * medium + 0.12 * fine
    zone_reference = zones[support.transition_mask]
    zone_low, zone_centre, zone_high = np.percentile(
        zone_reference,
        [5.0, 50.0, 95.0],
    )
    zones = np.where(
        zones >= zone_centre,
        (zones - zone_centre) / max(float(zone_high - zone_centre), 1.0e-6),
        (zones - zone_centre) / max(float(zone_centre - zone_low), 1.0e-6),
    )
    zones = np.clip(zones, -1.0, 1.0)
    positive = np.maximum(zones, 0.0) ** 1.20
    negative = np.maximum(-zones, 0.0) ** 1.10
    width = (
        base_width_metres
        + (maximum_width_metres - base_width_metres) * positive
        - (base_width_metres - minimum_width_metres) * negative
    )
    planarity, planarity_report = _grid_surface_planarity(
        local_1mm,
        info_1mm,
        support.component,
    )
    width += PATCH_04_JAGGED_PLANARITY_METRES * (2.0 * planarity - 1.0)
    width = cv2.GaussianBlur(width.astype(np.float64), (0, 0), 0.70)
    width = np.clip(width, minimum_width_metres, maximum_width_metres)
    support.edge_profile = "jagged"
    support.fade_width_map = width
    support.surface_planarity_map = planarity
    support.core_width_metres = core_width_metres
    support.noise_seed = noise_seed
    active = support.transition_mask
    active_widths = width[active]
    active_planarity = planarity[active]
    return {
        "profile": "connected-jagged",
        "noise": "deterministic three-octave smooth value noise",
        "noise_seed": int(noise_seed),
        "noise_wavelengths_m": [0.78, 0.34, 0.15],
        "raw_noise_p05": float(zone_low),
        "raw_noise_p50": float(zone_centre),
        "raw_noise_p95": float(zone_high),
        "minimum_width_m": float(minimum_width_metres),
        "base_width_m": float(base_width_metres),
        "maximum_width_m": float(maximum_width_metres),
        "core_width_m": float(core_width_metres),
        "planarity_modulation_m": PATCH_04_JAGGED_PLANARITY_METRES,
        "width_p05_m": float(np.percentile(active_widths, 5)),
        "width_p50_m": float(np.percentile(active_widths, 50)),
        "width_p95_m": float(np.percentile(active_widths, 95)),
        "planarity_p05": float(np.percentile(active_planarity, 5)),
        "planarity_p50": float(np.percentile(active_planarity, 50)),
        "planarity_p95": float(np.percentile(active_planarity, 95)),
        **planarity_report,
    }


def _srgb_to_oklab(rgb: np.ndarray) -> np.ndarray:
    srgb = np.asarray(rgb, dtype=np.float64) / 255.0
    linear = np.where(srgb <= 0.04045, srgb / 12.92, ((srgb + 0.055) / 1.055) ** 2.4)
    r, g, b = linear.T
    l = np.cbrt(0.4122214708*r + 0.5363325363*g + 0.0514459929*b)
    m = np.cbrt(0.2119034982*r + 0.6806995451*g + 0.1073969566*b)
    s = np.cbrt(0.0883024619*r + 0.2817188376*g + 0.6299787005*b)
    return np.column_stack((
        0.2104542553*l + 0.7936177850*m - 0.0040720468*s,
        1.9779984951*l - 2.4285922050*m + 0.4505937099*s,
        0.0259040371*l + 0.7827717662*m - 0.8086757660*s,
    ))


def _oklab_to_linear_rgb(lab: np.ndarray) -> np.ndarray:
    lightness, a, b = np.asarray(lab, dtype=np.float64).T
    l = (lightness + 0.3963377774*a + 0.2158037573*b) ** 3
    m = (lightness - 0.1055613458*a - 0.0638541728*b) ** 3
    s = (lightness - 0.0894841775*a - 1.2914855480*b) ** 3
    return np.column_stack((
        4.0767416621*l - 3.3077115913*m + 0.2309699292*s,
        -1.2684380046*l + 2.6097574011*m - 0.3413193965*s,
        -0.0041960863*l - 0.7034186147*m + 1.7076147010*s,
    ))


def _oklab_to_srgb_bytes(lab: np.ndarray) -> tuple[np.ndarray, float]:
    adjusted = np.asarray(lab, dtype=np.float64).copy()
    adjusted[:, 0] = np.clip(adjusted[:, 0], 0.0, 1.0)
    linear = _oklab_to_linear_rgb(adjusted)
    bad = np.any((linear < 0.0) | (linear > 1.0), axis=1)
    fraction = float(np.mean(bad)) if len(adjusted) else 0.0
    if np.any(bad):
        problem = adjusted[bad]
        low, high = np.zeros(len(problem)), np.ones(len(problem))
        for _ in range(12):
            scale = (low + high) * 0.5
            candidate = problem.copy()
            candidate[:, 1:] *= scale[:, None]
            valid = np.all((_oklab_to_linear_rgb(candidate) >= -1e-10) & (_oklab_to_linear_rgb(candidate) <= 1 + 1e-10), axis=1)
            low[valid], high[~valid] = scale[valid], scale[~valid]
        problem[:, 1:] *= low[:, None]
        adjusted[bad] = problem
        linear = _oklab_to_linear_rgb(adjusted)
    linear = np.clip(linear, 0.0, 1.0)
    srgb = np.where(linear <= 0.0031308, 12.92*linear, 1.055*np.power(linear, 1/2.4) - 0.055)
    return np.rint(np.clip(srgb, 0, 1)*255).astype(np.uint8), fraction


def _load_component_definitions(report_path: Path) -> list[dict[str, Any]]:
    report = json.loads(report_path.read_text(encoding="utf-8"))
    components = report.get("components", [])
    requested = ((0, "Patch 01", 3), (2, "Patch 02", 5), (4, "Patch 03", 0))
    result: list[dict[str, Any]] = []
    for zero_index, camera_name, host_scan_id in requested:
        if zero_index >= len(components):
            raise ValueError(f"CleanMesh report does not contain component {zero_index + 1}")
        source = components[zero_index]
        result.append({
            "camera_name": camera_name,
            "cleanmesh_index": zero_index + 1,
            "host_scan_id": host_scan_id,
            "bounds_min": np.asarray(source["bounds_min"], dtype=np.float64),
            "bounds_max": np.asarray(source["bounds_max"], dtype=np.float64),
        })
    return result


def build_component_supports(
    dense_info: PlyInfo,
    report_path: Path,
    chunk_records: int,
) -> list[ComponentSupport]:
    definitions = _load_component_definitions(report_path)
    dense = records(dense_info)
    pieces: list[list[np.ndarray]] = [[] for _ in definitions]
    started = time.monotonic()
    for start in range(0, dense_info.vertex_count, chunk_records):
        end = min(start + chunk_records, dense_info.vertex_count)
        chunk = dense[start:end]
        x = np.asarray(chunk[dense_info.fields["x"]], dtype=np.float64)
        y = np.asarray(chunk[dense_info.fields["y"]], dtype=np.float64)
        z = np.asarray(chunk[dense_info.fields["z"]], dtype=np.float64)
        finite = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        for index, definition in enumerate(definitions):
            lower, upper = definition["bounds_min"], definition["bounds_max"]
            selected = finite & (x >= lower[0]) & (x <= upper[0]) & (y >= lower[1]) & (y <= upper[1]) & (z >= lower[2]) & (z <= upper[2])
            if np.any(selected):
                pieces[index].append(np.array(chunk[selected], copy=True))
        if start == 0 or end == dense_info.vertex_count or end // chunk_records % 10 == 0:
            print(f"  dense footprint pass: {100*end/dense_info.vertex_count:5.1f}%", flush=True)
    del dense
    supports: list[ComponentSupport] = []
    for definition, component_pieces in zip(definitions, pieces, strict=True):
        if not component_pieces:
            raise ValueError(f"No dense points found for {definition['camera_name']}")
        component_records = np.concatenate(component_pieces)
        x = np.asarray(component_records[dense_info.fields["x"]], dtype=np.float64)
        y = np.asarray(component_records[dense_info.fields["y"]], dtype=np.float64)
        z = np.asarray(component_records[dense_info.fields["z"]], dtype=np.float64)
        ix = np.floor(x / COMPONENT_CELL_METRES).astype(np.int64)
        iy = np.floor(y / COMPONENT_CELL_METRES).astype(np.int64)
        keys = _pack_cells(ix, iy)
        order = np.argsort(keys, kind="stable")
        unique, starts = np.unique(keys[order], return_index=True)
        z_sorted = z[order]
        z_min = np.minimum.reduceat(z_sorted, starts)
        z_max = np.maximum.reduceat(z_sorted, starts)
        cell_ix, cell_iy = _unpack_cells(unique)
        padding = 6
        origin_ix = int(np.min(cell_ix)) - padding
        origin_iy = int(np.min(cell_iy)) - padding
        width = int(np.max(cell_ix)) - origin_ix + padding + 1
        height = int(np.max(cell_iy)) - origin_iy + padding + 1
        mask = np.zeros((height, width), dtype=np.uint8)
        mask[cell_iy - origin_iy, cell_ix - origin_ix] = 1
        # Close one-cell sampling pinholes, but retain the measured component
        # outline and require a matching per-cell Z range during selection.
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, _ellipse_kernel(1))
        eroded = cv2.erode(mask, _ellipse_kernel(2))
        dilated = cv2.dilate(mask, _ellipse_kernel(4))
        supports.append(ComponentSupport(
            camera_name=definition["camera_name"],
            cleanmesh_index=definition["cleanmesh_index"],
            host_scan_id=definition["host_scan_id"],
            bounds_min=definition["bounds_min"],
            bounds_max=definition["bounds_max"],
            dense_records=component_records,
            cell_keys=unique,
            cell_z_min=z_min,
            cell_z_max=z_max,
            origin_ix=origin_ix,
            origin_iy=origin_iy,
            mask=mask.astype(bool),
            boundary=mask.astype(bool) & ~eroded.astype(bool),
            outside_ring=dilated.astype(bool) & ~mask.astype(bool),
        ))
        print(
            f"  {definition['camera_name']}: {len(component_records):,} dense points, "
            f"{len(unique):,} support cells, host ScanID {definition['host_scan_id']}",
            flush=True,
        )
    print(f"  dense support construction: {time.monotonic()-started:.1f}s", flush=True)
    return supports


def collect_local_records(
    info: PlyInfo,
    components: Sequence[ComponentSupport],
    chunk_records: int,
) -> list[np.ndarray]:
    source = records(info)
    pieces: list[list[np.ndarray]] = [[] for _ in components]
    for start in range(0, info.vertex_count, chunk_records):
        end = min(start + chunk_records, info.vertex_count)
        chunk = source[start:end]
        x = np.asarray(chunk[info.fields["x"]], dtype=np.float64)
        y = np.asarray(chunk[info.fields["y"]], dtype=np.float64)
        z = np.asarray(chunk[info.fields["z"]], dtype=np.float64)
        finite = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        for index, component in enumerate(components):
            lower = component.bounds_min - np.array([REFERENCE_MARGIN_METRES, REFERENCE_MARGIN_METRES, REFERENCE_MARGIN_METRES])
            upper = component.bounds_max + np.array([REFERENCE_MARGIN_METRES, REFERENCE_MARGIN_METRES, REFERENCE_MARGIN_METRES])
            selected = finite & (x >= lower[0]) & (x <= upper[0]) & (y >= lower[1]) & (y <= upper[1]) & (z >= lower[2]) & (z <= upper[2])
            if np.any(selected):
                pieces[index].append(np.array(chunk[selected], copy=True))
        if start == 0 or end == info.vertex_count or end // chunk_records % 10 == 0:
            print(f"  {info.path.name} local pass: {100*end/info.vertex_count:5.1f}%", flush=True)
    del source
    result = [np.concatenate(part) if part else np.empty(0, dtype=info.dtype) for part in pieces]
    for component, local in zip(components, result, strict=True):
        print(f"    {component.camera_name}: {len(local):,} nearby records", flush=True)
    return result


def _record_values(chunk: np.ndarray, info: PlyInfo) -> np.ndarray:
    rgb = np.column_stack((
        chunk[info.fields["red"]],
        chunk[info.fields["green"]],
        chunk[info.fields["blue"]],
    ))
    result = np.empty((len(chunk), 6), dtype=np.float64)
    result[:, :3] = _srgb_to_oklab(rgb)
    for channel, name in enumerate(SMOOTH_SCALARS, start=3):
        result[:, channel] = np.asarray(chunk[info.fields[name]], dtype=np.float64)
    return result


def aggregate_cell_stats(points: np.ndarray, info: PlyInfo) -> CellStats:
    if not len(points):
        return CellStats(
            np.empty(0, np.int64), np.empty(0, np.int64),
            np.empty((0, 3)), np.empty((0, 3)), np.empty((0, 6)), np.empty(0, np.int64),
        )
    x = np.asarray(points[info.fields["x"]], dtype=np.float64)
    y = np.asarray(points[info.fields["y"]], dtype=np.float64)
    z = np.asarray(points[info.fields["z"]], dtype=np.float64)
    ix = np.floor(x / COMPONENT_CELL_METRES).astype(np.int64)
    iy = np.floor(y / COMPONENT_CELL_METRES).astype(np.int64)
    keys = _pack_cells(ix, iy)
    order = np.argsort(keys, kind="stable")
    _, starts, counts = np.unique(keys[order], return_index=True, return_counts=True)
    values = _record_values(points, info)
    xyz_source = np.column_stack((x, y, z))
    normals_source = np.column_stack((
        points[info.fields["nx"]], points[info.fields["ny"]], points[info.fields["nz"]],
    )).astype(np.float64)
    size = len(starts)
    result_ix = np.empty(size, np.int64)
    result_iy = np.empty(size, np.int64)
    xyz = np.empty((size, 3), np.float64)
    normals = np.empty((size, 3), np.float64)
    medians = np.empty((size, 6), np.float64)
    for group, (start, count) in enumerate(zip(starts, counts, strict=True)):
        indices = order[start:start + count]
        first = indices[0]
        result_ix[group], result_iy[group] = ix[first], iy[first]
        xyz[group] = np.nanmedian(xyz_source[indices], axis=0)
        medians[group] = np.nanmedian(values[indices], axis=0)
        normal = np.nanmedian(normals_source[indices], axis=0)
        norm = float(np.linalg.norm(normal))
        normals[group] = normal / norm if np.isfinite(norm) and norm > 1e-8 else np.nan
    return CellStats(result_ix, result_iy, xyz, normals, medians, counts)


def _nearest_boundary_pairs(
    source: CellStats,
    target: CellStats,
    source_choice: np.ndarray,
    target_choice: np.ndarray,
    *,
    relaxed: bool = False,
) -> tuple[np.ndarray, np.ndarray]:
    source_indices = np.flatnonzero(source_choice)
    target_indices = np.flatnonzero(target_choice)
    if not len(source_indices) or not len(target_indices):
        return np.empty(0, np.int64), np.empty(0, np.int64)
    chosen_source: list[np.ndarray] = []
    chosen_target: list[np.ndarray] = []
    maximum = MAX_PAIR_DISTANCE_METRES * (1.5 if relaxed else 1.0)
    for begin in range(0, len(source_indices), 512):
        subset = source_indices[begin:begin + 512]
        delta = source.xyz[subset, None, :2] - target.xyz[target_indices, :2]
        distance_sq = np.sum(delta * delta, axis=2)
        allowed = np.abs(source.xyz[subset, 2, None] - target.xyz[target_indices, 2]) <= PAIR_Z_TOLERANCE_METRES * (2 if relaxed else 1)
        if not relaxed:
            sn, tn = source.normals[subset], target.normals[target_indices]
            valid_s = np.all(np.isfinite(sn), axis=1)
            valid_t = np.all(np.isfinite(tn), axis=1)
            if np.any(valid_s) and np.any(valid_t):
                dots = np.abs(sn @ tn.T)
                allowed &= (dots >= PAIR_NORMAL_DOT_MIN) | ~valid_s[:, None] | ~valid_t[None, :]
        distance_sq[~allowed] = np.inf
        nearest = np.argmin(distance_sq, axis=1)
        valid = distance_sq[np.arange(len(subset)), nearest] <= maximum * maximum
        chosen_source.append(subset[valid])
        chosen_target.append(target_indices[nearest[valid]])
    return np.concatenate(chosen_source), np.concatenate(chosen_target)


def _robust_affine(source: np.ndarray, target: np.ndarray, bounds: tuple[float, float], minimum_correlation: float) -> tuple[float, float]:
    finite = np.isfinite(source) & np.isfinite(target)
    x, y = np.asarray(source)[finite], np.asarray(target)[finite]
    if len(x) < 10 or np.ptp(x) < 1e-8:
        return 1.0, float(np.nanmedian(y - x)) if len(x) else 0.0
    correlation = float(np.corrcoef(x, y)[0, 1])
    if not np.isfinite(correlation) or abs(correlation) < minimum_correlation:
        return 1.0, float(np.median(y - x))
    design = np.column_stack((x, np.ones_like(x)))
    coefficients, *_ = np.linalg.lstsq(design, y, rcond=None)
    for _ in range(20):
        residual = y - design @ coefficients
        scale = 1.4826 * np.median(np.abs(residual - np.median(residual))) + 1e-9
        weight = np.minimum(1.0, 1.345 * scale / (np.abs(residual) + 1e-12))
        root = np.sqrt(weight)
        updated, *_ = np.linalg.lstsq(design * root[:, None], y * root, rcond=None)
        if np.max(np.abs(updated - coefficients)) < 1e-8:
            coefficients = updated
            break
        coefficients = updated
    gain = float(np.clip(coefficients[0], *bounds))
    return gain, float(np.median(y - gain * x))


def _smooth_residual_field(
    residual: np.ndarray,
    stats: CellStats,
    indices: np.ndarray,
    component: ComponentSupport,
) -> np.ndarray:
    height, width = component.mask.shape
    channels = residual.shape[1]
    sums = np.zeros((height, width, channels), np.float64)
    weights = np.zeros((height, width), np.float64)
    gx = stats.ix[indices] - component.origin_ix
    gy = stats.iy[indices] - component.origin_iy
    for row, x, y in zip(residual, gx, gy, strict=True):
        if 0 <= x < width and 0 <= y < height and np.all(np.isfinite(row)):
            sums[y, x] += row
            weights[y, x] += 1.0
    sigma = max(1.0, LOCAL_SMOOTHING_METRES / COMPONENT_CELL_METRES)
    blurred_weight = cv2.GaussianBlur(weights, (0, 0), sigmaX=sigma, sigmaY=sigma)
    field = np.zeros_like(sums)
    for channel in range(channels):
        numerator = cv2.GaussianBlur(sums[:, :, channel], (0, 0), sigmaX=sigma, sigmaY=sigma)
        field[:, :, channel] = numerator / np.maximum(blurred_weight, 1e-12)
    known = component.boundary & (blurred_weight > max(float(np.max(blurred_weight)) * 0.01, 1e-12))
    if not np.any(known):
        known = weights > 0
    kernel = np.array([[0, .25, 0], [.25, 0, .25], [0, .25, 0]], dtype=np.float64)
    median = np.nanmedian(residual, axis=0)
    for channel in range(channels):
        values = field[:, :, channel]
        values[~known] = median[channel] if np.isfinite(median[channel]) else 0.0
        fixed = values.copy()
        for iteration in range(800):
            updated = cv2.filter2D(values, -1, kernel, borderType=cv2.BORDER_REPLICATE)
            updated[known] = fixed[known]
            if iteration % 25 == 24 and float(np.max(np.abs(updated - values))) < 1e-7:
                values = updated
                break
            values = updated
        field[:, :, channel] = values
    field[:, :, 0] = np.clip(field[:, :, 0], -0.10, 0.10)
    field[:, :, 1:3] = np.clip(field[:, :, 1:3], -0.05, 0.05)
    for channel in range(3, channels):
        finite = residual[:, channel][np.isfinite(residual[:, channel])]
        if len(finite):
            centre = float(np.median(finite))
            mad = 1.4826 * float(np.median(np.abs(finite - centre))) + 1e-9
            field[:, :, channel] = np.clip(field[:, :, channel], centre - 4 * mad, centre + 4 * mad)
    return field


def _sample_field(field: np.ndarray, x: np.ndarray, y: np.ndarray, component: ComponentSupport) -> np.ndarray:
    height, width, _ = field.shape
    gx = np.asarray(x, dtype=np.float64) / COMPONENT_CELL_METRES - component.origin_ix - 0.5
    gy = np.asarray(y, dtype=np.float64) / COMPONENT_CELL_METRES - component.origin_iy - 0.5
    if height == 1 or width == 1:
        return field[np.clip(np.rint(gy).astype(int), 0, height - 1), np.clip(np.rint(gx).astype(int), 0, width - 1)]
    x0, y0 = np.floor(gx).astype(int), np.floor(gy).astype(int)
    tx, ty = gx - x0, gy - y0
    x0, y0 = np.clip(x0, 0, width - 2), np.clip(y0, 0, height - 2)
    return (
        field[y0, x0] * ((1 - tx) * (1 - ty))[:, None]
        + field[y0, x0 + 1] * (tx * (1 - ty))[:, None]
        + field[y0 + 1, x0] * ((1 - tx) * ty)[:, None]
        + field[y0 + 1, x0 + 1] * (tx * ty)[:, None]
    )


def fit_correction_model(
    local: np.ndarray,
    info: PlyInfo,
    component: ComponentSupport,
    source_scan_id: int,
) -> CorrectionModel:
    scan_ids, valid_scan = _scan_ids(local, info)
    inside = component.contains(
        local[info.fields["x"]], local[info.fields["y"]], local[info.fields["z"]],
    )
    source_points = local[valid_scan & inside & (scan_ids == source_scan_id)]
    host_points = local[valid_scan & (scan_ids == component.host_scan_id)]
    if len(source_points) < 100 or len(host_points) < 100:
        raise ValueError(
            f"{component.camera_name} ScanID {source_scan_id}: insufficient source/host points "
            f"({len(source_points):,}/{len(host_points):,})"
        )
    source_stats = aggregate_cell_stats(source_points, info)
    host_stats = aggregate_cell_stats(host_points, info)
    sgx = source_stats.ix - component.origin_ix
    sgy = source_stats.iy - component.origin_iy
    hgx = host_stats.ix - component.origin_ix
    hgy = host_stats.iy - component.origin_iy
    source_within = (sgx >= 0) & (sgx < component.mask.shape[1]) & (sgy >= 0) & (sgy < component.mask.shape[0])
    host_within = (hgx >= 0) & (hgx < component.mask.shape[1]) & (hgy >= 0) & (hgy < component.mask.shape[0])
    source_boundary = np.zeros(len(source_stats.ix), dtype=bool)
    host_ring = np.zeros(len(host_stats.ix), dtype=bool)
    source_boundary[source_within] = component.boundary[sgy[source_within], sgx[source_within]]
    host_ring[host_within] = component.outside_ring[hgy[host_within], hgx[host_within]]
    source_index, target_index = _nearest_boundary_pairs(source_stats, host_stats, source_boundary, host_ring)
    pairing_mode = "inner-edge to outer-ring"
    if len(source_index) < 50:
        source_index, target_index = _nearest_boundary_pairs(
            source_stats, host_stats, source_within, host_within, relaxed=True,
        )
        pairing_mode = "relaxed nearest overlap"
    if len(source_index) < 30:
        raise ValueError(
            f"{component.camera_name} ScanID {source_scan_id}: only {len(source_index)} boundary pairs"
        )
    source_values = source_stats.values[source_index]
    target_values = host_stats.values[target_index]
    gain = np.ones(6, dtype=np.float64)
    offset = np.zeros(6, dtype=np.float64)
    channel_settings = (
        ((0.85, 1.15), 0.15),
        ((0.90, 1.10), 0.25),
        ((0.90, 1.10), 0.25),
        ((0.50, 1.50), 0.10),
        ((0.65, 1.35), 0.10),
        ((0.65, 1.35), 0.10),
    )
    for channel, (bounds, minimum_correlation) in enumerate(channel_settings):
        gain[channel], offset[channel] = _robust_affine(
            source_values[:, channel], target_values[:, channel], bounds, minimum_correlation,
        )
    offset[:3] = np.clip(offset[:3], [-0.25, -0.15, -0.15], [0.25, 0.15, 0.15])
    residual = target_values - (source_values * gain + offset)
    residual_field = _smooth_residual_field(residual, source_stats, source_index, component)
    local_at_pairs = _sample_field(
        residual_field,
        source_stats.xyz[source_index, 0],
        source_stats.xyz[source_index, 1],
        component,
    )
    fitted = source_values * gain + offset + local_at_pairs
    rgb_before = 100.0 * np.linalg.norm(target_values[:, :3] - source_values[:, :3], axis=1)
    rgb_after = 100.0 * np.linalg.norm(target_values[:, :3] - fitted[:, :3], axis=1)
    clip_low = np.full(6, -np.inf, dtype=np.float64)
    clip_high = np.full(6, np.inf, dtype=np.float64)
    clip_low[0], clip_high[0] = np.nanpercentile(host_stats.values[:, 0], [0.5, 99.5])
    for channel in range(3, 6):
        finite = host_stats.values[:, channel][np.isfinite(host_stats.values[:, channel])]
        if len(finite):
            clip_low[channel], clip_high[channel] = np.percentile(finite, [0.5, 99.5])
    scalar_metrics: dict[str, Any] = {}
    for channel, name in enumerate(SMOOTH_SCALARS, start=3):
        valid = np.isfinite(source_values[:, channel]) & np.isfinite(target_values[:, channel])
        scale = float(np.nanpercentile(target_values[valid, channel], 95) - np.nanpercentile(target_values[valid, channel], 5)) if np.any(valid) else 1.0
        scale = max(scale, 1e-9)
        before = np.abs(target_values[valid, channel] - source_values[valid, channel]) / scale
        after = np.abs(target_values[valid, channel] - fitted[valid, channel]) / scale
        scalar_metrics[name] = {
            "normalized_median_before": float(np.median(before)) if len(before) else None,
            "normalized_median_after": float(np.median(after)) if len(after) else None,
            "normalized_p95_before": float(np.percentile(before, 95)) if len(before) else None,
            "normalized_p95_after": float(np.percentile(after, 95)) if len(after) else None,
        }
    metrics = {
        "camera": component.camera_name,
        "component": component.cleanmesh_index,
        "source_scan_id": source_scan_id,
        "host_scan_id": component.host_scan_id,
        "source_point_count": int(len(source_points)),
        "host_point_count": int(len(host_points)),
        "pair_count": int(len(source_index)),
        "pairing_mode": pairing_mode,
        "gain": gain.tolist(),
        "offset": offset.tolist(),
        "rgb_delta_e_median_before": float(np.median(rgb_before)),
        "rgb_delta_e_median_after": float(np.median(rgb_after)),
        "rgb_delta_e_p95_before": float(np.percentile(rgb_before, 95)),
        "rgb_delta_e_p95_after": float(np.percentile(rgb_after, 95)),
        "scalar_boundaries": scalar_metrics,
    }
    return CorrectionModel(
        component_index=component.cleanmesh_index,
        source_scan_id=source_scan_id,
        host_scan_id=component.host_scan_id,
        gain=gain,
        offset=offset,
        residual_field=residual_field,
        clip_low=clip_low,
        clip_high=clip_high,
        pair_source=source_values,
        pair_target=target_values,
        metrics=metrics,
    )


def apply_correction_model(
    target: np.ndarray,
    selected: np.ndarray,
    info: PlyInfo,
    component: ComponentSupport,
    model: CorrectionModel,
) -> float:
    indices = np.flatnonzero(selected)
    if not len(indices):
        return 0.0
    points = target[indices]
    values = _record_values(points, info)
    local = _sample_field(
        model.residual_field,
        points[info.fields["x"]], points[info.fields["y"]], component,
    )
    corrected = values * model.gain + model.offset + local
    corrected[:, 0] = np.clip(corrected[:, 0], model.clip_low[0], model.clip_high[0])
    rgb, gamut_fraction = _oklab_to_srgb_bytes(corrected[:, :3])
    target[info.fields["red"]][indices] = rgb[:, 0]
    target[info.fields["green"]][indices] = rgb[:, 1]
    target[info.fields["blue"]][indices] = rgb[:, 2]
    for channel, name in enumerate(SMOOTH_SCALARS, start=3):
        original = values[:, channel]
        result = corrected[:, channel]
        finite = np.isfinite(original)
        result[finite] = np.clip(result[finite], model.clip_low[channel], model.clip_high[channel])
        # Preserve the source NaN pattern exactly; these fields occasionally
        # carry CloudCompare missing-value sentinels.
        target[info.fields[name]][indices[finite]] = result[finite]
    target[info.fields["scan_id"]][indices] = float(REPLACEMENT_SCAN_ID)
    return gamut_fraction


def _grid_counts(
    local: np.ndarray,
    info: PlyInfo,
    component: ComponentSupport,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    height, width = component.mask.shape
    x = np.asarray(local[info.fields["x"]], dtype=np.float64)
    y = np.asarray(local[info.fields["y"]], dtype=np.float64)
    z = np.asarray(local[info.fields["z"]], dtype=np.float64)
    gx, gy, within = component.grid_coordinates(x, y)
    scan_ids, valid_scan = _scan_ids(local, info)
    flat = gy[within] * width + gx[within]
    total = np.bincount(flat, minlength=height * width).reshape(height, width).astype(np.float64)
    target_mask = within & valid_scan & np.isin(scan_ids, SOURCE_SCAN_IDS) & component.contains(x, y, z)
    target_flat = gy[target_mask] * width + gx[target_mask]
    target = np.bincount(target_flat, minlength=height * width).reshape(height, width).astype(np.float64)
    host_mask = within & valid_scan & (scan_ids == component.host_scan_id)
    host_flat = gy[host_mask] * width + gx[host_mask]
    host = np.bincount(host_flat, minlength=height * width).reshape(height, width).astype(np.float64)
    other = np.maximum(0.0, total - target - host)
    return total, target, host, other


def _harmonic_density(values: np.ndarray, known: np.ndarray, fill: np.ndarray) -> np.ndarray:
    finite_known = known & np.isfinite(values)
    seed = float(np.median(values[finite_known])) if np.any(finite_known) else 0.0
    result = np.full(values.shape, seed, dtype=np.float64)
    result[finite_known] = values[finite_known]
    kernel = np.array([[0, .25, 0], [.25, 0, .25], [0, .25, 0]], dtype=np.float64)
    update_region = fill & ~finite_known
    for iteration in range(1000):
        updated = cv2.filter2D(result, -1, kernel, borderType=cv2.BORDER_REPLICATE)
        updated[finite_known] = values[finite_known]
        updated[~fill & ~finite_known] = result[~fill & ~finite_known]
        if iteration % 25 == 24 and (not np.any(update_region) or float(np.max(np.abs(updated[update_region] - result[update_region]))) < 1e-4):
            result = updated
            break
        result = updated
    return np.maximum(result, 0.0)


def plan_density_additions(
    local: np.ndarray,
    info: PlyInfo,
    component: ComponentSupport,
) -> tuple[dict[int, int], dict[str, Any]]:
    total, target, host, other = _grid_counts(local, info, component)
    ring_known = component.outside_ring & (total > 0)
    fill = component.mask | component.outside_ring
    expected_total = _harmonic_density(total, ring_known, fill)
    expected_host = _harmonic_density(host, component.outside_ring & (host > 0), fill)
    match_total_density = component.cleanmesh_index == 1
    desired = (
        expected_total
        if match_total_density
        else np.minimum(expected_total, other + expected_host)
    )
    fill_fraction = (
        PATCH_01_DENSITY_DEFICIT_FILL_FRACTION
        if match_total_density
        else DENSITY_DEFICIT_FILL_FRACTION
    )
    raw_deficit = np.maximum(0.0, desired - total)
    threshold = np.maximum(2.0, np.ceil(desired * 0.10))
    deficient = component.mask & (raw_deficit >= threshold)
    labels_count, labels = cv2.connectedComponents(deficient.astype(np.uint8), connectivity=8)
    keep = np.zeros_like(deficient)
    for label in range(1, labels_count):
        selected = labels == label
        if np.count_nonzero(selected) >= 3:
            keep |= selected
    add = np.zeros_like(total, dtype=np.int64)
    add[keep] = np.floor(
        raw_deficit[keep] * fill_fraction
    ).astype(np.int64)
    ys, xs = np.nonzero(add > 0)
    keys = _pack_cells(xs.astype(np.int64) + component.origin_ix, ys.astype(np.int64) + component.origin_iy)
    plan = {int(key): int(add[y, x]) for key, x, y in zip(keys, xs, ys, strict=True)}
    inside_values = total[component.mask]
    ring_values = total[component.outside_ring & (total > 0)]
    report = {
        "current_points": int(np.sum(total[component.mask])),
        "target_patch_points": int(np.sum(target[component.mask])),
        "planned_additions": int(np.sum(add)),
        "density_target": "total_density" if match_total_density else "host_scan_density",
        "deficit_fill_fraction": fill_fraction,
        "deficient_cells": int(np.count_nonzero(add)),
        "interior_mean_points_per_cell": float(np.mean(inside_values)) if len(inside_values) else 0.0,
        "ring_mean_points_per_cell": float(np.mean(ring_values)) if len(ring_values) else 0.0,
        "projected_interior_mean_points_per_cell": float(np.mean(inside_values + add[component.mask])) if len(inside_values) else 0.0,
    }
    return plan, report


def select_dense_additions(
    dense: np.ndarray,
    dense_info: PlyInfo,
    local: np.ndarray,
    local_info: PlyInfo,
    component: ComponentSupport,
    requested: dict[int, int],
    spacing_metres: float,
) -> tuple[np.ndarray, dict[str, Any]]:
    if not requested:
        return np.empty(0, dtype=dense_info.dtype), {"requested": 0, "selected": 0, "shortfall": 0}
    dx = np.asarray(dense[dense_info.fields["x"]], dtype=np.float64)
    dy = np.asarray(dense[dense_info.fields["y"]], dtype=np.float64)
    dz = np.asarray(dense[dense_info.fields["z"]], dtype=np.float64)
    dense_cells = _pack_cells(
        np.floor(dx / COMPONENT_CELL_METRES).astype(np.int64),
        np.floor(dy / COMPONENT_CELL_METRES).astype(np.int64),
    )
    requested_keys = np.asarray(sorted(requested), dtype=np.int64)
    wanted = np.isin(dense_cells, requested_keys, assume_unique=False)
    candidate_indices = np.flatnonzero(wanted)
    source_clearance = spacing_metres * SOURCE_CLEARANCE_FACTOR
    clearance_rejected = 0
    if len(candidate_indices) and len(local):
        current_xyz = np.column_stack([
            local[local_info.fields[channel]]
            for channel in ("x", "y", "z")
        ]).astype(np.float32, copy=False)
        nearest_index = cv2.flann_Index(
            current_xyz,
            {"algorithm": 1, "trees": 4},
        )
        keep_clear = np.empty(len(candidate_indices), dtype=bool)
        query_chunk = 250_000
        threshold_squared = source_clearance * source_clearance
        for begin in range(0, len(candidate_indices), query_chunk):
            end = min(begin + query_chunk, len(candidate_indices))
            subset = candidate_indices[begin:end]
            query_xyz = np.column_stack((dx[subset], dy[subset], dz[subset])).astype(
                np.float32,
                copy=False,
            )
            _, distance_squared = nearest_index.knnSearch(
                query_xyz,
                1,
                params={"checks": 64},
            )
            keep_clear[begin:end] = distance_squared[:, 0] >= threshold_squared
        clearance_rejected = int(np.count_nonzero(~keep_clear))
        candidate_indices = candidate_indices[keep_clear]
        del nearest_index, current_xyz, keep_clear
    packing_spacing = spacing_metres * RANDOM_PACKING_SPACING_FACTOR
    candidate_voxels = _pack_voxels(
        dx[candidate_indices],
        dy[candidate_indices],
        dz[candidate_indices],
        packing_spacing,
    )
    current_voxels = np.unique(_pack_voxels(
        local[local_info.fields["x"]],
        local[local_info.fields["y"]],
        local[local_info.fields["z"]],
        packing_spacing,
    ))
    free = ~np.isin(candidate_voxels, current_voxels, assume_unique=False)
    candidate_indices = candidate_indices[free]
    candidate_voxels = candidate_voxels[free]
    if not len(candidate_indices):
        requested_count = sum(requested.values())
        return np.empty(0, dtype=dense_info.dtype), {"requested": requested_count, "selected": 0, "shortfall": requested_count}
    # Keep one deterministic, spatially decorrelated representative per
    # random-packing voxel.  Using the nominal output spacing here leaves too
    # few candidates and selects nearly every occupied voxel, exposing the
    # world-grid shells as topographic contour bands at 5 mm.
    representative_priority = _splitmix64(
        candidate_indices.astype(np.uint64)
        ^ np.uint64(component.cleanmesh_index * 0x9E3779B1)
    )
    voxel_order = np.lexsort((representative_priority, candidate_voxels))
    _, first = np.unique(candidate_voxels[voxel_order], return_index=True)
    candidate_indices = candidate_indices[voxel_order[first]]
    candidate_voxels = candidate_voxels[voxel_order[first]]
    cells = dense_cells[candidate_indices]
    priority = _splitmix64(
        candidate_indices.astype(np.uint64)
        ^ np.uint64(component.cleanmesh_index * 0xD1B54A35)
    )
    order = np.argsort(priority, kind="stable")
    remaining = {int(key): int(count) for key, count in requested.items()}
    remaining_total = sum(remaining.values())
    selected_indices_list: list[int] = []
    selected_by_cell: dict[int, int] = {}
    accepted_buckets: dict[tuple[int, int, int], list[int]] = {}
    bucket_x = np.floor(dx[candidate_indices] / packing_spacing).astype(np.int64)
    bucket_y = np.floor(dy[candidate_indices] / packing_spacing).astype(np.int64)
    bucket_z = np.floor(dz[candidate_indices] / packing_spacing).astype(np.int64)
    minimum_squared = packing_spacing * packing_spacing
    poisson_rejected = 0
    for position in order:
        source_index = int(candidate_indices[position])
        plan_cell = int(cells[position])
        if remaining.get(plan_cell, 0) <= 0:
            continue
        bx = int(bucket_x[position])
        by = int(bucket_y[position])
        bz = int(bucket_z[position])
        separated = True
        for oz in (-1, 0, 1):
            if not separated:
                break
            for oy in (-1, 0, 1):
                if not separated:
                    break
                for ox in (-1, 0, 1):
                    for accepted_index in accepted_buckets.get((bx + ox, by + oy, bz + oz), ()):
                        delta_x = dx[source_index] - dx[accepted_index]
                        delta_y = dy[source_index] - dy[accepted_index]
                        delta_z = dz[source_index] - dz[accepted_index]
                        if delta_x*delta_x + delta_y*delta_y + delta_z*delta_z < minimum_squared:
                            separated = False
                            break
                    if not separated:
                        break
        if not separated:
            poisson_rejected += 1
            continue
        selected_indices_list.append(source_index)
        accepted_buckets.setdefault((bx, by, bz), []).append(source_index)
        remaining[plan_cell] -= 1
        remaining_total -= 1
        selected_by_cell[plan_cell] = selected_by_cell.get(plan_cell, 0) + 1
        if remaining_total == 0:
            break
    selected_indices = np.asarray(selected_indices_list, dtype=np.int64)
    requested_count = sum(requested.values())
    report = {
        "requested": requested_count,
        "selected": int(len(selected_indices)),
        "shortfall": requested_count - int(len(selected_indices)),
        "cells_with_shortfall": int(sum(selected_by_cell.get(key, 0) < count for key, count in requested.items())),
        "packing_spacing_m": packing_spacing,
        "source_clearance_m": source_clearance,
        "source_clearance_rejected": clearance_rejected,
        "poisson_spacing_m": packing_spacing,
        "poisson_rejected": poisson_rejected,
        "representative_strategy": "deterministic source-first clearance plus hashed Poisson packing",
    }
    return np.array(dense[selected_indices], copy=True), report


def collect_patch04_mesh_belt(
    mesh_info: PlyInfo,
    support: LinearEdgeSupport,
    chunk_records: int,
) -> np.ndarray:
    source = records(mesh_info)
    pieces: list[np.ndarray] = []
    component = support.component
    for start in range(0, mesh_info.vertex_count, chunk_records):
        end = min(start + chunk_records, mesh_info.vertex_count)
        chunk = source[start:end]
        x = np.asarray(chunk[mesh_info.fields["x"]], dtype=np.float64)
        y = np.asarray(chunk[mesh_info.fields["y"]], dtype=np.float64)
        gx, gy, within = component.grid_coordinates(x, y)
        selected = np.zeros(len(chunk), dtype=bool)
        if np.any(within):
            selected[within] = support.transition_mask[gy[within], gx[within]]
        if np.any(selected):
            pieces.append(np.array(chunk[selected], copy=True))
        if start == 0 or end == mesh_info.vertex_count or end // chunk_records % 10 == 0:
            print(
                f"  Patch 04 mesh transition pass: {100*end/mesh_info.vertex_count:5.1f}%",
                flush=True,
            )
    del source
    result = np.concatenate(pieces) if pieces else np.empty(0, dtype=mesh_info.dtype)
    print(f"    Patch 04: {len(result):,} raw mesh transition candidates", flush=True)
    return result


def filter_patch04_mesh_geometry(
    mesh: np.ndarray,
    mesh_info: PlyInfo,
    local_1mm: np.ndarray,
    info_1mm: PlyInfo,
) -> tuple[np.ndarray, dict[str, Any]]:
    if cv2 is None:
        raise RuntimeError("OpenCV (cv2) is required for Patch 04 geometry filtering")
    if not len(mesh) or not len(local_1mm):
        raise ValueError("Patch 04 geometry filtering requires mesh and measured points")
    measured_xyz = np.column_stack([
        local_1mm[info_1mm.fields[channel]]
        for channel in ("x", "y", "z")
    ]).astype(np.float32, copy=False)
    measured_normals = np.column_stack([
        local_1mm[info_1mm.fields[channel]]
        for channel in ("nx", "ny", "nz")
    ]).astype(np.float32, copy=False)
    nearest_index = cv2.flann_Index(
        measured_xyz,
        {"algorithm": 1, "trees": 4},
    )
    kept: list[np.ndarray] = []
    distances: list[np.ndarray] = []
    normal_dots: list[np.ndarray] = []
    nearest_scan_counts: collections.Counter[int] = collections.Counter()
    query_chunk = 250_000
    for begin in range(0, len(mesh), query_chunk):
        end = min(begin + query_chunk, len(mesh))
        subset = mesh[begin:end]
        query_xyz = np.column_stack([
            subset[mesh_info.fields[channel]]
            for channel in ("x", "y", "z")
        ]).astype(np.float32, copy=False)
        nearest, distance_squared = nearest_index.knnSearch(
            query_xyz,
            1,
            params={"checks": 64},
        )
        nearest = nearest[:, 0]
        distance = np.sqrt(np.maximum(distance_squared[:, 0], 0.0))
        query_normals = np.column_stack([
            subset[mesh_info.fields[channel]]
            for channel in ("nx", "ny", "nz")
        ]).astype(np.float32, copy=False)
        dot = np.abs(np.sum(query_normals * measured_normals[nearest], axis=1))
        accepted = (
            np.isfinite(distance)
            & np.isfinite(dot)
            & (distance <= PATCH_04_GEOMETRY_MAX_DISTANCE_METRES)
            & (dot >= PATCH_04_GEOMETRY_NORMAL_DOT_MIN)
        )
        if np.any(accepted):
            kept.append(np.array(subset[accepted], copy=True))
        distances.append(distance)
        normal_dots.append(dot)
        nearest_ids, valid_ids = _scan_ids(local_1mm[nearest], info_1mm)
        values, counts = np.unique(nearest_ids[valid_ids], return_counts=True)
        nearest_scan_counts.update({int(value): int(count) for value, count in zip(values, counts, strict=True)})
        if begin == 0 or end == len(mesh) or end // query_chunk % 10 == 0:
            print(
                f"  Patch 04 geometry agreement: {100*end/len(mesh):5.1f}%",
                flush=True,
            )
    del nearest_index, measured_xyz, measured_normals
    all_distances = np.concatenate(distances)
    all_dots = np.concatenate(normal_dots)
    result = np.concatenate(kept) if kept else np.empty(0, dtype=mesh_info.dtype)
    report = {
        "requested": int(len(mesh)),
        "accepted": int(len(result)),
        "rejected": int(len(mesh) - len(result)),
        "acceptance_fraction": float(len(result) / len(mesh)),
        "maximum_distance_m": PATCH_04_GEOMETRY_MAX_DISTANCE_METRES,
        "minimum_abs_normal_dot": PATCH_04_GEOMETRY_NORMAL_DOT_MIN,
        "distance_p50_m": float(np.percentile(all_distances, 50)),
        "distance_p95_m": float(np.percentile(all_distances, 95)),
        "normal_dot_p05": float(np.percentile(all_dots, 5)),
        "nearest_scan_counts": dict(sorted(nearest_scan_counts.items())),
    }
    if not len(result):
        raise ValueError("Patch 04 mesh filter rejected every transition candidate")
    return result, report


def _surface_cell_ranges(
    points: np.ndarray,
    info: PlyInfo,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    x = np.asarray(points[info.fields["x"]], dtype=np.float64)
    y = np.asarray(points[info.fields["y"]], dtype=np.float64)
    z = np.asarray(points[info.fields["z"]], dtype=np.float64)
    keys = _pack_cells(
        np.floor(x / COMPONENT_CELL_METRES).astype(np.int64),
        np.floor(y / COMPONENT_CELL_METRES).astype(np.int64),
    )
    order = np.argsort(keys, kind="stable")
    unique, starts = np.unique(keys[order], return_index=True)
    sorted_z = z[order]
    z_min = np.minimum.reduceat(sorted_z, starts)
    z_max = np.maximum.reduceat(sorted_z, starts)
    return unique, z_min, z_max


def _robust_density_level(values: np.ndarray) -> float:
    finite = np.asarray(values, dtype=np.float64)
    finite = finite[np.isfinite(finite)]
    if not len(finite):
        return 0.0
    low, high = np.percentile(finite, (10, 90))
    return float(np.mean(np.clip(finite, low, high)))


def plan_patch04_edge_additions(
    local: np.ndarray,
    info: PlyInfo,
    support: LinearEdgeSupport,
) -> tuple[dict[int, int], dict[str, Any]]:
    component = support.component
    total, id9 = _grid_counts_for_scan(
        local,
        info,
        component,
        support.source_scan_id,
    )
    smoothed_total = cv2.GaussianBlur(total.astype(np.float32), (0, 0), 1.0)
    signed = support.signed_distance
    along = support.along_distance
    along_mask = (along >= support.along_min) & (along <= support.along_max)
    high_strip = (
        support.high_density_mask
        & (signed >= -0.120)
        & (signed <= -0.020)
        & along_mask
        & (total > 0)
    )
    ambient_strip = (
        ~support.high_density_mask
        & (signed >= support.blend_width_metres * 0.70)
        & (signed <= support.blend_width_metres)
        & along_mask
        & (total > 0)
    )
    high_level = _robust_density_level(smoothed_total[high_strip])
    ambient_level = _robust_density_level(smoothed_total[ambient_strip])
    amplitude = max(0.0, high_level - ambient_level)
    if high_level <= 0.0 or ambient_level <= 0.0:
        raise ValueError("Patch 04 density reference strips are empty")
    if amplitude < max(1.0, ambient_level * 0.02):
        raise ValueError(
            f"Patch 04 has no material ScanID 9 density step ({high_level:.2f} vs {ambient_level:.2f})"
        )
    if support.fade_width_map is not None:
        fade_width = support.fade_width_map
        fade_span = np.maximum(
            fade_width - support.core_width_metres,
            COMPONENT_CELL_METRES,
        )
        fade_distance = np.maximum(signed - support.core_width_metres, 0.0)
        u = np.clip(1.0 - fade_distance / fade_span, 0.0, 1.0)
        active_transition = support.transition_mask & (signed <= fade_width)
        density_target = "scan9-jagged-edge-ramp"
        falloff_name = "cubic smoothstep over connected jagged reach"
    else:
        u = np.clip(1.0 - signed / support.blend_width_metres, 0.0, 1.0)
        active_transition = support.transition_mask
        density_target = "scan9-linear-edge-ramp"
        falloff_name = "cubic smoothstep"
    falloff = u * u * (3.0 - 2.0 * u)
    additions = np.rint(amplitude * falloff).astype(np.int64)
    additions[~active_transition] = 0
    y, x = np.nonzero(additions > 0)
    keys = _pack_cells(x + component.origin_ix, y + component.origin_iy)
    requested = {
        int(key): int(count)
        for key, count in zip(keys, additions[y, x], strict=True)
    }
    transition_values = total[active_transition]
    near_edge = active_transition & (signed <= COMPONENT_CELL_METRES * 1.5)
    report = {
        "density_target": density_target,
        "source_scan_id": support.source_scan_id,
        "blend_width_m": support.blend_width_metres,
        "planned_additions": int(np.sum(additions)),
        "transition_cell_count": int(np.count_nonzero(active_transition)),
        "deficient_cells": int(len(requested)),
        "current_points": int(np.sum(transition_values)),
        "interior_mean_points_per_cell": float(np.mean(transition_values)) if len(transition_values) else 0.0,
        "ring_mean_points_per_cell": high_level,
        "high_reference_points_per_cell": high_level,
        "ambient_reference_points_per_cell": ambient_level,
        "density_step_before_points_per_cell": amplitude,
        "near_edge_current_points_per_cell": float(np.mean(total[near_edge])) if np.any(near_edge) else 0.0,
        "near_edge_target_points_per_cell": high_level,
        "falloff": falloff_name,
        "existing_records_modified": 0,
    }
    if support.fade_width_map is not None:
        active_widths = support.fade_width_map[active_transition]
        active_planarity = (
            support.surface_planarity_map[active_transition]
            if support.surface_planarity_map is not None
            else np.full(len(active_widths), 0.5)
        )
        report.update({
            "edge_profile": support.edge_profile,
            "core_width_m": support.core_width_metres,
            "noise_seed": support.noise_seed,
            "fade_width_min_m": float(np.min(active_widths)),
            "fade_width_p05_m": float(np.percentile(active_widths, 5)),
            "fade_width_p50_m": float(np.percentile(active_widths, 50)),
            "fade_width_p95_m": float(np.percentile(active_widths, 95)),
            "fade_width_max_m": float(np.max(active_widths)),
            "surface_planarity_p50": float(np.percentile(active_planarity, 50)),
        })
    return requested, report


def _copy_dense_to_output_schema(dense: np.ndarray, dense_info: PlyInfo, output_info: PlyInfo) -> np.ndarray:
    result = np.zeros(len(dense), dtype=output_info.dtype)
    dense_by_normalised = {_normalised_name(name): name for name in dense_info.property_names}
    for output_name in output_info.property_names:
        dense_name = dense_by_normalised.get(_normalised_name(output_name))
        if dense_name is not None:
            result[output_name] = dense[dense_name]
    result[output_info.fields["scan_id"]] = float(ADDITION_SCAN_ID)
    return result


def _xy_cell_index(
    points: np.ndarray,
    info: PlyInfo,
    cell: float = 0.001,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    x = np.asarray(points[info.fields["x"]], dtype=np.float64)
    y = np.asarray(points[info.fields["y"]], dtype=np.float64)
    z = np.asarray(points[info.fields["z"]], dtype=np.float64)
    ix = np.floor(x / cell).astype(np.int64)
    iy = np.floor(y / cell).astype(np.int64)
    keys = _pack_cells(ix, iy)
    order = np.argsort(keys, kind="stable")
    unique, starts, counts = np.unique(keys[order], return_index=True, return_counts=True)
    return unique, starts, counts, order, x, y, z


def transfer_ar_nearest(
    additions: np.ndarray,
    output_info: PlyInfo,
    source_1mm: np.ndarray,
    source_info: PlyInfo,
) -> tuple[np.ndarray, dict[str, Any]]:
    if not len(additions):
        return additions, {"requested": 0, "transferred": 0, "skipped": 0, "max_distance_m": 0.0}
    if output_info.ar_fields != source_info.ar_fields:
        raise ValueError("The output and 1 mm A_R field bundles do not match")
    unique_cells, cell_starts, cell_counts, cell_order, sx, sy, sz = _xy_cell_index(
        source_1mm,
        source_info,
    )
    def cell_indices(ix: int, iy: int) -> np.ndarray | None:
        key = _pack_cell_scalar(ix, iy)
        position = int(np.searchsorted(unique_cells, key))
        if position >= len(unique_cells) or int(unique_cells[position]) != key:
            return None
        start = int(cell_starts[position])
        return cell_order[start:start + int(cell_counts[position])]
    ax = np.asarray(additions[output_info.fields["x"]], dtype=np.float64)
    ay = np.asarray(additions[output_info.fields["y"]], dtype=np.float64)
    az = np.asarray(additions[output_info.fields["z"]], dtype=np.float64)
    cell = 0.001
    qx = np.floor(ax / cell).astype(np.int64)
    qy = np.floor(ay / cell).astype(np.int64)
    nearest = np.full(len(additions), -1, dtype=np.int64)
    distances = np.full(len(additions), np.inf, dtype=np.float64)
    for point_index in range(len(additions)):
        best_index = -1
        best_sq = float("inf")
        checked_radius = -1
        # First find a finite upper bound in expanding XY rings.
        for radius in range(0, 6):
            found = False
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    if radius and max(abs(dx), abs(dy)) != radius:
                        continue
                    indices = cell_indices(
                        int(qx[point_index] + dx),
                        int(qy[point_index] + dy),
                    )
                    if indices is None:
                        continue
                    found = True
                    delta_x = sx[indices] - ax[point_index]
                    delta_y = sy[indices] - ay[point_index]
                    delta_z = sz[indices] - az[point_index]
                    distance_sq = delta_x*delta_x + delta_y*delta_y + delta_z*delta_z
                    local = int(np.argmin(distance_sq))
                    if float(distance_sq[local]) < best_sq:
                        best_sq = float(distance_sq[local])
                        best_index = int(indices[local])
            checked_radius = radius
            if found and math.sqrt(best_sq) <= max(0.0, (radius - 1) * cell):
                break
        # Search every XY cell that could still contain a closer 3D point.
        if best_index >= 0:
            exact_radius = min(6, int(math.ceil(math.sqrt(best_sq) / cell)) + 1)
            for dx in range(-exact_radius, exact_radius + 1):
                for dy in range(-exact_radius, exact_radius + 1):
                    if max(abs(dx), abs(dy)) <= checked_radius:
                        continue
                    indices = cell_indices(
                        int(qx[point_index] + dx),
                        int(qy[point_index] + dy),
                    )
                    if indices is None:
                        continue
                    delta_x = sx[indices] - ax[point_index]
                    delta_y = sy[indices] - ay[point_index]
                    delta_z = sz[indices] - az[point_index]
                    distance_sq = delta_x*delta_x + delta_y*delta_y + delta_z*delta_z
                    local = int(np.argmin(distance_sq))
                    if float(distance_sq[local]) < best_sq:
                        best_sq = float(distance_sq[local])
                        best_index = int(indices[local])
        if best_index >= 0:
            nearest[point_index] = best_index
            distances[point_index] = math.sqrt(best_sq)
    valid = (nearest >= 0) & (distances <= NEAREST_AR_MAX_METRES)
    transferred = np.array(additions[valid], copy=True)
    source_indices = nearest[valid]
    for field in output_info.ar_fields:
        transferred[field] = source_1mm[field][source_indices]
    report = {
        "requested": int(len(additions)),
        "transferred": int(np.count_nonzero(valid)),
        "skipped": int(np.count_nonzero(~valid)),
        "max_distance_m": float(np.max(distances[valid])) if np.any(valid) else None,
        "p95_distance_m": float(np.percentile(distances[valid], 95)) if np.any(valid) else None,
    }
    return transferred, report


def transfer_full_nearest_1mm(
    additions: np.ndarray,
    output_info: PlyInfo,
    source_1mm: np.ndarray,
    source_info: PlyInfo,
) -> tuple[np.ndarray, dict[str, Any]]:
    """Copy scalar bundles from the nearest 1 mm point and blend measured RGB."""
    if cv2 is None:
        raise RuntimeError("OpenCV (cv2) is required for Patch 04 scalar transfer")
    if not len(additions):
        return additions, {
            "requested": 0,
            "transferred": 0,
            "skipped": 0,
            "max_distance_m": 0.0,
            "rgb_neighbours": PATCH_04_RGB_NEIGHBOURS,
        }
    if not len(source_1mm):
        raise ValueError("Patch 04 scalar transfer has no local 1 mm source records")
    source_xyz = np.column_stack([
        source_1mm[source_info.fields[channel]]
        for channel in ("x", "y", "z")
    ]).astype(np.float32, copy=False)
    index = cv2.flann_Index(source_xyz, {"algorithm": 1, "trees": 4})
    nearest_parts: list[np.ndarray] = []
    distance_parts: list[np.ndarray] = []
    query_chunk = 250_000
    neighbours = min(PATCH_04_RGB_NEIGHBOURS, len(source_1mm))
    for begin in range(0, len(additions), query_chunk):
        end = min(begin + query_chunk, len(additions))
        query_xyz = np.column_stack([
            additions[output_info.fields[channel]][begin:end]
            for channel in ("x", "y", "z")
        ]).astype(np.float32, copy=False)
        nearest, distance_squared = index.knnSearch(
            query_xyz,
            neighbours,
            params={"checks": 64},
        )
        nearest_parts.append(nearest.astype(np.int64, copy=False))
        distance_parts.append(np.sqrt(np.maximum(distance_squared, 0.0)))
    del index, source_xyz
    nearest = np.concatenate(nearest_parts)
    distances = np.concatenate(distance_parts)
    transfer_limit = PATCH_04_GEOMETRY_MAX_DISTANCE_METRES * 1.5
    valid = np.isfinite(distances[:, 0]) & (distances[:, 0] <= transfer_limit)
    transferred = np.array(additions[valid], copy=True)
    nearest = nearest[valid]
    distances = distances[valid]
    nearest_scalar = nearest[:, 0]

    source_by_normalised = {
        _normalised_name(name): name
        for name in source_info.property_names
    }
    protected = {
        output_info.fields[name]
        for name in ("x", "y", "z", "red", "green", "blue", "nx", "ny", "nz", "scan_id")
    }
    copied_fields: list[str] = []
    for output_name in output_info.property_names:
        if output_name in protected:
            continue
        source_name = source_by_normalised.get(_normalised_name(output_name))
        if source_name is None:
            raise ValueError(
                f"1 mm cloud has no nearest-transfer source for {output_name}"
            )
        transferred[output_name] = source_1mm[source_name][nearest_scalar]
        copied_fields.append(output_name)

    for channel in ("nx", "ny", "nz"):
        transferred[output_info.fields[channel]] = source_1mm[
            source_info.fields[channel]
        ][nearest_scalar]

    source_rgb = np.stack([
        source_1mm[source_info.fields[channel]][nearest]
        for channel in ("red", "green", "blue")
    ], axis=2)
    neighbour_lab = _srgb_to_oklab(source_rgb.reshape(-1, 3)).reshape(
        len(transferred), neighbours, 3
    )
    weights = 1.0 / np.square(distances + 0.0005)
    weights[distances > PATCH_04_RGB_RADIUS_METRES] = 0.0
    empty_weights = np.sum(weights, axis=1) <= 0.0
    if np.any(empty_weights):
        weights[empty_weights, 0] = 1.0
    weights /= np.sum(weights, axis=1, keepdims=True)
    blended_lab = np.sum(neighbour_lab * weights[:, :, None], axis=1)
    blended_rgb, gamut_fraction = _oklab_to_srgb_bytes(blended_lab)
    original_rgb = np.column_stack([
        transferred[output_info.fields[channel]]
        for channel in ("red", "green", "blue")
    ])
    original_lab = _srgb_to_oklab(original_rgb)
    for channel_index, channel in enumerate(("red", "green", "blue")):
        transferred[output_info.fields[channel]] = blended_rgb[:, channel_index]
    transferred[output_info.fields["scan_id"]] = float(ADDITION_SCAN_ID)

    report = {
        "requested": int(len(additions)),
        "transferred": int(len(transferred)),
        "skipped": int(np.count_nonzero(~valid)),
        "max_distance_m": float(np.max(distances[:, 0])) if len(distances) else None,
        "p95_distance_m": float(np.percentile(distances[:, 0], 95)) if len(distances) else None,
        "transfer_limit_m": transfer_limit,
        "copied_scalar_fields": copied_fields,
        "copied_scalar_field_count": len(copied_fields),
        "rgb_neighbours": neighbours,
        "rgb_radius_m": PATCH_04_RGB_RADIUS_METRES,
        "rgb_source": "inverse-distance Oklab blend of measured 1 mm neighbours",
        "mesh_to_blended_rgb_delta_e_median": float(
            np.median(np.linalg.norm(original_lab - blended_lab, axis=1))
        ) if len(transferred) else 0.0,
        "rgb_gamut_compressed_fraction": gamut_fraction,
    }
    return transferred, report


def prepare_additions_for_density(
    spacing_mm: int,
    local_records: Sequence[np.ndarray],
    local_1mm: Sequence[np.ndarray],
    info: PlyInfo,
    info_1mm: PlyInfo,
    dense_info: PlyInfo,
    components: Sequence[ComponentSupport],
    models: dict[tuple[int, int], CorrectionModel],
) -> tuple[np.ndarray, list[dict[str, Any]]]:
    additions: list[np.ndarray] = []
    reports: list[dict[str, Any]] = []
    spacing_metres = spacing_mm / 1000.0
    for component, local, source_1mm in zip(components, local_records, local_1mm, strict=True):
        requested, density_report = plan_density_additions(local, info, component)
        selected_dense, selection_report = select_dense_additions(
            component.dense_records,
            dense_info,
            local,
            info,
            component,
            requested,
            spacing_metres,
        )
        converted = _copy_dense_to_output_schema(selected_dense, dense_info, info)
        if len(converted):
            # This input is already the colour-matched CleanMesh artifact.
            # Correct its scalar fields to the local host, but do not apply a
            # second RGB match (which exaggerates the filled footprint).
            dense_rgb = np.column_stack([
                converted[info.fields[channel]].copy()
                for channel in ("red", "green", "blue")
            ])
            all_selected = np.ones(len(converted), dtype=bool)
            apply_correction_model(
                converted,
                all_selected,
                info,
                component,
                models[(component.cleanmesh_index, 11)],
            )
            for channel_index, channel in enumerate(("red", "green", "blue")):
                converted[info.fields[channel]] = dense_rgb[:, channel_index]
            converted[info.fields["scan_id"]] = float(ADDITION_SCAN_ID)
        converted, transfer_report = transfer_ar_nearest(converted, info, source_1mm, info_1mm)
        additions.append(converted)
        reports.append({
            "camera": component.camera_name,
            "component": component.cleanmesh_index,
            "host_scan_id": component.host_scan_id,
            "density": density_report,
            "selection": selection_report,
            "ar_transfer": transfer_report,
            "accepted_additions": int(len(converted)),
        })
        print(
            f"    {component.camera_name}: planned {density_report['planned_additions']:,}, "
            f"selected {selection_report['selected']:,}, accepted {len(converted):,}",
            flush=True,
        )
    combined = np.concatenate(additions) if any(len(value) for value in additions) else np.empty(0, dtype=info.dtype)
    return combined, reports


INDEX_MAGIC = b"IPIDX1\0\0"


def _write_index_header(stream: Any, count: int) -> None:
    stream.write(INDEX_MAGIC)
    stream.write(struct.pack("<Q", count))


def _read_indices(path: Path) -> np.ndarray:
    with path.open("rb") as stream:
        if stream.read(len(INDEX_MAGIC)) != INDEX_MAGIC:
            raise ValueError(f"{path} has an invalid index-map signature")
        count_bytes = stream.read(8)
        if len(count_bytes) != 8:
            raise ValueError(f"{path} has a truncated index-map header")
        count = struct.unpack("<Q", count_bytes)[0]
        result = np.fromfile(stream, dtype="<u8", count=count)
        if len(result) != count or stream.read(1):
            raise ValueError(f"{path} has an invalid index-map payload")
        return result


def _target_masks(
    chunk: np.ndarray,
    info: PlyInfo,
    components: Sequence[ComponentSupport],
) -> tuple[np.ndarray, list[tuple[ComponentSupport, np.ndarray, np.ndarray]]]:
    x = np.asarray(chunk[info.fields["x"]], dtype=np.float64)
    y = np.asarray(chunk[info.fields["y"]], dtype=np.float64)
    z = np.asarray(chunk[info.fields["z"]], dtype=np.float64)
    scan_ids, valid_scan = _scan_ids(chunk, info)
    combined = np.zeros(len(chunk), dtype=bool)
    component_masks: list[tuple[ComponentSupport, np.ndarray, np.ndarray]] = []
    for component in components:
        inside = component.contains(x, y, z)
        id10 = inside & valid_scan & (scan_ids == 10)
        id11 = inside & valid_scan & (scan_ids == 11)
        overlap = combined & (id10 | id11)
        if np.any(overlap):
            raise ValueError("Target component masks overlap")
        combined |= id10 | id11
        component_masks.append((component, id10, id11))
    return combined, component_masks


def count_selected_records(local_records: Sequence[np.ndarray], info: PlyInfo, components: Sequence[ComponentSupport]) -> int:
    count = 0
    for local, component in zip(local_records, components, strict=True):
        scan_ids, valid = _scan_ids(local, info)
        inside = component.contains(local[info.fields["x"]], local[info.fields["y"]], local[info.fields["z"]])
        count += int(np.count_nonzero(inside & valid & np.isin(scan_ids, SOURCE_SCAN_IDS)))
    return count


def write_subset_ply(path: Path, info: PlyInfo, subset: np.ndarray) -> str:
    digest = hashlib.sha256()
    header = patch_header_count(info.header, len(subset))
    with path.open("wb") as stream:
        stream.write(header)
        stream.write(subset.tobytes(order="C"))
    digest.update(header)
    digest.update(subset.tobytes(order="C"))
    return digest.hexdigest()


def build_density_candidate(
    spacing_mm: int,
    info: PlyInfo,
    local_records: Sequence[np.ndarray],
    additions: np.ndarray,
    components: Sequence[ComponentSupport],
    models: dict[tuple[int, int], CorrectionModel],
    density_dir: Path,
    chunk_records: int,
) -> dict[str, Any]:
    density_dir.mkdir(parents=True, exist_ok=False)
    selected_count = count_selected_records(local_records, info, components)
    candidate_path = density_dir / f"Site3-SAND-{spacing_mm}mm-candidate.ply"
    originals_path = density_dir / "original-scanid10-11.ply"
    replacements_path = density_dir / "replacement-scanid12.ply"
    additions_path = density_dir / "additions-scanid13.ply"
    indices_path = density_dir / "original-vertex-indices.u64"
    candidate_header = patch_header_count(info.header, info.vertex_count + len(additions))
    backup_header = patch_header_count(info.header, selected_count)
    source_digest = hashlib.sha256(info.header)
    candidate_digest = hashlib.sha256(candidate_header)
    source = records(info)
    written_selected = 0
    gamut_weighted = 0.0
    with (
        candidate_path.open("wb") as candidate_stream,
        originals_path.open("wb") as original_stream,
        replacements_path.open("wb") as replacement_stream,
        indices_path.open("wb") as index_stream,
    ):
        candidate_stream.write(candidate_header)
        original_stream.write(backup_header)
        replacement_stream.write(backup_header)
        _write_index_header(index_stream, selected_count)
        for start in range(0, info.vertex_count, chunk_records):
            end = min(start + chunk_records, info.vertex_count)
            original = np.array(source[start:end], copy=True)
            original_bytes = original.tobytes(order="C")
            source_digest.update(original_bytes)
            scan_ids, valid_scan = _scan_ids(original, info)
            if np.any(valid_scan & np.isin(scan_ids, (REPLACEMENT_SCAN_ID, ADDITION_SCAN_ID))):
                raise ValueError(f"{info.path} already contains ScanID 12 or 13")
            selected, component_masks = _target_masks(original, info, components)
            updated = np.array(original, copy=True)
            for component, id10, id11 in component_masks:
                for source_id, mask in ((10, id10), (11, id11)):
                    if np.any(mask):
                        fraction = apply_correction_model(
                            updated, mask, info, component,
                            models[(component.cleanmesh_index, source_id)],
                        )
                        gamut_weighted += fraction * int(np.count_nonzero(mask))
            if np.any(selected):
                original_stream.write(original[selected].tobytes(order="C"))
                replacement_stream.write(updated[selected].tobytes(order="C"))
                indices = np.flatnonzero(selected).astype("<u8") + np.uint64(start)
                index_stream.write(indices.tobytes(order="C"))
                written_selected += len(indices)
            updated_bytes = updated.tobytes(order="C")
            candidate_stream.write(updated_bytes)
            candidate_digest.update(updated_bytes)
            if start == 0 or end == info.vertex_count or end // chunk_records % 10 == 0:
                print(f"  {spacing_mm} mm candidate: {100*end/info.vertex_count:5.1f}%", flush=True)
        if len(additions):
            addition_bytes = additions.tobytes(order="C")
            candidate_stream.write(addition_bytes)
            candidate_digest.update(addition_bytes)
    del source
    if written_selected != selected_count:
        raise ValueError(f"{spacing_mm} mm selected-count drift: expected {selected_count}, wrote {written_selected}")
    additions_hash = write_subset_ply(additions_path, info, additions)
    result = {
        "spacing_mm": spacing_mm,
        "source_path": str(info.path),
        "source_size": info.path.stat().st_size,
        "source_mtime_ns": info.path.stat().st_mtime_ns,
        "source_sha256": source_digest.hexdigest(),
        "source_vertex_count": info.vertex_count,
        "source_header_hex": info.header.hex(),
        "record_stride": info.stride,
        "property_names": list(info.property_names),
        "candidate_path": str(candidate_path.resolve()),
        "candidate_sha256": candidate_digest.hexdigest(),
        "candidate_vertex_count": info.vertex_count + len(additions),
        "originals_path": str(originals_path.resolve()),
        "replacements_path": str(replacements_path.resolve()),
        "indices_path": str(indices_path.resolve()),
        "additions_path": str(additions_path.resolve()),
        "additions_sha256": additions_hash,
        "replacement_count": selected_count,
        "addition_count": int(len(additions)),
        "gamut_compressed_fraction": gamut_weighted / max(selected_count, 1),
    }
    return result


def build_append_only_density_candidate(
    spacing_mm: int,
    info: PlyInfo,
    additions: np.ndarray,
    density_dir: Path,
    chunk_records: int,
    role: str = "SAND",
) -> dict[str, Any]:
    """Build a byte-identical base payload followed only by new ScanID 13 records."""
    density_dir.mkdir(parents=True, exist_ok=False)
    if role not in {"ROCK", "SAND", "VEG"}:
        raise ValueError(f"Unsupported candidate role: {role}")
    candidate_path = density_dir / f"Site3-{role}-{spacing_mm}mm-candidate.ply"
    originals_path = density_dir / "original-scanid10-11.ply"
    replacements_path = density_dir / "replacement-scanid12.ply"
    additions_path = density_dir / "additions-scanid13.ply"
    indices_path = density_dir / "original-vertex-indices.u64"
    candidate_header = patch_header_count(
        info.header,
        info.vertex_count + len(additions),
    )
    empty = np.empty(0, dtype=info.dtype)
    empty_hash = write_subset_ply(originals_path, info, empty)
    replacement_hash = write_subset_ply(replacements_path, info, empty)
    if replacement_hash != empty_hash:
        raise ValueError("Empty append-only rollback artifacts disagree")
    with indices_path.open("wb") as index_stream:
        _write_index_header(index_stream, 0)

    if len(additions):
        addition_ids, valid = _scan_ids(additions, info)
        if not np.all(valid & (addition_ids == ADDITION_SCAN_ID)):
            raise ValueError("Append-only candidate contains a non-ScanID 13 addition")
    source_digest = hashlib.sha256(info.header)
    candidate_digest = hashlib.sha256(candidate_header)
    source = records(info)
    with candidate_path.open("wb") as candidate_stream:
        candidate_stream.write(candidate_header)
        for start in range(0, info.vertex_count, chunk_records):
            end = min(start + chunk_records, info.vertex_count)
            payload = np.array(source[start:end], copy=True).tobytes(order="C")
            source_digest.update(payload)
            candidate_digest.update(payload)
            candidate_stream.write(payload)
            if start == 0 or end == info.vertex_count or end // chunk_records % 10 == 0:
                print(
                    f"  {spacing_mm} mm append-only candidate: "
                    f"{100*end/info.vertex_count:5.1f}%",
                    flush=True,
                )
        if len(additions):
            payload = additions.tobytes(order="C")
            candidate_stream.write(payload)
            candidate_digest.update(payload)
    del source
    additions_hash = write_subset_ply(additions_path, info, additions)
    return {
        "spacing_mm": spacing_mm,
        "source_path": str(info.path),
        "source_size": info.path.stat().st_size,
        "source_mtime_ns": info.path.stat().st_mtime_ns,
        "source_sha256": source_digest.hexdigest(),
        "source_vertex_count": info.vertex_count,
        "source_header_hex": info.header.hex(),
        "record_stride": info.stride,
        "property_names": list(info.property_names),
        "candidate_path": str(candidate_path.resolve()),
        "candidate_sha256": candidate_digest.hexdigest(),
        "candidate_vertex_count": info.vertex_count + len(additions),
        "originals_path": str(originals_path.resolve()),
        "replacements_path": str(replacements_path.resolve()),
        "indices_path": str(indices_path.resolve()),
        "additions_path": str(additions_path.resolve()),
        "additions_sha256": additions_hash,
        "replacement_count": 0,
        "addition_count": int(len(additions)),
        "gamut_compressed_fraction": 0.0,
    }


def _prefix_info_from_installed_edge(
    current_info: PlyInfo,
    installed_entry: dict[str, Any],
) -> PlyInfo:
    base_count = int(installed_entry["source_vertex_count"])
    installed_count = int(installed_entry["candidate_vertex_count"])
    if current_info.vertex_count != installed_count:
        raise ValueError(
            f"Installed edge count changed for {current_info.path}: "
            f"expected {installed_count:,}, found {current_info.vertex_count:,}"
        )
    if installed_count != base_count + int(installed_entry["addition_count"]):
        raise ValueError("Installed edge manifest is not a simple append-only candidate")
    base_header = bytes.fromhex(installed_entry["source_header_hex"])
    if len(base_header) != current_info.header_size:
        raise ValueError("Installed edge base/current PLY header sizes differ")
    property_names = tuple(installed_entry.get("property_names", ()))
    if property_names and property_names != current_info.property_names:
        raise ValueError("Installed edge schema differs from the canonical ROCK cloud")
    return PlyInfo(
        path=current_info.path,
        header=base_header,
        vertex_count=base_count,
        dtype=current_info.dtype,
        property_names=current_info.property_names,
        fields=current_info.fields,
    )


def build_tail_replacement_candidate(
    spacing_mm: int,
    current_info: PlyInfo,
    base_info: PlyInfo,
    installed_entry: dict[str, Any],
    additions: np.ndarray,
    density_dir: Path,
    chunk_records: int,
    superseded_run_dir: Path,
) -> dict[str, Any]:
    """Replace one known append-only ScanID-13 tail without touching its base prefix."""
    density_dir.mkdir(parents=True, exist_ok=False)
    base_count = base_info.vertex_count
    superseded_count = current_info.vertex_count - base_count
    if superseded_count != int(installed_entry["addition_count"]):
        raise ValueError(
            f"Superseded {spacing_mm} mm tail count changed: expected "
            f"{installed_entry['addition_count']:,}, found {superseded_count:,}"
        )
    if len(additions):
        addition_ids, valid = _scan_ids(additions, base_info)
        if not np.all(valid & (addition_ids == ADDITION_SCAN_ID)):
            raise ValueError("Replacement candidate contains a non-ScanID 13 addition")

    candidate_path = density_dir / f"Site3-ROCK-{spacing_mm}mm-candidate.ply"
    originals_path = density_dir / "original-scanid10-11.ply"
    replacements_path = density_dir / "replacement-scanid12.ply"
    additions_path = density_dir / "additions-scanid13.ply"
    indices_path = density_dir / "original-vertex-indices.u64"
    superseded_path = density_dir / "superseded-additions-scanid13.ply"
    empty = np.empty(0, dtype=base_info.dtype)
    empty_hash = write_subset_ply(originals_path, base_info, empty)
    if write_subset_ply(replacements_path, base_info, empty) != empty_hash:
        raise ValueError("Empty tail-replacement rollback artifacts disagree")
    with indices_path.open("wb") as index_stream:
        _write_index_header(index_stream, 0)

    candidate_header = patch_header_count(
        base_info.header,
        base_count + len(additions),
    )
    source_digest = hashlib.sha256(current_info.header)
    base_digest = hashlib.sha256(base_info.header)
    candidate_digest = hashlib.sha256(candidate_header)
    source = records(current_info)
    with candidate_path.open("wb") as candidate_stream:
        candidate_stream.write(candidate_header)
        for start in range(0, base_count, chunk_records):
            end = min(start + chunk_records, base_count)
            payload = np.array(source[start:end], copy=True).tobytes(order="C")
            source_digest.update(payload)
            base_digest.update(payload)
            candidate_digest.update(payload)
            candidate_stream.write(payload)
            if start == 0 or end == base_count or end // chunk_records % 10 == 0:
                print(
                    f"  {spacing_mm} mm tail-replacement candidate: "
                    f"{100*end/base_count:5.1f}%",
                    flush=True,
                )
        superseded = np.array(source[base_count:current_info.vertex_count], copy=True)
        if len(superseded):
            superseded_ids, valid = _scan_ids(superseded, current_info)
            if not np.all(valid & (superseded_ids == ADDITION_SCAN_ID)):
                raise ValueError("Superseded edge tail contains records other than ScanID 13")
            source_digest.update(superseded.tobytes(order="C"))
        if len(additions):
            payload = additions.tobytes(order="C")
            candidate_stream.write(payload)
            candidate_digest.update(payload)
    del source

    source_sha = source_digest.hexdigest()
    base_sha = base_digest.hexdigest()
    if source_sha != installed_entry["candidate_sha256"]:
        raise ValueError(
            f"Canonical {spacing_mm} mm ROCK cloud is not the installed edge candidate"
        )
    if base_sha != installed_entry["source_sha256"]:
        raise ValueError(
            f"Canonical {spacing_mm} mm base prefix no longer matches the pre-edge cloud"
        )
    additions_hash = write_subset_ply(additions_path, base_info, additions)
    superseded_hash = write_subset_ply(superseded_path, base_info, superseded)
    return {
        "spacing_mm": spacing_mm,
        "role": "ROCK",
        "source_path": str(current_info.path),
        "source_size": current_info.path.stat().st_size,
        "source_mtime_ns": current_info.path.stat().st_mtime_ns,
        "source_sha256": source_sha,
        "source_vertex_count": current_info.vertex_count,
        "source_header_hex": current_info.header.hex(),
        "record_stride": current_info.stride,
        "property_names": list(current_info.property_names),
        "candidate_path": str(candidate_path.resolve()),
        "candidate_sha256": candidate_digest.hexdigest(),
        "candidate_vertex_count": base_count + len(additions),
        "candidate_base_vertex_count": base_count,
        "base_prefix_sha256": base_sha,
        "base_prefix_header_hex": base_info.header.hex(),
        "originals_path": str(originals_path.resolve()),
        "replacements_path": str(replacements_path.resolve()),
        "indices_path": str(indices_path.resolve()),
        "additions_path": str(additions_path.resolve()),
        "additions_sha256": additions_hash,
        "addition_count": int(len(additions)),
        "replacement_count": 0,
        "tail_replacement": True,
        "superseded_run_dir": str(superseded_run_dir.resolve()),
        "superseded_addition_count": int(len(superseded)),
        "superseded_additions_path": str(superseded_path.resolve()),
        "superseded_additions_sha256": superseded_hash,
        "gamut_compressed_fraction": 0.0,
    }


def _write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    os.replace(temporary, path)


def _manifest_path(run_dir: Path) -> Path:
    return run_dir / "manifest.json"


def _load_manifest(run_dir: Path) -> dict[str, Any]:
    path = _manifest_path(run_dir)
    if not path.is_file():
        raise ValueError(f"Run manifest not found: {path}")
    return json.loads(path.read_text(encoding="utf-8"))


def _replace_candidate_paths(value: Any, replacements: dict[str, str]) -> Any:
    if isinstance(value, dict):
        return {key: _replace_candidate_paths(child, replacements) for key, child in value.items()}
    if isinstance(value, list):
        return [_replace_candidate_paths(child, replacements) for child in value]
    if isinstance(value, str):
        replacement = replacements.get(Path(value).name)
        return replacement if replacement is not None else value
    return value


def build_validation_data_root(
    repo_root: Path,
    density_entries: Sequence[dict[str, Any]],
    validation_data_root: Path,
) -> dict[str, str]:
    validation_scene = validation_data_root / "Scene3"
    validation_scene.mkdir(parents=True, exist_ok=False)
    candidates = {
        Path(entry["source_path"]).name: Path(entry["candidate_path"])
        for entry in density_entries
    }
    if len(candidates) != len(density_entries):
        raise ValueError("Validation candidates contain duplicate canonical filenames")
    replacements: dict[str, str] = {}
    for source in sorted((repo_root / "Data" / "Scene3").iterdir()):
        if not source.is_file() or source.name.startswith("."):
            continue
        target = validation_scene / source.name
        link_source = candidates.get(source.name, source)
        target.symlink_to(link_source.resolve())
        replacements[source.name] = str(target.absolute())
    missing = sorted(set(candidates) - set(replacements))
    if missing:
        raise ValueError(f"Could not construct validation aliases for: {', '.join(missing)}")
    return replacements


def generate_validation_project(
    project_path: Path,
    replacements: dict[str, str],
    output_path: Path,
) -> None:
    document = json.loads(project_path.read_text(encoding="utf-8"))
    document = _replace_candidate_paths(document, replacements)
    document["selected_point_visual"] = "Projector-01"
    document["live_visual_effects"] = False
    if isinstance(document.get("water_rain_settings"), dict):
        document["water_rain_settings"]["enabled"] = False
        document["water_rain_settings"]["rain_level"] = 0.0
    if isinstance(document.get("water_dynamic_mesh_flow_settings"), dict):
        document["water_dynamic_mesh_flow_settings"]["enabled"] = False
    document["water_shoreline_instances"] = []
    for layer in document.get("layers", []):
        if isinstance(layer, dict) and str(layer.get("kind", "")).lower() in {"gaussian_splat", "gsplat"}:
            layer["visible"] = False
    output_path.write_text(json.dumps(document, indent=2) + "\n", encoding="utf-8")


def _validate_schema(current_infos: Sequence[PlyInfo], dense_info: PlyInfo) -> None:
    reference = current_infos[0]
    if reference.stride != 159 or len(reference.ar_fields) != 26:
        raise ValueError(
            f"Expected a 159-byte SAND record with 26 A_R fields; found "
            f"{reference.stride} bytes and {len(reference.ar_fields)} fields"
        )
    for info in current_infos[1:]:
        if info.dtype != reference.dtype or info.property_names != reference.property_names:
            raise ValueError(f"{info.path} does not share the canonical SAND schema")
    if dense_info.ar_fields:
        raise ValueError("The dense CleanMesh patch unexpectedly contains A_R fields")
    for canonical in BASE_ALIASES:
        if canonical not in dense_info.fields:
            raise ValueError(f"The dense CleanMesh patch has no {canonical} field")


def _save_model_artifacts(
    run_dir: Path,
    components: Sequence[ComponentSupport],
    models: dict[tuple[int, int], CorrectionModel],
) -> list[dict[str, Any]]:
    arrays: dict[str, np.ndarray] = {}
    metrics: list[dict[str, Any]] = []
    for component in components:
        prefix = f"component_{component.cleanmesh_index}"
        arrays[f"{prefix}_mask"] = component.mask.astype(np.uint8)
        arrays[f"{prefix}_cell_keys"] = component.cell_keys
        arrays[f"{prefix}_cell_z_min"] = component.cell_z_min
        arrays[f"{prefix}_cell_z_max"] = component.cell_z_max
        for source_id in SOURCE_SCAN_IDS:
            model = models[(component.cleanmesh_index, source_id)]
            model_prefix = f"{prefix}_scan_{source_id}"
            arrays[f"{model_prefix}_gain"] = model.gain
            arrays[f"{model_prefix}_offset"] = model.offset
            arrays[f"{model_prefix}_residual"] = model.residual_field
            arrays[f"{model_prefix}_clip_low"] = model.clip_low
            arrays[f"{model_prefix}_clip_high"] = model.clip_high
            metrics.append(model.metrics)
    np.savez_compressed(run_dir / "correction-models.npz", **arrays)
    _write_json(run_dir / "correction-model-report.json", metrics)
    return metrics


def _load_saved_correction_model(
    run_dir: Path,
    component: ComponentSupport,
    source_id: int,
) -> CorrectionModel:
    artifact_path = run_dir / "correction-models.npz"
    if not artifact_path.is_file():
        raise ValueError(f"Correction-model artifact not found: {artifact_path}")
    prefix = f"component_{component.cleanmesh_index}_scan_{source_id}"
    with np.load(artifact_path) as arrays:
        required = {
            suffix: f"{prefix}_{suffix}"
            for suffix in ("gain", "offset", "residual", "clip_low", "clip_high")
        }
        missing = [name for name in required.values() if name not in arrays]
        if missing:
            raise ValueError(
                "Correction-model artifact is incomplete: " + ", ".join(missing)
            )
        residual = np.array(arrays[required["residual"]], copy=True)
        if residual.shape[:2] != component.mask.shape:
            raise ValueError(
                f"Saved correction grid {residual.shape[:2]} no longer matches "
                f"{component.camera_name} support {component.mask.shape}"
            )
        return CorrectionModel(
            component_index=component.cleanmesh_index,
            source_scan_id=source_id,
            host_scan_id=component.host_scan_id,
            gain=np.array(arrays[required["gain"]], copy=True),
            offset=np.array(arrays[required["offset"]], copy=True),
            residual_field=residual,
            clip_low=np.array(arrays[required["clip_low"]], copy=True),
            clip_high=np.array(arrays[required["clip_high"]], copy=True),
            pair_source=np.empty((0, 6), dtype=np.float64),
            pair_target=np.empty((0, 6), dtype=np.float64),
            metrics={},
        )


def command_build(args: argparse.Namespace) -> int:
    if cv2 is None:
        raise RuntimeError("OpenCV (cv2) is required for component masks and harmonic blending")
    repo_root = args.repo_root.resolve()
    run_dir = args.run_dir.resolve() if args.run_dir else (
        repo_root / "Data" / "Scene3" / "PatchRefinement" /
        dt.datetime.now().strftime("%Y%m%d-%H%M%S")
    )
    if run_dir.exists():
        raise ValueError(f"Run directory already exists: {run_dir}")
    run_dir.mkdir(parents=True)
    (run_dir / DISCOVERY_IGNORE_MARKER).write_text(
        "Point-cloud refinement staging; discover validation-data explicitly.\n",
        encoding="utf-8",
    )
    started = time.monotonic()
    selected_spacings = tuple(dict.fromkeys(args.spacings))
    source_paths = [repo_root / "Data" / "Scene3" / f"Site3-SAND-{spacing}mm.ply" for spacing in SPACINGS_MM]
    dense_path = args.cleanmesh_patch.resolve()
    report_path = args.cleanmesh_report.resolve()
    project_path = args.project.resolve()
    current_infos = [read_ply_info(path) for path in source_paths]
    dense_info = read_ply_info(dense_path)
    _validate_schema(current_infos, dense_info)
    print("Building CleanMesh component support masks", flush=True)
    components = build_component_supports(dense_info, report_path, args.chunk_records)
    print("Collecting 1 mm model/reference records", flush=True)
    local_1mm = collect_local_records(current_infos[0], components, args.chunk_records)
    models: dict[tuple[int, int], CorrectionModel] = {}
    print("Fitting RGB and per-scan scalar correction models", flush=True)
    for component, local in zip(components, local_1mm, strict=True):
        for source_id in SOURCE_SCAN_IDS:
            model = fit_correction_model(local, current_infos[0], component, source_id)
            models[(component.cleanmesh_index, source_id)] = model
            print(
                f"  {component.camera_name} ID {source_id}: "
                f"DeltaE {model.metrics['rgb_delta_e_median_before']:.3f} -> "
                f"{model.metrics['rgb_delta_e_median_after']:.3f} "
                f"({model.metrics['pair_count']} pairs)",
                flush=True,
            )
    model_metrics = _save_model_artifacts(run_dir, components, models)
    manifest: dict[str, Any] = {
        "schema": 1,
        "status": "building",
        "created_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "repo_root": str(repo_root),
        "run_dir": str(run_dir),
        "cleanmesh_patch": str(dense_path),
        "cleanmesh_patch_sha256": sha256_path(dense_path),
        "cleanmesh_report": str(report_path),
        "project": str(project_path),
        "scan_id_contract": {
            "source_ids": list(SOURCE_SCAN_IDS),
            "replacement_id": REPLACEMENT_SCAN_ID,
            "addition_id": ADDITION_SCAN_ID,
        },
        "components": [
            {
                "camera": component.camera_name,
                "cleanmesh_component": component.cleanmesh_index,
                "host_scan_id": component.host_scan_id,
                "bounds_min": component.bounds_min.tolist(),
                "bounds_max": component.bounds_max.tolist(),
                "dense_point_count": int(len(component.dense_records)),
                "support_cell_count": int(len(component.cell_keys)),
            }
            for component in components
        ],
        "models": model_metrics,
        "requested_spacings_mm": list(selected_spacings),
        "densities": [],
        "verified": False,
        "installed": False,
    }
    _write_json(_manifest_path(run_dir), manifest)
    density_reports: dict[str, Any] = {}
    for info, spacing_mm in zip(current_infos, SPACINGS_MM, strict=True):
        if spacing_mm not in selected_spacings:
            continue
        print(f"Preparing {spacing_mm} mm cloud", flush=True)
        local = local_1mm if spacing_mm == 1 else collect_local_records(info, components, args.chunk_records)
        additions, reports = prepare_additions_for_density(
            spacing_mm, local, local_1mm, info, current_infos[0], dense_info,
            components, models,
        )
        density_dir = run_dir / f"{spacing_mm}mm"
        entry = build_density_candidate(
            spacing_mm, info, local, additions, components, models,
            density_dir, args.chunk_records,
        )
        entry["component_reports"] = reports
        manifest["densities"].append(entry)
        density_reports[f"{spacing_mm}mm"] = reports
        _write_json(_manifest_path(run_dir), manifest)
        if spacing_mm != 1:
            del local
    validation_data_root = run_dir / "validation-data"
    validation_replacements = build_validation_data_root(
        repo_root,
        manifest["densities"],
        validation_data_root,
    )
    validation_project = run_dir / "ExhibitionFinal_patch-validation_project.json"
    generate_validation_project(project_path, validation_replacements, validation_project)
    manifest["validation_project"] = str(validation_project.resolve())
    manifest["validation_data_root"] = str(validation_data_root.resolve())
    manifest["status"] = "built"
    manifest["elapsed_seconds"] = time.monotonic() - started
    _write_json(run_dir / "density-report.json", density_reports)
    _write_json(_manifest_path(run_dir), manifest)
    print(f"Built candidates in {run_dir}", flush=True)
    print(f"Validation project: {validation_project}", flush=True)
    return 0


def command_build_augmentation(args: argparse.Namespace) -> int:
    """Build a reversible density-only descendant of an installed patch run."""
    if cv2 is None:
        raise RuntimeError("OpenCV (cv2) is required for component masks and Poisson sampling")
    repo_root = args.repo_root.resolve()
    run_dir = args.run_dir.resolve()
    base_run_dir = args.base_run_dir.resolve()
    if run_dir.exists():
        raise ValueError(f"Run directory already exists: {run_dir}")
    base_manifest = _load_manifest(base_run_dir)
    if not base_manifest.get("installed"):
        raise ValueError("Density augmentation requires an installed base run")
    base_entries = {
        int(entry["spacing_mm"]): entry
        for entry in base_manifest.get("densities", [])
    }
    if set(base_entries) != set(SPACINGS_MM):
        raise ValueError("Installed base run does not contain complete 1/2/3/5 mm entries")

    run_dir.mkdir(parents=True)
    (run_dir / DISCOVERY_IGNORE_MARKER).write_text(
        "Point-cloud density-augmentation staging; discover validation-data explicitly.\n",
        encoding="utf-8",
    )
    started = time.monotonic()
    source_paths = [
        repo_root / "Data" / "Scene3" / f"Site3-SAND-{spacing}mm.ply"
        for spacing in SPACINGS_MM
    ]
    current_infos = [read_ply_info(path) for path in source_paths]
    dense_path = Path(base_manifest["cleanmesh_patch"])
    report_path = Path(base_manifest["cleanmesh_report"])
    project_path = Path(base_manifest["project"])
    dense_info = read_ply_info(dense_path)
    _validate_schema(current_infos, dense_info)
    for spacing_mm, info in zip(SPACINGS_MM, current_infos, strict=True):
        base_entry = base_entries[spacing_mm]
        expected_hash = base_entry.get("candidate_sha256")
        if not expected_hash or sha256_path(info.path) != expected_hash:
            raise ValueError(
                f"Canonical {spacing_mm} mm cloud is not the installed base-run candidate"
            )
        if info.vertex_count != int(base_entry["candidate_vertex_count"]):
            raise ValueError(
                f"Canonical {spacing_mm} mm vertex count differs from the installed base run"
            )

    print("Building Patch 01 CleanMesh support for density augmentation", flush=True)
    component = build_component_supports(
        dense_info,
        report_path,
        args.chunk_records,
    )[0]
    print("Collecting the installed 1 mm Patch 01 neighborhood", flush=True)
    local_1mm = collect_local_records(
        current_infos[0],
        [component],
        args.chunk_records,
    )
    models = {
        (component.cleanmesh_index, 11): _load_saved_correction_model(
            base_run_dir,
            component,
            11,
        )
    }
    manifest: dict[str, Any] = {
        "schema": 1,
        "kind": "append-only-density-augmentation",
        "status": "building",
        "created_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "repo_root": str(repo_root),
        "run_dir": str(run_dir),
        "base_run_dir": str(base_run_dir),
        "cleanmesh_patch": str(dense_path),
        "cleanmesh_patch_sha256": base_manifest["cleanmesh_patch_sha256"],
        "cleanmesh_report": str(report_path),
        "project": str(project_path),
        "scan_id_contract": {
            "source_ids": list(SOURCE_SCAN_IDS),
            "replacement_id": REPLACEMENT_SCAN_ID,
            "addition_id": ADDITION_SCAN_ID,
        },
        "components": [{
            "camera": component.camera_name,
            "cleanmesh_component": component.cleanmesh_index,
            "host_scan_id": component.host_scan_id,
            "bounds_min": component.bounds_min.tolist(),
            "bounds_max": component.bounds_max.tolist(),
            "dense_point_count": int(len(component.dense_records)),
            "support_cell_count": int(len(component.cell_keys)),
        }],
        "models": [],
        "model_source_run": str(base_run_dir),
        "requested_spacings_mm": list(SPACINGS_MM),
        "densities": [],
        "verified": False,
        "installed": False,
    }
    _write_json(_manifest_path(run_dir), manifest)
    density_reports: dict[str, Any] = {}
    for info, spacing_mm in zip(current_infos, SPACINGS_MM, strict=True):
        print(f"Preparing {spacing_mm} mm Patch 01 density augmentation", flush=True)
        local = (
            local_1mm
            if spacing_mm == 1
            else collect_local_records(info, [component], args.chunk_records)
        )
        additions, reports = prepare_additions_for_density(
            spacing_mm,
            local,
            local_1mm,
            info,
            current_infos[0],
            dense_info,
            [component],
            models,
        )
        entry = build_append_only_density_candidate(
            spacing_mm,
            info,
            additions,
            run_dir / f"{spacing_mm}mm",
            args.chunk_records,
            role="SAND",
        )
        if entry["source_sha256"] != base_entries[spacing_mm]["candidate_sha256"]:
            raise ValueError(
                f"Canonical {spacing_mm} mm payload changed while augmentation was building"
            )
        entry["component_reports"] = reports
        manifest["densities"].append(entry)
        density_reports[f"{spacing_mm}mm"] = reports
        _write_json(_manifest_path(run_dir), manifest)
        if spacing_mm != 1:
            del local

    validation_data_root = run_dir / "validation-data"
    validation_replacements = build_validation_data_root(
        repo_root,
        manifest["densities"],
        validation_data_root,
    )
    validation_project = run_dir / "ExhibitionFinal_patch-validation_project.json"
    generate_validation_project(project_path, validation_replacements, validation_project)
    manifest["validation_project"] = str(validation_project.resolve())
    manifest["validation_data_root"] = str(validation_data_root.resolve())
    manifest["status"] = "built"
    manifest["elapsed_seconds"] = time.monotonic() - started
    manifest["sampling_revision"] = "patch01-total-density-2mm-projector-v1"
    _write_json(run_dir / "density-report.json", density_reports)
    _write_json(_manifest_path(run_dir), manifest)
    print(f"Built append-only Patch 01 candidates in {run_dir}", flush=True)
    print(
        "Restore this augmentation before restoring its base run; both rollback steps are byte exact.",
        flush=True,
    )
    return 0


def command_build_edge_augmentation(args: argparse.Namespace) -> int:
    """Build a staged ScanID 9 density taper for the Patch 04 rock edge."""
    if cv2 is None:
        raise RuntimeError("OpenCV (cv2) is required for Patch 04 edge augmentation")
    repo_root = args.repo_root.resolve()
    run_dir = args.run_dir.resolve()
    base_run_dir = args.base_run_dir.resolve()
    superseded_run_dir = (
        args.replace_installed_edge_run.resolve()
        if args.replace_installed_edge_run is not None
        else None
    )
    mesh_path = args.mesh_samples.resolve()
    project_path = args.project.resolve()
    if run_dir.exists():
        raise ValueError(f"Run directory already exists: {run_dir}")
    base_manifest = _load_manifest(base_run_dir)
    if not base_manifest.get("installed"):
        raise ValueError("Patch 04 augmentation requires an installed base run")
    superseded_manifest: dict[str, Any] | None = None
    superseded_entries: dict[int, dict[str, Any]] = {}
    if superseded_run_dir is not None:
        superseded_manifest = _load_manifest(superseded_run_dir)
        if not superseded_manifest.get("installed"):
            raise ValueError("The Patch 04 edge run being replaced is not installed")
        superseded_entries = {
            int(entry["spacing_mm"]): entry
            for entry in superseded_manifest.get("densities", [])
            if entry.get("role", "ROCK") == "ROCK"
        }
        missing = sorted(set(SPACINGS_MM) - set(superseded_entries))
        if missing:
            raise ValueError(
                "Installed Patch 04 edge run is missing ROCK spacings: "
                + ", ".join(str(value) for value in missing)
            )
        if any(entry.get("tail_replacement") for entry in superseded_entries.values()):
            raise ValueError(
                "Replacing an already replaced edge tail requires first installing or restoring its parent"
            )
    spacings = tuple(sorted(set(int(value) for value in args.spacings)))
    run_dir.mkdir(parents=True)
    (run_dir / DISCOVERY_IGNORE_MARKER).write_text(
        "Patch 04 ScanID 9 edge-augmentation staging; discover validation-data explicitly.\n",
        encoding="utf-8",
    )
    started = time.monotonic()
    source_paths = [
        repo_root / "Data" / "Scene3" / f"Site3-ROCK-{spacing}mm.ply"
        for spacing in spacings
    ]
    current_infos = [read_ply_info(path) for path in source_paths]
    base_infos = [
        _prefix_info_from_installed_edge(info, superseded_entries[spacing])
        if superseded_manifest is not None
        else info
        for info, spacing in zip(current_infos, spacings, strict=True)
    ]
    mesh_info = read_ply_info(
        mesh_path,
        required_fields=("x", "y", "z", "red", "green", "blue", "nx", "ny", "nz"),
    )
    mesh_bounds_min, mesh_bounds_max = geometry_bounds(mesh_info, args.chunk_records)
    bootstrap = ComponentSupport(
        camera_name="Patch 04",
        cleanmesh_index=4,
        host_scan_id=PATCH_04_SOURCE_SCAN_ID,
        bounds_min=mesh_bounds_min,
        bounds_max=mesh_bounds_max,
        dense_records=np.empty(0, dtype=mesh_info.dtype),
        cell_keys=np.empty(0, dtype=np.int64),
        cell_z_min=np.empty(0, dtype=np.float64),
        cell_z_max=np.empty(0, dtype=np.float64),
        origin_ix=0,
        origin_iy=0,
        mask=np.zeros((1, 1), dtype=bool),
        boundary=np.zeros((1, 1), dtype=bool),
        outside_ring=np.zeros((1, 1), dtype=bool),
    )
    canonical_1mm_current_info = read_ply_info(
        repo_root / "Data" / "Scene3" / "Site3-ROCK-1mm.ply"
    )
    canonical_1mm_info = (
        _prefix_info_from_installed_edge(
            canonical_1mm_current_info,
            superseded_entries[1],
        )
        if superseded_manifest is not None
        else canonical_1mm_current_info
    )
    print("Collecting the measured 1 mm Patch 04 rock neighborhood", flush=True)
    local_1mm = collect_local_records(
        canonical_1mm_info,
        [bootstrap],
        args.chunk_records,
    )[0]
    support_width = (
        args.jagged_max_width
        if args.edge_profile == "jagged"
        else args.blend_width
    )
    support = build_patch04_edge_support(
        local_1mm,
        canonical_1mm_info,
        mesh_bounds_min,
        mesh_bounds_max,
        support_width,
    )
    edge_profile_report: dict[str, Any] = {
        "profile": "linear",
        "width_m": float(args.blend_width),
    }
    if args.edge_profile == "jagged":
        edge_profile_report = configure_patch04_jagged_fade(
            support,
            local_1mm,
            canonical_1mm_info,
            minimum_width_metres=args.jagged_min_width,
            base_width_metres=args.jagged_base_width,
            maximum_width_metres=args.jagged_max_width,
            core_width_metres=args.jagged_core_width,
            noise_seed=args.noise_seed,
        )
    print(
        "Patch 04 edge fit: "
        f"origin={support.edge_origin_xy.tolist()}, "
        f"direction={support.edge_direction_xy.tolist()}, "
        f"RMS={support.fit_rms_metres * 1000.0:.2f} mm, "
        f"profile={edge_profile_report['profile']}",
        flush=True,
    )
    mesh_belt = collect_patch04_mesh_belt(mesh_info, support, args.chunk_records)
    supported_mesh, geometry_report = filter_patch04_mesh_geometry(
        mesh_belt,
        mesh_info,
        local_1mm,
        canonical_1mm_info,
    )
    cell_keys, cell_z_min, cell_z_max = _surface_cell_ranges(supported_mesh, mesh_info)
    support.component.dense_records = supported_mesh
    support.component.cell_keys = cell_keys
    support.component.cell_z_min = cell_z_min
    support.component.cell_z_max = cell_z_max

    manifest: dict[str, Any] = {
        "schema": 1,
        "kind": (
            "replace-scan9-jagged-edge-augmentation"
            if superseded_manifest is not None
            else f"append-only-scan9-{args.edge_profile}-edge-augmentation"
        ),
        "status": "building",
        "created_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "repo_root": str(repo_root),
        "run_dir": str(run_dir),
        "base_run_dir": str(base_run_dir),
        "superseded_edge_run_dir": (
            str(superseded_run_dir) if superseded_run_dir is not None else None
        ),
        "mesh_samples": str(mesh_path),
        "mesh_samples_sha256": sha256_path(mesh_path),
        "project": str(project_path),
        "scan_id_contract": {
            "source_ids": [PATCH_04_SOURCE_SCAN_ID],
            "preserved_existing_ids": list(range(0, 10)),
            "replacement_id": REPLACEMENT_SCAN_ID,
            "addition_id": ADDITION_SCAN_ID,
            "existing_records_modified": 0,
            "superseded_addition_tail_replaced": superseded_manifest is not None,
        },
        "components": [{
            "camera": "Patch 04",
            "role": "ROCK",
            "source_scan_id": PATCH_04_SOURCE_SCAN_ID,
            "bounds_min": mesh_bounds_min.tolist(),
            "bounds_max": mesh_bounds_max.tolist(),
            "edge_origin_xy": support.edge_origin_xy.tolist(),
            "edge_direction_xy": support.edge_direction_xy.tolist(),
            "outside_normal_xy": support.outside_normal_xy.tolist(),
            "edge_fit_rms_m": support.fit_rms_metres,
            "blend_width_m": support.blend_width_metres,
            "edge_profile": edge_profile_report,
            "transition_cell_count": int(np.count_nonzero(support.transition_mask)),
            "supported_mesh_point_count": int(len(supported_mesh)),
            "geometry_filter": geometry_report,
        }],
        "models": [],
        "requested_spacings_mm": list(spacings),
        "densities": [],
        "verified": False,
        "installed": False,
    }
    _write_json(_manifest_path(run_dir), manifest)
    density_reports: dict[str, Any] = {}
    for current_info, info, spacing_mm in zip(
        current_infos,
        base_infos,
        spacings,
        strict=True,
    ):
        print(f"Preparing ROCK {spacing_mm} mm Patch 04 edge augmentation", flush=True)
        local = (
            local_1mm
            if spacing_mm == 1
            else collect_local_records(
                info,
                [support.component],
                args.chunk_records,
            )[0]
        )
        requested, density_report = plan_patch04_edge_additions(
            local,
            info,
            support,
        )
        selected_mesh, selection_report = select_dense_additions(
            supported_mesh,
            mesh_info,
            local,
            info,
            support.component,
            requested,
            spacing_mm / 1000.0,
        )
        converted = _copy_dense_to_output_schema(selected_mesh, mesh_info, info)
        converted, transfer_report = transfer_full_nearest_1mm(
            converted,
            info,
            local_1mm,
            canonical_1mm_info,
        )
        component_report = {
            "camera": "Patch 04",
            "component": 4,
            "role": "ROCK",
            "host_scan_id": PATCH_04_SOURCE_SCAN_ID,
            "density": density_report,
            "selection": selection_report,
            "scalar_transfer": transfer_report,
            "accepted_additions": int(len(converted)),
            "existing_records_modified": 0,
        }
        if superseded_manifest is not None:
            entry = build_tail_replacement_candidate(
                spacing_mm,
                current_info,
                info,
                superseded_entries[spacing_mm],
                converted,
                run_dir / f"ROCK-{spacing_mm}mm",
                args.chunk_records,
                superseded_run_dir,
            )
        else:
            entry = build_append_only_density_candidate(
                spacing_mm,
                info,
                converted,
                run_dir / f"ROCK-{spacing_mm}mm",
                args.chunk_records,
                role="ROCK",
            )
        entry["role"] = "ROCK"
        entry["component_reports"] = [component_report]
        manifest["densities"].append(entry)
        density_reports[f"ROCK-{spacing_mm}mm"] = [component_report]
        _write_json(_manifest_path(run_dir), manifest)
        print(
            f"    Patch 04: planned {density_report['planned_additions']:,}, "
            f"selected {selection_report['selected']:,}, accepted {len(converted):,}",
            flush=True,
        )
        if spacing_mm != 1:
            del local

    validation_data_root = run_dir / "validation-data"
    validation_replacements = build_validation_data_root(
        repo_root,
        manifest["densities"],
        validation_data_root,
    )
    validation_project = run_dir / "ExhibitionFinal_patch-validation_project.json"
    generate_validation_project(project_path, validation_replacements, validation_project)
    manifest["validation_project"] = str(validation_project.resolve())
    manifest["validation_data_root"] = str(validation_data_root.resolve())
    manifest["status"] = "built"
    manifest["elapsed_seconds"] = time.monotonic() - started
    manifest["sampling_revision"] = (
        "patch04-scan9-connected-jagged-density-taper-v2"
        if args.edge_profile == "jagged"
        else "patch04-scan9-linear-density-taper-v1"
    )
    _write_json(run_dir / "density-report.json", density_reports)
    _write_json(_manifest_path(run_dir), manifest)
    print(f"Built staged Patch 04 candidates in {run_dir}", flush=True)
    return 0


def _replace_candidate_additions(
    entry: dict[str, Any],
    source_info: PlyInfo,
    additions: np.ndarray,
) -> None:
    candidate_path = Path(entry["candidate_path"])
    additions_path = Path(entry["additions_path"])
    candidate_info = read_ply_info(candidate_path)
    if candidate_info.dtype != source_info.dtype:
        raise ValueError(f"Candidate schema changed before refresh: {candidate_path}")
    if candidate_info.vertex_count != source_info.vertex_count + int(entry["addition_count"]):
        raise ValueError(f"Candidate count changed before refresh: {candidate_path}")
    expected_size = (
        candidate_info.header_size
        + candidate_info.vertex_count * candidate_info.stride
    )
    if candidate_path.stat().st_size != expected_size:
        raise ValueError(f"Candidate payload is truncated before refresh: {candidate_path}")

    temporary_additions = additions_path.with_name(
        f".{additions_path.name}.refresh.tmp"
    )
    if temporary_additions.exists():
        raise ValueError(f"Stale refresh output exists: {temporary_additions}")
    additions_sha256 = write_subset_ply(temporary_additions, source_info, additions)
    new_header = patch_header_count(
        source_info.header,
        source_info.vertex_count + len(additions),
    )
    if len(new_header) != candidate_info.header_size:
        temporary_additions.unlink()
        raise ValueError("Candidate refresh would change the fixed PLY header width")
    source_payload_end = (
        candidate_info.header_size
        + source_info.vertex_count * source_info.stride
    )
    try:
        with candidate_path.open("r+b") as stream:
            stream.truncate(source_payload_end)
            stream.seek(source_payload_end)
            stream.write(additions.tobytes(order="C"))
            stream.seek(0)
            stream.write(new_header)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_additions, additions_path)
    finally:
        if temporary_additions.exists():
            temporary_additions.unlink()

    entry["addition_count"] = int(len(additions))
    entry["candidate_vertex_count"] = int(source_info.vertex_count + len(additions))
    entry["additions_sha256"] = additions_sha256
    entry["candidate_sha256"] = sha256_path(candidate_path)


def command_refresh_additions(args: argparse.Namespace) -> int:
    """Refresh only appended additions while retaining corrected source payloads.

    This is intentionally narrower than ``build``: the indexed originals,
    corrected ScanID-12 records, and first ``source_vertex_count`` candidate
    records must already exist and remain unchanged.
    """
    if cv2 is None:
        raise RuntimeError("OpenCV (cv2) is required for Poisson sampling")
    run_dir = args.run_dir.resolve()
    manifest = _load_manifest(run_dir)
    if manifest.get("installed"):
        raise ValueError("Cannot refresh additions in an installed run")
    spacings = {int(entry["spacing_mm"]) for entry in manifest["densities"]}
    if spacings != set(SPACINGS_MM):
        raise ValueError("Addition refresh requires a complete 1/2/3/5 mm run")
    manifest["status"] = "refreshing-additions"
    manifest["verified"] = False
    manifest.pop("verification_report", None)
    _write_json(_manifest_path(run_dir), manifest)

    infos_by_spacing = {
        int(entry["spacing_mm"]): read_ply_info(Path(entry["source_path"]))
        for entry in manifest["densities"]
    }
    current_infos = [infos_by_spacing[spacing] for spacing in SPACINGS_MM]
    dense_info = read_ply_info(Path(manifest["cleanmesh_patch"]))
    _validate_schema(current_infos, dense_info)
    components = build_component_supports(
        dense_info,
        Path(manifest["cleanmesh_report"]),
        args.chunk_records,
    )
    local_1mm = collect_local_records(
        infos_by_spacing[1],
        components,
        args.chunk_records,
    )
    models: dict[tuple[int, int], CorrectionModel] = {}
    for component, local in zip(components, local_1mm, strict=True):
        for source_id in SOURCE_SCAN_IDS:
            models[(component.cleanmesh_index, source_id)] = fit_correction_model(
                local,
                infos_by_spacing[1],
                component,
                source_id,
            )
    manifest["models"] = _save_model_artifacts(run_dir, components, models)

    density_reports: dict[str, Any] = {}
    for entry in sorted(manifest["densities"], key=lambda value: int(value["spacing_mm"])):
        spacing_mm = int(entry["spacing_mm"])
        info = infos_by_spacing[spacing_mm]
        print(f"Refreshing {spacing_mm} mm additions", flush=True)
        local = (
            local_1mm
            if spacing_mm == 1
            else collect_local_records(info, components, args.chunk_records)
        )
        additions, reports = prepare_additions_for_density(
            spacing_mm,
            local,
            local_1mm,
            info,
            infos_by_spacing[1],
            dense_info,
            components,
            models,
        )
        _replace_candidate_additions(entry, info, additions)
        entry["component_reports"] = reports
        density_reports[f"{spacing_mm}mm"] = reports
        _write_json(_manifest_path(run_dir), manifest)
        if spacing_mm != 1:
            del local
    _write_json(run_dir / "density-report.json", density_reports)
    manifest["status"] = "built"
    manifest["refreshed_utc"] = dt.datetime.now(dt.timezone.utc).isoformat()
    manifest["sampling_revision"] = "source-clearance-poisson-v1"
    _write_json(_manifest_path(run_dir), manifest)
    print(f"Refreshed Poisson additions in {run_dir}", flush=True)
    return 0


def _property_byte_mask(info: PlyInfo, allowed_fields: Iterable[str]) -> np.ndarray:
    allowed = np.zeros(info.stride, dtype=bool)
    for canonical in allowed_fields:
        name = info.fields[canonical]
        field_dtype, offset = info.dtype.fields[name][:2]
        allowed[offset:offset + field_dtype.itemsize] = True
    return allowed


def verify_tail_replacement_entry(
    entry: dict[str, Any],
    chunk_records: int,
) -> dict[str, Any]:
    source_info = read_ply_info(Path(entry["source_path"]))
    candidate_info = read_ply_info(Path(entry["candidate_path"]))
    additions_info = read_ply_info(Path(entry["additions_path"]))
    superseded_info = read_ply_info(Path(entry["superseded_additions_path"]))
    originals_info = read_ply_info(Path(entry["originals_path"]))
    replacements_info = read_ply_info(Path(entry["replacements_path"]))
    indices = _read_indices(Path(entry["indices_path"]))
    failures: list[str] = []
    base_count = int(entry["candidate_base_vertex_count"])
    if source_info.dtype != candidate_info.dtype or source_info.property_names != candidate_info.property_names:
        failures.append("tail-replacement candidate schema differs from source")
    if source_info.vertex_count != int(entry["source_vertex_count"]):
        failures.append("tail-replacement source vertex count changed")
    if candidate_info.vertex_count != base_count + int(entry["addition_count"]):
        failures.append("tail-replacement candidate vertex count is inconsistent")
    if source_info.vertex_count != base_count + int(entry["superseded_addition_count"]):
        failures.append("superseded source tail count is inconsistent")
    if (
        len(indices)
        or originals_info.vertex_count
        or replacements_info.vertex_count
        or int(entry["replacement_count"])
    ):
        failures.append("tail replacement unexpectedly contains edited-record rollback data")

    expected_source_header = bytes.fromhex(entry["source_header_hex"])
    expected_base_header = bytes.fromhex(entry["base_prefix_header_hex"])
    if source_info.header != expected_source_header:
        failures.append("tail-replacement source header changed")
    expected_candidate_header = patch_header_count(
        expected_base_header,
        candidate_info.vertex_count,
    )
    if candidate_info.header != expected_candidate_header:
        failures.append("tail-replacement candidate header is not based on the preserved prefix")

    source = records(source_info)
    candidate = records(candidate_info)
    additions = records(additions_info)
    superseded = records(superseded_info)
    source_digest = hashlib.sha256(source_info.header)
    candidate_digest = hashlib.sha256(candidate_info.header)
    base_digest = hashlib.sha256(expected_base_header)
    restored_digest = hashlib.sha256(source_info.header)
    unchanged_records = 0
    for start in range(0, base_count, chunk_records):
        end = min(start + chunk_records, base_count)
        source_chunk = np.array(source[start:end], copy=True)
        candidate_chunk = np.array(candidate[start:end], copy=True)
        source_bytes = source_chunk.tobytes(order="C")
        candidate_bytes = candidate_chunk.tobytes(order="C")
        if source_bytes != candidate_bytes:
            failures.append(f"base-prefix record bytes changed in [{start},{end})")
            break
        source_digest.update(source_bytes)
        candidate_digest.update(candidate_bytes)
        base_digest.update(source_bytes)
        restored_digest.update(candidate_bytes)
        unchanged_records += end - start

    source_tail = np.array(source[base_count:], copy=True)
    rollback_tail = np.array(superseded, copy=True)
    if source_tail.tobytes(order="C") != rollback_tail.tobytes(order="C"):
        failures.append("superseded tail backup differs from the installed source tail")
    source_digest.update(source_tail.tobytes(order="C"))
    restored_digest.update(rollback_tail.tobytes(order="C"))
    candidate_tail = np.array(candidate[base_count:], copy=True)
    addition_records = np.array(additions, copy=True)
    if candidate_tail.tobytes(order="C") != addition_records.tobytes(order="C"):
        failures.append("new candidate tail differs from the staged additions cloud")
    candidate_digest.update(candidate_tail.tobytes(order="C"))
    for label, values, info in (
        ("superseded", source_tail, source_info),
        ("new", candidate_tail, candidate_info),
    ):
        if len(values):
            scan_ids, valid = _scan_ids(values, info)
            if not np.all(valid & (scan_ids == ADDITION_SCAN_ID)):
                failures.append(f"{label} edge tail contains a record other than ScanID 13")
    del source, candidate, additions, superseded

    if source_digest.hexdigest() != entry["source_sha256"]:
        failures.append("installed source SHA-256 changed since the replacement build")
    if candidate_digest.hexdigest() != entry["candidate_sha256"]:
        failures.append("tail-replacement candidate SHA-256 does not match its manifest")
    if base_digest.hexdigest() != entry["base_prefix_sha256"]:
        failures.append("preserved pre-edge prefix SHA-256 does not match its manifest")
    if restored_digest.hexdigest() != entry["source_sha256"]:
        failures.append("tail-replacement restore dry run did not reproduce the installed source")
    if sha256_path(additions_info.path) != entry["additions_sha256"]:
        failures.append("staged additions subset SHA-256 changed")
    if sha256_path(superseded_info.path) != entry["superseded_additions_sha256"]:
        failures.append("superseded additions backup SHA-256 changed")
    return {
        "spacing_mm": entry["spacing_mm"],
        "passed": not failures,
        "failures": failures,
        "replacement_count": 0,
        "addition_count": int(len(candidate_tail)),
        "superseded_addition_count": int(len(source_tail)),
        "unchanged_record_count": unchanged_records,
        "restore_sha256": restored_digest.hexdigest(),
    }


def verify_density_entry(entry: dict[str, Any], chunk_records: int) -> dict[str, Any]:
    if entry.get("tail_replacement"):
        return verify_tail_replacement_entry(entry, chunk_records)
    source_info = read_ply_info(Path(entry["source_path"]))
    candidate_info = read_ply_info(Path(entry["candidate_path"]))
    originals_info = read_ply_info(Path(entry["originals_path"]))
    replacements_info = read_ply_info(Path(entry["replacements_path"]))
    additions_info = read_ply_info(Path(entry["additions_path"]))
    indices = _read_indices(Path(entry["indices_path"]))
    failures: list[str] = []
    if candidate_info.dtype != source_info.dtype or candidate_info.property_names != source_info.property_names:
        failures.append("candidate schema differs from source")
    if candidate_info.vertex_count != source_info.vertex_count + entry["addition_count"]:
        failures.append("candidate vertex count is inconsistent")
    if len(indices) != entry["replacement_count"] or originals_info.vertex_count != len(indices) or replacements_info.vertex_count != len(indices):
        failures.append("replacement backup/index counts disagree")
    if len(indices) and (np.any(indices[1:] <= indices[:-1]) or indices[-1] >= source_info.vertex_count):
        failures.append("replacement indices are not strictly increasing and in range")
    source = records(source_info)
    candidate = records(candidate_info)
    original_backup = records(originals_info)
    replacement_backup = records(replacements_info)
    allowed_bytes = _property_byte_mask(source_info, EDITABLE_FIELDS)
    backup_cursor = 0
    unchanged_records = 0
    restored_digest = hashlib.sha256(source_info.header)
    for start in range(0, source_info.vertex_count, chunk_records):
        end = min(start + chunk_records, source_info.vertex_count)
        source_chunk = np.array(source[start:end], copy=True)
        candidate_chunk = np.array(candidate[start:end], copy=True)
        left = int(np.searchsorted(indices, start, side="left"))
        right = int(np.searchsorted(indices, end, side="left"))
        local_indices = (indices[left:right] - np.uint64(start)).astype(np.int64)
        selected = np.zeros(end - start, dtype=bool)
        selected[local_indices] = True
        source_bytes = source_chunk.view(np.uint8).reshape(-1, source_info.stride)
        candidate_bytes = candidate_chunk.view(np.uint8).reshape(-1, source_info.stride)
        if np.any(source_bytes[~selected] != candidate_bytes[~selected]):
            failures.append(f"non-selected record bytes changed in [{start},{end})")
            break
        unchanged_records += int(np.count_nonzero(~selected))
        if len(local_indices):
            backups = np.array(original_backup[left:right], copy=True)
            replacements = np.array(replacement_backup[left:right], copy=True)
            if np.any(backups.view(np.uint8).reshape(-1, source_info.stride) != source_bytes[local_indices]):
                failures.append(f"original backups mismatch source in [{start},{end})")
                break
            if np.any(replacements.view(np.uint8).reshape(-1, source_info.stride) != candidate_bytes[local_indices]):
                failures.append(f"replacement backups mismatch candidate in [{start},{end})")
                break
            changed_disallowed = (source_bytes[local_indices] != candidate_bytes[local_indices]) & ~allowed_bytes[None, :]
            if np.any(changed_disallowed):
                failures.append(f"a selected record changed a protected byte in [{start},{end})")
                break
            replacement_ids, valid = _scan_ids(candidate_chunk[local_indices], candidate_info)
            if not np.all(valid & (replacement_ids == REPLACEMENT_SCAN_ID)):
                failures.append(f"a selected record is not ScanID 12 in [{start},{end})")
                break
            candidate_chunk[local_indices] = backups
            backup_cursor += len(local_indices)
        restored_digest.update(candidate_chunk.tobytes(order="C"))
    if backup_cursor != len(indices):
        failures.append("not every backup record was consumed")
    appended = np.array(candidate[source_info.vertex_count:], copy=True)
    addition_records = np.array(records(additions_info), copy=True)
    appended_bytes = appended.view(np.uint8).reshape(-1, candidate_info.stride)
    addition_bytes = addition_records.view(np.uint8).reshape(-1, candidate_info.stride)
    if len(appended) != additions_info.vertex_count or np.any(appended_bytes != addition_bytes):
        failures.append("appended candidate records differ from the additions cloud")
    if len(appended):
        addition_ids, valid = _scan_ids(appended, candidate_info)
        if not np.all(valid & (addition_ids == ADDITION_SCAN_ID)):
            failures.append("an appended record is not ScanID 13")
    del source, candidate, original_backup, replacement_backup
    if sha256_path(source_info.path) != entry["source_sha256"]:
        failures.append("source SHA-256 changed since build")
    if sha256_path(candidate_info.path) != entry["candidate_sha256"]:
        failures.append("candidate SHA-256 does not match its manifest")
    if restored_digest.hexdigest() != entry["source_sha256"]:
        failures.append("restore dry run did not reproduce the original SHA-256")
    return {
        "spacing_mm": entry["spacing_mm"],
        "passed": not failures,
        "failures": failures,
        "replacement_count": int(len(indices)),
        "addition_count": int(len(appended)),
        "unchanged_record_count": unchanged_records,
        "restore_sha256": restored_digest.hexdigest(),
    }


def verify_component_additions(component: dict[str, Any]) -> tuple[dict[str, Any], list[str], list[str]]:
    density = component["density"]
    selection = component["selection"]
    transfer = component.get("scalar_transfer", component.get("ar_transfer"))
    if not isinstance(transfer, dict):
        raise ValueError(f"{component.get('camera', 'patch')} has no scalar-transfer report")
    camera = component["camera"]
    planned = int(density["planned_additions"])
    selected = int(selection["selected"])
    accepted = int(component["accepted_additions"])
    transferred = int(transfer["transferred"])
    skipped = int(transfer["skipped"])
    failures: list[str] = []
    warnings: list[str] = []
    if int(selection["requested"]) != planned:
        failures.append(f"{camera}: selection request does not match the density plan")
    if selected > planned:
        failures.append(f"{camera}: selected more additions than planned")
    edge_transfer = "scalar_transfer" in component
    if accepted != selected and not edge_transfer:
        failures.append(f"{camera}: accepted {accepted} of {selected} selected additions")
    transfer_consistent = (
        int(transfer["requested"]) == selected
        and transferred == accepted
        and skipped == selected - accepted
    )
    if not transfer_consistent or (skipped and not edge_transfer):
        failures.append(
            f"{camera}: scalar transfer requested/accepted/skipped counts are inconsistent "
            f"({transfer['requested']}/{transferred}/{skipped})"
        )
    elif skipped:
        warnings.append(
            f"{camera}: omitted {skipped} selected mesh samples whose approximate nearest "
            "1 mm scalar source exceeded the transfer limit"
        )
    if planned and not accepted:
        failures.append(f"{camera}: the density plan produced no usable additions")
    if int(component.get("existing_records_modified", 0)) != 0:
        failures.append(f"{camera}: an append-only density pass reports modified existing records")

    shortfall = planned - accepted
    if shortfall:
        warnings.append(
            f"{camera}: {shortfall} of {planned} requested additions were blocked by source-clearance "
            "or Poisson-spacing constraints; the camera render is the residual-deficit gate"
        )
    current_points = int(density["current_points"])
    interior_mean = float(density["interior_mean_points_per_cell"])
    interior_cells = int(round(current_points / interior_mean)) if interior_mean > 0.0 else 0
    actual_mean = (
        float(current_points + accepted) / interior_cells
        if interior_cells
        else interior_mean
    )
    result = {
        "camera": camera,
        "planned_additions": planned,
        "selected_additions": selected,
        "accepted_additions": accepted,
        "spacing_limited_shortfall": shortfall,
        "fill_fraction": float(accepted / planned) if planned else 1.0,
        "actual_projected_interior_mean_points_per_cell": actual_mean,
        "ring_mean_points_per_cell": float(density["ring_mean_points_per_cell"]),
        "scalar_transfer_max_distance_m": transfer.get("max_distance_m"),
        "passed": not failures,
    }
    if density.get("density_target") in {
        "scan9-linear-edge-ramp",
        "scan9-jagged-edge-ramp",
    }:
        result.update({
            "density_target": density["density_target"],
            "source_scan_id": int(density["source_scan_id"]),
            "blend_width_m": float(density["blend_width_m"]),
            "density_step_before_points_per_cell": float(
                density["density_step_before_points_per_cell"]
            ),
            "existing_records_modified": int(
                component.get("existing_records_modified", 0)
            ),
            "copied_scalar_field_count": int(
                transfer.get("copied_scalar_field_count", 0)
            ),
            "rgb_source": transfer.get("rgb_source", ""),
        })
        if int(density["source_scan_id"]) != PATCH_04_SOURCE_SCAN_ID:
            failures.append(f"{camera}: edge taper is not derived from ScanID 9")
        if float(density["blend_width_m"]) <= COMPONENT_CELL_METRES:
            failures.append(f"{camera}: edge taper is narrower than one planning cell")
        if density["density_target"] == "scan9-jagged-edge-ramp":
            minimum = float(density.get("fade_width_min_m", 0.0))
            maximum = float(density.get("fade_width_max_m", 0.0))
            if minimum <= COMPONENT_CELL_METRES or maximum <= minimum + COMPONENT_CELL_METRES:
                failures.append(f"{camera}: jagged edge has no material fade-reach variation")
            if int(density.get("noise_seed", 0)) == 0:
                failures.append(f"{camera}: jagged edge has no deterministic noise seed")
            result.update({
                "fade_width_min_m": minimum,
                "fade_width_p50_m": float(density.get("fade_width_p50_m", 0.0)),
                "fade_width_max_m": maximum,
                "noise_seed": int(density.get("noise_seed", 0)),
            })
        if int(transfer.get("copied_scalar_field_count", 0)) <= 0:
            failures.append(f"{camera}: no scalar fields were transferred from the 1 mm cloud")
        if "Oklab" not in str(transfer.get("rgb_source", "")):
            failures.append(f"{camera}: RGB was not blended in measured Oklab space")
        result["passed"] = not failures
    return result, failures, warnings


def command_verify(args: argparse.Namespace) -> int:
    run_dir = args.run_dir.resolve()
    manifest = _load_manifest(run_dir)
    failures: list[str] = []
    model_results: list[dict[str, Any]] = []
    for model in manifest.get("models", []):
        passed = (
            model["rgb_delta_e_median_after"] < model["rgb_delta_e_median_before"]
            and model["rgb_delta_e_p95_after"] <= model["rgb_delta_e_p95_before"]
        )
        for scalar in model.get("scalar_boundaries", {}).values():
            before = scalar.get("normalized_median_before")
            after = scalar.get("normalized_median_after")
            if before is not None and after is not None:
                passed &= after <= before
        model_results.append({"camera": model["camera"], "source_scan_id": model["source_scan_id"], "passed": bool(passed)})
        if not passed:
            failures.append(f"{model['camera']} ScanID {model['source_scan_id']} boundary metrics did not improve")
    warnings: list[str] = []
    density_results: list[dict[str, Any]] = []
    for entry in manifest["densities"]:
        print(f"Verifying {entry['spacing_mm']} mm candidate", flush=True)
        result = verify_density_entry(entry, args.chunk_records)
        component_results: list[dict[str, Any]] = []
        density_results.append(result)
        failures.extend(f"{entry['spacing_mm']} mm: {failure}" for failure in result["failures"])
        for component in entry.get("component_reports", []):
            component_result, component_failures, component_warnings = verify_component_additions(component)
            component_results.append(component_result)
            failures.extend(f"{entry['spacing_mm']} mm: {failure}" for failure in component_failures)
            warnings.extend(f"{entry['spacing_mm']} mm: {warning}" for warning in component_warnings)
        if sum(item["accepted_additions"] for item in component_results) != int(entry["addition_count"]):
            failures.append(f"{entry['spacing_mm']} mm: component additions do not sum to the candidate total")
        result["components"] = component_results
    report = {
        "passed": not failures,
        "failures": failures,
        "warnings": warnings,
        "models": model_results,
        "densities": density_results,
        "verified_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
    }
    _write_json(run_dir / "verification-report.json", report)
    manifest["verified"] = report["passed"]
    manifest["verification_report"] = str((run_dir / "verification-report.json").resolve())
    manifest["status"] = "verified" if report["passed"] else "verification-failed"
    _write_json(_manifest_path(run_dir), manifest)
    if failures:
        for failure in failures:
            print(f"FAIL: {failure}", file=sys.stderr)
        return 1
    for warning in warnings:
        print(f"WARN: {warning}", file=sys.stderr)
    print("All byte-preservation, ScanID, density, boundary, and restore checks passed.", flush=True)
    return 0


def _read_smoke_report(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise ValueError(f"GUI-smoke report not found: {path}")
    report = json.loads(path.read_text(encoding="utf-8"))
    if report.get("scenario") != "scene3-patch-boundaries" or report.get("passed") is not True:
        raise ValueError(f"GUI-smoke report did not pass: {path}")
    return report


def _application_running() -> bool:
    try:
        result = subprocess.run(
            ["pgrep", "-x", "invisible_places"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
        return result.returncode == 0
    except FileNotFoundError:
        return False


def _rollback_install(entries: Sequence[dict[str, Any]], journal: dict[str, Any]) -> None:
    for state in reversed(journal.get("files", [])):
        canonical = Path(state["canonical"])
        candidate = Path(state["candidate"])
        temporary = Path(state["temporary_original"])
        if temporary.exists():
            if canonical.exists() and not candidate.exists():
                os.replace(canonical, candidate)
            if not canonical.exists():
                os.replace(temporary, canonical)
    journal["state"] = "rolled-back"


def _recover_interrupted_install(run_dir: Path, manifest: dict[str, Any]) -> None:
    journal_path = run_dir / "install-journal.json"
    if not journal_path.exists():
        return
    journal = json.loads(journal_path.read_text(encoding="utf-8"))
    if journal.get("state") not in {"swapping", "smoke-failed"}:
        return
    _rollback_install(manifest["densities"], journal)
    _write_json(journal_path, journal)
    raise ValueError("Recovered an interrupted installation by restoring every canonical source; rerun install")


def _run_patch_smoke(
    app_path: Path,
    repo_root: Path,
    project_path: Path,
    output_dir: Path,
) -> Path:
    if not app_path.is_file():
        raise ValueError(f"Invisible Places executable not found: {app_path}")
    output_dir.mkdir(parents=True, exist_ok=True)
    command = [
        str(app_path),
        str(repo_root / "Data"),
        "--gui-smoke", "scene3-patch-boundaries",
        "--smoke-project", str(project_path),
        "--smoke-output", str(output_dir),
    ]
    print("Running post-install patch-boundary GUI smoke", flush=True)
    result = subprocess.run(command, cwd=repo_root, check=False)
    report_path = output_dir / "scene3-patch-boundaries.json"
    if result.returncode != 0:
        raise ValueError(f"GUI smoke exited with code {result.returncode}; see {report_path}")
    _read_smoke_report(report_path)
    return report_path


def command_install(args: argparse.Namespace) -> int:
    run_dir = args.run_dir.resolve()
    manifest = _load_manifest(run_dir)
    _recover_interrupted_install(run_dir, manifest)
    if not manifest.get("verified"):
        raise ValueError("The run has not passed verify")
    if manifest.get("installed"):
        raise ValueError("This run is already installed")
    installed_spacings = {int(entry["spacing_mm"]) for entry in manifest["densities"]}
    if installed_spacings != set(SPACINGS_MM):
        raise ValueError(
            "Installation requires complete 1/2/3/5 mm candidates; "
            f"this preview run contains {sorted(installed_spacings)}"
        )
    candidate_smoke_report = (
        args.candidate_smoke_report.resolve()
        if args.candidate_smoke_report
        else run_dir / "validation-render" / "scene3-patch-boundaries.json"
    )
    _read_smoke_report(candidate_smoke_report)
    if _application_running():
        raise ValueError("Invisible Places is running; close it before installation")
    for entry in manifest["densities"]:
        canonical = Path(entry["source_path"])
        candidate = Path(entry["candidate_path"])
        if sha256_path(canonical) != entry["source_sha256"]:
            raise ValueError(f"Canonical source changed since build: {canonical}")
        if sha256_path(candidate) != entry["candidate_sha256"]:
            raise ValueError(f"Candidate changed since verify: {candidate}")
    run_id = run_dir.name
    journal_path = run_dir / "install-journal.json"
    journal: dict[str, Any] = {
        "state": "swapping",
        "started_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "files": [],
    }
    _write_json(journal_path, journal)
    try:
        for entry in manifest["densities"]:
            canonical = Path(entry["source_path"])
            candidate = Path(entry["candidate_path"])
            temporary = canonical.with_name(f".{canonical.name}.prepatch-{run_id}")
            if temporary.exists():
                raise ValueError(f"Temporary rollback file already exists: {temporary}")
            os.replace(canonical, temporary)
            state = {
                "canonical": str(canonical),
                "candidate": str(candidate),
                "temporary_original": str(temporary),
            }
            journal["files"].append(state)
            _write_json(journal_path, journal)
            os.replace(candidate, canonical)
        journal["state"] = "post-install-smoke"
        _write_json(journal_path, journal)
        post_report = _run_patch_smoke(
            args.app.resolve(),
            Path(manifest["repo_root"]),
            Path(manifest["project"]),
            run_dir / "post-install-render",
        )
        for state in journal["files"]:
            temporary = Path(state["temporary_original"])
            temporary.unlink()
        journal["state"] = "complete"
        journal["completed_utc"] = dt.datetime.now(dt.timezone.utc).isoformat()
        _write_json(journal_path, journal)
        manifest["installed"] = True
        manifest["status"] = "installed"
        manifest["candidate_smoke_report"] = str(candidate_smoke_report)
        manifest["post_install_smoke_report"] = str(post_report)
        manifest["installed_utc"] = journal["completed_utc"]
        _write_json(_manifest_path(run_dir), manifest)
    except Exception:
        journal["state"] = "smoke-failed"
        _rollback_install(manifest["densities"], journal)
        _write_json(journal_path, journal)
        raise
    installed_names = ", ".join(Path(entry["source_path"]).name for entry in manifest["densities"])
    print(
        f"Installed all four canonical patch clouds ({installed_names}); "
        "compact rollback artifacts remain in the run directory."
    )
    return 0


def _write_without_additions(entry: dict[str, Any], output_path: Path) -> str:
    canonical_info = read_ply_info(Path(entry["source_path"]))
    original_count = int(
        entry.get("candidate_base_vertex_count", entry["source_vertex_count"])
    )
    if canonical_info.vertex_count < original_count:
        raise ValueError(f"Installed cloud is shorter than its original record section: {canonical_info.path}")
    original_header = bytes.fromhex(
        entry.get("base_prefix_header_hex", entry["source_header_hex"])
    )
    header = patch_header_count(original_header, original_count)
    digest = hashlib.sha256(header)
    source = records(canonical_info)
    with output_path.open("wb") as stream:
        stream.write(header)
        for start in range(0, original_count, 1_000_000):
            end = min(start + 1_000_000, original_count)
            payload = np.array(source[start:end], copy=True).tobytes(order="C")
            stream.write(payload)
            digest.update(payload)
    del source
    return digest.hexdigest()


def command_remove_additions(args: argparse.Namespace) -> int:
    run_dir = args.run_dir.resolve()
    manifest = _load_manifest(run_dir)
    if not manifest.get("installed"):
        raise ValueError("The run is not installed")
    if _application_running():
        raise ValueError("Invisible Places is running; close it before changing canonical PLY files")
    for entry in manifest["densities"]:
        canonical = Path(entry["source_path"])
        current_hash = sha256_path(canonical)
        allowed = {entry["candidate_sha256"], entry.get("without_additions_sha256")}
        if current_hash not in allowed:
            raise ValueError(f"Canonical cloud is not in an installed state recognised by the manifest: {canonical}")
        if current_hash == entry.get("without_additions_sha256"):
            continue
        temporary = canonical.with_name(f".{canonical.name}.without-id13.tmp")
        result_hash = _write_without_additions(entry, temporary)
        os.replace(temporary, canonical)
        entry["without_additions_sha256"] = result_hash
    manifest["status"] = "installed-without-additions"
    manifest["additions_removed_utc"] = dt.datetime.now(dt.timezone.utc).isoformat()
    _write_json(_manifest_path(run_dir), manifest)
    print("Removed only ScanID 13 additions; ScanID 12 corrections remain installed.")
    return 0


def _restore_density(entry: dict[str, Any], output_path: Path, chunk_records: int) -> str:
    canonical_info = read_ply_info(Path(entry["source_path"]))
    if entry.get("tail_replacement"):
        base_count = int(entry["candidate_base_vertex_count"])
        if canonical_info.vertex_count < base_count:
            raise ValueError(
                f"Installed cloud is shorter than its preserved base prefix: {canonical_info.path}"
            )
        rollback_info = read_ply_info(Path(entry["superseded_additions_path"]))
        if rollback_info.vertex_count != int(entry["superseded_addition_count"]):
            raise ValueError("Superseded tail rollback count changed")
        source = records(canonical_info)
        rollback = records(rollback_info)
        original_header = bytes.fromhex(entry["source_header_hex"])
        digest = hashlib.sha256(original_header)
        with output_path.open("wb") as stream:
            stream.write(original_header)
            for start in range(0, base_count, chunk_records):
                end = min(start + chunk_records, base_count)
                payload = np.array(source[start:end], copy=True).tobytes(order="C")
                stream.write(payload)
                digest.update(payload)
            for start in range(0, rollback_info.vertex_count, chunk_records):
                end = min(start + chunk_records, rollback_info.vertex_count)
                payload = np.array(rollback[start:end], copy=True).tobytes(order="C")
                stream.write(payload)
                digest.update(payload)
        del source, rollback
        return digest.hexdigest()
    indices = _read_indices(Path(entry["indices_path"]))
    backup_info = read_ply_info(Path(entry["originals_path"]))
    backup = records(backup_info)
    source = records(canonical_info)
    original_count = int(entry["source_vertex_count"])
    original_header = bytes.fromhex(entry["source_header_hex"])
    digest = hashlib.sha256(original_header)
    with output_path.open("wb") as stream:
        stream.write(original_header)
        for start in range(0, original_count, chunk_records):
            end = min(start + chunk_records, original_count)
            chunk = np.array(source[start:end], copy=True)
            left = int(np.searchsorted(indices, start, side="left"))
            right = int(np.searchsorted(indices, end, side="left"))
            if right > left:
                local_indices = (indices[left:right] - np.uint64(start)).astype(np.int64)
                chunk[local_indices] = backup[left:right]
            payload = chunk.tobytes(order="C")
            stream.write(payload)
            digest.update(payload)
    del source, backup
    return digest.hexdigest()


def command_restore(args: argparse.Namespace) -> int:
    run_dir = args.run_dir.resolve()
    manifest = _load_manifest(run_dir)
    if not manifest.get("installed"):
        raise ValueError("The run is not installed")
    if _application_running():
        raise ValueError("Invisible Places is running; close it before restoration")
    prepared: list[tuple[dict[str, Any], Path]] = []
    try:
        for entry in manifest["densities"]:
            canonical = Path(entry["source_path"])
            current_hash = sha256_path(canonical)
            allowed = {entry["candidate_sha256"], entry.get("without_additions_sha256"), entry["source_sha256"]}
            if current_hash == entry["source_sha256"]:
                continue
            if current_hash not in allowed:
                raise ValueError(f"Refusing to restore an unrecognised canonical state: {canonical}")
            temporary = canonical.with_name(f".{canonical.name}.restore.tmp")
            restored_hash = _restore_density(entry, temporary, args.chunk_records)
            if restored_hash != entry["source_sha256"]:
                raise ValueError(f"Restored SHA-256 does not match the manifest for {canonical}")
            prepared.append((entry, temporary))
        for entry, temporary in prepared:
            os.replace(temporary, Path(entry["source_path"]))
    finally:
        for _, temporary in prepared:
            if temporary.exists():
                temporary.unlink()
    manifest["installed"] = False
    manifest["status"] = "restored"
    manifest["restored_utc"] = dt.datetime.now(dt.timezone.utc).isoformat()
    _write_json(_manifest_path(run_dir), manifest)
    print(
        "Restored the exact original headers and replacement records; "
        "all ScanID 13 additions were removed."
    )
    return 0


def _tree_logical_size(path: Path) -> int:
    if not path.exists() and not path.is_symlink():
        return 0
    if path.is_file() or path.is_symlink():
        return path.lstat().st_size
    return sum(
        child.lstat().st_size
        for child in path.rglob("*")
        if child.is_file() or child.is_symlink()
    )


def command_compact(args: argparse.Namespace) -> int:
    """Discard installed-run duplicates while retaining byte-exact rollback data."""
    run_dir = args.run_dir.resolve()
    manifest = _load_manifest(run_dir)
    if not manifest.get("installed"):
        raise ValueError("Only a successfully installed run can be compacted")
    if _application_running():
        raise ValueError("Invisible Places is running; close it before compacting patch data")

    removable: set[Path] = set()
    retained: list[str] = []
    for entry in manifest["densities"]:
        canonical = Path(entry["source_path"])
        current_hash = sha256_path(canonical)
        allowed = {
            entry["candidate_sha256"],
            entry.get("without_additions_sha256"),
            entry["source_sha256"],
        }
        if current_hash not in allowed:
            raise ValueError(
                f"Refusing to compact an unrecognised canonical state: {canonical}"
            )
        originals_path = Path(entry["originals_path"])
        indices_path = Path(entry["indices_path"])
        if not originals_path.is_file() or not indices_path.is_file():
            raise ValueError(
                f"Rollback records or index map are missing for {entry['spacing_mm']} mm"
            )
        originals_info = read_ply_info(originals_path)
        indices = _read_indices(indices_path)
        if (
            originals_info.vertex_count != int(entry["replacement_count"])
            or len(indices) != int(entry["replacement_count"])
        ):
            raise ValueError(
                f"Rollback record counts disagree for {entry['spacing_mm']} mm"
            )
        retained.extend((str(originals_path), str(indices_path)))
        if entry.get("tail_replacement"):
            superseded_path = Path(entry["superseded_additions_path"])
            if not superseded_path.is_file():
                raise ValueError(
                    f"Superseded tail backup is missing for {entry['spacing_mm']} mm"
                )
            superseded_info = read_ply_info(superseded_path)
            if superseded_info.vertex_count != int(entry["superseded_addition_count"]):
                raise ValueError(
                    f"Superseded tail count disagrees for {entry['spacing_mm']} mm"
                )
            if sha256_path(superseded_path) != entry["superseded_additions_sha256"]:
                raise ValueError(
                    f"Superseded tail backup changed for {entry['spacing_mm']} mm"
                )
            retained.append(str(superseded_path))
        removable.update((
            Path(entry["replacements_path"]),
            Path(entry["additions_path"]),
        ))
        candidate_path = Path(entry["candidate_path"])
        if candidate_path.exists():
            if sha256_path(candidate_path) != entry["candidate_sha256"]:
                raise ValueError(f"Refusing to remove a changed candidate: {candidate_path}")
            removable.add(candidate_path)

    preserved_reports: dict[str, str] = {}
    for label, source in (
        ("candidate_smoke_report", run_dir / "validation-render" / "scene3-patch-boundaries.json"),
        ("post_install_smoke_report", run_dir / "post-install-render" / "scene3-patch-boundaries.json"),
    ):
        if source.is_file():
            target = run_dir / ("candidate-smoke-report.json" if label == "candidate_smoke_report" else "post-install-smoke-report.json")
            shutil.copy2(source, target)
            preserved_reports[label] = str(target.resolve())

    if args.keep_final_pngs:
        source_dir = run_dir / "post-install-render"
        target_dir = run_dir / "final-render"
        for name in (
            "patch_01.png",
            "patch_02.png",
            "patch_03.png",
            "patch_04.png",
            "patch-01-02-03-contact-sheet.png",
            "patch-01-02-03-04-contact-sheet.png",
        ):
            source = source_dir / name
            if source.is_file():
                target_dir.mkdir(parents=True, exist_ok=True)
                shutil.copy2(source, target_dir / name)

    removable.update((
        run_dir / "validation-data",
        run_dir / "validation-render",
        run_dir / "post-install-render",
        run_dir / "1mm-2mm-baseline-render",
        run_dir / ".DS_Store",
    ))
    validation_project = manifest.get("validation_project")
    if validation_project:
        removable.add(Path(validation_project))
    removed_bytes = 0
    removed_files = 0
    removed_paths: list[str] = []
    for path in sorted(removable, key=lambda value: len(value.parts), reverse=True):
        resolved_parent = path.resolve(strict=False)
        if not resolved_parent.is_relative_to(run_dir):
            raise ValueError(f"Refusing to compact a path outside the run: {path}")
        if not path.exists() and not path.is_symlink():
            continue
        removed_bytes += _tree_logical_size(path)
        removed_files += (
            1
            if path.is_file() or path.is_symlink()
            else sum(1 for child in path.rglob("*") if child.is_file() or child.is_symlink())
        )
        removed_paths.append(str(path))
        if path.is_dir() and not path.is_symlink():
            shutil.rmtree(path)
        else:
            path.unlink()

    manifest.update(preserved_reports)
    manifest.pop("validation_project", None)
    manifest.pop("validation_data_root", None)
    manifest["compacted"] = True
    manifest["compacted_utc"] = dt.datetime.now(dt.timezone.utc).isoformat()
    manifest["compaction"] = {
        "removed_logical_bytes": removed_bytes,
        "removed_file_count": removed_files,
        "removed_paths": removed_paths,
        "retained_rollback_paths": retained,
        "kept_final_pngs": bool(args.keep_final_pngs),
    }
    _write_json(_manifest_path(run_dir), manifest)
    print(
        f"Compacted {run_dir.name}: removed {removed_bytes / (1024**3):.2f} GiB "
        f"across {removed_files} files; byte-exact rollback records remain.",
        flush=True,
    )
    return 0


def build_parser() -> argparse.ArgumentParser:
    repo_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    build = subparsers.add_parser("build", help="Build reversible candidates and a validation project")
    build.add_argument("--repo-root", type=Path, default=repo_root)
    build.add_argument("--run-dir", type=Path)
    build.add_argument(
        "--cleanmesh-patch", type=Path,
        default=Path("/Users/juju/Documents/Repositories/CleanMesh/Output/Site3-SAND-ScanID11-colour-matched.ply"),
    )
    build.add_argument(
        "--cleanmesh-report", type=Path,
        default=Path("/Users/juju/Documents/Repositories/CleanMesh/Output/Site3-SAND-colour-match-report.json"),
    )
    build.add_argument("--project", type=Path, default=repo_root / "Saved" / "ExhibitionFinal_project.json")
    build.add_argument(
        "--spacings",
        type=int,
        nargs="+",
        choices=SPACINGS_MM,
        default=list(SPACINGS_MM),
        help="LOD spacings to build; use '--spacings 5' for a non-installable camera preview",
    )
    build.add_argument("--chunk-records", type=int, default=2_000_000)
    build.set_defaults(handler=command_build)

    augmentation = subparsers.add_parser(
        "build-augmentation",
        help="Build append-only Patch 01 density candidates from an installed base run",
    )
    augmentation.add_argument("--repo-root", type=Path, default=repo_root)
    augmentation.add_argument("--run-dir", type=Path, required=True)
    augmentation.add_argument("--base-run-dir", type=Path, required=True)
    augmentation.add_argument("--chunk-records", type=int, default=2_000_000)
    augmentation.set_defaults(handler=command_build_augmentation)

    edge_augmentation = subparsers.add_parser(
        "build-edge-augmentation",
        help="Build a staged Patch 04 ScanID 9 density taper",
    )
    edge_augmentation.add_argument("--repo-root", type=Path, default=repo_root)
    edge_augmentation.add_argument("--run-dir", type=Path, required=True)
    edge_augmentation.add_argument("--base-run-dir", type=Path, required=True)
    edge_augmentation.add_argument(
        "--replace-installed-edge-run",
        type=Path,
        help=(
            "Replace the known ScanID 13 tail from this installed append-only "
            "Patch 04 run while leaving canonical clouds untouched during build"
        ),
    )
    edge_augmentation.add_argument(
        "--mesh-samples",
        type=Path,
        default=repo_root / "Data" / "Scene3" / "LinearNoisePAtchPoints.ply",
    )
    edge_augmentation.add_argument(
        "--project",
        type=Path,
        default=repo_root / "Saved" / "ExhibitionFinal_project.json",
    )
    edge_augmentation.add_argument(
        "--spacings",
        type=int,
        nargs="+",
        choices=SPACINGS_MM,
        default=list(SPACINGS_MM),
        help="ROCK spacings to build; all four are required for installation",
    )
    edge_augmentation.add_argument(
        "--edge-profile",
        choices=("linear", "jagged"),
        default="jagged",
        help="Use a constant reach or connected, surface-aware jagged fade",
    )
    edge_augmentation.add_argument(
        "--blend-width",
        type=float,
        default=PATCH_04_BLEND_WIDTH_METRES,
        help="Outside density-taper width in metres for the linear profile",
    )
    edge_augmentation.add_argument(
        "--jagged-min-width",
        type=float,
        default=PATCH_04_JAGGED_MIN_WIDTH_METRES,
        help="Minimum jagged fade reach in metres",
    )
    edge_augmentation.add_argument(
        "--jagged-base-width",
        type=float,
        default=PATCH_04_JAGGED_BASE_WIDTH_METRES,
        help="Median jagged fade reach in metres",
    )
    edge_augmentation.add_argument(
        "--jagged-max-width",
        type=float,
        default=PATCH_04_JAGGED_MAX_WIDTH_METRES,
        help="Maximum broad-lobe fade reach in metres",
    )
    edge_augmentation.add_argument(
        "--jagged-core-width",
        type=float,
        default=PATCH_04_JAGGED_CORE_WIDTH_METRES,
        help="Full-density core outside the measured edge before tapering",
    )
    edge_augmentation.add_argument(
        "--noise-seed",
        type=int,
        default=PATCH_04_JAGGED_NOISE_SEED,
        help="Deterministic connected-zone seed",
    )
    edge_augmentation.add_argument("--chunk-records", type=int, default=2_000_000)
    edge_augmentation.set_defaults(handler=command_build_edge_augmentation)

    refresh = subparsers.add_parser(
        "refresh-additions",
        help="Regenerate only addition tails in an existing complete candidate run",
    )
    refresh.add_argument("--run-dir", type=Path, required=True)
    refresh.add_argument("--chunk-records", type=int, default=2_000_000)
    refresh.set_defaults(handler=command_refresh_additions)

    verify = subparsers.add_parser("verify", help="Verify preservation, IDs, metrics, and byte-exact restoration")
    verify.add_argument("--run-dir", type=Path, required=True)
    verify.add_argument("--chunk-records", type=int, default=2_000_000)
    verify.set_defaults(handler=command_verify)

    install = subparsers.add_parser("install", help="Transactionally install verified candidates")
    install.add_argument("--run-dir", type=Path, required=True)
    install.add_argument("--candidate-smoke-report", type=Path)
    install.add_argument(
        "--app", type=Path,
        default=repo_root / "build" / "macos-debug" / "invisible_places.app" / "Contents" / "MacOS" / "invisible_places",
    )
    install.set_defaults(handler=command_install)

    remove = subparsers.add_parser("remove-additions", help="Remove only appended ScanID 13 points")
    remove.add_argument("--run-dir", type=Path, required=True)
    remove.set_defaults(handler=command_remove_additions)

    restore = subparsers.add_parser("restore", help="Restore byte-exact original canonical clouds")
    restore.add_argument("--run-dir", type=Path, required=True)
    restore.add_argument("--chunk-records", type=int, default=2_000_000)
    restore.set_defaults(handler=command_restore)

    compact = subparsers.add_parser(
        "compact",
        help="Remove installed-run duplicates while retaining exact rollback records",
    )
    compact.add_argument("--run-dir", type=Path, required=True)
    compact.add_argument(
        "--keep-final-pngs",
        action="store_true",
        help="Retain only the unannotated post-install PNGs and contact sheet",
    )
    compact.set_defaults(handler=command_compact)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if hasattr(args, "chunk_records") and args.chunk_records <= 0:
        parser.error("--chunk-records must be positive")
    try:
        return int(args.handler(args))
    except (OSError, ValueError, RuntimeError, json.JSONDecodeError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
