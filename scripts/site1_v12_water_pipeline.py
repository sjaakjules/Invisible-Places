#!/usr/bin/env python3
"""Build the fine Scene1 v12 WATER geometry from measured local support.

The review annotations select neighbourhoods to inspect; they never become
fill or deletion masks.  Candidate WATER is sampled from the verified v10
height/noise surface, connected to current WATER, checked against a robust
local terrain surface, and finally separated from the complete 1 mm terrain
by an absolute 3-D clearance pass.  Current WATER rows are copied byte-for-byte
and all appended rows are archived before scalar analysis.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import re
import sys
from typing import Iterable, Mapping, Sequence

import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import rebuild_site1_fossils_v10 as v10  # noqa: E402
import site1_v11_confidence as confidence  # noqa: E402
import site1_v11_water_density as density  # noqa: E402
import site1_v12_water_refinement as refinement  # noqa: E402


WATER_SCAN_ID = 999.0
DEFAULT_SEED = 120827
CHUNK_RECORDS = 1_000_000
DENSITY_CONTINUITY_KINDS = frozenset(("interface", "hole", "dip"))
COLLISION_QUERY_BLOCK = 250_000
MAX_FAR_LOBE_GRID_CELLS = 100_000_000


@dataclass(frozen=True)
class CircleSpec:
    identifier: str
    center_xy: tuple[float, float]
    radius_m: float
    kind: str
    oversample_pitch_m: float
    maximum_water_support_distance_m: float
    priority: float
    label: int


@dataclass(frozen=True)
class LocalRecords:
    source_indices: np.ndarray
    records: np.ndarray


@dataclass(frozen=True)
class GeometryBuildResult:
    candidate_path: str
    archive_path: str
    manifest_path: str
    source_points: int
    addition_count: int
    candidate_points: int
    candidate_sha256: str


@dataclass(frozen=True)
class DensityContinuitySettings:
    audit_radius_m: float
    audit_step_m: float
    local_reference_inner_fraction: float
    local_reference_outer_margin_m: float
    reference_neighbours: int
    minimum_reference_windows: int
    shoreline_minimum_terrain_fraction: float
    minimum_ratio: float
    maximum_ratio: float
    reservoir_margin_m: float
    support_margin_m: float


@dataclass(frozen=True)
class DensityAuditCentres:
    centres_xy: np.ndarray
    spec_id: tuple[str, ...]
    spec_kind: tuple[str, ...]
    spec_label: np.ndarray


@dataclass(frozen=True)
class AuditDiskSupport:
    """Deterministic physical support sampled inside moving audit disks."""

    sample_pitch_m: float
    sample_cell_area_m2: float
    full_disk_sample_count: int
    valid_footprint_sample_count: np.ndarray
    valid_footprint_area_m2: np.ndarray
    active_mask: np.ndarray


@dataclass(frozen=True)
class FillableSupportCells:
    """Unique preselection support cells surviving every geometry gate."""

    pitch_m: float
    cell_area_m2: float
    cell_keys: np.ndarray
    cell_centres_xy: np.ndarray
    representative_xy: np.ndarray


@dataclass(frozen=True)
class FarLobeCullSettings:
    seed_xy: tuple[float, float]
    maximum_seed_distance_m: float
    grid_pitch_m: float
    bridge_radius_m: float
    detachment_gap_m: float
    maximum_component_fraction: float


@dataclass(frozen=True)
class FarLobeGridPlan:
    grid_pitch_m: float
    origin_cell: tuple[int, int]
    selected_cell_mask: np.ndarray
    detached: bool
    reason: str
    selected_component_label: int | None
    occupied_cell_count: int
    selected_occupied_cell_count: int
    largest_occupied_cell_count: int
    seed_distance_m: float
    component_fraction: float
    minimum_cell_center_separation_m: float | None
    minimum_point_separation_lower_bound_m: float | None
    bridge_iterations: int


def sha256_path(path: str | Path, *, block_size: int = 32 * 1024 * 1024) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        while block := handle.read(block_size):
            digest.update(block)
    return digest.hexdigest()


def _fingerprint(path: str | Path, *, points: int | None = None) -> dict[str, object]:
    resolved = Path(path).resolve(strict=True)
    stat = resolved.stat()
    result: dict[str, object] = {
        "path": str(resolved),
        "size_bytes": stat.st_size,
        "mtime_ns": stat.st_mtime_ns,
        "sha256": sha256_path(resolved),
    }
    if points is not None:
        result["points"] = int(points)
    return result


def _load_json(path: str | Path) -> tuple[Path, Mapping[str, object]]:
    resolved = Path(path).resolve(strict=True)
    document = json.loads(resolved.read_text(encoding="utf-8"))
    if not isinstance(document, Mapping):
        raise ValueError(f"JSON root must be an object: {resolved}")
    return resolved, document


def load_circle_specs(config_path: str | Path) -> tuple[CircleSpec, ...]:
    """Translate written review actions into bounded search neighbourhoods."""

    _, document = _load_json(config_path)
    raw_marks = document.get("marked_locations")
    if not isinstance(raw_marks, list):
        raise ValueError("v12 config lacks marked_locations")
    mark_by_id = {str(item["id"]): item for item in raw_marks}
    parameters = document.get("parameters")
    if parameters is None:
        parameters = {}
    if not isinstance(parameters, Mapping):
        raise ValueError("v12 config parameters must be an object")
    interface_support = parameters.get("interface_search_distance_m")
    hole_support = parameters.get("hole_search_distance_m")
    dip_support = parameters.get("density_dip_search_distance_m")
    fade_support = parameters.get("clustered_fade_support_distance_m")
    definitions = (
        ("interface_density", "interface", 0.80, 0.00135, 0.10, 4.0),
        ("southern_interface", "interface", 1.00, 0.00135, 0.12, 5.0),
        ("interface_gap_1", "interface", 0.55, 0.00135, 0.10, 5.0),
        ("interface_gap_2", "interface", 0.55, 0.00135, 0.10, 5.0),
        ("small_hole_1", "hole", 0.35, 0.00135, 0.14, 3.0),
        ("small_hole_2", "hole", 0.35, 0.00135, 0.14, 3.0),
        ("density_dip_1", "dip", 0.65, 0.00135, 0.10, 4.5),
        ("density_dip_2", "dip", 0.65, 0.00135, 0.10, 4.5),
    )
    specs: list[CircleSpec] = []
    label = 1
    for identifier, kind, radius, pitch, _support, priority in definitions:
        item = mark_by_id.get(identifier)
        if not isinstance(item, Mapping):
            raise KeyError(f"v12 config lacks mark {identifier}")
        world = item.get("world")
        if not isinstance(world, Sequence) or len(world) < 2:
            raise ValueError(f"mark {identifier} has no world coordinate")
        configured_support = {
            "interface": interface_support,
            "hole": hole_support,
            "dip": dip_support,
        }[kind]
        support = (
            float(configured_support)
            if configured_support is not None
            else float(_support)
        )
        specs.append(CircleSpec(
            identifier=identifier,
            center_xy=(float(world[0]), float(world[1])),
            radius_m=float(item.get("radius_m", radius)),
            kind=kind,
            oversample_pitch_m=float(pitch),
            maximum_water_support_distance_m=float(support),
            priority=float(priority),
            label=label,
        ))
        label += 1
    regions = document.get("review_regions")
    if not isinstance(regions, Mapping):
        raise ValueError("v12 config lacks review_regions")
    fade = regions.get("clustered_fade")
    if not isinstance(fade, Mapping):
        raise ValueError("v12 config lacks clustered_fade review")
    center = fade.get("center")
    if not isinstance(center, Sequence) or len(center) < 2:
        raise ValueError("clustered_fade has no center")
    specs.append(CircleSpec(
        identifier="clustered_fade",
        center_xy=(float(center[0]), float(center[1])),
        radius_m=min(float(fade.get("search_radius_m", 4.0)), 2.50),
        kind="fade",
        oversample_pitch_m=0.0030,
        maximum_water_support_distance_m=(
            float(fade_support) if fade_support is not None else 0.12
        ),
        priority=1.0,
        label=label,
    ))
    return tuple(specs)


def _density_continuity_settings(
    parameters: Mapping[str, object],
) -> DensityContinuitySettings:
    result = DensityContinuitySettings(
        audit_radius_m=float(parameters["density_audit_radius_m"]),
        audit_step_m=float(parameters["density_audit_step_m"]),
        local_reference_inner_fraction=float(
            parameters["density_local_reference_inner_fraction"]
        ),
        local_reference_outer_margin_m=float(
            parameters["density_local_reference_outer_margin_m"]
        ),
        reference_neighbours=int(parameters["density_reference_neighbours"]),
        minimum_reference_windows=int(
            parameters["density_reference_minimum_windows"]
        ),
        shoreline_minimum_terrain_fraction=float(
            parameters["density_shoreline_minimum_terrain_fraction"]
        ),
        minimum_ratio=float(parameters["density_minimum_ratio"]),
        maximum_ratio=float(parameters["density_maximum_ratio"]),
        reservoir_margin_m=float(parameters["density_repair_reservoir_margin_m"]),
        support_margin_m=float(parameters["density_repair_support_margin_m"]),
    )
    if result.audit_radius_m <= 0.0 or result.audit_step_m <= 0.0:
        raise ValueError("density audit radius and step must be positive")
    if not 0.0 < result.local_reference_inner_fraction < 1.0:
        raise ValueError("density local-reference inner fraction must lie in (0, 1)")
    if result.local_reference_outer_margin_m <= 0.0:
        raise ValueError("density local-reference outer margin must be positive")
    if result.reference_neighbours < 1:
        raise ValueError("density reference_neighbours must be positive")
    if not 1 <= result.minimum_reference_windows <= result.reference_neighbours:
        raise ValueError("density minimum reference windows are inconsistent")
    if not 0.0 < result.shoreline_minimum_terrain_fraction < 1.0:
        raise ValueError("density shoreline terrain fraction must lie in (0, 1)")
    if not 0.0 < result.minimum_ratio <= 1.0:
        raise ValueError("density minimum ratio must lie in (0, 1]")
    if result.maximum_ratio < 1.0:
        raise ValueError("density maximum ratio must be at least 1")
    if result.reservoir_margin_m < result.audit_radius_m:
        raise ValueError(
            "density repair reservoir margin must cover one complete audit radius"
        )
    if result.support_margin_m <= 0.0:
        raise ValueError("density repair support margin must be positive")
    return result


def _far_lobe_cull_settings(
    config: Mapping[str, object],
    parameters: Mapping[str, object],
) -> FarLobeCullSettings:
    regions = config.get("review_regions")
    if not isinstance(regions, Mapping):
        raise ValueError("v12 config lacks review_regions")
    raw = regions.get("far_lobe")
    if not isinstance(raw, Mapping):
        raise ValueError("v12 config lacks far_lobe review evidence")
    seed = raw.get("seed")
    if not isinstance(seed, Sequence) or len(seed) < 2:
        raise ValueError("far_lobe review has no finite two-dimensional seed")
    result = FarLobeCullSettings(
        seed_xy=(float(seed[0]), float(seed[1])),
        maximum_seed_distance_m=float(raw["maximum_seed_search_m"]),
        grid_pitch_m=float(parameters["far_lobe_grid_pitch_m"]),
        bridge_radius_m=float(parameters["far_lobe_bridge_radius_m"]),
        detachment_gap_m=float(parameters["far_lobe_detachment_gap_m"]),
        maximum_component_fraction=float(
            parameters["far_lobe_maximum_component_fraction"]
        ),
    )
    numeric = np.asarray((
        *result.seed_xy,
        result.maximum_seed_distance_m,
        result.grid_pitch_m,
        result.bridge_radius_m,
        result.detachment_gap_m,
        result.maximum_component_fraction,
    ), np.float64)
    if not np.all(np.isfinite(numeric)):
        raise ValueError("far-lobe component settings must be finite")
    if min(
        result.maximum_seed_distance_m,
        result.grid_pitch_m,
        result.bridge_radius_m,
        result.detachment_gap_m,
    ) <= 0.0:
        raise ValueError("far-lobe distances must be positive")
    if not 0.0 < result.maximum_component_fraction < 1.0:
        raise ValueError("far-lobe maximum component fraction must lie in (0, 1)")
    return result


def _good_overlap_spec(config: Mapping[str, object]) -> CircleSpec:
    raw_marks = config.get("marked_locations")
    if not isinstance(raw_marks, list):
        raise ValueError("v12 config lacks marked_locations")
    item = next(
        (row for row in raw_marks if str(row.get("id")) == "good_overlap_reference"),
        None,
    )
    if not isinstance(item, Mapping):
        raise KeyError("v12 config lacks good_overlap_reference")
    world = item.get("world")
    if not isinstance(world, Sequence) or len(world) < 2:
        raise ValueError("good_overlap_reference has no world coordinate")
    return CircleSpec(
        identifier="good_overlap_reference",
        center_xy=(float(world[0]), float(world[1])),
        radius_m=float(item.get("radius_m", 0.8)),
        kind="reference",
        oversample_pitch_m=0.0,
        maximum_water_support_distance_m=0.0,
        priority=0.0,
        label=0,
    )


def _expanded_specs(specs: Sequence[CircleSpec], margin_m: float) -> tuple[CircleSpec, ...]:
    return tuple(CircleSpec(
        identifier=item.identifier,
        center_xy=item.center_xy,
        radius_m=item.radius_m + float(margin_m),
        kind=item.kind,
        oversample_pitch_m=item.oversample_pitch_m,
        maximum_water_support_distance_m=item.maximum_water_support_distance_m,
        priority=item.priority,
        label=item.label,
    ) for item in specs)


def _density_reservoir_specs(
    specs: Sequence[CircleSpec],
    settings: DensityContinuitySettings,
) -> tuple[CircleSpec, ...]:
    """Bound a repair reservoir around every release-gated review region.

    A moving window centred on the written circle boundary needs candidates
    for one complete audit radius beyond that boundary.  The extra WATER
    support is equally bounded and is only a proposal reservoir: surface,
    sidedness, complete 1 mm clearance, 1.8 mm spacing, and measured density
    gates still decide whether any row can be appended.
    """

    output: list[CircleSpec] = []
    for item in specs:
        if item.kind in DENSITY_CONTINUITY_KINDS:
            output.append(CircleSpec(
                identifier=item.identifier,
                center_xy=item.center_xy,
                radius_m=item.radius_m + settings.reservoir_margin_m,
                kind=item.kind,
                oversample_pitch_m=item.oversample_pitch_m,
                maximum_water_support_distance_m=(
                    item.maximum_water_support_distance_m
                    + settings.support_margin_m
                ),
                priority=item.priority,
                label=item.label,
            ))
        else:
            output.append(item)
    return tuple(output)


def _primary_proposal_mask(
    xy: np.ndarray,
    labels: np.ndarray,
    water_distance_m: np.ndarray,
    specs: Sequence[CircleSpec],
) -> np.ndarray:
    """Identify the unchanged primary search inside original review bounds."""

    keep = np.zeros(len(xy), dtype=bool)
    by_label = {item.label: item for item in specs}
    for label in np.unique(labels):
        item = by_label.get(int(label))
        if item is None:
            raise KeyError(f"proposal label has no original review spec: {label}")
        member = labels == label
        delta = xy[member] - np.asarray(item.center_xy)[None, :]
        inside = np.sum(delta * delta, axis=1) <= item.radius_m**2 + 1.0e-12
        supported = (
            water_distance_m[member]
            <= item.maximum_water_support_distance_m + 1.0e-12
        )
        keep[np.flatnonzero(member)] = inside & supported
    return keep


def _spec_bbox(specs: Sequence[CircleSpec]) -> tuple[float, float, float, float]:
    if not specs:
        raise ValueError("at least one circle spec is required")
    return (
        min(item.center_xy[0] - item.radius_m for item in specs),
        max(item.center_xy[0] + item.radius_m for item in specs),
        min(item.center_xy[1] - item.radius_m for item in specs),
        max(item.center_xy[1] + item.radius_m for item in specs),
    )


def _circle_union_mask(xy: np.ndarray, specs: Sequence[CircleSpec]) -> np.ndarray:
    keep = np.zeros(len(xy), dtype=bool)
    for item in specs:
        dx = xy[:, 0] - item.center_xy[0]
        dy = xy[:, 1] - item.center_xy[1]
        keep |= dx * dx + dy * dy <= item.radius_m * item.radius_m
    return keep


def collect_records_in_circles(
    path: str | Path,
    specs: Sequence[CircleSpec],
    *,
    chunk_records: int = CHUNK_RECORDS,
) -> LocalRecords:
    info = density.inspect_fixed_stride_ply(path)
    bbox = _spec_bbox(specs)
    index_parts: list[np.ndarray] = []
    record_parts: list[np.ndarray] = []
    for begin, records in density.iter_ply_chunks(
        info.path, info=info, chunk_size=chunk_records
    ):
        x = records["x"].astype(np.float64, copy=False)
        y = records["y"].astype(np.float64, copy=False)
        bbox_keep = (
            (x >= bbox[0]) & (x <= bbox[1])
            & (y >= bbox[2]) & (y <= bbox[3])
        )
        if not np.any(bbox_keep):
            continue
        local = np.flatnonzero(bbox_keep)
        xy = np.column_stack((x[local], y[local]))
        circle_keep = _circle_union_mask(xy, specs)
        local = local[circle_keep]
        if len(local):
            index_parts.append(local.astype(np.int64) + begin)
            record_parts.append(np.asarray(records[local]).copy())
    return LocalRecords(
        source_indices=(
            np.concatenate(index_parts) if index_parts else np.empty(0, np.int64)
        ),
        records=(
            np.concatenate(record_parts)
            if record_parts else np.empty(0, dtype=info.dtype)
        ),
    )


def _splitmix64(value: np.ndarray) -> np.ndarray:
    result = np.asarray(value, np.uint64)
    with np.errstate(over="ignore"):
        result = result + np.uint64(0x9E3779B97F4A7C15)
        result = (result ^ (result >> np.uint64(30))) * np.uint64(0xBF58476D1CE4E5B9)
        result = (result ^ (result >> np.uint64(27))) * np.uint64(0x94D049BB133111EB)
    return result ^ (result >> np.uint64(31))


def _world_jittered_circle(spec: CircleSpec, *, seed: int) -> np.ndarray:
    pitch = spec.oversample_pitch_m
    xmin = spec.center_xy[0] - spec.radius_m
    xmax = spec.center_xy[0] + spec.radius_m
    ymin = spec.center_xy[1] - spec.radius_m
    ymax = spec.center_xy[1] + spec.radius_m
    ix0 = int(math.floor(xmin / pitch)) - 1
    ix1 = int(math.ceil(xmax / pitch)) + 1
    iy0 = int(math.floor(ymin / pitch)) - 1
    iy1 = int(math.ceil(ymax / pitch)) + 1
    columns = ix1 - ix0
    rows = iy1 - iy0
    serial_count = rows * columns
    if serial_count > 4_500_000:
        raise RuntimeError(f"review circle {spec.identifier} creates too many proposals")
    ix = np.tile(np.arange(ix0, ix1, dtype=np.int64), rows)
    iy = np.repeat(np.arange(iy0, iy1, dtype=np.int64), columns)
    with np.errstate(over="ignore"):
        serial = (
            ix.astype(np.uint64) * np.uint64(0xD6E8FEB86659FD93)
            ^ iy.astype(np.uint64) * np.uint64(0xA5A3564E27F886D9)
            ^ np.uint64((seed + spec.label * 0x9E3779B9) & 0xFFFFFFFFFFFFFFFF)
        )
    hx = _splitmix64(serial)
    hy = _splitmix64(hx ^ np.uint64(0xD1B54A32D192ED03))
    jitter_x = ((hx >> np.uint64(11)).astype(np.float64) / float(1 << 53) - 0.5) * pitch
    jitter_y = ((hy >> np.uint64(11)).astype(np.float64) / float(1 << 53) - 0.5) * pitch
    x = (ix.astype(np.float64) + 0.5) * pitch + jitter_x
    y = (iy.astype(np.float64) + 0.5) * pitch + jitter_y
    dx = x - spec.center_xy[0]
    dy = y - spec.center_xy[1]
    keep = dx * dx + dy * dy <= spec.radius_m * spec.radius_m
    return np.column_stack((x[keep], y[keep]))


def generate_review_proposals(
    specs: Sequence[CircleSpec], *, seed: int = DEFAULT_SEED
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    parts: list[np.ndarray] = []
    labels: list[np.ndarray] = []
    priorities: list[np.ndarray] = []
    kinds: list[np.ndarray] = []
    kind_code = {"interface": 1, "hole": 2, "dip": 3, "fade": 4}
    for item in specs:
        points = _world_jittered_circle(item, seed=seed)
        parts.append(points)
        labels.append(np.full(len(points), item.label, np.int16))
        priorities.append(np.full(len(points), item.priority, np.float64))
        kinds.append(np.full(len(points), kind_code[item.kind], np.uint8))
    xy = np.concatenate(parts)
    label = np.concatenate(labels)
    priority = np.concatenate(priorities)
    kind = np.concatenate(kinds)
    # Identical proposals can occur where review circles overlap.  Quantised
    # world XY chooses one deterministically and favours the higher-priority
    # written action without exposing circle or tile boundaries.
    quantised = np.rint(xy / 1.0e-7).astype(np.int64)
    order = np.lexsort((label, -priority, quantised[:, 1], quantised[:, 0]))
    sorted_q = quantised[order]
    first = np.ones(len(order), dtype=bool)
    if len(order) > 1:
        first[1:] = np.any(sorted_q[1:] != sorted_q[:-1], axis=1)
    selected = order[first]
    return xy[selected], label[selected], priority[selected], kind[selected]


def _deduplicate_candidate_pool(
    xyz: np.ndarray,
    xy: np.ndarray,
    label: np.ndarray,
    priority: np.ndarray,
    kind: np.ndarray,
    water_distance: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Merge primary and repair candidates without duplicate spacing blockers.

    The repair reservoir deliberately contains the original primary candidates.
    Repacking both pools therefore needs one deterministic representative per
    exact world location.  Higher-priority review evidence wins; label is the
    stable final tie breaker.  Geometry itself is unchanged.
    """

    count = len(xy)
    shapes = (
        xyz.shape == (count, 3),
        label.shape == (count,),
        priority.shape == (count,),
        kind.shape == (count,),
        water_distance.shape == (count,),
    )
    if not all(shapes):
        raise ValueError("candidate-pool attributes have inconsistent shapes")
    if not count:
        return xyz, xy, label, priority, kind, water_distance
    quantised = np.rint(np.asarray(xy, np.float64) / 1.0e-10).astype(np.int64)
    order = np.lexsort((
        label,
        -np.asarray(priority, np.float64),
        quantised[:, 1],
        quantised[:, 0],
    ))
    sorted_xy = quantised[order]
    first = np.ones(len(order), dtype=bool)
    if len(order) > 1:
        first[1:] = np.any(sorted_xy[1:] != sorted_xy[:-1], axis=1)
    selected = order[first]
    return (
        xyz[selected],
        xy[selected],
        label[selected],
        priority[selected],
        kind[selected],
        water_distance[selected],
    )


def _nearest_distance(query: np.ndarray, support: np.ndarray) -> np.ndarray:
    if not len(support):
        return np.full(len(query), np.inf, dtype=np.float64)
    from scipy.spatial import cKDTree
    distance, _ = cKDTree(support).query(query, k=1, workers=-1)
    return np.asarray(distance, np.float64)


def _roundtrip_xy_to_record_dtype(
    xy: np.ndarray, record_dtype: np.dtype
) -> np.ndarray:
    """Return XY exactly as it will exist in the output PLY payload.

    Scene coordinates are near 800 m while the canonical PLY stores float32
    coordinates.  Selecting on higher-precision proposals can therefore turn
    an accepted 1.8 mm separation into a genuinely sub-1.8 mm stored distance.
    All geometric gates operate on this round-tripped representation instead.
    """

    points = np.asarray(xy, np.float64)
    if points.ndim != 2 or points.shape[1] != 2:
        raise ValueError("XY points must have shape (N, 2)")
    fields = record_dtype.fields or {}
    if "x" not in fields or "y" not in fields:
        raise ValueError("record dtype lacks x/y fields")
    x_dtype = fields["x"][0]
    y_dtype = fields["y"][0]
    result = np.column_stack((
        points[:, 0].astype(x_dtype).astype(np.float64),
        points[:, 1].astype(y_dtype).astype(np.float64),
    ))
    if not np.all(np.isfinite(result)):
        raise ValueError("stored-coordinate XY contains non-finite values")
    return result


def _load_surface(v9_run: str | Path, v10_config: str | Path):
    _, document = _load_json(v10_config)
    return v10.load_surface_reference(Path(v9_run).resolve(strict=True), dict(document))


def _surface_contains(surface, xy: np.ndarray) -> np.ndarray:
    return v10.footprint_contains(surface, xy[:, 0], xy[:, 1])


def _surface_intersects_audit_disks(
    surface,
    xy: np.ndarray,
    *,
    radius_m: float,
) -> np.ndarray:
    """Return exact exclusion-aware footprint support inside each audit disk."""

    support = _surface_audit_disk_support(
        surface,
        xy,
        radius_m=radius_m,
        sample_pitch_m=0.002,
    )
    return support.active_mask


def _audit_disk_offsets(radius_m: float, sample_pitch_m: float) -> np.ndarray:
    radius = float(radius_m)
    pitch = float(sample_pitch_m)
    if not np.isfinite(radius) or radius <= 0.0:
        raise ValueError("surface audit radius must be positive and finite")
    if not np.isfinite(pitch) or pitch <= 0.0:
        raise ValueError("surface support sample pitch must be positive and finite")
    extent = int(math.ceil(radius / pitch))
    axis = np.arange(-extent, extent + 1, dtype=np.float64) * pitch
    xx, yy = np.meshgrid(axis, axis, indexing="xy")
    offsets = np.column_stack((xx.ravel(), yy.ravel()))
    return offsets[
        np.sum(np.square(offsets), axis=1) <= radius * radius + 1.0e-15
    ]


def _surface_audit_disk_support(
    surface,
    xy: np.ndarray,
    *,
    radius_m: float,
    sample_pitch_m: float,
) -> AuditDiskSupport:
    """Measure immutable valid WATER-footprint area in every audit disk.

    Unlike signed-distance dilation, this calls the canonical
    :func:`footprint_contains` policy for every deterministic sample.  Explicit
    raster exclusions and the strict v9 polygon exclusions therefore cannot
    activate terrain-only windows.  Relative disk sampling also retains a
    centre just outside the footprint when its moving disk crosses the edge.
    """

    centres = np.asarray(xy, np.float64)
    if centres.ndim != 2 or centres.shape[1] != 2:
        raise ValueError("surface audit points must have shape (N, 2)")
    offsets = _audit_disk_offsets(radius_m, sample_pitch_m)
    counts = np.zeros(len(centres), dtype=np.int64)
    for row, centre in enumerate(centres):
        samples = centre[None, :] + offsets
        counts[row] = int(np.count_nonzero(_surface_contains(surface, samples)))
    pitch = float(sample_pitch_m)
    area = counts.astype(np.float64) * pitch * pitch
    return AuditDiskSupport(
        sample_pitch_m=pitch,
        sample_cell_area_m2=pitch * pitch,
        full_disk_sample_count=len(offsets),
        valid_footprint_sample_count=counts,
        valid_footprint_area_m2=area,
        active_mask=counts > 0,
    )


def _unique_fillable_support_cells(
    candidate_xy: np.ndarray,
    *,
    pitch_m: float,
) -> FillableSupportCells:
    """Rasterise a safe preselection reservoir without proposal multiplicity."""

    points = np.asarray(candidate_xy, np.float64)
    if points.ndim != 2 or points.shape[1] != 2:
        raise ValueError("fillable support candidates must have shape (N, 2)")
    pitch = float(pitch_m)
    if not np.isfinite(pitch) or pitch <= 0.0:
        raise ValueError("fillable support pitch must be positive and finite")
    if not len(points):
        keys = np.empty((0, 2), dtype=np.int64)
        representatives = np.empty((0, 2), dtype=np.float64)
    else:
        point_keys = np.floor(points / pitch).astype(np.int64)
        # Select one real proposal per occupied support cell.  Cell centres are
        # retained for physical-area quadrature, while representatives let the
        # independent audit prove that every counted vacant cell really had a
        # source-compatible, immutable-WATER-clear proposal.
        order = np.lexsort((points[:, 1], points[:, 0], point_keys[:, 1], point_keys[:, 0]))
        ordered_keys = point_keys[order]
        first = np.ones(len(order), dtype=bool)
        first[1:] = np.any(ordered_keys[1:] != ordered_keys[:-1], axis=1)
        selected = order[first]
        keys = point_keys[selected]
        representatives = points[selected].astype(np.float64, copy=True)
    centres = (keys.astype(np.float64) + 0.5) * pitch
    return FillableSupportCells(
        pitch_m=pitch,
        cell_area_m2=pitch * pitch,
        cell_keys=keys,
        cell_centres_xy=centres,
        representative_xy=representatives,
    )


def _vacant_candidate_mask(
    candidate_xy: np.ndarray,
    immutable_blocker_xy: np.ndarray,
    *,
    spacing_m: float,
    distance_tolerance_m: float = 1.0e-12,
) -> np.ndarray:
    """Select real proposals genuinely vacant of immutable surface WATER."""

    candidates = np.asarray(candidate_xy, np.float64)
    blockers = np.asarray(immutable_blocker_xy, np.float64)
    if candidates.ndim != 2 or candidates.shape[1] != 2:
        raise ValueError("vacant-support candidates must have shape (N, 2)")
    if blockers.ndim != 2 or blockers.shape[1] != 2:
        raise ValueError("immutable blockers must have shape (N, 2)")
    spacing = float(spacing_m)
    tolerance = float(distance_tolerance_m)
    if not np.isfinite(spacing) or spacing <= 0.0:
        raise ValueError("vacant-support spacing must be positive and finite")
    if not np.isfinite(tolerance) or tolerance < 0.0:
        raise ValueError("vacant-support tolerance must be finite and non-negative")
    if not len(candidates):
        return np.empty(0, dtype=bool)
    if not len(blockers):
        return np.ones(len(candidates), dtype=bool)
    distance = _nearest_distance(candidates, blockers)
    return distance >= max(spacing - tolerance, 0.0)


def _surface_records(
    surface,
    candidate_xy: np.ndarray,
    donor_records: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    if not len(donor_records):
        raise RuntimeError("no local WATER scalar donors were collected")
    from scipy.spatial import cKDTree
    donor_xy = np.column_stack((donor_records["x"], donor_records["y"])).astype(np.float64)
    _, donor_index = cKDTree(donor_xy).query(candidate_xy, k=1, workers=-1)
    donor_index = np.asarray(donor_index, np.int64)
    output = np.asarray(donor_records[donor_index]).copy()
    z, normal = v10.surface_values(surface, candidate_xy[:, 0], candidate_xy[:, 1])
    output["x"] = candidate_xy[:, 0].astype(output.dtype["x"])
    output["y"] = candidate_xy[:, 1].astype(output.dtype["y"])
    output["z"] = z.astype(output.dtype["z"])
    if {"nx", "ny", "nz"}.issubset(output.dtype.names or ()):
        output["nx"], output["ny"], output["nz"] = normal.T
    output["scalar_ScanID"] = np.asarray(
        WATER_SCAN_ID, dtype=output.dtype["scalar_ScanID"]
    )
    return output, donor_index


def _clustered_fade_keep(
    xy: np.ndarray,
    center: tuple[float, float],
    radius_m: float,
    *,
    seed: int,
    wavelengths_m: Sequence[float],
) -> tuple[np.ndarray, dict[str, object]]:
    if not len(xy):
        return np.empty(0, bool), {"candidate_count": 0, "kept_count": 0}
    wavelengths = tuple(float(value) for value in wavelengths_m)
    if not wavelengths or any(value <= 0.0 for value in wavelengths):
        raise ValueError("clustered fade wavelengths must be positive")
    raw_weights = np.asarray([0.5**index for index in range(len(wavelengths))])
    weights = tuple((raw_weights / np.sum(raw_weights)).tolist())
    base = refinement.correlated_fade_selection(
        xy,
        bias=0.0,
        strength=4.0,
        wavelengths_m=wavelengths,
        weights=weights,
        seed=seed,
    )
    radial = np.linalg.norm(xy - np.asarray(center)[None, :], axis=1) / radius_m
    radial = np.clip(radial, 0.0, 1.0)
    target = 0.82 * np.square(1.0 - radial) + 0.08
    logit = np.log(target / (1.0 - target))
    probability = 1.0 / (1.0 + np.exp(-(logit + 3.4 * base.noise)))
    keep = base.uniform_rank < probability
    return keep, {
        "candidate_count": int(len(xy)),
        "kept_count": int(np.count_nonzero(keep)),
        "mean_probability": float(np.mean(probability)),
        "mean_noise": float(np.mean(base.noise)),
        "wavelengths_m": list(wavelengths),
        "weights": list(weights),
        "radial_target_inner": 0.90,
        "radial_target_outer": 0.08,
        "stationary_world_coordinate_noise": True,
    }


def _moving_audit_centre_contract(
    specs: Sequence[CircleSpec], *, step_m: float = 0.08
) -> DensityAuditCentres:
    """Make release-gated centres for every high-priority WATER review.

    Rows intentionally remain associated with their written review even when
    two neighbourhoods overlap.  This prevents deduplication from silently
    dropping one region's acceptance evidence.
    """

    parts: list[np.ndarray] = []
    ids: list[str] = []
    kinds: list[str] = []
    labels: list[np.ndarray] = []
    for item in specs:
        if item.kind not in DENSITY_CONTINUITY_KINDS:
            continue
        xy = _circle_grid_centres(
            item.center_xy,
            item.radius_m,
            step_m=step_m,
        )
        parts.append(xy)
        ids.extend([item.identifier] * len(xy))
        kinds.extend([item.kind] * len(xy))
        labels.append(np.full(len(xy), item.label, np.int32))
    if not parts:
        return DensityAuditCentres(
            centres_xy=np.empty((0, 2), np.float64),
            spec_id=(),
            spec_kind=(),
            spec_label=np.empty(0, np.int32),
        )
    return DensityAuditCentres(
        centres_xy=np.concatenate(parts),
        spec_id=tuple(ids),
        spec_kind=tuple(kinds),
        spec_label=np.concatenate(labels),
    )


def _moving_audit_centres(
    specs: Sequence[CircleSpec], *, step_m: float = 0.08
) -> np.ndarray:
    """Compatibility wrapper returning all release-gated centre coordinates."""

    return _moving_audit_centre_contract(specs, step_m=step_m).centres_xy


def _circle_grid_centres(
    center_xy: tuple[float, float],
    radius_m: float,
    *,
    step_m: float,
    inner_radius_m: float = 0.0,
) -> np.ndarray:
    """Deterministic circular/annular moving-window centres."""

    radius = float(radius_m)
    step = float(step_m)
    inner = float(inner_radius_m)
    if radius <= 0.0 or step <= 0.0 or inner < 0.0 or inner >= radius:
        raise ValueError("invalid circular density-reference geometry")
    count = int(math.ceil(radius / step))
    offsets = np.arange(-count, count + 1, dtype=np.int32)
    iy, ix = np.meshgrid(offsets, offsets, indexing="ij")
    dx = ix.ravel().astype(np.float64) * step
    dy = iy.ravel().astype(np.float64) * step
    squared = dx * dx + dy * dy
    keep = (squared <= radius * radius + 1.0e-12) & (
        squared >= inner * inner - 1.0e-12
    )
    return np.column_stack((
        center_xy[0] + dx[keep],
        center_xy[1] + dy[keep],
    ))


def _density_reference_centres(
    specs: Sequence[CircleSpec],
    good_overlap: CircleSpec,
    settings: DensityContinuitySettings,
) -> tuple[np.ndarray, np.ndarray]:
    local_parts: list[np.ndarray] = []
    for item in specs:
        if item.kind not in DENSITY_CONTINUITY_KINDS:
            continue
        local_parts.append(_circle_grid_centres(
            item.center_xy,
            item.radius_m + settings.local_reference_outer_margin_m,
            step_m=settings.audit_step_m,
            inner_radius_m=(
                item.radius_m * settings.local_reference_inner_fraction
            ),
        ))
    if not local_parts:
        raise RuntimeError("no release-gated density reference annulus was configured")
    local = np.concatenate(local_parts)
    quantised = np.rint(local / 1.0e-5).astype(np.int64)
    _, unique = np.unique(quantised, axis=0, return_index=True)
    local = local[np.sort(unique)]
    overlap_radius = max(
        good_overlap.radius_m - settings.audit_radius_m,
        settings.audit_step_m,
    )
    overlap = _circle_grid_centres(
        good_overlap.center_xy,
        overlap_radius,
        step_m=settings.audit_step_m,
    )
    return local, overlap


def _points_within_audit_centres(
    xy: np.ndarray,
    centres_xy: np.ndarray,
    *,
    radius_m: float,
) -> np.ndarray:
    """Select a bounded proposal reservoir around deficient moving windows."""

    if not len(xy) or not len(centres_xy):
        return np.zeros(len(xy), dtype=bool)
    from scipy.spatial import cKDTree

    distance, _ = cKDTree(centres_xy).query(xy, k=1, workers=-1)
    return np.asarray(distance, np.float64) <= float(radius_m) + 1.0e-12


def _circle_point_counts(
    centres_xy: np.ndarray,
    support_xy: np.ndarray,
    *,
    radius_m: float,
) -> np.ndarray:
    if not len(centres_xy):
        return np.empty(0, np.int64)
    if not len(support_xy):
        return np.zeros(len(centres_xy), np.int64)
    from scipy.spatial import cKDTree

    return np.asarray(
        cKDTree(support_xy).query_ball_point(
            centres_xy,
            float(radius_m),
            return_length=True,
            workers=-1,
        ),
        np.int64,
    )


def _circle_spec_membership_counts(
    centres_xy: np.ndarray,
    centre_spec_label: np.ndarray,
    candidate_xy: np.ndarray,
    candidate_spec_label: np.ndarray,
    *,
    radius_m: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Count same-review and cross-review candidates in each audit disk."""

    centre_labels = np.asarray(centre_spec_label, np.int64)
    candidate_labels = np.asarray(candidate_spec_label, np.int64)
    if centre_labels.shape != (len(centres_xy),):
        raise ValueError("centre_spec_label count differs from centres")
    if candidate_labels.shape != (len(candidate_xy),):
        raise ValueError("candidate_spec_label count differs from candidates")
    same = np.zeros(len(centres_xy), np.int64)
    cross = np.zeros(len(centres_xy), np.int64)
    if not len(centres_xy) or not len(candidate_xy):
        return same, cross
    from scipy.spatial import cKDTree

    members = cKDTree(candidate_xy).query_ball_point(
        centres_xy,
        float(radius_m),
        workers=-1,
    )
    for row, indices in enumerate(members):
        labels = candidate_labels[np.asarray(indices, np.int64)]
        same[row] = np.count_nonzero(labels == centre_labels[row])
        cross[row] = len(labels) - same[row]
    return same, cross


def _density_failure_document(
    *,
    audit_contract: DensityAuditCentres,
    target_combined_count: np.ndarray,
    combined_lower_count: np.ndarray,
    combined_upper_count: np.ndarray,
    allowed_combined_upper_count: np.ndarray,
    base_water_count: np.ndarray,
    terrain_count: np.ndarray,
    combined_before_count: np.ndarray,
    combined_after_count: np.ndarray,
    primary_cleared_count: np.ndarray,
    provisional_primary_selected_count: np.ndarray,
    repair_raw_count: np.ndarray,
    repair_clear_count: np.ndarray,
    joint_same_spec_count: np.ndarray,
    joint_cross_spec_count: np.ndarray,
    refill: refinement.CircularDensityRefillResult,
    unresolved_mask: np.ndarray,
    required_mask: np.ndarray,
    reference_surface_active_mask: np.ndarray,
    terrain_boundary_mask: np.ndarray,
    source_water_active_mask: np.ndarray,
    minimum_ratio: float,
    maximum_ratio: float,
) -> dict[str, object]:
    """Build a compact, row-addressable fail-closed density diagnostic."""

    unresolved = np.asarray(unresolved_mask, bool)
    required = np.asarray(required_mask, bool)
    reference_surface_active = np.asarray(
        reference_surface_active_mask, bool
    )
    terrain_boundary = np.asarray(terrain_boundary_mask, bool)
    source_water_active = np.asarray(source_water_active_mask, bool)
    count = len(audit_contract.centres_xy)
    arrays = (
        target_combined_count,
        combined_lower_count,
        combined_upper_count,
        allowed_combined_upper_count,
        base_water_count,
        terrain_count,
        combined_before_count,
        combined_after_count,
        primary_cleared_count,
        provisional_primary_selected_count,
        repair_raw_count,
        repair_clear_count,
        joint_same_spec_count,
        joint_cross_spec_count,
        refill.candidate_count_per_centre,
        refill.selected_count_per_centre,
        refill.spacing_blocked_count_per_centre,
        refill.upper_blocked_count_per_centre,
        refill.remaining_deficit_count,
    )
    masks = (
        unresolved,
        required,
        reference_surface_active,
        terrain_boundary,
        source_water_active,
    )
    if (
        any(value.shape != (count,) for value in masks)
        or any(np.shape(value) != (count,) for value in arrays)
    ):
        raise ValueError("density failure diagnostic arrays differ from audit centres")
    if np.any(unresolved & ~required):
        raise ValueError("density failure diagnostic includes an inactive window")
    rows: list[dict[str, object]] = []
    by_region: dict[str, int] = {}
    required_by_region: dict[str, int] = {}
    for index in np.flatnonzero(required):
        identifier = audit_contract.spec_id[int(index)]
        required_by_region[identifier] = required_by_region.get(identifier, 0) + 1
    for index in np.flatnonzero(unresolved):
        row = int(index)
        identifier = audit_contract.spec_id[row]
        by_region[identifier] = by_region.get(identifier, 0) + 1
        pool_count = int(refill.candidate_count_per_centre[row])
        selected_count = int(refill.selected_count_per_centre[row])
        spacing_count = int(refill.spacing_blocked_count_per_centre[row])
        upper_count = int(refill.upper_blocked_count_per_centre[row])
        rows.append({
            "index": row,
            "spec_id": identifier,
            "spec_kind": audit_contract.spec_kind[row],
            "spec_label": int(audit_contract.spec_label[row]),
            "centre_xy": audit_contract.centres_xy[row].tolist(),
            "reference_surface_active": bool(reference_surface_active[row]),
            "terrain_boundary": bool(terrain_boundary[row]),
            "source_water_active": bool(source_water_active[row]),
            "target_combined_count": float(target_combined_count[row]),
            "required_combined_lower_count": int(combined_lower_count[row]),
            "combined_upper_count": int(combined_upper_count[row]),
            "allowed_combined_upper_count": int(
                allowed_combined_upper_count[row]
            ),
            "terrain_count": int(terrain_count[row]),
            "base_water_count": int(base_water_count[row]),
            "combined_before_count": int(combined_before_count[row]),
            "combined_after_count": int(combined_after_count[row]),
            "required_additions_before": int(max(
                combined_lower_count[row] - combined_before_count[row], 0
            )),
            "remaining_additions": int(max(
                combined_lower_count[row] - combined_after_count[row], 0
            )),
            "primary_clear_candidates": int(primary_cleared_count[row]),
            "provisional_primary_selected": int(
                provisional_primary_selected_count[row]
            ),
            "repair_candidates_before_complete_clearance": int(
                repair_raw_count[row]
            ),
            "repair_candidates_after_complete_clearance": int(
                repair_clear_count[row]
            ),
            "joint_candidates": pool_count,
            "joint_same_spec_candidates": int(joint_same_spec_count[row]),
            "joint_cross_spec_candidates": int(joint_cross_spec_count[row]),
            "joint_selected": selected_count,
            "spacing_blocked": spacing_count,
            "upper_bound_blocked": upper_count,
            "unclassified_joint_candidates": int(max(
                pool_count - selected_count - spacing_count - upper_count, 0
            )),
        })
    return {
        "schema_version": 2,
        "status": "failed_closed",
        "reason": "measured moving-window lower continuity unresolved",
        "minimum_ratio": float(minimum_ratio),
        "maximum_ratio": float(maximum_ratio),
        "eligibility_rule": (
            "interface=reference_surface_disk_intersection|terrain_boundary;"
            "hole_or_dip=reference_surface_disk_intersection|source_water"
        ),
        "declared_window_count": count,
        "required_window_count": int(np.count_nonzero(required)),
        "required_windows_by_region": required_by_region,
        "failed_window_count": int(np.count_nonzero(unresolved)),
        "failed_windows_by_region": by_region,
        "failed_windows": rows,
        "large_point_payload_included": False,
    }


def _terrain_boundary_centres(
    centres_xy: np.ndarray,
    terrain_inner_count: np.ndarray,
    terrain_outer_count: np.ndarray,
    *,
    step_m: float,
) -> np.ndarray:
    """Find candidate-independent terrain/free-space boundaries.

    A centre is boundary evidence when it is a terrain-free window with
    terrain in the surrounding ring, or when a terrain-occupied window has an
    adjacent terrain-free audit centre.  Neither current nor final WATER is
    consulted, so missing WATER cannot censor the edge audit.
    """

    centres = np.asarray(centres_xy, np.float64)
    inner = np.asarray(terrain_inner_count, np.int64)
    outer = np.asarray(terrain_outer_count, np.int64)
    if inner.shape != (len(centres),) or outer.shape != (len(centres),):
        raise ValueError("terrain boundary counts must match audit centres")
    boundary = (inner == 0) & (outer > 0)
    terrain_rows = np.flatnonzero(inner > 0)
    free_rows = np.flatnonzero(inner == 0)
    if len(terrain_rows) and len(free_rows):
        from scipy.spatial import cKDTree

        distance, _ = cKDTree(centres[free_rows]).query(
            centres[terrain_rows], k=1, workers=-1
        )
        boundary[terrain_rows] |= (
            np.asarray(distance, np.float64)
            <= math.sqrt(2.0) * float(step_m) + 1.0e-12
        )
    return boundary


def _density_required_mask(
    contract: DensityAuditCentres,
    reference_surface_active_mask: np.ndarray,
    fillable_support_active_mask: np.ndarray,
) -> np.ndarray:
    """Select meaningful, candidate-independent WATER audit windows.

    Written review circles remain neighbourhoods, not fill masks.  Interface
    and hole/dip windows require both an exact exclusion-aware intersection
    with the immutable reference WATER footprint and preselection support that
    survived the immutable surface, source-support, sidedness and complete
    terrain-clearance gates.  Final proposal presence is deliberately excluded
    from eligibility.

    Every configured high-priority review must retain at least one required
    centre; otherwise registration/support disagreement fails closed.
    """

    count = len(contract.centres_xy)
    surface = np.asarray(reference_surface_active_mask, bool)
    fillable = np.asarray(fillable_support_active_mask, bool)
    if any(value.shape != (count,) for value in (surface, fillable)):
        raise ValueError("density eligibility masks differ from audit centres")
    kinds = np.asarray(contract.spec_kind, dtype=object)
    interface = kinds == "interface"
    hole_or_dip = (kinds == "hole") | (kinds == "dip")
    known = interface | hole_or_dip
    if not np.all(known):
        unknown = sorted(set(kinds[~known].tolist()))
        raise ValueError(f"unsupported density eligibility kinds: {unknown}")
    required = surface & fillable
    missing: list[str] = []
    for identifier in dict.fromkeys(contract.spec_id):
        member = np.asarray(
            [value == identifier for value in contract.spec_id],
            dtype=bool,
        )
        if not np.any(required & member):
            missing.append(identifier)
    if missing:
        raise RuntimeError(
            "candidate-independent density eligibility retained no centre for "
            f"configured specs: {missing}"
        )
    return required


def _fine_terrain_collision_mask(
    candidate_xyz: np.ndarray,
    terrain_paths: Sequence[str | Path],
    specs: Sequence[CircleSpec],
    *,
    clearance_m: float,
    tolerance_m: float,
    chunk_records: int,
) -> tuple[np.ndarray, dict[str, object]]:
    """Reject every candidate with a complete 1 mm terrain neighbour."""

    from scipy.spatial import cKDTree
    if not len(candidate_xyz):
        return np.empty(0, bool), {"candidate_count": 0, "collisions": 0}
    threshold = float(clearance_m) - float(tolerance_m)
    if threshold <= 0.0:
        raise ValueError("terrain clearance tolerance consumes the threshold")
    tree = cKDTree(candidate_xyz)
    collision = np.zeros(len(candidate_xyz), dtype=bool)
    query_count = 0
    collision_relations = 0
    search_specs = _expanded_specs(specs, clearance_m + 0.003)
    bbox = _spec_bbox(search_specs)
    source_fingerprints = []
    for raw_path in terrain_paths:
        info = density.inspect_fixed_stride_ply(raw_path)
        source_fingerprints.append(_fingerprint(info.path, points=info.count))
        for _, records in density.iter_ply_chunks(
            info.path, info=info, chunk_size=chunk_records
        ):
            x = records["x"].astype(np.float64, copy=False)
            y = records["y"].astype(np.float64, copy=False)
            local = np.flatnonzero(
                (x >= bbox[0]) & (x <= bbox[1])
                & (y >= bbox[2]) & (y <= bbox[3])
            )
            if not len(local):
                continue
            xy = np.column_stack((x[local], y[local]))
            local = local[_circle_union_mask(xy, search_specs)]
            if not len(local):
                continue
            xyz = np.column_stack((
                records["x"][local], records["y"][local], records["z"][local]
            )).astype(np.float64)
            query_count += len(xyz)
            # One terrain row can lie within clearance of more than one WATER
            # proposal (for example at the midpoint of two 1.8 mm-spaced
            # candidates).  A nearest-only query would leave one colliding row
            # unmarked, so enumerate every neighbour in bounded query blocks.
            for begin in range(0, len(xyz), COLLISION_QUERY_BLOCK):
                neighbours = tree.query_ball_point(
                    xyz[begin : begin + COLLISION_QUERY_BLOCK],
                    r=threshold,
                    workers=-1,
                    return_sorted=False,
                )
                relation_count = sum(len(row) for row in neighbours)
                if not relation_count:
                    continue
                candidate_index = np.fromiter(
                    (int(index) for row in neighbours for index in row),
                    dtype=np.int64,
                    count=relation_count,
                )
                collision[candidate_index] = True
                collision_relations += relation_count
    return collision, {
        "candidate_count": int(len(candidate_xyz)),
        "terrain_points_queried": int(query_count),
        "collisions": int(np.count_nonzero(collision)),
        "collision_relations": int(collision_relations),
        "all_clearance_neighbours_enumerated": True,
        "absolute_clearance_m": float(clearance_m),
        "float32_tolerance_m": float(tolerance_m),
        "effective_collision_threshold_m": threshold,
        "terrain_sources": source_fingerprints,
    }


def _plan_far_lobe_grid_component(
    occupied_cells: np.ndarray,
    settings: FarLobeCullSettings,
) -> FarLobeGridPlan:
    """Resolve the configured far lobe on a conservative occupancy grid.

    The occupied grid is dilated before components are labelled.  This can
    merge nearby pieces, but cannot split a connected main sheet and thereby
    create a false detached lobe.  The final empty-gap check subtracts a full
    grid-cell diagonal from the measured cell-centre distance, making its
    point-to-point separation bound conservative.
    """

    cells = np.asarray(occupied_cells, np.int64)
    if cells.ndim != 2 or cells.shape[1] != 2 or not len(cells):
        raise ValueError("occupied_cells must contain at least one XY grid cell")
    cells = np.unique(cells, axis=0)
    pitch = float(settings.grid_pitch_m)
    bridge_iterations = max(1, int(math.ceil(settings.bridge_radius_m / pitch)))
    padding = bridge_iterations + 2
    minimum = np.min(cells, axis=0) - padding
    maximum = np.max(cells, axis=0) + padding
    shape_array = maximum - minimum + 1
    dense_cell_count = int(np.prod(shape_array, dtype=np.int64))
    if dense_cell_count > MAX_FAR_LOBE_GRID_CELLS:
        raise RuntimeError(
            "far-lobe occupancy grid exceeds the bounded safety limit: "
            f"{dense_cell_count:,} cells"
        )
    shape = (int(shape_array[0]), int(shape_array[1]))
    local = cells - minimum[None, :]
    occupancy = np.zeros(shape, dtype=bool)
    occupancy[local[:, 0], local[:, 1]] = True

    from scipy import ndimage

    bridged = ndimage.binary_dilation(
        occupancy,
        structure=np.ones((3, 3), dtype=bool),
        iterations=bridge_iterations,
    )
    labels, _ = ndimage.label(bridged, structure=np.ones((3, 3), dtype=bool))
    occupied_labels = labels[local[:, 0], local[:, 1]]
    cell_centres = (cells.astype(np.float64) + 0.5) * pitch
    seed = np.asarray(settings.seed_xy, np.float64)
    seed_distance = np.linalg.norm(cell_centres - seed[None, :], axis=1)
    nearest = int(np.argmin(seed_distance))
    selected_label = int(occupied_labels[nearest])
    selected = occupied_labels == selected_label
    component_counts = np.bincount(occupied_labels)
    positive_counts = component_counts[1:]
    selected_count = int(np.count_nonzero(selected))
    largest_count = int(np.max(positive_counts)) if len(positive_counts) else 0
    fraction = selected_count / len(cells)

    def result(
        *,
        detached: bool,
        reason: str,
        minimum_cell_separation: float | None = None,
        minimum_point_separation: float | None = None,
    ) -> FarLobeGridPlan:
        selected_mask = np.zeros(shape, dtype=bool)
        if detached:
            chosen = local[selected]
            selected_mask[chosen[:, 0], chosen[:, 1]] = True
        return FarLobeGridPlan(
            grid_pitch_m=pitch,
            origin_cell=(int(minimum[0]), int(minimum[1])),
            selected_cell_mask=selected_mask,
            detached=detached,
            reason=reason,
            selected_component_label=(selected_label if detached else None),
            occupied_cell_count=int(len(cells)),
            selected_occupied_cell_count=selected_count,
            largest_occupied_cell_count=largest_count,
            seed_distance_m=float(seed_distance[nearest]),
            component_fraction=float(fraction),
            minimum_cell_center_separation_m=minimum_cell_separation,
            minimum_point_separation_lower_bound_m=minimum_point_separation,
            bridge_iterations=bridge_iterations,
        )

    if float(seed_distance[nearest]) > settings.maximum_seed_distance_m:
        return result(
            detached=False,
            reason="no occupied WATER cell lies within maximum_seed_distance_m",
        )
    if selected_count >= largest_count:
        return result(
            detached=False,
            reason="seeded occupancy component is the largest component",
        )
    if fraction > settings.maximum_component_fraction:
        return result(
            detached=False,
            reason="seeded occupancy component exceeds maximum_component_fraction",
        )
    other_centres = cell_centres[~selected]
    if not len(other_centres):
        return result(
            detached=False,
            reason="seeded occupancy component has no independent comparison component",
        )
    from scipy.spatial import cKDTree

    nearest_other, _ = cKDTree(other_centres).query(
        cell_centres[selected], k=1, workers=-1
    )
    cell_separation = float(np.min(nearest_other))
    point_lower_bound = max(0.0, cell_separation - math.sqrt(2.0) * pitch)
    if point_lower_bound < settings.detachment_gap_m:
        return result(
            detached=False,
            reason="seeded occupancy component lacks the conservative detachment gap",
            minimum_cell_separation=cell_separation,
            minimum_point_separation=point_lower_bound,
        )
    return result(
        detached=True,
        reason="detached seeded far-lobe component accepted",
        minimum_cell_separation=cell_separation,
        minimum_point_separation=point_lower_bound,
    )


def _plan_source_far_lobe_cull(
    source_path: str | Path,
    settings: FarLobeCullSettings,
    *,
    chunk_records: int,
) -> FarLobeGridPlan:
    """Stream the immutable fine WATER once to build bounded XY occupancy."""

    info = density.inspect_fixed_stride_ply(source_path)
    occupied: set[tuple[int, int]] = set()
    pitch = settings.grid_pitch_m
    for _, records in density.iter_ply_chunks(
        info.path, info=info, chunk_size=chunk_records
    ):
        cells = np.floor(np.column_stack((records["x"], records["y"])) / pitch)
        cells = np.asarray(cells, np.int64)
        occupied.update(zip(cells[:, 0].tolist(), cells[:, 1].tolist()))
    return _plan_far_lobe_grid_component(
        np.asarray(sorted(occupied), np.int64), settings
    )


def _far_lobe_record_mask(records: np.ndarray, plan: FarLobeGridPlan) -> np.ndarray:
    if not plan.detached or not len(records):
        return np.zeros(len(records), dtype=bool)
    cells = np.floor(
        np.column_stack((records["x"], records["y"])) / plan.grid_pitch_m
    ).astype(np.int64)
    local = cells - np.asarray(plan.origin_cell, np.int64)[None, :]
    shape = np.asarray(plan.selected_cell_mask.shape, np.int64)
    inside = np.all((local >= 0) & (local < shape[None, :]), axis=1)
    result = np.zeros(len(records), dtype=bool)
    rows = np.flatnonzero(inside)
    if len(rows):
        result[rows] = plan.selected_cell_mask[
            local[rows, 0], local[rows, 1]
        ]
    return result


def _surviving_local_records(
    local: LocalRecords, plan: FarLobeGridPlan
) -> LocalRecords:
    """Apply the accepted source cull to an already-collected local view.

    Density baselines, spacing blockers, and scalar donors describe the output
    candidate, so source rows that the reversible far-lobe plan removes cannot
    participate in any of those roles. Retained rows keep source order.
    """

    removed = _far_lobe_record_mask(local.records, plan)
    if not np.any(removed):
        return local
    keep = ~removed
    return LocalRecords(
        source_indices=np.asarray(local.source_indices[keep]).copy(),
        records=np.asarray(local.records[keep]).copy(),
    )


def _archive_far_lobe_source_rows(
    source_path: str | Path,
    plan: FarLobeGridPlan,
    output_dir: str | Path,
    *,
    chunk_records: int,
) -> dict[str, object]:
    """Archive every removed fixed-stride row and original index exactly."""

    if not plan.detached:
        raise ValueError("far-lobe archive requires an accepted detached plan")
    source = density.inspect_fixed_stride_ply(source_path)
    destination = Path(output_dir).resolve(strict=False)
    destination.mkdir(parents=True, exist_ok=True)
    records_path = destination / "far-lobe-cull.records.bin"
    indices_path = destination / "far-lobe-cull.source-indices.i64"
    for path in (records_path, indices_path):
        if path.exists():
            raise FileExistsError(path)
    records_temporary = records_path.with_name(f".{records_path.name}.{os.getpid()}.partial")
    indices_temporary = indices_path.with_name(f".{indices_path.name}.{os.getpid()}.partial")
    removed = 0
    try:
        with records_temporary.open("wb") as records_handle, indices_temporary.open("wb") as indices_handle:
            for begin, records in density.iter_ply_chunks(
                source.path, info=source, chunk_size=chunk_records
            ):
                mask = _far_lobe_record_mask(records, plan)
                local = np.flatnonzero(mask)
                if not len(local):
                    continue
                np.asarray(records[local]).tofile(records_handle)
                (local.astype("<i8") + int(begin)).tofile(indices_handle)
                removed += len(local)
        if removed <= 0:
            raise RuntimeError("accepted far-lobe component contains no source rows")
        if records_temporary.stat().st_size != removed * source.dtype.itemsize:
            raise RuntimeError("far-lobe record archive size mismatch")
        if indices_temporary.stat().st_size != removed * np.dtype("<i8").itemsize:
            raise RuntimeError("far-lobe index archive size mismatch")
        os.replace(records_temporary, records_path)
        os.replace(indices_temporary, indices_path)
    except BaseException:
        records_temporary.unlink(missing_ok=True)
        indices_temporary.unlink(missing_ok=True)
        raise
    return {
        "removed_count": int(removed),
        "record_stride_bytes": int(source.dtype.itemsize),
        "record_archive_format": "raw-fixed-stride-source-dtype",
        "record_dtype_descr": [list(item) for item in source.dtype.descr],
        "records": _fingerprint(records_path),
        "source_indices": _fingerprint(indices_path, points=removed),
        "source_index_dtype": "<i8",
        "exact_source_rows_archived": True,
        "exact_source_indices_archived": True,
    }


def _header_with_comments(header: bytes, count: int, comments: Iterable[str]) -> bytes:
    vertex = re.compile(rb"(?m)^(element vertex )[0-9]+([ \t]*)(\r?)$")
    match = vertex.search(header)
    if match is None:
        raise RuntimeError("could not patch PLY vertex count")
    replacement = match.group(1) + str(int(count)).encode("ascii") + match.group(2) + match.group(3)
    patched = header[: match.start()] + replacement + header[match.end() :]
    marker = b"end_header"
    position = patched.find(marker)
    if position < 0:
        raise RuntimeError("PLY header has no end_header")
    lines = b"".join(
        b"comment " + str(value).encode("ascii", "strict") + b"\n"
        for value in comments
    )
    return patched[:position] + lines + patched[position:]


def append_records_with_comments(
    source_path: str | Path,
    additions: np.ndarray,
    output_path: str | Path,
    *,
    comments: Iterable[str],
    chunk_records: int = CHUNK_RECORDS,
    cull_plan: FarLobeGridPlan | None = None,
    expected_cull_count: int = 0,
) -> dict[str, object]:
    source = density.inspect_fixed_stride_ply(source_path)
    if additions.ndim != 1 or additions.dtype != source.dtype:
        raise ValueError("WATER additions must match the source schema")
    output = density.assert_candidate_output_path(output_path, source_paths=[source.path])
    if output.exists():
        raise FileExistsError(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_name(f".{output.name}.{os.getpid()}.partial")
    temporary.unlink(missing_ok=True)
    before = source.path.stat()
    expected_removed = int(expected_cull_count)
    if expected_removed < 0 or expected_removed > source.count:
        raise ValueError("expected_cull_count is outside the source row range")
    if (cull_plan is None or not cull_plan.detached) and expected_removed:
        raise ValueError("expected_cull_count requires an accepted cull plan")
    copied_source = 0
    try:
        with source.path.open("rb") as handle:
            header = handle.read(source.offset)
        with temporary.open("wb") as handle:
            handle.write(_header_with_comments(
                header, source.count - expected_removed + len(additions), comments
            ))
            for _, records in density.iter_ply_chunks(
                source.path, info=source, chunk_size=chunk_records
            ):
                if cull_plan is None or not cull_plan.detached:
                    surviving = np.asarray(records)
                else:
                    surviving = np.asarray(records)[
                        ~_far_lobe_record_mask(records, cull_plan)
                    ]
                surviving.tofile(handle)
                copied_source += len(surviving)
            additions.tofile(handle)
        if source.count - copied_source != expected_removed:
            raise RuntimeError(
                "candidate far-lobe cull count differs from its exact archive"
            )
        after = source.path.stat()
        if (before.st_size, before.st_mtime_ns) != (after.st_size, after.st_mtime_ns):
            raise RuntimeError("source WATER changed while the candidate was written")
        os.replace(temporary, output)
    except BaseException:
        temporary.unlink(missing_ok=True)
        raise
    candidate_info = density.inspect_fixed_stride_ply(output)
    return {
        "source_points": source.count,
        "culled_source_points": expected_removed,
        "surviving_source_points": copied_source,
        "addition_count": int(len(additions)),
        "candidate_points": candidate_info.count,
        "surviving_source_payload_byte_exact": True,
        "surviving_source_row_order_preserved": True,
        "existing_payload_byte_exact": True,
    }


def _atomic_json(path: Path, value: object) -> None:
    if path.exists():
        raise FileExistsError(path)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.partial")
    temporary.write_text(
        json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, path)


def build_fine_water_geometry(
    *,
    source_water_path: str | Path,
    sand_1mm_path: str | Path,
    rock_1mm_path: str | Path,
    sand_5mm_path: str | Path,
    rock_5mm_path: str | Path,
    review_config_path: str | Path,
    v10_config_path: str | Path,
    v9_run_path: str | Path,
    output_dir: str | Path,
    chunk_records: int = CHUNK_RECORDS,
    seed: int = DEFAULT_SEED,
    log=print,
) -> GeometryBuildResult:
    """Build current fine WATER + confidence-gated v12 additions."""

    destination = Path(output_dir).resolve(strict=False)
    if destination.exists():
        raise FileExistsError(f"refusing existing v12 geometry run: {destination}")
    destination.mkdir(parents=True)
    config_path, config = _load_json(review_config_path)
    parameters = config.get("parameters")
    if not isinstance(parameters, Mapping):
        raise ValueError("v12 config lacks parameters")
    terrain_clearance = float(parameters["terrain_absolute_clearance_m"])
    clearance_tolerance = float(parameters["terrain_clearance_float32_tolerance_m"])
    water_spacing = float(parameters["fine_water_selection_radius_m"])
    density_settings = _density_continuity_settings(parameters)
    support_sample_pitch = float(
        parameters.get(
            "density_support_sampling_pitch_m",
            parameters["fine_water_spacing_m"],
        )
    )
    if not np.isfinite(support_sample_pitch) or support_sample_pitch <= 0.0:
        raise ValueError("density support sampling pitch must be positive")
    far_lobe_settings = _far_lobe_cull_settings(config, parameters)
    specs = load_circle_specs(config_path)
    good_overlap = _good_overlap_spec(config)
    density_specs = tuple(
        item for item in specs if item.kind in DENSITY_CONTINUITY_KINDS
    )
    proposal_specs = _density_reservoir_specs(specs, density_settings)
    density_collection_specs = (
        *_expanded_specs(
            density_specs,
            density_settings.local_reference_outer_margin_m
            + density_settings.audit_radius_m
            + density_settings.support_margin_m,
        ),
        *_expanded_specs((good_overlap,), density_settings.audit_radius_m),
    )
    collection_specs = (
        *_expanded_specs(specs, 0.16),
        *density_collection_specs,
    )

    source_info = density.inspect_fixed_stride_ply(source_water_path)
    source_fp = _fingerprint(source_info.path, points=source_info.count)
    log("v12 geometry: resolve configured far-lobe WATER component")
    far_lobe_plan = _plan_source_far_lobe_cull(
        source_info.path,
        far_lobe_settings,
        chunk_records=chunk_records,
    )
    log(
        "v12 geometry: far-lobe decision: "
        f"{far_lobe_plan.reason} "
        f"({far_lobe_plan.selected_occupied_cell_count:,}/"
        f"{far_lobe_plan.occupied_cell_count:,} occupied cells)"
    )
    log("v12 geometry: collect local fine WATER")
    local_water = collect_records_in_circles(
        source_info.path, collection_specs, chunk_records=chunk_records
    )
    local_water = _surviving_local_records(local_water, far_lobe_plan)
    if not len(local_water.records):
        raise RuntimeError(
            "no surviving existing WATER was found near the v12 reviews"
        )
    water_xy = np.column_stack((
        local_water.records["x"], local_water.records["y"]
    )).astype(np.float64)

    terrain_parts = []
    terrain_source_fps = []
    log("v12 geometry: collect local 5 mm terrain support")
    for terrain_path in (sand_5mm_path, rock_5mm_path):
        local = collect_records_in_circles(
            terrain_path,
            _expanded_specs(proposal_specs, 0.02),
            chunk_records=chunk_records,
        )
        terrain_source_fps.append(_fingerprint(
            terrain_path,
            points=density.inspect_fixed_stride_ply(terrain_path).count,
        ))
        if len(local.records):
            terrain_parts.append(np.column_stack((
                local.records["x"], local.records["y"], local.records["z"]
            )).astype(np.float64))
    if not terrain_parts:
        raise RuntimeError("no 5 mm terrain support was found near v12 reviews")
    terrain_xyz = np.concatenate(terrain_parts)

    fine_terrain_parts: list[np.ndarray] = []
    log("v12 geometry: collect local 1 mm terrain for measured density targets")
    for terrain_path in (sand_1mm_path, rock_1mm_path):
        local = collect_records_in_circles(
            terrain_path,
            density_collection_specs,
            chunk_records=chunk_records,
        )
        if len(local.records):
            fine_terrain_parts.append(np.column_stack((
                local.records["x"], local.records["y"]
            )).astype(np.float64))
    fine_terrain_xy = (
        np.concatenate(fine_terrain_parts)
        if fine_terrain_parts
        else np.empty((0, 2), np.float64)
    )

    surface = _load_surface(v9_run_path, v10_config_path)
    log("v12 geometry: generate and classify world-coordinate proposals")
    proposals, proposal_label, proposal_priority, proposal_kind = (
        generate_review_proposals(proposal_specs, seed=seed)
    )
    # Quantise before every footprint, support, sidedness, terrain-clearance,
    # vacancy and spacing decision. Z/noise below is sampled again at these
    # exact stored XY coordinates.
    proposals = _roundtrip_xy_to_record_dtype(proposals, source_info.dtype)
    raw_proposal_count = len(proposals)
    inside = _surface_contains(surface, proposals)
    proposals = proposals[inside]
    proposal_label = proposal_label[inside]
    proposal_priority = proposal_priority[inside]
    proposal_kind = proposal_kind[inside]

    # A proposal must be a continuation of actual WATER.  The per-review
    # support radius is deliberately independent of the annotation radius.
    water_distance = _nearest_distance(proposals, water_xy)
    support_limit = np.zeros(len(proposals), np.float64)
    spec_by_label = {item.label: item for item in specs}
    reservoir_spec_by_label = {item.label: item for item in proposal_specs}
    for label, item in reservoir_spec_by_label.items():
        support_limit[proposal_label == label] = item.maximum_water_support_distance_m
    supported = water_distance <= support_limit
    proposals = proposals[supported]
    proposal_label = proposal_label[supported]
    proposal_priority = proposal_priority[supported]
    proposal_kind = proposal_kind[supported]
    water_distance = water_distance[supported]
    primary_proposal = _primary_proposal_mask(
        proposals,
        proposal_label,
        water_distance,
        specs,
    )

    fade_spec = next(item for item in specs if item.kind == "fade")
    fade_mask = proposal_kind == 4
    fade_keep, fade_audit = _clustered_fade_keep(
        proposals[fade_mask],
        fade_spec.center_xy,
        fade_spec.radius_m,
        seed=int(parameters["clustered_fade_seed"]),
        wavelengths_m=parameters["clustered_fade_wavelengths_m"],
    )
    correlated_keep = ~fade_mask
    correlated_keep[np.flatnonzero(fade_mask)] = fade_keep
    proposals = proposals[correlated_keep]
    proposal_label = proposal_label[correlated_keep]
    proposal_priority = proposal_priority[correlated_keep]
    proposal_kind = proposal_kind[correlated_keep]
    water_distance = water_distance[correlated_keep]
    primary_proposal = primary_proposal[correlated_keep]

    z, normal = v10.surface_values(surface, proposals[:, 0], proposals[:, 1])
    proposal_xyz = np.column_stack((proposals, z)).astype(np.float64)
    terrain_xy_distance = _nearest_distance(proposals, terrain_xyz[:, :2])
    near_terrain = terrain_xy_distance <= float(parameters["terrain_support_radius_m"])
    terrain_keep = np.ones(len(proposals), dtype=bool)
    terrain_reason = np.zeros(len(proposals), np.uint16)
    if np.any(near_terrain):
        log(
            "v12 geometry: robust local terrain gate for "
            f"{int(np.count_nonzero(near_terrain)):,} proposals"
        )
        gate = refinement.gate_candidates_against_terrain(
            proposal_xyz[near_terrain],
            terrain_xyz,
            absolute_clearance_m=terrain_clearance,
            clearance_tolerance_m=clearance_tolerance,
            neighbours=int(parameters["terrain_support_maximum_neighbours"]),
            support_radius_m=float(parameters["terrain_support_radius_m"]),
            minimum_support=int(parameters["terrain_support_minimum_neighbours"]),
            angular_sectors=8,
            minimum_sectors=3,
            maximum_residual_rms_m=float(parameters["terrain_plane_maximum_residual_m"]),
            below_tolerance_m=0.00025,
            maximum_height_above_m=0.025,
        )
        terrain_keep[np.flatnonzero(near_terrain)] = gate.accepted_mask
        terrain_reason[np.flatnonzero(near_terrain)] = gate.reason_mask
    proposal_xyz = proposal_xyz[terrain_keep]
    proposals = proposals[terrain_keep]
    proposal_label = proposal_label[terrain_keep]
    proposal_priority = proposal_priority[terrain_keep]
    proposal_kind = proposal_kind[terrain_keep]
    water_distance = water_distance[terrain_keep]
    primary_proposal = primary_proposal[terrain_keep]

    # Retain the wider, already surface/sidedness-gated pool.  Only deficient
    # measured windows may draw from it later, after complete 1 mm clearance.
    reservoir_xyz = proposal_xyz
    reservoir_xy = proposals
    reservoir_label = proposal_label
    reservoir_priority = proposal_priority
    reservoir_kind = proposal_kind
    reservoir_water_distance = water_distance
    reservoir_primary = primary_proposal

    raw_reservoir_count = len(reservoir_xy)
    log(
        "v12 geometry: complete 1 mm SAND/ROCK clearance audit for "
        f"{raw_reservoir_count:,} bounded preselection proposals"
    )
    reservoir_collision, reservoir_fine_clearance_audit = (
        _fine_terrain_collision_mask(
            reservoir_xyz,
            (sand_1mm_path, rock_1mm_path),
            proposal_specs,
            clearance_m=terrain_clearance,
            tolerance_m=clearance_tolerance,
            chunk_records=chunk_records,
        )
    )
    reservoir_clear = ~reservoir_collision
    reservoir_xyz = reservoir_xyz[reservoir_clear]
    reservoir_xy = reservoir_xy[reservoir_clear]
    reservoir_label = reservoir_label[reservoir_clear]
    reservoir_priority = reservoir_priority[reservoir_clear]
    reservoir_kind = reservoir_kind[reservoir_clear]
    reservoir_water_distance = reservoir_water_distance[reservoir_clear]
    reservoir_primary = reservoir_primary[reservoir_clear]
    if not len(reservoir_xy):
        raise RuntimeError("complete fine-terrain clearance rejected every proposal")

    primary_index = np.flatnonzero(reservoir_primary)
    if not len(primary_index):
        raise RuntimeError("v12 primary WATER proposal search is empty")
    log(
        "v12 geometry: WATER blue-noise selection from "
        f"{len(primary_index):,} primary proposals; "
        f"{len(reservoir_xy):,} bounded repair-reservoir proposals"
    )
    blue = confidence.variable_radius_blue_noise(
        reservoir_xy[primary_index],
        water_spacing,
        existing_points=water_xy,
        existing_radius=water_spacing,
        priority=reservoir_priority[primary_index],
        seed=seed,
        rebuild_interval=512,
    )
    selected = primary_index[blue.selected_indices]
    proposal_xyz = reservoir_xyz[selected]
    proposals = reservoir_xy[selected]
    proposal_label = reservoir_label[selected]
    proposal_priority = reservoir_priority[selected]
    proposal_kind = reservoir_kind[selected]
    water_distance = reservoir_water_distance[selected]

    # The whole bounded reservoir, including these primary selections, has
    # already passed the complete all-neighbour 1 mm terrain clearance.  No
    # provisional point can now hide a safe alternative by being cleared only
    # after blue-noise selection.

    # Resolve measured continuity only after the complete fine-terrain
    # collision gate.  Every registered interface, gap, hole and dip region is
    # release-gated; fade remains a separately audited advisory treatment.
    # Targets come from local terrain-free WATER and the registered good-
    # overlap shoreline, never from a site-wide constant.
    audit_contract = _moving_audit_centre_contract(
        specs, step_m=density_settings.audit_step_m
    )
    audit_centres = audit_contract.centres_xy
    if not len(audit_centres):
        raise RuntimeError("no release-gated density audit centres were configured")
    local_reference_centres, overlap_reference_centres = (
        _density_reference_centres(specs, good_overlap, density_settings)
    )
    footprint_support = _surface_audit_disk_support(
        surface,
        audit_centres,
        radius_m=density_settings.audit_radius_m,
        sample_pitch_m=support_sample_pitch,
    )
    local_reference_support = _surface_audit_disk_support(
        surface,
        local_reference_centres,
        radius_m=density_settings.audit_radius_m,
        sample_pitch_m=support_sample_pitch,
    )
    overlap_reference_support = _surface_audit_disk_support(
        surface,
        overlap_reference_centres,
        radius_m=density_settings.audit_radius_m,
        sample_pitch_m=support_sample_pitch,
    )
    all_vacant_reservoir_mask = _vacant_candidate_mask(
        reservoir_xy,
        water_xy,
        spacing_m=water_spacing,
    )
    all_vacant_reservoir_indices = np.flatnonzero(all_vacant_reservoir_mask)
    density_reservoir_indices = np.flatnonzero(reservoir_kind != 4)
    raw_fillable_cells = _unique_fillable_support_cells(
        reservoir_xy[density_reservoir_indices],
        pitch_m=support_sample_pitch,
    )
    vacant_local_mask = all_vacant_reservoir_mask[density_reservoir_indices]
    vacant_reservoir_indices = density_reservoir_indices[vacant_local_mask]
    vacant_density_reservoir_mask = np.zeros(len(reservoir_xy), dtype=bool)
    vacant_density_reservoir_mask[vacant_reservoir_indices] = True
    vacant_support_cells = _unique_fillable_support_cells(
        reservoir_xy[vacant_reservoir_indices],
        pitch_m=support_sample_pitch,
    )
    raw_support_sample_count = _circle_point_counts(
        audit_centres,
        raw_fillable_cells.cell_centres_xy,
        radius_m=density_settings.audit_radius_m,
    )
    raw_support_area = np.minimum(
        raw_support_sample_count.astype(np.float64)
        * raw_fillable_cells.cell_area_m2,
        footprint_support.valid_footprint_area_m2,
    )
    vacant_support_sample_count = _circle_point_counts(
        audit_centres,
        vacant_support_cells.cell_centres_xy,
        radius_m=density_settings.audit_radius_m,
    )
    vacant_support_area = np.minimum(
        vacant_support_sample_count.astype(np.float64)
        * vacant_support_cells.cell_area_m2,
        footprint_support.valid_footprint_area_m2,
    )

    # Eligibility is a property of exact footprint and genuinely vacant safe
    # support.  A zero-count capacity reservoir must remain active so that the
    # fail-closed capacity check below reports it instead of masking it.
    fillable_support_active = vacant_support_sample_count > 0
    reference_surface_active = footprint_support.active_mask
    required_mask = _density_required_mask(
        audit_contract,
        reference_surface_active,
        fillable_support_active,
    )
    reference_water_xy = water_xy[_surface_contains(surface, water_xy)]
    measured_targets = refinement.measured_circular_continuity_targets(
        audit_centres,
        water_xy,
        fine_terrain_xy,
        local_reference_centres,
        overlap_reference_centres,
        radius_m=density_settings.audit_radius_m,
        fillable_water_area_m2=vacant_support_area,
        reference_water_xy=reference_water_xy,
        local_reference_water_area_m2=(
            local_reference_support.valid_footprint_area_m2
        ),
        good_overlap_reference_water_area_m2=(
            overlap_reference_support.valid_footprint_area_m2
        ),
        reference_neighbours=density_settings.reference_neighbours,
        minimum_reference_windows=density_settings.minimum_reference_windows,
        shoreline_minimum_terrain_fraction=(
            density_settings.shoreline_minimum_terrain_fraction
        ),
    )
    terrain_boundary_outer_radius = (
        density_settings.audit_radius_m + density_settings.audit_step_m
    )
    terrain_outer_count = _circle_point_counts(
        audit_centres,
        fine_terrain_xy,
        radius_m=terrain_boundary_outer_radius,
    )
    terrain_boundary_mask = _terrain_boundary_centres(
        audit_centres,
        measured_targets.terrain_count,
        terrain_outer_count,
        step_m=density_settings.audit_step_m,
    )
    source_water_active = measured_targets.existing_water_count > 0
    source_support_active = (
        reference_surface_active
        | source_water_active
        | (terrain_outer_count > 0)
    )
    required_count_by_spec: dict[str, int] = {}
    for row in np.flatnonzero(required_mask):
        identifier = audit_contract.spec_id[int(row)]
        required_count_by_spec[identifier] = (
            required_count_by_spec.get(identifier, 0) + 1
        )
    in_density_window = _points_within_audit_centres(
        proposals,
        audit_centres,
        radius_m=density_settings.audit_radius_m,
    )
    fixed_fade_mask = (proposal_kind == 4) & ~in_density_window
    density_candidate_mask = ~fixed_fade_mask
    # Preserve the complete post-clearance primary pool.  The provisional
    # lower-bound pass below discovers which wider reservoir is needed, but it
    # is not frozen into the result: the final joint pass may repack these rows
    # together with safe repair alternatives.
    cleared_primary_xyz = proposal_xyz[density_candidate_mask]
    cleared_primary_xy = proposals[density_candidate_mask]
    cleared_primary_label = proposal_label[density_candidate_mask]
    cleared_primary_priority = proposal_priority[density_candidate_mask]
    cleared_primary_kind = proposal_kind[density_candidate_mask]
    cleared_primary_water_distance = water_distance[density_candidate_mask]
    fixed_fade_xyz = proposal_xyz[fixed_fade_mask]
    fixed_fade_xy = proposals[fixed_fade_mask]
    fixed_fade_label = proposal_label[fixed_fade_mask]
    fixed_fade_priority = proposal_priority[fixed_fade_mask]
    fixed_fade_kind = proposal_kind[fixed_fade_mask]
    fixed_fade_water_distance = water_distance[fixed_fade_mask]
    refill_existing = (
        np.concatenate((water_xy, fixed_fade_xy), axis=0)
        if len(fixed_fade_xy)
        else water_xy
    )

    # Build one globally spacing-valid reservoir against the *actual* fixed
    # solver state: surviving immutable WATER plus fixed fade additions.  The
    # complete packing is a necessary per-window lower-coverage check, not a
    # certificate that the whole packing satisfies overlapping upper bounds.
    # Only the final solver subset can certify both interval sides together.
    capacity_blue = confidence.variable_radius_blue_noise(
        reservoir_xy[vacant_reservoir_indices],
        water_spacing,
        existing_points=refill_existing,
        existing_radius=water_spacing,
        priority=reservoir_priority[vacant_reservoir_indices],
        seed=seed ^ 0xCA9AC17,
        rebuild_interval=512,
    )
    spacing_capacity_reservoir_indices = vacant_reservoir_indices[
        capacity_blue.selected_indices
    ]
    spacing_capacity_candidate_xy = reservoir_xy[
        spacing_capacity_reservoir_indices
    ]
    # Fixed fade rows are blockers, not members of this density-candidate
    # reservoir. They are outside all registered density windows by definition.
    spacing_capacity_xy = spacing_capacity_candidate_xy
    spacing_capacity_count = _circle_point_counts(
        audit_centres,
        spacing_capacity_xy,
        radius_m=density_settings.audit_radius_m,
    )
    addition_contract = refinement.attainable_addition_density_contract(
        measured_targets.existing_water_count,
        measured_targets.raw_desired_addition_count,
        spacing_capacity_count,
        minimum_ratio=density_settings.minimum_ratio,
        maximum_ratio=density_settings.maximum_ratio,
        active_centre_mask=required_mask,
    )
    target_water_count = addition_contract.target_water_count
    target_combined_count = (
        measured_targets.terrain_count.astype(np.float64) + target_water_count
    )
    disk_area = math.pi * density_settings.audit_radius_m**2
    target_water_density_per_m2 = target_water_count / disk_area
    target_combined_density_per_m2 = target_combined_count / disk_area
    water_lower_count = addition_contract.water_lower_count
    water_nominal_upper_count = addition_contract.water_upper_count.copy()
    water_upper_count = addition_contract.water_upper_count
    combined_lower_count = (
        measured_targets.terrain_count + water_lower_count
    ).astype(np.int64)
    # Source rows are outside the addition ratio by construction, so no
    # grandfather exception is needed or permitted.
    immutable_source_upper_grandfather_mask = np.zeros(
        len(audit_centres), dtype=bool
    )
    combined_nominal_upper_count = (
        measured_targets.terrain_count + water_nominal_upper_count
    ).astype(np.int64)
    combined_upper_count = (
        measured_targets.terrain_count + water_upper_count
    ).astype(np.int64)
    capacity_violation = required_mask & (
        addition_contract.addition_lower_count > spacing_capacity_count
    )
    if np.any(capacity_violation):
        failed = np.flatnonzero(capacity_violation)
        preview = [
            {
                "index": int(row),
                "spec_id": audit_contract.spec_id[int(row)],
                "raw_desired_addition_count": float(
                    measured_targets.raw_desired_addition_count[int(row)]
                ),
                "required_addition_lower_count": int(
                    addition_contract.addition_lower_count[int(row)]
                ),
                "spacing_capacity_count": int(spacing_capacity_count[int(row)]),
                "vacant_support_area_m2": float(
                    vacant_support_area[int(row)]
                ),
            }
            for row in failed[:24]
        ]
        _atomic_json(
            destination / "density-capacity-failure.json",
            {
                "status": "failed",
                "reason": (
                    "globally spacing-valid reservoir cannot cover a required "
                    "per-window lower addition count"
                ),
                "failed_window_count": int(len(failed)),
                "required_window_count": int(np.count_nonzero(required_mask)),
                "water_spacing_m": water_spacing,
                "raw_support_cell_count": int(len(raw_fillable_cells.cell_keys)),
                "vacant_support_cell_count": int(len(vacant_support_cells.cell_keys)),
                "spacing_reservoir_selection_count": int(len(spacing_capacity_xy)),
                "fixed_fade_blocker_count": int(len(fixed_fade_xy)),
                "failed_window_preview": preview,
            },
        )
        raise RuntimeError(
            "necessary 1.8 mm spacing-reservoir lower coverage failed "
            f"in {len(failed)} windows; compact diagnostic="
            f"{destination / 'density-capacity-failure.json'}"
        )
    primary_refill = refinement.refill_circular_density_dips(
        audit_centres,
        refill_existing,
        cleared_primary_xy,
        radius_m=density_settings.audit_radius_m,
        target_density_per_m2=target_water_density_per_m2,
        water_spacing_m=water_spacing,
        minimum_ratio=density_settings.minimum_ratio,
        minimum_observed_count=water_lower_count,
        maximum_observed_count=water_upper_count,
        active_centre_mask=required_mask,
        seed=seed ^ 0xD1F,
    )
    provisional_primary = primary_refill.selected_candidate_indices
    proposal_xyz = np.concatenate((
        fixed_fade_xyz,
        cleared_primary_xyz[provisional_primary],
    ))
    proposals = np.concatenate((
        fixed_fade_xy,
        cleared_primary_xy[provisional_primary],
    ))
    proposal_label = np.concatenate((
        fixed_fade_label,
        cleared_primary_label[provisional_primary],
    ))
    proposal_priority = np.concatenate((
        fixed_fade_priority,
        cleared_primary_priority[provisional_primary],
    ))
    proposal_kind = np.concatenate((
        fixed_fade_kind,
        cleared_primary_kind[provisional_primary],
    ))
    water_distance = np.concatenate((
        fixed_fade_water_distance,
        cleared_primary_water_distance[provisional_primary],
    ))
    after_primary_refill_count = len(proposal_xyz)

    combined_before_xy = (
        np.concatenate((water_xy, fine_terrain_xy), axis=0)
        if len(fine_terrain_xy)
        else water_xy
    )
    combined_after_initial_xy = np.concatenate(
        (water_xy, fine_terrain_xy, proposals), axis=0
    )
    before_combined = refinement.moving_circular_density_audit(
        audit_centres,
        combined_before_xy,
        radius_m=density_settings.audit_radius_m,
        target_density_per_m2=target_combined_density_per_m2,
        minimum_ratio=density_settings.minimum_ratio,
    )
    initial_combined = refinement.moving_circular_density_audit(
        audit_centres,
        combined_after_initial_xy,
        radius_m=density_settings.audit_radius_m,
        target_density_per_m2=target_combined_density_per_m2,
        minimum_ratio=density_settings.minimum_ratio,
    )
    # Immutable WATER is outside the addition interval and is therefore
    # accounted exactly once in ``combined_upper_count``.  The source cannot
    # require a grandfather exception under this contract.
    allowed_combined_upper = combined_upper_count.copy()
    initial_upper_violation = (
        initial_combined.observed_count > allowed_combined_upper
    ) & required_mask
    if np.any(initial_upper_violation):
        raise RuntimeError(
            "primary v12 additions exceed the measured moving-window upper "
            f"continuity bound in {int(np.count_nonzero(initial_upper_violation))} "
            "windows"
        )
    primary_cleared_count = _circle_point_counts(
        audit_centres,
        cleared_primary_xy,
        radius_m=density_settings.audit_radius_m,
    )
    provisional_primary_selected_count = _circle_point_counts(
        audit_centres,
        cleared_primary_xy[provisional_primary],
        radius_m=density_settings.audit_radius_m,
    )
    initial_water_count = _circle_point_counts(
        audit_centres,
        np.concatenate((water_xy, proposals), axis=0),
        radius_m=density_settings.audit_radius_m,
    )

    # The primary blue-noise pass occurred before complete terrain clearance.
    # A selected row removed by that clearance can leave a hole while having
    # excluded a nearby valid alternative.  Search around every immutable-base
    # deficit, not only deficits left after the provisional primary pass: the
    # final joint solver is allowed to discard a provisional point and needs
    # the alternatives which that provisional point temporarily satisfied.
    # Clear every repair proposal against all 1 mm terrain neighbours, then
    # apply the same 1.8 mm spacing and measured upper ceiling.
    base_lower = (
        measured_targets.existing_water_count < water_lower_count
    ) & required_mask
    initial_lower = (
        initial_water_count < water_lower_count
    ) & required_mask
    repair_search = (
        vacant_density_reservoir_mask
        & _points_within_audit_centres(
            reservoir_xy,
            audit_centres[base_lower],
            radius_m=density_settings.audit_radius_m,
        )
    )
    repair_xyz = reservoir_xyz[repair_search]
    repair_xy = reservoir_xy[repair_search]
    repair_label = reservoir_label[repair_search]
    repair_priority = reservoir_priority[repair_search]
    repair_kind = reservoir_kind[repair_search]
    repair_water_distance = reservoir_water_distance[repair_search]
    repair_raw_reservoir_count = _circle_point_counts(
        audit_centres,
        repair_xy,
        radius_m=density_settings.audit_radius_m,
    )
    repair_clearance_audit = {
        "candidate_count": int(len(repair_xy)),
        "terrain_points_queried": int(
            reservoir_fine_clearance_audit["terrain_points_queried"]
        ),
        "collisions": 0,
        "collision_relations": 0,
        "all_clearance_neighbours_enumerated": True,
        "absolute_clearance_m": terrain_clearance,
        "float32_tolerance_m": clearance_tolerance,
        "effective_collision_threshold_m": terrain_clearance - clearance_tolerance,
        "terrain_sources": reservoir_fine_clearance_audit["terrain_sources"],
        "inherited_from_complete_preselection_reservoir_clearance": True,
    }
    repair_reservoir_count = _circle_point_counts(
        audit_centres,
        repair_xy,
        radius_m=density_settings.audit_radius_m,
    )

    capacity_xyz = reservoir_xyz[spacing_capacity_reservoir_indices]
    capacity_label = reservoir_label[spacing_capacity_reservoir_indices]
    capacity_priority = reservoir_priority[spacing_capacity_reservoir_indices]
    capacity_kind = reservoir_kind[spacing_capacity_reservoir_indices]
    capacity_distance = reservoir_water_distance[
        spacing_capacity_reservoir_indices
    ]

    # Repack the surviving primary and wider repair candidates together.  The
    # complete spacing reservoir is included explicitly, so the necessary
    # lower-coverage check cannot refer to rows absent from the solver.
    # A provisional primary point is not privileged merely because it was seen
    # before complete clearance; the joint lower-bound solver may supersede it
    # with a candidate that covers more or scarcer deficient windows.
    joint_xyz, joint_xy, joint_label, joint_priority, joint_kind, joint_distance = (
        _deduplicate_candidate_pool(
            np.concatenate((cleared_primary_xyz, repair_xyz, capacity_xyz), axis=0),
            np.concatenate((
                cleared_primary_xy,
                repair_xy,
                spacing_capacity_candidate_xy,
            ), axis=0),
            np.concatenate((cleared_primary_label, repair_label, capacity_label), axis=0),
            np.concatenate((cleared_primary_priority, repair_priority, capacity_priority), axis=0),
            np.concatenate((cleared_primary_kind, repair_kind, capacity_kind), axis=0),
            np.concatenate((
                cleared_primary_water_distance,
                repair_water_distance,
                capacity_distance,
            ), axis=0),
        )
    )
    capacity_row_view = np.ascontiguousarray(spacing_capacity_candidate_xy).view(
        np.dtype((np.void, spacing_capacity_candidate_xy.dtype.itemsize * 2))
    ).ravel()
    joint_row_view = np.ascontiguousarray(joint_xy).view(
        np.dtype((np.void, joint_xy.dtype.itemsize * 2))
    ).ravel()
    capacity_candidate_rows_in_joint_pool = bool(
        np.all(np.isin(capacity_row_view, joint_row_view, assume_unique=False))
    )
    if not capacity_candidate_rows_in_joint_pool:
        raise RuntimeError("spacing-reservoir candidates are absent from joint pool")
    joint_same_spec_count, joint_cross_spec_count = (
        _circle_spec_membership_counts(
            audit_centres,
            audit_contract.spec_label,
            joint_xy,
            joint_label,
            radius_m=density_settings.audit_radius_m,
        )
    )
    joint_refill = refinement.refill_circular_density_dips(
        audit_centres,
        refill_existing,
        joint_xy,
        radius_m=density_settings.audit_radius_m,
        target_density_per_m2=target_water_density_per_m2,
        water_spacing_m=water_spacing,
        minimum_ratio=density_settings.minimum_ratio,
        minimum_observed_count=water_lower_count,
        maximum_observed_count=water_upper_count,
        active_centre_mask=required_mask,
        seed=seed ^ 0xA11CE,
    )
    joint_selected = joint_refill.selected_candidate_indices
    proposal_xyz = np.concatenate((fixed_fade_xyz, joint_xyz[joint_selected]))
    proposals = np.concatenate((fixed_fade_xy, joint_xy[joint_selected]))
    proposal_label = np.concatenate((fixed_fade_label, joint_label[joint_selected]))
    proposal_priority = np.concatenate((
        fixed_fade_priority,
        joint_priority[joint_selected],
    ))
    proposal_kind = np.concatenate((fixed_fade_kind, joint_kind[joint_selected]))
    water_distance = np.concatenate((
        fixed_fade_water_distance,
        joint_distance[joint_selected],
    ))
    repair_selected_count = _circle_point_counts(
        audit_centres,
        joint_xy[joint_selected],
        radius_m=density_settings.audit_radius_m,
    )

    combined_after_xy = np.concatenate(
        (water_xy, fine_terrain_xy, proposals), axis=0
    )
    after_combined = refinement.moving_circular_density_audit(
        audit_centres,
        combined_after_xy,
        radius_m=density_settings.audit_radius_m,
        target_density_per_m2=target_combined_density_per_m2,
        minimum_ratio=density_settings.minimum_ratio,
    )
    final_water_count = _circle_point_counts(
        audit_centres,
        np.concatenate((water_xy, proposals), axis=0),
        radius_m=density_settings.audit_radius_m,
    )
    unresolved_lower = (
        final_water_count < water_lower_count
    ) & required_mask
    upper_violation = (
        final_water_count > water_upper_count
    ) & required_mask
    if np.any(unresolved_lower):
        failed_regions: dict[str, int] = {}
        for row in np.flatnonzero(unresolved_lower):
            identifier = audit_contract.spec_id[int(row)]
            failed_regions[identifier] = failed_regions.get(identifier, 0) + 1
        failure_path = destination / "density-continuity-failure.json"
        _atomic_json(failure_path, _density_failure_document(
            audit_contract=audit_contract,
            target_combined_count=target_combined_count,
            combined_lower_count=combined_lower_count,
            combined_upper_count=combined_upper_count,
            allowed_combined_upper_count=allowed_combined_upper,
            base_water_count=measured_targets.existing_water_count,
            terrain_count=measured_targets.terrain_count,
            combined_before_count=before_combined.observed_count,
            combined_after_count=after_combined.observed_count,
            primary_cleared_count=primary_cleared_count,
            provisional_primary_selected_count=(
                provisional_primary_selected_count
            ),
            repair_raw_count=repair_raw_reservoir_count,
            repair_clear_count=repair_reservoir_count,
            joint_same_spec_count=joint_same_spec_count,
            joint_cross_spec_count=joint_cross_spec_count,
            refill=joint_refill,
            unresolved_mask=unresolved_lower,
            required_mask=required_mask,
            reference_surface_active_mask=reference_surface_active,
            terrain_boundary_mask=terrain_boundary_mask,
            source_water_active_mask=source_water_active,
            minimum_ratio=density_settings.minimum_ratio,
            maximum_ratio=density_settings.maximum_ratio,
        ))
        raise RuntimeError(
            "measured WATER/terrain continuity remains below its moving-window "
            f"lower bound in {int(np.count_nonzero(unresolved_lower))} windows "
            f"after safe reservoir refill; by region={failed_regions}; compact "
            f"diagnostic={failure_path}"
        )
    if np.any(upper_violation):
        raise RuntimeError(
            "v12 additions exceed the measured moving-window upper continuity "
            f"bound in {int(np.count_nonzero(upper_violation))} windows"
        )
    dip_refill_audit: dict[str, object] = {
        "candidate_count": int(len(joint_xy)),
        "selected_count": int(len(joint_selected)),
        "provisional_primary_candidate_count": int(len(cleared_primary_xy)),
        "provisional_primary_selected_count": int(len(provisional_primary)),
        "fixed_fade_count": int(np.count_nonzero(fixed_fade_mask)),
        "repair_candidate_count": int(len(repair_xy)),
        "joint_repack_performed": True,
        "audit_centres": int(len(audit_centres)),
        "circle_radius_m": density_settings.audit_radius_m,
        "minimum_ratio": density_settings.minimum_ratio,
        "maximum_ratio": density_settings.maximum_ratio,
        "target_source": (
            "nearest registered good-overlap WATER-only density divided by "
            "exact exclusion-aware WATER-footprint area, scaled by genuinely "
            "vacant safe support area after immutable-WATER clearance"
        ),
        "raw_desired_addition_count": (
            measured_targets.raw_desired_addition_count.tolist()
        ),
        "target_addition_count": addition_contract.target_addition_count.tolist(),
        "addition_lower_count": addition_contract.addition_lower_count.tolist(),
        "addition_upper_count": addition_contract.addition_upper_count.tolist(),
        "target_water_count": target_water_count.tolist(),
        "target_combined_count": target_combined_count.tolist(),
        "terrain_count": measured_targets.terrain_count.tolist(),
        "shoreline_mask": measured_targets.shoreline_mask.tolist(),
        "shoreline_terrain_count_threshold": (
            measured_targets.shoreline_terrain_count_threshold
        ),
        "reference_kind": list(measured_targets.reference_kind),
        "reference_sample_count": measured_targets.reference_sample_count.tolist(),
        "water_nominal_upper_count": water_nominal_upper_count.tolist(),
        "water_upper_count": water_upper_count.tolist(),
        "water_lower_count": water_lower_count.tolist(),
        "immutable_source_upper_grandfather_count": int(
            np.count_nonzero(immutable_source_upper_grandfather_mask)
        ),
        "combined_lower_count": combined_lower_count.tolist(),
        "combined_upper_count": combined_upper_count.tolist(),
        "dips_before": int(np.count_nonzero(before_combined.dip_mask)),
        "dips_after_primary": int(np.count_nonzero(initial_lower)),
        "dips_after": int(np.count_nonzero(unresolved_lower)),
        "upper_violations_after": int(np.count_nonzero(upper_violation)),
        "median_combined_ratio_before": float(np.median(before_combined.density_ratio)),
        "median_combined_ratio_after": float(np.median(after_combined.density_ratio)),
        "uses_overlapping_circular_windows": True,
        "lower_and_upper_bounds_enforced": True,
        "selection_stops_at_lower_acceptance_bound": True,
        "scarcity_and_multiwindow_ordering": True,
    }

    log(f"v12 geometry: construct {len(proposals):,} archived WATER rows")
    additions, donor_index = _surface_records(surface, proposals, local_water.records)
    far_lobe_archive: dict[str, object] | None = None
    if far_lobe_plan.detached:
        log("v12 geometry: archive configured far-lobe source rows exactly")
        far_lobe_archive = _archive_far_lobe_source_rows(
            source_info.path,
            far_lobe_plan,
            destination,
            chunk_records=chunk_records,
        )
    removed_source_count = (
        int(far_lobe_archive["removed_count"])
        if far_lobe_archive is not None
        else 0
    )
    candidate_path = destination / "Site1-WATER-2mm.geometry-v12.candidate.ply"
    log("v12 geometry: write byte-exact surviving base plus additions candidate")
    append_audit = append_records_with_comments(
        source_info.path,
        additions,
        candidate_path,
        comments=(
            "Scene1 Fossils WATER v12 fine-first interface refinement",
            "Surviving source WATER rows remain byte-exact and in source order",
            "Configured detached far-lobe rows have an exact reversible archive",
            "New WATER uses 1 mm terrain clearance and 1.8 mm WATER spacing",
            "Clustered outward fade uses stationary multiscale world-coordinate noise",
            "5 mm WATER must be derived as an exact full-row subset of this fine cloud",
        ),
        chunk_records=chunk_records,
        cull_plan=far_lobe_plan,
        expected_cull_count=removed_source_count,
    )

    archive_path = destination / "additions.npz"
    for label, cells in (
        ("raw", raw_fillable_cells),
        ("vacant", vacant_support_cells),
    ):
        if len(cells.cell_keys) and (
            np.min(cells.cell_keys) < np.iinfo(np.int32).min
            or np.max(cells.cell_keys) > np.iinfo(np.int32).max
        ):
            raise RuntimeError(
                f"{label} support-cell coordinates exceed int32 archive"
            )
    np.savez(
        archive_path,
        records=additions,
        candidate_xy=proposals.astype(np.float64),
        candidate_label=proposal_label.astype(np.int32),
        candidate_kind=proposal_kind.astype(np.uint8),
        donor_local_index=donor_index.astype(np.int64),
        water_distance_m=water_distance.astype(np.float32),
        priority=proposal_priority.astype(np.float32),
        raw_support_cell_keys=raw_fillable_cells.cell_keys.astype(np.int32),
        raw_support_representative_xy=(
            raw_fillable_cells.representative_xy.astype(np.float64)
        ),
        vacant_support_cell_keys=vacant_support_cells.cell_keys.astype(np.int32),
        vacant_support_representative_xy=(
            vacant_support_cells.representative_xy.astype(np.float64)
        ),
        vacant_safe_reservoir_xy=(
            reservoir_xy[all_vacant_reservoir_indices].astype(np.float64)
        ),
        support_pitch_m=np.asarray(raw_fillable_cells.pitch_m, dtype=np.float64),
        spacing_capacity_xy=spacing_capacity_xy.astype(np.float64),
    )
    candidate_fp = _fingerprint(
        candidate_path,
        points=source_info.count - removed_source_count + len(additions),
    )
    config_fp = _fingerprint(config_path)
    reference_surface_run = Path(v9_run_path).resolve(strict=True)
    reference_surface_path = (
        reference_surface_run / "surface-v9.npz"
    ).resolve(strict=True)
    reference_surface_config = Path(v10_config_path).resolve(strict=True)
    reference_surface_provenance = {
        "v9_run_path": str(reference_surface_run),
        "surface": _fingerprint(reference_surface_path),
        "v10_config": _fingerprint(reference_surface_config),
    }
    implementation_paths = (
        Path(__file__).resolve(),
        Path(refinement.__file__).resolve(),
        Path(confidence.__file__).resolve(),
        Path(v10.__file__).resolve(),
    )
    implementation_hashes = {
        path.name: sha256_path(path) for path in implementation_paths
    }
    accepted_labels = sorted(int(value) for value in np.unique(proposal_label))
    holes = []
    for label in accepted_labels:
        item = spec_by_label[label]
        member = proposal_label == label
        xy = proposals[member]
        holes.append({
            "seed_id": item.identifier,
            "label": label,
            "accepted": True,
            "kind": item.kind,
            "addition_count": int(np.count_nonzero(member)),
            "area_m2": float(math.pi * item.radius_m * item.radius_m),
            "bounds": [
                item.center_xy[0] - item.radius_m,
                item.center_xy[0] + item.radius_m,
                item.center_xy[1] - item.radius_m,
                item.center_xy[1] + item.radius_m,
            ],
        })

    far_lobe_audit: dict[str, object] = {
        "performed": bool(far_lobe_plan.detached),
        "reversible": True,
        "measured_no_eligible_component": not far_lobe_plan.detached,
        "reason": far_lobe_plan.reason,
        "seed_xy": list(far_lobe_settings.seed_xy),
        "maximum_seed_distance_m": far_lobe_settings.maximum_seed_distance_m,
        "grid_pitch_m": far_lobe_settings.grid_pitch_m,
        "bridge_radius_m": far_lobe_settings.bridge_radius_m,
        "bridge_iterations": far_lobe_plan.bridge_iterations,
        "detachment_gap_m": far_lobe_settings.detachment_gap_m,
        "maximum_component_fraction": (
            far_lobe_settings.maximum_component_fraction
        ),
        "occupied_cell_count": far_lobe_plan.occupied_cell_count,
        "selected_occupied_cell_count": (
            far_lobe_plan.selected_occupied_cell_count
        ),
        "largest_occupied_cell_count": (
            far_lobe_plan.largest_occupied_cell_count
        ),
        "seed_distance_m": far_lobe_plan.seed_distance_m,
        "component_fraction": far_lobe_plan.component_fraction,
        "minimum_cell_center_separation_m": (
            far_lobe_plan.minimum_cell_center_separation_m
        ),
        "minimum_point_separation_lower_bound_m": (
            far_lobe_plan.minimum_point_separation_lower_bound_m
        ),
        "removed_count": removed_source_count,
        "source_count_before": source_info.count,
        "surviving_source_count": source_info.count - removed_source_count,
        "surviving_source_payload_byte_exact": True,
        "surviving_source_row_order_preserved": True,
        "archive": far_lobe_archive,
    }

    manifest = {
        "version": 1,
        "algorithm": "site1-v12-fine-first-supported-water-interface-v1",
        "candidate_only": True,
        "canonical_install_performed": False,
        "annotations_are_seed_evidence_only": True,
        "existing_payload_byte_exact": True,
        "geometry_provenance": "byte-exact surviving source WATER with reversible detached-lobe cull, plus hash-verified v10 height/noise/analytic-normal additions, local 5mm sided terrain fit, and complete 1mm absolute-clearance audit",
        "source": source_fp,
        "candidate": candidate_fp,
        "config": config_fp,
        "reference_surface": reference_surface_provenance,
        "archive": str(archive_path.resolve()),
        "archive_sha256": sha256_path(archive_path),
        "addition_count": int(len(additions)),
        "component_membership": {
            "archive_key": "candidate_label",
            "accepted_labels": accepted_labels,
            "all_additions_assigned_to_accepted_component": True,
        },
        "holes": holes,
        "parameters": {
            "terrain_absolute_clearance_m": terrain_clearance,
            "terrain_float32_tolerance_m": clearance_tolerance,
            "water_selection_radius_m": water_spacing,
            "density_repair_reservoir_margin_m": density_settings.reservoir_margin_m,
            "density_repair_support_margin_m": density_settings.support_margin_m,
            "fine_first": True,
            "coarse_recomputation_allowed": False,
        },
        "proposal_audit": {
            "raw_count": int(raw_proposal_count),
            "inside_verified_water_footprint": int(np.count_nonzero(inside)),
            "supported_after_footprint": int(np.count_nonzero(supported)),
            "after_clustered_fade": int(np.count_nonzero(correlated_keep)),
            "after_local_terrain_gate": int(np.count_nonzero(terrain_keep)),
            "safe_reservoir_before_complete_clearance": raw_reservoir_count,
            "safe_reservoir_after_complete_clearance": int(len(reservoir_xy)),
            "after_water_blue_noise": int(len(selected)),
            "after_measured_primary_refill": int(after_primary_refill_count),
            "after_joint_density_repack": int(len(proposals)),
            "after_complete_fine_terrain_clearance": int(len(additions)),
            "clustered_fade": fade_audit,
            "measured_continuity_refill": dip_refill_audit,
            "fine_terrain_clearance": {
                "complete_preselection_reservoir": (
                    reservoir_fine_clearance_audit
                ),
                "repair_reservoir": repair_clearance_audit,
                "all_clearance_neighbours_enumerated": True,
            },
            "coarse_terrain_sources": terrain_source_fps,
        },
        "density_audit": {
            "method": (
                "measured-water-density-times-vacant-support-area-v3"
            ),
            "circle_radius_m": density_settings.audit_radius_m,
            "step_m": density_settings.audit_step_m,
            "centres_xy": audit_centres.tolist(),
            "centre_spec_id": list(audit_contract.spec_id),
            "centre_spec_kind": list(audit_contract.spec_kind),
            "centre_spec_label": audit_contract.spec_label.tolist(),
            "active_spec_kinds": sorted(DENSITY_CONTINUITY_KINDS),
            "required_mask": required_mask.tolist(),
            "density_eligibility_rule": (
                "all=exact_exclusion_aware_reference_footprint_intersection"
                "&genuinely_vacant_safe_support_after_immutable_water_clearance"
            ),
            "required_count_by_spec": required_count_by_spec,
            "reference_surface_active_mask": (
                reference_surface_active.tolist()
            ),
            "source_water_active_mask": source_water_active.tolist(),
            "source_support_active_mask": source_support_active.tolist(),
            "fillable_support_active_mask": fillable_support_active.tolist(),
            "vacant_support_active_mask": fillable_support_active.tolist(),
            "support_sampling_pitch_m": support_sample_pitch,
            "support_sample_cell_area_m2": (
                raw_fillable_cells.cell_area_m2
            ),
            "footprint_full_disk_sample_count": (
                footprint_support.full_disk_sample_count
            ),
            "valid_footprint_sample_count": (
                footprint_support.valid_footprint_sample_count.tolist()
            ),
            "valid_footprint_area_m2": (
                footprint_support.valid_footprint_area_m2.tolist()
            ),
            "raw_support_sample_count": raw_support_sample_count.tolist(),
            "raw_support_area_m2": raw_support_area.tolist(),
            "vacant_support_sample_count": vacant_support_sample_count.tolist(),
            "vacant_support_area_m2": vacant_support_area.tolist(),
            "raw_support_cell_count": int(len(raw_fillable_cells.cell_keys)),
            "vacant_support_cell_count": int(len(vacant_support_cells.cell_keys)),
            "raw_support_archive_key": "raw_support_cell_keys",
            "raw_support_representative_archive_key": (
                "raw_support_representative_xy"
            ),
            "vacant_support_archive_key": "vacant_support_cell_keys",
            "vacant_support_representative_archive_key": (
                "vacant_support_representative_xy"
            ),
            "vacant_safe_reservoir_archive_key": "vacant_safe_reservoir_xy",
            "vacant_safe_reservoir_count": int(
                len(all_vacant_reservoir_indices)
            ),
            "support_pitch_archive_key": "support_pitch_m",
            "immutable_water_spacing_m": water_spacing,
            "immutable_water_blocker_count": int(len(water_xy)),
            "all_surviving_immutable_water_rows_block_placement": True,
            "reference_water_density_per_m2": (
                measured_targets.reference_water_density_per_m2.tolist()
            ),
            "local_reference_centres_xy": local_reference_centres.tolist(),
            "local_reference_water_count": (
                measured_targets.local_reference_water_count.tolist()
            ),
            "local_reference_terrain_count": (
                measured_targets.local_reference_terrain_count.tolist()
            ),
            "local_reference_water_area_m2": (
                measured_targets.local_reference_water_area_m2.tolist()
            ),
            "good_overlap_reference_centres_xy": (
                overlap_reference_centres.tolist()
            ),
            "good_overlap_reference_water_count": (
                measured_targets.good_overlap_water_count.tolist()
            ),
            "good_overlap_reference_terrain_count": (
                measured_targets.good_overlap_terrain_count.tolist()
            ),
            "good_overlap_reference_water_area_m2": (
                measured_targets.good_overlap_reference_water_area_m2.tolist()
            ),
            "spacing_feasible_capacity_count": (
                spacing_capacity_count.tolist()
            ),
            "spacing_capacity_selection_count": int(len(spacing_capacity_xy)),
            "spacing_capacity_fixed_fade_blocker_count": int(
                len(fixed_fade_xy)
            ),
            "spacing_capacity_seed": int(seed ^ 0xCA9AC17),
            "spacing_capacity_archive_key": "spacing_capacity_xy",
            "spacing_capacity_uses_complete_safe_reservoir_and_fixed_fade": True,
            "spacing_capacity_is_spacing_feasible_reservoir_not_interval_certificate": True,
            "spacing_capacity_candidate_rows_in_joint_pool": (
                capacity_candidate_rows_in_joint_pool
            ),
            "final_selected_additions_are_joint_interval_certificate": True,
            "raw_desired_addition_count": (
                measured_targets.raw_desired_addition_count.tolist()
            ),
            "target_addition_count": (
                addition_contract.target_addition_count.tolist()
            ),
            "addition_lower_count": (
                addition_contract.addition_lower_count.tolist()
            ),
            "addition_upper_count": (
                addition_contract.addition_upper_count.tolist()
            ),
            "capacity_sufficient_mask": (
                addition_contract.capacity_sufficient_mask.tolist()
            ),
            "addition_bounds_rounding": "ceil-both-on-discrete-point-count-lattice",
            "immutable_source_water_count": (
                addition_contract.immutable_water_count.tolist()
            ),
            "target_water_count": target_water_count.tolist(),
            "target_combined_count": target_combined_count.tolist(),
            "target_water_density_per_m2": (
                target_water_density_per_m2.tolist()
            ),
            "target_combined_density_per_m2": (
                target_combined_density_per_m2.tolist()
            ),
            "terrain_count": measured_targets.terrain_count.tolist(),
            "terrain_outer_count": terrain_outer_count.tolist(),
            "terrain_boundary_mask": terrain_boundary_mask.tolist(),
            "terrain_boundary_outer_radius_m": terrain_boundary_outer_radius,
            "shoreline_mask": measured_targets.shoreline_mask.tolist(),
            "shoreline_terrain_count_threshold": (
                measured_targets.shoreline_terrain_count_threshold
            ),
            "reference_kind": list(measured_targets.reference_kind),
            "reference_sample_count": (
                measured_targets.reference_sample_count.tolist()
            ),
            "minimum_ratio": density_settings.minimum_ratio,
            "maximum_ratio": density_settings.maximum_ratio,
            "repair_reservoir_margin_m": density_settings.reservoir_margin_m,
            "repair_support_margin_m": density_settings.support_margin_m,
            "repair_reservoir_count": (
                joint_refill.candidate_count_per_centre.tolist()
            ),
            "repair_reservoir_selected_count": repair_selected_count.tolist(),
            "wider_repair_before_complete_clearance_count": (
                repair_raw_reservoir_count.tolist()
            ),
            "wider_repair_after_complete_clearance_count": (
                repair_reservoir_count.tolist()
            ),
            "primary_clear_candidate_count": primary_cleared_count.tolist(),
            "provisional_primary_selected_count": (
                provisional_primary_selected_count.tolist()
            ),
            "spacing_blocked_candidate_count": (
                joint_refill.spacing_blocked_count_per_centre.tolist()
            ),
            "upper_blocked_candidate_count": (
                joint_refill.upper_blocked_count_per_centre.tolist()
            ),
            "combined_lower_count": combined_lower_count.tolist(),
            "combined_nominal_upper_count": (
                combined_nominal_upper_count.tolist()
            ),
            "combined_upper_count": combined_upper_count.tolist(),
            "water_lower_count": water_lower_count.tolist(),
            "water_nominal_upper_count": (
                water_nominal_upper_count.tolist()
            ),
            "water_upper_count": water_upper_count.tolist(),
            "water_before_count": (
                measured_targets.existing_water_count.tolist()
            ),
            "water_after_count": final_water_count.tolist(),
            "immutable_source_upper_grandfather_mask": (
                immutable_source_upper_grandfather_mask.tolist()
            ),
            "immutable_source_upper_grandfather_count": int(
                np.count_nonzero(immutable_source_upper_grandfather_mask)
            ),
            "allowed_combined_upper_count": allowed_combined_upper.tolist(),
            "combined_before_count": before_combined.observed_count.tolist(),
            "combined_after_primary_count": initial_combined.observed_count.tolist(),
            "combined_after_count": after_combined.observed_count.tolist(),
            "combined_before_ratio": before_combined.density_ratio.tolist(),
            "combined_after_primary_ratio": initial_combined.density_ratio.tolist(),
            "combined_after_ratio": after_combined.density_ratio.tolist(),
            "initial_lower_violations": int(np.count_nonzero(initial_lower)),
            "unresolved_lower_after": int(np.count_nonzero(unresolved_lower)),
            "upper_violations_after": int(np.count_nonzero(upper_violation)),
            "new_upper_violations": int(np.count_nonzero(upper_violation)),
            "lower_and_allowed_upper_bounds_enforced": True,
            "strict_nominal_upper_ratio_enforced_for_non_grandfathered": True,
            "grandfathered_windows_cannot_increase": True,
            "water_only_center_count_is_acceptance_criterion": True,
            "uses_overlapping_circular_windows": True,
        },
        "append_audit": append_audit,
        "implementation_sha256": implementation_hashes,
        "far_lobe_cull": far_lobe_audit,
    }
    manifest_path = destination / "manifest.json"
    _atomic_json(manifest_path, manifest)
    log(
        "v12 geometry complete: "
        f"{source_info.count:,} - {removed_source_count:,} + "
        f"{len(additions):,} = {candidate_fp['points']:,} WATER rows"
    )
    return GeometryBuildResult(
        candidate_path=str(candidate_path),
        archive_path=str(archive_path),
        manifest_path=str(manifest_path),
        source_points=source_info.count,
        addition_count=len(additions),
        candidate_points=int(candidate_fp["points"]),
        candidate_sha256=str(candidate_fp["sha256"]),
    )


__all__ = [
    "CircleSpec",
    "FarLobeCullSettings",
    "FarLobeGridPlan",
    "GeometryBuildResult",
    "LocalRecords",
    "append_records_with_comments",
    "build_fine_water_geometry",
    "collect_records_in_circles",
    "generate_review_proposals",
    "load_circle_specs",
    "sha256_path",
]
