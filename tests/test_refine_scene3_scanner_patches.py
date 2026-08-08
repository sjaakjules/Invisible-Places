from __future__ import annotations

import hashlib
import importlib.util
import json
import os
import sys
import tempfile
import types
import unittest
from unittest import mock
from pathlib import Path

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = REPO_ROOT / "scripts" / "refine_scene3_scanner_patches.py"
SPEC = importlib.util.spec_from_file_location("refine_scene3_scanner_patches", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
patches = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = patches
SPEC.loader.exec_module(patches)


PROPERTIES = [
    ("float", "x", "<f4"), ("float", "y", "<f4"), ("float", "z", "<f4"),
    ("uchar", "red", "u1"), ("uchar", "green", "u1"), ("uchar", "blue", "u1"),
    ("float", "nx", "<f4"), ("float", "ny", "<f4"), ("float", "nz", "<f4"),
    ("double", "scalar_Intensity", "<f8"),
    ("float", "scalar_Ranges", "<f4"),
    ("float", "scalar_Composite", "<f4"),
    ("float", "scalar_ScanID", "<f4"),
    ("float", "scalar_Roughness", "<f4"),
    ("float", "scalar_Interest", "<f4"),
    ("float", "scalar_A_R_TestA", "<f4"),
    ("float", "scalar_A_R_TestB", "<f4"),
]
DTYPE = np.dtype([(name, dtype) for _, name, dtype in PROPERTIES], align=False)


def write_ply(path: Path, data: np.ndarray) -> None:
    lines = ["ply", "format binary_little_endian 1.0", f"element vertex {len(data)}"]
    lines.extend(f"property {kind} {name}" for kind, name, _ in PROPERTIES)
    lines.append("end_header")
    path.write_bytes(("\n".join(lines) + "\n").encode("ascii") + data.tobytes())


def point(x: float, y: float, z: float, scan_id: int, base: float) -> np.void:
    value = np.zeros(1, dtype=DTYPE)
    value["x"], value["y"], value["z"] = x, y, z
    value["red"], value["green"], value["blue"] = 100, 120, 140
    value["nz"] = 1.0
    value["scalar_Intensity"] = base
    value["scalar_Ranges"] = base + 1
    value["scalar_Composite"] = base + 2
    value["scalar_ScanID"] = scan_id
    value["scalar_Roughness"] = base + 3
    value["scalar_Interest"] = base + 4
    value["scalar_A_R_TestA"] = base + 5
    value["scalar_A_R_TestB"] = base + 6
    return value[0]


def component_for(points: np.ndarray) -> patches.ComponentSupport:
    ix = int(np.floor(float(points[0]["x"]) / patches.COMPONENT_CELL_METRES))
    iy = int(np.floor(float(points[0]["y"]) / patches.COMPONENT_CELL_METRES))
    mask = np.zeros((3, 3), dtype=bool)
    mask[1, 1] = True
    outside = np.ones((3, 3), dtype=bool)
    outside[1, 1] = False
    return patches.ComponentSupport(
        camera_name="Patch 01",
        cleanmesh_index=1,
        host_scan_id=3,
        bounds_min=np.array([0.0, 0.0, 0.0]),
        bounds_max=np.array([0.02, 0.02, 0.02]),
        dense_records=np.empty(0, dtype=DTYPE),
        cell_keys=patches._pack_cells(np.array([ix]), np.array([iy])),
        cell_z_min=np.array([0.0]),
        cell_z_max=np.array([0.02]),
        origin_ix=ix - 1,
        origin_iy=iy - 1,
        mask=mask,
        boundary=mask.copy(),
        outside_ring=outside,
    )


def identity_model(component: patches.ComponentSupport, source_id: int) -> patches.CorrectionModel:
    return patches.CorrectionModel(
        component_index=component.cleanmesh_index,
        source_scan_id=source_id,
        host_scan_id=component.host_scan_id,
        gain=np.ones(6),
        offset=np.zeros(6),
        residual_field=np.zeros((*component.mask.shape, 6)),
        clip_low=np.full(6, -np.inf),
        clip_high=np.full(6, np.inf),
        pair_source=np.empty((0, 6)),
        pair_target=np.empty((0, 6)),
        metrics={},
    )


def deferred_install_fixture(
    root: Path,
) -> tuple[dict[int, bytes], dict[int, bytes], Path]:
    originals: dict[int, bytes] = {}
    candidates: dict[int, bytes] = {}
    entries = []
    for spacing_mm in patches.SPACINGS_MM:
        source_path = root / f"Site3-SAND-{spacing_mm}mm.ply"
        source = np.array([
            point(0.001 * spacing_mm, 0.000, 0.000, 1, 10 + spacing_mm),
            point(0.002 * spacing_mm, 0.001, 0.000, 7, 20 + spacing_mm),
        ], dtype=DTYPE)
        write_ply(source_path, source)
        originals[spacing_mm] = source_path.read_bytes()
        info = patches.read_ply_info(source_path)
        addition = np.array([
            point(0.003 * spacing_mm, 0.002, 0.000, 13, 30 + spacing_mm),
        ], dtype=DTYPE)
        entry = patches.build_append_only_density_candidate(
            spacing_mm,
            info,
            addition,
            root / f"SAND-{spacing_mm}mm",
            2,
        )
        candidates[spacing_mm] = Path(entry["candidate_path"]).read_bytes()
        entries.append(entry)

    candidate_report = root / "validation-render" / "scene3-patch-boundaries.json"
    candidate_report.parent.mkdir()
    candidate_report.write_text(json.dumps({
        "scenario": "scene3-patch-boundaries",
        "passed": True,
        "failures": [],
    }))
    patches._write_json(root / "manifest.json", {
        "verified": True,
        "installed": False,
        "status": "verified",
        "repo_root": str(root),
        "project": str(root / "project.json"),
        "densities": entries,
    })
    return originals, candidates, candidate_report


class ScannerPatchRefinementTests(unittest.TestCase):
    def test_patch_04_jagged_profile_is_deterministic_connected_and_bounded(self) -> None:
        height, width = 5, 90
        mask = np.ones((height, width), dtype=bool)
        component = patches.ComponentSupport(
            camera_name="Patch 04",
            cleanmesh_index=4,
            host_scan_id=9,
            bounds_min=np.array([0.0, 0.0, 0.0]),
            bounds_max=np.array([width * patches.COMPONENT_CELL_METRES, 0.1, 0.02]),
            dense_records=np.empty(0, dtype=DTYPE),
            cell_keys=np.empty(0, dtype=np.int64),
            cell_z_min=np.empty(0),
            cell_z_max=np.empty(0),
            origin_ix=0,
            origin_iy=0,
            mask=mask,
            boundary=np.zeros_like(mask),
            outside_ring=np.zeros_like(mask),
        )
        along = np.tile(
            np.arange(width, dtype=np.float64) * patches.COMPONENT_CELL_METRES,
            (height, 1),
        )
        support = patches.LinearEdgeSupport(
            component=component,
            source_scan_id=9,
            edge_origin_xy=np.array([0.0, 0.0]),
            edge_direction_xy=np.array([1.0, 0.0]),
            outside_normal_xy=np.array([0.0, 1.0]),
            signed_distance=np.tile(np.arange(height)[:, None] * 0.02, (1, width)),
            along_distance=along,
            high_density_mask=np.zeros_like(mask),
            transition_mask=mask,
            along_min=0.0,
            along_max=float(np.max(along)),
            blend_width_metres=0.44,
            fit_rms_metres=0.001,
        )
        samples = []
        for y in range(height):
            for x in range(width):
                value = point(
                    (x + 0.5) * patches.COMPONENT_CELL_METRES,
                    (y + 0.5) * patches.COMPONENT_CELL_METRES,
                    0.0,
                    2,
                    1.0,
                )
                value["nx"], value["ny"], value["nz"] = 0.0, 0.0, 1.0
                samples.append(value)
        points = np.array(samples, dtype=DTYPE)
        info = types.SimpleNamespace(fields={
            "x": "x", "y": "y", "nx": "nx", "ny": "ny", "nz": "nz",
        })

        report = patches.configure_patch04_jagged_fade(
            support,
            points,
            info,
            minimum_width_metres=0.08,
            base_width_metres=0.20,
            maximum_width_metres=0.44,
            core_width_metres=0.03,
            noise_seed=12345,
        )
        first = np.array(support.fade_width_map, copy=True)
        patches.configure_patch04_jagged_fade(
            support,
            points,
            info,
            minimum_width_metres=0.08,
            base_width_metres=0.20,
            maximum_width_metres=0.44,
            core_width_metres=0.03,
            noise_seed=12345,
        )

        np.testing.assert_array_equal(first, support.fade_width_map)
        self.assertGreater(float(np.ptp(first)), 0.05)
        self.assertGreaterEqual(float(np.min(first)), 0.08)
        self.assertLessEqual(float(np.max(first)), 0.44)
        self.assertLess(float(np.max(np.abs(np.diff(first[2])))), 0.08)
        self.assertEqual(report["profile"], "connected-jagged")

    def test_patch_04_lobed_profile_has_three_plateaus_and_broad_outer_fades(self) -> None:
        height, width = 34, 150
        mask = np.ones((height, width), dtype=bool)
        component = patches.ComponentSupport(
            camera_name="Patch 04",
            cleanmesh_index=4,
            host_scan_id=9,
            bounds_min=np.array([-1.5, 0.0, 0.0]),
            bounds_max=np.array([1.5, height * patches.COMPONENT_CELL_METRES, 0.02]),
            dense_records=np.empty(0, dtype=DTYPE),
            cell_keys=np.empty(0, dtype=np.int64),
            cell_z_min=np.empty(0),
            cell_z_max=np.empty(0),
            origin_ix=-75,
            origin_iy=0,
            mask=mask,
            boundary=np.zeros_like(mask),
            outside_ring=np.zeros_like(mask),
        )
        along_axis = (
            np.arange(width, dtype=np.float64) - 75.0
        ) * patches.COMPONENT_CELL_METRES
        along = np.tile(along_axis, (height, 1))
        signed = np.tile(
            np.arange(height, dtype=np.float64)[:, None]
            * patches.COMPONENT_CELL_METRES,
            (1, width),
        )
        support = patches.LinearEdgeSupport(
            component=component,
            source_scan_id=9,
            edge_origin_xy=np.array([0.0, 0.0]),
            edge_direction_xy=np.array([1.0, 0.0]),
            outside_normal_xy=np.array([0.0, 1.0]),
            signed_distance=signed,
            along_distance=along,
            high_density_mask=np.zeros_like(mask),
            transition_mask=mask,
            along_min=float(np.min(along)),
            along_max=float(np.max(along)),
            blend_width_metres=patches.PATCH_04_LOBED_MAX_WIDTH_METRES,
            fit_rms_metres=0.001,
        )
        samples = []
        for y in range(height):
            for x in range(width):
                value = point(
                    (x - 75 + 0.5) * patches.COMPONENT_CELL_METRES,
                    (y + 0.5) * patches.COMPONENT_CELL_METRES,
                    0.0,
                    2,
                    1.0,
                )
                value["nx"], value["ny"], value["nz"] = 0.0, 0.0, 1.0
                samples.append(value)
        points = np.array(samples, dtype=DTYPE)
        info = types.SimpleNamespace(fields={
            "x": "x", "y": "y", "nx": "nx", "ny": "ny", "nz": "nz",
        })

        report = patches.configure_patch04_lobed_fade(
            support,
            points,
            info,
            maximum_width_metres=patches.PATCH_04_LOBED_MAX_WIDTH_METRES,
            noise_seed=12345,
        )
        first_core = np.array(support.core_width_map, copy=True)
        first_fade = np.array(support.fade_width_map, copy=True)
        patches.configure_patch04_lobed_fade(
            support,
            points,
            info,
            maximum_width_metres=patches.PATCH_04_LOBED_MAX_WIDTH_METRES,
            noise_seed=12345,
        )

        np.testing.assert_array_equal(first_core, support.core_width_map)
        np.testing.assert_array_equal(first_fade, support.fade_width_map)
        centre_row = height // 2
        sample = lambda location: int(np.argmin(np.abs(along_axis - location)))
        self.assertGreater(first_core[centre_row, sample(-0.72)], 0.05)
        self.assertLess(first_core[centre_row, sample(-0.58)], 0.01)
        self.assertGreater(first_core[centre_row, sample(-0.46)], 0.05)
        self.assertLess(first_core[centre_row, sample(-0.34)], 0.01)
        self.assertGreater(first_core[centre_row, sample(-0.08)], 0.20)
        self.assertGreater(first_core[centre_row, sample(0.38)], 0.14)
        self.assertGreater(
            first_fade[centre_row, sample(-0.08)]
            - first_core[centre_row, sample(-0.08)],
            0.20,
        )
        self.assertGreaterEqual(report["plateau_island_count"], 3)
        self.assertEqual(report["profile"], "reviewed-lobed-plateaus")
        self.assertEqual(
            report["quota_model"],
            "cellwise total-density deficit to the ScanID 9 reference",
        )

    def test_patch_04_sealed_profile_fills_empty_actual_edge_cells_on_five_mm_grid(self) -> None:
        height, width = 12, 24
        cell = patches.PATCH_04_CELL_METRES
        transition = np.zeros((height, width), dtype=bool)
        transition[2:, :] = True
        high_density = ~transition
        component = patches.ComponentSupport(
            camera_name="Patch 04_new",
            cleanmesh_index=4,
            host_scan_id=9,
            bounds_min=np.array([0.0, 0.0, 0.0]),
            bounds_max=np.array([width * cell, height * cell, 0.02]),
            dense_records=np.empty(0, dtype=DTYPE),
            cell_keys=np.empty(0, dtype=np.int64),
            cell_z_min=np.empty(0),
            cell_z_max=np.empty(0),
            origin_ix=0,
            origin_iy=0,
            mask=transition,
            boundary=np.zeros_like(transition),
            outside_ring=np.zeros_like(transition),
            cell_metres=cell,
        )
        along_axis = (np.arange(width) - width / 2) * cell
        along = np.tile(along_axis, (height, 1))
        signed_rows = np.array([-0.08, -0.04] + [index * cell for index in range(1, 11)])
        signed = np.tile(signed_rows[:, None], (1, width))
        edge_distance = np.tile(
            np.array([0.0, 0.0] + [index * cell for index in range(1, 11)])[:, None],
            (1, width),
        )
        support = patches.LinearEdgeSupport(
            component=component,
            source_scan_id=9,
            edge_origin_xy=np.array([0.0, 0.0]),
            edge_direction_xy=np.array([1.0, 0.0]),
            outside_normal_xy=np.array([0.0, 1.0]),
            signed_distance=signed,
            along_distance=along,
            high_density_mask=high_density,
            transition_mask=transition,
            along_min=float(np.min(along)),
            along_max=float(np.max(along)),
            blend_width_metres=0.05,
            fit_rms_metres=0.001,
            edge_distance=edge_distance,
            edge_profile="sealed",
            fade_width_map=np.full((height, width), 0.05),
            core_width_map=np.full((height, width), 0.02),
            noise_seed=123,
        )
        samples = []
        empty_cell = (2, width // 2)
        for iy in range(height):
            for ix in range(width):
                if (iy, ix) == empty_cell:
                    continue
                count = 18 if iy < 2 else (5 if iy >= 8 else 8)
                scan_id = 9 if iy < 2 else 2
                for sample_index in range(count):
                    samples.append(point(
                        (ix + 0.15 + sample_index * 0.001) * cell,
                        (iy + 0.25) * cell,
                        0.01,
                        scan_id,
                        1.0,
                    ))
        info = types.SimpleNamespace(fields={
            "x": "x", "y": "y", "z": "z", "scan_id": "scalar_ScanID",
        })
        requested, report = patches.plan_patch04_edge_additions(
            np.array(samples, dtype=DTYPE),
            info,
            support,
        )
        empty_key = int(patches._pack_cells(
            np.array([empty_cell[1]]),
            np.array([empty_cell[0]]),
        )[0])
        self.assertGreater(requested.get(empty_key, 0), 0)
        self.assertEqual(report["density_target"], "scan9-sealed-actual-edge-ramp")
        self.assertEqual(report["planning_cell_m"], cell)
        self.assertGreater(report["empty_transition_cell_count"], 0)

    def test_patch_04_contact_distance_uses_raw_scan_cells_not_blurred_mask(self) -> None:
        cell = patches.PATCH_04_CELL_METRES
        id9 = np.zeros((9, 12), dtype=np.int64)
        id9[:4, 2:10] = 12
        # The fitted density footprint has expanded two empty rows beyond the
        # final measured ScanID 9 cells, reproducing the former visible moat.
        high_density = np.zeros_like(id9, dtype=bool)
        high_density[:6, 2:10] = True

        contact, boundary, distance = patches._patch04_contact_geometry(
            id9,
            high_density,
            cell,
        )

        self.assertTrue(contact[3, 5])
        self.assertFalse(contact[4, 5])
        self.assertTrue(high_density[4, 5])
        self.assertTrue(boundary[3, 5])
        self.assertAlmostEqual(float(distance[4, 5]), cell, places=7)

    def test_patch_04_contact_geometry_ignores_internal_scan_holes(self) -> None:
        cell = patches.PATCH_04_CELL_METRES
        id9 = np.zeros((10, 12), dtype=np.int64)
        id9[1:8, 1:11] = 12
        id9[3:6, 4:7] = 0
        high_density = np.zeros_like(id9, dtype=bool)
        high_density[:9, :12] = True

        contact, boundary, distance = patches._patch04_contact_geometry(
            id9,
            high_density,
            cell,
        )

        self.assertTrue(contact[4, 5])
        self.assertFalse(boundary[4, 5])
        self.assertEqual(float(distance[4, 5]), 0.0)
        self.assertGreater(np.count_nonzero(boundary & (id9 > 0)), 0)

    def test_patch_04_contact_audit_matches_measured_boundary_spacing(self) -> None:
        cell = patches.PATCH_04_CELL_METRES
        shape = (4, 5)
        transition = np.zeros(shape, dtype=bool)
        transition[2:, :] = True
        contact = np.zeros(shape, dtype=bool)
        contact[1, :] = True
        component = patches.ComponentSupport(
            camera_name="Patch 04_new",
            cleanmesh_index=4,
            host_scan_id=9,
            bounds_min=np.array([0.0, 0.0, 0.0]),
            bounds_max=np.array([shape[1] * cell, shape[0] * cell, 0.02]),
            dense_records=np.empty(0, dtype=DTYPE),
            cell_keys=np.empty(0, dtype=np.int64),
            cell_z_min=np.empty(0),
            cell_z_max=np.empty(0),
            origin_ix=0,
            origin_iy=0,
            mask=transition,
            boundary=contact,
            outside_ring=np.zeros(shape, dtype=bool),
            cell_metres=cell,
        )
        support = patches.LinearEdgeSupport(
            component=component,
            source_scan_id=9,
            edge_origin_xy=np.array([0.0, 0.0]),
            edge_direction_xy=np.array([1.0, 0.0]),
            outside_normal_xy=np.array([0.0, 1.0]),
            signed_distance=np.zeros(shape),
            along_distance=np.zeros(shape),
            high_density_mask=contact,
            transition_mask=transition,
            along_min=0.0,
            along_max=1.0,
            blend_width_metres=0.05,
            fit_rms_metres=0.001,
            edge_distance=np.zeros(shape),
            contact_source_mask=contact,
            contact_boundary_mask=contact,
            edge_profile="sealed",
        )
        measured = []
        additions = []
        for ix in range(shape[1]):
            x = (ix + 0.35) * cell
            measured.extend((
                point(x, 0.009, 0.0, 9, 1.0),
                point(x + 0.001, 0.009, 0.0, 9, 1.0),
            ))
            additions.append(point(x, 0.010, 0.0, 13, 1.0))
        measured_array = np.array(measured * 12, dtype=DTYPE)
        # Offset repeated copies slightly in Z so FLANN does not see exact
        # duplicates while still measuring the intended 1 mm sampling scale.
        for index in range(len(measured_array)):
            measured_array[index]["z"] += (index // len(measured)) * 0.001
        additions_array = np.array(additions, dtype=DTYPE)
        info = types.SimpleNamespace(fields={
            "x": "x", "y": "y", "z": "z", "scan_id": "scalar_ScanID",
        })

        target, spacing = patches.estimate_patch04_contact_spacing(
            measured_array,
            info,
            support,
        )
        audit = patches.audit_patch04_contact_spacing(
            additions_array,
            info,
            measured_array,
            info,
            support,
            target,
        )

        self.assertAlmostEqual(target, 0.001, places=6)
        self.assertAlmostEqual(spacing["nearest_spacing_p50_m"], 0.001, places=6)
        self.assertAlmostEqual(audit["contact_distance_p50_m"], 0.001, places=6)
        self.assertEqual(audit["contact_fraction"], 1.0)

    def test_patch_04_3d_contact_bridge_closes_cell_quota_gap(self) -> None:
        cell = patches.PATCH_04_CELL_METRES
        shape = (6, 10)
        transition = np.zeros(shape, dtype=bool)
        transition[3:, :] = True
        contact_source = np.zeros(shape, dtype=bool)
        contact_source[:3, :] = True
        contact_boundary = np.zeros(shape, dtype=bool)
        contact_boundary[2, :] = True
        edge_distance = np.zeros(shape, dtype=np.float64)
        edge_distance[3, :] = cell
        edge_distance[4, :] = 2 * cell
        edge_distance[5, :] = 3 * cell
        component = patches.ComponentSupport(
            camera_name="Patch 04_new",
            cleanmesh_index=4,
            host_scan_id=9,
            bounds_min=np.array([0.0, 0.0, 0.0]),
            bounds_max=np.array([shape[1] * cell, shape[0] * cell, 0.02]),
            dense_records=np.empty(0, dtype=DTYPE),
            cell_keys=np.empty(0, dtype=np.int64),
            cell_z_min=np.empty(0),
            cell_z_max=np.empty(0),
            origin_ix=0,
            origin_iy=0,
            mask=transition,
            boundary=contact_boundary,
            outside_ring=np.zeros(shape, dtype=bool),
            cell_metres=cell,
        )
        support = patches.LinearEdgeSupport(
            component=component,
            source_scan_id=9,
            edge_origin_xy=np.array([0.0, 0.0]),
            edge_direction_xy=np.array([1.0, 0.0]),
            outside_normal_xy=np.array([0.0, 1.0]),
            signed_distance=np.zeros(shape),
            along_distance=np.zeros(shape),
            high_density_mask=contact_source,
            transition_mask=transition,
            along_min=0.0,
            along_max=1.0,
            blend_width_metres=0.05,
            fit_rms_metres=0.001,
            edge_distance=edge_distance,
            contact_source_mask=contact_source,
            contact_boundary_mask=contact_boundary,
            edge_profile="sealed",
        )
        local = np.array([
            point((ix + 0.25) * cell, 0.0140, 0.0, 9, 1.0)
            for ix in range(1, shape[1] - 1)
        ], dtype=DTYPE)
        selected = np.array([
            point((ix + 0.25) * cell, 0.0200, 0.0, 13, 1.0)
            for ix in range(1, shape[1] - 1)
        ], dtype=DTYPE)
        candidates = np.array([
            # The true edge lies inside the 5 mm planning cell, so a valid
            # bridge sample may share the measured boundary cell.
            point((ix + 0.25) * cell, 0.0149, 0.0, 11, 1.0)
            for ix in range(1, shape[1] - 1)
        ], dtype=DTYPE)
        info = types.SimpleNamespace(fields={
            "x": "x", "y": "y", "z": "z", "scan_id": "scalar_ScanID",
        })

        combined, report = patches.seal_patch04_contact(
            selected,
            info,
            candidates,
            info,
            local,
            info,
            support,
            0.001,
            0.001,
        )

        self.assertEqual(report["added"], len(candidates))
        self.assertEqual(len(combined), len(selected) + len(candidates))
        self.assertGreater(report["before"]["contact_distance_p50_m"], 0.005)
        self.assertAlmostEqual(report["after"]["contact_distance_p95_m"], 0.0009, places=6)
        self.assertEqual(report["after"]["point_contact_fraction"], 1.0)

    def test_tail_replacement_stages_new_id13_and_restores_prior_tail_exactly(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            base = np.array([
                point(0.000, 0.000, 0.000, 2, 10),
                point(0.004, 0.000, 0.000, 3, 20),
            ], dtype=DTYPE)
            old_tail = np.array([point(0.002, 0.002, 0.000, 13, 30)], dtype=DTYPE)
            current = np.concatenate((base, old_tail))
            base_path = root / "base.ply"
            current_path = root / "current.ply"
            write_ply(base_path, base)
            write_ply(current_path, current)
            prior_bytes = current_path.read_bytes()
            physical_base_info = patches.read_ply_info(base_path)
            current_info = patches.read_ply_info(current_path)
            base_info = patches.PlyInfo(
                path=current_path,
                header=physical_base_info.header,
                vertex_count=len(base),
                dtype=current_info.dtype,
                property_names=current_info.property_names,
                fields=current_info.fields,
            )
            installed_entry = {
                "source_vertex_count": len(base),
                "candidate_vertex_count": len(current),
                "addition_count": len(old_tail),
                "source_header_hex": physical_base_info.header.hex(),
                "source_sha256": patches.sha256_path(base_path),
                "candidate_sha256": patches.sha256_path(current_path),
                "property_names": list(current_info.property_names),
            }
            additions = np.array([
                point(0.001, 0.003, 0.000, 13, 40),
                point(0.003, 0.003, 0.000, 13, 50),
            ], dtype=DTYPE)
            entry = patches.build_tail_replacement_candidate(
                1,
                current_info,
                base_info,
                installed_entry,
                additions,
                root / "candidate",
                2,
                root / "installed-edge",
            )

            candidate_info = patches.read_ply_info(Path(entry["candidate_path"]))
            candidate = np.array(patches.records(candidate_info), copy=True)
            self.assertEqual(candidate_info.vertex_count, len(base) + len(additions))
            self.assertEqual(candidate[:len(base)].tobytes(), base.tobytes())
            self.assertEqual(candidate[len(base):].tobytes(), additions.tobytes())
            self.assertEqual(entry["superseded_addition_count"], len(old_tail))
            result = patches.verify_density_entry(entry, 2)
            self.assertTrue(result["passed"], result["failures"])

            os.replace(entry["candidate_path"], current_path)
            restored_path = root / "restored.ply"
            restored_hash = patches._restore_density(entry, restored_path, 2)
            self.assertEqual(restored_hash, entry["source_sha256"])
            self.assertEqual(restored_path.read_bytes(), prior_bytes)

    def test_tail_replacement_can_replace_an_installed_replacement_tail(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            base = np.array([
                point(0.000, 0.000, 0.000, 2, 10),
                point(0.004, 0.000, 0.000, 3, 20),
            ], dtype=DTYPE)
            first_tail = np.array([point(0.002, 0.002, 0.000, 13, 30)], dtype=DTYPE)
            current_path = root / "current.ply"
            base_path = root / "base.ply"
            write_ply(base_path, base)
            write_ply(current_path, np.concatenate((base, first_tail)))
            base_physical = patches.read_ply_info(base_path)
            current_info = patches.read_ply_info(current_path)
            base_info = patches.PlyInfo(
                path=current_path,
                header=base_physical.header,
                vertex_count=len(base),
                dtype=current_info.dtype,
                property_names=current_info.property_names,
                fields=current_info.fields,
            )
            first_entry = {
                "source_vertex_count": len(base),
                "candidate_vertex_count": len(base) + len(first_tail),
                "addition_count": len(first_tail),
                "source_header_hex": base_physical.header.hex(),
                "source_sha256": patches.sha256_path(base_path),
                "candidate_sha256": patches.sha256_path(current_path),
                "property_names": list(current_info.property_names),
            }
            second_tail = np.array([
                point(0.001, 0.003, 0.000, 13, 40),
                point(0.003, 0.003, 0.000, 13, 50),
            ], dtype=DTYPE)
            second_entry = patches.build_tail_replacement_candidate(
                1,
                current_info,
                base_info,
                first_entry,
                second_tail,
                root / "second",
                2,
                root / "first-run",
            )
            os.replace(second_entry["candidate_path"], current_path)
            installed_second_bytes = current_path.read_bytes()
            installed_second_info = patches.read_ply_info(current_path)

            chained_base = patches._prefix_info_from_installed_edge(
                installed_second_info,
                second_entry,
            )
            self.assertEqual(chained_base.vertex_count, len(base))
            self.assertEqual(chained_base.header, base_physical.header)
            third_tail = np.array([point(0.002, 0.004, 0.000, 13, 60)], dtype=DTYPE)
            third_entry = patches.build_tail_replacement_candidate(
                1,
                installed_second_info,
                chained_base,
                second_entry,
                third_tail,
                root / "third",
                2,
                root / "second-run",
            )
            result = patches.verify_density_entry(third_entry, 2)
            self.assertTrue(result["passed"], result["failures"])

            os.replace(third_entry["candidate_path"], current_path)
            restored_path = root / "restored-second.ply"
            restored_hash = patches._restore_density(
                third_entry,
                restored_path,
                2,
            )
            self.assertEqual(restored_hash, third_entry["source_sha256"])
            self.assertEqual(restored_path.read_bytes(), installed_second_bytes)

    def test_patch_04_edge_plan_tapers_additions_without_editing_existing_points(self) -> None:
        mask = np.array([[False, False, False, True, True, True]])
        component = patches.ComponentSupport(
            camera_name="Patch 04",
            cleanmesh_index=4,
            host_scan_id=9,
            bounds_min=np.array([0.0, 0.0, 0.0]),
            bounds_max=np.array([0.12, 0.02, 0.02]),
            dense_records=np.empty(0, dtype=DTYPE),
            cell_keys=np.empty(0, dtype=np.int64),
            cell_z_min=np.empty(0),
            cell_z_max=np.empty(0),
            origin_ix=0,
            origin_iy=0,
            mask=mask,
            boundary=np.array([[False, False, True, False, False, False]]),
            outside_ring=np.array([[False, False, False, False, False, True]]),
        )
        signed = np.array([[-0.10, -0.06, -0.02, 0.02, 0.10, 0.20]])
        support = patches.LinearEdgeSupport(
            component=component,
            source_scan_id=9,
            edge_origin_xy=np.array([0.06, 0.01]),
            edge_direction_xy=np.array([0.0, 1.0]),
            outside_normal_xy=np.array([1.0, 0.0]),
            signed_distance=signed,
            along_distance=np.zeros_like(signed),
            high_density_mask=~mask,
            transition_mask=mask,
            along_min=-1.0,
            along_max=1.0,
            blend_width_metres=0.22,
            fit_rms_metres=0.001,
        )
        samples = []
        for cell in range(6):
            count = 20 if cell < 3 else 10
            scan_id = 9 if cell < 3 else 2
            for index in range(count):
                samples.append(point(
                    (cell + 0.1 + index * 0.001) * patches.COMPONENT_CELL_METRES,
                    0.002,
                    0.01,
                    scan_id,
                    1.0,
                ))
        info = types.SimpleNamespace(fields={
            "x": "x",
            "y": "y",
            "scan_id": "scalar_ScanID",
        })

        requested, report = patches.plan_patch04_edge_additions(
            np.array(samples, dtype=DTYPE),
            info,
            support,
        )

        quotas = [requested[key] for key in sorted(requested)]
        self.assertGreater(len(quotas), 1)
        self.assertGreater(quotas[0], quotas[-1])
        self.assertEqual(report["density_target"], "scan9-linear-edge-ramp")
        self.assertEqual(report["existing_records_modified"], 0)

    def test_patch_04_full_transfer_blends_rgb_and_copies_all_scalars(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = np.array([
                point(0.000, 0.000, 0.000, 2, 10),
                point(0.004, 0.000, 0.000, 3, 20),
            ], dtype=DTYPE)
            source[0]["red"], source[0]["green"], source[0]["blue"] = 40, 60, 80
            source[1]["red"], source[1]["green"], source[1]["blue"] = 180, 160, 140
            path = root / "source.ply"
            write_ply(path, source)
            info = patches.read_ply_info(path)
            additions = np.array([
                point(0.001, 0.000, 0.000, 13, 0),
            ], dtype=DTYPE)
            additions["red"], additions["green"], additions["blue"] = 0, 0, 0

            transferred, report = patches.transfer_full_nearest_1mm(
                additions,
                info,
                source,
                info,
            )

            self.assertEqual(len(transferred), 1)
            self.assertEqual(int(round(float(transferred["scalar_ScanID"][0]))), 13)
            self.assertEqual(float(transferred["scalar_Intensity"][0]), 10.0)
            self.assertEqual(float(transferred["scalar_A_R_TestA"][0]), 15.0)
            self.assertGreater(int(transferred["red"][0]), 40)
            self.assertLess(int(transferred["red"][0]), 180)
            self.assertEqual(report["skipped"], 0)
            self.assertEqual(report["copied_scalar_field_count"], 7)
            self.assertIn("Oklab", report["rgb_source"])

    def test_compaction_keeps_only_the_data_needed_for_exact_rollback(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = np.array([
                point(0.000, 0.000, 0.000, 12, 10),
                point(0.004, 0.000, 0.000, 3, 20),
            ], dtype=DTYPE)
            source_path = root / "canonical.ply"
            write_ply(source_path, source)
            original_bytes = source_path.read_bytes()
            info = patches.read_ply_info(source_path)
            addition = np.array([
                point(0.002, 0.002, 0.000, 13, 40),
            ], dtype=DTYPE)
            entry = patches.build_append_only_density_candidate(
                1,
                info,
                addition,
                root / "1mm",
                2,
            )
            os.replace(entry["candidate_path"], source_path)
            validation = root / "validation-render"
            post_install = root / "post-install-render"
            validation.mkdir()
            post_install.mkdir()
            (validation / "scene3-patch-boundaries.json").write_text("{}")
            (post_install / "scene3-patch-boundaries.json").write_text("{}")
            (post_install / "patch_01.exr").write_bytes(b"duplicate-render")
            project = root / "validation-project.json"
            project.write_text("{}")
            patches._write_json(root / "manifest.json", {
                "installed": True,
                "status": "installed",
                "densities": [entry],
                "validation_project": str(project),
                "validation_data_root": str(root / "validation-data"),
            })
            with mock.patch.object(patches, "_application_running", return_value=False):
                result = patches.command_compact(types.SimpleNamespace(
                    run_dir=root,
                    keep_final_pngs=False,
                ))
            self.assertEqual(result, 0)
            self.assertTrue(Path(entry["originals_path"]).is_file())
            self.assertTrue(Path(entry["indices_path"]).is_file())
            self.assertFalse(Path(entry["replacements_path"]).exists())
            self.assertFalse(Path(entry["additions_path"]).exists())
            self.assertFalse(validation.exists())
            compacted = json.loads((root / "manifest.json").read_text())
            self.assertTrue(compacted["compacted"])
            restored = root / "restored.ply"
            self.assertEqual(
                patches._restore_density(entry, restored, 2),
                entry["source_sha256"],
            )
            self.assertEqual(restored.read_bytes(), original_bytes)

    def test_deferred_install_finalizes_only_after_external_smoke(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            originals, candidates, _ = deferred_install_fixture(root)
            with mock.patch.object(patches, "_application_running", return_value=False):
                result = patches.command_install(types.SimpleNamespace(
                    run_dir=root,
                    candidate_smoke_report=None,
                    defer_post_install_smoke=True,
                    app=root / "unused-app",
                ))
            self.assertEqual(result, 0)

            journal = json.loads((root / "install-journal.json").read_text())
            self.assertEqual(journal["state"], "awaiting-external-smoke")
            for spacing_mm, state in zip(patches.SPACINGS_MM, journal["files"], strict=True):
                canonical = root / f"Site3-SAND-{spacing_mm}mm.ply"
                self.assertEqual(canonical.read_bytes(), candidates[spacing_mm])
                self.assertFalse(Path(state["candidate"]).exists())
                self.assertEqual(
                    Path(state["temporary_original"]).read_bytes(),
                    originals[spacing_mm],
                )

            post_report = root / "post-install-render" / "scene3-patch-boundaries.json"
            post_report.parent.mkdir()
            post_report.write_text(json.dumps({
                "scenario": "scene3-patch-boundaries",
                "passed": True,
                "failures": [],
            }))
            with mock.patch.object(patches, "_application_running", return_value=False):
                result = patches.command_finalize_install(types.SimpleNamespace(
                    run_dir=root,
                    post_install_smoke_report=post_report,
                ))
            self.assertEqual(result, 0)
            finalized = json.loads((root / "manifest.json").read_text())
            self.assertTrue(finalized["installed"])
            self.assertEqual(finalized["status"], "installed")
            journal = json.loads((root / "install-journal.json").read_text())
            self.assertEqual(journal["state"], "complete")
            for state in journal["files"]:
                self.assertFalse(Path(state["temporary_original"]).exists())

    def test_deferred_install_can_restore_every_original_before_finalization(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            originals, candidates, _ = deferred_install_fixture(root)
            with mock.patch.object(patches, "_application_running", return_value=False):
                patches.command_install(types.SimpleNamespace(
                    run_dir=root,
                    candidate_smoke_report=None,
                    defer_post_install_smoke=True,
                    app=root / "unused-app",
                ))
                result = patches.command_rollback_install(types.SimpleNamespace(
                    run_dir=root,
                ))
            self.assertEqual(result, 0)
            journal = json.loads((root / "install-journal.json").read_text())
            self.assertEqual(journal["state"], "rolled-back")
            for spacing_mm, state in zip(patches.SPACINGS_MM, journal["files"], strict=True):
                canonical = root / f"Site3-SAND-{spacing_mm}mm.ply"
                self.assertEqual(canonical.read_bytes(), originals[spacing_mm])
                self.assertEqual(Path(state["candidate"]).read_bytes(), candidates[spacing_mm])
                self.assertFalse(Path(state["temporary_original"]).exists())

    def test_append_only_augmentation_preserves_base_and_restores_byte_exactly(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = np.array([
                point(0.000, 0.000, 0.000, 12, 10),
                point(0.004, 0.000, 0.000, 3, 20),
                point(0.008, 0.000, 0.000, 13, 30),
            ], dtype=DTYPE)
            source_path = root / "source.ply"
            write_ply(source_path, source)
            info = patches.read_ply_info(source_path)
            addition = np.array([
                point(0.002, 0.002, 0.000, 13, 40),
            ], dtype=DTYPE)
            entry = patches.build_append_only_density_candidate(
                1,
                info,
                addition,
                root / "candidate",
                2,
                role="ROCK",
            )
            self.assertEqual(Path(entry["candidate_path"]).name, "Site3-ROCK-1mm-candidate.ply")
            result = patches.verify_density_entry(entry, 2)
            self.assertTrue(result["passed"], result["failures"])
            installed_entry = dict(entry)
            installed_entry["source_path"] = entry["candidate_path"]
            restored_path = root / "restored.ply"
            restored_hash = patches._restore_density(
                installed_entry,
                restored_path,
                2,
            )
            self.assertEqual(restored_hash, entry["source_sha256"])
            self.assertEqual(restored_path.read_bytes(), source_path.read_bytes())

    def test_patch_01_matches_total_density_when_the_host_scan_is_also_sparse(self) -> None:
        mask = np.zeros((3, 5), dtype=bool)
        mask[1, 1:4] = True
        component = patches.ComponentSupport(
            camera_name="Patch 01",
            cleanmesh_index=1,
            host_scan_id=3,
            bounds_min=np.array([0.0, 0.0, 0.0]),
            bounds_max=np.array([0.10, 0.06, 0.02]),
            dense_records=np.empty(0, dtype=DTYPE),
            cell_keys=np.empty(0, dtype=np.uint64),
            cell_z_min=np.empty(0),
            cell_z_max=np.empty(0),
            origin_ix=0,
            origin_iy=0,
            mask=mask,
            boundary=mask.copy(),
            outside_ring=~mask,
        )
        samples = []
        for iy in range(3):
            for ix in range(5):
                count = 2 if mask[iy, ix] else 10
                for index in range(count):
                    samples.append(point(
                        (ix + 0.1 + index * 0.001) * patches.COMPONENT_CELL_METRES,
                        (iy + 0.1) * patches.COMPONENT_CELL_METRES,
                        0.01,
                        8,
                        1.0,
                    ))
        info = types.SimpleNamespace(fields={
            "x": "x",
            "y": "y",
            "z": "z",
            "scan_id": "scalar_ScanID",
        })
        plan, report = patches.plan_density_additions(
            np.array(samples, dtype=DTYPE),
            info,
            component,
        )
        self.assertEqual(sum(plan.values()), 24)
        self.assertEqual(report["density_target"], "total_density")
        self.assertEqual(report["deficit_fill_fraction"], 1.0)

    def test_validation_project_disables_every_water_visual_including_shoreline(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "source.json"
            output = root / "validation.json"
            source.write_text(json.dumps({
                "selected_point_visual": "Other",
                "point_cloud_preview_lod_mode": "adaptive",
                "live_visual_effects": True,
                "selected_layer_path": "/old/Site3-SAND-5mm.ply",
                "water_rain_settings": {"enabled": True, "rain_level": 1.0},
                "water_dynamic_mesh_flow_settings": {"enabled": True},
                "water_shoreline_instances": [{"id": 1, "enabled": True}],
                "point_visuals": [{
                    "name": "Projector-01",
                    "point_style": {
                        "screen_sprite_size_mode": "pixels",
                        "shoreline_wave_enabled": True,
                        "point_size": {
                            "active": True,
                            "mode": "field_mapped",
                            "constant_value": [5.2, 0.0, 0.0, 0.0],
                        },
                    },
                }],
                "layers": [
                    {
                        "kind": "point_cloud",
                        "scene_group": "Scene3",
                        "scene_role": "ROCK",
                        "source_path": "/old/Site3-ROCK-1mm.ply",
                        "selected_scene_variant_path": "/old/Site3-ROCK-5mm.ply",
                        "loaded": False,
                        "visible": False,
                    },
                    {
                        "kind": "point_cloud",
                        "scene_group": "Scene3",
                        "scene_role": "ROCK",
                        "source_path": "/old/Site3-ROCK-5mm.ply",
                        "selected_scene_variant_path": "/old/Site3-ROCK-5mm.ply",
                        "loaded": True,
                        "visible": True,
                    },
                    {"kind": "gaussian_splat", "visible": True},
                ],
            }), encoding="utf-8")

            patches.generate_validation_project(
                source,
                {
                    "Site3-SAND-5mm.ply": "/new/Site3-SAND-5mm.ply",
                    "Site3-ROCK-1mm.ply": "/new/Site3-ROCK-1mm.ply",
                    "Site3-ROCK-5mm.ply": "/new/Site3-ROCK-5mm.ply",
                },
                output,
            )

            document = json.loads(output.read_text(encoding="utf-8"))
            self.assertEqual(document["selected_point_visual"], "Projector-01")
            self.assertEqual(document["point_cloud_preview_lod_mode"], "full_resolution")
            self.assertFalse(document["live_visual_effects"])
            self.assertFalse(document["water_rain_settings"]["enabled"])
            self.assertEqual(document["water_rain_settings"]["rain_level"], 0.0)
            self.assertFalse(document["water_dynamic_mesh_flow_settings"]["enabled"])
            self.assertEqual(document["water_shoreline_instances"], [])
            self.assertTrue(document["layers"][0]["loaded"])
            self.assertTrue(document["layers"][0]["visible"])
            self.assertFalse(document["layers"][1]["loaded"])
            self.assertFalse(document["layers"][1]["visible"])
            self.assertFalse(document["layers"][2]["visible"])
            self.assertEqual(
                document["layers"][0]["selected_scene_variant_path"],
                "/new/Site3-ROCK-1mm.ply",
            )
            self.assertEqual(
                document["layers"][1]["selected_scene_variant_path"],
                "/new/Site3-ROCK-1mm.ply",
            )
            self.assertEqual(
                document["selected_layer_path"],
                "/new/Site3-ROCK-1mm.ply",
            )
            projector = document["point_visuals"][0]["point_style"]
            self.assertEqual(projector["screen_sprite_size_mode"], "world_millimeters")
            self.assertFalse(projector["shoreline_wave_enabled"])
            self.assertEqual(projector["point_size"]["mode"], "constant")
            self.assertEqual(projector["point_size"]["constant_value"][0], 2.0)

    def test_validation_project_aliases_patch06_and_loads_only_sand_1mm(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "source.json"
            output = root / "validation.json"
            source.write_text(json.dumps({
                "camera_shots": [
                    {"id": "camera_4", "name": "Patch 04", "camera": {"position": [0, 0, 1]}},
                    {"id": "camera_6", "name": "Patch 06", "camera": {"position": [6, 6, 6]}},
                ],
                "point_visuals": [{
                    "name": "Projector-01",
                    "point_style": {"point_size": {"constant_value": [5.0, 0, 0, 0]}},
                }],
                "layers": [
                    {"kind": "point_cloud", "scene_group": "Scene3", "scene_role": "ROCK", "source_path": "/old/Site3-ROCK-1mm.ply"},
                    {"kind": "point_cloud", "scene_group": "Scene3", "scene_role": "SAND", "source_path": "/old/Site3-SAND-1mm.ply"},
                ],
            }), encoding="utf-8")
            patches.generate_validation_project(
                source,
                {
                    "Site3-ROCK-1mm.ply": "/new/Site3-ROCK-1mm.ply",
                    "Site3-SAND-1mm.ply": "/new/Site3-SAND-1mm.ply",
                },
                output,
                scene_role="SAND",
                validation_camera_name="Patch 06",
            )
            document = json.loads(output.read_text(encoding="utf-8"))
            self.assertEqual(document["camera_shots"][0]["camera"]["position"], [6, 6, 6])
            self.assertFalse(document["layers"][0]["loaded"])
            self.assertFalse(document["layers"][0]["visible"])
            self.assertTrue(document["layers"][1]["loaded"])
            self.assertTrue(document["layers"][1]["visible"])
            self.assertEqual(document["selected_layer_path"], "/new/Site3-SAND-1mm.ply")

    def test_vertex_count_patch_preserves_a_padded_header_width(self) -> None:
        header = (
            b"ply\nformat binary_little_endian 1.0\n"
            b"element vertex 131795217           \n"
            b"property float x\nend_header\n"
        )
        updated = patches.patch_header_count(header, 42)
        self.assertEqual(len(updated), len(header))
        self.assertIn(b"element vertex 42                  \n", updated)

    def test_nearest_ar_bundle_is_transferred_and_far_points_are_skipped(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = np.array([
                point(0.000, 0.000, 0.000, 3, 10),
                point(0.004, 0.000, 0.000, 3, 20),
            ], dtype=DTYPE)
            source_path = root / "source.ply"
            write_ply(source_path, source)
            info = patches.read_ply_info(source_path)
            additions = np.array([
                point(0.0038, 0.0, 0.0, 13, 0),
                point(0.1000, 0.0, 0.0, 13, 0),
            ], dtype=DTYPE)
            transferred, report = patches.transfer_ar_nearest(additions, info, source, info)
            self.assertEqual(len(transferred), 1)
            self.assertEqual(float(transferred["scalar_A_R_TestA"][0]), 25.0)
            self.assertEqual(float(transferred["scalar_A_R_TestB"][0]), 26.0)
            self.assertEqual(report["skipped"], 1)

    def test_spacing_limited_density_plan_is_a_render_warning(self) -> None:
        component = {
            "camera": "Patch 03",
            "density": {
                "planned_additions": 100,
                "current_points": 900,
                "interior_mean_points_per_cell": 90.0,
                "ring_mean_points_per_cell": 110.0,
            },
            "selection": {"requested": 100, "selected": 75, "shortfall": 25},
            "ar_transfer": {
                "requested": 75,
                "transferred": 75,
                "skipped": 0,
                "max_distance_m": 0.001,
            },
            "accepted_additions": 75,
        }
        result, failures, warnings = patches.verify_component_additions(component)
        self.assertFalse(failures)
        self.assertEqual(len(warnings), 1)
        self.assertTrue(result["passed"])
        self.assertEqual(result["actual_projected_interior_mean_points_per_cell"], 97.5)

    def test_voxel_representative_is_not_source_order_biased(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            dense = np.array([
                point(0.001 + index * 0.0001, 0.001, 0.010, 11, index)
                for index in range(8)
            ], dtype=DTYPE)
            path = root / "dense.ply"
            write_ply(path, dense)
            info = patches.read_ply_info(path)
            component = component_for(dense)
            cell = int(patches._pack_cells(np.array([0]), np.array([0]))[0])
            selected, report = patches.select_dense_additions(
                dense,
                info,
                np.empty(0, dtype=DTYPE),
                info,
                component,
                {cell: 1},
                0.005,
            )
            self.assertEqual(len(selected), 1)
            self.assertEqual(float(selected["x"][0]), float(dense["x"][7]))
            self.assertIn("hashed", report["representative_strategy"])

    def test_candidate_preservation_and_restore_round_trip(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source_data = np.array([
                point(0.001, 0.001, 0.010, 10, 10),
                point(0.002, 0.001, 0.010, 11, 20),
                point(0.100, 0.100, 0.010, 5, 30),
            ], dtype=DTYPE)
            source_path = root / "Site3-SAND-1mm.ply"
            write_ply(source_path, source_data)
            original_sha = hashlib.sha256(source_path.read_bytes()).hexdigest()
            info = patches.read_ply_info(source_path)
            component = component_for(source_data)
            models = {
                (1, 10): identity_model(component, 10),
                (1, 11): identity_model(component, 11),
            }
            addition = np.array([point(0.003, 0.002, 0.010, 13, 40)], dtype=DTYPE)
            addition["scalar_Interest"] = np.nan
            entry = patches.build_density_candidate(
                1,
                info,
                [source_data],
                addition,
                [component],
                models,
                root / "1mm",
                2,
            )
            result = patches.verify_density_entry(entry, 2)
            self.assertTrue(result["passed"], result["failures"])
            self.assertEqual(result["replacement_count"], 2)
            self.assertEqual(result["addition_count"], 1)

            refreshed_additions = np.array([
                point(0.003, 0.002, 0.010, 13, 40),
                point(0.004, 0.002, 0.010, 13, 50),
            ], dtype=DTYPE)
            refreshed_additions["scalar_Interest"] = np.nan
            patches._replace_candidate_additions(entry, info, refreshed_additions)
            refreshed_result = patches.verify_density_entry(entry, 2)
            self.assertTrue(refreshed_result["passed"], refreshed_result["failures"])
            self.assertEqual(refreshed_result["addition_count"], 2)

            full_original = root / "full-original.ply"
            os.replace(source_path, full_original)
            os.replace(Path(entry["candidate_path"]), source_path)
            restored_path = root / "restored.ply"
            restored_sha = patches._restore_density(entry, restored_path, 2)
            self.assertEqual(restored_sha, original_sha)
            self.assertEqual(restored_path.read_bytes(), full_original.read_bytes())


if __name__ == "__main__":
    unittest.main()
