from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT = REPO_ROOT / "scripts" / "site1_v11_terrain.py"
SPEC = importlib.util.spec_from_file_location("site1_v11_terrain", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
V11 = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = V11
SPEC.loader.exec_module(V11)


PLY_DTYPE = np.dtype(
    [
        ("x", "<f4"),
        ("y", "<f4"),
        ("z", "<f4"),
        ("red", "u1"),
        ("green", "u1"),
        ("blue", "u1"),
        ("nx", "<f4"),
        ("ny", "<f4"),
        ("nz", "<f4"),
        ("scalar_Intensity", "<f8"),
        ("scalar_Composite", "<f4"),
        ("scalar_ScanID", "<f4"),
        ("scalar_A_R_Test", "<f4"),
    ],
    align=False,
)


def records(xyz, scan_ids, *, offset=0.0):
    xyz = np.asarray(xyz, np.float64)
    result = np.zeros(len(xyz), PLY_DTYPE)
    result["x"], result["y"], result["z"] = xyz.T
    result["red"] = np.arange(len(xyz), dtype=np.uint8) + 50
    result["green"] = 100
    result["blue"] = 150
    result["nz"] = 1.0
    result["scalar_Intensity"] = 5000.0 + offset + np.arange(len(xyz)) * 100.0
    result["scalar_Composite"] = 50.0 + offset + np.arange(len(xyz))
    result["scalar_ScanID"] = scan_ids
    result["scalar_A_R_Test"] = 0.25 + np.arange(len(xyz))
    return result


def write_ply(path: Path, values: np.ndarray) -> None:
    typemap = {
        "f4": "float", "f8": "double", "u1": "uchar", "i1": "char",
        "i2": "short", "u2": "ushort", "i4": "int", "u4": "uint",
    }
    with path.open("wb") as handle:
        handle.write(b"ply\nformat binary_little_endian 1.0\n")
        handle.write(f"element vertex {len(values)}\n".encode("ascii"))
        for name in values.dtype.names or ():
            code = values.dtype[name].str.lstrip("<>=|")
            handle.write(f"property {typemap[code]} {name}\n".encode("ascii"))
        handle.write(b"end_header\n")
        values.tofile(handle)


def read_ply(path: Path) -> np.ndarray:
    info = V11.inspect_fixed_stride_ply(path)
    memory = np.memmap(
        path, dtype=info.dtype, mode="r", offset=info.offset,
        shape=(info.vertex_count,),
    )
    result = np.asarray(memory).copy()
    del memory
    return result


def gridded_cell_points(width=10, height=10, points_per_cell=4, holes=()):
    holes = set(holes)
    output = []
    offsets = [(0.25, 0.25), (0.70, 0.25), (0.25, 0.70), (0.70, 0.70)]
    for iy in range(height):
        for ix in range(width):
            if (ix, iy) in holes:
                continue
            for ox, oy in offsets[:points_per_cell]:
                output.append(((ix + ox) * 0.1, (iy + oy) * 0.1))
    return np.asarray(output, np.float64)


class ReviewConfigTests(unittest.TestCase):
    def test_only_written_terrain_actions_become_targets(self):
        targets = V11.terrain_targets_from_review_config(
            REPO_ROOT / "scripts/config/site1_fossils_v11_review.json"
        )
        by_id = {target.target_id: target for target in targets}
        self.assertEqual(
            set(by_id),
            {"image_1_mark_1", "image_3_mark_2", "image_3_mark_3", "image_4_mark_2"},
        )
        self.assertEqual(by_id["image_3_mark_2"].kind, V11.DeficitKind.CRACK)
        self.assertEqual(
            by_id["image_3_mark_2"].minimum_tier,
            V11.ConfidenceTier.STRONG,
        )
        self.assertEqual(by_id["image_3_mark_3"].kind, V11.DeficitKind.SCANNER)
        self.assertAlmostEqual(by_id["image_3_mark_3"].search_radius_m, 0.6)


class DensityDeficitTests(unittest.TestCase):
    def test_marked_hole_is_connected_and_annotation_is_not_blanket_filled(self):
        points = gridded_cell_points(
            holes={(4, 4), (5, 4), (4, 5), (5, 5)}
        )
        result = V11.detect_density_deficits(
            points,
            bbox=[0.0, 1.0, 0.0, 1.0],
            cell_size_m=0.1,
            neighbourhood_radius_cells=2,
            minimum_expected_points=2.0,
            minimum_deficit_fraction=0.5,
            minimum_component_cells=3,
        )
        self.assertEqual(len(result.components), 1)
        self.assertEqual(result.components[0].cell_count, 4)
        self.assertEqual(int(np.count_nonzero(result.candidate_mask)), 4)
        self.assertLess(
            np.count_nonzero(result.candidate_mask), result.candidate_mask.size
        )
        np.testing.assert_allclose(
            result.components[0].centre_xy, [0.5, 0.5], atol=1e-12
        )

    def test_scanner_search_radius_does_not_prescribe_detected_boundary(self):
        holes = {
            (ix, iy)
            for iy in range(3, 7)
            for ix in range(3, 7)
            if (ix - 4.5) ** 2 + (iy - 4.5) ** 2 <= 3.0
        }
        points = gridded_cell_points(holes=holes)
        target = V11.TerrainReviewTarget(
            "scanner", V11.DeficitKind.SCANNER, (0.0, 1.0, 0.0, 1.0),
            V11.ConfidenceTier.SUPPORTED, (0.5, 0.5), 0.45,
        )
        result = V11.detect_scanner_footprint_deficit(
            points,
            target,
            cell_size_m=0.1,
            neighbourhood_radius_cells=2,
            minimum_expected_points=1.5,
            minimum_deficit_fraction=0.45,
            minimum_component_cells=3,
        )
        self.assertEqual(len(result.components), 1)
        detected = result.components[0]
        self.assertLess(detected.bbox[1] - detected.bbox[0], 0.9)
        self.assertLess(detected.bbox[3] - detected.bbox[2], 0.9)

    def test_crack_detector_rejects_round_hole_but_accepts_linear_deficit(self):
        round_points = gridded_cell_points(
            holes={(4, 4), (5, 4), (4, 5), (5, 5)}
        )
        target = V11.TerrainReviewTarget(
            "crack", V11.DeficitKind.CRACK, (0.0, 1.0, 0.0, 1.0),
            V11.ConfidenceTier.STRONG,
        )
        round_result = V11.detect_crack_density_deficit(
            round_points,
            target,
            cell_size_m=0.1,
            neighbourhood_radius_cells=2,
            minimum_expected_points=2.0,
            minimum_deficit_fraction=0.5,
        )
        self.assertEqual(len(round_result.components), 0)

        line_points = gridded_cell_points(
            holes={(2, 5), (3, 5), (4, 5), (5, 5), (6, 5), (7, 5)}
        )
        line_result = V11.detect_crack_density_deficit(
            line_points,
            target,
            cell_size_m=0.1,
            neighbourhood_radius_cells=2,
            minimum_expected_points=2.0,
            minimum_deficit_fraction=0.5,
        )
        self.assertEqual(len(line_result.components), 1)
        self.assertGreater(line_result.components[0].aspect_ratio, 3.0)


class GeometryEvidenceTests(unittest.TestCase):
    def test_eight_sector_support_requires_angular_coverage(self):
        angles = np.arange(8) * np.pi / 4.0 + 0.01
        donors = np.column_stack((np.cos(angles), np.sin(angles))) * 0.02
        full = V11.compute_eight_sector_support(
            [[0.0, 0.0]], donors, outer_radius_m=0.03
        )
        self.assertEqual(full.occupied_sector_count[0], 8)
        np.testing.assert_array_equal(full.counts[0], np.ones(8, dtype=int))

        one_side = V11.compute_eight_sector_support(
            [[0.0, 0.0]], donors[:3], outer_radius_m=0.03
        )
        self.assertEqual(one_side.occupied_sector_count[0], 3)

    def test_robust_quadratic_mls_recovers_plane_despite_outlier(self):
        x, y = np.meshgrid(np.linspace(-0.03, 0.03, 9), np.linspace(-0.03, 0.03, 9))
        z = 1.0 + 2.0 * x - 3.0 * y
        donor = np.column_stack((x.ravel(), y.ravel(), z.ravel()))
        donor[0, 2] += 0.5
        result = V11.predict_robust_quadratic_mls(
            [[0.0, 0.0]], donor, bandwidth_m=0.05,
            minimum_donors=12, iterations=6,
        )
        self.assertTrue(result.valid[0])
        self.assertAlmostEqual(result.height_m[0], 1.0, places=4)
        expected_normal = np.array([-2.0, 3.0, 1.0])
        expected_normal /= np.linalg.norm(expected_normal)
        np.testing.assert_allclose(result.normal[0], expected_normal, atol=2e-3)

    def test_linear_and_boundary_predictions_are_independent_and_planar(self):
        x, y = np.meshgrid(np.linspace(-1.0, 1.0, 7), np.linspace(-1.0, 1.0, 7))
        z = 2.0 + 0.25 * x - 0.4 * y
        donor = np.column_stack((x.ravel(), y.ravel(), z.ravel()))
        query = np.array([[0.13, -0.21]])
        expected = 2.0 + 0.25 * query[0, 0] - 0.4 * query[0, 1]
        linear = V11.predict_delaunay_linear_surface(query, donor)
        boundary = V11.predict_lower_boundary_surface(
            query,
            donor,
            inner_radius_m=0.1,
            outer_radius_m=1.6,
            minimum_sectors=6,
        )
        self.assertTrue(linear.valid[0])
        self.assertTrue(boundary.valid[0])
        self.assertAlmostEqual(linear.height_m[0], expected, places=8)
        self.assertAlmostEqual(boundary.height_m[0], expected, places=8)
        self.assertNotEqual(linear.method, boundary.method)

    def test_normal_thickness_and_multimodality_expose_noisy_layer(self):
        rng = np.random.default_rng(4)
        xy = rng.uniform(-0.02, 0.02, size=(80, 2))
        normals = np.tile([0.0, 0.0, 1.0], (80, 1))
        normals[::2] *= -1.0
        coherence = V11.compute_local_normal_coherence(
            [[0.0, 0.0]], xy, normals, radius_m=0.04
        )
        self.assertGreater(coherence.coherence[0], 0.999)

        single_z = rng.normal(0.0, 0.0005, len(xy))
        bimodal_z = np.r_[
            rng.normal(-0.006, 0.0003, len(xy) // 2),
            rng.normal(0.006, 0.0003, len(xy) // 2),
        ]
        single = V11.compute_vertical_distribution_metrics(
            [[0.0, 0.0]], np.column_stack((xy, single_z)), radius_m=0.04
        )
        bimodal = V11.compute_vertical_distribution_metrics(
            [[0.0, 0.0]], np.column_stack((xy, bimodal_z)), radius_m=0.04
        )
        self.assertLess(single.thickness_m[0], bimodal.thickness_m[0])
        self.assertLess(single.multimodality_score[0], 0.2)
        self.assertGreater(bimodal.multimodality_score[0], 0.35)

    def test_residual_energy_ratio_is_explicit_and_fail_closed_at_zero_reference(self):
        ratio = V11.residual_energy_ratio([0.0, 1.0, 2.0], [0.0, 0.0, 1.0])
        self.assertEqual(ratio[0], 0.0)
        self.assertTrue(np.isinf(ratio[1]))
        self.assertEqual(ratio[2], 2.0)


class SelectionAndDonorTests(unittest.TestCase):
    @staticmethod
    def confidence(tiers, reasons=None):
        tiers = np.asarray(tiers, np.uint8)
        if reasons is None:
            reasons = np.zeros(len(tiers), np.uint32)
        return V11.ConfidenceResult(
            reason_mask=np.asarray(reasons, np.uint32),
            tier=tiers,
            surface_spread_m=np.full(len(tiers), 0.001),
            preferred_gate_count=np.full(len(tiers), 5, np.uint8),
        )

    def test_variable_radius_selection_respects_confidence_and_existing_points(self):
        candidate = np.array(
            [[0.05, 0.0, 0.0], [0.30, 0.0, 0.0], [0.60, 0.0, 0.0], [0.90, 0.0, 0.0]]
        )
        confidence = self.confidence(
            [V11.ConfidenceTier.STRONG, V11.ConfidenceTier.CONSERVATIVE,
             V11.ConfidenceTier.STRONG, V11.ConfidenceTier.REJECTED],
            [0, 0, 0, int(V11.ConfidenceReason.LOW_NORMAL_COHERENCE)],
        )
        result = V11.select_variable_radius_interstitials(
            candidate,
            0.20,
            measured_xyz=[[0.0, 0.0, 0.0]],
            confidence=confidence,
            minimum_tier=V11.ConfidenceTier.SUPPORTED,
            priority=[4, 3, 2, 1],
            seed=11,
        )
        np.testing.assert_array_equal(result.selected_indices, [2])
        self.assertEqual(result.disposition[1], V11.TerrainDisposition.REVIEW)
        self.assertTrue(
            result.reason_mask[0] & int(V11.TerrainDecisionReason.EXISTING_CLEARANCE)
        )
        self.assertTrue(
            result.reason_mask[3] & int(V11.TerrainDecisionReason.HARD_GEOMETRY_VETO)
        )

    def test_scanid10_additions_ignore_scanid9_and_forbid_cross_role(self):
        donors = records(
            [
                [0.0, 0.0, 1.0],
                [0.01, 0.0, 1.001],
                [1.0, 0.0, 2.0],
                [2.0, 0.0, 3.0],
            ],
            [0, 8, 9, 10],
        )
        result = V11.build_scanid10_additions(
            [[0.005, 0.0, 1.0005], [1.0, 0.0, 2.0]],
            donors,
            role="SAND",
            donor_role="SAND",
            maximum_donor_distance_m=0.03,
            neighbours=4,
        )
        np.testing.assert_array_equal(result.resolved_query_indices, [0])
        np.testing.assert_array_equal(result.unresolved_query_indices, [1])
        self.assertEqual(result.records["scalar_ScanID"][0], 10)
        self.assertIn(result.primary_donor_indices[0], [0, 1])
        self.assertGreaterEqual(result.records["scalar_Intensity"][0], 5000.0)
        self.assertLessEqual(result.records["scalar_Intensity"][0], 5100.0)
        with self.assertRaisesRegex(ValueError, "cross-role"):
            V11.build_scanid10_additions(
                [[0.0, 0.0, 1.0]], donors, role="SAND", donor_role="ROCK"
            )

    def test_unresolved_donor_changes_selected_candidate_to_review(self):
        candidate = np.array([[0.005, 0.0, 1.0], [1.0, 0.0, 2.0]])
        confidence = self.confidence([3, 3])
        selection = V11.select_variable_radius_interstitials(
            candidate,
            0.001,
            measured_xyz=np.empty((0, 3)),
            confidence=confidence,
            minimum_tier=V11.ConfidenceTier.SUPPORTED,
            priority=[2, 1],
        )
        donors = records([[0.0, 0.0, 1.0]], [0])
        sampled = V11.build_scanid10_additions(
            candidate[selection.selected_indices], donors,
            role="ROCK", donor_role="ROCK", maximum_donor_distance_m=0.03,
        )
        provenance = V11.build_candidate_provenance(
            candidate, "review", confidence, selection, sampled
        )
        self.assertEqual(provenance.disposition[0], V11.TerrainDisposition.ACCEPTED)
        self.assertEqual(provenance.disposition[1], V11.TerrainDisposition.REVIEW)
        self.assertTrue(
            provenance.decision_reason_mask[1]
            & int(V11.TerrainDecisionReason.NO_SAME_ROLE_MEASURED_DONOR)
        )


class CleanMeshAdapterTests(unittest.TestCase):
    def test_local_analysis_round_trip_and_append_only_candidate(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            sand_path = root / "sand-source.ply"
            rock_path = root / "rock-source.ply"
            sand = records(
                [[0.0, 0.0, 1.0], [0.1, 0.0, 1.0], [0.2, 0.0, 1.0]],
                [0, 1, 2],
            )
            rock = records(
                [[0.0, 1.0, 2.0], [0.1, 1.0, 2.0], [0.2, 1.0, 2.0]],
                [3, 4, 5],
                offset=500.0,
            )
            write_ply(sand_path, sand)
            write_ply(rock_path, rock)
            addition = sand[[0]].copy()
            addition["x"], addition["y"], addition["z"] = 0.05, 0.05, 1.001
            addition["scalar_ScanID"] = 10

            selections = [
                V11.make_local_source_selection(
                    sand_path, [0, 2], role="SAND", label="sand-collar"
                ),
                V11.make_local_source_selection(
                    rock_path, [1], role="ROCK", label="rock-collar"
                ),
            ]
            input_path = root / "terrain-local.analysis-input.ply"
            manifest = V11.write_local_analysis_input(
                input_path,
                selections,
                [V11.LocalAdditionBatch(addition, "SAND", "sand-additions")],
                chunk_size=1,
            )
            local = read_ply(input_path)
            self.assertEqual(len(local), 4)
            np.testing.assert_array_equal(local["scalar_TypeID"], [1, 1, 0, 1])
            self.assertEqual(local["scalar_ScanID"][-1], 10)
            self.assertTrue(manifest.manifest_path.exists())

            analysed_dtype = np.dtype(
                [(name, local.dtype[name]) for name in local.dtype.names]
                + [("scalar_A_R_Test", "<f4")]
            )
            analysed = np.empty(len(local), analysed_dtype)
            for name in local.dtype.names:
                analysed[name] = local[name]
            analysed["scalar_A_R_Test"] = np.arange(len(local)) + 0.5
            analysed_path = root / "terrain-local.analysis.ply"
            # CleanMesh's tiled merge is allowed to reorder records.  Keep the
            # analysed scalar attached to its original point while moving the
            # ScanID 10 addition to the front of the output.
            output_order = np.array([3, 0, 2, 1], dtype=np.int64)
            write_ply(analysed_path, analysed[output_order])
            extracted = V11.extract_cleanmesh_analysed_additions(
                analysed_path, manifest
            )
            self.assertEqual(len(extracted), 1)
            self.assertEqual(len(extracted[0].records), 1)
            projected = V11.project_analysed_additions(
                extracted[0].records, PLY_DTYPE
            )
            self.assertEqual(projected["scalar_ScanID"][0], 10)
            self.assertEqual(projected["scalar_A_R_Test"][0], 3.5)

            corrupted = analysed[output_order].copy()
            corrupted[0]["x"] += 0.001
            corrupted_path = root / "terrain-local.corrupted-analysis.ply"
            write_ply(corrupted_path, corrupted)
            with self.assertRaisesRegex(RuntimeError, "changed an addition"):
                V11.extract_cleanmesh_analysed_additions(
                    corrupted_path, manifest
                )

            source_bytes = sand_path.read_bytes()
            source_layout = V11.inspect_fixed_stride_ply(sand_path)
            candidate_path = root / "Site1-SAND-1mm.candidate.ply"
            report = V11.write_append_only_candidate(
                sand_path, projected, candidate_path, chunk_size=1
            )
            candidate_layout = V11.inspect_fixed_stride_ply(candidate_path)
            self.assertEqual(candidate_layout.vertex_count, len(sand) + 1)
            candidate = read_ply(candidate_path)
            np.testing.assert_array_equal(candidate[: len(sand)], sand)
            np.testing.assert_array_equal(candidate[-1:], projected)
            self.assertTrue(report["base_payload_byte_identical"])
            self.assertEqual(
                source_bytes[source_layout.offset :],
                candidate_path.read_bytes()[
                    candidate_layout.offset : candidate_layout.offset
                    + len(sand) * PLY_DTYPE.itemsize
                ],
            )

    def test_writers_refuse_canonical_or_source_aliases(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = root / "source.ply"
            values = records([[0.0, 0.0, 0.0]], [0])
            write_ply(source, values)
            with self.assertRaisesRegex(ValueError, "canonical"):
                V11.write_append_only_candidate(
                    source, np.empty(0, PLY_DTYPE),
                    root / "Site1-ROCK-5mm.ply",
                )
            with self.assertRaisesRegex(ValueError, "aliases"):
                V11.write_append_only_candidate(
                    source, np.empty(0, PLY_DTYPE), source
                )


if __name__ == "__main__":
    unittest.main()
