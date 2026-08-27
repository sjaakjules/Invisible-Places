import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np


SCRIPT = Path(__file__).resolve().parents[1] / "scripts/site1_v11_obstructions.py"
SPEC = importlib.util.spec_from_file_location("site1_v11_obstructions", SCRIPT)
V11 = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = V11
SPEC.loader.exec_module(V11)


class ModelConsensusTests(unittest.TestCase):
    def test_two_close_models_survive_a_distant_third_model(self):
        result = V11.evaluate_model_consensus(
            [
                [1.000, 1.010, 1.300],
                [2.000, 2.100, 2.250],
                [np.nan, 3.000, 3.020],
            ],
            maximum_model_spread_m=0.05,
        )
        np.testing.assert_array_equal(result.agreeing_model_count, [2, 0, 2])
        np.testing.assert_array_equal(result.has_consensus, [True, False, True])
        np.testing.assert_allclose(
            result.surface_height_m[[0, 2]], [1.005, 3.010]
        )
        self.assertAlmostEqual(result.model_spread_m[0], 0.010)
        # With no accepted pair, the closest possible pair spread remains as
        # review evidence rather than being silently replaced by NaN.
        self.assertAlmostEqual(result.model_spread_m[1], 0.100)

    def test_surface_residual_is_measured_from_consensus_not_outlier_mean(self):
        result = V11.evaluate_surface_consensus(
            [1.090],
            [[1.000, 1.010, 1.500]],
            residual_threshold_m=0.08,
        )
        self.assertTrue(result.above_threshold[0])
        self.assertAlmostEqual(result.surface_height_m[0], 1.005)
        self.assertAlmostEqual(result.residual_m[0], 0.085)


class ObstructionClassificationTests(unittest.TestCase):
    def test_scanid9_requires_core_connected_two_model_consensus(self):
        models = np.array(
            [
                [1.000, 1.010, 1.300],
                [1.000, 1.010, 1.300],
                [1.000, 1.005, 1.010],
                [1.000, 1.010, 1.300],
            ]
        )
        result = V11.classify_obstruction_points(
            [1.100, 1.100, 1.015, 1.100],
            models,
            core_connected=[True, False, True, True],
            candidate_mode=[
                V11.CandidateMode.SEED,
                V11.CandidateMode.SEED,
                V11.CandidateMode.GROW,
                V11.CandidateMode.SEED,
            ],
            touches_roi_boundary=[False, False, False, True],
            scan_id=[9, 9, 4, 4],
        )
        np.testing.assert_array_equal(
            result.disposition,
            [
                V11.PointDisposition.AUTO_REMOVE,
                V11.PointDisposition.REVIEW,
                V11.PointDisposition.REJECTED,
                V11.PointDisposition.REVIEW,
            ],
        )
        self.assertFalse(
            result.reason_mask[0] & int(V11.ObstructionReason.SCANID9_PROTECTED)
        )
        self.assertTrue(
            result.reason_mask[1] & int(V11.ObstructionReason.SCANID9_PROTECTED)
        )
        self.assertTrue(
            result.reason_mask[2]
            & int(V11.ObstructionReason.GROUND_BAND_PROTECTED)
        )
        self.assertTrue(
            result.reason_mask[3]
            & int(V11.ObstructionReason.ROI_BOUNDARY_PROTECTED)
        )

    def test_1mm_transfer_needs_eight_mm_seed_and_independent_residual(self):
        models = np.array(
            [
                [1.000, 1.005, 1.300],
                [1.000, 1.005, 1.300],
                [1.000, 1.005, 1.300],
            ]
        )
        transfer = V11.evaluate_1mm_transfer(
            [1.060, 1.060, 1.020],
            models,
            [0.008, 0.008001, 0.007],
            core_connected=True,
            scan_id=[4, 4, 4],
        )
        np.testing.assert_array_equal(transfer.eligible, [True, False, False])
        self.assertTrue(
            transfer.reason_mask[1]
            & int(V11.ObstructionReason.FINE_SEED_MISSING_OR_TOO_FAR)
        )
        self.assertTrue(
            transfer.reason_mask[2]
            & int(V11.ObstructionReason.FINE_INDEPENDENT_RESIDUAL_FAILED)
        )

        classified = V11.classify_obstruction_points(
            [1.060, 1.060, 1.020],
            models,
            core_connected=True,
            candidate_mode=V11.CandidateMode.GROW,
            scan_id=[4, 4, 4],
            fine_seed_distance_m=[0.008, 0.008001, 0.007],
        )
        np.testing.assert_array_equal(
            classified.disposition,
            [
                V11.PointDisposition.AUTO_REMOVE,
                V11.PointDisposition.REVIEW,
                V11.PointDisposition.REJECTED,
            ],
        )

    def test_collar_points_are_hard_rejected_even_when_high(self):
        result = V11.classify_obstruction_points(
            [1.2],
            [[1.0, 1.01, 1.4]],
            core_connected=True,
            collar_point=True,
            scan_id=[4],
        )
        self.assertEqual(result.disposition[0], V11.PointDisposition.REJECTED)
        self.assertTrue(
            result.reason_mask[0] & int(V11.ObstructionReason.COLLAR_PROTECTED)
        )

    def test_sparse_archive_adapter_preserves_source_ids_and_metrics(self):
        result = V11.classify_obstruction_points(
            [1.10, 1.01],
            [[1.00, 1.01, 1.30], [1.00, 1.01, 1.30]],
            core_connected=[True, True],
            candidate_mode=[V11.CandidateMode.SEED, V11.CandidateMode.GROW],
        )
        sparse = V11.SparsePointClassifications.from_classification(
            [90, 12], result
        )
        np.testing.assert_array_equal(sparse.original_indices, [12, 90])
        np.testing.assert_array_equal(
            sparse.disposition,
            [V11.PointDisposition.REJECTED, V11.PointDisposition.AUTO_REMOVE],
        )
        np.testing.assert_allclose(sparse.residual_m, [0.005, 0.095])


class TerrainPreservationTests(unittest.TestCase):
    def test_ground_collar_and_per_cell_loss_are_fail_closed(self):
        result = V11.validate_terrain_preservation(
            [
                V11.PointDisposition.AUTO_REMOVE,
                V11.PointDisposition.AUTO_REMOVE,
                V11.PointDisposition.KEEP,
                V11.PointDisposition.AUTO_REMOVE,
            ],
            [0.10, 0.01, 0.00, 0.10],
            collar_mask=[False, False, False, True],
            terrain_cell_ids=[1, 1, 2, 2],
            well_supported_ground_mask=[True, True, True, True],
        )
        self.assertFalse(result.passed)
        self.assertEqual(result.ground_band_removed_count, 1)
        self.assertEqual(result.collar_removed_count, 1)
        self.assertAlmostEqual(result.maximum_cell_ground_loss_fraction, 1.0)
        np.testing.assert_array_equal(result.failing_cell_ids, [1, 2])

    def test_safe_high_residual_removal_passes(self):
        result = V11.validate_terrain_preservation(
            [V11.PointDisposition.AUTO_REMOVE, V11.PointDisposition.KEEP],
            [0.10, 0.00],
            collar_mask=[False, True],
            terrain_cell_ids=[1, 1],
            well_supported_ground_mask=[False, True],
        )
        self.assertTrue(result.passed)


class CandidateArchiveTests(unittest.TestCase):
    RECORD_DTYPE = np.dtype(
        [
            ("x", "<f4"),
            ("y", "<f4"),
            ("z", "<f4"),
            ("scan_id", "<f4"),
            ("tag", "<u4"),
        ]
    )

    @classmethod
    def write_fixture(cls, path: Path) -> np.ndarray:
        records = np.zeros(5, dtype=cls.RECORD_DTYPE)
        records["x"] = np.arange(5, dtype=np.float32) + 0.25
        records["y"] = np.arange(5, dtype=np.float32) + 10.5
        records["z"] = np.arange(5, dtype=np.float32) + 20.75
        records["scan_id"] = [4, 9, 5, 7, 6]
        records["tag"] = [101, 202, 303, 404, 505]
        header = (
            "ply\n"
            "format binary_little_endian 1.0\n"
            "comment exact-record fixture\n"
            "element vertex 5\n"
            "property float x\n"
            "property float y\n"
            "property float z\n"
            "property float scan_id\n"
            "property uint tag\n"
            "end_header\n"
        ).encode("ascii")
        with path.open("wb") as handle:
            handle.write(header)
            records.tofile(handle)
        return records

    @staticmethod
    def read_payload(path: Path, dtype: np.dtype) -> np.ndarray:
        layout = V11.inspect_binary_vertex_ply(path)
        return np.memmap(
            path,
            dtype=dtype,
            mode="r",
            offset=layout.payload_offset,
            shape=(layout.vertex_count,),
        ).copy()

    def test_stream_keeps_review_and_rejected_and_archives_exact_remove(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = root / "Site1-ROCK-5mm.ply"
            records = self.write_fixture(source)
            source_bytes = source.read_bytes()
            source_hash = hashlib.sha256(source_bytes).hexdigest()
            decisions = V11.SparsePointClassifications(
                original_indices=[3, 1, 2],
                disposition=[
                    V11.PointDisposition.REJECTED,
                    V11.PointDisposition.AUTO_REMOVE,
                    V11.PointDisposition.REVIEW,
                ],
                reason_mask=[
                    V11.ObstructionReason.GROUND_BAND_PROTECTED,
                    V11.ObstructionReason.NONE,
                    V11.ObstructionReason.MODEL_SPREAD,
                ],
                consensus_count=[2, 2, 0],
                model_spread_m=[0.01, 0.01, 0.10],
                residual_m=[0.01, 0.12, 0.08],
            )
            bundle = root / "candidate-bundle"
            result = V11.write_candidate_archive(
                source,
                bundle,
                decisions,
                expected_source_sha256=source_hash,
                chunk_records=2,
            )

            self.assertEqual(source.read_bytes(), source_bytes)
            self.assertEqual(result.source_count, 5)
            self.assertEqual(result.candidate_count, 4)
            self.assertEqual(result.removed_count, 1)
            candidate = self.read_payload(result.candidate_path, self.RECORD_DTYPE)
            removed = self.read_payload(
                result.removed_records_path, self.RECORD_DTYPE
            )
            np.testing.assert_array_equal(candidate, records[[0, 2, 3, 4]])
            np.testing.assert_array_equal(removed, records[[1]])
            removed_indices = np.load(
                result.removed_indices_path, allow_pickle=False
            )
            self.assertEqual(removed_indices.dtype, np.dtype(np.uint64))
            np.testing.assert_array_equal(removed_indices, [1])

            with np.load(result.classifications_path, allow_pickle=False) as saved:
                np.testing.assert_array_equal(saved["original_indices"], [1, 2, 3])
                np.testing.assert_array_equal(
                    saved["disposition"],
                    [
                        V11.PointDisposition.AUTO_REMOVE,
                        V11.PointDisposition.REVIEW,
                        V11.PointDisposition.REJECTED,
                    ],
                )
                self.assertIn("model_spread_m", saved.files)
                self.assertIn("residual_m", saved.files)
            manifest = json.loads(result.manifest_path.read_text())
            self.assertFalse(manifest["canonical_writes"])
            self.assertTrue(
                manifest["classifications"]["review_and_rejected_retained"]
            )
            self.assertTrue(
                manifest["removed"]["records_are_exact_source_bytes"]
            )
            self.assertEqual(V11.sha256_path(result.candidate_path), result.candidate_sha256)
            self.assertEqual(
                V11.sha256_path(result.removed_records_path),
                result.removed_records_sha256,
            )

    def test_source_hash_mismatch_publishes_nothing(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = root / "Site1-ROCK-1mm.ply"
            self.write_fixture(source)
            source_bytes = source.read_bytes()
            bundle = root / "candidate-bundle"
            decisions = V11.SparsePointClassifications(
                original_indices=[1],
                disposition=[V11.PointDisposition.AUTO_REMOVE],
                reason_mask=[0],
            )
            with self.assertRaisesRegex(RuntimeError, "source hash lock failed"):
                V11.write_candidate_archive(
                    source,
                    bundle,
                    decisions,
                    expected_source_sha256="0" * 64,
                    chunk_records=2,
                )
            self.assertFalse(bundle.exists())
            self.assertEqual(source.read_bytes(), source_bytes)

    def test_existing_bundle_is_never_overwritten(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = root / "source.ply"
            self.write_fixture(source)
            bundle = root / "candidate-bundle"
            bundle.mkdir()
            marker = bundle / "keep.txt"
            marker.write_text("untouched")
            decisions = V11.SparsePointClassifications([], [], [])
            with self.assertRaises(FileExistsError):
                V11.write_candidate_archive(
                    source,
                    bundle,
                    decisions,
                    expected_source_sha256=V11.sha256_path(source),
                )
            self.assertEqual(marker.read_text(), "untouched")


if __name__ == "__main__":
    unittest.main()
