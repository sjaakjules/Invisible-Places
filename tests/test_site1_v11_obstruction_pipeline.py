import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))
SCRIPT = SCRIPTS / "site1_v11_obstruction_pipeline.py"
SPEC = importlib.util.spec_from_file_location(
    "site1_v11_obstruction_pipeline", SCRIPT
)
V11 = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = V11
SPEC.loader.exec_module(V11)


class SurfaceModelTests(unittest.TestCase):
    def test_three_independent_models_reproduce_a_sloping_ground(self):
        axis = np.linspace(0.0, 1.0, 31)
        x, y = np.meshgrid(axis, axis)
        z = 0.4 + 0.025 * x - 0.018 * y + 0.002 * x * y
        terrain = np.column_stack((x.ravel(), y.ravel(), z.ravel()))
        object_xy = terrain[
            (np.abs(terrain[:, 0] - 0.5) < 0.09)
            & (np.abs(terrain[:, 1] - 0.5) < 0.09),
            :2,
        ]
        elevated = np.column_stack(
            (object_xy, 0.4 + 0.025 * object_xy[:, 0] - 0.018 * object_xy[:, 1] + 0.2)
        )
        points = np.concatenate((terrain, elevated))
        donor = ~(
            (np.abs(points[:, 0] - 0.5) < 0.12)
            & (np.abs(points[:, 1] - 0.5) < 0.12)
        )
        query = np.array([[0.45, 0.45], [0.50, 0.50], [0.55, 0.55]])
        result = V11.fit_independent_surface_models(
            points,
            donor,
            query,
            parameters=V11.SurfaceParameters(
                anchor_cell_m=0.06,
                minimum_points_per_anchor=1,
                neighbour_count=12,
            ),
        )
        expected = 0.4 + 0.025 * query[:, 0] - 0.018 * query[:, 1] + 0.002 * query[:, 0] * query[:, 1]
        self.assertEqual(
            result.model_names,
            ("robust_quadratic", "lower_envelope_idw", "local_weighted_plane"),
        )
        self.assertEqual(result.heights_m.shape, (3, 3))
        self.assertTrue(np.all(np.isfinite(result.heights_m)))
        self.assertLess(float(np.max(np.abs(result.heights_m - expected[:, None]))), 0.012)


class ConnectivityTests(unittest.TestCase):
    def test_only_seeded_component_is_core_connected(self):
        first = np.array(
            [[0.000, 0.000, 0.10], [0.004, 0.000, 0.10], [0.008, 0.000, 0.10]]
        )
        second = first + np.array([0.20, 0.0, 0.0])
        points = np.concatenate((first, second))
        modes = np.array(
            [V11.obstruction.CandidateMode.SEED]
            + [V11.obstruction.CandidateMode.GROW] * 5,
            dtype=np.uint8,
        )
        result = V11.connect_candidate_components(
            points,
            modes,
            core_seed_mask=[True, False, False, False, False, False],
            boundary_mask=False,
            voxel_size_m=0.006,
        )
        np.testing.assert_array_equal(
            result.core_connected,
            [True, True, True, False, False, False],
        )
        self.assertEqual(result.component_count, 2)

    def test_boundary_contact_propagates_to_whole_component(self):
        points = np.array(
            [[0.000, 0.000, 0.10], [0.004, 0.000, 0.10], [0.008, 0.000, 0.10]]
        )
        result = V11.connect_candidate_components(
            points,
            [V11.obstruction.CandidateMode.SEED, V11.obstruction.CandidateMode.GROW, V11.obstruction.CandidateMode.GROW],
            core_seed_mask=[True, False, False],
            boundary_mask=[False, False, True],
            voxel_size_m=0.006,
        )
        np.testing.assert_array_equal(result.touches_boundary, [True, True, True])


class CrossScaleTests(unittest.TestCase):
    @staticmethod
    def analysis(name, xyz, clip_id):
        xyz = np.asarray(xyz, dtype=np.float64)
        count = len(xyz)
        predictions = np.zeros((count, 3), dtype=np.float64)
        modes = np.full(
            count, V11.obstruction.CandidateMode.SEED, dtype=np.uint8
        )
        classification = V11.obstruction.classify_obstruction_points(
            xyz[:, 2],
            predictions,
            core_connected=True,
            candidate_mode=modes,
            scan_id=4,
        )
        layout = V11.NamedPlyLayout(
            path=Path("unused.ply"),
            dtype=np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4")]),
            vertex_count=count,
            payload_offset=0,
            header=b"",
        )
        roi = V11.RoiCloud(
            source_path=layout.path,
            source_sha256="0" * 64,
            layout=layout,
            original_indices=np.arange(count, dtype=np.uint64),
            xyz=xyz,
            scan_id=np.full(count, 4.0),
            clip_index=np.full(count, clip_id, dtype=np.int16),
            core_mask=np.ones(count, dtype=bool),
            boundary_mask=np.zeros(count, dtype=bool),
        )
        prediction = V11.SurfacePrediction(
            model_names=("one", "two", "three"),
            heights_m=predictions,
            anchors=V11.GroundAnchors(
                xy=np.empty((0, 2)),
                z=np.empty(0),
                support_count=np.empty(0, dtype=np.int32),
            ),
        )
        connectivity = V11.ConnectivityResult(
            core_connected=np.ones(count, dtype=bool),
            touches_boundary=np.zeros(count, dtype=bool),
            component_label=np.zeros(count, dtype=np.int32),
            component_count=1,
        )
        return V11.ScaleAnalysis(
            name=name,
            roi=roi,
            prediction=prediction,
            candidate_mode=modes,
            connectivity=connectivity,
            classification=classification,
        )

    def test_near_points_in_different_review_clips_do_not_cross_validate(self):
        fine = self.analysis("fine", [[0.0, 0.0, 0.10]], clip_id=0)
        coarse = self.analysis("coarse", [[0.0, 0.0, 0.10]], clip_id=1)
        fine, coarse = V11.enforce_bidirectional_cross_scale_agreement(
            fine, coarse, maximum_distance_m=0.008
        )
        self.assertFalse(np.any(fine.classification.auto_remove_mask))
        self.assertFalse(np.any(coarse.classification.auto_remove_mask))
        self.assertEqual(
            fine.classification.disposition[0],
            V11.obstruction.PointDisposition.REVIEW,
        )


class PipelineFixture:
    DTYPE = np.dtype(
        [
            ("x", "<f4"),
            ("y", "<f4"),
            ("z", "<f4"),
            ("red", "u1"),
            ("green", "u1"),
            ("blue", "u1"),
            ("scalar_Intensity", "<f4"),
            ("scalar_ScanID", "<f4"),
        ]
    )

    CENTERS = [(0.08, 0.08), (0.15, 0.08), (0.08, 0.15), (0.27, 0.30), (0.33, 0.30)]

    @classmethod
    def records(cls, spacing: float) -> np.ndarray:
        axis = np.arange(0.0, 0.400001, spacing)
        x, y = np.meshgrid(axis, axis)
        terrain = np.zeros(x.size, dtype=cls.DTYPE)
        terrain["x"] = x.ravel()
        terrain["y"] = y.ravel()
        terrain["z"] = 0.25 + 0.02 * terrain["x"] - 0.01 * terrain["y"]
        terrain["red"], terrain["green"], terrain["blue"] = 130, 95, 55
        terrain["scalar_Intensity"] = 280_000
        terrain["scalar_ScanID"] = 4

        object_parts = []
        for center_index, (cx, cy) in enumerate(cls.CENTERS):
            ox, oy = np.meshgrid(
                np.arange(cx - 0.018, cx + 0.018001, spacing),
                np.arange(cy - 0.018, cy + 0.018001, spacing),
            )
            part = np.zeros(ox.size, dtype=cls.DTYPE)
            part["x"], part["y"] = ox.ravel(), oy.ravel()
            # A connected vertical/body band, safely above all seed thresholds.
            part["z"] = 0.43 + 0.02 * part["x"] - 0.01 * part["y"]
            part["red"], part["green"], part["blue"] = 35, 55, 90
            part["scalar_Intensity"] = 750_000 + center_index * 10_000
            part["scalar_ScanID"] = 4
            object_parts.append(part)
        result = np.concatenate((terrain, *object_parts))
        # The elevated ScanID=9 point must be retained despite all other evidence.
        result["scalar_ScanID"][-1] = 9
        return result

    @classmethod
    def write_ply(cls, path: Path, spacing: float) -> np.ndarray:
        records = cls.records(spacing)
        header = (
            "ply\n"
            "format binary_little_endian 1.0\n"
            "comment synthetic obstruction pipeline fixture\n"
            f"element vertex {len(records)}\n"
            "property float x\n"
            "property float y\n"
            "property float z\n"
            "property uchar red\n"
            "property uchar green\n"
            "property uchar blue\n"
            "property float scalar_Intensity\n"
            "property float scalar_ScanID\n"
            "end_header\n"
        ).encode("ascii")
        with path.open("wb") as handle:
            handle.write(header)
            records.tofile(handle)
        return records

    @classmethod
    def write_config(cls, path: Path) -> None:
        def mark(mark_id, world, bbox, evidence):
            return {
                "id": mark_id,
                "world": list(world),
                "review_bbox": list(bbox),
                "screenshot_evidence": evidence,
            }

        config = {
            "marked_locations": {
                "image_1": [
                    mark("image_1_mark_5", (0.08, 0.08), (0.05, 0.11, 0.05, 0.11), "Plan-view bag candidate."),
                    mark("image_1_mark_6", (0.15, 0.08), (0.12, 0.18, 0.05, 0.11), "Plan-view seated-person candidate."),
                    mark("image_1_mark_7", (0.08, 0.15), (0.05, 0.11, 0.12, 0.18), "Plan-view bag candidate."),
                    mark("image_1_mark_8", (0.30, 0.30), (0.22, 0.38, 0.22, 0.38), "Plan-view group containing two seated-person candidates."),
                ]
            },
            "obstruction_review": {
                "southern_union_bbox": [0.02, 0.20, 0.02, 0.20],
                "northern_people_group_bbox": [0.20, 0.40, 0.20, 0.40],
            },
        }
        path.write_text(json.dumps(config), encoding="utf-8")


class ConfigAndExtractionTests(unittest.TestCase):
    def test_extraction_reads_only_configured_clips(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            config = root / "review.json"
            source = root / "rock.ply"
            PipelineFixture.write_config(config)
            PipelineFixture.write_ply(source, 0.01)
            spec = V11.load_review_spec(config)
            roi = V11.collect_configured_roi_points(
                source,
                spec,
                boundary_guard_m=0.005,
                chunk_records=97,
            )
            self.assertEqual(len(spec.clips), 2)
            self.assertEqual(
                {target.kind for clip in spec.clips for target in clip.targets},
                {"bag", "person"},
            )
            xy = roi.xyz[:, :2]
            inside = np.zeros(len(xy), dtype=bool)
            for clip in spec.clips:
                inside |= V11._bbox_contains(xy, clip.bbox)
            self.assertTrue(np.all(inside))
            self.assertTrue(np.any(roi.core_mask))
            self.assertTrue(np.any(roi.boundary_mask))


class FullCandidatePipelineTests(unittest.TestCase):
    def test_candidate_bundle_is_cross_scale_scan9_safe_and_round_trip_exact(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            config = root / "review.json"
            fine_source = root / "Site1-ROCK-1mm.ply"
            coarse_source = root / "Site1-ROCK-5mm.ply"
            output = root / "candidate-run"
            PipelineFixture.write_config(config)
            fine_records = PipelineFixture.write_ply(fine_source, 0.005)
            coarse_records = PipelineFixture.write_ply(coarse_source, 0.01)
            fine_bytes = fine_source.read_bytes()
            coarse_bytes = coarse_source.read_bytes()
            parameters = V11.PipelineParameters(
                surface=V11.SurfaceParameters(
                    anchor_cell_m=0.02,
                    anchor_quantile=0.15,
                    minimum_points_per_anchor=1,
                    neighbour_count=12,
                    query_chunk_points=2_000,
                ),
                fine_voxel_m=0.008,
                coarse_voxel_m=0.015,
                boundary_guard_m=0.005,
                cross_scale_distance_m=0.008,
                preservation_cell_m=0.02,
                chunk_records=137,
            )
            result = V11.build_obstruction_candidates(
                fine_source,
                coarse_source,
                config,
                output,
                parameters=parameters,
            )
            self.assertGreater(result.fine_removed_count, 0)
            self.assertGreater(result.coarse_removed_count, 0)
            self.assertEqual(fine_source.read_bytes(), fine_bytes)
            self.assertEqual(coarse_source.read_bytes(), coarse_bytes)
            self.assertTrue(V11.verify_round_trip_archive(result.fine_bundle_dir).passed)
            self.assertTrue(V11.verify_round_trip_archive(result.coarse_bundle_dir).passed)

            fine_manifest = json.loads(
                (result.fine_bundle_dir / "manifest.json").read_text()
            )
            coarse_manifest = json.loads(
                (result.coarse_bundle_dir / "manifest.json").read_text()
            )
            self.assertEqual(
                fine_manifest["source"]["sha256"], hashlib.sha256(fine_bytes).hexdigest()
            )
            self.assertEqual(
                coarse_manifest["source"]["sha256"], hashlib.sha256(coarse_bytes).hexdigest()
            )
            self.assertFalse(fine_manifest["canonical_writes"])
            self.assertFalse(coarse_manifest["canonical_writes"])

            def read_records(bundle, manifest):
                candidate = bundle / manifest["candidate"]["path"]
                layout = V11.inspect_named_vertex_ply(candidate)
                return np.memmap(
                    candidate,
                    dtype=layout.dtype,
                    mode="r",
                    offset=layout.payload_offset,
                    shape=(layout.vertex_count,),
                ).copy()

            fine_candidate = read_records(result.fine_bundle_dir, fine_manifest)
            coarse_candidate = read_records(result.coarse_bundle_dir, coarse_manifest)
            self.assertEqual(
                np.count_nonzero(np.isclose(fine_candidate["scalar_ScanID"], 9.0)),
                np.count_nonzero(np.isclose(fine_records["scalar_ScanID"], 9.0)),
            )
            self.assertEqual(
                np.count_nonzero(np.isclose(coarse_candidate["scalar_ScanID"], 9.0)),
                np.count_nonzero(np.isclose(coarse_records["scalar_ScanID"], 9.0)),
            )
            run_manifest = json.loads(result.manifest_path.read_text())
            self.assertFalse(run_manifest["canonical_writes"])
            self.assertTrue(run_manifest["fine"]["preservation"]["passed"])
            self.assertTrue(run_manifest["coarse"]["preservation"]["passed"])
            self.assertTrue(run_manifest["fine"]["round_trip"]["passed"])
            self.assertTrue(run_manifest["coarse"]["round_trip"]["passed"])
            self.assertEqual(run_manifest["parameters"], V11.asdict(parameters))
            self.assertEqual(
                set(run_manifest["implementation"]),
                {"site1_v11_obstruction_pipeline.py", "site1_v11_obstructions.py"},
            )


if __name__ == "__main__":
    unittest.main()
