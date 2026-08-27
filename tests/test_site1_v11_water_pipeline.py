import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np


SCRIPT = Path(__file__).resolve().parents[1] / "scripts/site1_v11_water_pipeline.py"
SPEC = importlib.util.spec_from_file_location("site1_v11_water_pipeline", SCRIPT)
PIPELINE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = PIPELINE
SPEC.loader.exec_module(PIPELINE)


SHORT_DTYPE = np.dtype(
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
FULL_DTYPE = np.dtype(SHORT_DTYPE.descr + [("scalar_A_R_Roughness_Combined", "<f4")])


def make_records(xyz, *, dtype=FULL_DTYPE, scan_id=999.0, offset=0.0):
    xyz = np.asarray(xyz, np.float64)
    records = np.zeros(len(xyz), dtype=dtype)
    records["x"], records["y"], records["z"] = xyz.T
    records["red"] = (31 + np.arange(len(records))) % 255
    records["green"] = (61 + np.arange(len(records))) % 255
    records["blue"] = (91 + np.arange(len(records))) % 255
    records["scalar_Intensity"] = 5000.0 + offset + np.arange(len(records))
    records["scalar_ScanID"] = scan_id
    if "scalar_A_R_Roughness_Combined" in (records.dtype.names or ()):
        records["scalar_A_R_Roughness_Combined"] = 0.1 + offset + np.arange(len(records))
    return records


def write_ply(path, records):
    typemap = {
        "f4": "float",
        "f8": "double",
        "u1": "uchar",
        "i1": "char",
        "i2": "short",
        "u2": "ushort",
        "i4": "int",
        "u4": "uint",
    }
    with Path(path).open("wb") as handle:
        handle.write(b"ply\nformat binary_little_endian 1.0\n")
        handle.write(f"element vertex {len(records)}\n".encode("ascii"))
        for name in records.dtype.names:
            code = records.dtype[name].str.lstrip("<>=|")
            handle.write(f"property {typemap[code]} {name}\n".encode("ascii"))
        handle.write(b"end_header\n")
        records.tofile(handle)


def read_ply(path):
    info = PIPELINE.density.inspect_fixed_stride_ply(path)
    mapped = np.memmap(path, dtype=info.dtype, mode="r", offset=info.offset, shape=(info.count,))
    records = np.asarray(mapped).copy()
    del mapped
    return records


def grid_xyz(x_values, y_values, z=0.0):
    x, y = np.meshgrid(np.asarray(x_values), np.asarray(y_values))
    return np.column_stack((x.ravel(), y.ravel(), np.full(x.size, z)))


class ReviewConfigurationTests(unittest.TestCase):
    def test_only_explicit_mark_ids_become_regions(self):
        with tempfile.TemporaryDirectory() as directory:
            config = Path(directory) / "review.json"
            config.write_text(
                json.dumps(
                    {
                        "marked_locations": {
                            "image_1": [
                                {"id": "keep", "review_bbox": [0, 1, 0, 1]},
                                {"id": "ignore", "review_bbox": [5, 6, 5, 6]},
                            ]
                        },
                        "plan_annotations": {
                            "red_density_taper_guide": {"polyline": [[0, 0], [0, 1]]}
                        },
                    }
                ),
                encoding="utf-8",
            )
            regions = PIPELINE.load_interface_review_regions(config, mark_ids=["keep"])
            self.assertEqual([region.id for region in regions], ["keep"])
            np.testing.assert_array_equal(
                regions[0].mask(np.array([[0.5, 0.5], [5.5, 5.5]])),
                [True, False],
            )


class DensityFieldTests(unittest.TestCase):
    def test_measured_field_is_c1_at_guide_and_nonincreasing_outward(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            water_path = root / "water.ply"
            terrain_path = root / "terrain.ply"
            water = make_records(
                grid_xyz(np.arange(-0.45, 1.96, 0.05), np.arange(-0.45, 0.46, 0.05))
            )
            # Terrain is dense west of x=0.4 and naturally sparse to the east.
            dense = grid_xyz(np.arange(-0.45, 0.41, 0.05), np.arange(-0.45, 0.46, 0.05))
            sparse = grid_xyz(np.arange(0.45, 1.96, 0.20), np.arange(-0.45, 0.46, 0.20))
            terrain = make_records(np.concatenate((dense, sparse)), scan_id=2.0)
            write_ply(water_path, water)
            write_ply(terrain_path, terrain)
            field = PIPELINE.build_measured_retention_grid(
                [terrain_path],
                water_path,
                bbox=[-0.5, 2.0, -0.5, 0.5],
                guide_xy=[[0.0, -1.0], [0.0, 1.0]],
                cell_size_m=0.05,
                smoothing_bandwidth_m=0.10,
                taper_end_m=1.0,
                floor_ratio=0.05,
                chunk_size=37,
            )
            query_x = np.linspace(-0.2, 1.9, 100)
            result = field.query(np.column_stack((query_x, np.zeros_like(query_x))))
            self.assertTrue(np.all(result[query_x <= 0.0] > 0.999999))
            self.assertTrue(np.all(np.diff(result[query_x >= 0.0]) <= 1e-12))
            self.assertLess(result[-1], 0.40)
            epsilon = 1e-5
            near = field.query([[-epsilon, 0.0], [epsilon, 0.0]])
            self.assertLess(abs(near[1] - near[0]), 1e-4)
            self.assertEqual(field.summary()["smoothing_bandwidth_m"], 0.10)

    def test_density_reference_changes_when_fine_vs_coarse_terrain_is_used(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            water_path = root / "water.ply"
            fine_path = root / "fine-terrain.ply"
            coarse_path = root / "coarse-terrain.ply"
            write_ply(
                water_path,
                make_records(grid_xyz(np.arange(0.0, 1.01, 0.05), np.arange(0.0, 1.01, 0.05))),
            )
            write_ply(
                fine_path,
                make_records(
                    grid_xyz(np.arange(0.0, 1.01, 0.05), np.arange(0.0, 1.01, 0.05)),
                    scan_id=2.0,
                ),
            )
            write_ply(
                coarse_path,
                make_records(
                    grid_xyz(np.arange(0.0, 1.01, 0.20), np.arange(0.0, 1.01, 0.20)),
                    scan_id=2.0,
                ),
            )
            kwargs = dict(
                current_water_path=water_path,
                bbox=[0.0, 1.0, 0.0, 1.0],
                guide_xy=[[0.0, -1.0], [0.0, 2.0]],
                cell_size_m=0.10,
                smoothing_bandwidth_m=0.15,
                taper_end_m=0.20,
                floor_ratio=0.05,
            )
            fine = PIPELINE.build_measured_retention_grid([fine_path], **kwargs)
            coarse = PIPELINE.build_measured_retention_grid([coarse_path], **kwargs)
            point = np.array([[0.75, 0.5]])
            self.assertGreater(fine.query(point)[0], coarse.query(point)[0])


class StableThinningTests(unittest.TestCase):
    def test_point_hash_is_repeatable_permutation_and_chunk_invariant(self):
        records = make_records(grid_xyz(np.arange(0.0, 1.0, 0.05), [0.0]))
        complete = PIPELINE.stable_point_uniform(records, seed=123)
        chunked = np.concatenate(
            [PIPELINE.stable_point_uniform(records[i : i + 3], seed=123) for i in range(0, len(records), 3)]
        )
        np.testing.assert_array_equal(complete, chunked)
        permutation = np.arange(len(records))[::-1]
        np.testing.assert_array_equal(
            complete[permutation],
            PIPELINE.stable_point_uniform(records[permutation], seed=123),
        )

    def test_candidate_survivors_are_byte_exact_and_index_audit_replays(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source_path = root / "current.ply"
            output_path = root / "Site1-WATER-2mm.v11-candidate.ply"
            index_path = root / "thinned.u64"
            source = make_records(grid_xyz(np.arange(0.0, 1.0, 0.025), [0.0]), offset=20)
            write_ply(source_path, source)
            retention = np.full((2, 20), 0.50, np.float64)
            field = PIPELINE.DensityRetentionGrid(
                bbox=(0.0, 1.0, -0.05, 0.05),
                cell_size_m=0.05,
                retention=retention,
                terrain_density_per_m2=np.ones_like(retention),
                water_density_per_m2=np.ones_like(retention),
                guide_xy=np.array([[-1.0, -1.0], [-1.0, 1.0]]),
                taper_start_m=0.0,
                taper_end_m=1.0,
                floor_ratio=0.5,
                terrain_points_measured=1,
                water_points_measured=len(source),
            )
            thinning = PIPELINE.plan_pointwise_thinning(
                source_path, field, index_path, seed=77, chunk_size=7
            )
            recovery = PIPELINE.ValidatedRecovery(
                records=np.empty(0, dtype=FULL_DTYPE),
                pre_indices=np.empty(0, np.int64),
                audits=(),
                accepted_per_region={},
                scalar_provenance="none",
            )
            PIPELINE.write_refined_candidate(
                source_path,
                output_path,
                field,
                thinning,
                recovery,
                seed=77,
                chunk_size=9,
            )
            rejected = np.fromfile(index_path, dtype="<u8").astype(np.int64)
            expected = np.delete(source, rejected)
            actual = read_ply(output_path)
            self.assertEqual(actual.tobytes(), expected.tobytes())
            self.assertEqual(thinning.source_points, len(source))
            self.assertEqual(thinning.kept_points + thinning.thinned_points, len(source))


class FinalBlockerRecoveryTests(unittest.TestCase):
    def _fixture(self, root):
        current_xyz = np.array(
            [
                [0.10, 0.00, 0.0],
                [0.30, 0.00, 0.0],
                [0.20, 0.15, 0.0],
                [0.50, 0.00, 0.0],
                [0.70, 0.00, 0.0],
                [0.60, 0.15, 0.0],
            ]
        )
        current = make_records(current_xyz, dtype=FULL_DTYPE, offset=100)
        current_path = root / "current.ply"
        write_ply(current_path, current)
        current_short = np.empty(len(current), dtype=SHORT_DTYPE)
        for name in SHORT_DTYPE.names:
            current_short[name] = current[name]
        candidate_xyz = np.array(
            [
                [0.20, 0.00, 0.0],  # 7 cm terrain clearance: eligible
                [0.60, 0.00, 0.0],  # 3 cm terrain clearance: unsafe
                [2.00, 0.00, 0.0],  # outside the review evidence
            ]
        )
        candidates = make_records(candidate_xyz, dtype=SHORT_DTYPE, offset=500)
        pre = np.concatenate((current_short, candidates))
        post = current_short.copy()
        pre_path = root / "combined-pre.ply"
        post_path = root / "combined-post.ply"
        write_ply(pre_path, pre)
        write_ply(post_path, post)
        terrain = make_records(
            [[0.20, 0.07, 0.0], [0.60, 0.03, 0.0], [2.00, 0.07, 0.0]],
            dtype=FULL_DTYPE,
            scan_id=2.0,
        )
        terrain_path = root / "terrain.ply"
        write_ply(terrain_path, terrain)
        return current_path, pre_path, post_path, terrain_path, current, candidates

    def test_exact_subsequence_difference_is_the_only_recovery_pool(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            _, pre, post, _, _, candidates = self._fixture(root)
            region = PIPELINE.ReviewRegion("interface", (0.0, 1.0, -0.2, 0.2))
            removed = PIPELINE.collect_reviewed_final_blocker_rejections(pre, post, [region])
            self.assertEqual(removed.total_removed_records, 3)
            self.assertEqual(removed.total_removed_water_records, 3)
            self.assertEqual(removed.reviewed_removed_water_records, 2)
            self.assertEqual(removed.records.tobytes(), candidates[:2].tobytes())
            np.testing.assert_array_equal(removed.pre_indices, [6, 7])

    def test_3d_clearance_accepts_only_safe_interface_record_and_grafts_scalars(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            current_path, pre, post, terrain, current, candidates = self._fixture(root)
            region = PIPELINE.ReviewRegion("interface", (0.0, 1.0, -0.2, 0.2))
            removed = PIPELINE.collect_reviewed_final_blocker_rejections(pre, post, [region])
            validated = PIPELINE.validate_recovery_candidates(
                removed,
                current_path,
                [terrain],
                [region],
                nominal_spacing_m=0.10,
                relaxed_terrain_ratio=0.60,
                duplicate_clearance_ratio=0.90,
                maximum_bridge_m=0.16,
                minimum_water_support=3,
                chunk_size=2,
            )
            self.assertEqual(len(validated.records), 1)
            recovered = validated.records[0]
            for name in SHORT_DTYPE.names:
                self.assertEqual(recovered[name], candidates[0][name])
            self.assertIn(
                recovered["scalar_A_R_Roughness_Combined"],
                current["scalar_A_R_Roughness_Combined"],
            )
            reasons = {audit.pre_index: audit.reason for audit in validated.audits}
            self.assertEqual(reasons[6], "accepted")
            self.assertEqual(reasons[7], "terrain-clearance-too-small")

    def test_full_schema_coarse_recovery_preserves_the_entire_source_record(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            current_path, _, _, terrain, current, short_candidates = self._fixture(root)
            candidates = np.zeros(len(short_candidates), dtype=FULL_DTYPE)
            for name in SHORT_DTYPE.names:
                candidates[name] = short_candidates[name]
            candidates["scalar_A_R_Roughness_Combined"] = [7.1, 7.2, 7.3]
            pre_path = root / "coarse-pre.ply"
            post_path = root / "coarse-post.ply"
            write_ply(pre_path, np.concatenate((current, candidates)))
            write_ply(post_path, current)
            region = PIPELINE.ReviewRegion("interface", (0.0, 1.0, -0.2, 0.2))
            removed = PIPELINE.collect_reviewed_final_blocker_rejections(
                pre_path, post_path, [region]
            )
            validated = PIPELINE.validate_recovery_candidates(
                removed,
                current_path,
                [terrain],
                [region],
                nominal_spacing_m=0.10,
                relaxed_terrain_ratio=0.60,
                duplicate_clearance_ratio=0.90,
                maximum_bridge_m=0.16,
                minimum_water_support=3,
            )
            self.assertEqual(len(validated.records), 1)
            self.assertEqual(validated.records[0].tobytes(), candidates[0].tobytes())
            self.assertEqual(validated.scalar_provenance, "exact-pre-allterrain-record")

    def test_recovery_support_is_rechecked_after_thinning(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            current_path, pre, post, terrain, _, _ = self._fixture(root)
            region = PIPELINE.ReviewRegion("interface", (0.0, 1.0, -0.2, 0.2))
            removed = PIPELINE.collect_reviewed_final_blocker_rejections(
                pre, post, [region]
            )
            rejected = root / "thinned.u64"
            np.asarray([0, 1, 2], dtype="<u8").tofile(rejected)
            validated = PIPELINE.validate_recovery_candidates(
                removed,
                current_path,
                [terrain],
                [region],
                nominal_spacing_m=0.10,
                relaxed_terrain_ratio=0.60,
                duplicate_clearance_ratio=0.90,
                maximum_bridge_m=0.16,
                minimum_water_support=3,
                thinning_rejected_index_path=rejected,
            )
            self.assertEqual(len(validated.records), 0)
            reasons = {audit.pre_index: audit.reason for audit in validated.audits}
            self.assertEqual(reasons[6], "insufficient-surviving-water-support")

    def test_recovery_must_pass_the_final_retention_field(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            current_path, pre, post, terrain, _, _ = self._fixture(root)
            region = PIPELINE.ReviewRegion("interface", (0.0, 1.0, -0.2, 0.2))
            removed = PIPELINE.collect_reviewed_final_blocker_rejections(
                pre, post, [region]
            )
            zeros = np.zeros((2, 10), np.float64)
            grid = PIPELINE.DensityRetentionGrid(
                bbox=(0.0, 1.0, -0.2, 0.2),
                cell_size_m=0.1,
                retention=zeros,
                terrain_density_per_m2=zeros,
                water_density_per_m2=np.ones_like(zeros),
                guide_xy=np.array([[0.0, -1.0], [0.0, 1.0]]),
                taper_start_m=0.0,
                taper_end_m=0.5,
                floor_ratio=0.0,
                terrain_points_measured=0,
                water_points_measured=0,
            )
            validated = PIPELINE.validate_recovery_candidates(
                removed,
                current_path,
                [terrain],
                [region],
                nominal_spacing_m=0.10,
                relaxed_terrain_ratio=0.60,
                duplicate_clearance_ratio=0.90,
                maximum_bridge_m=0.16,
                minimum_water_support=3,
                retention_grid=grid,
                thinning_seed=42,
            )
            self.assertEqual(len(validated.records), 0)
            self.assertTrue(all(
                audit.reason == "density-taper-rejected"
                for audit in validated.audits
            ))

    def test_non_subsequence_rewrite_fails_closed(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            pre = make_records([[0, 0, 0], [1, 0, 0]], dtype=SHORT_DTYPE)
            post = pre[[1, 0]].copy()
            pre_path, post_path = root / "pre.ply", root / "post.ply"
            write_ply(pre_path, pre)
            write_ply(post_path, post)
            with self.assertRaisesRegex(RuntimeError, "subsequence"):
                PIPELINE.collect_reviewed_final_blocker_rejections(
                    pre_path,
                    post_path,
                    [PIPELINE.ReviewRegion("r", (-1, 2, -1, 1))],
                )


class EndToEndCandidateOnlyTests(unittest.TestCase):
    def test_pipeline_writes_hash_locked_candidate_and_strict_json_audits(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            helper = FinalBlockerRecoveryTests()
            current, pre, post, terrain, _, _ = helper._fixture(root)
            config = root / "review.json"
            config.write_text(
                json.dumps(
                    {
                        "marked_locations": {
                            "image": [{"id": "interface", "review_bbox": [0.0, 1.0, -0.2, 0.2]}]
                        },
                        "plan_annotations": {
                            "red_density_taper_guide": {
                                "polyline": [[0.0, -1.0], [0.0, 1.0]]
                            }
                        },
                    }
                ),
                encoding="utf-8",
            )
            output = root / "Site1-WATER-2mm.v11-candidate.ply"
            audit = root / "water-audit.json"
            recovery_audit = root / "recovery-audit.json"
            rejected = root / "thinned.u64"
            result = PIPELINE.run_candidate_only_refinement(
                current_water_path=current,
                pre_allterrain_path=pre,
                post_allterrain_path=post,
                terrain_paths=[terrain],
                config_path=config,
                output_path=output,
                audit_path=audit,
                recovery_audit_path=recovery_audit,
                rejected_index_path=rejected,
                taper_bbox=[0.0, 1.0, -0.2, 0.2],
                nominal_spacing_m=0.10,
                interface_mark_ids=["interface"],
                cell_size_m=0.10,
                smoothing_bandwidth_m=0.10,
                taper_end_m=0.50,
                floor_ratio=1.00,
                relaxed_terrain_ratio=0.60,
                duplicate_clearance_ratio=0.90,
                maximum_bridge_m=0.16,
                minimum_water_support=3,
                seed=42,
                chunk_size=2,
            )
            self.assertTrue(output.exists())
            self.assertEqual(result.candidate_sha256, PIPELINE.sha256_path(output))
            report = json.loads(audit.read_text(encoding="utf-8"))
            recovery_report = json.loads(recovery_audit.read_text(encoding="utf-8"))
            self.assertTrue(report["candidate_only"])
            self.assertFalse(report["canonical_install_performed"])
            self.assertFalse(report["invariants"]["true_hole_synthesis_performed"])
            self.assertEqual(recovery_report["accepted"], 1)
            self.assertEqual(len(report["sources"]), 4)
            self.assertTrue(all(value["sha256"] for value in report["sources"].values()))
            self.assertEqual(report["config"]["sha256"], PIPELINE.sha256_path(config))
            self.assertTrue(report["recovery"]["support_is_post_thinning"])

    def test_pipeline_refuses_canonical_output(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(ValueError, "canonical"):
                PIPELINE._candidate_output_path(
                    Path(directory) / "Site1-WATER-5mm.ply", []
                )

    def test_pipeline_refuses_canonical_sidecar_before_any_write(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            canonical = root / "Site1-SAND-1mm.ply"
            canonical.write_bytes(b"protected-canonical")
            with self.assertRaisesRegex(ValueError, "canonical"):
                PIPELINE.run_candidate_only_refinement(
                    current_water_path=root / "missing-current.ply",
                    pre_allterrain_path=root / "missing-pre.ply",
                    post_allterrain_path=root / "missing-post.ply",
                    terrain_paths=[root / "missing-terrain.ply"],
                    config_path=root / "missing-config.json",
                    output_path=root / "candidate.ply",
                    audit_path=canonical,
                    recovery_audit_path=root / "recovery.json",
                    rejected_index_path=root / "rejected.u64",
                    taper_bbox=[0.0, 1.0, 0.0, 1.0],
                    nominal_spacing_m=0.005,
                    overwrite=True,
                )
            self.assertEqual(canonical.read_bytes(), b"protected-canonical")

    def test_pipeline_refuses_config_alias_before_any_write(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = root / "review.json"
            config.write_bytes(b"protected-config")
            with self.assertRaisesRegex(ValueError, "protected"):
                PIPELINE.run_candidate_only_refinement(
                    current_water_path=root / "missing-current.ply",
                    pre_allterrain_path=root / "missing-pre.ply",
                    post_allterrain_path=root / "missing-post.ply",
                    terrain_paths=[root / "missing-terrain.ply"],
                    config_path=config,
                    output_path=root / "candidate.ply",
                    audit_path=config,
                    recovery_audit_path=root / "recovery.json",
                    rejected_index_path=root / "rejected.u64",
                    taper_bbox=[0.0, 1.0, 0.0, 1.0],
                    nominal_spacing_m=0.005,
                    overwrite=True,
                )
            self.assertEqual(config.read_bytes(), b"protected-config")

    def test_pipeline_refuses_resolved_artifact_alias(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            output = root / "candidate.ply"
            alias = root / "candidate-link.ply"
            alias.symlink_to(output.name)
            with self.assertRaisesRegex(ValueError, "distinct"):
                PIPELINE.run_candidate_only_refinement(
                    current_water_path=root / "missing-current.ply",
                    pre_allterrain_path=root / "missing-pre.ply",
                    post_allterrain_path=root / "missing-post.ply",
                    terrain_paths=[root / "missing-terrain.ply"],
                    config_path=root / "missing-config.json",
                    output_path=output,
                    audit_path=alias,
                    recovery_audit_path=root / "recovery.json",
                    rejected_index_path=root / "rejected.u64",
                    taper_bbox=[0.0, 1.0, 0.0, 1.0],
                    nominal_spacing_m=0.005,
                    overwrite=True,
                )


if __name__ == "__main__":
    unittest.main()
