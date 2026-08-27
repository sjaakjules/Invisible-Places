import importlib.util
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts/site1_v12_water_pipeline.py"
SPEC = importlib.util.spec_from_file_location("site1_v12_water_pipeline", SCRIPT)
V12 = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = V12
SPEC.loader.exec_module(V12)
CONFIG = ROOT / "scripts/config/site1_fossils_v12_review.json"


def write_xyz_ply(path: Path, xyz) -> None:
    records = np.asarray(xyz, dtype="<f4").reshape(-1, 3)
    header = (
        "ply\n"
        "format binary_little_endian 1.0\n"
        f"element vertex {len(records)}\n"
        "property float x\n"
        "property float y\n"
        "property float z\n"
        "end_header\n"
    ).encode("ascii")
    path.write_bytes(header + records.tobytes())


class DensityConfigTests(unittest.TestCase):
    def test_search_support_and_density_controls_come_from_review_config(self):
        document = json.loads(CONFIG.read_text(encoding="utf-8"))
        parameters = document["parameters"]
        specs = V12.load_circle_specs(CONFIG)
        by_kind = {}
        for spec in specs:
            by_kind.setdefault(spec.kind, set()).add(
                spec.maximum_water_support_distance_m
            )
        self.assertEqual(
            by_kind["interface"], {parameters["interface_search_distance_m"]}
        )
        self.assertEqual(by_kind["hole"], {parameters["hole_search_distance_m"]})
        self.assertEqual(
            by_kind["dip"], {parameters["density_dip_search_distance_m"]}
        )
        self.assertEqual(
            by_kind["fade"], {parameters["clustered_fade_support_distance_m"]}
        )

        settings = V12._density_continuity_settings(parameters)
        self.assertEqual(settings.audit_radius_m, parameters["density_audit_radius_m"])
        self.assertEqual(settings.minimum_ratio, parameters["density_minimum_ratio"])
        self.assertEqual(settings.maximum_ratio, parameters["density_maximum_ratio"])
        self.assertEqual(
            settings.reservoir_margin_m,
            parameters["density_repair_reservoir_margin_m"],
        )
        self.assertEqual(
            settings.support_margin_m,
            parameters["density_repair_support_margin_m"],
        )
        far_lobe = V12._far_lobe_cull_settings(document, parameters)
        self.assertEqual(far_lobe.grid_pitch_m, parameters["far_lobe_grid_pitch_m"])
        self.assertEqual(
            far_lobe.detachment_gap_m,
            parameters["far_lobe_detachment_gap_m"],
        )

    def test_repair_reservoir_covers_complete_boundary_windows(self):
        document = json.loads(CONFIG.read_text(encoding="utf-8"))
        settings = V12._density_continuity_settings(document["parameters"])
        specs = V12.load_circle_specs(CONFIG)
        reservoir = V12._density_reservoir_specs(specs, settings)
        original = {item.label: item for item in specs}
        for expanded in reservoir:
            source = original[expanded.label]
            if source.kind in V12.DENSITY_CONTINUITY_KINDS:
                self.assertGreaterEqual(
                    expanded.radius_m - source.radius_m + 1.0e-12,
                    settings.audit_radius_m,
                )
                self.assertAlmostEqual(
                    expanded.maximum_water_support_distance_m,
                    source.maximum_water_support_distance_m
                    + settings.support_margin_m,
                )
            else:
                self.assertEqual(expanded, source)

    def test_every_interface_hole_and_dip_has_release_gated_centres(self):
        document = json.loads(CONFIG.read_text(encoding="utf-8"))
        settings = V12._density_continuity_settings(document["parameters"])
        specs = V12.load_circle_specs(CONFIG)
        contract = V12._moving_audit_centre_contract(
            specs, step_m=settings.audit_step_m
        )
        self.assertEqual(len(contract.centres_xy), len(contract.spec_id))
        self.assertEqual(len(contract.centres_xy), len(contract.spec_kind))
        self.assertEqual(len(contract.centres_xy), len(contract.spec_label))
        self.assertEqual(
            set(contract.spec_kind), V12.DENSITY_CONTINUITY_KINDS
        )
        expected = {
            item.identifier
            for item in specs
            if item.kind in V12.DENSITY_CONTINUITY_KINDS
        }
        self.assertEqual(set(contract.spec_id), expected)
        self.assertNotIn("clustered_fade", contract.spec_id)

    def test_density_required_mask_uses_exact_fillable_support_and_keeps_each_spec(self):
        contract = V12.DensityAuditCentres(
            centres_xy=np.column_stack((np.arange(7), np.zeros(7))),
            spec_id=(
                "interface_a", "interface_a", "interface_b",
                "hole_a", "hole_a", "dip_a", "dip_a",
            ),
            spec_kind=(
                "interface", "interface", "interface",
                "hole", "hole", "dip", "dip",
            ),
            spec_label=np.arange(7, dtype=np.int32),
        )
        surface = np.asarray([True, True, True, True, False, True, True])
        fillable = np.asarray([True, True, True, True, False, True, True])
        required = V12._density_required_mask(contract, surface, fillable)
        np.testing.assert_array_equal(
            required, [True, True, True, True, False, True, True]
        )
        for identifier in set(contract.spec_id):
            member = np.asarray([value == identifier for value in contract.spec_id])
            self.assertTrue(np.any(required & member))

        with self.assertRaisesRegex(RuntimeError, "interface_b"):
            V12._density_required_mask(
                contract,
                surface & np.asarray([True, True, False, True, True, True, True]),
                fillable,
            )

    def test_surface_disk_intersection_keeps_a_centre_just_outside_footprint(self):
        from scipy import ndimage

        grid = V12.v10.v6.GridSpec(0.0, 0.0, 1.0, 1.0, 0.05)
        wet = np.zeros(grid.shape, dtype=bool)
        wet[:, : grid.nx // 2] = True
        signed = (
            ndimage.distance_transform_edt(wet)
            - ndimage.distance_transform_edt(~wet)
        ).astype(np.float32)
        surface = SimpleNamespace(
            grid=grid,
            signed_cells=signed,
            exclusion=np.zeros(grid.shape, dtype=bool),
        )
        result = V12._surface_intersects_audit_disks(
            surface,
            np.asarray([[0.525, 0.50], [0.675, 0.50]]),
            radius_m=0.08,
        )
        # The first centre is outside the footprint, but its 8 cm audit disk
        # crosses the shoreline.  The second disk is wholly outside.
        np.testing.assert_array_equal(result, [True, False])

    def test_surface_disk_support_respects_raster_exclusion(self):
        grid = V12.v10.v6.GridSpec(0.0, 0.0, 1.0, 1.0, 0.01)
        signed = np.ones(grid.shape, dtype=np.float32)
        exclusion = np.zeros(grid.shape, dtype=bool)
        exclusion[42:59, 42:59] = True
        surface = SimpleNamespace(
            grid=grid,
            signed_cells=signed,
            exclusion=exclusion,
        )
        support = V12._surface_audit_disk_support(
            surface,
            np.asarray([[0.50, 0.50], [0.25, 0.25]]),
            radius_m=0.08,
            sample_pitch_m=0.002,
        )
        self.assertGreater(
            int(support.valid_footprint_sample_count[1]),
            int(support.valid_footprint_sample_count[0]),
        )
        np.testing.assert_array_equal(support.active_mask, [False, True])
        np.testing.assert_allclose(
            support.valid_footprint_area_m2,
            support.valid_footprint_sample_count * 0.002**2,
        )

    def test_fillable_support_cells_remove_oversample_multiplicity(self):
        cells = V12._unique_fillable_support_cells(
            np.asarray([
                [0.0001, 0.0001],
                [0.0019, 0.0019],
                [0.0021, 0.0001],
            ]),
            pitch_m=0.002,
        )
        self.assertEqual(len(cells.cell_keys), 2)
        self.assertEqual(cells.cell_area_m2, 0.002**2)
        np.testing.assert_array_equal(cells.cell_keys, [[0, 0], [1, 0]])
        np.testing.assert_allclose(
            cells.representative_xy,
            [[0.0001, 0.0001], [0.0021, 0.0001]],
        )

    def test_boundary_support_counts_real_safe_representative_not_cell_centre(self):
        cells = V12._unique_fillable_support_cells(
            np.asarray([[0.07999, 0.00199]]),
            pitch_m=0.002,
        )
        centre_count = V12._circle_point_counts(
            np.asarray([[0.0, 0.0]]),
            cells.cell_centres_xy,
            radius_m=0.08,
        )
        representative_count = V12._circle_point_counts(
            np.asarray([[0.0, 0.0]]),
            cells.representative_xy,
            radius_m=0.08,
        )
        np.testing.assert_array_equal(centre_count, [1])
        np.testing.assert_array_equal(representative_count, [0])

    def test_support_representative_is_nearest_real_point_to_cell_centre(self):
        cells = V12._unique_fillable_support_cells(
            np.asarray([
                [0.0001, 0.0001],
                [0.0011, 0.0010],
                [0.0019, 0.0019],
            ]),
            pitch_m=0.002,
        )
        np.testing.assert_allclose(cells.representative_xy, [[0.0011, 0.0010]])

    def test_vacant_candidate_mask_honours_spacing_boundary_and_tolerance(self):
        candidates = np.asarray([
            [0.00189, 0.0],
            [0.00190, 0.0],
            [0.00200, 0.0],
        ])
        keep = V12._vacant_candidate_mask(
            candidates,
            np.asarray([[0.0, 0.0]]),
            spacing_m=0.002,
            distance_tolerance_m=0.0001,
        )
        np.testing.assert_array_equal(keep, [False, True, True])
        np.testing.assert_array_equal(
            V12._vacant_candidate_mask(
                candidates,
                np.empty((0, 2)),
                spacing_m=0.002,
            ),
            [True, True, True],
        )

    def test_immutable_clearance_shrinks_raw_support_cells_and_area(self):
        candidates = np.asarray([
            [0.0001, 0.0001],
            [0.0015, 0.0001],
            [0.0021, 0.0001],
            [0.0041, 0.0001],
        ])
        blockers = np.asarray([[0.0, 0.0]])
        raw = V12._unique_fillable_support_cells(candidates, pitch_m=0.002)
        vacant_mask = V12._vacant_candidate_mask(
            candidates,
            blockers,
            spacing_m=0.0018,
        )
        vacant = V12._unique_fillable_support_cells(
            candidates[vacant_mask],
            pitch_m=0.002,
        )
        self.assertEqual(len(raw.cell_keys), 3)
        self.assertEqual(len(vacant.cell_keys), 2)
        self.assertGreater(
            len(raw.cell_keys) * raw.cell_area_m2,
            len(vacant.cell_keys) * vacant.cell_area_m2,
        )
        self.assertTrue(np.all(
            V12._nearest_distance(vacant.representative_xy, blockers)
            >= 0.0018 - 1.0e-12
        ))

    def test_xy_is_quantized_before_spacing_at_scene_world_scale(self):
        record_dtype = np.dtype([
            ("x", "<f4"), ("y", "<f4"), ("z", "<f4")
        ])
        raw = np.asarray([
            [780.0, 820.0],
            [780.0 + 0.0018, 820.0],
        ], np.float64)
        self.assertGreaterEqual(
            float(np.linalg.norm(raw[1] - raw[0])), 0.0018 - 1e-12
        )
        stored = V12._roundtrip_xy_to_record_dtype(raw, record_dtype)
        stored_distance = float(np.linalg.norm(stored[1] - stored[0]))
        self.assertLess(stored_distance, 0.0018)
        self.assertAlmostEqual(stored_distance, 0.00177001953125)
        keep = V12._vacant_candidate_mask(
            stored[1:], stored[:1], spacing_m=0.0018
        )
        np.testing.assert_array_equal(keep, [False])

    def test_zero_capacity_does_not_deactivate_vacant_registered_support(self):
        audit = V12.DensityAuditCentres(
            centres_xy=np.asarray([[0.0, 0.0]]),
            spec_id=("interface_density",),
            spec_kind=("interface",),
            spec_label=np.asarray([1], np.int32),
        )
        required = V12._density_required_mask(
            audit,
            np.asarray([True]),
            np.asarray([True]),
        )
        contract = V12.refinement.attainable_addition_density_contract(
            [0],
            [1.0],
            [0],
            active_centre_mask=required,
        )
        np.testing.assert_array_equal(required, [True])
        np.testing.assert_array_equal(contract.capacity_sufficient_mask, [False])

    def test_density_capacity_preempts_only_conflicting_advisory_fade(self):
        capacity = np.asarray([
            [780.0, 820.0],
            [780.004, 820.0],
        ], np.float64)
        fade = np.asarray([
            [780.001, 820.0],
            [780.002, 820.0],
            [780.010, 820.0],
        ], np.float64)
        keep = V12._fixed_fade_survival_mask(
            fade,
            capacity,
            spacing_m=0.0018,
        )
        np.testing.assert_array_equal(keep, [False, True, True])
        kept_distance = V12._nearest_distance(fade[keep], capacity)
        self.assertTrue(np.all(kept_distance >= 0.0018 - 1e-12))
        preempted_distance = V12._nearest_distance(fade[~keep], capacity)
        self.assertTrue(np.all(preempted_distance < 0.0018 - 1e-12))

    def test_scarcity_witness_keeps_sole_in_disk_candidate(self):
        centres = np.asarray([[0.0, 0.0]], np.float64)
        candidates = np.asarray([
            [0.0790, 0.0],   # sole row inside the 8 cm audit disk
            [0.0805, 0.0],   # outside, but close enough to block it
        ], np.float64)
        arbitrary = V12.confidence.variable_radius_blue_noise(
            candidates,
            0.0018,
            priority=np.asarray([0.0, 10.0]),
            seed=17,
        )
        self.assertEqual(arbitrary.selected_indices.tolist(), [1])
        self.assertEqual(
            int(V12._circle_point_counts(
                centres,
                candidates[arbitrary.selected_indices],
                radius_m=0.08,
            )[0]),
            0,
        )

        witness = V12.refinement.refill_circular_density_dips(
            centres,
            np.empty((0, 2), np.float64),
            candidates,
            radius_m=0.08,
            target_density_per_m2=[1.0 / (np.pi * 0.08**2)],
            water_spacing_m=0.0018,
            minimum_observed_count=[1],
            active_centre_mask=[True],
            seed=17,
        )
        self.assertEqual(witness.selected_candidate_indices.tolist(), [0])
        np.testing.assert_array_equal(witness.remaining_deficit_count, [0])

    def test_reference_centres_are_circular_annuli_not_square_cells(self):
        document = json.loads(CONFIG.read_text(encoding="utf-8"))
        settings = V12._density_continuity_settings(document["parameters"])
        specs = V12.load_circle_specs(CONFIG)
        good = V12._good_overlap_spec(document)
        local, overlap = V12._density_reference_centres(specs, good, settings)
        self.assertGreater(len(local), settings.minimum_reference_windows)
        self.assertGreater(len(overlap), settings.minimum_reference_windows)
        for spec in (
            item for item in specs
            if item.kind in V12.DENSITY_CONTINUITY_KINDS
        ):
            single, _ = V12._density_reference_centres((spec,), good, settings)
            selected = np.linalg.norm(single - np.asarray(spec.center_xy), axis=1)
            self.assertTrue(np.all(
                selected
                >= spec.radius_m * settings.local_reference_inner_fraction - 1e-12
            ))
            self.assertLessEqual(
                float(np.max(selected)),
                spec.radius_m + settings.local_reference_outer_margin_m + 1e-12,
            )
        overlap_distance = np.linalg.norm(
            overlap - np.asarray(good.center_xy), axis=1
        )
        self.assertLessEqual(
            float(np.max(overlap_distance)),
            good.radius_m - settings.audit_radius_m + 1e-12,
        )

    def test_terrain_boundary_mask_is_independent_of_water(self):
        centres = np.asarray([[0.0, 0.0], [0.08, 0.0], [0.16, 0.0]])
        mask = V12._terrain_boundary_centres(
            centres,
            np.asarray([0, 10, 10]),
            np.asarray([5, 10, 10]),
            step_m=0.08,
        )
        np.testing.assert_array_equal(mask, [True, True, False])

    def test_complete_clearance_marks_both_candidates_at_terrain_midpoint(self):
        with tempfile.TemporaryDirectory() as temporary:
            terrain = Path(temporary) / "terrain.ply"
            write_xyz_ply(terrain, [[0.0, 0.0, 0.0]])
            candidates = np.asarray([
                [-0.0009, 0.0, 0.0],
                [0.0009, 0.0, 0.0],
            ])
            spec = V12.CircleSpec(
                identifier="midpoint",
                center_xy=(0.0, 0.0),
                radius_m=0.01,
                kind="interface",
                oversample_pitch_m=0.00135,
                maximum_water_support_distance_m=0.12,
                priority=1.0,
                label=1,
            )
            collision, audit = V12._fine_terrain_collision_mask(
                candidates,
                (terrain,),
                (spec,),
                clearance_m=0.001,
                tolerance_m=0.00008,
                chunk_records=10,
            )
        np.testing.assert_array_equal(collision, [True, True])
        self.assertEqual(audit["collisions"], 2)
        self.assertEqual(audit["collision_relations"], 2)
        self.assertTrue(audit["all_clearance_neighbours_enumerated"])

    def test_joint_pool_deduplication_prefers_higher_priority_evidence(self):
        xy = np.asarray([[1.0, 2.0], [1.0, 2.0], [1.1, 2.0]])
        xyz = np.column_stack((xy, [3.0, 3.0, 3.1]))
        result = V12._deduplicate_candidate_pool(
            xyz,
            xy,
            np.asarray([7, 4, 9], np.int16),
            np.asarray([1.0, 5.0, 2.0]),
            np.asarray([1, 2, 3], np.uint8),
            np.asarray([0.02, 0.01, 0.03]),
        )
        out_xyz, out_xy, out_label, out_priority, out_kind, out_distance = result
        self.assertEqual(len(out_xy), 2)
        duplicate = np.flatnonzero(np.all(out_xy == [1.0, 2.0], axis=1))
        self.assertEqual(len(duplicate), 1)
        row = int(duplicate[0])
        self.assertEqual(int(out_label[row]), 4)
        self.assertEqual(float(out_priority[row]), 5.0)
        self.assertEqual(int(out_kind[row]), 2)
        self.assertEqual(float(out_distance[row]), 0.01)
        self.assertEqual(out_xyz.shape, (2, 3))

    def test_failed_density_document_is_compact_and_explains_blockers(self):
        centres = np.asarray([[0.0, 0.0]])
        refill = V12.refinement.refill_circular_density_dips(
            centres,
            np.empty((0, 2), np.float64),
            np.empty((0, 2), np.float64),
            radius_m=0.08,
            target_density_per_m2=[100.0],
            water_spacing_m=0.0018,
            minimum_observed_count=[2],
            maximum_observed_count=[3],
        )
        contract = V12.DensityAuditCentres(
            centres_xy=centres,
            spec_id=("interface_gap_1",),
            spec_kind=("interface",),
            spec_label=np.asarray([3], np.int32),
        )
        document = V12._density_failure_document(
            audit_contract=contract,
            target_combined_count=np.asarray([2.2]),
            combined_lower_count=np.asarray([2]),
            combined_upper_count=np.asarray([2]),
            allowed_combined_upper_count=np.asarray([2]),
            base_water_count=np.asarray([0]),
            terrain_count=np.asarray([0]),
            combined_before_count=np.asarray([0]),
            combined_after_count=np.asarray([0]),
            primary_cleared_count=np.asarray([0]),
            provisional_primary_selected_count=np.asarray([0]),
            repair_raw_count=np.asarray([5]),
            repair_clear_count=np.asarray([0]),
            joint_same_spec_count=np.asarray([0]),
            joint_cross_spec_count=np.asarray([0]),
            refill=refill,
            unresolved_mask=np.asarray([True]),
            required_mask=np.asarray([True]),
            reference_surface_active_mask=np.asarray([True]),
            terrain_boundary_mask=np.asarray([False]),
            source_water_active_mask=np.asarray([True]),
            minimum_ratio=0.85,
            maximum_ratio=1.25,
        )
        self.assertFalse(document["large_point_payload_included"])
        self.assertEqual(document["failed_window_count"], 1)
        self.assertEqual(document["required_window_count"], 1)
        self.assertEqual(
            document["required_windows_by_region"], {"interface_gap_1": 1}
        )
        row = document["failed_windows"][0]
        self.assertEqual(row["spec_id"], "interface_gap_1")
        self.assertEqual(row["required_additions_before"], 2)
        self.assertEqual(row["repair_candidates_before_complete_clearance"], 5)
        self.assertEqual(row["repair_candidates_after_complete_clearance"], 0)
        self.assertEqual(row["joint_candidates"], 0)
        self.assertTrue(row["reference_surface_active"])
        self.assertTrue(row["source_water_active"])
        self.assertFalse(row["terrain_boundary"])

    def test_far_lobe_grid_plan_is_conservative_and_reversible(self):
        main_cells = np.asarray([
            [x, y] for x in range(6) for y in range(6)
        ], np.int64)
        lobe_cells = np.asarray([[30, 0], [31, 0], [30, 1], [31, 1]], np.int64)
        cells = np.concatenate((main_cells, lobe_cells), axis=0)
        settings = V12.FarLobeCullSettings(
            seed_xy=(0.61, 0.01),
            maximum_seed_distance_m=0.08,
            grid_pitch_m=0.02,
            bridge_radius_m=0.02,
            detachment_gap_m=0.20,
            maximum_component_fraction=0.35,
        )
        plan = V12._plan_far_lobe_grid_component(cells, settings)
        self.assertTrue(plan.detached)
        self.assertEqual(plan.selected_occupied_cell_count, 4)
        self.assertGreaterEqual(
            plan.minimum_point_separation_lower_bound_m,
            settings.detachment_gap_m,
        )

        xyz = (cells.astype(np.float64) + 0.5) * settings.grid_pitch_m
        xyz = np.column_stack((xyz, np.linspace(1.0, 2.0, len(xyz))))
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            source = directory / "Site1-WATER-2mm.ply"
            candidate = directory / "Site1-WATER-2mm.geometry-v12.candidate.ply"
            write_xyz_ply(source, xyz)
            archive = V12._archive_far_lobe_source_rows(
                source, plan, directory, chunk_records=7
            )
            self.assertEqual(archive["removed_count"], 4)
            source_info = V12.density.inspect_fixed_stride_ply(source)
            empty = np.empty(0, dtype=source_info.dtype)
            audit = V12.append_records_with_comments(
                source,
                empty,
                candidate,
                comments=("test reversible far-lobe cull",),
                chunk_records=7,
                cull_plan=plan,
                expected_cull_count=4,
            )
            self.assertEqual(audit["surviving_source_points"], len(cells) - 4)
            candidate_info = V12.density.inspect_fixed_stride_ply(candidate)
            candidate_records = np.concatenate([
                np.asarray(records).copy()
                for _, records in V12.density.iter_ply_chunks(
                    candidate, info=candidate_info, chunk_size=7
                )
            ])
            removed_indices = np.fromfile(
                archive["source_indices"]["path"], dtype="<i8"
            )
            removed_records = np.fromfile(
                archive["records"]["path"], dtype=source_info.dtype
            )
            restored = np.empty(len(cells), dtype=source_info.dtype)
            keep = np.ones(len(cells), dtype=bool)
            keep[removed_indices] = False
            restored[keep] = candidate_records
            restored[removed_indices] = removed_records
            original = np.concatenate([
                np.asarray(records).copy()
                for _, records in V12.density.iter_ply_chunks(
                    source, info=source_info, chunk_size=7
                )
            ])
            self.assertEqual(restored.tobytes(), original.tobytes())

            local = V12.LocalRecords(
                source_indices=np.arange(len(original), dtype=np.int64),
                records=original,
            )
            surviving = V12._surviving_local_records(local, plan)
            self.assertEqual(
                surviving.source_indices.tolist(), np.flatnonzero(keep).tolist()
            )
            self.assertEqual(surviving.records.tobytes(), original[keep].tobytes())

    def test_far_lobe_grid_plan_refuses_seeded_main_sheet(self):
        cells = np.asarray([[x, y] for x in range(6) for y in range(6)], np.int64)
        settings = V12.FarLobeCullSettings(
            seed_xy=(0.05, 0.05),
            maximum_seed_distance_m=0.08,
            grid_pitch_m=0.02,
            bridge_radius_m=0.02,
            detachment_gap_m=0.05,
            maximum_component_fraction=0.35,
        )
        plan = V12._plan_far_lobe_grid_component(cells, settings)
        self.assertFalse(plan.detached)
        self.assertIn("largest", plan.reason)


if __name__ == "__main__":
    unittest.main()
