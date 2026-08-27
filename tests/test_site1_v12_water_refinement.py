import importlib.util
from pathlib import Path
import sys
import unittest

import numpy as np


SCRIPT = (
    Path(__file__).resolve().parents[1]
    / "scripts/site1_v12_water_refinement.py"
)
SPEC = importlib.util.spec_from_file_location(
    "site1_v12_water_refinement", SCRIPT
)
V12 = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = V12
SPEC.loader.exec_module(V12)


def grid_xy(start, stop, spacing):
    values = np.arange(start, stop + spacing * 0.25, spacing)
    x, y = np.meshgrid(values, values)
    return np.column_stack((x.ravel(), y.ravel()))


def flat_terrain(spacing=0.002):
    xy = grid_xy(-0.010, 0.010, spacing)
    z = 0.08 * xy[:, 0] - 0.03 * xy[:, 1]
    return np.column_stack((xy, z))


class CorrelatedFadeTests(unittest.TestCase):
    def test_fade_is_correlated_calibrated_and_chunk_permutation_invariant(self):
        x = np.arange(779.0, 780.0, 0.005)
        y = np.arange(816.0, 816.4, 0.005)
        xx, yy = np.meshgrid(x, y)
        points = np.column_stack((xx.ravel(), yy.ravel()))
        noise = V12.multiscale_correlated_noise(points, seed=771)
        bias = V12.solve_logistic_bias(noise, 0.37, strength=4.5)
        whole = V12.correlated_fade_selection(
            points, bias=bias, strength=4.5, seed=771
        )
        self.assertAlmostEqual(np.mean(whole.probability), 0.37, places=12)
        self.assertLess(abs(np.mean(whole.keep_mask) - 0.37), 0.025)

        # Adjacent samples share a slowly varying retention field rather than
        # an independent or bbox-cell reset pattern.
        noise_image = whole.noise.reshape(len(y), len(x))
        adjacent_correlation = np.corrcoef(
            noise_image[:, :-1].ravel(), noise_image[:, 1:].ravel()
        )[0, 1]
        self.assertGreater(adjacent_correlation, 0.97)
        keep_image = whole.keep_mask.reshape(len(y), len(x))
        adjacent_keep_agreement = np.mean(
            keep_image[:, :-1] == keep_image[:, 1:]
        )
        independent_agreement = (
            np.mean(keep_image) ** 2 + (1.0 - np.mean(keep_image)) ** 2
        )
        self.assertGreater(adjacent_keep_agreement, independent_agreement + 0.05)

        split = 7431  # deliberately cuts through a scan row and noise octave
        left = V12.correlated_fade_selection(
            points[:split], bias=bias, strength=4.5, seed=771
        )
        right = V12.correlated_fade_selection(
            points[split:], bias=bias, strength=4.5, seed=771
        )
        np.testing.assert_array_equal(
            whole.keep_mask,
            np.concatenate((left.keep_mask, right.keep_mask)),
        )
        np.testing.assert_array_equal(
            whole.noise,
            np.concatenate((left.noise, right.noise)),
        )

        rng = np.random.default_rng(12)
        permutation = rng.permutation(len(points))
        permuted = V12.correlated_fade_selection(
            points[permutation], bias=bias, strength=4.5, seed=771
        )
        restored = np.empty(len(points), dtype=bool)
        restored[permutation] = permuted.keep_mask
        np.testing.assert_array_equal(restored, whole.keep_mask)

    def test_noise_has_no_discontinuity_at_an_artificial_crop_edge(self):
        # The middle pair straddles an arbitrary review-bbox boundary.  Its
        # change should tend to zero with separation, not reset to a new phase.
        points = np.array(
            [
                [779.4990, 816.25],
                [779.499999, 816.25],
                [779.500001, 816.25],
                [779.5010, 816.25],
            ]
        )
        noise = V12.multiscale_correlated_noise(points, seed=19)
        edge_jump = abs(noise[2] - noise[1])
        outer_change = max(abs(noise[1] - noise[0]), abs(noise[3] - noise[2]))
        self.assertLess(edge_jump, outer_change * 0.01 + 1.0e-12)


class TerrainGateTests(unittest.TestCase):
    def test_clearance_resolves_point_nine_five_vs_one_point_zero_five_mm(self):
        terrain = flat_terrain()
        # Exercise the same large shifted world coordinates as the Scene1 PLY,
        # not only an origin-centred numerical toy.
        terrain[:, 0] += 780.0
        terrain[:, 1] += 820.0
        terrain[:, 2] += 2.0
        plane_z = 2.0
        candidates = np.array(
            [
                [780.0, 820.0, plane_z + 0.00095],
                [780.0, 820.0, plane_z + 0.00105],
                [780.0, 820.0, plane_z - 0.00105],
            ]
        )
        result = V12.gate_candidates_against_terrain(
            candidates,
            terrain,
            absolute_clearance_m=0.001,
            support_radius_m=0.008,
        )
        np.testing.assert_array_equal(result.accepted_mask, [False, True, False])
        self.assertTrue(
            result.reason_mask[0]
            & int(V12.TerrainRejectReason.ABSOLUTE_CLEARANCE)
        )
        self.assertTrue(
            result.reason_mask[2] & int(V12.TerrainRejectReason.BELOW_TERRAIN)
        )
        self.assertGreater(result.fitted_normal[1, 2], 0.99)

    def test_inverted_and_zero_stored_normals_cannot_change_sided_result(self):
        terrain = flat_terrain()
        candidates = np.array(
            [[0.001, -0.001, 0.00120], [0.001, -0.001, -0.00120]]
        )
        upward = np.tile([0.0, 0.0, 1.0], (len(terrain), 1))
        inverted = -upward
        zero = np.zeros_like(upward)
        baseline = V12.gate_candidates_against_terrain(
            candidates, terrain, terrain_normals=upward
        )
        for normals in (inverted, zero):
            result = V12.gate_candidates_against_terrain(
                candidates, terrain, terrain_normals=normals
            )
            np.testing.assert_array_equal(
                result.accepted_mask, baseline.accepted_mask
            )
            np.testing.assert_allclose(
                result.signed_height_m, baseline.signed_height_m
            )
            np.testing.assert_allclose(
                result.fitted_normal, baseline.fitted_normal
            )
        np.testing.assert_array_equal(baseline.accepted_mask, [True, False])

    def test_terrain_clearance_and_water_spacing_are_independent(self):
        terrain = flat_terrain()
        candidates = np.array(
            [
                [0.0000, 0.0000, 0.00105],
                [0.0015, 0.0000, 0.00117],
                [0.0040, 0.0000, 0.00137],
            ]
        )
        result = V12.select_water_additions(
            candidates,
            terrain,
            existing_water_xy=np.array([[-0.010, -0.010]]),
            terrain_clearance_m=0.001,
            water_spacing_m=0.0019,
            seed=31,
        )
        # All candidates clear terrain by 1.05 mm, while one of the first two
        # must lose the independent 1.9 mm WATER--WATER competition.
        self.assertEqual(len(result.terrain_rejected_indices), 0)
        self.assertEqual(len(result.selected_indices), 2)
        self.assertEqual(len(result.water_spacing_rejected_indices), 1)
        selected = candidates[result.selected_indices, :2]
        self.assertGreaterEqual(
            np.linalg.norm(selected[0] - selected[1]), 0.0019 - 1.0e-12
        )


PLY_DTYPE = np.dtype(
    [
        ("x", "<f4"),
        ("y", "<f4"),
        ("z", "<f4"),
        ("scalar_Intensity", "<f4"),
        ("scalar_ScanID", "<f4"),
        ("payload", "<u4"),
    ]
)


def records_for_points(points):
    points = np.asarray(points, dtype=np.float64)
    records = np.zeros(len(points), dtype=PLY_DTYPE)
    records["x"] = points[:, 0]
    records["y"] = points[:, 1]
    records["z"] = points[:, 2]
    records["scalar_Intensity"] = 5000.0 + np.arange(len(points)) * 13.0
    records["scalar_ScanID"] = 999.0
    records["payload"] = np.arange(len(points), dtype=np.uint32) * 2654435761
    return records


class ReversibleComponentCullTests(unittest.TestCase):
    def test_detached_seeded_component_is_archived_and_exactly_restorable(self):
        main = grid_xy(0.0, 0.15, 0.05)
        lobe = np.array([[0.60, 0.00], [0.64, 0.00], [0.62, 0.035]])
        xy = np.concatenate((main, lobe), axis=0)
        records = records_for_points(
            np.column_stack((xy, np.linspace(1.0, 2.0, len(xy))))
        )
        plan = V12.plan_seeded_component_cull(
            xy,
            [0.62, 0.01],
            connectivity_m=0.071,
            maximum_seed_distance_m=0.08,
            detachment_gap_m=0.20,
            records=records,
        )
        self.assertTrue(plan.detached)
        self.assertEqual(len(plan.cull_indices), 3)
        kept = records[plan.keep_indices].copy()
        restored = V12.restore_seeded_component_cull(kept, plan)
        self.assertEqual(restored.tobytes(), records.tobytes())

    def test_component_cull_fails_closed_when_not_detached_or_seed_hits_main(self):
        main = grid_xy(0.0, 0.15, 0.05)
        lobe = np.array([[0.31, 0.00], [0.35, 0.00], [0.33, 0.035]])
        xy = np.concatenate((main, lobe), axis=0)
        # It is a separate connectivity component, but not separated by the
        # stronger review gap needed to confidently call it a detached lobe.
        near_plan = V12.plan_seeded_component_cull(
            xy,
            [0.33, 0.01],
            connectivity_m=0.071,
            maximum_seed_distance_m=0.08,
            detachment_gap_m=0.20,
        )
        self.assertFalse(near_plan.detached)
        self.assertEqual(len(near_plan.cull_indices), 0)
        self.assertEqual(len(near_plan.keep_indices), len(xy))

        main_plan = V12.plan_seeded_component_cull(
            xy,
            [0.05, 0.05],
            connectivity_m=0.071,
            maximum_seed_distance_m=0.08,
        )
        self.assertFalse(main_plan.detached)
        self.assertIn("largest", main_plan.reason)


class CircularDensityTests(unittest.TestCase):
    def test_targets_scale_water_only_density_by_vacant_support_area(self):
        def cluster(center, count):
            angle = np.linspace(0.0, 2.0 * np.pi, count, endpoint=False)
            return np.column_stack((
                center[0] + 0.01 * np.cos(angle),
                center[1] + 0.01 * np.sin(angle),
            ))

        local_centres = np.array([
            [-0.5, -0.5], [-0.5, 0.5], [0.5, -0.5], [0.5, 0.5]
        ])
        overlap_centres = np.array([
            [2.5, -0.5], [2.5, 0.5], [3.5, -0.5], [3.5, 0.5]
        ])
        local_water = np.concatenate(
            [cluster(center, 8) for center in local_centres], axis=0
        )
        overlap_water = np.concatenate(
            [cluster(center, 5) for center in overlap_centres], axis=0
        )
        overlap_terrain = np.concatenate(
            [cluster(center, 15) for center in overlap_centres], axis=0
        )
        evaluated_water = np.array([[0.0, 0.0], [0.01, 0.0], [3.0, 0.0]])
        evaluated_terrain = cluster([3.0, 0.0], 12)
        radius = 0.11
        disk_area = np.pi * radius * radius
        fillable_area = np.full(2, 0.05 * disk_area)
        result = V12.measured_circular_continuity_targets(
            [[0.0, 0.0], [3.0, 0.0]],
            np.concatenate((local_water, overlap_water, evaluated_water)),
            np.concatenate((overlap_terrain, evaluated_terrain)),
            local_centres,
            overlap_centres,
            radius_m=radius,
            fillable_water_area_m2=fillable_area,
            reference_neighbours=3,
            minimum_reference_windows=3,
        )
        np.testing.assert_array_equal(result.shoreline_mask, [False, True])
        np.testing.assert_allclose(result.raw_desired_addition_count, [0.25, 0.25])
        np.testing.assert_array_equal(result.terrain_count, [0, 12])
        np.testing.assert_allclose(result.fillable_water_area_m2, fillable_area)
        np.testing.assert_allclose(
            result.reference_water_density_per_m2,
            np.full(2, 5.0 / disk_area),
        )
        self.assertEqual(
            result.reference_kind,
            (
                "good-overlap-water-area-normalized",
                "good-overlap-water-area-normalized",
            ),
        )

    def test_targets_fall_back_to_local_shoreline_areal_density(self):
        def cluster(center, count):
            angle = np.linspace(0.0, 2.0 * np.pi, count, endpoint=False)
            return np.column_stack((
                center[0] + 0.01 * np.cos(angle),
                center[1] + 0.01 * np.sin(angle),
            ))

        radius = 0.11
        disk_area = np.pi * radius * radius
        local_centres = np.array([
            [-0.5, -0.5], [-0.5, 0.5], [0.5, -0.5], [0.5, 0.5]
        ])
        local_water = np.concatenate(
            [cluster(center, 4) for center in local_centres], axis=0
        )
        local_terrain = np.concatenate(
            [cluster(center, 6) for center in local_centres], axis=0
        )
        result = V12.measured_circular_continuity_targets(
            [[0.0, 0.0]],
            local_water,
            local_terrain,
            local_centres,
            [[3.0, 3.0]],
            radius_m=radius,
            fillable_water_area_m2=[0.5 * disk_area],
            reference_neighbours=3,
            minimum_reference_windows=3,
        )
        np.testing.assert_allclose(result.raw_desired_addition_count, [2.0])
        np.testing.assert_allclose(
            result.reference_water_density_per_m2,
            [4.0 / disk_area],
        )
        self.assertEqual(
            result.reference_kind,
            ("local-water-area-normalized",),
        )

    def test_addition_contract_counts_immutable_once_and_never_caps_demand(self):
        result = V12.attainable_addition_density_contract(
            [7, 2],
            [10.0, 20.0],
            [9, 30],
            minimum_ratio=0.85,
            maximum_ratio=1.25,
        )
        np.testing.assert_allclose(result.target_addition_count, [10.0, 20.0])
        np.testing.assert_allclose(result.target_water_count, [17.0, 22.0])
        np.testing.assert_array_equal(result.addition_lower_count, [9, 17])
        np.testing.assert_array_equal(result.addition_upper_count, [13, 25])
        np.testing.assert_array_equal(result.water_lower_count, [16, 19])
        np.testing.assert_array_equal(result.water_upper_count, [20, 27])
        np.testing.assert_array_equal(result.capacity_sufficient_mask, [True, True])

    def test_addition_contract_ceil_upper_preserves_one_point_interval(self):
        result = V12.attainable_addition_density_contract(
            [4], [0.49], [1], minimum_ratio=0.85, maximum_ratio=1.25
        )
        np.testing.assert_array_equal(result.addition_lower_count, [1])
        np.testing.assert_array_equal(result.addition_upper_count, [1])
        np.testing.assert_array_equal(result.water_lower_count, [5])
        np.testing.assert_array_equal(result.water_upper_count, [5])

    def test_addition_contract_reports_capacity_failure_without_silent_cap(self):
        result = V12.attainable_addition_density_contract(
            [3], [10.0], [8], minimum_ratio=0.85, maximum_ratio=1.25
        )
        self.assertFalse(result.capacity_sufficient_mask[0])
        self.assertEqual(result.target_addition_count[0], 10.0)
        self.assertEqual(result.addition_lower_count[0], 9)

    def test_moving_windows_detect_and_refill_a_local_dip_without_square_cells(self):
        full = grid_xy(-0.50, 0.50, 0.10)
        hole = np.linalg.norm(full, axis=1) < 0.19
        existing = full[~hole]
        candidate_grid = grid_xy(-0.18, 0.18, 0.04)
        candidates = candidate_grid[np.linalg.norm(candidate_grid, axis=1) < 0.19]
        centres = np.column_stack((np.linspace(-0.10, 0.10, 9), np.zeros(9)))
        result = V12.refill_circular_density_dips(
            centres,
            existing,
            candidates,
            radius_m=0.25,
            target_density_per_m2=100.0,
            water_spacing_m=0.075,
            minimum_ratio=0.80,
            maximum_ratio=1.25,
            seed=25,
        )
        self.assertTrue(np.any(result.before.dip_mask))
        self.assertGreater(len(result.selected_candidate_indices), 0)
        self.assertGreater(
            np.median(result.after.density_ratio),
            np.median(result.before.density_ratio),
        )
        selected = candidates[result.selected_candidate_indices]
        combined = np.concatenate((existing, selected), axis=0)
        for point in selected:
            distance = np.linalg.norm(combined - point, axis=1)
            positive = distance[distance > 1.0e-12]
            self.assertGreaterEqual(np.min(positive), 0.075 - 1.0e-12)
        ceiling = np.ceil(1.25 * result.before.target_count).astype(np.int64)
        originally_below_ceiling = result.before.observed_count <= ceiling
        self.assertTrue(np.any(originally_below_ceiling))
        self.assertTrue(
            np.all(result.after.observed_count[originally_below_ceiling]
                   <= ceiling[originally_below_ceiling])
        )

    def test_circular_refill_rejects_invalid_maximum_ratio(self):
        with self.assertRaisesRegex(ValueError, "maximum_ratio"):
            V12.refill_circular_density_dips(
                [[0.0, 0.0]],
                np.empty((0, 2), dtype=np.float64),
                [[0.0, 0.0]],
                radius_m=0.25,
                target_density_per_m2=10.0,
                water_spacing_m=0.05,
                maximum_ratio=0.99,
            )

    def test_explicit_count_ceiling_prevents_a_new_upper_window(self):
        candidates = np.column_stack((
            np.linspace(-0.30, 0.30, 9),
            np.zeros(9),
        ))
        result = V12.refill_circular_density_dips(
            [[0.0, 0.0]],
            np.empty((0, 2), dtype=np.float64),
            candidates,
            radius_m=0.5,
            target_density_per_m2=8.0 / (np.pi * 0.5**2),
            water_spacing_m=0.05,
            minimum_ratio=0.85,
            maximum_observed_count=[3],
            seed=14,
        )
        self.assertEqual(len(result.selected_candidate_indices), 3)
        self.assertEqual(int(result.after.observed_count[0]), 3)

    def test_explicit_maximum_overrides_ratio_derived_water_ceiling(self):
        radius = 0.5
        target_count = 8.0
        candidates = np.column_stack((
            np.linspace(-0.30, 0.30, 12),
            np.zeros(12),
        ))
        result = V12.refill_circular_density_dips(
            [[0.0, 0.0]],
            np.empty((0, 2), dtype=np.float64),
            candidates,
            radius_m=radius,
            target_density_per_m2=target_count / (np.pi * radius**2),
            water_spacing_m=0.01,
            minimum_observed_count=[12],
            maximum_ratio=1.25,
            maximum_observed_count=[12],
            seed=14,
        )
        # The ratio-only ceiling would be ten.  The explicit count is a
        # caller-computed shoreline contract and must be authoritative.
        self.assertEqual(len(result.selected_candidate_indices), 12)
        np.testing.assert_array_equal(result.remaining_deficit_count, [0])
        np.testing.assert_array_equal(result.after.observed_count, [12])

    def test_active_existing_count_above_explicit_maximum_fails_closed(self):
        with self.assertRaisesRegex(RuntimeError, "existing.*explicit maximum"):
            V12.refill_circular_density_dips(
                [[0.0, 0.0]],
                [[0.0, 0.0]],
                np.empty((0, 2), dtype=np.float64),
                radius_m=0.5,
                target_density_per_m2=1.0,
                water_spacing_m=0.01,
                maximum_observed_count=[0],
            )

    def test_explicit_minimum_above_maximum_fails_closed(self):
        with self.assertRaisesRegex(RuntimeError, "minimum.*maximum"):
            V12.refill_circular_density_dips(
                [[0.0, 0.0]],
                np.empty((0, 2), dtype=np.float64),
                [[0.0, 0.0], [0.1, 0.0]],
                radius_m=0.5,
                target_density_per_m2=1.0,
                water_spacing_m=0.01,
                minimum_observed_count=[2],
                maximum_observed_count=[1],
            )

    def test_inactive_dense_window_cannot_block_an_active_interface_refill(self):
        centres = np.asarray([[0.0, 0.0], [0.04, 0.0]])
        candidate = np.asarray([[0.02, 0.0]])
        radius = 0.031
        result = V12.refill_circular_density_dips(
            centres,
            np.empty((0, 2), dtype=np.float64),
            candidate,
            radius_m=radius,
            target_density_per_m2=np.full(2, 1.0 / (np.pi * radius**2)),
            water_spacing_m=0.0018,
            minimum_observed_count=[1, 0],
            maximum_observed_count=[1, 0],
            active_centre_mask=[True, False],
            seed=14,
        )
        np.testing.assert_array_equal(result.selected_candidate_indices, [0])
        np.testing.assert_array_equal(result.required_minimum_count, [1, 0])
        np.testing.assert_array_equal(result.remaining_deficit_count, [0, 0])
        np.testing.assert_array_equal(result.active_centre_mask, [True, False])
        # The selected point geometrically covers both centres.  Only the
        # candidate-independent active interface centre is density-gated.
        np.testing.assert_array_equal(result.after.observed_count, [1, 1])

    def test_refill_stops_at_explicit_lower_bound_not_full_target(self):
        candidates = np.column_stack((
            np.linspace(-0.40, 0.40, 9),
            np.zeros(9),
        ))
        result = V12.refill_circular_density_dips(
            [[0.0, 0.0]],
            np.empty((0, 2), dtype=np.float64),
            candidates,
            radius_m=0.5,
            target_density_per_m2=8.0 / (np.pi * 0.5**2),
            water_spacing_m=0.05,
            minimum_ratio=0.85,
            minimum_observed_count=[3],
            maximum_observed_count=[8],
            seed=91,
        )
        self.assertEqual(len(result.selected_candidate_indices), 3)
        np.testing.assert_array_equal(result.required_minimum_count, [3])
        np.testing.assert_array_equal(result.remaining_deficit_count, [0])
        np.testing.assert_array_equal(result.selected_count_per_centre, [3])

    def test_multiwindow_candidate_is_preferred_to_two_single_window_points(self):
        centres = np.asarray([[-0.05, 0.0], [0.05, 0.0]])
        # Index zero covers both windows.  The other candidates cover one each,
        # and selecting either would exclude the shared row at 0.15 m spacing.
        candidates = np.asarray([[0.0, 0.0], [-0.10, 0.0], [0.10, 0.0]])
        result = V12.refill_circular_density_dips(
            centres,
            np.empty((0, 2), dtype=np.float64),
            candidates,
            radius_m=0.061,
            target_density_per_m2=np.full(2, 1.0 / (np.pi * 0.061**2)),
            water_spacing_m=0.15,
            minimum_observed_count=[1, 1],
            maximum_observed_count=[2, 2],
            seed=17,
        )
        np.testing.assert_array_equal(result.selected_candidate_indices, [0])
        np.testing.assert_array_equal(result.remaining_deficit_count, [0, 0])

    def test_joint_repack_can_supersede_a_blocking_primary_choice(self):
        centres = np.asarray([[-0.05, 0.0], [0.05, 0.0]])
        target = np.full(2, 1.0 / (np.pi * 0.061**2))
        frozen = V12.refill_circular_density_dips(
            centres,
            [[-0.10, 0.0]],
            [[0.0, 0.0]],
            radius_m=0.061,
            target_density_per_m2=target,
            water_spacing_m=0.15,
            minimum_observed_count=[1, 1],
            maximum_observed_count=[2, 2],
            seed=17,
        )
        self.assertGreater(int(frozen.remaining_deficit_count[1]), 0)

        repacked = V12.refill_circular_density_dips(
            centres,
            np.empty((0, 2), dtype=np.float64),
            [[-0.10, 0.0], [0.0, 0.0]],
            radius_m=0.061,
            target_density_per_m2=target,
            water_spacing_m=0.15,
            minimum_observed_count=[1, 1],
            maximum_observed_count=[2, 2],
            seed=17,
        )
        np.testing.assert_array_equal(repacked.selected_candidate_indices, [1])
        np.testing.assert_array_equal(repacked.remaining_deficit_count, [0, 0])

    def test_repack_preserves_upper_headroom_for_an_adjacent_deficit(self):
        centres = np.asarray([[0.0, 0.0], [1.5, 0.0], [3.0, 0.0]])
        candidates = np.asarray([
            [0.75, 0.0],
            [-0.75, 0.0],
            [2.25, 0.0],
            [2.25, 0.2],
            [2.25, -0.2],
        ])
        radius = 1.01

        def solve(rows):
            return V12.refill_circular_density_dips(
                centres,
                np.empty((0, 2), dtype=np.float64),
                candidates[rows],
                radius_m=radius,
                target_density_per_m2=np.ones(3),
                water_spacing_m=0.1,
                minimum_observed_count=[1, 0, 1],
                maximum_observed_count=[1, 1, 1],
                seed=0,
            )

        first = solve(np.arange(len(candidates)))
        permutation = np.asarray([4, 2, 0, 3, 1])
        second = solve(permutation)
        np.testing.assert_array_equal(first.remaining_deficit_count, [0, 0, 0])
        np.testing.assert_array_equal(second.remaining_deficit_count, [0, 0, 0])
        np.testing.assert_array_equal(first.after.observed_count, [1, 1, 1])
        np.testing.assert_array_equal(second.after.observed_count, [1, 1, 1])
        first_points = candidates[first.selected_candidate_indices]
        second_points = candidates[permutation[second.selected_candidate_indices]]
        np.testing.assert_allclose(
            first_points[np.lexsort((first_points[:, 1], first_points[:, 0]))],
            second_points[np.lexsort((second_points[:, 1], second_points[:, 0]))],
        )


class ExactRowDownsampleTests(unittest.TestCase):
    def test_exact_subset_spacing_and_order_are_source_and_chunk_invariant(self):
        rng = np.random.default_rng(701)
        xyz = rng.uniform(-0.5, 0.5, size=(450, 3))
        # Include same-position rows with different scalar bytes.  Stable row
        # tie-breaking must select the same one under source permutation.
        xyz = np.concatenate((xyz, np.repeat(xyz[:2], 3, axis=0)), axis=0)
        records = records_for_points(xyz)
        first = V12.exact_row_spatial_downsample(
            records, spacing_m=0.09, seed=117
        )

        permutation = rng.permutation(len(records))
        second = V12.exact_row_spatial_downsample(
            records[permutation], spacing_m=0.09, seed=117
        )
        self.assertEqual(
            first.selected_records.tobytes(), second.selected_records.tobytes()
        )

        chunks = np.array_split(records, 7)
        chunked = V12.exact_row_spatial_downsample_chunks(
            reversed(chunks), spacing_m=0.09, seed=117
        )
        self.assertEqual(
            first.selected_records.tobytes(), chunked.selected_records.tobytes()
        )

        selected_xyz = np.column_stack(
            [first.selected_records[name] for name in ("x", "y", "z")]
        ).astype(np.float64)
        difference = selected_xyz[:, None, :] - selected_xyz[None, :, :]
        distance = np.linalg.norm(difference, axis=2)
        np.fill_diagonal(distance, np.inf)
        self.assertGreaterEqual(np.min(distance), 0.09 - 2.0e-7)

        original_rows = {
            row.tobytes() for row in np.asarray(records).reshape(-1)
        }
        self.assertTrue(
            all(row.tobytes() in original_rows for row in first.selected_records)
        )


if __name__ == "__main__":
    unittest.main()
