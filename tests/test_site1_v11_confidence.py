import importlib.util
from pathlib import Path
import sys
import unittest

import numpy as np


SCRIPT = Path(__file__).resolve().parents[1] / "scripts/site1_v11_confidence.py"
SPEC = importlib.util.spec_from_file_location("site1_v11_confidence", SCRIPT)
V11 = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = V11
SPEC.loader.exec_module(V11)


class DensityInterpolationTests(unittest.TestCase):
    def test_exact_support_is_preserved_and_between_samples_is_smooth(self):
        support = np.array([[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]])
        density = np.array([10.0, 20.0, 30.0])
        exact = V11.interpolate_target_density(
            support,
            support,
            density,
            neighbours=3,
            minimum_bandwidth=2.1,
        )
        np.testing.assert_allclose(exact.target_density, density)

        query = np.array([[0.50, 0.0], [1.0 - 1e-5, 0.0], [1.0 + 1e-5, 0.0]])
        result = V11.interpolate_target_density(
            query,
            support,
            density,
            neighbours=3,
            minimum_bandwidth=2.1,
        )
        self.assertGreater(result.target_density[0], 10.0)
        self.assertLess(result.target_density[0], 20.0)
        self.assertLess(abs(result.target_density[2] - result.target_density[1]), 0.001)

    def test_outward_taper_is_bounded_monotone_and_c1_at_endpoints(self):
        outward = np.array([-1.0, 0.0, 1e-6, 0.5, 1.0 - 1e-6, 1.0, 2.0])
        query = np.column_stack((outward, np.zeros_like(outward)))
        support = np.array([[-2.0, 0.0], [3.0, 0.0]])
        result = V11.interpolate_target_density(
            query,
            support,
            [100.0, 100.0],
            neighbours=2,
            minimum_bandwidth=6.0,
            outward_distance=outward,
            taper=V11.OutwardTaper(start=0.0, end=1.0, floor_ratio=0.2),
        )
        self.assertTrue(np.all(np.diff(result.taper_factor) <= 0.0))
        self.assertGreaterEqual(float(result.taper_factor.min()), 0.2)
        self.assertLessEqual(float(result.taper_factor.max()), 1.0)
        self.assertAlmostEqual(result.target_density[3], 60.0, places=10)
        self.assertAlmostEqual(result.target_density[0], 100.0, places=10)
        self.assertAlmostEqual(result.target_density[-1], 20.0, places=10)
        # Cubic smoothstep has zero endpoint slope; a 1 um move changes the
        # multiplier by much less than a linear taper would.
        self.assertLess(1.0 - result.taper_factor[2], 1e-10)
        self.assertLess(result.taper_factor[4] - 0.2, 1e-10)

    def test_invalid_taper_pair_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "provided together"):
            V11.interpolate_target_density(
                [[0.0, 0.0]],
                [[0.0, 0.0]],
                [10.0],
                outward_distance=[0.0],
            )


class GeometryConfidenceTests(unittest.TestCase):
    @staticmethod
    def evaluate(
        sectors,
        spread,
        coherence,
        thickness,
        multimodality,
        energy,
    ):
        spread = np.asarray(spread, dtype=float)
        heights = np.column_stack((np.zeros(len(spread)), spread))
        return V11.evaluate_geometry_confidence(
            sectors,
            heights,
            coherence,
            thickness,
            multimodality,
            energy,
        )

    def test_tiers_are_assigned_only_after_all_hard_gates_pass(self):
        result = self.evaluate(
            sectors=[8, 7, 6],
            spread=[0.001, 0.001, 0.0025],
            coherence=[0.95, 0.95, 0.85],
            thickness=[0.004, 0.005, 0.010],
            multimodality=[0.10, 0.20, 0.30],
            energy=[1.0, 1.0, 1.0],
        )
        np.testing.assert_array_equal(
            result.tier,
            [
                V11.ConfidenceTier.STRONG,
                V11.ConfidenceTier.SUPPORTED,
                V11.ConfidenceTier.CONSERVATIVE,
            ],
        )
        self.assertTrue(result.accepted.all())

    def test_each_hard_veto_has_an_independent_reason_bit(self):
        result = self.evaluate(
            sectors=[5, 8, 8, 8, 8, 8],
            spread=[0.001, 0.004, 0.001, 0.001, 0.001, 0.001],
            coherence=[0.95, 0.95, 0.79, 0.95, 0.95, 0.95],
            thickness=[0.004, 0.004, 0.004, 0.013, 0.004, 0.004],
            multimodality=[0.10, 0.10, 0.10, 0.10, 0.36, 0.10],
            energy=[1.0, 1.0, 1.0, 1.0, 1.0, 2.01],
        )
        expected = [
            V11.ConfidenceReason.INSUFFICIENT_DONOR_SECTORS,
            V11.ConfidenceReason.SURFACE_HEIGHT_DISAGREEMENT,
            V11.ConfidenceReason.LOW_NORMAL_COHERENCE,
            V11.ConfidenceReason.EXCESSIVE_VERTICAL_THICKNESS,
            V11.ConfidenceReason.VERTICAL_MULTIMODALITY,
            V11.ConfidenceReason.EXCESSIVE_RESIDUAL_ENERGY,
        ]
        np.testing.assert_array_equal(result.reason_mask, [int(value) for value in expected])
        self.assertTrue(np.all(result.tier == V11.ConfidenceTier.REJECTED))

    def test_multiple_vetoes_accumulate_and_decode(self):
        result = self.evaluate(
            sectors=[4],
            spread=[0.005],
            coherence=[0.5],
            thickness=[0.020],
            multimodality=[0.8],
            energy=[4.0],
        )
        names = set(V11.confidence_reason_names(result.reason_mask[0]))
        self.assertEqual(
            names,
            {
                "INSUFFICIENT_DONOR_SECTORS",
                "SURFACE_HEIGHT_DISAGREEMENT",
                "LOW_NORMAL_COHERENCE",
                "EXCESSIVE_VERTICAL_THICKNESS",
                "VERTICAL_MULTIMODALITY",
                "EXCESSIVE_RESIDUAL_ENERGY",
            },
        )

    def test_residual_energy_is_veto_only_and_cannot_upgrade_tier(self):
        result = self.evaluate(
            sectors=[7, 7],
            spread=[0.001, 0.001],
            coherence=[0.95, 0.95],
            thickness=[0.005, 0.005],
            multimodality=[0.20, 0.20],
            energy=[0.0, 1.99],
        )
        np.testing.assert_array_equal(
            result.tier,
            [V11.ConfidenceTier.SUPPORTED, V11.ConfidenceTier.SUPPORTED],
        )
        np.testing.assert_array_equal(result.preferred_gate_count, [4, 4])

    def test_nonfinite_surface_prediction_is_invalid(self):
        result = V11.evaluate_geometry_confidence(
            [8],
            [[1.0, np.nan, 1.001]],
            [0.95],
            [0.004],
            [0.10],
            [1.0],
        )
        self.assertEqual(
            result.reason_mask[0], int(V11.ConfidenceReason.INVALID_METRIC)
        )
        self.assertTrue(np.isnan(result.surface_spread_m[0]))


class VariableRadiusBlueNoiseTests(unittest.TestCase):
    def test_existing_and_selected_clearance_are_reported_separately(self):
        candidates = np.array(
            [[0.2, 0.0], [1.0, 0.0], [1.4, 0.0], [2.0, 0.0]]
        )
        result = V11.variable_radius_blue_noise(
            candidates,
            [0.30, 0.60, 0.20, 0.20],
            existing_points=[[0.0, 0.0]],
            priority=[20.0, 10.0, 5.0, 0.0],
            rebuild_interval=2,
        )
        np.testing.assert_array_equal(result.selected_indices, [1, 3])
        self.assertEqual(
            result.reason_mask[0], int(V11.BlueNoiseRejection.EXISTING_CLEARANCE)
        )
        self.assertEqual(
            result.reason_mask[2], int(V11.BlueNoiseRejection.SELECTED_CLEARANCE)
        )

    def test_pairwise_distance_uses_larger_variable_radius(self):
        result = V11.variable_radius_blue_noise(
            [[0.0, 0.0], [0.15, 0.0]],
            [0.10, 0.20],
            priority=[1.0, 0.0],
        )
        np.testing.assert_array_equal(result.selected_indices, [0])
        self.assertEqual(
            result.reason_mask[1], int(V11.BlueNoiseRejection.SELECTED_CLEARANCE)
        )

    def test_equal_priority_result_is_independent_of_input_permutation(self):
        rng = np.random.default_rng(71)
        candidates = rng.uniform(-1.0, 1.0, size=(160, 2))
        radii = 0.08 + 0.05 * (candidates[:, 0] + 1.0) / 2.0
        first = V11.variable_radius_blue_noise(
            candidates,
            radii,
            seed=9871,
            rebuild_interval=7,
        )
        permutation = rng.permutation(len(candidates))
        second = V11.variable_radius_blue_noise(
            candidates[permutation],
            radii[permutation],
            seed=9871,
            rebuild_interval=31,
        )
        first_points = {tuple(row) for row in candidates[first.selected_indices]}
        second_points = {
            tuple(row) for row in candidates[permutation][second.selected_indices]
        }
        self.assertEqual(first_points, second_points)

    def test_rebuild_interval_does_not_change_selection(self):
        rng = np.random.default_rng(4)
        candidates = rng.uniform(0.0, 2.0, size=(300, 3))
        radii = rng.uniform(0.05, 0.12, size=len(candidates))
        priority = rng.normal(size=len(candidates))
        frequent = V11.variable_radius_blue_noise(
            candidates,
            radii,
            priority=priority,
            seed=33,
            rebuild_interval=1,
        )
        batched = V11.variable_radius_blue_noise(
            candidates,
            radii,
            priority=priority,
            seed=33,
            rebuild_interval=256,
        )
        np.testing.assert_array_equal(frequent.selected_indices, batched.selected_indices)
        np.testing.assert_array_equal(frequent.reason_mask, batched.reason_mask)

    def test_selected_points_obey_pointwise_spacing_without_grid_quotas(self):
        x, y = np.meshgrid(np.arange(20) * 0.04, np.arange(12) * 0.04)
        candidates = np.column_stack((x.ravel() + 0.013, y.ravel() + 0.017))
        # Continuous left-to-right density falloff represented by increasing
        # exclusion radii; no planning-cell label is supplied to the selector.
        radii = 0.045 + 0.035 * candidates[:, 0] / candidates[:, 0].max()
        result = V11.variable_radius_blue_noise(candidates, radii, seed=1001)
        selected = result.selected_indices
        for offset, first in enumerate(selected):
            later = selected[offset + 1 :]
            if not len(later):
                continue
            distance = np.linalg.norm(candidates[later] - candidates[first], axis=1)
            required = np.maximum(radii[later], radii[first])
            self.assertTrue(np.all(distance + 1e-12 >= required))
        self.assertGreater(len(selected), 10)
        self.assertLess(len(selected), len(candidates))

    def test_invalid_candidate_is_retained_in_reason_map(self):
        result = V11.variable_radius_blue_noise(
            [[0.0, 0.0], [np.nan, 1.0], [2.0, 0.0]],
            [0.2, 0.2, -1.0],
        )
        np.testing.assert_array_equal(result.selected_indices, [0])
        self.assertEqual(
            result.reason_mask[1], int(V11.BlueNoiseRejection.INVALID_CANDIDATE)
        )
        self.assertEqual(
            result.reason_mask[2], int(V11.BlueNoiseRejection.INVALID_CANDIDATE)
        )


if __name__ == "__main__":
    unittest.main()
