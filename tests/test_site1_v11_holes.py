import sys
import unittest
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import site1_v11_holes as holes


class Site1V11HoleTests(unittest.TestCase):
    def ring_support(self):
        angle = np.linspace(0.0, 2.0 * np.pi, 720, endpoint=False)
        water = np.column_stack((0.5 + 0.30 * np.cos(angle), 0.5 + 0.30 * np.sin(angle)))
        return water

    def policy(self):
        return holes.SeededHolePolicy(
            diagnostic_cell_m=0.02,
            water_support_radius_m=0.025,
            terrain_support_radius_m=0.025,
            seed_search_radius_m=0.10,
            minimum_area_m2=0.01,
            maximum_area_m2=0.40,
            sector_radius_m=0.40,
        )

    def test_enclosed_seed_selects_measured_component(self):
        water = self.ring_support()
        plan = holes.detect_seeded_holes(
            water,
            np.empty((0, 2)),
            review_bbox=(0.0, 1.0, 0.0, 1.0),
            seeds={"centre": (0.5, 0.5)},
            policy=self.policy(),
        )
        self.assertTrue(plan.holes[0].accepted)
        self.assertGreater(plan.holes[0].area_m2, 0.10)
        self.assertEqual(plan.holes[0].support_sectors, 8)

    def test_open_exterior_component_is_rejected(self):
        water = self.ring_support()
        plan = holes.detect_seeded_holes(
            water,
            np.empty((0, 2)),
            review_bbox=(0.0, 1.0, 0.0, 1.0),
            seeds={"outside": (0.05, 0.05)},
            policy=self.policy(),
        )
        self.assertFalse(plan.holes[0].accepted)
        self.assertTrue(
            plan.holes[0].reason_mask
            & int(holes.HoleReason.COMPONENT_TOUCHES_REVIEW_BOUNDARY)
        )

    def test_annotation_bbox_is_not_blanket_filled(self):
        water = self.ring_support()
        plan = holes.detect_seeded_holes(
            water,
            np.empty((0, 2)),
            review_bbox=(0.0, 1.0, 0.0, 1.0),
            seeds={"centre": (0.5, 0.5)},
            policy=self.policy(),
        )
        samples = holes.sample_accepted_holes(
            plan,
            water,
            np.empty((0, 2)),
            spacing_m=0.04,
            terrain_clearance_m=0.01,
            seed=123,
        )
        radius = np.sqrt(np.sum(np.square(samples.xy - 0.5), axis=1))
        self.assertGreater(len(samples.xy), 50)
        self.assertTrue(np.all(radius < 0.28))

    def test_sampling_is_deterministic_and_blue_noise_spaced(self):
        water = self.ring_support()
        plan = holes.detect_seeded_holes(
            water,
            np.empty((0, 2)),
            review_bbox=(0.0, 1.0, 0.0, 1.0),
            seeds={"centre": (0.5, 0.5)},
            policy=self.policy(),
        )
        first = holes.sample_accepted_holes(
            plan, water, np.empty((0, 2)), spacing_m=0.04,
            terrain_clearance_m=0.0, seed=99,
        )
        second = holes.sample_accepted_holes(
            plan, water, np.empty((0, 2)), spacing_m=0.04,
            terrain_clearance_m=0.0, seed=99,
        )
        np.testing.assert_array_equal(first.xy, second.xy)
        distance = np.sqrt(np.sum(np.square(first.xy[:, None] - first.xy[None, :]), axis=2))
        distance[distance == 0.0] = np.inf
        self.assertGreaterEqual(float(np.min(distance)), 0.04 - 1.0e-12)

    def test_terrain_clearance_is_honoured(self):
        water = self.ring_support()
        terrain = np.array([[0.5, 0.5]])
        plan = holes.detect_seeded_holes(
            water,
            terrain,
            review_bbox=(0.0, 1.0, 0.0, 1.0),
            seeds={"centre": (0.55, 0.5)},
            policy=self.policy(),
        )
        samples = holes.sample_accepted_holes(
            plan, water, terrain, spacing_m=0.04,
            terrain_clearance_m=0.10, seed=5,
        )
        distance = np.sqrt(np.sum(np.square(samples.xy - terrain[0]), axis=1))
        self.assertTrue(np.all(distance >= 0.10))


if __name__ == "__main__":
    unittest.main()
