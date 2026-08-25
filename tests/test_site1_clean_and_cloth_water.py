import importlib.util
from pathlib import Path
import unittest

import numpy as np


SCRIPT = Path(__file__).resolve().parents[1] / "scripts/site1_clean_and_cloth_water.py"
SPEC = importlib.util.spec_from_file_location("site1_clean_and_cloth_water", SCRIPT)
SITE1 = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SITE1)


class WaterXYRecoveryTests(unittest.TestCase):
    CELL = SITE1.WATER_SUPPORT_CELL

    def grids(self, size=81, z=2.2):
        shape = (size, size)
        in_hull = np.zeros(shape, bool)
        in_hull[5:-5, 5:-5] = True
        sand_count = np.zeros(shape, np.uint32)
        sand_count[in_hull] = 1
        sand_zsum = sand_count.astype(np.float64) * z
        rock_count = np.zeros(shape, np.uint32)
        rock_zsum = np.zeros(shape, np.float64)
        base_allowed = in_hull.copy()
        return [sand_count, sand_zsum, rock_count, rock_zsum,
                in_hull, base_allowed]

    def recover(self, grids):
        return SITE1.build_water_xy_recovery(*grids, cell=self.CELL)

    def test_recovers_enclosed_flat_sand_gap(self):
        grids = self.grids()
        sand_count, sand_zsum, _, _, _, base_allowed = grids
        hole = np.s_[32:45, 32:45]
        sand_count[hole] = 0
        sand_zsum[hole] = 0
        base_allowed[hole] = False

        confidence, sheet, stats = self.recover(grids)

        self.assertGreater(confidence[38, 38], 0.5)
        self.assertAlmostEqual(float(sheet[38, 38]), 2.2, places=3)
        self.assertGreater(stats["recovered_cells"], 0)

    def test_does_not_bridge_large_open_sand_gap(self):
        grids = self.grids(size=101)
        sand_count, sand_zsum, _, _, _, base_allowed = grids
        hole = np.s_[28:73, 28:73]  # 1.125 m: wider than the 0.4 m close radius
        sand_count[hole] = 0
        sand_zsum[hole] = 0
        base_allowed[hole] = False

        confidence, _, _ = self.recover(grids)

        self.assertEqual(float(confidence[50, 50]), 0.0)

    def test_high_rock_neighbourhood_vetoes_recovery(self):
        grids = self.grids()
        sand_count, sand_zsum, rock_count, rock_zsum, _, base_allowed = grids
        hole = np.s_[30:47, 30:47]
        sand_count[hole] = 0
        sand_zsum[hole] = 0
        base_allowed[hole] = False
        rock_count[hole] = 3
        rock_zsum[hole] = 9.3

        confidence, _, stats = self.recover(grids)

        self.assertEqual(float(confidence[38, 38]), 0.0)
        self.assertGreater(stats["rock_veto_cells"], 0)

    def test_inconsistent_sand_heights_veto_recovery(self):
        grids = self.grids()
        sand_count, sand_zsum, _, _, _, base_allowed = grids
        hole = np.s_[28:49, 34:43]
        sand_count[hole] = 0
        sand_zsum[hole] = 0
        base_allowed[hole] = False
        sand_zsum[:, :34] = sand_count[:, :34].astype(np.float64) * 2.05
        sand_zsum[:, 43:] = sand_count[:, 43:].astype(np.float64) * 2.45

        confidence, _, stats = self.recover(grids)

        self.assertEqual(float(confidence[38, 38]), 0.0)
        self.assertGreater(stats["z_veto_cells"], 0)

    def test_recovery_stays_clear_of_hull_edge(self):
        grids = self.grids()
        sand_count, sand_zsum, _, _, in_hull, base_allowed = grids
        strip = np.s_[5:8, 20:45]
        sand_count[strip] = 0
        sand_zsum[strip] = 0
        base_allowed[strip] = False

        confidence, _, _ = self.recover(grids)

        self.assertFalse(np.any(confidence[~in_hull]))
        self.assertEqual(float(confidence[6, 30]), 0.0)


if __name__ == "__main__":
    unittest.main()
