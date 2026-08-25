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


class ClassifyVerdictTests(unittest.TestCase):
    """The v4 cluster-verdict table: one row per calibrated behaviour."""

    def metrics(self, **overrides):
        base = {
            "n": np.asarray([1000]),
            "dens_med": np.asarray([20000.0], np.float32),
            "i_med": np.asarray([400000.0], np.float32),
            "i_p90": np.asarray([500000.0], np.float32),
            "share": np.asarray([0.4], np.float32),
            "f_under": np.asarray([0.9], np.float32),
            "f_gap8": np.asarray([0.05], np.float32),
            "man_f": np.asarray([0.0], np.float32),
            "flat_f": np.asarray([1.0], np.float32),
            "h_med": np.asarray([0.2], np.float32),
            "contact": np.asarray([500]),
            "seedhit": np.zeros(1, np.int8),
        }
        for key, value in overrides.items():
            base[key] = np.asarray([value],
                                   base[key].dtype if key in base else None)
        return base

    def verdict(self, **overrides):
        return int(SITE1.classify_verdicts(self.metrics(**overrides))[0])

    def test_dense_grounded_cluster_keeps(self):
        self.assertEqual(self.verdict(), 0)

    def test_big_structure_keeps_even_when_bright(self):
        self.assertEqual(self.verdict(n=30000, i_med=700000.0, share=1.0), 0)

    def test_sparse_cluster_marked(self):
        self.assertEqual(self.verdict(dens_med=3000.0), 2)

    def test_fully_detached_cluster_floats(self):
        self.assertEqual(self.verdict(contact=0), 3)

    def test_bright_single_scan_object_removed(self):
        self.assertEqual(self.verdict(i_med=650000.0, share=1.0), 4)

    def test_bright_multi_scan_surface_keeps(self):
        self.assertEqual(self.verdict(i_med=650000.0, share=0.4), 0)

    def test_dark_object_with_air_gap_removed(self):
        self.assertEqual(self.verdict(i_med=100000.0, f_gap8=0.5), 4)

    def test_self_shadow_needs_height_and_flat_ground(self):
        self.assertEqual(
            self.verdict(i_med=650000.0, f_under=0.2, h_med=0.05), 0)
        self.assertEqual(
            self.verdict(i_med=650000.0, f_under=0.2, h_med=0.2, flat_f=0.0), 0)
        self.assertEqual(
            self.verdict(i_med=650000.0, f_under=0.2, h_med=0.2), 4)

    def test_manual_majority_removed_only_on_the_flat(self):
        self.assertEqual(self.verdict(man_f=0.9), 4)
        self.assertEqual(self.verdict(man_f=0.9, flat_f=0.2), 0)

    def test_seed_marked_cluster_removed(self):
        self.assertEqual(self.verdict(seedhit=1), 5)
