import importlib.util
from pathlib import Path
import sys
import unittest

import numpy as np


SCRIPT = Path(__file__).resolve().parents[1] / "scripts/rebuild_site1_fossils_v7.py"
sys.path.insert(0, str(SCRIPT.parent))
SPEC = importlib.util.spec_from_file_location("rebuild_site1_fossils_v7", SCRIPT)
SITE1 = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = SITE1
SPEC.loader.exec_module(SITE1)


class HeaderTests(unittest.TestCase):
    def test_count_patch_preserves_reserved_header_width(self):
        header = b"ply\nformat binary_little_endian 1.0\nelement vertex 12      \nend_header\n"
        patched = SITE1.patch_header_count(header, 987654)
        self.assertEqual(len(patched), len(header))
        self.assertIn(b"element vertex 987654  \n", patched)


class CandidateTests(unittest.TestCase):
    def test_candidate_generation_uses_unoccupied_subcells(self):
        grid = SITE1.v6.GridSpec(0.0, 0.0, 0.05, 0.025, 0.025)
        quota = np.zeros(grid.shape, np.int16)
        quota[0, 0] = 4
        role = np.zeros(grid.shape, bool)
        role[0, 0] = True
        # Occupy the first three 5 mm cells in global row-major order.
        occupied = np.array([0, 1, 2], np.int64)
        batches = list(SITE1.iter_candidate_xy(
            quota, role, grid, 0.005, 13, occupied_keys=occupied))
        x, y, cell = batches[0]
        ix = np.floor(x / 0.005).astype(np.int64)
        iy = np.floor(y / 0.005).astype(np.int64)
        keys = iy * 10 + ix
        self.assertEqual(len(x), 4)
        self.assertTrue(np.all(cell == 0))
        self.assertFalse(np.isin(keys, occupied).any())
        self.assertEqual(len(np.unique(keys)), 4)


class PlaneTests(unittest.TestCase):
    def test_robust_plane_retains_slope_despite_high_outliers(self):
        rng = np.random.default_rng(4)
        x = rng.uniform(10.0, 13.0, 500)
        y = rng.uniform(20.0, 23.0, 500)
        z = 1.8 + 0.008 * x - 0.004 * y + rng.normal(0, 0.001, 500)
        z[:20] += 1.5
        plane, report = SITE1.robust_plane_fit(x, y, z)
        self.assertAlmostEqual(plane[0], 0.008, delta=0.001)
        self.assertAlmostEqual(plane[1], -0.004, delta=0.001)
        self.assertGreater(report["slope"], 0.005)
        self.assertLess(report["anchor_residual_mad_m"], 0.004)


class ConfigTests(unittest.TestCase):
    def test_registered_annotations_have_expected_areas(self):
        config, _ = SITE1.load_config(
            SCRIPT.parent / "config/site1_fossils_v7_regions.json")
        expected = {
            "cyan_mixed_review": 449.93,
            "pink_water_exclusion": 211.00,
            "red_water_connections": 115.83,
            "yellow_terrain_review": 88.24,
        }

        def polygon_area(polygon):
            points = np.asarray(polygon)
            return 0.5 * abs(
                np.dot(points[:, 0], np.roll(points[:, 1], 1)) -
                np.dot(points[:, 1], np.roll(points[:, 0], 1)))

        for name, polygons in config["annotations"].items():
            measured = sum(polygon_area(polygon) for polygon in polygons)
            self.assertAlmostEqual(measured, expected[name], delta=0.02)


if __name__ == "__main__":
    unittest.main()
