import importlib.util
from pathlib import Path
import sys
import unittest

import numpy as np
from scipy import ndimage


SCRIPT = Path(__file__).resolve().parents[1] / "scripts/rebuild_site1_fossils_water.py"
SPEC = importlib.util.spec_from_file_location("rebuild_site1_fossils_water", SCRIPT)
SITE1 = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = SITE1
SPEC.loader.exec_module(SITE1)


class PoolLevelTests(unittest.TestCase):
    def test_independent_pools_use_their_own_low_rim(self):
        grid = SITE1.GridSpec(0.0, 0.0, 2.5, 2.5, 0.025)
        shape = grid.shape
        water = np.zeros(shape, np.uint16)
        water_z = np.full(shape, np.nan, np.float32)
        first = np.zeros(shape, bool)
        second = np.zeros(shape, bool)
        first[15:45, 12:42] = True
        second[55:88, 58:91] = True
        water[first | second] = 4
        water_z[first] = 2.22
        water_z[second] = 2.63
        rim_first = ndimage.binary_dilation(first, iterations=4) & ~first
        rim_second = ndimage.binary_dilation(second, iterations=4) & ~second
        sand = np.zeros(shape, np.uint16)
        sand_z = np.full(shape, np.nan, np.float32)
        sand[rim_first | rim_second] = 12
        sand_z[rim_first] = 2.03
        sand_z[rim_second] = 2.41
        maps = SITE1.SupportMaps(
            grid, sand, np.zeros(shape, np.uint16), water, sand_z,
            np.full(shape, np.nan, np.float32), water_z)

        occ, surface, _, report = SITE1.build_pool_levels(
            maps, np.zeros(shape, bool), np.zeros(shape, bool))

        self.assertTrue(np.all(occ == (first | second)))
        self.assertAlmostEqual(float(np.median(surface[first])), 2.03, delta=0.015)
        self.assertAlmostEqual(float(np.median(surface[second])), 2.41, delta=0.015)
        self.assertGreaterEqual(report["pool_cores"], 2)


class RippleTests(unittest.TestCase):
    def test_multi_octave_ripple_is_deterministic_and_calibrated(self):
        grid = SITE1.GridSpec(0.0, 0.0, 2.0, 2.0, 0.025)
        occ = np.zeros(grid.shape, bool)
        occ[5:-5, 5:-5] = True
        edge = ndimage.distance_transform_edt(occ).astype(np.float32) * grid.cell

        first, report = SITE1.build_ripple_grid(grid, occ, edge)
        second, _ = SITE1.build_ripple_grid(grid, occ, edge)

        np.testing.assert_array_equal(first, second)
        self.assertAlmostEqual(report["rms_m"], SITE1.RIPPLE_RMS_M, delta=2e-6)
        self.assertGreater(float(np.std(first[occ])), 0.001)
        self.assertLess(report["max_abs_m"], 0.006)

    def test_masked_gradient_does_not_look_across_a_basin_gap(self):
        values = np.zeros((9, 13), np.float32)
        valid = np.zeros_like(values, bool)
        valid[2:7, 1:5] = True
        valid[2:7, 8:12] = True
        values[valid & (np.indices(valid.shape)[1] < 6)] = 2.0
        values[valid & (np.indices(valid.shape)[1] > 6)] = 2.5

        gy, gx = SITE1.masked_gradient(values, valid, 0.025)

        self.assertEqual(float(np.max(np.abs(gx[valid]))), 0.0)
        self.assertEqual(float(np.max(np.abs(gy[valid]))), 0.0)

    def test_float16_static_grid_uses_bilinear_sampling(self):
        grid = SITE1.GridSpec(0.0, 0.0, 2.0, 2.0, 1.0)
        values = np.array([[0.0, 2.0], [4.0, 6.0]], np.float16)

        sampled = SITE1.sample_grid(
            values, np.array([0.5, 1.0, 1.5]),
            np.array([0.5, 1.0, 1.5]), grid)

        np.testing.assert_allclose(sampled, [0.0, 3.0, 6.0], atol=1e-6)


class TerrainCandidateTests(unittest.TestCase):
    def test_candidates_are_feathered_inside_the_annotation(self):
        grid = SITE1.GridSpec(0.0, 0.0, 1.0, 1.0, 0.025)
        mask = np.zeros(grid.shape, bool)
        mask[5:35, 6:34] = True
        count = np.zeros(grid.shape, np.int32)

        x, y, quota = SITE1.terrain_candidate_xy(mask, count, 20, grid, 7)
        gx, gy = grid.indices(x, y)

        self.assertGreater(len(x), 5_000)
        self.assertTrue(np.all(mask[gy, gx]))
        self.assertLess(int(quota[5, 6]), int(quota[20, 20]))
        self.assertEqual(int(quota[20, 20]), 20)
        self.assertLessEqual(int(quota.max()), 25)

    def test_static_idw_is_normalised_local_and_range_preserving(self):
        distance = np.array([[0.001, 0.010, 0.020],
                             [0.030, 0.020, 0.010]], np.float32)
        values = np.array([[-2.0, 4.0, 9.0],
                           [10.0, 12.0, 20.0]], np.float32)

        weights = SITE1.inverse_distance_squared_weights(distance)
        interpolated = np.sum(values * weights, axis=1)

        np.testing.assert_allclose(weights.sum(axis=1), 1.0, atol=1e-6)
        self.assertGreater(float(weights[0, 0]), float(weights[0, 1]))
        self.assertTrue(np.all(interpolated >= values.min(axis=1)))
        self.assertTrue(np.all(interpolated <= values.max(axis=1)))


class AnnotationConfigTests(unittest.TestCase):
    def test_registered_regions_have_expected_world_areas(self):
        config, _ = SITE1.load_config(
            SCRIPT.parent / "config/site1_fossils_reconstruction_regions.json")
        expected = {
            "red-sand-holes": 29.89,
            "blue-rock-density": 64.44,
            "yellow-rock-edge": 19.47,
        }

        def area(polygon):
            p = np.asarray(polygon)
            return 0.5 * abs(np.dot(p[:, 0], np.roll(p[:, 1], 1)) -
                             np.dot(p[:, 1], np.roll(p[:, 0], 1)))

        for region in config["terrain_regions"]:
            measured = sum(area(polygon) for polygon in region["polygons"])
            self.assertAlmostEqual(measured, expected[region["id"]], delta=0.02)


if __name__ == "__main__":
    unittest.main()
