import importlib.util
from pathlib import Path
import sys
import unittest

import numpy as np


SCRIPT = Path(__file__).resolve().parents[1] / "scripts/rebuild_site1_fossils_v9.py"
sys.path.insert(0, str(SCRIPT.parent))
SPEC = importlib.util.spec_from_file_location("rebuild_site1_fossils_v9", SCRIPT)
V9 = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = V9
SPEC.loader.exec_module(V9)


class ExclusionTests(unittest.TestCase):
    def test_registered_boxes_are_inclusive_and_exact(self):
        x = np.array([756.6749, 756.6750, 759.5000, 760.3000,
                      759.5000, 760.2750, 760.2751])
        y = np.array([821.0, 820.9750, 824.5750, 822.0,
                      824.7000, 824.7750, 824.0])
        got = V9.points_in_exclusion(x, y)
        np.testing.assert_array_equal(
            got, [False, True, True, False, True, True, False])


class TerrainClassificationTests(unittest.TestCase):
    @staticmethod
    def evidence(shape, level=2.18, count=20, sand=20):
        count_grid = np.full(shape, count, np.int32)
        return {
            "terrain_count": count_grid,
            "terrain_sum": count_grid * level,
            "terrain_sum2": count_grid * level * level,
            "original_sand_count": np.full(shape, sand, np.int32),
        }

    def test_dense_original_terrain_on_high_side_of_step_is_carved(self):
        shape = (8, 8)
        water = np.full(shape, 2.12)
        water[:, 4:] = 2.20
        evidence = self.evidence(shape, level=2.10)
        terrain_level = np.full(shape, 2.10)
        terrain_level[:, 4:] = 2.18
        evidence["terrain_sum"] = evidence["terrain_count"] * terrain_level
        evidence["terrain_sum2"] = (evidence["terrain_count"] *
                                    terrain_level * terrain_level)
        result = V9.classify_terrain(
            water, np.ones(shape, bool), evidence, water)
        self.assertTrue(result["exposed"][:, 4:].all())
        self.assertFalse(result["film_compatible"].any())

    def test_dense_shallow_pool_without_step_is_not_globally_carved(self):
        shape = (8, 8)
        water = np.full(shape, 2.20)
        result = V9.classify_terrain(
            water, np.ones(shape, bool), self.evidence(shape), water)
        self.assertFalse(result["exposed"].any())

    def test_deep_reflected_terrain_neither_carves_nor_anchors(self):
        shape = (8, 8)
        result = V9.classify_terrain(
            np.full(shape, 2.20), np.ones(shape, bool),
            self.evidence(shape, level=1.95))
        self.assertFalse(result["exposed"].any())
        self.assertFalse(result["film_compatible"].any())

    def test_scanid9_only_cell_is_unknown_not_a_dam(self):
        shape = (8, 8)
        evidence = self.evidence(shape, count=0, sand=0)
        result = V9.classify_terrain(
            np.full(shape, 2.20), np.ones(shape, bool), evidence)
        self.assertFalse(result["dense_original"].any())
        self.assertFalse(result["exposed"].any())


class ConnectedSolveTests(unittest.TestCase):
    def setUp(self):
        self.grid = V9.v6.GridSpec(0.0, 0.0, 0.75, 0.50, 0.025)

    def test_unknown_strip_does_not_preserve_a_water_step(self):
        mask = np.ones(self.grid.shape, bool)
        water = np.ones(self.grid.shape, np.float32)
        water[:, self.grid.nx // 2:] = 1.08
        film = np.full(self.grid.shape, np.nan, np.float32)
        surface, report = V9.solve_connected_surface(mask, water, film,
                                                     self.grid)
        self.assertEqual(report["components"], 1)
        self.assertLess(report["linear_solver"]["relative_residual"], 5e-5)
        seam = np.abs(np.diff(surface, axis=1))
        self.assertLess(float(np.nanmax(seam)), 0.012)

    def test_continuous_measured_ridge_can_separate_levels(self):
        mask = np.ones(self.grid.shape, bool)
        split = self.grid.nx // 2
        mask[:, split] = False
        water = np.ones(self.grid.shape, np.float32)
        water[:, split + 1:] = 1.08
        water[:, split] = np.nan
        film = np.full(self.grid.shape, np.nan, np.float32)
        surface, report = V9.solve_connected_surface(mask, water, film,
                                                     self.grid)
        self.assertEqual(report["components"], 2)
        left = np.nanmedian(surface[:, :split])
        right = np.nanmedian(surface[:, split + 1:])
        self.assertGreater(float(right - left), 0.06)

    def test_long_connected_sheet_converges_with_preconditioner(self):
        grid = V9.v6.GridSpec(0.0, 0.0, 10.0, 0.25, 0.025)
        mask = np.ones(grid.shape, bool)
        x = np.linspace(1.0, 1.08, grid.nx, dtype=np.float32)
        water = np.broadcast_to(x, grid.shape).copy()
        water[:, grid.nx // 2:] += 0.06
        film = np.full(grid.shape, np.nan, np.float32)
        film[:, ::40] = np.linspace(1.0, 1.08, grid.nx)[::40]
        surface, report = V9.solve_connected_surface(mask, water, film, grid)
        self.assertTrue(np.isfinite(surface).all())
        self.assertLess(report["linear_solver"]["relative_residual"], 5e-5)
        self.assertLess(report["linear_solver"]["iterations"], 1800)

    def test_closure_only_fragment_is_pruned_before_solve(self):
        mask = np.zeros((8, 12), bool)
        mask[1:4, 1:4] = True
        mask[5:7, 9:11] = True
        water = np.full(mask.shape, np.nan, np.float32)
        water[2, 2] = 1.0
        film = np.full(mask.shape, np.nan, np.float32)
        kept, report = V9.prune_unanchored_components(mask, water, film)
        self.assertTrue(kept[1:4, 1:4].all())
        self.assertFalse(kept[5:7, 9:11].any())
        self.assertEqual(report, {"cells": 4, "components": 1})

    def test_open_gradient_projection_removes_unphysical_step(self):
        mask = np.ones((20, 30), bool)
        surface = np.ones(mask.shape, np.float32)
        surface[:, 15:] += 0.08
        projected, report = V9.project_open_surface_gradient(surface, mask)
        self.assertLessEqual(
            V9.max_open_neighbour_step(projected, mask),
            V9.MAX_OPEN_NEIGHBOUR_STEP + 1e-6)
        self.assertGreater(report["before_max_mm"], 70.0)
        self.assertLess(report["after_max_mm"], 13.1)

    def test_gradient_projection_respects_a_carved_ridge(self):
        mask = np.ones((20, 30), bool)
        mask[:, 15] = False
        surface = np.ones(mask.shape, np.float32)
        surface[:, 16:] += 0.08
        projected, _ = V9.project_open_surface_gradient(surface, mask)
        self.assertGreater(float(np.nanmedian(projected[:, 16:]) -
                                 np.nanmedian(projected[:, :15])), 0.07)


class DensityTests(unittest.TestCase):
    def test_shoreline_quota_has_no_edge_or_seam_suppression(self):
        wet = np.ones((7, 9), bool)
        terrain = np.full(wet.shape, 25, np.int16)
        quota = V9.shoreline_quota(wet, terrain)
        self.assertTrue(np.all(quota == V9.MIN_SHORE_POINTS_PER_CELL))
        terrain[3, 4] = 0
        quota = V9.shoreline_quota(wet, terrain)
        self.assertEqual(quota[0, 0], V9.MIN_SHORE_POINTS_PER_CELL)
        self.assertEqual(quota[3, 4], V9.TARGET_POINTS_PER_CELL)


class ScalarTests(unittest.TestCase):
    def test_donor_application_clips_all_declared_bounds(self):
        dtype = np.dtype([(name, "<f4") for name in
                          ("x", "y", *V9.FIELD_BOUNDS.keys())])
        record = np.zeros(3, dtype)
        record["x"] = [0.01, 0.03, 0.04]
        record["y"] = [0.01, 0.03, 0.04]
        grid = V9.v6.GridSpec(0, 0, 0.05, 0.05, 0.05)
        fields = {}
        for name, (lo, hi) in V9.FIELD_BOUNDS.items():
            fields[name] = np.array([[(lo + hi) * 0.5]], np.float32)
        V9.apply_donor_bundle(record, {"grid": grid, "fields": fields})
        for name, (lo, hi) in V9.FIELD_BOUNDS.items():
            self.assertTrue(np.isfinite(record[name]).all())
            self.assertTrue(np.all(record[name] >= lo))
            self.assertTrue(np.all(record[name] <= hi))


if __name__ == "__main__":
    unittest.main()
