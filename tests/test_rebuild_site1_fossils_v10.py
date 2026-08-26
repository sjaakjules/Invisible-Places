import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock

import numpy as np


SCRIPT = Path(__file__).resolve().parents[1] / "scripts/rebuild_site1_fossils_v10.py"
sys.path.insert(0, str(SCRIPT.parent))
SPEC = importlib.util.spec_from_file_location("rebuild_site1_fossils_v10", SCRIPT)
V10 = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = V10
SPEC.loader.exec_module(V10)


class MicroTextureTests(unittest.TestCase):
    def test_calibrated_spectrum_constants_are_pinned(self):
        np.testing.assert_array_equal(
            V10.NOISE_OCTAVE_WAVELENGTHS_M,
            [0.018, 0.030, 0.050, 0.084, 0.142, 0.240, 0.406],
        )
        np.testing.assert_array_equal(
            V10.NOISE_OCTAVE_RMS_M,
            [0.00040, 0.00065, 0.00090, 0.00100,
             0.00090, 0.00065, 0.00035],
        )
        np.testing.assert_array_equal(
            V10.NOISE_OCTAVE_SEEDS,
            [71, 137, 211, 293, 379, 463, 557],
        )
        self.assertEqual(V10.NOISE_MICROGRAIN_COMPONENTS, 18)
        self.assertEqual(V10.NOISE_MICROGRAIN_SEED, 98473)
        self.assertAlmostEqual(V10.NOISE_MICROGRAIN_RMS_M, 0.00060)
        self.assertAlmostEqual(
            V10.NOISE_UNCLIPPED_RMS_M,
            0.002026696819951124,
            places=12,
        )
        self.assertAlmostEqual(V10.SHORE_NOISE_FLOOR, 0.45)
        self.assertAlmostEqual(V10.SHORE_NOISE_RAMP_M, 0.0625)

    def test_height_gradient_matches_central_finite_difference(self):
        rng = np.random.default_rng(8)
        x = rng.uniform(760.0, 780.0, 4000)
        y = rng.uniform(800.0, 840.0, 4000)
        height, dx, dy = V10.micro_texture(x, y)
        epsilon = 2e-5
        hx0 = V10.micro_texture(x - epsilon, y)[0]
        hx1 = V10.micro_texture(x + epsilon, y)[0]
        hy0 = V10.micro_texture(x, y - epsilon)[0]
        hy1 = V10.micro_texture(x, y + epsilon)[0]
        np.testing.assert_allclose(
            (hx1 - hx0) / (2.0 * epsilon), dx,
            rtol=0.01, atol=0.002,
        )
        np.testing.assert_allclose(
            (hy1 - hy0) / (2.0 * epsilon), dy,
            rtol=0.01, atol=0.002,
        )
        self.assertTrue(np.all(np.isfinite(height)))

    def test_dense_domain_displacement_and_five_mm_steps_are_bounded(self):
        spacing = 0.005
        axis_x = 768.64 + (np.arange(300) + 0.5) * spacing
        axis_y = 807.36 + (np.arange(300) + 0.5) * spacing
        x, y = np.meshgrid(axis_x, axis_y)
        height = V10.micro_texture(x, y)[0]
        rms = float(np.sqrt(np.mean(height.astype(np.float64) ** 2)))
        self.assertGreater(rms, 0.00195)
        self.assertLess(rms, 0.00210)
        self.assertLess(float(np.quantile(np.abs(height), 0.99)), 0.0055)
        self.assertLessEqual(
            float(np.max(np.abs(height))), V10.NOISE_SOFT_CLIP_LIMIT_M
        )
        steps = np.concatenate((
            (V10.micro_texture(x + spacing, y)[0] - height).ravel(),
            (V10.micro_texture(x, y + spacing)[0] - height).ravel(),
        ))
        self.assertLess(float(np.quantile(np.abs(steps), 0.99)), 0.0022)

    def test_surface_uses_shore_floor_then_reaches_full_amplitude(self):
        grid = V10.v6.GridSpec(0.0, 0.0, 0.1, 0.1, 0.025)
        shape = grid.shape

        def surface_at(signed_cells):
            zeros = np.zeros(shape, np.float32)
            return V10.SurfaceReference(
                grid=grid,
                wet=np.ones(shape, bool),
                signed_cells=np.full(shape, signed_cells, np.float32),
                z=zeros,
                dzdx=zeros,
                dzdy=zeros,
                signed_dx=zeros,
                signed_dy=zeros,
                noise_mean=zeros,
                review_additions=np.zeros(shape, bool),
                exclusion=np.zeros(shape, bool),
            )

        x = np.array([0.051], np.float64)
        y = np.array([0.047], np.float64)
        raw = V10.micro_texture(x, y)[0]
        shore = V10.surface_values(surface_at(0.0), x, y)[0]
        interior_cells = V10.SHORE_NOISE_RAMP_M / grid.cell
        interior = V10.surface_values(
            surface_at(interior_cells), x, y
        )[0]
        np.testing.assert_allclose(
            shore, V10.SHORE_NOISE_FLOOR * raw, rtol=1e-6, atol=1e-9
        )
        np.testing.assert_allclose(interior, raw, rtol=1e-6, atol=1e-9)


class AnalysisCommandTests(unittest.TestCase):
    def test_cleanmesh_uses_three_mm_fine_proxy(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            executable = root / "cleanmesh_reduced_analysis"
            executable.touch()
            with mock.patch.object(V10.subprocess, "run") as run:
                V10.run_reduced_analysis(
                    executable,
                    root / "source.ply",
                    root / "analysed.ply",
                    root / "report.json",
                    overwrite=False,
                    log=lambda _: None,
                )
            command = run.call_args.args[0]
            base_index = command.index("--base-voxel")
            self.assertEqual(command[base_index + 1], "0.003")


class HeaderTests(unittest.TestCase):
    def test_fixed_width_count_header_round_trips(self):
        dtype = np.dtype([("x", "<f4"), ("scalar_ScanID", "<f4")])
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "counted.ply"
            records = np.zeros(7, dtype)
            with open(path, "w+b") as handle:
                offset = V10.write_counted_header(handle, dtype, ["test"])
                records.tofile(handle)
                V10.patch_count(handle, offset, len(records))
            parsed, count, _, _ = V10.v6.read_ply_header(path)
            self.assertEqual(parsed, dtype)
            self.assertEqual(count, len(records))


class ResumeTests(unittest.TestCase):
    def test_reusable_stage_rejects_content_drift(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "report.json"
            path.write_text("{}\n")
            state = {
                "stages": {
                    "example": {
                        "outputs": [V10.file_fingerprint(path)],
                    }
                }
            }
            self.assertTrue(V10.reusable_stage(state, "example", [path]))
            path.write_text('{"changed": true}\n')
            self.assertFalse(V10.reusable_stage(state, "example", [path]))


class SurfaceReviewTests(unittest.TestCase):
    def test_surface_reference_replaces_archived_ripple_with_base_surface(self):
        with tempfile.TemporaryDirectory() as directory:
            run = Path(directory)
            shape = (4, 4)
            base_surface = np.full(shape, 2.0, np.float32)
            archived_surface = np.full(shape, 9.0, np.float32)
            np.savez_compressed(
                run / "surface-v9.npz",
                meta=np.array([0.0, 0.0, 0.1, 0.1, 0.025]),
                wet=np.ones(shape, bool),
                base_surface=base_surface,
                surface=archived_surface,
                exclusion=np.zeros(shape, bool),
                terrain_count=np.zeros(shape, np.int32),
            )
            surface = V10.load_surface_reference(run, {})
            np.testing.assert_array_equal(surface.z, base_surface)
            self.assertFalse(np.any(surface.z == archived_surface))

    def test_harmonic_extension_interpolates_between_fixed_edges(self):
        values = np.zeros((3, 5), np.float32)
        fixed = np.zeros((3, 5), bool)
        additions = np.zeros((3, 5), bool)
        fixed[1, 0] = True
        fixed[1, 4] = True
        values[1, 0] = 2.0
        values[1, 4] = 4.0
        additions[1, 1:4] = True
        got = V10._harmonic_review_extension(values, fixed, additions)
        np.testing.assert_allclose(got[1], [2.0, 2.5, 3.0, 3.5, 4.0])

    def test_review_addition_keeps_measured_island_excluded(self):
        with tempfile.TemporaryDirectory() as directory:
            run = Path(directory)
            shape = (20, 20)
            wet = np.zeros(shape, bool)
            wet[:, :4] = True
            terrain_count = np.zeros(shape, np.int32)
            terrain_count[10, 12] = 20
            np.savez_compressed(
                run / "surface-v9.npz",
                meta=np.array([0.0, 0.0, 0.5, 0.5, 0.025]),
                wet=wet,
                base_surface=np.zeros(shape, np.float32),
                surface=np.zeros(shape, np.float32),
                exclusion=np.zeros(shape, bool),
                terrain_count=terrain_count,
            )
            config = {
                "add_water_regions": [{
                    "polygon": [
                        [0.075, 0.20], [0.40, 0.20],
                        [0.40, 0.40], [0.075, 0.40],
                    ],
                    "max_terrain_count": 0,
                }],
            }
            surface = V10.load_surface_reference(run, config)
            self.assertTrue(surface.exclusion[10, 12])
            point_x = np.array([(12 + 0.5) * 0.025])
            point_y = np.array([(10 + 0.5) * 0.025])
            self.assertFalse(V10.footprint_contains(
                surface, point_x, point_y
            )[0])

    def test_harmonic_extension_pins_unanchored_component_to_nearest_level(self):
        values = np.zeros((5, 5), np.float32)
        fixed = np.zeros((5, 5), bool)
        fixed[0, 0] = True
        values[0, 0] = 2.5
        additions = np.zeros((5, 5), bool)
        additions[3:5, 3:5] = True
        got = V10._harmonic_review_extension(values, fixed, additions)
        np.testing.assert_allclose(got[3:5, 3:5], 2.5)


class CombinedTests(unittest.TestCase):
    def test_combined_uses_cleanmesh_weighted_clipped_normalization(self):
        dtype = np.dtype([
            (f"scalar_A_R_MeanCurvature_{scale}", "<f4")
            for scale in V10.SCALES
        ])
        values = np.zeros(2, dtype)
        values["scalar_A_R_MeanCurvature_Fine"] = [1.0, 100.0]
        values["scalar_A_R_MeanCurvature_Medium"] = [2.0, -100.0]
        values["scalar_A_R_MeanCurvature_Broad"] = [3.0, 0.0]
        got = V10._combined("MeanCurvature", values, [2.0, 4.0, 6.0])
        self.assertAlmostEqual(got[0], 0.5)
        self.assertAlmostEqual(got[1], 0.45 - 0.35)


if __name__ == "__main__":
    unittest.main()
