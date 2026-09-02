from __future__ import annotations

import unittest

import numpy as np

from scripts import analyze_surface_composites as composites


def flat(rgb: tuple[float, float, float], size: int = 32) -> np.ndarray:
    image = np.zeros((size, size, 3), dtype=np.float64)
    image[...] = rgb
    return image


class CompositeModelTests(unittest.TestCase):
    def test_black_detail_leaves_base_untouched(self) -> None:
        base = flat((0.6, 0.5, 0.4))
        thin = flat((0.0, 0.0, 0.0))
        out = composites.composite_multiply_luma_matte(base, thin)
        np.testing.assert_allclose(out, base)

    def test_white_detail_leaves_base_untouched(self) -> None:
        base = flat((0.6, 0.5, 0.4))
        thin = flat((1.0, 1.0, 1.0))
        out = composites.composite_multiply_luma_matte(base, thin)
        np.testing.assert_allclose(out, base)

    def test_midtone_detail_darkens_most(self) -> None:
        base = flat((0.8, 0.8, 0.8))
        darkest = None
        for level in (0.1, 0.3, 0.5, 0.7, 0.9):
            out = composites.composite_multiply_luma_matte(
                base,
                flat((level, level, level)),
            )
            factor = float(out.mean() / base.mean())
            if darkest is None or factor < darkest[1]:
                darkest = (level, factor)
        # alpha * (1 - thin) peaks at thin = 0.5.
        self.assertEqual(darkest[0], 0.5)
        self.assertAlmostEqual(darkest[1], 0.75, places=5)

    def test_black_gap_speckle_is_false_detail(self) -> None:
        # A detail layer made of bright points over pure black gaps reads as
        # huge isolated "detail", but black composites to nothing: most of
        # the isolated reading must be flagged as false.
        rng = np.random.default_rng(7)
        base = flat((0.7, 0.65, 0.6), 64)
        thin = np.zeros((64, 64, 3))
        mask = rng.random((64, 64)) < 0.25
        thin[mask] = (1.0, 1.0, 1.0)
        result = composites.assess_composite(base, thin)
        self.assertGreater(result["isolated_detail"], 0.2)
        self.assertGreater(result["false_detail_fraction"], 0.5)

    def test_midtone_texture_beats_black_gap_speckle(self) -> None:
        # The metric must rank a midtone texture (which multiply carves into
        # the base) ahead of the same-energy bright-on-black speckle (which
        # the luma matte erases): higher surviving gain, lower false share.
        rng = np.random.default_rng(11)
        base = flat((0.7, 0.65, 0.6), 64)
        texture = 0.3 + 0.4 * rng.random((64, 64))
        midtone = np.repeat(texture[..., None], 3, axis=2)
        gaps = np.zeros((64, 64, 3))
        gaps[rng.random((64, 64)) < 0.25] = (1.0, 1.0, 1.0)
        midtone_result = composites.assess_composite(base, midtone)
        gaps_result = composites.assess_composite(base, gaps)
        self.assertGreater(midtone_result["composite_gain"], 0.0)
        self.assertGreater(result_ratio := (
            midtone_result["composite_gain"] /
            max(gaps_result["composite_gain"], 1e-9)), 1.0)
        self.assertLess(
            midtone_result["false_detail_fraction"],
            gaps_result["false_detail_fraction"],
        )
        self.assertGreater(midtone_result["mean_darkening"], 0.05)

    def test_bands_separate_speckle_from_structure(self) -> None:
        rng = np.random.default_rng(3)
        size = 96
        coordinates = np.arange(size, dtype=np.float64)
        # Mid-frequency waves (period ~32 px) live in the structure band; a
        # pure linear ramp is box-blur invariant and carries neither.
        waves = 0.5 + 0.25 * np.sin(
            2.0 * np.pi * coordinates[None, :] / 32.0
        ) * np.sin(2.0 * np.pi * coordinates[:, None] / 32.0)
        speckle_noise = rng.random((size, size))
        speckle_of_waves, structure_of_waves = composites.bands(waves)
        speckle_of_noise, structure_of_noise = composites.bands(
            speckle_noise
        )
        self.assertLess(speckle_of_waves, structure_of_waves)
        self.assertGreater(structure_of_waves, 0.02)
        self.assertGreater(speckle_of_noise, 0.2)
        self.assertGreater(speckle_of_noise, structure_of_noise * 4.0)

    def test_box_blur_preserves_flat_images(self) -> None:
        image = np.full((20, 20), 0.37)
        np.testing.assert_allclose(
            composites.box_blur(image, 4),
            image,
            atol=1e-12,
        )


if __name__ == "__main__":
    unittest.main()
