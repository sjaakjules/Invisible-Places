#!/usr/bin/env python3
"""Assess base/detail profile captures the way After Effects composites them.

The Surface two-export workflow applies the detail (thin) export over the
base with its own luma as alpha (luma matte) and Multiply blend. Per channel
with normalized colours that is

    out = base * (1 - alpha * (1 - thin)),  alpha = luma(thin)

so pure black detail pixels (alpha 0) leave the base untouched and pure
white multiplies by one — only midtones carve detail into the base. A
high-pass metric on the isolated detail frame therefore over-counts: the
black gaps between small points register as texture even though they vanish
in the composite. This module measures what actually survives:

  * composite detail gain  = highpass(composite) - highpass(base)
  * false-detail fraction  = share of the isolated high-pass reading that
                             does not survive compositing
  * mean darkening         = how much the detail layer dims the base overall

and scores base candidates on the smooth-but-not-blurry axes:

  * speckle band   = std(L - box9(L))    -> low for a smooth wash
  * structure band = std(box9 - box31)   -> high when form survives

Usage:
    python3 scripts/analyze_surface_composites.py \
        --base <base capture.ppm> --thin <thin capture.ppm> [...]

Captures come from the surface-profile-lab smoke (binary PPMs).
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import numpy as np

LUMA_WEIGHTS = np.array([0.2126, 0.7152, 0.0722], dtype=np.float64)


def read_ppm(path: Path) -> np.ndarray:
    """Binary P6 PPM to float RGB in [0, 1]."""
    data = path.read_bytes()
    if not data.startswith(b"P6"):
        raise ValueError(f"{path} is not a binary P6 PPM")
    parts = data.split(b"\n", 3)
    width, height = map(int, parts[1].split())
    maximum = int(parts[2])
    if maximum != 255:
        raise ValueError(f"{path} must be 8-bit (maxval 255)")
    pixels = np.frombuffer(parts[3], dtype=np.uint8, count=width * height * 3)
    return pixels.reshape(height, width, 3).astype(np.float64) / 255.0


def luma(rgb: np.ndarray) -> np.ndarray:
    return rgb @ LUMA_WEIGHTS


def box_blur(image: np.ndarray, radius: int) -> np.ndarray:
    """Exact box blur with edge clamping via integral images."""
    size = 2 * radius + 1
    padded = np.pad(image, radius, mode="edge")
    csum = np.cumsum(np.cumsum(padded, axis=0), axis=1)
    csum = np.pad(csum, ((1, 0), (1, 0)))
    total = (
        csum[size:, size:]
        - csum[:-size, size:]
        - csum[size:, :-size]
        + csum[:-size, :-size]
    )
    return total / (size * size)


def composite_multiply_luma_matte(
    base: np.ndarray,
    thin: np.ndarray,
) -> np.ndarray:
    """After Effects Multiply blend with the thin layer's luma as its matte."""
    alpha = luma(thin)[..., None]
    return base * (1.0 - alpha * (1.0 - thin))


def bands(image_luma: np.ndarray) -> tuple[float, float]:
    """(speckle, structure): micro high-pass std and mid-band std."""
    fine = box_blur(image_luma, 4)
    broad = box_blur(image_luma, 15)
    return float(np.std(image_luma - fine)), float(np.std(fine - broad))


def assess_composite(base: np.ndarray, thin: np.ndarray) -> dict:
    composite = composite_multiply_luma_matte(base, thin)
    base_luma = luma(base)
    thin_luma = luma(thin)
    composite_luma = luma(composite)
    isolated_speckle, _ = bands(thin_luma)
    base_speckle, base_structure = bands(base_luma)
    composite_speckle, composite_structure = bands(composite_luma)
    gain = composite_speckle - base_speckle
    false_fraction = (
        max(0.0, 1.0 - gain / isolated_speckle)
        if isolated_speckle > 0.0
        else 0.0
    )
    return {
        "isolated_detail": isolated_speckle,
        "base_detail": base_speckle,
        "composite_detail": composite_speckle,
        "composite_gain": gain,
        "false_detail_fraction": false_fraction,
        "mean_darkening": float(np.mean(base_luma - composite_luma)),
        "base_structure": base_structure,
        "composite_structure": composite_structure,
    }


def assess_base(base: np.ndarray, lit_threshold: float = 0.08) -> dict:
    base_luma = luma(base)
    speckle, structure = bands(base_luma)
    lit = base_luma > lit_threshold
    return {
        "speckle": speckle,
        "structure": structure,
        "lit_fraction": float(lit.mean()),
        "p10_lit_luma": float(
            np.percentile(base_luma[lit], 10.0) if lit.any() else 0.0
        ),
    }


def run(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--base", required=True, type=Path)
    parser.add_argument(
        "--thin",
        action="append",
        default=[],
        type=Path,
        help="Detail capture(s) to composite over the base (repeatable).",
    )
    parser.add_argument("--json-out", type=Path)
    arguments = parser.parse_args(argv)

    base = read_ppm(arguments.base)
    report = {"base": {arguments.base.name: assess_base(base)}}
    print(f"base {arguments.base.name}: {report['base'][arguments.base.name]}")
    composites = {}
    for thin_path in arguments.thin:
        result = assess_composite(base, read_ppm(thin_path))
        composites[thin_path.name] = result
        print(
            f"{thin_path.name}: gain {result['composite_gain']:.4f}, "
            f"false {result['false_detail_fraction'] * 100.0:.1f}%, "
            f"darken {result['mean_darkening']:.4f}"
        )
    report["composites"] = composites
    if arguments.json_out is not None:
        arguments.json_out.write_text(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(run())
