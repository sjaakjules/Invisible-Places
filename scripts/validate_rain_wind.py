#!/usr/bin/env python3
"""Validate rain pseudo-wind tuning and run the sample-scene rain checks."""

from __future__ import annotations

import argparse
import math
import pathlib
import subprocess
import sys


PRESETS = {
    "Light Mist": {
        "fall_speed": 2.8,
        "wind_strength": 0.62,
        "wind_noise": 0.82,
        "wind_response": 1.0,
    },
    "Rain": {
        "fall_speed": 8.0,
        "wind_strength": 0.30,
        "wind_noise": 0.45,
        "wind_response": 0.50,
    },
    "Heavy Downpour": {
        "fall_speed": 13.5,
        "wind_strength": 0.12,
        "wind_noise": 0.18,
        "wind_response": 0.16,
    },
}


def effective_response(preset: dict[str, float]) -> float:
    wind_noise = max(0.0, min(2.0, preset["wind_noise"]))
    return max(0.0, min(2.0, preset["wind_response"] * (0.55 + wind_noise * 0.45)))


def pseudo_wind_displacement(preset: dict[str, float]) -> float:
    return max(0.0, min(0.11, preset["wind_strength"] * effective_response(preset) * 0.10))


def fall_angle_degrees(preset: dict[str, float]) -> float:
    slope = max(0.0, min(0.22, preset["wind_strength"] * effective_response(preset) * 0.20))
    return math.degrees(math.atan(slope))


def validate_envelope() -> list[str]:
    issues: list[str] = []
    mist = PRESETS["Light Mist"]
    rain = PRESETS["Rain"]
    downpour = PRESETS["Heavy Downpour"]

    if not (1.5 <= mist["fall_speed"] <= 4.5):
        issues.append("Light Mist fall speed should stay in a slow mist/drizzle range.")
    if not (6.0 <= rain["fall_speed"] <= 10.0):
        issues.append("Rain fall speed should stay near moderate raindrop speeds.")
    if not (9.0 <= downpour["fall_speed"] <= 15.0):
        issues.append("Heavy Downpour fall speed should stay below hard terminal-velocity extremes.")

    displacements = [pseudo_wind_displacement(PRESETS[name]) for name in PRESETS]
    if not (displacements[0] > displacements[1] > displacements[2]):
        issues.append("Pseudo-wind displacement must damp from mist to heavy rain.")
    if not (0.040 <= displacements[0] <= 0.080):
        issues.append("Light Mist pseudo-wind displacement should stay around 4-8 cm.")
    if not (0.008 <= displacements[1] <= 0.020):
        issues.append("Rain pseudo-wind displacement should stay around 1 cm.")
    if not (displacements[2] <= 0.004):
        issues.append("Heavy Downpour pseudo-wind displacement should stay nearly ballistic.")

    angles = [fall_angle_degrees(PRESETS[name]) for name in PRESETS]
    if not (angles[0] > angles[1] > angles[2]):
        issues.append("Rain fall angle should damp from mist to heavy rain.")
    if not (4.0 <= angles[0] <= 8.0 and 0.8 <= angles[1] <= 2.0 and angles[2] <= 0.4):
        issues.append("Rain fall angles are outside the expected sheet-rain tuning envelope.")

    return issues


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--build-dir",
        default="build/macos-debug",
        help="CMake build directory containing invisible_places_tests.",
    )
    parser.add_argument(
        "--skip-tests",
        action="store_true",
        help="Only validate the numeric tuning envelope.",
    )
    args = parser.parse_args()

    repo = pathlib.Path(__file__).resolve().parents[1]
    issues = validate_envelope()
    print("Rain pseudo-wind tuning envelope:", flush=True)
    for name, preset in PRESETS.items():
        print(
            f"  {name:14s} fall={preset['fall_speed']:4.1f} m/s  "
            f"effective_wind={effective_response(preset):.3f}  "
            f"max_sway={pseudo_wind_displacement(preset) * 100.0:.1f} cm  "
            f"fall_angle={fall_angle_degrees(preset):.2f} deg",
            flush=True,
        )

    if issues:
        for issue in issues:
            print(f"ERROR: {issue}", file=sys.stderr)
        return 1

    if args.skip_tests:
        return 0

    ctest_regex = "Water Rain|SampleScene|Offline water trail overlays|Offline rain pseudo"
    command = [
        "ctest",
        "--test-dir",
        str(repo / args.build_dir),
        "--output-on-failure",
        "-R",
        ctest_regex,
    ]
    print("\nRunning validation tests:", flush=True)
    print("  " + " ".join(command), flush=True)
    return subprocess.run(command, cwd=repo, check=False).returncode


if __name__ == "__main__":
    raise SystemExit(main())
