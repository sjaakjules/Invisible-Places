#!/usr/bin/env python3
"""Copy visual-only "Surface" timing takes into another scene.

The Timings tab's takes became scene-scoped (schema 84): every existing take
belongs to the scene its state was authored under (Pools/Scene3). The purely
visual Surface takes translate to any scene whose clouds carry the same
scalar fields, so this tool copies them into the target scene (Fossils/
Scene1 by default) as "Surface01".."SurfaceNN" in creation order:

- a take qualifies when its states hold at least one visual feature
  (colourise/timing effects) and its water-feature runs are all empty;
- every scalar field referenced by an active field selector must exist in
  all three of the target scene's SAND/ROCK/VEG clouds (mirroring the
  field catalog's intersection rule); missing fields skip the take with a
  report;
- copies get fresh take ids from timing_take_sequence, scene_group on the
  take record, and a state copy holding the visual features with an empty
  water-run list.

Dry-run by default; --apply writes (timestamped backup + atomic replace).
Refuses to run while the app is open.
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import re
import shutil
import subprocess
import sys
from pathlib import Path


def app_running() -> bool:
    result = subprocess.run(["pgrep", "-f", "invisible_places.app"],
                            capture_output=True, text=True)
    return bool(result.stdout.strip())


def ply_scalar_fields(path: Path) -> set[str]:
    fields = set()
    with open(path, "rb") as handle:
        header = b""
        while not header.endswith(b"end_header\n"):
            line = handle.readline()
            if not line:
                raise RuntimeError(f"no end_header in {path}")
            header += line
    for line in header.decode("ascii", "replace").splitlines():
        parts = line.split()
        if len(parts) == 3 and parts[0] == "property":
            name = parts[2]
            # Field selectors store CloudCompare-normalised names without the
            # PLY "scalar_" prefix; compare in that namespace.
            fields.add(name.removeprefix("scalar_"))
    return fields


def effect_fields(state: dict) -> set[str]:
    names = set()
    for key in ("colourise_effects", "timing_effects"):
        for effect in state.get(key, []):
            field = effect.get("field", {})
            if field.get("source", "scalar") == "scalar":
                name = field.get("scalar_field_name", "")
                if name:
                    names.add(name)
    return names


def visual_only(states: list[dict]) -> bool:
    has_visuals = False
    for state in states:
        if state.get("colourise_effects") or state.get("timing_effects"):
            has_visuals = True
        for run in state.get("water_feature_timing_runs", []):
            if run.get("features"):
                return False
    return has_visuals


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("project", type=Path)
    parser.add_argument("--target-scene", default="Scene1")
    parser.add_argument("--target-data", type=Path,
                        default=Path("Data/Scene1"),
                        help="folder holding the target scene's clouds")
    parser.add_argument("--cloud-pattern", default="Site1-{role}-5mm.ply")
    parser.add_argument("--name-prefix", default="Surface")
    parser.add_argument("--take-filter", default="surface",
                        help="case-insensitive substring of source take names")
    parser.add_argument("--apply", action="store_true")
    parser.add_argument("--backup-dir", type=Path,
                        default=Path("Saved/repair_backups"))
    args = parser.parse_args()

    if args.apply and app_running():
        print("Refusing: invisible_places.app is running. Close it first "
              "(do not save) and re-run with --apply.")
        return 2

    target_fields = None
    for role in ("SAND", "ROCK", "VEG"):
        cloud = args.target_data / args.cloud_pattern.format(role=role)
        fields = ply_scalar_fields(cloud)
        target_fields = fields if target_fields is None else target_fields & fields
    print(f"target scene fields (intersection of SAND/ROCK/VEG): "
          f"{len(target_fields)}")

    document = json.loads(args.project.read_text())
    takes = document.get("timing_takes", [])
    states = document.get("timing_take_states", [])
    sequence = int(document.get("timing_take_sequence", 1))
    existing_names = {take.get("name", "") for take in takes}

    def take_number(take_id: str) -> int:
        match = re.fullmatch(r"timing-take-(\d+)", take_id)
        return int(match.group(1)) if match else -1

    candidates = []
    for take in sorted(takes, key=lambda t: take_number(t.get("id", ""))):
        name = take.get("name", "")
        if args.take_filter.lower() not in name.lower():
            continue
        if take.get("scene_group") == args.target_scene:
            continue
        own_states = [s for s in states if s.get("take_id") == take.get("id")]
        if not visual_only(own_states):
            print(f"  skip {name!r}: has water features")
            continue
        fields = set().union(*(effect_fields(s) for s in own_states)) \
            if own_states else set()
        missing = sorted(fields - target_fields)
        if missing:
            print(f"  skip {name!r}: fields missing in {args.target_scene}: "
                  f"{missing}")
            continue
        candidates.append((take, own_states, sorted(fields)))

    if not candidates:
        print("No takes to copy.")
        return 0

    print(f"\nCopy plan ({len(candidates)} takes -> {args.target_scene}):")
    new_takes, new_states = [], []
    ordinal = 1
    for take, own_states, fields in candidates:
        while f"{args.name_prefix}{ordinal:02d}" in existing_names:
            ordinal += 1
        new_name = f"{args.name_prefix}{ordinal:02d}"
        existing_names.add(new_name)
        new_id = f"timing-take-{sequence}"
        sequence += 1
        copy = dict(take)
        copy["id"] = new_id
        copy["name"] = new_name
        copy["scene_group"] = args.target_scene
        new_takes.append(copy)
        state_count = 0
        for state in own_states:
            if not (state.get("colourise_effects") or
                    state.get("timing_effects")):
                continue
            state_copy = dict(state)
            state_copy["take_id"] = new_id
            state_copy["scene_group"] = args.target_scene
            state_copy["water_feature_timing_runs"] = []
            state_copy["water_feature_timing_run_sequence"] = 1
            new_states.append(state_copy)
            state_count += 1
        print(f"  {take.get('name')!r} -> {new_name!r} (id {new_id}, "
              f"{state_count} state(s), fields {fields})")

    if not args.apply:
        print("\nDry run only. Re-run with --apply to write.")
        return 0

    document["timing_takes"] = takes + new_takes
    document["timing_take_states"] = states + new_states
    document["timing_take_sequence"] = sequence
    args.backup_dir.mkdir(parents=True, exist_ok=True)
    stamp = dt.datetime.now().strftime("%Y%m%d-%H%M%S")
    backup = args.backup_dir / f"{args.project.stem}.{stamp}.json"
    shutil.copy2(args.project, backup)
    print(f"backup: {backup}")
    temp = args.project.with_suffix(".json.copy_tmp")
    temp.write_text(json.dumps(document, indent=2))
    temp.replace(args.project)
    print(f"written: {args.project} (takes {len(takes)} -> "
          f"{len(document['timing_takes'])})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
