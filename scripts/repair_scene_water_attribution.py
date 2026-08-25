#!/usr/bin/env python3
"""Repair a project whose water scene state was saved under the wrong scene.

The 2026-08-25 scene switcher initially let the Lidar panel flip the active
water scene without swapping the per-scene water objects, so a save while the
wrong scene was named wrote Scene3's authored water (seepage nodes, manual
flow paths, emitters, shoreline effects) into a state named "Scene1" and left
the top-level lists empty. This tool moves the state back:

- renames the misattributed water_scene_states entry (--from-scene, default
  Scene1) to the owning scene (--to-scene, default Scene3), merging into an
  existing target state only if that target is empty;
- when the repaired scene is the active_water_scene_group, mirrors its lists
  into the top-level water_* fields the loader treats as authoritative;
- writes a timestamped backup beside the local Saved directory first and
  replaces the project atomically.

Dry-run by default; pass --apply to write. Refuses to run while the app is
open (the app's own save would race the repair).
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import shutil
import subprocess
import sys
from pathlib import Path

STATE_LISTS = ["water_emitters", "water_manual_flow_paths", "water_seepage_nodes",
               "water_shoreline_instances"]


def app_running() -> bool:
    result = subprocess.run(["pgrep", "-f", "invisible_places.app"],
                            capture_output=True, text=True)
    return bool(result.stdout.strip())


def state_content(state: dict) -> str:
    return ", ".join(f"{key.split('water_')[1]}={len(state.get(key, []))}"
                     for key in STATE_LISTS)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("project", type=Path, help="path to the project JSON")
    parser.add_argument("--from-scene", default="Scene1")
    parser.add_argument("--to-scene", default="Scene3")
    parser.add_argument("--apply", action="store_true")
    parser.add_argument("--backup-dir", type=Path,
                        default=Path("Saved/repair_backups"))
    args = parser.parse_args()

    if args.apply and app_running():
        print("Refusing: invisible_places.app is running. Close it first "
              "(do not save) and re-run with --apply.")
        return 2

    document = json.loads(args.project.read_text())
    states = document.get("water_scene_states", [])
    active = document.get("active_water_scene_group", "")
    print(f"active_water_scene_group: {active!r}")
    for state in states:
        print(f"  state {state.get('scene_group')!r}: {state_content(state)}")

    source = next((s for s in states
                   if s.get("scene_group") == args.from_scene), None)
    if source is None:
        print(f"No state named {args.from_scene!r}; nothing to repair.")
        return 0
    if not any(source.get(key) for key in STATE_LISTS):
        print(f"State {args.from_scene!r} is empty; nothing to move.")
        return 0

    target = next((s for s in states
                   if s.get("scene_group") == args.to_scene), None)
    if target is not None and any(target.get(key) for key in STATE_LISTS):
        print(f"Target state {args.to_scene!r} already has content "
              f"({state_content(target)}); refusing to overwrite. "
              "Resolve manually.")
        return 2

    print(f"\nRepair: rename state {args.from_scene!r} -> {args.to_scene!r} "
          f"({state_content(source)})")
    if target is not None:
        states.remove(target)
        print(f"  (dropping the empty existing {args.to_scene!r} state)")
    source["scene_group"] = args.to_scene
    if active == args.to_scene:
        for key in STATE_LISTS:
            if key in source:
                document[key] = source[key]
        print(f"  mirrored the state's lists into the top-level fields "
              f"(active scene is {args.to_scene!r})")

    if not args.apply:
        print("\nDry run only. Re-run with --apply to write.")
        return 0

    args.backup_dir.mkdir(parents=True, exist_ok=True)
    stamp = dt.datetime.now().strftime("%Y%m%d-%H%M%S")
    backup = args.backup_dir / f"{args.project.stem}.{stamp}.json"
    shutil.copy2(args.project, backup)
    print(f"backup: {backup}")
    temp = args.project.with_suffix(".json.repair_tmp")
    temp.write_text(json.dumps(document, indent=2))
    temp.replace(args.project)
    print(f"repaired: {args.project}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
