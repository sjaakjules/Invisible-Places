#!/usr/bin/env python3
"""Remove the user-confirmed Tier-2 leftovers (2026-08-25 decision).

Removes exactly three things, all confirmed unreferenced by the live
project and by the app's catalog:

  Data/Scene3/Site3-{ROCK,SAND,VEG}-{2,3}mm.ply   (~16.2 GB)
      Locally generated decimations, retired from the catalog by
      SupplementalLocalPointCloudAllowed, never uploaded to the OneDrive
      share, referenced zero times by the live shared project. NOTE: the
      patch-refinement tool's `restore` for the 2 mm/3 mm spacings becomes
      impossible afterwards (1 mm and 5 mm rollback is unaffected); the
      variants are regenerable from the 1 mm sources if ever wanted.
  Data/Scene3/tmp-CLEANED.bin                     (~8.7 GB)
      Orphaned intermediate from the Aug 9 patch install; referenced by
      nothing in code, docs, scripts, or any project.
  Saved/CodexWork                                 (~3.1 GB)
      Aug 8/20 agent working directories (candidate projects/animations
      plus review MP4s), long since published or rejected.

Deliberately KEPT (do not add here without a new decision):
  Saved/renders/Videos, the old Saved/*.json projects,
  Data/Scene3/LinearNoisePAtchPoints.ply (Patch-04 input),
  build/macos-sanitized, Data/Scene3/PatchRefinement (rollback records,
  including review-images).

Dry run by default; --apply deletes. Refuses to apply while
invisible_places is running unless --allow-running.
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent

TARGETS = [
    *(REPO / "Data" / "Scene3" / f"Site3-{role}-{mm}.ply"
      for role in ("ROCK", "SAND", "VEG") for mm in ("2mm", "3mm")),
    REPO / "Data" / "Scene3" / "tmp-CLEANED.bin",
    REPO / "Saved" / "CodexWork",
]


def tree_size(path: Path) -> int:
    if path.is_file():
        return path.stat().st_size
    return sum(f.stat().st_size for f in path.rglob("*") if f.is_file())


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--apply", action="store_true",
                        help="delete the listed items (default: dry run)")
    parser.add_argument("--allow-running", action="store_true",
                        help="proceed even while invisible_places is running")
    args = parser.parse_args()

    if args.apply and not args.allow_running:
        probe = subprocess.run(["pgrep", "-f", "invisible_places.app"],
                               capture_output=True, text=True)
        if probe.stdout.strip():
            print("invisible_places is running; close it first or pass "
                  "--allow-running.", file=sys.stderr)
            return 2

    total = 0
    present: list[tuple[Path, int]] = []
    for target in TARGETS:
        # Symlinks are never expected here; refuse rather than follow one.
        if target.is_symlink():
            print(f"SKIP (symlink, unexpected): {target}", file=sys.stderr)
            continue
        if not target.exists():
            print(f"already gone: {target.relative_to(REPO)}")
            continue
        size = tree_size(target)
        total += size
        present.append((target, size))
        print(f"{size / 1e9:7.2f} GB  {target.relative_to(REPO)}")

    print(f"{total / 1e9:7.2f} GB  TOTAL")
    if not present:
        print("Nothing left to remove.")
        return 0
    if not args.apply:
        print("\nDry run only. Re-run with --apply to delete.")
        return 0

    failures = 0
    for target, _size in present:
        try:
            if target.is_dir():
                shutil.rmtree(target)
            else:
                target.unlink()
        except OSError as error:
            failures += 1
            print(f"FAILED to remove {target}: {error}", file=sys.stderr)
    print(f"Removed {len(present) - failures} item(s), {failures} failure(s).")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
