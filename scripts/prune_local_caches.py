#!/usr/bin/env python3
"""Prune machine-local derived caches that the app no longer reads.

Everything this script touches is regenerable or verifiably stale derived
data. It NEVER touches source PLYs, the shared OneDrive roots, project
JSON, animations, renders, or the patch-refinement rollback records.

Categories (all reported in the default dry run; deleted with --apply):

  legacy-roots      Data/<scene>/.invisible_places/cache/{fields,colourise_histograms}
                    The app reads field caches only from the root installed by
                    SetPointCloudFieldCacheRoot (the local Saved tree) since the
                    scalar-residency work; the beside-source copies are dead.
                    Scene-local flow caches are deliberately KEPT (the loader
                    prefers them) and water surface caches are handled below.
  stale-water       Data/<scene>/.invisible_places/cache/water/*.surfacecache and
                    Saved/.invisible_places/cache/water/*.surfacecache that are
                    (a) not referenced by any *_project.json in the local Saved
                    tree or the authored workspace, and (b) not among the
                    --keep-latest-water newest files of their directory.
  stale-fields      Saved/.invisible_places/cache/fields/<stem>-<hash> whose
                    manifest (source_size_bytes, source_mtime_ns) no longer
                    matches the current source file for that stem, or whose
                    stem has no source file anywhere in the known data roots.
  alias-fields      Duplicate Saved-side field caches: several path spellings
                    of the same source (OneDrive path, pre-symlink local path,
                    patch-run validation roots) hash to different directories
                    with identical (stem, size, mtime). The largest directory
                    (most cached fields) is kept per identity.
  stale-density     Non-active fingerprint bundles under
                    Saved/.invisible_places/cache/display_density/Scene3.
  retired           Saved/lod, Saved/PointCloudCache, Saved/cache/rain,
                    Saved/Saved, Saved/diagnostics/lod_compare - subsystems
                    with no remaining readers in src/.

Run from the repository root. Refuses --apply while an invisible_places
process is running (its caches may be open) unless --allow-running.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
LOCAL_SAVED = REPO / "Saved"
LOCAL_DATA = REPO / "Data"


def tree_size(path: Path) -> int:
    if path.is_file() or path.is_symlink():
        try:
            return path.lstat().st_size
        except OSError:
            return 0
    total = 0
    for root, _dirs, files in os.walk(path, onerror=lambda _e: None):
        for name in files:
            try:
                total += (Path(root) / name).lstat().st_size
            except OSError:
                pass
    return total


def gb(size: int) -> str:
    return f"{size / 1e9:7.2f} GB"


def data_scene_dirs() -> list[Path]:
    if not LOCAL_DATA.is_dir():
        return []
    return [entry for entry in sorted(LOCAL_DATA.iterdir()) if entry.is_dir()]


def workspace_roots() -> list[Path]:
    """Local Saved plus the authored workspace named by the repo marker."""
    roots = [LOCAL_SAVED]
    marker = REPO / ".invisible_places-workspace"
    if marker.is_file():
        configured = marker.read_text().strip()
        if configured:
            roots.append(Path(configured))
    return roots


def referenced_surfacecaches() -> set[str]:
    """Filenames of every .surfacecache referenced by any reachable project."""
    referenced: set[str] = set()
    pattern = re.compile(r"([0-9a-f]{16}\.surfacecache)")
    for root in workspace_roots():
        try:
            candidates = sorted(root.glob("*.json"))
        except OSError:
            continue  # workspace may be unreadable (OneDrive permission)
        for project in candidates:
            try:
                referenced.update(pattern.findall(project.read_text(errors="ignore")))
            except OSError:
                pass
    return referenced


def source_identity_by_stem() -> dict[str, tuple[int, int]]:
    """Current (size, mtime_ns) for every candidate PLY stem the caches may
    describe, searched across the local data tree (symlinks resolve into the
    shared OneDrive set, so shared sources are covered transitively)."""
    identities: dict[str, tuple[int, int]] = {}
    if not LOCAL_DATA.is_dir():
        return identities
    for root, _dirs, files in os.walk(LOCAL_DATA):
        if ".invisible_places" in root:
            continue
        for name in files:
            if not name.endswith(".ply"):
                continue
            path = Path(root) / name
            try:
                stat = path.stat()  # follows symlinks
            except OSError:
                continue
            identities[path.stem] = (stat.st_size, stat.st_mtime_ns)
    return identities


def collect(args: argparse.Namespace) -> list[tuple[str, Path, int, str]]:
    items: list[tuple[str, Path, int, str]] = []

    # legacy-roots -----------------------------------------------------------
    for scene in data_scene_dirs():
        for sub in ("fields", "colourise_histograms"):
            legacy = scene / ".invisible_places" / "cache" / sub
            if legacy.is_dir():
                items.append(("legacy-roots", legacy, tree_size(legacy),
                              "beside-source cache root; app reads the Saved root"))

    # stale-water ------------------------------------------------------------
    keep_latest = max(0, args.keep_latest_water)
    referenced = referenced_surfacecaches()
    water_dirs = [scene / ".invisible_places" / "cache" / "water"
                  for scene in data_scene_dirs()]
    water_dirs.append(LOCAL_SAVED / ".invisible_places" / "cache" / "water")
    for water in water_dirs:
        if not water.is_dir():
            continue
        caches = sorted(water.glob("*.surfacecache"),
                        key=lambda p: p.stat().st_mtime, reverse=True)
        for index, cache in enumerate(caches):
            if cache.name in referenced:
                continue
            if index < keep_latest:
                continue
            items.append(("stale-water", cache, tree_size(cache),
                          "surface-cache fingerprint not referenced by any project"))

    # stale-fields / alias-fields ---------------------------------------------
    identities = source_identity_by_stem()
    fields_root = LOCAL_SAVED / ".invisible_places" / "cache" / "fields"
    groups: dict[tuple[str, int, int], list[tuple[Path, int]]] = {}
    if fields_root.is_dir():
        for entry in sorted(fields_root.iterdir()):
            if not entry.is_dir():
                continue
            stem = entry.name.rsplit("-", 1)[0]
            manifest_path = entry / "manifest.json"
            try:
                manifest = json.loads(manifest_path.read_text())
                identity = (int(manifest["source_size_bytes"]),
                            int(manifest["source_mtime_ns"]))
            except (OSError, KeyError, ValueError):
                items.append(("stale-fields", entry, tree_size(entry),
                              "unreadable manifest"))
                continue
            current = identities.get(stem)
            if current is None:
                items.append(("stale-fields", entry, tree_size(entry),
                              f"no source file named {stem}.ply exists any more"))
                continue
            if current != identity:
                items.append(("stale-fields", entry, tree_size(entry),
                              "source file changed since this cache was built"))
                continue
            groups.setdefault((stem, *identity), []).append(
                (entry, tree_size(entry)))
    for (stem, _size, _mtime), entries in groups.items():
        if len(entries) < 2:
            continue
        # Keep the directory with the most cached bytes (most fields); the
        # others are path-spelling aliases of the same source identity.
        entries.sort(key=lambda pair: pair[1], reverse=True)
        for entry, size in entries[1:]:
            items.append(("alias-fields", entry, size,
                          f"duplicate of {entries[0][0].name} (same source identity)"))

    # stale-density ------------------------------------------------------------
    density = LOCAL_SAVED / ".invisible_places" / "cache" / "display_density" / "Scene3"
    active = None
    active_file = density / "active-bundle.json"
    if active_file.is_file():
        try:
            active = json.loads(active_file.read_text()).get("fingerprint")
        except (OSError, ValueError):
            active = None
    if density.is_dir() and active:
        for entry in sorted(density.iterdir()):
            if entry.is_dir() and entry.name not in (active, "") and \
                    not entry.name.endswith(".work"):
                if entry.name != active:
                    items.append(("stale-density", entry, tree_size(entry),
                                  f"not the active bundle ({active[:12]}...)"))
            elif entry.is_dir() and entry.name.endswith(".work"):
                items.append(("stale-density", entry, tree_size(entry),
                              "leftover staging directory"))

    # retired -------------------------------------------------------------------
    for retired in ("lod", "PointCloudCache", "cache/rain", "Saved",
                    "diagnostics/lod_compare"):
        path = LOCAL_SAVED / retired
        if path.exists():
            items.append(("retired", path, tree_size(path),
                          "retired subsystem; no readers left in src/"))

    return items


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--apply", action="store_true",
                        help="delete the listed items (default: dry run)")
    parser.add_argument("--only", action="append", default=None,
                        help="restrict to one or more categories")
    parser.add_argument("--keep-latest-water", type=int, default=1,
                        help="always keep the N newest unreferenced .surfacecache "
                             "files per directory (default 1)")
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

    items = collect(args)
    if args.only:
        wanted = set(args.only)
        items = [item for item in items if item[0] in wanted]

    if not items:
        print("Nothing to prune.")
        return 0

    total = 0
    by_category: dict[str, int] = {}
    for category, path, size, reason in items:
        total += size
        by_category[category] = by_category.get(category, 0) + size
        print(f"{category:14} {gb(size)}  {path.relative_to(REPO) if path.is_relative_to(REPO) else path}"
              f"\n{'':14} {'':10}  - {reason}")

    print("\nPer category:")
    for category, size in sorted(by_category.items(), key=lambda kv: -kv[1]):
        print(f"  {category:14} {gb(size)}")
    print(f"  {'TOTAL':14} {gb(total)}")

    if not args.apply:
        print("\nDry run only. Re-run with --apply to delete.")
        return 0

    failures = 0
    for _category, path, _size, _reason in items:
        try:
            if path.is_dir() and not path.is_symlink():
                shutil.rmtree(path)
            else:
                path.unlink()
        except OSError as error:
            failures += 1
            print(f"FAILED to remove {path}: {error}", file=sys.stderr)
    print(f"Removed {len(items) - failures} item(s), {failures} failure(s).")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
