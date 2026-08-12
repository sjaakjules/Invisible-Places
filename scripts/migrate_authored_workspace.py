#!/usr/bin/env python3
"""Copy authored Invisible Places state into a portable live workspace.

Renders, caches, builds, validation output, and legacy multi-GB project
snapshots are deliberately excluded. Authored source files are left in place
as a rollback copy. An optional source-data destination moves only the selected
production PLY set because duplicating those files would require another
31 GB; the app uses the destination directly on its next launch.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import shutil
import sys
from typing import Any


PROJECT_FILENAME = "ExhibitionFinal_project.json"
PORTABLE_DIRECTORIES = ("animations",)
PORTABLE_FILES = (
    PROJECT_FILENAME,
    "pointcloud_style_preset.json",
    "water_sources.json",
)
SHARED_SOURCE_NAMES = (
    "Site3-ROCK-1mm.ply",
    "Site3-SAND-1mm.ply",
    "Site3-VEG-1mm.ply",
    "Site3-ROCK-5mm.ply",
    "Site3-SAND-5mm.ply",
    "Site3-VEG-5mm.ply",
    "Site3-MESH.ply",
    "Site3-MESHSampled-5mm.ply",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", type=Path, required=True)
    parser.add_argument("--destination", type=Path, required=True)
    parser.add_argument(
        "--shared-data-destination",
        type=Path,
        help=(
            "Move only Scene3 1 mm, 5 mm, MESH, and MESHSampled PLY sources "
            "into this synchronized folder and write the local data marker."
        ),
    )
    parser.add_argument(
        "--shared-data-only",
        action="store_true",
        help="Move/configure the selected production PLY set without recopying authored JSON.",
    )
    parser.add_argument(
        "--without-water-sources",
        action="store_true",
        help="Skip the optional standalone water_sources.json file.",
    )
    return parser.parse_args()


def portable_string(value: str, repo: Path, destination: Path) -> str:
    if not value or value.startswith("@") or value.startswith("__scene_group__/"):
        return value
    normalized = value.replace("\\", "/")
    repo_prefix = repo.resolve().as_posix().rstrip("/") + "/"
    destination_prefix = destination.resolve().as_posix().rstrip("/") + "/"
    if normalized.startswith(destination_prefix):
        return "@workspace/" + normalized[len(destination_prefix) :]
    if normalized.startswith(repo_prefix + "Data/"):
        return "@data/" + normalized[len(repo_prefix + "Data/") :]
    if normalized.startswith(repo_prefix + "Saved/animations/"):
        return "@workspace/animations/" + normalized[
            len(repo_prefix + "Saved/animations/") :
        ]
    render_prefix = repo_prefix + "Saved/renders/Invisible Places"
    if normalized == render_prefix:
        return "@local-renders"
    if normalized.startswith(render_prefix + "/"):
        return "@local-renders/" + normalized[len(render_prefix) + 1 :]
    legacy_video_prefix = repo_prefix + "Saved/renders/Videos"
    if normalized == legacy_video_prefix:
        return "@local-renders"
    if normalized.startswith(legacy_video_prefix + "/"):
        return "@local-renders/" + normalized[len(legacy_video_prefix) + 1 :]
    return value


def rewrite_json(value: Any, repo: Path, destination: Path) -> Any:
    if isinstance(value, dict):
        return {
            key: rewrite_json(child, repo, destination)
            for key, child in value.items()
        }
    if isinstance(value, list):
        return [rewrite_json(child, repo, destination) for child in value]
    if isinstance(value, str):
        return portable_string(value, repo, destination)
    return value


def remove_derived_cache_state(document: Any) -> None:
    if not isinstance(document, dict):
        return
    for key in (
        "water_path_cache",
        "water_path_cache_manifest",
        "water_ripple_runtime_caches",
    ):
        document.pop(key, None)
    for group in document.get("scene_point_cloud_groups", []):
        if isinstance(group, dict):
            group.pop("water_surface_cache", None)
    for state in document.get("water_scene_states", []):
        if not isinstance(state, dict):
            continue
        state.pop("water_path_cache", None)
        state.pop("water_path_cache_manifest", None)
        state.pop("water_ripple_runtime_caches", None)


def rewrite_json_file(path: Path, repo: Path, destination: Path) -> None:
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise RuntimeError(f"Could not read {path}: {error}") from error
    portable = rewrite_json(document, repo, destination)
    remove_derived_cache_state(portable)
    temporary = path.with_suffix(path.suffix + ".portable.tmp")
    temporary.write_text(
        json.dumps(portable, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )
    temporary.replace(path)


def copy_authored_state(
    repo: Path,
    destination: Path,
    include_water_sources: bool,
) -> None:
    saved = repo / "Saved"
    destination.mkdir(parents=True, exist_ok=True)
    for name in PORTABLE_DIRECTORIES:
        source = saved / name
        target = destination / name
        if source.is_dir():
            shutil.copytree(source, target, dirs_exist_ok=True)
    # `render_setups/history.json` indexes machine-local output sidecars; it
    # is not an authored named setup and must not be copied between machines.
    render_setups_source = saved / "render_setups"
    render_setups_target = destination / "render_setups"
    render_setups_target.mkdir(parents=True, exist_ok=True)
    for source in render_setups_source.glob("*.iprender.json"):
        shutil.copy2(source, render_setups_target / source.name)
    for name in PORTABLE_FILES:
        if name == "water_sources.json" and not include_water_sources:
            continue
        source = saved / name
        if source.is_file():
            shutil.copy2(source, destination / name)

    for path in destination.rglob("*.json"):
        rewrite_json_file(path, repo, destination)

    readme = destination / "README_SYNC.md"
    readme.write_text(
        """# Invisible Places shared authored workspace

This folder is the live source of truth for project JSON, animation JSON,
named render setups, point-style presets, and optional standalone water state.
Cloud sync runs after every successful application save.

Do not place point-cloud PLY files, renders, caches, build output, or validation
artifacts here. The separately configured Shared Source Data folder may hold
the production 1 mm/5 mm and mesh PLY subset.

Workflow: close Invisible Places on computer 1, wait for the cloud provider to
say this folder is up to date, ensure it is fully downloaded on computer 2,
then open it there. The app refuses stale project/animation overwrites, but two
offline writers still cannot be distributed-locked by OneDrive; use Save As
with different names when intentionally working in parallel.
""",
        encoding="utf-8",
    )

    marker = repo / ".invisible_places-workspace"
    temporary = marker.with_suffix(".tmp")
    temporary.write_text(str(destination.resolve()) + "\n", encoding="utf-8")
    temporary.replace(marker)


def move_shared_source_data(repo: Path, destination: Path) -> None:
    source_scene = repo / "Data" / "Scene3"
    target_scene = destination / "Scene3"
    target_scene.mkdir(parents=True, exist_ok=True)
    missing = [name for name in SHARED_SOURCE_NAMES if not (source_scene / name).is_file()]
    already_moved = [name for name in missing if (target_scene / name).is_file()]
    missing = [name for name in missing if name not in already_moved]
    if missing:
        raise RuntimeError(
            "Required production sources are missing: " + ", ".join(missing)
        )
    for name in SHARED_SOURCE_NAMES:
        source = source_scene / name
        target = target_scene / name
        if target.exists():
            if source.is_symlink() and source.resolve() == target.resolve():
                continue
            if source.exists() and source.stat().st_size != target.stat().st_size:
                raise RuntimeError(f"Destination already contains a different {name}")
            if source.exists():
                raise RuntimeError(
                    f"Both source and destination contain {name}; refusing to choose one"
                )
        else:
            shutil.move(str(source), str(target))
        # Preserve the familiar repository-relative Data layout for local
        # validation utilities. The desktop app itself resolves the shared
        # root directly and does not depend on these machine-local links.
        source.symlink_to(target)

    readme = destination / "README_SYNC.md"
    readme.write_text(
        """# Invisible Places shared source data

Only production Scene3 1 mm and 5 mm ROCK/SAND/VEG PLY files plus the MESH and
5 mm MESHSampled PLY belong here. Keep 2 mm/3 mm clouds, Gaussian splats,
SampleScene validation assets, caches, renders, render history, builds, and
validation output local to each computer.

Before launching Invisible Places on a computer, make this folder available
offline in OneDrive. Configure it with the Project panel, the ignored
`.invisible_places-data-workspace` marker, or
`INVISIBLE_PLACES_SHARED_DATA_DIR`. The app writes no caches beside these PLYs.
""",
        encoding="utf-8",
    )
    marker = repo / ".invisible_places-data-workspace"
    temporary = marker.with_suffix(".tmp")
    temporary.write_text(str(destination.resolve()) + "\n", encoding="utf-8")
    temporary.replace(marker)


def main() -> int:
    args = parse_args()
    repo = args.repo.resolve()
    destination = args.destination.expanduser().resolve()
    if not (repo / "Saved").is_dir():
        print(f"Saved directory not found below {repo}", file=sys.stderr)
        return 2
    if destination == repo or repo in destination.parents:
        print("Destination must be outside the repository.", file=sys.stderr)
        return 2
    if args.shared_data_destination is not None:
        shared_data_destination = args.shared_data_destination.expanduser().resolve()
        if shared_data_destination == repo or repo in shared_data_destination.parents:
            print(
                "Shared data destination must be outside the repository.",
                file=sys.stderr,
            )
            return 2
        if shared_data_destination == destination:
            print(
                "Authored workspace and shared data must be separate folders.",
                file=sys.stderr,
            )
            return 2
    if args.shared_data_only and args.shared_data_destination is None:
        print("--shared-data-only requires --shared-data-destination.", file=sys.stderr)
        return 2
    try:
        if not args.shared_data_only:
            copy_authored_state(
                repo,
                destination,
                include_water_sources=not args.without_water_sources,
            )
        if args.shared_data_destination is not None:
            move_shared_source_data(
                repo,
                args.shared_data_destination.expanduser().resolve(),
            )
    except (OSError, RuntimeError) as error:
        print(str(error), file=sys.stderr)
        return 1
    if not args.shared_data_only:
        print(f"Authored workspace ready: {destination}")
        print(f"Local marker written: {repo / '.invisible_places-workspace'}")
    if args.shared_data_destination is not None:
        print(
            "Shared source data ready: "
            f"{args.shared_data_destination.expanduser().resolve()}"
        )
        print(
            "Local data marker written: "
            f"{repo / '.invisible_places-data-workspace'}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
