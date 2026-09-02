#!/usr/bin/env python3
"""Append new point-visual profiles to a saved project's visual library.

The profile lab iterates on duplicated project copies while the artist keeps
working in the live app, so improved profiles land in the real project only
after the app is closed. This tool performs that hand-off: it takes the
artist's latest save, appends the new named profiles to ``point_visuals``,
and leaves every other byte of authored state (timings, animations, scene
visual states, selections) untouched.

Safety properties:
  * refuses to run while an Invisible Places instance is open (the app would
    overwrite the merge on its next save) unless ``--force-while-running``;
  * refuses to replace an existing profile name unless ``--replace``;
  * writes a verbatim backup copy next to the project before modifying it;
  * writes atomically (temp file + rename) so an interrupted run cannot
    truncate the project.
"""

from __future__ import annotations

import argparse
import datetime as _datetime
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile


class MergeError(RuntimeError):
    """A validation failure that should abort without touching the project."""


def invisible_places_running() -> bool:
    """True when a live app instance could overwrite the merged project."""
    try:
        probe = subprocess.run(
            ["pgrep", "-f", "invisible_places.app/Contents/MacOS"],
            check=False,
            capture_output=True,
        )
    except OSError:
        return False
    return probe.returncode == 0


def load_new_profiles(profiles_path: Path) -> list[dict]:
    """Read and validate the profiles file (a list or {"point_visuals": []})."""
    with open(profiles_path, "r", encoding="utf-8") as handle:
        document = json.load(handle)
    if isinstance(document, dict):
        entries = document.get("point_visuals")
    else:
        entries = document
    if not isinstance(entries, list) or not entries:
        raise MergeError(
            f"{profiles_path} holds no point_visuals list to merge."
        )
    seen_names: set[str] = set()
    for entry in entries:
        if not isinstance(entry, dict):
            raise MergeError("Every profile entry must be an object.")
        name = entry.get("name")
        style = entry.get("point_style")
        if not isinstance(name, str) or not name.strip():
            raise MergeError("A profile entry is missing its name.")
        if not isinstance(style, dict) or not style:
            raise MergeError(f"Profile '{name}' has no point_style object.")
        unexpected = sorted(set(entry.keys()) - {"name", "point_style"})
        if unexpected:
            raise MergeError(
                f"Profile '{name}' carries unexpected keys: {unexpected}."
            )
        if name in seen_names:
            raise MergeError(f"Profile '{name}' appears twice in the input.")
        seen_names.add(name)
    return entries


def merge_profiles(
    project: dict,
    new_entries: list[dict],
    *,
    replace: bool = False,
) -> tuple[list[str], list[str]]:
    """Merge entries into project["point_visuals"] in place.

    Returns (appended_names, replaced_names).
    """
    library = project.get("point_visuals")
    if not isinstance(library, list):
        raise MergeError(
            "The project has no point_visuals library; refusing to guess "
            "the schema."
        )
    existing_by_name = {
        entry.get("name"): index
        for index, entry in enumerate(library)
        if isinstance(entry, dict)
    }
    appended: list[str] = []
    replaced: list[str] = []
    for entry in new_entries:
        name = entry["name"]
        if name in existing_by_name:
            if not replace:
                raise MergeError(
                    f"Profile '{name}' already exists in the project; the "
                    "improved copies are meant to be saved under new names "
                    "(pass --replace only to overwrite deliberately)."
                )
            library[existing_by_name[name]] = {
                "name": name,
                "point_style": entry["point_style"],
            }
            replaced.append(name)
        else:
            library.append(
                {"name": name, "point_style": entry["point_style"]}
            )
            appended.append(name)
    return appended, replaced


def write_project_atomically(project: dict, project_path: Path) -> None:
    directory = project_path.parent
    handle, temporary_name = tempfile.mkstemp(
        prefix=project_path.stem + ".merge-",
        suffix=".json",
        dir=directory,
    )
    try:
        with os.fdopen(handle, "w", encoding="utf-8") as output:
            json.dump(project, output, indent=2)
            output.write("\n")
        os.replace(temporary_name, project_path)
    except BaseException:
        try:
            os.unlink(temporary_name)
        except OSError:
            pass
        raise


def run(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--project",
        required=True,
        type=Path,
        help="The saved project to merge into (the artist's latest save).",
    )
    parser.add_argument(
        "--profiles",
        required=True,
        type=Path,
        help="JSON with the new profiles: a list of {name, point_style} "
        "entries, or an object holding one under 'point_visuals'.",
    )
    parser.add_argument(
        "--replace",
        action="store_true",
        help="Allow overwriting an existing profile of the same name.",
    )
    parser.add_argument(
        "--no-backup",
        action="store_true",
        help="Skip the verbatim pre-merge backup copy.",
    )
    parser.add_argument(
        "--force-while-running",
        action="store_true",
        help="Merge even though an app instance appears to be open (its "
        "next save would overwrite the merge).",
    )
    arguments = parser.parse_args(argv)

    try:
        if not arguments.force_while_running and invisible_places_running():
            raise MergeError(
                "An Invisible Places instance is running; its next save "
                "would overwrite this merge. Close the app first (or pass "
                "--force-while-running)."
            )
        new_entries = load_new_profiles(arguments.profiles)
        with open(arguments.project, "r", encoding="utf-8") as handle:
            project = json.load(handle)
        if arguments.no_backup:
            backup_path = None
        else:
            timestamp = _datetime.datetime.now(_datetime.timezone.utc)
            backup_path = arguments.project.with_name(
                arguments.project.stem
                + timestamp.strftime(".pre-profile-merge-%Y%m%d-%H%M%S")
                + arguments.project.suffix
            )
            shutil.copy2(arguments.project, backup_path)
        appended, replaced = merge_profiles(
            project,
            new_entries,
            replace=arguments.replace,
        )
        write_project_atomically(project, arguments.project)
    except (MergeError, OSError, json.JSONDecodeError) as error:
        print(f"merge_point_visual_profiles: {error}", file=sys.stderr)
        return 1
    if backup_path is not None:
        print(f"Backup: {backup_path}")
    if appended:
        print(f"Appended: {', '.join(appended)}")
    if replaced:
        print(f"Replaced: {', '.join(replaced)}")
    print(f"Merged into: {arguments.project}")
    return 0


if __name__ == "__main__":
    raise SystemExit(run())
