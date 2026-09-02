from __future__ import annotations

import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

from scripts import merge_point_visual_profiles as merger


def make_project(names: list[str]) -> dict:
    return {
        "schema_version": 92,
        "selected_point_visual": names[0] if names else "",
        "point_visuals": [
            {"name": name, "point_style": {"falloff_profile": "gaussian"}}
            for name in names
        ],
        "layers": [{"path": "untouched.ply"}],
    }


def make_profiles(names: list[str]) -> list[dict]:
    return [
        {
            "name": name,
            "point_style": {
                "falloff_profile": "gaussian",
                "gaussian_sharpness": 2.0,
            },
        }
        for name in names
    ]


class MergeProfilesTests(unittest.TestCase):
    def test_appends_new_profiles_and_preserves_other_state(self) -> None:
        project = make_project(["Surface_05_Base"])
        appended, replaced = merger.merge_profiles(
            project,
            make_profiles(["Surface_05_Base_v2", "Surface_05_Thin_v2"]),
        )
        self.assertEqual(
            appended,
            ["Surface_05_Base_v2", "Surface_05_Thin_v2"],
        )
        self.assertEqual(replaced, [])
        self.assertEqual(
            [entry["name"] for entry in project["point_visuals"]],
            ["Surface_05_Base", "Surface_05_Base_v2", "Surface_05_Thin_v2"],
        )
        # Authored state outside the library must be untouched.
        self.assertEqual(project["selected_point_visual"], "Surface_05_Base")
        self.assertEqual(project["layers"], [{"path": "untouched.ply"}])

    def test_refuses_to_overwrite_existing_name_without_replace(self) -> None:
        project = make_project(["Surface_05_Base"])
        with self.assertRaises(merger.MergeError):
            merger.merge_profiles(project, make_profiles(["Surface_05_Base"]))
        # The failed merge must not have modified the library.
        self.assertEqual(len(project["point_visuals"]), 1)
        self.assertNotIn(
            "gaussian_sharpness",
            project["point_visuals"][0]["point_style"],
        )

    def test_replace_overwrites_in_place_keeping_order(self) -> None:
        project = make_project(["A", "Surface_05_Base", "B"])
        appended, replaced = merger.merge_profiles(
            project,
            make_profiles(["Surface_05_Base"]),
            replace=True,
        )
        self.assertEqual(appended, [])
        self.assertEqual(replaced, ["Surface_05_Base"])
        self.assertEqual(
            [entry["name"] for entry in project["point_visuals"]],
            ["A", "Surface_05_Base", "B"],
        )
        self.assertEqual(
            project["point_visuals"][1]["point_style"]["gaussian_sharpness"],
            2.0,
        )

    def test_rejects_malformed_profiles(self) -> None:
        for entries in (
            [],
            [{"point_style": {"a": 1}}],
            [{"name": "  ", "point_style": {"a": 1}}],
            [{"name": "X", "point_style": {}}],
            [{"name": "X", "point_style": {"a": 1}, "extra": True}],
            make_profiles(["Dup", "Dup"]),
        ):
            with tempfile.TemporaryDirectory() as scratch:
                profiles_path = Path(scratch) / "profiles.json"
                profiles_path.write_text(json.dumps(entries))
                with self.assertRaises(merger.MergeError, msg=str(entries)):
                    merger.load_new_profiles(profiles_path)

    def test_accepts_wrapped_point_visuals_object(self) -> None:
        with tempfile.TemporaryDirectory() as scratch:
            profiles_path = Path(scratch) / "profiles.json"
            profiles_path.write_text(
                json.dumps({"point_visuals": make_profiles(["X"])})
            )
            entries = merger.load_new_profiles(profiles_path)
            self.assertEqual(entries[0]["name"], "X")

    def test_run_merges_end_to_end_with_backup(self) -> None:
        with tempfile.TemporaryDirectory() as scratch:
            project_path = Path(scratch) / "proj.json"
            profiles_path = Path(scratch) / "profiles.json"
            project_path.write_text(
                json.dumps(make_project(["Surface_05_Base"]))
            )
            profiles_path.write_text(
                json.dumps(make_profiles(["Surface_05_Base_v2"]))
            )
            with mock.patch.object(
                merger,
                "invisible_places_running",
                return_value=False,
            ):
                exit_code = merger.run(
                    [
                        "--project",
                        str(project_path),
                        "--profiles",
                        str(profiles_path),
                    ]
                )
            self.assertEqual(exit_code, 0)
            merged = json.loads(project_path.read_text())
            self.assertEqual(
                [entry["name"] for entry in merged["point_visuals"]],
                ["Surface_05_Base", "Surface_05_Base_v2"],
            )
            backups = sorted(
                Path(scratch).glob("proj.pre-profile-merge-*.json")
            )
            self.assertEqual(len(backups), 1)
            original = json.loads(backups[0].read_text())
            self.assertEqual(len(original["point_visuals"]), 1)

    def test_run_refuses_while_app_is_running(self) -> None:
        with tempfile.TemporaryDirectory() as scratch:
            project_path = Path(scratch) / "proj.json"
            profiles_path = Path(scratch) / "profiles.json"
            original = json.dumps(make_project(["Surface_05_Base"]))
            project_path.write_text(original)
            profiles_path.write_text(json.dumps(make_profiles(["V2"])))
            with mock.patch.object(
                merger,
                "invisible_places_running",
                return_value=True,
            ):
                exit_code = merger.run(
                    [
                        "--project",
                        str(project_path),
                        "--profiles",
                        str(profiles_path),
                    ]
                )
            self.assertEqual(exit_code, 1)
            self.assertEqual(project_path.read_text(), original)
            self.assertEqual(
                list(Path(scratch).glob("proj.pre-profile-merge-*.json")),
                [],
            )


if __name__ == "__main__":
    unittest.main()
