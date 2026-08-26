import importlib.util
import contextlib
import io
import json
from pathlib import Path
from types import SimpleNamespace
import sys
import tempfile
import unittest
from unittest import mock

import numpy as np


SCRIPT = Path(__file__).resolve().parents[1] / "scripts/site1_scalar_fill_bundle.py"
sys.path.insert(0, str(SCRIPT.parent))
SPEC = importlib.util.spec_from_file_location("site1_scalar_fill_bundle", SCRIPT)
BUNDLE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = BUNDLE
SPEC.loader.exec_module(BUNDLE)
SCALAR = BUNDLE.scalar


def synthetic_dtype():
    fields = [
        ("x", "<f4"),
        ("y", "<f4"),
        ("z", "<f4"),
        ("nx", "<f4"),
        ("ny", "<f4"),
        ("nz", "<f4"),
        ("red", "u1"),
        ("green", "u1"),
        ("blue", "u1"),
        ("scalar_Intensity", "<f8"),
        ("scalar_Composite", "<f4"),
        ("scalar_ScanID", "<f4"),
    ]
    fields.extend((name, "<f4") for name in SCALAR.REPAIRABLE_FIELDS)
    return np.dtype(fields)


def populated_records():
    records = np.zeros(4, dtype=synthetic_dtype())
    records["x"] = [0.0, 0.02, 0.01, 0.50]
    records["y"] = 0.0
    records["z"] = 2.0
    records["red"] = 80
    records["green"] = 90
    records["blue"] = 100
    records["scalar_Intensity"] = 50_000.0
    records["scalar_Composite"] = 100.0
    records["scalar_ScanID"] = 7.0
    records["nz"] = 1.0
    for scale, multiplier in (("Fine", 1.0), ("Medium", 2.0), ("Broad", 3.0)):
        records[f"scalar_A_R_MeanCurvature_{scale}"] = 0.2 * multiplier
        records[f"scalar_A_R_CrossCurvature_{scale}"] = -0.1 * multiplier
        records[f"scalar_A_R_Recession_{scale}"] = 0.0002 * multiplier
        records[f"scalar_A_R_Roughness_{scale}"] = 0.002 * multiplier
    records["scalar_A_R_MeanCurvature_Combined"] = 0.2
    records["scalar_A_R_CrossCurvature_Combined"] = -0.1
    records["scalar_A_R_Recession_Combined"] = 0.1
    records["scalar_A_R_Roughness_Combined"] = 0.5
    records["scalar_A_R_RoughnessRelative_FineMedium"] = 0.5
    records["scalar_A_R_Shelter_Lower"] = 0.6
    records["scalar_A_R_RainExposure_Lower"] = 0.4
    records["scalar_A_R_SVF_Lower"] = 0.7
    records["scalar_A_R_Downhill_X"] = 0.2
    records["scalar_A_R_Downhill_Y"] = -0.3
    records["scalar_A_R_Downhill_Z"] = 0.0
    records["scalar_A_R_DownhillMagnitude"] = 0.4
    records["scalar_A_R_Horizontalness"] = 0.9
    records["scalar_A_R_Slope_deg"] = 20.0
    for fields in SCALAR.COMPONENT_GROUPS.values():
        if fields == SCALAR.COMPONENT_GROUPS["LowerContext"]:
            continue
        for field in fields:
            records[field][2] = np.nan
    for field in SCALAR.DERIVED_FIELDS:
        records[field][2] = np.nan
    return records


def write_ply(path, records):
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "wb") as handle:
        SCALAR.v6.write_ply_header(
            handle, records.dtype, len(records), ["synthetic bundle source"]
        )
        records.tofile(handle)


def mutate_field(path, field, index, value):
    info = SCALAR.inspect_fixed_stride_ply(path)
    records = np.memmap(
        path,
        dtype=info.dtype,
        mode="r+",
        offset=info.offset,
        shape=(info.count,),
    )
    records[field][index] = value
    records.flush()
    del records


class BundleFixture(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        root = Path(self.temporary.name)
        self.data_dir = root / "Scene1"
        self.run_dir = root / "v9/terrain-scalars"
        for role in BUNDLE.ROLES:
            for spacing in BUNDLE.SPACINGS:
                write_ply(
                    BUNDLE.cloud_path(self.data_dir, role, spacing),
                    populated_records(),
                )
        self.args = SimpleNamespace(
            data_dir=self.data_dir,
            run_dir=self.run_dir,
            overwrite=False,
            chunk_size=2,
            repair_options=SCALAR.RepairOptions(
                chunk_size=2,
                max_donors_per_group=100,
                donor_query_k=3,
                distance_buckets_m=(0.025, 0.05, 0.1, 0.2, 0.4, 0.8, 1.6),
                xy_fallback_buckets_m=(
                    0.025,
                    0.05,
                    0.1,
                    0.2,
                    0.4,
                    0.8,
                    1.6,
                    3.2,
                ),
                workers=1,
                prefer_clone=False,
            ),
        )

    def tearDown(self):
        self.temporary.cleanup()

    def build(self):
        return BUNDLE.build(self.args)


class BundleTransactionTests(BundleFixture):
    def test_build_verify_install_and_restore_six_file_bundle(self):
        original_hashes = {
            key: BUNDLE.sha256_path(
                BUNDLE.cloud_path(self.data_dir, *key.split("-"))
            )
            for key in BUNDLE.BUNDLE_KEYS
        }
        manifest = self.build()

        self.assertEqual(tuple(manifest["order"]), BUNDLE.BUNDLE_KEYS)
        self.assertEqual(
            set(manifest["derived_normalization_from_rock_by_spacing"]),
            set(BUNDLE.SPACINGS),
        )
        for key, entry in manifest["entries"].items():
            self.assertEqual(entry["source"]["sha256"], original_hashes[key])
            self.assertTrue(Path(entry["candidate"]).exists())
            self.assertFalse(entry["remaining_nonfinite"])
            self.assertIn(
                "le_800mm",
                entry["filled_rows_by_distance_bucket"]["Fine"],
            )
            self.assertIn(
                "le_1600mm",
                entry["filled_rows_by_distance_bucket"]["Fine"],
            )
            self.assertIn(
                "le_3200mm",
                entry["xy_fallback_rows_by_distance_bucket"]["Fine"],
            )
            self.assertEqual(
                entry["direct_from_normals"]["Directional"]["filled_rows"],
                1,
            )
            repair = json.loads(Path(entry["repair_report"]).read_text())
            self.assertEqual(repair["schema_version"], 3)
            self.assertEqual(
                repair["derived"]["combination"]["algorithm"],
                SCALAR.DERIVED_COMBINATION_ALGORITHM,
            )
            self.assertEqual(
                repair["derived"]["combination"]["normalization"]["mode"],
                "provided",
            )
            if entry["spacing"] == "1mm":
                expected = BUNDLE.candidate_path(
                    self.run_dir, entry["role"], "5mm"
                ).resolve()
                self.assertEqual(Path(repair["fallback_5mm"]), expected)

        verification = BUNDLE.verify(self.args)
        self.assertTrue(verification["verified"])
        self.assertTrue(verification["source_hashes_unchanged"])
        self.assertEqual(verification["remaining_nonfinite_total"], 0)
        self.assertIn(
            "le_1600mm", verification["filled_group_rows_by_distance_bucket"]
        )
        self.assertIn(
            "le_3200mm",
            verification["xy_fallback_group_rows_by_distance_bucket"],
        )
        self.assertEqual(verification["direct_from_normals_rows"], 6)

        candidate_hashes = {
            key: entry["candidate_sha256"]
            for key, entry in manifest["entries"].items()
        }
        with mock.patch.object(BUNDLE, "app_running", return_value=False):
            installed = BUNDLE.install(self.args)
        self.assertTrue(installed["installed"])
        for key in BUNDLE.BUNDLE_KEYS:
            role, spacing = key.split("-")
            canonical = BUNDLE.cloud_path(self.data_dir, role, spacing)
            candidate = BUNDLE.candidate_path(self.run_dir, role, spacing)
            backup = BUNDLE.backup_path(self.run_dir, role, spacing)
            self.assertFalse(candidate.exists())
            self.assertEqual(BUNDLE.sha256_path(canonical), candidate_hashes[key])
            self.assertEqual(BUNDLE.sha256_path(backup), original_hashes[key])

        with mock.patch.object(BUNDLE, "app_running", return_value=False):
            restored = BUNDLE.restore(self.args)
        self.assertFalse(restored["installed"])
        for key in BUNDLE.BUNDLE_KEYS:
            role, spacing = key.split("-")
            canonical = BUNDLE.cloud_path(self.data_dir, role, spacing)
            candidate = BUNDLE.candidate_path(self.run_dir, role, spacing)
            backup = BUNDLE.backup_path(self.run_dir, role, spacing)
            self.assertEqual(BUNDLE.sha256_path(canonical), original_hashes[key])
            self.assertEqual(BUNDLE.sha256_path(candidate), candidate_hashes[key])
            self.assertFalse(backup.exists())

    def test_install_refuses_source_hash_drift_before_any_move(self):
        manifest = self.build()
        first = BUNDLE.BUNDLE_KEYS[0]
        source = Path(manifest["entries"][first]["source"]["path"])
        mutate_field(source, "red", 0, 201)

        with mock.patch.object(BUNDLE, "app_running", return_value=False):
            with self.assertRaisesRegex(RuntimeError, "source hashes drifted"):
                BUNDLE.install(self.args)

        for key, entry in manifest["entries"].items():
            role, spacing = key.split("-")
            self.assertTrue(Path(entry["candidate"]).exists())
            self.assertFalse(BUNDLE.backup_path(self.run_dir, role, spacing).exists())

    def test_install_refuses_any_remaining_nonfinite_value(self):
        manifest = self.build()
        key = BUNDLE.BUNDLE_KEYS[0]
        candidate = Path(manifest["entries"][key]["candidate"])
        mutate_field(candidate, "scalar_A_R_MeanCurvature_Fine", 0, np.nan)
        manifest["entries"][key]["candidate_sha256"] = BUNDLE.sha256_path(candidate)
        BUNDLE._atomic_write_json(self.run_dir / BUNDLE.MANIFEST_NAME, manifest)

        with mock.patch.object(BUNDLE, "app_running", return_value=False):
            with self.assertRaisesRegex(RuntimeError, "nonzero remaining"):
                BUNDLE.install(self.args)

        for entry in manifest["entries"].values():
            self.assertTrue(Path(entry["candidate"]).exists())

    def test_install_rolls_back_all_prior_moves_after_mid_bundle_failure(self):
        manifest = self.build()
        original_hashes = {
            key: entry["source"]["sha256"]
            for key, entry in manifest["entries"].items()
        }
        candidate_hashes = {
            key: entry["candidate_sha256"]
            for key, entry in manifest["entries"].items()
        }
        real_replace = BUNDLE._replace_file
        calls = {"count": 0}

        def fail_once(source, destination):
            calls["count"] += 1
            if calls["count"] == 6:
                raise OSError("injected third-candidate rename failure")
            return real_replace(source, destination)

        with mock.patch.object(BUNDLE, "app_running", return_value=False), \
                mock.patch.object(BUNDLE, "_replace_file", side_effect=fail_once):
            with self.assertRaisesRegex(OSError, "injected"):
                BUNDLE.install(self.args)

        current_manifest = json.loads(
            (self.run_dir / BUNDLE.MANIFEST_NAME).read_text()
        )
        self.assertFalse(current_manifest["installed"])
        for key in BUNDLE.BUNDLE_KEYS:
            role, spacing = key.split("-")
            canonical = BUNDLE.cloud_path(self.data_dir, role, spacing)
            candidate = BUNDLE.candidate_path(self.run_dir, role, spacing)
            backup = BUNDLE.backup_path(self.run_dir, role, spacing)
            self.assertEqual(BUNDLE.sha256_path(canonical), original_hashes[key])
            self.assertEqual(BUNDLE.sha256_path(candidate), candidate_hashes[key])
            self.assertFalse(backup.exists())

    def test_running_application_refuses_before_verification_or_move(self):
        manifest = self.build()
        with mock.patch.object(BUNDLE, "app_running", return_value=True), \
                mock.patch.object(BUNDLE, "_verify_unlocked") as verify_mock:
            with self.assertRaisesRegex(RuntimeError, "is running"):
                BUNDLE.install(self.args)
        verify_mock.assert_not_called()
        for entry in manifest["entries"].values():
            self.assertTrue(Path(entry["candidate"]).exists())


class BundleResumeTests(BundleFixture):
    def _resume_args(self):
        values = dict(vars(self.args))
        values["resume"] = True
        values["overwrite"] = False
        return SimpleNamespace(**values)

    def _remove_manifest(self):
        (self.run_dir / BUNDLE.MANIFEST_NAME).unlink()

    def test_resume_admits_legacy_verified_5mm_reports_and_builds_only_1mm(self):
        self.build()
        self._remove_manifest()
        for role in BUNDLE.ROLES:
            report_path = BUNDLE.repair_report_path(self.run_dir, role, "5mm")
            report = json.loads(report_path.read_text())
            report.pop("bundle_resume")
            BUNDLE._atomic_write_json(report_path, report)
            BUNDLE.candidate_path(self.run_dir, role, "1mm").unlink()
            BUNDLE.repair_report_path(self.run_dir, role, "1mm").unlink()

        partial = (
            self.run_dir
            / "candidates/.Site1-SAND-1mm.scalar-candidate.ply.dead.partial"
        )
        partial.write_bytes(b"never reuse this")
        with mock.patch.object(
            SCALAR,
            "repair_scalar_file",
            wraps=SCALAR.repair_scalar_file,
        ) as repair_mock:
            manifest = BUNDLE.build(self._resume_args())

        self.assertEqual(repair_mock.call_count, 3)
        self.assertEqual(
            manifest["resume"]["reused_keys"],
            ["SAND-5mm", "ROCK-5mm", "VEG-5mm"],
        )
        self.assertEqual(
            manifest["resume"][
                "legacy_reports_admitted_after_full_verification"
            ],
            ["SAND-5mm", "ROCK-5mm", "VEG-5mm"],
        )
        self.assertEqual(
            manifest["resume"]["ignored_orphan_partial_files"],
            [str(partial.resolve())],
        )
        self.assertTrue(partial.exists())
        for role in BUNDLE.ROLES:
            report = json.loads(
                BUNDLE.repair_report_path(self.run_dir, role, "5mm").read_text()
            )
            metadata = report["bundle_resume"]
            self.assertTrue(metadata["full_verification"])
            self.assertTrue(metadata["legacy_report_admitted"])

    def test_resume_rebuilds_candidate_when_hash_or_report_config_mismatches(self):
        manifest = self.build()
        self._remove_manifest()
        sand_candidate = BUNDLE.candidate_path(self.run_dir, "SAND", "5mm")
        mutate_field(sand_candidate, "red", 0, 211)
        rock_report_path = BUNDLE.repair_report_path(self.run_dir, "ROCK", "5mm")
        rock_report = json.loads(rock_report_path.read_text())
        rock_report["distance_buckets_mm"] = [25.0, 100.0]
        BUNDLE._atomic_write_json(rock_report_path, rock_report)

        with mock.patch.object(
            SCALAR,
            "repair_scalar_file",
            wraps=SCALAR.repair_scalar_file,
        ) as repair_mock:
            resumed = BUNDLE.build(self._resume_args())

        self.assertEqual(repair_mock.call_count, 2)
        self.assertEqual(
            resumed["resume"]["rebuilt_keys"],
            ["SAND-5mm", "ROCK-5mm"],
        )
        self.assertIn("candidate_sha256", resumed["resume"]["rejections"]["SAND-5mm"])
        self.assertIn(
            "distance_buckets_mm",
            resumed["resume"]["rejections"]["ROCK-5mm"],
        )
        self.assertEqual(
            BUNDLE.sha256_path(sand_candidate),
            manifest["entries"]["SAND-5mm"]["candidate_sha256"],
        )

    def test_resume_rejects_legacy_report_without_cleanmesh_semantics(self):
        self.build()
        self._remove_manifest()
        report_path = BUNDLE.repair_report_path(self.run_dir, "SAND", "5mm")
        report = json.loads(report_path.read_text())
        report.pop("bundle_resume")
        report["derived"].pop("combination")
        BUNDLE._atomic_write_json(report_path, report)

        with mock.patch.object(
            SCALAR,
            "repair_scalar_file",
            wraps=SCALAR.repair_scalar_file,
        ) as repair_mock:
            resumed = BUNDLE.build(self._resume_args())

        self.assertEqual(repair_mock.call_count, 1)
        self.assertIn("SAND-5mm", resumed["resume"]["rebuilt_keys"])
        self.assertIn(
            "derived.combination.algorithm",
            resumed["resume"]["rejections"]["SAND-5mm"],
        )

    def test_resume_reclaims_only_a_dead_pid_lock(self):
        self.run_dir.mkdir(parents=True, exist_ok=True)
        lock = self.run_dir / BUNDLE.LOCK_NAME
        lock.write_text("pid=999999 stage=build created=earlier\n")
        with mock.patch.object(BUNDLE, "_pid_is_running", return_value=False):
            manifest = BUNDLE.build(self._resume_args())
        self.assertFalse(lock.exists())
        self.assertEqual(len(manifest["entries"]), 6)

        (self.run_dir / BUNDLE.MANIFEST_NAME).unlink()
        lock.write_text("pid=999999 stage=build created=now\n")
        with mock.patch.object(BUNDLE, "_pid_is_running", return_value=True):
            with self.assertRaisesRegex(RuntimeError, "scalar bundle is locked"):
                BUNDLE.build(self._resume_args())
        self.assertTrue(lock.exists())

    def test_resume_refuses_manifested_source_hash_drift(self):
        self.build()
        source = BUNDLE.cloud_path(self.data_dir, "SAND", "5mm")
        mutate_field(source, "red", 0, 212)
        with self.assertRaisesRegex(RuntimeError, "source hash drift"):
            BUNDLE.build(self._resume_args())


class ArgumentTests(unittest.TestCase):
    def test_extended_distance_buckets_are_default_and_validated(self):
        args = BUNDLE._parser().parse_args(["verify"])
        self.assertEqual(
            args.distance_buckets_m,
            tuple(value / 1000.0 for value in BUNDLE.BUNDLE_DISTANCE_BUCKETS_MM),
        )
        self.assertEqual(
            args.xy_fallback_buckets_m,
            tuple(
                value / 1000.0
                for value in BUNDLE.BUNDLE_XY_FALLBACK_BUCKETS_MM
            ),
        )
        custom = BUNDLE._parser().parse_args(
            [
                "verify",
                "--distance-buckets-mm",
                "25,100,800",
                "--xy-fallback-buckets-mm",
                "1000,5000,200000",
            ]
        )
        self.assertEqual(custom.distance_buckets_m, (0.025, 0.1, 0.8))
        self.assertEqual(custom.xy_fallback_buckets_m, (1.0, 5.0, 200.0))
        with contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):
                BUNDLE._parser().parse_args(
                    ["verify", "--distance-buckets-mm", "25,25,100"]
                )
            with self.assertRaises(SystemExit):
                BUNDLE._parser().parse_args(
                    ["build", "--resume", "--overwrite"]
                )


if __name__ == "__main__":
    unittest.main()
