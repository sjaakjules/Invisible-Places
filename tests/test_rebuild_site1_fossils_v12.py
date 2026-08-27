import importlib.util
import json
import os
from pathlib import Path
from types import SimpleNamespace
import sys
import tempfile
import unittest
from unittest import mock


SCRIPT = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "rebuild_site1_fossils_v12.py"
)
SPEC = importlib.util.spec_from_file_location("rebuild_site1_fossils_v12", SCRIPT)
V12 = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = V12
SPEC.loader.exec_module(V12)


class Fixture:
    def __init__(self, root):
        self.root = Path(root).resolve()
        self.data = self.root / "Data" / "Scene1"
        self.patch = self.data / "PatchRefinement"
        self.run = self.patch / "v12"
        self.v9 = self.patch / "v9"
        self.v11 = self.patch / "v11"
        self.terrain_coarse = self.patch / "terrain-v12"
        self.data.mkdir(parents=True)
        self.run.mkdir(parents=True)
        self.v9.mkdir()
        self.v11.mkdir()
        self.terrain_coarse.mkdir()
        (self.v9 / "surface-v9.npz").write_bytes(b"surface")
        self.config = self.root / "v12.json"
        self.v10_config = self.root / "v10.json"
        self.normalization = self.root / "normalization.json"
        self.cleanmesh = self.root / "cleanmesh"
        self.downsample = self.root / "downsample"
        for path in (
            self.config,
            self.v10_config,
            self.normalization,
            self.cleanmesh,
            self.downsample,
        ):
            path.write_bytes(b"fixture")
        for name in (
            "Site1-WATER-2mm.ply",
            "Site1-WATER-5mm.ply",
            "Site1-WATER-5mm-old01.ply",
            "Site1-SAND-1mm.ply",
            "Site1-ROCK-1mm.ply",
            "Site1-SAND-5mm.ply",
            "Site1-ROCK-5mm.ply",
        ):
            (self.data / name).write_bytes(name.encode())
        self.args = SimpleNamespace(
            data_dir=self.data,
            run_dir=self.run,
            v9_run=self.v9,
            v11_run=self.v11,
            terrain_coarse_run=self.terrain_coarse,
            review_config=self.config,
            v10_config=self.v10_config,
            normalization_manifest=self.normalization,
            cleanmesh=self.cleanmesh,
            downsample=self.downsample,
            chunk_records=1_000_000,
            seed=120827,
            execute_cleanup=False,
        )
        paths = V12._coerce_paths(self.args)
        for required_paths in V12._rollback_evidence_path_groups(paths).values():
            for path in required_paths:
                path.parent.mkdir(parents=True, exist_ok=True)
                if not path.exists():
                    path.write_bytes(str(path).encode())

    def geometry_placeholders(self):
        directory = self.run / "water-geometry-2mm"
        directory.mkdir(exist_ok=True)
        for name in (
            "Site1-WATER-2mm.geometry-v12.candidate.ply",
            "additions.npz",
            "manifest.json",
        ):
            (directory / name).write_bytes(b"geometry")

    def fine_placeholders(self):
        directory = self.run / "water-final-2mm"
        directory.mkdir(exist_ok=True)
        for name in ("Site1-WATER-2mm.candidate.ply", "manifest.json"):
            (directory / name).write_bytes(b"fine")

    def audit_placeholder(self):
        directory = self.run / "interface-audit"
        directory.mkdir(exist_ok=True)
        (directory / "manifest.json").write_bytes(b"audit")


class DefaultsAndCommandTests(unittest.TestCase):
    def test_defaults_match_the_fine_first_scene1_run(self):
        args = V12.parser().parse_args(["finish"])
        self.assertEqual(args.run_dir, V12.DEFAULT_RUN)
        self.assertEqual(args.v9_run, V12.DEFAULT_V9_RUN)
        self.assertEqual(args.v11_run, V12.DEFAULT_V11_RUN)
        self.assertEqual(
            args.terrain_coarse_run, V12.DEFAULT_TERRAIN_COARSE_RUN
        )
        self.assertEqual(args.review_config, V12.DEFAULT_V12_CONFIG)
        self.assertEqual(args.normalization_manifest, V12.DEFAULT_NORMALIZATION)

    def test_native_downsample_is_exactly_005_without_priority_or_force(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            paths = V12._coerce_paths(fixture.args)
            command = V12.native_downsample_command(paths, chunk_records=765432)
            self.assertEqual(command[command.index("--spacing") + 1], "0.005")
            self.assertEqual(command[command.index("--chunk-points") + 1], "765432")
            self.assertNotIn("--priority-scan-id", command)
            self.assertNotIn("--force", command)
            self.assertEqual(command[command.index("--input") + 1], str(paths.fine_candidate))
            self.assertEqual(command[command.index("--output") + 1], str(paths.coarse_candidate))

    def test_stage_fingerprint_comparison_rejects_incomplete_declarations(self):
        complete = {
            "path": "/tmp/example.ply",
            "bytes": 123,
            "sha256": "abc",
            "points": 4,
            "record_stride": 16,
            "schema": [["x", "<f4"]],
        }
        self.assertTrue(V12._fingerprint_equal(complete, dict(complete)))
        self.assertFalse(V12._fingerprint_equal(complete, {}))
        for key in complete:
            with self.subTest(missing=key):
                incomplete = dict(complete)
                incomplete.pop(key)
                self.assertFalse(V12._fingerprint_equal(complete, incomplete))


class OrchestrationTests(unittest.TestCase):
    def test_finish_requires_existing_geometry(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            with (
                mock.patch.object(V12, "_validate_build_inputs"),
                self.assertRaisesRegex(RuntimeError, "finish requires"),
            ):
                V12.finish(fixture.args)

    def test_build_orders_fine_geometry_enrichment_downsample_and_release(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            events = []

            def geometry_builder(**kwargs):
                events.append("geometry-build")
                fixture.geometry_placeholders()

            def scalar_enricher(**kwargs):
                events.append("fine-enrich")
                fixture.fine_placeholders()

            def derive(paths, **kwargs):
                events.append("native-downsample")
                return {"verified": True, "coarse_scalar_recalculation": False}

            def interface_auditor(**kwargs):
                events.append("interface-audit-build")
                Path(kwargs["output_path"]).write_bytes(b"audit")

            def interface_audit_verifier(**kwargs):
                events.append("interface-audit-verify")
                return {"verified": True, "status": "passed"}

            def release_builder(args):
                events.append("release-build")
                (fixture.run / "release").mkdir()
                return {"built": True}

            def release_verifier(args):
                events.append("release-verify")
                return {"verified": True, "status": "built"}

            with (
                mock.patch.object(V12, "_validate_build_inputs"),
                mock.patch.object(V12, "verify_geometry_stage", side_effect=lambda *a, **k: events.append("geometry-verify") or {"verified": True}),
                mock.patch.object(V12, "verify_fine_stage", side_effect=lambda *a, **k: events.append("fine-verify") or {"verified": True}),
                mock.patch.object(V12, "derive_coarse_from_fine", side_effect=derive),
            ):
                result = V12.build(
                    fixture.args,
                    geometry_builder=geometry_builder,
                    scalar_enricher=scalar_enricher,
                    interface_auditor=interface_auditor,
                    interface_audit_verifier=interface_audit_verifier,
                    release_builder=release_builder,
                    release_verifier=release_verifier,
                )
            self.assertTrue(result["built"])
            self.assertTrue(result["candidate_only"])
            self.assertFalse(result["coarse_scalar_recalculation"])
            self.assertEqual(
                events,
                [
                    "geometry-build",
                    "geometry-verify",
                    "fine-enrich",
                    "fine-verify",
                    "interface-audit-build",
                    "interface-audit-verify",
                    "native-downsample",
                    "release-build",
                    "release-verify",
                ],
            )

    def test_verified_existing_stages_are_reused_without_overwrite(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            fixture.geometry_placeholders()
            fixture.fine_placeholders()
            fixture.audit_placeholder()
            geometry_builder = mock.Mock(side_effect=AssertionError("must not rebuild geometry"))
            scalar_enricher = mock.Mock(side_effect=AssertionError("must not rerun CleanMesh"))
            with (
                mock.patch.object(V12, "_validate_build_inputs"),
                mock.patch.object(V12, "verify_geometry_stage", return_value={"verified": True}) as geometry_verify,
                mock.patch.object(V12, "verify_fine_stage", return_value={"verified": True}) as fine_verify,
                mock.patch.object(V12, "verify_interface_audit_stage", return_value={"verified": True}) as audit_verify,
                mock.patch.object(V12, "derive_coarse_from_fine", return_value={"verified": True}),
            ):
                result = V12.build(
                    fixture.args,
                    geometry_builder=geometry_builder,
                    scalar_enricher=scalar_enricher,
                    release_builder=lambda args: {"built": True},
                    release_verifier=lambda args: {"verified": True, "status": "built"},
                )
            self.assertTrue(result["geometry_reused"])
            geometry_builder.assert_not_called()
            scalar_enricher.assert_not_called()
            self.assertGreaterEqual(geometry_verify.call_count, 1)
            fine_verify.assert_called_once()
            audit_verify.assert_called_once()

    def test_failed_interface_audit_stops_before_downsample_and_release(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            fixture.geometry_placeholders()
            fixture.fine_placeholders()
            downsample = mock.Mock(side_effect=AssertionError("must not downsample"))
            release = mock.Mock(side_effect=AssertionError("must not release"))

            def auditor(**kwargs):
                Path(kwargs["output_path"]).write_bytes(b"failed audit")

            with (
                mock.patch.object(V12, "_validate_build_inputs"),
                mock.patch.object(V12, "verify_geometry_stage", return_value={"verified": True}),
                mock.patch.object(V12, "verify_fine_stage", return_value={"verified": True}),
                mock.patch.object(V12, "derive_coarse_from_fine", side_effect=downsample),
                self.assertRaisesRegex(RuntimeError, "density continuity failed"),
            ):
                V12.build(
                    fixture.args,
                    interface_auditor=auditor,
                    interface_audit_verifier=lambda **kwargs: (_ for _ in ()).throw(
                        RuntimeError("density continuity failed")
                    ),
                    release_builder=release,
                    release_verifier=lambda args: {"verified": True, "status": "built"},
                )
            downsample.assert_not_called()
            release.assert_not_called()

    def test_existing_release_short_circuits_all_heavy_stages(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            (fixture.run / "release").mkdir()
            fixture.audit_placeholder()
            result = V12.build(
                fixture.args,
                geometry_builder=mock.Mock(side_effect=AssertionError),
                scalar_enricher=mock.Mock(side_effect=AssertionError),
                release_builder=mock.Mock(side_effect=AssertionError),
                release_verifier=lambda args: {"verified": True, "status": "built"},
                interface_audit_verifier=lambda **kwargs: {"verified": True},
            )
            self.assertTrue(result["reused_release"])

    def test_partial_stage_fails_closed(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            geometry = fixture.run / "water-geometry-2mm"
            geometry.mkdir()
            (geometry / "manifest.json").write_text("{}")
            with self.assertRaisesRegex(RuntimeError, "partial v12 geometry stage"):
                V12.build(fixture.args)


class CleanupTests(unittest.TestCase):
    def _installed(self, args):
        return {"verified": True, "status": "installed"}

    def _plan(self, fixture, *, water=None, terrain=None):
        return V12.cleanup_plan(
            fixture.args,
            release_verifier=water or self._installed,
            terrain_release_verifier=terrain or self._installed,
        )

    def _cleanup(self, fixture, *, water=None, terrain=None):
        return V12.cleanup(
            fixture.args,
            release_verifier=water or self._installed,
            terrain_release_verifier=terrain or self._installed,
        )

    def _planned_directory(self, fixture):
        target = fixture.run / "superseded-pre-measured-density"
        nested = target / "nested"
        nested.mkdir(parents=True)
        child = nested / "payload.bin"
        child.write_bytes(b"alpha")
        (target / "root.bin").write_bytes(b"root")
        plan = self._plan(fixture)
        entry = next(item for item in plan["targets"] if item["path"] == str(target))
        return target, child, plan, entry, V12._coerce_paths(fixture.args)

    def test_cleanup_plan_requires_verified_installed_release(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            with self.assertRaisesRegex(RuntimeError, "verified installed"):
                V12.cleanup_plan(
                    fixture.args,
                    release_verifier=lambda args: {"verified": True, "status": "built"},
                    terrain_release_verifier=self._installed,
                )

    def test_cleanup_plan_requires_verified_installed_terrain_coarse_release(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            with self.assertRaisesRegex(RuntimeError, "terrain-coarse"):
                V12.cleanup_plan(
                    fixture.args,
                    release_verifier=self._installed,
                    terrain_release_verifier=lambda args: {
                        "verified": True,
                        "status": "built",
                    },
                )

    def test_cleanup_is_disabled_without_explicit_execute_flag(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            with self.assertRaisesRegex(RuntimeError, "--execute-cleanup"):
                self._cleanup(fixture)

    def test_plan_retires_only_allowlisted_outcomes_and_preserves_rollback(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            v4 = fixture.patch / "20260826-noise-cleanup-v4"
            rollback = v4 / "Site1-ROCK-1mm.removed.ply"
            target = fixture.v11 / "water-base-2mm" / "Site1-WATER-2mm.candidate.ply"
            target.parent.mkdir(parents=True)
            target.write_bytes(b"large reproducible candidate")
            plan = self._plan(fixture)
            target_paths = {item["path"] for item in plan["targets"]}
            self.assertIn(str(target), target_paths)
            self.assertNotIn(str(rollback), target_paths)
            self.assertNotIn(str(fixture.data / "Site1-WATER-5mm-old01.ply"), target_paths)
            self.assertIn(str(v4), plan["protected"])
            self.assertTrue(plan["dry_run"])

    def test_plan_hashes_the_exact_compact_rollback_evidence_set(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            plan = self._plan(fixture)
            evidence = plan["rollback_evidence"]
            self.assertEqual(evidence["schema_version"], 1)
            self.assertTrue(evidence["complete"])
            self.assertEqual(evidence["required_file_count"], 46)
            self.assertEqual(
                set(evidence["categories"]),
                {
                    "v4_cull",
                    "v11_obstruction",
                    "v11_terrain",
                    "v9_water_reference",
                },
            )
            expected_groups = V12._rollback_evidence_path_groups(
                V12._coerce_paths(fixture.args)
            )
            for category, expected_paths in expected_groups.items():
                rows = evidence["categories"][category]
                self.assertEqual(
                    [row["path"] for row in rows],
                    [str(path.resolve()) for path in expected_paths],
                )
                for row in rows:
                    self.assertGreater(row["bytes"], 0)
                    self.assertRegex(row["sha256"], r"^[0-9a-f]{64}$")

    def test_missing_required_rollback_evidence_blocks_planning(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            paths = V12._coerce_paths(fixture.args)
            missing = V12._rollback_evidence_path_groups(paths)["v11_terrain"][4]
            missing.unlink()
            with self.assertRaisesRegex(RuntimeError, "rollback evidence"):
                self._plan(fixture)

    def test_uninstalled_terrain_gate_preserves_v11_release_snapshots(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            snapshot = fixture.v11 / "release" / "source-snapshots" / "snapshot.ply"
            transaction = fixture.v11 / "release" / "transactions" / "intent.json"
            snapshot.parent.mkdir(parents=True)
            transaction.parent.mkdir(parents=True)
            snapshot.write_bytes(b"snapshot")
            transaction.write_bytes(b"transaction")
            fixture.args.execute_cleanup = True
            with self.assertRaisesRegex(RuntimeError, "terrain-coarse"):
                self._cleanup(
                    fixture,
                    terrain=lambda args: {"verified": True, "status": "built"},
                )
            self.assertTrue(snapshot.exists())
            self.assertTrue(transaction.exists())

    def test_plan_signs_a_recursive_content_addressed_inventory(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            target, _, plan, entry, _ = self._planned_directory(fixture)
            inventory = entry["inventory"]
            self.assertEqual(plan["schema_version"], 3)
            self.assertEqual(entry["state"], "planned")
            self.assertFalse(entry["removed"])
            self.assertEqual(
                Path(entry["quarantine_path"]).parent,
                fixture.run / ".cleanup-quarantine",
            )
            self.assertEqual(
                [item["relative_path"] for item in inventory["entries"]],
                [".", "nested", "nested/payload.bin", "root.bin"],
            )
            self.assertEqual(inventory["directory_count"], 2)
            self.assertEqual(inventory["file_count"], 2)
            self.assertEqual(inventory["total_bytes"], len(b"alpha") + len(b"root"))
            for item in inventory["entries"]:
                self.assertIn("device", item)
                self.assertIn("inode", item)
                self.assertIn("size", item)
                self.assertIn("mtime_ns", item)
                if item["kind"] == "file":
                    self.assertRegex(item["sha256"], r"^[0-9a-f]{64}$")
            self.assertRegex(inventory["digest_sha256"], r"^[0-9a-f]{64}$")
            tampered = json.loads(json.dumps(plan))
            tampered_file = next(
                item
                for item in tampered["targets"][0]["inventory"]["entries"]
                if item["kind"] == "file"
            )
            tampered_file["sha256"] = "0" * 64
            self.assertNotEqual(V12._plan_signature(plan), V12._plan_signature(tampered))
            self.assertTrue(target.exists())

    def test_child_content_mutation_with_restored_size_and_mtime_blocks_delete(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            target, child, _, entry, paths = self._planned_directory(fixture)
            before = child.stat()
            child.write_bytes(b"omega")
            os.utime(child, ns=(before.st_atime_ns, before.st_mtime_ns))
            with self.assertRaisesRegex(RuntimeError, "recursive inventory drift"):
                V12._delete_cleanup_target(entry, paths)
            self.assertTrue(target.exists())

    def test_child_insertion_blocks_delete(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            target, _, _, entry, paths = self._planned_directory(fixture)
            (target / "inserted.bin").write_bytes(b"new")
            with self.assertRaisesRegex(RuntimeError, "recursive inventory drift"):
                V12._delete_cleanup_target(entry, paths)
            self.assertTrue(target.exists())

    def test_planned_target_disappearance_is_not_claimed_as_removal(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            target, _, _, entry, paths = self._planned_directory(fixture)
            V12.shutil.rmtree(target)
            with self.assertRaisesRegex(RuntimeError, "disappeared before quarantine"):
                V12._delete_cleanup_target(entry, paths)
            self.assertEqual(entry["state"], "planned")
            self.assertFalse(entry["removed"])

    def test_same_path_file_replacement_blocks_delete_even_when_metadata_is_restored(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            target, child, _, entry, paths = self._planned_directory(fixture)
            directory_before = target.stat()
            child_before = child.stat()
            replacement = fixture.root / "replacement.bin"
            replacement.write_bytes(child.read_bytes())
            os.chmod(replacement, child_before.st_mode)
            os.utime(
                replacement,
                ns=(child_before.st_atime_ns, child_before.st_mtime_ns),
            )
            os.replace(replacement, child)
            os.utime(
                target,
                ns=(directory_before.st_atime_ns, directory_before.st_mtime_ns),
            )
            with self.assertRaisesRegex(RuntimeError, "recursive inventory drift"):
                V12._delete_cleanup_target(entry, paths)
            self.assertTrue(target.exists())

    def test_inserted_child_symlink_blocks_delete_and_preserves_destination(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            target, _, _, entry, paths = self._planned_directory(fixture)
            outside = fixture.root / "outside.bin"
            outside.write_bytes(b"must survive")
            (target / "inserted-link").symlink_to(outside)
            with self.assertRaisesRegex(RuntimeError, "contains a symlink"):
                V12._delete_cleanup_target(entry, paths)
            self.assertTrue(target.exists())
            self.assertEqual(outside.read_bytes(), b"must survive")

    def test_special_file_in_cleanup_tree_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            target, _, _, entry, paths = self._planned_directory(fixture)
            fifo = target / "inserted-fifo"
            os.mkfifo(fifo)
            with self.assertRaisesRegex(RuntimeError, "non-regular file"):
                V12._delete_cleanup_target(entry, paths)
            self.assertTrue(target.exists())

    def test_clean_delete_is_idempotent_for_journal_retry(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            target, _, _, entry, paths = self._planned_directory(fixture)
            V12._delete_cleanup_target(entry, paths)
            self.assertFalse(target.exists())
            self.assertEqual(entry["state"], "removed")
            self.assertTrue(entry["removed"])
            V12._delete_cleanup_target(entry, paths)
            self.assertFalse(target.exists())

    def test_interrupted_quarantine_erase_resumes_from_signed_remainder(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            target, _, _, _, _ = self._planned_directory(fixture)
            fixture.args.execute_cleanup = True
            original_rmtree = V12.shutil.rmtree

            def interrupt_after_one_delete(path):
                quarantined = Path(path)
                self.assertIn(".cleanup-quarantine", quarantined.parts)
                self.assertFalse(target.exists())
                (quarantined / "root.bin").unlink()
                raise OSError("simulated interrupted recursive erase")

            with (
                mock.patch.object(V12.shutil, "rmtree", side_effect=interrupt_after_one_delete),
                self.assertRaisesRegex(OSError, "interrupted recursive erase"),
            ):
                self._cleanup(fixture)

            journal_path = fixture.run / "cleanup-journal.json"
            interrupted = json.loads(journal_path.read_text())
            item = next(
                row for row in interrupted["targets"] if row["path"] == str(target)
            )
            quarantine = Path(item["quarantine_path"])
            self.assertEqual(item["state"], "erasing")
            self.assertFalse(item["removed"])
            self.assertFalse(target.exists())
            self.assertTrue(quarantine.exists())
            self.assertFalse((quarantine / "root.bin").exists())
            self.assertTrue((quarantine / "nested" / "payload.bin").exists())

            with mock.patch.object(V12.shutil, "rmtree", side_effect=original_rmtree):
                completed = self._cleanup(fixture)
            completed_item = next(
                row for row in completed["targets"] if row["path"] == str(target)
            )
            self.assertEqual(completed["state"], "complete")
            self.assertEqual(completed_item["state"], "removed")
            self.assertTrue(completed_item["removed"])
            self.assertFalse(quarantine.exists())

    def test_unknown_content_in_partial_quarantine_blocks_retry(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            target, _, _, _, _ = self._planned_directory(fixture)
            fixture.args.execute_cleanup = True

            def interrupt_after_one_delete(path):
                quarantined = Path(path)
                (quarantined / "root.bin").unlink()
                raise OSError("simulated interrupted recursive erase")

            with (
                mock.patch.object(V12.shutil, "rmtree", side_effect=interrupt_after_one_delete),
                self.assertRaisesRegex(OSError, "interrupted recursive erase"),
            ):
                self._cleanup(fixture)
            journal = json.loads((fixture.run / "cleanup-journal.json").read_text())
            item = next(row for row in journal["targets"] if row["path"] == str(target))
            quarantine = Path(item["quarantine_path"])
            (quarantine / "inserted.bin").write_bytes(b"not in signed inventory")
            with self.assertRaisesRegex(RuntimeError, "unknown content"):
                self._cleanup(fixture)
            self.assertFalse(target.exists())
            self.assertTrue(quarantine.exists())

    def test_rollback_evidence_mutation_blocks_quarantine_retry(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            self._planned_directory(fixture)
            fixture.args.execute_cleanup = True

            with (
                mock.patch.object(
                    V12.shutil,
                    "rmtree",
                    side_effect=OSError("simulated interrupted recursive erase"),
                ),
                self.assertRaisesRegex(OSError, "interrupted recursive erase"),
            ):
                self._cleanup(fixture)
            paths = V12._coerce_paths(fixture.args)
            evidence = V12._rollback_evidence_path_groups(paths)["v4_cull"][1]
            evidence.write_bytes(b"tampered rollback evidence")
            with self.assertRaisesRegex(RuntimeError, "rollback evidence differs"):
                self._cleanup(fixture)

    def test_cleanup_lock_rejects_a_concurrent_cleanup(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            paths = V12._coerce_paths(fixture.args)
            with V12.cleanup_lock(paths):
                with self.assertRaisesRegex(RuntimeError, "another Scene1 v12 cleanup"):
                    with V12.cleanup_lock(paths):
                        self.fail("nested cleanup lock unexpectedly acquired")

    def test_cleanup_journal_removes_exact_target_and_retains_old01_and_v4(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            v4 = fixture.patch / "20260826-noise-cleanup-v4"
            rollback = v4 / "Site1-SAND-1mm.removed.ply"
            target = fixture.v11 / "water-geometry-2mm" / "Site1-WATER-2mm.geometry-v11.candidate.ply"
            target.parent.mkdir(parents=True)
            target.write_bytes(b"candidate")
            fixture.args.execute_cleanup = True
            result = self._cleanup(fixture)
            self.assertEqual(result["state"], "complete")
            self.assertFalse(target.exists())
            self.assertTrue(rollback.exists())
            self.assertTrue((fixture.data / "Site1-WATER-5mm-old01.ply").exists())
            journal = json.loads((fixture.run / "cleanup-journal.json").read_text())
            self.assertEqual(journal["state"], "complete")
            self.assertTrue(all(item["removed"] for item in journal["targets"]))
            retry = self._cleanup(fixture)
            self.assertEqual(retry["state"], "complete")

    def test_symlink_cleanup_target_fails_closed(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            outside = fixture.root / "outside.bin"
            outside.write_bytes(b"must survive")
            target = fixture.v11 / "water-base-5mm" / "Site1-WATER-5mm.candidate.ply"
            target.parent.mkdir(parents=True)
            target.symlink_to(outside)
            with self.assertRaisesRegex(RuntimeError, "may not be a symlink"):
                self._plan(fixture)
            self.assertTrue(outside.exists())


if __name__ == "__main__":
    unittest.main()
