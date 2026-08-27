import importlib.util
import json
import os
from pathlib import Path
import shutil
from types import SimpleNamespace
import sys
import tempfile
import unittest
from unittest import mock

import numpy as np


SCRIPT = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "rebuild_site1_fossils_v11.py"
)
SPEC = importlib.util.spec_from_file_location("rebuild_site1_fossils_v11", SCRIPT)
V11 = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = V11
SPEC.loader.exec_module(V11)


DTYPE = np.dtype(
    [
        ("x", "<f4"),
        ("y", "<f4"),
        ("z", "<f4"),
        ("nx", "<f4"),
        ("ny", "<f4"),
        ("nz", "<f4"),
        ("red", "u1"),
        ("scalar_ScanID", "<f4"),
        ("scalar_A_R_Roughness_Fine", "<f4"),
        ("scalar_A_R_Roughness_Combined", "<f4"),
    ]
)


def records(count, *, start=0.0, scan_id=1.0):
    result = np.zeros(count, DTYPE)
    result["x"] = start + np.arange(count) * 0.01
    result["y"] = 2.0 + np.arange(count) * 0.02
    result["z"] = 3.0 + np.arange(count) * 0.001
    result["nx"] = 0.1
    result["ny"] = 0.2
    result["nz"] = 0.9746794
    result["red"] = np.arange(count) + 10
    result["scalar_ScanID"] = scan_id
    result["scalar_A_R_Roughness_Fine"] = 0.10 + np.arange(count) * 0.01
    result["scalar_A_R_Roughness_Combined"] = 0.20 + np.arange(count) * 0.01
    return result


def write_ply(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    type_name = {
        "<f4": "float",
        "|u1": "uchar",
    }
    lines = [
        "ply",
        "format binary_little_endian 1.0",
        f"element vertex {len(value)}",
    ]
    for name in value.dtype.names:
        lines.append(f"property {type_name[value.dtype.fields[name][0].str]} {name}")
    lines.append("end_header")
    with path.open("wb") as handle:
        handle.write(("\n".join(lines) + "\n").encode("ascii"))
        handle.write(value.tobytes())
    return path


def write_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n")
    return path


def ordinary_fingerprint(path):
    path = Path(path).resolve(strict=True)
    stat = path.stat()
    return {
        "path": str(path),
        "size_bytes": stat.st_size,
        "mtime_ns": stat.st_mtime_ns,
        "sha256": V11.sha256_path(path),
    }


def ply_fingerprint(path):
    result = ordinary_fingerprint(path)
    info = V11.inspect_ply(path)
    result.update(points=info.count, record_stride=info.stride)
    return result


class ExactPayloadTests(unittest.TestCase):
    def test_float32_footprint_interval_allows_one_ulp_only(self):
        lower, upper = V11._outward_stored_interval(
            772.93,
            773.25,
            np.dtype("<f4"),
        )
        rounded_lower = np.float32(772.93)
        one_ulp_below = np.nextafter(rounded_lower, np.float32(-np.inf))
        two_ulps_below = np.nextafter(one_ulp_below, np.float32(-np.inf))
        rounded_upper = np.float32(773.25)
        one_ulp_above = np.nextafter(rounded_upper, np.float32(np.inf))
        two_ulps_above = np.nextafter(one_ulp_above, np.float32(np.inf))
        self.assertGreaterEqual(float(one_ulp_below), lower)
        self.assertLessEqual(float(one_ulp_above), upper)
        self.assertLess(float(two_ulps_below), lower)
        self.assertGreater(float(two_ulps_above), upper)

    def test_app_process_probe_fails_closed_when_process_listing_is_unavailable(self):
        completed = SimpleNamespace(
            returncode=3,
            stdout="",
            stderr="process listing unavailable",
        )
        with (
            mock.patch.object(V11.subprocess, "run", return_value=completed),
            self.assertRaisesRegex(RuntimeError, "unable to determine"),
        ):
            V11.app_running()

    def test_append_prefix_and_scan_id_are_verified(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source_records = records(5)
            additions = records(2, start=8.0, scan_id=10.0)
            source = write_ply(root / "source.ply", source_records)
            candidate = write_ply(
                root / "candidate.ply", np.concatenate((source_records, additions))
            )
            report = V11.verify_exact_append_prefix(source, candidate)
            self.assertTrue(report["verified"])
            self.assertEqual(report["addition_points"], 2)
            scan = V11._scan_appended_scan_id(candidate, 5, 10.0)
            self.assertEqual(scan["checked"], 2)

    def test_survivor_partition_checks_candidate_and_removed_archive(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source_records = records(7)
            removed_indices = np.array([1, 4, 6], dtype=np.uint64)
            kept = np.delete(source_records, removed_indices)
            source = write_ply(root / "source.ply", source_records)
            candidate = write_ply(root / "candidate.ply", kept)
            removed = write_ply(root / "removed.ply", source_records[removed_indices])
            indices = root / "removed.npy"
            np.save(indices, removed_indices)
            report = V11.verify_exact_survivors(
                source, candidate, indices, len(kept), removed_records_path=removed
            )
            self.assertTrue(report["verified"])
            self.assertTrue(report["removed_archive_verified"])

            damaged = kept.copy()
            damaged[1]["z"] += 1.0
            write_ply(root / "damaged.ply", damaged)
            with self.assertRaisesRegex(RuntimeError, "survivor payload differs"):
                V11.verify_exact_survivors(
                    source, root / "damaged.ply", indices, len(kept)
                )

    def test_non_finite_xyz_is_counted(self):
        with tempfile.TemporaryDirectory() as directory:
            value = records(3)
            value[1]["z"] = np.nan
            path = write_ply(Path(directory) / "nonfinite.ply", value)
            self.assertEqual(V11._finite_xyz(path), 1)

    def test_raw_uint64_sidecar_rejects_partial_records(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "historical-name.npy"
            path.write_bytes(b"\x01\x00\x00")
            with self.assertRaisesRegex(RuntimeError, "uint64-aligned"):
                V11._load_indices(path, 10)


class TransactionTests(unittest.TestCase):
    def _items(self, root, count=3, names=None):
        items = []
        selected_names = list(names) if names is not None else [
            f"canonical-{index}.bin" for index in range(count)
        ]
        for index, name in enumerate(selected_names):
            canonical = root / name
            replacement = root / f"replacement-{index}.bin"
            canonical.write_bytes(f"old-{index}".encode())
            replacement.write_bytes(f"new-{index}".encode())
            items.append(
                V11.SwapItem(
                    label=str(index),
                    canonical=canonical,
                    replacement=replacement,
                    expected_current=V11.file_fingerprint(canonical, ply=False),
                    expected_replacement=V11.file_fingerprint(replacement, ply=False),
                )
            )
        return items

    def test_transaction_rename_swaps_everything_and_archives_previous_files(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            items = self._items(root, 6)
            committed = []
            with mock.patch.object(V11, "refuse_running_app") as process_probe:
                result = V11._transactional_replace(
                    items,
                    transaction_dir=root / "transaction",
                    manifest_commit=committed.append,
                )
            self.assertGreaterEqual(
                process_probe.call_count,
                2 * len(items) + 2,
            )
            self.assertEqual(len(committed), 1)
            self.assertEqual(result, committed[0])
            for index, item in enumerate(items):
                self.assertEqual(item.canonical.read_bytes(), f"new-{index}".encode())
                self.assertEqual(item.archived.read_bytes(), f"old-{index}".encode())
                self.assertFalse(item.replacement.exists())
                self.assertEqual(
                    result["replacement_methods"][item.label],
                    "same-filesystem-rename",
                )

    def test_transaction_rolls_back_all_prior_swaps_on_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            items = self._items(root)
            real_replace = V11.os.replace
            failed_once = False

            def replace(source, destination):
                nonlocal failed_once
                source = Path(source)
                destination = Path(destination)
                if (
                    not failed_once
                    and source == items[1].replacement
                    and destination == items[1].canonical
                ):
                    failed_once = True
                    raise OSError("injected swap failure")
                return real_replace(source, destination)

            with (
                mock.patch.object(V11, "refuse_running_app"),
                mock.patch.object(V11.os, "replace", side_effect=replace),
            ):
                with self.assertRaisesRegex(OSError, "injected swap failure"):
                    V11._transactional_replace(
                        items,
                        transaction_dir=root / "transaction",
                        manifest_commit=lambda _: None,
                    )
            for index, item in enumerate(items):
                self.assertEqual(item.canonical.read_bytes(), f"old-{index}".encode())
                self.assertEqual(item.replacement.read_bytes(), f"new-{index}".encode())
            journal = json.loads(
                (root / "transaction" / "journal.json").read_text()
            )
            self.assertEqual(journal["state"], "rolled-back")

    def test_rollback_preflights_every_item_before_any_rename(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            items = self._items(root, 2)
            transaction = root / "transaction"
            previous = transaction / "previous"
            previous.mkdir(parents=True)
            rows = []
            for item in items:
                item.archived = previous / item.canonical.name
                rows.append(
                    {
                        "label": item.label,
                        "canonical": str(item.canonical),
                        "replacement": str(item.replacement),
                        "archived": str(item.archived),
                        "expected_current": dict(item.expected_current),
                        "expected_replacement": dict(item.expected_replacement),
                        "phase": "intent",
                    }
                )

            # The final row encountered in reversed journal order is unknown.
            # The preceding valid row is a complete swap which the old
            # implementation would partially roll back before noticing it.
            V11._durable_replace(items[1].canonical, items[1].archived)
            V11._durable_replace(items[1].replacement, items[1].canonical)
            items[0].canonical.write_bytes(b"unknown-generation")
            before = {
                "canonical": items[1].canonical.read_bytes(),
                "archive": items[1].archived.read_bytes(),
                "replacement_exists": items[1].replacement.exists(),
            }
            journal_path = transaction / "journal.json"
            journal = {
                "schema_version": 2,
                "operation": "site1-v11-durable-six-cloud-transaction",
                "action": "install",
                "source_status": "built",
                "target_status": "installed",
                "state": "before-install-replacement",
                "created": V11._now(),
                "items": rows,
                "events": [],
            }
            V11._atomic_json(journal_path, journal)

            with self.assertRaisesRegex(
                RuntimeError, "matches neither journal generation"
            ):
                V11._rollback_transaction_journal(
                    journal_path,
                    journal,
                    recovered=True,
                    reason="test unknown generation",
                )

            self.assertEqual(items[1].canonical.read_bytes(), before["canonical"])
            self.assertEqual(items[1].archived.read_bytes(), before["archive"])
            self.assertEqual(
                items[1].replacement.exists(), before["replacement_exists"]
            )

    def test_startup_recovery_rolls_back_an_interrupted_install_journal(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            data = root / "Data" / "Scene1"
            data.mkdir(parents=True)
            release = root / "release"
            transaction = release / "transactions" / "install-interrupted"
            previous = transaction / "previous"
            previous.mkdir(parents=True)
            items = self._items(data, names=V11.CANONICAL_NAMES)
            for item, name in zip(items, V11.CANONICAL_NAMES):
                item.label = name.removeprefix("Site1-").removesuffix(".ply")
            rows = []
            for item in items:
                item.archived = previous / item.canonical.name
                rows.append(
                    {
                        "label": item.label,
                        "canonical": str(item.canonical),
                        "replacement": str(item.replacement),
                        "archived": str(item.archived),
                        "expected_current": dict(item.expected_current),
                        "expected_replacement": dict(item.expected_replacement),
                        "phase": "intent",
                    }
                )

            # Simulate a power loss after one complete swap and halfway through
            # the second; all remaining items have only been staged.
            V11._durable_replace(items[0].canonical, items[0].archived)
            V11._durable_replace(items[0].replacement, items[0].canonical)
            V11._durable_replace(items[1].canonical, items[1].archived)
            clouds = {
                item.label: {
                    "canonical": item.canonical.name,
                    "source": dict(item.expected_current),
                    "candidate": dict(item.expected_replacement),
                    "snapshot": {"path": str(release / item.canonical.name)},
                }
                for item in items
            }
            V11._atomic_json(
                release / "manifest.json",
                {
                    "schema_version": V11.SCHEMA_VERSION,
                    "operation": V11.OPERATION,
                    "status": "built",
                    "clouds": clouds,
                },
            )
            journal_path = transaction / "journal.json"
            V11._atomic_json(
                journal_path,
                {
                    "schema_version": 2,
                    "operation": "site1-v11-durable-six-cloud-transaction",
                    "action": "install",
                    "source_status": "built",
                    "target_status": "installed",
                    "state": "before-install-replacement",
                    "created": V11._now(),
                    "items": rows,
                    "events": [],
                },
            )

            with mock.patch.object(V11, "refuse_running_app"):
                recovered = V11._recover_incomplete_transactions(
                    SimpleNamespace(release_dir=release, data_dir=data)
                )
            self.assertEqual(
                recovered[0]["outcome"], "rolled-back-after-recovery"
            )
            for index, item in enumerate(items):
                self.assertEqual(
                    item.canonical.read_bytes(), f"old-{index}".encode()
                )
            final_journal = json.loads(journal_path.read_text())
            self.assertEqual(
                final_journal["state"], "rolled-back-after-recovery"
            )
            for index, item in enumerate(items):
                self.assertEqual(
                    item.replacement.read_bytes(), f"new-{index}".encode()
                )

    def test_fingerprints_reject_symlinks_and_special_files(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            target = root / "target.bin"
            target.write_bytes(b"immutable")
            link = root / "link.bin"
            link.symlink_to(target)
            with self.assertRaisesRegex(RuntimeError, "symbolic link"):
                V11.file_fingerprint(link, ply=False)

            fifo = root / "candidate.fifo"
            os.mkfifo(fifo)
            with self.assertRaisesRegex(RuntimeError, "special file"):
                V11.file_fingerprint(fifo, ply=False)

    def test_atomic_json_rejects_symlink_destination_without_touching_target(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            target = root / "outside.json"
            target.write_text('{"preserve": true}\n')
            redirected = root / "manifest.json"
            redirected.symlink_to(target)
            with self.assertRaisesRegex(RuntimeError, "symbolic link"):
                V11._atomic_json(redirected, {"preserve": False})
            self.assertEqual(target.read_text(), '{"preserve": true}\n')

    def test_load_json_rejects_atomic_entry_replacement_during_parse(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = write_json(root / "manifest.json", {"generation": 1})
            replacement = write_json(
                root / "replacement.json", {"generation": 1}
            )
            original_load = V11.json.load

            def replace_after_parse(handle):
                value = original_load(handle)
                os.replace(replacement, source)
                return value

            with (
                mock.patch.object(
                    V11.json, "load", side_effect=replace_after_parse
                ),
                self.assertRaisesRegex(RuntimeError, "changed while being read"),
            ):
                V11._load_json(source, "release manifest")

    def test_transaction_rejects_entry_identity_drift_before_any_rename(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            items = self._items(root, 2)
            replacement = items[1].replacement
            exact_copy = root / "hash-equal-replacement.bin"
            shutil.copy2(replacement, exact_copy)
            self.assertNotEqual(replacement.stat().st_ino, exact_copy.stat().st_ino)
            os.replace(exact_copy, replacement)
            with (
                mock.patch.object(V11, "refuse_running_app"),
                self.assertRaisesRegex(RuntimeError, "inode"),
            ):
                V11._transactional_replace(
                    items,
                    transaction_dir=root / "transaction",
                    manifest_commit=lambda _: None,
                )
            self.assertEqual(items[0].canonical.read_bytes(), b"old-0")
            self.assertEqual(items[0].replacement.read_bytes(), b"new-0")

    def test_transaction_rejects_symlinked_canonical_before_any_rename(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            items = self._items(root, 2)
            canonical = items[1].canonical
            target = root / "hash-equal-canonical.bin"
            canonical.rename(target)
            canonical.symlink_to(target)
            with (
                mock.patch.object(V11, "refuse_running_app"),
                self.assertRaisesRegex(RuntimeError, "symbolic link"),
            ):
                V11._transactional_replace(
                    items,
                    transaction_dir=root / "transaction",
                    manifest_commit=lambda _: None,
                )
            self.assertEqual(items[0].canonical.read_bytes(), b"old-0")
            self.assertEqual(items[0].replacement.read_bytes(), b"new-0")

    def test_transaction_binds_legacy_content_fingerprints_to_observed_inodes(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            items = self._items(root, 2)
            for item in items:
                item.expected_current = {
                    key: value
                    for key, value in item.expected_current.items()
                    if key not in {"device", "inode", "links", "mtime_ns"}
                }
                item.expected_replacement = {
                    key: value
                    for key, value in item.expected_replacement.items()
                    if key not in {"device", "inode", "links", "mtime_ns"}
                }
            with mock.patch.object(V11, "refuse_running_app"):
                V11._transactional_replace(
                    items,
                    transaction_dir=root / "transaction",
                    manifest_commit=lambda _: None,
                )
            journal = json.loads(
                (root / "transaction" / "journal.json").read_text()
            )
            for row in journal["items"]:
                self.assertIsInstance(row["expected_current"]["device"], int)
                self.assertIsInstance(row["expected_current"]["inode"], int)
                self.assertEqual(row["expected_current"]["links"], 1)
                self.assertIsInstance(row["expected_replacement"]["device"], int)
                self.assertIsInstance(row["expected_replacement"]["inode"], int)
                self.assertEqual(row["expected_replacement"]["links"], 1)

    def test_transaction_rejects_external_hardlink_before_any_rename(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            items = self._items(root, 2)
            for item in items:
                item.expected_current = dict(item.expected_current)
                item.expected_current.pop("links")
                item.expected_replacement = dict(item.expected_replacement)
                item.expected_replacement.pop("links")
            outside = root / "outside-alias.bin"
            os.link(items[1].replacement, outside)
            with (
                mock.patch.object(V11, "refuse_running_app"),
                self.assertRaisesRegex(RuntimeError, "external hard links"),
            ):
                V11._transactional_replace(
                    items,
                    transaction_dir=root / "transaction",
                    manifest_commit=lambda _: None,
                )
            self.assertEqual(items[0].canonical.read_bytes(), b"old-0")
            self.assertEqual(items[0].replacement.read_bytes(), b"new-0")
            self.assertEqual(outside.read_bytes(), b"new-1")

    def test_transaction_restores_validated_inode_after_source_path_race(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            items = self._items(root, 2)
            foreign = root / "foreign.bin"
            foreign.write_bytes(b"foreign-race-entry")
            real_replace = V11.os.replace
            raced = False

            def replace(source, destination):
                nonlocal raced
                source = Path(source)
                destination = Path(destination)
                if (
                    not raced
                    and source == items[0].canonical
                    and destination == items[0].archived
                ):
                    raced = True
                    # Swap the final pathname only after the helper's last
                    # fingerprint, exactly inside the rename call.
                    source.unlink()
                    real_replace(foreign, source)
                return real_replace(source, destination)

            with (
                mock.patch.object(V11, "refuse_running_app"),
                mock.patch.object(V11.os, "replace", side_effect=replace),
                self.assertRaisesRegex(RuntimeError, "rename source changed"),
            ):
                V11._transactional_replace(
                    items,
                    transaction_dir=root / "transaction",
                    manifest_commit=lambda _: None,
                )

            self.assertTrue(raced)
            self.assertEqual(items[0].canonical.read_bytes(), b"old-0")
            self.assertEqual(items[0].replacement.read_bytes(), b"new-0")
            self.assertFalse(items[0].archived.exists())
            conflicts = list(
                (root / "transaction" / "rename-guards").glob("*.conflict")
            )
            self.assertEqual(len(conflicts), 1)
            self.assertEqual(conflicts[0].read_bytes(), b"foreign-race-entry")
            journal = json.loads(
                (root / "transaction" / "journal.json").read_text()
            )
            self.assertEqual(journal["state"], "rolled-back")
            self.assertTrue(
                all(row.get("active_guard") is None for row in journal["items"])
            )

    def test_release_lock_is_shared_by_release_root_and_rejects_symlink(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            release = root / "shared-release"
            first = SimpleNamespace(run_dir=root / "run-a", release_dir=release)
            second = SimpleNamespace(run_dir=root / "run-b", release_dir=release)
            self.assertNotEqual(first.run_dir, second.run_dir)
            with V11.release_lock(first.release_dir):
                with self.assertRaisesRegex(RuntimeError, "holds the lock"):
                    with V11.release_lock(second.release_dir):
                        self.fail("same release lock was acquired twice")

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            release = root / "shared-release"
            target = root / "outside-lock.txt"
            target.write_text("preserve\n")
            lock = root / ".shared-release.site1-v11-release.lock"
            lock.symlink_to(target)
            with self.assertRaisesRegex(RuntimeError, "symbolic link"):
                with V11.release_lock(release):
                    self.fail("symlinked lock was acquired")
            self.assertEqual(target.read_text(), "preserve\n")

    def test_release_lock_rejects_hardlink_without_touching_target(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            release = root / "shared-release"
            target = root / "outside-lock.txt"
            target.write_text("do not touch\n")
            lock = root / ".shared-release.site1-v11-release.lock"
            os.link(target, lock)
            self.assertEqual(target.stat().st_nlink, 2)
            with self.assertRaisesRegex(RuntimeError, "multiple hard links"):
                with V11.release_lock(release):
                    self.fail("hard-linked lock was acquired")
            self.assertEqual(target.read_text(), "do not touch\n")


class ReleaseFixture:
    def __init__(self, root, *, drift_water_config=False):
        self.root = Path(root)
        self.data = self.root / "Data" / "Scene1"
        self.run = self.data / "PatchRefinement" / "v11"
        self.release = self.run / "release"
        self.data.mkdir(parents=True)
        self.config = write_json(
            self.run / "site1_fossils_v11_review.json",
            {"schema_version": 1, "fixture": True},
        )
        self.config_fingerprint = ordinary_fingerprint(self.config)
        self.water_config = self.config
        if drift_water_config:
            self.water_config = write_json(
                self.run / "different-water-review.json",
                {"schema_version": 1, "fixture": True, "different": True},
            )
        self.water_config_fingerprint = ordinary_fingerprint(self.water_config)
        scripts = SCRIPT.parent
        self.base_implementation = {
            name: V11.sha256_path(scripts / name)
            for name in (
                "site1_v11_water_pipeline.py",
                "site1_v11_water_density.py",
                "site1_v11_confidence.py",
            )
        }
        self.geometry_implementation = {
            name: V11.sha256_path(scripts / name)
            for name in (
                "site1_v11_hole_pipeline.py",
                "site1_v11_holes.py",
                "site1_v11_water_density.py",
                "site1_v11_confidence.py",
            )
        }
        self.scalar_implementation = {
            name: V11.sha256_path(scripts / name)
            for name in (
                "site1_v11_water_scalar_enrichment.py",
                "site1_v11_terrain.py",
                "site1_v11_terrain_pipeline.py",
                "site1_v11_hole_pipeline.py",
                "site1_v11_water_density.py",
                "site1_v11_confidence.py",
            )
        }
        self.obstruction_implementation = {
            name: V11.sha256_path(scripts / name)
            for name in (
                "site1_v11_obstruction_pipeline.py",
                "site1_v11_obstructions.py",
            )
        }
        self.terrain_implementation = {
            name: V11.sha256_path(scripts / name)
            for name in (
                "site1_v11_terrain_pipeline.py",
                "site1_v11_terrain.py",
                "site1_v11_confidence.py",
            )
        }
        surface_archive = self.run / "surface-reference.npz"
        surface_archive.write_bytes(b"fixture surface archive")
        surface_impl = scripts / "rebuild_site1_fossils_v10.py"
        self.reference_provenance = {
            "surface_archive": {
                "path": str(surface_archive.resolve()),
                "sha256": V11.sha256_path(surface_archive),
            },
            "surface_config": {
                "path": str(self.config.resolve()),
                "sha256": V11.sha256_path(self.config),
            },
            "implementation": {
                "path": str(surface_impl.resolve()),
                "sha256": V11.sha256_path(surface_impl),
            },
            "callable": "rebuild_site1_fossils_v10.surface_values",
            "noise_scale": 1.0,
        }
        self.original = records(6)
        for name in V11.CANONICAL_NAMES:
            write_ply(self.data / name, self.original)
        self.historic_water_5mm_old01 = write_ply(
            self.data / "Site1-WATER-5mm-old01.ply",
            records(2, start=99.0),
        )
        self.cleanmesh = self.run / "CleanMesh-fixture.bin"
        self.cleanmesh.write_bytes(b"fixture CleanMesh executable")
        values = {
            metric: {scale: 1.0 for scale in ("Fine", "Medium", "Broad")}
            for metric in (
                "MeanCurvature",
                "CrossCurvature",
                "Recession",
                "Roughness",
            )
        }
        self.normalization = write_json(
            self.run / "normalization.json",
            {"rock_combined_normalizations": values},
        )
        self.normalization_values = values
        self.terrain_candidates = {}
        self.water = {}
        self._obstructions()
        self._terrain()
        self._water_geometry("2mm")
        self._water_geometry("5mm")
        self._water_enrichment("2mm")
        self._water_enrichment("5mm")

    def _obstructions(self):
        top = {
            "schema": 1,
            "operation": "site1-v11-candidate-only-obstruction-pipeline",
            "canonical_writes": False,
            "config": {
                "source_path": str(self.config.resolve()),
                "sha256": V11.sha256_path(self.config),
                "archived_copy": "review-config.json",
            },
            "models": ["plane", "quadratic", "triangulated"],
            "parameters": {
                "surface": {
                    "anchor_cell_m": 0.025,
                    "anchor_quantile": 0.15,
                    "minimum_points_per_anchor": 3,
                    "maximum_anchors": 30000,
                    "neighbour_count": 16,
                    "query_chunk_points": 100000,
                    "huber_iterations": 8,
                    "huber_scale_m": 0.012,
                },
                "thresholds": {
                    "minimum_models": 2,
                    "maximum_model_spread_m": 0.05,
                    "seed_height_m": 0.08,
                    "grow_height_m": 0.035,
                    "review_height_m": 0.015,
                    "ground_stop_height_m": 0.02,
                    "fine_seed_distance_m": 0.008,
                },
                "preservation": {
                    "ground_band_m": 0.02,
                    "maximum_collar_removed_fraction": 0.001,
                    "maximum_cell_ground_loss_fraction": 0.10,
                },
                "fine_voxel_m": 0.006,
                "coarse_voxel_m": 0.0125,
                "boundary_guard_m": 0.02,
                "core_inset_fraction": 0.12,
                "cross_scale_distance_m": 0.008,
                "preservation_cell_m": 0.025,
                "chunk_records": 1000000,
            },
            "implementation": self.obstruction_implementation,
        }
        top["selection_contract"] = {
            "minimum_agreeing_surface_models": top["parameters"]["thresholds"]["minimum_models"],
            "maximum_model_spread_m": top["parameters"]["thresholds"]["maximum_model_spread_m"],
            "seed_height_m": top["parameters"]["thresholds"]["seed_height_m"],
            "grow_height_m": top["parameters"]["thresholds"]["grow_height_m"],
            "ground_stop_height_m": top["parameters"]["thresholds"]["ground_stop_height_m"],
            "cross_scale_distance_m": top["parameters"]["cross_scale_distance_m"],
            "bbox_is_locating_evidence_only": True,
            "requires_seed_connected_3d_component": True,
            "scanid9_absolute_protection": True,
        }
        archived_config = self.run / "obstructions" / "review-config.json"
        archived_config.parent.mkdir(parents=True, exist_ok=True)
        archived_config.write_bytes(self.config.read_bytes())
        for resolution, section in (("1mm", "fine"), ("5mm", "coarse")):
            source = self.data / f"Site1-ROCK-{resolution}.ply"
            source_hash = V11.sha256_path(source)
            bundle = self.run / "obstructions" / f"rock-{resolution}"
            bundle.mkdir(parents=True, exist_ok=True)
            removed_indices = np.array([2], dtype=np.uint64)
            np.save(bundle / "indices.npy", removed_indices)
            removed = write_ply(bundle / "removed.ply", self.original[removed_indices])
            kept = np.delete(self.original, removed_indices)
            candidate = write_ply(bundle / "candidate.ply", kept)
            bundle_manifest = {
                "canonical_writes": False,
                "source": {
                    "path": str(source),
                    "points": len(self.original),
                    "sha256": source_hash,
                },
                "candidate": {
                    "path": "candidate.ply",
                    "points": len(kept),
                    "sha256": V11.sha256_path(candidate),
                },
                "removed": {
                    "indices_path": "indices.npy",
                    "records_path": "removed.ply",
                    "records_sha256": V11.sha256_path(removed),
                    "records_are_exact_source_bytes": True,
                },
            }
            write_json(bundle / "manifest.json", bundle_manifest)
            top[section] = {
                "bundle": f"rock-{resolution}",
                "source_sha256": source_hash,
                "preservation": {"passed": True},
                "round_trip": {
                    "passed": True,
                    "reconstructed_sha256": source_hash,
                },
            }
        write_json(self.run / "obstructions" / "manifest.json", top)

    def _water_geometry(self, resolution):
        source = self.data / f"Site1-WATER-{resolution}.ply"
        base_dir = self.run / f"water-base-{resolution}"
        base_dir.mkdir(parents=True, exist_ok=True)
        removed_indices = np.array([1], dtype=np.uint64)
        # The production WATER stage intentionally uses a compact raw uint64
        # stream even though the historical sidecar suffix is ``.npy``.
        removed_indices.astype("<u8").tofile(base_dir / "thinned.npy")
        survivor = np.delete(self.original, removed_indices)
        recovery = records(1, start=20.0, scan_id=999.0)
        base_records = np.concatenate((survivor, recovery))
        base_candidate = write_ply(base_dir / "candidate.ply", base_records)
        recovery_audit = write_json(
            base_dir / "recovery.json",
            {
                "schema_version": 1,
                "candidate_only": True,
                "eligibility": (
                    "exact pre_allterrain minus post_allterrain WATER subsequence"
                ),
                "support_is_post_thinning": True,
                "recovery_passes_pointwise_retention": True,
                "accepted": 1,
            },
        )
        source_fp = ply_fingerprint(source)
        terrain_resolution = "1mm" if resolution == "2mm" else "5mm"
        comparison_dir = (
            self.data
            / "PatchRefinement"
            / "20260826-water-v10-blue-noise"
        )
        pre_comparison = write_ply(
            comparison_dir / f"combined-{resolution}-selected.ply",
            self.original,
        )
        post_comparison = write_ply(
            comparison_dir / f"combined-{resolution}-selected-allterrain.ply",
            self.original[:-1],
        )
        base_sources = [
            source,
            pre_comparison,
            post_comparison,
            self.terrain_candidates[f"SAND-{terrain_resolution}"],
            self.terrain_candidates[f"ROCK-{terrain_resolution}"],
        ]
        base_manifest = {
            "candidate_only": True,
            "canonical_install_performed": False,
            "config": self.water_config_fingerprint,
            "implementation": self.base_implementation,
            "parameters": {
                "taper_bbox": [0.0, 50.0, 0.0, 50.0],
                "nominal_spacing_m": 0.002 if resolution == "2mm" else 0.005,
                "interface_mark_ids": ["fixture-interface"],
                "cell_size_m": 0.05,
                "smoothing_bandwidth_m": 0.20,
                "taper_start_m": 0.0,
                "taper_end_m": 1.5,
                "floor_ratio": 0.08,
                "relaxed_terrain_ratio": 0.65,
                "duplicate_clearance_ratio": 0.90,
                "maximum_bridge_m": 0.03,
                "minimum_water_support": 3,
                "seed": 1234,
                "chunk_size": 1000000,
            },
            "interface_mark_ids": ["fixture-interface"],
            "density_reference": {
                "bbox": [0.0, 50.0, 0.0, 50.0],
                "cell_size_m": 0.05,
                "smoothing_bandwidth_m": 0.20,
                "taper_start_m": 0.0,
                "taper_end_m": 1.5,
                "floor_ratio": 0.08,
            },
            "candidate": {
                "path": str(base_candidate),
                "points": len(base_records),
                "sha256": V11.sha256_path(base_candidate),
            },
            "sources": {
                str(path.resolve()): ply_fingerprint(path)
                for path in base_sources
            },
            "thinning": {
                "source_points": len(self.original),
                "kept_points": len(survivor),
                "rejected_index_path": str(base_dir / "thinned.npy"),
                "rejected_index_sha256": V11.sha256_path(base_dir / "thinned.npy"),
            },
            "recovery": {
                "accepted": 1,
                "audit_path": str(recovery_audit),
                "audit_sha256": V11.sha256_path(recovery_audit),
                "support_is_post_thinning": True,
                "recovery_passes_pointwise_retention": True,
                "relaxed_terrain_clearance_m": (
                    (0.002 if resolution == "2mm" else 0.005) * 0.65
                ),
                "nominal_terrain_blocker_m": (
                    0.002 if resolution == "2mm" else 0.005
                ),
                "duplicate_clearance_m": (
                    (0.002 if resolution == "2mm" else 0.005) * 0.90
                ),
                "maximum_bridge_m": 0.03,
                "minimum_water_support": 3,
            },
            "invariants": {
                "existing_survivors_byte_exact": True,
                "heights_or_scalars_of_existing_records_rewritten": False,
                "recovery_validated_against_final_surviving_water": True,
                "pointwise_hash_seed": 1234,
            },
        }
        write_json(base_dir / "manifest.json", base_manifest)

        geometry_dir = self.run / f"water-geometry-{resolution}"
        geometry_dir.mkdir(parents=True, exist_ok=True)
        if resolution == "2mm":
            addition = records(2, start=30.0, scan_id=999.0)
            selection = None
            component_label = np.array([7, 7], dtype=np.int32)
        else:
            fine = self.water["2mm"]
            addition = fine["geometry_additions"][[0]].copy()
            selection = np.array([0], dtype=np.int64)
            component_label = np.array([7], dtype=np.int32)
        geometry_candidate = write_ply(
            geometry_dir / f"Site1-WATER-{resolution}.geometry.candidate.ply",
            np.concatenate((base_records, addition)),
        )
        archive = geometry_dir / "additions.npz"
        candidate_xy = np.column_stack((addition["x"], addition["y"]))
        if resolution == "2mm":
            np.savez_compressed(
                archive,
                records=addition,
                candidate_xy=candidate_xy,
                candidate_label=component_label,
                donor_local_index=np.arange(len(addition), dtype=np.int64),
                accepted_labels=np.array([7], dtype=np.int32),
            )
        else:
            np.savez_compressed(
                archive,
                records=addition,
                candidate_xy=candidate_xy,
                fine_selection_index=selection,
                fine_component_label=component_label,
                donor_local_index=np.arange(len(addition), dtype=np.int64),
            )
        holes = [{
            "seed_id": "hole-a",
            "label": 7,
            "bounds": [29.0, 31.0, 1.0, 3.0],
            "accepted": True,
        }]
        terrain_resolution = "1mm" if resolution == "2mm" else "5mm"
        terrain_sources = [
            ply_fingerprint(self.terrain_candidates[f"{role}-{terrain_resolution}"])
            for role in ("SAND", "ROCK")
        ]
        geometry_manifest = {
            "schema_version": 1,
            "candidate_only": True,
            "canonical_install_performed": False,
            "existing_payload_byte_exact": True,
            "config": self.water_config_fingerprint,
            "implementation": self.geometry_implementation,
            "reference_provenance": self.reference_provenance,
            "terrain_sources": terrain_sources,
            "source": {
                "path": str(base_candidate),
                "points": len(base_records),
                "sha256": V11.sha256_path(base_candidate),
            },
            "candidate": {
                "path": str(geometry_candidate),
                "points": len(base_records) + len(addition),
                "sha256": V11.sha256_path(geometry_candidate),
            },
            "review_bbox": [29.0, 31.0, 1.0, 3.0],
            "holes": holes,
            "sampling": {"xy": {"count": len(addition)}},
            "addition_count": len(addition),
            "archive": str(archive),
            "archive_sha256": V11.sha256_path(archive),
            "component_membership": {
                "archive_key": "candidate_label",
                "all_additions_assigned_to_accepted_component": True,
                "accepted_labels": [7],
            },
        }
        if resolution == "5mm":
            fine = self.water["2mm"]
            fine_manifest_fp = ordinary_fingerprint(fine["geometry_manifest"])
            fine_archive_fp = ordinary_fingerprint(fine["geometry_archive"])
            fine_xy = np.column_stack(
                (fine["geometry_additions"]["x"], fine["geometry_additions"]["y"])
            ).astype(np.float64)
            coarse_xy = np.column_stack((addition["x"], addition["y"])).astype(np.float64)
            maximum = float(np.max(np.linalg.norm(fine_xy - coarse_xy[0], axis=1)))
            geometry_manifest["cross_scale"] = {
                "method": "deterministic-variable-radius-blue-noise-subset-v1",
                "fine_manifest": fine_manifest_fp,
                "fine_archive": fine_archive_fp,
                "fine_candidate_sha256": fine["geometry_candidate_sha256"],
                "fine_addition_count": len(fine["geometry_additions"]),
                "fine_selection_index_count": len(selection),
                "fine_selection_index_unique": True,
                "coarse_xyz_exact_subset_of_fine_records_xyz": True,
                "coarse_normals_exact_subset_of_fine_records_normals": True,
                "nongeometry_fields_preserved_from_coarse_donors": True,
                "geometry_fields_copied_from_fine_records": [
                    "x", "y", "z", "nx", "ny", "nz",
                ],
                "selection_seed": 5678,
                "spacing_m": 0.05,
                "maximum_fine_to_coarse_or_terrain_support_distance_m": maximum,
                "accepted_hole_coverage": [{
                    "seed_id": "hole-a",
                    "component_label": 7,
                    "fine_count": 2,
                    "coarse_addition_count": 1,
                    "bounds": [29.0, 31.0, 1.0, 3.0],
                }],
            }
        geometry_manifest_path = write_json(
            geometry_dir / "manifest.json", geometry_manifest
        )
        self.water[resolution] = {
            "base_candidate": base_candidate,
            "base_records": base_records,
            "geometry_candidate": geometry_candidate,
            "geometry_candidate_sha256": V11.sha256_path(geometry_candidate),
            "geometry_manifest": geometry_manifest_path,
            "geometry_archive": archive,
            "geometry_additions": addition,
        }

    def _water_enrichment(self, resolution):
        water = self.water[resolution]
        geometry_manifest_path = water["geometry_manifest"]
        geometry_document = json.loads(geometry_manifest_path.read_text())
        base = water["base_candidate"]
        geometry_candidate = water["geometry_candidate"]
        archive = water["geometry_archive"]
        additions = water["geometry_additions"].copy()
        if resolution == "2mm":
            additions["scalar_A_R_Roughness_Fine"] += 0.5
            additions["scalar_A_R_Roughness_Combined"] += 0.25
        else:
            fine = self.water["2mm"]
            fine_enriched = V11.inspect_ply(fine["enriched_candidate"])
            memory = np.memmap(
                fine_enriched.path,
                dtype=fine_enriched.dtype,
                mode="r",
                offset=fine_enriched.offset,
                shape=(fine_enriched.count,),
            )
            additions["scalar_A_R_Roughness_Fine"] = memory[
                len(fine["base_records"])
            ]["scalar_A_R_Roughness_Fine"]
            additions["scalar_A_R_Roughness_Combined"] = memory[
                len(fine["base_records"])
            ]["scalar_A_R_Roughness_Combined"]
            del memory
        final_dir = self.run / f"water-final-{resolution}"
        final_dir.mkdir(parents=True, exist_ok=True)
        final_candidate = write_ply(
            final_dir / f"Site1-WATER-{resolution}.candidate.ply",
            np.concatenate((water["base_records"], additions)),
        )
        archived_geometry = final_dir / "geometry-manifest.json"
        archived_geometry.write_bytes(geometry_manifest_path.read_bytes())
        local_paths = {}
        for name in ("input", "input-manifest", "analysed", "cleanmesh-report"):
            local_paths[name] = final_dir / f"{name}.bin"
            local_paths[name].write_bytes(f"fixture {resolution} {name}".encode())
        terrain_resolution = "1mm" if resolution == "2mm" else "5mm"
        inputs = {
            "base_water": ply_fingerprint(base),
            "geometry_candidate": ply_fingerprint(geometry_candidate),
            "geometry_manifest": ordinary_fingerprint(geometry_manifest_path),
            "geometry_archive": ordinary_fingerprint(archive),
            "sand": ply_fingerprint(self.terrain_candidates[f"SAND-{terrain_resolution}"]),
            "rock": ply_fingerprint(self.terrain_candidates[f"ROCK-{terrain_resolution}"]),
            "cleanmesh": ordinary_fingerprint(self.cleanmesh),
            "normalization_manifest": ordinary_fingerprint(self.normalization),
        }
        if resolution == "5mm":
            fine = self.water["2mm"]
            inputs["fine_enriched_candidate"] = ply_fingerprint(fine["enriched_candidate"])
            inputs["fine_enriched_manifest"] = ordinary_fingerprint(fine["enriched_manifest"])
            inputs["fine_geometry_manifest"] = ordinary_fingerprint(
                fine["geometry_manifest"]
            )
            inputs["fine_geometry_archive"] = ordinary_fingerprint(
                fine["geometry_archive"]
            )
        archive_audit = V11._verify_npz_suffix_archive(
            geometry_candidate,
            base_count=len(water["base_records"]),
            archive_path=archive,
        )
        contract = {
            "base_points": len(water["base_records"]),
            "candidate_points": len(water["base_records"]) + len(additions),
            "addition_count": len(additions),
            "base_sha256": V11.sha256_path(base),
            "candidate_sha256": V11.sha256_path(geometry_candidate),
            "manifest_sha256": V11.sha256_path(geometry_manifest_path),
            "archive_sha256": V11.sha256_path(archive),
            "base_payload_sha256": V11._payload_sha256(base),
            "candidate_prefix_payload_sha256": V11._payload_sha256(
                geometry_candidate, count=len(water["base_records"])
            ),
            "archive_records_sha256": archive_audit["records_sha256"],
            "candidate_suffix_sha256": archive_audit["candidate_suffix_sha256"],
            "config_fingerprint": self.water_config_fingerprint,
        }
        geometry_fields = list(V11._water_scalar_geometry_fields(additions.dtype))
        scalar_audit = {
            "geometry_fields_replaced": geometry_fields,
            "changed_points_by_field": {
                name: len(additions) for name in geometry_fields
            },
            "non_geometry_fields_archive_exact": True,
            "intensity_and_composition_archive_exact": True,
            "restored_scan_id": 999,
            "minimum_component_field_finite_fraction": 1.0,
        }
        array_hash = lambda value: __import__("hashlib").sha256(
            np.ascontiguousarray(value).tobytes()
        ).hexdigest()
        if resolution == "2mm":
            with np.load(archive, allow_pickle=False) as loaded:
                candidate_label = np.asarray(
                    loaded["candidate_label"], np.int64
                )
            scalar_audit["geometry_component_membership"] = {
                "archive": ordinary_fingerprint(archive),
                "archive_arrays_verified": [
                    "candidate_label",
                    "candidate_xy",
                    "records",
                ],
                "candidate_xy_record_exact": True,
                "component_label_count": len(candidate_label),
                "component_label_sha256": array_hash(candidate_label),
                "accepted_labels": [7],
            }
            active_component_labels = candidate_label
        if resolution == "5mm":
            fine = self.water["2mm"]
            with np.load(archive, allow_pickle=False) as loaded:
                selection = np.asarray(loaded["fine_selection_index"])
                coarse_labels = np.asarray(loaded["fine_component_label"])
            with np.load(fine["geometry_archive"], allow_pickle=False) as loaded:
                fine_labels = np.asarray(loaded["candidate_label"], np.int64)
            cross = geometry_document["cross_scale"]
            required_cross_keys = {
                "method",
                "fine_manifest",
                "fine_archive",
                "fine_candidate_sha256",
                "fine_addition_count",
                "fine_selection_index_count",
                "fine_selection_index_unique",
                "coarse_xyz_exact_subset_of_fine_records_xyz",
                "coarse_normals_exact_subset_of_fine_records_normals",
                "nongeometry_fields_preserved_from_coarse_donors",
                "geometry_fields_copied_from_fine_records",
                "selection_seed",
                "spacing_m",
                "maximum_fine_to_coarse_or_terrain_support_distance_m",
                "accepted_hole_coverage",
            }
            scalar_audit.update({
                "method": "exact-fine-selection-index-geometry-transfer",
                "source_is_locally_cleanmesh_enriched_fine_suffix": True,
                "fine_candidate": {
                    "path": str(fine["enriched_candidate"]),
                    "sha256": V11.sha256_path(fine["enriched_candidate"]),
                    "points": V11.inspect_ply(fine["enriched_candidate"]).count,
                    "base_points": len(fine["base_records"]),
                    "addition_count": len(fine["geometry_additions"]),
                    "base_payload_sha256": V11._payload_sha256(
                        fine["enriched_candidate"], count=len(fine["base_records"])
                    ),
                },
                "fine_manifest": {
                    "path": str(fine["enriched_manifest"]),
                    "sha256": V11.sha256_path(fine["enriched_manifest"]),
                },
                "fine_selection_index": {
                    "archive_key": "fine_selection_index",
                    "count": len(selection),
                    "unique": True,
                    "minimum": int(selection.min()),
                    "maximum": int(selection.max()),
                    "sha256": array_hash(selection),
                },
                "component_membership": {
                    "archive": ordinary_fingerprint(fine["geometry_archive"]),
                    "archive_arrays_verified": ["candidate_label", "candidate_xy", "records"],
                    "candidate_xy_record_exact": True,
                    "component_label_count": len(fine_labels),
                    "component_label_sha256": array_hash(fine_labels.astype(np.int64)),
                    "accepted_labels": [7],
                    "coarse_archive_key": "fine_component_label",
                    "coarse_component_label_sha256": array_hash(
                        coarse_labels.astype(np.int64)
                    ),
                    "coarse_labels_match_selected_fine_labels": True,
                },
                "xyz_and_normals_byte_exact": True,
                "cross_scale_manifest": geometry_document["cross_scale"],
                "cross_scale_verification": {
                    "required_keys_verified": sorted(required_cross_keys),
                    "fine_geometry_manifest_sha256_verified": True,
                    "fine_geometry_archive_sha256_verified": True,
                    "fine_geometry_candidate_sha256_verified": True,
                    "fine_coarse_normalization_manifest_sha256_verified": True,
                    "fine_addition_count_verified": True,
                    "fine_selection_index_count_verified": True,
                    "fine_selection_index_unique_verified": True,
                    "coarse_xyz_subset_attestation_verified": True,
                    "coarse_normals_subset_attestation_verified": True,
                    "coarse_donor_nongeometry_attestation_verified": True,
                    "coarse_component_labels_match_selected_fine_labels": True,
                    "accepted_hole_coverage_rows_verified": 1,
                    "accepted_hole_coverage": cross["accepted_hole_coverage"],
                    "spacing_m": cross["spacing_m"],
                    "maximum_support_distance_m": cross[
                        "maximum_fine_to_coarse_or_terrain_support_distance_m"
                    ],
                },
            })
            active_component_labels = coarse_labels
        scalar_audit["component_field_finite_coverage"] = (
            V11._component_field_scalar_coverage(
                additions,
                active_component_labels,
                geometry_fields,
                label=f"fixture {resolution} WATER additions",
            )
        )
        final_manifest = dict(geometry_document)
        final_manifest.update({
            "operation": "site1-v11-candidate-only-water-addition-scalar-enrichment",
            "status": "built",
            "resolution_label": resolution,
            "nominal_spacing_m": 0.002 if resolution == "2mm" else 0.005,
            "geometry_contract": contract,
            "geometry_manifest": {
                "path": str(geometry_manifest_path),
                "sha256": V11.sha256_path(geometry_manifest_path),
                "archived_copy": "geometry-manifest.json",
                "archived_copy_sha256": V11.sha256_path(archived_geometry),
                "operation": "site1-v11-candidate-only-water-hole-completion",
                "candidate": geometry_document["candidate"],
            },
            "input_fingerprints": inputs,
            "scalar_enrichment_implementation": self.scalar_implementation,
            "parameters": {
                "semantic": {
                    "minimum_component_field_finite_fraction": 1.0,
                },
            },
            "combined_geometry_normalization": {
                "method": "provided-manifest",
                "path": str(self.normalization),
                "sha256": V11.sha256_path(self.normalization),
                "values": self.normalization_values,
            },
            "local_analysis": {
                "water_type_id": 1,
                "temporary_addition_scan_id": 10.0,
                "collar_m": 0.46,
                "collars": {},
                "input": local_paths["input"].name,
                "input_sha256": V11.sha256_path(local_paths["input"]),
                "input_manifest": local_paths["input-manifest"].name,
                "input_manifest_sha256": V11.sha256_path(local_paths["input-manifest"]),
                "analysed": local_paths["analysed"].name,
                "analysed_sha256": V11.sha256_path(local_paths["analysed"]),
                "cleanmesh_report": local_paths["cleanmesh-report"].name,
                "cleanmesh_report_sha256": V11.sha256_path(local_paths["cleanmesh-report"]),
                "runner": {},
                "full_cloud_analysis": False,
                "tagged_addition_count": len(additions),
                "tagged_identity_verified_after_tiled_output": True,
                "output_policy": {
                    "accepted_for_output": resolution == "2mm",
                    "purpose": "fixture",
                },
            },
            "scalar_enrichment": scalar_audit,
            "candidate": {
                "path": str(final_candidate),
                "points": len(water["base_records"]) + len(additions),
                "sha256": V11.sha256_path(final_candidate),
                "base_payload_sha256": V11._payload_sha256(
                    final_candidate, count=len(water["base_records"])
                ),
                "suffix_sha256": V11._payload_sha256(
                    final_candidate,
                    start=len(water["base_records"]),
                    count=len(additions),
                ),
            },
            "invariants": {
                **geometry_document.get("invariants", {}),
                "geometry_candidate_verified_as_base_plus_archive": True,
                "existing_base_payload_byte_exact": True,
                "coordinates_and_normals_archive_exact": True,
                "colour_intensity_composition_archive_exact": True,
                "visibility_fields_archive_exact": True,
                "geometry_metrics_from_local_cleanmesh": True,
                "coarse_geometry_metrics_from_exact_fine_selection": resolution == "5mm",
                "coarse_local_cleanmesh_is_diagnostic_only": resolution == "5mm",
                "combined_metrics_use_v10_global_normalization": True,
                "undefined_geometry_fallback_component_strict": True,
                "undefined_geometry_fallback_no_extrapolation": True,
                "geometry_component_membership_verified": True,
                "component_field_scalar_coverage_complete": True,
                "component_field_scalar_ranges_verified": True,
                "final_addition_scan_id": 999.0,
                "canonical_writes": False,
            },
        })
        final_manifest_path = write_json(final_dir / "manifest.json", final_manifest)
        water.update(
            enriched_candidate=final_candidate,
            enriched_manifest=final_manifest_path,
        )

    def _terrain(self):
        terrain_dir = self.run / "terrain"
        terrain_dir.mkdir(parents=True, exist_ok=True)
        targets = [
            {
                "id": "fixture-sand",
                "kind": "marked",
                "bbox": [-0.1, 0.1, 1.9, 2.2],
                "minimum_tier": "SUPPORTED",
            },
            {
                "id": "fixture-rock",
                "kind": "crack",
                "bbox": [-0.1, 0.1, 1.9, 2.2],
                "minimum_tier": "STRONG",
            },
        ]
        manifest = {
            "schema_version": 1,
            "operation": "site1-v11-candidate-only-terrain-interstitial-pipeline",
            "status": "built",
            "candidate_only": True,
            "canonical_install_performed": False,
            "config": {
                "path": str(self.config.resolve()),
                "sha256": V11.sha256_path(self.config),
                "archived_copy": "review-config.json",
            },
            "cleanmesh": {
                "path": str(self.cleanmesh),
                "sha256": V11.sha256_path(self.cleanmesh),
                "scope": "local measured collars plus ScanID10 additions only",
            },
            "parameters": {
                "fine": self._resolution_parameters("1mm", 0.0015),
                "coarse": self._resolution_parameters("5mm", 0.005),
                "confidence": {
                    "minimum_donor_sectors": 6,
                    "strong_donor_sectors": 7,
                    "maximum_surface_spread_m": 0.003,
                    "strong_surface_spread_m": 0.002,
                    "minimum_normal_coherence": 0.8,
                    "strong_normal_coherence": 0.9,
                    "maximum_vertical_thickness_m": 0.012,
                    "strong_vertical_thickness_m": 0.006,
                    "maximum_multimodality_score": 0.35,
                    "strong_multimodality_score": 0.15,
                    "maximum_residual_energy_ratio": 2.0,
                    "supported_preferred_gates": 4,
                },
                "role_dominance_ratio": 1.12,
                "chunk_records": 1000000,
                "cleanmesh_tile_width_m": 4.0,
                "cleanmesh_chunk_points": 1000000,
                "cleanmesh_normalization_samples": 2000000,
                "global_normalization_samples": 2000000,
                "cross_scale_vertical_tolerance_m": 0.012,
                "cross_scale_distance_tolerance_m": 1.0e-9,
                "seed": 1234,
            },
            "implementation": self.terrain_implementation,
            "combined_geometry_normalization": {
                "method": "provided-manifest",
                "path": str(self.normalization),
                "sha256": V11.sha256_path(self.normalization),
                "values": self.normalization_values,
                "archived_copy": "normalization-manifest.json",
            },
            "invariants": {
                "bbox_is_never_a_fill_mask": True,
                "connected_density_deficit_required": True,
                "all_three_surface_predictions_evaluated": True,
                "hard_geometry_vetoes_fail_closed": True,
                "overlapping_targets_globally_arbitrated_per_role": True,
                "same_role_scanid_0_to_8_donors_only": True,
                "existing_records_modified": 0,
                "all_additions_scanid": 10.0,
                "caller_rock_base_preserved_byte_exact": True,
                "fine_additions_authoritative_for_coarse_geometry": True,
                "coarse_additions_exact_fine_xyz_subset": True,
                "coarse_maximal_surface_coverage_verified": True,
                "cross_scale_vertical_support_guard_verified": True,
                "canonical_writes": False,
            },
            "sources": {},
            "resolutions": {},
            "targets": targets,
        }
        archived_config = terrain_dir / "review-config.json"
        archived_config.parent.mkdir(parents=True, exist_ok=True)
        archived_config.write_bytes(self.config.read_bytes())
        (terrain_dir / "normalization-manifest.json").write_bytes(
            self.normalization.read_bytes()
        )
        source_records = {
            "SAND": self.original,
            "ROCK": np.delete(self.original, [2]),
        }
        source_paths = {}
        for resolution in ("1mm", "5mm"):
            for role in ("SAND", "ROCK"):
                source = (
                    self.data / f"Site1-SAND-{resolution}.ply"
                    if role == "SAND"
                    else self.run / "obstructions" / f"rock-{resolution}" / "candidate.ply"
                )
                key = f"{role}-{resolution}" if role == "SAND" else f"ROCK-{resolution}-base"
                manifest["sources"][key] = ply_fingerprint(source)
                source_paths[f"{role}-{resolution}"] = source

        addition = records(1, start=0.0, scan_id=10.0)
        addition["x"] = 0.005
        addition["y"] = 2.01
        addition["z"] = 3.0
        fine_dir = terrain_dir / "terrain-1mm"
        fine_candidates = {}
        fine_hashes = {}
        fine_archives = {}
        fine_archive_hashes = {}
        fine_append = {}
        fine_archive_audits = {}
        arbitration_roles = {}
        target_audits = []
        for role in ("SAND", "ROCK"):
            target_id = f"fixture-{role.lower()}"
            source = source_paths[f"{role}-1mm"]
            source_fp = V11.file_fingerprint(source)
            candidate = write_ply(
                fine_dir / f"{role}.ply",
                np.concatenate((source_records[role], addition)),
            )
            candidate_hash = V11.sha256_path(candidate)
            fine_candidates[role] = str(candidate)
            fine_hashes[role] = candidate_hash
            self.terrain_candidates[f"{role}-1mm"] = candidate
            fine_append[role] = {
                "source_path": str(source),
                "source_sha256": source_fp["sha256"],
                "source_vertex_count": len(source_records[role]),
                "candidate_path": candidate.name,
                "candidate_sha256": candidate_hash,
                "candidate_vertex_count": len(source_records[role]) + 1,
                "addition_count": 1,
                "base_payload_byte_identical": True,
                "canonical_writes": False,
            }
            donor_xyz = np.column_stack(
                tuple(source_records[role][name][[0]] for name in ("x", "y", "z"))
            ).astype(np.float64)
            addition_xyz = np.column_stack(
                tuple(addition[name] for name in ("x", "y", "z"))
            ).astype(np.float64)
            donor_distance = float(np.linalg.norm(addition_xyz[0] - donor_xyz[0]))
            archive_arrays = {
                "records": addition.copy(),
                "fine_index": np.array([0], np.int64),
                "target_id": np.array([target_id]),
                "target_candidate_index": np.array([0], np.int64),
                "global_ledger_index": np.array([0], np.int64),
                "radius_m": np.array([0.0015], np.float64),
                "priority": np.array([1.0], np.float64),
                "confidence_reason_mask": np.array([0], np.uint32),
                "confidence_tier": np.array([2], np.uint8),
                "confidence_surface_spread_m": np.array([0.001], np.float64),
                "confidence_preferred_gate_count": np.array([4], np.uint8),
                "target_donor_index": np.array([0], np.int64),
                "target_donor_distance_m": np.array([donor_distance], np.float64),
                "target_donor_count": np.array([1], np.int32),
            }
            archive_path = fine_dir / f"authoritative-additions-{role.lower()}.npz"
            np.savez_compressed(archive_path, **archive_arrays)
            archive_hash = V11.sha256_path(archive_path)
            fine_archives[role] = str(archive_path)
            fine_archive_hashes[role] = archive_hash

            ledger_arrays = {
                "xyz": addition_xyz,
                "radius_m": archive_arrays["radius_m"],
                "local_priority": archive_arrays["priority"],
                "global_priority": np.array([5.0], np.float64),
                "target_candidate_index": archive_arrays["target_candidate_index"],
                "target_donor_index": archive_arrays["target_donor_index"],
                "target_donor_distance_m": archive_arrays["target_donor_distance_m"],
                "target_donor_count": archive_arrays["target_donor_count"],
                "confidence_reason_mask": archive_arrays["confidence_reason_mask"],
                "confidence_tier": archive_arrays["confidence_tier"],
                "confidence_surface_spread_m": archive_arrays["confidence_surface_spread_m"],
                "confidence_preferred_gate_count": archive_arrays["confidence_preferred_gate_count"],
                "target_id": archive_arrays["target_id"],
                "globally_selected": np.array([True]),
                "disposition": np.array([1], np.uint8),
                "decision_reason_mask": np.array([0], np.uint32),
            }
            ledger_path = fine_dir / f"global-arbitration-{role.lower()}.npz"
            np.savez_compressed(ledger_path, **ledger_arrays)
            ledger_audit = {
                "locally_accepted": 1,
                "globally_accepted": 1,
                "overlap_rejected": 0,
                "minimum_tier": "SUPPORTED",
                "target_counts": {
                    target_id: {"locally_accepted": 1, "globally_accepted": 1}
                },
                "spacing": {"measured_clearance_verified": True, "candidate_pair_clearance_verified": True},
                "ledger_npz": ledger_path.name,
                "ledger_npz_sha256": V11.sha256_path(ledger_path),
            }
            ledger_json = write_json(
                fine_dir / f"global-arbitration-{role.lower()}.json", ledger_audit
            )
            ledger_audit["ledger_json"] = ledger_json.name
            ledger_audit["ledger_json_sha256"] = V11.sha256_path(ledger_json)
            arbitration_roles[role] = ledger_audit

            provenance_dir = fine_dir / "targets" / target_id
            provenance_npz = provenance_dir / "proposal-provenance.npz"
            provenance_arrays = {
                "xyz": addition_xyz,
                "target_id": np.array([target_id]),
                "disposition": np.array([1], np.uint8),
                "decision_reason_mask": np.array([0], np.uint32),
                "confidence_reason_mask": archive_arrays["confidence_reason_mask"],
                "confidence_tier": archive_arrays["confidence_tier"],
                "surface_spread_m": archive_arrays["confidence_surface_spread_m"],
                "donor_index": archive_arrays["target_donor_index"],
                "donor_distance_m": archive_arrays["target_donor_distance_m"],
            }
            provenance_dir.mkdir(parents=True, exist_ok=True)
            np.savez_compressed(provenance_npz, **provenance_arrays)
            provenance_json = write_json(
                provenance_dir / "proposal-provenance.json", {"target_id": target_id}
            )
            provenance = {
                "npz": str(provenance_npz.relative_to(fine_dir)),
                "json": str(provenance_json.relative_to(fine_dir)),
                "npz_sha256": V11.sha256_path(provenance_npz),
                "json_sha256": V11.sha256_path(provenance_json),
            }
            fine_archive_audits[role] = {
                "path": archive_path.name,
                "sha256": archive_hash,
                "keys": sorted(V11.TERRAIN_FINE_ARCHIVE_KEYS),
                "points": 1,
                "record_stride": DTYPE.itemsize,
                "record_payload_sha256": V11._array_sha256(addition),
                "candidate_suffix_byte_exact": True,
                "candidate_sha256": candidate_hash,
                "global_arbitration_ledger": ledger_path.name,
                "global_arbitration_ledger_sha256": V11.sha256_path(ledger_path),
                "target_provenance": {target_id: provenance},
            }
            target = next(item for item in targets if item["id"] == target_id)
            target_audits.append(
                {
                    "target_id": target_id,
                    "kind": target["kind"],
                    "bbox": target["bbox"],
                    "minimum_tier": target["minimum_tier"],
                    "provenance": provenance,
                }
            )

        def cleanmesh_audit(directory, *, coarse):
            local_input = write_ply(directory / "local-collars.analysis-input.ply", addition)
            local_manifest = write_json(directory / "local-collars.analysis-input.ply.manifest.json", {"fixture": True})
            analysed = write_ply(directory / "local-collars.analysis.ply", addition)
            report_path = write_json(directory / "local-collars.cleanmesh-report.json", {"fixture": True})
            value = {
                "status": "completed",
                "local_input": local_input.name,
                "local_input_sha256": V11.sha256_path(local_input),
                "local_manifest": local_manifest.name,
                "local_manifest_sha256": V11.sha256_path(local_manifest),
                "analysed": analysed.name,
                "analysed_sha256": V11.sha256_path(analysed),
                "report": report_path.name,
                "report_sha256": V11.sha256_path(report_path),
                "runner": {
                    "command": [str(self.cleanmesh), "--fixture"],
                    "returncode": 0,
                    "output_sha256": V11.sha256_path(analysed),
                    "report_sha256": V11.sha256_path(report_path),
                },
                "collar_points": len(self.original),
                "addition_points": 2,
                "full_cloud_analysis": False,
                "span_identity_verified": True,
                "scalar_postprocess": [],
            }
            if coarse:
                value["exact_fine_xyz_preserved"] = True
            return value

        fine_report = {
            "schema_version": 1,
            "candidate_only": True,
            "canonical_writes": False,
            "resolution": manifest["parameters"]["fine"],
            "targets": target_audits,
            "global_arbitration": {"roles": arbitration_roles},
            "cleanmesh": cleanmesh_audit(fine_dir, coarse=False),
            "append_only": fine_append,
            "authoritative_addition_archives": fine_archive_audits,
            "addition_counts": {"SAND": 1, "ROCK": 1},
            "invariants": {
                "annotations_are_search_windows_only": True,
                "actual_connected_deficit_required": True,
                "same_role_measured_property_donors_only": True,
                "overlapping_targets_globally_arbitrated": True,
                "addition_scan_id": 10.0,
                "crack_minimum_tier": "STRONG",
                "marked_scanner_minimum_tier": "SUPPORTED",
                "existing_base_payload_byte_exact": True,
                "authoritative_archive_equals_candidate_suffix": True,
            },
        }
        fine_report_path = write_json(fine_dir / "resolution-report.json", fine_report)
        fine_result = {
            "label": "1mm",
            "candidate_paths": fine_candidates,
            "candidate_sha256": fine_hashes,
            "addition_counts": {"SAND": 1, "ROCK": 1},
            "report_path": str(fine_report_path),
            "report_sha256": V11.sha256_path(fine_report_path),
            "target_count": len(targets),
            "addition_archive_paths": fine_archives,
            "addition_archive_sha256": fine_archive_hashes,
            "cross_scale_report_path": None,
            "cross_scale_report_sha256": None,
        }
        manifest["resolutions"]["1mm"] = fine_result

        coarse_dir = terrain_dir / "terrain-5mm"
        coarse_candidates = {}
        coarse_hashes = {}
        coarse_archives = {}
        coarse_archive_hashes = {}
        coarse_append = {}
        cross_roles = {}
        for role in ("SAND", "ROCK"):
            target_id = f"fixture-{role.lower()}"
            source = source_paths[f"{role}-5mm"]
            source_fp = V11.file_fingerprint(source)
            candidate = write_ply(
                coarse_dir / f"{role}.ply",
                np.concatenate((source_records[role], addition)),
            )
            candidate_hash = V11.sha256_path(candidate)
            coarse_candidates[role] = str(candidate)
            coarse_hashes[role] = candidate_hash
            self.terrain_candidates[f"{role}-5mm"] = candidate
            coarse_append[role] = {
                "source_path": str(source),
                "source_sha256": source_fp["sha256"],
                "source_vertex_count": len(source_records[role]),
                "candidate_path": candidate.name,
                "candidate_sha256": candidate_hash,
                "candidate_vertex_count": len(source_records[role]) + 1,
                "addition_count": 1,
                "base_payload_byte_identical": True,
                "canonical_writes": False,
            }
            with np.load(fine_archives[role], allow_pickle=False) as fine_archive:
                fine_values = {name: np.asarray(fine_archive[name]).copy() for name in fine_archive.files}
            donor_xyz = np.column_stack(
                tuple(source_records[role][name][[0]] for name in ("x", "y", "z"))
            ).astype(np.float64)
            addition_xyz = np.column_stack(
                tuple(addition[name] for name in ("x", "y", "z"))
            ).astype(np.float64)
            donor_distance = float(np.linalg.norm(addition_xyz[0] - donor_xyz[0]))
            coarse_arrays = {
                "records": addition.copy(),
                "fine_selection_index": np.array([0], np.int64),
                "fine_target_id": fine_values["target_id"],
                "fine_target_candidate_index": fine_values["target_candidate_index"],
                "fine_global_ledger_index": fine_values["global_ledger_index"],
                "fine_confidence_reason_mask": fine_values["confidence_reason_mask"],
                "fine_confidence_tier": fine_values["confidence_tier"],
                "fine_confidence_surface_spread_m": fine_values["confidence_surface_spread_m"],
                "fine_confidence_preferred_gate_count": fine_values["confidence_preferred_gate_count"],
                "coarse_primary_donor_source_index": np.array([0], np.int64),
                "coarse_nearest_donor_distance_m": np.array([donor_distance], np.float64),
                "coarse_contributing_donor_count": np.array([1], np.int32),
            }
            archive_path = coarse_dir / f"cross-scale-additions-{role.lower()}.npz"
            np.savez_compressed(archive_path, **coarse_arrays)
            archive_hash = V11.sha256_path(archive_path)
            coarse_archives[role] = str(archive_path)
            coarse_archive_hashes[role] = archive_hash
            coverage_path = coarse_dir / f"cross-scale-coverage-{role.lower()}.npz"
            np.savez_compressed(
                coverage_path,
                fine_index=np.array([0], np.int64),
                represented_by_existing=np.array([False]),
                selected_for_coarse=np.array([True]),
                coverage_source=np.array([2], np.uint8),
                coverage_index=np.array([0], np.int64),
                coverage_xy_distance_m=np.array([0.0], np.float64),
                coverage_vertical_delta_m=np.array([0.0], np.float64),
            )
            cross_roles[role] = {
                "fine_authority": {
                    "candidate": os.path.relpath(fine_candidates[role], coarse_dir),
                    "candidate_sha256": fine_hashes[role],
                    "archive": os.path.relpath(fine_archives[role], coarse_dir),
                    "archive_sha256": fine_archive_hashes[role],
                    "points": 1,
                },
                "fine_points_already_represented_by_5mm_measured_terrain": 0,
                "fine_points_selected_for_5mm": 1,
                "fine_points_covered": 1,
                "maximum_coverage_xy_distance_m": 0.0,
                "maximum_coverage_vertical_delta_m": 0.0,
                "minimum_selected_to_existing_xy_distance_m": None,
                "pair_spacing": {
                    "minimum_vertical_compatible_pair_xy_distance_m": None,
                    "verified": True,
                },
                "target_counts": {target_id: 1},
                "coarse_archive": archive_path.name,
                "coarse_archive_sha256": archive_hash,
                "coarse_archive_keys": sorted(V11.TERRAIN_COARSE_ARCHIVE_KEYS),
                "coverage_ledger": coverage_path.name,
                "coverage_ledger_sha256": V11.sha256_path(coverage_path),
                "exact_xyz_subset_verified": True,
                "coverage_verified": True,
                "terrain_clearance_verified": True,
                "vertical_support_guard_verified": True,
                "same_role_measured_donor_transfer_verified": True,
                "cleanmesh_geometry_identity_verified": True,
            }

        cross_invariants = {
            "fine_final_additions_are_authoritative": True,
            "coarse_addition_xyz_is_exact_fine_subset": True,
            "coarse_independent_geometry_proposals": 0,
            "maximal_coverage_at_coarse_spacing": True,
            "floating_returns_do_not_count_as_surface_support": True,
            "same_role_scanid_0_to_8_property_donors_only": True,
            "cleanmesh_recomputed_coarse_scalars_without_xyz_changes": True,
            "candidate_only": True,
            "canonical_writes": False,
        }
        cross_report = {
            "schema_version": 1,
            "method": "deterministic-maximal-surface-aware-fine-xyz-subset-v1",
            "fine_resolution_report": os.path.relpath(fine_report_path, coarse_dir),
            "fine_resolution_report_sha256": fine_result["report_sha256"],
            "coarse_spacing_m": 0.005,
            "terrain_clearance_m": 0.005,
            "vertical_support_tolerance_m": 0.012,
            "distance_tolerance_m": 1.0e-9,
            "selection_dimensions": "XY with explicit absolute-Z compatibility guard",
            "roles": cross_roles,
            "invariants": cross_invariants,
        }
        cross_report_path = write_json(coarse_dir / "cross-scale-report.json", cross_report)
        coarse_report = {
            "schema_version": 2,
            "candidate_only": True,
            "canonical_writes": False,
            "resolution": manifest["parameters"]["coarse"],
            "construction": "exact-subset-of-final-1mm-authoritative-additions",
            "independent_target_proposals": 0,
            "fine_authority_report_sha256": fine_result["report_sha256"],
            "cross_scale": {
                "report": cross_report_path.name,
                "report_sha256": V11.sha256_path(cross_report_path),
            },
            "cleanmesh": cleanmesh_audit(coarse_dir, coarse=True),
            "append_only": coarse_append,
            "addition_counts": {"SAND": 1, "ROCK": 1},
            "invariants": cross_invariants,
        }
        coarse_report_path = write_json(coarse_dir / "resolution-report.json", coarse_report)
        coarse_result = {
            "label": "5mm",
            "candidate_paths": coarse_candidates,
            "candidate_sha256": coarse_hashes,
            "addition_counts": {"SAND": 1, "ROCK": 1},
            "report_path": str(coarse_report_path),
            "report_sha256": V11.sha256_path(coarse_report_path),
            "target_count": len(targets),
            "addition_archive_paths": coarse_archives,
            "addition_archive_sha256": coarse_archive_hashes,
            "cross_scale_report_path": str(cross_report_path),
            "cross_scale_report_sha256": V11.sha256_path(cross_report_path),
        }
        manifest["resolutions"]["5mm"] = coarse_result
        manifest["cross_scale"] = dict(cross_report)
        manifest["cross_scale"]["report"] = {
            "path": str(cross_report_path),
            "sha256": V11.sha256_path(cross_report_path),
        }
        write_json(terrain_dir / "manifest.json", manifest)

    @staticmethod
    def _resolution_parameters(label, spacing):
        return {
            "label": label,
            "nominal_spacing_m": spacing,
            "deficit_cell_size_m": 0.025,
            "neighbourhood_radius_cells": 3,
            "minimum_expected_points": 3.0,
            "minimum_deficit_fraction": 0.4,
            "minimum_component_cells": 2,
            "support_radius_m": 0.05,
            "source_collar_m": 0.08,
            "cleanmesh_collar_m": 0.35,
            "property_donor_distance_m": 0.06,
            "proposal_oversampling": 1.25,
            "minimum_radius_ratio": 0.8,
            "maximum_radius_ratio": 4.0,
            "maximum_proposals_per_target": 300000,
            "geometry_batch_points": 20000,
            "reference_energy_samples": 2048,
            "maximum_geometry_donors": 150000,
            "maximum_energy_donors": 96,
            "cleanmesh_base_voxel_m": 0.003,
        }

    def args(self):
        return SimpleNamespace(
            data_dir=self.data.resolve(),
            run_dir=self.run.resolve(),
            release_dir=self.release.resolve(),
            obstruction_manifest=None,
            water_2mm_base_manifest=None,
            water_5mm_base_manifest=None,
            water_2mm_geometry_manifest=None,
            water_5mm_geometry_manifest=None,
            water_2mm_final_manifest=None,
            water_5mm_final_manifest=None,
            terrain_manifest=None,
        )


class FullReleaseBuildTests(unittest.TestCase):
    @staticmethod
    def _tamper_fine_enriched_scalar(fixture, field, value):
        water = fixture.water["2mm"]
        candidate_path = water["enriched_candidate"]
        info = V11.inspect_ply(candidate_path)
        memory = np.memmap(
            info.path,
            dtype=info.dtype,
            mode="r+",
            offset=info.offset,
            shape=(info.count,),
        )
        memory[len(water["base_records"])][field] = value
        memory.flush()
        del memory
        manifest_path = water["enriched_manifest"]
        manifest = json.loads(manifest_path.read_text())
        manifest["candidate"]["sha256"] = V11.sha256_path(candidate_path)
        manifest["candidate"]["suffix_sha256"] = V11._payload_sha256(
            candidate_path,
            start=len(water["base_records"]),
            count=len(water["geometry_additions"]),
        )
        write_json(manifest_path, manifest)

    def test_build_consumes_all_stages_snapshots_and_verifies_without_installing(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = ReleaseFixture(directory)
            before = {
                name: V11.sha256_path(fixture.data / name)
                for name in V11.CANONICAL_NAMES
            }
            with mock.patch.object(V11, "app_running", return_value=False):
                report = V11.build(fixture.args())
            self.assertTrue(report["verified"])
            self.assertEqual(report["cloud_count"], 6)
            manifest = json.loads((fixture.release / "manifest.json").read_text())
            self.assertEqual(manifest["status"], "built")
            self.assertFalse(manifest["canonical_install_performed"])
            for name in V11.CANONICAL_NAMES:
                self.assertEqual(V11.sha256_path(fixture.data / name), before[name])
                self.assertTrue((fixture.release / "source-snapshots" / name).exists())

    def test_release_rejects_self_consistently_rehashed_nan_component_scalar(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = ReleaseFixture(directory)
            self._tamper_fine_enriched_scalar(
                fixture,
                "scalar_A_R_Roughness_Fine",
                np.nan,
            )
            with (
                mock.patch.object(V11, "app_running", return_value=False),
                self.assertRaisesRegex(
                    RuntimeError,
                    "incomplete or out-of-range component scalars",
                ),
            ):
                V11._assemble_stage_chain(fixture.args())

    def test_release_rejects_self_consistently_rehashed_out_of_range_scalar(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = ReleaseFixture(directory)
            self._tamper_fine_enriched_scalar(
                fixture,
                "scalar_A_R_Roughness_Combined",
                np.float32(1.25),
            )
            with (
                mock.patch.object(V11, "app_running", return_value=False),
                self.assertRaisesRegex(
                    RuntimeError,
                    "incomplete or out-of-range component scalars",
                ),
            ):
                V11._assemble_stage_chain(fixture.args())

    def test_release_rejects_manifest_that_omits_a_scalar_component(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = ReleaseFixture(directory)
            manifest_path = fixture.water["2mm"]["enriched_manifest"]
            manifest = json.loads(manifest_path.read_text())
            coverage = manifest["scalar_enrichment"][
                "component_field_finite_coverage"
            ]
            coverage["components"] = []
            coverage["component_count"] = 0
            coverage["all_components_all_required_fields_accepted"] = True
            write_json(manifest_path, manifest)
            with (
                mock.patch.object(V11, "app_running", return_value=False),
                self.assertRaisesRegex(
                    RuntimeError,
                    "component scalar audit differs from the candidate",
                ),
            ):
                V11._assemble_stage_chain(fixture.args())

    def test_install_and_restore_swap_all_six_and_update_release_state(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = ReleaseFixture(directory)
            args = fixture.args()
            with mock.patch.object(V11, "app_running", return_value=False):
                V11.build(args)
                built = json.loads((fixture.release / "manifest.json").read_text())
                V11.install(args)
                installed = json.loads((fixture.release / "manifest.json").read_text())
                self.assertEqual(installed["status"], "installed")
                for label, cloud in installed["clouds"].items():
                    self.assertEqual(
                        V11.sha256_path(fixture.data / cloud["canonical"]),
                        cloud["candidate"]["sha256"],
                        label,
                    )
                    self.assertFalse(Path(cloud["candidate"]["path"]).exists())
                    if label.startswith("WATER-"):
                        old01 = V11._water_old01_path(
                            fixture.data / cloud["canonical"]
                        )
                        self.assertEqual(
                            old01.exists(),
                            label == "WATER-5mm",
                        )
                V11.restore(args)
            restored = json.loads((fixture.release / "manifest.json").read_text())
            self.assertEqual(restored["status"], "restored")
            self.assertGreaterEqual(len(restored["transactions"]), 2)
            for label, cloud in built["clouds"].items():
                self.assertEqual(
                    V11.sha256_path(fixture.data / cloud["canonical"]),
                    cloud["source"]["sha256"],
                    label,
                )
                self.assertTrue(Path(cloud["candidate"]["path"]).exists())
                if label.startswith("WATER-"):
                    self.assertEqual(
                        V11._water_old01_path(
                            fixture.data / cloud["canonical"]
                        ).exists(),
                        label == "WATER-5mm",
                    )

    def test_install_preserves_an_existing_historic_water_old01(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = ReleaseFixture(directory)
            args = fixture.args()
            collision = fixture.historic_water_5mm_old01
            collision_hash = V11.sha256_path(collision)
            before = {
                name: V11.sha256_path(fixture.data / name)
                for name in V11.CANONICAL_NAMES
            }
            with mock.patch.object(V11, "app_running", return_value=False):
                V11.build(args)
                manifest = json.loads((fixture.release / "manifest.json").read_text())
                self.assertEqual(
                    manifest["protected_existing_water_old01"]["WATER-5mm"][
                        "sha256"
                    ],
                    V11.sha256_path(collision),
                )
                V11.install(args)
                for label, cloud in manifest["clouds"].items():
                    self.assertEqual(
                        V11.sha256_path(fixture.data / cloud["canonical"]),
                        cloud["candidate"]["sha256"],
                        label,
                    )
                V11.restore(args)
            for name, expected in before.items():
                self.assertEqual(V11.sha256_path(fixture.data / name), expected)
            self.assertEqual(V11.sha256_path(collision), collision_hash)
            self.assertFalse(
                (fixture.data / "Site1-WATER-2mm-old01.ply").exists()
            )

    def test_build_requires_exact_historic_water_old01_layout(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = ReleaseFixture(directory)
            fixture.historic_water_5mm_old01.unlink()
            with mock.patch.object(V11, "app_running", return_value=False):
                with self.assertRaisesRegex(RuntimeError, "historic.*5mm-old01"):
                    V11.build(fixture.args())
            self.assertFalse(fixture.release.exists())

        with tempfile.TemporaryDirectory() as directory:
            fixture = ReleaseFixture(directory)
            write_ply(
                fixture.data / "Site1-WATER-2mm-old01.ply",
                records(1, start=101.0),
            )
            with mock.patch.object(V11, "app_running", return_value=False):
                with self.assertRaisesRegex(RuntimeError, "unexpected.*2mm-old01"):
                    V11.build(fixture.args())
            self.assertFalse(fixture.release.exists())

    def test_restore_rejects_redirected_previous_paths(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = ReleaseFixture(directory)
            args = fixture.args()
            with mock.patch.object(V11, "app_running", return_value=False):
                V11.build(args)
                V11.install(args)
                manifest_path = fixture.release / "manifest.json"
                manifest = json.loads(manifest_path.read_text())
                install_row = next(
                    row for row in reversed(manifest["transactions"])
                    if row["action"] == "install"
                )
                install_row["previous"]["SAND-1mm"] = manifest["clouds"][
                    "SAND-1mm"
                ]["snapshot"]["path"]
                write_json(manifest_path, manifest)
                with self.assertRaisesRegex(
                    RuntimeError,
                    "install backup path is not the committed previous entry",
                ):
                    V11.restore(args)

    def test_installed_verify_requires_all_six_previous_files(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = ReleaseFixture(directory)
            args = fixture.args()
            with mock.patch.object(V11, "app_running", return_value=False):
                V11.build(args)
                V11.install(args)
                manifest = json.loads(
                    (fixture.release / "manifest.json").read_text()
                )
                install_row = next(
                    row for row in reversed(manifest["transactions"])
                    if row["action"] == "install"
                )
                Path(install_row["previous"]["ROCK-5mm"]).unlink()
                with self.assertRaisesRegex(
                    RuntimeError,
                    "installed previous generation",
                ):
                    V11.verify(args)

    def test_committed_recovery_requires_all_six_previous_files(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = ReleaseFixture(directory)
            args = fixture.args()
            with mock.patch.object(V11, "app_running", return_value=False):
                V11.build(args)
                V11.install(args)
                manifest = json.loads(
                    (fixture.release / "manifest.json").read_text()
                )
                install_row = next(
                    row for row in reversed(manifest["transactions"])
                    if row["action"] == "install"
                )
                journal_path = Path(install_row["journal"])
                journal = json.loads(journal_path.read_text())
                journal["state"] = "before-manifest-commit"
                write_json(journal_path, journal)
                Path(install_row["previous"]["SAND-5mm"]).unlink()
                with self.assertRaisesRegex(
                    RuntimeError,
                    "refusing ambiguous automatic recovery",
                ):
                    V11._recover_incomplete_transactions(args)

    def test_build_fails_closed_when_one_final_candidate_is_missing(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = ReleaseFixture(directory)
            (fixture.run / "water-final-5mm" / "Site1-WATER-5mm.candidate.ply").unlink()
            with mock.patch.object(V11, "app_running", return_value=False):
                with self.assertRaises(FileNotFoundError):
                    V11.build(fixture.args())
            self.assertFalse(fixture.release.exists())

    def test_build_rejects_cross_stage_review_config_drift(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = ReleaseFixture(directory, drift_water_config=True)
            with mock.patch.object(V11, "app_running", return_value=False):
                with self.assertRaisesRegex(
                    RuntimeError, "one review-config hash"
                ):
                    V11.build(fixture.args())

    def test_coarse_subset_recomputes_component_counts_and_support_distance(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = ReleaseFixture(directory)
            args = fixture.args()
            with mock.patch.object(V11, "app_running", return_value=False):
                clouds, _, _, _ = V11._assemble_stage_chain(args)
            fine = dict(clouds["WATER-2mm"])
            fine["candidate"] = fine["geometry_candidate"]
            coarse_path = fixture.water["5mm"]["geometry_manifest"]
            original = json.loads(coarse_path.read_text())
            supports = (
                fixture.water["5mm"]["base_candidate"],
                fixture.terrain_candidates["SAND-5mm"],
                fixture.terrain_candidates["ROCK-5mm"],
            )

            false_count = json.loads(json.dumps(original))
            false_count["cross_scale"]["accepted_hole_coverage"][0][
                "fine_count"
            ] = 1
            with self.assertRaisesRegex(RuntimeError, "fine count is false"):
                V11._verify_coarse_fine_subset(
                    false_count,
                    coarse_manifest_path=coarse_path,
                    coarse_archive_path=fixture.water["5mm"]["geometry_archive"],
                    fine_geometry=fine,
                    support_paths=supports,
                )

            duplicate = json.loads(json.dumps(original))
            duplicate["cross_scale"]["accepted_hole_coverage"].append(
                dict(duplicate["cross_scale"]["accepted_hole_coverage"][0])
            )
            with self.assertRaisesRegex(RuntimeError, "duplicated"):
                V11._verify_coarse_fine_subset(
                    duplicate,
                    coarse_manifest_path=coarse_path,
                    coarse_archive_path=fixture.water["5mm"]["geometry_archive"],
                    fine_geometry=fine,
                    support_paths=supports,
                )

            false_maximum = json.loads(json.dumps(original))
            false_maximum["cross_scale"][
                "maximum_fine_to_coarse_or_terrain_support_distance_m"
            ] = 0.0
            with self.assertRaisesRegex(
                RuntimeError, "differs from independent geometry"
            ):
                V11._verify_coarse_fine_subset(
                    false_maximum,
                    coarse_manifest_path=coarse_path,
                    coarse_archive_path=fixture.water["5mm"]["geometry_archive"],
                    fine_geometry=fine,
                    support_paths=supports,
                )

            archive_path = fixture.water["5mm"]["geometry_archive"]
            with np.load(archive_path, allow_pickle=False) as loaded:
                archive_arrays = {
                    name: np.asarray(loaded[name]).copy()
                    for name in loaded.files
                }
            damaged_z = {
                name: value.copy() for name, value in archive_arrays.items()
            }
            damaged_z["records"]["z"][0] += np.float32(0.001)
            np.savez_compressed(archive_path, **damaged_z)
            with self.assertRaisesRegex(RuntimeError, "5mm z is not an exact"):
                V11._verify_coarse_fine_subset(
                    original,
                    coarse_manifest_path=coarse_path,
                    coarse_archive_path=archive_path,
                    fine_geometry=fine,
                    support_paths=supports,
                )

            damaged_normal = {
                name: value.copy() for name, value in archive_arrays.items()
            }
            damaged_normal["records"]["nx"][0] += np.float32(0.001)
            np.savez_compressed(archive_path, **damaged_normal)
            with self.assertRaisesRegex(RuntimeError, "5mm nx is not an exact"):
                V11._verify_coarse_fine_subset(
                    original,
                    coarse_manifest_path=coarse_path,
                    coarse_archive_path=archive_path,
                    fine_geometry=fine,
                    support_paths=supports,
                )

    def test_terrain_cross_scale_rejects_a_self_consistently_rehashed_false_ledger(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = ReleaseFixture(directory)
            terrain_manifest_path = fixture.run / "terrain" / "manifest.json"
            terrain_manifest = json.loads(terrain_manifest_path.read_text())
            coarse = terrain_manifest["resolutions"]["5mm"]
            cross_path = Path(coarse["cross_scale_report_path"])
            cross = json.loads(cross_path.read_text())
            coverage_path = cross_path.parent / cross["roles"]["SAND"]["coverage_ledger"]
            with np.load(coverage_path, allow_pickle=False) as loaded:
                arrays = {name: np.asarray(loaded[name]).copy() for name in loaded.files}
            # The witness is the point itself, so its true distance is zero.
            # Rehash every enclosing manifest to prove the release gate does
            # not merely trust a self-consistent producer summary.
            arrays["coverage_xy_distance_m"][0] = 0.001
            np.savez_compressed(coverage_path, **arrays)
            cross["roles"]["SAND"]["coverage_ledger_sha256"] = V11.sha256_path(
                coverage_path
            )
            write_json(cross_path, cross)
            cross_hash = V11.sha256_path(cross_path)
            coarse_report_path = Path(coarse["report_path"])
            coarse_report = json.loads(coarse_report_path.read_text())
            coarse_report["cross_scale"]["report_sha256"] = cross_hash
            write_json(coarse_report_path, coarse_report)
            coarse["report_sha256"] = V11.sha256_path(coarse_report_path)
            coarse["cross_scale_report_sha256"] = cross_hash
            terrain_manifest["cross_scale"] = dict(cross)
            terrain_manifest["cross_scale"]["report"] = {
                "path": str(cross_path),
                "sha256": cross_hash,
            }
            write_json(terrain_manifest_path, terrain_manifest)
            with (
                mock.patch.object(V11, "app_running", return_value=False),
                self.assertRaisesRegex(
                    RuntimeError, "coverage ledger differs from independent geometry"
                ),
            ):
                V11._assemble_stage_chain(fixture.args())

    def test_release_manifest_binds_the_build_run_directory(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = ReleaseFixture(directory)
            args = fixture.args()
            with mock.patch.object(V11, "app_running", return_value=False):
                V11.build(args)
            different = fixture.args()
            different.run_dir = (fixture.root / "different-run").resolve()
            with self.assertRaisesRegex(RuntimeError, "run directory mismatch"):
                V11._read_release(different)

    def test_wrong_run_directory_cannot_recover_or_mutate_a_journal(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = ReleaseFixture(directory)
            args = fixture.args()
            with mock.patch.object(V11, "app_running", return_value=False):
                V11.build(args)
                V11.install(args)
            manifest = json.loads((fixture.release / "manifest.json").read_text())
            install_row = next(
                row
                for row in reversed(manifest["transactions"])
                if row["action"] == "install"
            )
            journal_path = Path(install_row["journal"])
            journal = json.loads(journal_path.read_text())
            journal["state"] = "before-manifest-commit"
            write_json(journal_path, journal)
            journal_before = journal_path.read_bytes()

            different = fixture.args()
            different.run_dir = (fixture.root / "different-run").resolve()
            with (
                mock.patch.object(V11, "app_running", return_value=False),
                self.assertRaisesRegex(RuntimeError, "run directory mismatch"),
            ):
                V11.verify(different)
            self.assertEqual(journal_path.read_bytes(), journal_before)
            self.assertEqual(
                json.loads((fixture.release / "manifest.json").read_text())[
                    "status"
                ],
                "installed",
            )

    def test_restore_remains_available_after_producer_input_drift(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = ReleaseFixture(directory)
            args = fixture.args()
            with mock.patch.object(V11, "app_running", return_value=False):
                V11.build(args)
                V11.install(args)
                fixture.config.write_text(
                    '{"schema_version": 1, "producer": "advanced"}\n'
                )
                with self.assertRaisesRegex(RuntimeError, "stage artifact"):
                    V11.verify(args)
                result = V11.restore(args)
            self.assertTrue(result["restored"])
            manifest = json.loads((fixture.release / "manifest.json").read_text())
            self.assertEqual(manifest["status"], "restored")
            for label, cloud in manifest["clouds"].items():
                self.assertEqual(
                    V11.sha256_path(fixture.data / cloud["canonical"]),
                    cloud["source"]["sha256"],
                    label,
                )

    def test_verify_rejects_hash_equal_snapshot_entry_replacement(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = ReleaseFixture(directory)
            args = fixture.args()
            with mock.patch.object(V11, "app_running", return_value=False):
                V11.build(args)
                manifest = json.loads((fixture.release / "manifest.json").read_text())
                snapshot = Path(manifest["clouds"]["SAND-1mm"]["snapshot"]["path"])
                replacement = snapshot.with_name(snapshot.name + ".replacement")
                shutil.copy2(snapshot, replacement)
                self.assertNotEqual(snapshot.stat().st_ino, replacement.stat().st_ino)
                os.replace(replacement, snapshot)
                with self.assertRaisesRegex(RuntimeError, "inode"):
                    V11.verify(args)

    def test_install_rejects_symlinked_candidate_even_when_target_is_exact(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = ReleaseFixture(directory)
            args = fixture.args()
            with mock.patch.object(V11, "app_running", return_value=False):
                V11.build(args)
                manifest = json.loads((fixture.release / "manifest.json").read_text())
                candidate = Path(manifest["clouds"]["SAND-1mm"]["candidate"]["path"])
                target = candidate.with_name(candidate.name + ".exact-target")
                candidate.rename(target)
                candidate.symlink_to(target)
                before = V11.sha256_path(fixture.data / "Site1-SAND-1mm.ply")
                with self.assertRaisesRegex(RuntimeError, "symbolic link"):
                    V11.install(args)
                self.assertEqual(
                    V11.sha256_path(fixture.data / "Site1-SAND-1mm.ply"),
                    before,
                )

    def test_restore_rejects_symlinked_committed_previous_backup(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = ReleaseFixture(directory)
            args = fixture.args()
            with mock.patch.object(V11, "app_running", return_value=False):
                V11.build(args)
                V11.install(args)
                manifest = json.loads((fixture.release / "manifest.json").read_text())
                install = next(
                    row for row in reversed(manifest["transactions"])
                    if row["action"] == "install"
                )
                previous = Path(install["previous"]["ROCK-1mm"])
                target = previous.with_name(previous.name + ".exact-target")
                previous.rename(target)
                previous.symlink_to(target)
                with self.assertRaisesRegex(RuntimeError, "backup path"):
                    V11.restore(args)

    def test_release_manifest_must_be_a_lexical_regular_file(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = ReleaseFixture(directory)
            args = fixture.args()
            with mock.patch.object(V11, "app_running", return_value=False):
                V11.build(args)
            manifest = fixture.release / "manifest.json"
            target = manifest.with_name("manifest.exact-target.json")
            manifest.rename(target)
            manifest.symlink_to(target)
            with self.assertRaisesRegex(RuntimeError, "symbolic link"):
                V11._read_release(args)


if __name__ == "__main__":
    unittest.main()
